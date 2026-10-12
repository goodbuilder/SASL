"""Automatic-point review owns proposals, paired exclusions and acceptance provenance.

The backend remains the approval owner. This view projects the existing core
point validators and edits only the explicit frozen annotation pair."""

from __future__ import annotations

import math
from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.annotation_image_view import AnnotationMode

from Workstation.workstation_core.optical_pose_auto_select import (
    validate_horizon_suggestion_points,
    validate_mountain_suggestion_points,
)
from Workstation.workstation_core.theme import ERROR_TEXT
from .annotations import FrozenAnnotations


class SuggestionReview(QtCore.QObject):
    command_requested = QtCore.pyqtSignal(str, object)
    information_changed = QtCore.pyqtSignal()

    def __init__(self, annotations: FrozenAnnotations, parent=None, *, make_solver_button):
        super().__init__(parent)
        if not isinstance(annotations, FrozenAnnotations) or not callable(make_solver_button):
            raise TypeError("Suggestion review requires the native annotation pair and command-button port")
        self.annotations = annotations
        self._auto_suggestions: dict[str, dict[str, object]] = {}
        self._active_auto_suggestion_id = ""
        self._auto_suggestion_excluded_indices: dict[str, set[int]] = {}
        self._visible_auto_suggestion_indices: dict[tuple[str, str], tuple[int, ...]] = {}
        self._accepted_annotation_snapshots: dict[str, dict[str, object]] = {}
        self._auto_suggestion_provenance = "NONE"
        self._auto_suggestion_error_active = False
        auto_group = QtWidgets.QGroupBox("Automatic point suggestion · review required")
        self.auto_group = auto_group
        auto_group.setObjectName("opticalPoseAutoSuggestionGroup")
        auto_layout = QtWidgets.QGridLayout(auto_group)
        auto_layout.setContentsMargins(6, 6, 6, 6)
        auto_layout.setSpacing(5)
        self.auto_select_buttons: dict[str, QtWidgets.QPushButton] = {}
        for column, (command_id, label) in enumerate(
            (
                ("alignment.pose.auto_select_horizon", "Auto-select Horizon"),
                ("alignment.pose.auto_select_mountain", "Auto-select Mountain"),
                ("alignment.pose.auto_select_cloud", "Auto-select Cloud"),
            )
        ):
            button = make_solver_button(command_id, label)
            if command_id.endswith("cloud"):
                button.setToolTip(
                    "Match far-field sky/cloud texture on the frozen native left/right frames with no "
                    "ridge bias. The points feed the rotation-only Cloud solve; translation always "
                    "comes from live GNSS. Both frames must be simultaneous."
                )
            else:
                button.setToolTip(
                    "Analyze the frozen native left/right frames and draw a separate suggestion overlay. "
                    "No Horizon/Mountain solver runs until the suggestion is explicitly accepted."
                )
            self.auto_select_buttons[command_id] = button
            auto_layout.addWidget(button, 0, column)
        self.use_suggestion_button = make_solver_button(
            "alignment.pose.use_suggestion",
            "Use suggestion",
        )
        self.use_suggestion_button.setEnabled(False)
        self.use_suggestion_button.setToolTip(
            "Copy the currently displayed suggestion into the manual annotation buffer. "
            "Horizon/Mountain detector quality is advisory, but current-frame native points must "
            "remain structurally usable. Horizon requires two separated endpoints per side; "
            "Mountain requires paired distributed points after exclusions."
        )
        auto_layout.addWidget(self.use_suggestion_button, 0, 3)
        self.auto_suggestion_status_label = QtWidgets.QLabel("No automatic suggestion")
        self.auto_suggestion_status_label.setObjectName("statusStrip")
        self.auto_suggestion_status_label.setWordWrap(True)
        auto_layout.addWidget(self.auto_suggestion_status_label, 1, 0, 1, 4)
        self.auto_suggestion_metrics_label = QtWidgets.QLabel("Confidence / inliers / residual / detector: --")
        self.auto_suggestion_metrics_label.setObjectName("compactContract")
        self.auto_suggestion_metrics_label.setWordWrap(True)
        auto_layout.addWidget(self.auto_suggestion_metrics_label, 2, 0, 1, 3)

        annotations.annotation_changed.connect(self._on_annotation_changed)
        annotations.suggestion_point_clicked.connect(self._on_suggestion_point_right_clicked)
        annotations.acceptance_reset.connect(self._reset_auto_acceptance)
        annotations.frame_identity_changed.connect(self._frozen_identity_changed)
        self.destroyed.connect(self.auto_group.deleteLater)

    @property
    def annotation_views(self):
        return self.annotations.annotation_views

    @property
    def annotation_mode_combo(self):
        return self.annotations.annotation_mode_combo

    @property
    def _auto_accept_in_progress(self):
        return self.annotations._auto_accept_in_progress

    @_auto_accept_in_progress.setter
    def _auto_accept_in_progress(self, value):
        self.annotations._auto_accept_in_progress = value

    def annotation_payloads(self):
        return self.annotations.annotation_payloads()

    def acceptance_payload(self):
        return {
            "suggestion_id": self._active_auto_suggestion_id,
            "provenance_state": self._auto_suggestion_provenance,
            "excluded_point_indices": sorted(
                self._auto_suggestion_excluded_indices.get(self._active_auto_suggestion_id, set())
            ),
        }

    def _frozen_identity_changed(self):
        if self._auto_suggestion_error_active:
            self._auto_suggestion_error_active = False
            self.auto_suggestion_status_label.setStyleSheet("")
            self.auto_suggestion_status_label.setText("No automatic suggestion")
            self.auto_suggestion_metrics_label.setText("Confidence / inliers / residual / detector: --")

    @staticmethod
    def _auto_points(value: object) -> tuple[tuple[float, float], ...]:
        raw = value if isinstance(value, (tuple, list)) else ()
        points: list[tuple[float, float]] = []
        for point in raw:
            if isinstance(point, Mapping):
                point = (point.get("u_px"), point.get("v_px"))
            if not isinstance(point, (tuple, list)) or len(point) != 2:
                continue
            try:
                u, v = float(point[0]), float(point[1])
            except (TypeError, ValueError):
                continue
            if math.isfinite(u) and math.isfinite(v):
                points.append((u, v))
        return tuple(points)

    @staticmethod
    def _auto_confidences(value: object) -> tuple[float, ...]:
        raw = value if isinstance(value, (tuple, list)) else ()
        confidences: list[float] = []
        for entry in raw:
            try:
                confidence = float(entry)
            except (TypeError, ValueError):
                return ()
            if not math.isfinite(confidence):
                return ()
            confidences.append(confidence)
        return tuple(confidences)

    def _clear_auto_overlays(self) -> None:
        for view in self.annotation_views.values():
            view.clear_suggestion_points()

    def _on_suggestion_point_right_clicked(
        self,
        side: str,
        mode: str,
        visible_index: int,
    ) -> None:
        suggestion = self._auto_suggestions.get(self._active_auto_suggestion_id)
        if not isinstance(suggestion, Mapping) or str(suggestion.get("mode") or "") not in {"mountain", "cloud"}:
            return
        if str(mode) != AnnotationMode.RIDGE.value:
            return
        visible_indices = self._visible_auto_suggestion_indices.get((side, str(mode)), ())
        if not 0 <= int(visible_index) < len(visible_indices):
            return
        original_index = visible_indices[int(visible_index)]
        excluded = self._auto_suggestion_excluded_indices.setdefault(self._active_auto_suggestion_id, set())
        excluded.add(original_index)
        self.set_auto_suggestions(list(self._auto_suggestions.values()))

    def _filtered_auto_points(
        self,
        suggestion_id: str,
        side: str,
        mode: AnnotationMode,
        row: Mapping[str, object],
    ) -> tuple[tuple[tuple[float, float], ...], tuple[float, ...]]:
        points = self._auto_points(row.get("points", row.get("annotation_points", ())))
        confidences = self._auto_confidences(row.get("point_confidences", ()))
        excluded = (
            self._auto_suggestion_excluded_indices.get(suggestion_id, set()) if mode == AnnotationMode.RIDGE else set()
        )
        visible_indices = tuple(index for index in range(len(points)) if index not in excluded)
        self._visible_auto_suggestion_indices[(side, mode.value)] = visible_indices
        filtered_points = tuple(points[index] for index in visible_indices)
        filtered_confidences = (
            tuple(confidences[index] for index in visible_indices) if len(confidences) == len(points) else ()
        )
        return filtered_points, filtered_confidences

    def _mountain_suggestion_copy_status(
        self,
        suggestion_id: str,
        sides: Mapping[str, object],
    ) -> tuple[bool, str]:
        """Project the shared owner validation as a button affordance."""

        raw_points: dict[str, object] = {}
        image_sizes: dict[str, tuple[int, int]] = {}
        for side in ("left", "right"):
            row = sides.get(side)
            if not isinstance(row, Mapping):
                return False, f"{side} suggestion points are missing"
            view = self.annotation_views[side]
            if str(row.get("frame_id") or "") != str(view.frame_id or "") or str(row.get("image_sha256") or "") != str(
                view.image_sha256 or ""
            ):
                return False, f"{side} frame identity changed"
            size = view.source_size
            if not size.isValid() or size.isEmpty():
                return False, f"{side} frozen native frame is unavailable"
            image_sizes[side] = (size.width(), size.height())
            try:
                declared_size = (
                    int(row.get("source_width_px", -1)),
                    int(row.get("source_height_px", -1)),
                )
            except (TypeError, ValueError, OverflowError):
                return False, f"{side} native image dimensions are invalid"
            if declared_size != image_sizes[side]:
                return False, f"{side} native image dimensions changed"
            raw_points[side] = row.get("points", row.get("ridge_polyline", ()))
        try:
            validate_mountain_suggestion_points(
                raw_points["left"],
                raw_points["right"],
                excluded_indices=sorted(self._auto_suggestion_excluded_indices.get(suggestion_id, set())),
                left_image_size=image_sizes["left"],
                right_image_size=image_sizes["right"],
            )
        except (TypeError, ValueError) as exc:
            return False, str(exc)
        return True, ""

    def _horizon_suggestion_copy_status(
        self,
        sides: Mapping[str, object],
    ) -> tuple[bool, str]:
        raw_points: dict[str, object] = {}
        image_sizes: dict[str, tuple[int, int]] = {}
        for side in ("left", "right"):
            row = sides.get(side)
            if not isinstance(row, Mapping):
                return False, f"{side} Horizon endpoints are missing"
            view = self.annotation_views[side]
            if str(row.get("frame_id") or "") != str(view.frame_id or "") or str(row.get("image_sha256") or "") != str(
                view.image_sha256 or ""
            ):
                return False, f"{side} frame identity changed"
            size = view.source_size
            if not size.isValid() or size.isEmpty():
                return False, f"{side} frozen native frame is unavailable"
            image_sizes[side] = (size.width(), size.height())
            try:
                declared_size = (
                    int(row.get("source_width_px", -1)),
                    int(row.get("source_height_px", -1)),
                )
            except (TypeError, ValueError, OverflowError):
                return False, f"{side} native image dimensions are invalid"
            if declared_size != image_sizes[side]:
                return False, f"{side} native image dimensions changed"
            raw_points[side] = row.get("points", row.get("horizon_line", ()))
        try:
            validate_horizon_suggestion_points(
                raw_points["left"],
                raw_points["right"],
                left_image_size=image_sizes["left"],
                right_image_size=image_sizes["right"],
            )
        except (TypeError, ValueError) as exc:
            return False, str(exc)
        return True, ""

    def set_auto_suggestion_busy(self, busy: bool, mode: str = "") -> None:
        """Show an in-flight selector and prevent duplicate CPU-heavy runs."""

        busy = bool(busy)
        for button in self.auto_select_buttons.values():
            button.setEnabled(not busy)
        if not busy:
            return
        mode_label = {"horizon": "Horizon", "cloud": "Cloud"}.get(str(mode).lower(), "Mountain")
        self._auto_suggestion_error_active = False
        self.auto_suggestion_status_label.setStyleSheet("")
        self.auto_suggestion_status_label.setText(f"{mode_label} · RUNNING · analyzing frozen frames in background")
        self.auto_suggestion_metrics_label.setText("Confidence / inliers / residual / detector: calculating…")

    def _reset_auto_acceptance(self) -> None:
        """Invalidate accepted-point provenance when the frozen frame changes."""

        self._accepted_annotation_snapshots = {}
        self._accepted_suggestion_id = ""
        self._auto_suggestion_provenance = "NONE"

    def set_auto_suggestion_error(self, command_id: str, message: str) -> None:
        """Show why Auto-select produced nothing at all.

        Auto-select can raise before a suggestion is ever recorded (no
        frozen burst yet, missing K,D, native-raster mismatch, etc.), which
        otherwise leaves the status/metrics labels silently stuck on their
        placeholder text with only a status-bar toast as feedback. This
        keeps the reason visible next to the buttons until the next attempt.
        """

        self._auto_suggestion_error_active = True
        self.use_suggestion_button.setEnabled(False)
        self.auto_suggestion_status_label.setText(f"{command_id}: failed — {message}")
        self.auto_suggestion_status_label.setStyleSheet(f"color: {ERROR_TEXT};")
        self.auto_suggestion_metrics_label.setText("Confidence / inliers / residual / detector: --")

    def set_auto_suggestions(self, suggestions: object) -> None:
        """Render the latest owner proposal without touching manual points."""

        rows = (
            [dict(row) for row in suggestions if isinstance(row, Mapping)]
            if isinstance(suggestions, (tuple, list))
            else []
        )
        self._auto_suggestions = {
            str(row.get("suggestion_id") or f"suggestion-{index}"): row for index, row in enumerate(rows)
        }
        self._clear_auto_overlays()
        if not rows:
            if self._auto_suggestion_error_active:
                # A failed attempt is still on screen and periodic snapshot
                # polling keeps seeing the same empty list (the failure was
                # never recorded as a suggestion) -- leave the reason up
                # rather than silently reverting to the placeholder text.
                return
            self.auto_suggestion_status_label.setStyleSheet("")
            self._active_auto_suggestion_id = ""
            self._auto_suggestion_provenance = "NONE"
            self.use_suggestion_button.setEnabled(False)
            self.auto_suggestion_status_label.setText("No automatic suggestion")
            self.auto_suggestion_metrics_label.setText("Confidence / inliers / residual / detector: --")
            return
        self._auto_suggestion_error_active = False
        self.auto_suggestion_status_label.setStyleSheet("")
        suggestion = rows[-1]
        suggestion_id = str(suggestion.get("suggestion_id") or "")
        self._active_auto_suggestion_id = suggestion_id
        mode = str(suggestion.get("mode") or "").lower()
        annotation_mode = (
            AnnotationMode.HORIZON
            if mode == "horizon"
            else AnnotationMode.RIDGE
            if mode in {"mountain", "cloud"}
            else None
        )
        sides = suggestion.get("sides")
        sides = sides if isinstance(sides, Mapping) else {}
        if annotation_mode is not None:
            for side, view in self.annotation_views.items():
                row = sides.get(side)
                if not isinstance(row, Mapping):
                    continue
                points, confidences = self._filtered_auto_points(suggestion_id, side, annotation_mode, row)
                try:
                    view.set_suggestion_points(points, mode=annotation_mode, confidences=confidences)
                except (RuntimeError, ValueError):
                    # A stale frame identity is shown in the status strip; it
                    # must not affect committed native annotations.
                    continue
        state = str(suggestion.get("state", "NO_DATA") or "NO_DATA").upper()
        accepted_id = str(getattr(self, "_accepted_suggestion_id", "") or "")
        if suggestion_id == accepted_id and self._auto_suggestion_provenance == "MANUAL_EDITED":
            provenance = "MANUAL_EDITED"
        elif suggestion_id == accepted_id:
            provenance = "AUTO_ACCEPTED"
        else:
            provenance = str(suggestion.get("provenance_state", "AUTO_PROPOSED") or "AUTO_PROPOSED")
        self._auto_suggestion_provenance = provenance
        confidence = suggestion.get("confidence")
        inliers = suggestion.get("inlier_count", suggestion.get("match_count", "--"))
        residual = suggestion.get("residual_px")
        detector = suggestion.get("detection_method", "--")
        try:
            confidence_text = f"{float(confidence):.2f}"
        except (TypeError, ValueError):
            confidence_text = "--"
        try:
            residual_text = f"{float(residual):.2f}px"
        except (TypeError, ValueError):
            residual_text = "--"
        excluded_count = len(self._auto_suggestion_excluded_indices.get(suggestion_id, set()))
        excluded_text = f" · excluded pairs {excluded_count}" if mode in {"mountain", "cloud"} else ""
        self.auto_suggestion_metrics_label.setText(
            f"Confidence {confidence_text} · inliers {inliers} · residual {residual_text} "
            f"· detector {detector}{excluded_text}"
        )
        reason = str(suggestion.get("failure_reason") or suggestion.get("reason") or "")
        suffix = f" · {reason}" if reason else ""
        copy_eligible = False
        copy_status = ""
        if mode in {"mountain", "cloud"}:
            copy_eligible, copy_reason = self._mountain_suggestion_copy_status(suggestion_id, sides)
        elif mode == "horizon":
            copy_eligible, copy_reason = self._horizon_suggestion_copy_status(sides)
        else:
            copy_reason = "unsupported automatic suggestion mode"
        if mode in {"horizon", "mountain", "cloud"}:
            copy_status = " · COPY READY" if copy_eligible else f" · COPY BLOCKED: {copy_reason}"
        self.auto_suggestion_status_label.setText(
            f"{mode.title() or 'Automatic'} · {state} · {provenance}{copy_status} · {suggestion_id[:18]}{suffix}"
        )
        self.use_suggestion_button.setEnabled(
            bool(suggestion_id)
            and copy_eligible
            and provenance != "AUTO_ACCEPTED"
            and self._auto_suggestion_provenance != "MANUAL_EDITED"
        )

    def _apply_accepted_auto_suggestion(self, suggestion: Mapping[str, object]) -> None:
        """Copy approved points into the existing annotation buffer exactly once."""

        suggestion_id = str(suggestion.get("suggestion_id") or "")
        if not suggestion_id or suggestion_id == getattr(self, "_accepted_suggestion_id", ""):
            return
        mode = str(suggestion.get("mode") or "").lower()
        annotation_mode = (
            AnnotationMode.HORIZON
            if mode == "horizon"
            else AnnotationMode.RIDGE
            if mode in {"mountain", "cloud"}
            else None
        )
        sides = suggestion.get("sides")
        sides = sides if isinstance(sides, Mapping) else {}
        if annotation_mode is None:
            return
        committed: dict[str, dict[str, object]] = {}
        self._auto_accept_in_progress = True
        try:
            for side, view in self.annotation_views.items():
                row = sides.get(side)
                if not isinstance(row, Mapping):
                    continue
                expected_frame = str(row.get("frame_id") or "")
                expected_sha = str(row.get("image_sha256") or "")
                if view.frame_id != expected_frame or view.image_sha256 != expected_sha:
                    self.auto_suggestion_status_label.setText(
                        f"{mode.title()} · AUTO_ACCEPTED blocked · {side} frame identity changed"
                    )
                    return
                points = self._auto_points(row.get("points", row.get("annotation_points", ())))
                if not points:
                    continue
                view.replace_source_points(points, mode=annotation_mode, emit=False)
                payload = view.export_annotations()
                if payload is not None:
                    committed[side] = payload.to_mapping()
        finally:
            self._auto_accept_in_progress = False
        self._accepted_suggestion_id = suggestion_id
        self._accepted_annotation_snapshots = committed
        self._auto_suggestion_provenance = "AUTO_ACCEPTED"
        quality_state = str(suggestion.get("state") or "UNKNOWN").upper()
        self.auto_suggestion_status_label.setText(
            f"{mode.title()} · {quality_state} · AUTO_ACCEPTED · points copied to manual annotation"
        )
        self.use_suggestion_button.setEnabled(False)

    def _on_annotation_changed(self, side: str, payload: object) -> None:
        if self._auto_accept_in_progress:
            return
        current = payload.to_mapping() if hasattr(payload, "to_mapping") else None
        if current is None and not any(value is not None for value in self.annotation_payloads().values()):
            return
        expected = self._accepted_annotation_snapshots.get(side)
        if self._accepted_annotation_snapshots and expected is not None and current != expected:
            self._auto_suggestion_provenance = "MANUAL_EDITED"
            self.auto_suggestion_status_label.setText(
                "Automatic suggestion · MANUAL_EDITED · operator annotation differs from AUTO_ACCEPTED points"
            )
            self.use_suggestion_button.setEnabled(False)
        provenance = self._auto_suggestion_provenance if self._accepted_annotation_snapshots else "MANUAL"
        self.command_requested.emit(
            "alignment.pose.annotation_changed",
            {
                "command_id": "alignment.pose.annotation_changed",
                "suggestion_id": str(getattr(self, "_accepted_suggestion_id", "") or ""),
                "provenance_state": provenance,
                "annotation_mode": str(self.annotation_mode_combo.currentData() or AnnotationMode.HORIZON.value),
                "annotations": self.annotation_payloads(),
                "annotation_contract": (
                    "frozen native OpenCV pixels: u right, v down; identity is "
                    "frame_id + image_sha256 + native dimensions"
                ),
            },
        )
        self.information_changed.emit()
