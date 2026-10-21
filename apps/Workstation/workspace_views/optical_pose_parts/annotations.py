"""Frozen native preview and manual pixel-annotation widgets.

AnnotationImageView remains the authoritative point buffer. Native u/v pixels
and frame/hash/dimension identity are never rescaled into saved coordinates."""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtCore, QtGui, QtWidgets
from .layouts import load_component_form
from Workstation.ui.annotation_image_view import AnnotationImageView, AnnotationMode



class FrozenAnnotations(QtCore.QObject):
    command_requested = QtCore.pyqtSignal(str, object)
    annotation_changed = QtCore.pyqtSignal(str, object)
    suggestion_point_clicked = QtCore.pyqtSignal(str, str, int)
    acceptance_reset = QtCore.pyqtSignal()
    frame_identity_changed = QtCore.pyqtSignal()

    def __init__(self, parent=None, *, make_solver_button, collected_frame_ids):
        super().__init__(parent)
        if not callable(make_solver_button) or not callable(collected_frame_ids):
            raise TypeError("Frozen annotations require command-button and collected-frame ports")
        self._collected_frame_ids = collected_frame_ids
        self._auto_accept_in_progress = False
        self.preview_group = load_component_form(
            "optical_pose_annotations.ui", self, "annotations", make_solver_button
        )
        self.frozen_preview_layout = self.preview_group.findChild(QtWidgets.QBoxLayout, "frozen_preview_layout")
        self._manual_mode_tooltips = {
            AnnotationMode.HORIZON.value: (
                "Mark two points on the visible horizon in the frozen native image. "
                "The Horizon solver estimates Roll/Pitch only; Yaw and camera position "
                "remain at the prior pose."
            ),
            AnnotationMode.RIDGE.value: (
                "Mark corresponding Mountain features in the same order on the left and "
                "right frozen images. GNSS + Mountain needs at least four paired points "
                "to solve metric R,t."
            ),
            AnnotationMode.TARGET.value: (
                "Mark one Drone RTK target pixel in each frozen image. The single-target "
                "Drone RTK Yaw solver uses the current pair; once both sides are marked, "
                "the pair and capture-time Drone/P1/P2 GNSS are also accumulated for "
                "JSON-free multi-view refinement."
            ),
        }
        self.annotation_mode_combo.currentIndexChanged.connect(self._set_annotation_mode)
        self.annotation_clear_button.clicked.connect(self._clear_current_annotations)
        self.import_frozen_burst_button.clicked.connect(self._import_frozen_burst)
        for side, view in self.annotation_views.items():
            view.annotation_changed.connect(lambda payload, side=side: self._on_annotation_changed(side, payload))
            view.suggestion_point_right_clicked.connect(
                lambda mode, index, side=side: self._on_suggestion_point_right_clicked(side, mode, index)
            )
        self.destroyed.connect(self.preview_group.deleteLater)

    def annotation_payloads(self) -> dict[str, object]:
        """Return frame-pinned immutable annotation objects for both sides.

        ``FrozenAnnotationPayload`` is a frozen dataclass.  It intentionally
        remains an object at the UI/controller boundary instead of being
        flattened into mutable JSON here; the evidence owner decides when to
        serialize its fresh ``to_mapping()`` copy.
        """

        return {side: view.export_annotations() for side, view in self.annotation_views.items()}

    def _import_frozen_burst(self) -> None:
        left_path, _left_filter = QtWidgets.QFileDialog.getOpenFileName(
            self.preview_group,
            "Import left frozen frame image",
            "",
            "Images (*.png *.jpg *.jpeg *.tif *.tiff);;All files (*)",
        )
        if not left_path:
            return
        right_path, _right_filter = QtWidgets.QFileDialog.getOpenFileName(
            self.preview_group,
            "Import right frozen frame image",
            "",
            "Images (*.png *.jpg *.jpeg *.tif *.tiff);;All files (*)",
        )
        if not right_path:
            return
        sources: dict[str, dict[str, object]] = {
            "left": {"path": left_path},
            "right": {"path": right_path},
        }
        self.command_requested.emit(
            "alignment.pose.import_burst",
            {"sources": sources},
        )

    def set_frozen_preview_state(self, side: str, detail: object) -> None:
        if side not in self.frozen_preview_labels:
            raise ValueError(f"unsupported camera side: {side}")
        label = self.frozen_preview_labels[side]
        view = self.annotation_views[side]
        if not isinstance(detail, Mapping):
            text = str(detail or "NO FROZEN BURST")
            if not detail or "NO FROZEN" in text.upper() or "CLEARED" in text.upper():
                view.clear_frame()
            label.setText(f"{side.title()}: {text}")
            return

        if bool(detail.get("clear", False)):
            self._auto_accept_in_progress = True
            try:
                view.clear_frame()
            finally:
                self._auto_accept_in_progress = False
            self.acceptance_reset.emit()
            label.setText(f"{side.title()}: NO FROZEN BURST")
            return
        image = detail.get("image", detail.get("qimage"))
        if image is None:
            text = detail.get("detail", detail.get("state", "NO FROZEN BURST"))
            label.setText(f"{side.title()}: {text}")
            return
        if not isinstance(image, QtGui.QImage) or image.isNull():
            label.setText(f"{side.title()}: INVALID FROZEN IMAGE")
            return
        frame_id = str(detail.get("frame_id", "") or "").strip()
        if not frame_id:
            label.setText(f"{side.title()}: BLOCKED · frame_id missing")
            return
        metadata = detail.get("metadata")
        metadata = dict(metadata) if isinstance(metadata, Mapping) else {}
        metadata.setdefault("native_width", detail.get("width", image.width()))
        metadata.setdefault("native_height", detail.get("height", image.height()))
        metadata.setdefault("overlay_applied", detail.get("overlay_applied", False))
        previous_identity = (view.frame_id, view.image_sha256)
        # Once a target pair has been bound to capture-time RTK evidence, a
        # new Frozen burst starts with a clean target editor automatically.
        # An incomplete/unrecorded mark is retained and remains pinned to its
        # old frame identity so capture cannot silently discard operator work.
        annotation_payload = view.export_annotations()
        previous_annotation_frame_id = str(annotation_payload.frame_id) if annotation_payload is not None else ""
        previous_target_collected = previous_annotation_frame_id in self._collected_frame_ids(side)
        preserve_annotations = bool(view.annotation_points(AnnotationMode.TARGET) and not previous_target_collected)
        try:
            self._auto_accept_in_progress = True
            try:
                view.load_frozen_native_frame(
                    image,
                    frame_id=frame_id,
                    image_sha256=str(detail.get("image_sha256", detail.get("sha256", "")) or "") or None,
                    metadata=metadata,
                    preserve_annotations=preserve_annotations,
                )
            finally:
                self._auto_accept_in_progress = False
        except (TypeError, ValueError) as exc:
            label.setText(f"{side.title()}: BLOCKED · {exc}")
            return
        if previous_identity != (view.frame_id, view.image_sha256):
            self.acceptance_reset.emit()
            self.frame_identity_changed.emit()
        digest = str(view.image_sha256 or "")
        retained_note = " · previous target mark retained; Clear to edit" if view.annotations_stale else ""
        label.setText(
            f"{side.title()}: {frame_id} · {image.width()}×{image.height()} · sha256={digest[:12]}…{retained_note}"
        )

    def _set_mountain_auto_lines_hidden(self, hidden: bool) -> None:
        for view in self.annotation_views.values():
            view.set_suggestion_polyline_visible(not bool(hidden), mode=AnnotationMode.RIDGE)

    def _set_annotation_mode(self, _index: int = -1) -> None:
        mode = str(self.annotation_mode_combo.currentData() or AnnotationMode.HORIZON.value)
        for view in self.annotation_views.values():
            view.set_annotation_mode(mode)
        self.annotation_mode_combo.setToolTip(
            "Click the frozen native image to add points. " + self._manual_mode_tooltips.get(mode, "")
        )

    def _clear_current_annotations(self) -> None:
        mode = str(self.annotation_mode_combo.currentData() or AnnotationMode.HORIZON.value)
        for view in self.annotation_views.values():
            view.clear_annotations(mode)
        # A stale automatic suggestion for this mode would otherwise keep
        # reappearing in the snapshot after the manual points it produced
        # are gone. "ridge" annotations come from Mountain suggestions;
        # "target" has no automatic-suggestion counterpart, so this is a
        # harmless no-op for it server-side.
        suggestion_mode = "mountain" if mode == AnnotationMode.RIDGE.value else mode
        self.command_requested.emit(
            "alignment.pose.clear_auto_suggestions",
            {"mode": suggestion_mode},
        )

    def _on_annotation_changed(self, side, payload):
        self.annotation_changed.emit(side, payload)

    def _on_suggestion_point_right_clicked(self, side, mode, index):
        self.suggestion_point_clicked.emit(side, mode, index)
