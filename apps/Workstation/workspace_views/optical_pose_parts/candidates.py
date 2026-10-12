"""Candidate comparison, selection, provenance details and per-source DOF choices.

This component formats owner evidence and emits commands; it never applies a
pose or computes an RDF/NED transform. Candidate buffers have one owner."""

from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path
from typing import Sequence
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import EngineeringValue


class CandidateReview(QtCore.QObject):
    command_requested = QtCore.pyqtSignal(str, object)
    information_changed = QtCore.pyqtSignal()
    CANDIDATE_HEADERS = (
        "Candidate",
        "Method",
        "R source",
        "t source",
        "K,D / zoom",
        "Residual",
        "Observable",
        "State",
    )
    _CANDIDATE_DETAIL_INPUT_HASH_KEYS = (
        "lever_arm_status",
        "left_anchor_pose_sha256",
        "horizon_model",
        "camera_height_above_reference_surface_m",
        "pose_component_sources.relative_rotation",
        "pose_component_sources.relative_translation",
        "pose_component_sources.left_rotation",
        "pose_component_sources.left_position",
    )
    _CANDIDATE_DETAIL_QUALITY_KEYS = (
        "sampson_rms_normalized",
        "sampson_p95_normalized",
        "sampson_rms_px_equivalent",
        "inlier_count",
        "outlier_count",
        "inlier_ratio",
        "gnss_baseline_m",
        "left_rms_normalized",
        "right_rms_normalized",
        "left_roll_sensitivity_1px_deg",
        "left_pitch_sensitivity_1px_deg",
        "right_roll_sensitivity_1px_deg",
        "right_pitch_sensitivity_1px_deg",
    )

    def __init__(self, parent=None, *, make_solver_button):
        super().__init__(parent)
        if not callable(make_solver_button):
            raise TypeError("Candidate review requires a command-button port")
        self._make_solver_button = make_solver_button
        self._active_profile_id = ""
        self._candidate_rows = {}
        candidate_group = QtWidgets.QGroupBox("R,t candidates · compare")
        self.candidate_group = candidate_group
        candidate_group.setObjectName("opticalPoseCandidatesGroup")
        candidate_layout = QtWidgets.QVBoxLayout(candidate_group)
        self.candidate_table = QtWidgets.QTableWidget(0, len(self.CANDIDATE_HEADERS))
        self.candidate_table.setObjectName("opticalPoseCandidateTable")
        self.candidate_table.setHorizontalHeaderLabels(list(self.CANDIDATE_HEADERS))
        self.candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.candidate_table.setAlternatingRowColors(True)
        self.candidate_table.verticalHeader().setVisible(False)
        self.candidate_table.horizontalHeader().setStretchLastSection(True)
        self.candidate_table.setMinimumHeight(145)
        self.candidate_table.itemSelectionChanged.connect(self._refresh_candidate_actions)
        candidate_layout.addWidget(self.candidate_table)
        self.candidate_detail_text = QtWidgets.QPlainTextEdit()
        self.candidate_detail_text.setObjectName("opticalPoseCandidateDetail")
        self.candidate_detail_text.setReadOnly(True)
        self.candidate_detail_text.setMaximumHeight(110)
        self.candidate_detail_text.setPlaceholderText(
            "Select a candidate to review its provenance: rotation/translation/lever-arm "
            "source, GNSS fix quality, Sampson/rms residuals, inliers, apply eligibility, "
            "and warnings."
        )
        candidate_layout.addWidget(self.candidate_detail_text)
        self.reference_value = EngineeringValue("Existing calibration R,t")
        candidate_layout.addWidget(self.reference_value)

        candidate_actions = QtWidgets.QHBoxLayout()
        self.apply_button = QtWidgets.QPushButton("Apply candidate")
        self.apply_button.setProperty("commandId", "alignment.pose.apply_candidate")
        self.apply_button.setEnabled(False)
        self.apply_button.setToolTip(
            "Apply every DOF the selected candidate optimized and observed on both camera sides. "
            "Use Combine when axes must come from different candidates."
        )
        self.apply_button.clicked.connect(self._request_apply)
        self.revert_button = QtWidgets.QPushButton("Revert active pose")
        self.revert_button.setProperty("commandId", "alignment.pose.revert")
        self.revert_button.setEnabled(False)
        self.revert_button.clicked.connect(self._request_revert)
        self.reset_button = QtWidgets.QPushButton("Reset Alignment")
        self.reset_button.setProperty("commandId", "alignment.pose.reset")
        self.reset_button.setToolTip(
            "Clear candidates, frozen frames, annotations, observations, and the active pose. "
            "A reset event is appended; stored data is not deleted."
        )
        self.reset_button.clicked.connect(self._request_reset)
        self.candidate_status_label = QtWidgets.QLabel("No pose candidate")
        self.candidate_status_label.setObjectName("statusStrip")
        candidate_actions.addWidget(self.apply_button)
        candidate_actions.addWidget(self.revert_button)
        candidate_actions.addWidget(self.reset_button)
        candidate_actions.addWidget(self.candidate_status_label, 1)
        candidate_layout.addLayout(candidate_actions)
        self.destroyed.connect(self.candidate_group.deleteLater)

    def create_combine_button(self):
        make_solver_button = self._make_solver_button
        self.combine_button = make_solver_button(
            "alignment.pose.solve_combined",
            "Build combined candidate",
        )
        self.combine_button.setEnabled(False)
        self.combine_button.setToolTip(
            "Combine explicitly checked candidates only when they share the same prior and exact "
            "operating K,D profile and optimize non-overlapping DOF. This creates a new candidate; "
            "it does not apply the pose automatically."
        )
        return self.combine_button

    def populate_combine_page(self, page_layout: QtWidgets.QVBoxLayout) -> None:
        selection_note = QtWidgets.QLabel(
            "Check at least two immutable candidates. A typical use is Horizon R/P + Drone RTK Yaw: "
            "the first contributes Roll/Pitch and the second contributes Yaw. Candidates with "
            "overlapping DOF, different explicit priors, or different exact K,D are rejected by the owner."
        )
        selection_note.setObjectName("compactContract")
        selection_note.setWordWrap(True)
        page_layout.addWidget(selection_note)
        self.combine_candidate_list = QtWidgets.QListWidget()
        self.combine_candidate_list.setObjectName("opticalPoseCombineCandidateList")
        self.combine_candidate_list.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.combine_candidate_list.setMinimumHeight(82)
        self.combine_candidate_list.setMaximumHeight(160)
        self.combine_candidate_list.setToolTip(
            "Explicit source candidates for the combined pose. Check each source; the owner "
            "retains their IDs, hashes, shared prior, and quality evidence."
        )
        self.combine_candidate_list.itemChanged.connect(self._on_combine_candidates_changed)
        page_layout.addWidget(self.combine_candidate_list)

        dof_note = QtWidgets.QLabel(
            "Per checked candidate, which axes it contributes to the combined pose. Defaults to "
            "every axis that candidate actually observed; uncheck an axis to exclude it (e.g. take "
            "only Checkerboard's translation while Horizon supplies Roll/Pitch). An axis checked by "
            "more than one candidate blocks the build below until resolved."
        )
        dof_note.setObjectName("compactContract")
        dof_note.setWordWrap(True)
        page_layout.addWidget(dof_note)
        self.combine_dof_table = QtWidgets.QTableWidget(0, 7)
        self.combine_dof_table.setObjectName("opticalPoseCombineDofTable")
        self.combine_dof_table.setHorizontalHeaderLabels(["Candidate", "Roll", "Pitch", "Yaw", "N", "E", "D"])
        self.combine_dof_table.verticalHeader().setVisible(False)
        self.combine_dof_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        for column in range(1, 7):
            self.combine_dof_table.horizontalHeader().setSectionResizeMode(
                column, QtWidgets.QHeaderView.ResizeToContents
            )
        self.combine_dof_table.setMinimumHeight(82)
        self.combine_dof_table.setMaximumHeight(160)
        self.combine_dof_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.combine_dof_table.itemChanged.connect(self._refresh_combine_action)
        page_layout.addWidget(self.combine_dof_table)

        self.combine_candidate_status = QtWidgets.QLabel("No candidates checked · select at least two")
        self.combine_candidate_status.setObjectName("statusStrip")
        self.combine_candidate_status.setWordWrap(True)
        page_layout.addWidget(self.combine_candidate_status)

    @staticmethod
    def _candidate_text(value: object, *, digits: int = 3) -> str:
        if value in (None, ""):
            return "--"
        if isinstance(value, (tuple, list)):
            parts = []
            for item in value:
                try:
                    parts.append(f"{float(item):+.{digits}f}")
                except (TypeError, ValueError):
                    parts.append(str(item))
            return "/".join(parts)
        try:
            return f"{float(value):.{digits}f}"
        except (TypeError, ValueError):
            return str(value)

    @staticmethod
    def _candidate_observability(candidate: Mapping[str, object]) -> str:
        by_side = candidate.get("observable_dof_by_side")
        if isinstance(by_side, Mapping):
            parts = []
            for side, label in (("left", "L"), ("right", "R")):
                values = by_side.get(side)
                values = values if isinstance(values, Mapping) else {}
                enabled = [str(key) for key, value in values.items() if bool(value)]
                parts.append(f"{label}[{','.join(enabled) or 'none'}]")
            return " ".join(parts)
        raw = candidate.get("observable_dof", candidate.get("observability"))
        if isinstance(raw, Mapping):
            enabled = [str(key) for key, value in raw.items() if bool(value)]
            return ",".join(enabled) or "none"
        if isinstance(raw, (tuple, list, set)):
            return ",".join(str(value) for value in raw) or "none"
        return str(raw or "--")

    @staticmethod
    def _candidate_owned_dofs(candidate: Mapping[str, object]) -> tuple[str, ...]:
        """Read-only owner mask shown to the operator and sent implicitly on Apply."""

        optimized = candidate.get("optimized_dof")
        observable = candidate.get("observable_dof")
        source = optimized if isinstance(optimized, Mapping) else observable
        if not isinstance(source, Mapping):
            return ()
        return tuple(str(name) for name, enabled in source.items() if bool(enabled))

    def selected_candidate_id(self) -> str:
        row = self.candidate_table.currentRow()
        if row < 0:
            selected = self.candidate_table.selectionModel().selectedRows()
            row = selected[0].row() if selected else -1
        item = self.candidate_table.item(row, 0) if row >= 0 else None
        return "" if item is None else str(item.data(QtCore.Qt.UserRole) or "")

    def apply_payload(self) -> dict[str, object] | None:
        candidate_id = self.selected_candidate_id()
        if not candidate_id:
            return None
        return {"candidate_id": candidate_id}

    def revert_payload(self) -> dict[str, object]:
        return {
            "active_profile_id": self._active_profile_id,
            "restore": "previous_immutable_profile",
        }

    def set_active_profile(self, profile_id: object) -> None:
        self._active_profile_id = str(profile_id or "")
        self.revert_button.setEnabled(bool(self._active_profile_id))

    def set_reference_profile(self, calibration: Mapping[str, object]) -> None:
        """Show the existing checkerboard R,t as a comparison reference."""

        extrinsics = calibration.get("extrinsics")
        extrinsics = extrinsics if isinstance(extrinsics, Mapping) else {}
        state = str(extrinsics.get("state", "NO DATA") or "NO DATA")
        baseline = extrinsics.get("baseline_m")
        profile_path = str(calibration.get("extrinsics_profile_path", calibration.get("profile_path", "")) or "")
        baseline_text = self._candidate_text(baseline, digits=4)
        profile_text = Path(profile_path).name if profile_path else "profile path unavailable"
        self.reference_value.set_value(
            f"{state} · baseline={baseline_text} m · {profile_text}",
            source=str(calibration.get("source", "Calibration owner snapshot") or ""),
            frame=str(extrinsics.get("frame_contract", "OpenCV optical RDF") or ""),
            valid=True if state in {"READY", "VALID", "LOADED", "ACTIVE"} else None,
        )
        self.information_changed.emit()

    @staticmethod
    def _candidate_rt_sources(candidate: Mapping[str, object]) -> tuple[str, str, str]:
        method = str(candidate.get("method", candidate.get("solver", "")) or "").lower()
        if "mountain" in method or "terrain" in method:
            rotation = "paired Mountain u,v"
            translation = "GNSS baseline" if "gnss" in method else "terrain/PnP"
        elif "horizon" in method:
            rotation = "horizon R/P"
            translation = "prior pose"
        elif "drone" in method:
            rotation = "Drone RTK yaw"
            translation = "prior pose"
        elif "combined" in method:
            rotation = "source candidates"
            translation = "source candidates"
        else:
            rotation = "candidate evidence"
            translation = "candidate evidence"
        k_source = "exact K,D" if candidate.get("input_hashes") else "K,D unavailable"
        return rotation, translation, k_source

    def set_candidates(self, candidates: object) -> None:
        """Replace displayed rows while preserving selection by candidate ID."""

        selected_id = self.selected_candidate_id()
        selected_combine_ids = set(self.combine_candidate_ids())
        rows = (
            [dict(value) for value in candidates if isinstance(value, Mapping)]
            if isinstance(candidates, (tuple, list))
            else []
        )
        self._candidate_rows = {}
        self.candidate_table.setRowCount(len(rows))
        selected_row = -1
        for row_index, candidate in enumerate(rows):
            candidate_id = str(candidate.get("candidate_id", candidate.get("id", "")) or "")
            if not candidate_id:
                candidate_id = f"candidate-{row_index + 1:03d}"
            self._candidate_rows[candidate_id] = candidate
            rotation_source, translation_source, intrinsics_source = self._candidate_rt_sources(candidate)
            values = (
                candidate_id,
                candidate.get("method", candidate.get("solver", "--")),
                rotation_source,
                translation_source,
                intrinsics_source,
                self._candidate_text(candidate.get("residual", candidate.get("residual_px"))),
                self._candidate_observability(candidate),
                candidate.get("state", candidate.get("quality", "--")),
            )
            applicable = bool(candidate.get("applicable", True))
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(value))
                item.setData(QtCore.Qt.UserRole, candidate_id)
                item.setData(QtCore.Qt.UserRole + 1, applicable)
                self.candidate_table.setItem(row_index, column, item)
            if candidate_id == selected_id:
                selected_row = row_index
        if selected_row < 0 and rows:
            selected_row = 0
        if selected_row >= 0:
            self.candidate_table.selectRow(selected_row)
            self.candidate_table.setCurrentCell(selected_row, 0)
        else:
            self.candidate_table.clearSelection()
        self._sync_combine_candidate_list(rows, selected_combine_ids)
        self._refresh_candidate_actions()
        self.information_changed.emit()

    def _selected_candidate_applicable(self) -> bool:
        row = self.candidate_table.currentRow()
        item = self.candidate_table.item(row, 0) if row >= 0 else None
        return bool(item is not None and item.data(QtCore.Qt.UserRole + 1))

    @staticmethod
    def _candidate_relative_pose_text(candidate: Mapping[str, object]) -> str:
        """Render the owner-projected Mountain relative OpenCV R/t readout."""

        if str(candidate.get("method", "") or "") not in {"gnss-constrained-mountain-uv", "cloud-far-field-rotation"}:
            return ""
        relative = candidate.get("relative_pose_rdf")
        if not isinstance(relative, Mapping):
            return ""
        raw_rotation = relative.get("rotation_right_from_left_rdf")
        raw_translation = relative.get("translation_right_from_left_rdf_m")
        raw_origin = relative.get("right_origin_in_left_rdf_m")
        if not isinstance(raw_rotation, (tuple, list)) or len(raw_rotation) != 3:
            return ""
        if not isinstance(raw_translation, (tuple, list)) or len(raw_translation) != 3:
            return ""
        try:
            rotation = []
            for row in raw_rotation:
                if not isinstance(row, (tuple, list)) or len(row) != 3:
                    return ""
                values = tuple(float(value) for value in row)
                if not all(math.isfinite(value) for value in values):
                    return ""
                rotation.append(values)
            translation = tuple(float(value) for value in raw_translation)
            if not all(math.isfinite(value) for value in translation):
                return ""
            origin = None
            if isinstance(raw_origin, (tuple, list)) and len(raw_origin) == 3:
                parsed_origin = tuple(float(value) for value in raw_origin)
                if all(math.isfinite(value) for value in parsed_origin):
                    origin = parsed_origin
        except (TypeError, ValueError, OverflowError):
            return ""

        lines = [
            "relative_frame: OpenCV optical RDF",
            "equation: X_right = R_right_from_left_rdf * X_left + t_right_from_left_rdf_m",
            "R_right_from_left_rdf:",
        ]
        lines.extend("  [" + ", ".join(f"{value:+.9f}" for value in row) + "]" for row in rotation)
        lines.append("t_right_from_left_rdf_m: [" + ", ".join(f"{value:+.9f}" for value in translation) + "]")
        if origin is not None:
            lines.append("right_origin_in_left_rdf_m: [" + ", ".join(f"{value:+.9f}" for value in origin) + "]")
        return "\n".join(lines)

    @classmethod
    def _candidate_detail_text(cls, candidate: Mapping[str, object]) -> str:
        """Render provenance/quality/warnings for the selected candidate.

        Purely additive review text: it only formats fields the owner
        snapshot already exposes (``input_hashes``/``quality_metrics``/
        ``warnings``) so a reviewer can see each value's source without a
        dedicated widget per method.
        """

        lines: list[str] = []
        reference = candidate.get("yaw90_reference")
        if isinstance(reference, Mapping):
            lines.append("기준: GNSS Yaw90 · 촬영 요청 시 고정 · P1 R/P 정렬 가정")
            lines.append(f"요청 UTC ns={reference.get('requested_utc_ns')} · Reverse={reference.get('reverse')}")
            for side, role in (("left", "기준 자세 (가정)"), ("right", "영상 상대 R로 유도한 자세")):
                pose = candidate.get(f"{side}_pose", {})
                lines.append(
                    f"{side} {role}: R/P/Y={pose.get('virtual_frd_rpy_deg')} deg; C_NED={pose.get('center_ned_m')} m"
                )
            lines.append("")
        relative_pose_text = cls._candidate_relative_pose_text(candidate)
        if relative_pose_text:
            lines.extend(relative_pose_text.splitlines())
            lines.append("")
        input_hashes = candidate.get("input_hashes")
        input_hashes = input_hashes if isinstance(input_hashes, Mapping) else {}
        for key in cls._CANDIDATE_DETAIL_INPUT_HASH_KEYS:
            if key in input_hashes:
                lines.append(f"{key}: {input_hashes[key]}")
        quality = candidate.get("quality_metrics")
        quality = quality if isinstance(quality, Mapping) else {}
        for key in cls._CANDIDATE_DETAIL_QUALITY_KEYS:
            if key in quality:
                lines.append(f"{key}: {quality[key]:.6g}")
        warnings = candidate.get("warnings")
        by_side = candidate.get("observable_dof_by_side")
        if isinstance(by_side, Mapping):
            lines.append("observable_dof_by_side:")
            for side in ("left", "right"):
                values = by_side.get(side)
                values = values if isinstance(values, Mapping) else {}
                enabled = [str(key) for key, value in values.items() if bool(value)]
                lines.append(f"  {side}: {','.join(enabled) or 'none'}")
        if isinstance(warnings, (tuple, list)) and warnings:
            lines.append("warnings:")
            lines.extend(f"  - {text}" for text in warnings)
        return "\n".join(lines)

    def _refresh_candidate_actions(self) -> None:
        candidate_id = self.selected_candidate_id()
        detail_widget = getattr(self, "candidate_detail_text", None)
        if not candidate_id:
            self.apply_button.setEnabled(False)
            self.candidate_status_label.setText("No pose candidate")
            if detail_widget is not None:
                detail_widget.setPlainText("")
            return
        candidate = self._candidate_rows.get(candidate_id, {})
        state = candidate.get("state", candidate.get("quality", "available"))
        owned_dofs = self._candidate_owned_dofs(candidate)
        observable = candidate.get("observable_dof")
        observable = observable if isinstance(observable, Mapping) else {}
        unobservable = tuple(name for name in owned_dofs if not bool(observable.get(name, False)))
        apply_ready = self._selected_candidate_applicable() and bool(owned_dofs) and not unobservable
        self.apply_button.setEnabled(apply_ready)
        dof_text = ",".join(owned_dofs) or "none"
        suffix = " · BLOCKED unobservable=" + ",".join(unobservable) if unobservable else ""
        self.candidate_status_label.setText(f"{candidate_id} · {state} · Apply DOF={dof_text}{suffix}")
        if detail_widget is not None:
            detail_widget.setPlainText(self._candidate_detail_text(candidate))

    def combine_candidate_ids(self) -> tuple[str, ...]:
        """Return only the source candidates explicitly checked for composition."""

        candidate_list = getattr(self, "combine_candidate_list", None)
        if candidate_list is None:
            return ()
        candidate_ids: list[str] = []
        for index in range(candidate_list.count()):
            item = candidate_list.item(index)
            if item.checkState() == QtCore.Qt.Checked:
                candidate_id = str(item.data(QtCore.Qt.UserRole) or "")
                if candidate_id:
                    candidate_ids.append(candidate_id)
        return tuple(candidate_ids)

    def _sync_combine_candidate_list(
        self,
        candidates: Sequence[Mapping[str, object]],
        selected_ids: set[str],
    ) -> None:
        candidate_list = getattr(self, "combine_candidate_list", None)
        if candidate_list is None:
            return
        candidate_list.blockSignals(True)
        try:
            candidate_list.clear()
            for candidate in candidates:
                candidate_id = str(candidate.get("candidate_id", candidate.get("id", "")) or "")
                if not candidate_id:
                    continue
                method = str(candidate.get("method", candidate.get("solver", "candidate")) or "candidate")
                state = str(candidate.get("state", candidate.get("quality", "AVAILABLE")) or "AVAILABLE")
                observable = self._candidate_observability(candidate)
                item = QtWidgets.QListWidgetItem(f"{method} · {candidate_id} · DOF={observable} · {state}")
                item.setData(QtCore.Qt.UserRole, candidate_id)
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
                item.setCheckState(QtCore.Qt.Checked if candidate_id in selected_ids else QtCore.Qt.Unchecked)
                candidate_list.addItem(item)
        finally:
            candidate_list.blockSignals(False)
        self._on_combine_candidates_changed()

    def combine_candidate_dofs(self) -> dict[str, list[str]]:
        """Per checked candidate, only the axes explicitly kept checked."""

        table = getattr(self, "combine_dof_table", None)
        if table is None:
            return {}
        axis_names = ("roll_deg", "pitch_deg", "yaw_deg", "north_m", "east_m", "down_m")
        result: dict[str, list[str]] = {}
        for row in range(table.rowCount()):
            header_item = table.item(row, 0)
            if header_item is None:
                continue
            candidate_id = str(header_item.data(QtCore.Qt.UserRole) or "")
            if not candidate_id:
                continue
            checked: list[str] = []
            for column, axis in enumerate(axis_names, start=1):
                cell = table.item(row, column)
                if cell is not None and cell.checkState() == QtCore.Qt.Checked:
                    checked.append(axis)
            result[candidate_id] = checked
        return result

    def _sync_combine_dof_table(self) -> None:
        """Rebuild per-candidate axis rows for the currently checked candidates.

        Defaults each axis to checked when the candidate actually observed
        it; an axis the candidate never observed is shown unchecked and
        disabled since it has nothing valid to contribute there. An axis an
        operator already unchecked is preserved across a rebuild triggered
        by checking/unchecking a different candidate.
        """

        table = getattr(self, "combine_dof_table", None)
        if table is None:
            return
        axis_names = ("roll_deg", "pitch_deg", "yaw_deg", "north_m", "east_m", "down_m")
        previous: dict[str, set[str]] = {}
        for row in range(table.rowCount()):
            header_item = table.item(row, 0)
            if header_item is None:
                continue
            candidate_id = str(header_item.data(QtCore.Qt.UserRole) or "")
            previous[candidate_id] = {
                axis_names[column - 1]
                for column in range(1, 7)
                if (cell := table.item(row, column)) is not None and cell.checkState() == QtCore.Qt.Checked
            }
        checked_ids = self.combine_candidate_ids()
        table.blockSignals(True)
        try:
            table.setRowCount(len(checked_ids))
            for row, candidate_id in enumerate(checked_ids):
                candidate = self._candidate_rows.get(candidate_id, {})
                method = str(candidate.get("method", candidate.get("solver", "candidate")) or "candidate")
                header_item = QtWidgets.QTableWidgetItem(f"{method} · {candidate_id}")
                header_item.setFlags(QtCore.Qt.ItemIsEnabled)
                header_item.setData(QtCore.Qt.UserRole, candidate_id)
                table.setItem(row, 0, header_item)
                observable = candidate.get("observable_dof")
                observable = observable if isinstance(observable, Mapping) else {}
                kept = previous.get(candidate_id)
                for column, axis in enumerate(axis_names, start=1):
                    cell = QtWidgets.QTableWidgetItem()
                    if bool(observable.get(axis)):
                        cell.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
                        default_checked = axis in kept if kept is not None else True
                        cell.setCheckState(QtCore.Qt.Checked if default_checked else QtCore.Qt.Unchecked)
                    else:
                        cell.setFlags(QtCore.Qt.NoItemFlags)
                        cell.setCheckState(QtCore.Qt.Unchecked)
                    table.setItem(row, column, cell)
        finally:
            table.blockSignals(False)

    def _on_combine_candidates_changed(self, *_args: object) -> None:
        self._sync_combine_dof_table()
        self._refresh_combine_action()

    def _refresh_combine_action(self, *_args: object) -> None:
        selected_count = len(self.combine_candidate_ids())
        button = getattr(self, "combine_button", None)
        status = getattr(self, "combine_candidate_status", None)
        overlap: set[str] = set()
        if selected_count >= 2:
            claimed_by: dict[str, str] = {}
            for candidate_id, axes in self.combine_candidate_dofs().items():
                for axis in axes:
                    owner = claimed_by.get(axis)
                    if owner is not None and owner != candidate_id:
                        overlap.add(axis)
                    claimed_by[axis] = candidate_id
        if button is not None:
            button.setEnabled(selected_count >= 2 and not overlap)
        if status is None:
            return
        if selected_count < 2:
            status.setText(f"{selected_count} candidate checked · select at least two")
        elif overlap:
            status.setText("Axis claimed by more than one candidate: " + ", ".join(sorted(overlap)) + " · uncheck one")
        else:
            status.setText(f"{selected_count} candidates checked · owner will verify shared prior and exact K,D")

    def _request_apply(self) -> None:
        payload = self.apply_payload()
        if payload is not None:
            self.command_requested.emit("alignment.pose.apply_candidate", payload)

    def _request_revert(self) -> None:
        self.command_requested.emit("alignment.pose.revert", self.revert_payload())

    def _request_reset(self) -> None:
        self.command_requested.emit("alignment.pose.reset", {})

    def information_rows(self):
        return tuple(dict(row) for row in self._candidate_rows.values())
