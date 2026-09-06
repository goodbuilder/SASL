from __future__ import annotations

from typing import Callable, Mapping

from PyQt5 import QtCore, QtWidgets

from Workstation.workstation_core.coordinate_contract import MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME

from .components import EngineeringValue, PropertySection
from .evidence import EvidenceAdapter
from .navigation import canonical_context_id
from .status_model import RuntimeStatus, StatusModel


_CONTEXT_LABELS = {
    "overview": "Overview",
    "devices.cameras": "Cameras",
    "devices.time": "Time / Sync",
    "devices.pixhawk": "Pixhawk",
    "devices.gnss": "Base GNSS",
    "processing.experiment": "Protocol",
    "processing.measurement": "Measurement",
    "processing.alignment": "Alignment",
    "tasks.data": "Data",
    "tasks.calibration": "Calibration",
    "tasks.advanced": "Advanced",
    "tasks.settings": "Settings",
    "diagnostics.system": "Diagnostics · System",
    "diagnostics.camera": "Diagnostics · Camera",
    "diagnostics.pixhawk": "Diagnostics · Pixhawk / MAVLink",
    "diagnostics.gnss": "Diagnostics · GNSS / RTK",
    "diagnostics.logs": "Diagnostics · Logs",
    "diagnostics.ftp": "Diagnostics · FTP",
    "diagnostics.measurement": "Diagnostics · Measurement",
    "diagnostics.performance": "Diagnostics · Performance",
}

_SELECTION_FIELDS = ("Selection", "Value", "Freshness", "Contract", "Verdict")

_CONTEXT_LAYOUTS = {
    "overview": ("System summary", ("Health", "Run state", "Update", "Contract", "Verdict")),
    "devices.cameras": (
        "Camera pair",
        ("Pair", "Detection", "Freshness", "Image contract", "Next check"),
    ),
    "devices.pixhawk": (
        "Pixhawk pair",
        ("Receivers", "Attitude", "Freshness", "Frame contract", "Next check"),
    ),
    "devices.gnss": (
        "Base operation",
        ("Mode / state", "Survey / NTRIP", "RTCM delivery", "Position contract", "Fault / next"),
    ),
    "devices.time": (
        "Time synchronization",
        ("State", "Evidence", "Clock basis", "Time contract", "Next check"),
    ),
    "processing.experiment": (
        "Protocol",
        ("Readiness", "Current step", "Freshness", "Evidence contract", "Next action"),
    ),
    "processing.alignment": (
        "Physical alignment",
        ("Sampling", "Offsets", "Quality", "Frame contract", "Instruction"),
    ),
    "processing.measurement": (
        "Measurement",
        ("State", "Result / reason", "Timestamp", "Output contract", "Verdict"),
    ),
    "tasks.data": (
        "Data",
        ("State", "Recording / queue", "Last write", "Storage root", "Verdict"),
    ),
    "tasks.calibration": (
        "Calibration",
        ("Profile", "Result", "Image grid", "Frame contract", "Next check"),
    ),
    "tasks.advanced": (
        "Advanced tools",
        ("Scope", "Controls", "Timing", "Contracts", "Access"),
    ),
    "tasks.settings": (
        "Settings",
        ("Scope", "Applied values", "Persistence", "Contract", "Next step"),
    ),
    "diagnostics.system": (
        "System diagnostics",
        ("Channels", "Faults", "Clock", "Aggregation", "Verdict"),
    ),
    "diagnostics.camera": (
        "Camera diagnostics",
        ("Pair", "Detection", "Freshness", "Image contract", "Next check"),
    ),
    "diagnostics.pixhawk": (
        "Pixhawk diagnostics",
        ("Receivers", "Attitude", "Freshness", "Frame contract", "Next check"),
    ),
    "diagnostics.gnss": (
        "GNSS / RTK diagnostics",
        ("Mode / state", "Survey / NTRIP", "RTCM delivery", "Position contract", "Fault / next"),
    ),
    "diagnostics.logs": (
        "Log evidence",
        ("Entries", "Latest", "Ordering", "Parsing", "Verdict"),
    ),
    "diagnostics.ftp": (
        "FTP evidence",
        ("Entries", "Latest", "Ordering", "Scope", "Verdict"),
    ),
    "diagnostics.measurement": (
        "Measurement diagnostics",
        ("State", "Result / reason", "Timestamp", "Output contract", "Verdict"),
    ),
    "diagnostics.performance": (
        "Performance evidence",
        ("State", "Rates", "Clock", "Units", "Verdict"),
    ),
}


class InspectorWidget(QtWidgets.QWidget):
    command_requested = QtCore.pyqtSignal(str, object)

    def __init__(self, backend: object, open_advanced: Callable[[], None], parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.open_advanced = open_advanced
        self._context_id = "overview"
        self._selection: dict[str, object] | None = None
        self._optical_pose_snapshot: dict[str, object] = {}
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)
        self.context_label = QtWidgets.QLabel("Overview")
        self.context_label.setObjectName("inspectorContext")
        self.context_label.setWordWrap(True)
        root.addWidget(self.context_label)
        self.status_section = PropertySection("Status")
        self.status_value = EngineeringValue("")
        self.source_value = EngineeringValue("")
        self.status_value.label.hide()
        self.source_value.label.hide()
        self.status_section.add_row("State", self.status_value)
        self.status_section.add_row("Evidence", self.source_value)
        root.addWidget(self.status_section)

        self.mountain_margin_section = PropertySection("Mountain detection")
        self.mountain_margin_combo = QtWidgets.QComboBox()
        self.mountain_margin_combo.setObjectName("inspectorMountainRidgeMargin")
        self.mountain_margin_combo.setEditable(False)
        for percent in (2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 15.0):
            label = f"{percent:.0f}%" + ("  (default)" if percent == 6.0 else "")
            self.mountain_margin_combo.addItem(label, percent / 100.0)
        self.mountain_margin_combo.setToolTip(
            "How close a matched point must be to Mountain's estimated ridge/skyline\n"
            "boundary to be kept, as a fraction of image height. Wider keeps more\n"
            "matches but risks drifting off the ridge; narrower is stricter but can\n"
            "drop legitimate points on a jagged silhouette. Takes effect on the next\n"
            "Auto-select Mountain run, not live."
        )
        self.mountain_margin_combo.currentIndexChanged.connect(self._request_mountain_ridge_margin)
        self.mountain_margin_section.add_row("Ridge margin", self.mountain_margin_combo)

        self.mountain_max_points_combo = QtWidgets.QComboBox()
        self.mountain_max_points_combo.setObjectName("inspectorMountainMaxPoints")
        self.mountain_max_points_combo.setEditable(False)
        for count in (4, 8, 12, 16, 20, 30, 40):
            label = f"{count}" + ("  (default)" if count == 12 else "")
            self.mountain_max_points_combo.addItem(label, count)
        self.mountain_max_points_combo.setToolTip(
            "How many spatially-spread correspondences Mountain keeps for the\n"
            "final suggestion, out of however many actually pass matching and\n"
            "RANSAC. More gives the solver more evidence but crowds the overlay;\n"
            "fewer keeps the overlay readable. Takes effect on the next\n"
            "Auto-select Mountain run, not live."
        )
        self.mountain_max_points_combo.currentIndexChanged.connect(self._request_mountain_max_points)
        self.mountain_margin_section.add_row("Max points", self.mountain_max_points_combo)
        root.addWidget(self.mountain_margin_section)

        self.context_section = PropertySection("Context")
        self.context_values: dict[str, EngineeringValue] = {}
        self.context_row_labels: dict[str, QtWidgets.QLabel] = {}
        for key, label in (
            ("primary", "Selection"),
            ("secondary", "Value"),
            ("timing", "Freshness"),
            ("contract", "Contract"),
            ("verdict", "Verdict"),
        ):
            value = EngineeringValue("")
            value.label.hide()
            self.context_values[key] = value
            self.context_section.add_row(label, value)
            row_label = self.context_section.form.labelForField(value)
            if isinstance(row_label, QtWidgets.QLabel):
                self.context_row_labels[key] = row_label
        root.addWidget(self.context_section)

        self.geometry_section = PropertySection("Active Geometry")
        self.geometry_values: dict[str, EngineeringValue] = {}
        for key, label in (
            ("bundle", "Bundle"),
            ("intrinsics", "K,D"),
            ("extrinsics", "R,t"),
            ("pose", "Pose"),
            ("evidence", "Evidence"),
        ):
            value = EngineeringValue(label)
            value.label.hide()
            self.geometry_values[key] = value
            self.geometry_section.add_row(label, value)

        self.geometry_candidate_combo = QtWidgets.QComboBox()
        self.geometry_candidate_combo.setObjectName("inspectorPoseCandidate")
        self.geometry_candidate_combo.setToolTip(
            "Select an immutable optical-pose candidate. The full observable DOF "
            "mask is applied through the OpticalPoseController."
        )
        self.geometry_candidate_combo.currentIndexChanged.connect(
            self._refresh_geometry_actions
        )
        self.geometry_section.add_row("Candidate", self.geometry_candidate_combo)

        geometry_actions = QtWidgets.QHBoxLayout()
        self.geometry_apply_candidate_button = QtWidgets.QPushButton("Apply pose candidate")
        self.geometry_apply_candidate_button.setObjectName("inspectorApplyPoseCandidate")
        self.geometry_apply_candidate_button.setToolTip(
            "Apply the selected candidate's observable Roll/Pitch/Yaw and NED position DOFs."
        )
        self.geometry_apply_candidate_button.clicked.connect(
            self._request_geometry_candidate
        )
        self.geometry_revert_button = QtWidgets.QPushButton("Revert pose")
        self.geometry_revert_button.setObjectName("inspectorRevertPose")
        self.geometry_revert_button.setToolTip(
            "Restore the immutable extrinsics profile that was active before the pose selection."
        )
        self.geometry_revert_button.clicked.connect(self._request_geometry_revert)
        self.geometry_apply_bundle_button = QtWidgets.QPushButton("Apply K,D + R,t")
        self.geometry_apply_bundle_button.setObjectName("inspectorApplyGeometryBundle")
        self.geometry_apply_bundle_button.setToolTip(
            "Apply the currently selected operating K,D and R,t bundle through the calibration owner."
        )
        self.geometry_apply_bundle_button.clicked.connect(self._request_geometry_bundle)
        geometry_actions.addWidget(self.geometry_apply_candidate_button)
        geometry_actions.addWidget(self.geometry_revert_button)
        geometry_actions.addWidget(self.geometry_apply_bundle_button)
        geometry_actions.addStretch(1)
        geometry_action_host = QtWidgets.QWidget()
        geometry_action_host.setLayout(geometry_actions)
        self.geometry_section.form.addRow(geometry_action_host)
        root.addWidget(self.geometry_section)

        self.output_section = PropertySection("Output")
        contract_value = QtWidgets.QLabel(f"{MEASUREMENT_FRAME}\n[F, R, D] · metres")
        contract_value.setWordWrap(True)
        contract_value.setToolTip(MEASUREMENT_AXIS_CONTRACT)
        contract_value.setMinimumHeight(42)
        contract_value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.output_section.add_row("Frame", contract_value)
        root.addWidget(self.output_section)

        self.open_advanced_button = QtWidgets.QPushButton("Advanced tools")
        self.open_advanced_button.setToolTip(
            "Open Workstation low-level controls and diagnostics."
        )
        self.open_advanced_button.clicked.connect(self.open_advanced)
        root.addWidget(self.open_advanced_button)
        root.addStretch(1)

    def refresh(self, context_id: str, status_model: StatusModel) -> None:
        context_id = canonical_context_id(context_id)
        if context_id != self._context_id:
            self._selection = None
        self._context_id = context_id
        self._configure_context_section(context_id)
        self._status_model = status_model
        self.context_label.setText(_CONTEXT_LABELS.get(context_id, context_id.replace(".", " · ").title()))
        key_map = {
            "devices.cameras": ("camera.left", "camera.right"),
            "devices.pixhawk": ("pixhawk.left", "pixhawk.right"),
            "devices.gnss": ("gnss.base", "gnss.ntrip"),
            "devices.time": ("synchronization",),
            "processing.experiment": ("gnss", "synchronization", "measurement", "recording"),
            "processing.measurement": ("measurement",),
            "tasks.data": ("recording", "measurement"),
            "diagnostics.camera": ("camera.left", "camera.right"),
            "diagnostics.pixhawk": ("pixhawk.left", "pixhawk.right"),
            "diagnostics.gnss": ("gnss.base", "gnss.ntrip"),
            "diagnostics.measurement": ("measurement",),
            "diagnostics.performance": ("fps",),
            "processing.alignment": ("gnss",),
        }
        status_keys = key_map.get(context_id, ())
        statuses = [status for key in status_keys if (status := status_model.record(key)) is not None]
        self.status_section.setVisible(bool(status_keys))
        if len(statuses) > 1:
            state_text = " · ".join(f"{status.label}={status.state}" for status in statuses)
            detail_text = " · ".join(f"{status.label}: {status.detail}" for status in statuses)
            source_text = " + ".join(dict.fromkeys(status.source for status in statuses if status.source))
            ages = [status.age_s for status in statuses if status.age_s is not None]
            age_s = max(ages) if ages else None
            severity_values = {status.severity.value for status in statuses}
            valid = False if "bad" in severity_values else True if severity_values == {"ok"} else None
        else:
            status = statuses[0] if statuses else None
            state_text = status.state if status else "UNKNOWN"
            detail_text = status.detail if status else "no evidence"
            source_text = status.source if status else "status adapter"
            age_s = status.age_s if status else None
            valid = (
                True
                if status is not None and status.severity.value == "ok"
                else False
                if status is not None and status.severity.value == "bad"
                else None
            )
        self.status_value.set_value(
            state_text,
            source=source_text,
            age_s=age_s,
            valid=valid,
        )
        self.source_value.set_value(detail_text, source=source_text)
        self.output_section.setVisible(context_id in {"processing.measurement", "tasks.data"})
        self.geometry_section.setVisible(context_id != "tasks.data")
        self.open_advanced_button.setVisible(context_id.startswith("diagnostics."))
        self._refresh_context_details(context_id)
        self._refresh_active_geometry()
        self._refresh_mountain_margin_control(context_id)

    def inspect_selection(
        self,
        context_id: str,
        selection: Mapping[str, object],
        status_model: StatusModel,
    ) -> None:
        """Show one workspace-selected value without changing its contract."""

        context_id = canonical_context_id(context_id)
        self._context_id = context_id
        self._status_model = status_model
        self._selection = dict(selection)
        self._configure_context_section(context_id)
        self.context_label.setText(_CONTEXT_LABELS.get(context_id, context_id.replace(".", " · ").title()))
        self.geometry_section.setVisible(context_id != "tasks.data")
        self._refresh_context_details(context_id)
        self._refresh_active_geometry()
        self._refresh_mountain_margin_control(context_id)

    def _read_optical_pose_snapshot(self) -> dict[str, object]:
        readers = (
            getattr(self.backend, "optical_pose_snapshot", None),
            getattr(getattr(self.backend, "panel", self.backend), "workstation_optical_pose_snapshot", None),
        )
        for reader in readers:
            if not callable(reader):
                continue
            try:
                value = reader(include_frames=False)
            except TypeError:
                try:
                    value = reader()
                except Exception:
                    continue
            except Exception:
                continue
            if isinstance(value, Mapping):
                return dict(value)
        return dict(self._optical_pose_snapshot)

    @staticmethod
    def _format_float(value: object, *, digits: int = 3) -> str:
        try:
            return f"{float(value):.{digits}f}"
        except (TypeError, ValueError):
            return "--"

    @classmethod
    def _format_vector(cls, value: object, *, digits: int = 3) -> str:
        if not isinstance(value, (tuple, list)):
            return "--"
        return "[" + ", ".join(cls._format_float(item, digits=digits) for item in value) + "]"

    @classmethod
    def _format_matrix(cls, value: object, *, digits: int = 4) -> str:
        if not isinstance(value, (tuple, list)):
            return "--"
        return "[" + "; ".join(
            cls._format_vector(row, digits=digits) for row in value
            if isinstance(row, (tuple, list))
        ) + "]"

    def _refresh_active_geometry(self) -> None:
        """Keep active K,D, R,t, pose selection, and provenance together."""

        evidence = getattr(self.backend, "evidence", None)
        if not isinstance(evidence, EvidenceAdapter):
            evidence = EvidenceAdapter(getattr(self.backend, "panel", self.backend))
        try:
            calibration = evidence.calibration()
        except Exception:
            calibration = {}
        snapshot = self._read_optical_pose_snapshot()
        self._optical_pose_snapshot = snapshot

        state = str(calibration.get("state", "NO DATA") or "NO DATA")
        combination = str(calibration.get("combination", "K,D + R,t not selected") or "")
        self.geometry_values["bundle"].set_value(
            combination,
            source="EvidenceAdapter.calibration · active_intrinsics + selected_extrinsics_profile",
            frame=str(calibration.get("frame_contract", "OpenCV optical RDF")),
            valid=state not in {"NO DATA", "CALCULATING"},
        )

        left = calibration.get("left") if isinstance(calibration.get("left"), Mapping) else {}
        right = calibration.get("right") if isinstance(calibration.get("right"), Mapping) else {}
        intrinsics_text = (
            f"{state} · zoom={calibration.get('intrinsics_source_zoom', calibration.get('zoom', '--'))} · "
            f"profile={calibration.get('profile_path') or '--'} · "
            f"hash={str(calibration.get('intrinsics_profile_hash') or '')[:12] or '--'}"
        )
        self.geometry_values["intrinsics"].set_value(
            intrinsics_text,
            source=str(calibration.get("profile_path") or "active K,D source unavailable"),
            frame="OpenCV camera matrix K in pixels; distortion D; native image u/v",
            valid=state in {"ACTIVE", "AVAILABLE"},
        )
        self.geometry_values["intrinsics"].setToolTip(
            "K,D values\n"
            f"Left K fx/fy/cx/cy={self._format_float(left.get('fx_px'))}/"
            f"{self._format_float(left.get('fy_px'))}/"
            f"{self._format_float(left.get('cx_px'))}/"
            f"{self._format_float(left.get('cy_px'))}; D={left.get('distortion_coefficients', '--')}\n"
            f"Right K fx/fy/cx/cy={self._format_float(right.get('fx_px'))}/"
            f"{self._format_float(right.get('fy_px'))}/"
            f"{self._format_float(right.get('cx_px'))}/"
            f"{self._format_float(right.get('cy_px'))}; D={right.get('distortion_coefficients', '--')}\n"
            f"Source: {calibration.get('profile_path') or '--'}\n"
            f"Profile SHA-256: {calibration.get('intrinsics_profile_hash') or '--'}"
        )

        extrinsics = calibration.get("extrinsics") if isinstance(calibration.get("extrinsics"), Mapping) else {}
        extrinsics_method = str(calibration.get("extrinsics_method") or "base/legacy")
        extrinsics_path = str(calibration.get("extrinsics_profile_path") or "--")
        extrinsics_text = (
            f"{extrinsics_method} · {extrinsics_path} · "
            f"T={self._format_vector(extrinsics.get('translation_left_to_right_m'))} m · "
            f"baseline={self._format_float(extrinsics.get('baseline_m'))} m"
        )
        self.geometry_values["extrinsics"].set_value(
            extrinsics_text,
            source=f"{extrinsics_method} · {extrinsics_path}",
            frame=str(extrinsics.get("frame_contract", "OpenCV optical RDF; X_right=R*X_left+T")),
            valid=bool(extrinsics),
        )
        self.geometry_values["extrinsics"].setToolTip(
            f"Method: {extrinsics_method}\n"
            f"Profile: {extrinsics_path}\n"
            f"Profile SHA-256: {calibration.get('extrinsics_profile_hash') or '--'}\n"
            f"R={self._format_matrix(extrinsics.get('rotation_left_to_right'))}\n"
            f"T={self._format_vector(extrinsics.get('translation_left_to_right_m'))} m\n"
            f"Right origin in left RDF={self._format_vector(extrinsics.get('right_camera_origin_in_left_rdf_m'))} m\n"
            f"Reprojection error={extrinsics.get('reprojection_error_px', '--')} px"
        )

        active_candidate_id = str(snapshot.get("active_candidate_id") or "")
        active_selection_id = str(snapshot.get("active_selection_id") or "")
        active_dof = snapshot.get("active_selected_dof")
        if active_selection_id:
            dof_text = ",".join(
                key for key, value in (active_dof.items() if isinstance(active_dof, Mapping) else ()) if value
            ) or "none"
            pose_text = f"ACTIVE · candidate={active_candidate_id} · selection={active_selection_id} · DOF={dof_text}"
            pose_valid = True
        else:
            pose_text = f"NOT APPLIED · {len(snapshot.get('candidates', ()) or ())} candidate(s) available"
            pose_valid = None
        self.geometry_values["pose"].set_value(
            pose_text,
            source="OpticalPoseController selection history",
            frame="camera optical RDF pose in world NED; derived stereo R,t",
            valid=pose_valid,
        )

        active_candidate = snapshot.get("active_candidate")
        source_ids = active_candidate.get("source_observation_ids", ()) if isinstance(active_candidate, Mapping) else ()
        hashes = active_candidate.get("input_hashes", {}) if isinstance(active_candidate, Mapping) else {}
        quality = active_candidate.get("quality_metrics", {}) if isinstance(active_candidate, Mapping) else {}
        evidence_text = (
            f"source ids={len(source_ids) if isinstance(source_ids, (tuple, list)) else 0} · "
            f"K,D hash={str(hashes.get('operating_intrinsics_profile_sha256', '') if isinstance(hashes, Mapping) else '')[:12] or '--'} · "
            f"GNSS baseline={self._format_float(quality.get('gnss_baseline_m') if isinstance(quality, Mapping) else None)} m"
        )
        self.geometry_values["evidence"].set_value(
            evidence_text,
            source="candidate source_observation_ids + input_hashes + quality_metrics",
            frame="GNSS baseline: right minus left, world NED metres",
            valid=True if active_candidate else None,
        )
        self.geometry_values["evidence"].setToolTip(
            "Active candidate provenance\n"
            f"Source observation IDs: {source_ids or '--'}\n"
            f"Input hashes: {hashes or '--'}\n"
            f"Quality metrics: {quality or '--'}\n"
            f"Warnings: {active_candidate.get('warnings', '--') if isinstance(active_candidate, Mapping) else '--'}"
        )

        self._refresh_geometry_candidates(snapshot)

    def _refresh_geometry_candidates(self, snapshot: Mapping[str, object]) -> None:
        candidates = snapshot.get("candidates")
        rows = [dict(row) for row in candidates if isinstance(row, Mapping)] if isinstance(candidates, (tuple, list)) else []
        previous_id = str(self.geometry_candidate_combo.currentData() or "")
        desired_id = str(snapshot.get("active_candidate_id") or previous_id)
        self.geometry_candidate_combo.blockSignals(True)
        self.geometry_candidate_combo.clear()
        desired_index = -1
        for index, row in enumerate(rows):
            candidate_id = str(row.get("candidate_id") or "")
            method = str(row.get("method") or "candidate")
            state = str(row.get("state") or "")
            label = f"{candidate_id} · {method} · {state}" if candidate_id else f"{method} · {state}"
            self.geometry_candidate_combo.addItem(label, candidate_id)
            self.geometry_candidate_combo.setItemData(index, row, QtCore.Qt.UserRole + 1)
            if candidate_id == desired_id:
                desired_index = index
        if desired_index < 0 and rows:
            desired_index = len(rows) - 1
        if desired_index >= 0:
            self.geometry_candidate_combo.setCurrentIndex(desired_index)
        self.geometry_candidate_combo.blockSignals(False)
        self._refresh_geometry_actions()

    def _selected_geometry_candidate(self) -> Mapping[str, object] | None:
        value = self.geometry_candidate_combo.itemData(
            self.geometry_candidate_combo.currentIndex(),
            QtCore.Qt.UserRole + 1,
        )
        return value if isinstance(value, Mapping) else None

    def _refresh_geometry_actions(self, _index: int = -1) -> None:
        candidate = self._selected_geometry_candidate()
        applicable = bool(candidate and candidate.get("applicable", True))
        observable = candidate.get("observable_dof") if isinstance(candidate, Mapping) else {}
        optimized = candidate.get("optimized_dof") if isinstance(candidate, Mapping) else {}
        owned = optimized if isinstance(optimized, Mapping) else observable
        owned_names = (
            tuple(str(name) for name, enabled in owned.items() if bool(enabled))
            if isinstance(owned, Mapping)
            else ()
        )
        apply_ready = bool(
            applicable
            and owned_names
            and isinstance(observable, Mapping)
            and all(bool(observable.get(name, False)) for name in owned_names)
        )
        self.geometry_apply_candidate_button.setEnabled(apply_ready)
        active = bool(self._optical_pose_snapshot.get("active_selection_id"))
        self.geometry_revert_button.setEnabled(active)
        evidence = getattr(self.backend, "evidence", None)
        calibration_state = "NO DATA"
        if isinstance(evidence, EvidenceAdapter):
            try:
                calibration_state = str(evidence.calibration().get("state", "NO DATA") or "NO DATA")
            except Exception:
                calibration_state = "NO DATA"
        has_bundle = calibration_state in {"ACTIVE", "AVAILABLE"}
        self.geometry_apply_bundle_button.setEnabled(has_bundle)

    def _request_geometry_candidate(self) -> None:
        candidate = self._selected_geometry_candidate()
        if not isinstance(candidate, Mapping):
            return
        self.command_requested.emit(
            "alignment.pose.apply_candidate",
            {"candidate_id": str(candidate.get("candidate_id") or "")},
        )

    def _request_geometry_revert(self) -> None:
        self.command_requested.emit(
            "alignment.pose.revert",
            {"restore": "previous_immutable_profile"},
        )

    def _request_geometry_bundle(self) -> None:
        self.command_requested.emit("calibration.apply_bundle", {})

    def _request_mountain_ridge_margin(self, _index: int = -1) -> None:
        fraction = self.mountain_margin_combo.currentData()
        if not isinstance(fraction, float):
            return
        self.command_requested.emit(
            "alignment.mountain_ridge_margin.set",
            {"ridge_margin_fraction": fraction},
        )

    def _request_mountain_max_points(self, _index: int = -1) -> None:
        count = self.mountain_max_points_combo.currentData()
        if not isinstance(count, int):
            return
        self.command_requested.emit(
            "alignment.mountain_max_points.set",
            {"max_points": count},
        )

    def _refresh_mountain_margin_control(self, context_id: str) -> None:
        self.mountain_margin_section.setVisible(context_id == "processing.alignment")
        if not self.mountain_margin_combo.view().isVisible():
            raw_margin = self._optical_pose_snapshot.get("mountain_ridge_margin_fraction")
            try:
                fraction = float(raw_margin)
            except (TypeError, ValueError):
                fraction = None
            if fraction is not None:
                closest_index = min(
                    range(self.mountain_margin_combo.count()),
                    key=lambda index: abs(float(self.mountain_margin_combo.itemData(index)) - fraction),
                    default=-1,
                )
                if closest_index >= 0:
                    self.mountain_margin_combo.blockSignals(True)
                    self.mountain_margin_combo.setCurrentIndex(closest_index)
                    self.mountain_margin_combo.blockSignals(False)
        if not self.mountain_max_points_combo.view().isVisible():
            raw_count = self._optical_pose_snapshot.get("mountain_max_points")
            try:
                count = int(raw_count)
            except (TypeError, ValueError):
                count = None
            if count is not None:
                closest_index = min(
                    range(self.mountain_max_points_combo.count()),
                    key=lambda index: abs(int(self.mountain_max_points_combo.itemData(index)) - count),
                    default=-1,
                )
                if closest_index >= 0:
                    self.mountain_max_points_combo.blockSignals(True)
                    self.mountain_max_points_combo.setCurrentIndex(closest_index)
                    self.mountain_max_points_combo.blockSignals(False)

    def _configure_context_section(self, context_id: str) -> None:
        if self._selection is not None:
            title = "Selection details"
            labels = _SELECTION_FIELDS
        else:
            title, labels = _CONTEXT_LAYOUTS.get(context_id, ("Context summary", _SELECTION_FIELDS))
        self.context_section.setTitle(title)
        for key, label in zip(self.context_values, labels):
            row_label = self.context_row_labels.get(key)
            if row_label is not None:
                row_label.setText(label)

    def _refresh_context_details(self, context_id: str) -> None:
        evidence = getattr(self.backend, "evidence", None)
        if not isinstance(evidence, EvidenceAdapter):
            evidence = EvidenceAdapter(getattr(self.backend, "panel", self.backend))
        values = {key: "--" for key in self.context_values}
        self.context_row_labels["timing"].setVisible(True)
        self.context_values["timing"].setVisible(True)
        source = "read-only source projection"
        frame = ""
        if self._selection is not None:
            selection = self._selection
            resolver = selection.get("_resolver")
            if callable(resolver):
                try:
                    refreshed = resolver()
                except Exception:
                    refreshed = None
                if isinstance(refreshed, Mapping):
                    selection.update(refreshed)
            values.update(
                primary=selection.get("selection", "--"),
                secondary=selection.get("value", "--"),
                timing=selection.get("freshness", "--"),
                contract=selection.get("contract", "--"),
                verdict=selection.get("verdict", "--"),
            )
            hide_timing = bool(selection.get("hide_freshness", False))
            self.context_row_labels["timing"].setVisible(not hide_timing)
            self.context_values["timing"].setVisible(not hide_timing)
            source = str(selection.get("source", source) or source)
            frame = str(selection.get("contract", "") or "")
        elif context_id == "overview":
            model = getattr(self, "_status_model", None)
            snapshot = model.snapshot() if isinstance(model, StatusModel) else {}
            states = list(snapshot.values())
            bad_count = sum(1 for item in states if item.severity.value == "bad")
            warn_count = sum(1 for item in states if item.severity.value == "warn")
            ready_count = sum(1 for item in states if item.severity.value == "ok")
            recording = evidence.recording()
            measurement = snapshot.get("measurement")
            values.update(
                primary=f"ready={ready_count} · warning={warn_count} · error={bad_count}",
                secondary=(
                    f"measurement={measurement.state if measurement else 'NO DATA'} · "
                    f"recording={recording.get('state', 'IDLE')}"
                ),
                timing="Live StatusModel snapshot",
                contract="Overview preserves each producer contract; it performs no coordinate conversion",
                verdict="ERROR PRESENT" if bad_count else "CHECK WARNINGS" if warn_count else "NO ACTIVE ERROR",
            )
            source = "StatusModel + recording evidence"
        elif context_id == "tasks.data":
            snapshot_method = getattr(self.backend, "data_storage_snapshot", None)
            if not callable(snapshot_method):
                snapshot_method = getattr(
                    getattr(self.backend, "panel", self.backend),
                    "data_storage_snapshot",
                    None,
                )
            try:
                storage = snapshot_method() if callable(snapshot_method) else {}
            except Exception as exc:
                storage = {"state": "UNAVAILABLE", "error": str(exc)}
            storage = storage if isinstance(storage, Mapping) else {}
            state = str(storage.get("state") or "UNAVAILABLE").upper()
            streams = storage.get("streams")
            streams = streams if isinstance(streams, Mapping) else {}
            errors = [
                str(row.get("last_error") or "")
                for row in streams.values()
                if isinstance(row, Mapping) and row.get("last_error")
            ]
            values.update(
                primary=state,
                secondary=(
                    f"priority={int(storage.get('priority_pending') or 0)} · "
                    f"bulk={int(storage.get('bulk_pending') or 0)} · "
                    f"dropped={int(storage.get('dropped_records') or 0)}"
                ),
                timing="Per-record UTC/source clocks",
                contract=str(storage.get("root") or "--"),
                verdict=(errors[0] if errors else str(storage.get("error") or state)),
            )
            source = "data_storage_snapshot"
        elif context_id == "tasks.settings":
            snapshot_method = getattr(self.backend, "workstation_settings_snapshot", None)
            try:
                settings = snapshot_method() if callable(snapshot_method) else {}
            except Exception:
                settings = {}
            settings = settings if isinstance(settings, Mapping) else {}
            gnss = settings.get("gnss_operation") if isinstance(settings.get("gnss_operation"), Mapping) else {}
            values.update(
                primary="Workstation system settings",
                secondary=(
                    f"camera host={settings.get('camera_host_address', '--')} · "
                    f"Pixhawk host={settings.get('pixhawk_host_address', '--')} · "
                    f"GNSS mode={gnss.get('operation_mode', settings.get('gnss_operation_mode', '--'))}"
                ),
                timing="Apply changes to the current runtime; Save persists them for the next run",
                contract="Credentials remain environment-variable references; coordinate units/frames are not converted here",
                verdict="Review, Apply, then Save",
            )
            source = "workstation_settings_snapshot"
        elif context_id == "diagnostics.system":
            model = getattr(self, "_status_model", None)
            snapshot = model.snapshot() if isinstance(model, StatusModel) else {}
            failing = [item.label for item in snapshot.values() if item.severity.value == "bad"]
            waiting = [item.label for item in snapshot.values() if item.severity.value in {"warn", "unknown"}]
            values.update(
                primary=f"{len(snapshot)} status channels",
                secondary=f"errors={', '.join(failing) or 'none'} · waiting={', '.join(waiting) or 'none'}",
                timing="StatusModel app_receive_monotonic snapshot",
                contract="System diagnostics aggregates state only; producer frame contracts remain unchanged",
                verdict="ERROR PRESENT" if failing else "WAITING EVIDENCE" if waiting else "HEALTHY",
            )
            source = "StatusModel"
        elif context_id == "diagnostics.logs":
            diagnostic_method = getattr(self.backend, "diagnostic_text", None)
            try:
                log_text = str(diagnostic_method() if callable(diagnostic_method) else "")
            except Exception:
                log_text = ""
            lines = [line for line in log_text.splitlines() if line.strip()]
            values.update(
                primary=f"{len(lines)} retained log lines",
                secondary=(lines[-1] if lines else "No diagnostic log evidence"),
                timing="Log ordering follows producer append order",
                contract="Text diagnostics only; numeric frame fields are not parsed or converted",
                verdict="AVAILABLE" if lines else "EMPTY",
            )
            source = "backend diagnostic_text"
        elif context_id == "diagnostics.ftp":
            diagnostic_method = getattr(self.backend, "ftp_diagnostic_text", None)
            try:
                log_text = str(diagnostic_method() if callable(diagnostic_method) else "")
            except Exception:
                log_text = ""
            lines = [line for line in log_text.splitlines() if line.strip()]
            values.update(
                primary=f"{len(lines)} retained FTP log lines",
                secondary=(lines[-1] if lines else "No FTP activity yet"),
                timing="Bounded append order; newest entries remain at the end",
                contract="FTP capture/download/save status and local paths are text evidence only",
                verdict="AVAILABLE" if lines and log_text != "No FTP activity yet." else "EMPTY",
            )
            source = "backend ftp_diagnostic_text"
        elif context_id == "diagnostics.performance":
            performance = self._status_for_context(context_id)
            values.update(
                primary=performance.state if performance else "NO DATA",
                secondary=performance.detail if performance else "No decoded FPS evidence",
                timing=performance.clock if performance else "--",
                contract="Decoded/inferred/displayed rates are hertz; no geometry contract applies",
                verdict=performance.state if performance else "NO DATA",
            )
            source = performance.source if performance else "YoloStreamWorker stats"
        elif context_id in {"devices.pixhawk", "diagnostics.pixhawk"}:
            left_attitude = evidence.attitude("left")
            right_attitude = evidence.attitude("right")
            baseline = evidence.baseline()
            values.update(
                primary=f"P1/P2={left_attitude.get('state', 'WAITING')}/{right_attitude.get('state', 'WAITING')}",
                secondary=(
                    f"r/p/y P1={left_attitude.get('roll_deg', '--')}/"
                    f"{left_attitude.get('pitch_deg', '--')}/{left_attitude.get('yaw_deg', '--')}° · "
                    f"P2={right_attitude.get('roll_deg', '--')}/"
                    f"{right_attitude.get('pitch_deg', '--')}/{right_attitude.get('yaw_deg', '--')}°"
                ),
                timing=f"age P1/P2 {left_attitude.get('age_s', '--')}/{right_attitude.get('age_s', '--')}s · MAVLink",
                contract=(
                    "attitude PX4 body FRD → world NED; "
                    f"baseline P1→P2 NED={baseline.get('north_m', '--')}/"
                    f"{baseline.get('east_m', '--')}/{baseline.get('down_m', '--')} m"
                ),
                verdict="Inspect P1/P2 rows or a baseline value for field-level evidence",
            )
            frame = "PX4 body FRD → world NED (orientation)"
            source = "CameraExtrinsicState"
        elif context_id in {"devices.gnss", "diagnostics.gnss"}:
            operation = evidence.gnss_operation()
            mode = str(operation.get("mode", "UNKNOWN") or "UNKNOWN")
            state = str(operation.get("state", "UNKNOWN") or "UNKNOWN")
            survey = operation.get("survey_in") if isinstance(operation.get("survey_in"), Mapping) else {}
            ntrip = operation.get("ntrip") if isinstance(operation.get("ntrip"), Mapping) else {}
            routes = operation.get("routes") if isinstance(operation.get("routes"), Mapping) else {}
            if mode == "LOCAL_SURVEY_IN":
                operation_value = (
                    f"Survey {('VALID' if survey.get('valid') else 'ACTIVE' if survey.get('active') else 'NOT RUNNING')} · "
                    f"{survey.get('duration_s', 0)}/{survey.get('target_duration_s', '--')} s · "
                    f"{survey.get('mean_accuracy_m', '--')}/{survey.get('target_accuracy_m', '--')} m"
                )
            else:
                operation_value = (
                    f"NTRIP {('CONNECTED' if ntrip.get('connected') else 'DISCONNECTED')} · "
                    f"{ntrip.get('caster_host', '--')}:{ntrip.get('caster_port', '--')}/"
                    f"{ntrip.get('mountpoint', '--')}"
                )
            route_ages = []
            for target in ("left", "right"):
                route = routes.get(target) if isinstance(routes, Mapping) else None
                if isinstance(route, Mapping):
                    route_ages.append(f"{target.upper()}={route.get('delivery_age_s', '--')}s")
            values.update(
                primary=f"{mode} · {state}",
                secondary=operation_value,
                timing=f"RTCM age={operation.get('last_frame_age_s', '--')}s · " + " · ".join(route_ages),
                contract="Base position WGS84 degrees/metres; Survey-in mean is ECEF XYZ metres; RTCM is transport data",
                verdict=(
                    str(operation.get("invalidation_reason"))
                    if operation.get("invalidation_reason")
                    else "FAULT" if state in {"FAULT", "INVALIDATED"} else "Inspect a Base GNSS value for details"
                ),
            )
            frame = "Base WGS84 / Survey-in ECEF; no rover baseline"
            source = "GnssOperationManager + UBX NAV-SVIN/NAV-PVT + NTRIP/RTCM counters"
        elif context_id in {"devices.cameras", "diagnostics.camera"}:
            config = evidence.detection_config()
            left_target = evidence.detection("left")
            right_target = evidence.detection("right")
            values.update(
                primary=f"CAM L/R={left_target.get('runtime_state', 'NO DATA')}/{right_target.get('runtime_state', 'NO DATA')}",
                secondary=f"conf/IoU={config.get('confidence', '--')}/{config.get('iou', '--')} "
                f"gates={','.join(str(item) for item in config.get('gate_enabled', ()) or ()) or 'OFF'}",
                timing=f"target age L/R={left_target.get('age_s', '--')}/{right_target.get('age_s', '--')}s · "
                f"device={config.get('device', '--')} input={config.get('input_size_px', '--')}px",
                contract="bbox x1/y1/x2/y2 and center_u/v are image pixels",
            )
            frame = "OpenCV image pixel u/v; no metric conversion"
            source = "AppSettings + DetectionObservation"
        elif context_id == "devices.time":
            status = self._status_for_context(context_id)
            values.update(
                primary=status.state if status else "WAITING",
                secondary=status.detail if status else "RTCP/NTP evidence waiting",
                timing=status.clock if status else "--",
                contract="WIRIS RTCP/NTP timestamps; camera-pair timing evidence",
            )
            source = status.source if status else "clock sync status"
        elif context_id == "processing.alignment":
            alignment = evidence.physical_alignment()
            quality = alignment.get("quality") if isinstance(alignment.get("quality"), Mapping) else {}
            values.update(
                primary=f"{alignment.get('state', 'WAITING')} · samples={alignment.get('sample_count', 0)}",
                secondary=(
                    f"P1→P2 FRD={alignment.get('current_baseline_frd_m', '--')} m · "
                    f"ΔR/P/Y={alignment.get('relative_rpy_deg', '--')}°"
                ),
                timing=f"fixed ratio={quality.get('fixed_ratio', '--')} · iterations={len(alignment.get('iterations', ()) or ())}",
                contract=str(alignment.get("coordinate_contract", "raw NED; P1 body FRD; relative R/P/Y")),
                verdict="Physical adjustment only; no software R/P/Y correction is applied",
            )
            frame = str(alignment.get("coordinate_contract", ""))
            source = "Physical alignment evidence"
        elif context_id == "tasks.advanced":
            values.update(
                primary="Advanced",
                secondary="Camera · PX4 · fields · preview · files",
                timing="manual controls",
                contract="camera side, PX4 body FRD→world NED, baseline NED, saved time axis preserved",
            )
            source = "Workstation advanced facade"
        elif context_id == "tasks.calibration":
            try:
                calibration = evidence.calibration()
            except Exception:
                calibration = {}
            left = calibration.get("left", {})
            values.update(
                primary=f"{calibration.get('state', 'NO DATA')} zoom={calibration.get('zoom', '--')}",
                secondary=f"profile={calibration.get('profile_path') or '--'}",
                timing=f"left grid={left.get('width_px', '--')}×{left.get('height_px', '--')} px",
                contract=str(calibration.get("frame_contract", "OpenCV optical RDF")),
            )
            source = "active_intrinsics/last_calibration_result"
        else:
            measurement = self._status_for_context(context_id)
            values.update(
                primary=measurement.state if measurement else "NO DATA",
                secondary=measurement.detail if measurement else "evidence waiting",
                timing=measurement.clock if measurement else "--",
                contract=f"{MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]",
                verdict=(measurement.detail if measurement else "No accepted measurement"),
            )
            source = measurement.source if measurement else source
        for key, value in self.context_values.items():
            self.context_values[key].set_value(values[key], source=source, frame=frame if key == "contract" else "")

    def _status_for_context(self, context_id: str) -> RuntimeStatus | None:
        key_map = {
            "processing.measurement": "measurement",
            "processing.synchronization": "synchronization",
            "devices.time": "synchronization",
            "diagnostics.measurement": "measurement",
            "diagnostics.performance": "fps",
            "processing.alignment": "gnss",
        }
        model = getattr(self, "_status_model", None)
        status_key = key_map.get(context_id)
        return model.record(status_key) if isinstance(model, StatusModel) and status_key is not None else None
