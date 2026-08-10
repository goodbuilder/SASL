from __future__ import annotations

from typing import Callable, Mapping

from PyQt5 import QtCore, QtWidgets

from workstation_core.coordinate_contract import MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME

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
    "tasks.calibration": "Pre-run Calibration",
    "tasks.recording": "Recording / Sessions",
    "tasks.advanced": "Advanced",
    "tasks.settings": "Settings",
    "diagnostics.system": "Diagnostics · System",
    "diagnostics.camera": "Diagnostics · Camera",
    "diagnostics.pixhawk": "Diagnostics · Pixhawk / MAVLink",
    "diagnostics.gnss": "Diagnostics · GNSS / RTK",
    "diagnostics.logs": "Diagnostics · Logs",
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
        "Field protocol",
        ("Sequence", "Field path", "Evidence", "Coordinate impact", "Next step"),
    ),
    "processing.alignment": (
        "Physical alignment",
        ("Sampling", "Offsets", "Quality", "Frame contract", "Instruction"),
    ),
    "processing.measurement": (
        "Measurement",
        ("State", "Result / reason", "Timestamp", "Output contract", "Verdict"),
    ),
    "tasks.calibration": (
        "Calibration",
        ("Profile", "Result", "Image grid", "Frame contract", "Next check"),
    ),
    "tasks.recording": (
        "Recording session",
        ("Session", "Samples", "Finalization", "Output", "Verdict"),
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
    def __init__(self, backend: object, open_advanced: Callable[[], None], parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.open_advanced = open_advanced
        self._context_id = "overview"
        self._selection: dict[str, object] | None = None
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
            "devices.gnss": ("gnss.base",),
            "devices.time": ("synchronization",),
            "processing.measurement": ("measurement",),
            "tasks.recording": ("recording",),
            "diagnostics.camera": ("camera.left", "camera.right"),
            "diagnostics.pixhawk": ("pixhawk.left", "pixhawk.right"),
            "diagnostics.gnss": ("gnss.base",),
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
        self.output_section.setVisible(context_id in {"processing.measurement", "tasks.recording"})
        self.open_advanced_button.setVisible(context_id.startswith("diagnostics."))
        self._refresh_context_details(context_id)

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
        self._refresh_context_details(context_id)

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
        elif context_id == "processing.experiment":
            values.update(
                primary="0 external prerequisite · 1–11 field steps",
                secondary="connect → RTK/fix → physical alignment → measure → save",
                timing="advisory evidence checklist",
                contract="Protocol order does not alter measurement fields or frames",
            )
            source = "Experiment protocol evidence"
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
        elif context_id in {"tasks.recording"}:
            recording = evidence.recording()
            counts = recording.get("sample_counts", {})
            values.update(
                primary=f"{recording.get('state', 'IDLE')} session={recording.get('session_id') or '--'}",
                secondary=f"pairs={counts.get('pairs', 0) if isinstance(counts, dict) else '--'}",
                timing=f"elapsed={recording.get('elapsed_s', '--')}s; retry={recording.get('retry_required', False)}",
                contract=str(recording.get("output_dir") or "output directory unavailable"),
                verdict="RETRY REQUIRED" if recording.get("retry_required") else str(recording.get("state", "IDLE")),
            )
            source = "recording session/save worker"
        elif context_id == "tasks.advanced":
            values.update(
                primary="Advanced",
                secondary="Camera · PX4 · fields · preview · files",
                timing="manual controls",
                contract="camera side, PX4 body FRD→world NED, baseline NED, saved time axis preserved",
            )
            source = "Workstation advanced facade"
        elif context_id == "tasks.calibration":
            calibration = evidence.calibration()
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
