from __future__ import annotations

from typing import Callable

from PyQt5 import QtCore, QtWidgets

from wiris_stereo_distance.coordinate_contract import MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME

from .components import EngineeringValue, PropertySection
from .evidence import EvidenceAdapter
from .status_model import RuntimeStatus, StatusModel


class InspectorWidget(QtWidgets.QWidget):
    def __init__(self, backend: object, open_legacy: Callable[[], None], parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.open_legacy = open_legacy
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)
        self.context_label = QtWidgets.QLabel("Overview")
        self.context_label.setObjectName("inspectorContext")
        self.context_label.setWordWrap(True)
        root.addWidget(self.context_label)
        self.status_section = PropertySection("Runtime evidence")
        self.status_value = EngineeringValue("Value", show_source=True, source_prefix="")
        self.source_value = EngineeringValue("Detail", show_source=True, source_prefix="")
        self.status_section.add_row("State", self.status_value)
        self.status_section.add_row("Evidence", self.source_value)
        root.addWidget(self.status_section)

        self.camera_section = PropertySection("Camera configuration")
        self.left_url = QtWidgets.QLineEdit()
        self.right_url = QtWidgets.QLineEdit()
        self.left_url.setPlaceholderText("Left RTSP URL")
        self.right_url.setPlaceholderText("Right RTSP URL")
        self.apply_camera = QtWidgets.QPushButton("Apply camera URLs")
        self.apply_feedback = QtWidgets.QLabel("")
        self.apply_feedback.setWordWrap(True)
        self.apply_feedback.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.apply_camera.clicked.connect(self._apply_camera_settings)
        self.camera_section.add_row("Left", self.left_url)
        self.camera_section.add_row("Right", self.right_url)
        self.camera_section.add_row("", self.apply_camera)
        self.camera_section.add_row("Result", self.apply_feedback)
        root.addWidget(self.camera_section)

        self.context_section = PropertySection("Source-backed context")
        self.context_values: dict[str, EngineeringValue] = {}
        for key, label in (
            ("primary", "Primary"),
            ("secondary", "Secondary"),
            ("timing", "Timing"),
            ("contract", "Contract"),
        ):
            value = EngineeringValue(label, show_source=True, source_prefix="")
            self.context_values[key] = value
            self.context_section.add_row(label, value)
        root.addWidget(self.context_section)

        contract_section = PropertySection("Measurement contract")
        contract_value = QtWidgets.QLabel(f"{MEASUREMENT_FRAME}\naxis: forward / right / down\nunit: meters")
        contract_value.setWordWrap(True)
        contract_value.setToolTip(MEASUREMENT_AXIS_CONTRACT)
        contract_value.setMinimumHeight(42)
        contract_value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        contract_section.add_row("Public output", contract_value)
        root.addWidget(contract_section)

        open_legacy_button = QtWidgets.QPushButton("Open advanced owner tools")
        open_legacy_button.setToolTip(
            "Open the remaining low-level SDK, PX4, field-watch, and media tools."
        )
        open_legacy_button.clicked.connect(self.open_legacy)
        root.addWidget(open_legacy_button)
        root.addStretch(1)

    def _apply_camera_settings(self) -> None:
        method = getattr(self.backend, "apply_camera_settings", None)
        if not callable(method):
            self.apply_feedback.setText("ERROR: camera settings adapter unavailable")
            return
        try:
            applied = bool(method(self.left_url.text(), self.right_url.text()))
        except Exception as exc:  # pragma: no cover - Qt slot boundary.
            self.apply_feedback.setText(f"ERROR: {exc}")
            return
        self.apply_feedback.setText("Applied to existing panel settings" if applied else "ERROR: settings were not applied")

    def refresh(self, context_id: str, status_model: StatusModel) -> None:
        self._status_model = status_model
        self.context_label.setText(f"Selected context\n{context_id}")
        key_map = {
            "devices.cameras": "camera.left",
            "devices.cameras.left": "camera.left",
            "devices.cameras.right": "camera.right",
            "devices.cameras.stereo": "camera.left",
            "devices.pixhawk.p1": "pixhawk.left",
            "devices.pixhawk.p2": "pixhawk.right",
            "devices.pixhawk": "pixhawk.left",
            "devices.gnss": "gnss",
            "processing.detection": "detection",
            "processing.measurement": "measurement",
            "processing.synchronization": "synchronization",
            "tasks.recording": "recording",
            "diagnostics.camera": "camera.left",
            "diagnostics.pixhawk": "pixhawk.left",
            "diagnostics.gnss": "gnss",
            "diagnostics.performance": "fps",
        }
        status = status_model.record(key_map.get(context_id, "measurement"))
        self.status_value.set_value(
            status.state if status else "UNKNOWN",
            source=status.source if status else "status adapter",
            age_s=status.age_s if status else None,
            valid=(
                True
                if status is not None and status.severity.value == "ok"
                else False
                if status is not None and status.severity.value == "bad"
                else None
            ),
        )
        self.source_value.set_value(status.detail if status else "no evidence", source=status.source if status else "")
        panel = getattr(self.backend, "panel", self.backend)
        left_edit = getattr(panel, "left_url_edit", None)
        right_edit = getattr(panel, "right_url_edit", None)
        if left_edit is not None and not self.left_url.hasFocus():
            self.left_url.setText(left_edit.text())
        if right_edit is not None and not self.right_url.hasFocus():
            self.right_url.setText(right_edit.text())
        self._refresh_context_details(context_id)

    def _refresh_context_details(self, context_id: str) -> None:
        evidence = getattr(self.backend, "evidence", None)
        if not isinstance(evidence, EvidenceAdapter):
            evidence = EvidenceAdapter(getattr(self.backend, "panel", self.backend))
        values = {key: "--" for key in self.context_values}
        source = "read-only source projection"
        frame = ""
        if context_id in {"devices.pixhawk", "devices.pixhawk.p1", "devices.pixhawk.p2", "diagnostics.pixhawk"}:
            side = "right" if context_id.endswith("p2") else "left"
            attitude = evidence.attitude(side)
            values.update(
                primary=f"{side.upper()} {attitude.get('state', 'WAITING')} r/p/y="
                f"{attitude.get('roll_deg', '--')}/{attitude.get('pitch_deg', '--')}/{attitude.get('yaw_deg', '--')}°",
                secondary=f"topic={attitude.get('topic') or '--'}; PX4 t={attitude.get('px4_timestamp_s', '--')}",
                timing=f"age={attitude.get('age_s', '--')}s; clock={attitude.get('clock', '--')}",
                contract=str(attitude.get("orientation_contract", "PX4 body FRD → world NED")),
            )
            frame = "PX4 body FRD → world NED (orientation)"
            source = "CameraExtrinsicState"
        elif context_id in {"devices.gnss", "diagnostics.gnss"}:
            left = evidence.gps("left")
            right = evidence.gps("right")
            baseline = evidence.baseline()
            values.update(
                primary=f"P1/P2={left.get('rtk_label', 'NO DATA')}/{right.get('rtk_label', 'NO DATA')}",
                secondary=f"sats={left.get('satellites', '--')}/{right.get('satellites', '--')} "
                f"eph={left.get('eph_m', '--')}/{right.get('eph_m', '--')}m",
                timing=f"baseline age P1/P2={baseline.get('left_age_s', '--')}/{baseline.get('right_age_s', '--')}s",
                contract=f"N/E/D={baseline.get('north_m', '--')}/{baseline.get('east_m', '--')}/{baseline.get('down_m', '--')} m",
            )
            frame = "GPS WGS84 receiver fields; baseline NED north/east/down"
            source = "GpsPositionState + RtkBaselineEstimate"
        elif context_id in {"processing.detection", "devices.cameras", "devices.cameras.left", "devices.cameras.right", "devices.cameras.stereo", "diagnostics.camera"}:
            config = evidence.detection_config()
            target = evidence.detection("right" if context_id.endswith("right") else "left")
            values.update(
                primary=f"device={config.get('device', '--')} input={config.get('input_size_px', '--')}px",
                secondary=f"conf/IoU={config.get('confidence', '--')}/{config.get('iou', '--')} "
                f"gates={','.join(str(item) for item in config.get('gate_enabled', ()) or ()) or 'OFF'}",
                timing=f"target={target.get('runtime_state', 'NO TARGET')} age={target.get('age_s', '--')}s",
                contract="bbox x1/y1/x2/y2 and center_u/v are image pixels",
            )
            frame = "OpenCV image pixel u/v; no metric conversion"
            source = "AppSettings + DetectionObservation"
        elif context_id in {"tasks.recording", "tasks.playback", "tasks.export"}:
            recording = evidence.recording()
            counts = recording.get("sample_counts", {})
            values.update(
                primary=f"{recording.get('state', 'IDLE')} session={recording.get('session_id') or '--'}",
                secondary=f"pairs={counts.get('pairs', 0) if isinstance(counts, dict) else '--'}",
                timing=f"elapsed={recording.get('elapsed_s', '--')}s; retry={recording.get('retry_required', False)}",
                contract=str(recording.get("output_dir") or "output directory unavailable"),
            )
            source = "recording session/save worker"
        elif context_id == "tasks.advanced":
            values.update(
                primary="Advanced owner tools",
                secondary="WIRIS SDK / PX4 / field-watch / preview / media",
                timing="delegated to existing panel resource owners",
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
            )
            source = measurement.source if measurement else source
        for key, value in self.context_values.items():
            self.context_values[key].set_value(values[key], source=source, frame=frame if key == "contract" else "")

    def _status_for_context(self, context_id: str) -> RuntimeStatus | None:
        key_map = {
            "processing.measurement": "measurement",
            "processing.synchronization": "synchronization",
            "diagnostics.measurement": "measurement",
            "diagnostics.performance": "fps",
        }
        model = getattr(self, "_status_model", None)
        return model.record(key_map.get(context_id, "measurement")) if isinstance(model, StatusModel) else None
