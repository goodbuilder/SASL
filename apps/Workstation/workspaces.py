from __future__ import annotations

import math
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Callable, Sequence

from PyQt5 import QtCore, QtGui, QtWidgets

from .annotation_image_view import AnnotationImageView, AnnotationMode
from Workstation.workstation_core.coordinate_contract import (
    BASELINE_TRANSLATION_FRAME,
    BODY_FRAME,
    CAMERA_FRAME,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    WORLD_FRAME,
)
from Workstation.workstation_core.pixhawk_attitude import MAVLINK_MAG_STREAMS
from Workstation.workstation_core.px4_attitude_cal_manager import (
    board_rotation_label,
)
from Workstation.workstation_core.px4_calibration_groups import PX4_CALIBRATION_GROUPS
from Workstation.workstation_core.px4_mag_manager import (
    MAG_PRIORITY_LEVELS,
    mag_priority_display,
)
from Workstation.workstation_core.theme import (
    ACCENT,
    ERROR_TEXT,
    PANEL_BORDER,
    SUCCESS_TEXT,
)
from Workstation.workstation_core.stereo import DetectionObservation

from .components import EngineeringValue, StatusIndicator, WorkspaceHeader
from .evidence import EvidenceAdapter
from .protocol import evaluate_recording_protocol
from .status_model import RuntimeStatus, StatusModel, StatusSeverity


SESSION_CATALOG_REFRESH_INTERVAL_S = 5.0


def _status(model: StatusModel, key: str) -> RuntimeStatus | None:
    return model.record(key)


def _evidence(backend: object) -> EvidenceAdapter:
    adapter = getattr(backend, "evidence", None)
    if isinstance(adapter, EvidenceAdapter):
        return adapter
    return EvidenceAdapter(getattr(backend, "panel", backend))


def _display_number(value: object, *, digits: int = 3, suffix: str = "") -> str:
    if value in (None, ""):
        return "--"
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except (TypeError, ValueError):
        return f"{value}{suffix}"


def _compact_text(value: object, *, limit: int = 128) -> str:
    text = " ".join(str(value or "").split())
    if len(text) <= limit:
        return text
    return text[: max(1, limit - 1)].rstrip() + "…"


def _display_class(target: dict[str, object]) -> str:
    class_name = target.get("class_name")
    if class_name not in (None, ""):
        return str(class_name)
    class_id = target.get("class_id")
    return "--" if class_id is None else str(class_id)


def _table_model(parent: QtCore.QObject, headers: tuple[str, ...]) -> QtGui.QStandardItemModel:
    model = QtGui.QStandardItemModel(0, len(headers), parent)
    model.setHorizontalHeaderLabels(list(headers))
    return model


def _append_table_row(model: QtGui.QStandardItemModel, values: tuple[object, ...]) -> None:
    items = []
    for value in values:
        item = QtGui.QStandardItem(str(value))
        item.setEditable(False)
        items.append(item)
    model.appendRow(items)


def _set_px4_priority_combo_value(combo: QtWidgets.QComboBox, value: int | float | None) -> None:
    if value is None:
        return
    raw = int(value)
    index = combo.findData(raw)
    if index < 0:
        combo.addItem(f"Custom ({raw})", raw)
        index = combo.count() - 1
    combo.setCurrentIndex(index)
    combo.setToolTip(f"PX4 priority: {mag_priority_display(raw)}")


def _build_px4_priority_combo(value: int) -> QtWidgets.QComboBox:
    combo = QtWidgets.QComboBox()
    for raw, label in MAG_PRIORITY_LEVELS:
        combo.addItem(label, raw)
    _set_px4_priority_combo_value(combo, value)
    combo.currentIndexChanged.connect(
        lambda _index, combo=combo: combo.setToolTip(
            f"PX4 priority: {mag_priority_display(combo.currentData())}"
        )
    )
    return combo


def _px4_priority_combo_value(combo: QtWidgets.QComboBox) -> int:
    value = combo.currentData()
    return 0 if value is None else int(value)


def _format_px4_parameter_value(name: str, value: object) -> str:
    if value in (None, ""):
        return "N/A"
    if name.endswith("_PRIO"):
        return mag_priority_display(value)  # type: ignore[arg-type]
    if name.endswith("_ROT"):
        raw = int(value)
        return f"{board_rotation_label(raw)} ({raw})"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def _sync_table_rows(
    model: QtGui.QStandardItemModel,
    rows: tuple[tuple[object, ...], ...],
) -> None:
    """Update table cells in place so polling does not discard selection."""

    if model.rowCount() != len(rows):
        model.setRowCount(len(rows))
    for row_index, values in enumerate(rows):
        for column_index, value in enumerate(values):
            text = str(value)
            item = model.item(row_index, column_index)
            if item is None:
                item = QtGui.QStandardItem(text)
                item.setEditable(False)
                model.setItem(row_index, column_index, item)
            elif item.text() != text:
                item.setText(text)


def _format_drone_rtk_sync_delta_ms(value: object) -> str:
    if value is None:
        return "--"
    try:
        return f"{float(value) * 1000.0:+.0f} ms"
    except (TypeError, ValueError):
        return "--"


def _format_drone_rtk_degrees(value: object) -> str:
    if value is None:
        return "--"
    try:
        # Display-only rounding; the stored observation float is untouched.
        return f"{float(value):.9f}°"
    except (TypeError, ValueError):
        return "--"


def _format_drone_rtk_metres(value: object) -> str:
    if value is None:
        return "--"
    try:
        return f"{float(value):.3f} m"
    except (TypeError, ValueError):
        return "--"


def _format_drone_rtk_seconds(value: object) -> str:
    if value is None:
        return "--"
    try:
        return f"{float(value):.3f} s"
    except (TypeError, ValueError):
        return "--"


def _drone_rtk_gnss_label(row: object) -> str:
    """"NO DATA" unless this GNSS row actually carries a position fix."""

    if not isinstance(row, Mapping) or not row.get("has_position"):
        return "NO DATA"
    return str(row.get("rtk_label") or "UNKNOWN")


def _balance_preview_pair(splitter: QtWidgets.QSplitter) -> None:
    """Give the two camera previews equal room before sizing them by aspect."""

    if splitter.width() <= 0 or splitter.height() <= 0:
        return
    if splitter.orientation() == QtCore.Qt.Horizontal:
        available = max(2, splitter.width() - splitter.handleWidth())
    else:
        available = max(2, splitter.height() - splitter.handleWidth())
    first = available // 2
    sizes = [first, available - first]
    if splitter.sizes() != sizes:
        splitter.setSizes(sizes)


def _preview_pair_height(
    splitter: QtWidgets.QSplitter,
    previews: Mapping[str, "CameraPreview"],
) -> int:
    """Return the height needed to show both previews without letterboxing."""

    _balance_preview_pair(splitter)
    heights = [
        max(0, preview.heightForWidth(max(1, preview.width())))
        for preview in previews.values()
    ]
    if not heights:
        return 0
    if splitter.orientation() == QtCore.Qt.Horizontal:
        return max(heights)
    return sum(heights) + splitter.handleWidth() * max(0, len(heights) - 1)


def _command_button(
    text: str,
    command_id: str,
    invoke_command: Callable[[str], bool] | None,
) -> QtWidgets.QPushButton:
    button = QtWidgets.QPushButton(text)
    button.setProperty("commandId", command_id)
    if invoke_command is None:
        button.setEnabled(False)
    else:
        button.clicked.connect(lambda _checked=False, command=command_id: invoke_command(command))
    return button


def _live_inspection(
    snapshot: dict[str, object],
    resolver: Callable[[], dict[str, object]],
) -> dict[str, object]:
    snapshot["_resolver"] = resolver
    return snapshot


class OverviewWorkspace(QtWidgets.QWidget):
    open_context = QtCore.pyqtSignal(str)

    def __init__(self, backend: object, status_model: StatusModel, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Overview", "Connect → verify → measure → record"))

        status_group = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QGridLayout(status_group)
        status_layout.setContentsMargins(6, 6, 6, 6)
        status_layout.setSpacing(5)
        self.indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (
                ("camera.left", "CAM L"),
                ("camera.right", "CAM R"),
                ("pixhawk.left", "P1"),
                ("pixhawk.right", "P2"),
                ("gnss.base", "BASE"),
                ("detection", "DET"),
                ("measurement", "MEAS"),
                ("recording", "REC"),
            )
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator, index // 2, index % 2)
        layout.addWidget(status_group)

        evidence_group = QtWidgets.QGroupBox("Latest result")
        evidence_layout = QtWidgets.QGridLayout(evidence_group)
        evidence_layout.setContentsMargins(8, 8, 8, 8)
        self.session_value = EngineeringValue("Session")
        self.measurement_value = EngineeringValue("Result")
        self.contract_value = EngineeringValue("Output")
        evidence_layout.addWidget(self.session_value, 0, 0)
        evidence_layout.addWidget(self.measurement_value, 1, 0)
        evidence_layout.addWidget(self.contract_value, 2, 0)
        evidence_layout.setColumnStretch(0, 1)
        layout.addWidget(evidence_group)

        actions = QtWidgets.QHBoxLayout()
        camera_button = QtWidgets.QPushButton("Connect cameras")
        camera_button.clicked.connect(lambda: self.open_context.emit("devices.cameras"))
        measurement_button = QtWidgets.QPushButton("Measure")
        measurement_button.clicked.connect(lambda: self.open_context.emit("processing.measurement"))
        diagnostics_button = QtWidgets.QPushButton("Diagnostics")
        diagnostics_button.clicked.connect(lambda: self.open_context.emit("diagnostics.system"))
        actions.addWidget(camera_button)
        actions.addWidget(measurement_button)
        actions.addWidget(diagnostics_button)
        actions.addStretch(1)
        layout.addLayout(actions)
        layout.addStretch(1)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        panel = getattr(self.backend, "panel", self.backend)
        session = str(getattr(panel, "recording_session_id", "") or "") or "--"
        self.session_value.set_value(session, source="recording session metadata")
        measurement_status = snapshot.get("measurement")
        self.measurement_value.set_value(
            str(getattr(panel, "last_distance_status", "Waiting for measurement") or ""),
            source="MeasurementAttempt/controller result",
            frame=MEASUREMENT_FRAME,
            age_s=measurement_status.age_s if measurement_status else None,
            valid=str(getattr(panel, "last_distance_quality", "WARNING")).upper() in {"GOOD", "VALID"},
        )
        self.contract_value.set_value(
            f"{MEASUREMENT_AXIS_CONTRACT} · metres",
            frame=MEASUREMENT_FRAME,
            source="coordinate_contract.py",
        )


class ExperimentWorkspace(QtWidgets.QWidget):
    """Guided view of the field protocol using existing owner evidence."""

    open_context = QtCore.pyqtSignal(str)

    def __init__(self, backend: object, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Protocol",
                "0 external prerequisite → 1–11 field protocol · evidence follows the operator order",
            )
        )
        prerequisite_row = QtWidgets.QHBoxLayout()
        note = QtWidgets.QLabel(
            "QGC Pixhawk Sensor Calibration is completed outside Workstation. "
            "Camera intrinsic/stereo calibration remains available in Calibration before the run."
        )
        note.setObjectName("compactContract")
        note.setWordWrap(True)
        prerequisite_row.addWidget(note, 1)
        self.pre_run_calibration_button = QtWidgets.QPushButton("Open pre-run calibration")
        self.pre_run_calibration_button.clicked.connect(
            lambda _checked=False: self.open_context.emit("tasks.calibration")
        )
        prerequisite_row.addWidget(self.pre_run_calibration_button)
        layout.addLayout(prerequisite_row)

        protocol_group = QtWidgets.QGroupBox("Protocol")
        protocol_layout = QtWidgets.QGridLayout(protocol_group)
        protocol_layout.setColumnStretch(1, 1)
        protocol_layout.setColumnStretch(2, 3)
        protocol_layout.addWidget(QtWidgets.QLabel("Step"), 0, 0)
        protocol_layout.addWidget(QtWidgets.QLabel("State"), 0, 1)
        protocol_layout.addWidget(QtWidgets.QLabel("Evidence / action"), 0, 2)
        protocol_layout.addWidget(QtWidgets.QLabel("Open"), 0, 3)
        self.step_states: dict[int, QtWidgets.QLabel] = {}
        self.step_details: dict[int, QtWidgets.QLabel] = {}
        self.step_buttons: dict[int, QtWidgets.QPushButton] = {}
        self.step_action_buttons: dict[int, tuple[QtWidgets.QPushButton, ...]] = {}
        steps = (
            (0, "QGC Pixhawk calibration", "External", None),
            (
                1,
                "Connect and verify WIRIS / Pixhawk / Base GNSS",
                "",
                (
                    ("Cameras", "devices.cameras"),
                    ("Pixhawk", "devices.pixhawk"),
                    ("Base", "devices.gnss"),
                ),
            ),
            (2, "Start Base Survey-in or NTRIP", "", "devices.gnss"),
            (3, "Initial WIRIS time synchronization", "", "devices.time"),
            (
                4,
                "RTCM injection and numeric fix_type",
                "",
                (("Base", "devices.gnss"), ("Pixhawk", "devices.pixhawk")),
            ),
            (5, "Physical baseline alignment", "", "processing.alignment"),
            (6, "Physical Roll / Pitch / Yaw alignment", "", "processing.alignment"),
            (7, "Repeat physical alignment evidence", "", "processing.alignment"),
            (8, "Final time-sync quality check / resync", "", "devices.time"),
            (9, "YOLO and measurement settings", "", "processing.measurement"),
            (10, "Start measurement", "", "processing.measurement"),
            (11, "Save measured data", "", "tasks.recording"),
        )
        for row, (number, label, initial_state, context) in enumerate(steps, start=1):
            protocol_layout.addWidget(QtWidgets.QLabel(f"{number}. {label}"), row, 0)
            state = QtWidgets.QLabel(initial_state or "WAIT")
            state.setObjectName("protocolState")
            self.step_states[number] = state
            protocol_layout.addWidget(state, row, 1)
            detail = QtWidgets.QLabel("--")
            detail.setObjectName("protocolCheck")
            detail.setWordWrap(True)
            self.step_details[number] = detail
            protocol_layout.addWidget(detail, row, 2)
            if context is None:
                button = QtWidgets.QPushButton("External")
                button.setEnabled(False)
                action_widget: QtWidgets.QWidget = button
                action_buttons = (button,)
            elif isinstance(context, tuple):
                action_widget = QtWidgets.QWidget()
                action_layout = QtWidgets.QHBoxLayout(action_widget)
                action_layout.setContentsMargins(0, 0, 0, 0)
                action_layout.setSpacing(4)
                buttons: list[QtWidgets.QPushButton] = []
                for button_label, target in context:
                    action_button = QtWidgets.QPushButton(button_label)
                    action_button.clicked.connect(
                        lambda _checked=False, target=target: self.open_context.emit(target)
                    )
                    action_layout.addWidget(action_button)
                    buttons.append(action_button)
                action_buttons = tuple(buttons)
                button = action_buttons[0]
            else:
                button = QtWidgets.QPushButton("Open")
                button.clicked.connect(lambda _checked=False, target=context: self.open_context.emit(target))
                action_widget = button
                action_buttons = (button,)
            self.step_buttons[number] = button
            self.step_action_buttons[number] = action_buttons
            protocol_layout.addWidget(action_widget, row, 3)
        layout.addWidget(protocol_group)
        contract = QtWidgets.QLabel(
            "Fix evidence is numeric PX4/MAVLink fix_type; RTK_FIXED=6 is reported as a ratio and is not assumed for every run. "
            "Raw baseline remains NED [north,east,down]; alignment view is P1 body FRD [forward,right,down]."
        )
        contract.setObjectName("compactContract")
        contract.setWordWrap(True)
        layout.addWidget(contract)
        layout.addStretch(1)

    @staticmethod
    def _state(status: RuntimeStatus | None) -> str:
        return "UNKNOWN" if status is None else str(status.state or "UNKNOWN").upper()

    def _set_step(self, number: int, state: str, detail: str) -> None:
        self.step_states[number].setText(state)
        self.step_details[number].setText(detail)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        camera_states = (self._state(snapshot.get("camera.left")), self._state(snapshot.get("camera.right")))
        pixhawk_states = (self._state(snapshot.get("pixhawk.left")), self._state(snapshot.get("pixhawk.right")))
        gps = {side: evidence.gps(side) for side in ("left", "right")}
        gnss = evidence.gnss_operation()
        routes = gnss.get("routes") if isinstance(gnss.get("routes"), dict) else {}
        physical = evidence.physical_alignment()
        quality = physical.get("quality") if isinstance(physical.get("quality"), dict) else {}
        calibration = evidence.calibration()
        sync_state = self._state(snapshot.get("synchronization"))
        checkpoints = getattr(getattr(self.backend, "panel", self.backend), "workstation_time_sync_checkpoints", ())
        final_sync = any(
            isinstance(item, dict)
            and str(item.get("phase", "")) == "final"
            and "VERIFIED" in str(item.get("status", "")).upper()
            for item in checkpoints or ()
        )

        connected = all(state in {"LIVE", "CONNECTED", "STARTING"} for state in (*camera_states, *pixhawk_states))
        gps_present = all(data.get("fix_type") is not None for data in gps.values())
        mode = str(gnss.get("mode", "UNKNOWN") or "UNKNOWN")
        base_serial = gnss.get("base_serial") if isinstance(gnss.get("base_serial"), dict) else {}
        ntrip = gnss.get("ntrip") if isinstance(gnss.get("ntrip"), dict) else {}
        if mode in {"LOCAL_SURVEY_IN", "NETWORK_ASSISTED_FIXED_BASE"}:
            base_link_ready = bool(base_serial.get("connected", False))
            base_link_text = "CONNECTED" if base_link_ready else "DISCONNECTED"
        elif mode == "NETWORK_RTK_DIRECT":
            base_link_ready = bool(ntrip.get("caster_host") and ntrip.get("mountpoint"))
            base_link_text = "NTRIP CONFIGURED" if base_link_ready else "NTRIP NOT CONFIGURED"
        else:
            base_link_ready = False
            base_link_text = f"MODE {mode}"
        self._set_step(
            1,
            "READY" if connected and gps_present and base_link_ready else "WAIT",
            f"CAM L/R={camera_states[0]}/{camera_states[1]} · P1/P2={pixhawk_states[0]}/{pixhawk_states[1]} · "
            f"BASE={base_link_text} · GPS fix_type={gps['left'].get('fix_type', '--')}/{gps['right'].get('fix_type', '--')}",
        )
        base_frames = int(gnss.get("base_frames") or 0)
        operation_state = str(gnss.get("state", "UNKNOWN") or "UNKNOWN")
        survey_or_ntrip = operation_state in {
            "BASE_SURVEYING",
            "NTRIP_ACTIVE",
            "LOCAL_RTCM_ACTIVE",
            "ROVERS_CONVERGING",
            "READY",
            "MEASURING",
        }
        self._set_step(2, "READY" if survey_or_ntrip or base_frames > 0 else "WAIT", f"operation={operation_state} · base RTCM frames={base_frames}")
        self._set_step(3, "READY" if sync_state == "SYNCED" else "WAIT", f"RTCP/NTP state={sync_state}")
        route_ready = all(
            isinstance(routes.get(side), dict)
            and int(routes[side].get("packets") or 0) > 0
            for side in ("left", "right")
        )
        self._set_step(
            4,
            "READY" if route_ready and gps_present else "WAIT",
            f"RTCM route L/R={'OK' if route_ready else 'WAIT'} · numeric fix_type="
            f"{gps['left'].get('fix_type', '--')}/{gps['right'].get('fix_type', '--')} · RTK_FIXED ratio="
            f"{float(quality.get('fixed_ratio', 0.0)) * 100.0:.1f}%" if quality.get("fixed_ratio") not in (None, "") else
            f"RTCM route L/R={'OK' if route_ready else 'WAIT'} · numeric fix_type="
            f"{gps['left'].get('fix_type', '--')}/{gps['right'].get('fix_type', '--')}",
        )
        alignment_state = str(physical.get("state", "WAITING") or "WAITING")
        relative = physical.get("relative_rpy_deg")
        relative_text = "R/P/Y=--"
        if isinstance(relative, (tuple, list)) and len(relative) == 3:
            relative_text = "R/P/Y=" + "/".join(f"{float(value):+.2f}°" for value in relative)
        sampling_active = bool(physical.get("sampling_active"))
        quality_reasons = {
            str(reason)
            for reason in (quality.get("reasons") or ())
        }
        fix_reasons = {"fix_type_missing", "fix_type_below_minimum", "fixed_ratio_below_minimum"}
        fix_ok = not bool(quality_reasons & fix_reasons)
        baseline_error = quality.get("baseline_error_m")
        baseline_evaluated = isinstance(baseline_error, (tuple, list)) and len(baseline_error) == 3
        baseline_ok = baseline_evaluated and fix_ok and "baseline_frd_out_of_tolerance" not in quality_reasons
        rpy_error = quality.get("relative_rpy_error_deg")
        rpy_evaluated = isinstance(rpy_error, (tuple, list)) and len(rpy_error) == 3
        rpy_ok = rpy_evaluated and fix_ok and "relative_rpy_out_of_tolerance" not in quality_reasons

        def vector_text(value: object, *, suffix: str) -> str:
            if not isinstance(value, (tuple, list)) or len(value) != 3:
                return "--"
            return "/".join(f"{float(item):+.3f}{suffix}" for item in value)

        samples = physical.get("sample_count", quality.get("sample_count", "--"))
        self._set_step(
            5,
            "SAMPLING" if sampling_active else "READY" if baseline_ok else "CHECK" if baseline_evaluated else "WAIT",
            f"P1→P2 FRD error={vector_text(baseline_error, suffix='m')} · samples={samples}",
        )
        self._set_step(
            6,
            "SAMPLING" if sampling_active else "READY" if rpy_ok else "CHECK" if rpy_evaluated else "WAIT",
            f"{relative_text} · error={vector_text(rpy_error, suffix='°')} · samples={samples}",
        )
        iterations = physical.get("iterations")
        iteration_count = len(iterations) if isinstance(iterations, (tuple, list)) else 0
        repeat_ready = iteration_count >= 2 and baseline_ok and rpy_ok
        self._set_step(
            7,
            "SAMPLING" if sampling_active else "READY" if repeat_ready else "CHECK" if iteration_count else "WAIT",
            f"completed iterations={iteration_count}/2 · latest state={alignment_state}",
        )
        self._set_step(8, "READY" if final_sync else "CHECK", f"current={sync_state} · final checkpoint={'recorded' if final_sync else 'not recorded'}")
        calibration_ready = bool(calibration.get("active"))
        measurement_config = evidence.measurement_settings()
        self._set_step(9, "READY" if calibration_ready else "WAIT", f"active calibration={'yes' if calibration_ready else 'no'} · mode={measurement_config.get('distance_mode', '--')}")
        measurement_state = self._state(snapshot.get("measurement"))
        detection_state = self._state(snapshot.get("detection"))
        if measurement_state == "VALID":
            run_state = "VALID"
        elif detection_state in {"RUNNING", "STARTING"}:
            run_state = "RUNNING"
        else:
            run_state = "WAIT"
        self._set_step(
            10,
            run_state,
            f"detection={detection_state} · measurement result={measurement_state}",
        )
        panel = getattr(self.backend, "panel", self.backend)
        has_data = bool(getattr(panel, "recording_session_id", "")) or bool(getattr(panel, "distance_history", ()))
        recording_state = self._state(snapshot.get("recording"))
        if recording_state == "SAVED":
            save_state, save_detail = "SAVED", "session artifacts finalized"
        elif recording_state == "FINALIZING":
            save_state, save_detail = "SAVING", "session save worker running"
        elif recording_state == "RECOVERABLE":
            save_state, save_detail = "RETRY", "retryable spool preserved; save again"
        elif has_data:
            save_state, save_detail = "SAVE", "unsaved session data available"
        else:
            save_state, save_detail = "WAIT", "save becomes available after a measurement"
        self._set_step(11, save_state, save_detail)


class CameraPreview(QtWidgets.QFrame):
    IMAGE_ASPECT_RATIO = 16.0 / 9.0

    def __init__(self, side: str, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.side = side
        self.setObjectName("cameraPreview")
        self.setMinimumSize(0, 0)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(3)
        title = QtWidgets.QLabel("Left camera" if side == "left" else "Right camera")
        title.setObjectName("cameraPreviewTitle")
        self.image = QtWidgets.QLabel("NO VIDEO")
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.image.setMinimumSize(0, 0)
        self.image.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        self.image.setObjectName("cameraPreviewImage")
        self.meta = QtWidgets.QLabel("WAITING")
        self.meta.setObjectName("cameraPreviewMeta")
        self.meta.setWordWrap(True)
        self.meta.setMinimumWidth(0)
        self.meta.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        layout.addWidget(title)
        layout.addWidget(self.image, 1)
        layout.addWidget(self.meta)
        self._image: QtGui.QImage | None = None
        self._detection: DetectionObservation | None = None
        self._metadata: object = {}

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt override.
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt override.
        content_width = max(0, int(width) - self.layout().contentsMargins().left() - self.layout().contentsMargins().right())
        image_height = int(round(content_width / self.IMAGE_ASPECT_RATIO))
        chrome_height = sum(
            widget.sizeHint().height()
            for widget in (self.findChild(QtWidgets.QLabel, "cameraPreviewTitle"), self.meta)
        )
        return image_height + chrome_height + self.layout().spacing() * 2 + self.layout().contentsMargins().top() + self.layout().contentsMargins().bottom()

    def set_frame(self, image: QtGui.QImage, metadata: object) -> None:
        self._image = image
        self._render()
        self._set_metadata(metadata, live=True)

    def clear(self) -> None:
        """Clear the displayed frame when the camera Preview is stopped."""

        self._image = None
        self._detection = None
        self._metadata = {}
        self._render()
        self._set_metadata({}, live=False)

    def set_detection(self, detection: DetectionObservation | None) -> None:
        """Keep the latest image-space bbox for this display preview.

        Detection coordinates remain in the source image pixel contract
        (x1/y1/x2/y2, or u/v).  Only the copy rendered into the widget is
        scaled for presentation; no camera/world-frame conversion occurs.
        """

        self._detection = detection
        self._render()
        self._set_metadata(
            self._metadata,
            live=self._image is not None and not self._image.isNull(),
        )

    def set_metadata(self, metadata: object) -> None:
        live = self._image is not None and not self._image.isNull()
        self._set_metadata(metadata, live=live)

    def _set_metadata(self, metadata: object, *, live: bool) -> None:
        self._metadata = metadata
        detail = self._metadata_text(metadata, live=live)
        self.meta.setText(detail)
        self.meta.setToolTip(detail)

    def _metadata_text(self, metadata: object, *, live: bool) -> str:
        parts = ["LIVE" if live else "WAITING"]
        if self._detection is None:
            parts.append("Class: --")
        else:
            class_name = self._detection.class_name or f"#{self._detection.class_id}"
            parts.append(f"Class: {class_name}")
        return " | ".join(parts)

    def _render(self) -> None:
        if self._image is None or self._image.isNull():
            self.image.setText("NO VIDEO")
            self.image.setPixmap(QtGui.QPixmap())
            return
        target_size = self.image.size()
        source_pixmap = QtGui.QPixmap.fromImage(self._image)
        scaled = source_pixmap.scaled(
            target_size, QtCore.Qt.KeepAspectRatioByExpanding, QtCore.Qt.SmoothTransformation
        )
        crop_x = max(0, (scaled.width() - target_size.width()) // 2)
        crop_y = max(0, (scaled.height() - target_size.height()) // 2)
        pixmap = scaled.copy(crop_x, crop_y, target_size.width(), target_size.height())
        if self._detection is not None:
            pixmap = self._draw_detection_overlay(
                pixmap,
                self._detection,
                scale_x=scaled.width() / source_pixmap.width(),
                scale_y=scaled.height() / source_pixmap.height(),
                crop_x=crop_x,
                crop_y=crop_y,
            )
        self.image.setPixmap(pixmap)

    @staticmethod
    def _draw_detection_overlay(
        pixmap: QtGui.QPixmap,
        detection: DetectionObservation,
        *,
        scale_x: float | None = None,
        scale_y: float | None = None,
        crop_x: int = 0,
        crop_y: int = 0,
    ) -> QtGui.QPixmap:
        """Draw a bbox and circular center marker on the display pixmap."""

        if detection.width <= 0 or detection.height <= 0:
            return pixmap
        if scale_x is None:
            scale_x = pixmap.width() / detection.width
        if scale_y is None:
            scale_y = pixmap.height() / detection.height
        painter = QtGui.QPainter(pixmap)
        try:
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            color = QtGui.QColor(ACCENT)
            rect = QtCore.QRectF(
                detection.x1 * scale_x - crop_x,
                detection.y1 * scale_y - crop_y,
                (detection.x2 - detection.x1) * scale_x,
                (detection.y2 - detection.y1) * scale_y,
            )
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawRect(rect)
            center_x, center_y = detection.center
            center = QtCore.QPointF(
                center_x * scale_x - crop_x,
                center_y * scale_y - crop_y,
            )
            marker_radius = max(3.0, min(6.0, min(pixmap.width(), pixmap.height()) * 0.012))
            painter.setBrush(QtCore.Qt.NoBrush)
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawEllipse(center, marker_radius, marker_radius)
            label = detection.class_name or f"#{detection.class_id}"
            painter.drawText(
                QtCore.QPointF(rect.left(), max(10.0, rect.top() - 4.0)),
                f"{label} {detection.confidence:.2f}",
            )
        finally:
            painter.end()
        return pixmap

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        self._render()
        super().resizeEvent(event)


class CameraPreviewPair(QtWidgets.QSplitter):
    """Aspect-aware pair container for the calibration camera previews."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(QtCore.Qt.Horizontal, parent)
        self.setChildrenCollapsible(False)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt override.
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt override.
        previews = [self.widget(index) for index in range(self.count())]
        if not previews:
            return 0
        if self.orientation() == QtCore.Qt.Horizontal:
            available_width = max(0, int(width) - self.handleWidth())
            first_width = available_width // len(previews)
            widths = [first_width] * len(previews)
            widths[-1] += available_width - sum(widths)
            heights = [
                int(preview.heightForWidth(max(0, preview_width)))
                if callable(getattr(preview, "heightForWidth", None))
                else 0
                for preview, preview_width in zip(previews, widths)
            ]
            return max(heights, default=0)
        preview_height = max(0, int(width))
        return sum(
            int(preview.heightForWidth(preview_height))
            if callable(getattr(preview, "heightForWidth", None))
            else 0
            for preview in previews
        ) + self.handleWidth() * max(0, len(previews) - 1)

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt override.
        hint = super().sizeHint()
        if hint.width() > 0:
            hint.setHeight(self.heightForWidth(hint.width()))
        return hint

    def setOrientation(self, orientation: QtCore.Qt.Orientation) -> None:  # noqa: N802 - Qt override.
        super().setOrientation(orientation)
        self.updateGeometry()


class CameraWorkspace(QtWidgets.QWidget):
    inspection_requested = QtCore.pyqtSignal(object)
    CAMERA_ASPECT_RATIO = 16.0 / 9.0
    # A bottom Diagnostics dock can reduce the central workspace to a
    # landscape rectangle.  Once that dock is hidden, the same workspace can
    # become tall enough that side-by-side 16:9 panes waste most of the
    # available height.  In that case the camera pair moves to a vertical
    # stack and the details pane moves alongside it.
    TALL_LAYOUT_RATIO = 1.45

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.setMinimumWidth(0)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Cameras"))

        # Keep the live pair usable even when the camera workspace also shows
        # discovery, detection, and mapping controls.  Those controls have
        # sizeable layout hints and previously squeezed this splitter down to
        # a narrow strip on typical workstation displays.
        self.workspace_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.workspace_splitter.setChildrenCollapsible(False)
        self.workspace_splitter.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        self.preview_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.preview_splitter.setChildrenCollapsible(False)
        self.preview_splitter.setMinimumHeight(320)
        self.preview_splitter.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        self.previews = {side: CameraPreview(side) for side in ("left", "right")}
        self.preview_splitter.addWidget(self.previews["left"])
        self.preview_splitter.addWidget(self.previews["right"])
        self.preview_splitter.setStretchFactor(0, 1)
        self.preview_splitter.setStretchFactor(1, 1)
        self.workspace_splitter.addWidget(self.preview_splitter)

        details = QtWidgets.QWidget()
        details_layout = QtWidgets.QVBoxLayout(details)
        details_layout.setContentsMargins(0, 0, 0, 0)
        details_layout.setSpacing(6)
        facts = QtWidgets.QGroupBox("Status")
        facts_layout = QtWidgets.QGridLayout(facts)
        self.camera_values = {}
        for col, (key, label) in enumerate((("camera.left", "CAM L"), ("camera.right", "CAM R"), ("fps", "Decoded FPS"), ("detection", "Detection"))):
            value = EngineeringValue(label)
            self.camera_values[key] = value
            facts_layout.addWidget(value, 0, col)
        details_layout.addWidget(facts)

        detection_group = QtWidgets.QGroupBox("Detection")
        detection_layout = QtWidgets.QVBoxLayout(detection_group)
        detection_layout.setContentsMargins(6, 6, 6, 6)
        config_layout = QtWidgets.QGridLayout()
        config_layout.setContentsMargins(0, 0, 0, 0)
        self.detection_config_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("weights_path", "Weights"),
                ("device", "Device"),
                ("input_size_px", "Input"),
                ("confidence", "Conf"),
                ("iou", "IoU"),
                ("gate_enabled", "Gates"),
            )
        ):
            value = EngineeringValue(label)
            self.detection_config_values[key] = value
            config_layout.addWidget(value, index // 3, index % 3)
        detection_layout.addLayout(config_layout)
        self.target_model = _table_model(
            self,
            ("Camera", "State", "Class", "Conf", "bbox_px [x1,y1,x2,y2]", "Detection t", "Age"),
        )
        self.target_table = QtWidgets.QTableView()
        self.target_table.setModel(self.target_model)
        self.target_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.target_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.target_table.verticalHeader().setVisible(False)
        self.target_table.setMaximumHeight(118)
        self.target_table.horizontalHeader().setStretchLastSection(True)
        detection_layout.addWidget(self.target_table)
        details_layout.addWidget(detection_group)

        operations_group = QtWidgets.QGroupBox("Connect")
        operations_layout = QtWidgets.QGridLayout(operations_group)
        operations_layout.setContentsMargins(6, 6, 6, 6)
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for index, (command_id, label) in enumerate(
            (
                ("camera.discover_host_ip", "Find network"),
                ("camera.toggle", "Preview"),
                ("camera.auto_connect", "Connect cameras"),
                ("camera.check_control", "Check control"),
                ("camera.enable_rtsp", "Enable stream"),
                ("camera.check_rtsp", "Check video"),
            )
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            operations_layout.addWidget(button, index // 3, index % 3)
        details_layout.addWidget(operations_group)

        candidates_group = QtWidgets.QGroupBox("Discovered endpoints")
        candidates_layout = QtWidgets.QVBoxLayout(candidates_group)
        self.discovery_status = EngineeringValue("Discovery")
        candidates_layout.addWidget(self.discovery_status)
        self.candidate_model = _table_model(
            self,
            ("Kind", "Address", "Port", "Status", "Interface", "Host IP"),
        )
        self.candidate_table = QtWidgets.QTableView()
        self.candidate_table.setModel(self.candidate_model)
        self.candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.candidate_table.verticalHeader().setVisible(False)
        self.candidate_table.setMaximumHeight(120)
        self.candidate_table.horizontalHeader().setStretchLastSection(True)
        self.candidate_table.selectionModel().selectionChanged.connect(self._refresh_candidate_actions)
        self.candidate_table.clicked.connect(self._inspect_candidate_row)
        candidates_layout.addWidget(self.candidate_table)
        candidate_actions = QtWidgets.QHBoxLayout()
        self.candidate_buttons: dict[str, QtWidgets.QPushButton] = {}
        for target, label in (("nic", "Use host interface"), ("left", "Assign CAM L"), ("right", "Assign CAM R")):
            button = QtWidgets.QPushButton(label)
            button.setEnabled(False)
            button.clicked.connect(lambda _checked=False, target=target: self._apply_candidate(target))
            self.candidate_buttons[target] = button
            candidate_actions.addWidget(button)
        candidate_actions.addStretch(1)
        candidates_layout.addLayout(candidate_actions)
        self.candidate_feedback = QtWidgets.QLabel("")
        self.candidate_feedback.setObjectName("statusStrip")
        self.candidate_feedback.setWordWrap(True)
        candidates_layout.addWidget(self.candidate_feedback)
        details_layout.addWidget(candidates_group)
        self._candidate_signature: tuple[tuple[object, ...], ...] = ()
        self._recording_locked = False

        mapping_group = QtWidgets.QGroupBox("Mapping")
        mapping_layout = QtWidgets.QHBoxLayout(mapping_group)
        mapping_layout.setContentsMargins(6, 6, 6, 6)
        for command_id, label, tooltip in (
            ("camera.swap", "Swap L/R", "Swap the current Left/Right camera mapping."),
            ("camera.save_settings", "Save", "Save the current camera settings."),
            (
                "camera.reactivate_left",
                "Reactivate CAM L",
                "Safely send SETH TRUE for the left camera, verify frames, and reconnect its preview.",
            ),
            (
                "camera.reactivate_right",
                "Reactivate CAM R",
                "Safely send SETH TRUE for the right camera, verify frames, and reconnect its preview.",
            ),
            ("advanced.open", "Advanced", "Open Workstation advanced diagnostics and tools."),
        ):
            button = _command_button(label, command_id, invoke_command)
            button.setToolTip(tooltip)
            self.operation_buttons[command_id] = button
            mapping_layout.addWidget(button)
        mapping_layout.addStretch(1)
        details_layout.addWidget(mapping_group)
        details_layout.addStretch(1)

        self.details_scroll = QtWidgets.QScrollArea()
        self.details_scroll.setWidgetResizable(True)
        self.details_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.details_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.details_scroll.setWidget(details)
        self.details_scroll.setMinimumHeight(120)
        self.workspace_splitter.addWidget(self.details_scroll)
        self.workspace_splitter.setStretchFactor(0, 3)
        self.workspace_splitter.setStretchFactor(1, 1)
        self.workspace_splitter.setSizes([600, 240])
        layout.addWidget(self.workspace_splitter, 1)
        self._connect_panes()

    @classmethod
    def preview_pair_orientation(cls, width: int, height: int) -> QtCore.Qt.Orientation:
        """Choose the pair layout that gives two 16:9 frames more usable area."""

        if width <= 0 or height <= 0:
            return QtCore.Qt.Horizontal
        return (
            QtCore.Qt.Horizontal
            if float(width) / float(height) >= cls.CAMERA_ASPECT_RATIO
            else QtCore.Qt.Vertical
        )

    def _update_preview_pair_orientation(self) -> None:
        if self.workspace_splitter.width() <= 0 or self.workspace_splitter.height() <= 0:
            return
        available_height = self._available_camera_height()
        tall_layout = (
            float(self.workspace_splitter.width()) / float(max(1, available_height))
            < self.TALL_LAYOUT_RATIO
        )
        orientation = QtCore.Qt.Vertical if tall_layout else QtCore.Qt.Horizontal
        if self.preview_splitter.orientation() == orientation:
            return
        self.preview_splitter.setOrientation(orientation)
        self.preview_splitter.setSizes([1, 1])

    def _available_camera_height(self) -> int:
        """Use the visible scroll viewport when content is taller than the window."""

        parent = self.parentWidget()
        while parent is not None:
            if isinstance(parent, QtWidgets.QScrollArea):
                viewport_height = parent.viewport().height()
                if viewport_height > 0:
                    return min(self.workspace_splitter.height(), viewport_height)
            parent = parent.parentWidget()
        return self.workspace_splitter.height()

    def _fit_preview_layout(self) -> None:
        self._update_preview_pair_orientation()
        if self.workspace_splitter.width() <= 0 or self.workspace_splitter.height() <= 0:
            return
        tall_layout = self.preview_splitter.orientation() == QtCore.Qt.Vertical
        if tall_layout:
            # In the tall mode the preview pair is the left pane of a
            # horizontal workspace splitter.  Give the details pane a stable
            # inspection width, then size the pair from its real 16:9
            # height-for-width contract.  The outer workspace scroll area may
            # extend below the viewport when two full-width previews cannot
            # fit at once; it must not crop native image coordinates.
            self.workspace_splitter.setOrientation(QtCore.Qt.Horizontal)
            total_width = max(1, self.workspace_splitter.width() - self.workspace_splitter.handleWidth())
            details_width = min(380, max(280, int(round(total_width * 0.35))))
            preview_width = max(1, total_width - details_width)
            self.workspace_splitter.setSizes([preview_width, details_width])
            self.preview_splitter.setMinimumHeight(0)
            self.preview_splitter.setMaximumHeight(16777215)
            required_height = _preview_pair_height(self.preview_splitter, self.previews)
            self.preview_splitter.setMinimumHeight(max(320, required_height))
            _balance_preview_pair(self.preview_splitter)
            return

        # Landscape mode keeps the familiar side-by-side pair above the
        # scrollable detail surface.  The pair gets its natural aspect-ratio
        # height; extra vertical room is intentionally left to the details
        # surface instead of stretching/cropping the live image.
        self.workspace_splitter.setOrientation(QtCore.Qt.Vertical)
        self.preview_splitter.setMinimumHeight(0)
        self.preview_splitter.setMaximumHeight(16777215)
        required_height = _preview_pair_height(self.preview_splitter, self.previews)
        if required_height <= 0:
            return
        target_height = max(320, required_height)
        self.preview_splitter.setMinimumHeight(target_height)
        total_height = self.workspace_splitter.height()
        if total_height <= 0:
            return
        self.workspace_splitter.setStretchFactor(0, 0)
        self.workspace_splitter.setStretchFactor(1, 1)
        self.workspace_splitter.setSizes(
            [target_height, max(1, total_height - target_height)]
        )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._update_preview_pair_orientation()
        QtCore.QTimer.singleShot(0, self._fit_preview_layout)

    def _connect_panes(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        clear_signal = getattr(panel, "camera_preview_cleared", None)
        connect = getattr(clear_signal, "connect", None)
        if callable(connect):
            connect(self._clear_camera_previews)
        for side in ("left", "right"):
            pane = getattr(panel, f"{side}_pane", None)
            signal = getattr(pane, "frame_changed", None)
            if signal is not None:
                signal.connect(lambda _label, image, metadata, side=side: self.previews[side].set_frame(image, metadata))
            stats_signal = getattr(pane, "stats_changed", None)
            if stats_signal is not None:
                stats_signal.connect(lambda _label, stats, side=side: self.previews[side].set_metadata(stats))

    def _clear_camera_previews(self) -> None:
        for preview in self.previews.values():
            preview.clear()

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        for side in ("left", "right"):
            status = snapshot.get(f"camera.{side}")
            self.camera_values[f"camera.{side}"].set_value(
                status.state if status else "UNKNOWN",
                source=status.source if status else "status adapter",
                age_s=status.age_s if status else None,
                valid=(
                    True
                    if status is not None and status.state == "LIVE"
                    else False
                    if status is not None and status.state == "ERROR"
                    else None
                ),
            )
        fps = snapshot.get("fps")
        self.camera_values["fps"].set_value(fps.detail if fps else "--", source=fps.source if fps else "")
        detection = snapshot.get("detection")
        self.camera_values["detection"].set_value(
            detection.state if detection else "UNKNOWN",
            source=detection.source if detection else "",
        )
        recording = snapshot.get("recording")
        recording_locked = recording is not None and recording.state in {"ACTIVE", "FINALIZING"}
        self._recording_locked = recording_locked
        panel = getattr(self.backend, "panel", self.backend)
        rtsp_operation_active = any(
            getattr(panel, attribute, None) is not None
            for attribute in ("_rtsp_reactivation_side", "rtsp_enable_thread", "rtsp_check_thread")
        )
        preview_button = self.operation_buttons.get("camera.toggle")
        if preview_button is not None:
            camera_running = any(
                status is not None and status.state in {"LIVE", "STARTING"}
                for status in (snapshot.get("camera.left"), snapshot.get("camera.right"))
            )
            preview_button.setText("Stop preview" if camera_running else "Preview")
            preview_button.setEnabled(not recording_locked)
        for command_id in (
            "camera.swap",
            "camera.save_settings",
        ):
            button = self.operation_buttons.get(command_id)
            if button is not None:
                button.setEnabled(not recording_locked)
        for command_id in ("camera.reactivate_left", "camera.reactivate_right"):
            button = self.operation_buttons.get(command_id)
            if button is not None:
                button.setEnabled(not recording_locked and not rtsp_operation_active)
        connection = evidence.camera_connection_candidates()
        camera_discovery_status = str(connection.get("status", "NOT RUN") or "NOT RUN")
        self.discovery_status.set_value(
            camera_discovery_status,
            valid=False if camera_discovery_status.startswith("ERROR") else None,
        )
        candidates = connection.get("candidates")
        candidates = tuple(row for row in candidates if isinstance(row, dict)) if isinstance(candidates, (tuple, list)) else ()
        candidate_signature = tuple(
            tuple(row.get(key, "") for key in ("kind", "address", "port", "status", "interface", "host_ip"))
            for row in candidates
        )
        if candidate_signature != self._candidate_signature:
            selected = self._selected_candidate_identity()
            self.candidate_model.removeRows(0, self.candidate_model.rowCount())
            for row in candidates:
                _append_table_row(
                    self.candidate_model,
                    tuple(row.get(key, "") for key in ("kind", "address", "port", "status", "interface", "host_ip")),
                )
            self._candidate_signature = candidate_signature
            if selected:
                for row_index, row in enumerate(candidates):
                    if (str(row.get("kind", "")), str(row.get("address", ""))) == selected:
                        self.candidate_table.selectRow(row_index)
                        break
        self.operation_buttons["camera.discover_host_ip"].setEnabled(
            not recording_locked and not bool(connection.get("discovery_active", False))
        )
        self.operation_buttons["camera.auto_connect"].setEnabled(
            not recording_locked and not bool(connection.get("discovery_active", False))
        )
        self._refresh_candidate_actions()
        config = evidence.detection_config()
        for key, value in self.detection_config_values.items():
            raw = config.get(key)
            if key == "input_size_px" and raw not in (None, ""):
                raw = f"{raw} px"
            elif key in {"confidence", "iou"} and raw not in (None, ""):
                raw = f"{float(raw):.2f}"
            elif key == "gate_enabled":
                raw = ", ".join(str(item) for item in (raw or ())) or "OFF"
            value.set_value(raw, source=str(config.get("source", "")))
        target_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            target = evidence.detection(side)
            bbox = "--"
            if target.get("x1_px") is not None:
                bbox = "[" + ", ".join(
                    _display_number(target.get(name), digits=1)
                    for name in ("x1_px", "y1_px", "x2_px", "y2_px")
                ) + "]"
            class_text = _display_class(target)
            confidence = _display_number(target.get("confidence"), digits=3)
            timestamp = _display_number(target.get("timestamp_s"), digits=3)
            age = _display_number(target.get("age_s"), digits=2, suffix="s")
            target_rows.append(
                (
                    side.upper(),
                    target.get("runtime_state", "NO DATA"),
                    class_text,
                    confidence,
                    bbox,
                    timestamp,
                    age,
                ),
            )
        _sync_table_rows(self.target_model, tuple(target_rows))

    def _selected_candidate_row(self) -> int:
        selected = self.candidate_table.selectionModel().selectedRows()
        return selected[0].row() if selected else -1

    def _selected_candidate_identity(self) -> tuple[str, str] | None:
        row = self._selected_candidate_row()
        kind = self.candidate_model.item(row, 0) if row >= 0 else None
        address = self.candidate_model.item(row, 1) if row >= 0 else None
        if kind is None or address is None:
            return None
        return kind.text(), address.text()

    def _refresh_candidate_actions(self, *_args: object) -> None:
        row = self._selected_candidate_row()
        item = self.candidate_model.item(row, 0) if row >= 0 else None
        kind = item.text() if item is not None else ""
        self.candidate_buttons["nic"].setEnabled(not self._recording_locked and kind == "NIC")
        side_enabled = kind in {"RTSP", "WIRIS"}
        self.candidate_buttons["left"].setEnabled(not self._recording_locked and side_enabled)
        self.candidate_buttons["right"].setEnabled(not self._recording_locked and side_enabled)

    def _apply_candidate(self, target: str) -> None:
        row = self._selected_candidate_row()
        method = getattr(self.backend, "camera_apply_candidate", None)
        if row < 0 or not callable(method):
            self.candidate_feedback.setText("ERROR: select a compatible endpoint")
            return
        try:
            applied = method(row, target)
        except Exception as exc:
            self.candidate_feedback.setText(f"ERROR: {exc}")
            return
        if applied is False:
            self.candidate_feedback.setText("ERROR: endpoint was not applied")
            return
        self.candidate_feedback.setText(f"Applied row {row + 1} to {target.upper()}")
        self.refresh(self.status_model.snapshot())

    def _inspect_candidate_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        self.inspection_requested.emit(
            _live_inspection(
                self._candidate_inspection(row),
                lambda row=row: self._candidate_inspection(row),
            )
        )

    def _candidate_inspection(self, row: int) -> dict[str, object]:
        candidates = _evidence(self.backend).camera_connection_candidates().get("candidates")
        candidates = candidates if isinstance(candidates, (tuple, list)) else ()
        candidate = dict(candidates[row]) if 0 <= row < len(candidates) and isinstance(candidates[row], dict) else {}
        return {
            "selection": f"Camera endpoint · {candidate.get('kind', '--')}",
            "value": (
                f"address={candidate.get('address', '--')} · port={candidate.get('port', '--')} · "
                f"host interface={candidate.get('interface', '--')} {candidate.get('host_ip', '')}"
            ),
            "freshness": "Latest completed camera network discovery",
            "contract": "Network/RTSP/control endpoint metadata; no image pixels or camera-frame coordinates",
            "verdict": candidate.get("status") or "UNVERIFIED",
            "source": "CameraNetworkDiscoveryWorker",
        }


class TimeWorkspace(QtWidgets.QWidget):
    """Field-facing WIRIS clock and RTCP/NTP verification surface.

    The page only exposes the existing clock workflow owner.  It does not
    create a second clock service or reinterpret timestamps; the labels mirror
    the legacy panel's source-backed status widgets.
    """

    def __init__(
        self,
        backend: object,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.invoke_command = invoke_command
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader("Clock", "RTCP/NTP")
        )

        workflow_group = QtWidgets.QGroupBox("Clock")
        workflow_layout = QtWidgets.QHBoxLayout(workflow_group)
        workflow_layout.setContentsMargins(6, 6, 6, 6)
        workflow_layout.setSpacing(6)
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("time.set_pc", "Set camera clock"),
            ("time.check_sync", "Check sync"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            workflow_layout.addWidget(button)
        workflow_layout.addStretch(1)
        layout.addWidget(workflow_group)

        sync_group = QtWidgets.QGroupBox("Sync status")
        sync_layout = QtWidgets.QVBoxLayout(sync_group)
        sync_status_row = QtWidgets.QHBoxLayout()
        self.sync_status = QtWidgets.QLabel("Waiting for RTCP/NTP")
        self.sync_status.setObjectName("statusStrip")
        self.sync_status.setWordWrap(True)
        self.sync_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.sync_reason = QtWidgets.QLabel("")
        self.sync_reason.setObjectName("statusReason")
        self.sync_reason.setWordWrap(True)
        self.sync_reason.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.sync_reason.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.sync_reason.setStyleSheet(f"color: {ERROR_TEXT};")
        self.sync_reason.hide()
        sync_status_row.addWidget(self.sync_status, 1)
        sync_status_row.addWidget(self.sync_reason, 1)
        sync_layout.addLayout(sync_status_row)
        self.sync_summary = QtWidgets.QPlainTextEdit()
        self.sync_summary.setReadOnly(True)
        self.sync_summary.setMaximumBlockCount(80)
        # Tall enough to show recent RTCP/NTP sync lines without an inner
        # scrollbar; the workspace itself scrolls at the page level.
        self.sync_summary.setFixedHeight(200)
        sync_layout.addWidget(self.sync_summary)
        layout.addWidget(sync_group)

        settings = _evidence(self.backend).measurement_settings()
        self.settings_widgets: dict[str, QtWidgets.QWidget] = {}

        def double_spin(
            key: str,
            minimum: float,
            maximum: float,
            decimals: int,
            suffix: str,
        ) -> QtWidgets.QDoubleSpinBox:
            widget = QtWidgets.QDoubleSpinBox()
            widget.setRange(minimum, maximum)
            widget.setDecimals(decimals)
            widget.setValue(float(settings.get(key) if settings.get(key) is not None else minimum))
            widget.setSuffix(suffix)
            self.settings_widgets[key] = widget
            return widget

        def int_spin(
            key: str,
            minimum: int,
            maximum: int,
            suffix: str = "",
        ) -> QtWidgets.QSpinBox:
            widget = QtWidgets.QSpinBox()
            widget.setRange(minimum, maximum)
            value = settings.get(key)
            widget.setValue(int(value if value is not None else minimum))
            widget.setSuffix(suffix)
            self.settings_widgets[key] = widget
            return widget

        limits_group = QtWidgets.QGroupBox("Sync limits")
        limits_form = QtWidgets.QGridLayout(limits_group)
        limits_form.setHorizontalSpacing(8)
        limits_form.setVerticalSpacing(5)
        limits_form.setColumnStretch(1, 1)
        limits_form.setColumnStretch(3, 1)
        limits_fields = (
            ("History", double_spin("camera_sync_history_window_s", 0.1, 5.0, 2, " s")),
            ("Receive skew", double_spin("camera_sync_receive_pair_max_delta_ms", 0.0, 2000.0, 1, " ms")),
            ("RTCP/NTP Δt", double_spin("camera_sync_rtcp_pair_max_delta_ms", 0.0, 1000.0, 1, " ms")),
            ("SR stale", double_spin("camera_sync_max_sr_age_s", 0.1, 60.0, 2, " s")),
            ("SR-PC offset", double_spin("camera_sync_max_pc_offset_ms", 0.0, 5000.0, 1, " ms")),
            ("Clock rate error", double_spin("camera_sync_clock_rate_error_pct", 0.0, 10.0, 3, " %")),
            ("Required SR", int_spin("camera_sync_required_sr_count", 1, 20, " count")),
            ("Warmup", double_spin("camera_sync_warmup_timeout_s", 1.0, 120.0, 1, " s")),
        )
        for index, (label, widget) in enumerate(limits_fields):
            row = index // 2
            column = (index % 2) * 2
            limits_form.addWidget(QtWidgets.QLabel(label), row, column)
            limits_form.addWidget(widget, row, column + 1)
        layout.addWidget(limits_group)

        self.sync_settings_status = QtWidgets.QLabel(
            "Sync limits apply now; Save keeps them for the next run."
        )
        self.sync_settings_status.setObjectName("statusStrip")
        self.sync_settings_status.setWordWrap(True)
        layout.addWidget(self.sync_settings_status)
        for command_id, label in (
            ("time.apply_settings", "Apply limits"),
            ("time.save_settings", "Save limits"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            workflow_layout.addWidget(button)

        contract = QtWidgets.QLabel("RTCP/NTP timestamps")
        contract.setObjectName("compactContract")
        layout.addWidget(contract)
        layout.addStretch(1)

    @staticmethod
    def _widget_text(widget: object, default: str = "--") -> str:
        if widget is None:
            return default
        text = getattr(widget, "text", None)
        if callable(text):
            return str(text())
        plain_text = getattr(widget, "toPlainText", None)
        if callable(plain_text):
            return str(plain_text())
        return str(widget)

    def refresh(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        # The Workstation owns a separate visible Time workspace while the
        # source panel (and its legacy time tab) stays hidden.  Once the
        # finite STUS verification timer reaches verified/failed, frame-path
        # refreshes therefore no longer update the source labels.  Recompute
        # the source-backed summary at the bounded 4 Hz workspace poll while
        # this page is visible, then mirror the resulting widgets below.
        update_clock_sync = getattr(panel, "_update_clock_sync_display", None)
        if callable(update_clock_sync):
            update_clock_sync()
        sync_status = self._widget_text(
            getattr(panel, "clock_sync_status_label", None), "Waiting for RTCP/NTP"
        )
        self.sync_status.setText(_compact_text(sync_status))
        self.sync_status.setToolTip(sync_status)
        sync_state = str(getattr(panel, "camera_sync_status", "") or "").lower()
        if sync_state == "failed":
            reason = str(getattr(panel, "camera_sync_reason", "") or "reason unavailable")
            self.sync_reason.setText(f"Failed: {reason}")
            self.sync_reason.setToolTip(reason)
            self.sync_reason.show()
        else:
            self.sync_reason.clear()
            self.sync_reason.hide()
        summary = self._widget_text(getattr(panel, "clock_sync_summary_text", None), "")
        if summary != self.sync_summary.toPlainText():
            self.sync_summary.setPlainText(summary)

    def settings_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {}
        for key, widget in self.settings_widgets.items():
            if isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
                payload[key] = widget.value()
        return payload


class MeasurementWorkspace(QtWidgets.QWidget):
    inspection_requested = QtCore.pyqtSignal(object)
    CAMERA_ASPECT_RATIO = 16.0 / 9.0

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.invoke_command = invoke_command
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Measurement"))

        run_group = QtWidgets.QGroupBox("Run")
        run_layout = QtWidgets.QHBoxLayout(run_group)
        run_layout.setContentsMargins(6, 6, 6, 6)
        run_layout.setSpacing(6)
        self.control_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("camera.toggle", "Preview"),
            ("detection.toggle", "Detection"),
            ("recording.toggle", "Record"),
            ("session.save", "Save"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.control_buttons[command_id] = button
            run_layout.addWidget(button)
        run_layout.addStretch(1)
        layout.addWidget(run_group)

        upstream_group = QtWidgets.QGroupBox("Inputs")
        upstream_layout = QtWidgets.QVBoxLayout(upstream_group)
        upstream_layout.setContentsMargins(6, 6, 6, 6)
        upstream_layout.setSpacing(4)
        self.preview_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.preview_splitter.setChildrenCollapsible(False)
        self.preview_row = self.preview_splitter
        preview_row = self.preview_splitter
        self.previews = {side: CameraPreview(side) for side in ("left", "right")}
        for side in ("left", "right"):
            preview = self.previews[side]
            preview.setMinimumHeight(280)
            preview_row.addWidget(preview)
        preview_row.setMinimumHeight(280)
        preview_row.setStretchFactor(0, 1)
        preview_row.setStretchFactor(1, 1)
        upstream_layout.addWidget(preview_row)
        self.upstream_values: dict[str, EngineeringValue] = {}
        upstream_fields = (
            ("camera.left", "CAM L"),
            ("camera.right", "CAM R"),
            ("detection", "Detection"),
            ("synchronization", "Sync"),
            ("gnss", "GNSS / RTK"),
            ("recording", "Recording"),
            ("fps", "Decoded FPS"),
        )
        facts_layout = QtWidgets.QGridLayout()
        facts_layout.setContentsMargins(0, 0, 0, 0)
        facts_layout.setHorizontalSpacing(8)
        facts_layout.setVerticalSpacing(4)
        for index, (key, label) in enumerate(upstream_fields):
            value = EngineeringValue(label)
            self.upstream_values[key] = value
            value.activated.connect(lambda key=key: self._inspect_upstream(key))
            facts_layout.addWidget(value, index // 4, index % 4)
        upstream_layout.addLayout(facts_layout)
        details = QtWidgets.QWidget()
        details_layout = QtWidgets.QVBoxLayout(details)
        details_layout.setContentsMargins(0, 0, 0, 0)
        details_layout.setSpacing(6)
        self.upstream_group = upstream_group
        self.details = details
        # Inputs must be part of the same page flow as the result/settings
        # content.  Keeping it as the first child of the details page lets the
        # outer Measurement workspace scroll move the whole page together.
        details_layout.addWidget(upstream_group)

        quality_group = QtWidgets.QGroupBox("Quality")
        quality_layout = QtWidgets.QGridLayout(quality_group)
        quality_layout.setContentsMargins(6, 6, 6, 6)
        quality_layout.setHorizontalSpacing(8)
        quality_layout.setVerticalSpacing(4)
        self.quality_labels: dict[str, QtWidgets.QLabel] = {}
        for key, label in (
            ("status", "Status"),
            ("gap", "Ray gap"),
            ("distance", "Midpoint range"),
            ("mode", "Method"),
            ("attitude", "Attitude timing"),
            ("raw_point", "Raw point"),
        ):
            value = QtWidgets.QLabel("--")
            value.setWordWrap(True)
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            if key in {"status", "attitude", "raw_point"}:
                value.setObjectName("statusStrip")
            self.quality_labels[key] = value
            if key == "status":
                quality_layout.addWidget(QtWidgets.QLabel(label), 0, 0)
                quality_layout.addWidget(value, 0, 1, 1, 3)
            elif key == "attitude":
                quality_layout.addWidget(QtWidgets.QLabel(label), 2, 0)
                quality_layout.addWidget(value, 2, 1, 1, 3)
            elif key == "raw_point":
                quality_layout.addWidget(QtWidgets.QLabel(label), 3, 0)
                quality_layout.addWidget(value, 3, 1, 1, 3)
            else:
                column = {"gap": 0, "distance": 1, "mode": 2}[key]
                quality_layout.addWidget(QtWidgets.QLabel(label), 1, column * 2)
                quality_layout.addWidget(value, 1, column * 2 + 1)
        quality_layout.setColumnStretch(1, 1)
        quality_layout.setColumnStretch(3, 1)
        details_layout.addWidget(quality_group)

        values_group = QtWidgets.QGroupBox("Result")
        values_layout = QtWidgets.QGridLayout(values_group)
        self.values = {
            "range": EngineeringValue("Range"),
            "forward": EngineeringValue("Forward"),
            "right": EngineeringValue("Right"),
            "down": EngineeringValue("Down"),
        }
        for index, key in enumerate(("range", "forward", "right", "down")):
            self.values[key].activated.connect(lambda key=key: self._inspect_result(key))
            values_layout.addWidget(self.values[key], index // 2, index % 2)
        details_layout.addWidget(values_group)

        comparison_group = QtWidgets.QGroupBox("Extrinsics 비교 (F/R/D/Rg)")
        comparison_layout = QtWidgets.QVBoxLayout(comparison_group)
        comparison_layout.setContentsMargins(6, 6, 6, 6)
        comparison_note = QtWidgets.QLabel(
            "동일 좌/우 검출 픽셀에 대해 각 외부 파라미터 소스로 계산한 결과 미리보기 · "
            "실제 기록되는 값은 Result에 표시된 방식(Distance Mode)만 사용합니다."
        )
        comparison_note.setObjectName("statusStrip")
        comparison_note.setWordWrap(True)
        comparison_layout.addWidget(comparison_note)
        self.geometry_comparison_model = _table_model(
            self, ("Method", "Forward m", "Right m", "Down m", "Range m", "Status")
        )
        self.geometry_comparison_table = QtWidgets.QTableView()
        self.geometry_comparison_table.setModel(self.geometry_comparison_model)
        self.geometry_comparison_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.geometry_comparison_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.geometry_comparison_table.verticalHeader().setVisible(False)
        self.geometry_comparison_table.horizontalHeader().setStretchLastSection(True)
        self.geometry_comparison_table.setMaximumHeight(220)
        comparison_layout.addWidget(self.geometry_comparison_table)
        details_layout.addWidget(comparison_group)

        settings_group = QtWidgets.QGroupBox("Settings")
        settings_layout = QtWidgets.QVBoxLayout(settings_group)
        settings_layout.setContentsMargins(6, 6, 6, 6)
        settings_layout.setSpacing(5)
        settings = _evidence(self.backend).measurement_settings()
        self.settings_widgets: dict[str, QtWidgets.QWidget] = {}

        def combo(
            key: str,
            items: tuple[tuple[str, str], ...],
        ) -> QtWidgets.QComboBox:
            widget = QtWidgets.QComboBox()
            for label, value in items:
                widget.addItem(label, value)
            current = str(settings.get(key) or "")
            index = widget.findData(current)
            widget.setCurrentIndex(max(0, index))
            self.settings_widgets[key] = widget
            return widget

        def double_spin(
            key: str,
            minimum: float,
            maximum: float,
            decimals: int,
            suffix: str,
        ) -> QtWidgets.QDoubleSpinBox:
            widget = QtWidgets.QDoubleSpinBox()
            widget.setRange(minimum, maximum)
            widget.setDecimals(decimals)
            widget.setValue(float(settings.get(key) if settings.get(key) is not None else minimum))
            widget.setSuffix(suffix)
            self.settings_widgets[key] = widget
            return widget

        def int_spin(
            key: str,
            minimum: int,
            maximum: int,
            suffix: str = "",
        ) -> QtWidgets.QSpinBox:
            widget = QtWidgets.QSpinBox()
            widget.setRange(minimum, maximum)
            value = settings.get(key)
            widget.setValue(int(value if value is not None else minimum))
            widget.setSuffix(suffix)
            self.settings_widgets[key] = widget
            return widget

        def toggle(key: str, label: str) -> QtWidgets.QPushButton:
            widget = QtWidgets.QPushButton(label)
            widget.setCheckable(True)
            widget.setChecked(bool(settings.get(key, False)))
            widget.setProperty("role", "toggle")
            self.settings_widgets[key] = widget
            return widget

        def text_edit(key: str, placeholder: str = "") -> QtWidgets.QLineEdit:
            widget = QtWidgets.QLineEdit(str(settings.get(key) or ""))
            if placeholder:
                widget.setPlaceholderText(placeholder)
            self.settings_widgets[key] = widget
            return widget

        geometry_form = QtWidgets.QGridLayout()
        geometry_form.setHorizontalSpacing(8)
        geometry_form.setVerticalSpacing(5)
        geometry_form.setColumnStretch(1, 1)
        geometry_form.setColumnStretch(3, 1)
        geometry_fields = (
            (
                "Intrinsics",
                combo("intrinsics_source", (("Checkerboard", "checkerboard"), ("Manual fallback", "manual"))),
            ),
            (
                "Focal length",
                double_spin("focal_px", 1.0, 100000.0, 3, " px"),
            ),
            (
                "Baseline",
                combo(
                    "distance_mode",
                    (("Pixhawk attitude + GPS/RTK NED", "pixhawk"), ("Checkerboard stereo R/T", "checkerboard")),
                ),
            ),
            (
                "YOLO input",
                int_spin("yolo_input_size_px", 160, 1920, " px"),
            ),
            (
                "Confidence",
                double_spin("yolo_confidence", 0.01, 1.0, 3, ""),
            ),
            (
                "IoU",
                double_spin("yolo_iou", 0.01, 1.0, 3, ""),
            ),
        )
        for index, (label, widget) in enumerate(geometry_fields):
            row = index // 2
            column = (index % 2) * 2
            geometry_form.addWidget(QtWidgets.QLabel(label), row, column)
            geometry_form.addWidget(widget, row, column + 1)
        settings_layout.addLayout(geometry_form)

        gate_group = QtWidgets.QGroupBox("Pair gates")
        gate_form = QtWidgets.QGridLayout(gate_group)
        gate_form.setHorizontalSpacing(8)
        gate_form.setVerticalSpacing(5)
        gate_form.setColumnStretch(1, 1)
        gate_form.setColumnStretch(4, 1)
        gate_form.addWidget(toggle("class_gate_enabled", "Class gate"), 0, 0)
        gate_form.addWidget(text_edit("class_gate", "class id or name"), 0, 1, 1, 2)
        gate_form.addWidget(toggle("matching_gate_enabled", "Time matching"), 1, 0)
        gate_form.addWidget(
            combo(
                "time_basis",
                (("RTCP/NTP required", "camera_rtc_required"), ("RTCP/NTP preferred", "camera_rtcp"), ("App receive", "detection")),
            ),
            1,
            1,
        )
        gate_form.addWidget(toggle("time_delta_gate_enabled", "Max Δt"), 1, 2)
        gate_form.addWidget(double_spin("max_pair_delta_s", 0.0, 10.0, 3, " s"), 1, 3)
        gate_form.addWidget(toggle("vertical_delta_gate_enabled", "Vertical Δ"), 1, 4)
        gate_form.addWidget(double_spin("max_vertical_delta_px", 0.0, 5000.0, 1, " px"), 1, 5)
        settings_layout.addWidget(gate_group)

        self.measurement_settings_status = QtWidgets.QLabel(
            "Changes apply now; Save keeps them for the next run."
        )
        self.measurement_settings_status.setObjectName("statusStrip")
        self.measurement_settings_status.setWordWrap(True)
        settings_layout.addWidget(self.measurement_settings_status)
        settings_actions = QtWidgets.QHBoxLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("measurement.apply_settings", "Apply"),
            ("measurement.save_settings", "Save"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            settings_actions.addWidget(button)
        settings_actions.addStretch(1)
        settings_layout.addLayout(settings_actions)
        details_layout.addWidget(settings_group)

        self.history_model = QtGui.QStandardItemModel(0, 5, self)
        self.history_model.setHorizontalHeaderLabels(["timestamp_s", "Forward m", "Right m", "Down m", "Quality"])
        self.history = QtWidgets.QTableView()
        self.history.setModel(self.history_model)
        self.history.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.history.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.history.verticalHeader().setVisible(False)
        self.history.setColumnWidth(0, 150)
        for column in (1, 2, 3):
            self.history.setColumnWidth(column, 100)
        self.history.horizontalHeader().setStretchLastSection(True)
        self.history.clicked.connect(self._inspect_history_row)
        details_layout.addWidget(self.history, 1)
        layout.addWidget(details, 1)
        self._connect_panes()

    @classmethod
    def preview_pair_orientation(cls, width: int, height: int) -> QtCore.Qt.Orientation:
        """Choose the pair layout that gives two 16:9 frames more usable area."""

        if width <= 0 or height <= 0:
            return QtCore.Qt.Horizontal
        return (
            QtCore.Qt.Horizontal
            if float(width) / float(height) >= cls.CAMERA_ASPECT_RATIO
            else QtCore.Qt.Vertical
        )

    def _update_preview_pair_orientation(self) -> None:
        width = self.preview_splitter.width()
        if width <= 0:
            return
        available_height = self.preview_splitter.height()
        parent = self.parentWidget()
        while parent is not None:
            if isinstance(parent, QtWidgets.QScrollArea):
                viewport_height = parent.viewport().height()
                if viewport_height > 0:
                    available_height = viewport_height
                    break
            parent = parent.parentWidget()
        horizontal_width = max(1, width - self.preview_splitter.handleWidth())
        horizontal_preview_width = max(1, horizontal_width // 2)
        horizontal_required_height = max(
            int(preview.heightForWidth(horizontal_preview_width))
            for preview in self.previews.values()
        )
        orientation = (
            QtCore.Qt.Horizontal
            if available_height >= horizontal_required_height
            else QtCore.Qt.Vertical
        )
        if self.preview_splitter.orientation() == orientation:
            return
        self.preview_splitter.setOrientation(orientation)
        self.preview_splitter.setSizes([1, 1])

    def _fit_preview_layout(self) -> None:
        self._update_preview_pair_orientation()
        required_height = _preview_pair_height(self.preview_splitter, self.previews)
        if required_height <= 0:
            return
        target_height = max(280, required_height)
        if self.preview_splitter.minimumHeight() == target_height:
            return
        # setFixedHeight() pins both the minimum and maximum height, so any
        # heightForWidth() fluctuation forces an immediate hard resize.  That
        # resize can toggle the ancestor QScrollArea's AsNeeded vertical
        # scrollbar, which changes this widget's width, which changes
        # heightForWidth() again -- an infinite grow/shrink loop.  A
        # minimum-only constraint (mirroring CameraWorkspace) lets the layout
        # settle instead of re-forcing the exact pixel height every pass.
        self.preview_splitter.setMaximumHeight(16777215)
        self.preview_splitter.setMinimumHeight(target_height)

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt override.
        super().resizeEvent(event)
        self._update_preview_pair_orientation()
        QtCore.QTimer.singleShot(0, self._fit_preview_layout)

    def _inspect_upstream(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._upstream_inspection(key),
                lambda key=key: self._upstream_inspection(key),
            )
        )

    def _upstream_inspection(self, key: str) -> dict[str, object]:
        status = self.status_model.record(key)
        return {
            "selection": self.upstream_values[key].label.text(),
            "value": status.state if status is not None else "NO DATA",
            "freshness": "--" if status is None or status.age_s is None else f"age {status.age_s:.2f}s · {status.clock}",
            "contract": "Producer contract is preserved; inspect the owning device screen for field coordinates",
            "verdict": status.detail if status is not None else "No evidence",
            "source": status.source if status is not None else "StatusModel",
        }

    def _inspect_result(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._result_inspection(key),
                lambda key=key: self._result_inspection(key),
            )
        )

    def _result_inspection(self, key: str) -> dict[str, object]:
        status = self.status_model.record("measurement")
        value = self.values[key]
        return {
            "selection": f"Measurement {value.label.text()}",
            "value": f"{value.value.text()} {value.unit.text()}".strip(),
            "freshness": "--" if status is None or status.age_s is None else f"age {status.age_s:.2f}s · {status.clock}",
            "contract": f"{MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}] · metres",
            "verdict": self.quality_labels["status"].text(),
            "source": "accepted DistanceHistoryPoint",
        }

    def _inspect_history_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        values = [self.history_model.item(row, column).text() for column in range(self.history_model.columnCount())]
        self.inspection_requested.emit(
            {
                "selection": f"Measurement history row {row + 1}",
                "value": f"F/R/D={values[1]}/{values[2]}/{values[3]} m · quality={values[4]}",
                "freshness": f"timestamp_s={values[0]}",
                "contract": f"{MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}] · metres",
                "verdict": values[4],
                "source": "DistanceHistoryPoint",
            }
        )

    def settings_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {}
        for key, widget in self.settings_widgets.items():
            if isinstance(widget, QtWidgets.QComboBox):
                payload[key] = widget.currentData()
            elif isinstance(widget, QtWidgets.QAbstractButton):
                payload[key] = widget.isChecked()
            elif isinstance(widget, QtWidgets.QLineEdit):
                payload[key] = widget.text().strip()
            elif isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
                payload[key] = widget.value()
        return payload

    def _connect_panes(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        clear_signal = getattr(panel, "camera_preview_cleared", None)
        connect = getattr(clear_signal, "connect", None)
        if callable(connect):
            connect(self._clear_camera_previews)
        for side in ("left", "right"):
            pane = getattr(panel, f"{side}_pane", None)
            signal = getattr(pane, "frame_changed", None)
            if signal is not None:
                signal.connect(lambda _label, image, metadata, side=side: self.previews[side].set_frame(image, metadata))
            detection_signal = getattr(pane, "detection_changed", None)
            if detection_signal is not None:
                detection_signal.connect(
                    lambda _label, detection, side=side: self.previews[side].set_detection(detection)
                )
            stats_signal = getattr(pane, "stats_changed", None)
            if stats_signal is not None:
                stats_signal.connect(lambda _label, stats, side=side: self.previews[side].set_metadata(stats))

    def _clear_camera_previews(self) -> None:
        for preview in self.previews.values():
            preview.clear()

    def _refresh_preview_metadata(self, panel: object) -> None:
        metadata = getattr(panel, "camera_frame_metadata", {}) or {}
        if not isinstance(metadata, dict):
            return
        for side, preview in self.previews.items():
            current = metadata.get(side)
            if isinstance(current, dict):
                preview.set_metadata(current)

    def _refresh_quality(self, panel: object) -> None:
        quality_sources = {
            "status": ("triangulation_quality_status_label", "3D ray triangulation 대기 중"),
            "gap": ("triangulation_quality_gap_label", "gap: --"),
            "distance": ("triangulation_quality_distance_label", "distance: --"),
            "mode": ("triangulation_quality_mode_label", "mode: --"),
            "attitude": ("triangulation_quality_attitude_label", "Attitude timing: --"),
            "raw_point": ("triangulation_quality_raw_point_label", "Raw baseline_center_frd point: --"),
        }
        for key, (attribute, default) in quality_sources.items():
            widget = getattr(panel, attribute, None)
            text = getattr(widget, "text", None)
            self.quality_labels[key].setText(str(text()) if callable(text) else default)

    def refresh_run_controls(self, snapshot: dict[str, RuntimeStatus], panel: object) -> None:
        camera = snapshot.get("camera.left")
        detection = snapshot.get("detection")
        recording = snapshot.get("recording")
        camera_state = camera.state if camera else "STOPPED"
        detection_state = detection.state if detection else "DISABLED"
        recording_state = recording.state if recording else "IDLE"
        recording_locked = recording_state in {"ACTIVE", "FINALIZING"}
        camera_running = bool(getattr(panel, "camera_connected", False)) or camera_state in {
            "LIVE",
            "STARTING",
        }
        clock_change_active = getattr(panel, "wiris_time_sync_thread", None) is not None
        self.control_buttons["camera.toggle"].setText(
            "Stop preview" if camera_state in {"LIVE", "STARTING"} else "Preview"
        )
        self.control_buttons["detection.toggle"].setText(
            "Stop detection" if detection_state in {"RUNNING", "STARTING"} else "Detection"
        )
        self.control_buttons["recording.toggle"].setText(
            "Stop recording" if recording_state in {"ACTIVE", "FINALIZING"} else "Record"
        )
        self.control_buttons["camera.toggle"].setEnabled(not recording_locked)
        self.control_buttons["detection.toggle"].setEnabled(
            not (recording_locked and camera_running)
        )
        self.control_buttons["recording.toggle"].setEnabled(
            recording_state not in {"FINALIZING", "RECOVERABLE"}
            and (recording_locked or not clock_change_active)
        )
        has_session_data = bool(getattr(panel, "recording_session_id", "")) or bool(
            getattr(panel, "distance_history", [])
        )
        self.control_buttons["session.save"].setEnabled(
            not recording_locked and (has_session_data or recording_state == "RECOVERABLE")
        )

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        self._refresh_preview_metadata(panel)
        self._refresh_quality(panel)
        self.refresh_run_controls(snapshot, panel)
        for key, value in self.upstream_values.items():
            status = snapshot.get(key)
            if status is None:
                value.set_value("NO DATA", source="status adapter")
                continue
            value.set_value(
                status.state,
                source=status.source,
                age_s=status.age_s,
                valid=(
                    True
                    if status.severity == StatusSeverity.OK
                    else False
                    if status.severity == StatusSeverity.BAD
                    else None
                ),
            )
        point = None
        history = getattr(panel, "distance_history", None)
        try:
            if history:
                point = list(history)[-1]
        except (TypeError, IndexError):
            point = None
        if point is not None:
            self.values["range"].set_value(getattr(point, "distance_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint")
            self.values["forward"].set_value(getattr(point, "forward_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint")
            self.values["right"].set_value(getattr(point, "right_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint")
            self.values["down"].set_value(getattr(point, "down_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint")
        else:
            for value in self.values.values():
                value.set_value(None, unit="m", frame=MEASUREMENT_FRAME, source="no accepted measurement")
        rows = []
        try:
            rows = list(history or [])[-100:]
        except TypeError:
            rows = []
        history_rows = tuple(
            (
                getattr(row, "timestamp_s", "--"),
                getattr(row, "forward_m", "--"),
                getattr(row, "right_m", "--"),
                getattr(row, "down_m", "--"),
                getattr(row, "quality", getattr(row, "method", "--")),
            )
            for row in rows
        )
        _sync_table_rows(self.history_model, history_rows)

        comparison_rows = tuple(
            (
                row.get("method", "--"),
                _display_number(row.get("forward_m"), digits=4),
                _display_number(row.get("right_m"), digits=4),
                _display_number(row.get("down_m"), digits=4),
                _display_number(row.get("range_m"), digits=4),
                row.get("status", "--"),
            )
            for row in (getattr(panel, "geometry_comparison_rows", ()) or ())
        )
        _sync_table_rows(self.geometry_comparison_model, comparison_rows)


class PixhawkWorkspace(QtWidgets.QWidget):
    """P1/P2 MAVLink attitude and GNSS-baseline evidence."""

    inspection_requested = QtCore.pyqtSignal(object)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Pixhawk"))

        actions = QtWidgets.QHBoxLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("pixhawk.connect_mavlink", "Connect"),
            ("pixhawk.disconnect_mavlink", "Disconnect"),
            ("pixhawk.discover_usb", "Find USB"),
            ("pixhawk.discover_mavlink_candidates", "Find network"),
            ("pixhawk.discover_host_ip", "Find host IP"),
            ("alignment.swap_pixhawks", "Swap P1/P2"),
            ("pixhawk.reset_level_calibration", "Reset level cal."),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            actions.addWidget(button)
        # Keep the legacy command addressable for existing integrations while
        # exposing explicit Connect/Disconnect controls in the UI.
        self.operation_buttons["pixhawk.toggle_mavlink"] = self.operation_buttons["pixhawk.connect_mavlink"]
        for side, label in (("left", "P1 Shell"), ("right", "P2 Shell")):
            button = QtWidgets.QPushButton(label)
            button.setToolTip("활성 Ethernet MAVLink 연결로 PX4 Shell을 엽니다.")
            button.clicked.connect(lambda _checked=False, side=side: self._open_px4_shell(side))
            self.operation_buttons[f"pixhawk.open_shell.{side}"] = button
            actions.addWidget(button)
        actions.addStretch(1)
        layout.addLayout(actions)

        status_group = QtWidgets.QGroupBox("Connection")
        status_layout = QtWidgets.QHBoxLayout(status_group)
        self.indicators: dict[str, StatusIndicator] = {}
        for key, label in (("pixhawk.left", "P1"), ("pixhawk.right", "P2")):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator)
        layout.addWidget(status_group)

        recovery_group = QtWidgets.QGroupBox("ETH MAV")
        recovery_layout = QtWidgets.QVBoxLayout(recovery_group)
        recovery_hint = QtWidgets.QLabel(
            "USB 제어 포트로 PX4의 Ethernet MAVLink를 런타임 시작합니다. "
            "시작 후 Heartbeat와 ATTITUDE 수신을 별도로 확인합니다."
        )
        recovery_hint.setObjectName("statusStrip")
        recovery_hint.setWordWrap(True)
        recovery_layout.addWidget(recovery_hint)
        recovery_cards = QtWidgets.QHBoxLayout()
        self.recovery_values: dict[str, dict[str, EngineeringValue]] = {}
        self.recovery_buttons: dict[str, QtWidgets.QPushButton] = {}
        for side, label, command_id in (
            ("left", "P1", "pixhawk.recover_eth_mavlink.p1"),
            ("right", "P2", "pixhawk.recover_eth_mavlink.p2"),
        ):
            card = QtWidgets.QGroupBox(label)
            card_layout = QtWidgets.QGridLayout(card)
            values: dict[str, EngineeringValue] = {}
            for row, (key, value_label) in enumerate(
                (
                    ("usb", "USB 제어"),
                    ("baud", "Baud"),
                    ("recovery", "상태"),
                    ("telemetry", "Telemetry"),
                )
            ):
                value = EngineeringValue(value_label)
                values[key] = value
                card_layout.addWidget(value, row, 0, 1, 2)
            button = _command_button(
                f"{label} ETH MAV",
                command_id,
                invoke_command,
            )
            button.setMinimumWidth(170)
            button.setToolTip(
                "USB 제어 포트로 PX4 Ethernet MAVLink를 런타임 시작합니다. "
                "재부팅 후 유지하려면 별도의 ETH 저장을 사용하세요."
            )
            card_layout.addWidget(button, 4, 0, 1, 2)
            self.recovery_values[side] = values
            self.recovery_buttons[side] = button
            recovery_cards.addWidget(card)
        recovery_cards.addStretch(1)
        recovery_layout.addLayout(recovery_cards)
        self.recovery_feedback = QtWidgets.QLabel("")
        self.recovery_feedback.setObjectName("statusStrip")
        self.recovery_feedback.setWordWrap(True)
        recovery_layout.addWidget(self.recovery_feedback)

        candidate_group = QtWidgets.QGroupBox("Discovered endpoints")
        candidate_layout = QtWidgets.QVBoxLayout(candidate_group)
        self.discovery_status = EngineeringValue("Discovery")
        candidate_layout.addWidget(self.discovery_status)
        self.candidate_model = _table_model(
            self,
            ("Source", "Kind", "Value", "Baud", "Status", "Interface"),
        )
        self.candidate_table = QtWidgets.QTableView()
        self.candidate_table.setModel(self.candidate_model)
        self.candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.candidate_table.verticalHeader().setVisible(False)
        self.candidate_table.setMaximumHeight(120)
        self.candidate_table.horizontalHeader().setStretchLastSection(True)
        self.candidate_table.selectionModel().selectionChanged.connect(self._refresh_candidate_actions)
        self.candidate_table.clicked.connect(self._inspect_candidate_row)
        candidate_layout.addWidget(self.candidate_table)
        candidate_actions = QtWidgets.QHBoxLayout()
        self.candidate_buttons: dict[str, QtWidgets.QPushButton] = {}
        for target, label in (("host", "Use host IP"), ("left", "Assign P1"), ("right", "Assign P2")):
            button = QtWidgets.QPushButton(label)
            button.setEnabled(False)
            button.clicked.connect(lambda _checked=False, target=target: self._apply_candidate(target))
            self.candidate_buttons[target] = button
            candidate_actions.addWidget(button)
        candidate_actions.addStretch(1)
        candidate_layout.addLayout(candidate_actions)
        self.candidate_feedback = QtWidgets.QLabel("")
        self.candidate_feedback.setObjectName("statusStrip")
        self.candidate_feedback.setWordWrap(True)
        candidate_layout.addWidget(self.candidate_feedback)
        self._candidate_signature: tuple[tuple[object, ...], ...] = ()
        self._candidate_rows: tuple[dict[str, object], ...] = ()
        self._recording_locked = False
        self._invoke_command_available = invoke_command is not None
        self._invoke_command = invoke_command

        attitude_group = QtWidgets.QGroupBox("Attitude (R/P/Y)")
        attitude_layout = QtWidgets.QVBoxLayout(attitude_group)
        self.attitude_model = _table_model(
            self,
            ("Pixhawk", "State", "Roll°", "Pitch°", "Yaw°", "Age", "PX4 time", "Endpoint"),
        )
        self.attitude_table = QtWidgets.QTableView()
        self.attitude_table.setModel(self.attitude_model)
        self.attitude_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.attitude_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.attitude_table.verticalHeader().setVisible(False)
        self.attitude_table.setMaximumHeight(112)
        self.attitude_table.horizontalHeader().setStretchLastSection(True)
        self.attitude_table.clicked.connect(self._inspect_attitude_row)
        attitude_layout.addWidget(self.attitude_table)
        layout.addWidget(attitude_group)

        receiver_group = QtWidgets.QGroupBox("P1/P2 GNSS receivers")
        receiver_layout = QtWidgets.QVBoxLayout(receiver_group)
        self.receiver_model = _table_model(
            self,
            ("Receiver", "Fix number", "RTK state", "Satellites", "Eph m", "Epv m", "Age"),
        )
        self.receiver_table = QtWidgets.QTableView()
        self.receiver_table.setModel(self.receiver_model)
        self.receiver_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.receiver_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.receiver_table.verticalHeader().setVisible(False)
        self.receiver_table.setMaximumHeight(112)
        self.receiver_table.horizontalHeader().setStretchLastSection(True)
        self.receiver_table.clicked.connect(self._inspect_receiver_row)
        receiver_layout.addWidget(self.receiver_table)
        layout.addWidget(receiver_group)

        magnetometer_group = QtWidgets.QGroupBox("IMU magnetometer (MAVLink)")
        magnetometer_layout = QtWidgets.QVBoxLayout(magnetometer_group)
        magnetometer_hint = QtWidgets.QLabel(
            "IMU stream 1/2/3 · SCALED_IMU/2/3 xmag/ymag/zmag · producer sensor axes · "
            "mG · no CAN-port mapping or NED/ENU/FRD conversion"
        )
        magnetometer_hint.setObjectName("statusStrip")
        magnetometer_hint.setWordWrap(True)
        magnetometer_layout.addWidget(magnetometer_hint)
        self.magnetometer_model = _table_model(
            self,
            ("Pixhawk", "IMU stream", "State", "xmag", "ymag", "zmag", "Norm", "Unit", "Age", "Topic"),
        )
        self.magnetometer_table = QtWidgets.QTableView()
        self.magnetometer_table.setModel(self.magnetometer_model)
        self.magnetometer_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.magnetometer_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.magnetometer_table.verticalHeader().setVisible(False)
        self.magnetometer_table.setMaximumHeight(220)
        self.magnetometer_table.horizontalHeader().setStretchLastSection(True)
        self.magnetometer_table.clicked.connect(self._inspect_magnetometer_row)
        magnetometer_layout.addWidget(self.magnetometer_table)
        layout.addWidget(magnetometer_group)

        can_magnetometer_group = QtWidgets.QGroupBox("CAN magnetometer (dedicated MAVLink)")
        can_magnetometer_layout = QtWidgets.QVBoxLayout(can_magnetometer_group)
        can_magnetometer_hint = QtWidgets.QLabel(
            "CAN1/CAN2 값은 PX4 DEBUG_VECT CAN_MAG 호환 메시지가 수신될 때만 표시 · "
            "SCALED_IMU* 및 CAL_MAG 설정값을 대체 사용하지 않음"
        )
        can_magnetometer_hint.setObjectName("statusStrip")
        can_magnetometer_hint.setWordWrap(True)
        can_magnetometer_layout.addWidget(can_magnetometer_hint)
        self.can_magnetometer_model = _table_model(
            self,
            (
                "Pixhawk",
                "CAN port",
                "Node ID",
                "Device ID",
                "State",
                "xmag",
                "ymag",
                "zmag",
                "Norm",
                "Unit",
                "Age",
                "Topic",
            ),
        )
        self.can_magnetometer_table = QtWidgets.QTableView()
        self.can_magnetometer_table.setModel(self.can_magnetometer_model)
        self.can_magnetometer_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.can_magnetometer_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.can_magnetometer_table.verticalHeader().setVisible(False)
        self.can_magnetometer_table.setMaximumHeight(160)
        self.can_magnetometer_table.horizontalHeader().setStretchLastSection(True)
        self.can_magnetometer_table.clicked.connect(self._inspect_can_magnetometer_row)
        can_magnetometer_layout.addWidget(self.can_magnetometer_table)
        layout.addWidget(can_magnetometer_group)

        self._build_px4_parameter_group(layout)

        baseline_group = QtWidgets.QGroupBox("P1→P2 baseline")
        baseline_layout = QtWidgets.QGridLayout(baseline_group)
        self.baseline_values: dict[str, EngineeringValue] = {}
        baseline_fields = (
            ("north_m", "NED North"),
            ("east_m", "NED East"),
            ("down_m", "NED Down"),
            ("length_m", "Length"),
            ("p1_body_frd_forward_m", "FRD Forward"),
            ("p1_body_frd_right_m", "FRD Right"),
            ("p1_body_frd_down_m", "FRD Down"),
            ("state", "Quality"),
        )
        for index, (key, label) in enumerate(baseline_fields):
            value = EngineeringValue(label)
            self.baseline_values[key] = value
            value.activated.connect(lambda key=key: self._inspect_baseline_value(key))
            baseline_layout.addWidget(value, index // 4, index % 4)
        layout.addWidget(baseline_group)
        layout.addWidget(candidate_group)
        layout.addWidget(recovery_group)
        self.operation_buttons.update(
            {
                "pixhawk.recover_eth_mavlink.p1": self.recovery_buttons["left"],
                "pixhawk.recover_eth_mavlink.p2": self.recovery_buttons["right"],
            }
        )
        layout.addStretch(1)

    def _build_px4_parameter_group(self, layout: QtWidgets.QVBoxLayout) -> None:
        group = QtWidgets.QGroupBox("Pixhawk Parameters · calibration-owned groups")
        parameter_group_layout = QtWidgets.QVBoxLayout(group)
        hint = QtWidgets.QLabel(
            "QGC에서 Pixhawk에 저장한 값을 캘리브레이션별로 나누어 표시합니다. "
            "각 초기화 버튼은 해당 그룹의 파라미터만 PX4 firmware default로 되돌리며, "
            "값은 PX4 센서축/보드 기준 raw parameter라 NED/ENU/카메라 좌표로 변환하지 않습니다."
        )
        hint.setObjectName("statusStrip")
        hint.setWordWrap(True)
        parameter_group_layout.addWidget(hint)

        self.px4_parameter_models: dict[str, dict[str, QtGui.QStandardItemModel]] = {}
        self.px4_parameter_tables: dict[str, dict[str, QtWidgets.QTableView]] = {}
        self.px4_parameter_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.px4_parameter_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.px4_parameter_group_models: dict[str, dict[str, QtGui.QStandardItemModel]] = {}
        self.px4_parameter_group_status_labels: dict[str, dict[str, QtWidgets.QLabel]] = {}
        self.px4_parameter_reset_buttons: dict[str, dict[str, QtWidgets.QPushButton]] = {}
        tabs = QtWidgets.QTabWidget()
        for side, side_code, title, command_id in (
            ("left", "p1", "Pixhawk 1 / P1", "pixhawk.refresh_parameters.p1"),
            ("right", "p2", "Pixhawk 2 / P2", "pixhawk.refresh_parameters.p2"),
        ):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            controls = QtWidgets.QHBoxLayout()
            refresh_button = _command_button("QGC 결과 새로고침", command_id, self._invoke_command)
            controls.addWidget(refresh_button)
            controls.addStretch(1)
            tab_layout.addLayout(controls)
            status_label = QtWidgets.QLabel("PARAM readback 대기 중")
            status_label.setObjectName("statusStrip")
            status_label.setWordWrap(True)
            status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            tab_layout.addWidget(status_label)
            self.px4_parameter_status_labels[side] = status_label
            self.px4_parameter_refresh_buttons[side] = refresh_button
            self.px4_parameter_group_models[side] = {}
            self.px4_parameter_models[side] = self.px4_parameter_group_models[side]
            self.px4_parameter_tables[side] = {}
            self.px4_parameter_group_status_labels[side] = {}
            self.px4_parameter_reset_buttons[side] = {}
            group_tabs = QtWidgets.QTabWidget()
            for calibration in PX4_CALIBRATION_GROUPS:
                group_tab = QtWidgets.QWidget()
                calibration_layout = QtWidgets.QVBoxLayout(group_tab)
                group_hint = QtWidgets.QLabel(calibration.description)
                group_hint.setObjectName("compactContract")
                group_hint.setWordWrap(True)
                calibration_layout.addWidget(group_hint)
                group_controls = QtWidgets.QHBoxLayout()
                if calibration.resettable:
                    reset_command_id = f"pixhawk.reset_calibration.{side_code}.{calibration.key}"
                    reset_button = _command_button(
                        "Reset to PX4 default",
                        reset_command_id,
                        self._invoke_command,
                    )
                else:
                    reset_button = QtWidgets.QPushButton("Manual parameter · no calibration reset")
                    reset_button.setEnabled(False)
                    reset_button.setToolTip(
                        "SENS_BOARD_ROT/Z_OFF are manual board-orientation parameters and are not changed by level calibration."
                    )
                group_controls.addWidget(reset_button)
                group_controls.addStretch(1)
                calibration_layout.addLayout(group_controls)
                group_status = QtWidgets.QLabel("PARAM readback 대기 중")
                group_status.setObjectName("statusStrip")
                group_status.setWordWrap(True)
                group_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                calibration_layout.addWidget(group_status)
                model = _table_model(self, ("Parameter", "Current", "Reset target", "Contract"))
                table = QtWidgets.QTableView()
                table.setModel(model)
                table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
                table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
                table.setAlternatingRowColors(True)
                table.verticalHeader().setVisible(False)
                table.setMinimumHeight(240)
                table.horizontalHeader().setStretchLastSection(True)
                table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
                table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
                table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
                calibration_layout.addWidget(table, 1)
                self.px4_parameter_group_models[side][calibration.key] = model
                self.px4_parameter_tables[side][calibration.key] = table
                self.px4_parameter_group_status_labels[side][calibration.key] = group_status
                self.px4_parameter_reset_buttons[side][calibration.key] = reset_button
                group_tabs.addTab(group_tab, calibration.label)
            tab_layout.addWidget(group_tabs, 1)
            tabs.addTab(tab, title)
        parameter_group_layout.addWidget(tabs)
        layout.addWidget(group)

    def _open_px4_shell(self, side: str) -> None:
        opener = getattr(self.backend, "advanced_open_px4_shell", None)
        if callable(opener):
            opener(str(side))
            return
        panel = getattr(self.backend, "panel", None)
        opener = getattr(panel, "open_px4_shell", None)
        if callable(opener):
            opener(str(side))

    def _refresh_px4_parameter_group(self, snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        mag_cache = getattr(panel, "_px4_mag_status_cache", {})
        attitude_cache = getattr(panel, "_px4_attitude_cal_status_cache", {})
        errors = getattr(panel, "_px4_calibration_parameter_error", {})
        reset_errors = getattr(panel, "_px4_calibration_reset_error", {})
        for side, label in (("left", "P1"), ("right", "P2")):
            mag_status = mag_cache.get(side) if isinstance(mag_cache, Mapping) else None
            attitude_status = attitude_cache.get(side) if isinstance(attitude_cache, Mapping) else None
            params: dict[str, object] = {}
            if attitude_status is not None:
                params.update(getattr(attitude_status, "params", {}) or {})
            if mag_status is not None:
                params.update(getattr(mag_status, "params", {}) or {})
            button = self.px4_parameter_refresh_buttons[side]
            state = snapshot.get(f"pixhawk.{side}")
            connected = state is not None and str(getattr(state, "state", "")).upper() in {
                "LIVE",
                "CONNECTED",
                "STARTING",
            }
            open_sides = getattr(panel, "_mavlink_open_sides", None)
            if callable(open_sides):
                connected = connected or side in set(open_sides())
            active = side in getattr(panel, "pixhawk_parameter_threads", {}) or side in getattr(
                panel, "pixhawk_shell_threads", {}
            )
            button.setEnabled(self._invoke_command_available and connected and not active and not self._recording_locked)
            if side in errors:
                status_text = f"{label}: PARAM readback failed · {errors[side]}"
            elif mag_status is None and attitude_status is None:
                status_text = f"{label}: PARAM readback 대기 중"
            else:
                status_text = (
                    f"{label}: QGC calibration 결과 표시 중 · "
                    "Accelerometer/Gyroscope/Compass/Level groups · PX4 sensor axes"
                )
            self.px4_parameter_status_labels[side].setText(status_text)
            for calibration in PX4_CALIBRATION_GROUPS:
                group_params = tuple(params.get(name) for name in calibration.parameter_names)
                rows = tuple(
                    (
                        name,
                        _format_px4_parameter_value(name, params.get(name)),
                        "PX4 firmware default" if calibration.resettable else "Manual / no reset",
                        calibration.contract,
                    )
                    for name in calibration.parameter_names
                )
                _sync_table_rows(self.px4_parameter_group_models[side][calibration.key], rows)
                reset_button = self.px4_parameter_reset_buttons[side][calibration.key]
                reset_button.setEnabled(
                    calibration.resettable
                    and self._invoke_command_available
                    and connected
                    and not active
                    and not self._recording_locked
                )
                group_error = reset_errors.get(f"{side}:{calibration.key}")
                if group_error:
                    group_status = f"{label}: reset failed · {group_error}"
                elif not params or all(value is None for value in group_params):
                    group_status = f"{label}: PARAM readback 대기 중"
                elif calibration.resettable:
                    group_status = (
                        f"{label}: {calibration.label} readback 완료 · "
                        "Reset은 PX4 firmware default를 사용합니다."
                    )
                else:
                    group_status = f"{label}: {calibration.label}은 수동 파라미터입니다."
                self.px4_parameter_group_status_labels[side][calibration.key].setText(group_status)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        evidence = _evidence(self.backend)
        connection = evidence.pixhawk_connection_candidates()
        pixhawk_discovery_status = str(connection.get("status", "NOT RUN") or "NOT RUN")
        self.discovery_status.set_value(
            pixhawk_discovery_status,
            valid=False if pixhawk_discovery_status.startswith("ERROR") else None,
        )
        candidates = connection.get("candidates")
        candidates = tuple(row for row in candidates if isinstance(row, dict)) if isinstance(candidates, (tuple, list)) else ()
        candidate_signature = tuple(
            tuple(row.get(key, "") for key in ("source", "source_row", "kind", "value", "baud", "status", "interface"))
            for row in candidates
        )
        if candidate_signature != self._candidate_signature:
            selected = self._selected_candidate_identity()
            self.candidate_model.removeRows(0, self.candidate_model.rowCount())
            for row in candidates:
                _append_table_row(
                    self.candidate_model,
                    tuple(row.get(key, "") for key in ("source", "kind", "value", "baud", "status", "interface")),
                )
            self._candidate_signature = candidate_signature
            self._candidate_rows = candidates
            if selected:
                for row_index, row in enumerate(candidates):
                    if (str(row.get("source", "")), str(row.get("kind", "")), str(row.get("value", ""))) == selected:
                        self.candidate_table.selectRow(row_index)
                        break
        discovery_active = bool(connection.get("discovery_active", False))
        recording = snapshot.get("recording")
        self._recording_locked = recording is not None and recording.state in {"ACTIVE", "FINALIZING"}
        panel = getattr(self.backend, "panel", self.backend)
        open_sides = getattr(panel, "_mavlink_open_sides", None)
        connected_sides = set(open_sides()) if callable(open_sides) else set()
        connection_in_progress = getattr(panel, "pixhawk_connection_thread", None) is not None
        status_connected = any(
            getattr(snapshot.get(f"pixhawk.{side}"), "state", "").upper()
            in {"LIVE", "CONNECTED", "STARTING"}
            for side in ("left", "right")
        )
        connection_active = bool(connected_sides) or connection_in_progress or status_connected
        self.operation_buttons["pixhawk.connect_mavlink"].setEnabled(
            self._invoke_command_available and not self._recording_locked and not connection_active
        )
        self.operation_buttons["pixhawk.disconnect_mavlink"].setEnabled(
            self._invoke_command_available and not self._recording_locked and connection_active
        )
        for side in ("left", "right"):
            self.operation_buttons[f"pixhawk.open_shell.{side}"].setEnabled(
                side in connected_sides and not self._recording_locked
            )
        reset_level_button = self.operation_buttons.get("pixhawk.reset_level_calibration")
        if reset_level_button is not None:
            reset_level_button.setEnabled(not self._recording_locked)
        recovery_feedback: list[str] = []
        for side, label in (("left", "P1"), ("right", "P2")):
            recovery = evidence.pixhawk_recovery(side)
            self.recovery_values[side]["usb"].set_value(
                recovery.get("usb_url", "--"),
                source="configured USB PX4 shell/control transport",
            )
            baud = recovery.get("usb_baud")
            self.recovery_values[side]["baud"].set_value(
                baud if baud is not None else "--",
                unit="baud" if baud is not None else "",
                source="configured USB bootstrap baud",
            )
            recovery_state = str(recovery.get("state", "IDLE") or "IDLE").upper()
            recovery_valid = True if recovery_state == "RECOVERED" else False if recovery_state == "ERROR" else None
            self.recovery_values[side]["recovery"].set_value(
                recovery_state,
                source=str(recovery.get("detail", "") or ""),
                age_s=recovery.get("updated_age_s"),
                valid=recovery_valid,
            )
            heartbeat_state = str(recovery.get("heartbeat_state", "NO LINK") or "NO LINK")
            attitude_state = str(recovery.get("attitude_state", "WAITING") or "WAITING")
            telemetry_valid = True if heartbeat_state == "OK" and attitude_state == "OK" else None
            if heartbeat_state == "NO LINK":
                telemetry_valid = False
            self.recovery_values[side]["telemetry"].set_value(
                f"HB {heartbeat_state} · ATT {attitude_state}",
                source=str(recovery.get("contract", "") or ""),
                valid=telemetry_valid,
            )
            recovery_feedback.append(f"{label}: {recovery_state} · {recovery.get('detail', '')}")
            self.recovery_buttons[side].setEnabled(
                self._invoke_command_available
                and not self._recording_locked
                and not bool(recovery.get("active", False))
            )
        self.recovery_feedback.setText(" | ".join(recovery_feedback))
        for command_id in ("pixhawk.discover_usb", "pixhawk.discover_host_ip"):
            self.operation_buttons[command_id].setEnabled(not discovery_active)
        self._refresh_candidate_actions()
        attitude_rows: list[tuple[object, ...]] = []
        receiver_rows: list[tuple[object, ...]] = []
        for side, label in (("left", "P1"), ("right", "P2")):
            attitude = evidence.attitude(side)
            attitude_rows.append(
                (
                    label,
                    attitude.get("state", "WAITING"),
                    _display_number(attitude.get("roll_deg")),
                    _display_number(attitude.get("pitch_deg")),
                    _display_number(attitude.get("yaw_deg")),
                    _display_number(attitude.get("age_s"), digits=2, suffix="s"),
                    _display_number(attitude.get("px4_timestamp_s"), digits=6),
                    attitude.get("mavlink_endpoint", "--") or "--",
                ),
            )
            gps = evidence.gps(side)
            receiver_rows.append(
                (
                    label,
                    gps.get("fix_type", "--"),
                    gps.get("rtk_label", gps.get("state", "NO DATA")),
                    gps.get("satellites", "--"),
                    _display_number(gps.get("eph_m")),
                    _display_number(gps.get("epv_m")),
                    _display_number(gps.get("age_s"), digits=2, suffix="s"),
                ),
            )
        _sync_table_rows(self.attitude_model, tuple(attitude_rows))
        _sync_table_rows(self.receiver_model, tuple(receiver_rows))
        magnetometer_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            for row in evidence.magnetometers(side):
                magnetometer_rows.append(
                    (
                        row.get("pixhawk", "--"),
                        row.get("imu", "--"),
                        row.get("state", "WAITING"),
                        _display_number(row.get("xmag")),
                        _display_number(row.get("ymag")),
                        _display_number(row.get("zmag")),
                        _display_number(row.get("norm")),
                        row.get("unit", "--"),
                        _display_number(row.get("age_s"), digits=2, suffix="s"),
                        row.get("topic", "--"),
                    )
                )
        _sync_table_rows(self.magnetometer_model, tuple(magnetometer_rows))
        can_magnetometer_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            for row in evidence.can_magnetometers(side):
                can_magnetometer_rows.append(
                    (
                        row.get("pixhawk", "--"),
                        row.get("can_port", "--"),
                        row.get("node_id", "--"),
                        row.get("device_id", "--"),
                        row.get("state", "WAITING"),
                        _display_number(row.get("xmag")),
                        _display_number(row.get("ymag")),
                        _display_number(row.get("zmag")),
                        _display_number(row.get("norm")),
                        row.get("unit", "--"),
                        _display_number(row.get("age_s"), digits=2, suffix="s"),
                        row.get("topic", "--"),
                    )
                )
        _sync_table_rows(self.can_magnetometer_model, tuple(can_magnetometer_rows))
        self._refresh_px4_parameter_group(snapshot)
        baseline = evidence.baseline()
        projection_state = str(baseline.get("p1_body_frd_state", "WAITING") or "WAITING")
        for key, value in self.baseline_values.items():
            raw = baseline.get(key)
            is_frd = key.startswith("p1_body_frd_")
            is_metric = key.endswith("_m")
            if is_frd:
                valid = None if projection_state == "WAITING" else projection_state == "READY"
                frame = "P1 body FRD [Forward,Right,Down]"
            else:
                valid = None if baseline.get("state") in {"NO DATA", "UNKNOWN"} else baseline.get("state") == "GOOD"
                frame = "local NED [North,East,Down]" if is_metric else ""
            value.set_value(
                _display_number(raw) if is_metric else raw,
                unit="m" if is_metric else "",
                frame=frame,
                valid=valid,
            )

    def _inspect_magnetometer_row(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0 or index.row() >= self.magnetometer_model.rowCount():
            return
        side = "left" if index.row() < len(MAVLINK_MAG_STREAMS) else "right"
        stream_index = index.row() % len(MAVLINK_MAG_STREAMS)
        rows = _evidence(self.backend).magnetometers(side)
        if stream_index >= len(rows):
            return
        row = rows[stream_index]
        self.inspection_requested.emit(
            _live_inspection(
                {
                    "selection": f"{row.get('pixhawk', '--')} {row.get('imu', '--')} magnetometer",
                    "value": (
                        f"x/y/z={_display_number(row.get('xmag'))}/"
                        f"{_display_number(row.get('ymag'))}/"
                        f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                        f"norm={_display_number(row.get('norm'))}"
                    ),
                    "freshness": f"age={_display_number(row.get('age_s'), digits=2, suffix='s')}",
                    "contract": str(row.get("axis_contract", "producer sensor axes; no coordinate conversion")),
                    "verdict": str(row.get("state", "WAITING")),
                    "source": str(row.get("source", "MAVLink SCALED_IMU telemetry")),
                },
                lambda side=side, stream_index=stream_index: self._magnetometer_inspection(side, stream_index),
            )
        )

    def _inspect_can_magnetometer_row(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0 or index.row() >= self.can_magnetometer_model.rowCount():
            return
        side = "left" if index.row() < 2 else "right"
        port_index = index.row() % 2
        rows = _evidence(self.backend).can_magnetometers(side)
        if port_index >= len(rows):
            return
        row = rows[port_index]
        self.inspection_requested.emit(
            _live_inspection(
                {
                    "selection": f"{row.get('pixhawk', '--')} {row.get('can_port', '--')} magnetometer",
                    "value": (
                        f"x/y/z={_display_number(row.get('xmag'))}/"
                        f"{_display_number(row.get('ymag'))}/"
                        f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                        f"node={row.get('node_id', '--')} device={row.get('device_id', '--')}"
                    ),
                    "freshness": f"age={_display_number(row.get('age_s'), digits=2, suffix='s')}",
                    "contract": str(row.get("axis_contract", "producer sensor axes; no coordinate conversion")),
                    "verdict": str(row.get("state", "WAITING")),
                    "source": str(row.get("source", "PX4 DEBUG_VECT CAN_MAG compatibility telemetry")),
                },
                lambda side=side, port_index=port_index: self._can_magnetometer_inspection(side, port_index),
            )
        )

    def _can_magnetometer_inspection(self, side: str, port_index: int) -> dict[str, object]:
        rows = _evidence(self.backend).can_magnetometers(side)
        if port_index < 0 or port_index >= len(rows):
            return {"selection": "CAN magnetometer", "verdict": "WAITING"}
        row = rows[port_index]
        return {
            "selection": f"{row.get('pixhawk', '--')} {row.get('can_port', '--')} magnetometer",
            "value": (
                f"x/y/z={_display_number(row.get('xmag'))}/"
                f"{_display_number(row.get('ymag'))}/"
                f"{_display_number(row.get('zmag'))} {row.get('unit', '')}"
            ),
            "freshness": f"age={_display_number(row.get('age_s'), digits=2, suffix='s')}",
            "contract": str(row.get("axis_contract", "producer sensor axes; no coordinate conversion")),
            "verdict": str(row.get("state", "WAITING")),
            "source": str(row.get("source", "PX4 DEBUG_VECT CAN_MAG compatibility telemetry")),
        }

    def _magnetometer_inspection(self, side: str, stream_index: int) -> dict[str, object]:
        rows = _evidence(self.backend).magnetometers(side)
        if stream_index < 0 or stream_index >= len(rows):
            return {"selection": "Magnetometer", "verdict": "WAITING"}
        row = rows[stream_index]
        return {
            "selection": f"{row.get('pixhawk', '--')} {row.get('imu', '--')} magnetometer",
            "value": (
                f"x/y/z={_display_number(row.get('xmag'))}/"
                f"{_display_number(row.get('ymag'))}/"
                f"{_display_number(row.get('zmag'))} {row.get('unit', '')}"
            ),
            "freshness": f"age={_display_number(row.get('age_s'), digits=2, suffix='s')}",
            "contract": str(row.get("axis_contract", "producer sensor axes; no coordinate conversion")),
            "verdict": str(row.get("state", "WAITING")),
            "source": str(row.get("source", "MAVLink SCALED_IMU telemetry")),
        }

    def _selected_candidate_row(self) -> int:
        selected = self.candidate_table.selectionModel().selectedRows()
        return selected[0].row() if selected else -1

    def _selected_candidate_identity(self) -> tuple[str, str, str] | None:
        row = self._selected_candidate_row()
        if not 0 <= row < len(self._candidate_rows):
            return None
        candidate = self._candidate_rows[row]
        return str(candidate.get("source", "")), str(candidate.get("kind", "")), str(candidate.get("value", ""))

    def _refresh_candidate_actions(self, *_args: object) -> None:
        row = self._selected_candidate_row()
        candidate = self._candidate_rows[row] if 0 <= row < len(self._candidate_rows) else {}
        kind = str(candidate.get("kind", ""))
        source = str(candidate.get("source", ""))
        self.candidate_buttons["host"].setEnabled(
            not self._recording_locked and source == "pc" and kind == "PC_IP"
        )
        endpoint_enabled = source == "mavlink" and kind in {"USB", "MAVLink"}
        self.candidate_buttons["left"].setEnabled(not self._recording_locked and endpoint_enabled)
        self.candidate_buttons["right"].setEnabled(not self._recording_locked and endpoint_enabled)

    def _apply_candidate(self, target: str) -> None:
        row = self._selected_candidate_row()
        if not 0 <= row < len(self._candidate_rows):
            self.candidate_feedback.setText("ERROR: select a compatible endpoint")
            return
        candidate = self._candidate_rows[row]
        method = getattr(self.backend, "pixhawk_apply_candidate", None)
        if not callable(method):
            self.candidate_feedback.setText("ERROR: Pixhawk endpoint adapter unavailable")
            return
        try:
            applied = method(str(candidate.get("source", "")), int(candidate.get("source_row", -1)), target)
        except Exception as exc:
            self.candidate_feedback.setText(f"ERROR: {exc}")
            return
        if applied is False:
            self.candidate_feedback.setText("ERROR: endpoint was not applied")
            return
        self.candidate_feedback.setText(f"Applied {candidate.get('kind', 'endpoint')} to {target.upper()}")
        self.refresh(self.status_model.snapshot())

    def _inspect_candidate_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        self.inspection_requested.emit(
            _live_inspection(
                self._candidate_inspection(row),
                lambda row=row: self._candidate_inspection(row),
            )
        )

    def _candidate_inspection(self, row: int) -> dict[str, object]:
        candidates = _evidence(self.backend).pixhawk_connection_candidates().get("candidates")
        candidates = candidates if isinstance(candidates, (tuple, list)) else ()
        candidate = dict(candidates[row]) if 0 <= row < len(candidates) and isinstance(candidates[row], dict) else {}
        return {
            "selection": f"Pixhawk endpoint · {candidate.get('kind', '--')}",
            "value": (
                f"value={candidate.get('value', '--')} · baud={candidate.get('baud', '--')} · "
                f"discovery path={candidate.get('source', '--')}"
            ),
            "freshness": "Latest completed Pixhawk endpoint discovery",
            "contract": "Host/serial/MAVLink endpoint metadata; no attitude, GPS position, or baseline values",
            "verdict": candidate.get("status") or "UNVERIFIED",
            "source": "Pixhawk discovery workers",
        }

    def _inspect_attitude_row(self, index: QtCore.QModelIndex) -> None:
        side = "left" if index.row() == 0 else "right"
        self.inspection_requested.emit(
            _live_inspection(
                self._attitude_inspection(side),
                lambda side=side: self._attitude_inspection(side),
            )
        )

    def _attitude_inspection(self, side: str) -> dict[str, object]:
        label = "P1" if side == "left" else "P2"
        data = _evidence(self.backend).attitude(side)
        return {
            "selection": f"{label} attitude",
            "value": (
                f"R/P/Y={_display_number(data.get('roll_deg'))}/"
                f"{_display_number(data.get('pitch_deg'))}/{_display_number(data.get('yaw_deg'))}°"
            ),
            "freshness": f"age={_display_number(data.get('age_s'), digits=2, suffix='s')} · PX4 t={data.get('px4_timestamp_s', '--')}",
            "contract": str(data.get("orientation_contract", "PX4 body FRD → world NED; R/P/Y degrees")),
            "verdict": str(data.get("state", "WAITING")),
            "source": str(data.get("source", "MAVLink attitude telemetry")),
        }

    def _inspect_receiver_row(self, index: QtCore.QModelIndex) -> None:
        side = "left" if index.row() == 0 else "right"
        self.inspection_requested.emit(
            _live_inspection(
                self._receiver_inspection(side),
                lambda side=side: self._receiver_inspection(side),
            )
        )

    def _receiver_inspection(self, side: str) -> dict[str, object]:
        label = "P1" if side == "left" else "P2"
        data = _evidence(self.backend).gps(side)
        return {
            "selection": f"{label} GNSS receiver",
            "value": (
                f"fix number={data.get('fix_type', '--')} · {data.get('rtk_label', 'NO DATA')} · "
                f"sats={data.get('satellites', '--')} · eph/epv={data.get('eph_m', '--')}/{data.get('epv_m', '--')} m"
            ),
            "freshness": f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}",
            "contract": str(data.get("position_contract", "WGS84 latitude/longitude degrees; altitude metres")),
            "verdict": str(data.get("state", "NO DATA")),
            "source": str(data.get("source", "MAVLink GPS telemetry")),
        }

    def _inspect_baseline_value(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._baseline_inspection(key),
                lambda key=key: self._baseline_inspection(key),
            )
        )

    def _baseline_inspection(self, key: str) -> dict[str, object]:
        data = _evidence(self.backend).baseline()
        is_frd = key.startswith("p1_body_frd_")
        contract = (
            str(data.get("p1_body_frd_axis_contract", "P1→P2 in P1 body FRD [Forward,Right,Down]; metres"))
            if is_frd
            else str(data.get("axis_contract", "P1→P2 local NED [North,East,Down]; metres"))
        )
        freshness = (
            f"P1 attitude age={data.get('p1_body_frd_attitude_age_s', '--')}s"
            if is_frd
            else f"P1/P2 age={data.get('left_age_s', '--')}/{data.get('right_age_s', '--')}s"
        )
        verdict = data.get("p1_body_frd_state", "WAITING") if is_frd else data.get("state", "UNKNOWN")
        return {
            "selection": self.baseline_values[key].label.text(),
            "value": f"{_display_number(data.get(key))}{' m' if key.endswith('_m') else ''}",
            "freshness": freshness,
            "contract": contract,
            "verdict": verdict,
            "source": data.get("p1_body_frd_source" if is_frd else "source", "baseline evidence"),
        }


class TelemetryWorkspace(QtWidgets.QWidget):
    """Active Drone Pixhawk telemetry, RTCM delivery, and disparity reference."""

    def __init__(
        self,
        backend: object,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self._invoke_command = invoke_command
        self._invoke_command_available = invoke_command is not None
        self._last_snapshot: dict[str, object] = {}
        defaults = self._settings_snapshot()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Telemetry"))

        connection_group = QtWidgets.QGroupBox("Drone Pixhawk MAVLink")
        connection_form = QtWidgets.QGridLayout(connection_group)
        connection_form.setColumnStretch(1, 1)
        self.endpoint_edit = QtWidgets.QLineEdit(
            str(defaults.get("drone_telemetry_url", "udpin:0.0.0.0:14552") or "")
        )
        self.endpoint_edit.setPlaceholderText("udpin:0.0.0.0:14552 / serial / tcp")
        self.baud_spin = QtWidgets.QSpinBox()
        self.baud_spin.setRange(1_200, 4_000_000)
        self.baud_spin.setSingleStep(9_600)
        self.baud_spin.setValue(int(defaults.get("drone_telemetry_baud", 57_600) or 57_600))
        self.rtcm_source_combo = QtWidgets.QComboBox()
        self.rtcm_source_combo.addItem("Base GNSS", "base")
        self.rtcm_source_combo.addItem("Network NTRIP", "ntrip")
        rtcm_source = str(defaults.get("drone_telemetry_rtcm_source", "base") or "").strip().lower()
        source_index = self.rtcm_source_combo.findData(rtcm_source)
        self.rtcm_source_combo.setCurrentIndex(max(0, source_index))
        self.forward_rtcm_checkbox = QtWidgets.QCheckBox("Forward selected RTCM source to this Pixhawk")
        self.forward_rtcm_checkbox.setChecked(bool(defaults.get("drone_telemetry_rtcm_forward", True)))
        connection_form.addWidget(QtWidgets.QLabel("MAVLink URL"), 0, 0)
        connection_form.addWidget(self.endpoint_edit, 0, 1, 1, 3)
        connection_form.addWidget(QtWidgets.QLabel("Baud"), 1, 0)
        connection_form.addWidget(self.baud_spin, 1, 1)
        connection_form.addWidget(QtWidgets.QLabel("RTCM source"), 2, 0)
        connection_form.addWidget(self.rtcm_source_combo, 2, 1)
        connection_form.addWidget(self.forward_rtcm_checkbox, 2, 2, 1, 2)
        actions = QtWidgets.QHBoxLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("telemetry.discover_usb", "Find USB"),
            ("telemetry.apply", "Apply"),
            ("telemetry.connect", "Connect"),
            ("telemetry.disconnect", "Disconnect"),
            ("telemetry.save_settings", "Save"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            actions.addWidget(button)
        actions.addStretch(1)
        connection_form.addLayout(actions, 3, 0, 1, 4)
        self.connection_status = QtWidgets.QLabel("DISCONNECTED")
        self.connection_status.setObjectName("statusStrip")
        self.connection_status.setWordWrap(True)
        connection_form.addWidget(self.connection_status, 4, 0, 1, 4)
        layout.addWidget(connection_group)

        position_group = QtWidgets.QGroupBox("Live position")
        position_layout = QtWidgets.QVBoxLayout(position_group)
        self.position_model = _table_model(
            self,
            ("Source", "State", "Fix", "RTK", "Latitude", "Longitude", "Alt m", "Sats", "Age"),
        )
        self.position_table = QtWidgets.QTableView()
        self.position_table.setModel(self.position_model)
        self.position_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.position_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.position_table.verticalHeader().setVisible(False)
        self.position_table.setMaximumHeight(150)
        self.position_table.horizontalHeader().setStretchLastSection(True)
        position_layout.addWidget(self.position_table)
        position_hint = QtWidgets.QLabel(
            "GPS_RAW_INT/GPS2_RAW position remains WGS84 latitude/longitude with MAVLink MSL altitude; "
            "no camera-frame conversion is applied."
        )
        position_hint.setObjectName("statusStrip")
        position_hint.setWordWrap(True)
        position_layout.addWidget(position_hint)
        layout.addWidget(position_group)

        rtcm_group = QtWidgets.QGroupBox("RTCM delivery")
        rtcm_form = QtWidgets.QGridLayout(rtcm_group)
        self.rtcm_values: dict[str, QtWidgets.QLabel] = {}
        for row, (key, title) in enumerate(
            (
                ("route", "Route"),
                ("frames", "Frames"),
                ("bytes", "Bytes"),
                ("packets", "MAVLink packets"),
                ("fail", "Failures"),
                ("age", "Last delivery age"),
            )
        ):
            label = QtWidgets.QLabel("--")
            label.setObjectName("statusStrip")
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.rtcm_values[key] = label
            rtcm_form.addWidget(QtWidgets.QLabel(title), row // 3, (row % 3) * 2)
            rtcm_form.addWidget(label, row // 3, (row % 3) * 2 + 1)
        layout.addWidget(rtcm_group)

        disparity_group = QtWidgets.QGroupBox("Ideal disparity from three GNSS positions + zoom")
        disparity_form = QtWidgets.QGridLayout(disparity_group)
        self.disparity_values: dict[str, QtWidgets.QLabel] = {}
        for row, (key, title) in enumerate(
            (
                ("state", "State"),
                ("baseline", "Live GPS baseline"),
                ("focal", "Focal length"),
                ("target", "GNSS target range"),
                ("disparity", "Ideal disparity"),
                ("reason", "Reason"),
            )
        ):
            label = QtWidgets.QLabel("--")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.disparity_values[key] = label
            disparity_form.addWidget(QtWidgets.QLabel(title), row, 0)
            disparity_form.addWidget(label, row, 1)
        assumption = QtWidgets.QLabel(
            "Model: B = distance(P1 GNSS, P2 GNSS), Z = distance(P1/P2 baseline midpoint, Drone GNSS), "
            "fx = active WIRIS zoom focal length, d = fx·B/Z. This is a theoretical reference value; "
            "it is not the observed YOLO/image disparity and does not estimate target bearing or pose."
        )
        assumption.setObjectName("compactContract")
        assumption.setWordWrap(True)
        disparity_form.addWidget(assumption, 6, 0, 1, 2)
        layout.addWidget(disparity_group)
        layout.addStretch(1)


    def _settings_snapshot(self) -> dict[str, object]:
        reader = getattr(self.backend, "workstation_settings_snapshot", None)
        if not callable(reader):
            return {}
        try:
            value = reader()
        except Exception:
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def payload(self) -> dict[str, object]:
        return {
            "drone_telemetry_url": self.endpoint_edit.text().strip(),
            "drone_telemetry_baud": int(self.baud_spin.value()),
            "drone_telemetry_rtcm_source": str(self.rtcm_source_combo.currentData() or "base"),
            "drone_telemetry_rtcm_forward": bool(self.forward_rtcm_checkbox.isChecked()),
        }

    def _snapshot(self) -> dict[str, object]:
        reader = getattr(self.backend, "drone_telemetry_snapshot", None)
        if not callable(reader):
            return {}
        try:
            value = reader()
        except Exception:
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def refresh(self, _status_snapshot: dict[str, RuntimeStatus] | None = None) -> None:
        data = self._snapshot()
        self._last_snapshot = data
        if not data:
            data = self._settings_snapshot()
        endpoint = str(data.get("endpoint", data.get("drone_telemetry_url", "")) or "")
        if endpoint and not self.endpoint_edit.hasFocus() and self.endpoint_edit.text() != endpoint:
            self.endpoint_edit.setText(endpoint)
        if "baud" in data and not self.baud_spin.hasFocus():
            self.baud_spin.setValue(int(data.get("baud") or self.baud_spin.value()))
        if "forward_rtcm" in data:
            self.forward_rtcm_checkbox.setChecked(bool(data.get("forward_rtcm")))
        if "rtcm_source" in data:
            source_index = self.rtcm_source_combo.findData(str(data.get("rtcm_source") or "base").lower())
            if source_index >= 0 and not self.rtcm_source_combo.hasFocus():
                self.rtcm_source_combo.setCurrentIndex(source_index)
        state = str(data.get("connection_state", "DISCONNECTED") or "DISCONNECTED")
        heartbeat = "heartbeat=OK" if data.get("heartbeat_received") else "heartbeat=WAITING"
        error = str(data.get("last_error") or "")
        rtcm_source_label = str(data.get("rtcm_source_label", "Base GNSS") or "Base GNSS")
        usb_discovery_state = str(data.get("usb_discovery_state") or "")
        self.connection_status.setText(
            f"{state} · {heartbeat} · RTCM={rtcm_source_label if data.get('forward_rtcm', True) else 'OFF'} · "
            f"endpoint={endpoint or '--'}"
            + (f" · {error}" if error else "")
            + (f" · USB 탐색: {usb_discovery_state}" if usb_discovery_state else "")
        )
        connection_active = state in {"CONNECTING", "CONNECTED", "LIVE"}
        usb_discovery_active = bool(data.get("usb_discovery_active"))
        self.operation_buttons["telemetry.connect"].setEnabled(
            self._invoke_command_available and not connection_active
        )
        self.operation_buttons["telemetry.disconnect"].setEnabled(
            self._invoke_command_available and connection_active
        )
        self.operation_buttons["telemetry.discover_usb"].setEnabled(
            self._invoke_command_available and not usb_discovery_active
        )
        self._refresh_position_table(data)
        self._refresh_rtcm(data)
        self._refresh_disparity(data)

    def _refresh_position_table(self, data: Mapping[str, object]) -> None:
        rows = []
        for key, label in (
            ("left_position", "Left Pixhawk"),
            ("right_position", "Right Pixhawk"),
            ("position", "Drone Pixhawk"),
        ):
            row = data.get(key)
            row = row if isinstance(row, Mapping) else {}
            rows.append(
                (
                    label,
                    row.get("state", "NO DATA"),
                    row.get("fix_type", "--"),
                    row.get("rtk_label", "UNKNOWN"),
                    _display_number(row.get("latitude_deg"), digits=8),
                    _display_number(row.get("longitude_deg"), digits=8),
                    _display_number(row.get("altitude_m"), digits=3),
                    row.get("satellites", "--"),
                    _display_number(row.get("age_s"), digits=2, suffix="s"),
                )
            )
        _sync_table_rows(self.position_model, tuple(rows))

    def _refresh_rtcm(self, data: Mapping[str, object]) -> None:
        rtcm = data.get("rtcm")
        rtcm = rtcm if isinstance(rtcm, Mapping) else {}
        connected = str(data.get("connection_state", "")) in {"CONNECTED", "LIVE"}
        enabled = bool(data.get("forward_rtcm", False))
        source_label = str(data.get("rtcm_source_label", "Base GNSS") or "Base GNSS")
        self.rtcm_values["route"].setText(
            f"enabled · {source_label} RTCM → Drone Pixhawk"
            if connected and enabled
            else "disabled / endpoint not live"
        )
        self.rtcm_values["frames"].setText(str(rtcm.get("frames", 0)))
        self.rtcm_values["bytes"].setText(str(rtcm.get("bytes", 0)))
        self.rtcm_values["packets"].setText(str(rtcm.get("packets", 0)))
        self.rtcm_values["fail"].setText(str(rtcm.get("fail", 0)))
        self.rtcm_values["age"].setText(_display_number(rtcm.get("last_delivery_age_s"), digits=2, suffix="s"))

    def _refresh_disparity(self, data: Mapping[str, object]) -> None:
        ideal = data.get("ideal_disparity")
        ideal = ideal if isinstance(ideal, Mapping) else {}
        baseline_m = ideal.get("baseline_m")
        focal = data.get("focal_length_px", ideal.get("focal_length_px"))
        target = ideal.get("target_distance_m", data.get("target_distance_m"))
        try:
            target = float(target) if target is not None else None
        except (TypeError, ValueError):
            target = None
        target_source = str(data.get("target_distance_source", "three GNSS positions") or "three GNSS positions")
        disparity = ideal.get("disparity_px")
        state = str(ideal.get("state", "WAITING") or "WAITING")
        reason = str(ideal.get("reason", "") or "")
        self.disparity_values["state"].setText(state)
        self.disparity_values["baseline"].setText(
            "--"
            if baseline_m is None
            else f"{float(baseline_m):.4f} m · Δt={_display_number(ideal.get('timestamp_delta_ms'), digits=1, suffix='ms')}"
        )
        self.disparity_values["focal"].setText(
            "--" if focal is None else f"{float(focal):.3f} px · {data.get('focal_source', '--')}"
        )
        self.disparity_values["target"].setText(
            "--" if target is None or target <= 0.0 else f"{target:.3f} m · {target_source}"
        )
        self.disparity_values["disparity"].setText(
            "--" if disparity is None else f"{disparity:.3f} px"
        )
        self.disparity_values["reason"].setText(reason or "--")


class BaseGnssWorkspace(QtWidgets.QWidget):
    """Current Base GNSS, Survey-in, NTRIP, and RTCM distribution state."""

    inspection_requested = QtCore.pyqtSignal(object)

    MODE_LABELS = {
        "LOCAL_SURVEY_IN": "Local Survey-in",
        "NETWORK_ASSISTED_FIXED_BASE": "NTRIP-assisted Fixed Base",
        "NETWORK_RTK_DIRECT": "Network RTK Direct",
    }

    def __init__(
        self,
        backend: object,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self._ntrip_was_connected = False
        self._ntrip_connect_monotonic_s: float | None = None
        self._ntrip_pre_connect_fix_text = "--"
        self._ntrip_fixed_after_connect_s: float | None = None
        self._last_state_text = "--"
        self._last_base_fix: bool | None = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Base GNSS"))

        summary_group = QtWidgets.QGroupBox("Current state")
        summary_layout = QtWidgets.QGridLayout(summary_group)
        self.summary_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (("mode", "Mode"), ("state", "Operation"), ("source", "Correction source"), ("rtcm", "RTCM stream"))
        ):
            value = EngineeringValue(label)
            self.summary_values[key] = value
            summary_layout.addWidget(value, index // 2, index % 2)
        layout.addWidget(summary_group)

        self.serial_group = QtWidgets.QGroupBox("Base receiver")
        serial_layout = QtWidgets.QGridLayout(self.serial_group)
        self.serial_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate((("connection", "Connection"), ("device", "Device"), ("baud", "Baud"))):
            value = EngineeringValue(label)
            self.serial_values[key] = value
            serial_layout.addWidget(value, 0, index)
        layout.addWidget(self.serial_group)

        self.candidate_group = QtWidgets.QGroupBox("Discovered Base receivers")
        candidate_layout = QtWidgets.QVBoxLayout(self.candidate_group)
        self.discovery_status = EngineeringValue("Discovery")
        candidate_layout.addWidget(self.discovery_status)
        self.candidate_model = _table_model(self, ("Port", "Target", "Baud", "Status", "UBX messages"))
        self.candidate_table = QtWidgets.QTableView()
        self.candidate_table.setModel(self.candidate_model)
        self.candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.candidate_table.verticalHeader().setVisible(False)
        self.candidate_table.setMaximumHeight(132)
        self.candidate_table.horizontalHeader().setStretchLastSection(True)
        self.candidate_table.clicked.connect(self._inspect_candidate_row)
        self.candidate_table.selectionModel().selectionChanged.connect(self._refresh_candidate_action)
        candidate_layout.addWidget(self.candidate_table)
        candidate_actions = QtWidgets.QHBoxLayout()
        self.apply_candidate_button = QtWidgets.QPushButton("Use selected receiver")
        self.apply_candidate_button.setEnabled(False)
        self.apply_candidate_button.clicked.connect(self._apply_selected_candidate)
        candidate_actions.addWidget(self.apply_candidate_button)
        candidate_actions.addStretch(1)
        candidate_layout.addLayout(candidate_actions)
        self.candidate_feedback = QtWidgets.QLabel("")
        self.candidate_feedback.setObjectName("statusStrip")
        self.candidate_feedback.setWordWrap(True)
        candidate_layout.addWidget(self.candidate_feedback)
        layout.addWidget(self.candidate_group)
        self._candidate_signature: tuple[tuple[object, ...], ...] = ()

        self.survey_group = QtWidgets.QGroupBox("Survey-in")
        survey_layout = QtWidgets.QGridLayout(self.survey_group)
        self.survey_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("status", "Status"),
                ("duration", "Duration"),
                ("accuracy", "Mean accuracy"),
                ("observations", "Observations"),
                ("ecef", "Mean ECEF X/Y/Z"),
            )
        ):
            value = EngineeringValue(label)
            self.survey_values[key] = value
            survey_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(self.survey_group)

        self.ntrip_group = QtWidgets.QGroupBox("NTRIP")
        ntrip_layout = QtWidgets.QGridLayout(self.ntrip_group)
        self.ntrip_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("connection", "Connection"),
                ("health", "Health"),
                ("caster", "Caster"),
                ("mountpoint", "Mountpoint"),
                ("gga", "GGA policy"),
                ("status", "Caster response"),
                ("transport", "Transport"),
                ("rtcm", "RTCM"),
                ("reconnects", "Reconnects"),
                ("error", "Last error"),
            )
        ):
            value = EngineeringValue(label)
            self.ntrip_values[key] = value
            ntrip_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(self.ntrip_group)

        self.fixed_group = QtWidgets.QGroupBox("Fixed Base acquisition")
        fixed_layout = QtWidgets.QGridLayout(self.fixed_group)
        self.fixed_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("position", "WGS84 position"),
                ("height", "Height"),
                ("fix", "Base fix"),
                ("samples", "Samples"),
                ("scatter", "Scatter H/V"),
                ("reconfigure", "Apply verification"),
                ("ntrip_delta", "NTRIP 연결 전/후"),
            )
        ):
            value = EngineeringValue(label)
            self.fixed_values[key] = value
            fixed_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(self.fixed_group)

        self.checklist_group = QtWidgets.QGroupBox("Base-assisted checklist (informational only)")
        checklist_layout = QtWidgets.QGridLayout(self.checklist_group)
        self.checklist_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("base_coordinate", "1. Base coordinate confirmed"),
                ("base_rtcm", "2. Base RTCM output"),
                ("left_fixed", "3. Left Rover RTK Fixed"),
                ("right_fixed", "4. Right Rover RTK Fixed"),
                ("stabilized", "5. L/R Fixed stabilized"),
            )
        ):
            value = EngineeringValue(label)
            self.checklist_values[key] = value
            checklist_layout.addWidget(value, index // 2, index % 2)
        checklist_note = QtWidgets.QLabel("표시 전용 상태 · 이 항목들은 측정 시작/진행을 차단하지 않습니다.")
        checklist_note.setObjectName("statusStrip")
        checklist_note.setWordWrap(True)
        checklist_layout.addWidget(checklist_note, 3, 0, 1, 2)
        layout.addWidget(self.checklist_group)

        route_group = QtWidgets.QGroupBox("RTCM distribution")
        route_layout = QtWidgets.QVBoxLayout(route_group)
        self.route_model = _table_model(self, ("Target", "Frames", "Packets", "Bytes", "Failures", "Last delivery"))
        self.route_table = QtWidgets.QTableView()
        self.route_table.setModel(self.route_model)
        self.route_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.route_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.route_table.verticalHeader().setVisible(False)
        self.route_table.setMaximumHeight(112)
        self.route_table.horizontalHeader().setStretchLastSection(True)
        self.route_table.clicked.connect(self._inspect_route_row)
        route_layout.addWidget(self.route_table)
        layout.addWidget(route_group)

        for group_name, group_values in (
            ("summary", self.summary_values),
            ("serial", self.serial_values),
            ("survey", self.survey_values),
            ("ntrip", self.ntrip_values),
            ("fixed", self.fixed_values),
            ("checklist", self.checklist_values),
        ):
            for key, value in group_values.items():
                value.activated.connect(
                    lambda group_name=group_name, key=key: self._inspect_value(group_name, key)
                )

        actions = QtWidgets.QGridLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for index, (command_id, label) in enumerate(
            (
                ("gnss.base_discover", "Find Base"),
                ("gnss.base_connect", "Connect"),
                ("gnss.base_disconnect", "Disconnect"),
                ("gnss.survey_start", "Start Survey-in"),
                ("gnss.survey_stop", "Stop Survey-in"),
                ("gnss.ntrip_start", "Start NTRIP"),
                ("gnss.ntrip_stop", "Stop NTRIP"),
                ("gnss.ntrip_forget_password", "Forget NTRIP password"),
            )
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            actions.addWidget(button, index // 4, index % 4)
        layout.addLayout(actions)
        layout.addStretch(1)

    @staticmethod
    def _dict(value: object) -> dict[str, object]:
        return dict(value) if isinstance(value, dict) else {}

    def refresh(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        data = _evidence(self.backend).gnss_operation()
        mode = str(data.get("mode", "UNKNOWN") or "UNKNOWN")
        state = str(data.get("state", "UNKNOWN") or "UNKNOWN")
        source = self._dict(data.get("source_identity"))
        serial = self._dict(data.get("base_serial"))
        survey = self._dict(data.get("survey_in"))
        ntrip = self._dict(data.get("ntrip"))
        coordinate = self._dict(data.get("base_coordinate"))
        fixed = self._dict(data.get("fixed_base"))
        profile = self._dict(fixed.get("profile"))
        base_discovery_status = str(data.get("base_discovery_status", "NOT RUN") or "NOT RUN")
        self.discovery_status.set_value(
            base_discovery_status,
            valid=False if base_discovery_status.startswith("ERROR") else None,
        )

        candidates = data.get("base_candidates")
        candidates = tuple(row for row in candidates if isinstance(row, dict)) if isinstance(candidates, (tuple, list)) else ()
        candidate_signature = tuple(
            (
                row.get("port", ""),
                row.get("target", ""),
                row.get("baud"),
                row.get("status", ""),
                row.get("messages", ""),
                bool(row.get("ubx", False)),
            )
            for row in candidates
        )
        if candidate_signature != self._candidate_signature:
            selected_port = self._selected_candidate_port()
            self.candidate_model.removeRows(0, self.candidate_model.rowCount())
            for row in candidates:
                _append_table_row(
                    self.candidate_model,
                    (
                        row.get("port", ""),
                        row.get("target", ""),
                        row.get("baud") or "--",
                        row.get("status", ""),
                        row.get("messages", ""),
                    ),
                )
            self._candidate_signature = candidate_signature
            if selected_port:
                for row_index, row in enumerate(candidates):
                    if str(row.get("port", "")) == selected_port:
                        self.candidate_table.selectRow(row_index)
                        break

        source_text = str(source.get("source_kind", "") or "--")
        if source.get("caster_host"):
            source_text = f"{source.get('caster_host')}:{source.get('caster_port', '--')}/{source.get('mountpoint', '')}"
        elif source.get("base_device"):
            source_text = str(source.get("base_device"))
        frame_age = data.get("last_frame_age_s")
        frame_text = (
            f"{data.get('base_frames', 0)} frames · {_display_number(data.get('average_frame_rate_hz'), digits=2)} Hz"
            f" · age {_display_number(frame_age, digits=2, suffix='s')}"
        )
        self.summary_values["mode"].set_value(self.MODE_LABELS.get(mode, mode))
        self.summary_values["state"].set_value(state, valid=False if state in {"FAULT", "INVALIDATED"} else None)
        self.summary_values["source"].set_value(source_text)
        self.summary_values["rtcm"].set_value(frame_text, age_s=float(frame_age) if frame_age not in (None, "") else None)

        serial_connected = bool(serial.get("connected", False))
        self.serial_values["connection"].set_value(
            "CONNECTED" if serial_connected else "DISCONNECTED",
            valid=True if serial_connected else None,
        )
        self.serial_values["device"].set_value(serial.get("device") or "--")
        self.serial_values["baud"].set_value(serial.get("baud") or "--")

        survey_active = bool(survey.get("active", False))
        survey_valid = bool(survey.get("valid", False))
        survey_state = "VALID" if survey_valid else "ACTIVE" if survey_active else "NOT RUNNING"
        self.survey_values["status"].set_value(survey_state, valid=True if survey_valid else None)
        self.survey_values["duration"].set_value(
            f"{survey.get('duration_s', 0)} / {_display_number(survey.get('target_duration_s'), digits=0)}",
            unit="s",
        )
        self.survey_values["accuracy"].set_value(
            f"{_display_number(survey.get('mean_accuracy_m'), digits=4)} / {_display_number(survey.get('target_accuracy_m'), digits=4)}",
            unit="m",
        )
        self.survey_values["observations"].set_value(survey.get("observations") or "--")
        ecef = survey.get("mean_ecef_m")
        ecef_text = "--"
        if isinstance(ecef, (tuple, list)) and len(ecef) == 3 and all(value is not None for value in ecef):
            ecef_text = "/".join(f"{float(value):.3f}" for value in ecef)
        self.survey_values["ecef"].set_value(ecef_text, unit="m", frame="ECEF [X,Y,Z]")

        ntrip_connected = bool(ntrip.get("connected", False))
        ntrip_running = bool(ntrip.get("running", False))
        ntrip_health = str(ntrip.get("health_state", "DISCONNECTED") or "DISCONNECTED")
        self.ntrip_values["connection"].set_value(
            "CONNECTED" if ntrip_connected else "DISCONNECTED",
            valid=True if ntrip_connected else None,
        )
        self.ntrip_values["health"].set_value(
            ntrip_health,
            valid=True if ntrip_health == "RTCM_ACTIVE" else False if ntrip_health in {"ERROR", "STALE", "RTCM_STALE"} else None,
        )
        tls_suffix = " · TLS" if ntrip.get("tls") else ""
        self.ntrip_values["caster"].set_value(f"{ntrip.get('caster_host', '--')}:{ntrip.get('caster_port', '--')}{tls_suffix}")
        self.ntrip_values["mountpoint"].set_value(ntrip.get("mountpoint") or "--")
        self.ntrip_values["gga"].set_value(ntrip.get("gga_policy") or "--")
        self.ntrip_values["status"].set_value(ntrip.get("status") or "--")
        transport_age = ntrip.get("last_chunk_age_s")
        self.ntrip_values["transport"].set_value(
            f"chunks={ntrip.get('chunks_received', 0)} · bytes={ntrip.get('bytes_received', 0)} · "
            f"rate={_display_number(ntrip.get('byte_rate_bps'), digits=1)} B/s · "
            f"last={_display_number(transport_age, digits=2, suffix='s')}",
            age_s=float(transport_age) if transport_age not in (None, "") else None,
            source="NtripStreamStats",
        )
        rtcm_age = ntrip.get("rtcm_age_s")
        message_types = ntrip.get("rtcm_message_types")
        type_text = "--"
        if isinstance(message_types, dict) and message_types:
            type_text = ", ".join(
                f"{key}:{value}"
                for key, value in sorted(message_types.items(), key=lambda item: str(item[0]))[:6]
            )
        self.ntrip_values["rtcm"].set_value(
            f"frames={ntrip.get('rtcm_frames', data.get('base_frames', 0))} · "
            f"age={_display_number(rtcm_age, digits=2, suffix='s')} · types={type_text}",
            age_s=float(rtcm_age) if rtcm_age not in (None, "") else None,
            source="Base RTCM parser",
        )
        self.ntrip_values["reconnects"].set_value(ntrip.get("reconnect_count", 0))
        self.ntrip_values["error"].set_value(
            ntrip.get("last_error") or "--",
            valid=False if ntrip.get("last_error") else None,
        )

        latitude = coordinate.get("latitude_deg")
        longitude = coordinate.get("longitude_deg")
        position_text = "--" if latitude is None or longitude is None else f"{float(latitude):.7f}, {float(longitude):.7f}"
        self.fixed_values["position"].set_value(position_text, frame="WGS84 geodetic latitude/longitude degrees")
        self.fixed_values["height"].set_value(
            _display_number(coordinate.get("selected_height_m")),
            unit="m",
            frame=str(coordinate.get("height_type", fixed.get("height_type", "UNKNOWN"))),
        )
        base_fix = coordinate.get("fixed")
        self.fixed_values["fix"].set_value("--" if base_fix is None else "FIXED" if base_fix else "NOT FIXED", valid=base_fix)
        self.fixed_values["samples"].set_value(fixed.get("sample_count", 0))
        self.fixed_values["scatter"].set_value(
            f"{_display_number(profile.get('horizontal_scatter_m'))} / {_display_number(profile.get('vertical_scatter_m'))}",
            unit="m",
        )
        verification = "/".join(
            "OK" if fixed.get(key) else "WAIT"
            for key in ("reconfigure_ack", "reconfigure_readback", "reconfigure_rtcm")
        )
        self.fixed_values["reconfigure"].set_value(verification)

        now_monotonic_s = time.monotonic()
        if ntrip_connected and not self._ntrip_was_connected:
            self._ntrip_connect_monotonic_s = now_monotonic_s
            self._ntrip_pre_connect_fix_text = (
                "--" if self._last_base_fix is None else "FIXED" if self._last_base_fix else "NOT FIXED"
            )
            self._ntrip_fixed_after_connect_s = None
        elif not ntrip_connected:
            self._ntrip_connect_monotonic_s = None
            self._ntrip_fixed_after_connect_s = None
        if (
            ntrip_connected
            and base_fix
            and self._ntrip_connect_monotonic_s is not None
            and self._ntrip_fixed_after_connect_s is None
        ):
            self._ntrip_fixed_after_connect_s = now_monotonic_s - self._ntrip_connect_monotonic_s
        self._ntrip_was_connected = ntrip_connected
        self._last_state_text = state
        self._last_base_fix = base_fix
        if self._ntrip_connect_monotonic_s is None:
            ntrip_delta_text = f"NTRIP 미연결 · 최근 FIX={self._ntrip_pre_connect_fix_text}"
            ntrip_delta_valid = None
        elif self._ntrip_fixed_after_connect_s is not None:
            ntrip_delta_text = (
                f"연결 전={self._ntrip_pre_connect_fix_text} → 연결 후 {self._ntrip_fixed_after_connect_s:.0f}s만에 FIXED"
            )
            ntrip_delta_valid = True
        else:
            elapsed_since_connect = now_monotonic_s - self._ntrip_connect_monotonic_s
            ntrip_delta_text = (
                f"연결 전={self._ntrip_pre_connect_fix_text} → 연결 후 {elapsed_since_connect:.0f}s 경과, FIX 대기중"
            )
            ntrip_delta_valid = None
        self.fixed_values["ntrip_delta"].set_value(ntrip_delta_text, valid=ntrip_delta_valid)

        route_rows: list[tuple[object, ...]] = []
        routes = data.get("routes")
        routes = routes if isinstance(routes, dict) else {}
        for side, label in (("left", "P1"), ("right", "P2")):
            route = self._dict(routes.get(side))
            route_rows.append(
                (
                    label,
                    route.get("frames", 0),
                    route.get("packets", 0),
                    route.get("bytes", 0),
                    route.get("fail", 0),
                    _display_number(route.get("delivery_age_s"), digits=2, suffix="s"),
                ),
            )
        _sync_table_rows(self.route_model, tuple(route_rows))

        readiness_limits = self._dict(data.get("readiness_limits"))
        rover = self._dict(data.get("rover"))
        left_rover = self._dict(rover.get("left"))
        right_rover = self._dict(rover.get("right"))
        coordinate_confirmed = bool(profile)
        try:
            correction_age_limit = max(
                0.0,
                float(readiness_limits.get("maximum_correction_age_s", 3.0)),
            )
        except (TypeError, ValueError):
            correction_age_limit = 3.0
        try:
            base_rtcm_active = (
                int(data.get("base_frames") or 0) > 0
                and frame_age not in (None, "")
                and float(frame_age) <= correction_age_limit
            )
        except (TypeError, ValueError):
            base_rtcm_active = False
        left_fixed = left_rover.get("rtk_fixed") is True
        right_fixed = right_rover.get("rtk_fixed") is True
        try:
            fixed_duration_target = max(
                0.0,
                float(readiness_limits.get("minimum_fixed_duration_s", 30.0)),
            )
        except (TypeError, ValueError):
            fixed_duration_target = 30.0
        try:
            fixed_stabilized = (
                left_fixed
                and right_fixed
                and left_rover.get("fixed_age_s") not in (None, "")
                and right_rover.get("fixed_age_s") not in (None, "")
                and float(left_rover.get("fixed_age_s")) >= fixed_duration_target
                and float(right_rover.get("fixed_age_s")) >= fixed_duration_target
            )
        except (TypeError, ValueError):
            fixed_stabilized = False
        checklist_statuses = {
            "base_coordinate": (coordinate_confirmed, "validated Fixed Base profile"),
            "base_rtcm": (base_rtcm_active, "fresh Base RTCM frame"),
            "left_fixed": (
                None if left_rover.get("fix_type") is None else left_fixed,
                "Left GPS fix_type=6",
            ),
            "right_fixed": (
                None if right_rover.get("fix_type") is None else right_fixed,
                "Right GPS fix_type=6",
            ),
            "stabilized": (
                None
                if left_rover.get("fix_type") is None or right_rover.get("fix_type") is None
                else fixed_stabilized,
                f"both fixed for ≥{fixed_duration_target:.0f}s",
            ),
        }
        for key, (satisfied, source_text) in checklist_statuses.items():
            self.checklist_values[key].set_value(
                "SATISFIED" if satisfied is True else "WAITING",
                source=source_text,
                valid=satisfied if satisfied is not None else None,
            )

        local_mode = mode == "LOCAL_SURVEY_IN"
        assisted_mode = mode == "NETWORK_ASSISTED_FIXED_BASE"
        network_mode = assisted_mode or mode == "NETWORK_RTK_DIRECT"
        self.serial_group.setVisible(local_mode or assisted_mode)
        self.candidate_group.setVisible(local_mode or assisted_mode)
        self.survey_group.setVisible(local_mode)
        self.ntrip_group.setVisible(network_mode)
        self.fixed_group.setVisible(assisted_mode)
        self.checklist_group.setVisible(assisted_mode)
        for command_id in ("gnss.base_discover", "gnss.base_connect", "gnss.base_disconnect"):
            self.operation_buttons[command_id].setVisible(local_mode or assisted_mode)
        for command_id in ("gnss.survey_start", "gnss.survey_stop"):
            self.operation_buttons[command_id].setVisible(local_mode)
        for command_id in ("gnss.ntrip_start", "gnss.ntrip_stop"):
            self.operation_buttons[command_id].setVisible(network_mode)
        self.operation_buttons["gnss.base_connect"].setEnabled((local_mode or assisted_mode) and not serial_connected)
        self.operation_buttons["gnss.base_discover"].setEnabled(
            (local_mode or assisted_mode) and not bool(data.get("base_discovery_active", False))
        )
        self.operation_buttons["gnss.base_disconnect"].setEnabled((local_mode or assisted_mode) and serial_connected)
        self.operation_buttons["gnss.survey_start"].setEnabled(local_mode and serial_connected and not survey_active)
        self.operation_buttons["gnss.survey_stop"].setEnabled(local_mode and serial_connected and survey_active)
        self.operation_buttons["gnss.ntrip_start"].setEnabled(
            network_mode and not ntrip_running and (not assisted_mode or serial_connected)
        )
        self.operation_buttons["gnss.ntrip_stop"].setEnabled(network_mode and ntrip_running)
        self._refresh_candidate_action()

    def _selected_candidate_row(self) -> int:
        selected = self.candidate_table.selectionModel().selectedRows()
        return selected[0].row() if selected else -1

    def _selected_candidate_port(self) -> str:
        row = self._selected_candidate_row()
        item = self.candidate_model.item(row, 0) if row >= 0 else None
        return item.text() if item is not None else ""

    def _refresh_candidate_action(self, *_args: object) -> None:
        self.apply_candidate_button.setEnabled(self._selected_candidate_row() >= 0)

    def _apply_selected_candidate(self) -> None:
        row = self._selected_candidate_row()
        method = getattr(self.backend, "gnss_base_apply_candidate", None)
        if row < 0 or not callable(method):
            self.candidate_feedback.setText("ERROR: select a Base receiver")
            return
        try:
            applied = method(row)
        except Exception as exc:
            self.candidate_feedback.setText(f"ERROR: {exc}")
            return
        if applied is False:
            self.candidate_feedback.setText("ERROR: Base receiver was not applied")
            return
        self.candidate_feedback.setText(f"Applied Base receiver row {row + 1}")
        self.refresh({})

    def _inspect_candidate_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        self.inspection_requested.emit(
            _live_inspection(
                self._candidate_inspection(row),
                lambda row=row: self._candidate_inspection(row),
            )
        )

    def _candidate_inspection(self, row: int) -> dict[str, object]:
        candidates = _evidence(self.backend).gnss_operation().get("base_candidates")
        candidates = candidates if isinstance(candidates, (tuple, list)) else ()
        candidate = self._dict(candidates[row]) if 0 <= row < len(candidates) else {}
        return {
            "selection": f"Base receiver candidate · {candidate.get('port', '--')}",
            "value": (
                f"target={candidate.get('target', '--')} · baud={candidate.get('baud', '--')} · "
                f"messages={candidate.get('messages', '--')}"
            ),
            "freshness": "Latest completed Base serial discovery",
            "contract": "Serial endpoint discovery metadata; contains no coordinates or rover baseline",
            "verdict": candidate.get("status") or ("UBX DETECTED" if candidate.get("ubx") else "UNVERIFIED"),
            "source": "GnssBaseSerialDetectWorker",
        }

    def _inspect_value(self, group_name: str, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._value_inspection(group_name, key),
                lambda group_name=group_name, key=key: self._value_inspection(group_name, key),
            )
        )

    def _value_inspection(self, group_name: str, key: str) -> dict[str, object]:
        data = _evidence(self.backend).gnss_operation()
        widget_groups = {
            "summary": self.summary_values,
            "serial": self.serial_values,
            "survey": self.survey_values,
            "ntrip": self.ntrip_values,
            "fixed": self.fixed_values,
            "checklist": self.checklist_values,
        }
        widget = widget_groups[group_name][key]
        displayed = f"{widget.value.text()} {widget.unit.text()}".strip()
        freshness = "current operation configuration"
        contract = "Base GNSS operation state; no rover baseline or Pixhawk attitude"
        verdict = widget.meta.text() or "Informational"
        source = "GnssOperationManager"
        if group_name == "survey":
            survey = self._dict(data.get("survey_in"))
            freshness = f"duration={survey.get('duration_s', 0)}s · observations={survey.get('observations', '--')}"
            contract = (
                "UBX NAV-SVIN mean ECEF [X,Y,Z] metres"
                if key == "ecef"
                else "Survey-in duration seconds and mean accuracy metres"
            )
            verdict = "VALID" if survey.get("valid") else "ACTIVE" if survey.get("active") else "NOT RUNNING"
            source = "UBX NAV-SVIN"
        elif group_name == "ntrip":
            ntrip = self._dict(data.get("ntrip"))
            freshness = str(ntrip.get("status", "No NTRIP status") or "No NTRIP status")
            contract = "NTRIP connection metadata and RTCM transport; no coordinate-frame conversion"
            verdict = "CONNECTED" if ntrip.get("connected") else "DISCONNECTED"
            source = "NTRIP worker + GnssOperationConfig"
        elif group_name == "fixed":
            coordinate = self._dict(data.get("base_coordinate"))
            freshness = f"age={_display_number(coordinate.get('age_s'), digits=2, suffix='s')}"
            contract = "WGS84 latitude/longitude degrees; height metres with explicit height type"
            verdict = "FIXED" if coordinate.get("fixed") else "NOT FIXED" if coordinate.get("fixed") is not None else "NO DATA"
            source = "UBX NAV-PVT / validated Fixed Base profile"
        elif group_name == "checklist":
            freshness = "current GNSS operation evidence"
            contract = "Informational checklist only; it does not block measurement"
            verdict = "SATISFIED" if widget.value.text() == "SATISFIED" else "WAITING"
            source = "Base-assisted GNSS checklist projection"
        elif group_name == "serial":
            serial = self._dict(data.get("base_serial"))
            verdict = "CONNECTED" if serial.get("connected") else "DISCONNECTED"
            source = "Base serial worker"
        elif key == "rtcm":
            freshness = f"last frame age={_display_number(data.get('last_frame_age_s'), digits=2, suffix='s')}"
            contract = "RTCM frame/byte counters are transport evidence, not position coordinates"
            verdict = "FRESH" if data.get("last_frame_age_s") is not None else "NO FRAME"
            source = "RTCM parser counters"
        elif key == "state":
            verdict = str(data.get("invalidation_reason") or data.get("state", "UNKNOWN"))
        return {
            "selection": f"{group_name.title()} · {widget.label.text()}",
            "value": displayed,
            "freshness": freshness,
            "contract": contract,
            "verdict": verdict,
            "source": source,
        }

    def _inspect_route_row(self, index: QtCore.QModelIndex) -> None:
        side = "left" if index.row() == 0 else "right"
        self.inspection_requested.emit(
            _live_inspection(
                self._route_inspection(side),
                lambda side=side: self._route_inspection(side),
            )
        )

    def _route_inspection(self, side: str) -> dict[str, object]:
        label = "P1" if side == "left" else "P2"
        data = _evidence(self.backend).gnss_operation()
        routes = data.get("routes") if isinstance(data.get("routes"), dict) else {}
        route = self._dict(routes.get(side))
        failures = int(route.get("fail") or 0)
        return {
            "selection": f"RTCM route · {label}",
            "value": (
                f"frames={route.get('frames', 0)} · packets={route.get('packets', 0)} · "
                f"bytes={route.get('bytes', 0)} · failures={failures}"
            ),
            "freshness": f"last delivery age={_display_number(route.get('delivery_age_s'), digits=2, suffix='s')}",
            "contract": "RTCM base stream → MAVLink GPS_RTCM_DATA; transport counters only",
            "verdict": "DELIVERY FAILURE" if failures else "DELIVERING" if route.get("packets") else "NO DELIVERY",
            "source": "RTCM injection route counters",
        }


class _FrozenPreviewColumn(QtWidgets.QWidget):
    """One full-width camera preview whose image child stays at 16:9."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Preferred,
        )

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt override.
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt override.
        content_width = max(
            0,
            int(width)
            - self.layout().contentsMargins().left()
            - self.layout().contentsMargins().right(),
        ) if self.layout() is not None else max(0, int(width))
        view = self.findChild(AnnotationImageView)
        image_height = view.heightForWidth(content_width) if view is not None else int(round(content_width * 9 / 16))
        label = self.findChild(QtWidgets.QLabel)
        label_height = label.sizeHint().height() if label is not None else 0
        spacing = self.layout().spacing() if self.layout() is not None else 0
        margins = self.layout().contentsMargins() if self.layout() is not None else QtCore.QMargins()
        return image_height + label_height + spacing + margins.top() + margins.bottom()

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt override.
        hint = super().sizeHint()
        if hint.width() > 0:
            hint.setHeight(self.heightForWidth(hint.width()))
        return hint

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt API.
        desired_height = self.heightForWidth(self.width())
        if self.minimumHeight() != desired_height:
            self.setMinimumHeight(desired_height)
        super().resizeEvent(event)


class _CurrentPageTabWidget(QtWidgets.QTabWidget):
    """Size to the current tab's page instead of the tallest page.

    QTabWidget's internal QStackedWidget normally reports the size hint of
    its largest page for every tab, so switching to a short page (e.g. a
    single note + button) leaves most of that page blank. Following only
    the current page's hint trades that wasted space for a height that
    shifts when the active tab changes.
    """

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.currentChanged.connect(lambda _index: self.updateGeometry())

    def _current_page_size(self) -> QtCore.QSize:
        current = self.currentWidget()
        page_size = current.sizeHint() if current is not None else QtCore.QSize(0, 0)
        tab_bar = self.tabBar()
        tab_bar_height = tab_bar.sizeHint().height() if tab_bar is not None else 0
        frame_width = self.style().pixelMetric(QtWidgets.QStyle.PM_DefaultFrameWidth, None, self)
        return QtCore.QSize(
            page_size.width() + 2 * frame_width,
            page_size.height() + tab_bar_height + 2 * frame_width,
        )

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt API.
        return self._current_page_size()

    def minimumSizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt API.
        return self._current_page_size()


class _FrozenPreviewGroup(QtWidgets.QGroupBox):
    """Prevent the Alignment page from vertically squeezing the 16:9 pair."""

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt API.
        desired_height = self.heightForWidth(self.width()) if self.hasHeightForWidth() else self.sizeHint().height()
        if desired_height > 0 and self.minimumHeight() != desired_height:
            self.setMinimumHeight(desired_height)
        super().resizeEvent(event)


class OpticalPosePane(QtWidgets.QWidget):
    """Presentation-only controls for optical pose initialization.

    The pane deliberately owns no pose solver, file I/O, or calibration
    state.  It renders immutable owner snapshots and emits typed command
    requests that a controller may handle outside the UI layer.
    """

    command_requested = QtCore.pyqtSignal(str, object)

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
    DOF_KEYS = ("roll", "pitch", "yaw", "north", "east", "down")
    DRONE_RTK_OBSERVATION_HEADERS = (
        "#",
        "State",
        "Drone RTK",
        "P1 RTK",
        "P2 RTK",
        "Sync L",
        "Sync R",
        "Target",
    )

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._active_profile_id = ""
        self._candidate_rows: dict[str, dict[str, object]] = {}
        self._yaw_checked_before_horizon = True
        self._dof_default_candidate_id = ""
        self._auto_suggestions: dict[str, dict[str, object]] = {}
        self._active_auto_suggestion_id = ""
        self._accepted_annotation_snapshots: dict[str, dict[str, object]] = {}
        self._auto_accept_in_progress = False
        self._auto_suggestion_provenance = "NONE"
        # True while the status/metrics labels show an Auto-select failure
        # reason instead of "No automatic suggestion". Periodic snapshot
        # polling still calls set_auto_suggestions([]) every tick even while
        # this is showing (the failed attempt was never recorded as a
        # suggestion, so every poll sees the same empty list); this flag
        # keeps the reason on screen instead of it being wiped within a
        # second by the next poll.
        self._auto_suggestion_error_active = False

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        readiness_group = QtWidgets.QGroupBox("Readiness")
        readiness_layout = QtWidgets.QGridLayout(readiness_group)
        readiness_layout.setContentsMargins(6, 6, 6, 6)
        self.readiness_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("state", "Pose state"),
                ("operating_zoom", "Operating zoom L/R"),
                ("intrinsics", "Exact K,D"),
                ("base_extrinsics", "Base R,t"),
                ("mountain_uv", "Mountain u,v"),
                ("terrain", "Terrain GeoTIFF"),
                ("target_reference", "Target reference"),
            )
        ):
            value = EngineeringValue(label)
            value.set_value("--", source="optical pose owner snapshot")
            self.readiness_values[key] = value
            readiness_layout.addWidget(value, index // 2, index % 2)
        layout.addWidget(readiness_group)

        self.solver_buttons: dict[str, QtWidgets.QPushButton] = {}

        def make_solver_button(command_id: str, label: str) -> QtWidgets.QPushButton:
            button = QtWidgets.QPushButton(label)
            button.setProperty("commandId", command_id)
            button.clicked.connect(
                lambda _checked=False, command_id=command_id: self.command_requested.emit(
                    command_id,
                    self.command_payload(command_id),
                )
            )
            self.solver_buttons[command_id] = button
            return button

        preview_group = _FrozenPreviewGroup("Frozen native reference burst + annotations")
        preview_group_layout = QtWidgets.QVBoxLayout(preview_group)
        preview_group_layout.setContentsMargins(6, 6, 6, 6)
        preview_group_layout.setSpacing(5)
        annotation_actions = QtWidgets.QHBoxLayout()
        annotation_actions.addWidget(QtWidgets.QLabel("Annotation"))
        self.annotation_mode_combo = QtWidgets.QComboBox()
        self.annotation_mode_combo.setObjectName("opticalPoseAnnotationMode")
        self.annotation_mode_combo.addItem("Horizon (2 points)", AnnotationMode.HORIZON.value)
        self.annotation_mode_combo.addItem("Mountain ridge (polyline)", AnnotationMode.RIDGE.value)
        self.annotation_mode_combo.addItem("Drone target (1 point)", AnnotationMode.TARGET.value)
        self.annotation_mode_combo.currentIndexChanged.connect(self._set_annotation_mode)
        annotation_actions.addWidget(self.annotation_mode_combo)
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
                "Drone RTK Yaw solver uses it with line-of-sight evidence; multi-view "
                "refinement uses imported observations separately."
            ),
        }
        self.annotation_clear_button = QtWidgets.QPushButton("Clear")
        self.annotation_clear_button.setObjectName("opticalPoseAnnotationClear")
        self.annotation_clear_button.setToolTip(
            "Clear the selected horizon/ridge/target points from both frozen frames."
        )
        self.annotation_clear_button.clicked.connect(self._clear_current_annotations)
        annotation_actions.addWidget(self.annotation_clear_button)
        self.capture_frozen_burst_button = make_solver_button(
            "alignment.pose.capture_burst",
            "Capture",
        )
        self.capture_frozen_burst_button.setToolTip(
            "Capture once, then use the same immutable native left/right frames across all R,t methods."
        )
        annotation_actions.addWidget(self.capture_frozen_burst_button)
        self.capture_frozen_burst_wiris_ftp_button = make_solver_button(
            "alignment.pose.capture_burst_wiris_ftp",
            "Capture FTP HR",
        )
        self.capture_frozen_burst_wiris_ftp_button.setToolTip(
            "Trigger a WIRIS HR still on both cameras and download it via FTP instead of the "
            "live RTSP feed, then freeze it the same way as Capture."
        )
        annotation_actions.addWidget(self.capture_frozen_burst_wiris_ftp_button)
        self.capture_frozen_burst_wiris_ftp_oz_button = make_solver_button(
            "alignment.pose.capture_burst_wiris_ftp_oz",
            "Capture FTP OZ",
        )
        self.capture_frozen_burst_wiris_ftp_oz_button.setToolTip(
            "Same as Capture FTP HR, using the lower-resolution "
            "operating-zoom still (SIVI) instead of the HR still."
        )
        annotation_actions.addWidget(self.capture_frozen_burst_wiris_ftp_oz_button)
        self.import_frozen_burst_button = QtWidgets.QPushButton("Import")
        self.import_frozen_burst_button.setObjectName("opticalPoseImportBurst")
        self.import_frozen_burst_button.setToolTip(
            "Load left/right images from disk instead of a live capture -- reuse an "
            "FTP-downloaded HR/OZ still, or feed local test imagery for offline accuracy evaluation. "
            "The same immutable-frame contract as Capture applies."
        )
        self.import_frozen_burst_button.clicked.connect(self._import_frozen_burst)
        annotation_actions.addWidget(self.import_frozen_burst_button)
        annotation_actions.addStretch(1)
        preview_group_layout.addLayout(annotation_actions)

        # Alignment annotations need accurate, readable pixels.  Keeping the
        # two frozen frames side by side halves each viewport on a normal
        # landscape workstation, which in turn halves the useful image size.
        # Stack them vertically so each camera receives the full content width;
        # the optical-pose page is already scrollable, so the extra height is
        # intentional and each 16:9 viewport can remain uncropped by layout.
        preview_layout = QtWidgets.QVBoxLayout()
        preview_layout.setContentsMargins(0, 0, 0, 0)
        preview_layout.setSpacing(6)
        self.frozen_preview_layout = preview_layout
        self.frozen_preview_labels: dict[str, QtWidgets.QLabel] = {}
        self.annotation_views: dict[str, AnnotationImageView] = {}
        for side, label in (("left", "Left"), ("right", "Right")):
            side_panel = _FrozenPreviewColumn()
            side_layout = QtWidgets.QVBoxLayout(side_panel)
            side_layout.setContentsMargins(0, 0, 0, 0)
            side_layout.setSpacing(3)
            view = AnnotationImageView(side_panel)
            view.setObjectName(f"opticalPoseAnnotation{side.title()}")
            # Frozen references use the same 16:9 camera viewport as the live
            # preview.  AnnotationImageView keeps this ratio through
            # heightForWidth and center-crops non-16:9 native evidence.  Each
            # stacked camera panel receives the full available content width,
            # so the annotation viewport does not lose half its pixels to a
            # side-by-side split.
            view.setMinimumHeight(view.heightForWidth(view.minimumWidth()))
            view.setToolTip(
                "Frozen native pixels only. Coordinates are OpenCV image pixels "
                "(u right, v down); the display is fixed 16:9 and center-crops "
                "only for presentation; saved points never change. Hover shows "
                "the exact native pixel under the pointer. Scroll over the image "
                "to zoom around the cursor; native coordinates remain unchanged."
            )
            self.annotation_views[side] = view
            view.annotation_changed.connect(
                lambda payload, side=side: self._on_annotation_changed(side, payload)
            )
            side_layout.addWidget(view, 0)
            preview = QtWidgets.QLabel(f"{label}: NO FROZEN BURST")
            preview.setObjectName(f"opticalPoseFrozen{side.title()}")
            preview.setAlignment(QtCore.Qt.AlignCenter)
            preview.setFrameShape(QtWidgets.QFrame.StyledPanel)
            preview.setWordWrap(True)
            self.frozen_preview_labels[side] = preview
            side_layout.addWidget(preview)
            preview_layout.addWidget(side_panel, 1)
        preview_group_layout.addLayout(preview_layout, 1)
        layout.addWidget(preview_group)

        observations_group = QtWidgets.QGroupBox("Drone RTK Capture Observations")
        observations_group.setObjectName("opticalPoseDroneRtkObservationsGroup")
        observations_layout = QtWidgets.QVBoxLayout(observations_group)
        observations_layout.setContentsMargins(6, 6, 6, 6)
        observations_layout.setSpacing(5)

        current_observation_header = QtWidgets.QLabel("Current Observation")
        current_observation_header.setObjectName("compactContract")
        observations_layout.addWidget(current_observation_header)
        self.current_observation_summary_label = QtWidgets.QLabel("No observation yet")
        self.current_observation_summary_label.setObjectName("statusStrip")
        observations_layout.addWidget(self.current_observation_summary_label)
        current_observation_grid = QtWidgets.QGridLayout()
        current_observation_grid.setSpacing(5)
        self.current_observation_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("drone_rtk", "Drone RTK"),
                ("gnss_age", "GNSS age"),
                ("sync", "Sync"),
                ("target", "Target"),
            )
        ):
            value = EngineeringValue(label)
            value.set_value("--")
            self.current_observation_values[key] = value
            current_observation_grid.addWidget(value, index // 2, index % 2)
        observations_layout.addLayout(current_observation_grid)

        history_header = QtWidgets.QLabel("Observation History")
        history_header.setObjectName("compactContract")
        observations_layout.addWidget(history_header)
        self.observation_history_table = QtWidgets.QTableWidget(
            0, len(self.DRONE_RTK_OBSERVATION_HEADERS)
        )
        self.observation_history_table.setObjectName("opticalPoseDroneRtkObservationTable")
        self.observation_history_table.setHorizontalHeaderLabels(self.DRONE_RTK_OBSERVATION_HEADERS)
        self.observation_history_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.observation_history_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.observation_history_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.observation_history_table.verticalHeader().setVisible(False)
        self.observation_history_table.setMaximumHeight(140)
        self.observation_history_table.horizontalHeader().setStretchLastSection(True)
        self.observation_history_table.itemSelectionChanged.connect(
            self._refresh_drone_rtk_observation_detail
        )
        observations_layout.addWidget(self.observation_history_table)

        details_header = QtWidgets.QLabel("Selected Observation Details")
        details_header.setObjectName("compactContract")
        observations_layout.addWidget(details_header)
        details_grid = QtWidgets.QGridLayout()
        details_grid.setSpacing(5)
        self.observation_detail_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("observation_id", "Observation ID"),
                ("observation_state", "State"),
                ("observation_burst_id", "Burst ID"),
                ("drone_latitude", "Drone latitude"),
                ("drone_longitude", "Drone longitude"),
                ("drone_altitude", "Drone altitude"),
                ("drone_ellipsoid_height", "Drone ellipsoid height"),
                ("drone_fix", "Drone fix"),
                ("drone_satellites", "Drone satellites"),
                ("drone_eph_epv", "Drone eph / epv"),
                ("drone_timestamp", "Drone GNSS timestamp"),
                ("p1_latitude", "P1 latitude"),
                ("p1_longitude", "P1 longitude"),
                ("p1_altitude", "P1 altitude"),
                ("p1_ellipsoid_height", "P1 ellipsoid height"),
                ("p1_fix", "P1 fix"),
                ("p1_satellites", "P1 satellites"),
                ("p1_eph_epv", "P1 eph / epv"),
                ("p1_timestamp", "P1 GNSS timestamp"),
                ("p2_latitude", "P2 latitude"),
                ("p2_longitude", "P2 longitude"),
                ("p2_altitude", "P2 altitude"),
                ("p2_ellipsoid_height", "P2 ellipsoid height"),
                ("p2_fix", "P2 fix"),
                ("p2_satellites", "P2 satellites"),
                ("p2_eph_epv", "P2 eph / epv"),
                ("p2_timestamp", "P2 GNSS timestamp"),
                ("left_frame_id", "Left frame ID"),
                ("left_frame_sha", "Left frame SHA prefix"),
                ("left_frame_timestamp", "Left frame timestamp"),
                ("left_frame_timestamp_reference", "Left timestamp reference"),
                ("right_frame_id", "Right frame ID"),
                ("right_frame_sha", "Right frame SHA prefix"),
                ("right_frame_timestamp", "Right frame timestamp"),
                ("right_frame_timestamp_reference", "Right timestamp reference"),
                ("sync_state", "Sync state"),
                ("sync_delta_left", "Sync delta L"),
                ("sync_delta_right", "Sync delta R"),
                ("sync_reason", "Sync reason"),
            )
        ):
            value = EngineeringValue(label)
            value.set_value("--")
            self.observation_detail_values[key] = value
            details_grid.addWidget(value, index // 2, index % 2)
        observations_layout.addLayout(details_grid)

        self._drone_rtk_observation_rows: dict[str, dict[str, object]] = {}
        self._drone_rtk_observation_order: list[str] = []
        layout.addWidget(observations_group)

        auto_group = QtWidgets.QGroupBox("Automatic point suggestion · review required")
        auto_group.setObjectName("opticalPoseAutoSuggestionGroup")
        auto_layout = QtWidgets.QGridLayout(auto_group)
        auto_layout.setContentsMargins(6, 6, 6, 6)
        auto_layout.setSpacing(5)
        self.auto_select_buttons: dict[str, QtWidgets.QPushButton] = {}
        for column, (command_id, label) in enumerate(
            (
                ("alignment.pose.auto_select_horizon", "Auto-select Horizon"),
                ("alignment.pose.auto_select_mountain", "Auto-select Mountain"),
            )
        ):
            button = make_solver_button(command_id, label)
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
            "Approve the currently displayed high-confidence suggestion and copy its points into "
            "the existing manual annotation buffer. Low-confidence suggestions remain overlay-only."
        )
        auto_layout.addWidget(self.use_suggestion_button, 0, 2)
        self.auto_suggestion_status_label = QtWidgets.QLabel("No automatic suggestion")
        self.auto_suggestion_status_label.setObjectName("statusStrip")
        self.auto_suggestion_status_label.setWordWrap(True)
        auto_layout.addWidget(self.auto_suggestion_status_label, 1, 0, 1, 3)
        self.auto_suggestion_metrics_label = QtWidgets.QLabel(
            "Confidence / inliers / residual / detector: --"
        )
        self.auto_suggestion_metrics_label.setObjectName("compactContract")
        self.auto_suggestion_metrics_label.setWordWrap(True)
        auto_layout.addWidget(self.auto_suggestion_metrics_label, 2, 0, 1, 3)
        layout.addWidget(auto_group)

        workflow_group = QtWidgets.QGroupBox("Pose workflow")
        workflow_group.setObjectName("opticalPoseWorkflowGroup")
        workflow_layout = QtWidgets.QGridLayout(workflow_group)
        workflow_layout.setContentsMargins(6, 6, 6, 6)
        workflow_layout.setSpacing(5)
        initialize_button = make_solver_button(
            "alignment.pose.initialize_telemetry",
            "Initialize from Telemetry",
        )
        initialize_button.setToolTip(
            "Build a reviewed initial pose candidate from fresh P1/P2/Drone GNSS and, when available, "
            "fresh Pixhawk R/P/Y. GNSS positions are converted to local world NED; this does not auto-apply."
        )
        multiview_button = make_solver_button(
            "alignment.pose.solve_drone_rtk_multiview",
            "Refine Drone RTK Multi-view R,t",
        )
        multiview_button.setToolTip(
            "Import a multi-view JSON containing per-side camera rays/pixels and synchronized Drone RTK "
            "target positions, then solve full 6-DOF R,t for both cameras."
        )
        workflow_layout.addWidget(initialize_button, 0, 0)
        workflow_layout.addWidget(multiview_button, 0, 1)
        workflow_note = QtWidgets.QLabel(
            "Two explicit stages: initialize with live Telemetry, then refine with several synchronized "
            "Drone RTK target observations. Both results become candidates and share the same review/apply path."
        )
        workflow_note.setObjectName("compactContract")
        workflow_note.setWordWrap(True)
        workflow_layout.addWidget(workflow_note, 1, 0, 1, 2)
        layout.addWidget(workflow_group)

        method_group = QtWidgets.QGroupBox("R,t measurement methods")
        method_layout = QtWidgets.QVBoxLayout(method_group)
        self.method_tabs = _CurrentPageTabWidget()
        self.method_tabs.setObjectName("opticalPoseMethodTabs")

        def add_method_tab(
            tab_label: str,
            description: str,
            buttons: Sequence[QtWidgets.QWidget],
            content: Callable[[QtWidgets.QVBoxLayout], None] | None = None,
        ) -> None:
            page = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(6, 6, 6, 6)
            note = QtWidgets.QLabel(description)
            note.setObjectName("compactContract")
            note.setWordWrap(True)
            page_layout.addWidget(note)
            if content is not None:
                content(page_layout)
            action_layout = QtWidgets.QHBoxLayout()
            for button in buttons:
                action_layout.addWidget(button)
            action_layout.addStretch(1)
            page_layout.addLayout(action_layout)
            page_layout.addStretch(1)
            self.method_tabs.addTab(page, tab_label)

        add_method_tab(
            "Horizon R/P",
            "Two horizon points constrain Roll/Pitch only. Yaw and camera position remain at the prior pose.",
            (
                make_solver_button("alignment.pose.solve_horizon", "Solve Horizon Roll/Pitch"),
            ),
        )
        add_method_tab(
            "Mountain + GNSS",
            "Click corresponding Mountain features in the same order on both frames. Uses paired u,v and the numeric GNSS baseline; no GeoTIFF or terrain height.",
            (
                make_solver_button("alignment.pose.solve_gnss_mountain", "Solve GNSS + Mountain R,t"),
            ),
        )
        def build_terrain_content(page_layout: QtWidgets.QVBoxLayout) -> None:
            terrain_note = QtWidgets.QLabel(
                "Uses the same Ridge points as Mountain + GNSS -- switch to Ridge mode on that "
                "tab to click them. \"Solve Terrain PnP\" will prompt for a correspondences JSON "
                "matching each side's clicked points 1:1 with object_points_ned_m; see the "
                "Readiness card above for GeoTIFF/vertical-datum status."
            )
            terrain_note.setObjectName("compactContract")
            terrain_note.setWordWrap(True)
            page_layout.addWidget(terrain_note)

        add_method_tab(
            "Terrain PnP",
            "Full 6-DOF pose from 2D/3D correspondences against an imported GeoTIFF -- the only "
            "method that does not inherit any axis from the prior pose when it succeeds.",
            (
                make_solver_button("alignment.pose.import_terrain", "Import GeoTIFF"),
                make_solver_button("alignment.pose.solve_terrain", "Solve Terrain PnP"),
            ),
            content=build_terrain_content,
        )
        add_method_tab(
            "Drone RTK Yaw",
            "Advanced single-target fallback: a frozen target pixel plus explicit Drone RTK line-of-sight "
            "evidence constrains Yaw. Use Pose workflow above for Telemetry initialization and full multi-view R,t.",
            (
                make_solver_button("alignment.pose.solve_drone_yaw", "Solve Drone RTK Yaw"),
            ),
        )
        def register_reference_candidate(source: str) -> None:
            self.command_requested.emit(
                "alignment.pose.register_reference",
                {"source": source},
            )

        register_checkerboard_button = QtWidgets.QPushButton("Register Checkerboard as candidate")
        register_checkerboard_button.setObjectName("opticalPoseRegisterCheckerboardReference")
        register_checkerboard_button.setToolTip(
            "Package the stored Checkerboard stereoCalibrate R,t (all 6 DOF) as an immutable "
            "candidate, anchored to the current prior's left pose, so Combine candidates can mix "
            "specific axes from it -- e.g. take only its translation alongside a rotation-only "
            "Horizon/Drone-yaw solve. This does not recalculate or overwrite the stored profile."
        )
        register_checkerboard_button.clicked.connect(
            lambda _checked=False: register_reference_candidate("checkerboard")
        )
        register_pixhawk_button = QtWidgets.QPushButton("Register Pixhawk + RTK (live) as candidate")
        register_pixhawk_button.setObjectName("opticalPoseRegisterPixhawkReference")
        register_pixhawk_button.setToolTip(
            "Package the current live RTK GNSS baseline as a translation-only candidate (rotation "
            "is not packaged here -- the attitude quaternion is body-FRD, not camera-RDF) so "
            "Combine candidates can mix its N/E/D with a separately solved rotation."
        )
        register_pixhawk_button.clicked.connect(
            lambda _checked=False: register_reference_candidate("pixhawk_rtk")
        )
        add_method_tab(
            "Existing calibration",
            "Reference only. Calibration creates the checkerboard stereo R,t profile; "
            "Alignment does not recalculate or overwrite it here. Apply the selected K,D + R,t "
            "bundle in Calibration first, then use this reference as the base pose for "
            "optical-pose candidates. The active profile and baseline are shown below in the "
            "common comparison card.",
            (register_checkerboard_button, register_pixhawk_button),
        )

        combine_button = make_solver_button(
            "alignment.pose.solve_combined",
            "Build combined candidate",
        )
        combine_button.setEnabled(False)
        combine_button.setToolTip(
            "Combine explicitly checked candidates only when they share the same prior and exact "
            "operating K,D profile and optimize non-overlapping DOF. This creates a new candidate; "
            "it does not apply the pose automatically."
        )

        def build_combine_content(page_layout: QtWidgets.QVBoxLayout) -> None:
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
            self.combine_dof_table.setHorizontalHeaderLabels(
                ["Candidate", "Roll", "Pitch", "Yaw", "N", "E", "D"]
            )
            self.combine_dof_table.verticalHeader().setVisible(False)
            self.combine_dof_table.horizontalHeader().setSectionResizeMode(
                0, QtWidgets.QHeaderView.Stretch
            )
            for column in range(1, 7):
                self.combine_dof_table.horizontalHeader().setSectionResizeMode(
                    column, QtWidgets.QHeaderView.ResizeToContents
                )
            self.combine_dof_table.setMinimumHeight(82)
            self.combine_dof_table.setMaximumHeight(160)
            self.combine_dof_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.combine_dof_table.itemChanged.connect(self._refresh_combine_action)
            page_layout.addWidget(self.combine_dof_table)

            self.combine_candidate_status = QtWidgets.QLabel(
                "No candidates checked · select at least two"
            )
            self.combine_candidate_status.setObjectName("statusStrip")
            self.combine_candidate_status.setWordWrap(True)
            page_layout.addWidget(self.combine_candidate_status)

        add_method_tab(
            "Combine candidates",
            "Compose independent evidence into a new immutable candidate. This is not averaging and "
            "does not silently choose the latest Horizon/Drone result; the source candidates must be "
            "checked explicitly below.",
            (combine_button,),
            content=build_combine_content,
        )
        method_layout.addWidget(self.method_tabs)
        layout.addWidget(method_group)

        self.solver_buttons["alignment.pose.solve_horizon"].setToolTip(
            "Automatically fit Roll/Pitch from the manually marked horizon line. "
            "The horizon cannot observe Yaw."
        )
        self.solver_buttons["alignment.pose.solve_gnss_mountain"].setToolTip(
            "Build metric R,t from at least four same-order paired Mountain u,v clicks "
            "on frozen left/right frames and the numeric left/right GNSS baseline. "
            "No GeoTIFF or terrain height is used. Fix quality is recorded as evidence, "
            "not used as a hard gate. Assumes camera optical center equals GNSS antenna."
        )

        dof_group = QtWidgets.QGroupBox("Candidate degrees of freedom")
        dof_layout = QtWidgets.QGridLayout(dof_group)
        self.dof_checkboxes: dict[str, QtWidgets.QCheckBox] = {}
        for index, key in enumerate(self.DOF_KEYS):
            label = key.title() if key not in {"north", "east", "down"} else f"{key.title()} position"
            checkbox = QtWidgets.QCheckBox(label)
            checkbox.setChecked(key in {"roll", "pitch", "yaw"})
            checkbox.setToolTip(
                "Rotation is PX4/NED Roll/Pitch/Yaw"
                if key in {"roll", "pitch", "yaw"}
                else "Optical-center displacement in world NED metres"
            )
            self.dof_checkboxes[key] = checkbox
            dof_layout.addWidget(checkbox, 0, index)
        # Horizon is the initial/manual mode and observes only Roll/Pitch.
        # Apply this after all checkboxes exist so the UI cannot briefly expose
        # an invalid Yaw selection during construction.
        self._set_annotation_mode(self.annotation_mode_combo.currentIndex())
        dof_contract = QtWidgets.QLabel(
            "Unchecked DOF remain at the base pose; the owner must re-solve the same observations "
            "with these constraints. The UI never splices Euler components into a rotation matrix."
        )
        dof_contract.setObjectName("compactContract")
        dof_contract.setWordWrap(True)
        dof_layout.addWidget(dof_contract, 1, 0, 1, len(self.DOF_KEYS))
        layout.addWidget(dof_group)

        candidate_group = QtWidgets.QGroupBox("R,t candidates · compare")
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
        self.apply_button = QtWidgets.QPushButton("Apply selected DOF")
        self.apply_button.setProperty("commandId", "alignment.pose.apply_candidate")
        self.apply_button.setEnabled(False)
        self.apply_button.clicked.connect(self._request_apply)
        self.revert_button = QtWidgets.QPushButton("Revert active pose")
        self.revert_button.setProperty("commandId", "alignment.pose.revert")
        self.revert_button.setEnabled(False)
        self.revert_button.clicked.connect(self._request_revert)
        self.candidate_status_label = QtWidgets.QLabel("No pose candidate")
        self.candidate_status_label.setObjectName("statusStrip")
        candidate_actions.addWidget(self.apply_button)
        candidate_actions.addWidget(self.revert_button)
        candidate_actions.addWidget(self.candidate_status_label, 1)
        candidate_layout.addLayout(candidate_actions)
        layout.addWidget(candidate_group)

        contract = QtWidgets.QLabel(
            "Frames: image pixels/OpenCV optical RDF · pose PX4 virtual FRD→world NED · "
            "GNSS WGS84 · candidate application is immutable and owner-controlled"
        )
        contract.setObjectName("compactContract")
        contract.setWordWrap(True)
        layout.addWidget(contract)

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
        raw = candidate.get("observable_dof", candidate.get("observability"))
        if isinstance(raw, Mapping):
            enabled = [str(key) for key, value in raw.items() if bool(value)]
            return ",".join(enabled) or "none"
        if isinstance(raw, (tuple, list, set)):
            return ",".join(str(value) for value in raw) or "none"
        return str(raw or "--")

    def dof_payload(self) -> dict[str, bool]:
        """Return an explicit, ordering-independent 6-DOF constraint mask."""

        payload = {
            key: bool(self.dof_checkboxes[key].isChecked())
            for key in self.DOF_KEYS
        }
        # Keep the owner contract fail-closed even if a test, snapshot, or
        # accessibility action programmatically checks a disabled checkbox.
        selected = self._candidate_rows.get(self.selected_candidate_id(), {})
        candidate_method = str(selected.get("method", "") or "")
        if (
            str(self.annotation_mode_combo.currentData() or "") == AnnotationMode.HORIZON.value
            and candidate_method != "gnss-constrained-mountain-uv"
        ):
            payload["yaw"] = False
        return payload

    def annotation_payloads(self) -> dict[str, object]:
        """Return frame-pinned immutable annotation objects for both sides.

        ``FrozenAnnotationPayload`` is a frozen dataclass.  It intentionally
        remains an object at the UI/controller boundary instead of being
        flattened into mutable JSON here; the evidence owner decides when to
        serialize its fresh ``to_mapping()`` copy.
        """

        return {
            side: view.export_annotations()
            for side, view in self.annotation_views.items()
        }

    def _import_frozen_burst(self) -> None:
        left_path, _left_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Import left frozen frame image",
            "",
            "Images (*.png *.jpg *.jpeg *.tif *.tiff);;All files (*)",
        )
        if not left_path:
            return
        right_path, _right_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Import right frozen frame image",
            "",
            "Images (*.png *.jpg *.jpeg *.tif *.tiff);;All files (*)",
        )
        if not right_path:
            return
        # Without a live camera to read GZVV from, the importer would
        # otherwise leave zoom_index unset and every solve fails with
        # zoom_readback_unknown -- ask once, up front, and let the operator
        # skip it only if they intend to rely on a live readback instead.
        zoom_index, confirmed = QtWidgets.QInputDialog.getInt(
            self,
            "Zoom level",
            "Zoom index these images were captured at (must match the loaded K,D "
            "profile's calibration zoom). Cancel to fall back to the live GZVV readback instead.",
            1,
            1,
            100,
            1,
        )
        sources: dict[str, dict[str, object]] = {
            "left": {"path": left_path},
            "right": {"path": right_path},
        }
        if confirmed:
            sources["left"]["zoom_index"] = zoom_index
            sources["right"]["zoom_index"] = zoom_index
        self.command_requested.emit(
            "alignment.pose.import_burst",
            {"sources": sources},
        )

    def command_payload(self, command_id: str) -> dict[str, object]:
        payload: dict[str, object] = {
            "command_id": str(command_id),
            "dof": self.dof_payload(),
            "annotation_mode": str(
                self.annotation_mode_combo.currentData()
                or AnnotationMode.HORIZON.value
            ),
            "annotations": self.annotation_payloads(),
            "annotation_contract": (
                "frozen native OpenCV pixels: u right, v down; identity is "
                "frame_id + image_sha256 + native dimensions"
            ),
            "mountain_contract": (
                "same-order paired left/right frozen native Mountain u,v; "
                "numeric GNSS baseline; no GeoTIFF or terrain height"
            ),
        }
        if command_id == "alignment.pose.solve_combined":
            payload["candidate_ids"] = list(self.combine_candidate_ids())
            payload["candidate_dofs"] = self.combine_candidate_dofs()
        if command_id in {
            "alignment.pose.auto_select_horizon",
            "alignment.pose.auto_select_mountain",
        }:
            payload["auto_mode"] = (
                "horizon"
                if command_id.endswith("horizon")
                else "mountain"
            )
            payload["frozen_frames_required"] = True
        if command_id == "alignment.pose.use_suggestion":
            payload["suggestion_id"] = self._active_auto_suggestion_id
            payload["provenance_state"] = self._auto_suggestion_provenance
        return payload

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
        return {
            "candidate_id": candidate_id,
            "dof": self.dof_payload(),
        }

    def revert_payload(self) -> dict[str, object]:
        return {
            "active_profile_id": self._active_profile_id,
            "restore": "previous_immutable_profile",
        }

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
            self._reset_auto_acceptance()
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
        # Drone RTK multi-view uses one target mark per captured frame. Keep
        # the previous target mark visible across a new frozen burst until
        # the operator explicitly presses Clear. The view
        # keeps its exported frame identity pinned to the old evidence, so a
        # normal single-frame solver cannot silently consume stale pixels.
        preserve_annotations = bool(view.annotation_points(AnnotationMode.TARGET))
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
            self._reset_auto_acceptance()
            if self._auto_suggestion_error_active:
                # The burst this failure applied to is gone; a fresh
                # Capture/Import burst makes the old reason stale.
                self._auto_suggestion_error_active = False
                self.auto_suggestion_status_label.setStyleSheet("")
                self.auto_suggestion_status_label.setText("No automatic suggestion")
                self.auto_suggestion_metrics_label.setText(
                    "Confidence / inliers / residual / detector: --"
                )
        digest = str(view.image_sha256 or "")
        retained_note = (
            " · previous target mark retained; Clear to edit"
            if view.annotations_stale
            else ""
        )
        label.setText(
            f"{side.title()}: {frame_id} · {image.width()}×{image.height()} · sha256={digest[:12]}…"
            f"{retained_note}"
        )

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

    def _reset_auto_acceptance(self) -> None:
        """Invalidate accepted-point provenance when the frozen frame changes."""

        self._accepted_annotation_snapshots = {}
        self._accepted_suggestion_id = ""
        self._auto_suggestion_provenance = "NONE"

    def set_auto_suggestion_error(self, command_id: str, message: str) -> None:
        """Show why Auto-select produced nothing at all.

        Auto-select can raise before a suggestion is ever recorded (no
        frozen burst yet, intrinsics not exact for this zoom, etc.), which
        otherwise leaves the status/metrics labels silently stuck on their
        placeholder text with only a status-bar toast as feedback. This
        keeps the reason visible next to the buttons until the next attempt.
        """

        self._auto_suggestion_error_active = True
        self.use_suggestion_button.setEnabled(False)
        self.auto_suggestion_status_label.setText(f"{command_id}: failed — {message}")
        self.auto_suggestion_status_label.setStyleSheet(f"color: {ERROR_TEXT};")
        self.auto_suggestion_metrics_label.setText(
            "Confidence / inliers / residual / detector: --"
        )

    def set_auto_suggestions(self, suggestions: object) -> None:
        """Render the latest owner proposal without touching manual points."""

        rows = [dict(row) for row in suggestions if isinstance(row, Mapping)] if isinstance(suggestions, (tuple, list)) else []
        self._auto_suggestions = {
            str(row.get("suggestion_id") or f"suggestion-{index}"): row
            for index, row in enumerate(rows)
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
            self.auto_suggestion_metrics_label.setText(
                "Confidence / inliers / residual / detector: --"
            )
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
            if mode == "mountain"
            else None
        )
        sides = suggestion.get("sides")
        sides = sides if isinstance(sides, Mapping) else {}
        if annotation_mode is not None:
            for side, view in self.annotation_views.items():
                row = sides.get(side)
                if not isinstance(row, Mapping):
                    continue
                points = self._auto_points(row.get("points", row.get("annotation_points", ())))
                confidences = self._auto_confidences(row.get("point_confidences", ()))
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
        self.auto_suggestion_metrics_label.setText(
            f"Confidence {confidence_text} · inliers {inliers} · residual {residual_text} · detector {detector}"
        )
        reason = str(suggestion.get("failure_reason") or suggestion.get("reason") or "")
        suffix = f" · {reason}" if reason else ""
        self.auto_suggestion_status_label.setText(
            f"{mode.title() or 'Automatic'} · {state} · {provenance} · {suggestion_id[:18]}{suffix}"
        )
        self.use_suggestion_button.setEnabled(
            bool(suggestion_id)
            and state == "GOOD"
            and bool(suggestion.get("eligible", False))
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
            AnnotationMode.HORIZON if mode == "horizon" else AnnotationMode.RIDGE if mode == "mountain" else None
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
        self.auto_suggestion_status_label.setText(
            f"{mode.title()} · GOOD · AUTO_ACCEPTED · points copied to manual annotation"
        )
        self.use_suggestion_button.setEnabled(False)

    def _on_annotation_changed(self, side: str, payload: object) -> None:
        if self._auto_accept_in_progress:
            return
        current = payload.to_mapping() if hasattr(payload, "to_mapping") else None
        if current is None and not any(
            value is not None for value in self.annotation_payloads().values()
        ):
            return
        expected = self._accepted_annotation_snapshots.get(side)
        if self._accepted_annotation_snapshots and expected is not None and current != expected:
            self._auto_suggestion_provenance = "MANUAL_EDITED"
            self.auto_suggestion_status_label.setText(
                "Automatic suggestion · MANUAL_EDITED · operator annotation differs from AUTO_ACCEPTED points"
            )
            self.use_suggestion_button.setEnabled(False)
        provenance = (
            self._auto_suggestion_provenance
            if self._accepted_annotation_snapshots
            else "MANUAL"
        )
        self.command_requested.emit(
            "alignment.pose.annotation_changed",
            {
                "command_id": "alignment.pose.annotation_changed",
                "suggestion_id": str(getattr(self, "_accepted_suggestion_id", "") or ""),
                "provenance_state": provenance,
                "annotation_mode": str(
                    self.annotation_mode_combo.currentData()
                    or AnnotationMode.HORIZON.value
                ),
                "annotations": self.annotation_payloads(),
                "annotation_contract": (
                    "frozen native OpenCV pixels: u right, v down; identity is "
                    "frame_id + image_sha256 + native dimensions"
                ),
            },
        )

    def _set_annotation_mode(self, _index: int = -1) -> None:
        mode = str(
            self.annotation_mode_combo.currentData()
            or AnnotationMode.HORIZON.value
        )
        for view in self.annotation_views.values():
            view.set_annotation_mode(mode)
        yaw_checkbox = self.dof_checkboxes.get("yaw")
        if yaw_checkbox is not None:
            if mode == AnnotationMode.HORIZON.value:
                if yaw_checkbox.isEnabled():
                    self._yaw_checked_before_horizon = yaw_checkbox.isChecked()
                yaw_checkbox.setChecked(False)
                yaw_checkbox.setEnabled(False)
                yaw_checkbox.setToolTip(
                    "Disabled: a flat horizon observes Roll/Pitch, not Yaw. "
                    "Use paired Mountain u,v + GNSS or Drone RTK for Yaw."
                )
            else:
                was_disabled = not yaw_checkbox.isEnabled()
                yaw_checkbox.setEnabled(True)
                if was_disabled:
                    yaw_checkbox.setChecked(self._yaw_checked_before_horizon)
                yaw_checkbox.setToolTip(
                    "Rotation is PX4/NED Roll/Pitch/Yaw. Mountain u,v + GNSS or Drone RTK "
                    "may provide Yaw when it is observable."
                )

        self.annotation_mode_combo.setToolTip(
            "Click the frozen native image to add points. "
            + self._manual_mode_tooltips.get(mode, "")
        )

    def _clear_current_annotations(self) -> None:
        mode = str(
            self.annotation_mode_combo.currentData()
            or AnnotationMode.HORIZON.value
        )
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

    def set_readiness(self, readiness: Mapping[str, object]) -> None:
        for key, widget in self.readiness_values.items():
            raw = readiness.get(key)
            if isinstance(raw, Mapping):
                display = raw.get("value", raw.get("state", "--"))
                source = str(raw.get("source", "optical pose owner snapshot") or "")
                frame = str(raw.get("frame", "") or "")
                valid = raw.get("valid")
                valid = valid if isinstance(valid, bool) else None
            else:
                display = "--" if raw in (None, "") else raw
                source = "optical pose owner snapshot"
                frame = ""
                valid = None
            widget.set_value(display, source=source, frame=frame, valid=valid)

    def set_active_profile(self, profile_id: object) -> None:
        self._active_profile_id = str(profile_id or "")
        self.revert_button.setEnabled(bool(self._active_profile_id))

    def set_reference_profile(self, calibration: Mapping[str, object]) -> None:
        """Show the existing checkerboard R,t as a comparison reference."""

        extrinsics = calibration.get("extrinsics")
        extrinsics = extrinsics if isinstance(extrinsics, Mapping) else {}
        state = str(extrinsics.get("state", "NO DATA") or "NO DATA")
        baseline = extrinsics.get("baseline_m")
        profile_path = str(
            calibration.get("extrinsics_profile_path", calibration.get("profile_path", ""))
            or ""
        )
        baseline_text = self._candidate_text(baseline, digits=4)
        profile_text = Path(profile_path).name if profile_path else "profile path unavailable"
        self.reference_value.set_value(
            f"{state} · baseline={baseline_text} m · {profile_text}",
            source=str(calibration.get("source", "Calibration owner snapshot") or ""),
            frame=str(extrinsics.get("frame_contract", "OpenCV optical RDF") or ""),
            valid=True if state in {"READY", "VALID", "LOADED", "ACTIVE"} else None,
        )

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
        rows = [dict(value) for value in candidates if isinstance(value, Mapping)] if isinstance(candidates, (tuple, list)) else []
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
        self._apply_candidate_dof_defaults()
        self._refresh_candidate_actions()

    def _selected_drone_rtk_observation_id(self) -> str:
        row = self.observation_history_table.currentRow()
        item = self.observation_history_table.item(row, 0) if row >= 0 else None
        return "" if item is None else str(item.data(QtCore.Qt.UserRole) or "")

    def set_drone_rtk_observations(self, observations: object) -> None:
        """Refresh Observation History from an append-only, immutable list.

        Row identity is the observation_id (mirrors ``set_candidates``'
        candidate_id pattern) so a selected row survives a snapshot poll
        instead of resetting to row 0 or vanishing when a new observation is
        appended.
        """

        rows = (
            [dict(value) for value in observations if isinstance(value, Mapping)]
            if isinstance(observations, (tuple, list))
            else []
        )
        previously_selected = self._selected_drone_rtk_observation_id()
        self._drone_rtk_observation_rows = {}
        self._drone_rtk_observation_order = []
        table = self.observation_history_table
        table.blockSignals(True)
        try:
            table.setRowCount(len(rows))
            selected_row = -1
            for row_index, observation in enumerate(rows):
                observation_id = str(observation.get("observation_id") or "")
                if not observation_id:
                    observation_id = f"observation-{row_index + 1:03d}"
                self._drone_rtk_observation_rows[observation_id] = observation
                self._drone_rtk_observation_order.append(observation_id)
                gnss = observation.get("gnss")
                gnss = gnss if isinstance(gnss, Mapping) else {}
                sync = observation.get("synchronization")
                sync = sync if isinstance(sync, Mapping) else {}
                sequence = observation.get("sequence")
                display_index = (
                    f"{int(sequence) + 1:03d}" if isinstance(sequence, (int, float)) else f"{row_index + 1:03d}"
                )
                values = (
                    display_index,
                    str(observation.get("state") or "--"),
                    _drone_rtk_gnss_label(gnss.get("drone")),
                    _drone_rtk_gnss_label(gnss.get("left")),
                    _drone_rtk_gnss_label(gnss.get("right")),
                    _format_drone_rtk_sync_delta_ms(sync.get("left_frame_to_drone_gnss_s")),
                    _format_drone_rtk_sync_delta_ms(sync.get("right_frame_to_drone_gnss_s")),
                    "NOT ANNOTATED",
                )
                for column, value in enumerate(values):
                    item = QtWidgets.QTableWidgetItem(str(value))
                    item.setData(QtCore.Qt.UserRole, observation_id)
                    table.setItem(row_index, column, item)
                if observation_id == previously_selected:
                    selected_row = row_index
            if selected_row < 0 and rows:
                # Default to the most recent capture, not row 0, since this
                # history is append-only and the newest entry is what an
                # operator just produced.
                selected_row = len(rows) - 1
            if selected_row >= 0:
                table.selectRow(selected_row)
                table.setCurrentCell(selected_row, 0)
            else:
                table.clearSelection()
        finally:
            table.blockSignals(False)
        self._refresh_drone_rtk_observation_detail()

    def _refresh_drone_rtk_observation_detail(self) -> None:
        latest_id = self._drone_rtk_observation_order[-1] if self._drone_rtk_observation_order else ""
        self._apply_current_drone_rtk_observation_summary(
            self._drone_rtk_observation_rows.get(latest_id)
        )
        selected_id = self._selected_drone_rtk_observation_id() or latest_id
        self._apply_selected_drone_rtk_observation_detail(
            self._drone_rtk_observation_rows.get(selected_id)
        )

    def _apply_current_drone_rtk_observation_summary(self, observation: object) -> None:
        if not isinstance(observation, Mapping):
            self.current_observation_summary_label.setText("No observation yet")
            for value in self.current_observation_values.values():
                value.set_value("--")
            return
        sequence = observation.get("sequence")
        display_index = f"#{int(sequence) + 1:03d}" if isinstance(sequence, (int, float)) else "#--"
        state = str(observation.get("state") or "--")
        self.current_observation_summary_label.setText(f"{display_index} · {state}")
        gnss = observation.get("gnss")
        gnss = gnss if isinstance(gnss, Mapping) else {}
        sync = observation.get("synchronization")
        sync = sync if isinstance(sync, Mapping) else {}
        drone = gnss.get("drone")
        self.current_observation_values["drone_rtk"].set_value(_drone_rtk_gnss_label(drone))
        age_text = "--"
        if isinstance(drone, Mapping) and drone.get("timestamp_s") is not None:
            age_s = max(0.0, time.monotonic() - float(drone["timestamp_s"]))
            age_text = f"{age_s:.2f} s"
        self.current_observation_values["gnss_age"].set_value(age_text)
        left_delta = _format_drone_rtk_sync_delta_ms(sync.get("left_frame_to_drone_gnss_s"))
        right_delta = _format_drone_rtk_sync_delta_ms(sync.get("right_frame_to_drone_gnss_s"))
        self.current_observation_values["sync"].set_value(f"L {left_delta} / R {right_delta}")
        # Target-pixel <-> observation binding is a later phase; always show
        # the same explicit "not yet connected" state rather than guessing.
        self.current_observation_values["target"].set_value("NOT ANNOTATED")

    @staticmethod
    def _apply_gnss_detail_rows(
        values: dict[str, "EngineeringValue"],
        *,
        prefix: str,
        row: object,
    ) -> None:
        row = row if isinstance(row, Mapping) else {}
        values[f"{prefix}_latitude"].set_value(_format_drone_rtk_degrees(row.get("latitude_deg")))
        values[f"{prefix}_longitude"].set_value(_format_drone_rtk_degrees(row.get("longitude_deg")))
        values[f"{prefix}_altitude"].set_value(_format_drone_rtk_metres(row.get("altitude_m")))
        values[f"{prefix}_ellipsoid_height"].set_value(
            _format_drone_rtk_metres(row.get("altitude_ellipsoid_m"))
        )
        values[f"{prefix}_fix"].set_value(_drone_rtk_gnss_label(row))
        satellites = row.get("satellites_used")
        values[f"{prefix}_satellites"].set_value("--" if satellites is None else str(satellites))
        eph = _format_drone_rtk_metres(row.get("eph_m"))
        epv = _format_drone_rtk_metres(row.get("epv_m"))
        values[f"{prefix}_eph_epv"].set_value("--" if eph == "--" and epv == "--" else f"{eph} / {epv}")
        values[f"{prefix}_timestamp"].set_value(_format_drone_rtk_seconds(row.get("timestamp_s")))

    @staticmethod
    def _apply_frame_detail_rows(
        values: dict[str, "EngineeringValue"],
        *,
        prefix: str,
        frame: object,
    ) -> None:
        frame = frame if isinstance(frame, Mapping) else {}
        values[f"{prefix}_frame_id"].set_value(frame.get("frame_id") or "--")
        sha = str(frame.get("sha256") or "")
        values[f"{prefix}_frame_sha"].set_value(sha[:12] if sha else "--")
        values[f"{prefix}_frame_timestamp"].set_value(_format_drone_rtk_seconds(frame.get("timestamp_s")))
        values[f"{prefix}_frame_timestamp_reference"].set_value(
            frame.get("timestamp_reference") or "--"
        )

    def _apply_selected_drone_rtk_observation_detail(self, observation: object) -> None:
        values = self.observation_detail_values
        if not isinstance(observation, Mapping):
            for value in values.values():
                value.set_value("--")
            return
        values["observation_id"].set_value(observation.get("observation_id") or "--")
        values["observation_state"].set_value(observation.get("state") or "--")
        values["observation_burst_id"].set_value(observation.get("burst_id") or "--")
        gnss = observation.get("gnss")
        gnss = gnss if isinstance(gnss, Mapping) else {}
        self._apply_gnss_detail_rows(values, prefix="drone", row=gnss.get("drone"))
        self._apply_gnss_detail_rows(values, prefix="p1", row=gnss.get("left"))
        self._apply_gnss_detail_rows(values, prefix="p2", row=gnss.get("right"))
        frames = observation.get("frames")
        frames = frames if isinstance(frames, Mapping) else {}
        self._apply_frame_detail_rows(values, prefix="left", frame=frames.get("left"))
        self._apply_frame_detail_rows(values, prefix="right", frame=frames.get("right"))
        sync = observation.get("synchronization")
        sync = sync if isinstance(sync, Mapping) else {}
        values["sync_state"].set_value(sync.get("state") or "--")
        values["sync_delta_left"].set_value(
            _format_drone_rtk_sync_delta_ms(sync.get("left_frame_to_drone_gnss_s"))
        )
        values["sync_delta_right"].set_value(
            _format_drone_rtk_sync_delta_ms(sync.get("right_frame_to_drone_gnss_s"))
        )
        values["sync_reason"].set_value(sync.get("reason") or "--")

    def apply_snapshot(self, snapshot: Mapping[str, object]) -> None:
        readiness = snapshot.get("readiness")
        if isinstance(readiness, Mapping):
            self.set_readiness(readiness)
        candidates = snapshot.get("candidates")
        if isinstance(candidates, (tuple, list)):
            self.set_candidates(candidates)
        self.set_active_profile(snapshot.get("active_profile_id"))
        drone_rtk_observations = snapshot.get("drone_rtk_capture_observations")
        if isinstance(drone_rtk_observations, (tuple, list)):
            self.set_drone_rtk_observations(drone_rtk_observations)
        # Text-only owner state remains supported, while ``frozen_frames`` is
        # the explicit QImage-bearing snapshot used by the annotation editor.
        for key in ("frozen_previews", "frozen_frames"):
            frozen = snapshot.get(key)
            if isinstance(frozen, Mapping):
                for side in ("left", "right"):
                    if side in frozen:
                        self.set_frozen_preview_state(side, frozen[side])
        auto_suggestions = snapshot.get("auto_suggestions")
        if isinstance(auto_suggestions, (tuple, list)):
            self.set_auto_suggestions(auto_suggestions)
        accepted = snapshot.get("accepted_auto_suggestion")
        if isinstance(accepted, Mapping):
            self._apply_accepted_auto_suggestion(accepted)

    def _selected_candidate_applicable(self) -> bool:
        row = self.candidate_table.currentRow()
        item = self.candidate_table.item(row, 0) if row >= 0 else None
        return bool(item is not None and item.data(QtCore.Qt.UserRole + 1))

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

    @classmethod
    def _candidate_detail_text(cls, candidate: Mapping[str, object]) -> str:
        """Render provenance/quality/warnings for the selected candidate.

        Purely additive review text: it only formats fields the owner
        snapshot already exposes (``input_hashes``/``quality_metrics``/
        ``warnings``) so a reviewer can see each value's source without a
        dedicated widget per method.
        """

        lines: list[str] = []
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
        if isinstance(warnings, (tuple, list)) and warnings:
            lines.append("warnings:")
            lines.extend(f"  - {text}" for text in warnings)
        return "\n".join(lines)

    def _refresh_candidate_actions(self) -> None:
        candidate_id = self.selected_candidate_id()
        self.apply_button.setEnabled(bool(candidate_id) and self._selected_candidate_applicable())
        detail_widget = getattr(self, "candidate_detail_text", None)
        if not candidate_id:
            self.candidate_status_label.setText("No pose candidate")
            if detail_widget is not None:
                detail_widget.setPlainText("")
            return
        candidate = self._candidate_rows.get(candidate_id, {})
        state = candidate.get("state", candidate.get("quality", "available"))
        self.candidate_status_label.setText(f"{candidate_id} · {state}")
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
                item = QtWidgets.QListWidgetItem(
                    f"{method} · {candidate_id} · DOF={observable} · {state}"
                )
                item.setData(QtCore.Qt.UserRole, candidate_id)
                item.setFlags(
                    item.flags()
                    | QtCore.Qt.ItemIsUserCheckable
                    | QtCore.Qt.ItemIsEnabled
                )
                item.setCheckState(
                    QtCore.Qt.Checked if candidate_id in selected_ids else QtCore.Qt.Unchecked
                )
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
                if (cell := table.item(row, column)) is not None
                and cell.checkState() == QtCore.Qt.Checked
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
        button = self.solver_buttons.get("alignment.pose.solve_combined")
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
            status.setText(
                "Axis claimed by more than one candidate: "
                + ", ".join(sorted(overlap))
                + " · uncheck one"
            )
        else:
            status.setText(
                f"{selected_count} candidates checked · owner will verify shared prior and exact K,D"
            )

    def _apply_candidate_dof_defaults(self) -> None:
        """Expose the full metric mask when GNSS constrains Mountain u,v."""

        candidate_id = self.selected_candidate_id()
        if not candidate_id or candidate_id == self._dof_default_candidate_id:
            return
        self._dof_default_candidate_id = candidate_id
        candidate = self._candidate_rows.get(candidate_id, {})
        if str(candidate.get("method", "") or "") != "gnss-constrained-mountain-uv":
            return
        for key, checkbox in self.dof_checkboxes.items():
            checkbox.setChecked(True)

    def _request_apply(self) -> None:
        payload = self.apply_payload()
        if payload is not None:
            self.command_requested.emit("alignment.pose.apply_candidate", payload)

    def _request_revert(self) -> None:
        self.command_requested.emit("alignment.pose.revert", self.revert_payload())


class _ContentSizedOperationTabs(QtWidgets.QTabWidget):
    """Size the operation strip from its active page, not the largest page."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.currentChanged.connect(self._refresh_geometry)

    def _refresh_geometry(self, _index: int = -1) -> None:
        self.updateGeometry()
        parent = self.parentWidget()
        while parent is not None:
            parent.updateGeometry()
            parent_layout = parent.layout()
            if parent_layout is not None:
                parent_layout.invalidate()
            parent = parent.parentWidget()

    def _active_page_height(self, *, minimum: bool) -> int:
        current = self.currentWidget()
        if current is None:
            return 0
        page_height = (
            current.minimumSizeHint().height()
            if minimum
            else current.sizeHint().height()
        )
        all_pages = [
            (
                self.widget(index).minimumSizeHint().height()
                if minimum
                else self.widget(index).sizeHint().height()
            )
            for index in range(self.count())
        ]
        base = (
            super().minimumSizeHint()
            if minimum
            else super().sizeHint()
        )
        largest_page = max(all_pages, default=0)
        tab_overhead = max(0, base.height() - largest_page)
        return page_height + tab_overhead

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt API.
        hint = super().sizeHint()
        active_height = self._active_page_height(minimum=False)
        if active_height > 0:
            hint.setHeight(active_height)
        return hint

    def minimumSizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt API.
        hint = super().minimumSizeHint()
        active_height = self._active_page_height(minimum=True)
        if active_height > 0:
            hint.setHeight(active_height)
        return hint


class AlignmentWorkspace(QtWidgets.QWidget):
    """Post-Devices workspace for measuring and comparing camera R,t."""

    inspection_requested = QtCore.pyqtSignal(object)
    optical_pose_command_requested = QtCore.pyqtSignal(str, object)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_advanced: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader("Alignment")
        )

        # Alignment owns R,t evidence only. Device connection, Survey-in, and
        # their command surfaces stay in Devices; this page compares geometry
        # candidates below.
        self.operation_tabs = self._build_operation_tabs(invoke_command)
        # The Alignment page itself lives in an outer scroll area.  Give the
        # R,t surface the remaining viewport height so hiding the bottom
        # Diagnostics dock expands the camera/annotation scroll surface all
        # the way to the window bottom. The LRF card is part of that same
        # scroll content, so it does not remain pinned as a separate footer.
        self.operation_tabs.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        layout.addWidget(self.operation_tabs, 1)
        self.indicators: dict[str, StatusIndicator] = {}

        attitude_group = QtWidgets.QGroupBox("Attitude")
        attitude_layout = QtWidgets.QVBoxLayout(attitude_group)
        self.attitude_model = _table_model(
            self,
            ("Side", "State", "Roll°", "Pitch°", "Yaw°", "Age", "PX4 t", "Transport"),
        )
        self.attitude_table = QtWidgets.QTableView()
        self.attitude_table.setModel(self.attitude_model)
        self.attitude_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.attitude_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.attitude_table.verticalHeader().setVisible(False)
        self.attitude_table.setMaximumHeight(112)
        self.attitude_table.horizontalHeader().setStretchLastSection(True)
        self.attitude_table.clicked.connect(self._inspect_attitude_row)
        attitude_layout.addWidget(self.attitude_table)
        layout.addWidget(attitude_group)
        attitude_group.setVisible(False)

        gnss_group = QtWidgets.QGroupBox("GNSS")
        gnss_layout = QtWidgets.QVBoxLayout(gnss_group)
        self.gnss_model = _table_model(
            self,
            ("Receiver", "RTK", "Fix", "Sats", "Eph m", "Epv m", "Lat°", "Lon°", "Alt m", "Age", "Source"),
        )
        self.gnss_table = QtWidgets.QTableView()
        self.gnss_table.setModel(self.gnss_model)
        self.gnss_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.gnss_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.gnss_table.verticalHeader().setVisible(False)
        self.gnss_table.setMaximumHeight(128)
        self.gnss_table.horizontalHeader().setStretchLastSection(True)
        self.gnss_table.clicked.connect(self._inspect_gnss_row)
        gnss_layout.addWidget(self.gnss_table)
        layout.addWidget(gnss_group)
        gnss_group.setVisible(False)

        baseline_group = QtWidgets.QGroupBox("RTK baseline")
        baseline_layout = QtWidgets.QGridLayout(baseline_group)
        self.baseline_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (("north_m", "North"), ("east_m", "East"), ("down_m", "Down"), ("length_m", "Length"), ("state", "Quality"))
        ):
            value = EngineeringValue(label)
            self.baseline_values[key] = value
            value.activated.connect(lambda key=key: self._inspect_alignment_value("ned", key))
            baseline_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(baseline_group)
        baseline_group.setVisible(False)

        body_frd_group = QtWidgets.QGroupBox("P1→P2 (FRD)")
        body_frd_layout = QtWidgets.QGridLayout(body_frd_group)
        self.p1_body_frd_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("p1_body_frd_forward_m", "Forward"),
                ("p1_body_frd_right_m", "Right"),
                ("p1_body_frd_down_m", "Down"),
                ("p1_body_frd_state", "State"),
            )
        ):
            value = EngineeringValue(label)
            self.p1_body_frd_values[key] = value
            value.activated.connect(lambda key=key: self._inspect_alignment_value("frd", key))
            body_frd_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(body_frd_group)
        body_frd_group.setVisible(False)

        physical_group = QtWidgets.QGroupBox("Physical alignment evidence")
        physical_layout = QtWidgets.QGridLayout(physical_group)
        self.physical_alignment_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("state", "State"),
                ("relative_rpy", "Δ Roll/Pitch/Yaw"),
                ("baseline_frd", "P1→P2 FRD"),
                ("fix_type", "Numeric fix_type"),
                ("sample_count", "Samples"),
                ("fixed_ratio", "RTK_FIXED ratio"),
            )
        ):
            value = EngineeringValue(label)
            self.physical_alignment_values[key] = value
            value.activated.connect(lambda key=key: self._inspect_alignment_value("physical", key))
            physical_layout.addWidget(value, index // 3, index % 3)
        physical_note = QtWidgets.QLabel(
            "Adjust the hardware physically. Workstation records baseline and attitude evidence; it applies no R/P/Y correction."
        )
        physical_note.setObjectName("compactContract")
        physical_note.setWordWrap(True)
        physical_layout.addWidget(physical_note, 2, 0, 1, 3)
        self.alignment_iteration_model = _table_model(
            self,
            ("Iteration", "State", "Samples", "Fix ratio", "FRD mean [F,R,D]", "ΔR/P/Y mean"),
        )
        self.alignment_iteration_table = QtWidgets.QTableView()
        self.alignment_iteration_table.setModel(self.alignment_iteration_model)
        self.alignment_iteration_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.alignment_iteration_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.alignment_iteration_table.verticalHeader().setVisible(False)
        self.alignment_iteration_table.setMaximumHeight(112)
        self.alignment_iteration_table.horizontalHeader().setStretchLastSection(True)
        self.alignment_iteration_table.clicked.connect(self._inspect_iteration_row)
        physical_layout.addWidget(self.alignment_iteration_table, 3, 0, 1, 3)
        layout.addWidget(physical_group)
        physical_group.setVisible(False)

        contract = QtWidgets.QLabel(
            "Frames: PX4 FRD→NED · RTK NED · P1→P2 FRD · GPS WGS84"
        )
        contract.setObjectName("compactContract")
        layout.addWidget(contract)
        contract.setVisible(False)

        button = QtWidgets.QPushButton("Advanced diagnostics")
        button.setToolTip(
            "Open PX4 MAG, field watch, and low-level alignment diagnostics."
        )
        button.clicked.connect(open_advanced)
        layout.addWidget(button, alignment=QtCore.Qt.AlignRight)
        button.setVisible(False)
        # ``operation_tabs`` owns the page's remaining height.  A trailing
        # stretch would split that space with the R,t surface and leave the
        # camera viewport stopping above the bottom of the Alignment window.

    def _build_operation_tabs(
        self,
        invoke_command: Callable[[str], bool] | None,
    ) -> QtWidgets.QTabWidget:
        operation_tabs = _ContentSizedOperationTabs()
        operation_tabs.setObjectName("alignmentOperationTabs")
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        # Keep one scrollable R,t work surface. Its inner method tabs split
        # Horizon, Mountain + GNSS, Drone RTK, existing calibration, and
        # candidate composition without duplicating Devices commands here.
        self.optical_pose_pane = OpticalPosePane()
        self.optical_pose_pane.command_requested.connect(
            self.optical_pose_command_requested.emit
        )
        self.operation_buttons.update(self.optical_pose_pane.solver_buttons)
        self.operation_buttons["alignment.pose.apply_candidate"] = self.optical_pose_pane.apply_button
        self.operation_buttons["alignment.pose.revert"] = self.optical_pose_pane.revert_button

        # Keep LRF status in the same scrollable content as the R,t controls.
        # It is an on-demand read: each refresh opens/closes its own control
        # port instead of starting a continuous background poll.
        lrf_group = QtWidgets.QGroupBox("LRF (Laser Range Finder)")
        lrf_group.setObjectName("alignmentLrfGroup")
        lrf_layout = QtWidgets.QGridLayout(lrf_group)
        self.lrf_group = lrf_group
        self.lrf_indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (("lrf.left", "CAM L"), ("lrf.right", "CAM R"))
        ):
            indicator = StatusIndicator(label)
            self.lrf_indicators[key] = indicator
            lrf_layout.addWidget(indicator, 0, index)
        lrf_refresh_button = _command_button("Refresh LRF", "alignment.query_lrf", invoke_command)
        self.operation_buttons["alignment.query_lrf"] = lrf_refresh_button
        lrf_layout.addWidget(lrf_refresh_button, 0, 2)
        self.optical_pose_pane.layout().addWidget(lrf_group)

        optical_pose_scroll = QtWidgets.QScrollArea()
        optical_pose_scroll.setObjectName("alignmentOpticalPoseScroll")
        optical_pose_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        optical_pose_scroll.setWidgetResizable(True)
        optical_pose_scroll.setMinimumSize(0, 0)
        optical_pose_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        optical_pose_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        optical_pose_scroll.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustIgnored)
        optical_pose_scroll.setWidget(self.optical_pose_pane)
        self.optical_pose_scroll = optical_pose_scroll
        operation_tabs.addTab(optical_pose_scroll, "R,t methods")
        return operation_tabs

    def select_tab_for_context(self, context_id: str) -> None:
        # Device contexts have their own workspaces. Alignment has one
        # R,t-focused surface, so navigation never switches a device tab here.
        del context_id

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        self.optical_pose_pane.set_reference_profile(evidence.calibration())

        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        for key, indicator in self.lrf_indicators.items():
            indicator.set_status(snapshot.get(key))
        attitude_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            data = evidence.attitude(side)
            attitude_rows.append(
                (
                    side.upper(),
                    data.get("state", "WAITING"),
                    _display_number(data.get("roll_deg")),
                    _display_number(data.get("pitch_deg")),
                    _display_number(data.get("yaw_deg")),
                    _display_number(data.get("age_s"), digits=2, suffix="s"),
                    _display_number(data.get("px4_timestamp_s"), digits=6),
                    "MAVLink",
                ),
            )
        _sync_table_rows(self.attitude_model, tuple(attitude_rows))
        gnss_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            data = evidence.gps(side)
            gnss_rows.append(
                (
                    side.upper(),
                    data.get("rtk_label", data.get("state", "NO DATA")),
                    data.get("fix_type", "--"),
                    data.get("satellites", "--"),
                    _display_number(data.get("eph_m")),
                    _display_number(data.get("epv_m")),
                    _display_number(data.get("latitude_deg"), digits=7),
                    _display_number(data.get("longitude_deg"), digits=7),
                    _display_number(data.get("altitude_m")),
                    _display_number(data.get("age_s"), digits=2, suffix="s"),
                    "MAVLink GPS",
                ),
            )
        _sync_table_rows(self.gnss_model, tuple(gnss_rows))
        baseline = evidence.baseline()
        for key, value in self.baseline_values.items():
            raw = baseline.get(key)
            self.baseline_values[key].set_value(
                _display_number(raw, suffix=" m") if key.endswith("_m") else raw,
                source=str(baseline.get("source", "")),
                frame="NED north/east/down" if key.endswith("_m") else "",
                valid=(None if baseline.get("state") in {"NO DATA", "UNKNOWN"} else baseline.get("state") == "GOOD"),
            )
        projection_state = str(baseline.get("p1_body_frd_state", "WAITING") or "WAITING")
        projection_valid = projection_state == "READY"
        projection_age_s = baseline.get("p1_body_frd_attitude_age_s")
        try:
            projection_age_s = None if projection_age_s in (None, "") else float(projection_age_s)
        except (TypeError, ValueError):
            projection_age_s = None
        for key, value in self.p1_body_frd_values.items():
            raw = baseline.get(key)
            is_metric = key.endswith("_m")
            value.set_value(
                _display_number(raw, suffix=" m") if is_metric else raw,
                source=str(baseline.get("p1_body_frd_source", "")),
                age_s=(
                    projection_age_s
                    if key == "p1_body_frd_state"
                    else None
                ),
                frame=(
                    "P1 body FRD P1→P2 [Forward,Right,Down]"
                    if is_metric
                    else str(baseline.get("p1_body_frd_axis_contract", ""))
                ),
                valid=(None if projection_state == "WAITING" else projection_valid),
            )
        physical = evidence.physical_alignment()
        quality = physical.get("quality") if isinstance(physical.get("quality"), dict) else {}
        physical_state = str(physical.get("state", "WAITING") or "WAITING")
        relative_rpy = physical.get("relative_rpy_deg")
        relative_text = "--"
        if isinstance(relative_rpy, (tuple, list)) and len(relative_rpy) == 3:
            relative_text = "R/P/Y=" + "/".join(f"{float(item):+.2f}°" for item in relative_rpy)
        baseline_frd = physical.get("current_baseline_frd_m")
        baseline_frd_text = "--"
        if isinstance(baseline_frd, (tuple, list)) and len(baseline_frd) == 3 and all(item is not None for item in baseline_frd):
            baseline_frd_text = "F/R/D=" + "/".join(f"{float(item):+.3f} m" for item in baseline_frd)
        fix_types = physical.get("fix_type")
        fix_type_text = "--"
        if isinstance(fix_types, dict):
            fix_type_text = f"P1={fix_types.get('left', '--')} / P2={fix_types.get('right', '--')}"
        fixed_ratio = quality.get("fixed_ratio")
        ratio_text = "--" if fixed_ratio in (None, "") else f"{float(fixed_ratio) * 100.0:.1f}%"
        physical_display = {
            "state": physical_state,
            "relative_rpy": relative_text,
            "baseline_frd": baseline_frd_text,
            "fix_type": fix_type_text,
            "sample_count": physical.get("sample_count", quality.get("sample_count", "--")),
            "fixed_ratio": ratio_text,
        }
        for key, value in self.physical_alignment_values.items():
            value.set_value(
                physical_display.get(key),
                source=str(physical.get("coordinate_contract", "")),
                valid=(
                    None
                    if physical_state in {"WAITING", "LIVE", "SAMPLING"}
                    else physical_state in {"PASS", "EVIDENCE"}
                ),
            )
        iteration_rows: list[tuple[object, ...]] = []
        iterations = physical.get("iterations", ())
        if isinstance(iterations, (tuple, list)):
            for iteration in iterations:
                if not isinstance(iteration, dict):
                    continue
                frd_mean = iteration.get("baseline_frd_mean_m", ())
                rpy_mean = iteration.get("relative_rpy_mean_deg", ())
                frd_text = "--" if len(frd_mean) != 3 else "/".join(f"{float(item):+.3f}" for item in frd_mean)
                rpy_text = "--" if len(rpy_mean) != 3 else "/".join(f"{float(item):+.2f}°" for item in rpy_mean)
                iteration_rows.append(
                    (
                        iteration.get("iteration", "--"),
                        iteration.get("state", "--"),
                        iteration.get("sample_count", "--"),
                        _display_number(float(iteration.get("fixed_ratio")) * 100.0, digits=1, suffix="%")
                        if iteration.get("fixed_ratio") not in (None, "")
                        else "--",
                        frd_text,
                        rpy_text,
                    ),
                )
        _sync_table_rows(self.alignment_iteration_model, tuple(iteration_rows))
        recording = snapshot.get("recording")
        recording_locked = recording is not None and recording.state in {"ACTIVE", "FINALIZING"}
        swap_button = self.operation_buttons.get("alignment.swap_pixhawks")
        if swap_button is not None:
            swap_button.setEnabled(not recording_locked)
        reset_level_button = self.operation_buttons.get("pixhawk.reset_level_calibration")
        if reset_level_button is not None:
            reset_level_button.setEnabled(not recording_locked)
        self._refresh_optical_pose_snapshot()

    def _refresh_optical_pose_snapshot(self) -> None:
        """Render an optional owner snapshot without inventing pose state."""

        panel = getattr(self.backend, "panel", self.backend)
        readers = (
            getattr(self.backend, "optical_pose_snapshot", None),
            getattr(panel, "workstation_optical_pose_snapshot", None),
        )
        for reader in readers:
            if not callable(reader):
                continue
            try:
                snapshot = reader()
            except Exception:
                return
            if isinstance(snapshot, Mapping):
                self.optical_pose_pane.apply_snapshot(snapshot)
            return

    def _inspect_attitude_row(self, index: QtCore.QModelIndex) -> None:
        side = "left" if index.row() == 0 else "right"
        self.inspection_requested.emit(
            _live_inspection(
                self._attitude_inspection(side),
                lambda side=side: self._attitude_inspection(side),
            )
        )

    def _attitude_inspection(self, side: str) -> dict[str, object]:
        label = "P1" if side == "left" else "P2"
        data = _evidence(self.backend).attitude(side)
        return {
            "selection": f"Alignment input · {label} attitude",
            "value": f"R/P/Y={data.get('roll_deg', '--')}/{data.get('pitch_deg', '--')}/{data.get('yaw_deg', '--')}°",
            "freshness": f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}",
            "contract": str(data.get("orientation_contract", "PX4 body FRD → world NED")),
            "verdict": str(data.get("state", "WAITING")),
            "source": str(data.get("source", "MAVLink attitude telemetry")),
        }

    def _inspect_gnss_row(self, index: QtCore.QModelIndex) -> None:
        side = "left" if index.row() == 0 else "right"
        self.inspection_requested.emit(
            _live_inspection(
                self._gnss_inspection(side),
                lambda side=side: self._gnss_inspection(side),
            )
        )

    def _gnss_inspection(self, side: str) -> dict[str, object]:
        label = "P1" if side == "left" else "P2"
        data = _evidence(self.backend).gps(side)
        return {
            "selection": f"Alignment input · {label} GNSS",
            "value": (
                f"fix={data.get('fix_type', '--')} · {data.get('rtk_label', 'NO DATA')} · "
                f"sats={data.get('satellites', '--')} · eph/epv={data.get('eph_m', '--')}/{data.get('epv_m', '--')} m"
            ),
            "freshness": f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}",
            "contract": str(data.get("position_contract", "WGS84 receiver position; baseline is derived separately in NED")),
            "verdict": str(data.get("state", "NO DATA")),
            "source": str(data.get("source", "MAVLink GPS telemetry")),
        }

    def _inspect_alignment_value(self, group: str, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._alignment_value_inspection(group, key),
                lambda group=group, key=key: self._alignment_value_inspection(group, key),
            )
        )

    def _alignment_value_inspection(self, group: str, key: str) -> dict[str, object]:
        evidence = _evidence(self.backend)
        if group in {"ned", "frd"}:
            data = evidence.baseline()
            widgets = self.baseline_values if group == "ned" else self.p1_body_frd_values
            is_metric = key.endswith("_m")
            contract = (
                str(data.get("axis_contract", "P1→P2 local NED [North,East,Down]; metres"))
                if group == "ned"
                else str(data.get("p1_body_frd_axis_contract", "P1→P2 in P1 body FRD [Forward,Right,Down]; metres"))
            )
            freshness = (
                f"P1/P2 age={data.get('left_age_s', '--')}/{data.get('right_age_s', '--')}s"
                if group == "ned"
                else f"P1 attitude age={data.get('p1_body_frd_attitude_age_s', '--')}s"
            )
            verdict = data.get("state", "UNKNOWN") if group == "ned" else data.get("p1_body_frd_state", "WAITING")
            value = f"{_display_number(data.get(key))}{' m' if is_metric else ''}"
            source = data.get("source" if group == "ned" else "p1_body_frd_source", "baseline evidence")
        else:
            data = evidence.physical_alignment()
            widgets = self.physical_alignment_values
            value = widgets[key].value.text()
            freshness = f"samples={data.get('sample_count', 0)}"
            contract = str(data.get("coordinate_contract", "raw NED; P1 body FRD; relative R/P/Y"))
            verdict = str(data.get("state", "WAITING"))
            source = "Physical alignment evidence"
        return {
            "selection": widgets[key].label.text(),
            "value": value,
            "freshness": freshness,
            "contract": contract,
            "verdict": verdict,
            "source": source,
        }

    def _inspect_iteration_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        self.inspection_requested.emit(
            _live_inspection(
                self._iteration_inspection(row),
                lambda row=row: self._iteration_inspection(row),
            )
        )

    def _iteration_inspection(self, row: int) -> dict[str, object]:
        if row < 0 or row >= self.alignment_iteration_model.rowCount():
            return {
                "selection": "Alignment iteration",
                "value": "--",
                "freshness": "selected row is no longer retained",
                "contract": "P1→P2 in P1 body FRD [Forward,Right,Down] metres; ΔR/P/Y degrees",
                "verdict": "EXPIRED",
                "source": "Physical alignment iteration history",
            }
        values = [self.alignment_iteration_model.item(row, column).text() for column in range(self.alignment_iteration_model.columnCount())]
        return {
            "selection": f"Alignment iteration {values[0]}",
            "value": f"FRD mean={values[4]} m · ΔR/P/Y mean={values[5]}",
            "freshness": f"samples={values[2]} · fixed ratio={values[3]}",
            "contract": "P1→P2 in P1 body FRD [Forward,Right,Down] metres; ΔR/P/Y degrees",
            "verdict": values[1],
            "source": "Physical alignment iteration history",
        }


class CalibrationWorkspace(QtWidgets.QWidget):
    """Calibration evidence with owner-backed capture/run/save/apply commands."""

    def __init__(
        self,
        backend: object,
        open_advanced: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.invoke_command = invoke_command
        self._camera_preview_metadata: dict[str, object] = {"left": {}, "right": {}}
        self._checkerboard_overlay_active = {"left": False, "right": False}
        self._checkerboard_preview_supported = callable(
            getattr(self.backend, "advanced_refresh_calibration_preview", None)
        )
        self._checkerboard_preview_timer = QtCore.QTimer(self)
        self._checkerboard_preview_timer.setInterval(500)
        self._checkerboard_preview_timer.timeout.connect(self._request_checkerboard_preview)
        self.setup_widgets: dict[str, QtWidgets.QWidget] = {}
        self._initial_calibration_setup = dict(_evidence(self.backend).calibration_setup())
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Calibration",
            )
        )

        zoom_group = QtWidgets.QGroupBox("Actual zoom readback")
        zoom_layout = QtWidgets.QHBoxLayout(zoom_group)
        self.zoom_level_spin = QtWidgets.QSpinBox()
        self.zoom_level_spin.setRange(1, 100)
        self.zoom_level_spin.setValue(1)
        self.zoom_level_spin.setSuffix("×")
        self.zoom_set_button = QtWidgets.QPushButton("Set both")
        self.zoom_query_button = QtWidgets.QPushButton("GZVV")
        self.zoom_readback_label = QtWidgets.QLabel("left=unknown, right=unknown")
        self.zoom_readback_label.setObjectName("statusStrip")
        self.zoom_readback_label.setWordWrap(True)
        self.zoom_set_button.clicked.connect(
            lambda: self._send_calibration_zoom("set", self.zoom_level_spin.value())
        )
        self.zoom_query_button.clicked.connect(lambda: self._send_calibration_zoom("query", None))
        zoom_layout.addWidget(QtWidgets.QLabel("Level"))
        zoom_layout.addWidget(self.zoom_level_spin)
        zoom_layout.addWidget(self.zoom_set_button)
        zoom_layout.addWidget(self.zoom_query_button)
        zoom_layout.addWidget(self.zoom_readback_label, 1)
        layout.addWidget(zoom_group)

        camera_group = QtWidgets.QGroupBox("Live cameras")
        camera_layout = QtWidgets.QVBoxLayout(camera_group)
        camera_layout.setContentsMargins(6, 6, 6, 6)
        camera_layout.setSpacing(5)
        self.preview_splitter = CameraPreviewPair(self)
        self.previews = {side: CameraPreview(side) for side in ("left", "right")}
        for side in ("left", "right"):
            preview = self.previews[side]
            self.preview_splitter.addWidget(preview)
        self.preview_splitter.setStretchFactor(0, 1)
        self.preview_splitter.setStretchFactor(1, 1)
        camera_layout.addWidget(self.preview_splitter)

        camera_actions = QtWidgets.QHBoxLayout()
        self.camera_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (("camera.toggle", "Preview"),):
            button = _command_button(label, command_id, invoke_command)
            self.camera_buttons[command_id] = button
            camera_actions.addWidget(button)
        camera_actions.addStretch(1)
        camera_layout.addLayout(camera_actions)

        self.operation_tabs = self._build_operation_tabs()
        camera_layout.addWidget(self.operation_tabs)
        layout.addWidget(camera_group)
        self._connect_camera_panes()
        self._connect_calibration_preview_signal()

        profile_group = QtWidgets.QGroupBox("Profile")
        profile_layout = QtWidgets.QGridLayout(profile_group)
        self.profile_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("state", "State"),
                ("profile_path", "Profile"),
                ("zoom", "Zoom"),
                ("extrinsics_profile_path", "R,t profile"),
                ("extrinsics_source_zoom", "R,t source zoom"),
                ("combination", "Combination"),
                ("zoom_readback", "Actual zoom"),
                ("validation_mode", "Calculation mode"),
                ("status_label", "Result"),
                ("valid_pair_count", "Valid pairs"),
                ("progress", "Progress"),
            )
        ):
            value = EngineeringValue(label)
            self.profile_values[key] = value
            profile_layout.addWidget(value, index // 2, index % 2)
        layout.addWidget(profile_group)

        intrinsic_group = QtWidgets.QGroupBox("Intrinsics")
        intrinsic_layout = QtWidgets.QVBoxLayout(intrinsic_group)
        self.intrinsic_model = _table_model(
            self,
            ("Camera", "Grid px", "fx px", "fy px", "cx px", "cy px", "Reproj px", "Images", "Frame"),
        )
        self.intrinsic_table = QtWidgets.QTableView()
        self.intrinsic_table.setModel(self.intrinsic_model)
        self.intrinsic_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.intrinsic_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.intrinsic_table.verticalHeader().setVisible(False)
        self.intrinsic_table.setMaximumHeight(108)
        self.intrinsic_table.horizontalHeader().setStretchLastSection(True)
        intrinsic_layout.addWidget(self.intrinsic_table)
        layout.addWidget(intrinsic_group)

        self.extrinsic_value = EngineeringValue("Stereo R/T")
        metrics_group = QtWidgets.QGroupBox("Calibration metrics")
        metrics_layout = QtWidgets.QVBoxLayout(metrics_group)
        metrics_layout.addWidget(self.extrinsic_value)
        self.calibration_contract = QtWidgets.QLabel()
        self.calibration_contract.setObjectName("compactContract")
        self.calibration_contract.setWordWrap(True)
        self.calibration_contract.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        metrics_layout.addWidget(self.calibration_contract)
        layout.addWidget(metrics_group)

        setup_group = QtWidgets.QGroupBox("Setup")
        setup_layout = QtWidgets.QVBoxLayout(setup_group)
        setup_layout.setContentsMargins(6, 6, 6, 6)
        setup_layout.setSpacing(5)
        setup = _evidence(self.backend).calibration_setup()
        self._initial_calibration_setup = dict(setup)

        def setup_spin(
            key: str,
            minimum: int,
            maximum: int,
            suffix: str = "",
        ) -> QtWidgets.QSpinBox:
            widget = QtWidgets.QSpinBox()
            widget.setRange(minimum, maximum)
            value = setup.get(key)
            default_value = 20 if key == "calculation_pair_count" else minimum
            widget.setValue(int(value if value is not None else default_value))
            widget.setSuffix(suffix)
            if key == "calculation_pair_count":
                widget.setSpecialValueText("all pairs")
            self.setup_widgets[key] = widget
            return widget

        def setup_double(
            key: str,
            minimum: float,
            maximum: float,
            suffix: str = "",
        ) -> QtWidgets.QDoubleSpinBox:
            widget = QtWidgets.QDoubleSpinBox()
            widget.setRange(minimum, maximum)
            widget.setDecimals(3)
            value = setup.get(key)
            widget.setValue(float(value if value is not None else minimum))
            widget.setSuffix(suffix)
            self.setup_widgets[key] = widget
            return widget

        def setup_text(key: str, placeholder: str = "") -> QtWidgets.QLineEdit:
            widget = QtWidgets.QLineEdit(str(setup.get(key) or ""))
            if placeholder:
                widget.setPlaceholderText(placeholder)
            self.setup_widgets[key] = widget
            return widget

        def setup_combo(key: str) -> QtWidgets.QComboBox:
            widget = QtWidgets.QComboBox()
            widget.addItem("Pixhawk attitude + GPS/RTK baseline", "pixhawk")
            widget.addItem("Checkerboard stereo R/T", "checkerboard")
            current = str(setup.get(key) or "pixhawk")
            widget.setCurrentIndex(max(0, widget.findData(current)))
            self.setup_widgets[key] = widget
            return widget

        setup_form = QtWidgets.QGridLayout()
        setup_form.setHorizontalSpacing(8)
        setup_form.setVerticalSpacing(5)
        setup_form.setColumnStretch(1, 1)
        setup_form.setColumnStretch(3, 1)
        for index, (label, widget) in enumerate(
            (
                ("Inner columns", setup_spin("inner_cols", 2, 30)),
                ("Inner rows", setup_spin("inner_rows", 2, 30)),
                ("Square size", setup_double("square_size_mm", 1.0, 500.0, " mm")),
                ("Minimum images", setup_spin("min_images", 3, 100)),
                ("Calculation pairs", setup_spin("calculation_pair_count", 0, 10000, " pair")),
            )
        ):
            row = index // 2
            column = (index % 2) * 2
            setup_form.addWidget(QtWidgets.QLabel(label), row, column)
            setup_form.addWidget(widget, row, column + 1)
        setup_layout.addLayout(setup_form)

        path_form = QtWidgets.QGridLayout()
        path_form.setHorizontalSpacing(8)
        path_form.setVerticalSpacing(5)
        path_form.setColumnStretch(1, 1)
        path_form.setColumnStretch(3, 1)
        path_form.addWidget(QtWidgets.QLabel("Calibration root"), 0, 0)
        path_form.addWidget(setup_text("calibration_root", "calibration_captures"), 0, 1, 1, 3)
        path_form.addWidget(QtWidgets.QLabel("Calculation mode"), 1, 0)
        path_form.addWidget(setup_combo("validation_mode"), 1, 1)
        setup_layout.addLayout(path_form)
        setup_contract = QtWidgets.QLabel(
            "Square: mm · pixels/intrinsics: OpenCV RDF · "
            "X_right = R_left_to_right X_left + T_left_to_right_m · "
            "save zoom comes from the captured camera readback"
        )
        setup_contract.setObjectName("compactContract")
        setup_layout.addWidget(setup_contract)
        setup_actions = QtWidgets.QHBoxLayout()
        for command_id, label in (
            ("calibration.apply_setup", "Apply setup"),
            ("calibration.save_setup", "Save setup"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            setup_actions.addWidget(button)
        setup_actions.addStretch(1)
        setup_layout.addLayout(setup_actions)
        layout.addWidget(setup_group)

        handoff_button = QtWidgets.QPushButton("Advanced Zoom")
        handoff_button.setToolTip(
            "Open Advanced camera controls for physical zoom commands."
        )
        handoff_button.clicked.connect(open_advanced)
        layout.addStretch(1)
        layout.addWidget(handoff_button, alignment=QtCore.Qt.AlignRight)

    def _send_calibration_zoom(self, action: str, level: int | None) -> None:
        sender = getattr(self.backend, "advanced_send_zoom", None)
        if not callable(sender):
            self.zoom_readback_label.setText("Zoom control unavailable")
            return
        try:
            sender("both", action, level)
        except Exception as exc:
            self.zoom_readback_label.setText(f"Zoom command failed: {exc}")
            return
        self.zoom_readback_label.setText(
            f"Requested both={level}×; GZVV readback pending"
            if action == "set"
            else "GZVV readback pending"
        )

    def _build_operation_tabs(self) -> QtWidgets.QTabWidget:
        operation_tabs = QtWidgets.QTabWidget()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        # Two tabs by workflow, not by K,D vs R,t: a fresh checkerboard run
        # always produces both together (R,t needs K,D already computed), so
        # "Capture & Compute" covers both, then saves each as its own file.
        # "Load & Apply" is the separate path for reusing a previously saved
        # profile without recapturing anything. The Setup group's own
        # Apply/Save setup buttons live next to that form, not in a tab here.
        operation_sections = (
            (
                "Capture & Compute",
                (
                    ("calibration.capture_images", "Capture"),
                    ("calibration.capture_images_wiris_ftp", "Capture FTP HR"),
                    ("calibration.capture_images_wiris_ftp_oz", "Capture FTP OZ"),
                    ("calibration.run_intrinsic", "Run calibration (K,D)"),
                    ("calibration.run_extrinsics", "Calculate R,t"),
                    ("calibration.save_intrinsics", "Save Intrinsics"),
                    ("calibration.save_extrinsics", "Save R,t"),
                ),
            ),
            (
                "Load & Apply",
                (
                    ("calibration.load_intrinsics", "Load Intrinsics"),
                    ("calibration.load_extrinsics", "Load R,t"),
                    ("calibration.apply_bundle", "Apply (K,D + R,t)"),
                ),
            ),
            ("Validation", ()),
        )
        for tab_label, actions in operation_sections:
            page = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(6, 6, 6, 6)
            actions_layout = QtWidgets.QGridLayout()
            for index, (command_id, label) in enumerate(actions):
                button = _command_button(label, command_id, self.invoke_command)
                self.operation_buttons[command_id] = button
                actions_layout.addWidget(button, index // 3, index % 3)
            page_layout.addLayout(actions_layout)
            if tab_label == "Capture & Compute":
                auto_group = QtWidgets.QGroupBox("Automatic capture")
                auto_layout = QtWidgets.QGridLayout(auto_group)
                auto_layout.setHorizontalSpacing(8)
                auto_layout.setVerticalSpacing(5)
                auto_layout.setColumnStretch(1, 1)
                auto_layout.setColumnStretch(3, 1)

                auto_interval = QtWidgets.QDoubleSpinBox()
                auto_interval.setRange(0.5, 10.0)
                auto_interval.setSingleStep(0.5)
                auto_interval.setDecimals(1)
                auto_interval.setSuffix(" s")
                auto_interval.setValue(
                    float(self._initial_calibration_setup.get("auto_interval_s") or 1.0)
                )
                self.setup_widgets["auto_interval_s"] = auto_interval
                auto_target = QtWidgets.QSpinBox()
                auto_target.setRange(1, 200)
                auto_target.setSuffix(" pair")
                auto_target.setValue(
                    int(self._initial_calibration_setup.get("auto_target_pairs") or 20)
                )
                self.setup_widgets["auto_target_pairs"] = auto_target
                auto_button = _command_button(
                    "Start auto capture",
                    "calibration.auto_capture",
                    self.invoke_command,
                )
                self.operation_buttons["calibration.auto_capture"] = auto_button
                auto_button.setToolTip(
                    "Capture only new left/right checkerboard detections whose RTCP/NTP timestamps differ by at most 100 ms."
                )
                auto_layout.addWidget(QtWidgets.QLabel("Interval"), 0, 0)
                auto_layout.addWidget(auto_interval, 0, 1)
                auto_layout.addWidget(QtWidgets.QLabel("Target pairs"), 0, 2)
                auto_layout.addWidget(auto_target, 0, 3)
                auto_layout.addWidget(auto_button, 1, 0, 1, 4)
                page_layout.addWidget(auto_group)
            elif tab_label == "Validation":
                page_layout.addWidget(self._build_checkerboard_validation_group())
            page_layout.addStretch(1)
            operation_tabs.addTab(page, tab_label)
        return operation_tabs

    def _build_checkerboard_validation_group(self) -> QtWidgets.QGroupBox:
        """Recompute K,D (and R,t) from random subsets of the scanned pairs
        and report how much the result varies, as a check on calibration
        stability -- distinct from the main Capture & Compute/Load & Apply
        flow, which only ever produces or loads one "official" result.

        This talks to the owner only through the payload-in/result-out
        ``*_workstation_checkerboard_validation_*`` facade methods; it does
        not reach into the legacy panel's own hidden validation widgets.
        """

        group = QtWidgets.QGroupBox("Checkerboard random validation")
        grid = QtWidgets.QGridLayout(group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(5)
        grid.setColumnStretch(1, 1)
        row = 0

        self.checkerboard_validation_root_edit = QtWidgets.QLineEdit()
        self.checkerboard_validation_root_edit.setToolTip(
            "Directory containing left/right checkerboard capture images to validate against."
        )
        browse_root_button = QtWidgets.QPushButton("Browse…")
        browse_root_button.clicked.connect(self._choose_checkerboard_validation_root)
        grid.addWidget(QtWidgets.QLabel("Image root"), row, 0)
        grid.addWidget(self.checkerboard_validation_root_edit, row, 1)
        grid.addWidget(browse_root_button, row, 2)
        row += 1

        self.checkerboard_validation_mode_combo = QtWidgets.QComboBox()
        self.checkerboard_validation_mode_combo.addItem("Pixhawk (K,D only)", "pixhawk")
        self.checkerboard_validation_mode_combo.addItem("Checkerboard (K,D + R,t)", "checkerboard")
        grid.addWidget(QtWidgets.QLabel("Mode"), row, 0)
        grid.addWidget(self.checkerboard_validation_mode_combo, row, 1, 1, 2)
        row += 1

        scan_button = QtWidgets.QPushButton("Scan")
        scan_button.clicked.connect(self._scan_checkerboard_validation)
        self.checkerboard_validation_scan_label = QtWidgets.QLabel("--")
        self.checkerboard_validation_scan_label.setWordWrap(True)
        grid.addWidget(scan_button, row, 0)
        grid.addWidget(self.checkerboard_validation_scan_label, row, 1, 1, 2)
        row += 1

        self.checkerboard_validation_json_edit = QtWidgets.QLineEdit()
        self.checkerboard_validation_json_edit.setToolTip(
            "Existing intrinsics/extrinsics JSON to use as the validation reference, instead of "
            "capturing and computing a fresh result in Capture & Compute first."
        )
        browse_json_button = QtWidgets.QPushButton("Browse…")
        browse_json_button.clicked.connect(self._choose_checkerboard_validation_json)
        grid.addWidget(QtWidgets.QLabel("Reference JSON"), row, 0)
        grid.addWidget(self.checkerboard_validation_json_edit, row, 1)
        grid.addWidget(browse_json_button, row, 2)
        row += 1
        load_json_button = QtWidgets.QPushButton("Load JSON as reference")
        load_json_button.clicked.connect(self._load_checkerboard_validation_json)
        grid.addWidget(load_json_button, row, 1, 1, 2)
        row += 1

        self.checkerboard_validation_random_pair_count_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_random_pair_count_spin.setRange(0, 500)
        self.checkerboard_validation_random_pair_count_spin.setToolTip(
            "Stereo pairs per random subset run; 0 auto-sizes the same way a normal run does."
        )
        self.checkerboard_validation_random_runs_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_random_runs_spin.setRange(1, 50)
        self.checkerboard_validation_random_runs_spin.setValue(5)
        self.checkerboard_validation_seed_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_seed_spin.setRange(0, 999999)
        self.checkerboard_validation_seed_spin.setToolTip("0 uses a non-deterministic seed.")
        grid.addWidget(QtWidgets.QLabel("Subset pairs"), row, 0)
        grid.addWidget(self.checkerboard_validation_random_pair_count_spin, row, 1, 1, 2)
        row += 1
        grid.addWidget(QtWidgets.QLabel("Runs"), row, 0)
        grid.addWidget(self.checkerboard_validation_random_runs_spin, row, 1, 1, 2)
        row += 1
        grid.addWidget(QtWidgets.QLabel("Seed"), row, 0)
        grid.addWidget(self.checkerboard_validation_seed_spin, row, 1, 1, 2)
        row += 1

        self.checkerboard_validation_random_button = QtWidgets.QPushButton("Run random validation")
        self.checkerboard_validation_random_button.setToolTip(
            "Requires a reference result first: load one above, or run a normal capture/compute in "
            "Capture & Compute. Recomputes K,D (and R,t in Checkerboard mode) from several random "
            "subsets of the scanned stereo pairs and reports how much the result varies."
        )
        self.checkerboard_validation_random_button.clicked.connect(self._run_checkerboard_validation)
        grid.addWidget(self.checkerboard_validation_random_button, row, 0, 1, 3)
        row += 1

        self.checkerboard_validation_status_label = QtWidgets.QLabel("계산 결과 없음")
        self.checkerboard_validation_status_label.setWordWrap(True)
        grid.addWidget(self.checkerboard_validation_status_label, row, 0, 1, 3)
        row += 1

        self.checkerboard_validation_results_text = QtWidgets.QPlainTextEdit()
        self.checkerboard_validation_results_text.setReadOnly(True)
        self.checkerboard_validation_results_text.setMaximumHeight(160)
        grid.addWidget(self.checkerboard_validation_results_text, row, 0, 1, 3)
        row += 1

        save_result_button = QtWidgets.QPushButton("Save result")
        save_result_button.clicked.connect(self._save_checkerboard_validation_result)
        apply_result_button = QtWidgets.QPushButton("Apply result")
        apply_result_button.clicked.connect(self._apply_checkerboard_validation_result)
        grid.addWidget(save_result_button, row, 0)
        grid.addWidget(apply_result_button, row, 1)

        self._checkerboard_validation_poll_timer = QtCore.QTimer(self)
        self._checkerboard_validation_poll_timer.setInterval(700)
        self._checkerboard_validation_poll_timer.timeout.connect(self._poll_checkerboard_validation)
        return group

    def _choose_checkerboard_validation_root(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Select checkerboard image root",
            self.checkerboard_validation_root_edit.text(),
        )
        if directory:
            self.checkerboard_validation_root_edit.setText(directory)

    def _choose_checkerboard_validation_json(self) -> None:
        path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select reference intrinsics JSON",
            "",
            "Intrinsics JSON (*.json);;All files (*)",
        )
        if path:
            self.checkerboard_validation_json_edit.setText(path)

    def _checkerboard_validation_payload(self) -> dict[str, object]:
        return {
            "root": self.checkerboard_validation_root_edit.text().strip(),
            "mode": str(self.checkerboard_validation_mode_combo.currentData() or "pixhawk"),
            "json_path": self.checkerboard_validation_json_edit.text().strip(),
            "random_pair_count": self.checkerboard_validation_random_pair_count_spin.value(),
            "runs": self.checkerboard_validation_random_runs_spin.value(),
            "seed": self.checkerboard_validation_seed_spin.value(),
        }

    def _apply_checkerboard_validation_snapshot(self, snapshot: object) -> None:
        if not isinstance(snapshot, Mapping):
            return
        self.checkerboard_validation_scan_label.setText(str(snapshot.get("scan_status") or "--"))
        self.checkerboard_validation_status_label.setText(str(snapshot.get("status") or "계산 결과 없음"))
        lines: list[str] = []
        summary = snapshot.get("summary")
        if isinstance(summary, Mapping) and summary:
            lines.append(f"summary: {summary}")
        run_results = snapshot.get("run_results")
        if isinstance(run_results, (list, tuple)):
            lines.extend(str(row) for row in run_results if isinstance(row, Mapping))
        self.checkerboard_validation_results_text.setPlainText("\n".join(lines))
        running = bool(snapshot.get("running"))
        self.checkerboard_validation_random_button.setEnabled(not running)
        if running:
            if not self._checkerboard_validation_poll_timer.isActive():
                self._checkerboard_validation_poll_timer.start()
        else:
            self._checkerboard_validation_poll_timer.stop()

    def _scan_checkerboard_validation(self) -> None:
        method = getattr(self.backend, "configure_workstation_checkerboard_validation", None)
        if not callable(method):
            return
        payload = {
            "root": self.checkerboard_validation_root_edit.text().strip(),
            "mode": str(self.checkerboard_validation_mode_combo.currentData() or "pixhawk"),
        }
        self._apply_checkerboard_validation_snapshot(method(payload))

    def _load_checkerboard_validation_json(self) -> None:
        method = getattr(self.backend, "load_workstation_checkerboard_validation_json", None)
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(method(self._checkerboard_validation_payload()))

    def _run_checkerboard_validation(self) -> None:
        method = getattr(self.backend, "run_workstation_checkerboard_validation", None)
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(method(self._checkerboard_validation_payload()))

    def _poll_checkerboard_validation(self) -> None:
        method = getattr(self.backend, "workstation_checkerboard_validation_snapshot", None)
        if not callable(method):
            self._checkerboard_validation_poll_timer.stop()
            return
        self._apply_checkerboard_validation_snapshot(method())

    def _save_checkerboard_validation_result(self) -> None:
        method = getattr(self.backend, "save_workstation_checkerboard_validation_result", None)
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(method())

    def _apply_checkerboard_validation_result(self) -> None:
        method = getattr(self.backend, "apply_workstation_checkerboard_validation_result", None)
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(method())

    def _connect_camera_panes(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        clear_signal = getattr(panel, "camera_preview_cleared", None)
        connect = getattr(clear_signal, "connect", None)
        if callable(connect):
            connect(self._clear_camera_previews)
        for side in ("left", "right"):
            pane = getattr(panel, f"{side}_pane", None)
            frame_signal = getattr(pane, "frame_changed", None)
            connect = getattr(frame_signal, "connect", None)
            if callable(connect):
                connect(
                    lambda _label, image, metadata, side=side: self._on_camera_frame(
                        side,
                        image,
                        metadata,
                    )
                )
            detection_signal = getattr(pane, "detection_changed", None)
            connect = getattr(detection_signal, "connect", None)
            if callable(connect):
                connect(
                    lambda _label, detection, side=side: self.previews[side].set_detection(detection)
                )
            stats_signal = getattr(pane, "stats_changed", None)
            connect = getattr(stats_signal, "connect", None)
            if callable(connect):
                connect(
                    lambda _label, stats, side=side: self.previews[side].set_metadata(stats)
                )

    def _clear_camera_previews(self) -> None:
        self._checkerboard_overlay_active = {"left": False, "right": False}
        self._camera_preview_metadata = {"left": {}, "right": {}}
        for preview in self.previews.values():
            preview.clear()

    def _connect_calibration_preview_signal(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        signal = getattr(panel, "calibration_preview_changed", None)
        connect = getattr(signal, "connect", None)
        if callable(connect):
            connect(self._on_calibration_preview_changed)

    def _on_camera_frame(self, side: str, image: QtGui.QImage, metadata: object) -> None:
        self._camera_preview_metadata[side] = metadata
        if not self._checkerboard_overlay_active[side]:
            self.previews[side].set_frame(image, metadata)

    def _on_calibration_preview_changed(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        camera = str(payload.get("camera") or "")
        image = payload.get("image")
        if camera not in self.previews or not isinstance(image, QtGui.QImage) or image.isNull():
            return
        kind = str(payload.get("kind") or "raw")
        metadata = payload.get("metadata", self._camera_preview_metadata[camera])
        if kind == "overlay":
            self._checkerboard_overlay_active[camera] = True
            self.previews[camera].set_frame(image, metadata)
        elif not self._checkerboard_overlay_active[camera]:
            self.previews[camera].set_frame(image, metadata)

    def _request_checkerboard_preview(self) -> None:
        if not self._checkerboard_preview_supported:
            return
        self._checkerboard_overlay_active = {"left": False, "right": False}
        refresh = getattr(self.backend, "advanced_refresh_calibration_preview", None)
        if not callable(refresh):
            return
        try:
            refresh()
        except Exception as exc:
            for preview in self.previews.values():
                preview.meta.setToolTip(f"Checkerboard preview unavailable: {exc}")

    def _start_checkerboard_preview(self) -> None:
        if not self._checkerboard_preview_supported:
            return
        if not self._checkerboard_preview_timer.isActive():
            self._checkerboard_preview_timer.start()
        self._request_checkerboard_preview()

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802 - Qt override.
        super().showEvent(event)
        self._start_checkerboard_preview()

    def hideEvent(self, event: QtGui.QHideEvent) -> None:  # noqa: N802 - Qt override.
        self._checkerboard_preview_timer.stop()
        super().hideEvent(event)

    def setup_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {}
        for key, widget in self.setup_widgets.items():
            if isinstance(widget, QtWidgets.QComboBox):
                payload[key] = widget.currentData()
            elif isinstance(widget, QtWidgets.QLineEdit):
                payload[key] = widget.text().strip()
            elif isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
                payload[key] = widget.value()
        return payload

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        camera = snapshot.get("camera.left")
        self.camera_buttons["camera.toggle"].setText(
            "Stop preview" if camera and camera.state in {"LIVE", "STARTING"} else "Preview"
        )
        panel = getattr(self.backend, "panel", self.backend)
        calibration_controller = getattr(panel, "_calibration_workflow_controller_owner", None)
        auto_capture_active = bool(getattr(calibration_controller, "_auto_capture_active", False))
        auto_capture_button = self.operation_buttons.get("calibration.auto_capture")
        if auto_capture_button is not None:
            auto_capture_button.setText("Stop auto capture" if auto_capture_active else "Start auto capture")
        recording = snapshot.get("recording")
        recording_locked = bool(recording and recording.state in {"ACTIVE", "FINALIZING"})
        self.camera_buttons["camera.toggle"].setEnabled(not recording_locked)
        data = _evidence(self.backend).calibration()
        self.zoom_readback_label.setText(
            f"capture zoom={data.get('zoom', '--')} | "
            f"actual={data.get('zoom_readback', 'left=unknown, right=unknown')} | "
            "capture metadata is required"
        )
        for key, value in self.profile_values.items():
            value.set_value(data.get(key), source=str(data.get("source", "")))
        intrinsic_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            camera = data.get(side, {})
            if not isinstance(camera, dict):
                camera = {}
            grid = "--"
            if camera.get("width_px") is not None and camera.get("height_px") is not None:
                grid = f"{camera['width_px']}×{camera['height_px']}"
            intrinsic_rows.append(
                (
                    side.upper(),
                    grid,
                    _display_number(camera.get("fx_px")),
                    _display_number(camera.get("fy_px")),
                    _display_number(camera.get("cx_px")),
                    _display_number(camera.get("cy_px")),
                    _display_number(camera.get("reprojection_error_px")),
                    camera.get("image_count", "--"),
                    camera.get("frame_contract", CAMERA_FRAME),
                ),
            )
        _sync_table_rows(self.intrinsic_model, tuple(intrinsic_rows))
        extrinsics = data.get("extrinsics", {})
        if not isinstance(extrinsics, dict):
            extrinsics = {}
        translation = extrinsics.get("translation_left_to_right_m")
        translation_text = "--" if not translation else "[" + ", ".join(_display_number(value, digits=6) for value in translation) + "] m"
        rotation = extrinsics.get("rotation_left_to_right")
        rotation_text = (
            "--"
            if not rotation
            else "[" + "; ".join(
                ", ".join(_display_number(value, digits=6) for value in row)
                for row in rotation
            ) + "]"
        )
        right_origin = extrinsics.get("right_camera_origin_in_left_rdf_m")
        right_origin_text = "--" if not right_origin else "[" + ", ".join(_display_number(value, digits=6) for value in right_origin) + "] m"
        self.extrinsic_value.set_value(
            f"{extrinsics.get('state', 'NO DATA')} | R_left_to_right={rotation_text} | "
            f"T_left_to_right={translation_text} | "
            f"C_right_in_left=-RᵀT={right_origin_text} | "
            f"baseline={_display_number(extrinsics.get('baseline_m'))} m",
            source=str(data.get("source", "")),
            frame=str(extrinsics.get("frame_contract", CAMERA_FRAME)),
        )
        self.calibration_contract.setText(
            f"Pixels/intrinsics: {CAMERA_FRAME} · "
            "X_right = R_left_to_right X_left + T_left_to_right_m · "
            f"output: {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]"
        )


class AdvancedWorkspace(QtWidgets.QWidget):
    """Direct Workstation surface for the remaining owner-controlled tools.

    The controls below are an adapter surface, not a second device stack. The
    backend calls the existing panel owner for every network, telemetry,
    preview-worker, and session operation. Read-only tables preserve producer
    labels and saved time/coordinate fields.
    """

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.invoke_command = invoke_command
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        self._media_signature: tuple[object, ...] = ()
        self._media_video_signature: tuple[tuple[str, str], ...] = ()
        self._media_records: dict[str, dict[str, object]] = {}
        self._preview_images: dict[str, QtGui.QImage] = {}
        self._last_media_scan_monotonic_s = float("-inf")

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Advanced",
            )
        )

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_wiris_tab(), "Camera control")
        self.tabs.addTab(self._build_px4_and_watch_tab(), "Pixhawk tools")
        self.tabs.addTab(self._build_preview_tab(), "Preview")
        self.tabs.addTab(self._build_media_tab(), "Files")
        layout.addWidget(self.tabs, 1)

        self._connect_preview_signal()
        self._connect_wiris_response_signal()
        self._refresh_custom_watch_selector()
        self.refresh_media()

    def _button(self, text: str, command_id: str) -> QtWidgets.QPushButton:
        button = _command_button(text, command_id, self.invoke_command)
        self.operation_buttons[command_id] = button
        return button

    @staticmethod
    def _combo(items: tuple[tuple[str, str], ...]) -> QtWidgets.QComboBox:
        combo = QtWidgets.QComboBox()
        for label, value in items:
            combo.addItem(label, value)
        return combo

    def _build_wiris_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        command_group = QtWidgets.QGroupBox("WIRIS SDK command")
        command_form = QtWidgets.QGridLayout(command_group)
        command_form.setColumnStretch(1, 1)
        command_form.setColumnStretch(2, 1)
        self.wiris_target_combo = self._combo(
            (("Both cameras", "both"), ("Left camera", "left"), ("Right camera", "right"))
        )
        self.wiris_catalog_combo = QtWidgets.QComboBox()
        self.wiris_command_edit = QtWidgets.QLineEdit()
        self.wiris_command_edit.setPlaceholderText("SDK command, e.g. HIWS")
        self.wiris_terminator_combo = self._combo((("LINE", "\n"), ("NULL", "\0")))
        self.wiris_send_button = self._button("Send", "advanced.wiris.send")
        self.wiris_status_label = QtWidgets.QLabel("")
        self.wiris_status_label.setObjectName("statusStrip")
        self.wiris_status_label.setWordWrap(True)
        command_form.addWidget(QtWidgets.QLabel("Target"), 0, 0)
        command_form.addWidget(self.wiris_target_combo, 0, 1)
        command_form.addWidget(QtWidgets.QLabel("Catalog"), 1, 0)
        command_form.addWidget(self.wiris_catalog_combo, 1, 1, 1, 2)
        command_form.addWidget(QtWidgets.QLabel("Command"), 2, 0)
        command_form.addWidget(self.wiris_command_edit, 2, 1, 1, 2)
        command_form.addWidget(QtWidgets.QLabel("Terminator"), 3, 0)
        command_form.addWidget(self.wiris_terminator_combo, 3, 1)
        command_form.addWidget(self.wiris_send_button, 3, 2)
        command_form.addWidget(self.wiris_status_label, 4, 0, 1, 3)
        layout.addWidget(command_group)

        response_group = QtWidgets.QGroupBox("Command responses")
        response_layout = QtWidgets.QVBoxLayout(response_group)
        self.wiris_response_log = QtWidgets.QPlainTextEdit()
        self.wiris_response_log.setReadOnly(True)
        self.wiris_response_log.setMaximumBlockCount(400)
        self.wiris_response_log.setMinimumHeight(150)
        self.wiris_response_log.setPlaceholderText(
            "WIRIS SDK responses from the selected camera target appear here."
        )
        response_actions = QtWidgets.QHBoxLayout()
        response_actions.addStretch(1)
        self.wiris_response_clear_button = QtWidgets.QPushButton("Clear responses")
        self.wiris_response_clear_button.clicked.connect(self.wiris_response_log.clear)
        response_actions.addWidget(self.wiris_response_clear_button)
        response_layout.addWidget(self.wiris_response_log)
        response_layout.addLayout(response_actions)
        layout.addWidget(response_group, 1)

        self.wiris_catalog_combo.currentIndexChanged.connect(self._apply_wiris_catalog_item)
        self._load_wiris_catalog()

        zoom_group = QtWidgets.QGroupBox("Zoom")
        zoom_layout = QtWidgets.QGridLayout(zoom_group)
        self.zoom_target_combo = self._combo(
            (("Both cameras", "both"), ("Left camera", "left"), ("Right camera", "right"))
        )
        self.zoom_level_spin = QtWidgets.QSpinBox()
        self.zoom_level_spin.setRange(1, 100)
        self.zoom_level_spin.setValue(10)
        self.zoom_buttons = {
            "in": self._button("In", "advanced.zoom.in"),
            "out": self._button("Out", "advanced.zoom.out"),
            "stop": self._button("Stop", "advanced.zoom.stop"),
            "set": self._button("Set", "advanced.zoom.set"),
        }
        self.zoom_status_label = QtWidgets.QLabel("")
        self.zoom_status_label.setObjectName("statusStrip")
        self.zoom_status_label.setWordWrap(True)
        zoom_layout.addWidget(QtWidgets.QLabel("Target"), 0, 0)
        zoom_layout.addWidget(self.zoom_target_combo, 0, 1)
        zoom_layout.addWidget(QtWidgets.QLabel("Level"), 0, 2)
        zoom_layout.addWidget(self.zoom_level_spin, 0, 3)
        zoom_layout.addWidget(self.zoom_buttons["set"], 0, 4)
        zoom_layout.addWidget(self.zoom_buttons["in"], 1, 0, 1, 2)
        zoom_layout.addWidget(self.zoom_buttons["out"], 1, 2, 1, 2)
        zoom_layout.addWidget(self.zoom_buttons["stop"], 1, 4)
        zoom_layout.addWidget(self.zoom_status_label, 2, 0, 1, 5)
        layout.addWidget(zoom_group)
        return page

    def _connect_wiris_response_signal(self) -> None:
        panel = getattr(self.backend, "panel", None)
        signal = getattr(panel, "wiris_command_response", None)
        connect = getattr(signal, "connect", None)
        if callable(connect):
            connect(self._append_wiris_command_response)

    def _append_wiris_command_response(self, payload: object) -> None:
        data = dict(payload or {}) if isinstance(payload, dict) else {}
        status_text = str(data.get("status_text") or "")
        if status_text:
            self.wiris_status_label.setText(status_text)

        phase = str(data.get("phase") or "completed")
        if phase == "sending":
            return

        timestamp = QtCore.QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        target = str(data.get("target") or "unknown")
        command = str(data.get("command") or "<unknown>")
        lines = [f"[{timestamp}] target={target} command={command}"]
        command_logs = {
            str(key): str(value).strip()
            for key, value in dict(data.get("command_logs") or {}).items()
        }
        statuses = {
            str(key): str(value)
            for key, value in dict(data.get("statuses") or {}).items()
        }
        target_infos = tuple(data.get("target_infos") or ())
        ordered_labels = [str(info[0]) for info in target_infos if isinstance(info, (tuple, list)) and info]
        for label in ordered_labels:
            if label in command_logs:
                lines.append(command_logs[label])
            elif label in statuses:
                lines.append(f"{label}: {statuses[label]}")
        for label, log_text in command_logs.items():
            if label not in ordered_labels:
                lines.append(log_text)
        for label, response in statuses.items():
            if label not in ordered_labels and label not in command_logs:
                lines.append(f"{label}: {response}")
        if len(lines) == 1 and status_text:
            lines.append(status_text)
        self.wiris_response_log.appendPlainText("\n".join(lines) + "\n")
        cursor = self.wiris_response_log.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        self.wiris_response_log.setTextCursor(cursor)

    def _load_wiris_catalog(self) -> None:
        fallback = (
            {"category": "Basic", "label": "Check connection", "command": "HIWS", "terminator": "\n", "description": "Connection check."},
            {"category": "Zoom", "label": "Zoom in", "command": "SZIN", "terminator": "\n", "description": "Zoom in."},
            {"category": "Zoom", "label": "Zoom out", "command": "SZOT", "terminator": "\n", "description": "Zoom out."},
            {"category": "System", "label": "Reboot", "command": "REBT", "terminator": "\n", "description": "Reboot camera; confirmation is required."},
        )
        catalog: object = fallback
        method = getattr(self.backend, "advanced_wiris_catalog", None)
        if callable(method):
            try:
                catalog = method()
            except Exception as exc:
                self.wiris_status_label.setText(f"Catalog unavailable: {exc}")
        self.wiris_catalog_combo.clear()
        self._wiris_catalog_records: list[dict[str, str]] = []
        for record in catalog if isinstance(catalog, (tuple, list)) else ():
            if not isinstance(record, dict):
                continue
            command = str(record.get("command") or "")
            label = str(record.get("label") or command or "Custom")
            if not command:
                label = f"{label} (manual)"
            normalized = {
                "category": str(record.get("category") or ""),
                "label": label,
                "command": command,
                "terminator": str(record.get("terminator") or "\n"),
                "description": str(record.get("description") or ""),
            }
            self._wiris_catalog_records.append(normalized)
            display = f"{normalized['category']} / {normalized['label']}" if normalized["category"] else normalized["label"]
            self.wiris_catalog_combo.addItem(display, len(self._wiris_catalog_records) - 1)
        if self.wiris_catalog_combo.count():
            self.wiris_catalog_combo.setCurrentIndex(0)

    def _apply_wiris_catalog_item(self, index: int) -> None:
        if index < 0 or index >= len(getattr(self, "_wiris_catalog_records", ())):
            return
        record = self._wiris_catalog_records[index]
        if record["command"]:
            self.wiris_command_edit.setText(record["command"])
        terminator_index = self.wiris_terminator_combo.findData(record["terminator"])
        if terminator_index >= 0:
            self.wiris_terminator_combo.setCurrentIndex(terminator_index)
        self.wiris_status_label.setText(record["description"] or "Custom SDK command")
        self.wiris_status_label.setToolTip(record["description"])

    def _build_px4_and_watch_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        px4_group = QtWidgets.QGroupBox("PX4 tools")
        px4_layout = QtWidgets.QGridLayout(px4_group)
        self.px4_camera_combo = self._combo((("Pixhawk 1 / Left", "left"), ("Pixhawk 2 / Right", "right")))
        # Historical attribute name retained for compatibility with the
        # backend facade; values are now PX4 semantic priority combos.
        self.px4_priority_spins: dict[str, QtWidgets.QComboBox] = {}
        for row, role in enumerate(("BOARD", "CAN1", "CAN2", "GPS1_MAG", "GPS2_MAG"), start=1):
            spin = _build_px4_priority_combo(
                {"BOARD": 0, "CAN1": 100, "CAN2": 90, "GPS1_MAG": 0, "GPS2_MAG": 0}[role]
            )
            self.px4_priority_spins[role] = spin
            px4_layout.addWidget(QtWidgets.QLabel(role), row, 2)
            px4_layout.addWidget(spin, row, 3)
        px4_layout.addWidget(QtWidgets.QLabel("Side"), 0, 0)
        px4_layout.addWidget(self.px4_camera_combo, 0, 1)
        px4_layout.addWidget(self._button("Refresh MAG", "advanced.px4.refresh_mag"), 0, 4)
        px4_layout.addWidget(self._button("Refresh attitude", "advanced.px4.refresh_attitude"), 1, 0, 1, 2)
        px4_layout.addWidget(self._button("Shell", "advanced.px4.shell"), 1, 4)
        px4_layout.addWidget(self._button("Apply MAG", "advanced.px4.apply_mag"), 2, 0, 1, 2)
        px4_layout.addWidget(self._button("Save MAG", "advanced.px4.save_mag"), 3, 0, 1, 2)
        px4_layout.addWidget(self._button("Reboot", "advanced.px4.reboot"), 4, 0, 1, 2)
        self.px4_status_label = QtWidgets.QLabel(
            "MAG: disarmed + PARAM readback · attitude calibration: QGC Sensor Setup"
        )
        self.px4_status_label.setObjectName("statusStrip")
        self.px4_status_label.setWordWrap(True)
        px4_layout.addWidget(self.px4_status_label, 6, 0, 1, 5)
        layout.addWidget(px4_group)

        watch_group = QtWidgets.QGroupBox("Field watch")
        watch_layout = QtWidgets.QGridLayout(watch_group)
        watch_layout.setColumnStretch(1, 1)
        self.watch_name_edit = QtWidgets.QLineEdit("local_z")
        self.watch_message_edit = QtWidgets.QLineEdit("ATTITUDE")
        self.watch_field_edit = QtWidgets.QLineEdit("z")
        self.watch_remove_combo = QtWidgets.QComboBox()
        watch_layout.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        watch_layout.addWidget(self.watch_name_edit, 0, 1)
        watch_layout.addWidget(QtWidgets.QLabel("Message"), 1, 0)
        watch_layout.addWidget(self.watch_message_edit, 1, 1)
        watch_layout.addWidget(QtWidgets.QLabel("Field"), 2, 0)
        watch_layout.addWidget(self.watch_field_edit, 2, 1)
        watch_layout.addWidget(self._button("Add", "advanced.watch.add_mavlink"), 0, 2)
        watch_layout.addWidget(self._button("Find fields", "advanced.watch.discover_mavlink"), 2, 2)
        watch_layout.addWidget(QtWidgets.QLabel("Remove"), 3, 0)
        watch_layout.addWidget(self.watch_remove_combo, 3, 1)
        watch_layout.addWidget(self._button("Remove", "advanced.watch.remove"), 3, 2)
        self.watch_status_label = QtWidgets.QLabel("")
        self.watch_status_label.setObjectName("statusStrip")
        self.watch_status_label.setWordWrap(True)
        watch_layout.addWidget(self.watch_status_label, 4, 0, 1, 3)
        layout.addWidget(watch_group)

        baseline_group = QtWidgets.QGroupBox("Message fields")
        baseline_layout = QtWidgets.QVBoxLayout(baseline_group)
        baseline_actions = QtWidgets.QHBoxLayout()
        baseline_actions.addWidget(self._button("Refresh", "advanced.baseline.scan"))
        baseline_actions.addStretch(1)
        baseline_layout.addLayout(baseline_actions)
        self.baseline_mavlink_model = _table_model(self, ("Message", "Fields"))
        self.baseline_mavlink_table = QtWidgets.QTableView()
        self.baseline_mavlink_table.setModel(self.baseline_mavlink_model)
        self.baseline_mavlink_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.baseline_mavlink_table.verticalHeader().setVisible(False)
        self.baseline_mavlink_table.setMaximumHeight(100)
        self.baseline_mavlink_table.horizontalHeader().setStretchLastSection(True)
        baseline_layout.addWidget(self.baseline_mavlink_table)
        self.baseline_status_label = QtWidgets.QLabel("")
        self.baseline_status_label.setObjectName("statusStrip")
        self.baseline_status_label.setWordWrap(True)
        baseline_layout.addWidget(self.baseline_status_label)
        layout.addWidget(baseline_group, 1)
        return page

    def _build_preview_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(self._button("Refresh", "advanced.preview.refresh"))
        controls.addStretch(1)
        layout.addLayout(controls)
        row = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.preview_labels: dict[str, QtWidgets.QLabel] = {}
        for side in ("left", "right"):
            label = QtWidgets.QLabel(f"{side.upper()} preview waiting")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setMinimumSize(320, 180)
            label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
            label.setObjectName("calibrationPreview")
            self.preview_labels[side] = label
            row.addWidget(label)
        row.setStretchFactor(0, 1)
        row.setStretchFactor(1, 1)
        layout.addWidget(row, 1)
        self.preview_status_label = QtWidgets.QLabel("Pixels: OpenCV RDF")
        self.preview_status_label.setObjectName("statusStrip")
        self.preview_status_label.setWordWrap(True)
        layout.addWidget(self.preview_status_label)
        return page

    def _connect_preview_signal(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        signal = getattr(panel, "calibration_preview_changed", None)
        if signal is not None:
            signal.connect(self._on_preview_changed)

    def _on_preview_changed(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        camera = str(payload.get("camera") or "")
        image = payload.get("image")
        if camera not in self.preview_labels or not isinstance(image, QtGui.QImage) or image.isNull():
            return
        self._set_preview_image(camera, image)

    def _set_preview_image(self, camera: str, image: QtGui.QImage) -> None:
        self._preview_images[camera] = image.copy()
        label = self.preview_labels[camera]
        label.setPixmap(
            QtGui.QPixmap.fromImage(image).scaled(
                label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )
        )
        label.setText("")

    def apply_preview_snapshot(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        images = payload.get("images", {})
        if isinstance(images, dict):
            for camera, image in images.items():
                if str(camera) in self.preview_labels and isinstance(image, QtGui.QImage) and not image.isNull():
                    self._set_preview_image(str(camera), image)
        preview_detail = str(payload.get("axis_contract") or "Preview snapshot updated.")
        self.preview_status_label.setText(_compact_text(preview_detail))
        self.preview_status_label.setToolTip(preview_detail)

    def _build_media_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)
        self.media_session_combo = QtWidgets.QComboBox()
        self.media_artifact_combo = QtWidgets.QComboBox()
        self.media_video_combo = QtWidgets.QComboBox()
        selector = QtWidgets.QGridLayout()
        selector.setColumnStretch(1, 1)
        selector.addWidget(QtWidgets.QLabel("Session"), 0, 0)
        selector.addWidget(self.media_session_combo, 0, 1)
        selector.addWidget(self._button("Refresh", "advanced.media.refresh"), 0, 2)
        selector.addWidget(self._button("Open folder", "advanced.media.open_session"), 0, 3)
        selector.addWidget(QtWidgets.QLabel("Artifact"), 1, 0)
        selector.addWidget(self.media_artifact_combo, 1, 1)
        selector.addWidget(self._button("Preview", "advanced.media.preview"), 1, 2)
        selector.addWidget(self._button("Open file", "advanced.media.open_artifact"), 1, 3)
        selector.addWidget(QtWidgets.QLabel("Video"), 2, 0)
        selector.addWidget(self.media_video_combo, 2, 1)
        selector.addWidget(self._button("Play", "advanced.media.open_video"), 2, 2, 1, 2)
        layout.addLayout(selector)
        self.media_status_label = QtWidgets.QLabel("Read-only")
        self.media_status_label.setToolTip("Saved fields and timestamps are unchanged.")
        self.media_status_label.setObjectName("statusStrip")
        self.media_status_label.setWordWrap(True)
        layout.addWidget(self.media_status_label)
        self.media_model = QtGui.QStandardItemModel(0, 0, self)
        self.media_table = QtWidgets.QTableView()
        self.media_table.setModel(self.media_model)
        self.media_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.media_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.media_table.verticalHeader().setVisible(False)
        self.media_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.media_table, 1)
        actions = QtWidgets.QHBoxLayout()
        actions.addWidget(self._button("Save", "session.save"))
        actions.addStretch(1)
        layout.addLayout(actions)
        self.media_session_combo.currentIndexChanged.connect(lambda _index: self._refresh_media_artifacts())
        self.media_artifact_combo.currentIndexChanged.connect(lambda _index: self._preview_media_artifact())
        return page

    def _refresh_custom_watch_selector(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        specs = getattr(panel, "custom_watch_specs", ()) or ()
        keys = tuple(str(spec.get("key", "")) for spec in specs if isinstance(spec, dict) and spec.get("key"))
        current = str(self.watch_remove_combo.currentData() or "") if hasattr(self, "watch_remove_combo") else ""
        if not hasattr(self, "watch_remove_combo"):
            return
        if tuple(self.watch_remove_combo.itemData(index) for index in range(self.watch_remove_combo.count())) == keys:
            return
        self.watch_remove_combo.clear()
        for key in keys:
            self.watch_remove_combo.addItem(key, key)
        if current:
            index = self.watch_remove_combo.findData(current)
            if index >= 0:
                self.watch_remove_combo.setCurrentIndex(index)

    def _selected_px4_side(self) -> str:
        return str(self.px4_camera_combo.currentData() or "left")

    def px4_priority_payload(self) -> dict[str, int]:
        return {role: _px4_priority_combo_value(spin) for role, spin in self.px4_priority_spins.items()}

    def apply_baseline_snapshot(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        self.baseline_mavlink_model.removeRows(0, self.baseline_mavlink_model.rowCount())
        for row in payload.get("mavlink_messages", ()) or ():
            if isinstance(row, (tuple, list)):
                _append_table_row(self.baseline_mavlink_model, tuple(row))
        self.baseline_status_label.setText(str(payload.get("axis_contract") or "Baseline fields updated."))

    def _selected_media_record(self) -> dict[str, object] | None:
        session_id = str(self.media_session_combo.currentData() or "")
        return self._media_records.get(session_id)

    def _selected_media_artifact_path(self) -> Path | None:
        record = self._selected_media_record()
        if record is None:
            return None
        output_dir = Path(str(record.get("path") or ""))
        artifact = str(self.media_artifact_combo.currentData() or "")
        if not artifact:
            return output_dir if output_dir.exists() else None
        candidate = output_dir / Path(artifact).name
        if candidate.exists():
            return candidate
        published = output_dir / "published" / Path(artifact).name
        return published if published.exists() else candidate

    def _selected_media_video_path(self) -> Path | None:
        value = str(self.media_video_combo.currentData() or "")
        if not value:
            return None
        return Path(value)

    def _open_path(self, path: Path | None, *, label: str) -> bool:
        if path is None or not path.exists():
            self.media_status_label.setText(f"{label}: path unavailable")
            return False
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))
        self.media_status_label.setText(f"Opened with system association: {path}")
        return True

    def refresh_media(self, *, force: bool = True) -> None:
        now_s = time.monotonic()
        if (
            not force
            and now_s - self._last_media_scan_monotonic_s
            < SESSION_CATALOG_REFRESH_INTERVAL_S
        ):
            self._refresh_media_video_paths()
            return
        self._last_media_scan_monotonic_s = now_s
        records = _evidence(self.backend).session_catalog()
        signature = tuple(
            (
                record.get("session_id"),
                record.get("status"),
                record.get("error"),
                tuple(record.get("artifacts", ()) or ()),
            )
            for record in records
        )
        changed = signature != self._media_signature or not self.media_session_combo.count()
        if changed:
            current = str(self.media_session_combo.currentData() or "")
            self._media_records = {
                str(record.get("session_id")): dict(record)
                for record in records
                if record.get("session_id")
            }
            self._media_signature = signature
            self.media_session_combo.blockSignals(True)
            try:
                self.media_session_combo.clear()
                for record in records:
                    session_id = str(record.get("session_id") or "")
                    self.media_session_combo.addItem(
                        f"{session_id} [{record.get('status', 'unknown')}]", session_id
                    )
                if current:
                    index = self.media_session_combo.findData(current)
                    if index >= 0:
                        self.media_session_combo.setCurrentIndex(index)
            finally:
                self.media_session_combo.blockSignals(False)
        if changed or force:
            self._refresh_media_artifacts()
        self._refresh_media_video_paths()

    def _refresh_media_video_paths(self) -> None:
        recording = _evidence(self.backend).recording()
        paths = recording.get("video_paths", {})
        video_signature = (
            tuple((str(side), str(path)) for side, path in sorted(paths.items()))
            if isinstance(paths, dict)
            else ()
        )
        if video_signature == self._media_video_signature:
            return
        self._media_video_signature = video_signature
        current = str(self.media_video_combo.currentData() or "")
        self.media_video_combo.blockSignals(True)
        try:
            self.media_video_combo.clear()
            for side, path in video_signature:
                self.media_video_combo.addItem(f"{side}: {path}", path)
            if current:
                index = self.media_video_combo.findData(current)
                if index >= 0:
                    self.media_video_combo.setCurrentIndex(index)
        finally:
            self.media_video_combo.blockSignals(False)

    def _refresh_media_artifacts(self) -> None:
        record = self._selected_media_record()
        current = str(self.media_artifact_combo.currentData() or "")
        self.media_artifact_combo.blockSignals(True)
        try:
            self.media_artifact_combo.clear()
            if record is not None:
                for artifact in record.get("artifacts", ()) or ():
                    self.media_artifact_combo.addItem(str(artifact), str(artifact))
            if current:
                index = self.media_artifact_combo.findData(current)
                if index >= 0:
                    self.media_artifact_combo.setCurrentIndex(index)
        finally:
            self.media_artifact_combo.blockSignals(False)
        self._preview_media_artifact()

    def _preview_media_artifact(self) -> None:
        record = self._selected_media_record()
        artifact = str(self.media_artifact_combo.currentData() or "")
        if record is None or not artifact:
            self.media_model.clear()
            return
        data = _evidence(self.backend).session_artifact_rows(str(record.get("session_id")), artifact)
        error = str(data.get("error") or "")
        if error:
            self.media_model.clear()
            self.media_status_label.setText(f"{artifact}: {error}")
            return
        columns = tuple(str(value) for value in data.get("columns", ()) or ())
        self.media_model.clear()
        self.media_model.setColumnCount(len(columns))
        self.media_model.setHorizontalHeaderLabels(list(columns))
        for row in data.get("rows", ()) or ():
            if isinstance(row, dict):
                _append_table_row(self.media_model, tuple(row.get(column, "") for column in columns))
        media_detail = (
            f"{artifact}: {len(data.get('rows', ()) or ())} rows · "
            f"{data.get('time_axis', 'stored time axis')} · fields unchanged"
        )
        self.media_status_label.setText(f"{artifact}: {len(data.get('rows', ()) or ())} rows")
        self.media_status_label.setToolTip(
            f"{media_detail}\n{data.get('coordinate_contract', 'stored fields preserved')}"
        )

    def media_open_session(self) -> bool:
        record = self._selected_media_record()
        return self._open_path(None if record is None else Path(str(record.get("path") or "")), label="Session")

    def media_open_artifact(self) -> bool:
        return self._open_path(self._selected_media_artifact_path(), label="Artifact")

    def media_open_video(self) -> bool:
        return self._open_path(self._selected_media_video_path(), label="Video")

    def media_preview(self) -> bool:
        self._preview_media_artifact()
        return True

    def refresh(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        self._refresh_custom_watch_selector()
        self.refresh_media(force=False)


class RecordingWorkspace(QtWidgets.QWidget):
    """Session/review/export evidence without a second recording owner."""

    hr_command_requested = QtCore.pyqtSignal(str, object)
    ulog_command_requested = QtCore.pyqtSignal(str, object)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_advanced: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.invoke_command = invoke_command
        self.session_records: dict[str, dict[str, object]] = {}
        self._session_catalog_signature: tuple[object, ...] = ()
        self._last_session_scan_monotonic_s = float("-inf")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Recording",
            )
        )

        run_group = QtWidgets.QGroupBox("Run setup")
        run_layout = QtWidgets.QGridLayout(run_group)
        run_layout.setColumnStretch(1, 1)
        run_layout.setColumnStretch(3, 1)
        self.run_setup_widgets: dict[str, QtWidgets.QWidget] = {}
        for index, (key, label) in enumerate(
            (
                ("experiment_id", "Experiment ID"),
                ("run_id", "Run ID"),
                ("condition_id", "Condition ID"),
            )
        ):
            editor = QtWidgets.QLineEdit()
            editor.setPlaceholderText(label)
            self.run_setup_widgets[key] = editor
            row = index // 2
            column = (index % 2) * 2
            run_layout.addWidget(QtWidgets.QLabel(label), row, column)
            run_layout.addWidget(editor, row, column + 1)
        self.run_platform_combo = QtWidgets.QComboBox()
        self.run_platform_combo.addItem("Drone", "drone")
        self.run_platform_combo.addItem("UAM", "uam")
        self.run_platform_combo.currentIndexChanged.connect(
            lambda _index: self._refresh_run_reference_label()
        )
        self.run_setup_widgets["target_platform"] = self.run_platform_combo
        run_layout.addWidget(QtWidgets.QLabel("Target platform"), 1, 2)
        run_layout.addWidget(self.run_platform_combo, 1, 3)
        identity_layout = QtWidgets.QGridLayout()
        identity_layout.setContentsMargins(0, 0, 0, 0)
        self.run_camera_serial_edits: dict[str, QtWidgets.QLineEdit] = {}
        self.run_camera_stabilization_combos: dict[str, QtWidgets.QComboBox] = {}
        for column, side in enumerate(("left", "right")):
            serial = QtWidgets.QLineEdit()
            serial.setPlaceholderText(f"{side.title()} camera serial")
            serial.setToolTip(
                "Operator-verified physical camera identity bound to the operating K,D profile"
            )
            stabilization = QtWidgets.QComboBox()
            stabilization.addItem("Stabilization unknown", "")
            stabilization.addItem("Stabilization disabled", "disabled")
            stabilization.addItem("Stabilization enabled", "enabled")
            self.run_camera_serial_edits[side] = serial
            self.run_camera_stabilization_combos[side] = stabilization
            self.run_setup_widgets[f"{side}_serial"] = serial
            self.run_setup_widgets[f"{side}_stabilization_state"] = stabilization
            pair = QtWidgets.QHBoxLayout()
            pair.setContentsMargins(0, 0, 0, 0)
            pair.addWidget(serial, 2)
            pair.addWidget(stabilization, 3)
            identity_layout.addLayout(pair, 0, column)
        run_layout.addWidget(QtWidgets.QLabel("Verified camera identity"), 2, 0)
        run_layout.addLayout(identity_layout, 2, 1, 1, 3)
        self.target_endpoint_edit = QtWidgets.QLineEdit()
        self.target_endpoint_edit.setObjectName("targetDroneEndpoint")
        self.target_endpoint_edit.setPlaceholderText("udp:0.0.0.0:14570")
        self.target_system_id_spin = QtWidgets.QSpinBox()
        self.target_system_id_spin.setRange(0, 255)
        self.target_system_id_spin.setSpecialValueText("Any")
        self.target_component_id_spin = QtWidgets.QSpinBox()
        self.target_component_id_spin.setRange(0, 255)
        self.target_component_id_spin.setSpecialValueText("Any")
        run_layout.addWidget(QtWidgets.QLabel("Target MAVLink"), 3, 0)
        run_layout.addWidget(self.target_endpoint_edit, 3, 1)
        target_ids = QtWidgets.QHBoxLayout()
        target_ids.setContentsMargins(0, 0, 0, 0)
        target_ids.addWidget(QtWidgets.QLabel("SYS"))
        target_ids.addWidget(self.target_system_id_spin)
        target_ids.addWidget(QtWidgets.QLabel("COMP"))
        target_ids.addWidget(self.target_component_id_spin)
        run_layout.addLayout(target_ids, 3, 2)
        target_buttons = QtWidgets.QHBoxLayout()
        target_buttons.setContentsMargins(0, 0, 0, 0)
        self.target_connect_button = QtWidgets.QPushButton("Connect target")
        self.target_connect_button.clicked.connect(self._connect_target_drone)
        self.target_stop_button = QtWidgets.QPushButton("Stop target")
        self.target_stop_button.clicked.connect(self._stop_target_drone)
        target_buttons.addWidget(self.target_connect_button)
        target_buttons.addWidget(self.target_stop_button)
        run_layout.addLayout(target_buttons, 3, 3)
        self.run_reference_label = QtWidgets.QLabel()
        self.run_reference_label.setObjectName("compactContract")
        self.run_reference_label.setWordWrap(True)
        run_layout.addWidget(self.run_reference_label, 4, 0, 1, 3)
        self.run_setup_apply_button = QtWidgets.QPushButton("Apply run setup")
        self.run_setup_apply_button.setProperty("commandId", "run.apply_setup")
        self.run_setup_apply_button.clicked.connect(self._apply_run_setup)
        run_layout.addWidget(self.run_setup_apply_button, 4, 3)
        self.run_setup_state_label = QtWidgets.QLabel("PENDING · run setup not locked")
        self.run_setup_state_label.setObjectName("statusStrip")
        self.run_setup_state_label.setWordWrap(True)
        run_layout.addWidget(self.run_setup_state_label, 5, 0, 1, 4)
        self.target_stream_state_label = QtWidgets.QLabel("Target Drone: NOT CONFIGURED")
        self.target_stream_state_label.setObjectName("compactContract")
        self.target_stream_state_label.setWordWrap(True)
        run_layout.addWidget(self.target_stream_state_label, 6, 0, 1, 4)
        layout.addWidget(run_group)
        self._refresh_run_reference_label()
        self._set_run_setup_locked(False)

        status_group = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QGridLayout(status_group)
        self.indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (("recording", "Recording"), ("measurement", "Measurement"), ("camera.left", "CAM L"), ("fps", "FPS"))
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator, index // 2, index % 2)
        layout.addWidget(status_group)

        protocol_group = QtWidgets.QGroupBox("Preflight")
        protocol_layout = QtWidgets.QVBoxLayout(protocol_group)
        protocol_layout.setContentsMargins(6, 6, 6, 6)
        self.protocol_state_label = QtWidgets.QLabel("BLOCKED · evidence waiting")
        self.protocol_state_label.setObjectName("protocolState")
        self.protocol_state_label.setWordWrap(True)
        protocol_layout.addWidget(self.protocol_state_label)
        self.protocol_check_labels: dict[str, QtWidgets.QLabel] = {}
        protocol_checks_layout = QtWidgets.QGridLayout()
        protocol_checks_layout.setContentsMargins(0, 0, 0, 0)
        # Pixhawk mode currently evaluates nine checks; keep one spare row so
        # an additional source-backed gate does not disappear silently.
        for index in range(10):
            label = QtWidgets.QLabel("--")
            label.setObjectName("protocolCheck")
            label.setWordWrap(True)
            self.protocol_check_labels[str(index)] = label
            protocol_checks_layout.addWidget(label, index // 2, index % 2)
        protocol_layout.addLayout(protocol_checks_layout)
        layout.addWidget(protocol_group)

        session_group = QtWidgets.QGroupBox("Session")
        session_layout = QtWidgets.QGridLayout(session_group)
        self.session_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (("state", "State"), ("session_id", "Session ID"), ("output_dir", "Output"), ("elapsed_s", "Elapsed"), ("status_label", "Finalization"))
        ):
            value = EngineeringValue(label)
            self.session_values[key] = value
            session_layout.addWidget(value, index // 2, index % 2)
        hr_capture_controls = QtWidgets.QHBoxLayout()
        hr_capture_controls.setContentsMargins(0, 0, 0, 0)
        hr_capture_controls.addWidget(QtWidgets.QLabel("HR purpose"))
        self.pnp_capture_purpose_combo = QtWidgets.QComboBox()
        self.pnp_capture_purpose_combo.setObjectName("hrCapturePurpose")
        self.pnp_capture_purpose_combo.addItem("Pre", "pre")
        self.pnp_capture_purpose_combo.addItem("Post", "post")
        self.pnp_capture_purpose_combo.addItem("Checkpoint", "checkpoint")
        self.pnp_capture_purpose_combo.addItem("Unspecified", "unspecified")
        hr_capture_controls.addWidget(self.pnp_capture_purpose_combo)
        self.pnp_capture_button = QtWidgets.QPushButton("HR Capture")
        self.pnp_capture_button.setProperty("commandId", "recording.hr.capture")
        self.pnp_capture_button.setToolTip(
            "Sends SIHR TRUE, CPTR, and ICPT to both cameras. Photos remain in camera "
            "internal storage. The chosen Pre/Post/Checkpoint purpose is recorded with "
            "the capture event; image import is a later, explicit evidence action."
        )
        self.pnp_capture_button.clicked.connect(self._capture_pnp_hr_pair)
        hr_capture_controls.addWidget(self.pnp_capture_button)
        self.oz_capture_button = QtWidgets.QPushButton("OZ Capture")
        self.oz_capture_button.setProperty("commandId", "recording.oz.capture")
        self.oz_capture_button.setToolTip(
            "Sends SIVI TRUE, CPTR, and ICPT to both cameras for a separate visible/OZ "
            "still. It is stored on the camera independently from the RTSP session video."
        )
        self.oz_capture_button.clicked.connect(self._capture_pnp_oz_pair)
        hr_capture_controls.addWidget(self.oz_capture_button)
        hr_capture_controls.addStretch(1)
        session_layout.addLayout(hr_capture_controls, 3, 0, 1, 2)
        self.pnp_capture_status_label = QtWidgets.QLabel(
            "HR Capture: camera-internal storage · select event, then import exact left/right files"
        )
        self.pnp_capture_status_label.setObjectName("compactContract")
        self.pnp_capture_status_label.setWordWrap(True)
        session_layout.addWidget(self.pnp_capture_status_label, 4, 0, 1, 2)
        self.oz_capture_status_label = QtWidgets.QLabel(
            "OZ Capture: camera-internal FHD still · separate from RTSP session video"
        )
        self.oz_capture_status_label.setObjectName("compactContract")
        self.oz_capture_status_label.setWordWrap(True)
        session_layout.addWidget(self.oz_capture_status_label, 5, 0, 1, 2)
        self.hr_capture_model = _table_model(
            self,
            ("Capture event", "Purpose", "State", "Left HR", "Right HR"),
        )
        self.hr_capture_table = QtWidgets.QTableView()
        self.hr_capture_table.setObjectName("hrCaptureEventTable")
        self.hr_capture_table.setModel(self.hr_capture_model)
        self.hr_capture_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.hr_capture_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.hr_capture_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.hr_capture_table.verticalHeader().setVisible(False)
        self.hr_capture_table.horizontalHeader().setStretchLastSection(True)
        self.hr_capture_table.setMaximumHeight(112)
        self.hr_capture_table.selectionModel().selectionChanged.connect(
            self._refresh_hr_import_actions
        )
        session_layout.addWidget(self.hr_capture_table, 6, 0, 1, 2)
        hr_import_actions = QtWidgets.QHBoxLayout()
        self.hr_import_left_button = QtWidgets.QPushButton("Import selected Left HR")
        self.hr_import_left_button.setProperty("commandId", "recording.hr.import")
        self.hr_import_left_button.clicked.connect(
            lambda _checked=False: self._import_selected_hr("left")
        )
        self.hr_import_right_button = QtWidgets.QPushButton("Import selected Right HR")
        self.hr_import_right_button.setProperty("commandId", "recording.hr.import")
        self.hr_import_right_button.clicked.connect(
            lambda _checked=False: self._import_selected_hr("right")
        )
        hr_import_actions.addWidget(self.hr_import_left_button)
        hr_import_actions.addWidget(self.hr_import_right_button)
        hr_import_actions.addStretch(1)
        session_layout.addLayout(hr_import_actions, 7, 0, 1, 2)
        self._refresh_hr_import_actions()
        layout.addWidget(session_group)

        ulog_group = QtWidgets.QGroupBox("PX4 ULog evidence")
        ulog_layout = QtWidgets.QGridLayout(ulog_group)
        self.ulog_source_combo = QtWidgets.QComboBox()
        self.ulog_source_combo.addItem("P1 / Left", "p1")
        self.ulog_source_combo.addItem("P2 / Right", "p2")
        self.ulog_source_combo.addItem("Target Drone", "target")
        self.ulog_ownership_combo = QtWidgets.QComboBox()
        self.ulog_ownership_combo.addItem("Ownership unknown", "UNKNOWN")
        self.ulog_ownership_combo.addItem("Workstation-owned claim", "WORKSTATION")
        self.ulog_ownership_combo.addItem("External logger", "EXTERNAL")
        self.ulog_import_button = QtWidgets.QPushButton("Import ULog")
        self.ulog_import_button.setProperty("commandId", "recording.ulog.import")
        self.ulog_import_button.clicked.connect(self._import_ulog)
        self.ulog_status_label = QtWidgets.QLabel(
            "No ULog imported · automatic capture requires verified logger ACK/status ownership"
        )
        self.ulog_status_label.setObjectName("compactContract")
        self.ulog_status_label.setWordWrap(True)
        ulog_layout.addWidget(QtWidgets.QLabel("Source"), 0, 0)
        ulog_layout.addWidget(self.ulog_source_combo, 0, 1)
        ulog_layout.addWidget(QtWidgets.QLabel("Provenance"), 0, 2)
        ulog_layout.addWidget(self.ulog_ownership_combo, 0, 3)
        ulog_layout.addWidget(self.ulog_import_button, 0, 4)
        ulog_layout.addWidget(self.ulog_status_label, 1, 0, 1, 5)
        self.ulog_model = _table_model(
            self,
            ("Source", "Acquisition", "Ownership", "Bytes", "SHA-256", "Path"),
        )
        self.ulog_table = QtWidgets.QTableView()
        self.ulog_table.setObjectName("ulogEvidenceTable")
        self.ulog_table.setModel(self.ulog_model)
        self.ulog_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.ulog_table.verticalHeader().setVisible(False)
        self.ulog_table.setMaximumHeight(120)
        self.ulog_table.horizontalHeader().setStretchLastSection(True)
        ulog_layout.addWidget(self.ulog_table, 2, 0, 1, 5)
        layout.addWidget(ulog_group)

        data_group = QtWidgets.QGroupBox("Files")
        data_layout = QtWidgets.QVBoxLayout(data_group)
        self.count_values: dict[str, EngineeringValue] = {}
        count_layout = QtWidgets.QGridLayout()
        for index, (key, label) in enumerate(
            (("pairs", "Pairs"), ("camera_left", "CAM L rows"), ("camera_right", "CAM R rows"), ("px4_left", "P1 rows"), ("px4_right", "P2 rows"), ("gnss_base", "GNSS rows"))
        ):
            value = EngineeringValue(label)
            self.count_values[key] = value
            count_layout.addWidget(value, index // 3, index % 3)
        data_layout.addLayout(count_layout)
        self.artifact_model = _table_model(self, ("File", "Path", "Kind"))
        self.artifact_table = QtWidgets.QTableView()
        self.artifact_table.setModel(self.artifact_model)
        self.artifact_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.artifact_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.artifact_table.verticalHeader().setVisible(False)
        self.artifact_table.setMaximumHeight(150)
        self.artifact_table.horizontalHeader().setStretchLastSection(True)
        data_layout.addWidget(self.artifact_table)
        layout.addWidget(data_group)

        review_group = QtWidgets.QGroupBox("Review")
        review_layout = QtWidgets.QVBoxLayout(review_group)
        review_layout.setContentsMargins(6, 6, 6, 6)
        review_layout.setSpacing(5)
        selector_layout = QtWidgets.QGridLayout()
        selector_layout.setHorizontalSpacing(8)
        selector_layout.setColumnStretch(1, 1)
        self.session_selector = QtWidgets.QComboBox()
        self.session_selector.setObjectName("sessionReviewSelector")
        self.session_selector.currentIndexChanged.connect(lambda _index: self._refresh_review_artifacts())
        self.refresh_sessions_button = QtWidgets.QPushButton("Refresh")
        self.refresh_sessions_button.clicked.connect(
            lambda _checked=False: self.refresh_sessions(force=True)
        )
        self.open_session_button = QtWidgets.QPushButton("Open folder")
        self.open_session_button.clicked.connect(self._open_selected_session)
        selector_layout.addWidget(QtWidgets.QLabel("Session"), 0, 0)
        selector_layout.addWidget(self.session_selector, 0, 1)
        selector_layout.addWidget(self.refresh_sessions_button, 0, 2)
        selector_layout.addWidget(self.open_session_button, 0, 3)
        self.artifact_selector = QtWidgets.QComboBox()
        self.artifact_selector.setObjectName("sessionArtifactSelector")
        self.artifact_selector.currentIndexChanged.connect(lambda _index: self._preview_selected_artifact())
        self.preview_artifact_button = QtWidgets.QPushButton("Preview")
        self.preview_artifact_button.clicked.connect(self._preview_selected_artifact)
        self.open_artifact_button = QtWidgets.QPushButton("Open file")
        self.open_artifact_button.clicked.connect(self._open_selected_artifact)
        selector_layout.addWidget(QtWidgets.QLabel("Artifact"), 1, 0)
        selector_layout.addWidget(self.artifact_selector, 1, 1)
        selector_layout.addWidget(self.preview_artifact_button, 1, 2)
        selector_layout.addWidget(self.open_artifact_button, 1, 3)
        review_layout.addLayout(selector_layout)
        self.review_status_label = QtWidgets.QLabel(
            "Read-only review"
        )
        self.review_status_label.setObjectName("statusStrip")
        self.review_status_label.setWordWrap(True)
        self.review_status_label.setToolTip(
            "Saved fields and timestamps are unchanged; no NED/ENU conversion is applied."
        )
        review_layout.addWidget(self.review_status_label)
        self.review_model = QtGui.QStandardItemModel(0, 0, self)
        self.review_table = QtWidgets.QTableView()
        self.review_table.setModel(self.review_model)
        self.review_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.review_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.review_table.verticalHeader().setVisible(False)
        self.review_table.setMaximumHeight(230)
        self.review_table.horizontalHeader().setStretchLastSection(True)
        review_layout.addWidget(self.review_table)
        review_actions = QtWidgets.QHBoxLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        if invoke_command is not None:
            save_button = _command_button("Save", "session.save", invoke_command)
            self.operation_buttons["session.save"] = save_button
            review_actions.addWidget(save_button)
        review_actions.addStretch(1)
        review_layout.addLayout(review_actions)
        layout.addWidget(review_group)

        self.contract_label = QtWidgets.QLabel("Time: recording clock")
        self.contract_label.setToolTip("Session IDs use wall time; samples retain the recording time axis.")
        self.contract_label.setObjectName("compactContract")
        layout.addWidget(self.contract_label)
        button = QtWidgets.QPushButton("Advanced files")
        button.setToolTip(
            "Open playback, export, and media review tools."
        )
        button.clicked.connect(open_advanced)
        layout.addWidget(button, alignment=QtCore.Qt.AlignRight)
        layout.addStretch(1)
        self.refresh_sessions()

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        recording_status = snapshot.get("recording")
        recording_locked = bool(
            recording_status
            and recording_status.state in {"ACTIVE", "FINALIZING"}
        )
        self._refresh_run_setup_snapshot(recording_locked=recording_locked)
        self._refresh_protocol(snapshot)
        data = _evidence(self.backend).recording()
        panel = getattr(self.backend, "panel", self.backend)
        pnp_capture_active = getattr(panel, "pnp_photo_capture_thread", None) is not None
        oz_capture_active = getattr(panel, "oz_photo_capture_thread", None) is not None
        capture_active = bool(pnp_capture_active or oz_capture_active)
        self.pnp_capture_button.setEnabled(not capture_active)
        self.oz_capture_button.setEnabled(not capture_active)
        records = list(getattr(panel, "pnp_photo_capture_records", ()) or ())
        self._sync_hr_capture_records(records)
        if pnp_capture_active:
            self.pnp_capture_status_label.setText("HR Capture: in progress")
        elif records:
            latest = records[-1]
            ftp_text = "FTP=ok" if latest.get("ftp_downloaded") else "FTP=disabled/failed"
            self.pnp_capture_status_label.setText(
                f"HR Capture: {latest.get('capture_id', 'unknown')} · "
                f"{latest.get('state', 'unknown')} · camera-internal · {ftp_text}"
            )
        else:
            self.pnp_capture_status_label.setText(
                "HR Capture: camera-internal storage · select event, then import exact left/right files"
            )
        oz_records = list(getattr(panel, "oz_photo_capture_records", ()) or ())
        if oz_capture_active:
            self.oz_capture_status_label.setText("OZ Capture: in progress")
        elif oz_records:
            latest_oz = oz_records[-1]
            oz_ftp_text = "FTP=ok" if latest_oz.get("ftp_downloaded") else "FTP=disabled/failed"
            self.oz_capture_status_label.setText(
                f"OZ Capture: {latest_oz.get('capture_id', 'unknown')} · "
                f"{latest_oz.get('state', 'unknown')} · camera-internal · RTSP separate · {oz_ftp_text}"
            )
        else:
            self.oz_capture_status_label.setText(
                "OZ Capture: camera-internal FHD still · separate from RTSP session video"
            )
        for key, value in self.session_values.items():
            raw = data.get(key)
            if key == "elapsed_s" and raw not in (None, ""):
                raw = f"{float(raw):.1f} s"
            value.set_value(raw, source=str(data.get("source", "")))
        counts = data.get("sample_counts", {})
        for key, value in self.count_values.items():
            value.set_value(counts.get(key, 0) if isinstance(counts, dict) else "--", source="session spool rows")
        artifact_rows: list[tuple[object, ...]] = []
        output_dir = str(data.get("output_dir") or "")
        paths = data.get("video_paths", {})
        if isinstance(paths, dict):
            for side, path in sorted(paths.items()):
                artifact_rows.append((f"{side} video", path or "not prepared", "Video"))
        for artifact in data.get("artifacts", ()) or ():
            artifact_rows.append(
                (artifact, f"{output_dir}/{artifact}" if output_dir else "--", "Session file")
            )
        if not artifact_rows:
            artifact_rows.append(("--", "No finalized files", "Recording"))
        _sync_table_rows(self.artifact_model, tuple(artifact_rows))
        ulog_reader = getattr(self.backend, "ulog_snapshot", None)
        if callable(ulog_reader):
            try:
                self.apply_ulog_snapshot(ulog_reader())
            except Exception as exc:
                self.ulog_status_label.setText(f"ULog status unavailable · {exc}")
        target_drone_reader = getattr(self.backend, "target_drone_snapshot", None)
        if callable(target_drone_reader):
            try:
                self._apply_target_drone_snapshot(target_drone_reader())
            except Exception as exc:
                self.target_stream_state_label.setText(f"Target Drone: status unavailable · {exc}")
        self.refresh_sessions(force=False)

    def run_setup_payload(self) -> dict[str, object]:
        return {
            "experiment_id": self.run_setup_widgets["experiment_id"].text().strip(),
            "run_id": self.run_setup_widgets["run_id"].text().strip(),
            "condition_id": self.run_setup_widgets["condition_id"].text().strip(),
            "target_platform": str(self.run_platform_combo.currentData() or "drone"),
            "condition": {
                "left_serial": self.run_camera_serial_edits["left"].text().strip(),
                "right_serial": self.run_camera_serial_edits["right"].text().strip(),
                "left_stabilization_state": str(
                    self.run_camera_stabilization_combos["left"].currentData() or ""
                ),
                "right_stabilization_state": str(
                    self.run_camera_stabilization_combos["right"].currentData() or ""
                ),
                "camera_identity_source": "operator_verified_run_setup",
            },
        }

    def target_drone_payload(self) -> dict[str, object]:
        system_id = int(self.target_system_id_spin.value())
        component_id = int(self.target_component_id_spin.value())
        return {
            "mode": str(self.run_platform_combo.currentData() or "drone"),
            "endpoint": self.target_endpoint_edit.text().strip(),
            "expected_system_id": None if system_id == 0 else system_id,
            "expected_component_id": None if component_id == 0 else component_id,
        }

    def _connect_target_drone(self) -> None:
        method = getattr(self.backend, "target_drone_connect", None)
        if not callable(method):
            self.target_stream_state_label.setText("Target Drone: owner unavailable")
            return
        try:
            result = method(self.target_drone_payload())
        except Exception as exc:
            self.target_stream_state_label.setText(f"Target Drone: ERROR · {exc}")
            return
        self._apply_target_drone_snapshot(result)

    def _stop_target_drone(self) -> None:
        method = getattr(self.backend, "target_drone_stop", None)
        if not callable(method):
            self.target_stream_state_label.setText("Target Drone: owner unavailable")
            return
        try:
            result = method(timeout_s=3.0)
        except Exception as exc:
            self.target_stream_state_label.setText(f"Target Drone: ERROR · {exc}")
            return
        self._apply_target_drone_snapshot(result)

    def _apply_target_drone_snapshot(self, value: object) -> None:
        if not isinstance(value, Mapping):
            return
        phase = str(value.get("phase", "UNKNOWN") or "UNKNOWN")
        count = value.get("accepted_message_count", 0)
        detail = str(value.get("last_error", "") or "")
        text = f"Target Drone: {phase} · accepted={count} · recv-only"
        if detail:
            text += f" · {detail}"
        self.target_stream_state_label.setText(text)

    def _run_setup_apply_method(self) -> Callable[[dict[str, object]], object] | None:
        panel = getattr(self.backend, "panel", self.backend)
        for owner, name in (
            (self.backend, "apply_run_setup"),
            (panel, "apply_workstation_run_setup"),
        ):
            method = getattr(owner, name, None)
            if callable(method):
                return method
        return None

    def _apply_run_setup(self) -> None:
        apply_method = self._run_setup_apply_method()
        if apply_method is None:
            self.run_setup_state_label.setText(
                "PENDING · run setup owner interface unavailable"
            )
            return
        try:
            result = apply_method(self.run_setup_payload())
        except Exception as exc:
            self.run_setup_state_label.setText(f"ERROR · {exc}")
            return
        self.run_setup_state_label.setText(
            "REQUESTED · awaiting owner readback"
            if not isinstance(result, Mapping)
            else str(result.get("state", "REQUESTED"))
        )
        if isinstance(result, Mapping):
            self._apply_run_setup_snapshot(result, recording_locked=False)

    def _refresh_run_reference_label(self) -> None:
        is_uam = str(self.run_platform_combo.currentData() or "drone") == "uam"
        if is_uam:
            detail = "Reference acquisition: deferred external UAM RTK (expected; not a live error)"
        else:
            detail = "Reference acquisition: live target Drone Pixhawk / RTK when available"
        self.run_reference_label.setText(detail)
        self.target_endpoint_edit.setVisible(not is_uam)
        self.target_system_id_spin.setVisible(not is_uam)
        self.target_component_id_spin.setVisible(not is_uam)
        self.target_connect_button.setText("Set UAM deferred" if is_uam else "Connect target")

    def _set_run_setup_locked(self, locked: bool) -> None:
        for widget in self.run_setup_widgets.values():
            widget.setEnabled(not locked)
        self.run_setup_apply_button.setEnabled(
            not locked and self._run_setup_apply_method() is not None
        )
        # The target link is independent of immutable experiment identity.
        # It may be stopped during a run, but mode/endpoint remain frozen once
        # the first evidence event locks setup.
        self.target_endpoint_edit.setEnabled(not locked)
        self.target_system_id_spin.setEnabled(not locked)
        self.target_component_id_spin.setEnabled(not locked)

    def _refresh_run_setup_snapshot(self, *, recording_locked: bool) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        readers = (
            getattr(self.backend, "run_setup_snapshot", None),
            getattr(panel, "workstation_run_setup_snapshot", None),
        )
        for reader in readers:
            if not callable(reader):
                continue
            try:
                result = reader()
            except Exception:
                break
            if isinstance(result, Mapping):
                self._apply_run_setup_snapshot(result, recording_locked=recording_locked)
                return
            break
        self._set_run_setup_locked(recording_locked)

    def _apply_run_setup_snapshot(
        self,
        snapshot: Mapping[str, object],
        *,
        recording_locked: bool,
    ) -> None:
        for key in ("experiment_id", "run_id", "condition_id"):
            value = snapshot.get(key)
            widget = self.run_setup_widgets[key]
            if value is not None and isinstance(widget, QtWidgets.QLineEdit):
                widget.setText(str(value))
        condition = snapshot.get("condition")
        if isinstance(condition, Mapping):
            for side in ("left", "right"):
                serial = condition.get(f"{side}_serial")
                if serial is not None:
                    self.run_camera_serial_edits[side].setText(str(serial))
                stabilization = str(
                    condition.get(f"{side}_stabilization_state", "") or ""
                )
                index = self.run_camera_stabilization_combos[side].findData(stabilization)
                if index >= 0:
                    self.run_camera_stabilization_combos[side].setCurrentIndex(index)
        platform = str(snapshot.get("target_platform", "") or "").lower()
        if platform:
            index = self.run_platform_combo.findData(platform)
            if index >= 0:
                self.run_platform_combo.setCurrentIndex(index)
        locked = recording_locked or bool(snapshot.get("locked", False))
        state = str(snapshot.get("state", "LOCKED" if locked else "PENDING") or "PENDING")
        source = str(snapshot.get("source", "run setup owner snapshot") or "")
        self.run_setup_state_label.setText(f"{state} · {source}")
        self._set_run_setup_locked(locked)
        self._refresh_run_reference_label()

    def _capture_pnp_hr_pair(self) -> None:
        payload = {
            "purpose": str(
                self.pnp_capture_purpose_combo.currentData()
                or "unspecified"
            ),
            "storage": "camera_internal_then_exact_file_import",
        }
        self._request_hr_command("recording.hr.capture", payload)

    def _capture_pnp_oz_pair(self) -> None:
        payload = {
            "purpose": str(
                self.pnp_capture_purpose_combo.currentData()
                or "unspecified"
            ),
            "storage": "camera_internal_separate_from_rtsp",
            "capture_channel": "oz",
        }
        self._request_hr_command("recording.oz.capture", payload)

    @staticmethod
    def _hr_side_state(record: Mapping[str, object], side: str) -> str:
        imported = record.get(side)
        if isinstance(imported, Mapping):
            digest = str(imported.get("sha256", "") or "")
            return f"IMPORTED {digest[:8]}…" if digest else str(imported.get("state", "IMPORTED"))
        cameras = record.get("cameras")
        camera = cameras.get(side) if isinstance(cameras, Mapping) else None
        if isinstance(camera, Mapping):
            return str(camera.get("import_state", camera.get("state", "--")) or "--")
        return "--"

    def _sync_hr_capture_records(self, records: list[object]) -> None:
        selected_capture_id = self._selected_hr_capture_id()
        rows: list[tuple[object, ...]] = []
        for raw in records:
            if not isinstance(raw, Mapping):
                continue
            rows.append(
                (
                    raw.get("capture_id", raw.get("event_id", "--")),
                    raw.get("purpose", "unspecified"),
                    raw.get("state", "--"),
                    self._hr_side_state(raw, "left"),
                    self._hr_side_state(raw, "right"),
                )
            )
        _sync_table_rows(self.hr_capture_model, tuple(rows))
        selected_row = -1
        if selected_capture_id:
            for row in range(self.hr_capture_model.rowCount()):
                item = self.hr_capture_model.item(row, 0)
                if item is not None and item.text() == selected_capture_id:
                    selected_row = row
                    break
        if selected_row < 0 and rows:
            selected_row = len(rows) - 1
        if selected_row >= 0:
            self.hr_capture_table.selectRow(selected_row)
        else:
            self.hr_capture_table.clearSelection()
        self._refresh_hr_import_actions()

    def _selected_hr_capture_id(self) -> str:
        rows = self.hr_capture_table.selectionModel().selectedRows()
        row = rows[0].row() if rows else self.hr_capture_table.currentIndex().row()
        item = self.hr_capture_model.item(row, 0) if row >= 0 else None
        return "" if item is None else item.text().strip()

    def _selected_hr_capture_purpose(self) -> str:
        rows = self.hr_capture_table.selectionModel().selectedRows()
        row = rows[0].row() if rows else self.hr_capture_table.currentIndex().row()
        item = self.hr_capture_model.item(row, 1) if row >= 0 else None
        return "unspecified" if item is None else (item.text().strip().lower() or "unspecified")

    def _refresh_hr_import_actions(self, *_args: object) -> None:
        enabled = bool(self._selected_hr_capture_id())
        self.hr_import_left_button.setEnabled(enabled)
        self.hr_import_right_button.setEnabled(enabled)

    def _import_selected_hr(self, side: str) -> None:
        capture_id = self._selected_hr_capture_id()
        if not capture_id:
            self.pnp_capture_status_label.setText("HR Import: select a capture event")
            return
        self._request_hr_command(
            "recording.hr.import",
            {
                "capture_id": capture_id,
                "purpose": self._selected_hr_capture_purpose(),
                "side": str(side),
                "file_selection": "owner_dialog_required",
                "byte_contract": "exact source bytes; owner computes SHA-256 and decoded resolution",
            },
        )

    def _import_ulog(self) -> None:
        self.ulog_command_requested.emit(
            "recording.ulog.import",
            {
                "source": str(self.ulog_source_combo.currentData() or "p1"),
                "ownership": str(self.ulog_ownership_combo.currentData() or "UNKNOWN"),
                "file_selection": "owner_dialog_required",
            },
        )
        self.ulog_status_label.setText("ULog import requested: awaiting owner file selection")

    def apply_ulog_snapshot(self, snapshot: object) -> None:
        if not isinstance(snapshot, Mapping):
            return
        artifacts = snapshot.get("artifacts")
        rows: list[tuple[object, ...]] = []
        for artifact in artifacts if isinstance(artifacts, (tuple, list)) else ():
            if not isinstance(artifact, Mapping):
                continue
            rows.append(
                (
                    artifact.get("source_id", "--"),
                    artifact.get("acquisition", "--"),
                    artifact.get("ownership", "--"),
                    artifact.get("size_bytes", "--"),
                    str(artifact.get("sha256", ""))[:16],
                    artifact.get("relative_path", "--"),
                )
            )
        _sync_table_rows(self.ulog_model, tuple(rows))
        if rows:
            self.ulog_status_label.setText(
                f"ULog artifacts: {len(rows)} · exact bytes/hash verified · runtime telemetry remains reference"
            )
        elif str(snapshot.get("state", "")) == "NOT_PREPARED":
            self.ulog_status_label.setText(
                "No ULog imported · automatic capture requires verified logger ACK/status ownership"
            )

    def _request_hr_command(self, command_id: str, payload: dict[str, object]) -> None:
        """Emit typed camera-still work, with legacy capture fallback when unconnected.

        The presentation layer never opens or hashes an HR file.  Main-window
        or controller owners receive only the selected event/side/purpose and
        perform the file dialog, exact-byte import, validation, and storage.
        """

        if self.receivers(self.hr_command_requested) > 0:
            self.hr_command_requested.emit(command_id, dict(payload))
            action = "Capture requested" if command_id.endswith("capture") else "Import requested"
            status_label = (
                self.pnp_capture_status_label
                if command_id.startswith("recording.hr")
                else self.oz_capture_status_label
            )
            channel = "HR" if command_id.startswith("recording.hr") else "OZ"
            status_label.setText(f"{channel} {action}: awaiting owner")
            return
        if command_id == "recording.hr.capture":
            method = getattr(self.backend, "capture_pnp_hr_photos", None)
            status_label = self.pnp_capture_status_label
            channel = "HR"
        elif command_id == "recording.oz.capture":
            method = getattr(self.backend, "capture_pnp_oz_photos", None)
            status_label = self.oz_capture_status_label
            channel = "OZ"
        else:
            method = getattr(self.backend, "import_pnp_hr_photo", None)
            status_label = self.pnp_capture_status_label
            channel = "HR"
        if not callable(method):
            status_label.setText(f"{channel} {command_id.rsplit('.', 1)[-1].title()}: control unavailable")
            return
        try:
            try:
                result = method(dict(payload))
            except TypeError:
                # Keep the established no-argument capture facade functional
                # until its owner adopts the typed purpose payload.
                if command_id not in {"recording.hr.capture", "recording.oz.capture"}:
                    raise
                result = method()
            if result is not False:
                state = "in progress" if command_id.endswith("capture") else "requested"
                status_label.setText(f"{channel} {command_id.rsplit('.', 1)[-1].title()}: {state}")
        except Exception as exc:
            status_label.setText(f"{channel} {command_id.rsplit('.', 1)[-1].title()}: {exc}")

    def _refresh_protocol(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        panel = getattr(self.backend, "panel", self.backend)
        settings = getattr(panel, "settings", None)
        snapshot_method = getattr(self.backend, "workstation_settings_snapshot", None)
        try:
            settings_snapshot = snapshot_method() if callable(snapshot_method) else {}
        except Exception:
            settings_snapshot = {}
        settings_snapshot = settings_snapshot if isinstance(settings_snapshot, Mapping) else {}
        gnss_config = settings_snapshot.get("gnss_operation")
        gnss_config = gnss_config if isinstance(gnss_config, Mapping) else {}
        readiness_config = gnss_config.get("readiness")
        readiness_config = readiness_config if isinstance(readiness_config, Mapping) else {}
        try:
            minimum_fix_type = int(readiness_config.get("minimum_fix_type", 3))
        except (TypeError, ValueError):
            minimum_fix_type = 3
        physical_alignment = evidence.physical_alignment()
        checkpoints = physical_alignment.get("time_sync_checkpoints")
        if not isinstance(checkpoints, (tuple, list)):
            checkpoints = getattr(panel, "workstation_time_sync_checkpoints", ())
        final_sync_verified = any(
            isinstance(item, Mapping)
            and str(item.get("phase", "")) == "final"
            and "VERIFIED" in str(item.get("status", "")).upper()
            for item in checkpoints or ()
        )
        measurement = evidence.measurement_settings()
        readiness = evaluate_recording_protocol(
            snapshot,
            distance_mode=str(measurement.get("distance_mode", "pixhawk") or "pixhawk"),
            calibration=evidence.calibration(),
            baseline=evidence.baseline(),
            require_rtk_fixed=bool(
                settings_snapshot.get(
                    "require_rtk_fixed",
                    getattr(settings, "require_rtk_fixed", False),
                )
            ),
            minimum_fix_type=minimum_fix_type,
            physical_alignment=physical_alignment,
            final_sync_verified=final_sync_verified,
        )
        self.protocol_state_label.setText(f"{readiness.state} · {len(readiness.blocking)} checks")
        state_style = (
            f"color:{SUCCESS_TEXT}; border:1px solid {PANEL_BORDER}; padding:5px;"
            if readiness.state == "READY"
            else f"color:{ERROR_TEXT}; border:1px solid {ERROR_TEXT}; padding:5px;"
        )
        self.protocol_state_label.setStyleSheet(state_style)
        for index, label in enumerate(self.protocol_check_labels.values()):
            if index >= len(readiness.checks):
                label.clear()
                label.setVisible(False)
                continue
            check = readiness.checks[index]
            label.setVisible(True)
            detail = str(check.detail)
            label.setText(f"{'✓' if check.passed else '×'} {check.label}: {_compact_text(detail, limit=112)}")
            label.setToolTip(detail)
            label.setStyleSheet(
                f"color:{SUCCESS_TEXT};" if check.passed else f"color:{ERROR_TEXT};"
            )

    def refresh_sessions(self, *, force: bool = True) -> None:
        now_s = time.monotonic()
        if (
            not force
            and now_s - self._last_session_scan_monotonic_s
            < SESSION_CATALOG_REFRESH_INTERVAL_S
        ):
            return
        self._last_session_scan_monotonic_s = now_s
        records = _evidence(self.backend).session_catalog()
        signature = tuple(
            (
                record.get("session_id"),
                record.get("status"),
                record.get("error"),
                record.get("retryable"),
                tuple(record.get("artifacts", ()) or ()),
            )
            for record in records
        )
        if signature == self._session_catalog_signature and self.session_selector.count() > 0:
            return
        current = str(self.session_selector.currentData() or "")
        panel = getattr(self.backend, "panel", self.backend)
        current = current or str(getattr(panel, "recording_session_id", "") or "")
        self.session_records = {
            str(record.get("session_id")): dict(record)
            for record in records
            if record.get("session_id")
        }
        self._session_catalog_signature = signature
        self.session_selector.blockSignals(True)
        try:
            self.session_selector.clear()
            for record in records:
                session_id = str(record.get("session_id") or "")
                status = str(record.get("status") or "unknown")
                self.session_selector.addItem(f"{session_id} [{status}]", session_id)
            if current:
                index = self.session_selector.findData(current)
                if index >= 0:
                    self.session_selector.setCurrentIndex(index)
        finally:
            self.session_selector.blockSignals(False)
        self._refresh_review_artifacts()

    def _selected_record(self) -> dict[str, object] | None:
        session_id = str(self.session_selector.currentData() or "")
        return self.session_records.get(session_id)

    def _refresh_review_artifacts(self) -> None:
        record = self._selected_record()
        current = str(self.artifact_selector.currentData() or "")
        self.artifact_selector.blockSignals(True)
        try:
            self.artifact_selector.clear()
            if record is not None:
                for artifact in record.get("artifacts", ()) or ():
                    self.artifact_selector.addItem(str(artifact), str(artifact))
            if current:
                index = self.artifact_selector.findData(current)
                if index >= 0:
                    self.artifact_selector.setCurrentIndex(index)
            if self.artifact_selector.currentIndex() < 0 and self.artifact_selector.count() > 0:
                self.artifact_selector.setCurrentIndex(0)
        finally:
            self.artifact_selector.blockSignals(False)
        self._preview_selected_artifact()

    def _selected_artifact_name(self) -> str:
        return str(self.artifact_selector.currentData() or "")

    def _selected_artifact_path(self) -> Path | None:
        record = self._selected_record()
        if record is None:
            return None
        output_dir = Path(str(record.get("path") or ""))
        artifact = self._selected_artifact_name()
        if not artifact:
            return output_dir if output_dir.exists() else None
        candidate = output_dir / Path(artifact).name
        if candidate.exists():
            return candidate
        published = output_dir / "published" / Path(artifact).name
        return published if published.exists() else candidate

    def _open_selected_session(self) -> None:
        record = self._selected_record()
        if record is None:
            self.review_status_label.setText("No session selected")
            return
        self._open_path(Path(str(record.get("path") or "")))

    def _open_selected_artifact(self) -> None:
        path = self._selected_artifact_path()
        if path is None:
            self.review_status_label.setText("No file selected")
            return
        self._open_path(path)

    def _open_path(self, path: Path) -> None:
        if not path.exists():
            self.review_status_label.setText(f"Path unavailable: {path}")
            return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))
        self.review_status_label.setText(f"Opened: {path}")

    def _preview_selected_artifact(self) -> None:
        record = self._selected_record()
        artifact = self._selected_artifact_name()
        if record is None or not artifact:
            self.review_model.clear()
            self.review_status_label.setText("Select a session and file")
            return
        data = _evidence(self.backend).session_artifact_rows(str(record.get("session_id")), artifact)
        error = str(data.get("error") or "")
        if error:
            self.review_model.clear()
            self.review_status_label.setText(f"{artifact}: {error}")
            return
        columns = tuple(str(value) for value in data.get("columns", ()) or ())
        rows = data.get("rows", ()) or ()
        self.review_model.clear()
        self.review_model.setColumnCount(len(columns))
        self.review_model.setHorizontalHeaderLabels(list(columns))
        for row in rows:
            if not isinstance(row, dict):
                continue
            _append_table_row(self.review_model, tuple(row.get(column, "") for column in columns))
        record_error = str(record.get("error") or "")
        failure_note = f" | manifest error: {record_error}" if record_error else ""
        review_detail = (
            f"{artifact}: {len(rows)} rows · {data.get('time_axis', 'stored time axis')} · "
            "fields unchanged · no NED/ENU conversion"
            f"{failure_note}"
        )
        self.review_status_label.setText(f"{artifact}: {len(rows)} rows")
        self.review_status_label.setToolTip(
            f"{review_detail}\n{data.get('coordinate_contract', 'stored fields preserved')}"
        )


class DiagnosticsContent(QtWidgets.QWidget):
    def __init__(self, backend: object, status_model: StatusModel, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self.tabs = QtWidgets.QTabWidget()
        self.system_model = QtGui.QStandardItemModel(0, 6, self)
        self.system_model.setHorizontalHeaderLabels(["Key", "State", "Level", "Source", "Age", "Detail"])
        self.system_table = QtWidgets.QTableView()
        self.system_table.setModel(self.system_model)
        self.system_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.system_table.verticalHeader().setVisible(False)
        for column, width in ((0, 180), (1, 110), (2, 70), (3, 360), (4, 80)):
            self.system_table.setColumnWidth(column, width)
        self.system_table.horizontalHeader().setStretchLastSection(True)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self._log_pending_count = 0
        self.log_new_label = QtWidgets.QLabel("")
        self.log_new_label.setObjectName("statusStrip")
        self.log_new_label.hide()
        self.log_follow_button = QtWidgets.QPushButton("Follow latest")
        self.log_follow_button.setObjectName("secondaryAction")
        self.log_follow_button.clicked.connect(self._follow_latest_log)
        self.log_follow_button.hide()
        log_page = QtWidgets.QWidget()
        log_layout = QtWidgets.QVBoxLayout(log_page)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_toolbar = QtWidgets.QHBoxLayout()
        log_toolbar.addWidget(self.log_new_label)
        log_toolbar.addStretch(1)
        log_toolbar.addWidget(self.log_follow_button)
        log_layout.addLayout(log_toolbar)
        log_layout.addWidget(self.log_view)
        self.measurement_view = QtWidgets.QPlainTextEdit()
        self.measurement_view.setReadOnly(True)
        self.measurement_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.camera_view = self._diagnostic_text_view()
        self.pixhawk_view = self._diagnostic_text_view()
        self.gnss_view = self._diagnostic_text_view()
        self.performance_view = self._diagnostic_text_view()
        self.tabs.addTab(self.system_table, "System")
        self.tabs.addTab(self.camera_view, "Camera")
        self.tabs.addTab(self.pixhawk_view, "Pixhawk")
        self.tabs.addTab(self.gnss_view, "GNSS")
        self.tabs.addTab(log_page, "Log")
        self.tabs.addTab(self.measurement_view, "Measurement")
        self.tabs.addTab(self.performance_view, "Performance")
        layout.addWidget(self.tabs)

    @staticmethod
    def _diagnostic_text_view() -> QtWidgets.QPlainTextEdit:
        view = QtWidgets.QPlainTextEdit()
        view.setReadOnly(True)
        view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        return view

    def select_tab_for_context(self, context_id: str) -> None:
        tab_indexes = {
            "diagnostics.camera": 1,
            "diagnostics.pixhawk": 2,
            "diagnostics.gnss": 3,
            "diagnostics.logs": 4,
            "diagnostics.measurement": 5,
            "diagnostics.performance": 6,
        }
        self.tabs.setCurrentIndex(tab_indexes.get(context_id, 0))

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        refreshers = (
            self._refresh_system,
            self._refresh_camera,
            self._refresh_pixhawk,
            self._refresh_gnss,
            self._refresh_log,
            self._refresh_measurement,
            self._refresh_performance,
        )
        index = self.tabs.currentIndex()
        if 0 <= index < len(refreshers):
            refreshers[index](snapshot)

    def _refresh_system(self, snapshot: dict[str, RuntimeStatus]) -> None:
        rows = tuple(
            (
                status.key,
                status.state,
                status.severity.value,
                status.source,
                "--" if status.age_s is None else f"{status.age_s:.2f}s",
                status.detail,
            )
            for status in snapshot.values()
        )
        _sync_table_rows(self.system_model, rows)

    def _refresh_log(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        backend = getattr(self.backend, "diagnostic_text", None)
        if callable(backend):
            self._set_log_text(str(backend()))

    def _follow_latest_log(self) -> None:
        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        self._log_pending_count = 0
        self.log_new_label.clear()
        self.log_new_label.hide()
        self.log_follow_button.hide()

    def _set_log_text(self, text: str) -> None:
        if text == self.log_view.toPlainText():
            return
        scrollbar = self.log_view.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 4
        old_line_count = self.log_view.blockCount()
        old_scroll_value = scrollbar.value()
        self.log_view.setPlainText(text)
        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())
            self._log_pending_count = 0
            self.log_new_label.clear()
            self.log_new_label.hide()
            self.log_follow_button.hide()
        else:
            scrollbar.setValue(min(old_scroll_value, scrollbar.maximum()))
            added_lines = max(1, self.log_view.blockCount() - old_line_count)
            self._log_pending_count += added_lines
            self.log_new_label.setText(f"New logs: {self._log_pending_count}")
            self.log_new_label.show()
            self.log_follow_button.show()

    def _refresh_camera(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        config = evidence.detection_config()
        detection_lines = [
            "Detection configuration (AppSettings / existing controls):",
            f"weights={config.get('weights_path', '--')}",
            f"device={config.get('device', '--')} input={config.get('input_size_px', '--')} px "
            f"conf/IoU={config.get('confidence', '--')}/{config.get('iou', '--')}",
            f"gates={', '.join(str(item) for item in config.get('gate_enabled', ()) or ()) or 'OFF'}",
        ]
        target_lines = ["Target projection (bbox fields are image pixels u/v):"]
        for side in ("left", "right"):
            target = evidence.detection(side)
            target_lines.append(
                f"{side}: {target.get('runtime_state', 'NO TARGET')} class="
                f"{_display_class(target)} "
                f"conf={_display_number(target.get('confidence'), digits=3)} "
                f"age={_display_number(target.get('age_s'), digits=2, suffix='s')}"
            )
        text = self._status_text(
            snapshot,
            ("camera.left", "camera.right", "detection", "synchronization"),
        )
        self._set_diagnostic_text(
            self.camera_view,
            "\n\n".join((text, "\n".join(detection_lines + target_lines))),
        )

    def _refresh_pixhawk(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        attitude_lines = ["Attitude source projection (orientation only; PX4 body FRD → world NED):"]
        for side in ("left", "right"):
            data = evidence.attitude(side)
            attitude_lines.append(
                f"{side}: state={data.get('state')} rpy_deg="
                f"{_display_number(data.get('roll_deg'))}/"
                f"{_display_number(data.get('pitch_deg'))}/"
                f"{_display_number(data.get('yaw_deg'))}; "
                f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}; "
                f"px4_t={_display_number(data.get('px4_timestamp_s'), digits=6)}; "
                "transport=MAVLink"
            )
        text = self._status_text(snapshot, ("pixhawk.left", "pixhawk.right"))
        self._set_diagnostic_text(self.pixhawk_view, "\n\n".join((text, "\n".join(attitude_lines))))

    def _refresh_gnss(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        baseline = evidence.baseline()
        operation = evidence.gnss_operation()
        base_serial = operation.get("base_serial") if isinstance(operation.get("base_serial"), dict) else {}
        survey = operation.get("survey_in") if isinstance(operation.get("survey_in"), dict) else {}
        ntrip = operation.get("ntrip") if isinstance(operation.get("ntrip"), dict) else {}
        lines = [
            # "gnss.ntrip" is the actually-registered status key (see
            # status_model.py's _refresh_ntrip) -- this used to ask for a
            # plain "gnss" key that was never registered, so the detailed
            # NTRIP status line (phase/GGA age/RTCM age/reconnects/error)
            # silently rendered as "NO DATA" here no matter what NTRIP was
            # actually doing.
            self._status_text(snapshot, ("gnss.base", "gnss.ntrip")),
            "Base operation (Base receiver / Survey-in / NTRIP / RTCM):",
            f"mode={operation.get('mode', 'UNKNOWN')} state={operation.get('state', 'UNKNOWN')} "
            f"serial={'CONNECTED' if base_serial.get('connected') else 'DISCONNECTED'} "
            f"device={base_serial.get('device') or '--'} discovery={operation.get('base_discovery_status', 'NOT RUN')}",
            f"survey active/valid={bool(survey.get('active'))}/{bool(survey.get('valid'))} "
            f"duration={survey.get('duration_s', 0)} s accuracy={_display_number(survey.get('mean_accuracy_m'), digits=4)} m",
            f"NTRIP={'CONNECTED' if ntrip.get('connected') else 'DISCONNECTED'} "
            f"caster={ntrip.get('caster_host') or '--'}:{ntrip.get('caster_port') or '--'}/"
            f"{ntrip.get('mountpoint') or '--'}; RTCM frames={operation.get('base_frames', 0)} "
            f"age={_display_number(operation.get('last_frame_age_s'), digits=2, suffix='s')}",
            "",
            "Rover receiver projection (WGS84 geodetic; not NED baseline):",
        ]
        for side in ("left", "right"):
            data = evidence.gps(side)
            lines.append(
                f"{side}: {data.get('rtk_label', data.get('state', 'NO DATA'))}; "
                f"fix={data.get('fix_type', '--')} sats={data.get('satellites', '--')} "
                f"eph/epv={_display_number(data.get('eph_m'))}/{_display_number(data.get('epv_m'))} m; "
                f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}; "
                f"lat/lon={_display_number(data.get('latitude_deg'), digits=7)}/"
                f"{_display_number(data.get('longitude_deg'), digits=7)}"
            )
        lines.extend(
            [
                "",
                "Baseline source projection (NED north/east/down metres):",
                f"state={baseline.get('state', 'NO DATA')} "
                f"N/E/D={_display_number(baseline.get('north_m'))}/"
                f"{_display_number(baseline.get('east_m'))}/"
                f"{_display_number(baseline.get('down_m'))}; "
                f"length={_display_number(baseline.get('length_m'))} m",
                "P1→P2 @ P1 body FRD [F,R,D]="
                f"state={baseline.get('p1_body_frd_state', 'WAITING')} "
                f"{_display_number(baseline.get('p1_body_frd_forward_m'))}/"
                f"{_display_number(baseline.get('p1_body_frd_right_m'))}/"
                f"{_display_number(baseline.get('p1_body_frd_down_m'))} m; "
                f"attitude_age={_display_number(baseline.get('p1_body_frd_attitude_age_s'), digits=2, suffix='s')}",
            ]
        )
        self._set_diagnostic_text(self.gnss_view, "\n".join(lines))

    def _refresh_measurement(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        attempt = getattr(panel, "_last_measurement_attempt", None)
        pair_delta = getattr(panel, "selected_runtime_frame_pair_delta_s", None)
        pair_basis = getattr(panel, "selected_runtime_frame_pair_basis", "") or "--"
        rtc_delta = getattr(panel, "selected_runtime_frame_pair_camera_rtc_delta_s", None)
        baseline = getattr(panel, "latest_rtk_baseline", None)
        body_frd_projection = getattr(panel, "latest_p1_body_frd_baseline", None)
        if baseline is None:
            baseline_text = "NO DATA"
        else:
            baseline_text = (
                "NED="
                f"{float(getattr(baseline, 'north_m', 0.0)):+.3f}/"
                f"{float(getattr(baseline, 'east_m', 0.0)):+.3f}/"
                f"{float(getattr(baseline, 'down_m', 0.0)):+.3f} m; "
                f"length={float(getattr(baseline, 'baseline_m', 0.0)):.3f} m; "
                f"quality={getattr(baseline, 'fix_quality', 'UNKNOWN')}"
            )
        if body_frd_projection is None:
            body_frd_text = "WAITING"
        else:
            body_frd_text = (
                f"{float(getattr(body_frd_projection, 'forward_m', 0.0)):+.3f}/"
                f"{float(getattr(body_frd_projection, 'right_m', 0.0)):+.3f}/"
                f"{float(getattr(body_frd_projection, 'down_m', 0.0)):+.3f} m"
            )
        lines = [
            f"status: {getattr(panel, 'last_distance_status', '--')}",
            f"quality: {getattr(panel, 'last_distance_quality', '--')}",
            f"sync_status: {getattr(panel, 'camera_sync_status', '--')}",
            f"sync_reason: {getattr(panel, 'camera_sync_reason', '--')}",
            f"selected_pair_delta_s: {pair_delta if pair_delta is not None else '--'}",
            f"selected_pair_basis: {pair_basis}",
            f"selected_pair_rtcp_ntp_delta_s: {rtc_delta if rtc_delta is not None else '--'}",
            f"rtk_baseline_ned_m: {baseline_text}",
            f"rtk_baseline_p1_body_frd_m: {body_frd_text}",
            f"measurement_frame: {MEASUREMENT_FRAME}",
            f"axis_contract: {MEASUREMENT_AXIS_CONTRACT}",
            f"world_frame: {WORLD_FRAME}",
            f"body_frame: {BODY_FRAME}",
            f"camera_frame: {CAMERA_FRAME}",
            f"baseline_translation_frame: {BASELINE_TRANSLATION_FRAME}",
            f"attempt: {attempt!r}",
        ]
        self._set_diagnostic_text(self.measurement_view, "\n".join(lines))

    def _refresh_performance(self, snapshot: dict[str, RuntimeStatus]) -> None:
        self._set_diagnostic_text(self.performance_view, self._status_text(snapshot, ("fps",)))

    def _status_text(self, snapshot: dict[str, RuntimeStatus], keys: tuple[str, ...]) -> str:
        return "\n".join(self._format_status(snapshot.get(key)) for key in keys)

    @staticmethod
    def _set_diagnostic_text(
        view: QtWidgets.QPlainTextEdit,
        text: str,
        *,
        move_to_end: bool = False,
    ) -> None:
        if text == view.toPlainText():
            return
        view.setPlainText(text)
        if move_to_end:
            view.moveCursor(QtGui.QTextCursor.End)

    @staticmethod
    def _format_status(status: RuntimeStatus | None) -> str:
        if status is None:
            return "NO DATA"
        age = "--" if status.age_s is None else f"{status.age_s:.2f}s"
        return (
            f"{status.label} | state={status.state} | severity={status.severity.value} | age={age}\n"
            f"From: {status.source} | Clock: {status.clock}\n"
            f"detail={status.detail}"
        )
