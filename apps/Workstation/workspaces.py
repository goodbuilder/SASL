from __future__ import annotations

import json
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
from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS,
    MEASUREMENT_METHOD_LABELS,
)
from Workstation.workstation_core.optical_pose_auto_select import (
    validate_horizon_suggestion_points,
    validate_mountain_suggestion_points,
)
from Workstation.workstation_core.network import candidate_network_role_labels
from Workstation.workstation_core.pixhawk_attitude import MAVLINK_EXPECTED_SYSTEM_IDS
from Workstation.workstation_core.runtime_camera_status import (
    decoder_presentation,
    format_camera_input_metrics,
)
from Workstation.workstation_core.px4_attitude_cal_manager import (
    board_rotation_label,
)
from Workstation.workstation_core.px4_calibration_groups import PX4_CALIBRATION_GROUPS
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    build_mag_selection_preview,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    MAG_PRIORITY_KEYS,
    MAG_PRIORITY_LEVELS,
    MAG_SLOTS,
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
    mag_priority_slot_key,
    mag_priority_display,
)
from Workstation.workstation_core.theme import (
    ACCENT,
    ERROR_TEXT,
    PANEL_BORDER,
    SUCCESS_TEXT,
)
from Workstation.workstation_core.stereo import DetectionObservation
from Workstation.workstation_core.settings import (
    DEFAULT_CALIBRATION_ROOT,
    PIXHAWK_SERIAL_TRANSPORTS,
    normalize_pixhawk_gnss_source,
    normalize_pixhawk_transport,
)
from Workstation.workstation_core.calibration_selection import (
    CALIBRATION_IMAGE_SUFFIXES,
    CALIBRATION_SELECTION_SCHEMA,
    empty_calibration_selection,
    load_calibration_selection_sidecar,
    portable_calibration_selection_paths,
    resolve_calibration_selection_paths,
)
from Workstation.workstation_core.calibration_timing import (
    INTRINSICS_ONLY,
    INVALID_METADATA,
    LEGACY_UNVERIFIED,
    load_calibration_pair_usage,
)
from Workstation.calibration_sessions import discover_calibration_sessions

from .components import EngineeringValue, StatusIndicator, WorkspaceHeader
from .evidence import EvidenceAdapter
from .protocol import evaluate_recording_protocol
from .status_model import RuntimeStatus, StatusModel, StatusSeverity
from .telemetry_plots import BaselineYawAlignmentPlot


SESSION_CATALOG_REFRESH_INTERVAL_S = 5.0
PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX = 420
def _status(model: StatusModel, key: str) -> RuntimeStatus | None:
    return model.record(key)


def _evidence(backend: object) -> EvidenceAdapter:
    adapter = getattr(backend, "evidence", None)
    if isinstance(adapter, EvidenceAdapter):
        return adapter
    return EvidenceAdapter(getattr(backend, "panel", backend))


def _panel_widget_text(panel: object, name: str) -> str:
    """Read a panel value that lives in a QLineEdit, falling back to a plain attribute."""

    widget = getattr(panel, f"{name}_edit", None)
    reader = getattr(widget, "text", None)
    if callable(reader):
        return str(reader()).strip()
    return str(getattr(panel, name, "") or "").strip()


def _panel_widget_value(panel: object, name: str, default: int) -> int:
    """Read a panel value that lives in a QSpinBox, falling back to a plain attribute."""

    widget = getattr(panel, f"{name}_spin", None)
    reader = getattr(widget, "value", None)
    if callable(reader):
        return int(reader())
    try:
        return int(getattr(panel, name, default))
    except (TypeError, ValueError):
        return default


def _display_number(value: object, *, digits: int = 3, suffix: str = "") -> str:
    if value in (None, ""):
        return "--"
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except (TypeError, ValueError):
        return f"{value}{suffix}"


def _display_signed_number(value: object, *, digits: int = 3, suffix: str = "") -> str:
    if value in (None, ""):
        return "--"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(number):
        return "--"
    return f"{number:+.{digits}f}{suffix}"


# Side -> column label for Pixhawk role columns; the sysid map is derived so
# the labels can never drift from the MAVLink identity contract.
_PIXHAWK_SIDE_LABELS = (("left", "P1"), ("right", "P2"), ("drone", "Drone"))
_PIXHAWK_SYSTEM_ID_LABELS = {
    MAVLINK_EXPECTED_SYSTEM_IDS[side]: label for side, label in _PIXHAWK_SIDE_LABELS
}


def _network_role_column_text(labels: Sequence[str]) -> str:
    """Render the saved roles one NIC row satisfies for a candidate table."""

    if len(labels) > 1:
        return f"{', '.join(labels)} (shared NIC)"
    return labels[0] if labels else "--"


def _apply_network_role_discovery_status(
    status_widget: object,
    connection: Mapping[str, object],
    *,
    default_status: str = "NOT RUN",
) -> tuple[str, str]:
    """Show discovery status, overridden while a network-role operation runs.

    Returns the role state and its detail so callers can add their own feedback.
    """

    role_status = connection.get("network_role_status")
    role_status = role_status if isinstance(role_status, Mapping) else {}
    role_state = str(role_status.get("state") or "IDLE")
    detail = str(role_status.get("detail") or "")
    discovery_status = str(connection.get("status", default_status) or default_status)
    if role_state != "IDLE":
        discovery_status = f"NETWORK ROLE {role_state}"
    status_widget.set_value(
        discovery_status,
        source=detail if role_state != "IDLE" else "",
        valid=False if discovery_status.startswith("ERROR") else None,
    )
    return role_state, detail


def _display_compass_deg(value: object) -> str:
    if value in (None, ""):
        return "--"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(number):
        return "--"
    normalized = number % 360.0
    if round(normalized, 2) >= 360.0:
        normalized = 0.0
    return f"{normalized:06.2f}"


def _display_signed_deg(value: object) -> str:
    if value in (None, ""):
        return "--"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(number):
        return "--"
    wrapped = (number + 180.0) % 360.0 - 180.0
    if abs(wrapped) < 0.005:
        wrapped = 0.0
    if round(wrapped, 2) >= 180.0:
        wrapped = -180.0
    return f"{wrapped:+07.2f}"


def _display_magnetometer_px4_yaw(row: Mapping[str, object]) -> str:
    """Render PX4 yaw only when the evidence row is exactly selected."""

    value = _display_signed_deg(row.get("px4_yaw_deg"))
    if value != "--":
        return value
    return {
        "WAITING": "대기",
        "STALE": "STALE",
        "ACTIVE ID WAITING": "active ID 대기",
        "IDENTITY UNKNOWN": "ID 확인 대기",
        "NOT SELECTED": "미선택",
    }.get(str(row.get("px4_yaw_state", "WAITING")), "--")


def _display_magnetometer_corrected_yaw(row: Mapping[str, object]) -> str:
    value = _display_signed_deg(row.get("corrected_yaw_deg"))
    if value != "--":
        return value
    return {
        "WAITING": "대기",
        "PARAM WAITING": "PARAM 대기",
        "PARAM INCOMPLETE": "PARAM 불완전",
        "POWER COMP PENDING": "전류 보정 대기",
        "BOARD ROTATION WAITING": "보드 회전 대기",
        "IDENTITY UNKNOWN": "ID 확인 대기",
    }.get(str(row.get("corrected_yaw_state", "WAITING")), "--")


def _magnetometer_yaw_summary(row: Mapping[str, object]) -> str:
    """Build a compact Inspector summary for raw and corrected yaw."""

    raw = _display_signed_deg(row.get("raw_yaw_deg"))
    corrected = _display_magnetometer_corrected_yaw(row)
    return f"RAW={raw}° · Corrected={corrected}"


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
        layout.addWidget(WorkspaceHeader("Overview", "Connect → verify → measure → auto-save"))

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
        self.session_value = EngineeringValue("Storage")
        self.measurement_value = EngineeringValue("Result")
        self.contract_value = EngineeringValue("Output")
        evidence_layout.addWidget(self.session_value, 0, 0)
        evidence_layout.addWidget(self.measurement_value, 1, 0)
        evidence_layout.addWidget(self.contract_value, 2, 0)
        evidence_layout.setColumnStretch(0, 1)
        layout.addWidget(evidence_group)

        methods_group = QtWidgets.QGroupBox("Measurement methods · read-only")
        methods_layout = QtWidgets.QGridLayout(methods_group)
        methods_layout.setContentsMargins(6, 6, 6, 6)
        methods_layout.setSpacing(5)
        self.measurement_method_cards: dict[str, QtWidgets.QPushButton] = {}
        for index, method_id in enumerate(MEASUREMENT_METHOD_IDS):
            card = QtWidgets.QPushButton(
                f"{MEASUREMENT_METHOD_LABELS[method_id]}\nBLOCKED · V0 / B0 / F0"
            )
            card.setObjectName(f"overviewMethodCard_{method_id}")
            card.setProperty("navigationOnly", True)
            card.setMinimumHeight(54)
            card.setStyleSheet("text-align:left; padding:6px;")
            card.clicked.connect(
                lambda _checked=False: self.open_context.emit(
                    "processing.measurement"
                )
            )
            self.measurement_method_cards[method_id] = card
            methods_layout.addWidget(card, index // 2, index % 2)
        layout.addWidget(methods_group)

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
        storage_reader = getattr(panel, "data_storage_snapshot", None)
        try:
            storage = storage_reader() if callable(storage_reader) else {}
        except Exception:
            storage = {}
        storage_state = str(storage.get("state") or "UNAVAILABLE")
        self.session_value.set_value(
            storage_state,
            source=str(storage.get("root") or "measurement_data_root"),
            valid=True if storage_state == "OK" else False if storage_state == "UNAVAILABLE" else None,
        )
        measurement_status = snapshot.get("measurement")
        measurement_is_current_valid = bool(
            measurement_status and measurement_status.state == "VALID"
        )
        self.measurement_value.set_value(
            str(getattr(panel, "last_distance_status", "Waiting for measurement") or ""),
            source="MeasurementAttempt/controller result",
            frame=MEASUREMENT_FRAME,
            age_s=measurement_status.age_s if measurement_status else None,
            valid=(
                True
                if measurement_is_current_valid
                else False
                if measurement_status
                and measurement_status.state in {"ERROR", "REJECTED"}
                else None
            ),
        )
        self.contract_value.set_value(
            f"{MEASUREMENT_AXIS_CONTRACT} · metres",
            frame=MEASUREMENT_FRAME,
            source="coordinate_contract.py",
        )
        reader = getattr(self.backend, "measurement_methods_snapshot", None)
        try:
            method_snapshot = reader() if callable(reader) else {}
        except Exception:
            method_snapshot = {}
        catalog = method_snapshot.get("method_catalog") if isinstance(method_snapshot, Mapping) else None
        catalog_rows = {
            str(row.get("method_id") or ""): dict(row)
            for row in catalog or ()
            if isinstance(row, Mapping)
        }
        for method_id, card in self.measurement_method_cards.items():
            row = catalog_rows.get(method_id, {})
            state = str(row.get("runtime_status") or "BLOCKED")
            counters = row.get("result_counters")
            counters = counters if isinstance(counters, Mapping) else {}
            blocker = str(row.get("primary_blocker") or "")
            extra = int(row.get("additional_blocker_count") or 0)
            blocker_text = blocker + (f" · +{extra} more" if extra else "")
            primary = "PRIMARY · " if method_id == "pixhawk_rtk_live" else ""
            card.setText(
                f"{primary}{MEASUREMENT_METHOD_LABELS[method_id]}\n"
                f"{state} · V{int(counters.get('VALID', 0) or 0)} / "
                f"B{int(counters.get('BLOCKED', 0) or 0)} / "
                f"F{int(counters.get('FAILED', 0) or 0)}"
                + (f"\n{_compact_text(blocker_text, limit=72)}" if blocker_text else "")
            )
            card.setToolTip(
                blocker_text
                or "Click to open read-only Measurement details."
            )


class ExperimentWorkspace(QtWidgets.QWidget):
    """Guided view of the field protocol using existing owner evidence."""

    open_context = QtCore.pyqtSignal(str)

    def __init__(self, backend: object, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        layout = QtWidgets.QVBoxLayout(self)
        self._layout = layout
        self._run_preparation_controller: object | None = None
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
        self._protocol_group = protocol_group
        contract = QtWidgets.QLabel(
            "Fix evidence is numeric PX4/MAVLink fix_type; RTK_FIXED=6 is reported as a ratio and is not assumed for every run. "
            "Raw baseline remains NED [north,east,down]; alignment view is P1 body FRD [forward,right,down]."
        )
        contract.setObjectName("compactContract")
        contract.setWordWrap(True)
        layout.addWidget(contract)
        layout.addStretch(1)

    def attach_run_preparation(
        self,
        preflight_group: QtWidgets.QWidget,
        controller: object,
    ) -> None:
        """Place the owner-backed preflight controls in the Protocol workflow."""

        index = self._layout.indexOf(self._protocol_group)
        self._layout.insertWidget(index, preflight_group)
        self._run_preparation_controller = controller

    @staticmethod
    def _state(status: RuntimeStatus | None) -> str:
        return "UNKNOWN" if status is None else str(status.state or "UNKNOWN").upper()

    def _set_step(self, number: int, state: str, detail: str) -> None:
        self.step_states[number].setText(state)
        self.step_details[number].setText(detail)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        refresh_preparation = getattr(
            self._run_preparation_controller,
            "refresh_run_preparation",
            None,
        )
        if callable(refresh_preparation):
            refresh_preparation(snapshot)
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
        gnss_state = self._state(snapshot.get("gnss"))
        gps_present = (
            gnss_state in {"FIX", "FLOAT", "NO FIX"}
            and all(data.get("fix_type") is not None for data in gps.values())
        )
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
        elif recording_state == "ACTIVE" and bool(
            getattr(panel, "_automatic_session_recording", False)
        ):
            save_state, save_detail = "AUTO", "session is recording with Detection"
        elif has_data:
            save_state, save_detail = "READY", "session details are handled automatically"
        else:
            save_state, save_detail = "WAIT", "session starts with Detection"
        self._set_step(11, save_state, save_detail)


class _ElidedStatusLabel(QtWidgets.QLabel):
    """One stable-height status line with full evidence in its tooltip."""

    def __init__(self, text: str = "", parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._full_text = ""
        self.setWordWrap(False)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.setFixedHeight(max(22, self.fontMetrics().lineSpacing() + 8))
        self.set_full_text(text)

    def set_full_text(self, text: object) -> None:
        self._full_text = " ".join(str(text or "").split())
        self.setToolTip(self._full_text)
        self._refresh_elision()

    def _refresh_elision(self) -> None:
        width = max(1, self.contentsRect().width())
        super().setText(
            self.fontMetrics().elidedText(
                self._full_text,
                QtCore.Qt.ElideRight,
                width,
            )
        )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._refresh_elision()


class CameraPreview(QtWidgets.QFrame):
    IMAGE_ASPECT_RATIO = 16.0 / 9.0

    def __init__(
        self,
        side: str,
        parent: QtWidgets.QWidget | None = None,
        *,
        crop_to_fill: bool = True,
        lock_chrome_height: bool = False,
        lock_image_aspect: bool = False,
        active_label: str = "LIVE",
        empty_image_text: str = "NO VIDEO",
        maximum_image_height: int | None = None,
    ) -> None:
        super().__init__(parent)
        self.side = side
        self.crop_to_fill = bool(crop_to_fill)
        self.lock_chrome_height = bool(lock_chrome_height)
        self.lock_image_aspect = bool(lock_image_aspect)
        self.active_label = str(active_label)
        self.empty_image_text = str(empty_image_text)
        self.maximum_image_height = (
            None
            if maximum_image_height is None
            else max(1, int(maximum_image_height))
        )
        self.setObjectName("cameraPreview")
        self.setMinimumSize(0, 0)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(3)
        title = QtWidgets.QLabel("Left camera" if side == "left" else "Right camera")
        title.setObjectName("cameraPreviewTitle")
        self.image = QtWidgets.QLabel(self.empty_image_text)
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.image.setMinimumSize(0, 0)
        self.image.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Fixed
            if self.lock_image_aspect
            else QtWidgets.QSizePolicy.Expanding,
        )
        self.image.setObjectName("cameraPreviewImage")
        self.meta = QtWidgets.QLabel("WAITING")
        self.meta.setObjectName("cameraPreviewMeta")
        self.meta.setWordWrap(not self.lock_chrome_height)
        self.meta.setMinimumWidth(0)
        if self.lock_chrome_height:
            # Measurement receives live metadata and detection labels while
            # the image surface is being painted.  A wrapping metadata label
            # changes its sizeHint when a class name or status changes, which
            # lets the parent splitter resize the camera surface on every
            # update.  Keep one reserved line and expose the full text in the
            # tooltip so content changes cannot move the image.
            self.meta.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            self._locked_meta_height = max(1, self.meta.fontMetrics().lineSpacing())
            self.meta.setFixedHeight(self._locked_meta_height)
        else:
            self.meta.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        layout.addWidget(title)
        layout.addWidget(self.image, 0 if self.lock_image_aspect else 1)
        layout.addWidget(self.meta)
        if self.lock_image_aspect:
            layout.addStretch(1)
        self._image: QtGui.QImage | None = None
        self._detection: DetectionObservation | None = None
        self._metadata: object = {}

    def hasHeightForWidth(self) -> bool:  # noqa: N802 - Qt override.
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 - Qt override.
        content_width = max(0, int(width) - self.layout().contentsMargins().left() - self.layout().contentsMargins().right())
        image_height = int(round(content_width / self.IMAGE_ASPECT_RATIO))
        if self.maximum_image_height is not None:
            image_height = min(image_height, self.maximum_image_height)
        title = self.findChild(QtWidgets.QLabel, "cameraPreviewTitle")
        chrome_height = title.sizeHint().height() if title is not None else 0
        meta_visible = not self.meta.isHidden()
        if meta_visible:
            chrome_height += (
                self._locked_meta_height
                if self.lock_chrome_height
                else self.meta.sizeHint().height()
            )
        spacing_count = 2 if meta_visible else 1
        return image_height + chrome_height + self.layout().spacing() * spacing_count + self.layout().contentsMargins().top() + self.layout().contentsMargins().bottom()

    def set_frame(self, image: QtGui.QImage, metadata: object) -> None:
        self._image = image
        if self.isVisible():
            self._render()
        self._set_metadata(metadata, live=True)

    def clear(self) -> None:
        """Clear the displayed frame when the camera Preview is stopped."""

        self._image = None
        self._detection = None
        self._metadata = {}
        if self.isVisible():
            self._render()
        self._set_metadata({}, live=False)

    def set_detection(self, detection: DetectionObservation | None) -> None:
        """Keep the latest image-space bbox for this display preview.

        Detection coordinates remain in the source image pixel contract
        (x1/y1/x2/y2, or u/v).  Only the copy rendered into the widget is
        scaled for presentation; no camera/world-frame conversion occurs.
        """

        self._detection = detection
        if self.isVisible():
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
        parts = [self.active_label if live else "WAITING"]
        if self.active_label != "LIVE":
            if isinstance(metadata, Mapping):
                path = str(metadata.get("path") or "")
                if path:
                    parts.append(Path(path).name)
                width = metadata.get("width")
                height = metadata.get("height")
                if width and height:
                    parts.append(f"{width}×{height}")
            return " | ".join(parts)
        if self._detection is None:
            parts.append("Class: --")
        else:
            class_name = self._detection.class_name or f"#{self._detection.class_id}"
            parts.append(f"Class: {class_name}")
        if (
            isinstance(metadata, Mapping)
            and bool(metadata.get("media_integrity_unverified"))
        ):
            parts.append("INPUT UNVERIFIED")
        return " | ".join(parts)

    def _render(self) -> None:
        if self._image is None or self._image.isNull():
            self.image.setText(self.empty_image_text)
            self.image.setPixmap(QtGui.QPixmap())
            return
        target_size = self.image.size()
        source_pixmap = QtGui.QPixmap.fromImage(self._image)
        if self.crop_to_fill:
            scaled = source_pixmap.scaled(
                target_size, QtCore.Qt.KeepAspectRatioByExpanding, QtCore.Qt.SmoothTransformation
            )
            crop_x = max(0, (scaled.width() - target_size.width()) // 2)
            crop_y = max(0, (scaled.height() - target_size.height()) // 2)
            pixmap = scaled.copy(crop_x, crop_y, target_size.width(), target_size.height())
        else:
            scaled = source_pixmap.scaled(
                target_size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )
            offset_x = max(0, (target_size.width() - scaled.width()) // 2)
            offset_y = max(0, (target_size.height() - scaled.height()) // 2)
            pixmap = QtGui.QPixmap(target_size)
            pixmap.fill(QtCore.Qt.black)
            painter = QtGui.QPainter(pixmap)
            painter.drawPixmap(offset_x, offset_y, scaled)
            painter.end()
            # _draw_detection_overlay uses display = source * scale - crop.
            crop_x = -offset_x
            crop_y = -offset_y
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

    def _lock_image_height(self, preview_width: int) -> None:
        if not self.lock_image_aspect:
            return
        margins = self.layout().contentsMargins()
        content_width = max(0, int(preview_width) - margins.left() - margins.right())
        desired_height = int(round(content_width / self.IMAGE_ASPECT_RATIO))
        if self.maximum_image_height is not None:
            desired_height = min(desired_height, self.maximum_image_height)
        if desired_height > 0 and self.image.height() != desired_height:
            self.image.setFixedHeight(desired_height)

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
        self._lock_image_height(event.size().width())
        super().resizeEvent(event)
        if self.isVisible():
            self._render()

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802
        super().showEvent(event)
        self._lock_image_height(self.width())
        self._render()


class CameraPreviewPair(QtWidgets.QSplitter):
    """Aspect-aware pair container for the calibration camera previews."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(QtCore.Qt.Horizontal, parent)
        self._equalizing_sizes = False
        self.setChildrenCollapsible(False)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.splitterMoved.connect(self._restore_equal_sizes)

    def _restore_equal_sizes(self, *_args: object) -> None:
        """Keep both calibration preview surfaces at the same display size."""

        if self._equalizing_sizes or self.orientation() != QtCore.Qt.Horizontal:
            return
        if self.count() != 2:
            return
        available_width = sum(self.sizes())
        if available_width <= 0:
            available_width = max(0, self.width() - self.handleWidth())
        left_width = available_width // 2
        equal_sizes = [left_width, available_width - left_width]
        if self.sizes() == equal_sizes:
            return
        self._equalizing_sizes = True
        try:
            self.setSizes(equal_sizes)
        finally:
            self._equalizing_sizes = False

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

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt override.
        super().resizeEvent(event)
        self._restore_equal_sizes()
        desired_height = self.heightForWidth(event.size().width())
        if desired_height > 0 and self.minimumHeight() != desired_height:
            self.setMinimumHeight(desired_height)

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
        self.decoder_values: dict[str, EngineeringValue] = {}
        for col, side in enumerate(("left", "right")):
            value = EngineeringValue(f"Decoder {side.upper()}")
            self.decoder_values[side] = value
            facts_layout.addWidget(value, 1, col)
        details_layout.addWidget(facts)

        stream_metrics_group = QtWidgets.QGroupBox("RTP / decode")
        stream_metrics_layout = QtWidgets.QVBoxLayout(stream_metrics_group)
        stream_metrics_layout.setContentsMargins(6, 6, 6, 6)
        stream_metrics_layout.setSpacing(2)
        self.stream_metric_labels: dict[str, QtWidgets.QLabel] = {}
        for side in ("left", "right"):
            label = QtWidgets.QLabel(format_camera_input_metrics(side, {}))
            label.setObjectName("statusStrip")
            label.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
            label.setWordWrap(False)
            label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
            label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            label.setFixedHeight(label.fontMetrics().lineSpacing() * 2 + 2)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.stream_metric_labels[side] = label
            stream_metrics_layout.addWidget(label)
        stream_metrics_group.setToolTip(
            "GStreamer worker의 최근 5초 rolling rate입니다. "
            "RTP missing은 sequence gap으로 추정한 누락 패킷 수입니다."
        )
        self.stream_metrics_group = stream_metrics_group
        details_layout.addWidget(stream_metrics_group)

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
            ("Kind", "Address", "Port", "Status", "Interface", "Host IP", "Assigned"),
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
        for target, label in (("nic", "Assign Camera NIC"), ("left", "Assign CAM L"), ("right", "Assign CAM R")):
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
        self._network_role_locked = False

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

        details_layout.addWidget(self._build_image_format_group(invoke_command))
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

    # -- Image Format group ---------------------------------------------
    #
    # CPTR emits one file per enabled SIxx save-format toggle
    # (SIRJ=radiometric, SISR=super-resolution, SIHR=HR, SIVI=OZ visible).
    # Left/right were found with different toggles on, which breaks any
    # "one file per capture" assumption downstream (e.g. the calibration OZ
    # FTP sync). This group makes that state visible and lets the operator
    # force both cameras to a known state; the calibration OZ capture path
    # additionally forces it automatically before every trigger.

    IMAGE_FORMAT_TOGGLES = (
        ("SIRJ", "GIRJ", "Radiometric"),
        ("SISR", "GISR", "Super-res"),
        ("SIHR", "GIHR", "HR"),
        ("SIVI", "GIVI", "Visible (OZ)"),
    )

    def _build_image_format_group(
        self, invoke_command: Callable[[str], bool] | None
    ) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Image Format")
        layout = QtWidgets.QVBoxLayout(group)

        status_layout = QtWidgets.QGridLayout()
        status_layout.setHorizontalSpacing(10)
        self.image_format_values: dict[str, EngineeringValue] = {}
        get_commands = tuple(get_cmd for _set_cmd, get_cmd, _label in self.IMAGE_FORMAT_TOGGLES) + ("GILC",)
        for row, side in enumerate(("left", "right")):
            for col, get_cmd in enumerate(get_commands):
                value = EngineeringValue(f"{side.title()} {get_cmd}")
                self.image_format_values[f"{side}_{get_cmd}"] = value
                status_layout.addWidget(value, row, col)
        layout.addLayout(status_layout)

        toggle_layout = QtWidgets.QHBoxLayout()
        self.image_format_checkboxes: dict[str, QtWidgets.QCheckBox] = {}
        for set_cmd, _get_cmd, label in self.IMAGE_FORMAT_TOGGLES:
            checkbox = QtWidgets.QCheckBox(label)
            checkbox.setToolTip(
                f"체크: 양쪽 카메라에 {set_cmd} TRUE 전송. 해제: 양쪽에 {set_cmd} FALSE 전송."
            )
            checkbox.toggled.connect(
                lambda checked, set_cmd=set_cmd: self._on_image_format_checkbox_toggled(set_cmd, checked)
            )
            self.image_format_checkboxes[set_cmd] = checkbox
            toggle_layout.addWidget(checkbox)
        toggle_layout.addStretch(1)
        layout.addLayout(toggle_layout)

        actions_layout = QtWidgets.QHBoxLayout()
        refresh_button = _command_button(
            "새로고침", "camera.refresh_image_format_status", invoke_command
        )
        refresh_button.setToolTip("좌우 GIVI/GIRJ/GISR/GIHR/GILC 상태를 다시 조회합니다.")
        self.operation_buttons["camera.refresh_image_format_status"] = refresh_button
        actions_layout.addWidget(refresh_button)
        preset_button = _command_button(
            "OZ 전용 초기화", "camera.apply_oz_only_image_format", invoke_command
        )
        preset_button.setToolTip(
            "SIRJ FALSE, SISR FALSE, SIHR FALSE, SIVI TRUE를 양쪽 카메라에 전송합니다."
        )
        self.operation_buttons["camera.apply_oz_only_image_format"] = preset_button
        actions_layout.addWidget(preset_button)
        actions_layout.addStretch(1)
        layout.addLayout(actions_layout)
        return group

    def _on_image_format_checkbox_toggled(self, set_command: str, checked: bool) -> None:
        setter = getattr(self.backend, "camera_set_image_format", None)
        if not callable(setter):
            return
        setter(set_command, checked)

    def _sync_image_format_status(self) -> None:
        values = getattr(self, "image_format_values", None)
        if values is None:
            return
        status = _evidence(self.backend).image_format_status()
        for side in ("left", "right"):
            responses = status.get(side) if isinstance(status.get(side), dict) else {}
            for _set_cmd, get_cmd, _label in self.IMAGE_FORMAT_TOGGLES:
                widget = values.get(f"{side}_{get_cmd}")
                if widget is not None:
                    widget.set_value(responses.get(get_cmd))
            gilc_widget = values.get(f"{side}_GILC")
            if gilc_widget is not None:
                gilc_widget.set_value(responses.get("GILC"))
        left = status.get("left") if isinstance(status.get("left"), dict) else {}
        right = status.get("right") if isinstance(status.get("right"), dict) else {}
        for set_cmd, get_cmd, _label in self.IMAGE_FORMAT_TOGGLES:
            checkbox = self.image_format_checkboxes.get(set_cmd)
            if checkbox is None:
                continue
            both_true = str(left.get(get_cmd, "")).strip().upper() == "TRUE" and (
                str(right.get(get_cmd, "")).strip().upper() == "TRUE"
            )
            checkbox.blockSignals(True)
            checkbox.setChecked(both_true)
            checkbox.blockSignals(False)

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

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802
        super().showEvent(event)
        # Navigation swaps the page without necessarily resizing the widget;
        # size the preview once it becomes visible so the first paint cannot
        # inherit an unconstrained splitter height from the scroll area.
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
        self._sync_image_format_status()
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
        stream_stats_by_side = getattr(panel, "camera_stream_stats", {}) or {}
        if not isinstance(stream_stats_by_side, Mapping):
            stream_stats_by_side = {}
        for side, label in self.stream_metric_labels.items():
            side_stats = stream_stats_by_side.get(side, {})
            side_stats = dict(side_stats) if isinstance(side_stats, Mapping) else {}
            metrics_text = format_camera_input_metrics(side, side_stats)
            if label.text() != metrics_text:
                label.setText(metrics_text)
            window_s = side_stats.get("rtp_metrics_window_s", 5.0)
            try:
                window_text = f"{max(1.0, float(window_s)):.0f}s"
            except (TypeError, ValueError):
                window_text = "5s"
            label.setToolTip(
                f"최근 {window_text} rolling rate\n"
                "RTP missing = sequence gap으로 추정한 누락 패킷\n"
                f"누적 RTP {side_stats.get('rtp_packets_received_total', '--')} pkt, "
                f"누적 missing {side_stats.get('rtp_missing_packets_total', '--')} pkt"
            )
        camera_recovery_allowed = not recording_locked or bool(
            getattr(panel, "_automatic_session_recording", False)
        )
        active_decoders = getattr(panel, "active_stream_decoders", {})
        if not isinstance(active_decoders, Mapping):
            active_decoders = {}
        decoder_combo = getattr(panel, "decoder_combo", None)
        requested_decoder = (
            decoder_combo.currentData()
            if decoder_combo is not None and callable(getattr(decoder_combo, "currentData", None))
            else getattr(getattr(panel, "settings", None), "rtsp_decoder", "auto")
        )
        fallback_reasons = getattr(panel, "decoder_fallback_reasons", {}) or {}
        runtime_overrides = getattr(panel, "_runtime_decoder_overrides", {}) or {}
        for side, value in self.decoder_values.items():
            decoder = decoder_presentation(
                requested_mode=requested_decoder,
                active_decoder=active_decoders.get(side),
                runtime_override=(
                    runtime_overrides.get(side)
                    if isinstance(runtime_overrides, Mapping) and side in runtime_overrides
                    else getattr(panel, "_runtime_decoder_override", None)
                ),
                fallback_reason=(fallback_reasons.get(side) if isinstance(fallback_reasons, Mapping) else ""),
            )
            value.set_value(
                decoder.value_text,
                source=decoder.detail,
            )
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
                button.setEnabled(camera_recovery_allowed and not rtsp_operation_active)
        connection = evidence.camera_connection_candidates()
        self._network_role_locked = bool(connection.get("network_role_active", False))
        role_state, role_detail = _apply_network_role_discovery_status(
            self.discovery_status, connection
        )
        if role_state != "IDLE":
            self.candidate_feedback.setText(f"Network role: {role_state} · {role_detail}")
        candidates = connection.get("candidates")
        candidates = tuple(row for row in candidates if isinstance(row, dict)) if isinstance(candidates, (tuple, list)) else ()
        network_roles = connection.get("network_roles")
        network_roles = network_roles if isinstance(network_roles, Mapping) else {}

        def assigned_roles(row: Mapping[str, object]) -> str:
            if not str(row.get("interface") or ""):
                return "--"
            return _network_role_column_text(candidate_network_role_labels(row, network_roles))
        candidate_signature = tuple(
            (
                *tuple(
                    row.get(key, "")
                    for key in ("kind", "address", "port", "status", "interface", "host_ip")
                ),
                assigned_roles(row),
            )
            for row in candidates
        )
        if candidate_signature != self._candidate_signature:
            selected = self._selected_candidate_identity()
            self.candidate_model.removeRows(0, self.candidate_model.rowCount())
            for row in candidates:
                _append_table_row(
                    self.candidate_model,
                    (
                        *tuple(
                            row.get(key, "")
                            for key in ("kind", "address", "port", "status", "interface", "host_ip")
                        ),
                        assigned_roles(row),
                    ),
                )
            self._candidate_signature = candidate_signature
            if selected:
                for row_index, row in enumerate(candidates):
                    if (str(row.get("kind", "")), str(row.get("address", ""))) == selected:
                        self.candidate_table.selectRow(row_index)
                        break
        self.operation_buttons["camera.discover_host_ip"].setEnabled(
            not recording_locked
            and not bool(connection.get("discovery_active", False))
            and not bool(connection.get("network_role_active", False))
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
        self.candidate_buttons["nic"].setEnabled(
            not self._recording_locked and not self._network_role_locked and kind == "NIC"
        )
        side_enabled = kind in {"RTSP", "WIRIS"}
        self.candidate_buttons["left"].setEnabled(
            not self._recording_locked and not self._network_role_locked and side_enabled
        )
        self.candidate_buttons["right"].setEnabled(
            not self._recording_locked and not self._network_role_locked and side_enabled
        )

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
        self.candidate_feedback.setText(
            f"Requested row {row + 1} for {target.upper()} · follow network role status"
        )
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
        self.schedule_status = QtWidgets.QLabel(
            "STUS schedule: MANUAL · manual seed required"
        )
        self.schedule_status.setObjectName("statusStrip")
        self.schedule_status.setWordWrap(True)
        self.schedule_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        sync_layout.addWidget(self.schedule_status)
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
            ("L/R RTCP Δt gate", double_spin("camera_sync_rtcp_pair_max_delta_ms", 0.0, 1000.0, 1, " ms")),
            ("SR stale", double_spin("camera_sync_max_sr_age_s", 0.1, 60.0, 2, " s")),
            ("SR-PC diagnostic", double_spin("camera_sync_max_pc_offset_ms", 0.0, 5000.0, 1, " ms")),
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
        # source panel (and its legacy time tab) stays hidden.  The STUS
        # controller continues a low-rate health poll after initial VERIFIED;
        # this bounded 4 Hz workspace refresh mirrors that source-backed state
        # without creating a second timing owner.
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
        schedule_reader = getattr(panel, "_stus_schedule_snapshot", None)
        schedule = schedule_reader() if callable(schedule_reader) else {}
        mode = str(schedule.get("mode") or "manual").upper()
        state = str(schedule.get("state") or "UNAVAILABLE")
        remaining = schedule.get("remaining_s")
        due_text = "--"
        try:
            remaining_s = max(0.0, float(remaining))
        except (TypeError, ValueError):
            pass
        else:
            due_text = f"{int(remaining_s) // 60:02d}:{int(remaining_s) % 60:02d}"
        frame_state = str(
            getattr(panel, "frame_timing_status", "not-verified")
        ).upper()
        unavailable = set(getattr(panel, "_camera_integrity_unavailable_sides", ()) or ())
        media_state = "UNAVAILABLE" if unavailable else ("LIVE" if frame_state == "CURRENT" else "STOPPED")
        self.schedule_status.setText(
            f"STUS schedule: {mode} · {state} · next {due_text} | "
            f"Clock {str(getattr(panel, 'camera_sync_status', 'unknown')).upper()} · "
            f"Frame timing {frame_state} · Media {media_state}"
        )

    def settings_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {}
        for key, widget in self.settings_widgets.items():
            if isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
                payload[key] = widget.value()
        return payload


class SessionWorkspace(QtWidgets.QWidget):
    """Create and present the timestamped folder that owns one work session."""

    inspection_requested = QtCore.pyqtSignal(object)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_measurement: Callable[[], None] | None = None,
        open_review: Callable[[], None] | None = None,
        parent: QtWidgets.QWidget | None = None,
        *,
        begin_session_action: Callable[[], object] | None = None,
        embedded: bool = False,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.begin_session_action = begin_session_action
        self._can_open_measurement = open_measurement is not None
        self._can_open_review = open_review is not None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(*(0, 0, 0, 0) if embedded else (8, 8, 8, 8))
        layout.setSpacing(6)
        if not embedded:
            layout.addWidget(
                WorkspaceHeader(
                    "Session",
                    "Create one timestamped folder, then keep this run's outputs together",
                )
            )

        self.lifecycle_group = QtWidgets.QGroupBox("Session folder")
        lifecycle_layout = QtWidgets.QVBoxLayout(self.lifecycle_group)
        lifecycle_layout.setContentsMargins(10, 10, 10, 10)
        lifecycle_layout.setSpacing(8)

        action_row = QtWidgets.QHBoxLayout()
        action_row.setSpacing(8)
        self.start_session_button = QtWidgets.QPushButton("Create Session Folder")
        self.start_session_button.setObjectName("sessionWorkspaceOpen")
        self.start_session_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.start_session_button.setSizePolicy(
            QtWidgets.QSizePolicy.Maximum,
            QtWidgets.QSizePolicy.Fixed,
        )
        self.start_session_button.setToolTip(
            "Create recordings/YYYYMMDD_HHMMSS_nanoseconds and route subsequent outputs into it."
        )
        self.start_session_button.clicked.connect(self._begin_measurement_session)
        self.close_session_button = QtWidgets.QPushButton("Close Session")
        self.close_session_button.setObjectName("sessionWorkspaceClose")
        self.close_session_button.setSizePolicy(
            QtWidgets.QSizePolicy.Maximum,
            QtWidgets.QSizePolicy.Fixed,
        )
        self.close_session_button.setToolTip(
            "Stop active Detection, save and seal the current Measurement, then close the Session."
        )
        self.close_session_button.clicked.connect(self._close_measurement_session)
        action_row.addWidget(self.start_session_button)
        action_row.addWidget(self.close_session_button)
        action_row.addStretch(1)
        lifecycle_layout.addLayout(action_row)

        folder_form = QtWidgets.QFormLayout()
        folder_form.setContentsMargins(0, 0, 0, 0)
        folder_form.setHorizontalSpacing(12)
        folder_form.setVerticalSpacing(6)
        self.session_status_label = QtWidgets.QLabel("No session folder")
        self.session_status_label.setObjectName("sessionWorkspaceLifecycle")
        self.session_status_label.setWordWrap(True)
        self.folder_name_label = QtWidgets.QLabel("—")
        self.folder_name_label.setObjectName("sessionWorkspaceFolderName")
        self.folder_name_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.folder_name_label.setWordWrap(True)
        self.folder_created_label = QtWidgets.QLabel("—")
        self.folder_created_label.setObjectName("sessionWorkspaceCreated")
        self.folder_created_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.folder_path_label = QtWidgets.QLabel("—")
        self.folder_path_label.setObjectName("sessionWorkspaceFolderPath")
        self.folder_path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.folder_path_label.setWordWrap(True)
        folder_form.addRow("State", self.session_status_label)
        folder_form.addRow("Folder name", self.folder_name_label)
        folder_form.addRow("Created", self.folder_created_label)
        folder_form.addRow("Storage path", self.folder_path_label)
        lifecycle_layout.addLayout(folder_form)

        self.storage_rule_label = QtWidgets.QLabel(
            "Outputs created after this folder opens are stored beneath it. "
            "Calibration, Alignment, Measurement, and Recording remain responsible for their own files."
        )
        self.storage_rule_label.setObjectName("compactContract")
        self.storage_rule_label.setWordWrap(True)
        lifecycle_layout.addWidget(self.storage_rule_label)
        self.action_feedback_label = QtWidgets.QLabel("")
        self.action_feedback_label.setObjectName("sessionWorkspaceFeedback")
        self.action_feedback_label.setProperty("severity", "info")
        self.action_feedback_label.setWordWrap(True)
        self.action_feedback_label.hide()
        lifecycle_layout.addWidget(self.action_feedback_label)
        layout.addWidget(self.lifecycle_group)

        self.navigation_group = QtWidgets.QGroupBox("Continue")
        navigation_row = QtWidgets.QHBoxLayout(self.navigation_group)
        navigation_row.setContentsMargins(10, 10, 10, 10)
        navigation_row.setSpacing(8)
        self.open_measurement_button = QtWidgets.QPushButton("Continue to Measurement")
        self.open_measurement_button.setEnabled(self._can_open_measurement)
        if open_measurement is not None:
            self.open_measurement_button.clicked.connect(open_measurement)
        self.open_review_button = QtWidgets.QPushButton("Review Stored Files")
        self.open_review_button.setEnabled(self._can_open_review)
        if open_review is not None:
            self.open_review_button.clicked.connect(open_review)
        navigation_row.addWidget(self.open_measurement_button)
        navigation_row.addWidget(self.open_review_button)
        navigation_row.addStretch(1)
        layout.addWidget(self.navigation_group)
        if not embedded:
            layout.addStretch(1)

        self._action_feedback = ""
        self.refresh({})

    def _set_action_feedback(self, text: str, *, severity: str = "error") -> None:
        self._action_feedback = str(text)
        self.action_feedback_label.setProperty("severity", str(severity))
        style = self.action_feedback_label.style()
        style.unpolish(self.action_feedback_label)
        style.polish(self.action_feedback_label)
        self._render_action_feedback()

    def _owner_method(self, *names: str) -> Callable[..., object] | None:
        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            for name in names:
                method = getattr(owner, name, None)
                if callable(method):
                    return method
        return None

    def _snapshot(self, *names: str) -> dict[str, object]:
        reader = self._owner_method(*names)
        if reader is None:
            return {}
        try:
            value = reader()
        except Exception as exc:
            return {"state": "ERROR", "error": str(exc)}
        return dict(value) if isinstance(value, Mapping) else {}

    def _run_action(
        self,
        names: tuple[str, ...],
        *,
        action: Callable[[], object] | None = None,
    ) -> None:
        method = action or self._owner_method(*names)
        if method is None:
            self._set_action_feedback("Session folder control is unavailable.")
            return
        try:
            result = method()
        except Exception as exc:
            self._set_action_feedback(f"Session folder action failed: {exc}")
            return
        if result is False:
            session = self._snapshot("measurement_session_snapshot")
            package = self._snapshot("measurement_package_snapshot")
            session_state = str(session.get("state") or "").upper()
            package_state = str(package.get("state") or "").upper()
            close_pending = (
                any(
                    name in {"close_measurement_session", "close_session"}
                    for name in names
                )
                and (
                    session_state == "CLOSING"
                    or package_state == "VERIFYING"
                    or bool(package.get("finalize_requested", False))
                )
            )
            if close_pending:
                self._set_action_feedback("Closing Session…", severity="info")
                self.refresh(self.status_model.snapshot())
                return
            reason = str(
                session.get("error")
                or package.get("error")
                or package.get("detail")
                or "The backend rejected the request."
            )
            self._set_action_feedback(f"Session folder was not changed: {reason}")
            self.refresh(self.status_model.snapshot())
            return
        self._set_action_feedback("", severity="info")
        self.refresh(self.status_model.snapshot())

    def _render_action_feedback(self) -> None:
        self.action_feedback_label.setText(self._action_feedback)
        self.action_feedback_label.setVisible(bool(self._action_feedback))

    def _begin_measurement_session(self) -> None:
        self._run_action(
            ("begin_measurement_session", "begin_session"),
            action=self.begin_session_action,
        )

    def _close_measurement_session(self) -> None:
        self._run_action(("close_measurement_session", "close_session"))

    def refresh(self, _runtime: dict[str, RuntimeStatus]) -> None:
        session = self._snapshot("measurement_session_snapshot")
        session_state = str(session.get("state") or "IDLE").upper()
        session_id = str(session.get("session_id") or "").strip()
        session_dir = str(session.get("session_dir") or "").strip()
        metadata = session.get("metadata")
        metadata = metadata if isinstance(metadata, Mapping) else {}
        opened_date = str(metadata.get("opened_local_date") or "").strip()
        opened_time = str(metadata.get("opened_local_time") or "").strip()
        created_text = " ".join(value for value in (opened_date, opened_time) if value)
        if not created_text and len(session_id) >= 15:
            date_part = session_id[:8]
            time_part = session_id[9:15]
            if date_part.isdigit() and time_part.isdigit():
                created_text = (
                    f"{date_part[:4]}-{date_part[4:6]}-{date_part[6:8]} "
                    f"{time_part[:2]}:{time_part[2:4]}:{time_part[4:6]}"
                )

        state_text = {
            "IDLE": "No folder created",
            "OPEN": "Ready for saving",
            "CLOSING": "Closing folder",
            "SEALED": "Closed",
            "RECOVERABLE": "Recovery required",
            "ERROR": "Folder status unavailable",
        }.get(session_state, session_state.replace("_", " ").title())
        self.session_status_label.setText(state_text)
        self.folder_name_label.setText(session_id or "—")
        self.folder_created_label.setText(created_text or "—")
        self.folder_path_label.setText(session_dir or "—")

        snapshot_error = str(session.get("error") or "").strip()
        current_measurement = session.get("current_measurement")
        current_measurement = (
            current_measurement if isinstance(current_measurement, Mapping) else {}
        )
        current_state = str(current_measurement.get("state") or "").upper()
        if session_state == "SEALED":
            self._set_action_feedback("", severity="info")
        elif (
            current_state == "RECOVERABLE"
            and self.action_feedback_label.property("severity") == "info"
            and self._action_feedback
        ):
            reason = str(
                current_measurement.get("error")
                or current_measurement.get("detail")
                or "Measurement recovery is required."
            )
            self._set_action_feedback(
                f"Session folder was not changed: {reason}",
                severity="error",
            )
        elif snapshot_error and not self._action_feedback:
            self._set_action_feedback(
                f"Session folder status error: {snapshot_error}",
                severity="error",
            )
        self._render_action_feedback()

        begin = self.begin_session_action or self._owner_method(
            "begin_measurement_session", "begin_session"
        )
        close = self._owner_method("close_measurement_session", "close_session")
        self.start_session_button.setEnabled(
            callable(begin) and session_state in {"IDLE", "SEALED"}
        )
        self.start_session_button.setText(
            "Create Another Session Folder" if session_state == "SEALED" else "Create Session Folder"
        )
        self.close_session_button.setEnabled(
            close is not None and session_state in {"OPEN", "CLOSING", "RECOVERABLE"}
        )
        self.open_measurement_button.setEnabled(
            self._can_open_measurement and session_state == "OPEN" and bool(session_id)
        )
        self.open_review_button.setEnabled(
            self._can_open_review and bool(session_id)
        )
        self.navigation_group.setVisible(
            bool(session_id) and (self._can_open_measurement or self._can_open_review)
        )


class MeasurementWorkspace(QtWidgets.QWidget):
    inspection_requested = QtCore.pyqtSignal(object)
    CAMERA_ASPECT_RATIO = 16.0 / 9.0
    RUN_STATUS_TEXT_LIMIT = 48

    @classmethod
    def _set_run_status_text(
        cls,
        label: QtWidgets.QLabel,
        text: str,
        *,
        detail: str = "",
    ) -> None:
        """Keep the Run summary compact while retaining its full evidence."""

        label.setText(_compact_text(text, limit=cls.RUN_STATUS_TEXT_LIMIT))
        label.setToolTip(detail or text)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        invoke_command: Callable[[str], bool] | None = None,
        open_session_details: Callable[[], None] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.invoke_command = invoke_command
        self._sessionless = not bool(
            getattr(
                getattr(self.backend, "panel", self.backend),
                "_measurement_session_hierarchy_enabled",
                True,
            )
        )
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Measurement"))

        run_group = QtWidgets.QGroupBox("Run")
        run_layout = QtWidgets.QHBoxLayout(run_group)
        run_layout.setContentsMargins(6, 6, 6, 6)
        run_layout.setSpacing(6)
        self.control_buttons: dict[str, QtWidgets.QPushButton] = {}
        preview_button = _command_button("Preview", "camera.toggle", invoke_command)
        self.control_buttons["camera.toggle"] = preview_button
        run_layout.addWidget(preview_button)
        detection_button = _command_button("측정 시작", "detection.toggle", invoke_command)
        detection_button.setToolTip(
            "Request Measurement start/stop at any lifecycle state. Unsafe requests are "
            "reported by the runtime without disabling this control."
        )
        self.control_buttons["detection.toggle"] = detection_button
        run_layout.addWidget(detection_button)
        self.finalize_session_button = QtWidgets.QPushButton("Retry Save")
        self.finalize_session_button.setObjectName("measurementPackageFinalize")
        self.finalize_session_button.setProperty("commandId", "session.finalize")
        self.finalize_session_button.setToolTip(
            "Retry automatic persistence/sealing after a recoverable save failure."
        )
        self.finalize_session_button.clicked.connect(self._finalize_measurement_package)
        run_layout.addWidget(self.finalize_session_button)
        if self._sessionless:
            self.finalize_session_button.hide()
        package_status_layout = QtWidgets.QVBoxLayout()
        package_status_layout.setContentsMargins(0, 0, 0, 0)
        package_status_layout.setSpacing(2)
        self.session_status_label = QtWidgets.QLabel(
            "Measurement: IDLE" if self._sessionless else "Session: IDLE"
        )
        self.session_status_label.setObjectName("measurementPackageLifecycle")
        self.session_status_label.setToolTip(
            "Transient run state; stored data never owns Measurement admission."
            if self._sessionless
            else "Create the parent Session in Sessions > Current before measuring."
        )
        package_status_layout.addWidget(self.session_status_label)
        self.measurement_status_label = QtWidgets.QLabel(
            "Recording: IDLE" if self._sessionless else "Measurement: NONE"
        )
        self.measurement_status_label.setObjectName("measurementPackageCurrent")
        self.measurement_status_label.setToolTip("No active Measurement.")
        package_status_layout.addWidget(self.measurement_status_label)
        self.session_step_label = QtWidgets.QLabel(
            "Next: 측정 시작" if self._sessionless else "Next: Sessions > Current"
        )
        self.session_step_label.setObjectName("measurementPackageStep")
        self.session_step_label.setToolTip(
            "Workflow: 측정 시작 → available inputs → 측정 종료 → immediately ready"
            if self._sessionless
            else "Workflow: Sessions > Current → 측정 시작 → 측정 종료 → automatic save"
        )
        package_status_layout.addWidget(self.session_step_label)
        self.evidence_summary_label = QtWidgets.QLabel(
            "Storage: OK" if self._sessionless else "Evidence: PENDING"
        )
        self.evidence_summary_label.setObjectName("measurementPackageEvidence")
        package_status_layout.addWidget(self.evidence_summary_label)
        for status_label in (
            self.session_status_label,
            self.measurement_status_label,
            self.session_step_label,
            self.evidence_summary_label,
        ):
            status_label.setMinimumWidth(0)
            status_label.setWordWrap(False)
            status_label.setSizePolicy(
                QtWidgets.QSizePolicy.Ignored,
                QtWidgets.QSizePolicy.Preferred,
            )
        run_layout.addLayout(package_status_layout, 1)
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
        self.previews = {
            side: CameraPreview(side, crop_to_fill=False, lock_chrome_height=True)
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            preview = self.previews[side]
            preview.setMinimumHeight(280)
            preview.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            preview_row.addWidget(preview)
        preview_row.setMinimumHeight(280)
        preview_row.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        preview_row.setStretchFactor(0, 1)
        preview_row.setStretchFactor(1, 1)
        upstream_layout.addWidget(preview_row)
        self.distance_timing_contract = QtWidgets.QLabel(
            "Timing: Settings preferred · fallback APP RECEIVE · |Δ|≤100 ms"
        )
        self.distance_timing_contract.setObjectName("compactContract")
        self.distance_timing_contract.setWordWrap(False)
        self.distance_timing_contract.setMinimumWidth(0)
        self.distance_timing_contract.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Preferred,
        )
        self.distance_timing_contract.setToolTip(
            "App receive is the default. RTCP/NTP preferred selects pairs only with VERIFIED, "
            "current same-epoch evidence; any timing fault falls back to exact app receive."
        )
        upstream_layout.addWidget(self.distance_timing_contract)
        self.upstream_values: dict[str, EngineeringValue] = {}
        upstream_fields = (
            ("camera.left", "CAM L"),
            ("camera.right", "CAM R"),
            ("detection", "Detection"),
            ("synchronization", "NTP Clock"),
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

        method_group = QtWidgets.QGroupBox("Measurement method")
        method_layout = QtWidgets.QGridLayout(method_group)
        method_layout.setContentsMargins(6, 6, 6, 6)
        method_layout.setHorizontalSpacing(8)
        self.measurement_method_combo = QtWidgets.QComboBox()
        self.measurement_method_combo.setObjectName("measurementMethodCombo")
        self.measurement_method_combo.setToolTip(
            "현재 exact K,D와 필요한 live/candidate 입력이 모두 준비된 방식만 표시합니다. "
            "선택 방식이 준비 상태를 잃으면 이 Measurement가 생성될 때 고정된 fallback "
            "순서로 자동 전환합니다."
        )
        self.measurement_method_combo.currentIndexChanged.connect(
            self._measurement_method_selected
        )
        self.measurement_method_status = QtWidgets.QLabel("준비된 측정 방식 확인 중")
        self.measurement_method_status.setObjectName("statusStrip")
        self.measurement_method_status.setWordWrap(True)
        method_layout.addWidget(QtWidgets.QLabel("Ready method"), 0, 0)
        method_layout.addWidget(self.measurement_method_combo, 0, 1)
        method_layout.addWidget(self.measurement_method_status, 1, 0, 1, 2)
        self.measurement_fill_method_combo = QtWidgets.QComboBox()
        self.measurement_fill_method_combo.setObjectName("measurementFillMethodCombo")
        self.measurement_fill_method_combo.setToolTip(
            "방식이 직접 제공하는 축은 OWN으로 잠깁니다. 미관측 회전축은 PX4, "
            "미관측 이동축은 GNSS 토글로 이 Measurement에서만 채웁니다."
        )
        self.measurement_fill_method_combo.currentIndexChanged.connect(
            self._measurement_fill_method_changed
        )
        method_layout.addWidget(QtWidgets.QLabel("DOF fill for"), 2, 0)
        method_layout.addWidget(self.measurement_fill_method_combo, 2, 1)
        fill_widget = QtWidgets.QWidget()
        fill_layout = QtWidgets.QGridLayout(fill_widget)
        fill_layout.setContentsMargins(0, 0, 0, 0)
        fill_layout.setHorizontalSpacing(6)
        fill_layout.setVerticalSpacing(3)
        dof_columns = (
            ("roll_deg", "Roll"),
            ("pitch_deg", "Pitch"),
            ("yaw_deg", "Yaw"),
            ("north_m", "North"),
            ("east_m", "East"),
            ("down_m", "Down"),
        )
        for column, (_dof, label) in enumerate(dof_columns, start=1):
            fill_layout.addWidget(QtWidgets.QLabel(label), 0, column)
        self.measurement_fill_buttons: dict[str, QtWidgets.QPushButton] = {}
        for row, side in enumerate(("left", "right"), start=1):
            fill_layout.addWidget(QtWidgets.QLabel(side.title()), row, 0)
            for column, (dof, _label) in enumerate(dof_columns, start=1):
                component = f"{side}.{dof}"
                button = QtWidgets.QPushButton("--")
                button.setCheckable(True)
                button.setObjectName(
                    "measurementFill" + side.title() + "".join(part.title() for part in dof.split("_"))
                )
                button.setProperty("component", component)
                button.toggled.connect(
                    lambda checked, component=component: self._measurement_fill_toggled(
                        component,
                        checked,
                    )
                )
                self.measurement_fill_buttons[component] = button
                fill_layout.addWidget(button, row, column)
        self.measurement_fill_status = QtWidgets.QLabel(
            "Pre 증거로 Measurement 초안이 준비된 경우 측정 시작 전에 fill source를 조정할 수 있습니다."
        )
        self.measurement_fill_status.setObjectName("statusStrip")
        self.measurement_fill_status.setWordWrap(True)
        method_layout.addWidget(fill_widget, 3, 0, 1, 2)
        method_layout.addWidget(self.measurement_fill_status, 4, 0, 1, 2)
        method_layout.setColumnStretch(1, 1)
        details_layout.addWidget(method_group)

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
                "Confidence",
                double_spin("yolo_confidence", 0.01, 1.0, 3, ""),
            ),
            (
                "IoU",
                double_spin("yolo_iou", 0.01, 1.0, 3, ""),
            ),
            (
                "P1 GNSS source",
                combo(
                    "pixhawk_left_gnss_source",
                    (("GPS1", "GPS1"), ("GPS2", "GPS2")),
                ),
            ),
            (
                "P2 GNSS source",
                combo(
                    "pixhawk_right_gnss_source",
                    (("GPS1", "GPS1"), ("GPS2", "GPS2")),
                ),
            ),
        )
        for index, (label, widget) in enumerate(geometry_fields):
            row = index // 2
            column = (index % 2) * 2
            geometry_form.addWidget(QtWidgets.QLabel(label), row, column)
            geometry_form.addWidget(widget, row, column + 1)
        settings_layout.addLayout(geometry_form)
        reverse_yaw = toggle(
            "measurement_baseline_yaw_reverse",
            "Reverse baseline yaw branch (persistent)",
        )
        reverse_yaw.setToolTip(
            "새 baseline-yaw Measurement에서 Auto 선택 branch를 반대로 사용합니다. "
            "현재 Measurement는 시작 시 고정된 branch를 유지합니다."
        )
        settings_layout.addWidget(reverse_yaw)

        gate_group = QtWidgets.QGroupBox("Pair gates")
        gate_form = QtWidgets.QGridLayout(gate_group)
        gate_form.setHorizontalSpacing(8)
        gate_form.setVerticalSpacing(5)
        gate_form.setColumnStretch(1, 1)
        gate_form.setColumnStretch(4, 1)
        gate_form.addWidget(toggle("class_gate_enabled", "Class gate"), 0, 0)
        gate_form.addWidget(text_edit("class_gate", "class id or name"), 0, 1, 1, 2)
        matching_toggle = toggle("matching_gate_enabled", "Time matching")
        matching_toggle.setChecked(True)
        matching_toggle.setEnabled(False)
        gate_form.addWidget(matching_toggle, 1, 0)
        time_basis_combo = combo(
            "time_basis",
            (
                ("App receive (default)", "app_receive_monotonic"),
                ("RTCP/NTP preferred · fallback App receive", "camera_rtcp"),
            ),
        )
        time_basis_combo.setEnabled(True)
        time_basis_combo.setToolTip(
            "RTCP/NTP is used only while Clock and current frame timing are usable; "
            "otherwise live distance uses exact app-receive monotonic time."
        )
        gate_form.addWidget(time_basis_combo, 1, 1)
        time_delta_toggle = toggle("time_delta_gate_enabled", "Max Δt")
        time_delta_toggle.setChecked(True)
        time_delta_toggle.setEnabled(False)
        gate_form.addWidget(time_delta_toggle, 1, 2)
        app_delta_spin = double_spin("max_pair_delta_s", 0.0, 0.100, 3, " s")
        if app_delta_spin.value() > 0.100:
            app_delta_spin.setValue(0.100)
        app_delta_spin.setToolTip(
            "Authoritative app-receive pair limit; values may be stricter than 100 ms."
        )
        gate_form.addWidget(app_delta_spin, 1, 3)
        gate_form.addWidget(toggle("vertical_delta_gate_enabled", "Vertical Δ"), 1, 4)
        gate_form.addWidget(double_spin("max_vertical_delta_px", 0.0, 5000.0, 1, " px"), 1, 5)
        settings_layout.addWidget(gate_group)

        self.measurement_settings_status = QtWidgets.QLabel(
            "Apply updates idle preview now; an active Measurement keeps timing until the next run. Save persists."
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

        self.session_details_group = QtWidgets.QGroupBox(
            "Data" if self._sessionless else "Session Details"
        )
        session_details_layout = QtWidgets.QHBoxLayout(self.session_details_group)
        session_details_layout.setContentsMargins(6, 6, 6, 6)
        session_details_label = QtWidgets.QLabel(
            "Open type-owned paths, capture tools, ULogs, and live writer health."
            if self._sessionless
            else "Review measurement packages, captures, ULogs, media, and saved-file evidence."
        )
        session_details_label.setObjectName("compactContract")
        session_details_label.setWordWrap(True)
        session_details_layout.addWidget(session_details_label, 1)
        self.session_details_button = QtWidgets.QPushButton(
            "Open Data" if self._sessionless else "Open Session Details"
        )
        self.session_details_button.setObjectName("measurementSessionDetails")
        self.session_details_button.setToolTip(
            "Open optional evidence, capture, ULog, and saved-file review tools."
        )
        self.session_details_button.setEnabled(open_session_details is not None)
        if open_session_details is not None:
            self.session_details_button.clicked.connect(open_session_details)
        session_details_layout.addWidget(self.session_details_button)
        details_layout.addWidget(self.session_details_group)
        layout.addWidget(details, 1)
        self._connect_panes()
        self._refresh_package_controls({}, getattr(self.backend, "panel", self.backend))

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
        if (
            self.preview_splitter.minimumHeight() == target_height
            and self.preview_splitter.maximumHeight() == target_height
        ):
            return
        # Measurement is a fixed-aspect inspection surface.  Leaving the
        # maximum height unconstrained lets the outer QScrollArea stretch the
        # splitter, after which KeepAspectRatioByExpanding crops the live image
        # and makes it look zoomed.  Recompute the natural height on resize,
        # then clamp both bounds to that value.
        self.preview_splitter.setMinimumHeight(target_height)
        self.preview_splitter.setMaximumHeight(target_height)

    def refresh_preview_layout(self) -> None:
        """Clamp the Measurement preview after navigation changes its viewport."""

        self._fit_preview_layout()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt override.
        super().resizeEvent(event)
        self._update_preview_pair_orientation()
        QtCore.QTimer.singleShot(0, self._fit_preview_layout)

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802 - Qt override.
        super().showEvent(event)
        # The first show can establish the final central-widget geometry
        # without delivering a resize event to this page.  Fit once after the
        # layout has settled so the first camera frame cannot trigger a large
        # orientation/height correction.
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

    def _measurement_fill_method_changed(self, _index: int) -> None:
        if bool(self.measurement_fill_method_combo.property("refreshing")):
            return
        self._render_measurement_fill_controls()

    def _render_measurement_fill_controls(self) -> None:
        row = self.measurement_fill_method_combo.currentData(QtCore.Qt.UserRole + 1)
        capability = dict(row) if isinstance(row, Mapping) else {}
        components = capability.get("components")
        component_rows = dict(components) if isinstance(components, Mapping) else {}
        locked_count = 0
        fill_count = 0
        unavailable_count = 0
        for component, button in self.measurement_fill_buttons.items():
            values = component_rows.get(component)
            state = dict(values) if isinstance(values, Mapping) else {}
            button.setProperty("refreshing", True)
            try:
                provided = bool(state.get("provided", False))
                editable = bool(state.get("editable", False))
                locked = bool(state.get("locked", True))
                source = str(state.get("source") or "none")
                compatible = str(state.get("compatible_fill_source") or "")
                if provided:
                    button.setText("OWN")
                    button.setChecked(True)
                    button.setEnabled(False)
                    locked_count += 1
                elif editable:
                    button.setText("PX4" if compatible == "live_pixhawk" else "GNSS")
                    button.setChecked(bool(state.get("fill_enabled", False)))
                    button.setEnabled(not locked)
                    fill_count += int(button.isChecked())
                else:
                    button.setText("N/A")
                    button.setChecked(False)
                    button.setEnabled(False)
                    unavailable_count += 1
                readiness = "ready" if bool(state.get("source_ready", False)) else "not ready"
                button.setToolTip(
                    f"{component} · source={source} · {state.get('reason', '')} · {readiness}"
                )
            finally:
                button.setProperty("refreshing", False)
        method_id = str(capability.get("method_id") or "")
        if not method_id:
            self.measurement_fill_status.setText("구성할 Measurement method가 없습니다.")
            return
        policy_state = "LOCKED" if any(
            bool(dict(value).get("locked", True))
            for value in component_rows.values()
            if isinstance(value, Mapping) and bool(dict(value).get("editable", False))
        ) else "EDITABLE"
        self.measurement_fill_status.setText(
            f"{policy_state} · method={method_id} · own={locked_count} · "
            f"fill enabled={fill_count} · unavailable={unavailable_count}"
        )

    def _measurement_fill_toggled(self, component: str, checked: bool) -> None:
        button = self.measurement_fill_buttons[component]
        if bool(button.property("refreshing")):
            return
        method_id = self.measurement_fill_method_combo.currentData()
        configurator = getattr(self.backend, "configure_measurement_method_fill", None)
        if not method_id or not callable(configurator):
            return
        try:
            snapshot = configurator(
                {
                    "method_id": str(method_id),
                    "component": component,
                    "enabled": bool(checked),
                }
            )
        except Exception as exc:
            self.measurement_fill_status.setText(f"DOF fill 변경 실패: {exc}")
            button.setProperty("refreshing", True)
            try:
                button.setChecked(not checked)
            finally:
                button.setProperty("refreshing", False)
            return
        if isinstance(snapshot, Mapping):
            self._refresh_measurement_methods(snapshot)

    def _measurement_method_selected(self, _index: int) -> None:
        method_id = self.measurement_method_combo.currentData()
        if not method_id or bool(self.measurement_method_combo.property("refreshing")):
            return
        selector = getattr(self.backend, "select_measurement_method", None)
        if not callable(selector):
            self.measurement_method_status.setText("Measurement method selector unavailable")
            return
        try:
            snapshot = selector({"method_id": str(method_id)})
        except Exception as exc:
            self.measurement_method_status.setText(f"방식 선택 실패: {exc}")
            return
        if isinstance(snapshot, Mapping):
            self._refresh_measurement_methods(snapshot)

    def _measurement_methods_snapshot(self) -> dict[str, object]:
        reader = getattr(self.backend, "measurement_methods_snapshot", None)
        if not callable(reader):
            return {}
        try:
            value = reader()
        except Exception:
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def _refresh_measurement_methods(self, snapshot: Mapping[str, object] | None = None) -> None:
        values = dict(snapshot or self._measurement_methods_snapshot())
        ready = values.get("ready_methods")
        rows = [dict(row) for row in ready if isinstance(row, Mapping)] if isinstance(ready, list) else []
        raw_capabilities = values.get("method_capabilities")
        capabilities = (
            [dict(row) for row in raw_capabilities if isinstance(row, Mapping)]
            if isinstance(raw_capabilities, list)
            else []
        )
        selected_method = str(values.get("selected_method_id") or "")
        selected_instance = str(values.get("selected_instance_id") or "")
        selection_reason = str(values.get("selection_reason") or "")
        reverse_yaw_widget = self.settings_widgets.get(
            "measurement_baseline_yaw_reverse"
        )
        if reverse_yaw_widget is not None:
            reverse_yaw_widget.setEnabled(not bool(values.get("fill_policy_locked", False)))
        previous_fill_method = str(self.measurement_fill_method_combo.currentData() or "")
        self.measurement_fill_method_combo.setProperty("refreshing", True)
        try:
            self.measurement_fill_method_combo.clear()
            fill_index = 0
            preferred_fill_method = previous_fill_method or selected_method
            for index, capability in enumerate(capabilities):
                method_id = str(capability.get("method_id") or "")
                label = str(capability.get("label") or method_id)
                state = "READY" if bool(capability.get("ready", False)) else "BLOCKED"
                self.measurement_fill_method_combo.addItem(f"{label} · {state}", method_id)
                self.measurement_fill_method_combo.setItemData(
                    index,
                    capability,
                    QtCore.Qt.UserRole + 1,
                )
                if method_id == preferred_fill_method:
                    fill_index = index
            self.measurement_fill_method_combo.setEnabled(bool(capabilities))
            if capabilities:
                self.measurement_fill_method_combo.setCurrentIndex(fill_index)
        finally:
            self.measurement_fill_method_combo.setProperty("refreshing", False)
        self._render_measurement_fill_controls()
        self.measurement_method_combo.setProperty("refreshing", True)
        try:
            self.measurement_method_combo.clear()
            if not rows:
                self.measurement_method_combo.addItem("준비된 측정 방식 없음", None)
                self.measurement_method_combo.setEnabled(False)
                self.measurement_method_status.setText(
                    "측정 차단 · Calibration/Alignment에서 한 방식의 K,D와 R,t 입력을 준비하세요."
                )
                return
            selected_index = 0
            for index, row in enumerate(rows):
                method_id = str(row.get("method_id") or "")
                label = str(row.get("label") or method_id)
                rank = row.get("fallback_rank")
                rank_text = "--" if rank is None else str(rank)
                self.measurement_method_combo.addItem(f"#{rank_text} · {label}", method_id)
                self.measurement_method_combo.setItemData(index, row, QtCore.Qt.UserRole + 1)
                if method_id == selected_method:
                    selected_index = index
            self.measurement_method_combo.setEnabled(True)
            self.measurement_method_combo.setCurrentIndex(selected_index)
            current = rows[selected_index]
            components = current.get("component_sources")
            component_text = ", ".join(
                f"{key}={value}" for key, value in dict(components or {}).items()
            )
            input_values = current.get("input_values")
            input_values = input_values if isinstance(input_values, Mapping) else {}
            branch = input_values.get("baseline_yaw_selection")
            branch = branch if isinstance(branch, Mapping) else {}
            branch_text = ""
            if branch:
                selected_yaw = branch.get("selected_yaw_deg")
                offset = branch.get("selected_offset_deg")
                mode = str(branch.get("mode") or "auto")
                selected_text = "--" if selected_yaw is None else f"{float(selected_yaw):.2f}°"
                offset_text = "--" if offset is None else f"{float(offset):+.0f}°"
                branch_text = f" · yaw={selected_text} ({offset_text}, {mode})"
                if branch.get("low_margin"):
                    branch_text += " · low margin"
                if branch.get("consensus") is False:
                    branch_text += " · P1/P2 disagree"
            self.measurement_method_status.setText(
                f"{selection_reason or 'selected'} · instance={selected_instance or current.get('instance_id', '--')}"
                + (f" · {component_text}" if component_text else "")
                + branch_text
            )
        finally:
            self.measurement_method_combo.setProperty("refreshing", False)

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
                signal.connect(
                    lambda _label, image, metadata, side=side: self._on_live_camera_frame(
                        side, image, metadata
                    )
                )
            measurement_signal = getattr(pane, "measurement_frame_changed", None)
            if measurement_signal is not None:
                measurement_signal.connect(
                    lambda _label, image, metadata, side=side: self.previews[side].set_frame(
                        image, metadata
                    )
                )
            detection_signal = getattr(pane, "detection_changed", None)
            if detection_signal is not None:
                detection_signal.connect(
                    lambda _label, detection, side=side: self.previews[side].set_detection(detection)
                )
            stats_signal = getattr(pane, "stats_changed", None)
            if stats_signal is not None:
                stats_signal.connect(lambda _label, stats, side=side: self.previews[side].set_metadata(stats))

    def _on_live_camera_frame(self, side: str, image: QtGui.QImage, metadata: object) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        pane = getattr(panel, f"{side}_pane", None)
        if bool(getattr(pane, "measurement_pair_display_active", False)):
            return
        self.previews[side].set_frame(image, metadata)

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

    def _refresh_quality(
        self,
        panel: object,
        measurement_status: RuntimeStatus | None = None,
    ) -> None:
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
        if measurement_status is not None and measurement_status.state != "VALID":
            state = str(measurement_status.state or "NO DATA")
            detail = str(measurement_status.detail or "measurement evidence is not current")
            self.quality_labels["status"].setText(f"{state} · {detail}")

    def _package_method(self, *names: str) -> Callable[..., object] | None:
        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            for name in names:
                method = getattr(owner, name, None)
                if callable(method):
                    return method
        return None

    def _measurement_package_snapshot(self) -> dict[str, object] | None:
        reader = self._package_method("measurement_package_snapshot")
        if reader is None:
            return None
        try:
            value = reader()
        except Exception as exc:
            self._set_run_status_text(
                self.evidence_summary_label,
                "Evidence: ERROR",
                detail=f"Evidence: status unavailable · {exc}",
            )
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def _measurement_session_snapshot(self) -> dict[str, object] | None:
        reader = self._package_method("measurement_session_snapshot")
        if reader is None:
            return None
        try:
            value = reader()
        except Exception as exc:
            self._set_run_status_text(
                self.evidence_summary_label,
                "Evidence: ERROR",
                detail=f"Session evidence: status unavailable · {exc}",
            )
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def _refresh_package_controls(
        self,
        _runtime: Mapping[str, RuntimeStatus],
        _panel: object | None = None,
    ) -> None:
        if self._sessionless:
            panel = getattr(self.backend, "panel", self.backend)
            run_reader = getattr(self.backend, "measurement_run_snapshot", None)
            if not callable(run_reader):
                run_reader = getattr(panel, "measurement_run_snapshot", None)
            storage_reader = getattr(self.backend, "data_storage_snapshot", None)
            if not callable(storage_reader):
                storage_reader = getattr(panel, "data_storage_snapshot", None)
            try:
                run = run_reader() if callable(run_reader) else {}
            except Exception as exc:
                run = {"state": "ERROR", "error": str(exc)}
            try:
                storage = storage_reader() if callable(storage_reader) else {}
            except Exception as exc:
                storage = {"state": "UNAVAILABLE", "error": str(exc)}
            state = str(run.get("state") or "IDLE").upper()
            detail = str(run.get("detail") or run.get("error") or "")
            sources = run.get("sources")
            sources = sources if isinstance(sources, Mapping) else {}
            source_counts = {
                source_state: sum(
                    1
                    for value in sources.values()
                    if isinstance(value, Mapping)
                    and str(value.get("state") or "").upper() == source_state
                )
                for source_state in ("LIVE", "PENDING", "WAITING_RESOURCE", "UNAVAILABLE")
            }
            source_summary = (
                f"live={source_counts['LIVE']} · wait="
                f"{source_counts['PENDING'] + source_counts['WAITING_RESOURCE']} · "
                f"unavailable={source_counts['UNAVAILABLE']}"
            )
            self._set_run_status_text(
                self.session_status_label,
                f"Measurement: {state} · {source_summary}",
                detail=(
                    f"Measurement: {state} · {source_summary}"
                    + (f" · {detail}" if detail else "")
                ),
            )
            recording = "MANUAL" if bool(
                getattr(panel, "_manual_recording_active", False)
            ) else "MEASUREMENT" if bool(
                getattr(panel, "_automatic_session_recording", False)
            ) else "IDLE"
            presence = str(
                getattr(panel, "_last_recording_presence_state", "IDLE") or "IDLE"
            )
            self._set_run_status_text(
                self.measurement_status_label,
                f"Recording: {recording if recording != 'IDLE' else presence}",
                detail=(
                    f"Recording: {recording if recording != 'IDLE' else presence} · "
                    "SAVED means file presence only; playback integrity is not verified"
                ),
            )
            next_text = {
                "IDLE": "Next: 측정 시작",
                "QUEUED": "Next: camera owner release",
                "STARTING": "Starting…",
                "RUNNING": "Next: 측정 종료",
                "STOPPING": "Stopping camera/video…",
                "ERROR": "Next: retry Measurement",
            }.get(state, state)
            self._set_run_status_text(
                self.session_step_label,
                next_text,
                detail=str(run.get("queued_reason") or detail or next_text),
            )
            storage_state = str(storage.get("state") or "UNAVAILABLE").upper()
            dropped = int(storage.get("dropped_records") or 0)
            pending = int(storage.get("priority_pending") or 0) + int(
                storage.get("bulk_pending") or 0
            )
            retry_pending = int(storage.get("essential_retry_pending") or 0)
            self._set_run_status_text(
                self.evidence_summary_label,
                f"Storage: {storage_state} · retry={retry_pending} · drop={dropped}",
                detail=(
                    f"Storage: {storage_state} · pending={pending} · "
                    f"essential retry={retry_pending} · dropped={dropped} · "
                    f"root={storage.get('root', '--')}"
                ),
            )
            self.finalize_session_button.setEnabled(False)
            return
        package = self._measurement_package_snapshot()
        session = self._measurement_session_snapshot()
        if package is None:
            self.finalize_session_button.setEnabled(False)
            return
        self._apply_measurement_package_snapshot(package, session=session)

    @staticmethod
    def _package_artifact_counts(snapshot: Mapping[str, object]) -> dict[str, int]:
        counts = {"present": 0, "missing": 0, "failed": 0, "not_applicable": 0}
        artifacts = snapshot.get("artifacts")
        rows = artifacts if isinstance(artifacts, (tuple, list)) else ()
        for raw in rows:
            if not isinstance(raw, Mapping):
                continue
            state = str(raw.get("state", raw.get("status", "")) or "").lower()
            if state in counts:
                counts[state] += 1
        return counts

    def _apply_measurement_package_snapshot(
        self,
        package: Mapping[str, object],
        *,
        session: Mapping[str, object] | None = None,
    ) -> None:
        state = str(package.get("state", "IDLE") or "IDLE").upper()
        session_id = str(package.get("session_id", "") or "")
        measurement_id = str(package.get("measurement_id", "") or "")
        revision = str(package.get("revision", "") or "")
        detail = str(package.get("detail", package.get("error", "")) or "")
        identity = " · ".join(value for value in (session_id, revision) if value)
        session_state = str(
            (session or {}).get("state", package.get("session_state", "")) or ""
        ).upper()
        if session is not None:
            rows = (session or {}).get("measurements")
            measurement_count = len(rows) if isinstance(rows, (tuple, list)) else 0
            full_session_text = (
                f"Session: {session_state or 'IDLE'}"
                + (f" · {session_id}" if session_id else "")
                + f" · measurements={measurement_count}"
            )
            self._set_run_status_text(
                self.session_status_label,
                f"Session: {session_state or 'IDLE'} · {measurement_count} measurements",
                detail=full_session_text,
            )
            visible_measurement_state = (
                "NONE" if not measurement_id and state == "IDLE" else state
            )
            full_measurement_text = f"Measurement: {visible_measurement_state}"
            measurement_text = full_measurement_text
            if measurement_id:
                full_measurement_text += f" · {measurement_id} · {revision}"
                measurement_text += f" · {measurement_id}/{revision}"
            if detail:
                full_measurement_text += f" · {detail}"
            self._set_run_status_text(
                self.measurement_status_label,
                measurement_text,
                detail=full_measurement_text,
            )
        else:
            status_text = f"Session: {state}"
            if identity:
                status_text += f" · {identity}"
            if detail:
                status_text += f" · {detail}"
            self._set_run_status_text(
                self.session_status_label,
                f"Session: {state}",
                detail=status_text,
            )
            measurement_text = f"Measurement: {state}" + (
                f" · {revision}" if revision else ""
            )
            self._set_run_status_text(
                self.measurement_status_label,
                measurement_text,
            )

        detection_started = bool(package.get("detection_started", False))
        detection_completed = bool(package.get("detection_completed", False))
        step_text, step_detail = {
            "IDLE": (
                "Next: Sessions > Current",
                "Workflow: create or open the parent Session in Sessions > Current",
            ),
            "POST_CAPTURE": (
                "Next: Save",
                "Workflow: Measurement complete · automatic save pending",
            ),
            "VERIFYING": (
                "Saving…",
                "Workflow: draining producers and verifying inventory",
            ),
            "SEALED_COMPLETE": (
                "Saved",
                "Workflow: sealed with complete evidence",
            ),
            "SEALED_INCOMPLETE": (
                "Saved · evidence missing",
                "Workflow: sealed with documented missing evidence",
            ),
            "RECOVERABLE": (
                "Next: Retry Save",
                "Workflow: recoverable spool · retry Finalize",
            ),
            "INTEGRITY_FAILED": (
                "Integrity failed",
                "Workflow: sealed data failed integrity verification",
            ),
        }.get(state, (f"State: {state}", f"Workflow: {state}"))
        if session is not None and state == "IDLE" and session_state == "OPEN":
            step_text = "Next: 측정 시작"
            step_detail = "Workflow: session open · start creates the first Measurement"
        elif session is not None and state in {"SEALED_COMPLETE", "SEALED_INCOMPLETE"} and session_state == "OPEN":
            step_text = "Next: 다음 측정 시작"
            step_detail = "Workflow: previous Measurement sealed · start creates the next child"
        elif session is not None and session_state == "SEALED":
            step_text = "Session closed"
            step_detail = "Workflow: session sealed · start a new session when ready"
        if state == "COLLECTING":
            step_text, step_detail = (
                ("Detection running", "Workflow: Detection running")
                if detection_started and not detection_completed
                else (
                    "Next: 측정 시작",
                    "Workflow: Pre evidence draft · start Detection once",
                )
            )
        self._set_run_status_text(
            self.session_step_label,
            step_text,
            detail=step_detail,
        )

        completeness = package.get("evidence_completeness")
        completeness = completeness if isinstance(completeness, Mapping) else {}
        evidence_state = str(
            completeness.get(
                "status",
                package.get("completeness", package.get("completeness_state", "")),
            )
            or ""
        ).upper()
        if not evidence_state:
            if state == "SEALED_COMPLETE":
                evidence_state = "COMPLETE"
            elif state in {"SEALED_INCOMPLETE", "INTEGRITY_FAILED"}:
                evidence_state = "INCOMPLETE"
            else:
                evidence_state = "PENDING"
        counts = self._package_artifact_counts(package)
        missing_required = completeness.get("missing_required")
        if isinstance(missing_required, (tuple, list)):
            counts["missing"] = max(counts["missing"], len(missing_required))
        count_text = ", ".join(
            f"{key}={value}" for key, value in counts.items() if value
        )
        compact_counts = " / ".join(
            f"{value} {'ok' if key == 'present' else key.replace('_', ' ')}"
            for key, value in counts.items()
            if value
        )
        evidence_text = f"Evidence: {evidence_state}" + (
            f" · {compact_counts}" if compact_counts else ""
        )
        self._set_run_status_text(
            self.evidence_summary_label,
            evidence_text,
            detail=f"Evidence: {evidence_state}"
            + (f" · {count_text}" if count_text else ""),
        )

        finalize_method = self._package_method(
            "finalize_measurement_package", "seal_session", "finalize_session"
        )
        self.finalize_session_button.setEnabled(
            finalize_method is not None and state in {"POST_CAPTURE", "RECOVERABLE"}
        )

    def _run_package_action(
        self,
        names: tuple[str, ...],
        *,
        unavailable_text: str,
        pending_text: str,
        action: Callable[[], object] | None = None,
    ) -> None:
        method = action or self._package_method(*names)
        if method is None:
            self._set_run_status_text(
                self.session_status_label,
                "Session: unavailable",
                detail=unavailable_text,
            )
            return
        try:
            result = method()
        except Exception as exc:
            self._set_run_status_text(
                self.session_status_label,
                "Session: ERROR",
                detail=f"Session: ERROR · {exc}",
            )
            return
        package = dict(result) if isinstance(result, Mapping) else self._measurement_package_snapshot()
        if package:
            self._apply_measurement_package_snapshot(
                package,
                session=self._measurement_session_snapshot(),
            )
        else:
            self._set_run_status_text(
                self.session_status_label,
                "Session: pending",
                detail=pending_text,
            )

    def _finalize_measurement_package(self) -> None:
        self._run_package_action(
            ("finalize_measurement_package", "seal_session", "finalize_session"),
            unavailable_text="Session: Finalize control unavailable",
            pending_text="Session: Finalize requested · waiting for producer drain",
        )

    def refresh_run_controls(self, snapshot: dict[str, RuntimeStatus], panel: object) -> None:
        camera = snapshot.get("camera.left")
        detection = snapshot.get("detection")
        recording = snapshot.get("recording")
        camera_state = camera.state if camera else "STOPPED"
        detection_state = detection.state if detection else "DISABLED"
        recording_state = recording.state if recording else "IDLE"
        run_reader = getattr(self.backend, "measurement_run_snapshot", None)
        if not callable(run_reader):
            run_reader = getattr(panel, "measurement_run_snapshot", None)
        try:
            run_snapshot = run_reader() if callable(run_reader) else {}
        except Exception:
            run_snapshot = {}
        run_state = str(
            run_snapshot.get("state") if isinstance(run_snapshot, Mapping) else ""
        ).upper()
        recording_locked = recording_state in {"ACTIVE", "FINALIZING"}
        self.control_buttons["camera.toggle"].setText(
            "Stop preview" if camera_state in {"LIVE", "STARTING"} else "Preview"
        )
        self.control_buttons["detection.toggle"].setText(
            "측정 종료"
            if run_state in {"RUNNING", "STOPPING"}
            or detection_state in {"RUNNING", "STARTING"}
            else "측정 시작"
        )
        self.control_buttons["camera.toggle"].setEnabled(not recording_locked)
        # Keep the operator command available in every lifecycle state.  The
        # camera/package owners remain the source of truth for rejecting an
        # unsafe duplicate interval, active writer, unfinished save, or bad
        # RTSP input and can therefore return an actionable failure reason.
        self.control_buttons["detection.toggle"].setEnabled(self.invoke_command is not None)
        if self._sessionless:
            self._refresh_package_controls(snapshot, panel)
            return
        session_id = str(getattr(panel, "recording_session_id", "") or "")
        automatic = bool(getattr(panel, "_automatic_session_recording", False))
        if recording_state == "ACTIVE" and automatic:
            self._set_run_status_text(
                self.session_status_label,
                "Session: recording",
                detail=f"Session: recording automatically · {session_id or 'preparing'}",
            )
        elif recording_state == "FINALIZING":
            self._set_run_status_text(
                self.session_status_label,
                "Session: saving",
                detail=f"Session: saving · {session_id or 'current run'}",
            )
        elif recording_state == "RECOVERABLE":
            self._set_run_status_text(
                self.session_status_label,
                "Session: retry save",
                detail=f"Session: save needs a retry · {session_id or 'recovered run'}",
            )
        elif recording_state == "SAVED":
            self._set_run_status_text(
                self.session_status_label,
                "Session: saved",
                detail=f"Session: saved · {session_id or 'latest run'}",
            )
        elif recording_state == "ACTIVE":
            self._set_run_status_text(
                self.session_status_label,
                "Session: manual recording",
                detail=f"Session: manual evidence recording · {session_id or 'current run'}",
            )
        else:
            self._set_run_status_text(
                self.session_status_label,
                "Session: automatic",
                detail="Session: automatic · 측정 시작 creates a child; 측정 종료 saves it",
            )
        self._refresh_package_controls(snapshot, panel)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        self._refresh_preview_metadata(panel)
        self._refresh_distance_timing_contract(panel)
        measurement_status = snapshot.get("measurement")
        self._refresh_quality(panel, measurement_status)
        self._refresh_measurement_methods()
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
            current_valid = bool(measurement_status and measurement_status.state == "VALID")
            rejected = bool(
                measurement_status
                and measurement_status.state in {"ERROR", "REJECTED"}
            )
            result_valid = True if current_valid else False if rejected else None
            result_age_s = measurement_status.age_s if measurement_status else None
            self.values["range"].set_value(getattr(point, "distance_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint", age_s=result_age_s, valid=result_valid)
            self.values["forward"].set_value(getattr(point, "forward_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint", age_s=result_age_s, valid=result_valid)
            self.values["right"].set_value(getattr(point, "right_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint", age_s=result_age_s, valid=result_valid)
            self.values["down"].set_value(getattr(point, "down_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint", age_s=result_age_s, valid=result_valid)

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

    def _refresh_distance_timing_contract(self, panel: object) -> None:
        requested_reader = getattr(panel, "_yolo_time_basis", None)
        requested = (
            str(requested_reader())
            if callable(requested_reader)
            else str(
                getattr(
                    getattr(panel, "settings", None),
                    "yolo_gate_time_basis",
                    "app_receive_monotonic",
                )
            )
        )
        result = getattr(panel, "_coordinated_pairing_result", None)
        timing = getattr(result, "timing", None)
        effective = str(getattr(timing, "effective_basis", requested) or requested)
        fallback_reason = str(getattr(timing, "fallback_reason", "") or "")
        requested_text = "RTCP/NTP" if requested == "camera_rtcp" else "APP RECEIVE"
        effective_text = "RTCP/NTP" if effective == "camera_rtcp" else "APP RECEIVE"
        suffix = f" · fallback={fallback_reason}" if fallback_reason else ""
        basis_text = (
            requested_text
            if requested_text == effective_text
            else f"{requested_text} → {effective_text}"
        )
        self.distance_timing_contract.setText(
            f"Timing: {basis_text}{suffix} · |Δ|≤100 ms"
        )
        self.distance_timing_contract.setToolTip(
            "The active Measurement request is frozen at start. RTCP/NTP is selected only with "
            "VERIFIED/current same-epoch evidence; otherwise exact app receive is authoritative."
        )


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

        transport_group = QtWidgets.QGroupBox("Board transport · explicit Connect only")
        transport_layout = QtWidgets.QGridLayout(transport_group)
        self.transport_combos: dict[str, QtWidgets.QComboBox] = {}
        self.radio_device_edits: dict[str, QtWidgets.QLineEdit] = {}
        self.radio_baud_spins: dict[str, QtWidgets.QSpinBox] = {}
        self.role_connect_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.role_disconnect_buttons: dict[str, QtWidgets.QPushButton] = {}
        settings_reader = getattr(backend, "workstation_settings_snapshot", None)
        try:
            transport_defaults = dict(settings_reader()) if callable(settings_reader) else {}
        except Exception:
            transport_defaults = {}
        for column, title in enumerate(("Board", "Transport", "Serial device", "Baud", "", "")):
            if title:
                transport_layout.addWidget(QtWidgets.QLabel(title), 0, column)
        for row, (side, label, radio_label) in enumerate(
            (
                ("left", "P1 / sysid 1", "P1 Telemetry Radio"),
                ("right", "P2 / sysid 2", "P2 Telemetry Radio"),
            ),
            start=1,
        ):
            combo = QtWidgets.QComboBox()
            combo.addItem(radio_label, "radio")
            combo.addItem("Ethernet", "ethernet")
            combo.addItem("USB", "usb")
            selected = str(transport_defaults.get(f"{side}_pixhawk_transport", "radio"))
            combo.setCurrentIndex(max(0, combo.findData(selected)))
            serial_key = "bootstrap_url" if selected == "usb" else "radio_device"
            serial_baud_key = "bootstrap_baud" if selected == "usb" else "radio_baud"
            device = QtWidgets.QLineEdit(
                str(transport_defaults.get(f"{side}_pixhawk_{serial_key}", "") or "")
            )
            device.setPlaceholderText("/dev/serial/by-id/...")
            baud = QtWidgets.QSpinBox()
            baud.setRange(1_200, 4_000_000)
            baud.setValue(int(transport_defaults.get(f"{side}_pixhawk_{serial_baud_key}", 57_600)))
            connect_button = QtWidgets.QPushButton("Connect")
            disconnect_button = QtWidgets.QPushButton("Disconnect")
            combo.currentIndexChanged.connect(
                lambda _index, side=side, combo=combo: self._set_transport(side, combo)
            )
            device.editingFinished.connect(
                lambda side=side, device=device, baud=baud: self._apply_radio_endpoint(side, device, baud)
            )
            baud.editingFinished.connect(
                lambda side=side, device=device, baud=baud: self._apply_radio_endpoint(side, device, baud)
            )
            connect_button.clicked.connect(
                lambda _checked=False, side=side: self._connect_role(side)
            )
            disconnect_button.clicked.connect(
                lambda _checked=False, side=side: self._disconnect_role(side)
            )
            self.transport_combos[side] = combo
            self.radio_device_edits[side] = device
            self.radio_baud_spins[side] = baud
            self.role_connect_buttons[side] = connect_button
            self.role_disconnect_buttons[side] = disconnect_button
            transport_layout.addWidget(QtWidgets.QLabel(label), row, 0)
            transport_layout.addWidget(combo, row, 1)
            transport_layout.addWidget(device, row, 2)
            transport_layout.addWidget(baud, row, 3)
            transport_layout.addWidget(connect_button, row, 4)
            transport_layout.addWidget(disconnect_button, row, 5)
        # Drone / sysid 3 shares the same MAVLink subscriber side as the
        # Telemetry workspace, so it is owned there instead of duplicated here.
        drone_hint = QtWidgets.QLabel(
            "Drone / sysid 3 transport와 Connect는 Devices → Telemetry에서 관리합니다."
        )
        drone_hint.setObjectName("statusStrip")
        drone_hint.setWordWrap(True)
        transport_layout.addWidget(drone_hint, transport_layout.rowCount(), 0, 1, 6)
        layout.addWidget(transport_group)

        status_group = QtWidgets.QGroupBox("Connection")
        status_layout = QtWidgets.QHBoxLayout(status_group)
        self.indicators: dict[str, StatusIndicator] = {}
        for key, label in (
            ("pixhawk.left", "P1"),
            ("pixhawk.right", "P2"),
            ("pixhawk.drone", "Drone"),
        ):
            indicator = StatusIndicator(label)
            # The detail line contains a live PX4 timestamp. Keep the card
            # width independent of that changing string so the Connection
            # row does not reflow on every attitude sample.
            indicator.setFixedWidth(PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator)
        status_layout.addStretch(1)
        layout.addWidget(status_group)

        recovery_group = QtWidgets.QGroupBox("ETH MAV")
        recovery_layout = QtWidgets.QVBoxLayout(recovery_group)
        recovery_hint = QtWidgets.QLabel(
            "ETH MAVLINK RX STALLED일 때만, Find USB에서 현재 heartbeat system ID가 유일하게 "
            "검증된 해당 Pixhawk를 operator 확인 후 재부팅합니다. 설정/파라미터는 쓰지 않으며 "
            "재부팅 후 Heartbeat+ATTITUDE 10초 qualification을 다시 통과해야 합니다."
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
                f"{label} verified-USB reboot",
                command_id,
                invoke_command,
            )
            button.setMinimumWidth(170)
            button.setToolTip(
                f"{label} ETH RX stall과 고유 USB sysid를 다시 검증하고 확인창 뒤 해당 side만 "
                "one-shot 재부팅합니다. USB가 없거나 애매하면 command 없이 수동 전원 복구를 안내합니다."
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

        candidate_group = QtWidgets.QGroupBox("Network candidates and assignments")
        candidate_layout = QtWidgets.QVBoxLayout(candidate_group)
        assignment_layout = QtWidgets.QHBoxLayout()
        self.network_assignment_values: dict[str, dict[str, EngineeringValue]] = {}
        for side, label in (
            ("left", "P1 network assignment"),
            ("right", "P2 network assignment"),
        ):
            card = QtWidgets.QGroupBox(label)
            card_layout = QtWidgets.QGridLayout(card)
            card_layout.setContentsMargins(6, 6, 6, 6)
            card_layout.setHorizontalSpacing(6)
            card_layout.setVerticalSpacing(4)
            values: dict[str, EngineeringValue] = {}
            for key, value_label, row, column, column_span in (
                ("state", "State", 0, 0, 1),
                ("nic", "NIC", 0, 1, 1),
                ("local", "Local IPv4", 0, 2, 1),
                ("endpoint", "Endpoint", 1, 0, 1),
                ("route", "Route", 1, 1, 2),
            ):
                value = EngineeringValue(value_label)
                value.set_value(
                    "LEGACY AUTO ROUTE" if key == "state" else "--",
                    source="no explicit NIC assignment",
                )
                values[key] = value
                card_layout.addWidget(value, row, column, 1, column_span)
            for column in range(3):
                card_layout.setColumnStretch(column, 1)
            self.network_assignment_values[side] = values
            assignment_layout.addWidget(card, 1)
        candidate_layout.addLayout(assignment_layout)
        self.discovery_status = EngineeringValue("Discovery")
        candidate_layout.addWidget(self.discovery_status)
        self.candidate_model = _table_model(
            self,
            (
                "Source",
                "NIC / Device",
                "Link / Kind",
                "Evidence",
                "Role",
            ),
        )
        self.candidate_table = QtWidgets.QTableView()
        self.candidate_table.setModel(self.candidate_model)
        self.candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.candidate_table.verticalHeader().setVisible(False)
        self.candidate_table.setMaximumHeight(170)
        self.candidate_table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        candidate_header = self.candidate_table.horizontalHeader()
        candidate_header.setStretchLastSection(False)
        candidate_header.setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        for column, width in enumerate((90, 280, 110, 260, 170)):
            candidate_header.resizeSection(column, width)
        self.candidate_table.selectionModel().selectionChanged.connect(self._refresh_candidate_actions)
        self.candidate_table.clicked.connect(self._inspect_candidate_row)
        candidate_layout.addWidget(self.candidate_table)
        candidate_actions = QtWidgets.QHBoxLayout()
        self.candidate_buttons: dict[str, QtWidgets.QPushButton] = {}
        for target, label in (("left", "Assign P1"), ("right", "Assign P2")):
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
        self._discovery_revision = -1
        self._recording_locked = False
        self._network_role_locked = False
        self._invoke_command_available = invoke_command is not None
        self._invoke_command = invoke_command
        layout.addWidget(candidate_group)

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
            (
                "Receiver",
                "Fix number",
                "RTK state",
                "Satellites",
                "Altitude MSL m",
                "Altitude ellipsoid m",
                "HDOP/VDOP",
                "H/V acc m",
                "Age",
            ),
        )
        self.receiver_table = QtWidgets.QTableView()
        self.receiver_table.setModel(self.receiver_model)
        self.receiver_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.receiver_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.receiver_table.verticalHeader().setVisible(False)
        self.receiver_table.setMaximumHeight(190)
        self.receiver_table.horizontalHeader().setStretchLastSection(True)
        self.receiver_table.clicked.connect(self._inspect_receiver_row)
        receiver_layout.addWidget(self.receiver_table)
        layout.addWidget(receiver_group)
        self._receiver_row_keys: tuple[tuple[str, str], ...] = ()

        magnetometer_group = QtWidgets.QGroupBox("Magnetometer by CAL_MAG slot")
        magnetometer_layout = QtWidgets.QVBoxLayout(magnetometer_group)
        magnetometer_hint = QtWidgets.QLabel(
            "CAL_MAG0..3 순서 · Device ID로 실제 센서 sample 연결 · "
            "RAW yaw=atan2(-y,x) · Corrected yaw=CAL_MAG 보정값"
        )
        magnetometer_hint.setObjectName("statusStrip")
        magnetometer_hint.setWordWrap(True)
        magnetometer_layout.addWidget(magnetometer_hint)
        self.magnetometer_model = _table_model(
            self,
            (
                "Pixhawk",
                "CAL_MAG",
                "Sensor",
                "Device ID",
                "State",
                "xmag",
                "ymag",
                "zmag",
                "Norm",
                "RAW yaw°",
                "Corrected yaw°",
                "Δyaw°",
                "Priority",
                "Unit",
                "Age",
                "Topic",
            ),
        )
        self.magnetometer_table = QtWidgets.QTableView()
        self.magnetometer_table.setModel(self.magnetometer_model)
        self.magnetometer_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.magnetometer_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.magnetometer_table.verticalHeader().setVisible(False)
        # P1/P2 each expose the four configured CAL_MAG slots. Keep all
        # eight rows visible so the table does not acquire an inner scrollbar.
        self.magnetometer_table.setMinimumHeight(300)
        self.magnetometer_table.setMaximumHeight(300)
        self.magnetometer_table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        magnetometer_header = self.magnetometer_table.horizontalHeader()
        magnetometer_header.setStretchLastSection(False)
        magnetometer_header.setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        for column, width in enumerate(
            (72, 76, 190, 100, 72, 72, 72, 72, 72, 88, 96, 88, 88, 52, 72, 190)
        ):
            magnetometer_header.resizeSection(column, width)
        magnetometer_header.setSectionHidden(10, False)
        magnetometer_header.setSectionHidden(11, True)
        self.magnetometer_table.clicked.connect(self._inspect_magnetometer_row)
        magnetometer_layout.addWidget(self.magnetometer_table)
        layout.addWidget(magnetometer_group)

        self._build_mag_priority_group(layout)

        can_magnetometer_group = QtWidgets.QGroupBox("CAN magnetometer (dedicated MAVLink)")
        can_magnetometer_layout = QtWidgets.QVBoxLayout(can_magnetometer_group)
        can_magnetometer_hint = QtWidgets.QLabel(
            "CAN1/CAN2 값은 PX4 DEBUG_VECT CAN_MAG 호환 메시지가 수신될 때만 표시 · "
            "RAW yaw=센서축 진단값 · Corrected yaw=CAL_MAG 보정값"
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
                "RAW yaw°",
                "Corrected yaw°",
                "Δyaw°",
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
        self.can_magnetometer_table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        can_magnetometer_header = self.can_magnetometer_table.horizontalHeader()
        can_magnetometer_header.setStretchLastSection(False)
        can_magnetometer_header.setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        for column, width in enumerate((72, 72, 72, 112, 72, 72, 72, 72, 72, 88, 96, 88, 52, 72, 220)):
            can_magnetometer_header.resizeSection(column, width)
        can_magnetometer_header.setSectionHidden(10, False)
        can_magnetometer_header.setSectionHidden(11, True)
        self.can_magnetometer_table.clicked.connect(self._inspect_can_magnetometer_row)
        can_magnetometer_layout.addWidget(self.can_magnetometer_table)
        layout.addWidget(can_magnetometer_group)

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

        baseline_yaw_group = QtWidgets.QGroupBox("P1→P2 baseline / Pixhawk yaw")
        baseline_yaw_group_layout = QtWidgets.QHBoxLayout(baseline_yaw_group)
        baseline_yaw_group_layout.setContentsMargins(8, 8, 8, 8)
        baseline_yaw_group_layout.setSpacing(8)
        summary = QtWidgets.QWidget()
        summary.setMinimumWidth(330)
        summary.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        summary_layout = QtWidgets.QVBoxLayout(summary)
        summary_layout.setContentsMargins(0, 0, 0, 0)
        summary_layout.setSpacing(4)
        self.baseline_yaw_status_label = QtWidgets.QLabel(
            "WAITING · Baseline WAITING · P1 WAITING · P2 WAITING"
        )
        self.baseline_yaw_status_label.setObjectName("statusStrip")
        self.baseline_yaw_status_label.setFixedHeight(24)
        summary_layout.addWidget(self.baseline_yaw_status_label)
        values_grid = QtWidgets.QGridLayout()
        values_grid.setContentsMargins(0, 0, 0, 0)
        values_grid.setHorizontalSpacing(8)
        values_grid.setVerticalSpacing(3)
        self.baseline_yaw_values: dict[str, EngineeringValue] = {}
        baseline_yaw_fields = (
            ("baseline_azimuth", "Baseline azimuth", 0, 0),
            ("p2_minus_p1", "P2−P1 yaw", 0, 1),
            ("p1_yaw", "P1 yaw", 1, 0),
            ("baseline_p1", "Baseline−P1", 1, 1),
            ("p2_yaw", "P2 yaw", 2, 0),
            ("baseline_p2", "Baseline−P2", 2, 1),
        )
        for key, label_text, row, column in baseline_yaw_fields:
            value = EngineeringValue(label_text)
            reserve_width = value.value.fontMetrics().horizontalAdvance("-180.00")
            value.value.setMinimumWidth(reserve_width)
            value.activated.connect(
                lambda key=key: self._inspect_baseline_yaw_value(key)
            )
            self.baseline_yaw_values[key] = value
            values_grid.addWidget(value, row, column)
            values_grid.setColumnStretch(column, 1)
        summary_layout.addLayout(values_grid)
        self.baseline_yaw_evidence_label = QtWidgets.QLabel(
            "P1/P2 selected GNSS and P1/P2 ATTITUDE evidence waiting"
        )
        self.baseline_yaw_evidence_label.setObjectName("statusStrip")
        self.baseline_yaw_evidence_label.setWordWrap(True)
        self.baseline_yaw_evidence_label.setFixedHeight(42)
        summary_layout.addWidget(self.baseline_yaw_evidence_label)
        self.baseline_yaw_contract_label = QtWidgets.QLabel(
            "Baseline−Pn = wrap180(azimuth(P1→P2) − Pn yaw), Pn=P1/P2 · + right/CW · horizontal NED"
        )
        self.baseline_yaw_contract_label.setObjectName("compactContract")
        self.baseline_yaw_contract_label.setWordWrap(True)
        self.baseline_yaw_contract_label.setFixedHeight(36)
        summary_layout.addWidget(self.baseline_yaw_contract_label)
        self.baseline_yaw_plot = BaselineYawAlignmentPlot()
        baseline_yaw_group_layout.addWidget(summary, 1)
        baseline_yaw_group_layout.addWidget(self.baseline_yaw_plot, 1)
        self.baseline_yaw_group = baseline_yaw_group
        layout.addWidget(baseline_yaw_group)

        drone_geometry_group = QtWidgets.QGroupBox("Drone Pixhawk geometry · F/R/D/Rg")
        drone_geometry_layout = QtWidgets.QVBoxLayout(drone_geometry_group)
        drone_geometry_layout.setContentsMargins(8, 8, 8, 8)
        drone_geometry_layout.setSpacing(5)
        drone_geometry_hint = QtWidgets.QLabel(
            "Dr = Drone Pixhawk · Dr from reference means reference→Dr · "
            "F/R/D/Rg = Forward/Right/Down/Range (m)"
        )
        drone_geometry_hint.setObjectName("statusStrip")
        drone_geometry_hint.setWordWrap(True)
        drone_geometry_layout.addWidget(drone_geometry_hint)
        self.drone_geometry_model = _table_model(
            self,
            ("State", "Vector", "F (m)", "R (m)", "D (m)", "Rg (m)", "Frame", "Age"),
        )
        self.drone_geometry_table = QtWidgets.QTableView()
        self.drone_geometry_table.setModel(self.drone_geometry_model)
        self.drone_geometry_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.drone_geometry_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.drone_geometry_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.drone_geometry_table.setAlternatingRowColors(True)
        self.drone_geometry_table.verticalHeader().setVisible(False)
        self.drone_geometry_table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.drone_geometry_table.setMinimumHeight(118)
        self.drone_geometry_table.setMaximumHeight(142)
        geometry_header = self.drone_geometry_table.horizontalHeader()
        geometry_header.setStretchLastSection(False)
        geometry_header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        geometry_header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        for column in (2, 3, 4, 5):
            geometry_header.setSectionResizeMode(column, QtWidgets.QHeaderView.Fixed)
            geometry_header.resizeSection(column, 86)
        geometry_header.setSectionResizeMode(6, QtWidgets.QHeaderView.Stretch)
        geometry_header.setSectionResizeMode(7, QtWidgets.QHeaderView.ResizeToContents)
        self.drone_geometry_table.clicked.connect(self._inspect_drone_geometry_vector)
        drone_geometry_layout.addWidget(self.drone_geometry_table)

        drone_angle_title = QtWidgets.QLabel("Horizontal triangle angles (F/R only)")
        drone_angle_title.setObjectName("compactContract")
        drone_geometry_layout.addWidget(drone_angle_title)
        angle_layout = QtWidgets.QGridLayout()
        angle_layout.setContentsMargins(0, 0, 0, 0)
        angle_layout.setHorizontalSpacing(12)
        angle_layout.setVerticalSpacing(3)
        self.drone_geometry_angle_values: dict[str, EngineeringValue] = {}
        for column, (key, label) in enumerate(
            (("dr_p1_p2", "∠Dr-P1-P2"), ("dr_p2_p1", "∠Dr-P2-P1"))
        ):
            value = EngineeringValue(label)
            value.value.setMinimumWidth(value.value.fontMetrics().horizontalAdvance("180.00"))
            value.activated.connect(
                lambda key=key: self._inspect_drone_geometry_angle(key)
            )
            self.drone_geometry_angle_values[key] = value
            angle_layout.addWidget(value, 0, column)
            angle_layout.setColumnStretch(column, 1)
        drone_geometry_layout.addLayout(angle_layout)
        self.drone_geometry_status_label = QtWidgets.QLabel("WAITING · geometry evidence 대기 중")
        self.drone_geometry_status_label.setObjectName("statusStrip")
        self.drone_geometry_status_label.setFixedHeight(32)
        self.drone_geometry_status_label.setWordWrap(False)
        drone_geometry_layout.addWidget(self.drone_geometry_status_label)
        self.drone_geometry_contract_label = QtWidgets.QLabel(
            "Angles: ∠Dr-P1-P2 / ∠Dr-P2-P1 · horizontal North/East only · unsigned internal angle"
        )
        self.drone_geometry_contract_label.setObjectName("compactContract")
        self.drone_geometry_contract_label.setWordWrap(True)
        self.drone_geometry_contract_label.setFixedHeight(32)
        drone_geometry_layout.addWidget(self.drone_geometry_contract_label)
        self.drone_geometry_group = drone_geometry_group
        layout.addWidget(drone_geometry_group)
        layout.addWidget(recovery_group)

        gnss_source_group = QtWidgets.QGroupBox("GNSS used for Pixhawk calculation")
        gnss_source_layout = QtWidgets.QGridLayout(gnss_source_group)
        gnss_source_hint = QtWidgets.QLabel(
            "P1/P2의 GPS1·GPS2는 동시에 표시됩니다. 아래 선택값이 P1→P2 baseline과 측정 계산에 사용됩니다."
        )
        gnss_source_hint.setObjectName("statusStrip")
        gnss_source_hint.setWordWrap(True)
        gnss_source_layout.addWidget(gnss_source_hint, 0, 0, 1, 4)
        self.gnss_source_combos: dict[str, QtWidgets.QComboBox] = {}
        for column, (side, label) in enumerate((("left", "P1 antenna"), ("right", "P2 antenna"))):
            gnss_source_layout.addWidget(QtWidgets.QLabel(label), 1, column * 2)
            combo = QtWidgets.QComboBox()
            combo.addItem("GPS1", "GPS1")
            combo.addItem("GPS2", "GPS2")
            combo.currentIndexChanged.connect(
                lambda _index, side=side: self._set_gnss_source_from_workspace(side)
            )
            self.gnss_source_combos[side] = combo
            gnss_source_layout.addWidget(combo, 1, column * 2 + 1)
        self.gnss_source_summary = QtWidgets.QLabel("계산 source: P1 GPS1 → P2 GPS1")
        self.gnss_source_summary.setObjectName("statusStrip")
        self.gnss_source_summary.setWordWrap(True)
        gnss_source_layout.addWidget(self.gnss_source_summary, 2, 0, 1, 4)
        layout.addWidget(gnss_source_group)
        self.operation_buttons.update(
            {
                "pixhawk.recover_eth_mavlink.p1": self.recovery_buttons["left"],
                "pixhawk.recover_eth_mavlink.p2": self.recovery_buttons["right"],
            }
        )
        self._build_px4_parameter_group(layout)
        layout.addStretch(1)

    @staticmethod
    def _mag_role_for_device_id(device_id: int | None) -> str | None:
        if device_id == BOARD_INTERNAL_MAG_ID:
            return "BOARD"
        if device_id in RM3100_PRIMARY_IDS:
            return "CAN1"
        if device_id in RM3100_BACKUP_IDS:
            return "CAN2"
        if device_id == GPS1_MAG_ID:
            return "GPS1_MAG"
        if device_id == GPS2_MAG_ID:
            return "GPS2_MAG"
        return None

    def _build_mag_priority_group(self, layout: QtWidgets.QVBoxLayout) -> None:
        group = QtWidgets.QGroupBox("MAG Priority / active selection")
        group_layout = QtWidgets.QVBoxLayout(group)
        self._mag_status_auto_refresh_requested: set[str] = set()
        hint = QtWidgets.QLabel(
            "P1/P2는 독립적으로 적용됩니다. CAL_MAGn_PRIO를 직접 수정하며, "
            "Apply 후 PARAM readback, 실제 active device ID, fresh ATTITUDE가 모두 확인되어야 "
            "ACTIVE VERIFIED가 됩니다."
        )
        hint.setObjectName("statusStrip")
        hint.setWordWrap(True)
        group_layout.addWidget(hint)
        cards = QtWidgets.QHBoxLayout()
        self.mag_priority_combos: dict[str, dict[str, QtWidgets.QComboBox]] = {}
        self.mag_priority_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.mag_priority_preview_labels: dict[str, QtWidgets.QLabel] = {}
        self.mag_priority_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.mag_priority_apply_buttons: dict[str, QtWidgets.QPushButton] = {}
        defaults = {key: 0 for key in MAG_PRIORITY_KEYS}
        for side, label, code in (("left", "P1", "p1"), ("right", "P2", "p2")):
            card = QtWidgets.QGroupBox(label)
            card_layout = QtWidgets.QGridLayout(card)
            card_layout.setContentsMargins(6, 6, 6, 6)
            controls = QtWidgets.QHBoxLayout()
            refresh = _command_button(
                "Refresh MAG",
                f"pixhawk.mag.refresh.{code}",
                self._invoke_command,
            )
            apply = _command_button(
                "Apply & Verify",
                f"pixhawk.mag.apply.{code}",
                self._invoke_command,
            )
            refresh.setToolTip("이 side의 CAL_MAG Priority를 다시 읽습니다.")
            apply.setToolTip("이 side만 Priority를 적용하고 실제 active MAG/ATTITUDE를 검증합니다.")
            controls.addWidget(refresh)
            controls.addWidget(apply)
            controls.addStretch(1)
            card_layout.addLayout(controls, 0, 0, 1, 2)
            self.mag_priority_refresh_buttons[side] = refresh
            self.mag_priority_apply_buttons[side] = apply
            combos: dict[str, QtWidgets.QComboBox] = {}
            for row, slot_key in enumerate(MAG_PRIORITY_KEYS, start=1):
                combo = _build_px4_priority_combo(defaults[slot_key])
                combo.setAccessibleName(f"{label} {slot_key} priority")
                combo.setToolTip(
                    f"{slot_key}_PRIO 값을 설정합니다. 센서 identity는 {slot_key}_ID에서 읽습니다."
                )
                card_layout.addWidget(QtWidgets.QLabel(slot_key), row, 0)
                card_layout.addWidget(combo, row, 1)
                combos[slot_key] = combo
            self.mag_priority_combos[side] = combos
            for role, combo in combos.items():
                combo.currentIndexChanged.connect(
                    lambda _index, side=side: self._refresh_mag_priority_preview(side)
                )
            status = QtWidgets.QLabel("PARAM readback 대기 중")
            status.setObjectName("statusStrip")
            status.setWordWrap(True)
            status.setMinimumHeight(42)
            card_layout.addWidget(status, 6, 0, 1, 2)
            self.mag_priority_status_labels[side] = status
            preview = QtWidgets.QLabel("Selection preview: topology readback 대기 중")
            preview.setObjectName("statusStrip")
            preview.setWordWrap(True)
            preview.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            preview.setMinimumHeight(58)
            preview.setMaximumHeight(92)
            card_layout.addWidget(preview, 7, 0, 1, 2)
            self.mag_priority_preview_labels[side] = preview
            cards.addWidget(card, 1)
        group_layout.addLayout(cards)
        self.mag_priority_group = group
        layout.addWidget(group)

    def mag_priority_payload(self, side: str) -> dict[str, int]:
        controls = self.mag_priority_combos.get(str(side), {})
        return {slot_key: _px4_priority_combo_value(combo) for slot_key, combo in controls.items()}

    @staticmethod
    def _mag_preview_role_label(role: str | None) -> str:
        return str(role or "UNKNOWN")

    def _refresh_mag_priority_preview(self, side: str) -> None:
        label = self.mag_priority_preview_labels.get(str(side))
        if label is None:
            return
        panel = getattr(self.backend, "panel", self.backend)
        status_cache = getattr(panel, "_px4_mag_status_cache", {})
        status = status_cache.get(str(side)) if isinstance(status_cache, Mapping) else None
        topology_store = getattr(panel, "latest_sensor_mag_instances", {})
        topology = topology_store.get(str(side), {}) if isinstance(topology_store, Mapping) else {}
        if not isinstance(topology, Mapping) or not topology:
            label.setText("Selection preview: topology readback 대기 중")
            return
        instance_device_ids = {
            int(instance): int(getattr(state, "device_id"))
            for instance, state in topology.items()
            if getattr(state, "device_id", None) is not None
        }
        params = getattr(status, "params", {}) if status is not None else {}
        if not isinstance(params, Mapping):
            label.setText("Selection preview: CAL_MAG PARAM readback 대기 중")
            return
        preview = build_mag_selection_preview(
            params,
            instance_device_ids,
            requested_priorities=self.mag_priority_payload(str(side)),
            actual_selected_device_id=getattr(panel, "latest_vehicle_mag_device_ids", {}).get(str(side)),
        )
        topology_text = " · ".join(
            f"i{candidate.instance}:{candidate.role} id={candidate.device_id} "
            f"CAL_MAG{candidate.cal_slot if candidate.cal_slot is not None else '-'}"
            for candidate in preview.candidates
        )
        if preview.predicted_device_id is None:
            candidate_text = "N/A"
        else:
            predicted_slot = next(
                (
                    candidate.cal_slot
                    for candidate in preview.candidates
                    if candidate.device_id == preview.predicted_device_id
                ),
                None,
            )
            candidate_text = (
                f"{self._mag_preview_role_label(preview.predicted_role)} "
                f"id={preview.predicted_device_id} inst={preview.predicted_instance} "
                f"CAL_MAG{predicted_slot if predicted_slot is not None else '-'} "
                f"prio={preview.predicted_priority}"
            )
            if preview.priority_tie:
                candidate_text += " (tie)"
        actual_text = (
            "N/A"
            if preview.actual_device_id is None
            else f"{self._mag_preview_role_label(preview.actual_role)} id={preview.actual_device_id}"
        )
        label.setText(
            f"Topology: {topology_text}\n"
            f"Priority candidate: {candidate_text}\n"
            f"Actual PX4: {actual_text}"
        )

    def _refresh_mag_priority_group(self, snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        status_cache = getattr(panel, "_px4_mag_status_cache", {})
        open_sides = getattr(panel, "_mavlink_open_sides", None)
        connected = set(open_sides()) if callable(open_sides) else set()
        for side, label, code in (("left", "P1", "p1"), ("right", "P2", "p2")):
            status = status_cache.get(side) if isinstance(status_cache, Mapping) else None
            controls = self.mag_priority_combos.get(side, {})
            if side not in connected:
                self._mag_status_auto_refresh_requested.discard(side)
            elif (
                status is None
                and self._invoke_command_available
                and side not in getattr(panel, "pixhawk_parameter_threads", {})
                and side not in self._mag_status_auto_refresh_requested
            ):
                # The CAL_MAG-slot table and priority combos need a live
                # CAL_MAGn_ID/_PRIO param readback to know which physical
                # sensor sits in which slot. Request it automatically once
                # per connect so the table doesn't sit empty until the
                # operator manually presses "Refresh MAG".
                self._mag_status_auto_refresh_requested.add(side)
                self._invoke_command(f"pixhawk.mag.refresh.{code}")
            if status is not None:
                slots_data = getattr(status, "slots", ()) or ()
                if slots_data:
                    for slot in slots_data:
                        slot_key = mag_priority_slot_key(getattr(slot, "slot", -1))
                        if slot_key in controls:
                            _set_px4_priority_combo_value(controls[slot_key], getattr(slot, "priority", None))
                else:
                    # ``status.slots`` is derived from ``status.params`` (see
                    # build_mag_status). When only the raw params are available
                    # (e.g. before the slots readback lands) fall back to the
                    # same CAL_MAGn_PRIO source so the combos still reflect the
                    # live PX4 state instead of staying at their placeholder 0.
                    params = getattr(status, "params", {})
                    if isinstance(params, Mapping):
                        for slot_number in MAG_SLOTS:
                            slot_key = mag_priority_slot_key(slot_number)
                            if slot_key in controls:
                                _set_px4_priority_combo_value(
                                    controls[slot_key], params.get(f"CAL_MAG{slot_number}_PRIO")
                                )
            verification_reader = getattr(self.backend, "pixhawk_mag_verification_snapshot", None)
            verification = {}
            if callable(verification_reader):
                try:
                    value = verification_reader(side)
                    verification = dict(value) if isinstance(value, Mapping) else {}
                except (RuntimeError, TypeError, ValueError):
                    verification = {}
            if verification.get("verification_started_s") is not None:
                status_text = (
                    f"{label}: {verification.get('state', 'VERIFYING')} · "
                    f"expected={verification.get('expected_device_id', '--')} · "
                    f"actual={verification.get('actual_device_id', '--')} · "
                    f"ATT={_display_number(verification.get('attitude_age_s'), digits=2, suffix='s')}\n"
                    f"{verification.get('reason', '')}"
                )
            elif status is None:
                status_text = f"{label}: PARAM readback 대기 중"
            else:
                active = getattr(panel, "latest_vehicle_mag_device_ids", {}).get(side)
                status_text = (
                    f"{label}: PARAM VERIFIED · actual={active if active is not None else '--'} · "
                    "Apply & Verify를 눌러 active selection을 확인하세요."
                )
            self.mag_priority_status_labels[side].setText(status_text)
            self._refresh_mag_priority_preview(side)
            active_operation = side in getattr(panel, "pixhawk_parameter_threads", {})
            manager = getattr(panel, "px4_mag_managers", {}).get(side)
            armed = bool(getattr(manager, "armed", False))
            enabled = (
                self._invoke_command_available
                and side in connected
                and not active_operation
                and not self._recording_locked
            )
            self.mag_priority_refresh_buttons[side].setEnabled(enabled)
            self.mag_priority_apply_buttons[side].setEnabled(enabled and not armed)
            tooltip = "Disarm required" if armed else ""
            self.mag_priority_apply_buttons[side].setToolTip(tooltip or "이 side만 Priority를 적용하고 검증합니다.")

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

    def _set_transport(self, side: str, combo: QtWidgets.QComboBox) -> None:
        transport = normalize_pixhawk_transport(combo.currentData())
        serial_selected = transport in PIXHAWK_SERIAL_TRANSPORTS
        self.radio_device_edits[side].setEnabled(serial_selected)
        self.radio_baud_spins[side].setEnabled(serial_selected)
        setter = getattr(self.backend, "set_pixhawk_transport", None)
        if callable(setter):
            try:
                setter(side, transport)
            except Exception as exc:
                self.candidate_feedback.setText(f"{side} transport 변경 실패: {exc}")

    def _apply_radio_endpoint(
        self,
        side: str,
        device: QtWidgets.QLineEdit,
        baud: QtWidgets.QSpinBox,
    ) -> None:
        # Radio and USB keep separate saved endpoints, so the shared device
        # row writes whichever one the selected transport actually connects to.
        transport = str(self.transport_combos[side].currentData() or "radio")
        if transport == "ethernet":
            return
        command = "configure_pixhawk_usb" if transport == "usb" else "configure_pixhawk_radio"
        setter = getattr(self.backend, command, None)
        if callable(setter):
            try:
                setter(side, device.text().strip(), int(baud.value()))
            except Exception as exc:
                self.candidate_feedback.setText(
                    f"{side} {transport.upper()} endpoint 적용 실패: {exc}"
                )

    def _connect_role(self, side: str) -> None:
        connector = getattr(self.backend, "connect_pixhawk_role", None)
        if callable(connector):
            try:
                connector(side)
            except Exception as exc:
                self.candidate_feedback.setText(f"{side} Connect 실패: {exc}")

    def _disconnect_role(self, side: str) -> None:
        disconnect = getattr(self.backend, "disconnect_pixhawk_role", None)
        if callable(disconnect):
            try:
                disconnect(side)
            except Exception as exc:
                self.candidate_feedback.setText(f"{side} Disconnect 실패: {exc}")

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
            button.setEnabled(
                self._invoke_command_available
                and connected
                and not active
                and not self._recording_locked
            )
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
                manager = (getattr(panel, "px4_mag_managers", {}) or {}).get(side)
                disarmed = not bool(getattr(manager, "armed", False))
                reset_button.setEnabled(
                    calibration.resettable
                    and self._invoke_command_available
                    and connected
                    and disarmed
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
        panel = getattr(self.backend, "panel", self.backend)
        for side in ("left", "right"):
            transport = str(getattr(panel, f"{side}_pixhawk_transport", "radio") or "radio")
            combo = self.transport_combos[side]
            index = combo.findData(transport)
            if index >= 0 and combo.currentIndex() != index and not combo.hasFocus():
                blocked = combo.blockSignals(True)
                combo.setCurrentIndex(index)
                combo.blockSignals(blocked)
            # Radio and USB save separate endpoints; show the one the selected
            # transport will actually open.
            if transport == "usb":
                device_text = _panel_widget_text(
                    panel, f"{side}_pixhawk_bootstrap_url"
                )
                baud_value = _panel_widget_value(
                    panel, f"{side}_pixhawk_bootstrap_baud", 57_600
                )
            else:
                device_text = str(getattr(panel, f"{side}_pixhawk_radio_device", "") or "")
                baud_value = int(getattr(panel, f"{side}_pixhawk_radio_baud", 57_600))
            edit = self.radio_device_edits[side]
            if not edit.hasFocus() and edit.text() != device_text:
                edit.setText(device_text)
            baud = self.radio_baud_spins[side]
            if not baud.hasFocus() and baud.value() != baud_value:
                blocked = baud.blockSignals(True)
                baud.setValue(baud_value)
                baud.blockSignals(blocked)
        connection = evidence.pixhawk_connection_candidates()
        network_role_active = bool(connection.get("network_role_active", False))
        self._network_role_locked = network_role_active
        role_state, role_detail = _apply_network_role_discovery_status(
            self.discovery_status, connection
        )
        candidates = connection.get("candidates")
        candidates = tuple(row for row in candidates if isinstance(row, dict)) if isinstance(candidates, (tuple, list)) else ()
        discovery_revision = int(connection.get("revision", 0) or 0)
        discovery_replaced = discovery_revision != self._discovery_revision
        if discovery_replaced:
            self._discovery_revision = discovery_revision
            self.candidate_table.clearSelection()
            self.candidate_feedback.clear()
        assignments = connection.get("assignments")
        assignments = assignments if isinstance(assignments, Mapping) else {}
        network_roles = connection.get("network_roles")
        network_roles = network_roles if isinstance(network_roles, Mapping) else {}
        if role_state != "IDLE":
            self.candidate_feedback.setText(f"Network role: {role_state} · {role_detail}")
        for side, label in (("left", "P1"), ("right", "P2")):
            assignment = assignments.get(side)
            assignment = assignment if isinstance(assignment, Mapping) else {}
            state = str(assignment.get("state") or "LEGACY AUTO ROUTE")
            interface_name = str(assignment.get("interface") or "")
            local_address = str(assignment.get("local_address") or "")
            endpoint = (
                getattr(panel, "left_mavlink_url_edit", None)
                if side == "left"
                else getattr(panel, "right_mavlink_url_edit", None)
            )
            endpoint_text = endpoint.text().strip() if endpoint is not None and hasattr(endpoint, "text") else ""
            detail = str(assignment.get("detail") or "")
            valid = (
                True
                if state in {"ASSIGNED", "ROUTE OK"}
                else False
                if state not in {"LEGACY AUTO ROUTE"}
                else None
            )
            values = self.network_assignment_values[side]
            source = detail or f"{label} operator assignment"
            values["state"].set_value(state, source=source, valid=valid)
            values["nic"].set_value(interface_name or "--", source=source)
            values["local"].set_value(local_address or "--", source=source)
            values["endpoint"].set_value(endpoint_text or "--", source=source)
            values["route"].set_value(detail or "--", source=source, valid=valid)

        # Role labels resolve serial symlinks; compute each once per tick and
        # reuse it for both the change check and the rebuild.
        saved_serial_keys = self._saved_serial_endpoint_keys(panel)
        role_labels = tuple(
            self._candidate_role_label(
                row, panel, network_roles, saved_serial_keys=saved_serial_keys
            )
            for row in candidates
        )
        candidate_signature = tuple(
            (*tuple(
                row.get(key, "")
                for key in (
                    "source",
                    "source_row",
                    "candidate_id",
                    "kind",
                    "value",
                    "resolved_device",
                    "baud",
                    "system_id",
                    "component_id",
                    "status",
                    "interface",
                    "carrier",
                    "local_cidr",
                    "p1_route",
                    "p2_route",
                    "p1_endpoint",
                    "p2_endpoint",
                )
            ), role_label)
            for row, role_label in zip(candidates, role_labels)
        )
        if candidate_signature != self._candidate_signature:
            selected = None if discovery_replaced else self._selected_candidate_identity()
            self.candidate_model.removeRows(0, self.candidate_model.rowCount())
            for row, role_label in zip(candidates, role_labels):
                source = str(row.get("source") or "")
                if source == "network":
                    values = (
                        "NETWORK",
                        row.get("interface", "--"),
                        row.get("carrier", "UNKNOWN"),
                        row.get("local_cidr", "--") or "--",
                        role_label,
                    )
                else:
                    kind = str(row.get("kind") or "")
                    values = (
                        "USB" if kind == "USB" else source.upper() or "--",
                        row.get("value", "--"),
                        "MAVLink" if kind == "USB" and row.get("system_id") is not None else kind or "--",
                        row.get("status", "--"),
                        role_label,
                    )
                _append_table_row(
                    self.candidate_model,
                    values,
                )
                model_row = self.candidate_model.rowCount() - 1
                for column, value in enumerate(values):
                    item = self.candidate_model.item(model_row, column)
                    if item is not None:
                        item.setToolTip(str(value))
            self._candidate_signature = candidate_signature
            self._candidate_rows = candidates
            if selected:
                for row_index, row in enumerate(candidates):
                    if self._candidate_identity(row) == selected:
                        self.candidate_table.selectRow(row_index)
                        break
        discovery_active = bool(connection.get("discovery_active", False))
        recording = snapshot.get("recording")
        self._recording_locked = recording is not None and recording.state in {"ACTIVE", "FINALIZING"}
        source_connection_allowed = not self._recording_locked or bool(
            getattr(panel, "_automatic_session_recording", False)
        )
        open_sides = getattr(panel, "_mavlink_open_sides", None)
        connected_sides = set(open_sides()) if callable(open_sides) else set()
        connection_in_progress = getattr(panel, "pixhawk_connection_thread", None) is not None
        status_connected_sides = {
            side
            for side in ("left", "right", "drone")
            if getattr(snapshot.get(f"pixhawk.{side}"), "state", "").upper()
            in {"LIVE", "CONNECTED", "STARTING"}
        }
        connected_sides.update(status_connected_sides)
        status_connected = bool(status_connected_sides)
        connection_active = bool(connected_sides) or connection_in_progress or status_connected
        configured_reader = getattr(panel, "_configured_mavlink_sides", None)
        configured_sides = set(configured_reader()) if callable(configured_reader) else {"left", "right"}
        has_disconnected_target = bool(configured_sides - connected_sides)
        self.operation_buttons["pixhawk.connect_mavlink"].setEnabled(
            self._invoke_command_available
            and source_connection_allowed
            and not connection_in_progress
            and has_disconnected_target
        )
        self.operation_buttons["pixhawk.disconnect_mavlink"].setEnabled(
            self._invoke_command_available and source_connection_allowed and connection_active
        )
        for side in ("left", "right"):
            side_connected = side in connected_sides
            self.role_connect_buttons[side].setEnabled(
                source_connection_allowed
                and not connection_in_progress
                and not side_connected
            )
            self.role_disconnect_buttons[side].setEnabled(
                source_connection_allowed and side_connected
            )
            serial_selected = (
                normalize_pixhawk_transport(self.transport_combos[side].currentData())
                in PIXHAWK_SERIAL_TRANSPORTS
            )
            self.transport_combos[side].setEnabled(
                not side_connected
            )
            self.radio_device_edits[side].setEnabled(
                serial_selected and not side_connected
            )
            self.radio_baud_spins[side].setEnabled(
                serial_selected and not side_connected
            )
        for side in ("left", "right"):
            self.operation_buttons[f"pixhawk.open_shell.{side}"].setEnabled(
                side in connected_sides
                and not self._recording_locked
            )
        reset_level_button = self.operation_buttons.get("pixhawk.reset_level_calibration")
        if reset_level_button is not None:
            reset_level_button.setEnabled(
                not self._recording_locked
            )
        recovery_feedback: list[str] = []
        for side, label in (("left", "P1"), ("right", "P2")):
            recovery = evidence.pixhawk_recovery(side)
            self.recovery_values[side]["usb"].set_value(
                recovery.get("usb_url", "--"),
                source=(
                    f"current heartbeat identity sysid={recovery.get('usb_system_id', '--')} "
                    f"compid={recovery.get('usb_component_id', '--')}"
                ),
            )
            baud = recovery.get("usb_baud")
            self.recovery_values[side]["baud"].set_value(
                baud if baud is not None else "--",
                unit="baud" if baud is not None else "",
                source="verified USB heartbeat baud",
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
                and str(getattr(snapshot.get(f"pixhawk.{side}"), "state", "")).upper()
                == "ETH MAVLINK RX STALLED"
            )
        self.recovery_feedback.setText(" | ".join(recovery_feedback))
        for command_id in ("pixhawk.discover_usb", "pixhawk.discover_mavlink_candidates"):
            self.operation_buttons[command_id].setEnabled(
                not discovery_active
                and not network_role_active
            )
        self._refresh_candidate_actions()
        self._refresh_gnss_source_controls(panel)
        attitude_rows: list[tuple[object, ...]] = []
        receiver_rows: list[tuple[object, ...]] = []
        receiver_keys: list[tuple[str, str]] = []
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
            selected = self._selected_gnss_source(panel, side)
            for source in ("GPS1", "GPS2"):
                gps = evidence.gps(side, source=source)
                marker = " · selected" if source == selected else ""
                receiver_keys.append((side, source))
                receiver_rows.append(
                    (
                        f"{label} {source}{marker}",
                        gps.get("fix_type", "--"),
                        gps.get("rtk_label", gps.get("state", "NO DATA")),
                        gps.get("satellites", "--"),
                        _display_number(gps.get("altitude_m")),
                        _display_number(gps.get("altitude_ellipsoid_m")),
                        f"{_display_number(gps.get('hdop'))}/{_display_number(gps.get('vdop'))}",
                        f"{_display_number(gps.get('h_acc_m'))}/{_display_number(gps.get('v_acc_m'))}",
                        _display_number(gps.get("age_s"), digits=2, suffix="s"),
                    ),
                )
        self._receiver_row_keys = tuple(receiver_keys)
        _sync_table_rows(self.attitude_model, tuple(attitude_rows))
        _sync_table_rows(self.receiver_model, tuple(receiver_rows))
        magnetometer_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            for row in evidence.magnetometers_for_cal_mag_view(side):
                magnetometer_rows.append(
                    (
                        row.get("pixhawk", "--"),
                        row.get("cal_mag", "--"),
                        row.get("sensor", "--"),
                        row.get("device_id", "--"),
                        row.get("state", "WAITING"),
                        _display_number(row.get("xmag")),
                        _display_number(row.get("ymag")),
                        _display_number(row.get("zmag")),
                        _display_number(row.get("norm")),
                        _display_signed_deg(row.get("raw_yaw_deg")),
                        _display_magnetometer_corrected_yaw(row),
                        _display_signed_deg(row.get("delta_yaw_deg")),
                        mag_priority_display(row.get("priority")),
                        row.get("unit", "--"),
                        _display_number(row.get("age_s"), digits=2, suffix="s"),
                        row.get("topic", "--"),
                    )
                )
        _sync_table_rows(self.magnetometer_model, tuple(magnetometer_rows))
        ui_yaw_signature = tuple(
            (
                str(row[0]),
                str(row[1]),
                str(row[9]),
                str(row[10]),
                str(row[11]),
                str(row[15]),
            )
            for row in magnetometer_rows
        )
        if (
            len(ui_yaw_signature) == 8
            and all(row[2] != "--" for row in ui_yaw_signature)
        ):
            previous_signature = getattr(self, "_last_ui_yaw_signature", None)
            if previous_signature != ui_yaw_signature:
                self._last_ui_yaw_signature = ui_yaw_signature
                logger = getattr(panel, "_log", None)
                if callable(logger):
                    logger(
                        "[픽스호크 UI] P1/P2 5-MAG yaw cells populated: "
                        + "; ".join(
                            f"{pixhawk}/{mag} RAW={raw} CORR={px4} Δ={delta} topic={topic}"
                            for pixhawk, mag, raw, px4, delta, topic in ui_yaw_signature
                        )
                    )
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
                        _display_signed_deg(row.get("raw_yaw_deg")),
                        _display_magnetometer_corrected_yaw(row),
                        _display_signed_deg(row.get("delta_yaw_deg")),
                        row.get("unit", "--"),
                        _display_number(row.get("age_s"), digits=2, suffix="s"),
                        row.get("topic", "--"),
                    )
                )
        _sync_table_rows(self.can_magnetometer_model, tuple(can_magnetometer_rows))
        self._refresh_mag_priority_group(snapshot)
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
        self._refresh_baseline_yaw_alignment(evidence.baseline_yaw_alignment())
        self._refresh_drone_pixhawk_geometry(evidence.drone_pixhawk_geometry())

    def _refresh_baseline_yaw_alignment(self, data: Mapping[str, object]) -> None:
        state = str(data.get("state") or "WAITING")
        baseline_state = str(data.get("baseline_state") or "WAITING")
        p1_state = str(data.get("p1_state") or "WAITING")
        p2_state = str(data.get("p2_state") or "WAITING")
        status_text = (
            f"{state} · Baseline {baseline_state} · P1 {p1_state} · P2 {p2_state}"
        )
        self.baseline_yaw_status_label.setText(status_text)

        field_specs = {
            "baseline_azimuth": (
                "baseline_azimuth_deg",
                _display_compass_deg,
                "selected P1/P2 GNSS baseline",
                "local NED horizontal; North=0 deg, clockwise",
            ),
            "p1_yaw": (
                "p1_yaw_deg",
                _display_compass_deg,
                "P1 MAVLink ATTITUDE",
                "P1 body FRD Forward in world NED; compass-normalized",
            ),
            "baseline_p1": (
                "baseline_p1_deg",
                _display_signed_deg,
                "selected GNSS baseline + P1 MAVLink ATTITUDE",
                "wrap180(P1→P2 azimuth - P1 yaw); + right/CW",
            ),
            "p2_yaw": (
                "p2_yaw_deg",
                _display_compass_deg,
                "P2 MAVLink ATTITUDE",
                "P2 body FRD Forward in world NED; compass-normalized",
            ),
            "baseline_p2": (
                "baseline_p2_deg",
                _display_signed_deg,
                "selected GNSS baseline + P2 MAVLink ATTITUDE",
                "wrap180(P1→P2 azimuth - P2 yaw); + right/CW",
            ),
            "p2_minus_p1": (
                "p2_minus_p1_deg",
                _display_signed_deg,
                "P1/P2 MAVLink ATTITUDE",
                "wrap180(P2 yaw - P1 yaw); + clockwise",
            ),
        }
        for key, (data_key, formatter, source, frame) in field_specs.items():
            self.baseline_yaw_values[key].set_value(
                formatter(data.get(data_key)),
                unit="deg",
                source=source,
                frame=frame,
            )

        def age_text(value: object) -> str:
            if value in (None, ""):
                return "--"
            try:
                age_s = float(value)
            except (TypeError, ValueError):
                return "--"
            return "--" if not math.isfinite(age_s) else f"{max(0.0, age_s):.2f}s"

        pair_delta = data.get("gps_pair_delta_ms")
        pair_delta_text = _display_number(pair_delta, digits=1, suffix="ms")
        evidence_text = (
            f"P1 {data.get('p1_gps_source', 'GPS1')} "
            f"fix={data.get('p1_fix_type', '--')}/{data.get('p1_rtk_label', 'NO DATA')} "
            f"GPS={age_text(data.get('p1_gps_age_s'))} ATT={age_text(data.get('p1_attitude_age_s'))} · "
            f"P2 {data.get('p2_gps_source', 'GPS1')} "
            f"fix={data.get('p2_fix_type', '--')}/{data.get('p2_rtk_label', 'NO DATA')} "
            f"GPS={age_text(data.get('p2_gps_age_s'))} ATT={age_text(data.get('p2_attitude_age_s'))} · "
            f"pair Δt={pair_delta_text} · baseline={data.get('baseline_quality', 'NO DATA')}"
        )
        self.baseline_yaw_evidence_label.setText(evidence_text)
        reason = str(data.get("reason") or "all required baseline/yaw evidence current")
        tooltip = (
            f"{status_text}\n{evidence_text}\nReason: {reason}\n"
            f"Clock: {data.get('clock', '')}\nContract: {data.get('angle_contract', '')}"
        )
        self.baseline_yaw_status_label.setToolTip(tooltip)
        self.baseline_yaw_evidence_label.setToolTip(tooltip)
        self.baseline_yaw_contract_label.setToolTip(
            f"{data.get('frame_contract', '')}\n{data.get('angle_contract', '')}\n"
            "Latest-state physical-alignment aid; not simultaneous measurement evidence."
        )
        self.baseline_yaw_plot.set_snapshot(data)

    def _refresh_drone_pixhawk_geometry(self, data: Mapping[str, object]) -> None:
        state = str(data.get("state") or "WAITING")
        reason = str(data.get("reason") or "geometry evidence waiting")
        vector_rows = tuple(
            row for row in (data.get("vectors") or ()) if isinstance(row, Mapping)
        )
        table_rows = tuple(
            (
                str(row.get("state") or "WAITING"),
                str(row.get("label") or "--"),
                _display_signed_number(row.get("forward_m")),
                _display_signed_number(row.get("right_m")),
                _display_signed_number(row.get("down_m")),
                _display_number(row.get("range_m")),
                str(row.get("frame") or "--"),
                _display_number(row.get("age_s"), digits=2, suffix="s"),
            )
            for row in vector_rows
        )
        _sync_table_rows(self.drone_geometry_model, table_rows)
        for row_index, row in enumerate(vector_rows):
            item = self.drone_geometry_model.item(row_index, 1)
            if item is None:
                continue
            item.setToolTip(
                "\n".join(
                    (
                        str(row.get("label") or "--"),
                        f"State: {row.get('state', 'WAITING')}",
                        (
                            "F/R/D/Rg="
                            f"{_display_signed_number(row.get('forward_m'))}/"
                            f"{_display_signed_number(row.get('right_m'))}/"
                            f"{_display_signed_number(row.get('down_m'))}/"
                            f"{_display_number(row.get('range_m'))} m"
                        ),
                        f"Frame: {row.get('frame', '--')}",
                        f"From: {row.get('source', '--')}",
                        f"Reason: {row.get('reason', '--')}",
                    )
                )
            )
            for column in range(self.drone_geometry_model.columnCount()):
                cell = self.drone_geometry_model.item(row_index, column)
                if cell is not None:
                    cell.setToolTip(item.toolTip())

        status_text = f"{state} · {reason}"
        status_tooltip = (
            f"{status_text}\n"
            f"P1 source={data.get('p1_gps_source', 'GPS1')} "
            f"age={_display_number(data.get('p1_gps_age_s'), digits=2, suffix='s')}\n"
            f"P2 source={data.get('p2_gps_source', 'GPS1')} "
            f"age={_display_number(data.get('p2_gps_age_s'), digits=2, suffix='s')}\n"
            f"Dr source=Drone GNSS age={_display_number(data.get('drone_gps_age_s'), digits=2, suffix='s')}\n"
            f"Source: {data.get('source', '')}\n"
            f"Clock: {data.get('clock', '')}"
        )
        self.drone_geometry_status_label.setText(_compact_text(status_text, limit=160))
        self.drone_geometry_status_label.setToolTip(status_tooltip)
        self.drone_geometry_contract_label.setToolTip(
            f"{data.get('frame_contract', '')}\n{data.get('angle_contract', '')}\n"
            "Latest-state display; GNSS/ATTITUDE samples are not simultaneous measurement evidence."
        )

        angle_rows = {
            str(row.get("key")): row
            for row in (data.get("angles") or ())
            if isinstance(row, Mapping)
        }
        for key, value in self.drone_geometry_angle_values.items():
            row = angle_rows.get(key, {})
            value.set_value(
                _display_number(row.get("angle_deg"), digits=2),
                unit="°",
                source=str(row.get("source") or data.get("source") or ""),
                age_s=row.get("age_s"),
                frame=str(row.get("frame") or "horizontal local NED (North/East only)"),
            )
            angle_tooltip = "\n".join(
                (
                    f"State: {row.get('state', 'WAITING')}",
                    f"Value: {_display_number(row.get('angle_deg'), digits=2)}°",
                    f"Vertex: {row.get('vertex', '--')}",
                    f"Frame: {row.get('frame', 'horizontal local NED (North/East only)')}",
                    f"From: {row.get('source', data.get('source', '--'))}",
                    f"Reason: {row.get('reason', reason)}",
                    f"Age: {_display_number(row.get('age_s'), digits=2, suffix='s')}",
                )
            )
            value.setToolTip(angle_tooltip)
            value.value.setToolTip(angle_tooltip)

    def _inspect_drone_geometry_vector(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0:
            return
        data = _evidence(self.backend).drone_pixhawk_geometry()
        rows = tuple(
            row for row in (data.get("vectors") or ()) if isinstance(row, Mapping)
        )
        if index.row() >= len(rows):
            return
        key = str(rows[index.row()].get("key") or "")
        self.inspection_requested.emit(
            _live_inspection(
                self._drone_geometry_vector_inspection(key),
                lambda key=key: self._drone_geometry_vector_inspection(key),
            )
        )

    def _drone_geometry_vector_inspection(self, key: str) -> dict[str, object]:
        data = _evidence(self.backend).drone_pixhawk_geometry()
        rows = tuple(
            row for row in (data.get("vectors") or ()) if isinstance(row, Mapping)
        )
        row = next((row for row in rows if str(row.get("key")) == key), {})
        return {
            "selection": str(row.get("label") or key),
            "value": (
                "F/R/D/Rg="
                f"{_display_signed_number(row.get('forward_m'))}/"
                f"{_display_signed_number(row.get('right_m'))}/"
                f"{_display_signed_number(row.get('down_m'))}/"
                f"{_display_number(row.get('range_m'))} m"
            ),
            "freshness": (
                f"state={row.get('state', 'WAITING')} · "
                f"age={_display_number(row.get('age_s'), digits=2, suffix='s')}"
            ),
            "contract": (
                f"frame={row.get('frame', '--')} · "
                "vector direction is named reference → Dr; F/R/D/Rg are metres"
            ),
            "verdict": str(row.get("state", "WAITING")),
            "source": str(row.get("source", data.get("source", "geometry evidence"))),
        }

    def _inspect_drone_geometry_angle(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._drone_geometry_angle_inspection(key),
                lambda key=key: self._drone_geometry_angle_inspection(key),
            )
        )

    def _drone_geometry_angle_inspection(self, key: str) -> dict[str, object]:
        data = _evidence(self.backend).drone_pixhawk_geometry()
        rows = {
            str(row.get("key")): row
            for row in (data.get("angles") or ())
            if isinstance(row, Mapping)
        }
        row = rows.get(key, {})
        return {
            "selection": str(row.get("label") or key),
            "value": f"{_display_number(row.get('angle_deg'), digits=2)}°",
            "freshness": (
                f"state={row.get('state', 'WAITING')} · "
                f"age={_display_number(row.get('age_s'), digits=2, suffix='s')}"
            ),
            "contract": str(row.get("frame") or data.get("angle_contract") or "horizontal F/R angle"),
            "verdict": str(row.get("state", "WAITING")),
            "source": str(row.get("source", data.get("source", "geometry evidence"))),
        }

    def _inspect_magnetometer_row(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0 or index.row() >= self.magnetometer_model.rowCount():
            return
        rows_per_side = len(MAG_SLOTS)
        side = "left" if index.row() < rows_per_side else "right"
        view_index = index.row() % rows_per_side
        rows = _evidence(self.backend).magnetometers_for_cal_mag_view(side)
        if view_index >= len(rows):
            return
        row = rows[view_index]
        selection = (
            f"{row.get('pixhawk', '--')} {row.get('cal_mag', '--')} · "
            f"{row.get('sensor', 'Magnetometer')}"
        )
        refresh = lambda side=side, source_index=view_index: self._cal_mag_magnetometer_inspection(
            side, source_index
        )
        fallback_source = "PX4 CAL_MAG slot readback"
        self.inspection_requested.emit(
            _live_inspection(
                {
                    "selection": selection,
                    "value": (
                        f"x/y/z={_display_number(row.get('xmag'))}/"
                        f"{_display_number(row.get('ymag'))}/"
                        f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                        + (
                            f"node={row.get('node_id', '--')} device={row.get('device_id', '--')} · "
                            if row.get('node_id') is not None
                            else f"device={row.get('device_id', '--')} · "
                        )
                        + f"norm={_display_number(row.get('norm'))} · "
                        + f"{_magnetometer_yaw_summary(row)}"
                    ),
                    "freshness": (
                        f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                        f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
                    ),
                    "contract": (
                        f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                        f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                        f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
                    ),
                    "verdict": (
                        f"{row.get('state', 'WAITING')} · "
                        f"Corrected {row.get('corrected_yaw_state', 'WAITING')}"
                    ),
                    "source": str(row.get("source", fallback_source)),
                },
                refresh,
            )
        )

    def _cal_mag_magnetometer_inspection(self, side: str, slot_index: int) -> dict[str, object]:
        rows = _evidence(self.backend).magnetometers_for_cal_mag_view(side)
        if slot_index < 0 or slot_index >= len(rows):
            return {"selection": "CAL_MAG", "verdict": "WAITING"}
        row = rows[slot_index]
        return {
            "selection": (
                f"{row.get('pixhawk', '--')} {row.get('cal_mag', '--')} · "
                f"{row.get('sensor', 'Magnetometer')}"
            ),
            "value": (
                f"x/y/z={_display_number(row.get('xmag'))}/"
                f"{_display_number(row.get('ymag'))}/"
                f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                f"device={row.get('device_id', '--')} · "
                f"{_magnetometer_yaw_summary(row)}"
            ),
            "freshness": (
                f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                f"Priority={mag_priority_display(row.get('priority'))} · "
                f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
            ),
            "contract": (
                f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
            ),
            "verdict": (
                f"{row.get('state', 'WAITING')} · "
                f"Corrected {row.get('corrected_yaw_state', 'WAITING')}"
            ),
            "source": str(row.get("source", "PX4 CAL_MAG slot readback")),
        }

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
                        f"node={row.get('node_id', '--')} device={row.get('device_id', '--')} · "
                        f"{_magnetometer_yaw_summary(row)}"
                    ),
                    "freshness": (
                        f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                        f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
                    ),
                    "contract": (
                        f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                        f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                        f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
                    ),
                    "verdict": (
                        f"{row.get('state', 'WAITING')} · "
                        f"Corrected {row.get('corrected_yaw_state', 'WAITING')}"
                    ),
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
                f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                f"{_magnetometer_yaw_summary(row)}"
            ),
            "freshness": (
                f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
            ),
            "contract": (
                f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
            ),
            "verdict": (
                f"{row.get('state', 'WAITING')} · "
                f"Corrected {row.get('corrected_yaw_state', 'WAITING')}"
            ),
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
                f"{_display_number(row.get('zmag'))} {row.get('unit', '')} · "
                f"{_magnetometer_yaw_summary(row)}"
            ),
            "freshness": (
                f"MAG age={_display_number(row.get('age_s'), digits=2, suffix='s')} · "
                f"Corrected={row.get('corrected_yaw_state', 'WAITING')}"
            ),
            "contract": (
                f"{row.get('axis_contract', 'producer sensor axes; no coordinate conversion')} · "
                f"{row.get('raw_yaw_contract', 'RAW sensor-axis azimuth; no correction')} · "
                f"{row.get('corrected_yaw_contract', 'CAL_MAG corrected sensor vector')}"
            ),
            "verdict": (
                f"{row.get('state', 'WAITING')} · "
                f"Corrected {row.get('corrected_yaw_state', 'WAITING')}"
            ),
            "source": str(row.get("source", "MAVLink SCALED_IMU telemetry")),
        }

    def _selected_candidate_row(self) -> int:
        selected = self.candidate_table.selectionModel().selectedRows()
        return selected[0].row() if selected else -1

    @staticmethod
    def _serial_endpoint_key(value: object) -> str:
        text = str(value or "").strip()
        if not text:
            return ""
        try:
            return str(Path(text).resolve(strict=False))
        except (OSError, RuntimeError, ValueError):
            return text

    @classmethod
    def _saved_serial_endpoint_keys(cls, panel: object) -> tuple[tuple[str, str], ...]:
        """Resolve each side's saved serial device once for a whole refresh."""

        return tuple(
            (label, cls._serial_endpoint_key(getattr(panel, f"{side}_pixhawk_radio_device", "")))
            for side, label in _PIXHAWK_SIDE_LABELS
        )

    @classmethod
    def _candidate_role_label(
        cls,
        candidate: Mapping[str, object],
        panel: object,
        network_roles: Mapping[str, object],
        *,
        saved_serial_keys: Sequence[tuple[str, str]] | None = None,
    ) -> str:
        if str(candidate.get("source") or "") == "network":
            return _network_role_column_text(
                candidate_network_role_labels(candidate, network_roles)
            )

        if str(candidate.get("kind") or "") != "USB":
            return "--"
        candidate_keys = {
            cls._serial_endpoint_key(candidate.get("value")),
            cls._serial_endpoint_key(candidate.get("resolved_device")),
        }
        candidate_keys.discard("")
        if saved_serial_keys is None:
            saved_serial_keys = cls._saved_serial_endpoint_keys(panel)
        saved_roles = [label for label, key in saved_serial_keys if key and key in candidate_keys]
        system_id = candidate.get("system_id")
        try:
            observed_system_id = int(system_id) if system_id is not None else None
        except (TypeError, ValueError):
            observed_system_id = None
        observed_role = _PIXHAWK_SYSTEM_ID_LABELS.get(observed_system_id)
        if observed_system_id is not None:
            observed_text = (
                f"{observed_role} (sysid {observed_system_id})"
                if observed_role
                else f"Unknown (sysid {observed_system_id})"
            )
            if saved_roles:
                saved_text = ", ".join(saved_roles)
                if saved_roles == [observed_role]:
                    return f"{observed_role} saved · sysid {observed_system_id}"
                if observed_role not in saved_roles:
                    return f"{saved_text} saved ≠ {observed_text}"
                return f"{saved_text} saved · {observed_text}"
            return observed_text
        if saved_roles:
            return f"{', '.join(saved_roles)} saved · identity unverified"
        return "UNASSIGNED · identity unverified"

    @staticmethod
    def _candidate_identity(candidate: Mapping[str, object]) -> tuple[str, str, str]:
        source = str(candidate.get("source", ""))
        if source == "network":
            return source, "NETWORK", str(candidate.get("candidate_id", ""))
        return source, str(candidate.get("kind", "")), str(candidate.get("value", ""))

    def _selected_candidate_identity(self) -> tuple[str, str, str] | None:
        row = self._selected_candidate_row()
        if not 0 <= row < len(self._candidate_rows):
            return None
        return self._candidate_identity(self._candidate_rows[row])

    def _refresh_candidate_actions(self, *_args: object) -> None:
        row = self._selected_candidate_row()
        candidate = self._candidate_rows[row] if 0 <= row < len(self._candidate_rows) else {}
        kind = str(candidate.get("kind", ""))
        source = str(candidate.get("source", ""))
        endpoint_enabled = source == "network" or (
            source == "mavlink" and kind in {"USB", "MAVLink"}
        )
        self.candidate_buttons["left"].setEnabled(
            not self._recording_locked
            and not self._network_role_locked
            and endpoint_enabled
        )
        self.candidate_buttons["right"].setEnabled(
            not self._recording_locked
            and not self._network_role_locked
            and endpoint_enabled
        )

    def _apply_candidate(self, target: str) -> None:
        row = self._selected_candidate_row()
        if not 0 <= row < len(self._candidate_rows):
            self.candidate_feedback.setText("ERROR: select a compatible endpoint")
            return
        candidate = self._candidate_rows[row]
        source = str(candidate.get("source", ""))
        if source == "network":
            method = getattr(self.backend, "pixhawk_assign_network_candidate", None)
            args = (str(candidate.get("candidate_id", "")), target)
        else:
            method = getattr(self.backend, "pixhawk_apply_candidate", None)
            args = (source, int(candidate.get("source_row", -1)), target)
        if not callable(method):
            self.candidate_feedback.setText("ERROR: Pixhawk endpoint adapter unavailable")
            return
        try:
            applied = method(*args)
        except Exception as exc:
            self.candidate_feedback.setText(f"ERROR: {exc}")
            return
        if applied is False:
            self.candidate_feedback.setText("ERROR: endpoint was not applied")
            return
        self.candidate_feedback.setText(
            f"Requested {candidate.get('kind', 'endpoint')} for {target.upper()} · follow role status"
        )
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
        source = str(candidate.get("source") or "")
        is_usb = str(candidate.get("kind") or "") == "USB"
        return {
            "selection": f"Pixhawk endpoint · {candidate.get('kind', '--')}",
            "value": (
                f"NIC/device={candidate.get('interface') or candidate.get('value', '--')} · "
                + (
                    f"sysid={candidate.get('system_id') or '--'} · "
                    f"compid={candidate.get('component_id') or '--'} · baud={candidate.get('baud') or '--'}"
                    if is_usb
                    else f"local={candidate.get('local_cidr') or '--'} · "
                    "Assign P1/P2 to inspect that side's endpoint and route"
                )
            ),
            "freshness": "Latest completed Pixhawk endpoint discovery",
            "contract": (
                "Find USB is read-only serial/heartbeat identity evidence and never assigns or connects"
                if is_usb
                else "Find network is read-only NIC/address/kernel-route metadata; confirmed Assign owns only "
                "the selected role's NetworkManager delta; no attitude/GPS/baseline coordinate values"
            ),
            "verdict": candidate.get("status") or "UNVERIFIED",
            "source": (
                f"Pixhawk {source or 'candidate'} discovery · "
                f"mac={candidate.get('mac') or '--'} · driver={candidate.get('driver') or '--'} · "
                f"path={candidate.get('resolved_device') or candidate.get('device_path') or '--'}"
            ),
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
            "hide_freshness": True,
            "contract": str(data.get("orientation_contract", "PX4 body FRD → world NED; R/P/Y degrees")),
            "verdict": str(data.get("state", "WAITING")),
            "source": str(data.get("source", "MAVLink attitude telemetry")),
        }

    def _inspect_receiver_row(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0 or index.row() >= len(self._receiver_row_keys):
            return
        side, source = self._receiver_row_keys[index.row()]
        self.inspection_requested.emit(
            _live_inspection(
                self._receiver_inspection(side, source),
                lambda side=side, source=source: self._receiver_inspection(side, source),
            )
        )

    def _receiver_inspection(self, side: str, source: str = "GPS1") -> dict[str, object]:
        label = "P1" if side == "left" else "P2"
        data = _evidence(self.backend).gps(side, source=source)
        return {
            "selection": f"{label} {source} GNSS receiver",
            "value": (
                f"fix number={data.get('fix_type', '--')} · {data.get('rtk_label', 'NO DATA')} · "
                f"sats={data.get('satellites', '--')} · alt MSL={data.get('altitude_m', '--')} m · "
                f"HDOP/VDOP={data.get('hdop', '--')}/{data.get('vdop', '--')} · "
                f"H/V acc={data.get('h_acc_m', '--')}/{data.get('v_acc_m', '--')} m"
            ),
            "freshness": f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}",
            "contract": str(data.get("position_contract", "WGS84 latitude/longitude degrees; altitude metres")),
            "verdict": str(data.get("state", "NO DATA")),
            "source": str(data.get("source", "MAVLink GPS telemetry")),
        }

    @staticmethod
    def _selected_gnss_source(panel: object, side: str) -> str:
        reader = getattr(panel, "_pixhawk_gnss_source", None)
        if callable(reader):
            return normalize_pixhawk_gnss_source(reader(side))
        settings = getattr(panel, "settings", None)
        key = "pixhawk_left_gnss_source" if side == "left" else "pixhawk_right_gnss_source"
        return normalize_pixhawk_gnss_source(getattr(settings, key, "GPS1"))

    def _refresh_gnss_source_controls(self, panel: object) -> None:
        selected: dict[str, str] = {}
        for side, combo in self.gnss_source_combos.items():
            source = self._selected_gnss_source(panel, side)
            selected[side] = source
            index = combo.findData(source)
            if index >= 0 and combo.currentIndex() != index:
                blocked = combo.blockSignals(True)
                combo.setCurrentIndex(index)
                combo.blockSignals(blocked)
        self.gnss_source_summary.setText(
            f"계산 source: P1 {selected.get('left', 'GPS1')} → P2 {selected.get('right', 'GPS1')}"
        )

    def _set_gnss_source_from_workspace(self, side: str) -> None:
        combo = self.gnss_source_combos.get(side)
        if combo is None:
            return
        source = normalize_pixhawk_gnss_source(combo.currentData())
        panel = getattr(self.backend, "panel", self.backend)
        setter = getattr(panel, "set_pixhawk_gnss_source", None)
        if callable(setter):
            setter(side, source)
        self._refresh_gnss_source_controls(panel)

    def _inspect_baseline_value(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._baseline_inspection(key),
                lambda key=key: self._baseline_inspection(key),
            )
        )

    def _inspect_baseline_yaw_value(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._baseline_yaw_inspection(key),
                lambda key=key: self._baseline_yaw_inspection(key),
            )
        )

    def _baseline_yaw_inspection(self, key: str) -> dict[str, object]:
        data = _evidence(self.backend).baseline_yaw_alignment()
        fields = {
            "baseline_azimuth": ("baseline_azimuth_deg", _display_compass_deg, "baseline_state"),
            "p1_yaw": ("p1_yaw_deg", _display_compass_deg, "p1_state"),
            "baseline_p1": ("baseline_p1_deg", _display_signed_deg, "p1_state"),
            "p2_yaw": ("p2_yaw_deg", _display_compass_deg, "p2_state"),
            "baseline_p2": ("baseline_p2_deg", _display_signed_deg, "p2_state"),
            "p2_minus_p1": ("p2_minus_p1_deg", _display_signed_deg, "state"),
        }
        data_key, formatter, state_key = fields[key]
        return {
            "selection": self.baseline_yaw_values[key].label.text(),
            "value": f"{formatter(data.get(data_key))} deg",
            "freshness": (
                f"baseline={data.get('baseline_state', 'WAITING')} "
                f"P1={data.get('p1_state', 'WAITING')} "
                f"P2={data.get('p2_state', 'WAITING')} · "
                f"GNSS pair Δt={_display_number(data.get('gps_pair_delta_ms'), digits=1, suffix='ms')}"
            ),
            "contract": str(data.get("angle_contract", "horizontal local NED angle")),
            "verdict": str(data.get(state_key, "WAITING")),
            "source": str(data.get("source", "selected GNSS + MAVLink ATTITUDE")),
        }

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
        self._endpoint_dirty = False
        self.endpoint_edit.textEdited.connect(lambda _text: setattr(self, "_endpoint_dirty", True))
        # This workspace owns the Drone "drone" MAVLink side outright, so the
        # transport selector lives here rather than on the Pixhawk workspace.
        self.transport_combo = QtWidgets.QComboBox()
        self.transport_combo.addItem("Drone Telemetry Radio", "radio")
        self.transport_combo.addItem("Ethernet", "ethernet")
        self.transport_combo.addItem("USB", "usb")
        transport = normalize_pixhawk_transport(defaults.get("drone_pixhawk_transport"))
        self.transport_combo.setCurrentIndex(max(0, self.transport_combo.findData(transport)))
        # An operator's transport choice is only pushed to the backend on
        # Configure/Connect/Save, so refresh() must not snap the combo back to
        # the backend's last-applied transport in the meantime.
        self._transport_dirty = False
        self.transport_combo.currentIndexChanged.connect(self._on_transport_changed)
        self.serial_device_edit = QtWidgets.QLineEdit(
            str(
                defaults.get(
                    "drone_pixhawk_bootstrap_url"
                    if transport == "usb"
                    else "drone_pixhawk_radio_device",
                    "",
                )
                or ""
            )
        )
        self.serial_device_edit.setPlaceholderText("/dev/serial/by-id/...")
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
        connection_form.addWidget(QtWidgets.QLabel("Transport"), 0, 0)
        connection_form.addWidget(self.transport_combo, 0, 1)
        connection_form.addWidget(QtWidgets.QLabel("MAVLink URL"), 1, 0)
        connection_form.addWidget(self.endpoint_edit, 1, 1, 1, 3)
        connection_form.addWidget(QtWidgets.QLabel("Serial device"), 2, 0)
        connection_form.addWidget(self.serial_device_edit, 2, 1, 1, 3)
        connection_form.addWidget(QtWidgets.QLabel("Baud"), 3, 0)
        connection_form.addWidget(self.baud_spin, 3, 1)
        connection_form.addWidget(QtWidgets.QLabel("RTCM source"), 4, 0)
        connection_form.addWidget(self.rtcm_source_combo, 4, 1)
        connection_form.addWidget(self.forward_rtcm_checkbox, 4, 2, 1, 2)
        actions = QtWidgets.QHBoxLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("telemetry.discover_usb", "Find USB"),
            ("telemetry.discover_network", "Find Network"),
            ("telemetry.apply", "Apply"),
            ("telemetry.connect", "Connect"),
            ("telemetry.disconnect", "Disconnect"),
            ("telemetry.save_settings", "Save"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            actions.addWidget(button)
        actions.addStretch(1)
        connection_form.addLayout(actions, 5, 0, 1, 4)
        self.connection_status = QtWidgets.QLabel("DISCONNECTED")
        self.connection_status.setObjectName("statusStrip")
        self.connection_status.setWordWrap(True)
        connection_form.addWidget(self.connection_status, 6, 0, 1, 4)
        layout.addWidget(connection_group)

        discovery_group = QtWidgets.QGroupBox("Drone network candidates")
        discovery_layout = QtWidgets.QVBoxLayout(discovery_group)
        self.network_candidate_model = _table_model(
            self, ("State", "NIC", "Source IP", "UDP port", "System ID", "Heartbeats")
        )
        self.network_candidate_table = QtWidgets.QTableView()
        self.network_candidate_table.setModel(self.network_candidate_model)
        self.network_candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.network_candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.network_candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.network_candidate_table.verticalHeader().setVisible(False)
        self.network_candidate_table.setFixedHeight(92)
        self.network_candidate_table.horizontalHeader().setStretchLastSection(True)
        discovery_layout.addWidget(self.network_candidate_table)
        discovery_actions = QtWidgets.QHBoxLayout()
        self.network_discovery_status = QtWidgets.QLabel("IDLE")
        self.network_discovery_status.setObjectName("statusStrip")
        self.network_discovery_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        discovery_actions.addWidget(self.network_discovery_status, 1)
        use_button = _command_button(
            "Use selected", "telemetry.use_network_candidate", invoke_command
        )
        self.operation_buttons["telemetry.use_network_candidate"] = use_button
        discovery_actions.addWidget(use_button)
        discovery_layout.addLayout(discovery_actions)
        layout.addWidget(discovery_group)
        self._network_candidate_ids: list[str] = []
        self._network_candidate_signature: tuple[tuple[object, ...], ...] = ()
        self._refresh_transport_fields()

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

        gnss_center_group = QtWidgets.QGroupBox("GNSS BASELINE_CENTER F/R/D/Rg/Angle")
        gnss_center_layout = QtWidgets.QVBoxLayout(gnss_center_group)
        gnss_center_layout.setContentsMargins(6, 6, 6, 6)
        gnss_center_layout.setSpacing(4)
        self.gnss_baseline_center_model = _table_model(
            self,
            (
                "State",
                "Forward m",
                "Right m",
                "Down m",
                "Range Rg m",
                "Baseline↔Drone angle°",
                "90° error°",
                "Max age s",
                "Max Δt ms",
                "Fix P1/P2/Drone",
            ),
        )
        self.gnss_baseline_center_table = QtWidgets.QTableView()
        self.gnss_baseline_center_table.setModel(self.gnss_baseline_center_model)
        self.gnss_baseline_center_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.gnss_baseline_center_table.setSelectionMode(
            QtWidgets.QAbstractItemView.NoSelection
        )
        self.gnss_baseline_center_table.verticalHeader().setVisible(False)
        self.gnss_baseline_center_table.verticalHeader().setDefaultSectionSize(26)
        self.gnss_baseline_center_table.setHorizontalScrollMode(
            QtWidgets.QAbstractItemView.ScrollPerPixel
        )
        self.gnss_baseline_center_table.setFixedHeight(68)
        gnss_center_header = self.gnss_baseline_center_table.horizontalHeader()
        gnss_center_header.setStretchLastSection(True)
        gnss_center_header.setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        for column, width in enumerate((90, 105, 105, 105, 110, 145, 105, 90, 95, 230)):
            gnss_center_header.resizeSection(column, width)
        gnss_center_layout.addWidget(self.gnss_baseline_center_table)
        self.gnss_baseline_center_status = QtWidgets.QLabel(
            "WAITING · P1/P2/Drone GNSS position waiting"
        )
        self.gnss_baseline_center_status.setObjectName("statusStrip")
        self.gnss_baseline_center_status.setFixedHeight(24)
        self.gnss_baseline_center_status.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse
        )
        gnss_center_layout.addWidget(self.gnss_baseline_center_status)
        self.gnss_baseline_center_contract = QtWidgets.QLabel(
            "GNSS-derived · origin=P1/P2 antenna midpoint · R=horizontal P1→P2 · "
            "D=NED Down · F=R×D · angle=atan2(|F|,|R|) · target=90° · "
            "not camera baseline_center_frd"
        )
        self.gnss_baseline_center_contract.setObjectName("compactContract")
        self.gnss_baseline_center_contract.setWordWrap(True)
        self.gnss_baseline_center_contract.setMaximumHeight(96)
        self.gnss_baseline_center_contract.setToolTip(
            "This is a GNSS antenna-midpoint local-level reference. It does not include "
            "camera optical centers, camera/body attitude, or antenna-to-camera lever arms."
        )
        gnss_center_layout.addWidget(self.gnss_baseline_center_contract)
        self.gnss_baseline_center_group = gnss_center_group
        layout.addWidget(gnss_center_group)

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
        self.rtcm_group = rtcm_group

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

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt API.
        super().resizeEvent(event)
        contract = getattr(self, "gnss_baseline_center_contract", None)
        if contract is None:
            return
        available_width = max(240, int(event.size().width()) - 32)
        required_height = contract.heightForWidth(available_width)
        if required_height < 0:
            required_height = contract.sizeHint().height()
        reserved_height = max(24, min(96, int(required_height)))
        if contract.height() != reserved_height:
            contract.setFixedHeight(reserved_height)

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
            "drone_pixhawk_transport": normalize_pixhawk_transport(self.transport_combo.currentData()),
            "drone_telemetry_url": self.endpoint_edit.text().strip(),
            "drone_telemetry_baud": int(self.baud_spin.value()),
            "drone_pixhawk_serial_device": self.serial_device_edit.text().strip(),
            "drone_pixhawk_serial_baud": int(self.baud_spin.value()),
            "drone_telemetry_rtcm_source": str(self.rtcm_source_combo.currentData() or "base"),
            "drone_telemetry_rtcm_forward": bool(self.forward_rtcm_checkbox.isChecked()),
        }

    def _serial_transport_selected(self) -> bool:
        return normalize_pixhawk_transport(self.transport_combo.currentData()) in PIXHAWK_SERIAL_TRANSPORTS

    def _on_transport_changed(self, _index: int) -> None:
        self._transport_dirty = True
        self._refresh_transport_fields()

    def _refresh_transport_fields(self) -> None:
        """Show only the endpoint field the selected transport actually opens."""

        serial_selected = self._serial_transport_selected()
        self.endpoint_edit.setEnabled(not serial_selected)
        self.serial_device_edit.setEnabled(serial_selected)

    def accept_endpoint(self, result: object) -> None:
        data = dict(result) if isinstance(result, Mapping) else {}
        target_edit = (
            self.serial_device_edit if self._serial_transport_selected() else self.endpoint_edit
        )
        endpoint = str(data.get("endpoint") or target_edit.text()).strip()
        target_edit.setText(endpoint)
        self._endpoint_dirty = False
        self._transport_dirty = False

    def set_candidate_endpoint(self, result: object) -> None:
        data = dict(result) if isinstance(result, Mapping) else {}
        endpoint = str(data.get("endpoint") or "").strip()
        if not endpoint:
            raise ValueError("Selected Drone network candidate has no endpoint")
        self.endpoint_edit.setText(endpoint)
        self._endpoint_dirty = True

    def selected_network_candidate_id(self) -> str:
        selection = self.network_candidate_table.selectionModel().selectedRows()
        if len(selection) != 1:
            raise ValueError("Select one Drone network candidate")
        row = selection[0].row()
        if not 0 <= row < len(self._network_candidate_ids):
            raise ValueError("Selected Drone network candidate is stale")
        return self._network_candidate_ids[row]

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
        transport = normalize_pixhawk_transport(
            data.get("transport", data.get("drone_pixhawk_transport"))
        )
        transport_index = self.transport_combo.findData(transport)
        # Keyboard focus alone is not an edit: the combo is the first widget in
        # tab order and holds focus right after the page is shown. Only an
        # operator change (the dirty flag) keeps the backend value out.
        if (
            transport_index >= 0
            and transport_index != self.transport_combo.currentIndex()
            and not self._transport_dirty
        ):
            blocked = self.transport_combo.blockSignals(True)
            self.transport_combo.setCurrentIndex(transport_index)
            self.transport_combo.blockSignals(blocked)
        # The snapshot reports the endpoint of the backend's active transport;
        # only route it into the field the operator is currently editing for.
        serial_selected = self._serial_transport_selected()
        target_edit = self.serial_device_edit if serial_selected else self.endpoint_edit
        if (
            endpoint
            and not self._endpoint_dirty
            and not self._transport_dirty
            and not target_edit.hasFocus()
            and target_edit.text() != endpoint
        ):
            target_edit.setText(endpoint)
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
        network_discovery_active = bool(data.get("network_discovery_active"))
        self._refresh_transport_fields()
        self.operation_buttons["telemetry.connect"].setEnabled(
            self._invoke_command_available and not connection_active
        )
        self.operation_buttons["telemetry.disconnect"].setEnabled(
            self._invoke_command_available and connection_active
        )
        self.operation_buttons["telemetry.discover_usb"].setEnabled(
            self._invoke_command_available
            and not usb_discovery_active
        )
        self.operation_buttons["telemetry.discover_network"].setEnabled(
            self._invoke_command_available
            and not network_discovery_active
        )
        for command_id in ("telemetry.apply", "telemetry.save_settings"):
            self.operation_buttons[command_id].setEnabled(
                self._invoke_command_available
            )
        self._refresh_network_candidates(data)
        self._refresh_position_table(data)
        self._refresh_gnss_baseline_center(data)
        self._refresh_rtcm(data)
        self._refresh_disparity(data)

    def _refresh_network_candidates(self, data: Mapping[str, object]) -> None:
        rows = [dict(row) for row in (data.get("network_candidates") or ()) if isinstance(row, Mapping)]
        signature = tuple(
            (
                row.get("candidate_id"), row.get("state"), row.get("interface"),
                row.get("source_ip"), row.get("listen_port"), row.get("system_id"),
                row.get("heartbeat_count"), bool(row.get("selectable")),
            )
            for row in rows
        )
        if signature != self._network_candidate_signature:
            self.network_candidate_model.removeRows(0, self.network_candidate_model.rowCount())
            self._network_candidate_ids = []
            for row in rows:
                values = (
                    row.get("state", "--"), row.get("interface") or "--",
                    row.get("source_ip") or "--", row.get("listen_port") or "--",
                    row.get("system_id") or "--", row.get("heartbeat_count") or 0,
                )
                self.network_candidate_model.appendRow(
                    [QtGui.QStandardItem(str(value)) for value in values]
                )
                self._network_candidate_ids.append(str(row.get("candidate_id") or ""))
            self._network_candidate_signature = signature
        state = str(data.get("network_discovery_state") or "IDLE")
        self.network_discovery_status.setText(state)
        selected = self.network_candidate_table.selectionModel().selectedRows()
        selectable = False
        if len(selected) == 1 and 0 <= selected[0].row() < len(rows):
            selectable = bool(rows[selected[0].row()].get("selectable"))
        self.operation_buttons["telemetry.use_network_candidate"].setEnabled(
            self._invoke_command_available
            and selectable
            and not bool(data.get("network_discovery_active"))
        )

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

    @staticmethod
    def _gnss_center_axis_text(value: object) -> str:
        if value in (None, ""):
            return "--"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "--"
        return f"{number:+.3f}" if math.isfinite(number) else "--"

    @staticmethod
    def _gnss_center_unsigned_text(value: object, *, digits: int) -> str:
        if value in (None, ""):
            return "--"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "--"
        return f"{number:.{digits}f}" if math.isfinite(number) else "--"

    def _refresh_gnss_baseline_center(self, data: Mapping[str, object]) -> None:
        geometry = data.get("gnss_baseline_center_frd")
        geometry = geometry if isinstance(geometry, Mapping) else {}
        state = str(geometry.get("state") or "WAITING").upper()
        reason = str(
            geometry.get("reason") or "P1/P2/Drone GNSS baseline-center evidence waiting"
        )
        fix_labels = geometry.get("fix_labels")
        fix_labels = fix_labels if isinstance(fix_labels, Mapping) else {}
        fix_text = " / ".join(
            (
                str(fix_labels.get("p1") or "--"),
                str(fix_labels.get("p2") or "--"),
                str(fix_labels.get("drone") or "--"),
            )
        )
        row = (
            state,
            self._gnss_center_axis_text(geometry.get("forward_m")),
            self._gnss_center_axis_text(geometry.get("right_m")),
            self._gnss_center_axis_text(geometry.get("down_m")),
            self._gnss_center_unsigned_text(geometry.get("range_m"), digits=3),
            self._gnss_center_unsigned_text(
                geometry.get("baseline_drone_angle_deg"),
                digits=2,
            ),
            self._gnss_center_unsigned_text(
                geometry.get("baseline_drone_angle_error_deg"),
                digits=2,
            ),
            self._gnss_center_unsigned_text(geometry.get("maximum_age_s"), digits=2),
            self._gnss_center_unsigned_text(
                geometry.get("timestamp_spread_ms"),
                digits=1,
            ),
            fix_text,
        )
        _sync_table_rows(self.gnss_baseline_center_model, (row,))
        frame = str(
            geometry.get("frame") or "gnss_baseline_center_local_level_frd"
        )
        tooltip = f"{state}: {reason}\nFrame: {frame}"
        for column in range(self.gnss_baseline_center_model.columnCount()):
            item = self.gnss_baseline_center_model.item(0, column)
            if item is None:
                continue
            item.setToolTip(tooltip)
            if 1 <= column <= 8:
                item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        angle_text = self._gnss_center_unsigned_text(
            geometry.get("baseline_drone_angle_deg"),
            digits=2,
        )
        angle_error_text = self._gnss_center_unsigned_text(
            geometry.get("baseline_drone_angle_error_deg"),
            digits=2,
        )
        angle_detail = (
            ""
            if angle_text == "--"
            else f" · angle={angle_text}° · 90° error={angle_error_text}°"
        )
        full_status = f"{state} · {reason}{angle_detail}"
        self.gnss_baseline_center_status.setText(_compact_text(full_status, limit=180))
        self.gnss_baseline_center_status.setToolTip(full_status)

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

    # Ordered acquisition chain for NETWORK_ASSISTED_FIXED_BASE / NETWORK_RTK_DIRECT:
    # position in this tuple is the stage number shown to the user. States outside
    # this chain (fault/terminal/other-mode states) are labelled but not numbered.
    ACQUISITION_STAGES: tuple[tuple[str, str], ...] = (
        # DISCONNECTED/CONFIGURED describe the operation-state-machine
        # session, not the Base serial link -- the serial link is shown
        # separately in the "Base receiver" panel's Connection field, and
        # can already be CONNECTED while this still reads stage 1: Start
        # NTRIP is what actually advances the state machine past here.
        ("DISCONNECTED", "GNSS 세션 미시작 (NTRIP Start 대기)"),
        ("CONFIGURED", "운용 설정 적용됨 · NTRIP Start 대기"),
        ("NTRIP_CONNECTING", "NTRIP 접속 중"),
        ("NTRIP_ACTIVE", "NTRIP 보정 수신 중"),
        # These two stage names ("float"/"fixed") name what the app is
        # WAITING for, not the receiver's current carrier-solution state --
        # BASE_AS_ROVER_FLOAT starts unconditionally right after NTRIP_ACTIVE,
        # before the receiver has any RTK solution at all. Check the
        # "GNSS Fixed 단계 (수신기 raw)" field for the receiver's actual
        # live fix quality (NO FIX/3D FIX/RTK FLOAT/RTK FIXED).
        ("BASE_AS_ROVER_FLOAT", "수신기 RTK Fix 수렴 대기 중"),
        ("BASE_AS_ROVER_FIXED", "수신기 RTK Fixed 획득 · 좌표 수집 시작"),
        ("FIXED_COORDINATE_COLLECTING", "좌표 안정성 검증 중"),
        ("FIXED_COORDINATE_VALIDATED", "좌표 검증 완료"),
        ("BASE_RECONFIGURING", "Base를 고정 기준점으로 재설정 중"),
        ("LOCAL_RTCM_ACTIVE", "Base 활성화 완료 · 자체 RTCM 송출 중"),
    )
    _ACQUISITION_STAGE_INDEX: dict[str, int] = {
        name: index for index, (name, _label) in enumerate(ACQUISITION_STAGES, start=1)
    }
    _ACQUISITION_STAGE_LABEL: dict[str, str] = dict(ACQUISITION_STAGES)
    _ACQUISITION_STAGE_TOTAL = len(ACQUISITION_STAGES)

    OTHER_STATE_LABELS: dict[str, str] = {
        "INITIALIZING": "초기화 중",
        "BASE_SURVEYING": "자체 Survey-in 진행 중",
        "ROVERS_CONVERGING": "로버 수렴 대기 중",
        "ALIGNMENT_COLLECTING": "정렬 데이터 수집 중",
        "ALIGNMENT_VALID": "정렬 완료",
        "READY": "측정 준비 완료",
        "MEASURING": "측정 중",
        "DEGRADED": "성능 저하",
        "FAULT": "오류 발생",
        "INVALIDATED": "무효화됨",
        "STOPPED": "중지됨",
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

        # Fixed 3-column x 2-row action grid. Survey-in and NTRIP start/stop
        # share a cell with their counterpart because refresh() never makes
        # both visible at once (local_mode vs network_mode are exclusive),
        # so the visible grid always stays a clean, fully-packed 3x2 layout
        # regardless of which operation mode is active.
        actions = QtWidgets.QGridLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label, row, col in (
            ("gnss.base_discover", "Find Base", 0, 0),
            ("gnss.base_connect", "Connect", 0, 1),
            ("gnss.base_disconnect", "Disconnect", 0, 2),
            ("gnss.survey_start", "Start Survey-in", 1, 0),
            ("gnss.ntrip_start", "Start NTRIP", 1, 0),
            ("gnss.survey_stop", "Stop Survey-in", 1, 1),
            ("gnss.ntrip_stop", "Stop NTRIP", 1, 1),
            ("gnss.ntrip_forget_password", "Forget NTRIP password", 1, 2),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            actions.addWidget(button, row, col)
        layout.addLayout(actions)

        summary_group = QtWidgets.QGroupBox("Current state")
        summary_layout = QtWidgets.QGridLayout(summary_group)
        self.summary_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("progress", "진행 단계"),
                ("mode", "Mode"),
                ("state", "Operation (raw)"),
                ("source", "Correction source"),
                ("rtcm", "RTCM stream"),
            )
        ):
            value = EngineeringValue(label)
            self.summary_values[key] = value
            summary_layout.addWidget(value, index // 2, index % 2)
        layout.addWidget(summary_group)

        self.serial_group = QtWidgets.QGroupBox("Base receiver")
        serial_layout = QtWidgets.QGridLayout(self.serial_group)
        self.serial_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (("connection", "Connection"), ("device", "Device"), ("baud", "Baud"), ("satellites", "Satellites"))
        ):
            value = EngineeringValue(label)
            self.serial_values[key] = value
            serial_layout.addWidget(value, index // 4, index % 4)
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

        self.survey_group = QtWidgets.QGroupBox("Survey-in (수신기 자체 관측치)")
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
                ("gga_transmission", "GGA 전송 상태"),
                ("status", "Caster response"),
                ("transport", "Transport"),
                ("rtcm", "RTCM"),
                ("coverage", "RTCM 완전성 (관측치 포함?)"),
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
                ("gnss_fix_stage", "GNSS Fixed 단계 (수신기 raw)"),
                ("fix", "Base fix (임시 · 좌표 확보 단계)"),
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
                ("rtcm_complete", "6. RTCM 관측치(MSM) 포함"),
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
        self.route_table.setMaximumHeight(136)
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
        stage_index = self._ACQUISITION_STAGE_INDEX.get(state)
        if stage_index is not None:
            stage_label = self._ACQUISITION_STAGE_LABEL[state]
            progress_text = f"{stage_index}/{self._ACQUISITION_STAGE_TOTAL} · {stage_label}"
            progress_valid = True if state == "LOCAL_RTCM_ACTIVE" else None
        else:
            progress_text = self.OTHER_STATE_LABELS.get(state, state)
            progress_valid = False if state in {"FAULT", "INVALIDATED"} else None
        self.summary_values["progress"].set_value(progress_text, valid=progress_valid)
        self.summary_values["mode"].set_value(self.MODE_LABELS.get(mode, mode))
        self.summary_values["state"].set_value(state, valid=False if state in {"FAULT", "INVALIDATED"} else None)
        self.summary_values["source"].set_value(source_text)
        self.summary_values["rtcm"].set_value(frame_text)

        serial_connected = bool(serial.get("connected", False))
        self.serial_values["connection"].set_value(
            "CONNECTED" if serial_connected else "DISCONNECTED",
            valid=True if serial_connected else None,
        )
        self.serial_values["device"].set_value(serial.get("device") or "--")
        self.serial_values["baud"].set_value(serial.get("baud") or "--")
        satellites = self._dict(data.get("satellites"))
        sat_used = satellites.get("used")
        sat_tracked = satellites.get("tracked")
        sat_count = satellites.get("count")
        if sat_used is not None or sat_tracked is not None:
            sat_text = f"used={_display_number(sat_used, digits=0)} / tracked={_display_number(sat_tracked, digits=0)}"
        elif sat_count is not None:
            sat_text = f"{sat_count} ({satellites.get('count_source') or '--'})"
        else:
            sat_text = "--"
        self.serial_values["satellites"].set_value(
            sat_text,
            valid=True if (sat_used or sat_count or 0) >= 12 else False if serial_connected else None,
        )

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
        # A VRS caster (e.g. NGII RTS2/VRS-RTCM32) withholds RTCM without
        # periodic GGA while the transport itself still reports "connected",
        # so this is the only place that failure mode becomes visible.
        last_gga_age = ntrip.get("last_gga_age_s")
        gga_skip_reason = str(ntrip.get("gga_skip_reason") or "")
        gga_skip_age = ntrip.get("gga_skip_age_s")
        gga_skip_reason_text = gga_skip_reason.removeprefix("[GNSS][NTRIP] GGA skipped: ")
        currently_blocked = bool(gga_skip_reason) and (
            last_gga_age is None or (gga_skip_age is not None and gga_skip_age <= last_gga_age)
        )
        if not ntrip.get("connected") and not ntrip.get("running"):
            gga_transmission_text, gga_transmission_valid = "--", None
        elif currently_blocked:
            gga_transmission_text = (
                f"차단 · {gga_skip_reason_text} "
                f"({_display_number(gga_skip_age, digits=1, suffix='s')} 째)"
            )
            gga_transmission_valid = False
        elif last_gga_age is not None:
            gga_transmission_text = f"전송 중 · last={_display_number(last_gga_age, digits=1, suffix='s')}"
            gga_transmission_valid = True
        else:
            gga_transmission_text, gga_transmission_valid = "전송 대기", None
        self.ntrip_values["gga_transmission"].set_value(gga_transmission_text, valid=gga_transmission_valid)
        self.ntrip_values["status"].set_value(ntrip.get("status") or "--")
        transport_age = ntrip.get("last_chunk_age_s")
        self.ntrip_values["transport"].set_value(
            f"chunks={ntrip.get('chunks_received', 0)} · bytes={ntrip.get('bytes_received', 0)} · "
            f"rate={_display_number(ntrip.get('byte_rate_bps'), digits=1)} B/s · "
            f"last={_display_number(transport_age, digits=2, suffix='s')}",
            source="NtripStreamStats",
        )
        rtcm_age = ntrip.get("rtcm_age_s")
        message_types = ntrip.get("rtcm_message_types")
        type_text = "--"
        if isinstance(message_types, dict) and message_types:
            # No truncation: a stream can look fine at a glance (fresh, high
            # frame rate) while still lacking the MSM observables an RTK
            # solution actually needs -- that gap only shows up by seeing
            # every message type present, not just the lowest-numbered ones.
            type_text = ", ".join(
                f"{key}:{value}"
                for key, value in sorted(message_types.items(), key=lambda item: int(item[0]) if str(item[0]).lstrip("-").isdigit() else 0)
            )
        self.ntrip_values["rtcm"].set_value(
            f"frames={ntrip.get('rtcm_frames', data.get('base_frames', 0))} · "
            f"age={_display_number(rtcm_age, digits=2, suffix='s')} · types={type_text}",
            source="Base RTCM parser",
        )
        # A fresh, high-rate RTCM stream can still be unusable for RTK: an
        # RTK solution needs a station-coordinate message (1005/1006) AND at
        # least one MSM observation message (1070-1129). Ephemeris/antenna
        # messages (1019/1020/1033/1042/1007/...) alone will never converge
        # -- this is the gap that silently stalls acquisition at "3D FIX".
        base_position_present = bool(ntrip.get("rtcm_base_position_present"))
        msm_present = bool(ntrip.get("rtcm_msm_present"))
        if not ntrip.get("connected") or not message_types:
            coverage_text, coverage_valid = "--", None
        elif base_position_present and msm_present:
            coverage_text, coverage_valid = "OK (station + MSM 관측치 포함)", True
        elif base_position_present and not msm_present:
            coverage_text, coverage_valid = "부족 — MSM 관측치 없음, RTK 불가", False
        elif msm_present and not base_position_present:
            coverage_text, coverage_valid = "부족 — station 좌표(1005/1006) 없음, RTK 불가", False
        else:
            coverage_text, coverage_valid = "부족 — station·MSM 모두 없음, RTK 불가", False
        self.ntrip_values["coverage"].set_value(coverage_text, valid=coverage_valid)
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
        fix_stage_label = str(coordinate.get("fix_stage_label") or "--")
        self.fixed_values["gnss_fix_stage"].set_value(
            fix_stage_label,
            valid=True if fix_stage_label == "RTK FIXED" else None,
        )
        base_fix = coordinate.get("fixed")
        self.fixed_values["fix"].set_value("--" if base_fix is None else "FIXED" if base_fix else "NOT FIXED", valid=base_fix)
        self.fixed_values["samples"].set_value(fixed.get("sample_count", 0))
        # profile is only populated once the coordinate is fully validated
        # AND applied to the receiver (LOCAL_RTCM_ACTIVE); live_profile
        # carries the same shape while still collecting/validating, so use
        # it as the in-progress preview instead of leaving this blank.
        live_profile = self._dict(fixed.get("live_profile"))
        scatter_source = profile if profile else live_profile
        self.fixed_values["scatter"].set_value(
            f"{_display_number(scatter_source.get('horizontal_scatter_m'))} / {_display_number(scatter_source.get('vertical_scatter_m'))}",
            unit="m",
            source="live (collecting)" if not profile and live_profile else "",
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
        for side, label in (("left", "P1"), ("right", "P2"), ("drone", "T")):
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
            "rtcm_complete": (
                None if not ntrip.get("connected") or not message_types else (base_position_present and msm_present),
                "station 1005/1006 + MSM 1070-1129 present",
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
        # Survey-in (NAV-SVIN) is a receiver-native TMODE3 feature that is
        # only ever commanded in LOCAL_SURVEY_IN mode (start_gnss_base_survey_in
        # sends CFG-TMODE3 survey-in start). NETWORK_ASSISTED_FIXED_BASE never
        # enables it -- that mode tracks convergence via its own NAV-PVT
        # sample count/scatter instead (see fixed_values["scatter"]/["samples"]
        # below) -- so showing this panel there would just display a
        # permanently-idle/frozen NAV-SVIN poll and look broken.
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
        route_targets = (("left", "P1"), ("right", "P2"), ("drone", "T"))
        if not 0 <= index.row() < len(route_targets):
            return
        side, label = route_targets[index.row()]
        self.inspection_requested.emit(
            _live_inspection(
                self._route_inspection(side, label),
                lambda side=side, label=label: self._route_inspection(side, label),
            )
        )

    def _route_inspection(self, side: str, label: str | None = None) -> dict[str, object]:
        target_label = label or {"left": "P1", "right": "P2", "drone": "T"}.get(side, side)
        data = _evidence(self.backend).gnss_operation()
        routes = data.get("routes") if isinstance(data.get("routes"), dict) else {}
        route = self._dict(routes.get(side))
        failures = int(route.get("fail") or 0)
        return {
            "selection": f"RTCM route · {target_label}",
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
        self._auto_suggestions: dict[str, dict[str, object]] = {}
        self._active_auto_suggestion_id = ""
        self._auto_suggestion_excluded_indices: dict[str, set[int]] = {}
        self._visible_auto_suggestion_indices: dict[tuple[str, str], tuple[int, ...]] = {}
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
        self._optical_pose_layout_width_hint = 0

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        readiness_group = QtWidgets.QGroupBox("Readiness")
        self.readiness_group = readiness_group
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
        self.preview_group = preview_group
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
                "Drone RTK Yaw solver uses the current pair; once both sides are marked, "
                "the pair and capture-time Drone/P1/P2 GNSS are also accumulated for "
                "JSON-free multi-view refinement."
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
            "Capture Live Pair",
        )
        self.capture_frozen_burst_button.setToolTip(
            "Capture the newest decoded native left/right frames from the live RTSP streams, "
            "then use the same immutable pair across all R,t methods. Raw RTP packets are not captured."
        )
        annotation_actions.addWidget(self.capture_frozen_burst_button)
        self.capture_frozen_burst_wiris_ftp_button = make_solver_button(
            "alignment.pose.capture_burst_wiris_ftp",
            "Capture HR Still (FTP)",
        )
        self.capture_frozen_burst_wiris_ftp_button.setToolTip(
            "Trigger a WIRIS HR still on both cameras and download it via FTP instead of the "
            "live RTSP feed, then store it under the same immutable-pair contract."
        )
        annotation_actions.addWidget(self.capture_frozen_burst_wiris_ftp_button)
        self.capture_frozen_burst_wiris_ftp_oz_button = make_solver_button(
            "alignment.pose.capture_burst_wiris_ftp_oz",
            "Capture OZ Still (FTP)",
        )
        self.capture_frozen_burst_wiris_ftp_oz_button.setToolTip(
            "Same as Capture HR Still (FTP), using the lower-resolution "
            "operating-zoom still (SIVI) instead of the HR still."
        )
        annotation_actions.addWidget(self.capture_frozen_burst_wiris_ftp_oz_button)
        self.import_frozen_burst_button = QtWidgets.QPushButton("Import Pair")
        self.import_frozen_burst_button.setObjectName("opticalPoseImportBurst")
        self.import_frozen_burst_button.setToolTip(
            "Load left/right images from disk instead of a live capture -- reuse an "
            "FTP-downloaded HR/OZ still, or feed local test imagery for offline accuracy evaluation. "
            "The same immutable-frame contract as Capture Live Pair applies."
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
        preview_layout = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.TopToBottom)
        preview_layout.setContentsMargins(0, 0, 0, 0)
        preview_layout.setSpacing(6)
        self.frozen_preview_layout = preview_layout
        self.frozen_preview_labels: dict[str, QtWidgets.QLabel] = {}
        self.annotation_views: dict[str, AnnotationImageView] = {}
        self.frozen_preview_panels: list[QtWidgets.QWidget] = []
        for side, label in (("left", "Left"), ("right", "Right")):
            side_panel = _FrozenPreviewColumn()
            self.frozen_preview_panels.append(side_panel)
            side_layout = QtWidgets.QVBoxLayout(side_panel)
            side_layout.setContentsMargins(0, 0, 0, 0)
            side_layout.setSpacing(3)
            view = AnnotationImageView(side_panel, wheel_zoom_requires_control=False)
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
            view.suggestion_point_right_clicked.connect(
                lambda mode, index, side=side: self._on_suggestion_point_right_clicked(
                    side, mode, index
                )
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
        self.observations_group = observations_group
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
        self._drone_rtk_multiview_samples: list[dict[str, object]] = []
        self._drone_rtk_multiview_sample_by_observation: dict[str, dict[str, object]] = {}
        self._drone_rtk_multiview_sample_by_burst: dict[str, dict[str, object]] = {}
        self._drone_rtk_multiview_collected_frame_ids: dict[str, set[str]] = {
            "left": set(),
            "right": set(),
        }

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
            "Copy the currently displayed suggestion into the manual annotation buffer. "
            "Horizon/Mountain detector quality is advisory, but current-frame native points must "
            "remain structurally usable. Horizon requires two separated endpoints per side; "
            "Mountain requires paired distributed points after exclusions."
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

        multiview_button = make_solver_button(
            "alignment.pose.solve_drone_rtk_multiview",
            "Solve Drone RTK Multi-view R,t",
        )
        multiview_button.setToolTip(
            "Use at least four READY Frozen target pairs accumulated since the last Alignment Reset, together "
            "with their capture-time Drone RTK positions, to solve full 6-DOF R,t for both cameras."
        )
        workflow_note = QtWidgets.QLabel(
            "Each solver below creates an independent candidate and shares the same review/apply path."
        )
        workflow_note.setObjectName("compactContract")
        workflow_note.setWordWrap(True)
        readiness_layout.addWidget(workflow_note, 4, 0, 1, 2)

        self.method_tabs = _CurrentPageTabWidget()
        self.method_tabs.setObjectName("alignmentOperationTabs")
        self.method_information_labels: dict[str, QtWidgets.QLabel] = {}
        self.method_detail_labels: dict[str, QtWidgets.QLabel] = {}
        method_contracts = {
            "Horizon": "Observed: Roll/Pitch · Measurement fill: Yaw=pair-frozen Pixhawk, t=pair-frozen GNSS",
            "Mountain + GNSS": "Observed: relative R from paired u,v · t: frozen GNSS baseline · no terrain height",
            "Terrain PnP": "Observed: full 6-DOF · requires GeoTIFF vertical datum and matched 2D/3D points",
            "Drone Rotation": "1 target: Yaw · 2+ targets: Roll/Pitch/Yaw · Measurement t: pair-frozen GNSS",
            "Drone Multi-view": "Observed: full 6-DOF R,t · requires at least four READY accumulated Frozen target pairs",
            "References": "Register existing Checkerboard R,t or live Pixhawk+RTK Translation as reusable reference evidence",
            "Combine": "Compose only non-overlapping axes observable on both camera sides with shared prior/K,D",
        }
        method_details = {
            "Horizon": "Evidence: horizon point count, model fit, Roll/Pitch sensitivity, prior Yaw/Translation source",
            "Mountain + GNSS": "Evidence: paired point count, inliers/outliers, Sampson residual, GNSS baseline, lever-arm status",
            "Terrain PnP": "Evidence: GeoTIFF/vertical datum, 2D↔3D counts, reprojection error, depth ratio, rank/condition",
            "Drone Rotation": "Evidence: target count, Yaw/full-R mode, Drone/P1/P2 RTK state, sync delta, angular residual",
            "Drone Multi-view": "Evidence: JSON hash, per-side observation count, timestamp verification, 3D spread, full-rank status",
            "References": "Evidence: active K,D compatibility, profile hash, source zoom, baseline and reference state",
            "Combine": "Evidence: selected candidates, per-side observability, DOF ownership, overlap and shared-prior checks",
        }

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
            contract_label = QtWidgets.QLabel(method_contracts.get(tab_label, ""))
            contract_label.setObjectName("compactContract")
            contract_label.setWordWrap(True)
            page_layout.addWidget(contract_label)
            information_label = QtWidgets.QLabel("Solve readiness: waiting for owner snapshot")
            information_label.setObjectName("statusStrip")
            information_label.setWordWrap(True)
            self.method_information_labels[tab_label] = information_label
            page_layout.addWidget(information_label)
            detail_label = QtWidgets.QLabel(method_details.get(tab_label, ""))
            detail_label.setObjectName("compactContract")
            detail_label.setWordWrap(True)
            self.method_detail_labels[tab_label] = detail_label
            page_layout.addWidget(detail_label)
            if content is not None:
                content(page_layout)
            action_layout = QtWidgets.QHBoxLayout()
            for button in buttons:
                action_layout.addWidget(button)
            action_layout.addStretch(1)
            page_layout.addLayout(action_layout)
            page_layout.addStretch(1)
            index = self.method_tabs.addTab(page, tab_label)
            self.method_tabs.setTabToolTip(index, description)

        add_method_tab(
            "Horizon",
            "Two horizon points constrain Roll/Pitch only. Yaw and camera position remain at the prior pose.",
            (
                make_solver_button("alignment.pose.solve_horizon", "Solve Horizon Roll/Pitch"),
            ),
        )
        def build_mountain_content(page_layout: QtWidgets.QVBoxLayout) -> None:
            self.mountain_hide_auto_lines_checkbox = QtWidgets.QCheckBox(
                "Hide Auto-select connecting lines"
            )
            self.mountain_hide_auto_lines_checkbox.setObjectName(
                "mountainHideAutoSuggestionLines"
            )
            self.mountain_hide_auto_lines_checkbox.setChecked(True)
            self.mountain_hide_auto_lines_checkbox.setToolTip(
                "Hide only the line joining Mountain Auto-select points. Right-click an "
                "automatic point to exclude its corresponding left/right pair from the overlay "
                "and from the accepted solver input."
            )
            self.mountain_hide_auto_lines_checkbox.toggled.connect(
                self._set_mountain_auto_lines_hidden
            )
            page_layout.addWidget(self.mountain_hide_auto_lines_checkbox)

        add_method_tab(
            "Mountain + GNSS",
            "Click corresponding Mountain features in the same order on both frames. Uses paired u,v and the numeric GNSS baseline; no GeoTIFF or terrain height.",
            (
                make_solver_button("alignment.pose.solve_gnss_mountain", "Solve GNSS + Mountain R,t"),
            ),
            content=build_mountain_content,
        )
        self._set_mountain_auto_lines_hidden(True)
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
            "Drone Rotation",
            "One synchronized target constrains Yaw; two or more non-degenerate target/ray observations "
            "constrain Roll/Pitch/Yaw. Missing Translation is supplied by live GNSS in Measurement.",
            (
                make_solver_button("alignment.pose.solve_drone_yaw", "Solve Drone RTK Rotation"),
            ),
        )
        add_method_tab(
            "Drone Multi-view",
            "Accumulate at least four synchronized Frozen target pairs in Drone Rotation, then solve "
            "full six-DOF camera pose without a separate JSON file. Translation is solved from the "
            "observations instead of inherited from live GNSS.",
            (multiview_button,),
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
            "References",
            "Register Checkerboard or live Pixhawk+RTK reference evidence for later method composition.",
            (register_checkerboard_button, register_pixhawk_button),
        )
        add_method_tab(
            "Combine",
            "Compose explicitly selected non-overlapping candidate axes into one immutable candidate.",
            (combine_button,),
            content=build_combine_content,
        )
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

        contract = QtWidgets.QLabel(
            "Frames: image pixels/OpenCV optical RDF · pose PX4 virtual FRD→world NED · "
            "GNSS WGS84 · candidate application is immutable and owner-controlled"
        )
        contract.setObjectName("compactContract")
        contract.setWordWrap(True)
        self.contract_label = contract

        method_column = QtWidgets.QWidget()
        method_column.setObjectName("alignmentMethodColumn")
        method_column_layout = QtWidgets.QVBoxLayout(method_column)
        method_column_layout.setContentsMargins(0, 0, 0, 0)
        method_column_layout.setSpacing(6)
        method_column_layout.addWidget(self.method_tabs)
        method_column_layout.addWidget(self.auto_group)
        method_column_layout.addStretch(1)
        self.method_column = method_column

        candidate_review = QtWidgets.QWidget()
        candidate_review.setObjectName("alignmentCandidateReview")
        candidate_review_layout = QtWidgets.QVBoxLayout(candidate_review)
        candidate_review_layout.setContentsMargins(0, 0, 0, 0)
        candidate_review_layout.setSpacing(6)
        candidate_review_layout.addWidget(self.candidate_group)
        candidate_review_layout.addWidget(self.observations_group)
        candidate_review_layout.addWidget(self.contract_label)
        candidate_review_layout.addStretch(1)
        self.candidate_review_widget = candidate_review

        lower_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        lower_splitter.setObjectName("alignmentMethodReviewSplitter")
        lower_splitter.setChildrenCollapsible(False)
        lower_splitter.setHandleWidth(8)
        lower_splitter.addWidget(method_column)
        lower_splitter.addWidget(candidate_review)
        lower_splitter.setStretchFactor(0, 3)
        lower_splitter.setStretchFactor(1, 2)
        lower_splitter.setSizes([720, 480])
        self.method_review_splitter = lower_splitter
        layout.addWidget(lower_splitter)

        self.method_tabs.currentChanged.connect(self._handle_method_tab_changed)
        self._handle_method_tab_changed(self.method_tabs.currentIndex())

    def _handle_method_tab_changed(self, index: int) -> None:
        label = self.method_tabs.tabText(index) if index >= 0 else ""
        annotation_modes = {
            "Horizon": AnnotationMode.HORIZON.value,
            "Mountain + GNSS": AnnotationMode.RIDGE.value,
            "Terrain PnP": AnnotationMode.RIDGE.value,
            "Drone Rotation": AnnotationMode.TARGET.value,
        }
        mode = annotation_modes.get(label)
        editable = mode is not None
        self.annotation_mode_combo.setEnabled(editable)
        self.annotation_clear_button.setEnabled(editable)
        for view in self.annotation_views.values():
            view.setEnabled(editable)
        if mode is not None:
            mode_index = self.annotation_mode_combo.findData(mode)
            if mode_index >= 0:
                self.annotation_mode_combo.setCurrentIndex(mode_index)
            self._set_annotation_mode(self.annotation_mode_combo.currentIndex())

        show_horizon_auto = label == "Horizon"
        show_mountain_auto = label == "Mountain + GNSS"
        self.auto_group.setVisible(show_horizon_auto or show_mountain_auto)
        horizon_button = self.auto_select_buttons.get("alignment.pose.auto_select_horizon")
        mountain_button = self.auto_select_buttons.get("alignment.pose.auto_select_mountain")
        if horizon_button is not None:
            horizon_button.setVisible(show_horizon_auto)
        if mountain_button is not None:
            mountain_button.setVisible(show_mountain_auto)
        self.observations_group.setVisible(
            label in {"Drone Rotation", "Drone Multi-view"}
        )
        self._refresh_method_information()

    def _update_optical_pose_layout_mode(self) -> None:
        width_hint = self._optical_pose_layout_width_hint or self.width()
        preview_width = min(
            self.preview_group.contentsRect().width(),
            max(0, width_hint - 12),
        )
        margins = self.frozen_preview_layout.contentsMargins()
        usable_width = max(0, preview_width - margins.left() - margins.right())
        horizontal_previews = (usable_width - self.frozen_preview_layout.spacing()) // 2 >= 560
        preview_direction = (
            QtWidgets.QBoxLayout.LeftToRight
            if horizontal_previews
            else QtWidgets.QBoxLayout.TopToBottom
        )
        if self.frozen_preview_layout.direction() != preview_direction:
            self.frozen_preview_layout.setDirection(preview_direction)
            self.preview_group.updateGeometry()

        splitter_orientation = (
            QtCore.Qt.Horizontal if width_hint >= 1200 else QtCore.Qt.Vertical
        )
        if self.method_review_splitter.orientation() != splitter_orientation:
            self.method_review_splitter.setOrientation(splitter_orientation)
            self.method_review_splitter.setSizes(
                [720, 480] if splitter_orientation == QtCore.Qt.Horizontal else [420, 360]
            )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt override.
        self._optical_pose_layout_width_hint = event.size().width()
        super().resizeEvent(event)
        self._update_optical_pose_layout_mode()

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802 - Qt override.
        super().showEvent(event)
        QtCore.QTimer.singleShot(0, self._update_optical_pose_layout_mode)

    def _refresh_method_information(self) -> None:
        annotation_counts: dict[str, dict[str, int]] = {
            "left": {"horizon": 0, "ridge": 0, "target": 0},
            "right": {"horizon": 0, "ridge": 0, "target": 0},
        }
        for side, view in self.annotation_views.items():
            payload = view.export_annotations()
            if payload is None:
                continue
            annotation_counts[side] = {
                "horizon": len(payload.horizon_line),
                "ridge": len(payload.ridge_polyline),
                "target": len(payload.target_point),
            }

        candidate_state_by_method: dict[str, str] = {}
        candidate_by_method: dict[str, Mapping[str, object]] = {}
        method_categories = {
            "flat-horizon-roll-pitch": "Horizon",
            "spherical-horizon-roll-pitch": "Horizon",
            "gnss-constrained-mountain-uv": "Mountain + GNSS",
            "terrain-pnp-ensemble": "Terrain PnP",
            "drone-rtk-target-yaw": "Drone Rotation",
            "drone-rtk-multiview-r-t": "Drone Multi-view",
            "combined-independent-axis-evidence": "Combine",
        }
        for candidate in self._candidate_rows.values():
            method = str(candidate.get("method", "") or "")
            category = method_categories.get(method)
            if category is not None:
                candidate_by_method[category] = candidate
                candidate_state_by_method[category] = str(
                    candidate.get("state", candidate.get("quality", "CANDIDATE"))
                    or "CANDIDATE"
                )

        left = annotation_counts["left"]
        right = annotation_counts["right"]
        terrain_state = self.readiness_values["terrain"].value.text()
        base_state = self.readiness_values["base_extrinsics"].value.text()
        observation_count = len(self._drone_rtk_observation_order)
        multiview_sample_count = len(self._drone_rtk_multiview_samples)
        multiview_ready_count = sum(
            str(sample.get("state") or "").upper() == "READY"
            for sample in self._drone_rtk_multiview_samples
        )
        candidate_count = len(self._candidate_rows)
        information = {
            "Horizon": (
                f"Solve readiness · horizon points L/R={left['horizon']}/{right['horizon']} "
                f"(2 required each) · candidate={candidate_state_by_method.get('Horizon', '--')} · "
                "Measurement also requires fresh Pixhawk Yaw and usable GNSS t"
            ),
            "Mountain + GNSS": (
                f"Solve readiness · paired ridge points L/R={left['ridge']}/{right['ridge']} "
                f"(≥4, equal order) · candidate={candidate_state_by_method.get('Mountain + GNSS', '--')} · "
                "frozen GNSS baseline and lever-arm provenance required"
            ),
            "Terrain PnP": (
                f"Solve readiness · ridge points L/R={left['ridge']}/{right['ridge']} · "
                f"terrain={terrain_state or '--'} · candidate={candidate_state_by_method.get('Terrain PnP', '--')}"
            ),
            "Drone Rotation": (
                f"Solve readiness · target points L/R={left['target']}/{right['target']} · "
                f"captured observations={observation_count} · candidate={candidate_state_by_method.get('Drone Rotation', '--')}"
            ),
            "Drone Multi-view": (
                f"Solve readiness · accumulated READY/total={multiview_ready_count}/{multiview_sample_count} "
                f"(≥4 READY required) · captured evidence={observation_count} · "
                f"candidate={candidate_state_by_method.get('Drone Multi-view', '--')}"
            ),
            "References": (
                f"Reference readiness · base R,t={base_state or '--'} · candidates={candidate_count} · "
                "register a reusable source candidate"
            ),
            "Combine": (
                f"Combine readiness · candidates={candidate_count} · "
                f"combined={candidate_state_by_method.get('Combine', '--')}"
            ),
        }
        def candidate_metric(category: str, key: str, default: object = "--") -> object:
            row = candidate_by_method.get(category, {})
            metrics = row.get("quality_metrics")
            metrics = metrics if isinstance(metrics, Mapping) else {}
            return metrics.get(key, default)

        def candidate_input(category: str, key: str, default: object = "--") -> object:
            row = candidate_by_method.get(category, {})
            hashes = row.get("input_hashes")
            hashes = hashes if isinstance(hashes, Mapping) else {}
            return hashes.get(key, default)

        details = {
            "Horizon": (
                f"model={candidate_input('Horizon', 'horizon_model')} · "
                f"L/R RMS px={candidate_metric('Horizon', 'left_rms_px_equivalent')}/"
                f"{candidate_metric('Horizon', 'right_rms_px_equivalent')} · "
                f"sensitivity R/P={candidate_metric('Horizon', 'left_roll_sensitivity_1px_deg')}/"
                f"{candidate_metric('Horizon', 'left_pitch_sensitivity_1px_deg')} deg"
            ),
            "Mountain + GNSS": (
                f"pairs={candidate_metric('Mountain + GNSS', 'mountain_uv_pair_count')} · "
                f"inlier ratio={candidate_metric('Mountain + GNSS', 'inlier_ratio')} · "
                f"Sampson RMS px={candidate_metric('Mountain + GNSS', 'sampson_rms_px_equivalent')} · "
                f"baseline={candidate_metric('Mountain + GNSS', 'gnss_baseline_m')} m · "
                f"lever-arm={candidate_input('Mountain + GNSS', 'lever_arm_status')}"
            ),
            "Terrain PnP": (
                f"L/R reprojection RMSE={candidate_metric('Terrain PnP', 'left_reprojection_rmse_px')}/"
                f"{candidate_metric('Terrain PnP', 'right_reprojection_rmse_px')} px · "
                f"positive depth={candidate_metric('Terrain PnP', 'left_positive_depth_ratio')}/"
                f"{candidate_metric('Terrain PnP', 'right_positive_depth_ratio')}"
            ),
            "Drone Rotation": (
                f"observations L/R={candidate_metric('Drone Rotation', 'left_observation_count')}/"
                f"{candidate_metric('Drone Rotation', 'right_observation_count')} · "
                f"angular RMS={candidate_metric('Drone Rotation', 'left_rotation_rms_chord_residual')} · "
                "Translation=pair-frozen GNSS"
            ),
            "Drone Multi-view": (
                f"observations L/R={candidate_metric('Drone Multi-view', 'left_observation_count')}/"
                f"{candidate_metric('Drone Multi-view', 'right_observation_count')} · "
                f"angular RMS={candidate_metric('Drone Multi-view', 'left_angular_rms_chord')} / "
                f"{candidate_metric('Drone Multi-view', 'right_angular_rms_chord')} · "
                f"collection={candidate_input('Drone Multi-view', 'drone_rtk_multiview_collection_sha256')} · "
                f"JSON fallback={candidate_input('Drone Multi-view', 'drone_rtk_multiview_json_sha256')}"
            ),
            "References": (
                f"checkerboard profile={base_state or '--'} · "
                f"reference candidates={candidate_count} · source zoom/status are shown in Candidate review"
            ),
            "Combine": (
                f"sources={len(candidate_by_method.get('Combine', {}).get('source_observation_ids', ())) or '--'} · "
                "axes must be non-overlapping and observable on both sides"
            ),
        }
        for label, widget in self.method_information_labels.items():
            widget.setText(information.get(label, "Solve readiness: --"))
        for label, widget in self.method_detail_labels.items():
            widget.setText(details.get(label, "Evidence: --"))

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
        sources: dict[str, dict[str, object]] = {
            "left": {"path": left_path},
            "right": {"path": right_path},
        }
        self.command_requested.emit(
            "alignment.pose.import_burst",
            {"sources": sources},
        )

    def command_payload(self, command_id: str) -> dict[str, object]:
        payload: dict[str, object] = {
            "command_id": str(command_id),
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
            payload["excluded_point_indices"] = sorted(
                self._auto_suggestion_excluded_indices.get(
                    self._active_auto_suggestion_id, set()
                )
            )
        if command_id == "alignment.pose.solve_drone_rtk_multiview":
            payload["multiview_source"] = "accumulated_frozen_targets"
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
        return {"candidate_id": candidate_id}

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
        # Once a target pair has been bound to capture-time RTK evidence, a
        # new Frozen burst starts with a clean target editor automatically.
        # An incomplete/unrecorded mark is retained and remains pinned to its
        # old frame identity so capture cannot silently discard operator work.
        annotation_payload = view.export_annotations()
        previous_annotation_frame_id = (
            str(annotation_payload.frame_id)
            if annotation_payload is not None
            else ""
        )
        previous_target_collected = (
            previous_annotation_frame_id
            in self._drone_rtk_multiview_collected_frame_ids.get(side, set())
        )
        preserve_annotations = bool(
            view.annotation_points(AnnotationMode.TARGET)
            and not previous_target_collected
        )
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

    def _set_mountain_auto_lines_hidden(self, hidden: bool) -> None:
        for view in self.annotation_views.values():
            view.set_suggestion_polyline_visible(
                not bool(hidden), mode=AnnotationMode.RIDGE
            )

    def _on_suggestion_point_right_clicked(
        self,
        side: str,
        mode: str,
        visible_index: int,
    ) -> None:
        suggestion = self._auto_suggestions.get(self._active_auto_suggestion_id)
        if not isinstance(suggestion, Mapping) or str(suggestion.get("mode") or "") != "mountain":
            return
        if str(mode) != AnnotationMode.RIDGE.value:
            return
        visible_indices = self._visible_auto_suggestion_indices.get((side, str(mode)), ())
        if not 0 <= int(visible_index) < len(visible_indices):
            return
        original_index = visible_indices[int(visible_index)]
        excluded = self._auto_suggestion_excluded_indices.setdefault(
            self._active_auto_suggestion_id, set()
        )
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
            self._auto_suggestion_excluded_indices.get(suggestion_id, set())
            if mode == AnnotationMode.RIDGE
            else set()
        )
        visible_indices = tuple(
            index for index in range(len(points)) if index not in excluded
        )
        self._visible_auto_suggestion_indices[(side, mode.value)] = visible_indices
        filtered_points = tuple(points[index] for index in visible_indices)
        filtered_confidences = (
            tuple(confidences[index] for index in visible_indices)
            if len(confidences) == len(points)
            else ()
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
            if (
                str(row.get("frame_id") or "") != str(view.frame_id or "")
                or str(row.get("image_sha256") or "") != str(view.image_sha256 or "")
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
                excluded_indices=sorted(
                    self._auto_suggestion_excluded_indices.get(suggestion_id, set())
                ),
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
            if (
                str(row.get("frame_id") or "") != str(view.frame_id or "")
                or str(row.get("image_sha256") or "") != str(view.image_sha256 or "")
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
        mode_label = "Horizon" if str(mode).lower() == "horizon" else "Mountain"
        self._auto_suggestion_error_active = False
        self.auto_suggestion_status_label.setStyleSheet("")
        self.auto_suggestion_status_label.setText(
            f"{mode_label} · RUNNING · analyzing frozen frames in background"
        )
        self.auto_suggestion_metrics_label.setText(
            "Confidence / inliers / residual / detector: calculating…"
        )

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
                points, confidences = self._filtered_auto_points(
                    suggestion_id, side, annotation_mode, row
                )
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
        excluded_count = len(
            self._auto_suggestion_excluded_indices.get(suggestion_id, set())
        )
        excluded_text = f" · excluded pairs {excluded_count}" if mode == "mountain" else ""
        self.auto_suggestion_metrics_label.setText(
            f"Confidence {confidence_text} · inliers {inliers} · residual {residual_text} "
            f"· detector {detector}{excluded_text}"
        )
        reason = str(suggestion.get("failure_reason") or suggestion.get("reason") or "")
        suffix = f" · {reason}" if reason else ""
        copy_eligible = False
        copy_status = ""
        if mode == "mountain":
            copy_eligible, copy_reason = self._mountain_suggestion_copy_status(
                suggestion_id, sides
            )
        elif mode == "horizon":
            copy_eligible, copy_reason = self._horizon_suggestion_copy_status(sides)
        else:
            copy_reason = "unsupported automatic suggestion mode"
        if mode in {"horizon", "mountain"}:
            copy_status = (
                " · COPY READY"
                if copy_eligible
                else f" · COPY BLOCKED: {copy_reason}"
            )
        self.auto_suggestion_status_label.setText(
            f"{mode.title() or 'Automatic'} · {state} · {provenance}{copy_status} "
            f"· {suggestion_id[:18]}{suffix}"
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
        quality_state = str(suggestion.get("state") or "UNKNOWN").upper()
        self.auto_suggestion_status_label.setText(
            f"{mode.title()} · {quality_state} · AUTO_ACCEPTED · points copied to manual annotation"
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
        self._refresh_method_information()

    def _set_annotation_mode(self, _index: int = -1) -> None:
        mode = str(
            self.annotation_mode_combo.currentData()
            or AnnotationMode.HORIZON.value
        )
        for view in self.annotation_views.values():
            view.set_annotation_mode(mode)
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
        self._refresh_method_information()

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
        self._refresh_method_information()

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
        self._refresh_candidate_actions()
        self._refresh_method_information()

    def _selected_drone_rtk_observation_id(self) -> str:
        row = self.observation_history_table.currentRow()
        item = self.observation_history_table.item(row, 0) if row >= 0 else None
        return "" if item is None else str(item.data(QtCore.Qt.UserRole) or "")

    def set_drone_rtk_multiview_samples(self, samples: object) -> None:
        """Project frozen-target/RTK bindings without mutating capture rows."""

        rows = (
            [dict(value) for value in samples if isinstance(value, Mapping)]
            if isinstance(samples, (tuple, list))
            else []
        )
        self._drone_rtk_multiview_samples = rows
        self._drone_rtk_multiview_sample_by_observation = {}
        self._drone_rtk_multiview_sample_by_burst = {}
        self._drone_rtk_multiview_collected_frame_ids = {
            "left": set(),
            "right": set(),
        }
        for sample in rows:
            observation_id = str(sample.get("capture_observation_id") or "")
            burst_id = str(sample.get("burst_id") or "")
            if observation_id:
                self._drone_rtk_multiview_sample_by_observation[observation_id] = sample
            if burst_id:
                self._drone_rtk_multiview_sample_by_burst[burst_id] = sample
            if str(sample.get("state") or "").upper() != "READY":
                # A BLOCKED sample (e.g. GNSS sync not yet verified) is
                # recorded for auditability, but its frame must not be
                # treated as collected -- the operator's marks need to
                # survive so the pair can be retried on the next capture.
                continue
            annotations = sample.get("annotations")
            annotations = annotations if isinstance(annotations, Mapping) else {}
            for side in ("left", "right"):
                annotation = annotations.get(side)
                if isinstance(annotation, Mapping):
                    frame_id = str(annotation.get("frame_id") or "")
                    if frame_id:
                        self._drone_rtk_multiview_collected_frame_ids[side].add(frame_id)

        # A snapshot may update only the derived sample projection. Repaint
        # existing immutable observation rows so Target status stays current.
        if self._drone_rtk_observation_order:
            self.set_drone_rtk_observations(
                [
                    self._drone_rtk_observation_rows[observation_id]
                    for observation_id in self._drone_rtk_observation_order
                    if observation_id in self._drone_rtk_observation_rows
                ]
            )
        else:
            self._refresh_method_information()

    def _drone_rtk_target_state(self, observation: Mapping[str, object]) -> str:
        observation_id = str(observation.get("observation_id") or "")
        burst_id = str(observation.get("burst_id") or "")
        sample = self._drone_rtk_multiview_sample_by_observation.get(observation_id)
        if sample is None:
            sample = self._drone_rtk_multiview_sample_by_burst.get(burst_id)
        if sample is None:
            return "NOT ANNOTATED"
        return str(sample.get("state") or "ANNOTATED")

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
                    self._drone_rtk_target_state(observation),
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
        self._refresh_method_information()

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
        self.current_observation_values["target"].set_value(
            self._drone_rtk_target_state(observation)
        )

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
        drone_rtk_multiview_samples = snapshot.get("drone_rtk_multiview_samples")
        if isinstance(drone_rtk_multiview_samples, (tuple, list)):
            self.set_drone_rtk_multiview_samples(drone_rtk_multiview_samples)
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

    @staticmethod
    def _candidate_relative_pose_text(candidate: Mapping[str, object]) -> str:
        """Render the owner-projected Mountain relative OpenCV R/t readout."""

        if str(candidate.get("method", "") or "") != "gnss-constrained-mountain-uv":
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
        lines.extend(
            "  [" + ", ".join(f"{value:+.9f}" for value in row) + "]"
            for row in rotation
        )
        lines.append(
            "t_right_from_left_rdf_m: ["
            + ", ".join(f"{value:+.9f}" for value in translation)
            + "]"
        )
        if origin is not None:
            lines.append(
                "right_origin_in_left_rdf_m: ["
                + ", ".join(f"{value:+.9f}" for value in origin)
                + "]"
            )
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
        apply_ready = (
            self._selected_candidate_applicable()
            and bool(owned_dofs)
            and not unobservable
        )
        self.apply_button.setEnabled(apply_ready)
        dof_text = ",".join(owned_dofs) or "none"
        suffix = (
            " · BLOCKED unobservable=" + ",".join(unobservable)
            if unobservable
            else ""
        )
        self.candidate_status_label.setText(
            f"{candidate_id} · {state} · Apply DOF={dof_text}{suffix}"
        )
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

    def _request_apply(self) -> None:
        payload = self.apply_payload()
        if payload is not None:
            self.command_requested.emit("alignment.pose.apply_candidate", payload)

    def _request_revert(self) -> None:
        self.command_requested.emit("alignment.pose.revert", self.revert_payload())

    def _request_reset(self) -> None:
        self.command_requested.emit("alignment.pose.reset", {})


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


class CalibrationErrorPlot(QtWidgets.QWidget):
    """Compact source-backed reprojection plot with exclusion actions."""

    exclude_requested = QtCore.pyqtSignal(object)
    restore_requested = QtCore.pyqtSignal(object)

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._diagnostics: tuple[dict[str, object], ...] = ()
        self._excluded_paths: set[str] = set()
        self._excluded_pair_keys: set[str] = set()
        self._point_positions: list[tuple[QtCore.QPointF, dict[str, object]]] = []
        self._threshold_px: float | None = None
        self.setMinimumHeight(150)

    @staticmethod
    def _finite_error(row: Mapping[str, object]) -> float | None:
        try:
            value = float(row.get("reprojection_error_px"))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value >= 0.0 else None

    def set_diagnostics(
        self,
        rows: Sequence[Mapping[str, object]],
        *,
        excluded_paths: set[str] | None = None,
        excluded_pair_keys: set[str] | None = None,
    ) -> None:
        self._diagnostics = tuple(dict(row) for row in rows)
        self._excluded_paths = set(excluded_paths or ())
        self._excluded_pair_keys = set(excluded_pair_keys or ())
        errors = sorted(
            value
            for row in self._diagnostics
            if (value := self._finite_error(row)) is not None
        )
        if errors:
            median = errors[len(errors) // 2]
            self._threshold_px = max(1.0, median * 2.0)
        else:
            self._threshold_px = None
        self.update()

    def _layout_points(self) -> None:
        self._point_positions = []
        rows = [
            row for row in self._diagnostics if self._finite_error(row) is not None
        ]
        if not rows:
            return
        left, top, right, bottom = 38.0, 18.0, 18.0, 26.0
        width = max(1.0, self.width() - left - right)
        height = max(1.0, self.height() - top - bottom)
        maximum = max(self._finite_error(row) or 0.0 for row in rows)
        maximum = max(maximum, self._threshold_px or 0.0, 1.0)
        for index, row in enumerate(rows):
            x = left + width * (index + 0.5) / len(rows)
            y = top + height * (1.0 - (self._finite_error(row) or 0.0) / maximum)
            self._point_positions.append((QtCore.QPointF(x, y), row))

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:  # noqa: N802
        self._layout_points()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor("#ffffff"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#c6c6c6"), 1))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))
        for point, row in self._point_positions:
            excluded = (
                str(row.get("path") or "") in self._excluded_paths
                or str(row.get("capture_key") or "") in self._excluded_pair_keys
            )
            error = self._finite_error(row)
            candidate = bool(
                self._threshold_px is not None
                and error is not None
                and error >= self._threshold_px
            )
            color = "#888888" if excluded else "#c0392b" if candidate else SUCCESS_TEXT
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1))
            painter.drawEllipse(point, 5.0, 5.0)

    def contextMenuEvent(self, event: QtGui.QContextMenuEvent) -> None:  # noqa: N802
        self._layout_points()
        if not self._point_positions:
            return
        target = min(
            self._point_positions,
            key=lambda item: QtCore.QLineF(item[0], event.pos()).length(),
        )
        point, row = target
        if QtCore.QLineF(point, event.pos()).length() > 20.0:
            return
        path = str(row.get("path") or "")
        capture_key = str(row.get("capture_key") or "")
        side = str(row.get("side") or "")
        image_excluded = path in self._excluded_paths
        pair_excluded = capture_key in self._excluded_pair_keys
        menu = QtWidgets.QMenu(self)
        image_action = menu.addAction(
            "Restore Intrinsic image" if image_excluded else "Exclude Intrinsic image"
        )
        pair_action = menu.addAction(
            "Restore Stereo pair" if pair_excluded else "Exclude Stereo pair"
        )
        chosen = menu.exec_(event.globalPos())
        if chosen is None:
            return
        if chosen is image_action:
            signal = self.restore_requested if image_excluded else self.exclude_requested
            signal.emit(
                {
                    "scope": "image",
                    "side": side,
                    "path": path,
                    "capture_key": capture_key,
                }
            )
        elif chosen is pair_action:
            signal = self.restore_requested if pair_excluded else self.exclude_requested
            signal.emit(
                {
                    "scope": "pair",
                    "side": side,
                    "path": path,
                    "capture_key": capture_key,
                }
            )


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
        # This page is hosted by a horizontally non-scrolling QScrollArea.
        # Wide method tabs and evidence tables have a large size hint, but
        # that hint must not keep the page wider than the central viewport
        # when the Inspector dock opens.
        self.setMinimumWidth(0)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Expanding,
        )
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
        # The Alignment page itself lives in the workspace host's outer scroll
        # area.  Common readiness/frozen evidence stays above the promoted
        # method tabs owned by OpticalPosePane.
        self.optical_pose_pane.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Expanding,
        )
        layout.addWidget(self.optical_pose_pane, 1)
    def _build_operation_tabs(
        self,
        _invoke_command: Callable[[str], bool] | None = None,
    ) -> QtWidgets.QTabWidget:
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.optical_pose_pane = OpticalPosePane()
        operation_tabs = self.optical_pose_pane.method_tabs
        self.optical_pose_pane.command_requested.connect(
            self.optical_pose_command_requested.emit
        )
        self.operation_buttons.update(self.optical_pose_pane.solver_buttons)
        self.operation_buttons["alignment.pose.apply_candidate"] = (
            self.optical_pose_pane.apply_button
        )
        self.operation_buttons["alignment.pose.revert"] = (
            self.optical_pose_pane.revert_button
        )
        self.operation_buttons["alignment.pose.reset"] = (
            self.optical_pose_pane.reset_button
        )
        self.optical_pose_scroll = None
        return operation_tabs

    def select_tab_for_context(self, _context_id: str) -> None:
        """Alignment has one promoted method surface; preserve its local tab."""

        return


class CalibrationWorkspace(QtWidgets.QWidget):
    """Separate Intrinsics and Extrinsics workflows over shared captured inputs."""

    _CALIBRATION_QUALITY_GATED_COMMANDS = frozenset(
        {
            "calibration.apply_bundle",
            "calibration.validation.apply",
        }
    )

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
        self._calibration_pair_picker_signature: tuple[object, ...] | None = None
        self._calibration_pair_picker_thumbnail_generation = 0
        self._calibration_pair_picker_thumbnail_paths: list[tuple[str, Path]] = []
        self._calibration_pair_picker_thumbnail_index = 0
        self._calibration_pair_picker_items: dict[
            str, QtWidgets.QListWidgetItem
        ] = {}
        self._calibration_pair_picker_visible_keys: set[str] = set()
        self._calibration_pair_picker_session_started_unix_ns = time.time_ns()
        self._calibration_session_catalog_signature: tuple[object, ...] | None = None
        self._calibration_session_explicit_root = ""
        self._calibration_diagnostics_signature: tuple[object, ...] | None = None
        self._calibration_quality_signature: tuple[object, ...] | None = None
        self._calibration_quality_state = "UNVERIFIED"
        self._calibration_quality_snapshot_cache: dict[str, object] = {}
        self._calibration_quality_warning_confirmed = False
        self._calibration_capture_log_lines: dict[str, list[str]] = {
            "left": [],
            "right": [],
        }
        self._checkerboard_preview_supported = callable(
            getattr(self.backend, "advanced_refresh_calibration_preview", None)
        )
        self._checkerboard_preview_timer = QtCore.QTimer(self)
        self._checkerboard_preview_timer.setInterval(500)
        self._checkerboard_preview_timer.timeout.connect(
            self._request_checkerboard_preview
        )
        self._ftp_preview_timer = QtCore.QTimer(self)
        self._ftp_preview_timer.setSingleShot(True)
        self._ftp_preview_timer.setInterval(1000)
        self._ftp_preview_timer.timeout.connect(
            self._return_to_live_after_ftp_preview
        )
        self._ftp_auto_preview_active = False
        self._ftp_manual_hold = False
        self._camera_mode_programmatic_change = False
        self.setup_widgets: dict[str, QtWidgets.QWidget] = {}
        self._initial_calibration_setup = dict(
            _evidence(self.backend).calibration_setup()
        )

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Calibration"))

        advanced_content = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QVBoxLayout(advanced_content)
        advanced_layout.setContentsMargins(6, 6, 6, 6)
        advanced_layout.setSpacing(6)

        zoom_group = QtWidgets.QGroupBox("Actual zoom readback")
        zoom_layout = QtWidgets.QHBoxLayout(zoom_group)
        self.zoom_level_spin = QtWidgets.QSpinBox()
        # This control carries the WIRIS zoom *index*, not the optical ratio:
        # index 0 is the nominal 1.0x position and index 1 is nominal 1.2x.
        self.zoom_level_spin.setRange(0, 100)
        self.zoom_level_spin.setValue(1)
        self.zoom_level_spin.setToolTip(
            "WIRIS zoom index (index 0 = nominal 1.0x; index 1 = nominal 1.2x)."
        )
        self.zoom_set_button = QtWidgets.QPushButton("Set both")
        self.zoom_in_button = QtWidgets.QPushButton("Zoom-in")
        self.zoom_out_button = QtWidgets.QPushButton("Zoom-out")
        self.zoom_query_button = QtWidgets.QPushButton("GZVV")
        self.zoom_readback_label = QtWidgets.QLabel(
            "left=unknown, right=unknown"
        )
        self.zoom_readback_label.setObjectName("statusStrip")
        self.zoom_readback_label.setWordWrap(True)
        self.zoom_set_button.clicked.connect(
            lambda: self._send_calibration_zoom(
                "set", self.zoom_level_spin.value()
            )
        )
        self.zoom_query_button.clicked.connect(
            lambda: self._send_calibration_zoom("query", None)
        )
        self.zoom_in_button.clicked.connect(
            lambda: self._send_calibration_zoom("in", None)
        )
        self.zoom_out_button.clicked.connect(
            lambda: self._send_calibration_zoom("out", None)
        )
        self.zoom_in_button.setToolTip(
            "Send WIRIS SZIN to both cameras, then refresh the GZVV readback."
        )
        self.zoom_out_button.setToolTip(
            "Send WIRIS SZOT to both cameras, then refresh the GZVV readback."
        )
        zoom_layout.addWidget(QtWidgets.QLabel("Index"))
        zoom_layout.addWidget(self.zoom_level_spin)
        zoom_layout.addWidget(self.zoom_set_button)
        zoom_layout.addWidget(self.zoom_in_button)
        zoom_layout.addWidget(self.zoom_out_button)
        zoom_layout.addWidget(self.zoom_query_button)
        zoom_layout.addWidget(self.zoom_readback_label, 1)
        advanced_layout.addWidget(zoom_group)

        camera_group = QtWidgets.QGroupBox("Shared calibration inputs")
        camera_layout = QtWidgets.QVBoxLayout(camera_group)
        camera_layout.setContentsMargins(6, 6, 6, 6)
        camera_layout.setSpacing(5)
        camera_layout.setAlignment(QtCore.Qt.AlignTop)

        self.camera_mode_tabs = QtWidgets.QTabWidget()
        self.camera_mode_tabs.setObjectName("calibrationCameraModeTabs")
        live_page = QtWidgets.QWidget()
        live_layout = QtWidgets.QVBoxLayout(live_page)
        live_layout.setContentsMargins(4, 4, 4, 4)
        live_layout.setSpacing(5)
        self.preview_splitter = CameraPreviewPair(live_page)
        self.previews = {
            side: CameraPreview(
                side,
                crop_to_fill=False,
                lock_chrome_height=True,
                lock_image_aspect=True,
            )
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            self.preview_splitter.addWidget(self.previews[side])
        self.preview_splitter.setStretchFactor(0, 1)
        self.preview_splitter.setStretchFactor(1, 1)
        live_layout.addWidget(self.preview_splitter)
        self.camera_buttons: dict[str, QtWidgets.QPushButton] = {}

        ftp_page = QtWidgets.QWidget()
        ftp_layout = QtWidgets.QVBoxLayout(ftp_page)
        ftp_layout.setContentsMargins(4, 4, 4, 4)
        ftp_layout.setSpacing(5)
        self._ftp_preview_status_text = ""
        self.ftp_preview_splitter = CameraPreviewPair(ftp_page)
        self.ftp_previews = {
            side: CameraPreview(
                side,
                crop_to_fill=False,
                lock_chrome_height=True,
                lock_image_aspect=True,
                active_label="FTP STILL",
                empty_image_text="NO FTP IMAGE",
            )
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            self.ftp_preview_splitter.addWidget(self.ftp_previews[side])
        self.ftp_preview_splitter.setStretchFactor(0, 1)
        self.ftp_preview_splitter.setStretchFactor(1, 1)
        ftp_layout.addWidget(self.ftp_preview_splitter)

        self.camera_mode_tabs.addTab(live_page, "Live")
        self.camera_mode_tabs.addTab(ftp_page, "FTP")
        self.camera_mode_tabs.currentChanged.connect(
            self._on_camera_mode_changed
        )
        self.camera_mode_tabs.tabBar().tabBarClicked.connect(
            self._on_camera_mode_tab_clicked
        )
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        calibration_session_group = self._build_calibration_session_group()
        calibration_file_group = self._build_calibration_file_selection_group()
        self.operation_tabs = self._build_operation_tabs()
        self.capture_surface = self._build_shared_capture_surface()
        camera_layout.addWidget(self.operation_tabs)
        camera_layout.addWidget(self.camera_mode_tabs)
        camera_layout.addWidget(self.capture_surface)
        camera_layout.addWidget(calibration_session_group)
        camera_layout.addWidget(calibration_file_group)
        self._calibration_workflow_syncing = False
        self._calibration_workflow_index = 0
        self.operation_tabs.currentChanged.connect(
            self._on_calibration_kind_changed
        )
        self.intrinsics_operation_tabs.currentChanged.connect(
            self._on_calibration_workflow_changed
        )
        self.extrinsics_operation_tabs.currentChanged.connect(
            self._on_calibration_workflow_changed
        )
        self._sync_calibration_capture_surface()
        self.refresh_calibration_quality()
        advanced_layout.addWidget(camera_group)

        self._connect_calibration_capture_event_signal()
        self._connect_calibration_diagnostics_signal()
        self._connect_camera_panes()
        self._connect_calibration_preview_signal()

        profile_group = QtWidgets.QGroupBox("Active calibration bundle")
        profile_layout = QtWidgets.QGridLayout(profile_group)
        self.profile_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("state", "State"),
                ("profile_path", "K,D profile"),
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
        advanced_layout.addWidget(profile_group)

        intrinsic_group = QtWidgets.QGroupBox("Intrinsics result")
        intrinsic_layout = QtWidgets.QVBoxLayout(intrinsic_group)
        self.intrinsic_model = _table_model(
            self,
            (
                "Camera",
                "Grid px",
                "fx px",
                "fy px",
                "cx px",
                "cy px",
                "Reproj px",
                "Images",
                "Frame",
            ),
        )
        self.intrinsic_table = QtWidgets.QTableView()
        self.intrinsic_table.setModel(self.intrinsic_model)
        self.intrinsic_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.intrinsic_table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )
        self.intrinsic_table.verticalHeader().setVisible(False)
        self.intrinsic_table.setMaximumHeight(108)
        self.intrinsic_table.horizontalHeader().setStretchLastSection(True)
        intrinsic_layout.addWidget(self.intrinsic_table)
        self.intrinsics_result_group = intrinsic_group
        self.intrinsics_page_layout.addWidget(intrinsic_group)

        self.extrinsic_value = EngineeringValue("Stereo R/T")
        metrics_group = QtWidgets.QGroupBox("Extrinsics result")
        metrics_layout = QtWidgets.QVBoxLayout(metrics_group)
        metrics_layout.addWidget(self.extrinsic_value)
        self.calibration_contract = QtWidgets.QLabel()
        self.calibration_contract.setObjectName("compactContract")
        self.calibration_contract.setWordWrap(True)
        self.calibration_contract.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse
        )
        metrics_layout.addWidget(self.calibration_contract)
        self.extrinsics_result_group = metrics_group
        self.extrinsics_page_layout.addWidget(metrics_group)

        setup_group = QtWidgets.QGroupBox("Shared checkerboard setup")
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

        def setup_text(
            key: str,
            placeholder: str = "",
        ) -> QtWidgets.QLineEdit:
            widget = QtWidgets.QLineEdit(str(setup.get(key) or ""))
            if placeholder:
                widget.setPlaceholderText(placeholder)
            self.setup_widgets[key] = widget
            return widget

        def setup_combo(key: str) -> QtWidgets.QComboBox:
            widget = QtWidgets.QComboBox()
            widget.addItem(
                "Pixhawk attitude + GPS/RTK baseline", "pixhawk"
            )
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
                (
                    "Square size",
                    setup_double("square_size_mm", 1.0, 500.0, " mm"),
                ),
                ("Minimum images", setup_spin("min_images", 3, 100)),
                (
                    "Calculation pairs",
                    setup_spin(
                        "calculation_pair_count", 0, 10000, " pair"
                    ),
                ),
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
        path_form.addWidget(
            setup_text("calibration_root", DEFAULT_CALIBRATION_ROOT),
            0,
            1,
            1,
            3,
        )
        path_form.addWidget(QtWidgets.QLabel("Calculation mode"), 1, 0)
        path_form.addWidget(setup_combo("validation_mode"), 1, 1)
        setup_layout.addLayout(path_form)
        setup_contract = QtWidgets.QLabel(
            "Square: mm · pixels/intrinsics: OpenCV optical RDF · "
            "X_right = R_left_to_right X_left + T_left_to_right_m · "
            "save zoom comes from the captured camera readback"
        )
        setup_contract.setObjectName("compactContract")
        setup_contract.setWordWrap(True)
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
        advanced_layout.addWidget(setup_group)

        validation_group = QtWidgets.QGroupBox(
            "Advanced intrinsics validation (optional)"
        )
        self.checkerboard_validation_outer_group = validation_group
        validation_group.setCheckable(True)
        validation_group.setChecked(False)
        validation_layout = QtWidgets.QVBoxLayout(validation_group)
        validation_content = self._build_checkerboard_validation_group()
        validation_layout.addWidget(validation_content)
        validation_content.setVisible(False)
        validation_group.toggled.connect(validation_content.setVisible)
        validation_group.toggled.connect(
            lambda _checked: self._apply_calibration_quality_gate()
        )
        self.intrinsics_page_layout.addWidget(validation_group)
        self.intrinsics_page_layout.addStretch(1)
        self.extrinsics_page_layout.addStretch(1)
        self._sync_calibration_capture_surface()

        handoff_button = QtWidgets.QPushButton("Advanced Zoom")
        handoff_button.setToolTip(
            "Open Advanced camera controls for physical zoom commands."
        )
        handoff_button.clicked.connect(open_advanced)
        advanced_layout.addStretch(1)
        advanced_layout.addWidget(handoff_button, alignment=QtCore.Qt.AlignRight)

        self.simple_calibration_tabs = self._build_simple_calibration_tabs()
        layout.addWidget(self.simple_calibration_tabs)
        self.apply_calibration_capture_snapshot(
            self.calibration_capture_snapshot()
        )

        advanced_group = QtWidgets.QGroupBox()
        advanced_group.setObjectName("advancedCalibrationGroup")
        advanced_group.setToolTip(
            "파일별 선택, subset 품질, K/D·R/t 상세, 세션 경로를 검토합니다."
        )
        advanced_group_layout = QtWidgets.QVBoxLayout(advanced_group)
        advanced_group_layout.setContentsMargins(6, 6, 6, 6)
        advanced_toggle = QtWidgets.QToolButton()
        advanced_toggle.setObjectName("advancedCalibrationToggle")
        advanced_toggle.setText("고급 캘리브레이션")
        advanced_toggle.setCheckable(True)
        advanced_toggle.setChecked(False)
        advanced_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        advanced_toggle.setArrowType(QtCore.Qt.RightArrow)
        advanced_toggle.toggled.connect(
            lambda checked: advanced_toggle.setArrowType(
                QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow
            )
        )
        advanced_group_layout.addWidget(advanced_toggle)
        advanced_group_layout.addWidget(advanced_content)
        advanced_content.setVisible(False)
        advanced_toggle.toggled.connect(advanced_content.setVisible)
        self.advanced_calibration_group = advanced_group
        self.advanced_calibration_toggle = advanced_toggle
        self.advanced_calibration_content = advanced_content
        layout.addWidget(advanced_group)
        layout.addStretch(1)

    def _build_simple_calibration_tabs(self) -> QtWidgets.QTabWidget:
        """Build the StereoWIRIS-style primary capture-to-apply workflow.

        This is intentionally a thin command surface.  The authoritative
        calculation, quality, zoom, native-raster, persistence, and Apply
        gates remain owned by the existing Workstation controllers exposed in
        the advanced surface below it.
        """

        tabs = QtWidgets.QTabWidget()
        tabs.setObjectName("simpleCalibrationTabs")
        tabs.setDocumentMode(True)
        self.simple_calibration_buttons: dict[str, list[QtWidgets.QPushButton]] = {}

        def command_button(command_id: str, label: str) -> QtWidgets.QPushButton:
            button = _command_button(label, command_id, self.invoke_command)
            button.setMinimumWidth(150)
            button.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )
            self.simple_calibration_buttons.setdefault(command_id, []).append(button)
            self.operation_buttons.setdefault(command_id, button)
            self.operation_button_instances.setdefault(command_id, []).append(button)
            return button

        capture_page = QtWidgets.QWidget()
        capture_layout = QtWidgets.QVBoxLayout(capture_page)
        capture_layout.setContentsMargins(8, 8, 8, 8)
        capture_layout.setSpacing(6)
        capture_title = QtWidgets.QLabel(
            "1. 운용 줌을 확인하고 보드를 상·하·좌·우, 거리와 기울기를 바꿔 촬영하세요."
        )
        capture_title.setObjectName("compactContract")
        capture_title.setWordWrap(True)
        capture_layout.addWidget(capture_title)
        zoom_row = QtWidgets.QHBoxLayout()
        zoom_row.addWidget(QtWidgets.QLabel("줌 index"))
        simple_zoom_spin = QtWidgets.QSpinBox()
        simple_zoom_spin.setObjectName("simpleCalibrationZoomIndex")
        simple_zoom_spin.setRange(self.zoom_level_spin.minimum(), self.zoom_level_spin.maximum())
        simple_zoom_spin.setValue(self.zoom_level_spin.value())
        simple_zoom_spin.valueChanged.connect(self.zoom_level_spin.setValue)
        self.zoom_level_spin.valueChanged.connect(simple_zoom_spin.setValue)
        zoom_row.addWidget(simple_zoom_spin)
        simple_set_zoom = QtWidgets.QPushButton("좌우 줌 적용")
        simple_set_zoom.clicked.connect(
            lambda: self._send_calibration_zoom("set", simple_zoom_spin.value())
        )
        simple_query_zoom = QtWidgets.QPushButton("줌 확인")
        simple_query_zoom.clicked.connect(
            lambda: self._send_calibration_zoom("query", None)
        )
        zoom_row.addWidget(simple_set_zoom)
        zoom_row.addWidget(simple_query_zoom)
        zoom_row.addStretch(1)
        capture_layout.addLayout(zoom_row)
        simple_zoom_status = QtWidgets.QLabel(self.zoom_readback_label.text())
        simple_zoom_status.setObjectName("simpleCalibrationZoomStatus")
        simple_zoom_status.setWordWrap(True)
        self.simple_calibration_zoom_status = simple_zoom_status
        capture_layout.addWidget(simple_zoom_status)

        simple_preview_splitter = CameraPreviewPair(capture_page)
        simple_preview_splitter.setObjectName("simpleCalibrationPreviewPair")
        self.simple_calibration_previews = {
            side: CameraPreview(
                side,
                crop_to_fill=False,
                lock_chrome_height=True,
                lock_image_aspect=True,
                active_label="CHECKERBOARD",
                empty_image_text="NO CALIBRATION VIDEO",
                maximum_image_height=200,
            )
            for side in ("left", "right")
        }
        self._simple_calibration_detection_text = {
            "left": "보드: 프레임 없음",
            "right": "보드: 프레임 없음",
        }
        for side in ("left", "right"):
            simple_preview_splitter.addWidget(self.simple_calibration_previews[side])
            self.simple_calibration_previews[side].meta.hide()
        simple_preview_splitter.setStretchFactor(0, 1)
        simple_preview_splitter.setStretchFactor(1, 1)
        self.simple_calibration_preview_splitter = simple_preview_splitter

        board_status_row = QtWidgets.QHBoxLayout()
        board_status_row.setSpacing(6)
        self.simple_calibration_board_indicators: dict[str, StatusIndicator] = {}
        for side, label in (("left", "보드 L"), ("right", "보드 R")):
            indicator = StatusIndicator(label, compact=True)
            indicator.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )
            self.simple_calibration_board_indicators[side] = indicator
            board_status_row.addWidget(indicator, 1)
        capture_layout.addLayout(board_status_row)

        capture_actions = QtWidgets.QGridLayout()
        for index, (command_id, label) in enumerate(
            (
                ("camera.toggle", "미리보기"),
                ("calibration.capture_images", "Live pair 캡처"),
                ("calibration.capture_images_wiris_ftp_oz", "OZ 촬영 + 가져오기"),
                ("calibration.auto_capture", "자동 캡처"),
            )
        ):
            capture_actions.addWidget(command_button(command_id, label), index // 2, index % 2)
        capture_layout.addLayout(capture_actions)

        simple_run_status = _ElidedStatusLabel(
            "이번 촬영: 검사 0 · Stereo 저장 0 · K,D 전용 0 · 실패 0"
        )
        simple_run_status.setObjectName("simpleCalibrationRunStatus")
        self.simple_calibration_run_status = simple_run_status
        capture_layout.addWidget(simple_run_status)

        simple_set_status = _ElidedStatusLabel(
            "현재 캡처 세트: L 0/20 · R 0/20 · Stereo 저장 후보 0 · K,D 전용 L0/R0"
        )
        simple_set_status.setObjectName("simpleCalibrationSetStatus")
        self.simple_calibration_set_status = simple_set_status
        capture_layout.addWidget(simple_set_status)

        simple_capture_status = _ElidedStatusLabel(
            f"최근 상태: {self.capture_summary_label.text()}"
        )
        simple_capture_status.setObjectName("simpleCalibrationCaptureStatus")
        self.simple_calibration_capture_status = simple_capture_status
        capture_layout.addWidget(simple_capture_status)
        capture_layout.addWidget(simple_preview_splitter)
        capture_layout.addStretch(1)
        tabs.addTab(capture_page, "촬영")

        compute_page = QtWidgets.QWidget()
        compute_layout = QtWidgets.QVBoxLayout(compute_page)
        compute_layout.setContentsMargins(8, 8, 8, 8)
        compute_layout.setSpacing(8)
        compute_title = QtWidgets.QLabel(
            "2. 전체 사진으로 K,D를 계산하고 품질 통과 후 필요하면 stereo R,t를 계산하세요."
        )
        compute_title.setObjectName("compactContract")
        compute_title.setWordWrap(True)
        compute_layout.addWidget(compute_title)
        compute_actions = QtWidgets.QGridLayout()
        for index, (command_id, label) in enumerate(
            (
                ("calibration.run_intrinsic", "K,D 계산"),
                ("calibration.save_intrinsics", "K,D 저장"),
                ("calibration.run_extrinsics", "R,t 계산"),
                ("calibration.save_extrinsics", "R,t 저장"),
                ("calibration.apply_bundle", "검증된 bundle 적용"),
            )
        ):
            compute_actions.addWidget(command_button(command_id, label), index // 2, index % 2)
        compute_layout.addLayout(compute_actions)
        simple_quality_status = QtWidgets.QLabel(self.calibration_quality_gate_label.text())
        simple_quality_status.setObjectName("simpleCalibrationQualityStatus")
        simple_quality_status.setWordWrap(True)
        self.simple_calibration_quality_status = simple_quality_status
        compute_layout.addWidget(simple_quality_status)
        compute_note = QtWidgets.QLabel(
            "FAIL은 저장·적용을 차단하고, WARNING은 고급 캘리브레이션에서 사유 검토와 명시적 승인이 필요합니다."
        )
        compute_note.setObjectName("compactContract")
        compute_note.setWordWrap(True)
        compute_layout.addWidget(compute_note)
        compute_layout.addStretch(1)
        tabs.addTab(compute_page, "계산·적용")

        profile_page = QtWidgets.QWidget()
        profile_layout = QtWidgets.QVBoxLayout(profile_page)
        profile_layout.setContentsMargins(8, 8, 8, 8)
        profile_layout.setSpacing(8)
        profile_title = QtWidgets.QLabel(
            "3. 현재 줌·해상도와 일치하는 K,D와 호환되는 R,t만 불러와 적용합니다."
        )
        profile_title.setObjectName("compactContract")
        profile_title.setWordWrap(True)
        profile_layout.addWidget(profile_title)
        profile_actions = QtWidgets.QGridLayout()
        for index, (command_id, label) in enumerate(
            (
                ("calibration.load_intrinsics", "K,D 불러오기"),
                ("calibration.load_extrinsics", "R,t 불러오기"),
                ("calibration.apply_bundle", "검증된 bundle 적용"),
            )
        ):
            profile_actions.addWidget(command_button(command_id, label), index // 2, index % 2)
        profile_layout.addLayout(profile_actions)
        simple_profile_status = QtWidgets.QLabel(
            self.profile_values["state"].value.text()
        )
        simple_profile_status.setObjectName("simpleCalibrationProfileStatus")
        simple_profile_status.setWordWrap(True)
        self.simple_calibration_profile_status = simple_profile_status
        profile_layout.addWidget(simple_profile_status)
        profile_layout.addStretch(1)
        tabs.addTab(profile_page, "프로파일")
        return tabs

    def calibration_capture_snapshot(self) -> dict[str, object]:
        """Read the Calibration owner's capture status without deriving truth."""

        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            reader = getattr(owner, "workstation_calibration_capture_snapshot", None)
            if not callable(reader):
                continue
            try:
                snapshot = reader()
            except Exception as exc:
                return {
                    "schema": "workstation-calibration-capture-status-v1",
                    "capture_set_id": "",
                    "detection": {},
                    "run": {},
                    "capture_set": {
                        "state": "UNAVAILABLE",
                        "error": f"capture snapshot unavailable: {exc}",
                    },
                    "last": {"message": f"Capture status unavailable: {exc}"},
                }
            if isinstance(snapshot, Mapping):
                return dict(snapshot)
        adapter = _evidence(self.backend)
        calibration_reader = getattr(adapter, "calibration", None)
        calibration = calibration_reader() if callable(calibration_reader) else {}
        snapshot = calibration.get("capture_status") if isinstance(calibration, Mapping) else None
        return dict(snapshot) if isinstance(snapshot, Mapping) else {}

    @staticmethod
    def _calibration_capture_count(value: object) -> str:
        if isinstance(value, bool):
            return "--"
        try:
            count = int(value)
        except (TypeError, ValueError, OverflowError):
            return "--"
        return str(count) if count >= 0 else "--"

    def _set_simple_calibration_preview_meta(self, side: str) -> None:
        previews = getattr(self, "simple_calibration_previews", {})
        preview = previews.get(side) if isinstance(previews, dict) else None
        if preview is None:
            return
        text = str(
            getattr(self, "_simple_calibration_detection_text", {}).get(
                side,
                "보드: 상태 없음",
            )
        )
        preview.meta.setText(text)
        preview.meta.setToolTip(text)

    def apply_calibration_capture_snapshot(self, value: object) -> dict[str, object]:
        snapshot = dict(value) if isinstance(value, Mapping) else {}
        detections = snapshot.get("detection")
        detections = detections if isinstance(detections, Mapping) else {}
        state_labels = {
            "DETECTING": "검출 중",
            "FOUND": "인식",
            "NOT_FOUND": "미인식",
            "NO_FRAME": "프레임 없음",
            "STALE": "STALE",
            "UNAVAILABLE": "사용 불가",
        }
        state_severity = {
            "DETECTING": StatusSeverity.INFO,
            "FOUND": StatusSeverity.OK,
            "NOT_FOUND": StatusSeverity.INFO,
            "NO_FRAME": StatusSeverity.UNKNOWN,
            "STALE": StatusSeverity.WARN,
            "UNAVAILABLE": StatusSeverity.WARN,
        }
        for side in ("left", "right"):
            row = detections.get(side)
            row = row if isinstance(row, Mapping) else {}
            state = str(row.get("state") or "NO_FRAME").upper()
            corner_count = self._calibration_capture_count(row.get("corner_count"))
            display_state = state_labels.get(state, state)
            if state == "FOUND" and corner_count != "--":
                display_state += f" {corner_count}"
            try:
                age_s = float(row.get("age_s"))
            except (TypeError, ValueError, OverflowError):
                age_s = None
            else:
                if not math.isfinite(age_s) or age_s < 0.0:
                    age_s = None
            detail = str(row.get("detail") or "checkerboard evidence unavailable")
            indicator = getattr(self, "simple_calibration_board_indicators", {}).get(side)
            if isinstance(indicator, StatusIndicator):
                indicator.set_status(
                    RuntimeStatus(
                        key=f"calibration.board.{side}",
                        label=f"보드 {'L' if side == 'left' else 'R'}",
                        state=display_state,
                        severity=state_severity.get(state, StatusSeverity.UNKNOWN),
                        detail=detail,
                        source="CheckerboardPreviewWorker",
                        age_s=age_s,
                        clock="app-receive monotonic",
                        observed_monotonic_s=time.monotonic(),
                    )
                )
            age_text = "" if age_s is None else f" · {age_s:.1f} s"
            self._simple_calibration_detection_text[side] = (
                f"보드: {display_state}{age_text}"
            )
            self._set_simple_calibration_preview_meta(side)

        run = snapshot.get("run")
        run = run if isinstance(run, Mapping) else {}
        run_text = (
            "이번 촬영: 검사 "
            f"{self._calibration_capture_count(run.get('inspection_request_count'))} · "
            "Stereo 저장 "
            f"{self._calibration_capture_count(run.get('stereo_saved_count'))} · "
            "K,D 전용 "
            f"{self._calibration_capture_count(run.get('intrinsics_only_saved_count'))} · "
            f"실패 {self._calibration_capture_count(run.get('failed_count'))}"
        )
        self.simple_calibration_run_status.set_full_text(run_text)
        self.simple_calibration_run_status.setToolTip(
            run_text
            + "\n검사는 checkerboard 저장 검증 worker가 실제 시작된 경우만 셉니다."
        )

        capture_set = snapshot.get("capture_set")
        capture_set = capture_set if isinstance(capture_set, Mapping) else {}
        minimum = self._calibration_capture_count(
            snapshot.get("minimum_images_per_side")
        )
        set_id = str(snapshot.get("capture_set_id") or "--")
        set_text = (
            f"현재 캡처 세트 {set_id}: "
            f"L {self._calibration_capture_count(capture_set.get('left_image_count'))}/{minimum} · "
            f"R {self._calibration_capture_count(capture_set.get('right_image_count'))}/{minimum} · "
            "Stereo 저장 후보 "
            f"{self._calibration_capture_count(capture_set.get('stereo_candidate_count'))} · "
            "K,D 전용 L"
            f"{self._calibration_capture_count(capture_set.get('left_intrinsics_only_count'))}/R"
            f"{self._calibration_capture_count(capture_set.get('right_intrinsics_only_count'))}"
        )
        warning_count = self._calibration_capture_count(
            capture_set.get("evidence_warning_pair_count")
        )
        if warning_count not in {"--", "0"}:
            set_text += f" · 증거 경고 {warning_count}"
        self.simple_calibration_set_status.set_full_text(set_text)
        set_tooltip = [set_text]
        capture_root = str(snapshot.get("capture_root") or capture_set.get("root") or "")
        if capture_root:
            set_tooltip.append(f"Root: {capture_root}")
        error = str(capture_set.get("error") or "")
        if error:
            set_tooltip.append(f"Inventory: {error}")
        set_tooltip.append(
            "Stereo 저장 후보는 수동 제외 및 solver pose 필터 전의 persisted pair 수입니다."
        )
        self.simple_calibration_set_status.setToolTip("\n".join(set_tooltip))

        last = snapshot.get("last")
        last = last if isinstance(last, Mapping) else {}
        message = str(last.get("message") or "").strip()
        if not message:
            message = self.capture_summary_label.text()
        target = self.simple_calibration_capture_status
        target.set_full_text(f"최근 상태: {message}")
        target.setToolTip(message)
        return snapshot

    def _sync_simple_calibration_status(
        self,
        capture_status: object = None,
    ) -> None:
        """Mirror authoritative advanced status text into the simple surface."""

        for target_name, source_name in (
            ("simple_calibration_zoom_status", "zoom_readback_label"),
            ("simple_calibration_quality_status", "calibration_quality_gate_label"),
        ):
            target = getattr(self, target_name, None)
            source = getattr(self, source_name, None)
            if isinstance(target, QtWidgets.QLabel) and isinstance(source, QtWidgets.QLabel):
                target.setText(source.text())
        profile = getattr(self, "simple_calibration_profile_status", None)
        state = getattr(getattr(self, "profile_values", {}).get("state"), "value", None)
        if isinstance(profile, QtWidgets.QLabel) and isinstance(state, QtWidgets.QLabel):
            profile.setText(f"Active bundle: {state.text()}")
        if hasattr(self, "simple_calibration_run_status"):
            self.apply_calibration_capture_snapshot(
                capture_status
                if isinstance(capture_status, Mapping)
                else self.calibration_capture_snapshot()
            )

    def _send_calibration_zoom(
        self,
        action: str,
        level: int | None,
    ) -> None:
        sender = getattr(self.backend, "advanced_send_zoom", None)
        if not callable(sender):
            self.zoom_readback_label.setText("Zoom control unavailable")
            return
        try:
            sender("both", action, level)
        except Exception as exc:
            self.zoom_readback_label.setText(
                f"Zoom command failed: {exc}"
            )
            return
        if action == "set":
            status = f"Requested both={level}×; GZVV readback pending"
        elif action == "in":
            status = "Requested Zoom-in for both; GZVV readback pending"
        elif action == "out":
            status = "Requested Zoom-out for both; GZVV readback pending"
        else:
            status = "GZVV readback pending"
        self.zoom_readback_label.setText(status)
        self._sync_simple_calibration_status()

    def _build_operation_tabs(self) -> QtWidgets.QTabWidget:
        kind_tabs = _ContentSizedOperationTabs()
        kind_tabs.setObjectName("calibrationKindTabs")
        kind_tabs.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Maximum,
        )
        self.operation_button_instances: dict[
            str, list[QtWidgets.QPushButton]
        ] = {}

        def command_button(
            command_id: str,
            label: str,
        ) -> QtWidgets.QPushButton:
            button = _command_button(
                label, command_id, self.invoke_command
            )
            button.setMinimumWidth(0)
            button.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )
            self.operation_buttons.setdefault(command_id, button)
            self.operation_button_instances.setdefault(
                command_id, []
            ).append(button)
            return button

        def workflow_tabs(
            *,
            kind: str,
            capture_actions: tuple[tuple[str, str], ...],
            compute_actions: tuple[tuple[str, str], ...],
            profile_actions: tuple[tuple[str, str], ...],
            capture_hint: str,
            include_auto_capture: bool,
            include_intrinsic_quality: bool,
        ) -> QtWidgets.QTabWidget:
            tabs = _ContentSizedOperationTabs()
            tabs.setObjectName(f"calibration{kind}WorkflowTabs")
            tabs.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Maximum,
            )
            sections = (
                ("Capture", capture_actions),
                ("Compute", compute_actions),
                ("Profiles", profile_actions),
            )
            for tab_label, actions in sections:
                page = QtWidgets.QWidget()
                page_layout = QtWidgets.QVBoxLayout(page)
                page_layout.setContentsMargins(6, 6, 6, 6)
                page_layout.setSpacing(6)
                actions_layout = QtWidgets.QGridLayout()
                columns = 4 if tab_label == "Capture" else 3
                for index, (command_id, label) in enumerate(actions):
                    actions_layout.addWidget(
                        command_button(command_id, label),
                        index // columns,
                        index % columns,
                    )
                page_layout.addLayout(actions_layout)
                if tab_label == "Capture":
                    hint = QtWidgets.QLabel(capture_hint)
                    hint.setObjectName("compactContract")
                    hint.setWordWrap(True)
                    hint.setFixedHeight(hint.fontMetrics().lineSpacing() * 2 + 6)
                    page_layout.addWidget(hint)
                if tab_label == "Capture" and include_auto_capture:
                    auto_group = QtWidgets.QGroupBox(
                        "Automatic capture"
                    )
                    auto_layout = QtWidgets.QGridLayout(auto_group)
                    auto_layout.setHorizontalSpacing(8)
                    auto_layout.setVerticalSpacing(5)
                    auto_layout.setColumnStretch(1, 1)
                    auto_interval = QtWidgets.QDoubleSpinBox()
                    auto_interval.setRange(0.1, 10.0)
                    auto_interval.setSingleStep(0.1)
                    auto_interval.setDecimals(1)
                    auto_interval.setSuffix(" s")
                    auto_interval.setValue(
                        float(
                            self._initial_calibration_setup.get(
                                "auto_interval_s"
                            )
                            or 1.0
                        )
                    )
                    self.setup_widgets["auto_interval_s"] = (
                        auto_interval
                    )
                    auto_button = command_button(
                        "calibration.auto_capture",
                        "Start auto capture",
                    )
                    auto_button.setToolTip(
                        "Capture only new left/right checkerboard "
                        "detections whose decoded-frame app-receive timestamps differ "
                        "by no more than the configured Max Live pair Δt."
                    )
                    auto_layout.addWidget(
                        QtWidgets.QLabel("Interval"), 0, 0
                    )
                    auto_layout.addWidget(auto_interval, 0, 1)
                    auto_layout.addWidget(
                        auto_button, 1, 0, 1, 2
                    )
                    page_layout.addWidget(auto_group)
                    page_layout.addWidget(
                        self._build_calibration_capture_response_group()
                    )
                if (
                    tab_label == "Compute"
                    and include_intrinsic_quality
                ):
                    page_layout.addWidget(
                        self._build_calibration_quality_group()
                    )
                    page_layout.addWidget(
                        self._build_calibration_outlier_group()
                    )
                page_layout.addStretch(1)
                tabs.addTab(page, tab_label)
            return tabs

        intrinsics_page = QtWidgets.QWidget()
        self.intrinsics_page_layout = QtWidgets.QVBoxLayout(
            intrinsics_page
        )
        self.intrinsics_page_layout.setContentsMargins(6, 6, 6, 6)
        self.intrinsics_page_layout.setAlignment(QtCore.Qt.AlignTop)
        intrinsics_contract = QtWidgets.QLabel(
            "Per-camera image pixels → K,D in OpenCV optical RDF. "
            "Profiles are camera-, native-raster-, focus-, and "
            "zoom-specific."
        )
        intrinsics_contract.setObjectName("compactContract")
        intrinsics_contract.setWordWrap(True)
        self.intrinsics_contract_label = intrinsics_contract
        self.intrinsics_page_layout.addWidget(intrinsics_contract)
        self.intrinsics_operation_tabs = workflow_tabs(
            kind="Intrinsics",
            capture_actions=(),
            compute_actions=(
                (
                    "calibration.run_intrinsic",
                    "Run calibration (K,D)",
                ),
                (
                    "calibration.save_intrinsics",
                    "Save Intrinsics",
                ),
            ),
            profile_actions=(
                (
                    "calibration.load_intrinsics",
                    "Load Intrinsics",
                ),
            ),
            capture_hint=(
                "Each camera contributes its own checkerboard image "
                "set. Left/right files remain separate inputs to the "
                "two K,D solves."
            ),
            include_auto_capture=False,
            include_intrinsic_quality=True,
        )
        self.intrinsics_page_layout.addWidget(
            self.intrinsics_operation_tabs
        )
        kind_tabs.addTab(intrinsics_page, "Intrinsics")

        extrinsics_page = QtWidgets.QWidget()
        self.extrinsics_page_layout = QtWidgets.QVBoxLayout(
            extrinsics_page
        )
        self.extrinsics_page_layout.setContentsMargins(6, 6, 6, 6)
        self.extrinsics_page_layout.setAlignment(QtCore.Qt.AlignTop)
        self.extrinsics_bundle_contract_label = QtWidgets.QLabel(
            "Input bundle: Left K,D + Right K,D + synchronized "
            "stereo pairs. Output: "
            "X_right = R_left_to_right X_left + "
            "T_left_to_right_m in OpenCV optical RDF."
        )
        self.extrinsics_bundle_contract_label.setObjectName(
            "compactContract"
        )
        self.extrinsics_bundle_contract_label.setWordWrap(True)
        self.extrinsics_bundle_contract_label.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse
        )
        self.extrinsics_page_layout.addWidget(
            self.extrinsics_bundle_contract_label
        )
        self.extrinsics_operation_tabs = workflow_tabs(
            kind="Extrinsics",
            capture_actions=(),
            compute_actions=(
                ("calibration.run_extrinsics", "Calculate R,t"),
                ("calibration.save_extrinsics", "Save R,t"),
            ),
            profile_actions=(
                ("calibration.load_extrinsics", "Load R,t"),
                (
                    "calibration.apply_bundle",
                    "Apply verified K,D + R,t bundle",
                ),
            ),
            capture_hint=(
                "Extrinsics consumes synchronized left/right rows "
                "from the shared photo selection. Applied Left/Right "
                "K,D must match the native raster. Camera, focus, and zoom "
                "provenance is operator-reviewed and does not gate readiness."
            ),
            include_auto_capture=False,
            include_intrinsic_quality=False,
        )
        self.extrinsics_page_layout.addWidget(
            self.extrinsics_operation_tabs
        )
        kind_tabs.addTab(extrinsics_page, "Extrinsics")
        self._update_calibration_compute_selection_state()
        return kind_tabs

    def _register_capture_button(
        self,
        command_id: str,
        label: str,
    ) -> QtWidgets.QPushButton:
        button = _command_button(label, command_id, self.invoke_command)
        button.setMinimumWidth(0)
        button.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Fixed,
        )
        self.operation_buttons[command_id] = button
        self.operation_button_instances.setdefault(command_id, []).append(button)
        return button

    def _build_shared_capture_surface(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Capture while viewing")
        group.setObjectName("calibrationSharedCaptureSurface")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        actions = QtWidgets.QGridLayout()
        actions.setHorizontalSpacing(6)
        actions.setVerticalSpacing(5)
        capture_actions = (
            ("camera.toggle", "Preview"),
            ("calibration.capture_images", "Capture Live Pair"),
            ("calibration.auto_capture", "Start auto capture"),
            ("calibration.capture_images_wiris_ftp", "Capture HR Still (FTP)"),
            ("calibration.capture_images_wiris_ftp_oz", "Capture OZ Still (FTP)"),
            ("calibration.capture_images_wiris_oz_capture_only", "OZ 촬영"),
            ("calibration.sync_images_wiris_oz_ftp", "FTP 자동 전송"),
        )
        for index, (command_id, label) in enumerate(capture_actions):
            if command_id == "camera.toggle":
                button = _command_button(label, command_id, self.invoke_command)
                self.camera_buttons[command_id] = button
            else:
                button = self._register_capture_button(command_id, label)
                if command_id == "calibration.auto_capture":
                    button.setToolTip(
                        "Capture checkerboards only from exact NAL1 non-delta native "
                        "FrameKeys. Expired keys wait for the next candidate; pairs over "
                        "Max Live pair Δt remain per-camera K,D inputs and are excluded "
                        "from stereo R,t."
                    )
            actions.addWidget(button, index // 4, index % 4)
            if command_id == "calibration.capture_images":
                self.capture_primary_button = button
                font = button.font()
                font.setBold(True)
                button.setFont(font)
        layout.addLayout(actions)

        pair_timing = QtWidgets.QHBoxLayout()
        pair_timing_label = QtWidgets.QLabel("Max Live pair Δt")
        pair_timing_label.setToolTip(
            "Maximum absolute left/right timing difference accepted for a calibration "
            "stereo pair. Live capture uses same-host decoded-frame app-receive monotonic "
            "timestamps; FTP capture uses the synchronized CPTR host-dispatch proxy. "
            "Neither host proxy proves exposure simultaneity. Captures over the limit "
            "remain available per camera for K,D only."
        )
        pair_max_delta = QtWidgets.QDoubleSpinBox()
        pair_max_delta.setObjectName("calibrationPairMaxDeltaMs")
        pair_max_delta.setRange(0.0, 1000.0)
        pair_max_delta.setSingleStep(5.0)
        pair_max_delta.setDecimals(1)
        initial_pair_max_delta = self._initial_calibration_setup.get(
            "pair_max_delta_ms"
        )
        pair_max_delta.setValue(
            100.0
            if initial_pair_max_delta is None
            else float(initial_pair_max_delta)
        )
        pair_max_delta.setSuffix(" ms")
        pair_max_delta.setToolTip(pair_timing_label.toolTip())
        self.setup_widgets["pair_max_delta_ms"] = pair_max_delta
        pair_timing.addWidget(pair_timing_label)
        pair_timing.addWidget(pair_max_delta)
        pair_timing.addWidget(
            QtWidgets.QLabel(
                "Live: app-receive proxy · FTP: CPTR dispatch proxy · exposure unverified"
            )
        )
        pair_timing.addStretch(1)
        layout.addLayout(pair_timing)

        self.capture_summary_label = QtWidgets.QLabel(
            "L: WAITING · R: WAITING · Pair: --"
        )
        self.capture_summary_label.setObjectName("statusStrip")
        self.capture_summary_label.setWordWrap(True)
        layout.addWidget(self.capture_summary_label)
        self._capture_summary_states = {"left": "WAITING", "right": "WAITING"}
        self._capture_pair_text = "--"

        auto_group = QtWidgets.QGroupBox("Auto capture settings")
        self.auto_capture_settings_group = auto_group
        auto_group.setCheckable(True)
        auto_group.setChecked(False)
        auto_content = QtWidgets.QWidget()
        self.auto_capture_settings_content = auto_content
        auto_layout = QtWidgets.QGridLayout(auto_content)
        auto_layout.setContentsMargins(6, 4, 6, 4)
        auto_interval = QtWidgets.QDoubleSpinBox()
        auto_interval.setRange(0.1, 10.0)
        auto_interval.setSingleStep(0.1)
        auto_interval.setDecimals(1)
        auto_interval.setSuffix(" s")
        auto_interval.setValue(
            float(self._initial_calibration_setup.get("auto_interval_s") or 1.0)
        )
        self.setup_widgets["auto_interval_s"] = auto_interval
        auto_layout.addWidget(QtWidgets.QLabel("Interval"), 0, 0)
        auto_layout.addWidget(auto_interval, 0, 1)
        auto_group_layout = QtWidgets.QVBoxLayout(auto_group)
        auto_group_layout.addWidget(auto_content)
        auto_content.setVisible(False)
        auto_group.toggled.connect(auto_content.setVisible)
        layout.addWidget(auto_group)

        response_group = QtWidgets.QGroupBox("Capture command / response")
        self.capture_response_outer_group = response_group
        response_group.setCheckable(True)
        response_group.setChecked(False)
        response_content = self._build_calibration_capture_response_group()
        self.capture_response_content = response_content
        response_layout = QtWidgets.QVBoxLayout(response_group)
        response_layout.addWidget(response_content)
        response_content.setVisible(False)
        response_group.toggled.connect(response_content.setVisible)
        layout.addWidget(response_group)
        return group

    def _on_calibration_workflow_changed(self, index: int) -> None:
        if self._calibration_workflow_syncing:
            return
        self._calibration_workflow_index = max(0, int(index))
        self._calibration_workflow_syncing = True
        try:
            for tabs in (
                self.intrinsics_operation_tabs,
                self.extrinsics_operation_tabs,
            ):
                if tabs.currentIndex() != self._calibration_workflow_index:
                    tabs.setCurrentIndex(self._calibration_workflow_index)
        finally:
            self._calibration_workflow_syncing = False
        self._sync_calibration_capture_surface()

    def _on_calibration_kind_changed(self, _index: int) -> None:
        self._calibration_workflow_syncing = True
        try:
            workflow_tabs = (
                self.intrinsics_operation_tabs
                if self.operation_tabs.currentIndex() == 0
                else self.extrinsics_operation_tabs
            )
            if workflow_tabs.currentIndex() != self._calibration_workflow_index:
                workflow_tabs.setCurrentIndex(self._calibration_workflow_index)
        finally:
            self._calibration_workflow_syncing = False
        self._sync_calibration_capture_surface()

    def _sync_calibration_capture_surface(self, _index: int = -1) -> None:
        kind_index = self.operation_tabs.currentIndex()
        workflow_tabs = (
            self.intrinsics_operation_tabs
            if kind_index == 0
            else self.extrinsics_operation_tabs
        )
        capture_visible = workflow_tabs.currentIndex() == 0
        self.camera_mode_tabs.setVisible(capture_visible)
        self.capture_surface.setVisible(capture_visible)
        if hasattr(self, "intrinsics_result_group"):
            self.intrinsics_result_group.setVisible(
                kind_index == 0 and workflow_tabs.currentIndex() != 0
            )
        if hasattr(self, "extrinsics_result_group"):
            self.extrinsics_result_group.setVisible(
                kind_index == 1 and workflow_tabs.currentIndex() != 0
            )
        if hasattr(self, "checkerboard_validation_outer_group"):
            self.checkerboard_validation_outer_group.setVisible(
                kind_index == 0 and workflow_tabs.currentIndex() == 2
            )
        if hasattr(self, "capture_primary_button"):
            self.capture_primary_button.setText(
                "Capture Live Pair"
                if kind_index == 0
                else "Capture Stereo Pair"
            )
        if capture_visible:
            self.operation_tabs.setFixedHeight(260)
        else:
            self.operation_tabs.setMinimumHeight(0)
            self.operation_tabs.setMaximumHeight(QtWidgets.QWIDGETSIZE_MAX)
            self.operation_tabs.updateGeometry()


    def _build_calibration_capture_response_group(self) -> QtWidgets.QGroupBox:
        """Show the raw capture lifecycle separately for the two cameras."""

        group = QtWidgets.QGroupBox("Capture command / response")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)
        split = QtWidgets.QHBoxLayout()
        split.setSpacing(6)
        self.calibration_capture_response_edits: dict[str, QtWidgets.QPlainTextEdit] = {}
        self.calibration_capture_response_status: dict[str, QtWidgets.QLabel] = {}
        for side, title in (("left", "Left camera"), ("right", "Right camera")):
            side_group = QtWidgets.QGroupBox(title)
            side_layout = QtWidgets.QVBoxLayout(side_group)
            side_layout.setContentsMargins(5, 5, 5, 5)
            status = QtWidgets.QLabel("대기")
            status.setObjectName("statusStrip")
            status.setWordWrap(True)
            log = QtWidgets.QPlainTextEdit()
            log.setReadOnly(True)
            log.setMaximumBlockCount(160)
            log.setMinimumHeight(92)
            log.setPlaceholderText("이 카메라의 capture 명령과 응답이 표시됩니다.")
            self.calibration_capture_response_status[side] = status
            self.calibration_capture_response_edits[side] = log
            side_layout.addWidget(status)
            side_layout.addWidget(log)
            split.addWidget(side_group, 1)
        layout.addLayout(split)
        actions = QtWidgets.QHBoxLayout()
        actions.addStretch(1)
        clear_button = QtWidgets.QPushButton("Clear capture responses")
        clear_button.clicked.connect(self._clear_calibration_capture_response_logs)
        actions.addWidget(clear_button)
        layout.addLayout(actions)
        return group

    def _connect_calibration_capture_event_signal(self) -> None:
        panel = getattr(self.backend, "panel", None)
        signal = getattr(panel, "calibration_capture_event", None)
        connect = getattr(signal, "connect", None)
        if callable(connect):
            connect(self._append_calibration_capture_event)

    def _connect_calibration_diagnostics_signal(self) -> None:
        panel = getattr(self.backend, "panel", None)
        signal = getattr(panel, "calibration_diagnostics_changed", None)
        connect = getattr(signal, "connect", None)
        if callable(connect):
            connect(self._on_calibration_diagnostics_changed)

    def _on_calibration_diagnostics_changed(self, _payload: object = None) -> None:
        self._calibration_diagnostics_signature = None
        self._sync_calibration_outlier_view()

    def _clear_calibration_capture_response_logs(self) -> None:
        for side in ("left", "right"):
            self._calibration_capture_log_lines[side] = []
            status = self.calibration_capture_response_status.get(side)
            if status is not None:
                status.setText("대기")
            editor = self.calibration_capture_response_edits.get(side)
            if editor is not None:
                editor.clear()
        self._capture_summary_states = {"left": "WAITING", "right": "WAITING"}
        self._capture_pair_text = "--"
        self._refresh_capture_summary()

    def _refresh_capture_summary(self) -> None:
        self.capture_summary_label.setText(
            f"L: {self._capture_summary_states['left']} · "
            f"R: {self._capture_summary_states['right']} · "
            f"Pair: {self._capture_pair_text}"
        )
        self._sync_simple_calibration_status()

    def _append_calibration_capture_event(self, payload: object) -> None:
        if not isinstance(payload, Mapping):
            return
        operation = str(payload.get("operation") or "Capture Live Pair")
        phase = str(payload.get("phase") or "event")
        if phase == "ftp_pair_ready":
            self._capture_summary_states = {"left": "READY", "right": "READY"}
            pair_delta = payload.get("pair_delta_ms", payload.get("timestamp_delta_ms"))
            if pair_delta is None:
                self._capture_pair_text = "READY"
            else:
                try:
                    self._capture_pair_text = f"Δt={float(pair_delta):.1f} ms"
                except (TypeError, ValueError):
                    self._capture_pair_text = str(pair_delta)
            self._refresh_capture_summary()
            self._show_ftp_pair_preview(payload)
            return
        if "FTP" in operation and phase == "failed":
            detail = str(payload.get("response") or payload.get("state") or "unknown error")
            self._set_ftp_preview_status(
                f"{operation} failed: {detail} · last successful FTP pair retained."
            )
        side = str(payload.get("side") or "").strip().lower()
        if side not in {"left", "right"}:
            return
        state = str(payload.get("state") or "")
        self._capture_summary_states[side] = (state or phase).upper()
        pair_delta = payload.get("pair_delta_ms", payload.get("timestamp_delta_ms"))
        if pair_delta is not None:
            try:
                self._capture_pair_text = f"Δt={float(pair_delta):.1f} ms"
            except (TypeError, ValueError):
                self._capture_pair_text = str(pair_delta)
        self._refresh_capture_summary()
        command = str(payload.get("command") or "").strip()
        response = str(payload.get("response") or "").strip()
        capture_id = str(payload.get("capture_id") or "").strip()
        timestamp = QtCore.QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        prefix = f"[{timestamp}] {operation}"
        if capture_id:
            prefix += f" {capture_id}"
        lines: list[str]
        if phase == "command":
            lines = [f"{prefix} · {side}", f"> {command or '<command>'}"]
        elif phase == "response":
            lines = [f"{prefix} · {side}", f"< {command or '<command>'}: {response or '<empty>'}"]
        else:
            detail = response or str(payload.get("path") or "") or state or phase
            lines = [f"{prefix} · {phase} · {state or 'unknown'}: {detail}"]
            remote_path = str(payload.get("remote_path") or "").strip()
            if remote_path:
                lines.append(f"remote: {remote_path}")
        buffer = self._calibration_capture_log_lines.setdefault(side, [])
        buffer.extend(lines)
        del buffer[:-160]
        editor = self.calibration_capture_response_edits.get(side)
        if editor is not None:
            editor.setPlainText("\n".join(buffer))
            cursor = editor.textCursor()
            cursor.movePosition(QtGui.QTextCursor.End)
            editor.setTextCursor(cursor)
        status = self.calibration_capture_response_status.get(side)
        if status is not None:
            status.setText(f"{operation}: {state or phase}")

    def _set_ftp_preview_status(self, text: object) -> None:
        """Retain FTP preview detail without reserving a visible status strip."""

        self._ftp_preview_status_text = str(text or "")
        ftp_index = self.camera_mode_tabs.indexOf(self.ftp_preview_splitter.parentWidget())
        if ftp_index >= 0:
            self.camera_mode_tabs.setTabToolTip(ftp_index, self._ftp_preview_status_text)

    def _show_ftp_pair_preview(self, payload: Mapping[str, object]) -> None:
        raw_paths = payload.get("paths")
        paths = raw_paths if isinstance(raw_paths, Mapping) else {}
        images: dict[str, QtGui.QImage] = {}
        for side in ("left", "right"):
            path = str(paths.get(side) or "").strip()
            image = QtGui.QImage(path) if path else QtGui.QImage()
            if image.isNull():
                self._set_ftp_preview_status(
                    f"FTP preview failed: {side} image is missing or unreadable ({path or 'no path'})."
                )
                return
            images[side] = image

        capture_id = str(payload.get("capture_id") or "").strip() or "unknown"
        operation = str(payload.get("operation") or "FTP capture")
        for side in ("left", "right"):
            path = str(paths.get(side) or "")
            self.ftp_previews[side].set_frame(
                images[side],
                {
                    "capture_id": capture_id,
                    "path": path,
                    "width": images[side].width(),
                    "height": images[side].height(),
                },
            )
        left_name = Path(str(paths.get("left") or "")).name
        right_name = Path(str(paths.get("right") or "")).name
        self._set_ftp_preview_status(
            f"{operation} · {capture_id} · Left={left_name} · Right={right_name}"
        )

        self._ftp_preview_timer.stop()
        if self._ftp_manual_hold and self.camera_mode_tabs.currentIndex() == 1:
            self._ftp_auto_preview_active = False
            return
        self._ftp_auto_preview_active = True
        self._camera_mode_programmatic_change = True
        try:
            self.camera_mode_tabs.setCurrentIndex(1)
        finally:
            self._camera_mode_programmatic_change = False
        self._ftp_preview_timer.start()

    def _on_camera_mode_tab_clicked(self, index: int) -> None:
        self._ftp_manual_hold = index == 1
        self._ftp_auto_preview_active = False
        self._ftp_preview_timer.stop()

    def _on_camera_mode_changed(self, index: int) -> None:
        if self._camera_mode_programmatic_change:
            return
        self._ftp_manual_hold = index == 1
        self._ftp_auto_preview_active = False
        self._ftp_preview_timer.stop()

    def _return_to_live_after_ftp_preview(self) -> None:
        if not self._ftp_auto_preview_active:
            return
        self._ftp_auto_preview_active = False
        if self.camera_mode_tabs.currentIndex() != 1:
            return
        self._camera_mode_programmatic_change = True
        try:
            self.camera_mode_tabs.setCurrentIndex(0)
        finally:
            self._camera_mode_programmatic_change = False

    def _build_calibration_session_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Calibration session")
        layout = QtWidgets.QGridLayout(group)
        layout.setHorizontalSpacing(8)
        self.calibration_session_combo = QtWidgets.QComboBox()
        self.calibration_session_combo.setObjectName("calibrationSessionSelector")
        self.calibration_session_combo.currentIndexChanged.connect(
            self._on_calibration_session_selected
        )
        self.calibration_session_refresh_button = QtWidgets.QPushButton("Refresh sessions")
        self.calibration_session_refresh_button.clicked.connect(
            lambda _checked=False: self._refresh_calibration_session_selector(force=True)
        )
        self.calibration_session_path_label = QtWidgets.QLabel("No saved session selected")
        self.calibration_session_path_label.setObjectName("statusStrip")
        self.calibration_session_path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.calibration_session_result_label = QtWidgets.QLabel(
            "Results: beside the selected left/right photo set"
        )
        self.calibration_session_result_label.setObjectName("compactContract")
        self.calibration_session_result_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(QtWidgets.QLabel("Photo set"), 0, 0)
        layout.addWidget(self.calibration_session_combo, 0, 1)
        layout.addWidget(self.calibration_session_refresh_button, 0, 2)
        layout.addWidget(self.calibration_session_path_label, 1, 0, 1, 3)
        layout.addWidget(self.calibration_session_result_label, 2, 0, 1, 3)
        self._refresh_calibration_session_selector(force=True)
        return group

    def _refresh_calibration_session_selector(self, *, force: bool = False) -> None:
        combo = getattr(self, "calibration_session_combo", None)
        if combo is None:
            return
        configured = str(
            _evidence(self.backend).calibration_setup().get("calibration_root_resolved") or ""
        )
        default_base = Path(DEFAULT_CALIBRATION_ROOT)
        catalog_roots = [default_base]
        configured_base = Path(configured) if configured else None
        if (
            configured_base is not None
            and configured_base.is_dir()
            and configured_base.resolve() != default_base.resolve()
        ):
            catalog_roots.append(configured_base)
        sessions_by_root = {}
        for catalog_root in catalog_roots:
            for session in discover_calibration_sessions(catalog_root):
                sessions_by_root[str(session.root)] = session
        sessions = tuple(
            sorted(
                sessions_by_root.values(),
                key=lambda session: (session.updated_ns, str(session.root)),
                reverse=True,
            )
        )
        signature = (
            tuple(str(root) for root in catalog_roots),
            tuple((str(session.root), session.pair_count, session.updated_ns) for session in sessions),
        )
        if not force and signature == self._calibration_session_catalog_signature:
            return
        self._calibration_session_catalog_signature = signature
        selected_root = self._calibration_session_explicit_root
        combo.blockSignals(True)
        try:
            combo.clear()
            combo.addItem("Select a photo set…", None)
            for session in sessions:
                combo.addItem(session.display_name, str(session.root))
            selected_index = combo.findData(selected_root) if selected_root else 0
            if selected_index > 0:
                combo.setCurrentIndex(selected_index)
                self.calibration_session_path_label.setText(
                    str(combo.currentData() or "No saved session selected")
                )
                self.calibration_session_result_label.setText(
                    f"Results: {combo.currentData()}/intrinsics_zoom_##.json and intrinsics.json"
                )
            else:
                self._calibration_session_explicit_root = ""
                combo.setCurrentIndex(0)
                self.calibration_session_path_label.setText("No saved session selected")
                self.calibration_session_result_label.setText(
                    "Select a Photo set before loading calculation inputs."
                )
        finally:
            combo.blockSignals(False)
        self._update_calibration_compute_selection_state()

    def _on_calibration_session_selected(self, index: int) -> None:
        combo = getattr(self, "calibration_session_combo", None)
        if combo is None or index < 0:
            return
        root = str(combo.itemData(index) or "").strip()
        if not root:
            self._calibration_session_explicit_root = ""
            self.calibration_session_path_label.setText("No saved session selected")
            self.calibration_session_result_label.setText(
                "Select a Photo set before loading calculation inputs."
            )
            self._calibration_file_picker_signature = None
            self._update_calibration_compute_selection_state()
            self._sync_calibration_file_selection()
            return
        root_widget = self.setup_widgets.get("calibration_root")
        if isinstance(root_widget, QtWidgets.QLineEdit):
            root_widget.setText(root)
        apply_setup = getattr(self.backend, "calibration_apply_setup", None)
        if callable(apply_setup):
            try:
                apply_setup(self.setup_payload())
            except Exception as exc:
                self._calibration_session_explicit_root = ""
                self.calibration_session_path_label.setText(f"Session apply failed: {exc}")
                self._update_calibration_compute_selection_state()
                return
        self._calibration_session_explicit_root = root
        self.calibration_session_path_label.setText(root)
        self.calibration_session_result_label.setText(
            f"Results: {root}/intrinsics_zoom_##.json and intrinsics.json"
        )
        self._calibration_pair_picker_signature = None
        self._calibration_file_picker_signature = None
        self._update_calibration_compute_selection_state()
        self._sync_calibration_file_selection()
        self._calibration_diagnostics_signature = None
        self._sync_calibration_outlier_view()

    def apply_selected_calibration_session(self) -> bool:
        """Apply the visible session choice immediately before a compute action."""

        self._last_calibration_session_apply_performed = False
        combo = getattr(self, "calibration_session_combo", None)
        if combo is None:
            return False
        root = str(combo.currentData() or "").strip()
        if not root or root != self._calibration_session_explicit_root:
            self.calibration_session_path_label.setText(
                "Select a Photo set before running calibration."
            )
            self._update_calibration_compute_selection_state()
            return False
        root_widget = self.setup_widgets.get("calibration_root")
        configured_root = str(
            _evidence(self.backend).calibration_setup().get("calibration_root_resolved") or ""
        )
        needs_apply = configured_root != root
        if isinstance(root_widget, QtWidgets.QLineEdit) and root_widget.text().strip() != root:
            root_widget.setText(root)
            needs_apply = True
        self._normalize_legacy_calibration_selection(Path(root))
        if not needs_apply:
            return True
        apply_setup = getattr(self.backend, "calibration_apply_setup", None)
        if not callable(apply_setup):
            return True
        try:
            apply_setup(self.setup_payload())
        except Exception as exc:
            self.calibration_session_path_label.setText(f"Session apply failed: {exc}")
            return False
        self._last_calibration_session_apply_performed = True
        self.calibration_session_path_label.setText(root)
        self.calibration_session_result_label.setText(
            f"Results: {root}/intrinsics_zoom_##.json and intrinsics.json"
        )
        self._calibration_file_picker_signature = None
        self._calibration_diagnostics_signature = None
        self._sync_calibration_file_selection()
        self._sync_calibration_outlier_view()
        return True

    def _update_calibration_compute_selection_state(self) -> None:
        combo = getattr(self, "calibration_session_combo", None)
        current_root = str(combo.currentData() or "").strip() if combo is not None else ""
        enabled = bool(
            current_root
            and self._calibration_session_explicit_root
            and current_root == self._calibration_session_explicit_root
        )
        tooltip = "" if enabled else "Select a Photo set before running calibration."
        for command_id in ("calibration.run_intrinsic", "calibration.run_extrinsics"):
            button = self.operation_buttons.get(command_id)
            if button is not None:
                button.setEnabled(enabled)
                button.setToolTip(tooltip)

    @staticmethod
    def _normalize_legacy_calibration_selection(root: Path) -> None:
        """Retain the hook without rewriting unsupported or malformed evidence."""

        _ = root

    # -- Explicit calculation-file selection ---------------------------

    def _build_calibration_file_selection_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("계산에 사용할 사진")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)

        controls = QtWidgets.QGridLayout()
        controls.setHorizontalSpacing(8)
        controls.setVerticalSpacing(5)
        self.calibration_left_files_button = QtWidgets.QPushButton("Left 사진 선택…")
        self.calibration_right_files_button = QtWidgets.QPushButton("Right 사진 선택…")
        self.calibration_clear_files_button = QtWidgets.QPushButton("선택 초기화")
        self.calibration_left_files_button.clicked.connect(
            lambda _checked=False: self._choose_calibration_files("left")
        )
        self.calibration_right_files_button.clicked.connect(
            lambda _checked=False: self._choose_calibration_files("right")
        )
        self.calibration_clear_files_button.clicked.connect(
            self._clear_calibration_file_selection
        )
        self.calibration_file_selection_summary = QtWidgets.QLabel(
            "파일창에서 좌/우 사진을 각각 선택하세요."
        )
        self.calibration_file_selection_summary.setObjectName("statusStrip")
        self.calibration_file_selection_summary.setWordWrap(True)
        controls.addWidget(self.calibration_left_files_button, 0, 0)
        controls.addWidget(self.calibration_right_files_button, 0, 1)
        controls.addWidget(self.calibration_clear_files_button, 0, 2)
        controls.addWidget(self.calibration_file_selection_summary, 1, 0, 1, 3)
        layout.addLayout(controls)

        self.calibration_selected_files_table = QtWidgets.QTableWidget(0, 6)
        self.calibration_selected_files_table.setObjectName("calibrationSelectedFilesTable")
        self.calibration_selected_files_table.setHorizontalHeaderLabels(
            ("#", "Left", "Right", "Left 정보", "Right 정보", "Pair")
        )
        self.calibration_selected_files_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.calibration_selected_files_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.calibration_selected_files_table.verticalHeader().setVisible(False)
        self.calibration_selected_files_table.setMinimumHeight(100)
        self.calibration_selected_files_table.setMaximumHeight(230)
        header = self.calibration_selected_files_table.horizontalHeader()
        header.setStretchLastSection(True)
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QtWidgets.QHeaderView.ResizeToContents)
        layout.addWidget(self.calibration_selected_files_table)

        hint = QtWidgets.QLabel(
            "intrinsic은 좌/우 목록을 각각 사용하고, Checkerboard R,t는 같은 행의 좌/우 파일을 pair로 사용합니다. "
            "사진 정보의 시각은 exact RTCP/NTP frame → app capture record/request → legacy file mtime "
            "순으로 표시하며 출처를 함께 적습니다. 선택 목록은 세션 폴더의 "
            "calibration_pair_selection.json에 저장됩니다."
        )
        hint.setObjectName("compactContract")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self._calibration_file_picker_signature = None
        return group

    def _build_calibration_outlier_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Outlier analysis")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)
        self.calibration_outlier_summary = QtWidgets.QLabel(
            "Run calibration (K,D) 후 사진별 reprojection error가 표시됩니다."
        )
        self.calibration_outlier_summary.setObjectName("statusStrip")
        self.calibration_outlier_summary.setWordWrap(True)
        layout.addWidget(self.calibration_outlier_summary)
        self.calibration_outlier_plot = CalibrationErrorPlot()
        self.calibration_outlier_plot.exclude_requested.connect(
            lambda payload: self._handle_calibration_outlier_action(payload, restore=False)
        )
        self.calibration_outlier_plot.restore_requested.connect(
            lambda payload: self._handle_calibration_outlier_action(payload, restore=True)
        )
        layout.addWidget(self.calibration_outlier_plot)
        self.calibration_outlier_table = QtWidgets.QTableWidget(0, 6)
        self.calibration_outlier_table.setObjectName("calibrationOutlierTable")
        self.calibration_outlier_table.setHorizontalHeaderLabels(
            ("Side", "File", "Error px", "Coverage", "Board", "State")
        )
        self.calibration_outlier_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.calibration_outlier_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.calibration_outlier_table.verticalHeader().setVisible(False)
        self.calibration_outlier_table.setMinimumHeight(72)
        self.calibration_outlier_table.setMaximumHeight(190)
        header = self.calibration_outlier_table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QtWidgets.QHeaderView.ResizeToContents)
        header.setStretchLastSection(True)
        layout.addWidget(self.calibration_outlier_table)
        return group

    def _build_calibration_quality_group(self) -> QtWidgets.QGroupBox:
        """Build a presentation-only view of the owner's quality snapshot."""

        group = QtWidgets.QGroupBox("Calibration quality gate")
        group.setObjectName("calibrationQualityGroup")
        self.calibration_quality_group = group
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)

        values_layout = QtWidgets.QGridLayout()
        values_layout.setHorizontalSpacing(8)
        values_layout.setVerticalSpacing(4)
        self.calibration_quality_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("state", "Quality state"),
                ("effective_pose_count", "Effective poses"),
                ("coverage", "Center coverage"),
                ("principal_offsets", "Principal offsets"),
                ("subset_stability", "Subset cx/cy/focal stability"),
                ("uncertainty", "Solver uncertainty / max view error"),
                ("reasons", "Reasons"),
                ("source", "Source"),
            )
        ):
            value = EngineeringValue(label)
            value.setObjectName(f"calibrationQuality_{key}")
            self.calibration_quality_values[key] = value
            values_layout.addWidget(value, index // 2, index % 2)
        self.calibration_quality_state_label = self.calibration_quality_values["state"].value
        self.calibration_quality_reasons_label = self.calibration_quality_values["reasons"].value
        layout.addLayout(values_layout)

        coverage_label = QtWidgets.QLabel(
            "Board-center occupancy (source-provided 3×3 grid; no UI geometry recomputation)"
        )
        coverage_label.setObjectName("compactContract")
        coverage_label.setWordWrap(True)
        layout.addWidget(coverage_label)
        self.calibration_center_coverage_table = QtWidgets.QTableWidget(3, 3)
        self.calibration_center_coverage_table.setObjectName("calibrationCenterCoverage3x3")
        self.calibration_center_coverage_table.setHorizontalHeaderLabels(
            ("Left", "Center", "Right")
        )
        self.calibration_center_coverage_table.setVerticalHeaderLabels(
            ("Top", "Middle", "Bottom")
        )
        self.calibration_center_coverage_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.calibration_center_coverage_table.setSelectionMode(
            QtWidgets.QAbstractItemView.NoSelection
        )
        self.calibration_center_coverage_table.setMaximumHeight(126)
        self.calibration_center_coverage_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch
        )
        self.calibration_center_coverage_table.verticalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch
        )
        layout.addWidget(self.calibration_center_coverage_table)

        self.calibration_quality_warning_checkbox = QtWidgets.QCheckBox(
            "I reviewed this WARNING and explicitly allow Save / Apply for this result"
        )
        self.calibration_quality_warning_checkbox.setObjectName(
            "calibrationQualityWarningConfirmation"
        )
        self.calibration_quality_warning_checkbox.setToolTip(
            "Confirmation applies only to the currently displayed source-backed quality snapshot."
        )
        self.calibration_quality_warning_checkbox.toggled.connect(
            self.confirm_calibration_quality_warning
        )
        self.calibration_quality_warning_checkbox.setVisible(False)
        layout.addWidget(self.calibration_quality_warning_checkbox)

        self.calibration_quality_gate_label = QtWidgets.QLabel(
            "UNVERIFIED · legacy result has no quality snapshot; existing Save / Apply behavior is preserved."
        )
        self.calibration_quality_gate_label.setObjectName("statusStrip")
        self.calibration_quality_gate_label.setWordWrap(True)
        layout.addWidget(self.calibration_quality_gate_label)

        for row in range(3):
            for column in range(3):
                item = QtWidgets.QTableWidgetItem("--")
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.calibration_center_coverage_table.setItem(row, column, item)
        return group

    @staticmethod
    def _calibration_quality_state_text(value: object) -> str:
        state = str(getattr(value, "value", value) or "").strip().upper()
        aliases = {
            "GOOD": "PASS",
            "VALID": "PASS",
            "PASSED": "PASS",
            "OK": "PASS",
            "WARN": "WARNING",
            "INVALID": "FAIL",
            "FAILED": "FAIL",
            "ERROR": "FAIL",
            "UNKNOWN": "UNVERIFIED",
            "NOT_VERIFIED": "UNVERIFIED",
            "LEGACY_UNVERIFIED": "UNVERIFIED",
        }
        normalized = aliases.get(state, state)
        return normalized if normalized in {"PASS", "WARNING", "FAIL"} else "UNVERIFIED"

    @staticmethod
    def _calibration_quality_reasons(value: object) -> tuple[str, ...]:
        reasons: list[str] = []
        rows = value if isinstance(value, (list, tuple, set)) else (value,)
        for row in rows:
            if row in (None, ""):
                continue
            if isinstance(row, Mapping):
                code = str(row.get("code") or row.get("id") or "").strip()
                detail = str(
                    row.get("message")
                    or row.get("reason")
                    or row.get("detail")
                    or row.get("status")
                    or ""
                ).strip()
                text = f"{code}: {detail}" if code and detail else code or detail
            else:
                text = str(row).strip()
            if text and text not in reasons:
                reasons.append(text)
        return tuple(reasons)

    @staticmethod
    def _calibration_quality_display(value: object) -> str:
        if value in (None, "", (), [], {}):
            return "--"
        if isinstance(value, bool):
            return "yes" if value else "no"
        if isinstance(value, float):
            return _display_number(value, digits=4)
        if isinstance(value, Mapping):
            parts = []
            for key, child in value.items():
                if key in {"grid", "grid_3x3", "cells", "coverage_grid_3x3"}:
                    continue
                parts.append(
                    f"{key}={CalibrationWorkspace._calibration_quality_display(child)}"
                )
            return " · ".join(parts) or "--"
        if isinstance(value, (list, tuple)):
            return ", ".join(
                CalibrationWorkspace._calibration_quality_display(child) for child in value
            )
        return str(value)

    @staticmethod
    def _calibration_quality_mapping(value: object) -> dict[str, object]:
        if not isinstance(value, Mapping):
            return {}
        snapshot = dict(value)
        for key in ("calibration_quality", "quality_snapshot", "quality"):
            nested = snapshot.get(key)
            if isinstance(nested, Mapping) and any(
                field in nested
                for field in (
                    "state",
                    "status",
                    "verdict",
                    "quality_state",
                    "effective_pose_count",
                    "coverage_grid_3x3",
                )
            ):
                merged = dict(snapshot)
                merged.update(dict(nested))
                return merged
        return snapshot

    def calibration_quality_snapshot(self) -> dict[str, object]:
        """Read one owner-produced quality snapshot without deriving metrics."""

        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            for name in (
                "calibration_quality_snapshot",
                "workstation_calibration_quality_snapshot",
                "checkerboard_calibration_quality_snapshot",
            ):
                reader = getattr(owner, name, None)
                if not callable(reader):
                    continue
                try:
                    snapshot = reader()
                except Exception as exc:
                    return {
                        "state": "UNVERIFIED",
                        "reasons": (f"quality snapshot unavailable: {exc}",),
                        "source": f"{type(owner).__name__}.{name}",
                    }
                if isinstance(snapshot, Mapping):
                    return self._calibration_quality_mapping(snapshot)
        for owner in (panel, self.backend):
            for name in ("calibration_quality", "last_calibration_quality"):
                snapshot = getattr(owner, name, None)
                if isinstance(snapshot, Mapping):
                    return self._calibration_quality_mapping(snapshot)
        result = getattr(panel, "last_calibration_result", None)
        snapshot = getattr(result, "quality_snapshot", None) if result is not None else None
        if isinstance(snapshot, Mapping):
            return self._calibration_quality_mapping(snapshot)
        return {}

    @staticmethod
    def _calibration_quality_field(
        snapshot: Mapping[str, object],
        *names: str,
    ) -> object:
        for container in (
            snapshot,
            snapshot.get("metrics") if isinstance(snapshot.get("metrics"), Mapping) else {},
        ):
            if not isinstance(container, Mapping):
                continue
            for name in names:
                if name in container and container[name] not in (None, ""):
                    return container[name]
        return None

    @staticmethod
    def _calibration_coverage_grid(snapshot: Mapping[str, object]) -> object:
        coverage = snapshot.get("coverage")
        for value in (
            snapshot.get("coverage_grid_3x3"),
            snapshot.get("center_coverage_grid"),
            coverage.get("grid_3x3") if isinstance(coverage, Mapping) else None,
            coverage.get("grid") if isinstance(coverage, Mapping) else None,
            coverage.get("cells") if isinstance(coverage, Mapping) else None,
        ):
            if value not in (None, ""):
                return value
        return None

    @staticmethod
    def _calibration_grid_cell(grid: object, row: int, column: int) -> object:
        if isinstance(grid, (list, tuple)):
            if len(grid) == 9:
                return grid[row * 3 + column]
            if len(grid) > row and isinstance(grid[row], (list, tuple)) and len(grid[row]) > column:
                return grid[row][column]
        if isinstance(grid, Mapping):
            names = (
                ("top_left", "top_center", "top_right"),
                ("middle_left", "center", "middle_right"),
                ("bottom_left", "bottom_center", "bottom_right"),
            )
            for key in (
                names[row][column],
                f"{row},{column}",
                f"r{row}c{column}",
                str(row * 3 + column),
            ):
                if key in grid:
                    return grid[key]
        return None

    @staticmethod
    def _calibration_grid_cell_text(value: object) -> tuple[str, bool | None]:
        if value is None:
            return "--", None
        if isinstance(value, Mapping):
            covered = value.get("covered")
            display = next(
                (
                    value[key]
                    for key in ("label", "count", "pose_count", "fraction", "value")
                    if key in value
                ),
                "covered" if covered is True else "empty" if covered is False else "--",
            )
            return CalibrationWorkspace._calibration_quality_display(display), (
                bool(covered) if covered is not None else None
            )
        if isinstance(value, bool):
            return ("covered", True) if value else ("empty", False)
        return CalibrationWorkspace._calibration_quality_display(value), None

    def apply_calibration_quality_snapshot(self, value: object) -> dict[str, object]:
        """Present and gate one source snapshot; never calculate quality here."""

        snapshot = self._calibration_quality_mapping(value)
        raw_state = self._calibration_quality_field(
            snapshot, "state", "quality_state", "verdict", "status"
        )
        state = self._calibration_quality_state_text(raw_state)
        reasons = self._calibration_quality_reasons(
            self._calibration_quality_field(
                snapshot,
                "reasons",
                "quality_reasons",
                "failure_reasons",
                "warning_reasons",
            )
        )
        if not snapshot:
            reasons = ("No source-backed calibration quality snapshot (legacy result)",)
        elif state == "UNVERIFIED" and raw_state not in (None, "") and not reasons:
            reasons = (f"Unsupported source quality state: {raw_state}",)
        source = str(
            snapshot.get("source")
            or snapshot.get("producer")
            or snapshot.get("source_id")
            or "legacy/no quality source"
        )
        effective_pose_count = self._calibration_quality_field(
            snapshot,
            "effective_pose_count",
            "effective_poses",
            "usable_pose_count",
            "inlier_pose_count",
        )
        coverage = self._calibration_quality_field(
            snapshot,
            "coverage",
            "center_coverage",
            "board_center_coverage",
            "coverage_fraction",
        )
        principal_offsets = self._calibration_quality_field(
            snapshot,
            "principal_point_offsets_px",
            "principal_offsets_px",
            "principal_offsets",
        )
        if principal_offsets is None:
            principal_parts = {
                key: self._calibration_quality_field(snapshot, *aliases)
                for key, aliases in (
                    ("left", ("left_principal_offset_px", "left_principal_point_offset_px")),
                    ("right", ("right_principal_offset_px", "right_principal_point_offset_px")),
                    ("cx", ("principal_offset_cx_px", "principal_cx_offset_px")),
                    ("cy", ("principal_offset_cy_px", "principal_cy_offset_px")),
                )
            }
            principal_offsets = {
                key: child for key, child in principal_parts.items() if child is not None
            } or None
        subset_stability = self._calibration_quality_field(
            snapshot,
            "subset_stability_px",
            "subset_stability",
            "random_subset_stability",
        )
        if subset_stability is None:
            subset_parts = {
                key: self._calibration_quality_field(snapshot, *aliases)
                for key, aliases in (
                    ("cx", ("subset_cx_stability_px", "subset_cx_std_px")),
                    ("cy", ("subset_cy_stability_px", "subset_cy_std_px")),
                    (
                        "focal",
                        ("subset_focal_stability_px", "subset_focal_std_px", "subset_fx_std_px"),
                    ),
                )
            }
            subset_stability = {
                key: child for key, child in subset_parts.items() if child is not None
            } or None
        uncertainty = self._calibration_quality_field(
            snapshot,
            "uncertainty",
            "solver_uncertainty",
            "intrinsic_uncertainty",
        )
        signature_values = {
            "state": state,
            "reasons": reasons,
            "effective_pose_count": effective_pose_count,
            "coverage": coverage,
            "principal_offsets": principal_offsets,
            "subset_stability": subset_stability,
            "uncertainty": uncertainty,
            "grid": self._calibration_coverage_grid(snapshot),
            "source": source,
            "quality_id": snapshot.get(
                "quality_id", snapshot.get("result_id", snapshot.get("profile_sha256"))
            ),
        }
        signature = (
            json.dumps(signature_values, sort_keys=True, ensure_ascii=False, default=str),
        )
        if signature != self._calibration_quality_signature:
            self._calibration_quality_signature = signature
            self._calibration_quality_warning_confirmed = False
            checkbox = getattr(self, "calibration_quality_warning_checkbox", None)
            if checkbox is not None:
                checkbox.blockSignals(True)
                checkbox.setChecked(False)
                checkbox.blockSignals(False)

        self._calibration_quality_state = state
        normalized = dict(snapshot)
        normalized.update(state=state, reasons=reasons, source=source)
        self._calibration_quality_snapshot_cache = normalized
        valid = True if state == "PASS" else False if state == "FAIL" else None
        self.calibration_quality_values["state"].set_value(
            state, source=source, valid=valid
        )
        self.calibration_quality_values["effective_pose_count"].set_value(
            effective_pose_count, source=source
        )
        self.calibration_quality_values["coverage"].set_value(
            self._calibration_quality_display(coverage), source=source
        )
        self.calibration_quality_values["principal_offsets"].set_value(
            self._calibration_quality_display(principal_offsets), unit="px", source=source
        )
        self.calibration_quality_values["subset_stability"].set_value(
            self._calibration_quality_display(subset_stability), unit="px", source=source
        )
        self.calibration_quality_values["uncertainty"].set_value(
            self._calibration_quality_display(uncertainty), source=source
        )
        self.calibration_quality_values["reasons"].set_value(
            " · ".join(reasons) if reasons else "none", source=source
        )
        self.calibration_quality_values["source"].set_value(source, source=source)

        grid = self._calibration_coverage_grid(snapshot)
        for row in range(3):
            for column in range(3):
                text, covered = self._calibration_grid_cell_text(
                    self._calibration_grid_cell(grid, row, column)
                )
                item = self.calibration_center_coverage_table.item(row, column)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    self.calibration_center_coverage_table.setItem(row, column, item)
                item.setText(text)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                item.setToolTip(f"Source: {source}\nValue: {text}")
                item.setForeground(
                    QtGui.QBrush(
                        QtGui.QColor(SUCCESS_TEXT if covered is True else ERROR_TEXT)
                    )
                    if covered is not None
                    else QtGui.QBrush()
                )

        self.calibration_quality_warning_checkbox.setVisible(state == "WARNING")
        self._apply_calibration_quality_gate()
        self._sync_simple_calibration_status()
        return dict(normalized)

    def refresh_calibration_quality(self) -> dict[str, object]:
        return self.apply_calibration_quality_snapshot(self.calibration_quality_snapshot())

    def confirm_calibration_quality_warning(self, confirmed: bool = True) -> bool:
        if self._calibration_quality_state != "WARNING":
            return False
        self._calibration_quality_warning_confirmed = bool(confirmed)
        checkbox = getattr(self, "calibration_quality_warning_checkbox", None)
        if checkbox is not None and checkbox.isChecked() != bool(confirmed):
            checkbox.blockSignals(True)
            checkbox.setChecked(bool(confirmed))
            checkbox.blockSignals(False)
        self._apply_calibration_quality_gate()
        return self._calibration_quality_warning_confirmed

    def calibration_quality_action_allowed(self, command_id: str) -> bool:
        if command_id not in self._CALIBRATION_QUALITY_GATED_COMMANDS:
            return True
        if self._calibration_quality_state == "FAIL":
            return False
        if self._calibration_quality_state == "WARNING":
            return bool(self._calibration_quality_warning_confirmed)
        return True

    def _apply_calibration_quality_gate(self) -> None:
        owned = getattr(self, "_calibration_quality_gate_owned", set())
        original_tooltips = getattr(self, "_calibration_quality_original_tooltips", {})
        reasons = tuple(self._calibration_quality_snapshot_cache.get("reasons") or ())
        reason_text = " · ".join(str(reason) for reason in reasons) or "no reason supplied"
        state = self._calibration_quality_state
        for command_id in self._CALIBRATION_QUALITY_GATED_COMMANDS:
            buttons = tuple(self.operation_button_instances.get(command_id, ()))
            if not buttons:
                primary = self.operation_buttons.get(command_id)
                buttons = () if primary is None else (primary,)
            if not buttons:
                continue
            allowed = self.calibration_quality_action_allowed(command_id)
            for index, button in enumerate(buttons):
                owner_key = f"{command_id}:{index}"
                original_tooltips.setdefault(owner_key, button.toolTip())
                if not allowed:
                    if button.isEnabled():
                        owned.add(owner_key)
                    button.setEnabled(False)
                    if state == "FAIL":
                        button.setToolTip(f"Blocked by calibration quality FAIL: {reason_text}")
                    else:
                        button.setToolTip(
                            "Calibration quality WARNING requires the explicit confirmation checkbox. "
                            + reason_text
                        )
                elif owner_key in owned:
                    button.setEnabled(True)
                    owned.discard(owner_key)
                    button.setToolTip(original_tooltips.get(owner_key, ""))
        self._calibration_quality_gate_owned = owned
        self._calibration_quality_original_tooltips = original_tooltips

        if state == "PASS":
            text = "PASS · result can be saved as an approved profile and applied."
        elif state == "WARNING":
            text = (
                "WARNING acknowledged · Apply explicitly enabled; Save remains evidence-preserving."
                if self._calibration_quality_warning_confirmed
                else "WARNING · Save preserves an attempt; review reasons and confirm before Apply."
            )
        elif state == "FAIL":
            text = f"FAIL · Save preserves an attempt; Apply is blocked. {reason_text}"
        else:
            text = (
                "UNVERIFIED · legacy/no-quality result; it is not presented as PASS and "
                "existing Save / Apply behavior is preserved."
            )
        self.calibration_quality_gate_label.setText(text)
        self._sync_simple_calibration_status()

    def _calibration_diagnostic_rows(self) -> tuple[dict[str, object], ...]:
        panel = getattr(self.backend, "panel", self.backend)
        rows = getattr(panel, "calibration_image_diagnostics", ())
        if not rows:
            result = getattr(panel, "last_calibration_result", None)
            rows = getattr(result, "image_diagnostics", ()) if result is not None else ()
        return tuple(dict(row) for row in rows if isinstance(row, Mapping))

    def _calibration_exclusion_sets(self) -> tuple[set[str], set[str]]:
        selection = self._load_calibration_pair_selection()
        root_text = str(_evidence(self.backend).calibration_setup().get("calibration_root_resolved") or "")
        root = Path(root_text) if root_text else Path(DEFAULT_CALIBRATION_ROOT)
        left = {
            str(path)
            for path in self._resolve_calibration_selection_paths(
                selection.get("excluded_left_paths"),
                root,
                stored_root=selection.get("root"),
            )
        }
        right = {
            str(path)
            for path in self._resolve_calibration_selection_paths(
                selection.get("excluded_right_paths"),
                root,
                stored_root=selection.get("root"),
            )
        }
        return left, right

    def _sync_calibration_outlier_view(self) -> None:
        plot = getattr(self, "calibration_outlier_plot", None)
        table = getattr(self, "calibration_outlier_table", None)
        if plot is None or table is None:
            return
        rows = self._calibration_diagnostic_rows()
        excluded_left, excluded_right = self._calibration_exclusion_sets()
        selection = self._load_calibration_pair_selection()
        excluded_pairs = {str(key) for key in selection.get("excluded_pair_keys") or ()}
        signature = (
            tuple(tuple(sorted(row.items())) for row in rows),
            tuple(sorted(excluded_left)),
            tuple(sorted(excluded_right)),
            tuple(sorted(excluded_pairs)),
        )
        if signature == self._calibration_diagnostics_signature:
            return
        self._calibration_diagnostics_signature = signature
        excluded_paths = excluded_left | excluded_right
        plot.set_diagnostics(
            rows,
            excluded_paths=excluded_paths,
            excluded_pair_keys=excluded_pairs,
        )
        table.setRowCount(len(rows))
        threshold = plot._threshold_px
        for index, row in enumerate(rows):
            error = plot._finite_error(row)
            path = Path(str(row.get("path") or ""))
            excluded = str(path) in excluded_paths or str(row.get("capture_key") or "") in excluded_pairs
            state = "EXCLUDED" if excluded else "candidate" if threshold is not None and error is not None and error >= threshold else "included"
            coverage = row.get("board_coverage_fraction")
            coverage_text = "--" if coverage is None else f"{float(coverage) * 100.0:.1f}%"
            values = (
                str(row.get("side") or "").upper(),
                path.name or str(row.get("path") or "--"),
                "--" if error is None else f"{error:.3f}",
                coverage_text,
                str(row.get("capture_key") or "--"),
                state,
            )
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setToolTip(str(row.get("path") or value))
                if state == "EXCLUDED":
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#888888")))
                elif state == "candidate":
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#c0392b")))
                table.setItem(index, column, item)
        if not rows:
            self.calibration_outlier_summary.setText(
                "계산 결과가 없습니다. Run calibration (K,D)를 먼저 실행하세요."
            )
        else:
            valid = sum(1 for row in rows if plot._finite_error(row) is not None)
            excluded = len(excluded_paths)
            candidate_count = sum(
                1
                for row in rows
                if plot._finite_error(row) is not None
                and plot._finite_error(row) >= (threshold or float("inf"))
            )
            self.calibration_outlier_summary.setText(
                f"진단 {len(rows)}장 · 유효 잔차 {valid}장 · 자동 후보 {candidate_count}장 · "
                f"제외 {excluded}장 | 점 우클릭으로 제외/복원 후 다시 계산하세요."
            )

    def _handle_calibration_outlier_action(self, payload: object, *, restore: bool) -> None:
        if not isinstance(payload, Mapping):
            return
        path_text = str(payload.get("path") or "").strip()
        side = str(payload.get("side") or "").strip().lower()
        if side not in {"left", "right"} or not path_text:
            return
        root_text = str(_evidence(self.backend).calibration_setup().get("calibration_root_resolved") or "")
        root = (
            Path(root_text) if root_text else Path(DEFAULT_CALIBRATION_ROOT)
        ).expanduser().resolve(strict=False)
        path = Path(path_text).expanduser().resolve()
        selection = self._load_calibration_pair_selection()
        excluded_left_paths, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"),
            root,
            stored_root=selection.get("root"),
        )
        excluded_right_paths, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"),
            root,
            stored_root=selection.get("root"),
        )
        selected_left_paths, selected_left_errors = resolve_calibration_selection_paths(
            selection.get("left_paths"),
            root,
            stored_root=selection.get("root"),
        )
        selected_right_paths, selected_right_errors = resolve_calibration_selection_paths(
            selection.get("right_paths"),
            root,
            stored_root=selection.get("root"),
        )
        path_errors = (
            *excluded_left_errors,
            *excluded_right_errors,
            *selected_left_errors,
            *selected_right_errors,
        )
        if path_errors:
            self.calibration_outlier_summary.setText(
                "기존 선택/제외 경로를 복구할 수 없어 변경하지 않았습니다: "
                + ", ".join(path_errors)
            )
            return
        excluded_left = {str(item) for item in excluded_left_paths}
        excluded_right = {str(item) for item in excluded_right_paths}
        excluded_pairs = {
            str(item) for item in selection.get("excluded_pair_keys") or ()
        }
        capture_key = str(payload.get("capture_key") or "").strip()
        pair_left_paths = {str(path)} if side == "left" else set()
        pair_right_paths = {str(path)} if side == "right" else set()
        if capture_key:
            for row in self._calibration_diagnostic_rows():
                if (
                    str(row.get("capture_key") or "") == capture_key
                    and str(row.get("path") or "")
                    and str(row.get("path")) != str(path)
                ):
                    counterpart = str(Path(str(row["path"])).expanduser().resolve())
                    if str(row.get("side") or "").lower() == "left":
                        pair_left_paths.add(counterpart)
                    elif str(row.get("side") or "").lower() == "right":
                        pair_right_paths.add(counterpart)
        scope = str(payload.get("scope") or "image")
        if restore:
            if scope == "pair":
                excluded_pairs.discard(capture_key)
                excluded_left.difference_update(pair_left_paths)
                excluded_right.difference_update(pair_right_paths)
            elif side == "left":
                excluded_left.discard(str(path))
            else:
                excluded_right.discard(str(path))
            action_text = "복원"
        else:
            if scope == "pair":
                excluded_pairs.add(capture_key)
                excluded_left.update(pair_left_paths)
                excluded_right.update(pair_right_paths)
                action_text = "stereo pair 제외"
            elif side == "left":
                excluded_left.add(str(path))
                action_text = "Left 사진 제외"
            else:
                excluded_right.add(str(path))
                action_text = "Right 사진 제외"
        data = {
            "schema": CALIBRATION_SELECTION_SCHEMA,
            "root": ".",
            "manual_mode": bool(selection.get("manual_mode")),
            "left_paths": portable_calibration_selection_paths(
                root, selected_left_paths
            ),
            "right_paths": portable_calibration_selection_paths(
                root, selected_right_paths
            ),
            "included_keys": list(selection.get("included_keys") or ()),
            "excluded_left_paths": portable_calibration_selection_paths(
                root, [Path(item) for item in sorted(excluded_left)]
            ),
            "excluded_right_paths": portable_calibration_selection_paths(
                root, [Path(item) for item in sorted(excluded_right)]
            ),
            "excluded_pair_keys": sorted(key for key in excluded_pairs if key),
        }
        try:
            sidecar = root / "calibration_pair_selection.json"
            sidecar.parent.mkdir(parents=True, exist_ok=True)
            sidecar.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        except OSError as exc:
            self.calibration_outlier_summary.setText(f"제외 상태 저장 실패: {exc}")
            return
        self._calibration_diagnostics_signature = None
        self._calibration_file_picker_signature = None
        self._sync_calibration_file_selection()
        self._sync_calibration_outlier_view()
        self.calibration_outlier_summary.setText(
            f"{action_text}: {Path(path_text).name} | 원본 파일은 삭제하지 않았습니다. 다시 계산하면 반영됩니다."
        )

    @staticmethod
    def _calibration_file_root(path: Path) -> Path:
        resolved = path.expanduser().resolve()
        if resolved.parent.name.lower() in {"left", "right"}:
            return resolved.parent.parent
        return resolved.parent

    @staticmethod
    def _calibration_file_key(path: Path, side: str) -> str:
        prefix = f"{side}_"
        return path.stem.removeprefix(prefix) if path.stem.startswith(prefix) else path.stem

    def _resolve_calibration_selection_paths(
        self,
        raw_paths: object,
        root: Path,
        *,
        stored_root: object = None,
    ) -> list[Path]:
        paths, _errors = resolve_calibration_selection_paths(
            raw_paths,
            root,
            stored_root=stored_root,
        )
        return paths

    def _calibration_file_selection_for_root(self, root: Path) -> tuple[list[Path], list[Path], bool]:
        selection_path = root / "calibration_pair_selection.json"
        data = load_calibration_selection_sidecar(selection_path).to_mapping()
        if str(data.get("_sidecar_state") or "") == "INVALID":
            self._calibration_file_selection_path_errors = tuple(
                str(value) for value in data.get("_sidecar_errors") or ()
            )
            return [], [], True
        manual = bool(data.get("manual_mode"))
        stored_root = data.get("root")
        left, left_errors = resolve_calibration_selection_paths(
            data.get("left_paths"), root, stored_root=stored_root
        )
        right, right_errors = resolve_calibration_selection_paths(
            data.get("right_paths"), root, stored_root=stored_root
        )
        self._calibration_file_selection_path_errors = (*left_errors, *right_errors)
        return left, right, manual

    def _choose_calibration_files(self, side: str) -> None:
        if side not in {"left", "right"}:
            return
        root_text = str(_evidence(self.backend).calibration_setup().get("calibration_root_resolved") or "")
        root = Path(root_text) if root_text else Path(DEFAULT_CALIBRATION_ROOT)
        current_left, current_right, manual = self._calibration_file_selection_for_root(root)
        current = current_left if side == "left" and manual else current_right if side == "right" and manual else []
        start = str(current[0].parent if current else root / side)
        files, _selected_filter = QtWidgets.QFileDialog.getOpenFileNames(
            self,
            f"{side.title()} calibration images 선택",
            start,
            "Calibration images (*.png *.jpg *.jpeg *.tif *.tiff *.bmp);;All files (*)",
        )
        if not files:
            return
        selected = [Path(path).expanduser().resolve() for path in files if Path(path).is_file()]
        if not selected:
            return
        other = current_right if side == "left" and manual else current_left if side == "right" and manual else []
        candidate_paths = [*selected, *other]
        roots = {self._calibration_file_root(path) for path in candidate_paths}
        if len(roots) > 1:
            self.calibration_file_selection_summary.setText(
                "좌/우 사진은 같은 calibration session 폴더에서 선택해야 합니다."
            )
            return
        selected_root = next(iter(roots), root).resolve()
        left = selected if side == "left" else other
        right = selected if side == "right" else other
        root_widget = self.setup_widgets.get("calibration_root")
        if isinstance(root_widget, QtWidgets.QLineEdit):
            root_widget.setText(str(selected_root))
        apply_setup = getattr(self.backend, "calibration_apply_setup", None)
        if callable(apply_setup):
            try:
                apply_setup(self.setup_payload())
            except Exception as exc:
                self.calibration_file_selection_summary.setText(f"사진 root 적용 실패: {exc}")
                return
        self._save_calibration_file_selection(selected_root, left, right)
        self._calibration_session_explicit_root = str(selected_root)
        self._refresh_calibration_session_selector(force=True)
        self._calibration_file_picker_signature = None
        self._sync_calibration_file_selection()

    def _save_calibration_file_selection(
        self,
        root: Path,
        left: Sequence[Path],
        right: Sequence[Path],
    ) -> None:
        root = Path(root).expanduser().resolve(strict=False)
        left_absolute = [Path(path).expanduser().resolve(strict=False) for path in left]
        right_absolute = [Path(path).expanduser().resolve(strict=False) for path in right]
        left_paths = portable_calibration_selection_paths(root, left_absolute)
        right_paths = portable_calibration_selection_paths(root, right_absolute)
        common_keys = [
            self._calibration_file_key(left_path, "left")
            for left_path in left_absolute
            if any(
                self._calibration_file_key(right_path, "right")
                == self._calibration_file_key(left_path, "left")
                for right_path in right_absolute
            )
        ]
        previous = self._load_calibration_pair_selection()
        previous_left, left_errors = resolve_calibration_selection_paths(
            previous.get("excluded_left_paths"),
            root,
            stored_root=previous.get("root"),
        )
        previous_right, right_errors = resolve_calibration_selection_paths(
            previous.get("excluded_right_paths"),
            root,
            stored_root=previous.get("root"),
        )
        if left_errors or right_errors:
            self.calibration_file_selection_summary.setText(
                "기존 제외 경로를 복구할 수 없어 선택 목록을 덮어쓰지 않았습니다: "
                + ", ".join((*left_errors, *right_errors))
            )
            return
        selected_left = set(left_absolute)
        selected_right = set(right_absolute)
        excluded_left = sorted(path for path in previous_left if path in selected_left)
        excluded_right = sorted(path for path in previous_right if path in selected_right)
        payload = {
            "schema": CALIBRATION_SELECTION_SCHEMA,
            "root": ".",
            "manual_mode": bool(left_paths or right_paths),
            "left_paths": left_paths,
            "right_paths": right_paths,
            "included_keys": sorted(set(common_keys)),
            "excluded_left_paths": portable_calibration_selection_paths(root, excluded_left),
            "excluded_right_paths": portable_calibration_selection_paths(root, excluded_right),
            "excluded_pair_keys": list(previous.get("excluded_pair_keys") or ()),
        }
        path = root / "calibration_pair_selection.json"
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        except OSError as exc:
            self.calibration_file_selection_summary.setText(f"선택 목록 저장 실패: {exc}")

    def _clear_calibration_file_selection(self) -> None:
        root_text = str(_evidence(self.backend).calibration_setup().get("calibration_root_resolved") or "")
        root = Path(root_text) if root_text else Path(DEFAULT_CALIBRATION_ROOT)
        try:
            path = root / "calibration_pair_selection.json"
            data = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
            if not isinstance(data, dict):
                data = {}
            data.update(
                {
                    "schema": CALIBRATION_SELECTION_SCHEMA,
                    "root": ".",
                    "manual_mode": False,
                    "left_paths": [],
                    "right_paths": [],
                    "included_keys": [],
                    "excluded_left_paths": [],
                    "excluded_right_paths": [],
                    "excluded_pair_keys": [],
                }
            )
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        except (OSError, ValueError, TypeError) as exc:
            self.calibration_file_selection_summary.setText(f"선택 초기화 실패: {exc}")
            return
        self._calibration_file_picker_signature = None
        self._sync_calibration_file_selection()

    @classmethod
    def _calibration_capture_metadata_path(cls, path: Path) -> Path:
        side = path.parent.name.lower()
        key = cls._calibration_file_key(path, side)
        return cls._calibration_file_root(path) / "capture_metadata" / f"capture_{key}.json"

    @classmethod
    def _calibration_file_capture_time(
        cls,
        path: Path,
    ) -> tuple[int | None, str, str]:
        """Resolve one photo's wall-clock time and its independent provenance.

        Camera RTCP/NTP frame time is the strongest source because it belongs
        to the exact left or right image. Pair-level application record/request
        time is retained as a fallback; legacy external files use mtime and are
        labelled as such instead of being presented as a camera timestamp.
        """

        metadata_path = cls._calibration_capture_metadata_path(path)
        try:
            payload = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            payload = {}
        if isinstance(payload, Mapping):
            side = path.parent.name.lower()
            capture_origin = str(payload.get("capture_origin") or "").strip().lower()
            nested = payload.get("frame_grid_by_side")
            side_payload = (
                nested.get(side, {})
                if isinstance(nested, Mapping) and isinstance(nested.get(side), Mapping)
                else {}
            )
            # FTP/OZ metadata historically copied the contemporaneous live
            # preview RTCP time, not the downloaded still's exposure time.
            # Never label that value as the selected FTP photo's timestamp.
            rtcp_is_bound_to_saved_frame = capture_origin in {"manual", "auto"} or (
                not capture_origin and bool(side_payload.get("native_lossless_source"))
            )
            if rtcp_is_bound_to_saved_frame:
                for container, field in (
                    (side_payload, "rtcp_frame_ntp_unix_s"),
                    (payload, f"{side}_rtcp_frame_ntp_unix_s"),
                ):
                    try:
                        seconds = float(container.get(field))
                    except (AttributeError, TypeError, ValueError):
                        continue
                    if math.isfinite(seconds) and seconds > 0.0:
                        whole_seconds = int(seconds)
                        fractional_ns = int(
                            round((seconds - whole_seconds) * 1_000_000_000)
                        )
                        if fractional_ns >= 1_000_000_000:
                            whole_seconds += 1
                            fractional_ns -= 1_000_000_000
                        return (
                            whole_seconds * 1_000_000_000 + fractional_ns,
                            "RTCP/NTP frame",
                            str(metadata_path),
                        )
            for field, source in (
                ("capture_recorded_unix_ns", "app capture record"),
                ("capture_requested_unix_ns", "capture request"),
            ):
                try:
                    unix_ns = int(payload.get(field))
                except (TypeError, ValueError):
                    continue
                if unix_ns > 0:
                    return unix_ns, source, str(metadata_path)
        try:
            return int(path.stat().st_mtime_ns), "file mtime", str(path)
        except OSError:
            return None, "unavailable", str(path)

    @staticmethod
    def _format_calibration_file_capture_time(unix_ns: int | None, source: str) -> str:
        if unix_ns is None or unix_ns <= 0:
            return "시각 없음 · unavailable"
        seconds, nanoseconds = divmod(int(unix_ns), 1_000_000_000)
        try:
            local_time = time.localtime(seconds)
        except (OverflowError, OSError, ValueError):
            return f"시각 해석 불가 ({unix_ns} ns) · {source}"
        timezone_name = time.strftime("%Z", local_time).strip()
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", local_time)
        timestamp += f".{nanoseconds // 1_000_000:03d}"
        if timezone_name:
            timestamp += f" {timezone_name}"
        return f"{timestamp} · {source}"

    @classmethod
    def _calibration_file_info(cls, path: Path) -> str:
        try:
            size_kib = path.stat().st_size / 1024.0
        except OSError:
            size_kib = 0.0
        size = QtGui.QImageReader(str(path)).size()
        dimensions = f"{size.width()}×{size.height()}" if size.isValid() else "?×?"
        unix_ns, source, _evidence_path = cls._calibration_file_capture_time(path)
        captured_at = cls._format_calibration_file_capture_time(unix_ns, source)
        return f"{dimensions} · {size_kib:.1f} KiB\n{captured_at}"

    @classmethod
    def _calibration_file_tooltip(cls, path: Path) -> str:
        unix_ns, source, evidence_path = cls._calibration_file_capture_time(path)
        captured_at = cls._format_calibration_file_capture_time(unix_ns, source)
        return (
            f"{path}\n"
            f"사진 시각: {captured_at}\n"
            f"시각 근거: {evidence_path}"
        )

    @classmethod
    def _calibration_file_signature(cls, path: Path) -> tuple[object, ...]:
        try:
            image_stat = path.stat()
            image_stamp = (int(image_stat.st_mtime_ns), int(image_stat.st_size))
        except OSError:
            image_stamp = (0, 0)
        metadata_path = cls._calibration_capture_metadata_path(path)
        try:
            metadata_stat = metadata_path.stat()
            metadata_stamp = (int(metadata_stat.st_mtime_ns), int(metadata_stat.st_size))
        except OSError:
            metadata_stamp = (0, 0)
        return str(path), *image_stamp, str(metadata_path), *metadata_stamp

    def _sync_calibration_file_selection(self) -> None:
        table = getattr(self, "calibration_selected_files_table", None)
        if table is None:
            return
        if not self._calibration_session_explicit_root:
            table.setRowCount(0)
            self._calibration_file_picker_signature = None
            self.calibration_file_selection_summary.setText(
                "Photo set을 선택하면 계산 후보가 표시됩니다."
            )
            return
        root = Path(self._calibration_session_explicit_root)
        left, right, manual = self._calibration_file_selection_for_root(root)
        selection = self._load_calibration_pair_selection()
        excluded_left_paths, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"),
            root,
            stored_root=selection.get("root"),
        )
        excluded_right_paths, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"),
            root,
            stored_root=selection.get("root"),
        )
        path_errors = (
            *getattr(self, "_calibration_file_selection_path_errors", ()),
            *excluded_left_errors,
            *excluded_right_errors,
        )
        if path_errors:
            table.setRowCount(0)
            self._calibration_file_picker_signature = None
            self.calibration_file_selection_summary.setText(
                "선택/제외 경로 복구 실패 · 전체 후보 fallback 차단: "
                + ", ".join(path_errors)
            )
            return
        excluded_left = {str(path) for path in excluded_left_paths}
        excluded_right = {str(path) for path in excluded_right_paths}
        excluded_pair_keys = {str(key) for key in selection.get("excluded_pair_keys") or ()}
        if not manual:
            # Before the operator opens a file dialog, show the available
            # matching session pairs as a review-only preview. The controller
            # retains its legacy even-spread selection contract until explicit
            # files are saved here.
            left_candidates = sorted(
                path
                for path in (root / "left").iterdir()
                if path.is_file() and path.suffix.lower() in CALIBRATION_IMAGE_SUFFIXES
            ) if (root / "left").is_dir() else []
            right_candidates = sorted(
                path
                for path in (root / "right").iterdir()
                if path.is_file() and path.suffix.lower() in CALIBRATION_IMAGE_SUFFIXES
            ) if (root / "right").is_dir() else []
            left_by_key = {
                self._calibration_file_key(path, "left"): path
                for path in left_candidates
            }
            right_by_key = {
                self._calibration_file_key(path, "right"): path
                for path in right_candidates
            }
            keys = sorted(left_by_key.keys() & right_by_key.keys())
            if keys:
                left = [left_by_key[key] for key in keys]
                right = [right_by_key[key] for key in keys]
            else:
                left = left_candidates
                right = right_candidates
        signature = (
            str(root),
            manual,
            tuple(self._calibration_file_signature(path) for path in left),
            tuple(self._calibration_file_signature(path) for path in right),
        )
        if signature == getattr(self, "_calibration_file_picker_signature", None):
            return
        self._calibration_file_picker_signature = signature
        table.setRowCount(0)
        row_count = max(len(left), len(right))
        table.setRowCount(row_count)
        for index in range(row_count):
            left_path = left[index] if index < len(left) else None
            right_path = right[index] if index < len(right) else None
            left_key = self._calibration_file_key(left_path, "left") if left_path is not None else ""
            right_key = self._calibration_file_key(right_path, "right") if right_path is not None else ""
            is_excluded = (
                (left_path is not None and str(left_path) in excluded_left)
                or (right_path is not None and str(right_path) in excluded_right)
                or (left_key and left_key == right_key and left_key in excluded_pair_keys)
            )
            pair_usage = (
                load_calibration_pair_usage(
                    root / "capture_metadata" / f"capture_{left_key}.json"
                )
                if left_key and left_key == right_key
                else None
            )
            if is_excluded:
                usage_state = "EXCLUDED"
            elif pair_usage is not None and not pair_usage.stereo_eligible:
                usage_state = (
                    "INTRINSICS ONLY"
                    if pair_usage.state == INTRINSICS_ONLY
                    else "INVALID METADATA"
                )
            elif pair_usage is not None and pair_usage.state == LEGACY_UNVERIFIED:
                usage_state = "matched · timing unverified"
            elif left_path is not None and right_path is not None and left_key == right_key:
                usage_state = "matched"
            elif left_path is not None and right_path is not None:
                usage_state = "same row"
            else:
                usage_state = "incomplete"
            values = (
                str(index + 1),
                left_path.name if left_path is not None else "--",
                right_path.name if right_path is not None else "--",
                self._calibration_file_info(left_path) if left_path is not None else "--",
                self._calibration_file_info(right_path) if right_path is not None else "--",
                usage_state,
            )
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                tooltip_path = (
                    left_path
                    if column in {1, 3}
                    else right_path
                    if column in {2, 4}
                    else None
                )
                if tooltip_path is not None:
                    tooltip = self._calibration_file_tooltip(tooltip_path)
                elif column == 5 and pair_usage is not None:
                    usage_contract = (
                        "K,D input: allowed\nStereo R,t: excluded"
                        if not pair_usage.stereo_eligible
                        else "K,D input: allowed\nStereo R,t: allowed"
                    )
                    tooltip = f"{pair_usage.detail}\n{usage_contract}"
                else:
                    tooltip = value
                item.setToolTip(tooltip)
                table.setItem(index, column, item)
        table.resizeRowsToContents()
        if manual:
            pair_count = min(len(left), len(right))
            self.calibration_file_selection_summary.setText(
                f"지정 파일: left={len(left)}, right={len(right)}, 계산 행 pair={pair_count}"
            )
        else:
            self.calibration_file_selection_summary.setText(
                f"자동 후보 미리보기: left={len(left)}, right={len(right)} · 파일창에서 고정 선택 가능"
            )

    # -- Manual calculation-pair picker --------------------------------
    #
    # ``_select_calibration_pairs`` on the hidden panel auto-samples an
    # even spread of pairs by count. This picker lets the operator instead
    # check exactly which left/right pairs go into the next calculation --
    # useful once WIRIS FTP OZ sync has pulled in more pairs than are
    # actually good (blur, board partly out of frame, etc). Selection state
    # is handed to the hidden panel via a JSON sidecar
    # (``calibration_pair_selection.json`` next to the images) rather than a
    # cross-panel command, since it needs to be read at calculation time by
    # ``CalibrationWorkflowController._choose_calibration_pairs``.

    def _build_calibration_pair_picker_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("계산용 사진 선택")
        layout = QtWidgets.QVBoxLayout(group)
        self.calibration_manual_selection_checkbox = QtWidgets.QCheckBox("수동 선택 사용")
        self.calibration_manual_selection_checkbox.setToolTip(
            "끄면 기존처럼 균등 분산 자동 선택을 사용합니다. 켜면 아래에서 체크한 쌍만 계산에 사용합니다."
        )
        self.calibration_manual_selection_checkbox.toggled.connect(
            self._on_calibration_pair_selection_changed
        )
        layout.addWidget(self.calibration_manual_selection_checkbox)
        filter_row = QtWidgets.QHBoxLayout()
        self.calibration_show_all_pairs_checkbox = QtWidgets.QCheckBox("전체 사진 보기")
        self.calibration_show_all_pairs_checkbox.setToolTip(
            "기본값은 Workstation 실행 이후 저장된 pair만 표시합니다. "
            "켜면 calibration root의 기존 pair까지 모두 표시합니다."
        )
        self.calibration_show_all_pairs_checkbox.toggled.connect(
            self._handle_calibration_pair_picker_filter_changed
        )
        filter_row.addWidget(self.calibration_show_all_pairs_checkbox)
        self.calibration_pair_picker_summary = QtWidgets.QLabel("앱 실행 후 캡처 대기")
        self.calibration_pair_picker_summary.setObjectName("statusStrip")
        filter_row.addWidget(self.calibration_pair_picker_summary, 1)
        layout.addLayout(filter_row)
        self.calibration_pair_picker_list = QtWidgets.QListWidget()
        self.calibration_pair_picker_list.setObjectName("calibrationPairPickerList")
        self.calibration_pair_picker_list.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.calibration_pair_picker_list.setIconSize(QtCore.QSize(64, 36))
        self.calibration_pair_picker_list.setMinimumHeight(90)
        self.calibration_pair_picker_list.setMaximumHeight(220)
        self.calibration_pair_picker_list.setToolTip(
            "체크된 쌍만(수동 선택 사용 시) Run calibration / Calculate R,t 에 사용됩니다."
        )
        self.calibration_pair_picker_list.itemChanged.connect(
            self._on_calibration_pair_selection_changed
        )
        layout.addWidget(self.calibration_pair_picker_list)
        self._calibration_pair_picker_root: Path | None = None
        return group

    def _calibration_pair_selection_path(self) -> Path | None:
        root_text = str(_evidence(self.backend).calibration_setup().get("calibration_root_resolved") or "")
        if not root_text:
            return None
        return Path(root_text) / "calibration_pair_selection.json"

    def _load_calibration_pair_selection(self) -> dict[str, object]:
        path = self._calibration_pair_selection_path()
        if path is None:
            return {
                **empty_calibration_selection(),
                "_sidecar_state": "ABSENT",
                "_sidecar_errors": [],
            }
        return load_calibration_selection_sidecar(path).to_mapping()

    def _on_calibration_pair_selection_changed(self, *_args: object) -> None:
        path = self._calibration_pair_selection_path()
        if path is None:
            return
        selection = self._load_calibration_pair_selection()
        included = {str(key) for key in selection.get("included_keys") or ()}
        visible_keys = set(getattr(self, "_calibration_pair_picker_visible_keys", set()))
        for index in range(self.calibration_pair_picker_list.count()):
            item = self.calibration_pair_picker_list.item(index)
            key = str(item.data(QtCore.Qt.UserRole))
            if key not in visible_keys:
                continue
            if item.checkState() == QtCore.Qt.Checked:
                included.add(key)
            else:
                included.discard(key)
        payload = {
            "schema": CALIBRATION_SELECTION_SCHEMA,
            "root": ".",
            "manual_mode": bool(self.calibration_manual_selection_checkbox.isChecked()),
            "included_keys": sorted(included),
            "left_paths": list(selection.get("left_paths") or ()),
            "right_paths": list(selection.get("right_paths") or ()),
            "excluded_left_paths": list(selection.get("excluded_left_paths") or ()),
            "excluded_right_paths": list(selection.get("excluded_right_paths") or ()),
            "excluded_pair_keys": list(selection.get("excluded_pair_keys") or ()),
        }
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except OSError:
            pass

    def _handle_calibration_pair_picker_filter_changed(self, _checked: bool) -> None:
        self._calibration_pair_picker_signature = None
        self._sync_calibration_pair_picker()

    def _sync_calibration_pair_picker(self) -> None:
        picker = getattr(self, "calibration_pair_picker_list", None)
        if picker is None:
            return
        root_text = str(_evidence(self.backend).calibration_setup().get("calibration_root_resolved") or "")
        if not root_text:
            return
        root = Path(root_text)
        left_dir, right_dir = root / "left", root / "right"

        def path_stamp(path: Path) -> tuple[int, int] | None:
            try:
                stat = path.stat()
            except OSError:
                return None
            return int(stat.st_mtime_ns), int(stat.st_size)

        def directory_stamp(path: Path, prefix: str) -> tuple[object, ...] | None:
            try:
                stat = path.stat()
                names = tuple(
                    sorted(
                        entry.name
                        for entry in path.iterdir()
                        if entry.is_file() and entry.name.startswith(prefix) and entry.suffix.lower() == ".png"
                    )
                )
            except OSError:
                return None
            # Directory mtime catches rewrites; names catch filesystems whose
            # directory mtime granularity is too coarse for rapid captures.
            return int(stat.st_mtime_ns), int(stat.st_size), names

        def metadata_directory_stamp(path: Path) -> tuple[object, ...] | None:
            try:
                stat = path.stat()
                entries = tuple(
                    sorted(
                        (entry.name, int(entry.stat().st_mtime_ns), int(entry.stat().st_size))
                        for entry in path.iterdir()
                        if entry.is_file()
                        and entry.name.startswith("capture_")
                        and entry.suffix.lower() == ".json"
                    )
                )
            except OSError:
                return None
            return int(stat.st_mtime_ns), int(stat.st_size), entries

        selection_path = root / "calibration_pair_selection.json"
        show_all_widget = getattr(self, "calibration_show_all_pairs_checkbox", None)
        show_all = bool(show_all_widget.isChecked()) if show_all_widget is not None else True
        session_started_ns = int(
            getattr(self, "_calibration_pair_picker_session_started_unix_ns", 0) or 0
        )
        signature = (
            str(root),
            directory_stamp(left_dir, "left_"),
            directory_stamp(right_dir, "right_"),
            metadata_directory_stamp(root / "capture_metadata"),
            path_stamp(selection_path),
            show_all,
            session_started_ns,
        )
        if signature == self._calibration_pair_picker_signature:
            return

        left_keys = {path.stem.removeprefix("left_") for path in left_dir.glob("left_*.png")} if left_dir.is_dir() else set()
        right_keys = {path.stem.removeprefix("right_") for path in right_dir.glob("right_*.png")} if right_dir.is_dir() else set()
        all_keys = sorted(left_keys & right_keys)
        metadata_dir = root / "capture_metadata"

        def pair_capture_time_ns(key: str) -> int:
            metadata_path = metadata_dir / f"capture_{key}.json"
            try:
                payload = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                payload = {}
            if isinstance(payload, dict):
                for field in ("capture_recorded_unix_ns", "capture_requested_unix_ns"):
                    try:
                        value = int(payload.get(field))
                    except (TypeError, ValueError):
                        continue
                    if value > 0:
                        return value
            try:
                return int((left_dir / f"left_{key}.png").stat().st_mtime_ns)
            except OSError:
                return 0

        if show_all or session_started_ns <= 0:
            keys = all_keys
        else:
            keys = [key for key in all_keys if pair_capture_time_ns(key) >= session_started_ns]
        self._calibration_pair_picker_visible_keys = set(keys)
        summary_label = getattr(self, "calibration_pair_picker_summary", None)
        if summary_label is not None:
            scope = "전체" if show_all else "앱 실행 후"
            summary_label.setText(f"{scope} 표시: {len(keys)} / 전체 {len(all_keys)} pair")
        selection = self._load_calibration_pair_selection()
        included = set(selection.get("included_keys") or ())
        picker.blockSignals(True)
        self.calibration_manual_selection_checkbox.blockSignals(True)
        try:
            self.calibration_manual_selection_checkbox.setChecked(bool(selection.get("manual_mode")))
            picker.clear()
            self._calibration_pair_picker_items = {}
            thumbnail_paths: list[tuple[str, Path]] = []
            for key in keys:
                origin = "manual"
                metadata_path = metadata_dir / f"capture_{key}.json"
                pair_usage = load_calibration_pair_usage(metadata_path)
                if metadata_path.exists():
                    try:
                        metadata_payload = json.loads(
                            metadata_path.read_text(encoding="utf-8")
                        )
                        if isinstance(metadata_payload, Mapping):
                            origin = str(metadata_payload.get("capture_origin", "manual"))
                    except (OSError, ValueError, TypeError):
                        pass
                left_path = left_dir / f"left_{key}.png"
                right_path = right_dir / f"right_{key}.png"
                left_ns, left_source, _left_evidence = self._calibration_file_capture_time(
                    left_path
                )
                right_ns, right_source, _right_evidence = self._calibration_file_capture_time(
                    right_path
                )
                left_time = self._format_calibration_file_capture_time(left_ns, left_source)
                right_time = self._format_calibration_file_capture_time(right_ns, right_source)
                usage_suffix = (
                    " · INTRINSICS ONLY"
                    if pair_usage.state == INTRINSICS_ONLY
                    else " · INVALID METADATA"
                    if pair_usage.state == INVALID_METADATA
                    else " · LEGACY TIMING UNVERIFIED"
                    if pair_usage.state == LEGACY_UNVERIFIED
                    else ""
                )
                item = QtWidgets.QListWidgetItem(
                    f"{key} · {origin}{usage_suffix} · L {left_time} · R {right_time}"
                )
                usage_contract = (
                    "K,D input: allowed\nStereo R,t: excluded"
                    if not pair_usage.stereo_eligible
                    else "K,D input: allowed\nStereo R,t: allowed"
                )
                item.setToolTip(
                    "Left\n"
                    + self._calibration_file_tooltip(left_path)
                    + "\n\nRight\n"
                    + self._calibration_file_tooltip(right_path)
                    + f"\n\nPair usage\n{pair_usage.detail}\n{usage_contract}"
                )
                item.setData(QtCore.Qt.UserRole, key)
                if pair_usage.stereo_eligible:
                    item.setFlags(
                        item.flags()
                        | QtCore.Qt.ItemIsUserCheckable
                        | QtCore.Qt.ItemIsEnabled
                    )
                    item.setCheckState(
                        QtCore.Qt.Checked
                        if not included or key in included
                        else QtCore.Qt.Unchecked
                    )
                else:
                    item.setCheckState(QtCore.Qt.Unchecked)
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsUserCheckable)
                picker.addItem(item)
                self._calibration_pair_picker_items[key] = item
                thumbnail_paths.append((key, left_path))
        finally:
            picker.blockSignals(False)
            self.calibration_manual_selection_checkbox.blockSignals(False)
        self._calibration_pair_picker_signature = signature
        self._calibration_pair_picker_thumbnail_generation += 1
        self._calibration_pair_picker_thumbnail_paths = thumbnail_paths
        self._calibration_pair_picker_thumbnail_index = 0
        generation = self._calibration_pair_picker_thumbnail_generation
        QtCore.QTimer.singleShot(0, lambda generation=generation: self._load_next_calibration_pair_thumbnail(generation))

    def _load_next_calibration_pair_thumbnail(self, generation: int) -> None:
        """Load one picker thumbnail between normal UI event-loop turns."""

        if generation != self._calibration_pair_picker_thumbnail_generation:
            return
        index = self._calibration_pair_picker_thumbnail_index
        paths = self._calibration_pair_picker_thumbnail_paths
        if index >= len(paths):
            return
        key, left_path = paths[index]
        self._calibration_pair_picker_thumbnail_index = index + 1
        item = self._calibration_pair_picker_items.get(key)
        if item is not None:
            try:
                thumbnail = QtGui.QPixmap(str(left_path))
                if not thumbnail.isNull():
                    item.setIcon(QtGui.QIcon(thumbnail.scaledToHeight(36, QtCore.Qt.SmoothTransformation)))
            except RuntimeError:
                # A refresh may have replaced the QListWidget while a
                # previously queued thumbnail callback was still pending.
                return
        QtCore.QTimer.singleShot(
            0,
            lambda generation=generation: self._load_next_calibration_pair_thumbnail(generation),
        )

    def _build_checkerboard_validation_group(self) -> QtWidgets.QGroupBox:
        """Recompute K,D (and R,t) from random subsets of the scanned pairs
        and report how much the result varies, as a check on calibration
        stability -- distinct from the main Capture/Compute/Load & Apply
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
            "capturing and computing a fresh result in Compute first."
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
            "Requires a reference result first: load one above, or run a normal compute in "
            "Compute. Recomputes K,D (and R,t in Checkerboard mode) from several random "
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

        self.checkerboard_validation_save_button = QtWidgets.QPushButton("Save result")
        self.checkerboard_validation_save_button.setProperty(
            "commandId", "calibration.validation.save"
        )
        self.checkerboard_validation_save_button.clicked.connect(
            self._save_checkerboard_validation_result
        )
        self.checkerboard_validation_apply_button = QtWidgets.QPushButton("Apply result")
        self.checkerboard_validation_apply_button.setProperty(
            "commandId", "calibration.validation.apply"
        )
        self.checkerboard_validation_apply_button.clicked.connect(
            self._apply_checkerboard_validation_result
        )
        self.operation_buttons["calibration.validation.save"] = (
            self.checkerboard_validation_save_button
        )
        self.operation_buttons["calibration.validation.apply"] = (
            self.checkerboard_validation_apply_button
        )
        grid.addWidget(self.checkerboard_validation_save_button, row, 0)
        grid.addWidget(self.checkerboard_validation_apply_button, row, 1)

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
        quality = snapshot.get("calibration_quality")
        if isinstance(quality, Mapping):
            self.apply_calibration_quality_snapshot(quality)
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
        self._apply_checkerboard_validation_snapshot(
            method(
                quality_warning_confirmed=bool(
                    self._calibration_quality_warning_confirmed
                )
            )
        )

    def _apply_checkerboard_validation_result(self) -> None:
        method = getattr(self.backend, "apply_workstation_checkerboard_validation_result", None)
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(
            method(
                quality_warning_confirmed=bool(
                    self._calibration_quality_warning_confirmed
                )
            )
        )

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
        for previews in (
            self.previews,
            getattr(self, "simple_calibration_previews", {}),
        ):
            for preview in previews.values():
                preview.clear()
        if hasattr(self, "_simple_calibration_detection_text"):
            self._simple_calibration_detection_text = {
                "left": "보드: 프레임 없음",
                "right": "보드: 프레임 없음",
            }
            for side in ("left", "right"):
                self._set_simple_calibration_preview_meta(side)

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
            simple = getattr(self, "simple_calibration_previews", {}).get(side)
            if simple is not None:
                simple.set_frame(image, metadata)
                self._set_simple_calibration_preview_meta(side)

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
            simple = getattr(self, "simple_calibration_previews", {}).get(camera)
            if simple is not None:
                simple.set_frame(image, metadata)
                self._set_simple_calibration_preview_meta(camera)
            self._sync_simple_calibration_status()
        elif not self._checkerboard_overlay_active[camera]:
            self.previews[camera].set_frame(image, metadata)
            simple = getattr(self, "simple_calibration_previews", {}).get(camera)
            if simple is not None:
                simple.set_frame(image, metadata)
                self._set_simple_calibration_preview_meta(camera)

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
            for previews in (
                self.previews,
                getattr(self, "simple_calibration_previews", {}),
            ):
                for preview in previews.values():
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
        # The visible Photo set is the calculation input root. Keep the hidden
        # owner-side validation widget in the same transaction so Run cannot
        # scan a stale directory that differs from the table the operator saw.
        if "calibration_root" in payload:
            payload["validation_root"] = payload["calibration_root"]
        return payload

    def calibration_input_request(self) -> dict[str, object]:
        """Return the visible calculation authority, not hidden owner widgets."""

        root = str(
            getattr(self, "_calibration_session_explicit_root", "") or ""
        ).strip()
        if not root:
            raise RuntimeError("Select a Photo set before running calibration")
        setup = self.setup_payload()
        return {
            "root": root,
            "mode": str(setup.get("validation_mode") or "pixhawk"),
            "pair_limit": int(setup.get("calculation_pair_count") or 0),
        }

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        camera = snapshot.get("camera.left")
        preview_text = (
            "Stop preview" if camera and camera.state in {"LIVE", "STARTING"} else "Preview"
        )
        self.camera_buttons["camera.toggle"].setText(preview_text)
        for button in self.simple_calibration_buttons.get("camera.toggle", ()):
            button.setText(preview_text)
        panel = getattr(self.backend, "panel", self.backend)
        calibration_controller = getattr(panel, "_calibration_workflow_controller_owner", None)
        auto_capture_active = bool(getattr(calibration_controller, "_auto_capture_active", False))
        auto_capture_button = self.operation_buttons.get("calibration.auto_capture")
        if auto_capture_button is not None:
            auto_capture_button.setText("Stop auto capture" if auto_capture_active else "Start auto capture")
        for button in self.simple_calibration_buttons.get("calibration.auto_capture", ()):
            button.setText("Stop auto capture" if auto_capture_active else "Start auto capture")
        recording = snapshot.get("recording")
        recording_locked = bool(recording and recording.state in {"ACTIVE", "FINALIZING"})
        self.camera_buttons["camera.toggle"].setEnabled(not recording_locked)
        for button in self.simple_calibration_buttons.get("camera.toggle", ()):
            button.setEnabled(not recording_locked)
        self._refresh_calibration_session_selector()
        self._sync_calibration_file_selection()
        self._sync_calibration_outlier_view()
        self.refresh_calibration_quality()
        # Keep the old in-memory pair picker path available for compatibility
        # with legacy embedded callers and tests; the visible Workstation UI
        # now uses explicit left/right file lists above.
        if hasattr(self, "calibration_pair_picker_list"):
            self._sync_calibration_pair_picker()
        data = _evidence(self.backend).calibration()
        self.zoom_readback_label.setText(
            f"capture zoom={data.get('zoom', '--')} | "
            f"actual={data.get('zoom_readback', 'left=unknown, right=unknown')} | "
            "capture metadata is required"
        )
        for key, value in self.profile_values.items():
            value.set_value(data.get(key), source=str(data.get("source", "")))
        self._sync_simple_calibration_status(data.get("capture_status"))
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
        left_data = data.get("left")
        left_data = left_data if isinstance(left_data, Mapping) else {}
        right_data = data.get("right")
        right_data = right_data if isinstance(right_data, Mapping) else {}
        common_intrinsics = str(data.get("profile_path") or "--")
        left_intrinsics = str(left_data.get("profile_path") or common_intrinsics)
        right_intrinsics = str(right_data.get("profile_path") or common_intrinsics)
        extrinsics_profile = str(data.get("extrinsics_profile_path") or "--")
        self.extrinsics_bundle_contract_label.setText(
            f"Input bundle · Left K,D={left_intrinsics} · "
            f"Right K,D={right_intrinsics} · R,t={extrinsics_profile} · "
            f"zoom={data.get('zoom', '--')} · "
            "X_right = R_left_to_right X_left + T_left_to_right_m "
            "[OpenCV optical RDF]"
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
        self._media_video_signature: tuple[tuple[str, str], ...] = ()
        self._preview_images: dict[str, QtGui.QImage] = {}

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
        self._refresh_media_video_paths()

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
        self.zoom_level_spin.setRange(0, 100)
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
        self.px4_camera_combo = self._combo(
            (
                ("Pixhawk 1 / Left", "left"),
                ("Pixhawk 2 / Right", "right"),
                ("Drone Pixhawk / sysid 3", "drone"),
            )
        )
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
        manual_eth_button = QtWidgets.QPushButton("Apply persistent Ethernet MAV_2")
        manual_eth_button.clicked.connect(self._apply_manual_px4_ethernet_setup)
        px4_layout.addWidget(manual_eth_button, 5, 0, 1, 2)
        self.px4_status_label = QtWidgets.QLabel(
            "MAG: disarmed + PARAM readback · attitude calibration: QGC Sensor Setup"
        )
        self.px4_status_label.setObjectName("statusStrip")
        self.px4_status_label.setWordWrap(True)
        px4_layout.addWidget(self.px4_status_label, 6, 0, 1, 5)
        layout.addWidget(px4_group)

        transfer_group = QtWidgets.QGroupBox("MAVFTP file / log / firmware staging")
        transfer_layout = QtWidgets.QGridLayout(transfer_group)
        self.px4_transfer_local_edit = QtWidgets.QLineEdit()
        self.px4_transfer_remote_edit = QtWidgets.QLineEdit("/fs/microsd/workstation/files/upload.bin")
        self.px4_flash_usb_edit = QtWidgets.QLineEdit()
        self.px4_flash_usb_edit.setPlaceholderText("Direct Pixhawk USB, e.g. /dev/ttyACM0")
        self.px4_transfer_status_label = QtWidgets.QLabel("IDLE")
        self.px4_transfer_status_label.setObjectName("statusStrip")
        self.px4_transfer_status_label.setWordWrap(True)
        browse_button = QtWidgets.QPushButton("Browse")
        browse_button.clicked.connect(self._browse_px4_transfer_file)
        restart_button = QtWidgets.QPushButton("Restart upload")
        restart_button.clicked.connect(lambda: self._start_px4_transfer(resume=False, firmware=False))
        resume_button = QtWidgets.QPushButton("Resume verified partial")
        resume_button.clicked.connect(lambda: self._start_px4_transfer(resume=True, firmware=False))
        download_button = QtWidgets.QPushButton("Restart download")
        download_button.clicked.connect(lambda: self._start_px4_download(resume=False))
        resume_download_button = QtWidgets.QPushButton("Resume download")
        resume_download_button.clicked.connect(lambda: self._start_px4_download(resume=True))
        stage_button = QtWidgets.QPushButton("Stage firmware")
        stage_button.clicked.connect(lambda: self._start_px4_transfer(resume=False, firmware=True))
        cancel_button = QtWidgets.QPushButton("Cancel")
        cancel_button.clicked.connect(self._cancel_px4_transfer)
        flash_button = QtWidgets.QPushButton("Flash selected local firmware by USB")
        flash_button.clicked.connect(self._start_px4_usb_flash)
        cancel_flash_button = QtWidgets.QPushButton("Cancel USB flash")
        cancel_flash_button.clicked.connect(self._cancel_px4_usb_flash)
        transfer_layout.addWidget(QtWidgets.QLabel("Local"), 0, 0)
        transfer_layout.addWidget(self.px4_transfer_local_edit, 0, 1, 1, 3)
        transfer_layout.addWidget(browse_button, 0, 4)
        transfer_layout.addWidget(QtWidgets.QLabel("Remote"), 1, 0)
        transfer_layout.addWidget(self.px4_transfer_remote_edit, 1, 1, 1, 4)
        transfer_layout.addWidget(restart_button, 2, 0)
        transfer_layout.addWidget(resume_button, 2, 1)
        transfer_layout.addWidget(stage_button, 2, 2)
        transfer_layout.addWidget(cancel_button, 2, 3)
        transfer_layout.addWidget(download_button, 2, 4)
        transfer_layout.addWidget(resume_download_button, 2, 5)
        transfer_layout.addWidget(QtWidgets.QLabel("Direct USB"), 3, 0)
        transfer_layout.addWidget(self.px4_flash_usb_edit, 3, 1, 1, 2)
        transfer_layout.addWidget(flash_button, 3, 3)
        transfer_layout.addWidget(cancel_flash_button, 3, 4)
        transfer_layout.addWidget(self.px4_transfer_status_label, 4, 0, 1, 6)
        layout.addWidget(transfer_group)

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
        tools_group = QtWidgets.QGroupBox("Manual recording and files")
        tools_layout = QtWidgets.QHBoxLayout(tools_group)
        tools_layout.setContentsMargins(6, 6, 6, 6)
        manual_record_button = self._button("Manual record", "recording.toggle")
        manual_record_button.setToolTip(
            "Advanced fallback for recording outside an automatic Measurement run."
        )
        tools_layout.addWidget(manual_record_button)
        tools_layout.addStretch(1)
        layout.addWidget(tools_group)

        video_group = QtWidgets.QGroupBox("Video playback")
        video_layout = QtWidgets.QHBoxLayout(video_group)
        video_layout.setContentsMargins(6, 6, 6, 6)
        self.media_video_combo = QtWidgets.QComboBox()
        video_layout.addWidget(self.media_video_combo, 1)
        video_layout.addWidget(self._button("Play", "advanced.media.open_video"))
        layout.addWidget(video_group)
        self.media_status_label = QtWidgets.QLabel(
            "Current data and storage health are available in Data; legacy recordings open in the file manager."
        )
        self.media_status_label.setObjectName("statusStrip")
        self.media_status_label.setWordWrap(True)
        layout.addWidget(self.media_status_label)
        layout.addStretch(1)
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

    def _browse_px4_transfer_file(self) -> None:
        path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select Pixhawk transfer file",
            "",
            "PX4 firmware (*.px4);;ULog (*.ulg);;All files (*)",
        )
        if path:
            self.px4_transfer_local_edit.setText(path)

    def _apply_manual_px4_ethernet_setup(self) -> None:
        apply_setup = getattr(self.backend, "apply_manual_pixhawk_ethernet_setup", None)
        if callable(apply_setup):
            try:
                apply_setup(self._selected_px4_side())
            except Exception as exc:
                self.px4_status_label.setText(f"Persistent Ethernet setup failed: {exc}")

    def _start_px4_transfer(self, *, resume: bool, firmware: bool) -> None:
        local_path = self.px4_transfer_local_edit.text().strip()
        if not local_path:
            self.px4_transfer_status_label.setText("Local file is required")
            return
        remote_path = self.px4_transfer_remote_edit.text().strip()
        if firmware:
            remote_path = f"/fs/microsd/workstation/staged/{Path(local_path).name}"
            self.px4_transfer_remote_edit.setText(remote_path)
        uploader = getattr(self.backend, "pixhawk_mavftp_upload", None)
        if not callable(uploader):
            self.px4_transfer_status_label.setText("MAVFTP backend unavailable")
            return
        try:
            uploader(
                self._selected_px4_side(),
                local_path,
                remote_path,
                resume=bool(resume),
                category="firmware_stage" if firmware else "file_upload",
            )
        except Exception as exc:
            self.px4_transfer_status_label.setText(f"Transfer start failed: {exc}")

    def _cancel_px4_transfer(self) -> None:
        cancel = getattr(self.backend, "pixhawk_mavftp_cancel", None)
        if callable(cancel):
            cancel(self._selected_px4_side())

    def _start_px4_download(self, *, resume: bool) -> None:
        download = getattr(self.backend, "pixhawk_mavftp_download", None)
        if not callable(download):
            self.px4_transfer_status_label.setText("MAVFTP download backend unavailable")
            return
        try:
            download(
                self._selected_px4_side(),
                self.px4_transfer_remote_edit.text().strip(),
                self.px4_transfer_local_edit.text().strip(),
                resume=bool(resume),
            )
        except Exception as exc:
            self.px4_transfer_status_label.setText(f"Download start failed: {exc}")

    def _start_px4_usb_flash(self) -> None:
        flash = getattr(self.backend, "pixhawk_usb_flash", None)
        if not callable(flash):
            self.px4_transfer_status_label.setText("USB flash backend unavailable")
            return
        try:
            flash(
                self._selected_px4_side(),
                self.px4_flash_usb_edit.text().strip(),
                self.px4_transfer_local_edit.text().strip(),
            )
        except Exception as exc:
            self.px4_transfer_status_label.setText(f"USB flash start failed: {exc}")

    def _cancel_px4_usb_flash(self) -> None:
        cancel = getattr(self.backend, "pixhawk_usb_flash_cancel", None)
        if callable(cancel):
            cancel()

    def apply_baseline_snapshot(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        self.baseline_mavlink_model.removeRows(0, self.baseline_mavlink_model.rowCount())
        for row in payload.get("mavlink_messages", ()) or ():
            if isinstance(row, (tuple, list)):
                _append_table_row(self.baseline_mavlink_model, tuple(row))
        self.baseline_status_label.setText(str(payload.get("axis_contract") or "Baseline fields updated."))

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

    def media_open_video(self) -> bool:
        return self._open_path(self._selected_media_video_path(), label="Video")

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        self._refresh_custom_watch_selector()
        self._refresh_media_video_paths()
        recording = snapshot.get("recording")
        panel = getattr(self.backend, "panel", self.backend)
        automatic = bool(getattr(panel, "_automatic_session_recording", False))
        transfer_reader = getattr(self.backend, "pixhawk_transfer_snapshot", None)
        try:
            transfers = dict(transfer_reader()) if callable(transfer_reader) else {}
        except Exception:
            transfers = {}
        transfer = transfers.get(self._selected_px4_side(), {})
        if isinstance(transfer, Mapping) and transfer:
            total = int(transfer.get("total") or transfer.get("local_size") or 0)
            sent = int(transfer.get("sent") or transfer.get("remote_size") or 0)
            self.px4_transfer_status_label.setText(
                f"{transfer.get('state', 'UNKNOWN')} · {sent}/{total} bytes · "
                f"remote={transfer.get('remote_path', '--')}"
                + (f" · {transfer.get('error')}" if transfer.get("error") else "")
            )
        flash_status = getattr(panel, "pixhawk_usb_flash_status", {}) or {}
        if isinstance(flash_status, Mapping) and str(flash_status.get("state") or "IDLE") != "IDLE":
            self.px4_transfer_status_label.setText(
                f"USB FLASH {flash_status.get('state')} · role={flash_status.get('camera', '--')}"
                + (f" · {flash_status.get('detail')}" if flash_status.get("detail") else "")
                + (f" · {flash_status.get('error')}" if flash_status.get("error") else "")
            )
        manual_active = bool(
            recording and recording.state == "ACTIVE" and not automatic
        )
        button = self.operation_buttons.get("recording.toggle")
        if button is not None:
            button.setText(
                "Stop manual recording" if manual_active else "Manual record"
            )
            button.setEnabled(
                not automatic
                and (
                    manual_active
                    or recording is None
                    or recording.state in {"IDLE", "SAVED"}
                )
            )


class RecordingWorkspace(QtWidgets.QWidget):
    """Sessionless data status/capture tools with legacy UI fallback."""

    hr_command_requested = QtCore.pyqtSignal(str, object)
    ulog_command_requested = QtCore.pyqtSignal(str, object)
    inspection_requested = QtCore.pyqtSignal(object)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_advanced: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
        *,
        open_measurement: Callable[[], None] | None = None,
        begin_session_action: Callable[[], object] | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.invoke_command = invoke_command
        panel = getattr(self.backend, "panel", self.backend)
        self._sessionless = not bool(
            getattr(panel, "_measurement_session_hierarchy_enabled", True)
        )
        self.session_records: dict[str, dict[str, object]] = {}
        self._session_catalog_signature: tuple[object, ...] = ()
        self._last_session_scan_monotonic_s = float("-inf")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        self.header = WorkspaceHeader(
            "Data" if self._sessionless else "Sessions",
            (
                "Type-owned append files, storage health, and capture tools"
                if self._sessionless
                else "Current session, evidence completeness, and saved review"
            ),
        )
        layout.addWidget(self.header)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setObjectName("sessionsTabs")
        self._tab_ids = ("current", "evidence", "review")
        tab_layouts: dict[str, QtWidgets.QVBoxLayout] = {}
        for tab_id, title in zip(
            self._tab_ids,
            ("Current", "Evidence", "Review"),
        ):
            page = QtWidgets.QWidget()
            page.setObjectName(f"sessions{title}Tab")
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(6, 6, 6, 6)
            page_layout.setSpacing(6)
            tab_layouts[tab_id] = page_layout
            self.tabs.addTab(page, title)
        layout.addWidget(self.tabs, 1)
        self._tab_layouts = tab_layouts

        self.current_session = SessionWorkspace(
            backend,
            status_model,
            open_measurement,
            lambda: self.select_tab("review"),
            begin_session_action=begin_session_action,
            embedded=True,
        )
        self.current_session.inspection_requested.connect(self.inspection_requested.emit)
        tab_layouts["current"].addWidget(self.current_session)
        if self._sessionless:
            self.current_session.hide()

        measurements_group = QtWidgets.QGroupBox("Measurements")
        measurements_layout = QtWidgets.QVBoxLayout(measurements_group)
        measurements_layout.setContentsMargins(6, 6, 6, 6)
        self.measurements_model = _table_model(
            self,
            ("#", "Measurement", "State", "Evidence", "Revision"),
        )
        self.measurements_table = QtWidgets.QTableView()
        self.measurements_table.setObjectName("sessionsMeasurementTable")
        self.measurements_table.setModel(self.measurements_model)
        self.measurements_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.measurements_table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )
        self.measurements_table.verticalHeader().setVisible(False)
        self.measurements_table.horizontalHeader().setStretchLastSection(True)
        self.measurements_table.setMaximumHeight(180)
        measurements_layout.addWidget(self.measurements_table)
        self.measurements_group = measurements_group
        tab_layouts["current"].addWidget(measurements_group)
        if self._sessionless:
            measurements_group.hide()

        package_group = QtWidgets.QGroupBox("Evidence package")
        package_group.setObjectName("measurementPackageDetails")
        package_layout = QtWidgets.QVBoxLayout(package_group)
        package_layout.setContentsMargins(6, 6, 6, 6)
        package_layout.setSpacing(5)
        package_summary_layout = QtWidgets.QGridLayout()
        package_summary_layout.setContentsMargins(0, 0, 0, 0)
        self.package_state_label = QtWidgets.QLabel("State: IDLE")
        self.package_state_label.setObjectName("measurementPackageDetailsState")
        self.package_completeness_label = QtWidgets.QLabel("Evidence: PENDING")
        self.package_completeness_label.setObjectName("measurementPackageDetailsCompleteness")
        self.package_verification_label = QtWidgets.QLabel("Verification: NOT RUN")
        self.package_verification_label.setObjectName("measurementPackageDetailsVerification")
        self.package_revision_label = QtWidgets.QLabel("Revision: --")
        self.package_revision_label.setObjectName("measurementPackageDetailsRevision")
        for index, label in enumerate(
            (
                self.package_state_label,
                self.package_completeness_label,
                self.package_verification_label,
                self.package_revision_label,
            )
        ):
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            package_summary_layout.addWidget(label, index // 2, index % 2)
        package_layout.addLayout(package_summary_layout)

        self.package_source_model = _table_model(
            self,
            ("Source", "Expected", "State / active range", "Detail"),
        )
        self.package_source_table = QtWidgets.QTableView()
        self.package_source_table.setObjectName("measurementPackageSourceExpectations")
        self.package_source_table.setModel(self.package_source_model)
        self.package_source_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.package_source_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.package_source_table.verticalHeader().setVisible(False)
        self.package_source_table.horizontalHeader().setStretchLastSection(True)
        self.package_source_table.setMaximumHeight(125)
        package_layout.addWidget(self.package_source_table)

        self.package_artifact_model = _table_model(
            self,
            (
                "Artifact",
                "Role",
                "Required",
                "State",
                "Bytes",
                "SHA-256",
                "Revision",
                "Missing / failed reason",
            ),
        )
        self.package_artifact_table = QtWidgets.QTableView()
        self.package_artifact_table.setObjectName("measurementPackageRequiredArtifacts")
        self.package_artifact_table.setModel(self.package_artifact_model)
        self.package_artifact_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.package_artifact_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.package_artifact_table.verticalHeader().setVisible(False)
        self.package_artifact_table.horizontalHeader().setStretchLastSection(True)
        self.package_artifact_table.setMaximumHeight(170)
        package_layout.addWidget(self.package_artifact_table)

        package_actions = QtWidgets.QHBoxLayout()
        self.verify_package_button = QtWidgets.QPushButton("Verify package")
        self.verify_package_button.setObjectName("measurementPackageVerify")
        self.verify_package_button.setProperty("commandId", "session.verify_package")
        self.verify_package_button.clicked.connect(self._verify_measurement_package)
        package_actions.addWidget(self.verify_package_button)
        self.create_revision_button = QtWidgets.QPushButton("Create Revision")
        self.create_revision_button.setObjectName("measurementPackageCreateRevision")
        self.create_revision_button.setProperty("commandId", "session.create_revision")
        self.create_revision_button.setToolTip(
            "Create a new append-only revision; the sealed revision remains unchanged."
        )
        self.create_revision_button.clicked.connect(self._begin_measurement_revision)
        package_actions.addWidget(self.create_revision_button)
        self.package_action_status_label = QtWidgets.QLabel("Package actions: ready")
        self.package_action_status_label.setObjectName("measurementPackageActionStatus")
        self.package_action_status_label.setWordWrap(True)
        package_actions.addWidget(self.package_action_status_label, 1)
        package_layout.addLayout(package_actions)
        self.package_group = package_group
        tab_layouts["evidence"].addWidget(package_group)
        if self._sessionless:
            package_group.hide()

        run_group = QtWidgets.QGroupBox("Target reference")
        run_layout = QtWidgets.QGridLayout(run_group)
        run_layout.setColumnStretch(1, 1)
        run_layout.setColumnStretch(3, 1)
        self.run_platform_combo = QtWidgets.QComboBox()
        self.run_platform_combo.addItem("Drone", "drone")
        self.run_platform_combo.addItem("UAM", "uam")
        self.run_platform_combo.setCurrentIndex(
            self.run_platform_combo.findData("uam")
        )
        self.run_platform_combo.currentIndexChanged.connect(
            lambda _index: self._refresh_run_reference_label()
        )
        run_layout.addWidget(QtWidgets.QLabel("Target platform"), 0, 0)
        run_layout.addWidget(self.run_platform_combo, 0, 1)
        self.target_endpoint_edit = QtWidgets.QLineEdit()
        self.target_endpoint_edit.setObjectName("targetDroneEndpoint")
        self.target_endpoint_edit.setPlaceholderText("udp:0.0.0.0:14570")
        self.target_system_id_spin = QtWidgets.QSpinBox()
        self.target_system_id_spin.setRange(0, 255)
        self.target_system_id_spin.setSpecialValueText("Any")
        self.target_component_id_spin = QtWidgets.QSpinBox()
        self.target_component_id_spin.setRange(0, 255)
        self.target_component_id_spin.setSpecialValueText("Any")
        run_layout.addWidget(QtWidgets.QLabel("Target MAVLink"), 1, 0)
        run_layout.addWidget(self.target_endpoint_edit, 1, 1)
        target_ids = QtWidgets.QHBoxLayout()
        target_ids.setContentsMargins(0, 0, 0, 0)
        target_ids.addWidget(QtWidgets.QLabel("SYS"))
        target_ids.addWidget(self.target_system_id_spin)
        target_ids.addWidget(QtWidgets.QLabel("COMP"))
        target_ids.addWidget(self.target_component_id_spin)
        run_layout.addLayout(target_ids, 1, 2)
        target_buttons = QtWidgets.QHBoxLayout()
        target_buttons.setContentsMargins(0, 0, 0, 0)
        self.target_connect_button = QtWidgets.QPushButton("Connect target")
        self.target_connect_button.clicked.connect(self._connect_target_drone)
        self.target_stop_button = QtWidgets.QPushButton("Stop target")
        self.target_stop_button.clicked.connect(self._stop_target_drone)
        target_buttons.addWidget(self.target_connect_button)
        target_buttons.addWidget(self.target_stop_button)
        run_layout.addLayout(target_buttons, 1, 3)
        self.run_reference_label = QtWidgets.QLabel()
        self.run_reference_label.setObjectName("compactContract")
        self.run_reference_label.setWordWrap(True)
        run_layout.addWidget(self.run_reference_label, 2, 0, 1, 4)
        self.target_stream_state_label = QtWidgets.QLabel("Target Drone: NOT CONFIGURED")
        self.target_stream_state_label.setObjectName("compactContract")
        self.target_stream_state_label.setWordWrap(True)
        run_layout.addWidget(self.target_stream_state_label, 3, 0, 1, 4)
        self.target_reference_group = run_group
        tab_layouts["current"].addWidget(run_group)
        self._refresh_run_reference_label()

        status_group = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QGridLayout(status_group)
        self.indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (
                ("recording", "Recording"),
                ("measurement", "Measurement"),
                ("camera.left", "CAM L"),
                ("camera.right", "CAM R"),
                ("fps", "FPS"),
            )
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator, index // 2, index % 2)
        self.status_group = status_group
        tab_layouts["current"].addWidget(status_group)

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
        self.preflight_group = protocol_group

        session_group = QtWidgets.QGroupBox("Capture evidence")
        session_layout = QtWidgets.QGridLayout(session_group)
        self.session_values: dict[str, EngineeringValue] = {}
        capture_fields = (
            (
                ("state", "Storage"),
                ("session_id", "Recording mode"),
                ("output_dir", "Data root"),
                ("elapsed_s", "Elapsed"),
                ("status_label", "Writer status"),
            )
            if self._sessionless
            else (
                ("state", "State"),
                ("session_id", "Session ID"),
                ("output_dir", "Output"),
                ("elapsed_s", "Elapsed"),
                ("status_label", "Finalization"),
            )
        )
        for index, (key, label) in enumerate(capture_fields):
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
        self.pnp_capture_button = QtWidgets.QPushButton("Capture HR Still")
        self.pnp_capture_button.setProperty("commandId", "recording.hr.capture")
        self.pnp_capture_button.setToolTip(
            "Sends SIHR TRUE, CPTR, and ICPT to both cameras. Photos remain in camera "
            "internal storage. The chosen Pre/Post/Checkpoint purpose is recorded with "
            "the capture event; image import is a later, explicit evidence action."
        )
        self.pnp_capture_button.clicked.connect(self._capture_pnp_hr_pair)
        hr_capture_controls.addWidget(self.pnp_capture_button)
        self.oz_capture_button = QtWidgets.QPushButton("Capture OZ Still")
        self.oz_capture_button.setProperty("commandId", "recording.oz.capture")
        self.oz_capture_button.setToolTip(
            "Sends SIVI TRUE, CPTR, and ICPT to both cameras for a separate visible/OZ "
            "still. It is stored on the camera independently from the recorded RTSP video."
        )
        self.oz_capture_button.clicked.connect(self._capture_pnp_oz_pair)
        hr_capture_controls.addWidget(self.oz_capture_button)
        hr_capture_controls.addStretch(1)
        session_layout.addLayout(hr_capture_controls, 3, 0, 1, 2)
        self.pnp_capture_status_label = QtWidgets.QLabel(
            "Capture HR Still: camera-internal storage · select event, then import exact left/right files"
        )
        self.pnp_capture_status_label.setObjectName("compactContract")
        self.pnp_capture_status_label.setWordWrap(True)
        session_layout.addWidget(self.pnp_capture_status_label, 4, 0, 1, 2)
        self.oz_capture_status_label = QtWidgets.QLabel(
            "Capture OZ Still: camera-internal FHD still · separate from recorded RTSP video"
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
        self.capture_evidence_group = session_group
        tab_layouts["evidence"].addWidget(session_group)

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
        tab_layouts["evidence"].addWidget(ulog_group)

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
        self.artifact_model = _table_model(
            self,
            (
                (
                    "Type",
                    "JSONL path",
                    "JSONL state",
                    "CSV path",
                    "CSV state",
                    "JSONL drop",
                    "CSV drop",
                    "Last write",
                    "Error",
                )
                if self._sessionless
                else ("File", "Path", "Kind")
            ),
        )
        self.artifact_table = QtWidgets.QTableView()
        self.artifact_table.setModel(self.artifact_model)
        self.artifact_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.artifact_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.artifact_table.verticalHeader().setVisible(False)
        self.artifact_table.setMaximumHeight(150)
        self.artifact_table.horizontalHeader().setStretchLastSection(True)
        data_layout.addWidget(self.artifact_table)
        if self._sessionless:
            data_actions = QtWidgets.QGridLayout()
            data_actions.setContentsMargins(0, 0, 0, 0)
            data_actions.setHorizontalSpacing(6)
            data_actions.setVerticalSpacing(4)
            self.open_data_folder_button = QtWidgets.QPushButton("Open Data Folder")
            self.open_data_folder_button.setObjectName("dataWorkspaceOpenRoot")
            self.open_data_folder_button.clicked.connect(self._open_data_folder)
            self.open_legacy_recordings_button = QtWidgets.QPushButton(
                "Open Legacy Recordings Folder"
            )
            self.open_legacy_recordings_button.setObjectName("dataWorkspaceOpenLegacy")
            self.open_legacy_recordings_button.clicked.connect(
                self._open_legacy_recordings_folder
            )
            self.retry_storage_button = QtWidgets.QPushButton("Retry Storage")
            self.retry_storage_button.setObjectName("dataWorkspaceRetryStorage")
            self.retry_storage_button.clicked.connect(self._retry_data_storage)
            self.update_latest_xlsx_button = QtWidgets.QPushButton(
                "Update Latest XLSX"
            )
            self.update_latest_xlsx_button.setObjectName(
                "dataWorkspaceUpdateLatestXlsx"
            )
            self.update_latest_xlsx_button.setAccessibleName(
                "Update latest Measurement XLSX"
            )
            self.update_latest_xlsx_button.setToolTip(
                "Generate a timestamped Measurement XLSX from the most recent "
                "saved Measurement. Timestamped JSONL remains authoritative."
            )
            self.update_latest_xlsx_button.clicked.connect(
                self._request_latest_measurement_xlsx
            )
            self.open_latest_xlsx_button = QtWidgets.QPushButton(
                "Open Latest XLSX"
            )
            self.open_latest_xlsx_button.setObjectName(
                "dataWorkspaceOpenLatestXlsx"
            )
            self.open_latest_xlsx_button.setAccessibleName(
                "Open latest Measurement XLSX"
            )
            self.open_latest_xlsx_button.setToolTip(
                "Open the successfully published timestamped Measurement XLSX with "
                "the system spreadsheet application."
            )
            self.open_latest_xlsx_button.clicked.connect(
                self._open_latest_measurement_xlsx
            )
            data_actions.addWidget(self.open_data_folder_button, 0, 0)
            data_actions.addWidget(self.open_legacy_recordings_button, 0, 1)
            data_actions.addWidget(self.retry_storage_button, 0, 2)
            data_actions.addWidget(self.update_latest_xlsx_button, 1, 0)
            data_actions.addWidget(self.open_latest_xlsx_button, 1, 1)
            data_actions.setColumnStretch(3, 1)
            data_layout.addLayout(data_actions)
            self.latest_xlsx_status_label = QtWidgets.QLabel(
                "Latest XLSX · NOT CREATED"
            )
            self.latest_xlsx_status_label.setObjectName("statusStrip")
            self.latest_xlsx_status_label.setTextInteractionFlags(
                QtCore.Qt.TextSelectableByMouse
            )
            self.latest_xlsx_status_label.setWordWrap(True)
            status_height = (
                self.latest_xlsx_status_label.fontMetrics().lineSpacing() * 2 + 8
            )
            self.latest_xlsx_status_label.setMinimumHeight(status_height)
            self.latest_xlsx_status_label.setMaximumHeight(status_height)
            data_layout.addWidget(self.latest_xlsx_status_label)
        tab_layouts["current"].addWidget(data_group)

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
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        tab_layouts["review"].addWidget(review_group)

        self.contract_label = QtWidgets.QLabel(
            "Time: per-record UTC/source clock" if self._sessionless else "Time: recording clock"
        )
        self.contract_label.setToolTip(
            "Every record retains its own UTC, monotonic, and source-clock evidence."
            if self._sessionless
            else "Session IDs use wall time; samples retain the recording time axis."
        )
        self.contract_label.setObjectName("compactContract")
        tab_layouts["evidence"].addWidget(self.contract_label)
        button = QtWidgets.QPushButton("Advanced files")
        button.setToolTip(
            "Open playback, export, and media review tools."
        )
        button.clicked.connect(open_advanced)
        tab_layouts["review"].addWidget(button, alignment=QtCore.Qt.AlignRight)
        for tab_layout in tab_layouts.values():
            tab_layout.addStretch(1)
        self.tabs.currentChanged.connect(self._inspect_current_tab)
        if self._sessionless:
            review_index = self.tabs.indexOf(review_group.parentWidget())
            if review_index >= 0:
                self.tabs.removeTab(review_index)
            self._tab_ids = ("current", "evidence")
            self.tabs.setTabText(0, "Status")
            self.tabs.setTabText(1, "Capture")
            self._refresh_data_storage()
        else:
            self.refresh_sessions()

    def _data_storage_snapshot(self) -> dict[str, object]:
        reader = getattr(self.backend, "data_storage_snapshot", None)
        if not callable(reader):
            reader = getattr(
                getattr(self.backend, "panel", self.backend),
                "data_storage_snapshot",
                None,
            )
        try:
            value = reader() if callable(reader) else {}
        except Exception as exc:
            value = {"state": "UNAVAILABLE", "streams": {}, "error": str(exc)}
        return dict(value) if isinstance(value, Mapping) else {}

    def _refresh_data_storage(self) -> None:
        if not self._sessionless:
            return
        storage = self._data_storage_snapshot()
        streams = storage.get("streams")
        streams = streams if isinstance(streams, Mapping) else {}
        rows = []
        for name, raw in sorted(streams.items()):
            value = raw if isinstance(raw, Mapping) else {}
            csv_value = value.get("csv")
            csv_value = csv_value if isinstance(csv_value, Mapping) else {}
            last = value.get("last_write_utc_ns")
            last_text = "--" if last in (None, "") else str(last)
            errors = []
            jsonl_error = str(value.get("last_error") or "").strip()
            csv_error = str(csv_value.get("last_error") or "").strip()
            if jsonl_error:
                errors.append(f"JSONL: {jsonl_error}")
            if csv_error:
                errors.append(f"CSV: {csv_error}")
            rows.append(
                (
                    name,
                    value.get("path", "--"),
                    value.get("state", "IDLE"),
                    csv_value.get("path", "--"),
                    csv_value.get("state", "IDLE"),
                    value.get("dropped_records", 0),
                    csv_value.get("dropped_records", 0),
                    last_text,
                    _compact_text(" · ".join(errors), limit=96) or "--",
                )
            )
        _sync_table_rows(self.artifact_model, tuple(rows))
        root = str(storage.get("root") or "--")
        state = str(storage.get("state") or "UNAVAILABLE")
        dropped = int(storage.get("dropped_records") or 0)
        csv_dropped = int(storage.get("csv_dropped_records") or 0)
        self.session_values["state"].set_value(
            state,
            source="MeasurementDataStore",
            valid=True if state == "OK" else False if state == "UNAVAILABLE" else None,
        )
        self.session_values["session_id"].set_value(
            str(getattr(getattr(self.backend, "panel", self.backend), "_recording_mode", "idle")),
            source="transient recording owner",
        )
        self.session_values["output_dir"].set_value(root, source="measurement_data_root")
        self.session_values["status_label"].set_value(
            f"priority={int(storage.get('priority_pending') or 0)} · "
            f"bulk={int(storage.get('bulk_pending') or 0)} · "
            f"dropped={dropped} · csv_dropped={csv_dropped}",
            source="bounded writer queues",
        )
        self._refresh_latest_measurement_xlsx()

    def _latest_measurement_xlsx_snapshot(self) -> dict[str, object]:
        method = getattr(self.backend, "latest_measurement_xlsx_snapshot", None)
        if not callable(method):
            method = getattr(
                getattr(self.backend, "panel", self.backend),
                "latest_measurement_xlsx_snapshot",
                None,
            )
        try:
            value = method() if callable(method) else {}
        except Exception as exc:
            value = {"state": "FAILED", "error": str(exc)}
        return dict(value) if isinstance(value, Mapping) else {}

    def _refresh_latest_measurement_xlsx(self) -> None:
        if not self._sessionless or not hasattr(self, "latest_xlsx_status_label"):
            return
        snapshot = self._latest_measurement_xlsx_snapshot()
        state = str(snapshot.get("state") or "NOT_CREATED").replace("_", " ")
        target = str(snapshot.get("target_generation") or "")
        published = str(snapshot.get("published_generation") or "")
        row_counts = snapshot.get("row_counts")
        total_rows = 0
        if isinstance(row_counts, Mapping):
            for value in row_counts.values():
                try:
                    total_rows += max(0, int(value or 0))
                except (TypeError, ValueError):
                    continue
        generation = target or published or "--"
        visible_generation = _compact_text(generation, limit=34)
        error = str(snapshot.get("error") or "")
        detail = (
            _compact_text(error, limit=88)
            if error
            else f"generation={visible_generation} · rows={total_rows}"
        )
        self.latest_xlsx_status_label.setText(
            f"Latest XLSX · {state} · {detail}"
        )
        output = str(snapshot.get("output_path") or "")
        self.latest_xlsx_status_label.setToolTip(
            "\n".join(
                value
                for value in (
                    f"State: {state}",
                    f"Target generation: {target or '--'}",
                    f"Published generation: {published or '--'}",
                    f"Output: {output or '--'}",
                    f"Error: {error}" if error else "",
                )
                if value
            )
        )
        active = bool(snapshot.get("recording_active", False))
        generating = str(snapshot.get("state") or "") == "GENERATING"
        self.update_latest_xlsx_button.setEnabled(not active and not generating)
        self.open_latest_xlsx_button.setEnabled(
            bool(published and output and Path(output).is_file())
        )

    def _request_latest_measurement_xlsx(self) -> None:
        method = getattr(self.backend, "request_latest_measurement_xlsx", None)
        if not callable(method):
            method = getattr(
                getattr(self.backend, "panel", self.backend),
                "request_latest_measurement_xlsx",
                None,
            )
        accepted = False
        error = ""
        try:
            accepted = bool(method()) if callable(method) else False
        except Exception as exc:
            error = str(exc)
        if not accepted:
            self.latest_xlsx_status_label.setText(
                "Latest XLSX · REQUEST REJECTED"
                + (f" · {_compact_text(error, limit=88)}" if error else "")
            )
            self.latest_xlsx_status_label.setToolTip(error)
            return
        self._refresh_latest_measurement_xlsx()

    def _open_latest_measurement_xlsx(self) -> None:
        snapshot = self._latest_measurement_xlsx_snapshot()
        path = Path(str(snapshot.get("output_path") or ""))
        if not path.is_file():
            self._refresh_latest_measurement_xlsx()
            return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))
        self._refresh_latest_measurement_xlsx()

    def _open_data_folder(self) -> None:
        root = Path(str(self._data_storage_snapshot().get("root") or ""))
        try:
            root.mkdir(parents=True, exist_ok=True)
        except OSError:
            return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(root)))

    def _open_legacy_recordings_folder(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        root = Path(getattr(panel, "app_root", Path.cwd())) / "recordings"
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(root)))

    def _retry_data_storage(self) -> None:
        method = getattr(self.backend, "retry_data_storage", None)
        if not callable(method):
            method = getattr(
                getattr(self.backend, "panel", self.backend),
                "retry_data_storage",
                None,
            )
        if callable(method):
            try:
                method()
            except Exception:
                pass
        self._refresh_data_storage()

    def select_tab(self, tab_id: str) -> bool:
        try:
            index = self._tab_ids.index(str(tab_id))
        except ValueError:
            return False
        self.tabs.setCurrentIndex(index)
        return True

    def current_tab_id(self) -> str:
        index = self.tabs.currentIndex()
        return self._tab_ids[index] if 0 <= index < len(self._tab_ids) else "current"

    def _inspect_current_tab(self, index: int) -> None:
        tab_id = self._tab_ids[index] if 0 <= index < len(self._tab_ids) else "current"
        labels = {
            "current": "Data status" if self._sessionless else "Current session",
            "evidence": "Capture tools" if self._sessionless else "Active session evidence",
            "review": "Saved session review",
        }
        self.inspection_requested.emit(
            {
                "selection": labels[tab_id],
                "value": tab_id.upper(),
                "freshness": "Live owner snapshot" if tab_id != "review" else "Saved catalog",
                "contract": (
                    "Type-owned paths and producer fields are shown without coordinate conversion"
                    if self._sessionless
                    else "Session/package fields are projected without coordinate conversion"
                ),
                "verdict": "Review the selected tab",
                "source": "DataWorkspace" if self._sessionless else "SessionsWorkspace",
            }
        )

    def _refresh_current_measurements(self) -> None:
        reader = self._package_method("measurement_session_snapshot")
        try:
            snapshot = reader() if callable(reader) else {}
        except Exception:
            snapshot = {}
        snapshot = snapshot if isinstance(snapshot, Mapping) else {}
        rows = snapshot.get("measurements")
        rows = rows if isinstance(rows, (tuple, list)) else ()
        self.measurements_model.removeRows(0, self.measurements_model.rowCount())
        for row in rows:
            if not isinstance(row, Mapping):
                continue
            complete = row.get("evidence_complete")
            evidence = (
                "COMPLETE"
                if complete is True
                else "INCOMPLETE"
                if complete is False
                else "PENDING"
            )
            _append_table_row(
                self.measurements_model,
                (
                    row.get("sequence", "--"),
                    row.get("measurement_id", "--"),
                    row.get("state", "--"),
                    evidence,
                    row.get("revision", "--"),
                ),
            )
        for column in range(4):
            self.measurements_table.resizeColumnToContents(column)

    def refresh_run_preparation(self, snapshot: dict[str, RuntimeStatus]) -> None:
        self._refresh_protocol(snapshot)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        if not self._sessionless:
            self.current_session.refresh(snapshot)
            self._refresh_current_measurements()
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        recording_status = snapshot.get("recording")
        recording_locked = bool(
            recording_status
            and recording_status.state in {"ACTIVE", "FINALIZING"}
        )
        manual_record_button = self.operation_buttons.get("recording.toggle")
        if manual_record_button is not None:
            panel = getattr(self.backend, "panel", self.backend)
            automatic = bool(getattr(panel, "_automatic_session_recording", False))
            manual_active = bool(
                recording_status
                and recording_status.state == "ACTIVE"
                and not automatic
            )
            manual_record_button.setText(
                "Stop manual recording" if manual_active else "Manual record"
            )
            manual_record_button.setEnabled(
                manual_active
                or not recording_status
                or recording_status.state in {"IDLE", "SAVED", "DEGRADED"}
            )
        self.refresh_run_preparation(snapshot)
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
            self.pnp_capture_status_label.setText("Capture HR Still: in progress")
        elif records:
            latest = records[-1]
            ftp_text = "FTP=ok" if latest.get("ftp_downloaded") else "FTP=disabled/failed"
            self.pnp_capture_status_label.setText(
                f"Capture HR Still: {latest.get('capture_id', 'unknown')} · "
                f"{latest.get('state', 'unknown')} · camera-internal · {ftp_text}"
            )
        else:
            self.pnp_capture_status_label.setText(
                "Capture HR Still: camera-internal storage · select event, then import exact left/right files"
            )
        oz_records = list(getattr(panel, "oz_photo_capture_records", ()) or ())
        if oz_capture_active:
            self.oz_capture_status_label.setText("Capture OZ Still: in progress")
        elif oz_records:
            latest_oz = oz_records[-1]
            oz_ftp_text = "FTP=ok" if latest_oz.get("ftp_downloaded") else "FTP=disabled/failed"
            self.oz_capture_status_label.setText(
                f"Capture OZ Still: {latest_oz.get('capture_id', 'unknown')} · "
                f"{latest_oz.get('state', 'unknown')} · camera-internal · RTSP separate · {oz_ftp_text}"
            )
        else:
            self.oz_capture_status_label.setText(
                "Capture OZ Still: camera-internal FHD still · separate from RTSP video"
            )
        for key, value in self.session_values.items():
            raw = data.get(key)
            if key == "elapsed_s" and raw not in (None, ""):
                raw = f"{float(raw):.1f} s"
            value.set_value(raw, source=str(data.get("source", "")))
        counts = data.get("sample_counts", {})
        for key, value in self.count_values.items():
            value.set_value(counts.get(key, 0) if isinstance(counts, dict) else "--", source="session spool rows")
        if not self._sessionless:
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
        if self._sessionless:
            self._refresh_data_storage()
        else:
            self.refresh_sessions(force=False)
            self._refresh_package_details()

    def _package_method(self, *names: str) -> Callable[..., object] | None:
        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            for name in names:
                method = getattr(owner, name, None)
                if callable(method):
                    return method
        return None

    @staticmethod
    def _record_package_snapshot(record: Mapping[str, object] | None) -> dict[str, object]:
        if not isinstance(record, Mapping):
            return {}
        values = dict(record)
        for key in ("measurement_package", "package_manifest", "package"):
            nested = record.get(key)
            if isinstance(nested, Mapping):
                values.update(nested)
        if not values.get("state"):
            values["state"] = values.get("lifecycle_state", values.get("status", "IDLE"))
        if not values.get("evidence_completeness") and values.get("completeness_state"):
            values["evidence_completeness"] = {
                "status": values.get("completeness_state"),
                "complete": values.get("evidence_complete"),
                "missing_required": values.get("missing_reasons", ()),
            }
        return values

    def _selected_package_snapshot(self) -> dict[str, object]:
        record = self._selected_record() if hasattr(self, "session_selector") else None
        selected = self._record_package_snapshot(record)
        reader = self._package_method("measurement_package_snapshot")
        if reader is None:
            return selected
        try:
            live = reader()
        except Exception as exc:
            self.package_action_status_label.setText(f"Package status unavailable: {exc}")
            return selected
        if not isinstance(live, Mapping):
            return selected
        live_values = dict(live)
        selected_id = str(selected.get("session_id", "") or "")
        selected_measurement_id = str(selected.get("measurement_id", "") or "")
        live_id = str(live_values.get("session_id", "") or "")
        live_measurement_id = str(live_values.get("measurement_id", "") or "")
        live_state = str(live_values.get("state", "IDLE") or "IDLE").upper()
        if selected_id and live_id and (
            selected_id != live_id
            or selected_measurement_id != live_measurement_id
        ):
            selected["_actions_available"] = False
            return selected
        if live_state in {"COLLECTING", "POST_CAPTURE", "VERIFYING", "RECOVERABLE"}:
            live_values["_actions_available"] = True
            return live_values
        if selected_id and not live_id:
            selected["_actions_available"] = False
            return selected
        selected.update(live_values)
        if isinstance(live_values.get("artifacts"), (tuple, list, Mapping)):
            selected["artifact_details"] = live_values["artifacts"]
        selected["_actions_available"] = True
        return selected

    @staticmethod
    def _package_mapping_rows(value: object, *, identity_key: str) -> list[dict[str, object]]:
        if isinstance(value, Mapping):
            rows: list[dict[str, object]] = []
            for identity, raw in value.items():
                if isinstance(raw, Mapping):
                    row = dict(raw)
                else:
                    row = {"state": raw}
                row.setdefault(identity_key, str(identity))
                rows.append(row)
            return rows
        if isinstance(value, (tuple, list)):
            return [dict(raw) for raw in value if isinstance(raw, Mapping)]
        return []

    @staticmethod
    def _source_expectation_row(source: Mapping[str, object]) -> tuple[object, ...]:
        source_id = source.get(
            "source_id",
            source.get("producer_id", source.get("name", source.get("id", "--"))),
        )
        required_roles = source.get("required_roles")
        if isinstance(required_roles, (tuple, list)):
            expected = ", ".join(str(role) for role in required_roles) or "none"
        else:
            required = source.get("required", source.get("expected", True))
            expected = (
                "required"
                if required is True
                else "optional"
                if required is False
                else str(required or "--")
            )
        state = str(source.get("state", source.get("status", "")) or "")
        if not state and "active" in source:
            state = "ACTIVE" if bool(source.get("active")) else "INACTIVE"
        metadata = source.get("metadata")
        metadata = metadata if isinstance(metadata, Mapping) else {}
        start = source.get(
            "active_from",
            source.get(
                "activated_utc_ns",
                source.get(
                    "active_from_pair_sequence",
                    metadata.get("active_from", metadata.get("activated_utc_ns")),
                ),
            ),
        )
        end = source.get(
            "active_until",
            source.get(
                "deactivated_utc_ns",
                source.get(
                    "active_to_pair_sequence",
                    metadata.get("active_until", metadata.get("deactivated_utc_ns")),
                ),
            ),
        )
        active_range = " → ".join(
            ("--" if value in (None, "") else str(value)) for value in (start, end)
        )
        state_range = state or "--"
        if start not in (None, "") or end not in (None, ""):
            state_range += f" · {active_range}"
        detail = source.get(
            "detail",
            source.get(
                "reason",
                source.get(
                    "expectation",
                    metadata.get("detail", metadata.get("reason", source.get("role", ""))),
                ),
            ),
        )
        return (source_id, expected, state_range, detail or "--")

    @staticmethod
    def _artifact_inventory_row(
        artifact: Mapping[str, object],
        *,
        package_revision: object,
    ) -> tuple[object, ...]:
        path = artifact.get(
            "relative_path",
            artifact.get("path", artifact.get("name", artifact.get("artifact", "--"))),
        )
        role = artifact.get("role", artifact.get("artifact_role", artifact.get("kind", "--")))
        required = artifact.get("required")
        required_text = "yes" if required is True else "no" if required is False else "--"
        state = artifact.get("state", artifact.get("status", "--"))
        size_bytes = artifact.get("size_bytes", artifact.get("bytes", "--"))
        sha256 = artifact.get("sha256", artifact.get("hash", "--"))
        revision = artifact.get("revision", package_revision or "--")
        metadata = artifact.get("metadata")
        metadata = metadata if isinstance(metadata, Mapping) else {}
        reason = artifact.get(
            "failure_reason",
            artifact.get(
                "missing_reason",
                artifact.get(
                    "reason",
                    artifact.get(
                        "error",
                        artifact.get(
                            "detail",
                            metadata.get(
                                "failure_reason",
                                metadata.get("missing_reason", metadata.get("reason", "")),
                            ),
                        ),
                    ),
                ),
            ),
        )
        return (
            path,
            role,
            required_text,
            state,
            size_bytes if size_bytes not in (None, "") else "--",
            sha256 or "--",
            revision,
            reason or "--",
        )

    @staticmethod
    def _reason_count(value: object) -> int:
        if isinstance(value, Mapping):
            return len(value)
        if isinstance(value, (tuple, list, set)):
            return len(value)
        return 1 if value not in (None, "") else 0

    def _apply_package_details(self, package: Mapping[str, object]) -> None:
        state = str(package.get("state", package.get("lifecycle_state", "IDLE")) or "IDLE").upper()
        session_id = str(package.get("session_id", "") or "")
        self.package_state_label.setText(
            f"State: {state}" + (f" · {session_id}" if session_id else "")
        )
        revision = package.get("revision", package.get("revision_number", "--"))
        self.package_revision_label.setText(f"Revision: {revision or '--'}")

        completeness = package.get("evidence_completeness")
        completeness = completeness if isinstance(completeness, Mapping) else {}
        completeness_state = str(
            completeness.get(
                "status",
                package.get("completeness_state", package.get("completeness", "")),
            )
            or ""
        ).upper()
        if not completeness_state:
            completeness_state = (
                "COMPLETE"
                if state == "SEALED_COMPLETE"
                else "INCOMPLETE"
                if state in {"SEALED_INCOMPLETE", "INTEGRITY_FAILED"}
                else "PENDING"
            )
        missing_count = max(
            self._reason_count(completeness.get("missing_required")),
            self._reason_count(package.get("missing_reasons")),
        )
        failed_count = self._reason_count(package.get("failed_reasons"))
        reason_summary = ", ".join(
            text
            for count, text in (
                (missing_count, f"missing={missing_count}"),
                (failed_count, f"failed={failed_count}"),
            )
            if count
        )
        self.package_completeness_label.setText(
            f"Evidence: {completeness_state}"
            + (f" · {reason_summary}" if reason_summary else "")
        )

        verification = package.get("hash_verification", package.get("verification"))
        verification = verification if isinstance(verification, Mapping) else {}
        verification_state = str(
            package.get(
                "integrity_status",
                package.get(
                    "verification_state",
                    verification.get("status", verification.get("state", "NOT RUN")),
                ),
            )
            or "NOT RUN"
        ).upper()
        verification_parts = []
        manifest_status = verification.get("manifest_status")
        if manifest_status not in (None, ""):
            verification_parts.append(f"manifest={manifest_status}")
        declared = verification.get(
            "artifact_declared_count", verification.get("artifact_hashes_declared")
        )
        not_run = verification.get(
            "artifact_not_run_count", verification.get("artifacts_not_run")
        )
        if declared not in (None, ""):
            verification_parts.append(f"artifacts={declared}")
        if not_run not in (None, "", 0):
            verification_parts.append(f"not-run={not_run}")
        self.package_verification_label.setText(
            f"Verification: {verification_state}"
            + (f" · {', '.join(verification_parts)}" if verification_parts else "")
        )

        sources = self._package_mapping_rows(
            package.get("source_expectations"), identity_key="source_id"
        )
        source_rows = tuple(self._source_expectation_row(source) for source in sources)
        _sync_table_rows(self.package_source_model, source_rows)

        artifact_values = package.get("artifact_details", package.get("artifacts"))
        artifacts = self._package_mapping_rows(artifact_values, identity_key="relative_path")
        artifact_rows = tuple(
            self._artifact_inventory_row(artifact, package_revision=revision)
            for artifact in artifacts
        )
        _sync_table_rows(self.package_artifact_model, artifact_rows)

        sealed = state in {"SEALED_COMPLETE", "SEALED_INCOMPLETE"}
        actions_available = bool(package.get("_actions_available", True))
        self.verify_package_button.setEnabled(
            actions_available
            and state in {"SEALED_COMPLETE", "SEALED_INCOMPLETE", "INTEGRITY_FAILED"}
            and self._package_method("verify_measurement_package", "verify_package") is not None
        )
        self.create_revision_button.setEnabled(
            actions_available
            and sealed
            and self._package_method("begin_measurement_revision", "create_revision") is not None
        )

    def _refresh_package_details(self) -> None:
        package = self._selected_package_snapshot()
        if not package:
            self._apply_package_details({"state": "IDLE"})
            return
        self._apply_package_details(package)

    def _verify_measurement_package(self) -> None:
        method = self._package_method("verify_measurement_package", "verify_package")
        if method is None:
            self.package_action_status_label.setText("Verify package: control unavailable")
            return
        try:
            result = method()
        except Exception as exc:
            self.package_action_status_label.setText(f"Verify package: ERROR · {exc}")
            return
        self._refresh_package_details()
        result_mapping = result if isinstance(result, Mapping) else None
        if result_mapping is None:
            mapper = getattr(result, "to_mapping", None)
            try:
                mapped = mapper() if callable(mapper) else None
            except Exception:
                mapped = None
            result_mapping = mapped if isinstance(mapped, Mapping) else None
        if result_mapping is not None:
            state = str(
                result_mapping.get("status", result_mapping.get("state", "completed"))
                or "completed"
            )
            errors = result_mapping.get("errors", result_mapping.get("issues", ()))
            error_count = self._reason_count(errors)
            self.package_verification_label.setText(
                f"Verification: {state.upper()}"
                + (f" · issues={error_count}" if error_count else "")
            )
        self.package_action_status_label.setText("Verify package: completed")

    def _begin_measurement_revision(self) -> None:
        method = self._package_method("begin_measurement_revision", "create_revision")
        if method is None:
            self.package_action_status_label.setText("Create Revision: control unavailable")
            return
        try:
            result = method()
        except Exception as exc:
            self.package_action_status_label.setText(f"Create Revision: ERROR · {exc}")
            return
        self._refresh_package_details()
        self.package_action_status_label.setText(
            "Create Revision: started"
            if result is not False
            else "Create Revision: request rejected"
        )

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
        # Qt delivers same-thread slots synchronously.  Publish the pending
        # state first so a completed owner readback is the last writer.
        self.ulog_status_label.setText("ULog import requested: awaiting owner file selection")
        self.ulog_command_requested.emit(
            "recording.ulog.import",
            {
                "source": str(self.ulog_source_combo.currentData() or "p1"),
                "ownership": str(self.ulog_ownership_combo.currentData() or "UNKNOWN"),
                "file_selection": "owner_dialog_required",
            },
        )

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
            action = "Capture requested" if command_id.endswith("capture") else "Import requested"
            status_label = (
                self.pnp_capture_status_label
                if command_id.startswith("recording.hr")
                else self.oz_capture_status_label
            )
            channel = "HR" if command_id.startswith("recording.hr") else "OZ"
            status_label.setText(f"{channel} {action}: awaiting owner")
            # Direct Qt connections run before emit() returns.  Set pending
            # first so the owner's synchronous success/failure text wins.
            self.hr_command_requested.emit(command_id, dict(payload))
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
        method_snapshot_reader = getattr(self.backend, "measurement_methods_snapshot", None)
        try:
            method_snapshot = method_snapshot_reader() if callable(method_snapshot_reader) else {}
        except Exception:
            method_snapshot = {}
        selected_method: dict[str, object] | None = None
        if isinstance(method_snapshot, Mapping):
            selected_id = str(method_snapshot.get("selected_method_id") or "")
            rows = method_snapshot.get("ready_methods")
            if isinstance(rows, list):
                selected_method = next(
                    (
                        dict(row)
                        for row in rows
                        if isinstance(row, Mapping)
                        and str(row.get("method_id") or "") == selected_id
                    ),
                    None,
                )
            if selected_method is None and str(method_snapshot.get("schema") or ""):
                selected_method = {
                    "method_id": "",
                    "instance_id": "",
                    "label": "Ready measurement method",
                    "ready": False,
                    "reasons": ["no ready measurement method"],
                    "requires_live_pixhawk": False,
                    "requires_live_gnss": False,
                }
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
            measurement_method=selected_method,
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
        if self._sessionless:
            self._refresh_data_storage()
            return
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
                record.get("measurement_id"),
                record.get("status"),
                record.get("lifecycle_state"),
                record.get("completeness_state"),
                record.get("revision"),
                record.get("integrity_status"),
                record.get("error"),
                record.get("retryable"),
                tuple(record.get("artifacts", ()) or ()),
                json.dumps(record.get("source_expectations", ()), sort_keys=True, default=str),
                json.dumps(record.get("artifact_details", ()), sort_keys=True, default=str),
                json.dumps(record.get("hash_verification", {}), sort_keys=True, default=str),
            )
            for record in records
        )
        if signature == self._session_catalog_signature and self.session_selector.count() > 0:
            return
        current = str(self.session_selector.currentData() or "")
        panel = getattr(self.backend, "panel", self.backend)
        if not current:
            panel_session_id = str(getattr(panel, "recording_session_id", "") or "")
            panel_measurement_id = str(getattr(panel, "_measurement_id", "") or "")
            current = (
                f"{panel_session_id}::{panel_measurement_id}"
                if panel_session_id and panel_measurement_id
                else panel_session_id
            )
        self.session_records = {
            str(record.get("catalog_key") or record.get("session_id")): dict(record)
            for record in records
            if record.get("session_id")
        }
        self._session_catalog_signature = signature
        self.session_selector.blockSignals(True)
        try:
            self.session_selector.clear()
            for record in records:
                session_id = str(record.get("session_id") or "")
                measurement_id = str(record.get("measurement_id") or "")
                catalog_key = str(record.get("catalog_key") or session_id)
                status = str(record.get("status") or "unknown")
                identity = f"{session_id} / {measurement_id}" if measurement_id else session_id
                self.session_selector.addItem(f"{identity} [{status}]", catalog_key)
            if current:
                index = self.session_selector.findData(current)
                if index >= 0:
                    self.session_selector.setCurrentIndex(index)
        finally:
            self.session_selector.blockSignals(False)
        self._refresh_review_artifacts()

    def _selected_record(self) -> dict[str, object] | None:
        catalog_key = str(self.session_selector.currentData() or "")
        return self.session_records.get(catalog_key)

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
        self._refresh_package_details()

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
        relative = Path(artifact)
        if relative.is_absolute() or ".." in relative.parts:
            return None
        root = output_dir.resolve()
        candidates = (output_dir / relative, output_dir / "published" / relative)
        for candidate in candidates:
            try:
                candidate.resolve().relative_to(root)
            except (OSError, ValueError):
                continue
            if candidate.exists():
                return candidate
        return candidates[0]

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
        measurement_id = str(record.get("measurement_id") or "")
        if measurement_id:
            data = _evidence(self.backend).session_artifact_rows(
                str(record.get("session_id")),
                artifact,
                measurement_id=measurement_id,
            )
        else:
            data = _evidence(self.backend).session_artifact_rows(
                str(record.get("session_id")), artifact
            )
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


# Public name for the sessionless Data surface; keep compatibility aliases for
# embedded callers while the visible product vocabulary is Data.
DataWorkspace = RecordingWorkspace
SessionsWorkspace = RecordingWorkspace


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
        self.ftp_view = self._diagnostic_text_view()
        self.tabs.addTab(self.system_table, "System")
        self.tabs.addTab(self.camera_view, "Camera")
        self.tabs.addTab(self.pixhawk_view, "Pixhawk")
        self.tabs.addTab(self.gnss_view, "GNSS")
        self.tabs.addTab(log_page, "Log")
        self.tabs.addTab(self.ftp_view, "FTP")
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
            "diagnostics.ftp": 5,
            "diagnostics.measurement": 6,
            "diagnostics.performance": 7,
        }
        self.tabs.setCurrentIndex(tab_indexes.get(context_id, 0))

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        refreshers = (
            self._refresh_system,
            self._refresh_camera,
            self._refresh_pixhawk,
            self._refresh_gnss,
            self._refresh_log,
            self._refresh_ftp,
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

    def _refresh_ftp(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        backend = getattr(self.backend, "ftp_diagnostic_text", None)
        if callable(backend):
            self._set_diagnostic_text(self.ftp_view, str(backend()), move_to_end=True)
        else:
            self._set_diagnostic_text(self.ftp_view, "FTP diagnostic log is unavailable.")

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
        panel = getattr(self.backend, "panel", self.backend)
        identity = getattr(panel, "nvidia_runtime_identity", {}) or {}
        selection = getattr(panel, "decoder_selection_status", {}) or {}
        active_decoders = getattr(panel, "active_stream_decoders", {}) or {}
        fallback_reasons = getattr(panel, "decoder_fallback_reasons", {}) or {}
        runtime_overrides = getattr(panel, "_runtime_decoder_overrides", {}) or {}
        requested = selection.get("requested") or getattr(
            getattr(panel, "settings", None), "rtsp_decoder", "auto"
        )
        lines = [
            self._status_text(snapshot, ("fps",)),
            "",
            "NVIDIA / H.264 decoder evidence:",
            f"GPU model: {identity.get('model') or '--'}",
            f"Driver: {identity.get('driver_version') or '--'}",
            f"GPU UUID: {identity.get('gpu_uuid') or '--'}",
            f"Bus: {identity.get('bus_location') or '--'}",
            f"NVDEC available: {selection.get('nvidia_available') if selection.get('nvidia_available') is not None else '--'}",
            f"Requested mode: {requested}",
            f"Selected candidate: {selection.get('effective') or '--'}",
            f"Candidates: {', '.join(str(item) for item in tuple(selection.get('candidates') or ())) or '--'}",
            f"Allowed/degraded: {selection.get('allowed')}/{bool(selection.get('degraded', False))}",
            f"Selection reason: {selection.get('reason') or '--'}",
        ]
        for side, label in (("left", "Left"), ("right", "Right")):
            presentation = decoder_presentation(
                requested_mode=requested,
                active_decoder=(active_decoders.get(side) if isinstance(active_decoders, Mapping) else None),
                runtime_override=(
                    runtime_overrides.get(side)
                    if isinstance(runtime_overrides, Mapping) and side in runtime_overrides
                    else getattr(panel, "_runtime_decoder_override", None)
                ),
                fallback_reason=(fallback_reasons.get(side) if isinstance(fallback_reasons, Mapping) else ""),
            )
            lines.append(f"{label}: {presentation.value_text} | {presentation.detail}")
        stream_stats_by_side = getattr(panel, "camera_stream_stats", {}) or {}
        if not isinstance(stream_stats_by_side, Mapping):
            stream_stats_by_side = {}
        lines.extend(
            (
                "",
                "RTP / H.264 input measurements (worker rolling rates):",
                "RTP missing = sequence gap estimated missing packets; -- = no metric yet.",
            )
        )
        for side, label in (("left", "Left"), ("right", "Right")):
            side_stats = stream_stats_by_side.get(side, {})
            side_stats = dict(side_stats) if isinstance(side_stats, Mapping) else {}
            metrics_text = format_camera_input_metrics(side, side_stats).replace("\n", " | ")
            lines.append(
                f"{label}: {metrics_text}"
            )
        lines.append("GPU utilization is diagnostic only; it is not a readiness or success gate.")
        self._set_diagnostic_text(self.performance_view, "\n".join(lines))

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
