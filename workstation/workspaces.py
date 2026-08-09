from __future__ import annotations

import time
from pathlib import Path
from typing import Callable

from PyQt5 import QtCore, QtGui, QtWidgets

from workstation_core.coordinate_contract import (
    BASELINE_TRANSLATION_FRAME,
    BODY_FRAME,
    CAMERA_FRAME,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    WORLD_FRAME,
)

from .components import EngineeringValue, EmptyStateView, StatusIndicator, WorkspaceHeader
from .evidence import EvidenceAdapter
from .protocol import ProtocolReadiness, evaluate_recording_protocol
from .status_model import RuntimeStatus, StatusModel, StatusSeverity


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


class OverviewWorkspace(QtWidgets.QWidget):
    open_context = QtCore.pyqtSignal(str)

    def __init__(self, backend: object, status_model: StatusModel, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Overview"))

        status_group = QtWidgets.QGroupBox("Global status")
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
                ("gnss", "RTK / GNSS"),
                ("detection", "DET"),
                ("measurement", "MEAS"),
                ("recording", "REC"),
            )
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator, index // 2, index % 2)
        layout.addWidget(status_group)

        evidence_group = QtWidgets.QGroupBox("Measurement evidence")
        evidence_layout = QtWidgets.QGridLayout(evidence_group)
        evidence_layout.setContentsMargins(8, 8, 8, 8)
        self.session_value = EngineeringValue("Session")
        self.measurement_value = EngineeringValue("Last result")
        self.contract_value = EngineeringValue("Public contract")
        evidence_layout.addWidget(self.session_value, 0, 0)
        evidence_layout.addWidget(self.measurement_value, 1, 0)
        evidence_layout.addWidget(self.contract_value, 2, 0)
        evidence_layout.setColumnStretch(0, 1)
        layout.addWidget(evidence_group)

        actions = QtWidgets.QHBoxLayout()
        camera_button = QtWidgets.QPushButton("Camera workspace")
        camera_button.clicked.connect(lambda: self.open_context.emit("devices.cameras"))
        measurement_button = QtWidgets.QPushButton("Measurement workspace")
        measurement_button.clicked.connect(lambda: self.open_context.emit("processing.measurement"))
        diagnostics_button = QtWidgets.QPushButton("Show diagnostics")
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
        self.measurement_value.set_value(
            str(getattr(panel, "last_distance_status", "measurement evidence 대기") or ""),
            source="MeasurementAttempt/controller result",
            frame=MEASUREMENT_FRAME,
            valid=str(getattr(panel, "last_distance_quality", "BAD")).upper() in {"GOOD", "VALID"},
        )
        self.contract_value.set_value(
            f"{MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]",
            source="coordinate_contract.py",
        )


class CameraPreview(QtWidgets.QFrame):
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
        self.meta = QtWidgets.QLabel("status: waiting")
        self.meta.setObjectName("cameraPreviewMeta")
        self.meta.setWordWrap(True)
        self.meta.setMinimumWidth(0)
        self.meta.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        layout.addWidget(title)
        layout.addWidget(self.image, 1)
        layout.addWidget(self.meta)
        self._image: QtGui.QImage | None = None

    def set_frame(self, image: QtGui.QImage, metadata: object) -> None:
        self._image = image
        self._render()
        self.meta.setText(self._metadata_text(metadata, live=True))

    def set_metadata(self, metadata: object) -> None:
        live = self._image is not None and not self._image.isNull()
        self.meta.setText(self._metadata_text(metadata, live=live))

    def _metadata_text(self, metadata: object, *, live: bool) -> str:
        data = metadata if isinstance(metadata, dict) else {}
        stamp = data.get("timestamp_s")
        source_stamp = data.get("source_timestamp_s")
        parts = ["LIVE" if live else "WAITING"]
        if stamp is not None:
            parts.append(f"receive={float(stamp):.3f} monotonic")
        if source_stamp is not None:
            parts.append(f"camera_ts={float(source_stamp):.3f}")
        return " | ".join(parts)

    def _render(self) -> None:
        if self._image is None or self._image.isNull():
            self.image.setText("NO VIDEO")
            self.image.setPixmap(QtGui.QPixmap())
            return
        self.image.setPixmap(
            QtGui.QPixmap.fromImage(self._image).scaled(
                self.image.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )
        )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        self._render()
        super().resizeEvent(event)


class CameraWorkspace(QtWidgets.QWidget):
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
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.previews = {side: CameraPreview(side) for side in ("left", "right")}
        splitter.addWidget(self.previews["left"])
        splitter.addWidget(self.previews["right"])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, 1)
        facts = QtWidgets.QGroupBox("Runtime facts")
        facts_layout = QtWidgets.QGridLayout(facts)
        self.camera_values = {}
        for col, (key, label) in enumerate((("camera.left", "CAM L"), ("camera.right", "CAM R"), ("fps", "Decoded FPS"), ("detection", "Detection"))):
            value = EngineeringValue(label)
            self.camera_values[key] = value
            facts_layout.addWidget(value, 0, col)
        layout.addWidget(facts)

        detection_group = QtWidgets.QGroupBox("Detection configuration / target evidence")
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
        layout.addWidget(detection_group)

        operations_group = QtWidgets.QGroupBox("Camera readiness")
        operations_layout = QtWidgets.QGridLayout(operations_group)
        operations_layout.setContentsMargins(6, 6, 6, 6)
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for index, (command_id, label) in enumerate(
            (
                ("camera.discover_host_ip", "Discover Camera Network"),
                ("camera.auto_connect", "Auto-Connect Cameras"),
                ("camera.check_control", "Verify Camera Control"),
                ("camera.enable_rtsp", "Enable Video Stream"),
                ("camera.check_rtsp", "Verify Video Stream"),
            )
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            operations_layout.addWidget(button, index // 3, index % 3)
        layout.addWidget(operations_group)

        mapping_group = QtWidgets.QGroupBox("Camera mapping / settings")
        mapping_layout = QtWidgets.QHBoxLayout(mapping_group)
        mapping_layout.setContentsMargins(6, 6, 6, 6)
        for command_id, label in (
            ("camera.swap", "Swap Camera Sides"),
            ("camera.save_settings", "Save Camera Settings"),
            ("legacy.open", "Open Device Candidate Tools"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            mapping_layout.addWidget(button)
        mapping_layout.addStretch(1)
        layout.addWidget(mapping_group)
        self._connect_panes()

    def _connect_panes(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        for side in ("left", "right"):
            pane = getattr(panel, f"{side}_pane", None)
            signal = getattr(pane, "frame_changed", None)
            if signal is not None:
                signal.connect(lambda _label, image, metadata, side=side: self.previews[side].set_frame(image, metadata))
            stats_signal = getattr(pane, "stats_changed", None)
            if stats_signal is not None:
                stats_signal.connect(lambda _label, stats, side=side: self.previews[side].set_metadata(stats))

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
        for command_id in ("camera.swap", "camera.save_settings"):
            button = self.operation_buttons.get(command_id)
            if button is not None:
                button.setEnabled(not recording_locked)
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
        self.target_model.removeRows(0, self.target_model.rowCount())
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
            _append_table_row(
                self.target_model,
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
            WorkspaceHeader(
                "Time / Sync",
                "RTCP/NTP frame evidence",
            )
        )

        workflow_group = QtWidgets.QGroupBox("Clock readiness workflow")
        workflow_layout = QtWidgets.QHBoxLayout(workflow_group)
        workflow_layout.setContentsMargins(6, 6, 6, 6)
        workflow_layout.setSpacing(6)
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("time.set_pc", "Set WIRIS time from PC"),
            ("time.check_sync", "Check RTCP/NTP sync"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            workflow_layout.addWidget(button)
        workflow_layout.addStretch(1)
        layout.addWidget(workflow_group)

        sync_group = QtWidgets.QGroupBox("RTCP/NTP stream evidence")
        sync_layout = QtWidgets.QVBoxLayout(sync_group)
        self.sync_status = QtWidgets.QLabel("RTCP/NTP frame timestamp 대기 중")
        self.sync_status.setObjectName("statusStrip")
        self.sync_status.setWordWrap(True)
        self.sync_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.sync_summary = QtWidgets.QPlainTextEdit()
        self.sync_summary.setReadOnly(True)
        self.sync_summary.setMaximumBlockCount(80)
        self.sync_summary.setFixedHeight(110)
        sync_layout.addWidget(self.sync_status)
        sync_layout.addWidget(self.sync_summary)
        layout.addWidget(sync_group)

        contract = QtWidgets.QLabel("Evidence: RTCP/NTP frame timestamps")
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
        sync_status = self._widget_text(
            getattr(panel, "clock_sync_status_label", None), "RTCP/NTP frame timestamp 대기 중"
        )
        self.sync_status.setText(_compact_text(sync_status))
        self.sync_status.setToolTip(sync_status)
        self.sync_summary.setPlainText(
            self._widget_text(getattr(panel, "clock_sync_summary_text", None), "")
        )


class MeasurementWorkspace(QtWidgets.QWidget):
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
        layout.addWidget(WorkspaceHeader("Measurement", "Quality · timing · output frame"))
        self.result_status = QtWidgets.QLabel("measurement evidence 대기")
        self.result_status.setObjectName("measurementResultStatus")
        self.result_status.setWordWrap(True)
        layout.addWidget(self.result_status)

        run_group = QtWidgets.QGroupBox("Run control")
        run_layout = QtWidgets.QHBoxLayout(run_group)
        run_layout.setContentsMargins(6, 6, 6, 6)
        run_layout.setSpacing(6)
        self.control_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("camera.toggle", "Preview"),
            ("detection.toggle", "Detection"),
            ("recording.toggle", "Record"),
            ("session.save", "Save session"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.control_buttons[command_id] = button
            run_layout.addWidget(button)
        run_layout.addStretch(1)
        layout.addWidget(run_group)

        upstream_group = QtWidgets.QGroupBox("Upstream evidence")
        upstream_layout = QtWidgets.QVBoxLayout(upstream_group)
        upstream_layout.setContentsMargins(6, 6, 6, 6)
        upstream_layout.setSpacing(4)
        preview_row = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.previews = {side: CameraPreview(side) for side in ("left", "right")}
        for side in ("left", "right"):
            preview = self.previews[side]
            preview.setMinimumHeight(112)
            preview.setMaximumHeight(220)
            preview_row.addWidget(preview)
        preview_row.setMinimumHeight(112)
        preview_row.setMaximumHeight(220)
        preview_row.setStretchFactor(0, 1)
        preview_row.setStretchFactor(1, 1)
        upstream_layout.addWidget(preview_row)
        self.upstream_values: dict[str, EngineeringValue] = {}
        upstream_fields = (
            ("camera.left", "CAM L"),
            ("camera.right", "CAM R"),
            ("detection", "Detection"),
            ("synchronization", "Timing / sync"),
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
            facts_layout.addWidget(value, index // 4, index % 4)
        upstream_layout.addLayout(facts_layout)
        layout.addWidget(upstream_group)
        self._connect_panes()

        quality_group = QtWidgets.QGroupBox("Current measurement quality")
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
        layout.addWidget(quality_group)

        values_group = QtWidgets.QGroupBox("Public result")
        values_layout = QtWidgets.QGridLayout(values_group)
        self.values = {
            "range": EngineeringValue("Range"),
            "forward": EngineeringValue("Forward"),
            "right": EngineeringValue("Right"),
            "down": EngineeringValue("Down"),
        }
        for index, key in enumerate(("range", "forward", "right", "down")):
            values_layout.addWidget(self.values[key], index // 2, index % 2)
        layout.addWidget(values_group)
        contract = QtWidgets.QGroupBox("Input / output contract")
        contract_layout = QtWidgets.QFormLayout(contract)
        contract_layout.setContentsMargins(8, 8, 8, 8)
        for label, value in (
            ("World / attitude", f"{BODY_FRAME} → {WORLD_FRAME}"),
            ("Camera input", f"{CAMERA_FRAME} · [u,v]"),
            ("Baseline translation", f"{BASELINE_TRANSLATION_FRAME} · m"),
            ("Public output", f"{MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}] · m"),
        ):
            label_widget = QtWidgets.QLabel(value)
            label_widget.setObjectName("compactContract")
            label_widget.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            label_widget.setWordWrap(True)
            contract_layout.addRow(label, label_widget)
        layout.addWidget(contract)

        settings_group = QtWidgets.QGroupBox("Measurement / synchronization settings")
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
                "Intrinsics source",
                combo("intrinsics_source", (("Checkerboard", "checkerboard"), ("Manual fallback", "manual"))),
            ),
            (
                "Focal length",
                double_spin("focal_px", 1.0, 100000.0, 3, " px"),
            ),
            (
                "Extrinsics / baseline",
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

        gate_group = QtWidgets.QGroupBox("Detection pair gates")
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

        sync_group = QtWidgets.QGroupBox("Camera RTCP/NTP synchronization gate")
        sync_form = QtWidgets.QGridLayout(sync_group)
        sync_form.setHorizontalSpacing(8)
        sync_form.setVerticalSpacing(5)
        sync_form.setColumnStretch(1, 1)
        sync_form.setColumnStretch(3, 1)
        sync_fields = (
            ("History", double_spin("camera_sync_history_window_s", 0.1, 5.0, 2, " s")),
            ("Receive skew", double_spin("camera_sync_receive_pair_max_delta_ms", 0.0, 2000.0, 1, " ms")),
            ("RTCP/NTP Δt", double_spin("camera_sync_rtcp_pair_max_delta_ms", 0.0, 1000.0, 1, " ms")),
            ("SR stale", double_spin("camera_sync_max_sr_age_s", 0.1, 60.0, 2, " s")),
            ("SR-PC offset", double_spin("camera_sync_max_pc_offset_ms", 0.0, 5000.0, 1, " ms")),
            ("Clock rate error", double_spin("camera_sync_clock_rate_error_pct", 0.0, 10.0, 3, " %")),
            ("Required SR", int_spin("camera_sync_required_sr_count", 1, 20, " count")),
            ("Warmup", double_spin("camera_sync_warmup_timeout_s", 1.0, 120.0, 1, " s")),
        )
        for index, (label, widget) in enumerate(sync_fields):
            row = index // 2
            column = (index % 2) * 2
            sync_form.addWidget(QtWidgets.QLabel(label), row, column)
            sync_form.addWidget(widget, row, column + 1)
        settings_layout.addWidget(sync_group)

        self.measurement_settings_status = QtWidgets.QLabel(
            "Apply: runtime · Save: workstation_data/stereo_distance_settings.json"
        )
        self.measurement_settings_status.setObjectName("statusStrip")
        self.measurement_settings_status.setWordWrap(True)
        settings_layout.addWidget(self.measurement_settings_status)
        settings_actions = QtWidgets.QHBoxLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("measurement.apply_settings", "Apply measurement settings"),
            ("measurement.save_settings", "Save measurement settings"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            settings_actions.addWidget(button)
        settings_actions.addStretch(1)
        settings_layout.addLayout(settings_actions)
        layout.addWidget(settings_group)

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
        layout.addWidget(self.history, 1)

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
        for side in ("left", "right"):
            pane = getattr(panel, f"{side}_pane", None)
            signal = getattr(pane, "frame_changed", None)
            if signal is not None:
                signal.connect(lambda _label, image, metadata, side=side: self.previews[side].set_frame(image, metadata))
            stats_signal = getattr(pane, "stats_changed", None)
            if stats_signal is not None:
                stats_signal.connect(lambda _label, stats, side=side: self.previews[side].set_metadata(stats))

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

    def _refresh_run_controls(self, snapshot: dict[str, RuntimeStatus], panel: object) -> None:
        camera = snapshot.get("camera.left")
        detection = snapshot.get("detection")
        recording = snapshot.get("recording")
        camera_state = camera.state if camera else "STOPPED"
        detection_state = detection.state if detection else "DISABLED"
        recording_state = recording.state if recording else "IDLE"
        recording_locked = recording_state in {"ACTIVE", "FINALIZING"}
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
        self.control_buttons["detection.toggle"].setEnabled(True)
        self.control_buttons["recording.toggle"].setEnabled(
            recording_state not in {"FINALIZING", "RECOVERABLE"}
        )
        has_session_data = bool(getattr(panel, "recording_session_id", "")) or bool(
            getattr(panel, "distance_history", [])
        )
        self.control_buttons["session.save"].setEnabled(
            not recording_locked and (has_session_data or recording_state == "RECOVERABLE")
        )

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        measurement = snapshot.get("measurement")
        measurement_detail = measurement.detail if measurement else "measurement evidence 대기"
        self.result_status.setText(_compact_text(measurement_detail))
        self.result_status.setToolTip(
            f"{measurement_detail}\nclock={measurement.clock}" if measurement else ""
        )
        self._refresh_preview_metadata(panel)
        self._refresh_quality(panel)
        self._refresh_run_controls(snapshot, panel)
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
        self.history_model.removeRows(0, self.history_model.rowCount())
        for row_index, row in enumerate(rows):
            values = (
                getattr(row, "timestamp_s", "--"),
                getattr(row, "forward_m", "--"),
                getattr(row, "right_m", "--"),
                getattr(row, "down_m", "--"),
                getattr(row, "quality", getattr(row, "method", "--")),
            )
            items = []
            for value in values:
                item = QtGui.QStandardItem(str(value))
                item.setEditable(False)
                items.append(item)
            self.history_model.appendRow(items)


class AlignmentWorkspace(QtWidgets.QWidget):
    """Source-backed Pixhawk/GNSS view with phase-one operation facades."""

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_legacy: Callable[[], None],
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
            WorkspaceHeader(
                "Alignment / Pixhawk / GNSS",
                "Telemetry · baseline · attitude",
            )
        )

        evidence_group = QtWidgets.QGroupBox("Transport and readiness")
        evidence_layout = QtWidgets.QGridLayout(evidence_group)
        self.indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (
                ("pixhawk.left", "P1"),
                ("pixhawk.right", "P2"),
                ("gnss", "GNSS / RTK"),
                ("synchronization", "SYNC"),
            )
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            evidence_layout.addWidget(indicator, index // 2, index % 2)
        layout.addWidget(evidence_group)

        attitude_group = QtWidgets.QGroupBox("Attitude samples")
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
        attitude_layout.addWidget(self.attitude_table)
        layout.addWidget(attitude_group)

        gnss_group = QtWidgets.QGroupBox("GPS receiver samples")
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
        gnss_layout.addWidget(self.gnss_table)
        layout.addWidget(gnss_group)

        baseline_group = QtWidgets.QGroupBox("RTK baseline evidence")
        baseline_layout = QtWidgets.QGridLayout(baseline_group)
        self.baseline_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (("north_m", "North"), ("east_m", "East"), ("down_m", "Down"), ("length_m", "Length"), ("state", "Quality"))
        ):
            value = EngineeringValue(label)
            self.baseline_values[key] = value
            baseline_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(baseline_group)

        body_frd_group = QtWidgets.QGroupBox("P1→P2 body-FRD projection")
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
            body_frd_layout.addWidget(value, index // 3, index % 3)
        layout.addWidget(body_frd_group)

        contract = QtWidgets.QLabel(
            "PX4: FRD→NED · RTK raw: NED [N,E,D] · P1→P2: P1 FRD [F,R,D] · GPS: WGS84"
        )
        contract.setObjectName("compactContract")
        layout.addWidget(contract)

        self.operation_tabs = QtWidgets.QTabWidget()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        operation_sections = (
            (
                "Pixhawk",
                (
                    ("pixhawk.toggle_mavlink", "Connect Pixhawk MAVLink"),
                    ("pixhawk.discover_usb", "Scan USB Pixhawks"),
                    ("pixhawk.discover_mavlink_candidates", "Generate MAVLink Candidates"),
                    ("pixhawk.discover_host_ip", "Discover Pixhawk Host IP"),
                    ("alignment.swap_pixhawks", "Swap P1 / P2 Roles"),
                ),
            ),
            (
                "GNSS / RTK",
                (
                    ("gnss.base_discover", "Scan Base GNSS Ports"),
                    ("gnss.base_connect", "Connect Base GNSS"),
                    ("gnss.base_disconnect", "Disconnect Base GNSS"),
                    ("gnss.survey_start", "Start Survey-in"),
                    ("gnss.survey_stop", "Stop Survey-in"),
                    ("gnss.ntrip_start", "Start NTRIP Corrections"),
                    ("gnss.ntrip_stop", "Stop NTRIP Corrections"),
                ),
            ),
            (
                "Alignment",
                (
                    ("alignment.scan_sources", "Scan alignment sources"),
                    ("alignment.start_sampling", "Start baseline sampling"),
                    ("alignment.stop_sampling", "Stop baseline sampling"),
                    ("alignment.save_profile", "Save alignment profile"),
                    ("alignment.load_profile", "Load alignment profile"),
                ),
            ),
        )
        for tab_label, actions in operation_sections:
            page = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(6, 6, 6, 6)
            actions_layout = QtWidgets.QGridLayout()
            for index, (command_id, label) in enumerate(actions):
                button = _command_button(label, command_id, invoke_command)
                self.operation_buttons[command_id] = button
                actions_layout.addWidget(button, index // 3, index % 3)
            page_layout.addLayout(actions_layout)
            page_layout.addStretch(1)
            self.operation_tabs.addTab(page, tab_label)
        layout.addWidget(self.operation_tabs)
        button = QtWidgets.QPushButton("Open advanced sensor diagnostics")
        button.setToolTip(
            "Open PX4 MAG, raw field/source watch, and low-level alignment diagnostics."
        )
        button.clicked.connect(open_legacy)
        layout.addWidget(button, alignment=QtCore.Qt.AlignRight)
        layout.addStretch(1)

    def select_tab_for_context(self, context_id: str) -> None:
        if context_id.startswith("devices.pixhawk"):
            self.operation_tabs.setCurrentIndex(0)
        elif context_id == "devices.gnss":
            self.operation_tabs.setCurrentIndex(1)
        elif context_id == "processing.alignment":
            self.operation_tabs.setCurrentIndex(2)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        evidence = _evidence(self.backend)
        self.attitude_model.removeRows(0, self.attitude_model.rowCount())
        for side in ("left", "right"):
            data = evidence.attitude(side)
            _append_table_row(
                self.attitude_model,
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
        self.gnss_model.removeRows(0, self.gnss_model.rowCount())
        for side in ("left", "right"):
            data = evidence.gps(side)
            _append_table_row(
                self.gnss_model,
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
        recording = snapshot.get("recording")
        recording_locked = recording is not None and recording.state in {"ACTIVE", "FINALIZING"}
        swap_button = self.operation_buttons.get("alignment.swap_pixhawks")
        if swap_button is not None:
            swap_button.setEnabled(not recording_locked)


class CalibrationWorkspace(QtWidgets.QWidget):
    """Calibration evidence with owner-backed capture/run/save/apply commands."""

    def __init__(
        self,
        backend: object,
        open_legacy: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Calibration",
                "Profile · validation",
            )
        )
        profile_group = QtWidgets.QGroupBox("Active profile")
        profile_layout = QtWidgets.QGridLayout(profile_group)
        self.profile_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (("state", "State"), ("profile_path", "Profile"), ("zoom", "Zoom"), ("validation_mode", "Validation"), ("status_label", "Result"))
        ):
            value = EngineeringValue(label)
            self.profile_values[key] = value
            profile_layout.addWidget(value, index // 2, index % 2)
        layout.addWidget(profile_group)

        intrinsic_group = QtWidgets.QGroupBox("Camera intrinsics")
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
        self.validation_value = EngineeringValue("Validation")
        metrics_group = QtWidgets.QGroupBox("Stereo extrinsics / validation")
        metrics_layout = QtWidgets.QVBoxLayout(metrics_group)
        metrics_layout.addWidget(self.extrinsic_value)
        metrics_layout.addWidget(self.validation_value)
        self.calibration_contract = QtWidgets.QLabel()
        self.calibration_contract.setObjectName("compactContract")
        self.calibration_contract.setWordWrap(True)
        self.calibration_contract.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        metrics_layout.addWidget(self.calibration_contract)
        layout.addWidget(metrics_group)

        setup_group = QtWidgets.QGroupBox("Calibration setup")
        setup_layout = QtWidgets.QVBoxLayout(setup_group)
        setup_layout.setContentsMargins(6, 6, 6, 6)
        setup_layout.setSpacing(5)
        setup = _evidence(self.backend).calibration_setup()
        self.setup_widgets: dict[str, QtWidgets.QWidget] = {}

        def setup_spin(
            key: str,
            minimum: int,
            maximum: int,
            suffix: str = "",
        ) -> QtWidgets.QSpinBox:
            widget = QtWidgets.QSpinBox()
            widget.setRange(minimum, maximum)
            value = setup.get(key)
            widget.setValue(int(value if value is not None else minimum))
            widget.setSuffix(suffix)
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
                ("Zoom", setup_spin("zoom_level", 0, 100)),
                ("Inner columns", setup_spin("inner_cols", 2, 30)),
                ("Inner rows", setup_spin("inner_rows", 2, 30)),
                ("Square size", setup_double("square_size_mm", 1.0, 500.0, " mm")),
                ("Minimum images", setup_spin("min_images", 3, 100)),
                ("Validation runs", setup_spin("validation_runs", 1, 20)),
                ("Pairs / run", setup_spin("validation_pair_count", 0, 10000)),
                ("Seed", setup_spin("validation_seed", 0, 2147483647)),
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
        path_form.addWidget(QtWidgets.QLabel("Validation mode"), 1, 0)
        path_form.addWidget(setup_combo("validation_mode"), 1, 1)
        path_form.addWidget(QtWidgets.QLabel("Validation root"), 1, 2)
        path_form.addWidget(setup_text("validation_root", "validation image root"), 1, 3)
        path_form.addWidget(QtWidgets.QLabel("Intrinsics JSON"), 2, 0)
        path_form.addWidget(setup_text("validation_json", "intrinsics.json"), 2, 1, 1, 3)
        setup_layout.addLayout(path_form)
        setup_contract = QtWidgets.QLabel(
            "Square: mm · pixels/intrinsics: OpenCV RDF · stereo T: left-camera→right-camera"
        )
        setup_contract.setObjectName("compactContract")
        setup_layout.addWidget(setup_contract)
        layout.addWidget(setup_group)

        handoff_button = QtWidgets.QPushButton("Open calibration preview / zoom tools")
        handoff_button.setToolTip(
            "Open live checkerboard preview, zoom capture, and per-zoom calibration tools."
        )
        handoff_button.clicked.connect(open_legacy)
        layout.addWidget(handoff_button, alignment=QtCore.Qt.AlignRight)

        self.operation_tabs = QtWidgets.QTabWidget()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        operation_sections = (
            (
                "Setup",
                (
                    ("calibration.apply_setup", "Apply setup"),
                    ("calibration.save_setup", "Save setup"),
                ),
            ),
            (
                "Capture / intrinsics",
                (
                    ("calibration.capture_images", "Capture images"),
                    ("calibration.run_intrinsic", "Run intrinsic calibration"),
                    ("calibration.save_intrinsics", "Save intrinsics"),
                    ("calibration.load_intrinsics", "Load intrinsics"),
                    ("calibration.apply_intrinsics", "Apply intrinsics"),
                ),
            ),
            (
                "Validation",
                (
                    ("calibration.scan_validation", "Scan validation files"),
                    ("calibration.load_validation", "Load validation JSON"),
                    ("calibration.run_validation", "Run validation"),
                    ("calibration.save_validation", "Save validation"),
                    ("calibration.apply_validation", "Apply validation"),
                ),
            ),
        )
        for tab_label, actions in operation_sections:
            page = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(6, 6, 6, 6)
            actions_layout = QtWidgets.QGridLayout()
            for index, (command_id, label) in enumerate(actions):
                button = _command_button(label, command_id, invoke_command)
                self.operation_buttons[command_id] = button
                actions_layout.addWidget(button, index // 3, index % 3)
            page_layout.addLayout(actions_layout)
            page_layout.addStretch(1)
            self.operation_tabs.addTab(page, tab_label)
        layout.addWidget(self.operation_tabs)
        layout.addStretch(1)

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

    def refresh(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        data = _evidence(self.backend).calibration()
        for key, value in self.profile_values.items():
            value.set_value(data.get(key), source=str(data.get("source", "")))
        self.intrinsic_model.removeRows(0, self.intrinsic_model.rowCount())
        for side in ("left", "right"):
            camera = data.get(side, {})
            if not isinstance(camera, dict):
                camera = {}
            grid = "--"
            if camera.get("width_px") is not None and camera.get("height_px") is not None:
                grid = f"{camera['width_px']}×{camera['height_px']}"
            _append_table_row(
                self.intrinsic_model,
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
        extrinsics = data.get("extrinsics", {})
        if not isinstance(extrinsics, dict):
            extrinsics = {}
        translation = extrinsics.get("translation_left_to_right_m")
        translation_text = "--" if not translation else "[" + ", ".join(_display_number(value, digits=6) for value in translation) + "] m"
        self.extrinsic_value.set_value(
            f"{extrinsics.get('state', 'NO DATA')} | T_left_to_right={translation_text} | baseline={_display_number(extrinsics.get('baseline_m'))} m",
            source=str(data.get("source", "")),
            frame=str(extrinsics.get("frame_contract", CAMERA_FRAME)),
        )
        summary = data.get("validation_summary") or {}
        summary_text = "; ".join(f"{key}={value}" for key, value in summary.items()) if isinstance(summary, dict) else str(summary)
        self.validation_value.set_value(
            summary_text or data.get("validation_mode") or "NO DATA",
            source="checkerboard_validation_summary",
            valid=True if summary_text else None,
        )
        self.calibration_contract.setText(
            f"Pixels/intrinsics: {CAMERA_FRAME} · stereo T: left→right · "
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
        self._media_records: dict[str, dict[str, object]] = {}
        self._preview_images: dict[str, QtGui.QImage] = {}

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Advanced owner tools",
            )
        )

        status_group = QtWidgets.QGroupBox("Live owner status")
        status_layout = QtWidgets.QGridLayout(status_group)
        self.indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (
                ("camera.left", "CAM L"),
                ("camera.right", "CAM R"),
                ("pixhawk.left", "P1"),
                ("pixhawk.right", "P2"),
                ("recording", "REC"),
                ("measurement", "MEAS"),
            )
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator, index // 3, index % 3)
        layout.addWidget(status_group)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_wiris_tab(), "WIRIS SDK / Zoom")
        self.tabs.addTab(self._build_px4_and_watch_tab(), "PX4 / Field watch")
        self.tabs.addTab(self._build_preview_tab(), "Calibration preview")
        self.tabs.addTab(self._build_media_tab(), "Session Review / Artifacts")
        layout.addWidget(self.tabs, 1)

        self._connect_preview_signal()
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

        command_group = QtWidgets.QGroupBox("Raw WIRIS SDK command")
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
        self.wiris_send_button = self._button("Send SDK command", "advanced.wiris.send")
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

        self._load_wiris_catalog()
        self.wiris_catalog_combo.currentIndexChanged.connect(self._apply_wiris_catalog_item)

        zoom_group = QtWidgets.QGroupBox("WIRIS camera zoom control")
        zoom_layout = QtWidgets.QGridLayout(zoom_group)
        self.zoom_target_combo = self._combo(
            (("Both cameras", "both"), ("Left camera", "left"), ("Right camera", "right"))
        )
        self.zoom_level_spin = QtWidgets.QSpinBox()
        self.zoom_level_spin.setRange(1, 100)
        self.zoom_level_spin.setValue(10)
        self.zoom_buttons = {
            "in": self._button("Zoom in", "advanced.zoom.in"),
            "out": self._button("Zoom out", "advanced.zoom.out"),
            "stop": self._button("Stop", "advanced.zoom.stop"),
            "set": self._button("Set level", "advanced.zoom.set"),
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
        layout.addStretch(1)
        return page

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

        px4_group = QtWidgets.QGroupBox("PX4 MAG / attitude calibration / Shell")
        px4_layout = QtWidgets.QGridLayout(px4_group)
        self.px4_camera_combo = self._combo((("Pixhawk 1 / Left", "left"), ("Pixhawk 2 / Right", "right")))
        self.px4_priority_spins: dict[str, QtWidgets.QSpinBox] = {}
        for row, role in enumerate(("BOARD", "CAN1", "CAN2", "GPS1_MAG", "GPS2_MAG"), start=1):
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, 100)
            spin.setValue({"BOARD": 0, "CAN1": 100, "CAN2": 90, "GPS1_MAG": 0, "GPS2_MAG": 0}[role])
            spin.setSuffix(" prio")
            self.px4_priority_spins[role] = spin
            px4_layout.addWidget(QtWidgets.QLabel(role), row, 2)
            px4_layout.addWidget(spin, row, 3)
        px4_layout.addWidget(QtWidgets.QLabel("Side"), 0, 0)
        px4_layout.addWidget(self.px4_camera_combo, 0, 1)
        px4_layout.addWidget(self._button("Refresh MAG", "advanced.px4.refresh_mag"), 0, 4)
        px4_layout.addWidget(self._button("Refresh attitude calibration", "advanced.px4.refresh_attitude"), 1, 0, 1, 2)
        px4_layout.addWidget(self._button("Open PX4 Shell", "advanced.px4.shell"), 1, 4)
        px4_layout.addWidget(self._button("Apply MAG priorities", "advanced.px4.apply_mag"), 2, 0, 1, 2)
        px4_layout.addWidget(self._button("Save MAG params", "advanced.px4.save_mag"), 3, 0, 1, 2)
        px4_layout.addWidget(self._button("Reboot PX4", "advanced.px4.reboot"), 4, 0, 1, 2)
        self.px4_status_label = QtWidgets.QLabel(
            "MAG: disarmed + PARAM readback · attitude calibration: QGC Sensor Setup"
        )
        self.px4_status_label.setObjectName("statusStrip")
        self.px4_status_label.setWordWrap(True)
        px4_layout.addWidget(self.px4_status_label, 6, 0, 1, 5)
        layout.addWidget(px4_group)

        watch_group = QtWidgets.QGroupBox("MAVLink field watch")
        watch_layout = QtWidgets.QGridLayout(watch_group)
        watch_layout.setColumnStretch(1, 1)
        self.watch_name_edit = QtWidgets.QLineEdit("local_z")
        self.watch_message_edit = QtWidgets.QLineEdit("ATTITUDE")
        self.watch_field_edit = QtWidgets.QLineEdit("z")
        self.watch_remove_combo = QtWidgets.QComboBox()
        watch_layout.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        watch_layout.addWidget(self.watch_name_edit, 0, 1)
        watch_layout.addWidget(QtWidgets.QLabel("MAVLink message"), 1, 0)
        watch_layout.addWidget(self.watch_message_edit, 1, 1)
        watch_layout.addWidget(QtWidgets.QLabel("Field"), 2, 0)
        watch_layout.addWidget(self.watch_field_edit, 2, 1)
        watch_layout.addWidget(self._button("Add MAVLink watch", "advanced.watch.add_mavlink"), 0, 2)
        watch_layout.addWidget(self._button("Discover MAVLink", "advanced.watch.discover_mavlink"), 2, 2)
        watch_layout.addWidget(QtWidgets.QLabel("Remove"), 3, 0)
        watch_layout.addWidget(self.watch_remove_combo, 3, 1)
        watch_layout.addWidget(self._button("Remove watch", "advanced.watch.remove"), 3, 2)
        self.watch_status_label = QtWidgets.QLabel("")
        self.watch_status_label.setObjectName("statusStrip")
        self.watch_status_label.setWordWrap(True)
        watch_layout.addWidget(self.watch_status_label, 4, 0, 1, 3)
        layout.addWidget(watch_group)

        baseline_group = QtWidgets.QGroupBox("MAVLink field reference")
        baseline_layout = QtWidgets.QVBoxLayout(baseline_group)
        baseline_actions = QtWidgets.QHBoxLayout()
        baseline_actions.addWidget(self._button("Scan fields", "advanced.baseline.scan"))
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
        controls.addWidget(self._button("Refresh checkerboard preview", "advanced.preview.refresh"))
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
        self.preview_status_label = QtWidgets.QLabel("Pixels: OpenCV RDF · overlay: checkerboard worker")
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
        selector.addWidget(self._button("Open session folder", "advanced.media.open_session"), 0, 3)
        selector.addWidget(QtWidgets.QLabel("Artifact"), 1, 0)
        selector.addWidget(self.media_artifact_combo, 1, 1)
        selector.addWidget(self._button("Preview CSV", "advanced.media.preview"), 1, 2)
        selector.addWidget(self._button("Open artifact", "advanced.media.open_artifact"), 1, 3)
        selector.addWidget(QtWidgets.QLabel("Video"), 2, 0)
        selector.addWidget(self.media_video_combo, 2, 1)
        selector.addWidget(self._button("Open in system player", "advanced.media.open_video"), 2, 2, 1, 2)
        layout.addLayout(selector)
        self.media_status_label = QtWidgets.QLabel(
            "Read-only · saved fields/timestamps unchanged"
        )
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
        actions.addWidget(self._button("Save / generate session artifacts", "session.save"))
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
        return {role: int(spin.value()) for role, spin in self.px4_priority_spins.items()}

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

    def refresh_media(self) -> None:
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
        if signature != self._media_signature or not self.media_session_combo.count():
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
        self._refresh_media_artifacts()
        recording = _evidence(self.backend).recording()
        self.media_video_combo.blockSignals(True)
        try:
            self.media_video_combo.clear()
            paths = recording.get("video_paths", {})
            if isinstance(paths, dict):
                for side, path in sorted(paths.items()):
                    self.media_video_combo.addItem(f"{side}: {path}", str(path))
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
        self.media_status_label.setText(_compact_text(media_detail))
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

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        self._refresh_custom_watch_selector()
        self.refresh_media()


class RecordingWorkspace(QtWidgets.QWidget):
    """Session/review/export evidence without a second recording owner."""

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_legacy: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.invoke_command = invoke_command
        self.session_records: dict[str, dict[str, object]] = {}
        self._session_catalog_signature: tuple[object, ...] = ()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader(
                "Recording / Review / Export",
                "Session · preflight · artifacts",
            )
        )
        status_group = QtWidgets.QGroupBox("Session status")
        status_layout = QtWidgets.QGridLayout(status_group)
        self.indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (("recording", "Recording"), ("measurement", "Measurement"), ("camera.left", "CAM L"), ("fps", "FPS"))
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator, index // 2, index % 2)
        layout.addWidget(status_group)

        protocol_group = QtWidgets.QGroupBox("Experiment preflight (advisory)")
        protocol_layout = QtWidgets.QVBoxLayout(protocol_group)
        protocol_layout.setContentsMargins(6, 6, 6, 6)
        self.protocol_state_label = QtWidgets.QLabel("BLOCKED · evidence waiting")
        self.protocol_state_label.setObjectName("protocolState")
        self.protocol_state_label.setWordWrap(True)
        protocol_layout.addWidget(self.protocol_state_label)
        self.protocol_check_labels: dict[str, QtWidgets.QLabel] = {}
        protocol_checks_layout = QtWidgets.QGridLayout()
        protocol_checks_layout.setContentsMargins(0, 0, 0, 0)
        for index in range(6):
            label = QtWidgets.QLabel("--")
            label.setObjectName("protocolCheck")
            label.setWordWrap(True)
            self.protocol_check_labels[str(index)] = label
            protocol_checks_layout.addWidget(label, index // 2, index % 2)
        protocol_layout.addLayout(protocol_checks_layout)
        layout.addWidget(protocol_group)

        session_group = QtWidgets.QGroupBox("Current session")
        session_layout = QtWidgets.QGridLayout(session_group)
        self.session_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (("state", "State"), ("session_id", "Session ID"), ("output_dir", "Output"), ("elapsed_s", "Elapsed"), ("status_label", "Finalization"))
        ):
            value = EngineeringValue(label)
            self.session_values[key] = value
            session_layout.addWidget(value, index // 2, index % 2)
        layout.addWidget(session_group)

        data_group = QtWidgets.QGroupBox("Saved artifacts and sample counts")
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
        self.artifact_model = _table_model(self, ("Artifact", "Output", "Source"))
        self.artifact_table = QtWidgets.QTableView()
        self.artifact_table.setModel(self.artifact_model)
        self.artifact_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.artifact_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.artifact_table.verticalHeader().setVisible(False)
        self.artifact_table.setMaximumHeight(150)
        self.artifact_table.horizontalHeader().setStretchLastSection(True)
        data_layout.addWidget(self.artifact_table)
        layout.addWidget(data_group)

        review_group = QtWidgets.QGroupBox("Session review / export")
        review_layout = QtWidgets.QVBoxLayout(review_group)
        review_layout.setContentsMargins(6, 6, 6, 6)
        review_layout.setSpacing(5)
        selector_layout = QtWidgets.QGridLayout()
        selector_layout.setHorizontalSpacing(8)
        selector_layout.setColumnStretch(1, 1)
        self.session_selector = QtWidgets.QComboBox()
        self.session_selector.setObjectName("sessionReviewSelector")
        self.session_selector.currentIndexChanged.connect(lambda _index: self._refresh_review_artifacts())
        self.refresh_sessions_button = QtWidgets.QPushButton("Refresh sessions")
        self.refresh_sessions_button.clicked.connect(self.refresh_sessions)
        self.open_session_button = QtWidgets.QPushButton("Open session folder")
        self.open_session_button.clicked.connect(self._open_selected_session)
        selector_layout.addWidget(QtWidgets.QLabel("Session"), 0, 0)
        selector_layout.addWidget(self.session_selector, 0, 1)
        selector_layout.addWidget(self.refresh_sessions_button, 0, 2)
        selector_layout.addWidget(self.open_session_button, 0, 3)
        self.artifact_selector = QtWidgets.QComboBox()
        self.artifact_selector.setObjectName("sessionArtifactSelector")
        self.artifact_selector.currentIndexChanged.connect(lambda _index: self._preview_selected_artifact())
        self.preview_artifact_button = QtWidgets.QPushButton("Preview CSV")
        self.preview_artifact_button.clicked.connect(self._preview_selected_artifact)
        self.open_artifact_button = QtWidgets.QPushButton("Open artifact")
        self.open_artifact_button.clicked.connect(self._open_selected_artifact)
        selector_layout.addWidget(QtWidgets.QLabel("Artifact"), 1, 0)
        selector_layout.addWidget(self.artifact_selector, 1, 1)
        selector_layout.addWidget(self.preview_artifact_button, 1, 2)
        selector_layout.addWidget(self.open_artifact_button, 1, 3)
        review_layout.addLayout(selector_layout)
        self.review_status_label = QtWidgets.QLabel(
            "Read-only · saved fields/timestamps unchanged · no NED/ENU conversion"
        )
        self.review_status_label.setObjectName("statusStrip")
        self.review_status_label.setWordWrap(True)
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
            save_button = _command_button("Save current session", "session.save", invoke_command)
            self.operation_buttons["session.save"] = save_button
            review_actions.addWidget(save_button)
        review_actions.addStretch(1)
        review_layout.addLayout(review_actions)
        layout.addWidget(review_group)

        self.contract_label = QtWidgets.QLabel("Session clock: monotonic + wall ID")
        self.contract_label.setObjectName("compactContract")
        layout.addWidget(self.contract_label)
        button = QtWidgets.QPushButton("Open playback / export tools")
        button.setToolTip(
            "Open the Workstation playback, workbook/export, and media review workspace."
        )
        button.clicked.connect(open_legacy)
        layout.addWidget(button, alignment=QtCore.Qt.AlignRight)
        layout.addStretch(1)
        self.refresh_sessions()

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        self._refresh_protocol(snapshot)
        data = _evidence(self.backend).recording()
        for key, value in self.session_values.items():
            raw = data.get(key)
            if key == "elapsed_s" and raw not in (None, ""):
                raw = f"{float(raw):.1f} s"
            value.set_value(raw, source=str(data.get("source", "")))
        counts = data.get("sample_counts", {})
        for key, value in self.count_values.items():
            value.set_value(counts.get(key, 0) if isinstance(counts, dict) else "--", source="session spool rows")
        self.artifact_model.removeRows(0, self.artifact_model.rowCount())
        output_dir = str(data.get("output_dir") or "")
        paths = data.get("video_paths", {})
        if isinstance(paths, dict):
            for side, path in sorted(paths.items()):
                _append_table_row(self.artifact_model, (f"{side} video", path or "not prepared", "video_recording_paths"))
        for artifact in data.get("artifacts", ()) or ():
            _append_table_row(self.artifact_model, (artifact, f"{output_dir}/{artifact}" if output_dir else "--", "session directory"))
        if not self.artifact_model.rowCount():
            _append_table_row(self.artifact_model, ("--", "No finalized artifacts", "recording session evidence"))
        self.refresh_sessions()

    def _refresh_protocol(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        settings = getattr(getattr(self.backend, "panel", self.backend), "settings", None)
        measurement = evidence.measurement_settings()
        readiness = evaluate_recording_protocol(
            snapshot,
            distance_mode=str(measurement.get("distance_mode", "pixhawk") or "pixhawk"),
            calibration=evidence.calibration(),
            baseline=evidence.baseline(),
            require_rtk_fixed=bool(getattr(settings, "require_rtk_fixed", False)),
        )
        self.protocol_state_label.setText(
            f"{readiness.state} · {len(readiness.blocking)} blocking prerequisite(s) · advisory only"
        )
        state_style = (
            "background:#123d2a; color:#b8f5d0; border:1px solid #2e8b57; padding:5px;"
            if readiness.state == "READY"
            else "background:#4a241f; color:#ffd1c8; border:1px solid #c9574a; padding:5px;"
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
                "color:#b8f5d0;" if check.passed else "color:#ffd1c8;"
            )

    def refresh_sessions(self) -> None:
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
            self.review_status_label.setText("열 세션이 없습니다.")
            return
        self._open_path(Path(str(record.get("path") or "")))

    def _open_selected_artifact(self) -> None:
        path = self._selected_artifact_path()
        if path is None:
            self.review_status_label.setText("열 artifact가 없습니다.")
            return
        self._open_path(path)

    def _open_path(self, path: Path) -> None:
        if not path.exists():
            self.review_status_label.setText(f"경로가 없습니다: {path}")
            return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))
        self.review_status_label.setText(f"열기 요청: {path}")

    def _preview_selected_artifact(self) -> None:
        record = self._selected_record()
        artifact = self._selected_artifact_name()
        if record is None or not artifact:
            self.review_model.clear()
            self.review_status_label.setText("CSV preview를 위해 세션과 artifact를 선택하세요.")
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
            f"{artifact}: {len(rows)}개 행 · {data.get('time_axis', 'stored time axis')} · "
            "fields unchanged · no NED/ENU conversion"
            f"{failure_note}"
        )
        self.review_status_label.setText(_compact_text(review_detail))
        self.review_status_label.setToolTip(
            f"{review_detail}\n{data.get('coordinate_contract', 'stored fields preserved')}"
        )


class LegacyHandoffWorkspace(QtWidgets.QWidget):
    def __init__(
        self,
        title: str,
        detail: str,
        open_legacy: Callable[[], None],
        *,
        status_model: StatusModel | None = None,
        status_keys: tuple[tuple[str, str], ...] = (),
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.status_model = status_model
        self.status_keys = status_keys
        self.indicators: dict[str, StatusIndicator] = {}
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(WorkspaceHeader(title, detail))
        if status_keys:
            evidence_group = QtWidgets.QGroupBox("Current evidence")
            evidence_layout = QtWidgets.QGridLayout(evidence_group)
            evidence_layout.setContentsMargins(6, 6, 6, 6)
            for index, (key, label) in enumerate(status_keys):
                indicator = StatusIndicator(label)
                self.indicators[key] = indicator
                evidence_layout.addWidget(indicator, index // 2, index % 2)
            layout.addWidget(evidence_group)
        notice = EmptyStateView(
            "Legacy workflow remains the operational owner",
            "This context is kept in the existing panel until its adapter and lifecycle contract are verified.",
        )
        layout.addWidget(notice, 1)
        button = QtWidgets.QPushButton("Open legacy workflow")
        button.clicked.connect(open_legacy)
        layout.addWidget(button, alignment=QtCore.Qt.AlignRight)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))


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
        self.tabs.addTab(self.log_view, "Log")
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
        records = list(snapshot.values())
        self.system_model.removeRows(0, self.system_model.rowCount())
        for status in records:
            values = (
                status.key,
                status.state,
                status.severity.value,
                status.source,
                "--" if status.age_s is None else f"{status.age_s:.2f}s",
                status.detail,
            )
            items = []
            for value in values:
                item = QtGui.QStandardItem(str(value))
                item.setEditable(False)
                items.append(item)
            self.system_model.appendRow(items)
        backend = getattr(self.backend, "diagnostic_text", None)
        if callable(backend):
            self.log_view.setPlainText(str(backend()))
            self.log_view.moveCursor(QtGui.QTextCursor.End)
        for view, keys in (
            (self.camera_view, ("camera.left", "camera.right", "detection", "synchronization")),
            (self.pixhawk_view, ("pixhawk.left", "pixhawk.right")),
            (self.gnss_view, ("gnss",)),
            (self.performance_view, ("fps",)),
        ):
            view.setPlainText("\n".join(self._format_status(snapshot.get(key)) for key in keys))
        panel = getattr(self.backend, "panel", self.backend)
        evidence = _evidence(self.backend)
        attitude_lines = ["", "Attitude source projection (orientation only; PX4 body FRD → world NED):"]
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
        self.pixhawk_view.appendPlainText("\n".join(attitude_lines))
        baseline = evidence.baseline()
        gnss_lines = [
            "",
            "Receiver source projection (WGS84 geodetic; not NED baseline):",
        ]
        for side in ("left", "right"):
            data = evidence.gps(side)
            gnss_lines.append(
                f"{side}: {data.get('rtk_label', data.get('state', 'NO DATA'))}; "
                f"fix={data.get('fix_type', '--')} sats={data.get('satellites', '--')} "
                f"eph/epv={_display_number(data.get('eph_m'))}/{_display_number(data.get('epv_m'))} m; "
                f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}; "
                f"lat/lon={_display_number(data.get('latitude_deg'), digits=7)}/"
                f"{_display_number(data.get('longitude_deg'), digits=7)}"
            )
        gnss_lines.extend(
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
        self.gnss_view.appendPlainText("\n".join(gnss_lines))
        config = evidence.detection_config()
        detection_lines = [
            "",
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
        self.camera_view.appendPlainText("\n".join(detection_lines + target_lines))
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
        self.measurement_view.setPlainText("\n".join(lines))

    @staticmethod
    def _format_status(status: RuntimeStatus | None) -> str:
        if status is None:
            return "NO DATA"
        age = "--" if status.age_s is None else f"{status.age_s:.2f}s"
        return (
            f"{status.label} | state={status.state} | severity={status.severity.value} | age={age}\n"
            f"source={status.source} | clock={status.clock}\n"
            f"detail={status.detail}"
        )
