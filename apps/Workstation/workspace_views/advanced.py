"""Advanced camera, PX4, preview and media operation controls."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Callable
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.ui.components import WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus, StatusModel
from .px4_widgets import _build_px4_priority_combo, _px4_priority_combo_value
from .shared import _append_table_row, _command_button, _compact_text, _evidence, _table_model


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
