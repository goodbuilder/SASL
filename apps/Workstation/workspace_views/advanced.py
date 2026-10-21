"""Advanced camera, PX4, preview and media operation controls."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Callable
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.ui.designer import load_form
from Workstation.application.status_model import RuntimeStatus, StatusModel
from .px4_widgets import _set_px4_priority_combo_value, _px4_priority_combo_value
from Workstation.workstation_core.px4_mag_manager import MAG_PRIORITY_LEVELS, mag_priority_display
from .shared import _append_table_row, _compact_text, _evidence, _table_model


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

        load_form("advanced.ui", self)
        for button in self.findChildren(QtWidgets.QPushButton):
            command = button.property("commandId")
            if command:
                self.operation_buttons[command] = button
                button.setEnabled(invoke_command is not None)
                if invoke_command is not None:
                    button.clicked.connect(lambda _checked=False, command=command: invoke_command(command))
        for combo, values in (
            (self.wiris_target_combo, ("both", "left", "right")),
            (self.zoom_target_combo, ("both", "left", "right")),
            (self.wiris_terminator_combo, ("\n", "\0")),
            (self.px4_camera_combo, ("left", "right", "drone")),
        ):
            for index, value in enumerate(values):
                combo.setItemData(index, value)
        self.zoom_buttons = {key: getattr(self, f"zoom_{key}_button") for key in ("in", "out", "stop", "set")}
        self.wiris_response_clear_button.clicked.connect(self.wiris_response_log.clear)
        self.wiris_catalog_combo.currentIndexChanged.connect(self._apply_wiris_catalog_item)
        self._load_wiris_catalog()
        # Priority values and descriptions follow the PX4 semantic contract.
        self.px4_priority_spins = {}
        for role, value in {"BOARD": 0, "CAN1": 100, "CAN2": 90, "GPS1_MAG": 0, "GPS2_MAG": 0}.items():
            combo = getattr(self, f"priority_{role}")
            for raw, label in MAG_PRIORITY_LEVELS:
                combo.addItem(label, raw)
            _set_px4_priority_combo_value(combo, value)
            combo.currentIndexChanged.connect(lambda _index, combo=combo: combo.setToolTip(
                f"PX4 priority: {mag_priority_display(combo.currentData())}"
            ))
            self.px4_priority_spins[role] = combo
        self.manual_eth_button.clicked.connect(self._apply_manual_px4_ethernet_setup)
        self.browse_button.clicked.connect(self._browse_px4_transfer_file)
        self.restart_button.clicked.connect(lambda: self._start_px4_transfer(resume=False, firmware=False))
        self.resume_button.clicked.connect(lambda: self._start_px4_transfer(resume=True, firmware=False))
        self.download_button.clicked.connect(lambda: self._start_px4_download(resume=False))
        self.resume_download_button.clicked.connect(lambda: self._start_px4_download(resume=True))
        self.stage_button.clicked.connect(lambda: self._start_px4_transfer(resume=False, firmware=True))
        self.cancel_button.clicked.connect(self._cancel_px4_transfer)
        self.flash_button.clicked.connect(self._start_px4_usb_flash)
        self.cancel_flash_button.clicked.connect(self._cancel_px4_usb_flash)
        self.baseline_mavlink_model = _table_model(self, ("Message", "Fields"))
        self.baseline_mavlink_table.setModel(self.baseline_mavlink_model)
        self.baseline_mavlink_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.baseline_mavlink_table.verticalHeader().setVisible(False)
        self.baseline_mavlink_table.horizontalHeader().setStretchLastSection(True)
        self.preview_labels = {side: getattr(self, f"{side}_preview") for side in ("left", "right")}
        for index, label in enumerate(self.preview_labels.values()):
            label.setObjectName("calibrationPreview")
            self.previewSplitter.setStretchFactor(index, 1)
        for label in self.findChildren(QtWidgets.QLabel):
            if label.objectName().endswith("status_label"):
                label.setObjectName("statusStrip")

        self._connect_preview_signal()
        self._connect_wiris_response_signal()
        self._refresh_custom_watch_selector()
        self._refresh_media_video_paths()




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
