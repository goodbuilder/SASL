"""Drone telemetry connection, position and RTCM presentation."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Callable
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.workstation_core.settings import PIXHAWK_SERIAL_TRANSPORTS, normalize_pixhawk_transport
from Workstation.ui.components import WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus
from .shared import _command_button, _display_number, _sync_table_rows, _table_model


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
