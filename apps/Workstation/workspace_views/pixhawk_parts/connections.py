"""Endpoint and role controls with identity-bound candidate presentation."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import EngineeringValue
from Workstation.application.status_model import RuntimeStatus
from ..shared import _append_table_row, _command_button, _live_inspection, _table_model

from pathlib import Path
from typing import Sequence
from Workstation.ui.components import StatusIndicator
from Workstation.workstation_core.network import candidate_network_role_labels
from Workstation.workstation_core.pixhawk_link.policy import MAVLINK_EXPECTED_SYSTEM_IDS
from Workstation.workstation_core.settings import PIXHAWK_SERIAL_TRANSPORTS, normalize_pixhawk_transport
from ..network_helpers import _apply_network_role_discovery_status, _network_role_column_text
from .contracts import ConnectionCommands, ConnectionInputs

PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX = 420
_PIXHAWK_SIDE_LABELS = (("left", "P1"), ("right", "P2"), ("drone", "Drone"))
_PIXHAWK_SYSTEM_ID_LABELS = {MAVLINK_EXPECTED_SYSTEM_IDS[side]: label for side, label in _PIXHAWK_SIDE_LABELS}


class ConnectionView(QtCore.QObject):
    inspection_requested = QtCore.pyqtSignal(object)
    refresh_requested = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        read_inputs: Callable[[], ConnectionInputs],
        commands: Callable[[], ConnectionCommands],
        transport_defaults: Mapping[str, object],
        invoke_command=None,
        parent=None,
    ):
        super().__init__(parent)
        if not callable(read_inputs) or not callable(commands):
            raise TypeError("Connection view requires input and command readers")
        self._read_inputs = read_inputs
        self._commands = commands
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
        self.actions = actions

        transport_group = QtWidgets.QGroupBox("Board transport · explicit Connect only")
        transport_layout = QtWidgets.QGridLayout(transport_group)
        self.transport_combos: dict[str, QtWidgets.QComboBox] = {}
        self.radio_device_edits: dict[str, QtWidgets.QLineEdit] = {}
        self.radio_baud_spins: dict[str, QtWidgets.QSpinBox] = {}
        self.role_connect_buttons: dict[str, QtWidgets.QPushButton] = {}
        self.role_disconnect_buttons: dict[str, QtWidgets.QPushButton] = {}
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
            device = QtWidgets.QLineEdit(str(transport_defaults.get(f"{side}_pixhawk_{serial_key}", "") or ""))
            device.setPlaceholderText("/dev/serial/by-id/...")
            baud = QtWidgets.QSpinBox()
            baud.setRange(1_200, 4_000_000)
            baud.setValue(int(transport_defaults.get(f"{side}_pixhawk_{serial_baud_key}", 57_600)))
            connect_button = QtWidgets.QPushButton("Connect")
            disconnect_button = QtWidgets.QPushButton("Disconnect")
            combo.currentIndexChanged.connect(lambda _index, side=side, combo=combo: self._set_transport(side, combo))
            device.editingFinished.connect(
                lambda side=side, device=device, baud=baud: self._apply_radio_endpoint(side, device, baud)
            )
            baud.editingFinished.connect(
                lambda side=side, device=device, baud=baud: self._apply_radio_endpoint(side, device, baud)
            )
            connect_button.clicked.connect(lambda _checked=False, side=side: self._connect_role(side))
            disconnect_button.clicked.connect(lambda _checked=False, side=side: self._disconnect_role(side))
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
        drone_hint = QtWidgets.QLabel("Drone / sysid 3 transport와 Connect는 Devices → Telemetry에서 관리합니다.")
        drone_hint.setObjectName("statusStrip")
        drone_hint.setWordWrap(True)
        transport_layout.addWidget(drone_hint, transport_layout.rowCount(), 0, 1, 6)
        self.transport_group = transport_group

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
        self.status_group = status_group

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
        for target, label in (
            ("left", "Assign P1"),
            ("right", "Assign P2"),
            ("host", "Assign Host"),
        ):
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
        self.candidate_group = candidate_group

        self.recovery_group = recovery_group
        self.operation_buttons.update(
            {
                "pixhawk.recover_eth_mavlink.p1": self.recovery_buttons["left"],
                "pixhawk.recover_eth_mavlink.p2": self.recovery_buttons["right"],
            }
        )

        if isinstance(parent, QtWidgets.QWidget):
            for group in (self.transport_group, self.status_group, self.candidate_group, self.recovery_group):
                group.setParent(parent)
            for button in self.operation_buttons.values():
                button.setParent(parent)

    def _set_transport(self, side: str, combo: QtWidgets.QComboBox) -> None:
        transport = normalize_pixhawk_transport(combo.currentData())
        serial_selected = transport in PIXHAWK_SERIAL_TRANSPORTS
        self.radio_device_edits[side].setEnabled(serial_selected)
        self.radio_baud_spins[side].setEnabled(serial_selected)
        setter = self._commands().set_transport
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
        setter = self._commands().configure_usb if transport == "usb" else self._commands().configure_radio
        if callable(setter):
            try:
                setter(side, device.text().strip(), int(baud.value()))
            except Exception as exc:
                self.candidate_feedback.setText(f"{side} {transport.upper()} endpoint 적용 실패: {exc}")

    def _connect_role(self, side: str) -> None:
        connector = self._commands().connect
        if callable(connector):
            try:
                connector(side)
            except Exception as exc:
                self.candidate_feedback.setText(f"{side} Connect 실패: {exc}")

    def _disconnect_role(self, side: str) -> None:
        disconnect = self._commands().disconnect
        if callable(disconnect):
            try:
                disconnect(side)
            except Exception as exc:
                self.candidate_feedback.setText(f"{side} Disconnect 실패: {exc}")

    def _open_px4_shell(self, side: str) -> None:
        opener = self._commands().open_shell
        if callable(opener):
            opener(str(side))

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        inputs = self._read_inputs()
        for side in ("left", "right"):
            transport = inputs.transports[side].transport
            combo = self.transport_combos[side]
            index = combo.findData(transport)
            if index >= 0 and combo.currentIndex() != index and not combo.hasFocus():
                blocked = combo.blockSignals(True)
                combo.setCurrentIndex(index)
                combo.blockSignals(blocked)
            # Radio and USB save separate endpoints; show the one the selected
            # transport will actually open.
            device_text = inputs.transports[side].device
            baud_value = inputs.transports[side].baud
            edit = self.radio_device_edits[side]
            if not edit.hasFocus() and edit.text() != device_text:
                edit.setText(device_text)
            baud = self.radio_baud_spins[side]
            if not baud.hasFocus() and baud.value() != baud_value:
                blocked = baud.blockSignals(True)
                baud.setValue(baud_value)
                baud.blockSignals(blocked)
        connection = inputs.connection
        network_role_active = bool(connection.get("network_role_active", False))
        self._network_role_locked = network_role_active
        role_state, role_detail = _apply_network_role_discovery_status(self.discovery_status, connection)
        candidates = connection.get("candidates")
        candidates = (
            tuple(row for row in candidates if isinstance(row, dict)) if isinstance(candidates, (tuple, list)) else ()
        )
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
            endpoint_text = inputs.endpoints[side]
            detail = str(assignment.get("detail") or "")
            valid = True if state in {"ASSIGNED", "ROUTE OK"} else False if state not in {"LEGACY AUTO ROUTE"} else None
            values = self.network_assignment_values[side]
            source = detail or f"{label} operator assignment"
            values["state"].set_value(state, source=source, valid=valid)
            values["nic"].set_value(interface_name or "--", source=source)
            values["local"].set_value(local_address or "--", source=source)
            values["endpoint"].set_value(endpoint_text or "--", source=source)
            values["route"].set_value(detail or "--", source=source, valid=valid)

        # Role labels resolve serial symlinks; compute each once per tick and
        # reuse it for both the change check and the rebuild.
        saved_serial_keys = inputs.saved_serial_keys
        role_labels = tuple(
            self._candidate_role_label(row, network_roles, saved_serial_keys=saved_serial_keys) for row in candidates
        )
        candidate_signature = tuple(
            (
                *tuple(
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
                ),
                role_label,
            )
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
        source_connection_allowed = not self._recording_locked or bool(inputs.automatic_recording)
        connected_sides = set(inputs.open_sides)
        connection_in_progress = inputs.connection_in_progress
        status_connected_sides = {
            side
            for side in ("left", "right", "drone")
            if getattr(snapshot.get(f"pixhawk.{side}"), "state", "").upper() in {"LIVE", "CONNECTED", "STARTING"}
        }
        connected_sides.update(status_connected_sides)
        status_connected = bool(status_connected_sides)
        connection_active = bool(connected_sides) or connection_in_progress or status_connected
        configured_sides = set(inputs.configured_sides)
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
                source_connection_allowed and not connection_in_progress and not side_connected
            )
            self.role_disconnect_buttons[side].setEnabled(source_connection_allowed and side_connected)
            serial_selected = (
                normalize_pixhawk_transport(self.transport_combos[side].currentData()) in PIXHAWK_SERIAL_TRANSPORTS
            )
            self.transport_combos[side].setEnabled(not side_connected)
            self.radio_device_edits[side].setEnabled(serial_selected and not side_connected)
            self.radio_baud_spins[side].setEnabled(serial_selected and not side_connected)
        for side in ("left", "right"):
            self.operation_buttons[f"pixhawk.open_shell.{side}"].setEnabled(
                side in connected_sides and not self._recording_locked
            )
        reset_level_button = self.operation_buttons.get("pixhawk.reset_level_calibration")
        if reset_level_button is not None:
            reset_level_button.setEnabled(not self._recording_locked)
        recovery_feedback: list[str] = []
        for side, label in (("left", "P1"), ("right", "P2")):
            recovery = inputs.recovery[side]
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
                and str(getattr(snapshot.get(f"pixhawk.{side}"), "state", "")).upper() == "ETH MAVLINK RX STALLED"
            )
        self.recovery_feedback.setText(" | ".join(recovery_feedback))
        for command_id in ("pixhawk.discover_usb", "pixhawk.discover_mavlink_candidates"):
            self.operation_buttons[command_id].setEnabled(not discovery_active and not network_role_active)
        self._refresh_candidate_actions()

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
    def _candidate_role_label(
        cls,
        candidate: Mapping[str, object],
        network_roles: Mapping[str, object],
        *,
        saved_serial_keys: Sequence[tuple[str, str]] = (),
    ) -> str:
        if str(candidate.get("source") or "") == "network":
            return _network_role_column_text(candidate_network_role_labels(candidate, network_roles))

        if str(candidate.get("kind") or "") != "USB":
            return "--"
        candidate_keys = {
            cls._serial_endpoint_key(candidate.get("value")),
            cls._serial_endpoint_key(candidate.get("resolved_device")),
        }
        candidate_keys.discard("")
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
        endpoint_enabled = source == "network" or (source == "mavlink" and kind in {"USB", "MAVLink"})
        host_enabled = source == "pc"
        not_locked = not self._recording_locked and not self._network_role_locked
        self.candidate_buttons["left"].setEnabled(not_locked and endpoint_enabled)
        self.candidate_buttons["right"].setEnabled(not_locked and endpoint_enabled)
        self.candidate_buttons["host"].setEnabled(not_locked and host_enabled)

    def _apply_candidate(self, target: str) -> None:
        row = self._selected_candidate_row()
        if not 0 <= row < len(self._candidate_rows):
            self.candidate_feedback.setText("ERROR: select a compatible endpoint")
            return
        candidate = self._candidate_rows[row]
        source = str(candidate.get("source", ""))
        if source == "network":
            method = self._commands().assign_network
            args = (str(candidate.get("candidate_id", "")), target)
        else:
            method = self._commands().apply_candidate
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
        self.refresh_requested.emit()

    def _inspect_candidate_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        self.inspection_requested.emit(
            _live_inspection(
                self._candidate_inspection(row),
                lambda row=row: self._candidate_inspection(row),
            )
        )

    def _candidate_inspection(self, row: int) -> dict[str, object]:
        candidates = self._read_inputs().connection.get("candidates")
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
