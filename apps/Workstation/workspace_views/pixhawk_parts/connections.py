"""Endpoint and role controls with identity-bound candidate presentation."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.designer import load_form
from .designer import load_pixhawk_form
from Workstation.application.status_model import RuntimeStatus
from ..shared import _append_table_row, _live_inspection, _table_model

from pathlib import Path
from typing import Sequence
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
        self._controls = load_form("pixhawk_connections.ui", QtWidgets.QWidget())
        controls = self._controls
        self.actions = controls.actionsLayout
        controls.layout().removeItem(self.actions)
        self.actions.setParent(None)
        self.operation_buttons = {}
        for button in controls.findChildren(QtWidgets.QPushButton):
            command = button.property("commandId")
            if command:
                self.operation_buttons[command] = button
                button.setEnabled(invoke_command is not None)
                if invoke_command is not None:
                    button.clicked.connect(lambda _checked=False, command=command: invoke_command(command))
        self.operation_buttons["pixhawk.toggle_mavlink"] = self.operation_buttons["pixhawk.connect_mavlink"]
        for side in ("left", "right"):
            button = getattr(controls, f"{side}_shell")
            button.clicked.connect(lambda _checked=False, side=side: self._open_px4_shell(side))
            self.operation_buttons[f"pixhawk.open_shell.{side}"] = button
        self.transport_combos = {}
        self.radio_device_edits = {}
        self.radio_baud_spins = {}
        self.role_connect_buttons = {}
        self.role_disconnect_buttons = {}
        for side in ("left", "right"):
            combo = getattr(controls, f"{side}_transport")
            for index, value in enumerate(("radio", "ethernet", "usb")):
                combo.setItemData(index, value)
            selected = str(transport_defaults.get(f"{side}_pixhawk_transport", "radio"))
            combo.setCurrentIndex(max(0, combo.findData(selected)))
            serial_key = "bootstrap_url" if selected == "usb" else "radio_device"
            serial_baud_key = "bootstrap_baud" if selected == "usb" else "radio_baud"
            device = getattr(controls, f"{side}_device")
            device.setText(str(transport_defaults.get(f"{side}_pixhawk_{serial_key}", "") or ""))
            baud = getattr(controls, f"{side}_baud")
            baud.setValue(int(transport_defaults.get(f"{side}_pixhawk_{serial_baud_key}", 57_600)))
            connect_button = getattr(controls, f"{side}_connect")
            disconnect_button = getattr(controls, f"{side}_disconnect")
            combo.currentIndexChanged.connect(lambda _index, side=side, combo=combo: self._set_transport(side, combo))
            device.editingFinished.connect(lambda side=side, device=device, baud=baud: self._apply_radio_endpoint(side, device, baud))
            baud.editingFinished.connect(lambda side=side, device=device, baud=baud: self._apply_radio_endpoint(side, device, baud))
            connect_button.clicked.connect(lambda _checked=False, side=side: self._connect_role(side))
            disconnect_button.clicked.connect(lambda _checked=False, side=side: self._disconnect_role(side))
            self.transport_combos[side] = combo
            self.radio_device_edits[side] = device
            self.radio_baud_spins[side] = baud
            self.role_connect_buttons[side] = connect_button
            self.role_disconnect_buttons[side] = disconnect_button
        controls.droneHint.setObjectName("statusStrip")
        self.transport_group = controls.transport_group

        self._designer_form = load_pixhawk_form("pixhawk_connection_evidence.ui")
        form = self._designer_form
        self.indicators = {'pixhawk.left': form.indicators_pixhawk_left, 'pixhawk.right': form.indicators_pixhawk_right, 'pixhawk.drone': form.indicators_pixhawk_drone}
        self.status_group = form.status_group
        self.recovery_values = {'left': {'usb': form.recovery_values_left_usb, 'baud': form.recovery_values_left_baud, 'recovery': form.recovery_values_left_recovery, 'telemetry': form.recovery_values_left_telemetry}, 'right': {'usb': form.recovery_values_right_usb, 'baud': form.recovery_values_right_baud, 'recovery': form.recovery_values_right_recovery, 'telemetry': form.recovery_values_right_telemetry}}
        self.recovery_buttons = {'left': form.recovery_buttons_left, 'right': form.recovery_buttons_right}
        self.recovery_feedback = form.recovery_feedback
        self.network_assignment_values = {'left': {'state': form.network_assignment_values_left_state, 'nic': form.network_assignment_values_left_nic, 'local': form.network_assignment_values_left_local, 'endpoint': form.network_assignment_values_left_endpoint, 'route': form.network_assignment_values_left_route}, 'right': {'state': form.network_assignment_values_right_state, 'nic': form.network_assignment_values_right_nic, 'local': form.network_assignment_values_right_local, 'endpoint': form.network_assignment_values_right_endpoint, 'route': form.network_assignment_values_right_route}}
        self.discovery_status = form.discovery_status
        self.candidate_table = form.candidate_table
        self.candidate_buttons = {'left': form.candidate_buttons_left, 'right': form.candidate_buttons_right, 'host': form.candidate_buttons_host}
        self.candidate_feedback = form.candidate_feedback
        self.candidate_group = form.candidate_group
        self.recovery_group = form.recovery_group
        self.candidate_model = _table_model(self, ('Source', 'NIC / Device', 'Link / Kind', 'Evidence', 'Role'))
        self.candidate_table.setModel(self.candidate_model)
        header = form.candidate_table.horizontalHeader()
        for column, (width, mode) in enumerate(((90, 0), (280, 0), (110, 0), (260, 0), (170, 0))):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeMode(mode))
            header.resizeSection(column, width)
        for key, indicator in self.indicators.items():
            indicator.setFixedWidth(PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX)
        for side, values in self.network_assignment_values.items():
            for key, value in values.items():
                value.set_value("LEGACY AUTO ROUTE" if key == "state" else "--", source="no explicit NIC assignment")
        for button in self.recovery_buttons.values():
            command = button.property("commandId")
            button.setEnabled(invoke_command is not None)
            if invoke_command is not None:
                button.clicked.connect(lambda _checked=False, command=command: invoke_command(command))
        self.candidate_table.selectionModel().selectionChanged.connect(self._refresh_candidate_actions)
        self.candidate_table.clicked.connect(self._inspect_candidate_row)
        for target, button in self.candidate_buttons.items():
            button.clicked.connect(lambda _checked=False, target=target: self._apply_candidate(target))
        self._candidate_signature: tuple[tuple[object, ...], ...] = ()
        self._candidate_rows: tuple[dict[str, object], ...] = ()
        self._discovery_revision = -1
        self._recording_locked = False
        self._network_role_locked = False
        self._invoke_command_available = invoke_command is not None
        self._invoke_command = invoke_command

        self.operation_buttons.update(
            {
                "pixhawk.recover_eth_mavlink.p1": self.recovery_buttons["left"],
                "pixhawk.recover_eth_mavlink.p2": self.recovery_buttons["right"],
            }
        )

        if isinstance(parent, QtWidgets.QWidget):
            for group in (self.transport_group, self.status_group, self.candidate_group, self.recovery_group):
                group.setParent(parent)

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
