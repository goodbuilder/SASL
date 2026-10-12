"""Pixhawk workspace composition and legacy panel-to-display adapters.

Feature parts own widgets/models/presentation state. This outer edge alone
reads legacy panel fields and exposes the existing workspace names to callers.
Device decisions, numerical projection and resource lifetime stay in their
existing runtime/evidence producers.
"""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Callable, Sequence
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus, StatusModel
from Workstation.workstation_core.settings import normalize_pixhawk_gnss_source
from Workstation.workstation_core.px4_mag_manager import build_mag_selection_preview
from .shared import _evidence
from .pixhawk_parts.connections import (
    ConnectionView,
    PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX,
    _PIXHAWK_SIDE_LABELS,
    _PIXHAWK_SYSTEM_ID_LABELS,
)
from .pixhawk_parts.attitude import AttitudeView
from .pixhawk_parts.gnss import GnssView
from .pixhawk_parts.magnetometers import MagnetometerView
from .pixhawk_parts.geometry import GeometryView
from .pixhawk_parts.contracts import ConnectionCommands, ConnectionInputs, MagnetometerInputs, TransportDisplay
from .pixhawk_parts.formatting import (
    _display_signed_number,
    _display_compass_deg,
    _display_signed_deg,
    _display_magnetometer_px4_yaw,
    _display_magnetometer_corrected_yaw,
    _magnetometer_yaw_summary,
    _format_px4_parameter_value,
)


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


def _selected_gnss_source(panel: object, side: str) -> str:
    reader = getattr(panel, "_pixhawk_gnss_source", None)
    if callable(reader):
        return normalize_pixhawk_gnss_source(reader(side))
    settings = getattr(panel, "settings", None)
    key = "pixhawk_left_gnss_source" if side == "left" else "pixhawk_right_gnss_source"
    return normalize_pixhawk_gnss_source(getattr(settings, key, "GPS1"))


def _saved_serial_endpoint_keys(panel: object) -> tuple[tuple[str, str], ...]:
    return tuple(
        (label, ConnectionView._serial_endpoint_key(getattr(panel, f"{side}_pixhawk_radio_device", "")))
        for side, label in _PIXHAWK_SIDE_LABELS
    )


def _view_alias(owner: str, name: str) -> property:
    """Finite outer compatibility aliases; the part keeps the actual state."""
    return property(lambda workspace: getattr(getattr(workspace, owner), name))


class _PixhawkViewSources:
    """Legacy read boundary, never passed to a component as a whole context."""

    def __init__(self, backend):
        self.backend = backend

    @property
    def panel(self):
        return getattr(self.backend, "panel", self.backend)

    def connection_inputs(self) -> ConnectionInputs:
        panel = self.panel
        evidence = _evidence(self.backend)
        transports = {}
        endpoints = {}
        for side in ("left", "right"):
            transport = str(getattr(panel, f"{side}_pixhawk_transport", "radio") or "radio")
            if transport == "usb":
                device = _panel_widget_text(panel, f"{side}_pixhawk_bootstrap_url")
                baud = _panel_widget_value(panel, f"{side}_pixhawk_bootstrap_baud", 57_600)
            else:
                device = str(getattr(panel, f"{side}_pixhawk_radio_device", "") or "")
                baud = int(getattr(panel, f"{side}_pixhawk_radio_baud", 57_600))
            transports[side] = TransportDisplay(transport, device, baud)
            endpoint = getattr(panel, f"{side}_mavlink_url_edit", None)
            endpoints[side] = endpoint.text().strip() if endpoint is not None and hasattr(endpoint, "text") else ""
        opened = getattr(panel, "_mavlink_open_sides", None)
        configured = getattr(panel, "_configured_mavlink_sides", None)
        return ConnectionInputs(
            transports=transports,
            endpoints=endpoints,
            saved_serial_keys=_saved_serial_endpoint_keys(panel),
            connection=evidence.pixhawk_connection_candidates(),
            recovery={side: evidence.pixhawk_recovery(side) for side in ("left", "right")},
            open_sides=frozenset(opened()) if callable(opened) else frozenset(),
            configured_sides=frozenset(configured()) if callable(configured) else frozenset(("left", "right")),
            connection_in_progress=getattr(panel, "pixhawk_connection_thread", None) is not None,
            automatic_recording=bool(getattr(panel, "_automatic_session_recording", False)),
        )

    def connection_commands(self) -> ConnectionCommands:
        backend = self.backend
        opener = getattr(backend, "advanced_open_px4_shell", None)
        if not callable(opener):
            opener = getattr(getattr(backend, "panel", None), "open_px4_shell", None)
        return ConnectionCommands(
            set_transport=getattr(backend, "set_pixhawk_transport", None),
            configure_usb=getattr(backend, "configure_pixhawk_usb", None),
            configure_radio=getattr(backend, "configure_pixhawk_radio", None),
            connect=getattr(backend, "connect_pixhawk_role", None),
            disconnect=getattr(backend, "disconnect_pixhawk_role", None),
            open_shell=opener,
            assign_network=getattr(backend, "pixhawk_assign_network_candidate", None),
            apply_candidate=getattr(backend, "pixhawk_apply_candidate", None),
        )

    def magnetometer_inputs(self) -> MagnetometerInputs:
        panel = self.panel
        opened = getattr(panel, "_mavlink_open_sides", None)
        connected = frozenset(opened()) if callable(opened) else frozenset()
        verified = getattr(panel, "_mavlink_connected_sides", None)
        managers = getattr(panel, "px4_mag_managers", {}) or {}
        return MagnetometerInputs(
            status_cache=getattr(panel, "_px4_mag_status_cache", {}),
            incomplete_cache=getattr(panel, "_px4_mag_status_incomplete", {}),
            attitude_cache=getattr(panel, "_px4_attitude_cal_status_cache", {}),
            rotation_cache=getattr(panel, "_px4_board_rotation_params", {}),
            active_ids=getattr(panel, "latest_vehicle_mag_device_ids", {}),
            connected=connected,
            verified=frozenset(verified()) if callable(verified) else connected,
            parameter_sides=frozenset(getattr(panel, "pixhawk_parameter_threads", {})),
            shell_sides=frozenset(getattr(panel, "pixhawk_shell_threads", {})),
            armed_sides=frozenset(side for side, manager in managers.items() if getattr(manager, "armed", False)),
            parameter_errors=getattr(panel, "_px4_calibration_parameter_error", {}),
            reset_errors=getattr(panel, "_px4_calibration_reset_error", {}),
            now_s=time.monotonic(),
        )

    def selected_source(self, side):
        return _selected_gnss_source(self.panel, side)

    def set_source(self, side, source):
        setter = getattr(self.panel, "set_pixhawk_gnss_source", None)
        if callable(setter):
            setter(side, source)

    def mag_preview(self, side, requested_priorities):
        panel = self.panel
        status_cache = getattr(panel, "_px4_mag_status_cache", {})
        status = status_cache.get(side) if isinstance(status_cache, Mapping) else None
        topology_store = getattr(panel, "latest_sensor_mag_instances", {})
        topology = topology_store.get(side, {}) if isinstance(topology_store, Mapping) else {}
        if not isinstance(topology, Mapping) or not topology:
            return "TOPOLOGY WAITING", None
        instance_device_ids = {
            int(instance): int(getattr(state, "device_id"))
            for instance, state in topology.items()
            if getattr(state, "device_id", None) is not None
        }
        params = getattr(status, "params", {}) if status is not None else {}
        if not isinstance(params, Mapping):
            return "PARAM WAITING", None
        return "READY", build_mag_selection_preview(
            params,
            instance_device_ids,
            requested_priorities=requested_priorities,
            actual_selected_device_id=getattr(panel, "latest_vehicle_mag_device_ids", {}).get(side),
        )

    def mag_verification(self, side):
        reader = getattr(self.backend, "pixhawk_mag_verification_snapshot", None)
        return reader(side) if callable(reader) else {}

    def log(self, message):
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            logger(message)


class PixhawkWorkspace(QtWidgets.QWidget):
    """Composition of independent P1/P2 connection and evidence views."""

    inspection_requested = QtCore.pyqtSignal(object)
    MAG_STATUS_AUTO_REFRESH_RETRY_S = MagnetometerView.MAG_STATUS_AUTO_REFRESH_RETRY_S
    MAG_STATUS_AUTO_REFRESH_MAX_ATTEMPTS = MagnetometerView.MAG_STATUS_AUTO_REFRESH_MAX_ATTEMPTS

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
        sources = _PixhawkViewSources(backend)
        settings_reader = getattr(backend, "workstation_settings_snapshot", None)
        try:
            transport_defaults = dict(settings_reader()) if callable(settings_reader) else {}
        except Exception:
            transport_defaults = {}
        self.connections = ConnectionView(
            read_inputs=sources.connection_inputs,
            commands=sources.connection_commands,
            transport_defaults=transport_defaults,
            invoke_command=invoke_command,
            parent=self,
        )
        self.attitude = AttitudeView(
            read_attitude=lambda side: _evidence(backend).attitude(side),
            parent=self,
        )
        self.gnss = GnssView(
            read_gps=lambda side, source="GPS1": _evidence(backend).gps(side, source=source),
            read_selected_source=sources.selected_source,
            set_source=sources.set_source,
            read_baseline=lambda: _evidence(backend).baseline(),
            read_yaw=lambda: _evidence(backend).baseline_yaw_alignment(),
            parent=self,
        )
        self.magnetometers = MagnetometerView(
            read_inputs=sources.magnetometer_inputs,
            read_cal_mag=lambda side: _evidence(backend).magnetometers_for_cal_mag_view(side),
            read_can=lambda side: _evidence(backend).can_magnetometers(side),
            read_legacy=lambda side: _evidence(backend).magnetometers(side),
            read_verification=sources.mag_verification,
            read_preview=sources.mag_preview,
            log=sources.log,
            invoke_command=invoke_command,
            parent=self,
        )
        self.geometry = GeometryView(read_geometry=lambda: _evidence(backend).drone_pixhawk_geometry(), parent=self)
        for part in (self.connections, self.attitude, self.gnss, self.magnetometers, self.geometry):
            part.inspection_requested.connect(self.inspection_requested.emit)
        self.connections.refresh_requested.connect(lambda: self.refresh(self.status_model.snapshot()))

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Pixhawk"))
        layout.addLayout(self.connections.actions)
        for group in (
            self.connections.transport_group,
            self.connections.status_group,
            self.connections.candidate_group,
            self.attitude.group,
            self.gnss.receiver_group,
            self.magnetometers.magnetometer_group,
            self.magnetometers.mag_priority_group,
            self.magnetometers.can_magnetometer_group,
            self.gnss.baseline_group,
            self.gnss.baseline_yaw_group,
            self.geometry.group,
            self.connections.recovery_group,
            self.gnss.gnss_source_group,
            self.magnetometers.parameter_group,
        ):
            layout.addWidget(group)
        layout.addStretch(1)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        self.connections.refresh(snapshot)
        self.attitude.refresh()
        self.gnss.refresh_receivers()
        self.magnetometers.refresh(snapshot)
        self.gnss.refresh_baseline()
        self.geometry.refresh()

    @staticmethod
    def _candidate_role_label(
        candidate: Mapping[str, object],
        panel: object,
        network_roles: Mapping[str, object],
        *,
        saved_serial_keys: Sequence[tuple[str, str]] | None = None,
    ) -> str:
        return ConnectionView._candidate_role_label(
            candidate,
            network_roles,
            saved_serial_keys=_saved_serial_endpoint_keys(panel) if saved_serial_keys is None else saved_serial_keys,
        )

    _saved_serial_endpoint_keys = staticmethod(_saved_serial_endpoint_keys)
    _selected_gnss_source = staticmethod(_selected_gnss_source)
    _serial_endpoint_key = staticmethod(ConnectionView._serial_endpoint_key)
    _candidate_identity = staticmethod(ConnectionView._candidate_identity)
    _mag_role_for_device_id = staticmethod(MagnetometerView._mag_role_for_device_id)
    _mag_preview_role_label = staticmethod(MagnetometerView._mag_preview_role_label)
    _board_rotation_known = staticmethod(MagnetometerView._board_rotation_known)

    def _refresh_gnss_source_controls(self, panel: object) -> None:
        self.gnss._refresh_gnss_source_controls()

    # Existing callers access the same objects through the connections owner.
    _apply_candidate = _view_alias("connections", "_apply_candidate")
    _apply_radio_endpoint = _view_alias("connections", "_apply_radio_endpoint")
    _candidate_inspection = _view_alias("connections", "_candidate_inspection")
    _candidate_rows = _view_alias("connections", "_candidate_rows")
    _candidate_signature = _view_alias("connections", "_candidate_signature")
    _connect_role = _view_alias("connections", "_connect_role")
    _disconnect_role = _view_alias("connections", "_disconnect_role")
    _discovery_revision = _view_alias("connections", "_discovery_revision")
    _inspect_candidate_row = _view_alias("connections", "_inspect_candidate_row")
    _invoke_command = _view_alias("connections", "_invoke_command")
    _invoke_command_available = _view_alias("connections", "_invoke_command_available")
    _network_role_locked = _view_alias("connections", "_network_role_locked")
    _open_px4_shell = _view_alias("connections", "_open_px4_shell")
    _recording_locked = _view_alias("connections", "_recording_locked")
    _refresh_candidate_actions = _view_alias("connections", "_refresh_candidate_actions")
    _selected_candidate_identity = _view_alias("connections", "_selected_candidate_identity")
    _selected_candidate_row = _view_alias("connections", "_selected_candidate_row")
    _set_transport = _view_alias("connections", "_set_transport")
    candidate_buttons = _view_alias("connections", "candidate_buttons")
    candidate_feedback = _view_alias("connections", "candidate_feedback")
    candidate_model = _view_alias("connections", "candidate_model")
    candidate_table = _view_alias("connections", "candidate_table")
    discovery_status = _view_alias("connections", "discovery_status")
    indicators = _view_alias("connections", "indicators")
    network_assignment_values = _view_alias("connections", "network_assignment_values")
    operation_buttons = _view_alias("connections", "operation_buttons")
    radio_baud_spins = _view_alias("connections", "radio_baud_spins")
    radio_device_edits = _view_alias("connections", "radio_device_edits")
    recovery_buttons = _view_alias("connections", "recovery_buttons")
    recovery_feedback = _view_alias("connections", "recovery_feedback")
    recovery_values = _view_alias("connections", "recovery_values")
    role_connect_buttons = _view_alias("connections", "role_connect_buttons")
    role_disconnect_buttons = _view_alias("connections", "role_disconnect_buttons")
    transport_combos = _view_alias("connections", "transport_combos")

    # Existing callers access the same objects through the attitude owner.
    _attitude_inspection = _view_alias("attitude", "_attitude_inspection")
    _inspect_attitude_row = _view_alias("attitude", "_inspect_attitude_row")
    attitude_model = _view_alias("attitude", "attitude_model")
    attitude_table = _view_alias("attitude", "attitude_table")

    # Existing callers access the same objects through the gnss owner.
    _baseline_inspection = _view_alias("gnss", "_baseline_inspection")
    _baseline_yaw_inspection = _view_alias("gnss", "_baseline_yaw_inspection")
    _inspect_baseline_value = _view_alias("gnss", "_inspect_baseline_value")
    _inspect_baseline_yaw_value = _view_alias("gnss", "_inspect_baseline_yaw_value")
    _inspect_receiver_row = _view_alias("gnss", "_inspect_receiver_row")
    _receiver_inspection = _view_alias("gnss", "_receiver_inspection")
    _receiver_row_keys = _view_alias("gnss", "_receiver_row_keys")
    _refresh_baseline_yaw_alignment = _view_alias("gnss", "_refresh_baseline_yaw_alignment")
    _set_gnss_source_from_workspace = _view_alias("gnss", "_set_gnss_source_from_workspace")
    baseline_values = _view_alias("gnss", "baseline_values")
    baseline_yaw_contract_label = _view_alias("gnss", "baseline_yaw_contract_label")
    baseline_yaw_evidence_label = _view_alias("gnss", "baseline_yaw_evidence_label")
    baseline_yaw_group = _view_alias("gnss", "baseline_yaw_group")
    baseline_yaw_plot = _view_alias("gnss", "baseline_yaw_plot")
    baseline_yaw_status_label = _view_alias("gnss", "baseline_yaw_status_label")
    baseline_yaw_values = _view_alias("gnss", "baseline_yaw_values")
    gnss_source_combos = _view_alias("gnss", "gnss_source_combos")
    gnss_source_summary = _view_alias("gnss", "gnss_source_summary")
    receiver_model = _view_alias("gnss", "receiver_model")
    receiver_table = _view_alias("gnss", "receiver_table")

    # Existing callers access the same objects through the magnetometers owner.
    _board_rotation_auto_refresh_attempts = _view_alias("magnetometers", "_board_rotation_auto_refresh_attempts")
    _cal_mag_magnetometer_inspection = _view_alias("magnetometers", "_cal_mag_magnetometer_inspection")
    _can_magnetometer_inspection = _view_alias("magnetometers", "_can_magnetometer_inspection")
    _inspect_can_magnetometer_row = _view_alias("magnetometers", "_inspect_can_magnetometer_row")
    _inspect_magnetometer_row = _view_alias("magnetometers", "_inspect_magnetometer_row")
    _last_ui_yaw_signature = _view_alias("magnetometers", "_last_ui_yaw_signature")
    _mag_status_auto_refresh_attempts = _view_alias("magnetometers", "_mag_status_auto_refresh_attempts")
    _magnetometer_inspection = _view_alias("magnetometers", "_magnetometer_inspection")
    _refresh_mag_priority_group = _view_alias("magnetometers", "_refresh_mag_priority_group")
    _refresh_mag_priority_preview = _view_alias("magnetometers", "_refresh_mag_priority_preview")
    _refresh_px4_parameter_group = _view_alias("magnetometers", "_refresh_px4_parameter_group")
    can_magnetometer_model = _view_alias("magnetometers", "can_magnetometer_model")
    can_magnetometer_table = _view_alias("magnetometers", "can_magnetometer_table")
    mag_priority_apply_buttons = _view_alias("magnetometers", "mag_priority_apply_buttons")
    mag_priority_combos = _view_alias("magnetometers", "mag_priority_combos")
    mag_priority_group = _view_alias("magnetometers", "mag_priority_group")
    mag_priority_payload = _view_alias("magnetometers", "mag_priority_payload")
    mag_priority_preview_labels = _view_alias("magnetometers", "mag_priority_preview_labels")
    mag_priority_refresh_buttons = _view_alias("magnetometers", "mag_priority_refresh_buttons")
    mag_priority_status_labels = _view_alias("magnetometers", "mag_priority_status_labels")
    magnetometer_model = _view_alias("magnetometers", "magnetometer_model")
    magnetometer_table = _view_alias("magnetometers", "magnetometer_table")
    px4_parameter_group_models = _view_alias("magnetometers", "px4_parameter_group_models")
    px4_parameter_group_status_labels = _view_alias("magnetometers", "px4_parameter_group_status_labels")
    px4_parameter_models = _view_alias("magnetometers", "px4_parameter_models")
    px4_parameter_refresh_buttons = _view_alias("magnetometers", "px4_parameter_refresh_buttons")
    px4_parameter_reset_buttons = _view_alias("magnetometers", "px4_parameter_reset_buttons")
    px4_parameter_status_labels = _view_alias("magnetometers", "px4_parameter_status_labels")
    px4_parameter_tables = _view_alias("magnetometers", "px4_parameter_tables")

    # Existing callers access the same objects through the geometry owner.
    _drone_geometry_angle_inspection = _view_alias("geometry", "_drone_geometry_angle_inspection")
    _drone_geometry_vector_inspection = _view_alias("geometry", "_drone_geometry_vector_inspection")
    _inspect_drone_geometry_angle = _view_alias("geometry", "_inspect_drone_geometry_angle")
    _inspect_drone_geometry_vector = _view_alias("geometry", "_inspect_drone_geometry_vector")
    _refresh_drone_pixhawk_geometry = _view_alias("geometry", "_refresh_drone_pixhawk_geometry")
    drone_geometry_angle_values = _view_alias("geometry", "drone_geometry_angle_values")
    drone_geometry_contract_label = _view_alias("geometry", "drone_geometry_contract_label")
    drone_geometry_group = _view_alias("geometry", "drone_geometry_group")
    drone_geometry_model = _view_alias("geometry", "drone_geometry_model")
    drone_geometry_status_label = _view_alias("geometry", "drone_geometry_status_label")
    drone_geometry_table = _view_alias("geometry", "drone_geometry_table")
