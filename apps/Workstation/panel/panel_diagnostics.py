"""Diagnostics feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
from Workstation.device_runtime.connection_view import (
    PixhawkConnectionView, PixhawkConnectionSettings, PixhawkConnectionCommands,
    PIXHAWK_CONNECTION_VIEW_FIELDS, PIXHAWK_CANDIDATE_VIEW_FIELDS,
    swap_spinbox_value, swap_line_edit_text, sample_hz,
)
from Workstation.workstation_core.mavlink_liveness import MAVLINK_QUALIFICATION_S
from Workstation.workstation_core.runtime_camera_status import camera_display_name

from Workstation.device_runtime.connection_jobs import PixhawkConnectionJobs
from Workstation.device_runtime.state import DiagnosticsState
from Workstation.device_runtime.presentation import attitude_cal_status_text
import math
import re
import time
from types import SimpleNamespace
from PyQt5 import (
    QtCore,
    QtGui,
    QtWidgets,
)
from Workstation.workers.device_runtime_workers import PixhawkConnectionWorker, PixhawkDisconnectWorker, PixhawkHostIpDiscoveryWorker, PixhawkParameterWorker, PixhawkPersistentConfigWorker, PixhawkShellCommandWorker, PixhawkStreamSetupWorker, PixhawkUsbBootstrapWorker, PixhawkUsbDiscoveryWorker, PixhawkUsbRecoveryWorker
from Workstation.panel.panel_controls import FieldSelectionDialog, TopicBrowserDialog
from Workstation.panel.panel_support import _build_px4_priority_combo, _px4_priority_combo_value, _set_px4_priority_combo_value
from Workstation.ui.telemetry_plots import AttitudeHistoryPoint, Px4ShellDialog, TelemetryHistoryPlot, TelemetryHistoryPoint
from Workstation.workstation_core.alignment_managers import (
    AttitudeMonitor,
    MagnetometerMonitor,
)
from Workstation.workstation_core.mag_selection import (
    evaluate_mag_selection_verification,
)
from Workstation.workstation_core.mavlink_parameter_store import (
    MavlinkAttitudeParameterStore,
    MavlinkParameterStore,
)
from Workstation.workstation_core.pixhawk_attitude import (
    CameraExtrinsicState,
    CustomTopicSample,
    DEFAULT_USB_MAVLINK_BAUDRATES,
    DualPixhawkMavlinkSubscriber,
    MAVLINK_ATTITUDE_STALE_S,
    camera_label_ko,
    discover_mavlink_messages,
    mavlink_graphable_fields_for_message,
    parse_px4_sensor_mag_shell_output,
    parse_px4_vehicle_magnetometer_shell_device_id,
)
from Workstation.workstation_core.px4_attitude_cal_manager import (
    Px4AttitudeCalManager,
    Px4AttitudeCalStatus,
    current_pixhawk_6x_attitude_cal_params,
)
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    MAG_PRIORITY_KEYS,
    MockParameterStore,
    Px4MagManager,
    RM3100_BACKUP_IDS,
    RM3100_IDS,
    RM3100_PRIMARY_IDS,
    current_pixhawk_6x_mag_params,
    mag_priority_display,
    mag_priority_slot_from_key,
    mag_priority_slot_key,
)
from Workstation.workstation_core.rtk_geometry import (
    BodyFrdBaselineProjection,
    GpsPositionState,
    RtkBaselineEstimate,
)
from Workstation.workstation_core.sync import (
    TimedSampleBuffer,
)
from Workstation.workstation_core.theme import (
    ACCENT,
    SUCCESS_TEXT,
    WARNING_TEXT,
)
from collections import (
    deque,
)
from collections.abc import (
    Sequence,
)
from dataclasses import (
    replace,
)
from ipaddress import (
    IPv4Address,
)


def _mag_tab_alias(key: str) -> property:
    """Keep a known legacy name backed by the per-view tab registry."""
    def read(views):
        try:
            return views.px4_mag_tab_widgets[key]
        except KeyError:
            raise AttributeError(f"px4_mag_tab_widget_{key}") from None
    def write(views, value):
        views.px4_mag_tab_widgets[key] = value
    def delete(views):
        del views.px4_mag_tab_widgets[key]
    return property(read, write, delete)


class DiagnosticsViews(SimpleNamespace):
    """Widget references, including repeated/custom magnetometer panels."""
    px4_mag_tab_widgets: dict[str, QtWidgets.QTabWidget]

    def __init__(self):
        super().__init__(px4_mag_tab_widgets={}, connection=None)

    px4_mag_tab_widget_manager = _mag_tab_alias("manager")
    px4_mag_tab_widget_detail = _mag_tab_alias("detail")
    px4_mag_tab_widget_sensor_tab = _mag_tab_alias("sensor_tab")


def _connection_view_field(name):
    return property(
        lambda views: getattr(views.connection, name),
        lambda views, value: setattr(views.connection, name, value),
    )


for _name in (*PIXHAWK_CONNECTION_VIEW_FIELDS, *PIXHAWK_CANDIDATE_VIEW_FIELDS):
    setattr(DiagnosticsViews, _name, _connection_view_field(_name))
del _name


class DiagnosticsFeature(PanelFeature):
    feature_name = 'diagnostics'
    dependency_names = (
        '_build_alignment_magnetometer_tab',
        '_build_alignment_sensor_recognition_tab',
        '_build_alignment_source_quality_tab',
        '_build_baseline_attitude_alignment_tab',
        '_build_baseline_calculation_view',
        '_build_baseline_field_browser_view',
        '_build_gnss_base_tab',
        '_build_optical_alignment_tab',
        '_configure_collapsible_group',
        '_distance_mode',
        '_handle_alignment_step_changed',
        '_init_baseline_alignment_state',
        '_log',
        '_mavlink_connected_sides',
        '_mavlink_open_sides',
        '_measurement_view_controller',
        '_px4_board_rotation_params',
        '_px4_calibration_parameter_error',
        '_px4_calibration_reset_error',
        '_px4_calibration_reset_pending',
        '_px4_sensor_mag_shell_commands',
        '_record_px4_mag_status_snapshot',
        '_refresh_baseline_alignment_ui',
        '_start_pixhawk_parameter_worker',
        '_start_pixhawk_shell_worker',
        '_update_mag_sensor',
        'advanced_baseline_field_tab_index',
        'advanced_wiris_command_tab_index',
        'f9p_compare_labels',
        'f9p_quality_plots',
        'f9p_sats_plots',
        'f9p_slot_labels',
        'gnss_heading_labels',
        'gps_sensor_labels',
        'gps_sensor_plots',
        'left_gps_label',
        'px4_shell_dialogs',
        'refresh_baseline_field_browser',
        'right_gps_label',
        'rm3100_compare_labels',
        'rm3100_delta_plots',
        'rm3100_norm_plots',
        'rm3100_slot_labels',
        'rtk_baseline_label',
        'rtk_fixed_status_label',
        'rtk_summary_labels',
        'settings',
        'wiris_command_tab_index',
        '_build_toggle_button',
        '_refresh_operation_header',
        '_save_current_settings',
        '_session_recording_active',
        'apply_persistent_pixhawk_eth',
        'apply_selected_pixhawk_candidate',
        'bootstrap_ethernet_mavlink_from_usb',
        'disconnect_mavlink_pixhawk',
        'discover_pc_pixhawk_ip',
        'reboot_pixhawk_from_mavlink',
        'toggle_mavlink_connection',
    )
    writable_dependencies = frozenset(('advanced_baseline_field_tab_index', 'advanced_wiris_command_tab_index', 'f9p_compare_labels', 'f9p_quality_plots', 'f9p_sats_plots', 'f9p_slot_labels', 'gnss_heading_labels', 'gps_sensor_labels', 'gps_sensor_plots', 'left_gps_label', 'right_gps_label', 'rm3100_compare_labels', 'rm3100_delta_plots', 'rm3100_norm_plots', 'rm3100_slot_labels', 'rtk_baseline_label', 'rtk_fixed_status_label', 'rtk_summary_labels', 'wiris_command_tab_index'))

    def __init__(self, widget):
        super().__init__(
            widget,
            state_factory=lambda owner: DiagnosticsState(PixhawkConnectionJobs(owner, thread_parent=widget)),
        )
        self.views = DiagnosticsViews()

    def resource_inventory(self):
        return {**super().resource_inventory(), **self.state.connection_jobs.resource_inventory(),
                **self.state.maintenance_jobs.resource_inventory(), **self.state.admin_jobs.resource_inventory(),
                **self.state.discovery_jobs.resource_inventory()}

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.latest_extrinsics: dict[str, CameraExtrinsicState] = {}
        self.latest_gps: dict[str, GpsPositionState] = {}
        self.custom_watch_specs: list[dict[str, str]] = []
        self.latest_rtk_baseline: RtkBaselineEstimate | None = None
        self.latest_p1_body_frd_baseline: BodyFrdBaselineProjection | None = None
        self.attitude_buffers = {
            "left": TimedSampleBuffer[CameraExtrinsicState](maxlen=1000),
            "right": TimedSampleBuffer[CameraExtrinsicState](maxlen=1000),
        }
        self.mavlink_pixhawk: DualPixhawkMavlinkSubscriber | None = None
        self._eth0_probe_buffers: dict[str, str] = {}
        self._eth0_probe_tokens: dict[str, int] = {}
        self._eth0_probe_timeout_tokens: dict[str, int] = {}
        self._eth0_probe_counter = 0
        self._eth0_probe_timers: dict[str, QtCore.QTimer] = {}
        self.pixhawk_usb_discovery_thread: QtCore.QThread | None = None
        self.pixhawk_usb_discovery_worker: PixhawkUsbDiscoveryWorker | None = None
        self.pixhawk_usb_recovery_thread: QtCore.QThread | None = None
        self.pixhawk_usb_recovery_worker: PixhawkUsbRecoveryWorker | None = None
        self.pixhawk_usb_identity_evidence: list[dict[str, object]] = []
        self.pixhawk_usb_recovery_status: dict[str, dict[str, object]] = {}
        self.pixhawk_host_ip_discovery_thread: QtCore.QThread | None = None
        self.pixhawk_host_ip_discovery_worker: PixhawkHostIpDiscoveryWorker | None = None
        self.camera_network_discovery_thread: QtCore.QThread | None = None
        self.camera_network_discovery_worker: QtCore.QObject | None = None
        self.camera_network_apply_thread: QtCore.QThread | None = None
        self.camera_network_apply_worker: QtCore.QObject | None = None
        self.pixhawk_network_discovery_thread: QtCore.QThread | None = None
        self.pixhawk_network_discovery_worker: QtCore.QObject | None = None
        self.drone_telemetry_network_discovery_thread: QtCore.QThread | None = None
        self.drone_telemetry_network_discovery_worker: QtCore.QObject | None = None
        self.drone_telemetry_network_discovery_generation = 0
        self.drone_telemetry_network_discovery_state = "IDLE"
        self.drone_telemetry_network_candidates: tuple[dict[str, object], ...] = ()
        self._pending_pixhawk_network_assignment: dict[str, dict[str, object]] = {}
        self.pixhawk_connection_thread: QtCore.QThread | None = None
        self.pixhawk_connection_worker: PixhawkConnectionWorker | None = None
        self.pixhawk_disconnection_thread: QtCore.QThread | None = None
        self.pixhawk_disconnection_worker: PixhawkDisconnectWorker | None = None
        self.pixhawk_stream_setup_thread: QtCore.QThread | None = None
        self.pixhawk_stream_setup_worker: PixhawkStreamSetupWorker | None = None
        self.pixhawk_usb_bootstrap_thread: QtCore.QThread | None = None
        self.pixhawk_usb_bootstrap_worker: PixhawkUsbBootstrapWorker | None = None
        self.pixhawk_usb_bootstrap_status: dict[str, dict[str, object]] = {
            "left": {"state": "IDLE", "detail": "대기", "updated_s": time.monotonic()},
            "right": {"state": "IDLE", "detail": "대기", "updated_s": time.monotonic()},
        }
        self.pixhawk_persistent_config_thread: QtCore.QThread | None = None
        self.pixhawk_persistent_config_worker: PixhawkPersistentConfigWorker | None = None
        self.pixhawk_shell_threads: dict[str, QtCore.QThread] = {}
        self.pixhawk_shell_workers: dict[str, PixhawkShellCommandWorker] = {}
        self.pixhawk_parameter_threads: dict[str, QtCore.QThread] = {}
        self.pixhawk_parameter_workers: dict[str, PixhawkParameterWorker] = {}
        self.left_pixhawk_transport = self.settings.left_pixhawk_transport
        self.right_pixhawk_transport = self.settings.right_pixhawk_transport
        self.drone_pixhawk_transport = self.settings.drone_pixhawk_transport
        self.left_pixhawk_radio_device = self.settings.left_pixhawk_radio_device
        self.right_pixhawk_radio_device = self.settings.right_pixhawk_radio_device
        self.drone_pixhawk_radio_device = self.settings.drone_pixhawk_radio_device
        self.left_pixhawk_radio_usb_serial = self.settings.left_pixhawk_radio_usb_serial
        self.right_pixhawk_radio_usb_serial = self.settings.right_pixhawk_radio_usb_serial
        self.drone_pixhawk_radio_usb_serial = self.settings.drone_pixhawk_radio_usb_serial
        self.left_pixhawk_radio_baud = self.settings.left_pixhawk_radio_baud
        self.right_pixhawk_radio_baud = self.settings.right_pixhawk_radio_baud
        self.drone_pixhawk_radio_baud = self.settings.drone_pixhawk_radio_baud
        self.drone_pixhawk_eth_ip = self.settings.drone_pixhawk_eth_ip
        self.drone_telemetry_url = self.settings.drone_telemetry_url
        self.drone_telemetry_baud = self.settings.drone_telemetry_baud
        self.drone_telemetry_connection_state = "DISCONNECTED"
        self.drone_telemetry_last_error = ""
        self.drone_telemetry_connection_thread: QtCore.QThread | None = None
        self.drone_telemetry_connection_worker: PixhawkConnectionWorker | None = None
        self.drone_telemetry_usb_discovery_thread: QtCore.QThread | None = None
        self.drone_telemetry_usb_discovery_worker: PixhawkUsbDiscoveryWorker | None = None
        self.drone_telemetry_usb_discovery_state = ""
        self.pixhawk_transfer_status: dict[str, dict[str, object]] = {}
        self.pixhawk_mavftp_threads: dict[str, QtCore.QThread] = {}
        self.pixhawk_mavftp_workers: dict[str, QtCore.QObject] = {}
        self.pixhawk_usb_flash_thread: QtCore.QThread | None = None
        self.pixhawk_usb_flash_worker: QtCore.QObject | None = None
        self.pixhawk_usb_flash_status: dict[str, object] = {"state": "IDLE"}
        self.pixhawk_attitude_times = {
            "left": deque(maxlen=240),
            "right": deque(maxlen=240),
            "drone": deque(maxlen=240),
        }
        self.attitude_history: dict[str, deque[AttitudeHistoryPoint]] = {
            "roll": deque(maxlen=600),
            "pitch": deque(maxlen=600),
            "yaw": deque(maxlen=600),
        }
        self.magnetometer_monitor = MagnetometerMonitor()
        self.attitude_monitor = AttitudeMonitor()
        self.latest_mag_sensors: dict[str, dict[str, object]] = {"left": {}, "right": {}, "drone": {}}
        self.latest_can_mag_sensors: dict[str, dict[str, object]] = {"left": {}, "right": {}, "drone": {}}
        self.latest_sensor_mag_instances: dict[str, dict[int, object]] = {
            "left": {},
            "right": {},
            "drone": {},
        }
        self.latest_vehicle_mag_device_ids: dict[str, int | None] = {"left": None, "right": None, "drone": None}
        self.latest_vehicle_mag_device_id_timestamps: dict[str, float | None] = {
            "left": None,
            "right": None,
            "drone": None,
        }
        self.px4_mag_managers = {
            "left": Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params())),
            "right": Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params())),
            "drone": Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params())),
        }
        self._px4_mag_status_cache: dict[str, object] = {}
        self._px4_mag_status_incomplete: dict[str, tuple[str, ...]] = {}
        self._px4_mag_verification_targets: dict[str, dict[str, object]] = {}
        self._px4_attitude_cal_status_cache: dict[str, Px4AttitudeCalStatus] = {}
        self.px4_mag_manager = self.px4_mag_managers["left"]
        self.px4_attitude_cal_managers = {
            "left": Px4AttitudeCalManager(MockParameterStore(current_pixhawk_6x_attitude_cal_params())),
            "right": Px4AttitudeCalManager(MockParameterStore(current_pixhawk_6x_attitude_cal_params())),
            "drone": Px4AttitudeCalManager(MockParameterStore(current_pixhawk_6x_attitude_cal_params())),
        }
        self.latest_gps_sensors: dict[str, dict[str, object]] = {"left": {}, "right": {}, "drone": {}}

    def _pixhawk_ethernet_mode(self) -> str:
        mode = str(self.pixhawk_ethernet_mode_combo.currentData() or "mavlink")
        return mode if mode == "mavlink" else "mavlink"

    def _refresh_pixhawk_ethernet_mode_controls(self) -> None:
        # Workstation has a MAVLink-only Pixhawk transport surface.
        self.pixhawk_ethernet_mode_combo.setCurrentIndex(0)

    @staticmethod
    def _px4_mag_registry_key_camera(key: object) -> object:
        """Return the camera-side component of a PX4 MAG widget-registry key.

        Registry keys are ``(camera, panel_key)`` tuples so that multiple
        simultaneously-visible panels (e.g. the Alignment tab manager panel
        and the Pixhawk sensor tab manager panel) can each keep their own
        widgets without one panel's build silently overwriting another
        panel's entries for the same camera side.
        """
        return key[0] if isinstance(key, tuple) else key

    def _px4_mag_registry_values(self, registry: dict, camera: str) -> list:
        """Return every widget registered for *camera* across all PX4 MAG panels."""
        return [
            value
            for key, value in registry.items()
            if self._px4_mag_registry_key_camera(key) == camera
        ]

    def _px4_mag_registry_get(self, registry: dict, camera: str, panel_key: str | None):
        """Return the widget for (*camera*, *panel_key*), falling back to any panel's entry."""
        if panel_key is not None:
            value = registry.get((camera, panel_key))
            if value is not None:
                return value
        values = self._px4_mag_registry_values(registry, camera)
        return values[0] if values else None

    def _ensure_px4_mag_widget_registries(self) -> None:
        if not hasattr(self, "px4_mag_tables"):
            self.px4_mag_tables: dict[tuple[str, str], QtWidgets.QTableWidget] = {}
        if not hasattr(self, "px4_mag_status_labels"):
            self.px4_mag_status_labels: dict[tuple[str, str], QtWidgets.QLabel] = {}
        if not hasattr(self, "px4_mag_refresh_buttons"):
            self.px4_mag_refresh_buttons: dict[tuple[str, str], QtWidgets.QPushButton] = {}
        if not hasattr(self, "px4_mag_apply_manual_buttons"):
            self.px4_mag_apply_manual_buttons: dict[tuple[str, str], QtWidgets.QPushButton] = {}
        if not hasattr(self, "px4_mag_save_buttons"):
            self.px4_mag_save_buttons: dict[tuple[str, str], QtWidgets.QPushButton] = {}
        if not hasattr(self, "px4_mag_reboot_buttons"):
            self.px4_mag_reboot_buttons: dict[tuple[str, str], QtWidgets.QPushButton] = {}
        if not hasattr(self, "px4_mag_priority_spins"):
            # Kept under the historical ``*_spins`` attribute name for backend
            # compatibility; the controls now expose PX4's semantic priority enum.
            self.px4_mag_priority_spins: dict[tuple[str, str], dict[str, QtWidgets.QComboBox]] = {}

    def _build_px4_mag_manager_panel(
        self,
        layout: QtWidgets.QVBoxLayout,
        *,
        collapsible: bool = True,
        panel_key: str = "manager",
    ) -> None:
        group = QtWidgets.QGroupBox("PX4 Magnetometer 역할 / Priority 관리")
        group.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        group_layout = QtWidgets.QVBoxLayout(group)

        # NOTE: these registries persist across repeated calls (they are only
        # created if missing) and are keyed by (camera, panel_key) so that a
        # second call for a different panel (e.g. a Pixhawk sensor tab
        # instance of this same manager panel) does not silently replace the
        # widgets registered by an earlier call for a different panel.
        self._ensure_px4_mag_widget_registries()

        tab_widget = QtWidgets.QTabWidget()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            self._build_single_px4_mag_manager_tab(tab_layout, camera, panel_key)
            tab_widget.addTab(tab, title)
        # A view registry owns arbitrary repeated-panel keys without creating
        # mutable attributes on the feature QObject.
        self.px4_mag_tab_widgets[str(panel_key)] = tab_widget
        group_layout.addWidget(tab_widget)
        layout.addWidget(group)
        if collapsible:
            self._configure_collapsible_group(group, checked=False)
        self.refresh_px4_mag_manager()

    def _build_single_px4_mag_manager_tab(
        self, layout: QtWidgets.QVBoxLayout, camera: str, panel_key: str = "manager"
    ) -> None:
        control_row = QtWidgets.QHBoxLayout()
        refresh_button = QtWidgets.QPushButton("새로고침")
        apply_manual_button = QtWidgets.QPushButton("수동 Priority 적용")
        save_button = QtWidgets.QPushButton("파라미터 저장")
        reboot_button = QtWidgets.QPushButton("PX4 재부팅")
        for button in (
            refresh_button,
            apply_manual_button,
            save_button,
            reboot_button,
        ):
            control_row.addWidget(button)
        control_row.addStretch(1)
        layout.addLayout(control_row)

        status_label = QtWidgets.QLabel("PX4 MAG 상태: 대기 중")
        status_label.setObjectName("statusStrip")
        status_label.setWordWrap(True)
        status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(status_label)

        manual_group = QtWidgets.QGroupBox("수동 Priority 입력")
        manual_layout = QtWidgets.QGridLayout(manual_group)
        manual_layout.setColumnStretch(1, 1)
        default_priorities = {key: 0 for key in MAG_PRIORITY_KEYS}
        registry_key = (camera, panel_key)
        self.px4_mag_priority_spins[registry_key] = {}
        for row, slot_key in enumerate(MAG_PRIORITY_KEYS):
            spin = _build_px4_priority_combo(default_priorities[slot_key])
            spin.setToolTip(
                f"{slot_key}_PRIO 값을 설정합니다. 센서 identity는 {slot_key}_ID에서 읽습니다. "
                f"현재 선택: {mag_priority_display(default_priorities[slot_key])}"
            )
            manual_layout.addWidget(QtWidgets.QLabel(slot_key), row, 0)
            manual_layout.addWidget(spin, row, 1)
            self.px4_mag_priority_spins[registry_key][slot_key] = spin
        manual_note = QtWidgets.QLabel(
            "PX4 값: Uninitialized=-1, Disabled=0, Min=1, Low=25, Medium=50, High=75, Max=100. "
            "기존의 비표준 값은 Custom으로 보존합니다. MAVLink heartbeat + PARAM readback 상태에서만 적용됩니다."
        )
        manual_note.setObjectName("statusStrip")
        manual_note.setWordWrap(True)
        manual_layout.addWidget(manual_note, len(default_priorities), 0, 1, 2)
        layout.addWidget(manual_group)

        self.px4_mag_refresh_buttons[registry_key] = refresh_button
        self.px4_mag_apply_manual_buttons[registry_key] = apply_manual_button
        self.px4_mag_save_buttons[registry_key] = save_button
        self.px4_mag_reboot_buttons[registry_key] = reboot_button
        self.px4_mag_status_labels[registry_key] = status_label

        refresh_button.clicked.connect(
            lambda _checked=False, side=camera: self.refresh_px4_mag_manager(side, read_params=True)
        )
        apply_manual_button.clicked.connect(
            lambda _checked=False, side=camera, pk=panel_key: self.apply_manual_mag_priorities_from_panel(side, panel_key=pk)
        )
        save_button.clicked.connect(lambda _checked=False, side=camera: self.save_px4_mag_params_from_panel(side))
        reboot_button.clicked.connect(lambda _checked=False, side=camera: self.reboot_px4_from_panel(side))

    def _build_px4_attitude_cal_panel(self, layout: QtWidgets.QVBoxLayout, *, collapsible: bool = True) -> None:
        group = QtWidgets.QGroupBox("PX4 가속도계 / 자이로 / 보드 회전 보정 상태 (QGC Sensor Calibration)")
        group.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        group_layout = QtWidgets.QVBoxLayout(group)

        self.px4_attitude_cal_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.px4_attitude_cal_refresh_buttons: dict[str, QtWidgets.QPushButton] = {}

        tab_widget = QtWidgets.QTabWidget()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            self._build_single_px4_attitude_cal_tab(tab_layout, camera)
            tab_widget.addTab(tab, title)
        self.px4_attitude_cal_tab_widget = tab_widget
        group_layout.addWidget(tab_widget)
        layout.addWidget(group)
        if collapsible:
            self._configure_collapsible_group(group, checked=False)
        self.refresh_px4_attitude_cal_status()

    def _build_single_px4_attitude_cal_tab(self, layout: QtWidgets.QVBoxLayout, camera: str) -> None:
        control_row = QtWidgets.QHBoxLayout()
        refresh_button = QtWidgets.QPushButton("새로고침")
        control_row.addWidget(refresh_button)
        control_row.addStretch(1)
        layout.addLayout(control_row)

        status_label = QtWidgets.QLabel("PX4 자세 보정 상태: 대기 중")
        status_label.setObjectName("statusStrip")
        status_label.setWordWrap(True)
        status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(status_label)

        note = QtWidgets.QLabel(
            "가속도계/자이로/보드 회전(레벨 호라이즌) 보정은 QGC Sensor Setup에서 수행합니다. "
            "이 패널은 PX4에 저장된 보정 결과를 읽기 전용으로 표시하며, 여기서 값을 수정하지는 않습니다."
        )
        note.setObjectName("statusStrip")
        note.setWordWrap(True)
        layout.addWidget(note)

        self.px4_attitude_cal_refresh_buttons[camera] = refresh_button
        self.px4_attitude_cal_status_labels[camera] = status_label

        refresh_button.clicked.connect(
            lambda _checked=False, side=camera: self.refresh_px4_attitude_cal_status(side, read_params=True)
        )

    def _build_px4_mag_detail_panel(self, layout: QtWidgets.QVBoxLayout, *, panel_key: str = "detail") -> None:
        group = QtWidgets.QGroupBox("PX4 MAG 고급 상세: CAL_MAG 슬롯 / Priority")
        group_layout = QtWidgets.QVBoxLayout(group)
        note = QtWidgets.QLabel(
            "일반 MAG 관리에는 역할 상태와 Priority 수정만 표시합니다. "
            "CAL_MAG 슬롯 매핑과 PX4 PARAM readback은 이 고급 화면에서 확인합니다."
        )
        note.setObjectName("statusStrip")
        note.setWordWrap(True)
        group_layout.addWidget(note)

        self._ensure_px4_mag_widget_registries()
        tab_widget = QtWidgets.QTabWidget()
        for camera, title in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            tab = QtWidgets.QWidget()
            tab_layout = QtWidgets.QVBoxLayout(tab)
            table = QtWidgets.QTableWidget(0, 10)
            table.setHorizontalHeaderLabels(
                [
                    "Slot",
                    "Device ID",
                    "Device Type",
                    "Interface",
                    "RM3100",
                    "Priority",
                    "Enabled",
                    "Expected Selected",
                    "Actual Selected",
                    "Status",
                ]
            )
            table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
            table.setAlternatingRowColors(True)
            table.setWordWrap(True)
            table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            table.verticalHeader().setVisible(False)
            table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
            table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            tab_layout.addWidget(table)
            self.px4_mag_tables[(camera, panel_key)] = table
            tab_widget.addTab(tab, title)
        self.px4_mag_tab_widgets[str(panel_key)] = tab_widget
        group_layout.addWidget(tab_widget)
        layout.addWidget(group)
        self.refresh_px4_mag_manager()

    def _build_single_pixhawk_sensor_tab(self, layout: QtWidgets.QVBoxLayout, camera: str, title: str) -> None:
        layout.setSpacing(8)

        top_row = QtWidgets.QHBoxLayout()
        top_row.setSpacing(8)

        rm_group = QtWidgets.QGroupBox(f"{title} · RM3100 CAN")
        rm_form = QtWidgets.QGridLayout(rm_group)
        rm_form.setContentsMargins(8, 10, 8, 8)
        rm_form.setHorizontalSpacing(8)
        rm_form.setVerticalSpacing(6)
        rm_source_label = QtWidgets.QLabel("소스: MAVLink SCALED_IMU* / MAVLink sensor telemetry")
        rm_source_label.setObjectName("statusStrip")
        rm_form.addWidget(rm_source_label, 0, 0, 1, 2)
        rm_form.addWidget(QtWidgets.QLabel("센서"), 1, 0)
        rm_form.addWidget(QtWidgets.QLabel("최근 샘플"), 1, 1)
        for row, slot in enumerate(("CAN1", "CAN2"), start=2):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            rm_form.addWidget(QtWidgets.QLabel(f"RM3100 · {slot}"), row, 0)
            rm_form.addWidget(label, row, 1)
            self.rm3100_slot_labels[camera][slot] = label
            self.mag_sensor_labels[camera][slot] = label
        rm3100_compare_label = QtWidgets.QLabel("CAN1/CAN2 비교: 대기 중")
        rm3100_compare_label.setObjectName("statusStrip")
        rm_form.addWidget(QtWidgets.QLabel("비교"), 4, 0)
        rm_form.addWidget(rm3100_compare_label, 4, 1)
        self.rm3100_compare_labels[camera] = rm3100_compare_label
        top_row.addWidget(rm_group, 1)

        gnss_group = QtWidgets.QGroupBox(f"{title} · F9P GNSS")
        gnss_form = QtWidgets.QGridLayout(gnss_group)
        gnss_form.setContentsMargins(8, 10, 8, 8)
        gnss_form.setHorizontalSpacing(8)
        gnss_form.setVerticalSpacing(6)
        gnss_source_label = QtWidgets.QLabel("소스: MAVLink GPS / GNSS telemetry")
        gnss_source_label.setObjectName("statusStrip")
        gnss_form.addWidget(gnss_source_label, 0, 0, 1, 2)
        gnss_form.addWidget(QtWidgets.QLabel("수신기"), 1, 0)
        gnss_form.addWidget(QtWidgets.QLabel("최근 샘플"), 1, 1)
        for row, slot in enumerate(("GPS1", "GPS2"), start=2):
            label = QtWidgets.QLabel("대기 중")
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            gnss_form.addWidget(QtWidgets.QLabel(f"F9P · {slot}"), row, 0)
            gnss_form.addWidget(label, row, 1)
            self.f9p_slot_labels[camera][slot] = label
            self.gps_sensor_labels[camera][slot] = label
        f9p_compare_label = QtWidgets.QLabel("GPS1/GPS2 품질 비교: 대기 중")
        f9p_compare_label.setObjectName("statusStrip")
        f9p_compare_label.setWordWrap(True)
        f9p_compare_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        gnss_form.addWidget(QtWidgets.QLabel("비교"), 4, 0)
        gnss_form.addWidget(f9p_compare_label, 4, 1)
        self.f9p_compare_labels[camera] = f9p_compare_label
        gnss_heading_label = QtWidgets.QLabel("GNSS Heading: /fmu/out/sensor_gnss_relative 대기 중")
        gnss_heading_label.setObjectName("statusStrip")
        gnss_form.addWidget(QtWidgets.QLabel("Moving Base"), 5, 0)
        gnss_form.addWidget(gnss_heading_label, 5, 1)
        self.gnss_heading_labels[camera] = gnss_heading_label
        layout.addLayout(top_row)

        rm3100_norm_plot = TelemetryHistoryPlot(f"{title} 자기장 크기 안정성 (CAN1/CAN2)", "G")
        rm3100_delta_plot = TelemetryHistoryPlot(f"{title} CAN1/CAN2 자기장 불일치", "G/deg")
        f9p_sats_plot = TelemetryHistoryPlot(f"{title} GPS 위성 수 (GPS1/GPS2)", "sats")
        f9p_quality_plot = TelemetryHistoryPlot(f"{title} GPS 수평 정확도 eph (GPS1/GPS2)", "m")
        for plot in (rm3100_norm_plot, rm3100_delta_plot, f9p_sats_plot, f9p_quality_plot):
            plot.setMinimumHeight(145)
        self.rm3100_norm_plots[camera] = rm3100_norm_plot
        self.rm3100_delta_plots[camera] = rm3100_delta_plot
        self.f9p_sats_plots[camera] = f9p_sats_plot
        self.f9p_quality_plots[camera] = f9p_quality_plot

        rm_graph_layout = QtWidgets.QHBoxLayout()
        rm_graph_layout.setSpacing(8)
        rm_graph_layout.addWidget(rm3100_norm_plot, 1)
        rm_graph_layout.addWidget(rm3100_delta_plot, 1)
        layout.addLayout(rm_graph_layout, 1)

        f9p_detail_group = QtWidgets.QGroupBox(f"상세: {title} · F9P GNSS")
        f9p_detail_layout = QtWidgets.QVBoxLayout(f9p_detail_group)
        f9p_detail_layout.addWidget(gnss_group)
        f9p_graph_layout = QtWidgets.QHBoxLayout()
        f9p_graph_layout.setContentsMargins(0, 0, 0, 0)
        f9p_graph_layout.setSpacing(8)
        f9p_graph_layout.addWidget(f9p_sats_plot, 1)
        f9p_graph_layout.addWidget(f9p_quality_plot, 1)
        f9p_detail_layout.addLayout(f9p_graph_layout)
        self._configure_collapsible_group(f9p_detail_group, checked=False)
        layout.addWidget(f9p_detail_group)

    def discover_custom_mavlink_watch(self) -> None:
        messages = discover_mavlink_messages()
        dialog = TopicBrowserDialog(messages, parent=self.widget, title="MAVLink Message Browser")
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        selected_message = dialog.selected_topic()
        if not selected_message:
            return
        message_name = selected_message.get("name", "").strip().upper()
        if not message_name:
            return
        fields = mavlink_graphable_fields_for_message(message_name)
        if not fields:
            self._log(f"[픽스호크] 그래프 가능한 MAVLink field를 찾지 못했습니다: {message_name}")
            return
        field_dialog = FieldSelectionDialog(message_name, "MAVLink", fields, parent=self.widget)
        if field_dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        selected_fields = field_dialog.selected_fields()
        if not selected_fields:
            self._log("[픽스호크] 선택한 MAVLink field가 없습니다.")
            return
        for field in selected_fields:
            key = f"mavlink:{message_name}.{field}"
            self._add_custom_mavlink_watch(
                key=key,
                topic=message_name,
                field=field,
                message=message_name,
            )
        self._log(f"[픽스호크] MAVLink Custom watch {len(selected_fields)}개 추가: {message_name}")

    def _add_custom_mavlink_watch(
        self,
        *,
        key: str,
        topic: str,
        field: str,
        message: str | None = None,
    ) -> None:
        spec = {"key": key, "topic": topic, "field": field, "source": "mavlink"}
        if message:
            spec["message"] = message
        if spec not in self.custom_watch_specs:
            self.custom_watch_specs.append(spec)
        if self.custom_watch_remove_combo.findData(key) < 0:
            self.custom_watch_remove_combo.addItem(key, key)
        if self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.add_custom_watch(spec)

    def remove_custom_mavlink_watch(self, key: str) -> None:
        removed_specs = [spec for spec in self.custom_watch_specs if str(spec.get("key", "")) == key]
        if not removed_specs:
            self._log(f"[픽스호크] 제거할 Custom watch가 없습니다: {key}")
            return
        self.custom_watch_specs = [spec for spec in self.custom_watch_specs if str(spec.get("key", "")) != key]
        combo_index = self.custom_watch_remove_combo.findData(key)
        if combo_index >= 0:
            self.custom_watch_remove_combo.removeItem(combo_index)
        self.custom_watch_plot.remove_series(key)
        self.custom_watch_plot.remove_series(f"left:{key}")
        self.custom_watch_plot.remove_series(f"right:{key}")
        if self.mavlink_pixhawk is not None:
            self.mavlink_pixhawk.remove_custom_watch(key)
        self._log(f"[픽스호크] Custom watch 제거: {key}")

    def remove_selected_custom_topic_watch(self) -> None:
        key = self.custom_watch_remove_combo.currentData()
        if key is None:
            self._log("[픽스호크] 제거할 Custom watch가 없습니다.")
            return
        self.remove_custom_mavlink_watch(str(key))

    def add_custom_mavlink_watch_from_ui(self) -> None:
        message = self.custom_watch_message_edit.text().strip().upper()
        field = self.custom_watch_field_edit.text().strip()
        name = self.custom_watch_name_edit.text().strip() or f"mavlink:{message}.{field}"
        if not message or not field:
            self._log("[픽스호크] MAVLink watch 추가 실패: message/field가 비어 있습니다.")
            return
        self._add_custom_mavlink_watch(
            key=name,
            topic=message,
            field=field,
            message=message,
        )
        self._log(f"[픽스호크] MAVLink watch 추가: {name} = {message}.{field}")

    def _update_custom_watch(self, sample: CustomTopicSample) -> None:
        if sample.numeric_value is not None:
            self.custom_watch_plot.add_point(
                TelemetryHistoryPoint(sample.timestamp_s, sample.key, sample.numeric_value)
            )

    def _usb_bootstrap_baud_candidates(self, selected_baud: int) -> tuple[int, ...]:
        ordered = [int(selected_baud)]
        ordered.extend(int(baud) for baud in DEFAULT_USB_MAVLINK_BAUDRATES)
        return tuple(dict.fromkeys(baud for baud in ordered if baud > 0))

    def _set_usb_bootstrap_baud(self, camera: str, baud: int) -> None:
        if camera == "left":
            self.left_pixhawk_bootstrap_baud_spin.setValue(int(baud))
        else:
            self.right_pixhawk_bootstrap_baud_spin.setValue(int(baud))

    def open_px4_shell(self, camera: str) -> None:
        if self.mavlink_pixhawk is None or camera not in self.mavlink_pixhawk.connections:
            self._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell을 열 수 없습니다: MAVLink 연결이 없습니다.")
            return
        dialog = self.px4_shell_dialogs.get(camera)
        if dialog is None:
            dialog = Px4ShellDialog(
                f"{camera_label_ko(camera)} PX4 Shell",
                lambda command, camera=camera: self._send_px4_shell_command(camera, command),
                lambda _checked=False, camera=camera: self._read_px4_sensor_mag_shell(camera),
            )
            dialog.destroyed.connect(lambda _obj=None, camera=camera: self.px4_shell_dialogs.pop(camera, None))
            self.px4_shell_dialogs[camera] = dialog
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell 창을 열었습니다.")

    def _send_px4_shell_command(self, camera: str, command: str) -> None:
        if self.mavlink_pixhawk is None:
            self._log(f"[픽스호크] {camera_label_ko(camera)} PX4 Shell 명령 실패: MAVLink 연결이 없습니다.")
            return
        self._start_pixhawk_shell_worker(camera, command)

    def _read_px4_sensor_mag_shell(self, camera: str) -> None:
        """Read sensor_mag instances through the existing Ethernet Shell path."""
        commands = self._px4_sensor_mag_shell_commands()
        self._start_pixhawk_shell_worker(
            camera,
            commands,
            operation="sensor_mag_shell_watch",
            timeout_s=4.0,
            on_complete=lambda result, camera=camera: self._apply_px4_sensor_mag_shell_result(camera, result),
        )

    def _apply_px4_sensor_mag_shell_result(self, camera: str, result: dict[str, object]) -> None:
        output = str(result.get("output") or "")
        operation = str(result.get("operation") or "")
        polling = operation == "sensor_mag_shell_poll"
        active_device_id = parse_px4_vehicle_magnetometer_shell_device_id(output)
        if active_device_id is not None:
            previous_active_device_id = self.latest_vehicle_mag_device_ids.get(camera)
            self.latest_vehicle_mag_device_ids[camera] = active_device_id
            timestamps = getattr(self, "latest_vehicle_mag_device_id_timestamps", None)
            if not isinstance(timestamps, dict):
                timestamps = {}
                setattr(self, "latest_vehicle_mag_device_id_timestamps", timestamps)
            timestamps[camera] = time.monotonic()
            if previous_active_device_id != active_device_id:
                self._log(
                    f"[픽스호크] {camera_label_ko(camera)} Shell active MAG 갱신: "
                    f"vehicle_magnetometer.device_id={active_device_id}"
                )
                self.refresh_px4_mag_manager(camera)
        parsed_states = parse_px4_sensor_mag_shell_output(
            output,
            camera=camera,
            known_only=False,
        )
        topology_by_instance: dict[int, object] = {}
        for state in parsed_states:
            match = re.search(r"sensor_mag/(\d+)$", str(getattr(state, "topic", "")))
            if match is None or getattr(state, "device_id", None) is None:
                continue
            topology_by_instance[int(match.group(1))] = state
        if topology_by_instance:
            topology_store = getattr(self, "latest_sensor_mag_instances", None)
            if not isinstance(topology_store, dict):
                topology_store = {}
                setattr(self, "latest_sensor_mag_instances", topology_store)
            topology_store[camera] = topology_by_instance
        if polling and parsed_states:
            parsed_signature = tuple(
                sorted(
                    f"{state.sensor_slot}:{state.device_id or 'N/A'}"
                    for state in parsed_states
                )
            )
            parsed_log = getattr(self, "_sensor_mag_shell_parsed_signature", {})
            if not isinstance(parsed_log, dict):
                parsed_log = {}
                setattr(self, "_sensor_mag_shell_parsed_signature", parsed_log)
            if parsed_log.get(camera) != parsed_signature:
                parsed_log[camera] = parsed_signature
                self._log(
                    f"[픽스호크] {camera_label_ko(camera)} sensor_mag Shell instances: "
                    + ", ".join(parsed_signature)
                )
        standard_slot_by_device = {
            BOARD_INTERNAL_MAG_ID: "CAN1",
            GPS1_MAG_ID: "CAN2",
            GPS2_MAG_ID: "CAN3",
        }
        states_by_source_slot = {}
        for state in parsed_states:
            if state.device_id in RM3100_IDS:
                slot = state.sensor_slot
                source_key = "rm"
            else:
                slot = standard_slot_by_device.get(state.device_id)
                source_key = "standard"
            if slot is None:
                continue
            states_by_source_slot[(source_key, slot)] = replace(state, sensor_slot=slot)
        states = tuple(states_by_source_slot.values())
        if not states:
            if polling:
                last_empty = getattr(self, "_sensor_mag_shell_empty_log_s", {})
                if not isinstance(last_empty, dict):
                    last_empty = {}
                    setattr(self, "_sensor_mag_shell_empty_log_s", last_empty)
                now_s = time.monotonic()
                previous_s = float(last_empty.get(camera, float("-inf")))
                if now_s - previous_s >= 5.0:
                    last_empty[camera] = now_s
                    snippet = " ".join(output.split())[-600:] or "<empty>"
                    self._log(
                        f"[픽스호크] {camera_label_ko(camera)} sensor_mag Shell polling 응답에 "
                        f"알려진 RM3100 없음 (output_bytes={len(output.encode('utf-8'))}, "
                        f"active_device_id={active_device_id or 'N/A'}, output={snippet})"
                    )
            else:
                self._log(
                    f"[픽스호크] {camera_label_ko(camera)} sensor_mag Shell 결과에서 CAN device_id를 찾지 못했습니다."
                )
            return
        update_state = getattr(self, "_update_mag_sensor", None)
        for state in states:
            if callable(update_state):
                update_state(camera, state)
            else:
                self.latest_can_mag_sensors.setdefault(camera, {})[state.sensor_slot] = state
        if polling:
            def yaw_text(state: object) -> str:
                try:
                    x_value = float(getattr(state, "x"))
                    y_value = float(getattr(state, "y"))
                    return f"{math.degrees(math.atan2(-y_value, x_value)):+.2f}deg"
                except (TypeError, ValueError, AttributeError):
                    return "N/A"

            signatures = tuple(
                sorted(
                    f"{state.sensor_slot}:{state.device_id}:{yaw_text(state)}"
                    for state in states
                )
            )
            last_success = getattr(self, "_sensor_mag_shell_success_signature", {})
            if not isinstance(last_success, dict):
                last_success = {}
                setattr(self, "_sensor_mag_shell_success_signature", last_success)
            if last_success.get(camera) != signatures:
                last_success[camera] = signatures
                self._log(
                    f"[픽스호크] {camera_label_ko(camera)} sensor_mag Shell polling 수신: "
                    + ", ".join(signatures)
                )
        else:
            self._log(
                f"[픽스호크] {camera_label_ko(camera)} Shell sensor_mag 감시 갱신: "
                + ", ".join(f"{state.sensor_slot} device_id={state.device_id}" for state in states)
            )

    def _px4_mag_camera_label(self, camera: str) -> str:
        if camera == "left":
            return "Pixhawk 1"
        if camera == "right":
            return "Pixhawk 2"
        if camera == "drone":
            return "Drone Pixhawk"
        return str(camera)

    def _px4_mag_verification_snapshot(self, camera: str, *, now_s: float | None = None) -> dict[str, object]:
        camera = str(camera or "")
        now = time.monotonic() if now_s is None else float(now_s)
        target = self._px4_mag_verification_targets.get(camera, {})
        status = self._px4_mag_status_cache.get(camera)
        expected = target.get("expected_device_id")
        if expected is None and status is not None:
            expected = getattr(status, "expected_selected_device_id", None)
        actual = self.latest_vehicle_mag_device_ids.get(camera)
        active_timestamp = self.latest_vehicle_mag_device_id_timestamps.get(camera)
        attitude = self.latest_extrinsics.get(camera)
        attitude_timestamp = None if attitude is None else getattr(attitude, "timestamp_s", None)
        verification = evaluate_mag_selection_verification(
            expected_device_id=expected,
            actual_device_id=actual,
            active_sample_timestamp_s=active_timestamp,
            attitude_timestamp_s=attitude_timestamp,
            started_s=target.get("started_s"),
            now_s=now,
            attitude_stale_s=MAVLINK_ATTITUDE_STALE_S,
        )
        if not target:
            return {
                "camera": camera,
                "label": self._px4_mag_camera_label(camera),
                "verification_started_s": None,
                "state": "UNVERIFIED",
                "expected_device_id": expected,
                "actual_device_id": actual,
                "active_sample_age_s": verification.active_sample_age_s,
                "attitude_age_s": verification.attitude_age_s,
                "reason": "Apply & Verify required after the last MAG change",
                "verified": False,
            }
        if target.get("error"):
            return {
                "camera": camera,
                "label": self._px4_mag_camera_label(camera),
                "verification_started_s": target.get("started_s"),
                "state": "ERROR",
                "expected_device_id": expected,
                "actual_device_id": actual,
                "active_sample_age_s": verification.active_sample_age_s,
                "attitude_age_s": verification.attitude_age_s,
                "reason": str(target.get("error")),
                "verified": False,
            }
        return {
            "camera": camera,
            "label": self._px4_mag_camera_label(camera),
            "verification_started_s": target.get("started_s"),
            **verification.to_mapping(),
        }

    def workstation_px4_mag_verification_snapshot(self, camera: str | None = None) -> object:
        cameras = ("left", "right") if camera is None else (str(camera),)
        snapshots = {
            side: self._px4_mag_verification_snapshot(side)
            for side in cameras
            if side in {"left", "right"}
        }
        return snapshots if camera is None else snapshots.get(str(camera), {})

    def workstation_px4_mag_measurement_gate(
        self,
        required_sides: Sequence[str] = ("left", "right"),
    ) -> dict[str, object]:
        sides = tuple(side for side in (str(value) for value in required_sides) if side in {"left", "right"})
        snapshots = {
            side: self._px4_mag_verification_snapshot(side)
            for side in sides
        }
        blockers = tuple(
            f"{snapshots[side]['label']}: {snapshots[side]['state']} · {snapshots[side]['reason']}"
            for side in sides
            if not bool(snapshots[side].get("verified"))
        )
        result = {
            "schema": "pixhawk-mag-measurement-gate-v1",
            "required": bool(sides),
            "required_sides": list(sides),
            "ready": bool(sides) and not blockers,
            "blockers": list(blockers),
            "sides": snapshots,
        }
        self._last_measurement_mag_gate = result
        return result

    def measurement_mag_start_gate(self) -> bool:
        """Report MAG active-selection readiness for methods that consume live Pixhawk data.

        The result is advisory: ``start_yolo`` logs an unverified side and
        continues. The refreshed ``_last_measurement_mag_gate`` snapshot is
        still recorded as run provenance.
        """

        try:
            snapshot = self._measurement_view_controller().workstation_measurement_methods_snapshot()
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return True
        selected_id = str(snapshot.get("selected_method_id") or "") if isinstance(snapshot, dict) else ""
        catalog = snapshot.get("method_catalog") if isinstance(snapshot, dict) else ()
        selected = next(
            (
                row for row in (catalog or ())
                if isinstance(row, dict) and str(row.get("method_id") or "") == selected_id
            ),
            None,
        )
        if not isinstance(selected, dict) or not bool(selected.get("requires_live_pixhawk")):
            self._last_measurement_mag_gate = {
                "schema": "pixhawk-mag-measurement-gate-v1",
                "required": False,
                "required_sides": [],
                "ready": True,
                "blockers": [],
                "sides": {},
            }
            return True
        open_sides_reader = getattr(self, "_mavlink_open_sides", None)
        open_sides = set(open_sides_reader()) if callable(open_sides_reader) else set()
        active_ids = getattr(self, "latest_vehicle_mag_device_ids", {})
        has_active_evidence = any(active_ids.get(side) is not None for side in ("left", "right"))
        has_verification_target = any(
            side in self._px4_mag_verification_targets
            for side in ("left", "right")
        )
        # Preserve the existing disconnected/offline behavior. The MAG gate
        # becomes authoritative once Pixhawk live evidence or an explicit
        # side-specific MAG apply exists.
        if not (open_sides or has_active_evidence or has_verification_target):
            self._last_measurement_mag_gate = {
                "schema": "pixhawk-mag-measurement-gate-v1",
                "required": False,
                "required_sides": [],
                "ready": True,
                "blockers": [],
                "sides": {},
            }
            return True
        gate = self.workstation_px4_mag_measurement_gate(("left", "right"))
        if not bool(gate.get("ready")):
            self._log(
                "[측정] MAG active-selection 미검증 (측정 시작은 차단하지 않음): "
                + "; ".join(str(value) for value in gate.get("blockers") or ())
            )
            return False
        return True

    def _px4_mag_manager_has_data_source(self, camera: str) -> bool:
        if self.latest_vehicle_mag_device_ids.get(camera) is not None:
            return True
        return camera in self._mavlink_connected_sides()

    def _sync_px4_mag_manager_store(self, camera: str) -> bool:
        manager = self.px4_mag_managers.get(camera) if hasattr(self, "px4_mag_managers") else None
        if manager is None:
            return False
        mavlink = getattr(self, "mavlink_pixhawk", None)
        if mavlink is not None and camera in self._mavlink_connected_sides():
            current_store = getattr(manager, "store", None)
            if not (
                isinstance(current_store, MavlinkParameterStore)
                and current_store.subscriber is mavlink
                and current_store.camera == camera
            ):
                manager.store = MavlinkParameterStore(mavlink, camera)
            return True
        current_store = getattr(manager, "store", None)
        if isinstance(current_store, MavlinkParameterStore):
            manager.store = MockParameterStore(current_pixhawk_6x_mag_params())
        return False

    def _set_px4_mag_manager_button_state(
        self,
        camera: str,
        *,
        refresh_enabled: bool,
        actions_enabled: bool,
        tooltip: str = "",
    ) -> None:
        for refresh_button in self._px4_mag_registry_values(self.px4_mag_refresh_buttons, camera):
            refresh_button.setEnabled(refresh_enabled)
            refresh_button.setToolTip("" if refresh_enabled else tooltip)
        for registry in (
            self.px4_mag_apply_manual_buttons,
            self.px4_mag_save_buttons,
            self.px4_mag_reboot_buttons,
        ):
            for button in self._px4_mag_registry_values(registry, camera):
                button.setEnabled(actions_enabled)
                button.setToolTip("" if actions_enabled else tooltip)

    def _clear_px4_mag_manager_ui(self, camera: str) -> None:
        if not hasattr(self, "px4_mag_tables"):
            return
        # Update every panel instance registered for this camera (e.g. the
        # manager panel's status label and the detail panel's table), not
        # just whichever panel happened to be built most recently.
        for table in self._px4_mag_registry_values(self.px4_mag_tables, camera):
            table.setRowCount(0)
        for status_label in self._px4_mag_registry_values(self.px4_mag_status_labels, camera):
            status_label.setText(
                f"{self._px4_mag_camera_label(camera)} | 픽스호크 MAG 관리: MAVLink 수신 대기 중\n"
                "MAVLink에서 실제 MAG device_id를 수신하면 관리 내용이 표시됩니다."
            )
        self._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=False,
            actions_enabled=False,
            tooltip="MAVLink MAG 수신 후 사용 가능",
        )

    def _clear_px4_mag_manager_connection_state(self, camera: str | None = None) -> None:
        sides = ("left", "right") if camera is None else (camera,)
        timestamps = getattr(self, "latest_vehicle_mag_device_id_timestamps", None)
        if not isinstance(timestamps, dict):
            timestamps = {}
            setattr(self, "latest_vehicle_mag_device_id_timestamps", timestamps)
        targets = getattr(self, "_px4_mag_verification_targets", None)
        if not isinstance(targets, dict):
            targets = {}
            setattr(self, "_px4_mag_verification_targets", targets)
        for side in sides:
            self.latest_vehicle_mag_device_ids[side] = None
            timestamps[side] = None
            self._px4_mag_status_cache.pop(side, None)
            self._px4_mag_status_incomplete.pop(side, None)
            self._px4_board_rotation_params.pop(side, None)
            targets.pop(side, None)
            self._px4_calibration_parameter_error.pop(side, None)
            for key in tuple(self._px4_calibration_reset_error):
                if key.startswith(f"{side}:"):
                    self._px4_calibration_reset_error.pop(key, None)
            self._px4_calibration_reset_pending.pop(side, None)
            self._clear_px4_mag_manager_ui(side)

    def _render_px4_mag_mavlink_pending_status(self, camera: str) -> None:
        status_labels = (
            self._px4_mag_registry_values(self.px4_mag_status_labels, camera)
            if hasattr(self, "px4_mag_status_labels")
            else []
        )
        for status_label in status_labels:
            actual = self.latest_vehicle_mag_device_ids.get(camera)
            status_label.setText(
                f"{self._px4_mag_camera_label(camera)} | Actual selected MAG: "
                f"{'N/A' if actual is None else actual}\n"
                "MAVLink MAG 수신 중. PARAM readback은 새로고침 버튼을 눌렀을 때 실행합니다."
            )
        self._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=True,
            actions_enabled=False,
            tooltip="먼저 새로고침으로 CAL_MAG PARAM readback을 완료하세요.",
        )

    def refresh_px4_mag_manager(self, camera: str | None = None, *, read_params: bool = False) -> None:
        if not hasattr(self, "px4_mag_managers"):
            return
        cameras = ("left", "right") if camera is None else (camera,)
        for side in cameras:
            manager = self.px4_mag_managers.get(side)
            if manager is None:
                continue
            if not self._px4_mag_manager_has_data_source(side):
                self._clear_px4_mag_manager_ui(side)
                continue
            using_real_params = self._sync_px4_mag_manager_store(side)
            manager.actual_selected_device_id = self.latest_vehicle_mag_device_ids.get(side)
            if using_real_params and not read_params:
                self._render_px4_mag_mavlink_pending_status(side)
                continue
            if using_real_params and read_params:
                self._start_pixhawk_parameter_worker(side, "mag_refresh")
                continue
            status = manager.refresh()
            if using_real_params:
                self._render_px4_mag_status(side, status)
            else:
                self._render_px4_mag_mavlink_pending_status(side)
            self._record_px4_mag_status_snapshot(side)

    def _sync_px4_attitude_cal_store(self, camera: str) -> bool:
        manager = self.px4_attitude_cal_managers.get(camera) if hasattr(self, "px4_attitude_cal_managers") else None
        if manager is None:
            return False
        mavlink = getattr(self, "mavlink_pixhawk", None)
        if mavlink is not None and camera in self._mavlink_connected_sides():
            current_store = getattr(manager, "store", None)
            if not (
                isinstance(current_store, MavlinkAttitudeParameterStore)
                and current_store.subscriber is mavlink
                and current_store.camera == camera
            ):
                manager.store = MavlinkAttitudeParameterStore(mavlink, camera)
            return True
        current_store = getattr(manager, "store", None)
        if isinstance(current_store, MavlinkAttitudeParameterStore):
            manager.store = MockParameterStore(current_pixhawk_6x_attitude_cal_params())
        return False

    def _clear_px4_attitude_cal_ui(self, camera: str) -> None:
        label = getattr(self, "px4_attitude_cal_status_labels", {}).get(camera)
        if label is not None:
            label.setText(
                f"{self._px4_mag_camera_label(camera)} | PX4 자세 보정 상태: 프로토콜 수신 대기 중\n"
                "MAVLink 연결 후 새로고침을 누르면 가속도계/자이로/보드 회전 보정값을 읽습니다."
            )
        button = getattr(self, "px4_attitude_cal_refresh_buttons", {}).get(camera)
        if button is not None:
            button.setEnabled(False)
            button.setToolTip("MAVLink 연결 후 사용 가능")

    def _clear_px4_attitude_cal_connection_state(self, camera: str | None = None) -> None:
        sides = ("left", "right") if camera is None else (camera,)
        for side in sides:
            self._px4_attitude_cal_status_cache.pop(side, None)
            self._clear_px4_attitude_cal_ui(side)

    def _render_px4_attitude_cal_pending_status(self, camera: str) -> None:
        label = self.px4_attitude_cal_status_labels.get(camera) if hasattr(self, "px4_attitude_cal_status_labels") else None
        if label is not None:
            label.setText(
                f"{self._px4_mag_camera_label(camera)} | MAVLink 연결됨. "
                "새로고침을 눌러 CAL_ACC/CAL_GYRO/SENS_BOARD PARAM을 읽으세요."
            )
        button = self.px4_attitude_cal_refresh_buttons.get(camera) if hasattr(self, "px4_attitude_cal_refresh_buttons") else None
        if button is not None:
            button.setEnabled(True)
            button.setToolTip("")

    @staticmethod
    def _format_attitude_cal_value(value: float | None) -> str:
        return "N/A" if value is None else f"{value:.5f}"

    def _format_attitude_cal_status_text(self, camera: str, status: Px4AttitudeCalStatus) -> str:
        return attitude_cal_status_text(
            self._px4_mag_camera_label(camera), status,
            format_value=self._format_attitude_cal_value,
        )

    def _render_px4_attitude_cal_status(
        self, camera: str, status: Px4AttitudeCalStatus, *, using_real_params: bool
    ) -> None:
        self._px4_attitude_cal_status_cache[camera] = status
        label = self.px4_attitude_cal_status_labels.get(camera) if hasattr(self, "px4_attitude_cal_status_labels") else None
        if label is not None:
            label.setText(self._format_attitude_cal_status_text(camera, status))
        button = self.px4_attitude_cal_refresh_buttons.get(camera) if hasattr(self, "px4_attitude_cal_refresh_buttons") else None
        if button is not None:
            button.setEnabled(using_real_params)
            button.setToolTip("" if using_real_params else "MAVLink 연결 후 사용 가능")

    def refresh_px4_attitude_cal_status(self, camera: str | None = None, *, read_params: bool = False) -> None:
        if not hasattr(self, "px4_attitude_cal_managers"):
            return
        cameras = ("left", "right") if camera is None else (camera,)
        for side in cameras:
            manager = self.px4_attitude_cal_managers.get(side)
            if manager is None:
                continue
            if not self._px4_mag_manager_has_data_source(side):
                self._clear_px4_attitude_cal_ui(side)
                continue
            using_real_params = self._sync_px4_attitude_cal_store(side)
            if using_real_params and not read_params:
                self._render_px4_attitude_cal_pending_status(side)
                continue
            if using_real_params and read_params:
                self._start_pixhawk_parameter_worker(side, "attitude_refresh")
                continue
            status = manager.refresh()
            self._render_px4_attitude_cal_status(side, status, using_real_params=using_real_params)

    def _px4_mag_role_for_device_id(self, device_id: int | None) -> str | None:
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

    def _px4_mag_priority_slot_for_key(self, camera: str, key: object) -> int | None:
        slot = mag_priority_slot_from_key(key)
        if slot is not None:
            return slot
        role = str(key or "").strip()
        status_cache = getattr(self, "_px4_mag_status_cache", {})
        status = status_cache.get(camera) if isinstance(status_cache, dict) else None
        for candidate in getattr(status, "slots", ()) or ():
            if self._px4_mag_role_for_device_id(getattr(candidate, "device_id", None)) == role:
                return int(getattr(candidate, "slot"))
        return None

    def apply_manual_mag_priorities_from_panel(self, camera: str = "left", *, panel_key: str | None = None) -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용 차단: MAVLink MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용 차단: Disarm required")
            self.refresh_px4_mag_manager(camera, read_params=True)
            return
        using_mavlink_params = self._sync_px4_mag_manager_store(camera)
        if not using_mavlink_params:
            self._log(
                f"[PX4 MAG] {label} 수동 Priority 적용 차단: "
                "MAVLink PARAM 연결 후 다시 시도하세요."
            )
            self.refresh_px4_mag_manager(camera, read_params=True)
            return
        # Read from the specific panel instance whose button triggered this
        # (falling back to any registered panel for this camera) so a manual
        # priority edit made in one panel isn't silently discarded in favor
        # of another simultaneously-visible panel's values.
        spins = self._px4_mag_registry_get(self.px4_mag_priority_spins, camera, panel_key) or {}
        desired = {slot_key: _px4_priority_combo_value(spin) for slot_key, spin in spins.items()}
        if using_mavlink_params:
            if self._start_pixhawk_parameter_worker(
                camera,
                "mag_apply",
                priorities=desired,
            ):
                return
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용이 이미 진행 중입니다.")
            return
        try:
            status = manager.refresh()
            applied: list[str] = []
            missing = set(desired)
            for slot in status.slots:
                slot_key = mag_priority_slot_key(slot.slot)
                if slot_key not in desired:
                    continue
                name = f"CAL_MAG{slot.slot}_PRIO"
                manager.store.set_param(name, desired[slot_key])
                applied.append(f"{slot_key}:{name}={desired[slot_key]}")
                missing.discard(slot_key)
            if not applied:
                raise RuntimeError("적용 가능한 BOARD/CAN/GPS MAG 슬롯을 찾지 못했습니다.")
            status = manager.refresh()
        except Exception as exc:
            self._log(f"[PX4 MAG] {label} 수동 Priority 적용 실패: {exc}")
            self.refresh_px4_mag_manager(camera, read_params=True)
            return
        self._log(f"[PX4 MAG] {label} 수동 Priority 적용: {', '.join(applied)}")
        if missing:
            self._log(f"[PX4 MAG] {label} 수동 Priority 미적용 role: {', '.join(sorted(missing))}")
        self._render_px4_mag_status(camera, status)

    def save_px4_mag_params_from_panel(self, camera: str = "left") -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} param save 차단: MAVLink MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} param save 차단: Disarm required")
        else:
            if self.mavlink_pixhawk is not None and camera in self._mavlink_connected_sides():
                if self._start_pixhawk_shell_worker(
                    camera,
                    "param save",
                    timeout_s=2.0,
                    on_complete=lambda _result, camera=camera: self._start_pixhawk_parameter_worker(
                        camera, "mag_refresh"
                    ),
                ):
                    return
            else:
                self._log(f"[PX4 MAG] {label} QGC 또는 PX4 shell에서 `param save` 후 reboot 필요")
        self.refresh_px4_mag_manager(camera, read_params=True)

    def reboot_px4_from_panel(self, camera: str = "left") -> None:
        manager = self.px4_mag_managers[camera]
        label = self._px4_mag_camera_label(camera)
        if not self._px4_mag_manager_has_data_source(camera):
            self._log(f"[PX4 MAG] {label} reboot 차단: MAVLink MAG 수신 없음")
            self._clear_px4_mag_manager_ui(camera)
            return
        if manager.armed:
            self._log(f"[PX4 MAG] {label} reboot 차단: Disarm required")
            self.refresh_px4_mag_manager(camera)
            return
        if self.mavlink_pixhawk is None or camera not in self._mavlink_connected_sides():
            self._log(f"[PX4 MAG] {label} PX4 재부팅: QGC 또는 PX4 shell에서 수행 필요")
            self.refresh_px4_mag_manager(camera)
            return
        result = QtWidgets.QMessageBox.question(
            self.widget,
            f"Reboot {label}",
            f"{label} Pixhawk에 MAVLink 재부팅 명령을 보냅니다.\n계속할까요?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result == QtWidgets.QMessageBox.Yes:
            self._start_pixhawk_shell_worker(
                camera,
                "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
                operation="reboot",
                timeout_s=1.0,
            )
        else:
            self._log(f"[PX4 MAG] {label} MAVLink 재부팅 명령을 취소했습니다.")
        self.refresh_px4_mag_manager(camera)

    def _render_px4_mag_status(self, camera: str, status) -> None:
        self._px4_mag_status_cache[camera] = status
        if not hasattr(self, "px4_mag_tables"):
            return
        # Update every panel instance registered for this camera (manager
        # panel(s) and the detail panel) rather than only whichever panel
        # happened to be built most recently.
        for priority_controls in self._px4_mag_registry_values(self.px4_mag_priority_spins, camera):
            for slot in status.slots:
                slot_key = mag_priority_slot_key(slot.slot)
                if slot_key not in priority_controls or slot.priority is None:
                    continue
                _set_px4_priority_combo_value(priority_controls[slot_key], slot.priority)
        tables = self._px4_mag_registry_values(self.px4_mag_tables, camera)
        status_labels = self._px4_mag_registry_values(self.px4_mag_status_labels, camera)
        if not status_labels:
            return
        for table in tables:
            table.setRowCount(len(status.slots))
            for row, slot in enumerate(status.slots):
                values = [
                    f"CAL_MAG{slot.slot}",
                    "N/A" if slot.device_id is None else str(slot.device_id),
                    slot.device_type,
                    slot.interface,
                    "true" if slot.is_rm3100 else "false",
                    mag_priority_display(slot.priority),
                    "true" if slot.enabled else "false",
                    "true" if slot.expected_selected else "false",
                    "true" if slot.actual_selected else "false",
                    slot.status,
                ]
                for column, value in enumerate(values):
                    item = QtWidgets.QTableWidgetItem(value)
                    if slot.is_rm3100:
                        item.setForeground(QtGui.QColor(SUCCESS_TEXT))
                    if slot.actual_selected:
                        item.setForeground(QtGui.QColor(ACCENT))
                    elif slot.expected_selected:
                        item.setForeground(QtGui.QColor(WARNING_TEXT))
                    item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                    table.setItem(row, column, item)
            table.resizeRowsToContents()
            table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        expected = status.expected_selected_device_id
        actual = status.actual_selected_device_id
        warning = status.warning_text or "No warnings."
        label = self._px4_mag_camera_label(camera)
        text = (
            f"{label} | Expected selected MAG: {'N/A' if expected is None else expected} | "
            f"Actual selected MAG: {'N/A' if actual is None else actual} | "
            "Save/Reboot: QGC 또는 PX4 shell에서 `param save` 후 reboot 필요\n"
            f"{warning}"
        )
        for status_label in status_labels:
            status_label.setText(text)
        disable_actions = bool(status.armed)
        self._set_px4_mag_manager_button_state(
            camera,
            refresh_enabled=True,
            actions_enabled=not disable_actions,
            tooltip="Disarm required" if disable_actions else "",
        )

    def _append_px4_shell_output(self, camera: str, text: str) -> None:
        self._handle_eth0_probe_output(camera, text)
        dialog = self.px4_shell_dialogs.get(camera)
        if dialog is not None:
            dialog.append_output(text)

    def _request_mavlink_eth0_status(self, target_sides: Sequence[str]) -> None:
        if self.mavlink_pixhawk is None:
            return
        connected_sides = self._mavlink_connected_sides()
        for camera in target_sides:
            if camera not in connected_sides:
                continue
            self._clear_eth0_probe(camera)
            self._eth0_probe_counter += 1
            token = self._eth0_probe_counter
            self._eth0_probe_tokens[camera] = token
            self._eth0_probe_buffers[camera] = ""
            label = camera_label_ko(camera)
            if not self._start_pixhawk_shell_worker(camera, "ifconfig eth0", timeout_s=2.0):
                self._clear_eth0_probe(camera)
                continue
            configured_ip = (
                self.left_pixhawk_eth_ip_edit.text().strip()
                if camera == "left"
                else self.right_pixhawk_eth_ip_edit.text().strip()
            )
            self._log(
                f"[픽스호크] {label} 실제 eth0 확인 요청: ifconfig eth0 "
                f"(GUI 설정값={configured_ip or '<empty>'})"
            )
            timer = self._eth0_probe_timers[camera]
            timer.stop()
            self._eth0_probe_timeout_tokens[camera] = token
            timer.start()

    def _handle_eth0_probe_output(self, camera: str, text: str) -> None:
        if camera not in self._eth0_probe_tokens:
            return
        buffer = self._eth0_probe_buffers.get(camera, "") + text
        self._eth0_probe_buffers[camera] = buffer
        lower = buffer.lower()
        if "eth0" not in lower:
            return
        match = re.search(r"(?:inet addr:|inet\s+)(\d{1,3}(?:\.\d{1,3}){3})", buffer)
        if match:
            ip_text = match.group(1)
            try:
                IPv4Address(ip_text)
            except ValueError:
                return
            self._clear_eth0_probe(camera)
            self._log(f"[픽스호크] {camera_label_ko(camera)} 실제 eth0 IP 확인: {ip_text}")
            return
        if any(marker in lower for marker in ("no such device", "not found", "invalid interface", "does not exist")):
            self._clear_eth0_probe(camera)
            self._log(f"[픽스호크] {camera_label_ko(camera)} 실제 eth0 확인 실패: eth0 인터페이스를 찾지 못했습니다.")

    def _finish_eth0_probe_if_pending(self, camera: str, token: int) -> None:
        if self._eth0_probe_tokens.get(camera) != token:
            return
        self._clear_eth0_probe(camera)
        self._log(
            f"[픽스호크] {camera_label_ko(camera)} 실제 eth0 확인 실패: "
            "PX4 shell 응답이 없어 MAVLink만으로 현재 eth0 IP를 확인할 수 없습니다."
        )

    def _eth0_probe_timeout_fired(self, camera: str) -> None:
        token = self._eth0_probe_timeout_tokens.get(camera)
        if token is None:
            return
        self._finish_eth0_probe_if_pending(camera, token)

    def _clear_eth0_probe(self, camera: str) -> None:
        timer = self._eth0_probe_timers.get(camera)
        if timer is not None:
            timer.stop()
        self._eth0_probe_timeout_tokens.pop(camera, None)
        self._eth0_probe_tokens.pop(camera, None)
        self._eth0_probe_buffers.pop(camera, None)

    def _pixhawk_host_ip_for_mavlink(self, *, extra_hints: Sequence[str] = ()) -> str:
        text = self.pixhawk_host_address_edit.text().strip()
        if "/" in text:
            text = text.split("/", 1)[0]
        if not text:
            raise ValueError("Pixhawk Host IP가 비어 있어 Ethernet MAVLink를 시작할 수 없습니다.")
        try:
            IPv4Address(text)
        except ValueError as exc:
            raise ValueError(f"Pixhawk Host IP 형식이 올바르지 않습니다: {text}") from exc
        # Host-interface discovery is an explicit asynchronous Workstation
        # command.  Do not rescan NetworkManager from a PX4 command handler;
        # the selected candidate is already the operator's input here.
        return text

    def _active_mavlink_remote_hosts(self) -> tuple[str, ...]:
        subscriber = self.mavlink_pixhawk
        if subscriber is None:
            return ()
        return self._mavlink_remote_hosts_for_sides(subscriber, tuple(getattr(subscriber, "connections", {}).keys()))

    def _mavlink_remote_hosts_for_sides(
        self,
        subscriber: DualPixhawkMavlinkSubscriber,
        sides: Sequence[str],
    ) -> tuple[str, ...]:
        hosts: list[str] = []
        side_set = set(sides)
        for connection in getattr(subscriber, "connections", {}).values():
            side = next(
                (
                    key
                    for key, candidate in getattr(subscriber, "connections", {}).items()
                    if candidate is connection
                ),
                "",
            )
            if side_set and side not in side_set:
                continue
            clients = getattr(connection, "clients", None)
            if not clients:
                continue
            for client in clients:
                if isinstance(client, tuple) and client:
                    hosts.append(str(client[0]))
        return tuple(dict.fromkeys(hosts))

    def _build_mavlink_baud_spin(self) -> QtWidgets.QSpinBox:
        spin = QtWidgets.QSpinBox()
        spin.setRange(1_200, 4_000_000)
        spin.setValue(57_600)
        spin.setSingleStep(9_600)
        return spin

    def _mount_assumption_status_label(self) -> str:
        if self._distance_mode() == "checkerboard":
            return "checkerboard stereo extrinsics"
        return "GPS/RTK baseline only"

    def _format_mount_assumption_summary(self) -> str:
        if self._distance_mode() == "checkerboard":
            return (
                "mount_assumption:\n"
                "  model: checkerboard-stereo-extrinsics-v1\n"
                "  calculation: fixed_left_to_right_optical_rdf_transform_only"
            )
        return (
            "mount_assumption:\n"
            "  model: gps-rtk-baseline-only-v1\n"
            "  calculation: live_gps_rtk_baseline_and_pixhawk_attitude_only"
        )

    def _set_combo_data(self, combo: QtWidgets.QComboBox, value: str) -> None:
        index = combo.findData(value)
        if index >= 0:
            combo.setCurrentIndex(index)

    def _mavlink_stream_port(self, url: str, *, default: int) -> int:
        text = url.strip().rsplit(":", 1)[-1]
        if "/" in text:
            text = text.split("/", 1)[0]
        try:
            return int(text)
        except ValueError:
            return int(default)

    def workstation_discover_custom_mavlink_watch(self) -> None:
        self.discover_custom_mavlink_watch()

    def workstation_add_custom_mavlink_watch(self, name: str, message: str, field: str) -> str:
        message = str(message or "").strip().upper()
        field = str(field or "").strip()
        name = str(name or "").strip() or f"mavlink:{message}.{field}"
        if not message or not field:
            raise ValueError("MAVLink custom watch requires message and field")
        self._add_custom_mavlink_watch(
            key=name,
            topic=message,
            field=field,
            message=message,
        )
        self._log(f"[픽스호크] Workstation MAVLink watch 추가: {name} = {message}.{field}")
        return name

    def workstation_remove_custom_mavlink_watch(self, key: str) -> None:
        key = str(key or "").strip()
        if not key:
            raise ValueError("custom watch key is empty")
        self.remove_custom_mavlink_watch(key)

    def workstation_refresh_baseline_field_browser(self) -> dict[str, object]:
        mavlink_rows = tuple(
            (
                str(item.get("name") or ""),
                ", ".join(mavlink_graphable_fields_for_message(str(item.get("name") or ""))),
            )
            for item in discover_mavlink_messages()
            if str(item.get("name") or "")
        )
        return {
            "mavlink_messages": mavlink_rows,
            "mavlink_headers": ("message", "fields"),
            "axis_contract": "GPS/RTK source fields remain producer-labelled; no NED/ENU/camera-frame conversion",
        }

    def workstation_open_px4_shell(self, camera: str) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        self.open_px4_shell(camera)

    def workstation_refresh_px4_mag(self, camera: str, *, read_params: bool = True) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        if read_params and self._start_pixhawk_parameter_worker(camera, "mag_refresh"):
            return
        self.refresh_px4_mag_manager(camera, read_params=bool(read_params))

    def workstation_refresh_px4_board_rotation(self, camera: str) -> bool:
        """Read only SENS_BOARD_ROT so internal-MAG corrected yaw can be shown."""
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        return bool(self._start_pixhawk_parameter_worker(camera, "board_rotation_refresh"))

    def workstation_refresh_px4_attitude_cal(self, camera: str, *, read_params: bool = True) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        if read_params and self._start_pixhawk_parameter_worker(camera, "attitude_refresh"):
            return
        self.refresh_px4_attitude_cal_status(camera, read_params=bool(read_params))

    def workstation_refresh_px4_calibration_params(self, camera: str, *, read_params: bool = True) -> None:
        """Read the QGC calibration parameter set for one Pixhawk."""

        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        if read_params and self._start_pixhawk_parameter_worker(camera, "calibration_refresh"):
            return
        self._log(
            f"[픽스호크] {self._px4_mag_camera_label(camera)} QGC calibration PARAM readback을 시작할 수 없습니다."
        )

    def workstation_apply_px4_mag_priorities(self, camera: str, priorities: dict[str, int]) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        spins = self.px4_mag_priority_spins.get(camera, {})
        for key, value in dict(priorities).items():
            slot = self._px4_mag_priority_slot_for_key(camera, key)
            normalized_key = mag_priority_slot_key(slot) if slot is not None else str(key)
            spin = spins.get(normalized_key)
            if spin is None and str(key) in spins:
                normalized_key = str(key)
                spin = spins.get(normalized_key)
            if spin is None:
                raise ValueError(f"Unknown PX4 MAG priority key: {key}")
            _set_px4_priority_combo_value(spin, int(value))
        if self._start_pixhawk_parameter_worker(
            camera,
            "mag_apply",
            priorities={
                (
                    mag_priority_slot_key(slot)
                    if (slot := self._px4_mag_priority_slot_for_key(camera, key)) is not None
                    else str(key)
                ): int(value)
                for key, value in dict(priorities).items()
            },
        ):
            return
        self.apply_manual_mag_priorities_from_panel(camera)

    def workstation_save_px4_mag_params(self, camera: str) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        self._start_pixhawk_shell_worker(
            camera,
            "param save",
            timeout_s=2.0,
            on_complete=lambda _result, camera=camera: self._start_pixhawk_parameter_worker(
                camera, "mag_refresh"
            ),
        )

    def workstation_reboot_px4(self, camera: str) -> None:
        camera = str(camera or "")
        if camera not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {camera}")
        manager = self.px4_mag_managers.get(camera)
        if manager is None or manager.armed:
            self._log(f"[PX4 MAG] {self._px4_mag_camera_label(camera)} 재부팅 차단: Disarm required")
            return
        if self.mavlink_pixhawk is None or camera not in self._mavlink_open_sides():
            self._log(f"[PX4 MAG] {self._px4_mag_camera_label(camera)} 재부팅 차단: MAVLink 연결 없음")
            return
        result = QtWidgets.QMessageBox.question(
            self.widget,
            f"Reboot {self._px4_mag_camera_label(camera)}",
            f"{self._px4_mag_camera_label(camera)} Pixhawk에 MAVLink 재부팅 명령을 보냅니다.\n계속할까요?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if result != QtWidgets.QMessageBox.Yes:
            self._log(f"[PX4 MAG] {self._px4_mag_camera_label(camera)} 재부팅을 취소했습니다.")
            return
        self._start_pixhawk_shell_worker(
            camera,
            "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
            operation="reboot",
            timeout_s=1.0,
        )

    def _pixhawk_candidate_table_for_source(self, source: str) -> QtWidgets.QTableWidget:
        return self.views.connection._pixhawk_candidate_table_for_source(source)

    def _pixhawk_candidate_source_for_kind(self, kind: str) -> str:
        return self.views.connection._pixhawk_candidate_source_for_kind(kind)

    def _sync_legacy_pixhawk_candidate_rows(self) -> None:
        return self.views.connection._sync_legacy_pixhawk_candidate_rows()

    def _set_pixhawk_candidate_rows(self, rows: list[dict[str, object]], *, source: str | None = None) -> None:
        return self.views.connection._set_pixhawk_candidate_rows(rows, source=source)

    def _render_pixhawk_candidate_rows(self, source: str, rows: list[dict[str, object]]) -> None:
        return self.views.connection._render_pixhawk_candidate_rows(source, rows)

    def _append_pixhawk_candidate_rows(self, rows: list[dict[str, object]], *, source: str | None = None) -> None:
        return self.views.connection._append_pixhawk_candidate_rows(rows, source=source)

    def clear_pixhawk_candidate_history(self, source: str | None = None) -> None:
        return self.views.connection.clear_pixhawk_candidate_history(source)

    def _refresh_pixhawk_candidate_actions(self) -> None:
        return self.views.connection._refresh_pixhawk_candidate_actions()

    def _mavlink_url_candidate_rows(self) -> list[dict[str, object]]:
        return self.views.connection._mavlink_url_candidate_rows()

    def _selected_pixhawk_candidate(self, source: str | None = None) -> dict[str, object] | None:
        return self.views.connection._selected_pixhawk_candidate(source)

    def _pixhawk_candidate_discovery_active(self) -> bool:
        return any(
            getattr(self, name, None) is not None
            for name in (
                "pixhawk_usb_discovery_thread",
                "pixhawk_host_ip_discovery_thread",
                "pixhawk_network_discovery_thread",
            )
        )


    def _begin_pixhawk_candidate_discovery(
        self,
        status: str,
        *,
        clear_usb_identity: bool = False,
    ) -> None:
        """Replace the transient candidate snapshot before one accepted search."""

        self._set_pixhawk_candidate_rows([])
        if clear_usb_identity:
            self.pixhawk_usb_identity_evidence = []
        self.pixhawk_discovery_revision += 1
        self.workstation_pixhawk_discovery_status = str(status)


    def discover_mavlink_url_candidates(self) -> None:
        if self._pixhawk_candidate_discovery_active():
            self._log("[픽스호크] 다른 Discovery가 진행 중이어서 network 후보 탐색을 시작하지 않았습니다.")
            return
        self._begin_pixhawk_candidate_discovery("SEARCHING NETWORK")
        rows = self._mavlink_url_candidate_rows()
        self._set_pixhawk_candidate_rows(rows, source="mavlink")
        self.workstation_pixhawk_discovery_status = f"FOUND {len(rows)} NETWORK ENDPOINTS"
        for row in rows:
            self._log(
                f"[픽스호크] MAVLink URL 후보: {row['value']} "
                f"status={row['status']} host={row['interface']}"
            )


    def discover_usb_pixhawk_connections(self) -> None:
        if self.pixhawk_usb_discovery_thread is not None:
            self._log("[픽스호크] USB Pixhawk 연결 탐색이 이미 진행 중입니다.")
            return
        if self._pixhawk_candidate_discovery_active():
            self._log("[픽스호크] 다른 Discovery가 진행 중이어서 USB 탐색을 시작하지 않았습니다.")
            return
        self._begin_pixhawk_candidate_discovery(
            "SEARCHING USB",
            clear_usb_identity=True,
        )
        self.pixhawk_usb_discover_button.setEnabled(False)
        self._log("[픽스호크] USB Pixhawk 연결 탐색을 시작합니다. 후보는 자동 할당하지 않습니다.")
        self.pixhawk_usb_discovery_thread = QtCore.QThread(self.widget)
        self.pixhawk_usb_discovery_worker = PixhawkUsbDiscoveryWorker(timeout_s=0.8)
        self.pixhawk_usb_discovery_worker.moveToThread(self.pixhawk_usb_discovery_thread)
        self.pixhawk_usb_discovery_thread.started.connect(self.pixhawk_usb_discovery_worker.run)
        self.pixhawk_usb_discovery_worker.discovered.connect(self._apply_pixhawk_usb_discovery)
        self.pixhawk_usb_discovery_worker.failed.connect(self._pixhawk_usb_discovery_failed)
        self.pixhawk_usb_discovery_worker.finished.connect(self.pixhawk_usb_discovery_thread.quit)
        self.pixhawk_usb_discovery_worker.finished.connect(self.pixhawk_usb_discovery_worker.deleteLater)
        self.pixhawk_usb_discovery_thread.finished.connect(self.pixhawk_usb_discovery_thread.deleteLater)
        self.pixhawk_usb_discovery_thread.finished.connect(self._clear_pixhawk_usb_discovery)
        self.pixhawk_usb_discovery_thread.start()


    def _apply_pixhawk_usb_discovery(self, result: object) -> None:
        data = dict(result or {}) if isinstance(result, dict) else {}
        candidates = list(data.get("candidates") or [])
        found = list(data.get("found") or [])
        identities: list[dict[str, object]] = []
        for item in found:
            if isinstance(item, dict):
                identity = dict(item)
            else:
                try:
                    device, baud = item
                except (TypeError, ValueError):
                    continue
                identity = {
                    "device": str(device),
                    "resolved_device": str(device),
                    "baud": int(baud),
                    "system_id": None,
                    "component_id": None,
                    "observed_monotonic_s": None,
                }
            identities.append(identity)
        self.pixhawk_usb_identity_evidence = identities
        rows: list[dict[str, object]] = []
        found_by_device = {
            str(identity.get("device") or ""): identity
            for identity in identities
            if str(identity.get("device") or "")
        }
        for device in candidates:
            identity = found_by_device.get(str(device), {})
            baud = identity.get("baud", "") if isinstance(identity, dict) else ""
            system_id = identity.get("system_id") if isinstance(identity, dict) else None
            component_id = identity.get("component_id") if isinstance(identity, dict) else None
            if baud and system_id is not None and component_id is not None:
                status = f"MAVLink heartbeat sysid={system_id} compid={component_id}"
            elif baud:
                status = "MAVLink heartbeat (identity unavailable; recovery disabled)"
            else:
                status = self._usb_pixhawk_probe_status(device)
            row = {
                "kind": "USB",
                "value": device,
                "resolved_device": str(identity.get("resolved_device") or device)
                if isinstance(identity, dict)
                else str(device),
                "baud": baud,
                "status": status,
                "system_id": system_id,
                "component_id": component_id,
                "observed_monotonic_s": identity.get("observed_monotonic_s") if isinstance(identity, dict) else None,
            }
            rows.append(row)
        if not rows:
            self._set_pixhawk_candidate_rows([])
            self.workstation_pixhawk_discovery_status = "FOUND 0 USB ENDPOINTS"
            self._log("[픽스호크] USB Pixhawk 장치가 보이지 않습니다: /dev/ttyACM*, /dev/ttyUSB* 없음")
            return
        self._set_pixhawk_candidate_rows(rows, source="mavlink")
        self.workstation_pixhawk_discovery_status = f"FOUND {len(rows)} USB ENDPOINTS"
        for row in rows:
            self._log(
                f"[픽스호크] USB 후보: {row['value']} "
                f"baud={row['baud'] or '-'} status={row['status']}"
            )


    def _pixhawk_usb_discovery_failed(self, error: str) -> None:
        self._set_pixhawk_candidate_rows([])
        self.pixhawk_usb_identity_evidence = []
        self.workstation_pixhawk_discovery_status = f"ERROR · {error}"
        self._log(f"[픽스호크] USB Pixhawk 연결 탐색 실패: {error}")


    def _clear_pixhawk_usb_discovery(self) -> None:
        self.pixhawk_usb_discovery_thread = None
        self.pixhawk_usb_discovery_worker = None
        self.pixhawk_usb_discover_button.setEnabled(True)


    def _usb_pixhawk_probe_status(self, device: str) -> str:
        try:
            with open(device, "rb", buffering=0):
                pass
        except PermissionError:
            return "permission denied: add user to dialout"
        except OSError as exc:
            return f"open failed: {exc}"
        return "no heartbeat"


    def apply_pixhawk_candidate(self, source: str, row: int, target: str) -> bool:
        """Apply a discovered endpoint selected by the Workstation surface."""

        if self._session_recording_active():
            self._log("[픽스호크] 데이터 기록 중에는 Pixhawk Host/P1/P2 후보를 적용할 수 없습니다.")
            return False
        if source not in {"pc", "mavlink"} or target not in {"host", "left", "right"}:
            self._log("[픽스호크] 후보 source 또는 적용 대상이 올바르지 않습니다.")
            return False
        rows = self._pixhawk_candidate_rows_by_source.get(source, [])
        if isinstance(row, bool) or not isinstance(row, int) or not 0 <= row < len(rows):
            self._log("[픽스호크] 적용할 탐색 후보가 올바르지 않습니다.")
            return False
        self._pixhawk_candidate_table_for_source(source).selectRow(row)
        self.apply_selected_pixhawk_candidate(target, source=source)
        return True


    def swap_pixhawks(self) -> bool | None:
        if self._session_recording_active():
            self._log("[픽스호크] 데이터 기록 중에는 P1/P2 Pixhawk 매핑을 교체할 수 없습니다.")
            return False
        mavlink_connected_sides = self._mavlink_open_sides()
        for side in ("left", "right"):
            if side in mavlink_connected_sides:
                self.disconnect_mavlink_pixhawk(side)
        self.left_mavlink_url_edit.setText(swap_line_edit_text(self.left_mavlink_url_edit, self.right_mavlink_url_edit))
        self.left_pixhawk_eth_ip_edit.setText(
            swap_line_edit_text(self.left_pixhawk_eth_ip_edit, self.right_pixhawk_eth_ip_edit)
        )
        self.left_mavlink_baud_spin.setValue(
            int(swap_spinbox_value(self.left_mavlink_baud_spin, self.right_mavlink_baud_spin))
        )
        self.left_pixhawk_bootstrap_url_edit.setText(
            swap_line_edit_text(self.left_pixhawk_bootstrap_url_edit, self.right_pixhawk_bootstrap_url_edit)
        )
        self.left_pixhawk_bootstrap_baud_spin.setValue(
            int(swap_spinbox_value(self.left_pixhawk_bootstrap_baud_spin, self.right_pixhawk_bootstrap_baud_spin))
        )
        assignments = getattr(self, "pixhawk_network_assignments", {})
        if isinstance(assignments, dict):
            left_assignment = dict(assignments.get("left") or {})
            assignments["left"] = dict(assignments.get("right") or {})
            assignments["right"] = left_assignment
        assignment_status = getattr(self, "pixhawk_network_assignment_status", {})
        if isinstance(assignment_status, dict):
            left_status = dict(assignment_status.get("left") or {})
            assignment_status["left"] = dict(assignment_status.get("right") or {})
            assignment_status["right"] = left_status
        self._save_current_settings()
        self._log(
            "[픽스호크] Pixhawk 1/2 MAVLink 매핑을 교체했습니다. "
            "기존 연결은 복구하지 않으며 각 Connect를 다시 누르세요."
        )
        return None


    def _update_pixhawk_connection_status(self) -> None:
        now_s = time.monotonic()
        parts = []
        health_reader = getattr(self.mavlink_pixhawk, "mavlink_health_snapshot", None)
        for camera in ("left", "right"):
            if callable(health_reader):
                try:
                    health = health_reader(camera, now_s=now_s)
                except Exception:
                    health = None
                if health is not None:
                    state = str(
                        getattr(
                            getattr(health, "state", None),
                            "value",
                            getattr(health, "state", "UNKNOWN"),
                        )
                    )
                    if state in {"QUALIFYING", "RECOVERING"}:
                        elapsed_s = max(0, int(getattr(health, "qualification_elapsed_s", 0.0)))
                        required_s = max(
                            1,
                            int(getattr(health, "qualification_required_s", MAVLINK_QUALIFICATION_S)),
                        )
                        state = (
                            f"{state} {elapsed_s // 60:02d}:{elapsed_s % 60:02d} / "
                            f"{required_s // 60:02d}:{required_s % 60:02d}"
                        )
                    parts.append(f"{camera_display_name(camera)}={state}")
                    continue
            timestamps = self.pixhawk_attitude_times[camera]
            if not timestamps:
                parts.append(f"{camera_display_name(camera)}=대기")
                continue
            age_s = now_s - timestamps[-1]
            state = "connected" if age_s < 1.0 else "stale"
            state_text = "연결" if state == "connected" else "지연"
            parts.append(f"{camera_display_name(camera)}={state_text} {sample_hz(timestamps):.1f} Hz")
        text = f"픽스호크 MAVLink: {', '.join(parts)}"
        if self.pixhawk_connection_label.text() != text:
            self.pixhawk_connection_label.setText(text)
        self._refresh_operation_header()


    def build_legacy_pixhawk_connection_controls(self):
        self.views.connection = PixhawkConnectionView(
            PixhawkConnectionSettings(
                drone_pixhawk_bootstrap_baud=self.settings.drone_pixhawk_bootstrap_baud,
                drone_pixhawk_bootstrap_url=self.settings.drone_pixhawk_bootstrap_url,
                left_mavlink_baud=self.settings.left_mavlink_baud,
                left_mavlink_url=self.settings.left_mavlink_url,
                left_pixhawk_bootstrap_baud=self.settings.left_pixhawk_bootstrap_baud,
                left_pixhawk_bootstrap_url=self.settings.left_pixhawk_bootstrap_url,
                left_pixhawk_eth_ip=self.settings.left_pixhawk_eth_ip,
                mavlink_stream_rate=self.settings.mavlink_stream_rate,
                pixhawk_ethernet_mode=self.settings.pixhawk_ethernet_mode,
                pixhawk_host_address=self.settings.pixhawk_host_address,
                right_mavlink_baud=self.settings.right_mavlink_baud,
                right_mavlink_url=self.settings.right_mavlink_url,
                right_pixhawk_bootstrap_baud=self.settings.right_pixhawk_bootstrap_baud,
                right_pixhawk_bootstrap_url=self.settings.right_pixhawk_bootstrap_url,
                right_pixhawk_eth_ip=self.settings.right_pixhawk_eth_ip,

            ),
            PixhawkConnectionCommands(
                discover_usb_pixhawk_connections=self.discover_usb_pixhawk_connections,
                discover_mavlink_url_candidates=self.discover_mavlink_url_candidates,
                apply_selected_pixhawk_candidate=self.apply_selected_pixhawk_candidate,
                swap_pixhawks=self.swap_pixhawks,
                discover_pc_pixhawk_ip=self.discover_pc_pixhawk_ip,
                toggle_mavlink_connection=self.toggle_mavlink_connection,
                bootstrap_ethernet_mavlink_from_usb=self.bootstrap_ethernet_mavlink_from_usb,
                open_px4_shell=self.open_px4_shell,
                apply_persistent_pixhawk_eth=self.apply_persistent_pixhawk_eth,
                reboot_pixhawk_from_mavlink=self.reboot_pixhawk_from_mavlink,
                _refresh_pixhawk_ethernet_mode_controls=self._refresh_pixhawk_ethernet_mode_controls,

            ),
            build_toggle=self._build_toggle_button, log=self._log, parent=self.widget,
        )
        self.pixhawk_network_assignments: dict[str, dict[str, str]] = {
            "left": {
                "interface": str(getattr(self.settings, "left_mavlink_interface", "") or ""),
                "local_address": str(
                    getattr(self.settings, "left_mavlink_local_address", "") or ""
                ),
            },
            "right": {
                "interface": str(getattr(self.settings, "right_mavlink_interface", "") or ""),
                "local_address": str(
                    getattr(self.settings, "right_mavlink_local_address", "") or ""
                ),
            },
        }
        self.pixhawk_network_assignment_status: dict[str, dict[str, str]] = {
            side: {
                "state": "ASSIGNED" if values["interface"] else "LEGACY AUTO ROUTE",
                "detail": "saved operator assignment" if values["interface"] else "no explicit NIC assignment",
            }
            for side, values in self.pixhawk_network_assignments.items()
        }

    def build_legacy_pixhawk_connection_layout(self, pixhawk_connection_layout, pixhawk_form, pixhawk_group):
        self.views.connection.build_layout(pixhawk_connection_layout, pixhawk_form, pixhawk_group)

    def legacy_pixhawk_network_assignment_snapshot(self):
        return {side: dict(self.pixhawk_network_assignments.get(side) or {}) for side in ("left", "right")}

    def build_sensor_tabs(self, advanced_tabs, alignment_attitude_layout, alignment_mag_layout, alignment_overview_layout, alignment_tabs, baseline_field_browser_layout, baseline_field_browser_tab, connection_tabs, gnss_base_layout, mag_management_layout, optical_alignment_layout, px4_mag_advanced_layout, px4_mag_advanced_tab, rtk_layout, setup_layout, wiris_command_panel):
        self.left_gps_label = QtWidgets.QLabel("픽스호크 1 GPS/RTK: 미연결")
        self.right_gps_label = QtWidgets.QLabel("픽스호크 2 GPS/RTK: 미연결")
        self.rtk_baseline_label = QtWidgets.QLabel("GPS baseline: 대기 중")
        self.rtk_fixed_status_label = QtWidgets.QLabel("RTK Fixed: 좌/우 GPS 대기 중")
        self.rtk_summary_labels: dict[str, QtWidgets.QLabel] = {}
        for key, text in (
            ("baseline", "Baseline: 대기 중"),
            ("camera", "Camera: 자세 대기 중"),
            ("status", "Status: GPS 대기 중"),
        ):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statChip")
            label.setWordWrap(True)
            self.rtk_summary_labels[key] = label
        for rtk_label in (self.left_gps_label, self.right_gps_label, self.rtk_baseline_label, self.rtk_fixed_status_label):
            rtk_label.setObjectName("statusStrip")
            rtk_label.setWordWrap(True)
        setup_layout.addWidget(connection_tabs, 1)
        pixhawk_tabs = QtWidgets.QTabWidget()
        pixhawk_summary_tab = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(pixhawk_summary_tab)
        pixhawk_1_tab = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(pixhawk_1_tab)
        pixhawk_2_tab = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(pixhawk_2_tab)
        pixhawk_attitude_tab = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(pixhawk_attitude_tab)
        custom_watch_tab = QtWidgets.QWidget()
        custom_watch_layout = QtWidgets.QVBoxLayout(custom_watch_tab)
        pixhawk_tabs.addTab(pixhawk_summary_tab, "요약")
        pixhawk_tabs.addTab(pixhawk_1_tab, "픽스호크 1")
        pixhawk_tabs.addTab(pixhawk_2_tab, "픽스호크 2")
        self.pixhawk_tabs = pixhawk_tabs
        advanced_tabs.addTab(wiris_command_panel, "WIRIS SDK")
        advanced_tabs.addTab(custom_watch_tab, "필드 감시")
        advanced_tabs.addTab(baseline_field_browser_tab, "Baseline 필드")
        advanced_tabs.addTab(px4_mag_advanced_tab, "PX4 MAG")
        self._handle_alignment_step_changed(alignment_tabs.currentIndex())
        self.advanced_wiris_command_tab_index = advanced_tabs.indexOf(wiris_command_panel)
        self.advanced_custom_watch_tab_index = advanced_tabs.indexOf(custom_watch_tab)
        self.advanced_baseline_field_tab_index = advanced_tabs.indexOf(baseline_field_browser_tab)
        self.advanced_px4_mag_tab_index = advanced_tabs.indexOf(px4_mag_advanced_tab)
        self.wiris_command_tab_index = self.advanced_wiris_command_tab_index

        custom_watch_group = QtWidgets.QGroupBox("사용자 지정 MAVLink 감시")
        custom_watch_group_layout = QtWidgets.QVBoxLayout(custom_watch_group)
        custom_watch_form = QtWidgets.QGridLayout()
        self.custom_watch_name_edit = QtWidgets.QLineEdit("local_z")
        self.custom_watch_message_edit = QtWidgets.QLineEdit("LOCAL_POSITION_NED")
        self.custom_watch_field_edit = QtWidgets.QLineEdit("z")
        self.custom_watch_add_button = QtWidgets.QPushButton("감시 추가")
        self.custom_watch_add_button.clicked.connect(self.add_custom_mavlink_watch_from_ui)
        self.custom_watch_mavlink_discover_button = QtWidgets.QPushButton("MAVLink 필드 찾기")
        self.custom_watch_mavlink_discover_button.clicked.connect(self.discover_custom_mavlink_watch)
        self.custom_watch_remove_combo = QtWidgets.QComboBox()
        self.custom_watch_remove_combo.setMinimumWidth(260)
        self.custom_watch_remove_button = QtWidgets.QPushButton("감시 제거")
        self.custom_watch_remove_button.clicked.connect(self.remove_selected_custom_topic_watch)
        custom_watch_form.addWidget(QtWidgets.QLabel("이름"), 0, 0)
        custom_watch_form.addWidget(self.custom_watch_name_edit, 0, 1)
        custom_watch_form.addWidget(QtWidgets.QLabel("MAVLink message"), 1, 0)
        custom_watch_form.addWidget(self.custom_watch_message_edit, 1, 1)
        custom_watch_form.addWidget(QtWidgets.QLabel("필드"), 2, 0)
        custom_watch_form.addWidget(self.custom_watch_field_edit, 2, 1)
        custom_watch_buttons = QtWidgets.QHBoxLayout()
        custom_watch_buttons.addWidget(self.custom_watch_mavlink_discover_button)
        custom_watch_buttons.addWidget(self.custom_watch_add_button)
        custom_watch_buttons.addWidget(self.custom_watch_remove_combo)
        custom_watch_buttons.addWidget(self.custom_watch_remove_button)
        custom_watch_buttons.addStretch(1)
        custom_watch_form.addLayout(custom_watch_buttons, 3, 0, 1, 2)
        custom_watch_group_layout.addLayout(custom_watch_form)
        self.custom_watch_plot = TelemetryHistoryPlot("사용자 지정 MAVLink 숫자 필드", "값")
        self.custom_watch_plot.setMinimumHeight(360)
        custom_watch_group_layout.addWidget(self.custom_watch_plot, 1)
        custom_watch_layout.addWidget(custom_watch_group, 1)

        self.mag_sensor_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.gps_sensor_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.mag_sensor_plots: dict[str, TelemetryHistoryPlot] = {}
        self.gps_sensor_plots: dict[str, TelemetryHistoryPlot] = {}
        self.pixhawk_sensor_summary_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.rm3100_slot_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.f9p_slot_labels: dict[str, dict[str, QtWidgets.QLabel]] = {"left": {}, "right": {}}
        self.rm3100_compare_labels: dict[str, QtWidgets.QLabel] = {}
        self.f9p_compare_labels: dict[str, QtWidgets.QLabel] = {}
        self.gnss_heading_labels: dict[str, QtWidgets.QLabel] = {}
        self.rm3100_norm_plots: dict[str, TelemetryHistoryPlot] = {}
        self.rm3100_delta_plots: dict[str, TelemetryHistoryPlot] = {}
        self.f9p_sats_plots: dict[str, TelemetryHistoryPlot] = {}
        self.f9p_quality_plots: dict[str, TelemetryHistoryPlot] = {}
        self._build_baseline_field_browser_view(baseline_field_browser_layout)
        self._init_baseline_alignment_state()
        self._build_alignment_sensor_recognition_tab(alignment_overview_layout)
        self._build_alignment_source_quality_tab(rtk_layout)
        self._build_gnss_base_tab(gnss_base_layout)
        self._build_alignment_magnetometer_tab(
            mag_management_layout,
            include_manager=True,
            include_plot=False,
            expand_manager=True,
        )
        self._build_px4_mag_detail_panel(px4_mag_advanced_layout)
        self._build_baseline_calculation_view(alignment_mag_layout, include_source_controls=False, include_alignment_plot=False)
        self._build_baseline_attitude_alignment_tab(alignment_attitude_layout)
        self._build_optical_alignment_tab(optical_alignment_layout)
        self.refresh_baseline_field_browser()
        self._refresh_baseline_alignment_ui()

