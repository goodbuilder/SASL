"""Gnss feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""

from __future__ import annotations

import time
from collections import (
    deque,
)

from PyQt5 import (
    QtCore,
    QtWidgets,
)

from Workstation.workers.device_runtime_workers import PixhawkRtcmInjectionWorker
from Workstation.controllers.drone_telemetry_controller import normalize_drone_rtcm_source
from Workstation.gnss_runtime.evidence import GnssEvidenceRowsPort
from Workstation.gnss_runtime.state import GnssRuntimeState
from Workstation.workers.gnss_runtime_workers import GnssBaseSerialDetectWorker, GnssBaseSerialWorker, NtripNetworkWorker, RtcmDataPlaneWorker
from Workstation.panel.panel_feature_runtime import PanelFeature
from Workstation.ui.telemetry_plots import TelemetryHistoryPlot
from Workstation.workstation_core.alignment_managers import (
    GNSSMonitor,
)
from Workstation.workstation_core.gnss_operation import (
    DEFAULT_NTRIP_SESSION_PASSWORD,
    GgaPolicy,
    GnssOperationConfig,
    GnssOperationManager,
    GnssOperationMode,
    HeightType,
)
from Workstation.workstation_core.gnss_profile import (
    GnssCoordinateSample,
)
from Workstation.workstation_core.gnss_protocol import (
    Rtcm3StreamParser,
    preferred_gnss_base_serial_port,
)
from Workstation.workstation_core.session_storage.spool import SpoolRows


class GnssFeature(PanelFeature):
    def __init__(self, widget):
        # This existing feature dependency is routed by PanelApplication's
        # EXTERNAL_STATE_FIELDS into RecordingRowsState. Resolve it lazily so
        # session/recovery LiveDataRows or SpoolRows replacements stay visible.
        super().__init__(
            widget,
            state_factory=lambda owner: GnssRuntimeState(
                evidence_rows=GnssEvidenceRowsPort(
                    read=lambda: owner.gnss_base_recording_rows,
                    replace=lambda rows: setattr(owner, "gnss_base_recording_rows", rows),
                ),
            ),
        )

    def resource_inventory(self):
        return {
            **super().resource_inventory(),
            **self.state.ntrip.resource_inventory(),
            **self.state.base.resource_inventory(),
            **self.state.corrections.resource_inventory(),
        }

    feature_name = "gnss"
    dependency_names = (
        "_gnss_runtime_controller_owner",
        "_handle_gnss_operation_mode_changed",
        "_refresh_gnss_base_candidate_actions",
        "_refresh_gnss_base_tab",
        "_refresh_ntrip_credential_status",
        "_set_combo_data",
        "_set_health_label",
        "apply_selected_gnss_base_candidate",
        "connect_gnss_base_serial",
        "disconnect_gnss_base_serial",
        "discover_gnss_base_serial_ports",
        "forget_gnss_ntrip_password",
        "settings",
        "start_gnss_base_survey_in",
        "start_gnss_network",
        "stop_gnss_base_survey_in",
        "stop_gnss_network",
    )
    writable_dependencies = frozenset(())

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.pixhawk_rtcm_thread: QtCore.QThread | None = None
        self.pixhawk_rtcm_worker: PixhawkRtcmInjectionWorker | None = None
        self.gnss_base_serial_thread: QtCore.QThread | None = None
        self.gnss_base_serial_worker: GnssBaseSerialWorker | None = None
        self.gnss_base_serial_connected = False
        self.gnss_network_thread: QtCore.QThread | None = None
        self.gnss_network_worker: NtripNetworkWorker | None = None
        self.rtcm_pipeline_thread: QtCore.QThread | None = None
        self.rtcm_pipeline_worker: RtcmDataPlaneWorker | None = None
        self._rtcm_pipeline_base_worker: GnssBaseSerialWorker | None = None
        self._rtcm_pipeline_injection_worker: PixhawkRtcmInjectionWorker | None = None
        self._rtcm_package_signal_connected = False
        self.gnss_network_connected = False
        self.gnss_network_phase = "DISCONNECTED"
        self.gnss_network_stats: object | None = None
        self.gnss_network_status_line = ""
        self.gnss_network_last_error = ""
        self.gnss_network_reconnect_count = 0
        self._gnss_ntrip_session_password_explicit = False
        self._gnss_ntrip_session_credential_key: tuple[str, str, str] | None = None
        self._gnss_ntrip_pending_credential: object | None = None
        self._gnss_ntrip_active_credential_source = ""
        self._gnss_ntrip_active_credential_key: tuple[str, str, str] | None = None
        self._gnss_last_gga_skip_reason = ""
        self._gnss_last_gga_skip_monotonic_s: float | None = None
        self.gnss_network_rtcm_parser = Rtcm3StreamParser()
        self.gnss_coordinate_samples: deque[GnssCoordinateSample] = deque(maxlen=2400)
        self.latest_gnss_base_position: GnssCoordinateSample | None = None
        self.gnss_fixed_base_profile = None
        self.latest_rtcm_reference_station: dict[str, object] | None = None
        self.gnss_fixed_base_live_profile = None
        self.gnss_fixed_base_pending_profile = None
        self.gnss_fixed_base_command_sent = False
        self.gnss_fixed_base_reconfigure_expected: dict[str, object] | None = None
        self.gnss_fixed_base_reconfigure_ack = False
        self.gnss_fixed_base_reconfigure_readback = False
        self.gnss_fixed_base_reconfigure_rtcm = False
        self.gnss_fixed_base_reconfigure_deadline_s: float | None = None
        self.gnss_active_operation_config: GnssOperationConfig | None = None
        self.gnss_rover_fixed_since: dict[str, float | None] = {"left": None, "right": None, "drone": None}
        self.gnss_operation_gate_active = False
        self.gnss_base_detect_thread: QtCore.QThread | None = None
        self.gnss_base_detect_worker: GnssBaseSerialDetectWorker | None = None
        self._gnss_base_candidate_rows: list[dict[str, object]] = []
        self.workstation_gnss_base_discovery_status = "NOT RUN"
        self.latest_gnss_base_survey_in: dict[str, object] | None = None
        self.gnss_base_survey_samples: deque[tuple[float, int, float, bool, bool]] = deque(maxlen=240)
        self.gnss_base_rtcm_stats = self._new_gnss_base_rtcm_stats()
        self.gnss_base_satellite_count: int | None = None
        self.gnss_base_satellite_count_source: str = ""
        self.gnss_base_satellite_quality: dict[str, object] | None = None
        self.drone_telemetry_rtcm_forward = self.settings.drone_telemetry_rtcm_forward
        self.drone_telemetry_rtcm_source = normalize_drone_rtcm_source(self.settings.drone_telemetry_rtcm_source)
        self.gnss_operation_manager = GnssOperationManager(self.settings.gnss_operation)
        self.gnss_base_recording_rows: SpoolRows = SpoolRows()
        self._gnss_baseline_center_recording_sequence = 0
        self.gnss_monitor = GNSSMonitor()
        self.latest_gnss_relative: dict[str, object] = {}

    def _build_gnss_base_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        layout.setSpacing(8)
        self.gnss_base_status_labels: dict[str, QtWidgets.QLabel] = {}
        self.gnss_base_status_title_labels: dict[str, QtWidgets.QLabel] = {}
        self.gnss_base_quality_labels: dict[str, QtWidgets.QLabel] = {}
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setObjectName("gnssBaseScrollArea")
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(8)

        operation_config = self.settings.gnss_operation
        self.gnss_site_label_edit = QtWidgets.QLineEdit(operation_config.site_label)
        self.gnss_site_label_edit.setPlaceholderText("이번 실험의 장소/지점명")
        self.gnss_site_address_edit = QtWidgets.QLineEdit(operation_config.site_address)
        self.gnss_site_address_edit.setPlaceholderText("이번 실험의 주소 또는 현장 설명")
        self.gnss_operation_mode_combo = QtWidgets.QComboBox()
        self.gnss_operation_mode_combo.addItem("현장 Base Survey-in", GnssOperationMode.LOCAL_SURVEY_IN.value)
        self.gnss_operation_mode_combo.addItem(
            "NTRIP 보조 Fixed Base", GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE.value
        )
        self.gnss_operation_mode_combo.addItem("Network RTK 직접", GnssOperationMode.NETWORK_RTK_DIRECT.value)
        self._set_combo_data(self.gnss_operation_mode_combo, operation_config.operation_mode.value)
        self.gnss_operation_mode_combo.currentIndexChanged.connect(
            lambda _index: self._handle_gnss_operation_mode_changed()
        )
        self.gnss_ntrip_host_edit = QtWidgets.QLineEdit(operation_config.network.caster_host)
        self.gnss_ntrip_host_edit.setPlaceholderText("caster.example.com (hostname only)")
        self.gnss_ntrip_port_spin = QtWidgets.QSpinBox()
        self.gnss_ntrip_port_spin.setRange(1, 65535)
        self.gnss_ntrip_port_spin.setValue(operation_config.network.caster_port)
        self.gnss_ntrip_mountpoint_edit = QtWidgets.QLineEdit(operation_config.network.mountpoint)
        self.gnss_ntrip_mountpoint_edit.setPlaceholderText("MOUNTPOINT")
        self.gnss_ntrip_username_edit = QtWidgets.QLineEdit(operation_config.network.username)
        self.gnss_ntrip_username_edit.setPlaceholderText("NTRIP username")
        # Session-only: initialized from the deployment default, never read
        # back into GnssOperationConfig/settings JSON, and still directly
        # replaceable/clearable by the operator. See
        # gnss_runtime_controller._resolve_ntrip_credentials for how this
        # value is used (and workstation_core/ntrip_credentials.py for where
        # "Remember" actually persists it).
        self.gnss_ntrip_password_edit = QtWidgets.QLineEdit(DEFAULT_NTRIP_SESSION_PASSWORD)
        self.gnss_ntrip_password_edit.setEchoMode(QtWidgets.QLineEdit.Password)
        self.gnss_ntrip_password_edit.setPlaceholderText("NTRIP password")
        self.gnss_ntrip_password_edit.textChanged.connect(
            lambda _text: (
                setattr(self, "_gnss_ntrip_session_password_explicit", True),
                setattr(
                    self,
                    "_gnss_ntrip_session_credential_key",
                    (
                        self.gnss_ntrip_host_edit.text().strip().casefold(),
                        self.gnss_ntrip_mountpoint_edit.text().strip().lstrip("/"),
                        self.gnss_ntrip_username_edit.text().strip(),
                    ),
                ),
            )
        )
        self.gnss_ntrip_remember_checkbox = QtWidgets.QCheckBox("Remember password securely")
        self.gnss_ntrip_credential_status_label = QtWidgets.QLabel("--")
        self.gnss_ntrip_forget_password_button = QtWidgets.QPushButton("Forget saved password")
        self.gnss_ntrip_forget_password_button.clicked.connect(self.forget_gnss_ntrip_password)
        self.gnss_ntrip_host_edit.editingFinished.connect(self._refresh_ntrip_credential_status)
        self.gnss_ntrip_mountpoint_edit.editingFinished.connect(self._refresh_ntrip_credential_status)
        self.gnss_ntrip_username_edit.editingFinished.connect(self._refresh_ntrip_credential_status)
        # Legacy indirection, kept only so settings files written before this
        # UI existed keep authenticating.  New setups should leave these
        # alone and use the Username/Password fields above instead.
        self.gnss_ntrip_username_env_edit = QtWidgets.QLineEdit(operation_config.network.username_env)
        self.gnss_ntrip_password_env_edit = QtWidgets.QLineEdit(operation_config.network.password_env)
        self.gnss_ntrip_gga_policy_combo = QtWidgets.QComboBox()
        self.gnss_ntrip_gga_policy_combo.addItem("Base GNSS (보조 Fixed Base)", GgaPolicy.BASE_GNSS.value)
        self.gnss_ntrip_gga_policy_combo.addItem("좌우 중심", GgaPolicy.SYSTEM_CENTER.value)
        self.gnss_ntrip_gga_policy_combo.addItem("Left GNSS", GgaPolicy.LEFT_GNSS.value)
        self.gnss_ntrip_gga_policy_combo.addItem("수동/미사용", GgaPolicy.MANUAL.value)
        self.gnss_ntrip_gga_policy_combo.addItem("세션별 rover", GgaPolicy.PER_ROVER_SESSION.value)
        self._set_combo_data(self.gnss_ntrip_gga_policy_combo, operation_config.network.gga_policy.value)
        self.gnss_fixed_height_combo = QtWidgets.QComboBox()
        self.gnss_fixed_height_combo.addItem("WGS84 ellipsoid height", HeightType.ELLIPSOID.value)
        self.gnss_fixed_height_combo.addItem("MSL (고정 좌표 전 변환 필요)", HeightType.MSL.value)
        self.gnss_fixed_height_combo.addItem("Unknown / not validated", HeightType.UNKNOWN.value)
        self._set_combo_data(self.gnss_fixed_height_combo, operation_config.fixed_base.height_type.value)
        self.gnss_fixed_acquisition_duration_spin = QtWidgets.QSpinBox()
        self.gnss_fixed_acquisition_duration_spin.setRange(1, 86400)
        self.gnss_fixed_acquisition_duration_spin.setValue(int(operation_config.fixed_base.acquisition_duration_s))
        self.gnss_fixed_min_duration_spin = QtWidgets.QSpinBox()
        self.gnss_fixed_min_duration_spin.setRange(1, 86400)
        self.gnss_fixed_min_duration_spin.setValue(int(operation_config.fixed_base.minimum_fixed_duration_s))
        self.gnss_rover_min_fix_type_spin = QtWidgets.QSpinBox()
        self.gnss_rover_min_fix_type_spin.setRange(0, 255)
        self.gnss_rover_min_fix_type_spin.setValue(int(operation_config.readiness.minimum_fix_type))
        self.gnss_base_serial_port_edit = QtWidgets.QLineEdit(
            preferred_gnss_base_serial_port(self.settings.gnss_base_serial_port)
        )
        self.gnss_base_baud_spin = QtWidgets.QSpinBox()
        self.gnss_base_baud_spin.setRange(1200, 4000000)
        self.gnss_base_baud_spin.setValue(int(self.settings.gnss_base_baud))
        self.gnss_base_survey_duration_spin = QtWidgets.QSpinBox()
        self.gnss_base_survey_duration_spin.setRange(1, 86400)
        self.gnss_base_survey_duration_spin.setValue(int(self.settings.gnss_base_survey_in_min_duration_s))
        self.gnss_base_survey_accuracy_spin = QtWidgets.QDoubleSpinBox()
        self.gnss_base_survey_accuracy_spin.setDecimals(3)
        self.gnss_base_survey_accuracy_spin.setRange(0.001, 100.0)
        self.gnss_base_survey_accuracy_spin.setSingleStep(0.1)
        self.gnss_base_survey_accuracy_spin.setValue(float(self.settings.gnss_base_survey_in_accuracy_m))
        self.gnss_base_rtcm_target_combo = QtWidgets.QComboBox()
        self.gnss_base_rtcm_target_combo.addItem("비활성", "disabled")
        self.gnss_base_rtcm_target_combo.addItem("좌/우 Pixhawk", "both")
        self.gnss_base_rtcm_target_combo.addItem("Left Pixhawk", "left")
        self.gnss_base_rtcm_target_combo.addItem("Right Pixhawk", "right")
        self._set_combo_data(self.gnss_base_rtcm_target_combo, self.settings.gnss_base_rtcm_injection_target)

        def add_setting(form: QtWidgets.QGridLayout, row: int, title: str, widget: QtWidgets.QWidget) -> None:
            form.addWidget(QtWidgets.QLabel(title), row, 0)
            form.addWidget(widget, row, 1)
            form.setColumnStretch(1, 1)

        self.gnss_common_config_group = QtWidgets.QGroupBox("GNSS 운용")
        common_form = QtWidgets.QGridLayout(self.gnss_common_config_group)
        add_setting(common_form, 0, "GNSS 운용 모드", self.gnss_operation_mode_combo)
        add_setting(common_form, 1, "실험 장소/지점명", self.gnss_site_label_edit)
        add_setting(common_form, 2, "실험 주소/현장 설명", self.gnss_site_address_edit)
        add_setting(common_form, 3, "RTCM 주입 대상", self.gnss_base_rtcm_target_combo)
        add_setting(common_form, 4, "Rover 최소 fix_type (3=3D, 6=Fixed)", self.gnss_rover_min_fix_type_spin)

        self.gnss_base_config_group = QtWidgets.QGroupBox("Base Serial 설정")
        base_config_form = QtWidgets.QGridLayout(self.gnss_base_config_group)
        add_setting(base_config_form, 0, "Base serial port", self.gnss_base_serial_port_edit)
        add_setting(base_config_form, 1, "Baudrate", self.gnss_base_baud_spin)

        self.gnss_survey_config_group = QtWidgets.QGroupBox("Local Survey-in 설정")
        survey_config_form = QtWidgets.QGridLayout(self.gnss_survey_config_group)
        add_setting(survey_config_form, 0, "Survey-in 최소 시간(s)", self.gnss_base_survey_duration_spin)
        add_setting(survey_config_form, 1, "Survey-in 목표 정확도(m)", self.gnss_base_survey_accuracy_spin)

        self.gnss_network_config_group = QtWidgets.QGroupBox("임의 NTRIP caster 설정")
        network_config_form = QtWidgets.QGridLayout(self.gnss_network_config_group)
        add_setting(network_config_form, 0, "NTRIP caster host", self.gnss_ntrip_host_edit)
        add_setting(network_config_form, 1, "NTRIP port", self.gnss_ntrip_port_spin)
        add_setting(network_config_form, 2, "NTRIP mountpoint", self.gnss_ntrip_mountpoint_edit)
        add_setting(network_config_form, 3, "NTRIP username", self.gnss_ntrip_username_edit)
        add_setting(network_config_form, 4, "NTRIP password", self.gnss_ntrip_password_edit)
        add_setting(network_config_form, 5, "", self.gnss_ntrip_remember_checkbox)
        add_setting(network_config_form, 6, "Credential status", self.gnss_ntrip_credential_status_label)
        add_setting(network_config_form, 7, "", self.gnss_ntrip_forget_password_button)
        add_setting(network_config_form, 8, "NTRIP GGA 정책", self.gnss_ntrip_gga_policy_combo)

        self.gnss_network_legacy_env_group = QtWidgets.QGroupBox(
            "레거시 환경변수 인증 (선택 사항 -- 위 Username/Password가 비어 있을 때만 fallback으로 사용)"
        )
        network_legacy_env_form = QtWidgets.QGridLayout(self.gnss_network_legacy_env_group)
        add_setting(network_legacy_env_form, 0, "NTRIP username 환경변수", self.gnss_ntrip_username_env_edit)
        add_setting(network_legacy_env_form, 1, "NTRIP password 환경변수", self.gnss_ntrip_password_env_edit)

        self.gnss_fixed_config_group = QtWidgets.QGroupBox("Fixed Base 설정")
        fixed_config_form = QtWidgets.QGridLayout(self.gnss_fixed_config_group)
        add_setting(fixed_config_form, 0, "Fixed Base 높이 기준", self.gnss_fixed_height_combo)
        add_setting(fixed_config_form, 1, "Fixed 좌표 수집 시간(s)", self.gnss_fixed_acquisition_duration_spin)
        add_setting(fixed_config_form, 2, "Fixed 상태 최소 시간(s)", self.gnss_fixed_min_duration_spin)

        self.gnss_base_connect_button = QtWidgets.QPushButton("Connect Base GNSS")
        self.gnss_base_connect_button.clicked.connect(self.connect_gnss_base_serial)
        self.gnss_base_disconnect_button = QtWidgets.QPushButton("Disconnect Base GNSS")
        self.gnss_base_disconnect_button.clicked.connect(self.disconnect_gnss_base_serial)
        self.gnss_base_disconnect_button.setEnabled(False)
        self.gnss_base_survey_start_button = QtWidgets.QPushButton("Start Survey-in")
        self.gnss_base_survey_start_button.clicked.connect(self.start_gnss_base_survey_in)
        self.gnss_base_survey_start_button.setEnabled(False)
        self.gnss_base_survey_stop_button = QtWidgets.QPushButton("Stop Survey-in")
        self.gnss_base_survey_stop_button.clicked.connect(self.stop_gnss_base_survey_in)
        self.gnss_base_survey_stop_button.setEnabled(False)
        self.gnss_network_start_button = QtWidgets.QPushButton("Start NTRIP Corrections")
        self.gnss_network_start_button.clicked.connect(self.start_gnss_network)
        self.gnss_network_stop_button = QtWidgets.QPushButton("Stop NTRIP Corrections")
        self.gnss_network_stop_button.clicked.connect(self.stop_gnss_network)
        self.gnss_network_stop_button.setEnabled(False)
        self.gnss_base_discover_button = QtWidgets.QPushButton("Scan Base GNSS Ports")
        self.gnss_base_discover_button.clicked.connect(self.discover_gnss_base_serial_ports)
        self.gnss_base_apply_candidate_button = QtWidgets.QPushButton("Apply Selected Port")
        self.gnss_base_apply_candidate_button.clicked.connect(self.apply_selected_gnss_base_candidate)
        self.gnss_base_apply_candidate_button.setEnabled(False)
        button_row = QtWidgets.QHBoxLayout()
        button_row.addWidget(self.gnss_base_discover_button)
        button_row.addWidget(self.gnss_base_connect_button)
        button_row.addWidget(self.gnss_base_disconnect_button)
        button_row.addWidget(self.gnss_base_survey_start_button)
        button_row.addWidget(self.gnss_base_survey_stop_button)
        button_row.addWidget(self.gnss_network_start_button)
        button_row.addWidget(self.gnss_network_stop_button)
        button_row.addWidget(self.gnss_base_apply_candidate_button)
        button_row.addStretch(1)
        self.gnss_base_control_group = QtWidgets.QGroupBox("GNSS 운용 제어")
        self.gnss_base_control_group.setLayout(button_row)

        self.gnss_base_candidate_table = QtWidgets.QTableWidget(0, 5)
        self.gnss_base_candidate_table.setHorizontalHeaderLabels(["Port", "Target", "Baud", "상태", "UBX 메시지"])
        self.gnss_base_candidate_table.verticalHeader().setVisible(False)
        self.gnss_base_candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.gnss_base_candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.gnss_base_candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.gnss_base_candidate_table.setMinimumHeight(110)
        self.gnss_base_candidate_table.itemSelectionChanged.connect(self._refresh_gnss_base_candidate_actions)
        gnss_candidate_header = self.gnss_base_candidate_table.horizontalHeader()
        gnss_candidate_header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.gnss_base_candidate_group = QtWidgets.QGroupBox("Base 포트 후보")
        candidate_layout = QtWidgets.QVBoxLayout(self.gnss_base_candidate_group)
        candidate_layout.addWidget(self.gnss_base_candidate_table)

        content_layout.addWidget(self.gnss_common_config_group)
        content_layout.addWidget(self.gnss_base_config_group)
        content_layout.addWidget(self.gnss_survey_config_group)
        content_layout.addWidget(self.gnss_network_config_group)
        content_layout.addWidget(self.gnss_network_legacy_env_group)
        content_layout.addWidget(self.gnss_fixed_config_group)
        content_layout.addWidget(self.gnss_base_control_group)
        content_layout.addWidget(self.gnss_base_candidate_group)

        base_group = QtWidgets.QGroupBox("GNSS Base 연결")
        base_group.setObjectName("gnssBaseConnectionGroup")
        base_group.setMinimumHeight(190)
        base_grid = QtWidgets.QGridLayout(base_group)
        base_items = (
            ("connection", "Base 연결", "대기 중", "idle"),
            ("operation", "측정 준비 상태", "대기 중", "idle"),
            ("network", "NTRIP", "미연결", "idle"),
            ("survey_in", "Survey-in", "미연결", "idle"),
            ("base_satellites", "Base 위성 수", "대기 중", "idle"),
            ("rtcm_rx", "Raw 수신", "없음", "idle"),
            ("rtcm_injection", "RTCM 주입", "비활성", "idle"),
        )
        for row, (key, title, text, level) in enumerate(base_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            title_label = QtWidgets.QLabel(title)
            base_grid.addWidget(title_label, row, 0)
            base_grid.addWidget(label, row, 1)
            self.gnss_base_status_labels[key] = label
            self.gnss_base_status_title_labels[key] = title_label
            self._set_health_label(label, text, level)

        base_fixed_group = QtWidgets.QGroupBox("Base RTK-Fixed 진행")
        base_fixed_group.setObjectName("gnssBaseFixedProgressGroup")
        base_fixed_grid = QtWidgets.QGridLayout(base_fixed_group)
        base_fixed_label = QtWidgets.QLabel("대기 중")
        base_fixed_label.setObjectName("statusStrip")
        base_fixed_label.setWordWrap(True)
        base_fixed_grid.addWidget(QtWidgets.QLabel("Base Fixed 상태"), 0, 0)
        base_fixed_grid.addWidget(base_fixed_label, 0, 1)
        self.gnss_base_status_labels["base_fixed"] = base_fixed_label
        self._set_health_label(base_fixed_label, "대기 중", "idle")
        base_position_label = QtWidgets.QLabel("대기 중")
        base_position_label.setObjectName("statusStrip")
        base_position_label.setWordWrap(True)
        base_fixed_grid.addWidget(QtWidgets.QLabel("Base 위치"), 1, 0)
        base_fixed_grid.addWidget(base_position_label, 1, 1)
        self.gnss_base_status_labels["base_position"] = base_position_label
        self._set_health_label(base_position_label, "대기 중", "idle")
        base_fixed_plot_row = QtWidgets.QHBoxLayout()
        self.gnss_base_fixed_state_plot = TelemetryHistoryPlot("Base RTK Fix 상태 (Float→Fixed)", "fix")
        self.gnss_base_fixed_state_plot.setMinimumHeight(180)
        self.gnss_base_fixed_scatter_plot = TelemetryHistoryPlot("Fixed 좌표 수렴 (scatter H/V)", "m")
        self.gnss_base_fixed_scatter_plot.setMinimumHeight(180)
        base_fixed_plot_row.addWidget(self.gnss_base_fixed_state_plot, 1)
        base_fixed_plot_row.addWidget(self.gnss_base_fixed_scatter_plot, 1)
        base_fixed_grid.addLayout(base_fixed_plot_row, 2, 0, 1, 2)
        self.gnss_base_fixed_progress_group = base_fixed_group

        rtcm_group = QtWidgets.QGroupBox("RTCM 보정 데이터")
        rtcm_grid = QtWidgets.QGridLayout(rtcm_group)
        rtcm_items = (
            ("rtcm_receive", "수신", "RTCM 대기 중", "idle"),
            ("rtcm_messages", "메시지", "type 대기 중", "idle"),
            ("rtcm_injection_stats", "주입", "대상 대기 중", "idle"),
            ("rtcm_bandwidth", "대역폭", "packet 대기 중", "idle"),
        )
        for row, (key, title, text, level) in enumerate(rtcm_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            rtcm_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            rtcm_grid.addWidget(label, row, 1)
            self.gnss_base_status_labels[key] = label
            self._set_health_label(label, text, level)

        rover_group = QtWidgets.QGroupBox("Rover RTK 상태")
        rover_grid = QtWidgets.QGridLayout(rover_group)
        rover_items = (
            ("left_rover", "Left rover", "GPS/RTK 대기 중"),
            ("right_rover", "Right rover", "GPS/RTK 대기 중"),
            ("baseline", "GPS/RTK baseline", "대기 중"),
        )
        for row, (key, title, text) in enumerate(rover_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            rover_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            rover_grid.addWidget(label, row, 1)
            self.gnss_base_status_labels[key] = label
            self._set_health_label(label, text, "idle")

        survey_group = QtWidgets.QGroupBox("Survey-in 진행")
        survey_group.setObjectName("gnssBaseSurveyProgressGroup")
        survey_group.setMinimumHeight(320)
        survey_layout = QtWidgets.QVBoxLayout(survey_group)
        survey_summary_row = QtWidgets.QHBoxLayout()
        self.gnss_base_survey_progress_bar = QtWidgets.QProgressBar()
        self.gnss_base_survey_progress_bar.setRange(0, 100)
        self.gnss_base_survey_progress_bar.setValue(0)
        self.gnss_base_survey_progress_bar.setFormat("not running | time 0% | accuracy --")
        self.gnss_base_survey_progress_bar.setMinimumHeight(24)
        self.gnss_base_survey_progress_bar.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.gnss_base_survey_accuracy_label = QtWidgets.QLabel("accuracy 대기 중")
        self.gnss_base_survey_accuracy_label.setObjectName("statusStrip")
        self.gnss_base_survey_accuracy_label.setWordWrap(True)
        survey_summary_row.addWidget(self.gnss_base_survey_progress_bar, 2)
        survey_summary_row.addWidget(self.gnss_base_survey_accuracy_label, 1)
        survey_layout.addLayout(survey_summary_row)
        self.gnss_base_survey_convergence_label = QtWidgets.QLabel("수렴: 대기 중 | ETA: -- | 최근 개선: --")
        self.gnss_base_survey_convergence_label.setObjectName("statusStrip")
        self.gnss_base_survey_convergence_label.setWordWrap(True)
        survey_layout.addWidget(self.gnss_base_survey_convergence_label)

        survey_plot_row = QtWidgets.QHBoxLayout()
        self.gnss_base_survey_accuracy_plot = TelemetryHistoryPlot("Survey-in accuracy", "m")
        self.gnss_base_survey_accuracy_plot.setMinimumHeight(180)
        self.gnss_base_survey_observation_plot = TelemetryHistoryPlot("Survey-in observations", "obs")
        self.gnss_base_survey_observation_plot.setMinimumHeight(180)
        survey_plot_row.addWidget(self.gnss_base_survey_accuracy_plot, 1)
        survey_plot_row.addWidget(self.gnss_base_survey_observation_plot, 1)
        survey_layout.addLayout(survey_plot_row)

        quality_group = QtWidgets.QGroupBox("Survey-in 품질 진단")
        self.gnss_survey_quality_group = quality_group
        quality_grid = QtWidgets.QGridLayout(quality_group)
        quality_items = (
            ("satellites", "위성", "NAV-SAT 대기 중"),
            ("signal", "신호", "C/N0 대기 중"),
            ("elevation", "고도각", "elevation 대기 중"),
            ("constellations", "구성", "constellation 대기 중"),
        )
        for row, (key, title, text) in enumerate(quality_items):
            label = QtWidgets.QLabel(text)
            label.setObjectName("statusStrip")
            label.setWordWrap(True)
            quality_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            quality_grid.addWidget(label, row, 1)
            self.gnss_base_quality_labels[key] = label
            self._set_health_label(label, text, "idle")

        note = QtWidgets.QLabel(
            "현장 Base Survey-in, NTRIP 보조 Fixed Base, Network RTK 직접 모드를 지원합니다. "
            "NTRIP 비밀번호는 설정 파일에 저장하지 않고 지정한 환경변수에서 읽습니다."
        )
        note.setWordWrap(True)
        note.setObjectName("statusStrip")

        content_layout.addWidget(base_group)
        content_layout.addWidget(base_fixed_group)
        content_layout.addWidget(rtcm_group)
        content_layout.addWidget(rover_group)
        self.gnss_survey_progress_group = survey_group
        content_layout.addWidget(survey_group)
        content_layout.addWidget(quality_group)
        content_layout.addWidget(note)
        content_layout.addStretch(1)
        scroll_area.setWidget(content)
        layout.addWidget(scroll_area, 1)
        self._refresh_gnss_base_tab()
        self._handle_gnss_operation_mode_changed()

    def _new_gnss_base_rtcm_stats(self) -> dict[str, object]:
        return {
            "start_s": time.monotonic(),
            "last_frame_s": None,
            "frames": 0,
            "bytes": 0,
            "max_frame_bytes": 0,
            "message_types": {},
            "injection": {
                "left": {"frames": 0, "bytes": 0, "packets": 0, "fail": 0, "blocked_stall": 0},
                "right": {"frames": 0, "bytes": 0, "packets": 0, "fail": 0, "blocked_stall": 0},
                "drone": {"frames": 0, "bytes": 0, "packets": 0, "fail": 0, "blocked_stall": 0},
            },
        }

    def _clear_gnss_base_serial_worker(self, thread: QtCore.QThread) -> None:
        controller = getattr(self, "_gnss_runtime_controller_owner", None)
        runtime = getattr(controller, "_base_owner", None)
        binding = None if runtime is None else runtime.bindings["serial"]
        if binding is not None and binding.thread is thread:
            runtime.retire(binding, "serial")
            return
        if self.gnss_base_serial_thread is thread:
            self.gnss_base_serial_thread = None
            self.gnss_base_serial_worker = None
            self.gnss_base_serial_connected = False
            owner = getattr(self, "_gnss_runtime_controller_owner", None)
            if owner is not None:
                owner._wire_rtcm_pipeline_base_worker(None)
                owner._sync_rtcm_pipeline_config()
            if hasattr(self, "gnss_base_connect_button"):
                self.gnss_base_connect_button.setEnabled(True)
            if hasattr(self, "gnss_base_disconnect_button"):
                self.gnss_base_disconnect_button.setEnabled(False)
            if hasattr(self, "gnss_base_survey_start_button"):
                self.gnss_base_survey_start_button.setEnabled(False)
            if hasattr(self, "gnss_base_survey_stop_button"):
                self.gnss_base_survey_stop_button.setEnabled(False)
        thread.deleteLater()
