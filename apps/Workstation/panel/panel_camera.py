"""Camera feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
from Workstation.calibration_workflow.calibration_geometry_state import CameraGeometryState
from Workstation.workstation_core.active_geometry import ActiveGeometryStore
import time
from PyQt5 import (
    QtCore,
    QtGui,
    QtWidgets,
)
from Workstation.controllers.camera_time_sync_controller import CAMERA_SYNC_STATUS_NOT_VERIFIED
from Workstation.workers.device_runtime_workers import CameraNetworkAddressApplyWorker, CameraNetworkDiscoveryWorker, InterfaceStatusWorker, PixhawkUsbDiscoveryWorker
from Workstation.workers.panel_camera_workers import CameraAutoDetectWorker, WirisControlCheckWorker, _control_candidate_value
from Workstation.panel.panel_controls import NoWheelSpinBox
from Workstation.panel.panel_support import CAMERA_AUTO_CONNECT_MAX_ATTEMPTS, CAMERA_AUTO_CONNECT_RETRY_DELAY_MS, NETWORK_FIELD_MIN_WIDTHS, RTSP_STREAM_CHECK_TIMEOUT_S, RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX, RUNTIME_CAMERA_LAYOUT_HYSTERESIS_PX, RUNTIME_CAMERA_STATUS_GROUP_HEIGHT_PX, RUNTIME_CAMERA_STATUS_LABEL_HEIGHT_PX, STARTUP_CAMERA_STATUS, STARTUP_PIXHAWK_STATUS, YOLO_CPU_DEVICE, YOLO_CUDA_DEVICE, _clear_side_active_decoder, _ensure_side_decoder_overrides
from Workstation.wiris_workers.probe import RtspStreamCheckWorker
from Workstation.wiris_workers.clock import WirisClockCommandWorker
from Workstation.wiris_workers.commands import WirisCommandWorker, WirisLrfPollWorker, WirisZoomWorker
from Workstation.wiris_workers.network import WirisRtspEnableWorker, WirisStipWorker
from Workstation.wiris_workers.wiris_sdk_catalog import WIRIS_COMMAND_CATALOG
from Workstation.workstation_core.camera_pane import (
    CameraPane,
)
from Workstation.workstation_core.mavlink_liveness import (
    MAVLINK_QUALIFICATION_S,
)
from Workstation.workstation_core.measurement_widgets import (
    ClockOffsetHistoryPlot,
    DistanceHistoryPlot,
)
from Workstation.workstation_core.network import (
    DEFAULT_CAMERA_URLS,
    DEFAULT_HOST_ADDRESS,
    DEFAULT_INTERFACE,
    rtsp_host_port,
    select_host_for_camera,
)
from Workstation.workstation_core.runtime_camera_status import (
    camera_display_name,
    camera_resolution_text,
    format_camera_stream_stats,
    format_runtime_camera_ntp_compact,
    format_runtime_camera_stats,
    format_runtime_camera_timestamp,
    format_runtime_camera_timestamp_compact,
    format_signed_ms_compact,
    format_unix_timestamp_compact,
)
from Workstation.workstation_core.settings import (
    normalize_live_distance_time_basis,
)
from Workstation.workstation_core.stus_scheduler import (
    StusScheduler,
)
from Workstation.workstation_core.wiris_zoom import (
    DEFAULT_ZOOM_CONTROL_PORT,
    LINE_TERMINATOR,
    MAX_ZOOM_LEVEL,
    MIN_ZOOM_LEVEL,
    NUL_TERMINATOR,
)
from Workstation.workstation_core.media_stream.policy import DECODER_AUTO, DECODER_NVIDIA, DECODER_SOFTWARE, DEFAULT_YOLO_WEIGHTS, NVIDIA_H264_DECODER, RTSP_TRANSPORT_TCP, RTSP_TRANSPORT_UDP, SOFTWARE_H264_DECODER, STREAM_BACKEND_GSTREAMER, STREAM_BACKEND_OPENCV, STREAM_BACKEND_OPENCV_AUTO, STREAM_BACKEND_PYAV
from collections import (
    deque,
)
from ipaddress import (
    IPv4Address,
)
from math import (
    isfinite,
)
from pathlib import (
    Path,
)
from urllib.parse import (
    urlparse,
)


from Workstation.camera_feature.state import CameraFeatureState
from Workstation.camera_feature.control import CameraControlRuntime
from Workstation.camera_runtime.recovery import SideStreamState
from Workstation.camera_runtime.rejoin import ReactivationPorts, RejoinRecordingSnapshot, RejoinTarget
from Workstation.camera_feature.connection_view import CameraConnectionControls, CameraConnectionSettings, CameraConnectionActions
from Workstation.camera_feature.candidates import CameraCandidateView
from Workstation.camera_feature.views import CameraViews
from Workstation.camera_feature.command_view import CameraCommandView, CameraCommandActions
from Workstation.camera_runtime.readiness import StreamReadinessTracker
from Workstation.camera_runtime.lifecycle_timers import CameraLifecycleTimers
from Workstation.device_runtime.network_roles import NetworkRoleRuntime

class CameraFeature(PanelFeature):
    def __init__(self, widget, *, geometry_store=None, stream_readiness=None):
        store = ActiveGeometryStore() if geometry_store is None else geometry_store
        readiness = StreamReadinessTracker() if stream_readiness is None else stream_readiness
        super().__init__(widget, state_factory=lambda owner: CameraFeatureState(store, NetworkRoleRuntime(owner, thread_parent=widget), readiness, CameraLifecycleTimers(owner), CameraControlRuntime(owner, thread_parent=widget)))
        self.views = CameraViews()

    def resource_inventory(self):
        return {**super().resource_inventory(), **self.state.network_roles.resource_inventory(),
                **self.state.lifecycle_timers.resource_inventory(), **self.state.control_jobs.resource_inventory()}

    feature_name = 'camera'
    dependency_names = ('_apply_selected_wiris_command', '_automatic_session_recording', '_build_toggle_button', '_camera_stream_frame_ready', '_camera_stream_start_status', '_configure_collapsible_group', '_control_port_for_camera', '_log', '_measurement_run_state', '_normalized_yolo_device', '_refresh_distance', '_refresh_operation_header', '_refresh_session_toggle_buttons', '_refresh_yolo_device_toggle', '_refresh_yolo_gate_controls', '_refresh_zoom_intrinsics_tab', '_remove_distance_history_point', '_save_current_settings', '_session_recording_active', '_set_camera_sync_state', '_set_combo_data', '_set_control_port_for_camera', '_set_yolo_device_combo', '_shutdown_in_progress', '_start_camera_streams', '_startup_camera_sequence_pending', '_update_calibration_preview', '_update_detection', '_video_recording_allocation_sizes', '_video_recording_kwargs', 'allocate_measurement_video_segment', 'apply_pc_camera_ip', 'apply_wiris_stip', 'browse_weights_button', 'browse_yolo_weights', 'camera_yolo_log', 'clear_distance_history', 'clear_history_button', 'clock_offset_plot', 'clock_sync_check_button', 'clock_sync_status_label', 'clock_sync_summary_text', 'conf_spin', 'connect_cameras', 'depth_tab_index', 'device_combo', 'device_edit', 'discover_pc_camera_ip', 'distance_plot', 'input_size_spin', 'iou_spin', 'left_runtime_stats_label', 'measurement_source_transition', 'native_frame_persistence_pending_count', 'preview_width_spin', 'record_button', 'record_measurement_video_segment_presence', 'refresh_clock_sync_status', 'request_network_role_assignment', 'right_runtime_stats_label', 'runtime_distance_graph_group', 'runtime_header_layout', 'runtime_pair_status_label', 'selected_runtime_frame_metadata', 'selected_runtime_frame_pair_basis', 'selected_runtime_frame_pair_delta_s', 'selected_runtime_frame_pair_fallback_reason', 'selected_runtime_frame_pair_requested_basis', 'selected_zoom_target', 'send_selected_wiris_command', 'send_zoom', 'set_selected_wiris_time_from_pc', 'settings', 'start_automatic_session_recording', 'start_button', 'start_yolo', 'stop_yolo_streams', 'tabs', 'toggle_camera_connection', 'toggle_session_recording', 'toggle_yolo_detection', 'video_recording_enabled', 'video_recording_paths', 'weights_edit', 'width', 'yolo_enabled', 'yolo_event_queue_size_spin', 'yolo_frame_queue_size_spin', 'yolo_gate_class_checkbox', 'yolo_gate_class_edit', 'yolo_gate_matching_checkbox', 'yolo_gate_status_label', 'yolo_gate_time_basis_combo', 'yolo_gate_time_delta_checkbox', 'yolo_gate_time_delta_spin', 'yolo_gate_vertical_delta_checkbox', 'yolo_gate_vertical_delta_spin', 'yolo_kwargs', 'yolo_log_splitter', 'zoom_hosts', 'build_legacy_pixhawk_connection_controls', 'build_legacy_pixhawk_connection_layout', 'legacy_pixhawk_network_assignment_snapshot')
    writable_dependencies = frozenset(('_startup_camera_sequence_pending', 'browse_weights_button', 'camera_yolo_log', 'clear_history_button', 'clock_offset_plot', 'clock_sync_check_button', 'clock_sync_status_label', 'clock_sync_summary_text', 'conf_spin', 'device_combo', 'device_edit', 'distance_plot', 'input_size_spin', 'iou_spin', 'left_runtime_stats_label', 'preview_width_spin', 'record_button', 'right_runtime_stats_label', 'runtime_distance_graph_group', 'runtime_header_layout', 'runtime_pair_status_label', 'start_button', 'video_recording_enabled', 'weights_edit', 'yolo_enabled', 'yolo_event_queue_size_spin', 'yolo_frame_queue_size_spin', 'yolo_gate_class_checkbox', 'yolo_gate_class_edit', 'yolo_gate_matching_checkbox', 'yolo_gate_status_label', 'yolo_gate_time_basis_combo', 'yolo_gate_time_delta_checkbox', 'yolo_gate_time_delta_spin', 'yolo_gate_vertical_delta_checkbox', 'yolo_gate_vertical_delta_spin', 'yolo_log_splitter'))

    def _camera_control_runtime(self):
        return self.state.control_jobs

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.wiris_image_format_worker: QtCore.QObject | None = None
        self.wiris_image_format_thread: QtCore.QThread | None = None
        self.wiris_image_format_status: dict[str, object] = {}
        self.last_capture_zoom_level: int | None = None
        self._active_extrinsics_source_zoom_level: int | None = None
        self.actual_visible_zoom: dict[str, dict[str, object]] = {
            "left": {},
            "right": {},
        }
        self._zoom_readback_target: str | None = None
        self.camera_connected = False
        self._camera_start_request_generation = 0
        self._pending_camera_start: tuple[bool, bool] | None = None
        self._camera_start_retry_deadline_s: float | None = None
        self._camera_start_readiness_generation = 0
        self._camera_start_readiness: dict[str, object] | None = None
        self._camera_stop_watchdog_generation = 0
        self.camera_auto_detect_thread: QtCore.QThread | None = None
        self.camera_auto_detect_worker: CameraAutoDetectWorker | None = None
        self._camera_auto_connect_attempt = 0
        self._camera_auto_connect_retry_generation = 0
        self._auto_wiris_enabled_cameras: set[tuple[str, str]] = set()
        self._rtsp_reactivation_side: str | None = None
        self._rtsp_reactivation_previous_video_path: Path | None = None
        self._measurement_camera_rejoin_expected_generations: dict[str, int] = {}
        self._rtsp_reactivation_generation = 0
        self._rtsp_reactivation_stop_generation: int | None = None
        self.wiris_control_check_thread: QtCore.QThread | None = None
        self.wiris_control_check_worker: WirisControlCheckWorker | None = None
        self.wiris_command_thread: QtCore.QThread | None = None
        self.wiris_command_worker: WirisCommandWorker | None = None
        self.wiris_lrf_thread: QtCore.QThread | None = None
        self.wiris_lrf_worker: WirisLrfPollWorker | None = None
        self.wiris_zoom_thread: QtCore.QThread | None = None
        self.wiris_zoom_worker: WirisZoomWorker | None = None
        self.wiris_time_sync_thread: QtCore.QThread | None = None
        self.wiris_time_sync_worker: WirisClockCommandWorker | None = None
        self.wiris_stip_thread: QtCore.QThread | None = None
        self.wiris_stip_worker: WirisStipWorker | None = None
        self.interface_status_thread: QtCore.QThread | None = None
        self.interface_status_worker: InterfaceStatusWorker | None = None
        self.rtsp_check_thread: QtCore.QThread | None = None
        self.rtsp_check_worker: RtspStreamCheckWorker | None = None
        self.rtsp_enable_thread: QtCore.QThread | None = None
        self.rtsp_enable_worker: WirisRtspEnableWorker | None = None
        self.auto_wiris_connect_pending = False
        self._last_wiris_pc_utc_sync: dict[str, object] | None = None
        self._camera_sync_stus_operation: dict[str, object] | None = None
        self._rtcp_clock_observer_thread: QtCore.QThread | None = None
        self._rtcp_clock_observer_worker = None
        self._rtcp_clock_observer_generation = 0
        self._rtcp_clock_observer_pending_start: dict[str, object] | None = None
        self._rtcp_clock_observation_history: dict[str, deque[dict[str, object]]] = {
            "left": deque(maxlen=16),
            "right": deque(maxlen=16),
        }
        self._stus_connection_generation = 1
        self._stus_scheduler = StusScheduler(
            mode=getattr(self.settings, "camera_sync_mode", "manual"),
            interval_s=getattr(self.settings, "camera_sync_interval_s", 1800.0),
        )
        self.camera_sync_status = CAMERA_SYNC_STATUS_NOT_VERIFIED
        self.camera_sync_reason = "STUS 동기화 후 RTCP/NTP 검증이 필요합니다."
        self.frame_timing_status = "not-verified"
        self.frame_timing_reason = "decoded frame RTCP mapping has not been observed"
        self.frame_timing_status_by_side = {
            "left": "not-verified",
            "right": "not-verified",
        }
        self._camera_sync_transport_override: str | None = None
        self._camera_sync_deadline_monotonic_s: float | None = None
        self._camera_sync_stus_sr_baseline: dict[str, int] = {}
        self._camera_sync_last_sr_counts: dict[str, int] = {}
        self._camera_sync_timer = QtCore.QTimer(self.widget)
        self._stus_schedule_timer = QtCore.QTimer(self.widget)
        self.camera_frame_times = {
            "left": deque(maxlen=120),
            "right": deque(maxlen=120),
        }
        self.camera_frame_metadata = {
            "left": {},
            "right": {},
        }
        self.camera_frame_metadata_history: dict[str, deque[dict[str, object]]] = {
            "left": deque(maxlen=240),
            "right": deque(maxlen=240),
        }
        self.selected_runtime_frame_pair_camera_rtc_delta_s: float | None = None
        self.camera_stream_stats = {
            "left": {},
            "right": {},
        }
        self.active_stream_decoders: dict[str, str | None] = {
            "left": None,
            "right": None,
        }
        self.decoder_fallback_reasons: dict[str, str] = {"left": "", "right": ""}
        self.decoder_selection_status: dict[str, object] = {
            "requested": str(self.settings.rtsp_decoder or DECODER_AUTO),
            "effective": None,
            "candidates": (),
            "allowed": None,
            "degraded": False,
            "reason": "not probed for this process",
            "nvidia_available": None,
        }
        self._runtime_decoder_overrides: dict[str, str | None] = {
            "left": None,
            "right": None,
        }
        self._runtime_decoder_override: str | None = None
        self._last_logged_camera_states: dict[str, tuple[str, float]] = {}
        self.latest_wiris_lrf: dict[str, dict[str, object]] = {
            "left": {},
            "right": {},
        }
        self.last_rtsp_check_status: dict[str, str] = {"left": "unchecked", "right": "unchecked"}

    def _build_clock_sync_tab(self, layout: QtWidgets.QVBoxLayout) -> None:
        """Build the Workstation camera-clock and RTCP/NTP evidence surface.

        Workstation timing evidence includes the WIRIS RTC command, editable
        RTCP/NTP sync limits, and live RTCP/NTP frame metadata.
        """

        layout.setSpacing(8)

        rtc_group = QtWidgets.QGroupBox("WIRIS RTC / 스트림 동기화")
        rtc_layout = QtWidgets.QGridLayout(rtc_group)
        rtc_layout.setHorizontalSpacing(8)
        rtc_layout.setVerticalSpacing(6)
        rtc_layout.setColumnStretch(1, 1)
        rtc_layout.setColumnStretch(3, 1)
        self.clock_wiris_time_target_combo = QtWidgets.QComboBox()
        self.clock_wiris_time_target_combo.addItem("양쪽 카메라", "both")
        self.clock_wiris_time_target_combo.addItem("왼쪽 카메라", "left")
        self.clock_wiris_time_target_combo.addItem("오른쪽 카메라", "right")
        self.wiris_set_pc_time_button = QtWidgets.QPushButton("PC UTC로 시간 설정")
        self.wiris_set_pc_time_button.clicked.connect(
            lambda _checked=False: self.set_selected_wiris_time_from_pc(
                str(self.clock_wiris_time_target_combo.currentData() or "both")
            )
        )
        self.clock_sync_check_button = QtWidgets.QPushButton("RTCP/NTP 동기화 확인")
        self.clock_sync_check_button.clicked.connect(self.refresh_clock_sync_status)
        self.clock_sync_status_label = QtWidgets.QLabel("RTCP/NTP frame timestamp 대기 중")
        self.clock_sync_status_label.setObjectName("statusStrip")
        self.clock_sync_status_label.setWordWrap(True)
        self.clock_sync_status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.clock_sync_summary_text = QtWidgets.QPlainTextEdit()
        self.clock_sync_summary_text.setReadOnly(True)
        self.clock_sync_summary_text.setMaximumBlockCount(80)
        self.clock_sync_summary_text.setFixedHeight(104)
        self.clock_sync_summary_text.setPlaceholderText(
            "YOLO/preview stream에서 RTCP Sender Report timestamp가 들어오면 좌/우 RTC offset이 표시됩니다."
        )
        rtc_layout.addWidget(QtWidgets.QLabel("시간 설정 대상"), 0, 0)
        rtc_layout.addWidget(self.clock_wiris_time_target_combo, 0, 1)
        rtc_layout.addWidget(self.wiris_set_pc_time_button, 0, 2)
        rtc_layout.addWidget(self.clock_sync_check_button, 0, 3)
        rtc_layout.addWidget(self.clock_sync_status_label, 1, 0, 1, 4)
        rtc_layout.addWidget(self.clock_sync_summary_text, 2, 0, 1, 4)

        self.clock_offset_plot = ClockOffsetHistoryPlot()
        history_group = QtWidgets.QGroupBox("RTCP/NTP 오프셋 추이")
        history_layout = QtWidgets.QVBoxLayout(history_group)
        history_layout.addWidget(self.clock_offset_plot)
        layout.addWidget(rtc_group)
        layout.addWidget(self.camera_sync_settings_group)
        layout.addWidget(history_group, 1)

    def _format_age(self, age_s: float) -> str:
        if not isfinite(age_s):
            return "n/a"
        if age_s < 1.0:
            return f"{age_s * 1000.0:.0f}ms"
        return f"{age_s:.2f}s"

    def _rtsp_url_host_text(self, url: str) -> str:
        if not url:
            return "missing"
        try:
            host, port = rtsp_host_port(url)
        except Exception:
            return "invalid-url"
        return f"{host}:{port}"

    def _rtsp_url_host_only(self, url: str) -> str:
        try:
            host, _port = rtsp_host_port(url.strip())
        except Exception:
            return ""
        return host

    def detect_camera_control(self, side: str | None = None) -> None:
        self._start_camera_detection("control", target_side=side)

    def detect_camera_rtsp(self) -> None:
        self._start_camera_detection("rtsp")

    def auto_connect_wiris(self) -> bool | None:
        if bool(getattr(self, "_shutdown_in_progress", False)):
            self._log("[카메라] application shutdown 중에는 WIRIS 자동 연결을 시작할 수 없습니다.")
            return False
        stus_operation = getattr(self, "_camera_sync_stus_operation", None)
        if isinstance(stus_operation, dict) and stus_operation.get("active"):
            self._log("[카메라] STUS closed-loop 중에는 WIRIS 자동 연결을 시작할 수 없습니다.")
            return False
        recording_active_reader = getattr(self, "_session_recording_active", None)
        if callable(recording_active_reader) and recording_active_reader():
            self._log("[카메라] 데이터 기록 중에는 WIRIS 자동 연결을 시작할 수 없습니다.")
            return False
        self._camera_control_runtime().cancel_retry()
        self._camera_auto_connect_retry_generation += 1
        self._camera_auto_connect_attempt = 0
        self._auto_wiris_enabled_cameras.clear()
        self.auto_wiris_connect_pending = True
        self._start_camera_detection("auto_connect")
        return None

    def _start_startup_camera_sequence(self) -> None:
        """Enable configured WIRIS streams without a blocking network-wide scan."""

        if self._session_recording_active():
            self._log("[카메라] --start: 데이터 기록 중이어서 startup 카메라 연결을 건너뜁니다.")
            return
        self._startup_camera_sequence_pending = True
        self.auto_wiris_connect_pending = True
        self._camera_auto_connect_attempt = 1
        self._auto_wiris_enabled_cameras.clear()
        self._log(
            "[카메라] --start: 설정된 Left/Right Control Host만 사용해 "
            "WIRIS 활성화와 RTSP 프레임 확인을 시작합니다. "
            "서비스가 준비되지 않으면 자동 탐색으로 fallback합니다."
        )
        self._start_rtsp_enable_worker(target_labels=("left", "right"), reactivate=False)

    def _camera_host_hints(self) -> tuple[str, ...]:
        hints: list[str] = []
        for text in (
            self.left_zoom_host_edit.text(),
            self.right_zoom_host_edit.text(),
            self.left_url_edit.text(),
            self.right_url_edit.text(),
            *DEFAULT_CAMERA_URLS,
        ):
            value = str(text).strip()
            if not value:
                continue
            if value.startswith("rtsp://"):
                try:
                    host, _port = rtsp_host_port(value)
                except Exception:
                    continue
                value = host
            try:
                IPv4Address(value)
            except ValueError:
                continue
            if value not in hints:
                hints.append(value)
        return tuple(hints)

    def _start_camera_detection(self, mode: str, *, target_side: str | None = None) -> None:
        if self.camera_auto_detect_thread is not None:
            return
        attempt_text = ""
        if mode == "auto_connect":
            self._camera_auto_connect_attempt += 1
            attempt_text = (
                f" (시도 {self._camera_auto_connect_attempt}/{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS})"
            )
        full_control_scan = mode != "auto_connect" or self._camera_auto_connect_attempt <= 1
        self.workstation_camera_discovery_status = f"SEARCHING {mode.upper()}"
        self._log(
            f"[연결] 카메라 {mode} 탐색은 설정된 인터페이스를 사용합니다. "
            "필요하면 Discover Camera Network로 후보 NIC/IP를 먼저 확인하세요."
        )
        if mode == "auto_connect":
            self._log(
                "[카메라] WIRIS 자동 연결을 시작합니다"
                f"{attempt_text}: 설정/기본 제어 포트, HIWS 검증, "
                "RTSP 활성화, 프레임 확인을 순차 실행합니다."
            )
            self._log("[arp] scan: 카메라망 후보 탐색을 시작합니다.")
        elif mode == "control":
            self._log("[카메라] 카메라 네트워크에서 WIRIS 제어 포트를 전체 스캔하고 HIWS로 검증합니다.")
            self._log("[arp] scan: 카메라망 후보 탐색을 시작합니다.")
        else:
            self._log("[카메라] 카메라 네트워크와 RTSP 후보를 탐색합니다.")
        self.auto_wiris_connect_button.setEnabled(False)
        self.detect_control_button.setEnabled(False)
        self.right_detect_control_button.setEnabled(False)
        self.detect_rtsp_button.setEnabled(False)
        self.right_detect_rtsp_button.setEnabled(False)
        self._camera_control_runtime().launch(
            'detect', worker_factory=lambda: CameraAutoDetectWorker(mode=mode, control_port=self.zoom_port_spin.value(), host_hints=self._camera_host_hints(), target_side=target_side, full_control_scan=full_control_scan),
            completed=self._apply_camera_detection, failed=self._camera_auto_detect_failed, retired=self._clear_camera_auto_detect_worker,
        )

    def _cancel_auto_wiris_connect_retry(self) -> None:
        self._camera_auto_connect_retry_generation += 1
        self._camera_auto_connect_attempt = 0
        self.auto_wiris_connect_pending = False
        self._startup_camera_sequence_pending = False

    def _schedule_auto_wiris_connect_retry(self, reason: str) -> None:
        if not self.auto_wiris_connect_pending:
            return
        if self._startup_camera_sequence_pending:
            # Startup must remain responsive.  A failed configured-host
            # attempt is reported for manual recovery instead of escalating
            # into a repository-wide TCP/HIWS scan while the operator is
            # waiting for the window to become usable.
            self.auto_wiris_connect_pending = False
            self._startup_camera_sequence_pending = False
            self.workstation_camera_discovery_status = "STARTUP CONFIGURED HOSTS UNAVAILABLE"
            self.camera_connection_label.setText(
                "WIRIS startup: configured camera hosts unavailable; use Connect cameras"
            )
            self._log(
                f"[카메라] startup configured-host 연결 실패: {reason}. "
                "전체 자동 탐색은 생략했습니다. Connect cameras로 수동 탐색하세요."
            )
            self._refresh_operation_header()
            return
        attempt = max(0, int(self._camera_auto_connect_attempt))
        if attempt >= CAMERA_AUTO_CONNECT_MAX_ATTEMPTS:
            self.auto_wiris_connect_pending = False
            self._startup_camera_sequence_pending = False
            self._log(
                "[카메라] WIRIS 자동 연결을 중단했습니다: "
                f"{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS}회 시도 후에도 카메라 서비스가 준비되지 않았습니다. "
                "전원/Ethernet Mode/IP를 확인하세요."
            )
            self.camera_connection_label.setText("WIRIS auto connect: camera service unavailable")
            self._refresh_operation_header()
            return

        self._camera_auto_connect_retry_generation += 1
        generation = self._camera_auto_connect_retry_generation
        next_attempt = attempt + 1
        delay_s = CAMERA_AUTO_CONNECT_RETRY_DELAY_MS / 1000.0
        self.workstation_camera_discovery_status = (
            f"RETRYING AUTO_CONNECT {next_attempt}/{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS}"
        )
        self.camera_connection_label.setText(
            f"WIRIS auto connect: waiting for camera service "
            f"({next_attempt}/{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS})"
        )
        self._log(
            f"[카메라] WIRIS 자동 연결 재시도 예약: {reason}; "
            f"{delay_s:.1f}초 후 {next_attempt}/{CAMERA_AUTO_CONNECT_MAX_ATTEMPTS}회 시도"
        )

        def retry() -> None:
            if generation != self._camera_auto_connect_retry_generation:
                return
            if not self.auto_wiris_connect_pending:
                return
            if (
                self.camera_auto_detect_thread is not None
                or self.rtsp_enable_thread is not None
                or self.rtsp_check_thread is not None
            ):
                self._camera_control_runtime().schedule_retry(100, retry)
                return
            self._start_camera_detection("auto_connect")

        self._camera_control_runtime().schedule_retry(CAMERA_AUTO_CONNECT_RETRY_DELAY_MS, retry)

    def _apply_camera_detection(self, result: object) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and not owner.accepts_legacy_outcome('detect'):
            return
        if isinstance(result, dict):
            mode = str(result.get("mode") or "rtsp")
            target_side = result.get("target_side")
            host_interfaces = list(result.get("host_interfaces") or [])
            rtsp_candidates = list(result.get("rtsp") or [])
            control_candidates = list(result.get("control") or [])
            verified_control_candidates = list(result.get("verified_control") or [])
            arp_scan_reports = list(result.get("arp_scan") or [])
            full_control_scan = bool(result.get("full_control_scan", False))
        else:
            mode = "rtsp"
            target_side = None
            host_interfaces = []
            rtsp_candidates = list(result or [])
            control_candidates = []
            verified_control_candidates = []
            arp_scan_reports = []
            full_control_scan = False
        self._log_arp_scan_reports(arp_scan_reports)
        self._set_camera_candidate_rows(host_interfaces, rtsp_candidates, control_candidates, verified_control_candidates)
        if mode == "auto_connect" and self._session_recording_active():
            self._cancel_auto_wiris_connect_retry()
            self.camera_connection_label.setText("WIRIS auto connect: recording is active")
            self._log("[카메라] 탐색 중 데이터 기록이 시작되어 자동 후보 적용과 프리뷰 연결을 건너뛰었습니다.")
            return
        if not (host_interfaces or rtsp_candidates or control_candidates or verified_control_candidates):
            self.workstation_camera_discovery_status = "FOUND 0 ENDPOINTS"
        if host_interfaces:
            self._apply_host_interface_candidates(host_interfaces)
        if mode == "auto_connect":
            self._apply_auto_wiris_connect_candidates(
                verified_control_candidates,
                control_candidates,
                full_control_scan=full_control_scan,
            )
        elif mode == "control":
            self._apply_control_candidates(verified_control_candidates, control_candidates, target_side=target_side)
        elif not rtsp_candidates:
            self._log("[연결] RTSP 후보를 찾지 못했습니다. 카메라 전원, IP 대역, RTSP 포트를 확인하세요.")
            self.camera_connection_label.setText("camera RTSP: auto detect found no candidates")
        else:
            self._apply_rtsp_candidates(rtsp_candidates)

    def _set_camera_candidate_rows(self, host_interfaces, rtsp_candidates, control_candidates, verified_control_candidates):
        return self.views.candidates._set_camera_candidate_rows(host_interfaces, rtsp_candidates, control_candidates, verified_control_candidates)

    def _set_camera_candidate_table_rows(self, rows):
        return self.views.candidates._set_camera_candidate_table_rows(rows)

    def clear_camera_candidate_history(self):
        return self.views.candidates.clear_camera_candidate_history()

    def _refresh_camera_candidate_actions(self):
        return self.views.candidates._refresh_camera_candidate_actions()

    def _selected_camera_candidate(self):
        return self.views.candidates._selected_camera_candidate()

    def apply_selected_camera_candidate_nic(self) -> bool:
        entry = self._selected_camera_candidate()
        if entry is None:
            return False
        interface = entry.get("interface")
        if interface is None:
            self._log("[연결] 선택 후보에 적용 가능한 NIC 정보가 없습니다.")
            return False
        interface_name = str(getattr(interface, "name", ""))
        candidate = {
            "kind": "NETWORK",
            "interface": interface_name,
            "local_cidr": str(getattr(interface, "address", "")),
            "candidate_id": f"camera:{interface_name}",
        }
        return bool(self.request_network_role_assignment("camera", candidate))

    def apply_selected_camera_candidate(self, side: str) -> None:
        entry = self._selected_camera_candidate()
        if entry is None:
            return
        kind = str(entry.get("kind", ""))
        if kind == "NIC":
            self.apply_selected_camera_candidate_nic()
            return
        host = str(entry.get("host") or entry.get("address") or "").strip()
        if not host:
            self._log("[카메라] 선택 후보에 적용 가능한 host가 없습니다.")
            return
        if kind == "RTSP":
            url = str(entry.get("address") or "")
            url_edit, ip_edit = self._rtsp_widgets_for_camera(side)
            url_edit.setText(url)
            ip_edit.setText(host)
        else:
            if entry.get("candidate") is not None:
                self._apply_control_candidate_to_side(side, entry["candidate"])
            elif side == "left":
                self.left_zoom_host_edit.setText(host)
                self.left_stip_ip_edit.setText(host)
            else:
                self.right_zoom_host_edit.setText(host)
                self.right_stip_ip_edit.setText(host)
            self._sync_rtsp_url_from_control_host(side, host)
            try:
                port = int(str(entry.get("port") or "0"))
            except ValueError:
                port = 0
            if 1 <= port <= 65535:
                self._set_control_port_for_camera(side, port)
        interface = entry.get("interface")
        if interface is not None:
            self.interface_edit.setText(str(getattr(interface, "name", "")))
            self.host_address_edit.setText(str(getattr(interface, "address", "")))
        self._log(f"[카메라] 선택 후보를 {self._camera_name_ko(side)}에 적용했습니다: {kind} {host}")
        self._refresh_operation_header()

    def apply_camera_candidate(self, row: int, target: str) -> bool:
        """Apply a discovered camera endpoint selected by the Workstation surface."""

        if self._session_recording_active():
            self._log("[카메라] 데이터 기록 중에는 카메라 네트워크/Left/Right 후보를 적용할 수 없습니다.")
            return False
        if isinstance(row, bool) or not isinstance(row, int) or not 0 <= row < len(self._camera_candidate_rows):
            self._log("[카메라] 적용할 탐색 후보가 올바르지 않습니다.")
            return False
        if target not in {"nic", "left", "right"}:
            self._log(f"[카메라] 지원하지 않는 후보 적용 대상입니다: {target}")
            return False
        self.camera_candidate_table.selectRow(row)
        if target == "nic":
            return self.apply_selected_camera_candidate_nic()
        else:
            self.apply_selected_camera_candidate(target)
        return True

    def _log_arp_scan_reports(self, reports: list[object]) -> None:
        if not reports:
            self._log("[연결] arp-scan 결과 없음: 설치되지 않았거나 스캔 가능한 로컬 IPv4 /24 대역이 없습니다.")
            return
        for report in reports:
            if not getattr(report, "available", False):
                error = getattr(report, "error", "") or "unavailable"
                self._log(f"[연결] arp-scan 사용 불가: {error}")
                continue
            interface = getattr(report, "interface", None)
            interface_name = getattr(interface, "name", "unknown")
            network = getattr(report, "network", "")
            hosts = tuple(getattr(report, "hosts", ()) or ())
            returncode = getattr(report, "returncode", None)
            error = getattr(report, "error", "")
            if returncode not in (0, None):
                detail = f": {error}" if error else ""
                self._log(f"[연결] arp-scan 실패: {interface_name} {network} rc={returncode}{detail}")
                continue
            host_text = ", ".join(hosts) if hosts else "none"
            self._log(f"[연결] arp-scan: {interface_name} {network}, hosts={host_text}")

    def _apply_host_interface_candidates(self, candidates: list[object]) -> None:
        selected = candidates[0]
        interface = selected.interface
        self.interface_edit.setText(interface.name)
        self.host_address_edit.setText(str(interface.address))
        neighbor_hosts = tuple(getattr(selected, "neighbor_hosts", ()) or ())
        neighbor_text = f", neighbors={', '.join(neighbor_hosts)}" if neighbor_hosts else ""
        self._log(f"[연결] Host IP 자동 선택: {interface.name} {interface.address}{neighbor_text}")
        for index, candidate in enumerate(candidates[1:], start=2):
            candidate_interface = candidate.interface
            candidate_neighbors = tuple(getattr(candidate, "neighbor_hosts", ()) or ())
            candidate_neighbor_text = (
                f", neighbors={', '.join(candidate_neighbors)}" if candidate_neighbors else ""
            )
            self._log(
                f"[연결] Host IP 후보 {index}: "
                f"{candidate_interface.name} {candidate_interface.address}{candidate_neighbor_text}"
            )

    def _rtsp_url_host(self, url_edit: QtWidgets.QLineEdit) -> str | None:
        try:
            host, _port = rtsp_host_port(url_edit.text().strip())
        except Exception:
            return None
        return host

    def _normalized_rtsp_identity(self, url: str) -> tuple[str, int, str] | None:
        try:
            host, port = rtsp_host_port(url.strip())
        except Exception:
            return None
        path = ""
        try:
            path = urlparse(url.strip()).path or ""
        except Exception:
            path = ""
        return (host, int(port), path)

    def _rtsp_urls_are_same_stream(self) -> bool:
        left_identity = self._normalized_rtsp_identity(self.left_url_edit.text())
        right_identity = self._normalized_rtsp_identity(self.right_url_edit.text())
        return left_identity is not None and left_identity == right_identity

    def _preferred_rtsp_side_for_candidate(self, candidate: object) -> str | None:
        candidate_host = str(_control_candidate_value(candidate, "host", "") or "").strip()
        if not candidate_host:
            return None
        scores: list[tuple[int, str]] = []
        for side, control_edit, url_edit in (
            ("left", self.left_zoom_host_edit, self.left_url_edit),
            ("right", self.right_zoom_host_edit, self.right_url_edit),
        ):
            score = 0
            if control_edit.text().strip() == candidate_host:
                score += 2
            if self._rtsp_url_host(url_edit) == candidate_host:
                score += 1
            if score:
                scores.append((score, side))
        if not scores:
            return None
        scores.sort(key=lambda item: (item[0], 1 if item[1] == "right" else 0), reverse=True)
        if len(scores) > 1 and scores[0][0] == scores[1][0]:
            return None
        return scores[0][1]

    def _apply_rtsp_candidates(self, candidates: list[object]) -> None:
        selected = candidates[:2]
        self._log(f"[카메라] RTSP 후보 {len(candidates)}개 발견")
        for index, candidate in enumerate(selected, start=1):
            interface_text = ""
            if getattr(candidate, "interface", None) is not None:
                interface = candidate.interface
                interface_text = f", interface={interface.name}, host={interface.address}"
            self._log(
                f"[카메라] RTSP 후보 {index}: {candidate.url} "
                f"(host={candidate.host}, port={candidate.port}{interface_text})"
            )
        host_interface = getattr(selected[0], "interface", None)
        if host_interface is None:
            host_interface = select_host_for_camera(selected[0].host)
        if host_interface is not None:
            self.interface_edit.setText(host_interface.name)
            self.host_address_edit.setText(str(host_interface.address))
            self._log(f"[연결] Host IP 자동 선택: {host_interface.name} {host_interface.address}")
        if len(selected) == 1:
            candidate = selected[0]
            preferred_side = self._preferred_rtsp_side_for_candidate(candidate)
            if preferred_side == "left":
                self.left_url_edit.setText(candidate.url)
                self.left_rtsp_ip_edit.setText(candidate.host)
                self.left_zoom_host_edit.setText(candidate.host)
                self._log(
                    "[카메라] RTSP 후보가 1개만 발견되어 Left에만 자동 입력했습니다. "
                    "Right는 기존 URL을 유지합니다."
                )
            elif preferred_side == "right":
                self.right_url_edit.setText(candidate.url)
                self.right_rtsp_ip_edit.setText(candidate.host)
                self.right_zoom_host_edit.setText(candidate.host)
                self._log(
                    "[카메라] RTSP 후보가 1개만 발견되어 Right에만 자동 입력했습니다. "
                    "Left는 기존 URL을 유지합니다."
                )
            else:
                self._log(
                    "[카메라] RTSP 후보가 1개만 발견되었지만 좌/우 매칭이 불확실해서 "
                    "기존 URL을 유지했습니다."
                )
        else:
            self.left_url_edit.setText(selected[0].url)
            self.left_rtsp_ip_edit.setText(selected[0].host)
            self.left_zoom_host_edit.setText(selected[0].host)
            self.right_url_edit.setText(selected[1].url)
            self.right_rtsp_ip_edit.setText(selected[1].host)
            self.right_zoom_host_edit.setText(selected[1].host)
            self._log("[카메라] RTSP 후보 2개를 Left/Right에 자동 입력했습니다.")
        self.check_cameras()

    def _log_verified_control_candidates(self, verified: list[object]) -> None:
        for candidate in verified:
            host = str(_control_candidate_value(candidate, "host", "") or "")
            response = str(_control_candidate_value(candidate, "response", "") or "")
            ok_text = "OK" if _control_candidate_value(candidate, "ok") else "FAIL"
            self._log(
                f"[카메라] WIRIS SDK 후보 검증: "
                f"{host}:{_control_candidate_value(candidate, 'port')} HIWS={ok_text} 응답={response or '<empty>'}"
            )

    def _ok_control_candidates(self, verified: list[object]) -> list[object]:
        return [candidate for candidate in verified if bool(_control_candidate_value(candidate, "ok"))]

    def _candidate_control_port(self, candidate: object) -> int | None:
        try:
            port = int(_control_candidate_value(candidate, "port", 0) or 0)
        except (TypeError, ValueError):
            return None
        return port if 1 <= port <= 65535 else None

    def _apply_discovered_control_port(self, side: str, candidate: object) -> None:
        port = self._candidate_control_port(candidate)
        if port is None:
            return
        self._set_control_port_for_camera(side, port)
        self._log(f"[카메라] {self._camera_name_ko(side)} WIRIS 제어 포트를 발견된 값으로 설정했습니다: {port}")

    def _apply_control_candidate_to_side(self, side: str, candidate: object) -> None:
        host = str(_control_candidate_value(candidate, "host", "") or "")
        if side == "left":
            self.left_zoom_host_edit.setText(host)
            self.left_stip_ip_edit.setText(host)
        else:
            self.right_zoom_host_edit.setText(host)
            self.right_stip_ip_edit.setText(host)
        self._apply_discovered_control_port(side, candidate)
        interface = _control_candidate_value(candidate, "interface")
        if interface is not None:
            self.interface_edit.setText(interface.name)
            self.host_address_edit.setText(str(interface.address))
            self._log(f"[연결] Host IP 자동 선택: {interface.name} {interface.address}")

    def _apply_discovered_control_port_hint(self, candidates: list[object]) -> None:
        ports: list[int] = []
        for candidate in candidates:
            port = self._candidate_control_port(candidate)
            if port is None:
                continue
            if port not in ports:
                ports.append(port)
        if not ports:
            return
        selected_port = ports[0]
        if self.zoom_port_spin.value() != selected_port:
            self.zoom_port_spin.setValue(selected_port)
            self._log(f"[카메라] WIRIS 스캔 기본 포트를 발견된 값으로 설정했습니다: {selected_port}")
        if len(ports) > 1:
            self._log("[카메라] 좌/우 WIRIS 제어 포트가 서로 달라 각 카메라별 포트에 따로 적용합니다.")

    def _apply_control_candidates(
        self,
        verified: list[object],
        fallback: list[object] | None = None,
        *,
        target_side: object | None = None,
    ) -> None:
        fallback = list(fallback or [])
        if verified:
            self._log_verified_control_candidates(verified)
        candidates = self._ok_control_candidates(verified) if verified else list(fallback)
        if not candidates:
            if fallback:
                self._log("[카메라] 열린 TCP 포트 후보는 있으나 HIWS=OK 응답을 받은 WIRIS를 찾지 못했습니다.")
            else:
                self._log("[카메라] WIRIS 제어 포트 후보를 찾지 못했습니다. IP 대역, 전원, 제어 포트를 확인하세요.")
            self.camera_connection_label.setText("camera control: detect found no candidates")
            return
        selected = candidates[:2]
        self._apply_discovered_control_port_hint(selected)
        self._log(f"[카메라] WIRIS 제어 후보 {len(candidates)}개 발견")
        for index, candidate in enumerate(selected, start=1):
            interface_text = ""
            interface = _control_candidate_value(candidate, "interface")
            if interface is not None:
                interface_text = f", interface={interface.name}, host={interface.address}"
            self._log(
                f"[카메라] WIRIS 제어 후보 {index}: "
                f"{_control_candidate_value(candidate, 'host')}:{_control_candidate_value(candidate, 'port')}{interface_text}"
            )
        if target_side in {"left", "right"}:
            self._apply_control_candidate_to_side(str(target_side), selected[0])
            self._log(f"[카메라] WIRIS 제어 후보를 {self._camera_name_ko(str(target_side))}에만 입력했습니다.")
        elif len(selected) == 1:
            self._apply_control_candidate_to_side("left", selected[0])
            self._log("[카메라] WIRIS 제어 후보가 1개만 발견되어 Left Control Host에만 자동 입력했습니다.")
        else:
            self._apply_control_candidate_to_side("left", selected[0])
            self._apply_control_candidate_to_side("right", selected[1])
            self._log("[카메라] WIRIS 제어 후보 2개를 Left/Right Control Host에 자동 입력했습니다.")
        self.check_wiris_control()
        self._refresh_operation_header()

    def _apply_auto_wiris_connect_candidates(
        self,
        verified: list[object],
        fallback: list[object],
        *,
        full_control_scan: bool = False,
    ) -> None:
        ok_candidates = self._ok_control_candidates(verified)
        self._log_verified_control_candidates(verified)
        if not ok_candidates:
            if fallback:
                reason = "열린 포트에서 HIWS=OK 응답 없음"
                self._log("[카메라] 열린 TCP 포트 후보는 있으나 HIWS=OK 응답을 받은 WIRIS를 찾지 못했습니다.")
            else:
                reason = "WIRIS 제어 포트가 아직 열리지 않음"
                self._log("[카메라] WIRIS 제어 포트 후보를 찾지 못했습니다. PC IP 대역 또는 카메라 IP를 확인하세요.")
            if verified:
                open_ports = ", ".join(
                    f"{_control_candidate_value(candidate, 'host')}:{_control_candidate_value(candidate, 'port')}"
                    for candidate in verified
                )
                self._log(
                    "[카메라] 열린 포트의 HIWS 검증 결과: "
                    f"{open_ports}; WIRIS SDK 응답 없음"
                )
            elif full_control_scan:
                self._log(
                    "[카메라] 설정/기본 포트와 전체 TCP 1-65535 스캔 모두에서 "
                    "WIRIS SDK 포트를 찾지 못했습니다."
                )
            self._schedule_auto_wiris_connect_retry(reason)
            self._refresh_operation_header()
            return

        self._camera_auto_connect_retry_generation += 1
        selected = ok_candidates[:2]
        self._apply_discovered_control_port_hint(selected)
        self._apply_control_candidate_to_side("left", selected[0])
        left_host = str(_control_candidate_value(selected[0], "host", "") or "")
        self.left_zoom_host_edit.setText(left_host)
        self.left_stip_ip_edit.setText(left_host)
        self._sync_rtsp_url_from_control_host("left", left_host)
        if len(selected) > 1:
            self._apply_control_candidate_to_side("right", selected[1])
            right_host = str(_control_candidate_value(selected[1], "host", "") or "")
            self.right_zoom_host_edit.setText(right_host)
            self.right_stip_ip_edit.setText(right_host)
            self._sync_rtsp_url_from_control_host("right", right_host)
        if len(selected) == 1:
            self._log("[카메라] HIWS=OK WIRIS가 1개만 발견되어 Left Control Host에 입력했습니다.")
        else:
            self._log("[카메라] HIWS=OK WIRIS 2개를 Left/Right Control Host에 입력했습니다.")
        selected_sides = ("left",) if len(selected) == 1 else ("left", "right")
        activation_targets: list[str] = []
        for side, candidate in zip(selected_sides, selected):
            host = str(_control_candidate_value(candidate, "host", "") or "").strip()
            camera_key = (side, host)
            if camera_key in self._auto_wiris_enabled_cameras:
                self._log(
                    f"[카메라] {self._camera_name_ko(side)}는 이미 SETH TRUE가 승인되어 "
                    "RTSP 프레임만 다시 확인합니다."
                )
                continue
            activation_targets.append(side)

        if activation_targets:
            self.camera_connection_label.setText("WIRIS auto connect: control verified, enabling RTSP")
            self.enable_wiris_rtsp(target_labels=tuple(activation_targets))
        else:
            self.camera_connection_label.setText("WIRIS auto connect: control verified, checking RTSP")
            self.check_cameras()




















    def _camera_auto_detect_failed(self, error: str) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and not owner.accepts_legacy_outcome('detect'):
            return
        self.workstation_camera_discovery_status = f"ERROR · {error}"
        self._log(f"[카메라] 자동 탐색 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP: auto detect failed ({error})")
        self.last_rtsp_check_status = {"left": "check-failed", "right": "check-failed"}
        if self.auto_wiris_connect_pending:
            self._schedule_auto_wiris_connect_retry(f"탐색 오류: {error}")
        self._refresh_operation_header()

    def _clear_camera_auto_detect_worker(self) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and 'detect' in owner.jobs:
            return
        self.camera_auto_detect_thread = None
        self.camera_auto_detect_worker = None
        self.auto_wiris_connect_button.setEnabled(True)
        self.detect_control_button.setEnabled(True)
        self.right_detect_control_button.setEnabled(True)
        self.detect_rtsp_button.setEnabled(True)
        self.right_detect_rtsp_button.setEnabled(True)

    def swap_cameras(self) -> bool | None:
        if self._session_recording_active():
            self._log("[카메라] 데이터 기록 중에는 Left/Right 카메라 매핑을 교체할 수 없습니다.")
            return False
        cameras_running = self.camera_connected or bool(self.left_pane.worker or self.right_pane.worker)
        detection_was_enabled = self.yolo_enabled
        if cameras_running:
            self.stop_yolo_streams()
        self.left_url_edit.setText(self._swap_line_edit_text(self.left_url_edit, self.right_url_edit))
        self.left_rtsp_ip_edit.setText(self._swap_line_edit_text(self.left_rtsp_ip_edit, self.right_rtsp_ip_edit))
        self.left_zoom_host_edit.setText(self._swap_line_edit_text(self.left_zoom_host_edit, self.right_zoom_host_edit))
        self.left_stip_ip_edit.setText(self._swap_line_edit_text(self.left_stip_ip_edit, self.right_stip_ip_edit))
        left_control_port = self._control_port_for_camera("left")
        right_control_port = self._control_port_for_camera("right")
        self._set_control_port_for_camera("left", right_control_port)
        self._set_control_port_for_camera("right", left_control_port)
        self._log("[카메라] Left/Right 카메라 매핑을 교체했습니다.")
        if cameras_running:
            self._start_camera_streams(enable_detection=detection_was_enabled, switch_to_depth=False)
        return None


    def _swap_spinbox_value(self, left: QtWidgets.QAbstractSpinBox, right: QtWidgets.QAbstractSpinBox) -> object:
        left_value = left.value()
        right_value = right.value()
        right.setValue(left_value)
        return right_value

    def _swap_line_edit_text(self, left: QtWidgets.QLineEdit, right: QtWidgets.QLineEdit) -> str:
        left_text = left.text()
        right_text = right.text()
        right.setText(left_text)
        return right_text

    def _finish_auto_wiris_connect(self) -> None:
        startup_sequence = self._startup_camera_sequence_pending
        self._cancel_auto_wiris_connect_retry()
        if startup_sequence:
            self._log(
                "[카메라] 시작 시퀀스 완료: WIRIS Ethernet/RTSP 활성화와 좌/우 프레임 확인 후 "
                "카메라 스트림을 시작합니다."
            )
            self.start_yolo()
            return
        self._log("[카메라] WIRIS 자동 연결: 좌/우 RTSP 프레임 확인 완료. 카메라 프리뷰를 연결합니다.")
        self.connect_cameras()

    def check_cameras(self) -> None:
        if self.rtsp_check_thread is not None:
            self._log("[카메라] RTSP 스트림 확인이 이미 진행 중입니다.")
            return
        self._log("[연결] RTSP 확인은 설정된 카메라망 인터페이스와 URL을 사용합니다.")
        urls = {
            "left": self.left_url_edit.text().strip(),
            "right": self.right_url_edit.text().strip(),
        }
        self._log("[카메라] RTSP 포트와 실제 프레임 수신을 확인합니다.")
        self.check_button.setEnabled(False)
        self.camera_connection_label.setText("camera RTSP: checking stream frames...")
        self.last_rtsp_check_status = {"left": "checking", "right": "checking"}
        self._refresh_operation_header()
        self._camera_control_runtime().launch(
            'check', worker_factory=lambda: RtspStreamCheckWorker(urls, latency_ms=self.rtsp_latency_spin.value(), decoder=self.decoder_combo.currentData(), backend=self.stream_backend_combo.currentData(), transport=self.rtsp_transport_combo.currentData(), timeout_s=RTSP_STREAM_CHECK_TIMEOUT_S),
            completed=self._apply_rtsp_stream_check, failed=self._rtsp_stream_check_failed, retired=self._clear_rtsp_check_worker,
        )

    def check_wiris_control(self) -> None:
        if self.wiris_control_check_thread is not None:
            self._log("[카메라] WIRIS 제어 확인이 이미 진행 중입니다.")
            return
        self._log("[연결] WIRIS 제어 확인은 설정된 Control Host/port를 사용합니다.")
        targets = []
        for label, host in self.zoom_hosts("both"):
            normalized_host = host.strip()
            if normalized_host:
                self._log_rtsp_control_host_mismatch(label, normalized_host)
            targets.append((label, normalized_host, self._control_port_for_camera(label)))
        self.check_control_button.setEnabled(False)
        self.camera_connection_label.setText("camera control: checking...")
        self._camera_control_runtime().launch(
            'control', worker_factory=lambda: WirisControlCheckWorker(targets, timeout_s=self.zoom_timeout_spin.value()),
            completed=self._apply_wiris_control_check, failed=self._wiris_control_check_failed, retired=self._clear_wiris_control_check_worker,
        )

    def _apply_wiris_control_check(self, results: object) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and not owner.accepts_legacy_outcome('control'):
            return
        statuses: list[str] = []
        for result in list(results or []):
            if not isinstance(result, dict):
                continue
            label = str(result.get("label") or "")
            host = str(result.get("host") or "")
            port = result.get("port", "")
            state = str(result.get("state") or "error")
            camera_name = self._camera_name_ko(label)
            statuses.append(f"{label}={state}")
            if state == "missing-host":
                self._log(f"[카메라] {camera_name} WIRIS 제어 host가 비어 있습니다.")
            elif state == "control-open":
                self._log(f"[카메라] {camera_name} WIRIS 제어 포트 열림: {host}:{port}")
            elif state == "control-closed":
                self._log(f"[카메라] {camera_name} WIRIS 제어 포트 닫힘/오류: {host}:{port}")
            else:
                self._log(f"[카메라] {camera_name} WIRIS 제어 확인 오류: {result.get('error', state)}")
        self.camera_connection_label.setText(f"camera control: {', '.join(statuses)}")
        self._refresh_operation_header()

    def _wiris_control_check_failed(self, error: str) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and not owner.accepts_legacy_outcome('control'):
            return
        self._log(f"[카메라] WIRIS 제어 확인 실패: {error}")
        self.camera_connection_label.setText(f"camera control: failed ({error})")
        self._refresh_operation_header()

    def _clear_wiris_control_check_worker(self) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and 'control' in owner.jobs:
            return
        self.wiris_control_check_thread = None
        self.wiris_control_check_worker = None
        self.check_control_button.setEnabled(True)

    def _ethernet_mode_candidate_ports(self, camera: str) -> list[int]:
        ports = [
            self._control_port_for_camera(camera),
            DEFAULT_ZOOM_CONTROL_PORT,
            self.zoom_port_spin.value(),
        ]
        unique_ports = []
        for port in ports:
            port = int(port)
            if 1 <= port <= 65535 and port not in unique_ports:
                unique_ports.append(port)
        return unique_ports

    def _start_rtsp_enable_worker(self, *, target_labels: tuple[str, ...], reactivate: bool) -> None:
        stus_operation = getattr(self, "_camera_sync_stus_operation", None)
        if isinstance(stus_operation, dict) and stus_operation.get("active"):
            self._log("[카메라] STUS closed-loop 중 RTSP 활성화/재활성화를 거부했습니다.")
            return
        if bool(getattr(self, "_shutdown_in_progress", False)):
            self._log("[카메라] application shutdown 중 RTSP 활성화를 거부했습니다.")
            return
        hosts = {label: host.strip() for label, host in self.zoom_hosts("both")}
        candidate_ports = {label: self._ethernet_mode_candidate_ports(label) for label in ("left", "right")}
        urls = {
            "left": self.left_url_edit.text().strip(),
            "right": self.right_url_edit.text().strip(),
        }
        self.enable_rtsp_button.setEnabled(False)
        status_text = "camera RTSP reactivate: enabling safely..." if reactivate else "camera RTSP enable: enabling..."
        self.camera_connection_label.setText(status_text)
        self._camera_control_runtime().launch(
            'enable', worker_factory=lambda: WirisRtspEnableWorker(hosts=hosts, candidate_ports=candidate_ports, urls=urls, timeout_s=self.zoom_timeout_spin.value(), target_labels=target_labels, reactivate=reactivate),
            completed=self._apply_wiris_rtsp_enable_result, failed=self._wiris_rtsp_enable_failed, retired=self._clear_rtsp_enable_worker,
        )

    def enable_wiris_rtsp(self, *, target_labels: tuple[str, ...] = ("left", "right")) -> None:
        stus_operation = getattr(self, "_camera_sync_stus_operation", None)
        if isinstance(stus_operation, dict) and stus_operation.get("active"):
            self._log("[카메라] STUS closed-loop 중 RTSP 활성화를 거부했습니다.")
            return
        if self.rtsp_enable_thread is not None:
            self._log("[카메라] RTSP 송출 활성화가 이미 진행 중입니다.")
            return
        if self._rtsp_reactivation_side is not None:
            self._log("[카메라] 카메라별 RTSP 재활성화가 진행 중입니다.")
            return
        self._log("[연결] RTSP 활성화는 설정된 Control Host/port와 영상 URL을 사용합니다.")
        self._start_rtsp_enable_worker(target_labels=target_labels, reactivate=False)

    def reactivate_wiris_rtsp(self, camera: str) -> bool | None:
        if camera not in {"left", "right"}:
            self._log(f"[카메라] 알 수 없는 재활성화 대상입니다: {camera}")
            return False
        stus_operation = getattr(self, "_camera_sync_stus_operation", None)
        if isinstance(stus_operation, dict) and stus_operation.get("active"):
            self._log("[카메라] STUS closed-loop 중 RTSP 재활성화를 거부했습니다.")
            return False
        if self._session_recording_active() and not bool(
            getattr(self, "_automatic_session_recording", False)
        ):
            self._log(
                "[카메라] 수동 기록 중에는 RTSP 재활성화를 실행할 수 없습니다. "
                "Measurement 관측 중 source 복구만 허용됩니다."
            )
            return False
        if self.rtsp_enable_thread is not None or self.rtsp_check_thread is not None:
            self._log("[카메라] 다른 RTSP 확인/활성화 작업이 진행 중입니다.")
            return False
        host = dict(self.zoom_hosts("both")).get(camera, "").strip()
        if not host:
            self._log(f"[카메라] {self._camera_name_ko(camera)} Control Host가 비어 있어 재활성화할 수 없습니다.")
            self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=missing-host")
            return False
        if self.auto_wiris_connect_pending:
            self._cancel_auto_wiris_connect_retry()

        self._rtsp_reactivation_side = camera
        run_state = str(
            getattr(self, "_measurement_run_state", "IDLE") or "IDLE"
        ).upper()
        self._rtsp_reactivation_enable_detection = bool(
            self.yolo_enabled or run_state == "RUNNING"
        )
        self._rtsp_reactivation_previous_video_path = (
            Path(self.video_recording_paths[camera])
            if self._session_recording_active()
            and camera in self.video_recording_paths
            else None
        )
        self._rtsp_reactivation_generation += 1
        generation = self._rtsp_reactivation_generation
        self._rtsp_reactivation_stop_generation = generation
        self._rtsp_reactivation_stop_timer.stop()
        camera_name = self._camera_name_ko(camera)
        source_publisher = getattr(self, "measurement_source_transition", None)
        if callable(source_publisher) and run_state == "RUNNING":
            source_publisher(
                f"camera.{camera}",
                "WAITING_RESOURCE",
                "manual RTSP reactivation requested",
            )
            source_publisher(
                f"recording.{camera}",
                "WAITING_RESOURCE",
                "camera recovery pending",
            )
        self._log(
            f"[카메라] {camera_name} RTSP 재활성화를 시작합니다: "
            "스트림 정리 → GETH 확인 → 비활성 시만 SETH TRUE → 프레임 확인 → Preview 재연결 "
            "(제어 서비스 보호를 위해 SETH FALSE 생략)"
        )
        self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=stopping")
        pane = self.left_pane if camera == "left" else self.right_pane
        pane.stop(wait=False)
        self._continue_rtsp_reactivation_after_stop()
        return None

    def _continue_rtsp_reactivation_after_stop(self) -> None:
        generation = self._rtsp_reactivation_stop_generation
        camera = self._rtsp_reactivation_side
        if (
            generation is None
            or camera not in {"left", "right"}
            or generation != self._rtsp_reactivation_generation
        ):
            self._rtsp_reactivation_stop_timer.stop()
            self._rtsp_reactivation_stop_generation = None
            return
        if bool(getattr(self, "_shutdown_in_progress", False)):
            self._finish_camera_rtsp_reactivation(
                False,
                "application shutdown interrupted reactivation",
            )
            return
        pane = self.left_pane if camera == "left" else self.right_pane
        if pane.worker is not None or pane.thread is not None:
            pane.stop(wait=False)
            self._rtsp_reactivation_stop_timer.start()
            return
        previous_video_path = self._rtsp_reactivation_previous_video_path
        if previous_video_path is not None:
            recorder = getattr(self, "record_measurement_video_segment_presence", None)
            if callable(recorder):
                recorder(
                    camera,
                    previous_video_path,
                    detail="camera side stopped for manual RTSP reactivation",
                )
            self._rtsp_reactivation_previous_video_path = None
        self._rtsp_reactivation_stop_timer.stop()
        self._rtsp_reactivation_stop_generation = None
        self._start_rtsp_enable_worker(target_labels=(camera,), reactivate=True)

    def _apply_wiris_rtsp_enable_result(self, result: object) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and not owner.accepts_legacy_outcome('enable'):
            return
        data = dict(result or {})
        for label, port in dict(data.get("corrected_ports") or {}).items():
            self._set_control_port_for_camera(str(label), int(port))
            self.zoom_port_spin.setValue(int(port))
            self._save_current_settings()
        for message in list(data.get("logs") or []):
            self._log(str(message))
        statuses = [str(status) for status in list(data.get("statuses") or [])]
        mode = str(data.get("mode") or "enable")
        if mode == "enable" and self.auto_wiris_connect_pending:
            for status in statuses:
                label, separator, state = status.partition("=")
                if not separator or state != "enable-sent" or label not in {"left", "right"}:
                    continue
                host = dict(self.zoom_hosts("both")).get(label, "").strip()
                if host:
                    self._auto_wiris_enabled_cameras.add((label, host))
                    self._log(
                        f"[카메라] {self._camera_name_ko(label)} GETH 활성 상태를 기억합니다. "
                        "RTSP 실패 재시도에서는 불필요한 SETH TRUE를 전송하지 않습니다."
                    )
        self.camera_connection_label.setText(
            f"camera RTSP {'reactivate' if mode == 'reactivate' else 'enable'}: {', '.join(statuses)}"
        )
        if mode == "reactivate" and self._rtsp_reactivation_side is not None:
            camera = self._rtsp_reactivation_side
            expected_status = f"{camera}=reactivated-sent"
            if expected_status not in statuses:
                self._finish_camera_rtsp_reactivation(False, f"{camera}=reactivation-failed")
            elif data.get("should_check"):
                self._log(f"[카메라] {self._camera_name_ko(camera)} RTSP 재활성화 후 실제 프레임을 확인합니다.")
                self.check_cameras()
            else:
                self._finish_camera_rtsp_reactivation(False, f"{camera}=no-stream")
            return
        if data.get("should_check"):
            self.check_cameras()
        elif self.auto_wiris_connect_pending:
            self._schedule_auto_wiris_connect_retry("RTSP 활성화에 성공한 카메라가 없음")

    def _wiris_rtsp_enable_failed(self, error: str) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and not owner.accepts_legacy_outcome('enable'):
            return
        self._log(f"[카메라] RTSP 송출 활성화 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP enable: failed ({error})")
        if self._rtsp_reactivation_side is not None:
            self._finish_camera_rtsp_reactivation(False, error)
            return
        if self.auto_wiris_connect_pending:
            self._schedule_auto_wiris_connect_retry(f"RTSP 활성화 오류: {error}")

    def _clear_rtsp_enable_worker(self) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and 'enable' in owner.jobs:
            return
        self.rtsp_enable_thread = None
        self.rtsp_enable_worker = None
        self.enable_rtsp_button.setEnabled(True)

    def _finish_camera_rtsp_reactivation(self, success: bool, detail: str = "") -> None:
        owner = self.state.control_jobs
        if owner._rtsp_reactivation_side is None:
            return

        def side_state(camera):
            pane = self.left_pane if camera == "left" else self.right_pane
            return SideStreamState(
                int(getattr(pane, "stream_generation", -1)),
                getattr(pane, "worker", None) is not None or getattr(pane, "thread", None) is not None,
                bool(getattr(pane, "detection_stream_active", False)),
            )

        def capture_target(camera):
            pane = self.left_pane if camera == "left" else self.right_pane
            url = self.left_url_edit.text().strip() if camera == "left" else self.right_url_edit.text().strip()
            return RejoinTarget(
                url, pane.start_stream,
                lambda: SideStreamState(
                    int(getattr(pane, "stream_generation", -1)),
                    getattr(pane, "worker", None) is not None or getattr(pane, "thread", None) is not None,
                    bool(getattr(pane, "detection_stream_active", False)),
                ),
            )

        def prepare_preview(camera, enable_detection):
            self.camera_frame_times[camera].clear()
            self.camera_frame_metadata[camera] = {}
            self.camera_frame_metadata_history[camera].clear()
            self.camera_stream_stats[camera] = {}
            self.active_stream_decoders[camera] = None
            if enable_detection:
                self.tabs.setCurrentIndex(self.depth_tab_index)
            self._camera_sync_transport_override = None
            self._set_camera_sync_state(
                CAMERA_SYNC_STATUS_NOT_VERIFIED,
                f"{self._camera_name_ko(camera)} RTSP가 재활성화되어 RTCP/NTP 검증이 필요합니다.",
            )

        def read_recording():
            reader = getattr(self, "_session_recording_active", None)
            return RejoinRecordingSnapshot(
                bool(callable(reader) and reader()),
                str(getattr(self, "_measurement_run_state", "IDLE") or "IDLE").upper(),
            )

        def start_recording():
            start = getattr(self, "start_automatic_session_recording", None)
            return bool(callable(start) and start())

        def allocate(camera):
            allocator = getattr(self, "allocate_measurement_video_segment", None)
            return allocator(camera) if callable(allocator) else None

        def install(camera, path):
            self.video_recording_paths[camera] = Path(path)
            self._video_recording_allocation_sizes[camera] = 0
            self.video_recording_enabled = True

        def publish(source, state, reason):
            publisher = getattr(self, "measurement_source_transition", None)
            if callable(publisher):
                publisher(source, state, reason)

        def availability(sides, detection):
            self.camera_connected = bool(sides)
            self.yolo_enabled = bool(detection)

        def report(camera, phase, reason, integrity_pending):
            camera_name = self._camera_name_ko(camera)
            if phase == "failed":
                self._log(f"[카메라] {camera_name} RTSP 재활성화 실패: {reason}")
                self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=failed")
            elif phase == "preview-start-failed":
                self._log(f"[카메라] {camera_name} RTSP 재활성화 후 Preview 재연결을 시작하지 못했습니다.")
                self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=preview-start-failed")
            else:
                self.camera_connection_label.setText(f"camera RTSP reactivate: {camera}=current-frame verification pending")
                if integrity_pending:
                    self._log(
                        f"[카메라] {camera_name} Preview worker는 재시작했지만 "
                        "LIVE는 newer-generation current frame 후에만 회복합니다."
                    )
                else:
                    self._log(f"[카메라] {camera_name} worker를 재시작했고 current-generation frame을 기다립니다.")

        def refresh(include_session):
            if include_session:
                self._refresh_session_toggle_buttons()
            self._refresh_operation_header()

        owner.finish_reactivation(success, detail, recovery=self.state.recovery, ports=ReactivationPorts(
            capture_target=capture_target, prepare_preview=prepare_preview, read_recording=read_recording,
            start_recording=start_recording, current_segment=lambda camera: self.video_recording_paths.get(camera),
            allocate_segment=allocate, install_segment=install,
            stream_options=lambda camera: self.yolo_kwargs(camera),
            recording_options=lambda camera: self._video_recording_kwargs(camera),
            read_side=side_state, publish_source=publish, set_availability=availability,
            report=report, refresh=refresh, log=self._log,
        ))

    def _rtsp_widgets_for_camera(self, camera: str) -> tuple[QtWidgets.QLineEdit, QtWidgets.QLineEdit]:
        if camera == "left":
            return self.left_url_edit, self.left_rtsp_ip_edit
        return self.right_url_edit, self.right_rtsp_ip_edit

    def _sync_rtsp_url_from_control_host(self, camera: str, host: str) -> None:
        url_edit, ip_edit = self._rtsp_widgets_for_camera(camera)
        old_host = self._rtsp_url_host_only(url_edit.text())
        self._replace_rtsp_url_host(url_edit, host)
        ip_edit.setText(host)
        if old_host != host:
            camera_name = self._camera_name_ko(camera)
            self._log(f"[카메라] {camera_name} RTSP URL host 자동 갱신: {old_host or 'invalid'} -> {host}")

    def _log_rtsp_control_host_mismatch(self, camera: str, control_host: str) -> None:
        url_edit, _ip_edit = self._rtsp_widgets_for_camera(camera)
        rtsp_host = self._rtsp_url_host_only(url_edit.text())
        if not rtsp_host or rtsp_host == control_host:
            return
        camera_name = self._camera_name_ko(camera)
        self._log(
            f"[카메라] 경고: {camera_name} OSD IP Address({control_host})와 "
            f"RTSP URL host({rtsp_host})가 다릅니다. RTSP 변경은 RTSP port / 영상 그룹의 IP Address 적용을 사용하세요."
        )

    def apply_control_settings(self, camera: str) -> None:
        camera_name = self._camera_name_ko(camera)
        host_edit = self.left_zoom_host_edit if camera == "left" else self.right_zoom_host_edit
        host = host_edit.text().strip()
        port = self._control_port_for_camera(camera)
        if not host:
            self._log(f"[카메라] {camera_name} IP Address/Telnet port 적용 실패: IP Address가 비어 있습니다.")
            self.camera_connection_label.setText(f"camera control: {camera}=missing-host")
            self._refresh_operation_header()
            return
        self._sync_rtsp_url_from_control_host(camera, host)
        self._save_current_settings()
        self._log(f"[카메라] {camera_name} IP Address/Telnet port 적용: {host}:{port}")
        self.camera_connection_label.setText(f"camera control: {camera}=applied {host}:{port}")
        self._refresh_operation_header()

    def apply_rtsp_url(self, camera: str) -> None:
        camera_name = self._camera_name_ko(camera)
        url_edit, ip_edit = self._rtsp_widgets_for_camera(camera)
        url = url_edit.text().strip()
        new_ip = ip_edit.text().strip()
        try:
            IPv4Address(new_ip)
            current_host, port = rtsp_host_port(url)
        except Exception as exc:
            self._log(f"[카메라] {camera_name} RTSP 입력 오류: {exc}")
            self.camera_connection_label.setText(f"camera RTSP: {camera}=invalid-input")
            return
        if current_host == new_ip:
            self._log(f"[카메라] {camera_name} RTSP URL 적용 건너뜀: 현재 URL host와 같습니다 ({new_ip}).")
            self.camera_connection_label.setText(f"camera RTSP: {camera}=unchanged")
            return
        self._replace_rtsp_url_host(url_edit, new_ip)
        self._log(f"[카메라] {camera_name} RTSP URL 적용: {current_host} -> {new_ip} (port={port})")
        self.camera_connection_label.setText(f"camera RTSP: {camera}={current_host}->{new_ip}")
        self._refresh_operation_header()

    def _replace_rtsp_url_host(self, url_edit: QtWidgets.QLineEdit, new_host: str) -> None:
        url = url_edit.text().strip()
        try:
            _old_host, port = rtsp_host_port(url)
        except Exception:
            url_edit.setText(f"rtsp://{new_host}:8554/visible")
            return
        suffix = "/visible"
        if url.startswith("rtsp://") and "/" in url.removeprefix("rtsp://"):
            suffix = "/" + url.removeprefix("rtsp://").split("/", 1)[1]
        url_edit.setText(f"rtsp://{new_host}:{port}{suffix}")

    def _apply_rtsp_stream_check(self, results: object) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and not owner.accepts_legacy_outcome('check'):
            return
        statuses = []
        next_status = {}
        for label in ("left", "right"):
            result = dict((results or {}).get(label, {}))
            camera_name = self._camera_name_ko(label)
            if not result.get("port_open"):
                error = result.get("error", "port closed")
                self._log(f"[카메라] {camera_name} RTSP 포트 닫힘/오류: {error}")
                statuses.append(f"{label}=port-closed")
                next_status[label] = "port-closed"
                continue
            if not result.get("frame_ok"):
                error = result.get("error", "no frame")
                decoder = result.get("decoder", self.decoder_combo.currentText())
                self._log(f"[카메라] {camera_name} RTSP 포트 열림, 프레임 수신 실패: {error} (decoder={decoder})")
                statuses.append(f"{label}=open-no-frame")
                next_status[label] = "open-no-frame"
                continue
            pts = result.get("pts_s")
            pts_text = "PTS=n/a" if pts is None else f"PTS={float(pts):.3f}s"
            decoder = result.get("decoder", self.decoder_combo.currentText())
            backend = result.get("backend", self.stream_backend_combo.currentText())
            self._log(
                f"[카메라] {camera_name} RTSP 스트림 정상: "
                f"{result.get('width')}x{result.get('height')}, {pts_text}, "
                f"firstFrame={float(result.get('elapsed_s', 0.0)):.2f}s, backend={backend}, decoder={decoder}"
            )
            statuses.append(f"{label}=frame-ok")
            next_status[label] = "frame-ok"
        self.camera_connection_label.setText(f"camera RTSP stream: {', '.join(statuses)}")
        self.last_rtsp_check_status = next_status
        self._refresh_operation_header()
        if self._rtsp_reactivation_side is not None:
            camera = self._rtsp_reactivation_side
            if next_status.get(camera) == "frame-ok":
                self._finish_camera_rtsp_reactivation(True)
            else:
                self._finish_camera_rtsp_reactivation(False, str(next_status.get(camera) or "frame check failed"))
            return
        if self.auto_wiris_connect_pending:
            if set(next_status) == {"left", "right"} and all(
                status == "frame-ok" for status in next_status.values()
            ):
                self._finish_auto_wiris_connect()
            else:
                self._log("[카메라] WIRIS 자동 연결: RTSP 프레임 확인 실패. 제어 주소, Ethernet Mode, 영상 주소를 확인하세요.")
                self._schedule_auto_wiris_connect_retry("RTSP 프레임 확인 실패")

    def _rtsp_stream_check_failed(self, error: str) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and not owner.accepts_legacy_outcome('check'):
            return
        self._log(f"[카메라] RTSP 스트림 확인 실패: {error}")
        self.camera_connection_label.setText(f"camera RTSP stream: check failed ({error})")
        self.last_rtsp_check_status = {"left": "check-failed", "right": "check-failed"}
        if self._rtsp_reactivation_side is not None:
            self._finish_camera_rtsp_reactivation(False, error)
            return
        if self.auto_wiris_connect_pending:
            self._schedule_auto_wiris_connect_retry(f"RTSP 확인 오류: {error}")
        self._refresh_operation_header()

    def _clear_rtsp_check_worker(self) -> None:
        owner = getattr(getattr(self, "state", None), "control_jobs", None)
        if owner is not None and 'check' in owner.jobs:
            return
        self.rtsp_check_thread = None
        self.rtsp_check_worker = None
        self.check_button.setEnabled(True)

    def _sample_hz(self, timestamps: deque[float]) -> float:
        if len(timestamps) < 2:
            return 0.0
        elapsed_s = timestamps[-1] - timestamps[0]
        if elapsed_s <= 0.0:
            return 0.0
        return (len(timestamps) - 1) / elapsed_s

    def _update_camera_connection_status(self) -> None:
        now_s = time.monotonic()
        parts = []
        for camera in ("left", "right"):
            stats = self.camera_stream_stats.get(camera) or {}
            if stats and now_s - float(stats.get("timestamp_s", 0.0)) < 2.0:
                state = "connected"
                parts.append(format_camera_stream_stats(camera, state, stats))
                continue
            timestamps = self.camera_frame_times[camera]
            if not timestamps:
                parts.append(f"{camera_display_name(camera)}=대기")
                continue
            age_s = now_s - timestamps[-1]
            state = "connected" if age_s < 2.0 else "stale"
            state_text = "연결" if state == "connected" else "지연"
            parts.append(f"{camera_display_name(camera)}={state_text} {self._sample_hz(timestamps):.1f} Hz")
        prefix = (
            "BACKPRESSURE/DEGRADED | "
            if bool(getattr(self, "_camera_backpressure_degraded", False))
            else ""
        )
        self.camera_connection_label.setText(
            f"{prefix}카메라: {', '.join(parts)}"
        )
        self._update_runtime_camera_stats_labels(now_s)
        self._refresh_operation_header()

    def _update_runtime_camera_settings_layout_mode(self, width: int | None = None) -> None:
        if not hasattr(self, "runtime_header_layout") or not hasattr(self, "runtime_camera_settings_group"):
            return
        current_width = self.runtime_camera_settings_group.width() if width is None else width
        current_direction = self.runtime_header_layout.direction()
        # Hysteresis prevents flip-flopping right at the breakpoint: toggling
        # direction changes the header's own sizeHint, which changes how much
        # vertical space is left for the camera panes below it (video_row
        # holds nearly all of runtime_layout's stretch). Without a dead zone,
        # each resize pass near the breakpoint could cross back over it,
        # making the camera preview visibly jitter in size.
        if current_direction == QtWidgets.QBoxLayout.LeftToRight:
            should_stack = current_width < RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX - RUNTIME_CAMERA_LAYOUT_HYSTERESIS_PX
            direction = QtWidgets.QBoxLayout.TopToBottom if should_stack else QtWidgets.QBoxLayout.LeftToRight
        else:
            should_unstack = current_width >= RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX + RUNTIME_CAMERA_LAYOUT_HYSTERESIS_PX
            direction = QtWidgets.QBoxLayout.LeftToRight if should_unstack else QtWidgets.QBoxLayout.TopToBottom
        if direction != current_direction:
            self.runtime_header_layout.setDirection(direction)

    def _format_runtime_pair_status(self) -> tuple[str, str]:
        if self.selected_runtime_frame_pair_delta_s is None:
            text = "Pair: 대기 | pair: missing"
            return text, text
        left_metadata = self.selected_runtime_frame_metadata.get("left") or {}
        right_metadata = self.selected_runtime_frame_metadata.get("right") or {}
        def app_time(metadata: dict[str, object]) -> str:
            try:
                return f"{float(metadata['timestamp_s']):.6f}s"
            except (KeyError, TypeError, ValueError):
                return "--"

        def app_ns(metadata: dict[str, object]) -> int | None:
            try:
                value = metadata.get("app_receive_monotonic_ns")
                if value is not None:
                    return int(value)
                return int(round(float(metadata["timestamp_s"]) * 1_000_000_000.0))
            except (KeyError, TypeError, ValueError):
                return None

        left_pair = app_time(left_metadata)
        right_pair = app_time(right_metadata)
        selected_delta_ms = self.selected_runtime_frame_pair_delta_s * 1000.0
        left_app_ns = app_ns(left_metadata)
        right_app_ns = app_ns(right_metadata)
        app_delta_ms = (
            None
            if left_app_ns is None or right_app_ns is None
            else abs(right_app_ns - left_app_ns) * 1.0e-6
        )
        requested = str(
            getattr(self, "selected_runtime_frame_pair_requested_basis", "")
            or getattr(self, "selected_runtime_frame_pair_basis", "")
            or "app_receive_monotonic"
        )
        effective = str(
            getattr(self, "selected_runtime_frame_pair_basis", "")
            or "app_receive_monotonic"
        )
        requested_text = "RTCP/NTP" if requested == "camera_rtcp" else "APP RECEIVE"
        effective_text = "RTCP/NTP" if effective == "camera_rtcp" else "APP RECEIVE"
        parts = [
            f"Pair: requested={requested_text}",
            f"effective={effective_text}",
            f"selected Δ {selected_delta_ms:.1f} ms",
            f"app L {left_pair}",
            f"R {right_pair}",
            "app Δ --" if app_delta_ms is None else f"app Δ {app_delta_ms:.1f} ms",
        ]
        fallback_reason = str(
            getattr(self, "selected_runtime_frame_pair_fallback_reason", "") or ""
        )
        if fallback_reason:
            parts.append(
                f"fallback={fallback_reason}"
            )
        if self.selected_runtime_frame_pair_camera_rtc_delta_s is not None:
            ntp_role = "selected" if effective == "camera_rtcp" else "record-only"
            parts.append(
                "NTP Δ "
                f"{self.selected_runtime_frame_pair_camera_rtc_delta_s * 1000.0:.1f} ms "
                f"({ntp_role})"
            )
        else:
            parts.append("NTP Δ --")
        text = " | ".join(parts)
        tooltip = (
            f"{text}\n"
            f"left pair 전체 시각: {format_runtime_camera_timestamp(left_metadata)}\n"
            f"right pair 전체 시각: {format_runtime_camera_timestamp(right_metadata)}"
        )
        return text, tooltip

    def _format_runtime_camera_summary(self) -> tuple[str, str]:
        def compact_camera_status(camera: str, label: QtWidgets.QLabel) -> str:
            text = label.text()
            prefix = "L" if camera == "left" else "R"
            if "대기" in text:
                state = "대기"
            elif "지연" in text:
                state = "지연"
            else:
                state = "연결"
            stats = self.camera_stream_stats.get(camera) or {}
            resolution = camera_resolution_text(stats).removeprefix(" @ ") or "--"
            decoded_hz = float(stats.get("decoded_hz", 0.0)) if stats else 0.0
            live_metadata = self.camera_frame_metadata.get(camera) or {}
            ntp_text = format_runtime_camera_ntp_compact(live_metadata)
            text = f"{prefix} {state:<2} {resolution:>9} {decoded_hz:4.1f}Hz NTP:{ntp_text}"
            if stats.get("enable_detection"):
                inferred_hz = float(stats.get("inferred_hz", 0.0))
                text += f" Y:{inferred_hz:4.1f}Hz"
            return text

        left_text = compact_camera_status("left", self.left_runtime_stats_label)
        right_text = compact_camera_status("right", self.right_runtime_stats_label)
        pair_text, pair_tooltip = self._format_runtime_pair_status()
        if self.selected_runtime_frame_pair_delta_s is None:
            pair_summary = "Pair 대기            "
        else:
            basis = (
                "RTCP/NTP"
                if self.selected_runtime_frame_pair_basis == "camera_rtcp"
                else "APP RECEIVE"
            )
            delta_ms = self.selected_runtime_frame_pair_delta_s * 1000.0
            pair_summary = f"Pair {basis:<18} Δ{delta_ms:+7.1f}ms"
        summary = f"{left_text} | {right_text} | {pair_summary}"

        clock_lines = []
        left_metadata = self.camera_frame_metadata.get("left") or {}
        right_metadata = self.camera_frame_metadata.get("right") or {}
        for camera_prefix, metadata in (("L", left_metadata), ("R", right_metadata)):
            ntp_text = format_unix_timestamp_compact(metadata.get("rtcp_frame_ntp_unix_s"))
            pc_utc_text = format_unix_timestamp_compact(metadata.get("rtcp_frame_received_wall_s"))
            delta_text = format_signed_ms_compact(metadata.get("rtcp_frame_receive_delay_s"))
            clock_lines.append(f"{camera_prefix} NTP: {ntp_text} | PC UTC: {pc_utc_text} | Δt(PC-NTP): {delta_text}")
        left_ntp_s = left_metadata.get("rtcp_frame_ntp_unix_s")
        right_ntp_s = right_metadata.get("rtcp_frame_ntp_unix_s")
        if isinstance(left_ntp_s, (int, float)) and isinstance(right_ntp_s, (int, float)):
            ntp_delta_text = format_signed_ms_compact(float(right_ntp_s) - float(left_ntp_s))
        else:
            ntp_delta_text = "--"
        clock_lines.append(f"NTP Δt(R-L): {ntp_delta_text}")

        tooltip = (
            f"{summary}\n"
            f"{self.left_runtime_stats_label.toolTip()}\n"
            f"{self.right_runtime_stats_label.toolTip()}\n"
            f"{pair_tooltip}\n"
            + "\n".join(clock_lines)
        )
        return summary, tooltip

    def _update_runtime_camera_stats_labels(self, now_s: float | None = None) -> None:
        now_s = time.monotonic() if now_s is None else now_s
        for camera, label in (
            ("left", self.left_runtime_stats_label),
            ("right", self.right_runtime_stats_label),
        ):
            stats = self.camera_stream_stats.get(camera) or {}
            timestamps = self.camera_frame_times[camera]
            stats_age_s = now_s - float(stats.get("timestamp_s", 0.0)) if stats else float("inf")
            frame_age_s = now_s - timestamps[-1] if timestamps else float("inf")
            has_recent_stats = stats_age_s < 2.0
            has_recent_frame = frame_age_s < 2.0
            live_metadata = self.camera_frame_metadata.get(camera) or {}
            status_text = (
                f"{format_runtime_camera_stats(camera, stats, self._sample_hz(timestamps), stale=not (has_recent_stats or has_recent_frame))}"
                f" | live: {format_runtime_camera_timestamp_compact(live_metadata)}"
            )
            pending_reader = getattr(
                self,
                "native_frame_persistence_pending_count",
                None,
            )
            evidence_pending = (
                int(pending_reader()) if callable(pending_reader) else 0
            )
            status_text += f" | Evidence W{evidence_pending}"
            tooltip_text = f"{status_text}\nlive 전체 시각: {format_runtime_camera_timestamp(live_metadata)}"
            label.setText(status_text)
            label.setToolTip(tooltip_text)
        if hasattr(self, "runtime_pair_status_label"):
            pair_text, pair_tooltip = self._format_runtime_pair_status()
            self.runtime_pair_status_label.setText(pair_text)
            self.runtime_pair_status_label.setToolTip(pair_tooltip)
        if hasattr(self, "runtime_camera_summary_label"):
            summary_text, summary_tooltip = self._format_runtime_camera_summary()
            self.runtime_camera_summary_label.setText(summary_text)
            self.runtime_camera_summary_label.setToolTip(summary_tooltip)

    def _update_camera_stream_stats(self, camera: str, payload: object) -> None:
        stats = dict(payload or {})
        stats["timestamp_s"] = time.monotonic()
        self.camera_stream_stats[camera] = stats
        degraded = any(
            int(stats.get(key, 0) or 0) > 0
            for key in (
                "native_ring_dropped",
                "metadata_dropped",
                "control_dropped",
            )
        ) or float(stats.get("gui_preview_age_s", 0.0) or 0.0) > 0.5
        self._camera_backpressure_degraded = bool(
            degraded
            or getattr(self, "_camera_backpressure_degraded", False)
        )
        self._update_camera_connection_status()


    def _log_state(self, camera: str, state: str) -> None:
        if state.startswith("DECODER_FALLBACK_ACTIVE:"):
            self._handle_runtime_decoder_fallback_active(camera, state)
            return
        if state.startswith("DECODER_FALLBACK_REQUIRED:"):
            self._handle_runtime_decoder_fallback(camera, state)
            return
        start_handler = getattr(self, "_camera_stream_start_status", None)
        if callable(start_handler):
            start_handler(camera, state)
        if CameraFeature._decoder_state_clears_active(state):
            self._set_active_stream_decoder(camera, None)
        now_s = time.monotonic()
        previous_state, previous_s = self._last_logged_camera_states.get(
            camera,
            ("", float("-inf")),
        )
        if state == previous_state and now_s - previous_s < 30.0:
            return
        self._last_logged_camera_states[camera] = (state, now_s)
        self._log(f"[카메라] {self._camera_name_ko(camera)} 스트림 상태: {state}")

    @staticmethod
    def _decoder_state_clears_active(state: str) -> bool:
        text = str(state)
        return text.startswith(
            (
                "GStreamer decoder candidate:",
                "스트림 프로세스 오류:",
                "스트림 프로세스 종료:",
                "스트림 프로세스 영구 종료:",
                "스트림 프로세스 크래시 복구 중",
                "스트림 프로세스 크래시 복구 한도 초과:",
                "GStreamer 오류:",
                "GStreamer pipeline 오류:",
                "RTSP/source 오류:",
                "MP4 mux/filesink 오류:",
                "NVIDIA 디코더 오류:",
                "디코더 오류:",
                "스트림 종료",
                "중지됨",
            )
        )

    def _handle_runtime_decoder_fallback_active(
        self,
        camera: str,
        state: str,
    ) -> None:
        requested = str(self.decoder_combo.currentData() or DECODER_AUTO)
        reason = state.removeprefix("DECODER_FALLBACK_ACTIVE:").strip()
        _clear_side_active_decoder(self, camera)
        if requested != DECODER_AUTO:
            self.decoder_fallback_reasons[camera] = reason
            self._log(
                f"[카메라] {self._camera_name_ko(camera)} decoder fallback 상태를 "
                f"무시했습니다: requested={requested}; {reason}"
            )
            return
        overrides = _ensure_side_decoder_overrides(self)
        overrides[camera] = DECODER_SOFTWARE
        self.decoder_fallback_reasons[camera] = reason
        self._log(
            f"[카메라] {self._camera_name_ko(camera)} Auto decoder fallback: "
            f"{reason}. 반대쪽 decoder/worker는 유지하며 이 side의 "
            "current-generation avdec_h264 frame을 대기합니다."
        )

    def _handle_runtime_decoder_fallback(self, camera: str, state: str) -> None:
        requested = str(self.decoder_combo.currentData() or DECODER_AUTO)
        reason = state.removeprefix("DECODER_FALLBACK_REQUIRED:").strip()
        _clear_side_active_decoder(self, camera)
        if requested == DECODER_AUTO:
            overrides = _ensure_side_decoder_overrides(self)
            overrides[camera] = DECODER_SOFTWARE
            self.decoder_fallback_reasons[camera] = reason
            policy = "이 side의 다음 Start는 avdec_h264"
        else:
            self.decoder_fallback_reasons[camera] = reason
            policy = "explicit decoder는 fail-closed"
        start_handler = getattr(self, "_camera_stream_start_status", None)
        if callable(start_handler):
            start_handler(camera, state)
        if requested != DECODER_AUTO:
            self._log(
                f"[카메라] NVIDIA mode decoder failure: {reason}; software fallback은 금지됩니다. "
                "Auto 또는 Software로 변경하세요. 반대쪽 worker는 유지합니다."
            )
            return
        self._log(
            f"[카메라] {self._camera_name_ko(camera)} nvh264dec 비정상: "
            f"{reason}; 해당 side MP4는 종료됐고 {policy}. "
            "반대쪽 decoder/recording worker는 재시작하지 않습니다."
        )

    def _handle_decoder_mode_changed(self, _index: int) -> None:
        self._runtime_decoder_override = None
        _ensure_side_decoder_overrides(self).update(left=None, right=None)
        self.decoder_fallback_reasons.update(left="", right="")
        requested = str(self.decoder_combo.currentData() or DECODER_AUTO)
        self.decoder_selection_status = {
            "requested": requested,
            "effective": None,
            "candidates": (),
            "allowed": None,
            "degraded": False,
            "reason": "mode changed; applies on next stream start",
            "nvidia_available": self.decoder_selection_status.get("nvidia_available"),
        }

    def _camera_name_ko(self, camera: str) -> str:
        return camera_display_name(camera)

    def _extract_active_decoder_from_state(self, state: str) -> str | None:
        prefix = "GStreamer 디코더: "
        if state.startswith(prefix):
            decoder = state.removeprefix(prefix).strip()
            return decoder or None
        return None

    def _set_active_stream_decoder(self, camera: str, decoder: str | None) -> None:
        if camera not in self.active_stream_decoders:
            return
        normalized = (decoder or "").strip() or None
        if self.active_stream_decoders.get(camera) == normalized:
            return
        self.active_stream_decoders[camera] = normalized
        if normalized == NVIDIA_H264_DECODER:
            _ensure_side_decoder_overrides(self)[camera] = None
            self.decoder_fallback_reasons[camera] = ""
        elif normalized == SOFTWARE_H264_DECODER and str(self.decoder_combo.currentData()) == DECODER_SOFTWARE:
            self.decoder_fallback_reasons[camera] = ""

    def build_connection_controls(self, camera_form):
        self.build_legacy_pixhawk_connection_controls()
        pixhawk_assignments = self.legacy_pixhawk_network_assignment_snapshot()
        self.views.connection = CameraConnectionControls(
            CameraConnectionSettings(
                camera_host_address=self.settings.camera_host_address,
                interface_name=self.settings.interface_name,
                left_control_host=self.settings.left_control_host,
                left_control_port=self.settings.left_control_port,
                left_rtsp_url=self.settings.left_rtsp_url,
                left_stip_ip=self.settings.left_stip_ip,
                right_control_host=self.settings.right_control_host,
                right_control_port=self.settings.right_control_port,
                right_rtsp_url=self.settings.right_rtsp_url,
                right_stip_ip=self.settings.right_stip_ip,
                rtsp_decoder=self.settings.rtsp_decoder,
                rtsp_latency_ms=self.settings.rtsp_latency_ms,
                rtsp_transport=self.settings.rtsp_transport,
                stip_reboot=self.settings.stip_reboot,
                stip_subnet_mask=self.settings.stip_subnet_mask,
                stream_backend=self.settings.stream_backend,
                wiris_control_port=self.settings.wiris_control_port,
                wiris_control_timeout_s=self.settings.wiris_control_timeout_s,
            ),
            CameraConnectionActions(
                _handle_decoder_mode_changed=self._handle_decoder_mode_changed,
                apply_control_settings=self.apply_control_settings,
                apply_pc_camera_ip=self.apply_pc_camera_ip,
                apply_rtsp_url=self.apply_rtsp_url,
                apply_wiris_stip=self.apply_wiris_stip,
                auto_connect_wiris=self.auto_connect_wiris,
                check_cameras=self.check_cameras,
                check_wiris_control=self.check_wiris_control,
                detect_camera_control=self.detect_camera_control,
                detect_camera_rtsp=self.detect_camera_rtsp,
                discover_pc_camera_ip=self.discover_pc_camera_ip,
                enable_wiris_rtsp=self.enable_wiris_rtsp,
                swap_cameras=self.swap_cameras,
                toggle_camera_connection=self.toggle_camera_connection,
            ),
            rtsp_host_text=self._rtsp_url_host_text, build_toggle=self._build_toggle_button,
            set_combo=self._set_combo_data, parent=self.widget,
        )
        self.weights_edit = QtWidgets.QLineEdit(self.settings.yolo_weights_path or DEFAULT_YOLO_WEIGHTS)
        self.device_edit = QtWidgets.QLineEdit(self._normalized_yolo_device(self.settings.yolo_device))
        self.device_edit.hide()
        self.device_combo = QtWidgets.QComboBox()
        self.device_combo.addItem("CPU", YOLO_CPU_DEVICE)
        self.device_combo.addItem("CUDA", YOLO_CUDA_DEVICE)
        self.device_combo.setAccessibleName("추론 장치")
        self._set_yolo_device_combo(self._normalized_yolo_device(self.settings.yolo_device))
        self.device_combo.currentIndexChanged.connect(self._refresh_yolo_device_toggle)
        self.input_size_spin = QtWidgets.QSpinBox()
        self.input_size_spin.setRange(160, 1920)
        self.input_size_spin.setSingleStep(32)
        self.input_size_spin.setValue(self.settings.yolo_input_size)
        self.conf_spin = QtWidgets.QDoubleSpinBox()
        self.conf_spin.setRange(0.01, 1.0)
        self.conf_spin.setSingleStep(0.05)
        self.conf_spin.setValue(self.settings.yolo_conf_threshold)
        self.iou_spin = QtWidgets.QDoubleSpinBox()
        self.iou_spin.setRange(0.01, 1.0)
        self.iou_spin.setSingleStep(0.05)
        self.iou_spin.setValue(self.settings.yolo_iou_threshold)
        self.yolo_gate_class_checkbox = self._build_toggle_button(
            "대상 클래스 제한",
            self.settings.yolo_gate_class_enabled,
        )
        self.yolo_gate_class_edit = QtWidgets.QLineEdit(self.settings.yolo_gate_class_text)
        self.yolo_gate_class_edit.setPlaceholderText("예: 0 또는 person")
        self.yolo_gate_matching_checkbox = self._build_toggle_button(
            "시간 기준",
            self.settings.yolo_gate_matching_enabled,
        )
        self.yolo_gate_time_delta_checkbox = self._build_toggle_button(
            "최대 Δt",
            self.settings.yolo_gate_time_delta_enabled,
        )
        self.yolo_gate_vertical_delta_checkbox = self._build_toggle_button(
            "vertical Δ",
            self.settings.yolo_gate_vertical_delta_enabled,
        )
        self.yolo_gate_vertical_delta_spin = QtWidgets.QDoubleSpinBox()
        self.yolo_gate_vertical_delta_spin.setRange(0.0, 5000.0)
        self.yolo_gate_vertical_delta_spin.setValue(self.settings.yolo_gate_max_vertical_delta_px)
        self.yolo_gate_vertical_delta_spin.setSuffix(" px")
        self.yolo_gate_time_delta_spin = QtWidgets.QDoubleSpinBox()
        self.yolo_gate_time_delta_spin.setRange(0.0, 10.0)
        self.yolo_gate_time_delta_spin.setDecimals(3)
        self.yolo_gate_time_delta_spin.setMaximum(0.100)
        self.yolo_gate_time_delta_spin.setValue(
            min(0.100, self.settings.yolo_gate_max_time_delta_s)
        )
        self.yolo_gate_time_delta_spin.setSuffix(" s")
        self.yolo_gate_time_basis_combo = QtWidgets.QComboBox()
        self.yolo_gate_time_basis_combo.addItem(
            "App receive (default)", "app_receive_monotonic"
        )
        self.yolo_gate_time_basis_combo.addItem(
            "RTCP/NTP preferred · fallback App receive", "camera_rtcp"
        )
        selected_time_basis = normalize_live_distance_time_basis(
            self.settings.yolo_gate_time_basis
        )
        selected_time_basis_index = self.yolo_gate_time_basis_combo.findData(
            selected_time_basis
        )
        self.yolo_gate_time_basis_combo.setCurrentIndex(
            max(0, selected_time_basis_index)
        )
        self.yolo_gate_matching_checkbox.setChecked(True)
        self.yolo_gate_matching_checkbox.setEnabled(False)
        self.yolo_gate_time_delta_checkbox.setChecked(True)
        self.yolo_gate_time_delta_checkbox.setEnabled(False)
        self.yolo_gate_time_basis_combo.setEnabled(True)
        self.yolo_gate_time_basis_combo.setToolTip(
            "App receive는 기본 기준입니다. RTCP/NTP preferred는 Clock과 현재 frame "
            "evidence가 정상이면 RTCP를 사용하고, 문제가 있으면 App receive로 fallback합니다."
        )
        self.preview_width_spin = QtWidgets.QSpinBox()
        self.preview_width_spin.setRange(320, 3840)
        self.preview_width_spin.setSingleStep(160)
        self.preview_width_spin.setValue(self.settings.preview_width)
        self.yolo_event_queue_size_spin = QtWidgets.QSpinBox()
        self.yolo_event_queue_size_spin.setRange(1, 10000)
        self.yolo_event_queue_size_spin.setValue(self.settings.yolo_event_queue_size)
        self.yolo_frame_queue_size_spin = QtWidgets.QSpinBox()
        self.yolo_frame_queue_size_spin.setRange(1, 1000)
        self.yolo_frame_queue_size_spin.setValue(self.settings.yolo_frame_queue_size)
        self.views.candidates = CameraCandidateView(
            self.state.candidates, apply_nic=self.apply_selected_camera_candidate_nic,
            apply_side=self.apply_selected_camera_candidate, log=self._log, parent=self.widget,
        )
        self.network_role_assignments: dict[str, dict[str, str]] = {
            "camera": {
                "interface": str(getattr(self.settings, "interface_name", "") or ""),
                "stable_id": str(
                    getattr(self.settings, "camera_network_stable_id", "") or ""
                ),
            },
            "p1": {
                "interface": pixhawk_assignments["left"]["interface"],
                "stable_id": str(
                    getattr(self.settings, "left_mavlink_interface_stable_id", "") or ""
                ),
            },
            "p2": {
                "interface": pixhawk_assignments["right"]["interface"],
                "stable_id": str(
                    getattr(self.settings, "right_mavlink_interface_stable_id", "") or ""
                ),
            },
        }
        self.network_role_owned_entries: dict[str, dict[str, list[str]]] = {
            str(key): {
                "addresses": list((value or {}).get("addresses") or ()),
                "routes": list((value or {}).get("routes") or ()),
            }
            for key, value in (
                getattr(self.settings, "network_role_owned_entries", {}) or {}
            ).items()
            if isinstance(value, dict)
        }
        self.network_role_operation: dict[str, object] | None = None
        self.network_role_status: dict[str, object] = {
            "state": "IDLE",
            "detail": "No network role operation",
        }
        self.network_role_apply_thread: QtCore.QThread | None = None
        self.network_role_apply_worker: QtCore.QObject | None = None
        self.browse_weights_button = QtWidgets.QPushButton("파일 선택")
        self.browse_weights_button.clicked.connect(self.browse_yolo_weights)
        self.start_button = QtWidgets.QPushButton("YOLO 측정 시작")
        self.start_button.clicked.connect(self.toggle_yolo_detection)
        self.detect_control_button.setText("Scan Control Port")
        self.right_detect_control_button.setText("Scan Control Port")
        self.detect_rtsp_button.setText("Scan Video URL")
        self.right_detect_rtsp_button.setText("Scan Video URL")
        self.auto_wiris_connect_button.setText("Auto-Connect Cameras")
        self.detect_pc_ip_button.setText("Discover PC IP")
        self.apply_pc_ip_button.setText("Apply PC IP")
        self.apply_candidate_nic_button.setText("Apply Selected NIC")
        self.apply_candidate_left_button.setText("Apply to Left")
        self.apply_candidate_right_button.setText("Apply to Right")
        self.clear_camera_candidate_button.setText("Clear")
        self.left_control_apply_button.setText("Apply Control Port")
        self.right_control_apply_button.setText("Apply Control Port")
        self.left_rtsp_apply_button.setText("Apply Video URL")
        self.right_rtsp_apply_button.setText("Apply Video URL")
        self.swap_camera_button.setText("Swap Camera Sides")
        self.check_button.setText("Verify Video Stream")
        self.check_control_button.setText("Verify Camera Control")
        self.enable_rtsp_button.setText("Enable Video Stream")
        self.camera_connect_button.setText("Connect Camera Preview")
        self.start_button.setText("YOLO 측정 시작")
        self.left_stip_button.setText("STIP 적용")
        self.right_stip_button.setText("STIP 적용")
        self.browse_weights_button.setText("파일 선택")
        self._refresh_yolo_device_toggle()

        for widget, width in (
            (self.interface_edit, NETWORK_FIELD_MIN_WIDTHS["interface"]),
            (self.host_address_edit, NETWORK_FIELD_MIN_WIDTHS["host"]),
            (self.zoom_port_spin, NETWORK_FIELD_MIN_WIDTHS["port"]),
            (self.left_control_port_spin, NETWORK_FIELD_MIN_WIDTHS["port"]),
            (self.right_control_port_spin, NETWORK_FIELD_MIN_WIDTHS["port"]),
            (self.zoom_timeout_spin, NETWORK_FIELD_MIN_WIDTHS["timeout"]),
            (self.stip_mask_edit, NETWORK_FIELD_MIN_WIDTHS["mask"]),
        ):
            widget.setProperty("compactNetworkField", True)
            widget.setMinimumWidth(width)
            widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        for label_text, widget, label_column, field_column in (
            ("인터페이스", self.interface_edit, 0, 1),
            ("PC IP", self.host_address_edit, 2, 3),
            ("스캔 기본 포트", self.zoom_port_spin, 4, 5),
            ("타임아웃", self.zoom_timeout_spin, 6, 7),
        ):
            label = QtWidgets.QLabel(label_text)
            label.setProperty("compactNetworkLabel", True)
            camera_form.addWidget(label, 0, label_column)
            camera_form.addWidget(widget, 0, field_column)
            camera_form.setColumnStretch(field_column, 1)

    def build_connection_layout(self, camera_form, camera_group, camera_layout, camera_layout_outer, camera_network_group, pixhawk_connection_layout, pixhawk_form, pixhawk_group, yolo_form, yolo_group, yolo_layout_outer):
        self.views.connection.build_layout(
            camera_form, camera_network_group, camera_layout,
            candidate_group=self.views.candidates.build_group(), sequence_label=self.connection_sequence_label,
        )

        weights_row = QtWidgets.QHBoxLayout()
        weights_row.addWidget(self.weights_edit, 1)
        weights_row.addWidget(self.browse_weights_button)
        yolo_form.addWidget(QtWidgets.QLabel("가중치"), 0, 0)
        yolo_form.addLayout(weights_row, 0, 1, 1, 5)
        yolo_form.addWidget(QtWidgets.QLabel("추론 장치"), 1, 0)
        yolo_form.addWidget(self.device_combo, 1, 1, 1, 5)
        yolo_advanced_group = QtWidgets.QGroupBox("고급 스트림 / YOLO 설정")
        yolo_advanced_group.setObjectName("yoloAdvancedSettingsGroup")
        yolo_advanced_layout = QtWidgets.QVBoxLayout(yolo_advanced_group)
        yolo_param_grid = QtWidgets.QGridLayout()
        yolo_param_grid.setHorizontalSpacing(8)
        yolo_param_grid.setVerticalSpacing(6)
        for index, (label_text, widget) in enumerate(
            (
                ("입력 크기(px)", self.input_size_spin),
                ("신뢰도 임계값", self.conf_spin),
                ("IoU 임계값", self.iou_spin),
                ("미리보기 폭(px)", self.preview_width_spin),
                ("RTP 지연(ms)", self.rtsp_latency_spin),
                ("Event 큐", self.yolo_event_queue_size_spin),
                ("Preview 큐", self.yolo_frame_queue_size_spin),
                ("디코더", self.decoder_combo),
                ("수신 백엔드", self.stream_backend_combo),
                ("RTSP 전송", self.rtsp_transport_combo),
            )
        ):
            row = index // 3
            label_column = (index % 3) * 2
            field_column = label_column + 1
            label = QtWidgets.QLabel(label_text)
            label.setBuddy(widget)
            label.setProperty("compactAdvancedLabel", True)
            widget.setAccessibleName(label_text)
            yolo_param_grid.addWidget(label, row, label_column)
            yolo_param_grid.addWidget(widget, row, field_column)
            yolo_param_grid.setColumnStretch(field_column, 1)
        yolo_advanced_layout.addLayout(yolo_param_grid)
        yolo_form.addWidget(yolo_advanced_group, 2, 0, 1, 6)

        yolo_gate_group = QtWidgets.QGroupBox("측정 Gate")
        yolo_gate_layout = QtWidgets.QGridLayout(yolo_gate_group)
        yolo_gate_layout.setHorizontalSpacing(8)
        yolo_gate_layout.setVerticalSpacing(6)
        yolo_gate_layout.setColumnStretch(1, 1)
        yolo_gate_layout.setColumnStretch(3, 1)
        yolo_gate_layout.addWidget(self.yolo_gate_class_checkbox, 0, 0)
        yolo_gate_layout.addWidget(self.yolo_gate_class_edit, 0, 1, 1, 3)
        yolo_gate_layout.addWidget(self.yolo_gate_matching_checkbox, 1, 0)
        yolo_gate_layout.addWidget(self.yolo_gate_time_basis_combo, 1, 1)
        yolo_gate_layout.addWidget(self.yolo_gate_time_delta_checkbox, 1, 2)
        yolo_gate_layout.addWidget(self.yolo_gate_time_delta_spin, 1, 3)
        yolo_gate_layout.addWidget(self.yolo_gate_vertical_delta_checkbox, 1, 4)
        yolo_gate_layout.addWidget(self.yolo_gate_vertical_delta_spin, 1, 5)
        self.yolo_gate_status_label = QtWidgets.QLabel("Gate OFF: 기존 YOLO 후보 선택/매칭을 사용합니다.")
        self.yolo_gate_status_label.setObjectName("statusStrip")
        self.yolo_gate_status_label.setWordWrap(True)
        yolo_gate_layout.addWidget(self.yolo_gate_status_label, 2, 0, 1, 7)
        yolo_form.addWidget(yolo_gate_group, 3, 0, 1, 6)
        for checkbox in (
            self.yolo_gate_class_checkbox,
            self.yolo_gate_matching_checkbox,
            self.yolo_gate_time_delta_checkbox,
            self.yolo_gate_vertical_delta_checkbox,
        ):
            checkbox.toggled.connect(lambda _checked=False: self._refresh_yolo_gate_controls())
            checkbox.toggled.connect(lambda _checked=False: self._refresh_distance())
        self.yolo_gate_vertical_delta_spin.valueChanged.connect(lambda _value: self._refresh_distance())
        self.yolo_gate_time_delta_spin.valueChanged.connect(lambda _value: self._refresh_distance())
        self.yolo_gate_time_basis_combo.currentIndexChanged.connect(lambda _index: self._refresh_distance())
        self._refresh_yolo_gate_controls()

        self.build_legacy_pixhawk_connection_layout(pixhawk_connection_layout, pixhawk_form, pixhawk_group)

        self.views.connection.build_container(camera_group, camera_layout_outer)
        self.camera_yolo_log = QtWidgets.QPlainTextEdit()
        self.camera_yolo_log.setReadOnly(True)
        self.camera_yolo_log.setMaximumBlockCount(600)
        yolo_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        yolo_splitter.addWidget(yolo_group)
        yolo_splitter.addWidget(self.camera_yolo_log)
        yolo_splitter.setStretchFactor(0, 2)
        yolo_splitter.setStretchFactor(1, 1)
        yolo_splitter.setChildrenCollapsible(False)
        yolo_splitter.setSizes([360, 220])
        self.yolo_log_splitter = yolo_splitter
        yolo_layout_outer.addWidget(yolo_splitter, 1)

    def build_camera_commands(self, wiris_command_layout):
        self.views.commands = CameraCommandView(
            self.settings.calibration_zoom_level,
            CameraCommandActions(
                send_zoom=self.send_zoom,
                selected_zoom_target=self.selected_zoom_target,
                _refresh_zoom_intrinsics_tab=self._refresh_zoom_intrinsics_tab,
                _apply_selected_wiris_command=self._apply_selected_wiris_command,
                send_selected_wiris_command=self.send_selected_wiris_command,
                _configure_collapsible_group=self._configure_collapsible_group,
            ), parent=self.widget,
        )
        self.views.commands.install(wiris_command_layout)
        self._apply_selected_wiris_command()
        return self.views.commands.zoom_group,

    def build_live_camera(self, runtime_layout):
        video_row = QtWidgets.QHBoxLayout()
        self.left_pane = CameraPane("left")
        self.right_pane = CameraPane("right")
        self.left_pane.detection_changed.connect(self._update_detection)
        self.right_pane.detection_changed.connect(self._update_detection)
        self.left_pane.frame_changed.connect(self._camera_stream_frame_ready)
        self.right_pane.frame_changed.connect(self._camera_stream_frame_ready)
        self.left_pane.frame_changed.connect(self._update_calibration_preview)
        self.right_pane.frame_changed.connect(self._update_calibration_preview)
        self.left_pane.stats_changed.connect(self._update_camera_stream_stats)
        self.right_pane.stats_changed.connect(self._update_camera_stream_stats)
        self.left_pane.state_changed.connect(self._log_state)
        self.right_pane.state_changed.connect(self._log_state)
        video_row.addWidget(self.left_pane, 1)
        video_row.addWidget(self.right_pane, 1)
        runtime_controls = QtWidgets.QGridLayout()
        runtime_controls.setHorizontalSpacing(6)
        runtime_controls.setVerticalSpacing(6)
        runtime_controls.addWidget(self.start_button, 0, 0)
        self.record_button = QtWidgets.QPushButton("수동 기록 시작")
        self.record_button.clicked.connect(self.toggle_session_recording)
        self.clear_history_button = QtWidgets.QPushButton("그래프/기록 초기화")
        self.clear_history_button.clicked.connect(self.clear_distance_history)
        self.record_button.setText("수동 기록 시작")
        self.clear_history_button.setText("그래프/기록 초기화")
        runtime_action_buttons = (
            self.start_button,
            self.record_button,
            self.clear_history_button,
        )
        runtime_action_width = max(button.sizeHint().width() for button in runtime_action_buttons)
        for button in runtime_action_buttons:
            button.setFixedWidth(runtime_action_width)
        runtime_controls.addWidget(self.record_button, 0, 1)
        runtime_controls.addWidget(self.clear_history_button, 0, 2)
        for column in range(3):
            runtime_controls.setColumnStretch(column, 1)
        self.runtime_camera_settings_group = QtWidgets.QGroupBox("실시간 카메라 / NTP / Pair 상태")
        self.runtime_camera_settings_group.setObjectName("runtimeCameraSettingsGroup")
        runtime_camera_settings_layout = QtWidgets.QVBoxLayout(self.runtime_camera_settings_group)
        runtime_camera_settings_layout.setSpacing(4)
        self.runtime_header_layout = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.LeftToRight)
        self.runtime_header_layout.setContentsMargins(0, 0, 0, 0)
        self.runtime_header_layout.setSpacing(8)
        self.runtime_header_layout.addLayout(runtime_controls, 2)
        self.runtime_camera_status_group = QtWidgets.QGroupBox()
        self.runtime_camera_status_group.setObjectName("runtimeCameraStatusGroup")
        self.runtime_camera_status_group.setFixedHeight(RUNTIME_CAMERA_STATUS_GROUP_HEIGHT_PX)
        self.left_runtime_stats_label = QtWidgets.QLabel(
            "왼쪽: -- | -- | 대기 중 | live: missing", self.widget
        )
        self.right_runtime_stats_label = QtWidgets.QLabel(
            "오른쪽: -- | -- | 대기 중 | live: missing", self.widget
        )
        self.runtime_pair_status_label = QtWidgets.QLabel("Pair: 대기 | pair: missing", self.widget)
        for stats_label in (self.left_runtime_stats_label, self.right_runtime_stats_label, self.runtime_pair_status_label):
            stats_label.setMinimumWidth(320)
            stats_label.setFixedHeight(RUNTIME_CAMERA_STATUS_LABEL_HEIGHT_PX)
            stats_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            stats_label.setAlignment(QtCore.Qt.AlignCenter)
            stats_label.setObjectName("statChip")
            stats_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            stats_label.setWordWrap(True)
            stats_label.hide()
        self.runtime_camera_summary_label = QtWidgets.QLabel("카메라: 좌 NTP -- | 우 NTP -- | Pair 대기")
        self.runtime_camera_summary_label.setObjectName("runtimeCameraSummary")
        self.runtime_camera_summary_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.runtime_camera_summary_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.runtime_camera_summary_label.setWordWrap(False)
        self.runtime_camera_summary_label.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        )
        self.runtime_camera_summary_label.setFixedHeight(RUNTIME_CAMERA_STATUS_LABEL_HEIGHT_PX)
        self.runtime_camera_summary_label.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.runtime_camera_status_layout = QtWidgets.QHBoxLayout(self.runtime_camera_status_group)
        self.runtime_camera_status_layout.setContentsMargins(8, 2, 8, 6)
        self.runtime_camera_status_layout.addWidget(self.runtime_camera_summary_label)
        self.runtime_camera_status_group.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        runtime_camera_settings_layout.addLayout(self.runtime_header_layout)
        self.runtime_header_layout.addWidget(self.runtime_camera_status_group, 3)
        self._configure_collapsible_group(self.runtime_camera_settings_group, checked=True)
        runtime_layout.addWidget(self.runtime_camera_settings_group)
        runtime_layout.addLayout(video_row, 10)
        self.runtime_distance_graph_group = QtWidgets.QGroupBox("3D 위치 / 거리 그래프")
        self.runtime_distance_graph_group.setObjectName("runtimeDistanceGraphGroup")
        distance_plot_layout = QtWidgets.QVBoxLayout(self.runtime_distance_graph_group)
        self.distance_plot = DistanceHistoryPlot()
        self.distance_plot.point_deleted.connect(self._remove_distance_history_point)
        distance_plot_layout.addWidget(self.distance_plot)
        self._configure_collapsible_group(self.runtime_distance_graph_group, checked=True)
        runtime_layout.addWidget(self.runtime_distance_graph_group, 1)
        self._update_runtime_camera_settings_layout_mode(self.width())

