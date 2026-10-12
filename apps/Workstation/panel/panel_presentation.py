"""Presentation feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
from Workstation.panel.panel_view_parts import PanelShellParts
import time
from PyQt5 import (
    QtCore,
    QtGui,
    QtWidgets,
)
from Workstation.panel.panel_support import FORM_BUTTON_MIN_WIDTH, FORM_LABEL_MIN_WIDTH, WIRIS_VISIBLE_ZOOM_FX_TOOLTIP
from Workstation.controllers.run_evidence_controller import report_recoverable_persistence_failure
from Workstation.ui.telemetry_plots import Px4ShellDialog
from Workstation.workstation_core.alignment_managers import (
    BaselineManager,
)
from Workstation.workstation_core.runtime_camera_status import (
    read_nvidia_runtime_identity,
)
from Workstation.workstation_core.theme import (
    ACCENT,
    ACCENT_ACTIVE,
    ACCENT_HOVER,
    APP_BG,
    ERROR,
    FONT_FAMILY,
    INFO_BG,
    INVERSE_BG,
    INVERSE_TEXT_MUTED,
    PANEL_ALT_BG,
    PANEL_ALT_STRONG,
    PANEL_BG,
    PANEL_BORDER,
    PANEL_BORDER_STRONG,
    SUCCESS,
    TEXT_MAIN,
    TEXT_MUTED,
    TEXT_SUBTLE,
)


from Workstation.camera_runtime.readiness import StreamReadinessTracker
from Workstation.camera_runtime.state import PresentationReadinessState

class PresentationFeature(PanelFeature):
    def __init__(self, widget, *, stream_readiness=None):
        readiness = StreamReadinessTracker() if stream_readiness is None else stream_readiness
        super().__init__(widget, state_factory=lambda owner: PresentationReadinessState(readiness))

    feature_name = 'presentation'
    dependency_names = (
        'apply_pc_ip_button',
        'check_button',
        'detect_pc_ip_button',
        'left_persist_eth_button',
        'right_persist_eth_button',
        'left_px4_shell_button',
        'right_px4_shell_button',
        '_handle_tab_changed',
        '_mavlink_open_sides',
        '_measurement_package_state',
        '_measurement_session_hierarchy_enabled',
        '_recording_preview_rows',
        '_recording_session_dir',
        '_refresh_camera_parameters_tab',
        '_session_recording_active',
        '_set_focal_override_enabled',
        '_threadsafe_log_requested',
        '_update_distance_settings_visibility',
        'add_zoom_intrinsics_file_button',
        'alignment_overview_tab_index',
        'alignment_profile_tab_index',
        'alignment_tab_index',
        'alignment_tabs',
        'append_workstation_data',
        'apply_intrinsics_button',
        'apply_zoom_intrinsics_file_button',
        'auto_wiris_connect_button',
        'baseline_alignment_tab_index',
        'baseline_attitude_alignment_tab_index',
        'browse_weights_button',
        'calib_calculation_pairs_spin',
        'calib_cols_spin',
        'calib_dir_edit',
        'calib_min_images_spin',
        'calib_rows_spin',
        'calib_square_spin',
        'calibration_auto_interval_spin',
        'calibration_left_preview',
        'calibration_pair_max_delta_spin',
        'calibration_progress_bar',
        'calibration_right_preview',
        'calibration_tab_index',
        'calibration_tabs',
        'camera_connect_button',
        'camera_connected',
        'camera_connection_label',
        'camera_network_form',
        'camera_parameters_tab_index',
        'camera_yolo_log',
        'capture_calibration_auto_button',
        'capture_calibration_button',
        'check_control_button',
        'checkerboard_calibration_tab_index',
        'checkerboard_validation_tab_index',
        'clock_wiris_time_target_combo',
        'conf_spin',
        'coordinate_alignment_tab_index',
        'custom_watch_add_button',
        'custom_watch_field_edit',
        'custom_watch_mavlink_discover_button',
        'custom_watch_message_edit',
        'custom_watch_name_edit',
        'custom_watch_plot',
        'custom_watch_remove_button',
        'custom_watch_remove_combo',
        'decoder_combo',
        'detect_control_button',
        'detect_pixhawk_host_ip_button',
        'detect_rtsp_button',
        'device_combo',
        'distance_history',
        'distance_label',
        'distance_mode_combo',
        'distance_parameters_label',
        'distance_plot',
        'distance_recording_rows',
        'enable_rtsp_button',
        'findChildren',
        'focal_spin',
        'ftp_log_lines',
        'gnss_base_alignment_tab_index',
        'gnss_monitor',
        'host_address_edit',
        'input_size_spin',
        'interface_edit',
        'intrinsic_parameters_tab_index',
        'intrinsics_source_combo',
        'intrinsics_status_label',
        'iou_spin',
        'layout',
        'left_control_apply_button',
        'left_mavlink_baud_spin',
        'left_mavlink_button',
        'left_mavlink_url_edit',
        'left_pixhawk_eth_ip_edit',
        'left_reboot_pixhawk_button',
        'left_rtsp_apply_button',
        'left_rtsp_ip_edit',
        'left_stip_button',
        'left_stip_ip_edit',
        'left_url_edit',
        'left_zoom_host_edit',
        'manual_focal_checkbox',
        'mavlink_configure_streams_checkbox',
        'mavlink_pixhawk_candidate_table',
        'mavlink_stream_rate_spin',
        'optical_alignment_tab_index',
        'pc_pixhawk_candidate_table',
        'pixhawk_candidate_group',
        'pixhawk_connection_label',
        'pixhawk_connection_tab_index',
        'pixhawk_connection_thread',
        'pixhawk_detail_tab_index',
        'pixhawk_disconnection_thread',
        'pixhawk_host_address_edit',
        'pixhawk_log',
        'pixhawk_mavlink_url_candidate_button',
        'pixhawk_pair_settings_group',
        'pixhawk_settings_group',
        'pixhawk_usb_discover_button',
        'record_button',
        'recording_sample_detail',
        'recording_session_id',
        'recording_sheet_tabs',
        'recording_status_label',
        'remove_zoom_intrinsics_file_button',
        'right_control_apply_button',
        'right_detect_control_button',
        'right_detect_rtsp_button',
        'right_mavlink_baud_spin',
        'right_mavlink_button',
        'right_mavlink_url_edit',
        'right_pixhawk_eth_ip_edit',
        'right_reboot_pixhawk_button',
        'right_rtsp_apply_button',
        'right_rtsp_ip_edit',
        'right_stip_button',
        'right_stip_ip_edit',
        'right_url_edit',
        'right_zoom_host_edit',
        'rm3100_alignment_tab_index',
        'rtk_alignment_detail_label',
        'rtk_alignment_plot',
        'rtsp_latency_spin',
        'rtsp_transport_combo',
        'run_calibration_button',
        'runtime_camera_settings_group',
        'runtime_camera_status_group',
        'runtime_camera_summary_label',
        'runtime_distance_graph_group',
        'save_intrinsics_button',
        'session_summary_label',
        'setFont',
        'setStyleSheet',
        'signal_alignment_tab_index',
        'source_quality_alignment_tab_index',
        'stip_mask_edit',
        'stip_reboot_checkbox',
        'stream_backend_combo',
        'swap_camera_button',
        'swap_pixhawk_button',
        'thread',
        'triangulation_quality_tab_index',
        'validation_alignment_tab_index',
        'weights_edit',
        'wiris_command_combo',
        'wiris_command_edit',
        'wiris_command_log',
        'wiris_command_send_button',
        'wiris_command_target_combo',
        'wiris_command_terminator_combo',
        'wiris_log',
        'wiris_set_pc_time_button',
        'yaw_alignment_tab_index',
        'yolo_enabled',
        'yolo_event_queue_size_spin',
        'yolo_frame_queue_size_spin',
        'yolo_gate_class_checkbox',
        'yolo_gate_class_edit',
        'yolo_gate_matching_checkbox',
        'yolo_gate_time_basis_combo',
        'yolo_gate_time_delta_checkbox',
        'yolo_gate_time_delta_spin',
        'yolo_gate_vertical_delta_checkbox',
        'yolo_gate_vertical_delta_spin',
        'zoom_in_button',
        'zoom_intrinsics_compare_list',
        'zoom_intrinsics_tab_index',
        'zoom_level_spin',
        'zoom_out_button',
        'zoom_port_spin',
        'zoom_set_button',
        'zoom_stop_button',
        'zoom_target_combo',
        'zoom_timeout_spin',
    )
    writable_dependencies = frozenset(('alignment_overview_tab_index', 'alignment_profile_tab_index', 'alignment_tab_index', 'alignment_tabs', 'baseline_alignment_tab_index', 'baseline_attitude_alignment_tab_index', 'calibration_tab_index', 'calibration_tabs', 'camera_network_form', 'camera_parameters_tab_index', 'checkerboard_calibration_tab_index', 'checkerboard_validation_tab_index', 'coordinate_alignment_tab_index', 'gnss_base_alignment_tab_index', 'intrinsic_parameters_tab_index', 'optical_alignment_tab_index', 'pixhawk_connection_tab_index', 'pixhawk_detail_tab_index', 'pixhawk_settings_group', 'rm3100_alignment_tab_index', 'signal_alignment_tab_index', 'source_quality_alignment_tab_index', 'triangulation_quality_tab_index', 'validation_alignment_tab_index', 'yaw_alignment_tab_index', 'zoom_intrinsics_tab_index'))

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.px4_shell_dialogs: dict[str, Px4ShellDialog] = {}
        self._video_start_readiness_generation = 0
        self._video_start_readiness: dict[str, object] | None = None
        self._native_frame_stop_retry_scheduled = False
        self._pending_measurement_run_start = False
        self._measurement_auto_finalize_enabled = True
        self.workstation_time_sync_checkpoints: list[dict[str, object]] = []
        self._last_clock_offset_sample_s = 0.0
        self.selected_runtime_frame_metadata = {
            "left": {},
            "right": {},
        }
        self.selected_runtime_frame_pair_delta_s: float | None = None
        self.selected_runtime_frame_pair_basis = ""
        self.selected_runtime_frame_pair_requested_basis = ""
        self.selected_runtime_frame_pair_fallback_reason = ""
        self._runtime_pair_uses_app_fallback = False
        self._coordinated_pairing_result = None
        self.nvidia_runtime_identity = read_nvidia_runtime_identity()
        self.baseline_manager = BaselineManager(self.gnss_monitor)
        self._px4_board_rotation_params: dict[str, dict[str, object]] = {}

    def _build_toggle_button(self, text: str, checked: bool = False) -> QtWidgets.QPushButton:
        button = QtWidgets.QPushButton(text)
        button.setCheckable(True)
        button.setChecked(bool(checked))
        button.setProperty("role", "toggle")
        button.setAccessibleName(text)
        return button

    def _set_button_role(self, button: QtWidgets.QPushButton, role: str) -> None:
        if button.property("role") == role:
            return
        button.setProperty("role", role)
        # Do not synchronously force a QStyle unpolish/polish from a live
        # status callback. Under a saturated desktop accessibility service the
        # native polish path can stop returning, which blocks the Qt event loop
        # before transport completion/timers are serviced. Button text remains
        # the authoritative Connect/Disconnect/Start/Stop cue. Retain the role
        # property for state/semantic inspection without making an immediate
        # stylesheet repolish part of the command-completion critical path.
        button.update()

    def _refresh_session_toggle_buttons(self) -> None:
        if not hasattr(self, "camera_connect_button"):
            return
        self.camera_connect_button.setText(
            "Disconnect Camera Preview" if self.camera_connected else "Connect Camera Preview"
        )
        self._set_button_role(self.camera_connect_button, "danger" if self.camera_connected else "primary")

        self.start_button.setText("YOLO 측정 중지" if self.yolo_enabled else "YOLO 측정 시작")
        self._set_button_role(self.start_button, "danger" if self.yolo_enabled else "primary")

        self.record_button.setText("수동 기록 중지" if self._session_recording_active() else "수동 기록 시작")
        self._set_button_role(self.record_button, "danger" if self._session_recording_active() else "success")

        connected_mavlink_sides = self._mavlink_open_sides()
        mavlink_operation_busy = bool(
            self.pixhawk_connection_thread is not None
            or self.pixhawk_disconnection_thread is not None
        )
        self.left_mavlink_button.setText(
            "Disconnect P1 MAVLink" if "left" in connected_mavlink_sides else "Connect P1 MAVLink"
        )
        self.left_mavlink_button.setEnabled(not mavlink_operation_busy)
        self._set_button_role(self.left_mavlink_button, "danger" if "left" in connected_mavlink_sides else "primary")
        self.right_mavlink_button.setText(
            "Disconnect P2 MAVLink" if "right" in connected_mavlink_sides else "Connect P2 MAVLink"
        )
        self.right_mavlink_button.setEnabled(not mavlink_operation_busy)
        self._set_button_role(self.right_mavlink_button, "danger" if "right" in connected_mavlink_sides else "primary")

    def _install_tooltips(self) -> None:
        tooltips = {
            self.interface_edit: "카메라 네트워크 도달성을 확인할 때 사용할 로컬 네트워크 인터페이스입니다.",
            self.host_address_edit: "WIRIS 카메라망에서 PC가 사용할 IP입니다. 카메라 IP가 아니며 Pixhawk host 주소와도 별개입니다.",
            self.pixhawk_host_address_edit: "Pixhawk MAVLink가 사용할 PC 주소입니다. 예: 10.0.0.10",
            self.left_url_edit: "왼쪽 카메라 스트림의 전체 RTSP URL입니다. 카메라 IP, 포트, 경로를 포함합니다.",
            self.right_url_edit: "오른쪽 카메라 스트림의 전체 RTSP URL입니다. 카메라 IP, 포트, 경로를 포함합니다.",
            self.left_rtsp_ip_edit: "왼쪽 RTSP URL에 적용할 OSD IP Address 값입니다. Telnet 제어 host는 이 버튼으로 바꾸지 않습니다.",
            self.right_rtsp_ip_edit: "오른쪽 RTSP URL에 적용할 OSD IP Address 값입니다. Telnet 제어 host는 이 버튼으로 바꾸지 않습니다.",
            self.left_rtsp_apply_button: "왼쪽 RTSP URL의 host만 OSD IP Address 값으로 바꿉니다. RTSP port는 URL 안의 포트를 유지합니다.",
            self.right_rtsp_apply_button: "오른쪽 RTSP URL의 host만 OSD IP Address 값으로 바꿉니다. RTSP port는 URL 안의 포트를 유지합니다.",
            self.weights_edit: (
                "거리 추정 전 객체 검출에 사용할 YOLO 모델 파일입니다. .pt/.onnx/.engine 등 Ultralytics가 읽을 수 있는 "
                "weight를 지정하며, 모델의 학습 클래스와 입력 전처리도 이 파일에 따라 결정됩니다. "
                "파일을 바꾸면 다음 `YOLO 측정 시작` 때 새 모델을 로드하고, 모델 로드 실패 시 카메라 Preview만 유지합니다."
            ),
            self.browse_weights_button: ".pt, .onnx, .engine 같은 YOLO weight 파일을 선택합니다.",
            self.device_combo: (
                "YOLO 추론을 실행할 장치입니다. `CPU`는 PyTorch `cpu`, `CUDA`는 `cuda:0`을 사용합니다. "
                "CUDA를 선택해도 드라이버/모델 추론이 실패하면 worker가 가능한 경우 CPU로 재시도합니다. "
                "이 값은 RTSP 수신/디코더 장치가 아니라 객체 추론 장치만 결정합니다."
            ),
            self.input_size_spin: (
                "Ultralytics에 전달할 YOLO 추론 입력 크기(px)입니다. 디코드된 원본 프레임을 이 크기로 letterbox/resize한 뒤 "
                "추론하며, GUI Preview 크기는 바꾸지 않습니다. 값을 키우면 작은 객체를 찾는 데 유리할 수 있지만 "
                "GPU 메모리·추론 시간·지연이 증가하고 YOLO Hz가 낮아질 수 있습니다. 값을 줄이면 처리량은 좋아지지만 "
                "작은 객체가 사라질 수 있습니다. worker는 모델 stride에 맞게 내부적으로 크기를 보정할 수 있습니다."
            ),
            self.conf_spin: (
                "검출 후보를 유지할 최소 confidence(0.01~1.0)입니다. YOLO가 계산한 class confidence가 이 값보다 낮은 "
                "박스는 좌/우 매칭 전에 버립니다. 낮추면 놓치는 객체는 줄지만 오검출이 늘고, 높이면 오검출은 줄지만 "
                "희미하거나 작은 객체를 놓칠 수 있습니다. 좌/우 동기화의 Δt 기준이나 NMS IoU와는 별도 값입니다."
            ),
            self.iou_spin: (
                "NMS(non-maximum suppression)에 사용하는 박스 겹침 기준(IoU)입니다. 같은 class의 두 박스가 겹칠 때 "
                "높은 confidence 박스를 남길지 판단합니다. 낮추면 겹친 박스를 더 공격적으로 제거하고, 높이면 겹친 "
                "후보를 더 오래 남깁니다. 이 값은 좌/우 frame의 시간 차이 또는 RTCP/NTP 동기화 판정이 아닙니다."
            ),
            self.preview_width_spin: (
                "Qt GUI Preview에 복사할 최대 너비(px)입니다. 1920으로 두면 현재 1080p WIRIS 프레임을 추가 축소 없이 "
                "표시할 수 있습니다. 이 값은 Preview 복사/렌더링 비용만 바꾸며 YOLO 검출 입력 크기와 다릅니다. "
                "YOLO는 Preview 이미지가 아니라 원본 디코딩 프레임을 입력으로 사용합니다."
            ),
            self.rtsp_latency_spin: (
                "GStreamer `rtpjitterbuffer`가 RTP 패킷을 기다리는 버퍼 시간(ms)입니다. 이것은 YOLO 추론 지연이 아니라 "
                "RTSP 수신 경로의 지터 흡수 설정입니다. 낮추면 최신 프레임 지연은 줄지만 패킷 순서 흔들림/손실에 민감해지고, "
                "높추면 네트워크 끊김에는 강해질 수 있지만 오래된 프레임과 전체 처리 지연이 늘어납니다. "
                "GStreamer backend에서 주로 적용되며 RTCP/NTP 오프셋 추이의 transit delay와 같은 값이 아닙니다."
            ),
            self.yolo_event_queue_size_spin: (
                "YOLO worker에서 GUI로 보내는 detection, measurement frame, stats, status 이벤트 큐의 최대 크기입니다. "
                "작게 두면 이벤트 backlog로 인한 UI 지연을 줄일 수 있지만, 처리 속도가 UI보다 빠를 때 오래된 이벤트가 "
                "버려질 수 있습니다. YOLO 모델의 정확도나 RTSP 디코딩 속도 자체를 바꾸지는 않습니다."
            ),
            self.yolo_frame_queue_size_spin: (
                "RTSP reader에서 YOLO worker로 전달하는 decoded frame 큐 크기입니다. 작을수록 최신 frame을 우선해 실시간 "
                "지연을 낮추지만, worker가 느리면 중간 frame을 버립니다. 크게 하면 burst를 흡수할 수 있지만 오래된 frame이 "
                "추론될 가능성과 지연 누적이 커집니다. 수신 Hz는 reader 기준이고 YOLO Hz는 추론 완료 기준이라 서로 다를 수 있습니다."
            ),
            self.decoder_combo: (
                "RTSP H.264 디코더 선택입니다. `NVIDIA`는 nvh264dec, `Software`는 avdec_h264를 사용하고, `Auto`는 NVIDIA를 "
                "먼저 시도한 뒤 생성 실패 시 소프트웨어로 전환합니다. 디코더 선택은 YOLO 정확도보다 CPU/GPU 사용량, "
                "프레임 수신 안정성, 지연에 영향을 줍니다."
            ),
            self.stream_backend_combo: (
                "RTSP 수신/디코드 백엔드입니다. `GStreamer`는 jitter buffer, decoder, frame drop을 세밀하게 제어하는 기본 경로이고, "
                "`OpenCV/FFmpeg`는 단순한 대체 경로, `OpenCV Auto`는 OpenCV가 사용 가능한 backend를 선택합니다. "
                "`PyAV`는 av 패키지가 있을 때 FFmpeg를 직접 사용합니다. backend를 바꾸면 RTP/RTCP timestamp와 지연 특성이 달라질 수 있습니다."
            ),
            self.rtsp_transport_combo: (
                "RTSP media transport입니다. `UDP`는 지연 누적이 적지만 방화벽/패킷 손실에 영향을 받고, `TCP` interleaving은 "
                "RTSP control channel로 RTP/RTCP를 함께 받아 Sender Report 유실을 줄이는 대신 수신 queue 지연이 누적될 수 있습니다. "
                "카메라 시간 검증에서는 RTCP Sender Report가 필요하므로 TCP를 기본값으로 사용합니다."
            ),
            self.yolo_gate_class_checkbox: (
                "대상 class gate를 켭니다. 켜면 아래 입력한 class id 또는 class 이름과 일치하는 YOLO 검출만 좌/우 매칭 후보로 사용합니다. "
                "예: `0` 또는 `person`. YOLO 모델의 실제 class names와 맞아야 하며, 검출 confidence/NMS 이후에 적용됩니다."
            ),
            self.yolo_gate_class_edit: "대상 class id 또는 class 이름입니다. `0`, `1`, `person`처럼 입력하며 대상 class gate가 켜져 있을 때만 적용됩니다.",
            self.yolo_gate_matching_checkbox: (
                "Workstation 실시간 거리는 Settings의 선택 기준으로 좌/우 후보를 매칭합니다. "
                "RTCP/NTP timing에 문제가 있으면 exact app-receive monotonic으로 fallback합니다."
            ),
            self.yolo_gate_time_basis_combo: (
                "App receive는 기본값입니다. RTCP/NTP preferred는 Clock VERIFIED와 current same-epoch frame evidence가 있을 때만 RTCP를 선택하고, "
                "그 외에는 exact app-receive monotonic으로 fallback합니다. 실행 중 변경은 다음 Measurement부터 적용됩니다."
            ),
            self.yolo_gate_time_delta_checkbox: (
                "좌/우 detection pair의 최대 시간 차이 gate를 켭니다. 아래 최대 Δt보다 timestamp 차이가 큰 후보는 매칭하지 않습니다. "
                "실제 선택된 timing basis의 좌/우 pair 최대 Δt입니다. 100ms보다 크게 설정할 수 없습니다."
            ),
            self.yolo_gate_time_delta_spin: "YOLO 좌/우 pair 최대 Δt(s)입니다. 최대 Δt gate가 켜져 있을 때 이 값보다 오래 떨어진 detection pair를 제외합니다.",
            self.yolo_gate_vertical_delta_checkbox: (
                "좌/우 bbox 중심의 세로 위치 차이 gate를 켭니다. 카메라 정렬이 양호하면 같은 대상의 중심 y 차이가 작아야 하므로, "
                "아래 vertical Δ보다 큰 pair를 제외해 잘못된 좌/우 매칭을 줄입니다."
            ),
            self.yolo_gate_vertical_delta_spin: "좌/우 bbox 중심 y 좌표의 최대 차이(px)입니다. vertical Δ gate가 켜져 있을 때 이 값보다 큰 pair를 제외합니다.",
            self.detect_pc_ip_button: "선택한 인터페이스의 활성 NetworkManager 연결에서 카메라망 IP를 찾아 PC IP 입력칸에 채웁니다.",
            self.apply_pc_ip_button: "선택한 로컬 인터페이스에 PC 카메라망 IP를 적용해 카메라 NIC로 통신하게 합니다.",
            self.auto_wiris_connect_button: "WIRIS Telnet port 전체 스캔, HIWS 확인, Ethernet/RTSP 활성화, 프레임 확인, 프리뷰 연결을 순서대로 시도합니다.",
            self.detect_control_button: "후보 WIRIS IP Address의 TCP 1-65535 포트를 스캔하고 HIWS=OK 응답을 받은 Telnet port를 적용합니다.",
            self.right_detect_control_button: "후보 WIRIS IP Address의 TCP 1-65535 포트를 스캔하고 HIWS=OK 응답을 받은 Telnet port를 적용합니다.",
            self.left_control_apply_button: "왼쪽 OSD IP Address/Telnet port를 저장하고 RTSP URL host와 IP Address 입력을 같은 값으로 맞춥니다.",
            self.right_control_apply_button: "오른쪽 OSD IP Address/Telnet port를 저장하고 RTSP URL host와 IP Address 입력을 같은 값으로 맞춥니다.",
            self.detect_rtsp_button: "로컬 카메라망에서 RTSP port를 찾아 좌/우 RTSP URL을 채우고 프레임 수신을 확인합니다.",
            self.right_detect_rtsp_button: "로컬 카메라망에서 RTSP port를 찾아 좌/우 RTSP URL을 채우고 프레임 수신을 확인합니다.",
            self.check_control_button: "각 WIRIS IP Address가 설정된 Telnet port에서 명령 연결을 받는지 확인합니다.",
            self.enable_rtsp_button: "설정된 WIRIS IP Address/Telnet port에 Ethernet Mode 명령을 보내 RTSP 서버를 엽니다.",
            self.left_stip_ip_edit: "왼쪽 WIRIS에 쓸 새 OSD IP Address입니다. 대상 카메라가 명확할 때만 사용하세요.",
            self.right_stip_ip_edit: "오른쪽 WIRIS에 쓸 새 OSD IP Address입니다. 대상 카메라가 명확할 때만 사용하세요.",
            self.stip_mask_edit: "STIP 이후 STMS로 함께 쓸 OSD IP Mask입니다.",
            self.stip_reboot_checkbox: "STIP/STMS 후 REBT를 보내 새 IP Address로 WIRIS를 재시작합니다.",
            self.left_stip_button: (
                "IP Address / IP Mask 변경 그룹의 Left IP Address를 왼쪽 WIRIS에 씁니다.\n"
                "완료되면 왼쪽 RTSP URL의 host도 같이 갱신됩니다."
            ),
            self.right_stip_button: (
                "IP Address / IP Mask 변경 그룹의 Right IP Address를 오른쪽 WIRIS에 씁니다.\n"
                "완료되면 오른쪽 RTSP URL의 host도 같이 갱신됩니다."
            ),
            self.pixhawk_usb_discover_button: "OS에 보이는 USB Pixhawk 포트와 MAVLink heartbeat/권한 상태를 후보 표에 표시합니다. 자동 할당은 하지 않습니다.",
            self.pixhawk_mavlink_url_candidate_button: "Ethernet MAVLink UDP 기본 URL 후보를 표에 추가합니다. PX4가 해당 UDP 포트로 MAVLink를 보내고 있어야 연결됩니다.",
            self.detect_pixhawk_host_ip_button: "P1/P2 Pixhawk ETH IP 대역에 맞는 이 PC의 IPv4를 찾아 Pixhawk Host IP에 채웁니다.",
            self.swap_camera_button: "좌/우 카메라 RTSP URL과 IP Address/Telnet port를 서로 바꿉니다. 실행 중인 스트림은 새 매핑으로 재시작됩니다.",
            self.swap_pixhawk_button: "P1/P2의 MAVLink URL, USB 제어 포트, ETH IP 매핑을 서로 바꿉니다. 실행 중인 MAVLink 링크는 새 매핑으로 재시작됩니다.",
            self.check_button: "좌/우 RTSP 호스트의 RTSP 포트가 열려 있는지 확인합니다.",
            self.camera_connect_button: "좌/우 RTSP Preview를 연결하거나, 연결 중이면 카메라 Preview와 활성 YOLO 검출을 중지합니다.",
            self.left_mavlink_button: "Pixhawk 1 MAVLink 세션을 연결하거나, 연결 중이면 P1 MAVLink 링크만 중지합니다.",
            self.right_mavlink_button: "Pixhawk 2 MAVLink 세션을 연결하거나, 연결 중이면 P2 MAVLink 링크만 중지합니다.",
            self.left_px4_shell_button: "활성 MAVLink 연결을 통해 Pixhawk 1 PX4 shell 창을 엽니다.",
            self.right_px4_shell_button: "활성 MAVLink 연결을 통해 Pixhawk 2 PX4 shell 창을 엽니다.",
            self.left_persist_eth_button: "왼쪽 Pixhawk eth0/MAVLink UDP 설정을 즉시 적용하고 extras.txt에 저장합니다.",
            self.right_persist_eth_button: "오른쪽 Pixhawk eth0/MAVLink UDP 설정을 즉시 적용하고 extras.txt에 저장합니다.",
            self.left_reboot_pixhawk_button: "활성 MAVLink 연결로 Pixhawk 1에 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN을 보냅니다.",
            self.right_reboot_pixhawk_button: "활성 MAVLink 연결로 Pixhawk 2에 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN을 보냅니다.",
            self.camera_connection_label: "좌/우 카메라 Preview 연결 상태와 측정된 프레임 속도를 표시합니다.",
            self.runtime_camera_settings_group: "실시간 카메라 화면 위의 실행 버튼, 카메라별 live 상태, 좌우 pair 상태를 접거나 펼칩니다.",
            self.runtime_camera_status_group: "좌측/우측 카메라 상태와 좌우 계산 pair 상태를 한 줄로 요약해 보여주는 상태 그룹입니다.",
            self.runtime_camera_summary_label: "좌측/우측 카메라의 현재 수신 상태와 각 카메라의 RTCP Sender Report 기반 NTP 시각, 계산에 선택된 좌우 pair 동기화 상태입니다. 상세 시각은 상태 툴팁에서 확인할 수 있습니다.",
            self.left_runtime_stats_label: (
                "측정 탭 왼쪽 카메라의 최신 해상도/Hz, YOLO Hz, live 시각입니다.\n"
                "수신 Hz: 스트림 reader가 디코딩한 프레임 속도입니다. 측정 모드에서도 YOLO 추론 완료 속도와 분리됩니다.\n"
                "YOLO Hz: 실제 YOLO 추론이 완료된 속도입니다. -- 는 아직 통계가 없고, 0.0 Hz 는 통계는 있으나 최근 추론 완료가 없다는 뜻입니다.\n"
                "CUDA/CPU: 설정값이 아니라 YOLO worker가 마지막 추론에 실제 사용한 장치입니다. CPU fallback은 CUDA 추론 실패 후 CPU로 전환된 상태입니다.\n"
                "실시간/지연: 최근 스트림 통계나 프레임이 2초 안에 갱신됐는지 표시합니다.\n"
                "GNSS-UTC: timestamp basis가 명시적으로 GNSS-UTC일 때만 UTC로 표시합니다.\n"
                "source timestamp: RTCP/NTP가 없을 때만 표시하는 디코더/소스 시각입니다.\n"
                "missing: RTCP/NTP와 source timestamp 모두 없는 상태입니다.\n"
                "live: 최신 수신 프레임의 RTCP Sender Report 기반 RTCP/NTP 시각입니다. WIRIS GDTI와의 UTC 일치는 미검증입니다."
            ),
            self.right_runtime_stats_label: (
                "측정 탭 오른쪽 카메라의 최신 해상도/Hz, YOLO Hz, live 시각입니다.\n"
                "수신 Hz: 스트림 reader가 디코딩한 프레임 속도입니다. 측정 모드에서도 YOLO 추론 완료 속도와 분리됩니다.\n"
                "YOLO Hz: 실제 YOLO 추론이 완료된 속도입니다. -- 는 아직 통계가 없고, 0.0 Hz 는 통계는 있으나 최근 추론 완료가 없다는 뜻입니다.\n"
                "CUDA/CPU: 설정값이 아니라 YOLO worker가 마지막 추론에 실제 사용한 장치입니다. CPU fallback은 CUDA 추론 실패 후 CPU로 전환된 상태입니다.\n"
                "실시간/지연: 최근 스트림 통계나 프레임이 2초 안에 갱신됐는지 표시합니다.\n"
                "GNSS-UTC: timestamp basis가 명시적으로 GNSS-UTC일 때만 UTC로 표시합니다.\n"
                "source timestamp: RTCP/NTP가 없을 때만 표시하는 디코더/소스 시각입니다.\n"
                "missing: RTCP/NTP와 source timestamp 모두 없는 상태입니다.\n"
                "live: 최신 수신 프레임의 RTCP Sender Report 기반 RTCP/NTP 시각입니다. WIRIS GDTI와의 UTC 일치는 미검증입니다."
            ),
            self.runtime_pair_status_label: "좌우 계산/동기화 frame pair의 time basis, pair 시각, 좌우 delta를 표시합니다.",
            self.pixhawk_connection_label: "Pixhawk 자세 구독 상태와 메시지 수신 속도를 표시합니다.",
            self.left_pixhawk_eth_ip_edit: "왼쪽 Pixhawk에 적용하고 저장할 eth0 고정 IP입니다. 예: 10.0.0.2",
            self.right_pixhawk_eth_ip_edit: "오른쪽 Pixhawk에 적용하고 저장할 eth0 고정 IP입니다. 예: 10.0.0.3",
            self.left_mavlink_url_edit: "Pixhawk 1 MAVLink URL입니다. 예: udpin:0.0.0.0:14550, /dev/ttyACM0, tcp:192.168.0.3:5760",
            self.right_mavlink_url_edit: "Pixhawk 2 MAVLink URL입니다. 예: udpin:0.0.0.0:14551, /dev/ttyACM1, tcp:10.41.10.2:5760",
            self.left_mavlink_baud_spin: "Pixhawk 1 MAVLink URL이 시리얼 장치일 때 사용할 baud rate입니다.",
            self.right_mavlink_baud_spin: "Pixhawk 2 MAVLink URL이 시리얼 장치일 때 사용할 baud rate입니다.",
            self.mavlink_configure_streams_checkbox: "MAVLink 연결 후 MAV_SYS_ID와 MAV_2 단일-instance 파라미터를 저장합니다. 수동 mavlink start는 실행하지 않으며 재부팅 후 적용됩니다.",
            self.mavlink_stream_rate_spin: "재부팅 후 MAV_2 Ethernet instance에 적용할 MAV_2_RATE 값입니다.",
            self.custom_watch_name_edit: "사용자 지정 MAVLink 필드 감시의 표시 이름입니다.",
            self.custom_watch_message_edit: "감시할 MAVLink message 이름입니다. 예: ATTITUDE, GPS_RAW_INT",
            self.custom_watch_field_edit: "읽을 메시지 필드 경로입니다. 예: z, q[0], position[2]. 숫자 필드는 그래프로 표시됩니다.",
            self.custom_watch_add_button: "사용자 지정 MAVLink 감시를 추가합니다.",
            self.custom_watch_mavlink_discover_button: "ATTITUDE.roll 같은 MAVLink 메시지/필드를 골라 활성 MAVLink Pixhawk 연결에서 그래프로 표시합니다.",
            self.custom_watch_remove_combo: "제거할 기존 사용자 지정 감시를 선택합니다.",
            self.custom_watch_remove_button: "선택한 사용자 지정 감시를 제거하고 그래프 series를 지웁니다.",
            self.custom_watch_plot: "사용자 지정 숫자 감시값의 이력 그래프입니다.",
            self.rtk_baseline_label: "실시간 baseline 길이, N/E/D 벡터, 샘플 age, GPS/RTK 품질 경고를 표시합니다.",
            self.rtk_alignment_detail_label: "안테나 baseline과 카메라 기준 Right/Down/Forward 성분을 분리해 표시합니다.",
            self.rtk_alignment_plot: "GPS 안테나 baseline과 카메라 기준 baseline 성분을 시각화합니다.",
            self.left_gps_label: "Pixhawk 1의 최신 GPS/RTK 상태입니다.",
            self.right_gps_label: "Pixhawk 2의 최신 GPS/RTK 상태입니다.",
            self.left_zoom_host_edit: "왼쪽 WIRIS의 OSD IP Address입니다. Telnet/SDK 명령과 RTSP URL host의 기준값입니다.",
            self.right_zoom_host_edit: "오른쪽 WIRIS의 OSD IP Address입니다. Telnet/SDK 명령과 RTSP URL host의 기준값입니다.",
            self.zoom_port_spin: "Telnet port 탐색을 시작할 때 먼저 확인하는 스캔 기본 포트입니다. 좌/우 실제 명령은 각 WIRIS의 Telnet port를 사용합니다.",
            self.zoom_timeout_spin: "WIRIS SDK/줌 명령 TCP 요청 timeout입니다. 왼쪽 카메라 SDK 응답 지연을 포함해야 합니다.",
            self.zoom_target_combo: "줌 명령을 보낼 카메라 대상을 선택합니다.",
            self.zoom_in_button: "기본 Zoom In 명령(SZIN)을 보냅니다.",
            self.zoom_out_button: "기본 Zoom Out 명령(SZOT)을 보냅니다.",
            self.zoom_stop_button: "기본 Zoom Stop 명령(SGFA)을 보냅니다.",
            self.zoom_set_button: "선택한 절대 줌 레벨을 기본 명령(SZVN)으로 보냅니다.",
            self.zoom_level_spin: "WIRIS 가시광 줌 레벨입니다.\n\n" + WIRIS_VISIBLE_ZOOM_FX_TOOLTIP,
            self.wiris_command_target_combo: "선택한 SDK 명령을 왼쪽, 오른쪽, 양쪽 WIRIS 중 어디로 보낼지 선택합니다.",
            self.wiris_command_combo: "공식 WIRIS Ethernet Stream SDK 명령 preset입니다.",
            self.wiris_command_edit: "전송할 SDK 명령 텍스트입니다. <IPv4> 같은 placeholder는 전송 전에 바꾸세요.",
            self.wiris_command_terminator_combo: "명령 구분자입니다. 소프트웨어에서는 NULL을, telnet 방식에서는 LINE을 주로 사용합니다.",
            self.wiris_command_send_button: "편집한 SDK 명령을 선택한 WIRIS 대상으로 전송합니다.",
            self.clock_wiris_time_target_combo: "PC UTC를 설정할 WIRIS RTC 대상을 선택합니다.",
            self.wiris_set_pc_time_button: (
                "선택한 WIRIS 대상의 RTC를 PC 현재 UTC로 설정합니다. "
                "양쪽은 왼쪽 STUS의 OK ACK를 확인한 뒤에만 오른쪽을 전송합니다. "
                "카메라별 HIWS 제어 채널 RTT를 측정해 RTT/2와 파싱 여유를 더한 STUS 목표시각을 사용하고, 측정값을 로그에 남깁니다."
            ),
            self.clock_sync_check_button: (
                "최근 스트림 frame의 RTCP/NTP 기반 시각을 PC UTC와 비교합니다. "
                "프레임 수신 age를 보정해 해당 프레임 시점의 PC UTC offset을 계산하며, WIRIS GDTI와의 일치는 별도 검증 대상입니다."
            ),
            self.wiris_command_log: "카메라 SDK 명령 응답 로그입니다.",
            self.calib_cols_spin: "체커보드 내부 코너의 열 개수입니다.",
            self.calib_rows_spin: "체커보드 내부 코너의 행 개수입니다.",
            self.calib_square_spin: "내부 파라미터 보정에 사용할 체커보드 한 칸 크기입니다.",
            self.calib_min_images_spin: "카메라별 권장 유효 체커보드 프레임 최소 개수입니다.",
            self.calib_calculation_pairs_spin: (
                "내부 파라미터 계산에 실제로 사용할 matching pair 수입니다. "
                "0(전체 pair)이면 입력된 모든 pair를 사용합니다."
            ),
            self.calib_dir_edit: "보정 이미지와 intrinsics.json / intrinsics_zoom_##.json을 저장/로드할 디렉터리입니다.",
            self.capture_calibration_button: "현재 좌/우 카메라 프레임 중 체커보드가 검출된 쪽만 내부 파라미터 보정 이미지로 저장합니다.",
            self.calibration_pair_max_delta_spin: "좌·우 timing 차이 허용치입니다. Live는 decoded-frame app-receive monotonic, FTP는 CPTR host-dispatch proxy를 사용합니다. 두 host proxy 모두 실제 노출시각을 증명하지 않으며, 초과 이미지는 K,D용으로만 저장됩니다.",
            self.calibration_auto_interval_spin: "NAL1 non-delta exact FrameKey 자동 촬영 시도 간격입니다. 새 보드 pose는 저장하고, 좌우 시각 차이 초과분은 K,D용으로만 분류합니다.",
            self.capture_calibration_auto_button: "좌·우 NAL1 non-delta frame의 exact FrameKey에서 체커보드가 검출된 새 pose를 Stop할 때까지 저장합니다. 만료된 key는 대체하지 않으며 최대 pair Δt 초과분은 K,D용입니다.",
            self.run_calibration_button: "캡처된 체커보드 이미지로 OpenCV 내부 파라미터 보정을 실행합니다.",
            self.calibration_progress_bar: "내부 파라미터 및 stereo 계산에서 실제 검사한 이미지/pair 기준 진행률입니다.",
            self.save_intrinsics_button: "촬영 당시 양쪽 카메라 GZVV readback으로 확인된 줌 레벨의 내부 파라미터를 intrinsics_zoom_##.json으로 저장하고 latest alias intrinsics.json도 갱신합니다.",
            self.apply_intrinsics_button: "현재 WIRIS 줌 레벨의 intrinsics_zoom_##.json 또는 막 계산한 결과를 거리 측정에 적용합니다.",
            self.add_zoom_intrinsics_file_button: "비교 목록에 저장된 intrinsics JSON 파일을 추가합니다. 파일 자체는 복사하거나 변경하지 않습니다.",
            self.remove_zoom_intrinsics_file_button: "선택한 intrinsics JSON 파일을 비교 목록에서만 제거합니다. 저장된 파일은 삭제하지 않습니다.",
            self.apply_zoom_intrinsics_file_button: "비교 목록에서 선택한 intrinsics JSON 파일을 거리 계산에 바로 적용합니다.",
            self.zoom_intrinsics_compare_list: "저장된 내부 파라미터 파일들을 스크롤로 비교합니다. 항목은 드래그해서 순서를 바꿀 수 있습니다.",
            self.intrinsics_source_combo: "거리 계산에 사용할 내부 파라미터 출처입니다. 체커보드 보정값 또는 수동 초점거리 fallback 중 하나만 선택합니다.",
            self.intrinsics_status_label: "현재 내부 파라미터 적용 상태와 초점거리 출처를 표시합니다.",
            self.distance_mode_combo: "3D ray 거리 계산은 Pixhawk 자세 + GPS/RTK baseline 또는 체커보드 stereo R/T를 사용합니다.",
            self.distance_parameters_label: "현재 거리 계산에 쓰이는 초점거리, baseline/translation, 위치/자세 출처를 요약합니다.",
            self.calibration_left_preview: "체커보드 검출 overlay가 표시되는 왼쪽 보정 미리보기입니다.",
            self.calibration_right_preview: "체커보드 검출 overlay가 표시되는 오른쪽 보정 미리보기입니다.",
            self.focal_spin: (
                "거리 계산에 사용할 초점거리 fx(px) 참고값입니다.\n"
                "3D ray 계산은 적용된 내부 파라미터를 우선 사용하며, 체커보드가 없으면 "
                "`수동 초점거리 사용`을 켠 뒤 추정값을 입력하세요.\n\n"
                + WIRIS_VISIBLE_ZOOM_FX_TOOLTIP
            ),
            self.manual_focal_checkbox: (
                "로드된 내부 파라미터 대신 초점거리를 수동으로 편집할 수 있게 합니다.\n"
                "체커보드가 없을 때만 켜고, 체커보드 보정값을 쓰려면 끄세요.\n\n"
                + WIRIS_VISIBLE_ZOOM_FX_TOOLTIP
            ),
            self.baseline_spin: "거리 계산에 항상 사용하는 수동 카메라 baseline입니다. Baseline 탭의 보조 수동 계산 값을 여기에 적용할 수 있습니다.",
            self.distance_label: "현재 거리 추정값, 계산 방식, 동기화 품질, 경고를 자세히 표시합니다.",
            self.start_button: "YOLO 검출과 3D 거리 측정을 시작하거나, 측정 중이면 Pixhawk 연결은 유지한 채 카메라 Preview로 돌아갑니다.",
            self.record_button: (
                "측정 밖에서 수동 기록을 시작하거나 중지합니다. 좌/우 MP4는 "
                "생성 시각 파일명으로 저장되고, 구조화 데이터는 유형별 JSONL에 append됩니다."
            ),
            self.clear_history_button: "거리 그래프와 아직 저장하지 않은 메모리 기록을 지웁니다.",
            self.runtime_distance_graph_group: "baseline_center_frd Forward/Right/Down/Range 이력 그래프 영역을 접거나 펼칩니다.",
            self.distance_plot: (
                "3D ray 샘플의 baseline_center_frd Forward/Right/Down/Range 이력 그래프입니다. "
                "점 색상은 동기화 품질을 따릅니다.\n"
                "왼쪽 클릭: 점의 Forward/Right/Down과 midpoint 기준 Range를 표시합니다.\n"
                "오른쪽 클릭: 해당 점을 삭제합니다."
            ),
            self.save_history_button: "유형별 append writer에 flush를 요청합니다.",
            self.save_settings_button: "현재 내부 파라미터 경로와 전체 런타임 설정을 저장합니다.",
            self.recording_status_label: "메모리 기록, 매칭 프레임쌍, append writer 상태입니다.",
            self.recording_sheet_tabs: "현재 실행에서 수집된 기록을 미리 보여줍니다.",
            self.recording_sample_detail: "선택한 매칭 프레임쌍의 전체 내부 기록 필드를 표시합니다.",
            self.wiris_log: "WIRIS, 카메라 Preview 연결, 보정, 시간 확인 메시지 로그입니다.",
            self.camera_yolo_log: "YOLO, 거리 측정, 저장 메시지 로그입니다.",
            self.pixhawk_log: "Pixhawk, PX4, MAVLink 메시지 로그입니다.",
        }
        for widget, tooltip in tooltips.items():
            widget.setToolTip(tooltip)
        if hasattr(self, "rm3100_slot_labels"):
            for labels in self.rm3100_slot_labels.values():
                for slot, label in labels.items():
                    label.setToolTip(
                        f"{slot}에 배정된 최신 RM3100 샘플입니다. device_id는 Pixhawk별 최초 발견 순서로 매핑됩니다."
                    )
        if hasattr(self, "f9p_slot_labels"):
            for labels in self.f9p_slot_labels.values():
                for slot, label in labels.items():
                    label.setToolTip(
                        f"{slot}에 배정된 최신 F9P GNSS 샘플입니다. device_id는 Pixhawk별 최초 발견 순서로 매핑됩니다."
                    )
        if hasattr(self, "rm3100_norm_plots"):
            for plot in self.rm3100_norm_plots.values():
                plot.setToolTip("해당 Pixhawk의 RM3100 CAN1/CAN2 자기장 크기 overlay 그래프입니다.")
        if hasattr(self, "rm3100_delta_plots"):
            for plot in self.rm3100_delta_plots.values():
                plot.setToolTip("해당 Pixhawk의 RM3100 CAN1/CAN2 크기 차이와 벡터 각도 차이 그래프입니다.")
        if hasattr(self, "f9p_sats_plots"):
            for plot in self.f9p_sats_plots.values():
                plot.setToolTip("해당 Pixhawk의 F9P GPS1/GPS2 위성 수 overlay 그래프입니다.")
        if hasattr(self, "f9p_quality_plots"):
            for plot in self.f9p_quality_plots.values():
                plot.setToolTip("해당 Pixhawk의 F9P GPS1/GPS2 수평 정확도 overlay 그래프입니다.")

    def _refresh_operation_header(self) -> None:
        self._refresh_results_summary()

    def _refresh_results_summary(self) -> None:
        if not hasattr(self, "session_summary_label"):
            return
        rows = self._recording_preview_rows()
        if not rows:
            latest = self.distance_history[-1].distance_m if self.distance_history else None
            latest_text = "" if latest is None else f"\n최근 표시 거리: {latest:.2f} m"
            self.session_summary_label.setText(f"아직 기록된 매칭 프레임쌍이 없습니다.{latest_text}")
            return
        distances = []
        qualities: dict[str, int] = {}
        for row in rows:
            quality = str(row.get("quality", "unknown"))
            qualities[quality] = qualities.get(quality, 0) + 1
            try:
                distances.append(float(row.get("distance_m", 0.0)))
            except (TypeError, ValueError):
                pass
        latest_text = f"{distances[-1]:.2f} m" if distances else "--"
        average_text = f"{sum(distances) / len(distances):.2f} m" if distances else "--"
        quality_text = ", ".join(f"{name} {count}" for name, count in sorted(qualities.items()))
        self.session_summary_label.setText(
            f"매칭 프레임쌍: {len(self.distance_recording_rows)}개"
            f" (최근 {len(rows)}개 요약)\n"
            f"품질 분포: {quality_text or '없음'}\n"
            f"최신 거리: {latest_text} | 평균 거리: {average_text}"
        )

    def _apply_visual_style(self) -> None:
        base_font = QtGui.QFont("IBM Plex Sans")
        base_font.setPointSize(10)
        self.setFont(base_font)
        self.setStyleSheet(
            f"""
            QWidget {{
                font-family: {FONT_FAMILY};
                background: {APP_BG};
                color: {TEXT_MAIN};
                selection-background-color: {ACCENT};
                selection-color: #ffffff;
            }}
            QTabWidget::pane {{
                border: 1px solid {PANEL_BORDER};
                background: {PANEL_BG};
                top: -1px;
            }}
            QTabBar::tab {{
                background: {PANEL_ALT_BG};
                color: {TEXT_MUTED};
                border: 1px solid {PANEL_BORDER};
                border-bottom: none;
                padding: 7px 12px;
                min-height: 22px;
            }}
            QTabBar::tab:selected {{
                background: {PANEL_BG};
                color: {TEXT_MAIN};
                border-top: 3px solid {ACCENT};
            }}
            QGroupBox {{
                background: {PANEL_BG};
                border: 1px solid {PANEL_BORDER};
                border-radius: 2px;
                margin-top: 10px;
                padding: 10px 10px 8px 10px;
                font-weight: 600;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
                color: {TEXT_MAIN};
            }}
            QLabel {{
                background: transparent;
            }}
            QLineEdit, QPlainTextEdit, QTableWidget, QSpinBox, QDoubleSpinBox, QComboBox {{
                background: {PANEL_ALT_BG};
                color: {TEXT_MAIN};
                border: 1px solid {PANEL_BORDER_STRONG};
                border-radius: 0px;
                padding: 5px 7px;
                min-height: 22px;
            }}
            QHeaderView::section {{
                background: {PANEL_ALT_STRONG};
                color: {TEXT_MAIN};
                border: 1px solid {PANEL_BORDER};
                padding: 4px 6px;
                font-weight: 600;
            }}
            QTableWidget::item:selected {{
                background: {ACCENT};
                color: #ffffff;
            }}
            QPlainTextEdit {{
                font-family: monospace;
            }}
            QLineEdit:focus, QPlainTextEdit:focus, QTableWidget:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
                border-color: {ACCENT};
            }}
            QPushButton {{
                background: {PANEL_BG};
                color: {ACCENT};
                border: 1px solid {PANEL_BORDER_STRONG};
                border-radius: 0px;
                padding: 7px 10px;
                min-height: 28px;
                font-weight: 500;
            }}
            QPushButton:hover {{
                background: {INFO_BG};
                border-color: {ACCENT};
                color: {ACCENT_ACTIVE};
            }}
            QPushButton:disabled {{
                background: {PANEL_ALT_BG};
                color: {TEXT_SUBTLE};
                border-color: {PANEL_BORDER};
            }}
            QPushButton[role="primary"] {{
                background: {ACCENT};
                border-color: {ACCENT};
                color: #ffffff;
            }}
            QPushButton[role="primary"]:hover {{
                background: {ACCENT_HOVER};
                border-color: {ACCENT_HOVER};
            }}
            QPushButton[role="danger"] {{
                background: {ERROR};
                border-color: {ERROR};
                color: #ffffff;
            }}
            QPushButton[role="success"] {{
                background: {SUCCESS};
                border-color: {SUCCESS};
                color: #ffffff;
            }}
            QPushButton[role="toggle"] {{
                background: {PANEL_BG};
                border-color: {PANEL_BORDER_STRONG};
                color: {TEXT_MUTED};
            }}
            QPushButton[role="toggle"]:hover {{
                background: {INFO_BG};
                border-color: {ACCENT};
                color: {ACCENT_ACTIVE};
            }}
            QPushButton[role="toggle"]:checked {{
                background: {INFO_BG};
                border-color: {ACCENT};
                color: {ACCENT_ACTIVE};
            }}
            QPushButton[role="toggle"]:checked:hover {{
                background: {PANEL_ALT_STRONG};
                border-color: {ACCENT_HOVER};
            }}
            QCheckBox {{
                spacing: 7px;
                background: transparent;
            }}
            QLabel#statusStrip, QLabel#statChip {{
                background: {PANEL_ALT_BG};
                border: 1px solid {PANEL_BORDER};
                border-left: 3px solid {ACCENT};
                border-radius: 0px;
                color: {TEXT_MAIN};
                padding: 5px 7px;
            }}
            QLabel#distanceStatus {{
                background: {PANEL_ALT_BG};
                border: 1px solid {ACCENT};
                border-radius: 0px;
                padding: 7px 10px;
                font-size: 18px;
                font-weight: 700;
            }}
            QGroupBox#cameraPane {{
                margin-top: 16px;
            }}
            QLabel#cameraPreview, QLabel#calibrationPreview {{
                background: {INVERSE_BG};
                border: 1px solid {PANEL_BORDER_STRONG};
                border-radius: 0px;
                color: {INVERSE_TEXT_MUTED};
            }}
            QLabel#cameraMeta {{
                color: {TEXT_MUTED};
                padding: 2px 4px;
                font-weight: 500;
            }}
            QScrollBar:vertical, QScrollBar:horizontal {{
                background: {PANEL_ALT_BG};
                border: none;
                margin: 0;
            }}
            QScrollBar::handle {{
                background: {PANEL_BORDER_STRONG};
                border-radius: 0px;
                min-height: 24px;
                min-width: 24px;
            }}
            """
        )

    def _polish_controls(self) -> None:
        self._polish_form_layouts()
        primary_buttons = (
            self.detect_control_button,
            self.right_detect_control_button,
            self.left_control_apply_button,
            self.right_control_apply_button,
            self.auto_wiris_connect_button,
            self.detect_rtsp_button,
            self.right_detect_rtsp_button,
            self.left_rtsp_apply_button,
            self.right_rtsp_apply_button,
            self.browse_weights_button,
            self.camera_connect_button,
            self.left_mavlink_button,
            self.right_mavlink_button,
            self.start_button,
            self.capture_calibration_button,
            self.capture_calibration_auto_button,
            self.run_calibration_button,
            self.save_intrinsics_button,
            self.save_settings_button,
        )
        danger_buttons = ()
        success_buttons = (
            self.save_history_button,
            self.record_button,
            self.apply_intrinsics_button,
        )
        for button in primary_buttons:
            button.setProperty("role", "primary")
        for button in danger_buttons:
            button.setProperty("role", "danger")
        for button in success_buttons:
            button.setProperty("role", "success")
        for button in self.findChildren(QtWidgets.QPushButton):
            button.setCursor(QtCore.Qt.PointingHandCursor)
            button.setMinimumWidth(max(button.minimumWidth(), button.sizeHint().width() + 6, FORM_BUTTON_MIN_WIDTH))
            button.setMinimumHeight(max(button.minimumHeight(), button.sizeHint().height(), 38))
            button.style().unpolish(button)
            button.style().polish(button)
        for line_edit in self.findChildren(QtWidgets.QLineEdit):
            if line_edit.property("compactNetworkField") or line_edit.property("compactFormField"):
                continue
            line_edit.setMinimumWidth(max(line_edit.minimumWidth(), 220))
        for combo in self.findChildren(QtWidgets.QComboBox):
            combo.setMinimumWidth(max(combo.minimumWidth(), 170))
        for spin in self.findChildren(QtWidgets.QAbstractSpinBox):
            if spin.property("compactNetworkField") or spin.property("compactFormField"):
                continue
            spin.setMinimumWidth(max(spin.minimumWidth(), 120))
        self._polish_button_rows()
        self._reserve_pixhawk_pair_settings_height()
        self._reserve_pixhawk_candidate_group_height()

    def _reserve_pixhawk_pair_settings_height(self) -> None:
        group = getattr(self, "pixhawk_pair_settings_group", None)
        if group is None:
            return
        layout = group.layout()
        if layout is not None:
            layout.activate()
        group.setMinimumHeight(max(group.minimumHeight(), group.sizeHint().height() + 16))

    def _reserve_pixhawk_candidate_group_height(self) -> None:
        group = getattr(self, "pixhawk_candidate_group", None)
        if group is None:
            return
        for table in (
            getattr(self, "pc_pixhawk_candidate_table", None),
            getattr(self, "mavlink_pixhawk_candidate_table", None),
        ):
            if table is not None:
                table.setFixedHeight(240)
        layout = group.layout()
        if layout is not None:
            layout.activate()
        group.setMinimumHeight(max(group.minimumHeight(), group.sizeHint().height()))

    def _polish_form_layouts(self) -> None:
        layouts: list[QtWidgets.QLayout] = []
        root_layout = self.layout()
        if root_layout is not None:
            layouts.append(root_layout)
        layouts.extend(self.findChildren(QtWidgets.QLayout))
        for layout in dict.fromkeys(layouts):
            if isinstance(layout, QtWidgets.QGridLayout):
                self._polish_grid_form_layout(layout)
            elif isinstance(layout, QtWidgets.QHBoxLayout):
                self._polish_inline_form_layout(layout)

    def _iter_layouts(self, layout: QtWidgets.QLayout, seen: set[int] | None = None):
        if seen is None:
            seen = set()
        layout_id = id(layout)
        if layout_id in seen:
            return
        seen.add(layout_id)
        yield layout
        for index in range(layout.count()):
            item = layout.itemAt(index)
            child_layout = item.layout()
            if child_layout is not None:
                yield from self._iter_layouts(child_layout, seen)
            child_widget = item.widget()
            if child_widget is not None and child_widget.layout() is not None:
                yield from self._iter_layouts(child_widget.layout(), seen)

    def _polish_grid_form_layout(self, layout: QtWidgets.QGridLayout) -> None:
        for index in range(layout.count()):
            item = layout.itemAt(index)
            widget = item.widget()
            if not isinstance(widget, QtWidgets.QLabel) or not self._is_form_label(widget):
                continue
            row, column, _row_span, _column_span = layout.getItemPosition(index)
            next_item = layout.itemAtPosition(row, column + 1)
            if next_item is None or not self._layout_item_has_form_field(next_item):
                continue
            self._polish_form_label(widget)
            label_min_width = widget.minimumWidth()
            layout.setColumnMinimumWidth(column, max(layout.columnMinimumWidth(column), label_min_width))
            layout.setColumnStretch(column + 1, max(layout.columnStretch(column + 1), 1))

    def _polish_inline_form_layout(self, layout: QtWidgets.QHBoxLayout) -> None:
        for index in range(layout.count() - 1):
            item = layout.itemAt(index)
            next_item = layout.itemAt(index + 1)
            widget = item.widget()
            if not isinstance(widget, QtWidgets.QLabel) or not self._is_form_label(widget):
                continue
            if not self._layout_item_has_form_field(next_item):
                continue
            self._polish_form_label(widget)

    def _layout_item_has_form_field(self, item: QtWidgets.QLayoutItem) -> bool:
        widget = item.widget()
        if widget is not None:
            return isinstance(
                widget,
                (
                    QtWidgets.QLineEdit,
                    QtWidgets.QComboBox,
                    QtWidgets.QAbstractSpinBox,
                ),
            )
        layout = item.layout()
        if layout is None:
            return False
        for index in range(layout.count()):
            if self._layout_item_has_form_field(layout.itemAt(index)):
                return True
        return False

    def _is_form_label(self, label: QtWidgets.QLabel) -> bool:
        if not label.text().strip():
            return False
        if label.objectName() in {
            "statusStrip",
            "statChip",
            "distanceStatus",
            "cameraPreview",
            "calibrationPreview",
            "cameraMeta",
        }:
            return False
        if label.pixmap() is not None and not label.pixmap().isNull():
            return False
        return True

    def _polish_form_label(self, label: QtWidgets.QLabel) -> None:
        if (
            label.property("compactNetworkLabel")
            or label.property("compactAdvancedLabel")
            or label.property("compactFormLabel")
        ):
            label.setMinimumWidth(max(label.minimumWidth(), label.sizeHint().width() + 8))
        else:
            label.setMinimumWidth(max(label.minimumWidth(), FORM_LABEL_MIN_WIDTH))
        label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

    def _polish_button_rows(self) -> None:
        layouts: list[QtWidgets.QLayout] = []
        root_layout = self.layout()
        if root_layout is not None:
            layouts.append(root_layout)
        layouts.extend(self.findChildren(QtWidgets.QLayout))
        for layout in dict.fromkeys(layouts):
            if isinstance(layout, QtWidgets.QGridLayout):
                self._polish_grid_button_rows(layout)
            elif isinstance(layout, QtWidgets.QHBoxLayout):
                self._polish_hbox_button_row(layout)

    def _polish_grid_button_rows(self, layout: QtWidgets.QGridLayout) -> None:
        row_buttons: dict[int, list[tuple[int, int, QtWidgets.QPushButton]]] = {}
        rows_with_form_fields: set[int] = set()
        for index in range(layout.count()):
            item = layout.itemAt(index)
            widget = item.widget()
            if isinstance(widget, (QtWidgets.QLineEdit, QtWidgets.QComboBox, QtWidgets.QAbstractSpinBox)):
                row, _column, _row_span, _column_span = layout.getItemPosition(index)
                rows_with_form_fields.add(row)
                continue
            if not isinstance(widget, QtWidgets.QPushButton):
                continue
            row, column, _row_span, column_span = layout.getItemPosition(index)
            row_buttons.setdefault(row, []).append((column, column_span, widget))
        for row, buttons in row_buttons.items():
            if row in rows_with_form_fields:
                continue
            if len(buttons) < 2:
                continue
            for column, column_span, button in buttons:
                self._make_button_expand(button)
                for stretched_column in range(column, column + column_span):
                    layout.setColumnStretch(stretched_column, max(layout.columnStretch(stretched_column), 1))

    def _polish_hbox_button_row(self, layout: QtWidgets.QHBoxLayout) -> None:
        button_indexes: list[tuple[int, QtWidgets.QPushButton]] = []
        non_button_widgets = 0
        for index in range(layout.count()):
            widget = layout.itemAt(index).widget()
            if isinstance(widget, QtWidgets.QPushButton):
                button_indexes.append((index, widget))
            elif widget is not None and widget.isVisible():
                non_button_widgets += 1
        if len(button_indexes) < 2 or non_button_widgets:
            return
        for index, button in button_indexes:
            self._make_button_expand(button)
            layout.setStretch(index, 1)

    def _make_button_expand(self, button: QtWidgets.QPushButton) -> None:
        button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

    def _report_recoverable_persistence_failure(self, message: str) -> None:
        report_recoverable_persistence_failure(self, message)

    def _log(self, text: str) -> None:
        if QtCore.QThread.currentThread() != self.thread():
            self._threadsafe_log_requested.emit(str(text))
            return
        if (
            not bool(getattr(self, "_measurement_session_hierarchy_enabled", False))
            and bool(self._session_recording_active())
        ):
            writer = getattr(self, "append_workstation_data", None)
            if callable(writer):
                try:
                    from Workstation.workstation_core.package_provenance import redact_secrets
                    from Workstation.workstation_core.log_level import classify_log_level

                    writer(
                        "application.log",
                        {
                            "message": redact_secrets(str(text)),
                            "source": "Workstation._log",
                            "level": classify_log_level(text),
                        },
                        priority=False,
                    )
                except (OSError, TypeError, ValueError):
                    pass
        package_state = str(
            getattr(self, "_measurement_package_state", "IDLE") or "IDLE"
        ).upper()
        if bool(getattr(self, "_measurement_session_hierarchy_enabled", False)) and package_state in {
            "COLLECTING",
            "POST_CAPTURE",
            "VERIFYING",
            "RECOVERABLE",
        }:
            session_id = str(getattr(self, "recording_session_id", "") or "")
            if session_id:
                try:
                    from Workstation.workstation_core.package_provenance import redact_secrets
                    from Workstation.workstation_core.strict_jsonl import (
                        append_jsonl_record,
                        prepare_jsonl_append,
                    )

                    log_path = self._recording_session_dir(session_id) / "audit" / "application_log.jsonl"
                    log_path.parent.mkdir(parents=True, exist_ok=True)
                    from Workstation.workstation_core.log_level import classify_log_level

                    record = {
                        "schema": "workstation-application-log-v1",
                        "utc_ns": time.time_ns(),
                        "level": classify_log_level(text),
                        "message": redact_secrets(str(text)),
                    }
                    prepared_paths = getattr(
                        self,
                        "_prepared_application_log_paths",
                        set(),
                    )
                    if log_path not in prepared_paths:
                        prepare_jsonl_append(log_path)
                        prepared_paths.add(log_path)
                        self._prepared_application_log_paths = prepared_paths
                    append_jsonl_record(log_path, record, prepared=True)
                except (OSError, TypeError, ValueError):
                    # Evidence-log failure is surfaced by package inventory;
                    # it must never recursively break the operator log path.
                    pass
        targets: list[QtWidgets.QPlainTextEdit] = []
        is_ftp_message = "FTP" in text.upper()
        is_wiris_message = (
            text.startswith("[카메라]")
            or text.startswith("[캘리브레이션]")
            or text.startswith("[시계]")
            or text.startswith("[연결]")
            or text.startswith("[arp]")
            or "WIRIS" in text
        )
        is_pixhawk_message = text.startswith("[픽스호크]") or (
            not is_wiris_message and ("Pixhawk" in text or "PX4" in text or "MAVLink" in text)
        )
        if is_pixhawk_message:
            targets.append(self.pixhawk_log)
        if is_wiris_message:
            targets.append(self.wiris_log)
        if is_ftp_message:
            self.ftp_log_lines.append(text)
        if text.startswith("[YOLO]") or text.startswith("[Depth]") or text.startswith("[저장]"):
            targets.append(self.camera_yolo_log)
        if not targets:
            targets.append(self.wiris_log)
        for target in dict.fromkeys(targets):
            target.appendPlainText(text)
        if is_wiris_message or is_pixhawk_message:
            try:
                print(text, flush=True)
            except OSError:
                # A GUI launched without a live terminal must keep its UI/log
                # path alive even if stdout was closed by the launcher.
                pass

    def build_shell(self):
        root = QtWidgets.QVBoxLayout(self.widget)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        root.addWidget(self.tabs, 1)
        self._apply_visual_style()

        setup_tab = QtWidgets.QWidget()
        setup_layout = QtWidgets.QVBoxLayout(setup_tab)
        time_tab = QtWidgets.QWidget()
        time_layout = QtWidgets.QVBoxLayout(time_tab)
        alignment_tab = QtWidgets.QWidget()
        alignment_layout = QtWidgets.QVBoxLayout(alignment_tab)
        alignment_overview_tab = QtWidgets.QWidget()
        alignment_overview_layout = QtWidgets.QVBoxLayout(alignment_overview_tab)
        rtk_tab = QtWidgets.QWidget()
        rtk_layout = QtWidgets.QVBoxLayout(rtk_tab)
        mag_management_tab = QtWidgets.QWidget()
        mag_management_layout = QtWidgets.QVBoxLayout(mag_management_tab)
        alignment_mag_tab = QtWidgets.QWidget()
        alignment_mag_layout = QtWidgets.QVBoxLayout(alignment_mag_tab)
        alignment_attitude_tab = QtWidgets.QWidget()
        alignment_attitude_layout = QtWidgets.QVBoxLayout(alignment_attitude_tab)
        optical_alignment_tab = QtWidgets.QWidget()
        optical_alignment_layout = QtWidgets.QVBoxLayout(optical_alignment_tab)
        alignment_coordinate_tab = QtWidgets.QWidget()
        alignment_coordinate_layout = QtWidgets.QVBoxLayout(alignment_coordinate_tab)
        alignment_validation_tab = QtWidgets.QWidget()
        alignment_validation_layout = QtWidgets.QVBoxLayout(alignment_validation_tab)
        alignment_profile_tab = QtWidgets.QWidget()
        alignment_profile_layout = QtWidgets.QVBoxLayout(alignment_profile_tab)
        gnss_base_tab = QtWidgets.QWidget()
        gnss_base_layout = QtWidgets.QVBoxLayout(gnss_base_tab)
        signal_alignment_tab = QtWidgets.QWidget()
        signal_alignment_layout = QtWidgets.QVBoxLayout(signal_alignment_tab)
        rm3100_alignment_tab = QtWidgets.QWidget()
        rm3100_alignment_layout = QtWidgets.QVBoxLayout(rm3100_alignment_tab)
        yaw_alignment_tab = QtWidgets.QWidget()
        yaw_alignment_layout = QtWidgets.QVBoxLayout(yaw_alignment_tab)
        calibration_tab = QtWidgets.QWidget()
        calibration_layout = QtWidgets.QVBoxLayout(calibration_tab)
        checkerboard_calibration_tab = QtWidgets.QWidget()
        checkerboard_calibration_layout = QtWidgets.QVBoxLayout(checkerboard_calibration_tab)
        checkerboard_validation_tab = QtWidgets.QWidget()
        checkerboard_validation_layout = QtWidgets.QVBoxLayout(checkerboard_validation_tab)
        zoom_intrinsics_tab = QtWidgets.QWidget()
        zoom_intrinsics_layout = QtWidgets.QVBoxLayout(zoom_intrinsics_tab)
        camera_params_tab = QtWidgets.QWidget()
        camera_params_layout = QtWidgets.QVBoxLayout(camera_params_tab)
        intrinsic_parameters_tab = QtWidgets.QWidget()
        intrinsic_parameters_layout = QtWidgets.QVBoxLayout(intrinsic_parameters_tab)
        extrinsic_parameters_tab = QtWidgets.QWidget()
        extrinsic_parameters_layout = QtWidgets.QVBoxLayout(extrinsic_parameters_tab)
        triangulation_quality_tab = QtWidgets.QWidget()
        triangulation_quality_layout = QtWidgets.QVBoxLayout(triangulation_quality_tab)
        yolo_tab = QtWidgets.QWidget()
        yolo_layout_outer = QtWidgets.QVBoxLayout(yolo_tab)
        measurement_tab = QtWidgets.QWidget()
        measurement_layout = QtWidgets.QVBoxLayout(measurement_tab)
        stereo_tab = QtWidgets.QWidget()
        stereo_layout = QtWidgets.QVBoxLayout(stereo_tab)
        runtime_tab = QtWidgets.QWidget()
        runtime_layout = QtWidgets.QVBoxLayout(runtime_tab)
        save_tab = QtWidgets.QWidget()
        save_layout = QtWidgets.QVBoxLayout(save_tab)
        advanced_tab = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QVBoxLayout(advanced_tab)
        wiris_command_panel = QtWidgets.QWidget()
        wiris_command_layout = QtWidgets.QVBoxLayout(wiris_command_panel)
        baseline_field_browser_tab = QtWidgets.QWidget()
        baseline_field_browser_layout = QtWidgets.QVBoxLayout(baseline_field_browser_tab)
        px4_mag_advanced_tab = QtWidgets.QWidget()
        px4_mag_advanced_layout = QtWidgets.QVBoxLayout(px4_mag_advanced_tab)
        for layout in (
            setup_layout,
            time_layout,
            alignment_layout,
            alignment_overview_layout,
            rtk_layout,
            mag_management_layout,
            alignment_mag_layout,
            alignment_attitude_layout,
            optical_alignment_layout,
            alignment_coordinate_layout,
            alignment_validation_layout,
            alignment_profile_layout,
            signal_alignment_layout,
            rm3100_alignment_layout,
            yaw_alignment_layout,
            calibration_layout,
            checkerboard_calibration_layout,
            checkerboard_validation_layout,
            zoom_intrinsics_layout,
            camera_params_layout,
            intrinsic_parameters_layout,
            extrinsic_parameters_layout,
            yolo_layout_outer,
            measurement_layout,
            stereo_layout,
            runtime_layout,
            save_layout,
            advanced_layout,
            wiris_command_layout,
            baseline_field_browser_layout,
            px4_mag_advanced_layout,
        ):
            layout.setContentsMargins(6, 6, 6, 6)
            layout.setSpacing(6)

        self.tabs.addTab(setup_tab, "연결")
        self.tabs.addTab(time_tab, "시간")
        self.tabs.addTab(calibration_tab, "보정")
        self.tabs.addTab(camera_params_tab, "파라미터")
        self.tabs.addTab(alignment_tab, "정렬")
        self.tabs.addTab(yolo_tab, "YOLO")
        self.tabs.addTab(measurement_tab, "측정")
        self.tabs.addTab(save_tab, "저장·분석")
        self.tabs.addTab(advanced_tab, "고급")
        self.time_tab_index = self.tabs.indexOf(time_tab)
        self.setup_tab_index = self.tabs.indexOf(setup_tab)
        self.alignment_tab_index = self.tabs.indexOf(alignment_tab)
        self.calibration_tab_index = self.tabs.indexOf(calibration_tab)
        self.parameter_tab_index = self.tabs.indexOf(camera_params_tab)
        self.camera_parameters_tab_index = self.parameter_tab_index
        self.measurement_parameter_tab_index = self.parameter_tab_index
        self.depth_tab_index = self.tabs.indexOf(measurement_tab)
        self.results_tab_index = self.tabs.indexOf(save_tab)
        self.pixhawk_detail_tab_index = self.tabs.indexOf(alignment_tab)
        self.rtk_detail_tab_index = self.tabs.indexOf(alignment_tab)
        self.advanced_tab_index = self.tabs.indexOf(advanced_tab)
        self.tabs.currentChanged.connect(self._handle_tab_changed)

        calibration_tabs = QtWidgets.QTabWidget()
        calibration_tabs.setDocumentMode(True)
        calibration_tabs.addTab(checkerboard_calibration_tab, "체커보드")
        calibration_tabs.addTab(checkerboard_validation_tab, "체커보드 검증/계산")
        calibration_tabs.addTab(zoom_intrinsics_tab, "줌 내부 파라미터")
        calibration_layout.addWidget(calibration_tabs, 1)
        self.calibration_tabs = calibration_tabs
        self.checkerboard_calibration_tab_index = calibration_tabs.indexOf(checkerboard_calibration_tab)
        self.checkerboard_validation_tab_index = calibration_tabs.indexOf(checkerboard_validation_tab)
        self.zoom_intrinsics_tab_index = calibration_tabs.indexOf(zoom_intrinsics_tab)

        parameter_tabs = QtWidgets.QTabWidget()
        parameter_tabs.setDocumentMode(True)
        parameter_tabs.addTab(intrinsic_parameters_tab, "내부")
        parameter_tabs.addTab(extrinsic_parameters_tab, "외부/기하")
        parameter_tabs.addTab(stereo_tab, "계산 설정")
        camera_params_layout.addWidget(parameter_tabs, 1)
        self.parameter_tabs = parameter_tabs
        self.intrinsic_parameters_tab_index = parameter_tabs.indexOf(intrinsic_parameters_tab)
        self.extrinsic_parameters_tab_index = parameter_tabs.indexOf(extrinsic_parameters_tab)
        self.stereo_settings_tab_index = parameter_tabs.indexOf(stereo_tab)

        alignment_tabs = QtWidgets.QTabWidget()
        alignment_tabs.setDocumentMode(True)
        alignment_tabs.addTab(alignment_overview_tab, "센서 인식")
        alignment_tabs.addTab(rtk_tab, "소스 / 품질")
        alignment_tabs.addTab(gnss_base_tab, "GNSS Base")
        alignment_tabs.addTab(mag_management_tab, "MAG 관리")
        alignment_tabs.addTab(alignment_mag_tab, "Baseline 계산")
        alignment_tabs.addTab(alignment_attitude_tab, "Baseline / 자세 정렬")
        alignment_tabs.addTab(optical_alignment_tab, "Optical 정렬")
        alignment_layout.addWidget(alignment_tabs, 1)
        self.alignment_tabs = alignment_tabs
        self.alignment_overview_tab_index = alignment_tabs.indexOf(alignment_overview_tab)
        self.signal_alignment_tab_index = self.alignment_overview_tab_index
        self.source_quality_alignment_tab_index = alignment_tabs.indexOf(rtk_tab)
        self.gnss_base_alignment_tab_index = alignment_tabs.indexOf(gnss_base_tab)
        self.rm3100_alignment_tab_index = alignment_tabs.indexOf(mag_management_tab)
        self.baseline_alignment_tab_index = alignment_tabs.indexOf(alignment_mag_tab)
        self.yaw_alignment_tab_index = alignment_tabs.indexOf(alignment_attitude_tab)
        self.baseline_attitude_alignment_tab_index = self.yaw_alignment_tab_index
        self.optical_alignment_tab_index = alignment_tabs.indexOf(optical_alignment_tab)
        self.coordinate_alignment_tab_index = -1
        self.validation_alignment_tab_index = -1
        self.alignment_profile_tab_index = -1

        advanced_tabs = QtWidgets.QTabWidget()
        advanced_tabs.setDocumentMode(True)
        advanced_tabs.currentChanged.connect(lambda _index: self._handle_tab_changed(self.tabs.currentIndex()))
        advanced_layout.addWidget(advanced_tabs, 1)
        self.advanced_tabs = advanced_tabs

        measurement_tabs = QtWidgets.QTabWidget()
        measurement_tabs.setDocumentMode(True)
        measurement_tabs.addTab(runtime_tab, "실시간")
        measurement_tabs.addTab(triangulation_quality_tab, "측정 품질")
        measurement_layout.addWidget(measurement_tabs, 1)
        self.measurement_tabs = measurement_tabs
        self.live_measurement_tab_index = measurement_tabs.indexOf(runtime_tab)
        self.triangulation_quality_tab_index = measurement_tabs.indexOf(triangulation_quality_tab)
        measurement_tabs.currentChanged.connect(lambda _index: self._handle_tab_changed(self.tabs.currentIndex()))

        connection_tabs = QtWidgets.QTabWidget()
        connection_tabs.setDocumentMode(True)
        camera_tab = QtWidgets.QWidget()
        camera_layout_outer = QtWidgets.QVBoxLayout(camera_tab)
        pixhawk_connection_tab = QtWidgets.QWidget()
        pixhawk_connection_layout = QtWidgets.QVBoxLayout(pixhawk_connection_tab)
        for layout in (camera_layout_outer, yolo_layout_outer, pixhawk_connection_layout):
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(6)
        connection_tabs.addTab(camera_tab, "WIRIS")
        connection_tabs.addTab(pixhawk_connection_tab, "Pixhawk")
        self.setup_tabs = connection_tabs
        self.pixhawk_connection_tab_index = connection_tabs.indexOf(pixhawk_connection_tab)
        camera_group = QtWidgets.QGroupBox("카메라 / WIRIS")
        camera_layout = QtWidgets.QVBoxLayout(camera_group)
        camera_layout.setSpacing(10)
        camera_network_group = QtWidgets.QGroupBox("카메라망 / NIC")
        camera_form = QtWidgets.QGridLayout(camera_network_group)
        self.camera_network_form = camera_form
        camera_form.setHorizontalSpacing(8)
        camera_form.setVerticalSpacing(10)
        yolo_group = QtWidgets.QGroupBox("YOLO / 객체 검출")
        yolo_form = QtWidgets.QGridLayout(yolo_group)
        yolo_form.setHorizontalSpacing(8)
        yolo_form.setVerticalSpacing(8)
        pixhawk_group = QtWidgets.QGroupBox("픽스호크 / PX4")
        pixhawk_form = QtWidgets.QGridLayout(pixhawk_group)
        pixhawk_form.setHorizontalSpacing(8)
        pixhawk_form.setVerticalSpacing(8)
        self.pixhawk_settings_group = pixhawk_group
        return PanelShellParts(
            advanced_tabs=advanced_tabs,
            alignment_attitude_layout=alignment_attitude_layout,
            alignment_mag_layout=alignment_mag_layout,
            alignment_overview_layout=alignment_overview_layout,
            alignment_tabs=alignment_tabs,
            baseline_field_browser_layout=baseline_field_browser_layout,
            baseline_field_browser_tab=baseline_field_browser_tab,
            camera_form=camera_form,
            camera_group=camera_group,
            camera_layout=camera_layout,
            camera_layout_outer=camera_layout_outer,
            camera_network_group=camera_network_group,
            checkerboard_calibration_layout=checkerboard_calibration_layout,
            checkerboard_validation_layout=checkerboard_validation_layout,
            connection_tabs=connection_tabs,
            extrinsic_parameters_layout=extrinsic_parameters_layout,
            gnss_base_layout=gnss_base_layout,
            intrinsic_parameters_layout=intrinsic_parameters_layout,
            mag_management_layout=mag_management_layout,
            optical_alignment_layout=optical_alignment_layout,
            pixhawk_connection_layout=pixhawk_connection_layout,
            pixhawk_form=pixhawk_form,
            pixhawk_group=pixhawk_group,
            px4_mag_advanced_layout=px4_mag_advanced_layout,
            px4_mag_advanced_tab=px4_mag_advanced_tab,
            rtk_layout=rtk_layout,
            runtime_layout=runtime_layout,
            save_layout=save_layout,
            setup_layout=setup_layout,
            stereo_layout=stereo_layout,
            time_layout=time_layout,
            triangulation_quality_layout=triangulation_quality_layout,
            wiris_command_layout=wiris_command_layout,
            wiris_command_panel=wiris_command_panel,
            yolo_form=yolo_form,
            yolo_group=yolo_group,
            yolo_layout_outer=yolo_layout_outer,
            zoom_intrinsics_layout=zoom_intrinsics_layout,
        )

    def build_presentation_finish(self):
        self._refresh_session_toggle_buttons()
        self._install_tooltips()
        self._set_focal_override_enabled(self.manual_focal_checkbox.isChecked())
        self._update_distance_settings_visibility()
        self._refresh_camera_parameters_tab()
        self._refresh_operation_header()

