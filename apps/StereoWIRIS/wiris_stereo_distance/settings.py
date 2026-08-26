from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from wiris_stereo_distance.gnss_operation import GnssOperationConfig


APP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTINGS_PATH = APP_ROOT / "stereo_distance_settings.json"
DEFAULT_INTRINSICS_PATH = "calibration_captures/intrinsics.json"
DEFAULT_CLOCK_DOWNLOAD_DIR = "clock_captures"
PIXHAWK_ETHERNET_MODES = ("dual", "mavlink", "uxrce")
DEFAULT_PIXHAWK_ETHERNET_MODE = "dual"


@dataclass(frozen=True)
class AppSettings:
    intrinsics_path: str = DEFAULT_INTRINSICS_PATH
    yolo_weights_path: str = ".local/models/yolov8m.pt"
    interface_name: str = "enx588694fce07e"
    camera_host_address: str = "10.0.0.10/24"
    left_rtsp_url: str = "rtsp://10.0.0.231:8554/visible"
    right_rtsp_url: str = "rtsp://10.0.0.232:8554/visible"
    left_control_host: str = "10.0.0.231"
    right_control_host: str = "10.0.0.232"
    left_stip_ip: str = "10.0.0.231"
    right_stip_ip: str = "10.0.0.232"
    stip_subnet_mask: str = "255.255.255.0"
    stip_reboot: bool = True
    wiris_control_port: int = 2240
    left_control_port: int = 2240
    right_control_port: int = 2240
    # The left camera's SDK can acknowledge read commands after roughly 2 s.
    wiris_control_timeout_s: float = 5.0
    # Field-validated WIRIS visible encoder profile. RTSP enable reapplies
    # changed fields and verifies each readback before frame probing.
    wiris_visible_encoder_bitrate: int = 0
    wiris_visible_encoder_qp: int = 1
    wiris_visible_encoder_gop: int = 90
    wiris_visible_encoder_idr: int = 1
    yolo_device: str = "cpu"
    yolo_input_size: int = 416
    yolo_conf_threshold: float = 0.5
    yolo_iou_threshold: float = 0.45
    yolo_gate_class_enabled: bool = False
    yolo_gate_class_text: str = ""
    yolo_gate_matching_enabled: bool = False
    yolo_gate_time_delta_enabled: bool = False
    yolo_gate_vertical_delta_enabled: bool = False
    yolo_gate_max_vertical_delta_px: float = 80.0
    yolo_gate_max_time_delta_s: float = 0.1
    # Prefer RTCP/NTP when the two cameras agree, but allow live pairing to
    # fall back to the app's monotonic receive clock when their camera clocks
    # are not mutually synchronized.
    yolo_gate_time_basis: str = "camera_rtcp"
    preview_width: int = 1920
    rtsp_latency_ms: int = 50
    yolo_event_queue_size: int = 128
    yolo_frame_queue_size: int = 2
    rtsp_decoder: str = "auto"
    stream_backend: str = "gstreamer"
    # TCP interleaving keeps RTCP Sender Reports on the RTSP control channel;
    # use it as the safe default for absolute camera-clock verification.
    rtsp_transport: str = "tcp"
    camera_sync_history_window_s: float = 1.0
    camera_sync_receive_pair_max_delta_s: float = 0.350
    camera_sync_rtcp_pair_max_delta_s: float = 0.050
    camera_sync_max_sr_age_s: float = 7.0
    camera_sync_max_pc_offset_s: float = 0.100
    camera_sync_max_clock_rate_error: float = 0.005
    camera_sync_required_sr_count: int = 2
    camera_sync_warmup_timeout_s: float = 30.0
    ros_domain_id: int = 0
    left_namespace: str = "uav_0"
    right_namespace: str = "uav_1"
    pixhawk_host_address: str = "10.0.0.10"
    left_pixhawk_eth_ip: str = "10.0.0.2"
    right_pixhawk_eth_ip: str = "10.0.0.3"
    left_mavlink_url: str = "udpin:0.0.0.0:14550"
    right_mavlink_url: str = "udpin:0.0.0.0:14551"
    left_mavlink_baud: int = 57600
    right_mavlink_baud: int = 57600
    left_pixhawk_bootstrap_url: str = ""
    right_pixhawk_bootstrap_url: str = ""
    left_pixhawk_bootstrap_baud: int = 57600
    right_pixhawk_bootstrap_baud: int = 57600
    mavlink_configure_streams: bool = True
    mavlink_stream_rate: int = 2_000_000
    mavlink_configure_uxrce_agent: bool = True
    uxrce_port: int = 8888
    pixhawk_ethernet_mode: str = DEFAULT_PIXHAWK_ETHERNET_MODE
    auto_load_intrinsics: bool = True
    manual_focal_override: bool = False
    use_triangulation: bool = True
    triangulation_extrinsics_source: str = "pixhawk"
    measurement_display_mode: str = "detection"
    use_rtk_baseline: bool = True
    require_rtk_fixed: bool = False
    gnss_base_serial_port: str = "/dev/ttyACM0"
    gnss_base_baud: int = 460800
    gnss_base_survey_in_min_duration_s: int = 300
    gnss_base_survey_in_accuracy_m: float = 0.5
    gnss_base_rtcm_injection_target: str = "both"
    # Three-mode GNSS operation contract.  Legacy flat GNSS fields above are
    # kept for compatibility with the existing GUI and local Survey-in flow.
    gnss_operation: GnssOperationConfig = field(default_factory=GnssOperationConfig)
    clock_ftp_user: str = "anonymous"
    clock_ftp_password: str = ""
    clock_ftp_port: int = 21
    clock_ftp_remote_dir: str = "/"
    clock_download_dir: str = DEFAULT_CLOCK_DOWNLOAD_DIR
    clock_capture_wait_s: float = 10.0


def load_settings(path: str | Path = DEFAULT_SETTINGS_PATH) -> AppSettings:
    settings_path = Path(path)
    if not settings_path.exists():
        settings = AppSettings()
        save_settings(settings, settings_path)
        return settings
    payload = json.loads(settings_path.read_text())
    pixhawk_ethernet_mode = str(
        payload.get("pixhawk_ethernet_mode", DEFAULT_PIXHAWK_ETHERNET_MODE)
    )
    if pixhawk_ethernet_mode not in PIXHAWK_ETHERNET_MODES:
        pixhawk_ethernet_mode = DEFAULT_PIXHAWK_ETHERNET_MODE
    triangulation_extrinsics_source = str(
        payload.get("triangulation_extrinsics_source", AppSettings.triangulation_extrinsics_source)
    )
    if triangulation_extrinsics_source == "calibration":
        triangulation_extrinsics_source = "checkerboard"
    if triangulation_extrinsics_source not in {"pixhawk", "checkerboard"}:
        triangulation_extrinsics_source = "pixhawk"
    return AppSettings(
        intrinsics_path=str(payload.get("intrinsics_path", AppSettings.intrinsics_path)),
        yolo_weights_path=str(payload.get("yolo_weights_path", AppSettings.yolo_weights_path)),
        interface_name=str(payload.get("interface_name", AppSettings.interface_name)),
        camera_host_address=str(payload.get("camera_host_address", AppSettings.camera_host_address)),
        left_rtsp_url=str(payload.get("left_rtsp_url", AppSettings.left_rtsp_url)),
        right_rtsp_url=str(payload.get("right_rtsp_url", AppSettings.right_rtsp_url)),
        left_control_host=str(payload.get("left_control_host", AppSettings.left_control_host)),
        right_control_host=str(payload.get("right_control_host", AppSettings.right_control_host)),
        left_stip_ip=str(payload.get("left_stip_ip", AppSettings.left_stip_ip)),
        right_stip_ip=str(payload.get("right_stip_ip", AppSettings.right_stip_ip)),
        stip_subnet_mask=str(payload.get("stip_subnet_mask", AppSettings.stip_subnet_mask)),
        stip_reboot=bool(payload.get("stip_reboot", AppSettings.stip_reboot)),
        wiris_control_port=int(payload.get("wiris_control_port", AppSettings.wiris_control_port)),
        left_control_port=int(
            payload.get("left_control_port", payload.get("wiris_control_port", AppSettings.left_control_port))
        ),
        right_control_port=int(
            payload.get("right_control_port", payload.get("wiris_control_port", AppSettings.right_control_port))
        ),
        wiris_control_timeout_s=float(payload.get("wiris_control_timeout_s", AppSettings.wiris_control_timeout_s)),
        wiris_visible_encoder_bitrate=max(
            0,
            min(2**32 - 1, int(payload.get("wiris_visible_encoder_bitrate", AppSettings.wiris_visible_encoder_bitrate))),
        ),
        wiris_visible_encoder_qp=max(
            0,
            min(51, int(payload.get("wiris_visible_encoder_qp", AppSettings.wiris_visible_encoder_qp))),
        ),
        wiris_visible_encoder_gop=max(
            0,
            min(32767, int(payload.get("wiris_visible_encoder_gop", AppSettings.wiris_visible_encoder_gop))),
        ),
        wiris_visible_encoder_idr=max(
            0,
            min(2**32 - 1, int(payload.get("wiris_visible_encoder_idr", AppSettings.wiris_visible_encoder_idr))),
        ),
        yolo_device=str(payload.get("yolo_device", AppSettings.yolo_device)),
        yolo_input_size=int(payload.get("yolo_input_size", AppSettings.yolo_input_size)),
        yolo_conf_threshold=float(payload.get("yolo_conf_threshold", AppSettings.yolo_conf_threshold)),
        yolo_iou_threshold=float(payload.get("yolo_iou_threshold", AppSettings.yolo_iou_threshold)),
        yolo_gate_class_enabled=bool(payload.get("yolo_gate_class_enabled", AppSettings.yolo_gate_class_enabled)),
        yolo_gate_class_text=str(payload.get("yolo_gate_class_text", AppSettings.yolo_gate_class_text)),
        yolo_gate_matching_enabled=bool(
            payload.get("yolo_gate_matching_enabled", AppSettings.yolo_gate_matching_enabled)
        ),
        yolo_gate_time_delta_enabled=bool(
            payload.get("yolo_gate_time_delta_enabled", AppSettings.yolo_gate_time_delta_enabled)
        ),
        yolo_gate_vertical_delta_enabled=bool(
            payload.get("yolo_gate_vertical_delta_enabled", AppSettings.yolo_gate_vertical_delta_enabled)
        ),
        yolo_gate_max_vertical_delta_px=float(
            payload.get("yolo_gate_max_vertical_delta_px", AppSettings.yolo_gate_max_vertical_delta_px)
        ),
        yolo_gate_max_time_delta_s=float(
            payload.get("yolo_gate_max_time_delta_s", AppSettings.yolo_gate_max_time_delta_s)
        ),
        yolo_gate_time_basis=str(payload.get("yolo_gate_time_basis", AppSettings.yolo_gate_time_basis)),
        preview_width=int(payload.get("preview_width", AppSettings.preview_width)),
        rtsp_latency_ms=int(payload.get("rtsp_latency_ms", AppSettings.rtsp_latency_ms)),
        yolo_event_queue_size=max(
            1,
            int(payload.get("yolo_event_queue_size", AppSettings.yolo_event_queue_size)),
        ),
        yolo_frame_queue_size=max(
            1,
            int(payload.get("yolo_frame_queue_size", AppSettings.yolo_frame_queue_size)),
        ),
        rtsp_decoder=str(payload.get("rtsp_decoder", AppSettings.rtsp_decoder)),
        stream_backend=str(payload.get("stream_backend", AppSettings.stream_backend)),
        rtsp_transport=str(payload.get("rtsp_transport", AppSettings.rtsp_transport)),
        camera_sync_history_window_s=max(
            0.1,
            float(payload.get("camera_sync_history_window_s", AppSettings.camera_sync_history_window_s)),
        ),
        camera_sync_receive_pair_max_delta_s=max(
            0.0,
            float(
                payload.get(
                    "camera_sync_receive_pair_max_delta_s",
                    AppSettings.camera_sync_receive_pair_max_delta_s,
                )
            ),
        ),
        camera_sync_rtcp_pair_max_delta_s=max(
            0.0,
            float(
                payload.get(
                    "camera_sync_rtcp_pair_max_delta_s",
                    AppSettings.camera_sync_rtcp_pair_max_delta_s,
                )
            ),
        ),
        camera_sync_max_sr_age_s=max(
            0.1,
            float(payload.get("camera_sync_max_sr_age_s", AppSettings.camera_sync_max_sr_age_s)),
        ),
        camera_sync_max_pc_offset_s=max(
            0.0,
            float(payload.get("camera_sync_max_pc_offset_s", AppSettings.camera_sync_max_pc_offset_s)),
        ),
        camera_sync_max_clock_rate_error=max(
            0.0,
            float(
                payload.get(
                    "camera_sync_max_clock_rate_error",
                    AppSettings.camera_sync_max_clock_rate_error,
                )
            ),
        ),
        camera_sync_required_sr_count=max(
            1,
            int(payload.get("camera_sync_required_sr_count", AppSettings.camera_sync_required_sr_count)),
        ),
        camera_sync_warmup_timeout_s=max(
            1.0,
            float(payload.get("camera_sync_warmup_timeout_s", AppSettings.camera_sync_warmup_timeout_s)),
        ),
        ros_domain_id=int(payload.get("ros_domain_id", AppSettings.ros_domain_id)),
        left_namespace=str(payload.get("left_namespace", AppSettings.left_namespace)),
        right_namespace=str(payload.get("right_namespace", AppSettings.right_namespace)),
        pixhawk_host_address=str(payload.get("pixhawk_host_address", AppSettings.pixhawk_host_address)),
        left_pixhawk_eth_ip=str(payload.get("left_pixhawk_eth_ip", AppSettings.left_pixhawk_eth_ip)),
        right_pixhawk_eth_ip=str(payload.get("right_pixhawk_eth_ip", AppSettings.right_pixhawk_eth_ip)),
        left_mavlink_url=str(payload.get("left_mavlink_url", AppSettings.left_mavlink_url)),
        right_mavlink_url=str(payload.get("right_mavlink_url", AppSettings.right_mavlink_url)),
        left_mavlink_baud=int(payload.get("left_mavlink_baud", AppSettings.left_mavlink_baud)),
        right_mavlink_baud=int(payload.get("right_mavlink_baud", AppSettings.right_mavlink_baud)),
        left_pixhawk_bootstrap_url=str(
            payload.get("left_pixhawk_bootstrap_url", AppSettings.left_pixhawk_bootstrap_url)
        ),
        right_pixhawk_bootstrap_url=str(
            payload.get("right_pixhawk_bootstrap_url", AppSettings.right_pixhawk_bootstrap_url)
        ),
        left_pixhawk_bootstrap_baud=int(
            payload.get("left_pixhawk_bootstrap_baud", AppSettings.left_pixhawk_bootstrap_baud)
        ),
        right_pixhawk_bootstrap_baud=int(
            payload.get("right_pixhawk_bootstrap_baud", AppSettings.right_pixhawk_bootstrap_baud)
        ),
        mavlink_configure_streams=bool(
            payload.get("mavlink_configure_streams", AppSettings.mavlink_configure_streams)
        ),
        mavlink_stream_rate=int(payload.get("mavlink_stream_rate", AppSettings.mavlink_stream_rate)),
        mavlink_configure_uxrce_agent=bool(
            payload.get(
                "mavlink_configure_uxrce_agent",
                payload.get("mavlink_start_ros2", AppSettings.mavlink_configure_uxrce_agent),
            )
        ),
        uxrce_port=int(payload.get("uxrce_port", AppSettings.uxrce_port)),
        pixhawk_ethernet_mode=pixhawk_ethernet_mode,
        auto_load_intrinsics=bool(payload.get("auto_load_intrinsics", True)),
        manual_focal_override=bool(payload.get("manual_focal_override", False)),
        use_triangulation=bool(payload.get("use_triangulation", True)),
        triangulation_extrinsics_source=triangulation_extrinsics_source,
        measurement_display_mode=str(payload.get("measurement_display_mode", AppSettings.measurement_display_mode)),
        use_rtk_baseline=bool(payload.get("use_rtk_baseline", True)),
        require_rtk_fixed=bool(payload.get("require_rtk_fixed", False)),
        gnss_base_serial_port=str(payload.get("gnss_base_serial_port", AppSettings.gnss_base_serial_port)),
        gnss_base_baud=int(payload.get("gnss_base_baud", AppSettings.gnss_base_baud)),
        gnss_base_survey_in_min_duration_s=int(
            payload.get(
                "gnss_base_survey_in_min_duration_s",
                AppSettings.gnss_base_survey_in_min_duration_s,
            )
        ),
        gnss_base_survey_in_accuracy_m=float(
            payload.get("gnss_base_survey_in_accuracy_m", AppSettings.gnss_base_survey_in_accuracy_m)
        ),
        gnss_base_rtcm_injection_target=str(
            payload.get("gnss_base_rtcm_injection_target", AppSettings.gnss_base_rtcm_injection_target)
        ),
        gnss_operation=GnssOperationConfig.from_mapping(payload.get("gnss_operation")),
        clock_ftp_user=str(payload.get("clock_ftp_user", AppSettings.clock_ftp_user)),
        clock_ftp_password=str(payload.get("clock_ftp_password", AppSettings.clock_ftp_password)),
        clock_ftp_port=int(payload.get("clock_ftp_port", AppSettings.clock_ftp_port)),
        clock_ftp_remote_dir=str(payload.get("clock_ftp_remote_dir", AppSettings.clock_ftp_remote_dir)),
        clock_download_dir=str(payload.get("clock_download_dir", AppSettings.clock_download_dir)),
        clock_capture_wait_s=float(payload.get("clock_capture_wait_s", AppSettings.clock_capture_wait_s)),
    )


def save_settings(settings: AppSettings, path: str | Path = DEFAULT_SETTINGS_PATH) -> None:
    settings_path = Path(path)
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(settings.__dict__)
    payload["gnss_operation"] = settings.gnss_operation.to_mapping()
    settings_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False))


def resolve_settings_path(raw_path: str, *, base_dir: str | Path) -> Path:
    path = Path(raw_path).expanduser()
    if path.is_absolute():
        return path
    return Path(base_dir) / path
