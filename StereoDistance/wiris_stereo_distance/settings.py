from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any


APP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTINGS_PATH = APP_ROOT / "stereo_distance_settings.json"
DEFAULT_INTRINSICS_PATH = "calibration_captures/intrinsics.json"
DEFAULT_CLOCK_DOWNLOAD_DIR = "clock_captures"
LEGACY_DISPLAY_WIDTH_KEY = "pre" + "view_width"


@dataclass(frozen=True)
class AppSettings:
    intrinsics_path: str = DEFAULT_INTRINSICS_PATH
    yolo_weights_path: str = "yolov8m.pt"
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
    wiris_control_timeout_s: float = 1.0
    yolo_device: str = "cpu"
    yolo_input_size: int = 416
    yolo_conf_threshold: float = 0.5
    yolo_iou_threshold: float = 0.45
    yolo_gate_class_enabled: bool = False
    yolo_gate_class_text: str = ""
    yolo_gate_min_bbox_enabled: bool = False
    yolo_gate_min_width_px: int = 0
    yolo_gate_min_height_px: int = 0
    yolo_gate_min_area_px: int = 0
    yolo_gate_matching_enabled: bool = False
    yolo_gate_time_delta_enabled: bool = False
    yolo_gate_vertical_delta_enabled: bool = False
    yolo_gate_max_vertical_delta_px: float = 80.0
    yolo_gate_max_time_delta_s: float = 0.25
    yolo_gate_time_basis: str = "camera_rtc_required"
    preview_width: int = 1920
    rtsp_latency_ms: int = 50
    yolo_event_queue_size: int = 128
    yolo_frame_queue_size: int = 2
    record_video_queue_size: int = 300
    ffmpeg_stimeout_us: int = 2_000_000
    ffmpeg_max_delay_us: int = 500_000
    rtsp_decoder: str = "nvidia"
    stream_backend: str = "gstreamer"
    rtsp_transport: str = "udp"
    tracking_process_accel_std_mps2: float = 4.0
    tracking_initial_position_std_m: float = 25.0
    tracking_initial_velocity_std_mps: float = 20.0
    tracking_reset_after_s: float = 3.0
    tracking_max_normalized_innovation: float = 12.0
    tracking_min_lateral_std_m: float = 0.25
    tracking_min_forward_std_m: float = 2.0
    tracking_max_lateral_std_m: float = 50.0
    tracking_max_forward_std_m: float = 250.0
    baseline_bad_std_m: float = 0.10
    baseline_jump_m: float = 0.20
    baseline_max_timestamp_delta_ms: float = 200.0
    baseline_good_rtk_ratio: float = 0.95
    baseline_good_std_m: float = 0.02
    baseline_good_h_acc_m: float = 0.03
    baseline_good_v_acc_m: float = 0.05
    baseline_good_timestamp_delta_ms: float = 50.0
    baseline_warning_rtk_ratio: float = 0.70
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
    auto_load_intrinsics: bool = True
    manual_focal_override: bool = False
    use_triangulation: bool = True
    triangulation_extrinsics_source: str = "pixhawk"
    live_attitude_max_age_s: float = 0.25
    live_attitude_max_pair_delta_s: float = 0.10
    use_rtk_baseline: bool = True
    require_rtk_fixed: bool = False
    gnss_base_serial_port: str = "/dev/ttyACM0"
    gnss_base_baud: int = 460800
    gnss_base_survey_in_min_duration_s: int = 300
    gnss_base_survey_in_accuracy_m: float = 1.0
    gnss_base_rtcm_injection_target: str = "both"
    gnss_base_rtcm_transport: str = "mavlink"
    uam_telemetry_source: str = "mavlink"
    uam_ros2_topic: str = "/uav_0/fmu/out/vehicle_local_position"
    uam_ros2_x_field: str = "x"
    uam_ros2_y_field: str = "y"
    uam_ros2_z_field: str = "z"
    uam_mavlink_side: str = "left"
    uam_mavlink_url: str = "/dev/ttyACM0"
    uam_mavlink_baud: int = 57600
    uam_mavlink_message: str = "LOCAL_POSITION_NED"
    uam_mavlink_x_field: str = "x"
    uam_mavlink_y_field: str = "y"
    uam_mavlink_z_field: str = "z"
    uam_compare_max_dt_s: float = 0.25
    uam_compare_time_offset_s: float = 0.0
    clock_ftp_user: str = "anonymous"
    clock_ftp_password: str = ""
    clock_ftp_port: int = 21
    clock_ftp_remote_dir: str = "/"
    clock_download_dir: str = DEFAULT_CLOCK_DOWNLOAD_DIR
    clock_capture_wait_s: float = 10.0


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number is not allowed: {value}")


def _normalize_settings_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep malformed persisted values from becoming truthy/NaN runtime settings."""
    normalized = dict(payload)
    if "left_control_port" not in normalized and "wiris_control_port" in normalized:
        normalized["left_control_port"] = normalized["wiris_control_port"]
    if "right_control_port" not in normalized and "wiris_control_port" in normalized:
        normalized["right_control_port"] = normalized["wiris_control_port"]
    if "preview_width" not in normalized:
        for legacy_key in ("display_width", LEGACY_DISPLAY_WIDTH_KEY):
            if legacy_key in normalized:
                normalized["preview_width"] = normalized[legacy_key]
                break

    defaults = AppSettings().__dict__
    for key, default in defaults.items():
        if key not in normalized:
            continue
        value = normalized[key]
        if isinstance(default, bool):
            normalized[key] = value if isinstance(value, bool) else default
        elif isinstance(default, int) and not isinstance(default, bool):
            if isinstance(value, bool):
                parsed = default
            else:
                try:
                    parsed = int(value)
                except (TypeError, ValueError, OverflowError):
                    parsed = default
            normalized[key] = parsed
        elif isinstance(default, float):
            if isinstance(value, bool):
                parsed_float = default
            else:
                try:
                    parsed_float = float(value)
                except (TypeError, ValueError, OverflowError):
                    parsed_float = default
            normalized[key] = parsed_float if parsed_float == parsed_float and abs(parsed_float) != float("inf") else default
        elif isinstance(default, str):
            normalized[key] = value if isinstance(value, str) else default

    limits: dict[str, tuple[float | int | None, float | int | None]] = {
        "wiris_control_port": (1, 65535),
        "left_control_port": (1, 65535),
        "right_control_port": (1, 65535),
        "yolo_input_size": (32, 4096),
        "yolo_conf_threshold": (0.0, 1.0),
        "yolo_iou_threshold": (0.0, 1.0),
        "yolo_gate_min_width_px": (0, 10000),
        "yolo_gate_min_height_px": (0, 10000),
        "yolo_gate_min_area_px": (0, 100_000_000),
        "yolo_gate_max_vertical_delta_px": (0.0, 10000.0),
        "yolo_gate_max_time_delta_s": (0.001, 60.0),
        "preview_width": (64, 10000),
        "rtsp_latency_ms": (0, 60000),
        "yolo_event_queue_size": (1, 10000),
        "yolo_frame_queue_size": (1, 1000),
        "record_video_queue_size": (1, 1000),
        "ffmpeg_stimeout_us": (0, 60_000_000),
        "ffmpeg_max_delay_us": (0, 60_000_000),
        "tracking_process_accel_std_mps2": (0.001, 1000.0),
        "tracking_initial_position_std_m": (0.001, 10000.0),
        "tracking_initial_velocity_std_mps": (0.001, 10000.0),
        "tracking_reset_after_s": (0.001, 3600.0),
        "tracking_max_normalized_innovation": (0.001, 1000.0),
        "baseline_max_timestamp_delta_ms": (1.0, 60000.0),
        "baseline_good_rtk_ratio": (0.0, 1.0),
        "baseline_warning_rtk_ratio": (0.0, 1.0),
        "gnss_base_survey_in_accuracy_m": (0.001, 1000.0),
        "uam_compare_max_dt_s": (0.001, 60.0),
        "clock_capture_wait_s": (0.1, 3600.0),
    }
    for key, (lower, upper) in limits.items():
        if key not in normalized:
            continue
        value = normalized[key]
        try:
            valid = (lower is None or value >= lower) and (upper is None or value <= upper)
        except TypeError:
            valid = False
        if not valid:
            normalized[key] = defaults[key]
    return normalized


def load_settings(path: str | Path = DEFAULT_SETTINGS_PATH) -> AppSettings:
    settings_path = Path(path)
    if not settings_path.exists():
        settings = AppSettings()
        save_settings(settings, settings_path)
        return settings
    try:
        raw_payload = json.loads(settings_path.read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    except (OSError, json.JSONDecodeError, ValueError):
        return AppSettings()
    if not isinstance(raw_payload, dict):
        return AppSettings()
    payload = _normalize_settings_payload(raw_payload)
    triangulation_extrinsics_source = str(
        payload.get("triangulation_extrinsics_source", AppSettings.triangulation_extrinsics_source)
    )
    if triangulation_extrinsics_source not in {"pixhawk", "checkerboard_stereo_calibration"}:
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
        yolo_device=str(payload.get("yolo_device", AppSettings.yolo_device)),
        yolo_input_size=int(payload.get("yolo_input_size", AppSettings.yolo_input_size)),
        yolo_conf_threshold=float(payload.get("yolo_conf_threshold", AppSettings.yolo_conf_threshold)),
        yolo_iou_threshold=float(payload.get("yolo_iou_threshold", AppSettings.yolo_iou_threshold)),
        yolo_gate_class_enabled=bool(payload.get("yolo_gate_class_enabled", AppSettings.yolo_gate_class_enabled)),
        yolo_gate_class_text=str(payload.get("yolo_gate_class_text", AppSettings.yolo_gate_class_text)),
        yolo_gate_min_bbox_enabled=bool(
            payload.get("yolo_gate_min_bbox_enabled", AppSettings.yolo_gate_min_bbox_enabled)
        ),
        yolo_gate_min_width_px=int(payload.get("yolo_gate_min_width_px", AppSettings.yolo_gate_min_width_px)),
        yolo_gate_min_height_px=int(payload.get("yolo_gate_min_height_px", AppSettings.yolo_gate_min_height_px)),
        yolo_gate_min_area_px=int(payload.get("yolo_gate_min_area_px", AppSettings.yolo_gate_min_area_px)),
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
        preview_width=int(payload.get("preview_width", payload.get("display_width", payload.get(LEGACY_DISPLAY_WIDTH_KEY, AppSettings.preview_width)))),
        rtsp_latency_ms=int(payload.get("rtsp_latency_ms", AppSettings.rtsp_latency_ms)),
        yolo_event_queue_size=max(
            1,
            int(payload.get("yolo_event_queue_size", AppSettings.yolo_event_queue_size)),
        ),
        yolo_frame_queue_size=max(
            1,
            int(payload.get("yolo_frame_queue_size", AppSettings.yolo_frame_queue_size)),
        ),
        record_video_queue_size=max(
            1,
            int(payload.get("record_video_queue_size", AppSettings.record_video_queue_size)),
        ),
        ffmpeg_stimeout_us=max(
            0,
            int(payload.get("ffmpeg_stimeout_us", AppSettings.ffmpeg_stimeout_us)),
        ),
        ffmpeg_max_delay_us=max(
            0,
            int(payload.get("ffmpeg_max_delay_us", AppSettings.ffmpeg_max_delay_us)),
        ),
        rtsp_decoder=str(payload.get("rtsp_decoder", AppSettings.rtsp_decoder)),
        stream_backend=str(payload.get("stream_backend", AppSettings.stream_backend)),
        rtsp_transport=str(payload.get("rtsp_transport", AppSettings.rtsp_transport)),
        tracking_process_accel_std_mps2=float(
            payload.get("tracking_process_accel_std_mps2", AppSettings.tracking_process_accel_std_mps2)
        ),
        tracking_initial_position_std_m=float(
            payload.get("tracking_initial_position_std_m", AppSettings.tracking_initial_position_std_m)
        ),
        tracking_initial_velocity_std_mps=float(
            payload.get("tracking_initial_velocity_std_mps", AppSettings.tracking_initial_velocity_std_mps)
        ),
        tracking_reset_after_s=float(payload.get("tracking_reset_after_s", AppSettings.tracking_reset_after_s)),
        tracking_max_normalized_innovation=float(
            payload.get("tracking_max_normalized_innovation", AppSettings.tracking_max_normalized_innovation)
        ),
        tracking_min_lateral_std_m=float(
            payload.get("tracking_min_lateral_std_m", AppSettings.tracking_min_lateral_std_m)
        ),
        tracking_min_forward_std_m=float(
            payload.get("tracking_min_forward_std_m", AppSettings.tracking_min_forward_std_m)
        ),
        tracking_max_lateral_std_m=float(
            payload.get("tracking_max_lateral_std_m", AppSettings.tracking_max_lateral_std_m)
        ),
        tracking_max_forward_std_m=float(
            payload.get("tracking_max_forward_std_m", AppSettings.tracking_max_forward_std_m)
        ),
        baseline_bad_std_m=float(payload.get("baseline_bad_std_m", AppSettings.baseline_bad_std_m)),
        baseline_jump_m=float(payload.get("baseline_jump_m", AppSettings.baseline_jump_m)),
        baseline_max_timestamp_delta_ms=float(
            payload.get("baseline_max_timestamp_delta_ms", AppSettings.baseline_max_timestamp_delta_ms)
        ),
        baseline_good_rtk_ratio=float(payload.get("baseline_good_rtk_ratio", AppSettings.baseline_good_rtk_ratio)),
        baseline_good_std_m=float(payload.get("baseline_good_std_m", AppSettings.baseline_good_std_m)),
        baseline_good_h_acc_m=float(payload.get("baseline_good_h_acc_m", AppSettings.baseline_good_h_acc_m)),
        baseline_good_v_acc_m=float(payload.get("baseline_good_v_acc_m", AppSettings.baseline_good_v_acc_m)),
        baseline_good_timestamp_delta_ms=float(
            payload.get("baseline_good_timestamp_delta_ms", AppSettings.baseline_good_timestamp_delta_ms)
        ),
        baseline_warning_rtk_ratio=float(
            payload.get("baseline_warning_rtk_ratio", AppSettings.baseline_warning_rtk_ratio)
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
        auto_load_intrinsics=bool(payload.get("auto_load_intrinsics", True)),
        manual_focal_override=bool(payload.get("manual_focal_override", False)),
        use_triangulation=bool(payload.get("use_triangulation", True)),
        triangulation_extrinsics_source=triangulation_extrinsics_source,
        live_attitude_max_age_s=max(
            0.001,
            float(payload.get("live_attitude_max_age_s", AppSettings.live_attitude_max_age_s)),
        ),
        live_attitude_max_pair_delta_s=max(
            0.0,
            float(
                payload.get(
                    "live_attitude_max_pair_delta_s",
                    AppSettings.live_attitude_max_pair_delta_s,
                )
            ),
        ),
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
        gnss_base_rtcm_transport=str(
            payload.get("gnss_base_rtcm_transport", AppSettings.gnss_base_rtcm_transport)
        ),
        uam_telemetry_source=str(payload.get("uam_telemetry_source", AppSettings.uam_telemetry_source)),
        uam_ros2_topic=str(payload.get("uam_ros2_topic", AppSettings.uam_ros2_topic)),
        uam_ros2_x_field=str(payload.get("uam_ros2_x_field", AppSettings.uam_ros2_x_field)),
        uam_ros2_y_field=str(payload.get("uam_ros2_y_field", AppSettings.uam_ros2_y_field)),
        uam_ros2_z_field=str(payload.get("uam_ros2_z_field", AppSettings.uam_ros2_z_field)),
        uam_mavlink_side=str(payload.get("uam_mavlink_side", AppSettings.uam_mavlink_side)),
        uam_mavlink_url=str(payload.get("uam_mavlink_url", AppSettings.uam_mavlink_url)),
        uam_mavlink_baud=int(payload.get("uam_mavlink_baud", AppSettings.uam_mavlink_baud)),
        uam_mavlink_message=str(payload.get("uam_mavlink_message", AppSettings.uam_mavlink_message)),
        uam_mavlink_x_field=str(payload.get("uam_mavlink_x_field", AppSettings.uam_mavlink_x_field)),
        uam_mavlink_y_field=str(payload.get("uam_mavlink_y_field", AppSettings.uam_mavlink_y_field)),
        uam_mavlink_z_field=str(payload.get("uam_mavlink_z_field", AppSettings.uam_mavlink_z_field)),
        uam_compare_max_dt_s=float(payload.get("uam_compare_max_dt_s", AppSettings.uam_compare_max_dt_s)),
        uam_compare_time_offset_s=float(
            payload.get("uam_compare_time_offset_s", AppSettings.uam_compare_time_offset_s)
        ),
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
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=settings_path.parent,
            prefix=f".{settings_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            json.dump(settings.__dict__, handle, indent=2, ensure_ascii=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, settings_path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


def resolve_settings_path(raw_path: str, *, base_dir: str | Path) -> Path:
    path = Path(raw_path).expanduser()
    if path.is_absolute():
        return path
    return Path(base_dir) / path
