from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from Workstation.workstation_core.gnss_operation import GnssOperationConfig


APP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTINGS_PATH = APP_ROOT / "stereo_distance_settings.json"
DEFAULT_INTRINSICS_PATH = "calibration_captures/intrinsics.json"
DEFAULT_CALIBRATION_ROOT = "calibration_captures"
PIXHAWK_ETHERNET_MODES = ("mavlink",)
DEFAULT_PIXHAWK_ETHERNET_MODE = "mavlink"


def _load_gnss_operation(value: object) -> GnssOperationConfig:
    """Load GNSS settings and retire the former public-caster profile.

    Existing field settings that still point at the retired caster are
    switched to the NGII RTS2 profile before the runtime can open a network
    connection. Explicit non-retired operation modes and caster settings are
    preserved.
    """

    if value is None:
        return GnssOperationConfig.ngii_rts2_base_assisted()
    config = GnssOperationConfig.from_mapping(value)
    host = config.network.caster_host.casefold().strip().rstrip(".")
    mountpoint = config.network.mountpoint.casefold().strip().lstrip("/")
    if host == "rtk2go.com" or host.endswith(".rtk2go.com") or mountpoint == "kaist-rtk-rtcm":
        return GnssOperationConfig.ngii_rts2_base_assisted()
    return config


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
    # WIRIS Enterprise's onboard FTP server, reached over the same host as
    # the TCP command channel above (different port only). Used to retrieve
    # a captured HR/OZ still after the camera reports it stored.
    # Each camera has its own local FTP account (the login is set per-unit on
    # the camera itself), so port/user/password/remote dir/timeout are kept
    # separate per side; only the download toggle is shared.
    wiris_ftp_enabled: bool = False
    wiris_ftp_port: int = 21
    wiris_ftp_user: str = "anonymous"
    wiris_ftp_password: str = ""
    wiris_ftp_remote_dir: str = "/"
    wiris_ftp_timeout_s: float = 10.0
    left_ftp_port: int = 21
    right_ftp_port: int = 21
    left_ftp_user: str = "anonymous"
    right_ftp_user: str = "anonymous"
    left_ftp_password: str = ""
    right_ftp_password: str = ""
    left_ftp_remote_dir: str = "/"
    right_ftp_remote_dir: str = "/"
    left_ftp_timeout_s: float = 10.0
    right_ftp_timeout_s: float = 10.0
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
    preview_width: int = 1280
    calibration_zoom_level: int = 10
    calibration_inner_cols: int = 13
    calibration_inner_rows: int = 8
    calibration_square_size_mm: float = 40.0
    calibration_min_images: int = 20
    # Maximum number of matching stereo pairs used by the full calibration
    # calculation.  Zero means use every pair in the selected root.
    calibration_calculation_pair_count: int = 20
    calibration_auto_interval_s: float = 1.0
    calibration_auto_target_pairs: int = 20
    calibration_root: str = DEFAULT_CALIBRATION_ROOT
    calibration_validation_mode: str = "pixhawk"
    calibration_validation_root: str = DEFAULT_CALIBRATION_ROOT
    calibration_validation_json: str = DEFAULT_INTRINSICS_PATH
    calibration_validation_runs: int = 3
    calibration_validation_pair_count: int = 0
    calibration_validation_seed: int = 0
    rtsp_latency_ms: int = 100
    yolo_event_queue_size: int = 128
    yolo_frame_queue_size: int = 2
    rtsp_decoder: str = "auto"
    stream_backend: str = "gstreamer"
    rtsp_transport: str = "udp"
    camera_sync_history_window_s: float = 1.0
    camera_sync_receive_pair_max_delta_s: float = 0.350
    camera_sync_rtcp_pair_max_delta_s: float = 0.050
    camera_sync_max_sr_age_s: float = 7.0
    camera_sync_max_pc_offset_s: float = 0.100
    camera_sync_max_clock_rate_error: float = 0.005
    camera_sync_required_sr_count: int = 2
    camera_sync_warmup_timeout_s: float = 30.0
    pixhawk_host_address: str = "10.0.0.10"
    left_pixhawk_eth_ip: str = "10.0.0.2"
    right_pixhawk_eth_ip: str = "10.0.0.3"
    left_mavlink_url: str = "udpin:0.0.0.0:14550"
    right_mavlink_url: str = "udpin:0.0.0.0:14551"
    left_mavlink_baud: int = 57600
    right_mavlink_baud: int = 57600
    # The third Pixhawk link is an active telemetry/correction endpoint for
    # the drone.  It is deliberately separate from the receive-only target
    # drone artifact path.
    drone_telemetry_url: str = "udpin:0.0.0.0:14552"
    drone_telemetry_baud: int = 57600
    drone_telemetry_rtcm_forward: bool = True
    drone_telemetry_rtcm_source: str = "base"
    left_pixhawk_bootstrap_url: str = ""
    right_pixhawk_bootstrap_url: str = ""
    left_pixhawk_bootstrap_baud: int = 57600
    right_pixhawk_bootstrap_baud: int = 57600
    mavlink_configure_streams: bool = True
    mavlink_stream_rate: int = 2_000_000
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


def load_settings(path: str | Path = DEFAULT_SETTINGS_PATH) -> AppSettings:
    settings_path = Path(path)
    if not settings_path.exists():
        settings = AppSettings(gnss_operation=GnssOperationConfig.ngii_rts2_base_assisted())
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
    calibration_validation_mode = str(
        payload.get("calibration_validation_mode", AppSettings.calibration_validation_mode)
    )
    if calibration_validation_mode not in {"pixhawk", "checkerboard"}:
        calibration_validation_mode = AppSettings.calibration_validation_mode
    drone_telemetry_rtcm_source = str(
        payload.get("drone_telemetry_rtcm_source", AppSettings.drone_telemetry_rtcm_source)
    ).strip().lower()
    if drone_telemetry_rtcm_source not in {"base", "ntrip"}:
        drone_telemetry_rtcm_source = AppSettings.drone_telemetry_rtcm_source
    gnss_operation = _load_gnss_operation(payload.get("gnss_operation"))
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
        wiris_ftp_enabled=bool(payload.get("wiris_ftp_enabled", AppSettings.wiris_ftp_enabled)),
        wiris_ftp_port=int(payload.get("wiris_ftp_port", AppSettings.wiris_ftp_port)),
        wiris_ftp_user=str(payload.get("wiris_ftp_user", AppSettings.wiris_ftp_user)),
        wiris_ftp_password=str(payload.get("wiris_ftp_password", AppSettings.wiris_ftp_password)),
        wiris_ftp_remote_dir=str(payload.get("wiris_ftp_remote_dir", AppSettings.wiris_ftp_remote_dir)),
        wiris_ftp_timeout_s=float(payload.get("wiris_ftp_timeout_s", AppSettings.wiris_ftp_timeout_s)),
        left_ftp_port=int(payload.get("left_ftp_port", payload.get("wiris_ftp_port", AppSettings.left_ftp_port))),
        right_ftp_port=int(
            payload.get("right_ftp_port", payload.get("wiris_ftp_port", AppSettings.right_ftp_port))
        ),
        left_ftp_user=str(payload.get("left_ftp_user", payload.get("wiris_ftp_user", AppSettings.left_ftp_user))),
        right_ftp_user=str(
            payload.get("right_ftp_user", payload.get("wiris_ftp_user", AppSettings.right_ftp_user))
        ),
        left_ftp_password=str(
            payload.get("left_ftp_password", payload.get("wiris_ftp_password", AppSettings.left_ftp_password))
        ),
        right_ftp_password=str(
            payload.get("right_ftp_password", payload.get("wiris_ftp_password", AppSettings.right_ftp_password))
        ),
        left_ftp_remote_dir=str(
            payload.get("left_ftp_remote_dir", payload.get("wiris_ftp_remote_dir", AppSettings.left_ftp_remote_dir))
        ),
        right_ftp_remote_dir=str(
            payload.get(
                "right_ftp_remote_dir", payload.get("wiris_ftp_remote_dir", AppSettings.right_ftp_remote_dir)
            )
        ),
        left_ftp_timeout_s=float(
            payload.get("left_ftp_timeout_s", payload.get("wiris_ftp_timeout_s", AppSettings.left_ftp_timeout_s))
        ),
        right_ftp_timeout_s=float(
            payload.get("right_ftp_timeout_s", payload.get("wiris_ftp_timeout_s", AppSettings.right_ftp_timeout_s))
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
        calibration_zoom_level=max(
            0,
            min(100, int(payload.get("calibration_zoom_level", AppSettings.calibration_zoom_level))),
        ),
        calibration_inner_cols=max(
            2,
            min(30, int(payload.get("calibration_inner_cols", AppSettings.calibration_inner_cols))),
        ),
        calibration_inner_rows=max(
            2,
            min(30, int(payload.get("calibration_inner_rows", AppSettings.calibration_inner_rows))),
        ),
        calibration_square_size_mm=max(
            1.0,
            min(500.0, float(payload.get("calibration_square_size_mm", AppSettings.calibration_square_size_mm))),
        ),
        calibration_min_images=max(
            3,
            min(100, int(payload.get("calibration_min_images", AppSettings.calibration_min_images))),
        ),
        calibration_calculation_pair_count=max(
            0,
            min(
                10000,
                int(
                    payload.get(
                        "calibration_calculation_pair_count",
                        AppSettings.calibration_calculation_pair_count,
                    )
                ),
            ),
        ),
        calibration_auto_interval_s=max(
            0.5,
            min(10.0, float(payload.get("calibration_auto_interval_s", AppSettings.calibration_auto_interval_s))),
        ),
        calibration_auto_target_pairs=max(
            1,
            min(200, int(payload.get("calibration_auto_target_pairs", AppSettings.calibration_auto_target_pairs))),
        ),
        calibration_root=str(payload.get("calibration_root", AppSettings.calibration_root)),
        calibration_validation_mode=calibration_validation_mode,
        calibration_validation_root=str(
            payload.get("calibration_validation_root", AppSettings.calibration_validation_root)
        ),
        calibration_validation_json=str(
            payload.get("calibration_validation_json", AppSettings.calibration_validation_json)
        ),
        calibration_validation_runs=max(
            1,
            min(20, int(payload.get("calibration_validation_runs", AppSettings.calibration_validation_runs))),
        ),
        calibration_validation_pair_count=max(
            0,
            min(
                10000,
                int(payload.get("calibration_validation_pair_count", AppSettings.calibration_validation_pair_count)),
            ),
        ),
        calibration_validation_seed=max(
            0,
            int(payload.get("calibration_validation_seed", AppSettings.calibration_validation_seed)),
        ),
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
        pixhawk_host_address=str(payload.get("pixhawk_host_address", AppSettings.pixhawk_host_address)),
        left_pixhawk_eth_ip=str(payload.get("left_pixhawk_eth_ip", AppSettings.left_pixhawk_eth_ip)),
        right_pixhawk_eth_ip=str(payload.get("right_pixhawk_eth_ip", AppSettings.right_pixhawk_eth_ip)),
        left_mavlink_url=str(payload.get("left_mavlink_url", AppSettings.left_mavlink_url)),
        right_mavlink_url=str(payload.get("right_mavlink_url", AppSettings.right_mavlink_url)),
        left_mavlink_baud=int(payload.get("left_mavlink_baud", AppSettings.left_mavlink_baud)),
        right_mavlink_baud=int(payload.get("right_mavlink_baud", AppSettings.right_mavlink_baud)),
        drone_telemetry_url=str(payload.get("drone_telemetry_url", AppSettings.drone_telemetry_url)),
        drone_telemetry_baud=int(payload.get("drone_telemetry_baud", AppSettings.drone_telemetry_baud)),
        drone_telemetry_rtcm_forward=bool(
            payload.get("drone_telemetry_rtcm_forward", AppSettings.drone_telemetry_rtcm_forward)
        ),
        drone_telemetry_rtcm_source=drone_telemetry_rtcm_source,
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
        gnss_operation=gnss_operation,
    )


def save_settings(settings: AppSettings, path: str | Path = DEFAULT_SETTINGS_PATH) -> None:
    settings_path = Path(path)
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(settings.__dict__)
    payload["gnss_operation"] = settings.gnss_operation.to_mapping()
    # Write via a temp file + atomic replace so a write interrupted mid-way
    # (disk full, process killed, power loss) can never leave a half-written
    # settings file behind -- load_settings() has no error handling around
    # json.loads() and runs at startup, so a truncated file would otherwise
    # crash the app on every subsequent launch.
    fd, temp_name = tempfile.mkstemp(
        dir=settings_path.parent, prefix=f".{settings_path.name}.", suffix=".tmp"
    )
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, indent=2, ensure_ascii=False))
        os.replace(temp_path, settings_path)
    finally:
        temp_path.unlink(missing_ok=True)


def resolve_settings_path(raw_path: str, *, base_dir: str | Path) -> Path:
    path = Path(raw_path).expanduser()
    if path.is_absolute():
        return path
    return Path(base_dir) / path
