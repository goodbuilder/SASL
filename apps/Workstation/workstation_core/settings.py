from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

from Workstation.runtime_paths import (
    REPO_ROOT,
    WORKSTATION_DATA_ROOT,
    WORKSTATION_SETTINGS_PATH,
)
from Workstation.workstation_core.gnss_operation import GnssOperationConfig
from Workstation.workstation_core.measurement_methods import (
    DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
    PIXHAWK_RTK_LIVE,
    normalize_measurement_method_fallback_order,
    normalize_measurement_method_id,
)
from Workstation.workstation_core.stus_scheduler import (
    STUS_DEFAULT_INTERVAL_S,
    STUS_MODE_MANUAL,
    normalize_stus_interval_s,
    normalize_stus_mode,
)


APP_ROOT = Path(__file__).resolve().parents[1]
LEGACY_DEFAULT_SETTINGS_PATH = APP_ROOT / "stereo_distance_settings.json"
LEGACY_REPOSITORY_DATA_ROOT = REPO_ROOT / ".local" / "workstation_data"
LEGACY_CALIBRATION_CAPTURES_ROOT = WORKSTATION_DATA_ROOT / "calibration_captures"
LEGACY_MODELS_ROOT = REPO_ROOT / ".local" / "models"
DEFAULT_SETTINGS_PATH = WORKSTATION_SETTINGS_PATH
DEFAULT_CALIBRATION_ROOT = "calibration"
DEFAULT_INTRINSICS_PATH = "calibration/intrinsics.json"
DEFAULT_YOLO_WEIGHTS_PATH = "models/yolov8m.pt"
DEFAULT_MEASUREMENT_DATA_ROOT = ""
LEGACY_DEFAULT_MEASUREMENT_DATA_ROOT = "measurement_data"
PIXHAWK_ETHERNET_MODES = ("mavlink",)
DEFAULT_PIXHAWK_ETHERNET_MODE = "mavlink"
PIXHAWK_TRANSPORT_SCHEMA_VERSION = 2
PIXHAWK_TRANSPORT_RADIO = "radio"
PIXHAWK_TRANSPORT_ETHERNET = "ethernet"
PIXHAWK_TRANSPORT_USB = "usb"
PIXHAWK_TRANSPORTS = (
    PIXHAWK_TRANSPORT_RADIO,
    PIXHAWK_TRANSPORT_ETHERNET,
    PIXHAWK_TRANSPORT_USB,
)
# Radio and USB both open one exclusive serial device; only the bandwidth
# policy differs, so endpoint locking and path preflight key off this set.
PIXHAWK_SERIAL_TRANSPORTS = (PIXHAWK_TRANSPORT_RADIO, PIXHAWK_TRANSPORT_USB)
PIXHAWK_GNSS_SOURCES = ("GPS1", "GPS2")
RTSP_ENCODED_QUEUE_MODE_LATEST = "latest"
RTSP_ENCODED_QUEUE_MODE_REFERENCE_SAFE = "reference_safe"
RTSP_ENCODED_QUEUE_MODES = (
    RTSP_ENCODED_QUEUE_MODE_LATEST,
    RTSP_ENCODED_QUEUE_MODE_REFERENCE_SAFE,
)
LIVE_DISTANCE_TIME_BASIS_APP_RECEIVE = "app_receive_monotonic"
LIVE_DISTANCE_TIME_BASIS_RTCP_PREFERRED = "camera_rtcp"
LIVE_DISTANCE_TIME_BASES = (
    LIVE_DISTANCE_TIME_BASIS_APP_RECEIVE,
    LIVE_DISTANCE_TIME_BASIS_RTCP_PREFERRED,
)


def normalize_rtsp_encoded_queue_mode(value: object) -> str:
    mode = str(value or RTSP_ENCODED_QUEUE_MODE_LATEST).strip().lower()
    return (
        mode
        if mode in RTSP_ENCODED_QUEUE_MODES
        else RTSP_ENCODED_QUEUE_MODE_LATEST
    )


def normalize_live_distance_time_basis(value: object) -> str:
    """Normalize the persisted preference while preserving mandatory app fallback.

    ``camera_rtc_required`` was the former strict live value.  The current
    operator contract has no strict live mode, so it migrates to the RTCP
    preference whose runtime fallback is app-receive monotonic.
    """

    basis = str(value or LIVE_DISTANCE_TIME_BASIS_APP_RECEIVE).strip().lower()
    if basis in {LIVE_DISTANCE_TIME_BASIS_RTCP_PREFERRED, "camera_rtc_required"}:
        return LIVE_DISTANCE_TIME_BASIS_RTCP_PREFERRED
    return LIVE_DISTANCE_TIME_BASIS_APP_RECEIVE


def normalize_measurement_data_root(value: object) -> str:
    """Map the former implicit default onto the repository recordings root.

    An empty value is a portable sentinel resolved by ``DataStorageController``.
    Explicit custom paths remain relative to ``SASL_WORKSTATION_DATA_ROOT``.
    """

    text = str(value or "").strip()
    return "" if text in {"", LEGACY_DEFAULT_MEASUREMENT_DATA_ROOT} else text


def normalize_pixhawk_gnss_source(value: object, default: str = "GPS1") -> str:
    source = str(value or default).strip().upper()
    return source if source in PIXHAWK_GNSS_SOURCES else default


def normalize_pixhawk_transport(value: object) -> str:
    transport = str(value or PIXHAWK_TRANSPORT_RADIO).strip().lower()
    return transport if transport in PIXHAWK_TRANSPORTS else PIXHAWK_TRANSPORT_RADIO


def normalize_network_role_owned_entries(value: object) -> dict[str, dict[str, list[str]]]:
    """Keep only JSON-safe exact entries attributed to Workstation."""

    if not isinstance(value, dict):
        return {}
    normalized: dict[str, dict[str, list[str]]] = {}
    for raw_key, raw_entries in value.items():
        key = str(raw_key or "").strip()
        if not key or not isinstance(raw_entries, dict):
            continue
        addresses = [
            str(item)
            for item in (raw_entries.get("addresses") or ())
            if str(item or "").strip()
        ]
        routes = [
            str(item)
            for item in (raw_entries.get("routes") or ())
            if str(item or "").strip()
        ]
        normalized[key] = {
            "addresses": list(dict.fromkeys(addresses)),
            "routes": list(dict.fromkeys(routes)),
        }
    return normalized


def _default_path_with_legacy_read(
    value: object,
    *,
    canonical: str | Path,
    legacy_markers: set[str],
    legacy_candidates: tuple[Path, ...],
) -> str:
    """Resolve only historical default spellings, never explicit custom paths.

    New/default writes target the canonical Workstation data root.  A settings
    file that still contains a historical default continues to read an
    existing legacy asset in place; the asset is not copied, renamed, or
    deleted.  Any non-default value is returned byte-for-byte.
    """

    text = str(value or "")
    marker = text.strip()
    if not marker:
        return str(canonical)
    if marker not in legacy_markers:
        return text
    for candidate in legacy_candidates:
        if candidate.exists():
            return str(candidate)
    return str(canonical)


def _settings_read_path(settings_path: Path) -> Path:
    if settings_path.exists():
        return settings_path
    if settings_path == WORKSTATION_SETTINGS_PATH and LEGACY_DEFAULT_SETTINGS_PATH.exists():
        return LEGACY_DEFAULT_SETTINGS_PATH
    return settings_path


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
    yolo_weights_path: str = DEFAULT_YOLO_WEIGHTS_PATH
    interface_name: str = "enx588694fce07e"
    # Camera traffic is isolated from the Pixhawk NIC, which owns
    # 10.0.0.10/24.  The /32 address relies on the explicit .231/.232 host
    # routes and prevents a second connected 10.0.0.0/24 from recreating the
    # shared-L2 MAVLink stall trigger.
    camera_host_address: str = "10.0.0.11/32"
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
    # Stored reference values for explicit/manual encoder commands. Automatic
    # RTSP enable, startup, Auto-Connect, and reactivation never apply them.
    wiris_visible_encoder_bitrate: int = 0
    wiris_visible_encoder_qp: int = 1
    wiris_visible_encoder_gop: int = 90
    wiris_visible_encoder_idr: int = 0
    # WIRIS Enterprise's onboard FTP server, reached over the same host as
    # the TCP command channel above (different port only). Used to retrieve
    # a captured HR/OZ still after the camera reports it stored.
    # Each camera has its own local FTP account (the login is set per-unit on
    # the camera itself), so port/user/password/remote dir/timeout are kept
    # separate per side; only the download toggle is shared.
    wiris_ftp_enabled: bool = True
    wiris_ftp_port: int = 21
    wiris_ftp_user: str = "anonymous"
    wiris_ftp_password: str = ""
    wiris_ftp_remote_dir: str = "/"
    wiris_ftp_timeout_s: float = 30.0
    left_ftp_port: int = 21
    right_ftp_port: int = 21
    left_ftp_user: str = "wiris-enterprise"
    right_ftp_user: str = "wiris-enterprise"
    left_ftp_password: str = "ABCD"
    right_ftp_password: str = "ABCD"
    left_ftp_remote_dir: str = "/"
    right_ftp_remote_dir: str = "/"
    left_ftp_timeout_s: float = 30.0
    right_ftp_timeout_s: float = 30.0
    yolo_device: str = "cpu"
    yolo_input_size: int = 416
    yolo_conf_threshold: float = 0.5
    yolo_iou_threshold: float = 0.45
    yolo_gate_class_enabled: bool = False
    yolo_gate_class_text: str = ""
    yolo_gate_matching_enabled: bool = True
    yolo_gate_time_delta_enabled: bool = True
    yolo_gate_vertical_delta_enabled: bool = False
    yolo_gate_max_vertical_delta_px: float = 80.0
    yolo_gate_max_time_delta_s: float = 0.1
    # Operator preference for live pair selection. App receive is the default
    # and mandatory fallback when preferred RTCP timing is unusable.
    yolo_gate_time_basis: str = "app_receive_monotonic"
    preview_width: int = 1280
    calibration_zoom_level: int = 10
    calibration_inner_cols: int = 13
    calibration_inner_rows: int = 8
    calibration_square_size_mm: float = 40.0
    calibration_min_images: int = 20
    # Maximum number of matching stereo pairs used by the full calibration
    # calculation.  Zero means use every pair in the selected root.
    calibration_calculation_pair_count: int = 20
    # Maximum absolute RTCP/NTP exposure-time delta accepted for one
    # calibration stereo pair.  The Calibration UI presents this in ms.
    calibration_pair_max_delta_s: float = 0.100
    calibration_auto_interval_s: float = 1.0
    calibration_root: str = DEFAULT_CALIBRATION_ROOT
    measurement_data_root: str = DEFAULT_MEASUREMENT_DATA_ROOT
    calibration_validation_mode: str = "pixhawk"
    calibration_validation_root: str = DEFAULT_CALIBRATION_ROOT
    calibration_validation_json: str = DEFAULT_INTRINSICS_PATH
    calibration_validation_runs: int = 3
    calibration_validation_pair_count: int = 0
    calibration_validation_seed: int = 0
    rtsp_latency_ms: int = 100
    # Compressed H.264 queue policy is a Workstation-only tradeoff. It never
    # changes WIRIS settings or the tolerant decoded-frame admission contract.
    rtsp_encoded_queue_mode: str = RTSP_ENCODED_QUEUE_MODE_LATEST
    yolo_event_queue_size: int = 128
    yolo_frame_queue_size: int = 2
    rtsp_decoder: str = "auto"
    stream_backend: str = "gstreamer"
    # A failed camera child exits and waits for an operator/controller-owned
    # reconnect. "automatic" restores the former nested retry behavior.
    rtsp_reconnect_policy: str = "manual"
    # Interleaved TCP avoids the unrecoverable H.264 gaps seen when UDP loses
    # SPS/PPS or keyframe packets. Operators can still opt into UDP explicitly.
    rtsp_transport: str = "tcp"
    camera_sync_history_window_s: float = 1.0
    camera_sync_receive_pair_max_delta_s: float = 0.350
    camera_sync_rtcp_pair_max_delta_s: float = 0.100
    camera_sync_max_sr_age_s: float = 7.0
    camera_sync_max_pc_offset_s: float = 0.100
    camera_sync_max_clock_rate_error: float = 0.005
    camera_sync_required_sr_count: int = 2
    camera_sync_warmup_timeout_s: float = 30.0
    camera_sync_mode: str = STUS_MODE_MANUAL
    camera_sync_interval_s: float = STUS_DEFAULT_INTERVAL_S
    pixhawk_host_address: str = "10.0.0.10"
    # Versioned, one-time transport migration. Radio selection never implies
    # opening or assigning a serial device; explicit Connect owns that step.
    pixhawk_transport_schema_version: int = PIXHAWK_TRANSPORT_SCHEMA_VERSION
    left_pixhawk_transport: str = PIXHAWK_TRANSPORT_RADIO
    right_pixhawk_transport: str = PIXHAWK_TRANSPORT_RADIO
    drone_pixhawk_transport: str = PIXHAWK_TRANSPORT_RADIO
    left_pixhawk_radio_device: str = ""
    right_pixhawk_radio_device: str = ""
    drone_pixhawk_radio_device: str = ""
    left_pixhawk_radio_usb_serial: str = ""
    right_pixhawk_radio_usb_serial: str = ""
    drone_pixhawk_radio_usb_serial: str = ""
    left_pixhawk_radio_baud: int = 57600
    right_pixhawk_radio_baud: int = 57600
    drone_pixhawk_radio_baud: int = 57600
    left_pixhawk_eth_ip: str = "10.0.0.2"
    right_pixhawk_eth_ip: str = "10.0.0.3"
    drone_pixhawk_eth_ip: str = ""
    left_mavlink_url: str = "udpin:0.0.0.0:14550"
    right_mavlink_url: str = "udpin:0.0.0.0:14551"
    left_mavlink_baud: int = 57600
    right_mavlink_baud: int = 57600
    # Machine-local operator assignments.  Interface names are deliberately
    # persisted as device contracts, not portable asset paths.  A replacement
    # NIC with a different name must be reassigned from Find network.
    left_mavlink_interface: str = ""
    right_mavlink_interface: str = ""
    drone_mavlink_interface: str = ""
    left_mavlink_local_address: str = ""
    right_mavlink_local_address: str = ""
    drone_mavlink_local_address: str = ""
    camera_network_stable_id: str = ""
    left_mavlink_interface_stable_id: str = ""
    right_mavlink_interface_stable_id: str = ""
    drone_mavlink_interface_stable_id: str = ""
    network_role_owned_entries: dict[str, dict[str, list[str]]] = field(default_factory=dict)
    # The third Pixhawk link is an active telemetry/correction endpoint for
    # the drone.  It is deliberately separate from the receive-only target
    # drone artifact path.
    drone_telemetry_url: str = "udpin:0.0.0.0:14552"
    drone_telemetry_baud: int = 57600
    drone_telemetry_rtcm_forward: bool = True
    drone_telemetry_rtcm_source: str = "base"
    left_pixhawk_bootstrap_url: str = ""
    right_pixhawk_bootstrap_url: str = ""
    drone_pixhawk_bootstrap_url: str = ""
    left_pixhawk_bootstrap_baud: int = 57600
    right_pixhawk_bootstrap_baud: int = 57600
    drone_pixhawk_bootstrap_baud: int = 57600
    # Deprecated read-compatibility flag. Persistent MAV_2 setup is now an
    # explicit command and this value never triggers work during Connect.
    mavlink_configure_streams: bool = False
    mavlink_stream_rate: int = 2_000_000
    pixhawk_ethernet_mode: str = DEFAULT_PIXHAWK_ETHERNET_MODE
    pixhawk_left_gnss_source: str = "GPS1"
    pixhawk_right_gnss_source: str = "GPS1"
    auto_load_intrinsics: bool = True
    manual_focal_override: bool = False
    use_triangulation: bool = True
    triangulation_extrinsics_source: str = "pixhawk"
    measurement_preferred_method: str = PIXHAWK_RTK_LIVE
    measurement_method_fallback_order: tuple[str, ...] = (
        DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER
    )
    measurement_baseline_yaw_reverse: bool = False
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


def load_settings(
    path: str | Path = DEFAULT_SETTINGS_PATH,
    *,
    create_if_missing: bool = True,
) -> AppSettings:
    settings_path = Path(path)
    read_path = _settings_read_path(settings_path)
    if not read_path.exists():
        settings = AppSettings(gnss_operation=GnssOperationConfig.ngii_rts2_base_assisted())
        if create_if_missing:
            save_settings(settings, settings_path)
        return settings
    payload = json.loads(read_path.read_text())
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
    measurement_preferred_method = normalize_measurement_method_id(
        payload.get("measurement_preferred_method", triangulation_extrinsics_source)
    )
    measurement_method_fallback_order = normalize_measurement_method_fallback_order(
        payload.get(
            "measurement_method_fallback_order",
            DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
        )
    )
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
    intrinsics_path = _default_path_with_legacy_read(
        payload.get("intrinsics_path", AppSettings.intrinsics_path),
        canonical=DEFAULT_INTRINSICS_PATH,
        legacy_markers={"calibration_captures/intrinsics.json"},
        legacy_candidates=(
            LEGACY_CALIBRATION_CAPTURES_ROOT / "intrinsics.json",
            LEGACY_REPOSITORY_DATA_ROOT / "calibration_captures" / "intrinsics.json",
            APP_ROOT / "calibration_captures" / "intrinsics.json",
        ),
    )
    raw_yolo_weights_path = payload.get(
        "yolo_weights_path", AppSettings.yolo_weights_path
    )
    if (
        str(raw_yolo_weights_path or "").strip() == "yolov8n.pt"
        and (APP_ROOT / "yolov8n.pt").is_file()
    ):
        yolo_weights_path = str(APP_ROOT / "yolov8n.pt")
    else:
        yolo_weights_path = _default_path_with_legacy_read(
            raw_yolo_weights_path,
            canonical=DEFAULT_YOLO_WEIGHTS_PATH,
            legacy_markers={".local/models/yolov8m.pt"},
            legacy_candidates=(
                LEGACY_MODELS_ROOT / "yolov8m.pt",
                WORKSTATION_DATA_ROOT / ".local" / "models" / "yolov8m.pt",
            ),
        )
    calibration_root = _default_path_with_legacy_read(
        payload.get("calibration_root", AppSettings.calibration_root),
        canonical=DEFAULT_CALIBRATION_ROOT,
        legacy_markers={"calibration_captures"},
        legacy_candidates=(
            LEGACY_CALIBRATION_CAPTURES_ROOT,
            LEGACY_REPOSITORY_DATA_ROOT / "calibration_captures",
            APP_ROOT / "calibration_captures",
        ),
    )
    calibration_validation_root = _default_path_with_legacy_read(
        payload.get("calibration_validation_root", AppSettings.calibration_validation_root),
        canonical=DEFAULT_CALIBRATION_ROOT,
        legacy_markers={"calibration_captures"},
        legacy_candidates=(
            LEGACY_CALIBRATION_CAPTURES_ROOT,
            LEGACY_REPOSITORY_DATA_ROOT / "calibration_captures",
            APP_ROOT / "calibration_captures",
        ),
    )
    calibration_validation_json = _default_path_with_legacy_read(
        payload.get("calibration_validation_json", AppSettings.calibration_validation_json),
        canonical=DEFAULT_INTRINSICS_PATH,
        legacy_markers={"calibration_captures/intrinsics.json"},
        legacy_candidates=(
            LEGACY_CALIBRATION_CAPTURES_ROOT / "intrinsics.json",
            LEGACY_REPOSITORY_DATA_ROOT / "calibration_captures" / "intrinsics.json",
            APP_ROOT / "calibration_captures" / "intrinsics.json",
        ),
    )
    settings = AppSettings(
        intrinsics_path=intrinsics_path,
        yolo_weights_path=yolo_weights_path,
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
            min(
                2**32 - 1,
                int(payload.get("wiris_visible_encoder_bitrate", AppSettings.wiris_visible_encoder_bitrate)),
            ),
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
            min(
                2**32 - 1,
                int(payload.get("wiris_visible_encoder_idr", AppSettings.wiris_visible_encoder_idr)),
            ),
        ),
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
        yolo_gate_matching_enabled=True,
        yolo_gate_time_delta_enabled=True,
        yolo_gate_vertical_delta_enabled=bool(
            payload.get("yolo_gate_vertical_delta_enabled", AppSettings.yolo_gate_vertical_delta_enabled)
        ),
        yolo_gate_max_vertical_delta_px=float(
            payload.get("yolo_gate_max_vertical_delta_px", AppSettings.yolo_gate_max_vertical_delta_px)
        ),
        yolo_gate_max_time_delta_s=min(
            0.100,
            max(
                0.0,
                float(
                    payload.get(
                        "yolo_gate_max_time_delta_s",
                        AppSettings.yolo_gate_max_time_delta_s,
                    )
                ),
            ),
        ),
        yolo_gate_time_basis=normalize_live_distance_time_basis(
            payload.get("yolo_gate_time_basis", AppSettings.yolo_gate_time_basis)
        ),
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
        calibration_pair_max_delta_s=max(
            0.0,
            min(
                1.0,
                float(
                    payload.get(
                        "calibration_pair_max_delta_s",
                        AppSettings.calibration_pair_max_delta_s,
                    )
                ),
            ),
        ),
        calibration_auto_interval_s=max(
            0.1,
            min(10.0, float(payload.get("calibration_auto_interval_s", AppSettings.calibration_auto_interval_s))),
        ),
        calibration_root=calibration_root,
        measurement_data_root=normalize_measurement_data_root(
            payload.get("measurement_data_root", AppSettings.measurement_data_root)
        ),
        calibration_validation_mode=calibration_validation_mode,
        calibration_validation_root=calibration_validation_root,
        calibration_validation_json=calibration_validation_json,
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
        rtsp_encoded_queue_mode=normalize_rtsp_encoded_queue_mode(
            payload.get(
                "rtsp_encoded_queue_mode",
                AppSettings.rtsp_encoded_queue_mode,
            )
        ),
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
        rtsp_reconnect_policy=str(
            payload.get(
                "rtsp_reconnect_policy",
                AppSettings.rtsp_reconnect_policy,
            )
        ),
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
        camera_sync_mode=normalize_stus_mode(
            payload.get("camera_sync_mode", AppSettings.camera_sync_mode)
        ),
        camera_sync_interval_s=normalize_stus_interval_s(
            payload.get("camera_sync_interval_s", AppSettings.camera_sync_interval_s)
        ),
        pixhawk_host_address=str(payload.get("pixhawk_host_address", AppSettings.pixhawk_host_address)),
        pixhawk_transport_schema_version=PIXHAWK_TRANSPORT_SCHEMA_VERSION,
        left_pixhawk_transport=normalize_pixhawk_transport(
            payload.get("left_pixhawk_transport", PIXHAWK_TRANSPORT_RADIO)
        ),
        right_pixhawk_transport=normalize_pixhawk_transport(
            payload.get("right_pixhawk_transport", PIXHAWK_TRANSPORT_RADIO)
        ),
        drone_pixhawk_transport=normalize_pixhawk_transport(
            payload.get("drone_pixhawk_transport", PIXHAWK_TRANSPORT_RADIO)
        ),
        left_pixhawk_radio_device=str(payload.get("left_pixhawk_radio_device", "")),
        right_pixhawk_radio_device=str(payload.get("right_pixhawk_radio_device", "")),
        drone_pixhawk_radio_device=str(payload.get("drone_pixhawk_radio_device", "")),
        left_pixhawk_radio_usb_serial=str(payload.get("left_pixhawk_radio_usb_serial", "")),
        right_pixhawk_radio_usb_serial=str(payload.get("right_pixhawk_radio_usb_serial", "")),
        drone_pixhawk_radio_usb_serial=str(payload.get("drone_pixhawk_radio_usb_serial", "")),
        left_pixhawk_radio_baud=int(
            payload.get("left_pixhawk_radio_baud", AppSettings.left_pixhawk_radio_baud)
        ),
        right_pixhawk_radio_baud=int(
            payload.get("right_pixhawk_radio_baud", AppSettings.right_pixhawk_radio_baud)
        ),
        drone_pixhawk_radio_baud=int(
            payload.get("drone_pixhawk_radio_baud", AppSettings.drone_pixhawk_radio_baud)
        ),
        left_pixhawk_eth_ip=str(payload.get("left_pixhawk_eth_ip", AppSettings.left_pixhawk_eth_ip)),
        right_pixhawk_eth_ip=str(payload.get("right_pixhawk_eth_ip", AppSettings.right_pixhawk_eth_ip)),
        drone_pixhawk_eth_ip=str(payload.get("drone_pixhawk_eth_ip", AppSettings.drone_pixhawk_eth_ip)),
        left_mavlink_url=str(payload.get("left_mavlink_url", AppSettings.left_mavlink_url)),
        right_mavlink_url=str(payload.get("right_mavlink_url", AppSettings.right_mavlink_url)),
        left_mavlink_baud=int(payload.get("left_mavlink_baud", AppSettings.left_mavlink_baud)),
        right_mavlink_baud=int(payload.get("right_mavlink_baud", AppSettings.right_mavlink_baud)),
        left_mavlink_interface=str(
            payload.get("left_mavlink_interface", AppSettings.left_mavlink_interface)
        ),
        right_mavlink_interface=str(
            payload.get("right_mavlink_interface", AppSettings.right_mavlink_interface)
        ),
        drone_mavlink_interface=str(
            payload.get("drone_mavlink_interface", AppSettings.drone_mavlink_interface)
        ),
        left_mavlink_local_address=str(
            payload.get("left_mavlink_local_address", AppSettings.left_mavlink_local_address)
        ),
        right_mavlink_local_address=str(
            payload.get("right_mavlink_local_address", AppSettings.right_mavlink_local_address)
        ),
        drone_mavlink_local_address=str(
            payload.get("drone_mavlink_local_address", AppSettings.drone_mavlink_local_address)
        ),
        camera_network_stable_id=str(
            payload.get("camera_network_stable_id", AppSettings.camera_network_stable_id)
        ),
        left_mavlink_interface_stable_id=str(
            payload.get(
                "left_mavlink_interface_stable_id",
                AppSettings.left_mavlink_interface_stable_id,
            )
        ),
        right_mavlink_interface_stable_id=str(
            payload.get(
                "right_mavlink_interface_stable_id",
                AppSettings.right_mavlink_interface_stable_id,
            )
        ),
        drone_mavlink_interface_stable_id=str(
            payload.get(
                "drone_mavlink_interface_stable_id",
                AppSettings.drone_mavlink_interface_stable_id,
            )
        ),
        network_role_owned_entries=normalize_network_role_owned_entries(
            payload.get("network_role_owned_entries")
        ),
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
        drone_pixhawk_bootstrap_url=str(
            payload.get("drone_pixhawk_bootstrap_url", AppSettings.drone_pixhawk_bootstrap_url)
        ),
        left_pixhawk_bootstrap_baud=int(
            payload.get("left_pixhawk_bootstrap_baud", AppSettings.left_pixhawk_bootstrap_baud)
        ),
        right_pixhawk_bootstrap_baud=int(
            payload.get("right_pixhawk_bootstrap_baud", AppSettings.right_pixhawk_bootstrap_baud)
        ),
        drone_pixhawk_bootstrap_baud=int(
            payload.get("drone_pixhawk_bootstrap_baud", AppSettings.drone_pixhawk_bootstrap_baud)
        ),
        mavlink_configure_streams=bool(
            payload.get("mavlink_configure_streams", AppSettings.mavlink_configure_streams)
        ),
        mavlink_stream_rate=int(payload.get("mavlink_stream_rate", AppSettings.mavlink_stream_rate)),
        pixhawk_ethernet_mode=pixhawk_ethernet_mode,
        pixhawk_left_gnss_source=normalize_pixhawk_gnss_source(
            payload.get("pixhawk_left_gnss_source", AppSettings.pixhawk_left_gnss_source)
        ),
        pixhawk_right_gnss_source=normalize_pixhawk_gnss_source(
            payload.get("pixhawk_right_gnss_source", AppSettings.pixhawk_right_gnss_source)
        ),
        auto_load_intrinsics=bool(payload.get("auto_load_intrinsics", True)),
        manual_focal_override=bool(payload.get("manual_focal_override", False)),
        use_triangulation=bool(payload.get("use_triangulation", True)),
        triangulation_extrinsics_source=triangulation_extrinsics_source,
        measurement_preferred_method=measurement_preferred_method,
        measurement_method_fallback_order=measurement_method_fallback_order,
        measurement_baseline_yaw_reverse=bool(
            payload.get(
                "measurement_baseline_yaw_reverse",
                AppSettings.measurement_baseline_yaw_reverse,
            )
        ),
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
    if (
        create_if_missing
        and read_path == settings_path
        and int(payload.get("pixhawk_transport_schema_version", 0) or 0)
        < PIXHAWK_TRANSPORT_SCHEMA_VERSION
    ):
        save_settings(settings, settings_path)
    return settings


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
