"""Pure Workstation camera runtime status formatting.

Timestamps retain their declared source: RTCP/NTP UTC, source-clock seconds,
and host receive time are never treated as interchangeable clocks.
"""

from datetime import datetime, timezone
from math import isfinite


def camera_resolution_text(stats: dict) -> str:
    try:
        width = int(stats.get("width", 0))
        height = int(stats.get("height", 0))
    except (TypeError, ValueError):
        return ""
    if width <= 0 or height <= 0:
        return ""
    return f" @ {width}x{height}"


def camera_display_name(camera: str) -> str:
    return {"left": "왼쪽", "right": "오른쪽"}.get(camera, camera)


def format_camera_stream_stats(camera: str, state: str, stats: dict) -> str:
    camera_name = camera_display_name(camera)
    state_text = {"connected": "연결", "stale": "지연", "waiting": "대기"}.get(state, state)
    decoded_hz = float(stats.get("decoded_hz", 0.0))
    inferred_hz = float(stats.get("inferred_hz", 0.0))
    resolution = camera_resolution_text(stats)
    resolution_text = resolution.removeprefix(" @ ")
    if stats.get("enable_detection"):
        receive_rate_text = f"{resolution_text} 수신 {decoded_hz:.1f} Hz".strip()
        return f"{camera_name}={state_text} {receive_rate_text}, YOLO {inferred_hz:.1f} Hz"
    receive_rate_text = f"{resolution_text} 수신 {decoded_hz:.1f} Hz".strip()
    return f"{camera_name}={state_text} {receive_rate_text}"


def format_runtime_camera_stats(camera: str, stats: dict, fallback_hz: float, *, stale: bool) -> str:
    label = camera_display_name(camera)
    resolution = camera_resolution_text(stats) or " @ --"
    has_stats = bool(stats)
    decoded_hz = float(stats.get("decoded_hz", 0.0)) if has_stats else 0.0
    if has_stats:
        stream_hz_text = f"{resolution.removeprefix(' @ ')} {decoded_hz:.1f} Hz".strip()
        yolo_hz_text = f"{float(stats.get('inferred_hz', 0.0)):.1f} Hz"
    else:
        stream_hz_text = resolution.removeprefix(" @ ") or "--"
        yolo_hz_text = "--"
    state_text = " | 지연" if stale else ""
    return (
        f"{label}: {stream_hz_text} | "
        f"{yolo_hz_text}{format_runtime_yolo_device_text(stats)}{state_text}"
    )


def format_runtime_camera_status(camera: str, stats: dict, metadata: dict, fallback_hz: float, *, stale: bool) -> str:
    return f"{format_runtime_camera_stats(camera, stats, fallback_hz, stale=stale)} | 시각: {format_runtime_camera_timestamp(metadata)}"


def format_runtime_yolo_device_text(stats: dict) -> str:
    if not stats or not stats.get("enable_detection"):
        return ""
    actual_device = str(stats.get("yolo_actual_device") or "").strip()
    if not actual_device:
        return " (장치 대기)"
    normalized = actual_device.lower()
    if normalized == "cpu":
        return " (CPU fallback)" if stats.get("yolo_cpu_fallback") else " (CPU)"
    if normalized.startswith("cuda"):
        return " (CUDA)"
    return f" ({actual_device})"


def infer_runtime_camera_timestamp_reference(metadata: dict) -> str:
    reference = (
        str(metadata.get("timestamp_reference") or "")
        .strip()
        .lower()
        .replace("_", "-")
        .replace("/", "-")
        .replace(" ", "-")
    )
    if reference:
        return reference
    if metadata.get("rtcp_timing_invalid_reason"):
        return "rtcp-invalid"
    if metadata.get("rtcp_frame_ntp_unix_s") is not None:
        return "rtcp-ntp"
    if metadata.get("source_timestamp_s") is not None:
        return "source"
    return "app-receive"


def format_runtime_camera_timestamp(metadata: dict) -> str:
    frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
    timestamp_reference = infer_runtime_camera_timestamp_reference(metadata)
    if timestamp_reference == "rtcp-invalid":
        return "RTCP invalid (SR-PC offset)"
    if frame_ntp_unix_s is not None:
        try:
            utc_text = datetime.fromtimestamp(float(frame_ntp_unix_s), tz=timezone.utc).isoformat(timespec="microseconds")
        except (TypeError, ValueError, OverflowError, OSError):
            utc_text = ""
        if utc_text:
            if timestamp_reference == "gnss-utc":
                return f"UTC {utc_text.replace('+00:00', 'Z')}"
            return f"RTCP NTP {utc_text.replace('+00:00', 'Z')}"
    source_timestamp_s = metadata.get("source_timestamp_s")
    if source_timestamp_s is not None:
        source_source = str(metadata.get("timestamp_source") or "source")
        return f"src {float(source_timestamp_s):.6f} s [{source_source}]"
    return "missing"


def format_runtime_camera_timestamp_compact(metadata: dict) -> str:
    frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
    timestamp_reference = infer_runtime_camera_timestamp_reference(metadata)
    if timestamp_reference == "rtcp-invalid":
        return "RTCP invalid"
    if frame_ntp_unix_s is not None:
        try:
            utc_dt = datetime.fromtimestamp(float(frame_ntp_unix_s), tz=timezone.utc)
        except (TypeError, ValueError, OverflowError, OSError):
            utc_dt = None
        if utc_dt is not None:
            prefix = "UTC" if timestamp_reference == "gnss-utc" else "RTCP"
            return f"{prefix} {utc_dt.strftime('%H:%M:%S.%f')[:-3]}Z"
    source_timestamp_s = metadata.get("source_timestamp_s")
    if source_timestamp_s is not None:
        source_source = str(metadata.get("timestamp_source") or "source")
        return f"src {float(source_timestamp_s):.3f}s [{source_source}]"
    return "missing"


def format_unix_timestamp_compact(value: float | None) -> str:
    if value is None:
        return "--"
    try:
        utc_dt = datetime.fromtimestamp(float(value), tz=timezone.utc)
    except (TypeError, ValueError, OverflowError, OSError):
        return "invalid"
    return f"{utc_dt.strftime('%H:%M:%S.%f')[:-3]}Z"


def format_runtime_camera_ntp_compact(metadata: dict) -> str:
    return format_unix_timestamp_compact(metadata.get("rtcp_frame_ntp_unix_s"))


def format_signed_ms_compact(value_s: object) -> str:
    try:
        value_s = float(value_s)
    except (TypeError, ValueError):
        return "--"
    if not isfinite(value_s):
        return "--"
    return f"{value_s * 1000.0:+.1f} ms"
