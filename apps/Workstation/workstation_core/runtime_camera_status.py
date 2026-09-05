"""Pure Workstation camera runtime status formatting.

Timestamps retain their declared source: RTCP/NTP UTC, source-clock seconds,
and host receive time are never treated as interchangeable clocks.
"""

from datetime import datetime, timezone
from dataclasses import dataclass
from math import isfinite
from pathlib import Path
import re


@dataclass(frozen=True)
class DecoderPresentation:
    value_text: str
    detail: str
    severity: str
    gpu_active: bool
    fallback_active: bool


def parse_nvidia_runtime_identity(version_text: object, gpu_information_text: object) -> dict[str, object]:
    version_source = str(version_text or "")
    information_source = str(gpu_information_text or "")
    driver_match = re.search(r"NVRM version:.*?\s(\d+(?:\.\d+)+)\s", version_source)
    values: dict[str, str] = {}
    for line in information_source.splitlines():
        key, separator, value = line.partition(":")
        if separator:
            values[key.strip().lower().replace(" ", "_")] = value.strip()
    model = values.get("model", "")
    driver = "" if driver_match is None else driver_match.group(1)
    return {
        "available": bool(model and driver),
        "model": model,
        "driver_version": driver,
        "gpu_uuid": values.get("gpu_uuid", ""),
        "bus_location": values.get("bus_location", ""),
    }


def read_nvidia_runtime_identity(
    *,
    version_path: Path = Path("/proc/driver/nvidia/version"),
    gpu_root: Path = Path("/proc/driver/nvidia/gpus"),
) -> dict[str, object]:
    try:
        version_text = version_path.read_text(encoding="utf-8")
        information_paths = sorted(gpu_root.glob("*/information"))
        information_text = information_paths[0].read_text(encoding="utf-8") if information_paths else ""
    except OSError as exc:
        return {
            "available": False,
            "model": "",
            "driver_version": "",
            "gpu_uuid": "",
            "bus_location": "",
            "error": str(exc),
        }
    return parse_nvidia_runtime_identity(version_text, information_text)


def decoder_presentation(
    *,
    requested_mode: object,
    active_decoder: object,
    runtime_override: object = None,
    fallback_reason: object = None,
) -> DecoderPresentation:
    """Map decoder intent and observed pipeline evidence to operator text."""

    requested = str(requested_mode or "auto").strip().lower()
    if requested not in {"auto", "nvidia", "software"}:
        requested = "auto"
    active = str(active_decoder or "").strip().lower()
    override = str(runtime_override or "").strip().lower()
    reason = " ".join(str(fallback_reason or "").split())
    if not active:
        return DecoderPresentation(
            value_text="--",
            detail=f"requested={requested}; active decoder evidence 없음",
            severity="unknown",
            gpu_active=False,
            fallback_active=False,
        )
    if active == "nvh264dec":
        return DecoderPresentation(
            value_text="GPU · nvh264dec",
            detail=f"requested={requested}; actual=nvh264dec; NVIDIA NVDEC active",
            severity="ok",
            gpu_active=True,
            fallback_active=False,
        )
    if active == "avdec_h264":
        fallback = requested == "auto" and (override == "software" or bool(reason))
        if fallback:
            suffix = f"; reason={reason}" if reason else ""
            return DecoderPresentation(
                value_text="CPU fallback · avdec_h264",
                detail=f"requested=auto; actual=avdec_h264{suffix}",
                severity="warn",
                gpu_active=False,
                fallback_active=True,
            )
        if requested == "software":
            return DecoderPresentation(
                value_text="CPU · avdec_h264 (selected)",
                detail="requested=software; actual=avdec_h264",
                severity="ok",
                gpu_active=False,
                fallback_active=False,
            )
        return DecoderPresentation(
            value_text="Decoder mismatch · avdec_h264",
            detail=f"requested={requested}; actual=avdec_h264; fallback evidence 없음",
            severity="bad",
            gpu_active=False,
            fallback_active=False,
        )
    return DecoderPresentation(
        value_text=f"Active · {active}",
        detail=f"requested={requested}; actual={active}; unclassified decoder",
        severity="warn",
        gpu_active=False,
        fallback_active=False,
    )


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


def _finite_metric(value: object, *, upper: float | None = None) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not isfinite(number):
        return None
    number = max(0.0, number)
    return min(number, upper) if upper is not None else number


def _metric_text(value: object, *, suffix: str = "", upper: float | None = None) -> str:
    number = _finite_metric(value, upper=upper)
    return "--" if number is None else f"{number:.1f}{suffix}"


def format_camera_input_metrics(camera: str, stats: dict) -> str:
    """Format measured RTP/H.264 input rates for a fixed two-line UI slot."""

    stats = stats if isinstance(stats, dict) else {}
    camera_label = {"left": "CAM L", "right": "CAM R"}.get(camera, str(camera))
    return (
        f"{camera_label} | RTP {_metric_text(stats.get('rtp_packets_hz'), suffix=' pkt/s')}"
        f" | missing {_metric_text(stats.get('rtp_missing_hz'), suffix=' pkt/s')}"
        f" | loss {_metric_text(stats.get('rtp_loss_pct'), suffix='%', upper=100.0)}\n"
        f"      | H.264 AU {_metric_text(stats.get('h264_au_hz'), suffix=' /s')}"
        f" | decoded {_metric_text(stats.get('decoded_hz'), suffix=' Hz')}"
    )


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
    ring_capacity = int(stats.get("native_ring_capacity", 0) or 0)
    ring_ready = int(stats.get("native_ring_ready_slots", 0) or 0)
    ring_pinned = int(stats.get("native_ring_pinned_slots", 0) or 0)
    ring_dropped = int(stats.get("native_ring_dropped", 0) or 0)
    preview_replaced = int(stats.get("preview_replaced", 0) or 0)
    metadata_dropped = int(stats.get("metadata_dropped", 0) or 0)
    gui_age = stats.get("gui_preview_age_s")
    try:
        gui_age_ms = max(0.0, float(gui_age)) * 1000.0
    except (TypeError, ValueError):
        gui_age_ms = 0.0
    flow_text = ""
    if ring_capacity > 0:
        flow_text = (
            f" | Ring {ring_ready}/{ring_capacity} P{ring_pinned} D{ring_dropped}"
            f" | GUI {gui_age_ms:.0f}ms R{preview_replaced}"
            f" Mdrop{metadata_dropped}"
        )
    return (
        f"{label}: {stream_hz_text} | "
        f"{yolo_hz_text}{format_runtime_yolo_device_text(stats)}"
        f"{flow_text}{state_text}"
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


def is_sr_pc_diagnostic_reason(reason: object) -> bool:
    """Return whether a legacy invalid marker is now observation-only."""

    normalized = (
        str(reason or "")
        .strip()
        .lower()
        .replace("_", "-")
        .replace("/", "-")
        .replace(" ", "-")
    )
    return normalized.startswith("sr-pc")


def infer_runtime_camera_timestamp_reference(metadata: dict) -> str:
    reference = (
        str(metadata.get("timestamp_reference") or "")
        .strip()
        .lower()
        .replace("_", "-")
        .replace("/", "-")
        .replace(" ", "-")
    )
    invalid_reason = metadata.get("rtcp_timing_invalid_reason")
    if reference and not (
        reference == "rtcp-invalid" and is_sr_pc_diagnostic_reason(invalid_reason)
    ):
        return reference
    if invalid_reason and not is_sr_pc_diagnostic_reason(invalid_reason):
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
        reason = str(metadata.get("rtcp_timing_invalid_reason") or "mapping")
        return f"RTCP invalid ({reason})"
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
