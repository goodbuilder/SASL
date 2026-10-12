"""decoder policy owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

from typing import Any
from .policy import (
    DECODER_AUTO,
    DECODER_FALLBACK_FIRST_FRAME_TIMEOUT_S,
    DECODER_FALLBACK_STALL_TIMEOUT_S,
    DECODER_NVIDIA,
    DECODER_SOFTWARE,
    GST_ERROR_DECODER_HEALTH,
    GST_ERROR_PIPELINE,
    GST_ERROR_RECORDING_OUTPUT,
    GST_ERROR_RTSP_SOURCE,
    NVIDIA_H264_DECODER,
    SOFTWARE_H264_DECODER,
)


def normalize_decoder_mode(decoder: str | None) -> str:
    value = (decoder or DECODER_AUTO).strip().lower()
    if value in {DECODER_AUTO, DECODER_NVIDIA, DECODER_SOFTWARE}:
        return value
    if value in {NVIDIA_H264_DECODER, "nvdec", "cuda"}:
        return DECODER_NVIDIA
    if value in {SOFTWARE_H264_DECODER, "ffmpeg", "cpu"}:
        return DECODER_SOFTWARE
    return DECODER_AUTO


def decoder_candidates(decoder: str | None) -> tuple[str, ...]:
    mode = normalize_decoder_mode(decoder)
    if mode == DECODER_NVIDIA:
        return (NVIDIA_H264_DECODER,)
    if mode == DECODER_SOFTWARE:
        return (SOFTWARE_H264_DECODER,)
    return (NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER)


def classify_gstreamer_error(
    *,
    source_name: object,
    detail: object,
    active_decoder: object,
) -> str:
    """Classify a GStreamer failure by the element that owns the boundary.

    Generic pipeline errors are deliberately *not* treated as decoder health.
    Falling back to CPU for an RTSP, parser, mux, or filesink failure merely
    hides the real fault and restarts an otherwise healthy decoder.
    """

    source = str(source_name or "").strip().lower().replace("-", "_")
    message = str(detail or "").strip().lower()
    decoder = str(active_decoder or "").strip().lower()

    recording_sources = ("filesink", "record_mux", "mp4mux", "qtmux")
    if any(token in source for token in recording_sources):
        return GST_ERROR_RECORDING_OUTPUT

    rtsp_sources = (
        "rtspsrc",
        "rtph264depay",
        "h264parse",
        "typefind",
        "tcpclientsrc",
        "udpsrc",
    )
    if source == "src" or any(token in source for token in rtsp_sources):
        return GST_ERROR_RTSP_SOURCE

    decoder_sources = tuple(token for token in (decoder, NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER) if token)
    if any(token in source for token in decoder_sources):
        return GST_ERROR_DECODER_HEALTH

    # parse_launch has no GstMessage source. In that one boundary, an explicit
    # missing/failing decoder element name is still direct decoder evidence.
    if not source and any(token in message for token in decoder_sources):
        return GST_ERROR_DECODER_HEALTH
    if not source and any(token in message for token in recording_sources):
        return GST_ERROR_RECORDING_OUTPUT
    if not source and any(token in message for token in rtsp_sources):
        return GST_ERROR_RTSP_SOURCE
    return GST_ERROR_PIPELINE


def gstreamer_error_fallback_reason(
    *,
    requested_decoder: str,
    active_decoder: str,
    already_fell_back: bool,
    error_class: str,
    detail: object,
) -> str | None:
    """Return an Auto CPU fallback reason only for direct NVDEC health evidence."""

    if error_class != GST_ERROR_DECODER_HEALTH:
        return None
    if normalize_decoder_mode(requested_decoder) != DECODER_AUTO:
        return None
    if active_decoder != NVIDIA_H264_DECODER or already_fell_back:
        return None
    normalized_detail = " ".join(str(detail or "unknown decoder error").split())
    return f"nvh264dec decoder-health error: {normalized_detail}"


def _gstreamer_message_source_name(message: object) -> str:
    source = getattr(message, "src", None)
    getter = getattr(source, "get_name", None)
    if callable(getter):
        try:
            return str(getter() or "")
        except Exception:
            return ""
    return str(source or "")


def _requested_decoder_mode(config: dict[str, Any]) -> str:
    return normalize_decoder_mode(str(config.get("_requested_decoder_mode", config.get("decoder", DECODER_AUTO))))


def runtime_decoder_fallback_reason(
    *,
    requested_decoder: str,
    active_decoder: str,
    started_s: float,
    last_frame_s: float | None,
    now_s: float,
    already_fell_back: bool,
    rtp_recovery_pending: bool = False,
    last_recovery_s: float | None = None,
    encoded_input_active: bool = True,
) -> str | None:
    if normalize_decoder_mode(requested_decoder) != DECODER_AUTO:
        return None
    if rtp_recovery_pending:
        return None
    return nvidia_decoder_health_failure_reason(
        active_decoder=active_decoder,
        started_s=started_s,
        last_frame_s=last_frame_s,
        now_s=now_s,
        already_fell_back=already_fell_back,
        rtp_recovery_pending=rtp_recovery_pending,
        last_recovery_s=last_recovery_s,
        encoded_input_active=encoded_input_active,
    )


def nvidia_decoder_health_failure_reason(
    *,
    active_decoder: str,
    started_s: float,
    last_frame_s: float | None,
    now_s: float,
    already_fell_back: bool,
    rtp_recovery_pending: bool = False,
    last_recovery_s: float | None = None,
    encoded_input_active: bool = True,
) -> str | None:
    if rtp_recovery_pending:
        return None
    if already_fell_back or active_decoder != NVIDIA_H264_DECODER or not encoded_input_active:
        return None
    if last_recovery_s is not None and (last_frame_s is None or float(last_recovery_s) > float(last_frame_s)):
        last_frame_s = float(last_recovery_s)
    if last_frame_s is None:
        if now_s - started_s >= DECODER_FALLBACK_FIRST_FRAME_TIMEOUT_S:
            return "nvh264dec first-frame timeout"
        return None
    if now_s - last_frame_s >= DECODER_FALLBACK_STALL_TIMEOUT_S:
        return "nvh264dec decoded-frame stall"
    return None


def should_suppress_decoded_output(
    *,
    active_decoder: str,
    rtp_discontinuity_latched: bool,
    tolerant_input: bool = False,
) -> bool:
    """Keep decoder concealment frames out unless the YOLO trial opts in."""

    _ = active_decoder
    return bool(rtp_discontinuity_latched) and not bool(tolerant_input)


def decoder_element_text(decoder: str) -> str:
    decoder_element = decoder_candidates(decoder)[0]
    if decoder_element == SOFTWARE_H264_DECODER:
        # Do not skip B-frames: decoded_hz must represent the complete raw
        # stream. Corrupt frames are still rejected at the decoder boundary.
        return f"{SOFTWARE_H264_DECODER} output-corrupt=false"
    return decoder_element
