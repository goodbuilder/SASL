"""configuration owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import math
from typing import Any
from .policy import (
    DEFAULT_CPU_YOLO_MAX_RATE_HZ,
    DEFAULT_CUDA_YOLO_MAX_RATE_HZ,
    DEFAULT_RECORD_VIDEO_QUEUE_SIZE,
    DEFAULT_RTSP_FRAME_TIMEOUT_S,
    RECORDING_ENCODING_DECODED_NVENC_LOSSLESS,
    RECORDING_ENCODING_DECODED_REENCODE_UNVERIFIED,
    RTSP_RECONNECT_AUTOMATIC,
    RTSP_RECONNECT_MANUAL,
    RTSP_TRANSPORT_TCP,
    RTSP_TRANSPORT_UDP,
    STREAM_BACKEND_GSTREAMER,
    STREAM_BACKEND_OPENCV,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
    YOLO_INPUT_SIZE_STRIDE,
)


def _record_video_path(config: dict[str, Any]) -> str:
    return str(config.get("record_video_path") or "").strip()


def _record_video_fps(config: dict[str, Any]) -> float:
    try:
        fps = float(config.get("record_video_fps") or 30.0)
    except (TypeError, ValueError):
        fps = 30.0
    return max(1.0, fps)


def _record_video_queue_size(config: dict[str, Any]) -> int:
    try:
        size = int(config.get("record_video_queue_size") or DEFAULT_RECORD_VIDEO_QUEUE_SIZE)
    except (TypeError, ValueError):
        size = DEFAULT_RECORD_VIDEO_QUEUE_SIZE
    return max(1, size)


def normalize_stream_backend(backend: str | None) -> str:
    value = (backend or STREAM_BACKEND_GSTREAMER).strip().lower()
    if value in {STREAM_BACKEND_GSTREAMER, "gst"}:
        return STREAM_BACKEND_GSTREAMER
    if value in {STREAM_BACKEND_OPENCV, "ffmpeg", "opencv_ffmpeg", "cv2"}:
        return STREAM_BACKEND_OPENCV
    if value in {STREAM_BACKEND_OPENCV_AUTO, "opencv-auto", "opencv_any", "opencv-any", "cap_any"}:
        return STREAM_BACKEND_OPENCV_AUTO
    if value in {STREAM_BACKEND_PYAV, "av"}:
        return STREAM_BACKEND_PYAV
    return STREAM_BACKEND_GSTREAMER


def recording_integrity_contract(backend: str | None) -> dict[str, str | bool]:
    normalized_backend = normalize_stream_backend(backend)
    if normalized_backend == STREAM_BACKEND_GSTREAMER:
        return {
            "recording_encoding_mode": RECORDING_ENCODING_DECODED_NVENC_LOSSLESS,
            "recording_integrity_verified": False,
            "recording_integrity_scope": (
                "decoded-nv12:nvenc-lossless+finalized-mp4;source-media-integrity=tolerant-unverified"
            ),
        }
    return {
        "recording_encoding_mode": RECORDING_ENCODING_DECODED_REENCODE_UNVERIFIED,
        "recording_integrity_verified": False,
        "recording_integrity_scope": "decoded-frames-only",
    }


def normalize_rtsp_transport(transport: str | None) -> str:
    value = (transport or RTSP_TRANSPORT_UDP).strip().lower()
    if value in {RTSP_TRANSPORT_UDP, "udp-mcast", "udp_multicast"}:
        return RTSP_TRANSPORT_UDP
    if value in {RTSP_TRANSPORT_TCP, "interleaved"}:
        return RTSP_TRANSPORT_TCP
    return RTSP_TRANSPORT_UDP


def normalize_rtsp_reconnect_policy(policy: str | None) -> str:
    value = (policy or RTSP_RECONNECT_MANUAL).strip().lower()
    if value in {RTSP_RECONNECT_AUTOMATIC, "auto", "retry"}:
        return RTSP_RECONNECT_AUTOMATIC
    return RTSP_RECONNECT_MANUAL


def _device_uses_cpu(device: str | None) -> bool:
    return not (device or "").strip() or (device or "").strip().lower() == "cpu"


def default_yolo_max_rate_hz(device: str | None) -> float:
    return DEFAULT_CPU_YOLO_MAX_RATE_HZ if _device_uses_cpu(device) else DEFAULT_CUDA_YOLO_MAX_RATE_HZ


def effective_yolo_max_rate_hz(
    max_rate_hz: float | int | str | None, *, device: str | None, enable_detection: bool
) -> float:
    try:
        rate_hz = float(max_rate_hz or 0.0)
    except (TypeError, ValueError):
        rate_hz = 0.0
    if enable_detection and rate_hz <= 0.0:
        return default_yolo_max_rate_hz(device)
    return max(0.0, rate_hz)


def normalize_yolo_input_size(input_size: int | str | None) -> int:
    try:
        value = int(input_size or 0)
    except (TypeError, ValueError):
        value = 640
    value = max(YOLO_INPUT_SIZE_STRIDE, value)
    return ((value + YOLO_INPUT_SIZE_STRIDE - 1) // YOLO_INPUT_SIZE_STRIDE) * YOLO_INPUT_SIZE_STRIDE


def _resolve_frame_timeout_s(config: dict[str, Any]) -> float:
    """Return the configured stall-detection timeout, matching the GStreamer path."""
    try:
        frame_timeout_s = float(config.get("rtsp_frame_timeout_s", DEFAULT_RTSP_FRAME_TIMEOUT_S))
    except (TypeError, ValueError):
        frame_timeout_s = DEFAULT_RTSP_FRAME_TIMEOUT_S
    if not math.isfinite(frame_timeout_s) or frame_timeout_s <= 0.0:
        frame_timeout_s = DEFAULT_RTSP_FRAME_TIMEOUT_S
    return frame_timeout_s
