"""frame output owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import time
from typing import Any
from Workstation.workstation_core.error_throttle import should_emit_throttled_error
from Workstation.workstation_core.native_frame_ring import SharedNativeFrameRing
from Workstation.workstation_core.runtime_camera_status import is_sr_pc_diagnostic_reason
from .events import _emit_status


def app_receive_monotonic_sample() -> tuple[int, float]:
    """Capture one exact host-monotonic decoded-frame receipt timestamp."""

    monotonic_ns = time.monotonic_ns()
    return monotonic_ns, monotonic_ns * 1.0e-9


def _should_emit_preview_frame(config: dict[str, Any], timestamp_s: float) -> bool:
    """Rate-limit UI preview frames while keeping capture/detection independent."""

    try:
        display_rate_hz = float(config.get("display_rate_hz", 0.0) or 0.0)
    except (TypeError, ValueError):
        display_rate_hz = 0.0
    if display_rate_hz <= 0.0:
        return True

    current_s = float(timestamp_s)
    last_s = config.get("_last_preview_emit_s")
    try:
        elapsed_s = current_s - float(last_s) if last_s is not None else float("inf")
    except (TypeError, ValueError):
        elapsed_s = float("inf")
    if elapsed_s >= 0.0 and elapsed_s < 1.0 / display_rate_hz:
        return False
    config["_last_preview_emit_s"] = current_s
    return True


def _publish_native_frame_to_ring(
    frame,
    config: dict[str, Any],
    output_queue,
    *,
    timestamp_s: float,
    source_timestamp_s: float | None,
) -> dict[str, Any]:
    ring = config.get("_native_frame_ring")
    if not isinstance(ring, SharedNativeFrameRing):
        return {}
    try:
        descriptor = ring.publish_frame(
            frame,
            timestamp_s=timestamp_s,
            source_timestamp_s=source_timestamp_s,
        )
    except Exception as exc:
        throttle = config.setdefault("_native_ring_error_throttle", {})
        if should_emit_throttled_error(
            throttle,
            "native-ring-publish",
            str(exc),
            float(timestamp_s),
        ):
            _emit_status(output_queue, f"NATIVE_RING publish 실패: {exc}")
        return {
            "native_ring_available": False,
            "native_ring_unavailable_reason": f"publish-error:{type(exc).__name__}",
        }
    metadata = descriptor.to_mapping()
    metadata.update(
        {
            "source_native_frame_id": descriptor.key.text,
            "native_lossless_source": True,
            "overlay_applied": False,
        }
    )
    if not descriptor.available:
        throttle = config.setdefault("_native_ring_full_throttle", {})
        if should_emit_throttled_error(
            throttle,
            "native-ring-unavailable",
            descriptor.unavailable_reason,
            float(timestamp_s),
        ):
            _emit_status(
                output_queue,
                "BACKPRESSURE/DEGRADED: native ring frame unavailable "
                f"frame={descriptor.key.text}, reason={descriptor.unavailable_reason}",
            )
    return metadata


def _emit_preview_frame(
    cv2,
    frame,
    config: dict[str, Any],
    output_queue,
    *,
    timestamp_s: float,
    source_timestamp_s: float | None = None,
    timestamp_source: str = "app-receive",
    extra_metadata: dict[str, Any] | None = None,
    force_emit: bool = False,
) -> bool:
    native_height, native_width = frame.shape[:2]
    if not force_emit and not _should_emit_preview_frame(config, timestamp_s):
        return False
    display_frame = frame
    max_preview_width = int(config["preview_max_width"])
    if max_preview_width > 0 and display_frame.shape[1] > max_preview_width:
        scale = max_preview_width / float(display_frame.shape[1])
        display_frame = cv2.resize(
            display_frame,
            (max_preview_width, max(1, int(round(display_frame.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA,
        )
    rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
    metadata = {
        "timestamp_s": float(timestamp_s),
        "source_timestamp_s": None if source_timestamp_s is None else float(source_timestamp_s),
        "timestamp_source": timestamp_source,
        "native_width": int(native_width),
        "native_height": int(native_height),
        "preview_width": int(rgb.shape[1]),
        "preview_height": int(rgb.shape[0]),
    }
    if extra_metadata:
        metadata.update(extra_metadata)
    metadata.setdefault(
        "timestamp_reference",
        "rtcp-invalid"
        if metadata.get("rtcp_timing_invalid_reason")
        and not is_sr_pc_diagnostic_reason(metadata.get("rtcp_timing_invalid_reason"))
        else "rtcp-ntp"
        if metadata.get("rtcp_frame_ntp_unix_s") is not None
        else "source"
        if metadata.get("source_timestamp_s") is not None
        else "app-receive",
    )
    output_queue.put(("frame", metadata, rgb.shape[1], rgb.shape[0], rgb.tobytes()))
    return True


def _emit_measurement_frame(
    cv2,
    frame,
    config: dict[str, Any],
    output_queue,
    *,
    timestamp_s: float,
    timestamp_metadata: dict[str, Any] | None = None,
) -> None:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    metadata = dict(timestamp_metadata or {})
    metadata.update(
        {
            "timestamp_s": float(timestamp_s),
            "native_width": int(rgb.shape[1]),
            "native_height": int(rgb.shape[0]),
            "native_lossless_source": True,
            "overlay_applied": False,
            "source_native_frame_id": (f"decoded-{int(round(float(timestamp_s) * 1_000_000_000.0)):019d}"),
        }
    )
    output_queue.put(
        (
            "measurement_frame",
            metadata,
            rgb.shape[1],
            rgb.shape[0],
            rgb.tobytes(),
        )
    )


def _emit_stream_status(
    output_queue, enable_detection: bool, best_detection, now_s: float, last_status_s: float
) -> float:
    if now_s - last_status_s < 1.0:
        return last_status_s
    if not enable_detection:
        _emit_status(output_queue, "카메라 스트림 실행 중")
    elif best_detection is None:
        _emit_status(output_queue, "YOLO 실행 중: 검출 없음")
    else:
        center_x, center_y = best_detection.center
        class_label = best_detection.class_name or f"class {best_detection.class_id}"
        _emit_status(
            output_queue,
            "YOLO 실행 중: "
            f"최고 class={class_label} "
            f"신뢰도={best_detection.confidence:.2f} "
            f"중심=({center_x:.1f}, {center_y:.1f}) px",
        )
    return now_s
