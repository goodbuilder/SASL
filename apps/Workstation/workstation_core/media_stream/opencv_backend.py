"""opencv backend owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Any
from .configuration import (
    _record_video_path,
    _resolve_frame_timeout_s,
    normalize_rtsp_transport,
    normalize_stream_backend,
    recording_integrity_contract,
)
from .events import _emit_status
from .frame_output import (
    _emit_measurement_frame,
    _emit_preview_frame,
    _emit_stream_status,
    _publish_native_frame_to_ring,
    app_receive_monotonic_sample,
)
from .inference import _process_frame_for_output, _yolo_device_stats
from .policy import RTSP_TRANSPORT_UDP, STREAM_BACKEND_OPENCV, STREAM_BACKEND_OPENCV_AUTO
from .rates import AdaptiveYoloRateLimiter, RollingFrameRateEstimator, _activate_cpu_fallback_rate_limiter
from .recorders import _open_record_video_writer, _release_record_video_writer, _write_record_video_frame


def _run_opencv_stream(config: dict[str, Any], output_queue, stop_event, cv2, model, enable_detection: bool) -> None:
    transport = normalize_rtsp_transport(str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)))
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{transport}|stimeout;2000000|max_delay;500000"
    backend = normalize_stream_backend(str(config.get("stream_backend", STREAM_BACKEND_OPENCV)))
    config.update(recording_integrity_contract(backend))
    api_preference = cv2.CAP_ANY if backend == STREAM_BACKEND_OPENCV_AUTO else cv2.CAP_FFMPEG
    backend_name = "OpenCV Auto" if backend == STREAM_BACKEND_OPENCV_AUTO else "OpenCV/FFmpeg"
    cap = cv2.VideoCapture(config["url"], api_preference)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    if not cap.isOpened():
        _emit_status(output_queue, f"{backend_name} 스트림 열기 실패")
        cap.release()
        return

    rate_limiter = AdaptiveYoloRateLimiter.from_config(config, enable_detection=enable_detection)
    next_frame_s = 0.0
    last_status_s = 0.0
    last_stats_s = time.monotonic()
    displayed_frames = 0
    inferred_frames = 0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    latest_lock = threading.Lock()
    latest_frame: dict[str, Any] | None = None
    latest_sequence = 0
    decoded_rate = RollingFrameRateEstimator()
    reader_stop = threading.Event()
    record_writer_state = None
    stream_started_s = time.monotonic()
    frame_timeout_s = _resolve_frame_timeout_s(config)
    last_frame_received_s: float | None = None

    def reader_loop() -> None:
        nonlocal \
            displayed_frames, \
            height, \
            last_frame_received_s, \
            latest_frame, \
            latest_sequence, \
            record_writer_state, \
            width
        while not stop_event.is_set() and not reader_stop.is_set():
            ok, frame = cap.read()
            receive_monotonic_ns, receive_s = app_receive_monotonic_sample()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            with latest_lock:
                decoded_rate.add(receive_s)
                last_frame_received_s = time.monotonic()
            frame_height, frame_width = frame.shape[:2]
            if _record_video_path(config):
                if record_writer_state is None:
                    record_writer_state = _open_record_video_writer(
                        cv2, config, output_queue, frame_width, frame_height
                    )
                _write_record_video_frame(cv2, record_writer_state, frame)
            native_metadata = _publish_native_frame_to_ring(
                frame,
                config,
                output_queue,
                timestamp_s=receive_s,
                source_timestamp_s=None,
            )
            native_metadata.update(
                {
                    "app_receive_monotonic_ns": receive_monotonic_ns,
                    "app_receive_timestamp_role": "authoritative_distance",
                }
            )
            preview_emitted = _emit_preview_frame(
                cv2,
                frame,
                config,
                output_queue,
                timestamp_s=receive_s,
                timestamp_source=f"{backend}-read",
                extra_metadata=native_metadata,
            )
            if config.get("emit_measurement_frames", False):
                _emit_measurement_frame(
                    cv2,
                    frame,
                    config,
                    output_queue,
                    timestamp_s=receive_s,
                    timestamp_metadata=native_metadata,
                )
            with latest_lock:
                latest_sequence += 1
                displayed_frames += int(preview_emitted)
                width = frame_width
                height = frame_height
                latest_frame = {
                    "sequence": latest_sequence,
                    "frame": frame,
                    "timestamp_s": receive_s,
                    "native_metadata": native_metadata,
                    "width": frame_width,
                    "height": frame_height,
                }

    reader = threading.Thread(target=reader_loop, name=f"stereo-{backend}-reader", daemon=True)
    reader.start()
    status = f"{backend_name} YOLO 실행 중" if enable_detection else f"{backend_name} 카메라 스트림 실행 중"
    if _record_video_path(config):
        status += ": UNVERIFIED_REENCODE (decoded frame->MP4; RTP/AU 무결성 보장 없음)"
    else:
        status += ": UNVERIFIED_DECODE (RTP/AU 무결성 게이트 없음)"
    _emit_status(output_queue, status)
    last_processed_sequence = 0
    try:
        while not stop_event.is_set():
            now_s = time.monotonic()
            if now_s - last_stats_s >= 1.0:
                elapsed_s = now_s - last_stats_s
                with latest_lock:
                    interval_displayed_frames = displayed_frames
                    displayed_frames = 0
                    stats_width = width
                    stats_height = height
                    decoded_hz = decoded_rate.rate(now_s)
                displayed_hz = interval_displayed_frames / elapsed_s
                inferred_hz = inferred_frames / elapsed_s
                output_queue.put(
                    (
                        "stats",
                        decoded_hz,
                        displayed_hz,
                        inferred_hz,
                        stats_width,
                        stats_height,
                        rate_limiter.target_rate_hz,
                        *_yolo_device_stats(config),
                    )
                )
                rate_limiter.update(decoded_hz=decoded_hz, inferred_hz=inferred_hz)
                inferred_frames = 0
                last_stats_s = now_s
            with latest_lock:
                frame_record = None if latest_frame is None else dict(latest_frame)
                last_receive_s = last_frame_received_s
            frame_age_s = now_s - (stream_started_s if last_receive_s is None else last_receive_s)
            if frame_age_s >= frame_timeout_s:
                # cap.isOpened() can stay true even when the underlying
                # source has stopped delivering fresh frames (e.g. a dead
                # RTSP session). Bail out so the worker process exits and
                # the supervising controller reconnects, matching the
                # GStreamer backend's stall watchdog.
                phase = "첫 프레임" if last_receive_s is None else "새 프레임"
                _emit_status(
                    output_queue,
                    f"{backend_name} 프레임 타임아웃: {phase}이 {frame_age_s:.1f}s 동안 없습니다.",
                )
                break
            if frame_record is None:
                if now_s - last_status_s > 1.0:
                    _emit_status(output_queue, f"{backend_name} 프레임 수신 대기 중")
                    last_status_s = now_s
                time.sleep(0.02)
                continue
            sequence = int(frame_record["sequence"])
            if sequence == last_processed_sequence:
                time.sleep(0.005)
                continue
            if now_s < next_frame_s:
                last_processed_sequence = sequence
                time.sleep(min(0.02, max(0.0, next_frame_s - now_s)))
                continue
            last_processed_sequence = sequence
            next_frame_s = now_s + rate_limiter.interval_s

            frame = frame_record["frame"]
            detection_timestamp_metadata = {
                "timestamp_s": float(frame_record["timestamp_s"]),
                "source_timestamp_s": None,
                "timestamp_source": f"{backend}-read",
                "timestamp_reference": "app-receive",
            }
            detection_timestamp_metadata.update(dict(frame_record.get("native_metadata") or {}))
            processed_frame, best_detection, inferred = _process_frame_for_output(
                cv2,
                frame,
                model=model,
                config=config,
                output_queue=output_queue,
                now_s=float(frame_record["timestamp_s"]),
                enable_detection=enable_detection,
                timestamp_metadata=detection_timestamp_metadata,
            )
            if processed_frame is None:
                time.sleep(0.2)
                continue
            if config.get("_yolo_cpu_fallback_active"):
                _activate_cpu_fallback_rate_limiter(rate_limiter, config)
            if inferred:
                inferred_frames += 1
            last_status_s = _emit_stream_status(output_queue, enable_detection, best_detection, now_s, last_status_s)

    finally:
        reader_stop.set()
        reader.join(timeout=1.0)
        _release_record_video_writer(output_queue, record_writer_state)
        cap.release()
        _emit_status(output_queue, "중지됨")
