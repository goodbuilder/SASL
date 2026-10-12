"""gstreamer backend owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any
from Workstation.workstation_core.runtime_camera_status import is_sr_pc_diagnostic_reason
from Workstation.workstation_core.settings import (
    RTSP_ENCODED_QUEUE_MODE_LATEST,
    RTSP_ENCODED_QUEUE_MODE_REFERENCE_SAFE,
    normalize_rtsp_encoded_queue_mode,
)
from .configuration import (
    _record_video_fps,
    _record_video_path,
    normalize_rtsp_reconnect_policy,
    normalize_rtsp_transport,
    recording_integrity_contract,
)
from .decoder_policy import (
    _gstreamer_message_source_name,
    _requested_decoder_mode,
    classify_gstreamer_error,
    decoder_candidates,
    decoder_element_text,
    gstreamer_error_fallback_reason,
    normalize_decoder_mode,
    nvidia_decoder_health_failure_reason,
    runtime_decoder_fallback_reason,
    should_suppress_decoded_output,
)
from .events import _emit_decoder_fallback, _emit_recording_finalized, _emit_status, _emit_terminal_exit
from .frame_output import (
    _emit_measurement_frame,
    _emit_preview_frame,
    _emit_stream_status,
    _publish_native_frame_to_ring,
    app_receive_monotonic_sample,
)
from .inference import _process_frame_for_output, _yolo_device_stats
from .policy import (
    DECODER_AUTO,
    DECODER_NVIDIA,
    DECODER_SOFTWARE,
    DEFAULT_DECODE_RATE_WINDOW_S,
    DEFAULT_RTSP_FRAME_TIMEOUT_S,
    DEFAULT_RTSP_LATENCY_MS,
    GST_DECODED_QUEUE_MAX_BUFFERS,
    GST_ERROR_DECODER_HEALTH,
    GST_ERROR_RECORDING_OUTPUT,
    GST_ERROR_RTSP_SOURCE,
    GST_RTSP_RECONNECT_DELAYS_S,
    H264_RECOVERY_BASIS_IDR,
    H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL,
    INTEGRITY_RECOVERY_TIMEOUT_S,
    NVIDIA_H264_DECODER,
    RTSP_RECONNECT_AUTOMATIC,
    RTSP_RECONNECT_MANUAL,
    RTSP_TRANSPORT_UDP,
    SOFTWARE_H264_DECODER,
    STREAM_BACKEND_GSTREAMER,
)
from .rates import (
    AdaptiveYoloRateLimiter,
    RollingCounterRateEstimator,
    RollingFrameRateEstimator,
    _activate_cpu_fallback_rate_limiter,
)
from .recorders import DecodedNv12Recorder
from .rtp_timing import GstRtpTimingTracker, RtpDiscontinuityReporter, format_rtp_sequence_discontinuity


def _gst_location(value: str) -> str:
    # Gst.parse_launch() does not apply POSIX shell quoting.  Quote the value
    # as a GStreamer string so spaces and apostrophes remain part of the path.
    path = str(Path(value).expanduser())
    escaped = path.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def build_gst_pipeline_text(
    url: str,
    *,
    latency_ms: int = DEFAULT_RTSP_LATENCY_MS,
    decoder: str = NVIDIA_H264_DECODER,
    transport: str = RTSP_TRANSPORT_UDP,
    keyframe_only: bool = False,
    record_video_path: str = "",
    encoded_queue_mode: str = RTSP_ENCODED_QUEUE_MODE_LATEST,
) -> str:
    latency_ms = max(0, int(latency_ms))
    rtsp_transport = normalize_rtsp_transport(transport)
    decoder_element = decoder_element_text(decoder)
    record_path = str(record_video_path or "").strip()
    # The permanent camera contract is latest-only decoded input. RTP/IDR
    # integrity remains observable, but it never backpressures Preview, YOLO,
    # native evidence, or Measurement.
    h264parse_element = (
        "h264parse config-interval=-1 disable-passthrough=true name=parse "
        "! video/x-h264,stream-format=byte-stream,alignment=au"
    )
    if normalize_rtsp_encoded_queue_mode(encoded_queue_mode) == (RTSP_ENCODED_QUEUE_MODE_REFERENCE_SAFE):
        encoded_queue = "queue max-size-buffers=8 max-size-bytes=0 max-size-time=0"
    else:
        encoded_queue = "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0"
    decoded_queue = (
        f"queue leaky=downstream max-size-buffers={GST_DECODED_QUEUE_MAX_BUFFERS} max-size-bytes=0 max-size-time=0"
    )
    keyframe_filter = ""
    if keyframe_only:
        keyframe_filter = (
            "! identity drop-buffer-flags=delta-unit silent=true "
            "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        )
    pipeline = (
        f"rtspsrc name=src location={url} latency={latency_ms} protocols={rtsp_transport} "
        "drop-on-latency=true do-rtcp=true "
        "! rtph264depay name=depay source-info=true "
        f"! {encoded_queue} "
        f"! {h264parse_element} "
        f"{keyframe_filter}"
        f"! {encoded_queue} "
        f"! {decoder_element} "
        "! tee name=decoded "
        f"decoded. ! {decoded_queue} "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true"
    )
    if record_path:
        pipeline += (
            " decoded. ! queue name=record_capture_queue leaky=downstream "
            "max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            "! videoconvert ! video/x-raw,format=NV12 "
            "! appsink name=record_sink emit-signals=false sync=false max-buffers=2 drop=true"
        )
    return pipeline


def _launch_gst_pipeline(
    Gst,
    url: str,
    *,
    latency_ms: int,
    decoder_mode: str,
    transport: str,
    keyframe_only: bool,
    record_video_path: str,
    output_queue,
    rtp_timing_tracker: GstRtpTimingTracker | None = None,
    encoded_queue_mode: str = RTSP_ENCODED_QUEUE_MODE_LATEST,
):
    last_error: str | None = None
    for decoder in decoder_candidates(decoder_mode):
        pipeline_text = build_gst_pipeline_text(
            url,
            latency_ms=latency_ms,
            decoder=decoder,
            transport=transport,
            keyframe_only=keyframe_only,
            record_video_path=record_video_path,
            encoded_queue_mode=encoded_queue_mode,
        )
        pipeline = None
        try:
            pipeline = Gst.parse_launch(pipeline_text)
            sink = pipeline.get_by_name("sink")
            bus = pipeline.get_bus()
            if sink is None or bus is None:
                raise RuntimeError("pipeline missing appsink or bus")
            if rtp_timing_tracker is not None:
                rtp_timing_tracker.attach(pipeline)
            state_result = pipeline.set_state(Gst.State.PLAYING)
            if state_result == Gst.StateChangeReturn.FAILURE:
                # PLAYING failure alone does not identify the failing element.
                # Do not label it as a decoder fault and hide an RTSP/filesink
                # failure behind an unnecessary CPU retry.
                raise RuntimeError("pipeline failed to enter PLAYING state")
            _emit_status(
                output_queue,
                f"GStreamer decoder candidate: {decoder} (PLAYING requested; current-generation frame pending)",
            )
            if record_video_path:
                _emit_status(
                    output_queue,
                    f"GStreamer decoded NV12 recording capture ready: {record_video_path}",
                )
            return pipeline, sink, bus, decoder
        except Exception as exc:
            last_error = f"{decoder}: {exc}"
            if pipeline is not None:
                pipeline.set_state(Gst.State.NULL)
            error_class = classify_gstreamer_error(
                source_name="",
                detail=exc,
                active_decoder=decoder,
            )
            fallback_reason = gstreamer_error_fallback_reason(
                requested_decoder=decoder_mode,
                active_decoder=decoder,
                already_fell_back=False,
                error_class=error_class,
                detail=exc,
            )
            if fallback_reason is not None:
                _emit_decoder_fallback(
                    output_queue,
                    reason=(f"{fallback_reason}; software candidate pending current-generation frame"),
                    recording_restart_required=False,
                )
                continue
            break
    raise RuntimeError(last_error or "no usable decoder")


def _run_gstreamer_stream_once(
    config: dict[str, Any],
    output_queue,
    stop_event,
    cv2,
    model,
    enable_detection: bool,
) -> bool:
    try:
        import gi
        import numpy as np

        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtp", "1.0")
        from gi.repository import Gst
        from gi.repository import GstRtp
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer 사용 불가: {exc}")
        return False

    Gst.init(None)
    keyframe_only = False
    encoded_queue_mode = normalize_rtsp_encoded_queue_mode(
        config.get("rtsp_encoded_queue_mode", RTSP_ENCODED_QUEUE_MODE_LATEST)
    )
    tolerant_input = True
    record_video_path = _record_video_path(config)
    config["_media_input_policy"] = "tolerant_decoded"
    config["_integrity_recovery_mode"] = "tolerant_decoded"
    rtp_timing_tracker = GstRtpTimingTracker(
        Gst,
        GstRtp,
        status_callback=lambda text: _emit_status(output_queue, text),
        allow_non_idr_intra_experimental=False,
        allow_unverified_encoded_output=True,
    )
    config.update(recording_integrity_contract(STREAM_BACKEND_GSTREAMER))
    config["_integrity_gap_events"] = 0
    config["_integrity_missing_packets"] = 0
    config["_integrity_suppressed_decoded_frames"] = 0
    config.pop("_integrity_manual_reconnect_required", None)
    if record_video_path:
        try:
            Path(record_video_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            _emit_status(output_queue, f"GStreamer MP4 저장 경로 준비 실패: {exc}")
            return False
    stream_failed = False
    launch_decoder_mode = str(config.get("decoder", DECODER_AUTO))
    config.setdefault(
        "_requested_decoder_mode",
        normalize_decoder_mode(launch_decoder_mode),
    )
    try:
        pipeline, sink, bus, active_decoder = _launch_gst_pipeline(
            Gst,
            config["url"],
            latency_ms=int(config.get("rtsp_latency_ms", DEFAULT_RTSP_LATENCY_MS)),
            decoder_mode=launch_decoder_mode,
            transport=str(config.get("rtsp_transport", RTSP_TRANSPORT_UDP)),
            keyframe_only=keyframe_only,
            record_video_path=record_video_path,
            output_queue=output_queue,
            rtp_timing_tracker=rtp_timing_tracker,
            encoded_queue_mode=encoded_queue_mode,
        )
    except Exception as exc:
        _emit_status(output_queue, f"GStreamer 오류: {exc}")
        return False
    if normalize_decoder_mode(launch_decoder_mode) == DECODER_AUTO and active_decoder == SOFTWARE_H264_DECODER:
        # _launch_gst_pipeline authorizes this only for a classified NVDEC
        # launch failure. Keep later reconnects on the proven per-process
        # fallback instead of retrying NVDEC forever.
        config["_runtime_decoder_fallback_active"] = True
        config["decoder"] = DECODER_SOFTWARE

    rate_limiter = AdaptiveYoloRateLimiter.from_config(config, enable_detection=enable_detection)
    next_frame_s = 0.0
    last_status_s = 0.0
    last_stats_s = time.monotonic()
    displayed_frames = 0
    inferred_frames = 0
    latest_lock = threading.Lock()
    latest_frame: dict[str, Any] | None = None
    latest_sequence = 0
    decoded_rate = RollingFrameRateEstimator()
    latest_width = 0
    latest_height = 0
    stream_started_s = time.monotonic()
    rtp_packet_rate = RollingCounterRateEstimator()
    rtp_missing_rate = RollingCounterRateEstimator()
    rtp_gap_event_rate = RollingCounterRateEstimator()
    h264_access_unit_rate = RollingCounterRateEstimator()
    for estimator in (
        rtp_packet_rate,
        rtp_missing_rate,
        rtp_gap_event_rate,
        h264_access_unit_rate,
    ):
        estimator.add(stream_started_s, 0)
    last_frame_received_s: float | None = None
    try:
        frame_timeout_s = float(config.get("rtsp_frame_timeout_s", DEFAULT_RTSP_FRAME_TIMEOUT_S))
    except (TypeError, ValueError):
        frame_timeout_s = DEFAULT_RTSP_FRAME_TIMEOUT_S
    if not math.isfinite(frame_timeout_s) or frame_timeout_s <= 0.0:
        frame_timeout_s = DEFAULT_RTSP_FRAME_TIMEOUT_S
    try:
        integrity_recovery_timeout_s = float(config.get("integrity_recovery_timeout_s", INTEGRITY_RECOVERY_TIMEOUT_S))
    except (TypeError, ValueError):
        integrity_recovery_timeout_s = INTEGRITY_RECOVERY_TIMEOUT_S
    if not math.isfinite(integrity_recovery_timeout_s) or integrity_recovery_timeout_s <= 0.0:
        integrity_recovery_timeout_s = INTEGRITY_RECOVERY_TIMEOUT_S
    config["_gst_last_run_received_frame"] = False
    reader_stop = threading.Event()
    suppressed_decoded_frames = 0
    reported_suppressed_frames = 0
    decoder_frame_announced = False
    rtp_gap_reporter = RtpDiscontinuityReporter()
    record_sink = pipeline.get_by_name("record_sink") if record_video_path else None
    record_reader_stop = threading.Event()
    decoded_recorder: DecodedNv12Recorder | None = None
    recorder_start_error = ""

    def recording_loop() -> None:
        nonlocal decoded_recorder, recorder_start_error
        if record_sink is None:
            recorder_start_error = "record appsink is unavailable"
            return
        while not stop_event.is_set() and not record_reader_stop.is_set():
            sample = record_sink.emit("try-pull-sample", 200_000_000)
            if sample is None:
                continue
            buffer = sample.get_buffer()
            if decoded_recorder is None and not recorder_start_error:
                try:
                    structure = sample.get_caps().get_structure(0)
                    sample_duration_ns = None if buffer.duration == Gst.CLOCK_TIME_NONE else int(buffer.duration)
                    recording_fps = (
                        _record_video_fps(config)
                        if sample_duration_ns is None or sample_duration_ns <= 0
                        else 1_000_000_000.0 / sample_duration_ns
                    )
                    decoded_recorder = DecodedNv12Recorder(
                        Gst,
                        path=record_video_path,
                        width=int(structure.get_value("width")),
                        height=int(structure.get_value("height")),
                        fallback_fps=recording_fps,
                        output_queue=output_queue,
                    )
                except Exception as exc:
                    recorder_start_error = f"{type(exc).__name__}: {exc}"
                    _emit_status(
                        output_queue,
                        f"Decoded NVENC recording unavailable; camera/YOLO continue: {recorder_start_error}",
                    )
                    continue
            if decoded_recorder is not None:
                decoded_recorder.submit(
                    buffer,
                    observed_monotonic_ns=time.monotonic_ns(),
                )

    record_reader = (
        threading.Thread(
            target=recording_loop,
            name="stereo-gst-nv12-recorder",
            daemon=True,
        )
        if record_video_path
        else None
    )
    if record_reader is not None:
        record_reader.start()

    def reader_loop() -> None:
        nonlocal displayed_frames, last_frame_received_s
        nonlocal latest_frame, latest_height, latest_sequence, latest_width
        nonlocal decoder_frame_announced, suppressed_decoded_frames
        while not stop_event.is_set() and not reader_stop.is_set():
            sample = sink.emit("try-pull-sample", 200_000_000)
            if sample is None:
                continue
            receive_monotonic_ns, receive_s = app_receive_monotonic_sample()
            buffer = sample.get_buffer()
            decoded_pts_ns = None if buffer.pts == Gst.CLOCK_TIME_NONE else int(buffer.pts)
            initially_integrity_latched = rtp_timing_tracker.should_suppress_decoded_buffer(decoded_pts_ns)
            if should_suppress_decoded_output(
                active_decoder=active_decoder,
                rtp_discontinuity_latched=initially_integrity_latched,
                tolerant_input=tolerant_input,
            ):
                with latest_lock:
                    latest_frame = None
                suppressed_decoded_frames += 1
                continue
            with latest_lock:
                decoded_rate.add(receive_s)
            receive_wall_s = time.time()
            caps = sample.get_caps().get_structure(0)
            width = int(caps.get_value("width"))
            height = int(caps.get_value("height"))
            source_timestamp_s = None if buffer.pts == Gst.CLOCK_TIME_NONE else buffer.pts / 1e9
            rtp_timing_metadata = (
                rtp_timing_tracker.frame_metadata(
                    buffer,
                    receive_s,
                    received_wall_s=receive_wall_s,
                )
                or {}
            )
            rtp_timing_metadata.update(
                {
                    "app_receive_monotonic_ns": receive_monotonic_ns,
                    "app_receive_timestamp_role": "authoritative_distance",
                    "decoded_buffer_pts_ns": decoded_pts_ns,
                    "active_decoder": active_decoder,
                    "requested_decoder": _requested_decoder_mode(config),
                }
            )
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                continue
            try:
                frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3)).copy()
            finally:
                buffer.unmap(map_info)
            # A gap can be observed by the RTP probe while this buffer is being
            # copied.  Check the latch again before publishing to any consumer.
            after_copy_integrity_latched = rtp_timing_tracker.should_suppress_decoded_buffer(decoded_pts_ns)
            if should_suppress_decoded_output(
                active_decoder=active_decoder,
                rtp_discontinuity_latched=after_copy_integrity_latched,
                tolerant_input=tolerant_input,
            ):
                with latest_lock:
                    latest_frame = None
                suppressed_decoded_frames += 1
                continue
            unverified_tolerant_frame = initially_integrity_latched or after_copy_integrity_latched
            rtp_timing_metadata.update(
                {
                    "media_integrity_mode": "tolerant_decoded",
                    "media_integrity_unverified": unverified_tolerant_frame,
                    "source_media_integrity_verified": (not unverified_tolerant_frame),
                    "rtp_integrity_latched": unverified_tolerant_frame,
                    "media_random_access_guaranteed": (
                        not unverified_tolerant_frame
                        and rtp_timing_tracker.current_output_recovery_basis() == H264_RECOVERY_BASIS_IDR
                    ),
                    "measurement_input_admitted": True,
                    "overlay_applied": False,
                }
            )
            rtp_timing_metadata.update(
                _publish_native_frame_to_ring(
                    frame,
                    config,
                    output_queue,
                    timestamp_s=receive_s,
                    source_timestamp_s=source_timestamp_s,
                )
            )
            preview_emitted = _emit_preview_frame(
                cv2,
                frame,
                config,
                output_queue,
                timestamp_s=receive_s,
                source_timestamp_s=source_timestamp_s,
                timestamp_source="gstreamer-pts",
                extra_metadata=rtp_timing_metadata,
                force_emit=bool(rtp_timing_metadata.get("h264_nal1_non_idr_intra") is True),
            )
            if preview_emitted and not decoder_frame_announced:
                decoder_frame_announced = True
                _emit_status(
                    output_queue,
                    (
                        f"GStreamer decoded frame admitted (INPUT UNVERIFIED): {active_decoder}"
                        if unverified_tolerant_frame
                        else f"GStreamer decoder frame verified: {active_decoder}"
                    ),
                )
            if config.get("emit_measurement_frames", False):
                _emit_measurement_frame(
                    cv2,
                    frame,
                    config,
                    output_queue,
                    timestamp_s=receive_s,
                    timestamp_metadata=rtp_timing_metadata,
                )
            with latest_lock:
                latest_sequence += 1
                displayed_frames += int(preview_emitted)
                latest_width = width
                latest_height = height
                last_frame_received_s = receive_s
                config["_gst_last_run_received_frame"] = True
                latest_frame = {
                    "sequence": latest_sequence,
                    "frame": frame,
                    "timestamp_s": receive_s,
                    "source_timestamp_s": source_timestamp_s,
                    "rtp_timing_metadata": rtp_timing_metadata,
                    "width": width,
                    "height": height,
                }

    reader = threading.Thread(target=reader_loop, name="stereo-gst-reader", daemon=True)
    reader.start()
    last_rtp_recovery_s: float | None = None
    if keyframe_only:
        _emit_status(output_queue, "GStreamer I-frame only 진단 모드")
    recording_text = ", decoded NV12->lossless NVENC MP4" if record_video_path else ""
    encoded_recovery_boundary_text = "RTP/H264 provenance retained; "
    pending_recovery_text = (
        "TOLERANT INPUT: RTP/H264 integrity unverified; decoded Preview/YOLO/Measurement/recording continue "
    )
    _emit_status(
        output_queue,
        "Permanent tolerant decoded input enabled: Preview, YOLO, native "
        "evidence, Measurement, and decoded recording continue while source "
        "media provenance remains INPUT UNVERIFIED"
        f"{recording_text}",
    )
    last_processed_sequence = 0
    try:
        while not stop_event.is_set():
            while True:
                h264_discontinuity = rtp_timing_tracker.pop_h264_access_unit_discontinuity()
                if h264_discontinuity is None:
                    break
                _emit_status(
                    output_queue,
                    "RECOVERING_INTEGRITY: H264 access-unit discontinuity "
                    f"reason={h264_discontinuity.reason}, pts_ns={h264_discontinuity.pts_ns}; "
                    + encoded_recovery_boundary_text
                    + "decoded Preview/YOLO/Measurement/recording continue as INPUT UNVERIFIED",
                )
            while True:
                rtp_recovery = rtp_timing_tracker.pop_rtp_recovery()
                if rtp_recovery is None:
                    break
                last_rtp_recovery_s = time.monotonic()
                newly_suppressed_frames = max(
                    0,
                    suppressed_decoded_frames - reported_suppressed_frames,
                )
                reported_suppressed_frames = suppressed_decoded_frames
                config["_integrity_suppressed_decoded_frames"] = suppressed_decoded_frames
                if rtp_recovery.recovery_basis == H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL:
                    prefix = (
                        "EXPERIMENTAL NON-IDR INTRA: exact decoded PTS에서 출력 복구; "
                        "IDR/keyframe 보장 없음; direct-copy MP4 integrity 미검증 "
                    )
                else:
                    prefix = "HEALTHY: H264 IDR random-access + exact decoded PTS에서 출력 복구 "
                _emit_status(
                    output_queue,
                    prefix + f"ssrc={rtp_recovery.ssrc}, pts_ns={rtp_recovery.resume_pts_ns}, "
                    f"basis={rtp_recovery.recovery_basis}, "
                    f"blocked={rtp_recovery.blocked_duration_s:.3f}s, "
                    f"suppressed={newly_suppressed_frames}",
                )
            while True:
                rtp_discontinuity = rtp_timing_tracker.pop_rtp_discontinuity()
                if rtp_discontinuity is None:
                    break
                now_s = time.monotonic()
                if rtp_gap_reporter.total_events == 0:
                    _emit_status(
                        output_queue,
                        f"RECOVERING_INTEGRITY: {format_rtp_sequence_discontinuity(rtp_discontinuity)}; "
                        + encoded_recovery_boundary_text
                        + "decoded Preview/YOLO/Measurement/recording continue as INPUT UNVERIFIED",
                    )
                summary = rtp_gap_reporter.observe(rtp_discontinuity, now_s=now_s)
                config["_integrity_gap_events"] = rtp_gap_reporter.total_events
                config["_integrity_missing_packets"] = rtp_gap_reporter.total_missing_packets
                if summary is not None:
                    _emit_status(output_queue, f"RECOVERING_INTEGRITY: {summary}")
            message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if message is not None:
                if message.type == Gst.MessageType.ERROR:
                    error, debug = message.parse_error()
                    detail = error.message if debug is None else f"{error.message} ({debug})"
                    stream_failed = True
                    source_name = _gstreamer_message_source_name(message)
                    error_class = classify_gstreamer_error(
                        source_name=source_name,
                        detail=detail,
                        active_decoder=active_decoder,
                    )
                    fallback_reason = gstreamer_error_fallback_reason(
                        requested_decoder=_requested_decoder_mode(config),
                        active_decoder=active_decoder,
                        already_fell_back=bool(config.get("_runtime_decoder_fallback_active")),
                        error_class=error_class,
                        detail=detail,
                    )
                    if fallback_reason is not None:
                        config["_runtime_decoder_fallback_reason"] = fallback_reason
                        _emit_status(
                            output_queue,
                            f"NVIDIA decoder-health failure: source={source_name or '--'}; {detail}",
                        )
                    elif error_class == GST_ERROR_DECODER_HEALTH:
                        config["_decoder_manual_restart_required"] = True
                        if _requested_decoder_mode(config) == DECODER_NVIDIA and active_decoder == NVIDIA_H264_DECODER:
                            _emit_status(
                                output_queue,
                                "NVIDIA 디코더 오류: decoder-health failure; "
                                "software fallback 금지, Auto로 변경하세요. "
                                f"source={source_name or '--'}; {detail}",
                            )
                        else:
                            _emit_status(
                                output_queue,
                                f"디코더 오류: decoder-health failure; source={source_name or '--'}; {detail}",
                            )
                    elif error_class == GST_ERROR_RTSP_SOURCE:
                        _emit_status(
                            output_queue,
                            f"RTSP/source 오류: source={source_name or '--'}; {detail}",
                        )
                    elif error_class == GST_ERROR_RECORDING_OUTPUT:
                        config["_recording_output_failure"] = detail
                        _emit_status(
                            output_queue,
                            f"MP4 mux/filesink 오류: source={source_name or '--'}; {detail}",
                        )
                    else:
                        _emit_status(
                            output_queue,
                            f"GStreamer pipeline 오류: source={source_name or '--'}; {detail}",
                        )
                else:
                    stream_failed = True
                    _emit_status(output_queue, "스트림 종료")
                break

            now_s = time.monotonic()
            integrity_recovery_pending = rtp_timing_tracker.decoded_output_blocked()
            integrity_blocked_s = rtp_timing_tracker.blocked_duration_s(now_s=now_s)
            if integrity_recovery_pending and integrity_blocked_s >= integrity_recovery_timeout_s:
                config["_integrity_suppressed_decoded_frames"] = suppressed_decoded_frames
                summary = rtp_gap_reporter.flush(now_s=now_s)
                if summary is not None:
                    _emit_status(output_queue, f"RECOVERING_INTEGRITY: {summary}")
                if not config.get("_tolerant_integrity_timeout_reported"):
                    _emit_status(
                        output_queue,
                        "TOLERANT INPUT: integrity remained unverified for "
                        f"{integrity_blocked_s:.1f}s; all decoded consumers continue",
                    )
                    config["_tolerant_integrity_timeout_reported"] = True
            with latest_lock:
                last_receive_s = last_frame_received_s
            frame_age_s = now_s - (stream_started_s if last_receive_s is None else last_receive_s)
            recent_input = getattr(rtp_timing_tracker, "has_recent_rtp_input", None)
            try:
                encoded_input_active = (
                    bool(recent_input(now_s=now_s, max_age_s=2.0)) if callable(recent_input) else False
                )
            except (TypeError, ValueError):
                encoded_input_active = False
            fallback_reason = runtime_decoder_fallback_reason(
                requested_decoder=_requested_decoder_mode(config),
                active_decoder=active_decoder,
                started_s=stream_started_s,
                last_frame_s=last_receive_s,
                now_s=now_s,
                already_fell_back=bool(config.get("_runtime_decoder_fallback_active")),
                rtp_recovery_pending=integrity_recovery_pending,
                last_recovery_s=last_rtp_recovery_s,
                encoded_input_active=encoded_input_active,
            )
            if fallback_reason is not None:
                stream_failed = True
                config["_runtime_decoder_fallback_reason"] = fallback_reason
                break
            if _requested_decoder_mode(config) == DECODER_NVIDIA:
                strict_reason = nvidia_decoder_health_failure_reason(
                    active_decoder=active_decoder,
                    started_s=stream_started_s,
                    last_frame_s=last_receive_s,
                    now_s=now_s,
                    already_fell_back=False,
                    rtp_recovery_pending=integrity_recovery_pending,
                    last_recovery_s=last_rtp_recovery_s,
                    encoded_input_active=encoded_input_active,
                )
                if strict_reason is not None:
                    stream_failed = True
                    config["_decoder_manual_restart_required"] = True
                    _emit_status(
                        output_queue,
                        f"NVIDIA 디코더 오류: {strict_reason}; software fallback 금지, Auto로 변경하세요.",
                    )
                    break
            if frame_age_s >= frame_timeout_s:
                stream_failed = True
                phase = "첫 디코드 프레임" if last_receive_s is None else "새 디코드 프레임"
                _emit_status(
                    output_queue,
                    f"RTSP 프레임 타임아웃: {phase}이 {frame_age_s:.1f}s 동안 없습니다.",
                )
                break
            if now_s - last_stats_s >= 1.0:
                elapsed_s = now_s - last_stats_s
                with latest_lock:
                    interval_displayed_frames = displayed_frames
                    displayed_frames = 0
                    stats_width = latest_width
                    stats_height = latest_height
                    decoded_hz = decoded_rate.rate(now_s)
                displayed_hz = interval_displayed_frames / elapsed_s
                inferred_hz = inferred_frames / elapsed_s
                metrics_snapshot_reader = getattr(
                    rtp_timing_tracker,
                    "rtp_metrics_snapshot",
                    None,
                )
                try:
                    transport_snapshot = dict(metrics_snapshot_reader()) if callable(metrics_snapshot_reader) else {}
                except (AttributeError, TypeError, ValueError):
                    transport_snapshot = {}

                def counter_value(key: str) -> int:
                    try:
                        return max(0, int(transport_snapshot.get(key, 0) or 0))
                    except (TypeError, ValueError):
                        return 0

                rtp_packets_total = counter_value("rtp_packets_received_total")
                rtp_missing_total = counter_value("rtp_missing_packets_total")
                rtp_gap_events_total = counter_value("rtp_gap_events_total")
                h264_access_units_total = counter_value("h264_access_units_total")
                rtp_packet_rate.add(now_s, rtp_packets_total)
                rtp_missing_rate.add(now_s, rtp_missing_total)
                rtp_gap_event_rate.add(now_s, rtp_gap_events_total)
                h264_access_unit_rate.add(now_s, h264_access_units_total)
                rtp_packets_hz = rtp_packet_rate.rate(now_s)
                rtp_missing_hz = rtp_missing_rate.rate(now_s)
                rtp_loss_denominator = rtp_packets_hz + rtp_missing_hz
                rtp_loss_pct = 100.0 * rtp_missing_hz / rtp_loss_denominator if rtp_loss_denominator > 0.0 else None
                stream_metrics = {
                    "rtp_packets_hz": rtp_packets_hz,
                    "rtp_missing_hz": rtp_missing_hz,
                    "rtp_loss_pct": rtp_loss_pct,
                    "rtp_gap_events_hz": rtp_gap_event_rate.rate(now_s),
                    "h264_au_hz": h264_access_unit_rate.rate(now_s),
                    "rtp_packets_received_total": rtp_packets_total,
                    "rtp_missing_packets_total": rtp_missing_total,
                    "rtp_gap_events_total": rtp_gap_events_total,
                    "h264_access_units_total": h264_access_units_total,
                    "rtp_active_ssrc": transport_snapshot.get("rtp_active_ssrc"),
                    "rtp_metrics_window_s": DEFAULT_DECODE_RATE_WINDOW_S,
                    "rtp_metrics_available": callable(metrics_snapshot_reader),
                }
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
                        stream_metrics,
                    )
                )
                rate_limiter.update(decoded_hz=decoded_hz, inferred_hz=inferred_hz)
                inferred_frames = 0
                last_stats_s = now_s
            with latest_lock:
                frame_record = None if latest_frame is None else dict(latest_frame)
            if frame_record is None:
                if now_s - last_status_s > 1.0:
                    if integrity_recovery_pending:
                        _emit_status(
                            output_queue,
                            pending_recovery_text + f"({integrity_blocked_s:.1f}/{integrity_recovery_timeout_s:.0f}s)",
                        )
                    else:
                        _emit_status(output_queue, "프레임 수신 대기 중")
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
            source_timestamp_s = frame_record["source_timestamp_s"]
            rtp_timing_metadata = dict(frame_record["rtp_timing_metadata"])
            detection_timestamp_metadata = {
                "timestamp_s": float(frame_record["timestamp_s"]),
                "source_timestamp_s": None if source_timestamp_s is None else float(source_timestamp_s),
                "timestamp_source": "gstreamer-pts",
            }
            detection_timestamp_metadata.update(rtp_timing_metadata)
            detection_timestamp_metadata.setdefault(
                "timestamp_reference",
                "rtcp-invalid"
                if detection_timestamp_metadata.get("rtcp_timing_invalid_reason")
                and not is_sr_pc_diagnostic_reason(detection_timestamp_metadata.get("rtcp_timing_invalid_reason"))
                else "rtcp-ntp"
                if detection_timestamp_metadata.get("rtcp_frame_ntp_unix_s") is not None
                else "source"
                if detection_timestamp_metadata.get("source_timestamp_s") is not None
                else "app-receive",
            )
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
        record_reader_stop.set()
        if record_reader is not None:
            record_reader.join(timeout=1.0)
        config["_integrity_suppressed_decoded_frames"] = suppressed_decoded_frames
        if decoded_recorder is not None:
            _emit_recording_finalized(
                output_queue,
                path=record_video_path,
                backend=STREAM_BACKEND_GSTREAMER,
                success=False,
                detail="decoded NVENC recorder finalization started",
                contract=recording_integrity_contract(STREAM_BACKEND_GSTREAMER),
                metrics={
                    "container_finalization_success": False,
                    "source_media_integrity_verified": False,
                    "recording_file_integrity_verified": False,
                    "integrity_validation_status": "FINALIZING",
                    "integrity_failure_reasons": [],
                    "recording_usable": False,
                },
            )
            decoded_recorder.close()
        elif record_video_path:
            _emit_recording_finalized(
                output_queue,
                path=record_video_path,
                backend=STREAM_BACKEND_GSTREAMER,
                success=False,
                detail=recorder_start_error or "no decoded NV12 recording frame",
                contract=recording_integrity_contract(STREAM_BACKEND_GSTREAMER),
                metrics={
                    "container_finalization_success": False,
                    "source_media_integrity_verified": False,
                    "recording_file_integrity_verified": False,
                    "integrity_validation_status": "FAIL",
                    "integrity_failure_reasons": ["decoded_nvenc_recorder_unavailable"],
                    "recording_usable": False,
                },
            )
        pipeline.set_state(Gst.State.NULL)
        if not stream_failed or stop_event.is_set():
            _emit_status(output_queue, "중지됨")
    return not stream_failed


def _run_gstreamer_stream(config: dict[str, Any], output_queue, stop_event, cv2, model, enable_detection: bool) -> None:
    """Run GStreamer with one explicit owner for any reconnect policy."""

    config.setdefault(
        "_requested_decoder_mode",
        normalize_decoder_mode(str(config.get("decoder", DECODER_AUTO))),
    )
    reconnect_policy = normalize_rtsp_reconnect_policy(
        str(config.get("rtsp_reconnect_policy", RTSP_RECONNECT_AUTOMATIC))
    )
    if reconnect_policy == RTSP_RECONNECT_MANUAL:
        recovered = _run_gstreamer_stream_once(
            config,
            output_queue,
            stop_event,
            cv2,
            model,
            enable_detection,
        )
        integrity_manual_reconnect = bool(config.pop("_integrity_manual_reconnect_required", False))
        if integrity_manual_reconnect and not stop_event.is_set():
            _emit_terminal_exit(
                output_queue,
                "H264 IDR random-access recovery unavailable; manual reconnect required",
            )
            return
        recording_failure = config.pop("_recording_output_failure", None)
        if recording_failure and not stop_event.is_set():
            _emit_status(
                output_queue,
                "MP4 mux/filesink failure: recording side stopped; decoder fallback/reconnect disabled",
            )
            _emit_terminal_exit(output_queue, "MP4 mux/filesink failure")
            return
        if bool(config.pop("_decoder_manual_restart_required", False)):
            _emit_terminal_exit(output_queue, "decoder-health failure requires manual restart")
            return
        fallback_reason = config.pop("_runtime_decoder_fallback_reason", None)
        if fallback_reason and not stop_event.is_set():
            config["_runtime_decoder_fallback_active"] = True
            config["decoder"] = DECODER_SOFTWARE
            if _record_video_path(config):
                _emit_decoder_fallback(
                    output_queue,
                    reason=(
                        f"{fallback_reason}; this side MP4 finalized; software override applies on next side start"
                    ),
                    recording_restart_required=True,
                )
                return
            _emit_decoder_fallback(
                output_queue,
                reason=(f"{fallback_reason}; preview-only avdec_h264 restart; current-generation frame pending"),
                recording_restart_required=False,
            )
            _run_gstreamer_stream_once(config, output_queue, stop_event, cv2, model, enable_detection)
            return
        if not recovered and not stop_event.is_set():
            _emit_status(
                output_queue,
                "GStreamer RTSP 세션 종료: 자동 재연결 안 함",
            )
        return

    delays = tuple(max(0.0, float(delay)) for delay in GST_RTSP_RECONNECT_DELAYS_S) or (1.0,)
    consecutive_failures = 0
    reconnect_attempt = 0
    while not stop_event.is_set():
        if _run_gstreamer_stream_once(config, output_queue, stop_event, cv2, model, enable_detection):
            return
        if stop_event.is_set():
            return
        if bool(config.pop("_integrity_manual_reconnect_required", False)):
            _emit_status(
                output_queue,
                "H264 IDR random-access 복구 실패: 이 side unavailable; 자동 재연결 안 함; 수동 재연결이 필요합니다.",
            )
            _emit_terminal_exit(
                output_queue,
                "H264 IDR random-access recovery unavailable; manual reconnect required",
            )
            return
        recording_failure = config.pop("_recording_output_failure", None)
        if recording_failure:
            _emit_status(
                output_queue,
                "MP4 mux/filesink failure: recording side stopped; decoder fallback/reconnect disabled",
            )
            _emit_terminal_exit(output_queue, "MP4 mux/filesink failure")
            return
        if bool(config.pop("_decoder_manual_restart_required", False)):
            _emit_terminal_exit(output_queue, "decoder-health failure requires manual restart")
            return
        fallback_reason = config.pop("_runtime_decoder_fallback_reason", None)
        if fallback_reason:
            config["_runtime_decoder_fallback_active"] = True
            config["decoder"] = DECODER_SOFTWARE
            if _record_video_path(config):
                _emit_decoder_fallback(
                    output_queue,
                    reason=(
                        f"{fallback_reason}; this side MP4 finalized; software override applies on next side start"
                    ),
                    recording_restart_required=True,
                )
                return
            _emit_decoder_fallback(
                output_queue,
                reason=(f"{fallback_reason}; preview-only avdec_h264 restart; current-generation frame pending"),
                recording_restart_required=False,
            )
            continue
        if bool(config.pop("_gst_last_run_received_frame", False)):
            consecutive_failures = 0
        delay_s = delays[min(consecutive_failures, len(delays) - 1)]
        consecutive_failures += 1
        reconnect_attempt += 1
        _emit_status(
            output_queue,
            f"GStreamer RTSP 재연결 대기 (시도 {reconnect_attempt}): {delay_s:.1f}s 후 같은 endpoint 재시도",
        )
        if stop_event.wait(delay_s):
            return
