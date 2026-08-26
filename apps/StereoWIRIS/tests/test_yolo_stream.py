from types import SimpleNamespace
import queue
import signal
import threading
import time

import numpy as np
import pytest

from wiris_stereo_distance.yolo_stream import (
    AdaptiveYoloRateLimiter,
    CPU_DECODE_HZ_CRITICAL_GUARD,
    CPU_DECODE_HZ_FLOOR,
    CPU_DECODE_HZ_RECOVERY,
    DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_INITIAL_RATE_HZ,
    DEFAULT_CPU_WORKER_THREADS,
    DEFAULT_CUDA_WORKER_THREADS,
    DECODER_NVIDIA,
    NVIDIA_H264_DECODER,
    SOFTWARE_H264_DECODER,
    DEFAULT_RTSP_LATENCY_MS,
    GstRtpTimingTracker,
    RollingFrameRateEstimator,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
    RtpFrameTiming,
    RtcpSenderReportTiming,
    YoloStreamWorker,
    _BoundedProcessOutputQueue,
    _finalize_gst_recording,
    _iter_ffmpeg_recording_pids,
    _open_record_video_writer,
    _emit_preview_frame,
    _process_frame_for_output,
    _report_recording_process_early_exit,
    _release_record_video_writer,
    _run_opencv_stream,
    _activate_cpu_fallback_rate_limiter,
    _should_emit_preview_frame,
    _write_record_video_frame,
    build_ffmpeg_recording_command,
    build_gst_pipeline_text,
    build_gst_recording_pipeline_text,
    decoder_candidates,
    effective_yolo_max_rate_hz,
    estimate_rtcp_ntp_unix_seconds,
    ntp64_to_unix_seconds,
    normalize_yolo_input_size,
    normalize_rtsp_transport,
    normalize_stream_backend,
    rtp_timestamp_delta,
)


class _FakeScalar:
    def __init__(self, value) -> None:
        self.value = value

    def item(self):
        return self.value


class _FakeXyxy:
    def __init__(self, coords) -> None:
        self.coords = coords

    def __getitem__(self, _index):
        return self

    def tolist(self):
        return list(self.coords)


class _FakeBox:
    def __init__(self, coords, confidence, class_id) -> None:
        self.xyxy = _FakeXyxy(coords)
        self.conf = _FakeScalar(confidence)
        self.cls = _FakeScalar(class_id)


class _FakeModel:
    def __init__(self, boxes, names=None) -> None:
        self.boxes = boxes
        self.names = names or {0: "person", 1: "car", 2: "bottle"}

    def predict(self, *_args, **_kwargs):
        return [SimpleNamespace(boxes=self.boxes)]


class _CudaFailsFakeModel(_FakeModel):
    def __init__(self, boxes, names=None) -> None:
        super().__init__(boxes, names=names)
        self.devices = []

    def predict(self, *_args, **kwargs):
        device = kwargs.get("device")
        self.devices.append(device)
        if device == "cuda:0":
            raise RuntimeError("CUDA unavailable")
        return super().predict(*_args, **kwargs)


class _SlowFakeModel(_FakeModel):
    def predict(self, *_args, **_kwargs):
        time.sleep(0.03)
        return super().predict(*_args, **_kwargs)


class _FastFakeCapture:
    def __init__(self) -> None:
        self.released = False
        self.frame = np.zeros((24, 32, 3), dtype=np.uint8)

    def isOpened(self) -> bool:
        return True

    def set(self, *_args) -> bool:
        return True

    def get(self, prop) -> int:
        if prop == 3:
            return 32
        if prop == 4:
            return 24
        return 0

    def read(self):
        time.sleep(0.005)
        return True, self.frame.copy()

    def release(self) -> None:
        self.released = True


def test_process_output_queue_keeps_latest_preview_without_dropping_detection_events() -> None:
    event_queue: queue.Queue = queue.Queue(maxsize=1)
    frame_queue: queue.Queue = queue.Queue(maxsize=1)
    output_queue = _BoundedProcessOutputQueue(event_queue, frame_queue)

    output_queue.put(("frame", {}, 1, 1, b"a"))
    output_queue.put(("frame", {}, 1, 1, b"b"))
    output_queue.put(("detection", 1.0, 1, 1, {"confidence": 0.9}, [], {}))

    assert frame_queue.qsize() == 1
    assert frame_queue.get_nowait()[4] == b"b"
    event = event_queue.get_nowait()
    assert event[0] == "detection"
    assert event[4]["confidence"] == 0.9


def test_preview_output_rate_limiter_does_not_change_capture_timestamps() -> None:
    config = {"display_rate_hz": 10.0}

    assert _should_emit_preview_frame(config, 1.0)
    assert not _should_emit_preview_frame(config, 1.05)
    assert _should_emit_preview_frame(config, 1.1)
    assert _should_emit_preview_frame({"display_rate_hz": "invalid"}, 1.0)


def test_preview_identity_metadata_hashes_the_exact_queued_rgb_bytes(monkeypatch, tmp_path) -> None:
    import hashlib
    import cv2

    monkeypatch.setenv("STEREO_PREVIEW_IDENTITY_LOG", str(tmp_path / "identity.jsonl"))

    class OutputQueue:
        def __init__(self) -> None:
            self.messages = []

        def put(self, message) -> None:
            self.messages.append(message)

    frame = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
    output_queue = OutputQueue()
    config = {"display_rate_hz": 0.0, "preview_max_width": 0, "url": "rtsp://camera/visible"}

    assert _emit_preview_frame(
        cv2,
        frame,
        config,
        output_queue,
        timestamp_s=10.0,
        source_timestamp_s=9.5,
        timestamp_source="test-pts",
    )

    _kind, metadata, width, height, data = output_queue.messages[0]
    assert (width, height) == (2, 1)
    assert metadata["frame_identity_version"] == 1
    assert metadata["frame_seq"] == 1
    assert metadata["pixel_format"] == "RGB888"
    assert metadata["stride"] == 6
    assert metadata["byte_count"] == len(data) == 6
    assert metadata["producer_rgb_sha256"] == hashlib.sha256(data).hexdigest()


def test_worker_identity_log_proves_queue_qimage_match_and_detects_reorder(monkeypatch, tmp_path) -> None:
    import hashlib
    import json

    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    identity_path = tmp_path / "identity.jsonl"
    monkeypatch.setenv("STEREO_PREVIEW_IDENTITY_LOG", str(identity_path))

    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        enable_detection=False,
    )
    original = bytes([255, 0, 0, 0, 255, 0])
    digest = hashlib.sha256(original).hexdigest()

    def emit(sequence: int) -> None:
        worker._handle_worker_message(
            (
                "frame",
                {
                    "timestamp_s": float(sequence),
                    "frame_identity_version": 1,
                    "frame_seq": sequence,
                    "producer_rgb_sha256": digest,
                    "pixel_format": "RGB888",
                    "stride": 6,
                    "byte_count": len(original),
                },
                2,
                1,
                original,
            )
        )

    emit(1)
    emit(3)
    emit(2)

    records = [json.loads(line) for line in identity_path.read_text(encoding="utf-8").splitlines()]
    assert [record["frame_seq"] for record in records] == [1, 3, 2]
    assert all(record["producer_queue_match"] for record in records)
    assert all(record["queue_qimage_match"] for record in records)
    assert all(record["qimage_bytes_per_line"] >= 6 for record in records)
    assert records[1]["queue_sequence_delta"] == 2
    assert records[1]["queue_dropped_total"] == 1
    assert records[2]["queue_reordered"] is True
    assert records[2]["queue_reordered_total"] == 1
    _ = app


def test_worker_qimage_deep_copy_survives_source_buffer_overwrite(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        enable_detection=False,
    )
    received = []
    worker.frame_ready.connect(lambda image, _metadata: received.append(image))
    source = bytearray([255, 0, 0, 0, 255, 0])

    worker._handle_worker_message(("frame", {}, 2, 1, source))
    source[:] = b"\x00" * len(source)

    assert len(received) == 1
    assert received[0].pixelColor(0, 0).getRgb()[:3] == (255, 0, 0)
    assert received[0].pixelColor(1, 0).getRgb()[:3] == (0, 255, 0)
    _ = app


def _predict_config() -> dict:
    return {
        "input_size": 640,
        "conf_threshold": 0.5,
        "iou_threshold": 0.45,
        "device": "cuda:0",
    }


def test_gstreamer_pipeline_uses_configurable_low_latency_default() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible")

    assert f"latency={DEFAULT_RTSP_LATENCY_MS}" in pipeline
    assert "protocols=udp" in pipeline
    assert "drop-on-latency=true" in pipeline
    assert "do-rtcp=true" in pipeline
    assert "rtspsrc name=src" in pipeline
    assert "rtph264depay name=depay source-info=true" in pipeline
    assert "h264parse config-interval=-1" in pipeline
    assert "queue max-size-buffers=0 max-size-bytes=0 max-size-time=250000000" in pipeline
    assert "queue leaky=downstream max-size-buffers=8" in pipeline
    assert "queue leaky=downstream max-size-buffers=1" not in pipeline
    assert NVIDIA_H264_DECODER in pipeline
    assert SOFTWARE_H264_DECODER not in pipeline
    assert "appsink name=sink" in pipeline


def test_rtcp_sender_report_ntp_estimate_handles_rtp_wraparound() -> None:
    ntp_raw = (2_208_988_800 + 100) << 32

    assert ntp64_to_unix_seconds(ntp_raw) == 100.0
    assert rtp_timestamp_delta(10, 0xFFFF_FFF0) == 26
    assert (
        estimate_rtcp_ntp_unix_seconds(
            rtp_timestamp=10,
            sr_rtp_timestamp=0xFFFF_FFF0,
            sr_ntp_unix_s=100.0,
            clock_rate_hz=90_000,
        )
        == 100.0 + 26 / 90_000
    )


def test_rtcp_sender_report_rejects_large_pc_clock_offset() -> None:
    from wiris_stereo_distance.yolo_stream import (
        rtcp_sender_report_is_pc_valid,
        rtcp_sender_report_pc_offset_s,
    )

    assert abs(rtcp_sender_report_pc_offset_s(sr_ntp_unix_s=99.9, received_wall_s=100.0) + 0.1) < 1e-9
    assert rtcp_sender_report_is_pc_valid(sr_ntp_unix_s=99.9, received_wall_s=100.0)
    assert not rtcp_sender_report_is_pc_valid(sr_ntp_unix_s=98.0, received_wall_s=100.0)


def test_rtcp_sender_report_interval_logging_triggers_on_new_sr() -> None:
    messages: list[str] = []
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace(), status_callback=messages.append)

    first = RtcpSenderReportTiming(
        ssrc=1234,
        ntp_raw=1,
        ntp_unix_s=100.0,
        rtp_timestamp=10,
        clock_rate_hz=90_000,
        received_monotonic_s=1.0,
    )
    second = RtcpSenderReportTiming(
        ssrc=1234,
        ntp_raw=2,
        ntp_unix_s=102.0,
        rtp_timestamp=20,
        clock_rate_hz=90_000,
        received_monotonic_s=3.5,
    )

    assert tracker._register_sender_report(first) is None
    message = tracker._register_sender_report(second)
    assert tracker._sr_count_by_ssrc[1234] == 2
    assert message == (
        "[RTCP] Sender Report 갱신: ssrc=1234, interval=2.50s, ntp=102.000000"
        ", ntp_rate=0.800000, media_rate=0.000044"
    )
    tracker._emit_status(message)
    assert messages == [message]


def test_repeated_sender_report_keeps_its_original_arrival_timestamps() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    first = RtcpSenderReportTiming(
        ssrc=1234,
        ntp_raw=1,
        ntp_unix_s=100.0,
        rtp_timestamp=10,
        clock_rate_hz=90_000,
        received_monotonic_s=1.0,
        received_wall_s=1_000.0,
    )
    # The 250 ms poll re-reads the same report from the session stats, so it
    # arrives again stamped with the current time.
    same_report_polled_later = RtcpSenderReportTiming(
        ssrc=1234,
        ntp_raw=1,
        ntp_unix_s=100.0,
        rtp_timestamp=10,
        clock_rate_hz=90_000,
        received_monotonic_s=61.0,
        received_wall_s=1_060.0,
    )

    tracker._register_sender_report(first)
    assert tracker._register_sender_report(same_report_polled_later) is None
    assert tracker._sr_count_by_ssrc[1234] == 1

    stored = tracker._sr_by_ssrc[1234]
    # Otherwise rtcp_sr_age_s would report ~0 s forever and could never expose
    # an anchor that stopped refreshing.
    assert stored.received_monotonic_s == 1.0
    assert stored.received_wall_s == 1_000.0


def test_frame_transport_delay_cancels_camera_clock_offset() -> None:
    from wiris_stereo_distance.yolo_stream import (
        estimate_rtcp_ntp_unix_seconds,
        rtcp_sender_report_pc_offset_s,
    )

    clock_rate_hz = 90_000
    transport_delay_s = 0.05
    measured: list[tuple[float, float]] = []

    for elapsed_s, camera_clock_error_s in ((0.0, 1.0), (600.0, 1.6), (1800.0, 2.8)):
        sr_received_wall_s = 1_000.0 + elapsed_s
        sr_ntp_unix_s = sr_received_wall_s - camera_clock_error_s
        sr_rtp_timestamp = int(elapsed_s * clock_rate_hz)
        frame_rtp_timestamp = sr_rtp_timestamp + int(0.08 * clock_rate_hz)
        frame_received_wall_s = sr_received_wall_s + 0.08 + transport_delay_s

        frame_ntp_s = estimate_rtcp_ntp_unix_seconds(
            rtp_timestamp=frame_rtp_timestamp,
            sr_rtp_timestamp=sr_rtp_timestamp,
            sr_ntp_unix_s=sr_ntp_unix_s,
            clock_rate_hz=clock_rate_hz,
        )
        receive_delay_s = frame_received_wall_s - frame_ntp_s
        sr_pc_offset_s = rtcp_sender_report_pc_offset_s(
            sr_ntp_unix_s=sr_ntp_unix_s,
            received_wall_s=sr_received_wall_s,
        )
        measured.append((receive_delay_s, receive_delay_s + sr_pc_offset_s))

    receive_delays = [receive_delay_s for receive_delay_s, _ in measured]
    transport_delays = [transport_delay for _, transport_delay in measured]

    # receive_delay tracks the growing camera clock error ...
    assert receive_delays[0] < receive_delays[1] < receive_delays[2]
    # ... while the compensated transport delay stays at the real value.
    for value in transport_delays:
        assert value == pytest.approx(transport_delay_s, abs=1e-6)


def test_sender_report_clock_rates_separate_ntp_drift_from_media_clock_drift() -> None:
    from wiris_stereo_distance.yolo_stream import sender_report_clock_rates

    clock_rate_hz = 90_000

    def report(pc_s: float, ntp_s: float, media_s: float) -> RtcpSenderReportTiming:
        return RtcpSenderReportTiming(
            ssrc=1,
            ntp_raw=int(ntp_s * 1000),
            ntp_unix_s=ntp_s,
            rtp_timestamp=int(media_s * clock_rate_hz),
            clock_rate_hz=clock_rate_hz,
            received_monotonic_s=pc_s,
            received_wall_s=pc_s,
        )

    first = report(0.0, 1_000.0, 0.0)

    healthy = sender_report_clock_rates(first, report(5.0, 1_005.0, 5.0))
    assert healthy["ntp_rate"] == pytest.approx(1.0)
    assert healthy["media_rate"] == pytest.approx(1.0)

    # Wall clock losing time inflates transit delay only.
    ntp_drift = sender_report_clock_rates(first, report(5.0, 1_004.99, 5.0))
    assert ntp_drift["ntp_rate"] < 1.0
    assert ntp_drift["media_rate"] == pytest.approx(1.0)

    # A slow RTP clock undercounts media time, which inflates transport delay.
    media_drift = sender_report_clock_rates(first, report(5.0, 1_005.0, 4.99))
    assert media_drift["ntp_rate"] == pytest.approx(1.0)
    assert media_drift["media_rate"] < 1.0

    assert sender_report_clock_rates(first, report(0.0, 1_000.0, 0.0)) is None


def test_sender_report_clock_rates_use_median_of_recent_intervals() -> None:
    from wiris_stereo_distance.yolo_stream import robust_sender_report_clock_rates

    samples = [
        {"ntp_rate": 0.9998, "media_rate": 1.0002},
        {"ntp_rate": 1.0001, "media_rate": 0.9999},
        # A polling/dispatch outlier should not become the camera's rate.
        {"ntp_rate": 1.048, "media_rate": 0.971},
        {"ntp_rate": 1.0000, "media_rate": 1.0001},
    ]

    rates = robust_sender_report_clock_rates(samples)

    assert rates == pytest.approx({"ntp_rate": 1.00005, "media_rate": 1.0000})


def test_sender_report_arrival_cache_preserves_probe_timestamp() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())

    tracker._remember_sender_report_arrival(
        ssrc=1234,
        ntp_raw=99,
        received_monotonic_s=12.345,
        received_wall_s=1_800_000_012.345,
    )

    assert tracker._sender_report_arrival(ssrc=1234, ntp_raw=99) == pytest.approx(
        (12.345, 1_800_000_012.345)
    )


def test_sender_report_arrival_cache_evicts_old_keys_without_leaking() -> None:
    from wiris_stereo_distance.yolo_stream import MAX_RTCP_SR_ARRIVAL_CACHE

    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    for ntp_raw in range(MAX_RTCP_SR_ARRIVAL_CACHE + 1):
        tracker._remember_sender_report_arrival(
            ssrc=1234,
            ntp_raw=ntp_raw,
            received_monotonic_s=float(ntp_raw),
            received_wall_s=float(ntp_raw),
        )

    assert tracker._sender_report_arrival(ssrc=1234, ntp_raw=0) is None
    assert len(tracker._sr_arrival_by_key) == MAX_RTCP_SR_ARRIVAL_CACHE


def test_rtp_timing_tracker_does_not_attach_stale_timing_to_known_pts() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    timing = RtpFrameTiming(
        rtp_timestamp=90_000,
        rtp_ssrc=1234,
        rtp_marker_seq=10,
        rtp_pts_s=1.0,
    )
    with tracker._lock:
        tracker._timing_by_pts_ns[1_000_000_000] = timing

    assert tracker._pop_frame_timing(1_100_000_000) is None

    with tracker._lock:
        assert tracker._timing_by_pts_ns[1_000_000_000] is timing
    assert tracker._pop_frame_timing(None) is timing


def test_rtp_timing_tracker_keeps_relative_timestamp_when_pc_utc_is_invalid() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(CLOCK_TIME_NONE=-1), SimpleNamespace())
    with tracker._lock:
        tracker._timing_by_pts_ns[1_000_000_000] = RtpFrameTiming(
            rtp_timestamp=90_000,
            rtp_ssrc=1234,
            rtp_marker_seq=10,
            rtp_pts_s=1.0,
        )
        tracker._sr_by_ssrc[1234] = RtcpSenderReportTiming(
            ssrc=1234,
            ntp_raw=1,
            ntp_unix_s=98.0,
            rtp_timestamp=0,
            clock_rate_hz=90_000,
            received_monotonic_s=0.9,
            received_wall_s=100.0,
        )
        tracker._sr_count_by_ssrc[1234] = 7

    metadata = tracker.frame_metadata(
        SimpleNamespace(pts=1_000_000_000),
        1.0,
        received_wall_s=100.0,
    )

    assert metadata is not None
    assert abs(float(metadata["rtcp_frame_ntp_unix_s"]) - 99.0) < 1e-9
    assert abs(float(metadata["rtcp_sr_pc_offset_s"]) + 2.0) < 1e-9
    assert abs(float(metadata["rtcp_frame_receive_delay_s"]) - 1.0) < 1e-9
    assert metadata["rtcp_sr_received_count"] == 7
    assert "rtcp_timing_invalid_reason" not in metadata


def test_rtp_sequence_tracker_latches_forward_gap_until_pipeline_replacement() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())

    assert tracker.register_rtp_sequence(ssrc=0x0F078CC9, sequence=11570, observed_monotonic_s=1.0) is None
    discontinuity = tracker.register_rtp_sequence(
        ssrc=0x0F078CC9,
        sequence=11585,
        observed_monotonic_s=1.1,
        rtp_pts_ns=1_000,
    )

    assert discontinuity is not None
    assert discontinuity.ssrc == 0x0F078CC9
    assert discontinuity.previous_sequence == 11570
    assert discontinuity.sequence == 11585
    assert discontinuity.missing_packets == 14
    assert discontinuity.observed_monotonic_s == pytest.approx(1.1)
    assert tracker.decoded_output_blocked()
    assert tracker.pop_rtp_discontinuity() == discontinuity
    assert tracker.pop_rtp_discontinuity() is None
    # Consuming the diagnostic record must not reopen the NVIDIA output race.
    assert tracker.decoded_output_blocked()


def test_rtp_gate_reopens_only_at_clean_intra_after_gap_pts() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    tracker.register_rtp_sequence(ssrc=7, sequence=100, rtp_pts_ns=1_000)
    tracker.register_rtp_sequence(
        ssrc=7,
        sequence=103,
        observed_monotonic_s=5.0,
        rtp_pts_ns=2_000,
    )

    assert not tracker.register_access_unit(
        pts_ns=1_900,
        is_delta=False,
        is_corrupted=False,
        observed_monotonic_s=5.1,
    )
    assert not tracker.register_access_unit(
        pts_ns=2_000,
        is_delta=False,
        is_corrupted=False,
        observed_monotonic_s=5.15,
    )
    assert not tracker.register_access_unit(
        pts_ns=2_100,
        is_delta=True,
        is_corrupted=False,
        observed_monotonic_s=5.2,
    )
    assert not tracker.register_access_unit(
        pts_ns=2_200,
        is_delta=False,
        is_corrupted=True,
        observed_monotonic_s=5.3,
    )
    assert tracker.register_access_unit(
        pts_ns=2_300,
        is_delta=False,
        is_corrupted=False,
        observed_monotonic_s=5.4,
    )
    assert tracker.should_suppress_decoded_buffer(2_200)
    assert tracker.decoded_output_blocked()
    assert not tracker.should_suppress_decoded_buffer(2_300)
    assert not tracker.decoded_output_blocked()
    recovery = tracker.pop_rtp_recovery()
    assert recovery is not None
    assert recovery.ssrc == 7
    assert recovery.resume_pts_ns == 2_300
    assert recovery.blocked_duration_s == pytest.approx(0.4)


def test_encoded_au_gate_drops_missing_pts_and_delta_until_clean_intra() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())

    assert tracker.should_drop_encoded_access_unit(
        pts_ns=None,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        observed_monotonic_s=10.0,
    )
    event = tracker.pop_h264_access_unit_discontinuity()
    assert event is not None
    assert event.reason == "missing-pts"
    assert tracker.decoded_output_blocked()
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=1_000,
        is_delta=True,
        is_corrupted=False,
        is_discontinuous=False,
        observed_monotonic_s=10.1,
    )
    assert not tracker.should_drop_encoded_access_unit(
        pts_ns=2_000,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        observed_monotonic_s=10.2,
    )
    assert tracker.consume_encoded_recovery_marker(2_000)
    assert not tracker.consume_encoded_recovery_marker(2_000)
    assert tracker.decoded_output_blocked()
    assert tracker.should_suppress_decoded_buffer(1_999)
    assert not tracker.should_suppress_decoded_buffer(2_000)


def test_encoded_au_gate_relatches_on_discont_or_corrupted_buffer() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())

    assert tracker.should_drop_encoded_access_unit(
        pts_ns=5_000,
        is_delta=True,
        is_corrupted=False,
        is_discontinuous=True,
        observed_monotonic_s=20.0,
    )
    assert tracker.pop_h264_access_unit_discontinuity().reason == "discont"
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=5_100,
        is_delta=False,
        is_corrupted=True,
        is_discontinuous=False,
        observed_monotonic_s=20.1,
    )
    assert not tracker.should_drop_encoded_access_unit(
        pts_ns=5_200,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        observed_monotonic_s=20.2,
    )
    assert tracker.consume_encoded_recovery_marker(5_200)
    assert not tracker.should_suppress_decoded_buffer(5_200)


def test_encoded_au_gate_drops_non_delta_discont_before_later_clean_intra() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())

    assert tracker.should_drop_encoded_access_unit(
        pts_ns=7_000,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=True,
        observed_monotonic_s=30.0,
    )
    assert tracker.decoded_output_blocked()
    assert tracker.should_suppress_decoded_buffer(7_000)
    assert not tracker.should_drop_encoded_access_unit(
        pts_ns=8_000,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        observed_monotonic_s=30.5,
    )
    assert not tracker.should_suppress_decoded_buffer(8_000)


def test_rtp_sequence_tracker_ignores_wrap_duplicate_old_and_ssrc_change() -> None:
    tracker = GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())

    assert tracker.register_rtp_sequence(ssrc=1, sequence=65535) is None
    assert tracker.register_rtp_sequence(ssrc=1, sequence=0) is None
    assert tracker.register_rtp_sequence(ssrc=1, sequence=0) is None
    assert tracker.register_rtp_sequence(ssrc=1, sequence=65535) is None
    assert tracker.register_rtp_sequence(ssrc=1, sequence=1) is None
    assert tracker.register_rtp_sequence(ssrc=2, sequence=40000) is None
    assert tracker.register_rtp_sequence(ssrc=2, sequence=40001) is None

    assert tracker.pop_rtp_discontinuity() is None
    assert not tracker.decoded_output_blocked()


def test_rtp_gap_gates_both_decoders_without_triggering_health_fallback() -> None:
    from wiris_stereo_distance import yolo_stream

    discontinuity = yolo_stream.RtpSequenceDiscontinuity(
        ssrc=1234,
        previous_sequence=11570,
        sequence=11585,
        missing_packets=14,
        observed_monotonic_s=20.0,
    )
    common = {
        "active_decoder": yolo_stream.NVIDIA_H264_DECODER,
        "started_s": 10.0,
        "last_frame_s": 19.9,
        "now_s": 20.0,
        "already_fell_back": False,
        "rtp_discontinuity": discontinuity,
    }

    assert yolo_stream.runtime_decoder_fallback_reason(
        requested_decoder="auto",
        **common,
    ) is None
    assert yolo_stream.runtime_decoder_fallback_reason(
        requested_decoder="nvidia",
        **common,
    ) is None
    assert yolo_stream.nvidia_decoder_health_failure_reason(**common) is None
    assert yolo_stream.should_suppress_decoded_output(
        active_decoder=yolo_stream.NVIDIA_H264_DECODER,
        rtp_discontinuity_latched=True,
    )
    assert yolo_stream.should_suppress_decoded_output(
        active_decoder=yolo_stream.SOFTWARE_H264_DECODER,
        rtp_discontinuity_latched=True,
    )


def test_software_rtp_gap_reporter_aggregates_instead_of_flooding_status() -> None:
    from wiris_stereo_distance import yolo_stream

    reporter = yolo_stream.RtpDiscontinuityReporter(interval_s=5.0)

    def event(observed_s: float, missing: int) -> yolo_stream.RtpSequenceDiscontinuity:
        return yolo_stream.RtpSequenceDiscontinuity(
            ssrc=1,
            previous_sequence=10,
            sequence=11 + missing,
            missing_packets=missing,
            observed_monotonic_s=observed_s,
        )

    assert reporter.observe(event(1.0, 2), now_s=1.0) is None
    assert reporter.observe(event(3.0, 4), now_s=3.0) is None
    status = reporter.observe(event(6.0, 1), now_s=6.0)
    assert status is not None
    assert "events=3 (+3)" in status
    assert "missing=7 (+7)" in status
    assert reporter.observe(event(7.0, 5), now_s=7.0) is None
    final_status = reporter.flush(now_s=7.0)
    assert final_status is not None
    assert "events=4 (+1)" in final_status
    assert "missing=12 (+5)" in final_status
    assert reporter.flush(now_s=8.0) is None


def test_auto_fallback_503_reconnect_uses_bounded_backoff_only_for_software_transition() -> None:
    from wiris_stereo_distance import yolo_stream

    common = {
        "active_decoder": yolo_stream.SOFTWARE_H264_DECODER,
        "fallback_active": True,
        "error_detail": "Service Unavailable (503)",
    }
    assert yolo_stream.decoder_fallback_rtsp_retry_delay_s(**common, retry_count=0) == 0.5
    assert yolo_stream.decoder_fallback_rtsp_retry_delay_s(**common, retry_count=1) == 1.0
    assert yolo_stream.decoder_fallback_rtsp_retry_delay_s(**common, retry_count=2) == 2.0
    assert yolo_stream.decoder_fallback_rtsp_retry_delay_s(**common, retry_count=3) is None
    assert yolo_stream.decoder_fallback_rtsp_retry_delay_s(
        **{**common, "active_decoder": yolo_stream.NVIDIA_H264_DECODER},
        retry_count=0,
    ) is None
    assert yolo_stream.decoder_fallback_rtsp_retry_delay_s(
        **{**common, "fallback_active": False},
        retry_count=0,
    ) is None
    assert yolo_stream.decoder_fallback_rtsp_retry_delay_s(
        **{**common, "error_detail": "Unauthorized (401)"},
        retry_count=0,
    ) is None


def test_gstreamer_initial_playing_failure_uses_bounded_start_retry() -> None:
    from wiris_stereo_distance import yolo_stream

    detail = "avdec_h264: avdec_h264 failed to enter PLAYING state"
    assert yolo_stream.gstreamer_start_retry_delay_s(error_detail=detail, retry_count=0) == 0.5
    assert yolo_stream.gstreamer_start_retry_delay_s(error_detail=detail, retry_count=1) == 1.0
    assert yolo_stream.gstreamer_start_retry_delay_s(error_detail=detail, retry_count=2) == 2.0
    assert yolo_stream.gstreamer_start_retry_delay_s(error_detail=detail, retry_count=3) == 4.0
    assert yolo_stream.gstreamer_start_retry_delay_s(error_detail=detail, retry_count=4) is None
    assert yolo_stream.gstreamer_start_retry_delay_s(
        error_detail="pipeline missing appsink",
        retry_count=0,
    ) is None


def test_gstreamer_pipeline_can_drop_delta_frames_for_keyframe_only_diagnostics() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", keyframe_only=True)

    assert "h264parse config-interval=-1 disable-passthrough=true" in pipeline
    assert "video/x-h264,alignment=au" in pipeline
    assert "identity drop-buffer-flags=delta-unit" in pipeline


def test_gstreamer_pipeline_names_primary_parser_for_intra_recovery_probe() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible")

    assert "h264parse config-interval=-1 name=parse" in pipeline


def test_gstreamer_pipeline_records_encoded_h264_without_reencoding(tmp_path) -> None:
    record_path = tmp_path / "raw left.mp4"
    pipeline = build_gst_pipeline_text(
        "rtsp://10.0.0.232:8554/visible",
        record_video_path=str(record_path),
    )

    assert "! tee name=encoded" in pipeline
    assert "encoded. ! queue" in pipeline
    assert "! video/x-h264,stream-format=avc,alignment=au" in pipeline
    assert "fragment-duration" not in pipeline
    assert "! mp4mux ! filesink" in pipeline
    assert "matroskamux" not in pipeline
    assert f'! filesink location="{record_path}" sync=false' in pipeline
    assert pipeline.count("h264parse") >= 2
    assert "! videoconvert ! video/x-raw,format=BGR ! appsink" in pipeline
    assert pipeline.count("rtspsrc") == 1


def test_gstreamer_location_round_trips_non_ascii_path_without_literal_quotes(tmp_path) -> None:
    import gi

    from wiris_stereo_distance import yolo_stream

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    Gst.init(None)
    record_path = tmp_path / "바탕 화면" / 'raw "left".mp4'
    pipeline = Gst.parse_launch(
        "fakesrc ! filesink name=record_sink "
        f"location={yolo_stream._gst_location(str(record_path))}"
    )
    try:
        assert pipeline.get_by_name("record_sink").get_property("location") == str(record_path)
    finally:
        pipeline.set_state(Gst.State.NULL)


def test_gstreamer_recording_finalization_waits_beyond_old_three_second_budget(
    monkeypatch,
    tmp_path,
) -> None:
    from wiris_stereo_distance import yolo_stream

    clock_ticks = iter(float(index) * 0.5 for index in range(40))
    monkeypatch.setattr(yolo_stream.time, "monotonic", lambda: next(clock_ticks))

    class FakePipeline:
        def __init__(self) -> None:
            self.events = []

        def send_event(self, event) -> None:
            self.events.append(event)

    class FakeBus:
        def __init__(self) -> None:
            self.calls = 0

        def timed_pop_filtered(self, _timeout_ns, _message_types):
            self.calls += 1
            if self.calls <= 7:
                return None
            return SimpleNamespace(type=1)

    class OutputQueue:
        def __init__(self) -> None:
            self.messages = []

        def put(self, message) -> None:
            self.messages.append(message)

    Gst = SimpleNamespace(
        Event=SimpleNamespace(new_eos=lambda: "eos"),
        MessageType=SimpleNamespace(EOS=1, ERROR=2),
    )
    pipeline = FakePipeline()
    bus = FakeBus()
    output_queue = OutputQueue()
    record_path = tmp_path / "large.mp4"
    record_path.write_bytes(b"mp4-data")

    _finalize_gst_recording(
        Gst,
        pipeline,
        bus,
        output_queue,
        str(record_path),
    )

    assert pipeline.events == ["eos"]
    assert bus.calls == 8
    assert any("저장 완료" in message[1] for message in output_queue.messages)


def test_gstreamer_recording_pipeline_saves_encoded_h264_without_decode(tmp_path) -> None:
    record_path = tmp_path / "raw left.mp4"
    pipeline = build_gst_recording_pipeline_text(
        "rtsp://10.0.0.232:8554/visible",
        record_video_path=str(record_path),
    )

    assert "rtph264depay" in pipeline
    assert "! h264parse config-interval=-1" in pipeline
    assert "! video/x-h264,stream-format=avc,alignment=au" in pipeline
    assert "fragment-duration" not in pipeline
    assert "! mp4mux ! filesink" in pipeline
    assert "matroskamux" not in pipeline
    assert f'! filesink location="{record_path}" sync=false' in pipeline
    assert "videoconvert" not in pipeline
    assert "appsink" not in pipeline


def test_ffmpeg_recording_command_copies_h264_to_mp4_with_generated_pts(tmp_path) -> None:
    record_path = tmp_path / "raw left.mp4"
    command = build_ffmpeg_recording_command(
        "rtsp://10.0.0.232:8554/visible",
        transport="udp",
        record_video_path=str(record_path),
    )

    assert command[:7] == ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-rtsp_transport", "udp", "-use_wallclock_as_timestamps"]
    assert "+genpts" in command
    assert "-c:v" in command
    assert "copy" in command
    assert "-movflags" in command
    assert "+frag_keyframe+empty_moov+default_base_moof" in command
    assert str(record_path) == command[-1]


def test_yolo_worker_honors_stop_requested_before_run_starts_child() -> None:
    worker = YoloStreamWorker(
        url="rtsp://camera.invalid/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        enable_detection=False,
    )

    class FailIfStartedContext:
        def Event(self):
            raise AssertionError("child lifecycle started after stop was requested")

    worker._mp_context = FailIfStartedContext()
    worker.request_stop()
    worker.run()

    assert worker._process is None


def test_stale_ffmpeg_scan_covers_prior_session_for_same_camera(tmp_path) -> None:
    proc_root = tmp_path / "proc"
    recordings_root = tmp_path / "recordings"
    current_path = recordings_root / "sessions" / "current" / "video" / "raw_left.mp4"
    stale_path = recordings_root / "sessions" / "old" / "video" / "raw_left.mp4"
    other_path = recordings_root / "sessions" / "other" / "video" / "raw_right.mp4"
    camera_url = "rtsp://10.0.0.231:8554/visible"

    def write_cmdline(pid: int, parts: list[str]) -> None:
        pid_dir = proc_root / str(pid)
        pid_dir.mkdir(parents=True)
        (pid_dir / "cmdline").write_bytes(b"\0".join(part.encode() for part in parts) + b"\0")

    write_cmdline(101, ["ffmpeg", "-i", camera_url, "-y", str(stale_path)])
    write_cmdline(
        102,
        ["ffmpeg", "-i", "rtsp://10.0.0.232:8554/visible", "-y", str(other_path)],
    )
    write_cmdline(103, ["python", "worker.py", camera_url, str(stale_path)])

    assert _iter_ffmpeg_recording_pids(
        str(current_path),
        camera_url=camera_url,
        proc_root=proc_root,
    ) == [101]


def test_force_stopping_worker_signals_registered_recorder_group_first(monkeypatch) -> None:
    from wiris_stereo_distance import yolo_stream as yolo_stream_module

    worker = YoloStreamWorker(
        url="rtsp://camera.invalid/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        enable_detection=False,
    )
    calls: list[tuple[object, ...]] = []

    class FakeChild:
        def is_alive(self) -> bool:
            return True

        def terminate(self) -> None:
            calls.append(("terminate_child",))

    worker._process = FakeChild()
    worker._handle_worker_message(
        ("recording_process_started", 4321, "/recordings/session/video/raw_left.mp4")
    )
    monkeypatch.setattr(
        yolo_stream_module.os,
        "killpg",
        lambda pid, requested_signal: calls.append(("killpg", pid, requested_signal)),
    )

    worker.terminate_child_process()

    assert calls == [
        ("killpg", 4321, signal.SIGINT),
        ("terminate_child",),
    ]


def test_ffmpeg_early_exit_reports_captured_error_to_worker_status() -> None:
    class FakeProcess:
        returncode = 1
        _recording_stderr_tail = ["camera rejected second RTSP client"]
        _recording_stderr_thread = None

        def poll(self) -> int:
            return self.returncode

    class OutputQueue:
        def __init__(self) -> None:
            self.messages: list[object] = []

        def put(self, message: object) -> None:
            self.messages.append(message)

    output_queue = OutputQueue()

    assert _report_recording_process_early_exit(
        FakeProcess(),
        output_queue,
        "/recordings/session/video/raw_left.mp4",
        already_reported=False,
    ) is True
    assert any(
        message[0] == "status"
        and "조기 종료" in message[1]
        and "camera rejected second RTSP client" in message[1]
        for message in output_queue.messages
    )


def test_gstreamer_pipeline_can_use_nvidia_decoder() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", decoder="nvidia")

    assert NVIDIA_H264_DECODER in pipeline
    assert SOFTWARE_H264_DECODER not in pipeline


def test_gstreamer_pipeline_can_use_software_decoder() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", decoder="software")

    assert SOFTWARE_H264_DECODER in pipeline
    assert "skip-frame=1" not in pipeline
    assert "output-corrupt=false" in pipeline
    assert NVIDIA_H264_DECODER not in pipeline


def test_decoder_candidates_auto_falls_back_to_software() -> None:
    assert decoder_candidates("auto") == (NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER)
    assert decoder_candidates("nvidia") == (NVIDIA_H264_DECODER,)
    assert decoder_candidates("software") == (SOFTWARE_H264_DECODER,)


def test_runtime_nvidia_decoder_fallback_uses_first_frame_and_stall_deadlines() -> None:
    from wiris_stereo_distance import yolo_stream

    common = {
        "requested_decoder": "auto",
        "active_decoder": yolo_stream.NVIDIA_H264_DECODER,
        "started_s": 10.0,
        "already_fell_back": False,
    }
    first_frame_deadline_s = 10.0 + yolo_stream.DECODER_FALLBACK_FIRST_FRAME_TIMEOUT_S
    assert yolo_stream.runtime_decoder_fallback_reason(
        **common, last_frame_s=None, now_s=first_frame_deadline_s - 0.1
    ) is None
    assert "first-frame" in yolo_stream.runtime_decoder_fallback_reason(
        **common, last_frame_s=None, now_s=first_frame_deadline_s
    )
    stall_deadline_s = 12.0 + yolo_stream.DECODER_FALLBACK_STALL_TIMEOUT_S
    assert yolo_stream.runtime_decoder_fallback_reason(
        **common, last_frame_s=12.0, now_s=stall_deadline_s - 0.1
    ) is None
    assert "stall" in yolo_stream.runtime_decoder_fallback_reason(
        **common, last_frame_s=12.0, now_s=stall_deadline_s
    )
    assert yolo_stream.runtime_decoder_fallback_reason(
        **{**common, "active_decoder": yolo_stream.SOFTWARE_H264_DECODER},
        last_frame_s=None,
        now_s=99.0,
    ) is None
    assert yolo_stream.runtime_decoder_fallback_reason(
        **{**common, "requested_decoder": "nvidia"},
        last_frame_s=None,
        now_s=99.0,
    ) is None
    assert "first-frame" in yolo_stream.nvidia_decoder_health_failure_reason(
        active_decoder=yolo_stream.NVIDIA_H264_DECODER,
        started_s=10.0,
        last_frame_s=None,
        now_s=first_frame_deadline_s,
        already_fell_back=False,
    )
    assert yolo_stream.runtime_decoder_fallback_reason(
        **common,
        last_frame_s=12.0,
        now_s=99.0,
        rtp_recovery_pending=True,
    ) is None
    assert yolo_stream.nvidia_decoder_health_failure_reason(
        active_decoder=yolo_stream.NVIDIA_H264_DECODER,
        started_s=10.0,
        last_frame_s=12.0,
        now_s=99.0,
        already_fell_back=False,
        rtp_recovery_pending=True,
    ) is None
    assert yolo_stream.runtime_decoder_fallback_reason(
        **common,
        last_frame_s=12.0,
        now_s=20.0,
        last_recovery_s=19.0,
    ) is None
    assert yolo_stream.runtime_decoder_fallback_reason(
        **common,
        last_frame_s=12.0,
        now_s=22.0,
        last_recovery_s=19.0,
    ) is None
    assert "stall" in yolo_stream.runtime_decoder_fallback_reason(
        **common,
        last_frame_s=12.0,
        now_s=19.0 + yolo_stream.DECODER_FALLBACK_STALL_TIMEOUT_S,
        last_recovery_s=19.0,
    )


def test_decoded_rate_uses_a_rolling_window_and_expires_after_a_gap() -> None:
    estimator = RollingFrameRateEstimator(window_s=5.0)
    for index in range(100):
        estimator.add(index / 20.0)

    assert estimator.rate(5.0) == pytest.approx(19.8)
    assert estimator.rate(10.1) == 0.0


def test_stream_backend_normalization() -> None:
    assert normalize_stream_backend("gstreamer") == "gstreamer"
    assert normalize_stream_backend("gst") == "gstreamer"
    assert normalize_stream_backend("opencv") == "opencv"
    assert normalize_stream_backend("ffmpeg") == "opencv"
    assert normalize_stream_backend("opencv-auto") == STREAM_BACKEND_OPENCV_AUTO
    assert normalize_stream_backend("cap_any") == STREAM_BACKEND_OPENCV_AUTO
    assert normalize_stream_backend("pyav") == STREAM_BACKEND_PYAV
    assert normalize_stream_backend("av") == STREAM_BACKEND_PYAV
    assert normalize_stream_backend("unknown") == "gstreamer"


def test_rtsp_transport_normalization() -> None:
    assert normalize_rtsp_transport("udp") == "udp"
    assert normalize_rtsp_transport("tcp") == "tcp"
    assert normalize_rtsp_transport("interleaved") == "tcp"
    assert normalize_rtsp_transport("unknown") == "udp"


def test_cpu_yolo_detection_gets_safe_default_rate_limit() -> None:
    assert effective_yolo_max_rate_hz(0.0, device="cpu", enable_detection=True) == DEFAULT_CPU_YOLO_MAX_RATE_HZ
    assert effective_yolo_max_rate_hz(None, device="", enable_detection=True) == DEFAULT_CPU_YOLO_MAX_RATE_HZ
    assert effective_yolo_max_rate_hz(15.0, device="cpu", enable_detection=True) == 15.0
    assert effective_yolo_max_rate_hz(0.0, device="cuda:0", enable_detection=True) == 0.0
    assert effective_yolo_max_rate_hz(0.0, device="cpu", enable_detection=False) == 0.0


def test_yolo_input_size_is_rounded_up_to_stride_multiple() -> None:
    assert normalize_yolo_input_size(640) == 640
    assert normalize_yolo_input_size(720) == 736
    assert normalize_yolo_input_size("bad") == 640


def test_cpu_yolo_adaptive_rate_starts_safe_then_raises_and_lowers() -> None:
    limiter = AdaptiveYoloRateLimiter.from_config(
        {"max_rate_hz": DEFAULT_CPU_YOLO_MAX_RATE_HZ, "adaptive_rate": True},
        enable_detection=True,
    )

    assert limiter.enabled
    assert limiter.target_rate_hz == DEFAULT_CPU_YOLO_INITIAL_RATE_HZ
    assert limiter.max_rate_hz == DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ

    assert not limiter.update(decoded_hz=0.2, inferred_hz=0.0)
    assert limiter.target_rate_hz == DEFAULT_CPU_YOLO_INITIAL_RATE_HZ

    limiter.update(decoded_hz=CPU_DECODE_HZ_RECOVERY, inferred_hz=5.0)
    assert limiter.target_rate_hz > DEFAULT_CPU_YOLO_INITIAL_RATE_HZ

    raised_rate = limiter.target_rate_hz
    limiter.update(decoded_hz=CPU_DECODE_HZ_FLOOR, inferred_hz=5.0)
    assert limiter.target_rate_hz < raised_rate

    limiter.target_rate_hz = raised_rate
    limiter.update(decoded_hz=CPU_DECODE_HZ_CRITICAL_GUARD - 0.1, inferred_hz=5.0)
    assert limiter.target_rate_hz == limiter.min_rate_hz


def test_yolo_worker_passes_rtsp_latency_to_runtime_config() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        display_rate_hz=15.0,
        preview_max_width=960,
        rtsp_latency_ms=25,
        decoder="software",
        stream_backend="opencv",
        rtsp_transport="tcp",
        enable_detection=False,
        record_video_path="/tmp/stereo-left.mp4",
        record_video_fps=24.0,
    )

    assert worker.config["rtsp_latency_ms"] == 25
    assert worker.config["decoder"] == "software"
    assert worker.config["stream_backend"] == "opencv"
    assert worker.config["rtsp_transport"] == "tcp"
    assert worker.config["adaptive_rate"] is False
    assert worker.config["worker_cpu_threads"] == DEFAULT_CPU_WORKER_THREADS
    assert worker.config["record_video_path"] == "/tmp/stereo-left.mp4"
    assert worker.config["record_video_fps"] == 24.0
    pipeline = build_gst_pipeline_text(
        worker.config["url"],
        latency_ms=worker.config["rtsp_latency_ms"],
        decoder=worker.config["decoder"],
        transport=worker.config["rtsp_transport"],
    )
    assert "latency=25" in pipeline
    assert "protocols=tcp" in pipeline


def test_yolo_worker_request_stop_allows_child_cleanup() -> None:
    class FakeStopEvent:
        def __init__(self) -> None:
            self.set_called = False

        def set(self) -> None:
            self.set_called = True

    class FakeProcess:
        def __init__(self) -> None:
            self.terminate_called = False

        def is_alive(self) -> bool:
            return True

        def terminate(self) -> None:
            self.terminate_called = True

    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        enable_detection=False,
    )
    stop_event = FakeStopEvent()
    process = FakeProcess()
    worker._stop_event = stop_event
    worker._process = process

    worker.request_stop()

    assert stop_event.set_called
    assert not process.terminate_called


def test_record_video_writer_runs_on_background_thread(tmp_path) -> None:
    class FakeVideoWriter:
        written = 0
        released = False
        thread_ids: list[int] = []

        def __init__(self, *_args):
            pass

        def isOpened(self):
            return True

        def write(self, _frame):
            type(self).written += 1
            type(self).thread_ids.append(threading.get_ident())

        def release(self):
            type(self).released = True

    class FakeCv2:
        INTER_AREA = 3
        VideoWriter = FakeVideoWriter

        @staticmethod
        def VideoWriter_fourcc(*_args):
            return 0

        @staticmethod
        def resize(frame, size, interpolation=None):
            _ = interpolation
            return np.zeros((size[1], size[0], 3), dtype=frame.dtype)

    class FakeOutputQueue:
        def __init__(self):
            self.messages = []

        def put(self, message):
            self.messages.append(message)

    output_queue = FakeOutputQueue()
    writer_state = _open_record_video_writer(
        FakeCv2,
        {
            "record_video_path": str(tmp_path / "left.mp4"),
            "record_video_fps": 30.0,
            "record_video_queue_size": 8,
        },
        output_queue,
        16,
        9,
    )
    assert writer_state is not None
    caller_thread_id = threading.get_ident()

    for _ in range(4):
        _write_record_video_frame(FakeCv2, writer_state, np.zeros((9, 16, 3), dtype=np.uint8))
    _release_record_video_writer(output_queue, writer_state)

    assert FakeVideoWriter.released
    assert FakeVideoWriter.written == 4
    assert FakeVideoWriter.thread_ids
    assert all(thread_id != caller_thread_id for thread_id in FakeVideoWriter.thread_ids)
    assert any(message[0] == "status" and "영상 녹화 시작" in message[1] for message in output_queue.messages)
    assert any(message[0] == "status" and "영상 녹화 저장 완료" in message[1] for message in output_queue.messages)


def test_yolo_worker_uses_cuda_thread_default_without_rate_limit() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cuda:0",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=15.0,
        preview_max_width=960,
        enable_detection=True,
    )

    assert worker.config["max_rate_hz"] == 0.0
    assert worker.config["adaptive_rate"] is False
    assert worker.config["worker_cpu_threads"] == DEFAULT_CUDA_WORKER_THREADS
    assert worker.config["event_queue_size"] == 128
    assert worker.config["frame_queue_size"] == 2


def test_yolo_worker_accepts_configurable_process_queue_sizes() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cuda:0",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        event_queue_size=256,
        frame_queue_size=4,
        enable_detection=True,
    )

    assert worker.config["event_queue_size"] == 256
    assert worker.config["frame_queue_size"] == 4


def test_yolo_worker_defaults_to_nvidia_decoder() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cuda:0",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=15.0,
        preview_max_width=960,
        enable_detection=True,
    )

    assert worker.config["decoder"] == DECODER_NVIDIA


def test_yolo_processing_skips_preview_overlay_when_not_displaying() -> None:
    import numpy as np

    class NoDrawCv2:
        def __getattr__(self, name):
            raise AssertionError(f"unexpected drawing call: {name}")

    class FakeOutputQueue:
        def __init__(self) -> None:
            self.messages = []

        def put(self, message) -> None:
            self.messages.append(message)

    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    output_queue = FakeOutputQueue()

    annotated, best_detection, inferred = _process_frame_for_output(
        NoDrawCv2(),
        frame,
        model=_FakeModel([_FakeBox((1, 2, 3, 4), 0.9, 2)]),
        config=_predict_config(),
        output_queue=output_queue,
        now_s=10.0,
        enable_detection=True,
        draw_preview_overlay=False,
    )

    assert annotated is frame
    assert best_detection is not None
    assert inferred is True
    assert output_queue.messages[0][0] == "detection"


def test_yolo_processing_falls_back_to_cpu_when_cuda_predict_fails() -> None:
    import numpy as np

    class FakeOutputQueue:
        def __init__(self) -> None:
            self.messages = []

        def put(self, message) -> None:
            self.messages.append(message)

    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    output_queue = FakeOutputQueue()
    model = _CudaFailsFakeModel([_FakeBox((1, 2, 3, 4), 0.9, 2)])
    config = _predict_config()

    annotated, best_detection, inferred = _process_frame_for_output(
        SimpleNamespace(),
        frame,
        model=model,
        config=config,
        output_queue=output_queue,
        now_s=10.0,
        enable_detection=True,
        draw_preview_overlay=False,
    )

    assert annotated is frame
    assert best_detection is not None
    assert inferred is True
    assert model.devices == ["cuda:0", "cpu"]
    assert config["device"] == "cpu"
    assert any(message[0] == "status" and "CPU" in message[1] for message in output_queue.messages)
    assert output_queue.messages[-1][0] == "detection"


def test_cuda_fallback_activates_cpu_adaptive_rate_limit() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cuda:0",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=True,
    )
    limiter = AdaptiveYoloRateLimiter.from_config(worker.config, enable_detection=True)

    worker.config["_yolo_cpu_fallback_active"] = True
    _activate_cpu_fallback_rate_limiter(limiter, worker.config)

    assert limiter.enabled is True
    assert limiter.target_rate_hz == DEFAULT_CPU_YOLO_INITIAL_RATE_HZ
    assert limiter.max_rate_hz == DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ
    assert worker.config["device"] == "cpu"


def test_yolo_processing_emits_measurement_frame_before_detection_when_enabled() -> None:
    import numpy as np

    class FakeCv2:
        COLOR_BGR2RGB = 4

        def cvtColor(self, image, code):
            assert code == self.COLOR_BGR2RGB
            return image

    class FakeOutputQueue:
        def __init__(self) -> None:
            self.messages = []

        def put(self, message) -> None:
            self.messages.append(message)

    frame = np.zeros((4, 5, 3), dtype=np.uint8)
    output_queue = FakeOutputQueue()
    config = {
        **_predict_config(),
        "emit_measurement_frames": True,
        "preview_max_width": 960,
    }

    annotated, best_detection, inferred = _process_frame_for_output(
        FakeCv2(),
        frame,
        model=_FakeModel([_FakeBox((1, 2, 3, 4), 0.9, 2)]),
        config=config,
        output_queue=output_queue,
        now_s=10.0,
        enable_detection=True,
        draw_preview_overlay=False,
    )

    assert annotated is frame
    assert best_detection is not None
    assert inferred is True
    assert output_queue.messages[0][:4] == ("measurement_frame", 10.0, 5, 4)
    assert output_queue.messages[1][0] == "detection"


def test_yolo_processing_draws_only_best_box_for_preview_overlay() -> None:
    import numpy as np

    class FakeCv2:
        FONT_HERSHEY_SIMPLEX = 0
        LINE_AA = 16
        MARKER_CROSS = 0

        def __init__(self) -> None:
            self.rectangles = []
            self.markers = []
            self.texts = []

        def getTextSize(self, *_args):
            return (20, 8), 2

        def rectangle(self, _image, top_left, bottom_right, *_args):
            self.rectangles.append((top_left, bottom_right))

        def putText(self, _image, text, *_args):
            self.texts.append(text)
            return None

        def drawMarker(self, _image, position, *_args, **_kwargs):
            self.markers.append(position)

    class FakeOutputQueue:
        def put(self, _message) -> None:
            return None

    cv2 = FakeCv2()
    frame = np.zeros((80, 80, 3), dtype=np.uint8)

    annotated, best_detection, inferred = _process_frame_for_output(
        cv2,
        frame,
        model=_FakeModel(
            [
                _FakeBox((1, 2, 3, 4), 0.2, 1),
                _FakeBox((30, 40, 50, 60), 0.95, 2),
            ],
            names={1: "car", 2: "person"},
        ),
        config=_predict_config(),
        output_queue=FakeOutputQueue(),
        now_s=10.0,
        enable_detection=True,
        draw_preview_overlay=True,
    )

    assert annotated is not frame
    assert best_detection is not None
    assert best_detection.class_id == 2
    assert best_detection.class_name == "person"
    assert inferred is True
    assert ((30, 40), (50, 60)) in cv2.rectangles
    assert ((1, 2), (3, 4)) not in cv2.rectangles
    assert cv2.markers == [(40, 50)]
    assert any(text.startswith("BEST person 0.95") for text in cv2.texts)


def test_yolo_worker_enables_cpu_adaptive_rate_limit_by_default() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=20.0,
        preview_max_width=960,
        enable_detection=True,
    )

    limiter = AdaptiveYoloRateLimiter.from_config(worker.config, enable_detection=True)
    assert worker.config["max_rate_hz"] == DEFAULT_CPU_YOLO_MAX_RATE_HZ
    assert worker.config["adaptive_rate"] is True
    assert worker.config["adaptive_max_rate_hz"] == DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ
    assert limiter.enabled
    assert limiter.target_rate_hz == DEFAULT_CPU_YOLO_INITIAL_RATE_HZ
    assert limiter.max_rate_hz == DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ


def test_yolo_worker_normalizes_input_size_before_predict() -> None:
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=720,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=20.0,
        preview_max_width=960,
        enable_detection=True,
    )

    assert worker.config["input_size"] == 736


def test_yolo_worker_emits_frame_metadata(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=30.0,
        display_rate_hz=15.0,
        preview_max_width=960,
        rtsp_latency_ms=25,
        decoder="software",
        stream_backend="opencv",
        rtsp_transport="tcp",
        enable_detection=False,
    )
    received = []
    worker.frame_ready.connect(lambda image, metadata: received.append((image, metadata)))

    worker._handle_worker_message(
        (
            "frame",
            {"timestamp_s": 123.0, "source_timestamp_s": 5.5, "timestamp_source": "gstreamer-pts"},
            2,
            1,
            bytes([255, 0, 0, 0, 255, 0]),
        )
    )

    assert len(received) == 1
    image, metadata = received[0]
    assert image.width() == 2
    assert image.height() == 1
    assert metadata["timestamp_s"] == 123.0
    assert metadata["source_timestamp_s"] == 5.5
    assert metadata["timestamp_source"] == "gstreamer-pts"
    _ = app


def test_yolo_worker_emits_target_rate_in_stats(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=20.0,
        preview_max_width=960,
        enable_detection=True,
    )
    received = []
    worker.stats_ready.connect(received.append)

    worker._handle_worker_message(("stats", 18.0, 7.0, 7.0, 1280, 720, 9.5))

    assert len(received) == 1
    assert received[0]["decoded_hz"] == 18.0
    assert received[0]["inferred_hz"] == 7.0
    assert received[0]["yolo_target_hz"] == 9.5
    assert received[0]["yolo_requested_device"] == "cpu"
    assert received[0]["yolo_actual_device"] == ""
    assert received[0]["yolo_cpu_fallback"] is False
    _ = app


def test_yolo_worker_emits_actual_device_and_cpu_fallback_in_stats(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    worker = YoloStreamWorker(
        url="rtsp://10.0.0.232:8554/visible",
        weights_path="weights.pt",
        device="cuda:0",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        display_rate_hz=20.0,
        preview_max_width=960,
        enable_detection=True,
    )
    received = []
    worker.stats_ready.connect(received.append)

    worker._handle_worker_message(("stats", 18.0, 7.0, 7.0, 1280, 720, 9.5, "cuda:0", "cpu", True))

    assert len(received) == 1
    assert received[0]["yolo_requested_device"] == "cuda:0"
    assert received[0]["yolo_actual_device"] == "cpu"
    assert received[0]["yolo_cpu_fallback"] is True
    _ = app


def test_opencv_preview_hz_is_limited_without_limiting_capture_or_detection() -> None:
    stop_event = threading.Event()
    capture = _FastFakeCapture()
    stats_messages = []

    class _StatsQueue:
        def put(self, message):
            if message[0] == "stats":
                stats_messages.append(message)
                stop_event.set()

    fake_cv2 = SimpleNamespace(
        CAP_ANY=0,
        CAP_FFMPEG=1,
        CAP_PROP_BUFFERSIZE=38,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        COLOR_BGR2RGB=4,
        VideoCapture=lambda *_args: capture,
        cvtColor=lambda frame, _code: frame,
    )

    _run_opencv_stream(
        {
            "url": "rtsp://example/stream",
            "rtsp_transport": "tcp",
            "stream_backend": "opencv",
            "display_rate_hz": 10.0,
            "max_rate_hz": 5.0,
            "adaptive_rate": False,
            "input_size": 640,
            "conf_threshold": 0.5,
            "iou_threshold": 0.45,
            "device": "cpu",
            "preview_max_width": 0,
        },
        _StatsQueue(),
        stop_event,
        fake_cv2,
        _SlowFakeModel([]),
        True,
    )

    assert capture.released
    assert stats_messages
    (
        _kind,
        decoded_hz,
        displayed_hz,
        inferred_hz,
        width,
        height,
        _target_rate_hz,
        requested_device,
        actual_device,
        cpu_fallback,
    ) = stats_messages[0]
    assert width == 32
    assert height == 24
    assert decoded_hz > inferred_hz * 2.0
    assert displayed_hz <= 12.0
    assert decoded_hz > displayed_hz * 2.0
    assert requested_device == "cpu"
    assert actual_device == "cpu"
    assert cpu_fallback is False
