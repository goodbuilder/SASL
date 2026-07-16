from types import SimpleNamespace
import inspect
import queue
import threading
import time

import numpy as np
import pytest
import wiris_stereo_distance.yolo_stream as yolo_stream_module

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
    DEFAULT_FIRST_KEYFRAME_TIMEOUT_MS,
    DEFAULT_RECORDING_EOS_TIMEOUT_MS,
    DECODER_NVIDIA,
    NVIDIA_H264_DECODER,
    NTP_UNIX_EPOCH_OFFSET_S,
    SOFTWARE_H264_DECODER,
    DEFAULT_RTSP_LATENCY_MS,
    DetectionBox,
    GstRtpTimingTracker,
    MAX_RTP_TIMING_CACHE,
    STREAM_BACKEND_OPENCV_AUTO,
    STREAM_BACKEND_PYAV,
    RtpFrameTiming,
    RtcpSenderReportTiming,
    YoloStreamWorker,
    _CanonicalGstRecordingSession,
    _BoundedProcessOutputQueue,
    _gst_string_literal,
    _run_gstreamer_stream,
    _open_record_video_writer,
    _process_frame_for_output,
    _release_record_video_writer,
    _run_opencv_stream,
    _write_record_video_frame,
    filter_measurement_detections,
    build_ffmpeg_recording_command,
    build_gst_pipeline_text,
    build_gst_recording_pipeline_text,
    decoder_candidates,
    effective_yolo_max_rate_hz,
    estimate_rtcp_ntp_unix_seconds,
    gst_buffer_is_countable_au,
    ntp64_to_signed_int64,
    ntp64_to_unix_ns,
    ntp64_to_unix_seconds,
    normalize_yolo_input_size,
    normalize_rtsp_transport,
    normalize_stream_backend,
    rtp_timestamp_delta,
    signed_int64_to_ntp64,
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


def test_process_output_queue_drops_preview_frames_without_dropping_detection_events() -> None:
    event_queue: queue.Queue = queue.Queue(maxsize=1)
    frame_queue: queue.Queue = queue.Queue(maxsize=1)
    output_queue = _BoundedProcessOutputQueue(event_queue, frame_queue)

    output_queue.put(("frame", {}, 1, 1, b"a"))
    output_queue.put(("frame", {}, 1, 1, b"b"))
    output_queue.put(("detection", 1.0, 1, 1, {"confidence": 0.9}, [], {}))

    assert frame_queue.qsize() == 1
    assert frame_queue.get_nowait()[4] == b"a"
    event = event_queue.get_nowait()
    assert event[0] == "detection"
    assert event[4]["confidence"] == 0.9


def test_measurement_detection_gates_filter_class_and_bbox_size() -> None:
    detections = [
        DetectionBox(0.0, 0.0, 20.0, 20.0, 0.95, 0, "person"),
        DetectionBox(0.0, 0.0, 80.0, 60.0, 0.80, 1, "vehicle"),
    ]

    filtered = filter_measurement_detections(
        detections,
        {
            "gate_class_enabled": True,
            "gate_class_text": "vehicle",
            "gate_min_bbox_enabled": True,
            "gate_min_width_px": 40,
            "gate_min_height_px": 40,
            "gate_min_area_px": 2_000,
        },
    )

    assert filtered == [detections[1]]


def test_gstreamer_url_is_quoted_as_one_property_value() -> None:
    malicious = 'rtsp://camera/stream" ! fakesink name=unexpected'

    pipeline = build_gst_pipeline_text(malicious)

    assert 'location="rtsp://camera/stream\\" ! fakesink name=unexpected"' in pipeline


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
    assert "queue leaky=downstream max-size-buffers=1" in pipeline
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
    assert message == (
        "[RTCP] Sender Report 갱신: ssrc=1234, interval=2.50s, ntp=102.000000"
    )
    tracker._emit_status(message)
    assert messages == [message]


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


def test_rtp_timing_and_preview_caches_remain_bounded_and_order_stays_coherent() -> None:
    tracker = GstRtpTimingTracker(_FakeGst, SimpleNamespace())
    total = MAX_RTP_TIMING_CACHE + 25
    for index in range(total):
        tracker._pending_rtp_frames.append(
            RtpFrameTiming(
                rtp_timestamp=index,
                rtp_ssrc=1,
                rtp_marker_seq=index,
                rtp_pts_s=None,
            )
        )
        tracker._map_depay_buffer_pts(None, _FakeProbeInfo(_FakeBuffer(pts=index, dts=index)))
        tracker._cache_preview_metadata(index, {"rtp_timestamp": index})

    assert len(tracker._timing_by_pts_ns) == MAX_RTP_TIMING_CACHE
    assert len(tracker._pts_order) == MAX_RTP_TIMING_CACHE
    assert set(tracker._pts_order) == set(tracker._timing_by_pts_ns)
    assert len(tracker._preview_metadata_by_pts_ns) == MAX_RTP_TIMING_CACHE
    assert len(tracker._preview_pts_order) == MAX_RTP_TIMING_CACHE
    assert 0 not in tracker._timing_by_pts_ns
    assert 0 not in tracker._preview_metadata_by_pts_ns

    newest = total - 1
    assert tracker._pop_frame_timing(newest).rtp_timestamp == newest
    assert newest not in tracker._pts_order
    assert set(tracker._pts_order) == set(tracker._timing_by_pts_ns)


def test_gstreamer_pipeline_can_drop_delta_frames_for_keyframe_only_diagnostics() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", keyframe_only=True)

    assert "h264parse name=au_parser config-interval=-1 disable-passthrough=true" in pipeline
    assert "video/x-h264,alignment=au" in pipeline
    assert "identity drop-buffer-flags=delta-unit" in pipeline


def test_gstreamer_pipeline_records_encoded_h264_without_reencoding(tmp_path) -> None:
    record_path = tmp_path / "raw left.mkv"
    pipeline = build_gst_pipeline_text(
        "rtsp://10.0.0.232:8554/visible",
        record_video_path=str(record_path),
    )

    assert "! tee name=encoded" in pipeline
    assert "encoded. ! valve name=record_valve drop=true" in pipeline
    assert "! queue name=record_queue" in pipeline
    assert "! identity name=mux_ingress" in pipeline
    assert "! matroskamux name=record_mux" in pipeline
    assert f'location="{record_path}"' in pipeline
    assert pipeline.count("h264parse") == 1
    assert "! videoconvert ! video/x-raw,format=BGR ! appsink" in pipeline


def test_session_pipeline_uses_one_rtsp_source_and_shared_tee(tmp_path) -> None:
    pipeline = build_gst_pipeline_text(
        "rtsp://camera/visible",
        record_video_path=str(tmp_path / "left.mkv"),
    )

    assert pipeline.count("rtspsrc ") == 1
    assert "h264parse name=au_parser config-interval=-1 disable-passthrough=true" in pipeline
    assert "video/x-h264,stream-format=avc,alignment=au ! tee name=encoded" in pipeline
    assert pipeline.index("tee name=encoded") < pipeline.index("queue leaky=downstream")
    assert "encoded. ! valve name=record_valve drop=true" in pipeline
    record_branch = pipeline.split("encoded. ! valve name=record_valve drop=true", 1)[1]
    assert "leaky=" not in record_branch
    assert "drop=true" not in record_branch


def test_session_path_uses_gstreamer_string_escaping() -> None:
    path = 'folder/line\nquote"slash\\bang!.mkv'
    pipeline = build_gst_pipeline_text(
        "rtsp://camera/visible",
        decoder=SOFTWARE_H264_DECODER,
        record_video_path=path,
    )

    assert 'location="folder/line\nquote\\"slash\\\\bang!.mkv"' in pipeline
    gi = pytest.importorskip("gi", reason="GStreamer Python bindings are unavailable")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    Gst.init(None)
    try:
        parsed = Gst.parse_launch(pipeline)
    except Exception as exc:
        pytest.skip(f"required GStreamer plugins are unavailable: {exc}")
    assert parsed.get_by_name("record_sink").get_property("location") == path


class _FakeGst:
    CLOCK_TIME_NONE = -1

    class BufferFlags:
        HEADER = 1
        GAP = 2
        CORRUPTED = 4
        DELTA_UNIT = 8

    class PadProbeReturn:
        OK = "ok"
        DROP = "drop"

    class MapFlags:
        READ = 1


class _FakeBuffer:
    def __init__(self, *, pts=0, dts=0, flags=0, size=1) -> None:
        self.pts = pts
        self.dts = dts
        self._flags = flags
        self._size = size

    def get_flags(self):
        return self._flags

    def get_size(self):
        return self._size


class _FakeAvcBuffer(_FakeBuffer):
    def __init__(self, payload: bytes, *, pts=0, dts=0, flags=0) -> None:
        super().__init__(pts=pts, dts=dts, flags=flags, size=len(payload))
        self.payload = payload

    def map(self, _flags):
        return True, SimpleNamespace(data=self.payload)

    def unmap(self, _map_info) -> None:
        return None


def _avc_payload(*nal_types: int) -> bytes:
    payload = bytearray()
    for nal_type in nal_types:
        nal = bytes([nal_type & 0x1F, 0x00])
        payload.extend(len(nal).to_bytes(4, "big"))
        payload.extend(nal)
    return bytes(payload)


class _FakePad:
    def __init__(self, width=1280, height=720) -> None:
        self.width = width
        self.height = height

    def get_current_caps(self):
        values = {"width": self.width, "height": self.height}
        return SimpleNamespace(get_structure=lambda _index: SimpleNamespace(get_value=values.get))


class _FakeProbeInfo:
    def __init__(self, buffer) -> None:
        self.buffer = buffer

    def get_buffer(self):
        return self.buffer


class _FakeValve:
    def __init__(self) -> None:
        self.drop = True

    def set_property(self, name, value) -> None:
        assert name == "drop"
        self.drop = bool(value)


class _FakeTimingTracker:
    def __init__(self) -> None:
        self.calls = 0

    def consume_authority_metadata(self, _buffer, *, now_monotonic_ns):
        assert isinstance(now_monotonic_ns, int)
        self.calls += 1
        return {"rtp_timestamp": 90_000 + self.calls, "rtp_ssrc": 7, "rtp_marker_seq": self.calls}


class _FakeControl:
    def __init__(self) -> None:
        self.payloads = []

    def send(self, payload) -> None:
        self.payloads.append(payload)


def _recording_session(*, data_queue=None, control=None, stop_event=None):
    control = control or _FakeControl()
    state = _CanonicalGstRecordingSession(
        _FakeGst,
        config={
            "session_id": "session-1",
            "camera_side": "left",
            "camera_stream_id": "left-visible",
            "record_video_path": "/tmp/left.mkv",
            "recording_event_put_timeout_s": 0.001,
        },
        rtp_timing_tracker=_FakeTimingTracker(),
        recording_data_queue=data_queue or queue.Queue(maxsize=32),
        recording_control_conn=control,
        stop_event=stop_event or threading.Event(),
        output_queue=SimpleNamespace(put=lambda _message: None),
    )
    state._valve = _FakeValve()
    return state, control


def _canonical_runtime_config(*, keyframe_only: bool = False) -> dict:
    worker = YoloStreamWorker(
        url="rtsp://camera/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        stream_backend="gstreamer",
        record_video_path="/tmp/canonical-left.mkv",
        session_id="session-runtime",
        camera_side="left",
        camera_stream_id="left-visible",
    )
    config = dict(worker.config)
    config["keyframe_only"] = bool(keyframe_only)
    return config


def test_header_gap_corrupted_buffers_are_not_counted() -> None:
    for flag in (
        _FakeGst.BufferFlags.HEADER,
        _FakeGst.BufferFlags.GAP,
        _FakeGst.BufferFlags.CORRUPTED,
    ):
        assert not gst_buffer_is_countable_au(_FakeGst, _FakeBuffer(flags=flag))
    assert gst_buffer_is_countable_au(_FakeGst, _FakeBuffer(flags=_FakeGst.BufferFlags.DELTA_UNIT))


def test_header_flag_with_vcl_is_counted_but_config_only_is_not() -> None:
    header = _FakeGst.BufferFlags.HEADER
    header_with_idr = _FakeAvcBuffer(_avc_payload(7, 8, 5), flags=header)
    config_only = _FakeAvcBuffer(_avc_payload(7, 8), flags=header)
    unflagged_config_only = _FakeAvcBuffer(_avc_payload(9, 7, 8, 6))

    assert gst_buffer_is_countable_au(_FakeGst, header_with_idr)
    assert not gst_buffer_is_countable_au(_FakeGst, config_only)
    assert not gst_buffer_is_countable_au(_FakeGst, unflagged_config_only)
    assert not gst_buffer_is_countable_au(
        _FakeGst,
        _FakeAvcBuffer(_avc_payload(5), flags=_FakeGst.BufferFlags.CORRUPTED),
    )


def test_first_valid_keyframe_opens_valve_and_is_recorded() -> None:
    events = queue.Queue(maxsize=8)
    state, control = _recording_session(data_queue=events)
    pad = _FakePad()

    state._authority_probe(
        pad,
        _FakeProbeInfo(_FakeBuffer(pts=0, dts=0, flags=_FakeGst.BufferFlags.DELTA_UNIT)),
    )
    assert events.empty()
    assert state._valve.drop

    keyframe = _FakeBuffer(pts=1, dts=1)
    state._authority_probe(pad, _FakeProbeInfo(keyframe))
    state._mux_ingress_probe(pad, _FakeProbeInfo(keyframe))
    source = events.get_nowait()
    mapping = events.get_nowait()

    assert not state._valve.drop
    assert source["event_kind"] == "camera_source"
    assert source["camera_sequence"] == source["frame_id"] == 0
    assert source["record_event_sequence"] == 0
    assert isinstance(source["pc_receive_monotonic_ns"], int)
    assert isinstance(source["pc_receive_utc_ns"], int)
    assert mapping["event_kind"] == "video_mapping"
    assert mapping["video_frame_index"] == 0
    assert mapping["record_event_sequence"] == 1
    assert [payload["control_kind"] for payload in control.payloads] == ["recording_ready"]


def test_three_encoded_aus_emit_contiguous_sources_and_mappings() -> None:
    events = queue.Queue(maxsize=16)
    state, _control = _recording_session(data_queue=events)
    pad = _FakePad()
    for index in range(3):
        buffer = _FakeBuffer(pts=index + 1, dts=index + 1)
        state._authority_probe(pad, _FakeProbeInfo(buffer))
        state._mux_ingress_probe(pad, _FakeProbeInfo(buffer))

    payloads = [events.get_nowait() for _ in range(6)]
    sources = [payload for payload in payloads if payload["event_kind"] == "camera_source"]
    mappings = [payload for payload in payloads if payload["event_kind"] == "video_mapping"]
    assert [payload["record_event_sequence"] for payload in payloads] == list(range(6))
    assert [payload["camera_sequence"] for payload in sources] == [0, 1, 2]
    assert [payload["video_frame_index"] for payload in mappings] == [0, 1, 2]
    assert not state._source_fifo


def test_duplicate_and_nonmonotonic_pts_are_fail_stop() -> None:
    state, control = _recording_session()
    pad = _FakePad()
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=2, dts=1)))
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=2, dts=2)))

    failure = [payload for payload in control.payloads if payload["control_kind"] == "recording_failure"]
    assert failure[-1]["failure_stage"] == "frame_reordering_detected"
    assert state.pts_reordering_observed
    assert state.stop_event.is_set()


def test_caps_change_is_failure() -> None:
    state, control = _recording_session()
    state._authority_probe(_FakePad(1280, 720), _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))
    state._authority_probe(_FakePad(1920, 1080), _FakeProbeInfo(_FakeBuffer(pts=2, dts=2)))

    assert state.caps_changed
    assert any(payload.get("failure_stage") == "camera_caps_changed" for payload in control.payloads)


def test_canonical_timeout_defaults_are_exact(monkeypatch) -> None:
    now_ns = 123_000_000
    monkeypatch.setattr(yolo_stream_module.time, "monotonic_ns", lambda: now_ns)
    state, _control = _recording_session()
    worker = YoloStreamWorker(
        url="rtsp://camera/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
    )

    assert DEFAULT_FIRST_KEYFRAME_TIMEOUT_MS == 10_000
    assert DEFAULT_RECORDING_EOS_TIMEOUT_MS == 3_000
    assert state.first_keyframe_deadline_ns - now_ns == 10_000_000_000
    assert worker.config["first_keyframe_timeout_ms"] == 10_000
    assert worker.config["recording_eos_timeout_ms"] == 3_000


def test_first_keyframe_deadline_failure_uses_injected_monotonic_clock(monkeypatch) -> None:
    now_ns = [1_000]
    monkeypatch.setattr(yolo_stream_module.time, "monotonic_ns", lambda: now_ns[0])
    state, control = _recording_session()
    now_ns[0] = state.first_keyframe_deadline_ns

    assert state.check_first_keyframe_timeout()
    assert state.stop_event.is_set()
    assert control.payloads[-1]["failure_stage"] == "first_keyframe_timeout"


@pytest.mark.parametrize(
    "pad",
    [
        _FakePad(0, 720),
        _FakePad(1280, 0),
        SimpleNamespace(get_current_caps=lambda: None),
    ],
)
def test_missing_or_zero_encoded_caps_fail_before_source_emission(pad) -> None:
    events = queue.Queue(maxsize=4)
    state, control = _recording_session(data_queue=events)

    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))

    assert events.empty()
    assert state.stop_event.is_set()
    assert control.payloads[-1]["failure_stage"] == "camera_caps_missing"


def test_decreasing_dts_is_rejected_even_when_pts_increases() -> None:
    state, control = _recording_session()
    pad = _FakePad()
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=1, dts=2)))
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=2, dts=1)))

    assert state.pts_reordering_observed
    assert state.stop_event.is_set()
    assert control.payloads[-1]["failure_stage"] == "frame_reordering_detected"
    assert "DTS decreased" in control.payloads[-1]["detail"]


def test_rtp_association_mismatch_threshold_is_enforced() -> None:
    events = queue.Queue(maxsize=8)
    state, control = _recording_session(data_queue=events)
    state.rtp_timing_tracker = SimpleNamespace(
        consume_authority_metadata=lambda _buffer, **_kwargs: None
    )
    state.rtp_association_mismatch_limit = 1
    pad = _FakePad()

    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))
    assert state.rtp_association_mismatches == 1
    assert not state.stop_event.is_set()
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=2, dts=2)))

    assert state.rtp_association_mismatches == 2
    assert state.stop_event.is_set()
    assert control.payloads[-1]["failure_stage"] == "rtp_au_association_failed"


def test_mux_ingress_fifo_underflow_is_fail_stop() -> None:
    state, control = _recording_session()

    state._mux_ingress_probe(_FakePad(), _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))

    assert state.stop_event.is_set()
    assert control.payloads[-1]["failure_stage"] == "source_mapping_fifo_underflow"


def test_eos_terminal_reports_authority_fifo_residual_as_incomplete() -> None:
    state, control = _recording_session()
    state._authority_probe(_FakePad(), _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))

    terminal = state.send_terminal(eos_ok=True)

    assert terminal["terminal_status"] == "incomplete"
    assert terminal["eos_ok"] is True
    assert terminal["authority_fifo_residual"] == 1
    assert "authority_fifo_residual" in terminal["failure_stages"]
    assert control.payloads[-1] == terminal


def test_recording_ipc_timeout_is_fail_stop_and_terminal_control_survives() -> None:
    data_queue = queue.Queue(maxsize=1)
    data_queue.put({"occupied": True})
    stop_event = threading.Event()
    state, control = _recording_session(data_queue=data_queue, stop_event=stop_event)

    state._authority_probe(_FakePad(), _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))
    terminal = state.send_terminal(eos_ok=False, eos_error="backpressure")

    assert stop_event.is_set()
    assert terminal["ipc_backpressure"]
    assert terminal["terminal_status"] == "incomplete"
    assert [payload["control_kind"] for payload in control.payloads] == [
        "recording_failure",
        "recording_terminal",
    ]


def test_recording_ipc_timeout_is_fail_stop() -> None:
    data_queue = queue.Queue(maxsize=1)
    data_queue.put("occupied")
    state, control = _recording_session(data_queue=data_queue)
    state._authority_probe(_FakePad(), _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))
    assert state.stop_event.is_set()
    assert control.payloads[-1]["failure_stage"] == "recording_ipc_backpressure"


def test_ntp64_signed_storage_round_trip_and_integer_unix_ns() -> None:
    high_bit_raw = (3_900_000_000 << 32) | 0xFFFF_FFFF
    stored = ntp64_to_signed_int64(high_bit_raw)

    assert stored < 0
    assert signed_int64_to_ntp64(stored) == high_bit_raw
    assert ntp64_to_unix_ns(NTP_UNIX_EPOCH_OFFSET_S << 32) == 0
    half_second = (NTP_UNIX_EPOCH_OFFSET_S << 32) | (1 << 31)
    assert ntp64_to_unix_ns(half_second) == 500_000_000
    carry = (NTP_UNIX_EPOCH_OFFSET_S << 32) | 0xFFFF_FFFF
    assert ntp64_to_unix_ns(carry) == 1_000_000_000


def test_ntp64_signed_storage_round_trip() -> None:
    raw = (3_900_000_000 << 32) | 123
    assert signed_int64_to_ntp64(ntp64_to_signed_int64(raw)) == raw


def test_timing_tracker_consumes_authority_once_and_preserves_preview_copy() -> None:
    Gst = SimpleNamespace(CLOCK_TIME_NONE=-1)
    tracker = GstRtpTimingTracker(Gst, SimpleNamespace())
    timing = RtpFrameTiming(
        rtp_timestamp=90_000,
        rtp_ssrc=1234,
        rtp_marker_seq=10,
        rtp_pts_s=1.0,
    )
    with tracker._lock:
        tracker._timing_by_pts_ns[1_000_000_000] = timing
    buffer = SimpleNamespace(pts=1_000_000_000)

    authority = tracker.consume_authority_metadata(buffer, now_monotonic_ns=2_000_000_000)
    preview_first = tracker.preview_metadata(buffer)
    preview_second = tracker.preview_metadata(buffer)

    assert authority["rtp_timestamp"] == 90_000
    assert tracker._pop_frame_timing(1_000_000_000) is None
    assert preview_first == authority
    assert preview_second == authority


def test_session_record_branch_is_non_leaky(tmp_path) -> None:
    pipeline = build_gst_pipeline_text("rtsp://camera/visible", record_video_path=str(tmp_path / "left.mkv"))
    record_branch = pipeline.split("encoded. ! valve name=record_valve drop=true", 1)[1]
    assert "leaky=" not in record_branch
    assert "max-size-buffers=0" in record_branch


def test_session_record_branch_starts_with_closed_valve(tmp_path) -> None:
    pipeline = build_gst_pipeline_text("rtsp://camera/visible", record_video_path=str(tmp_path / "left.mkv"))
    assert "encoded. ! valve name=record_valve drop=true" in pipeline


def test_delta_frames_before_first_keyframe_are_not_counted_or_recorded() -> None:
    events = queue.Queue(maxsize=8)
    state, _control = _recording_session(data_queue=events)
    result = state._authority_probe(
        _FakePad(),
        _FakeProbeInfo(_FakeBuffer(pts=1, dts=1, flags=_FakeGst.BufferFlags.DELTA_UNIT)),
    )
    assert result == _FakeGst.PadProbeReturn.OK
    assert state._camera_sequence == state._video_frame_index == 0
    assert state._valve.drop
    assert events.empty()


def test_session_never_starts_secondary_rtsp_recorder() -> None:
    source = inspect.getsource(_run_gstreamer_stream)
    assert "recording_process = None if canonical_recording else _start_ffmpeg_encoded_recording_process" in source
    assert "_start_gst_encoded_recording_process(config" not in source


def test_session_rejects_keyframe_only_mode() -> None:
    source = inspect.getsource(_run_gstreamer_stream)
    rejection = source.index("keyframe_only_mode_not_supported")
    launch = source.index("_launch_gst_pipeline")
    assert rejection < launch


def test_canonical_runtime_rejects_keyframe_only_before_pipeline_launch(monkeypatch) -> None:
    pytest.importorskip("gi", reason="GStreamer Python bindings are unavailable")
    launches = []
    control_payloads = []
    monkeypatch.setattr(
        yolo_stream_module,
        "_launch_gst_pipeline",
        lambda *_args, **_kwargs: launches.append(True),
    )
    monkeypatch.setattr(
        yolo_stream_module,
        "_start_ffmpeg_encoded_recording_process",
        lambda *_args, **_kwargs: pytest.fail("secondary FFmpeg recorder was started"),
    )

    _run_gstreamer_stream(
        _canonical_runtime_config(keyframe_only=True),
        SimpleNamespace(put=lambda _message: None),
        threading.Event(),
        None,
        None,
        False,
        queue.Queue(maxsize=8),
        SimpleNamespace(send=control_payloads.append),
    )

    assert launches == []
    assert [payload["control_kind"] for payload in control_payloads] == [
        "recording_failure",
        "recording_terminal",
    ]
    assert control_payloads[0]["failure_stage"] == "keyframe_only_mode_not_supported"
    assert control_payloads[1]["terminal_status"] == "incomplete"


def test_canonical_runtime_uses_no_secondary_recorder_and_orders_eos_terminal_null(monkeypatch) -> None:
    pytest.importorskip("gi", reason="GStreamer Python bindings are unavailable")
    order = []
    launch_kwargs = []
    finalize_kwargs = []
    control_payloads = []
    secondary_calls = []

    class FakePipeline:
        def set_state(self, _state) -> None:
            order.append("null")

    class OrderedControl:
        def send(self, payload) -> None:
            control_payloads.append(payload)
            if payload.get("control_kind") == "recording_terminal":
                order.append("terminal")

    def fake_launch(*_args, **kwargs):
        launch_kwargs.append(kwargs)
        return FakePipeline(), SimpleNamespace(), SimpleNamespace(), "software"

    def fake_finalize(*_args, **kwargs):
        finalize_kwargs.append(kwargs)
        order.append("eos")
        return True, ""

    monkeypatch.setattr(yolo_stream_module, "_launch_gst_pipeline", fake_launch)
    monkeypatch.setattr(yolo_stream_module, "_finalize_gst_recording", fake_finalize)
    monkeypatch.setattr(
        yolo_stream_module,
        "_start_ffmpeg_encoded_recording_process",
        lambda *_args, **_kwargs: secondary_calls.append("ffmpeg"),
    )
    monkeypatch.setattr(
        yolo_stream_module,
        "_start_gst_encoded_recording_process",
        lambda *_args, **_kwargs: secondary_calls.append("gstreamer"),
    )
    stop_event = threading.Event()
    stop_event.set()

    _run_gstreamer_stream(
        _canonical_runtime_config(),
        SimpleNamespace(put=lambda _message: None),
        stop_event,
        None,
        None,
        False,
        queue.Queue(maxsize=8),
        OrderedControl(),
    )

    assert secondary_calls == []
    assert len(launch_kwargs) == 1
    assert launch_kwargs[0]["record_video_path"] == "/tmp/canonical-left.mkv"
    assert launch_kwargs[0]["recording_session"] is not None
    assert finalize_kwargs == [{"timeout_ms": 3_000}]
    assert order == ["eos", "terminal", "null"]
    assert control_payloads[-1]["terminal_status"] == "complete"


def test_duplicate_pts_is_rejected() -> None:
    state, control = _recording_session()
    pad = _FakePad()
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))
    assert any(payload.get("failure_stage") == "frame_reordering_detected" for payload in control.payloads)


def test_nonmonotonic_pts_is_rejected() -> None:
    state, control = _recording_session()
    pad = _FakePad()
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=2, dts=1)))
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=1, dts=2)))
    assert any(payload.get("failure_stage") == "frame_reordering_detected" for payload in control.payloads)


def test_b_frame_stream_is_rejected() -> None:
    state, control = _recording_session()
    pad = _FakePad()
    for pts, dts in ((0, 0), (2, 1), (1, 2)):
        state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=pts, dts=dts)))
    assert state.pts_reordering_observed
    assert any(payload.get("failure_stage") == "frame_reordering_detected" for payload in control.payloads)


def test_decode_drop_does_not_drop_recording_events() -> None:
    preview_events: queue.Queue = queue.Queue(maxsize=1)
    preview_frames: queue.Queue = queue.Queue(maxsize=1)
    preview_output = _BoundedProcessOutputQueue(preview_events, preview_frames)
    preview_output.put(("frame", {}, 1, 1, b"old"))
    preview_output.put(("frame", {}, 1, 1, b"dropped"))
    recording_events = queue.Queue(maxsize=8)
    state, _control = _recording_session(data_queue=recording_events)
    buffer = _FakeBuffer(pts=1, dts=1)
    state._authority_probe(_FakePad(), _FakeProbeInfo(buffer))
    state._mux_ingress_probe(_FakePad(), _FakeProbeInfo(buffer))
    assert recording_events.qsize() == 2


def test_recording_events_survive_preview_queue_saturation() -> None:
    event_queue: queue.Queue = queue.Queue(maxsize=1)
    frame_queue: queue.Queue = queue.Queue(maxsize=1)
    preview_output = _BoundedProcessOutputQueue(event_queue, frame_queue)
    preview_output.put(("status", "occupied"))
    preview_output.put(("status", "dropped"))
    recording_events = queue.Queue(maxsize=8)
    state, _control = _recording_session(data_queue=recording_events)
    buffer = _FakeBuffer(pts=1, dts=1)
    state._authority_probe(_FakePad(), _FakeProbeInfo(buffer))
    state._mux_ingress_probe(_FakePad(), _FakeProbeInfo(buffer))
    assert [recording_events.get_nowait()["event_kind"] for _ in range(2)] == [
        "camera_source",
        "video_mapping",
    ]


def test_terminal_control_survives_data_queue_saturation() -> None:
    data_queue = queue.Queue(maxsize=1)
    data_queue.put("occupied")
    state, control = _recording_session(data_queue=data_queue)
    state._authority_probe(_FakePad(), _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))
    state.send_terminal(eos_ok=False, eos_error="backpressure")
    assert control.payloads[-1]["control_kind"] == "recording_terminal"


def test_ntp64_to_unix_ns_uses_integer_arithmetic() -> None:
    raw = (NTP_UNIX_EPOCH_OFFSET_S << 32) | 1
    assert ntp64_to_unix_ns(raw) == 0
    assert "float" not in inspect.getsource(ntp64_to_unix_ns)


def test_eos_precedes_null() -> None:
    source = inspect.getsource(_run_gstreamer_stream)
    finalize = source.rindex("_finalize_gst_recording(")
    terminal = source.rindex("recording_session.send_terminal")
    pipeline_null = source.rindex("pipeline.set_state(Gst.State.NULL)")
    assert finalize < terminal < pipeline_null


def test_terminal_precedes_finished() -> None:
    worker = YoloStreamWorker(
        url="rtsp://camera/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
    )
    order = []
    worker.recording_terminal.connect(lambda _payload: order.append("terminal"))
    worker.finished.connect(lambda: order.append("finished"))
    worker._recording_terminal_payload = {"last_emitted_event_sequence": -1}
    worker._maybe_emit_recording_terminal()
    worker._emit_finished_once()
    assert order == ["terminal", "finished"]


def test_sequence_gap_emits_incomplete_terminal_before_finished(monkeypatch) -> None:
    worker = YoloStreamWorker(
        url="rtsp://camera/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        record_video_path="/tmp/left.mkv",
        session_id="session-1",
        camera_side="left",
    )
    worker._recording_data_queue = queue.Queue()
    worker._recording_terminal_payload = {
        "last_emitted_event_sequence": 1,
        "terminal_status": "complete",
    }
    monotonic_values = iter((0, 3_000_000_001))
    monkeypatch.setattr(time, "monotonic_ns", lambda: next(monotonic_values))
    order = []
    terminals = []
    worker.recording_terminal.connect(lambda payload: (terminals.append(payload), order.append("terminal")))
    worker.finished.connect(lambda: order.append("finished"))

    worker._finish_recording_channel_drain()
    worker._emit_finished_once()

    assert order == ["terminal", "finished"]
    assert terminals[0]["terminal_status"] == "incomplete"
    assert "recording_event_sequence_gap" in terminals[0]["failure_stages"]


@pytest.mark.parametrize("sequences", [(1,), (0, 2), (0, 1, 1)])
def test_worker_transport_requires_every_recording_sequence_to_be_contiguous(sequences) -> None:
    worker = YoloStreamWorker(
        url="rtsp://camera/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        record_video_path="/tmp/left.mkv",
        session_id="session-1",
        camera_side="left",
    )
    worker._recording_data_queue = queue.Queue()
    for sequence in sequences:
        worker._recording_data_queue.put(
            {
                "event_kind": "camera_source",
                "record_event_sequence": sequence,
            }
        )
    worker._recording_terminal_payload = {
        "last_emitted_event_sequence": sequences[-1],
        "terminal_status": "complete",
        "failure_stages": (),
        "failure_details": (),
    }
    failures = []
    terminals = []
    worker.recording_failure.connect(failures.append)
    worker.recording_terminal.connect(terminals.append)

    worker._finish_recording_channel_drain()

    assert len(failures) == 1
    assert failures[0]["failure_stage"] == "recording_event_sequence_gap"
    assert terminals[0]["terminal_status"] == "incomplete"
    assert "recording_event_sequence_gap" in terminals[0]["failure_stages"]


def test_stop_accepting_drops_new_aus_without_closing_eos_valve() -> None:
    state, _control = _recording_session()
    pad = _FakePad()
    state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=1, dts=1)))
    assert not state._valve.drop
    state.stop_accepting()
    assert not state._valve.drop
    result = state._authority_probe(pad, _FakeProbeInfo(_FakeBuffer(pts=2, dts=2)))
    assert result == _FakeGst.PadProbeReturn.DROP


def test_synthetic_gstreamer_au_authority_records_every_frame(tmp_path) -> None:
    gi = pytest.importorskip("gi", reason="GStreamer Python bindings are unavailable")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    Gst.init(None)
    required = ("videotestsrc", "x264enc", "h264parse", "tee", "valve", "matroskamux", "filesink")
    missing = [name for name in required if Gst.ElementFactory.find(name) is None]
    if missing:
        pytest.skip(f"required synthetic GStreamer plugins are unavailable: {', '.join(missing)}")

    output_path = tmp_path / "synthetic.mkv"
    pipeline_text = (
        "videotestsrc num-buffers=5 is-live=false "
        "! video/x-raw,format=I420,width=320,height=240,framerate=20/1 "
        "! x264enc bframes=0 key-int-max=30 tune=zerolatency byte-stream=true "
        "! h264parse name=au_parser config-interval=-1 disable-passthrough=true "
        "! video/x-h264,stream-format=avc,alignment=au "
        "! tee name=encoded "
        "encoded. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        "! fakesink sync=false "
        "encoded. ! valve name=record_valve drop=true "
        "! queue max-size-buffers=0 max-size-bytes=0 max-size-time=0 "
        "! identity name=mux_ingress silent=true "
        "! matroskamux "
        f"! filesink location={_gst_string_literal(str(output_path))} sync=false async=false"
    )
    pipeline = Gst.parse_launch(pipeline_text)
    events = queue.Queue(maxsize=32)
    control = _FakeControl()
    session = _CanonicalGstRecordingSession(
        Gst,
        config={
            "session_id": "synthetic",
            "camera_side": "left",
            "camera_stream_id": "left-visible",
            "record_video_path": str(output_path),
        },
        rtp_timing_tracker=_FakeTimingTracker(),
        recording_data_queue=events,
        recording_control_conn=control,
        stop_event=threading.Event(),
        output_queue=SimpleNamespace(put=lambda _message: None),
    )
    session.attach(pipeline)
    pipeline.set_state(Gst.State.PLAYING)
    message = pipeline.get_bus().timed_pop_filtered(
        10 * Gst.SECOND,
        Gst.MessageType.EOS | Gst.MessageType.ERROR,
    )
    if message is not None and message.type == Gst.MessageType.ERROR:
        error, debug = message.parse_error()
        pytest.fail(f"synthetic GStreamer pipeline failed: {error}: {debug}")
    assert message is not None and message.type == Gst.MessageType.EOS
    terminal = session.send_terminal(eos_ok=True)
    pipeline.set_state(Gst.State.NULL)

    payloads = []
    while True:
        try:
            payloads.append(events.get_nowait())
        except queue.Empty:
            break
    sources = [payload for payload in payloads if payload["event_kind"] == "camera_source"]
    mappings = [payload for payload in payloads if payload["event_kind"] == "video_mapping"]
    assert terminal["terminal_status"] == "complete"
    assert [payload["camera_sequence"] for payload in sources] == list(range(5))
    assert [payload["video_frame_index"] for payload in mappings] == list(range(5))
    assert terminal["authority_fifo_residual"] == 0

    from wiris_stereo_distance.session_video import probe_mkv

    probe = probe_mkv(output_path, include_packets=True)
    assert probe.codec_name == "h264"
    assert probe.decoded_frame_count == len(sources) == len(mappings) == 5


def test_gstreamer_recording_pipeline_saves_encoded_h264_without_decode(tmp_path) -> None:
    record_path = tmp_path / "raw left.mkv"
    pipeline = build_gst_recording_pipeline_text(
        "rtsp://10.0.0.232:8554/visible",
        record_video_path=str(record_path),
    )

    assert "rtph264depay" in pipeline
    assert "! h264parse config-interval=-1" in pipeline
    assert "! matroskamux" in pipeline
    assert f"! filesink location='{record_path}' sync=false" in pipeline
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
    assert "+faststart" in command
    assert str(record_path) == command[-1]


def test_gstreamer_pipeline_can_use_nvidia_decoder() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", decoder="nvidia")

    assert NVIDIA_H264_DECODER in pipeline
    assert SOFTWARE_H264_DECODER not in pipeline


def test_gstreamer_pipeline_can_use_software_decoder() -> None:
    pipeline = build_gst_pipeline_text("rtsp://10.0.0.232:8554/visible", decoder="software")

    assert SOFTWARE_H264_DECODER in pipeline
    assert "skip-frame=1" in pipeline
    assert NVIDIA_H264_DECODER not in pipeline


def test_decoder_candidates_auto_falls_back_to_software() -> None:
    assert decoder_candidates("auto") == (NVIDIA_H264_DECODER, SOFTWARE_H264_DECODER)
    assert decoder_candidates("software") == (SOFTWARE_H264_DECODER,)


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
        record_video_queue_size=9,
        ffmpeg_stimeout_us=3_000_000,
        ffmpeg_max_delay_us=700_000,
    )

    assert worker.config["rtsp_latency_ms"] == 25
    assert worker.config["decoder"] == "software"
    assert worker.config["stream_backend"] == "opencv"
    assert worker.config["rtsp_transport"] == "tcp"
    assert worker.config["adaptive_rate"] is False
    assert worker.config["worker_cpu_threads"] == DEFAULT_CPU_WORKER_THREADS
    assert worker.config["record_video_path"] == "/tmp/stereo-left.mp4"
    assert worker.config["record_video_fps"] == 24.0
    assert worker.config["record_video_queue_size"] == 9
    assert worker.config["ffmpeg_stimeout_us"] == 3_000_000
    assert worker.config["ffmpeg_max_delay_us"] == 700_000
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


def test_stop_requested_before_worker_run_never_launches_child() -> None:
    worker = YoloStreamWorker(
        url="rtsp://camera/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        record_video_path="/tmp/left.mkv",
        session_id="session-early-stop",
        camera_side="left",
    )
    order = []
    worker.recording_terminal.connect(lambda payload: order.append(("terminal", payload["terminal_status"])))
    worker.finished.connect(lambda: order.append(("finished", None)))

    worker.request_stop()
    worker.run()

    assert worker._process is None
    assert order == [("terminal", "incomplete"), ("finished", None)]


def test_process_start_exception_emits_canonical_failure_terminal_before_finished() -> None:
    class FakeConnection:
        def close(self) -> None:
            pass

    class FailingProcess:
        pid = None
        exitcode = None

        def start(self) -> None:
            raise RuntimeError("spawn denied")

        def is_alive(self) -> bool:
            return False

    class FailingContext:
        @staticmethod
        def Event():
            return threading.Event()

        @staticmethod
        def Queue(*, maxsize):
            return queue.Queue(maxsize=maxsize)

        @staticmethod
        def Pipe(*, duplex):
            assert duplex is False
            return FakeConnection(), FakeConnection()

        @staticmethod
        def Process(**_kwargs):
            return FailingProcess()

    worker = YoloStreamWorker(
        url="rtsp://camera/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        record_video_path="/tmp/left.mkv",
        session_id="session-parent-failure",
        camera_side="left",
    )
    worker._mp_context = FailingContext()
    order = []
    failures = []
    terminals = []
    worker.recording_failure.connect(lambda payload: (failures.append(payload), order.append("failure")))
    worker.recording_terminal.connect(lambda payload: (terminals.append(payload), order.append("terminal")))
    worker.finished.connect(lambda: order.append("finished"))

    worker.run()

    assert order == ["failure", "terminal", "finished"]
    assert len(failures) == len(terminals) == 1
    assert failures[0]["failure_stage"] == "recording_worker_parent_failure"
    assert terminals[0]["terminal_status"] == "incomplete"
    assert terminals[0]["eos_ok"] is False
    assert terminals[0]["last_emitted_event_sequence"] == -1
    assert terminals[0]["failure_stages"] == ("recording_worker_parent_failure",)


def test_parent_queue_setup_exception_emits_canonical_failure_terminal_before_finished() -> None:
    class FailingSetupContext:
        @staticmethod
        def Event():
            return threading.Event()

        @staticmethod
        def Queue(*, maxsize):
            raise RuntimeError(f"queue allocation denied: {maxsize}")

    worker = YoloStreamWorker(
        url="rtsp://camera/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        record_video_path="/tmp/right.mkv",
        session_id="session-parent-setup-failure",
        camera_side="right",
    )
    worker._mp_context = FailingSetupContext()
    order = []
    worker.recording_failure.connect(lambda _payload: order.append("failure"))
    worker.recording_terminal.connect(lambda _payload: order.append("terminal"))
    worker.finished.connect(lambda: order.append("finished"))

    worker.run()

    assert order == ["failure", "terminal", "finished"]


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
        frame_id=42,
    )

    assert annotated is frame
    assert best_detection is not None
    assert inferred is True
    assert output_queue.messages[0][:5] == ("measurement_frame", 10.0, 42, 5, 4)
    assert output_queue.messages[0][6]["frame_id"] == 42
    assert output_queue.messages[1][0] == "detection"
    assert output_queue.messages[1][-1]["frame_id"] == 42


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


def test_opencv_receive_and_display_hz_are_not_yolo_or_preview_limited() -> None:
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
            "display_rate_hz": 1.0,
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
    assert displayed_hz > 1.0
    assert abs(displayed_hz - decoded_hz) < 0.001
    assert requested_device == "cpu"
    assert actual_device == "cpu"
    assert cpu_fallback is False
