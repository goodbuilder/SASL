from __future__ import annotations

import inspect
import pytest
import sys
import threading
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace


def _install_fake_gstreamer(monkeypatch) -> SimpleNamespace:
    fake_gst = SimpleNamespace(
        CLOCK_TIME_NONE=-1,
        MessageType=SimpleNamespace(ERROR=1, EOS=2),
        State=SimpleNamespace(NULL="NULL"),
        init=lambda _value: None,
    )
    fake_gst_rtp = SimpleNamespace()
    repository = ModuleType("gi.repository")
    repository.Gst = fake_gst
    repository.GstRtp = fake_gst_rtp
    gi = ModuleType("gi")
    gi.require_version = lambda *_args: None
    gi.repository = repository
    monkeypatch.setitem(sys.modules, "gi", gi)
    monkeypatch.setitem(sys.modules, "gi.repository", repository)
    return fake_gst


def _idle_integrity_tracker() -> SimpleNamespace:
    return SimpleNamespace(
        pop_h264_access_unit_discontinuity=lambda: None,
        pop_rtp_recovery=lambda: None,
        pop_rtp_discontinuity=lambda: None,
        decoded_output_blocked=lambda: False,
        blocked_duration_s=lambda **_kwargs: 0.0,
    )


def test_gstreamer_no_frame_watchdog_reopens_a_silent_pipeline(monkeypatch) -> None:
    from Workstation.workstation_core import yolo_stream

    _install_fake_gstreamer(monkeypatch)
    messages: list[object] = []
    pipeline_states: list[object] = []

    class OutputQueue:
        def put(self, message: object) -> None:
            messages.append(message)

    class SilentSink:
        def emit(self, *_args: object) -> None:
            time.sleep(0.005)
            return None

    class EmptyBus:
        def pop_filtered(self, _message_types: object) -> None:
            return None

    class Pipeline:
        def set_state(self, state: object) -> None:
            pipeline_states.append(state)

    monkeypatch.setattr(
        yolo_stream,
        "GstRtpTimingTracker",
        lambda *_args, **_kwargs: _idle_integrity_tracker(),
    )
    monkeypatch.setattr(
        yolo_stream,
        "_launch_gst_pipeline",
        lambda *_args, **_kwargs: (Pipeline(), SilentSink(), EmptyBus(), "fake-decoder"),
    )

    stop_event = threading.Event()
    fallback_stop = threading.Timer(0.5, stop_event.set)
    fallback_stop.start()
    try:
        started_s = time.monotonic()
        recovered = yolo_stream._run_gstreamer_stream_once(
            {
                "url": "rtsp://camera.invalid/visible",
                "rtsp_latency_ms": 100,
                "decoder": "software",
                "rtsp_transport": "tcp",
                "rtsp_frame_timeout_s": 0.05,
            },
            OutputQueue(),
            stop_event,
            SimpleNamespace(),
            None,
            False,
        )
        elapsed_s = time.monotonic() - started_s
    finally:
        fallback_stop.cancel()
        stop_event.set()

    assert recovered is False
    assert elapsed_s < 0.4
    assert pipeline_states == ["NULL"]
    assert any(
        "프레임 타임아웃" in str(message[1])
        for message in messages
        if isinstance(message, tuple) and len(message) > 1
    )


def test_explicit_nvidia_decoder_has_no_software_candidate() -> None:
    from Workstation.workstation_core import yolo_stream

    assert yolo_stream.decoder_candidates("nvidia") == (yolo_stream.NVIDIA_H264_DECODER,)


def test_default_gstreamer_pipeline_preserves_compressed_access_units() -> None:
    from Workstation.workstation_core import yolo_stream

    pipeline = yolo_stream.build_gst_pipeline_text(
        "rtsp://camera.invalid/visible",
        decoder=yolo_stream.SOFTWARE_H264_DECODER,
        transport="tcp",
    )

    assert "h264parse config-interval=-1 disable-passthrough=true name=parse" in pipeline
    assert "video/x-h264,stream-format=byte-stream,alignment=au" in pipeline
    assert "queue max-size-buffers=0 max-size-bytes=0 max-size-time=250000000" in pipeline
    assert pipeline.count("queue leaky=downstream max-size-buffers=1") == 0
    assert pipeline.count("queue leaky=downstream max-size-buffers=8") == 1
    assert "appsink name=sink" in pipeline
    assert "pipeline_profile" not in inspect.signature(
        yolo_stream.build_gst_pipeline_text
    ).parameters


def test_runtime_nvidia_decoder_fallback_uses_first_frame_and_stall_deadlines() -> None:
    from Workstation.workstation_core import yolo_stream

    common = {
        "requested_decoder": "auto",
        "active_decoder": yolo_stream.NVIDIA_H264_DECODER,
        "started_s": 10.0,
        "already_fell_back": False,
    }
    assert yolo_stream.runtime_decoder_fallback_reason(
        **common, last_frame_s=None, now_s=39.9
    ) is None
    assert "first-frame" in yolo_stream.runtime_decoder_fallback_reason(
        **common, last_frame_s=None, now_s=40.0
    )
    assert yolo_stream.runtime_decoder_fallback_reason(
        **common, last_frame_s=12.0, now_s=41.9
    ) is None
    assert "stall" in yolo_stream.runtime_decoder_fallback_reason(
        **common, last_frame_s=12.0, now_s=42.0
    )
    assert yolo_stream.runtime_decoder_fallback_reason(
        **{**common, "already_fell_back": True},
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
        now_s=40.0,
        already_fell_back=False,
    )
    assert yolo_stream.runtime_decoder_fallback_reason(
        **common,
        last_frame_s=None,
        now_s=99.0,
        rtp_recovery_pending=True,
    ) is None


def test_workstation_live_pipeline_adds_only_the_au_integrity_parser_guard(monkeypatch) -> None:
    from Workstation.workstation_core import yolo_stream as workstation

    stereo_root = Path(__file__).resolve().parents[2] / "StereoWIRIS"
    monkeypatch.syspath_prepend(str(stereo_root))
    from wiris_stereo_distance import yolo_stream as stereo

    for decoder in ("nvidia", "software"):
        for transport in ("tcp", "udp"):
            kwargs = {
                "latency_ms": 100,
                "decoder": decoder,
                "transport": transport,
            }
            workstation_pipeline = workstation.build_gst_pipeline_text(
                "rtsp://camera.invalid/visible",
                **kwargs,
            )
            stereo_pipeline = stereo.build_gst_pipeline_text(
                "rtsp://camera.invalid/visible",
                **kwargs,
            )
            assert workstation_pipeline.replace(
                " disable-passthrough=true", ""
            ).replace(
                " ! video/x-h264,stream-format=byte-stream,alignment=au", ""
            ) == stereo_pipeline


def test_gstreamer_recording_branch_is_single_session_mp4_and_non_leaky() -> None:
    from Workstation.workstation_core import yolo_stream

    pipeline = yolo_stream.build_gst_pipeline_text(
        "rtsp://camera.invalid/visible",
        decoder=yolo_stream.SOFTWARE_H264_DECODER,
        transport="tcp",
        record_video_path="/tmp/camera recording.mp4",
    )

    assert (
        "encoded. ! queue max-size-buffers=0 max-size-bytes=0 "
        "max-size-time=5000000000"
    ) in pipeline
    assert pipeline.count("rtspsrc ") == 1
    assert "video/x-h264,stream-format=avc,alignment=au ! mp4mux name=record_mux" in pipeline
    assert 'filesink location="/tmp/camera recording.mp4"' in pipeline
    assert "matroskamux" not in pipeline
    assert not hasattr(yolo_stream, "_start_ffmpeg_encoded_recording_process")
    assert not hasattr(yolo_stream, "_start_gst_encoded_recording_process")
    assert pipeline.count("queue leaky=downstream max-size-buffers=1") == 0
    assert pipeline.count("queue leaky=downstream max-size-buffers=8") == 1


def test_h264_annex_b_idr_detection_uses_nal_headers_not_non_delta_flags() -> None:
    from Workstation.workstation_core import yolo_stream

    idr_access_unit = (
        b"\x00\x00\x00\x01\x67\x64\x00\x1f"
        b"\x00\x00\x01\x68\xee\x3c\x80"
        b"\x00\x00\x00\x01\x65\x88\x84"
    )
    non_idr_intra_access_unit = b"\x00\x00\x01\x61\x65\x99"

    assert yolo_stream.h264_byte_stream_nal_unit_types(idr_access_unit) == (7, 8, 5)
    assert yolo_stream.h264_access_unit_has_idr(idr_access_unit) is True
    # A payload byte whose low five bits are 5 is not a NAL header and cannot
    # authorize recovery. Neither can an AVCC/length-prefixed buffer because
    # the live parser boundary is explicitly normalized to Annex B.
    assert yolo_stream.h264_access_unit_has_idr(non_idr_intra_access_unit) is False
    assert yolo_stream.h264_access_unit_has_idr(b"\x00\x00\x00\x02\x65\x88") is False
    assert yolo_stream.h264_access_unit_has_idr(b"\x00\x00\x01\xe5\x88") is False
    assert yolo_stream.h264_access_unit_has_idr(b"\x00\x00\x01\x05\x88") is False


def test_rtp_gap_latch_ignores_wrap_duplicate_and_reorder_then_recovers_on_exact_idr() -> None:
    from Workstation.workstation_core import yolo_stream

    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    assert tracker.decoded_output_blocked() is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=50,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=True,
        is_random_access=True,
        observed_monotonic_s=9.0,
    ) is False
    assert tracker.should_suppress_decoded_buffer(51) is True
    assert tracker.should_suppress_decoded_buffer(50) is False
    assert tracker.pop_rtp_recovery() is not None
    assert tracker.register_rtp_sequence(ssrc=7, sequence=65534) is None
    assert tracker.register_rtp_sequence(ssrc=7, sequence=65535) is None
    assert tracker.register_rtp_sequence(ssrc=7, sequence=0) is None
    assert tracker.register_rtp_sequence(ssrc=7, sequence=0) is None
    assert tracker.register_rtp_sequence(ssrc=7, sequence=65535) is None

    gap = tracker.register_rtp_sequence(
        ssrc=7,
        sequence=2,
        observed_monotonic_s=10.0,
        rtp_pts_ns=100,
    )
    assert gap is not None
    assert gap.missing_packets == 1
    assert tracker.pop_rtp_discontinuity() == gap
    assert tracker.decoded_output_blocked() is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=100,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        observed_monotonic_s=10.1,
    ) is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=200,
        is_delta=True,
        is_corrupted=False,
        is_discontinuous=False,
        observed_monotonic_s=10.2,
    ) is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=300,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        is_random_access=False,
        observed_monotonic_s=12.0,
    ) is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=400,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=True,
        is_random_access=True,
        observed_monotonic_s=12.0,
    ) is False
    assert tracker.should_suppress_decoded_buffer(401) is True
    assert tracker.should_suppress_decoded_buffer(399) is True
    assert tracker.should_suppress_decoded_buffer(400) is False
    recovery = tracker.pop_rtp_recovery()
    assert recovery is not None
    assert recovery.resume_pts_ns == 400
    assert recovery.blocked_duration_s == pytest.approx(2.0)


def test_corrupt_access_unit_and_dependent_chain_never_reach_consumers() -> None:
    from Workstation.workstation_core import yolo_stream

    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=100,
        is_delta=False,
        is_corrupted=True,
        is_discontinuous=False,
        is_random_access=False,
        observed_monotonic_s=1.0,
    ) is True
    event = tracker.pop_h264_access_unit_discontinuity()
    assert event is not None and event.reason == "corrupted"
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=200,
        is_delta=True,
        is_corrupted=False,
        is_discontinuous=False,
        is_random_access=False,
    ) is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=300,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        is_random_access=False,
    ) is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=400,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        is_random_access=True,
    ) is False
    assert tracker.should_suppress_decoded_buffer(401) is True
    assert tracker.should_suppress_decoded_buffer(400) is False


def test_startup_discont_requires_idr_and_later_non_idr_discont_is_blocked() -> None:
    from Workstation.workstation_core import yolo_stream

    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=100,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=True,
        is_random_access=False,
    ) is True
    assert tracker.decoded_output_blocked() is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=150,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=True,
        is_random_access=True,
    ) is False
    assert tracker.should_suppress_decoded_buffer(150) is False
    assert tracker.decoded_output_blocked() is False
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=200,
        is_delta=True,
        is_corrupted=False,
        is_discontinuous=True,
        is_random_access=False,
    ) is True
    event = tracker.pop_h264_access_unit_discontinuity()
    assert event is not None and event.reason == "discont"


def test_parser_probe_drops_zero_idr_access_units_before_decoder_and_recording_tee() -> None:
    from Workstation.workstation_core import yolo_stream

    class Gst:
        CLOCK_TIME_NONE = -1
        MapFlags = SimpleNamespace(READ=1)
        BufferFlags = SimpleNamespace(DELTA_UNIT=1, CORRUPTED=2, DISCONT=4)
        PadProbeReturn = SimpleNamespace(OK="OK", DROP="DROP")

    class Buffer:
        def __init__(self, data: bytes, *, pts: int, flags: int = 0) -> None:
            self.data = data
            self.pts = pts
            self.flags = flags
            self.unmapped = False
            self.map_calls = 0

        def get_flags(self) -> int:
            return self.flags

        def map(self, _mode: object) -> tuple[bool, SimpleNamespace]:
            self.map_calls += 1
            return True, SimpleNamespace(data=self.data)

        def unmap(self, _map_info: object) -> None:
            self.unmapped = True

        def set_flags(self, flags: int) -> None:
            self.flags |= flags

    tracker = yolo_stream.GstRtpTimingTracker(Gst, SimpleNamespace())
    non_idr = Buffer(b"\x00\x00\x00\x01\x61\x65\x99", pts=100, flags=1)
    assert tracker._observe_access_unit(
        None, SimpleNamespace(get_buffer=lambda: non_idr)
    ) == "DROP"
    assert non_idr.unmapped is True
    # A decoder concealment frame racing with the parser probe is also barred
    # from the native ring, preview queue, and latest-frame YOLO handoff.
    assert tracker.should_suppress_decoded_buffer(100) is True

    idr = Buffer(b"\x00\x00\x01\x67\x64\x00\x1f\x00\x00\x01\x65\x88", pts=200)
    assert tracker._observe_access_unit(
        None, SimpleNamespace(get_buffer=lambda: idr)
    ) == "OK"
    assert idr.flags & Gst.BufferFlags.DISCONT
    assert tracker.should_suppress_decoded_buffer(201) is True
    assert tracker.should_suppress_decoded_buffer(200) is False
    dependent = Buffer(b"\x00\x00\x01\x61\x99", pts=300, flags=1)
    assert tracker._observe_access_unit(
        None, SimpleNamespace(get_buffer=lambda: dependent)
    ) == "OK"
    assert dependent.map_calls == 0


def test_zero_idr_sender_stays_blocked_without_extending_the_timeout(monkeypatch) -> None:
    from Workstation.workstation_core import yolo_stream

    clock = [100.0]
    monkeypatch.setattr(yolo_stream.time, "monotonic", lambda: clock[0])
    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())

    for index in range(30):
        clock[0] += 1.0
        pts_ns = (index + 1) * 1_000
        assert tracker.should_drop_encoded_access_unit(
            pts_ns=pts_ns,
            is_delta=bool(index % 2),
            is_corrupted=False,
            is_discontinuous=False,
            is_random_access=False,
        ) is True
        assert tracker.should_suppress_decoded_buffer(pts_ns) is True

    assert tracker.pop_rtp_recovery() is None
    assert tracker.blocked_duration_s(now_s=clock[0]) == pytest.approx(30.0)


def test_tcp_rtp_omission_isolated_to_one_side_and_blocks_all_bad_handoffs() -> None:
    from Workstation.workstation_core import yolo_stream

    left = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    right = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    for tracker, pts_ns in ((left, 10), (right, 20)):
        assert tracker.should_drop_encoded_access_unit(
            pts_ns=pts_ns,
            is_delta=False,
            is_corrupted=False,
            is_discontinuous=False,
            is_random_access=True,
        ) is False
        assert tracker.should_suppress_decoded_buffer(pts_ns) is False
        assert tracker.pop_rtp_recovery() is not None

    assert left.register_rtp_sequence(ssrc=11, sequence=100, rtp_pts_ns=10) is None
    assert right.register_rtp_sequence(ssrc=22, sequence=500, rtp_pts_ns=20) is None
    gap = right.register_rtp_sequence(
        ssrc=22,
        sequence=525,
        rtp_pts_ns=30,
        observed_monotonic_s=200.0,
    )
    assert gap is not None and gap.missing_packets == 24
    assert left.decoded_output_blocked() is False
    assert right.decoded_output_blocked() is True

    # Left remains independently usable. Right's dependent AU is dropped
    # before the encoded tee (recording/decoder), and any raced decoded output
    # remains barred from native evidence, preview, and YOLO.
    assert left.should_drop_encoded_access_unit(
        pts_ns=40,
        is_delta=True,
        is_corrupted=False,
        is_discontinuous=False,
        is_random_access=False,
    ) is False
    assert left.should_suppress_decoded_buffer(40) is False
    assert right.should_drop_encoded_access_unit(
        pts_ns=40,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        is_random_access=False,
    ) is True
    assert right.should_suppress_decoded_buffer(40) is True


def test_ssrc_restart_uses_new_pts_domain_and_still_requires_idr() -> None:
    from Workstation.workstation_core import yolo_stream

    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=9_000_000,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        is_random_access=True,
    ) is False
    assert tracker.should_suppress_decoded_buffer(9_000_000) is False
    assert tracker.pop_rtp_recovery() is not None
    assert tracker.register_rtp_sequence(
        ssrc=1,
        sequence=100,
        rtp_pts_ns=9_000_000,
    ) is None

    assert tracker.register_rtp_sequence(
        ssrc=2,
        sequence=1,
        rtp_pts_ns=100,
    ) is None
    event = tracker.pop_h264_access_unit_discontinuity()
    assert event is not None and event.reason == "ssrc-change"
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=150,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
        is_random_access=False,
    ) is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=200,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=True,
        is_random_access=True,
    ) is False
    assert tracker.should_suppress_decoded_buffer(200) is False


def test_gstreamer_launch_does_not_retry_software_for_explicit_nvidia(monkeypatch) -> None:
    from Workstation.workstation_core import yolo_stream

    messages: list[object] = []

    class OutputQueue:
        def put(self, message: object) -> None:
            messages.append(message)

    class Pipeline:
        def get_by_name(self, _name: str) -> object:
            return object()

        def get_bus(self) -> object:
            return object()

        def set_state(self, _state: object) -> str:
            return "SUCCESS"

    class FakeGst:
        class State:
            PLAYING = "PLAYING"
            NULL = "NULL"

        class StateChangeReturn:
            FAILURE = "FAILURE"

        @staticmethod
        def parse_launch(pipeline_text: str) -> Pipeline:
            if yolo_stream.NVIDIA_H264_DECODER in pipeline_text:
                raise RuntimeError('no element "nvh264dec"')
            return Pipeline()

    with pytest.raises(RuntimeError, match="nvh264dec"):
        yolo_stream._launch_gst_pipeline(
            FakeGst,
            "rtsp://camera.invalid/visible",
            latency_ms=100,
            decoder_mode="nvidia",
            transport="tcp",
            keyframe_only=False,
            record_video_path="",
            output_queue=OutputQueue(),
        )

    assert not any(
        "software로 재시도" in str(message[1])
        for message in messages
        if isinstance(message, tuple) and len(message) > 1
    )


def test_gstreamer_reconnect_does_not_exhaust_a_lifetime_retry_budget(monkeypatch) -> None:
    from Workstation.workstation_core import yolo_stream

    attempts: list[int] = []
    messages: list[object] = []
    stop_event = threading.Event()

    class OutputQueue:
        def put(self, message: object) -> None:
            messages.append(message)

    def fail_until_stopped(*_args: object) -> bool:
        attempts.append(len(attempts) + 1)
        if len(attempts) == 5:
            stop_event.set()
        return False

    monkeypatch.setattr(yolo_stream, "_run_gstreamer_stream_once", fail_until_stopped)
    monkeypatch.setattr(yolo_stream, "GST_RTSP_RECONNECT_DELAYS_S", (0.0, 0.0))

    yolo_stream._run_gstreamer_stream(
        {"rtsp_reconnect_policy": yolo_stream.RTSP_RECONNECT_AUTOMATIC},
        OutputQueue(),
        stop_event,
        None,
        None,
        False,
    )

    assert attempts == [1, 2, 3, 4, 5]
    assert not any(
        "재연결 포기" in str(message[1])
        for message in messages
        if isinstance(message, tuple) and len(message) > 1
    )


def test_manual_reconnect_policy_runs_one_gstreamer_session(monkeypatch) -> None:
    from Workstation.workstation_core import yolo_stream

    attempts: list[int] = []
    messages: list[object] = []

    class OutputQueue:
        def put(self, message: object) -> None:
            messages.append(message)

    def fail_once(*_args: object) -> bool:
        attempts.append(1)
        return False

    monkeypatch.setattr(yolo_stream, "_run_gstreamer_stream_once", fail_once)

    yolo_stream._run_gstreamer_stream(
        {"rtsp_reconnect_policy": yolo_stream.RTSP_RECONNECT_MANUAL},
        OutputQueue(),
        threading.Event(),
        None,
        None,
        False,
    )

    assert attempts == [1]
    assert not any(
        "RTSP 재연결 대기" in str(message[1])
        for message in messages
        if isinstance(message, tuple) and len(message) > 1
    )


def test_new_stream_worker_uses_only_stereowiris_pipeline_and_manual_policy() -> None:
    from Workstation.workstation_core import yolo_stream

    worker = yolo_stream.YoloStreamWorker(
        url="rtsp://camera.invalid/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
    )

    assert "rtsp_pipeline_profile" not in worker.config
    assert (
        worker.config["rtsp_reconnect_policy"]
        == yolo_stream.RTSP_RECONNECT_MANUAL
    )
    assert worker.config["recording_encoding_mode"] == "h264_copy_integrity_gated"
    assert worker.config["recording_integrity_verified"] is False
    assert worker.config["recording_integrity_scope"] == (
        "finalized-mp4:h264-idr+frame-num+decode-count+sample-timing"
    )


@pytest.mark.parametrize("backend", ["opencv", "opencv_auto", "pyav"])
def test_non_gstreamer_recording_contract_is_explicitly_unverified(backend: str) -> None:
    from Workstation.workstation_core import yolo_stream

    contract = yolo_stream.recording_integrity_contract(backend)
    assert contract == {
        "recording_encoding_mode": "decoded_reencode_unverified",
        "recording_integrity_verified": False,
        "recording_integrity_scope": "decoded-frames-only",
    }


def test_zero_idr_integrity_timeout_is_single_session_without_reconnect_storm() -> None:
    from Workstation.workstation_core import yolo_stream

    attempts: list[int] = []
    messages: list[object] = []
    config = {"rtsp_reconnect_policy": yolo_stream.RTSP_RECONNECT_AUTOMATIC}

    class OutputQueue:
        def put(self, message: object) -> None:
            messages.append(message)

    original = yolo_stream._run_gstreamer_stream_once
    try:
        def fail_integrity(*_args: object) -> bool:
            attempts.append(1)
            config["_integrity_manual_reconnect_required"] = True
            return False

        yolo_stream._run_gstreamer_stream_once = fail_integrity
        yolo_stream._run_gstreamer_stream(
            config,
            OutputQueue(),
            threading.Event(),
            None,
            None,
            False,
        )
    finally:
        yolo_stream._run_gstreamer_stream_once = original

    assert attempts == [1]
    assert any(
        "자동 재연결 안 함" in str(message[1])
        for message in messages
        if isinstance(message, tuple) and len(message) > 1
    )


def test_live_integrity_terminal_retires_only_failed_side_and_updates_truth() -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    stops: list[tuple[str, bool]] = []
    labels: list[str] = []
    logs: list[str] = []
    refreshes: list[str] = []
    calibration_preview_text: list[str] = []

    def pane(side: str) -> SimpleNamespace:
        return SimpleNamespace(
            worker=object(),
            thread=object(),
            detection_stream_active=True,
            stream_generation=4,
            stop=lambda *, wait=False, side=side: stops.append((side, wait)) or False,
        )

    class MeasurementOwner:
        state = "RUNNING"

        def _set_state(self, state: str, detail: str, error: str = "") -> None:
            self.state = state
            self.detail = detail
            self.error = error

    measurement_owner = MeasurementOwner()
    panel = SimpleNamespace(
        _camera_start_readiness=None,
        _camera_integrity_unavailable_sides=set(),
        _camera_integrity_failed_generations={},
        left_pane=pane("left"),
        right_pane=pane("right"),
        active_stream_decoders={"left": "nvh264dec", "right": "nvh264dec"},
        camera_frame_times={"left": [1.0], "right": [2.0]},
        camera_frame_metadata={"left": {"frame": 1}, "right": {"frame": 2}},
        camera_frame_metadata_history={"left": [{"frame": 1}], "right": [{"frame": 2}]},
        detection_buffers={"left": {"good-left"}, "right": {"stale-right"}},
        latest_left="good-left",
        latest_right="stale-right",
        latest_calibration_preview_images={"left": "good-left", "right": "stale-right"},
        calibration_preview_overlay_images={"left": "good-left", "right": "stale-right"},
        calibration_preview_detection_results={"left": "good-left", "right": "stale-right"},
        calibration_preview_request_ids={"left": 10, "right": 20},
        calibration_right_preview=SimpleNamespace(
            setPixmap=lambda _pixmap: refreshes.append("calibration-pixmap"),
            setText=calibration_preview_text.append,
        ),
        camera_connected=True,
        yolo_enabled=True,
        _measurement_run_active_sides=("left", "right"),
        _measurement_run_controller_owner=measurement_owner,
        _clear_measurement_pair_display=lambda: refreshes.append("pair"),
        _refresh_distance=lambda: refreshes.append("distance"),
        camera_connection_label=SimpleNamespace(setText=labels.append),
        _log=logs.append,
        _update_runtime_camera_stats_labels=lambda: refreshes.append("stats"),
        _refresh_session_toggle_buttons=lambda: refreshes.append("buttons"),
        _refresh_operation_header=lambda: refreshes.append("header"),
    )

    controller = CameraStreamLifecycleController(panel)
    controller._camera_stream_start_status(
        "right",
        "INTEGRITY_RECOVERY_TIMEOUT: 30.0s 동안 verified H264 IDR 없음",
    )

    assert stops == [("right", False)]
    assert panel._camera_integrity_unavailable_sides == {"right"}
    assert panel._camera_integrity_failed_generations == {"right": 4}
    assert panel.camera_connected is True
    assert panel.yolo_enabled is True
    assert panel._measurement_run_active_sides == ("left",)
    assert measurement_owner.state == "RUNNING"
    assert "left only" in measurement_owner.detail
    assert panel.active_stream_decoders == {"left": "nvh264dec", "right": None}
    assert panel.camera_frame_times == {"left": [1.0], "right": []}
    assert panel.camera_frame_metadata == {"left": {"frame": 1}, "right": {}}
    assert panel.camera_frame_metadata_history == {"left": [{"frame": 1}], "right": []}
    assert panel.detection_buffers == {"left": {"good-left"}, "right": set()}
    assert panel.latest_left == "good-left"
    assert panel.latest_right is None
    assert panel.latest_calibration_preview_images == {"left": "good-left"}
    assert panel.calibration_preview_overlay_images == {"left": "good-left"}
    assert panel.calibration_preview_detection_results == {"left": "good-left"}
    assert panel.calibration_preview_request_ids == {"left": 10, "right": 0}
    assert calibration_preview_text == ["right unavailable (H264 IDR integrity)"]
    assert labels[-1] == "camera partial: left LIVE; right unavailable (H264 IDR integrity)"
    assert any("opposite side preserved=True" in message for message in logs)
    assert {
        "pair",
        "distance",
        "stats",
        "buttons",
        "header",
        "calibration-pixmap",
    }.issubset(refreshes)

    # A queued frame from the terminal generation cannot undo the unavailable
    # latch even if it arrives after the terminal control message.
    controller._camera_stream_frame_ready(
        "right",
        None,
        {"stream_generation": 4, "active_decoder": "nvh264dec"},
    )
    assert panel._camera_integrity_unavailable_sides == {"right"}
    assert labels[-1] == "camera partial: left LIVE; right unavailable (H264 IDR integrity)"

    # A manual reconnect becomes truthful only on a newer generation's frame,
    # which the child emits only after parsed-IDR + exact decoded-PTS recovery.
    panel.right_pane.stream_generation = 5
    controller._camera_stream_frame_ready(
        "right",
        None,
        {"stream_generation": 5, "active_decoder": "nvh264dec"},
    )
    assert panel._camera_integrity_unavailable_sides == set()
    assert panel._camera_integrity_failed_generations == {}
    assert labels[-1] == "camera LIVE: left/right current frame verified"


def test_manual_rtsp_reactivation_stays_pending_until_new_idr_frame() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    labels: list[str] = []
    logs: list[str] = []
    starts: list[dict[str, object]] = []

    class Pane:
        def __init__(self, *, running: bool) -> None:
            self.worker = object() if running else None
            self.thread = object() if running else None
            self.detection_stream_active = False

        def start_stream(self, url: str, **kwargs: object) -> bool:
            starts.append({"url": url, **kwargs})
            self.worker = object()
            self.thread = object()
            return True

    panel = SimpleNamespace(
        _rtsp_reactivation_side="right",
        _rtsp_reactivation_enable_detection=False,
        _rtsp_reactivation_stop_timer=SimpleNamespace(stop=lambda: None),
        _rtsp_reactivation_stop_generation=1,
        _rtsp_reactivation_generation=3,
        _camera_integrity_unavailable_sides={"right"},
        _measurement_camera_rejoin_expected_generations={},
        left_pane=Pane(running=True),
        right_pane=Pane(running=False),
        left_url_edit=SimpleNamespace(text=lambda: "rtsp://left/visible"),
        right_url_edit=SimpleNamespace(text=lambda: "rtsp://right/visible"),
        camera_frame_times={"left": [1.0], "right": [2.0]},
        camera_frame_metadata={"left": {"frame": 1}, "right": {"frame": 2}},
        camera_frame_metadata_history={"left": [{"frame": 1}], "right": [{"frame": 2}]},
        camera_stream_stats={"left": {"hz": 15}, "right": {"hz": 10}},
        active_stream_decoders={"left": "nvh264dec", "right": "nvh264dec"},
        _camera_sync_transport_override="tcp",
        _camera_name_ko=lambda side: side,
        _set_camera_sync_state=lambda *_args: None,
        yolo_kwargs=lambda _camera: {"decoder": "software"},
        _video_recording_kwargs=lambda _camera: {},
        camera_connection_label=SimpleNamespace(setText=labels.append),
        _log=logs.append,
        _refresh_session_toggle_buttons=lambda: None,
        _refresh_operation_header=lambda: None,
        camera_connected=True,
        yolo_enabled=False,
    )

    WirisStereoDistancePanel._finish_camera_rtsp_reactivation(
        panel,
        True,
        "frame probe accepted",
    )

    assert starts == [
        {
            "url": "rtsp://right/visible",
            "enable_detection": False,
            "decoder": "software",
        }
    ]
    assert panel.camera_connected is True  # healthy left only
    assert panel._camera_integrity_unavailable_sides == {"right"}
    assert labels[-1] == "camera RTSP reactivate: right=current-frame verification pending"
    assert any("current frame" in message for message in logs)


def test_measurement_camera_rejoin_retries_recording_and_uses_new_segment(
    tmp_path: Path,
) -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    starts: list[dict[str, object]] = []
    transitions: list[tuple[str, str]] = []
    new_path = tmp_path / "right-rejoin.mp4"

    class Pane:
        worker = None
        thread = None
        detection_stream_active = False
        stream_generation = 7

        def start_stream(self, url: str, **kwargs: object) -> bool:
            starts.append({"url": url, **kwargs})
            self.worker = object()
            self.thread = object()
            return True

    panel = SimpleNamespace(
        _rtsp_reactivation_side="right",
        _rtsp_reactivation_enable_detection=True,
        _rtsp_reactivation_stop_timer=SimpleNamespace(stop=lambda: None),
        _rtsp_reactivation_stop_generation=1,
        _rtsp_reactivation_generation=3,
        _measurement_run_state="RUNNING",
        _measurement_camera_rejoin_expected_generations={},
        _camera_integrity_unavailable_sides=set(),
        left_pane=Pane(),
        right_pane=Pane(),
        left_url_edit=SimpleNamespace(text=lambda: "rtsp://left/visible"),
        right_url_edit=SimpleNamespace(text=lambda: "rtsp://right/visible"),
        camera_frame_times={"left": [], "right": []},
        camera_frame_metadata={"left": {}, "right": {}},
        camera_frame_metadata_history={"left": [], "right": []},
        camera_stream_stats={"left": {}, "right": {}},
        active_stream_decoders={"left": None, "right": None},
        _camera_sync_transport_override=None,
        _camera_name_ko=lambda side: side,
        _set_camera_sync_state=lambda *_args: None,
        tabs=SimpleNamespace(setCurrentIndex=lambda _index: None),
        depth_tab_index=1,
        yolo_kwargs=lambda _camera: {"decoder": "software"},
        video_recording_paths={},
        _video_recording_allocation_sizes={},
        video_recording_enabled=False,
        recording_active=False,
        _session_recording_active=lambda: panel.recording_active,
        start_automatic_session_recording=lambda: (
            setattr(panel, "recording_active", True)
            or panel.video_recording_paths.update(right=new_path)
            or True
        ),
        allocate_measurement_video_segment=lambda _side: (_ for _ in ()).throw(
            AssertionError("initial rejoin segment must be reused")
        ),
        _video_recording_kwargs=lambda camera: (
            {"record_video_path": str(panel.video_recording_paths[camera])}
            if camera in panel.video_recording_paths
            else {}
        ),
        measurement_source_transition=lambda source, state, _detail: transitions.append(
            (source, state)
        ),
        camera_connection_label=SimpleNamespace(setText=lambda _text: None),
        _log=lambda _message: None,
        _refresh_session_toggle_buttons=lambda: None,
        _refresh_operation_header=lambda: None,
        camera_connected=False,
        yolo_enabled=False,
    )

    WirisStereoDistancePanel._finish_camera_rtsp_reactivation(panel, True)

    assert starts == [
        {
            "url": "rtsp://right/visible",
            "enable_detection": True,
            "decoder": "software",
            "record_video_path": str(new_path),
        }
    ]
    assert panel.video_recording_enabled is True
    assert panel._measurement_camera_rejoin_expected_generations == {"right": 7}


def test_worker_shutdown_preserves_mp4_eos_window_before_escalation(monkeypatch) -> None:
    from Workstation.workstation_core import yolo_stream

    clock = [100.0]
    monkeypatch.setattr(yolo_stream.time, "monotonic", lambda: clock[0])

    class Process:
        def __init__(self) -> None:
            self.alive = True
            self.join_timeouts: list[float] = []
            self.terminated = False
            self.killed = False

        def is_alive(self) -> bool:
            return self.alive

        def join(self, timeout: float) -> None:
            self.join_timeouts.append(timeout)
            clock[0] += timeout

        def terminate(self) -> None:
            self.terminated = True

        def kill(self) -> None:
            self.killed = True
            self.alive = False

    worker = yolo_stream.YoloStreamWorker(
        url="rtsp://camera.invalid/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
    )
    process = Process()
    worker._process = process

    worker.stop()

    assert process.join_timeouts == pytest.approx([12.0, 8.0, 1.0])
    assert process.terminated is True
    assert process.killed is True


def test_worker_does_not_forget_child_that_survives_kill_join(monkeypatch) -> None:
    from Workstation.workstation_core import yolo_stream

    clock = [100.0]
    monkeypatch.setattr(yolo_stream.time, "monotonic", lambda: clock[0])

    class Process:
        terminated = False
        killed = False

        def is_alive(self) -> bool:
            return True

        def join(self, timeout: float) -> None:
            clock[0] += timeout

        def terminate(self) -> None:
            self.terminated = True

        def kill(self) -> None:
            self.killed = True

    worker = yolo_stream.YoloStreamWorker(
        url="rtsp://camera.invalid/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
    )
    process = Process()
    worker._process = process

    assert worker._stop_child_process() is False
    assert worker._process is process
    assert process.terminated is True
    assert process.killed is True


def test_gstreamer_eos_fails_closed_when_live_integrity_metrics_are_nonzero() -> None:
    from Workstation.workstation_core import yolo_stream

    messages: list[object] = []

    class OutputQueue:
        def put(self, message: object) -> None:
            messages.append(message)

    class Gst:
        class Event:
            @staticmethod
            def new_eos() -> str:
                return "EOS_EVENT"

        class MessageType:
            EOS = 2
            ERROR = 1

    class Pipeline:
        def send_event(self, event: object) -> bool:
            assert event == "EOS_EVENT"
            return True

    class Bus:
        def timed_pop_filtered(self, *_args: object) -> SimpleNamespace:
            return SimpleNamespace(type=Gst.MessageType.EOS)

    config = {
        "_integrity_gap_events": 2,
        "_integrity_missing_packets": 7,
        "_integrity_suppressed_decoded_frames": 11,
        "_integrity_manual_reconnect_required": True,
        "_recording_integrity_validator": lambda *_args, **_kwargs: {
            "integrity_schema": "workstation-h264-mp4-integrity-v1",
            "integrity_validation_status": "PASS",
            "integrity_failure_reasons": [],
            "recording_usable": True,
            "recording_integrity_verified": True,
        },
    }
    assert yolo_stream._finalize_gst_recording(
        Gst,
        Pipeline(),
        Bus(),
        OutputQueue(),
        "/tmp/finalized.mp4",
        config=config,
    ) is False

    event = next(
        message[1]
        for message in messages
        if isinstance(message, tuple) and message[0] == "recording_finalized"
    )
    assert event["success"] is False
    assert event["recording_encoding_mode"] == "h264_copy_integrity_gated"
    assert event["container_finalization_success"] is True
    assert event["integrity_terminal_failure"] == (
        "H264_IDR_RANDOM_ACCESS_UNAVAILABLE"
    )
    assert event["integrity_validation_status"] == "FAIL"
    assert event["recording_usable"] is False
    assert event["recording_integrity_verified"] is False
    assert event["integrity_gap_events"] == 2
    assert event["integrity_missing_packets"] == 7
    assert event["integrity_suppressed_decoded_frames"] == 11
    assert set(event["integrity_failure_reasons"]) >= {
        "live_h264_idr_random_access_unavailable",
        "live_rtp_sequence_gap_detected",
        "live_rtp_missing_packet_detected",
        "live_decoder_frame_suppression_detected",
    }


def test_video_finalization_artifact_rejects_eos_without_integrity_result(
    tmp_path: Path,
) -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    path = tmp_path / "left.mp4"
    path.write_bytes(b"valid-mp4-placeholder")
    artifacts: list[dict[str, object]] = []
    flushes: list[bool] = []
    finalization = {
        "path": str(path),
        "stream_backend": "gstreamer",
        "success": True,
        "detail": "EOS received; mp4mux finalized",
        "integrity_gap_events": 1,
    }
    panel = SimpleNamespace(
        _pending_video_finalization_artifacts={
            "paths": {"left": path},
            "backend": "gstreamer",
        },
        left_pane=SimpleNamespace(recording_finalizations={str(path): finalization}),
        append_workstation_data=lambda stream, payload, **_kwargs: artifacts.append(
            {"stream": stream, **dict(payload)}
        ),
        flush_workstation_data=lambda: flushes.append(True),
        _log=lambda _message: None,
    )

    CameraStreamLifecycleController(panel)._append_pending_video_finalization_artifacts()

    assert panel._pending_video_finalization_artifacts is None
    assert flushes == [True]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert artifact["stream"] == "artifacts"
    assert artifact["event_type"] == "video_finalized"
    assert artifact["path"] == str(path)
    assert artifact["recording_encoding_mode"] == "h264_copy_integrity_gated"
    assert artifact["recording_integrity_verified"] is False
    assert artifact["recording_usable"] is False
    assert artifact["integrity_validation_status"] == "FAIL"
    assert artifact["success"] is False
    assert artifact["container_finalization_success"] is True
    assert artifact["finalization_success"] is False
    assert artifact["integrity_failure_reasons"] == [
        "integrity_validation_schema_invalid"
    ]
    assert "MP4 integrity FAIL" in str(artifact["finalization_detail"])
    assert artifact["present"] is True
    assert artifact["size_bytes"] == len(b"valid-mp4-placeholder")


def test_video_finalization_evidence_retry_keeps_data_store_open_contract(
    tmp_path: Path,
) -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    path = tmp_path / "left.mp4"
    path.write_bytes(b"partial-or-final-mp4")
    finalization = {
        "path": str(path),
        "stream_backend": "gstreamer",
        "success": False,
        "detail": "EOS timeout",
    }
    append_results = iter((False, True))
    attempts: list[dict[str, object]] = []
    flushes: list[bool] = []
    panel = SimpleNamespace(
        _pending_video_finalization_artifacts={
            "paths": {"left": path},
            "backend": "gstreamer",
        },
        left_pane=SimpleNamespace(recording_finalizations={str(path): finalization}),
        append_workstation_data=lambda _stream, payload, **_kwargs: (
            attempts.append(dict(payload)),
            next(append_results),
        )[1],
        flush_workstation_data=lambda: flushes.append(True) or True,
        _log=lambda _message: None,
    )
    controller = CameraStreamLifecycleController(panel)

    assert controller._append_pending_video_finalization_artifacts() is False
    assert panel._pending_video_finalization_artifacts is not None
    assert str(path) in panel.left_pane.recording_finalizations
    assert flushes == []

    assert controller._append_pending_video_finalization_artifacts() is True
    assert panel._pending_video_finalization_artifacts is None
    assert str(path) not in panel.left_pane.recording_finalizations
    assert flushes == [True]
    assert [row["finalization_detail"] for row in attempts] == [
        "EOS timeout",
        "EOS timeout",
    ]


def test_new_workstation_settings_prefer_reliable_tcp_rtsp_transport() -> None:
    from Workstation.workstation_core.settings import AppSettings

    assert AppSettings().rtsp_transport == "tcp"
    assert not hasattr(AppSettings(), "rtsp_pipeline_profile")
    assert AppSettings().rtsp_reconnect_policy == "manual"
