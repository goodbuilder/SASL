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
    assert "video/x-h264,alignment=au" in pipeline
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
                " ! video/x-h264,alignment=au", ""
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


def test_rtp_gap_latch_ignores_wrap_duplicate_and_reorder_then_recovers_on_clean_au() -> None:
    from Workstation.workstation_core import yolo_stream

    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
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
        observed_monotonic_s=12.0,
    ) is False
    assert tracker.should_suppress_decoded_buffer(299) is True
    assert tracker.should_suppress_decoded_buffer(300) is False
    recovery = tracker.pop_rtp_recovery()
    assert recovery is not None
    assert recovery.resume_pts_ns == 300
    assert recovery.blocked_duration_s == pytest.approx(2.0)


def test_corrupt_access_unit_and_dependent_chain_never_reach_consumers() -> None:
    from Workstation.workstation_core import yolo_stream

    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=100,
        is_delta=False,
        is_corrupted=True,
        is_discontinuous=False,
        observed_monotonic_s=1.0,
    ) is True
    event = tracker.pop_h264_access_unit_discontinuity()
    assert event is not None and event.reason == "corrupted"
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=200,
        is_delta=True,
        is_corrupted=False,
        is_discontinuous=False,
    ) is True
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=300,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=False,
    ) is False
    assert tracker.should_suppress_decoded_buffer(300) is False


def test_normal_start_discont_is_allowed_but_later_discont_is_blocked() -> None:
    from Workstation.workstation_core import yolo_stream

    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=100,
        is_delta=False,
        is_corrupted=False,
        is_discontinuous=True,
    ) is False
    assert tracker.decoded_output_blocked() is False
    assert tracker.should_drop_encoded_access_unit(
        pts_ns=200,
        is_delta=True,
        is_corrupted=False,
        is_discontinuous=True,
    ) is True
    event = tracker.pop_h264_access_unit_discontinuity()
    assert event is not None and event.reason == "discont"


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
    assert worker.config["recording_integrity_verified"] is True


@pytest.mark.parametrize("backend", ["opencv", "opencv_auto", "pyav"])
def test_non_gstreamer_recording_contract_is_explicitly_unverified(backend: str) -> None:
    from Workstation.workstation_core import yolo_stream

    contract = yolo_stream.recording_integrity_contract(backend)
    assert contract == {
        "recording_encoding_mode": "decoded_reencode_unverified",
        "recording_integrity_verified": False,
        "recording_integrity_scope": "decoded-frames-only",
    }


def test_integrity_timeout_disables_automatic_reconnect() -> None:
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


def test_gstreamer_eos_emits_structured_mp4_finalization_evidence() -> None:
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
    }
    assert yolo_stream._finalize_gst_recording(
        Gst,
        Pipeline(),
        Bus(),
        OutputQueue(),
        "/tmp/finalized.mp4",
        config=config,
    ) is True

    event = next(
        message[1]
        for message in messages
        if isinstance(message, tuple) and message[0] == "recording_finalized"
    )
    assert event["success"] is True
    assert event["recording_encoding_mode"] == "h264_copy_integrity_gated"
    assert event["integrity_gap_events"] == 2
    assert event["integrity_missing_packets"] == 7
    assert event["integrity_suppressed_decoded_frames"] == 11


def test_video_finalization_artifact_uses_child_eos_result(tmp_path: Path) -> None:
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
    assert artifacts == [
        {
            "stream": "artifacts",
            "event_type": "video_finalized",
            "side": "left",
            "path": str(path),
            "media_type": "video/mp4",
            "stream_backend": "gstreamer",
            "recording_encoding_mode": "h264_copy_integrity_gated",
            "recording_integrity_verified": True,
            "recording_integrity_scope": "rtp-sequence+h264-access-unit",
            "success": True,
            "detail": "EOS received; mp4mux finalized",
            "integrity_gap_events": 1,
            "finalization_success": True,
            "finalization_detail": "EOS received; mp4mux finalized",
            "present": True,
            "size_bytes": len(b"valid-mp4-placeholder"),
        }
    ]


def test_new_workstation_settings_prefer_reliable_tcp_rtsp_transport() -> None:
    from Workstation.workstation_core.settings import AppSettings

    assert AppSettings().rtsp_transport == "tcp"
    assert not hasattr(AppSettings(), "rtsp_pipeline_profile")
    assert AppSettings().rtsp_reconnect_policy == "manual"
