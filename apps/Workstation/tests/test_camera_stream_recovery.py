from __future__ import annotations

import sys
import threading
import time
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
        lambda *_args, **_kwargs: SimpleNamespace(),
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


def test_explicit_nvidia_decoder_falls_back_when_plugin_is_missing() -> None:
    from Workstation.workstation_core import yolo_stream

    assert yolo_stream.decoder_candidates("nvidia") == (
        yolo_stream.NVIDIA_H264_DECODER,
        yolo_stream.SOFTWARE_H264_DECODER,
    )


def test_gstreamer_launch_retries_software_for_explicit_nvidia(monkeypatch) -> None:
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

    launched = yolo_stream._launch_gst_pipeline(
        FakeGst,
        "rtsp://camera.invalid/visible",
        latency_ms=100,
        decoder_mode="nvidia",
        transport="tcp",
        keyframe_only=False,
        record_video_path="",
        output_queue=OutputQueue(),
    )

    assert launched[3] == yolo_stream.SOFTWARE_H264_DECODER
    assert any(
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
        {},
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


def test_new_workstation_settings_prefer_reliable_tcp_rtsp_transport() -> None:
    from Workstation.workstation_core.settings import AppSettings

    assert AppSettings().rtsp_transport == "tcp"
