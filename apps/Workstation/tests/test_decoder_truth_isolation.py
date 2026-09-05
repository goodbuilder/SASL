from __future__ import annotations

from types import SimpleNamespace
from types import ModuleType
import multiprocessing
import queue
import sys
import threading

import pytest
from PyQt5 import QtCore


def _publish_terminal_decoder_exit(_config, output_queue, _stop_event) -> None:
    output_queue.put(("status", "decoder terminal sentinel"))
    output_queue.put(
        (
            "terminal_exit",
            {"reason": "recording-side decoder fallback requires a new side start"},
        )
    )


def _status_texts(messages: list[object]) -> list[str]:
    return [
        str(message[1])
        for message in messages
        if isinstance(message, tuple) and len(message) > 1 and message[0] == "status"
    ]


def test_auto_launch_fallback_is_explicit_but_playing_is_only_a_candidate() -> None:
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

    _pipeline, _sink, _bus, decoder = yolo_stream._launch_gst_pipeline(
        FakeGst,
        "rtsp://camera.invalid/visible",
        latency_ms=100,
        decoder_mode="auto",
        transport="tcp",
        keyframe_only=False,
        record_video_path="",
        output_queue=OutputQueue(),
    )

    statuses = _status_texts(messages)
    fallback_messages = [
        dict(message[1])
        for message in messages
        if isinstance(message, tuple)
        and len(message) > 1
        and message[0] == "decoder_fallback"
    ]
    assert decoder == yolo_stream.SOFTWARE_H264_DECODER
    assert len(fallback_messages) == 1
    assert fallback_messages[0]["recording_restart_required"] is False
    assert "nvh264dec" in fallback_messages[0]["reason"]
    assert any("decoder candidate" in text for text in statuses)
    assert not any(text.startswith("GStreamer 디코더: ") for text in statuses)


@pytest.mark.parametrize(
    ("source_name", "detail", "expected"),
    (
        ("src", "Could not read from resource", "rtsp-source"),
        ("rtph264depay0", "stream stopped", "rtsp-source"),
        ("h264parse0", "missing codec data", "rtsp-source"),
        ("record_mux", "Could not multiplex stream", "recording-output"),
        ("filesink0", "Permission denied", "recording-output"),
        ("nvh264dec0", "CUDA decoder failure", "decoder-health"),
        ("pipeline0", "Internal data flow error", "pipeline"),
    ),
)
def test_gstreamer_error_classification_is_boundary_specific(
    source_name: str,
    detail: str,
    expected: str,
) -> None:
    from Workstation.workstation_core import yolo_stream

    assert (
        yolo_stream.classify_gstreamer_error(
            source_name=source_name,
            detail=detail,
            active_decoder=yolo_stream.NVIDIA_H264_DECODER,
        )
        == expected
    )


def test_only_decoder_health_error_authorizes_auto_cpu_fallback() -> None:
    from Workstation.workstation_core import yolo_stream

    common = {
        "requested_decoder": "auto",
        "active_decoder": yolo_stream.NVIDIA_H264_DECODER,
        "already_fell_back": False,
        "detail": "sentinel failure",
    }
    assert yolo_stream.gstreamer_error_fallback_reason(
        **common,
        error_class="decoder-health",
    )
    for error_class in ("rtsp-source", "recording-output", "pipeline"):
        assert (
            yolo_stream.gstreamer_error_fallback_reason(
                **common,
                error_class=error_class,
            )
            is None
        )


def test_decoder_fallback_evidence_is_critical_and_carries_terminal_ownership() -> None:
    from Workstation.workstation_core import yolo_stream

    control: queue.Queue = queue.Queue(maxsize=1)
    frames: queue.Queue = queue.Queue(maxsize=1)
    output = yolo_stream._BoundedProcessOutputQueue(control, frames)
    output.put(("status", "replaceable status"))
    output.put(
        (
            "decoder_fallback",
            {
                "reason": "nvh264dec sentinel",
                "recording_restart_required": True,
            },
        )
    )
    message = control.get_nowait()
    assert message[0] == "decoder_fallback"

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
    statuses: list[str] = []
    worker.status_changed.connect(statuses.append, QtCore.Qt.DirectConnection)
    worker._handle_worker_message(message)

    assert statuses == ["DECODER_FALLBACK_REQUIRED: nvh264dec sentinel"]
    assert "new side start" in worker._child_terminal_exit_reason


def test_frame_timeout_without_current_rtp_is_source_loss_not_decoder_fallback() -> None:
    from Workstation.workstation_core import yolo_stream

    assert (
        yolo_stream.runtime_decoder_fallback_reason(
            requested_decoder="auto",
            active_decoder=yolo_stream.NVIDIA_H264_DECODER,
            started_s=1.0,
            last_frame_s=2.0,
            now_s=100.0,
            already_fell_back=False,
            encoded_input_active=False,
        )
        is None
    )


def test_recent_rtp_input_evidence_expires_independently_of_decoded_frames() -> None:
    from Workstation.workstation_core import yolo_stream

    tracker = yolo_stream.GstRtpTimingTracker(SimpleNamespace(), SimpleNamespace())
    tracker.register_rtp_sequence(
        ssrc=7,
        sequence=100,
        observed_monotonic_s=10.0,
    )

    assert tracker.has_recent_rtp_input(now_s=11.9, max_age_s=2.0)
    assert not tracker.has_recent_rtp_input(now_s=12.1, max_age_s=2.0)


@pytest.mark.parametrize(
    ("source_name", "expected_fallback", "status_fragment"),
    (
        ("src", False, "RTSP/source 오류"),
        ("filesink0", False, "MP4 mux/filesink 오류"),
        ("nvh264dec0", True, "decoder-health failure"),
    ),
)
def test_bus_error_routing_only_marks_decoder_health_for_fallback(
    monkeypatch: pytest.MonkeyPatch,
    source_name: str,
    expected_fallback: bool,
    status_fragment: str,
) -> None:
    from Workstation.workstation_core import yolo_stream

    fake_gst = SimpleNamespace(
        CLOCK_TIME_NONE=-1,
        MessageType=SimpleNamespace(ERROR=1, EOS=2),
        State=SimpleNamespace(NULL="NULL"),
        init=lambda _value: None,
    )
    repository = ModuleType("gi.repository")
    repository.Gst = fake_gst
    repository.GstRtp = SimpleNamespace()
    gi = ModuleType("gi")
    gi.require_version = lambda *_args: None
    gi.repository = repository
    monkeypatch.setitem(sys.modules, "gi", gi)
    monkeypatch.setitem(sys.modules, "gi.repository", repository)

    messages: list[object] = []

    class OutputQueue:
        def put(self, message: object) -> None:
            messages.append(message)

    class Source:
        def get_name(self) -> str:
            return source_name

    class ErrorMessage:
        type = fake_gst.MessageType.ERROR
        src = Source()

        @staticmethod
        def parse_error() -> tuple[object, None]:
            return SimpleNamespace(message="sentinel bus error"), None

    class Bus:
        def __init__(self) -> None:
            self.message = ErrorMessage()

        def pop_filtered(self, _message_types: object) -> object | None:
            message, self.message = self.message, None
            return message

    class Sink:
        def emit(self, *_args: object) -> None:
            return None

    class Pipeline:
        def get_by_name(self, _name: str) -> None:
            return None

        def set_state(self, _state: object) -> None:
            return None

    tracker = SimpleNamespace(
        pop_h264_access_unit_discontinuity=lambda: None,
        pop_rtp_recovery=lambda: None,
        pop_rtp_discontinuity=lambda: None,
        decoded_output_blocked=lambda: False,
        blocked_duration_s=lambda **_kwargs: 0.0,
        has_recent_rtp_input=lambda **_kwargs: True,
    )
    monkeypatch.setattr(yolo_stream, "GstRtpTimingTracker", lambda *_a, **_k: tracker)
    monkeypatch.setattr(
        yolo_stream,
        "_launch_gst_pipeline",
        lambda *_a, **_k: (
            Pipeline(),
            Sink(),
            Bus(),
            yolo_stream.NVIDIA_H264_DECODER,
        ),
    )
    monkeypatch.setattr(yolo_stream, "_finalize_gst_recording", lambda *_a, **_k: True)
    config = {
        "url": "rtsp://camera.invalid/visible",
        "decoder": "auto",
        "rtsp_transport": "tcp",
        "record_video_path": "/tmp/sentinel.mp4" if source_name == "filesink0" else "",
    }

    assert not yolo_stream._run_gstreamer_stream_once(
        config,
        OutputQueue(),
        threading.Event(),
        SimpleNamespace(),
        None,
        False,
    )

    assert bool(config.get("_runtime_decoder_fallback_reason")) is expected_fallback
    assert any(status_fragment in text for text in _status_texts(messages))


def test_current_generation_frame_is_the_only_active_decoder_ack() -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    observed: list[tuple[str, str | None]] = []
    panel = SimpleNamespace(
        _camera_start_readiness=None,
        left_pane=SimpleNamespace(stream_generation=7),
        right_pane=SimpleNamespace(stream_generation=3),
        _set_active_stream_decoder=lambda side, decoder: observed.append((side, decoder)),
    )
    controller = CameraStreamLifecycleController(panel)

    controller._camera_stream_frame_ready(
        "left",
        object(),
        {"stream_generation": 6, "active_decoder": "nvh264dec"},
    )
    assert observed == []

    controller._camera_stream_frame_ready(
        "left",
        object(),
        {"stream_generation": 7, "active_decoder": "nvh264dec"},
    )
    assert observed == [("left", "nvh264dec")]


def test_decoder_status_text_cannot_activate_gpu_and_terminal_exit_clears_it() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    active = {"left": None, "right": "nvh264dec"}
    panel = SimpleNamespace(
        active_stream_decoders=active,
        _last_logged_camera_states={},
        _camera_stream_start_status=lambda *_args: None,
        _extract_active_decoder_from_state=lambda state: (
            WirisStereoDistancePanel._extract_active_decoder_from_state(panel, state)
        ),
        _set_active_stream_decoder=lambda side, decoder: active.__setitem__(side, decoder),
        _camera_name_ko=lambda side: side,
        _log=lambda _text: None,
    )

    WirisStereoDistancePanel._log_state(
        panel,
        "left",
        "GStreamer 디코더: nvh264dec",
    )
    assert active["left"] is None

    WirisStereoDistancePanel._log_state(
        panel,
        "right",
        "스트림 프로세스 크래시 복구 한도 초과: exitcode=1",
    )
    assert active["right"] is None


def test_preview_fallback_override_and_reason_are_per_side() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    active = {"left": "nvh264dec", "right": "nvh264dec"}
    panel = SimpleNamespace(
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        _runtime_decoder_overrides={"left": None, "right": None},
        decoder_fallback_reasons={"left": "", "right": ""},
        active_stream_decoders=active,
        _set_active_stream_decoder=lambda side, decoder: active.__setitem__(side, decoder),
        _camera_name_ko=lambda side: side,
        _log=lambda _text: None,
    )

    WirisStereoDistancePanel._handle_runtime_decoder_fallback_active(
        panel,
        "left",
        "DECODER_FALLBACK_ACTIVE: nvh264dec decoded-frame stall; active=avdec_h264 pending frame",
    )

    assert panel._runtime_decoder_overrides == {"left": "software", "right": None}
    assert "decoded-frame stall" in panel.decoder_fallback_reasons["left"]
    assert panel.decoder_fallback_reasons["right"] == ""
    assert active == {"left": None, "right": "nvh264dec"}


def test_recording_decoder_failure_does_not_restart_healthy_side() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    stops: list[str] = []
    start_failures: list[tuple[str, str]] = []
    active = {"left": "nvh264dec", "right": "nvh264dec"}
    panel = SimpleNamespace(
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        _runtime_decoder_overrides={"left": None, "right": None},
        decoder_fallback_reasons={"left": "", "right": ""},
        active_stream_decoders=active,
        _measurement_run_state=lambda: "RUNNING",
        _camera_stream_start_status=lambda side, state: start_failures.append((side, state)),
        _set_active_stream_decoder=lambda side, decoder: active.__setitem__(side, decoder),
        _camera_name_ko=lambda side: side,
        _log=lambda _text: None,
        stop_yolo_streams=lambda: stops.append("both"),
    )

    WirisStereoDistancePanel._handle_runtime_decoder_fallback(
        panel,
        "left",
        "DECODER_FALLBACK_REQUIRED: nvh264dec bus error; recording finalized",
    )

    assert stops == []
    assert panel._runtime_decoder_overrides == {"left": "software", "right": None}
    assert panel.decoder_fallback_reasons["right"] == ""
    assert active == {"left": None, "right": "nvh264dec"}
    assert start_failures and start_failures[0][0] == "left"


def test_side_decoder_mode_keeps_healthy_side_on_auto() -> None:
    from Workstation.standalone_gui import decoder_mode_for_side

    overrides = {"left": "software", "right": None}
    assert decoder_mode_for_side("auto", overrides, "left") == "software"
    assert decoder_mode_for_side("auto", overrides, "right") == "auto"


def test_runtime_status_projects_fallback_per_side_without_degrading_peer() -> None:
    from Workstation.status_model import BackendStatusAdapter, StatusSeverity

    panel = SimpleNamespace(
        left_pane=SimpleNamespace(current_status="playing"),
        right_pane=SimpleNamespace(current_status="playing"),
        camera_connected=True,
        camera_frame_times={"left": [99.9], "right": [99.9]},
        decoder_combo=SimpleNamespace(currentData=lambda: "auto"),
        active_stream_decoders={"left": "avdec_h264", "right": "nvh264dec"},
        _runtime_decoder_overrides={"left": "software", "right": None},
        decoder_fallback_reasons={
            "left": "nvh264dec decoder-health error",
            "right": "",
        },
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    adapter._refresh_camera(panel, "left", "CAM L", now_s=100.0)
    adapter._refresh_camera(panel, "right", "CAM R", now_s=100.0)

    left = adapter.model.record("camera.left")
    right = adapter.model.record("camera.right")
    assert left is not None and left.severity == StatusSeverity.WARN
    assert "CPU fallback · avdec_h264" in left.detail
    assert right is not None and right.severity == StatusSeverity.OK
    assert "GPU · nvh264dec" in right.detail


def test_terminal_decoder_exit_is_not_misclassified_as_a_restartable_crash() -> None:
    from Workstation.workstation_core.yolo_stream import YoloStreamWorker

    worker = YoloStreamWorker(
        url="rtsp://camera.invalid/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        process_target=_publish_terminal_decoder_exit,
        mp_context=multiprocessing.get_context("spawn"),
        crash_restart_delays_s=(0.01, 0.01),
        rtsp_reconnect_policy="automatic",
    )
    statuses: list[str] = []
    worker.status_changed.connect(statuses.append, QtCore.Qt.DirectConnection)
    owner = threading.Thread(target=worker.run, name="decoder-terminal-exit-test")

    owner.start()
    owner.join(5.0)

    assert not owner.is_alive()
    assert "decoder terminal sentinel" in statuses
    assert len([text for text in statuses if "worker 시작 중: pid=" in text]) == 1
    assert any("영구 종료" in text and "자동 재시작 안 함" in text for text in statuses)
    assert not any("크래시 복구 중" in text for text in statuses)
