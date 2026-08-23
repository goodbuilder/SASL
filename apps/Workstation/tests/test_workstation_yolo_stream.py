from __future__ import annotations

import multiprocessing
import os
import queue
import signal
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore

from Workstation.workstation_core.yolo_stream import (
    AdaptiveYoloRateLimiter,
    DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_INITIAL_RATE_HZ,
    DEFAULT_PREVIEW_DISPLAY_RATE_HZ,
    RollingFrameRateEstimator,
    YoloStreamWorker,
    _BoundedProcessOutputQueue,
    _activate_cpu_fallback_rate_limiter,
    _process_frame_for_output,
    _should_emit_preview_frame,
    filter_detections_by_class_gate,
    DetectionBox,
)


def _crash_once_then_publish_live(config, output_queue, stop_event) -> None:
    marker = Path(str(config["_supervisor_test_marker"]))
    if not marker.exists():
        marker.write_text(str(os.getpid()), encoding="utf-8")
        os.kill(os.getpid(), signal.SIGKILL)
    output_queue.put(("status", f"supervisor sentinel live pid={os.getpid()}"))
    while not stop_event.wait(0.02):
        pass


def test_class_gate_filters_before_highest_confidence_selection() -> None:
    detections = [
        DetectionBox(0.0, 0.0, 20.0, 20.0, 0.99, 1, "car"),
        DetectionBox(2.0, 2.0, 22.0, 22.0, 0.40, 0, "person"),
    ]

    filtered = filter_detections_by_class_gate(
        detections,
        enabled=True,
        target="person",
    )

    assert [detection.class_id for detection in filtered] == [0]


def test_class_gate_accepts_numeric_ids_and_fails_closed_when_target_is_empty() -> None:
    detections = [
        DetectionBox(0.0, 0.0, 20.0, 20.0, 0.5, 7, "truck"),
        DetectionBox(2.0, 2.0, 22.0, 22.0, 0.6, 2, "car"),
    ]

    assert [
        detection.class_id
        for detection in filter_detections_by_class_gate(
            detections,
            enabled=True,
            target="#7",
        )
    ] == [7]
    assert filter_detections_by_class_gate(detections, enabled=True, target="") == []
    assert filter_detections_by_class_gate(detections, enabled=False, target="") == detections


def test_class_gate_is_applied_before_detection_event_is_emitted() -> None:
    class Scalar:
        def __init__(self, value: float | int) -> None:
            self.value = value

        def item(self) -> float | int:
            return self.value

    class XYXY:
        def __init__(self, values: list[float]) -> None:
            self.values = values

        def tolist(self) -> list[float]:
            return list(self.values)

    class Box:
        def __init__(self, class_id: int, confidence: float, values: list[float]) -> None:
            self.cls = Scalar(class_id)
            self.conf = Scalar(confidence)
            self.xyxy = [XYXY(values)]

    class Model:
        names = {0: "person", 1: "car"}

        def predict(self, *_args: object, **_kwargs: object):
            return [
                SimpleNamespace(
                    boxes=[
                        Box(1, 0.99, [0.0, 0.0, 20.0, 20.0]),
                        Box(0, 0.40, [2.0, 2.0, 22.0, 22.0]),
                    ]
                )
            ]

    class Output:
        def __init__(self) -> None:
            self.messages: list[object] = []

        def put(self, message: object) -> None:
            self.messages.append(message)

    output = Output()
    _annotated, best, inferred = _process_frame_for_output(
        None,
        SimpleNamespace(shape=(480, 640, 3)),
        model=Model(),
        config={
            "device": "cpu",
            "_requested_yolo_device": "cpu",
            "input_size": 640,
            "conf_threshold": 0.5,
            "iou_threshold": 0.45,
            "class_gate_enabled": True,
            "class_gate": "person",
            "emit_measurement_frames": False,
        },
        output_queue=output,
        now_s=1.0,
        enable_detection=True,
    )

    assert inferred is True
    assert best is not None and best.class_id == 0
    event = output.messages[-1]
    assert isinstance(event, tuple)
    assert event[0] == "detection"
    assert event[4]["class_id"] == 0
    assert [row["class_id"] for row in event[5]] == [0]


def test_preview_output_rate_limiter_keeps_latest_frame_timing_bounded() -> None:
    config = {"display_rate_hz": 10.0}

    assert _should_emit_preview_frame(config, 1.0)
    assert not _should_emit_preview_frame(config, 1.05)
    assert _should_emit_preview_frame(config, 1.1)


def test_preview_output_rate_limiter_accepts_invalid_or_unlimited_values() -> None:
    assert _should_emit_preview_frame({"display_rate_hz": "invalid"}, 1.0)
    assert _should_emit_preview_frame({"display_rate_hz": 0.0}, 1.0)
    assert _should_emit_preview_frame({"display_rate_hz": -1.0}, 1.0)


def test_decoded_rate_uses_a_rolling_window_and_expires_after_a_gap() -> None:
    estimator = RollingFrameRateEstimator(window_s=5.0)
    for index in range(100):
        estimator.add(index / 20.0)

    assert estimator.rate(5.0) == pytest.approx(19.8)
    assert estimator.rate(10.1) == 0.0


def test_preview_queue_replaces_stale_frame_but_preserves_events() -> None:
    event_queue: queue.Queue = queue.Queue(maxsize=1)
    frame_queue: queue.Queue = queue.Queue(maxsize=1)
    output_queue = _BoundedProcessOutputQueue(event_queue, frame_queue)

    output_queue.put(("frame", {}, 1, 1, b"old"))
    output_queue.put(("frame", {}, 1, 1, b"latest"))
    output_queue.put(("detection", 1.0, 1, 1, {"confidence": 0.9}, [], {}))

    assert frame_queue.get_nowait()[4] == b"latest"
    assert event_queue.get_nowait()[0] == "detection"
    assert DEFAULT_PREVIEW_DISPLAY_RATE_HZ == 15.0


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


@pytest.mark.skipif(not hasattr(signal, "SIGKILL"), reason="requires SIGKILL")
def test_stream_supervisor_recovers_after_child_sigkill(tmp_path: Path) -> None:
    marker = tmp_path / "first-child.pid"
    worker = YoloStreamWorker(
        url="rtsp://camera.invalid/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        process_target=_crash_once_then_publish_live,
        mp_context=multiprocessing.get_context("spawn"),
        crash_restart_delays_s=(0.05, 0.1, 0.2),
    )
    worker.config["_supervisor_test_marker"] = str(marker)
    statuses: list[str] = []
    finished: list[bool] = []
    worker.status_changed.connect(statuses.append, QtCore.Qt.DirectConnection)
    worker.finished.connect(lambda: finished.append(True), QtCore.Qt.DirectConnection)
    owner = threading.Thread(target=worker.run, name="yolo-supervisor-test")
    owner.start()
    try:
        deadline_s = time.monotonic() + 5.0
        while time.monotonic() < deadline_s and not any(
            "supervisor sentinel live" in status for status in statuses
        ):
            time.sleep(0.01)

        assert any("스트림 프로세스 크래시 복구 중" in status for status in statuses)
        assert any("supervisor sentinel live" in status for status in statuses)
        child_start_messages = [status for status in statuses if "worker 시작 중: pid=" in status]
        assert len(child_start_messages) >= 2
        assert marker.is_file()
    finally:
        worker.request_stop()
        owner.join(5.0)
    assert not owner.is_alive()
    assert finished == [True]
