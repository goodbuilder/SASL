from __future__ import annotations

import multiprocessing
import queue
import time

from Workstation.workstation_core.native_frame_ring import SharedNativeFrameRing
from Workstation.workstation_core.yolo_stream import (
    CameraIpcCounters,
    _BoundedProcessOutputQueue,
)


def _high_rate_camera_producer(
    ring: SharedNativeFrameRing,
    output: _BoundedProcessOutputQueue,
    stop_event,
) -> None:
    native_payload = bytes([31]) * (64 * 48 * 3)
    preview_payload = bytes([63]) * (16 * 12 * 3)
    sequence = 0
    while not stop_event.is_set():
        sequence += 1
        descriptor = ring.publish_bytes(
            native_payload,
            width=64,
            height=48,
            channels=3,
            timestamp_s=time.monotonic(),
        )
        metadata = descriptor.to_mapping()
        output.put(("frame", metadata, 16, 12, preview_payload))
        output.put(
            (
                "detection",
                time.monotonic(),
                64,
                48,
                None,
                [],
                metadata,
            )
        )
        if sequence % 100 == 0:
            output.put(("status", f"producer={sequence}"))
    ring.close()


def test_two_second_gui_stall_keeps_every_channel_bounded_and_stop_live() -> None:
    context = multiprocessing.get_context("spawn")
    ring = SharedNativeFrameRing.create(
        camera_id="left",
        stream_generation=1,
        capacity=4,
        max_width=64,
        max_height=48,
        channels=3,
        mp_context=context,
    )
    control_queue = context.Queue(maxsize=4)
    metadata_queue = context.Queue(maxsize=3)
    preview_queue = context.Queue(maxsize=1)
    counters = CameraIpcCounters(context)
    stop_event = context.Event()
    process = context.Process(
        target=_high_rate_camera_producer,
        args=(
            ring,
            _BoundedProcessOutputQueue(
                control_queue,
                preview_queue,
                metadata_queue,
                counters,
            ),
            stop_event,
        ),
    )
    process.start()
    try:
        # Deliberately do not drain any GUI-facing channel.
        time.sleep(2.0)
        stop_started_s = time.monotonic()
        stop_event.set()
        process.join(timeout=0.5)
        stop_elapsed_s = time.monotonic() - stop_started_s

        assert process.exitcode == 0
        assert stop_elapsed_s < 0.5
        assert preview_queue.qsize() <= 1
        assert metadata_queue.qsize() <= 3
        assert control_queue.qsize() <= 4
        metrics = counters.snapshot()
        assert metrics["preview_replaced"] > 0
        assert metrics["metadata_replaced"] > 0
        ring_metrics = ring.metrics()
        assert ring_metrics["ready_slots"] <= 4
        assert ring_metrics["dropped"] == 0

        preview = preview_queue.get_nowait()
        detection = metadata_queue.get_nowait()
        assert len(preview[4]) == 16 * 12 * 3
        assert detection[0] == "detection"
        assert detection[-1]["frame_key"].startswith("left:1:")
        assert not any(
            isinstance(value, (bytes, bytearray, memoryview))
            for value in detection
        )
    finally:
        if process.is_alive():
            stop_event.set()
            process.terminate()
            process.join(timeout=1.0)
        for message_queue in (control_queue, metadata_queue, preview_queue):
            try:
                message_queue.cancel_join_thread()
            except (AttributeError, OSError, ValueError):
                pass
            message_queue.close()
        ring.close()
        ring.unlink()
