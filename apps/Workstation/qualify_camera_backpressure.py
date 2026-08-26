"""Synthetic qualification for bounded Workstation camera data paths."""

from __future__ import annotations

import argparse
import json
import multiprocessing
import os
import queue
import tempfile
import time
from pathlib import Path

from Workstation.workstation_core.detection_pair_coordinator import (
    DetectionPairCoordinator,
    PairingPolicy,
)
from Workstation.workstation_core.native_evidence_writer import (
    AsyncNativeEvidenceWriter,
)
from Workstation.workstation_core.native_frame_ring import (
    FrameKey,
    SharedNativeFrameRing,
)
from Workstation.workstation_core.stereo import DetectionObservation
from Workstation.workstation_core.yolo_stream import (
    CameraIpcCounters,
    _BoundedProcessOutputQueue,
)


NATIVE_WIDTH = 320
NATIVE_HEIGHT = 180
PREVIEW_WIDTH = 160
PREVIEW_HEIGHT = 90
PRODUCER_RATE_HZ = 15.0


def _rss_bytes() -> int:
    fields = Path("/proc/self/statm").read_text(encoding="ascii").split()
    return int(fields[1]) * int(os.sysconf("SC_PAGE_SIZE"))


def _producer(
    side: str,
    ring: SharedNativeFrameRing,
    output: _BoundedProcessOutputQueue,
    stop_event,
) -> None:
    native = bytes([37 if side == "left" else 73]) * (
        NATIVE_WIDTH * NATIVE_HEIGHT * 3
    )
    preview = bytes([19 if side == "left" else 29]) * (
        PREVIEW_WIDTH * PREVIEW_HEIGHT * 3
    )
    interval_s = 1.0 / PRODUCER_RATE_HZ
    next_frame_s = time.monotonic()
    sequence = 0
    while not stop_event.is_set():
        now_s = time.monotonic()
        if now_s < next_frame_s:
            stop_event.wait(next_frame_s - now_s)
            continue
        next_frame_s += interval_s
        sequence += 1
        descriptor = ring.publish_bytes(
            native,
            width=NATIVE_WIDTH,
            height=NATIVE_HEIGHT,
            channels=3,
            timestamp_s=now_s,
            source_timestamp_s=now_s,
        )
        metadata = descriptor.to_mapping()
        metadata["timestamp_reference"] = "app-receive"
        output.put(("frame", metadata, PREVIEW_WIDTH, PREVIEW_HEIGHT, preview))
        output.put(
            (
                "detection",
                now_s,
                NATIVE_WIDTH,
                NATIVE_HEIGHT,
                {
                    "x1": 100.0 if side == "left" else 92.0,
                    "y1": 60.0,
                    "x2": 140.0 if side == "left" else 132.0,
                    "y2": 120.0,
                    "confidence": 0.9,
                    "class_id": 0,
                    "class_name": "synthetic-target",
                },
                [{"class_id": 0}],
                metadata,
            )
        )
        if sequence % 30 == 0:
            output.put(("status", f"{side} sequence={sequence}"))
    ring.close()


def _drain_latest(message_queue):
    latest = None
    while True:
        try:
            latest = message_queue.get_nowait()
        except queue.Empty:
            return latest


def run_qualification(
    *,
    duration_s: float,
    gui_stall_s: float,
    evidence_period_s: float,
) -> dict[str, object]:
    context = multiprocessing.get_context("spawn")
    cameras: dict[str, dict[str, object]] = {}
    for side in ("left", "right"):
        ring = SharedNativeFrameRing.create(
            camera_id=side,
            stream_generation=1,
            capacity=16,
            max_width=NATIVE_WIDTH,
            max_height=NATIVE_HEIGHT,
            channels=3,
            mp_context=context,
        )
        control = context.Queue(maxsize=8)
        metadata = context.Queue(maxsize=4)
        preview = context.Queue(maxsize=1)
        counters = CameraIpcCounters(context)
        stop = context.Event()
        process = context.Process(
            target=_producer,
            args=(
                side,
                ring,
                _BoundedProcessOutputQueue(
                    control,
                    preview,
                    metadata,
                    counters,
                ),
                stop,
            ),
        )
        process.start()
        cameras[side] = {
            "ring": ring,
            "control": control,
            "metadata": metadata,
            "preview": preview,
            "counters": counters,
            "stop": stop,
            "process": process,
            "queue_high_water": {"control": 0, "metadata": 0, "preview": 0},
        }

    pairer = DetectionPairCoordinator(input_capacity=16, buffer_capacity=16)
    started_s = time.monotonic()
    stall_start_s = started_s + min(5.0, max(0.5, duration_s * 0.2))
    stall_end_s = stall_start_s + gui_stall_s
    next_evidence_s = started_s + evidence_period_s
    rss_samples: list[tuple[float, int]] = []
    submitted_pair_keys: set[tuple[str, str]] = set()
    policy = PairingPolicy(timestamp_basis="detection", revision=1)

    with tempfile.TemporaryDirectory(prefix="workstation-native-evidence-") as root:
        writer = AsyncNativeEvidenceWriter(root, max_pending=4)
        try:
            while True:
                now_s = time.monotonic()
                if now_s - started_s >= duration_s:
                    break
                rss_samples.append((now_s, _rss_bytes()))
                stalled = stall_start_s <= now_s < stall_end_s
                for side, state in cameras.items():
                    for name in ("control", "metadata", "preview"):
                        try:
                            depth = int(state[name].qsize())
                        except (NotImplementedError, AttributeError):
                            depth = 0
                        state["queue_high_water"][name] = max(
                            state["queue_high_water"][name],
                            depth,
                        )
                    if stalled:
                        continue
                    _drain_latest(state["control"])
                    _drain_latest(state["preview"])
                    detection_message = _drain_latest(state["metadata"])
                    if detection_message is not None and detection_message[0] == "detection":
                        observation = DetectionObservation.from_payload(
                            side,
                            {
                                "timestamp_s": detection_message[1],
                                "width": detection_message[2],
                                "height": detection_message[3],
                                "best_detection": detection_message[4],
                                "detections": detection_message[5],
                                "timestamp_metadata": detection_message[6],
                            },
                        )
                        if observation is not None:
                            pairer.submit(side, observation, policy)

                pair_result = pairer.poll_latest()
                if (
                    pair_result is not None
                    and pair_result.pair is not None
                    and now_s >= next_evidence_s
                ):
                    pair = pair_result.pair
                    left_key = FrameKey.from_mapping(pair.left.timestamp_metadata)
                    right_key = FrameKey.from_mapping(pair.right.timestamp_metadata)
                    identity = (left_key.text, right_key.text)
                    if identity not in submitted_pair_keys:
                        request_id = f"synthetic-{len(submitted_pair_keys) + 1}"
                        writer.enqueue_ring_pair(
                            request_id=request_id,
                            pair_id=request_id,
                            purpose="qualification",
                            frames=(
                                (
                                    "left",
                                    cameras["left"]["ring"],
                                    left_key,
                                    pair.left.timestamp_metadata,
                                ),
                                (
                                    "right",
                                    cameras["right"]["ring"],
                                    right_key,
                                    pair.right.timestamp_metadata,
                                ),
                            ),
                            context={"qualification": True},
                        )
                        submitted_pair_keys.add(identity)
                    next_evidence_s = now_s + evidence_period_s
                time.sleep(0.02)

            stop_latencies: dict[str, float] = {}
            for side, state in cameras.items():
                stop_started_s = time.monotonic()
                state["stop"].set()
                state["process"].join(timeout=0.5)
                stop_latencies[side] = time.monotonic() - stop_started_s
            writer_deadline_s = time.monotonic() + 10.0
            while writer.pending_count and time.monotonic() < writer_deadline_s:
                time.sleep(0.01)
            writer_metrics = writer.metrics()
            writer.close()
            pair_metrics = pairer.metrics()
            camera_results: dict[str, object] = {}
            for side, state in cameras.items():
                camera_results[side] = {
                    "exitcode": state["process"].exitcode,
                    "stop_latency_s": stop_latencies[side],
                    "queues": state["queue_high_water"],
                    "ipc": state["counters"].snapshot(),
                    "ring": state["ring"].metrics(),
                }

            rss_start = rss_samples[0][1] if rss_samples else _rss_bytes()
            rss_end = rss_samples[-1][1] if rss_samples else rss_start
            rss_high = max((sample[1] for sample in rss_samples), default=rss_end)
            elapsed_h = max(duration_s, 1.0) / 3600.0
            rss_slope_bytes_per_hour = (rss_end - rss_start) / elapsed_h
            passed = bool(
                all(row["exitcode"] == 0 for row in camera_results.values())
                and all(
                    row["stop_latency_s"] < 0.5
                    for row in camera_results.values()
                )
                and all(
                    row["queues"]["preview"] <= 1
                    and row["queues"]["metadata"] <= 4
                    and row["queues"]["control"] <= 8
                    and row["ring"]["dropped"] == 0
                    for row in camera_results.values()
                )
                and writer_metrics["failed"] == 0
                and writer_metrics["pending"] == 0
            )
            return {
                "schema": "workstation-camera-backpressure-qualification-v1",
                "passed": passed,
                "duration_s": duration_s,
                "gui_stall_s": gui_stall_s,
                "rss_start_bytes": rss_start,
                "rss_end_bytes": rss_end,
                "rss_high_water_bytes": rss_high,
                "rss_slope_bytes_per_hour": rss_slope_bytes_per_hour,
                "pairing": pair_metrics,
                "evidence": writer_metrics,
                "cameras": camera_results,
            }
        finally:
            pairer.close()
            for state in cameras.values():
                if state["process"].is_alive():
                    state["stop"].set()
                    state["process"].terminate()
                    state["process"].join(timeout=1.0)
                for name in ("control", "metadata", "preview"):
                    message_queue = state[name]
                    try:
                        message_queue.cancel_join_thread()
                    except (AttributeError, OSError, ValueError):
                        pass
                    message_queue.close()
                state["ring"].close()
                state["ring"].unlink()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--gui-stall", type=float, default=2.0)
    parser.add_argument("--evidence-period", type=float, default=2.0)
    args = parser.parse_args()
    result = run_qualification(
        duration_s=max(1.0, args.duration),
        gui_stall_s=max(0.0, args.gui_stall),
        evidence_period_s=max(0.1, args.evidence_period),
    )
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
