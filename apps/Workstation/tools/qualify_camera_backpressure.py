"""Synthetic qualification for bounded Workstation camera data paths."""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import os
import queue
import tempfile
import time
from collections import Counter
from collections.abc import Mapping
from multiprocessing import shared_memory
from pathlib import Path

from Workstation.workstation_core.detection_pair_coordinator import (
    DetectionPairCoordinator,
    PairingPolicy,
)
from Workstation.workstation_core.native_evidence_writer import (
    AsyncNativeEvidenceWriter,
    RingPairReservation,
    reserve_ring_pair,
)
from Workstation.workstation_core.native_frame_ring import (
    FrameKey,
    SharedNativeFrameRing,
)
from Workstation.workstation_core.stereo import DetectionObservation
from Workstation.workstation_core.media_stream.ipc import CameraIpcCounters, _BoundedProcessOutputQueue


NATIVE_WIDTH = 320
NATIVE_HEIGHT = 180
PREVIEW_WIDTH = 160
PREVIEW_HEIGHT = 90
PRODUCER_RATE_HZ = 15.0
MIB = 1024 * 1024
DEFAULT_MINIMUM_SAVED_RATIO = 0.90
DEFAULT_MAX_RSS_GROWTH_BYTES = 128 * MIB
DEFAULT_MAX_RSS_HIGH_WATER_BYTES = 512 * MIB
DEFAULT_MAX_RSS_SLOPE_BYTES_PER_HOUR = 256 * MIB
DEFAULT_MAX_STOP_LATENCY_S = 0.5
DEFAULT_RSS_SAMPLE_INTERVAL_S = 1.0
DEFAULT_WRITER_SHUTDOWN_TIMEOUT_S = 5.0


def _reserve_qualification_pair(
    pair: object,
    sources: Mapping[str, object | None],
) -> RingPairReservation | None:
    frames = []
    for side, observation in (("left", pair.left), ("right", pair.right)):
        key = FrameKey.from_mapping(observation.timestamp_metadata)
        ring = sources.get(side)
        if not isinstance(ring, SharedNativeFrameRing):
            return None
        frames.append((side, ring, key, observation.timestamp_metadata))
    return reserve_ring_pair(frames)


def _finite_nonnegative(value: object, *, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite non-negative number") from exc
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number")
    return parsed


def _minimum_saved_pairs(
    *,
    duration_s: float,
    evidence_period_s: float,
    configured: int | None,
    minimum_ratio: float,
) -> tuple[int, int]:
    theoretical = max(
        0,
        int(math.floor(max(0.0, duration_s - 1e-9) / evidence_period_s)),
    )
    if configured is not None:
        return max(1, int(configured)), theoretical
    return max(1, int(math.floor(theoretical * minimum_ratio))), theoretical


def _rss_sample_summary(
    samples: list[tuple[float, int]],
) -> dict[str, object]:
    if not samples:
        current = _rss_bytes()
        samples = [(time.monotonic(), current)]
    started_s = samples[0][0]
    values = [int(sample[1]) for sample in samples]
    elapsed_s = max(0.0, samples[-1][0] - started_s)
    start_bytes = values[0]
    end_bytes = values[-1]
    high_water_bytes = max(values)
    slope_elapsed_h = max(elapsed_s, 1.0) / 3600.0

    def compact(sample: tuple[float, int]) -> dict[str, object]:
        return {
            "elapsed_s": round(max(0.0, sample[0] - started_s), 6),
            "rss_bytes": int(sample[1]),
        }

    return {
        "sample_count": len(samples),
        "elapsed_s": elapsed_s,
        "start_bytes": start_bytes,
        "end_bytes": end_bytes,
        "minimum_bytes": min(values),
        "high_water_bytes": high_water_bytes,
        "growth_bytes": end_bytes - start_bytes,
        "high_water_growth_bytes": high_water_bytes - start_bytes,
        "slope_bytes_per_hour": (end_bytes - start_bytes) / slope_elapsed_h,
        # The CLI already emits JSON, so retain the bounded 1 Hz-by-default raw
        # series in the same artifact instead of reducing qualification to one
        # start/end claim.  first/last_samples stay as a compact console aid.
        "samples": [compact(sample) for sample in samples],
        "first_samples": [compact(sample) for sample in samples[:3]],
        "last_samples": [compact(sample) for sample in samples[-3:]],
    }


def _collect_writer_results(
    writer: AsyncNativeEvidenceWriter,
    outstanding: set[str],
    summary: dict[str, object],
) -> None:
    states = summary.setdefault("states", Counter())
    statuses = summary.setdefault("statuses", Counter())
    assert isinstance(states, Counter)
    assert isinstance(statuses, Counter)
    for request_id in tuple(outstanding):
        payload = writer.result(request_id)
        if payload is None:
            continue
        fields = payload.get("fields")
        fields = dict(fields) if isinstance(fields, Mapping) else {}
        state = str(fields.get("evidence_state") or "MISSING")
        status = str(fields.get("native_pair_status") or "missing")
        states[state] += 1
        statuses[status] += 1
        summary["collected"] = int(summary.get("collected", 0)) + 1
        valid_saved = bool(
            state == "SAVED"
            and status == "complete"
            and int(fields.get("evidence_artifact_count") or 0) == 2
            and str(fields.get("evidence_ack_sha256") or "")
        )
        if not valid_saved:
            summary["invalid"] = int(summary.get("invalid", 0)) + 1
        writer.acknowledge(request_id)
        outstanding.discard(request_id)


def _shared_memory_leaks(names: list[str]) -> list[str]:
    leaked: list[str] = []
    for name in names:
        try:
            segment = shared_memory.SharedMemory(name=name, create=False)
        except FileNotFoundError:
            continue
        leaked.append(name)
        try:
            segment.unlink()
        except FileNotFoundError:
            pass
        finally:
            segment.close()
    return leaked


def _qualification_checks(
    *,
    camera_results: Mapping[str, Mapping[str, object]],
    writer_metrics: Mapping[str, object],
    writer_result_summary: Mapping[str, object],
    minimum_saved_pairs: int,
    writer_close_ok: bool,
    writer_thread_stopped: bool,
    pairer_close_ok: bool,
    leaked_child_pids: list[int],
    leaked_shared_memory_names: list[str],
    cleanup_errors: list[str],
    rss_summary: Mapping[str, object],
    max_rss_growth_bytes: int,
    max_rss_high_water_bytes: int,
    max_rss_slope_bytes_per_hour: int,
    max_stop_latency_s: float,
) -> tuple[bool, dict[str, bool], list[str]]:
    checks = {
        "camera_sides_present": set(camera_results) == {"left", "right"},
        "camera_processes_exited_cleanly": all(
            row.get("exitcode") == 0
            and not bool(row.get("forced_termination"))
            for row in camera_results.values()
        ),
        "stop_latency_within_limit": all(
            row.get("stop_latency_s") is not None
            and float(row["stop_latency_s"])
            <= float(max_stop_latency_s)
            for row in camera_results.values()
        ),
        "queues_within_capacity": all(
            row.get("queue_depth_supported") is True
            and int(dict(row.get("queues") or {}).get("preview", 0)) <= 1
            and int(dict(row.get("queues") or {}).get("metadata", 0)) <= 4
            and int(dict(row.get("queues") or {}).get("control", 0)) <= 8
            for row in camera_results.values()
        ),
        "native_rings_lossless": all(
            int(dict(row.get("ring") or {}).get("dropped", 0)) == 0
            and int(dict(row.get("ring") or {}).get("active_leases", 0)) == 0
            for row in camera_results.values()
        ),
        "evidence_no_failures": int(writer_metrics.get("failed") or 0) == 0,
        "evidence_pending_drained": int(writer_metrics.get("pending") or 0) == 0,
        "evidence_no_frame_unavailable": int(
            writer_metrics.get("frame_unavailable") or 0
        )
        == 0,
        "evidence_queue_not_full": int(writer_metrics.get("queue_full") or 0) == 0,
        "evidence_results_acknowledged": int(
            writer_metrics.get("unacknowledged_results") or 0
        )
        == 0,
        "evidence_saved_minimum": int(writer_metrics.get("saved") or 0)
        >= int(minimum_saved_pairs),
        "evidence_results_valid": int(writer_result_summary.get("invalid") or 0) == 0,
        "evidence_results_collected": int(
            writer_result_summary.get("outstanding") or 0
        )
        == 0,
        "writer_closed": bool(writer_close_ok),
        "writer_thread_stopped": bool(writer_thread_stopped),
        "pairer_thread_stopped": bool(pairer_close_ok),
        "no_active_child_leaks": not leaked_child_pids,
        "no_shared_memory_leaks": not leaked_shared_memory_names,
        "cleanup_error_free": not cleanup_errors,
        "rss_growth_within_limit": int(rss_summary.get("growth_bytes") or 0)
        <= int(max_rss_growth_bytes),
        "rss_high_water_within_limit": int(
            rss_summary.get("high_water_bytes") or 0
        )
        <= int(max_rss_high_water_bytes),
        "rss_slope_within_limit": float(
            rss_summary.get("slope_bytes_per_hour") or 0.0
        )
        <= float(max_rss_slope_bytes_per_hour),
    }
    failures = [name for name, passed in checks.items() if not passed]
    return not failures, checks, failures


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
    minimum_saved_pairs: int | None = None,
    minimum_saved_ratio: float = DEFAULT_MINIMUM_SAVED_RATIO,
    max_rss_growth_bytes: int = DEFAULT_MAX_RSS_GROWTH_BYTES,
    max_rss_high_water_bytes: int = DEFAULT_MAX_RSS_HIGH_WATER_BYTES,
    max_rss_slope_bytes_per_hour: int = DEFAULT_MAX_RSS_SLOPE_BYTES_PER_HOUR,
    max_stop_latency_s: float = DEFAULT_MAX_STOP_LATENCY_S,
    rss_sample_interval_s: float = DEFAULT_RSS_SAMPLE_INTERVAL_S,
    writer_shutdown_timeout_s: float = DEFAULT_WRITER_SHUTDOWN_TIMEOUT_S,
) -> dict[str, object]:
    duration_s = max(1.0, _finite_nonnegative(duration_s, name="duration_s"))
    gui_stall_s = _finite_nonnegative(gui_stall_s, name="gui_stall_s")
    evidence_period_s = max(
        0.1,
        _finite_nonnegative(evidence_period_s, name="evidence_period_s"),
    )
    minimum_saved_ratio = _finite_nonnegative(
        minimum_saved_ratio,
        name="minimum_saved_ratio",
    )
    if minimum_saved_ratio > 1.0:
        raise ValueError("minimum_saved_ratio must be <= 1.0")
    rss_sample_interval_s = max(
        0.05,
        _finite_nonnegative(rss_sample_interval_s, name="rss_sample_interval_s"),
    )
    writer_shutdown_timeout_s = max(
        0.1,
        _finite_nonnegative(
            writer_shutdown_timeout_s,
            name="writer_shutdown_timeout_s",
        ),
    )
    max_stop_latency_s = max(
        0.01,
        _finite_nonnegative(max_stop_latency_s, name="max_stop_latency_s"),
    )
    max_rss_growth_bytes = int(
        _finite_nonnegative(max_rss_growth_bytes, name="max_rss_growth_bytes")
    )
    max_rss_high_water_bytes = int(
        _finite_nonnegative(
            max_rss_high_water_bytes,
            name="max_rss_high_water_bytes",
        )
    )
    max_rss_slope_bytes_per_hour = int(
        _finite_nonnegative(
            max_rss_slope_bytes_per_hour,
            name="max_rss_slope_bytes_per_hour",
        )
    )
    required_saved_pairs, theoretical_saved_pairs = _minimum_saved_pairs(
        duration_s=duration_s,
        evidence_period_s=evidence_period_s,
        configured=minimum_saved_pairs,
        minimum_ratio=minimum_saved_ratio,
    )

    context = multiprocessing.get_context("spawn")
    baseline_child_pids = {
        int(process.pid)
        for process in multiprocessing.active_children()
        if process.pid is not None
    }
    rss_samples: list[tuple[float, int]] = [(time.monotonic(), _rss_bytes())]
    cameras: dict[str, dict[str, object]] = {}
    shared_memory_names: list[str] = []
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
        shared_memory_names.append(ring.shm_name)
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
            "queue_depth_supported": True,
            "forced_termination": False,
        }

    pairer = DetectionPairCoordinator(
        input_capacity=16,
        buffer_capacity=16,
        reservation_factory=_reserve_qualification_pair,
    )
    started_s = time.monotonic()
    stall_start_s = started_s + min(5.0, max(0.5, duration_s * 0.2))
    stall_end_s = stall_start_s + gui_stall_s
    next_evidence_s = started_s + evidence_period_s
    next_rss_sample_s = time.monotonic()
    submitted_pair_keys: set[tuple[str, str]] = set()
    outstanding_writer_results: set[str] = set()
    writer_result_summary: dict[str, object] = {
        "collected": 0,
        "invalid": 0,
        "states": Counter(),
        "statuses": Counter(),
    }
    evidence_attempts = 0
    policy = PairingPolicy(timestamp_basis="detection", revision=1)
    stop_latencies: dict[str, float] = {}
    camera_results: dict[str, dict[str, object]] = {}
    writer_metrics: dict[str, int] = {}
    pair_metrics: dict[str, int] = {}
    writer_close_ok = False
    writer_thread_stopped = False
    pairer_close_ok = False
    cleanup_errors: list[str] = []

    with tempfile.TemporaryDirectory(prefix="workstation-native-evidence-") as root:
        writer = AsyncNativeEvidenceWriter(root, max_pending=4)
        try:
            while True:
                now_s = time.monotonic()
                if now_s - started_s >= duration_s:
                    break
                if now_s >= next_rss_sample_s:
                    rss_samples.append((now_s, _rss_bytes()))
                    next_rss_sample_s = now_s + rss_sample_interval_s
                stalled = stall_start_s <= now_s < stall_end_s
                for side, state in cameras.items():
                    for name in ("control", "metadata", "preview"):
                        try:
                            depth = int(state[name].qsize())
                        except (NotImplementedError, AttributeError):
                            depth = 0
                            state["queue_depth_supported"] = False
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
                            pairer.submit(
                                side,
                                observation,
                                policy,
                                reservation_source=state["ring"],
                            )

                _collect_writer_results(
                    writer,
                    outstanding_writer_results,
                    writer_result_summary,
                )
                pair_result = pairer.poll_latest()
                pair_reservation = (
                    None if pair_result is None else pair_result.reservation
                )
                if (
                    not stalled
                    and pair_result is not None
                    and pair_result.pair is not None
                    and now_s >= next_evidence_s
                ):
                    pair = pair_result.pair
                    left_key = FrameKey.from_mapping(pair.left.timestamp_metadata)
                    right_key = FrameKey.from_mapping(pair.right.timestamp_metadata)
                    identity = (left_key.text, right_key.text)
                    if identity not in submitted_pair_keys:
                        evidence_attempts += 1
                        request_id = f"synthetic-{evidence_attempts}"
                        if not isinstance(pair_reservation, RingPairReservation):
                            raise RuntimeError(
                                "accepted qualification pair has no native reservation"
                            )
                        scheduled = writer.enqueue_reserved_pair(
                            request_id=request_id,
                            pair_id=request_id,
                            purpose="qualification",
                            reservation=pair_reservation,
                            context={"qualification": True},
                        )
                        pair_reservation = None
                        submitted_pair_keys.add(identity)
                        if scheduled.get("evidence_state") == "WRITING":
                            outstanding_writer_results.add(request_id)
                    next_evidence_s = now_s + evidence_period_s
                if pair_reservation is not None:
                    pair_reservation.release()
                time.sleep(0.02)

            for side, state in cameras.items():
                stop_started_s = time.monotonic()
                state["stop"].set()
                state["process"].join(timeout=max_stop_latency_s)
                stop_latencies[side] = time.monotonic() - stop_started_s
            writer_deadline_s = time.monotonic() + writer_shutdown_timeout_s
            while time.monotonic() < writer_deadline_s:
                _collect_writer_results(
                    writer,
                    outstanding_writer_results,
                    writer_result_summary,
                )
                if writer.pending_count == 0 and not outstanding_writer_results:
                    break
                time.sleep(0.01)
            _collect_writer_results(
                writer,
                outstanding_writer_results,
                writer_result_summary,
            )
            writer_result_summary["outstanding"] = len(outstanding_writer_results)
            writer_metrics = writer.metrics()
            writer_close_ok = bool(writer.close())
            if writer_close_ok:
                writer._thread.join(timeout=writer_shutdown_timeout_s)
            writer_thread_stopped = not writer._thread.is_alive()
            # Closing the pairer releases any latest accepted reservation that
            # the synthetic GUI did not poll before its duration deadline.
            # Snapshot ring metrics only after that ownership barrier.
            pairer_close_ok = bool(pairer.close())
            pair_metrics = pairer.metrics()
            for side, state in cameras.items():
                camera_results[side] = {
                    "exitcode": state["process"].exitcode,
                    "stop_latency_s": stop_latencies[side],
                    "forced_termination": False,
                    "queue_depth_supported": bool(state["queue_depth_supported"]),
                    "queues": state["queue_high_water"],
                    "ipc": state["counters"].snapshot(),
                    "ring": state["ring"].metrics(),
                }
        finally:
            if not writer_close_ok:
                writer_close_ok = bool(writer.close())
                if writer_close_ok:
                    writer._thread.join(timeout=writer_shutdown_timeout_s)
            writer_thread_stopped = not writer._thread.is_alive()
            if not pairer_close_ok:
                pairer_close_ok = bool(pairer.close())
            for side, state in cameras.items():
                if state["process"].is_alive():
                    state["stop"].set()
                    state["process"].terminate()
                    state["process"].join(timeout=1.0)
                    state["forced_termination"] = True
                if state["process"].is_alive():
                    state["process"].kill()
                    state["process"].join(timeout=1.0)
                    state["forced_termination"] = True
                row = camera_results.setdefault(
                    side,
                    {
                        "stop_latency_s": stop_latencies.get(side),
                        "queue_depth_supported": bool(
                            state["queue_depth_supported"]
                        ),
                        "queues": state["queue_high_water"],
                        "ipc": state["counters"].snapshot(),
                        "ring": state["ring"].metrics(),
                    },
                )
                row["exitcode"] = state["process"].exitcode
                row["forced_termination"] = bool(state["forced_termination"])
                for name in ("control", "metadata", "preview"):
                    message_queue = state[name]
                    try:
                        message_queue.cancel_join_thread()
                    except (AttributeError, OSError, ValueError):
                        pass
                    try:
                        message_queue.close()
                    except (AttributeError, OSError, ValueError) as exc:
                        cleanup_errors.append(f"{side}.{name}.close: {exc}")
                try:
                    state["ring"].close()
                    state["ring"].unlink()
                except (FileNotFoundError, OSError, RuntimeError) as exc:
                    cleanup_errors.append(f"{side}.ring.cleanup: {exc}")

    rss_samples.append((time.monotonic(), _rss_bytes()))
    rss_summary = _rss_sample_summary(rss_samples)
    active_child_pids = sorted(
        {
            int(process.pid)
            for process in multiprocessing.active_children()
            if process.pid is not None and int(process.pid) not in baseline_child_pids
        }
        | {
            int(state["process"].pid)
            for state in cameras.values()
            if state["process"].pid is not None and state["process"].is_alive()
        }
    )
    leaked_shared_memory_names = _shared_memory_leaks(shared_memory_names)
    writer_result_summary["states"] = dict(writer_result_summary.get("states") or {})
    writer_result_summary["statuses"] = dict(
        writer_result_summary.get("statuses") or {}
    )
    writer_result_summary.setdefault("outstanding", len(outstanding_writer_results))

    passed, checks, failure_reasons = _qualification_checks(
        camera_results=camera_results,
        writer_metrics=writer_metrics,
        writer_result_summary=writer_result_summary,
        minimum_saved_pairs=required_saved_pairs,
        writer_close_ok=writer_close_ok,
        writer_thread_stopped=writer_thread_stopped,
        pairer_close_ok=pairer_close_ok,
        leaked_child_pids=active_child_pids,
        leaked_shared_memory_names=leaked_shared_memory_names,
        cleanup_errors=cleanup_errors,
        rss_summary=rss_summary,
        max_rss_growth_bytes=max_rss_growth_bytes,
        max_rss_high_water_bytes=max_rss_high_water_bytes,
        max_rss_slope_bytes_per_hour=max_rss_slope_bytes_per_hour,
        max_stop_latency_s=max_stop_latency_s,
    )
    return {
        "schema": "workstation-camera-backpressure-qualification-v1",
        "qualification_contract_revision": 2,
        "passed": passed,
        "failure_reasons": failure_reasons,
        "checks": checks,
        "duration_s": duration_s,
        "gui_stall_s": gui_stall_s,
        "rss_start_bytes": rss_summary["start_bytes"],
        "rss_end_bytes": rss_summary["end_bytes"],
        "rss_high_water_bytes": rss_summary["high_water_bytes"],
        "rss_slope_bytes_per_hour": rss_summary["slope_bytes_per_hour"],
        "rss_samples": rss_summary,
        "thresholds": {
            "minimum_saved_pairs": required_saved_pairs,
            "minimum_saved_ratio": minimum_saved_ratio,
            "theoretical_saved_pairs": theoretical_saved_pairs,
            "max_rss_growth_bytes": max_rss_growth_bytes,
            "max_rss_high_water_bytes": max_rss_high_water_bytes,
            "max_rss_slope_bytes_per_hour": max_rss_slope_bytes_per_hour,
            "max_stop_latency_s": max_stop_latency_s,
            "writer_shutdown_timeout_s": writer_shutdown_timeout_s,
            "rss_sample_interval_s": rss_sample_interval_s,
        },
        "pairing": pair_metrics,
        "evidence": writer_metrics,
        "evidence_results": writer_result_summary,
        "evidence_attempts": evidence_attempts,
        "cleanup": {
            "writer_close_ok": writer_close_ok,
            "writer_thread_stopped": writer_thread_stopped,
            "pairer_close_ok": pairer_close_ok,
            "active_child_pids": active_child_pids,
            "leaked_shared_memory_names": leaked_shared_memory_names,
            "errors": cleanup_errors,
        },
        "cameras": camera_results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Qualify bounded camera IPC/evidence behavior and fail closed on "
            "resource, persistence, or cleanup regressions."
        )
    )
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--gui-stall", type=float, default=2.0)
    parser.add_argument("--evidence-period", type=float, default=2.0)
    parser.add_argument(
        "--minimum-saved-pairs",
        type=int,
        default=None,
        help=(
            "Required durable exact pairs. Default: 90%% of theoretical "
            "evidence opportunities."
        ),
    )
    parser.add_argument(
        "--minimum-saved-ratio",
        type=float,
        default=DEFAULT_MINIMUM_SAVED_RATIO,
    )
    parser.add_argument(
        "--max-rss-growth-mib",
        type=float,
        default=DEFAULT_MAX_RSS_GROWTH_BYTES / MIB,
    )
    parser.add_argument(
        "--max-rss-high-water-mib",
        type=float,
        default=DEFAULT_MAX_RSS_HIGH_WATER_BYTES / MIB,
    )
    parser.add_argument(
        "--max-rss-slope-mib-per-hour",
        type=float,
        default=DEFAULT_MAX_RSS_SLOPE_BYTES_PER_HOUR / MIB,
    )
    parser.add_argument(
        "--max-stop-latency",
        type=float,
        default=DEFAULT_MAX_STOP_LATENCY_S,
    )
    parser.add_argument(
        "--rss-sample-interval",
        type=float,
        default=DEFAULT_RSS_SAMPLE_INTERVAL_S,
    )
    parser.add_argument(
        "--writer-shutdown-timeout",
        type=float,
        default=DEFAULT_WRITER_SHUTDOWN_TIMEOUT_S,
    )
    args = parser.parse_args()
    result = run_qualification(
        duration_s=args.duration,
        gui_stall_s=args.gui_stall,
        evidence_period_s=args.evidence_period,
        minimum_saved_pairs=args.minimum_saved_pairs,
        minimum_saved_ratio=args.minimum_saved_ratio,
        max_rss_growth_bytes=int(args.max_rss_growth_mib * MIB),
        max_rss_high_water_bytes=int(args.max_rss_high_water_mib * MIB),
        max_rss_slope_bytes_per_hour=int(
            args.max_rss_slope_mib_per_hour * MIB
        ),
        max_stop_latency_s=args.max_stop_latency,
        rss_sample_interval_s=args.rss_sample_interval,
        writer_shutdown_timeout_s=args.writer_shutdown_timeout,
    )
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
