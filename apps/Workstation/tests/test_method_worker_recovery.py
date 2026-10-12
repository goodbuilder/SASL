"""Operator recovery must terminate only the displayed job/worker identity."""

from __future__ import annotations

import signal
import time
import queue
from types import SimpleNamespace

import pytest

from Workstation.workstation_core.method_evaluation import (
    MethodEvaluationRequest, evaluate_method_request,
)
from Workstation.workstation_core.method_evaluation_workers import MethodEvaluationCoordinator


PRIMARY = "pixhawk_rtk_live"
COMPARISON = "horizon_hybrid"


def request(method, sequence):
    return MethodEvaluationRequest(
        measurement_id="recovery-run", pair_id=f"pair-{sequence}",
        pair_sequence=sequence, snapshot_sha256="a" * 64, request_generation=1,
        method_id=method, method_instance_id=f"{method}:fixture",
        blocked_reasons=("fixture inputs unavailable",),
    )


def stalled_worker(method_id, _parent, inputs, outputs):
    while True:
        payload = inputs.get()
        if payload is None:
            return
        value = MethodEvaluationRequest.from_mapping(payload)
        started = time.monotonic_ns()
        if method_id == COMPARISON and value.pair_sequence == 1:
            # Force the bounded terminate -> kill path, not just cooperative exit.
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        outputs.put({"kind": "started", "method_id": method_id,
                     "pair_id": value.pair_id, "request_generation": value.request_generation,
                     "worker_started_monotonic_ns": started})
        if method_id == COMPARISON and value.pair_sequence == 1:
            while True:
                time.sleep(0.01)
        outcome = evaluate_method_request(value, worker_started_monotonic_ns=started)
        outputs.put({"kind": "outcome", "outcome": outcome.to_record()})


def wait_for(predicate, *, seconds=15):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    pytest.fail("worker condition was not observed before test deadline")


def worker_row(coordinator, method):
    return next(row for row in coordinator.status_snapshot()["workers"] if row["method_id"] == method)


def test_real_stalled_worker_recovery_preserves_peer_and_queued_work():
    coordinator = MethodEvaluationCoordinator(process_target=stalled_worker)
    outcomes, events = [], []

    def collect():
        polled = coordinator.poll()
        outcomes.extend(polled.outcomes)
        events.extend(polled.recovery_events)

    try:
        coordinator.submit((request(PRIMARY, 1), request(COMPARISON, 1)))
        wait_for(lambda: worker_row(coordinator, COMPARISON)["state"] == "RUNNING")
        original = worker_row(coordinator, COMPARISON)
        peer_generation = worker_row(coordinator, PRIMARY)["worker_generation"]
        assert original["elapsed_s"] >= 0
        coordinator.submit((request(PRIMARY, 2), request(COMPARISON, 2)))
        started = time.monotonic()
        accepted = coordinator.restart_method(
            COMPARISON, expected_worker_generation=original["worker_generation"],
            expected_pair_id=original["pair_id"], expected_request_generation=original["request_generation"],
        )
        assert time.monotonic() - started < 0.25
        assert accepted["accepted"]
        assert not coordinator.restart_method(
            COMPARISON, expected_worker_generation=original["worker_generation"],
            expected_pair_id=original["pair_id"], expected_request_generation=original["request_generation"],
        )["accepted"]

        def recovered():
            collect()
            return len(outcomes) == 4 and any(e["status"] == "RESTARTED" for e in events)

        wait_for(recovered)
        by_key = {(value.method_id, value.pair_sequence): value for value in outcomes}
        assert len(by_key) == 4
        interrupted = by_key[(COMPARISON, 1)]
        assert interrupted.status == "FAILED"
        assert any("operator_restart" in reason for reason in interrupted.reasons)
        assert by_key[(COMPARISON, 2)].status == "BLOCKED"
        assert by_key[(PRIMARY, 1)].status == by_key[(PRIMARY, 2)].status == "BLOCKED"
        assert worker_row(coordinator, PRIMARY)["worker_generation"] == peer_generation
        assert worker_row(coordinator, COMPARISON)["worker_generation"] == original["worker_generation"] + 1
        assert not coordinator.pending_jobs()
        assert events[0]["measurement_id"] == "recovery-run"
        assert events[0]["pair_id"] == "pair-1"
    finally:
        coordinator.close()
        assert coordinator.wait_closed(20)


def test_recovery_rejects_dormant_unknown_and_closed_workers():
    coordinator = MethodEvaluationCoordinator(start_workers=False)
    try:
        with pytest.raises(ValueError, match="method"):
            coordinator.restart_method("foreign", expected_worker_generation=1,
                                       expected_pair_id="pair-1", expected_request_generation=1)
        assert not coordinator.restart_method(PRIMARY, expected_worker_generation=1,
                                              expected_pair_id="pair-1", expected_request_generation=1)["accepted"]
    finally:
        coordinator.close()
        assert coordinator.wait_closed(10)
    assert coordinator.status_snapshot()["state"] == "CLOSED"
    assert not coordinator.restart_method(PRIMARY, expected_worker_generation=1,
                                          expected_pair_id="pair-1", expected_request_generation=1)["accepted"]


def test_actor_revalidates_identity_and_unconfirmed_exit_does_not_replace(monkeypatch):
    from Workstation.workstation_core.method_evaluation_workers import _MethodEvaluationRuntime, _WorkerSlot

    runtime = _MethodEvaluationRuntime(start_workers=False)
    value = request(COMPARISON, 1)
    runtime.submit((value,))
    slot = _WorkerSlot(COMPARISON, SimpleNamespace(), queue.Queue(), queue.Queue(), 2, current=value)
    runtime._slots[COMPARISON] = slot
    runtime._slot_generations[COMPARISON] = 2
    payload = {"method_id": COMPARISON, "worker_generation": 2, "pair_id": "wrong",
               "request_generation": 1, "measurement_id": "recovery-run", "command_id": "c-1"}
    stopped = []
    monkeypatch.setattr(runtime, "_terminate_slot", lambda slot: stopped.append(slot) or False)
    assert runtime.restart_method(payload)["status"] == "REJECTED"
    assert not stopped
    payload["pair_id"] = "pair-1"
    assert runtime.restart_method(payload)["status"] == "FAILED"
    assert runtime._slots[COMPARISON] is slot
    assert slot.current is value
    assert not runtime._outcomes_ready
    monkeypatch.setattr(runtime, "_drain_slot", lambda slot: setattr(slot, "current", None))
    assert runtime.restart_method(payload)["reason"] == "request completed before recovery"
    assert len(stopped) == 1


def test_replacement_failure_keeps_queue_and_supports_explicit_retry(monkeypatch):
    from Workstation.workstation_core.method_evaluation_workers import _MethodEvaluationRuntime, _WorkerSlot

    runtime = _MethodEvaluationRuntime(start_workers=False)
    first, second = request(COMPARISON, 1), request(COMPARISON, 2)
    runtime.submit((first,))
    runtime.submit((second,))
    slot = _WorkerSlot(COMPARISON, SimpleNamespace(), queue.Queue(), queue.Queue(), 2, current=first)
    runtime._slots[COMPARISON] = slot
    runtime._slot_generations[COMPARISON] = 2
    runtime._jobs[(first.pair_id, first.request_generation)].dispatched.add(COMPARISON)
    monkeypatch.setattr(runtime, "_terminate_slot", lambda slot: True)

    def fail_start(method):
        runtime._slot_generations[method] += 1
        raise OSError("injected spawn failure")

    monkeypatch.setattr(runtime, "_start_slot", fail_start)
    result = runtime.restart_method({"method_id": COMPARISON, "worker_generation": 2,
        "pair_id": "pair-1", "request_generation": 1, "measurement_id": "recovery-run"})
    assert result["status"] == "FAILED"
    assert runtime.pending_method_count(COMPARISON) == 1
    assert next(row for row in runtime.status_rows() if row["method_id"] == COMPARISON)["state"] == "UNAVAILABLE"

    def restored_start(method):
        runtime._slot_generations[method] += 1
        runtime._slots[method] = _WorkerSlot(method, SimpleNamespace(), queue.Queue(), queue.Queue(), 4)
        runtime._unavailable_errors.pop(method, None)

    monkeypatch.setattr(runtime, "_start_slot", restored_start)
    result = runtime.restart_method({"method_id": COMPARISON, "worker_generation": 3,
        "pair_id": "", "request_generation": None, "measurement_id": ""})
    assert result["status"] == "RESTARTED"
    assert runtime._slots[COMPARISON].input_queue.get_nowait()["pair_id"] == "pair-2"
    assert runtime._outcomes_ready[0].pair_id == "pair-1"
    assert runtime._outcomes_ready[0].status == "FAILED"
