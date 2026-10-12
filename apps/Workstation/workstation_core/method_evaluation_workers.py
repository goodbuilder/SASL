"""Isolated persistent workers for per-method Measurement evaluation.

The coordinator owns one process per canonical method for the lifetime of
the application.  A pair submits only the methods that are ready for it;
every outcome is published as soon as its worker returns, and a pair is
"complete" when all *submitted* methods are terminal.  There is no compute
deadline: a worker is only replaced when its process exits.
"""

from __future__ import annotations

import ctypes
import multiprocessing
import os
import queue
import signal
import sys
import threading
import time
from collections import OrderedDict, deque
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field

try:
    from typing import Self
except ImportError:  # Python 3.10 runtime supported by Workstation.
    from typing_extensions import Self

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.method_evaluation import (
    METHOD_EVALUATION_PRIMARY_METHOD,
    METHOD_EVALUATION_QUEUE_LIMIT,
    MethodEvaluationBundle,
    MethodEvaluationOutcome,
    MethodEvaluationRequest,
    canonical_method_order,
    failed_outcome,
)

# A spawned child that never acknowledges its first request is a broken
# process, not a slow solver; this is the only clock-based guard left.
METHOD_WORKER_START_TIMEOUT_S = 30.0
METHOD_WORKER_THREAD_ENV: Mapping[str, str] = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
    "VECLIB_MAXIMUM_THREADS": "1",
}
_LINUX_PR_SET_PDEATHSIG = 1


def _arm_linux_parent_death_signal(expected_parent_pid: int) -> bool:
    """Ask Linux to terminate the child if its creating process disappears."""

    if not sys.platform.startswith("linux"):
        return False
    if os.getppid() != int(expected_parent_pid):
        os._exit(128 + int(signal.SIGTERM))
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        prctl = libc.prctl
        prctl.argtypes = (
            ctypes.c_int,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_ulong,
        )
        prctl.restype = ctypes.c_int
        if prctl(_LINUX_PR_SET_PDEATHSIG, int(signal.SIGTERM), 0, 0, 0) != 0:
            return False
    except (AttributeError, OSError):
        return False
    if os.getppid() != int(expected_parent_pid):
        os._exit(128 + int(signal.SIGTERM))
    return True


def _configure_child_threads() -> None:
    for name, value in METHOD_WORKER_THREAD_ENV.items():
        os.environ[name] = value
    try:
        import cv2

        cv2.setNumThreads(1)
    except (ImportError, RuntimeError):
        pass


def _request_identity(request: MethodEvaluationRequest) -> dict[str, object]:
    return {
        "measurement_id": request.measurement_id,
        "pair_id": request.pair_id,
        "pair_sequence": request.pair_sequence,
        "snapshot_sha256": request.snapshot_sha256,
        "request_generation": request.request_generation,
        "method_id": request.method_id,
    }


def method_evaluation_worker_main(
    method_id: str,
    expected_parent_pid: int,
    input_queue: object,
    output_queue: object,
) -> None:
    """Persistent child entrypoint; arguments remain spawn-pickle-safe."""

    _configure_child_threads()
    _arm_linux_parent_death_signal(expected_parent_pid)
    from Workstation.workstation_core.method_evaluation import evaluate_method_request

    while os.getppid() == int(expected_parent_pid):
        try:
            payload = input_queue.get(timeout=0.25)
        except queue.Empty:
            continue
        if payload is None:
            return
        started_ns = time.monotonic_ns()
        request: MethodEvaluationRequest | None = None
        try:
            if not isinstance(payload, Mapping):
                raise TypeError("worker request must be a mapping")
            request = MethodEvaluationRequest.from_mapping(payload)
            if request.method_id != method_id:
                raise ValueError("request routed to the wrong method worker")
            output_queue.put(
                {
                    "kind": "started",
                    **_request_identity(request),
                    "worker_started_monotonic_ns": started_ns,
                }
            )
            outcome = evaluate_method_request(
                request,
                worker_started_monotonic_ns=started_ns,
            )
            output_queue.put({"kind": "outcome", "outcome": outcome.to_record()})
        except Exception as exc:  # noqa: BLE001 - solver failures become terminal evidence.
            identity = (
                {
                    "method_id": method_id,
                    "request_generation": -1,
                    "pair_id": "",
                }
                if request is None
                else _request_identity(request)
            )
            output_queue.put(
                {
                    "kind": "failure",
                    **identity,
                    "worker_started_monotonic_ns": started_ns,
                    "reason": f"{type(exc).__name__}: {exc}",
                }
            )


@dataclass(frozen=True)
class MethodEvaluationSubmission:
    accepted: bool
    pair_id: str
    submitted_method_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class MethodEvaluationPoll:
    """Outcomes published since the previous poll, plus fully terminal pairs."""

    outcomes: tuple[MethodEvaluationOutcome, ...] = ()
    completed_bundles: tuple[MethodEvaluationBundle, ...] = ()
    recovery_events: tuple[Mapping[str, object], ...] = ()

    @property
    def primary_outcomes(self) -> tuple[MethodEvaluationOutcome, ...]:
        return tuple(
            outcome
            for outcome in self.outcomes
            if outcome.method_id == METHOD_EVALUATION_PRIMARY_METHOD
        )


@dataclass
class _PairJob:
    requests: dict[str, MethodEvaluationRequest]
    planned_methods: dict[str, str] = field(default_factory=dict)
    outcomes: dict[str, MethodEvaluationOutcome] = field(default_factory=dict)
    dispatched: set[str] = field(default_factory=set)

    @property
    def first(self) -> MethodEvaluationRequest:
        return next(iter(self.requests.values()))

    @property
    def complete(self) -> bool:
        return set(self.outcomes) == set(self.requests)

    def request_for(self, method_id: str) -> MethodEvaluationRequest:
        return self.requests[method_id]


@dataclass
class _WorkerSlot:
    method_id: str
    process: object
    input_queue: object
    output_queue: object
    generation: int
    current: MethodEvaluationRequest | None = None
    dispatched_monotonic_ns: int = 0
    started_monotonic_ns: int = 0


def validate_submission(
    requests: Sequence[MethodEvaluationRequest],
) -> tuple[MethodEvaluationRequest, ...]:
    """Validate one pair's request subset and return it in canonical order."""

    frozen = tuple(requests)
    canonical_method_order(request.method_id for request in frozen)
    first = frozen[0]
    for request in frozen:
        if (
            request.measurement_id != first.measurement_id
            or request.pair_id != first.pair_id
            or request.pair_sequence != first.pair_sequence
            or request.snapshot_sha256 != first.snapshot_sha256
            or request.request_generation != first.request_generation
        ):
            raise ValueError("submission request identity mismatch")
    return frozen


class _MethodEvaluationRuntime:
    """Synchronous process runtime owned only by the coordinator actor thread."""

    def __init__(
        self,
        *,
        mp_context: multiprocessing.context.BaseContext | None = None,
        process_target: Callable[..., None] = method_evaluation_worker_main,
        start_workers: bool = True,
        queue_limit: int = METHOD_EVALUATION_QUEUE_LIMIT,
    ) -> None:
        self._context = mp_context or multiprocessing.get_context("spawn")
        self._process_target = process_target
        self._queue_limit = max(1, int(queue_limit))
        self._jobs: OrderedDict[tuple[str, int], _PairJob] = OrderedDict()
        self._slots: dict[str, _WorkerSlot] = {}
        self._slot_generations = {method_id: 0 for method_id in MEASUREMENT_METHOD_IDS}
        self._outcomes_ready: deque[MethodEvaluationOutcome] = deque()
        self._bundles_ready: deque[MethodEvaluationBundle] = deque()
        self._closed = False
        self._started = False
        self._comparison_workers_started = False
        self._unavailable_errors: dict[str, str] = {}
        if start_workers:
            self.start()

    @property
    def pending_pair_count(self) -> int:
        return len(self._jobs)

    def pending_method_count(self, method_id: str) -> int:
        return sum(
            1
            for job in self._jobs.values()
            if method_id in job.requests and method_id not in job.outcomes
        )

    @property
    def worker_generations(self) -> dict[str, int]:
        return dict(self._slot_generations)

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("method evaluation coordinator is closed")
        if self._started:
            return
        self._started = True
        self._start_slot(METHOD_EVALUATION_PRIMARY_METHOD)
        self._dispatch_all()

    @property
    def comparison_workers_started(self) -> bool:
        return self._comparison_workers_started

    def start_comparison_workers(self) -> None:
        if self._closed or self._comparison_workers_started:
            return
        self._comparison_workers_started = True
        for method_id in MEASUREMENT_METHOD_IDS:
            if method_id == METHOD_EVALUATION_PRIMARY_METHOD:
                continue
            self._start_slot(method_id)
        self._dispatch_all()

    def _start_slot(self, method_id: str) -> None:
        generation = self._slot_generations[method_id] + 1
        self._slot_generations[method_id] = generation
        input_queue = self._context.Queue(maxsize=1)
        output_queue = self._context.Queue()
        process = self._context.Process(
            name=f"measurement-{method_id}",
            target=self._process_target,
            args=(method_id, os.getpid(), input_queue, output_queue),
            daemon=True,
        )
        try:
            process.start()
        except Exception:
            for channel in (input_queue, output_queue):
                channel.cancel_join_thread()
                channel.close()
            raise
        self._slots[method_id] = _WorkerSlot(
            method_id=method_id,
            process=process,
            input_queue=input_queue,
            output_queue=output_queue,
            generation=generation,
        )
        self._unavailable_errors.pop(method_id, None)

    def status_rows(self) -> tuple[dict[str, object], ...]:
        pending = {method: 0 for method in MEASUREMENT_METHOD_IDS}
        for job in self._jobs.values():
            for method in job.requests.keys() - job.outcomes.keys():
                pending[method] += 1
        rows = []
        for method in MEASUREMENT_METHOD_IDS:
            slot = self._slots.get(method)
            request = None if slot is None else slot.current
            state = (
                "UNAVAILABLE" if method in self._unavailable_errors else
                "DORMANT" if slot is None else
                "RUNNING" if slot.started_monotonic_ns else
                "DISPATCHED" if request is not None else "IDLE"
            )
            rows.append({
                "method_id": method, "state": state,
                "worker_generation": self._slot_generations[method],
                "measurement_id": "" if request is None else request.measurement_id,
                "pair_id": "" if request is None else request.pair_id,
                "request_generation": None if request is None else request.request_generation,
                "worker_started_monotonic_ns": 0 if slot is None else slot.started_monotonic_ns,
                "dispatched_monotonic_ns": 0 if slot is None else slot.dispatched_monotonic_ns,
                "pending_jobs": pending[method],
                "error": self._unavailable_errors.get(method, ""),
            })
        return tuple(rows)

    def restart_method(self, payload: Mapping[str, object]) -> dict[str, object]:
        """Actor-only explicit recovery, guarded by the identity shown in UI."""
        method = str(payload["method_id"])
        result = {
            "event_type": "method_worker_restart_result", **dict(payload),
            "status": "REJECTED", "reason": "worker or request identity changed",
            "recovery_started_monotonic_ns": time.monotonic_ns(),
        }
        slot = self._slots.get(method)
        if self._closed or self._slot_generations[method] != payload["worker_generation"]:
            return result
        request = None if slot is None else slot.current
        if slot is None:
            if method not in self._unavailable_errors or payload["pair_id"]:
                return result
        elif request is None or (request.pair_id, request.request_generation) != (
            payload["pair_id"], payload["request_generation"],
        ):
            return result
        # Drain an already completed response before deciding to interrupt it.
        if slot is not None:
            self._drain_slot(slot)
            if slot.current is not request:
                result["reason"] = "request completed before recovery"
                return result
            try:
                if not self._terminate_slot(slot):
                    result.update(status="FAILED", reason="worker termination was not confirmed")
                    return result
            except Exception as exc:
                result.update(status="FAILED", reason=f"worker termination failed: {exc}")
                return result
            self._finish_slot_with(slot, failed_outcome(
                request, status="FAILED", reason="operator_restart: interrupted selected method worker",
                worker_started_monotonic_ns=slot.started_monotonic_ns,
            ))
            self._slots.pop(method, None)
        try:
            self._start_slot(method)
        except Exception as exc:
            reason = f"worker replacement failed: {type(exc).__name__}: {exc}"
            self._unavailable_errors[method] = reason
            result.update(status="FAILED", reason=reason)
        else:
            result.update(status="RESTARTED", reason="selected worker replaced; queued work retained")
        result["replacement_generation"] = self._slot_generations[method]
        self._seal_completed_jobs()
        self._dispatch_next(method)
        return result

    def submit(
        self,
        requests: Sequence[MethodEvaluationRequest],
        *,
        planned_methods: Mapping[str, str] | None = None,
    ) -> MethodEvaluationSubmission:
        if self._closed:
            raise RuntimeError("method evaluation coordinator is closed")
        frozen = validate_submission(requests)
        first = frozen[0]
        key = (first.pair_id, first.request_generation)
        if key in self._jobs:
            raise ValueError("duplicate method evaluation pair submission")
        job = _PairJob(
            requests={request.method_id: request for request in frozen},
            planned_methods=dict(planned_methods or {}),
        )
        for request in frozen:
            if self.pending_method_count(request.method_id) >= self._queue_limit:
                outcome = failed_outcome(
                    request,
                    status="BACKPRESSURE_BLOCKED",
                    reason=(
                        f"{request.method_id} pending queue limit "
                        f"{self._queue_limit} reached"
                    ),
                )
                job.outcomes[request.method_id] = outcome
                self._outcomes_ready.append(outcome)
        self._jobs[key] = job
        if self._started:
            self._dispatch_all()
        self._seal_completed_jobs()
        return MethodEvaluationSubmission(
            accepted=True,
            pair_id=first.pair_id,
            submitted_method_ids=tuple(request.method_id for request in frozen),
        )

    def _dispatch_all(self) -> None:
        for method_id in MEASUREMENT_METHOD_IDS:
            self._dispatch_next(method_id)

    def _dispatch_next(self, method_id: str) -> None:
        slot = self._slots.get(method_id)
        if slot is None or slot.current is not None:
            return
        for job in self._jobs.values():
            if (
                method_id not in job.requests
                or method_id in job.outcomes
                or method_id in job.dispatched
            ):
                continue
            request = job.request_for(method_id)
            try:
                slot.input_queue.put_nowait(request.to_mapping())
            except queue.Full:
                return
            job.dispatched.add(method_id)
            slot.current = request
            slot.dispatched_monotonic_ns = time.monotonic_ns()
            slot.started_monotonic_ns = 0
            return

    @staticmethod
    def _matches_current(message: Mapping[str, object], slot: _WorkerSlot) -> bool:
        request = slot.current
        return bool(
            request is not None
            and str(message.get("method_id") or "") == request.method_id
            and str(message.get("pair_id") or "") == request.pair_id
            and int(message.get("request_generation", -1)) == request.request_generation
        )

    def _record_outcome(self, outcome: MethodEvaluationOutcome) -> None:
        key = (outcome.pair_id, outcome.request_generation)
        job = self._jobs.get(key)
        if job is None or outcome.method_id in job.outcomes:
            return
        if outcome.method_id not in job.requests:
            return
        request = job.request_for(outcome.method_id)
        if (
            outcome.measurement_id != request.measurement_id
            or outcome.pair_sequence != request.pair_sequence
            or outcome.snapshot_sha256 != request.snapshot_sha256
            or outcome.method_instance_id != request.method_instance_id
        ):
            outcome = failed_outcome(
                request,
                status="FAILED",
                reason="worker outcome identity mismatch",
                worker_started_monotonic_ns=outcome.worker_started_monotonic_ns,
            )
        job.outcomes[outcome.method_id] = outcome
        self._outcomes_ready.append(outcome)

    def _finish_slot_with(self, slot: _WorkerSlot, outcome: MethodEvaluationOutcome) -> None:
        self._record_outcome(outcome)
        slot.current = None
        slot.dispatched_monotonic_ns = 0
        slot.started_monotonic_ns = 0

    def _drain_slot(self, slot: _WorkerSlot) -> None:
        while True:
            try:
                message = slot.output_queue.get_nowait()
            except queue.Empty:
                return
            if not isinstance(message, Mapping):
                continue
            kind = str(message.get("kind") or "")
            raw_outcome = message.get("outcome")
            identity = (
                raw_outcome
                if kind == "outcome" and isinstance(raw_outcome, Mapping)
                else message
            )
            if not self._matches_current(identity, slot):
                continue
            if kind == "started":
                slot.started_monotonic_ns = int(
                    message.get("worker_started_monotonic_ns") or 0
                )
                continue
            request = slot.current
            if request is None:
                continue
            if kind == "outcome":
                try:
                    if not isinstance(raw_outcome, Mapping):
                        raise TypeError("worker outcome is not a mapping")
                    outcome = MethodEvaluationOutcome.from_mapping(raw_outcome)
                except Exception as exc:  # noqa: BLE001 - reject malformed child output.
                    outcome = failed_outcome(
                        request,
                        status="FAILED",
                        reason=f"invalid worker outcome: {exc}",
                        worker_started_monotonic_ns=slot.started_monotonic_ns,
                    )
                self._finish_slot_with(slot, outcome)
                return
            if kind == "failure":
                self._finish_slot_with(
                    slot,
                    failed_outcome(
                        request,
                        status="FAILED",
                        reason=str(message.get("reason") or "method evaluator failed"),
                        worker_started_monotonic_ns=int(
                            message.get("worker_started_monotonic_ns") or 0
                        ),
                    ),
                )
                return

    @staticmethod
    def _terminate_slot(slot: _WorkerSlot) -> bool:
        process = slot.process
        if process.is_alive():
            process.terminate()
        process.join(timeout=0.5)
        if process.is_alive() and hasattr(process, "kill"):
            process.kill()
            process.join(timeout=0.5)
        if process.is_alive():
            return False
        for channel in (slot.input_queue, slot.output_queue):
            try:
                channel.cancel_join_thread()
                channel.close()
            except (AttributeError, OSError, ValueError):
                pass
        return True

    def _restart_slot(self, method_id: str) -> None:
        old = self._slots.get(method_id)
        if old is not None:
            if not self._terminate_slot(old):
                return
            self._slots.pop(method_id, None)
        if not self._closed and self._started:
            self._start_slot(method_id)

    def _check_slot_health(self, slot: _WorkerSlot, now_ns: int) -> None:
        request = slot.current
        if not slot.process.is_alive():
            if request is not None:
                self._record_outcome(
                    failed_outcome(
                        request,
                        status="FAILED",
                        reason=f"{slot.method_id} worker exited unexpectedly",
                        worker_started_monotonic_ns=slot.started_monotonic_ns,
                    )
                )
            self._restart_slot(slot.method_id)
            return
        if request is None or slot.started_monotonic_ns:
            return
        if now_ns - slot.dispatched_monotonic_ns <= int(
            METHOD_WORKER_START_TIMEOUT_S * 1_000_000_000
        ):
            return
        self._record_outcome(
            failed_outcome(
                request,
                status="FAILED",
                reason=f"{slot.method_id} worker did not acknowledge start",
            )
        )
        self._restart_slot(slot.method_id)

    def _seal_completed_jobs(self) -> None:
        completed_keys: list[tuple[str, int]] = []
        for key, job in self._jobs.items():
            if not job.complete:
                continue
            first = job.first
            bundle = MethodEvaluationBundle(
                measurement_id=first.measurement_id,
                pair_id=first.pair_id,
                pair_sequence=first.pair_sequence,
                snapshot_sha256=first.snapshot_sha256,
                request_generation=first.request_generation,
                outcomes=tuple(
                    job.outcomes[method_id]
                    for method_id in MEASUREMENT_METHOD_IDS
                    if method_id in job.outcomes
                ),
                snapshot_payload=dict(first.snapshot_payload),
                planned_methods=dict(job.planned_methods),
            )
            self._bundles_ready.append(bundle)
            completed_keys.append(key)
        for key in completed_keys:
            self._jobs.pop(key, None)

    def poll(self) -> MethodEvaluationPoll:
        if self._closed:
            return MethodEvaluationPoll()
        for slot in tuple(self._slots.values()):
            self._drain_slot(slot)
        now_ns = time.monotonic_ns()
        for slot in tuple(self._slots.values()):
            if self._slots.get(slot.method_id) is slot:
                self._check_slot_health(slot, now_ns)
        self._seal_completed_jobs()
        self._dispatch_all()
        outcomes = tuple(self._outcomes_ready)
        bundles = tuple(self._bundles_ready)
        self._outcomes_ready.clear()
        self._bundles_ready.clear()
        return MethodEvaluationPoll(outcomes=outcomes, completed_bundles=bundles)

    def close(self) -> MethodEvaluationPoll:
        """Stop every worker; unfinished requests become FAILED evidence."""

        if self._closed:
            return MethodEvaluationPoll()
        self._closed = True
        for slot in tuple(self._slots.values()):
            try:
                if slot.process.is_alive():
                    slot.input_queue.put_nowait(None)
            except (queue.Full, OSError, ValueError):
                pass
        for slot in tuple(self._slots.values()):
            self._terminate_slot(slot)
        self._slots.clear()
        for job in self._jobs.values():
            for method_id, request in job.requests.items():
                if method_id in job.outcomes:
                    continue
                self._record_outcome(
                    failed_outcome(
                        request,
                        status="FAILED",
                        reason="method worker closed before completion",
                    )
                )
        self._seal_completed_jobs()
        self._jobs.clear()
        outcomes = tuple(self._outcomes_ready)
        bundles = tuple(self._bundles_ready)
        self._outcomes_ready.clear()
        self._bundles_ready.clear()
        return MethodEvaluationPoll(outcomes=outcomes, completed_bundles=bundles)

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()


class MethodEvaluationCoordinator:
    """Non-blocking facade for the isolated per-method process runtime.

    Process spawn, join, and restart all run on one dedicated daemon actor
    thread.  ``submit()``, ``poll()``, and ``close()`` therefore remain
    bounded operations suitable for a 20 ms Qt timer.  The coordinator is
    intended to live for the whole application; Measurement Start/Stop never
    creates or destroys it.
    """

    def __init__(
        self,
        *,
        mp_context: multiprocessing.context.BaseContext | None = None,
        process_target: Callable[..., None] = method_evaluation_worker_main,
        start_workers: bool = True,
        queue_limit: int = METHOD_EVALUATION_QUEUE_LIMIT,
    ) -> None:
        self._context = mp_context or multiprocessing.get_context("spawn")
        self._process_target = process_target
        self._start_workers = bool(start_workers)
        self._queue_limit = max(1, int(queue_limit))
        self._commands: queue.Queue[tuple[str, object]] = queue.Queue()
        self._results: queue.Queue[MethodEvaluationPoll] = queue.Queue()
        self._lock = threading.RLock()
        self._pending_requests: dict[
            tuple[str, int], tuple[MethodEvaluationRequest, ...]
        ] = {}
        self._pending_jobs: dict[tuple[str, int], set[str]] = {}
        self._worker_generations = {
            method_id: 0 for method_id in MEASUREMENT_METHOD_IDS
        }
        self._start_requested = self._start_workers
        self._close_requested = False
        self._closed_event = threading.Event()
        self._actor_error = ""
        self._status_rows: tuple[dict[str, object], ...] = ()
        self._status_published_ns = 0
        self._restart_sequence = 0
        self._restart_pending: dict[str, str] = {}
        self._restart_results: dict[str, dict[str, object]] = {}
        self._actor = threading.Thread(
            target=self._run_actor,
            name="measurement-method-runtime",
            daemon=True,
        )
        self._actor.start()

    @property
    def pending_pair_count(self) -> int:
        with self._lock:
            return len(self._pending_requests)

    def pending_jobs(self) -> dict[tuple[str, int], tuple[str, ...]]:
        """(pair_id, request_generation) -> method ids without a terminal outcome."""

        with self._lock:
            return {
                key: tuple(sorted(methods))
                for key, methods in self._pending_jobs.items()
                if methods
            }

    def pending_job_count_for_measurement(self, measurement_id: str) -> int:
        with self._lock:
            return sum(
                len(methods)
                for key, methods in self._pending_jobs.items()
                if methods and self._pending_requests[key][0].measurement_id == measurement_id
            )

    @property
    def worker_generations(self) -> dict[str, int]:
        with self._lock:
            return dict(self._worker_generations)

    def status_snapshot(self) -> dict[str, object]:
        """Pure cached read: never query/join a child on the caller's thread."""
        now = time.monotonic_ns()
        with self._lock:
            rows = {str(row["method_id"]): dict(row) for row in self._status_rows}
            for method in MEASUREMENT_METHOD_IDS:
                row = rows.setdefault(method, {
                    "method_id": method, "state": "DORMANT", "worker_generation": 0,
                    "measurement_id": "", "pair_id": "", "request_generation": None,
                    "pending_jobs": 0, "worker_started_monotonic_ns": 0,
                    "dispatched_monotonic_ns": 0, "error": "",
                })
                start = int(row["worker_started_monotonic_ns"] or row["dispatched_monotonic_ns"] or 0)
                row["elapsed_s"] = max(0.0, (now - start) / 1e9) if start else 0.0
                row["restart_pending"] = method in self._restart_pending
                row["last_restart"] = dict(self._restart_results.get(method, {}))
            state = ("CLOSED" if self.closed else "CLOSING" if self._close_requested else
                     "FAILED" if self._actor_error else "OPEN")
            return {
                "schema": "workstation-method-worker-status-v1", "state": state,
                "error": self._actor_error, "published_monotonic_ns": self._status_published_ns,
                "workers": list(rows.values()),
            }

    def restart_method(self, method_id: str, *, expected_worker_generation: int,
                       expected_pair_id: str, expected_request_generation: int | None) -> dict[str, object]:
        if method_id not in MEASUREMENT_METHOD_IDS:
            raise ValueError(f"unsupported measurement method: {method_id}")
        with self._lock:
            if self._close_requested or self.closed:
                return {"accepted": False, "reason": "method runtime is closing or closed"}
            if method_id in self._restart_pending:
                return {"accepted": False, "reason": "recovery already pending"}
            row = next((row for row in self._status_rows if row["method_id"] == method_id), None)
            if row is None or row["state"] not in {"DISPATCHED", "RUNNING", "UNAVAILABLE"} or (
                row["worker_generation"], row["pair_id"], row["request_generation"]
            ) != (expected_worker_generation, expected_pair_id, expected_request_generation):
                return {"accepted": False, "reason": "worker or request identity changed"}
            self._restart_sequence += 1
            command_id = f"method-restart-{self._restart_sequence}"
            payload = {
                "command_id": command_id, "method_id": method_id,
                "worker_generation": expected_worker_generation,
                "pair_id": expected_pair_id, "request_generation": expected_request_generation,
                "measurement_id": row["measurement_id"],
            }
            self._restart_pending[method_id] = command_id
            self._commands.put(("restart", payload))
            return {"accepted": True, "command_id": command_id, "reason": "recovery queued"}

    @property
    def closed(self) -> bool:
        return self._closed_event.is_set()

    def start(self) -> None:
        with self._lock:
            if self._close_requested or self._closed_event.is_set():
                raise RuntimeError("method evaluation coordinator is closed")
            if self._start_requested:
                return
            self._start_requested = True
            self._commands.put(("start", None))

    def submit(
        self,
        requests: Sequence[MethodEvaluationRequest],
        *,
        planned_methods: Mapping[str, str] | None = None,
    ) -> MethodEvaluationSubmission:
        if self._close_requested or self._closed_event.is_set():
            reason = self._actor_error or "method evaluation coordinator is closed"
            raise RuntimeError(reason)
        frozen = validate_submission(requests)
        first = frozen[0]
        key = (first.pair_id, first.request_generation)
        with self._lock:
            if key in self._pending_requests:
                raise ValueError("duplicate method evaluation pair submission")
            self._pending_requests[key] = frozen
            self._pending_jobs[key] = {request.method_id for request in frozen}
        self._commands.put(("submit", (frozen, dict(planned_methods or {}))))
        return MethodEvaluationSubmission(
            accepted=True,
            pair_id=first.pair_id,
            submitted_method_ids=tuple(request.method_id for request in frozen),
        )

    def poll(self) -> MethodEvaluationPoll:
        outcomes: list[MethodEvaluationOutcome] = []
        bundles: list[MethodEvaluationBundle] = []
        recovery_events: list[Mapping[str, object]] = []
        while True:
            try:
                result = self._results.get_nowait()
            except queue.Empty:
                break
            outcomes.extend(result.outcomes)
            bundles.extend(result.completed_bundles)
            recovery_events.extend(result.recovery_events)
        return MethodEvaluationPoll(
            outcomes=tuple(outcomes),
            completed_bundles=tuple(bundles),
            recovery_events=tuple(recovery_events),
        )

    def close(self) -> bool:
        with self._lock:
            if not self._close_requested:
                self._close_requested = True
                self._commands.put(("close", None))
        return self._closed_event.is_set()

    def wait_closed(self, timeout_s: float = 5.0) -> bool:
        return self._closed_event.wait(max(0.0, float(timeout_s)))

    def _publish_poll(
        self,
        polled: MethodEvaluationPoll,
        runtime: _MethodEvaluationRuntime | None,
    ) -> None:
        with self._lock:
            if runtime is not None:
                self._worker_generations = runtime.worker_generations
                now = time.monotonic_ns()
                if now - self._status_published_ns >= 100_000_000 or polled.outcomes:
                    self._status_rows = runtime.status_rows()
                    self._status_published_ns = now
            for outcome in polled.outcomes:
                pending = self._pending_jobs.get((outcome.pair_id, outcome.request_generation))
                if pending is not None:
                    pending.discard(outcome.method_id)
            for bundle in polled.completed_bundles:
                key = (bundle.pair_id, bundle.request_generation)
                self._pending_jobs.pop(key, None)
                self._pending_requests.pop(key, None)
        if polled.outcomes or polled.completed_bundles:
            self._results.put(polled)

    def _publish_actor_failure(self, exc: BaseException) -> None:
        reason = f"method evaluation runtime failed: {type(exc).__name__}: {exc}"
        with self._lock:
            pending = tuple(self._pending_requests.values())
            pending_jobs = dict(self._pending_jobs)
            self._pending_requests.clear()
            self._pending_jobs.clear()
            self._actor_error = reason
        outcomes: list[MethodEvaluationOutcome] = []
        bundles: list[MethodEvaluationBundle] = []
        for requests in pending:
            first = requests[0]
            remaining = pending_jobs.get((first.pair_id, first.request_generation), set())
            failed = tuple(
                failed_outcome(request, status="FAILED", reason=reason)
                for request in requests
                if request.method_id in remaining
            )
            outcomes.extend(failed)
            # Completed methods of this pair were already published; the
            # bundle is reconstructed from failures only when everything
            # submitted is still pending (otherwise the pair stays partial).
            if len(failed) == len(requests):
                bundles.append(
                    MethodEvaluationBundle(
                        measurement_id=first.measurement_id,
                        pair_id=first.pair_id,
                        pair_sequence=first.pair_sequence,
                        snapshot_sha256=first.snapshot_sha256,
                        request_generation=first.request_generation,
                        outcomes=failed,
                        snapshot_payload=dict(first.snapshot_payload),
                    )
                )
        if outcomes or bundles:
            self._results.put(
                MethodEvaluationPoll(
                    outcomes=tuple(outcomes),
                    completed_bundles=tuple(bundles),
                )
            )

    def _run_actor(self) -> None:
        runtime: _MethodEvaluationRuntime | None = None
        actor_failed = False
        try:
            runtime = _MethodEvaluationRuntime(
                mp_context=self._context,
                process_target=self._process_target,
                start_workers=False,
                queue_limit=self._queue_limit,
            )
            if self._start_workers:
                runtime.start()
            with self._lock:
                self._worker_generations = runtime.worker_generations
            closing = False
            while not closing:
                try:
                    command, payload = self._commands.get(timeout=0.005)
                except queue.Empty:
                    command = ""
                    payload = None
                commands = [] if not command else [(command, payload)]
                while True:
                    try:
                        commands.append(self._commands.get_nowait())
                    except queue.Empty:
                        break
                for command, payload in commands:
                    if command == "close":
                        closing = True
                        break
                    if command == "start":
                        runtime.start()
                        continue
                    if command == "restart":
                        if self._close_requested:
                            result = {**payload, "event_type": "method_worker_restart_result",
                                      "status": "REJECTED", "reason": "runtime closing"}
                        else:
                            result = runtime.restart_method(payload)
                        with self._lock:
                            method = str(payload["method_id"])
                            self._restart_results[method] = dict(result)
                            self._restart_pending.pop(method, None)
                            self._status_published_ns = 0
                        self._results.put(MethodEvaluationPoll(recovery_events=(result,)))
                        continue
                    if command == "submit":
                        requests, planned_methods = payload  # type: ignore[misc]
                        runtime.submit(requests, planned_methods=planned_methods)
                if not closing:
                    polled = runtime.poll()
                    self._publish_poll(polled, runtime)
                    if (
                        polled.primary_outcomes
                        and not runtime.comparison_workers_started
                    ):
                        runtime.start_comparison_workers()
        except BaseException as exc:  # noqa: BLE001 - convert actor death to terminal outcomes.
            actor_failed = True
            self._publish_actor_failure(exc)
        finally:
            if runtime is not None:
                try:
                    closed = runtime.close()
                    if not actor_failed:
                        self._publish_poll(closed, None)
                except Exception as exc:  # noqa: BLE001 - actor must always publish closed.
                    self._actor_error = f"runtime close failed: {exc}"
            with self._lock:
                self._pending_requests.clear()
                self._pending_jobs.clear()
            self._closed_event.set()

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
        self.wait_closed()


__all__ = [
    "METHOD_WORKER_START_TIMEOUT_S",
    "METHOD_WORKER_THREAD_ENV",
    "MethodEvaluationCoordinator",
    "MethodEvaluationPoll",
    "MethodEvaluationSubmission",
    "method_evaluation_worker_main",
    "validate_submission",
]
