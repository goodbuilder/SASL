"""Isolated persistent workers for per-method Measurement evaluation.

The coordinator owns one process per canonical method.  A pair is committed
only after all eight workers have produced terminal outcomes; the Pixhawk/RTK
outcome is also surfaced as soon as its dedicated worker finishes so the UI
does not wait for comparison solvers.
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
    METHOD_EVALUATION_TIMEOUT_S,
    MethodEvaluationBundle,
    MethodEvaluationOutcome,
    MethodEvaluationRequest,
    failed_outcome,
)

METHOD_WORKER_START_TIMEOUT_S = 5.0
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
    backpressure_bundle: MethodEvaluationBundle | None = None


@dataclass(frozen=True)
class MethodEvaluationPoll:
    primary_outcomes: tuple[MethodEvaluationOutcome, ...] = ()
    completed_bundles: tuple[MethodEvaluationBundle, ...] = ()


@dataclass
class _PairJob:
    requests: tuple[MethodEvaluationRequest, ...]
    outcomes: dict[str, MethodEvaluationOutcome] = field(default_factory=dict)
    dispatched: set[str] = field(default_factory=set)
    primary_emitted: bool = False

    @property
    def first(self) -> MethodEvaluationRequest:
        return self.requests[0]

    def request_for(self, method_id: str) -> MethodEvaluationRequest:
        return self.requests[MEASUREMENT_METHOD_IDS.index(method_id)]


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


class _MethodEvaluationRuntime:
    """Synchronous process runtime owned only by the coordinator actor thread."""

    def __init__(
        self,
        *,
        mp_context: multiprocessing.context.BaseContext | None = None,
        process_target: Callable[..., None] = method_evaluation_worker_main,
        start_workers: bool = True,
    ) -> None:
        self._context = mp_context or multiprocessing.get_context("spawn")
        self._process_target = process_target
        self._jobs: OrderedDict[tuple[str, int], _PairJob] = OrderedDict()
        self._slots: dict[str, _WorkerSlot] = {}
        self._slot_generations = {method_id: 0 for method_id in MEASUREMENT_METHOD_IDS}
        self._primary_ready: deque[MethodEvaluationOutcome] = deque()
        self._bundles_ready: deque[MethodEvaluationBundle] = deque()
        self._closed = False
        self._started = False
        if start_workers:
            self.start()

    @property
    def pending_pair_count(self) -> int:
        return len(self._jobs)

    @property
    def worker_generations(self) -> dict[str, int]:
        return dict(self._slot_generations)

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("method evaluation coordinator is closed")
        if self._started:
            return
        self._started = True
        for method_id in MEASUREMENT_METHOD_IDS:
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
        process.start()
        self._slots[method_id] = _WorkerSlot(
            method_id=method_id,
            process=process,
            input_queue=input_queue,
            output_queue=output_queue,
            generation=generation,
        )

    @staticmethod
    def _validate_requests(
        requests: Sequence[MethodEvaluationRequest],
    ) -> tuple[MethodEvaluationRequest, ...]:
        frozen = tuple(requests)
        if tuple(request.method_id for request in frozen) != MEASUREMENT_METHOD_IDS:
            raise ValueError("submission must contain every method in canonical order")
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

    def submit(
        self,
        requests: Sequence[MethodEvaluationRequest],
    ) -> MethodEvaluationSubmission:
        if self._closed:
            raise RuntimeError("method evaluation coordinator is closed")
        frozen = self._validate_requests(requests)
        first = frozen[0]
        key = (first.pair_id, first.request_generation)
        if key in self._jobs:
            raise ValueError("duplicate method evaluation pair submission")
        if len(self._jobs) >= METHOD_EVALUATION_QUEUE_LIMIT:
            outcomes = tuple(
                failed_outcome(
                    request,
                    status="BACKPRESSURE_BLOCKED",
                    reason=(
                        f"method evaluation queue limit "
                        f"{METHOD_EVALUATION_QUEUE_LIMIT} reached"
                    ),
                )
                for request in frozen
            )
            return MethodEvaluationSubmission(
                accepted=False,
                pair_id=first.pair_id,
                backpressure_bundle=MethodEvaluationBundle(
                    measurement_id=first.measurement_id,
                    pair_id=first.pair_id,
                    pair_sequence=first.pair_sequence,
                    snapshot_sha256=first.snapshot_sha256,
                    request_generation=first.request_generation,
                    outcomes=outcomes,
                    snapshot_payload=dict(first.snapshot_payload),
                ),
            )
        self._jobs[key] = _PairJob(requests=frozen)
        if self._started:
            self._dispatch_all()
        return MethodEvaluationSubmission(accepted=True, pair_id=first.pair_id)

    def _dispatch_all(self) -> None:
        for method_id in MEASUREMENT_METHOD_IDS:
            self._dispatch_next(method_id)

    def _dispatch_next(self, method_id: str) -> None:
        slot = self._slots.get(method_id)
        if slot is None or slot.current is not None:
            return
        for job in self._jobs.values():
            if method_id in job.outcomes or method_id in job.dispatched:
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
        if outcome.method_id == METHOD_EVALUATION_PRIMARY_METHOD and not job.primary_emitted:
            job.primary_emitted = True
            self._primary_ready.append(outcome)

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
    def _terminate_slot(slot: _WorkerSlot) -> None:
        process = slot.process
        try:
            if process.is_alive():
                process.terminate()
            process.join(timeout=0.5)
            if process.is_alive() and hasattr(process, "kill"):
                process.kill()
                process.join(timeout=0.5)
        finally:
            for channel in (slot.input_queue, slot.output_queue):
                try:
                    channel.cancel_join_thread()
                    channel.close()
                except (AttributeError, OSError, ValueError):
                    pass

    def _restart_slot(self, method_id: str) -> None:
        old = self._slots.pop(method_id, None)
        if old is not None:
            self._terminate_slot(old)
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
        if request is None:
            return
        if not slot.started_monotonic_ns:
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
            return
        timeout_s = METHOD_EVALUATION_TIMEOUT_S.get(slot.method_id)
        if timeout_s is None:
            return
        if now_ns - slot.started_monotonic_ns <= int(timeout_s * 1_000_000_000):
            return
        self._record_outcome(
            failed_outcome(
                request,
                status="TIMEOUT",
                reason=f"{slot.method_id} exceeded {timeout_s:.3f}s worker timeout",
                worker_started_monotonic_ns=slot.started_monotonic_ns,
            )
        )
        self._restart_slot(slot.method_id)

    def _seal_completed_jobs(self) -> None:
        completed_keys: list[tuple[str, int]] = []
        for key, job in self._jobs.items():
            if len(job.outcomes) != len(MEASUREMENT_METHOD_IDS):
                continue
            first = job.first
            bundle = MethodEvaluationBundle(
                measurement_id=first.measurement_id,
                pair_id=first.pair_id,
                pair_sequence=first.pair_sequence,
                snapshot_sha256=first.snapshot_sha256,
                request_generation=first.request_generation,
                outcomes=tuple(job.outcomes[method_id] for method_id in MEASUREMENT_METHOD_IDS),
                snapshot_payload=dict(first.snapshot_payload),
            )
            if not job.primary_emitted:
                self._primary_ready.append(bundle.primary)
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
        primary = tuple(self._primary_ready)
        bundles = tuple(self._bundles_ready)
        self._primary_ready.clear()
        self._bundles_ready.clear()
        return MethodEvaluationPoll(
            primary_outcomes=primary,
            completed_bundles=bundles,
        )

    def close(self) -> None:
        if self._closed:
            return
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
        self._jobs.clear()
        self._primary_ready.clear()
        self._bundles_ready.clear()

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()


class MethodEvaluationCoordinator:
    """Non-blocking facade for the isolated per-method process runtime.

    Process spawn, join, timeout termination, and restart all run on one
    dedicated daemon actor thread.  ``submit()``, ``poll()``, and ``close()``
    therefore remain bounded operations suitable for a 20 ms Qt timer.
    """

    def __init__(
        self,
        *,
        mp_context: multiprocessing.context.BaseContext | None = None,
        process_target: Callable[..., None] = method_evaluation_worker_main,
        start_workers: bool = True,
    ) -> None:
        self._context = mp_context or multiprocessing.get_context("spawn")
        self._process_target = process_target
        self._start_workers = bool(start_workers)
        self._commands: queue.Queue[tuple[str, object]] = queue.Queue()
        self._results: queue.Queue[MethodEvaluationPoll] = queue.Queue()
        self._lock = threading.RLock()
        self._pending_requests: dict[
            tuple[str, int], tuple[MethodEvaluationRequest, ...]
        ] = {}
        self._worker_generations = {
            method_id: 0 for method_id in MEASUREMENT_METHOD_IDS
        }
        self._close_requested = False
        self._closed_event = threading.Event()
        self._actor_error = ""
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

    @property
    def worker_generations(self) -> dict[str, int]:
        with self._lock:
            return dict(self._worker_generations)

    @property
    def closed(self) -> bool:
        return self._closed_event.is_set()

    def start(self) -> None:
        if self._close_requested:
            raise RuntimeError("method evaluation coordinator is closed")

    def submit(
        self,
        requests: Sequence[MethodEvaluationRequest],
    ) -> MethodEvaluationSubmission:
        if self._close_requested:
            raise RuntimeError("method evaluation coordinator is closed")
        frozen = _MethodEvaluationRuntime._validate_requests(requests)
        first = frozen[0]
        key = (first.pair_id, first.request_generation)
        with self._lock:
            if key in self._pending_requests:
                raise ValueError("duplicate method evaluation pair submission")
            if len(self._pending_requests) >= METHOD_EVALUATION_QUEUE_LIMIT:
                outcomes = tuple(
                    failed_outcome(
                        request,
                        status="BACKPRESSURE_BLOCKED",
                        reason=(
                            f"method evaluation queue limit "
                            f"{METHOD_EVALUATION_QUEUE_LIMIT} reached"
                        ),
                    )
                    for request in frozen
                )
                return MethodEvaluationSubmission(
                    accepted=False,
                    pair_id=first.pair_id,
                    backpressure_bundle=MethodEvaluationBundle(
                        measurement_id=first.measurement_id,
                        pair_id=first.pair_id,
                        pair_sequence=first.pair_sequence,
                        snapshot_sha256=first.snapshot_sha256,
                        request_generation=first.request_generation,
                        outcomes=outcomes,
                        snapshot_payload=dict(first.snapshot_payload),
                    ),
                )
            self._pending_requests[key] = frozen
        self._commands.put(("submit", frozen))
        return MethodEvaluationSubmission(accepted=True, pair_id=first.pair_id)

    def poll(self) -> MethodEvaluationPoll:
        primary: list[MethodEvaluationOutcome] = []
        bundles: list[MethodEvaluationBundle] = []
        while True:
            try:
                result = self._results.get_nowait()
            except queue.Empty:
                break
            primary.extend(result.primary_outcomes)
            bundles.extend(result.completed_bundles)
        return MethodEvaluationPoll(
            primary_outcomes=tuple(primary),
            completed_bundles=tuple(bundles),
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
        runtime: _MethodEvaluationRuntime,
    ) -> None:
        with self._lock:
            self._worker_generations = runtime.worker_generations
            for bundle in polled.completed_bundles:
                self._pending_requests.pop(
                    (bundle.pair_id, bundle.request_generation),
                    None,
                )
        if polled.primary_outcomes or polled.completed_bundles:
            self._results.put(polled)

    def _publish_actor_failure(self, exc: BaseException) -> None:
        reason = f"method evaluation runtime failed: {type(exc).__name__}: {exc}"
        with self._lock:
            pending = tuple(self._pending_requests.values())
            self._pending_requests.clear()
            self._actor_error = reason
        bundles: list[MethodEvaluationBundle] = []
        for requests in pending:
            first = requests[0]
            bundles.append(
                MethodEvaluationBundle(
                    measurement_id=first.measurement_id,
                    pair_id=first.pair_id,
                    pair_sequence=first.pair_sequence,
                    snapshot_sha256=first.snapshot_sha256,
                    request_generation=first.request_generation,
                    outcomes=tuple(
                        failed_outcome(
                            request,
                            status="FAILED",
                            reason=reason,
                        )
                        for request in requests
                    ),
                    snapshot_payload=dict(first.snapshot_payload),
                )
            )
        if bundles:
            self._results.put(MethodEvaluationPoll(completed_bundles=tuple(bundles)))

    def _run_actor(self) -> None:
        runtime: _MethodEvaluationRuntime | None = None
        try:
            runtime = _MethodEvaluationRuntime(
                mp_context=self._context,
                process_target=self._process_target,
                start_workers=self._start_workers,
            )
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
                    if command == "submit":
                        submission = runtime.submit(payload)  # type: ignore[arg-type]
                        if submission.backpressure_bundle is not None:
                            bundle = submission.backpressure_bundle
                            with self._lock:
                                self._pending_requests.pop(
                                    (bundle.pair_id, bundle.request_generation),
                                    None,
                                )
                            self._results.put(
                                MethodEvaluationPoll(completed_bundles=(bundle,))
                            )
                if not closing:
                    self._publish_poll(runtime.poll(), runtime)
        except BaseException as exc:  # noqa: BLE001 - convert actor death to terminal outcomes.
            self._publish_actor_failure(exc)
        finally:
            if runtime is not None:
                try:
                    runtime.close()
                except Exception as exc:  # noqa: BLE001 - actor must always publish closed.
                    self._actor_error = f"runtime close failed: {exc}"
            with self._lock:
                self._pending_requests.clear()
            self._closed_event.set()

    def __enter__(self) -> Self:
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
]
