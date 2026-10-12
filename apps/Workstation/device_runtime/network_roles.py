"""One owner for NIC role transactions and their Qt worker lifetime.

The core network transaction owns OS mutation and its exact internal rollback.
This owner captures an operation, stops affected sources, accepts one outcome,
and publishes host assignment evidence. Pixhawk reconnection stays explicit.
Role/endpoint values are device identities; no spatial frame is transformed.
"""

from __future__ import annotations

import copy
import time
from dataclasses import dataclass, fields
from typing import Any, Callable
from PyQt5 import QtCore
from Workstation.workstation_core.network import NetworkRoleMovePlan

NETWORK_ROLE_FIELDS = (
    "network_role_assignments",
    "network_role_owned_entries",
    "network_role_operation",
    "network_role_status",
    "network_role_apply_thread",
    "network_role_apply_worker",
)


@dataclass(frozen=True)
class NetworkRolePorts:
    stop_owners: Callable[[tuple[str, ...]], None]
    owners_stopped: Callable[[tuple[str, ...]], bool]
    apply_assignments: Callable[..., None]
    save_settings: Callable[[], None]
    status_changed: Callable[[str, str], None]
    refresh: Callable[[], None]
    worker_factory: Callable[[tuple], QtCore.QObject]

    def __post_init__(self):
        for field in fields(self):
            if not callable(getattr(self, field.name)):
                raise TypeError(f"Missing network role operation: {field.name}")


@dataclass
class NetworkRoleJob:
    token: int
    operation: dict
    mode: str
    requests: tuple
    thread: Any
    worker: Any
    relay: Any = None
    terminal: bool = False


class _NetworkRoleRelay(QtCore.QObject):
    def __init__(self, owner, job):
        super().__init__(owner)
        self.owner, self.job = owner, job

    @QtCore.pyqtSlot(object)
    def completed(self, result):
        self.owner.deliver(self.job, result=result)

    @QtCore.pyqtSlot(str)
    def failed(self, error):
        self.owner.deliver(self.job, error=error)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.job)


class NetworkRoleRuntime(QtCore.QObject):
    def __init__(self, parent=None, *, thread_parent=None, publish_legacy=None):
        super().__init__(parent)
        self._thread_parent = thread_parent
        self._publish_legacy = publish_legacy
        self.ports: NetworkRolePorts | None = None
        self.network_role_assignments = {}
        self.network_role_owned_entries = {}
        self.network_role_operation = None
        self.network_role_status = {"state": "IDLE", "detail": "No network role operation"}
        self.network_role_apply_thread = None
        self.network_role_apply_worker = None
        self._next_token = 0
        self._next_operation_id = 0
        self._job = None
        self._jobs = {}
        self._closing = False
        self._scheduled = None
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._run_scheduled)

    def _publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    def set_status(self, state, detail):
        self.network_role_status = {"state": str(state), "detail": str(detail)}
        self._publish()
        try:
            self.ports.status_changed(str(state), str(detail))
        except Exception as exc:
            # Presentation cannot reopen or strand a completed host transaction.
            self.network_role_status["presentation_error"] = str(exc)
            self._publish()

    def _schedule(self, delay_ms, callback):
        if self._closing:
            return
        self._scheduled = (self.network_role_operation, callback)
        self._timer.start(delay_ms)

    @QtCore.pyqtSlot()
    def _run_scheduled(self):
        pending, self._scheduled = self._scheduled, None
        if pending is not None and not self._closing and pending[0] is self.network_role_operation:
            pending[1]()

    def begin(self, operation):
        if self._closing:
            raise RuntimeError("Network role runtime is shutting down")
        if self.network_role_operation is not None or self.network_role_apply_thread is not None or self._jobs:
            raise RuntimeError(
                "Network role operation already active "
                f"(operation={self.network_role_operation is not None}, "
                f"thread={self.network_role_apply_thread is not None}, jobs={len(self._jobs)})"
            )
        if self.ports is None:
            raise RuntimeError("Network role operations are not bound")
        self._next_operation_id += 1
        self.network_role_operation = copy.deepcopy(operation)
        self.network_role_operation["transaction_id"] = self._next_operation_id
        self._publish()
        active = tuple(operation.get("active_roles") or ())
        self.network_role_status = {
            "state": "STOPPING AFFECTED" if active else "APPLYING",
            "detail": f"{operation['role']} -> {operation['plan'].target_interface}",
        }
        self._publish()
        try:
            self.ports.stop_owners(active)
        except Exception as exc:
            self._fail(str(exc))
            return
        self._schedule(0, self.continue_assignment)

    def continue_assignment(self):
        operation = self.network_role_operation
        if self._closing or not isinstance(operation, dict) or operation.get("phase") != "STOPPING":
            return
        active = tuple(operation.get("active_roles") or ())
        try:
            stopped = self.ports.owners_stopped(active)
        except Exception as exc:
            self._fail(f"cannot inspect affected source shutdown: {exc}")
            return
        if not stopped:
            if time.monotonic() >= float(operation.get("deadline_s") or 0.0):
                self._fail("timed out stopping affected device owners")
            else:
                self._schedule(50, self.continue_assignment)
            return
        operation["phase"] = "APPLYING"
        self.network_role_status = {"state": "APPLYING", "detail": str(operation.get("role"))}
        self._publish()
        try:
            self.start_worker(tuple(operation.get("requests") or ()), mode="apply")
        except Exception as exc:
            self._fail(str(exc))

    def start_worker(self, requests, *, mode):
        if self._closing:
            raise RuntimeError("Network role runtime is shutting down")
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Network role jobs must start on their Qt owner thread")
        if self.network_role_apply_thread is not None or self._jobs:
            raise RuntimeError("network role worker already active")
        operation = self.network_role_operation
        if not isinstance(operation, dict):
            raise RuntimeError("network role operation is missing")
        requests = tuple(requests)
        worker = self.ports.worker_factory(requests)
        thread = QtCore.QThread(self._thread_parent)
        self._next_token += 1
        job = NetworkRoleJob(self._next_token, operation, mode, requests, thread, worker)
        relay = _NetworkRoleRelay(self, job)
        job.relay = relay
        try:
            thread.started.connect(worker.run)
            worker.completed.connect(relay.completed, QtCore.Qt.QueuedConnection)
            worker.failed.connect(relay.failed, QtCore.Qt.QueuedConnection)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(thread.deleteLater)
            thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
            worker.moveToThread(thread)
        except Exception:
            worker.deleteLater()
            thread.deleteLater()
            relay.deleteLater()
            raise
        operation["worker_mode"] = mode
        operation["active_requests"] = requests
        self._job = job
        self._jobs[job.token] = job
        self.network_role_apply_thread, self.network_role_apply_worker = thread, worker
        self._publish()
        thread.start()

    def deliver(self, job, *, result=None, error=None):
        if self._job is not job or self.network_role_operation is not job.operation or job.terminal:
            return
        job.terminal = True
        if error is not None:
            self._fail(str(error))
        else:
            try:
                self._apply_result(result, requests=job.requests, mode=job.mode)
            except Exception as exc:
                self._fail(f"network role result failed: {exc}")

    def _accepts_legacy_callback(self):
        operation = self.network_role_operation
        return self._job is None and (not isinstance(operation, dict) or "transaction_id" not in operation)

    def apply_result(self, result):
        """Compatibility for untagged embedders; admitted jobs require their relay."""
        if self._accepts_legacy_callback():
            try:
                self._apply_result(result)
            except Exception as exc:
                self._fail(f"network role result failed: {exc}")

    def fail(self, error):
        if self._accepts_legacy_callback():
            self._fail(error)

    def _complete(self):
        if self.network_role_apply_thread is None and not self._jobs:
            self.network_role_operation = None
        elif isinstance(self.network_role_operation, dict):
            self.network_role_operation["phase"] = "COMPLETE_NO_RECONNECT"
        self._publish()

    def _apply_result(self, result, *, requests=None, mode=None):
        operation = self.network_role_operation
        if not isinstance(operation, dict):
            return
        results = tuple(result or ())
        requests = tuple(operation.get("active_requests") or ()) if requests is None else requests
        mode = str(operation.get("worker_mode") or "apply") if mode is None else mode
        plan = operation.get("plan")
        if len(results) != len(requests):
            self._fail("worker result/request count mismatch")
            return
        if not isinstance(plan, NetworkRoleMovePlan):
            self._fail("network role plan is missing")
            return
        for request, response in zip(requests, results):
            if (
                getattr(response, "interface", None) != request.interface
                or getattr(response, "desired", None) != request.desired
            ):
                self._fail("worker result/request identity mismatch")
                return
        owned_by_interface = {request.interface: response.owned_after for request, response in zip(requests, results)}
        if mode == "rollback":
            self.network_role_owned_entries = copy.deepcopy(operation.get("before_owned", {}))
        for request, response in zip(requests, results):
            self.network_role_owned_entries[request.stable_id] = response.owned_after.to_mapping()
        operation["owned_after_by_interface"] = owned_by_interface
        assignments = plan.before_assignments if mode == "rollback" else plan.after_assignments
        self.network_role_assignments = {
            value.role: {"interface": value.interface, "stable_id": value.stable_id} for value in assignments
        }
        self._publish()
        try:
            self.ports.apply_assignments(
                plan.before_assignments if mode == "rollback" else plan.after_assignments,
                candidate=None if mode == "rollback" else dict(operation.get("candidate") or {}),
                restore_endpoints=mode == "rollback",
            )
            self.ports.save_settings()
        except Exception as exc:
            # The OS transaction already succeeded. Its exact prior NM snapshot
            # is not a returned receipt: never claim a rollback of the host here.
            self._finish_status(
                "APPLIED · SAVE FAILED" if mode != "rollback" else "ROLLBACK · SAVE FAILED",
                f"host operation completed but assignment publication/settings save failed: {exc}; Pixhawk remains disconnected until Connect",
            )
            return
        self._finish_status(
            "ROLLED BACK" if mode == "rollback" else "APPLIED",
            "host state restored; Pixhawk remains disconnected until Connect"
            if mode == "rollback"
            else "network role applied; Pixhawk remains disconnected until Connect",
        )

    def _finish_status(self, state, detail):
        try:
            self.set_status(state, detail)
        finally:
            self._complete()

    def _fail(self, error):
        operation = self.network_role_operation
        if isinstance(operation, dict):
            operation["failure_reason"] = str(error)
        mode = str(operation.get("worker_mode") or "apply") if isinstance(operation, dict) else "apply"
        self._finish_status(
            "ROLLBACK PARTIAL" if mode == "rollback" else "ERROR",
            str(error) if mode == "rollback" else f"{error}; Pixhawk remains disconnected until Connect",
        )

    def retire(self, job):
        if self._jobs.pop(job.token, None) is not job:
            return
        if self._job is job:
            self._job = None
            if self.network_role_apply_thread is job.thread:
                self.network_role_apply_thread = None
            if self.network_role_apply_worker is job.worker:
                self.network_role_apply_worker = None
            if self.network_role_operation is job.operation:
                if not job.terminal:
                    self._fail("network role worker finished without a result")
                elif job.operation.get("phase") == "COMPLETE_NO_RECONNECT":
                    self.network_role_operation = None
                else:
                    self._fail("network role result did not finalize its operation")
            self._publish()
            try:
                self.ports.refresh()
            except Exception as exc:
                self.network_role_status["presentation_error"] = str(exc)
                self._publish()
        job.relay.deleteLater()

    def clear_finished_legacy(self):
        if not self._accepts_legacy_callback():
            return
        self.network_role_apply_thread = None
        self.network_role_apply_worker = None
        operation = self.network_role_operation
        if isinstance(operation, dict) and operation.get("phase") == "COMPLETE_NO_RECONNECT":
            self.network_role_operation = None
        self._publish()
        self.ports.refresh()

    def prepare_shutdown(self):
        self._closing = True
        self._timer.stop()
        self._scheduled = None
        if not self._jobs:
            self.network_role_operation = None
            self._publish()

    def resource_inventory(self):
        result = {
            "network_role_apply_thread": self.network_role_apply_thread,
            "network_role_apply_worker": self.network_role_apply_worker,
            "network_role_transition_timer": self._timer,
        }
        for token, job in self._jobs.items():
            result[f"network_role_job_{token}_thread"] = job.thread
            result[f"network_role_job_{token}_worker"] = job.worker
        return result
