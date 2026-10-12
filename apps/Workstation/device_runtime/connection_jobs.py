"""Own the two mutually exclusive Pixhawk connection jobs and their Qt lifetime.

Roles identify devices, not coordinate origins. Payloads and producer frames
are passed through unchanged. The caller supplies frozen operation callbacks;
this owner never reads a panel, settings, widgets or a newer subscriber.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from PyQt5 import QtCore


@dataclass
class ConnectionJob:
    kind: str
    token: int = 0
    thread: Any = None
    worker: Any = None
    relay: Any = None
    terminal: bool = False
    completed: Callable[[object], None] | None = None
    failed: Callable[[str], None] | None = None
    finished: Callable[[], None] | None = None
    accepts_result: Callable[[], bool] | None = None


class _ConnectionJobRelay(QtCore.QObject):
    """Keep the job identity and receiver's GUI affinity after worker deletion."""

    def __init__(self, owner: "PixhawkConnectionJobs", job: ConnectionJob) -> None:
        super().__init__(owner)
        self.owner = owner
        self.job = job

    @QtCore.pyqtSlot(object)
    def completed(self, result: object) -> None:
        self.owner.deliver(self.job, result=result)

    @QtCore.pyqtSlot(str)
    def failed(self, error: str) -> None:
        self.owner.deliver(self.job, error=error)

    @QtCore.pyqtSlot()
    def finished(self) -> None:
        self.owner.retire(self.job)


class PixhawkConnectionJobs(QtCore.QObject):
    """Only owner of connect/disconnect thread and worker references."""

    def __init__(
        self,
        parent: QtCore.QObject | None,
        *,
        thread_parent: QtCore.QObject | None,
        publish_legacy: Callable[[str, Any, Any], None] | None = None,
    ) -> None:
        super().__init__(parent)
        self._thread_parent = thread_parent
        self._publish_legacy = publish_legacy
        self._next_token = 0
        self._slots = {kind: ConnectionJob(kind) for kind in ("connect", "disconnect")}
        self._jobs: dict[int, ConnectionJob] = {}

    def handle(self, kind: str, field: str) -> Any:
        if field not in {"thread", "worker"}:
            raise ValueError(f"Unsupported connection job handle: {field}")
        return getattr(self._slots[kind], field)

    def set_legacy_handle(self, kind: str, field: str, value: Any) -> None:
        """Named compatibility properties address the same stored handle."""
        if field not in {"thread", "worker"}:
            raise ValueError(f"Unsupported connection job handle: {field}")
        setattr(self._slots[kind], field, value)

    def _publish(self, job: ConnectionJob) -> None:
        if self._publish_legacy is not None:
            self._publish_legacy(job.kind, job.thread, job.worker)

    def prepare(
        self,
        kind: str,
        *,
        worker_factory: Callable[[], QtCore.QObject],
        completed: Callable[[object], None],
        failed: Callable[[str], None],
        finished: Callable[[], None],
        accepts_result: Callable[[], bool],
    ) -> ConnectionJob:
        if kind not in self._slots:
            raise ValueError(f"Unknown connection operation: {kind}")
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Connection jobs must be admitted on their Qt owner thread")
        for name, callback in (
            ("worker_factory", worker_factory),
            ("completed", completed),
            ("failed", failed),
            ("finished", finished),
            ("accepts_result", accepts_result),
        ):
            if not callable(callback):
                raise TypeError(f"Missing mandatory connection operation: {name}")
        if any(job.thread is not None or job.worker is not None for job in self._slots.values()):
            raise RuntimeError("A Pixhawk connect/disconnect operation is already in progress")

        worker = worker_factory()
        thread = QtCore.QThread(self._thread_parent)
        self._next_token += 1
        job = ConnectionJob(
            kind,
            token=self._next_token,
            thread=thread,
            worker=worker,
            completed=completed,
            failed=failed,
            finished=finished,
            accepts_result=accepts_result,
        )
        relay = _ConnectionJobRelay(self, job)
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
        self._slots[kind] = job
        self._jobs[job.token] = job
        self._publish(job)
        return job

    def start(self, job: ConnectionJob) -> None:
        if self._slots.get(job.kind) is not job or self._jobs.get(job.token) is not job:
            raise RuntimeError("Connection job is no longer current")
        job.thread.start()

    def deliver(self, job: ConnectionJob, *, result: object = None, error: str | None = None) -> None:
        if self._slots.get(job.kind) is not job or job.terminal:
            return
        job.terminal = True
        if not job.accepts_result():
            return
        if error is None:
            job.completed(result)
        else:
            job.failed(str(error))

    def retire(self, job: ConnectionJob) -> None:
        """Called only after QThread.finished; never clear a newer job's fields."""
        if self._jobs.pop(job.token, None) is not job:
            return
        if self._slots.get(job.kind) is job:
            job.thread = None
            job.worker = None
            self._publish(job)
            self._slots[job.kind] = ConnectionJob(job.kind)
            job.finished()
        if job.relay is not None:
            job.relay.deleteLater()

    def resource_inventory(self) -> dict[str, object]:
        resources = {}
        for kind, job in self._slots.items():
            prefix = "pixhawk_connection" if kind == "connect" else "pixhawk_disconnection"
            resources[f"{prefix}_thread"] = job.thread
            resources[f"{prefix}_worker"] = job.worker
        for token, job in self._jobs.items():
            if self._slots.get(job.kind) is not job:
                resources[f"pixhawk_retiring_{token}_thread"] = job.thread
                resources[f"pixhawk_retiring_{token}_worker"] = job.worker
        return resources
