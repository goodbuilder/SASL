"""Own side-scoped Pixhawk Shell/PARAM jobs, callbacks and Qt retirement.

Workers retain the existing shared transport, command locks and device policy.
This owner has no panel, parameter interpretation or coordinate conversion.
Different sides/kinds keep their existing concurrency; one lane stays reserved
until its actual thread exits, including during Stop and start failure.
"""

from dataclasses import dataclass
from PyQt5 import QtCore


MAINTENANCE_FIELDS = tuple(
    f"pixhawk_{kind}_{handle}s" for kind in ("shell", "parameter") for handle in ("thread", "worker")
)


@dataclass
class MaintenanceJob:
    kind: str
    camera: str
    token: int
    thread: object
    worker: object
    completed: object
    failed: object
    retired: object
    accepts_result: object
    relay: object = None
    terminal: bool = False
    stopping: bool = False


class _MaintenanceRelay(QtCore.QObject):
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


class PixhawkMaintenanceJobs(QtCore.QObject):
    def __init__(self, parent=None):
        super().__init__(parent)
        for name in MAINTENANCE_FIELDS:
            setattr(self, name, {})
        self._jobs = {}
        self._next_token = 0
        self.callback_error = ""

    def _handles(self, kind, field):
        if kind not in {"shell", "parameter"} or field not in {"thread", "worker"}:
            raise ValueError("Unknown Pixhawk maintenance lane")
        return getattr(self, f"pixhawk_{kind}_{field}s")

    def busy(self, kind, camera):
        return (kind, camera) in self._jobs or camera in self._handles(kind, "thread")

    def prepare(self, kind, camera, *, worker_factory, completed, failed, retired, accepts_result):
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Maintenance jobs must be admitted on the Qt owner thread")
        if self.busy(kind, camera):
            raise RuntimeError(f"Pixhawk {kind} job is already running for {camera}")
        for name, callback in dict(
            worker_factory=worker_factory,
            completed=completed,
            failed=failed,
            retired=retired,
            accepts_result=accepts_result,
        ).items():
            if not callable(callback):
                raise TypeError(f"Missing maintenance operation: {name}")
        worker = worker_factory()
        if not isinstance(worker, QtCore.QObject):
            raise TypeError("Maintenance worker must be a QObject")
        if not callable(getattr(worker, "run", None)):
            worker.deleteLater()
            raise TypeError("Maintenance worker requires run")
        thread = QtCore.QThread(self)
        self._next_token += 1
        job = MaintenanceJob(kind, camera, self._next_token, thread, worker, completed, failed, retired, accepts_result)
        relay = _MaintenanceRelay(self, job)
        job.relay = relay
        try:
            thread.started.connect(worker.run)
            worker.completed.connect(relay.completed, QtCore.Qt.QueuedConnection)
            worker.failed.connect(relay.failed, QtCore.Qt.QueuedConnection)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
            worker.moveToThread(thread)
        except Exception:
            worker.deleteLater()
            thread.deleteLater()
            relay.deleteLater()
            raise
        self._jobs[(kind, camera)] = job
        self._handles(kind, "thread")[camera] = thread
        self._handles(kind, "worker")[camera] = worker
        return job

    def start(self, job):
        if self._jobs.get((job.kind, job.camera)) is not job:
            raise RuntimeError("Maintenance job is no longer current")
        try:
            job.thread.start()
            if not job.thread.isRunning():
                raise RuntimeError("Maintenance thread did not start")
        except Exception:
            self.abort(job)
            raise

    def abort(self, job):
        if self._jobs.get((job.kind, job.camera)) is not job:
            return
        job.stopping = True
        job.thread.requestInterruption()
        job.thread.quit()
        if not job.thread.isRunning():
            self.retire(job)
            job.worker.deleteLater()

    def deliver(self, job, *, result=None, error=None):
        if self._jobs.get((job.kind, job.camera)) is not job or job.terminal or job.stopping:
            return
        job.terminal = True
        try:
            if not job.accepts_result():
                return
            if error is None:
                job.completed(result)
            else:
                job.failed(str(error))
        except Exception as exc:
            self.callback_error = str(exc)

    def retire(self, job):
        if self._jobs.get((job.kind, job.camera)) is not job:
            return
        try:
            if job.thread.isRunning():
                return
        except RuntimeError:
            pass
        del self._jobs[(job.kind, job.camera)]
        job.terminal = True
        threads, workers = self._handles(job.kind, "thread"), self._handles(job.kind, "worker")
        if threads.get(job.camera) is job.thread:
            threads.pop(job.camera, None)
        if workers.get(job.camera) is job.worker:
            workers.pop(job.camera, None)
        try:
            job.retired()
        except Exception as exc:
            self.callback_error = str(exc)
        finally:
            job.relay.deleteLater()
            job.thread.deleteLater()

    def clear_finished_legacy(self, kind, camera, thread):
        job = self._jobs.get((kind, camera))
        if job is not None and job.thread is thread:
            self.retire(job)

    def stop(self, *, wait=False):
        all_stopped = True
        jobs = list(self._jobs.values())
        # Capture true task references as well as supported legacy handles;
        # clearing a presentation dictionary must never lose a live worker.
        entries = {(job.kind, job.camera, id(job.thread)): job.thread for job in jobs}
        for kind in ("shell", "parameter"):
            for camera, thread in list(self._handles(kind, "thread").items()):
                entries.setdefault((kind, camera, id(thread)), thread)
        for job in jobs:
            job.stopping = True
        for (kind, camera, _identity), thread in entries.items():
            try:
                interrupter = getattr(thread, "requestInterruption", None)
                if callable(interrupter):
                    interrupter()
                thread.quit()
                if thread.isRunning() and (not wait or not thread.wait(1000)):
                    all_stopped = False
                    continue
            except RuntimeError:
                pass
            job = self._jobs.get((kind, camera))
            if job is not None and job.thread is thread:
                self.retire(job)
            elif self._handles(kind, "thread").get(camera) is thread:
                self._handles(kind, "thread").pop(camera, None)
                self._handles(kind, "worker").pop(camera, None)
        return all_stopped

    def resource_inventory(self):
        resources = {name: getattr(self, name) for name in MAINTENANCE_FIELDS}
        for job in self._jobs.values():
            if (
                self._handles(job.kind, "thread").get(job.camera) is not job.thread
                or self._handles(job.kind, "worker").get(job.camera) is not job.worker
            ):
                prefix = f"pixhawk_maintenance_{job.kind}_{job.camera}_{job.token}"
                resources[prefix + "_thread"] = job.thread
                resources[prefix + "_worker"] = job.worker
        return resources
