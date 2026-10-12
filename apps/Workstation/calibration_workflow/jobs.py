"""One shared Qt job slot for capture, calibration and random validation.

Workers keep the admitted native/input snapshots, numerical methods and files.
Closing refuses new work and continuations; it still consumes in-flight
results before retirement because a capture may already have saved images.
"""

from dataclasses import dataclass, field
import threading
from PyQt5 import QtCore


CALIBRATION_JOB_FIELDS = ("checkerboard_calibration_thread", "checkerboard_calibration_worker")


@dataclass
class CalibrationJob:
    token: int
    kind: str
    thread: object
    worker: object
    completed: object
    failed: object
    progress: object
    retired: object
    relay: object = None
    terminal: bool = False
    continuations: list = field(default_factory=list)
    started: threading.Event = field(default_factory=threading.Event)
    callback_depth: int = 0
    retire_requested: bool = False


class _CalibrationRelay(QtCore.QObject):
    def __init__(self, owner, job):
        super().__init__(owner)
        self.owner, self.job = owner, job

    @QtCore.pyqtSlot(object)
    def completed(self, result):
        self.owner.deliver(self.job, "completed", result)

    @QtCore.pyqtSlot(str)
    def failed(self, error):
        self.owner.deliver(self.job, "failed", error)

    @QtCore.pyqtSlot(object)
    def progress(self, value):
        self.owner.deliver(self.job, "progress", value)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.job)


class CalibrationJobs(QtCore.QObject):
    def __init__(self, parent=None, *, publish_legacy=None):
        super().__init__(parent)
        self.current = None
        self.closing = False
        self._next_token = 0
        self._legacy_thread = None
        self._legacy_worker = None
        self._publish_legacy = publish_legacy
        self.callback_error = ""

    @property
    def has_admitted(self):
        return self._next_token > 0

    @property
    def busy(self):
        return self.current is not None or self._legacy_thread is not None

    def _publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    @property
    def checkerboard_calibration_thread(self):
        return self.current.thread if self.current is not None else self._legacy_thread

    @checkerboard_calibration_thread.setter
    def checkerboard_calibration_thread(self, value):
        if self.current is not None:
            if value is not self.current.thread:
                raise RuntimeError("An admitted calibration thread must retire through its owner")
            return
        self._legacy_thread = value

    @property
    def checkerboard_calibration_worker(self):
        return self.current.worker if self.current is not None else self._legacy_worker

    @checkerboard_calibration_worker.setter
    def checkerboard_calibration_worker(self, value):
        if self.current is not None:
            if value is not self.current.worker:
                raise RuntimeError("An admitted calibration worker must retire through its owner")
            return
        self._legacy_worker = value

    def prepare(self, kind, *, worker_factory, completed, failed, progress, retired):
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Calibration jobs must be admitted on the Qt owner thread")
        if self.closing:
            raise RuntimeError("Calibration is shutting down")
        if self.busy:
            raise RuntimeError("A calibration operation is already running")
        if kind not in {"capture", "intrinsics", "extrinsics", "random_validation"}:
            raise ValueError("Unknown calibration job kind")
        for name, callback in dict(
            worker_factory=worker_factory, completed=completed, failed=failed, progress=progress, retired=retired
        ).items():
            if not callable(callback):
                raise TypeError(f"Missing calibration operation: {name}")
        worker = worker_factory()
        if not isinstance(worker, QtCore.QObject):
            raise TypeError("Calibration worker must be a QObject")
        if not callable(getattr(worker, "run", None)):
            worker.deleteLater()
            raise TypeError("Calibration worker requires run")
        thread = QtCore.QThread(self)
        self._next_token += 1
        job = CalibrationJob(self._next_token, kind, thread, worker, completed, failed, progress, retired)
        relay = _CalibrationRelay(self, job)
        job.relay = relay
        try:
            # Direct delivery records only thread-safe launch evidence, never
            # GUI state. A thread may finish before a start exception is seen.
            thread.started.connect(job.started.set, QtCore.Qt.DirectConnection)
            thread.started.connect(worker.run)
            worker.completed.connect(relay.completed, QtCore.Qt.QueuedConnection)
            worker.failed.connect(relay.failed, QtCore.Qt.QueuedConnection)
            worker.progress.connect(relay.progress, QtCore.Qt.QueuedConnection)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
            worker.moveToThread(thread)
        except Exception:
            worker.deleteLater()
            thread.deleteLater()
            relay.deleteLater()
            raise
        self.current = job
        self._publish()
        return job

    def start(self, job):
        try:
            if self.current is not job or self.closing:
                raise RuntimeError("Calibration job is no longer admissible")
            job.thread.start()
            if not job.thread.isRunning():
                raise RuntimeError("Calibration thread did not start")
        except Exception as exc:
            if self.fail_start(job, exc):
                return
            raise

    def fail_start(self, job, error):
        if self.current is not job:
            return False
        if job.started.is_set() or job.thread.isRunning():
            # The admitted operation may already have persisted a PNG. Keep
            # its outcome and report acceptance so FTP does not retry its head.
            self.callback_error = f"start reported an error after launch: {error}"
            return True
        self.deliver(job, "failed", str(error))
        job.thread.quit()
        if not job.thread.isRunning():
            self.retire(job)
            job.worker.deleteLater()
        return False

    def deliver(self, job, event, value):
        if self.current is not job or job.terminal:
            return
        if event != "progress":
            job.terminal = True
        job.callback_depth += 1
        try:
            getattr(job, event)(value)
        except Exception as exc:
            self.callback_error = f"{event}: {exc}"
            if event == "completed":
                try:
                    job.failed(str(exc))
                except Exception as failure:
                    self.callback_error += f"; failure presentation: {failure}"
        finally:
            job.callback_depth -= 1
            if job.callback_depth == 0 and job.retire_requested:
                self.retire(job)

    def after_current_retired(self, callback):
        if self.closing or self.current is None:
            return False
        self.current.continuations.append(callback)
        return True

    def retire(self, job):
        if self.current is not job:
            return
        if job.callback_depth:
            job.retire_requested = True
            return
        try:
            if job.thread.isRunning():
                return
        except RuntimeError:
            pass
        self.current = None
        job.retire_requested = False
        job.terminal = True
        self._publish()
        try:
            job.retired()
            # Cleanup can re-enter the UI and admit another operation. The old
            # continuation must not run inside that replacement's lifetime.
            if not self.closing and self.current is None and self._next_token == job.token:
                for callback in job.continuations:
                    if self.closing or self.current is not None or self._next_token != job.token:
                        break
                    callback()
        except Exception as exc:
            self.callback_error = f"retired: {exc}"
        finally:
            job.continuations.clear()
            job.relay.deleteLater()
            job.thread.deleteLater()

    def request_shutdown(self):
        self.closing = True
        job = self.current
        thread = self.checkerboard_calibration_thread
        if thread is not None:
            try:
                thread.quit()
                if job is None and not thread.isRunning():
                    self._legacy_thread = self._legacy_worker = None
                    self._publish()
            except RuntimeError:
                if job is None:
                    self._legacy_thread = self._legacy_worker = None
                    self._publish()
        # A stopped QThread is not proof that its queued result was consumed.
        # Only the ordered GUI finished relay releases an admitted job.
        return not self.busy

    def resource_inventory(self):
        return {name: getattr(self, name) for name in CALIBRATION_JOB_FIELDS}
