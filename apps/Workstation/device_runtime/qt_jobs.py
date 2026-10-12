"""Qt execution/retirement mechanism shared by two finite device job owners.

Operation commands, exclusions, configuration, transport locks and presentation
belong to the concrete owners/adapters. This mechanism knows only identity,
queued QObject delivery, partial start, native exit and resource visibility.
"""

from dataclasses import dataclass, field
import threading
from PyQt5 import QtCore


@dataclass
class OperationJob:
    token: int
    command: object
    thread: object
    worker: object
    completed: object
    failed: object
    progress: object
    retired: object
    accepts_result: object
    relay: object = None
    terminal: bool = False
    started: threading.Event = field(default_factory=threading.Event)
    callback_depth: int = 0
    retire_requested: bool = False

    @property
    def lane(self):
        return (self.command.kind, self.command.camera if self.command.per_camera else "")


class _OperationRelay(QtCore.QObject):
    def __init__(self, owner, job):
        super().__init__(owner)
        self.owner, self.job = owner, job

    @QtCore.pyqtSlot(object)
    def completed(self, value):
        self.owner.deliver(self.job, "completed", value)

    @QtCore.pyqtSlot(str)
    def failed(self, value):
        self.owner.deliver(self.job, "failed", value)

    @QtCore.pyqtSlot(object)
    def progress(self, value):
        self.owner.deliver(self.job, "progress", value)

    @QtCore.pyqtSlot(str)
    def text_progress(self, value):
        self.owner.deliver(self.job, "progress", value)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.job)


class QtOperationJobs(QtCore.QObject):
    def __init__(self, parent=None, *, publish_legacy=None):
        super().__init__(parent)
        self._jobs = {}
        self._next_token = 0
        self.closing = False
        self.callback_error = ""
        self._publish_legacy = publish_legacy
        for name in self.handle_fields:
            setattr(self, name, {} if name.endswith("s") else None)

    def _publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    # Subclasses declare finite operation kinds and their legacy aliases.
    # This executor owns Qt lifetime only, not operation admission/device policy.
    command_types = ()
    handle_fields = ()
    prefixes = {}
    per_camera_kinds = frozenset()
    cancel_kinds = frozenset()

    def _handle(self, kind, camera, field):
        prefix = self.prefixes[kind]
        if kind in self.per_camera_kinds:
            return getattr(self, f"{prefix}_{field}s").get(camera)
        return getattr(self, f"{prefix}_{field}")

    def _set_handle(self, kind, camera, field, value):
        prefix = self.prefixes[kind]
        if kind in self.per_camera_kinds:
            handles = getattr(self, f"{prefix}_{field}s")
            if value is None:
                handles.pop(camera, None)
            else:
                handles[camera] = value
        else:
            setattr(self, f"{prefix}_{field}", value)

    def _lane(self, kind, camera):
        return (kind, camera if kind in self.per_camera_kinds else "")

    def busy(self, kind, camera=""):
        if kind not in self.prefixes:
            raise ValueError("Unknown device operation")
        return self._lane(kind, camera) in self._jobs or self._handle(kind, camera, "thread") is not None

    def prepare(self, command, *, worker_factory, completed, failed, progress=None,
                retired=None, accepts_result=lambda: True):
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Device jobs must be admitted on the Qt owner thread")
        if not isinstance(command, self.command_types):
            raise TypeError("A captured Pixhawk device command is required")
        if self.closing:
            raise RuntimeError("Pixhawk administration is shutting down")
        if self.busy(command.kind, command.camera):
            raise RuntimeError(f"Pixhawk {command.kind} is already running")
        for name, callback in dict(worker_factory=worker_factory, completed=completed,
                                   failed=failed, accepts_result=accepts_result).items():
            if not callable(callback):
                raise TypeError(f"Missing device operation: {name}")
        worker = command.build(worker_factory)
        if not isinstance(worker, QtCore.QObject):
            raise TypeError("Device worker must be a QObject")
        thread = QtCore.QThread(self)
        self._next_token += 1
        job = OperationJob(self._next_token, command, thread, worker, completed, failed,
                       progress, retired, accepts_result)
        relay = _OperationRelay(self, job)
        job.relay = relay
        try:
            thread.started.connect(job.started.set, QtCore.Qt.DirectConnection)
            thread.started.connect(worker.run)
            getattr(worker, command.result_signal).connect(relay.completed, QtCore.Qt.QueuedConnection)
            worker.failed.connect(relay.failed, QtCore.Qt.QueuedConnection)
            if progress is not None:
                receiver = relay.text_progress if command.text_progress else relay.progress
                worker.progress.connect(receiver, QtCore.Qt.QueuedConnection)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
            worker.moveToThread(thread)
        except Exception:
            worker.deleteLater()
            thread.deleteLater()
            relay.deleteLater()
            raise
        self._jobs[job.lane] = job
        self._set_handle(command.kind, command.camera, "thread", thread)
        self._set_handle(command.kind, command.camera, "worker", worker)
        self._publish()
        return job

    def start(self, job):
        if self._jobs.get(job.lane) is not job or self.closing:
            self.fail_start(job, RuntimeError("Device job is no longer admissible"))
            return False
        try:
            job.thread.start()
            if not job.thread.isRunning() and not job.started.is_set():
                raise RuntimeError("Device thread did not start")
        except Exception as exc:
            return self.fail_start(job, exc)
        return True

    def fail_start(self, job, error):
        if self._jobs.get(job.lane) is not job:
            return False
        if job.started.is_set() or job.thread.isRunning():
            # A command can already be on the wire, or evidence on disk. Do not
            # advertise a failed admission that invites a duplicate operation.
            self.callback_error = f"start reported an error after launch: {error}"
            return True
        self.deliver(job, "failed", str(error))
        if self._jobs.get(job.lane) is job:
            job.thread.quit()
            self.retire(job)
        job.worker.deleteLater()
        return False

    def deliver(self, job, event, value):
        if self._jobs.get(job.lane) is not job or job.terminal:
            return
        if event not in {"completed", "failed", "progress"}:
            raise ValueError("Unknown device event")
        if event != "progress":
            job.terminal = True
        job.callback_depth += 1
        try:
            if job.accepts_result():
                callback = getattr(job, event)
                if callback is not None:
                    callback(value)
        except Exception as exc:
            self.callback_error = f"{event}: {exc}"
            if event == "completed":
                try:
                    job.failed(str(exc))
                except Exception as failure:
                    self.callback_error += f"; failure presentation: {failure}"
        finally:
            job.callback_depth -= 1
            if not job.callback_depth and job.retire_requested:
                self.retire(job)

    def retire(self, job):
        if self._jobs.get(job.lane) is not job:
            return
        if job.callback_depth:
            job.retire_requested = True
            return
        try:
            if job.thread.isRunning():
                return
        except RuntimeError:
            pass
        del self._jobs[job.lane]
        job.terminal = True
        for field_name in ("thread", "worker"):
            if self._handle(job.command.kind, job.command.camera, field_name) is getattr(job, field_name):
                self._set_handle(job.command.kind, job.command.camera, field_name, None)
        self._publish()
        try:
            if job.retired is not None:
                job.retired()
        except Exception as exc:
            self.callback_error = f"retired: {exc}"
        finally:
            job.relay.deleteLater()
            job.thread.deleteLater()

    def clear_finished_legacy(self, kind, camera=""):
        job = self._jobs.get(self._lane(kind, camera))
        # An untagged legacy finished callback cannot retire an admitted job.
        # Its ordered QObject relay alone proves that queued results drained.
        if job is not None:
            return False
        thread = self._handle(kind, camera, "thread")
        try:
            if thread is not None and thread.isRunning():
                return False
        except RuntimeError:
            pass
        for field_name in ("thread", "worker"):
            self._set_handle(kind, camera, field_name, None)
        self._publish()
        return True

    def cancel(self, kind, camera=""):
        job = self._jobs.get(self._lane(kind, camera))
        worker = job.worker if job is not None else self._handle(kind, camera, "worker")
        if worker is None:
            return False
        worker.cancel()
        return True

    def stop(self, *, wait=False):
        entries = {(job.command.kind, job.command.camera, id(job.thread)): (job.thread, job.worker)
                   for job in self._jobs.values()}
        for kind in self.prefixes:
            cameras = tuple(getattr(self, self.prefixes[kind] + "_threads")) if kind in self.per_camera_kinds else ("",)
            for camera in cameras:
                thread = self._handle(kind, camera, "thread")
                if thread is not None:
                    entries.setdefault((kind, camera, id(thread)), (thread, self._handle(kind, camera, "worker")))
        all_stopped = True
        for (kind, camera, _identity), (thread, worker) in entries.items():
            if kind in self.cancel_kinds and worker is not None:
                try:
                    worker.cancel()
                except Exception:
                    pass
            try:
                thread.quit()
                if thread.isRunning() and (not wait or not thread.wait(1000)):
                    all_stopped = False
                    continue
            except RuntimeError:
                pass
            if self._handle(kind, camera, "thread") is thread:
                self.clear_finished_legacy(kind, camera)
        # Native stop is not ordered result delivery. Keep real jobs inventoried
        # until their GUI finished relay consumes transfer/firmware evidence.
        return all_stopped and not self._jobs

    def request_shutdown(self):
        self.closing = True
        return self.stop(wait=False)

    def resource_inventory(self):
        resources = {name: getattr(self, name) for name in self.handle_fields}
        for job in self._jobs.values():
            if any(self._handle(job.command.kind, job.command.camera, field_name) is not getattr(job, field_name)
                   for field_name in ("thread", "worker")):
                prefix = f"{self.prefixes[job.command.kind]}_{job.command.camera}_{job.token}"
                resources[prefix + "_thread"] = job.thread
                resources[prefix + "_worker"] = job.worker
        return resources
