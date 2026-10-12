"""Camera control jobs and their GUI delivery/retirement identity.

Workers retain existing commands, retry policies and device I/O. A lane stays
busy until its actual QThread finishes; stopped/old outcomes cannot start new
camera work. Qt parentage remains the existing panel widget.
"""

from dataclasses import dataclass
from threading import Event
from PyQt5 import QtCore
from Workstation.camera_runtime.rejoin import ReactivationRequest, finish_reactivation


CONTROL_LANES = {
    "detect": ("camera_auto_detect", "detected"),
    "control": ("wiris_control_check", "checked"),
    "check": ("rtsp_check", "checked"),
    "enable": ("rtsp_enable", "enabled"),
}
CONTROL_JOB_FIELDS = tuple(f"{prefix}_{kind}" for prefix, _ in CONTROL_LANES.values() for kind in ("thread", "worker"))

CONTROL_STATE_FIELDS = (
    "_camera_auto_connect_attempt",
    "_camera_auto_connect_retry_generation",
    "_auto_wiris_enabled_cameras",
    "auto_wiris_connect_pending",
    "_rtsp_reactivation_side",
    "_rtsp_reactivation_enable_detection",
    "_rtsp_reactivation_previous_video_path",
    "_rtsp_reactivation_generation",
    "_rtsp_reactivation_stop_generation",
    "_rtsp_reactivation_stop_timer",
)


@dataclass
class CameraControlJob:
    token: int
    kind: str
    thread: object
    worker: object
    completed: object
    failed: object
    retired: object
    started: Event
    relay: object = None
    terminal: bool = False
    cancelled: bool = False
    callback_depth: int = 0
    retire_pending: bool = False


class _ControlRelay(QtCore.QObject):
    def __init__(self, owner, job):
        super().__init__(owner)
        self.owner, self.job = owner, job

    @QtCore.pyqtSlot(object)
    def completed(self, result):
        self.owner.deliver(self.job, "completed", result)

    @QtCore.pyqtSlot(str)
    def failed(self, error):
        self.owner.deliver(self.job, "failed", error)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.job)


class CameraControlRuntime(QtCore.QObject):
    def __init__(self, parent=None, *, thread_parent=None):
        super().__init__(parent)
        self._thread_parent = parent if thread_parent is None else thread_parent
        self.jobs = {}
        self._legacy = dict.fromkeys(CONTROL_JOB_FIELDS)
        self._next_token = 0
        self._admitted_lanes = set()
        self._delivering = []
        self.closing = False
        self.callback_error = ""
        self._retry_timer = None
        self._camera_auto_connect_attempt = 0
        self._camera_auto_connect_retry_generation = 0
        self._auto_wiris_enabled_cameras = set()
        self.auto_wiris_connect_pending = False
        self._rtsp_reactivation_side = None
        self._rtsp_reactivation_enable_detection = False
        self._rtsp_reactivation_previous_video_path = None
        self._rtsp_reactivation_generation = 0
        self._rtsp_reactivation_stop_generation = None
        self._rtsp_reactivation_stop_timer = QtCore.QTimer(self._thread_parent)

    def read_field(self, name):
        for kind, (prefix, _) in CONTROL_LANES.items():
            job = self.jobs.get(kind)
            if job is not None and name == f"{prefix}_thread":
                return job.thread
            if job is not None and name == f"{prefix}_worker":
                return job.worker
        return self._legacy[name]

    def write_field(self, name, value):
        for kind, (prefix, _) in CONTROL_LANES.items():
            job = self.jobs.get(kind)
            if job is not None and name in {f"{prefix}_thread", f"{prefix}_worker"}:
                current = job.thread if name.endswith("_thread") else job.worker
                if value is not current:
                    raise RuntimeError("An admitted camera control job must retire through its owner")
                return
        self._legacy[name] = value

    def accepts_legacy_outcome(self, kind):
        return kind not in self._admitted_lanes or bool(self._delivering and self._delivering[-1] == kind)

    def launch(self, kind, *, worker_factory, completed, failed, retired):
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Camera control jobs require the GUI owner thread")
        if kind not in CONTROL_LANES:
            raise ValueError("Unknown camera control operation")
        prefix, signal = CONTROL_LANES[kind]
        if self.closing or kind in self.jobs or self._legacy[f"{prefix}_thread"] is not None:
            return False
        if not all(callable(callback) for callback in (worker_factory, completed, failed, retired)):
            raise TypeError("Camera control requires worker/result/retirement callbacks")
        worker = thread = relay = None
        try:
            worker = worker_factory()
            if not isinstance(worker, QtCore.QObject) or not callable(getattr(worker, "run", None)):
                raise TypeError("Camera control worker requires QObject/run")
            thread = QtCore.QThread(self._thread_parent)
            self._next_token += 1
            job = CameraControlJob(self._next_token, kind, thread, worker, completed, failed, retired, Event())
            relay = _ControlRelay(self, job)
            job.relay = relay
            thread.started.connect(job.started.set, QtCore.Qt.DirectConnection)
            thread.started.connect(worker.run)
            getattr(worker, signal).connect(relay.completed, QtCore.Qt.QueuedConnection)
            worker.failed.connect(relay.failed, QtCore.Qt.QueuedConnection)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
            worker.moveToThread(thread)
            self.jobs[kind] = job
            self._admitted_lanes.add(kind)
            try:
                thread.start()
                if not thread.isRunning() and not job.started.is_set():
                    raise RuntimeError("Camera control thread did not start")
            except Exception as exc:
                if thread.isRunning() or job.started.is_set():
                    self.callback_error = f"start: {exc}"
                    return True
                self.deliver(job, "failed", str(exc))
                self.retire(job)
                worker.deleteLater()
                return False
            return True
        except Exception as exc:
            self.callback_error = f"prepare: {exc}"
            for resource in (worker, thread, relay):
                if isinstance(resource, QtCore.QObject):
                    resource.deleteLater()
            self._delivering.append(kind)
            try:
                failed(str(exc))
            finally:
                self._delivering.pop()
                retired()
            return False

    def deliver(self, job, event, value):
        if self.jobs.get(job.kind) is not job or job.terminal or job.cancelled or self.closing:
            return
        job.terminal = True
        job.callback_depth += 1
        self._delivering.append(job.kind)
        try:
            getattr(job, event)(value)
        except Exception as exc:
            self.callback_error = f"{event}: {exc}"
            if event == "completed":
                try:
                    job.failed(str(exc))
                except Exception as nested:
                    self.callback_error += f"; failure: {nested}"
        finally:
            self._delivering.pop()
            job.callback_depth -= 1
            if job.retire_pending and not job.callback_depth:
                self.retire(job)

    def retire(self, job):
        if self.jobs.get(job.kind) is not job:
            return
        if job.callback_depth:
            job.retire_pending = True
            return
        if job.thread.isRunning():
            return
        self.jobs.pop(job.kind)
        job.terminal = True
        try:
            job.retired()
        except Exception as exc:
            self.callback_error = f"retired: {exc}"
        finally:
            job.relay.deleteLater()
            job.thread.deleteLater()

    def schedule_retry(self, delay_ms, callback):
        self.cancel_retry()
        if self.closing:
            return
        timer = QtCore.QTimer(self)
        timer.setSingleShot(True)
        self._retry_timer = timer

        def invoke():
            if self._retry_timer is not timer or self.closing:
                return
            self._retry_timer = None
            timer.deleteLater()
            callback()

        timer.timeout.connect(invoke)
        timer.start(delay_ms)

    def cancel_retry(self):
        timer, self._retry_timer = self._retry_timer, None
        if timer is not None:
            timer.stop()
            timer.deleteLater()

    def invalidate_reactivation(self):
        self._rtsp_reactivation_generation += 1

    def finish_reactivation(self, success, detail, *, recovery, ports):
        side = self._rtsp_reactivation_side
        if side is None:
            return
        enabled = self._rtsp_reactivation_enable_detection
        self._rtsp_reactivation_stop_timer.stop()
        self._rtsp_reactivation_stop_generation = None
        self._rtsp_reactivation_side = None
        self._rtsp_reactivation_enable_detection = False
        self.invalidate_reactivation()
        request = ReactivationRequest(side, self._rtsp_reactivation_generation, bool(enabled))
        finish_reactivation(
            request,
            success=success,
            detail=detail,
            current=lambda: request.generation == self._rtsp_reactivation_generation,
            recovery=recovery,
            ports=ports,
        )

    def cancel_sequence(self):
        self.cancel_retry()
        self._camera_auto_connect_retry_generation += 1
        self.auto_wiris_connect_pending = False
        self.invalidate_reactivation()
        self._rtsp_reactivation_stop_generation = None
        self._rtsp_reactivation_side = None
        self._rtsp_reactivation_enable_detection = False
        self._rtsp_reactivation_stop_timer.stop()
        for kind in ("detect", "enable", "check"):
            job = self.jobs.get(kind)
            if job is not None:
                job.cancelled = True
                job.thread.quit()

    def stop(self, *, wait=False):
        """Stop admitted work without permanently closing future admission."""
        self.cancel_sequence()
        for job in tuple(self.jobs.values()):
            job.cancelled = True
            job.thread.quit()
            if wait and job.thread.isRunning():
                job.thread.wait(1000)
        for kind, (prefix, _) in CONTROL_LANES.items():
            if kind in self.jobs:
                continue
            name = f"{prefix}_thread"
            thread = self._legacy[name]
            if thread is not None:
                try:
                    thread.quit()
                    if thread.isRunning() and (not wait or not thread.wait(1000)):
                        continue
                except RuntimeError:
                    pass
            self._legacy[name] = self._legacy[f"{prefix}_worker"] = None
        # isRunning=False does not prove queued GUI outcomes were consumed.
        return not self.jobs and not any(value is not None for value in self._legacy.values())

    def request_shutdown(self):
        self.closing = True
        return self.stop(wait=False)

    def resource_inventory(self):
        return {
            **{name: self.read_field(name) for name in CONTROL_JOB_FIELDS},
            "camera_auto_connect_retry_timer": self._retry_timer,
            "_rtsp_reactivation_stop_timer": self._rtsp_reactivation_stop_timer,
        }
