"""Own deferred camera replacement and teardown, with cancellable Qt timers.

Host monotonic seconds and millisecond scheduling retain the existing policy.
Pane operations, recording finalization and Run effects remain explicit ports;
this owner never reads a panel, frame, recording path or measurement state.
"""

from dataclasses import dataclass
import time
from PyQt5 import QtCore


LIFECYCLE_TIMER_FIELDS = (
    "_camera_start_request_generation",
    "_pending_camera_start",
    "_camera_start_retry_deadline_s",
    "_camera_stop_watchdog_generation",
)


@dataclass
class _TimerJob:
    generation: int
    stop_pair: object
    completed: object
    timeout: object = None
    finalize: object = None
    timer: object = None


class CameraLifecycleTimers(QtCore.QObject):
    def __init__(self, parent=None, *, publish_legacy=None, now=None):
        super().__init__(parent)
        self._camera_start_request_generation = 0
        self._pending_camera_start = None
        self._camera_start_retry_deadline_s = None
        self._camera_stop_watchdog_generation = 0
        self._restart = None
        self._stop = None
        self._publish_legacy = publish_legacy
        self._now = now or time.monotonic

    def _publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    def _schedule(self, delay_ms, callback):
        timer = QtCore.QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(callback)
        timer.start(delay_ms)
        return timer

    @staticmethod
    def _release(job):
        if job is not None and job.timer is not None:
            job.timer.stop()
            job.timer.deleteLater()
            job.timer = None

    def schedule_restart(self, enable_detection, switch_to_depth, *, timeout_s, stop_pair, start, timeout):
        self._release(self._restart)
        self._camera_start_request_generation += 1
        self._pending_camera_start = (bool(enable_detection), bool(switch_to_depth))
        if self._camera_start_retry_deadline_s is None:
            self._camera_start_retry_deadline_s = self._now() + timeout_s
        job = _TimerJob(self._camera_start_request_generation, stop_pair, start, timeout=timeout)
        self._restart = job
        self._publish()
        job.timer = self._schedule(100, lambda: self._retry_restart(job))

    def _retry_restart(self, job):
        if self._restart is not job or job.generation != self._camera_start_request_generation:
            return
        self._release(job)
        pending = self._pending_camera_start
        if pending is None:
            self._restart = None
            return
        if self._now() >= float(self._camera_start_retry_deadline_s or 0.0):
            self._restart = None
            self._pending_camera_start = None
            self._camera_start_retry_deadline_s = None
            self._publish()
            job.timeout(pending)
            return
        stopped = job.stop_pair()
        if self._restart is not job or job.generation != self._camera_start_request_generation:
            return
        if not all(stopped):
            job.timer = self._schedule(100, lambda: self._retry_restart(job))
            return
        self._restart = None
        self._pending_camera_start = None
        self._camera_start_retry_deadline_s = None
        self._publish()
        job.completed(*pending)

    def cancel_restart(self):
        self._camera_start_request_generation += 1
        self._release(self._restart)
        self._restart = None
        self._pending_camera_start = None
        self._camera_start_retry_deadline_s = None
        self._publish()

    def schedule_stop(self, *, stop_pair, finalize, completed):
        self._release(self._stop)
        self._camera_stop_watchdog_generation += 1
        job = _TimerJob(self._camera_stop_watchdog_generation, stop_pair, completed, finalize=finalize)
        self._stop = job
        self._publish()
        job.timer = self._schedule(250, lambda: self._retry_stop(job))

    def _retry_stop(self, job):
        if self._stop is not job or job.generation != self._camera_stop_watchdog_generation:
            return
        self._release(job)
        stopped = job.stop_pair()
        if self._stop is not job or job.generation != self._camera_stop_watchdog_generation:
            return
        if not all(stopped):
            job.timer = self._schedule(250, lambda: self._retry_stop(job))
            return
        finalized = job.finalize()
        if self._stop is not job or job.generation != self._camera_stop_watchdog_generation:
            return
        if not finalized:
            job.timer = self._schedule(250, lambda: self._retry_stop(job))
            return
        self._stop = None
        job.completed()

    def cancel_stop(self):
        self._camera_stop_watchdog_generation += 1
        self._release(self._stop)
        self._stop = None
        self._publish()

    def resource_inventory(self):
        return {
            "camera_restart_timer": None if self._restart is None else self._restart.timer,
            "camera_stop_watchdog_timer": None if self._stop is None else self._stop.timer,
        }
