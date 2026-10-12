"""Own one optical auto-selection job and its Qt delivery/retirement identity.

The admitted job already owns frozen native inputs. This owner never reads a
window, panel, live profile or frame; GUI/backend effects use three callbacks.
Cancellation preserves the worker's queued-only contract. A force-stop request
does not release handles until the thread has actually stopped.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
from PyQt5 import QtCore


@dataclass
class AutoSelectionTask:
    token: int
    command_id: str
    thread: object
    worker: object
    relay: object = None
    terminal: bool = False


class _AutoSelectionRelay(QtCore.QObject):
    def __init__(self, owner, task):
        super().__init__(owner)
        self.owner, self.task = owner, task

    @QtCore.pyqtSlot(object)
    def completed(self, result):
        self.owner.deliver(self.task, result=result)

    @QtCore.pyqtSlot(str)
    def failed(self, message):
        self.owner.deliver(self.task, error=message)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.task)


class OpticalAutoSelectionRuntime(QtCore.QObject):
    def __init__(self, parent, *, worker_factory: Callable, completed: Callable, failed: Callable, retired: Callable):
        super().__init__(parent)
        for name, callback in (
            ("worker_factory", worker_factory),
            ("completed", completed),
            ("failed", failed),
            ("retired", retired),
        ):
            if not callable(callback):
                raise TypeError(f"Missing optical auto-selection operation: {name}")
        self._thread_parent = parent
        self._worker_factory = worker_factory
        self._completed = completed
        self._failed = failed
        self._retired = retired
        self._task = None
        self._next_token = 0
        self._closing = False
        self.worker_thread = None
        self.worker = None
        self.command_id = None
        self.callback_error = ""

    @property
    def busy(self) -> bool:
        return self._task is not None or self.worker_thread is not None

    def start(self, command_id: str, frozen_job) -> AutoSelectionTask:
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Optical jobs must start on the Qt owner thread")
        if self._closing:
            raise RuntimeError("Optical auto-selection is shutting down")
        if self.busy:
            raise RuntimeError("Optical auto-selection is already running")
        if not callable(getattr(frozen_job, "run", None)):
            raise TypeError("Optical auto-selection requires an admitted executable job")
        worker = self._worker_factory(frozen_job)
        if not isinstance(worker, QtCore.QObject):
            raise TypeError("Optical auto-selection worker factory must return a QObject")
        for operation in ("run", "cancel"):
            if not callable(getattr(worker, operation, None)):
                worker.deleteLater()
                raise TypeError(f"Missing optical auto-selection worker operation: {operation}")
        thread = QtCore.QThread(self._thread_parent)
        self._next_token += 1
        task = AutoSelectionTask(self._next_token, str(command_id), thread, worker)
        relay = _AutoSelectionRelay(self, task)
        task.relay = relay
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
        self._task = task
        self.worker_thread, self.worker, self.command_id = thread, worker, task.command_id
        try:
            thread.start()
            if not thread.isRunning():
                raise RuntimeError("Optical auto-selection thread did not start")
        except Exception:
            task.terminal = True
            try:
                worker.cancel()
            except Exception as exc:
                self.callback_error = f"start cleanup: {exc}"
            thread.quit()
            if not thread.isRunning():
                self.retire(task)
                worker.deleteLater()
                thread.deleteLater()
            raise
        return task

    def deliver(self, task, *, result=None, error=None):
        if self._task is not task or task.terminal:
            return
        task.terminal = True
        try:
            if error is None:
                self._completed(task.command_id, result)
            else:
                self._failed(task.command_id, str(error))
        except Exception as exc:
            self.callback_error = str(exc)
            try:
                self._failed(task.command_id, str(exc))
            except Exception as failure:
                self.callback_error = f"{exc}; failure presentation: {failure}"

    def retire(self, task):
        if self._task is not task:
            return
        self._task = None
        self.worker_thread = None
        self.worker = None
        self.command_id = None
        try:
            self._retired()
        except Exception as exc:
            self.callback_error = str(exc)
        finally:
            if task.relay is not None:
                task.relay.deleteLater()

    def clear_finished_legacy(self, thread):
        task = self._task
        if task is None or task.thread is not thread:
            return
        try:
            if thread.isRunning():
                return
        except RuntimeError:
            pass
        self.retire(task)

    def request_stop(self) -> bool:
        self._closing = True
        if self._task is not None:
            # The task remains authoritative if legacy presentation handles
            # were cleared before the Qt worker actually retired.
            self.worker_thread = self._task.thread
            self.worker = self._task.worker
            self.command_id = self._task.command_id
        worker = self.worker
        if worker is not None:
            try:
                worker.cancel()
            except RuntimeError:
                pass
        return not self.busy

    def force_stop(self, *, wait_ms: int = 500) -> bool:
        self.request_stop()
        task = self._task
        thread = self.worker_thread
        if thread is None:
            return task is None
        try:
            if thread.isRunning():
                thread.requestInterruption()
                thread.quit()
                thread.terminate()
                if not thread.wait(wait_ms) or thread.isRunning():
                    return False
        except RuntimeError:
            try:
                if thread.isRunning():
                    return False
            except RuntimeError:
                # Qt already destroyed a finished thread.
                pass
        if task is not None:
            task.terminal = True
            self.retire(task)
        else:
            self.worker_thread = None
            self.worker = None
            self.command_id = None
        return True

    def resource_inventory(self):
        prefix = f"optical_auto_{id(self)}"
        task = self._task
        return {
            f"{prefix}_thread": task.thread if task is not None else self.worker_thread,
            f"{prefix}_worker": task.worker if task is not None else self.worker,
        }
