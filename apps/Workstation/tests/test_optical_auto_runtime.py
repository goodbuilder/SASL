"""Optical analysis outcomes and shutdown retain the real worker identity."""

from __future__ import annotations

from types import SimpleNamespace
import threading
import time

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.optical_pose.auto_runtime import AutoSelectionTask, OpticalAutoSelectionRuntime
from Workstation.optical_pose_workers import OpticalPoseAutoSelectWorker
from Workstation.tests.test_panel_composition import panels


def _owner(parent=None, *, retired=lambda: None):
    events = []
    owner = OpticalAutoSelectionRuntime(
        parent,
        worker_factory=OpticalPoseAutoSelectWorker,
        completed=lambda command, result: events.append((command, result, QtCore.QThread.currentThread())),
        failed=lambda command, message: events.append(("failed", command, message)),
        retired=retired,
    )
    return owner, events


def _drain(owner):
    deadline = time.monotonic() + 3.0
    while owner.busy and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert not owner.busy


def test_worker_completion_runs_on_gui_and_stale_task_cannot_affect_replacement():
    owner, events = _owner()
    started, release = threading.Event(), threading.Event()

    class Job:
        def run(self):
            return QtCore.QThread.currentThread()

    class BlockingJob:
        def run(self):
            started.set()
            assert release.wait(2.0)
            return "new-result"

    try:
        first = owner.start("first", Job())
        _drain(owner)
        assert len(events) == 1
        assert events[0][0] == "first"
        assert events[0][1] is not owner.thread()
        assert events[0][2] is owner.thread()
        second = owner.start("second", BlockingJob())
        assert started.wait(1.0)
        owner.deliver(first, error="late failure")
        owner.retire(first)
        assert owner.worker_thread is second.thread
        assert len(events) == 1
        with pytest.raises(RuntimeError, match="already running"):
            owner.start("duplicate", Job())
        release.set()
        _drain(owner)
        assert events[1][:2] == ("second", "new-result")
        assert len(events) == 2
    finally:
        release.set()
        if owner.worker_thread is not None:
            owner.worker_thread.quit()
            owner.worker_thread.wait(2000)
            _drain(owner)
        owner.deleteLater()


def test_failed_force_stop_retains_thread_worker_and_inventory_until_exit():
    owner, _events = _owner()
    calls = []

    class Thread:
        running = True

        def isRunning(self):
            return self.running

        def requestInterruption(self):
            calls.append("interrupt")

        def quit(self):
            calls.append("quit")

        def terminate(self):
            calls.append("terminate")

        def wait(self, milliseconds):
            calls.append(("wait", milliseconds))
            return False

    thread = Thread()
    worker = SimpleNamespace(cancel=lambda: calls.append("cancel"))
    task = AutoSelectionTask(1, "fixture", thread, worker, SimpleNamespace(deleteLater=lambda: None))
    owner._task = task
    owner.worker_thread, owner.worker, owner.command_id = thread, worker, "fixture"
    try:
        owner.worker_thread = owner.worker = None  # An old cleanup adapter cannot orphan the live task.
        assert owner.force_stop(wait_ms=500) is False
        assert owner.busy
        assert owner.worker_thread is thread
        assert owner.worker is worker
        assert thread in owner.resource_inventory().values()
        assert calls == ["cancel", "interrupt", "quit", "terminate", ("wait", 500)]
        thread.running = False
        assert owner.force_stop() is True
        assert not owner.busy
        assert all(value is None for value in owner.resource_inventory().values())
    finally:
        owner.deleteLater()


@pytest.mark.parametrize("raise_error,cancel_error", [(True, False), (False, False), (True, True)])
def test_thread_start_failure_releases_reservation_and_allows_retry(monkeypatch, raise_error, cancel_error):
    owner, events = _owner()
    if cancel_error:

        class CancelFailureWorker(OpticalPoseAutoSelectWorker):
            def cancel(self):
                raise RuntimeError("cancel failure")

        owner._worker_factory = CancelFailureWorker

    def cannot_start(thread):
        if raise_error:
            raise RuntimeError("thread resource unavailable")

    try:
        with monkeypatch.context() as patch:
            patch.setattr(QtCore.QThread, "start", cannot_start)
            with pytest.raises(RuntimeError, match="unavailable|did not start"):
                owner.start("failed", SimpleNamespace(run=lambda: "result"))
            assert not owner.busy
            assert owner.worker_thread is None
            assert owner.worker is None
            assert all(value is None for value in owner.resource_inventory().values())
            if cancel_error:
                assert owner.callback_error == "start cleanup: cancel failure"
        owner.start("retry", SimpleNamespace(run=lambda: "retry result"))
        _drain(owner)
        assert events[0][:2] == ("retry", "retry result")
    finally:
        if owner.worker_thread is not None:
            owner.worker_thread.quit()
            owner.worker_thread.wait(2000)
            _drain(owner)
        owner.deleteLater()


def test_required_worker_capability_rejects_without_reserving_handles():
    owner, _events = _owner()
    try:
        with pytest.raises(TypeError, match="executable job"):
            owner.start("missing", object())
        assert not owner.busy
        owner._worker_factory = lambda job: object()
        with pytest.raises(TypeError, match="QObject"):
            owner.start("wrong worker", SimpleNamespace(run=lambda: None))
        assert not owner.busy
    finally:
        owner.deleteLater()


def test_window_waits_when_forced_analysis_stop_has_not_exited():
    from Workstation.main_window import WorkstationMainWindow
    from Workstation.tests.test_workstation_lifecycle import _FakeBackend

    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    owner = window._optical_auto_runtime

    class Thread:
        running = True

        def isRunning(self):
            return self.running

        def requestInterruption(self):
            pass

        def quit(self):
            pass

        def terminate(self):
            pass

        def wait(self, milliseconds):
            return False

    thread = Thread()
    worker = SimpleNamespace(cancel=lambda: None)
    task = AutoSelectionTask(1, "fixture", thread, worker, SimpleNamespace(deleteLater=lambda: None))
    owner._task = task
    owner.worker_thread, owner.worker = thread, worker
    window._close_requested_monotonic_s = time.monotonic() - 10.0
    try:
        event = QtGui.QCloseEvent()
        window.closeEvent(event)
        assert not event.isAccepted()
        assert window._optical_pose_auto_thread is thread
        assert owner.busy
        assert "shutdown" not in backend.calls
    finally:
        thread.running = False
        owner.force_stop()
        window.close()
        QtWidgets.QApplication.instance().processEvents()
        window.deleteLater()
        QtWidgets.QApplication.instance().processEvents()


def test_application_inventory_includes_each_optical_owner_without_name_collision(panels):
    panel = panels()
    first, _ = _owner(panel)
    second, _ = _owner(panel)
    try:
        panel.application.register_optical_auto_runtime(first)
        panel.application.register_optical_auto_runtime(first)
        panel.application.register_optical_auto_runtime(second)
        assert panel.application.optical_auto_runtimes == [first, second]
        first.worker_thread, second.worker_thread = object(), object()
        inventory = panel.application.resource_inventory()
        assert first.worker_thread in inventory.values()
        assert second.worker_thread in inventory.values()
        assert len([name for name in inventory if name.startswith("optical_auto_") and name.endswith("_thread")]) == 2
    finally:
        first.worker_thread = second.worker_thread = None
        first.deleteLater()
        second.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    assert panel.application.optical_auto_runtimes == []
