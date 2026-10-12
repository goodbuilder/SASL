"""Connection job identity, Qt handoff and single-owner handle regression."""

from __future__ import annotations

import time

import pytest
from PyQt5 import QtCore, QtWidgets

from Workstation.device_runtime.connection_jobs import PixhawkConnectionJobs
from Workstation.device_runtime.state import DiagnosticsState


class FixtureWorker(QtCore.QObject):
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, error: str | None = None):
        super().__init__()
        self.error = error

    @QtCore.pyqtSlot()
    def run(self):
        if self.error is None:
            self.completed.emit({"thread": QtCore.QThread.currentThread()})
        else:
            self.failed.emit(self.error)
        self.finished.emit()


def drain(owner, *, timeout_s=3.0):
    application = QtWidgets.QApplication.instance()
    deadline = time.monotonic() + timeout_s
    while any(value is not None for value in owner.resource_inventory().values()):
        application.processEvents()
        assert time.monotonic() < deadline, "connection job did not retire"
        QtCore.QThread.msleep(1)
    application.processEvents()


def prepare(
    owner,
    *,
    kind="connect",
    worker_factory=FixtureWorker,
    completed=lambda result: None,
    failed=lambda error: None,
    finished=lambda: None,
    accepts_result=lambda: True,
):
    return owner.prepare(
        kind,
        worker_factory=worker_factory,
        completed=completed,
        failed=failed,
        finished=finished,
        accepts_result=accepts_result,
    )


@pytest.mark.parametrize("error", [None, "connect failed"])
def test_job_callbacks_return_to_gui_and_handles_have_one_owner(error):
    panel = QtWidgets.QWidget()
    owner = PixhawkConnectionJobs(panel, thread_parent=panel)
    state = DiagnosticsState(owner)
    observed = []
    job = prepare(
        owner,
        worker_factory=lambda: FixtureWorker(error=error),
        completed=lambda result: observed.append(("done", QtCore.QThread.currentThread(), result)),
        failed=lambda value: observed.append(("failed", QtCore.QThread.currentThread(), value)),
        finished=lambda: observed.append(("finished", QtCore.QThread.currentThread())),
    )
    try:
        assert state.pixhawk_connection_thread is job.thread
        assert state.pixhawk_connection_worker is job.worker
        assert "pixhawk_connection_thread" not in vars(state)
        assert job.thread.parent() is panel
        assert owner.resource_inventory()["pixhawk_connection_thread"] is job.thread
        owner.start(job)
        with pytest.raises(RuntimeError, match="already in progress"):
            prepare(owner, kind="disconnect")
        drain(owner)
        assert observed[0][0] == ("done" if error is None else "failed")
        assert observed[-1][0] == "finished"
        assert all(row[1] is QtWidgets.QApplication.instance().thread() for row in observed)
        if error is None:
            assert observed[0][2]["thread"] is not QtWidgets.QApplication.instance().thread()
        assert state.pixhawk_connection_thread is None
        assert state.pixhawk_connection_worker is None
    finally:
        if job.thread is not None:
            job.thread.quit()
            job.thread.wait(2000)
            drain(owner)
        panel.deleteLater()


def test_late_old_result_and_old_retirement_do_not_overwrite_a_new_job():
    owner = PixhawkConnectionJobs(None, thread_parent=None)
    observed = []
    old = prepare(owner, completed=lambda value: observed.append("old"))
    owner.start(old)
    drain(owner)
    assert observed == ["old"]
    current = prepare(owner, completed=lambda value: observed.append(value))
    try:
        thread = current.thread
        owner.deliver(old, result="late-old")
        owner.retire(old)
        assert owner.handle("connect", "thread") is thread
        assert observed == ["old"]
        owner.deliver(current, result="current")
        owner.deliver(current, error="duplicate failure")
        assert observed == ["old", "current"]
        owner.start(current)
        drain(owner)
        assert observed == ["old", "current"]
    finally:
        if current.thread is not None:
            current.thread.quit()
            current.thread.wait(2000)
            drain(owner)
        owner.deleteLater()


def test_replaced_subscriber_result_is_ignored_but_resources_still_retire():
    owner = PixhawkConnectionJobs(None, thread_parent=None)
    current = [object()]
    original = current[0]
    observed = []
    job = prepare(
        owner,
        completed=lambda value: observed.append("unexpected"),
        finished=lambda: observed.append("retired"),
        accepts_result=lambda: current[0] is original,
    )
    try:
        current[0] = object()
        owner.start(job)
        drain(owner)
        assert observed == ["retired"]
        assert all(value is None for value in owner.resource_inventory().values())
    finally:
        if job.thread is not None:
            job.thread.quit()
            job.thread.wait(2000)
            drain(owner)
        owner.deleteLater()


def test_missing_capability_or_factory_failure_does_not_reserve_a_job():
    owner = PixhawkConnectionJobs(None, thread_parent=None)
    try:
        with pytest.raises(TypeError, match="completed"):
            prepare(owner, completed=None)

        def fail_factory():
            raise RuntimeError("factory failed")

        with pytest.raises(RuntimeError, match="factory failed"):
            prepare(owner, worker_factory=fail_factory)
        assert all(value is None for value in owner.resource_inventory().values())
    finally:
        owner.deleteLater()
