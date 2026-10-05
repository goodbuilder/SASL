from __future__ import annotations

import time
from types import SimpleNamespace

from PyQt5 import QtCore, QtWidgets

from Workstation.optical_pose_workers import OpticalPoseAutoSelectWorker


def test_auto_select_worker_runs_job_off_the_gui_thread() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    gui_thread = QtCore.QThread.currentThread()
    calls: list[QtCore.QThread] = []

    def run_job() -> dict[str, object]:
        calls.append(QtCore.QThread.currentThread())
        time.sleep(0.03)
        return {"state": "LOW_CONFIDENCE", "mode": "mountain"}

    worker = OpticalPoseAutoSelectWorker(SimpleNamespace(run=run_job))
    thread = QtCore.QThread()
    worker.moveToThread(thread)
    completed: list[dict[str, object]] = []
    failures: list[str] = []
    worker.completed.connect(completed.append)
    worker.failed.connect(failures.append)
    worker.finished.connect(thread.quit)
    worker.finished.connect(worker.deleteLater)
    thread.finished.connect(thread.deleteLater)
    thread.started.connect(worker.run)
    thread.start()

    deadline = time.monotonic() + 2.0
    while thread.isRunning() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    thread.wait(1000)
    app.processEvents()

    assert not failures
    assert completed == [{"state": "LOW_CONFIDENCE", "mode": "mountain"}]
    assert calls == [thread]
    assert calls[0] is not gui_thread


def test_auto_select_worker_cancel_skips_queued_job() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls: list[str] = []

    worker = OpticalPoseAutoSelectWorker(
        SimpleNamespace(run=lambda: calls.append("ran") or {"state": "GOOD"})
    )
    worker.cancel()
    worker.run()
    app.processEvents()

    assert calls == []
