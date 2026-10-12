"""Real Qt delivery, the shared calibration slot, and ordered retirement.

Payloads are opaque job evidence here; no image, optical RDF, or world-frame
conversion is performed by the owner or these lifetime tests.
"""

import threading
import time

import pytest
from PyQt5 import QtCore, QtWidgets

from Workstation.calibration_workflow.jobs import CALIBRATION_JOB_FIELDS, CalibrationJobs


JOB_KINDS = ("capture", "intrinsics", "extrinsics", "random_validation")


class HeldCalibrationWorker(QtCore.QObject):
    progress = QtCore.pyqtSignal(object)
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, kind, *, duplicate_terminal=False):
        super().__init__()
        self.kind = kind
        self.duplicate_terminal = duplicate_terminal
        self.entered = threading.Event()
        self.release_result = threading.Event()
        self.result_emitted = threading.Event()
        self.release_finish = threading.Event()

    @QtCore.pyqtSlot()
    def run(self):
        evidence = {"kind": self.kind, "worker_thread": QtCore.QThread.currentThread()}
        self.progress.emit(evidence)
        self.entered.set()
        if not self.release_result.wait(5.0):
            self.failed.emit("test result gate timed out")
            self.finished.emit()
            return
        self.completed.emit(evidence)
        if self.duplicate_terminal:
            self.progress.emit({"late": True})
            self.completed.emit({"duplicate": True})
            self.failed.emit("late failure")
        self.result_emitted.set()
        self.release_finish.wait(5.0)
        self.finished.emit()


def _until(predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def _prepare(owner, events, jobs, *, kind="capture", duplicate_terminal=False, completed=None, retired=None):
    def record(event, value):
        events.append((event, value, QtCore.QThread.currentThread()))

    job = owner.prepare(
        kind,
        worker_factory=lambda: HeldCalibrationWorker(kind, duplicate_terminal=duplicate_terminal),
        completed=completed if completed is not None else lambda result: record("completed", result),
        failed=lambda error: record("failed", error),
        progress=lambda value: record("progress", value),
        retired=retired if retired is not None else lambda: record("retired", kind),
    )
    jobs.append(job)
    return job


def _start(owner, job):
    owner.start(job)
    assert job.worker.entered.wait(1.0)


def _release(job):
    job.worker.release_result.set()
    job.worker.release_finish.set()


def _close(owner, jobs):
    for job in jobs:
        _release(job)
    owner.request_shutdown()
    for job in jobs:
        try:
            job.thread.quit()
            assert job.thread.wait(3000), "calibration test thread did not finish"
        except RuntimeError:
            pass  # A normal retirement may already have deleted its QThread.
    QtWidgets.QApplication.instance().processEvents()
    if owner.current is not None and not owner.current.worker.entered.is_set():
        owner.retire(owner.current)  # An assertion may have interrupted prepare before start.
    _until(lambda: not owner.busy)
    owner.deleteLater()
    QtWidgets.QApplication.instance().processEvents()


@pytest.mark.parametrize("kind", JOB_KINDS)
def test_all_job_kinds_share_one_slot_and_deliver_on_gui_thread(kind):
    owner, events, jobs = CalibrationJobs(), [], []
    job = _prepare(owner, events, jobs, kind=kind)
    try:
        _start(owner, job)
        for other_kind in JOB_KINDS:
            with pytest.raises(RuntimeError, match="already running"):
                _prepare(owner, events, jobs, kind=other_kind)
        assert len(jobs) == 1
        _until(lambda: len(events) == 1)
        assert events[0][0] == "progress"
        assert events[0][1]["worker_thread"] is job.thread
        assert events[0][2] is owner.thread()

        job.worker.release_result.set()
        _until(lambda: len(events) == 2)
        assert events[1][0] == "completed"
        assert events[1][1]["kind"] == kind
        assert events[1][1]["worker_thread"] is job.thread
        assert events[1][2] is owner.thread()
        owner.retire(job)
        assert owner.current is job
        assert owner.resource_inventory() == dict(zip(CALIBRATION_JOB_FIELDS, (job.thread, job.worker)))

        job.worker.release_finish.set()
        _until(lambda: not owner.busy)
        assert [event[0] for event in events] == ["progress", "completed", "retired"]
        assert all(value is None for value in owner.resource_inventory().values())
    finally:
        _close(owner, jobs)


def test_duplicate_and_old_events_cannot_complete_or_retire_a_new_job():
    owner, events, jobs = CalibrationJobs(), [], []
    old = _prepare(owner, events, jobs, duplicate_terminal=True)
    try:
        _start(owner, old)
        old.worker.release_result.set()
        assert old.worker.result_emitted.wait(1.0)
        _until(lambda: len(events) >= 2)
        QtWidgets.QApplication.instance().processEvents()
        assert [event[0] for event in events] == ["progress", "completed"]
        assert owner.current is old
        old.worker.release_finish.set()
        _until(lambda: not owner.busy)

        newer = _prepare(owner, events, jobs, kind="intrinsics")
        _start(owner, newer)
        _until(lambda: len(events) == 4)
        before = list(events)
        for event in ("progress", "completed", "failed"):
            owner.deliver(old, event, "late old outcome")
        owner.retire(old)
        owner.retire(old)
        assert events == before
        assert owner.current is newer
        assert not newer.terminal
        assert owner.checkerboard_calibration_thread is newer.thread
        assert owner.checkerboard_calibration_worker is newer.worker
        _release(newer)
        _until(lambda: not owner.busy)
        assert [event[0] for event in events] == [
            "progress",
            "completed",
            "retired",
            "progress",
            "completed",
            "retired",
        ]
    finally:
        _close(owner, jobs)


def test_closing_consumes_queued_results_and_keeps_handles_until_gui_retirement():
    owner, events, jobs, continuations = CalibrationJobs(), [], [], []
    job = _prepare(owner, events, jobs)
    try:
        assert owner.after_current_retired(lambda: continuations.append("old continuation"))
        _start(owner, job)
        assert owner.request_shutdown() is False
        assert owner.current is job
        assert job.thread.isRunning()
        assert not owner.after_current_retired(lambda: continuations.append("closing continuation"))
        with pytest.raises(RuntimeError, match="shutting down"):
            _prepare(owner, events, jobs, kind="extrinsics")

        _release(job)
        assert job.worker.result_emitted.wait(1.0)
        assert job.thread.wait(1000)
        # No GUI events have been processed: a stopped thread can still have
        # saved capture evidence waiting in its queued completed callback.
        assert events == []
        assert owner.request_shutdown() is False
        assert owner.resource_inventory() == dict(zip(CALIBRATION_JOB_FIELDS, (job.thread, job.worker)))
        _until(lambda: not owner.busy)
        assert [event[0] for event in events] == ["progress", "completed", "retired"]
        assert all(event[2] is owner.thread() for event in events)
        assert continuations == []
        assert owner.request_shutdown() is True
    finally:
        _close(owner, jobs)


@pytest.mark.parametrize("raises", [True, False], ids=["start-error", "no-start"])
def test_unstarted_job_cleans_up_and_accepts_a_real_retry(monkeypatch, raises):
    owner, events, jobs = CalibrationJobs(), [], []
    failed = _prepare(owner, events, jobs)

    def unavailable(thread):
        if raises:
            raise RuntimeError("thread unavailable")

    try:
        with monkeypatch.context() as patch:
            patch.setattr(QtCore.QThread, "start", unavailable)
            with pytest.raises(RuntimeError, match="unavailable|did not start"):
                owner.start(failed)
        assert not owner.busy
        assert all(value is None for value in owner.resource_inventory().values())
        assert [event[0] for event in events] == ["failed", "retired"]

        retry = _prepare(owner, events, jobs, kind="random_validation")
        _start(owner, retry)
        _release(retry)
        _until(lambda: not owner.busy)
        assert [event[0] for event in events] == ["failed", "retired", "progress", "completed", "retired"]
    finally:
        _close(owner, jobs)


def test_start_error_after_actual_launch_still_consumes_inflight_capture(monkeypatch):
    owner, events, jobs = CalibrationJobs(), [], []
    job = _prepare(owner, events, jobs)
    original_start = QtCore.QThread.start

    def start_then_raise(thread):
        original_start(thread)
        assert job.worker.entered.wait(1.0)
        raise RuntimeError("start reported failure after launch")

    try:
        with monkeypatch.context() as patch:
            patch.setattr(QtCore.QThread, "start", start_then_raise)
            owner.start(job)
        assert owner.current is job
        assert job.thread.isRunning()
        assert owner.checkerboard_calibration_worker is job.worker
        assert "after launch" in owner.callback_error
        job.worker.release_result.set()
        _until(lambda: any(event[0] == "completed" for event in events))
        assert owner.current is job
        job.worker.release_finish.set()
        _until(lambda: not owner.busy)
        completed = [event for event in events if event[0] == "completed"]
        assert len(completed) == 1
        assert completed[0][1]["kind"] == "capture"
        assert completed[0][2] is owner.thread()
        retry = _prepare(owner, events, jobs, kind="intrinsics")
        _start(owner, retry)
        _release(retry)
        _until(lambda: not owner.busy)
        assert [event[1]["kind"] for event in events if event[0] == "completed"] == ["capture", "intrinsics"]
    finally:
        _close(owner, jobs)


@pytest.mark.parametrize("replacement_retires_in_cleanup", [False, True], ids=["running", "already-retired"])
def test_cleanup_reentry_never_runs_the_old_continuation(replacement_retires_in_cleanup):
    owner, events, jobs, continuations = CalibrationJobs(), [], [], []

    def replace_during_cleanup():
        if owner.closing:
            return
        newer = _prepare(owner, events, jobs, kind="extrinsics")
        if replacement_retires_in_cleanup:
            owner.fail_start(newer, RuntimeError("replacement cancelled before start"))
        else:
            owner.start(newer)

    old = _prepare(owner, events, jobs, retired=replace_during_cleanup)
    try:
        assert owner.after_current_retired(lambda: continuations.append("old continuation"))
        _start(owner, old)
        _release(old)
        _until(lambda: len(jobs) == 2)
        newer = jobs[1]
        assert continuations == []
        if replacement_retires_in_cleanup:
            assert not owner.busy
        else:
            assert owner.current is newer
            assert newer.worker.entered.wait(1.0)
            _release(newer)
            _until(lambda: not owner.busy)
        assert continuations == []
    finally:
        _close(owner, jobs)


def test_nested_retirement_waits_for_completed_callback_to_return():
    owner, events, jobs, order, states = CalibrationJobs(), [], [], [], []

    def completed(result):
        order.append("completed-begin")
        owner.retire(job)
        QtWidgets.QApplication.instance().processEvents()
        states.append((owner.current is job, owner.busy, owner.checkerboard_calibration_worker is job.worker))
        order.append("completed-end")

    job = _prepare(owner, events, jobs, completed=completed, retired=lambda: order.append("retired"))
    try:
        assert owner.after_current_retired(lambda: order.append("continuation"))
        _start(owner, job)
        _release(job)
        job.thread.quit()
        assert job.thread.wait(1000)
        # Both completed and finished are queued while the real thread has
        # already stopped. A nested event loop must not free its slot early.
        _until(lambda: not owner.busy)
        assert states == [(True, True, True)]
        assert order == ["completed-begin", "completed-end", "retired", "continuation"]
        assert owner.callback_error == ""
    finally:
        _close(owner, jobs)
