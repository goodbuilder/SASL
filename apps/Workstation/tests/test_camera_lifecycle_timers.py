"""Deferred replacement/teardown identity, real Qt cancellation and lifetime."""

import time
import pytest
from PyQt5 import QtCore, QtTest, QtWidgets

from Workstation.camera_runtime.lifecycle_timers import CameraLifecycleTimers, LIFECYCLE_TIMER_FIELDS
from Workstation.tests.test_panel_composition import panels


class ScheduledTimer:
    def __init__(self):
        self.active = True
        self.deleted = False

    def stop(self):
        self.active = False

    def deleteLater(self):
        self.deleted = True


def _queue_owner(monkeypatch, clock):
    owner, queue = CameraLifecycleTimers(now=lambda: clock[0]), []

    def schedule(delay, callback):
        timer = ScheduledTimer()
        queue.append((delay, callback, timer))
        return timer

    monkeypatch.setattr(owner, "_schedule", schedule)
    return owner, queue


def test_repeated_restart_does_not_extend_deadline_and_old_callback_is_inert(monkeypatch):
    clock, events = [100.0], []
    owner, queue = _queue_owner(monkeypatch, clock)

    def start(detection, depth):
        events.append(("start", detection, depth))

    def timeout(pending):
        events.append(("timeout", pending))

    ports = dict(timeout_s=35.0, stop_pair=lambda: (False, True), start=start, timeout=timeout)
    try:
        owner.schedule_restart(False, False, **ports)
        first_delay, old, first_timer = queue.pop(0)
        assert first_delay == 100
        assert owner._camera_start_retry_deadline_s == 135.0
        clock[0] = 120.0
        owner.schedule_restart(True, False, **ports)
        assert first_timer.deleted and not first_timer.active
        assert owner._camera_start_retry_deadline_s == 135.0
        old()
        assert events == []
        assert owner._pending_camera_start == (True, False)
        _delay, current, _timer = queue.pop(0)
        clock[0] = 135.0
        current()
        assert events == [("timeout", (True, False))]
        assert owner._pending_camera_start is None
        assert owner._camera_start_retry_deadline_s is None
        assert owner.resource_inventory()["camera_restart_timer"] is None
    finally:
        owner.cancel_restart()
        owner.deleteLater()


def test_restart_waits_for_both_panes_and_cancel_does_not_cancel_stop_job(monkeypatch):
    owner, queue = _queue_owner(monkeypatch, [1.0])
    stopped, calls = [(True, False), (True, True)], []
    try:
        owner.schedule_restart(
            True,
            True,
            timeout_s=35.0,
            stop_pair=lambda: stopped.pop(0),
            start=lambda *args: calls.append(args),
            timeout=lambda _: calls.append("timeout"),
        )
        queue.pop(0)[1]()
        assert calls == []
        assert queue[0][0] == 100
        queue.pop(0)[1]()
        assert calls == [(True, True)]
        assert owner._pending_camera_start is None
        owner.schedule_stop(
            stop_pair=lambda: (True, True), finalize=lambda: True, completed=lambda: calls.append("stopped")
        )
        owner.schedule_restart(
            False,
            False,
            timeout_s=35.0,
            stop_pair=lambda: (True, True),
            start=lambda *args: calls.append("unexpected"),
            timeout=lambda _: None,
        )
        stop_callback = queue.pop(0)[1]
        stale_restart = queue.pop(0)[1]
        owner.cancel_restart()
        stale_restart()
        stop_callback()
        assert calls == [(True, True), "stopped"]
    finally:
        owner.cancel_restart()
        owner.cancel_stop()
        owner.deleteLater()


def test_stop_waits_for_actual_exit_and_finalization_then_completes_once(monkeypatch):
    owner, queue = _queue_owner(monkeypatch, [1.0])
    pane_results, finalized, done = [(False, True), (True, True), (True, True)], [], []

    def finalize():
        finalized.append(True)
        return len(finalized) == 2

    try:
        owner.schedule_stop(
            stop_pair=lambda: pane_results.pop(0), finalize=finalize, completed=lambda: done.append(True)
        )
        assert queue[0][0] == 250
        queue.pop(0)[1]()
        assert finalized == []
        queue.pop(0)[1]()
        assert finalized == [True] and done == []
        last = queue.pop(0)[1]
        last()
        last()
        assert finalized == [True, True] and done == [True]
        assert owner.resource_inventory()["camera_stop_watchdog_timer"] is None
    finally:
        owner.cancel_stop()
        owner.deleteLater()


def test_real_qt_timer_runs_on_gui_and_parent_destruction_cancels_pending_work():
    parent = QtCore.QObject()
    owner, events = CameraLifecycleTimers(parent), []
    owner.schedule_restart(
        True,
        False,
        timeout_s=35.0,
        stop_pair=lambda: (True, True),
        start=lambda *args: events.append((args, QtCore.QThread.currentThread())),
        timeout=lambda _: None,
    )
    timer = owner.resource_inventory()["camera_restart_timer"]
    assert timer.isActive() and timer.parent() is owner
    deadline = time.monotonic() + 2.0
    while not events and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert events == [((True, False), QtWidgets.QApplication.instance().thread())]
    assert owner.resource_inventory()["camera_restart_timer"] is None
    owner.schedule_restart(
        False,
        False,
        timeout_s=35.0,
        stop_pair=lambda: events.append("unexpected restart") or (True, True),
        start=lambda *args: None,
        timeout=lambda _: None,
    )
    owner.schedule_stop(
        stop_pair=lambda: events.append("unexpected stop") or (True, True),
        finalize=lambda: True,
        completed=lambda: None,
    )
    destroyed = []
    for active in owner.resource_inventory().values():
        active.destroyed.connect(lambda: destroyed.append(True))
    parent.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    QtTest.QTest.qWait(300)
    assert destroyed == [True, True]
    assert len(events) == 1


def test_real_panel_shares_timer_state_and_completed_stop_retires_watchdog(panels):
    panel = panels()
    state = panel.application.features["camera"].state
    owner = state.lifecycle_timers
    stopped = [False]
    panel.left_pane.stop = lambda *, wait=False: stopped[0]
    panel.right_pane.stop = lambda *, wait=False: stopped[0]
    try:
        assert panel.stop_yolo_streams(wait=False) is False
        for name in LIFECYCLE_TIMER_FIELDS:
            assert name not in vars(state)
            assert getattr(panel, name) == getattr(owner, name)
        timer = owner.resource_inventory()["camera_stop_watchdog_timer"]
        assert panel.application.resource_inventory()["camera_stop_watchdog_timer"] is timer
        assert timer.isActive()
        stopped[0] = True
        assert panel.stop_yolo_streams(wait=False) is True
        assert owner.resource_inventory()["camera_stop_watchdog_timer"] is None
        assert not timer.isActive()
    finally:
        stopped[0] = True
        owner.cancel_stop()
        owner.cancel_restart()


@pytest.mark.parametrize("replace", [False, True])
def test_restart_rechecks_identity_after_pane_stop_reenters_owner(monkeypatch, replace):
    owner, queue = _queue_owner(monkeypatch, [10.0])
    calls = []

    def stop_pair():
        owner.cancel_restart()
        if replace:
            owner.schedule_restart(
                False,
                True,
                timeout_s=35.0,
                stop_pair=lambda: (True, True),
                start=lambda *args: calls.append(("new", args)),
                timeout=lambda _: None,
            )
        return True, True

    try:
        owner.schedule_restart(
            True,
            False,
            timeout_s=35.0,
            stop_pair=stop_pair,
            start=lambda *args: calls.append(("old", args)),
            timeout=lambda _: None,
        )
        queue.pop(0)[1]()
        assert calls == []
        assert owner._pending_camera_start == ((False, True) if replace else None)
        if replace:
            queue.pop(0)[1]()
            assert calls == [("new", (False, True))]
    finally:
        owner.cancel_restart()
        owner.deleteLater()


@pytest.mark.parametrize("boundary", ["pane-stop", "finalize"])
@pytest.mark.parametrize("replace", [False, True])
def test_stop_rechecks_identity_after_external_callback_reenters_owner(monkeypatch, boundary, replace):
    owner, queue = _queue_owner(monkeypatch, [10.0])
    calls = []

    def reenter():
        owner.cancel_stop()
        if replace:
            owner.schedule_stop(
                stop_pair=lambda: (True, True), finalize=lambda: True, completed=lambda: calls.append("new")
            )

    def stop_pair():
        if boundary == "pane-stop":
            reenter()
        return True, True

    def finalize():
        if boundary == "finalize":
            reenter()
        return True

    try:
        owner.schedule_stop(stop_pair=stop_pair, finalize=finalize, completed=lambda: calls.append("old"))
        queue.pop(0)[1]()
        assert calls == []
        if replace:
            queue.pop(0)[1]()
            assert calls == ["new"]
    finally:
        owner.cancel_stop()
        owner.deleteLater()
