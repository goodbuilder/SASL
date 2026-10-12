"""Side-scoped command lifetime and the real panel readback handoff."""

import threading
import time
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

import Workstation.device_runtime_controller as controller_module
from Workstation.app_lifecycle_controller import AppLifecycleController
from Workstation.device_runtime.maintenance_jobs import MAINTENANCE_FIELDS, PixhawkMaintenanceJobs
from Workstation.tests.test_panel_composition import panels


class HeldWorker(QtCore.QObject):
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, subscriber=None, **inputs):
        super().__init__()
        self.subscriber = subscriber
        self.inputs = inputs
        self.entered, self.release = threading.Event(), threading.Event()

    @QtCore.pyqtSlot()
    def run(self):
        self.entered.set()
        self.release.wait(3.0)
        self.completed.emit(
            {
                "camera": self.inputs.get("camera", "left"),
                "kind": "board_rotation_refresh",
                "params": {"SENS_BOARD_ROT": 4},
                "worker_thread": QtCore.QThread.currentThread(),
            }
        )
        self.finished.emit()


def _until(predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def _prepare(owner, events, *, kind="shell", camera="left", accepts=lambda: True, retired=lambda: None):
    return owner.prepare(
        kind,
        camera,
        worker_factory=HeldWorker,
        completed=lambda result: events.append((result, QtCore.QThread.currentThread())),
        failed=lambda error: events.append(error),
        retired=retired,
        accepts_result=accepts,
    )


def _cleanup(owner):
    for job in list(owner._jobs.values()):
        job.worker.release.set()
    owner.stop(wait=False)
    _until(lambda: not owner._jobs)
    owner.deleteLater()


def test_distinct_side_and_kind_jobs_keep_existing_concurrency_and_gui_affinity():
    owner, events = PixhawkMaintenanceJobs(), []
    jobs = [_prepare(owner, events), _prepare(owner, events, kind="parameter"), _prepare(owner, events, camera="right")]
    try:
        for job in jobs:
            owner.start(job)
            assert job.worker.entered.wait(1.0)
        with pytest.raises(RuntimeError, match="already running"):
            _prepare(owner, events)
        first = jobs[0]
        owner.retire(first)
        assert owner.busy("shell", "left")
        first.worker.release.set()
        _until(lambda: not owner.busy("shell", "left"))
        assert owner.busy("parameter", "left")
        assert owner.busy("shell", "right")
        assert events[0][0]["worker_thread"] is not owner.thread()
        assert events[0][1] is owner.thread()
        newer = _prepare(owner, events)
        owner.deliver(first, error="old failure")
        owner.retire(first)
        assert owner.pixhawk_shell_threads["left"] is newer.thread
        assert len(events) == 1
        owner.start(newer)
    finally:
        _cleanup(owner)


@pytest.mark.parametrize("cleared", [("threads",), ("workers",), ("threads", "workers")])
def test_stop_rejects_queued_results_and_keeps_handles_even_after_legacy_clear(cleared):
    owner, events = PixhawkMaintenanceJobs(), []
    job = _prepare(owner, events)
    try:
        owner.start(job)
        assert job.worker.entered.wait(1.0)
        job.worker.completed.emit({"queued": True})
        for name in cleared:
            getattr(owner, "pixhawk_shell_" + name).clear()
        assert job.thread in owner.resource_inventory().values()
        assert job.worker in owner.resource_inventory().values()
        assert owner.stop(wait=False) is False
        assert owner.busy("shell", "left")
        job.worker.release.set()
        _until(lambda: not owner.busy("shell", "left"))
        assert events == []
    finally:
        _cleanup(owner)


def test_stop_covers_both_real_job_and_replaced_legacy_handle():
    owner, events = PixhawkMaintenanceJobs(), []
    job = _prepare(owner, events)
    replacement = QtCore.QThread()
    try:
        owner.start(job)
        assert job.worker.entered.wait(1.0)
        replacement.start()
        owner.pixhawk_shell_threads["left"] = replacement
        owner.pixhawk_shell_workers["left"] = object()
        assert owner.stop(wait=False) is False
        assert job.thread.isInterruptionRequested()
        _until(lambda: not replacement.isRunning())
        job.worker.release.set()
        _until(lambda: not owner._jobs)
        assert owner.stop(wait=False) is True
        assert owner.pixhawk_shell_threads == {}
        assert owner.pixhawk_shell_workers == {}
        assert events == []
    finally:
        replacement.quit()
        replacement.wait(1000)
        _cleanup(owner)
        replacement.deleteLater()


def test_stale_link_or_duplicate_terminal_is_ignored_and_callback_errors_retire():
    owner, events, retired = PixhawkMaintenanceJobs(), [], []
    current = [True]
    job = _prepare(owner, events, accepts=lambda: current[0], retired=lambda: retired.append(True))
    try:
        owner.start(job)
        assert job.worker.entered.wait(1.0)
        current[0] = False
        owner.deliver(job, result="old link")
        current[0] = True
        owner.deliver(job, result="duplicate")
        job.worker.release.set()
        _until(lambda: not owner.busy("shell", "left"))
        assert events == []
        assert retired == [True]
        next_job = _prepare(owner, events, retired=lambda: (_ for _ in ()).throw(RuntimeError("view closed")))
        owner.start(next_job)
        next_job.worker.release.set()
        _until(lambda: not owner.busy("shell", "left"))
        assert owner.callback_error == "view closed"
    finally:
        _cleanup(owner)


@pytest.mark.parametrize("raise_error", [True, False])
def test_failed_thread_start_releases_only_its_lane_and_allows_retry(monkeypatch, raise_error):
    owner, events = PixhawkMaintenanceJobs(), []
    survivor = _prepare(owner, events, camera="right")
    owner.start(survivor)
    try:
        job = _prepare(owner, events)

        def fail_start(thread):
            if raise_error:
                raise RuntimeError("thread unavailable")

        with monkeypatch.context() as patch:
            patch.setattr(QtCore.QThread, "start", fail_start)
            with pytest.raises(RuntimeError, match="unavailable|did not start"):
                owner.start(job)
        assert not owner.busy("shell", "left")
        assert owner.busy("shell", "right")
        retry = _prepare(owner, events)
        owner.start(retry)
        assert retry.worker.entered.wait(1.0)
    finally:
        _cleanup(owner)


@pytest.mark.parametrize("change", ["subscriber", "connection", "generation", "unchanged"])
def test_real_panel_param_readback_is_bound_to_admitted_connection(panels, monkeypatch, change):
    panel = panels()
    old_subscriber = panel.mavlink_pixhawk
    generation = [7]
    subscriber = SimpleNamespace(
        connections={"left": object()},
        mavlink_health_snapshot=lambda side: SimpleNamespace(connection_generation=generation[0]),
    )
    panel.mavlink_pixhawk = subscriber
    monkeypatch.setattr(panel, "_mavlink_open_sides", lambda: {"left"})
    monkeypatch.setattr(controller_module, "PixhawkParameterWorker", HeldWorker)
    owner = panel.application.features["diagnostics"].state.maintenance_jobs
    try:
        assert panel._start_pixhawk_parameter_worker("left", "board_rotation_refresh") is True
        job = owner._jobs[("parameter", "left")]
        assert job.worker.entered.wait(1.0)
        state = panel.application.features["diagnostics"].state
        for name in MAINTENANCE_FIELDS:
            assert name not in vars(state)
            assert getattr(panel, name) is getattr(owner, name)
        assert panel.application.resource_inventory()["pixhawk_parameter_threads"]["left"] is job.thread
        snapshot = AppLifecycleController(panel)._shutdown_resource_snapshot()
        assert snapshot["complete"] is False
        if change == "subscriber":
            panel.mavlink_pixhawk = SimpleNamespace(connections=subscriber.connections)
        elif change == "connection":
            subscriber.connections["left"] = object()
        elif change == "generation":
            generation[0] += 1
        # The untagged public adapter cannot inject even a plausible readback.
        panel._apply_pixhawk_parameter_worker_result(
            {"kind": "board_rotation_refresh", "camera": "left", "params": {"SENS_BOARD_ROT": 99}}
        )
        assert "left" not in panel._px4_board_rotation_params
        job.worker.release.set()
        _until(lambda: not owner.busy("parameter", "left"))
        expected = {"SENS_BOARD_ROT": 4} if change == "unchanged" else None
        assert panel._px4_board_rotation_params.get("left") == expected
        assert panel.pixhawk_parameter_threads == {}
    finally:
        for job in list(owner._jobs.values()):
            job.worker.release.set()
        owner.stop(wait=False)
        _until(lambda: not owner._jobs)
        panel.mavlink_pixhawk = old_subscriber


def test_real_panel_shell_captures_command_list_and_start_failure_restores_param_actions(panels, monkeypatch):
    panel = panels()
    original = panel.mavlink_pixhawk
    panel.mavlink_pixhawk = SimpleNamespace(connections={"left": object()})
    monkeypatch.setattr(panel, "_mavlink_open_sides", lambda: {"left"})
    monkeypatch.setattr(controller_module, "PixhawkShellCommandWorker", HeldWorker)
    monkeypatch.setattr(controller_module, "PixhawkParameterWorker", HeldWorker)
    controls = []
    monkeypatch.setattr(panel, "_set_px4_mag_manager_button_state", lambda camera, **values: controls.append(values))
    owner = panel.application.features["diagnostics"].state.maintenance_jobs
    try:
        commands = ["listener sensor_mag -i 0 -n 1"]
        assert panel._start_pixhawk_shell_worker("left", commands)
        job = owner._jobs[("shell", "left")]
        commands.append("not admitted")
        assert job.worker.inputs["command"] == ("listener sensor_mag -i 0 -n 1",)
        with monkeypatch.context() as patch:
            patch.setattr(QtCore.QThread, "start", lambda thread: None)
            assert panel._start_pixhawk_parameter_worker("left", "attitude_refresh") is False
        assert not owner.busy("parameter", "left")
        assert controls[-1]["refresh_enabled"] is True
        assert controls[-1]["actions_enabled"] is True
        assert owner.busy("shell", "left")
    finally:
        for job in list(owner._jobs.values()):
            job.worker.release.set()
        owner.stop(wait=False)
        _until(lambda: not owner._jobs)
        panel.mavlink_pixhawk = original
