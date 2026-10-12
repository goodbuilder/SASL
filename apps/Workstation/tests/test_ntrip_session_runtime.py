"""Real Qt admission, reconnect, queued Stop races, and binding-owned auth."""

from dataclasses import replace
import threading
import time
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

import Workstation.gnss_runtime_controller as controller_module
from Workstation.gnss_runtime.credentials import NtripCredentialResolution
from Workstation.gnss_runtime.ntrip import NTRIP_FIELDS, NtripSessionRuntime, NtripSessionState
from Workstation.gnss_runtime_workers import NtripNetworkWorker
from Workstation.tests.test_panel_composition import panels
from Workstation.workstation_core import ntrip_credentials
from Workstation.workstation_core.gnss_operation import GnssOperationConfig, GnssOperationMode, NetworkConfig
from Workstation.workstation_core.rtcm_pipeline import NtripSessionStats


class ControlledWorker(NtripNetworkWorker):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.entered = threading.Event()
        self.release = threading.Event()
        self.stop_called_on = None

    @QtCore.pyqtSlot()
    def run(self):
        self.connected.emit("HTTP/1.1 200 OK", QtCore.QThread.currentThread())
        self.entered.set()
        self.release.wait(3.0)
        self.finished.emit()

    def request_stop(self):
        self.stop_called_on = QtCore.QThread.currentThread()
        self.release.set()


def _until(predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def _new_runtime():
    events, saved = [], []

    def connected(status, stats):
        saved_credential = owner.take_accepted_credential(status)
        if saved_credential is not None:
            saved.append(saved_credential)
        events.append(("connected", stats, QtCore.QThread.currentThread()))

    owner = NtripSessionRuntime(
        None,
        NtripSessionState(),
        worker_factory=ControlledWorker,
        connected=connected,
        reconnecting=lambda message: events.append(("retry", message)),
        failed=lambda message: events.append(("failed", message)),
        diagnostic=lambda message: events.append(("diagnostic", message)),
        retired=lambda: events.append(("retired",)),
    )
    return owner, events, saved


def _start(owner, *, host="first"):
    config = NetworkConfig(caster_host=host, mountpoint="MOUNT", username="user", remember_password=True)
    credential = NtripCredentialResolution("user", "fixture-password", "session")
    session = owner.start(config, credential, lambda: None, wire_data_plane=lambda worker, token: None)
    assert session.worker.entered.wait(1.0)
    return session


def _close(owner):
    owner.request_stop()
    _until(lambda: not owner.busy)
    owner.deleteLater()


def test_stop_before_gui_delivery_drops_queued_control_events_and_auth():
    owner, events, saved = _new_runtime()
    try:
        session = _start(owner)
        session.worker.failed.emit("late 401")
        session.worker.diagnostic.emit("late log")
        session.worker.reconnecting.emit("late retry")
        assert owner.request_stop() is False
        assert session.worker.stop_called_on is owner.thread()  # direct, thread-safe stop
        assert owner.state._gnss_ntrip_pending_credential is None
        assert owner.resource_inventory()["gnss_network_thread"] is session.thread
        _until(lambda: not owner.busy)
        assert events == [("retired",)]
        assert saved == []
    finally:
        _close(owner)


def test_same_session_reconnects_and_old_events_cannot_touch_new_session():
    owner, events, saved = _new_runtime()
    try:
        old = _start(owner)
        _until(lambda: len(events) == 1)
        assert events[0][1] is not owner.thread()
        assert events[0][2] is owner.thread()
        old.worker.reconnecting.emit("temporary drop")
        old.worker.connected.emit("ICY 200 OK", "new stats")
        _until(lambda: len(events) == 3)
        assert [row[0] for row in events] == ["connected", "retry", "connected"]
        assert len(saved) == 1
        owner.retire(old)  # Cannot release a still-running thread.
        assert owner.busy
        with pytest.raises(RuntimeError, match="already running"):
            _start(owner)
        owner.request_stop()
        _until(lambda: not owner.busy)
        newer = _start(owner, host="second")
        before = len(events)
        owner.deliver(old, "connected", "HTTP/1.1 200 OK", "stale")
        owner.deliver(old, "failed", "old failure")
        owner.retire(old)
        assert len(events) == before
        assert owner.state.gnss_network_thread is newer.thread
        assert owner.state._gnss_ntrip_pending_credential[0].caster_host == "second"
        _until(lambda: len(saved) == 2)
        assert saved[1][0].caster_host == "second"
    finally:
        _close(owner)


def test_terminal_failure_blocks_duplicate_or_late_success_but_keeps_handles():
    owner, events, saved = _new_runtime()
    try:
        session = _start(owner)
        owner.deliver(session, "failed", "401 Unauthorized")
        owner.deliver(session, "failed", "duplicate")
        QtWidgets.QApplication.instance().processEvents()
        assert events == [("failed", "401 Unauthorized")]
        assert owner.busy
        assert saved == []
        assert owner.state._gnss_ntrip_pending_credential is None
    finally:
        _close(owner)


@pytest.mark.parametrize("failure", ["factory", "wire", "start", "no-start", "start-and-stop-error"])
def test_start_failures_release_resources_and_allow_retry(monkeypatch, failure):
    owner, events, saved = _new_runtime()
    network = NetworkConfig(caster_host="caster", mountpoint="MOUNT")
    credential = NtripCredentialResolution("user", "fixture", "session")

    def fail(*args, **kwargs):
        raise RuntimeError("injected start failure")

    try:
        with monkeypatch.context() as patch:
            if failure == "factory":
                patch.setattr(owner, "_worker_factory", fail)
            if failure in {"start", "start-and-stop-error"}:
                patch.setattr(QtCore.QThread, "start", fail)
            if failure == "start-and-stop-error":
                patch.setattr(ControlledWorker, "request_stop", fail)
            if failure == "no-start":
                patch.setattr(QtCore.QThread, "start", lambda self: None)
            with pytest.raises(RuntimeError, match="failure|did not start"):
                owner.start(
                    network,
                    credential,
                    lambda: None,
                    wire_data_plane=fail if failure == "wire" else lambda worker, token: None,
                )
            assert not owner.busy
            assert all(value is None for value in owner.resource_inventory().values())
        _start(owner)
        _until(lambda: len(saved) == 1)
    finally:
        _close(owner)


def _prepare_panel(panel, monkeypatch, *, worker_type=ControlledWorker):
    controller = panel._gnss_runtime_controller_owner
    config = GnssOperationConfig(
        operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT,
        network=NetworkConfig(
            caster_host="fixture.invalid", mountpoint="MOUNT", username="fixture", remember_password=True
        ),
    )
    monkeypatch.setattr(panel, "_build_gnss_operation_config", lambda: config)
    monkeypatch.setattr(panel, "_gnss_selected_operation_mode", lambda: config.operation_mode)
    monkeypatch.setattr(controller, "_prepare_ntrip_rtcm_route", lambda route: True)
    monkeypatch.setattr(controller_module, "NtripNetworkWorker", worker_type)
    panel.set_gnss_ntrip_session_password(
        "fixture-password", caster_host="fixture.invalid", mountpoint="MOUNT", username="fixture"
    )
    saved = []
    monkeypatch.setattr(ntrip_credentials, "save_password", lambda *args: saved.append(args) or True)
    return controller, saved


def test_real_panel_start_stop_preserves_stopped_state_and_raw_data_plane(panels, monkeypatch):
    panel = panels()
    controller, saved = _prepare_panel(panel, monkeypatch)
    panel.start_gnss_network()
    owner = controller._ntrip_owner
    session = owner.state.current
    try:
        assert session.worker.entered.wait(1.0)
        state = panel.application.features["gnss"].state
        assert owner.state is state.ntrip
        for name in NTRIP_FIELDS:
            assert name not in vars(state)
            assert name not in vars(panel)
        assert panel.application.resource_inventory()["gnss_network_thread"] is session.thread
        assert session.worker.receivers(session.worker.chunk_received) == 0
        assert session.worker.receivers(session.worker.stats_changed) == 0
        assert session.worker.receivers(session.worker.chunk_envelope_received) == 1
        assert panel.stop_gnss_network() in (False, True)
        # Exported compatibility callbacks have no binding token. They must
        # not bypass the relay after a real owner has been installed.
        panel._handle_gnss_network_connected("HTTP/1.1 200 OK", "old stats")
        panel._handle_gnss_network_failed("old 401")
        panel._handle_gnss_network_reconnecting("old retry")
        panel._handle_gnss_network_diagnostic("old diagnostic")
        panel._handle_gnss_network_stats("old snapshot")
        pipeline = panel.rtcm_pipeline_worker
        _until(lambda: not owner.busy)
        assert panel.gnss_network_phase == "STOPPED"
        assert panel.gnss_network_connected is False
        assert panel._gnss_ntrip_pending_credential is None
        assert panel.rtcm_pipeline_worker is pipeline
        assert saved == []
        assert panel.gnss_network_start_button.isEnabled()
        assert not panel.gnss_network_stop_button.isEnabled()
    finally:
        _close(owner)


def test_real_panel_factory_failure_unlocks_start_for_retry(panels, monkeypatch):
    panel = panels()

    def unavailable(**kwargs):
        raise RuntimeError("worker factory unavailable")

    controller, saved = _prepare_panel(panel, monkeypatch, worker_type=unavailable)
    panel.start_gnss_network()
    assert panel.gnss_network_phase == "ERROR"
    assert panel.gnss_network_thread is None
    assert panel.gnss_active_operation_config is None
    assert panel.gnss_operation_gate_active is False
    assert panel.gnss_network_start_button.isEnabled()
    assert not panel.gnss_network_stop_button.isEnabled()
    assert saved == []
    assert not controller._ntrip_owner.busy


def test_coalesced_stats_carry_binding_through_real_pipeline_and_reject_stale(panels, monkeypatch):
    panel = panels()
    controller, saved = _prepare_panel(panel, monkeypatch)
    panel.start_gnss_network()
    owner = controller._ntrip_owner
    old = owner.state.current
    snapshots = []
    pipeline = panel.rtcm_pipeline_worker
    pipeline.snapshot_ready.connect(snapshots.append, QtCore.Qt.QueuedConnection)
    received = []
    pipeline.ntrip_stats_received.connect(received.append, QtCore.Qt.QueuedConnection)
    try:
        _until(lambda: panel.gnss_network_connected)
        stats = SimpleNamespace(last_chunk_monotonic_s=10.0, bytes_received=123)
        old.worker.stats_snapshot_changed.emit(stats)
        _until(lambda: bool(received))
        # Force the existing coalesced publisher without changing its cadence.
        pipeline.configure_requested.emit({})
        _until(lambda: any(isinstance(row.get("ntrip_stats"), NtripSessionStats) for row in snapshots))
        previous = next(row for row in snapshots if isinstance(row.get("ntrip_stats"), NtripSessionStats))
        assert previous["ntrip_stats"].session_token == old.token
        assert previous["ntrip_stats"].stats is stats
        _until(lambda: panel.gnss_network_stats is stats)
        assert panel.gnss_network_phase == "STREAMING"
        panel.stop_gnss_network()
        _until(lambda: not owner.busy)
        panel.start_gnss_network()
        newer = owner.state.current
        assert newer.token != old.token
        assert newer.worker.entered.wait(1.0)
        # Neither the tagged old snapshot nor the untagged compatibility
        # callbacks may validate the newly captured session credential.
        pending = panel._gnss_ntrip_pending_credential
        controller._apply_rtcm_pipeline_snapshot(previous)
        panel._handle_gnss_network_connected("HTTP/1.1 200 OK", stats)
        panel._handle_gnss_network_failed("old 401")
        assert panel.gnss_network_phase == "CONNECTING"
        assert panel.gnss_network_stats is None
        assert panel._gnss_ntrip_pending_credential is pending
        assert len(saved) == 1
        _until(lambda: panel.gnss_network_connected)
        assert len(saved) == 2
        current_stats = panel.gnss_network_stats
        controller._apply_rtcm_pipeline_snapshot(previous)
        assert panel.gnss_network_stats is current_stats
    finally:
        panel.stop_gnss_network()
        _until(lambda: not owner.busy)
