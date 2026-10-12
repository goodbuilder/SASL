from __future__ import annotations

from types import SimpleNamespace
import json
import time

from PyQt5 import QtCore, QtWidgets

from Workstation.method_worker_pane import MethodWorkerRecoveryPane
from Workstation.standalone_backend import StandalonePanelBackend
from Workstation.measurement_view_controller import MeasurementViewController
from Workstation.workstation_core.method_evaluation_workers import MethodEvaluationPoll


def row(**changes):
    return {"method_id": "pixhawk_rtk_live", "state": "RUNNING", "worker_generation": 3,
            "pair_id": "old-run:pair-1", "measurement_id": "old-run", "request_generation": 4,
            "elapsed_s": 10.0, "pending_jobs": 2, "restart_pending": False, **changes}


def test_recovery_click_uses_displayed_identity_and_waits_for_observed_completion():
    rows = [row()]
    calls = []

    def restart(payload):
        calls.append(payload)
        rows[0]["restart_pending"] = True
        return {"accepted": True, "reason": "queued"}

    pane = MethodWorkerRecoveryPane(lambda: {"state": "OPEN", "workers": rows}, restart)
    pane.resize(900, 100)
    pane.show()
    QtWidgets.QApplication.processEvents()
    assert pane.restart_button.isEnabled()
    pane.restart_button.click()
    assert calls == [{"method_id": "pixhawk_rtk_live", "expected_worker_generation": 3,
                      "expected_pair_id": "old-run:pair-1", "expected_request_generation": 4}]
    assert not pane.restart_button.isEnabled()
    assert "완료" not in pane.feedback_label.toolTip()
    rows[0] = row(state="IDLE", pair_id="", restart_pending=False, worker_generation=4,
                  last_restart={"status": "RESTARTED", "reason": "replacement observed"})
    pane.refresh()
    assert "재시작 완료" in pane.feedback_label.toolTip()
    pane.close()


def test_status_updates_preserve_geometry_selection_focus_and_handle_failure():
    rows = [row(), row(method_id="horizon_hybrid")]
    state = {"state": "OPEN", "workers": rows}
    pane = MethodWorkerRecoveryPane(lambda: state, lambda payload: {"accepted": False, "reason": "stale identity"})
    pane.resize(900, 100)
    pane.show()
    pane.method_combo.setCurrentIndex(pane.method_combo.findData("horizon_hybrid"))
    pane.method_combo.setFocus()
    QtWidgets.QApplication.processEvents()
    before = pane.restart_button.geometry()
    for seconds in (9.99, 10.0, 3600.0, 999999999.0):
        rows[1].update(elapsed_s=seconds, pair_id="long-pair-" * 80)
        pane.refresh()
        QtWidgets.QApplication.processEvents()
        assert pane.restart_button.geometry() == before
        assert pane.method_combo.currentData() == "horizon_hybrid"
        assert pane.method_combo.hasFocus()
    pane.restart_button.click()
    assert "재시작 거부" in pane.feedback_label.toolTip()
    state["state"] = "CLOSING"
    pane.refresh()
    assert not pane.restart_button.isEnabled()
    state["workers"] = "malformed"
    pane.refresh()
    assert not pane.restart_button.isEnabled()
    pane.close()


def test_adapter_records_observed_recovery_under_original_run_and_tolerates_storage_failure():
    events = []
    panel = SimpleNamespace(
        _measurement_run_generation="new-run", _log=lambda text: None,
        append_workstation_data=lambda stream, payload, **kw: events.append((stream, payload)) or False,
    )
    controller = MeasurementViewController(panel)
    panel._measurement_view_controller_owner = controller
    backend = StandalonePanelBackend(panel)
    assert backend.measurement_worker_snapshot()["state"] == "NOT_STARTED"
    event = {"event_type": "method_worker_restart_result", "measurement_id": "old-run",
             "pair_id": "pair-1", "method_id": "horizon_hybrid", "status": "RESTARTED"}
    coordinator = SimpleNamespace(
        poll=lambda: MethodEvaluationPoll(recovery_events=(event,)),
        status_snapshot=lambda: {"state": "OPEN", "workers": [row()]},
        restart_method=lambda method, **kw: {"accepted": True, "reason": "queued"},
    )
    controller._method_evaluation_coordinator = coordinator
    assert backend.measurement_worker_snapshot()["workers"][0]["worker_generation"] == 3
    assert backend.restart_measurement_method_worker({"method_id": "pixhawk_rtk_live",
        "expected_worker_generation": 3, "expected_pair_id": "old-run:pair-1",
        "expected_request_generation": 4})["accepted"]
    controller._poll_method_evaluations()
    assert events == [("events", event)]
    assert panel._measurement_run_generation == "new-run"


def test_real_recovery_click_keeps_gui_live_and_persists_old_run_evidence(tmp_path):
    from Workstation.tests.test_method_worker_recovery import (
        PRIMARY, COMPARISON, request, stalled_worker,
    )
    from Workstation.workstation_core.method_evaluation_workers import MethodEvaluationCoordinator
    from Workstation.workstation_core.measurement_artifacts import reserve_measurement_context
    from Workstation.workstation_core.measurement_data_store import MeasurementDataStore

    store = MeasurementDataStore(tmp_path)
    old = reserve_measurement_context(tmp_path, "recovery-run", 1_000_000_000, 1)
    newer = reserve_measurement_context(tmp_path, "new-run", 2_000_000_000, 2)
    store.register_measurement(old)
    store.register_measurement(newer)
    panel = SimpleNamespace(
        _measurement_run_generation="new-run", _log=lambda text: None,
        append_workstation_data=store.enqueue,
        measurement_registry=lambda: SimpleNamespace(
            record_for=lambda measurement_id: None,
            route_outcome=lambda value: store.enqueue("method_results", value.to_record(), priority=True)),
    )
    controller = MeasurementViewController(panel)
    panel._measurement_view_controller_owner = controller
    coordinator = MethodEvaluationCoordinator(process_target=stalled_worker)
    controller._method_evaluation_coordinator = coordinator
    # Existing result projection is independently covered by method_evaluation
    # tests; this fixture writes those terminal records through the real store.
    controller._commit_primary_method_outcome = lambda value, **kw: (value, False, "ADMITTED")
    controller._commit_method_evaluation_bundle = lambda bundle: store.enqueue(
        "method_evaluations", bundle.to_record(), priority=True)
    backend = StandalonePanelBackend(panel)
    pane = MethodWorkerRecoveryPane(backend.measurement_worker_snapshot, backend.restart_measurement_method_worker)
    pane.resize(900, 100)
    pane.show()
    pane.method_combo.setCurrentIndex(pane.method_combo.findData(COMPARISON))
    ticks = []
    timer = QtCore.QTimer()
    timer.setInterval(10)
    timer.timeout.connect(lambda: ticks.append(time.monotonic()))
    timer.start()

    def until(predicate):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            QtWidgets.QApplication.processEvents()
            controller._poll_method_evaluations()
            pane.refresh()
            if predicate():
                return
            time.sleep(0.005)
        raise AssertionError("recovery UI postcondition not observed")

    try:
        coordinator.submit((request(PRIMARY, 1), request(COMPARISON, 1)))
        until(lambda: pane._selected_row.get("state") == "RUNNING")
        ticks.clear()
        pane.restart_button.click()
        until(lambda: pane._selected_row.get("last_restart", {}).get("status") == "RESTARTED")
        until(lambda: not coordinator.pending_jobs())
        assert len(ticks) >= 5  # GUI continues while actor waits for SIGTERM/kill.
        assert "재시작 완료" in pane.feedback_label.toolTip()
        assert panel._measurement_run_generation == "new-run"
    finally:
        timer.stop()
        pane.close()
        coordinator.close()
        assert coordinator.wait_closed(20)
        controller._poll_method_evaluations()
        assert store.close(timeout_s=5)
    events = [json.loads(line) for line in old.stream_path("events").read_text().splitlines()]
    assert len(events) == 1 and events[0]["status"] == "RESTARTED"
    assert events[0]["measurement_id"] == "recovery-run"
    assert not newer.stream_path("events").exists()
    results = [json.loads(line) for line in old.stream_path("method_results").read_text().splitlines()]
    failed = next(value for value in results if value["method_id"] == COMPARISON)
    assert failed["status"] == "FAILED"
    assert "operator_restart" in failed["reasons"][0]
