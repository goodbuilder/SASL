"""NIC role transactions retain identity and terminate after persistence errors."""

from __future__ import annotations

from types import SimpleNamespace
import time
from dataclasses import replace

import pytest
from PyQt5 import QtCore, QtWidgets

from Workstation.device_runtime.network_roles import (
    NetworkRoleJob,
    NetworkRolePorts,
    NetworkRoleRuntime,
    NETWORK_ROLE_FIELDS,
)
from Workstation.tests.test_panel_composition import panels

from Workstation.device_runtime_controller import DeviceRuntimeController
from Workstation.workstation_core.network import (
    NetworkRoleInterfaceRequest,
    NetworkRoleInterfaceResult,
    NetworkRoleOwnedEntries,
    network_role_requirements,
    plan_network_role_move,
)


def _request_and_result():
    desired = network_role_requirements(("p1",))
    request = NetworkRoleInterfaceRequest("enx-b", "mac-b", desired)
    owned = NetworkRoleOwnedEntries(desired.addresses, desired.routes)
    result = NetworkRoleInterfaceResult("enx-b", "b", desired, owned, desired.addresses, desired.routes)
    return request, result


def _operation(request):
    return dict(
        role="p1",
        candidate={"interface": "enx-b"},
        worker_mode="apply",
        plan=plan_network_role_move({}, role="p1", target_interface="enx-b", target_stable_id="mac-b"),
        active_requests=(request,),
        requests=(request,),
        active_roles=(),
        phase="APPLYING",
    )


def test_network_role_save_failure_releases_operation_and_keeps_applied_host_ownership(monkeypatch):
    request, result = _request_and_result()

    def fail_save():
        raise OSError("settings-disk-full")

    panel = SimpleNamespace(
        network_role_operation=_operation(request),
        network_role_apply_thread=None,
        network_role_owned_entries={},
        network_role_assignments={},
        network_role_status={},
        pixhawk_network_assignment_status={},
        _save_current_settings=fail_save,
        _log=lambda text: None,
        _refresh_operation_header=lambda: None,
    )
    controller = DeviceRuntimeController(panel)
    monkeypatch.setattr(controller, "_apply_role_assignment_state", lambda *args, **kwargs: None)
    controller._apply_network_role_result((result,))
    assert panel.network_role_operation is None
    assert panel.network_role_owned_entries["mac-b"] == result.owned_after.to_mapping()
    assert panel.network_role_status["state"] == "APPLIED · SAVE FAILED"
    assert "settings-disk-full" in panel.network_role_status["detail"]


def _runtime(*, worker_factory=None):
    events = []
    owner = NetworkRoleRuntime()
    owner.ports = NetworkRolePorts(
        stop_owners=lambda roles: events.append(("stop", roles)),
        owners_stopped=lambda roles: True,
        apply_assignments=lambda *args, **kwargs: events.append(("apply", QtCore.QThread.currentThread())),
        save_settings=lambda: events.append(("save",)),
        status_changed=lambda state, detail: events.append(("status", state)),
        refresh=lambda: events.append(("retire",)),
        worker_factory=worker_factory or (lambda requests: (_ for _ in ()).throw(AssertionError("unexpected worker"))),
    )
    return owner, events


def test_real_panel_network_fields_share_one_transaction_owner(panels):
    panel = panels()
    feature = panel.application.features["camera"]
    owner = feature.state.network_roles
    for name in NETWORK_ROLE_FIELDS:
        assert name not in vars(feature.views)
        assert name not in vars(feature.state)
        assert getattr(panel, name) is getattr(owner, name)
    owner.network_role_operation = {"phase": "fixture"}
    assert panel.network_role_operation is owner.network_role_operation
    panel.network_role_operation = None
    assert owner.network_role_operation is None
    assert panel.application.resource_inventory()["network_role_transition_timer"] is owner._timer


def test_role_publication_uses_admitted_settings_after_widgets_change(panels, monkeypatch):
    from Workstation import device_runtime_workers

    captured_requests = []

    def apply(requests):
        captured_requests.extend(requests)
        return tuple(
            NetworkRoleInterfaceResult(
                request.interface,
                "fixture",
                request.desired,
                NetworkRoleOwnedEntries(request.desired.addresses, request.desired.routes),
                request.desired.addresses,
                request.desired.routes,
            )
            for request in requests
        )

    monkeypatch.setattr(device_runtime_workers, "apply_network_role_transaction", apply)
    panel = panels()
    controller = panel._device_runtime_controller()
    monkeypatch.setattr(controller, "_confirm_network_role_assignment", lambda *args: True)
    original_local = panel.pixhawk_host_address_edit.text().strip()
    original_peer = panel.left_pixhawk_eth_ip_edit.text().strip()
    candidate = {"interface": "enx-captured", "mac": "fixture", "p1_endpoint": "udpin:0.0.0.0:14550"}
    assert controller.request_network_role_assignment("p1", candidate)
    owner = panel.application.features["camera"].state.network_roles
    try:
        panel.pixhawk_host_address_edit.setText("10.0.0.44")
        panel.left_pixhawk_eth_ip_edit.setText("10.0.0.45")
        panel.right_mavlink_url_edit.setText("udpin:0.0.0.0:19998")
        candidate["p1_endpoint"] = "udpin:0.0.0.0:19999"
        deadline = time.monotonic() + 3.0
        while owner.network_role_operation is not None and time.monotonic() < deadline:
            QtWidgets.QApplication.instance().processEvents()
            QtCore.QThread.msleep(1)
        assert owner.network_role_operation is None
        assert owner.network_role_status["state"] == "APPLIED"
        assert captured_requests[0].desired.addresses == (original_local + "/24",)
        assert panel.pixhawk_host_address_edit.text() == original_local
        assert panel.left_pixhawk_eth_ip_edit.text() == original_peer
        assert panel.left_mavlink_url_edit.text() == "udpin:0.0.0.0:14550"
        assert panel.right_mavlink_url_edit.text() == "udpin:0.0.0.0:14551"
        assert panel.settings.pixhawk_host_address == original_local
        assert panel.settings.left_pixhawk_eth_ip == original_peer
    finally:
        owner.prepare_shutdown()
        for job in tuple(owner._jobs.values()):
            job.thread.quit()
            job.thread.wait(2000)
        QtWidgets.QApplication.instance().processEvents()


def test_wrong_interface_result_cannot_publish_or_save_an_assignment():
    owner, events = _runtime()
    request, result = _request_and_result()
    owner.network_role_operation = _operation(request)
    wrong = NetworkRoleInterfaceResult(
        "other-nic", result.connection_name, result.desired, result.owned_after, result.addresses, result.routes
    )
    try:
        owner.apply_result((wrong,))
        assert owner.network_role_operation is None
        assert owner.network_role_status["state"] == "ERROR"
        assert owner.network_role_owned_entries == {}
        assert not any(event[0] in {"apply", "save"} for event in events)
    finally:
        owner.deleteLater()


def test_duplicate_and_late_job_outcomes_cannot_change_a_new_transaction():
    owner, events = _runtime()
    request, result = _request_and_result()
    original = _operation(request)
    owner.network_role_operation = original
    job = NetworkRoleJob(
        1, original, "apply", (request,), object(), object(), SimpleNamespace(deleteLater=lambda: None)
    )
    owner._job = job
    owner._jobs[1] = job
    owner.network_role_apply_thread, owner.network_role_apply_worker = job.thread, job.worker
    try:
        owner.deliver(job, result=(result,))
        owner.deliver(job, error="duplicate failure")
        assert events.count(("save",)) == 1
        # An external shutdown adapter may clear a legacy handle before Qt retire.
        owner.network_role_apply_thread = None
        assert owner.network_role_operation is original
        with pytest.raises(RuntimeError, match="already active"):
            owner.begin(_operation(request))
        owner.retire(job)
        assert owner.network_role_operation is None
        newer = _operation(request)
        owner.begin(newer)
        current = owner.network_role_operation
        owner.deliver(job, result=(result,))
        owner.retire(job)
        owner.apply_result((result,))
        owner.fail("unbound stale error")
        owner.clear_finished_legacy()
        assert owner.network_role_operation is current
        assert events.count(("save",)) == 1
    finally:
        owner.prepare_shutdown()
        owner.deleteLater()


def test_display_error_during_save_failure_cannot_strand_the_transaction():
    owner, events = _runtime()
    request, result = _request_and_result()

    def save_failed():
        raise OSError("disk full")

    def display_failed(state, detail):
        raise RuntimeError("view deleted")

    owner.ports = replace(owner.ports, save_settings=save_failed, status_changed=display_failed)
    owner.network_role_operation = _operation(request)
    try:
        owner.apply_result((result,))
        assert owner.network_role_operation is None
        assert owner.network_role_owned_entries["mac-b"] == result.owned_after.to_mapping()
        assert owner.network_role_status["state"] == "APPLIED · SAVE FAILED"
        assert owner.network_role_status["presentation_error"] == "view deleted"
    finally:
        owner.deleteLater()


def test_source_shutdown_query_error_ends_the_admitted_operation():
    owner, _events = _runtime()
    request, _result = _request_and_result()

    def broken_snapshot(roles):
        raise RuntimeError("source view unavailable")

    owner.ports = replace(owner.ports, owners_stopped=broken_snapshot)
    operation = _operation(request)
    operation.update(phase="STOPPING", deadline_s=time.monotonic() + 12.0)
    try:
        owner.begin(operation)
        owner._run_scheduled()
        assert owner.network_role_operation is None
        assert owner.network_role_status["state"] == "ERROR"
        assert "source view unavailable" in owner.network_role_status["detail"]
        assert not owner._jobs
    finally:
        owner.prepare_shutdown()
        owner.deleteLater()


def test_legacy_owner_is_closed_and_inventoried_without_creating_a_service():
    from Workstation.app_lifecycle_controller import AppLifecycleController

    owner, _events = _runtime()
    request, _result = _request_and_result()
    owner.begin(_operation(request))

    def reached_next_shutdown_step():
        raise RuntimeError("next shutdown step")

    panel = SimpleNamespace(
        _device_runtime_controller_owner=SimpleNamespace(_legacy_network_roles=owner),
        stop_auto_calibration_capture=reached_next_shutdown_step,
    )
    lifecycle = AppLifecycleController(panel)
    try:
        assert lifecycle._panel_attributes()["network_role_transition_timer"] is owner._timer
        with pytest.raises(RuntimeError, match="next shutdown step"):
            lifecycle._shutdown_stop_auxiliary_workers()
        assert owner._closing
        assert not owner._timer.isActive()
        assert owner.network_role_operation is None
    finally:
        owner.deleteLater()


def test_operation_capture_and_shutdown_prevent_later_input_or_timer_mutation():
    owner, events = _runtime()
    request, _result = _request_and_result()
    source = _operation(request)
    source["phase"] = "STOPPING"
    source["deadline_s"] = time.monotonic() + 12.0
    source["candidate"]["nested"] = {"address": "captured"}
    try:
        owner.begin(source)
        source["candidate"]["nested"]["address"] = "edited"
        assert owner.network_role_operation["candidate"]["nested"]["address"] == "captured"
        owner.prepare_shutdown()
        owner._run_scheduled()
        QtWidgets.QApplication.instance().processEvents()
        assert owner.network_role_operation is None
        assert not owner._timer.isActive()
        assert not owner._jobs
        assert not any(event[0] == "save" for event in events)
        with pytest.raises(RuntimeError, match="shutting down"):
            owner.begin(source)
    finally:
        owner.deleteLater()


@pytest.mark.parametrize("save_error,presentation_error", [(False, False), (True, False), (True, True), (False, True)])
def test_worker_results_keep_gui_affinity_and_retire_after_duplicate_signals(save_error, presentation_error):
    request, result = _request_and_result()
    run_threads = []

    class Worker(QtCore.QObject):
        completed = QtCore.pyqtSignal(object)
        failed = QtCore.pyqtSignal(str)
        finished = QtCore.pyqtSignal()

        @QtCore.pyqtSlot()
        def run(self):
            run_threads.append(QtCore.QThread.currentThread())
            self.completed.emit((result,))
            self.failed.emit("duplicate")
            self.finished.emit()

    owner, events = _runtime(worker_factory=lambda requests: Worker())

    def save():
        events.append(("save",))
        if save_error:
            raise OSError("disk full")

    def status_changed(state, detail):
        if presentation_error:
            raise RuntimeError("view deleted")

    def refresh():
        events.append(("retire",))
        if presentation_error:
            raise RuntimeError("view deleted")

    owner.ports = replace(owner.ports, save_settings=save, status_changed=status_changed, refresh=refresh)
    owner.network_role_operation = _operation(request)
    try:
        owner.start_worker((request,), mode="apply")
        deadline = time.monotonic() + 3.0
        while owner._jobs and time.monotonic() < deadline:
            QtWidgets.QApplication.instance().processEvents()
            QtCore.QThread.msleep(1)
        assert not owner._jobs
        assert owner.network_role_operation is None
        assert events.count(("save",)) == 1
        assert ("apply", owner.thread()) in events
        assert run_threads and run_threads[0] is not owner.thread()
        assert events[-1] == ("retire",)
        assert owner.network_role_status["state"] == ("APPLIED · SAVE FAILED" if save_error else "APPLIED")
        if presentation_error:
            assert owner.network_role_status["presentation_error"] == "view deleted"
    finally:
        owner.prepare_shutdown()
        for job in tuple(owner._jobs.values()):
            job.thread.quit()
            job.thread.wait(2000)
        QtWidgets.QApplication.instance().processEvents()
        owner.deleteLater()
