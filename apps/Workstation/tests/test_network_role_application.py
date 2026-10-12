from __future__ import annotations

import pytest
from types import SimpleNamespace

from Workstation.workstation_core.network import (
    NETWORK_ROLE_CAMERA,
    NETWORK_ROLE_P1,
    NETWORK_ROLE_P2,
    NetworkManagerConnectionSnapshot,
    NetworkRoleInterfaceRequest,
    NetworkRoleInterfaceResult,
    NetworkRoleOwnedEntries,
    apply_network_role_transaction,
    network_role_entry_delta,
    network_role_requirements,
    plan_network_role_move,
)


def test_same_nic_composes_camera_and_both_pixhawk_roles() -> None:
    requirements = network_role_requirements(
        (NETWORK_ROLE_CAMERA, NETWORK_ROLE_P1, NETWORK_ROLE_P2)
    )

    assert requirements.roles == ("camera", "p1", "p2")
    assert requirements.addresses == ("10.0.0.11/32", "10.0.0.10/24")
    assert requirements.routes == (
        "10.0.0.231/32 src=10.0.0.11",
        "10.0.0.232/32 src=10.0.0.11",
        "10.0.0.2/32 src=10.0.0.10",
        "10.0.0.3/32 src=10.0.0.10",
    )


def test_shared_pixhawk_address_is_reference_deduplicated() -> None:
    both = network_role_requirements(("p1", "p2"))
    p2_only = network_role_requirements(("p2",))

    assert both.addresses == ("10.0.0.10/24",)
    assert p2_only.addresses == ("10.0.0.10/24",)
    assert p2_only.routes == ("10.0.0.3/32 src=10.0.0.10",)


def test_move_camera_off_shared_nic_preserves_pixhawks_on_source() -> None:
    plan = plan_network_role_move(
        {
            "camera": {"interface": "enx-a", "stable_id": "mac-a"},
            "p1": {"interface": "enx-a", "stable_id": "mac-a"},
            "p2": {"interface": "enx-a", "stable_id": "mac-a"},
        },
        role="camera",
        target_interface="enx-b",
        target_stable_id="mac-b",
    )

    assert plan.source_interface == "enx-a"
    assert plan.target_interface == "enx-b"
    assert plan.affected_interfaces == ("enx-b", "enx-a")
    assert plan.roles_by_interface(after=False) == {
        "enx-a": ("camera", "p1", "p2")
    }
    assert plan.roles_by_interface() == {
        "enx-b": ("camera",),
        "enx-a": ("p1", "p2"),
    }


def test_move_to_same_nic_is_idempotent() -> None:
    plan = plan_network_role_move(
        {"camera": "enx-a", "p1": "enx-a", "p2": "enx-a"},
        role="p1",
        target_interface="enx-a",
    )

    assert plan.affected_interfaces == ("enx-a",)
    assert plan.roles_by_interface() == {"enx-a": ("camera", "p1", "p2")}


@pytest.mark.parametrize(
    ("roles", "kwargs"),
    [
        (("unknown",), {}),
        (("camera",), {"camera_local_address": "not-an-ip"}),
        (("p1",), {"p1_peer": "not-an-ip"}),
    ],
)
def test_role_contract_rejects_invalid_inputs(roles, kwargs) -> None:
    with pytest.raises(ValueError):
        network_role_requirements(roles, **kwargs)


def test_preexisting_matching_entry_is_reused_but_not_claimed() -> None:
    desired = network_role_requirements(("camera",))
    delta = network_role_entry_delta(
        current_addresses=desired.addresses,
        current_routes=desired.routes,
        owned_before=NetworkRoleOwnedEntries(),
        desired=desired,
    )

    assert delta.added_addresses == ()
    assert delta.added_routes == ()
    assert delta.owned_after == NetworkRoleOwnedEntries()


def test_role_delta_removes_only_owned_entries_and_preserves_unrelated() -> None:
    old = network_role_requirements(("camera", "p1", "p2"))
    desired = network_role_requirements(("p1", "p2"))
    delta = network_role_entry_delta(
        current_addresses=("192.168.0.254/24", *old.addresses),
        current_routes=("192.168.0.0/24", *old.routes),
        owned_before=NetworkRoleOwnedEntries(old.addresses, old.routes),
        desired=desired,
    )

    assert delta.addresses_after == ("192.168.0.254/24", "10.0.0.10/24")
    assert delta.routes_after == (
        "192.168.0.0/24",
        "10.0.0.2/32 src=10.0.0.10",
        "10.0.0.3/32 src=10.0.0.10",
    )
    assert delta.removed_addresses == ("10.0.0.11/32",)
    assert delta.removed_routes == (
        "10.0.0.231/32 src=10.0.0.11",
        "10.0.0.232/32 src=10.0.0.11",
    )


def test_transaction_adopts_only_exact_legacy_role_entries(monkeypatch) -> None:
    from Workstation.workstation_core import network

    old = network_role_requirements(("camera", "p1", "p2"))
    desired = network_role_requirements(("p1", "p2"))
    state = {
        "enx-a": NetworkManagerConnectionSnapshot(
            "enx-a",
            "legacy-profile",
            True,
            True,
            ("192.168.0.254/24", *old.addresses),
            ("192.168.0.0/24", *old.routes),
        )
    }
    monkeypatch.setattr(
        network,
        "network_manager_connection_snapshot",
        lambda interface, stable_id="": state[interface],
    )

    def apply(snapshot, *, addresses, routes):
        state[snapshot.interface] = NetworkManagerConnectionSnapshot(
            snapshot.interface,
            snapshot.connection_name,
            True,
            True,
            tuple(addresses),
            tuple(routes),
        )

    monkeypatch.setattr(network, "_apply_nm_connection_state", apply)
    monkeypatch.setattr(network, "_verify_network_role_kernel", lambda *_args, **_kwargs: None)

    result = apply_network_role_transaction(
        (
            NetworkRoleInterfaceRequest(
                "enx-a",
                "mac-a",
                desired,
                NetworkRoleOwnedEntries(),
                old,
            ),
        )
    )[0]

    assert result.addresses == ("192.168.0.254/24", "10.0.0.10/24")
    assert result.routes == (
        "192.168.0.0/24",
        "10.0.0.2/32 src=10.0.0.10",
        "10.0.0.3/32 src=10.0.0.10",
    )
    assert result.owned_after == NetworkRoleOwnedEntries(
        desired.addresses, desired.routes
    )


def test_transaction_applies_target_before_source(monkeypatch) -> None:
    from Workstation.workstation_core import network

    target_desired = network_role_requirements(("camera",))
    source_before = network_role_requirements(("camera", "p1", "p2"))
    source_desired = network_role_requirements(("p1", "p2"))
    state = {
        "enx-b": NetworkManagerConnectionSnapshot("enx-b", "b", True, True),
        "enx-a": NetworkManagerConnectionSnapshot(
            "enx-a", "a", True, True, source_before.addresses, source_before.routes
        ),
    }
    calls: list[tuple[str, str]] = []

    monkeypatch.setattr(
        network,
        "network_manager_connection_snapshot",
        lambda interface, stable_id="": state[interface],
    )

    def apply(snapshot, *, addresses, routes):
        calls.append(("apply", snapshot.interface))
        state[snapshot.interface] = NetworkManagerConnectionSnapshot(
            snapshot.interface,
            snapshot.connection_name,
            True,
            True,
            tuple(addresses),
            tuple(routes),
        )

    monkeypatch.setattr(network, "_apply_nm_connection_state", apply)
    monkeypatch.setattr(network, "_verify_network_role_kernel", lambda *_args, **_kwargs: None)

    result = apply_network_role_transaction(
        (
            NetworkRoleInterfaceRequest("enx-b", "mac-b", target_desired),
            NetworkRoleInterfaceRequest(
                "enx-a",
                "mac-a",
                source_desired,
                NetworkRoleOwnedEntries(source_before.addresses, source_before.routes),
            ),
        )
    )

    assert calls == [("apply", "enx-b"), ("apply", "enx-a")]
    assert result[0].owned_after == NetworkRoleOwnedEntries(
        target_desired.addresses, target_desired.routes
    )
    assert state["enx-a"].addresses == source_desired.addresses


def test_transaction_rolls_back_every_touched_interface_in_reverse(monkeypatch) -> None:
    from Workstation.workstation_core import network

    desired = network_role_requirements(("camera",))
    original = {
        "enx-b": NetworkManagerConnectionSnapshot("enx-b", "b", True, True),
        "enx-a": NetworkManagerConnectionSnapshot("enx-a", "a", True, True),
    }
    state = dict(original)
    calls: list[tuple[str, str]] = []
    monkeypatch.setattr(
        network,
        "network_manager_connection_snapshot",
        lambda interface, stable_id="": state[interface],
    )

    def apply(snapshot, *, addresses, routes):
        calls.append(("apply", snapshot.interface))
        state[snapshot.interface] = NetworkManagerConnectionSnapshot(
            snapshot.interface,
            snapshot.connection_name,
            True,
            True,
            tuple(addresses),
            tuple(routes),
        )

    def restore(snapshot):
        calls.append(("restore", snapshot.interface))
        state[snapshot.interface] = snapshot

    monkeypatch.setattr(network, "_apply_nm_connection_state", apply)
    monkeypatch.setattr(network, "_restore_nm_connection_snapshot", restore)
    monkeypatch.setattr(
        network,
        "_verify_network_role_kernel",
        lambda interface, _desired: (_ for _ in ()).throw(RuntimeError("route mismatch"))
        if interface == "enx-a"
        else None,
    )

    with pytest.raises(RuntimeError, match="route mismatch"):
        apply_network_role_transaction(
            (
                NetworkRoleInterfaceRequest("enx-b", "mac-b", desired),
                NetworkRoleInterfaceRequest("enx-a", "mac-a", desired),
            )
        )

    assert calls == [
        ("apply", "enx-b"),
        ("apply", "enx-a"),
        ("restore", "enx-a"),
        ("restore", "enx-b"),
    ]
    assert state == original


def test_existing_profile_mutation_preserves_unrelated_values_in_full_payload(monkeypatch) -> None:
    from Workstation.workstation_core import network

    commands: list[tuple[str, ...]] = []
    monkeypatch.setattr(network, "_run_nmcli_mutation", commands.append)
    snapshot = NetworkManagerConnectionSnapshot(
        "enx-a",
        "field-profile",
        True,
        True,
        ("192.168.0.254/24",),
        ("192.168.0.0/24",),
        "manual",
        "yes",
    )

    network._apply_nm_connection_state(
        snapshot,
        addresses=("192.168.0.254/24", "10.0.0.10/24"),
        routes=("192.168.0.0/24", "10.0.0.2/32 src=10.0.0.10"),
    )

    assert commands == [
        (
            "connection",
            "modify",
            "field-profile",
            "ipv4.addresses",
            "192.168.0.254/24, 10.0.0.10/24",
            "ipv4.routes",
            "192.168.0.0/24, 10.0.0.2/32 src=10.0.0.10",
        ),
        ("device", "reapply", "enx-a"),
    ]


def test_new_profile_has_no_default_route_and_is_activated(monkeypatch) -> None:
    from Workstation.workstation_core import network

    commands: list[tuple[str, ...]] = []
    monkeypatch.setattr(network, "_run_nmcli_mutation", commands.append)
    snapshot = NetworkManagerConnectionSnapshot(
        "enx-new", "sasl-workstation-macnew", False, False
    )

    network._apply_nm_connection_state(
        snapshot,
        addresses=("10.0.0.11/32",),
        routes=("10.0.0.231/32 src=10.0.0.11",),
    )

    add = commands[0]
    assert add[:7] == (
        "connection",
        "add",
        "type",
        "ethernet",
        "ifname",
        "enx-new",
        "con-name",
    )
    assert add[add.index("ipv4.never-default") + 1] == "yes"
    assert commands[-1] == (
        "connection",
        "up",
        "sasl-workstation-macnew",
        "ifname",
        "enx-new",
    )


def test_removing_last_app_role_disables_empty_manual_ipv4_profile(monkeypatch) -> None:
    from Workstation.workstation_core import network

    commands: list[tuple[str, ...]] = []
    monkeypatch.setattr(network, "_run_nmcli_mutation", commands.append)
    snapshot = NetworkManagerConnectionSnapshot(
        "enx-a",
        "sasl-workstation-maca",
        True,
        True,
        ("10.0.0.10/24",),
        ("10.0.0.2/32 src=10.0.0.10",),
        "manual",
        "yes",
    )

    network._apply_nm_connection_state(snapshot, addresses=(), routes=())

    assert commands[0] == (
        "connection",
        "modify",
        "sasl-workstation-maca",
        "ipv4.method",
        "disabled",
        "ipv4.addresses",
        "",
        "ipv4.routes",
        "",
    )
    assert commands[1] == ("device", "reapply", "enx-a")


def test_restore_reinstates_exact_snapshot(monkeypatch) -> None:
    from Workstation.workstation_core import network

    commands: list[tuple[str, ...]] = []
    monkeypatch.setattr(network, "_run_nmcli_mutation", commands.append)
    snapshot = NetworkManagerConnectionSnapshot(
        "enx-a",
        "field-profile",
        True,
        True,
        ("192.168.0.254/24",),
        ("192.168.0.0/24",),
        "manual",
        "yes",
    )

    network._restore_nm_connection_snapshot(snapshot)

    assert commands[0] == (
        "connection",
        "modify",
        "field-profile",
        "ipv4.method",
        "manual",
        "ipv4.addresses",
        "192.168.0.254/24",
        "ipv4.routes",
        "192.168.0.0/24",
        "ipv4.never-default",
        "yes",
    )
    assert commands[1] == (
        "connection",
        "up",
        "field-profile",
        "ifname",
        "enx-a",
    )


def test_network_role_provenance_round_trips_in_settings(tmp_path) -> None:
    from dataclasses import replace

    from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings

    path = tmp_path / "settings.json"
    expected = replace(
        AppSettings(),
        camera_network_stable_id="58:86:94:fc:e0:7e",
        left_mavlink_interface_stable_id="00:23:a4:0a:2e:39",
        right_mavlink_interface_stable_id="00:23:a4:0a:2e:39",
        network_role_owned_entries={
            "58:86:94:fc:e0:7e": {
                "addresses": ["10.0.0.11/32"],
                "routes": ["10.0.0.231/32 src=10.0.0.11"],
            }
        },
    )

    save_settings(expected, path)
    actual = load_settings(path)

    assert actual.camera_network_stable_id == expected.camera_network_stable_id
    assert actual.left_mavlink_interface_stable_id == expected.left_mavlink_interface_stable_id
    assert actual.network_role_owned_entries == expected.network_role_owned_entries


def test_controller_rejects_role_change_during_measurement() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    logs: list[str] = []
    panel = SimpleNamespace(
        network_role_apply_thread=None,
        network_role_operation=None,
        _session_recording_active=lambda: False,
        measurement_run_snapshot=lambda: {"state": "RUNNING"},
        _log=logs.append,
    )

    accepted = DeviceRuntimeController(panel).request_network_role_assignment(
        "camera", {"interface": "enx-a"}
    )

    assert accepted is False
    assert any("Measurement" in line for line in logs)


def test_controller_stops_only_active_owners_on_affected_nics(monkeypatch) -> None:
    from PyQt5 import QtCore
    from Workstation.device_runtime_controller import DeviceRuntimeController

    calls: list[object] = []
    panel = SimpleNamespace(
        network_role_apply_thread=None,
        network_role_operation=None,
        network_role_owned_entries={},
        network_role_assignments={
            "camera": {"interface": "enx-a", "stable_id": "mac-a"},
            "p1": {"interface": "enx-a", "stable_id": "mac-a"},
            "p2": {"interface": "enx-c", "stable_id": "mac-c"},
        },
        _session_recording_active=lambda: False,
        measurement_run_snapshot=lambda: {"state": "IDLE"},
        camera_connected=True,
        left_pane=SimpleNamespace(worker=object()),
        right_pane=SimpleNamespace(worker=None),
        _mavlink_open_sides=lambda: {"left", "right"},
        stop_yolo_streams=lambda: calls.append("stop-camera"),
        disconnect_mavlink_pixhawk=lambda side=None: calls.append(("disconnect", side)),
        left_mavlink_url_edit=SimpleNamespace(text=lambda: "udpin:0.0.0.0:14550"),
        right_mavlink_url_edit=SimpleNamespace(text=lambda: "udpin:0.0.0.0:14551"),
        _continue_network_role_assignment=lambda: None,
        _log=lambda message: calls.append(("log", message)),
    )
    controller = DeviceRuntimeController(panel)
    monkeypatch.setattr(controller, "_confirm_network_role_assignment", lambda *_args: True)
    monkeypatch.setattr(controller, "_network_role_requests", lambda *_args, **_kwargs: ())
    monkeypatch.setattr(controller, "_network_role_requirements_for", network_role_requirements)
    monkeypatch.setattr(QtCore.QTimer, "singleShot", lambda *_args: None)

    accepted = controller.request_network_role_assignment(
        "camera", {"interface": "enx-b", "mac": "mac-b"}
    )

    assert accepted is True
    assert panel.network_role_operation["active_roles"] == ("camera", "p1")
    assert "stop-camera" in calls
    assert ("disconnect", "left") in calls
    assert ("disconnect", "right") not in calls


def test_network_role_worker_maps_success_and_failure(monkeypatch) -> None:
    from Workstation import device_runtime_workers
    from Workstation.device_runtime_workers import NetworkRoleApplyWorker

    request = NetworkRoleInterfaceRequest(
        "enx-a", "mac-a", network_role_requirements(("camera",))
    )
    expected = ("result",)
    monkeypatch.setattr(
        device_runtime_workers,
        "apply_network_role_transaction",
        lambda _requests: expected,
    )
    completed: list[object] = []
    failed: list[str] = []
    worker = NetworkRoleApplyWorker((request,))
    worker.completed.connect(completed.append)
    worker.failed.connect(failed.append)
    worker.run()
    assert completed == [expected]
    assert failed == []

    monkeypatch.setattr(
        device_runtime_workers,
        "apply_network_role_transaction",
        lambda _requests: (_ for _ in ()).throw(RuntimeError("denied")),
    )
    worker = NetworkRoleApplyWorker((request,))
    worker.failed.connect(failed.append)
    worker.run()
    assert failed[-1] == "denied"


def test_active_pixhawk_move_commits_host_state_but_requires_explicit_connect(monkeypatch) -> None:
    from PyQt5 import QtCore
    from Workstation.device_runtime_controller import DeviceRuntimeController

    class Edit:
        def __init__(self, value: str) -> None:
            self.value = value

        def text(self) -> str:
            return self.value

        def setText(self, value: str) -> None:
            self.value = str(value)

    plan = plan_network_role_move(
        {
            "camera": {"interface": "enx-c", "stable_id": "mac-c"},
            "p1": {"interface": "enx-a", "stable_id": "mac-a"},
            "p2": {"interface": "enx-c", "stable_id": "mac-c"},
        },
        role="p1",
        target_interface="enx-b",
        target_stable_id="mac-b",
    )
    target = NetworkRoleInterfaceRequest(
        "enx-b", "mac-b", network_role_requirements(("p1",))
    )
    source = NetworkRoleInterfaceRequest(
        "enx-a", "mac-a", network_role_requirements(())
    )
    results = (
        NetworkRoleInterfaceResult(
            "enx-b",
            "b",
            target.desired,
            NetworkRoleOwnedEntries(target.desired.addresses, target.desired.routes),
            target.desired.addresses,
            target.desired.routes,
        ),
        NetworkRoleInterfaceResult(
            "enx-a", "a", source.desired, NetworkRoleOwnedEntries(), (), ()
        ),
    )
    calls: list[object] = []
    connected: set[str] = set()
    panel = SimpleNamespace(
        network_role_operation={
            "role": "p1",
            "candidate": {
                "interface": "enx-b",
                "p1_endpoint": "udpin:0.0.0.0:14550",
            },
            "plan": plan,
            "worker_mode": "apply",
            "active_requests": (target, source),
            "active_roles": ("p1",),
            "before_endpoints": {
                "left": "udpin:0.0.0.0:14550",
                "right": "udpin:0.0.0.0:14551",
            },
        },
        network_role_owned_entries={},
        network_role_assignments={},
        network_role_status={},
        pixhawk_network_assignments={"left": {}, "right": {}},
        pixhawk_network_assignment_status={"left": {}, "right": {}},
        interface_edit=Edit("enx-c"),
        host_address_edit=Edit("10.0.0.11/32"),
        pixhawk_host_address_edit=Edit("10.0.0.10"),
        left_pixhawk_eth_ip_edit=Edit("10.0.0.2"),
        right_pixhawk_eth_ip_edit=Edit("10.0.0.3"),
        left_rtsp_url_edit=Edit("rtsp://10.0.0.231:8554/visible"),
        right_rtsp_url_edit=Edit("rtsp://10.0.0.232:8554/visible"),
        left_mavlink_url_edit=Edit("udpin:0.0.0.0:14550"),
        right_mavlink_url_edit=Edit("udpin:0.0.0.0:14551"),
        left_pane=SimpleNamespace(worker=None),
        right_pane=SimpleNamespace(worker=None),
        camera_connected=False,
        pixhawk_connection_thread=None,
        _sync_mavlink_subscriber_settings=lambda: calls.append("sync"),
        _mavlink_open_sides=lambda: set(connected),
        _mavlink_connected_sides=lambda: set(connected),
        connect_mavlink_pixhawk=lambda side: (calls.append(("connect", side)), connected.add(side)),
        connect_cameras=lambda: calls.append("connect-camera"),
        _save_current_settings=lambda: calls.append("save"),
        _log=lambda message: calls.append(("log", message)),
        _refresh_operation_header=lambda: None,
    )
    controller = DeviceRuntimeController(panel)
    panel._continue_network_role_reconnect = controller._continue_network_role_reconnect
    panel._poll_network_role_reconnect = controller._poll_network_role_reconnect
    monkeypatch.setattr(QtCore.QTimer, "singleShot", lambda *_args: None)

    controller._apply_network_role_result(results)

    assert "save" in calls
    assert ("connect", "left") not in calls
    assert panel.network_role_operation is None
    assert panel.network_role_status["state"] == "APPLIED"
    assert panel.pixhawk_network_assignment_status["left"]["state"] == (
        "ASSIGNED · PRESS CONNECT"
    )
    assert panel.pixhawk_network_assignments["left"] == {
        "interface": "enx-b",
        "local_address": "10.0.0.10",
    }


def test_workspaces_show_composed_roles_without_fixed_nic_count() -> None:
    from PyQt5 import QtWidgets
    from Workstation.main_window import WorkstationMainWindow
    from Workstation.tests.workstation_test_support import _FakeBackend

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    camera_interface = SimpleNamespace(name="enx-a", address="10.0.0.11/32")
    backend.panel._camera_candidate_rows = [
        {
            "kind": "NIC",
            "address": "enx-a",
            "port": "",
            "status": "UP",
            "interface": camera_interface,
            "host_ip": "10.0.0.11",
        }
    ]
    backend.panel._pixhawk_candidate_rows_by_source = {
        "network": [
            {
                "kind": "NETWORK",
                "candidate_id": "8:enx-b:10.0.0.10/24",
                "interface": "enx-b",
                "carrier": "UP",
                "local_cidr": "10.0.0.10/24",
                "local_ip": "10.0.0.10",
            }
        ],
        "pc": [],
        "mavlink": [],
    }
    backend.panel.network_role_assignments = {
        "camera": {"interface": "enx-a", "stable_id": "mac-a"},
        "p1": {"interface": "enx-a", "stable_id": "mac-a"},
        "p2": {"interface": "enx-b", "stable_id": "mac-b"},
    }
    window = WorkstationMainWindow(backend=backend)
    snapshot = window.status_adapter.model.snapshot()

    window.camera.refresh(snapshot)
    window.pixhawk.refresh(snapshot)

    assert window.camera.candidate_model.columnCount() == 7
    assert window.camera.candidate_model.item(0, 6).text() == "Camera, P1 (shared NIC)"
    assert window.pixhawk.candidate_model.item(0, 4).text() == "P2"
    window.close()


def test_camera_and_pixhawk_candidate_evidence_share_network_role_projection() -> None:
    from Workstation.evidence import EvidenceAdapter

    panel = SimpleNamespace(
        network_role_assignments={
            "camera": {"interface": "enx-a", "stable_id": "mac-a"},
            "p1": {"interface": "enx-a", "stable_id": "mac-a"},
            "p2": "not-a-mapping",
        },
        network_role_status={"state": "APPLYING", "detail": "bringing up enx-a"},
        network_role_operation=object(),
        _camera_candidate_rows=[],
        _pixhawk_candidate_rows_by_source={},
    )
    keys = ("network_role_status", "network_role_active", "network_roles")

    camera = EvidenceAdapter(panel).camera_connection_candidates()
    pixhawk = EvidenceAdapter(panel).pixhawk_connection_candidates()

    assert {key: camera[key] for key in keys} == {key: pixhawk[key] for key in keys}
    assert camera["network_role_active"] is True
    assert camera["network_roles"] == {
        "camera": {"interface": "enx-a", "stable_id": "mac-a"},
        "p1": {"interface": "enx-a", "stable_id": "mac-a"},
    }


def test_camera_candidate_evidence_projects_interface_mac() -> None:
    from ipaddress import ip_interface

    from Workstation.evidence import EvidenceAdapter
    from Workstation.workstation_core.network import InterfaceCandidate

    legacy = InterfaceCandidate(name="enx-a", address=ip_interface("10.0.0.11/32"))
    assert legacy.mac == ""
    panel = SimpleNamespace(
        network_role_assignments={},
        _camera_candidate_rows=[
            {"kind": "NIC", "interface": legacy},
            {
                "kind": "NIC",
                "interface": SimpleNamespace(
                    name="enx-b", address="10.0.0.12/24", mac="AA:BB:CC:DD:EE:02"
                ),
            },
        ],
    )

    rows = EvidenceAdapter(panel).camera_connection_candidates()["candidates"]

    assert [(row["interface"], row["mac"]) for row in rows] == [
        ("enx-a", ""),
        ("enx-b", "AA:BB:CC:DD:EE:02"),
    ]


def test_camera_workspace_role_column_follows_saved_mac_across_rename() -> None:
    from PyQt5 import QtWidgets
    from Workstation.main_window import WorkstationMainWindow
    from Workstation.tests.workstation_test_support import _FakeBackend

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel._camera_candidate_rows = [
        {
            "kind": "NIC",
            "address": "enx-new",
            "port": "",
            "status": "UP",
            "interface": SimpleNamespace(
                name="enx-new", address="10.0.0.11/32", mac="aa:bb:cc:dd:ee:01"
            ),
            "host_ip": "10.0.0.11",
        },
        {
            "kind": "NIC",
            "address": "enx-old",
            "port": "",
            "status": "UP",
            "interface": SimpleNamespace(
                name="enx-old", address="10.0.0.12/32", mac="de:ad:be:ef:00:01"
            ),
            "host_ip": "10.0.0.12",
        },
    ]
    backend.panel.network_role_assignments = {
        "camera": {"interface": "enx-old", "stable_id": "aa:bb:cc:dd:ee:01"},
        "p1": {"interface": "", "stable_id": ""},
        "p2": {"interface": "", "stable_id": ""},
    }
    window = WorkstationMainWindow(backend=backend)
    snapshot = window.status_adapter.model.snapshot()

    window.camera.refresh(snapshot)

    # The adapter was renamed enx-old -> enx-new; the saved MAC keeps the role
    # on the same hardware and off the NIC that recycled the old name.
    assert window.camera.candidate_model.item(0, 6).text() == "Camera"
    assert window.camera.candidate_model.item(1, 6).text() == "--"
    window.close()
