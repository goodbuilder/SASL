from __future__ import annotations

import json
from ipaddress import ip_interface
from types import SimpleNamespace
import time

from PyQt5 import QtWidgets


def test_pixhawk_workspace_separates_candidate_rows_from_side_assignments() -> None:
    from PyQt5 import QtCore
    from Workstation.main_window import WorkstationMainWindow
    from Workstation.tests.workstation_test_support import _FakeBackend

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel._pixhawk_candidate_rows_by_source = {
        "network": [
            {
                "kind": "NETWORK",
                "candidate_id": "5:enp5s0:192.168.1.10/24",
                "interface": "enp5s0",
                "carrier": "UP",
                "local_cidr": "192.168.1.10/24",
                "local_ip": "192.168.1.10",
                "p1_route": "ROUTE OK · enp5s0 src=192.168.1.10",
                "p2_route": "ROUTE MISMATCH",
                "p1_route_state": "ROUTE OK",
                "p2_route_state": "ROUTE MISMATCH",
                "p1_endpoint": "udpin:0.0.0.0:14550",
                "p2_endpoint": "udpin:0.0.0.0:14551",
            }
        ],
        "pc": [],
        "mavlink": [],
    }
    backend.panel.pixhawk_network_assignments = {
        "left": {"interface": "enp5s0", "local_address": "192.168.1.10"},
        "right": {"interface": "", "local_address": ""},
    }
    backend.panel.pixhawk_network_assignment_status = {
        "left": {"state": "ASSIGNED", "detail": "ROUTE OK · enp5s0 src=192.168.1.10"},
        "right": {"state": "LEGACY AUTO ROUTE", "detail": "no explicit NIC assignment"},
    }
    backend.panel.left_mavlink_url_edit = SimpleNamespace(
        text=lambda: "udpin:0.0.0.0:14550"
    )
    backend.panel.right_mavlink_url_edit = SimpleNamespace(
        text=lambda: "udpin:0.0.0.0:14551"
    )
    backend.panel.workstation_pixhawk_discovery_status = "FOUND 1 NETWORK CHOICE"
    window = WorkstationMainWindow(backend=backend)

    window.pixhawk.refresh(window.status_adapter.model.snapshot())

    headers = [
        window.pixhawk.candidate_model.headerData(
            column, QtCore.Qt.Horizontal, QtCore.Qt.DisplayRole
        )
        for column in range(window.pixhawk.candidate_model.columnCount())
    ]
    assert headers == ["Source", "NIC / Device", "Link", "Local IPv4", "Assigned"]
    assert window.pixhawk.candidate_model.item(0, 4).text() == "P1"
    assert window.pixhawk.network_assignment_values["left"]["nic"].value.text() == "enp5s0"
    assert (
        window.pixhawk.network_assignment_values["left"]["endpoint"].value.text()
        == "udpin:0.0.0.0:14550"
    )
    assert "enp5s0" in window.pixhawk.network_assignment_values["left"]["route"].value.text()
    assert window.pixhawk.network_assignment_values["right"]["nic"].value.text() == "--"
    assert (
        window.pixhawk.network_assignment_values["right"]["endpoint"].value.text()
        == "udpin:0.0.0.0:14551"
    )
    window.close()
    window.deleteLater()
    app.processEvents()

def test_pixhawk_network_discovery_lists_each_address_and_unconfigured_link(monkeypatch) -> None:
    from Workstation.workstation_core import network

    monkeypatch.setattr(
        network,
        "_wired_link_inventory",
        lambda: [
            {
                "interface": "enx-camera-or-pixhawk",
                "ifindex": 7,
                "carrier": "UP",
                "mac": "00:11:22:33:44:55",
                "driver": "r8152",
                "device_path": "/sys/devices/pci/usb/7",
            },
            {
                "interface": "enx-new",
                "ifindex": 8,
                "carrier": "DOWN",
                "mac": "00:aa:bb:cc:dd:ee",
                "driver": "r8152",
                "device_path": "/sys/devices/pci/usb/8",
            },
        ],
    )
    monkeypatch.setattr(
        network,
        "_global_ipv4_inventory",
        lambda: {
            "enx-camera-or-pixhawk": [
                ip_interface("10.0.0.10/24"),
                ip_interface("192.168.0.254/24"),
            ]
        },
    )

    def route(peer: str, *, requested_source: str = "") -> network.PixhawkRouteEvidence:
        return network.PixhawkRouteEvidence(
            peer=peer,
            requested_source=requested_source,
            interface="enx-camera-or-pixhawk" if requested_source == "10.0.0.10" else "wlo1",
            source=requested_source or "172.20.10.3",
        )

    monkeypatch.setattr(network, "pixhawk_route_evidence", route)

    candidates = network.discover_pixhawk_network_candidates(
        p1_peer="10.0.0.2",
        p2_peer="10.0.0.3",
        p1_endpoint="udpin:0.0.0.0:14550",
        p2_endpoint="udpin:0.0.0.0:14551",
    )

    assert [(item.interface, item.local_cidr, item.carrier) for item in candidates] == [
        ("enx-camera-or-pixhawk", "10.0.0.10/24", "UP"),
        ("enx-camera-or-pixhawk", "192.168.0.254/24", "UP"),
        ("enx-new", "", "DOWN"),
    ]
    assert candidates[0].p1_route.status_for(candidates[0].interface, candidates[0].local_ip) == "ROUTE OK"
    assert candidates[1].p1_route.status_for(candidates[1].interface, candidates[1].local_ip) == "ROUTE MISMATCH"
    assert candidates[2].to_mapping()["local_cidr"] == ""


def test_pixhawk_route_evidence_uses_kernel_json_without_mutation(monkeypatch) -> None:
    from Workstation.workstation_core import network

    calls: list[list[str]] = []

    def run(args, **kwargs):
        calls.append(list(args))
        return SimpleNamespace(
            returncode=0,
            stdout=json.dumps(
                [
                    {
                        "dst": "10.0.0.2",
                        "dev": "enx-pixhawk",
                        "prefsrc": "10.0.0.10",
                    }
                ]
            ),
            stderr="",
        )

    monkeypatch.setattr(network.subprocess, "run", run)

    evidence = network.pixhawk_route_evidence(
        "10.0.0.2", requested_source="10.0.0.10"
    )

    assert evidence.status_for("enx-pixhawk", "10.0.0.10") == "ROUTE OK"
    assert calls == [
        ["ip", "-j", "-4", "route", "get", "10.0.0.2", "from", "10.0.0.10"]
    ]
    assert all("add" not in call and "replace" not in call for call in calls)


def test_pixhawk_assignment_validation_reports_route_mismatch(monkeypatch) -> None:
    from Workstation.workstation_core import network

    monkeypatch.setattr(
        network,
        "_wired_link_inventory",
        lambda: [
            {
                "interface": "enx-selected",
                "ifindex": 3,
                "carrier": "UP",
                "mac": "",
                "driver": "",
                "device_path": "",
            }
        ],
    )
    monkeypatch.setattr(
        network,
        "_global_ipv4_inventory",
        lambda: {"enx-selected": [ip_interface("10.0.0.10/24")]},
    )
    monkeypatch.setattr(
        network,
        "pixhawk_route_evidence",
        lambda peer, *, requested_source="": network.PixhawkRouteEvidence(
            peer=peer,
            requested_source=requested_source,
            interface="wlo1",
            source="172.20.10.3",
        ),
    )

    result = network.validate_pixhawk_network_assignment(
        interface_name="enx-selected",
        local_address="10.0.0.10",
        peer_host="10.0.0.2",
        endpoint="udpin:0.0.0.0:14550",
    )

    assert result.ok is False
    assert result.state == "ROUTE MISMATCH"
    assert "dev=wlo1" in result.detail


def test_pixhawk_assignment_validation_accepts_matching_existing_state(monkeypatch) -> None:
    from Workstation.workstation_core import network

    monkeypatch.setattr(
        network,
        "_wired_link_inventory",
        lambda: [
            {
                "interface": "enx-pixhawk",
                "ifindex": 3,
                "carrier": "UP",
                "mac": "",
                "driver": "",
                "device_path": "",
            }
        ],
    )
    monkeypatch.setattr(
        network,
        "_global_ipv4_inventory",
        lambda: {"enx-pixhawk": [ip_interface("10.0.0.10/24")]},
    )
    monkeypatch.setattr(
        network,
        "pixhawk_route_evidence",
        lambda peer, *, requested_source="": network.PixhawkRouteEvidence(
            peer=peer,
            requested_source=requested_source,
            interface="enx-pixhawk",
            source="10.0.0.10",
        ),
    )

    result = network.validate_pixhawk_network_assignment(
        interface_name="enx-pixhawk",
        local_address="10.0.0.10/24",
        peer_host="10.0.0.3",
        endpoint="udpin:0.0.0.0:14551",
    )

    assert result.ok is True
    assert result.state == "ROUTE OK"


def test_pixhawk_assignment_settings_round_trip_and_legacy_defaults(tmp_path) -> None:
    from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings

    legacy_path = tmp_path / "legacy.json"
    legacy_path.write_text(
        json.dumps(
            {
                "left_mavlink_url": "udpin:0.0.0.0:14550",
                "right_mavlink_url": "udpin:0.0.0.0:14551",
            }
        ),
        encoding="utf-8",
    )
    legacy = load_settings(legacy_path)
    assert legacy.left_mavlink_interface == ""
    assert legacy.right_mavlink_local_address == ""

    assigned = AppSettings(
        left_mavlink_interface="enx-p1",
        left_mavlink_local_address="10.0.0.10",
        right_mavlink_interface="enx-p2",
        right_mavlink_local_address="10.0.0.12",
    )
    assigned_path = tmp_path / "assigned.json"
    save_settings(assigned, assigned_path)
    loaded = load_settings(assigned_path)
    assert loaded.left_mavlink_interface == "enx-p1"
    assert loaded.left_mavlink_local_address == "10.0.0.10"
    assert loaded.right_mavlink_interface == "enx-p2"
    assert loaded.right_mavlink_local_address == "10.0.0.12"


def test_pixhawk_network_discovery_worker_returns_immutable_candidate_rows(monkeypatch) -> None:
    from Workstation import device_runtime_workers
    from Workstation.workstation_core.network import (
        PixhawkNetworkCandidate,
        PixhawkRouteEvidence,
    )

    route = PixhawkRouteEvidence(
        peer="10.0.0.2",
        requested_source="10.0.0.10",
        interface="enx-pixhawk",
        source="10.0.0.10",
    )
    candidate = PixhawkNetworkCandidate(
        candidate_id="7:enx-pixhawk:10.0.0.10/24",
        interface="enx-pixhawk",
        ifindex=7,
        carrier="UP",
        local_cidr="10.0.0.10/24",
        local_ip="10.0.0.10",
        mac="00:11:22:33:44:55",
        driver="r8152",
        device_path="/sys/devices/usb/7",
        p1_route=route,
        p2_route=PixhawkRouteEvidence(
            peer="10.0.0.3",
            requested_source="10.0.0.10",
            interface="enx-pixhawk",
            source="10.0.0.10",
        ),
        p1_endpoint="udpin:0.0.0.0:14550",
        p2_endpoint="udpin:0.0.0.0:14551",
    )
    monkeypatch.setattr(
        device_runtime_workers,
        "discover_pixhawk_network_candidates",
        lambda **_kwargs: [candidate],
    )
    results: list[object] = []
    failures: list[str] = []
    worker = device_runtime_workers.PixhawkNetworkDiscoveryWorker(
        p1_peer="10.0.0.2",
        p2_peer="10.0.0.3",
        p1_endpoint="udpin:0.0.0.0:14550",
        p2_endpoint="udpin:0.0.0.0:14551",
    )
    worker.discovered.connect(results.append)
    worker.failed.connect(failures.append)

    worker.run()

    assert failures == []
    assert isinstance(results[0], tuple)
    assert results[0][0]["candidate_id"] == candidate.candidate_id
    assert results[0][0]["p1_route_state"] == "ROUTE OK"


def test_network_candidate_assign_persists_nic_address_endpoint_without_host_mutation() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    class Edit:
        def __init__(self, value: str) -> None:
            self.value = value

        def text(self) -> str:
            return self.value

        def setText(self, value: str) -> None:
            self.value = str(value)

    calls: list[object] = []
    candidate = {
        "candidate_id": "7:enx-pixhawk:10.0.0.10/24",
        "interface": "enx-pixhawk",
        "local_ip": "10.0.0.10",
        "p1_endpoint": "udpin:0.0.0.0:14550",
        "p2_endpoint": "udpin:0.0.0.0:14551",
        "p1_route_state": "ROUTE OK",
        "p2_route_state": "ROUTE OK",
        "p1_route": "ROUTE OK · enx-pixhawk src=10.0.0.10",
        "p2_route": "ROUTE OK · enx-pixhawk src=10.0.0.10",
    }
    panel = SimpleNamespace(
        _pixhawk_candidate_rows_by_source={"network": [candidate]},
        pixhawk_network_assignments={
            "left": {"interface": "", "local_address": ""},
            "right": {"interface": "", "local_address": ""},
        },
        pixhawk_network_assignment_status={"left": {}, "right": {}},
        _pending_pixhawk_network_assignment={},
        left_mavlink_url_edit=Edit("udpin:0.0.0.0:14550"),
        right_mavlink_url_edit=Edit("udpin:0.0.0.0:14551"),
        pixhawk_connection_label=SimpleNamespace(setText=lambda value: calls.append(("label", value))),
        _session_recording_active=lambda: False,
        _mavlink_open_sides=lambda: set(),
        _sync_mavlink_subscriber_settings=lambda: calls.append("sync"),
        _save_current_settings=lambda: calls.append("save"),
        _px4_mag_camera_label=lambda side: "P1" if side == "left" else "P2",
        _log=lambda message: calls.append(("log", message)),
        _refresh_operation_header=lambda: calls.append("refresh"),
    )

    applied = DeviceRuntimeController(panel).assign_pixhawk_network_candidate(
        candidate["candidate_id"], "left"
    )

    assert applied is True
    assert panel.pixhawk_network_assignments["left"] == {
        "interface": "enx-pixhawk",
        "local_address": "10.0.0.10",
    }
    assert panel.left_mavlink_url_edit.text() == "udpin:0.0.0.0:14550"
    assert "sync" in calls
    assert "save" in calls
    assert all(not (isinstance(call, tuple) and "ip addr" in str(call)) for call in calls)


def test_network_candidate_reassign_disconnects_only_the_live_side_before_commit() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    disconnects: list[str] = []
    saves: list[str] = []
    candidate = {
        "candidate_id": "9:enx-new:10.0.0.12/24",
        "interface": "enx-new",
        "local_ip": "10.0.0.12",
        "p1_endpoint": "udpin:0.0.0.0:14550",
        "p2_endpoint": "udpin:0.0.0.0:14551",
        "p1_route_state": "ROUTE OK",
        "p2_route_state": "ROUTE OK",
        "p1_route": "ROUTE OK",
        "p2_route": "ROUTE OK",
    }
    panel = SimpleNamespace(
        _pixhawk_candidate_rows_by_source={"network": [candidate]},
        pixhawk_network_assignments={"left": {}, "right": {}},
        pixhawk_network_assignment_status={"left": {}, "right": {}},
        _pending_pixhawk_network_assignment={},
        _session_recording_active=lambda: False,
        _mavlink_open_sides=lambda: {"left"},
        _px4_mag_camera_label=lambda side: "P1" if side == "left" else "P2",
        disconnect_mavlink_pixhawk=disconnects.append,
        _log=lambda _message: None,
        _save_current_settings=lambda: saves.append("save"),
    )

    applied = DeviceRuntimeController(panel).assign_pixhawk_network_candidate(
        candidate["candidate_id"], "left"
    )

    assert applied is True
    assert disconnects == ["left"]
    assert panel._pending_pixhawk_network_assignment["left"] == candidate
    assert panel.pixhawk_network_assignment_status["left"]["state"] == "DISCONNECTING TO REASSIGN"
    assert saves == []


def test_network_assignment_save_failure_restores_previous_mapping() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    class Edit:
        def __init__(self, value: str) -> None:
            self.value = value

        def text(self) -> str:
            return self.value

        def setText(self, value: str) -> None:
            self.value = str(value)

    panel = SimpleNamespace(
        pixhawk_network_assignments={
            "left": {"interface": "enx-old", "local_address": "10.0.0.10"},
            "right": {},
        },
        pixhawk_network_assignment_status={
            "left": {"state": "ASSIGNED", "detail": "old"},
            "right": {},
        },
        left_mavlink_url_edit=Edit("udpin:0.0.0.0:15550"),
        right_mavlink_url_edit=Edit("udpin:0.0.0.0:14551"),
        _sync_mavlink_subscriber_settings=lambda: None,
        _save_current_settings=lambda: (_ for _ in ()).throw(OSError("disk full")),
        _px4_mag_camera_label=lambda _side: "P1",
        _log=lambda _message: None,
        pixhawk_connection_label=SimpleNamespace(setText=lambda _value: None),
        _refresh_operation_header=lambda: None,
    )
    candidate = {
        "interface": "enx-new",
        "local_ip": "10.0.0.12",
        "p1_endpoint": "udpin:0.0.0.0:14550",
        "p1_route_state": "ROUTE OK",
        "p1_route": "ROUTE OK",
    }

    applied = DeviceRuntimeController(panel)._commit_pixhawk_network_assignment(
        "left", candidate
    )

    assert applied is False
    assert panel.pixhawk_network_assignments["left"] == {
        "interface": "enx-old",
        "local_address": "10.0.0.10",
    }
    assert panel.left_mavlink_url_edit.text() == "udpin:0.0.0.0:15550"
    assert panel.pixhawk_network_assignment_status["left"]["state"] == "ASSIGN FAILED"
    assert "disk full" in panel.pixhawk_network_assignment_status["left"]["detail"]


def test_connect_preflight_failure_opens_no_mavlink_worker() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController
    from Workstation.workstation_core.network import PixhawkAssignmentValidation

    logs: list[str] = []
    labels: list[str] = []
    sources: list[tuple[str, str, str]] = []
    panel = SimpleNamespace(
        _automatic_session_recording=False,
        _session_recording_active=lambda: False,
        pixhawk_disconnection_thread=None,
        pixhawk_connection_thread=None,
        _mavlink_target_sides=lambda _camera: ("left", "right"),
        _mavlink_open_sides=lambda: set(),
        _px4_mag_camera_label=lambda side: "P1" if side == "left" else "P2",
        measurement_source_transition=lambda source, state, detail: sources.append(
            (source, state, detail)
        ),
        pixhawk_connection_label=SimpleNamespace(setText=labels.append),
        _refresh_session_toggle_buttons=lambda: None,
        _refresh_operation_header=lambda: None,
        _log=logs.append,
    )
    controller = DeviceRuntimeController(panel)
    controller._pixhawk_network_preflight = lambda side: PixhawkAssignmentValidation(
        False,
        "ROUTE MISMATCH",
        f"{side} selected enx-pixhawk but kernel uses wlo1",
    )
    panel._ensure_mavlink_subscriber = lambda: (_ for _ in ()).throw(
        AssertionError("subscriber must not be created")
    )

    controller.connect_mavlink_pixhawk()

    assert labels and labels[-1].startswith("Pixhawk Connect preflight failed")
    assert sum("Connect preflight 실패" in message for message in logs) == 2
    assert [state for _source, state, _detail in sources].count("UNAVAILABLE") == 2


def test_connect_preflight_partitions_valid_and_invalid_sides_independently() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController
    from Workstation.workstation_core.network import PixhawkAssignmentValidation

    logs: list[str] = []
    sources: list[tuple[str, str, str]] = []
    panel = SimpleNamespace(
        _px4_mag_camera_label=lambda side: "P1" if side == "left" else "P2",
        measurement_source_transition=lambda source, state, detail: sources.append(
            (source, state, detail)
        ),
        _log=logs.append,
    )
    controller = DeviceRuntimeController(panel)
    controller._pixhawk_network_preflight = lambda side: (
        PixhawkAssignmentValidation(True, "ROUTE OK", "P1 route matches")
        if side == "left"
        else PixhawkAssignmentValidation(
            False,
            "ROUTE MISMATCH",
            "P2 selected enx-p2 but kernel uses wlo1",
        )
    )

    valid, errors = controller._pixhawk_connect_preflight_sides(("left", "right"))

    assert valid == ("left",)
    assert errors == {
        "right": "ROUTE MISMATCH: P2 selected enx-p2 but kernel uses wlo1"
    }
    assert sources == [
        (
            "pixhawk.right",
            "UNAVAILABLE",
            "ROUTE MISMATCH: P2 selected enx-p2 but kernel uses wlo1",
        )
    ]


def test_real_panel_find_network_runs_worker_and_publishes_network_rows(
    tmp_path, monkeypatch
) -> None:
    from Workstation import device_runtime_workers, standalone_gui
    from Workstation.workstation_core.network import (
        PixhawkNetworkCandidate,
        PixhawkRouteEvidence,
    )

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda _self: None
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda _self: None,
    )
    route_p1 = PixhawkRouteEvidence(
        peer="10.0.0.2",
        requested_source="10.0.0.10",
        interface="enx-pixhawk",
        source="10.0.0.10",
    )
    route_p2 = PixhawkRouteEvidence(
        peer="10.0.0.3",
        requested_source="10.0.0.10",
        interface="enx-pixhawk",
        source="10.0.0.10",
    )
    candidate = PixhawkNetworkCandidate(
        candidate_id="7:enx-pixhawk:10.0.0.10/24",
        interface="enx-pixhawk",
        ifindex=7,
        carrier="UP",
        local_cidr="10.0.0.10/24",
        local_ip="10.0.0.10",
        mac="00:11:22:33:44:55",
        driver="r8152",
        device_path="/sys/devices/usb/7",
        p1_route=route_p1,
        p2_route=route_p2,
        p1_endpoint="udpin:0.0.0.0:14550",
        p2_endpoint="udpin:0.0.0.0:14551",
    )
    monkeypatch.setattr(
        device_runtime_workers,
        "discover_pixhawk_network_candidates",
        lambda **_kwargs: [candidate],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(
        app_root=tmp_path,
        settings_path=tmp_path / "settings.json",
    )

    panel.discover_pixhawk_network()
    deadline = time.monotonic() + 2.0
    while panel.pixhawk_network_discovery_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)

    assert panel.pixhawk_network_discovery_thread is None
    rows = panel._pixhawk_candidate_rows_by_source["network"]
    assert len(rows) == 1
    assert rows[0]["candidate_id"] == candidate.candidate_id
    assert rows[0]["p1_route_state"] == "ROUTE OK"
    assert panel.workstation_pixhawk_discovery_status == "FOUND 1 NETWORK CHOICES"

    panel.shutdown_for_close()
    panel.deleteLater()
    app.processEvents()


def test_real_panel_assignment_persists_across_restart(tmp_path, monkeypatch) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda _self: None
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda _self: None,
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    settings_path = tmp_path / "settings.json"
    candidate = {
        "candidate_id": "7:enx-pixhawk:10.0.0.10/24",
        "kind": "NETWORK",
        "interface": "enx-pixhawk",
        "local_cidr": "10.0.0.10/24",
        "local_ip": "10.0.0.10",
        "p1_endpoint": "udpin:0.0.0.0:14550",
        "p2_endpoint": "udpin:0.0.0.0:14551",
        "p1_route_state": "ROUTE OK",
        "p2_route_state": "ROUTE OK",
        "p1_route": "ROUTE OK · enx-pixhawk src=10.0.0.10",
        "p2_route": "ROUTE OK · enx-pixhawk src=10.0.0.10",
    }
    panel = standalone_gui.WirisStereoDistancePanel(
        app_root=tmp_path,
        settings_path=settings_path,
    )
    panel._set_pixhawk_candidate_rows([candidate], source="network")

    assert panel.assign_pixhawk_network_candidate(candidate["candidate_id"], "left") is True
    assert panel.assign_pixhawk_network_candidate(candidate["candidate_id"], "right") is True
    assert settings_path.exists()
    panel.shutdown_for_close()
    panel.deleteLater()
    app.processEvents()

    restarted = standalone_gui.WirisStereoDistancePanel(
        app_root=tmp_path,
        settings_path=settings_path,
    )
    assert restarted.pixhawk_network_assignments == {
        "left": {"interface": "enx-pixhawk", "local_address": "10.0.0.10"},
        "right": {"interface": "enx-pixhawk", "local_address": "10.0.0.10"},
    }
    assert restarted.left_mavlink_url_edit.text() == "udpin:0.0.0.0:14550"
    assert restarted.right_mavlink_url_edit.text() == "udpin:0.0.0.0:14551"
    restarted.apply_workstation_system_settings(
        {"left_mavlink_url": "udpin:0.0.0.0:15550"}
    )
    assert restarted.pixhawk_network_assignments["left"] == {
        "interface": "",
        "local_address": "",
    }
    assert restarted.pixhawk_network_assignments["right"]["interface"] == "enx-pixhawk"
    assert restarted.pixhawk_network_assignment_status["left"]["state"] == "LEGACY AUTO ROUTE"
    restarted.shutdown_for_close()
    restarted.deleteLater()
    app.processEvents()
