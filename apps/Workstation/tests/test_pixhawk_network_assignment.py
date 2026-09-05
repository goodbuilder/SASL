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
    assert headers == ["Source", "NIC / Device", "Link / Kind", "Evidence", "Role"]
    assert window.pixhawk.candidate_model.item(0, 4).text() == "P1"
    assert window.pixhawk.transport_combos["left"].currentText() == "P1 Telemetry Radio"
    assert window.pixhawk.transport_combos["right"].currentText() == "P2 Telemetry Radio"
    assert "drone" not in window.pixhawk.transport_combos
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


def test_usb_candidate_role_separates_saved_assignment_from_observed_identity(
    tmp_path,
) -> None:
    from Workstation.workspaces import PixhawkWorkspace

    resolved = tmp_path / "ttyUSB1"
    resolved.touch()
    stable = tmp_path / "p2-radio-by-id"
    stable.symlink_to(resolved)
    candidate = {
        "source": "mavlink",
        "kind": "USB",
        "value": str(stable),
        "resolved_device": str(resolved),
        "system_id": 2,
    }
    panel = SimpleNamespace(
        left_pixhawk_radio_device="",
        right_pixhawk_radio_device=str(resolved),
        drone_pixhawk_radio_device="",
    )

    assert PixhawkWorkspace._candidate_role_label(candidate, panel, {}) == "P2 saved · sysid 2"

    panel.left_pixhawk_radio_device = str(stable)
    assert PixhawkWorkspace._candidate_role_label(candidate, panel, {}) == (
        "P1, P2 saved · P2 (sysid 2)"
    )

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
    panel._pixhawk_candidate_rows_by_source = {
        "network": [{"kind": "NETWORK", "candidate_id": "stale-network"}],
        "pc": [{"kind": "PC_IP", "value": "192.0.2.10"}],
        "mavlink": [{"kind": "USB", "value": "/dev/stale-radio"}],
    }
    panel._sync_legacy_pixhawk_candidate_rows()
    saved_assignments = {
        side: dict(value)
        for side, value in panel.pixhawk_network_assignments.items()
    }
    revision = panel.pixhawk_discovery_revision

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
    assert panel._pixhawk_candidate_rows_by_source["pc"] == []
    assert panel._pixhawk_candidate_rows_by_source["mavlink"] == []
    assert panel.pixhawk_discovery_revision == revision + 1
    assert panel.pixhawk_network_assignments == saved_assignments
    assert panel.workstation_pixhawk_discovery_status == "FOUND 1 NETWORK CHOICES"

    panel.shutdown_for_close()
    panel.deleteLater()
    app.processEvents()


def test_real_panel_find_usb_replaces_snapshot_and_keeps_one_row_per_device(
    tmp_path, monkeypatch
) -> None:
    from Workstation import device_runtime_workers, standalone_gui
    from Workstation.workstation_core.pixhawk_attitude import UsbMavlinkIdentity

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda _self: None
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda _self: None,
    )
    devices = ["/dev/serial/by-id/p1-radio", "/dev/serial/by-id/p2-radio"]
    monkeypatch.setattr(
        device_runtime_workers,
        "usb_mavlink_device_candidates",
        lambda: list(devices),
    )
    monkeypatch.setattr(
        device_runtime_workers,
        "probe_usb_mavlink_devices",
        lambda *, candidates, timeout_s: [
            UsbMavlinkIdentity(
                device=candidates[1],
                resolved_device="/dev/ttyUSB1",
                baud=57_600,
                system_id=2,
                component_id=1,
                observed_monotonic_s=123.5,
            )
        ],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(
        app_root=tmp_path,
        settings_path=tmp_path / "settings.json",
    )
    panel._pixhawk_candidate_rows_by_source = {
        "network": [{"kind": "NETWORK", "candidate_id": "stale-network"}],
        "pc": [{"kind": "PC_IP", "value": "192.0.2.10"}],
        "mavlink": [{"kind": "USB", "value": "/dev/stale-radio"}],
    }
    panel._sync_legacy_pixhawk_candidate_rows()
    panel.left_pixhawk_radio_device = "/dev/saved-p1-radio"
    panel.right_pixhawk_radio_device = "/dev/saved-p2-radio"
    saved_assignments = {
        side: dict(value)
        for side, value in panel.pixhawk_network_assignments.items()
    }
    revision = panel.pixhawk_discovery_revision

    panel.discover_usb_pixhawk_connections()
    deadline = time.monotonic() + 2.0
    while panel.pixhawk_usb_discovery_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)

    assert panel.pixhawk_usb_discovery_thread is None
    assert panel._pixhawk_candidate_rows_by_source["network"] == []
    assert panel._pixhawk_candidate_rows_by_source["pc"] == []
    rows = panel._pixhawk_candidate_rows_by_source["mavlink"]
    assert [row["value"] for row in rows] == devices
    assert [row["kind"] for row in rows] == ["USB", "USB"]
    assert rows[1]["resolved_device"] == "/dev/ttyUSB1"
    assert rows[1]["system_id"] == 2
    assert rows[1]["component_id"] == 1
    assert panel.pixhawk_discovery_revision == revision + 1
    assert panel.left_pixhawk_radio_device == "/dev/saved-p1-radio"
    assert panel.right_pixhawk_radio_device == "/dev/saved-p2-radio"
    assert panel.pixhawk_network_assignments == saved_assignments
    assert panel.workstation_pixhawk_discovery_status == "FOUND 2 USB ENDPOINTS"

    panel.shutdown_for_close()
    panel.deleteLater()
    app.processEvents()


def test_duplicate_usb_discovery_does_not_clear_the_active_snapshot() -> None:
    from Workstation import standalone_gui

    logs: list[str] = []
    panel = SimpleNamespace(
        pixhawk_usb_discovery_thread=object(),
        _log=logs.append,
        _begin_pixhawk_candidate_discovery=lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("active discovery must not be reset")
        ),
    )

    standalone_gui.WirisStereoDistancePanel.discover_usb_pixhawk_connections(panel)

    assert logs == ["[픽스호크] USB Pixhawk 연결 탐색이 이미 진행 중입니다."]


def test_pixhawk_host_discovery_cannot_join_an_active_candidate_snapshot() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    logs: list[str] = []
    panel = SimpleNamespace(
        pixhawk_host_ip_discovery_thread=None,
        _pixhawk_candidate_discovery_active=lambda: True,
        _log=logs.append,
    )

    DeviceRuntimeController(panel).discover_pc_pixhawk_ip()

    assert logs == [
        "[픽스호크] 다른 Discovery가 진행 중이어서 PC IP 탐색을 시작하지 않았습니다."
    ]


def test_begin_pixhawk_discovery_clears_only_transient_evidence() -> None:
    from Workstation import standalone_gui

    replacements: list[tuple[list[dict[str, object]], object]] = []
    panel = SimpleNamespace(
        pixhawk_discovery_revision=4,
        pixhawk_usb_identity_evidence=[{"system_id": 1}],
        left_pixhawk_radio_device="/dev/saved-p1-radio",
        pixhawk_network_assignments={"left": {"interface": "enx-pixhawk"}},
        workstation_pixhawk_discovery_status="FOUND 9 ENDPOINTS",
        _set_pixhawk_candidate_rows=lambda rows, source=None: replacements.append(
            (list(rows), source)
        ),
    )

    standalone_gui.WirisStereoDistancePanel._begin_pixhawk_candidate_discovery(
        panel,
        "SEARCHING USB",
        clear_usb_identity=True,
    )

    assert replacements == [([], None)]
    assert panel.pixhawk_usb_identity_evidence == []
    assert panel.pixhawk_discovery_revision == 5
    assert panel.workstation_pixhawk_discovery_status == "SEARCHING USB"
    assert panel.left_pixhawk_radio_device == "/dev/saved-p1-radio"
    assert panel.pixhawk_network_assignments == {"left": {"interface": "enx-pixhawk"}}


def test_pixhawk_discovery_failures_cannot_restore_stale_rows() -> None:
    from Workstation import standalone_gui
    from Workstation.device_runtime_controller import DeviceRuntimeController

    usb_replacements: list[list[dict[str, object]]] = []
    usb_logs: list[str] = []
    usb_panel = SimpleNamespace(
        pixhawk_usb_identity_evidence=[{"system_id": 2}],
        workstation_pixhawk_discovery_status="SEARCHING USB",
        _set_pixhawk_candidate_rows=lambda rows: usb_replacements.append(list(rows)),
        _log=usb_logs.append,
    )

    standalone_gui.WirisStereoDistancePanel._pixhawk_usb_discovery_failed(
        usb_panel,
        "probe failed",
    )

    assert usb_replacements == [[]]
    assert usb_panel.pixhawk_usb_identity_evidence == []
    assert usb_panel.workstation_pixhawk_discovery_status == "ERROR · probe failed"

    network_replacements: list[list[dict[str, object]]] = []
    network_logs: list[str] = []
    connection_labels: list[str] = []
    network_panel = SimpleNamespace(
        workstation_pixhawk_discovery_status="SEARCHING NETWORK",
        _set_pixhawk_candidate_rows=lambda rows: network_replacements.append(list(rows)),
        pixhawk_connection_label=SimpleNamespace(setText=connection_labels.append),
        _log=network_logs.append,
        _refresh_operation_header=lambda: None,
    )

    DeviceRuntimeController(network_panel)._pixhawk_network_discovery_failed(
        "route read failed"
    )

    assert network_replacements == [[]]
    assert network_panel.workstation_pixhawk_discovery_status == "ERROR · route read failed"
    assert connection_labels == ["Pixhawk network discovery failed: route read failed"]


def test_empty_usb_discovery_publishes_an_empty_current_snapshot() -> None:
    from Workstation import standalone_gui

    replacements: list[tuple[list[dict[str, object]], object]] = []
    logs: list[str] = []
    panel = SimpleNamespace(
        pixhawk_usb_identity_evidence=[{"system_id": 1}],
        workstation_pixhawk_discovery_status="SEARCHING USB",
        _set_pixhawk_candidate_rows=lambda rows, source=None: replacements.append(
            (list(rows), source)
        ),
        _log=logs.append,
    )

    standalone_gui.WirisStereoDistancePanel._apply_pixhawk_usb_discovery(
        panel,
        {"candidates": [], "found": []},
    )

    assert replacements == [([], None)]
    assert panel.pixhawk_usb_identity_evidence == []
    assert panel.workstation_pixhawk_discovery_status == "FOUND 0 USB ENDPOINTS"
    assert logs == [
        "[픽스호크] USB Pixhawk 장치가 보이지 않습니다: /dev/ttyACM*, /dev/ttyUSB* 없음"
    ]


def test_real_panel_assignment_persists_across_restart(tmp_path, monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers
    from Workstation.workstation_core.network import (
        NetworkRoleInterfaceResult,
        NetworkRoleOwnedEntries,
    )

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda _self: None
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda _self: None,
    )
    monkeypatch.setattr(
        device_runtime_workers,
        "apply_network_role_transaction",
        lambda requests: tuple(
            NetworkRoleInterfaceResult(
                request.interface,
                "fake-profile",
                request.desired,
                NetworkRoleOwnedEntries(
                    request.desired.addresses,
                    request.desired.routes,
                ),
                request.desired.addresses,
                request.desired.routes,
            )
            for request in requests
        ),
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

    def wait_for_role_operation() -> None:
        deadline = time.monotonic() + 2.0
        while panel.network_role_operation is not None and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.005)
        assert panel.network_role_operation is None

    assert panel.assign_pixhawk_network_candidate(candidate["candidate_id"], "left") is True
    wait_for_role_operation()
    assert panel.assign_pixhawk_network_candidate(candidate["candidate_id"], "right") is True
    wait_for_role_operation()
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


def _renamed_nic_panel(saved_stable_id: str) -> SimpleNamespace:
    """One panel whose saved P1 NIC came back under a different kernel name."""

    calls: list[object] = []
    panel = SimpleNamespace(
        calls=calls,
        _pixhawk_candidate_rows_by_source={"network": [], "pc": [], "mavlink": []},
        _set_pixhawk_candidate_rows=lambda rows, source=None: panel.__setattr__(
            "published_rows", list(rows)
        ),
        workstation_pixhawk_discovery_status="",
        pixhawk_network_assignments={
            "left": {"interface": "enx-old", "local_address": "10.0.0.10"},
            "right": {"interface": "", "local_address": ""},
        },
        pixhawk_network_assignment_status={"left": {}, "right": {}},
        network_role_assignments={
            "camera": {"interface": "enx-cam", "stable_id": "aa:bb:cc:dd:ee:ff"},
            "p1": {"interface": "enx-old", "stable_id": saved_stable_id},
            "p2": {"interface": "", "stable_id": ""},
        },
        settings=SimpleNamespace(
            left_mavlink_interface_stable_id=saved_stable_id,
            right_mavlink_interface_stable_id="",
        ),
        pixhawk_connection_label=SimpleNamespace(setText=lambda value: calls.append(("label", value))),
        _px4_mag_camera_label=lambda side: "P1" if side == "left" else "P2",
        _save_current_settings=lambda: calls.append("save"),
        _log=lambda message: calls.append(("log", message)),
        _refresh_operation_header=lambda: calls.append("refresh"),
    )
    return panel


def test_renamed_nic_keeps_pixhawk_assignment_through_saved_mac() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = _renamed_nic_panel("00:11:22:33:44:55")
    controller = DeviceRuntimeController(panel)

    controller._apply_pixhawk_network_discovery(
        (
            {
                "candidate_id": "9:enx-new:10.0.0.10/24",
                "interface": "enx-new",
                "mac": "00:11:22:33:44:55",
                "carrier": "UP",
                "local_cidr": "10.0.0.10/24",
                "local_ip": "10.0.0.10",
                "p1_route_state": "ROUTE OK",
                "p2_route_state": "ROUTE OK",
                "p1_route": "ROUTE OK · enx-new src=10.0.0.10",
                "p2_route": "ROUTE OK · enx-new src=10.0.0.10",
            },
        )
    )

    status = panel.pixhawk_network_assignment_status["left"]
    assert status["state"] == "ASSIGNED"
    assert "enx-old -> enx-new" in status["detail"]
    assert panel.pixhawk_network_assignments["left"]["interface"] == "enx-new"
    assert panel.network_role_assignments["p1"] == {
        "interface": "enx-new",
        "stable_id": "00:11:22:33:44:55",
    }
    assert "save" in panel.calls


def test_renamed_nic_without_saved_mac_still_requires_reassignment() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = _renamed_nic_panel("")
    controller = DeviceRuntimeController(panel)

    controller._apply_pixhawk_network_discovery(
        (
            {
                "candidate_id": "9:enx-new:10.0.0.10/24",
                "interface": "enx-new",
                "mac": "00:11:22:33:44:55",
                "carrier": "UP",
                "local_cidr": "10.0.0.10/24",
                "local_ip": "10.0.0.10",
                "p1_route_state": "ROUTE OK",
                "p2_route_state": "ROUTE OK",
                "p1_route": "ROUTE OK",
                "p2_route": "ROUTE OK",
            },
        )
    )

    assert panel.pixhawk_network_assignment_status["left"]["state"] == "REASSIGN REQUIRED"
    assert panel.pixhawk_network_assignments["left"]["interface"] == "enx-old"
    assert "save" not in panel.calls


def test_saved_mac_present_but_address_moved_names_the_current_nic() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = _renamed_nic_panel("00:11:22:33:44:55")
    controller = DeviceRuntimeController(panel)

    controller._apply_pixhawk_network_discovery(
        (
            {
                "candidate_id": "9:enx-new:192.168.5.4/24",
                "interface": "enx-new",
                "mac": "00:11:22:33:44:55",
                "carrier": "UP",
                "local_cidr": "192.168.5.4/24",
                "local_ip": "192.168.5.4",
                "p1_route_state": "NO ROUTE",
                "p2_route_state": "NO ROUTE",
                "p1_route": "NO ROUTE",
                "p2_route": "NO ROUTE",
            },
        )
    )

    status = panel.pixhawk_network_assignment_status["left"]
    assert status["state"] == "REASSIGN REQUIRED"
    assert "00:11:22:33:44:55" in status["detail"]
    assert "enx-new" in status["detail"] and "192.168.5.4/24" in status["detail"]
    assert panel.pixhawk_network_assignments["left"]["interface"] == "enx-old"


def test_pixhawk_workspace_role_column_follows_mac_not_interface_name() -> None:
    from Workstation.workspaces import PixhawkWorkspace

    network_roles = {
        "camera": {"interface": "enx-old", "stable_id": "aa:bb:cc:dd:ee:ff"},
        "p1": {"interface": "enx-old", "stable_id": "00:11:22:33:44:55"},
        "p2": {"interface": "", "stable_id": ""},
    }
    renamed = {
        "source": "network",
        "interface": "enx-new",
        "mac": "00:11:22:33:44:55",
    }
    recycled_name = {
        "source": "network",
        "interface": "enx-old",
        "mac": "de:ad:be:ef:00:01",
    }

    assert PixhawkWorkspace._candidate_role_label(renamed, None, network_roles) == "P1"
    assert PixhawkWorkspace._candidate_role_label(recycled_name, None, network_roles) == "--"


def test_pixhawk_workspace_role_column_falls_back_to_name_without_stable_id() -> None:
    from Workstation.workspaces import PixhawkWorkspace

    network_roles = {
        "camera": {"interface": "", "stable_id": ""},
        "p1": {"interface": "enx-legacy", "stable_id": ""},
        "p2": {"interface": "", "stable_id": ""},
    }
    row = {"source": "network", "interface": "enx-legacy", "mac": "00:11:22:33:44:55"}

    assert PixhawkWorkspace._candidate_role_label(row, None, network_roles) == "P1"
