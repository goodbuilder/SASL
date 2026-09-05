from __future__ import annotations

import socket
import threading
import time
from types import SimpleNamespace

import pytest

from Workstation.drone_telemetry_controller import DroneTelemetryController
from Workstation.workstation_core.drone_network_discovery import (
    discover_drone_network_candidates,
    mavlink_heartbeat_sources,
)


def _mavlink2_heartbeat(system_id: int, component_id: int = 1) -> bytes:
    payload = bytes(9)
    header = bytes((0xFD, len(payload), 0, 0, 0, system_id, component_id, 0, 0, 0))
    return header + payload + bytes(2)


def test_mavlink_heartbeat_parser_reads_v2_identity_and_ignores_truncated_data() -> None:
    heartbeat = _mavlink2_heartbeat(3)
    assert mavlink_heartbeat_sources(b"noise" + heartbeat) == ((3, 1),)
    assert mavlink_heartbeat_sources(heartbeat[:-1]) == ()


def test_passive_discovery_reports_source_port_listen_port_and_sysid() -> None:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    port = int(receiver.getsockname()[1])
    receiver.close()

    def send() -> None:
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            time.sleep(0.05)
            for _ in range(2):
                sender.sendto(_mavlink2_heartbeat(3), ("127.0.0.1", port))
                time.sleep(0.02)
        finally:
            sender.close()

    thread = threading.Thread(target=send)
    thread.start()
    candidates = discover_drone_network_candidates(
        timeout_s=0.25,
        ports=(port,),
        route_resolver=lambda _peer: ("lo", "127.0.0.1"),
    )
    thread.join()

    assert len(candidates) == 1
    candidate = candidates[0]
    assert candidate.system_id == 3
    assert candidate.listen_port == port
    assert candidate.interface == "lo"
    assert candidate.selectable
    assert candidate.endpoint == f"udpin:0.0.0.0:{port}"


def test_candidate_selection_rejects_non_drone_and_accepts_current_sysid3() -> None:
    panel = SimpleNamespace(
        drone_telemetry_network_candidates=(
            {
                "candidate_id": "p2",
                "system_id": 2,
                "selectable": False,
                "endpoint": "udpin:0.0.0.0:14551",
            },
            {
                "candidate_id": "drone",
                "system_id": 3,
                "selectable": True,
                "endpoint": "udpin:0.0.0.0:14552",
            },
        )
    )
    controller = DroneTelemetryController(panel)

    with pytest.raises(ValueError, match="not verified"):
        controller.use_drone_telemetry_network_candidate("p2")
    assert controller.use_drone_telemetry_network_candidate("drone")["endpoint"] == (
        "udpin:0.0.0.0:14552"
    )
