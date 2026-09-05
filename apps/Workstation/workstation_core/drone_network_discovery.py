from __future__ import annotations

import select
import socket
import time
from dataclasses import dataclass
from typing import Callable, Iterable


DEFAULT_DRONE_MAVLINK_PORTS = tuple(range(14550, 14560))


@dataclass(frozen=True)
class DroneNetworkCandidate:
    candidate_id: str
    source_ip: str
    source_port: int
    listen_port: int
    system_id: int
    component_id: int
    heartbeat_count: int
    interface: str = ""
    route_source: str = ""

    @property
    def selectable(self) -> bool:
        return self.system_id == 3 and self.heartbeat_count > 0

    @property
    def endpoint(self) -> str:
        return f"udpin:0.0.0.0:{self.listen_port}"

    def to_mapping(self) -> dict[str, object]:
        return {
            "candidate_id": self.candidate_id,
            "source_ip": self.source_ip,
            "source_port": self.source_port,
            "listen_port": self.listen_port,
            "system_id": self.system_id,
            "component_id": self.component_id,
            "heartbeat_count": self.heartbeat_count,
            "interface": self.interface,
            "route_source": self.route_source,
            "endpoint": self.endpoint,
            "selectable": self.selectable,
            "state": "DRONE VERIFIED" if self.selectable else "OTHER SYSTEM",
        }


def mavlink_heartbeat_sources(payload: bytes) -> tuple[tuple[int, int], ...]:
    """Return MAVLink HEARTBEAT source IDs from one raw UDP datagram."""

    found: list[tuple[int, int]] = []
    offset = 0
    size = len(payload)
    while offset < size:
        magic = payload[offset]
        if magic == 0xFE and offset + 8 <= size:
            payload_len = payload[offset + 1]
            frame_len = payload_len + 8
            if offset + frame_len > size:
                break
            if payload[offset + 5] == 0:
                found.append((int(payload[offset + 3]), int(payload[offset + 4])))
            offset += frame_len
            continue
        if magic == 0xFD and offset + 12 <= size:
            payload_len = payload[offset + 1]
            signed = bool(payload[offset + 2] & 0x01)
            frame_len = payload_len + 12 + (13 if signed else 0)
            if offset + frame_len > size:
                break
            message_id = int.from_bytes(payload[offset + 7 : offset + 10], "little")
            if message_id == 0:
                found.append((int(payload[offset + 5]), int(payload[offset + 6])))
            offset += frame_len
            continue
        offset += 1
    return tuple(found)


def discover_drone_network_candidates(
    *,
    timeout_s: float = 3.0,
    ports: Iterable[int] = DEFAULT_DRONE_MAVLINK_PORTS,
    route_resolver: Callable[[str], tuple[str, str]] | None = None,
) -> tuple[DroneNetworkCandidate, ...]:
    """Passively observe bounded UDP ports without changing host or PX4 state."""

    timeout = max(0.05, float(timeout_s))
    sockets: list[socket.socket] = []
    listen_ports: dict[socket.socket, int] = {}
    bind_errors: list[str] = []
    try:
        for raw_port in ports:
            port = int(raw_port)
            if not 1 <= port <= 65535:
                continue
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(("0.0.0.0", port))
            except OSError as exc:
                bind_errors.append(f"{port}: {exc}")
                sock.close()
                continue
            sock.setblocking(False)
            sockets.append(sock)
            listen_ports[sock] = port
        if not sockets:
            detail = "; ".join(bind_errors) or "no valid UDP ports"
            raise RuntimeError(f"Drone network discovery could not bind: {detail}")

        counts: dict[tuple[str, int, int, int, int], int] = {}
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select(sockets, (), (), min(0.25, remaining))
            for sock in readable:
                try:
                    payload, address = sock.recvfrom(65535)
                except BlockingIOError:
                    continue
                source_ip, source_port = str(address[0]), int(address[1])
                for system_id, component_id in mavlink_heartbeat_sources(payload):
                    key = (
                        source_ip,
                        source_port,
                        listen_ports[sock],
                        system_id,
                        component_id,
                    )
                    counts[key] = counts.get(key, 0) + 1

        candidates: list[DroneNetworkCandidate] = []
        for key, heartbeat_count in sorted(counts.items()):
            source_ip, source_port, listen_port, system_id, component_id = key
            interface = ""
            route_source = ""
            if route_resolver is not None:
                interface, route_source = route_resolver(source_ip)
            candidate_id = (
                f"{source_ip}:{source_port}>{listen_port}/"
                f"{system_id}:{component_id}"
            )
            candidates.append(
                DroneNetworkCandidate(
                    candidate_id=candidate_id,
                    source_ip=source_ip,
                    source_port=source_port,
                    listen_port=listen_port,
                    system_id=system_id,
                    component_id=component_id,
                    heartbeat_count=heartbeat_count,
                    interface=interface,
                    route_source=route_source,
                )
            )
        return tuple(candidates)
    finally:
        for sock in sockets:
            sock.close()
