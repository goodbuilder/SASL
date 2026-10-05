from __future__ import annotations

import json
import os
import re
import socket
import subprocess
import shutil
from collections.abc import Iterable, Mapping, Sequence
from concurrent.futures import FIRST_COMPLETED, Future, ThreadPoolExecutor, as_completed, wait
from dataclasses import dataclass
from ipaddress import IPv4Address, IPv4Interface, IPv4Network, ip_interface
from pathlib import Path


DEFAULT_INTERFACE = "enx588694fce07e"
DEFAULT_HOST_ADDRESS = "10.0.0.10/24"
DEFAULT_CAMERA_URLS = (
    "rtsp://10.0.0.231:8554/visible",
    "rtsp://10.0.0.232:8554/visible",
)
LEGACY_CAMERA_URLS = (
    "rtsp://10.0.0.230:8554/visible",
)
DEFAULT_RTSP_PORTS = (8554, 554, 8555, 8556, 8080, 8081, 8000, 5000, 5004)
DEFAULT_RTSP_PATHS = ("visible", "stream1", "stream2", "live", "h264", "video", "cam/realmonitor")
DEFAULT_RTSP_SCAN_WORKERS = 64
DEFAULT_TCP_PORT_SCAN_WORKERS = 256
PKEXEC_TIMEOUT_SECONDS = 15.0
INTERFACE_STATUS_TIMEOUT_SECONDS = 2.0
PIXHAWK_NETWORK_COMMAND_TIMEOUT_SECONDS = 2.0
NETWORK_ROLE_APPLY_TIMEOUT_SECONDS = 60.0
NETWORK_ROLE_CAMERA = "camera"
NETWORK_ROLE_P1 = "p1"
NETWORK_ROLE_P2 = "p2"
NETWORK_ROLE_ORDER = (NETWORK_ROLE_CAMERA, NETWORK_ROLE_P1, NETWORK_ROLE_P2)
NETWORK_ROLE_LABELS = {
    NETWORK_ROLE_CAMERA: "Camera",
    NETWORK_ROLE_P1: "P1",
    NETWORK_ROLE_P2: "P2",
}


@dataclass(frozen=True)
class InterfaceCandidate:
    name: str
    address: IPv4Interface
    # Hardware address so a saved role can follow the adapter across a kernel
    # rename; empty for rows recorded before this identity existed.
    mac: str = ""


@dataclass(frozen=True)
class RtspCandidate:
    url: str
    host: str
    port: int
    interface: InterfaceCandidate | None = None


@dataclass(frozen=True)
class TcpPortCandidate:
    host: str
    port: int
    interface: InterfaceCandidate | None = None


@dataclass(frozen=True)
class HostInterfaceCandidate:
    interface: InterfaceCandidate
    neighbor_hosts: tuple[str, ...] = ()


@dataclass(frozen=True)
class ArpScanReport:
    interface: InterfaceCandidate | None
    network: str
    available: bool
    returncode: int | None
    hosts: tuple[str, ...] = ()
    error: str = ""


@dataclass(frozen=True)
class PixhawkRouteEvidence:
    """Kernel-selected route for one configured Pixhawk peer.

    This is readback evidence only.  Discovery and validation never add an
    address, route, NetworkManager profile, or default gateway.
    """

    peer: str
    requested_source: str = ""
    interface: str = ""
    source: str = ""
    gateway: str = ""
    error: str = ""

    @property
    def available(self) -> bool:
        return bool(self.interface) and not self.error

    def status_for(self, interface_name: str, local_ip: str = "") -> str:
        if not self.available:
            return "NO ROUTE"
        if self.interface != str(interface_name):
            return "ROUTE MISMATCH"
        if local_ip and self.source and self.source != str(local_ip):
            return "SOURCE MISMATCH"
        return "ROUTE OK"

    def summary_for(self, interface_name: str, local_ip: str = "") -> str:
        state = self.status_for(interface_name, local_ip)
        if state != "ROUTE OK":
            suffix = f" · {self.error}" if self.error else ""
            if self.interface:
                suffix += f" · dev={self.interface} src={self.source or '--'}"
            return f"{state}{suffix}"
        via = f" via {self.gateway}" if self.gateway else ""
        return f"ROUTE OK · {self.interface} src={self.source or '--'}{via}"


@dataclass(frozen=True)
class PixhawkNetworkCandidate:
    """One operator-selectable wired NIC/local-address combination."""

    candidate_id: str
    interface: str
    ifindex: int | None
    carrier: str
    local_cidr: str
    local_ip: str
    mac: str
    driver: str
    device_path: str
    p1_route: PixhawkRouteEvidence
    p2_route: PixhawkRouteEvidence
    p1_endpoint: str
    p2_endpoint: str

    def to_mapping(self) -> dict[str, object]:
        return {
            "candidate_id": self.candidate_id,
            "interface": self.interface,
            "ifindex": self.ifindex,
            "carrier": self.carrier,
            "local_cidr": self.local_cidr,
            "local_ip": self.local_ip,
            "mac": self.mac,
            "driver": self.driver,
            "device_path": self.device_path,
            "p1_route": self.p1_route.summary_for(self.interface, self.local_ip),
            "p2_route": self.p2_route.summary_for(self.interface, self.local_ip),
            "p1_route_state": self.p1_route.status_for(self.interface, self.local_ip),
            "p2_route_state": self.p2_route.status_for(self.interface, self.local_ip),
            "p1_route_interface": self.p1_route.interface,
            "p2_route_interface": self.p2_route.interface,
            "p1_route_source": self.p1_route.source,
            "p2_route_source": self.p2_route.source,
            "p1_endpoint": self.p1_endpoint,
            "p2_endpoint": self.p2_endpoint,
        }


@dataclass(frozen=True)
class PixhawkAssignmentValidation:
    ok: bool
    state: str
    detail: str
    route: PixhawkRouteEvidence | None = None


@dataclass(frozen=True)
class NetworkRoleAssignment:
    """Operator-owned placement of one logical device role on a physical NIC."""

    role: str
    interface: str
    stable_id: str = ""


@dataclass(frozen=True)
class NetworkRoleRequirements:
    """Canonical app-owned IPv4 entries required by a set of NIC roles."""

    roles: tuple[str, ...]
    addresses: tuple[str, ...]
    routes: tuple[str, ...]


@dataclass(frozen=True)
class NetworkRoleMovePlan:
    """Pure before/after role placement used by the privileged transaction owner."""

    role: str
    source_interface: str
    target_interface: str
    before_assignments: tuple[NetworkRoleAssignment, ...]
    after_assignments: tuple[NetworkRoleAssignment, ...]

    @property
    def affected_interfaces(self) -> tuple[str, ...]:
        if self.source_interface and self.source_interface != self.target_interface:
            return self.target_interface, self.source_interface
        return (self.target_interface,)

    def roles_by_interface(self, *, after: bool = True) -> dict[str, tuple[str, ...]]:
        assignments = self.after_assignments if after else self.before_assignments
        grouped: dict[str, list[str]] = {}
        for assignment in assignments:
            if assignment.interface:
                grouped.setdefault(assignment.interface, []).append(assignment.role)
        return {
            interface: tuple(role for role in NETWORK_ROLE_ORDER if role in roles)
            for interface, roles in grouped.items()
        }


@dataclass(frozen=True)
class NetworkRoleOwnedEntries:
    """Exact NetworkManager entries created by Workstation on one NIC."""

    addresses: tuple[str, ...] = ()
    routes: tuple[str, ...] = ()

    @classmethod
    def from_mapping(cls, value: object) -> NetworkRoleOwnedEntries:
        data = value if isinstance(value, Mapping) else {}
        return cls(
            tuple(str(item) for item in (data.get("addresses") or ()) if str(item)),
            tuple(str(item) for item in (data.get("routes") or ()) if str(item)),
        )

    def to_mapping(self) -> dict[str, list[str]]:
        return {"addresses": list(self.addresses), "routes": list(self.routes)}


@dataclass(frozen=True)
class NetworkRoleEntryDelta:
    addresses_after: tuple[str, ...]
    routes_after: tuple[str, ...]
    owned_after: NetworkRoleOwnedEntries
    added_addresses: tuple[str, ...]
    added_routes: tuple[str, ...]
    removed_addresses: tuple[str, ...]
    removed_routes: tuple[str, ...]


@dataclass(frozen=True)
class NetworkManagerConnectionSnapshot:
    interface: str
    connection_name: str
    exists: bool
    active: bool
    addresses: tuple[str, ...] = ()
    routes: tuple[str, ...] = ()
    ipv4_method: str = "manual"
    never_default: str = "yes"


@dataclass(frozen=True)
class NetworkRoleInterfaceRequest:
    interface: str
    stable_id: str
    desired: NetworkRoleRequirements
    owned_before: NetworkRoleOwnedEntries = NetworkRoleOwnedEntries()
    adoptable_before: NetworkRoleRequirements = NetworkRoleRequirements((), (), ())


@dataclass(frozen=True)
class NetworkRoleInterfaceResult:
    interface: str
    connection_name: str
    desired: NetworkRoleRequirements
    owned_after: NetworkRoleOwnedEntries
    addresses: tuple[str, ...]
    routes: tuple[str, ...]


def _ordered_union(base: Sequence[str], additions: Iterable[str]) -> tuple[str, ...]:
    values = list(base)
    for value in additions:
        if value not in values:
            values.append(value)
    return tuple(values)


def network_role_entry_delta(
    *,
    current_addresses: Sequence[str],
    current_routes: Sequence[str],
    owned_before: NetworkRoleOwnedEntries,
    desired: NetworkRoleRequirements,
) -> NetworkRoleEntryDelta:
    """Calculate a role-scoped delta without claiming pre-existing entries."""

    desired_addresses = set(desired.addresses)
    desired_routes = set(desired.routes)
    owned_addresses = set(owned_before.addresses)
    owned_routes = set(owned_before.routes)

    removed_addresses = tuple(
        value for value in current_addresses if value in owned_addresses - desired_addresses
    )
    removed_routes = tuple(
        value for value in current_routes if value in owned_routes - desired_routes
    )
    retained_addresses = tuple(value for value in current_addresses if value not in removed_addresses)
    retained_routes = tuple(value for value in current_routes if value not in removed_routes)
    added_addresses = tuple(value for value in desired.addresses if value not in retained_addresses)
    added_routes = tuple(value for value in desired.routes if value not in retained_routes)
    addresses_after = _ordered_union(retained_addresses, added_addresses)
    routes_after = _ordered_union(retained_routes, added_routes)
    owned_after = NetworkRoleOwnedEntries(
        tuple(
            value
            for value in desired.addresses
            if value in owned_addresses or value in added_addresses
        ),
        tuple(
            value
            for value in desired.routes
            if value in owned_routes or value in added_routes
        ),
    )
    return NetworkRoleEntryDelta(
        addresses_after,
        routes_after,
        owned_after,
        added_addresses,
        added_routes,
        removed_addresses,
        removed_routes,
    )


def _normalize_network_role(role: object) -> str:
    normalized = str(role or "").strip().lower()
    if normalized not in NETWORK_ROLE_ORDER:
        raise ValueError(f"unsupported network role: {role}")
    return normalized


def _canonical_role_address(value: object, *, default_prefix: int) -> IPv4Interface:
    text = str(value or "").strip()
    if not text:
        raise ValueError("network role address is empty")
    if "/" not in text:
        text = f"{text}/{int(default_prefix)}"
    address = ip_interface(text)
    if not isinstance(address, IPv4Interface):
        raise ValueError(f"network role address is not IPv4: {value}")
    return address


def _canonical_role_peer(value: object) -> str:
    return str(IPv4Address(str(value or "").strip()))


def network_role_requirements(
    roles: Iterable[object],
    *,
    camera_local_address: object = "10.0.0.11/32",
    camera_peers: Iterable[object] = ("10.0.0.231", "10.0.0.232"),
    pixhawk_local_address: object = "10.0.0.10/24",
    p1_peer: object = "10.0.0.2",
    p2_peer: object = "10.0.0.3",
) -> NetworkRoleRequirements:
    """Compose exact app-owned address/route entries for one physical NIC.

    Camera and Pixhawk entries deliberately coexist.  The camera /32 host
    routes are more specific than the Pixhawk connected /24 and retain their
    preferred source when every role shares one NIC.
    """

    normalized_roles = tuple(
        role for role in NETWORK_ROLE_ORDER if role in {_normalize_network_role(value) for value in roles}
    )
    addresses: list[str] = []
    routes: list[str] = []

    def append_unique(values: list[str], value: str) -> None:
        if value not in values:
            values.append(value)

    if NETWORK_ROLE_CAMERA in normalized_roles:
        camera_address = _canonical_role_address(camera_local_address, default_prefix=32)
        append_unique(addresses, str(camera_address))
        for raw_peer in camera_peers:
            peer = _canonical_role_peer(raw_peer)
            append_unique(routes, f"{peer}/32 src={camera_address.ip}")

    pixhawk_roles = {
        role for role in normalized_roles if role in {NETWORK_ROLE_P1, NETWORK_ROLE_P2}
    }
    if pixhawk_roles:
        pixhawk_address = _canonical_role_address(pixhawk_local_address, default_prefix=24)
        append_unique(addresses, str(pixhawk_address))
        if NETWORK_ROLE_P1 in pixhawk_roles:
            append_unique(
                routes,
                f"{_canonical_role_peer(p1_peer)}/32 src={pixhawk_address.ip}",
            )
        if NETWORK_ROLE_P2 in pixhawk_roles:
            append_unique(
                routes,
                f"{_canonical_role_peer(p2_peer)}/32 src={pixhawk_address.ip}",
            )

    return NetworkRoleRequirements(normalized_roles, tuple(addresses), tuple(routes))


def plan_network_role_move(
    current_assignments: Mapping[str, Mapping[str, object] | str],
    *,
    role: object,
    target_interface: object,
    target_stable_id: object = "",
) -> NetworkRoleMovePlan:
    """Return a role-scoped move plan without assuming NIC count or exclusivity."""

    moved_role = _normalize_network_role(role)
    target = str(target_interface or "").strip()
    if not target:
        raise ValueError("target interface is empty")

    before: list[NetworkRoleAssignment] = []
    by_role: dict[str, NetworkRoleAssignment] = {}
    for raw_role in NETWORK_ROLE_ORDER:
        raw = current_assignments.get(raw_role, {})
        if isinstance(raw, Mapping):
            interface = str(raw.get("interface") or "").strip()
            stable_id = str(raw.get("stable_id") or "").strip()
        else:
            interface = str(raw or "").strip()
            stable_id = ""
        assignment = NetworkRoleAssignment(raw_role, interface, stable_id)
        before.append(assignment)
        by_role[raw_role] = assignment

    source = by_role[moved_role].interface
    by_role[moved_role] = NetworkRoleAssignment(
        moved_role,
        target,
        str(target_stable_id or "").strip(),
    )
    after = tuple(by_role[role_name] for role_name in NETWORK_ROLE_ORDER)
    return NetworkRoleMovePlan(moved_role, source, target, tuple(before), after)


def rtsp_host_port(url: str) -> tuple[str, int]:
    if not url.startswith("rtsp://"):
        raise ValueError("RTSP URL must start with rtsp://")
    host_port = url.removeprefix("rtsp://").split("/", 1)[0]
    if not host_port:
        raise ValueError("RTSP URL is missing a host")
    host, separator, port_text = host_port.rpartition(":")
    if not separator:
        return host_port, 554
    if not host:
        raise ValueError("RTSP URL is missing a host")
    return host, int(port_text)


def check_tcp_port(host: str, port: int, *, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def local_ipv4_interfaces(*, include_wireless: bool = False) -> list[InterfaceCandidate]:
    result = subprocess.run(
        ["ip", "-o", "-4", "addr", "show", "scope", "global"],
        check=False,
        capture_output=True,
        text=True,
    )
    candidates: list[InterfaceCandidate] = []
    if result.returncode != 0:
        return candidates
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        name = parts[1]
        if _skip_interface(name, include_wireless=include_wireless):
            continue
        try:
            candidates.append(
                InterfaceCandidate(
                    name=name,
                    address=ip_interface(parts[3]),
                    mac=_interface_mac(name),
                )
            )
        except ValueError:
            continue
    return candidates


def neighbor_ipv4_addresses() -> list[IPv4Address]:
    result = subprocess.run(["ip", "-4", "neigh"], check=False, capture_output=True, text=True)
    addresses: list[IPv4Address] = []
    if result.returncode != 0:
        return addresses
    for line in result.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        try:
            address = IPv4Address(parts[0])
        except ValueError:
            continue
        if not address.is_loopback and not address.is_multicast:
            addresses.append(address)
    return _dedupe_addresses(addresses)


def arp_scan_ipv4_addresses(
    interfaces: list[InterfaceCandidate],
    *,
    timeout_s: float = 2.0,
    reports: list[ArpScanReport] | None = None,
) -> list[IPv4Address]:
    if shutil.which("arp-scan") is None:
        if reports is not None:
            reports.append(
                ArpScanReport(
                    interface=None,
                    network="",
                    available=False,
                    returncode=None,
                    error="arp-scan not found",
                )
            )
        return []
    addresses: list[IPv4Address] = []
    local_addresses = {candidate.address.ip for candidate in interfaces}
    for interface in interfaces:
        network = interface.address.network
        if not _scan_network(network, max_hosts_per_network=254):
            continue
        result = _run_arp_scan(interface, timeout_s=timeout_s)
        if result is None:
            if reports is not None:
                reports.append(
                    ArpScanReport(
                        interface=interface,
                        network=str(network),
                        available=True,
                        returncode=None,
                        error="arp-scan failed or timed out",
                    )
                )
            continue
        report_hosts: list[str] = []
        for line in result.stdout.splitlines():
            parts = line.split()
            if not parts:
                continue
            try:
                address = IPv4Address(parts[0])
            except ValueError:
                continue
            if address in local_addresses or address.is_loopback or address.is_multicast:
                continue
            if address in network:
                addresses.append(address)
                report_hosts.append(str(address))
        if reports is not None:
            reports.append(
                ArpScanReport(
                    interface=interface,
                    network=str(network),
                    available=True,
                    returncode=result.returncode,
                    hosts=tuple(report_hosts),
                    error=(result.stderr or "").strip(),
                )
            )
    return _dedupe_addresses(addresses)


def _run_arp_scan(interface: InterfaceCandidate, *, timeout_s: float):
    try:
        return subprocess.run(
            [
                "arp-scan",
                "--interface",
                interface.name,
                "--retry",
                "1",
                str(interface.address.network),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None


def discover_rtsp_candidates(
    *,
    timeout: float = 0.2,
    max_hosts_per_network: int = 254,
    fallback_scan_if_empty: bool = True,
    stop_after: int | None = 2,
    ports: tuple[int, ...] = DEFAULT_RTSP_PORTS,
    paths: tuple[str, ...] = DEFAULT_RTSP_PATHS,
    arp_scan_reports: list[ArpScanReport] | None = None,
    host_hints: tuple[str, ...] = (),
) -> list[RtspCandidate]:
    interfaces = local_ipv4_interfaces()

    # The deployed WIRIS pair has stable default addresses. Probe those first so
    # Auto Detect can recover a powered camera quickly instead of blocking on
    # stale ARP neighbors or full /24 sweeps across Pixhawk/camera networks.
    default_candidates = _probe_default_rtsp_urls(interfaces, timeout=timeout)
    if stop_after is not None and len(default_candidates) >= stop_after:
        return default_candidates

    hosts = _candidate_hosts(
        interfaces,
        max_hosts_per_network=max_hosts_per_network,
        arp_scan_reports=arp_scan_reports,
        host_hints=host_hints,
    )
    candidates = _merge_rtsp_candidates(
        default_candidates,
        _probe_rtsp_hosts(
            hosts,
            interfaces,
            timeout=timeout,
            ports=ports,
            paths=paths,
            stop_after=None if stop_after is None else max(0, stop_after - len(default_candidates)),
        ),
    )
    if (
        (stop_after is not None and len(candidates) >= stop_after)
        or not fallback_scan_if_empty
        or max_hosts_per_network > 0
    ):
        return candidates

    fallback_hosts = _candidate_hosts(
        interfaces,
        max_hosts_per_network=254,
        arp_scan_reports=arp_scan_reports,
        host_hints=host_hints,
    )
    return _merge_rtsp_candidates(
        candidates,
        _probe_rtsp_hosts(
            fallback_hosts,
            interfaces,
            timeout=timeout,
            ports=ports,
            paths=paths,
            stop_after=None if stop_after is None else max(0, stop_after - len(candidates)),
        ),
    )


def discover_tcp_port_candidates(
    port: int,
    *,
    timeout: float = 0.2,
    max_hosts_per_network: int = 254,
    fallback_scan_if_empty: bool = True,
    stop_after: int | None = None,
    arp_scan_reports: list[ArpScanReport] | None = None,
    host_hints: tuple[str, ...] = (),
) -> list[TcpPortCandidate]:
    interfaces = local_ipv4_interfaces()
    hosts = _candidate_hosts(
        interfaces,
        max_hosts_per_network=max_hosts_per_network,
        arp_scan_reports=arp_scan_reports,
        host_hints=host_hints,
    )
    candidates = _probe_tcp_port_hosts(hosts, interfaces, port=port, timeout=timeout, stop_after=stop_after)
    if candidates or not fallback_scan_if_empty or max_hosts_per_network > 0:
        return candidates

    fallback_hosts = _candidate_hosts(
        interfaces,
        max_hosts_per_network=254,
        arp_scan_reports=arp_scan_reports,
        host_hints=host_hints,
    )
    return _probe_tcp_port_hosts(fallback_hosts, interfaces, port=port, timeout=timeout, stop_after=stop_after)


def discover_tcp_port_range_candidates(
    ports: Iterable[int],
    *,
    timeout: float = 0.08,
    max_hosts_per_network: int = 0,
    fallback_scan_if_empty: bool = False,
    stop_after: int | None = None,
    arp_scan_reports: list[ArpScanReport] | None = None,
    host_hints: tuple[str, ...] = (),
) -> list[TcpPortCandidate]:
    interfaces = local_ipv4_interfaces()
    hosts = _candidate_hosts(
        interfaces,
        max_hosts_per_network=max_hosts_per_network,
        arp_scan_reports=arp_scan_reports,
        host_hints=host_hints,
    )
    candidates = _probe_tcp_port_range_hosts(hosts, interfaces, ports=ports, timeout=timeout, stop_after=stop_after)
    if candidates or not fallback_scan_if_empty or max_hosts_per_network > 0:
        return candidates

    fallback_hosts = _candidate_hosts(
        interfaces,
        max_hosts_per_network=254,
        arp_scan_reports=arp_scan_reports,
        host_hints=host_hints,
    )
    return _probe_tcp_port_range_hosts(
        fallback_hosts,
        interfaces,
        ports=ports,
        timeout=timeout,
        stop_after=stop_after,
    )


def tcp_port_scan_sequence(*preferred_ports: int, start: int = 1, end: int = 65535) -> tuple[int, ...]:
    if start < 1 or end > 65535 or start > end:
        raise ValueError("TCP port scan range must be within 1-65535")
    ports: list[int] = []
    seen: set[int] = set()

    def append_port(value: int) -> None:
        port = int(value)
        if port < 1 or port > 65535 or port in seen:
            return
        seen.add(port)
        ports.append(port)

    for preferred in preferred_ports:
        append_port(preferred)
    for port in range(start, end + 1):
        append_port(port)
    return tuple(ports)


def discover_host_interface_candidates(
    *,
    host_address: str = DEFAULT_HOST_ADDRESS,
    include_unconfigured: bool = False,
) -> list[HostInterfaceCandidate]:
    interfaces = local_ipv4_interfaces()
    neighbor_hosts = neighbor_ipv4_addresses()
    candidates: list[HostInterfaceCandidate] = []
    for interface in interfaces:
        hosts = tuple(str(host) for host in neighbor_hosts if host in interface.address.network)
        if hosts:
            candidates.append(HostInterfaceCandidate(interface=interface, neighbor_hosts=hosts))
    if candidates:
        return sorted(candidates, key=_host_interface_priority)
    candidates = [HostInterfaceCandidate(interface=interface) for interface in interfaces]
    if include_unconfigured:
        candidates.extend(_unconfigured_host_interface_candidates(interfaces, host_address))
    return sorted(candidates, key=_host_interface_priority)


def _probe_default_rtsp_urls(interfaces: list[InterfaceCandidate], *, timeout: float) -> list[RtspCandidate]:
    candidates: list[RtspCandidate] = []
    for url in (*DEFAULT_CAMERA_URLS, *LEGACY_CAMERA_URLS):
        try:
            host, port = rtsp_host_port(url)
            host_address = IPv4Address(host)
        except ValueError:
            continue
        interface = _interface_for_host(host_address, interfaces)
        if interface is None:
            continue
        if not check_tcp_port(host, port, timeout=timeout):
            continue
        candidates.append(RtspCandidate(url=url, host=host, port=port, interface=interface))
    return candidates


def _merge_rtsp_candidates(*groups: list[RtspCandidate]) -> list[RtspCandidate]:
    merged: list[RtspCandidate] = []
    seen: set[tuple[str, int]] = set()
    for group in groups:
        for candidate in group:
            key = (candidate.host, candidate.port)
            if key in seen:
                continue
            seen.add(key)
            merged.append(candidate)
    return merged


def _probe_rtsp_hosts(
    hosts: list[IPv4Address],
    interfaces: list[InterfaceCandidate],
    *,
    timeout: float,
    ports: tuple[int, ...],
    paths: tuple[str, ...],
    stop_after: int | None = None,
) -> list[RtspCandidate]:
    candidates: list[RtspCandidate] = []
    ordered_hosts = _dedupe_addresses(hosts)
    host_order = {host: index for index, host in enumerate(ordered_hosts)}
    for port in ports:
        if stop_after is not None and len(candidates) >= stop_after:
            break
        pending_hosts = [
            host
            for host in ordered_hosts
            if (str(host), port) not in {(candidate.host, candidate.port) for candidate in candidates}
            and str(host) not in {candidate.host for candidate in candidates}
        ]
        port_hits = _probe_rtsp_port_parallel(pending_hosts, port, timeout=timeout)
        for host in sorted(port_hits, key=lambda value: host_order.get(value, 999_999)):
            interface = _interface_for_host(host, interfaces)
            candidates.append(
                RtspCandidate(
                    url=f"rtsp://{host}:{port}/{paths[0]}",
                    host=str(host),
                    port=port,
                    interface=interface,
                )
            )
            if stop_after is not None and len(candidates) >= stop_after:
                break
    return candidates


def _probe_tcp_port_hosts(
    hosts: list[IPv4Address],
    interfaces: list[InterfaceCandidate],
    *,
    port: int,
    timeout: float,
    stop_after: int | None = None,
) -> list[TcpPortCandidate]:
    candidates: list[TcpPortCandidate] = []
    ordered_hosts = _dedupe_addresses(hosts)
    host_order = {host: index for index, host in enumerate(ordered_hosts)}
    port_hits = _probe_rtsp_port_parallel(ordered_hosts, int(port), timeout=timeout)
    for host in sorted(port_hits, key=lambda value: host_order.get(value, 999_999)):
        interface = _interface_for_host(host, interfaces)
        candidates.append(TcpPortCandidate(host=str(host), port=int(port), interface=interface))
        if stop_after is not None and len(candidates) >= stop_after:
            break
    return candidates


def _probe_tcp_port_range_hosts(
    hosts: list[IPv4Address],
    interfaces: list[InterfaceCandidate],
    *,
    ports: Iterable[int],
    timeout: float,
    stop_after: int | None = None,
) -> list[TcpPortCandidate]:
    candidates: list[TcpPortCandidate] = []
    ordered_hosts = _dedupe_addresses(hosts)
    ordered_ports = _dedupe_ports(ports)
    host_order = {host: index for index, host in enumerate(ordered_hosts)}
    port_order = {port: index for index, port in enumerate(ordered_ports)}
    port_hits = _probe_tcp_host_ports_parallel(
        ordered_hosts,
        ordered_ports,
        timeout=timeout,
        stop_after=stop_after,
    )
    for host, port in sorted(
        port_hits,
        key=lambda value: (host_order.get(value[0], 999_999), port_order.get(value[1], 999_999)),
    ):
        interface = _interface_for_host(host, interfaces)
        candidates.append(TcpPortCandidate(host=str(host), port=int(port), interface=interface))
        if stop_after is not None and len(candidates) >= stop_after:
            break
    return candidates


def _probe_rtsp_port_parallel(hosts: list[IPv4Address], port: int, *, timeout: float) -> list[IPv4Address]:
    if not hosts:
        return []
    max_workers = min(DEFAULT_RTSP_SCAN_WORKERS, max(1, len(hosts)))
    hits: list[IPv4Address] = []
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_host = {
            executor.submit(check_tcp_port, str(host), port, timeout=timeout): host
            for host in hosts
        }
        for future in as_completed(future_to_host):
            host = future_to_host[future]
            try:
                if future.result():
                    hits.append(host)
            except OSError:
                continue
    return hits


def _probe_tcp_host_ports_parallel(
    hosts: list[IPv4Address],
    ports: list[int],
    *,
    timeout: float,
    stop_after: int | None = None,
) -> list[tuple[IPv4Address, int]]:
    if not hosts or not ports:
        return []
    max_workers = min(DEFAULT_TCP_PORT_SCAN_WORKERS, max(1, len(hosts) * len(ports)))
    max_pending = max_workers * 4
    hits: list[tuple[IPv4Address, int]] = []

    def work_items():
        for port in ports:
            for host in hosts:
                yield host, port

    def collect_finished(pending: dict[Future[bool], tuple[IPv4Address, int]], done: set[Future[bool]]) -> bool:
        for future in done:
            host, port = pending.pop(future)
            try:
                if future.result():
                    hits.append((host, port))
                    if stop_after is not None and len(hits) >= stop_after:
                        return True
            except OSError:
                continue
        return False

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        pending: dict[Future[bool], tuple[IPv4Address, int]] = {}
        for host, port in work_items():
            future = executor.submit(check_tcp_port, str(host), port, timeout=timeout)
            pending[future] = (host, port)
            if len(pending) < max_pending:
                continue
            done, _not_done = wait(pending, return_when=FIRST_COMPLETED)
            if collect_finished(pending, done):
                for pending_future in pending:
                    pending_future.cancel()
                return hits
        while pending:
            done, _not_done = wait(pending, return_when=FIRST_COMPLETED)
            if collect_finished(pending, done):
                for pending_future in pending:
                    pending_future.cancel()
                return hits
    return hits


def select_host_for_camera(camera_host: str) -> InterfaceCandidate | None:
    try:
        address = IPv4Address(camera_host)
    except ValueError:
        return None
    return _interface_for_host(address, local_ipv4_interfaces())


def nm_device_connection_name(interface_name: str) -> str | None:
    interface_name = interface_name.strip()
    if not interface_name or shutil.which("nmcli") is None:
        return None
    result = subprocess.run(
        ["nmcli", "-g", "GENERAL.CONNECTION", "device", "show", interface_name],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return None
    output = (result.stdout or result.stderr).strip()
    if not output or output == "--":
        return None
    return output.splitlines()[0].strip() or None


def nm_connection_ipv4_addresses(connection_id: str) -> tuple[IPv4Interface, ...]:
    connection_id = connection_id.strip()
    if not connection_id or shutil.which("nmcli") is None:
        return ()
    result = subprocess.run(
        ["nmcli", "-g", "ipv4.addresses", "connection", "show", connection_id],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return ()
    output = (result.stdout or result.stderr).strip()
    if not output or output == "--":
        return ()
    addresses: list[IPv4Interface] = []
    for chunk in output.replace("\n", ",").split(","):
        text = chunk.strip()
        if not text or text == "--":
            continue
        try:
            addresses.append(ip_interface(text))
        except ValueError:
            continue
    return tuple(_dedupe_interfaces(addresses))


def resolve_nm_interface_address(interface_name: str, host_hints: Iterable[str]) -> IPv4Interface | None:
    connection_name = nm_device_connection_name(interface_name)
    if connection_name is None:
        return None
    candidates = nm_connection_ipv4_addresses(connection_name)
    if not candidates:
        return None
    for host in _host_hint_addresses(tuple(host_hints)):
        for candidate in candidates:
            if host in candidate.network:
                return candidate
    if len(candidates) == 1:
        return candidates[0]
    return None


def _nmcli_property(connection_name: str, property_name: str) -> str:
    nmcli = shutil.which("nmcli")
    if nmcli is None:
        raise RuntimeError("nmcli is unavailable")
    try:
        result = subprocess.run(
            [nmcli, "-g", property_name, "connection", "show", connection_name],
            check=False,
            capture_output=True,
            text=True,
            timeout=PIXHAWK_NETWORK_COMMAND_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError(f"timed out reading NetworkManager {property_name}") from exc
    if result.returncode != 0:
        error = (result.stderr or result.stdout or "").strip()
        raise RuntimeError(error or f"nmcli read failed for {connection_name} {property_name}")
    return (result.stdout or "").strip()


def _nmcli_list_property(connection_name: str, property_name: str) -> tuple[str, ...]:
    output = _nmcli_property(connection_name, property_name)
    if not output or output == "--":
        return ()
    values: list[str] = []
    for raw in output.replace("\n", ",").split(","):
        value = raw.strip()
        if value and value != "--" and value not in values:
            values.append(value)
    return tuple(values)


def nm_connection_ipv4_routes(connection_id: str) -> tuple[str, ...]:
    connection_id = str(connection_id).strip()
    if not connection_id:
        return ()
    try:
        return _nmcli_list_property(connection_id, "ipv4.routes")
    except (RuntimeError, TimeoutError):
        return ()


def _network_role_connection_name(interface_name: str, stable_id: str) -> str:
    token = re.sub(r"[^a-zA-Z0-9]+", "", stable_id) or re.sub(
        r"[^a-zA-Z0-9]+", "-", interface_name
    ).strip("-")
    return f"sasl-workstation-{token.lower()}"


def _nm_connection_exists(connection_name: str) -> bool:
    nmcli = shutil.which("nmcli")
    if nmcli is None:
        return False
    try:
        result = subprocess.run(
            [nmcli, "-g", "connection.id", "connection", "show", connection_name],
            check=False,
            capture_output=True,
            text=True,
            timeout=PIXHAWK_NETWORK_COMMAND_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0


def network_manager_connection_snapshot(
    interface_name: str,
    *,
    stable_id: str = "",
) -> NetworkManagerConnectionSnapshot:
    interface_name = str(interface_name).strip()
    if not interface_name:
        raise ValueError("interface is empty")
    active_name = nm_device_connection_name(interface_name)
    connection_name = active_name or _network_role_connection_name(interface_name, stable_id)
    exists = bool(active_name) or _nm_connection_exists(connection_name)
    if not exists:
        return NetworkManagerConnectionSnapshot(
            interface=interface_name,
            connection_name=connection_name,
            exists=False,
            active=False,
        )
    return NetworkManagerConnectionSnapshot(
        interface=interface_name,
        connection_name=connection_name,
        exists=True,
        active=active_name == connection_name,
        addresses=_nmcli_list_property(connection_name, "ipv4.addresses"),
        routes=_nmcli_list_property(connection_name, "ipv4.routes"),
        ipv4_method=_nmcli_property(connection_name, "ipv4.method") or "manual",
        never_default=_nmcli_property(connection_name, "ipv4.never-default") or "no",
    )


def _run_nmcli_mutation(args: Sequence[str]) -> None:
    nmcli = shutil.which("nmcli")
    if nmcli is None:
        raise RuntimeError("nmcli is unavailable")
    command = [nmcli, *[str(value) for value in args]]
    if os.geteuid() != 0:
        pkexec = shutil.which("pkexec")
        headless_qt = os.environ.get("QT_QPA_PLATFORM", "").lower() == "offscreen"
        has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if pkexec is None or headless_qt or not has_display:
            raise PermissionError(
                "NetworkManager role application requires graphical authorization; "
                f"run: sudo {' '.join(command)}"
            )
        command = [pkexec, *command]
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=NETWORK_ROLE_APPLY_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError(
            f"NetworkManager authorization/apply timed out after "
            f"{NETWORK_ROLE_APPLY_TIMEOUT_SECONDS:.0f}s"
        ) from exc
    if result.returncode != 0:
        error = (result.stderr or result.stdout or "").strip()
        if result.returncode in {126, 127}:
            raise PermissionError(error or "NetworkManager authorization was denied")
        raise RuntimeError(error or f"nmcli mutation failed with rc={result.returncode}")


def _apply_nm_connection_state(
    snapshot: NetworkManagerConnectionSnapshot,
    *,
    addresses: Sequence[str],
    routes: Sequence[str],
) -> None:
    address_text = ", ".join(addresses)
    route_text = ", ".join(routes)
    if snapshot.exists:
        modify_args: list[str] = [
            "connection",
            "modify",
            snapshot.connection_name,
        ]
        if addresses and snapshot.ipv4_method in {"disabled", "link-local"}:
            modify_args.extend(("ipv4.method", "manual"))
        elif not addresses and snapshot.ipv4_method == "manual":
            modify_args.extend(("ipv4.method", "disabled"))
        modify_args.extend(
            (
                "ipv4.addresses",
                address_text,
                "ipv4.routes",
                route_text,
            )
        )
        _run_nmcli_mutation(tuple(modify_args))
        if snapshot.active:
            _run_nmcli_mutation(("device", "reapply", snapshot.interface))
        else:
            _run_nmcli_mutation(
                ("connection", "up", snapshot.connection_name, "ifname", snapshot.interface)
            )
        return
    if not addresses:
        return
    args: list[str] = [
        "connection",
        "add",
        "type",
        "ethernet",
        "ifname",
        snapshot.interface,
        "con-name",
        snapshot.connection_name,
        "ipv4.method",
        "manual",
        "ipv4.addresses",
        address_text,
        "ipv4.never-default",
        "yes",
        "ipv6.method",
        "ignore",
        "connection.autoconnect",
        "yes",
    ]
    if routes:
        args.extend(("ipv4.routes", route_text))
    _run_nmcli_mutation(tuple(args))
    _run_nmcli_mutation(
        ("connection", "up", snapshot.connection_name, "ifname", snapshot.interface)
    )


def _restore_nm_connection_snapshot(snapshot: NetworkManagerConnectionSnapshot) -> None:
    if not snapshot.exists:
        if _nm_connection_exists(snapshot.connection_name):
            _run_nmcli_mutation(("connection", "delete", snapshot.connection_name))
        return
    _run_nmcli_mutation(
        (
            "connection",
            "modify",
            snapshot.connection_name,
            "ipv4.method",
            snapshot.ipv4_method,
            "ipv4.addresses",
            ", ".join(snapshot.addresses),
            "ipv4.routes",
            ", ".join(snapshot.routes),
            "ipv4.never-default",
            snapshot.never_default,
        )
    )
    if snapshot.active:
        _run_nmcli_mutation(("connection", "up", snapshot.connection_name, "ifname", snapshot.interface))
    elif nm_device_connection_name(snapshot.interface) == snapshot.connection_name:
        _run_nmcli_mutation(("connection", "down", snapshot.connection_name))


def _verify_network_role_profile(
    snapshot: NetworkManagerConnectionSnapshot,
    *,
    desired: NetworkRoleRequirements,
    delta: NetworkRoleEntryDelta,
) -> None:
    missing_addresses = set(desired.addresses) - set(snapshot.addresses)
    missing_routes = set(desired.routes) - set(snapshot.routes)
    retained_removed_addresses = set(delta.removed_addresses) & set(snapshot.addresses)
    retained_removed_routes = set(delta.removed_routes) & set(snapshot.routes)
    if missing_addresses or missing_routes or retained_removed_addresses or retained_removed_routes:
        raise RuntimeError(
            "NetworkManager readback mismatch: "
            f"missing_addresses={sorted(missing_addresses)} "
            f"missing_routes={sorted(missing_routes)} "
            f"retained_removed_addresses={sorted(retained_removed_addresses)} "
            f"retained_removed_routes={sorted(retained_removed_routes)}"
        )


def _verify_network_role_kernel(interface_name: str, desired: NetworkRoleRequirements) -> None:
    addresses = _global_ipv4_inventory()
    actual_ips = {str(address.ip) for address in addresses.get(interface_name, ())}
    for address_text in desired.addresses:
        address = _canonical_role_address(address_text, default_prefix=32)
        if str(address.ip) not in actual_ips:
            raise RuntimeError(
                f"kernel readback missing {address.ip} on {interface_name}"
            )
    for route_text in desired.routes:
        parts = route_text.split()
        peer = parts[0].split("/", 1)[0]
        source = next(
            (part.split("=", 1)[1] for part in parts[1:] if part.startswith("src=")),
            "",
        )
        evidence = pixhawk_route_evidence(peer, requested_source=source)
        state = evidence.status_for(interface_name, source)
        if state != "ROUTE OK":
            raise RuntimeError(evidence.summary_for(interface_name, source))


def apply_network_role_transaction(
    requests: Sequence[NetworkRoleInterfaceRequest],
) -> tuple[NetworkRoleInterfaceResult, ...]:
    """Apply target-first role deltas and restore exact snapshots on failure."""

    if not requests:
        return ()
    seen: set[str] = set()
    for request in requests:
        if not request.interface or request.interface in seen:
            raise ValueError(f"duplicate/empty role transaction interface: {request.interface}")
        seen.add(request.interface)

    snapshots = {
        request.interface: network_manager_connection_snapshot(
            request.interface, stable_id=request.stable_id
        )
        for request in requests
    }
    deltas: dict[str, NetworkRoleEntryDelta] = {}
    touched: list[NetworkManagerConnectionSnapshot] = []
    results: list[NetworkRoleInterfaceResult] = []
    try:
        for request in requests:
            before = snapshots[request.interface]
            adoptable_addresses = set(request.adoptable_before.addresses) | set(
                request.desired.addresses
            )
            adoptable_routes = set(request.adoptable_before.routes) | set(
                request.desired.routes
            )
            effective_owned = NetworkRoleOwnedEntries(
                _ordered_union(
                    request.owned_before.addresses,
                    (
                        value
                        for value in before.addresses
                        if value in adoptable_addresses
                    ),
                ),
                _ordered_union(
                    request.owned_before.routes,
                    (
                        value for value in before.routes if value in adoptable_routes
                    ),
                ),
            )
            delta = network_role_entry_delta(
                current_addresses=before.addresses,
                current_routes=before.routes,
                owned_before=effective_owned,
                desired=request.desired,
            )
            deltas[request.interface] = delta
            needs_apply = bool(
                delta.added_addresses
                or delta.added_routes
                or delta.removed_addresses
                or delta.removed_routes
                or (request.desired.addresses and not before.active)
            )
            if needs_apply:
                touched.append(before)
                _apply_nm_connection_state(
                    before,
                    addresses=delta.addresses_after,
                    routes=delta.routes_after,
                )
            after = network_manager_connection_snapshot(
                request.interface, stable_id=request.stable_id
            )
            _verify_network_role_profile(after, desired=request.desired, delta=delta)
            results.append(
                NetworkRoleInterfaceResult(
                    request.interface,
                    after.connection_name,
                    request.desired,
                    delta.owned_after,
                    after.addresses,
                    after.routes,
                )
            )
        for request in requests:
            _verify_network_role_kernel(request.interface, request.desired)
        return tuple(results)
    except Exception as exc:
        rollback_errors: list[str] = []
        for snapshot in reversed(touched):
            try:
                _restore_nm_connection_snapshot(snapshot)
            except Exception as rollback_exc:
                rollback_errors.append(f"{snapshot.interface}: {rollback_exc}")
        detail = f"; rollback incomplete: {' | '.join(rollback_errors)}" if rollback_errors else ""
        raise RuntimeError(f"network role transaction failed: {exc}{detail}") from exc


def interface_addresses(interface_name: str) -> str:
    try:
        result = subprocess.run(
            ["ip", "-br", "addr", "show", "dev", interface_name],
            check=False,
            capture_output=True,
            text=True,
            timeout=INTERFACE_STATUS_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError(
            f"Timed out reading {interface_name} after "
            f"{INTERFACE_STATUS_TIMEOUT_SECONDS:.1f}s"
        ) from exc
    output = (result.stdout or result.stderr).strip()
    if result.returncode != 0:
        return output or f"Unable to read {interface_name}"
    return output


def interface_link_exists(interface_name: str) -> bool:
    return interface_name.strip() in wired_interface_names()


def interface_has_address(interface_name: str, address_text: str) -> bool:
    try:
        address = ip_interface(address_text)
    except ValueError:
        return False
    return any(
        candidate.name == interface_name and candidate.address == address
        for candidate in local_ipv4_interfaces()
    )


def apply_interface_address(interface_name: str, address_text: str) -> tuple[bool, str]:
    try:
        address = ip_interface(address_text)
    except ValueError as exc:
        return False, f"invalid address {address_text}: {exc}"
    if not interface_name.strip():
        return False, "interface is empty"

    if interface_has_address(interface_name, str(address)):
        return True, f"{interface_name} already has {address}"

    command = ["ip", "addr", "add", str(address), "dev", interface_name]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode == 0:
        return True, f"applied {address} to {interface_name}"
    error = (result.stderr or result.stdout or "").strip()
    if "Operation not permitted" not in error and "RTNETLINK answers: File exists" not in error:
        return False, error or f"ip addr add failed with rc={result.returncode}"
    if "RTNETLINK answers: File exists" in error:
        return True, f"{interface_name} already has {address}"

    pkexec = shutil.which("pkexec")
    headless_qt = os.environ.get("QT_QPA_PLATFORM", "").lower() == "offscreen"
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if pkexec is None or headless_qt or not has_display:
        fallback_error = error or f"ip addr add failed with rc={result.returncode}"
        return False, f"{fallback_error}; run: sudo {' '.join(command)}"
    try:
        elevated = subprocess.run(
            [pkexec, *command],
            check=False,
            capture_output=True,
            text=True,
            timeout=PKEXEC_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired:
        return False, (
            f"pkexec authorization timed out after {PKEXEC_TIMEOUT_SECONDS:.0f}s; "
            f"run: sudo {' '.join(command)}"
        )
    if elevated.returncode == 0:
        return True, f"applied {address} to {interface_name}"
    elevated_error = (elevated.stderr or elevated.stdout or "").strip()
    return False, elevated_error or f"pkexec ip addr add failed with rc={elevated.returncode}"


def wired_interface_names() -> tuple[str, ...]:
    result = subprocess.run(
        ["ip", "-br", "link", "show"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return ()
    names: list[str] = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        name = parts[0].split("@", 1)[0]
        if _skip_interface(name) or name in names:
            continue
        names.append(name)
    return tuple(names)


def _run_ip_json(args: list[str]) -> list[dict[str, object]]:
    try:
        result = subprocess.run(
            ["ip", "-j", *args],
            check=False,
            capture_output=True,
            text=True,
            timeout=PIXHAWK_NETWORK_COMMAND_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError(
            f"Timed out reading kernel network state after "
            f"{PIXHAWK_NETWORK_COMMAND_TIMEOUT_SECONDS:.1f}s: ip -j {' '.join(args)}"
        ) from exc
    if result.returncode != 0:
        error = (result.stderr or result.stdout or "").strip()
        raise RuntimeError(error or f"ip -j {' '.join(args)} failed with rc={result.returncode}")
    try:
        payload = json.loads(result.stdout or "[]")
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"ip -j {' '.join(args)} returned invalid JSON: {exc}") from exc
    if not isinstance(payload, list):
        raise RuntimeError(f"ip -j {' '.join(args)} returned a non-list payload")
    return [row for row in payload if isinstance(row, dict)]


def _interface_mac(interface_name: str) -> str:
    mac_path = Path("/sys/class/net") / interface_name / "address"
    try:
        return mac_path.read_text(encoding="ascii").strip().lower()
    except (OSError, UnicodeError):
        return ""


def _interface_carrier(interface_name: str, link: dict[str, object]) -> str:
    carrier_path = Path("/sys/class/net") / interface_name / "carrier"
    try:
        raw = carrier_path.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError):
        raw = ""
    if raw == "1":
        return "UP"
    if raw == "0":
        return "DOWN"
    operstate = str(link.get("operstate") or "").upper()
    if operstate in {"UP", "DOWN"}:
        return operstate
    flags = {str(flag).upper() for flag in (link.get("flags") or ())}
    return "UP" if "LOWER_UP" in flags else "UNKNOWN"


def _interface_device_identity(interface_name: str) -> tuple[str, str]:
    device = Path("/sys/class/net") / interface_name / "device"
    try:
        device_path = str(device.resolve(strict=True))
    except OSError:
        device_path = ""
    driver = ""
    try:
        driver = (device / "driver").resolve(strict=True).name
    except OSError:
        pass
    return driver, device_path


def _wired_link_inventory() -> list[dict[str, object]]:
    links: list[dict[str, object]] = []
    for row in _run_ip_json(["link", "show"]):
        interface_name = str(row.get("ifname") or "").split("@", 1)[0]
        if not interface_name or _skip_interface(interface_name):
            continue
        if str(row.get("link_type") or "").lower() == "loopback":
            continue
        driver, device_path = _interface_device_identity(interface_name)
        links.append(
            {
                "interface": interface_name,
                "ifindex": row.get("ifindex") if isinstance(row.get("ifindex"), int) else None,
                "carrier": _interface_carrier(interface_name, row),
                "mac": str(row.get("address") or ""),
                "driver": driver,
                "device_path": device_path,
            }
        )
    return links


def network_interface_stable_id(interface_name: str) -> str:
    """Return MAC, then device path, then a name-scoped fallback for one NIC."""

    interface_name = str(interface_name).strip()
    try:
        inventory = _wired_link_inventory()
    except (RuntimeError, TimeoutError):
        inventory = []
    for row in inventory:
        if str(row.get("interface") or "") != interface_name:
            continue
        mac = str(row.get("mac") or "").strip().lower()
        if mac:
            return mac
        device_path = str(row.get("device_path") or "").strip()
        if device_path:
            return device_path
    return f"name:{interface_name}"


def candidate_stable_id(candidate: Mapping[str, object]) -> str:
    """Return the MAC-first identity carried by one discovered NIC candidate row.

    Kernel interface names are not stable across replug or reboot, so the row's
    own MAC is the identity contract.  The inventory lookup is only a fallback
    for rows that predate MAC capture.
    """

    mac = str(candidate.get("mac") or "").strip().lower()
    if mac:
        return mac
    device_path = str(candidate.get("device_path") or "").strip()
    if device_path:
        return device_path
    return network_interface_stable_id(str(candidate.get("interface") or ""))


def normalize_stable_id(value: object) -> str:
    """Fold one stored or discovered stable id for comparison only."""

    return str(value or "").strip().lower()


def candidate_matches_stable_id(candidate: Mapping[str, object], stable_id: object) -> bool:
    """Compare a discovered row against a stored identity without probing the host.

    Only fields the row already carries are considered, so this is safe to call
    from a UI refresh.  A row with no identity fields never matches, which lets
    callers fall back to kernel-name comparison.
    """

    wanted = normalize_stable_id(stable_id)
    if not wanted:
        return False
    for key in ("mac", "device_path"):
        value = normalize_stable_id(candidate.get(key))
        if value and value == wanted:
            return True
    interface_name = str(candidate.get("interface") or "").strip()
    return bool(interface_name) and wanted == normalize_stable_id(f"name:{interface_name}")


def candidate_has_stable_identity(candidate: Mapping[str, object]) -> bool:
    """Report whether a discovered row carries a name-independent identity."""

    return any(
        normalize_stable_id(candidate.get(key)) for key in ("mac", "device_path")
    )


def candidate_network_role_labels(
    candidate: Mapping[str, object],
    network_roles: Mapping[str, object],
) -> tuple[str, ...]:
    """Labels of the saved network roles that one discovered NIC row satisfies.

    A kernel name is not a NIC identity: the same USB adapter comes back as a
    different ifname across replug and reboot.  Match on the saved stable id
    whenever both sides carry one, and fall back to the name only for rows or
    roles recorded before that identity existed.  Labels keep NETWORK_ROLE_ORDER.
    """

    interface_name = str(candidate.get("interface") or "")
    row_identified = candidate_has_stable_identity(candidate)
    labels: list[str] = []
    for role in NETWORK_ROLE_ORDER:
        entry = network_roles.get(role)
        if not isinstance(entry, Mapping):
            continue
        stable_id = str(entry.get("stable_id") or "").strip()
        if stable_id and row_identified:
            if candidate_matches_stable_id(candidate, stable_id):
                labels.append(NETWORK_ROLE_LABELS[role])
            continue
        if interface_name and str(entry.get("interface") or "") == interface_name:
            labels.append(NETWORK_ROLE_LABELS[role])
    return tuple(labels)


def _global_ipv4_inventory() -> dict[str, list[IPv4Interface]]:
    addresses: dict[str, list[IPv4Interface]] = {}
    for row in _run_ip_json(["-4", "addr", "show", "scope", "global"]):
        interface_name = str(row.get("ifname") or "").split("@", 1)[0]
        if not interface_name:
            continue
        for info in row.get("addr_info") or ():
            if not isinstance(info, dict) or str(info.get("family") or "") != "inet":
                continue
            local = str(info.get("local") or "")
            prefixlen = info.get("prefixlen")
            try:
                address = ip_interface(f"{local}/{int(prefixlen)}")
            except (TypeError, ValueError):
                continue
            if isinstance(address, IPv4Interface):
                addresses.setdefault(interface_name, []).append(address)
    return {
        interface_name: _dedupe_interfaces(interface_addresses)
        for interface_name, interface_addresses in addresses.items()
    }


def pixhawk_route_evidence(peer_host: str, *, requested_source: str = "") -> PixhawkRouteEvidence:
    try:
        peer = str(IPv4Address(str(peer_host).strip()))
    except ValueError as exc:
        return PixhawkRouteEvidence(
            peer=str(peer_host),
            requested_source=str(requested_source),
            error=f"invalid peer IPv4: {exc}",
        )
    source = str(requested_source).strip().split("/", 1)[0]
    if source:
        try:
            source = str(IPv4Address(source))
        except ValueError as exc:
            return PixhawkRouteEvidence(
                peer=peer,
                requested_source=str(requested_source),
                error=f"invalid source IPv4: {exc}",
            )
    args = ["-4", "route", "get", peer]
    if source:
        args.extend(("from", source))
    try:
        rows = _run_ip_json(args)
    except (RuntimeError, TimeoutError) as exc:
        return PixhawkRouteEvidence(peer=peer, requested_source=source, error=str(exc))
    if not rows:
        return PixhawkRouteEvidence(peer=peer, requested_source=source, error="no route result")
    row = rows[0]
    return PixhawkRouteEvidence(
        peer=peer,
        requested_source=source,
        interface=str(row.get("dev") or ""),
        source=str(row.get("prefsrc") or row.get("src") or ""),
        gateway=str(row.get("gateway") or ""),
        error="" if row.get("dev") else "route result has no device",
    )


def discover_pixhawk_network_candidates(
    *,
    p1_peer: str,
    p2_peer: str,
    p1_endpoint: str,
    p2_endpoint: str,
) -> list[PixhawkNetworkCandidate]:
    """Return every physical wired NIC/address choice without changing host state."""

    links = _wired_link_inventory()
    addresses = _global_ipv4_inventory()
    route_cache: dict[tuple[str, str], PixhawkRouteEvidence] = {}

    def route(peer: str, source: str) -> PixhawkRouteEvidence:
        key = (str(peer), str(source))
        if key not in route_cache:
            route_cache[key] = pixhawk_route_evidence(peer, requested_source=source)
        return route_cache[key]

    candidates: list[PixhawkNetworkCandidate] = []
    for link in links:
        interface_name = str(link["interface"])
        interface_addresses = addresses.get(interface_name) or [None]
        for address in interface_addresses:
            local_cidr = "" if address is None else str(address)
            local_ip = "" if address is None else str(address.ip)
            ifindex = link.get("ifindex") if isinstance(link.get("ifindex"), int) else None
            candidate_id = f"{ifindex if ifindex is not None else '-'}:{interface_name}:{local_cidr or '-'}"
            candidates.append(
                PixhawkNetworkCandidate(
                    candidate_id=candidate_id,
                    interface=interface_name,
                    ifindex=ifindex,
                    carrier=str(link.get("carrier") or "UNKNOWN"),
                    local_cidr=local_cidr,
                    local_ip=local_ip,
                    mac=str(link.get("mac") or ""),
                    driver=str(link.get("driver") or ""),
                    device_path=str(link.get("device_path") or ""),
                    p1_route=route(p1_peer, local_ip),
                    p2_route=route(p2_peer, local_ip),
                    p1_endpoint=str(p1_endpoint),
                    p2_endpoint=str(p2_endpoint),
                )
            )
    return sorted(
        candidates,
        key=lambda item: (item.interface, item.local_cidr == "", item.local_cidr),
    )


def validate_pixhawk_network_assignment(
    *,
    interface_name: str,
    local_address: str,
    peer_host: str,
    endpoint: str,
) -> PixhawkAssignmentValidation:
    interface_name = str(interface_name).strip()
    if not interface_name:
        return PixhawkAssignmentValidation(True, "LEGACY AUTO ROUTE", "no explicit NIC assignment")
    try:
        links = {str(row["interface"]): row for row in _wired_link_inventory()}
    except (RuntimeError, TimeoutError) as exc:
        return PixhawkAssignmentValidation(False, "NETWORK READ ERROR", str(exc))
    link = links.get(interface_name)
    if link is None:
        return PixhawkAssignmentValidation(False, "INTERFACE MISSING", f"{interface_name} does not exist")
    if str(link.get("carrier") or "UNKNOWN") == "DOWN":
        return PixhawkAssignmentValidation(False, "LINK DOWN", f"{interface_name} has no carrier")
    local_ip = str(local_address).strip().split("/", 1)[0]
    if not local_ip:
        return PixhawkAssignmentValidation(
            False,
            "ADDRESS MISSING",
            f"{interface_name} assignment has no local IPv4 address",
        )
    try:
        local_ip = str(IPv4Address(local_ip))
    except ValueError as exc:
        return PixhawkAssignmentValidation(False, "ADDRESS INVALID", str(exc))
    try:
        addresses = _global_ipv4_inventory()
    except (RuntimeError, TimeoutError) as exc:
        return PixhawkAssignmentValidation(False, "NETWORK READ ERROR", str(exc))
    if local_ip not in {str(address.ip) for address in addresses.get(interface_name, ())}:
        return PixhawkAssignmentValidation(
            False,
            "ADDRESS MISSING",
            f"{local_ip} is not assigned to {interface_name}",
        )
    if not str(endpoint).strip():
        return PixhawkAssignmentValidation(False, "ENDPOINT INVALID", "MAVLink endpoint is empty")
    route = pixhawk_route_evidence(peer_host, requested_source=local_ip)
    state = route.status_for(interface_name, local_ip)
    if state != "ROUTE OK":
        return PixhawkAssignmentValidation(
            False,
            state,
            route.summary_for(interface_name, local_ip),
            route,
        )
    return PixhawkAssignmentValidation(
        True,
        "ROUTE OK",
        route.summary_for(interface_name, local_ip),
        route,
    )


def _skip_interface(name: str, *, include_wireless: bool = False) -> bool:
    prefixes = ("lo", "docker", "br-", "veth", "virbr", "tailscale", "zt", "wg")
    if not include_wireless:
        prefixes = (*prefixes, "wl", "wlan", "wlp", "wwan")
    return name.startswith(prefixes)


def _candidate_hosts(
    interfaces: list[InterfaceCandidate],
    *,
    max_hosts_per_network: int,
    arp_scan_reports: list[ArpScanReport] | None = None,
    host_hints: tuple[str, ...] = (),
) -> list[IPv4Address]:
    hosts: list[IPv4Address] = []
    neighbor_hosts = neighbor_ipv4_addresses()
    arp_scan_hosts = arp_scan_ipv4_addresses(interfaces, reports=arp_scan_reports)
    local_addresses = {candidate.address.ip for candidate in interfaces}
    for host in _host_hint_addresses(host_hints):
        if host not in local_addresses and not host.is_loopback and not host.is_multicast:
            hosts.append(host)
    for host in neighbor_hosts:
        if host not in local_addresses and _interface_for_host(host, interfaces) is not None:
            hosts.append(host)
    for host in arp_scan_hosts:
        if host not in local_addresses and _interface_for_host(host, interfaces) is not None:
            hosts.append(host)
    for candidate in interfaces:
        network = candidate.address.network
        if not _scan_network(network, max_hosts_per_network=max_hosts_per_network):
            continue
        for host in network.hosts():
            if host != candidate.address.ip:
                hosts.append(host)
    return _dedupe_addresses(hosts)


def _host_hint_addresses(host_hints: tuple[str, ...]) -> list[IPv4Address]:
    addresses: list[IPv4Address] = []
    for hint in host_hints:
        text = str(hint).strip()
        if not text:
            continue
        try:
            addresses.append(IPv4Address(text))
        except ValueError:
            continue
    return _dedupe_addresses(addresses)


def _unconfigured_host_interface_candidates(
    configured_interfaces: list[InterfaceCandidate],
    host_address: str,
) -> list[HostInterfaceCandidate]:
    try:
        address = ip_interface(host_address)
    except ValueError:
        return []
    configured_names = {candidate.name for candidate in configured_interfaces}
    candidates: list[HostInterfaceCandidate] = []
    for name in wired_interface_names():
        if name in configured_names:
            continue
        candidates.append(
            HostInterfaceCandidate(
                interface=InterfaceCandidate(name=name, address=address, mac=_interface_mac(name))
            )
        )
    return candidates


def _scan_network(network: IPv4Network, *, max_hosts_per_network: int) -> bool:
    if network.is_loopback or network.is_multicast or network.is_link_local:
        return False
    return network.num_addresses <= max_hosts_per_network + 2


def _interface_for_host(host: IPv4Address, interfaces: list[InterfaceCandidate]) -> InterfaceCandidate | None:
    for candidate in interfaces:
        if host in candidate.address.network:
            return candidate
    return None


def _host_interface_priority(candidate: HostInterfaceCandidate) -> tuple[int, str, str]:
    default_network = ip_interface(DEFAULT_HOST_ADDRESS).network
    default_priority = 0 if candidate.interface.address.network == default_network else 1
    return (default_priority, candidate.interface.name, str(candidate.interface.address))


def _dedupe_addresses(addresses: list[IPv4Address]) -> list[IPv4Address]:
    seen: set[IPv4Address] = set()
    deduped: list[IPv4Address] = []
    for address in addresses:
        if address in seen:
            continue
        seen.add(address)
        deduped.append(address)
    return deduped


def _dedupe_interfaces(addresses: list[IPv4Interface]) -> list[IPv4Interface]:
    seen: set[IPv4Interface] = set()
    deduped: list[IPv4Interface] = []
    for address in addresses:
        if address in seen:
            continue
        seen.add(address)
        deduped.append(address)
    return deduped


def _dedupe_ports(ports: Iterable[int]) -> list[int]:
    seen: set[int] = set()
    deduped: list[int] = []
    for value in ports:
        port = int(value)
        if port < 1 or port > 65535 or port in seen:
            continue
        seen.add(port)
        deduped.append(port)
    return deduped
