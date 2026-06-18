from __future__ import annotations

import os
import socket
import subprocess
import shutil
from collections.abc import Iterable
from concurrent.futures import FIRST_COMPLETED, Future, ThreadPoolExecutor, as_completed, wait
from dataclasses import dataclass
from ipaddress import IPv4Address, IPv4Interface, IPv4Network, ip_interface


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


@dataclass(frozen=True)
class InterfaceCandidate:
    name: str
    address: IPv4Interface


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


def local_ipv4_interfaces() -> list[InterfaceCandidate]:
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
        if _skip_interface(name):
            continue
        try:
            candidates.append(InterfaceCandidate(name=name, address=ip_interface(parts[3])))
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


def interface_addresses(interface_name: str) -> str:
    result = subprocess.run(
        ["ip", "-br", "addr", "show", "dev", interface_name],
        check=False,
        capture_output=True,
        text=True,
    )
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
    elevated = subprocess.run([pkexec, *command], check=False, capture_output=True, text=True)
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


def _skip_interface(name: str) -> bool:
    prefixes = ("lo", "docker", "br-", "veth", "virbr", "tailscale", "zt", "wg", "wl", "wlan", "wlp", "wwan")
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
        candidates.append(HostInterfaceCandidate(interface=InterfaceCandidate(name=name, address=address)))
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
