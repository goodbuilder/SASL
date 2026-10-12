"""Endpoint validation and temporary USB identity probes.

Probe connections and leases are paired in one try/finally scope. Persistent
connection lifetime belongs to the transport owner, not these operations.
Device paths and bounded /run or /tmp locks remain machine-local contracts.
"""

from __future__ import annotations

import errno
import glob
from ipaddress import IPv4Address
import os
import socket
import time
from typing import Any, Callable
from .types import UsbMavlinkIdentity
from .decode import mavlink_message_provenance
from .policy import (
    DEFAULT_USB_MAVLINK_BAUDRATES,
    MAVLINK_GCS_SOURCE_SYSTEM,
    MAVLINK_GCS_SOURCE_COMPONENT,
)


def _mavlink_udpin_endpoint(url: str) -> tuple[str, int] | None:
    """Return the IPv4 bind endpoint for a pymavlink ``udpin`` URL."""

    text = str(url).strip()
    if not text.lower().startswith("udpin:"):
        return None
    address = text[6:]
    try:
        host, port_text = address.rsplit(":", 1)
        port = int(port_text)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid MAVLink udpin URL: {url!r}") from exc
    if not host or port <= 0 or port > 65535:
        raise ValueError(f"Invalid MAVLink udpin URL: {url!r}")
    return host, port


def _assert_mavlink_udpin_available(
    url: str,
    *,
    socket_factory: Callable[..., Any] = socket.socket,
) -> None:
    """Fail before pymavlink silently shares an existing UDP input port.

    pymavlink enables ``SO_REUSEADDR`` before binding every ``udpin`` socket.
    That is convenient for generic telemetry tools, but unsafe for this
    owner: a leaked worker or another application can bind the same endpoint
    and make packet delivery non-deterministic while both UIs claim success.
    """

    endpoint = _mavlink_udpin_endpoint(url)
    if endpoint is None:
        return
    probe = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(endpoint)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            host, port = endpoint
            raise RuntimeError(
                f"MAVLink UDP input {host}:{port} is already in use; "
                "close the other MAVLink session or leaked worker before reconnecting"
            ) from exc
        raise
    finally:
        probe.close()


def _disable_mavlink_udpin_reuse(connection: Any, url: str) -> None:
    """Make the live pymavlink socket exclusive after its successful bind."""

    if _mavlink_udpin_endpoint(url) is None:
        return
    port_socket = getattr(connection, "port", None)
    setsockopt = getattr(port_socket, "setsockopt", None)
    if callable(setsockopt):
        setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)


def _seed_mavlink_udpin_peer(
    connection: Any,
    url: str,
    peer_host: str,
    *,
    observed_wall_s: float | None = None,
) -> tuple[str, int] | None:
    """Seed pymavlink's input-server destination before the first RX packet.

    ``mavudp(input=True)`` sends only to addresses learned by ``recvfrom``.
    The Workstation already owns a fixed PX4 Ethernet peer per side, so seed
    that peer on the same bound ``udpin`` socket and let the normal pymavlink
    encoder/write path send the first GCS heartbeat. The peer UDP port is the
    port in the side's approved ``udpin`` endpoint.
    """

    endpoint = _mavlink_udpin_endpoint(url)
    if endpoint is None:
        return None
    host_text = str(peer_host).strip()
    if not host_text:
        return None
    try:
        normalized_host = str(IPv4Address(host_text))
    except ValueError as exc:
        raise ValueError(f"Invalid MAVLink Ethernet peer IPv4 address: {peer_host!r}") from exc
    if not bool(getattr(connection, "udp_server", False)):
        raise RuntimeError("MAVLink udpin connection is not an input UDP server")
    clients = getattr(connection, "clients", None)
    if not isinstance(clients, set):
        raise RuntimeError("MAVLink udpin connection does not expose its peer set")
    peer = (normalized_host, int(endpoint[1]))
    clients.add(peer)
    last_alive = getattr(connection, "clients_last_alive", None)
    if isinstance(last_alive, dict):
        last_alive[peer] = time.time() if observed_wall_s is None else float(observed_wall_s)
    return peer


def camera_label_ko(camera: str) -> str:
    return {"left": "왼쪽", "right": "오른쪽", "drone": "Drone"}.get(camera, camera)


def _normalize_transport(value: Any) -> str:
    transport = str(value).strip().lower()
    return transport if transport in {"radio", "usb"} else "ethernet"


def _is_serial_transport(transport: Any) -> bool:
    """Radio and USB open one exclusive serial device; Ethernet does not.

    Only the bandwidth policy separates the two: Radio throttles optional
    streams, USB carries the stock Ethernet-class stream set.
    """

    return str(transport).strip().lower() in {"radio", "usb"}


def usb_mavlink_device_candidates(
    patterns: tuple[str, ...] = ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"),
) -> list[str]:
    candidates: list[str] = []
    targets_seen: set[str] = set()
    for pattern in patterns:
        for candidate in sorted(glob.glob(pattern)):
            target = os.path.realpath(candidate)
            if target in targets_seen:
                continue
            targets_seen.add(target)
            candidates.append(candidate)
    return candidates


def probe_usb_mavlink_devices(
    candidates: list[str] | None = None,
    *,
    baudrates: tuple[int, ...] = DEFAULT_USB_MAVLINK_BAUDRATES,
    timeout_s: float = 1.0,
) -> list[UsbMavlinkIdentity]:
    try:
        from pymavlink import mavutil
    except Exception as exc:
        raise RuntimeError(f"pymavlink를 사용할 수 없어 USB Pixhawk 자동 탐지를 할 수 없습니다. ({exc})") from exc
    from Workstation.workstation_core.serial_endpoint_lease import acquire_serial_endpoint

    found: list[UsbMavlinkIdentity] = []
    for device in candidates if candidates is not None else usb_mavlink_device_candidates():
        for baud in baudrates:
            connection = None
            lease = None
            try:
                lease = acquire_serial_endpoint(device)
                connection = mavutil.mavlink_connection(
                    device,
                    baud=int(baud),
                    autoreconnect=False,
                    robust_parsing=True,
                    source_system=MAVLINK_GCS_SOURCE_SYSTEM,
                    source_component=MAVLINK_GCS_SOURCE_COMPONENT,
                )
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                heartbeat = connection.wait_heartbeat(timeout=timeout_s)
                if heartbeat is not None:
                    provenance = mavlink_message_provenance(heartbeat)
                    system_id = provenance["mavlink_source_system"]
                    component_id = provenance["mavlink_source_component"]
                    if system_id is None or component_id is None:
                        continue
                    found.append(
                        UsbMavlinkIdentity(
                            device=str(device),
                            resolved_device=os.path.realpath(device),
                            baud=int(baud),
                            system_id=int(system_id),
                            component_id=int(component_id),
                            observed_monotonic_s=time.monotonic(),
                        )
                    )
                    break
            except Exception:
                continue
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
                if lease is not None:
                    lease.close()
    return found
