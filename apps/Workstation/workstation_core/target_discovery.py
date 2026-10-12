"""Target connection preferences and bounded receive-only discovery.

Only endpoint identities are handled here; WGS84/NED/body FRD measurements
retain their producer contracts in target_drone_stream.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import ipaddress
import os
from pathlib import Path
import threading
import time
from typing import Mapping

DEFAULT_PORTS = (14570, *range(14550, 14560))
MAX_CANDIDATES = 128
CANDIDATE_MAX_AGE_S = 30.0


def ports_from(value):
    parts = value.split(',') if isinstance(value, str) else value
    ports = tuple(dict.fromkeys(int(p) for p in parts))
    if not ports or len(ports) > 16 or any(not 1 <= p <= 65535 for p in ports):
        raise ValueError('Choose 1–16 UDP ports in 1..65535')
    return ports


def endpoint_parts(endpoint):
    value = str(endpoint).strip()
    if value.startswith(('udp:', 'udpin:', 'tcp:')):
        scheme, host, port = value.split(':')
        ipaddress.IPv4Address(host)  # No DNS/blocking hostname resolution in GUI.
        port = int(port)
        if not 1 <= port <= 65535:
            raise ValueError('Endpoint port must be in 1..65535')
        return ('udp' if scheme in ('udp', 'udpin') else 'tcp'), host, port
    if value.startswith('/dev/') and '..' not in Path(value).parts:
        return 'serial', value, None
    raise ValueError('Use udpin:IPv4:port, udp:IPv4:port, tcp:IPv4:port or /dev/...')


@dataclass(frozen=True)
class TargetReferenceSettings:
    mode: str = 'uam'
    transport: str = 'network'
    endpoint: str = ''
    baudrate: int = 57600
    expected_system_id: int | None = None
    expected_component_id: int | None = None
    ports: tuple[int, ...] = DEFAULT_PORTS

    @classmethod
    def from_mapping(cls, value=None):
        data = dict(value or {})
        mode = str(data.get('mode', 'uam')).lower()
        transport = str(data.get('transport', 'network')).lower()
        if mode not in ('drone', 'uam') or transport not in ('network', 'usb', 'manual'):
            raise ValueError('Invalid Target platform or transport')
        endpoint = str(data.get('endpoint', '')).strip()
        if endpoint:
            kind, _, _ = endpoint_parts(endpoint)
            if transport == 'usb' and kind != 'serial':
                raise ValueError('USB / Radio requires a /dev/... device')
            if transport == 'network' and kind != 'udp':
                raise ValueError('Network discovery uses a UDP listening endpoint')
        baud = int(data.get('baudrate', 57600))
        if not 1200 <= baud <= 4000000:
            raise ValueError('Baud rate must be in 1200..4000000')
        ids = []
        for name in ('expected_system_id', 'expected_component_id'):
            raw = data.get(name)
            if raw is None:
                ids.append(None)
            elif isinstance(raw, bool) or not 1 <= int(raw) <= 255:
                raise ValueError(f'{name} must be 1..255 or Any (null)')
            else:
                ids.append(int(raw))
        return cls(mode, transport, endpoint, baud, *ids, ports_from(data.get('ports', DEFAULT_PORTS)))

    def to_mapping(self):
        return {**self.__dict__, 'ports': list(self.ports)}


def serial_candidates():
    # Prefer stable device identity; do not import the rig's command/probe path.
    import glob
    found = {}
    for pattern in ('/dev/serial/by-id/*', '/dev/ttyACM*', '/dev/ttyUSB*'):
        for path in sorted(glob.glob(pattern)):
            found.setdefault(os.path.realpath(path), path)
    return tuple(found.values())


def discover_targets(kind, preferences, cancel, *, devices=None, duration_s=None, blocked=()):
    """Observe actual heartbeat packets. Never opens any write/send API."""
    from .target_transport import ReceiveOnlyConnection
    if kind not in ('network', 'usb'):
        raise ValueError('Unknown discovery kind')
    settings = TargetReferenceSettings.from_mapping(preferences)
    candidates, errors, connections = {}, [], []
    overflow = 0
    blocked = tuple(blocked)

    def busy(endpoint):
        kind, host, port = endpoint_parts(endpoint)
        for item in blocked:
            try:
                other, other_host, other_port = endpoint_parts(item['endpoint'])
            except (KeyError, TypeError, ValueError):
                continue
            if (kind == other == 'udp' and port == other_port) or (
                kind == other == 'serial' and os.path.realpath(host) == os.path.realpath(other_host)
            ):
                return str(item.get('owner', 'another connection'))
        return ''

    def observe(connection, endpoint, deadline):
        nonlocal overflow
        while not cancel.is_set() and time.monotonic() < deadline:
            msg = connection.recv_match(blocking=False)
            if msg is None:
                cancel.wait(0.01)
                return
            if msg.get_type() != 'HEARTBEAT':
                continue
            peer = getattr(msg, '_target_peer', None)
            sysid, compid = msg.get_srcSystem(), msg.get_srcComponent()
            key = (endpoint, peer, sysid, compid)
            now = time.monotonic()
            if key not in candidates:
                if len(candidates) >= MAX_CANDIDATES:
                    overflow += 1
                    continue
                candidates[key] = {
                    'candidate_id': hashlib.sha256(repr(key).encode()).hexdigest()[:24],
                    'transport': kind, 'endpoint': endpoint, 'baudrate': settings.baudrate,
                    'peer': list(peer) if peer else None,
                    'expected_system_id': sysid, 'expected_component_id': compid,
                    'heartbeat_count': 0, 'observed_monotonic_s': now,
                    'identity_state': 'OBSERVED' if sysid > 0 and compid > 0 else 'UNSUPPORTED ID 0',
                }
            candidates[key]['heartbeat_count'] += 1
            candidates[key]['observed_monotonic_s'] = now

    try:
        if kind == 'network':
            deadline = time.monotonic() + min(5.0, max(0.05, duration_s or 5.0))
            for port in settings.ports:
                if cancel.is_set():
                    break
                endpoint = f'udpin:0.0.0.0:{port}'
                try:
                    if owner := busy(endpoint):
                        raise RuntimeError(f'In use by {owner}')
                    connections.append((endpoint, ReceiveOnlyConnection(endpoint)))
                except (OSError, ValueError, RuntimeError) as exc:
                    errors.append(f'{endpoint}: {exc}')
            while connections and time.monotonic() < deadline and not cancel.is_set():
                for endpoint, connection in connections:
                    observe(connection, endpoint, min(deadline, time.monotonic() + 0.01))
        else:
            choices = tuple(serial_candidates() if devices is None else devices)
            unique = list(dict((os.path.realpath(d), d) for d in choices).values())
            if len(unique) > 8:
                errors.append('USB device limit: scanned first 8 devices')
            deadline = time.monotonic() + 10.0
            for endpoint in unique[:8]:
                if cancel.is_set() or time.monotonic() >= deadline:
                    break
                try:
                    if owner := busy(endpoint):
                        raise RuntimeError(f'In use by {owner}')
                    with ReceiveOnlyConnection(endpoint, baud=settings.baudrate) as connection:
                        until = min(deadline, time.monotonic() + min(1.0, duration_s or 1.0))
                        before = len(candidates)
                        while time.monotonic() < until and not cancel.is_set():
                            observe(connection, endpoint, until)
                        if len(candidates) == before and not cancel.is_set():
                            key = (endpoint, None, None, None)
                            candidates[key] = {
                                'candidate_id': hashlib.sha256(repr(key).encode()).hexdigest()[:24],
                                'transport':kind, 'endpoint':endpoint, 'baudrate':settings.baudrate,
                                'peer':None, 'expected_system_id':None, 'expected_component_id':None,
                                'heartbeat_count':0, 'observed_monotonic_s':time.monotonic(),
                                'identity_state':'NO HEARTBEAT',
                            }
                            errors.append(f'{endpoint}: device present; heartbeat not observed at {settings.baudrate} baud')
                except (OSError, ValueError, RuntimeError) as exc:
                    errors.append(f'{endpoint}: {exc}')
        state = 'CANCELLED' if cancel.is_set() else (
            'PARTIAL' if errors and candidates else 'FAILED' if errors else 'FOUND' if candidates else 'EMPTY'
        )
        return {'state': state, 'candidates': list(candidates.values()), 'errors': errors,
                'overflow': overflow, 'packet_overflow': sum(c.dropped for _, c in connections)}
    finally:
        for _, connection in connections:
            connection.close()
