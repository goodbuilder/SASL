"""Bounded MAVLink receive transport with exclusive OS ownership and no send API.

A validated heartbeat is reported identity, not cryptographic authentication.
No coordinate conversion is performed. Serial paths are machine-local devices.
"""
from __future__ import annotations
from collections import deque
import fcntl
import select
import socket
import time

from .target_discovery import endpoint_parts


class ReceiveOnlyConnection:
    def __init__(self, endpoint, *, baud=57600, expected_peer=None,
                 expected_system_id=None, expected_component_id=None, cancel=None):
        self.kind, host, port = endpoint_parts(endpoint)
        self.peer = tuple(expected_peer) if expected_peer is not None else None
        if self.peer is not None and self.kind != 'udp':
            raise ValueError('Peer pinning requires a UDP source')
        self.expected_system_id = expected_system_id
        self.expected_component_id = expected_component_id
        self.cancel = cancel
        self._socket = self._serial = self._lock = None
        self._messages = deque()
        self._parsers = {}
        self.dropped = 0
        self.rejected_peer_count = 0
        self.rejected_identity_count = 0
        try:
            if self.kind == 'udp':
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket.bind((host, port))  # Deliberately no REUSEADDR/REUSEPORT.
                self._socket.setblocking(False)
            elif self.kind == 'tcp':
                self._socket = socket.create_connection((host, port), timeout=0.5)
                self._socket.setblocking(False)
            else:
                import serial
                from .serial_endpoint_lease import acquire_serial_endpoint
                self._lock = acquire_serial_endpoint(host)
                self._serial = serial.Serial(port=None, baudrate=baud, timeout=0, exclusive=True)
                self._serial.dtr = False
                self._serial.rts = False
                self._serial.port = host
                self._serial.open()
                # Kernel tty exclusion also covers non-cooperating openers.
                import termios
                fcntl.ioctl(self._serial.fileno(), termios.TIOCEXCL)
        except BaseException:
            self.close()
            raise

    def _parse(self, payload, peer):
        from pymavlink.dialects.v20 import common
        if self.peer is not None and peer != self.peer:
            self.rejected_peer_count += 1
            return
        if peer not in self._parsers:
            if len(self._parsers) >= 64:
                self.dropped += 1
                return
            parser = common.MAVLink(None)
            parser.robust_parsing = True
            self._parsers[peer] = parser
        parser = self._parsers[peer]
        # Parse byte-wise: the output queue stays bounded even for large datagrams.
        for byte in payload:
            try:
                msg = parser.parse_char(bytes((byte,)))
            except (ValueError, common.MAVError):
                continue
            if msg is None or msg.get_type() == 'BAD_DATA':
                continue
            msg._target_peer = peer
            if len(self._messages) < 256:
                self._messages.append(msg)
            else:
                self.dropped += 1

    def recv_match(self, *, blocking=False):
        if self._messages:
            return self._messages.popleft()
        if self._socket is None and self._serial is None:
            raise OSError('Target transport is closed')
        if self._socket is not None:
            if not select.select([self._socket], [], [], 0.05 if blocking else 0)[0]:
                return None
            if self.kind == 'udp':
                payload, peer = self._socket.recvfrom(8192)
            else:
                payload, peer = self._socket.recv(8192), None
                if not payload:
                    raise OSError('Target TCP peer disconnected')
        else:
            payload, peer = self._serial.read(4096), None
        if payload:
            self._parse(payload, peer)
        return self._messages.popleft() if self._messages else None

    def wait_heartbeat(self, *, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.cancel is not None and self.cancel.is_set():
                return None
            msg = self.recv_match(blocking=False)
            if msg is None:
                if self.cancel is not None:
                    self.cancel.wait(0.01)
                else:
                    time.sleep(0.01)
                continue
            if msg.get_type() != 'HEARTBEAT':
                continue
            if ((self.expected_system_id is not None and msg.get_srcSystem() != self.expected_system_id)
                    or (self.expected_component_id is not None and msg.get_srcComponent() != self.expected_component_id)):
                self.rejected_identity_count += 1
                continue
            if self.kind == 'udp':
                self.peer = msg._target_peer
                self._messages = deque(m for m in self._messages if m._target_peer == self.peer)
            return msg
        return None

    def close(self):
        for name in ('_socket', '_serial', '_lock'):
            handle = getattr(self, name, None)
            if handle is not None:
                try:
                    if name == '_serial' and handle.is_open:
                        import termios
                        try:
                            fcntl.ioctl(handle.fileno(), termios.TIOCNXCL)
                        except OSError:
                            pass
                finally:
                    try:
                        handle.close()
                    finally:
                        setattr(self, name, None)
        self._messages.clear()
        self._parsers.clear()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()
