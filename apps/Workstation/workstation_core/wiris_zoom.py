from __future__ import annotations

import socket
import re
import threading
import time
from dataclasses import dataclass


DEFAULT_ZOOM_CONTROL_PORT = 2240
DEFAULT_ZOOM_IN_TEMPLATE = "SZIN"
DEFAULT_ZOOM_OUT_TEMPLATE = "SZOT"
DEFAULT_ZOOM_STOP_TEMPLATE = "SGFA"
DEFAULT_ZOOM_SET_TEMPLATE = "SZVN {level}"
DEFAULT_ZOOM_QUERY_TEMPLATE = "GZVV"
GET_ETHERNET_MODE_COMMAND = "GETH"
SET_ETHERNET_MODE_COMMAND = "SETH TRUE"
DISABLE_ETHERNET_MODE_COMMAND = "SETH FALSE"
SET_IP_ADDRESS_TEMPLATE = "STIP {address}"
SET_SUBNET_MASK_TEMPLATE = "STMS {mask}"
REBOOT_COMMAND = "REBT"
MIN_ZOOM_LEVEL = 0
MAX_ZOOM_LEVEL = 14
# Nominal 1280x720 WIRIS visible-camera focal lengths.  Calibrated intrinsics
# remain preferred when they match the active zoom; these values keep
# zoom-dependent reference calculations usable before a calibration bundle is
# loaded.
VISIBLE_ZOOM_FX_ESTIMATES = (
    (0, 1.0, 1030),
    (1, 1.2, 1236),
    (2, 1.5, 1545),
    (3, 2.0, 2060),
    (4, 3.0, 3091),
    (5, 4.0, 4121),
    (6, 5.0, 5151),
    (7, 6.0, 6181),
    (8, 8.0, 8242),
    (9, 10.0, 10302),
    (10, 12.0, 12362),
    (11, 16.0, 16483),
    (12, 20.0, 20604),
    (13, 25.0, 25755),
    (14, 30.0, 30906),
)
LINE_TERMINATOR = "\n"
NUL_TERMINATOR = "\0"

_ZOOM_NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)")


_CONTROL_LOCKS_GUARD = threading.Lock()
_CONTROL_LOCKS: dict[tuple[str, int], threading.Lock] = {}
MAX_CONTROL_RESPONSE_BYTES = 1024 * 1024
CONTROL_RESPONSE_IDLE_S = 0.1


class WirisControlBusyError(TimeoutError):
    """The endpoint is healthy but another Workstation command owns it."""


def _control_lock(host: str, port: int) -> threading.Lock:
    """Return the process-wide serializer for one camera control endpoint."""

    endpoint = (str(host).strip(), int(port))
    with _CONTROL_LOCKS_GUARD:
        lock = _CONTROL_LOCKS.get(endpoint)
        if lock is None:
            lock = threading.Lock()
            _CONTROL_LOCKS[endpoint] = lock
        return lock


@dataclass(frozen=True)
class WirisZoomCommand:
    host: str
    port: int
    command: str
    timeout_s: float = 1.0
    terminator: str = "\n"


def render_zoom_command(template: str, *, level: int | None = None) -> str:
    normalized = template.strip()
    if not normalized:
        raise ValueError("Zoom command template is empty")
    if "{level}" in normalized:
        if level is None:
            raise ValueError("Zoom level is required for this command template")
        return normalized.format(level=int(level))
    return normalized


def parse_visible_zoom_response(response: object) -> tuple[int | None, float | None]:
    """Parse a tolerant ``GZVV`` index/ratio response without treating it as a gate."""

    text = str(response or "").replace("\x00", " ").strip()
    if not text or text.upper().startswith("ERR"):
        return None, None
    normalized = re.sub(r"\bGZVV\b", " ", text, flags=re.IGNORECASE)
    numbers = [float(token) for token in _ZOOM_NUMBER_RE.findall(normalized)]
    if not numbers:
        return None, None
    index = next(
        (
            int(value)
            for value in numbers
            if value.is_integer() and MIN_ZOOM_LEVEL <= int(value) <= MAX_ZOOM_LEVEL
        ),
        None,
    )
    ratio = next((value for value in numbers[1:] if value > 0.0), None)
    if ratio is None and index is not None:
        ratio = float(index)
    return index, ratio


def nominal_focal_length_px(zoom_level: object) -> float | None:
    """Return the nominal visible-camera ``fx`` for a WIRIS zoom index."""

    try:
        index = int(zoom_level)
    except (TypeError, ValueError):
        return None
    for candidate, _ratio, focal_length_px in VISIBLE_ZOOM_FX_ESTIMATES:
        if candidate == index:
            return float(focal_length_px)
    return None


def build_zoom_command(
    *,
    host: str,
    port: int,
    template: str,
    level: int | None = None,
    timeout_s: float = 1.0,
    terminator: str = LINE_TERMINATOR,
) -> WirisZoomCommand:
    normalized_host = host.strip()
    if not normalized_host:
        raise ValueError("Zoom control host is empty")
    normalized_port = int(port)
    if normalized_port <= 0 or normalized_port > 65535:
        raise ValueError("Zoom control port must be between 1 and 65535")
    return WirisZoomCommand(
        host=normalized_host,
        port=normalized_port,
        command=render_zoom_command(template, level=level),
        timeout_s=float(timeout_s),
        terminator=terminator,
    )


def send_tcp_line(command: WirisZoomCommand) -> str:
    payload = (command.command.rstrip("\r\n\0") + command.terminator).encode("utf-8", errors="replace")
    # WIRIS exposes one SDK control service per camera.  Keep all Workstation
    # workers from opening overlapping command sessions against that service.
    timeout_s = max(0.01, float(command.timeout_s))
    lock = _control_lock(command.host, command.port)
    if not lock.acquire(timeout=timeout_s):
        raise WirisControlBusyError(
            f"WIRIS control busy: {command.host}:{command.port}"
        )
    # Start the socket-operation deadline only after the lock is held, so
    # time spent waiting for another worker's command session does not eat
    # into this command's own timeout budget.
    deadline_s = time.monotonic() + timeout_s
    try:
        remaining_s = max(0.01, deadline_s - time.monotonic())
        with socket.create_connection((command.host, command.port), timeout=remaining_s) as sock:
            sock.settimeout(remaining_s)
            sock.sendall(payload)
            chunks: list[bytes] = []
            size = 0
            while time.monotonic() < deadline_s:
                remaining_s = max(0.001, deadline_s - time.monotonic())
                sock.settimeout(
                    remaining_s
                    if not chunks
                    else min(CONTROL_RESPONSE_IDLE_S, remaining_s)
                )
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    break
                if not chunk:
                    break
                chunks.append(bytes(chunk))
                size += len(chunk)
                if size > MAX_CONTROL_RESPONSE_BYTES:
                    raise OSError("WIRIS control response exceeds 1 MiB")
                if any(terminator in chunk for terminator in (b"\n", b"\r", b"\0")):
                    break
    finally:
        lock.release()
    return b"".join(chunks).decode("utf-8", errors="replace").strip(" \t\r\n\0")


def build_wiris_command(
    *,
    host: str,
    port: int,
    command: str,
    timeout_s: float = 1.0,
    terminator: str = LINE_TERMINATOR,
) -> WirisZoomCommand:
    normalized_host = host.strip()
    if not normalized_host:
        raise ValueError("WIRIS control host is empty")
    normalized_command = command.strip()
    if not normalized_command:
        raise ValueError("WIRIS command is empty")
    normalized_port = int(port)
    if normalized_port <= 0 or normalized_port > 65535:
        raise ValueError("WIRIS control port must be between 1 and 65535")
    return WirisZoomCommand(
        host=normalized_host,
        port=normalized_port,
        command=normalized_command,
        timeout_s=float(timeout_s),
        terminator=terminator,
    )
