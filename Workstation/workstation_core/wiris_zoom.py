from __future__ import annotations

import socket
from dataclasses import dataclass


DEFAULT_ZOOM_CONTROL_PORT = 2240
DEFAULT_ZOOM_IN_TEMPLATE = "SZIN"
DEFAULT_ZOOM_OUT_TEMPLATE = "SZOT"
DEFAULT_ZOOM_STOP_TEMPLATE = "SGFA"
DEFAULT_ZOOM_SET_TEMPLATE = "SZVN {level}"
GET_ETHERNET_MODE_COMMAND = "GETH"
SET_ETHERNET_MODE_COMMAND = "SETH TRUE"
SET_IP_ADDRESS_TEMPLATE = "STIP {address}"
SET_SUBNET_MASK_TEMPLATE = "STMS {mask}"
REBOOT_COMMAND = "REBT"
MIN_ZOOM_LEVEL = 0
MAX_ZOOM_LEVEL = 14
LINE_TERMINATOR = "\n"
NUL_TERMINATOR = "\0"


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
    with socket.create_connection((command.host, command.port), timeout=command.timeout_s) as sock:
        sock.settimeout(command.timeout_s)
        sock.sendall(payload)
        try:
            response = sock.recv(4096)
        except socket.timeout:
            return ""
    return response.decode("utf-8", errors="replace").strip(" \t\r\n\0")


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
