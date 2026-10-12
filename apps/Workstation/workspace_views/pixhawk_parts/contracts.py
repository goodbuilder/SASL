"""Display-only contracts; device resources remain owned by their runtime.

Readback objects are read-only producer values for one GUI refresh. Collections
of handles are reduced to side names, so views cannot manipulate worker state.
Position/orientation and sensor-axis contracts remain in the evidence rows.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass


@dataclass(frozen=True)
class TransportDisplay:
    transport: str
    device: str
    baud: int


@dataclass(frozen=True)
class ConnectionInputs:
    transports: Mapping[str, TransportDisplay]
    endpoints: Mapping[str, str]
    saved_serial_keys: tuple[tuple[str, str], ...]
    connection: Mapping[str, object]
    recovery: Mapping[str, Mapping[str, object]]
    open_sides: frozenset[str]
    configured_sides: frozenset[str]
    connection_in_progress: bool
    automatic_recording: bool


@dataclass(frozen=True)
class ConnectionCommands:
    """Optional callbacks preserve the legacy read-only backend behavior."""

    set_transport: Callable | None = None
    configure_usb: Callable | None = None
    configure_radio: Callable | None = None
    connect: Callable | None = None
    disconnect: Callable | None = None
    open_shell: Callable | None = None
    assign_network: Callable | None = None
    apply_candidate: Callable | None = None


@dataclass(frozen=True)
class MagnetometerInputs:
    status_cache: Mapping[str, object]
    incomplete_cache: Mapping[str, object]
    attitude_cache: Mapping[str, object]
    rotation_cache: Mapping[str, object]
    active_ids: Mapping[str, object]
    connected: frozenset[str]
    verified: frozenset[str]
    parameter_sides: frozenset[str]
    shell_sides: frozenset[str]
    armed_sides: frozenset[str]
    parameter_errors: Mapping[str, object]
    reset_errors: Mapping[str, object]
    now_s: float
