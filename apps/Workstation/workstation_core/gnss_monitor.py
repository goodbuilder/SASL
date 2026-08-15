"""Pure NTRIP transport and correction-health monitoring contracts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any


class NtripHealthState(str, Enum):
    DISABLED = "DISABLED"
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED_NO_DATA = "CONNECTED_NO_DATA"
    STREAMING = "STREAMING"
    RTCM_ACTIVE = "RTCM_ACTIVE"
    RTCM_STALE = "RTCM_STALE"
    STALE = "STALE"
    RECONNECTING = "RECONNECTING"
    ERROR = "ERROR"


@dataclass(frozen=True)
class NtripHealthSnapshot:
    state: NtripHealthState
    session_age_s: float | None
    transport_age_s: float | None
    rtcm_age_s: float | None
    bytes_received: int
    chunks_received: int
    reconnect_count: int
    last_error: str


def _age(timestamp_s: Any, now_s: float) -> float | None:
    if timestamp_s in (None, ""):
        return None
    try:
        value = float(timestamp_s)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return max(0.0, float(now_s) - value)


def _counter(value: Any) -> int:
    try:
        return max(0, int(value or 0))
    except (TypeError, ValueError):
        return 0


def evaluate_ntrip_health(
    *,
    network_mode: bool,
    connected: bool,
    running: bool,
    phase: str,
    connected_at_s: Any,
    last_chunk_s: Any,
    last_rtcm_s: Any,
    now_s: float,
    transport_stale_after_s: float,
    correction_stale_after_s: float,
    bytes_received: Any = 0,
    chunks_received: Any = 0,
    reconnect_count: Any = 0,
    last_error: str = "",
) -> NtripHealthSnapshot:
    """Evaluate NTRIP health without changing transport or operation state.

    ``connected`` means that the caster accepted the NTRIP request.  It does
    not imply that bytes or valid RTCM are still arriving; those are assessed
    independently from the last transport chunk and last parsed RTCM frame.
    """

    session_age_s = _age(connected_at_s, now_s)
    transport_age_s = _age(last_chunk_s, now_s)
    if transport_age_s is None and connected:
        transport_age_s = session_age_s
    rtcm_age_s = _age(last_rtcm_s, now_s)
    phase_value = str(phase or "DISCONNECTED").upper()
    transport_limit = max(0.0, float(transport_stale_after_s))
    correction_limit = max(0.0, float(correction_stale_after_s))

    if not network_mode:
        state = NtripHealthState.DISABLED
    elif phase_value == "ERROR" or (
        last_error
        and not running
        and not connected
        and phase_value not in {"STOPPED", "DISCONNECTED"}
    ):
        state = NtripHealthState.ERROR
    elif phase_value == "RECONNECTING":
        state = NtripHealthState.RECONNECTING
    elif not connected:
        state = NtripHealthState.CONNECTING if running else NtripHealthState.DISCONNECTED
    elif transport_age_s is None:
        state = NtripHealthState.CONNECTED_NO_DATA
    elif transport_age_s > transport_limit:
        state = NtripHealthState.STALE
    elif rtcm_age_s is not None and rtcm_age_s > correction_limit:
        state = NtripHealthState.RTCM_STALE
    elif rtcm_age_s is not None:
        state = NtripHealthState.RTCM_ACTIVE
    else:
        state = NtripHealthState.STREAMING

    return NtripHealthSnapshot(
        state=state,
        session_age_s=session_age_s,
        transport_age_s=transport_age_s,
        rtcm_age_s=rtcm_age_s,
        bytes_received=_counter(bytes_received),
        chunks_received=_counter(chunks_received),
        reconnect_count=_counter(reconnect_count),
        last_error=str(last_error or ""),
    )
