"""Existing WIRIS protocol constants, units and stateless parsers.

STUS uses UTC seconds/microseconds, RTT is seconds, settle delays are
milliseconds and GLRD distance is metres. No spatial coordinate conversion.
"""
from __future__ import annotations

import math
import re
import threading


RTSP_ENABLE_SETTLE_DELAY_MS = 4000


RTSP_ENABLE_PORT_WAIT_INTERVAL_MS = 1000


RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS = 12


ENCODER_PROFILE_COMMAND_SETTLE_MS = 250


ENCODER_PROFILE_READBACK_RETRY_INTERVAL_MS = 1000


ENCODER_PROFILE_READBACK_MAX_ATTEMPTS = 30


WIRIS_STUS_RTT_PROBE_COMMAND = "HIWS"


WIRIS_STUS_RTT_PROBE_COUNT = 5


WIRIS_STUS_MIN_VALID_RTT_PROBES = 2


WIRIS_STUS_HIGH_RTT_WARNING_S = 0.250


WIRIS_STUS_MAX_VALID_RTT_S = 3.900


WIRIS_STUS_PARSE_MARGIN_S = 0.002


WIRIS_STUS_MAX_ABSOLUTE_COMPENSATION_S = 2.000


WIRIS_STUS_MAX_ATTEMPTS = 2


WIRIS_STUS_MAX_OPERATION_S = 90.0


WIRIS_STUS_RESPONSE_TIMEOUT_S = 15.0


WIRIS_LRF_COMMAND = "GLRD"


_LRF_NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:[.,]\d*)?|[.,]\d+)(?:[eE][-+]?\d+)?")


def _wait_for_peer_or_continue(
    barrier: threading.Barrier | None,
    *,
    timeout_s: float,
) -> bool:
    """Synchronize healthy peers without making one failed side contagious."""

    if barrier is None:
        return True
    try:
        barrier.wait(timeout=max(0.1, float(timeout_s)))
        return True
    except threading.BrokenBarrierError:
        return False


def _abort_barrier(barrier: threading.Barrier | None) -> None:
    if barrier is None:
        return
    try:
        barrier.abort()
    except threading.BrokenBarrierError:
        pass


def parse_lrf_distance(response: object) -> float | None:
    """Parse the WIRIS ``GLRD`` response, whose distance is in metres.

    The camera may return a bare number, a number prefixed by ``OK``, or
    ``ERR`` when the optional LRF is unavailable.  Keep the parser tolerant of
    decimal commas and the spacing used in SDK examples, but never turn an
    error response into a numeric sample.
    """

    text = str(response or "").replace("\x00", "").strip()
    if not text or text.upper().startswith("ERR"):
        return None
    matches = _LRF_NUMBER_RE.findall(text)
    if not matches:
        return None
    token = matches[-1]
    if "," in token and "." not in token:
        token = token.replace(",", ".")
    try:
        value = float(token)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value >= 0.0 else None
