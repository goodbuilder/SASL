"""Process-local Telemetry Radio stream demand and recovery policy."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


RADIO_INITIAL_RECOVERY_S = 30.0
RADIO_TIER_DWELL_S = 15.0
# After the link regressed while a tier was active, that tier is capped out
# and only retried after this much continuous healthy dwell one tier below.
# Without the cap the policy re-probed the failed tier every ~45 s and the
# 57600 radio spent most of its time in the STALE -> CRITICAL -> ... cycle.
RADIO_CAP_LIFT_S = 300.0

# P1/P2 Radio: measured minimal profile, reapplied on each connection/FC reboot.
# HEARTBEAT remains at the autopilot default; these are message demands only.
RADIO_MINIMAL_STREAM_REQUESTS = (
    ("ATTITUDE", 5.0),
    ("GPS_RAW_INT", 5.0),
    ("GPS2_RAW", 5.0),
    ("SYS_STATUS", 1.0),
    ("BATTERY_STATUS", 1.0),
)
# Explicitly stop Normal-mode defaults and optional demands left by older clients.
# Numeric IDs also cover newer PX4 messages absent from older pymavlink dialects.
RADIO_MINIMAL_DISABLED_MESSAGE_IDS = (
    4, 8, 26, 27, 29, 31, 32, 33, 36, 74, 83, 85, 116, 127,
    128, 129, 141, 230, 241, 242, 245, 250, 380, 436, 441, 12901, 12904,
)

RADIO_CRITICAL_STREAM_REQUESTS = (
    ("ATTITUDE", 5.0),
    ("SYS_STATUS", 1.0),
    ("GPS_RAW_INT", 5.0),
    ("GPS2_RAW", 5.0),
    ("GPS_RTK", 1.0),
    ("GPS2_RTK", 1.0),
    ("BATTERY_STATUS", 1.0),
)

RADIO_OPTIONAL_STOCK_STREAM_REQUESTS = (
    ("SCALED_IMU", 20.0),
    ("SCALED_IMU2", 20.0),
    ("SCALED_IMU3", 20.0),
    ("DEBUG_VECT", 20.0),
    ("RAW_IMU", 20.0),
    ("LOCAL_POSITION_NED", 10.0),
    ("GLOBAL_POSITION_INT", 10.0),
    ("VFR_HUD", 5.0),
)


class RadioStreamTier(IntEnum):
    CRITICAL = 0
    OPTIONAL_25 = 1
    OPTIONAL_50 = 2
    STOCK = 3


_TIER_SCALE = {
    RadioStreamTier.CRITICAL: 0.0,
    RadioStreamTier.OPTIONAL_25: 0.25,
    RadioStreamTier.OPTIONAL_50: 0.50,
    RadioStreamTier.STOCK: 1.0,
}


def radio_stream_requests(tier: RadioStreamTier | int) -> tuple[tuple[str, float], ...]:
    normalized = RadioStreamTier(int(tier))
    scale = _TIER_SCALE[normalized]
    optional = tuple(
        (name, rate_hz * scale)
        for name, rate_hz in RADIO_OPTIONAL_STOCK_STREAM_REQUESTS
        if scale > 0.0
    )
    return RADIO_CRITICAL_STREAM_REQUESTS + optional


@dataclass(frozen=True)
class RadioStreamPolicySnapshot:
    tier: RadioStreamTier
    healthy_since_s: float | None
    last_tier_change_s: float | None
    next_transition_in_s: float | None
    tier_cap: RadioStreamTier = RadioStreamTier.STOCK


class RadioStreamPolicy:
    """Restore optional demand only after deterministic healthy dwell periods.

    A regression observed while an optional tier was active caps future
    escalation one tier below the failed one; the cap lifts one step at a
    time after ``RADIO_CAP_LIFT_S`` of continuous healthy dwell at the cap.
    """

    def __init__(self) -> None:
        self.tier = RadioStreamTier.CRITICAL
        self.tier_cap = RadioStreamTier.STOCK
        self.healthy_since_s: float | None = None
        self.last_tier_change_s: float | None = None

    def reset(self) -> bool:
        """Full reset for a new transport/connection: the cap is forgotten too."""

        changed = self._regress()
        self.tier_cap = RadioStreamTier.STOCK
        return changed

    def _regress(self) -> bool:
        changed = self.tier is not RadioStreamTier.CRITICAL
        if changed:
            self.tier_cap = RadioStreamTier(int(self.tier) - 1)
        self.tier = RadioStreamTier.CRITICAL
        self.healthy_since_s = None
        self.last_tier_change_s = None
        return changed

    def _next_tier(self) -> RadioStreamTier | None:
        if self.tier is RadioStreamTier.STOCK:
            return None
        return RadioStreamTier(int(self.tier) + 1)

    def _dwell_required_s(self, next_tier: RadioStreamTier) -> float:
        base = RADIO_INITIAL_RECOVERY_S if self.tier is RadioStreamTier.CRITICAL else RADIO_TIER_DWELL_S
        if next_tier > self.tier_cap:
            return max(base, RADIO_CAP_LIFT_S)
        return base

    def _dwell_anchor_s(self) -> float | None:
        if self.tier is RadioStreamTier.CRITICAL:
            return self.healthy_since_s
        return self.last_tier_change_s

    def observe(self, *, qualified: bool, healthy: bool, now_s: float) -> bool:
        now_s = float(now_s)
        if not qualified or not healthy:
            return self._regress()
        if self.healthy_since_s is None:
            self.healthy_since_s = now_s
            return False
        next_tier = self._next_tier()
        if next_tier is None:
            return False
        anchor_s = self._dwell_anchor_s()
        assert anchor_s is not None
        if now_s - anchor_s < self._dwell_required_s(next_tier):
            return False
        if next_tier > self.tier_cap:
            self.tier_cap = next_tier
        self.tier = next_tier
        self.last_tier_change_s = now_s
        return True

    def snapshot(self, *, now_s: float) -> RadioStreamPolicySnapshot:
        now_s = float(now_s)
        next_tier = self._next_tier()
        anchor_s = self._dwell_anchor_s()
        if self.healthy_since_s is None or next_tier is None:
            remaining = None
        elif anchor_s is None:
            remaining = self._dwell_required_s(next_tier)
        else:
            remaining = max(0.0, self._dwell_required_s(next_tier) - (now_s - anchor_s))
        return RadioStreamPolicySnapshot(
            tier=self.tier,
            healthy_since_s=self.healthy_since_s,
            last_tier_change_s=self.last_tier_change_s,
            next_transition_in_s=remaining,
            tier_cap=self.tier_cap,
        )
