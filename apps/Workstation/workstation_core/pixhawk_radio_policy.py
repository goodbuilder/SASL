"""Process-local Telemetry Radio stream demand and recovery policy."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


RADIO_INITIAL_RECOVERY_S = 30.0
RADIO_TIER_DWELL_S = 15.0

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


class RadioStreamPolicy:
    """Restore optional demand only after deterministic healthy dwell periods."""

    def __init__(self) -> None:
        self.tier = RadioStreamTier.CRITICAL
        self.healthy_since_s: float | None = None
        self.last_tier_change_s: float | None = None

    def reset(self) -> bool:
        changed = self.tier is not RadioStreamTier.CRITICAL
        self.tier = RadioStreamTier.CRITICAL
        self.healthy_since_s = None
        self.last_tier_change_s = None
        return changed

    def observe(self, *, qualified: bool, healthy: bool, now_s: float) -> bool:
        now_s = float(now_s)
        if not qualified or not healthy:
            return self.reset()
        if self.healthy_since_s is None:
            self.healthy_since_s = now_s
            return False
        if self.tier is RadioStreamTier.CRITICAL:
            if now_s - self.healthy_since_s < RADIO_INITIAL_RECOVERY_S:
                return False
            self.tier = RadioStreamTier.OPTIONAL_25
            self.last_tier_change_s = now_s
            return True
        if self.tier is RadioStreamTier.STOCK:
            return False
        assert self.last_tier_change_s is not None
        if now_s - self.last_tier_change_s < RADIO_TIER_DWELL_S:
            return False
        self.tier = RadioStreamTier(int(self.tier) + 1)
        self.last_tier_change_s = now_s
        return True

    def snapshot(self, *, now_s: float) -> RadioStreamPolicySnapshot:
        now_s = float(now_s)
        if self.healthy_since_s is None or self.tier is RadioStreamTier.STOCK:
            remaining = None
        elif self.tier is RadioStreamTier.CRITICAL:
            remaining = max(0.0, RADIO_INITIAL_RECOVERY_S - (now_s - self.healthy_since_s))
        elif self.last_tier_change_s is None:
            remaining = RADIO_TIER_DWELL_S
        else:
            remaining = max(0.0, RADIO_TIER_DWELL_S - (now_s - self.last_tier_change_s))
        return RadioStreamPolicySnapshot(
            tier=self.tier,
            healthy_since_s=self.healthy_since_s,
            last_tier_change_s=self.last_tier_change_s,
            next_transition_in_s=remaining,
        )
