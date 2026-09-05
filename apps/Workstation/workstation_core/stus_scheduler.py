"""Pure Manual/Auto scheduling contract for Workstation STUS operations."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


STUS_MODE_MANUAL = "manual"
STUS_MODE_AUTO = "auto"
STUS_MODES = (STUS_MODE_MANUAL, STUS_MODE_AUTO)
STUS_DEFAULT_INTERVAL_S = 1800.0
STUS_MIN_INTERVAL_S = 300.0
STUS_MAX_INTERVAL_S = 86_400.0


def normalize_stus_mode(value: object) -> str:
    mode = str(value or STUS_MODE_MANUAL).strip().lower()
    return mode if mode in STUS_MODES else STUS_MODE_MANUAL


def normalize_stus_interval_s(value: object) -> float:
    try:
        interval_s = float(value)
    except (TypeError, ValueError):
        interval_s = STUS_DEFAULT_INTERVAL_S
    if not isfinite(interval_s):
        interval_s = STUS_DEFAULT_INTERVAL_S
    return max(STUS_MIN_INTERVAL_S, min(STUS_MAX_INTERVAL_S, interval_s))


@dataclass
class StusScheduler:
    """One-process scheduler; seed authority never survives a new connection."""

    mode: str = STUS_MODE_MANUAL
    interval_s: float = STUS_DEFAULT_INTERVAL_S
    connection_generation: int | None = None
    seeded_generation: int | None = None
    last_verified_monotonic_s: float | None = None
    next_due_monotonic_s: float | None = None
    due_dispatched: bool = False
    clock_revoked_reason: str = ""
    last_trigger: str = ""
    last_result: str = "NOT_RUN"

    def __post_init__(self) -> None:
        self.mode = normalize_stus_mode(self.mode)
        self.interval_s = normalize_stus_interval_s(self.interval_s)

    @property
    def seeded(self) -> bool:
        return (
            self.connection_generation is not None
            and self.seeded_generation == self.connection_generation
            and self.last_verified_monotonic_s is not None
        )

    def reset_for_connection(self, generation: int) -> None:
        self.connection_generation = int(generation)
        self.seeded_generation = None
        self.last_verified_monotonic_s = None
        self.next_due_monotonic_s = None
        self.due_dispatched = False
        self.clock_revoked_reason = ""
        self.last_trigger = ""
        self.last_result = "WAITING_MANUAL"

    def set_policy(self, *, mode: object, interval_s: object) -> None:
        self.mode = normalize_stus_mode(mode)
        self.interval_s = normalize_stus_interval_s(interval_s)
        self.due_dispatched = False
        if self.mode == STUS_MODE_AUTO and self.seeded:
            self.next_due_monotonic_s = (
                float(self.last_verified_monotonic_s) + self.interval_s
            )
        else:
            self.next_due_monotonic_s = None

    def mark_verified(
        self,
        *,
        now_monotonic_s: float,
        connection_generation: int,
        trigger: str,
    ) -> None:
        if self.connection_generation != int(connection_generation):
            self.reset_for_connection(int(connection_generation))
        self.seeded_generation = int(connection_generation)
        self.last_verified_monotonic_s = float(now_monotonic_s)
        self.clock_revoked_reason = ""
        self.due_dispatched = False
        self.last_trigger = str(trigger)
        self.last_result = "VERIFIED"
        self.next_due_monotonic_s = (
            float(now_monotonic_s) + self.interval_s
            if self.mode == STUS_MODE_AUTO
            else None
        )

    def mark_clock_revoked(self, reason: str) -> None:
        self.clock_revoked_reason = str(reason)
        self.last_result = "REVOKED"

    def poll(self, *, now_monotonic_s: float, operation_active: bool) -> str | None:
        if self.mode != STUS_MODE_AUTO or not self.seeded:
            return None
        due_s = self.next_due_monotonic_s
        if due_s is None or float(now_monotonic_s) < float(due_s):
            return None
        if operation_active or self.due_dispatched:
            return None
        self.due_dispatched = True
        self.last_trigger = STUS_MODE_AUTO
        self.last_result = "DUE"
        return STUS_MODE_AUTO

    def mark_finished(
        self,
        *,
        now_monotonic_s: float,
        trigger: str,
        verified: bool,
        reason: str = "",
    ) -> None:
        trigger_text = str(trigger)
        if verified:
            generation = self.connection_generation
            if generation is None:
                raise ValueError("STUS scheduler has no connection generation")
            self.mark_verified(
                now_monotonic_s=float(now_monotonic_s),
                connection_generation=generation,
                trigger=trigger_text,
            )
            return
        self.due_dispatched = False
        self.last_trigger = trigger_text
        self.last_result = "FAILED"
        self.clock_revoked_reason = str(reason)
        self.next_due_monotonic_s = (
            float(now_monotonic_s) + self.interval_s
            if self.mode == STUS_MODE_AUTO and self.seeded
            else None
        )

    def snapshot(self, *, now_monotonic_s: float) -> dict[str, object]:
        if not self.seeded:
            state = "WAITING_MANUAL"
        elif self.clock_revoked_reason:
            state = "REVOKED"
        elif (
            self.mode == STUS_MODE_AUTO
            and self.next_due_monotonic_s is not None
            and float(now_monotonic_s) >= float(self.next_due_monotonic_s)
        ):
            state = "OVERDUE" if not self.due_dispatched else "RUNNING"
        elif self.mode == STUS_MODE_AUTO:
            state = "ARMED"
        else:
            state = "MANUAL"
        remaining_s = (
            None
            if self.next_due_monotonic_s is None
            else max(0.0, float(self.next_due_monotonic_s) - float(now_monotonic_s))
        )
        return {
            "schema": "workstation-stus-schedule-v1",
            "mode": self.mode,
            "interval_s": self.interval_s,
            "state": state,
            "seeded": self.seeded,
            "connection_generation": self.connection_generation,
            "seeded_generation": self.seeded_generation,
            "last_verified_monotonic_s": self.last_verified_monotonic_s,
            "next_due_monotonic_s": self.next_due_monotonic_s,
            "remaining_s": remaining_s,
            "last_trigger": self.last_trigger,
            "last_result": self.last_result,
            "reason": self.clock_revoked_reason,
        }


__all__ = [
    "STUS_DEFAULT_INTERVAL_S",
    "STUS_MAX_INTERVAL_S",
    "STUS_MIN_INTERVAL_S",
    "STUS_MODE_AUTO",
    "STUS_MODE_MANUAL",
    "STUS_MODES",
    "StusScheduler",
    "normalize_stus_interval_s",
    "normalize_stus_mode",
]
