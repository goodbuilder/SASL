"""Pure runtime MAG selection verification rules for Workstation."""

from __future__ import annotations

from dataclasses import dataclass
import math


MAG_SELECTION_VERIFY_TIMEOUT_S = 8.0
MAG_SELECTION_ATTITUDE_STALE_S = 2.0


@dataclass(frozen=True)
class MagSelectionVerification:
    """Independent evidence that one Pixhawk side selected the requested MAG."""

    state: str
    expected_device_id: int | None
    actual_device_id: int | None
    active_sample_age_s: float | None
    attitude_age_s: float | None
    reason: str

    @property
    def verified(self) -> bool:
        return self.state == "ACTIVE VERIFIED"

    def to_mapping(self) -> dict[str, object]:
        return {
            "state": self.state,
            "expected_device_id": self.expected_device_id,
            "actual_device_id": self.actual_device_id,
            "active_sample_age_s": self.active_sample_age_s,
            "attitude_age_s": self.attitude_age_s,
            "reason": self.reason,
            "verified": self.verified,
        }


def _finite(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def evaluate_mag_selection_verification(
    *,
    expected_device_id: object,
    actual_device_id: object,
    active_sample_timestamp_s: object,
    attitude_timestamp_s: object,
    started_s: object,
    now_s: object,
    timeout_s: float = MAG_SELECTION_VERIFY_TIMEOUT_S,
    attitude_stale_s: float = MAG_SELECTION_ATTITUDE_STALE_S,
) -> MagSelectionVerification:
    """Evaluate one side without trusting PARAM readback as active selection."""

    expected = None
    actual = None
    try:
        expected = int(expected_device_id) if expected_device_id is not None else None
    except (TypeError, ValueError):
        pass
    try:
        actual = int(actual_device_id) if actual_device_id is not None else None
    except (TypeError, ValueError):
        pass

    now = _finite(now_s)
    started = _finite(started_s)
    active_sample = _finite(active_sample_timestamp_s)
    attitude_sample = _finite(attitude_timestamp_s)
    timeout = _finite(timeout_s)
    stale_limit = _finite(attitude_stale_s)
    if now is None:
        now = 0.0
    if timeout is None or timeout <= 0.0:
        timeout = MAG_SELECTION_VERIFY_TIMEOUT_S
    if stale_limit is None or stale_limit <= 0.0:
        stale_limit = MAG_SELECTION_ATTITUDE_STALE_S
    elapsed = None if started is None else max(0.0, now - started)
    active_age = None if active_sample is None else max(0.0, now - active_sample)
    attitude_age = None if attitude_sample is None else max(0.0, now - attitude_sample)

    if expected is None:
        return MagSelectionVerification(
            "ERROR", None, actual, active_age, attitude_age, "expected active MAG is unavailable"
        )
    if elapsed is not None and elapsed > timeout:
        if actual != expected:
            reason = f"active MAG mismatch or missing after {elapsed:.1f}s"
        elif attitude_sample is None or attitude_age is None or attitude_age > stale_limit:
            reason = f"fresh ATTITUDE missing after {elapsed:.1f}s"
        else:
            reason = f"MAG verification timed out after {elapsed:.1f}s"
        return MagSelectionVerification(
            "TIMEOUT", expected, actual, active_age, attitude_age, reason
        )
    if started is not None and (active_sample is None or active_sample < started):
        return MagSelectionVerification(
            "VERIFYING", expected, actual, active_age, attitude_age,
            "waiting for a post-apply vehicle_magnetometer sample",
        )
    if actual != expected:
        return MagSelectionVerification(
            "VERIFYING", expected, actual, active_age, attitude_age,
            f"expected device {expected}, actual device {actual if actual is not None else 'N/A'}",
        )
    if started is not None and (attitude_sample is None or attitude_sample < started):
        return MagSelectionVerification(
            "VERIFYING", expected, actual, active_age, attitude_age,
            "waiting for post-apply ATTITUDE",
        )
    if attitude_sample is None or attitude_age is None:
        return MagSelectionVerification(
            "VERIFYING", expected, actual, active_age, attitude_age,
            "fresh ATTITUDE unavailable",
        )
    if attitude_age > stale_limit:
        return MagSelectionVerification(
            "STALE", expected, actual, active_age, attitude_age,
            f"ATTITUDE stale ({attitude_age:.2f}s)",
        )
    return MagSelectionVerification(
        "ACTIVE VERIFIED", expected, actual, active_age, attitude_age,
        "active device ID matched and ATTITUDE is fresh",
    )

