from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import isfinite
from typing import Callable, Generic, Iterable, Protocol, TypeVar


ATTITUDE_STALE_AGE_S = 2.0
ATTITUDE_VERY_STALE_AGE_S = 10.0
ATTITUDE_STALE_PAIR_DELTA_S = 1.0
ATTITUDE_VERY_STALE_PAIR_DELTA_S = 5.0
ATTITUDE_TIMING_POLICY = "display_only_non_blocking"


class Timestamped(Protocol):
    timestamp_s: float


T = TypeVar("T", bound=Timestamped)


@dataclass(frozen=True)
class NearestSample(Generic[T]):
    sample: T
    age_s: float


@dataclass(frozen=True)
class MatchedPair(Generic[T]):
    left: T
    right: T
    delta_s: float
    midpoint_s: float


class TimedSampleBuffer(Generic[T]):
    def __init__(self, maxlen: int = 240) -> None:
        self.samples: deque[T] = deque(maxlen=maxlen)

    def add(self, sample: T | None) -> None:
        if sample is not None:
            self.samples.append(sample)

    def clear(self) -> None:
        self.samples.clear()

    def __iter__(self):
        return iter(self.samples)

    def __len__(self) -> int:
        return len(self.samples)

    def latest(self) -> T | None:
        return self.samples[-1] if self.samples else None

    def nearest(self, timestamp_s: float, max_age_s: float | None = None) -> NearestSample[T] | None:
        if not self.samples:
            return None
        best = min(self.samples, key=lambda sample: abs(float(sample.timestamp_s) - timestamp_s))
        age = abs(float(best.timestamp_s) - timestamp_s)
        if max_age_s is not None and age > max_age_s:
            return None
        return NearestSample(best, age)


def match_nearest_pair(
    left_samples: Iterable[T],
    right_samples: Iterable[T],
    *,
    max_delta_s: float,
    timestamp: Callable[[T], float | None] | None = None,
) -> MatchedPair[T] | None:
    left_list = list(left_samples)
    right_list = list(right_samples)
    if not left_list or not right_list:
        return None
    timestamp_fn = timestamp or (lambda sample: float(sample.timestamp_s))

    candidates: list[MatchedPair[T]] = []
    for left in left_list:
        for right in right_list:
            raw_left_timestamp_s = timestamp_fn(left)
            raw_right_timestamp_s = timestamp_fn(right)
            if raw_left_timestamp_s is None or raw_right_timestamp_s is None:
                continue
            left_timestamp_s = float(raw_left_timestamp_s)
            right_timestamp_s = float(raw_right_timestamp_s)
            delta = abs(left_timestamp_s - right_timestamp_s)
            if delta > max_delta_s:
                continue
            midpoint = (left_timestamp_s + right_timestamp_s) / 2.0
            candidates.append(MatchedPair(left=left, right=right, delta_s=delta, midpoint_s=midpoint))
    if not candidates:
        return None

    newest_midpoint = max(candidate.midpoint_s for candidate in candidates)
    recent_cutoff = newest_midpoint - max_delta_s
    recent_candidates = [candidate for candidate in candidates if candidate.midpoint_s >= recent_cutoff]
    return min(recent_candidates, key=lambda candidate: (candidate.delta_s, -candidate.midpoint_s))


def classify_sync_quality(
    *,
    camera_delta_s: float | None,
    max_camera_delta_s: float,
    left_attitude_age_s: float | None,
    right_attitude_age_s: float | None,
    max_attitude_age_s: float,
    attitude_pair_delta_s: float | None,
    max_attitude_pair_delta_s: float,
    require_attitude: bool,
) -> tuple[str, tuple[str, ...]]:
    warnings: list[str] = []
    if camera_delta_s is None:
        warnings.append("camera pair waiting")
    elif camera_delta_s > max_camera_delta_s:
        warnings.append(f"camera Δt {camera_delta_s * 1000:.0f}ms > {max_camera_delta_s * 1000:.0f}ms")

    missing_attitude = left_attitude_age_s is None or right_attitude_age_s is None
    if missing_attitude:
        warnings.append("attitude waiting")
    else:
        if left_attitude_age_s > max_attitude_age_s:
            warnings.append(f"left attitude age {left_attitude_age_s * 1000:.0f}ms")
        if right_attitude_age_s > max_attitude_age_s:
            warnings.append(f"right attitude age {right_attitude_age_s * 1000:.0f}ms")
        if attitude_pair_delta_s is not None and attitude_pair_delta_s > max_attitude_pair_delta_s:
            warnings.append(
                f"attitude Δt {attitude_pair_delta_s * 1000:.0f}ms > {max_attitude_pair_delta_s * 1000:.0f}ms"
            )

    hard_fail = camera_delta_s is None or (require_attitude and missing_attitude)
    stale_attitude = any("attitude age" in warning or "attitude Δt" in warning for warning in warnings)
    if hard_fail or (require_attitude and stale_attitude):
        return "BAD", tuple(warnings)
    if warnings:
        return "DEGRADED", tuple(warnings)
    return "GOOD", tuple()


def classify_attitude_timing(
    *,
    uses_pixhawk_attitude: bool,
    left_age_s: float | None,
    right_age_s: float | None,
    pair_delta_s: float | None,
) -> str:
    """Classify Pixhawk attitude timing for display and recording only.

    This status never accepts or rejects a measurement.  Camera-independent
    checkerboard geometry reports ``NOT_USED`` because Pixhawk attitude is not
    an input to that triangulation path.
    """

    if not uses_pixhawk_attitude:
        return "NOT_USED"
    values = (left_age_s, right_age_s, pair_delta_s)
    if any(value is None for value in values):
        return "WAITING"
    numeric_values = tuple(float(value) for value in values if value is not None)
    if any(not isfinite(value) or value < 0.0 for value in numeric_values):
        return "WAITING"
    left_age, right_age, pair_delta = numeric_values
    max_age = max(left_age, right_age)
    if max_age > ATTITUDE_VERY_STALE_AGE_S or pair_delta > ATTITUDE_VERY_STALE_PAIR_DELTA_S:
        return "VERY_STALE"
    if max_age > ATTITUDE_STALE_AGE_S or pair_delta > ATTITUDE_STALE_PAIR_DELTA_S:
        return "STALE"
    return "FRESH"
