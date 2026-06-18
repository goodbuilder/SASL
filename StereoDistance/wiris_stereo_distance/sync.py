from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Generic, Iterable, Protocol, TypeVar


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
) -> MatchedPair[T] | None:
    left_list = list(left_samples)
    right_list = list(right_samples)
    if not left_list or not right_list:
        return None

    candidates: list[MatchedPair[T]] = []
    for left in left_list:
        for right in right_list:
            delta = abs(float(left.timestamp_s) - float(right.timestamp_s))
            if delta > max_delta_s:
                continue
            midpoint = (float(left.timestamp_s) + float(right.timestamp_s)) / 2.0
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
    bias_captured: bool,
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

    if not bias_captured:
        warnings.append("bias not captured")

    hard_fail = camera_delta_s is None or (require_attitude and missing_attitude)
    stale_attitude = any("attitude age" in warning or "attitude Δt" in warning for warning in warnings)
    if hard_fail or (require_attitude and stale_attitude):
        return "BAD", tuple(warnings)
    if warnings:
        return "DEGRADED", tuple(warnings)
    return "GOOD", tuple()
