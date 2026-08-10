"""Pure trend-buffer helpers used by the PoseSimulation rqt plots."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence


@dataclass(frozen=True)
class TrendSample:
    timestamp_s: float
    values: Mapping[str, float | None]
    valid: bool
    token: object


def normalize_sample_values(values: Mapping[str, object], *, valid: bool) -> dict[str, float | None]:
    """Keep only finite values; invalid samples become explicit plot gaps."""

    normalized: dict[str, float | None] = {}
    for key, value in values.items():
        try:
            parsed = float(value) if value is not None else None
        except (TypeError, ValueError):
            parsed = None
        normalized[str(key)] = parsed if valid and parsed is not None and math.isfinite(parsed) else None
    return normalized


def append_sample(
    buffer: list[TrendSample],
    *,
    timestamp_s: float | None,
    values: Mapping[str, object],
    valid: bool,
    token: object,
    maxlen: int,
) -> bool:
    """Append a valid point or an unplotted gap marker, suppressing duplicates."""

    if timestamp_s is None or not math.isfinite(timestamp_s):
        return False
    if buffer and buffer[-1].token == token:
        return False
    buffer.append(TrendSample(timestamp_s, normalize_sample_values(values, valid=valid), valid, token))
    if len(buffer) > maxlen:
        del buffer[: len(buffer) - maxlen]
    return True


def series_for_key(samples: Sequence[TrendSample], key: str) -> list[tuple[float, float | None]]:
    """Return a series where invalid samples are explicit breaks, not zeroes."""

    return [(sample.timestamp_s, sample.values.get(key) if sample.valid else None) for sample in samples]


def split_valid_segments(series: Sequence[tuple[float, float | None]]) -> list[list[tuple[float, float]]]:
    """Split a series at invalid/NaN samples so no gap is bridged."""

    segments: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for timestamp, value in series:
        if value is None or not math.isfinite(value):
            if current:
                segments.append(current)
                current = []
            continue
        current.append((timestamp, value))
    if current:
        segments.append(current)
    return segments


__all__ = [
    "TrendSample",
    "append_sample",
    "normalize_sample_values",
    "series_for_key",
    "split_valid_segments",
]
