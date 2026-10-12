"""Existing scalar display formatting; no source access or frame conversion."""

from __future__ import annotations


def _number(value: object, default: float | None = None) -> float | None:
    if value in (None, ""):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _integer(value: object, default: int | None = None) -> int | None:
    if value in (None, ""):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _age(timestamp_s: object, now_s: float) -> float | None:
    value = _number(timestamp_s)
    if value is None:
        return None
    return max(0.0, now_s - value)


def _safe_len(value: object) -> int:
    try:
        return len(value)  # type: ignore[arg-type]
    except (TypeError, AttributeError):
        return 0
