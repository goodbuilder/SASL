"""Session values responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path


def _serializable_value(value: object) -> object:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {
            str(key): _serializable_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_serializable_value(item) for item in value]
    raise TypeError(f"session row contains unsupported value type: {type(value).__name__}")


def _seconds_to_ns(value: object) -> int | None:
    if value in (None, "") or isinstance(value, bool):
        return None
    try:
        seconds = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(seconds):
        return None
    return int(round(seconds * 1_000_000_000.0))


def _none_if_empty(value: object) -> object | None:
    if value is None:
        return None
    if isinstance(value, str) and value == "":
        return None
    return value


def _optional_int(value: object) -> int | None:
    value = _none_if_empty(value)
    if value is None:
        return None
    if isinstance(value, bool):
        return int(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(Decimal(str(value)).to_integral_value(rounding=ROUND_HALF_UP))
        except (InvalidOperation, ValueError):
            return None


def _optional_float(value: object) -> float | None:
    value = _none_if_empty(value)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_bool(value: object) -> bool | None:
    value = _none_if_empty(value)
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "y"}:
        return True
    if text in {"false", "0", "no", "n"}:
        return False
    return None


def _ns_from_seconds(value: object) -> int | None:
    value = _none_if_empty(value)
    if value is None:
        return None
    try:
        return int(
            (Decimal(str(value)) * Decimal(1_000_000_000)).to_integral_value(
                rounding=ROUND_HALF_UP
            )
        )
    except (InvalidOperation, ValueError):
        return None


def _first_present(row: dict[str, object], *keys: str) -> object | None:
    for key in keys:
        value = _none_if_empty(row.get(key))
        if value is not None:
            return value
    return None
