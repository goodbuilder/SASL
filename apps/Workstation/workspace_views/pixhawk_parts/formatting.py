"""Existing display formatting only; no position or attitude transform."""

from __future__ import annotations
import math
from collections.abc import Mapping
from Workstation.workstation_core.px4_attitude_cal_manager import board_rotation_label
from Workstation.workstation_core.px4_mag_manager import mag_priority_display


def _display_signed_number(value: object, *, digits: int = 3, suffix: str = "") -> str:
    if value in (None, ""):
        return "--"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(number):
        return "--"
    return f"{number:+.{digits}f}{suffix}"


def _display_compass_deg(value: object) -> str:
    if value in (None, ""):
        return "--"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(number):
        return "--"
    normalized = number % 360.0
    if round(normalized, 2) >= 360.0:
        normalized = 0.0
    return f"{normalized:06.2f}"


def _display_signed_deg(value: object) -> str:
    if value in (None, ""):
        return "--"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(number):
        return "--"
    wrapped = (number + 180.0) % 360.0 - 180.0
    if abs(wrapped) < 0.005:
        wrapped = 0.0
    if round(wrapped, 2) >= 180.0:
        wrapped = -180.0
    return f"{wrapped:+07.2f}"


def _display_magnetometer_px4_yaw(row: Mapping[str, object]) -> str:
    """Render PX4 yaw only when the evidence row is exactly selected."""

    value = _display_signed_deg(row.get("px4_yaw_deg"))
    if value != "--":
        return value
    return {
        "WAITING": "대기",
        "STALE": "STALE",
        "ACTIVE ID WAITING": "active ID 대기",
        "IDENTITY UNKNOWN": "ID 확인 대기",
        "NOT SELECTED": "미선택",
    }.get(str(row.get("px4_yaw_state", "WAITING")), "--")


def _display_magnetometer_corrected_yaw(row: Mapping[str, object]) -> str:
    value = _display_signed_deg(row.get("corrected_yaw_deg"))
    if value != "--":
        return value
    return {
        "WAITING": "대기",
        "PARAM WAITING": "PARAM 대기",
        "PARAM INCOMPLETE": "PARAM 불완전",
        "POWER COMP PENDING": "전류 보정 대기",
        "BOARD ROTATION WAITING": "보드 회전 대기",
        "IDENTITY UNKNOWN": "ID 확인 대기",
    }.get(str(row.get("corrected_yaw_state", "WAITING")), "--")


def _magnetometer_yaw_summary(row: Mapping[str, object]) -> str:
    """Build a compact Inspector summary for raw and corrected yaw."""

    raw = _display_signed_deg(row.get("raw_yaw_deg"))
    corrected = _display_magnetometer_corrected_yaw(row)
    return f"RAW={raw}° · Corrected={corrected}"


def _format_px4_parameter_value(name: str, value: object) -> str:
    if value in (None, ""):
        return "N/A"
    if name.endswith("_PRIO"):
        return mag_priority_display(value)  # type: ignore[arg-type]
    if name.endswith("_ROT"):
        raw = int(value)
        return f"{board_rotation_label(raw)} ({raw})"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)
