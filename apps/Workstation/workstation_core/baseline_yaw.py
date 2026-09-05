"""GPS-baseline perpendicular yaw selection for Measurement geometry.

The P1->P2 horizontal baseline has two perpendicular camera-forward
directions.  This module keeps the branch calculation pure so the controller
can freeze its result in one Measurement attempt and the worker can reproduce
it without reading live UI state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


BASELINE_YAW_MINUS_90_DEG = -90.0
BASELINE_YAW_PLUS_90_DEG = 90.0
BASELINE_YAW_LOW_MARGIN_DEG = 5.0


def wrap_compass_deg(value: float) -> float:
    """Normalize a compass heading to [0, 360)."""

    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("compass angle must be finite")
    return parsed % 360.0


def wrap_signed_deg(value: float) -> float:
    """Normalize an angular difference to [-180, 180)."""

    wrapped = (float(value) + 180.0) % 360.0 - 180.0
    return 0.0 if abs(wrapped) < 1.0e-12 else wrapped


def baseline_azimuth_deg(north_m: float, east_m: float) -> float:
    """Return the P1->P2 horizontal NED azimuth in degrees."""

    north = float(north_m)
    east = float(east_m)
    if not math.isfinite(north) or not math.isfinite(east):
        raise ValueError("baseline north/east must be finite")
    if math.hypot(north, east) <= 1.0e-9:
        raise ValueError("baseline horizontal direction is undefined")
    return wrap_compass_deg(math.degrees(math.atan2(east, north)))


@dataclass(frozen=True)
class BaselineYawSelection:
    """Deterministic branch selection plus its auditable evidence."""

    baseline_azimuth_deg: float
    candidate_minus90_deg: float
    candidate_plus90_deg: float
    selected_yaw_deg: float | None
    selected_offset_deg: float | None
    mode: str
    reverse: bool
    p1_reference_yaw_deg: float | None
    p2_reference_yaw_deg: float | None
    p1_error_minus90_deg: float | None
    p1_error_plus90_deg: float | None
    p2_error_minus90_deg: float | None
    p2_error_plus90_deg: float | None
    score_minus90_deg: float | None
    score_plus90_deg: float | None
    consensus: bool | None
    low_margin: bool
    ready: bool
    reason: str = ""

    def to_mapping(self) -> dict[str, object]:
        return {
            "baseline_azimuth_deg": self.baseline_azimuth_deg,
            "candidate_minus90_deg": self.candidate_minus90_deg,
            "candidate_plus90_deg": self.candidate_plus90_deg,
            "selected_yaw_deg": self.selected_yaw_deg,
            "selected_offset_deg": self.selected_offset_deg,
            "mode": self.mode,
            "reverse": self.reverse,
            "p1_reference_yaw_deg": self.p1_reference_yaw_deg,
            "p2_reference_yaw_deg": self.p2_reference_yaw_deg,
            "p1_error_minus90_deg": self.p1_error_minus90_deg,
            "p1_error_plus90_deg": self.p1_error_plus90_deg,
            "p2_error_minus90_deg": self.p2_error_minus90_deg,
            "p2_error_plus90_deg": self.p2_error_plus90_deg,
            "score_minus90_deg": self.score_minus90_deg,
            "score_plus90_deg": self.score_plus90_deg,
            "consensus": self.consensus,
            "low_margin": self.low_margin,
            "ready": self.ready,
            "reason": self.reason,
        }


def _finite_yaw(value: object) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return wrap_compass_deg(parsed) if math.isfinite(parsed) else None


def _error(candidate: float, reference: float | None) -> float | None:
    return None if reference is None else abs(wrap_signed_deg(candidate - reference))


def select_baseline_yaw(
    *,
    north_m: float,
    east_m: float,
    p1_yaw_deg: object = None,
    p2_yaw_deg: object = None,
    reverse: bool = False,
) -> BaselineYawSelection:
    """Select one of the two perpendicular baseline yaw candidates.

    P1/P2 references are equally weighted.  If only one is available it is
    used alone; if neither is available the branch is not ready.  Reverse is
    applied after Auto scoring so it is an explicit operator override rather
    than a hidden change to the reference selection.
    """

    azimuth = baseline_azimuth_deg(north_m, east_m)
    minus90 = wrap_compass_deg(azimuth + BASELINE_YAW_MINUS_90_DEG)
    plus90 = wrap_compass_deg(azimuth + BASELINE_YAW_PLUS_90_DEG)
    p1 = _finite_yaw(p1_yaw_deg)
    p2 = _finite_yaw(p2_yaw_deg)
    p1_minus = _error(minus90, p1)
    p1_plus = _error(plus90, p1)
    p2_minus = _error(minus90, p2)
    p2_plus = _error(plus90, p2)
    minus_errors = tuple(value for value in (p1_minus, p2_minus) if value is not None)
    plus_errors = tuple(value for value in (p1_plus, p2_plus) if value is not None)
    if not minus_errors and not plus_errors and bool(reverse):
        return BaselineYawSelection(
            baseline_azimuth_deg=azimuth,
            candidate_minus90_deg=minus90,
            candidate_plus90_deg=plus90,
            selected_yaw_deg=plus90,
            selected_offset_deg=BASELINE_YAW_PLUS_90_DEG,
            mode="reverse_forced",
            reverse=True,
            p1_reference_yaw_deg=p1,
            p2_reference_yaw_deg=p2,
            p1_error_minus90_deg=p1_minus,
            p1_error_plus90_deg=p1_plus,
            p2_error_minus90_deg=p2_minus,
            p2_error_plus90_deg=p2_plus,
            score_minus90_deg=None,
            score_plus90_deg=None,
            consensus=None,
            low_margin=False,
            ready=True,
            reason="persistent Reverse branch",
        )
    if not minus_errors and not plus_errors:
        return BaselineYawSelection(
            baseline_azimuth_deg=azimuth,
            candidate_minus90_deg=minus90,
            candidate_plus90_deg=plus90,
            selected_yaw_deg=None,
            selected_offset_deg=None,
            mode="auto",
            reverse=bool(reverse),
            p1_reference_yaw_deg=p1,
            p2_reference_yaw_deg=p2,
            p1_error_minus90_deg=p1_minus,
            p1_error_plus90_deg=p1_plus,
            p2_error_minus90_deg=p2_minus,
            p2_error_plus90_deg=p2_plus,
            score_minus90_deg=None,
            score_plus90_deg=None,
            consensus=None,
            low_margin=False,
            ready=False,
            reason="Pixhawk yaw reference unavailable",
        )

    score_minus = sum(minus_errors) / len(minus_errors)
    score_plus = sum(plus_errors) / len(plus_errors)
    auto_minus = score_minus <= score_plus
    selected_minus = not auto_minus if bool(reverse) else auto_minus
    selected = minus90 if selected_minus else plus90
    selected_offset = BASELINE_YAW_MINUS_90_DEG if selected_minus else BASELINE_YAW_PLUS_90_DEG
    votes = []
    if p1_minus is not None and p1_plus is not None:
        votes.append(p1_minus <= p1_plus)
    if p2_minus is not None and p2_plus is not None:
        votes.append(p2_minus <= p2_plus)
    consensus = None if not votes else len(set(votes)) == 1
    return BaselineYawSelection(
        baseline_azimuth_deg=azimuth,
        candidate_minus90_deg=minus90,
        candidate_plus90_deg=plus90,
        selected_yaw_deg=selected,
        selected_offset_deg=selected_offset,
        mode="auto_reverse" if bool(reverse) else "auto",
        reverse=bool(reverse),
        p1_reference_yaw_deg=p1,
        p2_reference_yaw_deg=p2,
        p1_error_minus90_deg=p1_minus,
        p1_error_plus90_deg=p1_plus,
        p2_error_minus90_deg=p2_minus,
        p2_error_plus90_deg=p2_plus,
        score_minus90_deg=score_minus,
        score_plus90_deg=score_plus,
        consensus=consensus,
        low_margin=abs(score_minus - score_plus) < BASELINE_YAW_LOW_MARGIN_DEG,
        ready=True,
        reason="P1/P2 yaw aggregate" if consensus is not False else "P1/P2 yaw branch disagreement",
    )


def quaternion_from_roll_pitch_yaw_deg(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float, float]:
    """Build a normalized PX4-style wxyz quaternion from Euler degrees."""

    roll = math.radians(float(roll_deg))
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    values = (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("effective baseline-yaw quaternion must be finite")
    return values


__all__ = [
    "BASELINE_YAW_LOW_MARGIN_DEG",
    "BASELINE_YAW_MINUS_90_DEG",
    "BASELINE_YAW_PLUS_90_DEG",
    "BaselineYawSelection",
    "baseline_azimuth_deg",
    "quaternion_from_roll_pitch_yaw_deg",
    "select_baseline_yaw",
    "wrap_compass_deg",
    "wrap_signed_deg",
]
