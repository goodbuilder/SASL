"""Pure evidence model for the Workstation physical-alignment procedure.

The operator physically moves the P1/P2 assembly.  This module therefore
evaluates observations and records evidence; it never applies a roll, pitch,
yaw, translation, or camera-frame correction.

Coordinate contract
-------------------
``baseline_ned_m`` is the raw GPS/RTK P1->P2 vector in local NED order
``[north, east, down]``.  ``baseline_frd_m`` is a separate projection of that
same vector into P1 body FRD order ``[forward, right, down]`` using P1's
attitude quaternion (body FRD -> world NED).  ``relative_rpy_deg`` is the
P2-relative-to-P1 attitude in ``[roll, pitch, yaw]`` degrees.  No value is
written into an optical RDF or public measurement field here.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, isfinite, radians, sin
from statistics import mean, pstdev
from typing import Any, Iterable

from workstation_core.attitude_geometry import relative_attitude, world_ned_vector_to_body_frd


Vector3 = tuple[float, float, float]


def _vector3(value: Iterable[float], name: str) -> Vector3:
    values = tuple(float(item) for item in value)
    if len(values) != 3 or not all(isfinite(item) for item in values):
        raise ValueError(f"{name} must contain three finite values")
    return values  # type: ignore[return-value]


def wrap_angle_deg(value: float) -> float:
    """Return an angle in the half-open interval [-180, 180)."""

    wrapped = (float(value) + 180.0) % 360.0 - 180.0
    # Keep a positive 180-degree input represented consistently at -180.
    return -180.0 if wrapped == 180.0 else wrapped


def angle_error_deg(reference: float, current: float) -> float:
    """Return the shortest signed current-reference angular error."""

    return wrap_angle_deg(float(current) - float(reference))


@dataclass(frozen=True)
class PhysicalAlignmentSample:
    """One simultaneous physical-alignment observation."""

    timestamp_s: float
    baseline_ned_m: Vector3
    baseline_frd_m: Vector3
    relative_rpy_deg: Vector3
    left_fix_type: int | None = None
    right_fix_type: int | None = None

    def __post_init__(self) -> None:
        timestamp = float(self.timestamp_s)
        if not isfinite(timestamp):
            raise ValueError("timestamp_s must be finite")
        object.__setattr__(self, "timestamp_s", timestamp)
        object.__setattr__(self, "baseline_ned_m", _vector3(self.baseline_ned_m, "baseline_ned_m"))
        object.__setattr__(self, "baseline_frd_m", _vector3(self.baseline_frd_m, "baseline_frd_m"))
        object.__setattr__(self, "relative_rpy_deg", _vector3(self.relative_rpy_deg, "relative_rpy_deg"))
        for name in ("left_fix_type", "right_fix_type"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, int(value))

    @property
    def both_rtk_fixed(self) -> bool:
        return self.left_fix_type == 6 and self.right_fix_type == 6

    @property
    def minimum_fix_type(self) -> int | None:
        values = [value for value in (self.left_fix_type, self.right_fix_type) if value is not None]
        return min(values) if values else None


def make_physical_alignment_sample(
    *,
    timestamp_s: float,
    baseline_ned_m: Iterable[float],
    p1_body_to_world_q: Iterable[float],
    p2_body_to_world_q: Iterable[float],
    left_fix_type: int | None = None,
    right_fix_type: int | None = None,
) -> PhysicalAlignmentSample:
    """Build a sample while making the NED -> P1 body FRD conversion explicit."""

    ned = _vector3(baseline_ned_m, "baseline_ned_m")
    frd = world_ned_vector_to_body_frd(ned, p1_body_to_world_q)
    relative = relative_attitude(p1_body_to_world_q, p2_body_to_world_q)
    return PhysicalAlignmentSample(
        timestamp_s=timestamp_s,
        baseline_ned_m=ned,
        baseline_frd_m=frd,
        relative_rpy_deg=(relative.roll_deg, relative.pitch_deg, relative.yaw_deg),
        left_fix_type=left_fix_type,
        right_fix_type=right_fix_type,
    )


@dataclass(frozen=True)
class PhysicalAlignmentQuality:
    """Summary of one sampling iteration."""

    state: str
    sample_count: int
    baseline_frd_mean_m: Vector3
    baseline_frd_std_m: Vector3
    relative_rpy_mean_deg: Vector3
    relative_rpy_std_deg: Vector3
    baseline_error_m: Vector3 | None
    relative_rpy_error_deg: Vector3
    fixed_sample_count: int
    fixed_ratio: float
    minimum_fix_type_observed: int | None
    reasons: tuple[str, ...]

    def to_mapping(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "sample_count": self.sample_count,
            "baseline_frd_mean_m": list(self.baseline_frd_mean_m),
            "baseline_frd_std_m": list(self.baseline_frd_std_m),
            "relative_rpy_mean_deg": list(self.relative_rpy_mean_deg),
            "relative_rpy_std_deg": list(self.relative_rpy_std_deg),
            "baseline_error_m": None if self.baseline_error_m is None else list(self.baseline_error_m),
            "relative_rpy_error_deg": list(self.relative_rpy_error_deg),
            "fixed_sample_count": self.fixed_sample_count,
            "fixed_ratio": self.fixed_ratio,
            "minimum_fix_type_observed": self.minimum_fix_type_observed,
            "reasons": list(self.reasons),
            "coordinate_contract": {
                "baseline_ned": "[north,east,down] metres",
                "baseline_frd": "P1->P2 [forward,right,down] metres",
                "relative_rpy": "P2 relative P1 [roll,pitch,yaw] degrees",
            },
        }


def _mean_vector(values: list[Vector3]) -> Vector3:
    return tuple(mean(axis) for axis in zip(*values))  # type: ignore[return-value]


def _std_vector(values: list[Vector3]) -> Vector3:
    if len(values) < 2:
        return 0.0, 0.0, 0.0
    return tuple(pstdev(axis) for axis in zip(*values))  # type: ignore[return-value]


def _mean_angle(values: list[float]) -> float:
    sine = mean(sin(radians(value)) for value in values)
    cosine = mean(cos(radians(value)) for value in values)
    return degrees(atan2(sine, cosine))


def _mean_rpy(values: list[Vector3]) -> Vector3:
    return tuple(_mean_angle(list(axis)) for axis in zip(*values))  # type: ignore[return-value]


def _std_rpy(values: list[Vector3], center: Vector3) -> Vector3:
    if len(values) < 2:
        return 0.0, 0.0, 0.0
    errors = [
        tuple(angle_error_deg(reference, current) for reference, current in zip(center, value))
        for value in values
    ]
    return tuple(pstdev(axis) for axis in zip(*errors))  # type: ignore[return-value]


def evaluate_physical_alignment(
    samples: Iterable[PhysicalAlignmentSample],
    *,
    target_baseline_frd_m: Iterable[float] | None = None,
    target_relative_rpy_deg: Iterable[float] = (0.0, 0.0, 0.0),
    baseline_tolerance_m: float = 0.02,
    rpy_tolerance_deg: float = 1.0,
    minimum_fix_type: int = 3,
    minimum_fixed_ratio: float = 0.0,
) -> PhysicalAlignmentQuality | None:
    """Evaluate evidence without requiring RTK_FIXED in every situation.

    ``minimum_fix_type`` is the numeric PX4/MAVLink ``fix_type`` acceptance
    threshold.  RTK_FIXED (6) is only required when the threshold is set to 6
    or higher, or when ``minimum_fixed_ratio`` is explicitly requested.
    """

    rows = list(samples)
    if not rows:
        return None
    if baseline_tolerance_m < 0.0 or rpy_tolerance_deg < 0.0:
        raise ValueError("alignment tolerances must be >= 0")
    if not 0.0 <= minimum_fixed_ratio <= 1.0:
        raise ValueError("minimum_fixed_ratio must be between 0 and 1")
    target_baseline = None if target_baseline_frd_m is None else _vector3(target_baseline_frd_m, "target_baseline_frd_m")
    target_rpy = _vector3(target_relative_rpy_deg, "target_relative_rpy_deg")
    minimum_fix_type = int(minimum_fix_type)

    frd_values = [sample.baseline_frd_m for sample in rows]
    rpy_values = [sample.relative_rpy_deg for sample in rows]
    frd_mean = _mean_vector(frd_values)
    rpy_mean = _mean_rpy(rpy_values)
    frd_std = _std_vector(frd_values)
    rpy_std = _std_rpy(rpy_values, rpy_mean)
    baseline_error = None
    if target_baseline is not None:
        baseline_error = tuple(current - target for current, target in zip(frd_mean, target_baseline))  # type: ignore[assignment]
    rpy_error = tuple(
        angle_error_deg(reference, current)
        for reference, current in zip(target_rpy, rpy_mean)
    )
    fixed_count = sum(1 for sample in rows if sample.both_rtk_fixed)
    fixed_ratio = fixed_count / len(rows)
    fix_values = [sample.minimum_fix_type for sample in rows if sample.minimum_fix_type is not None]
    minimum_observed = min(fix_values) if fix_values else None

    reasons: list[str] = []
    if minimum_observed is None:
        reasons.append("fix_type_missing")
    elif minimum_observed < minimum_fix_type:
        reasons.append("fix_type_below_minimum")
    if target_baseline is None:
        reasons.append("baseline_reference_missing")
    elif any(abs(value) > baseline_tolerance_m for value in baseline_error or ()):
        reasons.append("baseline_frd_out_of_tolerance")
    if any(abs(value) > rpy_tolerance_deg for value in rpy_error):
        reasons.append("relative_rpy_out_of_tolerance")
    if fixed_ratio < minimum_fixed_ratio:
        reasons.append("fixed_ratio_below_minimum")

    # A sampling iteration with no reference baseline is useful evidence, but
    # cannot be declared physically aligned until a reference is selected.
    if target_baseline is None:
        state = "EVIDENCE" if all(reason == "baseline_reference_missing" for reason in reasons) else "FAIL"
    else:
        state = "PASS" if not reasons else "FAIL"

    return PhysicalAlignmentQuality(
        state=state,
        sample_count=len(rows),
        baseline_frd_mean_m=frd_mean,
        baseline_frd_std_m=frd_std,
        relative_rpy_mean_deg=rpy_mean,
        relative_rpy_std_deg=rpy_std,
        baseline_error_m=baseline_error,
        relative_rpy_error_deg=rpy_error,
        fixed_sample_count=fixed_count,
        fixed_ratio=fixed_ratio,
        minimum_fix_type_observed=minimum_observed,
        reasons=tuple(reasons),
    )
