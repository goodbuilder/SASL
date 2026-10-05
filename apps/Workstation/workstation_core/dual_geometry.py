"""Evaluate one immutable observation with nominal and active geometry.

The Workstation keeps raw detections as the authority.  This helper attaches
two reproducible, explicitly labelled evaluations to the same observation:
the run-start ``nominal`` K,D,R,t snapshot and the currently selected
``active`` snapshot.  It does not use post-flight UAM truth or mutate either
profile.

Pixels and stereo transforms use OpenCV optical RDF.  Public triangulation
points retain the existing ``baseline_center_frd`` [Forward, Right, Down]
contract.  The two profile results must never be compared as if their point
coordinates were NED.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

from Workstation.workstation_core.measurement_attempt import (
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
    build_checkerboard_baseline_snapshot,
)
from Workstation.workstation_core.measurement_controller import (
    MeasurementGeometryResult,
    calculate_measurement_geometry,
)


@dataclass(frozen=True)
class GeometryProfileEvaluation:
    role: str
    profile_sha256: str
    selection_id: str
    effective_from_pair_sequence: int | None
    result: MeasurementGeometryResult

    def to_record_fields(self) -> dict[str, Any]:
        prefix = f"{self.role}_"
        triangulated = self.result.triangulated
        estimate = self.result.estimate
        valid = triangulated is not None and estimate is not None
        point = None if triangulated is None else tuple(float(value) for value in triangulated.point_m)
        return {
            f"{prefix}geometry_valid": bool(valid),
            f"{prefix}geometry_reason": str(self.result.triangulation_reason),
            f"{prefix}profile_sha256": self.profile_sha256,
            f"{prefix}selection_id": self.selection_id,
            f"{prefix}effective_from_pair_sequence": self.effective_from_pair_sequence,
            f"{prefix}range_m": None if estimate is None else float(estimate.distance_m),
            f"{prefix}point_forward_m": None if point is None else point[0],
            f"{prefix}point_right_m": None if point is None else point[1],
            f"{prefix}point_down_m": None if point is None else point[2],
            f"{prefix}closest_ray_gap_m": (
                None if triangulated is None else float(triangulated.closest_gap_m)
            ),
            f"{prefix}ray_angle_deg": (
                None if triangulated is None else float(triangulated.ray_angle_deg)
            ),
            f"{prefix}warnings": list(self.result.warnings),
            f"{prefix}output_frame": (
                "baseline_center_frd" if triangulated is None else triangulated.output_frame
            ),
            f"{prefix}axis_contract": (
                "baseline_center_frd:[forward,right,down,range]"
                if triangulated is None
                else triangulated.output_vector_contract
            ),
        }


def attempt_with_checkerboard_profile(
    attempt: MeasurementAttempt,
    intrinsics: StereoIntrinsicsSnapshot,
) -> MeasurementAttempt:
    """Return a new attempt over the same observation and one immutable R,t."""

    if attempt.triangulation_source != "checkerboard":
        raise ValueError("dual optical-profile evaluation requires checkerboard R,t mode")
    baseline = build_checkerboard_baseline_snapshot(intrinsics)
    return replace(
        attempt,
        intrinsics=intrinsics,
        baseline=baseline,
        warnings=(
            ("waiting for checkerboard stereo extrinsics",)
            if baseline is None
            else tuple(baseline.warnings)
        ),
    )


def evaluate_profile(
    attempt: MeasurementAttempt,
    intrinsics: StereoIntrinsicsSnapshot,
    *,
    role: str,
    selection_id: str = "",
    effective_from_pair_sequence: int | None = None,
) -> GeometryProfileEvaluation:
    role_value = str(role).strip().lower()
    if role_value not in {"nominal", "active"}:
        raise ValueError("role must be nominal or active")
    profiled = attempt_with_checkerboard_profile(attempt, intrinsics)
    return GeometryProfileEvaluation(
        role=role_value,
        profile_sha256=intrinsics.content_sha256,
        selection_id=str(selection_id),
        effective_from_pair_sequence=(
            None
            if effective_from_pair_sequence is None
            else int(effective_from_pair_sequence)
        ),
        result=calculate_measurement_geometry(profiled),
    )


def evaluate_nominal_active(
    attempt: MeasurementAttempt,
    *,
    nominal_intrinsics: StereoIntrinsicsSnapshot,
    active_intrinsics: StereoIntrinsicsSnapshot,
    active_selection_id: str = "",
    active_effective_from_pair_sequence: int | None = None,
) -> tuple[GeometryProfileEvaluation, GeometryProfileEvaluation]:
    """Evaluate both profiles without changing the frozen observation."""

    return (
        evaluate_profile(attempt, nominal_intrinsics, role="nominal"),
        evaluate_profile(
            attempt,
            active_intrinsics,
            role="active",
            selection_id=active_selection_id,
            effective_from_pair_sequence=active_effective_from_pair_sequence,
        ),
    )


__all__ = [
    "GeometryProfileEvaluation",
    "attempt_with_checkerboard_profile",
    "evaluate_nominal_active",
    "evaluate_profile",
]
