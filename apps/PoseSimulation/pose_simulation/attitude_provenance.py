"""Pure SO(3) provenance validation for Simulation optical attitude injection."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .experiment_context import ValidatedSimulationExperimentContext
from .pose_math import (
    IDENTITY_ROTATION,
    Matrix3,
    Pose3D,
    compute_rotation_error,
    decompose_optical_rdf_error,
    matrix_multiply,
    transpose,
    validate_rotation_matrix,
)


INJECTION_VALID = "INJECTION_VALIDATION_VALID"
INJECTION_RESIDUAL_EXCEEDS_THRESHOLD = "INJECTION_RESIDUAL_EXCEEDS_THRESHOLD"
COMPOSITION_CLOSURE_FAILED = "COMPOSITION_CLOSURE_FAILED"
POSE_EVALUATION_INVALID = "POSE_EVALUATION_INVALID"


def optical_rpy_error_rotation(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> Matrix3:
    """Build ``R_N_A`` for the project's optical RDF RPY contract."""

    values = (float(roll_deg), float(pitch_deg), float(yaw_deg))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("commanded optical RPY must be finite")
    roll, pitch, yaw = (math.radians(value) for value in values)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_forward: Matrix3 = (
        (cr, -sr, 0.0),
        (sr, cr, 0.0),
        (0.0, 0.0, 1.0),
    )
    rotation_right: Matrix3 = (
        (1.0, 0.0, 0.0),
        (0.0, cp, -sp),
        (0.0, sp, cp),
    )
    rotation_down: Matrix3 = (
        (cy, 0.0, sy),
        (0.0, 1.0, 0.0),
        (-sy, 0.0, cy),
    )
    result = matrix_multiply(rotation_down, matrix_multiply(rotation_right, rotation_forward))
    validate_rotation_matrix(result)
    return result


@dataclass(frozen=True)
class AttitudeProvenanceResult:
    valid: bool
    status: str
    reason: str
    left_commanded_effective_rpy_deg: tuple[float, float, float]
    right_commanded_effective_rpy_deg: tuple[float, float, float]
    left_observed_nominal_to_actual_rpy_deg: tuple[float, float, float]
    right_observed_nominal_to_actual_rpy_deg: tuple[float, float, float]
    left_component_delta_rpy_deg: tuple[float, float, float]
    right_component_delta_rpy_deg: tuple[float, float, float]
    left_injection_residual_rpy_deg: tuple[float, float, float]
    right_injection_residual_rpy_deg: tuple[float, float, float]
    left_injection_residual_rotation_deg: float
    right_injection_residual_rotation_deg: float
    left_gt_ideal_to_nominal_rpy_deg: tuple[float, float, float]
    right_gt_ideal_to_nominal_rpy_deg: tuple[float, float, float]
    left_gt_ideal_to_actual_rpy_deg: tuple[float, float, float]
    right_gt_ideal_to_actual_rpy_deg: tuple[float, float, float]
    left_composition_closure_rotation_deg: float
    right_composition_closure_rotation_deg: float
    relative_commanded_rpy_deg: tuple[float, float, float]
    relative_observed_rpy_deg: tuple[float, float, float]
    relative_residual_rpy_deg: tuple[float, float, float]
    relative_residual_rotation_deg: float


def _rpy_and_angle(rotation: Sequence[Sequence[float]]) -> tuple[tuple[float, float, float], float]:
    validate_rotation_matrix(rotation)
    rpy = decompose_optical_rdf_error(rotation)
    return (
        (rpy.roll_error_deg, rpy.pitch_error_deg, rpy.yaw_error_deg),
        math.degrees(_rotation_angle_rad(rotation)),
    )


def _rotation_angle_rad(rotation: Sequence[Sequence[float]]) -> float:
    """Compute a stable SO(3) angle, including the near-identity case."""

    validate_rotation_matrix(rotation)
    cosine = max(-1.0, min(1.0, (rotation[0][0] + rotation[1][1] + rotation[2][2] - 1.0) / 2.0))
    sine = 0.5 * math.sqrt(
        (rotation[2][1] - rotation[1][2]) ** 2
        + (rotation[0][2] - rotation[2][0]) ** 2
        + (rotation[1][0] - rotation[0][1]) ** 2
    )
    # atan2 preserves the useful sub-microdegree signal instead of turning a
    # rounded trace just below one into an artificial 1e-6 degree error.
    return math.atan2(sine, cosine)


def _sub(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float]:
    return tuple(float(left[i]) - float(right[i]) for i in range(3))  # type: ignore[return-value]


def _pose_rotation(pose: Pose3D, name: str) -> Matrix3:
    if pose.frame_id != "camera_optical_rdf":
        raise ValueError(f"{name} must use camera_optical_rdf")
    validate_rotation_matrix(pose.rotation_world_from_frame)
    return pose.rotation_world_from_frame


def validate_attitude_provenance(
    context: ValidatedSimulationExperimentContext,
    *,
    ideal_left_pose_world: Pose3D,
    ideal_right_pose_world: Pose3D,
    actual_left_pose_world: Pose3D,
    actual_right_pose_world: Pose3D,
    max_injection_residual_deg: float = 1.0e-6,
    max_composition_closure_deg: float = 1.0e-6,
) -> AttitudeProvenanceResult:
    """Separate nominal geometry, observed injection, and GT ideal error."""

    if not math.isfinite(float(max_injection_residual_deg)) or max_injection_residual_deg < 0.0:
        raise ValueError("max_injection_residual_deg must be finite and non-negative")
    if not math.isfinite(float(max_composition_closure_deg)) or max_composition_closure_deg < 0.0:
        raise ValueError("max_composition_closure_deg must be finite and non-negative")

    nominal_left = _pose_rotation(context.nominal_left_pose_world, "nominal_left_pose_world")
    nominal_right = _pose_rotation(context.nominal_right_pose_world, "nominal_right_pose_world")
    ideal_left = _pose_rotation(ideal_left_pose_world, "ideal_left_pose_world")
    ideal_right = _pose_rotation(ideal_right_pose_world, "ideal_right_pose_world")
    actual_left = _pose_rotation(actual_left_pose_world, "actual_left_pose_world")
    actual_right = _pose_rotation(actual_right_pose_world, "actual_right_pose_world")

    commanded_left = optical_rpy_error_rotation(*context.left_effective_rpy_deg)
    commanded_right = optical_rpy_error_rotation(*context.right_effective_rpy_deg)
    observed_left = matrix_multiply(transpose(nominal_left), actual_left)
    observed_right = matrix_multiply(transpose(nominal_right), actual_right)

    commanded_left_rpy, _ = _rpy_and_angle(commanded_left)
    commanded_right_rpy, _ = _rpy_and_angle(commanded_right)
    observed_left_rpy, _ = _rpy_and_angle(observed_left)
    observed_right_rpy, _ = _rpy_and_angle(observed_right)

    left_residual = matrix_multiply(transpose(commanded_left), observed_left)
    right_residual = matrix_multiply(transpose(commanded_right), observed_right)
    left_residual_rpy, left_residual_angle = _rpy_and_angle(left_residual)
    right_residual_rpy, right_residual_angle = _rpy_and_angle(right_residual)

    left_ideal_to_nominal = matrix_multiply(transpose(ideal_left), nominal_left)
    right_ideal_to_nominal = matrix_multiply(transpose(ideal_right), nominal_right)
    left_ideal_to_actual = matrix_multiply(transpose(ideal_left), actual_left)
    right_ideal_to_actual = matrix_multiply(transpose(ideal_right), actual_right)
    left_geometry_rpy, _ = _rpy_and_angle(left_ideal_to_nominal)
    right_geometry_rpy, _ = _rpy_and_angle(right_ideal_to_nominal)
    left_total_rpy, _ = _rpy_and_angle(left_ideal_to_actual)
    right_total_rpy, _ = _rpy_and_angle(right_ideal_to_actual)

    left_expected_total = matrix_multiply(left_ideal_to_nominal, observed_left)
    right_expected_total = matrix_multiply(right_ideal_to_nominal, observed_right)
    left_closure = matrix_multiply(transpose(left_expected_total), left_ideal_to_actual)
    right_closure = matrix_multiply(transpose(right_expected_total), right_ideal_to_actual)
    left_closure_angle = math.degrees(_rotation_angle_rad(left_closure))
    right_closure_angle = math.degrees(_rotation_angle_rad(right_closure))

    nominal_relative = matrix_multiply(transpose(nominal_left), nominal_right)
    actual_relative = matrix_multiply(transpose(actual_left), actual_right)
    observed_relative = matrix_multiply(transpose(nominal_relative), actual_relative)
    commanded_relative = matrix_multiply(
        transpose(commanded_left),
        matrix_multiply(nominal_relative, commanded_right),
    )
    relative_residual = matrix_multiply(transpose(commanded_relative), observed_relative)
    relative_commanded_rpy, _ = _rpy_and_angle(commanded_relative)
    relative_observed_rpy, _ = _rpy_and_angle(observed_relative)
    relative_residual_rpy, relative_residual_angle = _rpy_and_angle(relative_residual)

    reason = "attitude provenance validated"
    status = INJECTION_VALID
    valid = True
    relative_residual_angle = math.degrees(_rotation_angle_rad(relative_residual))
    if max(left_residual_angle, right_residual_angle, relative_residual_angle) > max_injection_residual_deg:
        valid = False
        status = INJECTION_RESIDUAL_EXCEEDS_THRESHOLD
        reason = "commanded and observed optical injection differ beyond the configured SO(3) threshold"
    elif max(left_closure_angle, right_closure_angle) > max_composition_closure_deg:
        valid = False
        status = COMPOSITION_CLOSURE_FAILED
        reason = "GT ideal/nominal/actual composition closure exceeded the configured threshold"

    return AttitudeProvenanceResult(
        valid=valid,
        status=status,
        reason=reason,
        left_commanded_effective_rpy_deg=commanded_left_rpy,
        right_commanded_effective_rpy_deg=commanded_right_rpy,
        left_observed_nominal_to_actual_rpy_deg=observed_left_rpy,
        right_observed_nominal_to_actual_rpy_deg=observed_right_rpy,
        left_component_delta_rpy_deg=_sub(observed_left_rpy, commanded_left_rpy),
        right_component_delta_rpy_deg=_sub(observed_right_rpy, commanded_right_rpy),
        left_injection_residual_rpy_deg=left_residual_rpy,
        right_injection_residual_rpy_deg=right_residual_rpy,
        left_injection_residual_rotation_deg=left_residual_angle,
        right_injection_residual_rotation_deg=right_residual_angle,
        left_gt_ideal_to_nominal_rpy_deg=left_geometry_rpy,
        right_gt_ideal_to_nominal_rpy_deg=right_geometry_rpy,
        left_gt_ideal_to_actual_rpy_deg=left_total_rpy,
        right_gt_ideal_to_actual_rpy_deg=right_total_rpy,
        left_composition_closure_rotation_deg=left_closure_angle,
        right_composition_closure_rotation_deg=right_closure_angle,
        relative_commanded_rpy_deg=relative_commanded_rpy,
        relative_observed_rpy_deg=relative_observed_rpy,
        relative_residual_rpy_deg=relative_residual_rpy,
        relative_residual_rotation_deg=relative_residual_angle,
    )


__all__ = [
    "AttitudeProvenanceResult",
    "COMPOSITION_CLOSURE_FAILED",
    "INJECTION_RESIDUAL_EXCEEDS_THRESHOLD",
    "INJECTION_VALID",
    "POSE_EVALUATION_INVALID",
    "optical_rpy_error_rotation",
    "validate_attitude_provenance",
]
