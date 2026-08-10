"""Pure GT-only measurement integration for PoseSimulation M3.

This module is the boundary between the M2 input contract and the M1 pose
mathematics.  It has no ROS, Gazebo, PX4, UI, or file-system dependency.  A
caller supplies one validated/aligned input set and receives the ideal poses
and diagnostic metrics for that evaluation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .evaluation import IdealStereoPoses, NominalStereoExtrinsics
from .input_types import AlignedPoseInputs
from .pose_math import (
    BaselineGeometryResult,
    CAMERA_OPTICAL_FRAME,
    CameraPoseError,
    Pose3D,
    PoseContract,
    StereoRelativePoseError,
    compute_baseline_geometry,
    compute_camera_pose_error,
    compute_look_at_rig_rotation,
    compute_relative_camera_pose_error,
    derive_camera_optical_pose,
    quaternion_xyzw_to_matrix,
)


MEASUREMENT_VALID = "VALID"
MEASUREMENT_INPUT_INVALID = "INPUT_INVALID"
MEASUREMENT_CONTRACT_INVALID = "POSE_CONTRACT_INVALID"
MEASUREMENT_MATH_INVALID = "MEASUREMENT_INVALID"


@dataclass(frozen=True)
class GtMeasurementResult:
    """One GT-triggered measurement result.

    Positions are metres in ``gazebo_world_enu``.  Ideal and actual camera
    rotations are ``R_W_C`` where C is Camera Optical RDF.  ``evaluation_id``
    is assigned by the ROS node; this pure result remains usable in tests.
    """

    evaluation_id: int
    evaluation_stamp_ns: int | None
    valid: bool
    status: str
    reason: str
    alignment_method: str
    ideal_ground_truth: IdealStereoPoses | None
    left_error: CameraPoseError | None
    right_error: CameraPoseError | None
    relative_error: StereoRelativePoseError | None
    baseline: BaselineGeometryResult | None
    ground_truth: AlignedPoseInputs


class GtMeasurementEngine:
    """Stateful, ROS-free evaluator that assigns monotonic evaluation IDs."""

    def __init__(self, contract: PoseContract, *, near_vertical_threshold: float = 1.0e-9) -> None:
        self.contract = contract
        self.near_vertical_threshold = float(near_vertical_threshold)
        self.next_evaluation_id = 0
        self.previous_right_axis_world_enu: tuple[float, float, float] | None = None

    def evaluate(self, aligned: AlignedPoseInputs) -> GtMeasurementResult:
        self.next_evaluation_id += 1
        result = evaluate_gt_aligned_inputs(
            aligned,
            self.contract,
            evaluation_id=self.next_evaluation_id,
            previous_right_axis_world_enu=self.previous_right_axis_world_enu,
            near_vertical_threshold=self.near_vertical_threshold,
        )
        if (
            result.valid
            and result.ideal_ground_truth is not None
            and result.ideal_ground_truth.rig_pose is not None
        ):
            rotation = result.ideal_ground_truth.rig_pose.rotation_world_from_frame
            self.previous_right_axis_world_enu = (
                rotation[0][1],
                rotation[1][1],
                rotation[2][1],
            )
        return result


def nominal_extrinsics_from_contract(contract: PoseContract) -> NominalStereoExtrinsics:
    """Build M1's validated nominal extrinsic object from the M1 contract."""

    return NominalStereoExtrinsics(
        left_translation_rig_m=contract.left_center_rig_frd_m,
        right_translation_rig_m=contract.right_center_rig_frd_m,
        left_rotation_rig_from_optical=contract.rotation_rig_from_left_optical,
        right_rotation_rig_from_optical=contract.rotation_rig_from_right_optical,
    ).validate()


def _invalid_result(
    *,
    evaluation_id: int,
    aligned: AlignedPoseInputs,
    status: str,
    reason: str,
    ideal_ground_truth: IdealStereoPoses | None = None,
    left_error: CameraPoseError | None = None,
    right_error: CameraPoseError | None = None,
    relative_error: StereoRelativePoseError | None = None,
    baseline: BaselineGeometryResult | None = None,
) -> GtMeasurementResult:
    return GtMeasurementResult(
        evaluation_id=evaluation_id,
        evaluation_stamp_ns=aligned.evaluation_stamp_ns,
        valid=False,
        status=status,
        reason=reason,
        alignment_method=aligned.alignment_method,
        ideal_ground_truth=ideal_ground_truth,
        left_error=left_error,
        right_error=right_error,
        relative_error=relative_error,
        baseline=baseline,
        ground_truth=aligned,
    )


def evaluate_gt_aligned_inputs(
    aligned: AlignedPoseInputs,
    contract: PoseContract,
    *,
    evaluation_id: int,
    previous_right_axis_world_enu: tuple[float, float, float] | None = None,
    near_vertical_threshold: float = 1.0e-9,
) -> GtMeasurementResult:
    """Evaluate one validated M2 input set with the M1 GT ideal policy.

    The rig origin is always the midpoint of the observed left/right optical
    centers.  This is intentionally an orientation comparison at the actual
    optical midpoint; no rig position error is invented.
    """

    if not aligned.valid:
        return _invalid_result(
            evaluation_id=evaluation_id,
            aligned=aligned,
            status=aligned.status or MEASUREMENT_INPUT_INVALID,
            reason=aligned.invalid_reason or "aligned input set is invalid",
        )
    if (
        aligned.ground_truth is None
        or aligned.left_actual_camera is None
        or aligned.right_actual_camera is None
    ):
        return _invalid_result(
            evaluation_id=evaluation_id,
            aligned=aligned,
            status=MEASUREMENT_INPUT_INVALID,
            reason="aligned input set is missing a required pose",
        )
    if not (
        aligned.ground_truth.valid
        and aligned.left_actual_camera.valid
        and aligned.right_actual_camera.valid
    ):
        return _invalid_result(
            evaluation_id=evaluation_id,
            aligned=aligned,
            status=MEASUREMENT_INPUT_INVALID,
            reason="one or more required pose samples are invalid",
        )

    gt = aligned.ground_truth
    left = aligned.left_actual_camera
    right = aligned.right_actual_camera
    if (
        gt.position_world_enu_m is None
        or left.position_world_enu_m is None
        or right.position_world_enu_m is None
        or left.orientation_world_from_optical_xyzw is None
        or right.orientation_world_from_optical_xyzw is None
    ):
        return _invalid_result(
            evaluation_id=evaluation_id,
            aligned=aligned,
            status=MEASUREMENT_INPUT_INVALID,
            reason="a valid input is missing its numeric pose fields",
        )

    try:
        nominal = nominal_extrinsics_from_contract(contract)
        left_actual_pose = Pose3D(
            left.position_world_enu_m,
            quaternion_xyzw_to_matrix(left.orientation_world_from_optical_xyzw),
            CAMERA_OPTICAL_FRAME,
            left.gazebo_stamp_ns,
        )
        right_actual_pose = Pose3D(
            right.position_world_enu_m,
            quaternion_xyzw_to_matrix(right.orientation_world_from_optical_xyzw),
            CAMERA_OPTICAL_FRAME,
            right.gazebo_stamp_ns,
        )
    except (TypeError, ValueError, OverflowError) as exc:
        return _invalid_result(
            evaluation_id=evaluation_id,
            aligned=aligned,
            status=MEASUREMENT_CONTRACT_INVALID,
            reason=str(exc),
        )

    rig_center = tuple(
        0.5 * (left.position_world_enu_m[index] + right.position_world_enu_m[index])
        for index in range(3)
    )
    look_at = compute_look_at_rig_rotation(
        rig_center,
        gt.position_world_enu_m,
        contract.world_up_world_enu,
        previous_right_axis_world_enu,
        near_vertical_threshold=near_vertical_threshold,
    )
    if not look_at.valid or look_at.rotation_world_from_rig is None:
        return _invalid_result(
            evaluation_id=evaluation_id,
            aligned=aligned,
            status=look_at.status,
            reason=f"GT ideal rig construction failed: {look_at.status}",
        )

    rig_pose = Pose3D(
        rig_center,
        look_at.rotation_world_from_rig,
        contract.rig_frame,
        aligned.evaluation_stamp_ns,
    )
    left_ideal_pose = derive_camera_optical_pose(
        rig_center,
        look_at.rotation_world_from_rig,
        nominal.left_translation_rig_m,
        nominal.left_rotation_rig_from_optical,
        source_timestamp_ns=aligned.evaluation_stamp_ns,
    )
    right_ideal_pose = derive_camera_optical_pose(
        rig_center,
        look_at.rotation_world_from_rig,
        nominal.right_translation_rig_m,
        nominal.right_rotation_rig_from_optical,
        source_timestamp_ns=aligned.evaluation_stamp_ns,
    )
    ideal = IdealStereoPoses(
        source_type="ground_truth",
        target_world_enu_m=gt.position_world_enu_m,
        rig_pose=rig_pose,
        left_pose=left_ideal_pose,
        right_pose=right_ideal_pose,
        look_at_status=look_at.status,
        valid=True,
    )

    try:
        left_error = compute_camera_pose_error(
            left_ideal_pose, left_actual_pose, gt.position_world_enu_m
        )
        right_error = compute_camera_pose_error(
            right_ideal_pose, right_actual_pose, gt.position_world_enu_m
        )
        relative = compute_relative_camera_pose_error(
            left_actual_pose.rotation_world_from_frame,
            right_actual_pose.rotation_world_from_frame,
            nominal.left_rotation_rig_from_optical,
            nominal.right_rotation_rig_from_optical,
        )
        baseline = compute_baseline_geometry(
            left_actual_pose.position_world_enu_m,
            right_actual_pose.position_world_enu_m,
            nominal.nominal_baseline_vector_rig_m,
            look_at.rotation_world_from_rig,
            nominal_midpoint_world_enu_m=None,
        )
    except (TypeError, ValueError, OverflowError) as exc:
        return _invalid_result(
            evaluation_id=evaluation_id,
            aligned=aligned,
            status=MEASUREMENT_MATH_INVALID,
            reason=str(exc),
            ideal_ground_truth=ideal,
        )

    metrics_valid = (
        left_error.valid
        and right_error.valid
        and relative.valid
        and baseline.valid
        and left_error.optical_axis_to_target_error_deg is not None
        and right_error.optical_axis_to_target_error_deg is not None
        and relative.optical_axis_angle_deg is not None
        and baseline.direction_error_deg is not None
        and all(
            math.isfinite(value)
            for value in (
                left_error.rotation.rotation_error_deg,
                right_error.rotation.rotation_error_deg,
                relative.rotation_error_deg,
                baseline.actual_length_m,
                baseline.length_error_m,
            )
        )
    )
    if not metrics_valid:
        return _invalid_result(
            evaluation_id=evaluation_id,
            aligned=aligned,
            status=MEASUREMENT_MATH_INVALID,
            reason="one or more GT attitude or geometry metrics is invalid",
            ideal_ground_truth=ideal,
            left_error=left_error,
            right_error=right_error,
            relative_error=relative,
            baseline=baseline,
        )
    return GtMeasurementResult(
        evaluation_id=evaluation_id,
        evaluation_stamp_ns=aligned.evaluation_stamp_ns,
        valid=True,
        status=MEASUREMENT_VALID,
        reason="",
        alignment_method=aligned.alignment_method,
        ideal_ground_truth=ideal,
        left_error=left_error,
        right_error=right_error,
        relative_error=relative,
        baseline=baseline,
        ground_truth=aligned,
    )


__all__ = [
    "GtMeasurementResult",
    "GtMeasurementEngine",
    "MEASUREMENT_CONTRACT_INVALID",
    "MEASUREMENT_INPUT_INVALID",
    "MEASUREMENT_MATH_INVALID",
    "MEASUREMENT_VALID",
    "evaluate_gt_aligned_inputs",
    "nominal_extrinsics_from_contract",
]
