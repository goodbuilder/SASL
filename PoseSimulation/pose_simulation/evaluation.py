"""Pure evaluation pipeline for one PoseSimulation snapshot."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from .coordinate_adapter import Wgs84ReferenceOrigin
from .pose_math import (
    BaselineGeometryResult,
    CameraPoseError,
    Matrix3,
    Pose3D,
    RotationError,
    StereoRelativePoseError,
    Vector3,
    angle_between,
    as_matrix3,
    as_vector3,
    compute_baseline_geometry,
    compute_camera_pose_error,
    compute_look_at_rig_rotation,
    compute_relative_camera_pose_error,
    compute_rotation_error,
    derive_camera_optical_pose,
    matrix_to_quaternion_xyzw,
    matrix_vector,
)
from .sources import PoseSourceSample


# Optical RDF [Right, Down, Forward] components to rig FRD [Front, Right, Down].
# This is a physical axis permutation, not an identity assumption.
DEFAULT_RIG_FROM_OPTICAL_RDF: Matrix3 = (
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)


@dataclass(frozen=True)
class NominalStereoExtrinsics:
    left_translation_rig_m: Vector3 = (0.0, -12.5, 0.0)
    right_translation_rig_m: Vector3 = (0.0, 12.5, 0.0)
    left_rotation_rig_from_optical: Matrix3 = DEFAULT_RIG_FROM_OPTICAL_RDF
    right_rotation_rig_from_optical: Matrix3 = DEFAULT_RIG_FROM_OPTICAL_RDF

    def validate(self) -> "NominalStereoExtrinsics":
        left_translation = as_vector3(self.left_translation_rig_m, "left_translation_rig_m")
        right_translation = as_vector3(self.right_translation_rig_m, "right_translation_rig_m")
        left_rotation = as_matrix3(self.left_rotation_rig_from_optical, "left_rotation_rig_from_optical")
        right_rotation = as_matrix3(self.right_rotation_rig_from_optical, "right_rotation_rig_from_optical")
        # derive_camera_optical_pose validates the rotations, but validating
        # here makes malformed config fail before a live evaluation.
        from .pose_math import validate_rotation_matrix

        validate_rotation_matrix(left_rotation)
        validate_rotation_matrix(right_rotation)
        return NominalStereoExtrinsics(
            left_translation, right_translation, left_rotation, right_rotation
        )

    @property
    def nominal_baseline_vector_rig_m(self) -> Vector3:
        return tuple(
            self.right_translation_rig_m[index] - self.left_translation_rig_m[index]
            for index in range(3)
        )  # type: ignore[return-value]


@dataclass(frozen=True)
class IdealStereoPoses:
    source_type: str
    target_world_enu_m: Vector3 | None
    rig_pose: Pose3D | None
    left_pose: Pose3D | None
    right_pose: Pose3D | None
    look_at_status: str
    valid: bool


@dataclass(frozen=True)
class PoseEvaluationResult:
    ideal_ground_truth: IdealStereoPoses
    ideal_rtk: IdealStereoPoses
    pose_error: dict[str, Any]
    stereo_geometry: dict[str, Any]
    status: dict[str, Any]


def _vector_dict(value: Sequence[float] | None) -> list[float] | None:
    if value is None:
        return None
    parsed = [float(component) for component in value]
    return parsed if all(math.isfinite(component) for component in parsed) else None


def _matrix_dict(value: Sequence[Sequence[float]] | None) -> list[list[float]] | None:
    return None if value is None else [[float(component) for component in row] for row in value]


def _rotation_error_dict(error: RotationError | None) -> dict[str, Any]:
    if error is None:
        return {
            "roll_error_deg": None,
            "pitch_error_deg": None,
            "yaw_error_deg": None,
            "rotation_error_deg": None,
            "relative_rotation": None,
            "valid": False,
        }
    return {
        "roll_error_deg": error.roll_error_deg,
        "pitch_error_deg": error.pitch_error_deg,
        "yaw_error_deg": error.yaw_error_deg,
        "rotation_error_deg": error.rotation_error_deg,
        "relative_rotation": _matrix_dict(error.relative_rotation),
        "valid": error.valid,
    }


def _camera_error_dict(error: CameraPoseError | None, *, reason: str | None = None) -> dict[str, Any]:
    if error is None:
        result = _rotation_error_dict(None)
        result["position_error_world_enu_m"] = None
        result["position_error_norm_m"] = None
        result["optical_axis_to_target_error_deg"] = None
        if reason:
            result["reason"] = reason
        return result
    result = _rotation_error_dict(error.rotation)
    result["position_error_world_enu_m"] = _vector_dict(error.position_error_world_enu_m)
    result["position_error_norm_m"] = error.position_error_norm_m
    result["optical_axis_to_target_error_deg"] = error.optical_axis_to_target_error_deg
    result["valid"] = error.valid
    return result


def _ideal_pose_dict(ideal: IdealStereoPoses) -> dict[str, Any]:
    def one(pose: Pose3D | None) -> dict[str, Any] | None:
        if pose is None:
            return None
        quaternion = matrix_to_quaternion_xyzw(pose.rotation_world_from_frame)
        from .pose_math import rotation_to_euler_zyx_rad

        roll, pitch, yaw = rotation_to_euler_zyx_rad(pose.rotation_world_from_frame)
        return {
            "frame_id": pose.frame_id,
            "position_world_enu_m": _vector_dict(pose.position_world_enu_m),
            "orientation_xyzw": _vector_dict(quaternion),
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "source_timestamp_ns": pose.source_timestamp_ns,
            "valid": True,
        }

    return {
        "source_type": ideal.source_type,
        "target_world_enu_m": _vector_dict(ideal.target_world_enu_m),
        "look_at_status": ideal.look_at_status,
        "rig": one(ideal.rig_pose),
        "left_optical": one(ideal.left_pose),
        "right_optical": one(ideal.right_pose),
        "valid": ideal.valid,
    }


def _invalid_baseline(reason: str) -> dict[str, Any]:
    return {
        "vector_world_enu_m": None,
        "actual_length_m": None,
        "nominal_length_m": None,
        "length_error_m": None,
        "direction_error_deg": None,
        "midpoint_world_enu_m": None,
        "midpoint_error_m": None,
        "midpoint_error_status": "not_available",
        "valid": False,
        "reason": reason,
    }


def _baseline_dict(result: BaselineGeometryResult) -> dict[str, Any]:
    return {
        "vector_world_enu_m": _vector_dict(result.vector_world_enu_m),
        "actual_length_m": result.actual_length_m,
        "nominal_length_m": result.nominal_length_m,
        "length_error_m": result.length_error_m,
        "direction_error_deg": result.direction_error_deg,
        "midpoint_world_enu_m": _vector_dict(result.midpoint_world_enu_m),
        "midpoint_error_m": result.midpoint_error_m,
        "midpoint_error_status": (
            "available" if result.midpoint_error_m is not None else "not_available"
        ),
        "valid": result.valid,
    }


def _invalid_relative(reason: str) -> dict[str, Any]:
    return {
        "relative_roll_error_deg": None,
        "relative_pitch_error_deg": None,
        "relative_yaw_error_deg": None,
        "relative_rotation_error_deg": None,
        "left_right_optical_axis_angle_deg": None,
        "valid": False,
        "reason": reason,
    }


def _relative_dict(result: StereoRelativePoseError) -> dict[str, Any]:
    return {
        "relative_roll_error_deg": result.roll_error_deg,
        "relative_pitch_error_deg": result.pitch_error_deg,
        "relative_yaw_error_deg": result.yaw_error_deg,
        "relative_rotation_error_deg": result.rotation_error_deg,
        "left_right_optical_axis_angle_deg": result.optical_axis_angle_deg,
        "valid": result.valid,
    }


def _make_ideal(
    source_type: str,
    target_world_enu_m: Sequence[float] | None,
    rig_center_world_enu_m: Vector3,
    extrinsics: NominalStereoExtrinsics,
    *,
    source_timestamp_ns: int | None,
    world_up_world_enu: Vector3,
    previous_right_axis_world_enu: Vector3 | None,
    near_vertical_threshold: float,
    fallback_horizontal_axis_world_enu: Vector3,
) -> IdealStereoPoses:
    if target_world_enu_m is None:
        return IdealStereoPoses(
            source_type,
            None,
            None,
            None,
            None,
            "INVALID_TARGET_UNAVAILABLE",
            False,
        )
    target = as_vector3(target_world_enu_m, "target_world_enu_m")
    try:
        center = as_vector3(rig_center_world_enu_m, "rig_center_world_enu_m")
    except ValueError:
        return IdealStereoPoses(
            source_type,
            target,
            None,
            None,
            None,
            "INVALID_RIG_CENTER_UNAVAILABLE",
            False,
        )
    look_at = compute_look_at_rig_rotation(
        center,
        target,
        world_up_world_enu,
        previous_right_axis_world_enu,
        near_vertical_threshold=near_vertical_threshold,
        fallback_horizontal_axis_world_enu=fallback_horizontal_axis_world_enu,
    )
    if not look_at.valid or look_at.rotation_world_from_rig is None:
        return IdealStereoPoses(
            source_type, target, None, None, None, look_at.status, False
        )
    rig_pose = Pose3D(
        center,
        look_at.rotation_world_from_rig,
        "baseline_center_frd",
        source_timestamp_ns,
    )
    left_pose = derive_camera_optical_pose(
        rig_pose.position_world_enu_m,
        rig_pose.rotation_world_from_frame,
        extrinsics.left_translation_rig_m,
        extrinsics.left_rotation_rig_from_optical,
        source_timestamp_ns=source_timestamp_ns,
    )
    right_pose = derive_camera_optical_pose(
        rig_pose.position_world_enu_m,
        rig_pose.rotation_world_from_frame,
        extrinsics.right_translation_rig_m,
        extrinsics.right_rotation_rig_from_optical,
        source_timestamp_ns=source_timestamp_ns,
    )
    return IdealStereoPoses(source_type, target, rig_pose, left_pose, right_pose, look_at.status, True)


def evaluate_pose_snapshot(
    *,
    ground_truth_target: PoseSourceSample | None,
    left_actual: PoseSourceSample | None,
    right_actual: PoseSourceSample | None,
    rtk_target_world_enu_m: Sequence[float] | None,
    rtk_valid: bool,
    rtk_status: str,
    extrinsics: NominalStereoExtrinsics,
    evaluation_timestamp_ns: int,
    ground_truth_source: str,
    target_entity_name: str,
    rtk_source_topic: str,
    reference_origin: Wgs84ReferenceOrigin,
    gazebo_origin_in_reference_enu_m: Vector3,
    altitude_datum: str,
    target_point_alignment_mode: str = "zero_lever_arm_assumption",
    lever_arm_applied: bool = False,
    rig_center_source: str = "actual_optical_midpoint",
    fallback_used: bool = False,
    time_alignment_method: str = "latest_common_gazebo_stamp_with_no_future_extrapolation",
    time_residuals: Mapping[str, Any] | None = None,
    near_vertical_threshold: float = 1.0e-9,
    world_up_world_enu: Vector3 = (0.0, 0.0, 1.0),
    fallback_horizontal_axis_world_enu: Vector3 = (1.0, 0.0, 0.0),
) -> PoseEvaluationResult:
    """Evaluate one synchronized set of GT, camera, and optional RTK samples."""

    nominal = extrinsics.validate()
    required_samples_valid = ground_truth_target is not None and left_actual is not None and right_actual is not None
    if required_samples_valid:
        assert left_actual is not None and right_actual is not None
        rig_center = tuple(
            0.5 * (left_actual.position_world_enu_m[index] + right_actual.position_world_enu_m[index])
            for index in range(3)
        )  # type: ignore[assignment]
        rig_center_source = rig_center_source or "actual_optical_midpoint"
    else:
        rig_center = (math.nan, math.nan, math.nan)

    gt_target = (
        None if ground_truth_target is None else ground_truth_target.position_world_enu_m
    )
    ideal_gt = _make_ideal(
        "ground_truth",
        gt_target,
        rig_center,
        nominal,
        source_timestamp_ns=(
            None if ground_truth_target is None else ground_truth_target.source_timestamp_ns
        ),
        world_up_world_enu=world_up_world_enu,
        previous_right_axis_world_enu=None,
        near_vertical_threshold=near_vertical_threshold,
        fallback_horizontal_axis_world_enu=fallback_horizontal_axis_world_enu,
    )
    ideal_rtk = _make_ideal(
        "rtk",
        rtk_target_world_enu_m if rtk_valid else None,
        rig_center,
        nominal,
        source_timestamp_ns=None,
        world_up_world_enu=world_up_world_enu,
        previous_right_axis_world_enu=None,
        near_vertical_threshold=near_vertical_threshold,
        fallback_horizontal_axis_world_enu=fallback_horizontal_axis_world_enu,
    )

    if left_actual is not None and ideal_gt.left_pose is not None and ideal_gt.target_world_enu_m is not None:
        left_gt_error = compute_camera_pose_error(ideal_gt.left_pose, left_actual.pose, ideal_gt.target_world_enu_m)
    else:
        left_gt_error = None
    if right_actual is not None and ideal_gt.right_pose is not None and ideal_gt.target_world_enu_m is not None:
        right_gt_error = compute_camera_pose_error(ideal_gt.right_pose, right_actual.pose, ideal_gt.target_world_enu_m)
    else:
        right_gt_error = None
    if left_actual is not None and ideal_rtk.left_pose is not None and ideal_rtk.target_world_enu_m is not None:
        left_rtk_error = compute_camera_pose_error(ideal_rtk.left_pose, left_actual.pose, ideal_rtk.target_world_enu_m)
    else:
        left_rtk_error = None
    if right_actual is not None and ideal_rtk.right_pose is not None and ideal_rtk.target_world_enu_m is not None:
        right_rtk_error = compute_camera_pose_error(ideal_rtk.right_pose, right_actual.pose, ideal_rtk.target_world_enu_m)
    else:
        right_rtk_error = None

    if ideal_gt.rig_pose is not None and ideal_rtk.rig_pose is not None:
        rtk_gt_rotation_error = compute_rotation_error(
            ideal_gt.rig_pose.rotation_world_from_frame,
            ideal_rtk.rig_pose.rotation_world_from_frame,
        )
        front_gt = matrix_vector(ideal_gt.rig_pose.rotation_world_from_frame, (1.0, 0.0, 0.0))
        front_rtk = matrix_vector(ideal_rtk.rig_pose.rotation_world_from_frame, (1.0, 0.0, 0.0))
        rtk_gt_los_error_deg = math.degrees(angle_between(front_gt, front_rtk))
    else:
        rtk_gt_rotation_error = None
        rtk_gt_los_error_deg = None

    if left_actual is not None and right_actual is not None:
        relative = compute_relative_camera_pose_error(
            left_actual.rotation_world_from_frame,
            right_actual.rotation_world_from_frame,
            nominal.left_rotation_rig_from_optical,
            nominal.right_rotation_rig_from_optical,
        )
    else:
        relative = None

    if (
        left_actual is not None
        and right_actual is not None
        and ideal_gt.rig_pose is not None
    ):
        baseline = compute_baseline_geometry(
            left_actual.position_world_enu_m,
            right_actual.position_world_enu_m,
            nominal.nominal_baseline_vector_rig_m,
            ideal_gt.rig_pose.rotation_world_from_frame,
            nominal_midpoint_world_enu_m=None,
        )
        baseline_dict = _baseline_dict(baseline)
    else:
        baseline_dict = _invalid_baseline("GT ideal rig or actual camera pair unavailable")

    pose_error = {
        "schema": "pose_simulation/pose_error/v1",
        "evaluation_timestamp_ns": int(evaluation_timestamp_ns),
        "frame_id": "gazebo_world_enu",
        "angle_unit": "deg",
        "euler_convention": "fixed_extrinsic_ZYX_RzRyRx_equivalent_intrinsic_XYZ",
        "quaternion_order": "xyzw",
        "left_actual_vs_gt_ideal": {
            **_camera_error_dict(left_gt_error, reason="GT ideal or actual left pose unavailable"),
            "optical_axis_to_gt_target_error_deg": (
                None if left_gt_error is None else left_gt_error.optical_axis_to_target_error_deg
            ),
        },
        "right_actual_vs_gt_ideal": {
            **_camera_error_dict(right_gt_error, reason="GT ideal or actual right pose unavailable"),
            "optical_axis_to_gt_target_error_deg": (
                None if right_gt_error is None else right_gt_error.optical_axis_to_target_error_deg
            ),
        },
        "left_actual_vs_rtk_ideal": {
            **_camera_error_dict(left_rtk_error, reason="RTK ideal or actual left pose unavailable"),
            "optical_axis_to_rtk_target_error_deg": (
                None if left_rtk_error is None else left_rtk_error.optical_axis_to_target_error_deg
            ),
        },
        "right_actual_vs_rtk_ideal": {
            **_camera_error_dict(right_rtk_error, reason="RTK ideal or actual right pose unavailable"),
            "optical_axis_to_rtk_target_error_deg": (
                None if right_rtk_error is None else right_rtk_error.optical_axis_to_target_error_deg
            ),
        },
        "rtk_ideal_vs_gt_ideal": {
            **_rotation_error_dict(rtk_gt_rotation_error),
            "los_direction_error_deg": rtk_gt_los_error_deg,
            "valid": rtk_gt_rotation_error is not None and rtk_gt_los_error_deg is not None,
        },
        "ideal_pose": {
            "ground_truth": _ideal_pose_dict(ideal_gt),
            "rtk": _ideal_pose_dict(ideal_rtk),
        },
    }
    stereo_geometry = {
        "schema": "pose_simulation/stereo_geometry/v1",
        "evaluation_timestamp_ns": int(evaluation_timestamp_ns),
        "frame_id": "gazebo_world_enu",
        **(_relative_dict(relative) if relative is not None else _invalid_relative("actual camera pair unavailable")),
        "baseline": baseline_dict,
        "relative_yaw_error_deg": None if relative is None else relative.yaw_error_deg,
    }
    status = {
        "schema": "pose_simulation/status/v1",
        "evaluation_timestamp_ns": int(evaluation_timestamp_ns),
        "ground_truth_source": ground_truth_source,
        "target_entity_name": target_entity_name,
        "target_gt_timestamp_ns": (
            None if ground_truth_target is None else ground_truth_target.source_timestamp_ns
        ),
        "left_camera_pose_timestamp_ns": (
            None if left_actual is None else left_actual.source_timestamp_ns
        ),
        "right_camera_pose_timestamp_ns": (
            None if right_actual is None else right_actual.source_timestamp_ns
        ),
        "target_gt_receive_timestamp_ns": (
            None if ground_truth_target is None else ground_truth_target.receive_timestamp_ns
        ),
        "target_gt_orientation_xyzw": (
            None
            if ground_truth_target is None
            else _vector_dict(matrix_to_quaternion_xyzw(ground_truth_target.rotation_world_from_frame))
        ),
        "left_camera_pose_receive_timestamp_ns": (
            None if left_actual is None else left_actual.receive_timestamp_ns
        ),
        "right_camera_pose_receive_timestamp_ns": (
            None if right_actual is None else right_actual.receive_timestamp_ns
        ),
        "left_optical_entity_name": None if left_actual is None else left_actual.entity_name,
        "right_optical_entity_name": None if right_actual is None else right_actual.entity_name,
        "rig_center_source": rig_center_source,
        "rtk_source_topic": rtk_source_topic,
        "rtk_status": rtk_status,
        "rtk_fix_state": rtk_status,
        "rtk_reference_valid_for_operational_evaluation": bool(rtk_valid),
        "reference_origin_wgs84": {
            "latitude_deg": reference_origin.latitude_deg,
            "longitude_deg": reference_origin.longitude_deg,
            "altitude_m": reference_origin.altitude_m,
            "altitude_datum": reference_origin.altitude_datum,
        },
        "altitude_datum": altitude_datum,
        "coordinate_frames": {
            "world": "gazebo_world_enu",
            "px4_local": "px4_local_ned",
            "rig": "baseline_center_frd",
            "camera_optical": "camera_optical_rdf",
        },
        "units": {"position": "m", "angle": "deg", "timestamp": "ns"},
        "world_up_world_enu": _vector_dict(world_up_world_enu),
        "near_vertical_threshold": near_vertical_threshold,
        "rig_center_world_enu_m": _vector_dict(rig_center),
        "target_point_alignment_mode": target_point_alignment_mode,
        "lever_arm_applied": bool(lever_arm_applied),
        "time_alignment_method": time_alignment_method,
        "time_residuals": dict(time_residuals or {}),
        "fallback_used": bool(fallback_used),
        "quaternion_order": "xyzw",
        "euler_convention": "fixed_extrinsic_ZYX_RzRyRx_equivalent_intrinsic_XYZ",
        "validity": {
            "required_pose_inputs": required_samples_valid,
            "ground_truth_ideal": ideal_gt.valid,
            "rtk_ideal": ideal_rtk.valid,
            "actual_vs_gt": left_gt_error is not None and right_gt_error is not None and left_gt_error.valid and right_gt_error.valid,
            "actual_vs_rtk": left_rtk_error is not None and right_rtk_error is not None and left_rtk_error.valid and right_rtk_error.valid,
            "stereo_relative": relative is not None and relative.valid,
            "baseline": baseline_dict["valid"],
        },
    }
    return PoseEvaluationResult(ideal_gt, ideal_rtk, pose_error, stereo_geometry, status)


__all__ = [
    "DEFAULT_RIG_FROM_OPTICAL_RDF",
    "IdealStereoPoses",
    "NominalStereoExtrinsics",
    "PoseEvaluationResult",
    "evaluate_pose_snapshot",
]
