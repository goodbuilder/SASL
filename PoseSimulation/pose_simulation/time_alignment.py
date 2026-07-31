"""Evaluation-time alignment for M2 input samples only."""

from __future__ import annotations

from dataclasses import replace
from typing import Sequence

from .input_types import (
    AlignedPoseInputs,
    CAMERA_OPTICAL_FRAME,
    CLOCK_DOMAIN_UNRESOLVED,
    DYNAMIC_STREAM_MODE,
    GAZEBO_SIM_CLOCK,
    GroundTruthPoseSample,
    CameraOpticalPoseSample,
    SOURCE_MODE_MISMATCH,
    STATIC_SNAPSHOT_MODE,
    TIME_ALIGNMENT_FAILED,
    VALID,
    WAITING_FOR_GT,
    WAITING_FOR_LEFT_CAMERA,
    WAITING_FOR_RIGHT_CAMERA,
    WORLD_FRAME,
)
from .pose_math import (
    Pose3D,
    interpolate_pose,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
)


def _invalid_alignment(
    *,
    status: str,
    reason: str,
    ground_truth: GroundTruthPoseSample | None = None,
    left: CameraOpticalPoseSample | None = None,
    right: CameraOpticalPoseSample | None = None,
    evaluation_stamp_ns: int | None = None,
    alignment_method: str = "unresolved",
    clock_domain: str | None = None,
) -> AlignedPoseInputs:
    return AlignedPoseInputs(
        evaluation_id=(
            None if evaluation_stamp_ns is None else f"gt:{evaluation_stamp_ns}"
        ),
        evaluation_stamp_ns=evaluation_stamp_ns,
        ground_truth=ground_truth,
        left_actual_camera=left,
        right_actual_camera=right,
        alignment_method=alignment_method,
        gt_time_residual_ns=None,
        left_time_residual_ns=None,
        right_time_residual_ns=None,
        clock_domain=clock_domain,
        valid=False,
        status=status,
        invalid_reason=reason,
    )


def align_static_snapshot(
    ground_truth: GroundTruthPoseSample | None,
    left_camera: CameraOpticalPoseSample | None,
    right_camera: CameraOpticalPoseSample | None,
    *,
    expected_clock_domain: str = GAZEBO_SIM_CLOCK,
) -> AlignedPoseInputs:
    """Use each GT source stamp as a trigger and immutable camera snapshots."""

    if ground_truth is None:
        return _invalid_alignment(status=WAITING_FOR_GT, reason="no Ground Truth sample received")
    if left_camera is None:
        return _invalid_alignment(
            status=WAITING_FOR_LEFT_CAMERA,
            reason="no left camera snapshot received",
            ground_truth=ground_truth,
            evaluation_stamp_ns=ground_truth.stamp_ns,
        )
    if right_camera is None:
        return _invalid_alignment(
            status=WAITING_FOR_RIGHT_CAMERA,
            reason="no right camera snapshot received",
            ground_truth=ground_truth,
            left=left_camera,
            evaluation_stamp_ns=ground_truth.stamp_ns,
        )
    if not ground_truth.valid:
        return _invalid_alignment(
            status=ground_truth.status,
            reason=ground_truth.invalid_reason,
            ground_truth=ground_truth,
            left=left_camera,
            right=right_camera,
            evaluation_stamp_ns=ground_truth.stamp_ns,
            alignment_method=STATIC_SNAPSHOT_MODE,
            clock_domain=ground_truth.clock_domain,
        )
    if not left_camera.valid:
        return _invalid_alignment(
            status=left_camera.status,
            reason=left_camera.invalid_reason,
            ground_truth=ground_truth,
            left=left_camera,
            right=right_camera,
            evaluation_stamp_ns=ground_truth.stamp_ns,
            alignment_method=STATIC_SNAPSHOT_MODE,
            clock_domain=left_camera.clock_domain,
        )
    if not right_camera.valid:
        return _invalid_alignment(
            status=right_camera.status,
            reason=right_camera.invalid_reason,
            ground_truth=ground_truth,
            left=left_camera,
            right=right_camera,
            evaluation_stamp_ns=ground_truth.stamp_ns,
            alignment_method=STATIC_SNAPSHOT_MODE,
            clock_domain=right_camera.clock_domain,
        )
    if ground_truth.clock_domain != expected_clock_domain:
        return _invalid_alignment(
            status=CLOCK_DOMAIN_UNRESOLVED,
            reason="Ground Truth clock domain does not match the configured source clock",
            ground_truth=ground_truth,
            left=left_camera,
            right=right_camera,
            evaluation_stamp_ns=ground_truth.stamp_ns,
            alignment_method=STATIC_SNAPSHOT_MODE,
            clock_domain=ground_truth.clock_domain,
        )
    if left_camera.clock_domain != expected_clock_domain or right_camera.clock_domain != expected_clock_domain:
        return _invalid_alignment(
            status=CLOCK_DOMAIN_UNRESOLVED,
            reason="camera source clock domain is unresolved",
            ground_truth=ground_truth,
            left=left_camera,
            right=right_camera,
            evaluation_stamp_ns=ground_truth.stamp_ns,
            alignment_method=STATIC_SNAPSHOT_MODE,
            clock_domain=expected_clock_domain,
        )
    if any(
        sample.source_mode != STATIC_SNAPSHOT_MODE
        for sample in (left_camera, right_camera)
    ):
        return _invalid_alignment(
            status=SOURCE_MODE_MISMATCH,
            reason="static alignment received a non-static camera source",
            ground_truth=ground_truth,
            left=left_camera,
            right=right_camera,
            evaluation_stamp_ns=ground_truth.stamp_ns,
            alignment_method=STATIC_SNAPSHOT_MODE,
            clock_domain=expected_clock_domain,
        )
    if ground_truth.stamp_ns is None:
        return _invalid_alignment(
            status=TIME_ALIGNMENT_FAILED,
            reason="valid Ground Truth sample has no source timestamp",
            ground_truth=ground_truth,
            left=left_camera,
            right=right_camera,
            alignment_method=STATIC_SNAPSHOT_MODE,
            clock_domain=expected_clock_domain,
        )
    return AlignedPoseInputs(
        evaluation_id=f"gt:{ground_truth.stamp_ns}",
        evaluation_stamp_ns=ground_truth.stamp_ns,
        ground_truth=ground_truth,
        left_actual_camera=left_camera,
        right_actual_camera=right_camera,
        alignment_method=STATIC_SNAPSHOT_MODE,
        gt_time_residual_ns=0,
        left_time_residual_ns=None,
        right_time_residual_ns=None,
        clock_domain=expected_clock_domain,
        valid=True,
        status=VALID,
        invalid_reason="",
    )


def _sample_stamp_ns(sample: GroundTruthPoseSample | CameraOpticalPoseSample) -> int | None:
    """Return the authoritative source stamp for either input type."""

    if isinstance(sample, GroundTruthPoseSample):
        return sample.stamp_ns
    return sample.gazebo_stamp_ns


def _sample_pose(sample: GroundTruthPoseSample | CameraOpticalPoseSample) -> Pose3D:
    if isinstance(sample, GroundTruthPoseSample):
        assert sample.position_world_enu_m is not None
        assert sample.orientation_world_from_reference_xyzw is not None
        return Pose3D(
            sample.position_world_enu_m,
            quaternion_xyzw_to_matrix(sample.orientation_world_from_reference_xyzw),
            WORLD_FRAME,
            sample.stamp_ns,
        )
    assert sample.position_world_enu_m is not None
    assert sample.orientation_world_from_optical_xyzw is not None
    return Pose3D(
        sample.position_world_enu_m,
        quaternion_xyzw_to_matrix(sample.orientation_world_from_optical_xyzw),
        CAMERA_OPTICAL_FRAME,
        sample.gazebo_stamp_ns,
    )


def _interpolate_sample(
    samples: Sequence[GroundTruthPoseSample | CameraOpticalPoseSample],
    evaluation_stamp_ns: int,
    *,
    max_gap_ns: int,
) -> tuple[GroundTruthPoseSample | CameraOpticalPoseSample | None, bool, str]:
    if not samples:
        return None, False, "sample buffer is empty"
    ordered = sorted(samples, key=lambda sample: _sample_stamp_ns(sample) or -1)
    if any(not sample.valid or _sample_stamp_ns(sample) is None for sample in ordered):
        return None, False, "sample buffer contains invalid or timestamp-less data"
    timestamps = [_sample_stamp_ns(sample) for sample in ordered]
    if any(timestamp is None for timestamp in timestamps):
        return None, False, "sample buffer contains timestamp-less data"
    if any(
        timestamps[index] <= timestamps[index - 1]
        for index in range(1, len(timestamps))
    ):
        return None, False, "sample buffer is out of order or contains duplicate stamps"
    if evaluation_stamp_ns < timestamps[0]:
        return None, False, "future extrapolation would be required"
    for sample in ordered:
        if _sample_stamp_ns(sample) == evaluation_stamp_ns:
            return sample, False, ""
    previous = None
    following = None
    for sample in ordered:
        sample_stamp_ns = _sample_stamp_ns(sample)
        assert sample_stamp_ns is not None
        if sample_stamp_ns < evaluation_stamp_ns:
            previous = sample
        elif sample_stamp_ns > evaluation_stamp_ns:
            following = sample
            break
    if previous is None or following is None:
        return None, False, "future extrapolation or missing bracket"
    previous_stamp_ns = _sample_stamp_ns(previous)
    following_stamp_ns = _sample_stamp_ns(following)
    assert previous_stamp_ns is not None and following_stamp_ns is not None
    gap_ns = following_stamp_ns - previous_stamp_ns
    if gap_ns <= 0 or gap_ns > max_gap_ns:
        return None, False, f"interpolation gap {gap_ns} ns exceeds policy"
    alpha = (evaluation_stamp_ns - previous_stamp_ns) / gap_ns
    pose = interpolate_pose(_sample_pose(previous), _sample_pose(following), alpha)
    if isinstance(previous, GroundTruthPoseSample):
        interpolated: GroundTruthPoseSample | CameraOpticalPoseSample = replace(
            previous,
            stamp_ns=evaluation_stamp_ns,
            position_world_enu_m=pose.position_world_enu_m,
            orientation_world_from_reference_xyzw=matrix_to_quaternion_xyzw(
                pose.rotation_world_from_frame
            ),
        )
    else:
        interpolated = replace(
            previous,
            gazebo_stamp_ns=evaluation_stamp_ns,
            position_world_enu_m=pose.position_world_enu_m,
            orientation_world_from_optical_xyzw=matrix_to_quaternion_xyzw(
                pose.rotation_world_from_frame
            ),
        )
    return interpolated, True, ""


def align_dynamic_streams(
    ground_truth_samples: Sequence[GroundTruthPoseSample],
    left_camera_samples: Sequence[CameraOpticalPoseSample],
    right_camera_samples: Sequence[CameraOpticalPoseSample],
    *,
    expected_clock_domain: str = GAZEBO_SIM_CLOCK,
    max_interpolation_gap_ns: int = 250_000_000,
    allow_future_extrapolation: bool = False,
) -> AlignedPoseInputs:
    """Align optional dynamic buffers without future extrapolation."""

    if allow_future_extrapolation:
        return _invalid_alignment(
            status=TIME_ALIGNMENT_FAILED,
            reason="future extrapolation is forbidden",
            alignment_method=DYNAMIC_STREAM_MODE,
        )
    if not ground_truth_samples:
        return _invalid_alignment(status=WAITING_FOR_GT, reason="no Ground Truth samples")
    if not left_camera_samples:
        return _invalid_alignment(
            status=WAITING_FOR_LEFT_CAMERA, reason="no left camera samples"
        )
    if not right_camera_samples:
        return _invalid_alignment(
            status=WAITING_FOR_RIGHT_CAMERA, reason="no right camera samples"
        )
    all_samples = [*ground_truth_samples, *left_camera_samples, *right_camera_samples]
    if any(not sample.valid for sample in all_samples):
        invalid = next(sample for sample in all_samples if not sample.valid)
        return _invalid_alignment(
            status=invalid.status,
            reason=invalid.invalid_reason,
            alignment_method=DYNAMIC_STREAM_MODE,
        )
    if any(
        getattr(sample, "clock_domain", None) != expected_clock_domain
        for sample in all_samples
    ):
        return _invalid_alignment(
            status=CLOCK_DOMAIN_UNRESOLVED,
            reason="one or more dynamic source clock domains are unresolved",
            alignment_method=DYNAMIC_STREAM_MODE,
            clock_domain=expected_clock_domain,
        )
    if any(
        getattr(sample, "source_mode", DYNAMIC_STREAM_MODE) != DYNAMIC_STREAM_MODE
        for sample in [*left_camera_samples, *right_camera_samples]
    ):
        return _invalid_alignment(
            status=SOURCE_MODE_MISMATCH,
            reason="dynamic alignment received a static camera sample",
            alignment_method=DYNAMIC_STREAM_MODE,
            clock_domain=expected_clock_domain,
        )
    if max_interpolation_gap_ns <= 0:
        return _invalid_alignment(
            status=TIME_ALIGNMENT_FAILED,
            reason="max_interpolation_gap_ns must be positive",
            alignment_method=DYNAMIC_STREAM_MODE,
            clock_domain=expected_clock_domain,
        )

    latest_stamps: list[int] = []
    for samples in (ground_truth_samples, left_camera_samples, right_camera_samples):
        source_stamps = [_sample_stamp_ns(sample) for sample in samples]
        if any(stamp is None for stamp in source_stamps):
            return _invalid_alignment(
                status=TIME_ALIGNMENT_FAILED,
                reason="one or more dynamic samples has no source timestamp",
                alignment_method=DYNAMIC_STREAM_MODE,
                clock_domain=expected_clock_domain,
            )
        latest_stamps.append(max(stamp for stamp in source_stamps if stamp is not None))
    evaluation_stamp_ns = min(latest_stamps)
    gt, gt_interpolated, reason = _interpolate_sample(
        ground_truth_samples, evaluation_stamp_ns, max_gap_ns=max_interpolation_gap_ns
    )
    if gt is None:
        return _invalid_alignment(
            status=TIME_ALIGNMENT_FAILED,
            reason=f"GT alignment failed: {reason}",
            evaluation_stamp_ns=evaluation_stamp_ns,
            alignment_method=DYNAMIC_STREAM_MODE,
            clock_domain=expected_clock_domain,
        )
    left, left_interpolated, reason = _interpolate_sample(
        left_camera_samples, evaluation_stamp_ns, max_gap_ns=max_interpolation_gap_ns
    )
    if left is None:
        return _invalid_alignment(
            status=TIME_ALIGNMENT_FAILED,
            reason=f"left camera alignment failed: {reason}",
            ground_truth=gt if isinstance(gt, GroundTruthPoseSample) else None,
            evaluation_stamp_ns=evaluation_stamp_ns,
            alignment_method=DYNAMIC_STREAM_MODE,
            clock_domain=expected_clock_domain,
        )
    right, right_interpolated, reason = _interpolate_sample(
        right_camera_samples, evaluation_stamp_ns, max_gap_ns=max_interpolation_gap_ns
    )
    if right is None:
        return _invalid_alignment(
            status=TIME_ALIGNMENT_FAILED,
            reason=f"right camera alignment failed: {reason}",
            ground_truth=gt if isinstance(gt, GroundTruthPoseSample) else None,
            left=left if isinstance(left, CameraOpticalPoseSample) else None,
            evaluation_stamp_ns=evaluation_stamp_ns,
            alignment_method=DYNAMIC_STREAM_MODE,
            clock_domain=expected_clock_domain,
        )
    if not isinstance(gt, GroundTruthPoseSample) or not isinstance(left, CameraOpticalPoseSample) or not isinstance(right, CameraOpticalPoseSample):
        return _invalid_alignment(
            status=TIME_ALIGNMENT_FAILED,
            reason="dynamic alignment returned an unexpected sample type",
            evaluation_stamp_ns=evaluation_stamp_ns,
            alignment_method=DYNAMIC_STREAM_MODE,
            clock_domain=expected_clock_domain,
        )
    method = "interpolate" if (gt_interpolated or left_interpolated or right_interpolated) else "exact"
    return AlignedPoseInputs(
        evaluation_id=f"dynamic:{evaluation_stamp_ns}",
        evaluation_stamp_ns=evaluation_stamp_ns,
        ground_truth=gt,
        left_actual_camera=left,
        right_actual_camera=right,
        alignment_method=method,
        gt_time_residual_ns=0,
        left_time_residual_ns=0,
        right_time_residual_ns=0,
        clock_domain=expected_clock_domain,
        valid=True,
        status=VALID,
        invalid_reason="",
    )


def align_pose_inputs(
    ground_truth: GroundTruthPoseSample | None,
    left_camera: CameraOpticalPoseSample | None,
    right_camera: CameraOpticalPoseSample | None,
    *,
    source_mode: str = STATIC_SNAPSHOT_MODE,
    expected_clock_domain: str = GAZEBO_SIM_CLOCK,
) -> AlignedPoseInputs:
    """Dispatch the configured input mode without calculating any pose error."""

    if source_mode == STATIC_SNAPSHOT_MODE:
        return align_static_snapshot(
            ground_truth,
            left_camera,
            right_camera,
            expected_clock_domain=expected_clock_domain,
        )
    if source_mode == DYNAMIC_STREAM_MODE:
        return _invalid_alignment(
            status=SOURCE_MODE_MISMATCH,
            reason="dynamic alignment requires explicit sample buffers; use align_dynamic_streams",
            alignment_method=DYNAMIC_STREAM_MODE,
        )
    return _invalid_alignment(
        status=SOURCE_MODE_MISMATCH,
        reason=f"unsupported source mode {source_mode!r}",
    )


__all__ = [
    "align_dynamic_streams",
    "align_pose_inputs",
    "align_static_snapshot",
]
