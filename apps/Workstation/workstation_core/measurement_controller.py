from __future__ import annotations

from dataclasses import dataclass

from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from Workstation.workstation_core.stereo import (
    StereoEstimate,
    StereoMatch,
    correspondence_rejection_reason,
    match_stereo_observations,
    stereo_estimate_from_match,
)
from Workstation.workstation_core.triangulation import (
    RayTriangulationAttempt,
    RayTriangulationResult,
    invert_left_to_right_camera_transform,
    triangulate_pixels_with_relative_pose_attempt,
    triangulate_pixels_with_pixhawk_mount_attempt,
)


@dataclass(frozen=True)
class MeasurementGeometryResult:
    match: StereoMatch | None
    triangulated: RayTriangulationResult | None
    estimate: StereoEstimate | None
    warnings: tuple[str, ...]
    triangulation_reason: str = "not_attempted"


def calculate_measurement_geometry(inputs: MeasurementAttempt) -> MeasurementGeometryResult:
    correspondence_reason = correspondence_rejection_reason(
        inputs.left_detection,
        inputs.right_detection,
    )
    match = match_stereo_observations(
        inputs.left_detection,
        inputs.right_detection,
        inputs.stereo_calibration(),
        left_extrinsic=inputs.left_attitude,
        right_extrinsic=inputs.right_attitude,
        left_attitude_age_s=inputs.left_attitude_age_s,
        right_attitude_age_s=inputs.right_attitude_age_s,
        attitude_pair_delta_s=inputs.attitude_pair_delta_s,
        relative_roll_deg=inputs.relative_roll_deg,
        relative_pitch_deg=inputs.relative_pitch_deg,
        relative_yaw_deg=inputs.relative_yaw_deg,
        sync_quality=inputs.source_quality,
        warnings=tuple(inputs.warnings),
    )
    if match is None:
        return MeasurementGeometryResult(
            match=None,
            triangulated=None,
            estimate=None,
            warnings=tuple(inputs.warnings),
            triangulation_reason=correspondence_reason or "correspondence_rejected",
        )
    triangulation = _triangulate(inputs, tuple(inputs.warnings))
    estimate = None
    if triangulation.result is not None:
        estimate = stereo_estimate_from_match(match, distance_m=triangulation.result.range_m)
    return MeasurementGeometryResult(
        match=match,
        triangulated=triangulation.result,
        estimate=estimate,
        warnings=triangulation.warnings,
        triangulation_reason=triangulation.reason,
    )


@dataclass(frozen=True)
class _TriangulationAttempt:
    result: RayTriangulationResult | None
    warnings: tuple[str, ...]
    reason: str


def _with_warning(warnings: tuple[str, ...], message: str) -> tuple[str, ...]:
    return tuple(dict.fromkeys((*warnings, message)))


def _triangulate(inputs: MeasurementAttempt, warnings: tuple[str, ...]) -> _TriangulationAttempt:
    if inputs.intrinsics is None:
        return _TriangulationAttempt(
            None,
            _with_warning(warnings, "triangulation waiting for intrinsics"),
            "missing_intrinsics",
        )
    grid_mismatches = _pixel_grid_mismatches(inputs)
    if grid_mismatches:
        mismatch_warnings = warnings
        for _camera, message in grid_mismatches:
            mismatch_warnings = _with_warning(mismatch_warnings, message)
        mismatched_cameras = tuple(camera for camera, _message in grid_mismatches)
        if mismatched_cameras == ("left",):
            reason = "left_pixel_grid_mismatch"
        elif mismatched_cameras == ("right",):
            reason = "right_pixel_grid_mismatch"
        else:
            reason = "stereo_pixel_grid_mismatch"
        return _TriangulationAttempt(None, mismatch_warnings, reason)
    if inputs.triangulation_source == "checkerboard":
        if inputs.intrinsics.extrinsics is None:
            return _TriangulationAttempt(
                None,
                _with_warning(warnings, "triangulation waiting for checkerboard stereo extrinsics"),
                "missing_checkerboard_extrinsics",
            )
    elif inputs.left_attitude is None or inputs.right_attitude is None:
        return _TriangulationAttempt(
            None,
            _with_warning(warnings, "triangulation waiting for Pixhawk R"),
            "missing_pixhawk_rotation",
        )
    elif inputs.translation_right_in_left_m is None:
        return _TriangulationAttempt(
            None,
            _with_warning(warnings, "triangulation waiting for RTK T"),
            "missing_rtk_translation",
        )

    try:
        ray_attempt = _triangulate_pixels_attempt(inputs, inputs.left.center, inputs.right.center)
    except Exception as exc:
        return _TriangulationAttempt(
            None,
            _with_warning(warnings, f"triangulation error: {exc}"),
            "geometry_error",
        )
    if ray_attempt.result is None:
        return _TriangulationAttempt(
            None,
            _with_warning(warnings, f"two-ray geometry rejected: {ray_attempt.reason}"),
            ray_attempt.reason,
        )
    return _TriangulationAttempt(ray_attempt.result, warnings, ray_attempt.reason)


def _pixel_grid_mismatches(inputs: MeasurementAttempt) -> tuple[tuple[str, str], ...]:
    """Reject bbox pixels that are not on the image grid used by each camera K."""

    if inputs.intrinsics is None:
        return ()
    mismatches: list[tuple[str, str]] = []
    for camera, observation, intrinsics in (
        ("left", inputs.left_detection, inputs.intrinsics.left),
        ("right", inputs.right_detection, inputs.intrinsics.right),
    ):
        observed_grid = (int(observation.width), int(observation.height))
        calibrated_grid = (int(intrinsics.width), int(intrinsics.height))
        if observed_grid == calibrated_grid:
            continue
        mismatches.append(
            (
                camera,
                f"{camera} detection pixel grid {observed_grid[0]}x{observed_grid[1]} "
                f"does not match {camera} intrinsics grid {calibrated_grid[0]}x{calibrated_grid[1]}",
            )
        )
    return tuple(mismatches)


def _triangulate_pixels_attempt(
    inputs: MeasurementAttempt,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
) -> RayTriangulationAttempt:
    if inputs.intrinsics is None:
        return RayTriangulationAttempt(None, "missing_intrinsics")
    if inputs.triangulation_source == "checkerboard":
        extrinsics = inputs.intrinsics.extrinsics
        if extrinsics is None:
            return RayTriangulationAttempt(None, "missing_checkerboard_extrinsics")
        rotation_right_to_left, translation_right_in_left_m = invert_left_to_right_camera_transform(
            extrinsics.rotation_left_to_right,
            extrinsics.translation_left_to_right_m,
        )
        return triangulate_pixels_with_relative_pose_attempt(
            left_pixel=left_pixel,
            right_pixel=right_pixel,
            left_intrinsics=inputs.intrinsics.left,
            right_intrinsics=inputs.intrinsics.right,
            rotation_right_to_left=rotation_right_to_left,
            translation_right_in_left_m=translation_right_in_left_m,
        )
    if inputs.left_attitude is None or inputs.right_attitude is None:
        return RayTriangulationAttempt(None, "missing_pixhawk_rotation")
    if inputs.translation_right_in_left_m is None:
        return RayTriangulationAttempt(None, "missing_rtk_translation")
    return triangulate_pixels_with_pixhawk_mount_attempt(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=inputs.intrinsics.left,
        right_intrinsics=inputs.intrinsics.right,
        left_body_to_world_q=inputs.left_attitude.quaternion_wxyz,
        right_body_to_world_q=inputs.right_attitude.quaternion_wxyz,
        translation_right_in_left_m=inputs.translation_right_in_left_m,
    )
