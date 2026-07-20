from __future__ import annotations

from dataclasses import dataclass

from wiris_stereo_distance.calibration import StereoIntrinsicsResult
from wiris_stereo_distance.stereo import (
    DetectionObservation,
    StereoCalibration,
    StereoEstimate,
    StereoMatch,
    match_stereo_observations,
    stereo_estimate_from_match,
)
from wiris_stereo_distance.triangulation import (
    RayTriangulationAttempt,
    RayTriangulationResult,
    invert_left_to_right_camera_transform,
    triangulate_pixels_with_relative_pose_attempt,
    triangulate_pixels_with_pixhawk_mount_attempt,
)


@dataclass(frozen=True)
class MeasurementGeometryInput:
    left: DetectionObservation
    right: DetectionObservation
    calibration: StereoCalibration
    intrinsics: StereoIntrinsicsResult | None
    triangulation_source: str
    left_extrinsic: object | None = None
    right_extrinsic: object | None = None
    left_attitude_age_s: float | None = None
    right_attitude_age_s: float | None = None
    attitude_pair_delta_s: float | None = None
    relative_roll_deg: float | None = None
    relative_pitch_deg: float | None = None
    relative_yaw_deg: float | None = None
    sync_quality: str = "unknown"
    warnings: tuple[str, ...] = ()
    translation_right_in_left_m: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class MeasurementGeometryResult:
    match: StereoMatch | None
    triangulated: RayTriangulationResult | None
    estimate: StereoEstimate | None
    warnings: tuple[str, ...]
    triangulation_reason: str = "not_attempted"


def calculate_measurement_geometry(inputs: MeasurementGeometryInput) -> MeasurementGeometryResult:
    attempt = _triangulate(inputs, tuple(inputs.warnings))
    match = match_stereo_observations(
        inputs.left,
        inputs.right,
        inputs.calibration,
        left_extrinsic=inputs.left_extrinsic,
        right_extrinsic=inputs.right_extrinsic,
        left_attitude_age_s=inputs.left_attitude_age_s,
        right_attitude_age_s=inputs.right_attitude_age_s,
        attitude_pair_delta_s=inputs.attitude_pair_delta_s,
        relative_roll_deg=inputs.relative_roll_deg,
        relative_pitch_deg=inputs.relative_pitch_deg,
        relative_yaw_deg=inputs.relative_yaw_deg,
        sync_quality=inputs.sync_quality,
        warnings=attempt.warnings,
    )
    estimate = None
    if match is not None and attempt.result is not None:
        estimate = stereo_estimate_from_match(match, distance_m=attempt.result.range_m)
    return MeasurementGeometryResult(
        match=match,
        triangulated=attempt.result,
        estimate=estimate,
        warnings=attempt.warnings,
        triangulation_reason=attempt.reason,
    )


@dataclass(frozen=True)
class _TriangulationAttempt:
    result: RayTriangulationResult | None
    warnings: tuple[str, ...]
    reason: str


def _with_warning(warnings: tuple[str, ...], message: str) -> tuple[str, ...]:
    return tuple(dict.fromkeys((*warnings, message)))


def _triangulate(inputs: MeasurementGeometryInput, warnings: tuple[str, ...]) -> _TriangulationAttempt:
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
    elif inputs.left_extrinsic is None or inputs.right_extrinsic is None:
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


def _pixel_grid_mismatches(inputs: MeasurementGeometryInput) -> tuple[tuple[str, str], ...]:
    """Reject bbox pixels that are not on the image grid used by each camera K."""

    if inputs.intrinsics is None:
        return ()
    mismatches: list[tuple[str, str]] = []
    for camera, observation, intrinsics in (
        ("left", inputs.left, inputs.intrinsics.left),
        ("right", inputs.right, inputs.intrinsics.right),
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
    inputs: MeasurementGeometryInput,
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
    if inputs.left_extrinsic is None or inputs.right_extrinsic is None:
        return RayTriangulationAttempt(None, "missing_pixhawk_rotation")
    if inputs.translation_right_in_left_m is None:
        return RayTriangulationAttempt(None, "missing_rtk_translation")
    return triangulate_pixels_with_pixhawk_mount_attempt(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=inputs.intrinsics.left,
        right_intrinsics=inputs.intrinsics.right,
        left_body_to_world_q=inputs.left_extrinsic.q,
        right_body_to_world_q=inputs.right_extrinsic.q,
        translation_right_in_left_m=inputs.translation_right_in_left_m,
    )
