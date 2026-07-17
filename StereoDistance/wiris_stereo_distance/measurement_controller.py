from __future__ import annotations

from dataclasses import dataclass

from wiris_stereo_distance.calibration import StereoIntrinsicsResult
from wiris_stereo_distance.reprojection_profile import OriginBBoxReprojectionProfile, ReprojectionLookupResult
from wiris_stereo_distance.stereo import (
    DetectionObservation,
    StereoCalibration,
    StereoEstimate,
    StereoMatch,
    match_stereo_observations,
    stereo_estimate_from_match,
)
from wiris_stereo_distance.triangulation import (
    RayTriangulationResult,
    invert_left_to_right_camera_transform,
    stereo_center_point_from_left_camera,
    triangulate_pixels_with_relative_pose,
    triangulate_pixels_with_pixhawk_mount,
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
    reprojection_profile: OriginBBoxReprojectionProfile | None = None


@dataclass(frozen=True)
class ReprojectionCorrectionResult:
    lookup: ReprojectionLookupResult
    triangulated: RayTriangulationResult | None
    estimate: StereoEstimate | None
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class MeasurementGeometryResult:
    match: StereoMatch | None
    triangulated: RayTriangulationResult | None
    estimate: StereoEstimate | None
    warnings: tuple[str, ...]
    reprojection: ReprojectionCorrectionResult | None = None


def calculate_measurement_geometry(inputs: MeasurementGeometryInput) -> MeasurementGeometryResult:
    warnings = tuple(inputs.warnings)
    triangulated = _triangulate(inputs, warnings)
    warnings = triangulated.warnings
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
        warnings=warnings,
    )
    if match is None or triangulated.result is None:
        return MeasurementGeometryResult(match=match, triangulated=triangulated.result, estimate=None, warnings=warnings)
    estimate = stereo_estimate_from_match(match, distance_m=triangulated.result.distance_m)
    reprojection = _apply_reprojection_profile(inputs, match, triangulated.result, warnings)
    return MeasurementGeometryResult(
        match=match,
        triangulated=triangulated.result,
        estimate=estimate,
        warnings=warnings,
        reprojection=reprojection,
    )


@dataclass(frozen=True)
class _TriangulationAttempt:
    result: RayTriangulationResult | None
    warnings: tuple[str, ...]


def _with_warning(warnings: tuple[str, ...], message: str) -> tuple[str, ...]:
    return tuple(dict.fromkeys((*warnings, message)))


def _triangulate(inputs: MeasurementGeometryInput, warnings: tuple[str, ...]) -> _TriangulationAttempt:
    if inputs.intrinsics is None:
        return _TriangulationAttempt(None, _with_warning(warnings, "triangulation waiting for intrinsics"))
    if inputs.triangulation_source == "checkerboard":
        if inputs.intrinsics.extrinsics is None:
            return _TriangulationAttempt(
                None,
                _with_warning(warnings, "triangulation waiting for checkerboard stereo extrinsics"),
            )
        try:
            return _TriangulationAttempt(_triangulate_pixels(inputs, inputs.left.center, inputs.right.center), warnings)
        except Exception as exc:
            return _TriangulationAttempt(None, _with_warning(warnings, f"triangulation error: {exc}"))
    try:
        if inputs.left_extrinsic is None or inputs.right_extrinsic is None:
            return _TriangulationAttempt(None, _with_warning(warnings, "triangulation waiting for Pixhawk R"))
        if inputs.translation_right_in_left_m is None:
            return _TriangulationAttempt(None, _with_warning(warnings, "triangulation waiting for RTK T"))
        return _TriangulationAttempt(_triangulate_pixels(inputs, inputs.left.center, inputs.right.center), warnings)
    except Exception as exc:
        return _TriangulationAttempt(None, _with_warning(warnings, f"triangulation error: {exc}"))


def _triangulate_pixels(
    inputs: MeasurementGeometryInput,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
) -> RayTriangulationResult | None:
    if inputs.intrinsics is None:
        return None
    if inputs.triangulation_source == "checkerboard":
        extrinsics = inputs.intrinsics.extrinsics
        if extrinsics is None:
            return None
        rotation_right_to_left, translation_right_in_left_m = invert_left_to_right_camera_transform(
            extrinsics.rotation_left_to_right,
            extrinsics.translation_left_to_right_m,
        )
        return triangulate_pixels_with_relative_pose(
            left_pixel=left_pixel,
            right_pixel=right_pixel,
            left_intrinsics=inputs.intrinsics.left,
            right_intrinsics=inputs.intrinsics.right,
            rotation_right_to_left=rotation_right_to_left,
            translation_right_in_left_m=translation_right_in_left_m,
        )
    if inputs.left_extrinsic is None or inputs.right_extrinsic is None:
        return None
    if inputs.translation_right_in_left_m is None:
        return None
    return triangulate_pixels_with_pixhawk_mount(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=inputs.intrinsics.left,
        right_intrinsics=inputs.intrinsics.right,
        left_body_to_world_q=inputs.left_extrinsic.q,
        right_body_to_world_q=inputs.right_extrinsic.q,
        translation_right_in_left_m=inputs.translation_right_in_left_m,
    )


def _range_for_lookup(
    triangulated: RayTriangulationResult,
    translation_right_in_left_m: tuple[float, float, float] | None,
) -> float:
    point = (
        stereo_center_point_from_left_camera(triangulated.point_left_m, translation_right_in_left_m)
        if translation_right_in_left_m is not None
        else triangulated.point_left_m
    )
    return (point[0] * point[0] + point[1] * point[1] + point[2] * point[2]) ** 0.5


def _apply_reprojection_profile(
    inputs: MeasurementGeometryInput,
    match: StereoMatch,
    triangulated: RayTriangulationResult,
    warnings: tuple[str, ...],
) -> ReprojectionCorrectionResult | None:
    profile = inputs.reprojection_profile
    if profile is None:
        return None
    raw_range_m = _range_for_lookup(triangulated, inputs.translation_right_in_left_m)
    lookup = profile.apply(
        inputs.left.center[0],
        inputs.left.center[1],
        inputs.right.center[0],
        inputs.right.center[1],
        raw_disparity_px=match.disparity_px,
        raw_range_m=raw_range_m,
    )
    corrected = _triangulate_pixels(
        inputs,
        (lookup.corrected_left_u, lookup.corrected_left_v),
        (lookup.corrected_right_u, lookup.corrected_right_v),
    )
    corrected_warnings = warnings
    if corrected is None:
        corrected_warnings = _with_warning(warnings, "reprojection corrected triangulation failed")
    corrected_estimate = None
    if corrected is not None:
        corrected_estimate = stereo_estimate_from_match(match, distance_m=corrected.distance_m)
    return ReprojectionCorrectionResult(
        lookup=lookup,
        triangulated=corrected,
        estimate=corrected_estimate,
        warnings=corrected_warnings,
    )
