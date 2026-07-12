from __future__ import annotations

"""Pure camera attitude-error transforms and Raw reconstruction math.

Phase 3A models camera attitude error as an active fixed-axis rotation expressed
in the nominal Camera Optical RDF frame. Axes are fixed nominal camera axes:
roll is +Z_C / Optical Forward, pitch is +X_C / Camera Right, and yaw is
+Y_C / Camera Down. Positive angles follow the right-hand rule.

For column vectors the user-facing application order is roll -> pitch -> yaw,
implemented as:

    R_delta_C = R_Y(yaw) @ R_X(pitch) @ R_Z(roll)

The actual Camera RDF -> baseline-center FRD rotation is:

    R_B_C_actual = R_B_C_nominal @ R_delta_C

This module does not claim equivalence with Gazebo SDF Euler semantics; that is
reserved for Phase 3C. It has no ROS, file I/O, environment, UI, or logging
dependencies.
"""

import math
from dataclasses import dataclass

import numpy as np

from stereo_ground_sim.coordinate_frames import (
    FrameId,
    FramePoint,
    RigidTransform3D,
    transform_point,
    transform_ray,
    validate_rotation_matrix,
)
from stereo_ground_sim.ideal_projection import (
    IdealStereoProjection,
    StereoProjectionStatus,
    project_point_to_camera_pixel,
)
from stereo_ground_sim.stereo_triangulation_adapter import nominal_camera_transforms
from stereo_ground_sim.triangulation_core import (
    PinholeIntrinsics,
    TriangulationResult,
    camera_ray_from_pixel,
    triangulate_rays_baseline_center_frd,
)


@dataclass(frozen=True)
class CameraAttitudeErrorDeg:
    """Camera attitude error in degrees.

    Units are degrees only. The fields are finite builtin floats after
    validation; bools, strings, NaN, and Inf are rejected.
    """

    roll_deg: float
    pitch_deg: float
    yaw_deg: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "roll_deg", _finite_float(self.roll_deg, field_name="roll_deg"))
        object.__setattr__(self, "pitch_deg", _finite_float(self.pitch_deg, field_name="pitch_deg"))
        object.__setattr__(self, "yaw_deg", _finite_float(self.yaw_deg, field_name="yaw_deg"))


@dataclass(frozen=True)
class StereoAttitudeErrorConfig:
    """Left/right camera attitude-error configuration."""

    left: CameraAttitudeErrorDeg
    right: CameraAttitudeErrorDeg

    def __post_init__(self) -> None:
        if not isinstance(self.left, CameraAttitudeErrorDeg):
            raise TypeError("left must be a CameraAttitudeErrorDeg")
        if not isinstance(self.right, CameraAttitudeErrorDeg):
            raise TypeError("right must be a CameraAttitudeErrorDeg")


@dataclass(frozen=True)
class AttitudeErrorDecomposition:
    """Euler metadata for sweep grouping, not an exact SO(3) decomposition."""

    common_roll_deg: float
    common_pitch_deg: float
    common_yaw_deg: float
    relative_roll_deg: float
    relative_pitch_deg: float
    relative_yaw_deg: float

    def __post_init__(self) -> None:
        for field_name in (
            "common_roll_deg",
            "common_pitch_deg",
            "common_yaw_deg",
            "relative_roll_deg",
            "relative_pitch_deg",
            "relative_yaw_deg",
        ):
            object.__setattr__(self, field_name, _finite_float(getattr(self, field_name), field_name=field_name))


@dataclass(frozen=True)
class StereoCameraTransforms:
    """Left/right Camera RDF -> baseline-center FRD transforms.

    `baseline_m` is meters and may be zero. Transform source frames must be
    `camera_optical_rdf`, and target frames must be `baseline_center_frd`.
    """

    left: RigidTransform3D
    right: RigidTransform3D
    baseline_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.left, RigidTransform3D):
            raise TypeError("left must be a RigidTransform3D")
        if not isinstance(self.right, RigidTransform3D):
            raise TypeError("right must be a RigidTransform3D")
        _validate_camera_to_baseline_transform(self.left, field_name="left")
        _validate_camera_to_baseline_transform(self.right, field_name="right")
        baseline = _finite_float(self.baseline_m, field_name="baseline_m")
        if baseline < 0.0:
            raise ValueError("baseline_m must be non-negative")
        object.__setattr__(self, "baseline_m", baseline)


@dataclass(frozen=True)
class RawAttitudeMeasurement:
    """Raw reconstruction result in baseline-center FRD.

    `actual_projection` contains detector-noise-free pixel observations from the
    actual camera transforms. `raw_triangulation` uses nominal transforms for
    reconstruction. All point/error tuples are meters in `baseline_center_frd`.
    """

    valid: bool
    status: str
    actual_projection: IdealStereoProjection
    raw_triangulation: TriangulationResult | None
    raw_point_baseline_center_frd_m: tuple[float, float, float] | None
    raw_error_baseline_center_frd_m: tuple[float, float, float] | None
    raw_error_norm_m: float | None
    actual_transforms: StereoCameraTransforms
    decomposition: AttitudeErrorDecomposition

    def __post_init__(self) -> None:
        if not isinstance(self.actual_projection, IdealStereoProjection):
            raise TypeError("actual_projection must be an IdealStereoProjection")
        if self.raw_triangulation is not None and not isinstance(self.raw_triangulation, TriangulationResult):
            raise TypeError("raw_triangulation must be a TriangulationResult or None")
        if not isinstance(self.actual_transforms, StereoCameraTransforms):
            raise TypeError("actual_transforms must be a StereoCameraTransforms")
        if not isinstance(self.decomposition, AttitudeErrorDecomposition):
            raise TypeError("decomposition must be an AttitudeErrorDecomposition")


def _finite_float(value: object, *, field_name: str) -> float:
    if isinstance(value, (bool, np.bool_, str)):
        raise TypeError(f"{field_name} must be a finite float")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{field_name} must be a finite float") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field_name} must be finite")
    return parsed


def _validate_camera_to_baseline_transform(transform: RigidTransform3D, *, field_name: str) -> None:
    if transform.source_frame != FrameId.CAMERA_OPTICAL_RDF:
        raise ValueError(f"{field_name} source frame must be camera_optical_rdf")
    if transform.target_frame != FrameId.BASELINE_CENTER_FRD:
        raise ValueError(f"{field_name} target frame must be baseline_center_frd")


def _finite_baseline(value: object) -> float:
    baseline = _finite_float(value, field_name="baseline_m")
    if baseline < 0.0:
        raise ValueError("baseline_m must be non-negative")
    return baseline


def _tuple3(values: np.ndarray) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def rotation_from_attitude_error_deg(
    error: CameraAttitudeErrorDeg,
) -> tuple[tuple[float, float, float], ...]:
    """Return `R_delta_C = R_Y(yaw) @ R_X(pitch) @ R_Z(roll)`.

    The input is in degrees. The axes are fixed nominal Camera RDF axes:
    roll about +Z_C, pitch about +X_C, and yaw about +Y_C.
    """

    if not isinstance(error, CameraAttitudeErrorDeg):
        raise TypeError("error must be a CameraAttitudeErrorDeg")
    roll = math.radians(error.roll_deg)
    pitch = math.radians(error.pitch_deg)
    yaw = math.radians(error.yaw_deg)

    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    r_z = np.asarray(
        (
            (cos_r, -sin_r, 0.0),
            (sin_r, cos_r, 0.0),
            (0.0, 0.0, 1.0),
        ),
        dtype=float,
    )
    r_x = np.asarray(
        (
            (1.0, 0.0, 0.0),
            (0.0, cos_p, -sin_p),
            (0.0, sin_p, cos_p),
        ),
        dtype=float,
    )
    r_y = np.asarray(
        (
            (cos_y, 0.0, sin_y),
            (0.0, 1.0, 0.0),
            (-sin_y, 0.0, cos_y),
        ),
        dtype=float,
    )
    return validate_rotation_matrix(r_y @ r_x @ r_z)


def camera_transform_with_fixed_center(
    nominal_transform: RigidTransform3D,
    error: CameraAttitudeErrorDeg,
) -> RigidTransform3D:
    """Apply attitude error while preserving the nominal optical center.

    The returned transform is Camera RDF -> baseline-center FRD. It uses
    `R_B_C_actual = R_B_C_nominal @ R_delta_C` and recomputes
    `origin_target_in_source_m` as `o_C_B_actual = -R_B_C_actual.T @ c_B`.
    """

    if not isinstance(nominal_transform, RigidTransform3D):
        raise TypeError("nominal_transform must be a RigidTransform3D")
    _validate_camera_to_baseline_transform(nominal_transform, field_name="nominal_transform")
    if not isinstance(error, CameraAttitudeErrorDeg):
        raise TypeError("error must be a CameraAttitudeErrorDeg")

    nominal_rotation = np.asarray(nominal_transform.rotation_target_source, dtype=float)
    delta_rotation = np.asarray(rotation_from_attitude_error_deg(error), dtype=float)
    actual_rotation = nominal_rotation @ delta_rotation

    camera_center = transform_point(
        FramePoint((0.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF),
        nominal_transform,
    )
    center_b = np.asarray(camera_center.xyz_m, dtype=float)
    actual_origin = -(actual_rotation.T @ center_b)

    return RigidTransform3D(
        source_frame=FrameId.CAMERA_OPTICAL_RDF,
        target_frame=FrameId.BASELINE_CENTER_FRD,
        rotation_target_source=validate_rotation_matrix(actual_rotation),
        origin_target_in_source_m=_tuple3(actual_origin),
    )


def stereo_actual_camera_transforms(
    baseline_m: float,
    config: StereoAttitudeErrorConfig,
) -> StereoCameraTransforms:
    """Return left/right actual camera transforms for the given baseline."""

    baseline = _finite_baseline(baseline_m)
    if not isinstance(config, StereoAttitudeErrorConfig):
        raise TypeError("config must be a StereoAttitudeErrorConfig")
    left_nominal, right_nominal = nominal_camera_transforms(baseline_m=baseline)
    return StereoCameraTransforms(
        left=camera_transform_with_fixed_center(left_nominal, config.left),
        right=camera_transform_with_fixed_center(right_nominal, config.right),
        baseline_m=baseline,
    )


def decompose_common_relative_attitude(
    config: StereoAttitudeErrorConfig,
) -> AttitudeErrorDecomposition:
    """Return arithmetic Euler metadata for common/relative sweep grouping."""

    if not isinstance(config, StereoAttitudeErrorConfig):
        raise TypeError("config must be a StereoAttitudeErrorConfig")
    return AttitudeErrorDecomposition(
        common_roll_deg=0.5 * (config.left.roll_deg + config.right.roll_deg),
        common_pitch_deg=0.5 * (config.left.pitch_deg + config.right.pitch_deg),
        common_yaw_deg=0.5 * (config.left.yaw_deg + config.right.yaw_deg),
        relative_roll_deg=config.right.roll_deg - config.left.roll_deg,
        relative_pitch_deg=config.right.pitch_deg - config.left.pitch_deg,
        relative_yaw_deg=config.right.yaw_deg - config.left.yaw_deg,
    )


def project_stereo_with_actual_transforms(
    point_baseline_center_frd: FramePoint,
    transforms: StereoCameraTransforms,
    intrinsics: PinholeIntrinsics,
    *,
    clip_near_m: float | None = None,
    clip_far_m: float | None = None,
) -> IdealStereoProjection:
    """Project a baseline-center FRD point with actual camera transforms."""

    if not isinstance(transforms, StereoCameraTransforms):
        raise TypeError("transforms must be a StereoCameraTransforms")
    if not isinstance(intrinsics, PinholeIntrinsics):
        raise TypeError("intrinsics must be a PinholeIntrinsics")
    left = project_point_to_camera_pixel(
        point_baseline_center_frd,
        transforms.left,
        intrinsics,
        clip_near_m=clip_near_m,
        clip_far_m=clip_far_m,
    )
    right = project_point_to_camera_pixel(
        point_baseline_center_frd,
        transforms.right,
        intrinsics,
        clip_near_m=clip_near_m,
        clip_far_m=clip_far_m,
    )

    if left.valid and right.valid:
        status = StereoProjectionStatus.OK
    elif not left.valid and not right.valid:
        status = StereoProjectionStatus.BOTH_INVALID
    elif not left.valid:
        status = StereoProjectionStatus.LEFT_INVALID
    else:
        status = StereoProjectionStatus.RIGHT_INVALID

    signed_disparity_px = None
    vertical_disparity_px = None
    if left.u_px is not None and left.v_px is not None and right.u_px is not None and right.v_px is not None:
        signed_disparity_px = float(left.u_px - right.u_px)
        vertical_disparity_px = float(left.v_px - right.v_px)

    return IdealStereoProjection(
        valid=status == StereoProjectionStatus.OK,
        status=status,
        source_frame=FrameId.BASELINE_CENTER_FRD,
        left=left,
        right=right,
        signed_disparity_px=signed_disparity_px,
        vertical_disparity_px=vertical_disparity_px,
    )


def reconstruct_raw_with_nominal_transforms(
    actual_projection: IdealStereoProjection,
    intrinsics: PinholeIntrinsics,
    baseline_m: float,
    *,
    min_intersection_angle_rad: float | None = None,
) -> TriangulationResult:
    """Reconstruct actual pixels using nominal camera transforms."""

    if not isinstance(actual_projection, IdealStereoProjection):
        raise TypeError("actual_projection must be an IdealStereoProjection")
    if not isinstance(intrinsics, PinholeIntrinsics):
        raise TypeError("intrinsics must be a PinholeIntrinsics")
    baseline = _finite_baseline(baseline_m)
    if not actual_projection.valid:
        raise ValueError("actual_projection must be valid")
    if actual_projection.left.u_px is None or actual_projection.left.v_px is None:
        raise ValueError("left projection pixels must be present")
    if actual_projection.right.u_px is None or actual_projection.right.v_px is None:
        raise ValueError("right projection pixels must be present")

    left_nominal, right_nominal = nominal_camera_transforms(baseline_m=baseline)
    left_ray = transform_ray(
        camera_ray_from_pixel(actual_projection.left.u_px, actual_projection.left.v_px, intrinsics),
        left_nominal,
    )
    right_ray = transform_ray(
        camera_ray_from_pixel(actual_projection.right.u_px, actual_projection.right.v_px, intrinsics),
        right_nominal,
    )
    return triangulate_rays_baseline_center_frd(
        left_ray,
        right_ray,
        min_intersection_angle_rad=min_intersection_angle_rad,
    )


def evaluate_raw_attitude_measurement(
    point_baseline_center_frd: FramePoint,
    intrinsics: PinholeIntrinsics,
    baseline_m: float,
    config: StereoAttitudeErrorConfig,
    *,
    clip_near_m: float | None = None,
    clip_far_m: float | None = None,
    min_intersection_angle_rad: float | None = None,
) -> RawAttitudeMeasurement:
    """Evaluate Actual projection and Nominal Raw reconstruction error."""

    if not isinstance(point_baseline_center_frd, FramePoint):
        raise TypeError("point_baseline_center_frd must be a FramePoint")
    if point_baseline_center_frd.frame != FrameId.BASELINE_CENTER_FRD:
        raise ValueError("point_baseline_center_frd frame must be baseline_center_frd")
    if not isinstance(config, StereoAttitudeErrorConfig):
        raise TypeError("config must be a StereoAttitudeErrorConfig")

    baseline = _finite_baseline(baseline_m)
    actual_transforms = stereo_actual_camera_transforms(baseline, config)
    decomposition = decompose_common_relative_attitude(config)
    actual_projection = project_stereo_with_actual_transforms(
        point_baseline_center_frd,
        actual_transforms,
        intrinsics,
        clip_near_m=clip_near_m,
        clip_far_m=clip_far_m,
    )

    if not actual_projection.valid:
        return RawAttitudeMeasurement(
            valid=False,
            status=f"projection_invalid:{_projection_invalid_suffix(actual_projection.status)}",
            actual_projection=actual_projection,
            raw_triangulation=None,
            raw_point_baseline_center_frd_m=None,
            raw_error_baseline_center_frd_m=None,
            raw_error_norm_m=None,
            actual_transforms=actual_transforms,
            decomposition=decomposition,
        )

    triangulation = reconstruct_raw_with_nominal_transforms(
        actual_projection,
        intrinsics,
        baseline,
        min_intersection_angle_rad=min_intersection_angle_rad,
    )
    if not triangulation.valid:
        return RawAttitudeMeasurement(
            valid=False,
            status=f"raw_triangulation_invalid:{triangulation.status.value}",
            actual_projection=actual_projection,
            raw_triangulation=triangulation,
            raw_point_baseline_center_frd_m=None,
            raw_error_baseline_center_frd_m=None,
            raw_error_norm_m=None,
            actual_transforms=actual_transforms,
            decomposition=decomposition,
        )

    assert triangulation.point_m is not None
    raw_point = tuple(float(value) for value in triangulation.point_m)
    raw_error_arr = np.asarray(raw_point, dtype=float) - np.asarray(point_baseline_center_frd.xyz_m, dtype=float)
    raw_error = _tuple3(raw_error_arr)
    return RawAttitudeMeasurement(
        valid=True,
        status="ok",
        actual_projection=actual_projection,
        raw_triangulation=triangulation,
        raw_point_baseline_center_frd_m=raw_point,
        raw_error_baseline_center_frd_m=raw_error,
        raw_error_norm_m=float(np.linalg.norm(raw_error_arr)),
        actual_transforms=actual_transforms,
        decomposition=decomposition,
    )


def _projection_invalid_suffix(status: StereoProjectionStatus) -> str:
    if status == StereoProjectionStatus.LEFT_INVALID:
        return "left"
    if status == StereoProjectionStatus.RIGHT_INVALID:
        return "right"
    if status == StereoProjectionStatus.BOTH_INVALID:
        return "both"
    return status.value


__all__ = [
    "AttitudeErrorDecomposition",
    "CameraAttitudeErrorDeg",
    "RawAttitudeMeasurement",
    "StereoAttitudeErrorConfig",
    "StereoCameraTransforms",
    "camera_transform_with_fixed_center",
    "decompose_common_relative_attitude",
    "evaluate_raw_attitude_measurement",
    "project_stereo_with_actual_transforms",
    "reconstruct_raw_with_nominal_transforms",
    "rotation_from_attitude_error_deg",
    "stereo_actual_camera_transforms",
]
