from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from stereo_ground_sim.coordinate_frames import (
    FrameId,
    FramePoint,
    RigidTransform3D,
    inverse_rigid_transform,
    transform_point,
)
from stereo_ground_sim.stereo_triangulation_adapter import nominal_camera_transforms
from stereo_ground_sim.triangulation_core import PinholeIntrinsics


class ProjectionStatus(str, Enum):
    OK = "ok"
    NON_POSITIVE_DEPTH = "non_positive_depth"
    OUTSIDE_CLIP_RANGE = "outside_clip_range"
    OUT_OF_BOUNDS = "out_of_bounds"
    OUTSIDE_CLIP_AND_BOUNDS = "outside_clip_and_bounds"


class StereoProjectionStatus(str, Enum):
    OK = "ok"
    LEFT_INVALID = "left_invalid"
    RIGHT_INVALID = "right_invalid"
    BOTH_INVALID = "both_invalid"


@dataclass(frozen=True)
class CameraProjectionResult:
    valid: bool
    status: ProjectionStatus
    source_frame: FrameId
    camera_frame: FrameId
    point_camera_rdf_m: tuple[float, float, float]
    depth_m: float
    u_px: float | None
    v_px: float | None
    in_bounds: bool | None
    clip_valid: bool

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", ProjectionStatus(self.status))
        object.__setattr__(self, "source_frame", FrameId(self.source_frame))
        object.__setattr__(self, "camera_frame", FrameId(self.camera_frame))


@dataclass(frozen=True)
class IdealStereoProjection:
    valid: bool
    status: StereoProjectionStatus
    source_frame: FrameId
    left: CameraProjectionResult
    right: CameraProjectionResult
    signed_disparity_px: float | None
    vertical_disparity_px: float | None

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", StereoProjectionStatus(self.status))
        object.__setattr__(self, "source_frame", FrameId(self.source_frame))


def _finite_float(value: object, *, field_name: str) -> float:
    if isinstance(value, (bool, str)):
        raise TypeError(f"{field_name} must be a finite float")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{field_name} must be a finite float") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field_name} must be finite")
    return parsed


def _clip_bounds(
    clip_near_m: float | None,
    clip_far_m: float | None,
) -> tuple[float | None, float | None]:
    if clip_near_m is None and clip_far_m is None:
        return None, None
    if clip_near_m is None or clip_far_m is None:
        raise ValueError("clip_near_m and clip_far_m must be provided together")
    near = _finite_float(clip_near_m, field_name="clip_near_m")
    far = _finite_float(clip_far_m, field_name="clip_far_m")
    if near < 0.0:
        raise ValueError("clip_near_m must be non-negative")
    if far <= near:
        raise ValueError("clip_far_m must be greater than clip_near_m")
    return near, far


def _validate_camera_projection_inputs(
    point_baseline_center_frd: FramePoint,
    camera_to_baseline_transform: RigidTransform3D,
    intrinsics: PinholeIntrinsics,
) -> None:
    if not isinstance(point_baseline_center_frd, FramePoint):
        raise TypeError("point_baseline_center_frd must be a FramePoint")
    if point_baseline_center_frd.frame != FrameId.BASELINE_CENTER_FRD:
        raise ValueError("point_baseline_center_frd frame must be baseline_center_frd")
    if not isinstance(camera_to_baseline_transform, RigidTransform3D):
        raise TypeError("camera_to_baseline_transform must be a RigidTransform3D")
    if camera_to_baseline_transform.source_frame != FrameId.CAMERA_OPTICAL_RDF:
        raise ValueError("camera_to_baseline_transform source frame must be camera_optical_rdf")
    if camera_to_baseline_transform.target_frame != FrameId.BASELINE_CENTER_FRD:
        raise ValueError("camera_to_baseline_transform target frame must be baseline_center_frd")
    if not isinstance(intrinsics, PinholeIntrinsics):
        raise TypeError("intrinsics must be a PinholeIntrinsics")


def _camera_projection_status(*, in_bounds: bool, clip_valid: bool) -> ProjectionStatus:
    if in_bounds and clip_valid:
        return ProjectionStatus.OK
    if not in_bounds and not clip_valid:
        return ProjectionStatus.OUTSIDE_CLIP_AND_BOUNDS
    if not in_bounds:
        return ProjectionStatus.OUT_OF_BOUNDS
    return ProjectionStatus.OUTSIDE_CLIP_RANGE


def project_point_to_camera_pixel(
    point_baseline_center_frd: FramePoint,
    camera_to_baseline_transform: RigidTransform3D,
    intrinsics: PinholeIntrinsics,
    *,
    clip_near_m: float | None = None,
    clip_far_m: float | None = None,
) -> CameraProjectionResult:
    """Project a baseline-center FRD point into one Camera Optical RDF image.

    `camera_to_baseline_transform` must be the Camera RDF -> baseline-center FRD
    transform for the camera being projected. The returned `depth_m` is the
    Camera RDF Z/Forward component.
    """

    _validate_camera_projection_inputs(point_baseline_center_frd, camera_to_baseline_transform, intrinsics)
    clip_near, clip_far = _clip_bounds(clip_near_m, clip_far_m)

    baseline_to_camera = inverse_rigid_transform(camera_to_baseline_transform)
    point_camera = transform_point(point_baseline_center_frd, baseline_to_camera)
    x_camera, y_camera, depth_m = point_camera.xyz_m

    if depth_m <= 0.0:
        return CameraProjectionResult(
            valid=False,
            status=ProjectionStatus.NON_POSITIVE_DEPTH,
            source_frame=FrameId.BASELINE_CENTER_FRD,
            camera_frame=FrameId.CAMERA_OPTICAL_RDF,
            point_camera_rdf_m=point_camera.xyz_m,
            depth_m=depth_m,
            u_px=None,
            v_px=None,
            in_bounds=None,
            clip_valid=False,
        )

    u_px = intrinsics.cx_px + intrinsics.fx_px * x_camera / depth_m
    v_px = intrinsics.cy_px + intrinsics.fy_px * y_camera / depth_m
    in_bounds = 0.0 <= u_px < float(intrinsics.width_px) and 0.0 <= v_px < float(intrinsics.height_px)
    clip_valid = True
    if clip_near is not None and clip_far is not None:
        clip_valid = clip_near <= depth_m <= clip_far
    status = _camera_projection_status(in_bounds=in_bounds, clip_valid=clip_valid)
    return CameraProjectionResult(
        valid=status == ProjectionStatus.OK,
        status=status,
        source_frame=FrameId.BASELINE_CENTER_FRD,
        camera_frame=FrameId.CAMERA_OPTICAL_RDF,
        point_camera_rdf_m=point_camera.xyz_m,
        depth_m=depth_m,
        u_px=float(u_px),
        v_px=float(v_px),
        in_bounds=bool(in_bounds),
        clip_valid=bool(clip_valid),
    )


def _finite_baseline(value: object) -> float:
    baseline = _finite_float(value, field_name="baseline_m")
    if baseline < 0.0:
        raise ValueError("baseline_m must be non-negative")
    return baseline


def _stereo_status(left: CameraProjectionResult, right: CameraProjectionResult) -> StereoProjectionStatus:
    if left.valid and right.valid:
        return StereoProjectionStatus.OK
    if not left.valid and not right.valid:
        return StereoProjectionStatus.BOTH_INVALID
    if not left.valid:
        return StereoProjectionStatus.LEFT_INVALID
    return StereoProjectionStatus.RIGHT_INVALID


def project_nominal_stereo_point(
    point_baseline_center_frd: FramePoint,
    intrinsics: PinholeIntrinsics,
    baseline_m: float,
    *,
    clip_near_m: float | None = None,
    clip_far_m: float | None = None,
) -> IdealStereoProjection:
    """Project a baseline-center FRD point into the nominal stereo pair."""

    baseline = _finite_baseline(baseline_m)
    left_transform, right_transform = nominal_camera_transforms(baseline_m=baseline)
    left = project_point_to_camera_pixel(
        point_baseline_center_frd,
        left_transform,
        intrinsics,
        clip_near_m=clip_near_m,
        clip_far_m=clip_far_m,
    )
    right = project_point_to_camera_pixel(
        point_baseline_center_frd,
        right_transform,
        intrinsics,
        clip_near_m=clip_near_m,
        clip_far_m=clip_far_m,
    )
    status = _stereo_status(left, right)
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


__all__ = [
    "CameraProjectionResult",
    "IdealStereoProjection",
    "ProjectionStatus",
    "StereoProjectionStatus",
    "project_nominal_stereo_point",
    "project_point_to_camera_pixel",
]
