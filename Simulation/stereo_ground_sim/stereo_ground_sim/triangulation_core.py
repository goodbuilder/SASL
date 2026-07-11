from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import numpy as np

from stereo_ground_sim.coordinate_frames import (
    FrameDirection,
    FrameId,
    FramePoint,
    Ray3D,
)


ZERO_BASELINE_TOLERANCE_M = 1.0e-12
PARALLEL_DENOMINATOR_TOLERANCE = 1.0e-12


@dataclass(frozen=True)
class PinholeIntrinsics:
    """Ideal pinhole intrinsics for Phase 1A.

    Phase 1A supports ideal pinhole intrinsics only.
    Lens-distortion inversion is outside this phase.

    Pixel units are pixels. The generated camera ray uses Camera Optical RDF:
    X right, Y down, Z forward.
    """

    width_px: int
    height_px: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float

    def __post_init__(self) -> None:
        if int(self.width_px) <= 0:
            raise ValueError("width_px must be positive")
        if int(self.height_px) <= 0:
            raise ValueError("height_px must be positive")
        object.__setattr__(self, "width_px", int(self.width_px))
        object.__setattr__(self, "height_px", int(self.height_px))
        for field_name in ("fx_px", "fy_px", "cx_px", "cy_px"):
            value = float(getattr(self, field_name))
            if not math.isfinite(value):
                raise ValueError(f"{field_name} must be finite")
            if field_name in ("fx_px", "fy_px") and value <= 0.0:
                raise ValueError(f"{field_name} must be positive")
            object.__setattr__(self, field_name, value)


class TriangulationStatus(str, Enum):
    OK = "ok"
    ZERO_BASELINE = "zero_baseline"
    PARALLEL_RAYS = "parallel_rays"
    NEAR_PARALLEL_RAYS = "near_parallel_rays"
    NON_POSITIVE_DEPTH = "non_positive_depth"


@dataclass(frozen=True)
class TriangulationResult:
    """Closest-point triangulation result.

    Positions are meters in `output_frame`. Angles are radians.
    """

    valid: bool
    status: TriangulationStatus
    output_frame: FrameId
    point_m: tuple[float, float, float] | None
    closest_point_left_m: tuple[float, float, float] | None
    closest_point_right_m: tuple[float, float, float] | None
    ray_gap_m: float | None
    intersection_angle_rad: float | None
    left_ray_parameter_m: float | None
    right_ray_parameter_m: float | None
    range_m: float | None


def _finite_float(value: float, *, field_name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{field_name} must be finite")
    return parsed


def _vector3(values: Sequence[float], *, field_name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{field_name} must contain exactly three values")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{field_name} must contain only finite values")
    return arr


def _tuple3(values: Sequence[float] | np.ndarray) -> tuple[float, float, float]:
    arr = _vector3(values, field_name="vector")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def _line_gap_for_parallel(o_left: np.ndarray, d_left: np.ndarray, o_right: np.ndarray) -> float:
    offset = o_right - o_left
    projected = offset - float(np.dot(offset, d_left)) * d_left
    return float(np.linalg.norm(projected))


def _invalid_result(
    *,
    status: TriangulationStatus,
    output_frame: FrameId,
    ray_gap_m: float | None,
    intersection_angle_rad: float | None,
    closest_point_left_m: tuple[float, float, float] | None = None,
    closest_point_right_m: tuple[float, float, float] | None = None,
    left_ray_parameter_m: float | None = None,
    right_ray_parameter_m: float | None = None,
) -> TriangulationResult:
    return TriangulationResult(
        valid=False,
        status=status,
        output_frame=output_frame,
        point_m=None,
        closest_point_left_m=closest_point_left_m,
        closest_point_right_m=closest_point_right_m,
        ray_gap_m=ray_gap_m,
        intersection_angle_rad=intersection_angle_rad,
        left_ray_parameter_m=left_ray_parameter_m,
        right_ray_parameter_m=right_ray_parameter_m,
        range_m=None,
    )


def camera_ray_from_pixel(
    u_px: float,
    v_px: float,
    intrinsics: PinholeIntrinsics,
    *,
    frame: FrameId = FrameId.CAMERA_OPTICAL_RDF,
) -> Ray3D:
    """Create an ideal pinhole camera ray from floating-point pixel coordinates.

    Phase 1A supports ideal pinhole intrinsics only.
    Lens-distortion inversion is outside this phase.

    The unnormalized Camera Optical RDF direction is
    `((u-cx)/fx, (v-cy)/fy, 1.0)`, where RDF is X right, Y down, Z forward.
    The ray origin is the camera optical center `(0, 0, 0)` in meters.
    """

    u = _finite_float(u_px, field_name="u_px")
    v = _finite_float(v_px, field_name="v_px")
    frame = FrameId(frame)
    direction = (
        (u - intrinsics.cx_px) / intrinsics.fx_px,
        (v - intrinsics.cy_px) / intrinsics.fy_px,
        1.0,
    )
    return Ray3D(
        origin=FramePoint((0.0, 0.0, 0.0), frame),
        direction=FrameDirection.from_vector(direction, frame),
    )


def closest_points_between_rays(
    left_ray: Ray3D,
    right_ray: Ray3D,
    *,
    zero_baseline_tolerance_m: float = ZERO_BASELINE_TOLERANCE_M,
    parallel_denominator_tolerance: float = PARALLEL_DENOMINATOR_TOLERANCE,
    min_intersection_angle_rad: float | None = None,
) -> TriangulationResult:
    """Compute closest points between two same-frame rays.

    Ray equations are `L(s) = o_left + s*d_left` and
    `R(t) = o_right + t*d_right`. Direction vectors are unitless unit vectors;
    origins, parameters, closest points, and output positions are meters.

    Invalid programmer input raises `ValueError`. Geometrically invalid
    measurements return `valid=False` with a `TriangulationStatus`.
    """

    if left_ray.frame != right_ray.frame:
        raise ValueError("left and right rays must be in the same frame")
    if zero_baseline_tolerance_m < 0.0:
        raise ValueError("zero_baseline_tolerance_m must be non-negative")
    if parallel_denominator_tolerance < 0.0:
        raise ValueError("parallel_denominator_tolerance must be non-negative")
    if min_intersection_angle_rad is not None:
        min_intersection_angle_rad = _finite_float(
            min_intersection_angle_rad,
            field_name="min_intersection_angle_rad",
        )
        if min_intersection_angle_rad < 0.0:
            raise ValueError("min_intersection_angle_rad must be non-negative")

    output_frame = left_ray.frame
    o_left = _vector3(left_ray.origin.xyz_m, field_name="left origin")
    o_right = _vector3(right_ray.origin.xyz_m, field_name="right origin")
    d_left = _vector3(left_ray.direction.xyz_unit, field_name="left direction")
    d_right = _vector3(right_ray.direction.xyz_unit, field_name="right direction")

    baseline = float(np.linalg.norm(o_right - o_left))
    dot = _clamp(float(np.dot(d_left, d_right)), -1.0, 1.0)
    intersection_angle_rad = math.acos(dot)
    conditioning_angle = min(intersection_angle_rad, math.pi - intersection_angle_rad)
    denominator = 1.0 - dot * dot

    if baseline <= zero_baseline_tolerance_m:
        return _invalid_result(
            status=TriangulationStatus.ZERO_BASELINE,
            output_frame=output_frame,
            ray_gap_m=0.0,
            intersection_angle_rad=intersection_angle_rad,
        )

    if denominator <= parallel_denominator_tolerance:
        return _invalid_result(
            status=TriangulationStatus.PARALLEL_RAYS,
            output_frame=output_frame,
            ray_gap_m=_line_gap_for_parallel(o_left, d_left, o_right),
            intersection_angle_rad=intersection_angle_rad,
        )

    w0 = o_left - o_right
    d = float(np.dot(d_left, w0))
    e = float(np.dot(d_right, w0))
    left_parameter = (dot * e - d) / denominator
    right_parameter = (e - dot * d) / denominator
    closest_left = o_left + left_parameter * d_left
    closest_right = o_right + right_parameter * d_right
    gap = float(np.linalg.norm(closest_left - closest_right))

    if min_intersection_angle_rad is not None and conditioning_angle < min_intersection_angle_rad:
        return _invalid_result(
            status=TriangulationStatus.NEAR_PARALLEL_RAYS,
            output_frame=output_frame,
            ray_gap_m=gap,
            intersection_angle_rad=intersection_angle_rad,
            closest_point_left_m=_tuple3(closest_left),
            closest_point_right_m=_tuple3(closest_right),
            left_ray_parameter_m=float(left_parameter),
            right_ray_parameter_m=float(right_parameter),
        )

    if left_parameter <= 0.0 or right_parameter <= 0.0:
        return _invalid_result(
            status=TriangulationStatus.NON_POSITIVE_DEPTH,
            output_frame=output_frame,
            ray_gap_m=gap,
            intersection_angle_rad=intersection_angle_rad,
            closest_point_left_m=_tuple3(closest_left),
            closest_point_right_m=_tuple3(closest_right),
            left_ray_parameter_m=float(left_parameter),
            right_ray_parameter_m=float(right_parameter),
        )

    midpoint = 0.5 * (closest_left + closest_right)
    return TriangulationResult(
        valid=True,
        status=TriangulationStatus.OK,
        output_frame=output_frame,
        point_m=_tuple3(midpoint),
        closest_point_left_m=_tuple3(closest_left),
        closest_point_right_m=_tuple3(closest_right),
        ray_gap_m=gap,
        intersection_angle_rad=intersection_angle_rad,
        left_ray_parameter_m=float(left_parameter),
        right_ray_parameter_m=float(right_parameter),
        range_m=float(np.linalg.norm(midpoint)),
    )


def triangulate_rays_baseline_center_frd(
    left_ray: Ray3D,
    right_ray: Ray3D,
    *,
    zero_baseline_tolerance_m: float = ZERO_BASELINE_TOLERANCE_M,
    parallel_denominator_tolerance: float = PARALLEL_DENOMINATOR_TOLERANCE,
    min_intersection_angle_rad: float | None = None,
) -> TriangulationResult:
    """Triangulate two rays already expressed in `baseline_center_frd`.

    Frame mismatch is programmer error and raises `ValueError`; it is not
    hidden as an invalid measurement result.
    """

    required = FrameId.BASELINE_CENTER_FRD
    if left_ray.frame != required or right_ray.frame != required:
        raise ValueError("triangulate_rays_baseline_center_frd requires baseline_center_frd rays")
    return closest_points_between_rays(
        left_ray,
        right_ray,
        zero_baseline_tolerance_m=zero_baseline_tolerance_m,
        parallel_denominator_tolerance=parallel_denominator_tolerance,
        min_intersection_angle_rad=min_intersection_angle_rad,
    )


__all__ = [
    "PARALLEL_DENOMINATOR_TOLERANCE",
    "PinholeIntrinsics",
    "TriangulationResult",
    "TriangulationStatus",
    "ZERO_BASELINE_TOLERANCE_M",
    "camera_ray_from_pixel",
    "closest_points_between_rays",
    "triangulate_rays_baseline_center_frd",
]
