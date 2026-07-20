from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]

CAMERA_OPTICAL_FRAME = "opencv_optical_rdf"
STEREO_CENTER_OPTICAL_FRAME = "stereo_center_opencv_optical_rdf"
BASELINE_CENTER_FRD_FRAME = "baseline_center_frd"
TRIANGULATION_METHOD = "two_ray_closest_midpoint"

IDENTITY_ROTATION: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


@dataclass(frozen=True)
class PinholeIntrinsics:
    """Pinhole matrix contract in pixels: u=fx*right/forward+cx, v=fy*down/forward+cy."""

    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float

    def __post_init__(self) -> None:
        values = (self.fx_px, self.fy_px, self.cx_px, self.cy_px)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx_px <= 0.0 or self.fy_px <= 0.0:
            raise ValueError("focal lengths must be positive")


@dataclass(frozen=True)
class RayTriangulationResult:
    """A raw ray-only result expressed at the stereo baseline center.

    ``point_center_rdf_m`` is ordered right, down, forward in metres.
    ``point_center_frd_m`` is the same vector reordered forward, right, down.
    The caller defines the center-frame orientation; StereoDistance keeps the
    left optical orientation while Simulation uses the shared nominal optical
    orientation.  Only the origin is guaranteed to be the camera-origin
    midpoint.
    Ray depths are signed distances from each camera origin along unit rays.
    """

    point_center_rdf_m: Vector3
    range_m: float
    closest_gap_m: float
    left_depth_m: float
    right_depth_m: float
    ray_angle_deg: float
    baseline_m: float
    method: str = TRIANGULATION_METHOD
    point_frame: str = STEREO_CENTER_OPTICAL_FRAME
    output_frame: str = BASELINE_CENTER_FRD_FRAME

    @property
    def point_center_frd_m(self) -> Vector3:
        return optical_rdf_to_frd(self.point_center_rdf_m)


@dataclass(frozen=True)
class RayTriangulationAttempt:
    result: RayTriangulationResult | None
    reason: str

    @property
    def valid(self) -> bool:
        return self.result is not None


def _vector3(values: Sequence[float], *, name: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    result = (float(values[0]), float(values[1]), float(values[2]))
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must be finite")
    return result


def _matrix3(values: Sequence[Sequence[float]], *, name: str) -> Matrix3:
    if len(values) != 3 or any(len(row) != 3 for row in values):
        raise ValueError(f"{name} must be a 3x3 matrix")
    return tuple(_vector3(row, name=name) for row in values)  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _add(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def _scale(vector: Vector3, scalar: float) -> Vector3:
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def normalize_vector(vector: Sequence[float]) -> Vector3:
    parsed = _vector3(vector, name="vector")
    norm = _norm(parsed)
    if norm <= 0.0:
        raise ValueError("vector must have non-zero length")
    return _scale(parsed, 1.0 / norm)


def rotate_vector(rotation_output_from_input: Sequence[Sequence[float]], vector: Sequence[float]) -> Vector3:
    rotation = _matrix3(rotation_output_from_input, name="rotation_output_from_input")
    parsed = _vector3(vector, name="vector")
    return (
        _dot(rotation[0], parsed),
        _dot(rotation[1], parsed),
        _dot(rotation[2], parsed),
    )


def transpose_rotation(rotation_output_from_input: Sequence[Sequence[float]]) -> Matrix3:
    rotation = _matrix3(rotation_output_from_input, name="rotation_output_from_input")
    return tuple(
        tuple(rotation[row][column] for row in range(3)) for column in range(3)
    )  # type: ignore[return-value]


def optical_rdf_to_frd(point_rdf: Sequence[float]) -> Vector3:
    right_m, down_m, forward_m = _vector3(point_rdf, name="point_rdf")
    return (forward_m, right_m, down_m)


def frd_to_optical_rdf(point_frd: Sequence[float]) -> Vector3:
    forward_m, right_m, down_m = _vector3(point_frd, name="point_frd")
    return (right_m, down_m, forward_m)


def ray_from_pixel(u_px: float, v_px: float, intrinsics: PinholeIntrinsics) -> Vector3:
    """Return a unit ray in the source camera's OpenCV optical RDF frame."""
    u_px = float(u_px)
    v_px = float(v_px)
    if not math.isfinite(u_px) or not math.isfinite(v_px):
        raise ValueError("pixel coordinates must be finite")
    return normalize_vector(
        (
            (u_px - intrinsics.cx_px) / intrinsics.fx_px,
            (v_px - intrinsics.cy_px) / intrinsics.fy_px,
            1.0,
        )
    )


def project_center_point_to_pixel(
    point_center_rdf_m: Sequence[float],
    *,
    camera_origin_center_rdf_m: Sequence[float],
    rotation_center_from_camera_rdf: Sequence[Sequence[float]],
    intrinsics: PinholeIntrinsics,
) -> tuple[float, float] | None:
    """Project a stereo-center RDF point with the complete camera pose and K matrix."""
    point = _vector3(point_center_rdf_m, name="point_center_rdf_m")
    origin = _vector3(camera_origin_center_rdf_m, name="camera_origin_center_rdf_m")
    camera_from_center = transpose_rotation(rotation_center_from_camera_rdf)
    right_m, down_m, forward_m = rotate_vector(camera_from_center, _subtract(point, origin))
    if forward_m <= 0.0:
        return None
    return (
        intrinsics.fx_px * right_m / forward_m + intrinsics.cx_px,
        intrinsics.fy_px * down_m / forward_m + intrinsics.cy_px,
    )


def triangulate_rays(
    *,
    left_origin_center_rdf_m: Sequence[float],
    right_origin_center_rdf_m: Sequence[float],
    left_ray_center_rdf: Sequence[float],
    right_ray_center_rdf: Sequence[float],
    parallel_epsilon: float = 1.0e-12,
) -> RayTriangulationAttempt:
    """Find the midpoint of the closest points on two 3-D camera rays.

    Inputs and the midpoint are in stereo-center OpenCV optical RDF.  The result
    exports the public measurement frame as baseline-center FRD without changing
    the underlying point.
    """
    left_origin = _vector3(left_origin_center_rdf_m, name="left_origin_center_rdf_m")
    right_origin = _vector3(right_origin_center_rdf_m, name="right_origin_center_rdf_m")
    left_ray = normalize_vector(left_ray_center_rdf)
    right_ray = normalize_vector(right_ray_center_rdf)
    baseline_vector = _subtract(right_origin, left_origin)
    baseline_m = _norm(baseline_vector)
    if baseline_m <= 0.0:
        return RayTriangulationAttempt(None, "zero_baseline")

    ray_dot = max(-1.0, min(1.0, _dot(left_ray, right_ray)))
    denominator = 1.0 - ray_dot * ray_dot
    if denominator <= float(parallel_epsilon):
        return RayTriangulationAttempt(None, "near_parallel_rays")

    origin_delta = _subtract(left_origin, right_origin)
    left_projection = _dot(left_ray, origin_delta)
    right_projection = _dot(right_ray, origin_delta)
    left_depth_m = (ray_dot * right_projection - left_projection) / denominator
    right_depth_m = (right_projection - ray_dot * left_projection) / denominator
    if left_depth_m <= 0.0 or right_depth_m <= 0.0:
        return RayTriangulationAttempt(None, "non_positive_depth")

    left_closest = _add(left_origin, _scale(left_ray, left_depth_m))
    right_closest = _add(right_origin, _scale(right_ray, right_depth_m))
    midpoint = _scale(_add(left_closest, right_closest), 0.5)
    range_m = _norm(midpoint)
    if not math.isfinite(range_m) or range_m <= 0.0:
        return RayTriangulationAttempt(None, "invalid_range")

    result = RayTriangulationResult(
        point_center_rdf_m=midpoint,
        range_m=range_m,
        closest_gap_m=_norm(_subtract(left_closest, right_closest)),
        left_depth_m=left_depth_m,
        right_depth_m=right_depth_m,
        ray_angle_deg=math.degrees(math.acos(ray_dot)),
        baseline_m=baseline_m,
    )
    return RayTriangulationAttempt(result, "ok")


def triangulate_pixels(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: PinholeIntrinsics,
    right_intrinsics: PinholeIntrinsics,
    left_origin_center_rdf_m: Sequence[float],
    right_origin_center_rdf_m: Sequence[float],
    rotation_center_from_left_camera_rdf: Sequence[Sequence[float]] = IDENTITY_ROTATION,
    rotation_center_from_right_camera_rdf: Sequence[Sequence[float]] = IDENTITY_ROTATION,
) -> RayTriangulationAttempt:
    left_ray_center = rotate_vector(
        rotation_center_from_left_camera_rdf,
        ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics),
    )
    right_ray_center = rotate_vector(
        rotation_center_from_right_camera_rdf,
        ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics),
    )
    return triangulate_rays(
        left_origin_center_rdf_m=left_origin_center_rdf_m,
        right_origin_center_rdf_m=right_origin_center_rdf_m,
        left_ray_center_rdf=left_ray_center,
        right_ray_center_rdf=right_ray_center,
    )
