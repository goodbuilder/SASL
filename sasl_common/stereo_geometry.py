from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]

CAMERA_OPTICAL_RDF_FRAME = "camera_optical_rdf"
LEFT_CAMERA_RDF_FRAME = "left_camera_rdf"
LEFT_CAMERA_RDF_AXIS_CONTRACT = "left_camera_rdf:right_down_forward"
LEFT_CAMERA_RDF_VECTOR_CONTRACT = "left_camera_rdf:[right,down,forward,range]"
LEFT_CAMERA_FRD_AXIS_CONTRACT = "left_camera_frd:forward_right_down"
TRIANGULATION_METHOD = "two_ray_closest_midpoint"

IDENTITY_ROTATION: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


@dataclass(frozen=True)
class PinholeIntrinsics:
    """Pinhole intrinsics in pixels for an OpenCV optical RDF camera.

    ``u = fx * Right / Forward + cx`` and
    ``v = fy * Down / Forward + cy``.
    """

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
    """Closest-ray midpoint expressed from the left optical center.

    ``point_left_rdf_m`` is ordered ``(Right, Down, Forward)`` in metres.
    ``range_m`` is its Euclidean norm from the left camera optical center.
    Both camera rays and the right-camera origin are expressed in the same
    ``left_camera_rdf`` frame before this result is constructed.
    """

    point_left_rdf_m: Vector3
    range_m: float
    closest_gap_m: float
    left_depth_m: float
    right_depth_m: float
    ray_angle_deg: float
    baseline_m: float
    method: str = TRIANGULATION_METHOD
    point_frame: str = LEFT_CAMERA_RDF_FRAME
    output_frame: str = LEFT_CAMERA_RDF_FRAME
    axis_contract: str = LEFT_CAMERA_RDF_AXIS_CONTRACT
    vector_contract: str = LEFT_CAMERA_RDF_VECTOR_CONTRACT

    @property
    def point_left_m(self) -> Vector3:
        """Compatibility alias with an explicit left-camera RDF contract."""

        return self.point_left_rdf_m

    @property
    def distance_m(self) -> float:
        """Compatibility alias for ``range_m``."""

        return self.range_m


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
    """Rotate a direction using ``R_output_from_input``."""

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


def ray_from_pixel(u_px: float, v_px: float, intrinsics: PinholeIntrinsics) -> Vector3:
    """Return a unit ray in the source camera's optical RDF frame."""

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


def triangulate_rays(
    *,
    right_origin_left_rdf_m: Sequence[float],
    left_ray_left_rdf: Sequence[float],
    right_ray_left_rdf: Sequence[float],
    parallel_epsilon: float = 1.0e-12,
) -> RayTriangulationAttempt:
    """Find the closest-ray midpoint in ``left_camera_rdf``.

    The left optical center is always ``(0, 0, 0)``.  The right optical
    center and both unit-ray directions are expressed in left-camera RDF.
    No baseline-center translation or FRD axis reorder is performed.
    """

    left_origin: Vector3 = (0.0, 0.0, 0.0)
    right_origin = _vector3(right_origin_left_rdf_m, name="right_origin_left_rdf_m")
    left_ray = normalize_vector(left_ray_left_rdf)
    right_ray = normalize_vector(right_ray_left_rdf)
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
        point_left_rdf_m=midpoint,
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
    right_origin_left_rdf_m: Sequence[float],
    rotation_left_from_right_rdf: Sequence[Sequence[float]] = IDENTITY_ROTATION,
) -> RayTriangulationAttempt:
    """Triangulate pinhole pixels after expressing the right ray in the left frame."""

    left_ray = ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics)
    right_ray = rotate_vector(
        rotation_left_from_right_rdf,
        ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics),
    )
    return triangulate_rays(
        right_origin_left_rdf_m=right_origin_left_rdf_m,
        left_ray_left_rdf=left_ray,
        right_ray_left_rdf=right_ray,
    )
