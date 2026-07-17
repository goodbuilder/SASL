from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Iterable

from wiris_stereo_distance.attitude_bias import normalize_quaternion, quaternion_conjugate, quaternion_multiply
from wiris_stereo_distance.calibration import CameraIntrinsics
from wiris_stereo_distance.coordinate_contract import (
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    WORLD_FRAME,
)

CAMERA_TO_BODY_ROTATION: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)
BODY_TO_CAMERA_ROTATION: tuple[tuple[float, float, float], ...] = (
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
)


@dataclass(frozen=True)
class RayTriangulationResult:
    distance_m: float
    point_left_m: tuple[float, float, float]
    closest_gap_m: float
    baseline_m: float
    method: str = "ray-triangulation"


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, s: float):
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm(a: tuple[float, float, float]) -> float:
    return sqrt(_dot(a, a))


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = _norm(v)
    if n <= 0.0:
        raise ValueError("zero-length vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def stereo_center_point_from_left_camera(
    point_left_m: tuple[float, float, float],
    translation_right_in_left_m: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Move a triangulated left-camera point to the stereo-rig center optical frame."""
    return _sub(point_left_m, _scale(translation_right_in_left_m, 0.5))


def _rotate(q: Iterable[float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    quat = normalize_quaternion(q)
    pure = (0.0, v[0], v[1], v[2])
    rotated = _quaternion_multiply_raw(_quaternion_multiply_raw(quat, pure), quaternion_conjugate(quat))
    return (rotated[1], rotated[2], rotated[3])


def _quaternion_multiply_raw(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _rotate_matrix(matrix: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("rotation matrix must be 3x3")
    return (
        float(matrix[0][0]) * v[0] + float(matrix[0][1]) * v[1] + float(matrix[0][2]) * v[2],
        float(matrix[1][0]) * v[0] + float(matrix[1][1]) * v[1] + float(matrix[1][2]) * v[2],
        float(matrix[2][0]) * v[0] + float(matrix[2][1]) * v[1] + float(matrix[2][2]) * v[2],
    )


def invert_left_to_right_camera_transform(
    rotation_left_to_right: list[list[float]],
    translation_left_to_right_m: Iterable[float],
) -> tuple[list[list[float]], tuple[float, float, float]]:
    """Invert OpenCV's left-to-right camera transform into right-in-left form.

    The checkerboard calibration contract is
    ``X_right = R_left_to_right * X_left + T_left_to_right_m`` in optical RDF.
    Triangulation needs the right-camera ray rotation and right-camera origin,
    both expressed in the left optical RDF frame.
    """
    if len(rotation_left_to_right) != 3 or any(len(row) != 3 for row in rotation_left_to_right):
        raise ValueError("rotation_left_to_right must be a 3x3 matrix")
    translation = tuple(float(value) for value in translation_left_to_right_m)
    if len(translation) != 3:
        raise ValueError("translation_left_to_right_m must contain 3 values")
    right_to_left = [
        [float(rotation_left_to_right[row][column]) for row in range(3)]
        for column in range(3)
    ]
    right_origin_in_left = tuple(
        -sum(right_to_left[row][column] * translation[row] for row in range(3))
        for column in range(3)
    )
    return right_to_left, right_origin_in_left


def _rotate_matrix_tuple(
    matrix: tuple[tuple[float, float, float], ...],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        matrix[0][0] * v[0] + matrix[0][1] * v[1] + matrix[0][2] * v[2],
        matrix[1][0] * v[0] + matrix[1][1] * v[1] + matrix[1][2] * v[2],
        matrix[2][0] * v[0] + matrix[2][1] * v[1] + matrix[2][2] * v[2],
    )


def camera_vector_to_body_frd(v: tuple[float, float, float]) -> tuple[float, float, float]:
    return _rotate_matrix_tuple(CAMERA_TO_BODY_ROTATION, v)


def body_frd_vector_to_camera(v: tuple[float, float, float]) -> tuple[float, float, float]:
    return _rotate_matrix_tuple(BODY_TO_CAMERA_ROTATION, v)


def world_ned_vector_to_body_frd(
    world_vector: tuple[float, float, float],
    body_to_world_q: Iterable[float],
) -> tuple[float, float, float]:
    return _rotate(quaternion_conjugate(normalize_quaternion(body_to_world_q)), world_vector)


def camera_translation_right_in_left_camera(
    *,
    baseline_right_in_left_world_m: tuple[float, float, float],
    left_body_to_world_q: Iterable[float],
    right_body_to_world_q: Iterable[float],
    scale_to_m: float | None = None,
) -> tuple[float, float, float] | None:
    _ = right_body_to_world_q
    translation_left_body = world_ned_vector_to_body_frd(baseline_right_in_left_world_m, left_body_to_world_q)
    translation_left_camera = body_frd_vector_to_camera(translation_left_body)
    if scale_to_m is None:
        return translation_left_camera
    norm = _norm(translation_left_camera)
    if norm <= 0.0:
        return None
    return _scale(translation_left_camera, float(scale_to_m) / norm)


def right_camera_ray_to_left_camera(
    right_ray_camera: tuple[float, float, float],
    *,
    left_body_to_world_q: Iterable[float],
    right_body_to_world_q: Iterable[float],
) -> tuple[float, float, float]:
    right_ray_body = camera_vector_to_body_frd(right_ray_camera)
    right_ray_world = _rotate(right_body_to_world_q, right_ray_body)
    right_ray_left_body = world_ned_vector_to_body_frd(right_ray_world, left_body_to_world_q)
    return body_frd_vector_to_camera(right_ray_left_body)


def relative_rotation_quaternion(left_q: Iterable[float], right_q: Iterable[float]):
    # Legacy helper for callers that already express camera rays in the Pixhawk attitude convention.
    # Live Pixhawk triangulation uses triangulate_pixels_with_pixhawk_mount instead.
    left = normalize_quaternion(left_q)
    right = normalize_quaternion(right_q)
    return quaternion_multiply(quaternion_conjugate(left), right)


def normalized_ray_from_pixel(u: float, v: float, intrinsics: CameraIntrinsics) -> tuple[float, float, float]:
    matrix = intrinsics.camera_matrix
    fx = float(matrix[0][0])
    fy = float(matrix[1][1])
    cx = float(matrix[0][2])
    cy = float(matrix[1][2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("invalid focal length")
    # Distortion is stored but not numerically inverted here to avoid a hard OpenCV dependency
    # in the live distance path. Calibration still records D for future undistortPoints use.
    x = (float(u) - cx) / fx
    y = (float(v) - cy) / fy
    return _normalize((x, y, 1.0))


def triangulate_rays(
    left_ray: tuple[float, float, float],
    right_ray_in_left: tuple[float, float, float],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationResult | None:
    d1 = _normalize(left_ray)
    d2 = _normalize(right_ray_in_left)
    t = translation_right_in_left_m
    baseline = _norm(t)
    if baseline <= 0.0:
        return None

    # Closest points between L(s)=s*d1 and R(u)=t+u*d2.
    a = _dot(d1, d1)
    b = _dot(d1, d2)
    c = _dot(d2, d2)
    w0 = _scale(t, -1.0)
    d = _dot(d1, w0)
    e = _dot(d2, w0)
    denom = a * c - b * b
    if abs(denom) <= 1e-9:
        return None
    s = (b * e - c * d) / denom
    u = (a * e - b * d) / denom
    if s <= 0.0 or u <= 0.0:
        return None
    p1 = _scale(d1, s)
    p2 = _add(t, _scale(d2, u))
    midpoint = _scale(_add(p1, p2), 0.5)
    gap = _norm(_sub(p1, p2))
    distance = _norm(midpoint)
    if distance <= 0.0:
        return None
    return RayTriangulationResult(distance, midpoint, gap, baseline)


def triangulate_pixels(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    left_q: Iterable[float],
    right_q: Iterable[float],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationResult | None:
    left_ray = normalized_ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics)
    right_ray = normalized_ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics)
    right_to_left_q = relative_rotation_quaternion(left_q, right_q)
    right_ray_in_left = _normalize(_rotate(right_to_left_q, right_ray))
    return triangulate_rays(left_ray, right_ray_in_left, translation_right_in_left_m)


def triangulate_pixels_with_pixhawk_mount(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    left_body_to_world_q: Iterable[float],
    right_body_to_world_q: Iterable[float],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationResult | None:
    left_ray = normalized_ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics)
    right_ray = normalized_ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics)
    right_ray_in_left = _normalize(
        right_camera_ray_to_left_camera(
            right_ray,
            left_body_to_world_q=left_body_to_world_q,
            right_body_to_world_q=right_body_to_world_q,
        )
    )
    return triangulate_rays(left_ray, right_ray_in_left, translation_right_in_left_m)


def triangulate_pixels_with_relative_pose(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    rotation_right_to_left: list[list[float]],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationResult | None:
    left_ray = normalized_ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics)
    right_ray = normalized_ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics)
    right_ray_in_left = _normalize(_rotate_matrix(rotation_right_to_left, right_ray))
    return triangulate_rays(left_ray, right_ray_in_left, translation_right_in_left_m)
