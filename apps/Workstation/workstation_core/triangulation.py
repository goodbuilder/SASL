from __future__ import annotations

from math import sqrt
from typing import Iterable

from sasl_common.stereo_geometry import (
    PinholeIntrinsics,
    RayTriangulationAttempt,
    RayTriangulationResult,
    normalize_vector,
    ray_from_pixel,
    result_as_baseline_center_frd,
    triangulate_rays as triangulate_common_rays,
)
from Workstation.workstation_core.attitude_geometry import (
    normalize_quaternion,
    quaternion_conjugate,
    quaternion_multiply,
    rotate_vector_by_quaternion,
    world_ned_vector_to_body_frd,
)
from Workstation.workstation_core.calibration import CameraIntrinsics
from Workstation.workstation_core.coordinate_contract import (
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

def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a, s: float):
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm(a: tuple[float, float, float]) -> float:
    return sqrt(_dot(a, a))


def stereo_center_point_from_left_camera(
    point_left_m: tuple[float, float, float],
    translation_right_in_left_m: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Move a triangulated left-camera point to the stereo-rig center optical frame."""
    return _sub(point_left_m, _scale(translation_right_in_left_m, 0.5))


def _rotate(q: Iterable[float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return rotate_vector_by_quaternion(v, q)


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
        -sum(right_to_left[row][column] * translation[column] for column in range(3))
        for row in range(3)
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
    """Convert a detector pixel to an undistorted optical RDF unit ray."""

    matrix = intrinsics.camera_matrix
    fx = float(matrix[0][0])
    fy = float(matrix[1][1])
    cx = float(matrix[0][2])
    cy = float(matrix[1][2])
    pinhole = PinholeIntrinsics(fx_px=fx, fy_px=fy, cx_px=cx, cy_px=cy)
    distortion = tuple(float(value) for value in intrinsics.distortion_coefficients)
    if distortion and any(abs(value) > 1.0e-15 for value in distortion):
        import cv2
        import numpy as np

        normalized = cv2.undistortPoints(
            np.asarray([[[float(u), float(v)]]], dtype=np.float64),
            np.asarray(intrinsics.camera_matrix, dtype=np.float64),
            np.asarray(distortion, dtype=np.float64),
        )[0, 0]
        return normalize_vector((float(normalized[0]), float(normalized[1]), 1.0))
    return ray_from_pixel(u, v, pinhole)


def triangulate_rays_attempt(
    left_ray: tuple[float, float, float],
    right_ray_in_left: tuple[float, float, float],
    translation_right_in_left_m: tuple[float, float, float],
    *,
    right_forward_in_left: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> RayTriangulationAttempt:
    """Run DKL closest-ray geometry and publish ``baseline_center_frd``.

    Inputs are ordered optical RDF in the left-camera frame. The internal
    closest-ray solution remains left-camera RDF for diagnostics, then the
    public result is transformed to attempt-frozen baseline-center FRD:
    origin is the midpoint of the left optical origin and selected right-origin
    estimate, output order is ``(Forward, Right, Down)``, and range is measured
    from that midpoint. The Pixhawk path records its GPS/RTK antenna baseline
    as an optical-baseline proxy rather than claiming a calibrated lever arm.
    """

    attempt = triangulate_common_rays(
        right_origin_left_rdf_m=translation_right_in_left_m,
        left_ray_left_rdf=left_ray,
        right_ray_left_rdf=right_ray_in_left,
    )
    if attempt.result is None:
        return attempt
    try:
        converted = result_as_baseline_center_frd(
            attempt.result,
            right_origin_left_rdf_m=translation_right_in_left_m,
            right_forward_left_rdf=right_forward_in_left,
        )
    except (TypeError, ValueError) as exc:
        return RayTriangulationAttempt(
            None,
            "baseline_center_transform_error",
            {**attempt.diagnostics, "error": str(exc)},
        )
    return RayTriangulationAttempt(converted, attempt.reason, attempt.diagnostics)


def triangulate_rays(
    left_ray: tuple[float, float, float],
    right_ray_in_left: tuple[float, float, float],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationResult | None:
    return triangulate_rays_attempt(left_ray, right_ray_in_left, translation_right_in_left_m).result


def triangulate_pixels_attempt(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    left_q: Iterable[float],
    right_q: Iterable[float],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationAttempt:
    left_ray = normalized_ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics)
    right_ray = normalized_ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics)
    right_to_left_q = relative_rotation_quaternion(left_q, right_q)
    right_ray_in_left = normalize_vector(_rotate(right_to_left_q, right_ray))
    right_forward_in_left = normalize_vector(_rotate(right_to_left_q, (0.0, 0.0, 1.0)))
    return triangulate_rays_attempt(
        left_ray,
        right_ray_in_left,
        translation_right_in_left_m,
        right_forward_in_left=right_forward_in_left,
    )


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
    return triangulate_pixels_attempt(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=left_intrinsics,
        right_intrinsics=right_intrinsics,
        left_q=left_q,
        right_q=right_q,
        translation_right_in_left_m=translation_right_in_left_m,
    ).result


def triangulate_pixels_with_pixhawk_mount_attempt(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    left_body_to_world_q: Iterable[float],
    right_body_to_world_q: Iterable[float],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationAttempt:
    left_ray = normalized_ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics)
    right_ray = normalized_ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics)
    right_ray_in_left = normalize_vector(
        right_camera_ray_to_left_camera(
            right_ray,
            left_body_to_world_q=left_body_to_world_q,
            right_body_to_world_q=right_body_to_world_q,
        )
    )
    right_forward_in_left = normalize_vector(
        right_camera_ray_to_left_camera(
            (0.0, 0.0, 1.0),
            left_body_to_world_q=left_body_to_world_q,
            right_body_to_world_q=right_body_to_world_q,
        )
    )
    return triangulate_rays_attempt(
        left_ray,
        right_ray_in_left,
        translation_right_in_left_m,
        right_forward_in_left=right_forward_in_left,
    )


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
    return triangulate_pixels_with_pixhawk_mount_attempt(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=left_intrinsics,
        right_intrinsics=right_intrinsics,
        left_body_to_world_q=left_body_to_world_q,
        right_body_to_world_q=right_body_to_world_q,
        translation_right_in_left_m=translation_right_in_left_m,
    ).result


def triangulate_pixels_with_relative_pose_attempt(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    rotation_right_to_left: list[list[float]],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationAttempt:
    left_ray = normalized_ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics)
    right_ray = normalized_ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics)
    right_ray_in_left = normalize_vector(_rotate_matrix(rotation_right_to_left, right_ray))
    right_forward_in_left = normalize_vector(_rotate_matrix(rotation_right_to_left, (0.0, 0.0, 1.0)))
    return triangulate_rays_attempt(
        left_ray,
        right_ray_in_left,
        translation_right_in_left_m,
        right_forward_in_left=right_forward_in_left,
    )


def triangulate_pixels_with_relative_pose(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    rotation_right_to_left: list[list[float]],
    translation_right_in_left_m: tuple[float, float, float],
) -> RayTriangulationResult | None:
    return triangulate_pixels_with_relative_pose_attempt(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=left_intrinsics,
        right_intrinsics=right_intrinsics,
        rotation_right_to_left=rotation_right_to_left,
        translation_right_in_left_m=translation_right_in_left_m,
    ).result
