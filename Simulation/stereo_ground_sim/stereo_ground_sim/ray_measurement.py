from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_FRAME,
    IDENTITY_ROTATION,
    PinholeIntrinsics,
    RayTriangulationAttempt,
    STEREO_CENTER_OPTICAL_FRAME,
    TRIANGULATION_METHOD,
    Vector3,
    project_center_point_to_pixel,
    triangulate_pixels,
)


PixelGrid = tuple[int, int]


@dataclass(frozen=True)
class CameraInfoPinholeModel:
    """Distortion-free K matrix tied to one exact ``(width, height)`` grid."""

    intrinsics: PinholeIntrinsics
    width_px: int
    height_px: int
    distortion_coefficients: tuple[float, ...]

    @property
    def pixel_grid(self) -> PixelGrid:
        return self.width_px, self.height_px


def empty_triangulation_status(reason: str) -> dict[str, object]:
    return {
        "valid": False,
        "reason": str(reason),
        "closest_gap_m": None,
        "left_depth_m": None,
        "right_depth_m": None,
        "ray_angle_deg": None,
        "point_frame": STEREO_CENTER_OPTICAL_FRAME,
        "frame": BASELINE_CENTER_FRD_FRAME,
        "method": TRIANGULATION_METHOD,
    }


def triangulation_status(attempt: RayTriangulationAttempt) -> dict[str, object]:
    if attempt.result is None:
        return empty_triangulation_status(attempt.reason)
    result = attempt.result
    return {
        "valid": True,
        "reason": "ok",
        "closest_gap_m": float(result.closest_gap_m),
        "left_depth_m": float(result.left_depth_m),
        "right_depth_m": float(result.right_depth_m),
        "ray_angle_deg": float(result.ray_angle_deg),
        "point_frame": result.point_frame,
        "frame": result.output_frame,
        "method": result.method,
    }


def intrinsics_from_camera_info_k(values: Sequence[float]) -> PinholeIntrinsics:
    """Parse ROS CameraInfo.K with pixel order [fx,0,cx; 0,fy,cy; 0,0,1]."""
    if len(values) != 9:
        raise ValueError("CameraInfo.k must contain nine values")
    return PinholeIntrinsics(
        fx_px=float(values[0]),
        fy_px=float(values[4]),
        cx_px=float(values[2]),
        cy_px=float(values[5]),
    )


def camera_model_from_camera_info(
    *,
    k: Sequence[float],
    width_px: int,
    height_px: int,
    distortion: Sequence[float],
) -> CameraInfoPinholeModel:
    """Build the Simulation camera model or reject unsupported pixel inputs.

    Simulation consumes raw image/bbox pixels with ``CameraInfo.K`` directly.
    It has no undistortion stage, so ``D`` must be empty or exactly zero and K
    remains valid only for the CameraInfo ``(width, height)`` pixel grid.
    """
    try:
        width_px = int(width_px)
        height_px = int(height_px)
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid_camera_info_grid") from exc
    if width_px <= 0 or height_px <= 0:
        raise ValueError("invalid_camera_info_grid")
    try:
        distortion_coefficients = tuple(float(value) for value in distortion)
    except (TypeError, ValueError) as exc:
        raise ValueError("unsupported_camera_distortion") from exc
    if any(not math.isfinite(value) or value != 0.0 for value in distortion_coefficients):
        raise ValueError("unsupported_camera_distortion")
    try:
        intrinsics = intrinsics_from_camera_info_k(k)
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid_camera_info_intrinsics") from exc
    return CameraInfoPinholeModel(
        intrinsics=intrinsics,
        width_px=width_px,
        height_px=height_px,
        distortion_coefficients=distortion_coefficients,
    )


def pixel_grid_contract_reason(
    *,
    left_input_grid: PixelGrid,
    right_input_grid: PixelGrid,
    left_camera: CameraInfoPinholeModel,
    right_camera: CameraInfoPinholeModel,
) -> str | None:
    """Return the fail-closed reason when input pixels are not on each K grid."""
    try:
        left_grid = (int(left_input_grid[0]), int(left_input_grid[1]))
        right_grid = (int(right_input_grid[0]), int(right_input_grid[1]))
    except (IndexError, TypeError, ValueError):
        return "pixel_grid_mismatch"
    if left_grid != left_camera.pixel_grid or right_grid != right_camera.pixel_grid:
        return "pixel_grid_mismatch"
    return None


def gazebo_world_xyz_to_baseline_center_frd(
    *,
    world_x_m: float,
    world_y_m: float,
    world_z_m: float,
    camera_height_m: float,
    target_origin_z_offset_m: float,
) -> Vector3:
    """Map Gazebo world XYZ to signed baseline-center FRD metres.

    The scenario contract aliases world +X to Right and world +Y to Forward.
    World +Z is Up, so Down is camera height minus the target reference height.
    In particular, a target at negative world Y remains negative Forward.
    """
    values = (
        world_x_m,
        world_y_m,
        world_z_m,
        camera_height_m,
        target_origin_z_offset_m,
    )
    parsed = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError("Gazebo world position and height inputs must be finite")
    world_x_m, world_y_m, world_z_m, camera_height_m, target_origin_z_offset_m = parsed
    target_reference_up_m = world_z_m + target_origin_z_offset_m
    return world_y_m, world_x_m, camera_height_m - target_reference_up_m


def nominal_camera_origins_center_rdf(baseline_m: float) -> tuple[Vector3, Vector3]:
    """Return left/right origins in stereo-center optical RDF metres.

    The baseline is on the optical right axis.  This raw estimator accepts no
    attitude-error input; simulated mounting errors remain physical SDF state.
    """
    baseline_m = float(baseline_m)
    if baseline_m <= 0.0:
        raise ValueError("baseline_m must be positive")
    half_baseline_m = baseline_m * 0.5
    return (-half_baseline_m, 0.0, 0.0), (half_baseline_m, 0.0, 0.0)


def triangulate_nominal_stereo_pixels(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: PinholeIntrinsics,
    right_intrinsics: PinholeIntrinsics,
    baseline_m: float,
) -> RayTriangulationAttempt:
    """Triangulate raw pixels into a stereo-center point using two 3-D rays."""
    left_origin, right_origin = nominal_camera_origins_center_rdf(baseline_m)
    return triangulate_pixels(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=left_intrinsics,
        right_intrinsics=right_intrinsics,
        left_origin_center_rdf_m=left_origin,
        right_origin_center_rdf_m=right_origin,
        rotation_center_from_left_camera_rdf=IDENTITY_ROTATION,
        rotation_center_from_right_camera_rdf=IDENTITY_ROTATION,
    )


def project_nominal_stereo_point(
    point_center_rdf_m: Sequence[float],
    *,
    left_intrinsics: PinholeIntrinsics,
    right_intrinsics: PinholeIntrinsics,
    baseline_m: float,
) -> tuple[tuple[float, float] | None, tuple[float, float] | None]:
    """Project a stereo-center RDF point through both complete pinhole poses."""
    left_origin, right_origin = nominal_camera_origins_center_rdf(baseline_m)
    left_pixel = project_center_point_to_pixel(
        point_center_rdf_m,
        camera_origin_center_rdf_m=left_origin,
        rotation_center_from_camera_rdf=IDENTITY_ROTATION,
        intrinsics=left_intrinsics,
    )
    right_pixel = project_center_point_to_pixel(
        point_center_rdf_m,
        camera_origin_center_rdf_m=right_origin,
        rotation_center_from_camera_rdf=IDENTITY_ROTATION,
        intrinsics=right_intrinsics,
    )
    return left_pixel, right_pixel
