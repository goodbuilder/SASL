from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Integral
from typing import Any

from stereo_ground_sim.coordinate_frames import (
    FrameId,
    RigidTransform3D,
    Ray3D,
    transform_ray,
)
from stereo_ground_sim.triangulation_core import (
    PinholeIntrinsics,
    TriangulationResult,
    camera_ray_from_pixel,
    triangulate_rays_baseline_center_frd,
)


R_FRD_RDF: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)
COMPARISON_SOURCE = "legacy_prefusion_bbox_centers"
NOT_EVALUATED_BOUNDS = "not_evaluated"


@dataclass(frozen=True)
class NominalStereoTriangulation:
    observation_valid: bool
    observation_status: str
    pixel_bounds_status: str
    triangulation_result: TriangulationResult | None
    left_ray_baseline_center_frd: Ray3D | None = None
    right_ray_baseline_center_frd: Ray3D | None = None


def _positive_int(value: object, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{field_name} must be a positive integer")
    parsed = int(value)
    if parsed <= 0:
        raise ValueError(f"{field_name} must be positive")
    return parsed


def _finite_float(value: object, *, field_name: str) -> float:
    if isinstance(value, (bool, str)):
        raise ValueError(f"{field_name} must be a finite float")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be a finite float") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field_name} must be finite")
    return parsed


def _finite_pixel(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def pinhole_intrinsics_from_horizontal_fov(
    *,
    width_px: int,
    height_px: int,
    horizontal_fov_rad: float,
) -> PinholeIntrinsics:
    """Build Phase 1B ideal pinhole intrinsics from the legacy Node contract.

    Distortion is not supported here. The principal point is the image center
    and square pixels are assumed, so `fy_px == fx_px`.
    """

    width = _positive_int(width_px, field_name="width_px")
    height = _positive_int(height_px, field_name="height_px")
    horizontal_fov = _finite_float(horizontal_fov_rad, field_name="horizontal_fov_rad")
    if horizontal_fov <= 0.0 or horizontal_fov >= math.pi:
        raise ValueError("horizontal_fov_rad must be > 0 and < pi")
    fx_px = (float(width) * 0.5) / math.tan(horizontal_fov * 0.5)
    return PinholeIntrinsics(
        width_px=width,
        height_px=height,
        fx_px=fx_px,
        fy_px=fx_px,
        cx_px=float(width) * 0.5,
        cy_px=float(height) * 0.5,
    )


def nominal_camera_transforms(
    *,
    baseline_m: float,
) -> tuple[RigidTransform3D, RigidTransform3D]:
    """Return left/right Camera RDF -> baseline-center FRD transforms.

    Camera centers in baseline-center FRD are `(0, -baseline/2, 0)` for the
    left camera and `(0, +baseline/2, 0)` for the right camera. Because
    `RigidTransform3D` stores the target origin in the source frame, the
    corresponding `origin_target_in_source_m` values are `(baseline/2, 0, 0)`
    for the left camera and `(-baseline/2, 0, 0)` for the right camera.
    """

    baseline = _finite_float(baseline_m, field_name="baseline_m")
    if baseline < 0.0:
        raise ValueError("baseline_m must be non-negative")
    half_baseline = baseline * 0.5
    left = RigidTransform3D(
        source_frame=FrameId.CAMERA_OPTICAL_RDF,
        target_frame=FrameId.BASELINE_CENTER_FRD,
        rotation_target_source=R_FRD_RDF,
        origin_target_in_source_m=(half_baseline, 0.0, 0.0),
    )
    right = RigidTransform3D(
        source_frame=FrameId.CAMERA_OPTICAL_RDF,
        target_frame=FrameId.BASELINE_CENTER_FRD,
        rotation_target_source=R_FRD_RDF,
        origin_target_in_source_m=(-half_baseline, 0.0, 0.0),
    )
    return left, right


def _pixel_bounds_status(
    *,
    left_u_px: float,
    left_v_px: float,
    right_u_px: float,
    right_v_px: float,
    width_px: int,
    height_px: int,
) -> str:
    def in_bounds(u_px: float, v_px: float) -> bool:
        return 0.0 <= u_px < float(width_px) and 0.0 <= v_px < float(height_px)

    left_in_bounds = in_bounds(left_u_px, left_v_px)
    right_in_bounds = in_bounds(right_u_px, right_v_px)
    if left_in_bounds and right_in_bounds:
        return "all_in_bounds"
    if not left_in_bounds and not right_in_bounds:
        return "both_out_of_bounds"
    if not left_in_bounds:
        return "left_out_of_bounds"
    return "right_out_of_bounds"


def nominal_stereo_rays_from_centers(
    *,
    left_cx_px: float,
    left_cy_px: float,
    right_cx_px: float,
    right_cy_px: float,
    intrinsics: PinholeIntrinsics,
    baseline_m: float,
) -> tuple[Ray3D, Ray3D]:
    left_transform, right_transform = nominal_camera_transforms(baseline_m=baseline_m)
    left_ray = transform_ray(
        camera_ray_from_pixel(left_cx_px, left_cy_px, intrinsics),
        left_transform,
    )
    right_ray = transform_ray(
        camera_ray_from_pixel(right_cx_px, right_cy_px, intrinsics),
        right_transform,
    )
    return left_ray, right_ray


def triangulate_nominal_stereo_centers(
    *,
    left_cx_px: float,
    left_cy_px: float,
    right_cx_px: float,
    right_cy_px: float,
    width_px: int,
    height_px: int,
    horizontal_fov_rad: float,
    baseline_m: float,
    min_intersection_angle_rad: float | None = None,
) -> NominalStereoTriangulation:
    width = _positive_int(width_px, field_name="width_px")
    height = _positive_int(height_px, field_name="height_px")
    baseline = _finite_float(baseline_m, field_name="baseline_m")
    if baseline < 0.0:
        raise ValueError("baseline_m must be non-negative")
    intrinsics = pinhole_intrinsics_from_horizontal_fov(
        width_px=width,
        height_px=height,
        horizontal_fov_rad=horizontal_fov_rad,
    )

    pixels = (
        _finite_pixel(left_cx_px),
        _finite_pixel(left_cy_px),
        _finite_pixel(right_cx_px),
        _finite_pixel(right_cy_px),
    )
    if any(value is None for value in pixels):
        return NominalStereoTriangulation(
            observation_valid=False,
            observation_status="non_finite_pixel",
            pixel_bounds_status=NOT_EVALUATED_BOUNDS,
            triangulation_result=None,
        )
    left_cx, left_cy, right_cx, right_cy = (float(value) for value in pixels)
    pixel_bounds_status = _pixel_bounds_status(
        left_u_px=left_cx,
        left_v_px=left_cy,
        right_u_px=right_cx,
        right_v_px=right_cy,
        width_px=width,
        height_px=height,
    )
    left_ray, right_ray = nominal_stereo_rays_from_centers(
        left_cx_px=left_cx,
        left_cy_px=left_cy,
        right_cx_px=right_cx,
        right_cy_px=right_cy,
        intrinsics=intrinsics,
        baseline_m=baseline,
    )
    triangulation = triangulate_rays_baseline_center_frd(
        left_ray,
        right_ray,
        min_intersection_angle_rad=min_intersection_angle_rad,
    )
    return NominalStereoTriangulation(
        observation_valid=True,
        observation_status="ok",
        pixel_bounds_status=pixel_bounds_status,
        triangulation_result=triangulation,
        left_ray_baseline_center_frd=left_ray,
        right_ray_baseline_center_frd=right_ray,
    )


def _finite_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _delta_or_none(general_value: float | None, legacy_value: Any) -> float | None:
    legacy = _finite_or_none(legacy_value)
    if general_value is None or legacy is None:
        return None
    return float(general_value) - legacy


def general_ray_shadow_not_computed(
    *,
    status: str,
    pixel_bounds_status: str = NOT_EVALUATED_BOUNDS,
    comparison_source: str = COMPARISON_SOURCE,
) -> dict[str, object]:
    return {
        "valid": False,
        "status": str(status),
        "output_frame": FrameId.BASELINE_CENTER_FRD.value,
        "point_baseline_center_frd_m": None,
        "forward_m": None,
        "right_m": None,
        "down_m": None,
        "range_m": None,
        "ray_gap_m": None,
        "intersection_angle_rad": None,
        "left_ray_parameter_m": None,
        "right_ray_parameter_m": None,
        "legacy_forward_delta_m": None,
        "legacy_right_delta_m": None,
        "legacy_down_delta_m": None,
        "legacy_range_delta_m": None,
        "pixel_bounds_status": pixel_bounds_status,
        "comparison_source": comparison_source,
    }


def general_ray_shadow_summary(
    result: NominalStereoTriangulation,
    *,
    legacy_forward_m: object,
    legacy_right_m: object,
    legacy_down_m: object,
    legacy_range_m: object,
    comparison_source: str = COMPARISON_SOURCE,
) -> dict[str, object]:
    triangulation = result.triangulation_result
    if triangulation is None:
        return general_ray_shadow_not_computed(
            status=result.observation_status,
            pixel_bounds_status=result.pixel_bounds_status,
            comparison_source=comparison_source,
        )

    point = None if triangulation.point_m is None else [float(value) for value in triangulation.point_m]
    forward_m = None if point is None else point[0]
    right_m = None if point is None else point[1]
    down_m = None if point is None else point[2]
    range_m = _finite_or_none(triangulation.range_m)
    return {
        "valid": bool(triangulation.valid),
        "status": triangulation.status.value,
        "output_frame": triangulation.output_frame.value,
        "point_baseline_center_frd_m": point,
        "forward_m": forward_m,
        "right_m": right_m,
        "down_m": down_m,
        "range_m": range_m,
        "ray_gap_m": _finite_or_none(triangulation.ray_gap_m),
        "intersection_angle_rad": _finite_or_none(triangulation.intersection_angle_rad),
        "left_ray_parameter_m": _finite_or_none(triangulation.left_ray_parameter_m),
        "right_ray_parameter_m": _finite_or_none(triangulation.right_ray_parameter_m),
        "legacy_forward_delta_m": _delta_or_none(forward_m, legacy_forward_m),
        "legacy_right_delta_m": _delta_or_none(right_m, legacy_right_m),
        "legacy_down_delta_m": _delta_or_none(down_m, legacy_down_m),
        "legacy_range_delta_m": _delta_or_none(range_m, legacy_range_m),
        "pixel_bounds_status": result.pixel_bounds_status,
        "comparison_source": comparison_source,
    }


__all__ = [
    "COMPARISON_SOURCE",
    "NOT_EVALUATED_BOUNDS",
    "NominalStereoTriangulation",
    "R_FRD_RDF",
    "general_ray_shadow_not_computed",
    "general_ray_shadow_summary",
    "nominal_camera_transforms",
    "nominal_stereo_rays_from_centers",
    "pinhole_intrinsics_from_horizontal_fov",
    "triangulate_nominal_stereo_centers",
]
