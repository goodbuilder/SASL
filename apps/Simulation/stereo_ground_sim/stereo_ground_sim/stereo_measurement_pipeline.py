"""Pure stereo ray triangulation with an explicit public frame contract."""

import math

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_FRAME,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
    GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    ray_from_pixel,
    triangulate_rays,
)


def triangulate_pixel_centers(
    pixels_uv,
    *,
    intrinsics,
    left_camera_pose,
    right_camera_pose,
    baseline_center_frd,
    visualization_builder=None,
):
    """Triangulate OpenCV optical pixels into baseline-center FRD metres.

    Input pixels are ``(left_u, left_v, right_u, right_v)`` with optical
    ``u=right, v=down``. Camera poses and baseline frame are expressed in
    world ENU. Output vectors are `[forward, right, down]`.
    """

    pixels = tuple(float(value) for value in pixels_uv)
    base = {
        "left_u_px": pixels[0], "left_v_px": pixels[1],
        "right_u_px": pixels[2], "right_v_px": pixels[3],
        "disparity_px": pixels[0] - pixels[2],
        "point_on_left_ray_baseline_center_frd_m": None,
        "point_on_right_ray_baseline_center_frd_m": None,
        "left_ray_origin_baseline_center_frd_m": None,
        "right_ray_origin_baseline_center_frd_m": None,
        "left_ray_direction_baseline_center_frd": None,
        "right_ray_direction_baseline_center_frd": None,
        "midpoint_baseline_center_frd_m": None,
        "forward_m": None, "right_m": None, "down_m": None,
        "range_m": None, "distance_m": None, "ray_gap_m": None,
        "intersection_angle_deg": None, "left_depth_m": None,
        "right_depth_m": None, "geometry_rank": None,
        "condition_number": None, "computation_status": "invalid_input",
        "output_frame": BASELINE_CENTER_FRD_FRAME,
        "point_frame": BASELINE_CENTER_FRD_FRAME,
        "axis_contract": BASELINE_CENTER_FRD_AXIS_CONTRACT,
        "vector_contract": BASELINE_CENTER_FRD_VECTOR_CONTRACT,
    }
    try:
        left_ray_world = left_camera_pose.ray_to_world(
            ray_from_pixel(pixels[0], pixels[1], intrinsics)
        )
        right_ray_world = right_camera_pose.ray_to_world(
            ray_from_pixel(pixels[2], pixels[3], intrinsics)
        )
        left_origin = baseline_center_frd.point_from_world(
            left_camera_pose.center_world_enu_m
        )
        right_origin = baseline_center_frd.point_from_world(
            right_camera_pose.center_world_enu_m
        )
        left_ray = baseline_center_frd.ray_from_world(left_ray_world)
        right_ray = baseline_center_frd.ray_from_world(right_ray_world)
        attempt = triangulate_rays(
            left_origin_left_rdf_m=left_origin,
            right_origin_left_rdf_m=right_origin,
            left_ray_left_rdf=left_ray,
            right_ray_left_rdf=right_ray,
            common_frame=BASELINE_CENTER_FRD_FRAME,
            axis_contract=BASELINE_CENTER_FRD_AXIS_CONTRACT,
            vector_contract=BASELINE_CENTER_FRD_VECTOR_CONTRACT,
            geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
        )
    except (TypeError, ValueError, ArithmeticError) as exc:
        base["diagnostic_error"] = str(exc)
        return base
    result = attempt.result
    base["geometry_rank"] = attempt.diagnostics.get("geometry_rank")
    base["condition_number"] = attempt.diagnostics.get("condition_number")
    if result is None:
        base["computation_status"] = attempt.reason
        return base
    midpoint = result.midpoint_rdf_m or result.point_left_rdf_m
    left_point = result.point_on_left_ray_rdf_m
    right_point = result.point_on_right_ray_rdf_m
    base.update(
        {
            "left_ray_origin_baseline_center_frd_m": tuple(left_origin),
            "right_ray_origin_baseline_center_frd_m": tuple(right_origin),
            "left_ray_direction_baseline_center_frd": tuple(left_ray),
            "right_ray_direction_baseline_center_frd": tuple(right_ray),
            "point_on_left_ray_baseline_center_frd_m": left_point,
            "point_on_right_ray_baseline_center_frd_m": right_point,
            "midpoint_baseline_center_frd_m": midpoint,
            "forward_m": float(midpoint[0]), "right_m": float(midpoint[1]),
            "down_m": float(midpoint[2]),
            "range_m": math.sqrt(sum(float(value) ** 2 for value in midpoint)),
            "distance_m": float(midpoint[0]),
            "ray_gap_m": float(result.closest_gap_m),
            "intersection_angle_deg": float(result.ray_angle_deg),
            "left_depth_m": float(result.left_depth_m),
            "right_depth_m": float(result.right_depth_m),
            "geometry_rank": int(result.geometry_rank),
            "condition_number": float(result.condition_number),
            "computation_status": result.computation_status,
            "point_frame": result.point_frame, "output_frame": result.output_frame,
            "axis_contract": result.axis_contract,
            "vector_contract": result.vector_contract,
        }
    )
    if visualization_builder is not None:
        base["ray_visualization_world_enu"] = visualization_builder(
            baseline_center_frd, left_camera_pose, right_camera_pose,
            left_point, right_point, midpoint,
        )
    return base
