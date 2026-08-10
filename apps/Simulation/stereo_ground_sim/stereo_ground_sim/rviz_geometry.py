"""Pure world-ENU geometry for RViz camera and ray visualization."""

import math

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_FRAME,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
)


def uam_gps_ray_visualization_world_enu_from_point(
    baseline_center_frd,
    point_world_enu_m,
):
    """Return one GPS reference ray from the fixed baseline-center origin.

    The GPS reference point is public ``baseline_center_frd`` data, while RViz
    renders in World ENU.  This helper keeps that conversion explicit and does
    not apply camera projection or stereo triangulation.
    """

    try:
        point_world = tuple(float(value) for value in point_world_enu_m[:3])
    except (TypeError, ValueError):
        return None
    if len(point_world) != 3 or not all(math.isfinite(value) for value in point_world):
        return None
    point_frd = tuple(baseline_center_frd.point_from_world(point_world))
    origin_world = tuple(baseline_center_frd.origin_world_enu_m)
    return {
        "origin_world_enu_m": origin_world,
        "target_world_enu_m": point_world,
        "point_baseline_center_frd_m": point_frd,
        "forward_m": float(point_frd[0]),
        "right_m": float(point_frd[1]),
        "down_m": float(point_frd[2]),
        "range_m": math.sqrt(sum(float(value) * float(value) for value in point_frd)),
        "computation_status": "computed",
        "output_frame": BASELINE_CENTER_FRD_FRAME,
        "axis_contract": BASELINE_CENTER_FRD_AXIS_CONTRACT,
        "vector_contract": BASELINE_CENTER_FRD_VECTOR_CONTRACT,
    }


def reference_ray_world_enu_from_point(
    baseline_center_frd,
    left_camera_pose,
    right_camera_pose,
    point_world_enu_m,
):
    """Return display-only camera rays to one World-ENU reference point.

    The input target and returned marker endpoints are World ENU metres.
    Scalar/vector measurement fields are explicitly converted to the
    baseline-center FRD contract `[forward, right, down]` in metres.
    """

    try:
        point_world = tuple(float(value) for value in point_world_enu_m[:3])
    except (TypeError, ValueError):
        return None
    if len(point_world) != 3 or not all(math.isfinite(value) for value in point_world):
        return None
    point_frd = tuple(baseline_center_frd.point_from_world(point_world))
    left_rdf = left_camera_pose.world_to_rdf(point_world)
    right_rdf = right_camera_pose.world_to_rdf(point_world)
    return {
        "point_on_left_ray_baseline_center_frd_m": point_frd,
        "point_on_right_ray_baseline_center_frd_m": point_frd,
        "midpoint_baseline_center_frd_m": point_frd,
        "forward_m": float(point_frd[0]),
        "right_m": float(point_frd[1]),
        "down_m": float(point_frd[2]),
        "range_m": math.sqrt(sum(float(value) * float(value) for value in point_frd)),
        "distance_m": float(point_frd[0]),
        "ray_gap_m": 0.0,
        "intersection_angle_deg": None,
        "left_depth_m": float(left_rdf[2]),
        "right_depth_m": float(right_rdf[2]),
        "geometry_rank": 3,
        "condition_number": 1.0,
        "computation_status": "computed",
        "output_frame": BASELINE_CENTER_FRD_FRAME,
        "point_frame": BASELINE_CENTER_FRD_FRAME,
        "axis_contract": BASELINE_CENTER_FRD_AXIS_CONTRACT,
        "vector_contract": BASELINE_CENTER_FRD_VECTOR_CONTRACT,
        "ray_visualization_world_enu": {
            "left_origin_world_enu_m": tuple(left_camera_pose.center_world_enu_m),
            "left_target_world_enu_m": point_world,
            "right_origin_world_enu_m": tuple(right_camera_pose.center_world_enu_m),
            "right_target_world_enu_m": point_world,
            "midpoint_world_enu_m": point_world,
        },
    }


def _add(a, b):
    return tuple(float(a[i]) + float(b[i]) for i in range(3))


def _scale(vector, scalar):
    return tuple(float(value) * float(scalar) for value in vector)


def camera_frustum_geometry(camera, *, width_px, height_px, horizontal_fov_rad, depth_m):
    """Return world ENU frustum triangles and line segments in metres.

    Camera basis vectors are world-ENU representations of optical
    right/down/forward. They are combined here, then discarded; returned
    points are exclusively world ENU `[x,east-like y,z-up]` positions.
    """

    width = max(1.0, float(width_px))
    height = max(1.0, float(height_px))
    fov = float(horizontal_fov_rad)
    if not math.isfinite(fov) or fov <= 0.0:
        raise ValueError("horizontal_fov_rad must be positive and finite")
    depth = min(max(1.0, float(depth_m)), 2500.0)
    half_width = depth * math.tan(fov * 0.5)
    half_height = half_width * height / width

    def world(right, down):
        return _add(
            camera.center_world_enu_m,
            _add(
                _add(_scale(camera.right_world_enu, right), _scale(camera.down_world_enu, down)),
                _scale(camera.forward_world_enu, depth),
            ),
        )

    origin = tuple(float(v) for v in camera.center_world_enu_m)
    tl, tr = world(-half_width, -half_height), world(half_width, -half_height)
    br, bl = world(half_width, half_height), world(-half_width, half_height)
    triangles = (
        origin, tl, tr, origin, tr, br, origin, br, bl, origin, bl, tl,
        tl, bl, br, tl, br, tr,
    )
    lines = (
        origin, tl, origin, tr, origin, br, origin, bl,
        tl, tr, tr, br, br, bl, bl, tl,
    )
    return {"triangles_world_enu_m": triangles, "lines_world_enu_m": lines}


def ray_line_geometry(result):
    """Return validated world-ENU ray primitives from a computed result."""

    if not isinstance(result, dict) or result.get("computation_status") != "computed":
        return None
    geometry = result.get("ray_visualization_world_enu")
    if not isinstance(geometry, dict):
        return None
    required = (
        "left_origin_world_enu_m", "left_target_world_enu_m",
        "right_origin_world_enu_m", "right_target_world_enu_m",
        "midpoint_world_enu_m",
    )
    if any(geometry.get(name) is None for name in required):
        return None
    return {name: tuple(float(v) for v in geometry[name]) for name in required}
