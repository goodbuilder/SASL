"""Project world-ENU reference points into OpenCV stereo observations."""

from stereo_ground_sim.coordinate_frames import project_world_point_to_pixel


def detection_pair_from_world(
    point_world_enu_m,
    *,
    left_camera,
    right_camera,
    intrinsics,
    width_px,
    height_px,
    bbox_size_px,
    enforce_fov_bounds=False,
):
    """Return left/right `(u=right,v=down)` point observations.

    Camera poses and input point are world ENU. Returned bbox coordinates are
    image pixels and carry no 3-D frame.
    """

    try:
        left_pixel = project_world_point_to_pixel(
            point_world_enu_m, left_camera,
            fx_px=intrinsics.fx_px, fy_px=intrinsics.fy_px,
            cx_px=intrinsics.cx_px, cy_px=intrinsics.cy_px,
        )
        right_pixel = project_world_point_to_pixel(
            point_world_enu_m, right_camera,
            fx_px=intrinsics.fx_px, fy_px=intrinsics.fy_px,
            cx_px=intrinsics.cx_px, cy_px=intrinsics.cy_px,
        )
    except (TypeError, ValueError, ArithmeticError):
        return None, None
    if left_pixel is None or right_pixel is None:
        return None, None
    if enforce_fov_bounds and any(
        not (
            0.0 <= float(pixel[0]) < float(width_px)
            and 0.0 <= float(pixel[1]) < float(height_px)
        )
        for pixel in (left_pixel, right_pixel)
    ):
        return None, None

    def detection(pixel):
        cx, cy = (float(pixel[0]), float(pixel[1]))
        half = float(bbox_size_px) * 0.5
        return {
            "cx": cx, "cy": cy, "confidence": 1.0,
            "bbox": (cx - half, cy - half, cx + half, cy + half),
        }

    return detection(left_pixel), detection(right_pixel)
