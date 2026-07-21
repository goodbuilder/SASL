import math

import pytest

from Simulation.stereo_ground_sim.stereo_ground_sim.coordinate_frames import (
    apply_optical_yaw,
    effective_optical_yaws_deg,
    nominal_camera_poses,
    project_world_point_to_pixel,
)


def test_actual_projection_and_nominal_triangulation_inputs_are_float_pixels():
    left, right = nominal_camera_poses(25.0, 5.0)
    ly, ry = effective_optical_yaws_deg(0.02, 0.01)
    actual_left = apply_optical_yaw(left, math.radians(ly))
    actual_right = apply_optical_yaw(right, math.radians(ry))
    target = (0.0, 500.0, 5.0)
    pixels = [
        project_world_point_to_pixel(target, camera, fx_px=8241.646774, fy_px=8241.646774, cx_px=640.0, cy_px=360.0)
        for camera in (actual_left, actual_right)
    ]
    assert all(pixel is not None for pixel in pixels)
    assert all(math.isfinite(value) for pixel in pixels for value in pixel)
    assert pixels[0][0] != int(pixels[0][0])
    assert 0.0 <= pixels[0][0] < 1280.0
    assert 0.0 <= pixels[0][1] < 720.0
    assert 0.0 <= pixels[1][0] < 1280.0
    assert 0.0 <= pixels[1][1] < 720.0


@pytest.mark.parametrize("point", [(0.0, 500.0, 5.0), (float("nan"), 1.0, 1.0), (0.0, 1.0, float("inf"))])
def test_projection_rejects_nonfinite_or_invalid_inputs(point):
    left, _ = nominal_camera_poses(25.0, 5.0)
    if all(math.isfinite(v) for v in point):
        assert project_world_point_to_pixel(point, left, fx_px=8241.6, fy_px=8241.6, cx_px=640, cy_px=360) is not None
    else:
        with pytest.raises(ValueError):
            project_world_point_to_pixel(point, left, fx_px=8241.6, fy_px=8241.6, cx_px=640, cy_px=360)
