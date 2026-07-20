from __future__ import annotations

import math

import pytest
from pathlib import Path

from Simulation.stereo_ground_sim.stereo_ground_sim.coordinate_frames import (
    apply_optical_yaw,
    camera_optical_pose_from_sdf_pose,
    effective_optical_yaws_deg,
    nominal_camera_poses,
    project_world_point_to_pixel,
)


def test_effective_common_relative_yaw_contract() -> None:
    assert effective_optical_yaws_deg(0.0, 0.01) == pytest.approx((-0.005, 0.005))
    assert effective_optical_yaws_deg(0.1, 0.0) == pytest.approx((0.1, 0.1))


def test_positive_optical_yaw_rotates_forward_toward_right() -> None:
    nominal, _ = nominal_camera_poses(25.0, 5.0)
    yawed = apply_optical_yaw(nominal, math.radians(1.0))
    assert sum(a * b for a, b in zip(yawed.forward_world_enu, nominal.right_world_enu)) > 0.0
    assert yawed.down_world_enu == nominal.down_world_enu


def test_sdf_yaw_sign_matches_optical_basis_and_projection() -> None:
    nominal, _ = nominal_camera_poses(25.0, 5.0)
    optical_yaw = math.radians(0.1)
    yawed = apply_optical_yaw(nominal, optical_yaw)
    sdf = camera_optical_pose_from_sdf_pose(
        nominal.center_world_enu_m,
        (0.0, 0.0, math.pi / 2.0 - optical_yaw),
    )
    assert sdf.right_world_enu == pytest.approx(yawed.right_world_enu, abs=1e-12)
    assert sdf.forward_world_enu == pytest.approx(yawed.forward_world_enu, abs=1e-12)
    target = tuple(
        nominal.center_world_enu_m[index]
        + 1000.0 * nominal.forward_world_enu[index]
        for index in range(3)
    )
    nominal_pixel = project_world_point_to_pixel(target, nominal, fx_px=1000, fy_px=1000, cx_px=640, cy_px=360)
    yawed_pixel = project_world_point_to_pixel(target, yawed, fx_px=1000, fy_px=1000, cx_px=640, cy_px=360)
    assert nominal_pixel is not None and yawed_pixel is not None
    assert yawed_pixel[0] < nominal_pixel[0]
    launch = (
        Path(__file__).resolve().parents[1]
        / "stereo_ground_sim"
        / "launch"
        / "stereo_ground_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert "camera_yaw - left_optical_yaw" in launch
    assert "camera_yaw - right_optical_yaw" in launch
