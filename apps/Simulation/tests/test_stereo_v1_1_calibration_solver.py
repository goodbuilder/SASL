from __future__ import annotations

import math

import numpy as np
import pytest

from campaigns.stereo_v1_1.calibration_solver import (
    rotation_world_from_optical,
    solve_calibration,
)


INTRINSICS = {"fx_px": 8241.646774127139, "fy_px": 8241.646774127139,
              "cx_px": 640.0, "cy_px": 360.0}


def _fixture():
    centers = {"left": np.array([-12.5, 0.0, 0.1]),
               "right": np.array([12.5, 0.0, 0.1])}
    sensor = {
        "estimator_input_allowlist": ["cameras.left", "cameras.right", "shared_sensor_key"],
        "shared_sensor_key": "fixture:R01:MC000", "cameras": {},
    }
    for side in ("left", "right"):
        sensor["cameras"][side] = {
            "optical_center_rtk_position_world_enu_m": centers[side].tolist(),
            "optical_center_rtk_covariance_world_enu_m2": np.diag([1e-4, 1e-4, 4e-4]).tolist(),
            "camera_imu_roll_pitch_mean_optical_rad": [math.radians(0.15), math.radians(-0.12)],
            "camera_imu_roll_pitch_covariance_optical_rad2": np.diag([math.radians(2.0) ** 2] * 2).tolist(),
        }
    trial = {
        "initial_yaw_rad": {"left": math.radians(0.3), "right": math.radians(-0.25)},
        "pixel_sigma_px": 0.5, "mc_seed": 17,
        "calibration_run_id": "G0-P0-R01", "evaluation_run_id": "G0-V0-R01",
    }
    points = []
    for index in range(24):
        north = 300.0 + (index % 6) * 16.0
        east = -7.0 + (index % 8) * 2.0
        up = north * math.tan(math.radians(20.0)) - 4.0 + (index % 5) * 2.0
        points.append([east, north, up])
    points = np.asarray(points)
    rotations = {side: rotation_world_from_optical(0.0, 0.0, 0.0) for side in centers}
    rows = []
    for index, point in enumerate(points):
        pixels = {}
        for side in ("left", "right"):
            camera = rotations[side].T @ (point - centers[side])
            pixels[side] = [INTRINSICS["fx_px"] * camera[0] / camera[2] + INTRINSICS["cx_px"],
                            INTRINSICS["fy_px"] * camera[1] / camera[2] + INTRINSICS["cy_px"]]
        rows.append({"pair_id": f"pair-{index:04d}", "left_detection_status": "DETECTED",
                     "right_detection_status": "DETECTED", "left_uv_px": pixels["left"],
                     "right_uv_px": pixels["right"]})
    rtk = [{"position_world_enu_m": point.tolist(),
            "covariance_world_enu_m2": np.diag([1e-4, 1e-4, 4e-4]).tolist()}
           for point in points]
    return sensor, trial, rows, rtk


def test_frozen_rotation_order_is_optical_rdf_to_world_enu():
    nominal = rotation_world_from_optical(0.0, 0.0, 0.0)
    np.testing.assert_allclose(
        nominal[:, 2],
        [0.0, math.cos(math.radians(20.0)), math.sin(math.radians(20.0))],
        atol=1e-12,
    )
    east_heading = rotation_world_from_optical(0.0, 0.0, math.radians(1.0))
    assert east_heading[0, 2] > nominal[0, 2]


@pytest.mark.parametrize("method", ["A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint"])
def test_rpy_solver_keeps_centers_fixed_and_reports_latent_dofs(method):
    sensor, trial, pairs, rtk = _fixture()
    result = solve_calibration(
        method=method, attitude_variant="RPY_IMU", sensor_observations=sensor,
        trial_config=trial, image_pairs=pairs, intrinsics=INTRINSICS,
        drone_rtk_observations=None if method == "C_no_drone_rtk_joint" else rtk,
    )
    assert result["camera_dof"] == 6
    assert result["latent_dof"] == (0 if method == "A_rtk_independent" else 3 * len(pairs))
    assert result["fixed_camera_centers_world_m"]["left"] == sensor["cameras"]["left"]["optical_center_rtk_position_world_enu_m"]
    assert result["convergence_status"] == "CONVERGED"
    assert len(result["estimated_R_WC"]["left"]) == 3
    tolerance = 0.01 if method == "C_no_drone_rtk_joint" else 1e-4
    for side in ("left", "right"):
        assert np.linalg.norm(result["estimated_phi_theta_psi_rad"][side]) < tolerance


def test_yaw_only_fixes_measured_imu_angles_and_c_rejects_forbidden_inputs():
    sensor, trial, pairs, rtk = _fixture()
    result = solve_calibration(
        method="B_rtk_joint", attitude_variant="YAW_ONLY", sensor_observations=sensor,
        trial_config=trial, image_pairs=pairs, intrinsics=INTRINSICS,
        drone_rtk_observations=rtk,
    )
    assert result["camera_dof"] == 2
    for side in ("left", "right"):
        assert result["estimated_phi_theta_psi_rad"][side][:2] == sensor["cameras"][side]["camera_imu_roll_pitch_mean_optical_rad"]
    with pytest.raises(ValueError, match="forbids drone RTK"):
        solve_calibration(
            method="C_no_drone_rtk_joint", attitude_variant="YAW_ONLY",
            sensor_observations=sensor, trial_config=trial, image_pairs=pairs,
            intrinsics=INTRINSICS, drone_rtk_observations=rtk,
        )
    with pytest.raises(ValueError, match="forbidden truth"):
        solve_calibration(
            method="C_no_drone_rtk_joint", attitude_variant="RPY_IMU",
            sensor_observations=sensor, trial_config={**trial, "ground_truth": []},
            image_pairs=pairs, intrinsics=INTRINSICS,
        )


def test_fixed_pose_triangulation_returns_world_enu_point():
    sensor, _trial, pairs, _rtk = _fixture()
    centers = {side: sensor["cameras"][side]["optical_center_rtk_position_world_enu_m"]
               for side in ("left", "right")}
    rotations = {side: rotation_world_from_optical(0.0, 0.0, 0.0)
                 for side in ("left", "right")}
    from campaigns.stereo_v1_1.calibration_solver import triangulate_stereo_pixels

    reconstructed = triangulate_stereo_pixels(
        pairs[0]["left_uv_px"], pairs[0]["right_uv_px"],
        camera_centers_world_enu_m=centers,
        rotations_world_from_optical=rotations,
        intrinsics=INTRINSICS,
    )
    expected = np.array([-7.0, 300.0, 300.0 * math.tan(math.radians(20.0)) - 4.0])
    np.testing.assert_allclose(reconstructed, expected, atol=1e-5)


def test_joint_rtk_accepts_zero_covariance_as_fixed_enu_points():
    sensor, trial, pairs, rtk = _fixture()
    exact = [{**row, "covariance_world_enu_m2": np.zeros((3, 3)).tolist()}
             for row in rtk]
    result = solve_calibration(
        method="B_rtk_joint", attitude_variant="RPY_IMU",
        sensor_observations=sensor, trial_config=trial,
        image_pairs=pairs, intrinsics=INTRINSICS,
        drone_rtk_observations=exact,
    )
    assert result["convergence_status"] == "CONVERGED"
    assert result["latent_dof"] == 0
    assert result["optimizer_policy"]["method_B"].startswith("shared per-pair latent world ENU points")
