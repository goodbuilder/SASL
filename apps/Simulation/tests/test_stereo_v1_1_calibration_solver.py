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


def test_block_loss_keeps_large_sensor_errors_quadratic_and_images_huber():
    from campaigns.stereo_v1_1.calibration_solver import _image_huber_prior_quadratic_loss, HUBER_F_SCALE
    delta = HUBER_F_SCALE
    residual = np.array([0.5, 4.0, 4.0, 10.0])
    rho = _image_huber_prior_quadratic_loss(2)((residual / delta) ** 2)
    np.testing.assert_allclose(delta ** 2 * rho[0],
                               [0.25, 2 * delta * 4 - delta ** 2, 16, 100])
    np.testing.assert_allclose(rho[1, 2:], [1, 1])
    np.testing.assert_allclose(rho[2, 2:], [0, 0])
    assert rho[1, 1] < 1


def test_gaussian_rtk_inner_normal_terms_match_outer_quadratic_loss():
    from campaigns.stereo_v1_1.calibration_solver import (
        _gaussian_rtk_normal_terms, _image_huber_prior_quadratic_loss, HUBER_F_SCALE)
    whitener = np.array([[[2., 0., 0.], [1., 3., 0.], [0., 0., 4.]]])
    displacement = np.array([[5., -3., 2.]])
    hessian, gradient = _gaussian_rtk_normal_terms(whitener, displacement)
    np.testing.assert_allclose(hessian[0], [[5, 3, 0], [3, 9, 0], [0, 0, 16]])
    np.testing.assert_allclose(gradient[0], [16, -12, 32])
    white = whitener[0] @ displacement[0]
    rho = _image_huber_prior_quadratic_loss(4)(
        np.r_[np.zeros(4), (white / HUBER_F_SCALE) ** 2])
    assert HUBER_F_SCALE ** 2 * sum(rho[0, 4:]) == pytest.approx(180)
    np.testing.assert_allclose(rho[1, 4:], np.ones(3))


def test_observability_reports_prior_rank_lifting_without_changing_data_rank():
    from campaigns.stereo_v1_1.calibration_solver import _prior_observability
    # Image block observes yaw only; two independent IMU rows observe roll/pitch.
    jacobian = np.array([[0., 0., 2.], [0., 0., 3.], [4., 0., 0.], [0., 5., 0.]])
    result = _prior_observability(jacobian, 2, 'camera attitude radians')
    import json
    json.dumps(result, allow_nan=False)
    assert result['data_only']['rank'] == 1
    assert result['data_only']['condition_number'] is None
    assert result['data_only']['rank_deficient'] is True
    assert result['imu_included']['rank_deficient'] is False
    assert result['imu_included']['rank'] == 3
    np.testing.assert_allclose(result['imu_included']['singular_values_per_rad'], [5, 4, np.sqrt(13)])
    assert result['imu_included']['condition_number'] == pytest.approx(5 / np.sqrt(13))
    yaw = _prior_observability(jacobian[:2, 2:], 0, 'yaw only; measured roll/pitch fixed')
    assert yaw['data_only'] == yaw['imu_included']


def test_attitude_correlation_positive_covariance_and_fixed_axes_are_explicit():
    import json
    from campaigns.stereo_v1_1.calibration_solver import _attitude_correlation, _full_covariance
    covariance = np.diag([4., 9., 16., 25., 36., 49.])
    covariance[0, 1] = covariance[1, 0] = 3.
    result = _attitude_correlation(covariance.tolist(), rank_deficient=False)
    assert result['status'] == 'DEFINED'
    assert result['parameter_order'] == ['left_phi','left_theta','left_psi','right_phi','right_theta','right_psi']
    assert result['matrix'][0][1] == pytest.approx(0.5)
    np.testing.assert_allclose(np.diag(result['matrix']), np.ones(6))
    yaw = _attitude_correlation(_full_covariance(np.array([[4., -3.], [-3., 9.]]), 'YAW_ONLY'), rank_deficient=False)
    assert yaw['matrix'][2][5] == pytest.approx(-0.5)
    assert yaw['matrix'][0][0] is None
    assert yaw['undefined_variance_axes'] == ['left_phi','left_theta','right_phi','right_theta']
    assert yaw['status'] == 'PARTIALLY_DEFINED'
    zero = _attitude_correlation(np.diag([0.,1.,1.,1.,1.,1.]).tolist(), rank_deficient=False)
    assert all(value is None for value in zero['matrix'][0])
    for payload in (result, yaw, zero): json.dumps(payload, allow_nan=False)


def test_attitude_correlation_rank_deficiency_and_nonfinite_are_json_safe():
    import json
    from campaigns.stereo_v1_1.calibration_solver import _attitude_correlation
    for rank_deficient, expected in [(True, 'UNDEFINED_RANK_DEFICIENT'), (False, 'INVALID_NONFINITE_COVARIANCE')]:
        covariance = np.eye(6)
        if not rank_deficient: covariance[0, 0] = np.nan
        result = _attitude_correlation(covariance.tolist(), rank_deficient=rank_deficient)
        assert result['status'] == expected
        assert all(value is None for row in result['matrix'] for value in row)
        json.dumps(result, allow_nan=False)
