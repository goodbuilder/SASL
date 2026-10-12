#!/usr/bin/env python3
"""Truth-isolated camera attitude calibration for stereo campaign v1.1.

Input frames are world ENU for positions and optical RDF for pixels. The
rotation contract is exactly
``R_WC(phi,theta,psi)=Rz_world(-psi) @ R_WC_nominal @ Rx_optical(theta) @
Rz_optical(phi)``. Camera centers are measured inputs and never variables.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from scipy.optimize import least_squares


R_WC_NOMINAL = np.array([
    [1.0, 0.0, 0.0],
    [0.0, math.sin(math.radians(20.0)), math.cos(math.radians(20.0))],
    [0.0, -math.cos(math.radians(20.0)), math.sin(math.radians(20.0))],
], dtype=float)
METHODS = frozenset({"A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint"})
VARIANTS = frozenset({"RPY_IMU", "YAW_ONLY"})
CAMERA_ORDER = ("left", "right")
HUBER_F_SCALE = 1.345


def _finite_array(values, shape: tuple[int, ...], label: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must be finite with shape {shape}")
    return result


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rotation_world_from_optical(phi_roll_rad: float, theta_pitch_correction_rad: float,
                                psi_heading_rad: float) -> np.ndarray:
    """Return optical RDF → world ENU R_WC using the frozen v1.1 order."""
    return (_rz(-float(psi_heading_rad)) @ R_WC_NOMINAL
            @ _rx(float(theta_pitch_correction_rad)) @ _rz(float(phi_roll_rad)))


def _project(points_world: np.ndarray, center_world: np.ndarray,
             rotation_world_from_optical: np.ndarray,
             intrinsics: tuple[float, float, float, float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    fx, fy, cx, cy = intrinsics
    camera = (points_world - center_world) @ rotation_world_from_optical
    depth = camera[:, 2]
    safe_depth = np.where(depth > 1e-6, depth, 1e-6)
    uv = np.column_stack((fx * camera[:, 0] / safe_depth + cx,
                          fy * camera[:, 1] / safe_depth + cy))
    return uv, camera, depth


def _projection_jacobian_world(point_world: np.ndarray, center_world: np.ndarray,
                               rotation_world_from_optical: np.ndarray,
                               intrinsics: tuple[float, float, float, float]) -> np.ndarray:
    fx, fy, _, _ = intrinsics
    camera = rotation_world_from_optical.T @ (point_world - center_world)
    x, y, z = camera
    z = max(float(z), 1e-6)
    jac_camera = np.array([[fx / z, 0.0, -fx * x / (z * z)],
                           [0.0, fy / z, -fy * y / (z * z)]])
    return jac_camera @ rotation_world_from_optical.T


def _projection_jacobian_world_batch(points_world: np.ndarray, center_world: np.ndarray,
                                     rotation_world_from_optical: np.ndarray,
                                     intrinsics: tuple[float, float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    fx, fy, _, _ = intrinsics
    camera = (points_world - center_world) @ rotation_world_from_optical
    depth = camera[:, 2]
    safe = np.maximum(depth, 1.0)
    jac_camera = np.zeros((len(points_world), 2, 3), dtype=float)
    jac_camera[:, 0, 0] = fx / safe
    jac_camera[:, 0, 2] = -fx * camera[:, 0] / (safe * safe)
    jac_camera[:, 1, 1] = fy / safe
    jac_camera[:, 1, 2] = -fy * camera[:, 1] / (safe * safe)
    jac_world = jac_camera @ rotation_world_from_optical.T
    invalid = depth <= 1.0
    jac_world[invalid] = 0.0
    return jac_world, depth


def _whiten(residual: np.ndarray, covariance: np.ndarray) -> np.ndarray:
    try:
        lower = np.linalg.cholesky(covariance)
    except np.linalg.LinAlgError:
        lower = np.linalg.cholesky(covariance + np.eye(covariance.shape[0]) * 1e-12)
    return np.linalg.solve(lower, residual)


def _huber_weights(standardized: np.ndarray) -> np.ndarray:
    absolute = np.abs(standardized)
    return np.where(absolute <= HUBER_F_SCALE, 1.0,
                    HUBER_F_SCALE / np.maximum(absolute, 1e-12))


def _camera_inputs(sensor_observations: dict, trial_config: dict,
                   variant: str) -> dict[str, dict[str, np.ndarray | float]]:
    if set(sensor_observations.get("cameras", {})) != set(CAMERA_ORDER):
        raise ValueError("sensor observations must contain exactly left and right cameras")
    initial_yaw = trial_config.get("initial_yaw_rad")
    if not isinstance(initial_yaw, dict) or set(initial_yaw) != set(CAMERA_ORDER):
        raise ValueError("trial_config.initial_yaw_rad must contain left and right")
    result = {}
    for side in CAMERA_ORDER:
        camera = sensor_observations["cameras"][side]
        center = _finite_array(camera["optical_center_rtk_position_world_enu_m"], (3,), f"{side} measured center")
        imu_mean = _finite_array(camera["camera_imu_roll_pitch_mean_optical_rad"], (2,), f"{side} camera IMU mean")
        imu_cov = _finite_array(camera["camera_imu_roll_pitch_covariance_optical_rad2"], (2, 2), f"{side} camera IMU covariance")
        min_imu_eigenvalue = float(np.min(np.linalg.eigvalsh(imu_cov)))
        if min_imu_eigenvalue < -1e-12:
            raise ValueError("camera-IMU covariance must be positive semidefinite")
        if variant == "RPY_IMU" and min_imu_eigenvalue <= 0:
            raise ValueError("RPY_IMU requires a finite positive camera-IMU covariance")
        result[side] = {"center": center, "imu_mean": imu_mean,
                        "imu_cov": imu_cov, "initial_yaw": float(initial_yaw[side])}
    return result


def _prepare_image_pairs(image_pairs: list[dict], intrinsics, max_pairs: int | None) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    usable = []
    for row in image_pairs:
        if row.get("left_detection_status") != "DETECTED" or row.get("right_detection_status") != "DETECTED":
            continue
        left = row.get("left_uv_px")
        right = row.get("right_uv_px")
        if left is None or right is None:
            continue
        left_uv = _finite_array(left, (2,), "left bbox center")
        right_uv = _finite_array(right, (2,), "right bbox center")
        width = row.get("image_width_px", 1280)
        height = row.get("image_height_px", 720)
        if not (0.0 <= left_uv[0] < width and 0.0 <= left_uv[1] < height
                and 0.0 <= right_uv[0] < width and 0.0 <= right_uv[1] < height):
            continue
        usable.append((row, left_uv, right_uv))
    if max_pairs is not None and len(usable) > max_pairs:
        indices = np.linspace(0, len(usable) - 1, max_pairs, dtype=int)
        usable = [usable[index] for index in indices]
    if len(usable) < 6:
        raise ValueError("at least six unique valid stereo pairs are required for calibration")
    return (np.stack([item[1] for item in usable]),
            np.stack([item[2] for item in usable]),
            [item[0] for item in usable])


def _pose_from_parameters(parameters: np.ndarray, side_index: int, variant: str,
                          camera_data: dict) -> tuple[float, float, float]:
    if variant == "RPY_IMU":
        offset = 3 * side_index
        return (float(parameters[offset]), float(parameters[offset + 1]),
                float(parameters[offset + 2]))
    return (float(camera_data["imu_mean"][0]), float(camera_data["imu_mean"][1]),
            float(parameters[side_index]))


def _initial_parameters(cameras: dict, variant: str) -> np.ndarray:
    if variant == "RPY_IMU":
        return np.array([value for side in CAMERA_ORDER for value in (
            *cameras[side]["imu_mean"], cameras[side]["initial_yaw"]
        )], dtype=float)
    return np.array([cameras[side]["initial_yaw"] for side in CAMERA_ORDER], dtype=float)


def _prior_residual(parameters: np.ndarray, cameras: dict, variant: str) -> list[float]:
    if variant != "RPY_IMU":
        return []
    residuals = []
    for index, side in enumerate(CAMERA_ORDER):
        phi, theta, _ = _pose_from_parameters(parameters, index, variant, cameras[side])
        mean = cameras[side]["imu_mean"]
        delta = np.array([wrap_angle(phi - mean[0]), theta - mean[1]])
        residuals.extend(_whiten(delta, cameras[side]["imu_cov"]))
    return residuals


def _rank_condition(jacobian: np.ndarray) -> tuple[int, float]:
    singular = np.linalg.svd(np.asarray(jacobian, dtype=float), compute_uv=False)
    if singular.size == 0 or singular[0] == 0:
        return 0, math.inf
    threshold = singular[0] * max(jacobian.shape) * np.finfo(float).eps
    rank = int(np.count_nonzero(singular > threshold))
    condition = math.inf if rank < jacobian.shape[1] else float(singular[0] / singular[-1])
    return rank, condition


def _result_covariance(jacobian: np.ndarray) -> np.ndarray:
    normal = np.asarray(jacobian, dtype=float).T @ np.asarray(jacobian, dtype=float)
    covariance = np.linalg.pinv(normal, rcond=1e-12)
    return np.nan_to_num(covariance, nan=0.0, posinf=1e12, neginf=-1e12)


def _json_number(value: float) -> float | None:
    number = float(value)
    return number if math.isfinite(number) else None


def _a_camera(side: str, side_index: int, parameters: np.ndarray,
              cameras: dict, variant: str, rtk_positions: np.ndarray,
              rtk_covariances: np.ndarray, observed_uv: np.ndarray,
              intrinsics, pixel_sigma: float) -> np.ndarray:
    data = cameras[side]
    phi, theta, psi = _pose_from_parameters(parameters, side_index, variant, data)
    rotation = rotation_world_from_optical(phi, theta, psi)
    predicted, _camera, depth = _project(rtk_positions, data["center"], rotation, intrinsics)
    jacobian, _depth_check = _projection_jacobian_world_batch(
        rtk_positions, data["center"], rotation, intrinsics,
    )
    covariance = jacobian @ rtk_covariances @ jacobian.transpose(0, 2, 1)
    covariance += np.eye(2)[None, :, :] * pixel_sigma ** 2
    lower = np.linalg.cholesky(covariance + np.eye(2)[None, :, :] * 1e-12)
    residual = predicted - observed_uv
    whitened = np.linalg.solve(lower, residual[..., None])[..., 0]
    invalid = depth <= 1.0
    whitened[invalid] = (1e4 + 10.0 * np.maximum(0.0, 1.0 - depth[invalid]))[:, None]
    residuals = list(whitened.reshape(-1))
    if variant == "RPY_IMU":
        mean = data["imu_mean"]
        residuals.extend(_whiten(np.array([wrap_angle(phi - mean[0]), theta - mean[1]]), data["imu_cov"]))
    return np.asarray(residuals, dtype=float)


def _ray_initial_points(left_uv, right_uv, cameras, rotations, intrinsics) -> np.ndarray:
    fx, fy, cx, cy = intrinsics
    directions = []
    projectors = []
    centers = []
    identity = np.eye(3)
    for side, pixels, rotation in zip(CAMERA_ORDER, (left_uv, right_uv), rotations):
        rays = np.column_stack(((pixels[:, 0] - cx) / fx,
                                (pixels[:, 1] - cy) / fy,
                                np.ones(len(pixels))))
        world = rays @ rotation.T
        world /= np.maximum(np.linalg.norm(world, axis=1)[:, None], 1e-12)
        projectors.append(identity[None, :, :] - world[:, :, None] * world[:, None, :])
        centers.append(cameras[side]["center"])
    matrix = projectors[0] + projectors[1]
    vector = (projectors[0] @ centers[0]) + (projectors[1] @ centers[1])
    return (np.linalg.pinv(matrix, rcond=1e-12) @ vector[..., None])[..., 0]


def _fit_latent_points(left_uv, right_uv, cameras, rotations, intrinsics,
                       pixel_sigma: float, rtk_positions=None, rtk_covariances=None) -> np.ndarray:
    """Variable-project each 3-D point with damped Gauss-Newton in ENU metres."""
    count = len(left_uv)
    has_rtk = rtk_positions is not None
    points = (np.array(rtk_positions, dtype=float, copy=True) if has_rtk
              else _ray_initial_points(left_uv, right_uv, cameras, rotations, intrinsics))
    if has_rtk:
        eigenvalues = np.linalg.eigvalsh(rtk_covariances)
        exact_rtk = np.max(np.abs(eigenvalues), axis=1) <= 1e-18
        uncertain_rtk = ~exact_rtk
        rtk_whitener = np.zeros_like(rtk_covariances)
        if np.any(uncertain_rtk):
            rtk_whitener[uncertain_rtk] = np.linalg.inv(
                np.linalg.cholesky(rtk_covariances[uncertain_rtk])
            )
    else:
        exact_rtk = np.zeros(count, dtype=bool)
        uncertain_rtk = np.zeros(count, dtype=bool)
        rtk_whitener = None
    identity = np.eye(3)[None, :, :]
    for _iteration in range(10):
        hessian = np.zeros((count, 3, 3), dtype=float)
        gradient = np.zeros((count, 3), dtype=float)
        for side, observed, rotation in zip(CAMERA_ORDER, (left_uv, right_uv), rotations):
            predicted, _camera, _depth = _project(points, cameras[side]["center"], rotation, intrinsics)
            jacobian, depth = _projection_jacobian_world_batch(
                points, cameras[side]["center"], rotation, intrinsics,
            )
            residual = (predicted - observed) / pixel_sigma
            scaled_jacobian = jacobian / pixel_sigma
            residual[depth <= 1.0] = 0.0
            weights = _huber_weights(residual)
            weights[depth <= 1.0] = 0.0
            hessian += np.einsum("nki,nk,nkj->nij", scaled_jacobian, weights, scaled_jacobian)
            gradient += np.einsum("nki,nk,nk->ni", scaled_jacobian, weights, residual)
        if np.any(uncertain_rtk):
            delta = points[uncertain_rtk] - rtk_positions[uncertain_rtk]
            whitener = rtk_whitener[uncertain_rtk]
            whitened = np.einsum("nij,nj->ni", whitener, delta)
            weights = _huber_weights(whitened)
            hessian[uncertain_rtk] += np.einsum("nki,nk,nkj->nij", whitener, weights, whitener)
            gradient[uncertain_rtk] += np.einsum("nki,nk,nk->ni", whitener, weights, whitened)
        damping = np.maximum(1e-10, np.trace(hessian, axis1=1, axis2=2) * 1e-12)
        damped = hessian + identity * damping[:, None, None]
        try:
            step = np.linalg.solve(damped, gradient[..., None])[..., 0]
        except np.linalg.LinAlgError:
            step = np.einsum("nij,nj->ni", np.linalg.pinv(damped, rcond=1e-12), gradient)
        finite = np.all(np.isfinite(step), axis=1)
        update = finite & uncertain_rtk if has_rtk else finite
        points[update] -= step[update]
        if has_rtk and np.any(exact_rtk):
            points[exact_rtk] = rtk_positions[exact_rtk]
    return points


def triangulate_stereo_pixels(left_uv_px, right_uv_px, *,
                              camera_centers_world_enu_m: dict,
                              rotations_world_from_optical: dict,
                              intrinsics: dict, pixel_sigma_px: float = 1.0) -> np.ndarray:
    """Reconstruct points from two optical-RDF rays into world ENU metres.

    This runtime/evaluation helper has no RTK or truth parameter. Camera
    positions and R_WC rotations are fixed inputs.
    """
    return triangulate_stereo_pairs(
        np.asarray(left_uv_px, dtype=float).reshape(1, 2),
        np.asarray(right_uv_px, dtype=float).reshape(1, 2),
        camera_centers_world_enu_m=camera_centers_world_enu_m,
        rotations_world_from_optical=rotations_world_from_optical,
        intrinsics=intrinsics,
        pixel_sigma_px=pixel_sigma_px,
    )[0]


def triangulate_stereo_pairs(left_uv_px, right_uv_px, *,
                             camera_centers_world_enu_m: dict,
                             rotations_world_from_optical: dict,
                             intrinsics: dict, pixel_sigma_px: float = 1.0) -> np.ndarray:
    """Vectorized two-view reconstruction of paired optical RDF pixels to ENU."""
    left = np.asarray(left_uv_px, dtype=float)
    right = np.asarray(right_uv_px, dtype=float)
    if left.ndim != 2 or left.shape[1] != 2 or right.shape != left.shape:
        raise ValueError("left/right pixel arrays must both have shape (N,2)")
    if not np.all(np.isfinite(left)) or not np.all(np.isfinite(right)):
        raise ValueError("paired pixels must be finite")
    centers = {}
    rotations = []
    for side in CAMERA_ORDER:
        centers[side] = {"center": _finite_array(camera_centers_world_enu_m[side], (3,), f"{side} ENU center")}
        rotation = _finite_array(rotations_world_from_optical[side], (3, 3), f"{side} R_WC")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or np.linalg.det(rotation) < 0.999:
            raise ValueError(f"{side} R_WC must be a proper rotation")
        rotations.append(rotation)
    intrinsics_tuple = tuple(float(intrinsics[key]) for key in ("fx_px", "fy_px", "cx_px", "cy_px"))
    sigma = float(pixel_sigma_px)
    if sigma <= 0 or not math.isfinite(sigma):
        raise ValueError("pixel_sigma_px must be finite and positive")
    return _fit_latent_points(left, right, centers, rotations, intrinsics_tuple, sigma)


def _joint_residual(parameters: np.ndarray, cameras: dict, variant: str,
                    left_uv: np.ndarray, right_uv: np.ndarray, intrinsics,
                    pixel_sigma: float, rtk_positions=None,
                    rtk_covariances=None) -> np.ndarray:
    angles = [_pose_from_parameters(parameters, index, variant, cameras[side])
              for index, side in enumerate(CAMERA_ORDER)]
    rotations = [rotation_world_from_optical(*angle) for angle in angles]
    points = _fit_latent_points(left_uv, right_uv, cameras, rotations, intrinsics,
                                pixel_sigma, rtk_positions, rtk_covariances)
    residuals = []
    for side, observed, rotation in zip(CAMERA_ORDER, (left_uv, right_uv), rotations):
        predicted, _camera, depth = _project(points, cameras[side]["center"], rotation, intrinsics)
        residual = (predicted - observed) / pixel_sigma
        residual[depth <= 1e-6] = 1e4
        residuals.extend(residual.reshape(-1))
    if rtk_positions is not None:
        eigenvalues = np.linalg.eigvalsh(rtk_covariances)
        uncertain = np.max(np.abs(eigenvalues), axis=1) > 1e-18
        if np.any(uncertain):
            lower = np.linalg.cholesky(rtk_covariances[uncertain])
            whitened = np.linalg.solve(
                lower, (points[uncertain] - rtk_positions[uncertain])[..., None]
            )[..., 0]
            residuals.extend(whitened.reshape(-1))
    residuals.extend(_prior_residual(parameters, cameras, variant))
    return np.asarray(residuals, dtype=float)


def _full_covariance(parameter_covariance: np.ndarray, variant: str) -> list[list[float | None]]:
    full: list[list[float | None]] = [[None for _ in range(6)] for _ in range(6)]
    if variant == "RPY_IMU":
        indices = list(range(6))
    else:
        indices = [2, 5]
    for row, full_row in enumerate(indices):
        for column, full_column in enumerate(indices):
            full[full_row][full_column] = float(parameter_covariance[row, column])
    return full


def solve_calibration(*, method: str, attitude_variant: str,
                      sensor_observations: dict, trial_config: dict,
                      image_pairs: list[dict], intrinsics: dict,
                      drone_rtk_observations: list[dict] | None = None,
                      max_pairs: int | None = None) -> dict:
    """Solve one calibration using only explicit estimator inputs.

    GT, physical camera truth, and commanded references are not arguments. C
    rejects RTK observations by construction. A marginalizes independent RTK
    uncertainty into each camera's image residual; B uses shared latent ENU
    points with a 3-D RTK covariance term; C uses shared latent points from
    the two image rays only.
    """
    if method not in METHODS:
        raise ValueError(f"unsupported calibration method: {method}")
    if attitude_variant not in VARIANTS:
        raise ValueError(f"unsupported attitude variant: {attitude_variant}")
    if method == "C_no_drone_rtk_joint" and drone_rtk_observations is not None:
        raise ValueError("method C input allowlist forbids drone RTK")
    if method in ("A_rtk_independent", "B_rtk_joint") and drone_rtk_observations is None:
        raise ValueError(f"{method} requires noisy drone RTK observations")
    if any(any(term in key.lower() for term in ("gt", "truth", "reference")) for key in trial_config):
        raise ValueError("trial_config contains a forbidden truth/reference field")
    allowed_trial_fields = {
        "schema_version",
        "initial_yaw_rad", "pixel_sigma_px", "mc_seed", "calibration_run_id",
        "evaluation_run_id", "yaw_case", "trial_index", "sample_scope",
    }
    unknown_trial_fields = set(trial_config) - allowed_trial_fields
    if unknown_trial_fields:
        raise ValueError(f"trial_config exceeds estimator input allowlist: {sorted(unknown_trial_fields)}")
    intrinsics_tuple = tuple(float(intrinsics[key]) for key in ("fx_px", "fy_px", "cx_px", "cy_px"))
    if not all(math.isfinite(value) for value in intrinsics_tuple) or intrinsics_tuple[0] <= 0 or intrinsics_tuple[1] <= 0:
        raise ValueError("intrinsics must be finite positive focal lengths")
    pixel_sigma = float(trial_config.get("pixel_sigma_px", 1.0))
    if not math.isfinite(pixel_sigma) or pixel_sigma <= 0:
        raise ValueError("pixel_sigma_px must be finite and positive")
    cameras = _camera_inputs(sensor_observations, trial_config, attitude_variant)
    left_uv, right_uv, selected_pairs = _prepare_image_pairs(image_pairs, intrinsics_tuple, max_pairs)
    rtk_positions = rtk_covariances = None
    if drone_rtk_observations is not None:
        if len(drone_rtk_observations) != len(selected_pairs):
            raise ValueError("drone RTK rows must be aligned one-to-one with selected image pairs")
        rtk_positions = np.stack([_finite_array(row["position_world_enu_m"], (3,), "drone RTK ENU position")
                                  for row in drone_rtk_observations])
        rtk_covariances = np.stack([_finite_array(row["covariance_world_enu_m2"], (3, 3), "drone RTK ENU covariance")
                                    for row in drone_rtk_observations])
        if any(np.min(np.linalg.eigvalsh(covariance)) < -1e-12 for covariance in rtk_covariances):
            raise ValueError("drone RTK covariance must be positive semidefinite")
        if any(np.max(np.abs(np.linalg.eigvalsh(covariance))) > 1e-18
               and np.min(np.linalg.eigvalsh(covariance)) <= 0
               for covariance in rtk_covariances):
            raise ValueError("nonzero drone RTK covariance must be positive definite")

    x0 = _initial_parameters(cameras, attitude_variant)
    fit_records = []
    if method == "A_rtk_independent":
        estimated = {}
        parameter_covariance = np.zeros((len(x0), len(x0)), dtype=float)
        fit_offset = 0
        for side_index, side in enumerate(CAMERA_ORDER):
            camera_x0 = (x0[3 * side_index:3 * side_index + 3]
                         if attitude_variant == "RPY_IMU" else x0[side_index:side_index + 1])

            def residual(local_parameters):
                if attitude_variant == "RPY_IMU":
                    joint = x0.copy()
                    joint[3 * side_index:3 * side_index + 3] = local_parameters
                else:
                    joint = x0.copy()
                    joint[side_index] = local_parameters[0]
                return _a_camera(side, side_index, joint, cameras, attitude_variant,
                                 rtk_positions, rtk_covariances,
                                 left_uv if side == "left" else right_uv,
                                 intrinsics_tuple, pixel_sigma)

            fit = least_squares(residual, camera_x0, loss="huber", f_scale=HUBER_F_SCALE,
                                max_nfev=500, x_scale="jac")
            rank, condition = _rank_condition(fit.jac)
            local_covariance = _result_covariance(fit.jac)
            if attitude_variant == "RPY_IMU":
                offset = 3 * side_index
                parameter_covariance[offset:offset + 3, offset:offset + 3] = local_covariance
                estimated[side] = tuple(float(value) for value in fit.x)
            else:
                parameter_covariance[side_index, side_index] = local_covariance[0, 0]
                estimated[side] = (float(cameras[side]["imu_mean"][0]),
                                   float(cameras[side]["imu_mean"][1]), float(fit.x[0]))
            fit_records.append({"success": bool(fit.success), "message": str(fit.message),
                                "rank": rank, "parameter_count": len(fit.x),
                                "condition_number": _json_number(condition), "cost": _json_number(fit.cost),
                                "evaluations": int(fit.nfev)})
    else:
        def residual(parameters):
            return _joint_residual(parameters, cameras, attitude_variant,
                                   left_uv, right_uv, intrinsics_tuple,
                                   pixel_sigma, rtk_positions, rtk_covariances)

        fit = least_squares(residual, x0, loss="huber", f_scale=HUBER_F_SCALE,
                            max_nfev=500, x_scale="jac")
        rank, condition = _rank_condition(fit.jac)
        parameter_covariance = _result_covariance(fit.jac)
        estimated = {side: _pose_from_parameters(fit.x, index, attitude_variant, cameras[side])
                     for index, side in enumerate(CAMERA_ORDER)}
        fit_records.append({"success": bool(fit.success), "message": str(fit.message),
                            "rank": rank, "parameter_count": len(fit.x),
                            "condition_number": _json_number(condition), "cost": _json_number(fit.cost),
                            "evaluations": int(fit.nfev)})

    estimated_rotations = {
        side: rotation_world_from_optical(*estimated[side]).tolist() for side in CAMERA_ORDER
    }
    initial_rotations = {}
    for side_index, side in enumerate(CAMERA_ORDER):
        initial_angles = _pose_from_parameters(x0, side_index, attitude_variant, cameras[side])
        initial_rotations[side] = rotation_world_from_optical(*initial_angles).tolist()
    ranks_ok = all(item["rank"] == item["parameter_count"] for item in fit_records)
    optimizers_ok = all(item["success"] for item in fit_records)
    status = "CONVERGED" if optimizers_ok and ranks_ok else (
        "RANK_DEFICIENT" if not ranks_ok else "OPTIMIZER_FAILED"
    )
    condition_values = [item["condition_number"] for item in fit_records]
    condition = max((value if value is not None else math.inf for value in condition_values), default=math.inf)
    covariance = _full_covariance(parameter_covariance, attitude_variant)
    return {
        "schema_version": "stereo_v1_1_calibration_result_v1",
        "method": method,
        "attitude_variant": attitude_variant,
        "camera_dof": 6 if attitude_variant == "RPY_IMU" else 2,
        "latent_dof": (
            0 if method == "A_rtk_independent" else
            3 * len(selected_pairs) if method == "C_no_drone_rtk_joint" else
            3 * int(np.count_nonzero(np.max(np.abs(np.linalg.eigvalsh(rtk_covariances)), axis=1) > 1e-18))
        ),
        "fixed_camera_centers_world_m": {
            side: cameras[side]["center"].tolist() for side in CAMERA_ORDER
        },
        "imu_prior_means_rad": {
            side: cameras[side]["imu_mean"].tolist() for side in CAMERA_ORDER
        },
        "imu_prior_covariance_rad2": {
            side: cameras[side]["imu_cov"].tolist() for side in CAMERA_ORDER
        },
        "initial_R_WC": initial_rotations,
        "estimated_R_WC": estimated_rotations,
        "estimated_phi_theta_psi_rad": {
            side: [float(value) for value in estimated[side]] for side in CAMERA_ORDER
        },
        "attitude_covariance_rad2": covariance,
        "covariance_estimated_parameter_order": (
            [f"{side}_{axis}" for side in CAMERA_ORDER for axis in ("phi", "theta", "psi")]
            if attitude_variant == "RPY_IMU" else ["left_psi", "right_psi"]
        ),
        "conditioning": {
            "condition_number": _json_number(condition),
            "fit_components": fit_records,
            "rank_deficient": not ranks_ok,
        },
        "convergence_status": status,
        "optimizer_policy": {
            "loss": "Huber", "f_scale_standardized_residual": HUBER_F_SCALE,
            "pixel_sigma_px": pixel_sigma,
            "yaw_prior_penalty": "none",
            "camera_translation_variables": 0,
            "method_A": "camera-independent fit; drone RTK covariance marginalized to pixel covariance by first-order projection Jacobian",
            "method_B": "shared per-pair latent world ENU points with the same noisy drone RTK observation and covariance",
            "method_C": "shared per-pair latent world ENU points from stereo images only; no drone RTK, GT, or commanded reference",
        },
        "observation_count": len(selected_pairs),
        "selected_pair_ids": [str(row.get("pair_id", "")) for row in selected_pairs],
        "calibration_run_id": trial_config.get("calibration_run_id"),
        "evaluation_run_id": trial_config.get("evaluation_run_id"),
        "mc_seed": trial_config.get("mc_seed"),
    }
