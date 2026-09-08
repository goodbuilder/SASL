from __future__ import annotations

from math import isinf, sqrt
from random import Random

import pytest

from Workstation.workstation_core.optical_pose import (
    POSE_DOF_NAMES,
    CameraPoseNed,
    PoseDofMask,
    PoseParametersNed,
    geometric_horizon_dip_deg,
    geometric_horizon_dip_rad,
    opencv_pnp_from_camera_pose_ned,
)
from Workstation.workstation_core.pose_solvers import (
    NORMALIZED_IDEAL,
    PNP_RANSAC_SEED_SEMANTICS,
    ConstrainedPoseLeastSquaresResult,
    JacobianDiagnostics,
    MountainRotationConfig,
    PnpCorrespondences,
    PnpEnsembleConfig,
    PnpEnsembleResult,
    PnpPoseCandidate,
    PoseLeastSquaresConfig,
    analyze_jacobian,
    analyze_object_point_geometry,
    fit_normalized_horizon_line,
    fixed_center_orientation_ray_residuals,
    mountain_sampson_residuals_normalized,
    numerical_pose_jacobian,
    pose_reprojection_residuals,
    solve_constrained_pose_least_squares,
    solve_mountain_relative_rotation_robust,
    solve_pnp_ensemble,
    solve_spherical_horizon_roll_pitch,
)

import cv2
import numpy as np


def _matvec(matrix, vector):
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))


def _add(left, right):
    return tuple(a + b for a, b in zip(left, right))


def _scale(vector, scalar):
    return tuple(value * scalar for value in vector)


def _synthetic_raw_correspondences(*, add_outliers: bool = False):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(12.0, -8.0, 2.5),
        roll_deg=4.0,
        pitch_deg=-7.0,
        yaw_deg=21.0,
    )
    camera_points = np.asarray(
        [
            [-12.0, -8.0, 80.0],
            [11.0, -5.0, 95.0],
            [-8.0, 9.0, 110.0],
            [14.0, 12.0, 125.0],
            [3.0, -14.0, 145.0],
            [-17.0, 3.0, 165.0],
            [19.0, 1.0, 180.0],
            [-2.0, 16.0, 205.0],
            [8.0, -19.0, 230.0],
            [-21.0, -3.0, 250.0],
            [23.0, 15.0, 270.0],
            [-5.0, -22.0, 295.0],
        ],
        dtype=np.float64,
    )
    rotation_ned_from_camera = np.asarray(pose.rotation_ned_from_camera_rdf)
    center = np.asarray(pose.center_ned_m)
    object_points = (rotation_ned_from_camera @ camera_points.T).T + center
    intrinsics = np.asarray(
        ((1600.0, 0.0, 640.0), (0.0, 1580.0, 360.0), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )
    distortion = np.asarray((0.012, -0.006, 0.0003, -0.0002, 0.001), dtype=np.float64)
    rotation_camera_from_ned, translation_camera_from_ned = opencv_pnp_from_camera_pose_ned(pose)
    rvec, _jacobian = cv2.Rodrigues(np.asarray(rotation_camera_from_ned))
    image_points, _projection_jacobian = cv2.projectPoints(
        object_points,
        rvec,
        np.asarray(translation_camera_from_ned).reshape(3, 1),
        intrinsics,
        distortion,
    )
    image_points = image_points.reshape(-1, 2)
    random = Random(20260813)
    image_points += np.asarray(
        [(random.gauss(0.0, 0.08), random.gauss(0.0, 0.08)) for _point in image_points]
    )
    if add_outliers:
        image_points[1] += (90.0, -70.0)
        image_points[9] += (-80.0, 60.0)
    correspondences = PnpCorrespondences.from_raw_camera_pixels(
        object_points_ned_m=object_points,
        image_points_px=image_points,
        camera_matrix=intrinsics,
        distortion_coefficients=distortion,
    )
    return pose, correspondences


def test_pnp_input_space_prevents_double_undistort_and_rejects_fake_methods() -> None:
    objects = ((0.0, 0.0, 10.0), (1.0, 0.0, 10.0), (0.0, 1.0, 10.0), (1.0, 1.0, 11.0))
    images = ((0.0, 0.0), (0.1, 0.0), (0.0, 0.1), (0.09, 0.09))

    normalized = PnpCorrespondences.from_normalized_ideal(
        object_points_ned_m=objects,
        image_points_normalized=images,
    )

    assert normalized.image_space == NORMALIZED_IDEAL
    assert normalized.camera_matrix is None
    assert normalized.distortion_coefficients == ()
    with pytest.raises(ValueError, match="double-undistort"):
        PnpCorrespondences(
            object_points_ned_m=objects,
            image_points=images,
            image_space=NORMALIZED_IDEAL,
            camera_matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        )
    with pytest.raises(ValueError, match="intentionally not exposed"):
        PnpEnsembleConfig(methods=("EPNP", "DLS"))
    with pytest.raises(ValueError, match="intentionally not exposed"):
        PnpEnsembleConfig(methods=("UPNP",))


def test_seeded_pnp_ensemble_runs_three_real_solvers_and_two_independent_refinements() -> None:
    expected_pose, correspondences = _synthetic_raw_correspondences(add_outliers=True)
    config = PnpEnsembleConfig(
        ransac_seed=7717,
        ransac_iterations=700,
        ransac_reprojection_threshold_px=2.0,
    )

    first = solve_pnp_ensemble(correspondences, config=config)
    second = solve_pnp_ensemble(correspondences, config=config)

    assert len(first.failures) == 1
    assert first.failures[0][0] == "IPPE/NONE"
    assert "coplanar" in first.failures[0][1]
    assert len(first.candidates) == 10
    assert {candidate.solver for candidate in first.candidates} == {"ITERATIVE", "EPNP", "SQPNP", "ROBUST_LSQ"}
    assert sum(1 for candidate in first.candidates if candidate.solver == "ROBUST_LSQ") == 1
    assert {candidate.refinement for candidate in first.candidates} == {"NONE", "LM", "VVS"}
    assert all("DLS" not in candidate.candidate_id and "UPNP" not in candidate.candidate_id for candidate in first.candidates)
    assert first.ransac_seed == 7717
    assert first.seed_semantics == PNP_RANSAC_SEED_SEMANTICS
    assert first.object_geometry.classification == "volumetric"
    assert [candidate.candidate_id for candidate in first.candidates] == [
        candidate.candidate_id for candidate in second.candidates
    ]
    for left, right in zip(first.candidates, second.candidates):
        assert left.inlier_indices == right.inlier_indices
        assert left.pose_ned.center_ned_m == pytest.approx(right.pose_ned.center_ned_m, abs=1.0e-12)

    accurate = min(
        first.candidates,
        key=lambda candidate: sqrt(
            sum(
                (actual - expected) ** 2
                for actual, expected in zip(candidate.pose_ned.center_ned_m, expected_pose.center_ned_m)
            )
        ),
    )
    center_error = sqrt(
        sum(
            (actual - expected) ** 2
            for actual, expected in zip(accurate.pose_ned.center_ned_m, expected_pose.center_ned_m)
        )
    )
    assert center_error < 0.25
    assert accurate.positive_depth_ratio == pytest.approx(1.0)
    assert accurate.jacobian_diagnostics.parameter_names == POSE_DOF_NAMES
    assert accurate.jacobian_diagnostics.rank == 6


def test_normalized_ideal_pnp_uses_implicit_identity_intrinsics() -> None:
    expected_pose, raw = _synthetic_raw_correspondences()
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    raw_points = np.asarray(raw.image_points, dtype=np.float64).reshape(-1, 1, 2)
    normalized_points = cv2.undistortPoints(
        raw_points,
        np.asarray(raw.camera_matrix),
        np.asarray(raw.distortion_coefficients),
    ).reshape(-1, 2)
    normalized = PnpCorrespondences.from_normalized_ideal(
        object_points_ned_m=raw.object_points_ned_m,
        image_points_normalized=normalized_points,
    )

    result = solve_pnp_ensemble(
        normalized,
        config=PnpEnsembleConfig(
            methods=("EPNP", "SQPNP"),
            refinements=(),
            use_ransac=False,
        ),
    )

    assert result.failures == ()
    assert len(result.candidates) == 2
    assert all(candidate.residual_unit == "normalized" for candidate in result.candidates)
    best = result.best_candidate
    assert best is not None
    assert best.pose_ned.center_ned_m == pytest.approx(expected_pose.center_ned_m, abs=0.15)


def _synthetic_pnp_candidate(
    *,
    candidate_id: str,
    reprojection_p95: float,
    reprojection_rmse: float,
    positive_depth_ratio: float,
) -> PnpPoseCandidate:
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0), roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0
    )
    diagnostics = JacobianDiagnostics(
        parameter_names=POSE_DOF_NAMES,
        parameter_scales=tuple(1.0 for _name in POSE_DOF_NAMES),
        residual_count=8,
        parameter_count=6,
        rank=6,
        singular_values=(6.0, 5.0, 4.0, 3.0, 2.0, 1.0),
        rank_tolerance=1.0e-8,
        condition_number=6.0,
        covariance=None,
        nullspace_basis=(),
    )
    return PnpPoseCandidate(
        candidate_id=candidate_id,
        solver="ITERATIVE",
        refinement="NONE",
        pose_ned=pose,
        inlier_indices=(0, 1, 2, 3),
        reprojection_rmse=reprojection_rmse,
        reprojection_median=reprojection_rmse,
        reprojection_p95=reprojection_p95,
        residual_unit="px",
        positive_depth_ratio=positive_depth_ratio,
        jacobian_diagnostics=diagnostics,
        ransac_seed=None,
        seed_semantics="not_used",
    )


def test_best_candidate_never_prefers_a_flipped_pose_over_a_valid_one() -> None:
    """A point-behind-camera pose is geometrically impossible, not just noisier.

    Regression for a real bug: ``best_candidate`` used to rank purely by
    reprojection error, so a candidate with most points behind the camera
    (a mirror/flipped PnP solution -- a known failure mode on near-planar
    or noisy object geometry) could win over a fully valid one just for
    having a smaller residual. ``positive_depth_ratio`` was already computed
    per candidate but never consulted when picking the winner.
    """

    geometry = analyze_object_point_geometry(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    )
    flipped = _synthetic_pnp_candidate(
        candidate_id="EPNP/NONE",
        reprojection_p95=0.5,
        reprojection_rmse=0.3,
        positive_depth_ratio=0.25,
    )
    valid = _synthetic_pnp_candidate(
        candidate_id="ITERATIVE/NONE",
        reprojection_p95=1.2,
        reprojection_rmse=0.9,
        positive_depth_ratio=1.0,
    )
    result = PnpEnsembleResult(
        candidates=(flipped, valid),
        failures=(),
        object_geometry=geometry,
        image_space="raw_camera_pixels",
        ransac_seed=None,
        seed_semantics="not_used",
    )

    best = result.best_candidate

    assert best is not None
    assert best.candidate_id == "ITERATIVE/NONE"


def test_best_candidate_still_returns_least_bad_option_when_all_are_flipped() -> None:
    """No fully-valid candidate must not turn into a hard failure.

    The module's contract is "surface evidence, never silently block" --
    ``positive_depth_ratio`` stays visible on the returned candidate for the
    caller to act on, but ``best_candidate`` still returns its best guess.
    """

    geometry = analyze_object_point_geometry(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    )
    worse = _synthetic_pnp_candidate(
        candidate_id="EPNP/NONE",
        reprojection_p95=2.0,
        reprojection_rmse=1.5,
        positive_depth_ratio=0.25,
    )
    better = _synthetic_pnp_candidate(
        candidate_id="ITERATIVE/NONE",
        reprojection_p95=0.7,
        reprojection_rmse=0.5,
        positive_depth_ratio=0.5,
    )
    result = PnpEnsembleResult(
        candidates=(worse, better),
        failures=(),
        object_geometry=geometry,
        image_space="raw_camera_pixels",
        ransac_seed=None,
        seed_semantics="not_used",
    )

    best = result.best_candidate

    assert best is not None
    assert best.candidate_id == "ITERATIVE/NONE"
    assert best.positive_depth_ratio < 1.0


def test_normalized_horizon_fit_recovers_roll_pitch_and_rejects_degenerate_points() -> None:
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=3.25,
        pitch_deg=-2.75,
        yaw_deg=117.0,
    )
    rotation_camera_from_ned, _translation = opencv_pnp_from_camera_pose_ned(pose)
    gravity = _matvec(rotation_camera_from_ned, (0.0, 0.0, 1.0))
    random = Random(9)
    points = []
    for index in range(31):
        x = -0.8 + 1.6 * index / 30.0
        y = -(gravity[0] * x + gravity[2]) / gravity[1]
        points.append((x, y + random.gauss(0.0, 2.0e-5)))

    fit = fit_normalized_horizon_line(points)

    assert fit.roll_deg == pytest.approx(3.25, abs=0.01)
    assert fit.pitch_deg == pytest.approx(-2.75, abs=0.01)
    assert fit.rms_normalized < 3.0e-5
    assert not hasattr(fit, "yaw_deg")
    with pytest.raises(ValueError, match="degenerate"):
        fit_normalized_horizon_line(((0.1, 0.2), (0.1, 0.2), (0.1, 0.2)))


def test_fixed_center_orientation_residual_has_two_direction_constraints_per_ray() -> None:
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(10.0, 20.0, -3.0),
        roll_deg=2.0,
        pitch_deg=-4.0,
        yaw_deg=31.0,
    )
    rays = ((0.0, 0.0, 1.0), (0.2, -0.1, 1.0), (-0.15, 0.12, 1.0))
    targets = tuple(
        _add(pose.center_ned_m, _scale(_matvec(pose.rotation_ned_from_camera_rdf, ray), distance))
        for ray, distance in zip(rays, (500.0, 750.0, 900.0))
    )

    residual = fixed_center_orientation_ray_residuals(
        (2.0, -4.0, 31.0),
        camera_center_ned_m=pose.center_ned_m,
        observed_rays_camera_rdf=rays,
        target_points_ned_m=targets,
    )
    wrong = fixed_center_orientation_ray_residuals(
        (2.0, -4.0, 35.0),
        camera_center_ned_m=pose.center_ned_m,
        observed_rays_camera_rdf=rays,
        target_points_ned_m=targets,
    )

    assert max(abs(value) for value in residual) < 1.0e-12
    assert sqrt(sum(value * value for value in wrong)) > 0.05


def test_constrained_least_squares_changes_only_checked_yaw_and_east_and_reports_covariance() -> None:
    pytest.importorskip("scipy")
    initial = PoseParametersNed(1.0, 2.0, -30.0, 10.0, -20.0, 30.0)
    mask = PoseDofMask.from_names(("yaw_deg", "east_m"))

    def residual(parameters: PoseParametersNed):
        return (
            parameters.yaw_deg - 12.0,
            0.5 * (parameters.yaw_deg - 12.0),
            parameters.east_m - 40.0,
            2.0 * (parameters.east_m - 40.0),
            0.01,
        )

    result = solve_constrained_pose_least_squares(
        initial,
        mask,
        residual,
        parameter_scales=(10.0, 10.0),
        config=PoseLeastSquaresConfig(loss="linear"),
    )

    assert isinstance(result, ConstrainedPoseLeastSquaresResult)
    assert result.success is True
    assert result.parameters.yaw_deg == pytest.approx(12.0, abs=1.0e-9)
    assert result.parameters.east_m == pytest.approx(40.0, abs=1.0e-9)
    assert result.parameters.roll_deg == initial.roll_deg
    assert result.parameters.pitch_deg == initial.pitch_deg
    assert result.parameters.north_m == initial.north_m
    assert result.parameters.down_m == initial.down_m
    assert result.jacobian_diagnostics.rank == 2
    assert result.jacobian_diagnostics.covariance is not None


def test_jacobian_rank_condition_covariance_and_nullspace_are_fail_closed() -> None:
    full = analyze_jacobian(
        ((1.0, 0.0), (0.0, 2.0), (1.0, 1.0)),
        parameter_names=("yaw_deg", "east_m"),
        parameter_scales=(10.0, 2.0),
        residual_variance=0.04,
    )
    deficient = analyze_jacobian(
        ((1.0, 1.0), (2.0, 2.0), (3.0, 3.0)),
        parameter_names=("yaw_deg", "east_m"),
        residual_variance=0.04,
    )

    assert full.full_rank is True
    assert full.rank == 2
    assert full.condition_number < 10.0
    assert full.covariance is not None
    assert deficient.rank == 1
    assert deficient.full_rank is False
    assert isinf(deficient.condition_number)
    assert deficient.covariance is None
    assert len(deficient.nullspace_basis) == 1


def test_near_collinear_and_planar_object_geometry_are_not_called_volumetric() -> None:
    near_collinear = tuple(
        (float(index), 1.0e-9 * index * index, -2.0e-9 * index)
        for index in range(8)
    )
    planar = (
        (0.0, 0.0, 5.0),
        (1.0, 0.0, 5.0),
        (0.0, 1.0, 5.0),
        (1.0, 1.0, 5.0),
        (0.5, 0.2, 5.0),
    )
    volumetric = planar + ((0.3, 0.4, 6.0),)

    line_result = analyze_object_point_geometry(near_collinear, relative_rank_tolerance=1.0e-6)
    plane_result = analyze_object_point_geometry(planar)
    volume_result = analyze_object_point_geometry(volumetric)

    assert line_result.classification == "near_collinear"
    assert line_result.rank == 1
    assert isinf(line_result.condition_number)
    assert plane_result.classification == "planar"
    assert plane_result.rank == 2
    assert isinf(plane_result.condition_number)
    assert volume_result.classification == "volumetric"
    assert volume_result.rank == 3


def test_planar_pnp_result_preserves_global_ambiguity_warning_even_with_local_jacobian() -> None:
    np = pytest.importorskip("numpy")
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=2.0,
        pitch_deg=-3.0,
        yaw_deg=15.0,
    )
    camera_points = (
        (-10.0, -7.0, 100.0),
        (12.0, -7.0, 100.0),
        (-10.0, 8.0, 100.0),
        (12.0, 8.0, 100.0),
        (-3.0, 2.0, 100.0),
        (5.0, -1.0, 100.0),
    )
    objects = tuple(
        _add(pose.center_ned_m, _matvec(pose.rotation_ned_from_camera_rdf, point))
        for point in camera_points
    )
    images = tuple((point[0] / point[2], point[1] / point[2]) for point in camera_points)
    correspondences = PnpCorrespondences.from_normalized_ideal(
        object_points_ned_m=objects,
        image_points_normalized=images,
    )

    result = solve_pnp_ensemble(
        correspondences,
        config=PnpEnsembleConfig(
            methods=("ITERATIVE", "EPNP", "SQPNP"),
            refinements=(),
            use_ransac=False,
        ),
    )

    assert result.object_geometry.classification == "planar"
    assert result.object_geometry.rank == 2
    assert result.candidates
    assert all(candidate.jacobian_diagnostics.parameter_count == 6 for candidate in result.candidates)


def test_numerical_pose_jacobian_uses_only_selected_native_dofs() -> None:
    parameters = PoseParametersNed(1.0, 2.0, 3.0, 10.0, 20.0, 30.0)
    mask = PoseDofMask.from_names(("pitch_deg", "down_m"))

    jacobian = numerical_pose_jacobian(
        parameters,
        mask,
        lambda value: (
            2.0 * value.pitch_deg + value.down_m,
            value.pitch_deg - 3.0 * value.down_m,
        ),
    )

    np = pytest.importorskip("numpy")
    assert np.asarray(jacobian) == pytest.approx(np.asarray(((2.0, 1.0), (1.0, -3.0))), abs=1.0e-9)


def test_pose_reprojection_residual_is_zero_on_the_declared_raw_grid() -> None:
    pose, correspondences = _synthetic_raw_correspondences()
    parameters = PoseParametersNed(4.0, -7.0, 21.0, 12.0, -8.0, 2.5)

    residuals = pose_reprojection_residuals(parameters, correspondences)

    # Synthetic observations include only deterministic subpixel marking noise.
    assert sqrt(sum(value * value for value in residuals) / len(residuals)) < 0.15
    assert parameters.to_camera_pose_ned().center_ned_m == pose.center_ned_m


def _synthetic_mountain_correspondences(seed: int = 0, *, spread: float = 4.0, count: int = 10):
    rng = np.random.default_rng(seed)
    rvec_true = np.radians(np.array([1.0, -2.0, 0.5]))
    rotation_true, _ = cv2.Rodrigues(rvec_true)
    points = rng.uniform(-spread, spread, size=(count, 3)) + np.array([0.0, 0.0, 20.0])
    baseline_direction = np.array([0.0, 1.0, 0.0])
    left_bearings = points / points[:, 2:3]
    right_points = (rotation_true @ (points - 25.0 * baseline_direction).T).T
    right_bearings = right_points / right_points[:, 2:3]
    return rotation_true, baseline_direction, left_bearings, right_bearings


def _rotation_angle_deg(a, b) -> float:
    delta = np.asarray(a) @ np.asarray(b).T
    rvec, _ = cv2.Rodrigues(delta)
    return float(np.degrees(np.linalg.norm(rvec)))


def test_mountain_rotation_recovers_synthetic_geometry_noiseless() -> None:
    rotation_true, baseline_direction, left, right = _synthetic_mountain_correspondences()

    result = solve_mountain_relative_rotation_robust(
        left_bearings_rdf=left,
        right_bearings_rdf=right,
        baseline_direction_left_rdf=baseline_direction,
        prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
    )

    assert result.apply_eligible
    assert result.jacobian_rank == 3
    assert _rotation_angle_deg(result.rotation_right_from_left_rdf, rotation_true) < 0.01
    assert result.sampson_rms_normalized < 1.0e-9


def test_mountain_rotation_is_invariant_to_baseline_metric_scale() -> None:
    rotation_true, baseline_direction, left, right = _synthetic_mountain_correspondences(seed=5)

    rotations = []
    sampson_rms = []
    for scale in (10.0, 25.0, 100.0):
        result = solve_mountain_relative_rotation_robust(
            left_bearings_rdf=left,
            right_bearings_rdf=right,
            baseline_direction_left_rdf=baseline_direction * scale,
            prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
        )
        rotations.append(result.rotation_right_from_left_rdf)
        sampson_rms.append(result.sampson_rms_normalized)

    for rotation in rotations[1:]:
        assert _rotation_angle_deg(rotation, rotations[0]) < 1.0e-9
    for value in sampson_rms[1:]:
        assert value == pytest.approx(sampson_rms[0], abs=1.0e-12)


def test_mountain_rotation_rejects_deliberate_mismatches_as_outliers() -> None:
    rotation_true, baseline_direction, left, right = _synthetic_mountain_correspondences(
        seed=2, spread=6.0
    )
    corrupted = right.copy()
    corrupted[0] = right[9]
    corrupted[3] = right[6]

    result = solve_mountain_relative_rotation_robust(
        left_bearings_rdf=left,
        right_bearings_rdf=corrupted,
        baseline_direction_left_rdf=baseline_direction,
        prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
    )

    assert 0 not in result.inlier_indices
    assert 3 not in result.inlier_indices
    assert result.total_count == 10
    assert len(result.inlier_indices) == 8
    assert _rotation_angle_deg(result.rotation_right_from_left_rdf, rotation_true) < 0.05


def test_mountain_rotation_reports_ineligible_when_most_correspondences_are_wrong() -> None:
    rotation_true, baseline_direction, left, right = _synthetic_mountain_correspondences(
        seed=3, spread=6.0
    )
    corrupted = right.copy()
    # Scramble six of ten pairs -- well below the 60% inlier-ratio gate.
    corrupted[0] = right[9]
    corrupted[1] = right[8]
    corrupted[2] = right[7]
    corrupted[3] = right[6]
    corrupted[4] = right[5]
    corrupted[5] = right[4]

    result = solve_mountain_relative_rotation_robust(
        left_bearings_rdf=left,
        right_bearings_rdf=corrupted,
        baseline_direction_left_rdf=baseline_direction,
        prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
    )

    assert result.apply_eligible is False
    assert result.ineligibility_reasons


def test_mountain_rotation_ill_conditioned_geometry_is_not_apply_eligible() -> None:
    rng = np.random.default_rng(3)
    rvec_true = np.radians(np.array([1.0, -2.0, 0.5]))
    rotation_true, _ = cv2.Rodrigues(rvec_true)
    baseline_direction = np.array([0.0, 1.0, 0.0])
    points = rng.uniform(-5.0e-5, 5.0e-5, size=(8, 3)) + np.array([0.0, 0.0, 20.0])
    left = points / points[:, 2:3]
    right_points = (rotation_true @ (points - 25.0 * baseline_direction).T).T
    right = right_points / right_points[:, 2:3]

    result = solve_mountain_relative_rotation_robust(
        left_bearings_rdf=left,
        right_bearings_rdf=right,
        baseline_direction_left_rdf=baseline_direction,
        prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
    )

    assert result.jacobian_rank == 3
    assert result.jacobian_condition > 1.0e6
    assert result.apply_eligible is False
    assert any("condition" in reason for reason in result.ineligibility_reasons)

    advisory = solve_mountain_relative_rotation_robust(
        left_bearings_rdf=left,
        right_bearings_rdf=right,
        baseline_direction_left_rdf=baseline_direction,
        prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
        config=MountainRotationConfig(quality_advisory=True),
    )
    assert advisory.apply_eligible
    assert not advisory.ineligibility_reasons
    assert any("condition" in reason for reason in advisory.quality_warnings)
    np.testing.assert_allclose(advisory.rotation_right_from_left_rdf, result.rotation_right_from_left_rdf)


def test_mountain_advisory_policy_still_blocks_rank_deficiency() -> None:
    result = solve_mountain_relative_rotation_robust(
        left_bearings_rdf=[(0, 0, 1)] * 4,
        right_bearings_rdf=[(0, 0, 1)] * 4,
        baseline_direction_left_rdf=(0, 1, 0),
        prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
        config=MountainRotationConfig(quality_advisory=True),
    )
    assert not result.apply_eligible
    assert any("rank" in reason for reason in result.ineligibility_reasons)


def test_mountain_advisory_policy_retains_fit_below_inlier_quality_target() -> None:
    _, baseline, left, right = _synthetic_mountain_correspondences(seed=3, spread=6.0)
    result = solve_mountain_relative_rotation_robust(
        left_bearings_rdf=left, right_bearings_rdf=right,
        baseline_direction_left_rdf=baseline,
        prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
        config=MountainRotationConfig(min_inliers=len(left) + 1, quality_advisory=True),
    )
    assert result.apply_eligible
    assert any("inlier_count" in reason for reason in result.quality_warnings)
    assert np.isfinite(result.rotation_right_from_left_rdf).all()


def test_mountain_rotation_requires_minimum_correspondences() -> None:
    with pytest.raises(ValueError, match="at least 4"):
        solve_mountain_relative_rotation_robust(
            left_bearings_rdf=[(0.0, 0.0, 1.0), (0.1, 0.0, 1.0), (0.0, 0.1, 1.0)],
            right_bearings_rdf=[(0.0, 0.0, 1.0), (0.1, 0.0, 1.0), (0.0, 0.1, 1.0)],
            baseline_direction_left_rdf=(0.0, 1.0, 0.0),
            prior_rotation_right_from_left_rdf=np.eye(3).tolist(),
        )


def test_mountain_sampson_residual_is_near_zero_on_the_true_epipolar_geometry() -> None:
    rotation_true, baseline_direction, left, right = _synthetic_mountain_correspondences(seed=7)

    residuals = mountain_sampson_residuals_normalized(
        rotation_true.tolist(), baseline_direction, left, right
    )

    assert all(abs(value) < 1.0e-9 for value in residuals)


def test_spherical_horizon_solver_recovers_roll_pitch_with_earth_dip_and_flat_model_is_biased() -> None:
    height_m = 25.0
    dip_rad = geometric_horizon_dip_rad(height_m)
    assert geometric_horizon_dip_deg(height_m) == pytest.approx(0.16051, abs=1.0e-4)

    true_roll, true_pitch, true_yaw = 3.25, -2.75, 40.0
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0), roll_deg=true_roll, pitch_deg=true_pitch, yaw_deg=true_yaw
    )
    gravity_camera = np.array(pose.rotation_ned_from_camera_rdf)[2, :]
    gravity_camera /= np.linalg.norm(gravity_camera)
    tmp = np.array([1.0, 0.0, 0.0]) if abs(gravity_camera[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(gravity_camera, tmp)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(gravity_camera, e1)
    theta = 0.5 * 3.141592653589793 - dip_rad
    # A realistic manual/auto horizon annotation samples a narrow azimuth arc
    # across the image width, not the full 360-degree circle around gravity;
    # a full-circle sample is azimuthally symmetric about the true gravity
    # direction and, by that symmetry, the flat-model least-squares optimum
    # coincides with the true direction even though point residuals are
    # individually nonzero. A ~40-degree arc has no such symmetry, so it
    # actually exercises the Earth-dip correction the way a real capture does.
    azimuths = np.radians(np.linspace(-20.0, 20.0, 24))
    rays = [
        np.cos(theta) * gravity_camera + np.sin(theta) * (np.cos(az) * e1 + np.sin(az) * e2)
        for az in azimuths
    ]

    spherical = solve_spherical_horizon_roll_pitch(
        rays, dip_rad=dip_rad, prior_roll_deg=0.0, prior_pitch_deg=0.0, prior_yaw_deg=true_yaw
    )
    assert abs(spherical.roll_deg - true_roll) < 0.01
    assert abs(spherical.pitch_deg - true_pitch) < 0.01
    assert spherical.rms_normalized < 1.0e-9

    # The same rays fed to a flat (dip=0) fit show a clear pitch bias -- proof
    # the Earth-dip correction is necessary, not merely nicer to have.
    flat = solve_spherical_horizon_roll_pitch(
        rays, dip_rad=0.0, prior_roll_deg=0.0, prior_pitch_deg=0.0, prior_yaw_deg=true_yaw
    )
    assert abs(flat.pitch_deg - true_pitch) > 0.1


def test_spherical_horizon_yaw_is_never_a_variable_of_the_fit() -> None:
    height_m = 40.0
    dip_rad = geometric_horizon_dip_rad(height_m)
    true_roll, true_pitch = -1.5, 2.0
    for true_yaw in (10.0, 200.0):
        pose = CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(0.0, 0.0, 0.0), roll_deg=true_roll, pitch_deg=true_pitch, yaw_deg=true_yaw
        )
        gravity_camera = np.array(pose.rotation_ned_from_camera_rdf)[2, :]
        gravity_camera /= np.linalg.norm(gravity_camera)
        tmp = np.array([1.0, 0.0, 0.0]) if abs(gravity_camera[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        e1 = np.cross(gravity_camera, tmp)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(gravity_camera, e1)
        theta = 0.5 * 3.141592653589793 - dip_rad
        rays = [
            np.cos(theta) * gravity_camera + np.sin(theta) * (np.cos(az) * e1 + np.sin(az) * e2)
            for az in np.linspace(0.0, 2.0 * 3.141592653589793, 24, endpoint=False)
        ]
        result = solve_spherical_horizon_roll_pitch(
            rays, dip_rad=dip_rad, prior_roll_deg=0.0, prior_pitch_deg=0.0, prior_yaw_deg=true_yaw
        )
        assert abs(result.roll_deg - true_roll) < 0.01
        assert abs(result.pitch_deg - true_pitch) < 0.01
