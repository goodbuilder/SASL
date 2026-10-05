from __future__ import annotations

from dataclasses import FrozenInstanceError
from math import cos, radians, sin
from random import Random

import pytest

from Workstation.workstation_core.optical_pose import (
    OPTICAL_RDF_TO_VIRTUAL_FRD,
    POSE_DOF_NAMES,
    CameraPoseNed,
    OpticalPoseCandidate,
    OpticalPoseSelection,
    PoseDofMask,
    PoseParametersNed,
    SE3Transform,
    StereoPoseRdf,
    angular_ray_residual_rad,
    apply_selected_dofs,
    boresight_relative_yaw_right_minus_left_deg,
    camera_pose_ned_from_opencv_pnp,
    compose_se3,
    estimate_drone_yaw_deg,
    expand_selected_dofs,
    geometric_horizon_dip_deg,
    gravity_camera_rdf_from_normalized_horizon_line,
    gravity_camera_rdf_from_pixel_horizon_line,
    horizon_roll_pitch_deg_from_gravity_camera_rdf,
    horizon_unit_ray_residual,
    opencv_pnp_from_camera_pose_ned,
    optical_rdf_to_virtual_frd,
    pack_selected_dofs,
    pixel_horizon_line_from_gravity_camera_rdf,
    pose_parameters_from_camera_pose,
    rotation_ned_from_virtual_frd_rpy_deg,
    stereo_pose_from_absolute,
    transfer_stereo_pose_through_bridges,
    virtual_frd_to_optical_rdf,
    wrap_angle_deg,
)


def _matvec(matrix, vector):
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))


def _transpose(matrix):
    return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))


def _subtract(left, right):
    return tuple(a - b for a, b in zip(left, right))


def _flatten_matrix(matrix):
    return tuple(value for row in matrix for value in row)


def _parameter_tuple(parameters: PoseParametersNed):
    return tuple(getattr(parameters, name) for name in POSE_DOF_NAMES)


def _observability(**overrides: bool) -> tuple[tuple[str, bool], ...]:
    values = {name: False for name in POSE_DOF_NAMES}
    values.update(overrides)
    return tuple((name, values[name]) for name in POSE_DOF_NAMES)


def test_optical_rdf_virtual_frd_axis_mapping_is_right_handed_and_round_trips() -> None:
    assert optical_rdf_to_virtual_frd((1.0, 2.0, 3.0)) == (3.0, 1.0, 2.0)
    assert virtual_frd_to_optical_rdf((3.0, 1.0, 2.0)) == (1.0, 2.0, 3.0)

    right = optical_rdf_to_virtual_frd((1.0, 0.0, 0.0))
    down = optical_rdf_to_virtual_frd((0.0, 1.0, 0.0))
    forward = optical_rdf_to_virtual_frd((0.0, 0.0, 1.0))

    assert forward == (1.0, 0.0, 0.0)
    assert right == (0.0, 1.0, 0.0)
    assert down == (0.0, 0.0, 1.0)
    assert OPTICAL_RDF_TO_VIRTUAL_FRD == (
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )


def test_absolute_pose_derives_opencv_stereo_contract_for_25m_east_baseline() -> None:
    left = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )
    right = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 25.0, 0.0),
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )

    stereo = stereo_pose_from_absolute(left, right)

    assert _flatten_matrix(stereo.rotation_right_from_left_rdf) == pytest.approx(
        (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        abs=1.0e-12,
    )
    assert stereo.translation_right_from_left_rdf_m == pytest.approx((-25.0, 0.0, 0.0), abs=1.0e-12)
    assert stereo.right_origin_in_left_rdf_m == pytest.approx((25.0, 0.0, 0.0), abs=1.0e-12)
    assert stereo.baseline_m == pytest.approx(25.0, abs=1.0e-12)


def test_absolute_to_stereo_transforms_the_same_world_point() -> None:
    left = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(13.0, -7.0, 2.0),
        roll_deg=3.0,
        pitch_deg=-4.0,
        yaw_deg=31.0,
    )
    right = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(2.0, 15.0, 3.5),
        roll_deg=-2.0,
        pitch_deg=5.0,
        yaw_deg=27.0,
    )
    point_ned = (170.0, 80.0, -15.0)
    left_point = _matvec(
        _transpose(left.rotation_ned_from_camera_rdf),
        _subtract(point_ned, left.center_ned_m),
    )
    expected_right = _matvec(
        _transpose(right.rotation_ned_from_camera_rdf),
        _subtract(point_ned, right.center_ned_m),
    )

    stereo = stereo_pose_from_absolute(left, right)
    actual_right = tuple(
        value + translation
        for value, translation in zip(
            _matvec(stereo.rotation_right_from_left_rdf, left_point),
            stereo.translation_right_from_left_rdf_m,
        )
    )

    assert actual_right == pytest.approx(expected_right, abs=1.0e-10)


def test_absolute_to_stereo_identity_holds_for_seeded_random_poses_and_points() -> None:
    random = Random(20260813)
    maximum_error = 0.0
    for _index in range(200):
        left = CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=tuple(random.uniform(-50.0, 50.0) for _axis in range(3)),
            roll_deg=random.uniform(-20.0, 20.0),
            pitch_deg=random.uniform(-20.0, 20.0),
            yaw_deg=random.uniform(-180.0, 180.0),
        )
        right = CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=tuple(random.uniform(-50.0, 50.0) for _axis in range(3)),
            roll_deg=random.uniform(-20.0, 20.0),
            pitch_deg=random.uniform(-20.0, 20.0),
            yaw_deg=random.uniform(-180.0, 180.0),
        )
        point_ned = tuple(random.uniform(-1000.0, 1000.0) for _axis in range(3))
        left_point = _matvec(
            _transpose(left.rotation_ned_from_camera_rdf),
            _subtract(point_ned, left.center_ned_m),
        )
        expected_right = _matvec(
            _transpose(right.rotation_ned_from_camera_rdf),
            _subtract(point_ned, right.center_ned_m),
        )
        stereo = stereo_pose_from_absolute(left, right)
        actual_right = tuple(
            value + translation
            for value, translation in zip(
                _matvec(stereo.rotation_right_from_left_rdf, left_point),
                stereo.translation_right_from_left_rdf_m,
            )
        )
        maximum_error = max(
            maximum_error,
            *(abs(actual - expected) for actual, expected in zip(actual_right, expected_right)),
        )

    assert maximum_error < 1.0e-9


def test_boresight_relative_yaw_is_right_minus_left_and_wraps() -> None:
    left = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=179.0,
    )
    right = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 25.0, 0.0),
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=-179.0,
    )

    assert boresight_relative_yaw_right_minus_left_deg(left, right) == pytest.approx(2.0, abs=1.0e-12)
    assert wrap_angle_deg(180.0) == -180.0
    assert wrap_angle_deg(-181.0) == 179.0


def test_horizon_normalized_and_ideal_pixel_lines_recover_roll_pitch_not_yaw() -> None:
    roll_deg, pitch_deg = 4.25, -3.75
    first = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        yaw_deg=12.0,
    )
    second = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        yaw_deg=143.0,
    )
    down_ned = (0.0, 0.0, 1.0)
    gravity_first = _matvec(_transpose(first.rotation_ned_from_camera_rdf), down_ned)
    gravity_second = _matvec(_transpose(second.rotation_ned_from_camera_rdf), down_ned)
    intrinsics = ((4200.0, 0.0, 640.0), (0.0, 4180.0, 360.0), (0.0, 0.0, 1.0))

    assert gravity_first == pytest.approx(gravity_second, abs=1.0e-12)
    normalized = gravity_camera_rdf_from_normalized_horizon_line(gravity_first)
    pixel_line = pixel_horizon_line_from_gravity_camera_rdf(gravity_first, intrinsics)
    from_pixels = gravity_camera_rdf_from_pixel_horizon_line(pixel_line, intrinsics)

    assert normalized == pytest.approx(gravity_first, abs=1.0e-12)
    assert from_pixels == pytest.approx(gravity_first, abs=1.0e-12)
    assert horizon_roll_pitch_deg_from_gravity_camera_rdf(normalized) == pytest.approx(
        (roll_deg, pitch_deg),
        abs=1.0e-10,
    )
    assert gravity_camera_rdf_from_normalized_horizon_line(tuple(-v for v in gravity_first)) == pytest.approx(
        gravity_first,
        abs=1.0e-12,
    )


def test_spherical_horizon_dip_and_unit_ray_residual_are_explicit() -> None:
    dip_deg = geometric_horizon_dip_deg(25.0)
    dip_rad = radians(dip_deg)
    horizon_ray = (0.0, sin(dip_rad), cos(dip_rad))

    assert dip_deg == pytest.approx(0.160510, abs=1.0e-6)
    assert horizon_unit_ray_residual((0.0, 1.0, 0.0), horizon_ray, dip_rad=dip_rad) == pytest.approx(
        0.0,
        abs=1.0e-12,
    )
    assert horizon_unit_ray_residual((0.0, 1.0, 0.0), horizon_ray) > 0.0


def test_drone_rtk_ray_recovers_yaw_with_fixed_roll_pitch() -> None:
    expected_pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=2.5,
        pitch_deg=-4.0,
        yaw_deg=37.0,
    )
    ray_camera = (0.17, -0.06, 1.0)
    expected_los = _matvec(expected_pose.rotation_ned_from_camera_rdf, ray_camera)

    estimate = estimate_drone_yaw_deg(
        ray_camera,
        expected_los,
        roll_deg=2.5,
        pitch_deg=-4.0,
    )

    assert estimate.observable is True
    assert estimate.reason == "ok"
    assert estimate.yaw_deg == pytest.approx(37.0, abs=1.0e-10)
    assert estimate.angular_residual_rad == pytest.approx(0.0, abs=1.0e-12)
    assert angular_ray_residual_rad(
        expected_pose.rotation_ned_from_camera_rdf,
        ray_camera,
        expected_los,
    ) == pytest.approx(0.0, abs=1.0e-12)


def test_drone_yaw_reports_vertical_ray_as_unobservable() -> None:
    estimate = estimate_drone_yaw_deg(
        (0.0, 1.0, 0.0),  # optical Down -> virtual/world Down when R/P are zero
        (0.0, 0.0, 1.0),
        roll_deg=0.0,
        pitch_deg=0.0,
    )

    assert estimate.observable is False
    assert estimate.yaw_deg is None
    assert estimate.angular_residual_rad is None
    assert estimate.reason == "vertical_ray_or_los"


def test_opencv_pnp_pose_inverse_round_trips_rotation_translation_and_center() -> None:
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(12.0, -8.0, 2.5),
        roll_deg=4.0,
        pitch_deg=-7.0,
        yaw_deg=21.0,
    )

    rotation_camera_from_ned, translation_camera_from_ned = opencv_pnp_from_camera_pose_ned(pose)
    recovered = camera_pose_ned_from_opencv_pnp(
        rotation_camera_from_ned,
        tuple((value,) for value in translation_camera_from_ned),
    )

    assert _flatten_matrix(recovered.rotation_ned_from_camera_rdf) == pytest.approx(
        _flatten_matrix(pose.rotation_ned_from_camera_rdf),
        abs=1.0e-12,
    )
    assert recovered.center_ned_m == pytest.approx(pose.center_ned_m, abs=1.0e-12)


def test_synthetic_opencv_pnp_recovers_camera_pose_ned() -> None:
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
        ],
        dtype=np.float64,
    )
    rotation_ned_from_camera = np.asarray(pose.rotation_ned_from_camera_rdf)
    center = np.asarray(pose.center_ned_m)
    object_points_ned = (rotation_ned_from_camera @ camera_points.T).T + center
    intrinsics = np.asarray(((1600.0, 0.0, 640.0), (0.0, 1580.0, 360.0), (0.0, 0.0, 1.0)))
    image_homogeneous = (intrinsics @ camera_points.T).T
    image_points = image_homogeneous[:, :2] / image_homogeneous[:, 2:]

    ok, rvec, tvec = cv2.solvePnP(
        object_points_ned,
        image_points,
        intrinsics,
        np.zeros(5),
        flags=cv2.SOLVEPNP_SQPNP,
    )
    rotation_camera_from_ned, _jacobian = cv2.Rodrigues(rvec)
    recovered = camera_pose_ned_from_opencv_pnp(rotation_camera_from_ned, tvec)

    assert ok is True
    assert _flatten_matrix(recovered.rotation_ned_from_camera_rdf) == pytest.approx(
        _flatten_matrix(pose.rotation_ned_from_camera_rdf),
        abs=1.0e-8,
    )
    assert recovered.center_ned_m == pytest.approx(pose.center_ned_m, abs=1.0e-6)


def test_full_se3_zoom_bridge_composition_matches_pointwise_chain() -> None:
    source = StereoPoseRdf(
        rotation_right_from_left_rdf=rotation_ned_from_virtual_frd_rpy_deg(2.0, -3.0, 5.0),
        translation_right_from_left_rdf_m=(-25.0, 0.2, -0.1),
    )
    left_bridge = SE3Transform(
        output_frame="left_zoom8_rdf",
        input_frame="left_zoom1_rdf",
        rotation_output_from_input=rotation_ned_from_virtual_frd_rpy_deg(0.3, -0.2, 0.1),
        translation_output_from_input_m=(0.01, -0.02, 0.03),
    )
    right_bridge = SE3Transform(
        output_frame="right_zoom8_rdf",
        input_frame="right_zoom1_rdf",
        rotation_output_from_input=rotation_ned_from_virtual_frd_rpy_deg(-0.1, 0.4, -0.2),
        translation_output_from_input_m=(-0.03, 0.02, 0.01),
    )

    transferred = transfer_stereo_pose_through_bridges(
        source,
        left_target_from_source=left_bridge,
        right_target_from_source=right_bridge,
    )
    left_target_point = (0.4, -0.3, 20.0)
    left_source_point = left_bridge.inverse().transform_point(left_target_point)
    right_source_point = tuple(
        value + translation
        for value, translation in zip(
            _matvec(source.rotation_right_from_left_rdf, left_source_point),
            source.translation_right_from_left_rdf_m,
        )
    )
    expected_right_target = right_bridge.transform_point(right_source_point)
    actual_right_target = tuple(
        value + translation
        for value, translation in zip(
            _matvec(transferred.rotation_right_from_left_rdf, left_target_point),
            transferred.translation_right_from_left_rdf_m,
        )
    )

    assert actual_right_target == pytest.approx(expected_right_target, abs=1.0e-10)
    with pytest.raises(ValueError, match="frame mismatch"):
        compose_se3(left_bridge, right_bridge)


def test_rotation_only_zoom_bridges_rotate_source_translation_without_inventing_offsets() -> None:
    source = StereoPoseRdf(
        rotation_right_from_left_rdf=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        translation_right_from_left_rdf_m=(-25.0, 0.0, 0.0),
    )
    left_bridge = SE3Transform(
        "left_b",
        "left_a",
        rotation_ned_from_virtual_frd_rpy_deg(0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )
    right_rotation = rotation_ned_from_virtual_frd_rpy_deg(0.0, 0.0, 2.0)
    right_bridge = SE3Transform("right_b", "right_a", right_rotation, (0.0, 0.0, 0.0))

    transferred = transfer_stereo_pose_through_bridges(
        source,
        left_target_from_source=left_bridge,
        right_target_from_source=right_bridge,
    )

    assert transferred.translation_right_from_left_rdf_m == pytest.approx(
        _matvec(right_rotation, source.translation_right_from_left_rdf_m),
        abs=1.0e-12,
    )


def test_selected_dof_helpers_keep_unchecked_prior_values_exact() -> None:
    prior = PoseParametersNed(1.0, 2.0, 3.0, 10.0, 20.0, 30.0)
    candidate = PoseParametersNed(-1.0, -2.0, -179.0, -10.0, -20.0, -30.0)
    mask = PoseDofMask.from_names(("yaw_deg", "east_m"))

    assert mask.selected_names == ("yaw_deg", "east_m")
    assert pack_selected_dofs(candidate, mask) == (-179.0, -20.0)
    assert expand_selected_dofs(prior, mask, (-179.0, -20.0)) == PoseParametersNed(
        1.0,
        2.0,
        -179.0,
        10.0,
        -20.0,
        30.0,
    )
    applied = apply_selected_dofs(prior, candidate, mask)
    assert applied.roll_deg == prior.roll_deg
    assert applied.pitch_deg == prior.pitch_deg
    assert applied.north_m == prior.north_m
    assert applied.down_m == prior.down_m
    assert _parameter_tuple(pose_parameters_from_camera_pose(applied.to_camera_pose_ned())) == pytest.approx(
        _parameter_tuple(applied),
        abs=1.0e-12,
    )


def test_candidate_and_selection_are_deeply_immutable_and_allow_prior_only_dofs() -> None:
    left = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )
    right = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 25.0, 0.0),
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.2,
    )
    source_ids = ["burst-001"]
    candidate = OpticalPoseCandidate(
        candidate_id="candidate-001",
        method="horizon_plus_drone_rtk",
        left_pose_ned=left,
        right_pose_ned=right,
        optimized_dofs=PoseDofMask(),
        observability=_observability(roll_deg=True, pitch_deg=True, yaw_deg=True),
        source_observation_ids=source_ids,  # type: ignore[arg-type]
        input_hashes=(("intrinsics", "a" * 64),),
        quality_metrics=(("angular_residual_rad", 0.001),),
        uncertainty_metrics=(("relative_yaw_sigma_deg", 0.02),),
        warnings=("translation_prior_only",),
        created_utc_ns=123,
    )
    source_ids.append("mutated-after-construction")
    selection = OpticalPoseSelection(
        selection_id="selection-001",
        candidate_id=candidate.candidate_id,
        selected_dofs=PoseDofMask.from_names(("roll_deg", "pitch_deg", "yaw_deg")),
        effective_from_pair_sequence=42,
        applied_utc_ns=456,
        previous_profile_id="nominal-001",
        quality_snapshot=candidate.quality_metrics,
    )

    assert candidate.source_observation_ids == ("burst-001",)
    assert candidate.stereo_pose_rdf.baseline_m == pytest.approx(25.0)
    assert candidate.relative_boresight_yaw_right_minus_left_deg == pytest.approx(0.2)
    assert dict(candidate.observability)["north_m"] is False
    assert selection.effective_from_pair_sequence == 42
    with pytest.raises(FrozenInstanceError):
        candidate.method = "changed"  # type: ignore[misc]
    with pytest.raises(TypeError):
        candidate.quality_metrics[0][1] = 3.0  # type: ignore[index]


def test_non_finite_and_non_rotation_inputs_fail_closed() -> None:
    with pytest.raises(ValueError, match="finite"):
        PoseParametersNed(0.0, 0.0, float("nan"), 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="orthonormal"):
        CameraPoseNed(
            ((1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 1.0)),
            (0.0, 0.0, 0.0),
        )
    with pytest.raises(ValueError, match="all pose DOFs"):
        OpticalPoseCandidate(
            candidate_id="invalid",
            method="test",
            left_pose_ned=CameraPoseNed.from_virtual_frd_rpy_deg(
                center_ned_m=(0.0, 0.0, 0.0),
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
            ),
            right_pose_ned=CameraPoseNed.from_virtual_frd_rpy_deg(
                center_ned_m=(0.0, 1.0, 0.0),
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
            ),
            optimized_dofs=PoseDofMask(),
            observability=(("yaw_deg", True),),
        )
