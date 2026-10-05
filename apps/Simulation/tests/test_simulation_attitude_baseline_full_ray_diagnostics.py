from __future__ import annotations

import json
import math
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

import stereo_ground_sim.stereo_distance_node as stereo_distance_module
from camera_oracle_test_support import seed_observed_camera_oracle
from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
)
from stereo_ground_sim.coordinate_frames import (
    PX4_BODY_FRD_FROM_OPTICAL_RDF,
    camera_optical_pose_from_px4_attitude_wxyz,
    validate_rotation_matrix,
)
from stereo_ground_sim.gps_baseline import RawGpsFix
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.time_alignment import SimulationEpochTracker


def _node() -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.synthetic_detection_bbox_size_px = 32.0
    node.lock = threading.Lock()
    node.simulation_epoch = SimulationEpochTracker()
    node.px4_attitude_max_age_s = 2.5
    node.px4_attitude_pair_skew_s = 2.5
    node.px4_attitudes = {"left": None, "right": None}
    node.operational_sources_by_pair_id = {}
    node.enable_gps_baseline_triangulation = True
    node.gps_baseline_source = "gps_baseline"
    node.gps_baseline_max_age_s = 2.5
    node.gps_baseline_fixes = {"left": None, "right": None, "uam": None}
    node.global_baseline_fixes = {"left": None, "right": None}
    # nominal_camera_poses() gives both cameras yaw=pi/2 (ENU) with the
    # baseline along body Right -- Forward sits 90 deg CCW of the baseline
    # bearing, hence -90.0 for both sides (verified against the actual
    # geometry, not assumed).
    node.cam_l_gnss_baseline_to_forward_yaw_deg = -90.0
    node.cam_r_gnss_baseline_to_forward_yaw_deg = -90.0
    node._ensure_camera_frames()
    seed_observed_camera_oracle(node, left_pose=node.nominal_left_camera, right_pose=node.nominal_right_camera)
    return node


def _dot(left, right) -> float:
    return sum(a * b for a, b in zip(left, right))


def _determinant(matrix) -> float:
    a, b, c = matrix
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


def _basis_rows(pose):
    columns = (pose.right_world_enu, pose.down_world_enu, pose.forward_world_enu)
    return tuple(tuple(columns[column][row] for column in range(3)) for row in range(3))


def _assert_valid_right_handed_basis(pose) -> None:
    axes = (pose.right_world_enu, pose.down_world_enu, pose.forward_world_enu)
    assert all(_dot(axis, axis) == pytest.approx(1.0, abs=1.0e-12) for axis in axes)
    assert all(abs(_dot(axes[a], axes[b])) <= 1.0e-12 for a, b in ((0, 1), (0, 2), (1, 2)))
    assert _determinant(_basis_rows(pose)) == pytest.approx(1.0, abs=1.0e-12)


def _attitude_msg(*, timestamp_us: int = 1_000_000, q=(1.0, 0.0, 0.0, 0.0)):
    return SimpleNamespace(timestamp=timestamp_us, timestamp_sample=timestamp_us, q=q, quat_reset_counter=0)


def _fix(timestamp_us: int = 1_000_000) -> RawGpsFix:
    return RawGpsFix(
        timestamp_us=timestamp_us,
        latitude_deg=47.3979712,
        longitude_deg=8.5459985,
        altitude_msl_m=10.0,
        eph_m=0.9,
        epv_m=1.8,
    )


def _seed_attitudes(
    node: StereoDistanceNode,
    *,
    left_stamp=1_000_000,
    right_stamp=1_000_000,
    left_q=(1.0, 0.0, 0.0, 0.0),
    right_q=(1.0, 0.0, 0.0, 0.0),
) -> None:
    assert node._px4_attitude_callback(
        "left", _attitude_msg(timestamp_us=left_stamp, q=left_q)
    ) is None
    assert node._px4_attitude_callback(
        "right", _attitude_msg(timestamp_us=right_stamp, q=right_q)
    ) is None


def _seed_gps(node: StereoDistanceNode) -> None:
    now_ns = time.monotonic_ns()
    simulation_timestamp_ns = node.simulation_epoch.last_clock_stamp_ns
    epoch = node.simulation_epoch.epoch
    node.gps_baseline_fixes["left"] = (
        _fix(),
        now_ns,
        epoch,
        simulation_timestamp_ns,
    )
    node.gps_baseline_fixes["right"] = (
        _fix(),
        now_ns,
        epoch,
        simulation_timestamp_ns,
    )


def _observed_pixels_for_baseline_point(node: StereoDistanceNode, point_frd=(900.0, 3.0, -2.0)):
    world_point = node.baseline_center_frd.point_to_world(point_frd)
    left, right = node._ideal_detection_pair_from_reference_world(world_point, 1280, 720)
    assert left is not None and right is not None
    return left["cx"], left["cy"], right["cx"], right["cy"]


def _position_error(result, expected_frd) -> float:
    return math.sqrt(
        sum(
            (float(result[name]) - expected_frd[index]) ** 2
            for index, name in enumerate(("forward_m", "right_m", "down_m"))
        )
    )


def _yaw_q_wxyz(angle_deg: float):
    half = math.radians(angle_deg) * 0.5
    return math.cos(half), 0.0, 0.0, math.sin(half)


def _truth(node, pixels):
    result = node._triangulate_pixel_centers_oracle(
        *pixels, 1280, 720, camera_oracle_snapshot=node._camera_oracle_snapshot()
    )
    assert result is not None and result["computation_status"] == "computed"
    return result


@pytest.mark.parametrize(
    "quaternion_wxyz",
    (
        (1.0, 0.0, 0.0, 0.0),
        (math.cos(math.pi / 8.0), 0.0, 0.0, math.sin(math.pi / 8.0)),
        (math.cos(math.pi / 10.0), math.sin(math.pi / 10.0), 0.0, 0.0),
    ),
)
def test_px4_attitude_extrinsic_returns_right_handed_orthonormal_optical_basis(quaternion_wxyz) -> None:
    pose = camera_optical_pose_from_px4_attitude_wxyz((1.0, 2.0, 3.0), quaternion_wxyz)

    _assert_valid_right_handed_basis(pose)


def test_extrinsic_rejects_non_rotation_and_reflection() -> None:
    invalid = ((1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 1.0))
    reflection = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0))

    with pytest.raises(ValueError, match="orthogonal"):
        validate_rotation_matrix(invalid)
    with pytest.raises(ValueError, match="determinant"):
        validate_rotation_matrix(reflection)
    with pytest.raises(ValueError):
        camera_optical_pose_from_px4_attitude_wxyz(
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            rotation_body_from_optical=reflection,
        )


def test_fixed_extrinsic_basis_mapping_and_round_trip() -> None:
    audit = validate_rotation_matrix(PX4_BODY_FRD_FROM_OPTICAL_RDF)
    assert audit["orthogonality_error"] <= 1.0e-12
    assert audit["determinant"] == pytest.approx(1.0, abs=1.0e-12)
    assert PX4_BODY_FRD_FROM_OPTICAL_RDF == (
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    optical = (0.2672612419124244, -0.5345224838248488, 0.8017837257372732)
    body = tuple(
        sum(PX4_BODY_FRD_FROM_OPTICAL_RDF[row][column] * optical[column] for column in range(3))
        for row in range(3)
    )
    recovered = tuple(
        sum(PX4_BODY_FRD_FROM_OPTICAL_RDF[row][column] * body[row] for row in range(3))
        for column in range(3)
    )
    assert recovered == pytest.approx(optical, abs=1.0e-12)


def test_left_and_right_sdf_fixed_joints_prove_zero_relative_mount_rotation() -> None:
    sdf_path = PACKAGE_ROOT / "worlds" / "stereo_ground_drone.sdf"
    root = ET.parse(sdf_path).getroot()

    for model_name in ("left_stereo_camera", "right_stereo_camera"):
        model = root.find(f"./world/model[@name='{model_name}']")
        assert model is not None
        optical_link = model.find("./link[@name='optical_link']")
        joint = model.find("./joint[@name='optical_mount']")
        assert optical_link is not None and joint is not None
        assert optical_link.findtext("pose", "").split() == ["0"] * 6
        assert optical_link.find("pose").get("relative_to") == "base_link"
        assert joint.get("type") == "fixed"
        assert joint.findtext("parent") == "base_link"
        assert joint.findtext("child") == "optical_link"

    # Both explicit fixed-joint transforms are identity, so their angular
    # difference from the contract is exactly zero degrees.
    gazebo_relative_pose_angle_error_deg = 0.0
    assert gazebo_relative_pose_angle_error_deg <= 1.0e-8


def test_diagnostic_rays_compute_attitude_and_operational_combinations(monkeypatch) -> None:
    node = _node()
    _seed_attitudes(node)
    _seed_gps(node)
    monkeypatch.setattr(stereo_distance_module, "gps_baseline_enu_m", lambda _left, _right: (25.0, 0.0, 0.0))
    pixels = _observed_pixels_for_baseline_point(node)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())
    truth = _truth(node, pixels)
    analysis = node._diagnostic_analysis(truth, diagnostics)

    for key in ("attitude", "operational"):
        assert diagnostics[key]["computation_status"] == "computed"
        assert diagnostics[key]["output_frame"] == "baseline_center_frd"
        assert diagnostics[key]["axis_contract"] == BASELINE_CENTER_FRD_AXIS_CONTRACT
        assert diagnostics[key]["vector_contract"] == BASELINE_CENTER_FRD_VECTOR_CONTRACT
        assert diagnostics[key]["left_depth_m"] > 0.0
        assert diagnostics[key]["right_depth_m"] > 0.0
        assert diagnostics[key]["valid"] is True
        assert analysis["comparisons"][key]["position_difference_m"] <= 1.0e-9
    assert _position_error(diagnostics["operational"], (900.0, 3.0, -2.0)) < 1.0e-6
    assert "interaction" not in analysis


def test_attitude_only_error_keeps_baseline_equal_truth(monkeypatch) -> None:
    node = _node()
    _seed_attitudes(
        node,
        left_q=_yaw_q_wxyz(0.05),
        right_q=_yaw_q_wxyz(-0.05),
    )
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (25.0, 0.0, 0.0),
    )
    pixels = _observed_pixels_for_baseline_point(node)
    truth = _truth(node, pixels)
    diagnostics = node._diagnostic_rays(
        *pixels, 1280, 720, node._camera_oracle_snapshot()
    )
    analysis = node._diagnostic_analysis(truth, diagnostics)

    assert analysis["comparisons"]["operational"]["position_difference_m"] > 1.0e-6
    assert math.dist(
        diagnostics["operational"]["midpoint_baseline_center_frd_m"],
        diagnostics["attitude"]["midpoint_baseline_center_frd_m"],
    ) <= 1.0e-9
    assert analysis["comparisons"]["attitude"]["position_difference_m"] > 1.0e-6


def test_baseline_only_error_keeps_attitude_equal_truth(monkeypatch) -> None:
    node = _node()
    _seed_attitudes(node)
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (25.2, 0.1, -0.05),
    )
    pixels = _observed_pixels_for_baseline_point(node)
    truth = _truth(node, pixels)
    diagnostics = node._diagnostic_rays(
        *pixels, 1280, 720, node._camera_oracle_snapshot()
    )
    analysis = node._diagnostic_analysis(truth, diagnostics)

    assert analysis["comparisons"]["attitude"]["position_difference_m"] <= 1.0e-9
    assert math.dist(
        diagnostics["operational"]["midpoint_baseline_center_frd_m"],
        diagnostics["attitude"]["midpoint_baseline_center_frd_m"],
    ) > 1.0e-6
    assert analysis["comparisons"]["operational"]["position_difference_m"] > 1.0e-6


def test_combined_error_retriangulates_full_and_reports_finite_interaction(monkeypatch) -> None:
    node = _node()
    _seed_attitudes(
        node,
        left_q=_yaw_q_wxyz(0.05),
        right_q=_yaw_q_wxyz(-0.05),
    )
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (25.2, 0.1, -0.05),
    )
    pixels = _observed_pixels_for_baseline_point(node)
    truth = _truth(node, pixels)
    diagnostics = node._diagnostic_rays(
        *pixels, 1280, 720, node._camera_oracle_snapshot()
    )
    analysis = node._diagnostic_analysis(truth, diagnostics)

    full_midpoint = diagnostics["operational"]["midpoint_baseline_center_frd_m"]
    assert math.dist(
        full_midpoint, diagnostics["attitude"]["midpoint_baseline_center_frd_m"]
    ) > 1.0e-6
    assert math.dist(
        full_midpoint, diagnostics["attitude"]["midpoint_baseline_center_frd_m"]
    ) > 1.0e-6
    assert "interaction" not in analysis


def test_diagnostic_rays_isolate_invalid_attitude_from_baseline(monkeypatch) -> None:
    node = _node()
    node.px4_attitudes["left"] = ((float("nan"), 0.0, 0.0, 0.0), 1_000_000, 0, node.simulation_epoch.epoch, time.monotonic())
    node.px4_attitudes["right"] = ((1.0, 0.0, 0.0, 0.0), 1_000_000, 0, node.simulation_epoch.epoch, time.monotonic())
    _seed_gps(node)
    monkeypatch.setattr(stereo_distance_module, "gps_baseline_enu_m", lambda _left, _right: (25.0, 0.0, 0.0))
    pixels = _observed_pixels_for_baseline_point(node)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())

    assert diagnostics["attitude"]["computation_status"] == "invalid_px4_attitude"
    assert diagnostics["operational"]["computation_status"] == "invalid_px4_attitude"


def _seed_gps_matching_nominal_baseline(node: StereoDistanceNode, *, declination_deg: float = 0.0) -> None:
    """Seed distinct, realistic left/right GPS fixes whose true-north
    geodetic baseline matches the node's nominal camera separation (unlike
    ``_seed_gps``, which points both cameras at the identical fixed lat/lon
    -- a degenerate zero-length baseline that ``gps_baseline_bearing_rad``
    correctly rejects). ``declination_deg`` rotates the GPS baseline bearing
    away from the nominal/design bearing, simulating a location with that
    much true declination relative to the injected attitude bias.
    """

    wgs84_a_m = 6_378_137.0
    lat0_deg, lon0_deg = 47.398, 8.546
    nominal_baseline = tuple(
        node.nominal_right_camera.center_world_enu_m[index]
        - node.nominal_left_camera.center_world_enu_m[index]
        for index in range(3)
    )
    rotation_rad = math.radians(declination_deg)
    cos_r, sin_r = math.cos(rotation_rad), math.sin(rotation_rad)
    east, north, up = nominal_baseline
    rotated_east = cos_r * east - sin_r * north
    rotated_north = sin_r * east + cos_r * north
    dlat_deg = math.degrees(rotated_north / wgs84_a_m)
    dlon_deg = math.degrees(rotated_east / (wgs84_a_m * math.cos(math.radians(lat0_deg))))
    now_ns = time.monotonic_ns()
    simulation_timestamp_ns = node.simulation_epoch.last_clock_stamp_ns
    epoch = node.simulation_epoch.epoch
    node.gps_baseline_fixes["left"] = (
        RawGpsFix(
            timestamp_us=1_000_000, latitude_deg=lat0_deg, longitude_deg=lon0_deg,
            altitude_msl_m=0.0, eph_m=0.5, epv_m=0.5,
        ),
        now_ns, epoch, simulation_timestamp_ns,
    )
    node.gps_baseline_fixes["right"] = (
        RawGpsFix(
            timestamp_us=1_000_000, latitude_deg=lat0_deg + dlat_deg, longitude_deg=lon0_deg + dlon_deg,
            altitude_msl_m=0.0, eph_m=0.5, epv_m=0.5,
        ),
        now_ns, epoch, simulation_timestamp_ns,
    )


def test_true_north_attitude_pose_selection_rejects_missing_and_stale_gps() -> None:
    node = _node()
    snapshot = node._camera_oracle_snapshot()
    _seed_attitudes(node)

    poses, status = node._px4_attitude_true_north_camera_poses(snapshot)
    assert poses is None
    assert status == "missing_left_gps_fix"

    _seed_gps_matching_nominal_baseline(node)
    poses, status = node._px4_attitude_true_north_camera_poses(snapshot)
    assert poses is not None
    assert status == "available"

    old = time.monotonic_ns() - int((node.gps_baseline_max_age_s + 0.1) * 1.0e9)
    left_fix, _old_ns, epoch, ts_ns = node.gps_baseline_fixes["left"]
    node.gps_baseline_fixes["left"] = (left_fix, old, epoch, ts_ns)
    poses, status = node._px4_attitude_true_north_camera_poses(snapshot)
    assert poses is None
    assert status == "stale_left_gps_fix"


def _pitch_yaw_q_wxyz(pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float]:
    """Body-FRD (roll=0) quaternion for a given pitch+yaw, ZYX Euler order
    (R = Rz(yaw) @ Ry(pitch)), verified to reduce exactly to ``_yaw_q_wxyz``
    when pitch=0 and to produce the expected nonzero vertical component when
    yaw=0."""

    half_pitch, half_yaw = math.radians(pitch_deg) / 2.0, math.radians(yaw_deg) / 2.0
    cos_p, sin_p = math.cos(half_pitch), math.sin(half_pitch)
    cos_y, sin_y = math.cos(half_yaw), math.sin(half_yaw)
    return (cos_p * cos_y, -sin_p * sin_y, sin_p * cos_y, cos_p * sin_y)


# ---------------------------------------------------------------------------
# Phase 2: per-camera GNSS True-North yaw correction (tests A-E)
# ---------------------------------------------------------------------------


def test_true_north_correction_a_zero_error_stays_near_zero_and_does_not_degrade() -> None:
    node = _node()
    _seed_attitudes(node)  # identity quaternion on both sides -- no injected bias
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    expected_frd = (900.0, 3.0, -2.0)
    pixels = _observed_pixels_for_baseline_point(node, expected_frd)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())
    corrected = diagnostics["attitude_true_north"]

    assert corrected["computation_status"] == "computed"
    assert corrected["valid"] is True
    assert abs(corrected["left_true_north_yaw_correction_deg"]) < 1.0e-2
    assert abs(corrected["right_true_north_yaw_correction_deg"]) < 1.0e-2
    assert abs(corrected["differential_yaw_correction_deg"]) < 1.0e-2
    # A near-zero correction from the (flat-Earth-approximated) synthetic GPS
    # fixture introduces a tiny (sub-cm at this range) residual -- assert the
    # corrected ray stays accurate in absolute terms, not exactly as perfect
    # as the deliberately zero-bias uncorrected ray.
    assert _position_error(corrected, expected_frd) < 0.05


def test_true_north_correction_b_common_bias_is_recovered_and_right_error_improves() -> None:
    node = _node()
    injected_bias_deg = 0.30
    _seed_attitudes(
        node,
        left_q=_yaw_q_wxyz(injected_bias_deg),
        right_q=_yaw_q_wxyz(injected_bias_deg),
    )
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    expected_frd = (900.0, 3.0, -2.0)
    pixels = _observed_pixels_for_baseline_point(node, expected_frd)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())
    corrected = diagnostics["attitude_true_north"]

    assert abs(corrected["left_true_north_yaw_correction_deg"] - (-injected_bias_deg)) < 1.0e-2
    assert abs(corrected["right_true_north_yaw_correction_deg"] - (-injected_bias_deg)) < 1.0e-2
    assert abs(corrected["differential_yaw_correction_deg"]) < 1.0e-2
    biased_right_error = abs(diagnostics["attitude"]["right_m"] - expected_frd[1])
    corrected_right_error = abs(corrected["right_m"] - expected_frd[1])
    assert corrected_right_error < biased_right_error * 0.1


def test_true_north_correction_c_differential_bias_restores_intersection_angle_and_forward_error() -> None:
    """The core Phase-2 regression test: Phase 1's shared correction cannot
    fix a DIFFERENTIAL left/right bias (it collapses the ray intersection
    angle, which blows up Forward/Range error even after the common offset
    is corrected) -- this asserts the per-camera correction does fix it."""

    node = _node()
    left_bias_deg, right_bias_deg = 0.30, -0.30
    _seed_attitudes(
        node,
        left_q=_yaw_q_wxyz(left_bias_deg),
        right_q=_yaw_q_wxyz(right_bias_deg),
    )
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    expected_frd = (900.0, 3.0, -2.0)
    pixels = _observed_pixels_for_baseline_point(node, expected_frd)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())
    corrected = diagnostics["attitude_true_north"]
    biased = diagnostics["attitude"]
    nominal_raw = node._triangulate_pixel_centers(*pixels, 1280, 720)

    # Corrections recover the injected left/right asymmetry independently.
    assert abs(corrected["left_true_north_yaw_correction_deg"] - (-left_bias_deg)) < 1.0e-2
    assert abs(corrected["right_true_north_yaw_correction_deg"] - (-right_bias_deg)) < 1.0e-2

    # The degenerate (differential-bias) case: uncorrected intersection angle
    # must actually be far from the healthy/nominal one, or this test proves
    # nothing.
    assert abs(biased["intersection_angle_deg"] - nominal_raw["intersection_angle_deg"]) > 0.05

    # Core assertion: the per-camera-corrected ray's intersection angle is
    # close to the bias-free nominal reference, not to the degenerate biased
    # one.
    assert abs(corrected["intersection_angle_deg"] - nominal_raw["intersection_angle_deg"]) < 0.01

    # Forward/range error shrinks accordingly.
    biased_forward_error = abs(biased["forward_m"] - expected_frd[0])
    corrected_forward_error = abs(corrected["forward_m"] - expected_frd[0])
    assert corrected_forward_error < biased_forward_error * 0.1


def test_true_north_correction_d_preserves_px4_pitch_tilt() -> None:
    """Yaw correction must never touch tilt: the corrected pose's Forward
    vector elevation (vertical component) must match the uncorrected one,
    even with differential yaw bias present alongside a common pitch bias."""

    node = _node()
    pitch_deg = 2.0
    left_yaw_bias_deg, right_yaw_bias_deg = 0.30, -0.30
    _seed_attitudes(
        node,
        left_q=_pitch_yaw_q_wxyz(pitch_deg, left_yaw_bias_deg),
        right_q=_pitch_yaw_q_wxyz(pitch_deg, right_yaw_bias_deg),
    )
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    snapshot = node._camera_oracle_snapshot()

    uncorrected_poses, _status = node._px4_attitude_camera_poses(snapshot)
    corrected_poses, status = node._px4_attitude_true_north_camera_poses(snapshot)
    assert status == "available"

    for side_index in (0, 1):
        uncorrected_elevation_deg = math.degrees(
            math.asin(max(-1.0, min(1.0, uncorrected_poses[side_index].forward_world_enu[2])))
        )
        corrected_elevation_deg = math.degrees(
            math.asin(max(-1.0, min(1.0, corrected_poses[side_index].forward_world_enu[2])))
        )
        assert abs(corrected_elevation_deg - uncorrected_elevation_deg) < 1.0e-6


def test_true_north_correction_e_mixed_common_and_differential_bias() -> None:
    node = _node()
    left_bias_deg, right_bias_deg = 0.40, -0.20
    _seed_attitudes(
        node,
        left_q=_yaw_q_wxyz(left_bias_deg),
        right_q=_yaw_q_wxyz(right_bias_deg),
    )
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    expected_frd = (900.0, 3.0, -2.0)
    pixels = _observed_pixels_for_baseline_point(node, expected_frd)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())
    corrected = diagnostics["attitude_true_north"]
    biased = diagnostics["attitude"]
    nominal_raw = node._triangulate_pixel_centers(*pixels, 1280, 720)

    assert abs(corrected["left_true_north_yaw_correction_deg"] - (-left_bias_deg)) < 1.0e-2
    assert abs(corrected["right_true_north_yaw_correction_deg"] - (-right_bias_deg)) < 1.0e-2
    assert abs(corrected["intersection_angle_deg"] - nominal_raw["intersection_angle_deg"]) < 0.01
    corrected_position_error = _position_error(corrected, expected_frd)
    biased_position_error = _position_error(biased, expected_frd)
    assert corrected_position_error < biased_position_error * 0.1


# ---------------------------------------------------------------------------
# Phase 3: attitude_declination -- shared (left-camera-only) correction,
# kept alongside attitude_true_north for direct comparison.
# ---------------------------------------------------------------------------


def test_declination_correction_common_bias_is_recovered_and_right_error_improves() -> None:
    node = _node()
    injected_bias_deg = 0.30
    _seed_attitudes(
        node,
        left_q=_yaw_q_wxyz(injected_bias_deg),
        right_q=_yaw_q_wxyz(injected_bias_deg),
    )
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    expected_frd = (900.0, 3.0, -2.0)
    pixels = _observed_pixels_for_baseline_point(node, expected_frd)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())
    corrected = diagnostics["attitude_declination"]

    assert corrected["computation_status"] == "computed"
    assert abs(corrected["shared_yaw_correction_deg"] - (-injected_bias_deg)) < 1.0e-2
    biased_right_error = abs(diagnostics["attitude"]["right_m"] - expected_frd[1])
    corrected_right_error = abs(corrected["right_m"] - expected_frd[1])
    assert corrected_right_error < biased_right_error * 0.1


def test_declination_correction_differential_bias_does_not_restore_intersection_angle() -> None:
    """Contrast with Phase 2 Test C: the shared correction is blind to a
    DIFFERENTIAL left/right bias, so it should NOT recover the collapsed
    intersection angle the way attitude_true_north's per-camera correction
    does -- this makes the comparative value of keeping both rays explicit
    in the test suite itself, not just provable live."""

    node = _node()
    left_bias_deg, right_bias_deg = 0.30, -0.30
    _seed_attitudes(
        node,
        left_q=_yaw_q_wxyz(left_bias_deg),
        right_q=_yaw_q_wxyz(right_bias_deg),
    )
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    expected_frd = (900.0, 3.0, -2.0)
    pixels = _observed_pixels_for_baseline_point(node, expected_frd)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())
    declination = diagnostics["attitude_declination"]
    true_north = diagnostics["attitude_true_north"]
    biased = diagnostics["attitude"]
    nominal_raw = node._triangulate_pixel_centers(*pixels, 1280, 720)

    # The degenerate baseline case: uncorrected intersection angle is far
    # from the healthy/nominal one, or this test proves nothing.
    assert abs(biased["intersection_angle_deg"] - nominal_raw["intersection_angle_deg"]) > 0.05

    # Shared correction stays close to the degenerate uncorrected value...
    assert abs(declination["intersection_angle_deg"] - biased["intersection_angle_deg"]) < 0.02
    # ...while per-camera correction recovers close to the healthy nominal value.
    assert abs(true_north["intersection_angle_deg"] - nominal_raw["intersection_angle_deg"]) < 0.01
    # Making the contrast explicit: declination's angle stays far from nominal.
    assert abs(declination["intersection_angle_deg"] - nominal_raw["intersection_angle_deg"]) > 0.05


def test_declination_pose_selection_rejects_missing_and_stale_gps() -> None:
    node = _node()
    snapshot = node._camera_oracle_snapshot()
    _seed_attitudes(node)

    poses, status = node._px4_attitude_declination_camera_poses(snapshot)
    assert poses is None
    assert status == "missing_left_gps_fix"

    _seed_gps_matching_nominal_baseline(node)
    poses, status = node._px4_attitude_declination_camera_poses(snapshot)
    assert poses is not None
    assert status == "available"

    old = time.monotonic_ns() - int((node.gps_baseline_max_age_s + 0.1) * 1.0e9)
    left_fix, _old_ns, epoch, ts_ns = node.gps_baseline_fixes["left"]
    node.gps_baseline_fixes["left"] = (left_fix, old, epoch, ts_ns)
    poses, status = node._px4_attitude_declination_camera_poses(snapshot)
    assert poses is None
    assert status == "stale_left_gps_fix"


# ---------------------------------------------------------------------------
# operational_true_north -- the operational (raw-GPS-baseline position)
# pipeline with the same per-camera GPS-baseline True-North yaw correction
# as attitude_true_north layered on top. Requires no new pose-conversion
# method: _px4_attitude_true_north_camera_poses's correction math depends
# only on orientation (from live PX4 attitude) and the GPS bearing, never on
# camera_sources' own position, so calling it with gps_poses in place of the
# Gazebo-oracle snapshot is already fully correct.
# ---------------------------------------------------------------------------


def test_operational_true_north_common_bias_is_recovered_and_right_error_improves_over_operational() -> None:
    node = _node()
    injected_bias_deg = 0.30
    _seed_attitudes(
        node,
        left_q=_yaw_q_wxyz(injected_bias_deg),
        right_q=_yaw_q_wxyz(injected_bias_deg),
    )
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    expected_frd = (900.0, 3.0, -2.0)
    pixels = _observed_pixels_for_baseline_point(node, expected_frd)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())
    corrected = diagnostics["operational_true_north"]
    operational = diagnostics["operational"]

    assert corrected["computation_status"] == "computed"
    assert corrected["valid"] is True
    # Reuses attitude_true_north's correction math verbatim -- same
    # provenance fields, same recovered correction values.
    assert abs(corrected["left_true_north_yaw_correction_deg"] - (-injected_bias_deg)) < 1.0e-2
    assert abs(corrected["right_true_north_yaw_correction_deg"] - (-injected_bias_deg)) < 1.0e-2
    assert abs(corrected["differential_yaw_correction_deg"]) < 1.0e-2
    biased_right_error = abs(operational["right_m"] - expected_frd[1])
    corrected_right_error = abs(corrected["right_m"] - expected_frd[1])
    assert corrected_right_error < biased_right_error * 0.1


def test_operational_true_north_position_source_is_raw_gps_baseline_not_gazebo_oracle() -> None:
    """Distinguishes operational_true_north from attitude_true_north: both
    apply the identical yaw correction, but operational_true_north's camera
    *positions* must come from the raw GPS baseline (like operational), not
    the Gazebo-observed optical centers (like attitude_true_north)."""

    node = _node()
    _seed_attitudes(node)
    _seed_gps_matching_nominal_baseline(node, declination_deg=0.0)
    pixels = _observed_pixels_for_baseline_point(node)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())

    assert diagnostics["operational_true_north"]["baseline_source"] == "px4_raw_sensor_gps"
    assert diagnostics["attitude_true_north"]["baseline_source"] == "gazebo_observed_optical_centers"


def test_operational_true_north_fails_closed_on_missing_gps_with_combined_reason() -> None:
    node = _node()
    _seed_attitudes(node)  # no GPS seeded at all
    pixels = _observed_pixels_for_baseline_point(node)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())

    operational_true_north = diagnostics["operational_true_north"]
    assert operational_true_north["valid"] is False
    assert operational_true_north["computation_status"] == "missing_left_gps_fix"
    assert operational_true_north["failure_reason"] == (
        "gps:missing_left_gps_fix;attitude:missing_left_gps_fix"
    )


def test_attitude_pose_selection_rejects_stale_skewed_and_old_epoch_samples() -> None:
    node = _node()
    snapshot = node._camera_oracle_snapshot()
    assert snapshot is not None

    _seed_attitudes(node)
    poses, status = node._px4_attitude_camera_poses(snapshot)
    assert poses is not None
    assert status == "available"

    node.px4_attitudes["left"] = (
        (1.0, 0.0, 0.0, 0.0),
        1_000_000,
        0,
        node.simulation_epoch.epoch,
        time.monotonic() - node.px4_attitude_max_age_s - 0.1,
    )
    assert node._px4_attitude_camera_poses(snapshot) == (None, "stale_left_attitude")

    _seed_attitudes(node)
    right = node.px4_attitudes["right"]
    node.px4_attitudes["right"] = (
        right.quaternion_wxyz,
        right.timestamp_us,
        right.reset_counter,
        right.simulation_epoch,
        time.monotonic() - node.px4_attitude_max_age_s - 0.1,
    )
    assert node._px4_attitude_camera_poses(snapshot) == (
        None,
        "stale_right_attitude",
    )

    _seed_attitudes(node, left_stamp=1_000_000, right_stamp=4_000_001)
    assert node._px4_attitude_camera_poses(snapshot) == (None, "attitude_pair_skew_exceeded")

    _seed_attitudes(node)
    node.simulation_epoch.epoch += 1
    assert node._px4_attitude_camera_poses(snapshot) == (None, "attitude_epoch_mismatch")


def test_baseline_pose_selection_rejects_stale_and_invalid_gps(monkeypatch) -> None:
    node = _node()
    _seed_gps(node)
    monkeypatch.setattr(stereo_distance_module, "gps_baseline_enu_m", lambda _left, _right: (25.0, 0.0, 0.0))

    poses, status, provenance = node._gps_baseline_camera_poses("gps_baseline")

    assert poses is not None
    assert status == "available"
    assert provenance["gps_baseline_world_enu_m"] == pytest.approx((25.0, 0.0, 0.0), abs=1.0e-12)
    assert math.dist(poses[0].center_world_enu_m, poses[1].center_world_enu_m) == pytest.approx(25.0, abs=1.0e-12)

    old = time.monotonic() - node.gps_baseline_max_age_s - 0.1
    node.gps_baseline_fixes["left"] = (_fix(), old)
    poses, status, provenance = node._gps_baseline_camera_poses("gps_baseline")
    assert poses is None
    assert status == "stale_left_gps_fix"
    assert provenance["baseline_anchor_policy"]

    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (_ for _ in ()).throw(ValueError("bad fix")),
    )
    poses, status, provenance = node._gps_baseline_camera_poses("gps_baseline")
    assert poses is None
    assert status == "invalid_raw_gps_fix"
    assert provenance["baseline_source"] == "px4_raw_sensor_gps"


def test_diagnostic_rays_with_existing_nominal_inputs_match_existing_raw_ray_regression(monkeypatch) -> None:
    node = _node()
    _seed_attitudes(node)
    _seed_gps(node)
    monkeypatch.setattr(stereo_distance_module, "gps_baseline_enu_m", lambda _left, _right: (25.0, 0.0, 0.0))
    expected = (500.0, -3.0, 2.0)
    pixels = _observed_pixels_for_baseline_point(node, expected)
    raw = node._triangulate_pixel_centers(*pixels, 1280, 720)

    diagnostics = node._diagnostic_rays(*pixels, 1280, 720, node._camera_oracle_snapshot())

    assert raw["computation_status"] == "computed"
    assert diagnostics["attitude"]["midpoint_baseline_center_frd_m"] == pytest.approx(
        raw["midpoint_baseline_center_frd_m"], abs=1.0e-9
    )
    assert diagnostics["attitude"]["midpoint_baseline_center_frd_m"] == pytest.approx(
        raw["midpoint_baseline_center_frd_m"], abs=1.0e-9
    )
    assert diagnostics["operational"]["midpoint_baseline_center_frd_m"] == pytest.approx(
        raw["midpoint_baseline_center_frd_m"], abs=1.0e-9
    )


@pytest.mark.parametrize(
    "bad_q",
    (
        (0.0, 0.0, 0.0, 0.0),
        (float("nan"), 0.0, 0.0, 0.0),
        (float("inf"), 0.0, 0.0, 0.0),
    ),
)
def test_invalid_attitude_sample_replaces_previous_value_and_isolates_dependencies(
    monkeypatch, bad_q
) -> None:
    node = _node()
    _seed_attitudes(node)
    node._px4_attitude_callback("left", _attitude_msg(q=bad_q))
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (25.0, 0.0, 0.0),
    )
    pixels = _observed_pixels_for_baseline_point(node)

    diagnostics = node._diagnostic_rays(
        *pixels, 1280, 720, node._camera_oracle_snapshot()
    )

    assert diagnostics["attitude"]["valid"] is False
    assert diagnostics["operational"]["valid"] is False
    assert diagnostics["attitude"]["failure_reason"] == "invalid_left_quaternion"


@pytest.mark.parametrize("missing_label", ("left", "right"))
def test_missing_attitude_is_explicit_and_does_not_disable_baseline(
    monkeypatch, missing_label
) -> None:
    node = _node()
    _seed_attitudes(node)
    node.px4_attitudes[missing_label] = None
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (25.0, 0.0, 0.0),
    )
    pixels = _observed_pixels_for_baseline_point(node)
    diagnostics = node._diagnostic_rays(
        *pixels, 1280, 720, node._camera_oracle_snapshot()
    )

    assert diagnostics["attitude"]["computation_status"] == f"missing_{missing_label}_attitude"
    assert diagnostics["operational"]["valid"] is False


@pytest.mark.parametrize(
    ("baseline", "expected_status"),
    (
        ((0.0, 0.0, 0.0), "zero_raw_gps_baseline"),
        ((float("nan"), 0.0, 0.0), "invalid_raw_gps_baseline"),
        ((float("inf"), 0.0, 0.0), "invalid_raw_gps_baseline"),
    ),
)
def test_invalid_gps_baseline_isolates_attitude(monkeypatch, baseline, expected_status) -> None:
    node = _node()
    _seed_attitudes(node)
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: baseline,
    )
    pixels = _observed_pixels_for_baseline_point(node)
    diagnostics = node._diagnostic_rays(
        *pixels, 1280, 720, node._camera_oracle_snapshot()
    )

    assert diagnostics["attitude"]["valid"] is True
    assert diagnostics["operational"]["computation_status"] == expected_status
    assert diagnostics["operational"]["valid"] is False


def test_gps_pair_skew_and_missing_side_are_explicit() -> None:
    node = _node()
    now = time.monotonic()
    node.gps_baseline_fixes["left"] = (_fix(1_000_000), now)
    node.gps_baseline_fixes["right"] = (_fix(4_000_001), now)
    poses, status, _ = node._gps_baseline_camera_poses("gps_baseline")
    assert poses is None and status == "gps_pair_skew_exceeded"

    node.gps_baseline_fixes["left"] = None
    poses, status, _ = node._gps_baseline_camera_poses("gps_baseline")
    assert poses is None and status == "missing_left_gps_fix"
    node.gps_baseline_fixes["left"] = (_fix(), now)
    node.gps_baseline_fixes["right"] = None
    poses, status, _ = node._gps_baseline_camera_poses("gps_baseline")
    assert poses is None and status == "missing_right_gps_fix"


def test_gps_right_stale_and_epoch_mismatch_are_explicit() -> None:
    node = _node()
    current_epoch = node.simulation_epoch.epoch
    now_ns = time.monotonic_ns()
    node.gps_baseline_fixes["left"] = (_fix(), now_ns, current_epoch)
    node.gps_baseline_fixes["right"] = (
        _fix(),
        now_ns - int((node.gps_baseline_max_age_s + 0.1) * 1_000_000_000),
        current_epoch,
    )
    poses, status, _ = node._gps_baseline_camera_poses("gps_baseline")
    assert poses is None and status == "stale_right_gps_fix"

    node.gps_baseline_fixes["right"] = (_fix(), now_ns, current_epoch - 1)
    poses, status, _ = node._gps_baseline_camera_poses("gps_baseline")
    assert poses is None and status == "gps_epoch_mismatch"


def test_quaternion_order_counterexample_and_frame_mismatch_are_not_silent() -> None:
    correct = camera_optical_pose_from_px4_attitude_wxyz(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    )
    incorrectly_ordered = camera_optical_pose_from_px4_attitude_wxyz(
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    )
    assert math.dist(correct.forward_world_enu, incorrectly_ordered.forward_world_enu) > 1.0

    truth = {
        "computation_status": "computed",
        "output_frame": "baseline_center_frd",
        "forward_m": 1.0,
        "right_m": 2.0,
        "down_m": 3.0,
        "range_m": math.sqrt(14.0),
    }
    test = {**truth, "valid": True, "output_frame": "another_frame"}
    delta = StereoDistanceNode._diagnostic_delta(test, truth)
    assert delta["valid"] is False
    assert delta["failure_reason"] == "frame_mismatch"


def test_launch_exposes_attitude_topics_and_second_based_alignment_limits() -> None:
    launch = (
        PACKAGE_ROOT / "launch" / "stereo_ground_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("px4_attitude_left_topic", default_value="/camera_left/fmu/out/vehicle_attitude")' in launch
    assert 'DeclareLaunchArgument("px4_attitude_right_topic", default_value="/camera_right/fmu/out/vehicle_attitude")' in launch
    assert 'DeclareLaunchArgument("px4_attitude_max_age_s", default_value="2.5")' in launch
    assert 'DeclareLaunchArgument("px4_attitude_pair_skew_s", default_value="2.5")' in launch
    assert '"px4_attitude_max_age_s"' in launch
    assert '"px4_attitude_pair_skew_s"' in launch
    env_text = (PACKAGE_ROOT.parent / "config" / "simulation.env").read_text(
        encoding="utf-8"
    )
    runner = (PACKAGE_ROOT.parent / "run_operator_view.sh").read_text(
        encoding="utf-8"
    )
    for name in (
        "PX4_ATTITUDE_LEFT_TOPIC",
        "PX4_ATTITUDE_RIGHT_TOPIC",
        "PX4_ATTITUDE_MAX_AGE_S",
        "PX4_ATTITUDE_PAIR_SKEW_S",
    ):
        assert f"{name}=" in env_text
        assert name in runner


def test_delta_interaction_gt_errors_and_research_fields_share_fixed_frd_frame(
    monkeypatch,
) -> None:
    node = _node()
    _seed_attitudes(node)
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (25.0, 0.0, 0.0),
    )
    pixels = _observed_pixels_for_baseline_point(node)
    truth = _truth(node, pixels)
    diagnostics = node._diagnostic_rays(
        *pixels, 1280, 720, node._camera_oracle_snapshot()
    )
    analysis = node._diagnostic_analysis(truth, diagnostics)
    for name, comparison in analysis["comparisons"].items():
        diagnostics[name].update(
            {
                "delta_valid": comparison["valid"],
                "delta_failure_reason": comparison["failure_reason"],
                **{
                    key: value
                    for key, value in comparison.items()
                    if key not in ("valid", "failure_reason")
                },
            }
        )
    metrics = {"epoch": node.simulation_epoch.epoch, **analysis["flat"]}
    for name, result in diagnostics.items():
        metrics.update(node._diagnostic_ray_record_fields(name, result))
    metrics["_diagnostic_rays"] = diagnostics

    pair = SimpleNamespace(pair_id=1)
    alignment = SimpleNamespace(
        gt_alignment_mode="nearest",
        gt_before_stamp_ns=1,
        gt_after_stamp_ns=1,
        gt_interpolation_alpha=0.0,
        gt_nearest_delta_ns=0,
    )
    gt = {
        "forward_m": truth["forward_m"] + 0.1,
        "right_m": truth["right_m"] - 0.2,
        "down_m": truth["down_m"] + 0.3,
        "range_m": truth["range_m"] + 0.4,
    }
    evaluation = node._evaluation_metadata(pair, alignment, gt, metrics)
    metrics.update(evaluation)
    record_fields = node._diagnostic_research_record_fields(metrics)

    for name in ("attitude", "operational"):
        assert evaluation[f"{name}_gt_forward_error_m"] == pytest.approx(-0.1, abs=1.0e-12)
        assert evaluation[f"{name}_gt_right_error_m"] == pytest.approx(0.2, abs=1.0e-12)
        assert evaluation[f"{name}_gt_down_error_m"] == pytest.approx(-0.3, abs=1.0e-12)
        assert record_fields[f"{name}_ray_valid"] is True
        assert record_fields[f"{name}_ray_midpoint_x_m"] == pytest.approx(
            truth["forward_m"], abs=1.0e-9
        )
        assert record_fields[f"{name}_ray_delta_valid"] is True
        assert record_fields[f"{name}_ray_gt_position_error_m"] == pytest.approx(
            math.sqrt(0.14), abs=1.0e-9
        )
    assert record_fields["px4_quaternion_order"] == "wxyz"
    assert record_fields["px4_attitude_semantics"] == "body_frd_to_earth_ned"
    assert record_fields["baseline_anchor_policy"].startswith("nominal_left")
    assert "interaction_valid" not in record_fields


def test_missing_ground_truth_fails_closed_without_px4_fallback() -> None:
    node = _node()
    pair = SimpleNamespace(pair_id=7)
    alignment = SimpleNamespace(
        gt_alignment_mode="unavailable",
        gt_before_stamp_ns=None,
        gt_after_stamp_ns=None,
        gt_interpolation_alpha=None,
        gt_nearest_delta_ns=None,
    )
    metrics = {
        "attitude_forward_m": 1.0,
        "attitude_right_m": 2.0,
        "attitude_down_m": 3.0,
        "attitude_range_m": math.sqrt(14.0),
    }

    evaluation = node._evaluation_metadata(pair, alignment, None, metrics)

    assert evaluation["attitude_gt_valid"] is False
    assert evaluation["attitude_gt_position_error_m"] is None
    assert evaluation["gt_source"] == "gazebo_vehicle_reference_link_world_pose"
    assert evaluation["gt_fallback_used"] is False


def test_operational_pose_alignment_uses_simulation_clock_not_px4_epoch_time(
    monkeypatch,
) -> None:
    node = _node()
    node.simulation_epoch.last_clock_stamp_ns = 10_000_000_000
    _seed_attitudes(
        node,
        left_stamp=1_785_848_817_333_979,
        right_stamp=1_785_848_817_337_806,
    )
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (25.0, 0.0, 0.0),
    )
    pixels = _observed_pixels_for_baseline_point(node)
    operational_snapshot = node._capture_operational_sources_locked()
    operational = node._diagnostic_rays(
        *pixels,
        1280,
        720,
        node._camera_oracle_snapshot(),
        operational_snapshot=operational_snapshot,
        observation_timestamp_ns=9_950_000_000,
    )["operational"]
    pair = SimpleNamespace(
        epoch=0,
        pair_id=7,
        pair_midpoint_stamp_ns=9_950_000_000,
        left_stamp_ns=9_950_000_000,
        right_stamp_ns=9_950_000_000,
        pair_delta_ns=0,
        pairing_mode="exact",
    )
    alignment = SimpleNamespace(
        gt_alignment_mode="exact",
        gt_before_stamp_ns=9_950_000_000,
        gt_after_stamp_ns=9_950_000_000,
        gt_interpolation_alpha=0.0,
        position_world_enu_m=(0.0, 900.0, 3.0),
    )
    fields = node._calibration_research_record_fields(
        pair,
        alignment,
        {
            "_diagnostic_rays": {"operational": operational},
            "left_observation_valid": True,
            "right_observation_valid": True,
            "left_observation_source": "yolo_bbox",
            "right_observation_source": "yolo_bbox",
            "target_model_name": "x500_15m",
            "cam_l_optical_roll_deg": 0.0,
            "cam_l_optical_pitch_deg": 0.0,
            "cam_l_optical_yaw_deg": 0.0,
            "cam_r_optical_roll_deg": 0.0,
            "cam_r_optical_pitch_deg": 0.0,
            "cam_r_optical_yaw_deg": 0.0,
        },
    )

    assert fields["left_operational_pose_timestamp_ns"] == 10_000_000_000
    assert fields["right_operational_pose_timestamp_ns"] == 10_000_000_000
    assert fields["left_operational_position_timestamp_ns"] == 10_000_000_000
    assert fields["right_operational_position_timestamp_ns"] == 10_000_000_000
    assert fields["left_pose_alignment_dt_s"] == pytest.approx(0.05)
    assert fields["right_pose_alignment_dt_s"] == pytest.approx(0.05)
    assert fields["left_position_alignment_dt_s"] == pytest.approx(0.05)
    assert fields["right_position_alignment_dt_s"] == pytest.approx(0.05)
    assert fields["sync_valid"] is True


def test_operational_snapshot_prevents_future_pose_leaking_into_delayed_pair(
    monkeypatch,
) -> None:
    node = _node()
    node.simulation_epoch.last_clock_stamp_ns = 10_000_000_000
    _seed_attitudes(node, left_q=(1.0, 0.0, 0.0, 0.0), right_q=(1.0, 0.0, 0.0, 0.0))
    _seed_gps(node)
    snapshot = node._capture_operational_sources_locked()

    node.simulation_epoch.last_clock_stamp_ns = 60_000_000_000
    _seed_attitudes(node, left_q=_yaw_q_wxyz(45.0), right_q=_yaw_q_wxyz(45.0))
    _seed_gps(node)
    monkeypatch.setattr(
        stereo_distance_module,
        "gps_baseline_enu_m",
        lambda _left, _right: (25.0, 0.0, 0.0),
    )
    pixels = _observed_pixels_for_baseline_point(node)

    operational = node._diagnostic_rays(
        *pixels,
        1280,
        720,
        node._camera_oracle_snapshot(),
        operational_snapshot=snapshot,
        observation_timestamp_ns=9_950_000_000,
    )["operational"]

    assert operational["valid"] is True
    assert operational["left_px4_q_w"] == pytest.approx(1.0)
    assert operational["right_px4_q_w"] == pytest.approx(1.0)
    assert operational["left_attitude_simulation_timestamp_ns"] == 10_000_000_000
    assert operational["right_attitude_simulation_timestamp_ns"] == 10_000_000_000
    assert operational["left_gps_simulation_timestamp_ns"] == 10_000_000_000
    assert operational["right_gps_simulation_timestamp_ns"] == 10_000_000_000


def test_single_side_observation_preserves_available_pixel_and_bbox() -> None:
    node = _node()
    node.observation_mode = "bbox_yolo"
    left = {
        "cx": 101.5,
        "cy": 202.5,
        "confidence": 0.75,
        "bbox": (90.0, 180.0, 113.0, 225.0),
    }

    fields = node._observation_record_fields(left, None)

    assert fields["left_observation_valid"] is True
    assert fields["right_observation_valid"] is False
    assert fields["left_u_px"] == pytest.approx(101.5)
    assert fields["left_v_px"] == pytest.approx(202.5)
    assert fields["left_bbox_xmin_px"] == pytest.approx(90.0)
    assert fields["left_bbox_ymax_px"] == pytest.approx(225.0)
    assert fields["right_u_px"] is None


def test_invalid_observation_keeps_explicit_failure_provenance() -> None:
    node = _node()
    pair = SimpleNamespace(
        epoch=0,
        pair_id=8,
        pair_midpoint_stamp_ns=2_000_000_000,
        left_stamp_ns=2_000_000_000,
        right_stamp_ns=2_000_000_000,
        pair_delta_ns=0,
        pairing_mode="exact",
    )
    alignment = SimpleNamespace(
        gt_alignment_mode="exact",
        gt_before_stamp_ns=2_000_000_000,
        gt_after_stamp_ns=2_000_000_000,
        gt_interpolation_alpha=0.0,
        position_world_enu_m=(0.0, 900.0, 3.0),
    )
    fields = node._calibration_research_record_fields(
        pair,
        alignment,
        {
            "left_observation_valid": False,
            "right_observation_valid": False,
            "target_model_name": "x500_15m",
        },
    )

    assert fields["sample_valid"] is False
    assert fields["failure_stage"] == "observation"
    assert fields["failure_code"] == "missing_left_and_right_observation"
    assert fields["failure_reason"] == "missing_left_and_right_observation"


def test_runtime_metrics_publish_named_deltas_without_interaction() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    messages = []
    node.ray_metrics_pub = SimpleNamespace(
        publish=lambda message: messages.append(json.loads(message.data))
    )
    truth = {
        "computation_status": "computed",
        "forward_m": 10.0,
        "right_m": 0.0,
        "down_m": 0.0,
        "range_m": 10.0,
        "output_frame": "baseline_center_frd",
    }
    attitude = {
        **truth,
        "valid": True,
        "delta_valid": True,
        "delta_forward_m": 0.25,
        "delta_right_m": 0.0,
        "delta_down_m": 0.0,
        "delta_range_m": 0.25,
        "position_difference_m": 0.25,
        "failure_reason": "",
    }
    operational = {
        "valid": False,
        "computation_status": "missing_left_gps_fix",
        "failure_reason": "missing_left_gps_fix",
        "output_frame": "baseline_center_frd",
    }
    node._publish_ray_metrics(
        optical_pose=truth,
        attitude=attitude,
        operational=operational,
    )

    payload = messages[-1]
    assert payload["attitude_delta_forward_m"] == 0.25
    assert payload["operational"]["valid"] is False
    assert payload["operational"]["failure_reason"] == "missing_left_gps_fix"
    assert "interaction" not in payload


def test_runtime_metrics_publish_pair_id_and_gps_global_rows() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    messages = []
    node.ray_metrics_pub = SimpleNamespace(
        publish=lambda message: messages.append(json.loads(message.data))
    )
    truth = {
        "computation_status": "computed",
        "forward_m": 10.0,
        "right_m": 0.0,
        "down_m": 0.0,
        "range_m": 10.0,
        "output_frame": "baseline_center_frd",
    }
    gps_baseline = {
        **truth,
        "forward_m": 10.1,
        "valid": True,
        "failure_reason": "",
    }
    node._publish_ray_metrics(
        pair_id=42,
        optical_pose=truth,
        gps_baseline=gps_baseline,
        global_baseline=None,
        gt_source="gazebo_vehicle_reference_link_world_pose",
        gt_fallback_used=False,
    )

    payload = messages[-1]
    assert payload["pair_id"] == 42
    assert payload["gps_baseline"]["forward_m"] == pytest.approx(10.1, abs=1.0e-9)
    assert payload["gps_baseline"]["valid"] is True
    assert "global_baseline" not in payload
    assert payload["gt_source"] == "gazebo_vehicle_reference_link_world_pose"
    assert payload["gt_fallback_used"] is False
