from __future__ import annotations

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import (
    CameraOpticalPose,
    apply_optical_rpy_error,
    apply_optical_yaw,
    camera_optical_pose_from_sdf_pose,
    effective_optical_rpy_errors_deg,
    nominal_camera_poses,
    optical_rpy_error_rotation,
    project_world_point_to_pixel,
    rotation_to_sdf_rpy,
    rpy_to_rotation_world_from_link,
    sdf_link_rpy_from_optical_error,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from camera_oracle_test_support import seed_observed_camera_oracle


AXES = ("right_world_enu", "down_world_enu", "forward_world_enu")
MIXED_DIRECT_LEFT = (0.03, 0.01, -0.02)
MIXED_DIRECT_RIGHT = (-0.02, 0.04, 0.01)
MIXED_COMMON = (0.02, -0.04, 0.10)
MIXED_RELATIVE = (0.01, 0.02, 0.04)
MIXED_EFFECTIVE_LEFT = (0.045, -0.040, 0.060)
MIXED_EFFECTIVE_RIGHT = (0.005, 0.010, 0.130)


def _dot(left, right):
    return sum(a * b for a, b in zip(left, right))


def _determinant(matrix):
    a, b, c = matrix
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


def _basis_columns(pose):
    return tuple(getattr(pose, name) for name in AXES)


def _max_pose_error(left, right):
    return max(
        abs(a - b)
        for name in AXES
        for a, b in zip(getattr(left, name), getattr(right, name))
    )


def _node(left=(0.0, 0.0, 0.0), right=(0.0, 0.0, 0.0), common=(0.0, 0.0, 0.0), relative=(0.0, 0.0, 0.0)):
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.synthetic_detection_bbox_size_px = 32.0
    for prefix, values in (("cam_l_optical", left), ("cam_r_optical", right)):
        setattr(node, f"{prefix}_roll_deg", values[0])
        setattr(node, f"{prefix}_pitch_deg", values[1])
        setattr(node, f"{prefix}_yaw_deg", values[2])
    for prefix in ("cam_l_px4", "cam_r_px4"):
        setattr(node, f"{prefix}_roll_deg", 0.0)
        setattr(node, f"{prefix}_pitch_deg", 0.0)
        setattr(node, f"{prefix}_yaw_deg", 0.0)
    for prefix, values in (("common_optical", common), ("relative_optical", relative)):
        setattr(node, f"{prefix}_roll_error_deg", values[0])
        setattr(node, f"{prefix}_pitch_error_deg", values[1])
        setattr(node, f"{prefix}_yaw_error_deg", values[2])
    node._ensure_camera_frames()
    return node


def _position_error(result, expected):
    return math.sqrt(
        sum(
            (result[name] - expected[index]) ** 2
            for index, name in enumerate(("forward_m", "right_m", "down_m"))
        )
    )


def test_effective_direct_common_relative_contract() -> None:
    left, right = effective_optical_rpy_errors_deg(
        MIXED_DIRECT_LEFT,
        MIXED_DIRECT_RIGHT,
        MIXED_COMMON,
        MIXED_RELATIVE,
    )
    assert left == pytest.approx(MIXED_EFFECTIVE_LEFT, abs=1.0e-15)
    assert right == pytest.approx(MIXED_EFFECTIVE_RIGHT, abs=1.0e-15)
    common_left, common_right = effective_optical_rpy_errors_deg(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.1, -0.2, 0.3),
        (0.0, 0.0, 0.0),
    )
    assert common_left == pytest.approx(common_right)
    relative_left, relative_right = effective_optical_rpy_errors_deg(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.2, -0.4, 0.6),
    )
    assert relative_left == pytest.approx((-0.1, 0.2, -0.3))
    assert relative_right == pytest.approx((0.1, -0.2, 0.3))


def test_zero_rotation_is_identity_and_preserves_center_and_basis_quality() -> None:
    nominal, _ = nominal_camera_poses(25.0, 5.0)
    actual = apply_optical_rpy_error(nominal, 0.0, 0.0, 0.0)
    assert actual == nominal
    columns = _basis_columns(actual)
    assert all(_dot(axis, axis) == pytest.approx(1.0, abs=1.0e-12) for axis in columns)
    assert all(abs(_dot(columns[a], columns[b])) <= 1.0e-12 for a, b in ((0, 1), (0, 2), (1, 2)))
    matrix_rows = tuple(tuple(columns[column][row] for column in range(3)) for row in range(3))
    assert _determinant(matrix_rows) == pytest.approx(1.0, abs=1.0e-12)


@pytest.mark.parametrize(
    ("rpy_deg", "fixed_axis", "moving_axis", "positive_axis", "direction"),
    (
        ((0.1, 0.0, 0.0), "forward_world_enu", "right_world_enu", "down_world_enu", 1.0),
        ((0.0, 0.1, 0.0), "right_world_enu", "forward_world_enu", "down_world_enu", -1.0),
        ((0.0, 0.0, 0.1), "down_world_enu", "forward_world_enu", "right_world_enu", 1.0),
    ),
)
def test_positive_single_axis_optical_signs(rpy_deg, fixed_axis, moving_axis, positive_axis, direction) -> None:
    nominal, _ = nominal_camera_poses(25.0, 5.0)
    actual = apply_optical_rpy_error(nominal, *(math.radians(value) for value in rpy_deg))
    assert getattr(actual, fixed_axis) == pytest.approx(getattr(nominal, fixed_axis), abs=1.0e-12)
    projection = _dot(getattr(actual, moving_axis), getattr(nominal, positive_axis))
    assert projection * direction > 0.0
    assert actual.center_world_enu_m == nominal.center_world_enu_m


def test_mixed_rotation_matches_independent_hard_coded_matrix() -> None:
    expected = (
        (0.9999991426895518, -0.0007861287333982878, 0.0010471971046042026),
        (0.0007853978912551963, 0.9999994478810276, 0.0006981316440875792),
        (-0.001047745347771925, -0.000697308579174342, 0.9999992079949023),
    )
    actual = optical_rpy_error_rotation(*(math.radians(value) for value in MIXED_EFFECTIVE_LEFT))
    for actual_row, expected_row in zip(actual, expected):
        assert actual_row == pytest.approx(expected_row, abs=1.0e-15)


def test_yaw_wrapper_is_exact_full_rpy_compatibility_path() -> None:
    nominal, _ = nominal_camera_poses(25.0, 5.0)
    yaw = math.radians(0.1)
    legacy = apply_optical_yaw(nominal, yaw)
    full = apply_optical_rpy_error(nominal, 0.0, 0.0, yaw)
    assert _max_pose_error(legacy, full) <= 1.0e-12
    target = nominal.center_world_enu_m[0], nominal.center_world_enu_m[1] + 1000.0, 3.0
    legacy_pixel = project_world_point_to_pixel(target, legacy, fx_px=8200.0, fy_px=8200.0, cx_px=640.0, cy_px=360.0)
    full_pixel = project_world_point_to_pixel(target, full, fx_px=8200.0, fy_px=8200.0, cx_px=640.0, cy_px=360.0)
    assert legacy_pixel == pytest.approx(full_pixel, abs=1.0e-9)


@pytest.mark.parametrize("invalid", (float("nan"), float("inf"), float("-inf")))
def test_nonfinite_and_invalid_vector_inputs_are_rejected(invalid) -> None:
    nominal, _ = nominal_camera_poses(25.0, 5.0)
    with pytest.raises(ValueError):
        apply_optical_rpy_error(nominal, invalid, 0.0, 0.0)
    with pytest.raises(ValueError):
        effective_optical_rpy_errors_deg((0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    invalid_pose = CameraOpticalPose(
        center_world_enu_m=nominal.center_world_enu_m,
        right_world_enu=(2.0, 0.0, 0.0),
        down_world_enu=nominal.down_world_enu,
        forward_world_enu=nominal.forward_world_enu,
    )
    with pytest.raises(ValueError):
        apply_optical_rpy_error(invalid_pose, 0.0, 0.0, 0.0)


def test_actual_ray_world_rdf_round_trip() -> None:
    nominal, _ = nominal_camera_poses(25.0, 5.0)
    actual = apply_optical_rpy_error(nominal, *(math.radians(value) for value in MIXED_EFFECTIVE_LEFT))
    ray_rdf = (0.05, -0.02, 1.0)
    norm = math.sqrt(sum(value * value for value in ray_rdf))
    expected = tuple(value / norm for value in ray_rdf)
    ray_world = actual.ray_to_world(ray_rdf)
    world_point = tuple(actual.center_world_enu_m[index] + ray_world[index] for index in range(3))
    recovered = actual.world_to_rdf(world_point)
    assert recovered == pytest.approx(expected, abs=1.0e-12)


SO3_CASES = (
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((0.1, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.1), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, -0.1, 0.0)),
    (MIXED_DIRECT_LEFT, MIXED_DIRECT_RIGHT, MIXED_COMMON, MIXED_RELATIVE),
)


@pytest.mark.parametrize(("left_direct", "right_direct", "common", "relative"), SO3_CASES)
def test_sdf_link_rpy_and_node_pose_are_equivalent(left_direct, right_direct, common, relative) -> None:
    nominal_left, nominal_right = nominal_camera_poses(25.0, 5.0)
    effective_left, effective_right = effective_optical_rpy_errors_deg(left_direct, right_direct, common, relative)
    for nominal, effective in ((nominal_left, effective_left), (nominal_right, effective_right)):
        actual = apply_optical_rpy_error(nominal, *(math.radians(value) for value in effective))
        link_rpy = sdf_link_rpy_from_optical_error(
            math.pi / 2.0,
            *(math.radians(value) for value in effective),
        )
        from_sdf = camera_optical_pose_from_sdf_pose(nominal.center_world_enu_m, link_rpy)
        assert _max_pose_error(actual, from_sdf) <= 1.0e-12
        assert from_sdf.center_world_enu_m == pytest.approx(nominal.center_world_enu_m, abs=1.0e-12)


def _launch_module():
    path = PACKAGE_ROOT / "launch" / "stereo_ground_sim.launch.py"
    spec = importlib.util.spec_from_file_location("stereo_ground_sim_launch_so3_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _world_context(left_direct, right_direct, common, relative):
    values = {
        "baseline_m": 25.0,
        "camera_height_m": 5.0,
        "horizontal_fov_rad": 0.154997710698,
        "camera_image_width_px": 1280,
        "camera_image_height_px": 720,
        "camera_update_rate_hz": 20.0,
        "camera_clip_near_m": 0.1,
        "camera_clip_far_m": 2500.0,
        "approach_start_distance_m": 2000.0,
        "target_spawn_x_m": 0.0,
        "target_spawn_y_m": 2000.0,
        "target_spawn_z_m": 0.75,
        "target_spawn_yaw_deg": -90.0,
    }
    for prefix, rpy in (("cam_l_optical", left_direct), ("cam_r_optical", right_direct)):
        values[f"{prefix}_roll_deg"] = rpy[0]
        values[f"{prefix}_pitch_deg"] = rpy[1]
        values[f"{prefix}_yaw_deg"] = rpy[2]
    for prefix in ("cam_l_px4", "cam_r_px4"):
        values[f"{prefix}_roll_deg"] = 0.0
        values[f"{prefix}_pitch_deg"] = 0.0
        values[f"{prefix}_yaw_deg"] = 0.0
    for prefix, rpy in (("common_optical", common), ("relative_optical", relative)):
        values[f"{prefix}_roll_error_deg"] = rpy[0]
        values[f"{prefix}_pitch_error_deg"] = rpy[1]
        values[f"{prefix}_yaw_error_deg"] = rpy[2]
    context = LaunchContext()
    for name, value in values.items():
        context.launch_configurations[name] = str(value)
    return context


@pytest.mark.parametrize(("left_direct", "right_direct", "common", "relative"), SO3_CASES)
def test_generated_world_pose_round_trip_matches_node_desired_pose(monkeypatch, left_direct, right_direct, common, relative) -> None:
    launch_module = _launch_module()
    monkeypatch.setattr(launch_module, "get_package_share_directory", lambda _: str(PACKAGE_ROOT))
    world_path = launch_module._generate_world(_world_context(left_direct, right_direct, common, relative))
    root = ET.parse(world_path).getroot()
    models = {
        model.attrib["name"]: model
        for model in root.findall("./world/model")
        if model.attrib.get("name") in {"left_stereo_camera", "right_stereo_camera"}
    }
    node = _node(left_direct, right_direct, common, relative)
    for name, desired in (
        ("left_stereo_camera", node.desired_left_camera),
        ("right_stereo_camera", node.desired_right_camera),
    ):
        model = models[name]
        pose = tuple(float(value) for value in model.findtext("pose").split())
        optical = model.find("./link[@name='optical_link']")
        assert optical is not None
        optical_pose = tuple(float(value) for value in optical.findtext("pose").split())
        parent_rotation = rpy_to_rotation_world_from_link(*pose[3:])
        relative_rotation = rpy_to_rotation_world_from_link(*optical_pose[3:])
        world_rotation = tuple(
            tuple(
                sum(parent_rotation[row][k] * relative_rotation[k][column] for k in range(3))
                for column in range(3)
            )
            for row in range(3)
        )
        from_world = camera_optical_pose_from_sdf_pose(
            pose[:3], rotation_to_sdf_rpy(world_rotation)
        )
        assert _max_pose_error(desired, from_world) <= 1.0e-8
        assert from_world.center_world_enu_m == pytest.approx(desired.center_world_enu_m, abs=1.0e-12)
    assert node.baseline_center_frd.origin_world_enu_m == pytest.approx((0.0, 0.0, 5.0), abs=1.0e-12)
    assert math.dist(node.desired_left_camera.center_world_enu_m, node.desired_right_camera.center_world_enu_m) == pytest.approx(25.0, abs=1.0e-12)


@pytest.mark.parametrize(("left_direct", "right_direct", "common", "relative"), SO3_CASES)
def test_same_pixel_observed_projection_raw_and_corrected_contract(left_direct, right_direct, common, relative) -> None:
    expected = (1000.0, 50.0, -5.0)
    node = _node(left_direct, right_direct, common, relative)
    seed_observed_camera_oracle(node)
    point_world = node.baseline_center_frd.point_to_world(expected)
    left, right = node._ideal_detection_pair_from_reference_world(point_world, 1280, 720)
    assert left is not None and right is not None
    assert 0.0 <= left["cx"] < 1280.0 and 0.0 <= right["cx"] < 1280.0
    assert 0.0 <= left["cy"] < 720.0 and 0.0 <= right["cy"] < 720.0
    pixels = (left["cx"], left["cy"], right["cx"], right["cy"], 1280, 720)
    raw = node._triangulate_pixel_centers(*pixels)
    corrected = node._triangulate_pixel_centers_oracle(*pixels)
    assert raw["computation_status"] == "computed"
    assert corrected["computation_status"] == "computed"
    assert corrected["ray_gap_m"] <= 1.0e-6
    assert corrected["left_depth_m"] > 0.0 and corrected["right_depth_m"] > 0.0
    corrected_error = _position_error(corrected, expected)
    raw_error = _position_error(raw, expected)
    assert corrected_error <= 1.0e-6
    if any(abs(value) > 0.0 for values in (left_direct, right_direct, common, relative) for value in values):
        assert raw_error > max(corrected_error * 10.0, 1.0e-3)
    else:
        assert raw_error <= 1.0e-6


def test_node_effective_state_metadata_and_nominal_baseline_are_complete() -> None:
    zero = _node()
    mixed = _node(MIXED_DIRECT_LEFT, MIXED_DIRECT_RIGHT, MIXED_COMMON, MIXED_RELATIVE)
    assert mixed.baseline_center_frd == zero.baseline_center_frd
    metadata = mixed._pair_result_metadata(None, None, None, {"computation_status": "computed"}, count_alignment=False)
    expected_sources = {
        "cam_l_optical_roll_deg": 0.03,
        "cam_l_optical_pitch_deg": 0.01,
        "cam_l_optical_yaw_deg": -0.02,
        "cam_r_optical_roll_deg": -0.02,
        "cam_r_optical_pitch_deg": 0.04,
        "cam_r_optical_yaw_deg": 0.01,
        "common_optical_roll_error_deg": 0.02,
        "relative_optical_roll_error_deg": 0.01,
        "common_optical_pitch_error_deg": -0.04,
        "relative_optical_pitch_error_deg": 0.02,
        "common_optical_yaw_error_deg": 0.10,
        "relative_optical_yaw_error_deg": 0.04,
        "cam_l_px4_roll_deg": 0.0,
        "cam_l_px4_pitch_deg": 0.0,
        "cam_l_px4_yaw_deg": 0.0,
        "cam_r_px4_roll_deg": 0.0,
        "cam_r_px4_pitch_deg": 0.0,
        "cam_r_px4_yaw_deg": 0.0,
    }
    for name, value in expected_sources.items():
        assert metadata[name] == pytest.approx(value)
    assert tuple(metadata[f"effective_left_optical_{axis}_error_deg"] for axis in ("roll", "pitch", "yaw")) == pytest.approx(MIXED_EFFECTIVE_LEFT)
    assert tuple(metadata[f"effective_right_optical_{axis}_error_deg"] for axis in ("roll", "pitch", "yaw")) == pytest.approx(MIXED_EFFECTIVE_RIGHT)
    assert metadata["attitude_error_frame"] == "camera_optical_rdf"
    assert metadata["attitude_angle_unit"] == "deg"
    assert metadata["attitude_parameterization"] == "direct_plus_common_relative"
    assert metadata["attitude_rotation_composition"] == "R_Cnominal_Cactual=R_Down(yaw)@R_Right(pitch)@R_Forward(roll)"
    assert metadata["correction_source"] == "gazebo_observed_optical_pose"
    assert metadata["output_frame"] == "baseline_center_frd"
