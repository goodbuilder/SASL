from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


def _parameter(name: str, value):
    return SimpleNamespace(name=name, value=value)


def _node_for_parameter_test() -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.reference_distance_m = 45.0
    node.approach_start_distance_m = 2000.0
    node.approach_end_distance_m = 300.0
    node.approach_duration_s = 120.0
    node.target_spawn_z_m = 0.75
    node.fly_target_in_gazebo = False
    node.baseline_center_frd = object()
    node.nominal_left_camera = object()
    node.nominal_right_camera = object()
    node.actual_left_camera = object()
    node.actual_right_camera = object()
    return node


def test_geometry_parameter_batch_is_rejected_before_any_mutation() -> None:
    node = _node_for_parameter_test()
    before = {
        name: getattr(node, name)
        for name in (
            "baseline_m",
            "camera_height_m",
            "horizontal_fov_rad",
            "baseline_center_frd",
            "nominal_left_camera",
            "nominal_right_camera",
            "actual_left_camera",
            "actual_right_camera",
        )
    }

    result = node._on_param_update(
        [
            _parameter("reference_distance_m", 999.0),
            _parameter("baseline_m", 40.0),
            _parameter("left_yaw_error_deg", -0.25),
            _parameter("common_optical_pitch_error_deg", 0.1),
            _parameter("relative_optical_yaw_error_deg", 0.5),
        ]
    )

    assert result.successful is False
    assert result.reason == "geometry-defining parameters are immutable for one run"
    for name, value in before.items():
        current = getattr(node, name)
        if isinstance(value, (float, int)):
            assert current == value
        else:
            assert current is value
    assert node.reference_distance_m == 45.0


def test_each_geometry_parameter_is_rejected_and_unrelated_parameter_remains_allowed() -> None:
    geometry_names = (
        "baseline_m",
        "camera_height_m",
        "horizontal_fov_rad",
        "target_model_name",
        "left_roll_error_deg",
        "left_pitch_error_deg",
        "left_yaw_error_deg",
        "right_roll_error_deg",
        "right_pitch_error_deg",
        "right_yaw_error_deg",
        "common_optical_roll_error_deg",
        "relative_optical_roll_error_deg",
        "common_optical_pitch_error_deg",
        "relative_optical_pitch_error_deg",
        "common_optical_yaw_error_deg",
        "relative_optical_yaw_error_deg",
    )
    for name in geometry_names:
        node = _node_for_parameter_test()
        result = node._on_param_update([_parameter(name, 1.0)])
        assert result.successful is False
        assert result.reason == "geometry-defining parameters are immutable for one run"

    node = _node_for_parameter_test()
    result = node._on_param_update([_parameter("reference_distance_m", 77.0)])
    assert result.successful is True
    assert node.reference_distance_m == 77.0
