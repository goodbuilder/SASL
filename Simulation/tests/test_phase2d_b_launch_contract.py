from __future__ import annotations

import importlib.util
import re
import sys
import tempfile
import types
import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "Simulation"
PACKAGE_SHARE = SIM_ROOT / "stereo_ground_sim"
LAUNCH_FILE = PACKAGE_SHARE / "launch/stereo_ground_sim.launch.py"
RUN_OPERATOR_VIEW = SIM_ROOT / "run_operator_view.sh"


class FakeLaunchConfiguration:
    def __init__(self, name: str):
        self.name = name

    def perform(self, context):
        return str(context.values[self.name])


class FakeContext:
    def __init__(self, values: dict[str, object]):
        self.values = values


def source_default(name: str) -> str:
    pattern = rf'DeclareLaunchArgument\("{re.escape(name)}", default_value="([^"]+)"\)'
    match = re.search(pattern, LAUNCH_FILE.read_text(encoding="utf-8"))
    if match is None:
        raise AssertionError(f"Launch default for {name!r} not found")
    return match.group(1)


def base_launch_values(**overrides: object) -> dict[str, object]:
    values: dict[str, object] = {
        "scenario_name": "unspecified",
        "target_class": "unspecified",
        "target_size_m": "nan",
        "scenario_range_min_m": "nan",
        "scenario_range_max_m": "nan",
        "zoom_index": "nan",
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
        "approach_end_distance_m": 200.0,
        "approach_duration_s": 120.0,
        "mission_profile": "manual",
        "observation_mode": "both",
        "takeoff_altitude_m": 10.0,
        "takeoff_duration_s": 10.0,
        "first_hover_duration_s": 5.0,
        "final_hover_duration_s": 5.0,
        "land_duration_s": 10.0,
        "target_center_z_m": 5.0,
        "fly_target_in_gazebo": "false",
        "actual_distance_read_hz": 15.0,
        "actual_distance_source": "gazebo",
        "target_model_name": "x500_15m",
        "target_reference_link_name": "target_reference_link",
        "oracle_base_link_name": "base_link",
        "yolo_model_path": "",
        "yolo_class_name": "drone",
        "yolo_confidence": 0.25,
        "yolo_device": "cuda:0",
        "yolo_target_hz": 40.0,
        "yolo_imgsz": 1280,
        "enable_synthetic_detections": "false",
        "synthetic_detection_width_px": 1280,
        "synthetic_detection_height_px": 720,
        "synthetic_detection_bbox_size_px": 32.0,
        "enable_external_bbox_detections": "false",
        "external_bbox_detections_topic": "/stereo/external_detections",
        "external_bbox_detection_width_px": 1280,
        "external_bbox_detection_height_px": 720,
        "plot_publish_hz": 30.0,
        "enable_bbox_center_adjustment": "false",
        "bbox_center_adjustment_path": "",
        "enable_observation_fusion": "false",
        "observation_fusion_manifest_path": "",
        "observation_fusion_tier": "",
        "enable_disparity_residual_profile": "false",
        "disparity_residual_profile_path": "",
        "enable_disparity_residual_feature_selector": "false",
        "disparity_residual_feature_selector_model_path": "",
        "disparity_residual_feature_selector_catalog_path": "",
        "disparity_residual_feature_selector_window_size": 1,
        "disparity_residual_feature_selector_min_samples": 1,
        "left_roll_error_deg": 0.0,
        "left_pitch_error_deg": 0.0,
        "left_yaw_error_deg": 0.0,
        "right_roll_error_deg": 0.0,
        "right_pitch_error_deg": 0.0,
        "right_yaw_error_deg": 0.0,
    }
    values.update(overrides)
    return values


def _make_launch_import_stubs() -> dict[str, types.ModuleType]:
    class Dummy:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    launch = types.ModuleType("launch")
    launch.LaunchDescription = Dummy
    launch_actions = types.ModuleType("launch.actions")
    launch_actions.DeclareLaunchArgument = Dummy
    launch_actions.ExecuteProcess = Dummy
    launch_actions.IncludeLaunchDescription = Dummy
    launch_actions.OpaqueFunction = Dummy
    launch_conditions = types.ModuleType("launch.conditions")
    launch_conditions.IfCondition = Dummy
    launch_substitutions = types.ModuleType("launch.substitutions")
    launch_substitutions.LaunchConfiguration = FakeLaunchConfiguration
    launch_substitutions.PathJoinSubstitution = Dummy
    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = Dummy
    launch_ros_substitutions = types.ModuleType("launch_ros.substitutions")
    launch_ros_substitutions.FindPackageShare = Dummy
    launch_description_sources = types.ModuleType("launch.launch_description_sources")
    launch_description_sources.PythonLaunchDescriptionSource = Dummy
    ament_index_python = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda _name: str(PACKAGE_SHARE)
    return {
        "launch": launch,
        "launch.actions": launch_actions,
        "launch.conditions": launch_conditions,
        "launch.substitutions": launch_substitutions,
        "launch_ros": launch_ros,
        "launch_ros.actions": launch_ros_actions,
        "launch_ros.substitutions": launch_ros_substitutions,
        "launch.launch_description_sources": launch_description_sources,
        "ament_index_python": ament_index_python,
        "ament_index_python.packages": ament_packages,
    }


def load_launch_module():
    module_name = "_phase2d_b_stereo_ground_launch"
    stubs = _make_launch_import_stubs()
    snapshot = {name: sys.modules.get(name) for name in stubs}
    missing = {name for name in snapshot if name not in sys.modules}
    try:
        sys.modules.update(stubs)
        spec = importlib.util.spec_from_file_location(module_name, LAUNCH_FILE)
        if spec is None or spec.loader is None:
            raise RuntimeError("Unable to load launch module")
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        module.LaunchConfiguration = FakeLaunchConfiguration
        module.get_package_share_directory = lambda _name: str(PACKAGE_SHARE)
        return module
    finally:
        sys.modules.pop(module_name, None)
        for name, original in snapshot.items():
            if name in missing:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


class Phase2DBLaunchContractTests(unittest.TestCase):
    def test_launch_defaults_include_observation_mode_and_camera_contract(self) -> None:
        self.assertEqual(source_default("observation_mode"), "both")
        self.assertEqual(source_default("camera_image_width_px"), "1280")
        self.assertEqual(source_default("camera_image_height_px"), "720")
        self.assertEqual(source_default("camera_clip_near_m"), "0.1")
        self.assertEqual(source_default("camera_clip_far_m"), "2500.0")
        self.assertEqual(source_default("camera_update_rate_hz"), "20.0")
        self.assertEqual(source_default("left_roll_error_deg"), "0.0")
        self.assertEqual(source_default("left_pitch_error_deg"), "0.0")
        self.assertEqual(source_default("left_yaw_error_deg"), "0.0")
        self.assertEqual(source_default("right_roll_error_deg"), "0.0")
        self.assertEqual(source_default("right_pitch_error_deg"), "0.0")
        self.assertEqual(source_default("right_yaw_error_deg"), "0.0")

    def test_distance_node_params_propagate_defaults_and_phase2c_contract(self) -> None:
        module = load_launch_module()
        with tempfile.NamedTemporaryFile(suffix=".yaml") as tmp:
            module._write_distance_node_params(FakeContext(base_launch_values()), params_file=tmp.name)
            data = yaml.safe_load(Path(tmp.name).read_text(encoding="utf-8"))
        params = data["/stereo_distance_node"]["ros__parameters"]

        self.assertEqual(params["observation_mode"], "both")
        self.assertEqual(params["camera_image_width_px"], 1280)
        self.assertEqual(params["camera_image_height_px"], 720)
        self.assertEqual(params["camera_clip_near_m"], 0.1)
        self.assertEqual(params["camera_clip_far_m"], 2500.0)
        self.assertEqual(params["left_roll_error_deg"], 0.0)
        self.assertEqual(params["left_pitch_error_deg"], 0.0)
        self.assertEqual(params["left_yaw_error_deg"], 0.0)
        self.assertEqual(params["right_roll_error_deg"], 0.0)
        self.assertEqual(params["right_pitch_error_deg"], 0.0)
        self.assertEqual(params["right_yaw_error_deg"], 0.0)
        self.assertEqual(params["baseline_origin_world_enu_m"], [0.0, 0.0, 5.0])
        self.assertEqual(
            params["rotation_baseline_world_row_major"],
            [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0],
        )
        self.assertEqual(params["target_reference_link_name"], "target_reference_link")
        self.assertEqual(params["oracle_base_link_name"], "base_link")

    def test_custom_observation_and_camera_params_propagate_with_types(self) -> None:
        module = load_launch_module()
        context = FakeContext(
            base_launch_values(
                observation_mode="ideal_projection",
                camera_image_width_px=1920,
                camera_image_height_px=1080,
                camera_clip_near_m=0.2,
                camera_clip_far_m=1800.5,
                left_roll_error_deg=0.1,
                left_pitch_error_deg=-0.2,
                left_yaw_error_deg=0.3,
                right_roll_error_deg=-0.4,
                right_pitch_error_deg=0.5,
                right_yaw_error_deg=-0.6,
            )
        )
        with tempfile.NamedTemporaryFile(suffix=".yaml") as tmp:
            module._write_distance_node_params(context, params_file=tmp.name)
            data = yaml.safe_load(Path(tmp.name).read_text(encoding="utf-8"))
        params = data["/stereo_distance_node"]["ros__parameters"]

        self.assertEqual(params["observation_mode"], "ideal_projection")
        self.assertEqual(params["camera_image_width_px"], 1920)
        self.assertIs(type(params["camera_image_width_px"]), int)
        self.assertEqual(params["camera_image_height_px"], 1080)
        self.assertIs(type(params["camera_image_height_px"]), int)
        self.assertEqual(params["camera_clip_near_m"], 0.2)
        self.assertIs(type(params["camera_clip_near_m"]), float)
        self.assertEqual(params["camera_clip_far_m"], 1800.5)
        self.assertIs(type(params["camera_clip_far_m"]), float)
        self.assertEqual(params["left_roll_error_deg"], 0.1)
        self.assertIs(type(params["left_roll_error_deg"]), float)
        self.assertEqual(params["left_pitch_error_deg"], -0.2)
        self.assertIs(type(params["left_pitch_error_deg"]), float)
        self.assertEqual(params["left_yaw_error_deg"], 0.3)
        self.assertIs(type(params["left_yaw_error_deg"]), float)
        self.assertEqual(params["right_roll_error_deg"], -0.4)
        self.assertIs(type(params["right_roll_error_deg"]), float)
        self.assertEqual(params["right_pitch_error_deg"], 0.5)
        self.assertIs(type(params["right_pitch_error_deg"]), float)
        self.assertEqual(params["right_yaw_error_deg"], -0.6)
        self.assertIs(type(params["right_yaw_error_deg"]), float)

    def test_launch_attitude_values_feed_world_and_distance_node_yaml(self) -> None:
        text = LAUNCH_FILE.read_text(encoding="utf-8")
        for name in (
            "left_roll_error_deg",
            "left_pitch_error_deg",
            "left_yaw_error_deg",
            "right_roll_error_deg",
            "right_pitch_error_deg",
            "right_yaw_error_deg",
        ):
            self.assertIn(f'_float_config(context, "{name}")', text)
            self.assertIn(f'"{name}"', text)

    def test_custom_bbox_mode_propagates(self) -> None:
        module = load_launch_module()
        with tempfile.NamedTemporaryFile(suffix=".yaml") as tmp:
            module._write_distance_node_params(
                FakeContext(base_launch_values(observation_mode="bbox")),
                params_file=tmp.name,
            )
            data = yaml.safe_load(Path(tmp.name).read_text(encoding="utf-8"))
        self.assertEqual(data["/stereo_distance_node"]["ros__parameters"]["observation_mode"], "bbox")

    def test_operator_script_default_and_override_contract(self) -> None:
        text = RUN_OPERATOR_VIEW.read_text(encoding="utf-8")
        self.assertIn(
            'add_default_launch_arg observation_mode "${OBSERVATION_MODE:-both}" "${launch_input[@]}"',
            text,
        )
        self.assertIn('[[ "$arg" == "$name":=* ]]', text)


if __name__ == "__main__":
    unittest.main()
