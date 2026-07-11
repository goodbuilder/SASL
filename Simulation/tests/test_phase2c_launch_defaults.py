from __future__ import annotations

import importlib.util
import json
import math
import re
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "Simulation"
PACKAGE_SHARE = SIM_ROOT / "stereo_ground_sim"
LAUNCH_FILE = PACKAGE_SHARE / "launch/stereo_ground_sim.launch.py"
WORLD_FILE = PACKAGE_SHARE / "worlds/stereo_ground_drone.sdf"
CAMERA_MODEL_ENV = SIM_ROOT / "config/camera_model.env"
RUN_OPERATOR_VIEW = SIM_ROOT / "run_operator_view.sh"
ATTITUDE_EXPERIMENT = SIM_ROOT / "scripts/attitude_error_experiment.py"
SCENARIOS_DIR = SIM_ROOT / "config/scenarios"
R_B_W_ROW_MAJOR = [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0]


class FakeLaunchConfiguration:
    def __init__(self, name: str):
        self.name = name

    def perform(self, context):
        return str(context.values[self.name])


class FakeContext:
    def __init__(self, values: dict[str, object]):
        self.values = values


def env_values(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, value = stripped.split("=", 1)
        values[key] = value
    return values


def source_default(name: str) -> str:
    pattern = rf'DeclareLaunchArgument\("{re.escape(name)}", default_value="([^"]+)"\)'
    match = re.search(pattern, LAUNCH_FILE.read_text(encoding="utf-8"))
    if not match:
        raise AssertionError(f"Launch default for {name!r} not found")
    return match.group(1)


def parse_floats(text: str) -> tuple[float, ...]:
    return tuple(float(part) for part in text.split())


def assert_tuple_close(testcase: unittest.TestCase, actual, expected, delta: float) -> None:
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(float(actual_value), float(expected_value), delta=delta)


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
    module_name = "_phase2c_stereo_ground_launch"
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


class Phase2CLaunchDefaultTests(unittest.TestCase):
    def test_static_defaults(self) -> None:
        self.assertEqual(source_default("camera_update_rate_hz"), "20.0")
        self.assertEqual(source_default("camera_height_m"), "5.0")
        self.assertEqual(source_default("camera_image_width_px"), "1280")
        self.assertEqual(source_default("camera_image_height_px"), "720")
        self.assertEqual(source_default("target_reference_link_name"), "target_reference_link")
        self.assertEqual(source_default("oracle_base_link_name"), "base_link")

        camera_env = env_values(CAMERA_MODEL_ENV)
        self.assertEqual(float(camera_env["CAMERA_HEIGHT_M"]), 5.0)
        self.assertEqual(float(camera_env["CAMERA_UPDATE_RATE_HZ"]), 20.0)

        self.assertIn(
            'add_default_launch_arg camera_update_rate_hz "${CAMERA_UPDATE_RATE_HZ:-20.0}"',
            RUN_OPERATOR_VIEW.read_text(encoding="utf-8"),
        )
        self.assertIn(
            'parser.add_argument("--camera-update-rate-hz", type=float, default=20.0)',
            ATTITUDE_EXPERIMENT.read_text(encoding="utf-8"),
        )

    def test_current_scenario_camera_rates_are_20_and_heights_preserved(self) -> None:
        expected_heights = {
            "small_drone_close_2m_4x.env": 1.5,
            "small_drone_mid_5m_8x.env": 3.0,
            "uam_far_25m_8x.env": 10.0,
        }
        for scenario_file in sorted(SCENARIOS_DIR.glob("*.env")):
            values = env_values(scenario_file)
            if "CAMERA_UPDATE_RATE_HZ" not in values:
                continue
            self.assertEqual(float(values["CAMERA_UPDATE_RATE_HZ"]), 20.0, scenario_file.name)
            self.assertEqual(float(values["CAMERA_HEIGHT_M"]), expected_heights[scenario_file.name])

    def test_generate_world_custom_rig_and_rate(self) -> None:
        module = load_launch_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            generated_world = Path(tmpdir) / "generated.sdf"
            context = FakeContext(
                base_launch_values(
                    baseline_m=40.0,
                    camera_height_m=10.0,
                    camera_update_rate_hz=17.5,
                    left_roll_error_deg=1.0,
                    left_pitch_error_deg=2.0,
                    left_yaw_error_deg=3.0,
                    right_roll_error_deg=-1.0,
                    right_pitch_error_deg=-2.0,
                    right_yaw_error_deg=-3.0,
                )
            )

            module._generate_world(context, generated_world_path=str(generated_world))

            root = ET.parse(generated_world).getroot()
            models = {model.attrib.get("name"): model for model in root.findall("world/model")}
            left_pose = parse_floats(models["left_stereo_camera"].findtext("pose"))
            right_pose = parse_floats(models["right_stereo_camera"].findtext("pose"))
            assert_tuple_close(self, left_pose[:3], (-20.0, 0.0, 10.0), 1.0e-9)
            assert_tuple_close(self, right_pose[:3], (20.0, 0.0, 10.0), 1.0e-9)

            rates = [
                float(sensor.findtext("update_rate"))
                for sensor in root.findall(".//model/link/sensor[@type='camera']")
            ]
            self.assertEqual(rates, [17.5, 17.5])

    def test_source_camera_rate_token_count_is_guarded(self) -> None:
        module = load_launch_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            package_share = Path(tmpdir) / "share"
            worlds_dir = package_share / "worlds"
            worlds_dir.mkdir(parents=True)
            broken_world = WORLD_FILE.read_text(encoding="utf-8").replace(
                "<update_rate>20</update_rate>",
                "<update_rate>21</update_rate>",
                1,
            )
            (worlds_dir / "stereo_ground_drone.sdf").write_text(broken_world, encoding="utf-8")
            module.get_package_share_directory = lambda _name: str(package_share)
            with self.assertRaisesRegex(RuntimeError, "Expected exactly two source camera update_rate tokens"):
                module._generate_world(FakeContext(base_launch_values()), generated_world_path=str(Path(tmpdir) / "out.sdf"))

    def test_write_distance_node_params_contains_numeric_oracle_arrays(self) -> None:
        module = load_launch_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            params_file = Path(tmpdir) / "params.yaml"
            context = FakeContext(
                base_launch_values(
                    baseline_m=40.0,
                    camera_height_m=10.0,
                    left_roll_error_deg=7.0,
                    right_yaw_error_deg=-7.0,
                )
            )

            module._write_distance_node_params(context, params_file=str(params_file))

            loaded = yaml.safe_load(params_file.read_text(encoding="utf-8"))
            params = loaded["/stereo_distance_node"]["ros__parameters"]
            self.assertEqual(params["target_model_name"], "x500_15m")
            self.assertEqual(params["target_reference_link_name"], "target_reference_link")
            self.assertEqual(params["oracle_base_link_name"], "base_link")

            origin = params["baseline_origin_world_enu_m"]
            rotation = params["rotation_baseline_world_row_major"]
            self.assertIsInstance(origin, list)
            self.assertIsInstance(rotation, list)
            self.assertEqual(len(origin), 3)
            self.assertEqual(len(rotation), 9)
            self.assertFalse(any(isinstance(item, bool) for item in origin + rotation))
            self.assertTrue(all(isinstance(item, (int, float)) for item in origin + rotation))
            assert_tuple_close(self, origin, (0.0, 0.0, 10.0), 1.0e-9)
            assert_tuple_close(self, rotation, R_B_W_ROW_MAJOR, 1.0e-12)

    def test_numeric_yaml_sequence_rejects_bool_and_nonfinite(self) -> None:
        module = load_launch_module()
        with self.assertRaisesRegex(ValueError, "finite numeric"):
            module._param_yaml_value([0.0, True, 1.0])
        with self.assertRaisesRegex(ValueError, "finite numeric"):
            module._param_yaml_value([0.0, math.inf, 1.0])
        with self.assertRaisesRegex(ValueError, "finite numeric"):
            module._param_yaml_value([0.0, "1.0", 1.0])


if __name__ == "__main__":
    unittest.main()
