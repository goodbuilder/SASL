from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "Simulation"
PACKAGE_ROOT = SIM_ROOT / "stereo_ground_sim"
PACKAGE_SHARE = PACKAGE_ROOT
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "stereo_ground_sim.launch.py"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.attitude_error_model import (  # noqa: E402
    CameraAttitudeErrorDeg,
    StereoAttitudeErrorConfig,
)
from stereo_ground_sim.gazebo_camera_attitude_adapter import (  # noqa: E402
    gazebo_stereo_model_rotations_from_attitude_errors,
)


class FakeLaunchConfiguration:
    def __init__(self, name: str):
        self.name = name

    def perform(self, context):
        return str(context.values[self.name])


class FakeContext:
    def __init__(self, values: dict[str, object]):
        self.values = values


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
    module_name = "_phase3c_b2_stereo_ground_launch"
    stubs = _make_launch_import_stubs()
    snapshot = {name: sys.modules.get(name) for name in stubs}
    missing = {name for name in snapshot if name not in sys.modules}
    try:
        sys.modules.update(stubs)
        spec = importlib.util.spec_from_file_location(module_name, LAUNCH_FILE)
        if spec is None or spec.loader is None:
            raise RuntimeError("Unable to load production launch module")
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


def launch_values(**overrides: object) -> dict[str, object]:
    values: dict[str, object] = {
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
        "left_roll_error_deg": 0.0,
        "left_pitch_error_deg": 0.0,
        "left_yaw_error_deg": 0.0,
        "right_roll_error_deg": 0.0,
        "right_pitch_error_deg": 0.0,
        "right_yaw_error_deg": 0.0,
    }
    values.update(overrides)
    return values


def _generated_pose(world_path: str | Path, model_name: str) -> tuple[float, ...]:
    root = ET.fromstring(Path(world_path).read_text(encoding="utf-8"))
    model = root.find(f"world/model[@name='{model_name}']")
    if model is None:
        raise AssertionError(f"model {model_name!r} not found")
    pose = model.findtext("pose")
    if pose is None:
        raise AssertionError(f"model {model_name!r} has no pose")
    values = tuple(float(value) for value in pose.split())
    if len(values) != 6:
        raise AssertionError(f"expected six pose values, got {values!r}")
    return values


def _rotation_from_sdf_rpy(pose: tuple[float, ...]) -> np.ndarray:
    roll, pitch, yaw = pose[3:]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.asarray(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.asarray(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


class Phase3CB2LaunchAdapterWiringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_launch_module()

    def generate(self, **overrides: object) -> tuple[Path, dict[str, tuple[float, ...]]]:
        with tempfile.NamedTemporaryFile(suffix=".sdf", delete=False) as tmp:
            world_path = Path(tmp.name)
        self.module._generate_world(FakeContext(launch_values(**overrides)), str(world_path))
        return world_path, {
            "left": _generated_pose(world_path, "left_stereo_camera"),
            "right": _generated_pose(world_path, "right_stereo_camera"),
        }

    def assert_translation_contract(self, poses: dict[str, tuple[float, ...]], baseline: float = 25.0, height: float = 5.0) -> None:
        self.assertLess(np.max(np.abs(np.asarray(poses["left"][:3]) - np.asarray((-baseline / 2.0, 0.0, height)))), 1.0e-12)
        self.assertLess(np.max(np.abs(np.asarray(poses["right"][:3]) - np.asarray((baseline / 2.0, 0.0, height)))), 1.0e-12)
        left = np.asarray(poses["left"][:3])
        right = np.asarray(poses["right"][:3])
        self.assertAlmostEqual(float(np.linalg.norm(right - left)), baseline, delta=1.0e-12)
        self.assertLess(np.max(np.abs((left + right) / 2.0 - np.asarray((0.0, 0.0, height)))), 1.0e-12)

    def test_zero_attitude_generates_nominal_sdf_rotations_and_translation(self) -> None:
        world_path, poses = self.generate()
        self.addCleanup(world_path.unlink, missing_ok=True)
        expected = gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(
                left=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
                right=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
            )
        )
        for side, rotation in (("left", expected.left), ("right", expected.right)):
            self.assertLess(
                float(np.max(np.abs(_rotation_from_sdf_rpy(poses[side]) - np.asarray(rotation.rotation_world_model)))),
                1.0e-10,
            )
        self.assertAlmostEqual(poses["left"][3], 0.0, delta=1.0e-12)
        self.assertAlmostEqual(poses["left"][4], 0.0, delta=1.0e-12)
        self.assertAlmostEqual(poses["left"][5], math.pi / 2.0, delta=1.0e-10)
        self.assertAlmostEqual(poses["right"][3], 0.0, delta=1.0e-12)
        self.assertAlmostEqual(poses["right"][4], 0.0, delta=1.0e-12)
        self.assertAlmostEqual(poses["right"][5], math.pi / 2.0, delta=1.0e-10)
        self.assert_translation_contract(poses)

    def test_composite_attitude_serializes_each_b1_rotation(self) -> None:
        values = {
            "left_roll_error_deg": 0.10,
            "left_pitch_error_deg": -0.05,
            "left_yaw_error_deg": 0.08,
            "right_roll_error_deg": -0.07,
            "right_pitch_error_deg": 0.04,
            "right_yaw_error_deg": -0.06,
        }
        world_path, poses = self.generate(**values)
        self.addCleanup(world_path.unlink, missing_ok=True)
        expected = gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(
                left=CameraAttitudeErrorDeg(0.10, -0.05, 0.08),
                right=CameraAttitudeErrorDeg(-0.07, 0.04, -0.06),
            )
        )
        for side, rotation in (("left", expected.left), ("right", expected.right)):
            self.assertLess(
                float(np.max(np.abs(_rotation_from_sdf_rpy(poses[side]) - np.asarray(rotation.rotation_world_model)))),
                1.0e-10,
            )
        self.assertAlmostEqual(poses["left"][3], math.radians(0.10), delta=1.0e-10)
        self.assertAlmostEqual(poses["left"][4], math.radians(0.05), delta=1.0e-10)
        self.assertAlmostEqual(poses["left"][5], math.radians(89.92), delta=1.0e-10)
        self.assert_translation_contract(poses)

    def test_left_and_right_attitude_inputs_are_independent(self) -> None:
        left_path, left_poses = self.generate(left_roll_error_deg=0.10, left_pitch_error_deg=-0.05, left_yaw_error_deg=0.08)
        right_path, right_poses = self.generate(right_roll_error_deg=-0.07, right_pitch_error_deg=0.04, right_yaw_error_deg=-0.06)
        self.addCleanup(left_path.unlink, missing_ok=True)
        self.addCleanup(right_path.unlink, missing_ok=True)
        zero = gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(
                left=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
                right=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
            )
        )
        left_only = gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(
                left=CameraAttitudeErrorDeg(0.10, -0.05, 0.08),
                right=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
            )
        )
        right_only = gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(
                left=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
                right=CameraAttitudeErrorDeg(-0.07, 0.04, -0.06),
            )
        )
        self.assertLess(np.max(np.abs(_rotation_from_sdf_rpy(left_poses["left"]) - np.asarray(left_only.left.rotation_world_model))), 1.0e-10)
        self.assertLess(np.max(np.abs(_rotation_from_sdf_rpy(left_poses["right"]) - np.asarray(zero.right.rotation_world_model))), 1.0e-10)
        self.assertLess(np.max(np.abs(_rotation_from_sdf_rpy(right_poses["left"]) - np.asarray(zero.left.rotation_world_model))), 1.0e-10)
        self.assertLess(np.max(np.abs(_rotation_from_sdf_rpy(right_poses["right"]) - np.asarray(right_only.right.rotation_world_model))), 1.0e-10)

    def test_translation_is_invariant_across_attitude_cases(self) -> None:
        cases = (
            {},
            {"left_roll_error_deg": 0.10, "right_yaw_error_deg": -0.06},
            {"left_pitch_error_deg": -0.05, "right_roll_error_deg": -0.07, "baseline_m": 31.0, "camera_height_m": 7.5},
        )
        for case in cases:
            world_path, poses = self.generate(**case)
            self.addCleanup(world_path.unlink, missing_ok=True)
            self.assert_translation_contract(
                poses,
                baseline=float(case.get("baseline_m", 25.0)),
                height=float(case.get("camera_height_m", 5.0)),
            )

    def test_node_yaml_preserves_original_camera_rdf_degrees(self) -> None:
        values = defaultdict(
            lambda: "0",
            launch_values(
                left_roll_error_deg=0.10,
                left_pitch_error_deg=-0.05,
                left_yaw_error_deg=0.08,
                right_roll_error_deg=-0.07,
                right_pitch_error_deg=0.04,
                right_yaw_error_deg=-0.06,
            ),
        )
        with tempfile.NamedTemporaryFile(suffix=".yaml") as tmp:
            self.module._write_distance_node_params(FakeContext(values), params_file=tmp.name)
            params = yaml.safe_load(Path(tmp.name).read_text(encoding="utf-8"))["/stereo_distance_node"]["ros__parameters"]
        self.assertEqual(params["left_pitch_error_deg"], -0.05)
        self.assertEqual(params["left_yaw_error_deg"], 0.08)
        self.assertEqual(params["right_pitch_error_deg"], 0.04)
        self.assertEqual(params["right_yaw_error_deg"], -0.06)

    def test_gimbal_lock_fails_before_serializing_malformed_pose(self) -> None:
        with tempfile.NamedTemporaryFile(suffix=".sdf") as tmp:
            with self.assertRaisesRegex(RuntimeError, r"left.*gimbal_lock"):
                self.module._generate_world(
                    FakeContext(launch_values(left_pitch_error_deg=90.0)),
                    tmp.name,
                )

    def test_source_contract_uses_public_adapter_without_direct_rpy_composition(self) -> None:
        source = LAUNCH_FILE.read_text(encoding="utf-8")
        for token in (
            "gazebo_stereo_model_rotations_from_attitude_errors",
            "CameraAttitudeErrorDeg",
            "StereoAttitudeErrorConfig",
        ):
            self.assertIn(token, source)
        self.assertNotIn("camera_yaw + left_yaw", source)
        self.assertNotIn("camera_yaw + right_yaw", source)


if __name__ == "__main__":
    unittest.main()
