from __future__ import annotations

import os
import ast
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

from pose_simulation.config import (
    DEFAULT_CONFIG_PATH,
    ConfigError,
    format_snapshot,
    resolve_config,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
POSESIM_ROOT = REPO_ROOT / "PoseSimulation"
RUNNER = POSESIM_ROOT / "run_pose_simulation.sh"


class PoseSimulationConfigTests(unittest.TestCase):
    def test_default_config_is_independent_and_read_only(self):
        config = resolve_config(config_path=DEFAULT_CONFIG_PATH, environ={})
        self.assertEqual(config.values["POSESIM_NAMESPACE"], "/pose_simulation")
        self.assertEqual(config.values["POSESIM_SOURCE_APPLICATION"], "Simulation")
        self.assertTrue(config.values["POSESIM_READ_ONLY"])
        self.assertFalse(config.values["POSESIM_START_SOURCE_APPLICATION"])
        self.assertFalse(config.values["POSESIM_OWN_SOURCE_PROCESSES"])
        self.assertFalse(config.values["POSESIM_ENABLE_VERTICAL_RTK_COMPARISON"])
        self.assertEqual(config.values["POSESIM_CAMERA_POSE_MODE"], "static_snapshot")
        self.assertEqual(
            config.values["POSESIM_LEFT_CAMERA_ENTITY_NAME"],
            "left_stereo_camera::optical_link",
        )
        self.assertEqual(config.values["POSESIM_MAX_GT_SOURCE_AGE_MS"], 500.0)
        for key in (
            "POSESIM_POSE_ERROR_TOPIC",
            "POSESIM_STEREO_GEOMETRY_TOPIC",
            "POSESIM_STATUS_TOPIC",
            "POSESIM_GT_IDEAL_POSE_TOPIC",
            "POSESIM_RTK_IDEAL_POSE_TOPIC",
            "POSESIM_DIAGNOSTICS_TOPIC",
            "POSESIM_MARKER_TOPIC",
        ):
            self.assertTrue(str(config.values[key]).startswith("/pose_simulation/"))

    def test_precedence_is_base_then_profile_then_environment_then_cli(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            profile = Path(temp_dir) / "profile.env"
            profile.write_text("POSESIM_EVALUATION_HZ=10\n", encoding="utf-8")
            config = resolve_config(
                config_path=DEFAULT_CONFIG_PATH,
                profile_path=profile,
                environ={"POSESIM_EVALUATION_HZ": "5"},
                cli_overrides={"POSESIM_EVALUATION_HZ": "2"},
            )
        self.assertEqual(config.values["POSESIM_EVALUATION_HZ"], 2.0)
        self.assertEqual(config.provenance.profile_path, str(profile.resolve()))
        self.assertEqual(config.provenance.explicit_override_names, ("POSESIM_EVALUATION_HZ",))

    def test_profile_relative_path_uses_caller_directory_first(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            profile = Path(temp_dir) / "profile.env"
            profile.write_text("POSESIM_EVALUATION_HZ=7\n", encoding="utf-8")
            config = resolve_config(
                config_path=DEFAULT_CONFIG_PATH,
                profile_path="profile.env",
                environ={},
                cwd=temp_dir,
            )
        self.assertEqual(config.values["POSESIM_EVALUATION_HZ"], 7.0)
        self.assertEqual(config.provenance.profile_path, str(profile.resolve()))

    def test_snapshot_is_deterministic_and_threshold_empty_is_unspecified(self):
        first = resolve_config(config_path=DEFAULT_CONFIG_PATH, environ={})
        second = resolve_config(config_path=DEFAULT_CONFIG_PATH, environ={})
        self.assertEqual(format_snapshot(first), format_snapshot(second))
        self.assertIsNone(first.values["POSESIM_WARN_RELATIVE_YAW_ERROR_DEG"])
        self.assertIn('POSESIM_WARN_RELATIVE_YAW_ERROR_DEG=""', format_snapshot(first))

    def test_runner_check_and_print_modes(self):
        env = {key: value for key, value in os.environ.items() if not key.startswith("POSESIM_")}
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        checked = subprocess.run(
            ["bash", str(RUNNER), "--check-config"],
            cwd=REPO_ROOT,
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(checked.returncode, 0, checked.stderr)
        self.assertIn("configuration valid", checked.stdout)
        self.assertIn("no source process was started", checked.stdout)

        printed = subprocess.run(
            ["bash", str(RUNNER), "--print-config", "--set", "POSESIM_EVALUATION_HZ=2"],
            cwd=REPO_ROOT,
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(printed.returncode, 0, printed.stderr)
        self.assertIn("POSESIM_EVALUATION_HZ=2", printed.stdout)
        self.assertIn("provenance.explicit_override_names=POSESIM_EVALUATION_HZ", printed.stdout)

    def test_runner_input_probe_mode_is_separate_from_configuration_check(self):
        env = {key: value for key, value in os.environ.items() if not key.startswith("POSESIM_")}
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        env["ROS_LOG_DIR"] = tempfile.gettempdir()
        probed = subprocess.run(
            ["bash", str(RUNNER), "--check-inputs", "--timeout-s", "0"],
            cwd=REPO_ROOT,
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertIn("PoseSimulation configuration valid.", probed.stdout)
        self.assertTrue(
            "Overall input status:" in probed.stdout
            or "SOURCE_UNAVAILABLE" in probed.stdout
            or "SOURCE_UNAVAILABLE" in probed.stderr
        )
        self.assertIn(probed.returncode, {0, 1, 2})

    def assert_config_error(self, **overrides):
        with self.assertRaises(ConfigError):
            resolve_config(
                config_path=DEFAULT_CONFIG_PATH,
                explicit_overrides={key: str(value) for key, value in overrides.items()},
                environ={},
            )

    def test_namespace_and_output_boundary_guards(self):
        self.assert_config_error(POSESIM_NAMESPACE="/simulation/pose_simulation")
        self.assert_config_error(POSESIM_STATUS_TOPIC="/simulation/pose_simulation/status")
        self.assert_config_error(POSESIM_GT_TOPIC="pose_simulation/input")
        self.assert_config_error(POSESIM_GT_TOPIC="/pose_simulation/input")

    def test_read_only_and_ground_truth_guards(self):
        self.assert_config_error(POSESIM_READ_ONLY="false")
        self.assert_config_error(POSESIM_MODE="controller")
        self.assert_config_error(POSESIM_START_SOURCE_APPLICATION="true")
        self.assert_config_error(POSESIM_OWN_SOURCE_PROCESSES="true")
        self.assert_config_error(POSESIM_ALLOW_GT_FALLBACK="true")
        self.assert_config_error(POSESIM_ALLOW_PX4_AS_GROUND_TRUTH="true")
        self.assert_config_error(POSESIM_ALLOW_PLANNED_POSITION_AS_GROUND_TRUTH="true")
        self.assert_config_error(POSESIM_GT_MESSAGE_TYPE="px4_msgs/msg/VehicleGlobalPosition")
        self.assert_config_error(POSESIM_CAMERA_POSE_MESSAGE_TYPE="geometry_msgs/msg/PoseStamped")
        self.assert_config_error(POSESIM_CAMERA_POSE_SCHEMA="other/v1")

    def test_coordinate_and_vertical_datum_guards(self):
        self.assert_config_error(POSESIM_EXPECTED_WORLD_FRAME="px4_local_ned")
        self.assert_config_error(POSESIM_EXPECTED_CAMERA_FORWARD_AXIS="positive_x")
        self.assert_config_error(POSESIM_ENABLE_VERTICAL_RTK_COMPARISON="true")
        config = resolve_config(
            config_path=DEFAULT_CONFIG_PATH,
            explicit_overrides={
                "POSESIM_ENABLE_VERTICAL_RTK_COMPARISON": "true",
                "POSESIM_ALLOW_UNRESOLVED_VERTICAL_DATUM": "true",
            },
            environ={},
        )
        self.assertTrue(config.values["POSESIM_ENABLE_VERTICAL_RTK_COMPARISON"])

    def test_invalid_scalar_and_enum_values_are_rejected(self):
        self.assert_config_error(POSESIM_READ_ONLY="sometimes")
        self.assert_config_error(POSESIM_EVALUATION_HZ="nan")
        self.assert_config_error(POSESIM_EVALUATION_HZ="inf")
        self.assert_config_error(POSESIM_EVALUATION_HZ="0")
        self.assert_config_error(POSESIM_STALE_AFTER_MS="-1")
        self.assert_config_error(POSESIM_MAX_GT_SOURCE_AGE_MS="-1")
        self.assert_config_error(POSESIM_CAMERA_POSE_MODE="unknown")
        self.assert_config_error(POSESIM_RVIZ_RAY_MODE="unknown")
        self.assert_config_error(POSESIM_GT_MESSAGE_TYPE="")

    def test_unknown_environment_override_is_rejected(self):
        with self.assertRaises(ConfigError):
            resolve_config(
                config_path=DEFAULT_CONFIG_PATH,
                environ={"POSESIM_NOT_A_SCHEMA_KEY": "value"},
            )

    def test_static_source_independence(self):
        source_paths = list(POSESIM_ROOT.glob("*.sh")) + list(
            (POSESIM_ROOT / "pose_simulation").glob("*.py")
        )
        source = "\n".join(path.read_text(encoding="utf-8") for path in source_paths)
        self.assertNotRegex(source, re.compile(r"(?<!POSE)SIM_CONFIG"))
        self.assertNotRegex(source, re.compile(r"(?<!POSE)SIM_SCENARIO_CONFIG"))
        self.assertNotIn("Simulation/config/simulation.env", source)
        for path in source_paths:
            if path.suffix != ".py":
                continue
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    modules = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom):
                    modules = [node.module or ""]
                else:
                    continue
                for module in modules:
                    self.assertFalse(module == "Simulation" or module.startswith("Simulation."))
                    self.assertFalse(module == "stereo_ground_sim" or module.startswith("stereo_ground_sim."))


if __name__ == "__main__":
    unittest.main()
