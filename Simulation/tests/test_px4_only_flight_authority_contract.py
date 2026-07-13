from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[1]
RUN_OPERATOR_VIEW = SIM_ROOT / "run_operator_view.sh"
RUN_AUTO_MISSION = SIM_ROOT / "run_auto_mission.sh"
RUN_ATTITUDE_SWEEP = SIM_ROOT / "run_attitude_error_experiment.sh"
ATTITUDE_SWEEP_PYTHON = SIM_ROOT / "scripts" / "attitude_error_experiment.py"
LAUNCH_FILE = SIM_ROOT / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"
NODE_FILE = SIM_ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py"


def run_command(
    command: list[str],
    *,
    cwd: Path = SIM_ROOT,
    env_overrides: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["DEFAULT_YOLO_MODEL"] = ""
    env["YOLO_MODEL_PATH"] = ""
    if env_overrides:
        env.update(env_overrides)
    return subprocess.run(
        command,
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20,
        check=False,
    )


class Px4OnlyFlightAuthorityContractTests(unittest.TestCase):
    def test_operator_rejects_no_px4_before_build_or_process_start(self) -> None:
        result = run_command([str(RUN_OPERATOR_VIEW), "--no-px4"])
        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn("--no-px4 is disabled", result.stdout)
        self.assertNotIn("colcon", result.stdout)
        self.assertNotIn("gz sim", result.stdout)

    def test_operator_rejects_unsafe_launch_contract_values_before_build(self) -> None:
        unsafe_args = (
            "flight_authority:=gazebo",
            "fly_target_in_gazebo:=true",
            "mission_profile:=takeoff_forward_land",
            "actual_distance_source:=px4",
        )
        for unsafe_arg in unsafe_args:
            with self.subTest(unsafe_arg=unsafe_arg):
                result = run_command([str(RUN_OPERATOR_VIEW), unsafe_arg])
                self.assertEqual(result.returncode, 2, result.stdout)
                self.assertIn(unsafe_arg.split(":=", 1)[0], result.stdout)
                self.assertNotIn("colcon", result.stdout)
                self.assertNotIn("gz sim", result.stdout)

    def test_operator_rejects_missing_px4_checkout_before_build(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            result = run_command(
                [str(RUN_OPERATOR_VIEW)],
                env_overrides={"PX4_DIR": str(Path(temporary_directory) / "not-px4")},
            )
        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn("PX4_Autopilot checkout was not found or is invalid", result.stdout)
        self.assertNotIn("colcon", result.stdout)
        self.assertNotIn("gz sim", result.stdout)

    def test_safe_contract_is_explicit_in_operator_launch_defaults(self) -> None:
        text = RUN_OPERATOR_VIEW.read_text(encoding="utf-8")
        for expected in (
            'add_default_launch_arg flight_authority "px4_sitl"',
            'add_default_launch_arg fly_target_in_gazebo "false"',
            'add_default_launch_arg mission_profile "manual"',
            'add_default_launch_arg actual_distance_source "gazebo"',
            "Flight authority: PX4 SITL ONLY",
            "Gazebo direct motion: BLOCKED",
            "Ground Truth: Gazebo target_reference_link",
            "PX4 Local NED: AUXILIARY ONLY — NOT GROUND TRUTH",
            "PX4 connection gate: checking heartbeat and LOCAL_POSITION_NED",
        ):
            self.assertIn(expected, text)

    def test_launch_and_node_have_independent_final_guards(self) -> None:
        launch_text = LAUNCH_FILE.read_text(encoding="utf-8")
        node_text = NODE_FILE.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("flight_authority", default_value=PX4_ONLY_FLIGHT_AUTHORITY)', launch_text)
        self.assertIn("_validate_px4_only_contract(context)", launch_text)
        self.assertIn("flight_authority", node_text)
        self.assertIn("validate_px4_only_contract(", node_text)
        self.assertIn(
            "Gazebo-driven target motion is prohibited by the PX4-only flight-authority contract.",
            node_text,
        )

    def test_run_auto_mission_reports_px4_only_status(self) -> None:
        text = RUN_AUTO_MISSION.read_text(encoding="utf-8")
        self.assertIn("Flight authority: PX4 SITL ONLY", text)
        self.assertIn("PX4 Local NED: AUXILIARY ONLY — NOT GROUND TRUTH", text)
        self.assertIn("--check-connection", text)

    def test_legacy_attitude_shell_fails_before_runtime(self) -> None:
        result = run_command([str(RUN_ATTITUDE_SWEEP), "--quick"])
        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn(
            "This legacy sweep is disabled by the PX4-only flight-authority contract.",
            result.stdout,
        )
        self.assertIn(
            "A PX4-native attitude-error sweep runner has not yet been implemented.",
            result.stdout,
        )
        self.assertNotIn("ros2 launch", result.stdout)
        self.assertNotIn("gz sim", result.stdout)

    def test_legacy_attitude_python_fails_without_creating_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output_directory = Path(temporary_directory) / "results"
            result = run_command(
                [
                    sys.executable,
                    str(ATTITUDE_SWEEP_PYTHON),
                    "--quick",
                    "--output-dir",
                    str(output_directory),
                ]
            )
            self.assertEqual(result.returncode, 2, result.stdout)
            self.assertFalse(output_directory.exists())
            self.assertIn("PX4-only flight-authority contract", result.stdout)


if __name__ == "__main__":
    unittest.main()
