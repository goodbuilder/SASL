from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[1]
RUN_OPERATOR_VIEW = SIM_ROOT / "run_operator_view.sh"
RUN_ATTITUDE_SWEEP = SIM_ROOT / "run_attitude_error_experiment.sh"


def run_entrypoint(arguments: list[str]) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["DEFAULT_YOLO_MODEL"] = ""
    env["YOLO_MODEL_PATH"] = ""
    return subprocess.run(
        arguments,
        cwd=SIM_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20,
        check=False,
    )


class Px4OnlyOperatorEntrypointTests(unittest.TestCase):
    def test_no_px4_does_not_create_runtime_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            result = run_entrypoint(
                [
                    str(RUN_OPERATOR_VIEW),
                    "--no-px4",
                ]
            )
            self.assertEqual(result.returncode, 2, result.stdout)
            self.assertFalse(build_base.exists())
            self.assertNotIn("Operator view started.", result.stdout)

    def test_unsafe_motion_and_mission_values_are_not_fallbacks(self) -> None:
        for argument in (
            "fly_target_in_gazebo:=true",
            "mission_profile:=scripted",
            "flight_authority:=gazebo",
            "actual_distance_source:=manual",
        ):
            with self.subTest(argument=argument):
                result = run_entrypoint([str(RUN_OPERATOR_VIEW), argument])
                self.assertEqual(result.returncode, 2, result.stdout)
                self.assertNotIn("Operator view started.", result.stdout)
                self.assertNotIn("PX4 auto mission will start", result.stdout)

    def test_attitude_sweep_shell_has_no_gazebo_fallback(self) -> None:
        result = run_entrypoint([str(RUN_ATTITUDE_SWEEP), "--quick"])
        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn("legacy sweep is disabled", result.stdout)
        self.assertIn("PX4-native attitude-error sweep runner", result.stdout)


if __name__ == "__main__":
    unittest.main()
