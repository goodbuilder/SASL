from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_loop_module():
    spec = importlib.util.spec_from_file_location(
        "run_calibration_profile_loop_under_test",
        ROOT_DIR / "scripts" / "run_calibration_profile_loop.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load run_calibration_profile_loop.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunCalibrationProfileLoopTests(unittest.TestCase):
    def test_pass_fail_requires_median_p90_axis_nonregression_and_target_count(self) -> None:
        module = load_loop_module()
        passing = {
            "validation_range_rmse_raw_m": "5.0",
            "validation_range_rmse_corrected_m": "2.0",
            "validation_position_rmse_raw_m": "6.0",
            "validation_position_rmse_corrected_m": "3.0",
            "validation_position_median_raw_m": "4.0",
            "validation_position_median_corrected_m": "2.0",
            "validation_position_p90_raw_m": "9.0",
            "validation_position_p90_corrected_m": "5.0",
            "validation_right_rmse_raw_m": "2.0",
            "validation_right_rmse_corrected_m": "1.0",
            "validation_down_rmse_raw_m": "2.0",
            "validation_down_rmse_corrected_m": "2.0",
            "validation_forward_rmse_raw_m": "5.0",
            "validation_forward_rmse_corrected_m": "2.0",
            "target_window_count": "3",
            "runtime_truth_used": "",
        }

        ok, failures = module.pass_fail(passing, min_target_count=3)

        self.assertTrue(ok)
        self.assertEqual(failures, [])

        failing = dict(passing)
        failing["validation_position_p90_corrected_m"] = "10.0"
        failing["validation_down_rmse_corrected_m"] = "3.0"
        failing["target_window_count"] = "2"
        ok, failures = module.pass_fail(failing, min_target_count=3)

        self.assertFalse(ok)
        self.assertIn("position_p90_not_improved", failures)
        self.assertIn("down_axis_regressed", failures)
        self.assertIn("target_window_count_below_minimum", failures)

    def test_build_loop_summary_records_forbidden_runtime_truth_inputs(self) -> None:
        module = load_loop_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            args = type(
                "Args",
                (),
                {
                    "calibration_csv": tmp / "cal.csv",
                    "validation_csv": tmp / "val.csv",
                    "min_target_count": 1,
                },
            )()
            summary = module.build_loop_summary(
                args=args,
                profile_json=tmp / "profile.json",
                validation_csv=tmp / "val_pipeline.csv",
                validation_conversion_summary={"converted_row_count": 3},
                replay_summary={
                    "validation_range_rmse_raw_m": "5.0",
                    "validation_range_rmse_corrected_m": "2.0",
                    "validation_position_rmse_raw_m": "6.0",
                    "validation_position_rmse_corrected_m": "3.0",
                    "validation_position_median_raw_m": "4.0",
                    "validation_position_median_corrected_m": "2.0",
                    "validation_position_p90_raw_m": "9.0",
                    "validation_position_p90_corrected_m": "5.0",
                    "validation_right_rmse_raw_m": "2.0",
                    "validation_right_rmse_corrected_m": "1.0",
                    "validation_down_rmse_raw_m": "2.0",
                    "validation_down_rmse_corrected_m": "2.0",
                    "validation_forward_rmse_raw_m": "5.0",
                    "validation_forward_rmse_corrected_m": "2.0",
                    "target_window_count": "2",
                },
            )

        self.assertEqual(summary["status"], "PASS")
        self.assertFalse(summary["runtime_truth_used"])
        self.assertIn("gazebo_truth_pose", summary["forbidden_runtime_inputs"])
        self.assertIn("truth_right_m", summary["forbidden_runtime_inputs"])
        self.assertEqual(summary["validation_conversion_summary"]["converted_row_count"], 3)

    def test_main_orchestrates_profile_build_validation_replay_and_summary(self) -> None:
        module = load_loop_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            cal_csv = tmp / "cal.csv"
            val_csv = tmp / "val.csv"
            cal_csv.write_text("truth_right_m,truth_down_m,truth_forward_m\n", encoding="utf-8")
            val_csv.write_text("truth_right_m,truth_down_m,truth_forward_m\n", encoding="utf-8")
            output_dir = tmp / "out"
            commands: list[list[str]] = []

            def fake_run_command(cmd, *, cwd):
                commands.append([str(part) for part in cmd])
                if "run_estimated_to_truth_calibration_exp.py" in str(cmd[1]):
                    profile_dir = output_dir / "calibration_profile"
                    profile_dir.mkdir(parents=True, exist_ok=True)
                    (profile_dir / "sim3_runtime_profile.json").write_text(
                        json.dumps({"mode": "estimated_to_truth_sim3"}),
                        encoding="utf-8",
                    )
                if "stereodistance_profile_replay.py" in str(cmd[1]):
                    replay_dir = output_dir / "validation_replay"
                    replay_dir.mkdir(parents=True, exist_ok=True)
                    (replay_dir / "profile_replay_summary.csv").write_text(
                        "validation_range_rmse_raw_m,validation_range_rmse_corrected_m,"
                        "validation_position_rmse_raw_m,validation_position_rmse_corrected_m,"
                        "validation_position_median_raw_m,validation_position_median_corrected_m,"
                        "validation_position_p90_raw_m,validation_position_p90_corrected_m,"
                        "validation_right_rmse_raw_m,validation_right_rmse_corrected_m,"
                        "validation_down_rmse_raw_m,validation_down_rmse_corrected_m,"
                        "validation_forward_rmse_raw_m,validation_forward_rmse_corrected_m,"
                        "target_window_count\n"
                        "5,2,6,3,4,2,9,5,2,1,2,2,5,2,3\n",
                        encoding="utf-8",
                    )

            old_run_command = module.run_command
            old_argv = sys.argv
            try:
                module.run_command = fake_run_command
                sys.argv = [
                    "run_calibration_profile_loop.py",
                    "--calibration-csv",
                    str(cal_csv),
                    "--validation-csv",
                    str(val_csv),
                    "--output-dir",
                    str(output_dir),
                ]
                self.assertEqual(module.main(), 0)
            finally:
                module.run_command = old_run_command
                sys.argv = old_argv

            summary = json.loads((output_dir / "two_flight_validation_summary.json").read_text(encoding="utf-8"))

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(len(commands), 2)
        self.assertIn(str(cal_csv), commands[0])
        self.assertIn(str(val_csv), commands[1])


if __name__ == "__main__":
    unittest.main()
