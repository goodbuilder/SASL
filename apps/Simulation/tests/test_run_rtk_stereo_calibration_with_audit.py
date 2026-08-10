from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_rtk_stereo_calibration_with_audit.py"
SPEC = importlib.util.spec_from_file_location("run_rtk_stereo_calibration_with_audit_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RunRtkStereoCalibrationWithAuditTests(unittest.TestCase):
    def test_pipeline_command_preserves_passthrough_args_and_forces_output_dir(self) -> None:
        args, passthrough = module.parse_args(
            [
                "--output-dir",
                "out",
                "--mode",
                "synthetic",
                "--sample-count",
                "12",
                "--target-range-m",
                "1852",
            ]
        )

        command = module.pipeline_command(args, passthrough)

        self.assertEqual(command[2:4], ["--output-dir", "out"])
        self.assertIn("--mode", command)
        self.assertIn("synthetic", command)
        self.assertIn("--sample-count", command)
        self.assertIn("12", command)

    def test_dry_run_returns_command_without_running_pipeline(self) -> None:
        with mock.patch.object(module, "run_pipeline") as run_pipeline:
            result = module.run(["--output-dir", "out", "--mode", "synthetic", "--dry-run"])

        self.assertEqual(result["status"], "DRY_RUN")
        run_pipeline.assert_not_called()
        self.assertIn("rtk_stereo_calibration_pipeline.py", result["pipeline_command"][1])

    def test_nonzero_pipeline_writes_failure_summary(self) -> None:
        completed = module.subprocess.CompletedProcess(
            args=["python"],
            returncode=2,
            stdout="out",
            stderr="err",
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir) / "result"
            with mock.patch.object(module, "run_pipeline", return_value=completed):
                result = module.run(["--output-dir", str(output_dir), "--mode", "synthetic"])

            summary = output_dir / "output_audit" / "rtk_stereo_calibration_with_audit_summary.txt"
            summary_exists = summary.is_file()

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["audit_status"], "SKIPPED")
        self.assertTrue(summary_exists)

    def test_successful_pipeline_runs_audit_and_writes_summary(self) -> None:
        completed = module.subprocess.CompletedProcess(
            args=["python"],
            returncode=0,
            stdout="ok",
            stderr="",
        )
        audit_result = {
            "status": "PASS",
            "result_dir": "result",
            "recommendation": "sim3",
            "method_count": 2,
            "target_1852m_method_count": 2,
            "validation_prediction_row_count": 4,
            "policy_count": 1,
            "train_count": 8,
            "validation_count": 2,
            "selected_profile_path": "runtime_profiles/sim3.json",
            "selected_profile_mode": "estimated_to_truth_sim3",
            "selected_profile_load_status": "PASS",
            "selected_profile_apply_status": "PASS",
            "top_level_profile_equivalence_status": "PASS",
            "policy_profile_count": 1,
            "policy_profile_load_pass_count": 1,
            "plot_dimensions": {"range_error_plot": "800x450"},
            "inventory_status": "PASS",
            "inventory_file_count": 10,
            "inventory_total_size_bytes": 1234,
            "failure_count": 0,
            "failures": [],
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": True,
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir) / "result"
            with mock.patch.object(module, "run_pipeline", return_value=completed):
                with mock.patch.object(module.output_audit, "audit_outputs", return_value=audit_result):
                    with mock.patch.object(
                        module.output_inventory,
                        "export_inventory",
                        return_value={"status": "PASS", "file_count": 10, "total_size_bytes": 1234},
                    ):
                        result = module.run(["--output-dir", str(output_dir), "--mode", "synthetic"])

            summary = output_dir / "output_audit" / "rtk_stereo_calibration_with_audit_summary.json"
            summary_exists = summary.is_file()

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["audit_status"], "PASS")
        self.assertEqual(result["validation_prediction_row_count"], 4)
        self.assertEqual(result["train_count"], 8)
        self.assertEqual(result["selected_profile_mode"], "estimated_to_truth_sim3")
        self.assertEqual(result["selected_profile_load_status"], "PASS")
        self.assertEqual(result["top_level_profile_equivalence_status"], "PASS")
        self.assertEqual(result["policy_profile_load_pass_count"], 1)
        self.assertEqual(result["plot_dimensions"], {"range_error_plot": "800x450"})
        self.assertEqual(result["inventory_status"], "PASS")
        self.assertEqual(result["inventory_file_count"], 10)
        self.assertTrue(summary_exists)


if __name__ == "__main__":
    unittest.main()
