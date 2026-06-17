from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_runner_module():
    spec = importlib.util.spec_from_file_location(
        "run_estimated_to_truth_calibration_exp_under_test",
        ROOT_DIR / "scripts" / "run_estimated_to_truth_calibration_exp.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load run_estimated_to_truth_calibration_exp.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class EstimatedToTruthCalibrationExpRunnerTests(unittest.TestCase):
    def test_build_metrics_maps_pipeline_outputs_to_gate_contract(self) -> None:
        module = load_runner_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            (output_dir / "run_config.json").write_text(
                json.dumps(
                    {
                        "recommendation": "sim3",
                        "sample_count": 10,
                        "train_ratio": 0.8,
                        "target_range_m": 1852.0,
                        "target_window_m": 30.0,
                    }
                ),
                encoding="utf-8",
            )
            write_csv(
                output_dir / "method_comparison.csv",
                [
                    {
                        "method": "raw",
                        "validation_range_rmse_m": 5.0,
                        "validation_position_rmse_m": 8.0,
                        "validation_right_rmse_m": 4.0,
                        "validation_down_rmse_m": 2.0,
                        "validation_forward_rmse_m": 5.0,
                        "validation_target_window_count": 4,
                        "validation_target_abs_mean_error_m": 6.0,
                        "validation_target_rmse_m": 6.5,
                        "validation_sample_count": 2,
                        "train_sample_count": 8,
                        "train_range_rmse_m": 4.5,
                        "generalization_gap_range_rmse_m": 0.5,
                    },
                    {
                        "method": "sim3",
                        "validation_range_rmse_m": 1.0,
                        "validation_position_rmse_m": 1.2,
                        "validation_right_rmse_m": 0.2,
                        "validation_down_rmse_m": 0.1,
                        "validation_forward_rmse_m": 1.0,
                        "validation_target_window_count": 4,
                        "validation_target_abs_mean_error_m": 0.7,
                        "validation_target_rmse_m": 1.1,
                        "validation_sample_count": 2,
                        "train_sample_count": 8,
                        "train_range_rmse_m": 1.1,
                        "generalization_gap_range_rmse_m": -0.1,
                    },
                ],
            )
            (output_dir / "sim3_runtime_profile.json").write_text(
                json.dumps({"mode": "estimated_to_truth_sim3"}),
                encoding="utf-8",
            )
            preflight_dir = output_dir / "preflight"
            preflight_dir.mkdir()
            (preflight_dir / "preflight_summary.json").write_text(
                json.dumps(
                    {
                        "pass_preflight": False,
                        "row_count": 5,
                        "usable_row_count": 5,
                        "target_window_usable_count": 1,
                    }
                ),
                encoding="utf-8",
            )
            promotion_dir = output_dir / "promotion_check"
            promotion_dir.mkdir()
            (promotion_dir / "promotion_check.json").write_text(
                json.dumps({"pass_promotion_check": False}),
                encoding="utf-8",
            )

            metrics = module.build_metrics(
                output_dir=output_dir,
                experiment_id="EXP-UNIT",
                mode="csv",
                input_csv=Path("telemetry.csv"),
                min_target_count=3,
                max_train_validation_gap_m=0.5,
            )

        self.assertEqual(metrics["experiment_id"], "EXP-UNIT")
        self.assertEqual(metrics["method"], "sim3")
        self.assertTrue(metrics["runtime_profile_exportable"])
        self.assertFalse(metrics["runtime_truth_used"])
        self.assertEqual(metrics["validation_range_rmse_raw_m"], 5.0)
        self.assertEqual(metrics["validation_range_rmse_corrected_m"], 1.0)
        self.assertEqual(metrics["validation_right_rmse_corrected_m"], 0.2)
        self.assertEqual(metrics["target_1852m_sample_count"], 4)
        self.assertEqual(metrics["target_1852m_abs_mean_error_m"], 0.7)
        self.assertEqual(metrics["px4_truth_axis_order"], "north,east,down")
        self.assertEqual(metrics["internal_correction_axis_order"], "right,down,forward")
        self.assertIn("preflight_failed", metrics["failure_tags"])
        self.assertIn("promotion_check_failed", metrics["failure_tags"])
        self.assertEqual(metrics["preflight"]["target_window_usable_count"], 1)
        self.assertFalse(metrics["promotion_check"]["pass_promotion_check"])


if __name__ == "__main__":
    unittest.main()
