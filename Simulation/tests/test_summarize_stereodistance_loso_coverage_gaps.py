from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_gap_module():
    spec = importlib.util.spec_from_file_location(
        "summarize_stereodistance_loso_coverage_gaps_under_test",
        ROOT_DIR / "scripts" / "summarize_stereodistance_loso_coverage_gaps.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_stereodistance_loso_coverage_gaps.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SummarizeStereoDistanceLosoCoverageGapsTests(unittest.TestCase):
    def test_summarizes_indispensable_target_family_gaps(self) -> None:
        module = load_gap_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            full_dir = sweep / "profile_selection_strategy_audit_all_catalog"
            loso_dir = sweep / "profile_selection_strategy_audit_all_catalog_loso"
            runtime_dir = sweep / "runtime_measurement_guard_policy_audit_loso" / "runs" / "all_catalog_transfer_risk_fallback"
            full_dir.mkdir(parents=True)
            loso_dir.mkdir(parents=True)
            runtime_dir.mkdir(parents=True)
            full_dir.joinpath("profile_selection_strategy_summary.json").write_text(
                json.dumps(
                    {
                        "scenario_summary": [
                            {
                                "target_scenario": "intrinsic_scale_stress",
                                "run_count": 3,
                                "calibration_pass_count": 3,
                                "calibration_validation_range_rmse_corrected_mean_m": 0.8,
                            },
                            {
                                "target_scenario": "bbox_outlier_stress",
                                "run_count": 3,
                                "calibration_pass_count": 3,
                                "calibration_validation_range_rmse_corrected_mean_m": 1.2,
                            },
                        ]
                    }
                ),
                encoding="utf-8",
            )
            loso_dir.joinpath("profile_selection_strategy_summary.json").write_text(
                json.dumps(
                    {
                        "scenario_summary": [
                            {
                                "target_scenario": "intrinsic_scale_stress",
                                "run_count": 3,
                                "calibration_pass_count": 0,
                            },
                            {
                                "target_scenario": "bbox_outlier_stress",
                                "run_count": 3,
                                "calibration_pass_count": 2,
                                "calibration_validation_range_rmse_corrected_mean_m": 1.5,
                            },
                        ]
                    }
                ),
                encoding="utf-8",
            )
            loso_dir.joinpath("profile_selection_strategy_runs.csv").write_text(
                "target_scenario,seed,calibration_pass,top_candidate_source_scenario,top_candidate_pass_reason\n"
                "intrinsic_scale_stress,1,False,nominal,no_positive_range_rmse_improvement\n"
                "intrinsic_scale_stress,2,False,nominal,no_positive_range_rmse_improvement\n"
                "intrinsic_scale_stress,3,False,nominal,no_positive_range_rmse_improvement\n"
                "bbox_outlier_stress,1,True,combined_sensor_stress,pass\n"
                "bbox_outlier_stress,2,True,combined_sensor_stress,pass\n"
                "bbox_outlier_stress,3,False,nominal,target_max_error_over_limit\n",
                encoding="utf-8",
            )
            runtime_dir.joinpath("runtime_measurement_guard_audit.csv").write_text(
                "target_scenario,pass_runtime_measurement_guard_replay,selected_source_scenario\n"
                "intrinsic_scale_stress,False,nominal\n"
                "intrinsic_scale_stress,False,nominal\n"
                "intrinsic_scale_stress,False,nominal\n",
                encoding="utf-8",
            )

            summary = module.summarize_coverage_gaps(sweep_dir=sweep, output_dir=root / "out")

            self.assertEqual(summary["full_gap_scenarios"], ["intrinsic_scale_stress"])
            self.assertEqual(summary["partial_gap_scenarios"], ["bbox_outlier_stress"])
            with (root / "out" / "loso_coverage_gap_scenarios.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            intrinsic = next(row for row in rows if row["target_scenario"] == "intrinsic_scale_stress")
            self.assertEqual(intrinsic["coverage_gap_type"], "target_family_required")
            self.assertIn("no_positive_range_rmse_improvement", intrinsic["loso_failed_top_candidate_reason_counts_json"])
            self.assertTrue((root / "out" / "loso_coverage_gap_summary.json").is_file())
            self.assertTrue((root / "out" / "loso_coverage_gap_pass_counts.png").is_file())

            direct_runtime = sweep / "runtime_measurement_guard_audit_policy_catalog_robust_family_loso"
            direct_runtime.mkdir()
            direct_runtime.joinpath("runtime_measurement_guard_audit.csv").write_text(
                "target_scenario,pass_runtime_measurement_guard_replay,selected_source_scenario\n"
                "bbox_outlier_stress,False,yaw_pitch_stress\n",
                encoding="utf-8",
            )
            direct_summary = module.summarize_coverage_gaps(
                sweep_dir=sweep,
                output_dir=root / "out_direct_runtime",
                runtime_loso_dir=direct_runtime,
            )
            self.assertTrue(direct_summary["runtime_loso_available"])
            with (root / "out_direct_runtime" / "loso_coverage_gap_scenarios.csv").open(
                newline="", encoding="utf-8"
            ) as handle:
                direct_rows = list(csv.DictReader(handle))
            bbox = next(row for row in direct_rows if row["target_scenario"] == "bbox_outlier_stress")
            self.assertEqual(bbox["runtime_loso_pass_count"], "0")
            self.assertIn("yaw_pitch_stress", bbox["runtime_loso_selected_source_counts_json"])


if __name__ == "__main__":
    unittest.main()
