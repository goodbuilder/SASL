from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_guard_operating_mode_batch_smoke.py"
SPEC = importlib.util.spec_from_file_location("run_runtime_guard_operating_mode_batch_smoke_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RunRuntimeGuardOperatingModeBatchSmokeTests(unittest.TestCase):
    def test_batch_smoke_exports_each_seed_and_summarizes_pass(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            for scenario, seed in (("bbox", "seed_1"), ("mixed", "seed_2")):
                csv_dir = sweep / scenario / seed
                csv_dir.mkdir(parents=True)
                csv_dir.joinpath("calibration_samples.csv").write_text(
                    "sample_id,split,raw_right_m,raw_down_m,raw_forward_m,raw_range_m,left_u_px,left_v_px,measured_disparity_px,truth_north_m\n"
                    "0,validation,1,2,300,300.01,640,360,12,123\n",
                    encoding="utf-8",
                )
            output_dir = root / "out"
            args = SimpleNamespace(
                sweep_dir=sweep,
                catalog_json=root / "catalog.json",
                output_dir=output_dir,
                minimal_set_json=None,
                scenarios="",
                seeds="",
                split="validation",
                max_samples=50,
                runtime_candidate_source_mode="all_catalog",
                runtime_selection_policy="transfer_risk_fallback",
                transfer_matrix_csv=None,
                transfer_low_risk_rmse_limit_m=1.3,
                transfer_low_risk_target_limit_m=2.5,
                transfer_risk_target_weight=0.1,
                knn_neighbor_count=5,
            )

            def fake_export_by_operating_mode(**kwargs):
                kwargs["output_profile"].parent.mkdir(parents=True, exist_ok=True)
                kwargs["output_profile"].write_text("{}\n", encoding="utf-8")
                return {
                    "selected": True,
                    "selected_operating_mode": "runtime_guard",
                    "has_truth_fields": False,
                    "selected_source_scenario": "bbox",
                    "calibration_required_detected": False,
                    "selection_summary": {
                        "preliminary_source_scenario": "bbox",
                        "selection_policy": "transfer_risk_fallback",
                    },
                }

            def fake_promotion_check(profile_json, output_dir, **kwargs):
                return {
                    "pass_promotion_check": True,
                    "runtime_truth_used": False,
                    "profile_id": "profile",
                    "profile_mode": "estimated_to_truth_sim3",
                    "sample_apply_count": 1,
                    "sample_invalid_count": 0,
                }

            with patch.object(module, "export_by_operating_mode", side_effect=fake_export_by_operating_mode), patch.object(
                module,
                "run_promotion_check",
                side_effect=fake_promotion_check,
            ):
                summary = module.run_batch(args)

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["run_count"], 2)
            self.assertEqual(summary["pass_count"], 2)
            self.assertFalse(summary["runtime_truth_used"])
            rows_csv = output_dir / "runtime_guard_operating_mode_batch_smoke.csv"
            self.assertTrue(rows_csv.is_file())
            summary_json = json.loads((output_dir / "runtime_guard_operating_mode_batch_smoke_summary.json").read_text())
            self.assertEqual(summary_json["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_batch_smoke_reports_no_selected_csvs(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            args = SimpleNamespace(
                sweep_dir=root / "empty",
                catalog_json=root / "catalog.json",
                output_dir=root / "out",
                minimal_set_json=None,
                scenarios="",
                seeds="",
                split="validation",
                max_samples=50,
                runtime_candidate_source_mode="all_catalog",
                runtime_selection_policy="transfer_risk_fallback",
                transfer_matrix_csv=None,
                transfer_low_risk_rmse_limit_m=1.3,
                transfer_low_risk_target_limit_m=2.5,
                transfer_risk_target_weight=0.1,
                knn_neighbor_count=5,
            )

            summary = module.run_batch(args)

            self.assertEqual(summary["status"], "FAIL")
            self.assertIn("no_calibration_csvs_selected", summary["failures"])


if __name__ == "__main__":
    unittest.main()
