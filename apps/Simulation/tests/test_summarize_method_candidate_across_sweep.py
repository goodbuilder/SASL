from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "summarize_method_candidate_across_sweep_under_test",
        ROOT_DIR / "scripts" / "summarize_method_candidate_across_sweep.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_method_candidate_across_sweep.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SummarizeMethodCandidateAcrossSweepTests(unittest.TestCase):
    def test_summarizes_candidate_rows_and_ranks(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            run_dir = sweep / "intrinsic_scale_stress" / "seed_1"
            run_dir.mkdir(parents=True)
            run_dir.joinpath("method_comparison.csv").write_text(
                "\n".join(
                    [
                        "method,registration_model,residual_model,runtime_profile_family,model_complexity,validation_range_rmse_m,validation_target_abs_mean_error_m,validation_target_max_abs_error_m",
                        "raw,none,none,none_raw,0,3.0,5.0,6.0",
                        "disparity_product_ray_scale_sim3,product,product,estimated_to_truth_sim3,9,1.2,1.0,2.0",
                        "best_method,sim3,none,estimated_to_truth_sim3,7,0.8,0.7,1.5",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.summarize_candidate_across_sweep(
                sweep,
                root / "out",
                method_substring="disparity_product_ray_scale",
                target_error_limit_m=10.0,
            )

            self.assertEqual(summary["candidate_row_count"], 1)
            self.assertEqual(summary["pass_target_max_limit_count"], 1)
            rows_csv = (root / "out" / "method_candidate_rows.csv").read_text(encoding="utf-8")
            self.assertIn("disparity_product_ray_scale_sim3", rows_csv)
            self.assertIn("rank_by_validation_rmse", rows_csv)
            summary_json = json.loads((root / "out" / "method_candidate_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary_json["best_candidate_method"], "disparity_product_ray_scale_sim3")
            self.assertEqual(summary_json["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_no_matching_candidate_still_writes_json_summary(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            run_dir = sweep / "nominal" / "seed_1"
            run_dir.mkdir(parents=True)
            run_dir.joinpath("method_comparison.csv").write_text(
                "\n".join(
                    [
                        "method,registration_model,residual_model,runtime_profile_family,model_complexity,validation_range_rmse_m,validation_target_abs_mean_error_m,validation_target_max_abs_error_m",
                        "raw,none,none,none_raw,0,3.0,5.0,6.0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.summarize_candidate_across_sweep(
                sweep,
                root / "out",
                method_substring="missing_method",
                target_error_limit_m=10.0,
            )

            self.assertEqual(summary["candidate_row_count"], 0)
            summary_json = json.loads((root / "out" / "method_candidate_summary.json").read_text(encoding="utf-8"))
            self.assertIsNone(summary_json["validation_range_rmse_mean_m"])


if __name__ == "__main__":
    unittest.main()
