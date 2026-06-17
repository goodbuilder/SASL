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
        "summarize_method_families_across_sweep_under_test",
        ROOT_DIR / "scripts" / "summarize_method_families_across_sweep.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_method_families_across_sweep.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SummarizeMethodFamiliesAcrossSweepTests(unittest.TestCase):
    def test_ranks_multiple_method_families_from_existing_comparison_csv(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            run_dir = sweep / "nominal" / "seed_1"
            run_dir.mkdir(parents=True)
            run_dir.joinpath("method_comparison.csv").write_text(
                "\n".join(
                    [
                        "method,registration_model,residual_model,regularization_model,runtime_profile_family,model_complexity,validation_range_rmse_m,validation_target_abs_mean_error_m,validation_target_max_abs_error_m",
                        "raw,none,none,none,none_raw,0,3.0,5.0,6.0",
                        "global_inverse_depth_linear,raw_rdf_inverse_depth_ray_scale,corrected_inverse_depth_linear,inverse_depth_residual,estimated_to_truth_disparity_poly,3,0.7,0.4,1.2",
                        "affine_binned_residual_raw_forward,affine_rdf_estimated_to_truth,forward_raw_rdf_forward_lut,none,estimated_to_truth_affine_rdf,19,0.9,0.5,1.0",
                        "truncated_ls_sim3,sim3_estimated_to_truth,none,truncated_least_squares_consensus,estimated_to_truth_sim3,8,1.1,0.8,2.0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.summarize_method_families_across_sweep(
                sweep,
                root / "out",
                family_specs=["inverse:inverse_depth", "affine:affine", "tls:truncated_ls"],
                target_error_limit_m=10.0,
            )

            self.assertEqual(summary["recommended_family_by_validation"], "affine")
            self.assertEqual(summary["oracle_recommended_family_by_validation"], "affine")
            self.assertTrue(summary["oracle_recommended_family_pass_target_max_limit_all"])
            rows_csv = (root / "out" / "method_family_rows.csv").read_text(encoding="utf-8")
            self.assertIn("global_inverse_depth_linear", rows_csv)
            summary_csv = (root / "out" / "method_family_summary.csv").read_text(encoding="utf-8")
            self.assertIn("estimated_to_truth_affine_rdf", summary_csv)
            self.assertIn("oracle_best_pass_target_max_limit_all", summary_csv)
            summary_json = json.loads((root / "out" / "method_family_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary_json["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertEqual(summary_json["public_topic_contract"], module.PUBLIC_TOPIC_CONTRACT)

    def test_default_families_are_available(self) -> None:
        module = load_module()
        families = module.parse_family_specs([])

        self.assertGreaterEqual(len(families), 5)
        self.assertIn("inverse_depth", {family.name for family in families})


if __name__ == "__main__":
    unittest.main()
