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
        "audit_method_family_profile_selection_under_test",
        ROOT_DIR / "scripts" / "audit_method_family_profile_selection.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_method_family_profile_selection.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditMethodFamilyProfileSelectionTests(unittest.TestCase):
    def test_selects_best_passing_runtime_profile_inside_family(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            run_dir = sweep / "nominal" / "seed_1"
            profiles = run_dir / "runtime_profiles"
            profiles.mkdir(parents=True)
            profiles.joinpath("affine_good.json").write_text(json.dumps({"method": "affine_good"}), encoding="utf-8")
            profiles.joinpath("affine_bad.json").write_text(json.dumps({"method": "affine_bad"}), encoding="utf-8")
            run_dir.joinpath("method_comparison.csv").write_text(
                "\n".join(
                    [
                        "method,registration_model,residual_model,regularization_model,runtime_profile_family,model_complexity,validation_range_rmse_m,validation_target_window_count,validation_target_abs_mean_error_m,validation_target_max_abs_error_m",
                        "raw,none,none,none,none_raw,0,3.0,2,5.0,6.0",
                        "affine_bad,affine_rdf_estimated_to_truth,none,none,estimated_to_truth_affine_rdf,6,0.7,2,0.3,12.0",
                        "affine_good,affine_rdf_estimated_to_truth,none,none,estimated_to_truth_affine_rdf,6,0.9,2,0.4,2.0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.audit_method_family_profile_selection(
                sweep,
                root / "out",
                family_name="affine_runtime",
                method_substring="affine",
                target_error_limit_m=10.0,
                export_selected_profiles=True,
            )

            self.assertTrue(summary["selected_pass_all"])
            self.assertEqual(summary["selected_pass_count"], 1)
            selected_csv = (root / "out" / "method_family_profile_selected.csv").read_text(encoding="utf-8")
            self.assertIn("affine_good", selected_csv)
            self.assertIn("target_max_error_over_limit", (root / "out" / "method_family_profile_candidates.csv").read_text(encoding="utf-8"))
            exported = root / "out" / "selected_profiles" / "nominal" / "seed_1" / "affine_good.json"
            self.assertTrue(exported.is_file())
            summary_json = json.loads((root / "out" / "method_family_profile_selection_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary_json["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertEqual(summary_json["public_topic_contract"], module.PUBLIC_TOPIC_CONTRACT)

    def test_missing_runtime_profile_prevents_selection(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            run_dir = sweep / "nominal" / "seed_1"
            run_dir.mkdir(parents=True)
            run_dir.joinpath("method_comparison.csv").write_text(
                "\n".join(
                    [
                        "method,runtime_profile_family,model_complexity,validation_range_rmse_m,validation_target_window_count,validation_target_abs_mean_error_m,validation_target_max_abs_error_m",
                        "raw,none_raw,0,3.0,2,5.0,6.0",
                        "affine_missing,estimated_to_truth_affine_rdf,6,0.9,2,0.4,2.0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            summary = module.audit_method_family_profile_selection(
                sweep,
                root / "out",
                family_name="affine_runtime",
                method_substring="affine",
                target_error_limit_m=10.0,
            )

            self.assertFalse(summary["selected_pass_all"])
            self.assertEqual(summary["selected_pass_count"], 0)
            selected_csv = (root / "out" / "method_family_profile_selected.csv").read_text(encoding="utf-8")
            self.assertIn("missing_runtime_profile", selected_csv)


if __name__ == "__main__":
    unittest.main()
