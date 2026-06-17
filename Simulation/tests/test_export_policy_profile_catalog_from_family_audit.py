from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_export_module():
    spec = importlib.util.spec_from_file_location(
        "export_policy_profile_catalog_from_family_audit_under_test",
        ROOT_DIR / "scripts" / "export_policy_profile_catalog_from_family_audit.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load export_policy_profile_catalog_from_family_audit.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportPolicyProfileCatalogFromFamilyAuditTests(unittest.TestCase):
    def test_builds_policy_catalog_with_best_source_seed_profile(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            base_catalog = root / "base" / "manifest.json"
            base_catalog.parent.mkdir()
            base_catalog.write_text(
                json.dumps(
                    {
                        "mode": "scenario_profile_catalog",
                        "profiles": [
                            {
                                "scenario": "bbox",
                                "purpose": "bbox stress",
                                "catalog_profile_path": "unused.json",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            audit = root / "audit"
            audit.mkdir()
            profile_a = audit / "exports" / "bbox" / "seed_a" / "target_max_stable" / "active_runtime_profile.json"
            profile_b = audit / "exports" / "bbox" / "seed_b" / "target_max_stable" / "active_runtime_profile.json"
            for profile, method in ((profile_a, "global_inverse_depth_linear"), (profile_b, "sim3")):
                profile.parent.mkdir(parents=True)
                profile.write_text(
                    json.dumps(
                        {
                            "mode": "estimated_to_truth_disparity_poly",
                            "method": method,
                            "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m"],
                            "forbidden_runtime_inputs": ["calibration_truth"],
                        }
                    ),
                    encoding="utf-8",
                )
            audit.joinpath("family_calibration_profile_scenario_policy_recommendations.csv").write_text(
                "scenario,source_policy,recommended_source_policy,heldout_replay_pass_all,heldout_replay_pass_count,heldout_replay_count,validation_range_rmse_corrected_max_m,validation_range_rmse_improvement_min_m,target_abs_mean_error_corrected_max_m,target_max_abs_error_max_m\n"
                "bbox,target_max_stable,target_max_stable,True,2,2,1.2,0.8,1.1,3.0\n",
                encoding="utf-8",
            )
            audit.joinpath("family_calibration_profile_transfer.csv").write_text(
                "scenario,source_policy,source_seed,target_seed,pass_family_profile_transfer,validation_range_rmse_corrected_m,validation_range_rmse_improvement_m,target_max_abs_error_corrected_m\n"
                "bbox,target_max_stable,seed_a,seed_a,True,1.0,1.0,5.0\n"
                "bbox,target_max_stable,seed_a,seed_b,True,1.1,0.9,4.0\n"
                "bbox,target_max_stable,seed_b,seed_a,True,1.2,0.8,2.5\n"
                "bbox,target_max_stable,seed_b,seed_b,True,1.0,1.0,3.0\n",
                encoding="utf-8",
            )
            audit.joinpath("family_calibration_profile_exports.csv").write_text(
                "scenario,source_seed,source_policy,active_profile,exported_method\n"
                f"bbox,seed_a,target_max_stable,{profile_a},global_inverse_depth_linear\n"
                f"bbox,seed_b,target_max_stable,{profile_b},sim3\n",
                encoding="utf-8",
            )
            output_dir = root / "policy_catalog"
            summary = module.build_policy_catalog(
                base_catalog_json=base_catalog,
                family_audit_dir=audit,
                output_dir=output_dir,
                output_catalog_json=output_dir / "manifest.json",
            )

            self.assertTrue(summary["pass_policy_profile_catalog"])
            manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["mode"], "scenario_profile_catalog")
            self.assertEqual(manifest["profiles"][0]["source_seed"], "seed_b")
            self.assertEqual(manifest["profiles"][0]["method"], "sim3")
            self.assertTrue(manifest["profiles"][0]["selected_source_pass_all_transfer"])
            self.assertEqual(manifest["profiles"][0]["selected_source_target_max_abs_error_max_m"], 3.0)
            self.assertEqual(
                manifest["profiles"][0]["runtime_input_contract"],
                "scenario-policy profile consumes raw stereo-center RDF/disparity only; no runtime truth",
            )
            summary_payload = json.loads((output_dir / "policy_profile_catalog_summary.json").read_text(encoding="utf-8"))
            self.assertTrue(summary_payload["selected_source_pass_all"])
            self.assertEqual(summary_payload["selected_source_target_max_abs_error_max_m"], 3.0)
            self.assertTrue((output_dir / "bbox_runtime_profile.json").is_file())
            self.assertTrue((output_dir / "policy_profile_catalog.csv").is_file())
            self.assertTrue((output_dir / "policy_profile_catalog_summary.json").is_file())


if __name__ == "__main__":
    unittest.main()
