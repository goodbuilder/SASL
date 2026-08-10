from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_family_module():
    spec = importlib.util.spec_from_file_location(
        "audit_family_calibration_profile_exports_under_test",
        ROOT_DIR / "scripts" / "audit_family_calibration_profile_exports.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_family_calibration_profile_exports.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditFamilyCalibrationProfileExportsTests(unittest.TestCase):
    def test_audit_exports_each_source_seed_and_replays_family_targets(self) -> None:
        module = load_family_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            for seed, raw_forward, truth_forward in (
                ("seed_a", 100.0, 105.0),
                ("seed_b", 120.0, 126.0),
            ):
                seed_dir = sweep / "scale_family" / seed
                seed_dir.mkdir(parents=True)
                seed_dir.joinpath("calibration_samples.csv").write_text(
                    "split,raw_right_m,raw_down_m,raw_forward_m,truth_right_m,truth_down_m,truth_forward_m,measured_disparity_px\n"
                    f"validation,0,0,{raw_forward},0,0,{truth_forward},10\n",
                    encoding="utf-8",
                )
            args = type(
                "Args",
                (),
                {
                    "sweep_dir": sweep,
                    "output_dir": root / "out",
                    "target_scenarios": "scale_family",
                    "source_seeds": "seed_a",
                    "export_policies": "final_recommendation",
                    "split": "validation",
                    "target_range_m": 105.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "train_ratio": 0.8,
                    "seed": 1,
                    "pipeline_script": root / "pipeline.py",
                },
            )()

            def fake_run_pipeline(**kwargs):
                fit_dir = kwargs["fit_dir"]
                profile = fit_dir / "sim3_runtime_profile.json"
                profile.parent.mkdir(parents=True)
                profile.write_text(
                    json.dumps(
                        {
                            "mode": "estimated_to_truth_disparity_poly",
                            "profile_id": "scale",
                            "method": "global_inverse_depth_linear",
                            "correction_space": "raw_rdf_inverse_depth_ray_scale",
                            "inverse_depth_coefficients_high_to_low": [1.0 / 1.05, 0.0],
                            "runtime_inputs": ["raw_rdf_m", "measured_disparity_px"],
                            "forbidden_runtime_inputs": ["calibration_truth"],
                        }
                    ),
                    encoding="utf-8",
                )
                (fit_dir / "run_config.json").write_text(
                    json.dumps({"recommendation": "global_inverse_depth_linear"}),
                    encoding="utf-8",
                )
                profile_dir = fit_dir / "runtime_profiles"
                profile_dir.mkdir()
                (profile_dir / "manifest.json").write_text(
                    json.dumps(
                        [
                            {
                                "method": "global_inverse_depth_linear",
                                "profile_path": "sim3_runtime_profile.json",
                                "selected": True,
                            }
                        ]
                    ),
                    encoding="utf-8",
                )
                (fit_dir / "policy_recommendations.csv").write_text(
                    "policy,purpose,method\n"
                    "balanced,default balance,global_inverse_depth_linear\n",
                    encoding="utf-8",
                )
                return type("Completed", (), {"stdout": "ok", "stderr": ""})()

            with patch.object(module, "run_pipeline", side_effect=fake_run_pipeline):
                summary = module.audit_family_exports(args)

            self.assertEqual(summary["export_count"], 1)
            self.assertEqual(summary["scenario_summary_count"], 1)
            self.assertEqual(summary["policy_summary_count"], 1)
            self.assertEqual(summary["scenario_policy_summary_count"], 1)
            self.assertEqual(summary["scenario_policy_recommendation_count"], 1)
            self.assertEqual(summary["recommended_source_policy"], "final_recommendation")
            self.assertEqual(summary["family_replay_count"], 2)
            self.assertEqual(summary["heldout_replay_count"], 1)
            self.assertTrue(summary["heldout_replay_pass_all"])
            self.assertTrue((args.output_dir / "family_calibration_profile_exports.csv").is_file())
            self.assertTrue((args.output_dir / "family_calibration_profile_transfer.csv").is_file())
            self.assertTrue((args.output_dir / "family_calibration_profile_scenario_summary.csv").is_file())
            self.assertTrue((args.output_dir / "family_calibration_profile_scenario_summary.json").is_file())
            self.assertTrue((args.output_dir / "family_calibration_profile_policy_summary.csv").is_file())
            self.assertTrue((args.output_dir / "family_calibration_profile_policy_summary.json").is_file())
            self.assertTrue((args.output_dir / "family_calibration_profile_scenario_policy_summary.csv").is_file())
            self.assertTrue((args.output_dir / "family_calibration_profile_scenario_policy_summary.json").is_file())
            self.assertTrue(
                (args.output_dir / "family_calibration_profile_scenario_policy_recommendations.csv").is_file()
            )
            self.assertTrue(
                (args.output_dir / "family_calibration_profile_scenario_policy_recommendations.json").is_file()
            )
            self.assertTrue((args.output_dir / "family_calibration_profile_export_summary.json").is_file())
            self.assertTrue((args.output_dir / "family_calibration_profile_export_summary.png").is_file())
            scenario_summary = json.loads(
                (args.output_dir / "family_calibration_profile_scenario_summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(scenario_summary[0]["scenario"], "scale_family")
            self.assertTrue(scenario_summary[0]["heldout_replay_pass_all"])
            policy_summary = json.loads(
                (args.output_dir / "family_calibration_profile_policy_summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(policy_summary[0]["source_policy"], "final_recommendation")
            scenario_policy_recommendations = json.loads(
                (args.output_dir / "family_calibration_profile_scenario_policy_recommendations.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(scenario_policy_recommendations[0]["recommended_source_policy"], "final_recommendation")


if __name__ == "__main__":
    unittest.main()
