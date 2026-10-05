from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_audit_module():
    spec = importlib.util.spec_from_file_location(
        "audit_stereodistance_profile_blend_strategies_under_test",
        ROOT_DIR / "scripts" / "audit_stereodistance_profile_blend_strategies.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_stereodistance_profile_blend_strategies.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditStereoDistanceProfileBlendStrategiesTests(unittest.TestCase):
    def test_audit_aggregates_policy_replay_results_across_seed_runs(self) -> None:
        module = load_audit_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep_dir = root / "sweep"
            catalog_dir = sweep_dir / "scenario_profile_catalog"
            catalog_dir.mkdir(parents=True)
            (catalog_dir / "manifest.json").write_text(
                json.dumps({"mode": "scenario_profile_catalog", "profiles": []}),
                encoding="utf-8",
            )
            (sweep_dir / "scenario_profile_catalog_minimal_set.json").write_text(
                json.dumps({"pass_minimal_catalog_profile_set": True, "rows": []}),
                encoding="utf-8",
            )
            (sweep_dir / "scenario_runs.csv").write_text(
                "scenario,pass_run\ncombined_sensor_stress,True\n",
                encoding="utf-8",
            )
            scenario_dir = sweep_dir / "combined_sensor_stress"
            seed_dir = scenario_dir / "seed_1"
            seed_dir.mkdir(parents=True)
            (seed_dir / "calibration_samples.csv").write_text(
                "split,rdf_right,rdf_down,rdf_forward\nvalidation,0,0,100\n",
                encoding="utf-8",
            )
            (scenario_dir / "seed_runs.csv").write_text(
                f"seed,run_dir,exit_code,pass_run\n1,{seed_dir},0,True\n",
                encoding="utf-8",
            )
            output_dir = root / "audit"

            def fake_explore_blends(**kwargs):
                run_output = kwargs["output_dir"]
                run_output.mkdir(parents=True, exist_ok=True)
                module.write_csv(
                    run_output / "profile_blend_policy_recommendations.csv",
                    [
                        {
                            "policy": "balanced",
                            "source_a": "nominal",
                            "source_b": "intrinsic_scale_stress",
                            "alpha_second_profile": "0.2",
                            "pass_replay": True,
                            "validation_range_rmse_corrected_m": "1.2",
                            "target_abs_mean_error_corrected_m": "0.3",
                            "target_max_abs_error_corrected_m": "0.6",
                            "stability_min_range_improvement_m": "0.9",
                            "replay_selection_score": "1.8",
                            "is_pareto_frontier": True,
                        },
                        {
                            "policy": "validation_best",
                            "source_a": "nominal",
                            "source_b": "intrinsic_scale_stress",
                            "alpha_second_profile": "0.0",
                            "pass_replay": True,
                            "validation_range_rmse_corrected_m": "1.0",
                            "target_abs_mean_error_corrected_m": "0.8",
                            "target_max_abs_error_corrected_m": "1.1",
                            "stability_min_range_improvement_m": "1.0",
                            "replay_selection_score": "2.0",
                            "is_pareto_frontier": True,
                        },
                    ],
                )
                return {"output_profile": str(kwargs["output_profile"])}

            with patch.object(module, "explore_blends", side_effect=fake_explore_blends):
                run_rows, policy_summary, summary = module.audit_blend_strategies(
                    sweep_dir=sweep_dir,
                    output_dir=output_dir,
                )

            self.assertEqual(len(run_rows), 2)
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertEqual(summary["public_topic_contract"], module.PUBLIC_TOPIC_CONTRACT)
            self.assertIn("calibration-time truth", summary["selection_contract"])
            balanced = next(row for row in policy_summary if row["policy"] == "balanced")
            self.assertEqual(balanced["run_count"], 1)
            self.assertEqual(balanced["pass_count"], 1)
            self.assertAlmostEqual(balanced["validation_range_rmse_corrected_mean_m"], 1.2)
            self.assertAlmostEqual(balanced["target_abs_mean_error_max_m"], 0.3)
            self.assertAlmostEqual(balanced["target_max_abs_error_max_m"], 0.6)
            self.assertEqual(balanced["alpha_values"], "0.2")
            self.assertTrue((output_dir / "blend_strategy_policy_runs.csv").is_file())
            self.assertTrue((output_dir / "blend_strategy_policy_summary.csv").is_file())
            self.assertTrue((output_dir / "blend_strategy_summary.json").is_file())
            self.assertTrue((output_dir / "blend_strategy_summary.txt").is_file())


if __name__ == "__main__":
    unittest.main()
