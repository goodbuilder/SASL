from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_oracle_module():
    spec = importlib.util.spec_from_file_location(
        "audit_runtime_measurement_guard_oracle_regret_under_test",
        ROOT_DIR / "scripts" / "audit_runtime_measurement_guard_oracle_regret.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load audit_runtime_measurement_guard_oracle_regret.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AuditRuntimeMeasurementGuardOracleRegretTests(unittest.TestCase):
    def test_audit_oracle_regret_compares_selected_to_best_candidate(self) -> None:
        module = load_oracle_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "calibration_samples.csv"
            input_csv.write_text("split\nvalidation\n", encoding="utf-8")
            guard_csv = root / "guard.csv"
            guard_csv.write_text(
                "target_scenario,target_seed,input_csv,selection_policy,preliminary_source_scenario,selected_source_scenario\n"
                f"nominal,seed_1,{input_csv},transfer_risk_fallback,nominal,selected\n",
                encoding="utf-8",
            )
            catalog_json = root / "catalog.json"
            catalog_json.write_text(json.dumps({"mode": "scenario_profile_catalog", "profiles": []}), encoding="utf-8")

            def fake_load_catalog(path):
                return {"mode": "scenario_profile_catalog", "profiles": []}

            def fake_candidate_source_scenarios(catalog, minimal_set_json):
                return ["selected", "oracle"]

            def fake_replay_candidate(**kwargs):
                scenario = kwargs["scenario"]
                corrected = 2.0 if scenario == "selected" else 1.0
                target_abs = 1.5 if scenario == "selected" else 0.5
                target_max = 3.0 if scenario == "selected" else 2.0
                result = {
                    "scenario": scenario,
                    "method": "sim3",
                    "pass_replay": True,
                    "pass_reason": "pass",
                    "validation_range_rmse_corrected_m": corrected,
                    "target_abs_mean_error_corrected_m": target_abs,
                    "target_max_abs_error_corrected_m": target_max,
                    "catalog_target_abs_mean_error_max_m": target_max,
                    "model_complexity_estimate": 8,
                    "stability_penalty_m": 0.0,
                }
                result["replay_selection_score"] = module.replay_selection_score(result)
                return result

            args = type(
                "Args",
                (),
                {
                    "catalog_json": catalog_json,
                    "guard_audit_csv": guard_csv,
                    "output_dir": root / "out",
                    "minimal_set_json": None,
                    "candidate_source_mode": "all_catalog",
                    "split": "validation",
                    "target_range_m": 1852.0,
                    "target_window_m": 30.0,
                    "target_error_limit_m": 10.0,
                    "allow_unvalidated": False,
                },
            )()

            with patch.object(module, "load_catalog", side_effect=fake_load_catalog), patch.object(
                module, "candidate_source_scenarios", side_effect=fake_candidate_source_scenarios
            ), patch.object(module, "replay_candidate", side_effect=fake_replay_candidate):
                summary = module.audit_oracle_regret(args)

            self.assertEqual(summary["sample_count"], 1)
            self.assertEqual(summary["selected_oracle_match_count"], 0)
            self.assertAlmostEqual(summary["rmse_regret_mean_m"], 1.0)
            self.assertTrue((args.output_dir / "runtime_measurement_guard_oracle_regret.csv").is_file())
            self.assertTrue((args.output_dir / "runtime_measurement_guard_oracle_candidates.csv").is_file())
            self.assertTrue((args.output_dir / "runtime_measurement_guard_oracle_regret.png").is_file())


if __name__ == "__main__":
    unittest.main()
