from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "evaluate_disparity_residual_catalog_transfer.py"
SPEC = importlib.util.spec_from_file_location("evaluate_disparity_residual_catalog_transfer_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class EvaluateDisparityResidualCatalogTransferTests(unittest.TestCase):
    def test_aggregate_prefers_full_pass_and_lower_target(self) -> None:
        rows = [
            {
                "eval_scenario": "s",
                "profile_label": "a",
                "seed": 1,
                "status": "PASS",
                "range_rmse_observation_m": 2.0,
                "range_rmse_residual_m": 1.0,
                "position_rmse_observation_m": 3.0,
                "position_rmse_residual_m": 2.0,
                "target_max_abs_error_observation_m": 5.0,
                "target_max_abs_error_residual_m": 2.0,
                "target_rmse_observation_m": 4.0,
                "target_rmse_residual_m": 1.5,
                "improves_observation_range": True,
                "improves_observation_target": True,
            },
            {
                "eval_scenario": "s",
                "profile_label": "b",
                "seed": 1,
                "status": "FAIL",
                "range_rmse_observation_m": 2.0,
                "range_rmse_residual_m": 0.8,
                "position_rmse_observation_m": 3.0,
                "position_rmse_residual_m": 2.0,
                "target_max_abs_error_observation_m": 5.0,
                "target_max_abs_error_residual_m": 1.0,
                "target_rmse_observation_m": 4.0,
                "target_rmse_residual_m": 1.0,
                "improves_observation_range": True,
                "improves_observation_target": True,
            },
        ]

        summary = module.aggregate(rows)

        self.assertEqual(summary[0]["profile_label"], "a")
        self.assertEqual(summary[0]["status"], "STRICT_PASS")
        self.assertEqual(summary[1]["status"], "FAIL")

    def test_catalog_profiles_resolves_relative_paths(self) -> None:
        import json

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            (root / "profiles").mkdir()
            profile = root / "profiles" / "p.json"
            profile.write_text("{}", encoding="utf-8")
            catalog = root / "catalog.json"
            catalog.write_text(json.dumps({"profile_paths": {"p": "profiles/p.json"}}), encoding="utf-8")

            profiles = module.catalog_profiles(catalog)

        self.assertEqual(profiles["p"], profile.resolve())


if __name__ == "__main__":
    unittest.main()
