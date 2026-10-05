from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "summarize_multiburst_disparity_residual_results.py"


def load_module():
    spec = importlib.util.spec_from_file_location("summarize_multiburst_disparity_residual_results_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SummarizeMultiburstDisparityResidualResultsTests(unittest.TestCase):
    def test_aggregate_prefers_full_scenario_coverage(self) -> None:
        module = load_module()
        rows = [
            {
                "method": "single_scenario_low_target",
                "scenario": "a",
                "status": "STRICT_PASS",
                "total_observation_count": "21",
                "target_max_abs_error_observation_max_m": "5.0",
                "target_max_abs_error_residual_max_m": "1.0",
                "range_rmse_observation_mean_m": "2.0",
                "range_rmse_residual_mean_m": "0.5",
                "position_rmse_observation_mean_m": "3.0",
                "position_rmse_residual_mean_m": "2.5",
                "runtime_truth_used": "False",
            },
            {
                "method": "full_coverage",
                "scenario": "a",
                "status": "STRICT_PASS",
                "total_observation_count": "21",
                "target_max_abs_error_observation_max_m": "5.0",
                "target_max_abs_error_residual_max_m": "2.0",
                "range_rmse_observation_mean_m": "2.0",
                "range_rmse_residual_mean_m": "0.6",
                "position_rmse_observation_mean_m": "3.0",
                "position_rmse_residual_mean_m": "2.6",
                "runtime_truth_used": "False",
            },
            {
                "method": "full_coverage",
                "scenario": "b",
                "status": "STRICT_PASS",
                "total_observation_count": "21",
                "target_max_abs_error_observation_max_m": "4.0",
                "target_max_abs_error_residual_max_m": "3.0",
                "range_rmse_observation_mean_m": "1.5",
                "range_rmse_residual_mean_m": "0.4",
                "position_rmse_observation_mean_m": "2.8",
                "position_rmse_residual_mean_m": "2.4",
                "runtime_truth_used": "False",
            },
        ]

        aggregate = module.aggregate_method_rows(rows)

        self.assertEqual(aggregate[0]["method"], "full_coverage")
        self.assertEqual(aggregate[0]["scenario_count"], 2)
        self.assertEqual(aggregate[0]["target_max_abs_error_residual_worst_m"], 3.0)
        self.assertFalse(aggregate[0]["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
