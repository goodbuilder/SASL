from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_multiburst_motion_fusion_sweep.py"


def load_module():
    spec = importlib.util.spec_from_file_location("run_runtime_multiburst_motion_fusion_sweep_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunRuntimeMultiburstMotionFusionSweepTests(unittest.TestCase):
    def test_aggregate_preserves_observation_and_motion_metrics(self) -> None:
        module = load_module()
        rows = [
            {
                "method": "combo",
                "status": "PASS",
                "burst_count": 7,
                "observations_per_burst": 3,
                "total_observation_count": 21,
                "inner_mode": "trimmed_mean",
                "outer_mode": "trimmed_mean",
                "trim_fraction": 0.1,
                "common_fraction": 0.5,
                "alpha": 1.0,
                "beta": 0.001,
                "innovation_clip_m": 40.0,
                "fusion_axes": "forward",
                "range_rmse_raw_m": 4.0,
                "range_rmse_observation_m": 2.0,
                "range_rmse_motion_m": 2.1,
                "position_rmse_observation_m": 3.0,
                "position_rmse_motion_m": 3.2,
                "target_max_abs_error_observation_m": 5.0,
                "target_max_abs_error_motion_m": 5.5,
                "target_rmse_observation_m": 3.5,
                "target_rmse_motion_m": 3.7,
            },
            {
                "method": "combo",
                "status": "PASS",
                "burst_count": 7,
                "observations_per_burst": 3,
                "total_observation_count": 21,
                "inner_mode": "trimmed_mean",
                "outer_mode": "trimmed_mean",
                "trim_fraction": 0.1,
                "common_fraction": 0.5,
                "alpha": 1.0,
                "beta": 0.001,
                "innovation_clip_m": 40.0,
                "fusion_axes": "forward",
                "range_rmse_raw_m": 6.0,
                "range_rmse_observation_m": 2.4,
                "range_rmse_motion_m": 2.5,
                "position_rmse_observation_m": 3.4,
                "position_rmse_motion_m": 3.6,
                "target_max_abs_error_observation_m": 6.0,
                "target_max_abs_error_motion_m": 6.5,
                "target_rmse_observation_m": 4.0,
                "target_rmse_motion_m": 4.2,
            },
        ]

        summary = module.aggregate(rows)

        self.assertEqual(len(summary), 1)
        self.assertEqual(summary[0]["status"], "STRICT_PASS")
        self.assertEqual(summary[0]["pass_count"], 2)
        self.assertEqual(summary[0]["range_rmse_observation_mean_m"], 2.2)
        self.assertEqual(summary[0]["range_rmse_motion_mean_m"], 2.3)
        self.assertEqual(summary[0]["target_max_abs_error_observation_max_m"], 6.0)
        self.assertEqual(summary[0]["target_max_abs_error_motion_max_m"], 6.5)
        self.assertFalse(summary[0]["runtime_truth_used"])
        self.assertIn("RDF", summary[0]["rdf_axis_contract"])


if __name__ == "__main__":
    unittest.main()
