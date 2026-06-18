from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "evaluate_disparity_residual_feature_guard.py"
SPEC = importlib.util.spec_from_file_location("evaluate_disparity_residual_feature_guard_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def sample(sample_id: int, truth_forward: float, observed_forward: float) -> object:
    return module.pipe.CalibrationSample(
        sample_id=sample_id,
        t_s=float(sample_id),
        split="train" if sample_id < 2 else "validation",
        truth_ned_m=np.array([truth_forward, 0.0, 0.0], dtype=float),
        truth_rdf_m=np.array([0.0, 0.0, truth_forward], dtype=float),
        raw_rdf_m=np.array([0.0, 0.0, observed_forward], dtype=float),
        left_u_px=640.0,
        left_v_px=360.0,
        measured_disparity_px=1000.0 / observed_forward,
        true_disparity_px=1000.0 / truth_forward,
    )


class EvaluateDisparityResidualFeatureGuardTests(unittest.TestCase):
    def test_feature_stats_uses_runtime_disparity_profile_features(self) -> None:
        series = module.residual.MultiburstProfileSeries(
            seed=1,
            scenario="unit",
            samples=[sample(0, 100.0, 90.0), sample(1, 200.0, 180.0), sample(2, 300.0, 270.0)],
            train_ids={0, 1},
            validation_ids={2},
            raw_validation_points=np.array([[0.0, 0.0, 270.0]], dtype=float),
            observation_points=np.array([[0.0, 0.0, 90.0], [0.0, 0.0, 180.0], [0.0, 0.0, 270.0]], dtype=float),
            fused_disparity_px=np.array([10.0, 5.0, 2.5], dtype=float),
        )
        profile = module.DisparityResidualRuntimeProfile.from_mapping(
            {
                "profile_type": "DisparityResidualRuntimeProfile",
                "feature_space": "disparity",
                "correction_space": "ray_scale",
                "degree": 1,
                "feature_mean": 5.0,
                "feature_scale": 2.5,
                "coefficients_low_to_high": [0.0, 0.1],
                "correction_clip": 0.5,
            }
        )

        stats = module.feature_stats(series, profile)

        self.assertAlmostEqual(stats["feature_z_abs_max"], 2.0)
        self.assertAlmostEqual(stats["correction_abs_max"], 0.2)

    def test_aggregate_prefers_wrong_profile_guard_over_lower_mean_error(self) -> None:
        rows = [
            {
                "eval_scenario": "a",
                "profile_label": "p",
                "z_p95_threshold": 2.0,
                "correction_p95_threshold": 0.1,
                "status": "PASS",
                "guard_applied_residual": True,
                "range_rmse_guarded_m": 3.0,
                "target_max_abs_error_guarded_m": 4.0,
                "target_rmse_guarded_m": 2.0,
            },
            {
                "eval_scenario": "p",
                "profile_label": "p",
                "z_p95_threshold": 2.0,
                "correction_p95_threshold": 0.1,
                "status": "PASS",
                "guard_applied_residual": True,
                "range_rmse_guarded_m": 3.0,
                "target_max_abs_error_guarded_m": 4.0,
                "target_rmse_guarded_m": 2.0,
            },
            {
                "eval_scenario": "a",
                "profile_label": "p",
                "z_p95_threshold": 4.0,
                "correction_p95_threshold": 0.1,
                "status": "PASS",
                "guard_applied_residual": False,
                "range_rmse_guarded_m": 4.0,
                "target_max_abs_error_guarded_m": 4.0,
                "target_rmse_guarded_m": 2.0,
            },
            {
                "eval_scenario": "p",
                "profile_label": "p",
                "z_p95_threshold": 4.0,
                "correction_p95_threshold": 0.1,
                "status": "PASS",
                "guard_applied_residual": True,
                "range_rmse_guarded_m": 4.0,
                "target_max_abs_error_guarded_m": 4.0,
                "target_rmse_guarded_m": 2.0,
            },
        ]

        summaries = module.aggregate(rows)

        self.assertEqual(summaries[0]["z_p95_threshold"], 4.0)
        self.assertEqual(summaries[0]["wrong_profile_apply_count"], 0)
        self.assertEqual(summaries[1]["wrong_profile_apply_count"], 1)

    def test_aggregate_does_not_treat_disable_all_as_viable_guard(self) -> None:
        rows = [
            {
                "eval_scenario": "p",
                "profile_label": "p",
                "z_p95_threshold": 2.0,
                "correction_p95_threshold": 0.01,
                "status": "PASS",
                "guard_applied_residual": False,
                "range_rmse_guarded_m": 3.0,
                "target_max_abs_error_guarded_m": 4.0,
                "target_rmse_guarded_m": 2.0,
            },
            {
                "eval_scenario": "q",
                "profile_label": "p",
                "z_p95_threshold": 2.0,
                "correction_p95_threshold": 0.01,
                "status": "PASS",
                "guard_applied_residual": False,
                "range_rmse_guarded_m": 3.0,
                "target_max_abs_error_guarded_m": 4.0,
                "target_rmse_guarded_m": 2.0,
            },
        ]

        summaries = module.aggregate(rows)

        self.assertFalse(summaries[0]["viable_guard"])
        self.assertEqual(summaries[0]["apply_count"], 0)


if __name__ == "__main__":
    unittest.main()
