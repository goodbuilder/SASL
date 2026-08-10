from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_pooled_disparity_residual_profile.py"
SPEC = importlib.util.spec_from_file_location("run_runtime_pooled_disparity_residual_profile_under_test", SCRIPT_PATH)
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


class RunRuntimePooledDisparityResidualProfileTests(unittest.TestCase):
    def test_fit_pooled_model_uses_train_samples_from_all_series(self) -> None:
        series_a = module.residual.MultiburstProfileSeries(
            seed=1,
            scenario="a",
            samples=[sample(0, 100.0, 90.0), sample(1, 200.0, 180.0), sample(2, 300.0, 270.0)],
            train_ids={0, 1},
            validation_ids={2},
            raw_validation_points=np.array([[0.0, 0.0, 270.0]], dtype=float),
            observation_points=np.array([[0.0, 0.0, 90.0], [0.0, 0.0, 180.0], [0.0, 0.0, 270.0]], dtype=float),
            fused_disparity_px=np.array([10.0, 5.0, 3.333333], dtype=float),
        )
        series_b = module.residual.MultiburstProfileSeries(
            seed=2,
            scenario="b",
            samples=[sample(0, 120.0, 108.0), sample(1, 240.0, 216.0), sample(2, 360.0, 324.0)],
            train_ids={0, 1},
            validation_ids={2},
            raw_validation_points=np.array([[0.0, 0.0, 324.0]], dtype=float),
            observation_points=np.array([[0.0, 0.0, 108.0], [0.0, 0.0, 216.0], [0.0, 0.0, 324.0]], dtype=float),
            fused_disparity_px=np.array([9.0, 4.5, 3.0], dtype=float),
        )

        model = module.fit_pooled_model(
            [series_a, series_b],
            feature_space="disparity",
            correction_space="ray_scale",
            degree=0,
            ridge_lambda=0.0,
            correction_clip=0.5,
        )

        self.assertAlmostEqual(model.coefficients_low_to_high[0], 1.0 / 0.9 - 1.0)
        self.assertFalse(module.profile_payload(
            profile_id="unit",
            method="unit_method",
            model=model,
            profile_json=Path("profile.json"),
            scenarios=["a", "b"],
            seed_start=1,
            trial_count=2,
            sample_count=3,
            train_ratio=0.67,
            target_range_m=1852.0,
            target_window_m=30.0,
            target_oversample_count=0,
            burst_count=7,
            observations_per_burst=3,
            inner_mode="trimmed_mean",
            outer_mode="trimmed_mean",
            trim_fraction=0.05,
            common_fraction=0.5,
        )["runtime_truth_used"])

    def test_parse_scenarios_rejects_unknown_name(self) -> None:
        with self.assertRaises(Exception):
            module.parse_scenarios("bbox_outlier_stress,unknown")


if __name__ == "__main__":
    unittest.main()
