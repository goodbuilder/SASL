from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "evaluate_disparity_residual_feature_selector.py"
SPEC = importlib.util.spec_from_file_location("evaluate_disparity_residual_feature_selector_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def sample(sample_id: int, truth_forward: float, observed_forward: float, u: float = 640.0) -> object:
    return module.pipe.CalibrationSample(
        sample_id=sample_id,
        t_s=float(sample_id),
        split="train" if sample_id < 2 else "validation",
        truth_ned_m=np.array([truth_forward, 0.0, 0.0], dtype=float),
        truth_rdf_m=np.array([0.0, 0.0, truth_forward], dtype=float),
        raw_rdf_m=np.array([0.0, 0.0, observed_forward], dtype=float),
        left_u_px=u,
        left_v_px=360.0,
        measured_disparity_px=1000.0 / observed_forward,
        true_disparity_px=1000.0 / truth_forward,
    )


class EvaluateDisparityResidualFeatureSelectorTests(unittest.TestCase):
    def test_runtime_feature_map_contains_no_truth_named_features(self) -> None:
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

        features = module.runtime_feature_map(series)

        self.assertIn("measured_disparity_px_mean", features)
        self.assertIn("observation_forward_m_p95", features)
        self.assertFalse(any("truth" in key or "true_" in key for key in features))

    def test_centroid_selector_picks_nearest_label(self) -> None:
        rows = [
            {"scenario": "near", "left_u_px_mean": 1.0, "measured_disparity_px_mean": 10.0},
            {"scenario": "near", "left_u_px_mean": 1.1, "measured_disparity_px_mean": 10.1},
            {"scenario": "far", "left_u_px_mean": 10.0, "measured_disparity_px_mean": 1.0},
            {"scenario": "far", "left_u_px_mean": 10.1, "measured_disparity_px_mean": 1.1},
        ]

        model = module.fit_centroids(rows)
        selection = module.select_label({"left_u_px_mean": 1.05, "measured_disparity_px_mean": 10.05}, model)

        self.assertEqual(selection["selected_label"], "near")
        self.assertGreater(selection["distance_margin"], 0.0)


if __name__ == "__main__":
    unittest.main()
