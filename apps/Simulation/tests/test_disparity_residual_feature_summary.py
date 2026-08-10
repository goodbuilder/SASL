from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim import disparity_residual_feature_summary as module


class DisparityResidualFeatureSummaryTests(unittest.TestCase):
    def test_feature_summary_from_arrays_uses_rdf_axis_order(self) -> None:
        features = module.feature_summary_from_arrays(
            left_u_px=[640.0, 650.0],
            left_v_px=[360.0, 362.0],
            measured_disparity_px=[10.0, 8.0],
            fused_disparity_px=[9.8, 8.2],
            raw_rdf_m=np.array([[1.0, 2.0, 100.0], [2.0, 3.0, 120.0]], dtype=float),
            observation_rdf_m=np.array([[1.1, 2.1, 101.0], [2.2, 3.2, 121.0]], dtype=float),
            feature_label="unit",
            observation_rdf_source="unit_source",
        )

        self.assertEqual(features["feature_label"], "unit")
        self.assertEqual(features["observation_rdf_source"], "unit_source")
        self.assertAlmostEqual(float(features["raw_right_m_mean"]), 1.5)
        self.assertAlmostEqual(float(features["raw_down_m_mean"]), 2.5)
        self.assertAlmostEqual(float(features["raw_forward_m_mean"]), 110.0)
        self.assertAlmostEqual(float(features["observation_delta_forward_m_mean"]), 1.0)
        self.assertIn("RDF(right,down,forward)", str(features["axis_contract"]))
        self.assertFalse(features["runtime_truth_used"])

    def test_feature_summary_from_runtime_rows_falls_back_to_raw_observation(self) -> None:
        features = module.feature_summary_from_runtime_rows(
            [
                {
                    "left_u_px": "640",
                    "left_v_px": "360",
                    "measured_disparity_px": "10",
                    "raw_right_m": "1",
                    "raw_down_m": "2",
                    "raw_forward_m": "100",
                },
                {
                    "left_u_px": "650",
                    "left_v_px": "362",
                    "measured_disparity_px": "8",
                    "raw_right_m": "2",
                    "raw_down_m": "3",
                    "raw_forward_m": "120",
                },
            ],
            feature_label="fallback",
        )

        self.assertEqual(features["observation_rdf_source"], "raw_rdf_fallback")
        self.assertAlmostEqual(float(features["observation_delta_norm_m_mean"]), 0.0)


if __name__ == "__main__":
    unittest.main()
