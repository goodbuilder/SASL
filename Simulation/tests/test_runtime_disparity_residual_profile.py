from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from Simulation.stereo_ground_sim.stereo_ground_sim.runtime_deployment_selector import (
    DisparityResidualRuntimeProfile,
    RuntimeMeasurementSample,
)


class RuntimeDisparityResidualProfileTests(unittest.TestCase):
    def test_ray_scale_profile_applies_without_truth_fields(self) -> None:
        profile = DisparityResidualRuntimeProfile.from_mapping(
            {
                "profile_type": "DisparityResidualRuntimeProfile",
                "feature_space": "log_disparity",
                "correction_space": "ray_scale",
                "degree": 1,
                "feature_mean": 2.0,
                "feature_scale": 1.0,
                "coefficients_low_to_high": [0.1, 0.0],
                "correction_clip": 0.5,
            }
        )
        sample = RuntimeMeasurementSample(
            raw_right_m=1.0,
            raw_down_m=2.0,
            raw_forward_m=10.0,
            measured_disparity_px=7.38905609893065,
        )

        corrected = profile.apply(sample)

        self.assertIsNotNone(corrected)
        assert corrected is not None
        self.assertAlmostEqual(corrected.raw_right_m, 1.1)
        self.assertAlmostEqual(corrected.raw_down_m, 2.2)
        self.assertAlmostEqual(corrected.raw_forward_m, 11.0)
        self.assertAlmostEqual(corrected.measured_disparity_px, sample.measured_disparity_px)

    def test_loads_from_json_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            path.write_text(
                json.dumps(
                    {
                        "profile_type": "DisparityResidualRuntimeProfile",
                        "feature_space": "disparity",
                        "correction_space": "forward_add",
                        "degree": 0,
                        "feature_mean": 0.0,
                        "feature_scale": 1.0,
                        "coefficients_low_to_high": [2.5],
                        "correction_clip": 3.0,
                    }
                ),
                encoding="utf-8",
            )

            profile = DisparityResidualRuntimeProfile.from_json_file(path)
            corrected = profile.apply(
                RuntimeMeasurementSample(
                    raw_right_m=1.0,
                    raw_down_m=2.0,
                    raw_forward_m=10.0,
                    measured_disparity_px=12.0,
                )
            )

        self.assertIsNotNone(corrected)
        assert corrected is not None
        self.assertAlmostEqual(corrected.raw_right_m, 1.0)
        self.assertAlmostEqual(corrected.raw_down_m, 2.0)
        self.assertAlmostEqual(corrected.raw_forward_m, 12.5)


if __name__ == "__main__":
    unittest.main()
