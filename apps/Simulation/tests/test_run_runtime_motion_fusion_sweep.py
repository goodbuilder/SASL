from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_motion_fusion_sweep.py"
SPEC = importlib.util.spec_from_file_location("run_runtime_motion_fusion_sweep_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RuntimeMotionFusionSweepTests(unittest.TestCase):
    def test_alpha_beta_clip_uses_rdf_axis_order_and_clips_outlier(self) -> None:
        samples = [
            module.pipe.CalibrationSample(
                sample_id=index,
                t_s=float(index),
                split="validation",
                truth_ned_m=np.array([0.0, 0.0, 0.0]),
                truth_rdf_m=np.array([0.0, 0.0, 0.0]),
                raw_rdf_m=np.array([0.0, 0.0, 0.0]),
                left_u_px=0.0,
                left_v_px=0.0,
                measured_disparity_px=1.0,
                true_disparity_px=1.0,
            )
            for index in range(4)
        ]
        measurements = np.array(
            [
                [1.0, 2.0, 100.0],
                [2.0, 3.0, 101.0],
                [3.0, 4.0, 180.0],
                [4.0, 5.0, 103.0],
            ],
            dtype=float,
        )
        fused = module.alpha_beta_innovation_clip(
            samples=samples,
            measurements_rdf_m=measurements,
            alpha=1.0,
            beta=0.0,
            innovation_clip_m=5.0,
            axes="forward",
        )
        self.assertEqual(fused.shape, (4, 3))
        np.testing.assert_allclose(fused[:, 0], measurements[:, 0])
        np.testing.assert_allclose(fused[:, 1], measurements[:, 1])
        self.assertLess(fused[2, 2], 110.0)

    def test_method_name_is_stable(self) -> None:
        self.assertEqual(
            module.method_name(alpha=0.8, beta=0.005, innovation_clip_m=40.0, axes="rdf"),
            "causal_alpha_beta_rdf_innovation_clip_a0p8_b0p005_g40",
        )


if __name__ == "__main__":
    unittest.main()
