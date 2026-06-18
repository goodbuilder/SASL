from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_repeated_observation_fusion_sweep.py"
SPEC = importlib.util.spec_from_file_location("run_runtime_repeated_observation_fusion_sweep_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RuntimeRepeatedObservationFusionSweepTests(unittest.TestCase):
    def test_fuse_values_median_rejects_single_outlier(self) -> None:
        self.assertEqual(module.fuse_values([1.0, 1.1, 50.0], mode="median"), 1.1)

    def test_fuse_values_trimmed_mean_rejects_edges(self) -> None:
        fused = module.fuse_values([1.0, 1.1, 1.2, 80.0, -30.0], mode="trimmed_mean", trim_fraction=0.2)
        self.assertAlmostEqual(fused, 1.1)

    def test_synthesize_observation_outputs_pixel_disparity_tuple(self) -> None:
        camera = module.pipe.CameraModel()
        injection = module.pipe.ErrorInjection(
            disparity_outlier_rate=0.0,
            bbox_outlier_rate=0.0,
        )
        obs = module.synthesize_observation(
            truth_rdf_m=np.array([0.0, 0.0, 1000.0], dtype=float),
            camera=camera,
            injection=injection,
            rng=np.random.default_rng(123),
        )
        self.assertEqual(len(obs), 3)
        self.assertGreater(obs[2], 0.0)

    def test_method_name_is_stable(self) -> None:
        self.assertEqual(
            module.method_name(repeat_count=5, mode="median", trim_fraction=0.2),
            "repeated_observation_median_n5_trim0p2",
        )


if __name__ == "__main__":
    unittest.main()
