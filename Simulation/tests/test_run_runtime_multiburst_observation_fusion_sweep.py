from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_multiburst_observation_fusion_sweep.py"
SPEC = importlib.util.spec_from_file_location("run_runtime_multiburst_observation_fusion_sweep_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RuntimeMultiburstObservationFusionSweepTests(unittest.TestCase):
    def test_parse_burst_specs(self) -> None:
        self.assertEqual(module.parse_burst_specs("1x15,3x5, 5x7"), [(1, 15), (3, 5), (5, 7)])

    def test_method_name_is_stable(self) -> None:
        self.assertEqual(
            module.method_name(
                burst_count=3,
                observations_per_burst=5,
                inner_mode="trimmed_mean",
                outer_mode="median",
                trim_fraction=0.2,
                common_fraction=1.0,
            ),
            "multiburst_observation_inner_trimmed_mean_outer_median_b3x5_trim0p2_common1",
        )

    def test_fuse_observation_tuples_applies_axiswise_median(self) -> None:
        fused = module.fuse_observation_tuples(
            [(1.0, 10.0, 100.0), (2.0, 20.0, 200.0), (99.0, -50.0, 3000.0)],
            mode="median",
            trim_fraction=0.2,
        )
        self.assertEqual(fused, (2.0, 10.0, 200.0))

    def test_synthesize_multiburst_observation_outputs_runtime_tuple(self) -> None:
        camera = module.pipe.CameraModel()
        injection = module.pipe.ErrorInjection(
            disparity_outlier_rate=0.0,
            bbox_outlier_rate=0.0,
        )
        observation = module.synthesize_multiburst_observation(
            truth_rdf_m=np.array([0.0, 0.0, 1000.0], dtype=float),
            camera=camera,
            injection=injection,
            rng=np.random.default_rng(123),
            burst_count=3,
            observations_per_burst=5,
            common_fraction=1.0,
            inner_mode="trimmed_mean",
            outer_mode="median",
            trim_fraction=0.2,
        )

        self.assertEqual(len(observation), 3)
        self.assertGreater(observation[2], 0.0)


if __name__ == "__main__":
    unittest.main()
