from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_correlated_repeated_observation_sweep.py"
SPEC = importlib.util.spec_from_file_location("run_runtime_correlated_repeated_observation_sweep_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RuntimeCorrelatedRepeatedObservationSweepTests(unittest.TestCase):
    def test_split_sigma_preserves_variance(self) -> None:
        common, independent = module.split_sigma(4.0, 0.25)
        self.assertAlmostEqual(common * common + independent * independent, 16.0)
        self.assertAlmostEqual(common, 2.0)

    def test_method_name_includes_common_fraction(self) -> None:
        self.assertEqual(
            module.method_name(
                repeat_count=15,
                mode="trimmed_mean",
                trim_fraction=0.2,
                common_fraction=0.75,
            ),
            "correlated_repeated_observation_trimmed_mean_n15_trim0p2_common0p75",
        )

    def test_synthesize_correlated_observations_outputs_runtime_measurements(self) -> None:
        camera = module.pipe.CameraModel()
        injection = module.pipe.ErrorInjection(
            disparity_outlier_rate=0.0,
            bbox_outlier_rate=0.0,
        )
        observations = module.synthesize_correlated_observations(
            truth_rdf_m=np.array([0.0, 0.0, 1000.0], dtype=float),
            camera=camera,
            injection=injection,
            rng=np.random.default_rng(123),
            repeat_count=5,
            common_fraction=0.5,
        )

        self.assertEqual(len(observations), 5)
        self.assertTrue(all(len(observation) == 3 for observation in observations))
        self.assertTrue(all(observation[2] > 0.0 for observation in observations))

    def test_parse_float_list_rejects_empty_list(self) -> None:
        with self.assertRaises(Exception):
            module.parse_float_list("")


if __name__ == "__main__":
    unittest.main()
