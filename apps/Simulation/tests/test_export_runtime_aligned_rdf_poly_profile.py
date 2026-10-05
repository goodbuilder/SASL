from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_runtime_aligned_rdf_poly_profile.py"
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))


def load_module():
    spec = importlib.util.spec_from_file_location("export_runtime_aligned_rdf_poly_profile_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load export_runtime_aligned_rdf_poly_profile.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportRuntimeAlignedRdfPolyProfileTests(unittest.TestCase):
    def test_zscore_poly_to_raw_disparity_preserves_predictions(self) -> None:
        module = load_module()
        z_coefficients = [0.5, -1.25, 0.75, 2.0]
        mean = 14.0
        std = 2.5
        raw_coefficients = module.zscore_poly_to_raw_disparity(z_coefficients, mean=mean, std=std)
        raw_disparities = np.array([9.0, 12.5, 14.0, 18.25], dtype=float)

        expected = np.polyval(z_coefficients, (raw_disparities - mean) / std)
        actual = np.polyval(raw_coefficients, raw_disparities)

        np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
