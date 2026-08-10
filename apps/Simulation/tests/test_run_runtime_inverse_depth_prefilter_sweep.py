from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_inverse_depth_prefilter_sweep.py"
SPEC = importlib.util.spec_from_file_location("run_runtime_inverse_depth_prefilter_sweep_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RuntimeInverseDepthPrefilterSweepTests(unittest.TestCase):
    def test_method_name_is_stable(self) -> None:
        self.assertEqual(
            module.method_name(alpha=0.9, beta=0.001, uv_clip_px=20.0, inverse_depth_clip_1pm=0.000001),
            "causal_inverse_depth_prefilter_a0p9_b0p001_uv20_id1em06",
        )

    def test_inverse_depth_clip_uses_one_over_meter_units(self) -> None:
        values = [0.0010, 0.0011, 0.0020]
        filtered = module.image_prefilter.causal_alpha_beta_scalar(
            values,
            [0.0, 1.0, 2.0],
            alpha=1.0,
            beta=0.0,
            innovation_clip=0.00005,
        )
        self.assertLess(filtered[2], 0.0012)


if __name__ == "__main__":
    unittest.main()
