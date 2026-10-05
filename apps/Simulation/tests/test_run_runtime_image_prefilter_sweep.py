from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_runtime_image_prefilter_sweep.py"
SPEC = importlib.util.spec_from_file_location("run_runtime_image_prefilter_sweep_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RuntimeImagePrefilterSweepTests(unittest.TestCase):
    def test_scalar_alpha_beta_filter_clips_large_pixel_jump(self) -> None:
        values = [10.0, 11.0, 50.0, 13.0]
        times = [0.0, 1.0, 2.0, 3.0]
        filtered = module.causal_alpha_beta_scalar(
            values,
            times,
            alpha=1.0,
            beta=0.0,
            innovation_clip=2.0,
        )
        self.assertEqual(filtered.shape, (4,))
        self.assertLess(filtered[2], 15.0)

    def test_reconstruct_raw_rdf_preserves_rdf_axis_contract(self) -> None:
        camera = module.pipe.CameraModel(baseline_scale_error=0.0, focal_scale_error=0.0)
        injection = module.pipe.ErrorInjection(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
        raw = module.reconstruct_raw_rdf(
            camera=camera,
            injection=injection,
            left_u_px=camera.cx_px,
            left_v_px=camera.cy_px,
            disparity_px=camera.fx_true_px * camera.baseline_m / 1000.0,
        )
        np.testing.assert_allclose(raw, np.array([-camera.baseline_m * 0.5, 0.0, 1000.0]), atol=1.0e-9)

    def test_method_name_is_stable(self) -> None:
        self.assertEqual(
            module.method_name(alpha=0.75, beta=0.005, uv_clip_px=2.5, disparity_clip_px=0.08),
            "causal_image_prefilter_a0p75_b0p005_uv2p5_d0p08",
        )


if __name__ == "__main__":
    unittest.main()
