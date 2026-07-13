from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "Simulation/scripts/phase3c_absolute_pixel_offset_analysis.py"
spec = importlib.util.spec_from_file_location("phase3c_absolute_pixel_offset_analysis", SCRIPT)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def cell(du, dv, *, radius=1.0, marker="center", camera="left", threshold=80.0, mode="weighted_rgb_dominance"):
    return {"delta_u_px": du, "delta_v_px": dv, "radius_m": radius, "marker": marker, "camera": camera, "threshold": threshold, "centroid_mode": mode, "valid": True}


class AbsoluteOffsetAnalysisTests(unittest.TestCase):
    def test_residual_sign_and_norm(self):
        result = module.residual_vector({"u_px": 12.0, "v_px": 9.0}, {"u_px": 10.0, "v_px": 6.0})
        self.assertEqual(result, {"du_px": 2.0, "dv_px": 3.0, "norm_px": math.hypot(2, 3)})

    def test_median_and_half_pixel_fit(self):
        cells = [cell(-0.49, -0.51), cell(-0.51, -0.49), cell(-0.5, -0.5)]
        self.assertEqual(module.continuous_median_offset(cells), (-0.5, -0.5))
        fit = module.half_pixel_candidate_scores(cells)
        self.assertEqual(fit["best_candidate_px"], [-0.5, -0.5])

    def test_cell_weighting_is_equal(self):
        cells = [cell(1.0, 0.0)] + [cell(0.0, 0.0) for _ in range(100)]
        self.assertEqual(module.continuous_median_offset(cells), (0.0, 0.0))

    def test_corrected_residual(self):
        corrected = module.apply_offset_correction([cell(-0.5, -0.5)], (-0.5, -0.5))[0]
        self.assertEqual(corrected["corrected_residual_norm_px"], 0.0)

    def test_dependence_and_cause_classification(self):
        radius_cells = [cell(0.0, 0.0, radius=r, marker=m, camera=c) for r in (0.5, 1.0, 2.0) for m in ("center", "right_positive", "down_positive", "nearer", "farther") for c in ("left", "right")]
        radius_cells.extend(cell(0.2, 0.0, radius=2.0, marker="center", camera="left") for _ in range(1))
        radius = module.evaluate_radius_dependence(radius_cells)
        self.assertFalse(radius["detected"])  # one stratum is insufficient
        evidence = {"evidence_valid": True, "half_pixel_component_present": True, "radius_dependence_detected": False, "segmentation_dependence_detected": False}
        self.assertEqual(module.classify_absolute_offset_cause(evidence)["cause_verdict"], "half_pixel_convention")

    def test_infrastructure_invalid_is_inconclusive(self):
        result = module.classify_absolute_offset_cause({"evidence_valid": False, "half_pixel_component_present": True})
        self.assertEqual(result["cause_verdict"], "inconclusive")

    def test_json_numeric_policy(self):
        self.assertIsNone(module._finite(float("inf")))
        self.assertIsNone(module._finite(float("nan")))


if __name__ == "__main__":
    unittest.main()
