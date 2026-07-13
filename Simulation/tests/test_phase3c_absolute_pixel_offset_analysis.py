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

    def _corrected(self, du, dv, **kwargs):
        item = cell(du, dv, **kwargs)
        item["corrected_delta_u_px"] = du
        item["corrected_delta_v_px"] = dv
        item["valid_frame_count"] = 20
        return item

    def test_radius_dependence_uses_complete_strata_and_vector_ranges(self):
        stable = [self._corrected(0.0, 0.0, radius=r, marker=m, camera=c) for r in (0.5, 1.0, 2.0) for m in ("center", "right_positive", "down_positive", "nearer", "farther") for c in ("left", "right")]
        self.assertFalse(module.evaluate_radius_dependence(stable)["detected"])
        changing = [self._corrected((r - 1.0) * 0.2 if m != "farther" else 0.0, 0.0, radius=r, marker=m, camera=c) for r in (0.5, 1.0, 2.0) for m in ("center", "right_positive", "down_positive", "nearer", "farther") for c in ("left", "right")]
        result = module.evaluate_radius_dependence(changing)
        self.assertTrue(result["detected"])
        self.assertEqual(result["evaluable_strata_count"], 10)
        self.assertEqual(result["dependent_strata_count"], 8)
        missing = [item for item in changing if not (item["marker"] == "center" and item["camera"] == "left" and item["radius_m"] == 2.0)]
        self.assertFalse(module.evaluate_radius_dependence(missing)["strata"]["center|left"]["evaluable"])

    def test_segmentation_uses_radius_one_and_cell_ranges(self):
        stable = [self._corrected(0.0, 0.0, radius=radius, marker=marker, camera=camera, threshold=threshold, mode=mode) for radius in (0.5, 1.0, 2.0) for marker in ("center", "right_positive", "down_positive", "nearer", "farther") for camera in ("left", "right") for threshold, mode in ((40.0, "weighted_rgb_dominance"), (80.0, "weighted_rgb_dominance"))]
        self.assertFalse(module.evaluate_segmentation_dependence(stable)["detected"])
        changing = []
        for marker in ("center", "right_positive", "down_positive", "nearer", "farther"):
            for camera in ("left", "right"):
                for threshold, mode in ((40.0, "weighted_rgb_dominance"), (80.0, "weighted_rgb_dominance")):
                    du = 0.2 if marker != "farther" and threshold == 80.0 else 0.0
                    changing.append(self._corrected(du, 0.0, radius=1.0, marker=marker, camera=camera, threshold=threshold, mode=mode))
        result = module.evaluate_segmentation_dependence(changing)
        self.assertTrue(result["detected"])
        self.assertEqual(result["dependent_cell_count"], 8)
        changing.append(self._corrected(10.0, 0.0, radius=2.0, marker="farther", camera="left", threshold=40, mode="weighted_rgb_dominance"))
        self.assertTrue(module.evaluate_segmentation_dependence(changing)["detected"])

    def test_configuration_eligibility_requires_24_and_each_radius_8(self):
        cells = []
        for radius in (0.5, 1.0, 2.0):
            for marker in ("center", "right_positive", "down_positive", "nearer", "farther"):
                for camera in ("left", "right"):
                    cells.append(self._corrected(0.0, 0.0, radius=radius, marker=marker, camera=camera))
        result = module.evaluate_configuration_eligibility(cells)
        self.assertTrue(result["details"]["80|weighted_rgb_dominance"]["eligible"])
        self.assertEqual(result["eligible_configuration_count"], 1)
        self.assertFalse(result["eligible"])
        sparse = [item for item in cells if not (item["radius_m"] == 2.0 and item["marker"] in ("farther", "nearer"))]
        self.assertFalse(module.evaluate_configuration_eligibility(sparse)["details"]["80|weighted_rgb_dominance"]["eligible"])

    def test_intrinsic_variants_are_actual_reprojections(self):
        cells = [self._corrected(0.5, -0.25)]
        cells[0].update({"rendered_u_px": 10.5, "rendered_v_px": 4.75, "analytic_u_px": 10.0, "analytic_v_px": 5.0, "analytic_fx_px": 10.0, "analytic_fy_px": 10.0, "analytic_cx_px": 0.0, "analytic_cy_px": 0.0, "camera_info_fx_px": 20.0, "camera_info_fy_px": 20.0, "camera_info_cx_px": 1.0, "camera_info_cy_px": 1.0})
        variants, failures = module._variant_cells(cells, (-0.5, -0.5))
        self.assertFalse(failures)
        self.assertNotEqual(variants[0]["variant_A_residual_norm_px"], variants[0]["variant_B_residual_norm_px"])
        self.assertNotEqual(variants[0]["variant_C_residual_norm_px"], variants[0]["variant_D_residual_norm_px"])
        cells[0]["camera_info_fx_px"] = float("nan")
        _, failures = module._variant_cells(cells, (-0.5, -0.5))
        self.assertTrue(failures)


if __name__ == "__main__":
    unittest.main()
