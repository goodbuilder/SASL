from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "convert_stereo_bbox_observations_to_fusion_input.py"
SPEC = importlib.util.spec_from_file_location("convert_stereo_bbox_observations_to_fusion_input_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ConvertStereoBboxObservationsToFusionInputTests(unittest.TestCase):
    def test_converts_bbox_extents_to_left_uv_and_disparity(self) -> None:
        rows = [
            {
                "group_id": "g0",
                "burst_id": "b0",
                "left_bbox_xmin_px": 630.0,
                "left_bbox_ymin_px": 350.0,
                "left_bbox_xmax_px": 650.0,
                "left_bbox_ymax_px": 370.0,
                "right_bbox_xmin_px": 620.0,
                "right_bbox_ymin_px": 350.0,
                "right_bbox_xmax_px": 640.0,
                "right_bbox_ymax_px": 370.0,
            }
        ]

        output, failures = module.convert_rows(rows, columns=list(rows[0].keys()))

        self.assertEqual(failures, [])
        self.assertEqual(len(output), 1)
        self.assertEqual(output[0]["group_id"], "g0")
        self.assertEqual(output[0]["burst_id"], "b0")
        self.assertAlmostEqual(output[0]["left_u_px"], 640.0)
        self.assertAlmostEqual(output[0]["left_v_px"], 360.0)
        self.assertAlmostEqual(output[0]["measured_disparity_px"], 10.0)
        self.assertFalse(output[0]["runtime_truth_used"])

    def test_accepts_center_aliases_and_explicit_disparity(self) -> None:
        rows = [
            {
                "group_id": "g0",
                "left_bbox_cx_px": 640.0,
                "left_bbox_cy_px": 360.0,
                "right_bbox_cx_px": 635.0,
                "right_bbox_cy_px": 360.5,
                "measured_disparity_px": 7.0,
            }
        ]

        output, failures = module.convert_rows(rows, columns=list(rows[0].keys()))

        self.assertEqual(failures, [])
        self.assertAlmostEqual(output[0]["measured_disparity_px"], 7.0)
        self.assertAlmostEqual(output[0]["bbox_vertical_center_delta_px"], -0.5)

    def test_rejects_truth_columns_and_negative_disparity(self) -> None:
        rows = [
            {
                "group_id": "g0",
                "left_bbox_cx_px": 640.0,
                "left_bbox_cy_px": 360.0,
                "right_bbox_cx_px": 645.0,
                "right_bbox_cy_px": 360.0,
                "px4_north_m": 1.0,
            }
        ]

        output, failures = module.convert_rows(rows, columns=list(rows[0].keys()))

        self.assertEqual(output, [])
        self.assertIn("forbidden_truth_column:px4_north_m", failures)
        self.assertIn("row0:measured_disparity_px_not_positive", failures)


if __name__ == "__main__":
    unittest.main()
