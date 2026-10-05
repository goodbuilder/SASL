from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "extract_observation_fusion_input_from_calibration_csv.py"
SPEC = importlib.util.spec_from_file_location("extract_observation_fusion_input_from_calibration_csv_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ExtractObservationFusionInputFromCalibrationCsvTests(unittest.TestCase):
    def test_drops_truth_columns_and_extracts_runtime_observation(self) -> None:
        row = {
            "group_id": "g0",
            "burst_id": "b0",
            "truth_north_m": 800.0,
            "truth_east_m": 4.0,
            "truth_down_m": 1.5,
            "left_cx_px": 650.0,
            "left_cy_px": 360.0,
            "right_cx_px": 640.0,
            "right_cy_px": 360.0,
            "measured_disparity_px": 10.0,
        }

        output, failures, dropped = module.extract_rows([row], columns=list(row.keys()))

        self.assertEqual(failures, [])
        self.assertEqual(dropped, ["truth_down_m", "truth_east_m", "truth_north_m"])
        self.assertEqual(len(output), 1)
        self.assertEqual(output[0]["group_id"], "g0")
        self.assertAlmostEqual(output[0]["left_u_px"], 650.0)
        self.assertAlmostEqual(output[0]["left_v_px"], 360.0)
        self.assertAlmostEqual(output[0]["measured_disparity_px"], 10.0)
        self.assertFalse(output[0]["runtime_truth_used"])
        self.assertIn("truth_north_m", output[0]["calibration_truth_columns_dropped"])

    def test_negative_disparity_still_fails_after_truth_drop(self) -> None:
        row = {
            "group_id": "g0",
            "truth_north_m": 800.0,
            "left_cx_px": 640.0,
            "left_cy_px": 360.0,
            "right_cx_px": 650.0,
            "right_cy_px": 360.0,
        }

        output, failures, dropped = module.extract_rows([row], columns=list(row.keys()))

        self.assertEqual(output, [])
        self.assertEqual(dropped, ["truth_north_m"])
        self.assertIn("row0:measured_disparity_px_not_positive", failures)


if __name__ == "__main__":
    unittest.main()
