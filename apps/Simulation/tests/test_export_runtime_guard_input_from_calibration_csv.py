from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_runtime_guard_input_from_calibration_csv.py"
SPEC = importlib.util.spec_from_file_location("export_runtime_guard_input_from_calibration_csv_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ExportRuntimeGuardInputFromCalibrationCsvTests(unittest.TestCase):
    def test_exports_runtime_fields_with_csv_parser_and_drops_truth(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "calibration.csv"
            with input_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "sample_id",
                        "split",
                        "rdf_axis_contract",
                        "truth_north_m",
                        "raw_right_m",
                        "raw_down_m",
                        "raw_forward_m",
                        "raw_range_m",
                        "left_u_px",
                        "left_v_px",
                        "measured_disparity_px",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "sample_id": "7",
                        "split": "validation",
                        "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                        "truth_north_m": "123.0",
                        "raw_right_m": "1.25",
                        "raw_down_m": "2.5",
                        "raw_forward_m": "300.0",
                        "raw_range_m": "300.02",
                        "left_u_px": "641.0",
                        "left_v_px": "359.0",
                        "measured_disparity_px": "12.5",
                    }
                )

            output_csv = root / "runtime.csv"
            summary = module.export_runtime_guard_input(
                input_csv=input_csv,
                output_csv=output_csv,
                output_json=root / "summary.json",
                output_txt=root / "summary.txt",
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertFalse(summary["runtime_truth_used"])
            with output_csv.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["sample_id"], "7")
            self.assertEqual(rows[0]["raw_forward_m"], "300.0")
            self.assertNotIn("truth_north_m", rows[0])
            self.assertNotIn("rdf_axis_contract", rows[0])

    def test_split_filter_and_positive_disparity_gate(self) -> None:
        rows = [
            {
                "sample_id": "0",
                "split": "train",
                "raw_right_m": "1",
                "raw_down_m": "2",
                "raw_forward_m": "3",
                "raw_range_m": "4",
                "left_u_px": "640",
                "left_v_px": "360",
                "measured_disparity_px": "5",
            },
            {
                "sample_id": "1",
                "split": "validation",
                "raw_right_m": "1",
                "raw_down_m": "2",
                "raw_forward_m": "3",
                "raw_range_m": "4",
                "left_u_px": "640",
                "left_v_px": "360",
                "measured_disparity_px": "-1",
            },
        ]

        exported, failures = module.export_rows(rows, split="validation")

        self.assertEqual(exported, [])
        self.assertIn("row1:measured_disparity_px_not_positive", failures)
        self.assertIn("no_runtime_rows_exported", failures)


if __name__ == "__main__":
    unittest.main()
