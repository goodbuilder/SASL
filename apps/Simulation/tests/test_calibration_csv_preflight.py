from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_preflight_module():
    spec = importlib.util.spec_from_file_location(
        "calibration_csv_preflight_under_test",
        ROOT_DIR / "scripts" / "calibration_csv_preflight.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load calibration_csv_preflight.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CalibrationCsvPreflightTests(unittest.TestCase):
    def test_preflight_accepts_named_ned_truth_and_frd_raw_aliases(self) -> None:
        module = load_preflight_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            input_csv = tmp / "telemetry.csv"
            input_csv.write_text(
                "truth_north_m,truth_east_m,truth_down_ned_m,"
                "truth_right_m,truth_down_m,truth_forward_m,"
                "estimated_forward_m,estimated_right_m,estimated_down_m,"
                "measured_disparity_px,left_cx_px,left_cy_px,right_cx_px\n"
                "1852.0,8.0,4.0,8.0,4.0,1852.0,1851.0,7.5,4.1,100.0,700.0,390.0,600.0\n"
                "1848.0,7.0,3.5,7.0,3.5,1848.0,1847.0,6.8,3.7,101.0,701.0,391.0,600.0\n"
                "1855.0,9.0,4.5,9.0,4.5,1855.0,1853.0,8.8,4.2,99.0,702.0,392.0,603.0\n",
                encoding="utf-8",
            )
            summary = module.run_preflight(
                input_csv,
                tmp / "preflight",
                target_range_m=1852.0,
                target_window_m=30.0,
                min_target_count=3,
            )

        self.assertTrue(summary["pass_preflight"])
        self.assertEqual(summary["row_count"], 3)
        self.assertEqual(summary["usable_row_count"], 3)
        self.assertEqual(summary["target_window_usable_count"], 3)
        self.assertIn("explicit_transform_required", summary["truth_rdf_transform_contract"])
        self.assertIn("T_source_to_stereo_rdf", summary["truth_rdf_transform_contract"])
        self.assertIn("axis alias", summary["truth_rdf_transform_contract"])
        self.assertFalse(summary["direct_source_axis_alias_allowed"])

    def test_preflight_rejects_ned_only_truth_without_explicit_rdf(self) -> None:
        module = load_preflight_module()
        row = {
            "truth_north_m": "1852",
            "truth_east_m": "8",
            "truth_down_ned_m": "4",
            "estimated_forward_m": "1851",
            "estimated_right_m": "7.5",
            "estimated_down_m": "4.1",
            "measured_disparity_px": "100",
        }

        diagnostic = module.row_diagnostic(
            0,
            row,
            target_range_m=1852.0,
            target_window_m=30.0,
        )

        self.assertFalse(diagnostic["usable"])
        self.assertIn("missing_truth_rdf", diagnostic["reasons"])
        self.assertIn("missing_explicit_T_source_to_stereo_rdf", diagnostic["reasons"])

    def test_preflight_reports_skipped_reasons(self) -> None:
        module = load_preflight_module()
        row = {
            "truth_north_m": "1852",
            "truth_east_m": "0",
            "truth_down_ned_m": "0",
            "left_cx_px": "700",
            "left_cy_px": "390",
        }

        diagnostic = module.row_diagnostic(
            0,
            row,
            target_range_m=1852.0,
            target_window_m=30.0,
        )

        self.assertFalse(diagnostic["usable"])
        self.assertIn("missing_raw_rdf_or_triangulation_inputs", diagnostic["reasons"])
        self.assertIn("missing_measured_disparity", diagnostic["reasons"])


if __name__ == "__main__":
    unittest.main()
