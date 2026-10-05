from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_converter_module():
    spec = importlib.util.spec_from_file_location(
        "convert_truth_source_to_stereo_rdf_under_test",
        ROOT_DIR / "scripts" / "convert_truth_source_to_stereo_rdf.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load convert_truth_source_to_stereo_rdf.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_preflight_module():
    spec = importlib.util.spec_from_file_location(
        "calibration_csv_preflight_for_converter_test",
        ROOT_DIR / "scripts" / "calibration_csv_preflight.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load calibration_csv_preflight.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class TruthSourceToStereoRdfConversionTests(unittest.TestCase):
    def test_convert_ned_source_with_explicit_non_alias_transform(self) -> None:
        module = load_converter_module()
        rows = [
            {
                "truth_north_m": "100.0",
                "truth_east_m": "20.0",
                "truth_down_ned_m": "5.0",
                "estimated_right_m": "9.0",
                "estimated_down_m": "4.0",
                "estimated_forward_m": "80.0",
                "measured_disparity_px": "50.0",
            }
        ]
        transform = {
            "source_frame": "px4_local_position_ned",
            "source_axis_order": ["north", "east", "down"],
            "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
            "origin_source_m": [10.0, 2.0, 1.0],
            "rotation_euler_zyx_deg_source_to_stereo_rdf": [30.0, -5.0, 2.0],
            "translation_rdf_m": [1.0, -2.0, 3.0],
        }

        converted, summary = module.convert_rows(rows, transform, strict=True)

        self.assertEqual(summary["converted_row_count"], 1)
        self.assertFalse(summary["direct_axis_alias_allowed"])
        self.assertEqual(converted[0]["truth_rdf_conversion_status"], "CONVERTED")
        right = float(converted[0]["truth_right_m"])
        forward = float(converted[0]["truth_forward_m"])
        self.assertNotAlmostEqual(right, 20.0)
        self.assertNotAlmostEqual(forward, 100.0)
        self.assertIn("T_source_to_stereo_rdf", converted[0]["truth_rdf_transform_contract"])
        self.assertEqual(converted[0]["truth_source_axis_order"], "north,east,down")
        self.assertIn("truth_source_origin_m", converted[0])
        self.assertIn("truth_source_to_stereo_rdf_translation_m", converted[0])
        self.assertIn("truth_source_to_stereo_rdf_rotation_matrix", converted[0])

    def test_rejects_direct_ned_to_rdf_axis_alias_matrix(self) -> None:
        module = load_converter_module()
        rows = [
            {
                "truth_north_m": "100.0",
                "truth_east_m": "20.0",
                "truth_down_ned_m": "5.0",
            }
        ]
        transform = {
            "source_frame": "px4_local_position_ned",
            "source_axis_order": ["north", "east", "down"],
            "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
            "rotation_matrix_source_to_stereo_rdf": [
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
            ],
        }

        with self.assertRaisesRegex(ValueError, "forbidden direct NED-to-RDF axis alias"):
            module.convert_rows(rows, transform, strict=True)

    def test_validate_only_writes_transform_summary(self) -> None:
        module = load_converter_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            transform_json = tmp / "transform.json"
            summary_json = tmp / "summary.json"
            transform_json.write_text(
                json.dumps(
                    {
                        "source_frame": "px4_local_position_ned",
                        "source_axis_order": ["north", "east", "down"],
                        "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
                        "origin_source_m": [0.0, 0.0, 0.0],
                        "rotation_euler_zyx_deg_source_to_stereo_rdf": [12.0, -3.0, 1.5],
                        "translation_rdf_m": [0.0, 0.0, 0.0],
                    }
                ),
                encoding="utf-8",
            )
            old_argv = sys.argv
            try:
                sys.argv = [
                    "convert_truth_source_to_stereo_rdf.py",
                    "--transform-json",
                    str(transform_json),
                    "--summary-json",
                    str(summary_json),
                    "--validate-only",
                ]
                self.assertEqual(module.main(), 0)
            finally:
                sys.argv = old_argv

            summary = json.loads(summary_json.read_text(encoding="utf-8"))

        self.assertTrue(summary["valid"])
        self.assertEqual(summary["mode"], "truth_source_to_stereo_rdf_transform_validation")
        self.assertFalse(summary["direct_axis_alias_allowed"])

    def test_rejects_transform_missing_source_contract(self) -> None:
        module = load_converter_module()
        transform = {
            "rotation_euler_zyx_deg_source_to_stereo_rdf": [12.0, -3.0, 1.5],
            "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
            "translation_rdf_m": [0.0, 0.0, 0.0],
        }

        with self.assertRaisesRegex(ValueError, "source_frame is required"):
            module.validate_transform(transform)

        transform["source_frame"] = "px4_local_position_ned"
        with self.assertRaisesRegex(ValueError, "source_axis_order is required"):
            module.validate_transform(transform)

        transform["source_axis_order"] = ["north", "east", "down"]
        del transform["source_fields"]
        with self.assertRaisesRegex(ValueError, "source_fields is required"):
            module.validate_transform(transform)

    def test_rejects_ambiguous_rotation_representation(self) -> None:
        module = load_converter_module()
        transform = {
            "source_frame": "px4_local_position_ned",
            "source_axis_order": ["north", "east", "down"],
            "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
            "rotation_matrix_source_to_stereo_rdf": [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            "rotation_euler_zyx_deg_source_to_stereo_rdf": [12.0, -3.0, 1.5],
        }

        with self.assertRaisesRegex(ValueError, "provide exactly one rotation representation"):
            module.validate_transform(transform)

    def test_converted_csv_passes_preflight_with_explicit_truth_rdf(self) -> None:
        converter = load_converter_module()
        preflight = load_preflight_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            input_csv = tmp / "ned_only.csv"
            output_csv = tmp / "with_truth_rdf.csv"
            input_csv.write_text(
                "truth_north_m,truth_east_m,truth_down_ned_m,"
                "estimated_right_m,estimated_down_m,estimated_forward_m,measured_disparity_px\n"
                "1852.0,20.0,5.0,19.0,4.0,1851.0,50.0\n"
                "1848.0,22.0,5.5,21.0,5.0,1847.0,51.0\n"
                "1855.0,18.0,4.5,17.0,4.0,1853.0,49.0\n",
                encoding="utf-8",
            )
            transform_json = tmp / "transform.json"
            transform_json.write_text(
                json.dumps(
                    {
                        "source_frame": "px4_local_position_ned",
                        "source_axis_order": ["north", "east", "down"],
                        "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
                        "origin_source_m": [0.0, 0.0, 0.0],
                        "rotation_euler_zyx_deg_source_to_stereo_rdf": [12.0, -3.0, 1.5],
                        "translation_rdf_m": [0.0, 0.0, 0.0],
                    }
                ),
                encoding="utf-8",
            )
            old_argv = sys.argv
            try:
                sys.argv = [
                    "convert_truth_source_to_stereo_rdf.py",
                    "--input-csv",
                    str(input_csv),
                    "--output-csv",
                    str(output_csv),
                    "--transform-json",
                    str(transform_json),
                    "--strict",
                ]
                self.assertEqual(converter.main(), 0)
            finally:
                sys.argv = old_argv

            with output_csv.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 3)
            self.assertTrue(all(row["truth_right_m"] for row in rows))
            self.assertTrue(math.isfinite(float(rows[0]["truth_range_m"])))

            summary = preflight.run_preflight(
                output_csv,
                tmp / "preflight",
                target_range_m=1852.0,
                target_window_m=30.0,
                min_target_count=3,
            )

        self.assertTrue(summary["pass_preflight"])
        self.assertEqual(summary["usable_row_count"], 3)


if __name__ == "__main__":
    unittest.main()
