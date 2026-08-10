from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_selector_observation_csv_from_operator_telemetry.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "export_selector_observation_csv_from_operator_telemetry_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportSelectorObservationCsvFromOperatorTelemetryTests(unittest.TestCase):
    def write_telemetry(self, path: Path, *, corrected: bool = True) -> None:
        columns = [
            "time_sec",
            "axis_contract",
            "drone_forward_m",
            "drone_right_m",
            "drone_down_m",
            "ground_truth_m",
            "raw_forward_m",
            "raw_right_m",
            "raw_down_m",
            "raw_range_m",
            "left_cx",
            "left_cy",
            "raw_disparity_px",
        ]
        if corrected:
            columns.extend(["corrected_forward_m", "corrected_right_m", "corrected_down_m", "corrected_range_m"])
        rows = [columns]
        values = [
            "0.0",
            "stereo_center_frd:forward_right_down",
            "1000.0",
            "8.0",
            "4.0",
            "1000.04",
            "1002.0",
            "7.5",
            "4.2",
            "1002.04",
            "700.0",
            "390.0",
            "200.0",
        ]
        if corrected:
            values.extend(["1001.0", "7.8", "4.1", "1001.04"])
        rows.append(values)
        path.write_text("\n".join(",".join(row) for row in rows) + "\n", encoding="utf-8")

    def test_exports_truth_free_rdf_selector_csv_from_operator_frd(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            telemetry = root / "telemetry.csv"
            output = root / "selector_observations.csv"
            self.write_telemetry(telemetry)

            summary = module.export_observation_csv(input_csv=telemetry, output_csv=output)
            with output.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["output_row_count"], 1)
        self.assertGreater(summary["dropped_truth_column_count"], 0)
        self.assertFalse(summary["runtime_truth_used"])
        self.assertEqual(rows[0]["left_u_px"], "700.0")
        self.assertEqual(rows[0]["left_v_px"], "390.0")
        self.assertEqual(rows[0]["measured_disparity_px"], "200.0")
        self.assertEqual(rows[0]["nominal_pose_right_m"], "7.5")
        self.assertEqual(rows[0]["nominal_pose_down_m"], "4.2")
        self.assertEqual(rows[0]["nominal_pose_forward_m"], "1002.0")
        self.assertEqual(rows[0]["observation_right_m"], "7.8")
        self.assertEqual(rows[0]["observation_down_m"], "4.1")
        self.assertEqual(rows[0]["observation_forward_m"], "1001.0")
        self.assertIn("public FRD", summary["coordinate_contract"])

    def test_missing_corrected_can_fallback_to_raw(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            telemetry = root / "telemetry.csv"
            output = root / "selector_observations.csv"
            self.write_telemetry(telemetry, corrected=False)

            summary = module.export_observation_csv(
                input_csv=telemetry,
                output_csv=output,
                allow_missing_corrected=True,
            )
            with output.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(rows[0]["observation_forward_m"], "1002.0")
        self.assertEqual(rows[0]["observation_right_m"], "7.5")
        self.assertEqual(rows[0]["observation_down_m"], "4.2")

    def test_missing_corrected_fails_without_fallback(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            telemetry = root / "telemetry.csv"
            output = root / "selector_observations.csv"
            self.write_telemetry(telemetry, corrected=False)

            summary = module.export_observation_csv(input_csv=telemetry, output_csv=output)

        self.assertEqual(summary["status"], "FAIL")
        self.assertIn("row0:missing_or_nonfinite:observation_forward_m", summary["failures"])

    def test_skips_warmup_rows_when_later_rows_are_valid(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            telemetry = root / "telemetry.csv"
            output = root / "selector_observations.csv"
            columns = [
                "time_sec",
                "raw_forward_m",
                "raw_right_m",
                "raw_down_m",
                "left_cx",
                "left_cy",
                "raw_disparity_px",
                "corrected_forward_m",
                "corrected_right_m",
                "corrected_down_m",
            ]
            telemetry.write_text(
                ",".join(columns)
                + "\n"
                + "0,1002,7.5,4.2,,,200,1001,7.8,4.1\n"
                + "1,1002,7.5,4.2,700,390,200,1001,7.8,4.1\n",
                encoding="utf-8",
            )

            summary = module.export_observation_csv(input_csv=telemetry, output_csv=output)
            with output.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["input_row_count"], 2)
        self.assertEqual(summary["output_row_count"], 1)
        self.assertEqual(summary["skipped_invalid_row_count"], 1)
        self.assertIn("row0:missing_or_nonfinite:left_u_px", summary["skipped_row_failures"])
        self.assertEqual(rows[0]["left_u_px"], "700.0")


if __name__ == "__main__":
    unittest.main()
