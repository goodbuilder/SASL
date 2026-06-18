from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "validate_observation_fusion_runtime_input_schema.py"
SPEC = importlib.util.spec_from_file_location("validate_observation_fusion_runtime_input_schema_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ValidateObservationFusionRuntimeInputSchemaTests(unittest.TestCase):
    def write_rows(self, tmpdir: str, rows: list[dict[str, object]]) -> Path:
        path = Path(tmpdir) / "input.csv"
        fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_passes_multiburst_runtime_observation_contract(self) -> None:
        rows = [
            {
                "group_id": "g0",
                "burst_id": "b0",
                "left_u_px": 641.0,
                "left_v_px": 361.0,
                "measured_disparity_px": 1.5,
            },
            {
                "group_id": "g0",
                "burst_id": "b1",
                "left_u_px": 642.0,
                "left_v_px": 362.0,
                "measured_disparity_px": 1.5,
            },
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            result = module.validate_csv(self.write_rows(tmpdir, rows), require_burst_id=True)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["row_count"], 2)
        self.assertEqual(result["burst_count_by_group"], {"g0": 2})
        self.assertFalse(result["runtime_truth_used"])

    def test_rejects_truth_columns_and_nonpositive_disparity(self) -> None:
        rows = [
            {
                "group_id": "g0",
                "burst_id": "b0",
                "left_u_px": 641.0,
                "left_v_px": 361.0,
                "measured_disparity_px": 0.0,
                "px4_north_m": 1.0,
            }
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            result = module.validate_csv(self.write_rows(tmpdir, rows), require_burst_id=True)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("forbidden_truth_column:px4_north_m", result["failures"])
        self.assertIn("row0:measured_disparity_px_not_positive", result["failures"])

    def test_requires_burst_id_for_multiburst_gate(self) -> None:
        rows = [
            {
                "group_id": "g0",
                "left_u_px": 641.0,
                "left_v_px": 361.0,
                "measured_disparity_px": 1.5,
            }
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            result = module.validate_csv(self.write_rows(tmpdir, rows), require_burst_id=True)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("missing_required_column:burst_id", result["failures"])


if __name__ == "__main__":
    unittest.main()
