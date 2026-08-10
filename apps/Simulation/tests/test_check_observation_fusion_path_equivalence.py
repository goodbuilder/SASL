from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_observation_fusion_path_equivalence.py"
SPEC = importlib.util.spec_from_file_location("check_observation_fusion_path_equivalence_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def sample_row(**overrides: object) -> dict[str, object]:
    row = {
        "group_id": "g0",
        "raw_right_m": 1.0,
        "raw_down_m": 2.0,
        "raw_forward_m": 800.0,
        "raw_range_m": 800.003,
        "measured_disparity_px": 10.0,
        "left_u_px": 640.0,
        "left_v_px": 360.0,
        "corrected_right_m": 1.1,
        "corrected_down_m": 2.1,
        "corrected_forward_m": 799.0,
        "corrected_range_m": 799.003,
        "corrected_public_forward_m": 799.0,
        "corrected_public_right_m": 1.1,
        "corrected_public_down_m": 2.1,
        "corrected_public_range_m": 799.003,
    }
    row.update(overrides)
    return row


class CheckObservationFusionPathEquivalenceTests(unittest.TestCase):
    def test_accepts_equivalent_paths(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            reference = root / "reference.csv"
            candidate = root / "candidate.csv"
            write_rows(reference, [sample_row()])
            write_rows(candidate, [sample_row()])

            result = module.compare_paths(
                reference_csv=reference,
                candidate_csvs={"candidate": candidate},
                tolerance_m=1.0e-9,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertFalse(result["runtime_truth_used"])

    def test_rejects_axis_or_output_delta(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            reference = root / "reference.csv"
            candidate = root / "candidate.csv"
            write_rows(reference, [sample_row()])
            write_rows(candidate, [sample_row(corrected_public_right_m=9.0)])

            result = module.compare_paths(
                reference_csv=reference,
                candidate_csvs={"candidate": candidate},
                tolerance_m=1.0e-9,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(
            any("corrected_public_right_m" in failure for failure in result["failures"]),
            result["failures"],
        )


if __name__ == "__main__":
    unittest.main()
