from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_external_bbox_topic_preflight.py"


def load_module():
    spec = importlib.util.spec_from_file_location("run_external_bbox_topic_preflight_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunExternalBBoxTopicPreflightTests(unittest.TestCase):
    def test_csv_preflight_passes_and_writes_artifacts(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "samples.csv"
            output_dir = root / "out"
            with input_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(module.bbox_validator.FIELD_NAMES)
                writer.writerow([1, 650, 340, 750, 394, 0.9, 700, 367, 1, 478, 340, 578, 394, 0.8, 528, 367])

            summary = module.run(input_csv=input_csv, output_dir=output_dir)

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["source_mode"], "csv")
            self.assertEqual(summary["sample_count"], 1)
            self.assertEqual(summary["min_valid_pairs"], 1)
            self.assertEqual(summary["valid_pair_count"], 1)
            self.assertEqual(summary["validation"]["valid_pair_count"], 1)
            self.assertFalse(summary["runtime_truth_used"])
            self.assertTrue((output_dir / "external_bbox_topic_preflight_summary.json").is_file())
            self.assertTrue((output_dir / "external_bbox_topic_contract_validation.json").is_file())

    def test_csv_preflight_fails_bad_disparity(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "samples.csv"
            output_dir = root / "out"
            with input_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(module.bbox_validator.FIELD_NAMES)
                writer.writerow([1, 450, 340, 550, 394, 0.9, 500, 367, 1, 650, 340, 750, 394, 0.8, 700, 367])

            summary = module.run(input_csv=input_csv, output_dir=output_dir)

        self.assertEqual(summary["status"], "FAIL")
        self.assertIn("validation:row0:left_minus_right_disparity_below_min:-200", summary["failures"])

    def test_csv_preflight_fails_when_valid_pairs_below_minimum(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "samples.csv"
            output_dir = root / "out"
            with input_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(module.bbox_validator.FIELD_NAMES)
                writer.writerow([1, 650, 340, 750, 394, 0.9, 700, 367, 1, 478, 340, 578, 394, 0.8, 528, 367])

            summary = module.run(input_csv=input_csv, output_dir=output_dir, min_valid_pairs=2)

        self.assertEqual(summary["status"], "FAIL")
        self.assertEqual(summary["valid_pair_count"], 1)
        self.assertIn("valid_pair_count_below_min:1<2", summary["failures"])

    def test_csv_preflight_rejects_nonpositive_min_disparity_config(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            input_csv = root / "samples.csv"
            output_dir = root / "out"
            with input_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(module.bbox_validator.FIELD_NAMES)
                writer.writerow([1, 650, 340, 750, 394, 0.9, 700, 367, 1, 478, 340, 578, 394, 0.8, 528, 367])

            summary = module.run(input_csv=input_csv, output_dir=output_dir, min_disparity_px=0.0)

        self.assertEqual(summary["status"], "FAIL")
        self.assertIn("config:min_disparity_px_not_positive:0", summary["failures"])

    def test_write_rows_csv_uses_contract_fields(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "samples.csv"
            module.write_rows_csv(path, [[1, 650, 340, 750, 394, 0.9, 700, 367, 1, 478, 340, 578, 394, 0.8, 528, 367]])
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.reader(handle))

        self.assertEqual(tuple(rows[0]), module.bbox_validator.FIELD_NAMES)
        self.assertEqual(len(rows[1]), 16)


if __name__ == "__main__":
    unittest.main()
