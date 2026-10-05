from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_rtk_stereo_calibration_output_inventory.py"
SPEC = importlib.util.spec_from_file_location("export_rtk_stereo_calibration_output_inventory_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ExportRtkStereoCalibrationOutputInventoryTests(unittest.TestCase):
    def test_build_inventory_excludes_inventory_and_output_audit(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            (root / "method_comparison.csv").write_text("a\n", encoding="utf-8")
            (root / "runtime_profiles").mkdir()
            (root / "runtime_profiles" / "profile.json").write_text("{}\n", encoding="utf-8")
            (root / "inventory").mkdir()
            (root / "inventory" / "old.json").write_text("excluded\n", encoding="utf-8")
            (root / "output_audit").mkdir()
            (root / "output_audit" / "audit.json").write_text("excluded\n", encoding="utf-8")

            inventory = module.build_inventory(root)

        self.assertEqual(inventory["status"], "PASS")
        self.assertEqual([row["relative_path"] for row in inventory["files"]], ["method_comparison.csv", "runtime_profiles/profile.json"])

    def test_validate_inventory_detects_changed_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            path = root / "summary.txt"
            path.write_text("before\n", encoding="utf-8")
            inventory = module.build_inventory(root)
            path.write_text("after\n", encoding="utf-8")

            result = module.validate_inventory(root, inventory)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("inventory:size_mismatch:summary.txt", result["failures"])
        self.assertIn("inventory:sha256_mismatch:summary.txt", result["failures"])

    def test_export_inventory_writes_json_csv_text(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            output = root / "inventory"
            (root / "summary.txt").write_text("hello\n", encoding="utf-8")

            payload = module.export_inventory(root, output)

            json_payload = json.loads((output / "rtk_stereo_calibration_output_inventory.json").read_text(encoding="utf-8"))
            with (output / "rtk_stereo_calibration_output_inventory.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(json_payload["file_count"], 1)
        self.assertEqual(rows[0]["relative_path"], "summary.txt")


if __name__ == "__main__":
    unittest.main()
