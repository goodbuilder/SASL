from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_observation_fusion_deployment_inventory.py"
SPEC = importlib.util.spec_from_file_location("export_observation_fusion_deployment_inventory_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class ExportObservationFusionDeploymentInventoryTests(unittest.TestCase):
    def test_build_inventory_records_size_and_sha256_excluding_inventory_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            (root / "a.txt").write_text("alpha\n", encoding="utf-8")
            (root / "nested").mkdir()
            (root / "nested" / "b.txt").write_text("beta\n", encoding="utf-8")
            (root / "inventory").mkdir()
            (root / "inventory" / "old.json").write_text("excluded\n", encoding="utf-8")
            (root / "acceptance_check").mkdir()
            (root / "acceptance_check" / "check.json").write_text("excluded\n", encoding="utf-8")

            inventory = module.build_inventory(root)

        self.assertEqual(inventory["status"], "PASS")
        self.assertEqual(inventory["file_count"], 2)
        paths = [row["relative_path"] for row in inventory["files"]]
        self.assertEqual(paths, ["a.txt", "nested/b.txt"])
        self.assertTrue(all(len(row["sha256"]) == 64 for row in inventory["files"]))

    def test_validate_inventory_ignores_excluded_acceptance_check_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            (root / "a.txt").write_text("alpha\n", encoding="utf-8")
            inventory = module.build_inventory(root)
            (root / "acceptance_check").mkdir()
            (root / "acceptance_check" / "rerun.json").write_text("rerun\n", encoding="utf-8")

            result = module.validate_inventory(root, inventory)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)

    def test_validate_inventory_detects_changed_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            path = root / "a.txt"
            path.write_text("alpha\n", encoding="utf-8")
            inventory = module.build_inventory(root)
            path.write_text("changed\n", encoding="utf-8")

            result = module.validate_inventory(root, inventory)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("inventory:size_mismatch:a.txt", result["failures"])
        self.assertIn("inventory:sha256_mismatch:a.txt", result["failures"])

    def test_validate_inventory_detects_unlisted_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            (root / "a.txt").write_text("alpha\n", encoding="utf-8")
            inventory = module.build_inventory(root)
            (root / "extra.txt").write_text("extra\n", encoding="utf-8")

            result = module.validate_inventory(root, inventory)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("inventory:unexpected_file:extra.txt", result["failures"])

    def test_export_inventory_writes_json_csv_and_text(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            output = root / "inventory"
            (root / "a.txt").write_text("alpha\n", encoding="utf-8")

            payload = module.export_inventory(root, output)

            json_payload = json.loads((output / "observation_fusion_deployment_inventory.json").read_text(encoding="utf-8"))
            with (output / "observation_fusion_deployment_inventory.csv").open(newline="", encoding="utf-8") as handle:
                csv_rows = list(csv.DictReader(handle))

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(json_payload["status"], "PASS")
        self.assertEqual(csv_rows[0]["relative_path"], "a.txt")
        self.assertEqual(csv_rows[0]["size_bytes"], "6")


if __name__ == "__main__":
    unittest.main()
