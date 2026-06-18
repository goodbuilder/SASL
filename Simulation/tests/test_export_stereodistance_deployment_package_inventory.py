from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_stereodistance_deployment_package_inventory.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "export_stereodistance_deployment_package_inventory_under_test",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExportStereoDistanceDeploymentPackageInventoryTests(unittest.TestCase):
    def test_build_inventory_excludes_inventory_dir(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            (root / "active_runtime_profile.json").write_text("{}\n", encoding="utf-8")
            (root / "evidence").mkdir()
            (root / "evidence" / "promotion_check.json").write_text("{}\n", encoding="utf-8")
            (root / "inventory").mkdir()
            (root / "inventory" / "old.json").write_text("ignored\n", encoding="utf-8")

            inventory = module.build_inventory(root)

        self.assertEqual(inventory["status"], "PASS")
        self.assertEqual(
            [row["relative_path"] for row in inventory["files"]],
            ["active_runtime_profile.json", "evidence/promotion_check.json"],
        )

    def test_validate_inventory_detects_changed_file(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            path = root / "deployment_package_manifest.json"
            path.write_text("before\n", encoding="utf-8")
            inventory = module.build_inventory(root)
            path.write_text("after\n", encoding="utf-8")

            result = module.validate_inventory(root, inventory)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("inventory:size_mismatch:deployment_package_manifest.json", result["failures"])
        self.assertIn("inventory:sha256_mismatch:deployment_package_manifest.json", result["failures"])

    def test_export_inventory_writes_json_csv_text(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            output = root / "inventory"
            (root / "DEPLOYMENT_README.txt").write_text("hello\n", encoding="utf-8")

            payload = module.export_inventory(root, output)
            json_payload = json.loads((output / "stereodistance_deployment_package_inventory.json").read_text(encoding="utf-8"))
            with (output / "stereodistance_deployment_package_inventory.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(json_payload["file_count"], 1)
        self.assertEqual(rows[0]["relative_path"], "DEPLOYMENT_README.txt")
        self.assertFalse(payload["runtime_truth_used"])
        self.assertIn("does not transform", payload["coordinate_contract"])

    def test_run_validation_mode_rechecks_existing_inventory(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            output = root / "inventory"
            (root / "active_runtime_profile.json").write_text("{}\n", encoding="utf-8")
            module.export_inventory(root, output)
            inventory_json = output / "stereodistance_deployment_package_inventory.json"

            payload = module.run(
                [
                    "--package-dir",
                    str(root),
                    "--inventory-json",
                    str(inventory_json),
                    "--output-dir",
                    str(output),
                ]
            )
            validation_txt = output / "stereodistance_deployment_package_inventory_validation.txt"
            validation_text = validation_txt.read_text(encoding="utf-8")

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(payload["actual_file_count"], 1)
        self.assertIn("status: PASS", validation_text)


if __name__ == "__main__":
    unittest.main()
