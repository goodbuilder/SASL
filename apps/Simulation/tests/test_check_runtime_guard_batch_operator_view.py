from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_runtime_guard_batch_operator_view.py"
SPEC = importlib.util.spec_from_file_location("check_runtime_guard_batch_operator_view_under_test", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class CheckRuntimeGuardBatchOperatorViewTests(unittest.TestCase):
    def test_check_batch_writes_pass_summary_for_lightweight_packages(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            package_dir = root / "batch" / "bbox" / "seed_1"
            package_dir.mkdir(parents=True)
            active_profile = package_dir / "active_runtime_profile.json"
            active_profile.write_text('{"profile_id":"unit"}\n', encoding="utf-8")
            batch_csv = root / "batch.csv"
            with batch_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["scenario", "seed", "status", "active_profile"])
                writer.writeheader()
                writer.writerow(
                    {
                        "scenario": "bbox",
                        "seed": "seed_1",
                        "status": "PASS",
                        "active_profile": str(active_profile),
                    }
                )

            def fake_operator_check(**kwargs):
                return {
                    "status": "PASS",
                    "package_type": "lightweight_profile_package",
                    "package_check_status": "PASS",
                    "operator_command_returncode": 0,
                    "launch_contract_status": "PASS",
                    "resolved_runtime_profile": str(active_profile.resolve()),
                    "profile_id": "unit",
                    "method": "sim3",
                    "profile_mode": "estimated_to_truth_sim3",
                    "runtime_truth_used": False,
                }

            with patch.object(module.operator_checker, "check_operator_view_launch", side_effect=fake_operator_check):
                summary = module.check_batch(batch_csv=batch_csv, output_dir=root / "out")

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["run_count"], 1)
            self.assertEqual(summary["pass_count"], 1)
            self.assertFalse(summary["runtime_truth_used"])
            self.assertTrue((root / "out" / "runtime_guard_batch_operator_view_readiness.csv").is_file())
            summary_json = json.loads(
                (root / "out" / "runtime_guard_batch_operator_view_readiness_summary.json").read_text()
            )
            self.assertEqual(summary_json["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_missing_active_profile_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            batch_csv = root / "batch.csv"
            with batch_csv.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["scenario", "seed", "status", "active_profile"])
                writer.writeheader()
                writer.writerow(
                    {
                        "scenario": "bbox",
                        "seed": "seed_1",
                        "status": "PASS",
                        "active_profile": str(root / "missing" / "active_runtime_profile.json"),
                    }
                )

            summary = module.check_batch(batch_csv=batch_csv, output_dir=root / "out")

            self.assertEqual(summary["status"], "FAIL")
            self.assertIn("active_profile:missing", ",".join(summary["failures"]))


if __name__ == "__main__":
    unittest.main()
