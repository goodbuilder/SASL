from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_rtk_stereo_local_sweep_frontier.py"


def load_module():
    spec = importlib.util.spec_from_file_location("export_rtk_stereo_local_sweep_frontier_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


class ExportRtkStereoLocalSweepFrontierTests(unittest.TestCase):
    def test_exports_only_added_methods_with_profiles_sorted_by_validation(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            method_csv = root / "method_comparison.csv"
            status_csv = root / "local_sweep_config_status.csv"
            profiles = root / "runtime_profiles"
            write_csv(
                method_csv,
                [
                    {
                        "method": "local_b",
                        "validation_range_rmse_m": "2.0",
                        "validation_target_max_abs_error_m": "4.0",
                        "validation_target_abs_mean_error_m": "1.0",
                        "model_complexity": "6",
                    },
                    {
                        "method": "local_a",
                        "validation_range_rmse_m": "1.5",
                        "validation_target_max_abs_error_m": "5.0",
                        "validation_target_abs_mean_error_m": "1.5",
                        "model_complexity": "6",
                    },
                    {
                        "method": "catalog_only",
                        "validation_range_rmse_m": "1.0",
                        "validation_target_max_abs_error_m": "3.0",
                        "model_complexity": "3",
                    },
                    {
                        "method": "missing_profile",
                        "validation_range_rmse_m": "1.1",
                        "validation_target_max_abs_error_m": "3.0",
                        "model_complexity": "3",
                    },
                ],
            )
            write_csv(
                status_csv,
                [
                    {"method": "local_a", "status": "ADDED", "reason": "native_pipeline"},
                    {"method": "local_b", "status": "ADDED", "reason": "native_pipeline"},
                    {"method": "catalog_only", "status": "SKIPPED", "reason": "already_exists"},
                    {"method": "missing_profile", "status": "ADDED", "reason": "native_pipeline"},
                ],
            )
            write_json(profiles / "local_a.json", {"method": "local_a"})
            write_json(profiles / "local_b.json", {"method": "local_b"})

            result = module.export_frontier(
                method_comparison_csv=method_csv,
                local_sweep_status_csv=status_csv,
                runtime_profiles_dir=profiles,
                output_csv=root / "frontier.csv",
                output_json=root / "frontier.json",
                output_txt=root / "frontier.txt",
                top_n=64,
            )

            self.assertEqual(result["status"], "PASS")
            self.assertEqual(result["method_count"], 2)
            self.assertEqual(result["missing_profile_count"], 1)
            self.assertEqual(result["selected_method"], "local_a")
            self.assertFalse(result["runtime_truth_used"])
            rows = result["rows"]
            self.assertEqual([row["method"] for row in rows], ["local_a", "local_b"])
            self.assertEqual(rows[0]["rank"], 1)
            self.assertIn("RDF(right,down,forward)", rows[0]["axis_contract"])

    def test_fails_when_no_added_profile_exists(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            write_csv(root / "method_comparison.csv", [{"method": "x", "validation_range_rmse_m": "1.0"}])
            write_csv(root / "local_sweep_config_status.csv", [{"method": "x", "status": "ADDED"}])
            result = module.export_frontier(
                method_comparison_csv=root / "method_comparison.csv",
                local_sweep_status_csv=root / "local_sweep_config_status.csv",
                runtime_profiles_dir=root / "runtime_profiles",
                output_csv=root / "frontier.csv",
                output_json=root / "frontier.json",
                output_txt=root / "frontier.txt",
                top_n=64,
            )

            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["method_count"], 0)


if __name__ == "__main__":
    unittest.main()
