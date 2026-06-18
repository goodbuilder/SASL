from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_common_csv_selector_package.py"
SELECT_SCRIPT_PATH = ROOT_DIR / "scripts" / "select_stereodistance_runtime_profile_by_label.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def profile(path: Path, method: str) -> None:
    write_json(
        path,
        {
            "method": method,
            "profile_id": f"profile-{method}",
            "mode": "estimated_to_truth_sim3",
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
            "runtime_truth_used": False,
        },
    )


class ExportCommonCsvSelectorPackageTests(unittest.TestCase):
    def test_exports_selector_package_resolvable_by_label(self) -> None:
        module = load_module(SCRIPT_PATH, "export_common_csv_selector_package_under_test")
        selector_module = load_module(SELECT_SCRIPT_PATH, "select_stereodistance_runtime_profile_by_label_under_test")
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            profile(root / "a.json", "method_a")
            profile(root / "b.json", "method_b")
            profile(root / "global.json", "method_global")
            summary_json = root / "selector_summary.json"
            write_json(summary_json, {"recommended_deployable_selector_mode": "scenario_loso"})
            selected_csv = root / "selected.csv"
            write_csv(
                selected_csv,
                [
                    {
                        "selector_mode": "global_single",
                        "scenario": "scenario_a",
                        "seed": "seed_1",
                        "method": "method_global",
                        "profile_json": root / "global.json",
                        "validation_range_rmse_corrected_m": 2.0,
                        "target_max_abs_error_corrected_m": 3.0,
                        "common_csv_score": 5.0,
                    },
                    {
                        "selector_mode": "scenario_loso",
                        "scenario": "scenario_a",
                        "seed": "seed_1",
                        "method": "method_a",
                        "profile_json": root / "a.json",
                        "validation_range_rmse_corrected_m": 1.0,
                        "target_max_abs_error_corrected_m": 2.0,
                        "common_csv_score": 3.0,
                    },
                    {
                        "selector_mode": "scenario_loso",
                        "scenario": "scenario_b",
                        "seed": "seed_1",
                        "method": "method_b",
                        "profile_json": root / "b.json",
                        "validation_range_rmse_corrected_m": 1.5,
                        "target_max_abs_error_corrected_m": 2.5,
                        "common_csv_score": 4.0,
                    },
                ],
            )

            result = module.export_selector_package(
                selector_summary_json=summary_json,
                selected_runs_csv=selected_csv,
                output_dir=root / "package",
                selector_mode="scenario_loso",
                objective="rmse_then_target",
            )

            self.assertEqual(result["status"], "PASS")
            self.assertFalse(result["runtime_truth_used"])
            selector_json = Path(result["selector_json"])
            manifest = json.loads(selector_json.read_text(encoding="utf-8"))
            self.assertEqual(manifest["label_profile_map"]["scenario_a"], "scenario_scenario_a")
            self.assertIn("scenario_a:seed_1", manifest["label_profile_map"])
            selection = selector_module.select_runtime_profile_by_label(
                selector_json=selector_json,
                deployment_label="scenario_a",
            )
            self.assertEqual(selection["status"], "PASS")
            self.assertEqual(selection["method"], "method_a")
            self.assertFalse(selection["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
