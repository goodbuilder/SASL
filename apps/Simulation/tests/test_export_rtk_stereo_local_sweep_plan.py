from __future__ import annotations

import importlib.util
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_rtk_stereo_local_sweep_plan.py"


def load_module():
    spec = importlib.util.spec_from_file_location("export_rtk_stereo_local_sweep_plan_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
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


def make_fixture(root: Path) -> tuple[Path, Path]:
    active_method = "sim3_inverse_depth_binned_residual_raw_forward_bins11"
    catalog = root / "catalog.json"
    write_json(
        catalog,
        {
            "status": "PASS",
            "selected_method": active_method,
            "runtime_truth_used": False,
            "candidates": [
                {"method": active_method, "status": "PASS"},
                {"method": "sim3_range_normalized_binned_residual_raw_forward_bins11", "status": "PASS"},
            ],
        },
    )
    active = root / "active_runtime_profile.json"
    write_json(
        active,
        {
            "method": active_method,
            "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
            "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
            "forward_residual_raw_forward_lut": {
                "x_values_m": [1.0 for _ in range(11)],
                "prior_strength_samples": 8.0,
            },
        },
    )
    return catalog, active


class ExportRtkStereoLocalSweepPlanTests(unittest.TestCase):
    def test_plan_exports_deduplicated_truth_free_candidates(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog, active = make_fixture(root)
            output_dir = root / "plan"
            result = module.export_plan(
                catalog_audit_json=catalog,
                active_profile_json=active,
                output_dir=output_dir,
                max_candidates=8,
            )

            self.assertEqual(result["status"], "PASS")
            self.assertEqual(result["candidate_count"], 8)
            self.assertFalse(result["runtime_truth_used"])
            self.assertTrue((output_dir / "rtk_stereo_local_sweep_candidates.csv").is_file())
            self.assertTrue((output_dir / "candidate_configs").is_dir())
            candidate_names = [row["method"] for row in module.generate_candidate_rows(catalog=json.loads(catalog.read_text()), active_profile=json.loads(active.read_text()), max_candidates=8)]
            self.assertNotIn("sim3_inverse_depth_binned_residual_raw_forward_bins11", candidate_names)
            self.assertNotIn("sim3_range_normalized_binned_residual_raw_forward_bins11", candidate_names)

    def test_plan_fails_when_no_new_candidates_requested(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog, active = make_fixture(root)
            result = module.export_plan(
                catalog_audit_json=catalog,
                active_profile_json=active,
                output_dir=root / "plan",
                max_candidates=0,
            )

            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["candidate_count"], 0)

    def test_plan_excludes_methods_from_extra_csv(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            catalog, active = make_fixture(root)
            excluded_method = "sim3_inverse_depth_monotone_binned_residual_raw_forward_bins11"
            exclude_csv = root / "local_sweep_frontier.csv"
            write_csv(exclude_csv, [{"method": excluded_method}])
            result = module.export_plan(
                catalog_audit_json=catalog,
                active_profile_json=active,
                output_dir=root / "plan",
                max_candidates=8,
                exclude_methods_csv=[exclude_csv],
            )

            self.assertEqual(result["status"], "PASS")
            self.assertEqual(result["excluded_extra_method_count"], 1)
            candidate_rows = module.read_csv(root / "plan" / "rtk_stereo_local_sweep_candidates.csv")
            self.assertNotIn(excluded_method, {row["method"] for row in candidate_rows})


if __name__ == "__main__":
    unittest.main()
