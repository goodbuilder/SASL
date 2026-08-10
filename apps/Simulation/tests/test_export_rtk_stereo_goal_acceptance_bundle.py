from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "export_rtk_stereo_goal_acceptance_bundle.py"


def load_module():
    spec = importlib.util.spec_from_file_location("export_rtk_stereo_goal_acceptance_bundle_under_test", SCRIPT_PATH)
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


def touch_text(path: Path, text: str = "ok\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def make_fixture(root: Path) -> tuple[Path, Path, Path, Path]:
    method = "sim3_inverse_depth_binned_residual_raw_forward_bins11"
    pipeline = root / "pipeline"
    package = root / "candidate_package"
    goal_dir = root / "goal_acceptance"
    deliverables_dir = root / "deliverables_check"
    pipeline.mkdir()
    package.mkdir()
    goal_dir.mkdir()
    deliverables_dir.mkdir()

    pipeline_files = {
        "method_comparison_csv": pipeline / "method_comparison.csv",
        "range_error_plot_png": pipeline / "range_error_before_after.png",
        "residual_disparity_plot_png": pipeline / "residual_disparity_by_distance.png",
        "rdf_axis_rmse_plot_png": pipeline / "rdf_axis_rmse.png",
        "target_1852m_stats_csv": pipeline / "target_1852m_stats.csv",
        "summary_txt": pipeline / "summary.txt",
        "validation_predictions_csv": pipeline / "validation_predictions.csv",
    }
    write_csv(pipeline_files["method_comparison_csv"], [{"method": method, "validation_range_rmse_m": 1.8}])
    write_csv(pipeline_files["target_1852m_stats_csv"], [{"method": method, "max_abs_error_m": 4.5}])
    write_csv(pipeline_files["validation_predictions_csv"], [{"method": method, "split": "validation"}])
    for role in ("range_error_plot_png", "residual_disparity_plot_png", "rdf_axis_rmse_plot_png", "summary_txt"):
        touch_text(pipeline_files[role], f"{role}\n")

    goal_json = goal_dir / "rtk_stereo_goal_acceptance.json"
    write_json(
        goal_json,
        {
            "status": "PASS",
            "selected_policy": "range_priority",
            "selected_method": method,
            "target_error_limit_m": 10.0,
            "pass_count": 11,
            "requirement_count": 11,
            "runtime_truth_used": False,
        },
    )
    write_csv(goal_json.with_suffix(".csv"), [{"requirement_id": "REQ-01", "status": "PASS"}])
    touch_text(goal_json.with_suffix(".txt"), "status: PASS\n")

    suite_json = root / "deployable_profile_policy_package_suite_summary.json"
    write_json(
        suite_json,
        {
            "status": "PASS",
            "selected_policy": "range_priority",
            "selected_method": method,
            "selected_install_status": "PASS",
            "selected_deliverables_status": "PASS",
            "selected_goal_acceptance_status": "PASS",
            "runtime_truth_used": False,
        },
    )
    touch_text(suite_json.with_suffix(".txt"), "status: PASS\n")

    deliverables_json = deliverables_dir / "deployable_profile_deliverables_check.json"
    write_json(
        deliverables_json,
        {
            "status": "PASS",
            "selected_policy": "range_priority",
            "selected_method": method,
            "validation_range_rmse_m": 1.85,
            "raw_validation_range_rmse_m": 3.78,
            "target_1852m_max_abs_error_m": 4.52,
            "runtime_truth_used": False,
            "pipeline_files": {
                role: {"path": str(path), "exists": True, "size_bytes": path.stat().st_size}
                for role, path in pipeline_files.items()
            },
        },
    )
    touch_text(deliverables_json.with_suffix(".txt"), "status: PASS\n")

    package_files = {
        "active_runtime_profile.json": {"method": method},
        "deployable_profile_candidate_package_manifest.json": {
            "status": "PASS",
            "active_profile_sha256": "abc123",
            "runtime_truth_used": False,
        },
        "deployable_profile_candidate_package_check.json": {"status": "PASS"},
        "deployable_profile_candidate_package_portability.json": {"status": "PASS"},
        "operator_view_launch_check.json": {"status": "PASS"},
    }
    for name, payload in package_files.items():
        write_json(package / name, payload)
    for name in (
        "deployable_profile_candidate_package_check.txt",
        "deployable_profile_candidate_package_portability.txt",
        "operator_view_launch_check.txt",
        "DEPLOYABLE_PROFILE_CANDIDATE_README.txt",
    ):
        touch_text(package / name, "status: PASS\n")
    write_json(package / "evidence" / "promotion_check.json", {"status": "PASS"})

    return goal_json, suite_json, deliverables_json, package


class ExportRtkStereoGoalAcceptanceBundleTests(unittest.TestCase):
    def test_bundle_exports_required_artifacts_and_contracts(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            goal, suite, deliverables, package = make_fixture(root)
            output_dir = root / "bundle"
            result = module.export_bundle(
                goal_acceptance_json=goal,
                suite_summary_json=suite,
                deliverables_json=deliverables,
                candidate_package_dir=package,
                output_dir=output_dir,
            )

            self.assertEqual(result["status"], "PASS")
            self.assertFalse(result["runtime_truth_used"])
            self.assertIn("RDF(right,down,forward)", result["axis_contract"])
            self.assertTrue((output_dir / "candidate_package" / "active_runtime_profile.json").is_file())
            self.assertTrue((output_dir / "artifacts" / "method_comparison.csv").is_file())
            self.assertTrue((output_dir / "rtk_stereo_goal_acceptance_bundle_files.csv").is_file())

    def test_bundle_fails_when_goal_acceptance_did_not_pass(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            goal, suite, deliverables, package = make_fixture(root)
            payload = json.loads(goal.read_text(encoding="utf-8"))
            payload["status"] = "FAIL"
            write_json(goal, payload)
            result = module.export_bundle(
                goal_acceptance_json=goal,
                suite_summary_json=suite,
                deliverables_json=deliverables,
                candidate_package_dir=package,
                output_dir=root / "bundle",
            )

            self.assertEqual(result["status"], "FAIL")
            self.assertIn("goal_acceptance_status:FAIL", result["failures"])


if __name__ == "__main__":
    unittest.main()
