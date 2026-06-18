from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_deployable_profile_deliverables.py"


def load_module():
    spec = importlib.util.spec_from_file_location("check_deployable_profile_deliverables_under_test", SCRIPT_PATH)
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


def make_fixture(root: Path) -> tuple[Path, Path, Path, Path]:
    pipeline = root / "pipeline"
    package = root / "candidate_package"
    method = "sim3_inverse_depth_binned_residual_raw_forward_bins11"
    pipeline.mkdir()
    package.mkdir()
    for name in ("range_error_before_after.png", "residual_disparity_by_distance.png", "rdf_axis_rmse.png"):
        (pipeline / name).write_bytes(b"\x89PNG\r\n\x1a\nunit")
    write_csv(
        pipeline / "method_comparison.csv",
        [
            {"method": "raw", "validation_range_rmse_m": 4.0},
            {"method": method, "validation_range_rmse_m": 2.0},
        ],
    )
    write_csv(
        pipeline / "target_1852m_stats.csv",
        [
            {
                "method": method,
                "target_range_m": 1852.0,
                "max_abs_error_m": 3.0,
                "pass_abs_mean_within_10m": "yes",
            }
        ],
    )
    write_csv(
        pipeline / "validation_predictions.csv",
        [
            {
                "method": method,
                "split": "validation",
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            }
        ],
    )
    (pipeline / "summary.txt").write_text(
        "NED contract: px4_local_position_ned:north,east,down\n"
        "RDF contract: stereo_center_opencv_optical:RDF(right,down,forward)\n",
        encoding="utf-8",
    )
    package_summary = root / "package_summary.json"
    write_json(
        package_summary,
        {
            "status": "PASS",
            "selected_policy": "range_priority",
            "selected_method": method,
            "selected_package_dir": str(package),
            "runtime_truth_used": False,
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            "candidates": [
                {
                    "status": "PASS",
                    "recommended_policy": "range_priority",
                    "method": method,
                    "package_dir": str(package),
                    "target_max_abs_error_corrected_max_m": 3.0,
                    "package_check_status": "PASS",
                    "operator_view_launch_status": "PASS",
                    "portability_status": "PASS",
                    "runtime_truth_used": False,
                }
            ],
        },
    )
    write_json(
        package / "deployable_profile_candidate_package_manifest.json",
        {
            "status": "PASS",
            "method": method,
            "active_profile_sha256": "sha",
            "runtime_truth_used": False,
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        },
    )
    write_json(package / "deployable_profile_candidate_package_check.json", {"status": "PASS"})
    write_json(package / "operator_view_launch_check.json", {"status": "PASS"})
    write_json(package / "deployable_profile_candidate_package_portability.json", {"status": "PASS"})
    install = root / "install_manifest.json"
    write_json(
        install,
        {
            "status": "PASS",
            "package_type": "candidate_package",
            "package_check_status": "PASS",
            "portability_check_status": "PASS",
            "installed_profile_sha256": "sha",
            "runtime_truth_used": False,
        },
    )
    return pipeline, package_summary, package, install


class CheckDeployableProfileDeliverablesTests(unittest.TestCase):
    def test_passes_complete_deliverable_bundle(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            pipeline, package_summary, package, install = make_fixture(Path(tmpdir))
            result = module.check_deliverables(
                pipeline_dir=pipeline,
                package_summary_json=package_summary,
                candidate_package_dir=package,
                install_manifest_json=install,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertEqual(result["selected_policy"], "range_priority")
        self.assertFalse(result["runtime_truth_used"])

    def test_fails_when_required_plot_missing(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            pipeline, package_summary, package, install = make_fixture(Path(tmpdir))
            (pipeline / "rdf_axis_rmse.png").unlink()
            result = module.check_deliverables(
                pipeline_dir=pipeline,
                package_summary_json=package_summary,
                candidate_package_dir=package,
                install_manifest_json=install,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("pipeline_file:rdf_axis_rmse_plot_png:missing", result["failures"])


if __name__ == "__main__":
    unittest.main()
