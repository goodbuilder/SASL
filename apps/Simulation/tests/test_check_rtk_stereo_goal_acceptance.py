from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_rtk_stereo_goal_acceptance.py"


def load_module():
    spec = importlib.util.spec_from_file_location("check_rtk_stereo_goal_acceptance_under_test", SCRIPT_PATH)
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
    method = "sim3_inverse_depth_binned_residual_raw_forward_bins11"
    pipeline = root / "pipeline"
    package = root / "candidate_package"
    pipeline.mkdir()
    package.mkdir()
    sample_csv = pipeline / "calibration_samples.csv"
    write_csv(
        sample_csv,
        [
            {
                "ned_axis_contract": "px4_local_position_ned:north,east,down",
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "truth_rdf_transform_contract": "T_source_to_stereo_rdf: explicit transform required",
                "truth_north_m": 1.0,
                "truth_east_m": 2.0,
                "truth_down_ned_m": 3.0,
                "truth_right_m": 2.0,
                "truth_down_m": 3.0,
                "truth_forward_m": 1.0,
                "raw_right_m": 2.1,
                "raw_down_m": 3.1,
                "raw_forward_m": 1.1,
                "left_u_px": 640.0,
                "left_v_px": 360.0,
                "measured_disparity_px": 25.0,
            }
        ],
    )
    method_csv = pipeline / "method_comparison.csv"
    write_csv(
        method_csv,
        [
            {"method": "raw", "validation_range_rmse_m": 4.0},
            {"method": "global_rotation", "validation_range_rmse_m": 3.8},
            {"method": "global_scale_rotation", "validation_range_rmse_m": 3.0},
            {"method": "sim3", "validation_range_rmse_m": 2.5},
            {"method": "disparity_linear", "validation_range_rmse_m": 2.4},
            {"method": "quadratic_lut", "validation_range_rmse_m": 2.3},
            {"method": method, "validation_range_rmse_m": 2.0},
        ],
    )
    validation_csv = pipeline / "validation_predictions.csv"
    write_csv(
        validation_csv,
        [
            {
                "method": method,
                "split": "validation",
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "truth_right_m": 1.0,
                "pred_right_m": 1.0,
                "measured_disparity_px": 25.0,
            }
        ],
    )
    deliverables = root / "deliverables.json"
    write_json(
        deliverables,
        {
            "status": "PASS",
            "selected_method": method,
            "selected_policy": "range_priority",
            "validation_range_rmse_m": 2.0,
            "raw_validation_range_rmse_m": 4.0,
            "target_1852m_max_abs_error_m": 3.0,
            "runtime_truth_used": False,
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            "truth_scope": "PX4/RTK/NED truth is calibration/evaluation evidence only",
            "pipeline_files": {
                "method_comparison_csv": {"path": str(method_csv), "exists": True, "size_bytes": 10},
                "range_error_plot_png": {"path": str(pipeline / "range_error_before_after.png"), "exists": True, "size_bytes": 10},
                "residual_disparity_plot_png": {
                    "path": str(pipeline / "residual_disparity_by_distance.png"),
                    "exists": True,
                    "size_bytes": 10,
                },
                "rdf_axis_rmse_plot_png": {"path": str(pipeline / "rdf_axis_rmse.png"), "exists": True, "size_bytes": 10},
                "target_1852m_stats_csv": {"path": str(pipeline / "target_1852m_stats.csv"), "exists": True, "size_bytes": 10},
                "summary_txt": {"path": str(pipeline / "summary.txt"), "exists": True, "size_bytes": 10},
                "validation_predictions_csv": {"path": str(validation_csv), "exists": True, "size_bytes": 10},
            },
        },
    )
    suite = root / "suite.json"
    write_json(
        suite,
        {
            "status": "PASS",
            "sample_csv": str(sample_csv),
            "selected_policy": "range_priority",
            "selected_method": method,
            "selected_install_status": "PASS",
            "selected_deliverables_status": "PASS",
            "policy_count": 4,
            "pass_count": 4,
            "policies": ["range_priority", "target_priority", "score_priority", "balanced"],
            "runtime_truth_used": False,
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "policy_runs": [{"status": "PASS"} for _ in range(4)],
        },
    )
    write_json(
        package / "deployable_profile_candidate_package_manifest.json",
        {
            "status": "PASS",
            "target_max_abs_error_corrected_max_m": 3.0,
            "public_topic_payload_status": "PASS",
            "public_topic_payload_node_equivalence_status": "PASS",
            "runtime_truth_used": False,
        },
    )
    readme = root / "README.md"
    readme.write_text(
        "px4_local_position_ned:north,east,down\n"
        "T_source_to_stereo_rdf\n"
        "synthetic mode\n"
        "cam_r_optical_yaw_deg\n"
        "measured_disparity_px\n"
        "/fmu/out/vehicle_local_position\n"
        "camera image topic\n"
        "YOLO BBox topic\n"
        "runtime input\n"
        "train validation\n",
        encoding="utf-8",
    )
    return suite, deliverables, package, readme


class CheckRtkStereoGoalAcceptanceTests(unittest.TestCase):
    def test_acceptance_passes_complete_fixture(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            suite, deliverables, package, readme = make_fixture(Path(tmpdir))
            result = module.check_acceptance(
                suite_summary_json=suite,
                deliverables_json=deliverables,
                candidate_package_dir=package,
                readme=readme,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["pass_count"], result["requirement_count"])
        self.assertFalse(result["runtime_truth_used"])

    def test_acceptance_does_not_require_ned_truth_columns_when_rdf_truth_contract_exists(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            suite, deliverables, package, readme = make_fixture(root)
            write_csv(
                root / "pipeline" / "calibration_samples.csv",
                [
                    {
                        "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                        "truth_rdf_transform_contract": "T_source_to_stereo_rdf: explicit transform from rtk_enu",
                        "truth_right_m": 2.0,
                        "truth_down_m": 3.0,
                        "truth_forward_m": 1.0,
                        "raw_right_m": 2.1,
                        "raw_down_m": 3.1,
                        "raw_forward_m": 1.1,
                        "left_u_px": 640.0,
                        "left_v_px": 360.0,
                        "measured_disparity_px": 25.0,
                    }
                ],
            )
            result = module.check_acceptance(
                suite_summary_json=suite,
                deliverables_json=deliverables,
                candidate_package_dir=package,
                readme=readme,
            )

        self.assertEqual(result["status"], "PASS")
        self.assertNotIn("REQ-03-bbox-disparity-inputs", result["failures"])

    def test_acceptance_fails_when_required_sample_columns_missing(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            suite, deliverables, package, readme = make_fixture(root)
            write_csv(root / "pipeline" / "calibration_samples.csv", [{"rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)"}])
            result = module.check_acceptance(
                suite_summary_json=suite,
                deliverables_json=deliverables,
                candidate_package_dir=package,
                readme=readme,
            )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("REQ-03-bbox-disparity-inputs", result["failures"])


if __name__ == "__main__":
    unittest.main()
