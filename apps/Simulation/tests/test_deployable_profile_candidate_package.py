from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
EXPORT_PATH = ROOT_DIR / "scripts" / "export_deployable_profile_candidate_package.py"
CHECK_PATH = ROOT_DIR / "scripts" / "check_deployable_profile_candidate_package.py"


def load_module(name: str, path: Path):
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


def make_candidate_fixture(root: Path) -> Path:
    checker = load_module("candidate_package_checker_hash_helper", CHECK_PATH)
    active = root / "active_runtime_profile.json"
    write_json(
        active,
        {
            "profile_id": "unit",
            "method": "range_best",
            "mode": "estimated_to_truth_sim3",
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
            "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth", "true_disparity"],
            "scale": 1.0,
            "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "translation_rdf_m": [0.0, 0.0, 0.0],
        },
    )
    active_sha = checker.sha256_file(active)
    policy = root / "policy.json"
    write_json(
        policy,
        {
            "status": "PASS",
            "recommended_policy": "range_priority",
            "recommended_method": "range_best",
            "recommended_profile_sha256": active_sha,
            "runtime_truth_used": False,
        },
    )
    promotion = root / "promotion_check.json"
    write_json(
        promotion,
        {
            "pass_promotion_check": True,
            "sample_apply_count": 2,
            "sample_invalid_count": 0,
            "runtime_truth_used": False,
        },
    )
    payload_json = root / "public_payload.json"
    write_json(
        payload_json,
        {
            "status": "PASS",
            "sample_count": 1,
            "invalid_count": 0,
            "node_public_payload_equivalence_status": "PASS",
            "runtime_truth_used": False,
            "input_coordinate_contract": "runtime_input:stereo_center_RDF(right,down,forward)",
            "public_topic_contract": "std_msgs/Float32MultiArray:data=[forward_m,right_m,down_m,range_m]",
        },
    )
    payload_csv = root / "payload.csv"
    with payload_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["data_axis_order", "runtime_truth_used", "node_public_payload_equivalent", "data"],
        )
        writer.writeheader()
        writer.writerow(
            {
                "data_axis_order": "forward_m,right_m,down_m,range_m",
                "runtime_truth_used": "False",
                "node_public_payload_equivalent": "True",
                "data": "[1.0, 2.0, 3.0, 4.0]",
            }
        )
    candidate = root / "candidate_export.json"
    write_json(
        candidate,
        {
            "mode": "deployable_profile_candidate_export",
            "status": "PASS",
            "recommendation": "use_as_next_acceptance_dashboard_active_profile",
            "policy_selection_json": str(policy),
            "recommended_policy": "range_priority",
            "method": "range_best",
            "output_active_profile": str(active),
            "active_profile_sha256": active_sha,
            "profile_sha256_matches_policy": True,
            "candidate_pass_count": 2,
            "candidate_expected_run_count": 2,
            "candidate_axis_pass_count": 2,
            "candidate_target_pass_count": 2,
            "promotion_check_json": str(promotion),
            "promotion_check_pass": True,
            "promotion_sample_apply_count": 2,
            "promotion_sample_invalid_count": 0,
            "public_topic_payload_json": str(payload_json),
            "public_topic_payload_csv": str(payload_csv),
            "public_topic_payload_status": "PASS",
            "public_topic_payload_sample_count": 1,
            "public_topic_payload_invalid_count": 0,
            "public_topic_payload_node_equivalence_status": "PASS",
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            "runtime_input_contract": "exported candidate consumes stereo-center RDF(right,down,forward) plus measured disparity",
            "runtime_truth_used": False,
        },
    )
    readiness = root / "candidate_readiness.json"
    write_json(
        readiness,
        {
            "mode": "deployable_profile_candidate_readiness",
            "status": "PASS",
            "candidate_export_json": str(candidate),
            "method": "range_best",
            "recommended_policy": "range_priority",
            "active_profile_sha256": active_sha,
            "candidate_pass_count": 2,
            "candidate_expected_run_count": 2,
            "candidate_axis_pass_count": 2,
            "candidate_target_pass_count": 2,
            "validation_range_rmse_corrected_mean_m": 1.0,
            "validation_range_rmse_corrected_max_m": 1.5,
            "target_max_abs_error_corrected_max_m": 2.0,
            "promotion_check_pass": True,
            "promotion_sample_apply_count": 2,
            "promotion_sample_invalid_count": 0,
            "public_topic_payload_status": "PASS",
            "public_topic_payload_sample_count": 1,
            "public_topic_payload_node_equivalence_status": "PASS",
            "runtime_truth_used": False,
        },
    )
    return readiness


class DeployableProfileCandidatePackageTests(unittest.TestCase):
    def test_exports_and_checks_candidate_package(self) -> None:
        exporter = load_module("export_deployable_profile_candidate_package_under_test", EXPORT_PATH)
        checker = load_module("check_deployable_profile_candidate_package_under_test", CHECK_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            readiness = make_candidate_fixture(root)
            package = root / "package"
            manifest = exporter.export_package(candidate_readiness_json=readiness, output_dir=package)
            result = checker.check_package(package)

        self.assertEqual(manifest["status"], "PASS")
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertEqual(result["method"], "range_best")
        self.assertEqual(result["candidate_pass_count"], 2)
        self.assertFalse(result["runtime_truth_used"])

    def test_checker_fails_when_active_profile_missing(self) -> None:
        exporter = load_module("export_deployable_profile_candidate_package_under_test_missing", EXPORT_PATH)
        checker = load_module("check_deployable_profile_candidate_package_under_test_missing", CHECK_PATH)
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            readiness = make_candidate_fixture(root)
            package = root / "package"
            exporter.export_package(candidate_readiness_json=readiness, output_dir=package)
            (package / "active_runtime_profile.json").unlink()
            result = checker.check_package(package)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("files:active_profile:missing", result["failures"])


if __name__ == "__main__":
    unittest.main()
