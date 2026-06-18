from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_deployable_profile_candidate_readiness.py"


def load_module():
    spec = importlib.util.spec_from_file_location("check_deployable_profile_candidate_readiness_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_profile(path: Path) -> None:
    path.write_text(
        json.dumps(
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
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def write_payload_csv(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
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


def make_fixture(root: Path, *, axis_pass_count: int = 2) -> Path:
    module = load_module()
    active = root / "active_runtime_profile.json"
    write_profile(active)
    active_sha = module.sha256_file(active)
    policy = root / "policy.json"
    policy.write_text(
        json.dumps(
            {
                "status": "PASS",
                "recommended_policy": "range_priority",
                "recommended_method": "range_best",
                "recommended_profile_sha256": active_sha,
                "runtime_truth_used": False,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    promotion = root / "promotion_check" / "promotion_check.json"
    promotion.parent.mkdir()
    promotion.write_text(
        json.dumps(
            {
                "pass_promotion_check": True,
                "schema_load_ok": True,
                "node_loader_ok": True,
                "axis_contract_declared": True,
                "runtime_truth_used": False,
                "sample_apply_count": 1,
                "sample_invalid_count": 0,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    payload_json = root / "public_payload" / "public_corrected_position_payload_summary.json"
    payload_json.parent.mkdir()
    payload_json.write_text(
        json.dumps(
            {
                "status": "PASS",
                "runtime_truth_used": False,
                "sample_count": 1,
                "invalid_count": 0,
                "node_public_payload_equivalence_status": "PASS",
                "input_coordinate_contract": "runtime_input:stereo_center_RDF(right,down,forward)",
                "public_topic_contract": "std_msgs/Float32MultiArray:data=[forward_m,right_m,down_m,range_m]",
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    payload_csv = root / "public_payload" / "public_corrected_position_payload.csv"
    write_payload_csv(payload_csv)
    candidate = root / "candidate.json"
    candidate.write_text(
        json.dumps(
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
                "candidate_axis_pass_count": axis_pass_count,
                "candidate_target_pass_count": 2,
                "promotion_check_json": str(promotion),
                "promotion_check_pass": True,
                "promotion_sample_apply_count": 1,
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
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return candidate


class CheckDeployableProfileCandidateReadinessTests(unittest.TestCase):
    def test_passes_complete_policy_candidate_evidence(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            candidate = make_fixture(root)
            result = module.check_candidate_readiness(candidate)
            module.write_json(root / "readiness.json", result)
            module.write_text(root / "readiness.txt", result)
            module.write_csv(root / "readiness.csv", [result])
            json_exists = (root / "readiness.json").exists()
            csv_exists = (root / "readiness.csv").exists()

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failure_count"], 0)
        self.assertFalse(result["runtime_truth_used"])
        self.assertEqual(result["candidate_axis_pass_count"], 2)
        self.assertTrue(json_exists)
        self.assertTrue(csv_exists)

    def test_fails_when_axis_pass_count_is_not_all_runs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            candidate = make_fixture(Path(tmpdir), axis_pass_count=1)
            result = module.check_candidate_readiness(candidate)

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("candidate:axis_pass_count", result["failures"])


if __name__ == "__main__":
    unittest.main()
