from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "summarize_deployable_profile_candidate_packages.py"


def load_module():
    spec = importlib.util.spec_from_file_location("summarize_deployable_profile_candidate_packages_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def make_package(root: Path, *, policy: str, method: str, rmse: float, status: str = "PASS") -> Path:
    package = root / policy
    package.mkdir()
    write_json(
        package / "deployable_profile_candidate_package_manifest.json",
        {
            "status": "PASS",
            "recommended_policy": policy,
            "method": method,
            "active_profile_sha256": f"sha-{policy}",
            "validation_range_rmse_corrected_mean_m": rmse,
            "validation_range_rmse_corrected_max_m": rmse + 1.0,
            "target_max_abs_error_corrected_max_m": 3.0,
            "candidate_pass_count": 2,
            "candidate_expected_run_count": 2,
            "candidate_axis_pass_count": 2,
            "candidate_target_pass_count": 2,
            "promotion_sample_apply_count": 5,
            "promotion_sample_invalid_count": 0,
            "public_topic_payload_sample_count": 5,
            "public_topic_payload_node_equivalence_status": "PASS",
        },
    )
    write_json(package / "deployable_profile_candidate_package_check.json", {"status": status, "failure_count": 0})
    write_json(
        package / "operator_view_launch_check.json",
        {
            "status": status,
            "package_type": "candidate_package",
            "bbox_adjustment_arg_present": True,
            "launch_contract_status": "PASS",
        },
    )
    write_json(
        package / "deployable_profile_candidate_package_portability.json",
        {
            "status": status,
            "copied_operator_view_status": status,
            "runtime_truth_used": False,
        },
    )
    return package


class SummarizeDeployableProfileCandidatePackagesTests(unittest.TestCase):
    def test_selects_lowest_rmse_passing_package(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            high = make_package(root, policy="target_priority", method="target", rmse=2.0)
            low = make_package(root, policy="range_priority", method="range", rmse=1.0)
            fail = make_package(root, policy="bad", method="bad", rmse=0.5, status="FAIL")
            summary = module.summarize([high, low, fail])
            module.write_outputs(root / "out", summary)
            csv_exists = (root / "out" / "deployable_profile_candidate_package_summary.csv").exists()

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["selected_policy"], "range_priority")
        self.assertEqual(summary["selected_method"], "range")
        self.assertEqual(summary["candidate_count"], 3)
        self.assertEqual(summary["pass_count"], 2)
        self.assertEqual(summary["candidates"][0]["portability_status"], "PASS")
        self.assertTrue(csv_exists)

    def test_rejects_failed_portability_evidence(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            low = make_package(root, policy="range_priority", method="range", rmse=1.0)
            high = make_package(root, policy="target_priority", method="target", rmse=2.0)
            write_json(
                low / "deployable_profile_candidate_package_portability.json",
                {
                    "status": "FAIL",
                    "copied_operator_view_status": "FAIL",
                    "runtime_truth_used": False,
                },
            )
            summary = module.summarize([low, high])

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["selected_policy"], "target_priority")
        self.assertEqual(summary["pass_count"], 1)


if __name__ == "__main__":
    unittest.main()
