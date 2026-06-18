from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "run_deployable_profile_policy_package_suite.py"


def load_module():
    spec = importlib.util.spec_from_file_location("run_deployable_profile_policy_package_suite_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunDeployableProfilePolicyPackageSuiteTests(unittest.TestCase):
    def test_orchestrates_policy_package_suite(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)

            def fake_select_profiles(**kwargs):
                policy = kwargs["recommended_policy"]
                return {
                    "status": "PASS",
                    "recommended_policy": policy,
                    "recommended_method": f"{policy}_method",
                    "recommended_profile_sha256": f"sha-{policy}",
                    "runtime_truth_used": False,
                }

            def fake_selection_outputs(output_dir, selection):
                output_dir.mkdir(parents=True, exist_ok=True)
                (output_dir / "deployable_profile_policy_selection.json").write_text(
                    json.dumps(selection),
                    encoding="utf-8",
                )

            def fake_export_candidate(**kwargs):
                policy = kwargs["policy_selection_json"].parents[1].name
                kwargs["output_dir"].mkdir(parents=True, exist_ok=True)
                kwargs["output_active_profile"].parent.mkdir(parents=True, exist_ok=True)
                kwargs["output_active_profile"].write_text("{}", encoding="utf-8")
                payload = {
                    "mode": "deployable_profile_candidate_export",
                    "status": "PASS",
                    "method": f"{policy}_method",
                    "active_profile_sha256": f"sha-{policy}",
                    "validation_range_rmse_corrected_mean_m": 1.0,
                    "target_max_abs_error_corrected_max_m": 2.0,
                    "runtime_truth_used": False,
                }
                (kwargs["output_dir"] / "deployable_profile_candidate_export.json").write_text(
                    json.dumps(payload),
                    encoding="utf-8",
                )
                return payload

            def fake_readiness(path):
                policy = path.parents[1].name
                return {
                    "status": "PASS",
                    "method": f"{policy}_method",
                    "active_profile_sha256": f"sha-{policy}",
                    "validation_range_rmse_corrected_mean_m": 1.0,
                    "validation_range_rmse_corrected_max_m": 2.0,
                    "target_max_abs_error_corrected_max_m": 3.0,
                    "runtime_truth_used": False,
                }

            def fake_export_package(**kwargs):
                kwargs["output_dir"].mkdir(parents=True, exist_ok=True)
                (kwargs["output_dir"] / "deployable_profile_candidate_package_manifest.json").write_text(
                    json.dumps({"status": "PASS"}),
                    encoding="utf-8",
                )
                return {"status": "PASS"}

            def fake_package_check(package_dir):
                return {"status": "PASS", "failure_count": 0}

            def fake_operator(package_dir):
                return {"status": "PASS", "package_type": "candidate_package"}

            def fake_portability(package_dir):
                return {
                    "status": "PASS",
                    "copied_operator_view_status": "PASS",
                    "runtime_truth_used": False,
                }

            def fake_summary(package_dirs):
                return {
                    "status": "PASS",
                    "selected_policy": "range_priority",
                    "selected_method": "range_priority_method",
                    "selected_package_dir": str(package_dirs[0]),
                    "selected_active_profile_sha256": "sha-range_priority",
                    "runtime_truth_used": False,
                    "candidates": [],
                }

            with (
                mock.patch.object(module.policy_selector, "select_profiles", fake_select_profiles),
                mock.patch.object(module.policy_selector, "write_outputs", fake_selection_outputs),
                mock.patch.object(module.candidate_exporter, "export_candidate", fake_export_candidate),
                mock.patch.object(module.readiness_checker, "check_candidate_readiness", fake_readiness),
                mock.patch.object(module.package_exporter, "export_package", fake_export_package),
                mock.patch.object(module.package_checker, "check_package", fake_package_check),
                mock.patch.object(module.operator_checker, "check_operator_view_launch", fake_operator),
                mock.patch.object(module.portability_checker, "check_portability", fake_portability),
                mock.patch.object(module.package_summarizer, "summarize", fake_summary),
            ):
                summary = module.run_suite(
                    matrix_summary_json=root / "matrix.json",
                    runtime_profiles_dir=root / "profiles",
                    sample_csv=root / "samples.csv",
                    output_dir=root / "out",
                    policies=["range_priority", "target_priority"],
                )

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["policy_count"], 2)
        self.assertEqual(summary["pass_count"], 2)
        self.assertEqual(summary["selected_policy"], "range_priority")
        self.assertFalse(summary["runtime_truth_used"])
        self.assertEqual(summary["policy_runs"][0]["portability_status"], "PASS")

    def test_optional_pipeline_dir_runs_selected_install_and_deliverables_audit(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)

            def fake_select_profiles(**kwargs):
                policy = kwargs["recommended_policy"]
                return {
                    "status": "PASS",
                    "recommended_policy": policy,
                    "recommended_method": f"{policy}_method",
                    "recommended_profile_sha256": f"sha-{policy}",
                    "runtime_truth_used": False,
                }

            def fake_selection_outputs(output_dir, selection):
                output_dir.mkdir(parents=True, exist_ok=True)
                (output_dir / "deployable_profile_policy_selection.json").write_text(
                    json.dumps(selection),
                    encoding="utf-8",
                )

            def fake_export_candidate(**kwargs):
                policy = kwargs["policy_selection_json"].parents[1].name
                kwargs["output_dir"].mkdir(parents=True, exist_ok=True)
                kwargs["output_active_profile"].parent.mkdir(parents=True, exist_ok=True)
                kwargs["output_active_profile"].write_text("{}", encoding="utf-8")
                return {
                    "status": "PASS",
                    "method": f"{policy}_method",
                    "active_profile_sha256": f"sha-{policy}",
                    "runtime_truth_used": False,
                }

            def fake_readiness(path):
                policy = path.parents[1].name
                return {
                    "status": "PASS",
                    "method": f"{policy}_method",
                    "active_profile_sha256": f"sha-{policy}",
                    "validation_range_rmse_corrected_mean_m": 1.0,
                    "validation_range_rmse_corrected_max_m": 2.0,
                    "target_max_abs_error_corrected_max_m": 3.0,
                    "runtime_truth_used": False,
                }

            def fake_export_package(**kwargs):
                kwargs["output_dir"].mkdir(parents=True, exist_ok=True)
                return {"status": "PASS"}

            def fake_package_check(package_dir):
                return {"status": "PASS", "failure_count": 0}

            def fake_operator(package_dir):
                return {"status": "PASS", "package_type": "candidate_package"}

            def fake_portability(package_dir):
                return {"status": "PASS", "runtime_truth_used": False}

            def fake_summary(package_dirs):
                return {
                    "status": "PASS",
                    "selected_policy": "range_priority",
                    "selected_method": "range_priority_method",
                    "selected_package_dir": str(package_dirs[0]),
                    "selected_active_profile_sha256": "sha-range_priority",
                    "runtime_truth_used": False,
                    "candidates": [],
                }

            def fake_install(**kwargs):
                kwargs["output_profile"].parent.mkdir(parents=True, exist_ok=True)
                kwargs["output_profile"].write_text("{}", encoding="utf-8")
                return {
                    "status": "PASS",
                    "package_type": "candidate_package",
                    "installed_profile_sha256": "sha-range_priority",
                    "runtime_truth_used": False,
                }

            def fake_deliverables(**kwargs):
                return {
                    "status": "PASS",
                    "selected_method": "range_priority_method",
                    "runtime_truth_used": False,
                }

            def fake_goal_acceptance(**kwargs):
                return {
                    "status": "PASS",
                    "pass_count": 11,
                    "requirement_count": 11,
                    "runtime_truth_used": False,
                    "requirements": [],
                }

            with (
                mock.patch.object(module.policy_selector, "select_profiles", fake_select_profiles),
                mock.patch.object(module.policy_selector, "write_outputs", fake_selection_outputs),
                mock.patch.object(module.candidate_exporter, "export_candidate", fake_export_candidate),
                mock.patch.object(module.readiness_checker, "check_candidate_readiness", fake_readiness),
                mock.patch.object(module.package_exporter, "export_package", fake_export_package),
                mock.patch.object(module.package_checker, "check_package", fake_package_check),
                mock.patch.object(module.operator_checker, "check_operator_view_launch", fake_operator),
                mock.patch.object(module.portability_checker, "check_portability", fake_portability),
                mock.patch.object(module.package_summarizer, "summarize", fake_summary),
                mock.patch.object(module.profile_installer, "install_package", fake_install),
                mock.patch.object(module.deliverables_checker, "check_deliverables", fake_deliverables),
                mock.patch.object(module.goal_acceptance_checker, "check_acceptance", fake_goal_acceptance),
            ):
                summary = module.run_suite(
                    matrix_summary_json=root / "matrix.json",
                    runtime_profiles_dir=root / "profiles",
                    sample_csv=root / "samples.csv",
                    output_dir=root / "out",
                    policies=["range_priority"],
                    pipeline_dir=root / "pipeline",
                )

        self.assertEqual(summary["status"], "PASS")
        self.assertEqual(summary["selected_install_status"], "PASS")
        self.assertEqual(summary["selected_deliverables_status"], "PASS")
        self.assertEqual(summary["selected_goal_acceptance_status"], "PASS")
        self.assertTrue(summary["selected_install_manifest_json"])
        self.assertTrue(summary["selected_deliverables_check_json"])
        self.assertTrue(summary["selected_goal_acceptance_json"])


if __name__ == "__main__":
    unittest.main()
