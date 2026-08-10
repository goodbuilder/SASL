from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "summarize_rtk_stereo_acceptance_matrices.py"


def load_module():
    spec = importlib.util.spec_from_file_location("summarize_rtk_stereo_acceptance_matrices_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_rtk_stereo_acceptance_matrices.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def complete_single_evidence_matrix(module, *, overrides=None):
    matrix = {
        item_name: {"status": "PASS", "path": f"{item_name}.artifact", "evidence": {}}
        for item_name in module.DELIVERABLE_EVIDENCE_ITEMS
    }
    if overrides:
        matrix.update(overrides)
    return matrix


class SummarizeRtkStereoAcceptanceMatricesTests(unittest.TestCase):
    def test_summarizes_all_acceptance_matrices(self) -> None:
        module = load_module()
        single = {
            "status": "PASS",
            "recommendation": "single_method",
            "deliverable_evidence_matrix": complete_single_evidence_matrix(module),
        }
        multi = {
            "multi_seed_acceptance_pass": True,
            "multi_seed_recommendation": {"method": "multi_method"},
            "multi_seed_acceptance_matrix": {
                "seed_run_coverage": {"status": "PASS", "path": "seed_runs.csv", "evidence": {}},
            },
        }
        robust = {
            "robustness_acceptance_pass": True,
            "robust_recommendation": {"method": "robust_method"},
            "deployment_strategy_recommendation": {"deployment_strategy": "single_runtime_profile"},
            "robustness_acceptance_matrix": {
                "deployment_strategy": {
                    "status": "PASS",
                    "path": "deployment_strategy_recommendation.json",
                    "evidence": {},
                },
            },
        }

        summary = module.summarize_acceptance(
            single_audit=single,
            multi_seed_summary=multi,
            robustness_summary=robust,
        )

        self.assertTrue(summary["overall_pass"])
        self.assertEqual(summary["failure_count"], 0)
        self.assertEqual(summary["matrix_item_count"], len(module.DELIVERABLE_EVIDENCE_ITEMS) + 2)
        self.assertEqual(summary["recommendations"]["single_run_recommendation"], "single_method")
        self.assertEqual(summary["recommendations"]["multi_seed_recommendation"], "multi_method")
        self.assertEqual(summary["recommendations"]["robust_recommendation"], "robust_method")
        self.assertEqual(summary["recommendations"]["deployment_strategy"], "single_runtime_profile")
        self.assertEqual(summary["rdf_axis_contract"], module.RDF_AXIS_CONTRACT)
        self.assertFalse(summary["runtime_truth_used"])

    def test_exposes_failed_matrix_items(self) -> None:
        module = load_module()
        summary = module.summarize_acceptance(
            single_audit={
                "status": "PASS",
                "deliverable_evidence_matrix": complete_single_evidence_matrix(module),
            },
            multi_seed_summary={
                "multi_seed_acceptance_pass": False,
                "multi_seed_acceptance_matrix": {
                    "representative_cross_seed_replay": {
                        "status": "FAIL",
                        "path": "representative_cross_seed_replay_summary.csv",
                        "evidence": {},
                    },
                },
            },
            robustness_summary={
                "robustness_acceptance_pass": True,
                "robustness_acceptance_matrix": {
                    "deployment_strategy": {
                        "status": "PASS",
                        "path": "deployment_strategy_recommendation.json",
                        "evidence": {},
                    },
                },
            },
        )

        self.assertFalse(summary["overall_pass"])
        self.assertEqual(summary["failure_count"], 1)
        self.assertEqual(summary["failed_items"][0]["stage"], "multi_seed_acceptance")
        self.assertEqual(summary["failed_items"][0]["item"], "representative_cross_seed_replay")

    def test_missing_required_single_run_evidence_fails_summary(self) -> None:
        module = load_module()
        incomplete_matrix = complete_single_evidence_matrix(module)
        incomplete_matrix.pop("rdf_axis_rmse_nonregression")

        summary = module.summarize_acceptance(
            single_audit={
                "status": "PASS",
                "deliverable_evidence_matrix": incomplete_matrix,
            },
            multi_seed_summary={
                "multi_seed_acceptance_pass": True,
                "multi_seed_acceptance_matrix": {
                    "seed_run_coverage": {"status": "PASS", "path": "seed_runs.csv", "evidence": {}}
                },
            },
            robustness_summary={
                "robustness_acceptance_pass": True,
                "robustness_acceptance_matrix": {
                    "deployment_strategy": {
                        "status": "PASS",
                        "path": "deployment_strategy_recommendation.json",
                        "evidence": {},
                    }
                },
            },
        )

        self.assertFalse(summary["overall_pass"])
        self.assertFalse(summary["stage_status"]["single_run_output_audit"])
        self.assertIn(
            "rdf_axis_rmse_nonregression",
            {row["item"] for row in summary["failed_items"]},
        )

    def test_includes_active_profile_selection_when_provided(self) -> None:
        module = load_module()
        summary = module.summarize_acceptance(
            single_audit={
                "status": "PASS",
                "deliverable_evidence_matrix": complete_single_evidence_matrix(module),
            },
            multi_seed_summary={
                "multi_seed_acceptance_pass": True,
                "multi_seed_acceptance_matrix": {
                    "seed_run_coverage": {"status": "PASS", "path": "seed_runs.csv", "evidence": {}}
                },
            },
            robustness_summary={
                "robustness_acceptance_pass": True,
                "robustness_acceptance_matrix": {
                    "deployment_strategy": {
                        "status": "PASS",
                        "path": "deployment_strategy_recommendation.json",
                        "evidence": {},
                    }
                },
            },
            active_profile_selection={
                "selected": True,
                "selection_mode": "deployment_strategy_single_runtime_profile",
                "deployment_strategy": "single_runtime_profile",
                "method": "sim3",
                "output_profile": "active_runtime_profile.json",
                "axis_contract": module.RDF_AXIS_CONTRACT,
                "runtime_input_contract": "no runtime truth",
            },
            active_profile_file_check={
                "pass_active_profile_file_check": True,
                "runtime_loader_status": "PASS",
                "runtime_apply_status": "ok",
            },
            promotion_check={
                "pass_promotion_check": True,
                "schema_load_ok": True,
                "node_loader_ok": True,
                "axis_contract_declared": True,
                "runtime_truth_used": False,
                "forbidden_runtime_inputs_declared": True,
                "sample_apply_count": 3,
                "sample_invalid_count": 0,
                "profile_path": "active_runtime_profile.json",
                "profile_mode": "estimated_to_truth_sim3",
            },
        )

        self.assertTrue(summary["overall_pass"])
        self.assertTrue(summary["active_profile_selected"])
        self.assertTrue(summary["stage_status"]["active_profile_export"])
        self.assertTrue(summary["stage_status"]["standalone_profile_promotion"])
        self.assertEqual(summary["matrix_rows"][-2]["stage"], "active_profile_export")
        self.assertEqual(summary["matrix_rows"][-1]["stage"], "standalone_profile_promotion")
        self.assertEqual(summary["matrix_rows"][-1]["status"], "PASS")
        self.assertTrue(summary["active_profile_file_check"]["pass_active_profile_file_check"])

    def test_standalone_promotion_check_failure_fails_overall_summary(self) -> None:
        module = load_module()
        summary = module.summarize_acceptance(
            single_audit={
                "status": "PASS",
                "deliverable_evidence_matrix": complete_single_evidence_matrix(module),
            },
            multi_seed_summary={
                "multi_seed_acceptance_pass": True,
                "multi_seed_acceptance_matrix": {
                    "seed_run_coverage": {"status": "PASS", "path": "seed_runs.csv", "evidence": {}}
                },
            },
            robustness_summary={
                "robustness_acceptance_pass": True,
                "robustness_acceptance_matrix": {
                    "deployment_strategy": {
                        "status": "PASS",
                        "path": "deployment_strategy_recommendation.json",
                        "evidence": {},
                    }
                },
            },
            promotion_check={
                "pass_promotion_check": True,
                "schema_load_ok": True,
                "node_loader_ok": True,
                "axis_contract_declared": True,
                "runtime_truth_used": False,
                "forbidden_runtime_inputs_declared": True,
                "sample_apply_count": 0,
                "sample_invalid_count": 0,
                "profile_path": "active_runtime_profile.json",
            },
        )

        self.assertFalse(summary["overall_pass"])
        self.assertFalse(summary["stage_status"]["standalone_profile_promotion"])
        self.assertEqual(summary["matrix_rows"][-1]["status"], "FAIL")

    def test_active_profile_file_check_failure_fails_overall_summary(self) -> None:
        module = load_module()
        summary = module.summarize_acceptance(
            single_audit={
                "status": "PASS",
                "deliverable_evidence_matrix": complete_single_evidence_matrix(module),
            },
            multi_seed_summary={
                "multi_seed_acceptance_pass": True,
                "multi_seed_acceptance_matrix": {
                    "seed_run_coverage": {"status": "PASS", "path": "seed_runs.csv", "evidence": {}}
                },
            },
            robustness_summary={
                "robustness_acceptance_pass": True,
                "robustness_acceptance_matrix": {
                    "deployment_strategy": {
                        "status": "PASS",
                        "path": "deployment_strategy_recommendation.json",
                        "evidence": {},
                    }
                },
            },
            active_profile_selection={
                "selected": True,
                "selection_mode": "deployment_strategy_single_runtime_profile",
                "deployment_strategy": "single_runtime_profile",
                "method": "sim3",
                "output_profile": "active_runtime_profile.json",
                "axis_contract": module.RDF_AXIS_CONTRACT,
                "runtime_input_contract": "no runtime truth",
            },
            active_profile_file_check={
                "pass_active_profile_file_check": False,
                "runtime_loader_status": "PASS",
                "runtime_apply_status": "missing corrected output",
            },
        )

        self.assertFalse(summary["overall_pass"])
        self.assertFalse(summary["stage_status"]["active_profile_export"])
        self.assertEqual(summary["matrix_rows"][-1]["status"], "FAIL")

    def test_audit_active_profile_file_loads_and_applies_runtime_profile(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            profile_path = Path(tmpdir) / "active_runtime_profile.json"
            profile_path.write_text(
                json.dumps(
                    {
                        "profile_id": "unit",
                        "method": "global_inverse_depth_linear",
                        "mode": "estimated_to_truth_disparity_poly",
                        "axis_contract": module.RDF_AXIS_CONTRACT,
                        "runtime_inputs": [
                            "raw_right_m",
                            "raw_down_m",
                            "raw_forward_m",
                            "raw_disparity_px",
                        ],
                        "forbidden_runtime_inputs": [
                            "calibration_truth",
                            "px4_rtk_truth",
                            "true_disparity",
                        ],
                        "correction_space": "raw_rdf_inverse_depth_ray_scale",
                        "inverse_depth_coefficients_high_to_low": [1.0, 0.0],
                    }
                ),
                encoding="utf-8",
            )

            audit = module.audit_active_profile_file({"selected": True, "output_profile": str(profile_path)})

        self.assertTrue(audit["pass_active_profile_file_check"])
        self.assertEqual(audit["runtime_loader_status"], "PASS")
        self.assertEqual(audit["runtime_apply_status"], "estimated_to_truth_disparity_poly")
        self.assertTrue(audit["runtime_inputs_truth_free"])
        self.assertTrue(audit["forbidden_truth_inputs_declared"])

    def test_run_writes_json_csv_and_text(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            single_path = root / "single.json"
            multi_path = root / "multi.json"
            robust_path = root / "robust.json"
            single_path.write_text(
                json.dumps(
                    {
                        "status": "PASS",
                        "deliverable_evidence_matrix": complete_single_evidence_matrix(module),
                    }
                ),
                encoding="utf-8",
            )
            multi_path.write_text(
                json.dumps(
                    {
                        "multi_seed_acceptance_pass": True,
                        "multi_seed_acceptance_matrix": {
                            "seed_run_coverage": {"status": "PASS", "path": "seed_runs.csv", "evidence": {}}
                        },
                    }
                ),
                encoding="utf-8",
            )
            robust_path.write_text(
                json.dumps(
                    {
                        "robustness_acceptance_pass": True,
                        "robustness_acceptance_matrix": {
                            "deployment_strategy": {
                                "status": "PASS",
                                "path": "deployment_strategy_recommendation.json",
                                "evidence": {},
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )

            summary = module.run(
                single_audit_json=single_path,
                multi_seed_summary_json=multi_path,
                robustness_summary_json=robust_path,
                output_dir=root / "out",
            )

            self.assertTrue(summary["overall_pass"])
            self.assertTrue((root / "out" / "rtk_stereo_acceptance_matrix_summary.json").is_file())
            self.assertTrue((root / "out" / "rtk_stereo_acceptance_matrix_summary.csv").is_file())
            self.assertTrue((root / "out" / "rtk_stereo_acceptance_matrix_summary.txt").is_file())


if __name__ == "__main__":
    unittest.main()
