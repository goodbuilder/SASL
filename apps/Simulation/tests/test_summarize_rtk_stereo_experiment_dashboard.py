from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_dashboard_module():
    spec = importlib.util.spec_from_file_location(
        "summarize_rtk_stereo_experiment_dashboard_under_test",
        ROOT_DIR / "scripts" / "summarize_rtk_stereo_experiment_dashboard.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load summarize_rtk_stereo_experiment_dashboard.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SummarizeRtkStereoExperimentDashboardTests(unittest.TestCase):
    def test_summarize_dashboard_reads_key_recommendations(self) -> None:
        module = load_dashboard_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sweep = root / "sweep"
            policy_dir = sweep / "runtime_measurement_guard_policy_audit"
            policy_dir.mkdir(parents=True)
            policy_dir.joinpath("runtime_measurement_guard_policy_audit_summary.json").write_text(
                json.dumps(
                    {
                        "recommended_candidate_source_mode": "all_catalog",
                        "recommended_selection_policy": "transfer_risk_fallback",
                        "recommended_pass_count": 18,
                        "recommended_audit_count": 18,
                        "recommended_target_max_abs_error_max_m": 9.0,
                    }
                ),
                encoding="utf-8",
            )
            loso_policy_dir = sweep / "runtime_measurement_guard_policy_audit_loso"
            loso_policy_dir.mkdir(parents=True)
            loso_policy_dir.joinpath("runtime_measurement_guard_policy_audit_summary.json").write_text(
                json.dumps(
                    {
                        "recommended_candidate_source_mode": "all_catalog",
                        "recommended_selection_policy": "transfer_risk_fallback",
                        "recommended_pass_count": 14,
                        "recommended_audit_count": 18,
                        "recommended_target_max_abs_error_max_m": 10.1,
                    }
                ),
                encoding="utf-8",
            )
            operating_dir = sweep / "selector_operating_mode_audit"
            operating_dir.mkdir(parents=True)
            operating_dir.joinpath("selector_operating_mode_recommendation.json").write_text(
                json.dumps(
                    {
                        "recommended_when_truth_available": "calibration_replay_all_catalog",
                        "recommended_when_truth_unavailable": "runtime_transfer_risk_guard",
                    }
                ),
                encoding="utf-8",
            )
            oracle_dir = sweep / "runtime_measurement_guard_oracle_regret_transfer_risk"
            oracle_dir.mkdir(parents=True)
            oracle_dir.joinpath("runtime_measurement_guard_oracle_regret_summary.json").write_text(
                json.dumps({"target_abs_regret_max_m": 0.6}),
                encoding="utf-8",
            )
            portfolio_dir = sweep / "active_profile_portfolio_audit"
            portfolio_dir.mkdir(parents=True)
            portfolio_dir.joinpath("active_profile_portfolio_recommendation.json").write_text(
                json.dumps(
                    {
                        "selected": False,
                        "recommended_profile_label": "target_guard_blend",
                        "transfer_replay_pass_count": 17,
                        "transfer_replay_count": 18,
                        "target_max_abs_error_max_m": 11.6,
                    }
                ),
                encoding="utf-8",
            )
            calibration_loso_dir = sweep / "profile_selection_strategy_audit_all_catalog_loso"
            calibration_loso_dir.mkdir(parents=True)
            calibration_loso_dir.joinpath("profile_selection_strategy_summary.json").write_text(
                json.dumps(
                    {
                        "calibration_pass_count": 15,
                        "run_count": 18,
                        "calibration_beats_label_count": 2,
                        "exclude_target_scenario_from_candidates": True,
                    }
                ),
                encoding="utf-8",
            )
            gap_dir = sweep / "loso_coverage_gap_summary"
            gap_dir.mkdir(parents=True)
            gap_dir.joinpath("loso_coverage_gap_summary.json").write_text(
                json.dumps(
                    {
                        "scenario_count": 6,
                        "indispensable_scenario_count": 1,
                        "full_gap_scenario_count": 1,
                        "full_gap_scenarios": ["intrinsic_scale_stress"],
                    }
                ),
                encoding="utf-8",
            )
            separability_dir = sweep / "gap_feature_separability_audit"
            separability_dir.mkdir(parents=True)
            separability_dir.joinpath("gap_feature_separability_summary.json").write_text(
                json.dumps(
                    {
                        "detection_recommendation": "raw_feature_gap_guard_candidate",
                        "best_feature": "feature_raw_forward_mean_m",
                        "best_accuracy": 1.0,
                    }
                ),
                encoding="utf-8",
            )
            threshold_loso_dir = sweep / "runtime_measurement_guard_transfer_threshold_sweep_loso"
            threshold_loso_dir.mkdir(parents=True)
            threshold_loso_dir.joinpath("runtime_measurement_guard_threshold_sweep_summary.json").write_text(
                json.dumps(
                    {
                        "recommended_rmse_limit_m": 1.8,
                        "recommended_target_limit_m": 2.5,
                        "recommended_pass_count": 15,
                        "recommended_audit_count": 18,
                        "recommended_target_max_abs_error_max_m": 9.6,
                    }
                ),
                encoding="utf-8",
            )
            intrinsic_export_dir = sweep / "operating_mode_export_intrinsic_scale_guard_smoke"
            intrinsic_export_dir.mkdir(parents=True)
            intrinsic_export_dir.joinpath("operating_mode_profile_export_summary.json").write_text(
                json.dumps(
                    {
                        "selected_operating_mode": "runtime_guard",
                        "selected_source_scenario": "intrinsic_scale_stress",
                        "selection_summary": {
                            "fallback_reason": "intrinsic_scale_product_ratio_guard_selects_scale_profile"
                        },
                    }
                ),
                encoding="utf-8",
            )
            intrinsic_calibration_dir = sweep / "intrinsic_scale_calibration_export_smoke"
            intrinsic_calibration_dir.mkdir(parents=True)
            intrinsic_calibration_dir.joinpath("intrinsic_scale_profile_export_summary.json").write_text(
                json.dumps({"selected": True, "recommended_method": "global_inverse_depth_linear"}),
                encoding="utf-8",
            )
            replay_dir = intrinsic_calibration_dir / "replay"
            replay_dir.mkdir()
            replay_dir.joinpath("profile_replay_summary.csv").write_text(
                "validation_range_rmse_corrected_m,target_abs_mean_error_corrected_m\n"
                "0.975,0.306\n",
                encoding="utf-8",
            )
            family_transfer_dir = intrinsic_calibration_dir / "intrinsic_family_transfer"
            family_transfer_dir.mkdir()
            family_transfer_dir.joinpath("active_profile_transfer_summary.json").write_text(
                json.dumps(
                    {
                        "target_scenarios": "intrinsic_scale_stress",
                        "transfer_replay_pass_count": 3,
                        "transfer_replay_count": 3,
                        "validation_range_rmse_corrected_max_m": 1.024,
                        "target_max_abs_error_max_m": 2.41,
                    }
                ),
                encoding="utf-8",
            )
            family_export_audit_dir = sweep / "intrinsic_scale_family_calibration_export_audit"
            family_export_audit_dir.mkdir()
            family_export_audit_dir.joinpath("family_calibration_profile_export_summary.json").write_text(
                json.dumps(
                    {
                        "target_scenarios": "intrinsic_scale_stress",
                        "heldout_replay_pass_count": 6,
                        "heldout_replay_count": 6,
                        "validation_range_rmse_corrected_max_m": 1.024,
                        "target_max_abs_error_max_m": 2.41,
                    }
                ),
                encoding="utf-8",
            )
            all_family_export_audit_dir = sweep / "all_family_calibration_export_audit"
            all_family_export_audit_dir.mkdir()
            all_family_export_audit_dir.joinpath("family_calibration_profile_export_summary.json").write_text(
                json.dumps(
                    {
                        "target_scenarios": "",
                        "heldout_replay_pass_count": 36,
                        "heldout_replay_count": 36,
                        "validation_range_rmse_corrected_max_m": 1.877,
                        "target_max_abs_error_max_m": 9.882,
                    }
                ),
                encoding="utf-8",
            )
            bbox_policy_audit_dir = sweep / "bbox_policy_family_calibration_export_audit"
            bbox_policy_audit_dir.mkdir()
            bbox_policy_audit_dir.joinpath("family_calibration_profile_export_summary.json").write_text(
                json.dumps({"recommended_source_policy": "target_max_stable"}),
                encoding="utf-8",
            )
            bbox_policy_audit_dir.joinpath("family_calibration_profile_policy_summary.csv").write_text(
                "source_policy,heldout_replay_pass_count,heldout_replay_count,validation_range_rmse_corrected_max_m,target_max_abs_error_max_m\n"
                "target_max_stable,6,6,1.553,9.681\n",
                encoding="utf-8",
            )
            all_policy_audit_dir = sweep / "all_policy_family_calibration_export_audit"
            all_policy_audit_dir.mkdir()
            all_policy_audit_dir.joinpath("family_calibration_profile_export_summary.json").write_text(
                json.dumps({"recommended_source_policy": "target_max_stable"}),
                encoding="utf-8",
            )
            all_policy_audit_dir.joinpath("family_calibration_profile_policy_summary.csv").write_text(
                "source_policy,heldout_replay_pass_count,heldout_replay_count,validation_range_rmse_corrected_max_m,target_max_abs_error_max_m\n"
                "target_max_stable,36,36,2.432,9.681\n",
                encoding="utf-8",
            )
            policy_catalog_dir = sweep / "policy_family_profile_catalog"
            policy_catalog_dir.mkdir()
            policy_catalog_dir.joinpath("policy_profile_catalog_summary.json").write_text(
                json.dumps(
                    {
                        "profile_count": 6,
                        "pass_profile_count": 6,
                        "pass_policy_profile_catalog": True,
                    }
                ),
                encoding="utf-8",
            )
            bbox_transfer_dir = policy_catalog_dir / "bbox_transfer_replay"
            bbox_transfer_dir.mkdir()
            bbox_transfer_dir.joinpath("active_profile_transfer_summary.json").write_text(
                json.dumps({"target_max_abs_error_max_m": 8.951}),
                encoding="utf-8",
            )
            runtime_policy_catalog_dir = sweep / "runtime_measurement_guard_audit_policy_catalog_transfer_risk"
            runtime_policy_catalog_dir.mkdir()
            runtime_policy_catalog_dir.joinpath("runtime_measurement_guard_summary.json").write_text(
                json.dumps(
                    {
                        "selection_policy": "transfer_risk_fallback",
                        "pass_count": 18,
                        "audit_count": 18,
                        "validation_range_rmse_corrected_max_m": 1.806,
                        "target_max_abs_error_max_m": 8.951,
                    }
                ),
                encoding="utf-8",
            )
            runtime_policy_catalog_robust_dir = sweep / "runtime_measurement_guard_audit_policy_catalog_robust_family"
            runtime_policy_catalog_robust_dir.mkdir()
            runtime_policy_catalog_robust_dir.joinpath("runtime_measurement_guard_summary.json").write_text(
                json.dumps(
                    {
                        "selection_policy": "robust_family_fallback",
                        "pass_count": 18,
                        "audit_count": 18,
                        "validation_range_rmse_corrected_max_m": 1.806,
                        "target_max_abs_error_max_m": 8.951,
                    }
                ),
                encoding="utf-8",
            )
            runtime_policy_catalog_robust_loso_dir = (
                sweep / "runtime_measurement_guard_audit_policy_catalog_robust_family_loso"
            )
            runtime_policy_catalog_robust_loso_dir.mkdir()
            runtime_policy_catalog_robust_loso_dir.joinpath("runtime_measurement_guard_summary.json").write_text(
                json.dumps(
                    {
                        "selection_policy": "robust_family_fallback",
                        "pass_count": 14,
                        "audit_count": 18,
                        "validation_only_pass_count": 17,
                        "validation_only_pass_all": False,
                        "validation_range_rmse_corrected_max_m": 2.375,
                        "target_max_abs_error_max_m": 7.377,
                    }
                ),
                encoding="utf-8",
            )
            calibration_required_dir = (
                sweep / "runtime_calibration_required_gap_summary_policy_catalog_robust_family_loso"
            )
            calibration_required_dir.mkdir()
            calibration_required_dir.joinpath("runtime_calibration_required_gap_summary.json").write_text(
                json.dumps(
                    {
                        "calibration_required_count": 3,
                        "calibration_required_validation_only_pass_count": 2,
                        "unsafe_correction_count": 1,
                        "calibration_required_target_within_limit_count": 3,
                    }
                ),
                encoding="utf-8",
            )
            calibration_required_block_dir = sweep / "operating_mode_export_intrinsic_scale_required_block_smoke"
            calibration_required_block_dir.mkdir()
            calibration_required_block_dir.joinpath("operating_mode_profile_export_summary.json").write_text(
                json.dumps(
                    {
                        "selected": False,
                        "calibration_required_blocked_export": True,
                        "calibration_required_block_reason": "intrinsic_scale_gap_detected_required_profile_unavailable",
                    }
                ),
                encoding="utf-8",
            )
            method_family_dir = sweep / "method_family_summary"
            method_family_dir.mkdir()
            method_family_dir.joinpath("method_family_summary.json").write_text(
                json.dumps(
                    {
                        "recommended_family_by_validation": "residual_disparity",
                        "oracle_recommended_family_by_validation": "affine_runtime",
                        "oracle_recommended_family_pass_target_max_limit_count": 20,
                        "oracle_recommended_family_run_count": 20,
                        "oracle_recommended_family_target_max_abs_error_max_m": 9.307,
                    }
                ),
                encoding="utf-8",
            )
            affine_selection_dir = sweep / "affine_family_profile_selection_audit"
            affine_selection_dir.mkdir()
            affine_selection_dir.joinpath("method_family_profile_selection_summary.json").write_text(
                json.dumps(
                    {
                        "family_name": "affine_runtime",
                        "run_count": 20,
                        "selected_pass_count": 20,
                        "selected_pass_all": True,
                        "selected_validation_target_max_abs_error_max_m": 9.307,
                    }
                ),
                encoding="utf-8",
            )
            affine_runtime_selector_dir = sweep / "affine_family_runtime_feature_selector_audit_robust_bbox"
            affine_runtime_selector_dir.mkdir()
            affine_runtime_selector_dir.joinpath("runtime_feature_selector_summary.json").write_text(
                json.dumps(
                    {
                        "selection_policy": "robust_bbox_fallback",
                        "audit_count": 20,
                        "pass_count": 20,
                        "validation_range_rmse_corrected_max_m": 2.648,
                        "target_max_abs_error_max_m": 9.423,
                    }
                ),
                encoding="utf-8",
            )
            selector_portfolio_dir = sweep / "runtime_selector_portfolio_policy_vs_affine"
            selector_portfolio_dir.mkdir()
            selector_portfolio_dir.joinpath("runtime_selector_portfolio_summary.json").write_text(
                json.dumps(
                    {
                        "objective": "target_max_then_rmse",
                        "audit_count": 18,
                        "pass_count": 18,
                        "validation_range_rmse_corrected_max_m": 1.758,
                        "target_max_abs_error_max_m": 8.951,
                        "selected_oracle_match_count": 11,
                        "selected_oracle_match_fraction": 0.611,
                        "rmse_regret_max_m": 0.337,
                        "oracle_separability_best_label": "affine_runtime",
                        "oracle_separability_best_feature": "feature_measured_disparity_mean_px",
                        "oracle_separability_best_accuracy": 0.778,
                        "precision_override_label": "affine_runtime",
                        "precision_override_pass_count": 18,
                        "precision_override_rmse_mean_m": 1.211,
                        "precision_override_target_max_abs_error_max_m": 8.951,
                    }
                ),
                encoding="utf-8",
            )

            summary = module.summarize_dashboard(sweep, root / "out")

            self.assertEqual(summary["current_runtime_recommendation"], "policy_catalog+robust_family_fallback")
            self.assertEqual(summary["runtime_pass_count"], 18)
            self.assertEqual(float(summary["runtime_target_max_abs_error_max_m"]), 8.951)
            dashboard_csv = (root / "out" / "rtk_stereo_experiment_dashboard.csv").read_text(encoding="utf-8")
            self.assertIn("active_profile_portfolio_audit", dashboard_csv)
            self.assertIn("target_guard_blend", dashboard_csv)
            self.assertIn("runtime_selector_portfolio_oracle_gap", dashboard_csv)
            self.assertIn("runtime_selector_portfolio_oracle_separability", dashboard_csv)
            self.assertIn("feature_measured_disparity_mean_px", dashboard_csv)
            self.assertIn("runtime_selector_portfolio_precision_override", dashboard_csv)
            self.assertIn("runtime_truth_free_selector_loso", dashboard_csv)
            self.assertIn("transfer_threshold_loso", dashboard_csv)
            self.assertIn("rmse<=1.8,target<=2.5", dashboard_csv)
            self.assertIn("active_export_intrinsic_scale_guard_smoke", dashboard_csv)
            self.assertIn("intrinsic_scale_product_ratio_guard_selects_scale_profile", dashboard_csv)
            self.assertIn("intrinsic_scale_calibration_export_smoke", dashboard_csv)
            self.assertIn("global_inverse_depth_linear", dashboard_csv)
            self.assertIn("intrinsic_scale_family_transfer", dashboard_csv)
            self.assertIn("intrinsic_scale_stress", dashboard_csv)
            self.assertIn("intrinsic_scale_family_calibration_export_audit", dashboard_csv)
            self.assertIn("all_family_calibration_export_audit", dashboard_csv)
            self.assertIn("all_scenarios", dashboard_csv)
            self.assertIn("bbox_policy_family_calibration_export_audit", dashboard_csv)
            self.assertIn("target_max_stable", dashboard_csv)
            self.assertIn("all_policy_family_calibration_export_audit", dashboard_csv)
            self.assertIn("policy_family_profile_catalog", dashboard_csv)
            self.assertIn("scenario_policy_catalog", dashboard_csv)
            self.assertIn("runtime_guard_policy_catalog_transfer_risk", dashboard_csv)
            self.assertIn("runtime_guard_policy_catalog_robust_family", dashboard_csv)
            self.assertIn("runtime_guard_policy_catalog_robust_family_loso", dashboard_csv)
            self.assertIn("runtime_guard_policy_catalog_robust_family_loso_validation_only", dashboard_csv)
            self.assertIn("runtime_calibration_required_gap_loso", dashboard_csv)
            self.assertIn("runtime_calibration_required_export_block_smoke", dashboard_csv)
            self.assertIn("intrinsic_scale_gap_detected_required_profile_unavailable", dashboard_csv)
            self.assertIn("method_family_potential", dashboard_csv)
            self.assertIn("affine_runtime", dashboard_csv)
            self.assertIn("affine_family_profile_selection", dashboard_csv)
            self.assertIn("affine_family_runtime_feature_selector", dashboard_csv)
            self.assertIn("robust_bbox_fallback", dashboard_csv)
            self.assertIn("runtime_selector_portfolio_policy_vs_affine", dashboard_csv)
            self.assertIn("target_max_then_rmse", dashboard_csv)
            self.assertIn("calibration_truth_selector_loso", dashboard_csv)
            self.assertIn("loso_coverage_gap", dashboard_csv)
            self.assertIn("intrinsic_scale_stress", dashboard_csv)
            self.assertIn("gap_feature_separability", dashboard_csv)
            self.assertIn("feature_raw_forward_mean_m", dashboard_csv)
            self.assertTrue((root / "out" / "rtk_stereo_experiment_dashboard.csv").is_file())
            self.assertTrue((root / "out" / "rtk_stereo_experiment_dashboard_summary.txt").is_file())
            self.assertTrue((root / "out" / "rtk_stereo_experiment_dashboard_pass_ratio.png").is_file())


if __name__ == "__main__":
    unittest.main()
