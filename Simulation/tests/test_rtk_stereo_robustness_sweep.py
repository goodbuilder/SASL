from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_sweep_module():
    spec = importlib.util.spec_from_file_location(
        "rtk_stereo_robustness_sweep_under_test",
        ROOT_DIR / "scripts" / "run_rtk_stereo_robustness_sweep.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load run_rtk_stereo_robustness_sweep.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RtkStereoRobustnessSweepTests(unittest.TestCase):
    def write_json(self, path: Path, payload: object) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def write_runtime_profile(self, path: Path, method: str) -> None:
        self.write_json(
            path,
            {
                "profile_id": f"profile-{method}",
                "method": method,
                "mode": "estimated_to_truth_sim3",
                "axis_contract": "internal_policy_vector=RDF(right,down,forward)",
                "runtime_inputs": ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_disparity_px"],
                "forbidden_runtime_inputs": ["calibration_truth", "px4_rtk_truth"],
                "scale": 1.0,
                "rotation_matrix_rdf": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "translation_rdf_m": [0.0, 0.0, 0.0],
            },
        )

    def selector_package_pass_summary(self) -> dict[str, object]:
        return {
            "status": "PASS",
            "selector_export_status": "PASS",
            "frontier_status": "PASS",
            "operator_launch_status": "PASS",
            "profile_count": 2,
            "deployment_label_count": 6,
            "operator_launch_check_count": 6,
            "operator_launch_pass_count": 6,
            "runtime_truth_used": False,
            "selector_json": "/tmp/runtime_label_selector_package/runtime_profile_label_selector.json",
            "selection_contract": "explicit deployment/scenario label only; no runtime truth",
        }

    def test_parse_scenario_list_selects_known_scenarios(self) -> None:
        module = load_sweep_module()

        scenarios = module.parse_scenario_list("nominal, disparity_noise_stress")

        self.assertEqual([scenario.name for scenario in scenarios], ["nominal", "disparity_noise_stress"])
        self.assertIn("--disparity-bias-px", scenarios[1].args)

    def test_default_scenarios_include_combined_but_not_sparse_diagnostic(self) -> None:
        module = load_sweep_module()

        self.assertIn("combined_sensor_stress", module.DEFAULT_SCENARIOS)
        self.assertIn("bbox_outlier_stress", module.DEFAULT_SCENARIOS)
        self.assertNotIn("sparse_target_window", module.DEFAULT_SCENARIOS)
        self.assertIn("--bbox-noise-px", module.SCENARIOS["combined_sensor_stress"].args)
        self.assertIn("--bbox-outlier-rate", module.SCENARIOS["bbox_outlier_stress"].args)

    def test_parse_scenario_list_rejects_unknown_scenario(self) -> None:
        module = load_sweep_module()

        with self.assertRaises(ValueError):
            module.parse_scenario_list("nominal,missing")

    def test_parse_args_supports_skipping_near_tie_transfer(self) -> None:
        module = load_sweep_module()

        with patch.object(sys, "argv", ["sweep", "--output-dir", "/tmp/out", "--skip-near-tie-transfer"]):
            args = module.parse_args()

        self.assertTrue(args.skip_near_tie_transfer)

    def test_export_runtime_label_selector_package_runs_selector_and_frontier(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            catalog_dir = output_dir / "scenario_profile_catalog"
            self.write_runtime_profile(catalog_dir / "nominal_runtime_profile.json", "sim3_nominal")
            self.write_runtime_profile(catalog_dir / "stress_runtime_profile.json", "sim3_stress")
            rows = [
                {
                    "scenario": "nominal",
                    "method": "sim3_nominal",
                    "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                },
                {
                    "scenario": "stress",
                    "method": "sim3_stress",
                    "catalog_profile_path": "scenario_profile_catalog/stress_runtime_profile.json",
                    "axis_contract": module.RDF_AXIS_CONTRACT,
                },
            ]
            scenario_catalog_summary = {
                "catalog_profile_count": 2,
                "profile_exists_count": 2,
                "cross_seed_pass_count": 2,
                "pass_scenario_profile_catalog": True,
                "rows": rows,
            }
            self.write_json(output_dir / "scenario_profile_catalog.json", scenario_catalog_summary)
            self.write_json(
                output_dir / "robustness_summary.json",
                {
                    "robustness_acceptance_pass": True,
                    "runtime_truth_used": False,
                    "rdf_axis_contract": module.RDF_AXIS_CONTRACT,
                    "truth_rdf_transform_contract": module.TRUTH_RDF_TRANSFORM_CONTRACT,
                },
            )
            (output_dir / "scenario_coordinate_contracts.csv").write_text(
                "scenario,all_loaded_contracts_forbid_direct_axis_alias\nnominal,True\nstress,True\n",
                encoding="utf-8",
            )

            summary = module.export_runtime_label_selector_package(
                output_dir=output_dir,
                scenario_catalog_summary=scenario_catalog_summary,
                baseline_scenario="nominal",
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["selector_export_status"], "PASS")
            self.assertEqual(summary["frontier_status"], "PASS")
            self.assertEqual(summary["profile_count"], 2)
            self.assertFalse(summary["runtime_truth_used"])
            self.assertTrue(Path(summary["selector_json"]).is_file())

    def test_custom_scenario_config_extends_scenario_registry(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "scenarios.json"
            path.write_text(
                json.dumps(
                    {
                        "scenarios": [
                            {
                                "name": "custom_disparity",
                                "purpose": "test custom disparity noise",
                                "args": ["--disparity-noise-px", 0.2, "--bbox-noise-px", "0.4"],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            custom = module.load_custom_scenarios(path)
            registry = module.scenario_registry(custom)
            scenarios = module.parse_scenario_list("nominal,custom_disparity", registry=registry)

        self.assertEqual([scenario.name for scenario in scenarios], ["nominal", "custom_disparity"])
        self.assertEqual(scenarios[1].args, ("--disparity-noise-px", "0.2", "--bbox-noise-px", "0.4"))

    def test_custom_scenario_config_accepts_mapping_shape(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "scenarios.json"
            path.write_text(
                json.dumps(
                    {
                        "pitch_sweep": {
                            "purpose": "test mapping shape",
                            "args": ["--pitch-error-deg", "-0.6"],
                        }
                    }
                ),
                encoding="utf-8",
            )

            custom = module.load_custom_scenarios(path)

        self.assertIn("pitch_sweep", custom)
        self.assertEqual(custom["pitch_sweep"].args, ("--pitch-error-deg", "-0.6"))

    def test_run_scenario_forwards_common_and_scenario_args(self) -> None:
        module = load_sweep_module()
        args = SimpleNamespace(
            seeds="1,2",
            sample_count=120,
            train_ratio=0.8,
            target_range_m=1852.0,
            target_window_m=30.0,
            synthetic_target_oversample_count=6,
            disparity_outlier_rate=0.0,
            disparity_outlier_px=0.0,
            bbox_outlier_rate=0.0,
            bbox_outlier_px=0.0,
            min_validation_improvement_m=0.01,
        )
        scenario = module.SCENARIOS["intrinsic_scale_stress"]

        with patch.object(module.subprocess, "run") as run_mock:
            run_mock.return_value.returncode = 0
            exit_code = module.run_scenario(args, scenario, Path("/tmp/scenario"))

        command = run_mock.call_args.args[0]
        self.assertEqual(exit_code, 0)
        self.assertIn("--synthetic-target-oversample-count", command)
        self.assertEqual(command[command.index("--synthetic-target-oversample-count") + 1], "6")
        self.assertIn("--bbox-outlier-rate", command)
        self.assertEqual(command[command.index("--bbox-outlier-rate") + 1], "0.0")
        self.assertIn("--baseline-scale-error", command)
        self.assertEqual(command[command.index("--baseline-scale-error") + 1], "0.012")

    def test_method_contract_fields_classify_runtime_profile_family(self) -> None:
        module = load_sweep_module()

        smoothed = module.method_contract_fields("sim3_smoothed_binned_residual_disparity_bins11")
        kernel_disparity = module.method_contract_fields("sim3_kernel_smoothed_binned_residual_disparity_sigma040")
        capped = module.method_contract_fields("sim3_capped_binned_residual_disparity")
        raw_forward = module.method_contract_fields("sim3_binned_residual_raw_forward")
        piecewise = module.method_contract_fields("sim3_piecewise_raw_forward_bins3")
        piecewise_blended = module.method_contract_fields("sim3_blended_piecewise_raw_forward_bins5_alpha050")
        blended = module.method_contract_fields("sim3_blended_binned_residual_disparity_raw_forward")
        gated = module.method_contract_fields("sim3_agreement_gated_binned_residual_disparity_raw_forward")
        affine = module.method_contract_fields("affine_binned_residual_disparity")
        disparity = module.method_contract_fields("global_disparity_linear")
        inverse_depth = module.method_contract_fields("global_inverse_depth_linear")
        inverse_depth_sim3 = module.method_contract_fields("target_weighted_inverse_depth_linear_then_sim3")
        target_inverse_depth = module.method_contract_fields("target_weighted_global_inverse_depth_linear")
        sigma_inverse_depth = module.method_contract_fields("target_weighted_global_inverse_depth_linear_sigma060")
        shrunk_inverse_depth = module.method_contract_fields("target_shrunk_global_inverse_depth_linear_shrink050")

        self.assertEqual(smoothed["registration_model"], "sim3_estimated_to_truth")
        self.assertEqual(smoothed["residual_model"], "forward_disparity_lut")
        self.assertIn("lut_shrinkage_neighbor_smoothing", smoothed["regularization_model"])
        self.assertEqual(kernel_disparity["residual_model"], "forward_disparity_lut")
        self.assertIn("kernel_smoothing", kernel_disparity["regularization_model"])
        self.assertIn("lut_robust_winsorized_cap", capped["regularization_model"])
        self.assertEqual(capped["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertEqual(raw_forward["residual_model"], "forward_raw_rdf_forward_lut")
        self.assertEqual(raw_forward["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertEqual(piecewise["registration_model"], "piecewise_raw_forward_sim3_estimated_to_truth")
        self.assertEqual(piecewise["residual_model"], "none")
        self.assertIn("raw_forward_piecewise_registration", piecewise["regularization_model"])
        self.assertEqual(piecewise["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertEqual(
            piecewise_blended["registration_model"],
            "piecewise_raw_forward_sim3_estimated_to_truth",
        )
        self.assertEqual(piecewise_blended["residual_model"], "none")
        self.assertIn("global_piecewise_registration_blend", piecewise_blended["regularization_model"])
        self.assertIn("piecewise_blend_alpha_grid", piecewise_blended["regularization_model"])
        self.assertEqual(blended["residual_model"], "forward_disparity_raw_forward_blend_lut")
        self.assertIn("dual_key_residual_blend", blended["regularization_model"])
        self.assertEqual(blended["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertEqual(gated["residual_model"], "forward_disparity_raw_forward_agreement_gated_lut")
        self.assertIn("dual_key_residual_agreement_gate", gated["regularization_model"])
        self.assertEqual(gated["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertEqual(smoothed["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertEqual(affine["runtime_profile_family"], "estimated_to_truth_affine_rdf")
        self.assertEqual(disparity["runtime_profile_family"], "estimated_to_truth_disparity_poly")
        self.assertEqual(inverse_depth["registration_model"], "raw_rdf_inverse_depth_ray_scale")
        self.assertEqual(inverse_depth["residual_model"], "corrected_inverse_depth_linear")
        self.assertEqual(inverse_depth["runtime_profile_family"], "estimated_to_truth_disparity_poly")
        self.assertEqual(
            inverse_depth_sim3["registration_model"],
            "inverse_depth_ray_scale_then_sim3_estimated_to_truth",
        )
        self.assertIn("target_range_weighting", inverse_depth_sim3["regularization_model"])
        self.assertEqual(inverse_depth_sim3["runtime_profile_family"], "estimated_to_truth_sim3")
        self.assertEqual(target_inverse_depth["runtime_profile_family"], "estimated_to_truth_disparity_poly")
        self.assertIn("target_range_weighting", target_inverse_depth["regularization_model"])
        self.assertEqual(sigma_inverse_depth["runtime_profile_family"], "estimated_to_truth_disparity_poly")
        self.assertIn("target_range_weighting", sigma_inverse_depth["regularization_model"])
        self.assertEqual(shrunk_inverse_depth["runtime_profile_family"], "estimated_to_truth_disparity_poly")
        self.assertIn("target_weight_shrinkage", shrunk_inverse_depth["regularization_model"])

    def test_aggregate_robust_methods_counts_cross_scenario_passes(self) -> None:
        module = load_sweep_module()
        rows = [
            {
                "scenario": "nominal",
                "method": "simple",
                "strict_pass": "True",
                "performance_pass": "True",
                "run_coverage_ok": "True",
                "target_error_within_tolerance": "True",
                "method_validation_range_rmse_mean_m": "1.0",
                "method_target_abs_mean_error_mean_m": "0.4",
                "method_target_abs_mean_error_max_m": "0.5",
                "axis_rmse_regression_sum_m": "0.0",
                "model_complexity_mean": "3",
            },
            {
                "scenario": "stress",
                "method": "simple",
                "strict_pass": "True",
                "performance_pass": "True",
                "run_coverage_ok": "True",
                "target_error_within_tolerance": "True",
                "method_validation_range_rmse_mean_m": "1.2",
                "method_target_abs_mean_error_mean_m": "0.6",
                "method_target_abs_mean_error_max_m": "0.8",
                "axis_rmse_regression_sum_m": "0.1",
                "model_complexity_mean": "3",
            },
            {
                "scenario": "nominal",
                "method": "partial",
                "strict_pass": "True",
                "performance_pass": "True",
                "run_coverage_ok": "True",
                "target_error_within_tolerance": "True",
                "method_validation_range_rmse_mean_m": "0.8",
                "method_target_abs_mean_error_mean_m": "0.2",
                "method_target_abs_mean_error_max_m": "0.3",
                "axis_rmse_regression_sum_m": "0.0",
                "model_complexity_mean": "8",
            },
        ]

        summary = module.aggregate_robust_methods(rows, scenario_count=2)
        by_method = {row["method"]: row for row in summary}

        self.assertTrue(by_method["simple"]["strict_pass_all_scenarios"])
        self.assertFalse(by_method["partial"]["strict_pass_all_scenarios"])
        self.assertEqual(by_method["simple"]["strict_pass_count"], 2)
        self.assertAlmostEqual(by_method["simple"]["validation_range_rmse_mean_m"], 1.1)
        self.assertEqual(by_method["simple"]["registration_model"], "other_estimated_to_truth")
        self.assertIn("runtime_profile_family", by_method["simple"])

    def test_choose_robust_recommendation_prefers_all_scenario_pass(self) -> None:
        module = load_sweep_module()
        rows = [
            {
                "method": "strict_all",
                "strict_pass_all_scenarios": True,
                "performance_pass_all_scenarios": True,
                "strict_pass_count": 2,
                "performance_pass_count": 2,
                "expected_scenario_count": 2,
                "validation_range_rmse_mean_m": 1.2,
                "validation_range_rmse_max_m": 1.3,
                "target_abs_mean_error_mean_m": 0.6,
                "target_abs_mean_error_max_m": 0.8,
                "axis_rmse_regression_sum_mean_m": 0.1,
                "model_complexity_mean": 4,
                "robust_score": 1.5,
            },
            {
                "method": "strict_all_axis_clean",
                "strict_pass_all_scenarios": True,
                "performance_pass_all_scenarios": True,
                "strict_pass_count": 2,
                "performance_pass_count": 2,
                "expected_scenario_count": 2,
                "validation_range_rmse_mean_m": 1.3,
                "validation_range_rmse_max_m": 1.4,
                "target_abs_mean_error_mean_m": 0.7,
                "target_abs_mean_error_max_m": 0.9,
                "axis_rmse_regression_sum_mean_m": 0.0,
                "model_complexity_mean": 4,
                "robust_score": 1.6,
            },
            {
                "method": "partial_low_rmse",
                "strict_pass_all_scenarios": False,
                "performance_pass_all_scenarios": False,
                "strict_pass_count": 1,
                "performance_pass_count": 1,
                "expected_scenario_count": 2,
                "validation_range_rmse_mean_m": 0.3,
                "validation_range_rmse_max_m": 0.4,
                "target_abs_mean_error_mean_m": 0.1,
                "target_abs_mean_error_max_m": 0.2,
                "axis_rmse_regression_sum_mean_m": 0.0,
                "model_complexity_mean": 10,
                "robust_score": 0.4,
            },
        ]

        recommendation = module.choose_robust_recommendation(rows)

        self.assertEqual(recommendation["recommendation_mode"], "strict_all_scenarios")
        self.assertEqual(recommendation["method"], "strict_all_axis_clean")
        self.assertTrue(recommendation["rdf_axis_nonregression_pass"])

    def test_choose_robust_recommendation_rejects_axis_regression_when_clean_candidate_exists(self) -> None:
        module = load_sweep_module()
        rows = [
            {
                "method": "strict_axis_regresses",
                "strict_pass_all_scenarios": True,
                "performance_pass_all_scenarios": True,
                "strict_pass_count": 2,
                "performance_pass_count": 2,
                "expected_scenario_count": 2,
                "validation_range_rmse_mean_m": 0.3,
                "validation_range_rmse_max_m": 0.4,
                "target_abs_mean_error_mean_m": 0.1,
                "target_abs_mean_error_max_m": 0.2,
                "axis_rmse_regression_sum_mean_m": 0.2,
                "model_complexity_mean": 3,
                "robust_score": 0.3,
            },
            {
                "method": "partial_axis_clean",
                "strict_pass_all_scenarios": False,
                "performance_pass_all_scenarios": False,
                "strict_pass_count": 1,
                "performance_pass_count": 1,
                "expected_scenario_count": 2,
                "validation_range_rmse_mean_m": 0.8,
                "validation_range_rmse_max_m": 0.9,
                "target_abs_mean_error_mean_m": 0.4,
                "target_abs_mean_error_max_m": 0.5,
                "axis_rmse_regression_sum_mean_m": 0.0,
                "model_complexity_mean": 4,
                "robust_score": 0.8,
            },
        ]

        recommendation = module.choose_robust_recommendation(rows)

        self.assertEqual(recommendation["recommendation_mode"], "best_partial_axis_clean_robustness")
        self.assertEqual(recommendation["method"], "partial_axis_clean")
        self.assertTrue(recommendation["rdf_axis_nonregression_pass"])

    def test_choose_robust_recommendation_reports_empty_input(self) -> None:
        module = load_sweep_module()

        recommendation = module.choose_robust_recommendation([])

        self.assertEqual(recommendation["recommendation_mode"], "none")

    def test_robustness_acceptance_matrix_records_deployment_evidence(self) -> None:
        module = load_sweep_module()

        matrix = module.build_robustness_acceptance_matrix(
            scenario_count=2,
            run_rows=[
                {"scenario": "nominal", "pass_run": True},
                {"scenario": "stress", "pass_run": True},
            ],
            recommendation_rows=[
                {"scenario": "nominal", "method": "sim3"},
                {"scenario": "stress", "method": "sim3"},
            ],
            robust_summary_rows=[
                {
                    "method": "sim3",
                    "strict_pass_all_scenarios": True,
                    "performance_pass_all_scenarios": True,
                }
            ],
            robust_recommendation={
                "method": "sim3",
                "recommendation_mode": "strict_all_scenarios",
                "strict_pass_count": 2,
                "performance_pass_count": 2,
                "axis_rmse_regression_sum_mean_m": 0.0,
            },
            policy_transfer_summary_rows=[
                {"policy": "balanced", "pass_all_transfer_replays": True},
            ],
            scenario_catalog_summary={
                "pass_scenario_profile_catalog": True,
                "catalog_profile_count": 2,
                "profile_exists_count": 2,
                "cross_seed_pass_count": 2,
            },
            scenario_catalog_promotion_summary={
                "pass_catalog_promotion_checks": True,
                "catalog_promotion_check_count": 2,
                "catalog_promotion_pass_count": 2,
            },
            catalog_transfer_source_summary_rows=[
                {"source_scenario": "nominal", "pass_all_transfer_replays": True},
            ],
            deployment_strategy={
                "deployment_strategy": "single_runtime_profile",
                "single_profile_pass": True,
                "scenario_catalog_ready": True,
                "runtime_profile_contract": "runtime uses raw RDF/disparity only",
            },
            coordinate_contract_summary={
                "scenario_count": 2,
                "summary_loaded_scenario_count": 2,
                "scenario_direct_axis_alias_forbidden_count": 2,
                "loaded_contract_count": 4,
                "direct_axis_alias_forbidden_count": 4,
                "all_scenarios_forbid_direct_axis_alias": True,
                "truth_rdf_transform_contract": module.TRUTH_RDF_TRANSFORM_CONTRACT,
            },
            runtime_label_selector_package=self.selector_package_pass_summary(),
        )

        self.assertEqual(sorted(matrix.keys()), sorted(module.ROBUSTNESS_ACCEPTANCE_ITEMS))
        self.assertTrue(all(item["status"] == "PASS" for item in matrix.values()))
        self.assertEqual(matrix["coordinate_contract"]["path"], "scenario_coordinate_contracts.csv")
        self.assertEqual(
            matrix["coordinate_contract"]["evidence"]["truth_rdf_transform_contract"],
            module.TRUTH_RDF_TRANSFORM_CONTRACT,
        )
        self.assertEqual(matrix["robust_method_summary"]["evidence"]["strict_all_scenario_method_count"], 1)
        self.assertTrue(matrix["robust_recommendation"]["evidence"]["rdf_axis_nonregression_pass"])
        self.assertEqual(matrix["deployment_strategy"]["evidence"]["deployment_strategy"], "single_runtime_profile")
        self.assertEqual(matrix["runtime_label_selector_package"]["status"], "PASS")
        self.assertFalse(matrix["runtime_label_selector_package"]["evidence"]["runtime_truth_used"])

    def test_robustness_acceptance_matrix_exposes_incomplete_strategy(self) -> None:
        module = load_sweep_module()

        matrix = module.build_robustness_acceptance_matrix(
            scenario_count=2,
            run_rows=[{"scenario": "nominal", "pass_run": True}],
            recommendation_rows=[{"scenario": "nominal", "method": ""}],
            robust_summary_rows=[],
            robust_recommendation={"method": "", "recommendation_mode": "none"},
            policy_transfer_summary_rows=[],
            scenario_catalog_summary={"pass_scenario_profile_catalog": False},
            scenario_catalog_promotion_summary={"pass_catalog_promotion_checks": False},
            catalog_transfer_source_summary_rows=[],
            deployment_strategy={
                "deployment_strategy": "no_validated_runtime_strategy",
                "single_profile_pass": False,
                "scenario_catalog_ready": False,
            },
            coordinate_contract_summary={
                "scenario_count": 2,
                "summary_loaded_scenario_count": 1,
                "scenario_direct_axis_alias_forbidden_count": 0,
                "loaded_contract_count": 1,
                "direct_axis_alias_forbidden_count": 0,
                "all_scenarios_forbid_direct_axis_alias": False,
                "truth_rdf_transform_contract": module.TRUTH_RDF_TRANSFORM_CONTRACT,
            },
            runtime_label_selector_package={
                "status": "FAIL",
                "selector_export_status": "PASS",
                "frontier_status": "FAIL",
            "profile_count": 0,
            "deployment_label_count": 0,
            "operator_launch_status": "FAIL",
            "operator_launch_check_count": 1,
            "operator_launch_pass_count": 0,
            "runtime_truth_used": True,
            "selector_json": "",
        },
        )

        self.assertEqual(matrix["scenario_run_coverage"]["status"], "FAIL")
        self.assertEqual(matrix["coordinate_contract"]["status"], "FAIL")
        self.assertEqual(matrix["scenario_recommendations"]["status"], "FAIL")
        self.assertEqual(matrix["robust_recommendation"]["status"], "FAIL")
        self.assertEqual(matrix["deployment_strategy"]["status"], "FAIL")
        self.assertEqual(matrix["runtime_label_selector_package"]["status"], "FAIL")

    def test_robustness_acceptance_allows_catalog_when_single_policy_transfer_fails(self) -> None:
        module = load_sweep_module()

        matrix = module.build_robustness_acceptance_matrix(
            scenario_count=2,
            run_rows=[
                {"scenario": "nominal", "pass_run": True},
                {"scenario": "stress", "pass_run": True},
            ],
            recommendation_rows=[
                {"scenario": "nominal", "method": "sim3"},
                {"scenario": "stress", "method": "inverse_depth"},
            ],
            robust_summary_rows=[
                {
                    "method": "sim3",
                    "strict_pass_all_scenarios": True,
                    "performance_pass_all_scenarios": True,
                }
            ],
            robust_recommendation={
                "method": "sim3",
                "recommendation_mode": "strict_all_scenarios",
                "strict_pass_count": 2,
                "performance_pass_count": 2,
                "axis_rmse_regression_sum_mean_m": 0.0,
            },
            policy_transfer_summary_rows=[
                {"policy": "balanced", "pass_all_transfer_replays": False},
            ],
            scenario_catalog_summary={
                "pass_scenario_profile_catalog": True,
                "catalog_profile_count": 2,
                "profile_exists_count": 2,
                "cross_seed_pass_count": 2,
            },
            scenario_catalog_promotion_summary={
                "pass_catalog_promotion_checks": True,
                "catalog_promotion_check_count": 2,
                "catalog_promotion_pass_count": 2,
            },
            catalog_transfer_source_summary_rows=[],
            deployment_strategy={
                "deployment_strategy": "scenario_profile_catalog",
                "single_profile_pass": False,
                "scenario_catalog_ready": True,
            },
            coordinate_contract_summary={
                "scenario_count": 2,
                "summary_loaded_scenario_count": 2,
                "scenario_direct_axis_alias_forbidden_count": 2,
                "loaded_contract_count": 2,
                "direct_axis_alias_forbidden_count": 2,
                "all_scenarios_forbid_direct_axis_alias": True,
                "truth_rdf_transform_contract": module.TRUTH_RDF_TRANSFORM_CONTRACT,
            },
            runtime_label_selector_package=self.selector_package_pass_summary(),
        )

        self.assertEqual(matrix["robust_policy_transfer"]["status"], "PASS")
        self.assertTrue(matrix["robust_policy_transfer"]["evidence"]["scenario_catalog_ready"])
        self.assertIn("scenario-specific catalog", matrix["robust_policy_transfer"]["evidence"]["interpretation"])

    def test_robust_coordinate_contract_summary_requires_all_scenarios_to_forbid_direct_alias(self) -> None:
        module = load_sweep_module()

        pass_summary = module.robustness_coordinate_contract_summary(
            [
                {
                    "summary_loaded": True,
                    "loaded_contract_count": 2,
                    "direct_axis_alias_forbidden_count": 2,
                    "all_loaded_contracts_forbid_direct_axis_alias": True,
                },
                {
                    "summary_loaded": True,
                    "loaded_contract_count": 2,
                    "direct_axis_alias_forbidden_count": 2,
                    "all_loaded_contracts_forbid_direct_axis_alias": True,
                },
            ],
            scenario_count=2,
        )
        fail_summary = module.robustness_coordinate_contract_summary(
            [
                {
                    "summary_loaded": True,
                    "loaded_contract_count": 2,
                    "direct_axis_alias_forbidden_count": 2,
                    "all_loaded_contracts_forbid_direct_axis_alias": True,
                },
                {
                    "summary_loaded": True,
                    "loaded_contract_count": 2,
                    "direct_axis_alias_forbidden_count": 1,
                    "all_loaded_contracts_forbid_direct_axis_alias": False,
                },
            ],
            scenario_count=2,
        )

        self.assertTrue(pass_summary["all_scenarios_forbid_direct_axis_alias"])
        self.assertFalse(fail_summary["all_scenarios_forbid_direct_axis_alias"])
        self.assertEqual(pass_summary["truth_rdf_transform_contract"], module.TRUTH_RDF_TRANSFORM_CONTRACT)

    def test_robust_policy_recommendations_expose_tradeoff_policies(self) -> None:
        module = load_sweep_module()
        rows = [
            {
                "method": "fast_low_rmse",
                "strict_pass_count": 2,
                "performance_pass_count": 2,
                "expected_scenario_count": 2,
                "strict_pass_all_scenarios": True,
                "performance_pass_all_scenarios": True,
                "validation_range_rmse_mean_m": 0.8,
                "validation_range_rmse_max_m": 1.0,
                "target_abs_mean_error_mean_m": 1.8,
                "target_abs_mean_error_max_m": 3.0,
                "axis_rmse_regression_sum_mean_m": 0.0,
                "model_complexity_mean": 15,
            },
            {
                "method": "target_safe",
                "strict_pass_count": 2,
                "performance_pass_count": 2,
                "expected_scenario_count": 2,
                "strict_pass_all_scenarios": True,
                "performance_pass_all_scenarios": True,
                "validation_range_rmse_mean_m": 1.2,
                "validation_range_rmse_max_m": 1.5,
                "target_abs_mean_error_mean_m": 0.5,
                "target_abs_mean_error_max_m": 0.9,
                "axis_rmse_regression_sum_mean_m": 0.0,
                "model_complexity_mean": 9,
            },
            {
                "method": "simple",
                "strict_pass_count": 2,
                "performance_pass_count": 2,
                "expected_scenario_count": 2,
                "strict_pass_all_scenarios": True,
                "performance_pass_all_scenarios": True,
                "validation_range_rmse_mean_m": 1.4,
                "validation_range_rmse_max_m": 1.6,
                "target_abs_mean_error_mean_m": 0.8,
                "target_abs_mean_error_max_m": 1.2,
                "axis_rmse_regression_sum_mean_m": 0.0,
                "model_complexity_mean": 7,
            },
        ]

        policies = module.robust_policy_recommendation_rows(rows)
        by_policy = {row["policy"]: row for row in policies}

        self.assertEqual(by_policy["validation_best"]["method"], "fast_low_rmse")
        self.assertEqual(by_policy["target_guard"]["method"], "target_safe")
        self.assertEqual(by_policy["simple_stable"]["method"], "simple")
        self.assertEqual(by_policy["balanced"]["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_aggregate_robust_method_families_groups_related_methods(self) -> None:
        module = load_sweep_module()
        rows = [
            {
                "method": "sim3_binned_residual_disparity",
                **module.method_contract_fields("sim3_binned_residual_disparity"),
                "strict_pass_all_scenarios": True,
                "performance_pass_all_scenarios": True,
                "robust_score": 1.0,
                "validation_range_rmse_mean_m": 0.8,
                "target_abs_mean_error_max_m": 1.2,
            },
            {
                "method": "sim3_binned_residual_disparity_bins9",
                **module.method_contract_fields("sim3_binned_residual_disparity_bins9"),
                "strict_pass_all_scenarios": True,
                "performance_pass_all_scenarios": True,
                "robust_score": 0.9,
                "validation_range_rmse_mean_m": 0.7,
                "target_abs_mean_error_max_m": 1.4,
            },
            {
                "method": "affine_binned_residual_disparity",
                **module.method_contract_fields("affine_binned_residual_disparity"),
                "strict_pass_all_scenarios": False,
                "performance_pass_all_scenarios": True,
                "robust_score": 1.1,
                "validation_range_rmse_mean_m": 0.9,
                "target_abs_mean_error_max_m": 1.0,
            },
        ]

        families = module.aggregate_robust_method_families(rows)
        by_key = {
            (row["registration_model"], row["residual_model"], row["regularization_model"]): row
            for row in families
        }

        raw_lut = by_key[("sim3_estimated_to_truth", "forward_disparity_lut", "none")]
        grid_lut = by_key[("sim3_estimated_to_truth", "forward_disparity_lut", "lut_bin_count_grid")]
        affine_lut = by_key[("affine_rdf_estimated_to_truth", "forward_disparity_lut", "none")]
        self.assertEqual(raw_lut["best_method"], "sim3_binned_residual_disparity")
        self.assertEqual(grid_lut["best_method"], "sim3_binned_residual_disparity_bins9")
        self.assertEqual(affine_lut["runtime_profile_family"], "estimated_to_truth_affine_rdf")
        self.assertEqual(raw_lut["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_policy_profile_selection_copies_closest_runtime_profile(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            scenario = module.Scenario("nominal", "test", ())
            per_seed = output_dir / "nominal" / "per_seed_method_comparison.csv"
            per_seed.parent.mkdir(parents=True)
            per_seed.write_text(
                "seed,method,validation_range_rmse_m,validation_target_abs_mean_error_m,"
                "validation_position_rmse_m,validation_right_rmse_m,validation_down_rmse_m,"
                "validation_forward_rmse_m,model_complexity\n"
                "1,sim3,1.0,0.4,1.1,0.1,0.2,1.0,7\n"
                "2,sim3,3.0,2.0,3.1,0.3,0.4,3.0,7\n",
                encoding="utf-8",
            )
            profile = output_dir / "nominal" / "seed_1" / "runtime_profiles" / "sim3.json"
            profile.parent.mkdir(parents=True)
            profile.write_text('{"mode":"estimated_to_truth_sim3"}\n', encoding="utf-8")
            profile2 = output_dir / "nominal" / "seed_2" / "runtime_profiles" / "sim3.json"
            profile2.parent.mkdir(parents=True)
            profile2.write_text('{"mode":"estimated_to_truth_sim3"}\n', encoding="utf-8")
            policy_rows = [
                {
                    "policy": "balanced",
                    "method": "sim3",
                    "validation_range_rmse_mean_m": 1.1,
                    "target_abs_mean_error_mean_m": 0.5,
                }
            ]

            candidates = module.collect_policy_profile_candidates(
                output_dir=output_dir,
                scenarios=[scenario],
                policy_rows=policy_rows,
            )
            selected = module.choose_policy_representative_profiles(
                output_dir=output_dir,
                policy_rows=policy_rows,
                profile_candidates=candidates,
            )

            self.assertEqual(len(candidates), 2)
            self.assertTrue(selected[0]["selected"])
            self.assertEqual(selected[0]["source_seed"], 1)
            self.assertTrue((output_dir / selected[0]["representative_profile_path"]).is_file())
            self.assertEqual(selected[0]["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_policy_profile_transfer_replay_aggregates_passes(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            scenario = module.Scenario("nominal", "test", ())
            seed_dir = output_dir / "nominal" / "seed_1"
            seed_dir.mkdir(parents=True)
            (seed_dir / "calibration_samples.csv").write_text("split\nvalidation\n", encoding="utf-8")
            (output_dir / "nominal" / "seed_runs.csv").write_text(
                "seed,run_dir,exit_code,pass_run\n"
                f"1,{seed_dir},0,True\n",
                encoding="utf-8",
            )
            profile = output_dir / "robust_policy_profiles" / "balanced_runtime_profile.json"
            profile.parent.mkdir(parents=True)
            profile.write_text("{}", encoding="utf-8")

            def fake_run(command, cwd):
                replay_dir = Path(command[command.index("--output-dir") + 1])
                replay_dir.mkdir(parents=True)
                (replay_dir / "profile_replay_summary.csv").write_text(
                    "validation_range_rmse_raw_m,validation_range_rmse_corrected_m,"
                    "target_abs_mean_error_corrected_m,target_window_count\n"
                    "2.0,1.0,0.5,1\n",
                    encoding="utf-8",
                )
                return SimpleNamespace(returncode=0)

            with patch.object(module.subprocess, "run", side_effect=fake_run):
                rows, summary = module.run_policy_profile_transfer_replay(
                    output_dir=output_dir,
                    scenarios=[scenario],
                    representative_profiles=[
                        {
                            "policy": "balanced",
                            "method": "sim3",
                            "selected": True,
                            "source_scenario": "nominal",
                            "source_seed": 1,
                            "representative_profile_path": "robust_policy_profiles/balanced_runtime_profile.json",
                        }
                    ],
                    target_range_m=1852.0,
                    target_window_m=30.0,
                )

            self.assertEqual(len(rows), 1)
            self.assertTrue(rows[0]["pass_replay"])
            self.assertEqual(summary[0]["transfer_replay_pass_count"], 1)
            self.assertTrue(summary[0]["pass_all_transfer_replays"])

    def test_transfer_failure_diagnostics_explain_universal_profile_failures(self) -> None:
        module = load_sweep_module()

        rows, summary = module.transfer_failure_diagnostic_rows(
            [
                {
                    "policy": "balanced",
                    "method": "sim3_binned_residual_disparity",
                    "source_scenario": "bbox_outlier_stress",
                    "target_scenario": "intrinsic_scale_stress",
                    "pass_replay": False,
                    "replay_returncode": 0,
                    "validation_range_rmse_raw_m": 1.8,
                    "validation_range_rmse_corrected_m": 2.5,
                    "validation_range_rmse_improvement_m": -0.7,
                    "target_window_count": 3,
                    "target_abs_mean_error_corrected_m": 4.0,
                },
                {
                    "policy": "balanced",
                    "method": "sim3_binned_residual_disparity",
                    "source_scenario": "bbox_outlier_stress",
                    "target_scenario": "intrinsic_scale_stress",
                    "pass_replay": True,
                    "replay_returncode": 0,
                    "validation_range_rmse_raw_m": 2.0,
                    "validation_range_rmse_corrected_m": 1.0,
                    "validation_range_rmse_improvement_m": 1.0,
                    "target_window_count": 3,
                    "target_abs_mean_error_corrected_m": 0.5,
                },
                {
                    "policy": "balanced",
                    "method": "sim3_binned_residual_disparity",
                    "source_scenario": "bbox_outlier_stress",
                    "target_scenario": "combined_sensor_stress",
                    "pass_replay": False,
                    "replay_returncode": 0,
                    "validation_range_rmse_raw_m": 3.0,
                    "validation_range_rmse_corrected_m": 2.0,
                    "validation_range_rmse_improvement_m": 1.0,
                    "target_window_count": 2,
                    "target_abs_mean_error_corrected_m": 12.0,
                },
            ]
        )

        by_scenario = {row["target_scenario"]: row for row in rows}
        self.assertEqual(by_scenario["intrinsic_scale_stress"]["fail_count"], 1)
        self.assertIn("no_validation_rmse_improvement", by_scenario["intrinsic_scale_stress"]["failure_reasons"])
        self.assertIn("target_error_over_10m", by_scenario["combined_sensor_stress"]["failure_reasons"])
        self.assertEqual(by_scenario["intrinsic_scale_stress"]["recommended_action"], "use_scenario_catalog_profile")
        self.assertEqual(summary["failed_policy_scenario_count"], 2)
        self.assertFalse(summary["pass_all_policy_scenarios"])
        self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_deployment_strategy_prefers_single_profile_when_transfer_passes(self) -> None:
        module = load_sweep_module()

        recommendation = module.deployment_strategy_recommendation(
            policy_transfer_summary_rows=[
                {
                    "policy": "balanced",
                    "method": "sim3",
                    "transfer_replay_count": 4,
                    "transfer_replay_pass_count": 4,
                    "pass_all_transfer_replays": True,
                    "validation_range_rmse_corrected_mean_m": 1.0,
                    "target_abs_mean_error_corrected_max_m": 0.5,
                }
            ],
            scenario_catalog_summary={"pass_scenario_profile_catalog": True, "catalog_profile_count": 2},
            scenario_catalog_promotion_summary={"pass_catalog_promotion_checks": True, "catalog_promotion_pass_count": 2},
        )

        self.assertEqual(recommendation["deployment_strategy"], "single_runtime_profile")
        self.assertEqual(recommendation["selected_policy"], "balanced")
        self.assertTrue(recommendation["single_profile_pass"])
        self.assertEqual(recommendation["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_deployment_strategy_prefers_better_near_tie_transfer_profile(self) -> None:
        module = load_sweep_module()

        recommendation = module.deployment_strategy_recommendation(
            policy_transfer_summary_rows=[
                {
                    "policy": "balanced",
                    "method": "target_shrunk_global_inverse_depth_linear_shrink050",
                    "transfer_replay_count": 4,
                    "transfer_replay_pass_count": 4,
                    "pass_all_transfer_replays": True,
                    "validation_range_rmse_corrected_mean_m": 0.8,
                    "target_abs_mean_error_corrected_max_m": 0.6,
                }
            ],
            scenario_catalog_summary={"pass_scenario_profile_catalog": True, "catalog_profile_count": 2},
            scenario_catalog_promotion_summary={"pass_catalog_promotion_checks": True, "catalog_promotion_pass_count": 2},
            best_near_tie_profile={
                "selected": True,
                "profile_id": "stress_rank004_seed1_candidate",
                "method": "target_shrunk_huber_global_inverse_depth_linear_shrink050",
                "promoted_profile_path": "scenario_near_tie_best_runtime_profile.json",
                "transfer_replay_count": 4,
                "transfer_replay_pass_count": 4,
                "validation_range_rmse_corrected_mean_m": 0.7,
                "target_abs_mean_error_corrected_max_m": 0.5,
            },
            best_near_tie_promotion_check={"pass_promotion_check": True},
        )

        self.assertEqual(recommendation["deployment_strategy"], "single_near_tie_runtime_profile")
        self.assertTrue(recommendation["near_tie_profile_pass"])
        self.assertEqual(recommendation["best_near_tie_method"], "target_shrunk_huber_global_inverse_depth_linear_shrink050")
        self.assertEqual(recommendation["best_near_tie_profile_path"], "scenario_near_tie_best_runtime_profile.json")
        self.assertTrue(recommendation["best_near_tie_promotion_pass"])

    def test_deployment_strategy_does_not_promote_near_tie_without_promotion_pass(self) -> None:
        module = load_sweep_module()

        recommendation = module.deployment_strategy_recommendation(
            policy_transfer_summary_rows=[],
            scenario_catalog_summary={"pass_scenario_profile_catalog": True, "catalog_profile_count": 2},
            scenario_catalog_promotion_summary={"pass_catalog_promotion_checks": True, "catalog_promotion_pass_count": 2},
            best_near_tie_profile={
                "selected": True,
                "profile_id": "stress_rank004_seed1_candidate",
                "method": "target_shrunk_huber_global_inverse_depth_linear_shrink050",
                "promoted_profile_path": "scenario_near_tie_best_runtime_profile.json",
                "transfer_replay_count": 4,
                "transfer_replay_pass_count": 4,
                "validation_range_rmse_corrected_mean_m": 0.7,
                "target_abs_mean_error_corrected_max_m": 0.5,
            },
            best_near_tie_promotion_check={"pass_promotion_check": False},
        )

        self.assertEqual(recommendation["deployment_strategy"], "scenario_profile_catalog")
        self.assertFalse(recommendation["near_tie_profile_pass"])
        self.assertFalse(recommendation["best_near_tie_promotion_pass"])

    def test_deployment_strategy_promotes_pooled_profile_before_catalog(self) -> None:
        module = load_sweep_module()

        recommendation = module.deployment_strategy_recommendation(
            policy_transfer_summary_rows=[
                {
                    "policy": "balanced",
                    "method": "sim3_binned_residual_disparity",
                    "transfer_replay_count": 6,
                    "transfer_replay_pass_count": 4,
                    "pass_all_transfer_replays": False,
                    "validation_range_rmse_corrected_mean_m": 1.5,
                    "target_abs_mean_error_corrected_max_m": 4.7,
                }
            ],
            scenario_catalog_summary={"pass_scenario_profile_catalog": True, "catalog_profile_count": 3},
            scenario_catalog_promotion_summary={"pass_catalog_promotion_checks": True, "catalog_promotion_pass_count": 3},
            pooled_profile_summary={
                "method": "sim3_kernel_smoothed_binned_residual_disparity_sigma080_support5",
                "profile_path": "pooled_scenario_profile/pipeline/sim3_runtime_profile.json",
                "transfer_replay_count": 6,
                "transfer_replay_pass_count": 6,
                "pass_all_transfer_replays": True,
                "pass_promotion_check": True,
                "runtime_truth_used": False,
            },
        )

        self.assertEqual(recommendation["deployment_strategy"], "single_pooled_runtime_profile")
        self.assertTrue(recommendation["single_profile_pass"])
        self.assertTrue(recommendation["pooled_profile_pass"])
        self.assertEqual(
            recommendation["pooled_profile_method"],
            "sim3_kernel_smoothed_binned_residual_disparity_sigma080_support5",
        )

    def test_deployment_strategy_prefers_catalog_when_universal_transfer_fails(self) -> None:
        module = load_sweep_module()

        recommendation = module.deployment_strategy_recommendation(
            policy_transfer_summary_rows=[
                {
                    "policy": "balanced",
                    "method": "sim3_binned_residual_disparity",
                    "transfer_replay_count": 18,
                    "transfer_replay_pass_count": 15,
                    "pass_all_transfer_replays": False,
                    "validation_range_rmse_corrected_mean_m": 1.5,
                    "target_abs_mean_error_corrected_max_m": 4.7,
                }
            ],
            scenario_catalog_summary={"pass_scenario_profile_catalog": True, "catalog_profile_count": 6},
            scenario_catalog_promotion_summary={"pass_catalog_promotion_checks": True, "catalog_promotion_pass_count": 6},
        )

        self.assertEqual(recommendation["deployment_strategy"], "scenario_profile_catalog")
        self.assertFalse(recommendation["single_profile_pass"])
        self.assertTrue(recommendation["scenario_catalog_ready"])
        self.assertIn("must not use PX4/RTK truth", recommendation["selection_contract"])

    def test_collect_scenario_profile_catalog_copies_profiles_and_gates_cross_seed(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            scenario = module.Scenario("nominal", "test purpose", ())
            scenario_dir = output_dir / "nominal"
            scenario_dir.mkdir(parents=True)
            (scenario_dir / "representative_runtime_profile.json").write_text("{}", encoding="utf-8")
            (scenario_dir / "multi_seed_representative_profile.json").write_text(
                '{"seed": 1, "method": "sim3"}\n',
                encoding="utf-8",
            )
            (scenario_dir / "multi_seed_recommendation.json").write_text(
                '{"method": "sim3", "recommendation_mode": "strict_pass", '
                '"validation_range_rmse_mean_m": 1.0, "target_abs_mean_error_max_m": 0.5}\n',
                encoding="utf-8",
            )
            (scenario_dir / "representative_cross_seed_replay_summary.json").write_text(
                '{"pass_cross_seed_replay": true, "passed_seed_count": 2, "seed_count": 2}\n',
                encoding="utf-8",
            )

            rows, summary = module.collect_scenario_profile_catalog(
                output_dir=output_dir,
                scenarios=[scenario],
            )

            self.assertEqual(len(rows), 1)
            self.assertTrue(rows[0]["profile_exists"])
            self.assertTrue(rows[0]["pass_cross_seed_replay"])
            self.assertTrue(summary["pass_scenario_profile_catalog"])
            self.assertTrue((output_dir / rows[0]["catalog_profile_path"]).is_file())
            self.assertTrue((output_dir / "scenario_profile_catalog" / "manifest.json").is_file())
            self.assertEqual(rows[0]["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_collect_scenario_near_tie_profiles_preserves_profile_paths(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            scenario = module.Scenario("nominal", "test purpose", ())
            scenario_dir = output_dir / "nominal"
            profile_path = scenario_dir / "seed_1" / "runtime_profiles" / "candidate.json"
            profile_path.parent.mkdir(parents=True)
            profile_path.write_text("{}", encoding="utf-8")
            (scenario_dir / "selection_near_tie_profiles.csv").write_text(
                "seed,method,profile_path,selection_rank,selection_score_delta_from_winner\n"
                "1,candidate,seed_1/runtime_profiles/candidate.json,2,0.01\n",
                encoding="utf-8",
            )

            rows = module.collect_scenario_near_tie_profiles(
                output_dir=output_dir,
                scenarios=[scenario],
            )

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["scenario"], "nominal")
        self.assertEqual(rows[0]["scenario_profile_path"], "nominal/seed_1/runtime_profiles/candidate.json")
        self.assertTrue(rows[0]["scenario_profile_exists"])
        self.assertEqual(rows[0]["axis_contract"], module.RDF_AXIS_CONTRACT)
        self.assertEqual(
            rows[0]["runtime_input_contract"],
            "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
        )

    def test_near_tie_profile_transfer_replay_summarizes_candidate_transfer(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            scenario = module.Scenario("nominal", "test purpose", ())
            profile = output_dir / "nominal" / "seed_1" / "runtime_profiles" / "candidate.json"
            profile.parent.mkdir(parents=True)
            profile.write_text("{}", encoding="utf-8")
            seed_dir = output_dir / "nominal" / "seed_1"
            seed_dir.mkdir(parents=True, exist_ok=True)
            (seed_dir / "calibration_samples.csv").write_text("split\nvalidation\n", encoding="utf-8")
            (output_dir / "nominal" / "seed_runs.csv").write_text(
                "seed,run_dir,pass_run\n"
                f"1,{seed_dir},True\n",
                encoding="utf-8",
            )

            def fake_run(command, cwd):
                replay_dir = Path(command[command.index("--output-dir") + 1])
                replay_dir.mkdir(parents=True)
                (replay_dir / "profile_replay_summary.csv").write_text(
                    "validation_range_rmse_raw_m,validation_range_rmse_corrected_m,"
                    "target_abs_mean_error_corrected_m,target_window_count\n"
                    "2.0,0.5,0.4,3\n",
                    encoding="utf-8",
                )
                return SimpleNamespace(returncode=0)

            with patch.object(module.subprocess, "run", side_effect=fake_run):
                replay_rows, summary_rows = module.run_scenario_near_tie_profile_transfer_replay(
                    output_dir=output_dir,
                    scenarios=[scenario],
                    near_tie_profiles=[
                        {
                            "scenario": "nominal",
                            "seed": "1",
                            "selection_rank": "2",
                            "method": "candidate",
                            "selection_score_delta_from_winner": "0.01",
                            "scenario_profile_path": "nominal/seed_1/runtime_profiles/candidate.json",
                            "scenario_profile_exists": True,
                        }
                    ],
                    target_range_m=1852.0,
                    target_window_m=30.0,
                )

        self.assertEqual(len(replay_rows), 1)
        self.assertTrue(replay_rows[0]["pass_replay"])
        self.assertEqual(replay_rows[0]["source_selection_rank"], 2)
        self.assertEqual(len(summary_rows), 1)
        self.assertTrue(summary_rows[0]["pass_all_transfer_replays"])
        self.assertEqual(summary_rows[0]["transfer_replay_pass_count"], 1)
        self.assertEqual(summary_rows[0]["registration_model"], "other_estimated_to_truth")
        self.assertEqual(summary_rows[0]["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_choose_best_near_tie_transfer_profile_copies_selected_profile(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            source_profile = output_dir / "nominal" / "seed_1" / "runtime_profiles" / "candidate.json"
            source_profile.parent.mkdir(parents=True)
            source_profile.write_text('{"mode": "estimated_to_truth_disparity_poly"}\n', encoding="utf-8")

            selected = module.choose_best_near_tie_transfer_profile(
                output_dir=output_dir,
                transfer_summary_rows=[
                    {
                        "profile_id": "nominal_rank002_seed1_candidate",
                        "source_scenario": "nominal",
                        "source_seed": 1,
                        "source_selection_rank": 2,
                        "source_method": "candidate",
                        "source_profile_path": "nominal/seed_1/runtime_profiles/candidate.json",
                        "source_score_delta_from_winner": 0.01,
                        "transfer_replay_count": 4,
                        "transfer_replay_pass_count": 4,
                        "pass_all_transfer_replays": True,
                        "validation_range_rmse_corrected_mean_m": 0.5,
                        "validation_range_rmse_corrected_max_m": 0.6,
                        "validation_range_rmse_improvement_min_m": 1.0,
                        "target_abs_mean_error_corrected_mean_m": 0.2,
                        "target_abs_mean_error_corrected_max_m": 0.3,
                    }
                ],
            )

            promoted = output_dir / "scenario_near_tie_best_runtime_profile.json"
            promoted_exists = promoted.is_file()

        self.assertTrue(selected["selected"])
        self.assertEqual(selected["promoted_profile_path"], "scenario_near_tie_best_runtime_profile.json")
        self.assertTrue(promoted_exists)
        self.assertEqual(selected["profile_id"], "nominal_rank002_seed1_candidate")
        self.assertEqual(selected["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_best_near_tie_profile_promotion_check_uses_promoted_profile(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            profile = output_dir / "scenario_near_tie_best_runtime_profile.json"
            profile.write_text("{}", encoding="utf-8")
            sample = output_dir / "nominal" / "seed_1" / "calibration_samples.csv"
            sample.parent.mkdir(parents=True)
            sample.write_text("split\nvalidation\n", encoding="utf-8")

            def fake_run(command, cwd):
                promotion_dir = Path(command[command.index("--output-dir") + 1])
                promotion_dir.mkdir(parents=True)
                (promotion_dir / "promotion_check.json").write_text(
                    "{"
                    '"pass_promotion_check": true,'
                    '"schema_load_ok": true,'
                    '"node_loader_ok": true,'
                    '"runtime_truth_used": false,'
                    '"forbidden_runtime_inputs_declared": true,'
                    '"sample_apply_count": 3,'
                    '"sample_invalid_count": 0'
                    "}\n",
                    encoding="utf-8",
                )
                return SimpleNamespace(returncode=0)

            with patch.object(module.subprocess, "run", side_effect=fake_run):
                summary = module.run_best_near_tie_profile_promotion_check(
                    output_dir=output_dir,
                    best_profile={
                        "selected": True,
                        "method": "candidate",
                        "promoted_profile_path": "scenario_near_tie_best_runtime_profile.json",
                        "source_scenario": "nominal",
                        "source_seed": 1,
                    },
                )

        self.assertTrue(summary["pass_promotion_check"])
        self.assertEqual(summary["profile_path"], "scenario_near_tie_best_runtime_profile.json")
        self.assertFalse(summary["runtime_truth_used"])
        self.assertTrue(summary["forbidden_runtime_inputs_declared"])
        self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_aggregate_near_tie_transfer_methods_ranks_repeatable_methods(self) -> None:
        module = load_sweep_module()

        rows = module.aggregate_near_tie_transfer_methods(
            [
                {
                    "profile_id": "a_rank001_seed1",
                    "source_method": "target_shrunk_huber_global_inverse_depth_linear_shrink050",
                    "source_scenario": "nominal",
                    "source_seed": 1,
                    "source_selection_rank": 1,
                    "source_profile_path": "nominal/seed_1/runtime_profiles/a.json",
                    "pass_all_transfer_replays": True,
                    "validation_range_rmse_corrected_mean_m": 0.7,
                    "validation_range_rmse_corrected_max_m": 0.8,
                    "validation_range_rmse_improvement_min_m": 1.0,
                    "target_abs_mean_error_corrected_max_m": 0.5,
                    "source_score_delta_from_winner": 0.01,
                },
                {
                    "profile_id": "a_rank002_seed2",
                    "source_method": "target_shrunk_huber_global_inverse_depth_linear_shrink050",
                    "source_scenario": "nominal",
                    "source_seed": 2,
                    "source_selection_rank": 2,
                    "source_profile_path": "nominal/seed_2/runtime_profiles/a.json",
                    "pass_all_transfer_replays": True,
                    "validation_range_rmse_corrected_mean_m": 0.6,
                    "validation_range_rmse_corrected_max_m": 0.75,
                    "validation_range_rmse_improvement_min_m": 1.2,
                    "target_abs_mean_error_corrected_max_m": 0.6,
                    "source_score_delta_from_winner": 0.02,
                },
                {
                    "profile_id": "b_rank001_seed1",
                    "source_method": "global_inverse_depth_linear",
                    "source_scenario": "nominal",
                    "source_seed": 1,
                    "source_selection_rank": 1,
                    "source_profile_path": "nominal/seed_1/runtime_profiles/b.json",
                    "pass_all_transfer_replays": False,
                    "validation_range_rmse_corrected_mean_m": 0.5,
                    "validation_range_rmse_corrected_max_m": 0.9,
                    "validation_range_rmse_improvement_min_m": -0.1,
                    "target_abs_mean_error_corrected_max_m": 0.4,
                    "source_score_delta_from_winner": 0.0,
                },
            ]
        )

        self.assertEqual(rows[0]["source_method"], "target_shrunk_huber_global_inverse_depth_linear_shrink050")
        self.assertEqual(rows[0]["near_tie_profile_count"], 2)
        self.assertEqual(rows[0]["pass_all_profile_count"], 2)
        self.assertEqual(rows[0]["best_profile_id"], "a_rank002_seed2")
        self.assertIn("huber_irls", rows[0]["regularization_model"])
        self.assertEqual(rows[0]["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_scenario_catalog_promotion_checks_aggregate_loader_results(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            profile = output_dir / "scenario_profile_catalog" / "nominal_runtime_profile.json"
            profile.parent.mkdir(parents=True)
            profile.write_text("{}", encoding="utf-8")
            sample = output_dir / "nominal" / "seed_1" / "calibration_samples.csv"
            sample.parent.mkdir(parents=True)
            sample.write_text("split\nvalidation\n", encoding="utf-8")

            def fake_run(command, cwd):
                promotion_dir = Path(command[command.index("--output-dir") + 1])
                promotion_dir.mkdir(parents=True)
                (promotion_dir / "promotion_check.json").write_text(
                    "{"
                    '"pass_promotion_check": true,'
                    '"schema_load_ok": true,'
                    '"node_loader_ok": true,'
                    '"runtime_truth_used": false,'
                    '"forbidden_runtime_inputs_declared": true,'
                    '"sample_apply_count": 3,'
                    '"sample_invalid_count": 0'
                    "}\n",
                    encoding="utf-8",
                )
                return SimpleNamespace(returncode=0)

            with patch.object(module.subprocess, "run", side_effect=fake_run):
                rows, summary = module.run_scenario_catalog_promotion_checks(
                    output_dir=output_dir,
                    catalog_rows=[
                        {
                            "scenario": "nominal",
                            "method": "sim3",
                            "source_seed": 1,
                            "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                            "profile_exists": True,
                        }
                    ],
                )

            self.assertEqual(len(rows), 1)
            self.assertTrue(rows[0]["pass_promotion_check"])
            self.assertEqual(summary["catalog_promotion_pass_count"], 1)
            self.assertTrue(summary["pass_catalog_promotion_checks"])

    def test_catalog_profile_transfer_matrix_summarizes_cross_scenario_coverage(self) -> None:
        module = load_sweep_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            profile_dir = output_dir / "scenario_profile_catalog"
            profile_dir.mkdir(parents=True)
            for source in ("nominal", "stress"):
                (profile_dir / f"{source}_runtime_profile.json").write_text("{}", encoding="utf-8")
            scenarios = [
                module.Scenario("nominal", "test nominal", ()),
                module.Scenario("stress", "test stress", ()),
            ]
            for scenario in scenarios:
                seed_dir = output_dir / scenario.name / "seed_1"
                seed_dir.mkdir(parents=True)
                (seed_dir / "calibration_samples.csv").write_text("split\nvalidation\n", encoding="utf-8")
                (output_dir / scenario.name / "seed_runs.csv").write_text(
                    "seed,run_dir,exit_code,pass_run\n"
                    f"1,{seed_dir},0,True\n",
                    encoding="utf-8",
                )

            def fake_run(command, cwd):
                profile_path = Path(command[command.index("--profile-json") + 1])
                replay_dir = Path(command[command.index("--output-dir") + 1])
                replay_dir.mkdir(parents=True)
                target = replay_dir.parent.name
                source = profile_path.name.replace("_runtime_profile.json", "")
                if source == "nominal" and target == "stress":
                    raw, corrected, target_abs = 2.0, 2.5, 0.5
                    returncode = 1
                else:
                    raw, corrected, target_abs = 2.0, 1.0, 0.5
                    returncode = 0
                (replay_dir / "profile_replay_summary.csv").write_text(
                    "validation_range_rmse_raw_m,validation_range_rmse_corrected_m,"
                    "target_abs_mean_error_corrected_m,target_window_count\n"
                    f"{raw},{corrected},{target_abs},1\n",
                    encoding="utf-8",
                )
                return SimpleNamespace(returncode=returncode)

            with patch.object(module.subprocess, "run", side_effect=fake_run):
                replay_rows, pair_rows, source_rows = module.run_catalog_profile_transfer_matrix(
                    output_dir=output_dir,
                    scenarios=scenarios,
                    catalog_rows=[
                        {
                            "scenario": "nominal",
                            "method": "sim3",
                            "profile_exists": True,
                            "catalog_profile_path": "scenario_profile_catalog/nominal_runtime_profile.json",
                        },
                        {
                            "scenario": "stress",
                            "method": "sim3",
                            "profile_exists": True,
                            "catalog_profile_path": "scenario_profile_catalog/stress_runtime_profile.json",
                        },
                    ],
                    target_range_m=1852.0,
                    target_window_m=30.0,
                )

            self.assertEqual(len(replay_rows), 4)
            by_pair = {(row["source_scenario"], row["target_scenario"]): row for row in pair_rows}
            self.assertFalse(by_pair[("nominal", "stress")]["pass_all"])
            self.assertTrue(by_pair[("stress", "nominal")]["pass_all"])
            by_source = {row["source_scenario"]: row for row in source_rows}
            self.assertEqual(by_source["nominal"]["covered_target_scenario_count"], 1)
            self.assertEqual(by_source["stress"]["covered_target_scenario_count"], 2)
            self.assertEqual(by_source["stress"]["axis_contract"], module.RDF_AXIS_CONTRACT)

            minimal_rows, minimal_summary = module.choose_catalog_minimal_profile_set(pair_rows)
            self.assertTrue(minimal_summary["pass_minimal_catalog_profile_set"])
            self.assertEqual(minimal_summary["selected_profile_count"], 1)
            self.assertEqual(minimal_summary["selected_source_scenarios"], ["stress"])
            self.assertEqual(minimal_rows[0]["newly_covered_target_scenarios"], "nominal,stress")
            self.assertEqual(minimal_rows[0]["axis_contract"], module.RDF_AXIS_CONTRACT)

            csv_roundtrip_rows = [{**row, "pass_all": str(row["pass_all"])} for row in pair_rows]
            _, csv_roundtrip_summary = module.choose_catalog_minimal_profile_set(csv_roundtrip_rows)
            self.assertEqual(csv_roundtrip_summary["selected_source_scenarios"], ["stress"])

    def test_catalog_minimal_profile_set_reports_uncovered_targets(self) -> None:
        module = load_sweep_module()
        pair_rows = [
            {
                "source_scenario": "nominal",
                "target_scenario": "nominal",
                "source_method": "sim3",
                "pass_all": True,
                "pass_count": 3,
                "replay_count": 3,
                "mean_validation_range_rmse_corrected_m": 1.0,
                "max_target_abs_mean_error_corrected_m": 2.0,
            },
            {
                "source_scenario": "nominal",
                "target_scenario": "stress",
                "source_method": "sim3",
                "pass_all": False,
                "pass_count": 1,
                "replay_count": 3,
                "mean_validation_range_rmse_corrected_m": 8.0,
                "max_target_abs_mean_error_corrected_m": 12.0,
            },
        ]
        minimal_rows, minimal_summary = module.choose_catalog_minimal_profile_set(pair_rows)
        self.assertFalse(minimal_summary["pass_minimal_catalog_profile_set"])
        self.assertEqual(minimal_summary["selected_source_scenarios"], ["nominal"])
        self.assertEqual(minimal_summary["uncovered_target_scenarios"], ["stress"])
        self.assertEqual(minimal_rows[0]["remaining_uncovered_target_scenarios"], "stress")

    def test_pooled_universal_profile_selection_prioritizes_target_worst_case(self) -> None:
        module = load_sweep_module()
        candidates = [
            {
                "method": "mean_rmse_best_but_target_worse",
                "transfer_replay_pass_count": 18,
                "pass_all_transfer_replays": True,
                "validation_range_rmse_corrected_mean_m": 1.40,
                "validation_range_rmse_corrected_max_m": 2.45,
                "validation_range_rmse_improvement_min_m": 0.1,
                "target_abs_mean_error_corrected_max_m": 4.2,
                "model_complexity": 9,
            },
            {
                "method": "target_worst_case_best",
                "transfer_replay_pass_count": 18,
                "pass_all_transfer_replays": True,
                "validation_range_rmse_corrected_mean_m": 1.45,
                "validation_range_rmse_corrected_max_m": 2.50,
                "validation_range_rmse_improvement_min_m": 0.1,
                "target_abs_mean_error_corrected_max_m": 3.8,
                "model_complexity": 13,
            },
            {
                "method": "failed_transfer",
                "transfer_replay_pass_count": 17,
                "pass_all_transfer_replays": False,
                "validation_range_rmse_corrected_mean_m": 0.4,
                "validation_range_rmse_corrected_max_m": 0.6,
                "validation_range_rmse_improvement_min_m": 0.1,
                "target_abs_mean_error_corrected_max_m": 1.0,
                "model_complexity": 3,
            },
        ]

        selected = sorted(candidates, key=module.pooled_universal_profile_sort_key)[0]

        self.assertEqual(selected["method"], "target_worst_case_best")

    def test_pooled_profile_failure_diagnostics_identifies_failed_scenario(self) -> None:
        module = load_sweep_module()
        replay_rows = [
            {
                "method": "pooled_a",
                "target_scenario": "nominal",
                "pass_replay": True,
                "replay_returncode": 0,
                "validation_range_rmse_raw_m": 2.0,
                "validation_range_rmse_corrected_m": 0.8,
                "validation_range_rmse_improvement_m": 1.2,
                "target_window_count": 4,
                "target_abs_mean_error_corrected_m": 1.0,
            },
            {
                "method": "pooled_a",
                "target_scenario": "intrinsic_scale_stress",
                "pass_replay": False,
                "replay_returncode": 0,
                "validation_range_rmse_raw_m": 1.6,
                "validation_range_rmse_corrected_m": 2.1,
                "validation_range_rmse_improvement_m": -0.5,
                "target_window_count": 4,
                "target_abs_mean_error_corrected_m": 3.0,
            },
            {
                "method": "pooled_b",
                "target_scenario": "intrinsic_scale_stress",
                "pass_replay": False,
                "replay_returncode": 0,
                "validation_range_rmse_raw_m": 1.6,
                "validation_range_rmse_corrected_m": 1.7,
                "validation_range_rmse_improvement_m": -0.1,
                "target_window_count": 4,
                "target_abs_mean_error_corrected_m": 2.0,
            },
        ]

        rows, summary = module.pooled_profile_failure_diagnostics(
            replay_rows,
            candidate_summary_rows=[
                {
                    "method": "pooled_a",
                    "pass_all_transfer_replays": False,
                }
            ],
            selected_summary={
                "method": "pooled_a",
                "pass_all_transfer_replays": False,
                "transfer_replay_pass_count": 1,
                "transfer_replay_count": 2,
            },
        )

        by_scenario = {row["target_scenario"]: row for row in rows}
        self.assertTrue(by_scenario["nominal"]["selected_pass_replay"])
        self.assertFalse(by_scenario["intrinsic_scale_stress"]["pass_any_pooled_candidate"])
        self.assertIn("no_validation_rmse_improvement", by_scenario["intrinsic_scale_stress"]["failure_reasons"])
        self.assertEqual(
            by_scenario["intrinsic_scale_stress"]["recommended_action"],
            "use_catalog_or_minimal_set_profile_for_this_scenario",
        )
        self.assertEqual(summary["selected_failed_scenarios"], ["intrinsic_scale_stress"])
        self.assertEqual(summary["recommended_action"], "use_catalog_or_minimal_set_for_failed_scenarios")
        self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)


if __name__ == "__main__":
    unittest.main()
