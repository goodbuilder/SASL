from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location(
        "plan_stereodistance_method_search_under_test",
        ROOT_DIR / "scripts" / "plan_stereodistance_method_search.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load plan_stereodistance_method_search.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PlanStereoDistanceMethodSearchTests(unittest.TestCase):
    def test_plans_frontier_and_family_actions_from_robustness_summary(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            robustness = root / "robustness_summary.json"
            dashboard = root / "dashboard_summary.json"
            robustness.write_text(
                json.dumps(
                    {
                        "robust_recommendation": {"method": "active_huber_ray"},
                        "robust_method_summary": [
                            {
                                "method": "active_huber_ray",
                                "registration_model": "raw_rdf_ray_scale",
                                "residual_model": "none",
                                "regularization_model": "huber_irls",
                                "runtime_profile_family": "estimated_to_truth_sim3",
                                "selection_note": "runtime candidate",
                                "expected_scenario_count": 3,
                                "strict_pass_count": 3,
                                "performance_pass_count": 3,
                                "strict_pass_all_scenarios": True,
                                "performance_pass_all_scenarios": True,
                                "validation_range_rmse_mean_m": 1.2,
                                "validation_range_rmse_max_m": 1.5,
                                "target_abs_mean_error_max_m": 2.0,
                                "axis_rmse_regression_sum_mean_m": 0.0,
                                "model_complexity_mean": 6,
                                "robust_score": 1.4,
                            },
                            {
                                "method": "needs_stability",
                                "registration_model": "affine_rdf_estimated_to_truth",
                                "residual_model": "forward_raw_rdf_forward_lut",
                                "regularization_model": "none",
                                "runtime_profile_family": "estimated_to_truth_affine_rdf",
                                "selection_note": "alternate",
                                "expected_scenario_count": 3,
                                "strict_pass_count": 2,
                                "performance_pass_count": 3,
                                "strict_pass_all_scenarios": False,
                                "performance_pass_all_scenarios": True,
                                "validation_range_rmse_mean_m": 1.0,
                                "validation_range_rmse_max_m": 1.7,
                                "target_abs_mean_error_max_m": 3.0,
                                "axis_rmse_regression_sum_mean_m": 0.0,
                                "model_complexity_mean": 12,
                                "robust_score": 1.6,
                            },
                            {
                                "method": "axis_regresses",
                                "registration_model": "sim3_estimated_to_truth",
                                "residual_model": "none",
                                "regularization_model": "none",
                                "runtime_profile_family": "estimated_to_truth_sim3",
                                "selection_note": "diagnostic",
                                "expected_scenario_count": 3,
                                "strict_pass_count": 3,
                                "performance_pass_count": 3,
                                "strict_pass_all_scenarios": True,
                                "performance_pass_all_scenarios": True,
                                "validation_range_rmse_mean_m": 0.9,
                                "validation_range_rmse_max_m": 1.1,
                                "target_abs_mean_error_max_m": 2.5,
                                "axis_rmse_regression_sum_mean_m": 0.4,
                                "model_complexity_mean": 9,
                                "robust_score": 1.0,
                            },
                        ],
                    },
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            dashboard.write_text(
                json.dumps({"active_method": "active_huber_ray"}, ensure_ascii=False),
                encoding="utf-8",
            )

            summary = module.plan_method_search(
                robustness_summary_json=robustness,
                dashboard_summary_json=dashboard,
                output_dir=root / "out",
                top_n=3,
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertFalse(summary["runtime_truth_used"])
            self.assertEqual(summary["axis_contract"], module.RDF_AXIS_CONTRACT)
            self.assertEqual(summary["active_method_source"], "dashboard_summary_json")
            self.assertTrue(summary["active_method_summary_rdf_axis_nonregression_pass"])
            frontier_csv = (root / "out" / "method_search_frontier.csv").read_text(encoding="utf-8")
            self.assertIn("active_huber_ray", frontier_csv)
            self.assertIn("rdf_axis_nonregression_pass", frontier_csv)
            family_csv = (root / "out" / "family_search_priorities.csv").read_text(encoding="utf-8")
            self.assertIn("exploit_active_family", family_csv)
            self.assertIn("add_stability_regularization", family_csv)
            self.assertIn("diagnose_axis_regression", family_csv)
            summary_txt = (root / "out" / "method_search_plan.txt").read_text(encoding="utf-8")
            self.assertIn("runtime StereoDistance profiles consume stereo-center RDF", summary_txt)

    def test_requires_non_empty_method_summary(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            robustness = root / "robustness_summary.json"
            robustness.write_text(json.dumps({"robust_method_summary": []}), encoding="utf-8")
            with self.assertRaises(ValueError):
                module.plan_method_search(
                    robustness_summary_json=robustness,
                    output_dir=root / "out",
                )

    def test_status_passes_when_active_method_is_not_in_top_frontier_but_is_validated(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            robustness = root / "robustness_summary.json"
            methods = [
                {
                    "method": f"frontier_{index}",
                    "registration_model": "sim3_estimated_to_truth",
                    "residual_model": "none",
                    "regularization_model": "none",
                    "runtime_profile_family": "estimated_to_truth_sim3",
                    "expected_scenario_count": 2,
                    "strict_pass_count": 2,
                    "performance_pass_count": 2,
                    "strict_pass_all_scenarios": True,
                    "performance_pass_all_scenarios": True,
                    "validation_range_rmse_mean_m": 0.5 + index * 0.001,
                    "validation_range_rmse_max_m": 0.7,
                    "target_abs_mean_error_max_m": 0.5,
                    "axis_rmse_regression_sum_mean_m": 0.0,
                    "model_complexity_mean": 5,
                    "robust_score": 0.5 + index * 0.001,
                }
                for index in range(25)
            ]
            methods.append(
                {
                    "method": "active_validated_but_not_top20",
                    "registration_model": "raw_rdf_inverse_depth_ray_scale",
                    "residual_model": "corrected_inverse_depth_linear",
                    "regularization_model": "theil_sen_robust_regression",
                    "runtime_profile_family": "estimated_to_truth_disparity_poly",
                    "expected_scenario_count": 2,
                    "strict_pass_count": 2,
                    "performance_pass_count": 2,
                    "strict_pass_all_scenarios": True,
                    "performance_pass_all_scenarios": True,
                    "validation_range_rmse_mean_m": 1.5,
                    "validation_range_rmse_max_m": 1.7,
                    "target_abs_mean_error_max_m": 0.6,
                    "axis_rmse_regression_sum_mean_m": 0.0,
                    "model_complexity_mean": 3,
                    "robust_score": 1.5,
                }
            )
            robustness.write_text(
                json.dumps(
                    {
                        "robust_recommendation": {"method": "active_validated_but_not_top20"},
                        "robust_method_summary": methods,
                    },
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )

            summary = module.plan_method_search(
                robustness_summary_json=robustness,
                output_dir=root / "out",
                top_n=20,
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertTrue(summary["active_method_found"])
            self.assertFalse(summary["active_method_in_top_frontier"])
            self.assertTrue(summary["active_method_rdf_axis_nonregression_pass"])

    def test_active_profile_json_disambiguates_exported_profile_from_robust_frontier(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            robustness = root / "robustness_summary.json"
            active_profile = root / "active_runtime_profile.json"
            methods = [
                {
                    "method": "frontier_diagnostic",
                    "registration_model": "raw_rdf_ray_scale",
                    "residual_model": "none",
                    "regularization_model": "huber_irls",
                    "runtime_profile_family": "offline_or_nonpreferred_runtime_candidate",
                    "expected_scenario_count": 2,
                    "strict_pass_count": 2,
                    "performance_pass_count": 2,
                    "strict_pass_all_scenarios": True,
                    "performance_pass_all_scenarios": True,
                    "validation_range_rmse_mean_m": 1.0,
                    "validation_range_rmse_max_m": 1.2,
                    "target_abs_mean_error_max_m": 0.8,
                    "axis_rmse_regression_sum_mean_m": 0.0,
                    "model_complexity_mean": 5,
                    "robust_score": 1.0,
                },
                {
                    "method": "exported_runtime_profile",
                    "registration_model": "sim3_estimated_to_truth",
                    "residual_model": "none",
                    "regularization_model": "none",
                    "runtime_profile_family": "estimated_to_truth_sim3",
                    "expected_scenario_count": 2,
                    "strict_pass_count": 2,
                    "performance_pass_count": 2,
                    "strict_pass_all_scenarios": True,
                    "performance_pass_all_scenarios": True,
                    "validation_range_rmse_mean_m": 1.1,
                    "validation_range_rmse_max_m": 1.3,
                    "target_abs_mean_error_max_m": 0.9,
                    "axis_rmse_regression_sum_mean_m": 0.0,
                    "model_complexity_mean": 6,
                    "robust_score": 1.1,
                },
            ]
            robustness.write_text(
                json.dumps(
                    {
                        "robust_recommendation": {"method": "frontier_diagnostic"},
                        "robust_method_summary": methods,
                    },
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            active_profile.write_text(
                json.dumps({"method": "exported_runtime_profile"}, ensure_ascii=False),
                encoding="utf-8",
            )

            summary = module.plan_method_search(
                robustness_summary_json=robustness,
                active_profile_json=active_profile,
                output_dir=root / "out",
                top_n=2,
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertEqual(summary["active_method"], "exported_runtime_profile")
            self.assertEqual(summary["active_method_source"], "active_profile_json")
            self.assertEqual(summary["robust_recommendation_method"], "frontier_diagnostic")
            self.assertFalse(summary["active_profile_matches_robust_recommendation"])
            txt = (root / "out" / "method_search_plan.txt").read_text(encoding="utf-8")
            self.assertIn("active_method_source: active_profile_json", txt)

    def test_dashboard_axis_pass_can_validate_exported_profile_when_method_summary_regresses(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            robustness = root / "robustness_summary.json"
            dashboard = root / "dashboard_summary.json"
            active_profile = root / "active_runtime_profile.json"
            robustness.write_text(
                json.dumps(
                    {
                        "robust_recommendation": {"method": "frontier_axis_clean"},
                        "robust_method_summary": [
                            {
                                "method": "exported_profile",
                                "registration_model": "raw_rdf_inverse_depth_ray_scale",
                                "residual_model": "corrected_inverse_depth_linear",
                                "regularization_model": "inverse_depth_residual",
                                "runtime_profile_family": "estimated_to_truth_disparity_poly",
                                "expected_scenario_count": 2,
                                "strict_pass_count": 2,
                                "performance_pass_count": 2,
                                "strict_pass_all_scenarios": True,
                                "performance_pass_all_scenarios": True,
                                "validation_range_rmse_mean_m": 1.2,
                                "validation_range_rmse_max_m": 1.4,
                                "target_abs_mean_error_max_m": 0.9,
                                "axis_rmse_regression_sum_mean_m": 0.01,
                                "model_complexity_mean": 4,
                                "robust_score": 1.2,
                            }
                        ],
                    },
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            dashboard.write_text(
                json.dumps({"final_rdf_axis_nonregression_status": "PASS"}, ensure_ascii=False),
                encoding="utf-8",
            )
            active_profile.write_text(
                json.dumps({"method": "exported_profile"}, ensure_ascii=False),
                encoding="utf-8",
            )

            summary = module.plan_method_search(
                robustness_summary_json=robustness,
                dashboard_summary_json=dashboard,
                active_profile_json=active_profile,
                output_dir=root / "out",
            )

            self.assertEqual(summary["status"], "PASS")
            self.assertFalse(summary["active_method_summary_rdf_axis_nonregression_pass"])
            self.assertTrue(summary["dashboard_rdf_axis_nonregression_pass"])
            self.assertTrue(summary["active_method_rdf_axis_nonregression_pass"])


if __name__ == "__main__":
    unittest.main()
