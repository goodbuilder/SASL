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


def load_multi_seed_module():
    spec = importlib.util.spec_from_file_location(
        "rtk_stereo_policy_multi_seed_validation_under_test",
        ROOT_DIR / "scripts" / "run_rtk_stereo_policy_multi_seed_validation.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load run_rtk_stereo_policy_multi_seed_validation.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RtkStereoPolicyMultiSeedValidationTests(unittest.TestCase):
    def test_parse_seed_list(self) -> None:
        module = load_multi_seed_module()

        self.assertEqual(module.parse_seed_list("1, 2,3"), [1, 2, 3])

    def test_resolve_runtime_paths_absolutizes_output_and_input_csv(self) -> None:
        module = load_multi_seed_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            args = SimpleNamespace(
                output_dir=Path("relative_results"),
                input_csv=tmp / "calibration.csv",
            )
            args.input_csv.write_text("sample_id\n", encoding="utf-8")

            resolved = module.resolve_runtime_paths(args)

        self.assertTrue(resolved.output_dir.is_absolute())
        self.assertTrue(resolved.input_csv.is_absolute())
        self.assertTrue(str(resolved.output_dir).endswith("relative_results"))

    def test_run_pipeline_passes_synthetic_target_oversample_count(self) -> None:
        module = load_multi_seed_module()
        args = SimpleNamespace(
            mode="synthetic",
            input_csv=None,
            sample_count=120,
            train_ratio=0.8,
            target_range_m=1852.0,
            target_window_m=30.0,
            synthetic_target_oversample_count=18,
            baseline_scale_error=0.012,
            focal_scale_error=-0.008,
            yaw_error_deg=0.7,
            pitch_error_deg=-0.35,
            roll_error_deg=0.12,
            disparity_bias_px=0.55,
            disparity_noise_px=0.12,
            bbox_noise_px=0.25,
            disparity_outlier_rate=0.06,
            disparity_outlier_px=0.45,
            bbox_outlier_rate=0.08,
            bbox_outlier_px=2.5,
        )

        with patch.object(module.subprocess, "run") as run_mock:
            run_mock.return_value.returncode = 0
            exit_code = module.run_pipeline(args, 20260613, Path("/tmp/run"))

        command = run_mock.call_args.args[0]
        self.assertEqual(exit_code, 0)
        self.assertIn("--synthetic-target-oversample-count", command)
        self.assertEqual(command[command.index("--synthetic-target-oversample-count") + 1], "18")
        self.assertEqual(command[command.index("--baseline-scale-error") + 1], "0.012")
        self.assertEqual(command[command.index("--focal-scale-error") + 1], "-0.008")
        self.assertEqual(command[command.index("--yaw-error-deg") + 1], "0.7")
        self.assertEqual(command[command.index("--disparity-bias-px") + 1], "0.55")
        self.assertEqual(command[command.index("--disparity-outlier-rate") + 1], "0.06")
        self.assertEqual(command[command.index("--bbox-outlier-px") + 1], "2.5")

    def test_policy_aggregate_tracks_method_stability(self) -> None:
        module = load_multi_seed_module()
        rows = [
            {
                "policy": "balanced",
                "method": "sim3",
                "validation_range_rmse_m": "1.0",
                "validation_position_rmse_m": "1.3",
                "validation_right_rmse_m": "0.2",
                "validation_down_rmse_m": "0.3",
                "validation_forward_rmse_m": "0.9",
                "validation_target_abs_mean_error_m": "0.5",
            },
            {
                "policy": "balanced",
                "method": "sim3",
                "validation_range_rmse_m": "1.2",
                "validation_position_rmse_m": "1.5",
                "validation_right_rmse_m": "0.4",
                "validation_down_rmse_m": "0.5",
                "validation_forward_rmse_m": "1.1",
                "validation_target_abs_mean_error_m": "0.6",
            },
            {
                "policy": "target_specialist",
                "method": "range_weighted_sim3",
                "validation_range_rmse_m": "1.5",
                "validation_position_rmse_m": "1.7",
                "validation_right_rmse_m": "0.5",
                "validation_down_rmse_m": "0.6",
                "validation_forward_rmse_m": "1.4",
                "validation_target_abs_mean_error_m": "0.1",
            },
        ]

        aggregate = module.aggregate_policy_rows(rows)
        by_policy = {row["policy"]: row for row in aggregate}

        self.assertEqual(by_policy["balanced"]["most_common_method"], "sim3")
        self.assertEqual(by_policy["balanced"]["most_common_method_count"], 2)
        self.assertEqual(by_policy["balanced"]["unique_method_count"], 1)
        self.assertAlmostEqual(by_policy["balanced"]["validation_range_rmse_mean_m"], 1.1)
        self.assertAlmostEqual(by_policy["balanced"]["validation_right_rmse_mean_m"], 0.3)
        self.assertAlmostEqual(by_policy["balanced"]["validation_down_rmse_mean_m"], 0.4)
        self.assertAlmostEqual(by_policy["balanced"]["validation_forward_rmse_mean_m"], 1.0)
        self.assertIn("validation_range_rmse_mean_ci90_low_m", by_policy["balanced"])
        self.assertIn("target_abs_mean_error_mean_ci90_high_m", by_policy["balanced"])

    def test_method_aggregate_tracks_selected_count(self) -> None:
        module = load_multi_seed_module()
        rows = [
            {
                "method": "sim3",
                "validation_range_rmse_m": "1.0",
                "validation_position_rmse_m": "1.4",
                "validation_right_rmse_m": "0.3",
                "validation_down_rmse_m": "0.4",
                "validation_forward_rmse_m": "0.9",
                "validation_target_abs_mean_error_m": "0.5",
                "model_complexity": "7",
                "is_final_recommendation": True,
            },
            {
                "method": "sim3",
                "validation_range_rmse_m": "1.2",
                "validation_position_rmse_m": "1.5",
                "validation_right_rmse_m": "0.5",
                "validation_down_rmse_m": "0.6",
                "validation_forward_rmse_m": "1.0",
                "validation_target_abs_mean_error_m": "0.6",
                "model_complexity": "7",
                "is_final_recommendation": False,
            },
        ]

        aggregate = module.aggregate_method_rows(rows)

        self.assertEqual(aggregate[0]["method"], "sim3")
        self.assertEqual(aggregate[0]["selected_count"], 1)
        self.assertAlmostEqual(aggregate[0]["target_abs_mean_error_mean_m"], 0.55)
        self.assertAlmostEqual(aggregate[0]["validation_right_rmse_mean_m"], 0.4)
        self.assertAlmostEqual(aggregate[0]["validation_down_rmse_mean_m"], 0.5)
        self.assertAlmostEqual(aggregate[0]["validation_forward_rmse_mean_m"], 0.95)
        self.assertIn("target_abs_mean_error_mean_ci90_low_m", aggregate[0])

    def test_method_aggregate_adds_runtime_contract_taxonomy(self) -> None:
        module = load_multi_seed_module()
        rows = [
            {
                "method": "sim3_piecewise_raw_forward_bins3",
                "validation_range_rmse_m": "0.8",
                "validation_position_rmse_m": "1.0",
                "validation_right_rmse_m": "0.2",
                "validation_down_rmse_m": "0.3",
                "validation_forward_rmse_m": "0.7",
                "validation_target_abs_mean_error_m": "0.4",
                "model_complexity": "28",
                "is_final_recommendation": False,
            }
        ]

        aggregate = module.aggregate_method_rows(rows)

        self.assertEqual(aggregate[0]["registration_model"], "piecewise_raw_forward_sim3_estimated_to_truth")
        self.assertEqual(aggregate[0]["residual_model"], "none")
        self.assertIn("raw_forward_piecewise_registration", aggregate[0]["regularization_model"])
        self.assertEqual(aggregate[0]["runtime_profile_family"], "estimated_to_truth_sim3")

    def test_bootstrap_mean_ci_handles_single_seed(self) -> None:
        module = load_multi_seed_module()

        low, high = module.bootstrap_mean_ci([1.25])

        self.assertEqual(low, 1.25)
        self.assertEqual(high, 1.25)

    def test_seed_contract_summary_rejects_direct_axis_alias(self) -> None:
        module = load_multi_seed_module()
        valid_payload = {
            "contracts": {
                "truth_rdf_transform_contract": "truth_rdf from explicit T_source_to_stereo_rdf",
                "ned_to_stereo_rdf": (
                    "explicit_transform: truth_rdf = R_ned_to_stereo_rdf * "
                    "(truth_ned - origin_ned); direct axis alias "
                    "rdf_right=east/rdf_down=down/rdf_forward=north is forbidden"
                ),
                "direct_ned_to_rdf_axis_alias_allowed": False,
            }
        }
        invalid_payload = {
            "truth_rdf_transform_contract": (
                "synthetic_bridge: rdf_right=east, rdf_down=down, rdf_forward=north"
            )
        }

        self.assertTrue(module.direct_axis_alias_forbidden(valid_payload))
        self.assertFalse(module.direct_axis_alias_forbidden(invalid_payload))

        summary = module.seed_contract_summary(
            [
                {"run_config_loaded": True, "direct_axis_alias_forbidden": True},
                {"run_config_loaded": True, "direct_axis_alias_forbidden": False},
            ]
        )

        self.assertFalse(summary["all_loaded_contracts_forbid_direct_axis_alias"])
        self.assertEqual(summary["direct_axis_alias_forbidden_count"], 1)

    def test_policy_stability_gate_separates_performance_from_method_stability(self) -> None:
        module = load_multi_seed_module()
        policy_rows = [
            {
                "policy": "balanced",
                "seed_count": 2,
                "most_common_method": "sim3",
                "most_common_method_count": 1,
                "validation_range_rmse_mean_m": 1.0,
                "target_abs_mean_error_max_m": 0.8,
                "target_abs_mean_error_finite_seed_count": 2,
            },
            {
                "policy": "stable_policy",
                "seed_count": 2,
                "most_common_method": "sim3",
                "most_common_method_count": 2,
                "validation_range_rmse_mean_m": 1.1,
                "target_abs_mean_error_max_m": 0.9,
                "target_abs_mean_error_finite_seed_count": 2,
            },
        ]
        method_rows = [
            {
                "method": "raw",
                "validation_range_rmse_mean_m": 5.0,
            }
        ]

        gate_rows = module.policy_stability_gate_rows(
            policy_rows,
            method_rows,
            completed_seed_count=2,
        )
        by_policy = {row["policy"]: row for row in gate_rows}

        self.assertFalse(by_policy["balanced"]["strict_pass"])
        self.assertTrue(by_policy["balanced"]["performance_pass"])
        self.assertTrue(by_policy["balanced"]["validation_ci90_high_improved_vs_raw_mean"])
        self.assertIn("policy_method_not_majority_stable", by_policy["balanced"]["reasons"])
        self.assertTrue(by_policy["stable_policy"]["strict_pass"])

    def test_policy_stability_gate_requires_meaningful_validation_improvement(self) -> None:
        module = load_multi_seed_module()
        policy_rows = [
            {
                "policy": "nearly_raw",
                "seed_count": 2,
                "most_common_method": "global_rotation",
                "most_common_method_count": 2,
                "validation_range_rmse_mean_m": 4.999,
                "target_abs_mean_error_max_m": 1.0,
                "target_abs_mean_error_finite_seed_count": 2,
            }
        ]
        method_rows = [
            {
                "method": "raw",
                "validation_range_rmse_mean_m": 5.0,
            }
        ]

        gate_rows = module.policy_stability_gate_rows(
            policy_rows,
            method_rows,
            completed_seed_count=2,
            min_validation_improvement_m=0.01,
        )

        self.assertFalse(gate_rows[0]["strict_pass"])
        self.assertFalse(gate_rows[0]["validation_improved_vs_raw_mean"])
        self.assertIn("policy_mean_validation_rmse_not_better_than_raw", gate_rows[0]["reasons"])

    def test_method_stability_gate_requires_validation_and_target_pass(self) -> None:
        module = load_multi_seed_module()
        method_rows = [
            {
                "method": "raw",
                "seed_count": 2,
                "selected_count": 0,
                "validation_range_rmse_mean_m": 5.0,
                "validation_range_rmse_mean_ci90_high_m": 5.0,
                "validation_range_rmse_max_m": 5.1,
                "target_abs_mean_error_mean_m": 20.0,
                "target_abs_mean_error_max_m": 21.0,
                "target_abs_mean_error_finite_seed_count": 2,
                "validation_right_rmse_mean_m": 5.0,
                "validation_down_rmse_mean_m": 5.0,
                "validation_forward_rmse_mean_m": 5.0,
                "model_complexity_mean": 0,
            },
            {
                "method": "stable_sim3",
                "seed_count": 2,
                "selected_count": 1,
                "validation_range_rmse_mean_m": 1.0,
                "validation_range_rmse_mean_ci90_high_m": 1.2,
                "validation_range_rmse_max_m": 1.3,
                "target_abs_mean_error_mean_m": 0.8,
                "target_abs_mean_error_max_m": 1.0,
                "target_abs_mean_error_finite_seed_count": 2,
                "validation_right_rmse_mean_m": 1.0,
                "validation_down_rmse_mean_m": 1.0,
                "validation_forward_rmse_mean_m": 1.0,
                "model_complexity_mean": 7,
            },
            {
                "method": "near_raw",
                "seed_count": 2,
                "selected_count": 0,
                "validation_range_rmse_mean_m": 4.995,
                "validation_range_rmse_mean_ci90_high_m": 5.1,
                "validation_range_rmse_max_m": 5.2,
                "target_abs_mean_error_mean_m": 0.8,
                "target_abs_mean_error_max_m": 1.0,
                "target_abs_mean_error_finite_seed_count": 2,
                "validation_right_rmse_mean_m": 4.0,
                "validation_down_rmse_mean_m": 4.0,
                "validation_forward_rmse_mean_m": 4.0,
                "model_complexity_mean": 3,
            },
        ]

        gate_rows = module.method_stability_gate_rows(
            method_rows,
            completed_seed_count=2,
            min_validation_improvement_m=0.01,
        )
        by_method = {row["method"]: row for row in gate_rows}

        self.assertFalse(by_method["raw"]["strict_pass"])
        self.assertTrue(by_method["stable_sim3"]["strict_pass"])
        self.assertTrue(by_method["stable_sim3"]["axis_rmse_not_regressed_vs_raw_mean"])
        self.assertTrue(by_method["stable_sim3"]["performance_pass"])
        self.assertFalse(by_method["near_raw"]["performance_pass"])
        self.assertIn("method_mean_validation_rmse_not_better_than_raw", by_method["near_raw"]["reasons"])

    def test_method_stability_gate_preserves_runtime_contract_taxonomy(self) -> None:
        module = load_multi_seed_module()
        method_rows = [
            {
                "method": "raw",
                "seed_count": 1,
                "selected_count": 0,
                "validation_range_rmse_mean_m": 5.0,
                "validation_range_rmse_mean_ci90_high_m": 5.0,
                "validation_range_rmse_max_m": 5.0,
                "target_abs_mean_error_mean_m": 20.0,
                "target_abs_mean_error_max_m": 20.0,
                "target_abs_mean_error_finite_seed_count": 1,
                "validation_right_rmse_mean_m": 5.0,
                "validation_down_rmse_mean_m": 5.0,
                "validation_forward_rmse_mean_m": 5.0,
                "model_complexity_mean": 0,
                **module.method_contract_fields("raw"),
            },
            {
                "method": "sim3_piecewise_raw_forward_bins3",
                "seed_count": 1,
                "selected_count": 0,
                "validation_range_rmse_mean_m": 0.8,
                "validation_range_rmse_mean_ci90_high_m": 0.8,
                "validation_range_rmse_max_m": 0.8,
                "target_abs_mean_error_mean_m": 0.4,
                "target_abs_mean_error_max_m": 0.4,
                "target_abs_mean_error_finite_seed_count": 1,
                "validation_right_rmse_mean_m": 0.2,
                "validation_down_rmse_mean_m": 0.3,
                "validation_forward_rmse_mean_m": 0.7,
                "model_complexity_mean": 28,
                **module.method_contract_fields("sim3_piecewise_raw_forward_bins3"),
            },
        ]

        gate_rows = module.method_stability_gate_rows(method_rows, completed_seed_count=1)
        piecewise = next(row for row in gate_rows if row["method"] == "sim3_piecewise_raw_forward_bins3")

        self.assertEqual(piecewise["registration_model"], "piecewise_raw_forward_sim3_estimated_to_truth")
        self.assertEqual(piecewise["runtime_profile_family"], "estimated_to_truth_sim3")

    def test_method_stability_gate_fails_when_target_window_missing(self) -> None:
        module = load_multi_seed_module()
        method_rows = [
            {
                "method": "raw",
                "seed_count": 3,
                "selected_count": 0,
                "validation_range_rmse_mean_m": 5.0,
                "validation_range_rmse_mean_ci90_high_m": 5.0,
                "validation_range_rmse_max_m": 5.1,
                "target_abs_mean_error_mean_m": 2.0,
                "target_abs_mean_error_max_m": 2.0,
                "target_abs_mean_error_finite_seed_count": 2,
                "model_complexity_mean": 0,
            },
            {
                "method": "candidate",
                "seed_count": 3,
                "selected_count": 1,
                "validation_range_rmse_mean_m": 1.0,
                "validation_range_rmse_mean_ci90_high_m": 1.1,
                "validation_range_rmse_max_m": 1.2,
                "target_abs_mean_error_mean_m": 0.5,
                "target_abs_mean_error_max_m": 0.6,
                "target_abs_mean_error_finite_seed_count": 2,
                "model_complexity_mean": 4,
            },
        ]

        gate_rows = module.method_stability_gate_rows(method_rows, completed_seed_count=3)
        candidate = next(row for row in gate_rows if row["method"] == "candidate")

        self.assertFalse(candidate["strict_pass"])
        self.assertFalse(candidate["performance_pass"])
        self.assertFalse(candidate["target_window_coverage_ok"])
        self.assertIn("method_target_window_missing_in_some_seeds", candidate["reasons"])

    def test_choose_multi_seed_recommendation_prefers_strict_pass_score(self) -> None:
        module = load_multi_seed_module()
        method_gate_rows = [
            {
                "method": "simple",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 1.1,
                "method_validation_range_rmse_mean_ci90_high_m": 1.2,
                "method_validation_range_rmse_max_m": 1.2,
                "method_target_abs_mean_error_mean_m": 0.4,
                "method_target_abs_mean_error_max_m": 0.8,
                "model_complexity_mean": 1,
            },
            {
                "method": "complex",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 1.0,
                "method_validation_range_rmse_mean_ci90_high_m": 1.5,
                "method_validation_range_rmse_max_m": 1.5,
                "method_target_abs_mean_error_mean_m": 1.0,
                "method_target_abs_mean_error_max_m": 1.2,
                "model_complexity_mean": 20,
            },
        ]

        recommendation = module.choose_multi_seed_recommendation(method_gate_rows)

        self.assertEqual(recommendation["recommendation_mode"], "strict_pass")
        self.assertEqual(recommendation["method"], "simple")
        self.assertIn("target_max", recommendation["score_contract"])

    def test_choose_multi_seed_recommendation_penalizes_target_worst_case(self) -> None:
        module = load_multi_seed_module()
        method_gate_rows = [
            {
                "method": "lower_mean_worse_target_max",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 0.9,
                "method_validation_range_rmse_mean_ci90_high_m": 0.95,
                "method_validation_range_rmse_max_m": 1.0,
                "method_target_abs_mean_error_mean_m": 0.5,
                "method_target_abs_mean_error_max_m": 3.0,
                "model_complexity_mean": 3,
            },
            {
                "method": "slightly_higher_mean_stable_target",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 1.0,
                "method_validation_range_rmse_mean_ci90_high_m": 1.05,
                "method_validation_range_rmse_max_m": 1.1,
                "method_target_abs_mean_error_mean_m": 0.5,
                "method_target_abs_mean_error_max_m": 0.6,
                "model_complexity_mean": 3,
            },
        ]

        recommendation = module.choose_multi_seed_recommendation(method_gate_rows)

        self.assertEqual(recommendation["method"], "slightly_higher_mean_stable_target")

    def test_annotate_method_selection_scores_adds_rank_to_all_gate_rows(self) -> None:
        module = load_multi_seed_module()
        method_gate_rows = [
            {
                "method": "raw",
                "strict_pass": False,
                "performance_pass": False,
            },
            {
                "method": "stable_target",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 1.0,
                "method_validation_range_rmse_mean_ci90_high_m": 1.05,
                "method_validation_range_rmse_max_m": 1.1,
                "method_target_abs_mean_error_mean_m": 0.5,
                "method_target_abs_mean_error_max_m": 0.6,
                "model_complexity_mean": 3,
            },
            {
                "method": "higher_target_max",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 0.95,
                "method_validation_range_rmse_mean_ci90_high_m": 1.0,
                "method_validation_range_rmse_max_m": 1.1,
                "method_target_abs_mean_error_mean_m": 0.5,
                "method_target_abs_mean_error_max_m": 1.6,
                "model_complexity_mean": 3,
            },
        ]

        annotated = module.annotate_method_selection_scores(method_gate_rows)
        by_method = {row["method"]: row for row in annotated}

        self.assertEqual(by_method["stable_target"]["selection_rank"], 1)
        self.assertEqual(by_method["higher_target_max"]["selection_rank"], 2)
        self.assertEqual(by_method["raw"]["selection_rank"], "")
        self.assertEqual(by_method["stable_target"]["selection_score_contract"], module.SELECTION_SCORE_CONTRACT)
        self.assertLess(by_method["stable_target"]["selection_score"], by_method["higher_target_max"]["selection_score"])

    def test_selection_near_tie_rows_report_close_candidates(self) -> None:
        module = load_multi_seed_module()
        annotated = [
            {
                "method": "winner",
                "selection_rank": 1,
                "selection_score": 1.0,
                "strict_pass": True,
                "performance_pass": True,
                "registration_model": "sim3_estimated_to_truth",
                "residual_model": "none",
                "regularization_model": "none",
                "runtime_profile_family": "estimated_to_truth_sim3",
                "method_validation_range_rmse_mean_m": 0.8,
                "method_target_abs_mean_error_max_m": 0.5,
                "model_complexity_mean": 7,
                "selection_score_contract": module.SELECTION_SCORE_CONTRACT,
            },
            {
                "method": "near",
                "selection_rank": 2,
                "selection_score": 1.015,
                "strict_pass": True,
                "performance_pass": True,
                "registration_model": "raw_rdf_inverse_depth_ray_scale",
                "residual_model": "corrected_inverse_depth_linear",
                "regularization_model": "inverse_depth_residual",
                "runtime_profile_family": "estimated_to_truth_disparity_poly",
                "method_validation_range_rmse_mean_m": 0.82,
                "method_target_abs_mean_error_max_m": 0.45,
                "model_complexity_mean": 3,
                "selection_score_contract": module.SELECTION_SCORE_CONTRACT,
            },
            {
                "method": "far",
                "selection_rank": 6,
                "selection_score": 1.5,
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 0.9,
                "method_target_abs_mean_error_max_m": 0.6,
                "model_complexity_mean": 4,
                "selection_score_contract": module.SELECTION_SCORE_CONTRACT,
            },
        ]

        rows = module.selection_near_tie_rows(annotated, score_delta_threshold=0.02, max_rank=2)
        by_method = {row["method"]: row for row in rows}

        self.assertIn("winner", by_method)
        self.assertIn("near", by_method)
        self.assertNotIn("far", by_method)
        self.assertTrue(by_method["near"]["within_near_tie_threshold"])
        self.assertAlmostEqual(by_method["near"]["selection_score_delta_from_winner"], 0.015)
        self.assertAlmostEqual(by_method["near"]["target_abs_max_delta_from_winner_m"], -0.05)

    def test_choose_multi_seed_recommendation_reports_no_candidate(self) -> None:
        module = load_multi_seed_module()

        recommendation = module.choose_multi_seed_recommendation(
            [
                {
                    "method": "raw",
                    "strict_pass": False,
                    "performance_pass": False,
                }
            ]
        )

        self.assertEqual(recommendation["recommendation_mode"], "none")

    def test_multi_seed_acceptance_matrix_records_required_evidence(self) -> None:
        module = load_multi_seed_module()

        matrix = module.build_multi_seed_acceptance_matrix(
            seed_count=2,
            completed_seed_count=2,
            contract_summary={
                "loaded_contract_count": 2,
                "direct_axis_alias_forbidden_count": 2,
                "all_loaded_contracts_forbid_direct_axis_alias": True,
                "truth_rdf_transform_contract": module.TRUTH_RDF_TRANSFORM_CONTRACT,
            },
            method_rows=[
                {"method": "raw"},
                {"method": "sim3"},
            ],
            policy_gate_rows=[
                {"policy": "balanced", "strict_pass": True, "performance_pass": True},
            ],
            method_gate_rows=[
                {"method": "raw", "strict_pass": False, "performance_pass": False},
                {"method": "sim3", "strict_pass": True, "performance_pass": True},
            ],
            recommendation={"method": "sim3", "recommendation_mode": "strict_pass", "score": 1.0},
            recommended_profiles=[
                {"profile_exists": True},
                {"profile_exists": True},
            ],
            representative_checks={
                "pass_representative_profile_checks": True,
                "replay_pass": True,
                "promotion_pass": True,
                "validation_range_rmse_raw_m": 2.0,
                "validation_range_rmse_corrected_m": 1.0,
                "target_abs_mean_error_corrected_m": 0.5,
            },
            cross_seed_replay={
                "pass_cross_seed_replay": True,
                "seed_count": 2,
                "passed_seed_count": 2,
            },
        )

        self.assertEqual(sorted(matrix.keys()), sorted(module.MULTI_SEED_ACCEPTANCE_ITEMS))
        self.assertTrue(all(item["status"] == "PASS" for item in matrix.values()))
        self.assertEqual(matrix["coordinate_contract"]["status"], "PASS")
        self.assertEqual(matrix["recommended_runtime_profiles"]["evidence"]["profile_exists_count"], 2)
        self.assertEqual(matrix["representative_cross_seed_replay"]["evidence"]["passed_seed_count"], 2)

    def test_multi_seed_acceptance_matrix_exposes_replay_failure(self) -> None:
        module = load_multi_seed_module()

        matrix = module.build_multi_seed_acceptance_matrix(
            seed_count=2,
            completed_seed_count=1,
            contract_summary={
                "loaded_contract_count": 1,
                "direct_axis_alias_forbidden_count": 0,
                "all_loaded_contracts_forbid_direct_axis_alias": False,
                "truth_rdf_transform_contract": module.TRUTH_RDF_TRANSFORM_CONTRACT,
            },
            method_rows=[{"method": "raw"}],
            policy_gate_rows=[],
            method_gate_rows=[],
            recommendation={"method": "", "recommendation_mode": "none"},
            recommended_profiles=[],
            representative_checks={
                "pass_representative_profile_checks": False,
                "replay_pass": False,
                "promotion_pass": True,
            },
            cross_seed_replay={
                "pass_cross_seed_replay": False,
                "seed_count": 1,
                "passed_seed_count": 0,
            },
        )

        self.assertEqual(matrix["seed_run_coverage"]["status"], "FAIL")
        self.assertEqual(matrix["coordinate_contract"]["status"], "FAIL")
        self.assertEqual(matrix["multi_seed_recommendation"]["status"], "FAIL")
        self.assertEqual(matrix["representative_profile_checks"]["status"], "FAIL")
        self.assertEqual(matrix["representative_cross_seed_replay"]["status"], "FAIL")

    def test_method_family_diagnostics_groups_and_ranks_contract_families(self) -> None:
        module = load_multi_seed_module()
        method_gate_rows = [
            {
                "method": "raw",
                "strict_pass": False,
                "performance_pass": False,
            },
            {
                "method": "sim3_kernel_smoothed_residual_raw_forward_sigma060",
                "registration_model": "sim3_estimated_to_truth",
                "residual_model": "forward_raw_rdf_forward_lut",
                "regularization_model": "kernel_smoothing",
                "runtime_profile_family": "estimated_to_truth_sim3",
                "selection_note": "kernel residual",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 0.55,
                "method_target_abs_mean_error_max_m": 0.8,
                "model_complexity_mean": 19,
            },
            {
                "method": "sim3_kernel_smoothed_residual_raw_forward_sigma120",
                "registration_model": "sim3_estimated_to_truth",
                "residual_model": "forward_raw_rdf_forward_lut",
                "regularization_model": "kernel_smoothing",
                "runtime_profile_family": "estimated_to_truth_sim3",
                "selection_note": "kernel residual",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 0.60,
                "method_target_abs_mean_error_max_m": 0.7,
                "model_complexity_mean": 19,
            },
            {
                "method": "sim3",
                "registration_model": "sim3_estimated_to_truth",
                "residual_model": "none",
                "regularization_model": "none",
                "runtime_profile_family": "estimated_to_truth_sim3",
                "selection_note": "baseline sim3",
                "strict_pass": True,
                "performance_pass": True,
                "method_validation_range_rmse_mean_m": 0.75,
                "method_target_abs_mean_error_max_m": 0.5,
                "model_complexity_mean": 7,
            },
        ]

        families = module.method_family_diagnostic_rows(method_gate_rows)

        self.assertEqual(families[0]["family_rank"], 1)
        self.assertEqual(families[0]["regularization_model"], "kernel_smoothing")
        self.assertEqual(families[0]["method_count"], 2)
        self.assertEqual(families[0]["strict_pass_count"], 2)
        self.assertEqual(families[0]["best_method"], "sim3_kernel_smoothed_residual_raw_forward_sigma060")
        self.assertEqual(families[0]["best_validation_range_rmse_mean_m"], 0.55)
        top_methods = json.loads(families[0]["top_methods_json"])
        self.assertEqual(top_methods[0]["method"], "sim3_kernel_smoothed_residual_raw_forward_sigma060")

    def test_recommended_profile_rows_collects_seed_runtime_profiles(self) -> None:
        module = load_multi_seed_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            profile_path = output_dir / "seed_1" / "runtime_profiles" / "sim3.json"
            profile_path.parent.mkdir(parents=True)
            profile_path.write_text("{}", encoding="utf-8")
            rows = module.recommended_profile_rows(
                output_dir=output_dir,
                run_rows=[
                    {
                        "seed": 1,
                        "run_dir": str(output_dir / "seed_1"),
                        "pass_run": True,
                    }
                ],
                per_seed_method_rows=[
                    {
                        "seed": 1,
                        "method": "sim3",
                        "is_final_recommendation": True,
                        "validation_range_rmse_m": "1.0",
                        "validation_position_rmse_m": "1.1",
                        "validation_right_rmse_m": "0.2",
                        "validation_down_rmse_m": "0.3",
                        "validation_forward_rmse_m": "0.9",
                        "validation_target_abs_mean_error_m": "0.4",
                        "validation_target_window_count": "2",
                        "model_complexity": "7",
                    }
                ],
                recommendation={"method": "sim3"},
            )

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["profile_path"], "seed_1/runtime_profiles/sim3.json")
        self.assertTrue(rows[0]["profile_exists"])
        self.assertTrue(rows[0]["is_seed_final_recommendation"])
        self.assertEqual(rows[0]["axis_contract"], module.RDF_AXIS_CONTRACT)

    def test_near_tie_profile_candidate_rows_collects_ranked_runtime_profiles(self) -> None:
        module = load_multi_seed_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            profile_path = output_dir / "seed_4" / "runtime_profiles" / "sim3_shrunk.json"
            profile_path.parent.mkdir(parents=True)
            profile_path.write_text("{}", encoding="utf-8")
            rows = module.near_tie_profile_candidate_rows(
                output_dir=output_dir,
                run_rows=[
                    {
                        "seed": 4,
                        "run_dir": str(output_dir / "seed_4"),
                        "pass_run": True,
                    }
                ],
                per_seed_method_rows=[
                    {
                        "seed": 4,
                        "method": "sim3_shrunk",
                        "is_final_recommendation": False,
                        "validation_range_rmse_m": "0.8",
                        "validation_position_rmse_m": "0.9",
                        "validation_right_rmse_m": "0.2",
                        "validation_down_rmse_m": "0.3",
                        "validation_forward_rmse_m": "0.7",
                        "validation_target_abs_mean_error_m": "0.5",
                        "validation_target_window_count": "3",
                        "model_complexity": "8",
                    }
                ],
                near_tie_rows=[
                    {
                        "selection_rank": 2,
                        "method": "sim3_shrunk",
                        "selection_score": 1.25,
                        "selection_score_delta_from_winner": 0.03,
                        "within_near_tie_threshold": True,
                        "near_tie_score_delta_threshold": 0.05,
                        "registration_model": "sim3",
                        "residual_model": "target_weighted_quadratic_residual",
                        "regularization_model": "shrinkage",
                        "runtime_profile_family": "stereodistance_runtime_profile",
                    }
                ],
            )

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["profile_path"], "seed_4/runtime_profiles/sim3_shrunk.json")
        self.assertTrue(rows[0]["profile_exists"])
        self.assertEqual(rows[0]["selection_rank"], 2)
        self.assertEqual(rows[0]["selection_score_delta_from_winner"], 0.03)
        self.assertTrue(rows[0]["within_near_tie_threshold"])
        self.assertEqual(rows[0]["registration_model"], "sim3")
        self.assertEqual(rows[0]["runtime_input_contract"], "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth")

    def test_choose_representative_profile_prefers_mean_like_seed_profile(self) -> None:
        module = load_multi_seed_module()
        rows = [
            {
                "seed": 1,
                "method": "sim3",
                "profile_path": "seed_1/runtime_profiles/sim3.json",
                "profile_exists": True,
                "is_seed_final_recommendation": False,
                "validation_range_rmse_m": 0.4,
                "validation_position_rmse_m": 0.5,
                "validation_right_rmse_m": 0.1,
                "validation_down_rmse_m": 0.1,
                "validation_forward_rmse_m": 0.4,
                "validation_target_abs_mean_error_m": 0.1,
                "axis_contract": module.RDF_AXIS_CONTRACT,
                "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
            },
            {
                "seed": 2,
                "method": "sim3",
                "profile_path": "seed_2/runtime_profiles/sim3.json",
                "profile_exists": True,
                "is_seed_final_recommendation": False,
                "validation_range_rmse_m": 0.8,
                "validation_position_rmse_m": 0.9,
                "validation_right_rmse_m": 0.2,
                "validation_down_rmse_m": 0.2,
                "validation_forward_rmse_m": 0.8,
                "validation_target_abs_mean_error_m": 0.5,
                "axis_contract": module.RDF_AXIS_CONTRACT,
                "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
            },
            {
                "seed": 3,
                "method": "sim3",
                "profile_path": "seed_3/runtime_profiles/sim3.json",
                "profile_exists": True,
                "is_seed_final_recommendation": False,
                "validation_range_rmse_m": 1.2,
                "validation_position_rmse_m": 1.3,
                "validation_right_rmse_m": 0.3,
                "validation_down_rmse_m": 0.3,
                "validation_forward_rmse_m": 1.2,
                "validation_target_abs_mean_error_m": 0.9,
                "axis_contract": module.RDF_AXIS_CONTRACT,
                "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
            },
        ]

        representative = module.choose_representative_profile(
            rows,
            {
                "method": "sim3",
                "validation_range_rmse_mean_m": 0.8,
                "target_abs_mean_error_mean_m": 0.5,
            },
        )

        self.assertTrue(representative["selected"])
        self.assertEqual(representative["seed"], 2)
        self.assertEqual(representative["profile_path"], "seed_2/runtime_profiles/sim3.json")

    def test_representative_profile_checks_reports_no_selection(self) -> None:
        module = load_multi_seed_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            summary = module.run_representative_profile_checks(
                output_dir=Path(tmpdir),
                representative_profile={"selected": False},
                target_range_m=1852.0,
                target_window_m=30.0,
            )

        self.assertFalse(summary["pass_representative_profile_checks"])
        self.assertEqual(summary["reason"], "no_representative_profile_selected")

    def test_representative_cross_seed_replay_reports_no_selection(self) -> None:
        module = load_multi_seed_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            summary = module.run_representative_cross_seed_replay(
                output_dir=Path(tmpdir),
                run_rows=[],
                representative_profile={"selected": False},
                target_range_m=1852.0,
                target_window_m=30.0,
            )

        self.assertFalse(summary["pass_cross_seed_replay"])
        self.assertEqual(summary["reason"], "no_representative_profile_selected")
        self.assertEqual(summary["rows"], [])

    def test_multi_seed_plots_write_png_files(self) -> None:
        module = load_multi_seed_module()
        policy_rows = [
            {
                "policy": "balanced",
                "validation_range_rmse_mean_m": 1.0,
                "validation_range_rmse_mean_ci90_low_m": 0.9,
                "validation_range_rmse_mean_ci90_high_m": 1.1,
                "target_abs_mean_error_mean_m": 0.4,
                "target_abs_mean_error_mean_ci90_low_m": 0.3,
                "target_abs_mean_error_mean_ci90_high_m": 0.5,
            }
        ]
        gate_rows = [{"policy": "balanced", "strict_pass": True}]
        method_rows = [
            {
                "method": "sim3",
                "validation_range_rmse_mean_m": 1.0,
                "validation_range_rmse_mean_ci90_low_m": 0.9,
                "validation_range_rmse_mean_ci90_high_m": 1.1,
                "target_abs_mean_error_mean_m": 0.4,
            }
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            policy_png = tmp / "policy.png"
            method_png = tmp / "method.png"

            module.plot_policy_stability(policy_png, policy_rows, gate_rows)
            module.plot_method_aggregate(method_png, method_rows)

            self.assertTrue(policy_png.is_file())
            self.assertTrue(method_png.is_file())
            self.assertGreater(policy_png.stat().st_size, 0)
            self.assertGreater(method_png.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
