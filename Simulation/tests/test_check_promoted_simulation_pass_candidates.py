from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_promoted_simulation_pass_candidates.py"
PROMOTED_MANIFEST_PATH = ROOT_DIR / "config" / "promoted_simulation_pass_candidates.json"


def load_module():
    spec = importlib.util.spec_from_file_location("check_promoted_simulation_pass_candidates_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_promoted_simulation_pass_candidates.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_evaluate_package_result_accepts_base_observation_fusion_pass() -> None:
    module = load_module()
    result = {
        "returncode": 0,
        "summary": {
            "acceptance_status": "PASS",
            "default_tier": "fresh_common_mode_b7x3_aggregate_best",
            "stereodistance_node_smoke_fusion_status": "fused:MultiBurstObservationFusion:21",
            "stereodistance_node_smoke_disparity_residual_status": "disabled",
            "operator_view_package_launch_status": "PASS",
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": True,
        },
    }

    evaluation = module.evaluate_package_result(result, expected_residual_status_prefix=None)

    assert evaluation["status"] == "PASS"
    assert evaluation["failure_count"] == 0


def test_evaluate_origin_reprojection_report_requires_all_axis_improvements() -> None:
    module = load_module()
    result = {
        "result_text": "final_pass=yes\n",
        "rows": [
            {
                "samples": "10",
                "pass": "yes",
                "right_rmse_improvement_m": "0.1",
                "down_rmse_improvement_m": "0.2",
                "forward_rmse_improvement_m": "1.0",
            }
            for _ in range(5)
        ],
    }

    evaluation = module.evaluate_origin_reprojection_report(result)

    assert evaluation["status"] == "PASS"
    assert evaluation["pass_count"] == 5
    assert evaluation["min_forward_improvement_m"] == 1.0


def test_evaluate_origin_reprojection_report_rejects_axis_regression() -> None:
    module = load_module()
    rows = [
        {
            "samples": "10",
            "pass": "yes",
            "right_rmse_improvement_m": "0.1",
            "down_rmse_improvement_m": "0.2",
            "forward_rmse_improvement_m": "1.0",
        }
        for _ in range(5)
    ]
    rows[2]["down_rmse_improvement_m"] = "-0.01"
    result = {
        "result_text": "final_pass=yes\n",
        "rows": rows,
    }

    evaluation = module.evaluate_origin_reprojection_report(result)

    assert evaluation["status"] == "FAIL"
    assert "origin_reprojection:down_improvement" in evaluation["failures"]


def test_evaluate_small_drone_report_accepts_strict_all_axis_pass() -> None:
    module = load_module()
    result = {
        "method": "small_drone_multiburst_observation_inner_median_outer_median_b1x3_trim0p2_common0_sim3",
        "metrics": {
            "sample_count": 360,
            "coordinate_contracts_explicit": True,
            "runtime_truth_used": False,
            "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        },
        "rows": [
            {
                "scenario": "small_drone_close_2m_4x",
                "status": "STRICT_PASS",
                "pass_count": "50",
                "trial_count": "50",
                "runtime_truth_used": "False",
                "range_rmse_raw_mean_m": "0.27",
                "range_rmse_corrected_mean_m": "0.06",
                "position_rmse_raw_mean_m": "0.31",
                "position_rmse_corrected_mean_m": "0.07",
                "right_rmse_raw_mean_m": "0.14",
                "right_rmse_corrected_mean_m": "0.01",
                "down_rmse_raw_mean_m": "0.05",
                "down_rmse_corrected_mean_m": "0.01",
                "forward_rmse_raw_mean_m": "0.28",
                "forward_rmse_corrected_mean_m": "0.06",
                "target_max_abs_error_corrected_max_m": "0.16",
                "target_error_limit_m": "1.5",
            },
            {
                "scenario": "small_drone_mid_5m_8x",
                "status": "STRICT_PASS",
                "pass_count": "50",
                "trial_count": "50",
                "runtime_truth_used": "False",
                "range_rmse_raw_mean_m": "0.61",
                "range_rmse_corrected_mean_m": "0.10",
                "position_rmse_raw_mean_m": "0.76",
                "position_rmse_corrected_mean_m": "0.11",
                "right_rmse_raw_mean_m": "0.42",
                "right_rmse_corrected_mean_m": "0.02",
                "down_rmse_raw_mean_m": "0.17",
                "down_rmse_corrected_mean_m": "0.01",
                "forward_rmse_raw_mean_m": "0.62",
                "forward_rmse_corrected_mean_m": "0.10",
                "target_max_abs_error_corrected_max_m": "0.30",
                "target_error_limit_m": "3.0",
            },
        ],
    }

    evaluation = module.evaluate_small_drone_report(result)

    assert evaluation["status"] == "PASS"
    assert evaluation["scenario_count"] == 2
    assert evaluation["pass_count"] == 100
    assert evaluation["sample_count"] == 36000


def test_evaluate_small_drone_report_rejects_axis_regression() -> None:
    module = load_module()
    result = {
        "method": "small_drone_multiburst_observation_inner_median_outer_median_b1x3_trim0p2_common0_sim3",
        "metrics": {
            "sample_count": 360,
            "coordinate_contracts_explicit": True,
            "runtime_truth_used": False,
            "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        },
        "rows": [
            {
                "scenario": "small_drone_close_2m_4x",
                "status": "STRICT_PASS",
                "pass_count": "50",
                "trial_count": "50",
                "runtime_truth_used": "False",
                "range_rmse_raw_mean_m": "0.27",
                "range_rmse_corrected_mean_m": "0.06",
                "position_rmse_raw_mean_m": "0.31",
                "position_rmse_corrected_mean_m": "0.07",
                "right_rmse_raw_mean_m": "0.14",
                "right_rmse_corrected_mean_m": "0.15",
                "down_rmse_raw_mean_m": "0.05",
                "down_rmse_corrected_mean_m": "0.01",
                "forward_rmse_raw_mean_m": "0.28",
                "forward_rmse_corrected_mean_m": "0.06",
                "target_max_abs_error_corrected_max_m": "0.16",
                "target_error_limit_m": "1.5",
            },
            {
                "scenario": "small_drone_mid_5m_8x",
                "status": "STRICT_PASS",
                "pass_count": "50",
                "trial_count": "50",
                "runtime_truth_used": "False",
                "range_rmse_raw_mean_m": "0.61",
                "range_rmse_corrected_mean_m": "0.10",
                "position_rmse_raw_mean_m": "0.76",
                "position_rmse_corrected_mean_m": "0.11",
                "right_rmse_raw_mean_m": "0.42",
                "right_rmse_corrected_mean_m": "0.02",
                "down_rmse_raw_mean_m": "0.17",
                "down_rmse_corrected_mean_m": "0.01",
                "forward_rmse_raw_mean_m": "0.62",
                "forward_rmse_corrected_mean_m": "0.10",
                "target_max_abs_error_corrected_max_m": "0.30",
                "target_error_limit_m": "3.0",
            },
        ],
    }

    evaluation = module.evaluate_small_drone_report(result)

    assert evaluation["status"] == "FAIL"
    assert "small_drone:right_improvement" in evaluation["failures"]


def test_evaluate_bbox_outlier_report_accepts_strict_pass() -> None:
    module = load_module()
    result = {
        "method": "repeated_observation_median_n7_trim0p2",
        "metrics": {
            "coordinate_contracts_explicit": True,
            "runtime_truth_used": False,
        },
        "rows": [
            {
                "status": "STRICT_PASS",
                "pass_count": "50",
                "trial_count": "50",
                "runtime_truth_used": "False",
                "range_rmse_raw_mean_m": "2.65",
                "range_rmse_corrected_mean_m": "0.35",
                "position_rmse_raw_mean_m": "5.09",
                "position_rmse_corrected_mean_m": "0.37",
                "target_max_abs_error_corrected_max_m": "2.15",
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            }
        ],
    }

    evaluation = module.evaluate_bbox_outlier_report(result)

    assert evaluation["status"] == "PASS"
    assert evaluation["pass_count"] == 50
    assert evaluation["target_max_abs_error_m"] == 2.15


def test_evaluate_bbox_outlier_report_rejects_target_gate_failure() -> None:
    module = load_module()
    result = {
        "method": "repeated_observation_median_n7_trim0p2",
        "metrics": {
            "coordinate_contracts_explicit": True,
            "runtime_truth_used": False,
        },
        "rows": [
            {
                "status": "STRICT_PASS",
                "pass_count": "50",
                "trial_count": "50",
                "runtime_truth_used": "False",
                "range_rmse_raw_mean_m": "2.65",
                "range_rmse_corrected_mean_m": "0.35",
                "position_rmse_raw_mean_m": "5.09",
                "position_rmse_corrected_mean_m": "0.37",
                "target_max_abs_error_corrected_max_m": "10.01",
                "rdf_axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
                "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            }
        ],
    }

    evaluation = module.evaluate_bbox_outlier_report(result)

    assert evaluation["status"] == "FAIL"
    assert "bbox_outlier:target_gate_10m" in evaluation["failures"]


def test_evaluate_package_result_rejects_missing_residual_apply() -> None:
    module = load_module()
    result = {
        "returncode": 0,
        "summary": {
            "acceptance_status": "PASS",
            "default_tier": "fresh_common_mode_b7x3_aggregate_best",
            "stereodistance_node_smoke_fusion_status": "fused:MultiBurstObservationFusion:21",
            "stereodistance_node_smoke_disparity_residual_status": "disabled",
            "operator_view_package_launch_status": "PASS",
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": True,
        },
    }

    evaluation = module.evaluate_package_result(result, expected_residual_status_prefix="applied:")

    assert evaluation["status"] == "FAIL"
    assert "residual:expected_applied" in evaluation["failures"]


def test_write_outputs_records_candidate_status(tmp_path: Path) -> None:
    module = load_module()
    payload = {
        "status": "PASS",
        "candidate_count": 1,
        "pass_count": 1,
        "failure_count": 0,
        "candidates": [
            {
                "name": "candidate",
                "evaluation": {
                    "status": "PASS",
                    "fusion_status": "fused:MultiBurstObservationFusion:21",
                    "disparity_residual_status": "disabled",
                },
            }
        ],
    }

    module.write_outputs(tmp_path, payload)

    saved = json.loads((tmp_path / "promoted_simulation_pass_candidates_check.json").read_text(encoding="utf-8"))
    text = (tmp_path / "promoted_simulation_pass_candidates_check.txt").read_text(encoding="utf-8")
    assert saved["status"] == "PASS"
    assert "candidate: status=PASS" in text


def test_promoted_manifest_declares_checker_candidate_sets() -> None:
    manifest = json.loads(PROMOTED_MANIFEST_PATH.read_text(encoding="utf-8"))
    candidates = manifest["candidates"]
    default_candidates = [row for row in candidates if row.get("checker_included_by_default") is True]
    optional_candidates = [row for row in candidates if row.get("checker_included_by_default") is False]

    assert manifest["hardware_validated_complete_all_axis"] is False
    assert len(default_candidates) == 19
    assert len(optional_candidates) == 1
    assert {row["id"] for row in default_candidates} == {
        "origin_bbox_reprojection_grid_v1_component_gains_range_only_forward_fusion",
        "small_drone_multiburst_observation_inner_median_outer_median_b1x3_trim0p2_common0_sim3",
        "small_drone_multiburst_observation_inner_median_outer_median_b3x5_trim0p2_common0_affine_ridge",
        "repeated_observation_median_n3_trim0p2",
        "repeated_observation_median_n7_trim0p2",
        "repeated_observation_trimmed_mean_n15_trim0p2",
        "repeated_observation_trimmed_mean_n15_trim0p1",
        "repeated_observation_trimmed_mean_n15_trim0p3",
        "repeated_observation_trimmed_mean_n15_trim0p4",
        "repeated_observation_trimmed_mean_n21_trim0p2",
        "repeated_observation_trimmed_mean_n31_trim0p2",
        "repeated_observation_trimmed_mean_n51_trim0p2",
        "repeated_observation_trimmed_mean_n75_trim0p2",
        "correlated_repeated_observation_trimmed_mean_n15_trim0p2_common0",
        "correlated_repeated_observation_trimmed_mean_n31_trim0p2_common0",
        "multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b5x5_trim0p2_common0p25",
        "multiburst_observation_inner_trimmed_mean_outer_median_b3x5_trim0p2_common1",
        "fresh_common_mode_b7x3_aggregate_best",
        "fresh_common_mode_b7x3_aggregate_best_disparity_noise_residual",
    }
    assert optional_candidates[0]["id"] == "non_alias_scenario_profile_catalog_label_selector"
    assert optional_candidates[0]["checker_option"] == "--include-non-alias-smoke"


def test_evaluate_non_alias_result_accepts_catalog_selector_pass() -> None:
    module = load_module()
    result = {
        "returncode": 0,
        "summary": {
            "robustness_acceptance_pass": True,
            "coordinate_contract_summary": {
                "all_scenarios_forbid_direct_axis_alias": True,
            },
            "deployment_strategy_recommendation": {
                "deployment_strategy": "scenario_profile_catalog",
                "single_profile_pass": False,
            },
            "runtime_label_selector_package": {
                "status": "PASS",
                "frontier_status": "PASS",
                "operator_launch_status": "PASS",
                "runtime_truth_used": False,
            },
        },
    }

    evaluation = module.evaluate_non_alias_result(result)

    assert evaluation["status"] == "PASS"
    assert evaluation["failure_count"] == 0


def test_evaluate_non_alias_result_rejects_single_profile_strategy() -> None:
    module = load_module()
    result = {
        "returncode": 0,
        "summary": {
            "robustness_acceptance_pass": True,
            "coordinate_contract_summary": {
                "all_scenarios_forbid_direct_axis_alias": True,
            },
            "deployment_strategy_recommendation": {
                "deployment_strategy": "single_runtime_profile",
                "single_profile_pass": True,
            },
            "runtime_label_selector_package": {
                "status": "PASS",
                "frontier_status": "PASS",
                "operator_launch_status": "PASS",
                "runtime_truth_used": False,
            },
        },
    }

    evaluation = module.evaluate_non_alias_result(result)

    assert evaluation["status"] == "FAIL"
    assert "deployment:strategy" in evaluation["failures"]
    assert "deployment:single_profile_rejected" in evaluation["failures"]
