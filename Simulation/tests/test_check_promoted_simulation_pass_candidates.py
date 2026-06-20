from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "check_promoted_simulation_pass_candidates.py"


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
