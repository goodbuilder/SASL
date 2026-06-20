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
