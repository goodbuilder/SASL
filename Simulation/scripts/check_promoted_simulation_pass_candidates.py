#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path
from typing import Mapping


ROOT_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = ROOT_DIR.parent
PACKAGE_SCRIPT = ROOT_DIR / "scripts" / "run_observation_fusion_deployment_package.py"
ROBUSTNESS_SCRIPT = ROOT_DIR / "scripts" / "run_rtk_stereo_robustness_sweep.py"
DEFAULT_TIER_SPEC = ROOT_DIR / "config" / "observation_fusion_fresh_deployment_tiers.json"
DEFAULT_RESIDUAL_PROFILE = ROOT_DIR / "config" / "disparity_residual_profile_disparity_noise_stress.json"
DEFAULT_PROMOTED_MANIFEST = ROOT_DIR / "config" / "promoted_simulation_pass_candidates.json"
DEFAULT_ORIGIN_REPROJECTION_REPORT = (
    ROOT_DIR
    / "reports"
    / "origin_bbox_reprojection_validation_attitude_aware_component_gains_longprofile_right_att_0p1_20260609_seed20260625"
)
DEFAULT_SMALL_DRONE_REPORT = (
    ROOT_DIR / "reports" / "experiments" / "EXP-20260617-213-small-drone-observation-fusion"
)
SMALL_DRONE_PROMOTED_METHODS = [
    "small_drone_multiburst_observation_inner_median_outer_median_b1x3_trim0p2_common0_sim3",
    "small_drone_multiburst_observation_inner_median_outer_median_b3x5_trim0p2_common0_affine_ridge",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Re-run repo-local gates for promoted Simulation PASS candidates.",
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--tier-spec-json", type=Path, default=DEFAULT_TIER_SPEC)
    parser.add_argument("--disparity-residual-profile-json", type=Path, default=DEFAULT_RESIDUAL_PROFILE)
    parser.add_argument("--promoted-manifest-json", type=Path, default=DEFAULT_PROMOTED_MANIFEST)
    parser.add_argument("--origin-reprojection-report-dir", type=Path, default=DEFAULT_ORIGIN_REPROJECTION_REPORT)
    parser.add_argument("--small-drone-report-dir", type=Path, default=DEFAULT_SMALL_DRONE_REPORT)
    parser.add_argument("--skip-origin-reprojection-report", action="store_true")
    parser.add_argument("--skip-small-drone-report", action="store_true")
    parser.add_argument("--skip-residual", action="store_true")
    parser.add_argument(
        "--include-non-alias-smoke",
        action="store_true",
        help="Also re-run the six-scenario non-alias robustness catalog smoke.",
    )
    parser.add_argument("--non-alias-seeds", default="282010")
    parser.add_argument("--non-alias-sample-count", type=int, default=80)
    parser.add_argument("--non-alias-target-oversample-count", type=int, default=12)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def float_value(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def run_origin_reprojection_report_check(report_dir: Path) -> dict[str, object]:
    result_path = report_dir / "result.txt"
    summary_path = report_dir / "validation_gate_summary.csv"
    result_text = result_path.read_text(encoding="utf-8") if result_path.is_file() else ""
    rows: list[dict[str, str]] = []
    if summary_path.is_file():
        with summary_path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    return {
        "report_dir": str(report_dir),
        "result_path": str(result_path),
        "summary_path": str(summary_path),
        "result_text": result_text,
        "rows": rows,
    }


def evaluate_origin_reprojection_report(result: Mapping[str, object]) -> dict[str, object]:
    failures: list[str] = []
    result_text = str(result.get("result_text", ""))
    rows = result.get("rows", [])
    if not isinstance(rows, list):
        rows = []
    require("final_pass=yes" in result_text, failures, "origin_reprojection:final_pass")
    require(len(rows) >= 5, failures, "origin_reprojection:trial_count")
    pass_count = 0
    min_right = float("inf")
    min_down = float("inf")
    min_forward = float("inf")
    sample_count = 0
    for row in rows:
        if not isinstance(row, Mapping):
            continue
        if str(row.get("pass", "")).strip().lower() == "yes":
            pass_count += 1
        sample_count += int(float_value(row.get("samples")))
        min_right = min(min_right, float_value(row.get("right_rmse_improvement_m"), default=-1.0))
        min_down = min(min_down, float_value(row.get("down_rmse_improvement_m"), default=-1.0))
        min_forward = min(min_forward, float_value(row.get("forward_rmse_improvement_m"), default=-1.0))
    require(pass_count == len(rows) and pass_count >= 5, failures, "origin_reprojection:trial_pass_count")
    require(min_right > 0.0, failures, "origin_reprojection:right_improvement")
    require(min_down > 0.0, failures, "origin_reprojection:down_improvement")
    require(min_forward > 0.0, failures, "origin_reprojection:forward_improvement")
    return {
        "status": "PASS" if not failures else "FAIL",
        "failure_count": len(failures),
        "failures": failures,
        "acceptance_status": "PASS" if "final_pass=yes" in result_text else "FAIL",
        "trial_count": len(rows),
        "pass_count": pass_count,
        "sample_count": sample_count,
        "min_right_improvement_m": None if min_right == float("inf") else min_right,
        "min_down_improvement_m": None if min_down == float("inf") else min_down,
        "min_forward_improvement_m": None if min_forward == float("inf") else min_forward,
        "fusion_status": "not_applicable",
        "disparity_residual_status": "not_applicable",
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def compact_origin_reprojection_result(result: Mapping[str, object]) -> dict[str, object]:
    rows = result.get("rows", [])
    return {
        "report_dir": result.get("report_dir", ""),
        "result_path": result.get("result_path", ""),
        "summary_path": result.get("summary_path", ""),
        "summary": {
            "row_count": len(rows) if isinstance(rows, list) else 0,
            "final_pass": "final_pass=yes" in str(result.get("result_text", "")),
        },
    }


def run_small_drone_report_check(report_dir: Path, method: str) -> dict[str, object]:
    metrics_path = report_dir / "small_drone_observation_metrics.json"
    summary_path = report_dir / "small_drone_observation_method_summary.csv"
    metrics = load_json(metrics_path) if metrics_path.is_file() else {}
    rows: list[dict[str, str]] = []
    if summary_path.is_file():
        with summary_path.open(newline="", encoding="utf-8") as handle:
            rows = [row for row in csv.DictReader(handle) if row.get("method") == method]
    return {
        "report_dir": str(report_dir),
        "metrics_path": str(metrics_path),
        "summary_path": str(summary_path),
        "method": method,
        "metrics": metrics,
        "rows": rows,
    }


def evaluate_small_drone_report(result: Mapping[str, object]) -> dict[str, object]:
    failures: list[str] = []
    metrics = result.get("metrics", {})
    if not isinstance(metrics, Mapping):
        metrics = {}
    rows = result.get("rows", [])
    if not isinstance(rows, list):
        rows = []
    method = str(result.get("method", ""))

    expected_scenarios = {"small_drone_close_2m_4x", "small_drone_mid_5m_8x"}
    seen_scenarios: set[str] = set()
    sample_count = 0
    min_range_improvement = float("inf")
    min_position_improvement = float("inf")
    min_right_improvement = float("inf")
    min_down_improvement = float("inf")
    min_forward_improvement = float("inf")
    max_target_ratio = 0.0
    pass_count = 0
    trial_count = 0

    require(metrics.get("coordinate_contracts_explicit") is True, failures, "small_drone:coordinate_contract")
    require(metrics.get("runtime_truth_used") is False, failures, "small_drone:runtime_truth")
    require(
        metrics.get("rdf_axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)",
        failures,
        "small_drone:rdf_axis_contract",
    )
    require(
        metrics.get("public_topic_contract") == "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        failures,
        "small_drone:public_topic_contract",
    )

    for row in rows:
        if not isinstance(row, Mapping):
            continue
        scenario = str(row.get("scenario", ""))
        if scenario in expected_scenarios:
            seen_scenarios.add(scenario)
        row_pass = int(float_value(row.get("pass_count")))
        row_trials = int(float_value(row.get("trial_count")))
        pass_count += row_pass
        trial_count += row_trials
        sample_count += row_trials * int(float_value(metrics.get("sample_count")))
        require(row.get("status") == "STRICT_PASS", failures, f"small_drone:{scenario}:status")
        require(row_pass == row_trials and row_trials == 50, failures, f"small_drone:{scenario}:pass_count")
        require(str(row.get("runtime_truth_used", "")).lower() == "false", failures, f"small_drone:{scenario}:runtime_truth")

        range_improvement = float_value(row.get("range_rmse_raw_mean_m")) - float_value(
            row.get("range_rmse_corrected_mean_m")
        )
        position_improvement = float_value(row.get("position_rmse_raw_mean_m")) - float_value(
            row.get("position_rmse_corrected_mean_m")
        )
        right_improvement = float_value(row.get("right_rmse_raw_mean_m")) - float_value(
            row.get("right_rmse_corrected_mean_m")
        )
        down_improvement = float_value(row.get("down_rmse_raw_mean_m")) - float_value(
            row.get("down_rmse_corrected_mean_m")
        )
        forward_improvement = float_value(row.get("forward_rmse_raw_mean_m")) - float_value(
            row.get("forward_rmse_corrected_mean_m")
        )
        min_range_improvement = min(min_range_improvement, range_improvement)
        min_position_improvement = min(min_position_improvement, position_improvement)
        min_right_improvement = min(min_right_improvement, right_improvement)
        min_down_improvement = min(min_down_improvement, down_improvement)
        min_forward_improvement = min(min_forward_improvement, forward_improvement)
        target_limit = float_value(row.get("target_error_limit_m"))
        target_max = float_value(row.get("target_max_abs_error_corrected_max_m"))
        if target_limit > 0.0:
            max_target_ratio = max(max_target_ratio, target_max / target_limit)
        require(target_max < target_limit, failures, f"small_drone:{scenario}:target_limit")

    require(seen_scenarios == expected_scenarios, failures, "small_drone:scenario_coverage")
    require(len(rows) == 2, failures, "small_drone:method_row_count")
    require(min_range_improvement > 0.0, failures, "small_drone:range_improvement")
    require(min_position_improvement > 0.0, failures, "small_drone:position_improvement")
    require(min_right_improvement > 0.0, failures, "small_drone:right_improvement")
    require(min_down_improvement > 0.0, failures, "small_drone:down_improvement")
    require(min_forward_improvement > 0.0, failures, "small_drone:forward_improvement")

    return {
        "status": "PASS" if not failures else "FAIL",
        "failure_count": len(failures),
        "failures": failures,
        "method": method,
        "acceptance_status": "STRICT_PASS" if not failures else "FAIL",
        "scenario_count": len(seen_scenarios),
        "pass_count": pass_count,
        "trial_count": trial_count,
        "sample_count": sample_count,
        "min_range_improvement_m": None if min_range_improvement == float("inf") else min_range_improvement,
        "min_position_improvement_m": None if min_position_improvement == float("inf") else min_position_improvement,
        "min_right_improvement_m": None if min_right_improvement == float("inf") else min_right_improvement,
        "min_down_improvement_m": None if min_down_improvement == float("inf") else min_down_improvement,
        "min_forward_improvement_m": None if min_forward_improvement == float("inf") else min_forward_improvement,
        "max_target_error_to_limit_ratio": max_target_ratio,
        "fusion_status": "small_drone_observation_fusion",
        "disparity_residual_status": "not_applicable",
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def compact_small_drone_result(result: Mapping[str, object]) -> dict[str, object]:
    rows = result.get("rows", [])
    metrics = result.get("metrics", {})
    if not isinstance(metrics, Mapping):
        metrics = {}
    return {
        "report_dir": result.get("report_dir", ""),
        "metrics_path": result.get("metrics_path", ""),
        "summary_path": result.get("summary_path", ""),
        "method": result.get("method", ""),
        "summary": {
            "row_count": len(rows) if isinstance(rows, list) else 0,
            "coordinate_contracts_explicit": metrics.get("coordinate_contracts_explicit", False),
            "runtime_truth_used": metrics.get("runtime_truth_used", True),
            "rdf_axis_contract": metrics.get("rdf_axis_contract", ""),
            "public_topic_contract": metrics.get("public_topic_contract", ""),
        },
    }


def run_package_check(
    *,
    output_dir: Path,
    tier_spec_json: Path,
    residual_profile_json: Path | None,
) -> dict[str, object]:
    command = [
        sys.executable,
        str(PACKAGE_SCRIPT),
        "--output-dir",
        str(output_dir),
        "--tier-spec-json",
        str(tier_spec_json),
    ]
    if residual_profile_json is not None:
        command.extend(["--disparity-residual-profile-json", str(residual_profile_json)])
    result = subprocess.run(command, cwd=REPO_DIR, check=False, text=True, capture_output=True)
    summary_path = output_dir / "observation_fusion_deployment_package_summary.json"
    summary = load_json(summary_path) if summary_path.is_file() else {}
    return {
        "command": " ".join(command),
        "returncode": result.returncode,
        "stdout_tail": result.stdout[-4000:],
        "stderr_tail": result.stderr[-4000:],
        "summary_path": str(summary_path),
        "summary": summary,
    }


def evaluate_package_result(
    result: Mapping[str, object],
    *,
    expected_residual_status_prefix: str | None,
) -> dict[str, object]:
    failures: list[str] = []
    summary = result.get("summary", {})
    if not isinstance(summary, Mapping):
        summary = {}
    require(result.get("returncode") == 0, failures, "command:returncode")
    require(summary.get("acceptance_status") == "PASS", failures, "package:acceptance_status")
    require(summary.get("default_tier") == "fresh_common_mode_b7x3_aggregate_best", failures, "package:default_tier")
    require(
        summary.get("stereodistance_node_smoke_fusion_status") == "fused:MultiBurstObservationFusion:21",
        failures,
        "node:fusion_status",
    )
    require(summary.get("operator_view_package_launch_status") == "PASS", failures, "operator:package_launch")
    require(summary.get("runtime_truth_used") is False, failures, "contract:runtime_truth_used")
    require(summary.get("coordinate_contracts_explicit") is True, failures, "contract:coordinate_contracts_explicit")
    residual_status = str(summary.get("stereodistance_node_smoke_disparity_residual_status", ""))
    if expected_residual_status_prefix is None:
        require(residual_status == "disabled", failures, "residual:expected_disabled")
    else:
        require(
            residual_status.startswith(expected_residual_status_prefix),
            failures,
            "residual:expected_applied",
        )
    return {
        "status": "PASS" if not failures else "FAIL",
        "failure_count": len(failures),
        "failures": failures,
        "acceptance_status": summary.get("acceptance_status", ""),
        "default_tier": summary.get("default_tier", ""),
        "fusion_status": summary.get("stereodistance_node_smoke_fusion_status", ""),
        "disparity_residual_status": residual_status,
        "runtime_truth_used": summary.get("runtime_truth_used", True),
        "coordinate_contracts_explicit": summary.get("coordinate_contracts_explicit", False),
    }


def run_non_alias_smoke(
    *,
    output_dir: Path,
    seeds: str,
    sample_count: int,
    target_oversample_count: int,
) -> dict[str, object]:
    command = [
        sys.executable,
        str(ROBUSTNESS_SCRIPT),
        "--output-dir",
        str(output_dir),
        "--seeds",
        seeds,
        "--sample-count",
        str(sample_count),
        "--synthetic-target-oversample-count",
        str(target_oversample_count),
        "--pooled-candidate-scan-limit",
        "2",
        "--skip-near-tie-transfer",
    ]
    result = subprocess.run(command, cwd=REPO_DIR, check=False, text=True, capture_output=True)
    summary_path = output_dir / "robustness_summary.json"
    summary = load_json(summary_path) if summary_path.is_file() else {}
    return {
        "command": " ".join(command),
        "returncode": result.returncode,
        "stdout_tail": result.stdout[-4000:],
        "stderr_tail": result.stderr[-4000:],
        "summary_path": str(summary_path),
        "summary": summary,
    }


def evaluate_non_alias_result(result: Mapping[str, object]) -> dict[str, object]:
    failures: list[str] = []
    summary = result.get("summary", {})
    if not isinstance(summary, Mapping):
        summary = {}
    coordinate_summary = summary.get("coordinate_contract_summary", {})
    if not isinstance(coordinate_summary, Mapping):
        coordinate_summary = {}
    deployment_strategy = summary.get("deployment_strategy_recommendation", {})
    if not isinstance(deployment_strategy, Mapping):
        deployment_strategy = {}
    selector_package = summary.get("runtime_label_selector_package", {})
    if not isinstance(selector_package, Mapping):
        selector_package = {}

    require(result.get("returncode") == 0, failures, "command:returncode")
    require(summary.get("robustness_acceptance_pass") is True, failures, "robustness:acceptance")
    require(
        coordinate_summary.get("all_scenarios_forbid_direct_axis_alias") is True,
        failures,
        "contract:direct_axis_alias",
    )
    require(
        deployment_strategy.get("deployment_strategy") == "scenario_profile_catalog",
        failures,
        "deployment:strategy",
    )
    require(deployment_strategy.get("single_profile_pass") is False, failures, "deployment:single_profile_rejected")
    require(selector_package.get("status") == "PASS", failures, "selector_package:status")
    require(selector_package.get("frontier_status") == "PASS", failures, "selector_package:frontier")
    require(selector_package.get("operator_launch_status") == "PASS", failures, "selector_package:operator_launch")
    require(selector_package.get("runtime_truth_used") is False, failures, "selector_package:runtime_truth")
    return {
        "status": "PASS" if not failures else "FAIL",
        "failure_count": len(failures),
        "failures": failures,
        "acceptance_status": summary.get("robustness_acceptance_pass", False),
        "deployment_strategy": deployment_strategy.get("deployment_strategy", ""),
        "fusion_status": "not_applicable",
        "disparity_residual_status": "not_applicable",
        "selector_package_status": selector_package.get("status", ""),
        "operator_launch_status": selector_package.get("operator_launch_status", ""),
        "runtime_truth_used": selector_package.get("runtime_truth_used", True),
        "coordinate_contracts_explicit": coordinate_summary.get("all_scenarios_forbid_direct_axis_alias", False),
    }


def compact_package_result(result: Mapping[str, object]) -> dict[str, object]:
    summary = result.get("summary", {})
    if not isinstance(summary, Mapping):
        summary = {}
    return {
        "command": result.get("command", ""),
        "returncode": result.get("returncode", -1),
        "summary_path": result.get("summary_path", ""),
        "summary": {
            "acceptance_status": summary.get("acceptance_status", ""),
            "default_tier": summary.get("default_tier", ""),
            "stereodistance_node_smoke_fusion_status": summary.get(
                "stereodistance_node_smoke_fusion_status", ""
            ),
            "stereodistance_node_smoke_disparity_residual_status": summary.get(
                "stereodistance_node_smoke_disparity_residual_status", ""
            ),
            "operator_view_package_launch_status": summary.get("operator_view_package_launch_status", ""),
            "runtime_truth_used": summary.get("runtime_truth_used", True),
            "coordinate_contracts_explicit": summary.get("coordinate_contracts_explicit", False),
            "inventory_file_count": summary.get("inventory_file_count", 0),
        },
    }


def compact_non_alias_result(result: Mapping[str, object]) -> dict[str, object]:
    summary = result.get("summary", {})
    if not isinstance(summary, Mapping):
        summary = {}
    coordinate_summary = summary.get("coordinate_contract_summary", {})
    if not isinstance(coordinate_summary, Mapping):
        coordinate_summary = {}
    deployment_strategy = summary.get("deployment_strategy_recommendation", {})
    if not isinstance(deployment_strategy, Mapping):
        deployment_strategy = {}
    selector_package = summary.get("runtime_label_selector_package", {})
    if not isinstance(selector_package, Mapping):
        selector_package = {}
    return {
        "command": result.get("command", ""),
        "returncode": result.get("returncode", -1),
        "summary_path": result.get("summary_path", ""),
        "summary": {
            "robustness_acceptance_pass": summary.get("robustness_acceptance_pass", False),
            "deployment_strategy": deployment_strategy.get("deployment_strategy", ""),
            "single_profile_pass": deployment_strategy.get("single_profile_pass", True),
            "scenario_catalog_profiles": deployment_strategy.get("scenario_catalog_profiles", 0),
            "scenario_catalog_promotion_pass_count": deployment_strategy.get(
                "scenario_catalog_promotion_pass_count", 0
            ),
            "selector_package_status": selector_package.get("status", ""),
            "frontier_status": selector_package.get("frontier_status", ""),
            "operator_launch_status": selector_package.get("operator_launch_status", ""),
            "operator_launch_pass_count": selector_package.get("operator_launch_pass_count", 0),
            "operator_launch_check_count": selector_package.get("operator_launch_check_count", 0),
            "runtime_truth_used": selector_package.get("runtime_truth_used", True),
            "all_scenarios_forbid_direct_axis_alias": coordinate_summary.get(
                "all_scenarios_forbid_direct_axis_alias", False
            ),
        },
    }


def write_outputs(output_dir: Path, payload: Mapping[str, object]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "promoted_simulation_pass_candidates_check.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    lines = [
        "Promoted Simulation PASS candidates check",
        f"status: {payload.get('status', '')}",
        f"candidate_count: {payload.get('candidate_count', 0)}",
        f"pass_count: {payload.get('pass_count', 0)}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    for row in payload.get("candidates", []):
        if not isinstance(row, Mapping):
            continue
        evaluation = row.get("evaluation", {})
        if not isinstance(evaluation, Mapping):
            evaluation = {}
        lines.append(
            "- {name}: status={status}, fusion={fusion}, residual={residual}".format(
                name=row.get("name", ""),
                status=evaluation.get("status", ""),
                fusion=evaluation.get("fusion_status", ""),
                residual=evaluation.get("disparity_residual_status", ""),
            )
        )
    (output_dir / "promoted_simulation_pass_candidates_check.txt").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)
    tier_spec_json = Path(args.tier_spec_json)
    residual_profile_json = None if args.skip_residual else Path(args.disparity_residual_profile_json)
    candidates: list[dict[str, object]] = []

    if not args.skip_origin_reprojection_report:
        origin_result = run_origin_reprojection_report_check(Path(args.origin_reprojection_report_dir))
        origin_evaluation = evaluate_origin_reprojection_report(origin_result)
        candidates.append(
            {
                "name": "origin_bbox_reprojection_grid_v1_component_gains_range_only_forward_fusion",
                "result": compact_origin_reprojection_result(origin_result),
                "evaluation": origin_evaluation,
            }
        )

    if not args.skip_small_drone_report:
        for method in SMALL_DRONE_PROMOTED_METHODS:
            small_drone_result = run_small_drone_report_check(Path(args.small_drone_report_dir), method)
            small_drone_evaluation = evaluate_small_drone_report(small_drone_result)
            candidates.append(
                {
                    "name": method,
                    "result": compact_small_drone_result(small_drone_result),
                    "evaluation": small_drone_evaluation,
                }
            )

    base_dir = output_dir / "observation_fusion"
    base_result = run_package_check(
        output_dir=base_dir,
        tier_spec_json=tier_spec_json,
        residual_profile_json=None,
    )
    base_evaluation = evaluate_package_result(base_result, expected_residual_status_prefix=None)
    candidates.append(
        {
            "name": "fresh_common_mode_b7x3_aggregate_best",
            "result": compact_package_result(base_result),
            "evaluation": base_evaluation,
        }
    )

    if residual_profile_json is not None:
        residual_dir = output_dir / "observation_fusion_with_disparity_residual"
        residual_result = run_package_check(
            output_dir=residual_dir,
            tier_spec_json=tier_spec_json,
            residual_profile_json=residual_profile_json,
        )
        residual_evaluation = evaluate_package_result(
            residual_result,
            expected_residual_status_prefix="applied:",
        )
        candidates.append(
            {
                "name": "fresh_common_mode_b7x3_aggregate_best+disparity_noise_stress_residual",
                "result": compact_package_result(residual_result),
                "evaluation": residual_evaluation,
            }
        )

    if args.include_non_alias_smoke:
        non_alias_result = run_non_alias_smoke(
            output_dir=output_dir / "non_alias_robustness_catalog",
            seeds=str(args.non_alias_seeds),
            sample_count=int(args.non_alias_sample_count),
            target_oversample_count=int(args.non_alias_target_oversample_count),
        )
        non_alias_evaluation = evaluate_non_alias_result(non_alias_result)
        candidates.append(
            {
                "name": "non_alias_scenario_profile_catalog_label_selector",
                "result": compact_non_alias_result(non_alias_result),
                "evaluation": non_alias_evaluation,
            }
        )

    failure_count = sum(
        int(candidate["evaluation"]["failure_count"])
        for candidate in candidates
        if isinstance(candidate.get("evaluation"), Mapping)
    )
    pass_count = sum(
        1
        for candidate in candidates
        if isinstance(candidate.get("evaluation"), Mapping)
        and candidate["evaluation"].get("status") == "PASS"
    )
    payload = {
        "mode": "promoted_simulation_pass_candidates_check",
        "status": "PASS" if failure_count == 0 and pass_count == len(candidates) else "FAIL",
        "candidate_count": len(candidates),
        "pass_count": pass_count,
        "failure_count": failure_count,
        "tier_spec_json": str(tier_spec_json),
        "disparity_residual_profile_json": "" if residual_profile_json is None else str(residual_profile_json),
        "promoted_manifest_json": str(Path(args.promoted_manifest_json)),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "candidates": candidates,
    }
    write_outputs(output_dir, payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
