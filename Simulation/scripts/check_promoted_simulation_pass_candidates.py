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
DEFAULT_BBOX_ANCHOR_FULLSCREEN_REPORT = (
    ROOT_DIR / "reports" / "bbox_anchor_global_fullscreen_right_att_0p1_seed20260625"
)
DEFAULT_ORIGIN_BBOX_OUTPUT_REPORT = (
    ROOT_DIR / "reports" / "origin_bbox_output_validation_8trials_slow_20260609_1530_seed20260616"
)
DEFAULT_SMALL_DRONE_REPORT = (
    ROOT_DIR / "reports" / "experiments" / "EXP-20260617-213-small-drone-observation-fusion"
)
DEFAULT_MULTIBURST_RESIDUAL_AGGREGATE_REPORT = (
    ROOT_DIR
    / "reports"
    / "experiments"
    / "EXP-20260618-281-fresh-stereodistance-disparity-residual-search"
    / "aggregate_summary"
)
DEFAULT_SCENARIO_RESIDUAL_CATALOG_DIR = (
    ROOT_DIR
    / "reports"
    / "experiments"
    / "EXP-20260618-281-fresh-stereodistance-disparity-residual-search"
    / "runtime_export_scenario_best"
    / "scenario_residual_catalog"
)
SMALL_DRONE_PROMOTED_METHODS = [
    "small_drone_multiburst_observation_inner_median_outer_median_b1x3_trim0p2_common0_sim3",
    "small_drone_multiburst_observation_inner_median_outer_median_b3x5_trim0p2_common0_affine_ridge",
]
BBOX_OUTLIER_PROMOTED_METHODS = [
    {
        "method": "repeated_observation_median_n3_trim0p2",
        "report_dir": ROOT_DIR
        / "reports"
        / "experiments"
        / "EXP-20260617-201-bbox-outlier-repeated-observation-fusion",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_median_n7_trim0p2",
        "report_dir": ROOT_DIR
        / "reports"
        / "experiments"
        / "EXP-20260617-201-bbox-outlier-repeated-observation-fusion",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_trimmed_mean_n15_trim0p2",
        "report_dir": ROOT_DIR
        / "reports"
        / "experiments"
        / "EXP-20260617-202-bbox-outlier-repeated-observation-fusion-extended",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_trimmed_mean_n15_trim0p1",
        "report_dir": ROOT_DIR / "reports" / "experiments" / "EXP-20260617-203-bbox-outlier-n15-trim0p1",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_trimmed_mean_n15_trim0p3",
        "report_dir": ROOT_DIR / "reports" / "experiments" / "EXP-20260617-204-bbox-outlier-n15-trim0p3",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_trimmed_mean_n15_trim0p4",
        "report_dir": ROOT_DIR / "reports" / "experiments" / "EXP-20260617-205-bbox-outlier-n15-trim0p4",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_trimmed_mean_n21_trim0p2",
        "report_dir": ROOT_DIR / "reports" / "experiments" / "EXP-20260617-206-bbox-outlier-n21-trim0p2",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_trimmed_mean_n31_trim0p2",
        "report_dir": ROOT_DIR / "reports" / "experiments" / "EXP-20260617-207-bbox-outlier-n31-trim0p2",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_trimmed_mean_n51_trim0p2",
        "report_dir": ROOT_DIR / "reports" / "experiments" / "EXP-20260617-208-bbox-outlier-n51-trim0p2",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "repeated_observation_trimmed_mean_n75_trim0p2",
        "report_dir": ROOT_DIR / "reports" / "experiments" / "EXP-20260617-209-bbox-outlier-n75-trim0p2",
        "summary_csv": "repeated_observation_method_summary.csv",
        "metrics_json": "repeated_observation_metrics.json",
    },
    {
        "method": "correlated_repeated_observation_trimmed_mean_n15_trim0p2_common0",
        "report_dir": ROOT_DIR
        / "reports"
        / "experiments"
        / "EXP-20260617-210-bbox-outlier-correlated-repeated-observation",
        "summary_csv": "correlated_repeated_observation_method_summary.csv",
        "metrics_json": "correlated_repeated_observation_metrics.json",
    },
    {
        "method": "correlated_repeated_observation_trimmed_mean_n31_trim0p2_common0",
        "report_dir": ROOT_DIR
        / "reports"
        / "experiments"
        / "EXP-20260617-210-bbox-outlier-correlated-repeated-observation",
        "summary_csv": "correlated_repeated_observation_method_summary.csv",
        "metrics_json": "correlated_repeated_observation_metrics.json",
    },
    {
        "method": "multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b5x5_trim0p2_common0p25",
        "report_dir": ROOT_DIR
        / "reports"
        / "experiments"
        / "EXP-20260617-211-bbox-outlier-multiburst-common-mode-fusion",
        "summary_csv": "multiburst_observation_method_summary.csv",
        "metrics_json": "multiburst_observation_metrics.json",
    },
    {
        "method": "multiburst_observation_inner_trimmed_mean_outer_median_b3x5_trim0p2_common1",
        "report_dir": ROOT_DIR
        / "reports"
        / "experiments"
        / "EXP-20260617-212-bbox-outlier-multiburst-common1-confirmation",
        "summary_csv": "multiburst_observation_method_summary.csv",
        "metrics_json": "multiburst_observation_metrics.json",
    },
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
    parser.add_argument("--bbox-anchor-fullscreen-report-dir", type=Path, default=DEFAULT_BBOX_ANCHOR_FULLSCREEN_REPORT)
    parser.add_argument("--origin-bbox-output-report-dir", type=Path, default=DEFAULT_ORIGIN_BBOX_OUTPUT_REPORT)
    parser.add_argument("--small-drone-report-dir", type=Path, default=DEFAULT_SMALL_DRONE_REPORT)
    parser.add_argument(
        "--multiburst-residual-aggregate-report-dir",
        type=Path,
        default=DEFAULT_MULTIBURST_RESIDUAL_AGGREGATE_REPORT,
    )
    parser.add_argument(
        "--scenario-residual-catalog-dir",
        type=Path,
        default=DEFAULT_SCENARIO_RESIDUAL_CATALOG_DIR,
    )
    parser.add_argument("--skip-origin-reprojection-report", action="store_true")
    parser.add_argument("--skip-bbox-anchor-fullscreen-report", action="store_true")
    parser.add_argument("--skip-origin-bbox-output-report", action="store_true")
    parser.add_argument("--skip-small-drone-report", action="store_true")
    parser.add_argument("--skip-bbox-outlier-report", action="store_true")
    parser.add_argument("--skip-multiburst-residual-aggregate-report", action="store_true")
    parser.add_argument("--skip-scenario-residual-catalog", action="store_true")
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


def run_bbox_anchor_fullscreen_report_check(report_dir: Path) -> dict[str, object]:
    result_path = report_dir / "result.txt"
    summary_path = report_dir / "validation_gate_summary.csv"
    profile_path = report_dir / "bbox_anchor_profile.json"
    result_text = result_path.read_text(encoding="utf-8") if result_path.is_file() else ""
    rows: list[dict[str, str]] = []
    if summary_path.is_file():
        with summary_path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    profile = load_json(profile_path) if profile_path.is_file() else {}
    return {
        "report_dir": str(report_dir),
        "result_path": str(result_path),
        "summary_path": str(summary_path),
        "profile_path": str(profile_path),
        "result_text": result_text,
        "rows": rows,
        "profile": profile,
    }


def evaluate_bbox_anchor_fullscreen_report(result: Mapping[str, object]) -> dict[str, object]:
    failures: list[str] = []
    result_text = str(result.get("result_text", ""))
    rows = result.get("rows", [])
    if not isinstance(rows, list):
        rows = []
    profile = result.get("profile", {})
    if not isinstance(profile, Mapping):
        profile = {}

    require("final_pass=yes" in result_text, failures, "bbox_anchor_fullscreen:final_pass")
    require(profile.get("method") == "bbox_internal_origin_anchor_profile_v1", failures, "bbox_anchor_fullscreen:profile_method")
    require(str(profile.get("application_contract", "")).startswith("runtime uses bbox"), failures, "bbox_anchor_fullscreen:application_contract")
    require(len(rows) >= 5, failures, "bbox_anchor_fullscreen:trial_count")
    pass_count = 0
    sample_count = 0
    min_right = float("inf")
    min_down = float("inf")
    min_forward = float("inf")
    min_position = float("inf")
    for row in rows:
        if not isinstance(row, Mapping):
            continue
        if str(row.get("pass", "")).strip().lower() == "yes":
            pass_count += 1
        sample_count += int(float_value(row.get("sample_count")))
        min_right = min(min_right, float_value(row.get("right_rmse_improvement_m"), default=-1.0))
        min_down = min(min_down, float_value(row.get("down_rmse_improvement_m"), default=-1.0))
        min_forward = min(min_forward, float_value(row.get("forward_rmse_improvement_m"), default=-1.0))
        min_position = min(min_position, float_value(row.get("position_rmse_improvement_m"), default=-1.0))
    require(pass_count == len(rows) and pass_count >= 5, failures, "bbox_anchor_fullscreen:trial_pass_count")
    require(min_right > 0.0, failures, "bbox_anchor_fullscreen:right_improvement")
    require(min_down > 0.0, failures, "bbox_anchor_fullscreen:down_improvement")
    require(min_forward > 0.0, failures, "bbox_anchor_fullscreen:forward_improvement")
    require(min_position > 0.0, failures, "bbox_anchor_fullscreen:position_improvement")
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
        "min_position_improvement_m": None if min_position == float("inf") else min_position,
        "fusion_status": "bbox_anchor_fullscreen_saved_report",
        "disparity_residual_status": "not_applicable",
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def compact_bbox_anchor_fullscreen_result(result: Mapping[str, object]) -> dict[str, object]:
    rows = result.get("rows", [])
    return {
        "report_dir": result.get("report_dir", ""),
        "result_path": result.get("result_path", ""),
        "summary_path": result.get("summary_path", ""),
        "profile_path": result.get("profile_path", ""),
        "summary": {
            "row_count": len(rows) if isinstance(rows, list) else 0,
            "final_pass": "final_pass=yes" in str(result.get("result_text", "")),
        },
    }


def run_origin_bbox_output_report_check(report_dir: Path) -> dict[str, object]:
    result_path = report_dir / "result.txt"
    summary_path = report_dir / "summary.csv"
    metadata_path = report_dir / "metadata.json"
    result_text = result_path.read_text(encoding="utf-8") if result_path.is_file() else ""
    rows: list[dict[str, str]] = []
    if summary_path.is_file():
        with summary_path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    metadata = load_json(metadata_path) if metadata_path.is_file() else {}
    return {
        "report_dir": str(report_dir),
        "result_path": str(result_path),
        "summary_path": str(summary_path),
        "metadata_path": str(metadata_path),
        "result_text": result_text,
        "rows": rows,
        "metadata": metadata,
    }


def evaluate_origin_bbox_output_report(result: Mapping[str, object]) -> dict[str, object]:
    failures: list[str] = []
    result_text = str(result.get("result_text", ""))
    rows = result.get("rows", [])
    if not isinstance(rows, list):
        rows = []
    metadata = result.get("metadata", {})
    if not isinstance(metadata, Mapping):
        metadata = {}

    require("final_pass=yes" in result_text, failures, "origin_bbox_output:final_pass")
    require(metadata.get("axis_contract") == "stereo_center_frd:forward_right_down", failures, "origin_bbox_output:axis_contract")
    require("strictly lower" in str(metadata.get("pass_rule", "")), failures, "origin_bbox_output:pass_rule")
    require(len(rows) >= 3, failures, "origin_bbox_output:trial_count")
    pass_count = 0
    sample_count = 0
    min_right = float("inf")
    min_down = float("inf")
    min_forward = float("inf")
    for row in rows:
        if not isinstance(row, Mapping):
            continue
        if str(row.get("pass", "")).strip().lower() == "yes":
            pass_count += 1
        sample_count += int(float_value(row.get("samples")))
        min_right = min(min_right, float_value(row.get("right_improvement_m"), default=-1.0))
        min_down = min(min_down, float_value(row.get("down_improvement_m"), default=-1.0))
        min_forward = min(min_forward, float_value(row.get("forward_improvement_m"), default=-1.0))
    require(pass_count == len(rows) and pass_count >= 3, failures, "origin_bbox_output:trial_pass_count")
    require(min_right > 0.0, failures, "origin_bbox_output:right_improvement")
    require(min_down > 0.0, failures, "origin_bbox_output:down_improvement")
    require(min_forward > 0.0, failures, "origin_bbox_output:forward_improvement")
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
        "fusion_status": "origin_bbox_output_saved_report",
        "disparity_residual_status": "not_applicable",
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def compact_origin_bbox_output_result(result: Mapping[str, object]) -> dict[str, object]:
    rows = result.get("rows", [])
    metadata = result.get("metadata", {})
    if not isinstance(metadata, Mapping):
        metadata = {}
    return {
        "report_dir": result.get("report_dir", ""),
        "result_path": result.get("result_path", ""),
        "summary_path": result.get("summary_path", ""),
        "metadata_path": result.get("metadata_path", ""),
        "summary": {
            "row_count": len(rows) if isinstance(rows, list) else 0,
            "final_pass": "final_pass=yes" in str(result.get("result_text", "")),
            "axis_contract": metadata.get("axis_contract", ""),
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


def run_bbox_outlier_report_check(config: Mapping[str, object]) -> dict[str, object]:
    report_dir = Path(config["report_dir"])
    method = str(config["method"])
    metrics_path = report_dir / str(config["metrics_json"])
    summary_path = report_dir / str(config["summary_csv"])
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


def evaluate_bbox_outlier_report(result: Mapping[str, object]) -> dict[str, object]:
    failures: list[str] = []
    metrics = result.get("metrics", {})
    if not isinstance(metrics, Mapping):
        metrics = {}
    rows = result.get("rows", [])
    if not isinstance(rows, list):
        rows = []
    row = rows[0] if rows and isinstance(rows[0], Mapping) else {}

    require(metrics.get("coordinate_contracts_explicit") is True, failures, "bbox_outlier:coordinate_contract")
    require(metrics.get("runtime_truth_used") is False, failures, "bbox_outlier:runtime_truth")
    require(
        row.get("rdf_axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)",
        failures,
        "bbox_outlier:rdf_axis_contract",
    )
    require(
        row.get("public_topic_contract") == "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        failures,
        "bbox_outlier:public_topic_contract",
    )
    require(len(rows) == 1, failures, "bbox_outlier:method_row_count")
    require(row.get("status") == "STRICT_PASS", failures, "bbox_outlier:status")
    pass_count = int(float_value(row.get("pass_count")))
    trial_count = int(float_value(row.get("trial_count")))
    require(pass_count == trial_count and trial_count == 50, failures, "bbox_outlier:pass_count")
    require(str(row.get("runtime_truth_used", "")).lower() == "false", failures, "bbox_outlier:row_runtime_truth")
    range_improvement = float_value(row.get("range_rmse_raw_mean_m")) - float_value(
        row.get("range_rmse_corrected_mean_m")
    )
    position_improvement = float_value(row.get("position_rmse_raw_mean_m")) - float_value(
        row.get("position_rmse_corrected_mean_m")
    )
    target_max = float_value(row.get("target_max_abs_error_corrected_max_m"))
    require(range_improvement > 0.0, failures, "bbox_outlier:range_improvement")
    require(position_improvement > 0.0, failures, "bbox_outlier:position_improvement")
    require(target_max < 10.0, failures, "bbox_outlier:target_gate_10m")
    return {
        "status": "PASS" if not failures else "FAIL",
        "failure_count": len(failures),
        "failures": failures,
        "method": result.get("method", ""),
        "acceptance_status": "STRICT_PASS" if not failures else "FAIL",
        "pass_count": pass_count,
        "trial_count": trial_count,
        "range_improvement_m": range_improvement,
        "position_improvement_m": position_improvement,
        "target_max_abs_error_m": target_max,
        "fusion_status": "bbox_outlier_observation_fusion",
        "disparity_residual_status": "not_applicable",
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def compact_bbox_outlier_result(result: Mapping[str, object]) -> dict[str, object]:
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
        },
    }


def run_multiburst_residual_aggregate_report_check(report_dir: Path) -> dict[str, object]:
    summary_path = report_dir / "multiburst_disparity_residual_summary.json"
    scenario_best_path = report_dir / "multiburst_disparity_residual_scenario_best.csv"
    summary = load_json(summary_path) if summary_path.is_file() else {}
    rows: list[dict[str, str]] = []
    if scenario_best_path.is_file():
        with scenario_best_path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    return {
        "report_dir": str(report_dir),
        "summary_path": str(summary_path),
        "scenario_best_path": str(scenario_best_path),
        "summary": summary,
        "rows": rows,
    }


def evaluate_multiburst_residual_aggregate_report(result: Mapping[str, object]) -> dict[str, object]:
    failures: list[str] = []
    summary = result.get("summary", {})
    if not isinstance(summary, Mapping):
        summary = {}
    best = summary.get("best_method", {})
    if not isinstance(best, Mapping):
        best = {}
    rows = result.get("rows", [])
    if not isinstance(rows, list):
        rows = []

    require(summary.get("status") == "PASS", failures, "multiburst_residual:summary_status")
    require(summary.get("scenario_count") == 4, failures, "multiburst_residual:scenario_count")
    require(summary.get("runtime_truth_used") is False, failures, "multiburst_residual:runtime_truth")
    require(best.get("all_scenarios_strict_pass") is True, failures, "multiburst_residual:all_scenarios_strict_pass")
    require(int(float_value(best.get("strict_pass_scenario_count"))) == 4, failures, "multiburst_residual:strict_pass_count")
    require(
        best.get("rdf_axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)",
        failures,
        "multiburst_residual:rdf_axis_contract",
    )
    require(
        best.get("public_topic_contract") == "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        failures,
        "multiburst_residual:public_topic_contract",
    )
    require(best.get("runtime_truth_used") is False, failures, "multiburst_residual:best_runtime_truth")
    observation_target_worst = float_value(best.get("target_max_abs_error_observation_worst_m"))
    residual_target_worst = float_value(best.get("target_max_abs_error_residual_worst_m"))
    observation_range_mean = float_value(best.get("range_rmse_observation_mean_across_scenarios_m"))
    residual_range_mean = float_value(best.get("range_rmse_residual_mean_across_scenarios_m"))
    observation_position_mean = float_value(best.get("position_rmse_observation_mean_across_scenarios_m"))
    residual_position_mean = float_value(best.get("position_rmse_residual_mean_across_scenarios_m"))
    require(residual_target_worst < observation_target_worst, failures, "multiburst_residual:target_worst_improvement")
    require(residual_target_worst < 10.0, failures, "multiburst_residual:target_gate_10m")
    require(residual_range_mean < observation_range_mean, failures, "multiburst_residual:range_improvement")
    require(residual_position_mean < observation_position_mean, failures, "multiburst_residual:position_improvement")

    scenario_pass_count = 0
    for row in rows:
        if not isinstance(row, Mapping):
            continue
        if row.get("status") == "STRICT_PASS":
            scenario_pass_count += 1
        require(str(row.get("runtime_truth_used", "")).lower() == "false", failures, "multiburst_residual:row_runtime_truth")
        require(
            row.get("rdf_axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)",
            failures,
            "multiburst_residual:row_rdf_axis_contract",
        )
        require(
            row.get("public_topic_contract") == "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            failures,
            "multiburst_residual:row_public_topic_contract",
        )
    require(len(rows) == 4 and scenario_pass_count == 4, failures, "multiburst_residual:scenario_rows")

    return {
        "status": "PASS" if not failures else "FAIL",
        "failure_count": len(failures),
        "failures": failures,
        "method": best.get("method", ""),
        "acceptance_status": summary.get("status", ""),
        "scenario_count": summary.get("scenario_count", 0),
        "strict_pass_scenario_count": best.get("strict_pass_scenario_count", 0),
        "target_max_abs_error_residual_worst_m": residual_target_worst,
        "range_rmse_residual_mean_across_scenarios_m": residual_range_mean,
        "position_rmse_residual_mean_across_scenarios_m": residual_position_mean,
        "fusion_status": "multiburst_disparity_residual_aggregate",
        "disparity_residual_status": "aggregate_saved_report",
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def compact_multiburst_residual_aggregate_result(result: Mapping[str, object]) -> dict[str, object]:
    summary = result.get("summary", {})
    if not isinstance(summary, Mapping):
        summary = {}
    best = summary.get("best_method", {})
    if not isinstance(best, Mapping):
        best = {}
    rows = result.get("rows", [])
    return {
        "report_dir": result.get("report_dir", ""),
        "summary_path": result.get("summary_path", ""),
        "scenario_best_path": result.get("scenario_best_path", ""),
        "summary": {
            "status": summary.get("status", ""),
            "scenario_count": summary.get("scenario_count", 0),
            "row_count": len(rows) if isinstance(rows, list) else 0,
            "best_method": best.get("method", ""),
            "runtime_truth_used": summary.get("runtime_truth_used", True),
        },
    }


def run_scenario_residual_catalog_check(catalog_dir: Path) -> dict[str, object]:
    selector_path = catalog_dir / "disparity_residual_profile_label_selector.json"
    catalog_csv_path = catalog_dir / "disparity_residual_profile_catalog.csv"
    runtime_export_summary_path = catalog_dir.parent / "runtime_export_summary.json"
    operator_launch_path = catalog_dir.parent / "operator_view_launch_check_disparity_noise.json"
    selector = load_json(selector_path) if selector_path.is_file() else {}
    runtime_export_summary = load_json(runtime_export_summary_path) if runtime_export_summary_path.is_file() else {}
    operator_launch = load_json(operator_launch_path) if operator_launch_path.is_file() else {}
    catalog_rows: list[dict[str, str]] = []
    if catalog_csv_path.is_file():
        with catalog_csv_path.open(newline="", encoding="utf-8") as handle:
            catalog_rows = list(csv.DictReader(handle))

    profile_paths = selector.get("profile_paths", {})
    if not isinstance(profile_paths, Mapping):
        profile_paths = {}
    selections: dict[str, object] = {}
    profiles: dict[str, object] = {}
    for label, profile_rel in profile_paths.items():
        label_text = str(label)
        selection_path = catalog_dir / f"selection_{label_text}.json"
        profile_path = catalog_dir / str(profile_rel)
        selections[label_text] = load_json(selection_path) if selection_path.is_file() else {}
        profiles[label_text] = load_json(profile_path) if profile_path.is_file() else {}

    return {
        "catalog_dir": str(catalog_dir),
        "selector_path": str(selector_path),
        "catalog_csv_path": str(catalog_csv_path),
        "runtime_export_summary_path": str(runtime_export_summary_path),
        "operator_launch_path": str(operator_launch_path),
        "selector": selector,
        "runtime_export_summary": runtime_export_summary,
        "operator_launch": operator_launch,
        "catalog_rows": catalog_rows,
        "selections": selections,
        "profiles": profiles,
    }


def evaluate_scenario_residual_catalog(result: Mapping[str, object]) -> dict[str, object]:
    failures: list[str] = []
    selector = result.get("selector", {})
    if not isinstance(selector, Mapping):
        selector = {}
    runtime_export_summary = result.get("runtime_export_summary", {})
    if not isinstance(runtime_export_summary, Mapping):
        runtime_export_summary = {}
    operator_launch = result.get("operator_launch", {})
    if not isinstance(operator_launch, Mapping):
        operator_launch = {}
    catalog_rows = result.get("catalog_rows", [])
    if not isinstance(catalog_rows, list):
        catalog_rows = []
    selections = result.get("selections", {})
    if not isinstance(selections, Mapping):
        selections = {}
    profiles = result.get("profiles", {})
    if not isinstance(profiles, Mapping):
        profiles = {}

    expected_labels = {
        "bbox_outlier_stress",
        "combined_sensor_stress",
        "disparity_noise_stress",
        "intrinsic_scale_stress",
    }
    profile_paths = selector.get("profile_paths", {})
    if not isinstance(profile_paths, Mapping):
        profile_paths = {}

    require(selector.get("status") == "PASS", failures, "scenario_residual_catalog:selector_status")
    require(
        selector.get("mode") == "stereodistance_disparity_residual_profile_label_selector",
        failures,
        "scenario_residual_catalog:selector_mode",
    )
    require(selector.get("runtime_truth_used") is False, failures, "scenario_residual_catalog:selector_runtime_truth")
    require(
        selector.get("coordinate_contracts_explicit") is True,
        failures,
        "scenario_residual_catalog:selector_coordinate_contract",
    )
    require(
        selector.get("axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)",
        failures,
        "scenario_residual_catalog:selector_axis_contract",
    )
    require(
        selector.get("public_topic_contract") == "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        failures,
        "scenario_residual_catalog:selector_public_topic_contract",
    )
    require(
        set(selector.get("selection_input_fields", [])) == {"deployment_label", "scenario_label", "scenario_name"},
        failures,
        "scenario_residual_catalog:selection_inputs",
    )
    forbidden_fields = set(selector.get("forbidden_runtime_fields", []))
    require(
        {"px4_local_position", "rtk_truth", "calibration_truth", "validation_error"}.issubset(forbidden_fields),
        failures,
        "scenario_residual_catalog:forbidden_runtime_fields",
    )
    require(set(profile_paths.keys()) == expected_labels, failures, "scenario_residual_catalog:profile_labels")
    require(runtime_export_summary.get("status") == "PASS", failures, "scenario_residual_catalog:runtime_summary_status")
    require(
        runtime_export_summary.get("runtime_truth_used") is False,
        failures,
        "scenario_residual_catalog:runtime_summary_truth",
    )
    require(
        runtime_export_summary.get("coordinate_contracts_explicit") is True,
        failures,
        "scenario_residual_catalog:runtime_summary_coordinate_contract",
    )
    require(
        int(float_value(runtime_export_summary.get("check_count"))) == 20
        and int(float_value(runtime_export_summary.get("pass_count"))) == 20,
        failures,
        "scenario_residual_catalog:runtime_summary_pass_count",
    )
    require(operator_launch.get("status") == "PASS", failures, "scenario_residual_catalog:operator_launch_status")
    require(
        operator_launch.get("residual_label_selection_status") == "PASS",
        failures,
        "scenario_residual_catalog:operator_residual_selection_status",
    )
    require(
        operator_launch.get("residual_selected_label") == "disparity_noise_stress",
        failures,
        "scenario_residual_catalog:operator_residual_label",
    )
    require(
        operator_launch.get("disparity_residual_arg_present") is True,
        failures,
        "scenario_residual_catalog:operator_residual_arg",
    )
    require(operator_launch.get("runtime_truth_used") is False, failures, "scenario_residual_catalog:operator_truth")
    require(len(catalog_rows) == 4, failures, "scenario_residual_catalog:catalog_row_count")

    selected_count = 0
    for label in sorted(expected_labels):
        selection = selections.get(label, {})
        if not isinstance(selection, Mapping):
            selection = {}
        profile = profiles.get(label, {})
        if not isinstance(profile, Mapping):
            profile = {}
        row = next((row for row in catalog_rows if isinstance(row, Mapping) and row.get("profile_label") == label), {})
        require(selection.get("status") == "PASS", failures, f"scenario_residual_catalog:{label}:selection_status")
        require(selection.get("selected_label") == label, failures, f"scenario_residual_catalog:{label}:selected_label")
        require(
            selection.get("label_selection_applied") is True,
            failures,
            f"scenario_residual_catalog:{label}:selection_applied",
        )
        require(selection.get("runtime_truth_used") is False, failures, f"scenario_residual_catalog:{label}:runtime_truth")
        require(profile.get("profile_type") == "DisparityResidualRuntimeProfile", failures, f"scenario_residual_catalog:{label}:profile_type")
        require(profile.get("runtime_truth_used") is False, failures, f"scenario_residual_catalog:{label}:profile_runtime_truth")
        require(
            profile.get("runtime_input_contract", "").startswith("exported residual profile"),
            failures,
            f"scenario_residual_catalog:{label}:runtime_input_contract",
        )
        require(
            profile.get("rdf_axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)",
            failures,
            f"scenario_residual_catalog:{label}:rdf_axis_contract",
        )
        require(
            profile.get("public_topic_contract") == "stereodistance_position_topics:FRD_range(forward,right,down,range)",
            failures,
            f"scenario_residual_catalog:{label}:public_topic_contract",
        )
        require(
            str(row.get("runtime_truth_used", "")).lower() == "false",
            failures,
            f"scenario_residual_catalog:{label}:catalog_runtime_truth",
        )
        if selection.get("status") == "PASS":
            selected_count += 1

    return {
        "status": "PASS" if not failures else "FAIL",
        "failure_count": len(failures),
        "failures": failures,
        "acceptance_status": selector.get("status", ""),
        "scenario_count": len(expected_labels),
        "selected_pass_count": selected_count,
        "runtime_export_check_count": runtime_export_summary.get("check_count", 0),
        "runtime_export_pass_count": runtime_export_summary.get("pass_count", 0),
        "operator_launch_status": operator_launch.get("status", ""),
        "fusion_status": "scenario_disparity_residual_catalog",
        "disparity_residual_status": "label_selector_saved_report",
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def compact_scenario_residual_catalog_result(result: Mapping[str, object]) -> dict[str, object]:
    selector = result.get("selector", {})
    if not isinstance(selector, Mapping):
        selector = {}
    runtime_export_summary = result.get("runtime_export_summary", {})
    if not isinstance(runtime_export_summary, Mapping):
        runtime_export_summary = {}
    operator_launch = result.get("operator_launch", {})
    if not isinstance(operator_launch, Mapping):
        operator_launch = {}
    catalog_rows = result.get("catalog_rows", [])
    selections = result.get("selections", {})
    if not isinstance(selections, Mapping):
        selections = {}
    return {
        "catalog_dir": result.get("catalog_dir", ""),
        "selector_path": result.get("selector_path", ""),
        "catalog_csv_path": result.get("catalog_csv_path", ""),
        "runtime_export_summary_path": result.get("runtime_export_summary_path", ""),
        "operator_launch_path": result.get("operator_launch_path", ""),
        "summary": {
            "selector_status": selector.get("status", ""),
            "catalog_row_count": len(catalog_rows) if isinstance(catalog_rows, list) else 0,
            "selection_count": len(selections),
            "runtime_export_status": runtime_export_summary.get("status", ""),
            "operator_launch_status": operator_launch.get("status", ""),
            "runtime_truth_used": selector.get("runtime_truth_used", True),
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

    if not args.skip_bbox_anchor_fullscreen_report:
        bbox_anchor_result = run_bbox_anchor_fullscreen_report_check(Path(args.bbox_anchor_fullscreen_report_dir))
        bbox_anchor_evaluation = evaluate_bbox_anchor_fullscreen_report(bbox_anchor_result)
        candidates.append(
            {
                "name": "bbox_anchor_global_fullscreen_right_att_0p1",
                "result": compact_bbox_anchor_fullscreen_result(bbox_anchor_result),
                "evaluation": bbox_anchor_evaluation,
            }
        )

    if not args.skip_origin_bbox_output_report:
        origin_bbox_output_result = run_origin_bbox_output_report_check(Path(args.origin_bbox_output_report_dir))
        origin_bbox_output_evaluation = evaluate_origin_bbox_output_report(origin_bbox_output_result)
        candidates.append(
            {
                "name": "origin_bbox_output_validation_8trials_slow",
                "result": compact_origin_bbox_output_result(origin_bbox_output_result),
                "evaluation": origin_bbox_output_evaluation,
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

    if not args.skip_bbox_outlier_report:
        for config in BBOX_OUTLIER_PROMOTED_METHODS:
            bbox_outlier_result = run_bbox_outlier_report_check(config)
            bbox_outlier_evaluation = evaluate_bbox_outlier_report(bbox_outlier_result)
            candidates.append(
                {
                    "name": config["method"],
                    "result": compact_bbox_outlier_result(bbox_outlier_result),
                    "evaluation": bbox_outlier_evaluation,
                }
            )

    if not args.skip_multiburst_residual_aggregate_report:
        residual_aggregate_result = run_multiburst_residual_aggregate_report_check(
            Path(args.multiburst_residual_aggregate_report_dir)
        )
        residual_aggregate_evaluation = evaluate_multiburst_residual_aggregate_report(residual_aggregate_result)
        candidates.append(
            {
                "name": "multiburst_disparity_residual_aggregate_best",
                "result": compact_multiburst_residual_aggregate_result(residual_aggregate_result),
                "evaluation": residual_aggregate_evaluation,
            }
        )

    if not args.skip_scenario_residual_catalog:
        scenario_catalog_result = run_scenario_residual_catalog_check(Path(args.scenario_residual_catalog_dir))
        scenario_catalog_evaluation = evaluate_scenario_residual_catalog(scenario_catalog_result)
        candidates.append(
            {
                "name": "scenario_labeled_disparity_residual_profile_catalog_exp281",
                "result": compact_scenario_residual_catalog_result(scenario_catalog_result),
                "evaluation": scenario_catalog_evaluation,
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
