#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build a consolidated dashboard from RTK stereo calibration, selector, guard, and export audits."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def read_json_optional(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv_optional(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def field(payload: Mapping[str, object], key: str, default: object = "") -> object:
    value = payload.get(key, default)
    return default if value is None else value


def dashboard_rows(sweep_dir: Path) -> list[dict[str, object]]:
    deployment = read_json_optional(sweep_dir / "deployment_strategy_recommendation.json")
    post_deployment = read_json_optional(
        sweep_dir
        / "post_sweep_deployment_audit_minimal_with_blend_tie_guard"
        / "deployment_audit_strategy_summary"
        / "deployment_audit_strategy_recommendation.json"
    )
    runtime_policy = read_json_optional(
        sweep_dir / "runtime_measurement_guard_policy_audit" / "runtime_measurement_guard_policy_audit_summary.json"
    )
    runtime_policy_loso = read_json_optional(
        sweep_dir
        / "runtime_measurement_guard_policy_audit_loso"
        / "runtime_measurement_guard_policy_audit_summary.json"
    )
    feature_vote_policy = read_json_optional(
        sweep_dir
        / "runtime_measurement_guard_policy_audit_feature_vote"
        / "runtime_measurement_guard_policy_audit_summary.json"
    )
    feature_vote_policy_rows = read_csv_optional(
        sweep_dir
        / "runtime_measurement_guard_policy_audit_feature_vote"
        / "runtime_measurement_guard_policy_audit.csv"
    )
    threshold = read_json_optional(
        sweep_dir
        / "runtime_measurement_guard_transfer_threshold_sweep"
        / "runtime_measurement_guard_threshold_sweep_summary.json"
    )
    threshold_loso = read_json_optional(
        sweep_dir
        / "runtime_measurement_guard_transfer_threshold_sweep_loso"
        / "runtime_measurement_guard_threshold_sweep_summary.json"
    )
    oracle = read_json_optional(
        sweep_dir
        / "runtime_measurement_guard_oracle_regret_transfer_risk"
        / "runtime_measurement_guard_oracle_regret_summary.json"
    )
    target_weight = read_json_optional(
        sweep_dir / "runtime_measurement_guard_target_weight_sweep" / "runtime_measurement_guard_target_weight_sweep_summary.json"
    )
    oracle_label = read_json_optional(
        sweep_dir / "runtime_oracle_label_selector_audit_nearest" / "runtime_oracle_label_selector_summary.json"
    )
    knn_sweep = read_json_optional(
        sweep_dir / "runtime_measurement_guard_knn_sweep" / "runtime_measurement_guard_knn_sweep_summary.json"
    )
    hybrid_policy_rows = read_csv_optional(
        sweep_dir / "runtime_measurement_guard_policy_audit_hybrid_veto" / "runtime_measurement_guard_policy_audit.csv"
    )
    blend_policy_rows = read_csv_optional(
        sweep_dir / "profile_blend_strategy_audit_all_catalog" / "blend_strategy_policy_summary.csv"
    )
    operating = read_json_optional(
        sweep_dir / "selector_operating_mode_audit" / "selector_operating_mode_recommendation.json"
    )
    calibration_selection = read_json_optional(
        sweep_dir / "profile_selection_strategy_audit_all_catalog" / "profile_selection_strategy_summary.json"
    )
    calibration_selection_loso = read_json_optional(
        sweep_dir
        / "profile_selection_strategy_audit_all_catalog_loso"
        / "profile_selection_strategy_summary.json"
    )
    loso_coverage_gap = read_json_optional(
        sweep_dir / "loso_coverage_gap_summary" / "loso_coverage_gap_summary.json"
    )
    gap_feature_separability = read_json_optional(
        sweep_dir / "gap_feature_separability_audit" / "gap_feature_separability_summary.json"
    )
    active_calibration = read_json_optional(
        sweep_dir / "operating_mode_export_smoke_calibration" / "operating_mode_profile_export_summary.json"
    )
    active_runtime = read_json_optional(
        sweep_dir / "operating_mode_export_smoke_runtime_guard" / "operating_mode_profile_export_summary.json"
    )
    active_intrinsic_scale_guard = read_json_optional(
        sweep_dir
        / "operating_mode_export_intrinsic_scale_guard_smoke"
        / "operating_mode_profile_export_summary.json"
    )
    intrinsic_scale_calibration_export = read_json_optional(
        sweep_dir
        / "intrinsic_scale_calibration_export_smoke"
        / "intrinsic_scale_profile_export_summary.json"
    )
    intrinsic_scale_calibration_replay_rows = read_csv_optional(
        sweep_dir
        / "intrinsic_scale_calibration_export_smoke"
        / "replay"
        / "profile_replay_summary.csv"
    )
    intrinsic_scale_calibration_replay = (
        intrinsic_scale_calibration_replay_rows[0] if intrinsic_scale_calibration_replay_rows else {}
    )
    intrinsic_scale_family_transfer = read_json_optional(
        sweep_dir
        / "intrinsic_scale_calibration_export_smoke"
        / "intrinsic_family_transfer"
        / "active_profile_transfer_summary.json"
    )
    intrinsic_scale_family_export_audit = read_json_optional(
        sweep_dir
        / "intrinsic_scale_family_calibration_export_audit"
        / "family_calibration_profile_export_summary.json"
    )
    all_family_export_audit = read_json_optional(
        sweep_dir
        / "all_family_calibration_export_audit"
        / "family_calibration_profile_export_summary.json"
    )
    bbox_policy_family_export_audit = read_json_optional(
        sweep_dir
        / "bbox_policy_family_calibration_export_audit"
        / "family_calibration_profile_export_summary.json"
    )
    bbox_policy_family_export_rows = read_csv_optional(
        sweep_dir
        / "bbox_policy_family_calibration_export_audit"
        / "family_calibration_profile_policy_summary.csv"
    )
    bbox_policy_family_export_row = next(
        (
            row
            for row in bbox_policy_family_export_rows
            if row.get("source_policy") == bbox_policy_family_export_audit.get("recommended_source_policy")
        ),
        {},
    )
    all_policy_family_export_audit = read_json_optional(
        sweep_dir
        / "all_policy_family_calibration_export_audit"
        / "family_calibration_profile_export_summary.json"
    )
    all_policy_family_export_rows = read_csv_optional(
        sweep_dir
        / "all_policy_family_calibration_export_audit"
        / "family_calibration_profile_policy_summary.csv"
    )
    all_policy_family_export_row = next(
        (
            row
            for row in all_policy_family_export_rows
            if row.get("source_policy") == all_policy_family_export_audit.get("recommended_source_policy")
        ),
        {},
    )
    policy_family_catalog = read_json_optional(
        sweep_dir / "policy_family_profile_catalog" / "policy_profile_catalog_summary.json"
    )
    policy_family_bbox_transfer = read_json_optional(
        sweep_dir
        / "policy_family_profile_catalog"
        / "bbox_transfer_replay"
        / "active_profile_transfer_summary.json"
    )
    runtime_policy_catalog_guard = read_json_optional(
        sweep_dir
        / "runtime_measurement_guard_audit_policy_catalog_transfer_risk"
        / "runtime_measurement_guard_summary.json"
    )
    runtime_policy_catalog_robust = read_json_optional(
        sweep_dir
        / "runtime_measurement_guard_audit_policy_catalog_robust_family"
        / "runtime_measurement_guard_summary.json"
    )
    runtime_policy_catalog_robust_loso = read_json_optional(
        sweep_dir
        / "runtime_measurement_guard_audit_policy_catalog_robust_family_loso"
        / "runtime_measurement_guard_summary.json"
    )
    runtime_calibration_required_loso = read_json_optional(
        sweep_dir
        / "runtime_calibration_required_gap_summary_policy_catalog_robust_family_loso"
        / "runtime_calibration_required_gap_summary.json"
    )
    method_family_summary = read_json_optional(
        sweep_dir
        / "method_family_summary"
        / "method_family_summary.json"
    )
    affine_family_selection = read_json_optional(
        sweep_dir
        / "affine_family_profile_selection_audit"
        / "method_family_profile_selection_summary.json"
    )
    affine_family_runtime_selector = read_json_optional(
        sweep_dir
        / "affine_family_runtime_feature_selector_audit_robust_bbox"
        / "runtime_feature_selector_summary.json"
    )
    runtime_selector_portfolio = read_json_optional(
        sweep_dir
        / "runtime_selector_portfolio_policy_vs_affine"
        / "runtime_selector_portfolio_summary.json"
    )
    calibration_required_block_smoke = read_json_optional(
        sweep_dir
        / "operating_mode_export_intrinsic_scale_required_block_smoke"
        / "operating_mode_profile_export_summary.json"
    )
    active_target_guard_blend = read_json_optional(
        sweep_dir
        / "active_profile_export_target_guard_blend"
        / "blend_profile_export"
        / "profile_blend_selection.json"
    )
    active_target_guard_blend_promotion = read_json_optional(
        sweep_dir
        / "active_profile_export_target_guard_blend"
        / "promotion_check"
        / "promotion_check.json"
    )
    active_target_guard_blend_transfer = read_json_optional(
        sweep_dir
        / "active_profile_export_target_guard_blend"
        / "transfer_replay"
        / "active_profile_transfer_summary.json"
    )
    active_profile_portfolio = read_json_optional(
        sweep_dir
        / "active_profile_portfolio_audit"
        / "active_profile_portfolio_recommendation.json"
    )
    feature_vote_row = next(
        (
            row
            for row in feature_vote_policy_rows
            if row.get("candidate_source_mode") == "all_catalog"
            and row.get("selection_policy") == "feature_vote_transfer_risk"
        ),
        {},
    )
    hybrid_veto_row = next(
        (
            row
            for row in hybrid_policy_rows
            if row.get("candidate_source_mode") == "all_catalog"
            and row.get("selection_policy") == "centroid_knn_veto_transfer_risk"
        ),
        {},
    )
    blend_target_guard_row = next(
        (row for row in blend_policy_rows if row.get("policy") == "target_guard"),
        {},
    )
    rows = [
        {
            "area": "deployment_strategy",
            "recommendation": field(deployment, "deployment_strategy", field(deployment, "recommended_strategy", "")),
            "pass_count": field(deployment, "pass_count", ""),
            "audit_count": field(deployment, "audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_mean_m",
            "metric_1_value": field(deployment, "validation_range_rmse_corrected_mean_m", ""),
            "metric_2_name": "target_abs_mean_error_max_m",
            "metric_2_value": field(deployment, "target_abs_mean_error_max_m", ""),
            "source_path": str(sweep_dir / "deployment_strategy_recommendation.json"),
        },
        {
            "area": "post_sweep_deployment_audit",
            "recommendation": field(post_deployment, "recommended_strategy", ""),
            "pass_count": field(post_deployment, "recommended_pass_count", ""),
            "audit_count": field(post_deployment, "recommended_audit_count", ""),
            "metric_1_name": "recommended_policy",
            "metric_1_value": field(post_deployment, "recommended_policy", ""),
            "metric_2_name": "recommended_target_abs_mean_error_max_m",
            "metric_2_value": field(post_deployment, "target_abs_mean_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "post_sweep_deployment_audit_minimal_with_blend_tie_guard"
                / "deployment_audit_strategy_summary"
                / "deployment_audit_strategy_recommendation.json"
            ),
        },
        {
            "area": "runtime_truth_free_selector",
            "recommendation": (
                f"{field(runtime_policy, 'recommended_candidate_source_mode', '')}+"
                f"{field(runtime_policy, 'recommended_selection_policy', '')}"
            ),
            "pass_count": field(runtime_policy, "recommended_pass_count", ""),
            "audit_count": field(runtime_policy, "recommended_audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(runtime_policy, "recommended_validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(runtime_policy, "recommended_target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir / "runtime_measurement_guard_policy_audit" / "runtime_measurement_guard_policy_audit_summary.json"
            ),
        },
        {
            "area": "runtime_truth_free_selector_loso",
            "recommendation": (
                f"{field(runtime_policy_loso, 'recommended_candidate_source_mode', '')}+"
                f"{field(runtime_policy_loso, 'recommended_selection_policy', '')}"
            ),
            "pass_count": field(runtime_policy_loso, "recommended_pass_count", ""),
            "audit_count": field(runtime_policy_loso, "recommended_audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(runtime_policy_loso, "recommended_validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(runtime_policy_loso, "recommended_target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_policy_audit_loso"
                / "runtime_measurement_guard_policy_audit_summary.json"
            ),
        },
        {
            "area": "raw_feature_vote_selector_exploration",
            "recommendation": "all_catalog+feature_vote_transfer_risk",
            "pass_count": field(feature_vote_row, "pass_count", field(feature_vote_policy, "recommended_pass_count", "")),
            "audit_count": field(feature_vote_row, "audit_count", field(feature_vote_policy, "recommended_audit_count", "")),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(
                feature_vote_row,
                "validation_range_rmse_corrected_max_m",
                field(feature_vote_policy, "recommended_validation_range_rmse_corrected_max_m", ""),
            ),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(
                feature_vote_row,
                "target_max_abs_error_max_m",
                field(feature_vote_policy, "recommended_target_max_abs_error_max_m", ""),
            ),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_policy_audit_feature_vote"
                / "runtime_measurement_guard_policy_audit_summary.json"
            ),
        },
        {
            "area": "transfer_threshold",
            "recommendation": (
                f"rmse<={field(threshold, 'recommended_rmse_limit_m', '')},"
                f"target<={field(threshold, 'recommended_target_limit_m', '')}"
            ),
            "pass_count": field(threshold, "recommended_pass_count", ""),
            "audit_count": field(threshold, "recommended_audit_count", ""),
            "metric_1_name": "threshold_selection_score",
            "metric_1_value": field(threshold, "recommended_threshold_selection_score", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(threshold, "recommended_target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_transfer_threshold_sweep"
                / "runtime_measurement_guard_threshold_sweep_summary.json"
            ),
        },
        {
            "area": "transfer_threshold_loso",
            "recommendation": (
                f"rmse<={field(threshold_loso, 'recommended_rmse_limit_m', '')},"
                f"target<={field(threshold_loso, 'recommended_target_limit_m', '')}"
            ),
            "pass_count": field(threshold_loso, "recommended_pass_count", ""),
            "audit_count": field(threshold_loso, "recommended_audit_count", ""),
            "metric_1_name": "threshold_selection_score",
            "metric_1_value": field(threshold_loso, "recommended_threshold_selection_score", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(threshold_loso, "recommended_target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_transfer_threshold_sweep_loso"
                / "runtime_measurement_guard_threshold_sweep_summary.json"
            ),
        },
        {
            "area": "oracle_regret",
            "recommendation": "runtime selector within sub-meter target regret",
            "pass_count": field(oracle, "selected_pass_count", ""),
            "audit_count": field(oracle, "sample_count", ""),
            "metric_1_name": "target_abs_regret_max_m",
            "metric_1_value": field(oracle, "target_abs_regret_max_m", ""),
            "metric_2_name": "target_max_regret_max_m",
            "metric_2_value": field(oracle, "target_max_regret_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_oracle_regret_transfer_risk"
                / "runtime_measurement_guard_oracle_regret_summary.json"
            ),
        },
        {
            "area": "target_weight_sweep",
            "recommendation": f"weight={field(target_weight, 'recommended_transfer_risk_target_weight', '')}",
            "pass_count": field(target_weight, "recommended_guard_pass_count", ""),
            "audit_count": field(target_weight, "recommended_guard_audit_count", ""),
            "metric_1_name": "target_abs_regret_max_m",
            "metric_1_value": field(target_weight, "recommended_target_abs_regret_max_m", ""),
            "metric_2_name": "target_max_regret_max_m",
            "metric_2_value": field(target_weight, "recommended_target_max_regret_max_m", ""),
            "source_path": str(
                sweep_dir / "runtime_measurement_guard_target_weight_sweep" / "runtime_measurement_guard_target_weight_sweep_summary.json"
            ),
        },
        {
            "area": "runtime_oracle_label_selector_exploration",
            "recommendation": field(oracle_label, "selection_policy", ""),
            "pass_count": field(oracle_label, "pass_count", ""),
            "audit_count": field(oracle_label, "audit_count", ""),
            "metric_1_name": "calibration_replay_label_match_count",
            "metric_1_value": field(oracle_label, "calibration_replay_label_match_count", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(oracle_label, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir / "runtime_oracle_label_selector_audit_nearest" / "runtime_oracle_label_selector_summary.json"
            ),
        },
        {
            "area": "runtime_knn_selector_exploration",
            "recommendation": f"k={field(knn_sweep, 'recommended_knn_neighbor_count', '')}",
            "pass_count": field(knn_sweep, "recommended_pass_count", ""),
            "audit_count": field(knn_sweep, "recommended_audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(knn_sweep, "recommended_validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(knn_sweep, "recommended_target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir / "runtime_measurement_guard_knn_sweep" / "runtime_measurement_guard_knn_sweep_summary.json"
            ),
        },
        {
            "area": "runtime_hybrid_veto_selector_exploration",
            "recommendation": "all_catalog+centroid_knn_veto_transfer_risk",
            "pass_count": field(hybrid_veto_row, "pass_count", ""),
            "audit_count": field(hybrid_veto_row, "audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(hybrid_veto_row, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(hybrid_veto_row, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_policy_audit_hybrid_veto"
                / "runtime_measurement_guard_policy_audit.csv"
            ),
        },
        {
            "area": "profile_blend_target_guard_exploration",
            "recommendation": "all_catalog+target_guard",
            "pass_count": field(blend_target_guard_row, "pass_count", ""),
            "audit_count": field(blend_target_guard_row, "run_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(blend_target_guard_row, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(blend_target_guard_row, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir / "profile_blend_strategy_audit_all_catalog" / "blend_strategy_policy_summary.csv"
            ),
        },
        {
            "area": "operating_mode",
            "recommendation": (
                f"truth:{field(operating, 'recommended_when_truth_available', '')}; "
                f"runtime:{field(operating, 'recommended_when_truth_unavailable', '')}"
            ),
            "pass_count": field(operating, "runtime_pass_count", ""),
            "audit_count": field(operating, "row_count", ""),
            "metric_1_name": "runtime_target_abs_mean_m",
            "metric_1_value": field(operating, "runtime_target_abs_mean_error_corrected_mean_m", ""),
            "metric_2_name": "calibration_target_abs_mean_m",
            "metric_2_value": field(operating, "calibration_target_abs_mean_error_corrected_mean_m", ""),
            "source_path": str(sweep_dir / "selector_operating_mode_audit" / "selector_operating_mode_recommendation.json"),
        },
        {
            "area": "calibration_truth_selector",
            "recommendation": "calibration_replay_all_catalog",
            "pass_count": field(calibration_selection, "calibration_pass_count", ""),
            "audit_count": field(calibration_selection, "run_count", ""),
            "metric_1_name": "calibration_beats_label_count",
            "metric_1_value": field(calibration_selection, "calibration_beats_label_count", ""),
            "metric_2_name": "calibration_matches_label_count",
            "metric_2_value": field(calibration_selection, "calibration_matches_label_count", ""),
            "source_path": str(
                sweep_dir / "profile_selection_strategy_audit_all_catalog" / "profile_selection_strategy_summary.json"
            ),
        },
        {
            "area": "calibration_truth_selector_loso",
            "recommendation": "calibration_replay_all_catalog_loso",
            "pass_count": field(calibration_selection_loso, "calibration_pass_count", ""),
            "audit_count": field(calibration_selection_loso, "run_count", ""),
            "metric_1_name": "calibration_beats_label_count",
            "metric_1_value": field(calibration_selection_loso, "calibration_beats_label_count", ""),
            "metric_2_name": "exclude_target_scenario_from_candidates",
            "metric_2_value": field(calibration_selection_loso, "exclude_target_scenario_from_candidates", ""),
            "source_path": str(
                sweep_dir
                / "profile_selection_strategy_audit_all_catalog_loso"
                / "profile_selection_strategy_summary.json"
            ),
        },
        {
            "area": "loso_coverage_gap",
            "recommendation": ",".join(field(loso_coverage_gap, "full_gap_scenarios", []) or []),
            "pass_count": field(loso_coverage_gap, "scenario_count", ""),
            "audit_count": field(loso_coverage_gap, "scenario_count", ""),
            "metric_1_name": "indispensable_scenario_count",
            "metric_1_value": field(loso_coverage_gap, "indispensable_scenario_count", ""),
            "metric_2_name": "full_gap_scenario_count",
            "metric_2_value": field(loso_coverage_gap, "full_gap_scenario_count", ""),
            "source_path": str(sweep_dir / "loso_coverage_gap_summary" / "loso_coverage_gap_summary.json"),
        },
        {
            "area": "gap_feature_separability",
            "recommendation": field(gap_feature_separability, "detection_recommendation", ""),
            "pass_count": 1 if field(gap_feature_separability, "detection_recommendation", "") else "",
            "audit_count": 1 if gap_feature_separability else "",
            "metric_1_name": "best_feature",
            "metric_1_value": field(gap_feature_separability, "best_feature", ""),
            "metric_2_name": "best_accuracy",
            "metric_2_value": field(gap_feature_separability, "best_accuracy", ""),
            "source_path": str(
                sweep_dir / "gap_feature_separability_audit" / "gap_feature_separability_summary.json"
            ),
        },
        {
            "area": "active_export_calibration_smoke",
            "recommendation": field(active_calibration, "selected_operating_mode", ""),
            "pass_count": 1 if active_calibration else "",
            "audit_count": 1 if active_calibration else "",
            "metric_1_name": "selected_source_scenario",
            "metric_1_value": field(active_calibration, "selected_source_scenario", ""),
            "metric_2_name": "target_abs_mean_error_corrected_m",
            "metric_2_value": field(active_calibration, "target_abs_mean_error_corrected_m", ""),
            "source_path": str(sweep_dir / "operating_mode_export_smoke_calibration" / "operating_mode_profile_export_summary.json"),
        },
        {
            "area": "active_export_runtime_smoke",
            "recommendation": field(active_runtime, "selected_operating_mode", ""),
            "pass_count": 1 if active_runtime else "",
            "audit_count": 1 if active_runtime else "",
            "metric_1_name": "selected_source_scenario",
            "metric_1_value": field(active_runtime, "selected_source_scenario", ""),
            "metric_2_name": "has_truth_fields",
            "metric_2_value": field(active_runtime, "has_truth_fields", ""),
            "source_path": str(sweep_dir / "operating_mode_export_smoke_runtime_guard" / "operating_mode_profile_export_summary.json"),
        },
        {
            "area": "active_export_intrinsic_scale_guard_smoke",
            "recommendation": field(active_intrinsic_scale_guard, "selected_operating_mode", ""),
            "pass_count": 1 if active_intrinsic_scale_guard else "",
            "audit_count": 1 if active_intrinsic_scale_guard else "",
            "metric_1_name": "selected_source_scenario",
            "metric_1_value": field(active_intrinsic_scale_guard, "selected_source_scenario", ""),
            "metric_2_name": "fallback_reason",
            "metric_2_value": field(
                field(active_intrinsic_scale_guard, "selection_summary", {})
                if isinstance(field(active_intrinsic_scale_guard, "selection_summary", {}), Mapping)
                else {},
                "fallback_reason",
                "",
            ),
            "source_path": str(
                sweep_dir
                / "operating_mode_export_intrinsic_scale_guard_smoke"
                / "operating_mode_profile_export_summary.json"
            ),
        },
        {
            "area": "intrinsic_scale_calibration_export_smoke",
            "recommendation": field(intrinsic_scale_calibration_export, "recommended_method", ""),
            "pass_count": 1 if intrinsic_scale_calibration_export.get("selected") is True else "",
            "audit_count": 1 if intrinsic_scale_calibration_export else "",
            "metric_1_name": "validation_range_rmse_corrected_m",
            "metric_1_value": field(intrinsic_scale_calibration_replay, "validation_range_rmse_corrected_m", ""),
            "metric_2_name": "target_abs_mean_error_corrected_m",
            "metric_2_value": field(intrinsic_scale_calibration_replay, "target_abs_mean_error_corrected_m", ""),
            "source_path": str(
                sweep_dir
                / "intrinsic_scale_calibration_export_smoke"
                / "intrinsic_scale_profile_export_summary.json"
            ),
        },
        {
            "area": "intrinsic_scale_family_transfer",
            "recommendation": field(intrinsic_scale_family_transfer, "target_scenarios", ""),
            "pass_count": field(intrinsic_scale_family_transfer, "transfer_replay_pass_count", ""),
            "audit_count": field(intrinsic_scale_family_transfer, "transfer_replay_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(intrinsic_scale_family_transfer, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(intrinsic_scale_family_transfer, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "intrinsic_scale_calibration_export_smoke"
                / "intrinsic_family_transfer"
                / "active_profile_transfer_summary.json"
            ),
        },
        {
            "area": "intrinsic_scale_family_calibration_export_audit",
            "recommendation": field(intrinsic_scale_family_export_audit, "target_scenarios", ""),
            "pass_count": field(intrinsic_scale_family_export_audit, "heldout_replay_pass_count", ""),
            "audit_count": field(intrinsic_scale_family_export_audit, "heldout_replay_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(
                intrinsic_scale_family_export_audit,
                "validation_range_rmse_corrected_max_m",
                "",
            ),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(intrinsic_scale_family_export_audit, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "intrinsic_scale_family_calibration_export_audit"
                / "family_calibration_profile_export_summary.json"
            ),
        },
        {
            "area": "all_family_calibration_export_audit",
            "recommendation": field(all_family_export_audit, "target_scenarios", "") or "all_scenarios",
            "pass_count": field(all_family_export_audit, "heldout_replay_pass_count", ""),
            "audit_count": field(all_family_export_audit, "heldout_replay_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(
                all_family_export_audit,
                "validation_range_rmse_corrected_max_m",
                "",
            ),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(all_family_export_audit, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "all_family_calibration_export_audit"
                / "family_calibration_profile_export_summary.json"
            ),
        },
        {
            "area": "bbox_policy_family_calibration_export_audit",
            "recommendation": field(bbox_policy_family_export_audit, "recommended_source_policy", ""),
            "pass_count": field(bbox_policy_family_export_row, "heldout_replay_pass_count", ""),
            "audit_count": field(bbox_policy_family_export_row, "heldout_replay_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(
                bbox_policy_family_export_row,
                "validation_range_rmse_corrected_max_m",
                "",
            ),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(bbox_policy_family_export_row, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "bbox_policy_family_calibration_export_audit"
                / "family_calibration_profile_export_summary.json"
            ),
        },
        {
            "area": "all_policy_family_calibration_export_audit",
            "recommendation": field(all_policy_family_export_audit, "recommended_source_policy", ""),
            "pass_count": field(all_policy_family_export_row, "heldout_replay_pass_count", ""),
            "audit_count": field(all_policy_family_export_row, "heldout_replay_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(
                all_policy_family_export_row,
                "validation_range_rmse_corrected_max_m",
                "",
            ),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(all_policy_family_export_row, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "all_policy_family_calibration_export_audit"
                / "family_calibration_profile_export_summary.json"
            ),
        },
        {
            "area": "policy_family_profile_catalog",
            "recommendation": "scenario_policy_catalog",
            "pass_count": field(policy_family_catalog, "pass_profile_count", ""),
            "audit_count": field(policy_family_catalog, "profile_count", ""),
            "metric_1_name": "pass_policy_profile_catalog",
            "metric_1_value": field(policy_family_catalog, "pass_policy_profile_catalog", ""),
            "metric_2_name": "bbox_target_max_abs_error_max_m",
            "metric_2_value": field(policy_family_bbox_transfer, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "policy_family_profile_catalog"
                / "manifest.json"
            ),
        },
        {
            "area": "runtime_guard_policy_catalog_transfer_risk",
            "recommendation": field(runtime_policy_catalog_guard, "selection_policy", ""),
            "pass_count": field(runtime_policy_catalog_guard, "pass_count", ""),
            "audit_count": field(runtime_policy_catalog_guard, "audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(runtime_policy_catalog_guard, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(runtime_policy_catalog_guard, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_audit_policy_catalog_transfer_risk"
                / "runtime_measurement_guard_summary.json"
            ),
        },
        {
            "area": "runtime_guard_policy_catalog_robust_family",
            "recommendation": field(runtime_policy_catalog_robust, "selection_policy", ""),
            "pass_count": field(runtime_policy_catalog_robust, "pass_count", ""),
            "audit_count": field(runtime_policy_catalog_robust, "audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(runtime_policy_catalog_robust, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(runtime_policy_catalog_robust, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_audit_policy_catalog_robust_family"
                / "runtime_measurement_guard_summary.json"
            ),
        },
        {
            "area": "runtime_guard_policy_catalog_robust_family_loso",
            "recommendation": field(runtime_policy_catalog_robust_loso, "selection_policy", ""),
            "pass_count": field(runtime_policy_catalog_robust_loso, "pass_count", ""),
            "audit_count": field(runtime_policy_catalog_robust_loso, "audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(runtime_policy_catalog_robust_loso, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(runtime_policy_catalog_robust_loso, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_audit_policy_catalog_robust_family_loso"
                / "runtime_measurement_guard_summary.json"
            ),
        },
        {
            "area": "runtime_guard_policy_catalog_robust_family_loso_validation_only",
            "recommendation": "validation_only_diagnostic",
            "pass_count": field(runtime_policy_catalog_robust_loso, "validation_only_pass_count", ""),
            "audit_count": field(runtime_policy_catalog_robust_loso, "audit_count", ""),
            "metric_1_name": "validation_only_pass_all",
            "metric_1_value": field(runtime_policy_catalog_robust_loso, "validation_only_pass_all", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(runtime_policy_catalog_robust_loso, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_measurement_guard_audit_policy_catalog_robust_family_loso"
                / "runtime_measurement_guard_summary.json"
            ),
        },
        {
            "area": "runtime_calibration_required_gap_loso",
            "recommendation": "calibration_required_regime_guard",
            "pass_count": field(runtime_calibration_required_loso, "calibration_required_validation_only_pass_count", ""),
            "audit_count": field(runtime_calibration_required_loso, "calibration_required_count", ""),
            "metric_1_name": "unsafe_correction_count",
            "metric_1_value": field(runtime_calibration_required_loso, "unsafe_correction_count", ""),
            "metric_2_name": "calibration_required_target_within_limit_count",
            "metric_2_value": field(runtime_calibration_required_loso, "calibration_required_target_within_limit_count", ""),
            "source_path": str(
                sweep_dir
                / "runtime_calibration_required_gap_summary_policy_catalog_robust_family_loso"
                / "runtime_calibration_required_gap_summary.json"
            ),
        },
        {
            "area": "runtime_calibration_required_export_block_smoke",
            "recommendation": "block_export_when_required_profile_unavailable",
            "pass_count": 1 if calibration_required_block_smoke.get("calibration_required_blocked_export") is True else "",
            "audit_count": 1 if calibration_required_block_smoke else "",
            "metric_1_name": "selected",
            "metric_1_value": field(calibration_required_block_smoke, "selected", ""),
            "metric_2_name": "calibration_required_block_reason",
            "metric_2_value": field(calibration_required_block_smoke, "calibration_required_block_reason", ""),
            "source_path": str(
                sweep_dir
                / "operating_mode_export_intrinsic_scale_required_block_smoke"
                / "operating_mode_profile_export_summary.json"
            ),
        },
        {
            "area": "method_family_potential",
            "recommendation": field(method_family_summary, "oracle_recommended_family_by_validation", ""),
            "pass_count": field(method_family_summary, "oracle_recommended_family_pass_target_max_limit_count", ""),
            "audit_count": field(method_family_summary, "oracle_recommended_family_run_count", ""),
            "metric_1_name": "row_level_recommended_family",
            "metric_1_value": field(method_family_summary, "recommended_family_by_validation", ""),
            "metric_2_name": "oracle_target_max_abs_error_max_m",
            "metric_2_value": field(method_family_summary, "oracle_recommended_family_target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "method_family_summary"
                / "method_family_summary.json"
            ),
        },
        {
            "area": "affine_family_profile_selection",
            "recommendation": field(affine_family_selection, "family_name", ""),
            "pass_count": field(affine_family_selection, "selected_pass_count", ""),
            "audit_count": field(affine_family_selection, "run_count", ""),
            "metric_1_name": "selected_pass_all",
            "metric_1_value": field(affine_family_selection, "selected_pass_all", ""),
            "metric_2_name": "selected_target_max_abs_error_max_m",
            "metric_2_value": field(affine_family_selection, "selected_validation_target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "affine_family_profile_selection_audit"
                / "method_family_profile_selection_summary.json"
            ),
        },
        {
            "area": "affine_family_runtime_feature_selector",
            "recommendation": field(affine_family_runtime_selector, "selection_policy", ""),
            "pass_count": field(affine_family_runtime_selector, "pass_count", ""),
            "audit_count": field(affine_family_runtime_selector, "audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(affine_family_runtime_selector, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(affine_family_runtime_selector, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "affine_family_runtime_feature_selector_audit_robust_bbox"
                / "runtime_feature_selector_summary.json"
            ),
        },
        {
            "area": "runtime_selector_portfolio_policy_vs_affine",
            "recommendation": field(runtime_selector_portfolio, "objective", ""),
            "pass_count": field(runtime_selector_portfolio, "pass_count", ""),
            "audit_count": field(runtime_selector_portfolio, "audit_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(runtime_selector_portfolio, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(runtime_selector_portfolio, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_selector_portfolio_policy_vs_affine"
                / "runtime_selector_portfolio_summary.json"
            ),
        },
        {
            "area": "runtime_selector_portfolio_oracle_gap",
            "recommendation": field(runtime_selector_portfolio, "objective", ""),
            "pass_count": field(runtime_selector_portfolio, "selected_oracle_match_count", ""),
            "audit_count": field(runtime_selector_portfolio, "audit_count", ""),
            "metric_1_name": "rmse_regret_max_m",
            "metric_1_value": field(runtime_selector_portfolio, "rmse_regret_max_m", ""),
            "metric_2_name": "selected_oracle_match_fraction",
            "metric_2_value": field(runtime_selector_portfolio, "selected_oracle_match_fraction", ""),
            "source_path": str(
                sweep_dir
                / "runtime_selector_portfolio_policy_vs_affine"
                / "runtime_selector_portfolio_summary.json"
            ),
        },
        {
            "area": "runtime_selector_portfolio_oracle_separability",
            "recommendation": field(runtime_selector_portfolio, "oracle_separability_best_label", ""),
            "pass_count": field(runtime_selector_portfolio, "selected_oracle_match_count", ""),
            "audit_count": field(runtime_selector_portfolio, "audit_count", ""),
            "metric_1_name": "best_feature",
            "metric_1_value": field(runtime_selector_portfolio, "oracle_separability_best_feature", ""),
            "metric_2_name": "best_accuracy",
            "metric_2_value": field(runtime_selector_portfolio, "oracle_separability_best_accuracy", ""),
            "source_path": str(
                sweep_dir
                / "runtime_selector_portfolio_policy_vs_affine"
                / "runtime_selector_portfolio_oracle_separability.csv"
            ),
        },
        {
            "area": "runtime_selector_portfolio_precision_override",
            "recommendation": field(runtime_selector_portfolio, "precision_override_label", ""),
            "pass_count": field(runtime_selector_portfolio, "precision_override_pass_count", ""),
            "audit_count": field(runtime_selector_portfolio, "audit_count", ""),
            "metric_1_name": "precision_override_rmse_mean_m",
            "metric_1_value": field(runtime_selector_portfolio, "precision_override_rmse_mean_m", ""),
            "metric_2_name": "precision_override_target_max_abs_error_max_m",
            "metric_2_value": field(runtime_selector_portfolio, "precision_override_target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "runtime_selector_portfolio_policy_vs_affine"
                / "runtime_selector_portfolio_precision_override_summary.json"
            ),
        },
        {
            "area": "active_export_target_guard_blend",
            "recommendation": field(active_target_guard_blend, "selection_policy", ""),
            "pass_count": 1 if active_target_guard_blend_promotion.get("pass_promotion_check") is True else "",
            "audit_count": 1 if active_target_guard_blend else "",
            "metric_1_name": "validation_range_rmse_corrected_m",
            "metric_1_value": field(active_target_guard_blend, "validation_range_rmse_corrected_m", ""),
            "metric_2_name": "target_max_abs_error_corrected_m",
            "metric_2_value": field(active_target_guard_blend, "target_max_abs_error_corrected_m", ""),
            "source_path": str(
                sweep_dir
                / "active_profile_export_target_guard_blend"
                / "blend_profile_export"
                / "profile_blend_selection.json"
            ),
        },
        {
            "area": "active_export_target_guard_blend_transfer",
            "recommendation": "fixed_exported_profile",
            "pass_count": field(active_target_guard_blend_transfer, "transfer_replay_pass_count", ""),
            "audit_count": field(active_target_guard_blend_transfer, "transfer_replay_count", ""),
            "metric_1_name": "validation_range_rmse_corrected_max_m",
            "metric_1_value": field(active_target_guard_blend_transfer, "validation_range_rmse_corrected_max_m", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(active_target_guard_blend_transfer, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "active_profile_export_target_guard_blend"
                / "transfer_replay"
                / "active_profile_transfer_summary.json"
            ),
        },
        {
            "area": "active_profile_portfolio_audit",
            "recommendation": field(active_profile_portfolio, "recommended_profile_label", ""),
            "pass_count": field(active_profile_portfolio, "transfer_replay_pass_count", ""),
            "audit_count": field(active_profile_portfolio, "transfer_replay_count", ""),
            "metric_1_name": "selected",
            "metric_1_value": field(active_profile_portfolio, "selected", ""),
            "metric_2_name": "target_max_abs_error_max_m",
            "metric_2_value": field(active_profile_portfolio, "target_max_abs_error_max_m", ""),
            "source_path": str(
                sweep_dir
                / "active_profile_portfolio_audit"
                / "active_profile_portfolio_recommendation.json"
            ),
        },
    ]
    return rows


def build_summary(rows: Sequence[Mapping[str, object]], sweep_dir: Path) -> dict[str, object]:
    def row_for(area: str) -> Mapping[str, object]:
        return next((row for row in rows if row.get("area") == area), {})

    runtime = row_for("runtime_truth_free_selector")
    runtime_policy_catalog = row_for("runtime_guard_policy_catalog_transfer_risk")
    runtime_policy_catalog_robust = row_for("runtime_guard_policy_catalog_robust_family")
    operating = row_for("operating_mode")
    oracle = row_for("oracle_regret")
    all_family_scenarios = read_csv_optional(
        sweep_dir
        / "all_family_calibration_export_audit"
        / "family_calibration_profile_scenario_summary.csv"
    )
    all_policy_scenario_recommendations = read_csv_optional(
        sweep_dir
        / "all_policy_family_calibration_export_audit"
        / "family_calibration_profile_scenario_policy_recommendations.csv"
    )
    policy_family_catalog = read_json_optional(
        sweep_dir / "policy_family_profile_catalog" / "policy_profile_catalog_summary.json"
    )
    next_hint_rows = all_policy_scenario_recommendations or all_family_scenarios
    weakest_family = max(
        next_hint_rows,
        key=lambda row: finite_float(row.get("target_max_abs_error_max_m"), float("-inf")),
        default={},
    )
    if policy_family_catalog:
        next_exploration_hint = (
            "Policy family catalog exports scenario-specific runtime profiles with selected-source "
            f"target max abs worst-case {policy_family_catalog.get('selected_source_target_max_abs_error_max_m', '')} m; "
            "next prioritize truth-free runtime detection that selects this policy catalog without calibration truth."
        )
    elif weakest_family:
        policy_text = ""
        if weakest_family.get("recommended_source_policy"):
            policy_text = f" with policy {weakest_family.get('recommended_source_policy')}"
        next_exploration_hint = (
            "All-family calibration export passes heldout replay; next prioritize "
            f"{weakest_family.get('scenario', '')}{policy_text} because its target-window max abs error is "
            f"{weakest_family.get('target_max_abs_error_max_m', '')} m, closest to the 10 m pass limit."
        )
    else:
        next_exploration_hint = (
            "LOSO coverage shows intrinsic_scale_stress is the current full-gap family; prioritize "
            "baseline/focal-scale regime detection or calibration-time profile export before adding "
            "more generic single-profile candidates."
        )
    runtime_candidates = [
        {
            "recommendation": str(runtime.get("recommendation", "")),
            "pass_count": runtime.get("pass_count", ""),
            "audit_count": runtime.get("audit_count", ""),
            "target_max": runtime.get("metric_2_value", ""),
            "complexity_rank": 2,
        }
    ]
    for row, complexity_rank in (
        (runtime_policy_catalog_robust, 0),
        (runtime_policy_catalog, 1),
    ):
        if row:
            runtime_candidates.append(
                {
                    "recommendation": f"policy_catalog+{row.get('recommendation', '')}",
                    "pass_count": row.get("pass_count", ""),
                    "audit_count": row.get("audit_count", ""),
                    "target_max": row.get("metric_2_value", ""),
                    "complexity_rank": complexity_rank,
                }
            )
    best_runtime = min(
        runtime_candidates,
        key=lambda item: (
            -finite_float(item.get("pass_count"), float("-inf")),
            finite_float(item.get("audit_count"), float("inf")),
            finite_float(item.get("target_max"), float("inf")),
            int(item.get("complexity_rank", 99)),
        ),
    )
    runtime_recommendation = best_runtime.get("recommendation", "")
    runtime_pass_count = best_runtime.get("pass_count", "")
    runtime_audit_count = best_runtime.get("audit_count", "")
    runtime_target_max = best_runtime.get("target_max", "")
    summary = {
        "mode": "rtk_stereo_experiment_dashboard",
        "sweep_dir": str(sweep_dir),
        "current_runtime_recommendation": runtime_recommendation,
        "current_truth_available_recommendation": "calibration_replay_all_catalog",
        "current_truth_unavailable_recommendation": "runtime_transfer_risk_guard",
        "runtime_pass_count": runtime_pass_count,
        "runtime_audit_count": runtime_audit_count,
        "runtime_target_max_abs_error_max_m": runtime_target_max,
        "oracle_target_abs_regret_max_m": oracle.get("metric_1_value", ""),
        "operating_mode_recommendation": operating.get("recommendation", ""),
        "next_exploration_hint": next_exploration_hint,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Calibration truth may be used only for offline validation/calibration replay selection. "
            "Runtime selection/exported profiles consume raw stereo-center RDF and measured disparity only."
        ),
    }
    return summary


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "RTK stereo calibration experiment dashboard",
        "",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Current Recommendation:",
        f"- runtime truth unavailable: {summary.get('current_truth_unavailable_recommendation', '')}",
        f"- calibration truth available: {summary.get('current_truth_available_recommendation', '')}",
        f"- runtime policy: {summary.get('current_runtime_recommendation', '')}",
        f"- runtime pass: {summary.get('runtime_pass_count', '')}/{summary.get('runtime_audit_count', '')}",
        f"- runtime target max abs worst-case: {summary.get('runtime_target_max_abs_error_max_m', '')} m",
        f"- oracle target abs regret max: {summary.get('oracle_target_abs_regret_max_m', '')} m",
        "",
        f"Next exploration: {summary.get('next_exploration_hint', '')}",
        "",
        "Rows:",
    ]
    for row in rows:
        lines.append(
            "- {area}: {recommendation}; pass={pass_count}/{audit_count}; {m1}={v1}; {m2}={v2}".format(
                area=row.get("area", ""),
                recommendation=row.get("recommendation", ""),
                pass_count=row.get("pass_count", ""),
                audit_count=row.get("audit_count", ""),
                m1=row.get("metric_1_name", ""),
                v1=row.get("metric_1_value", ""),
                m2=row.get("metric_2_name", ""),
                v2=row.get("metric_2_value", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_dashboard(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    plot_rows = [
        row
        for row in rows
        if str(row.get("pass_count", "")).strip() and str(row.get("audit_count", "")).strip()
    ]
    if not plot_rows:
        return
    labels = [str(row.get("area", "")) for row in plot_rows]
    pass_counts = np.asarray([finite_float(row.get("pass_count"), 0.0) for row in plot_rows], dtype=float)
    audit_counts = np.asarray([finite_float(row.get("audit_count"), 0.0) for row in plot_rows], dtype=float)
    ratios = np.divide(pass_counts, np.maximum(audit_counts, 1.0))
    fig, ax = plt.subplots(figsize=(max(10, len(labels) * 0.8), 5), constrained_layout=True)
    ax.bar(np.arange(len(labels)), ratios, color="tab:blue")
    ax.set_ylim(0.0, 1.05)
    ax.set_ylabel("pass ratio")
    ax.set_title("Consolidated experiment pass ratios")
    ax.set_xticks(np.arange(len(labels)), labels, rotation=35, ha="right")
    ax.grid(True, axis="y", alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def summarize_dashboard(sweep_dir: Path, output_dir: Path) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = dashboard_rows(sweep_dir)
    summary = build_summary(rows, sweep_dir)
    write_csv(output_dir / "rtk_stereo_experiment_dashboard.csv", rows)
    write_json(output_dir / "rtk_stereo_experiment_dashboard_summary.json", summary)
    write_summary_txt(output_dir / "rtk_stereo_experiment_dashboard_summary.txt", summary, rows)
    plot_dashboard(output_dir / "rtk_stereo_experiment_dashboard_pass_ratio.png", rows)
    return summary


def main() -> int:
    args = parse_args()
    summary = summarize_dashboard(args.sweep_dir, args.output_dir)
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
