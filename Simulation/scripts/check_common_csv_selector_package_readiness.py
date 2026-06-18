#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
RUNTIME_INPUT_CONTRACT = (
    "selector package uses deployment/scenario labels only for selection; selected runtime profiles consume "
    "stereo-center RDF(right,down,forward) plus measured disparity; PX4/RTK/NED truth is offline evidence only"
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check scenario-label selector package readiness evidence.")
    parser.add_argument("--selector-package-dir", type=Path, required=True)
    parser.add_argument("--portfolio-summary-json", type=Path, required=True)
    parser.add_argument("--frontier-json", type=Path, required=True)
    parser.add_argument("--operator-check-json", type=Path, required=True)
    parser.add_argument("--selection-json", type=Path, action="append", default=[])
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, payload: Mapping[str, object]) -> None:
    fields = list(payload.keys())
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow({key: encode_csv_value(value) for key, value in payload.items()})


def encode_csv_value(value: object) -> object:
    if isinstance(value, list):
        return ";".join(str(item) for item in value)
    return value


def finite_float(value: object, default: float = math.inf) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def selector_mode_row(portfolio_summary: Mapping[str, object], mode: str) -> dict[str, object]:
    rows = portfolio_summary.get("selector_modes", [])
    if not isinstance(rows, list):
        return {}
    for row in rows:
        if isinstance(row, Mapping) and row.get("selector_mode") == mode:
            return dict(row)
    return {}


def check_readiness(
    *,
    selector_package_dir: Path,
    portfolio_summary_json: Path,
    frontier_json: Path,
    operator_check_json: Path,
    selection_jsons: Sequence[Path],
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    failures: list[str] = []
    export_summary_path = selector_package_dir / "selector_package_export_summary.json"
    selector_json = selector_package_dir / "runtime_profile_label_selector.json"
    profiles_csv = selector_package_dir / "selector_package_profiles.csv"
    labels_csv = selector_package_dir / "selector_package_labels.csv"

    require(selector_package_dir.is_dir(), failures, "selector_package_dir:missing")
    require(export_summary_path.is_file(), failures, "selector_export_summary:missing")
    require(selector_json.is_file(), failures, "selector_json:missing")
    require(profiles_csv.is_file(), failures, "selector_profiles_csv:missing")
    require(labels_csv.is_file(), failures, "selector_labels_csv:missing")
    require(portfolio_summary_json.is_file(), failures, "portfolio_summary:missing")
    require(frontier_json.is_file(), failures, "frontier_json:missing")
    require(operator_check_json.is_file(), failures, "operator_check:missing")

    export_summary = load_json(export_summary_path) if export_summary_path.is_file() else {}
    selector_manifest = load_json(selector_json) if selector_json.is_file() else {}
    portfolio_summary = load_json(portfolio_summary_json) if portfolio_summary_json.is_file() else {}
    frontier = load_json(frontier_json) if frontier_json.is_file() else {}
    frontier_summary = frontier.get("summary", frontier)
    if not isinstance(frontier_summary, Mapping):
        frontier_summary = {}
    operator_check = load_json(operator_check_json) if operator_check_json.is_file() else {}
    selections = [load_json(path) for path in selection_jsons if path.is_file()]

    scenario_loso = selector_mode_row(portfolio_summary, "scenario_loso")
    global_single = selector_mode_row(portfolio_summary, "global_single")

    require(export_summary.get("status") == "PASS", failures, "selector_export_summary:status")
    require(not bool(export_summary.get("runtime_truth_used", True)), failures, "selector_export_summary:runtime_truth_used")
    require(selector_manifest.get("mode") == "stereodistance_runtime_profile_label_selector", failures, "selector_json:mode")
    require(not bool(selector_manifest.get("runtime_truth_used", True)), failures, "selector_json:runtime_truth_used")
    require("RDF(right,down,forward)" in str(selector_manifest.get("axis_contract", "")), failures, "selector_json:rdf_contract")
    require("FRD_range" in str(selector_manifest.get("public_topic_contract", "")), failures, "selector_json:public_contract")
    require(portfolio_summary.get("status") == "PASS", failures, "portfolio_summary:status")
    require(portfolio_summary.get("recommended_deployable_selector_mode") == "scenario_loso", failures, "portfolio_summary:recommended_mode")
    require(not bool(portfolio_summary.get("runtime_truth_used", True)), failures, "portfolio_summary:runtime_truth_used")
    require(bool(scenario_loso.get("pass_all_runs", False)), failures, "scenario_loso:pass_all_runs")
    require(bool(scenario_loso.get("target_limit_pass", False)), failures, "scenario_loso:target_limit")
    require(
        finite_float(scenario_loso.get("target_max_abs_error_corrected_max_m")) <= target_error_limit_m,
        failures,
        "scenario_loso:target_max_limit",
    )
    if global_single:
        require(
            finite_float(scenario_loso.get("validation_range_rmse_corrected_mean_m"))
            <= finite_float(global_single.get("validation_range_rmse_corrected_mean_m")),
            failures,
            "scenario_loso:rmse_mean_not_worse_than_global",
        )
    require(frontier_summary.get("status") == "PASS", failures, "frontier:status")
    require(not bool(frontier_summary.get("runtime_truth_used", True)), failures, "frontier:runtime_truth_used")
    require(bool(frontier_summary.get("truth_free_runtime_selection", False)), failures, "frontier:truth_free_selection")
    require(int(finite_float(frontier_summary.get("profile_count"), 0)) > 0, failures, "frontier:profile_count")
    require(int(finite_float(frontier_summary.get("deployment_label_count"), 0)) > 0, failures, "frontier:deployment_label_count")
    require(operator_check.get("status") == "PASS", failures, "operator_check:status")
    require(operator_check.get("source_mode") == "selector", failures, "operator_check:source_mode")
    require(operator_check.get("label_selection_status") == "PASS", failures, "operator_check:label_selection")
    require(operator_check.get("package_check_status") == "PASS", failures, "operator_check:package_check")
    require(not bool(operator_check.get("runtime_truth_used", True)), failures, "operator_check:runtime_truth_used")
    require(bool(operator_check.get("bbox_adjustment_arg_present", False)), failures, "operator_check:bbox_adjustment")
    require(selection_jsons and len(selections) == len(selection_jsons), failures, "selection_jsons:all_present")
    for selection in selections:
        label = selection.get("deployment_label", "")
        require(selection.get("status") == "PASS", failures, f"selection:{label}:status")
        require(bool(selection.get("label_selection_applied", False)), failures, f"selection:{label}:label_applied")
        require(not bool(selection.get("runtime_truth_used", True)), failures, f"selection:{label}:runtime_truth_used")
        require("RDF(right,down,forward)" in str(selection.get("axis_contract", "")), failures, f"selection:{label}:rdf_contract")

    payload = {
        "mode": "common_csv_selector_package_readiness",
        "status": "PASS" if not failures else "FAIL",
        "selector_package_dir": str(selector_package_dir),
        "selector_json": str(selector_json),
        "selector_export_summary_status": export_summary.get("status", ""),
        "portfolio_summary_status": portfolio_summary.get("status", ""),
        "recommended_deployable_selector_mode": portfolio_summary.get("recommended_deployable_selector_mode", ""),
        "scenario_loso_rmse_mean_m": scenario_loso.get("validation_range_rmse_corrected_mean_m", ""),
        "scenario_loso_rmse_max_m": scenario_loso.get("validation_range_rmse_corrected_max_m", ""),
        "scenario_loso_target_max_abs_m": scenario_loso.get("target_max_abs_error_corrected_max_m", ""),
        "global_single_rmse_mean_m": global_single.get("validation_range_rmse_corrected_mean_m", ""),
        "global_single_rmse_max_m": global_single.get("validation_range_rmse_corrected_max_m", ""),
        "global_single_target_max_abs_m": global_single.get("target_max_abs_error_corrected_max_m", ""),
        "frontier_status": frontier_summary.get("status", ""),
        "frontier_profile_count": frontier_summary.get("profile_count", 0),
        "frontier_deployment_label_count": frontier_summary.get("deployment_label_count", 0),
        "operator_check_status": operator_check.get("status", ""),
        "operator_selected_label": operator_check.get("selected_label", ""),
        "operator_package_type": operator_check.get("package_type", ""),
        "selection_count": len(selections),
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "failure_count": len(failures),
        "failures": failures,
    }
    return payload


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Common-CSV selector package readiness",
        f"status: {payload.get('status', '')}",
        f"selector_package_dir: {payload.get('selector_package_dir', '')}",
        f"selector_json: {payload.get('selector_json', '')}",
        f"recommended_deployable_selector_mode: {payload.get('recommended_deployable_selector_mode', '')}",
        f"scenario_loso_rmse_mean_m: {payload.get('scenario_loso_rmse_mean_m', '')}",
        f"scenario_loso_rmse_max_m: {payload.get('scenario_loso_rmse_max_m', '')}",
        f"scenario_loso_target_max_abs_m: {payload.get('scenario_loso_target_max_abs_m', '')}",
        f"global_single_rmse_mean_m: {payload.get('global_single_rmse_mean_m', '')}",
        f"global_single_rmse_max_m: {payload.get('global_single_rmse_max_m', '')}",
        f"global_single_target_max_abs_m: {payload.get('global_single_target_max_abs_m', '')}",
        f"frontier_status: {payload.get('frontier_status', '')}",
        f"frontier_profile_count: {payload.get('frontier_profile_count', 0)}",
        f"frontier_deployment_label_count: {payload.get('frontier_deployment_label_count', 0)}",
        f"operator_check_status: {payload.get('operator_check_status', '')}",
        f"operator_selected_label: {payload.get('operator_selected_label', '')}",
        f"operator_package_type: {payload.get('operator_package_type', '')}",
        f"selection_count: {payload.get('selection_count', 0)}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        f"runtime_input_contract: {payload.get('runtime_input_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.extend(["", "Failures:"])
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    payload = check_readiness(
        selector_package_dir=args.selector_package_dir,
        portfolio_summary_json=args.portfolio_summary_json,
        frontier_json=args.frontier_json,
        operator_check_json=args.operator_check_json,
        selection_jsons=args.selection_json,
        target_error_limit_m=args.target_error_limit_m,
    )
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    if args.output_csv is not None:
        write_csv(args.output_csv, payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
