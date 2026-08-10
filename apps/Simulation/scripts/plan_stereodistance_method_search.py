#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "runtime StereoDistance profiles consume stereo-center RDF(right,down,forward) plus measured disparity; "
    "calibration/evaluation truth may be PX4 local NED but is not a runtime input"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan the next StereoDistance correction-method search from robustness and deployment evidence. "
            "The planner does not fit a new correction; it ranks method families and emits the next "
            "automatic exploration axes while preserving runtime truth independence."
        )
    )
    parser.add_argument("--robustness-summary-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dashboard-summary-json", type=Path)
    parser.add_argument(
        "--active-profile-json",
        type=Path,
        help=(
            "Optional exported active_runtime_profile.json. When provided, the planner labels this "
            "runtime profile method separately from the robustness frontier recommendation."
        ),
    )
    parser.add_argument("--top-n", type=int, default=20)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--max-axis-regression-sum-m", type=float, default=0.0)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def finite_or_blank(value: object) -> float | str:
    parsed = finite_float(value, float("nan"))
    return parsed if math.isfinite(parsed) else ""


def method_family_key(row: Mapping[str, object]) -> tuple[str, str, str, str]:
    return (
        str(row.get("registration_model", "")),
        str(row.get("residual_model", "")),
        str(row.get("regularization_model", "")),
        str(row.get("runtime_profile_family", "")),
    )


def method_rank_score(row: Mapping[str, object], *, target_error_limit_m: float) -> tuple[object, ...]:
    expected = int(finite_float(row.get("expected_scenario_count"), 0.0))
    strict = int(finite_float(row.get("strict_pass_count"), 0.0))
    performance = int(finite_float(row.get("performance_pass_count"), 0.0))
    strict_all = parse_bool(row.get("strict_pass_all_scenarios")) or (expected > 0 and strict >= expected)
    performance_all = parse_bool(row.get("performance_pass_all_scenarios")) or (
        expected > 0 and performance >= expected
    )
    target_max = finite_float(row.get("target_abs_mean_error_max_m"), float("inf"))
    target_ok = target_max <= target_error_limit_m
    axis_regression = finite_float(row.get("axis_rmse_regression_sum_mean_m"), float("inf"))
    return (
        0 if strict_all and target_ok and axis_regression <= 0.0 else 1,
        0 if performance_all and target_ok else 1,
        -strict,
        -performance,
        finite_float(row.get("robust_score"), float("inf")),
        finite_float(row.get("validation_range_rmse_mean_m"), float("inf")),
        target_max,
        axis_regression,
        finite_float(row.get("model_complexity_mean"), float("inf")),
        str(row.get("method", "")),
    )


def action_for_family(
    *,
    strict_pass_count: int,
    performance_pass_count: int,
    expected_scenario_count: int,
    axis_regression_sum_max_m: float,
    target_abs_error_max_m: float,
    best_method: str,
    target_error_limit_m: float,
    active_method: str,
) -> tuple[str, str]:
    if axis_regression_sum_max_m > 0.0:
        return (
            "diagnose_axis_regression",
            "Axis RMSE regression was observed; keep the family out of deployment search until RDF axes improve.",
        )
    if expected_scenario_count > 0 and strict_pass_count >= expected_scenario_count:
        if best_method == active_method:
            return (
                "exploit_active_family",
                "Active PASS method is in this family; run a narrow hyperparameter sweep around its robust settings.",
            )
        return (
            "promote_as_alternate_frontier",
            "Family passed every scenario; keep it as a deployment alternate and test transfer against active profile.",
        )
    if expected_scenario_count > 0 and performance_pass_count >= expected_scenario_count:
        return (
            "add_stability_regularization",
            "Family meets performance but not strict stability; sweep shrinkage, Huber/TLS thresholds, or target weighting.",
        )
    if target_abs_error_max_m <= target_error_limit_m:
        return (
            "expand_stress_coverage",
            "Target-window error is acceptable but coverage is incomplete; replay on missing scenarios before promotion.",
        )
    return (
        "deprioritize_until_target_error_fixed",
        "Target-window error exceeds the limit; search residual disparity/range terms before deployment consideration.",
    )


def build_method_frontier(
    methods: Sequence[Mapping[str, object]],
    *,
    top_n: int,
    target_error_limit_m: float,
    active_method: str,
) -> list[dict[str, object]]:
    ranked = sorted(methods, key=lambda row: method_rank_score(row, target_error_limit_m=target_error_limit_m))
    rows: list[dict[str, object]] = []
    for rank, row in enumerate(ranked[:top_n], start=1):
        expected = int(finite_float(row.get("expected_scenario_count"), 0.0))
        strict = int(finite_float(row.get("strict_pass_count"), 0.0))
        performance = int(finite_float(row.get("performance_pass_count"), 0.0))
        target_max = finite_float(row.get("target_abs_mean_error_max_m"), float("inf"))
        axis_regression = finite_float(row.get("axis_rmse_regression_sum_mean_m"), float("inf"))
        rows.append(
            {
                "rank": rank,
                "method": row.get("method", ""),
                "is_active_method": str(row.get("method", "")) == active_method,
                "strict_pass_count": strict,
                "performance_pass_count": performance,
                "expected_scenario_count": expected,
                "strict_all_scenarios": expected > 0 and strict >= expected,
                "target_abs_mean_error_max_m": finite_or_blank(row.get("target_abs_mean_error_max_m")),
                "validation_range_rmse_mean_m": finite_or_blank(row.get("validation_range_rmse_mean_m")),
                "validation_range_rmse_max_m": finite_or_blank(row.get("validation_range_rmse_max_m")),
                "axis_rmse_regression_sum_mean_m": finite_or_blank(row.get("axis_rmse_regression_sum_mean_m")),
                "model_complexity_mean": finite_or_blank(row.get("model_complexity_mean")),
                "robust_score": finite_or_blank(row.get("robust_score")),
                "target_error_limit_pass": target_max <= target_error_limit_m,
                "rdf_axis_nonregression_pass": axis_regression <= 0.0,
                "registration_model": row.get("registration_model", ""),
                "residual_model": row.get("residual_model", ""),
                "regularization_model": row.get("regularization_model", ""),
                "runtime_profile_family": row.get("runtime_profile_family", ""),
                "selection_note": row.get("selection_note", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
            }
        )
    return rows


def build_family_priorities(
    methods: Sequence[Mapping[str, object]],
    *,
    target_error_limit_m: float,
    active_method: str,
) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str, str, str], list[Mapping[str, object]]] = {}
    for row in methods:
        grouped.setdefault(method_family_key(row), []).append(row)

    priority_rows: list[dict[str, object]] = []
    for key, rows in grouped.items():
        best = min(rows, key=lambda row: method_rank_score(row, target_error_limit_m=target_error_limit_m))
        expected = max(int(finite_float(row.get("expected_scenario_count"), 0.0)) for row in rows)
        strict_best = max(int(finite_float(row.get("strict_pass_count"), 0.0)) for row in rows)
        performance_best = max(int(finite_float(row.get("performance_pass_count"), 0.0)) for row in rows)
        axis_max = max(finite_float(row.get("axis_rmse_regression_sum_mean_m"), float("inf")) for row in rows)
        target_max = min(finite_float(row.get("target_abs_mean_error_max_m"), float("inf")) for row in rows)
        action, reason = action_for_family(
            strict_pass_count=strict_best,
            performance_pass_count=performance_best,
            expected_scenario_count=expected,
            axis_regression_sum_max_m=axis_max,
            target_abs_error_max_m=target_max,
            best_method=str(best.get("method", "")),
            target_error_limit_m=target_error_limit_m,
            active_method=active_method,
        )
        priority_rows.append(
            {
                "registration_model": key[0],
                "residual_model": key[1],
                "regularization_model": key[2],
                "runtime_profile_family": key[3],
                "method_count": len(rows),
                "best_method": best.get("method", ""),
                "best_is_active_method": str(best.get("method", "")) == active_method,
                "recommended_next_action": action,
                "recommendation_reason": reason,
                "strict_pass_count_best": strict_best,
                "performance_pass_count_best": performance_best,
                "expected_scenario_count": expected,
                "target_abs_mean_error_best_max_m": target_max if math.isfinite(target_max) else "",
                "validation_range_rmse_best_mean_m": finite_or_blank(best.get("validation_range_rmse_mean_m")),
                "axis_rmse_regression_sum_max_m": axis_max if math.isfinite(axis_max) else "",
                "model_complexity_best_mean": finite_or_blank(best.get("model_complexity_mean")),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
            }
        )
    priority_order = {
        "exploit_active_family": 0,
        "promote_as_alternate_frontier": 1,
        "add_stability_regularization": 2,
        "expand_stress_coverage": 3,
        "deprioritize_until_target_error_fixed": 4,
        "diagnose_axis_regression": 5,
    }
    return sorted(
        priority_rows,
        key=lambda row: (
            priority_order.get(str(row.get("recommended_next_action", "")), 99),
            finite_float(row.get("validation_range_rmse_best_mean_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_best_max_m"), float("inf")),
            finite_float(row.get("model_complexity_best_mean"), float("inf")),
            str(row.get("best_method", "")),
        ),
    )


def active_method_from_evidence(
    robustness_summary: Mapping[str, object],
    dashboard_summary: Mapping[str, object] | None,
    active_profile: Mapping[str, object] | None = None,
) -> str:
    if active_profile:
        method = str(active_profile.get("method", "") or "").strip()
        if method:
            return method
    if dashboard_summary:
        method = str(dashboard_summary.get("active_method", "") or "").strip()
        if method:
            return method
    recommendation = robustness_summary.get("robust_recommendation", {})
    if isinstance(recommendation, Mapping):
        return str(recommendation.get("method", "") or "").strip()
    return ""


def plan_method_search(
    *,
    robustness_summary_json: Path,
    output_dir: Path,
    dashboard_summary_json: Path | None = None,
    active_profile_json: Path | None = None,
    top_n: int = 20,
    target_error_limit_m: float = 10.0,
    max_axis_regression_sum_m: float = 0.0,
) -> dict[str, object]:
    robustness_summary = read_json(robustness_summary_json)
    dashboard_summary = read_json(dashboard_summary_json) if dashboard_summary_json else None
    active_profile = read_json(active_profile_json) if active_profile_json else None
    methods = robustness_summary.get("robust_method_summary", [])
    if not isinstance(methods, list) or not methods:
        raise ValueError("robustness summary requires a non-empty robust_method_summary list")
    active_method = active_method_from_evidence(robustness_summary, dashboard_summary, active_profile)
    robust_recommendation = robustness_summary.get("robust_recommendation", {})
    robust_recommendation_method = (
        str(robust_recommendation.get("method", "") or "").strip()
        if isinstance(robust_recommendation, Mapping)
        else ""
    )
    frontier_rows = build_method_frontier(
        methods,
        top_n=top_n,
        target_error_limit_m=target_error_limit_m,
        active_method=active_method,
    )
    family_rows = build_family_priorities(
        methods,
        target_error_limit_m=target_error_limit_m,
        active_method=active_method,
    )
    strict_all_count = sum(1 for row in methods if parse_bool(row.get("strict_pass_all_scenarios")))
    axis_clean_count = sum(
        1
        for row in methods
        if finite_float(row.get("axis_rmse_regression_sum_mean_m"), float("inf")) <= max_axis_regression_sum_m
    )
    active_frontier_row = next((row for row in frontier_rows if row.get("is_active_method")), None)
    active_method_row = next((row for row in methods if str(row.get("method", "")) == active_method), None)
    active_axis_nonregression = (
        active_method_row is not None
        and finite_float(active_method_row.get("axis_rmse_regression_sum_mean_m"), float("inf"))
        <= max_axis_regression_sum_m
    )
    dashboard_axis_status = (
        str(dashboard_summary.get("final_rdf_axis_nonregression_status", "") or "").strip()
        if dashboard_summary
        else ""
    )
    dashboard_axis_nonregression = dashboard_axis_status == "PASS"
    active_profile_axis_nonregression = active_axis_nonregression or dashboard_axis_nonregression
    active_expected = int(finite_float(active_method_row.get("expected_scenario_count"), 0.0)) if active_method_row else 0
    active_strict = int(finite_float(active_method_row.get("strict_pass_count"), 0.0)) if active_method_row else 0
    active_strict_all = bool(active_expected > 0 and active_strict >= active_expected)
    summary = {
        "status": "PASS" if active_method_row is not None and active_profile_axis_nonregression else "WARN",
        "planner": "stereodistance_method_search_priority",
        "robustness_summary_json": str(robustness_summary_json),
        "dashboard_summary_json": str(dashboard_summary_json) if dashboard_summary_json else "",
        "active_profile_json": str(active_profile_json) if active_profile_json else "",
        "active_method": active_method,
        "active_method_source": (
            "active_profile_json"
            if active_profile_json
            else ("dashboard_summary_json" if dashboard_summary_json else "robust_recommendation")
        ),
        "robust_recommendation_method": robust_recommendation_method,
        "active_profile_matches_robust_recommendation": (
            bool(active_method and robust_recommendation_method and active_method == robust_recommendation_method)
        ),
        "active_method_found": active_method_row is not None,
        "active_method_in_top_frontier": bool(active_frontier_row),
        "active_method_strict_all_scenarios": active_strict_all,
        "active_method_summary_rdf_axis_nonregression_pass": active_axis_nonregression,
        "dashboard_rdf_axis_nonregression_status": dashboard_axis_status,
        "dashboard_rdf_axis_nonregression_pass": dashboard_axis_nonregression,
        "active_method_rdf_axis_nonregression_pass": active_profile_axis_nonregression,
        "method_count": len(methods),
        "frontier_method_count": len(frontier_rows),
        "family_count": len(family_rows),
        "strict_all_scenario_method_count": strict_all_count,
        "rdf_axis_nonregression_method_count": axis_clean_count,
        "target_error_limit_m": target_error_limit_m,
        "max_axis_regression_sum_m": max_axis_regression_sum_m,
        "top_next_actions": [
            {
                "action": row.get("recommended_next_action", ""),
                "family_best_method": row.get("best_method", ""),
                "reason": row.get("recommendation_reason", ""),
            }
            for row in family_rows[:5]
        ],
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "method_search_frontier.csv", frontier_rows)
    write_csv(output_dir / "family_search_priorities.csv", family_rows)
    write_json(output_dir / "method_search_plan.json", summary)
    write_summary_txt(output_dir / "method_search_plan.txt", summary, frontier_rows, family_rows)
    return summary


def write_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    frontier_rows: Sequence[Mapping[str, object]],
    family_rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance method search plan",
        "",
        f"status: {summary.get('status', '')}",
        f"active_method: {summary.get('active_method', '')}",
        f"active_method_source: {summary.get('active_method_source', '')}",
        f"robust_recommendation_method: {summary.get('robust_recommendation_method', '')}",
        "active_profile_matches_robust_recommendation: "
        f"{summary.get('active_profile_matches_robust_recommendation', '')}",
        f"active_method_found: {summary.get('active_method_found', '')}",
        f"active_method_in_top_frontier: {summary.get('active_method_in_top_frontier', '')}",
        f"active_method_strict_all_scenarios: {summary.get('active_method_strict_all_scenarios', '')}",
        "active_method_summary_rdf_axis_nonregression_pass: "
        f"{summary.get('active_method_summary_rdf_axis_nonregression_pass', '')}",
        f"dashboard_rdf_axis_nonregression_status: {summary.get('dashboard_rdf_axis_nonregression_status', '')}",
        f"active_method_rdf_axis_nonregression_pass: {summary.get('active_method_rdf_axis_nonregression_pass', '')}",
        f"method_count: {summary.get('method_count', '')}",
        f"family_count: {summary.get('family_count', '')}",
        f"strict_all_scenario_method_count: {summary.get('strict_all_scenario_method_count', '')}",
        f"rdf_axis_nonregression_method_count: {summary.get('rdf_axis_nonregression_method_count', '')}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "",
        "Top method frontier:",
    ]
    for row in frontier_rows[:10]:
        lines.append(
            (
                "- #{rank} {method}: strict={strict}/{expected}, rmse_mean={rmse}, "
                "target_max={target}, axis_nonreg={axis_ok}, active={active}"
            ).format(
                rank=row.get("rank", ""),
                method=row.get("method", ""),
                strict=row.get("strict_pass_count", ""),
                expected=row.get("expected_scenario_count", ""),
                rmse=row.get("validation_range_rmse_mean_m", ""),
                target=row.get("target_abs_mean_error_max_m", ""),
                axis_ok=row.get("rdf_axis_nonregression_pass", ""),
                active=row.get("is_active_method", ""),
            )
        )
    lines.extend(["", "Family priorities:"])
    for row in family_rows[:10]:
        lines.append(
            (
                "- {action}: best={method}, strict={strict}/{expected}, rmse_mean={rmse}, "
                "target_best_max={target}, reason={reason}"
            ).format(
                action=row.get("recommended_next_action", ""),
                method=row.get("best_method", ""),
                strict=row.get("strict_pass_count_best", ""),
                expected=row.get("expected_scenario_count", ""),
                rmse=row.get("validation_range_rmse_best_mean_m", ""),
                target=row.get("target_abs_mean_error_best_max_m", ""),
                reason=row.get("recommendation_reason", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    plan_method_search(
        robustness_summary_json=args.robustness_summary_json,
        dashboard_summary_json=args.dashboard_summary_json,
        active_profile_json=args.active_profile_json,
        output_dir=args.output_dir,
        top_n=args.top_n,
        target_error_limit_m=args.target_error_limit_m,
        max_axis_regression_sum_m=args.max_axis_regression_sum_m,
    )
    print(f"Wrote method search plan to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
