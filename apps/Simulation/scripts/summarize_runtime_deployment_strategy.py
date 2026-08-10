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
SELECTION_CONTRACT = (
    "Deployment strategy selection is based on completed offline replay audits. Runtime execution uses "
    "fixed profiles and raw RDF/disparity/image-center features only; PX4/RTK truth is audit-only."
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize known-catalog and unknown-regime StereoDistance deployment recommendations "
            "from runtime guard, single pooled, and portfolio override audit artifacts."
        )
    )
    parser.add_argument("--known-catalog-summary-json", type=Path, required=True)
    parser.add_argument("--known-override-summary-json", type=Path, required=True)
    parser.add_argument("--single-pooled-candidate-csv", type=Path, required=True)
    parser.add_argument("--loso-override-summary-json", type=Path, required=True)
    parser.add_argument("--loso-single-pooled-candidate-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def finite_float(value: object, default: float = float("nan")) -> float:
    if value in (None, ""):
        return default
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def parse_bool(value: object) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes", "pass"}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def read_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def summarize_candidate_csv(path: Path, *, label: str) -> dict[str, object]:
    rows = read_csv(path)
    rmse = [finite_float(row.get("validation_range_rmse_corrected_m")) for row in rows]
    target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m")) for row in rows]
    target_max = [finite_float(row.get("target_max_abs_error_corrected_m")) for row in rows]
    rmse = [value for value in rmse if math.isfinite(value)]
    target_abs = [value for value in target_abs if math.isfinite(value)]
    target_max = [value for value in target_max if math.isfinite(value)]
    pass_count = sum(1 for row in rows if parse_bool(row.get("pass_runtime_measurement_guard_replay")))
    return {
        "strategy_label": label,
        "source_path": str(path),
        "audit_count": len(rows),
        "pass_count": pass_count,
        "pass_all": bool(rows) and pass_count == len(rows),
        "validation_range_rmse_corrected_mean_m": sum(rmse) / len(rmse) if rmse else float("inf"),
        "validation_range_rmse_corrected_max_m": max(rmse, default=float("inf")),
        "target_abs_mean_error_mean_m": sum(target_abs) / len(target_abs) if target_abs else float("inf"),
        "target_abs_mean_error_max_m": max(target_abs, default=float("inf")),
        "target_max_abs_error_max_m": max(target_max, default=float("inf")),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def summary_json_strategy(path: Path, *, label: str, metric_prefix: str = "") -> dict[str, object]:
    payload = read_json(path)
    prefix = f"{metric_prefix}_" if metric_prefix else ""
    target_max = payload.get(f"{prefix}target_max_abs_error_max_m", payload.get("target_max_abs_error_max_m"))
    rmse_mean = payload.get(
        f"{prefix}rmse_mean_m",
        payload.get(f"{prefix}validation_range_rmse_corrected_mean_m", payload.get("validation_range_rmse_corrected_mean_m")),
    )
    rmse_max = payload.get(
        f"{prefix}validation_range_rmse_corrected_max_m",
        payload.get("validation_range_rmse_corrected_max_m"),
    )
    pass_count = payload.get(f"{prefix}pass_count", payload.get("pass_count", 0))
    return {
        "strategy_label": label,
        "source_path": str(path),
        "audit_count": int(float(payload.get("audit_count", 0) or 0)),
        "pass_count": int(float(pass_count or 0)),
        "pass_all": parse_bool(payload.get("pass_all", False))
        or (
            int(float(payload.get("audit_count", 0) or 0)) > 0
            and int(float(pass_count or 0)) == int(float(payload.get("audit_count", 0) or 0))
        ),
        "validation_range_rmse_corrected_mean_m": finite_float(rmse_mean, float("inf")),
        "validation_range_rmse_corrected_max_m": finite_float(rmse_max, float("inf")),
        "target_abs_mean_error_mean_m": finite_float(payload.get("target_abs_mean_error_mean_m"), float("nan")),
        "target_abs_mean_error_max_m": finite_float(payload.get("target_abs_mean_error_max_m"), float("nan")),
        "target_max_abs_error_max_m": finite_float(target_max, float("inf")),
        "override_label": payload.get("override_label", ""),
        "baseline_label": payload.get("baseline_label", ""),
        "rule_feature_name": payload.get("rule_feature_name", ""),
        "rule_direction": payload.get("rule_direction", ""),
        "rule_threshold": payload.get("rule_threshold", ""),
        "override_count": payload.get("override_count", ""),
        "rule_false_positive_count": payload.get("rule_false_positive_count", ""),
        "selection_contract": payload.get("selection_contract", ""),
        "axis_contract": payload.get("axis_contract", RDF_AXIS_CONTRACT),
        "public_topic_contract": payload.get("public_topic_contract", PUBLIC_TOPIC_CONTRACT),
    }


def strategy_score(row: Mapping[str, object], *, target_error_limit_m: float) -> tuple[object, ...]:
    pass_all = parse_bool(row.get("pass_all"))
    target_max = finite_float(row.get("target_max_abs_error_max_m"), float("inf"))
    rmse_mean = finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf"))
    rmse_max = finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf"))
    over_target = max(0.0, target_max - target_error_limit_m)
    return (0 if pass_all else 1, over_target, target_max, rmse_mean, rmse_max, str(row.get("strategy_label", "")))


def choose_strategy(rows: Sequence[Mapping[str, object]], *, target_error_limit_m: float) -> Mapping[str, object]:
    if not rows:
        raise ValueError("no strategy rows to choose from")
    return min(rows, key=lambda row: strategy_score(row, target_error_limit_m=target_error_limit_m))


def summarize_runtime_deployment_strategy(
    *,
    known_catalog_summary_json: Path,
    known_override_summary_json: Path,
    single_pooled_candidate_csv: Path,
    loso_override_summary_json: Path,
    loso_single_pooled_candidate_csv: Path,
    output_dir: Path,
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    known_rows = [
        summary_json_strategy(known_catalog_summary_json, label="known_catalog_guard"),
        summary_json_strategy(known_override_summary_json, label="known_catalog_guard_with_single_pooled_override"),
        summarize_candidate_csv(single_pooled_candidate_csv, label="single_pooled_runtime_profile"),
    ]
    unknown_rows = [
        summary_json_strategy(
            loso_override_summary_json,
            label="loso_guard_with_single_pooled_override",
        ),
        summarize_candidate_csv(loso_single_pooled_candidate_csv, label="single_pooled_runtime_profile"),
    ]
    for row in known_rows:
        row["deployment_mode"] = "known_catalog_available"
        row["target_error_limit_m"] = target_error_limit_m
    for row in unknown_rows:
        row["deployment_mode"] = "unknown_or_unseen_regime"
        row["target_error_limit_m"] = target_error_limit_m

    known_best = dict(choose_strategy(known_rows, target_error_limit_m=target_error_limit_m))
    unknown_best = dict(choose_strategy(unknown_rows, target_error_limit_m=target_error_limit_m))
    rows_out = [*known_rows, *unknown_rows]
    recommendation = {
        "mode": "runtime_deployment_strategy_recommendation",
        "known_catalog_recommendation": known_best.get("strategy_label", ""),
        "unknown_regime_recommendation": unknown_best.get("strategy_label", ""),
        "known_catalog_pass_all": known_best.get("pass_all", False),
        "unknown_regime_pass_all": unknown_best.get("pass_all", False),
        "known_catalog_target_max_abs_error_max_m": known_best.get("target_max_abs_error_max_m", ""),
        "unknown_regime_target_max_abs_error_max_m": unknown_best.get("target_max_abs_error_max_m", ""),
        "known_catalog_validation_range_rmse_corrected_mean_m": known_best.get(
            "validation_range_rmse_corrected_mean_m", ""
        ),
        "unknown_regime_validation_range_rmse_corrected_mean_m": unknown_best.get(
            "validation_range_rmse_corrected_mean_m", ""
        ),
        "known_catalog_runtime_rule": runtime_rule_text(known_best),
        "unknown_regime_runtime_rule": runtime_rule_text(unknown_best),
        "target_error_limit_m": target_error_limit_m,
        "recommendation_reason": (
            "known catalog mode chooses the pass-all strategy with the lowest 1852m-near target max error; "
            "unknown/unseen mode applies the same criterion and currently favors the single pooled fallback "
            "when LOSO guard composition is weaker."
        ),
        "selection_contract": SELECTION_CONTRACT,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }
    write_csv(output_dir / "runtime_deployment_strategy_candidates.csv", rows_out)
    write_json(output_dir / "runtime_deployment_strategy_recommendation.json", recommendation)
    write_summary_txt(output_dir / "runtime_deployment_strategy_recommendation.txt", recommendation, rows_out)
    return recommendation


def runtime_rule_text(row: Mapping[str, object]) -> str:
    label = str(row.get("strategy_label", ""))
    feature = str(row.get("rule_feature_name", "")).strip()
    direction = str(row.get("rule_direction", "")).strip()
    threshold = row.get("rule_threshold", "")
    if feature and direction and threshold not in ("", None):
        return f"use {row.get('baseline_label', 'baseline')} unless {feature} {direction} {threshold}, then use {row.get('override_label', 'override')}"
    if label == "single_pooled_runtime_profile":
        return "use the validated single pooled runtime profile; no runtime selector truth is required"
    return "use the validated runtime guard/catalog selector; no runtime truth is required"


def write_summary_txt(
    path: Path,
    recommendation: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance runtime deployment strategy recommendation",
        "",
        f"known_catalog_recommendation: {recommendation.get('known_catalog_recommendation', '')}",
        f"known_catalog_runtime_rule: {recommendation.get('known_catalog_runtime_rule', '')}",
        f"known_catalog_target_max_abs_error_max_m: {recommendation.get('known_catalog_target_max_abs_error_max_m', '')}",
        f"unknown_regime_recommendation: {recommendation.get('unknown_regime_recommendation', '')}",
        f"unknown_regime_runtime_rule: {recommendation.get('unknown_regime_runtime_rule', '')}",
        f"unknown_regime_target_max_abs_error_max_m: {recommendation.get('unknown_regime_target_max_abs_error_max_m', '')}",
        f"target_error_limit_m: {recommendation.get('target_error_limit_m', '')}",
        f"selection_contract: {recommendation.get('selection_contract', '')}",
        f"axis_contract: {recommendation.get('axis_contract', '')}",
        f"public_topic_contract: {recommendation.get('public_topic_contract', '')}",
        "",
        f"recommendation_reason: {recommendation.get('recommendation_reason', '')}",
        "",
        "Candidates:",
    ]
    for row in rows:
        lines.append(
            (
                "- {mode}/{label}: pass={passed}/{total}, pass_all={pass_all}, "
                "rmse_mean={rmse_mean:.6f}m, rmse_max={rmse_max:.6f}m, target_max={target_max:.6f}m, "
                "rule={rule}"
            ).format(
                mode=row.get("deployment_mode", ""),
                label=row.get("strategy_label", ""),
                passed=row.get("pass_count", ""),
                total=row.get("audit_count", ""),
                pass_all=row.get("pass_all", ""),
                rmse_mean=finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
                rmse_max=finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf")),
                target_max=finite_float(row.get("target_max_abs_error_max_m"), float("inf")),
                rule=runtime_rule_text(row),
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    recommendation = summarize_runtime_deployment_strategy(
        known_catalog_summary_json=args.known_catalog_summary_json,
        known_override_summary_json=args.known_override_summary_json,
        single_pooled_candidate_csv=args.single_pooled_candidate_csv,
        loso_override_summary_json=args.loso_override_summary_json,
        loso_single_pooled_candidate_csv=args.loso_single_pooled_candidate_csv,
        output_dir=args.output_dir,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(recommendation, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
