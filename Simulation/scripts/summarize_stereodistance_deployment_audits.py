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
DEFAULT_SIMPLICITY_TIE_MARGIN = 0.01


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare post-sweep StereoDistance deployment audits. This summarizes single-profile "
            "calibration replay selection against blend-policy replay selection without using truth "
            "at runtime."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--selection-audit-dir", type=Path)
    parser.add_argument("--blend-audit-dir", type=Path)
    return parser.parse_args()


def finite_float(value: object, default: float = math.nan) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def read_json(path: Path) -> dict[str, object]:
    if not path.exists():
        return {}
    payload = json.loads(path.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def mean(values: Sequence[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / len(finite) if finite else float("inf")


def metric_score(row: Mapping[str, object]) -> float:
    return (
        finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf"))
        + 0.30 * finite_float(row.get("target_abs_mean_error_mean_m"), float("inf"))
        + 0.15 * finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf"))
        + 0.15 * finite_float(row.get("target_abs_mean_error_max_m"), float("inf"))
    )


def strategy_complexity(row: Mapping[str, object]) -> int:
    strategy = str(row.get("strategy", ""))
    if strategy.startswith("single_profile"):
        return 1
    if strategy.startswith("blend_policy"):
        return 2
    return 9


def selection_strategy_row(selection_runs_csv: Path, selection_summary_json: Path | None = None) -> dict[str, object]:
    rows = read_csv(selection_runs_csv)
    summary = read_json(selection_summary_json) if selection_summary_json is not None else {}
    candidate_source_mode = str(summary.get("candidate_source_mode", "")).strip()
    if not candidate_source_mode and rows:
        candidate_source_mode = str(rows[0].get("candidate_source_mode", "")).strip()
    if not candidate_source_mode:
        candidate_source_mode = "unknown"
    rmse = [finite_float(row.get("calibration_validation_range_rmse_corrected_m"), float("nan")) for row in rows]
    target = [finite_float(row.get("calibration_target_abs_mean_error_corrected_m"), float("nan")) for row in rows]
    pass_count = sum(1 for row in rows if str(row.get("calibration_pass", "")).lower() == "true")
    result = {
        "strategy": f"single_profile_calibration_replay_{candidate_source_mode}",
        "policy": "single_best",
        "candidate_source_mode": candidate_source_mode,
        "strategy_complexity": 1,
        "run_count": len(rows),
        "pass_count": pass_count,
        "validation_range_rmse_corrected_mean_m": mean(rmse),
        "validation_range_rmse_corrected_max_m": max([value for value in rmse if math.isfinite(value)], default=float("inf")),
        "target_abs_mean_error_mean_m": mean(target),
        "target_abs_mean_error_max_m": max([value for value in target if math.isfinite(value)], default=float("inf")),
        "source_csv": str(selection_runs_csv),
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "selected runtime profile consumes raw RDF/disparity only; no runtime truth",
    }
    result["deployment_audit_score"] = metric_score(result)
    return result


def blend_strategy_rows(blend_policy_summary_csv: Path, blend_summary_json: Path | None = None) -> list[dict[str, object]]:
    rows = read_csv(blend_policy_summary_csv)
    summary = read_json(blend_summary_json) if blend_summary_json is not None else {}
    candidate_source_mode = str(summary.get("candidate_source_mode", "")).strip() or "unknown"
    result: list[dict[str, object]] = []
    for row in rows:
        strategy_row = {
            "strategy": f"blend_policy_calibration_replay_{candidate_source_mode}",
            "policy": row.get("policy", ""),
            "candidate_source_mode": candidate_source_mode,
            "strategy_complexity": 2,
            "run_count": int(float(row.get("run_count", 0) or 0)),
            "pass_count": int(float(row.get("pass_count", 0) or 0)),
            "validation_range_rmse_corrected_mean_m": finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
            "validation_range_rmse_corrected_max_m": finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf")),
            "target_abs_mean_error_mean_m": finite_float(row.get("target_abs_mean_error_mean_m"), float("inf")),
            "target_abs_mean_error_max_m": finite_float(row.get("target_abs_mean_error_max_m"), float("inf")),
            "source_csv": str(blend_policy_summary_csv),
            "alpha_values": row.get("alpha_values", ""),
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": "selected blended runtime profile consumes raw RDF/disparity only; no runtime truth",
        }
        strategy_row["deployment_audit_score"] = metric_score(strategy_row)
        result.append(strategy_row)
    return result


def choose_recommendation(
    rows: Sequence[Mapping[str, object]],
    *,
    simplicity_tie_margin: float = DEFAULT_SIMPLICITY_TIE_MARGIN,
) -> dict[str, object]:
    if not rows:
        return {
            "selected": False,
            "reason": "no audit strategy rows",
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        }
    best_score_row = min(
        rows,
        key=lambda row: (
            -int(row.get("pass_count", 0) or 0),
            finite_float(row.get("deployment_audit_score"), float("inf")),
            finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_mean_m"), float("inf")),
            str(row.get("strategy", "")),
            str(row.get("policy", "")),
        ),
    )
    best_pass_count = int(best_score_row.get("pass_count", 0) or 0)
    best_score = finite_float(best_score_row.get("deployment_audit_score"), float("inf"))
    tied_rows = [
        row for row in rows
        if int(row.get("pass_count", 0) or 0) == best_pass_count
        and finite_float(row.get("deployment_audit_score"), float("inf")) <= best_score + max(float(simplicity_tie_margin), 0.0)
    ]
    best = min(
        tied_rows,
        key=lambda row: (
            int(row.get("strategy_complexity", strategy_complexity(row)) or strategy_complexity(row)),
            finite_float(row.get("deployment_audit_score"), float("inf")),
            finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_mean_m"), float("inf")),
            str(row.get("strategy", "")),
            str(row.get("policy", "")),
        ),
    )
    selected_score = finite_float(best.get("deployment_audit_score"), float("inf"))
    score_margin_to_best = selected_score - best_score if math.isfinite(selected_score) and math.isfinite(best_score) else float("nan")
    return {
        "selected": True,
        "recommended_strategy": best.get("strategy", ""),
        "recommended_policy": best.get("policy", ""),
        "deployment_audit_score": best.get("deployment_audit_score", ""),
        "best_score_strategy": best_score_row.get("strategy", ""),
        "best_score_policy": best_score_row.get("policy", ""),
        "score_margin_to_best_m": score_margin_to_best,
        "simplicity_tie_margin_m": float(simplicity_tie_margin),
        "strategy_complexity": int(best.get("strategy_complexity", strategy_complexity(best)) or strategy_complexity(best)),
        "validation_range_rmse_corrected_mean_m": best.get("validation_range_rmse_corrected_mean_m", ""),
        "validation_range_rmse_corrected_max_m": best.get("validation_range_rmse_corrected_max_m", ""),
        "target_abs_mean_error_mean_m": best.get("target_abs_mean_error_mean_m", ""),
        "target_abs_mean_error_max_m": best.get("target_abs_mean_error_max_m", ""),
        "recommendation_reason": (
            "selected lower-complexity strategy within score tie margin"
            if best is not best_score_row
            else "selected lowest deployment audit score among highest pass-count strategies"
        ),
        "selection_contract": (
            "Calibration-time truth may choose a profile or blend policy from completed audits. Runtime "
            "application must use only raw RDF/disparity and fixed profile parameters."
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def write_summary_txt(path: Path, recommendation: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "StereoDistance deployment audit strategy summary",
        "",
        f"axis_contract: {recommendation.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {recommendation.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {recommendation.get('selection_contract', '')}",
        "",
        (
            "recommended: {strategy}/{policy}, score={score}, rmse_mean={rmse_mean}, "
            "rmse_max={rmse_max}, target_mean={target_mean}, target_max={target_max}"
        ).format(
            strategy=recommendation.get("recommended_strategy", ""),
            policy=recommendation.get("recommended_policy", ""),
            score=recommendation.get("deployment_audit_score", ""),
            rmse_mean=recommendation.get("validation_range_rmse_corrected_mean_m", ""),
            rmse_max=recommendation.get("validation_range_rmse_corrected_max_m", ""),
            target_mean=recommendation.get("target_abs_mean_error_mean_m", ""),
            target_max=recommendation.get("target_abs_mean_error_max_m", ""),
        ),
        f"best_score: {recommendation.get('best_score_strategy', '')}/{recommendation.get('best_score_policy', '')}",
        f"score_margin_to_best_m: {recommendation.get('score_margin_to_best_m', '')}",
        f"simplicity_tie_margin_m: {recommendation.get('simplicity_tie_margin_m', '')}",
        f"recommendation_reason: {recommendation.get('recommendation_reason', '')}",
        "",
        "Candidates:",
    ]
    for row in rows:
        lines.append(
            (
                "- {strategy}/{policy}: pass={passed}/{runs}, score={score:.6f}, "
                "rmse_mean={rmse_mean:.6f}m, rmse_max={rmse_max:.6f}m, "
                "target_mean={target_mean:.6f}m, target_max={target_max:.6f}m"
            ).format(
                strategy=row.get("strategy", ""),
                policy=row.get("policy", ""),
                passed=row.get("pass_count", 0),
                runs=row.get("run_count", 0),
                score=finite_float(row.get("deployment_audit_score"), float("inf")),
                rmse_mean=finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
                rmse_max=finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf")),
                target_mean=finite_float(row.get("target_abs_mean_error_mean_m"), float("inf")),
                target_max=finite_float(row.get("target_abs_mean_error_max_m"), float("inf")),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_deployment_audits(
    *,
    sweep_dir: Path,
    output_dir: Path,
    selection_audit_dir: Path | None = None,
    blend_audit_dir: Path | None = None,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    selection_dir = selection_audit_dir or (sweep_dir / "profile_selection_strategy_audit_all_catalog_target_max_score")
    blend_dir = blend_audit_dir or (sweep_dir / "profile_blend_strategy_audit_all_catalog_target_max_score")
    rows: list[dict[str, object]] = []
    selection_csv = selection_dir / "profile_selection_strategy_runs.csv"
    if selection_csv.exists():
        rows.append(selection_strategy_row(selection_csv, selection_dir / "profile_selection_strategy_summary.json"))
    blend_csv = blend_dir / "blend_strategy_policy_summary.csv"
    if blend_csv.exists():
        rows.extend(blend_strategy_rows(blend_csv, blend_dir / "blend_strategy_summary.json"))
    rows.sort(
        key=lambda row: (
            -int(row.get("pass_count", 0) or 0),
            finite_float(row.get("deployment_audit_score"), float("inf")),
            str(row.get("strategy", "")),
            str(row.get("policy", "")),
        )
    )
    recommendation = choose_recommendation(rows)
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "deployment_audit_strategy_candidates.csv", rows)
    write_json(output_dir / "deployment_audit_strategy_recommendation.json", recommendation)
    write_summary_txt(output_dir / "deployment_audit_strategy_summary.txt", recommendation, rows)
    return rows, recommendation


def main() -> int:
    args = parse_args()
    rows, recommendation = summarize_deployment_audits(
        sweep_dir=args.sweep_dir,
        output_dir=args.output_dir,
        selection_audit_dir=args.selection_audit_dir,
        blend_audit_dir=args.blend_audit_dir,
    )
    print(f"wrote deployment audit strategy summary to {args.output_dir}")
    print(
        "recommended {strategy}/{policy}: score={score:.6f}, candidates={count}".format(
            strategy=recommendation.get("recommended_strategy", ""),
            policy=recommendation.get("recommended_policy", ""),
            score=finite_float(recommendation.get("deployment_audit_score"), float("inf")),
            count=len(rows),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
