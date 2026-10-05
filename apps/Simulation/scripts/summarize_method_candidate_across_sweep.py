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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize candidate calibration methods across a sweep without re-running fitting. "
            "The input method_comparison.csv files use stereo-center RDF metrics."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--method-substring", default="disparity_product_ray_scale")
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def finite_float(value: object, default: float = float("nan")) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def scenario_seed_from_path(path: Path, sweep_dir: Path) -> tuple[str, str]:
    relative = path.relative_to(sweep_dir)
    parts = relative.parts
    if len(parts) >= 3:
        return parts[0], parts[1]
    return "", ""


def rank_rows(rows: Sequence[Mapping[str, str]], key: str) -> dict[str, int]:
    ranked = sorted(
        rows,
        key=lambda row: (
            finite_float(row.get(key), float("inf")),
            finite_float(row.get("validation_target_max_abs_error_m"), float("inf")),
            finite_float(row.get("model_complexity"), float("inf")),
            str(row.get("method", "")),
        ),
    )
    return {str(row.get("method", "")): index + 1 for index, row in enumerate(ranked)}


def summarize_candidate_file(
    path: Path,
    *,
    sweep_dir: Path,
    method_substring: str,
    target_error_limit_m: float,
) -> list[dict[str, object]]:
    rows = read_csv(path)
    if not rows:
        return []
    scenario, seed = scenario_seed_from_path(path, sweep_dir)
    raw_row = next((row for row in rows if row.get("method") == "raw"), {})
    best_rmse = min(rows, key=lambda row: finite_float(row.get("validation_range_rmse_m"), float("inf")))
    best_target = min(rows, key=lambda row: finite_float(row.get("validation_target_max_abs_error_m"), float("inf")))
    rmse_ranks = rank_rows(rows, "validation_range_rmse_m")
    target_ranks = rank_rows(rows, "validation_target_max_abs_error_m")
    raw_rmse = finite_float(raw_row.get("validation_range_rmse_m"), float("nan"))
    matched = [row for row in rows if method_substring in str(row.get("method", ""))]
    out: list[dict[str, object]] = []
    for row in matched:
        method = str(row.get("method", ""))
        validation_rmse = finite_float(row.get("validation_range_rmse_m"), float("nan"))
        target_max = finite_float(row.get("validation_target_max_abs_error_m"), float("nan"))
        target_abs = finite_float(row.get("validation_target_abs_mean_error_m"), float("nan"))
        improvement = raw_rmse - validation_rmse if math.isfinite(raw_rmse) and math.isfinite(validation_rmse) else float("nan")
        out.append(
            {
                "scenario": scenario,
                "seed": seed,
                "method": method,
                "runtime_profile_family": row.get("runtime_profile_family", ""),
                "registration_model": row.get("registration_model", ""),
                "residual_model": row.get("residual_model", ""),
                "model_complexity": row.get("model_complexity", ""),
                "validation_range_rmse_raw_m": raw_row.get("validation_range_rmse_m", ""),
                "validation_range_rmse_m": row.get("validation_range_rmse_m", ""),
                "validation_range_rmse_improvement_vs_raw_m": improvement,
                "validation_target_abs_mean_error_m": row.get("validation_target_abs_mean_error_m", ""),
                "validation_target_max_abs_error_m": row.get("validation_target_max_abs_error_m", ""),
                "pass_target_max_limit": bool(math.isfinite(target_max) and target_max <= target_error_limit_m),
                "rank_by_validation_rmse": rmse_ranks.get(method, ""),
                "rank_by_validation_target_max": target_ranks.get(method, ""),
                "best_validation_rmse_method": best_rmse.get("method", ""),
                "best_validation_rmse_m": best_rmse.get("validation_range_rmse_m", ""),
                "best_target_max_method": best_target.get("method", ""),
                "best_target_max_abs_error_m": best_target.get("validation_target_max_abs_error_m", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    return out


def summarize(rows: Sequence[Mapping[str, object]], *, sweep_dir: Path, method_substring: str) -> dict[str, object]:
    rmse_values = [
        finite_float(row.get("validation_range_rmse_m"))
        for row in rows
        if math.isfinite(finite_float(row.get("validation_range_rmse_m")))
    ]
    target_max_values = [
        finite_float(row.get("validation_target_max_abs_error_m"))
        for row in rows
        if math.isfinite(finite_float(row.get("validation_target_max_abs_error_m")))
    ]
    improvement_values = [
        finite_float(row.get("validation_range_rmse_improvement_vs_raw_m"))
        for row in rows
        if math.isfinite(finite_float(row.get("validation_range_rmse_improvement_vs_raw_m")))
    ]
    best_row = min(rows, key=lambda row: finite_float(row.get("validation_range_rmse_m"), float("inf")), default={})
    rmse_mean = float(sum(rmse_values) / len(rmse_values)) if rmse_values else None
    rmse_max = float(max(rmse_values)) if rmse_values else None
    target_max = float(max(target_max_values)) if target_max_values else None
    improvement_min = float(min(improvement_values)) if improvement_values else None
    return {
        "mode": "method_candidate_across_sweep_summary",
        "sweep_dir": str(sweep_dir),
        "method_substring": method_substring,
        "candidate_row_count": len(rows),
        "scenario_count": len({str(row.get("scenario", "")) for row in rows}),
        "pass_target_max_limit_count": sum(1 for row in rows if bool(row.get("pass_target_max_limit"))),
        "validation_range_rmse_mean_m": rmse_mean,
        "validation_range_rmse_max_m": rmse_max,
        "validation_target_max_abs_error_max_m": target_max,
        "validation_range_rmse_improvement_min_m": improvement_min,
        "best_candidate_method": best_row.get("method", ""),
        "best_candidate_scenario": best_row.get("scenario", ""),
        "best_candidate_validation_range_rmse_m": best_row.get("validation_range_rmse_m", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "This is an offline summary of fitted method_comparison.csv rows. Runtime applicability is read from "
            "runtime_profile_family; no calibration truth is introduced into runtime profile application."
        ),
    }


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Method candidate across sweep summary",
        "",
        f"method_substring: {summary.get('method_substring', '')}",
        f"candidate_row_count: {summary.get('candidate_row_count', '')}",
        f"scenario_count: {summary.get('scenario_count', '')}",
        f"pass_target_max_limit_count: {summary.get('pass_target_max_limit_count', '')}",
        f"validation_range_rmse_mean_m: {summary.get('validation_range_rmse_mean_m', '')}",
        f"validation_range_rmse_max_m: {summary.get('validation_range_rmse_max_m', '')}",
        f"validation_target_max_abs_error_max_m: {summary.get('validation_target_max_abs_error_max_m', '')}",
        f"validation_range_rmse_improvement_min_m: {summary.get('validation_range_rmse_improvement_min_m', '')}",
        f"best_candidate_method: {summary.get('best_candidate_method', '')}",
        f"best_candidate_scenario: {summary.get('best_candidate_scenario', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        "",
        "Rows:",
    ]
    for row in rows:
        lines.append(
            "- {scenario}/{seed} {method}: rmse={rmse}, target_max={target_max}, "
            "rmse_rank={rmse_rank}, target_rank={target_rank}, best_rmse={best}".format(
                scenario=row.get("scenario", ""),
                seed=row.get("seed", ""),
                method=row.get("method", ""),
                rmse=row.get("validation_range_rmse_m", ""),
                target_max=row.get("validation_target_max_abs_error_m", ""),
                rmse_rank=row.get("rank_by_validation_rmse", ""),
                target_rank=row.get("rank_by_validation_target_max", ""),
                best=row.get("best_validation_rmse_method", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_candidate_across_sweep(
    sweep_dir: Path,
    output_dir: Path,
    *,
    method_substring: str,
    target_error_limit_m: float,
) -> dict[str, object]:
    rows: list[dict[str, object]] = []
    for path in sorted(sweep_dir.glob("*/*/method_comparison.csv")):
        rows.extend(
            summarize_candidate_file(
                path,
                sweep_dir=sweep_dir,
                method_substring=method_substring,
                target_error_limit_m=target_error_limit_m,
            )
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "method_candidate_rows.csv", rows)
    summary = summarize(rows, sweep_dir=sweep_dir, method_substring=method_substring)
    write_json(output_dir / "method_candidate_summary.json", summary)
    write_summary_txt(output_dir / "method_candidate_summary.txt", summary, rows)
    return summary


def main() -> int:
    args = parse_args()
    summary = summarize_candidate_across_sweep(
        args.sweep_dir,
        args.output_dir,
        method_substring=args.method_substring,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
