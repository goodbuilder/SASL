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
    "local sweep frontier is selected with calibration-time truth only for offline validation; "
    "runtime profiles consume stereo-center RDF(right,down,forward) plus measured disparity and publish FRD+range"
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a common-CSV frontier from a local-sweep pipeline probe. Only methods marked ADDED by "
            "local_sweep_config_status.csv are included, so the next matrix replay evaluates the new local "
            "search candidates rather than the full pooled method catalog."
        )
    )
    parser.add_argument("--method-comparison-csv", type=Path, required=True)
    parser.add_argument("--local-sweep-status-csv", type=Path, required=True)
    parser.add_argument("--runtime-profiles-dir", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-txt", type=Path, required=True)
    parser.add_argument("--top-n", type=int, default=64)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args(argv)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass"}


def finite_float(value: object, default: float = math.inf) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def added_local_methods(status_rows: Sequence[Mapping[str, str]]) -> dict[str, Mapping[str, str]]:
    methods: dict[str, Mapping[str, str]] = {}
    for row in status_rows:
        method = str(row.get("method", "")).strip()
        status = str(row.get("status", "")).strip().upper()
        if method and status == "ADDED":
            methods[method] = row
    return methods


def sort_key(row: Mapping[str, object]) -> tuple[float, float, float, str]:
    return (
        finite_float(row.get("validation_range_rmse_m")),
        finite_float(row.get("validation_target_max_abs_error_m")),
        finite_float(row.get("model_complexity")),
        str(row.get("method", "")),
    )


def export_frontier(
    *,
    method_comparison_csv: Path,
    local_sweep_status_csv: Path,
    runtime_profiles_dir: Path,
    output_csv: Path,
    output_json: Path,
    output_txt: Path,
    top_n: int = 64,
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    if top_n <= 0:
        raise ValueError("--top-n must be positive")

    status_by_method = added_local_methods(read_csv(local_sweep_status_csv))
    comparison_rows = read_csv(method_comparison_csv)
    frontier_rows: list[dict[str, object]] = []
    missing_profile_count = 0

    for row in comparison_rows:
        method = str(row.get("method", "")).strip()
        status_row = status_by_method.get(method)
        if status_row is None:
            continue
        profile_json = runtime_profiles_dir / f"{method}.json"
        profile_found = profile_json.is_file()
        if not profile_found:
            missing_profile_count += 1
            continue
        target_max = finite_float(row.get("validation_target_max_abs_error_m"))
        frontier_rows.append(
            {
                "method": method,
                "profile_json": str(profile_json),
                "validation_range_rmse_m": row.get("validation_range_rmse_m", ""),
                "validation_target_max_abs_error_m": row.get("validation_target_max_abs_error_m", ""),
                "validation_target_abs_mean_error_m": row.get("validation_target_abs_mean_error_m", ""),
                "model_complexity": row.get("model_complexity", ""),
                "registration_model": row.get("registration_model", ""),
                "residual_model": row.get("residual_model", ""),
                "runtime_profile_family": row.get("runtime_profile_family", ""),
                "local_sweep_status": status_row.get("status", ""),
                "local_sweep_reason": status_row.get("reason", ""),
                "profile_found": profile_found,
                "pass_target_error_limit": target_max <= target_error_limit_m,
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
        )

    frontier_rows = sorted(frontier_rows, key=sort_key)[:top_n]
    for index, row in enumerate(frontier_rows, start=1):
        row["rank"] = index

    ordered_rows = [
        {
            "rank": row["rank"],
            **{key: value for key, value in row.items() if key != "rank"},
        }
        for row in frontier_rows
    ]
    summary = {
        "status": "PASS" if ordered_rows else "FAIL",
        "reason": "local sweep frontier exported" if ordered_rows else "no added local sweep methods with profiles",
        "method_comparison_csv": str(method_comparison_csv),
        "local_sweep_status_csv": str(local_sweep_status_csv),
        "runtime_profiles_dir": str(runtime_profiles_dir),
        "output_csv": str(output_csv),
        "method_count": len(ordered_rows),
        "added_status_method_count": len(status_by_method),
        "missing_profile_count": missing_profile_count,
        "selected_method": ordered_rows[0]["method"] if ordered_rows else "",
        "target_error_limit_m": target_error_limit_m,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "rows": ordered_rows,
    }
    write_csv(output_csv, ordered_rows)
    write_json(output_json, summary)
    write_summary_txt(output_txt, summary, ordered_rows)
    return summary


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "RTK Stereo local sweep frontier",
        "",
        f"status: {summary.get('status', '')}",
        f"selected_method: {summary.get('selected_method', '')}",
        f"method_count: {summary.get('method_count', '')}",
        f"added_status_method_count: {summary.get('added_status_method_count', '')}",
        f"missing_profile_count: {summary.get('missing_profile_count', '')}",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        f"runtime_input_contract: {RUNTIME_INPUT_CONTRACT}",
        "runtime_truth_used: False",
        "",
        "Top candidates:",
    ]
    for row in rows[:20]:
        lines.append(
            (
                "- rank={rank} method={method} rmse={rmse} target_max={target_max} "
                "target_pass={target_pass}"
            ).format(
                rank=row.get("rank", ""),
                method=row.get("method", ""),
                rmse=row.get("validation_range_rmse_m", ""),
                target_max=row.get("validation_target_max_abs_error_m", ""),
                target_pass=row.get("pass_target_error_limit", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    summary = export_frontier(
        method_comparison_csv=args.method_comparison_csv,
        local_sweep_status_csv=args.local_sweep_status_csv,
        runtime_profiles_dir=args.runtime_profiles_dir,
        output_csv=args.output_csv,
        output_json=args.output_json,
        output_txt=args.output_txt,
        top_n=args.top_n,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
