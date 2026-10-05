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
from stereodistance_profile_replay import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "broad method-comparison ranking uses offline validation metrics only; runtime profiles consume "
    "stereo-center RDF(right,down,forward) plus measured disparity and publish FRD+range"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Aggregate every seed method_comparison.csv under a sweep root into a broad runtime-profile candidate "
            "frontier. PX4/RTK truth is used only by the existing offline validation metrics."
        )
    )
    parser.add_argument("--sweep-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def discover_method_comparison_csvs(sweep_root: Path) -> list[Path]:
    return sorted(sweep_root.glob("*/seed_*/method_comparison.csv"))


def scenario_seed_from_path(path: Path) -> tuple[str, str]:
    return path.parent.parent.name, path.parent.name


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass"}


def validation_pass(row: Mapping[str, object], raw: Mapping[str, object], profile_exists: bool, target_error_limit_m: float) -> bool:
    method_range = finite_float(row.get("validation_range_rmse_m"), float("inf"))
    raw_range = finite_float(raw.get("validation_range_rmse_m"), float("inf"))
    axis_pass = all(
        finite_float(row.get(f"validation_{axis}_rmse_m"), float("inf"))
        <= finite_float(raw.get(f"validation_{axis}_rmse_m"), float("inf")) + 1e-9
        for axis in ("right", "down", "forward")
    )
    target_count = int(finite_float(row.get("validation_target_window_count"), 0.0))
    target_max = finite_float(row.get("validation_target_max_abs_error_m"), float("inf"))
    return profile_exists and method_range < raw_range and axis_pass and target_count > 0 and target_max <= target_error_limit_m


def row_metrics(row: Mapping[str, object], raw: Mapping[str, object], profile_exists: bool, target_error_limit_m: float) -> dict[str, object]:
    raw_range = finite_float(raw.get("validation_range_rmse_m"), float("nan"))
    method_range = finite_float(row.get("validation_range_rmse_m"), float("nan"))
    axis_regression_sum = 0.0
    for axis in ("right", "down", "forward"):
        diff = finite_float(row.get(f"validation_{axis}_rmse_m"), float("inf")) - finite_float(
            raw.get(f"validation_{axis}_rmse_m"),
            float("inf"),
        )
        if math.isfinite(diff) and diff > 0.0:
            axis_regression_sum += diff
    target_max = finite_float(row.get("validation_target_max_abs_error_m"), float("nan"))
    return {
        "profile_exists": profile_exists,
        "validation_pass": validation_pass(row, raw, profile_exists, target_error_limit_m),
        "range_improvement_m": raw_range - method_range if math.isfinite(raw_range) and math.isfinite(method_range) else "",
        "axis_rmse_regression_sum_m": axis_regression_sum,
        "target_error_limit_pass": math.isfinite(target_max) and target_max <= target_error_limit_m,
        "validation_range_rmse_m": method_range,
        "validation_position_rmse_m": finite_float(row.get("validation_position_rmse_m"), float("nan")),
        "validation_target_max_abs_error_m": target_max,
        "validation_target_abs_mean_error_m": finite_float(row.get("validation_target_abs_mean_error_m"), float("nan")),
        "validation_target_rmse_m": finite_float(row.get("validation_target_rmse_m"), float("nan")),
        "model_complexity": finite_float(row.get("model_complexity"), float("nan")),
    }


def mean_or_blank(values: Sequence[float]) -> float | str:
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / len(finite) if finite else ""


def max_or_blank(values: Sequence[float]) -> float | str:
    finite = [value for value in values if math.isfinite(value)]
    return max(finite) if finite else ""


def aggregate_candidates(*, sweep_root: Path, target_error_limit_m: float) -> dict[str, object]:
    comparison_csvs = discover_method_comparison_csvs(sweep_root)
    grouped: dict[str, list[dict[str, object]]] = {}
    run_rows: list[dict[str, object]] = []
    for comparison_csv in comparison_csvs:
        rows = read_csv(comparison_csv)
        raw = next((row for row in rows if row.get("method") == "raw"), None)
        if raw is None:
            raise ValueError(f"missing raw row in {comparison_csv}")
        scenario, seed = scenario_seed_from_path(comparison_csv)
        runtime_profiles_dir = comparison_csv.parent / "runtime_profiles"
        for row in rows:
            method = str(row.get("method", "")).strip()
            if not method or method == "raw":
                continue
            profile_json = runtime_profiles_dir / f"{method}.json"
            metrics = row_metrics(row, raw, profile_json.is_file(), target_error_limit_m)
            run_row = {
                "scenario": scenario,
                "seed": seed,
                "method": method,
                "comparison_csv": str(comparison_csv),
                "profile_json": str(profile_json),
                "registration_model": row.get("registration_model", ""),
                "residual_model": row.get("residual_model", ""),
                "regularization_model": row.get("regularization_model", ""),
                "runtime_profile_family": row.get("runtime_profile_family", ""),
                "selection_note": row.get("selection_note", ""),
                "axis_contract": row.get("rdf_axis_contract", RDF_AXIS_CONTRACT),
                **metrics,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
            grouped.setdefault(method, []).append(run_row)
            run_rows.append(run_row)

    expected_run_count = len(comparison_csvs)
    method_rows: list[dict[str, object]] = []
    for method, rows in grouped.items():
        range_values = [finite_float(row.get("validation_range_rmse_m"), float("nan")) for row in rows]
        position_values = [finite_float(row.get("validation_position_rmse_m"), float("nan")) for row in rows]
        target_max_values = [finite_float(row.get("validation_target_max_abs_error_m"), float("nan")) for row in rows]
        target_abs_mean_values = [
            finite_float(row.get("validation_target_abs_mean_error_m"), float("nan")) for row in rows
        ]
        improvement_values = [finite_float(row.get("range_improvement_m"), float("nan")) for row in rows]
        axis_regression_values = [finite_float(row.get("axis_rmse_regression_sum_m"), float("nan")) for row in rows]
        complexity_values = [finite_float(row.get("model_complexity"), float("nan")) for row in rows]
        profile_exists_count = sum(1 for row in rows if parse_bool(row.get("profile_exists")))
        pass_count = sum(1 for row in rows if parse_bool(row.get("validation_pass")))
        method_rows.append(
            {
                "rank": 0,
                "method": method,
                "evaluated_run_count": len(rows),
                "expected_run_count": expected_run_count,
                "profile_exists_count": profile_exists_count,
                "pass_count": pass_count,
                "pass_all_runs": expected_run_count > 0 and pass_count == expected_run_count,
                "strict_pass_count": pass_count,
                "performance_pass_count": pass_count,
                "expected_scenario_count": expected_run_count,
                "strict_all_scenarios": expected_run_count > 0 and pass_count == expected_run_count,
                "validation_range_rmse_mean_m": mean_or_blank(range_values),
                "validation_range_rmse_max_m": max_or_blank(range_values),
                "validation_position_rmse_mean_m": mean_or_blank(position_values),
                "validation_position_rmse_max_m": max_or_blank(position_values),
                "target_abs_mean_error_max_m": max_or_blank(target_abs_mean_values),
                "validation_target_max_abs_error_max_m": max_or_blank(target_max_values),
                "range_improvement_mean_m": mean_or_blank(improvement_values),
                "axis_rmse_regression_sum_mean_m": mean_or_blank(axis_regression_values),
                "model_complexity_mean": mean_or_blank(complexity_values),
                "target_error_limit_pass": all(parse_bool(row.get("target_error_limit_pass")) for row in rows),
                "rdf_axis_nonregression_pass": all(finite_float(row.get("axis_rmse_regression_sum_m"), 1.0) <= 1e-9 for row in rows),
                "registration_model": rows[0].get("registration_model", ""),
                "residual_model": rows[0].get("residual_model", ""),
                "regularization_model": rows[0].get("regularization_model", ""),
                "runtime_profile_family": rows[0].get("runtime_profile_family", ""),
                "selection_note": rows[0].get("selection_note", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
        )
    method_rows = sorted(method_rows, key=method_sort_key)
    for rank, row in enumerate(method_rows, start=1):
        row["rank"] = rank
    selected = method_rows[0] if method_rows and parse_bool(method_rows[0].get("pass_all_runs")) else {}
    return {
        "status": "PASS" if selected else "FAIL",
        "selected_method": selected.get("method", ""),
        "reason": "lowest all-run passing broad validation score" if selected else "no all-run passing method",
        "sweep_root": str(sweep_root),
        "run_count": expected_run_count,
        "method_count": len(method_rows),
        "target_error_limit_m": target_error_limit_m,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "methods": method_rows,
        "run_rows": run_rows,
    }


def method_sort_key(row: Mapping[str, object]) -> tuple[object, ...]:
    return (
        0 if parse_bool(row.get("pass_all_runs")) else 1,
        -int(finite_float(row.get("pass_count"), 0.0)),
        finite_float(row.get("validation_range_rmse_mean_m"), float("inf")),
        finite_float(row.get("validation_target_max_abs_error_max_m"), float("inf")),
        finite_float(row.get("axis_rmse_regression_sum_mean_m"), float("inf")),
        finite_float(row.get("model_complexity_mean"), float("inf")),
        str(row.get("method", "")),
    )


def write_summary_txt(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Broad sweep method-comparison candidate ranking",
        "",
        f"status: {summary.get('status', '')}",
        f"selected_method: {summary.get('selected_method', '')}",
        f"reason: {summary.get('reason', '')}",
        f"run_count: {summary.get('run_count', 0)}",
        f"method_count: {summary.get('method_count', 0)}",
        f"target_error_limit_m: {summary.get('target_error_limit_m', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "runtime_truth_used: False",
        "",
        "Top methods:",
    ]
    methods = summary.get("methods", [])
    if isinstance(methods, list):
        for row in methods[:20]:
            if not isinstance(row, Mapping):
                continue
            lines.append(
                (
                    "- rank={rank} method={method} pass={pass_count}/{expected} "
                    "rmse_mean={rmse} target_max={target} axis_regression_mean={axis}"
                ).format(
                    rank=row.get("rank", ""),
                    method=row.get("method", ""),
                    pass_count=row.get("pass_count", ""),
                    expected=row.get("expected_run_count", ""),
                    rmse=row.get("validation_range_rmse_mean_m", ""),
                    target=row.get("validation_target_max_abs_error_max_m", ""),
                    axis=row.get("axis_rmse_regression_sum_mean_m", ""),
                )
            )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_outputs(output_dir: Path, summary: Mapping[str, object]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    methods = summary.get("methods", [])
    run_rows = summary.get("run_rows", [])
    write_csv(output_dir / "broad_method_candidate_rank.csv", methods if isinstance(methods, list) else [])
    write_csv(output_dir / "broad_method_candidate_run_rows.csv", run_rows if isinstance(run_rows, list) else [])
    write_json(output_dir / "broad_method_candidate_rank.json", summary)
    write_summary_txt(output_dir / "broad_method_candidate_rank.txt", summary)


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args() if argv is None else parse_args_from_sequence(argv)
    summary = aggregate_candidates(sweep_root=args.sweep_root, target_error_limit_m=args.target_error_limit_m)
    write_outputs(args.output_dir, summary)
    return summary


def parse_args_from_sequence(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args(argv)


def main() -> int:
    payload = run()
    compact = {
        key: payload.get(key)
        for key in (
            "status",
            "selected_method",
            "reason",
            "sweep_root",
            "run_count",
            "method_count",
            "target_error_limit_m",
            "axis_contract",
            "public_topic_contract",
            "runtime_input_contract",
            "runtime_truth_used",
        )
    }
    print(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if payload.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
