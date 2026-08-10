#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
RUNTIME_INPUT_CONTRACT = (
    "selector audit uses calibration-time truth only for offline scoring; deployable selector inputs are "
    "scenario/deployment metadata or raw runtime features plus stereo-center RDF/disparity, never PX4/RTK truth"
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit whether a common-CSV matrix candidate pool is better used as one global runtime profile "
            "or as a truth-free scenario-label selector/portfolio."
        )
    )
    parser.add_argument("--matrix-dir", type=Path, action="append", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--objective",
        choices=("score", "target_then_rmse", "rmse_then_target"),
        default="score",
    )
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args(argv)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def finite_float(value: object, default: float = math.inf) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass"}


def run_key_from_csv(path: Path, matrix_dir: Path) -> tuple[str, str]:
    relative = path.relative_to(matrix_dir)
    parts = relative.parts
    if len(parts) < 4 or parts[0] != "runs":
        raise ValueError(f"unexpected matrix run csv path: {path}")
    return parts[1], parts[2]


def row_identity(row: Mapping[str, object]) -> str:
    return str(row.get("method", "") or row.get("candidate", "")).strip()


def row_sort_key(row: Mapping[str, object], objective: str) -> tuple[object, ...]:
    pass_penalty = 0 if parse_bool(row.get("pass_common_replay")) else 1
    target = finite_float(row.get("target_max_abs_error_corrected_m"))
    rmse = finite_float(row.get("validation_range_rmse_corrected_m"))
    score = finite_float(row.get("common_csv_score"))
    if objective == "target_then_rmse":
        metric_key = (target, rmse, score)
    elif objective == "rmse_then_target":
        metric_key = (rmse, target, score)
    else:
        metric_key = (score, target, rmse)
    return (pass_penalty, *metric_key, row_identity(row))


def aggregate_sort_key(row: Mapping[str, object], objective: str) -> tuple[object, ...]:
    pass_all_penalty = 0 if parse_bool(row.get("pass_all_runs")) else 1
    pass_count = -int(finite_float(row.get("pass_count"), 0.0))
    target = finite_float(row.get("target_max_abs_error_corrected_max_m"))
    rmse_max = finite_float(row.get("validation_range_rmse_corrected_max_m"))
    rmse_mean = finite_float(row.get("validation_range_rmse_corrected_mean_m"))
    score = finite_float(row.get("common_csv_score_mean"))
    if objective == "target_then_rmse":
        metric_key = (target, rmse_max, rmse_mean, score)
    elif objective == "rmse_then_target":
        metric_key = (rmse_max, rmse_mean, target, score)
    else:
        metric_key = (score, target, rmse_max, rmse_mean)
    return (pass_all_penalty, pass_count, *metric_key, str(row.get("method", "")))


def discover_run_rows(matrix_dirs: Sequence[Path], objective: str) -> dict[tuple[str, str], dict[str, dict[str, object]]]:
    rows_by_run: dict[tuple[str, str], dict[str, dict[str, object]]] = {}
    for matrix_dir in matrix_dirs:
        for csv_path in sorted(matrix_dir.glob("runs/*/seed_*/common_csv_profile_comparison.csv")):
            scenario, seed = run_key_from_csv(csv_path, matrix_dir)
            run_rows = rows_by_run.setdefault((scenario, seed), {})
            for raw_row in read_csv(csv_path):
                method = row_identity(raw_row)
                if not method:
                    continue
                row = dict(raw_row)
                row.update(
                    {
                        "scenario": scenario,
                        "seed": seed,
                        "method": method,
                        "source_matrix_dir": str(matrix_dir),
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                        "runtime_truth_used": False,
                    }
                )
                previous = run_rows.get(method)
                if previous is None or row_sort_key(row, objective) < row_sort_key(previous, objective):
                    run_rows[method] = row
    return rows_by_run


def aggregate_methods(
    rows_by_run: Mapping[tuple[str, str], Mapping[str, Mapping[str, object]]],
    *,
    objective: str,
) -> list[dict[str, object]]:
    methods = sorted({method for rows in rows_by_run.values() for method in rows.keys()})
    total_runs = len(rows_by_run)
    output: list[dict[str, object]] = []
    for method in methods:
        rows = [rows[method] for rows in rows_by_run.values() if method in rows]
        target_values = [finite_float(row.get("target_max_abs_error_corrected_m")) for row in rows]
        rmse_values = [finite_float(row.get("validation_range_rmse_corrected_m")) for row in rows]
        score_values = [finite_float(row.get("common_csv_score")) for row in rows]
        target_values = [value for value in target_values if math.isfinite(value)]
        rmse_values = [value for value in rmse_values if math.isfinite(value)]
        score_values = [value for value in score_values if math.isfinite(value)]
        pass_count = sum(1 for row in rows if parse_bool(row.get("pass_common_replay")))
        output.append(
            {
                "method": method,
                "evaluated_run_count": len(rows),
                "expected_run_count": total_runs,
                "pass_count": pass_count,
                "pass_all_runs": total_runs > 0 and pass_count == total_runs,
                "validation_range_rmse_corrected_mean_m": mean_or_blank(rmse_values),
                "validation_range_rmse_corrected_max_m": max(rmse_values) if rmse_values else "",
                "target_max_abs_error_corrected_max_m": max(target_values) if target_values else "",
                "common_csv_score_mean": mean_or_blank(score_values),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
        )
    return sorted(output, key=lambda row: aggregate_sort_key(row, objective))


def mean_or_blank(values: Sequence[float]) -> float | str:
    return sum(values) / len(values) if values else ""


def selected_rows_summary(label: str, rows: Sequence[Mapping[str, object]], target_error_limit_m: float) -> dict[str, object]:
    target_values = [finite_float(row.get("target_max_abs_error_corrected_m")) for row in rows]
    rmse_values = [finite_float(row.get("validation_range_rmse_corrected_m")) for row in rows]
    score_values = [finite_float(row.get("common_csv_score")) for row in rows]
    target_values = [value for value in target_values if math.isfinite(value)]
    rmse_values = [value for value in rmse_values if math.isfinite(value)]
    score_values = [value for value in score_values if math.isfinite(value)]
    pass_count = sum(1 for row in rows if parse_bool(row.get("pass_common_replay")))
    return {
        "selector_mode": label,
        "run_count": len(rows),
        "pass_count": pass_count,
        "pass_all_runs": bool(rows) and pass_count == len(rows),
        "target_limit_pass": bool(target_values) and max(target_values) <= target_error_limit_m,
        "validation_range_rmse_corrected_mean_m": mean_or_blank(rmse_values),
        "validation_range_rmse_corrected_max_m": max(rmse_values) if rmse_values else "",
        "target_max_abs_error_corrected_max_m": max(target_values) if target_values else "",
        "common_csv_score_mean": mean_or_blank(score_values),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }


def choose_best_method(
    rows: Sequence[Mapping[str, object]],
    *,
    objective: str,
) -> str:
    if not rows:
        return ""
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row.get("method", "")), []).append(row)
    candidates: list[dict[str, object]] = []
    for method, method_rows in grouped.items():
        rows_by_run = {
            (str(row.get("scenario", "")), str(row.get("seed", ""))): {method: row}
            for row in method_rows
        }
        candidates.extend(aggregate_methods(rows_by_run, objective=objective))
    return str(sorted(candidates, key=lambda row: aggregate_sort_key(row, objective))[0].get("method", ""))


def selector_rows(
    rows_by_run: Mapping[tuple[str, str], Mapping[str, Mapping[str, object]]],
    *,
    objective: str,
) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    aggregate_rows = aggregate_methods(rows_by_run, objective=objective)
    global_method = str(aggregate_rows[0]["method"]) if aggregate_rows else ""
    global_rows: list[dict[str, object]] = []
    scenario_loso_rows: list[dict[str, object]] = []
    scenario_insample_rows: list[dict[str, object]] = []
    oracle_rows: list[dict[str, object]] = []
    flat_rows = [row for rows in rows_by_run.values() for row in rows.values()]
    for run_key, methods in sorted(rows_by_run.items()):
        scenario, seed = run_key
        if global_method and global_method in methods:
            global_rows.append(dict(methods[global_method], selector_mode="global_single", selected_by="global_pool"))
        same_scenario_train = [
            row
            for row in flat_rows
            if row.get("scenario") == scenario and row.get("seed") != seed
        ]
        loso_method = choose_best_method(same_scenario_train, objective=objective) or global_method
        if loso_method in methods:
            scenario_loso_rows.append(
                dict(methods[loso_method], selector_mode="scenario_loso", selected_by=f"scenario:{scenario}:other_seed")
            )
        same_scenario_all = [row for row in flat_rows if row.get("scenario") == scenario]
        insample_method = choose_best_method(same_scenario_all, objective=objective) or global_method
        if insample_method in methods:
            scenario_insample_rows.append(
                dict(methods[insample_method], selector_mode="scenario_insample", selected_by=f"scenario:{scenario}:all")
            )
        oracle = min(methods.values(), key=lambda row: row_sort_key(row, objective))
        oracle_rows.append(dict(oracle, selector_mode="per_run_oracle", selected_by="offline_upper_bound"))
    return global_rows, scenario_loso_rows, scenario_insample_rows, oracle_rows


def audit_selector_portfolio(
    *,
    matrix_dirs: Sequence[Path],
    output_dir: Path,
    objective: str = "score",
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows_by_run = discover_run_rows(matrix_dirs, objective)
    aggregate_rows = aggregate_methods(rows_by_run, objective=objective)
    global_rows, scenario_loso_rows, scenario_insample_rows, oracle_rows = selector_rows(
        rows_by_run,
        objective=objective,
    )
    selected_row_sets = [
        ("global_single", global_rows),
        ("scenario_loso", scenario_loso_rows),
        ("scenario_insample", scenario_insample_rows),
        ("per_run_oracle", oracle_rows),
    ]
    summary_rows = [
        selected_rows_summary(label, rows, target_error_limit_m) for label, rows in selected_row_sets
    ]
    detail_rows = [row for _, rows in selected_row_sets for row in rows]
    deployable_rows = [
        row for row in summary_rows if row.get("selector_mode") in {"global_single", "scenario_loso"}
    ]
    selected_deployable = sorted(deployable_rows, key=lambda row: aggregate_sort_key(row_to_aggregate(row), objective))[0]
    selected_upper_bound = sorted(summary_rows, key=lambda row: aggregate_sort_key(row_to_aggregate(row), objective))[0]
    summary = {
        "status": "PASS" if rows_by_run else "FAIL",
        "reason": "selector portfolio audit completed" if rows_by_run else "no common-CSV matrix rows found",
        "objective": objective,
        "matrix_dirs": [str(path) for path in matrix_dirs],
        "output_dir": str(output_dir),
        "run_count": len(rows_by_run),
        "method_count": len(aggregate_rows),
        "recommended_selector_mode": selected_deployable.get("selector_mode", ""),
        "recommended_deployable_selector_mode": selected_deployable.get("selector_mode", ""),
        "upper_bound_selector_mode": selected_upper_bound.get("selector_mode", ""),
        "selector_modes": summary_rows,
        "global_method": aggregate_rows[0]["method"] if aggregate_rows else "",
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    write_csv(output_dir / "selector_portfolio_methods.csv", aggregate_rows)
    write_csv(output_dir / "selector_portfolio_modes.csv", summary_rows)
    write_csv(output_dir / "selector_portfolio_selected_runs.csv", detail_rows)
    write_json(output_dir / "selector_portfolio_summary.json", summary)
    write_summary_txt(output_dir / "selector_portfolio_summary.txt", summary, summary_rows)
    plot_modes(output_dir / "selector_portfolio_modes.png", summary_rows)
    return summary


def row_to_aggregate(row: Mapping[str, object]) -> dict[str, object]:
    return {
        "method": row.get("selector_mode", ""),
        "pass_all_runs": row.get("pass_all_runs", False),
        "pass_count": row.get("pass_count", 0),
        "target_max_abs_error_corrected_max_m": row.get("target_max_abs_error_corrected_max_m", ""),
        "validation_range_rmse_corrected_max_m": row.get("validation_range_rmse_corrected_max_m", ""),
        "validation_range_rmse_corrected_mean_m": row.get("validation_range_rmse_corrected_mean_m", ""),
        "common_csv_score_mean": row.get("common_csv_score_mean", ""),
    }


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Common-CSV matrix selector portfolio audit",
        "",
        f"status: {summary.get('status', '')}",
        f"objective: {summary.get('objective', '')}",
        f"recommended_selector_mode: {summary.get('recommended_selector_mode', '')}",
        f"upper_bound_selector_mode: {summary.get('upper_bound_selector_mode', '')}",
        f"run_count: {summary.get('run_count', '')}",
        f"method_count: {summary.get('method_count', '')}",
        f"global_method: {summary.get('global_method', '')}",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        f"runtime_input_contract: {RUNTIME_INPUT_CONTRACT}",
        "runtime_truth_used: False",
        "",
        "Selector modes:",
    ]
    for row in rows:
        lines.append(
            (
                "- {mode}: pass={passed}/{total}, rmse_mean={rmse_mean}, rmse_max={rmse_max}, "
                "target_max={target_max}, score_mean={score_mean}"
            ).format(
                mode=row.get("selector_mode", ""),
                passed=row.get("pass_count", ""),
                total=row.get("run_count", ""),
                rmse_mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                target_max=row.get("target_max_abs_error_corrected_max_m", ""),
                score_mean=row.get("common_csv_score_mean", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_modes(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    labels = [str(row.get("selector_mode", "")) for row in rows]
    rmse_max = np.asarray([finite_float(row.get("validation_range_rmse_corrected_max_m"), 0.0) for row in rows])
    target_max = np.asarray([finite_float(row.get("target_max_abs_error_corrected_max_m"), 0.0) for row in rows])
    x = np.arange(len(rows))
    width = 0.38
    plt.figure(figsize=(10, 5))
    plt.bar(x - width / 2.0, rmse_max, width=width, label="range RMSE max")
    plt.bar(x + width / 2.0, target_max, width=width, label="1852m target max")
    plt.axhline(10.0, color="tab:red", linestyle="--", linewidth=1, label="10m target limit")
    plt.xticks(x, labels, rotation=20, ha="right")
    plt.ylabel("meters")
    plt.title("Selector portfolio audit")
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    summary = audit_selector_portfolio(
        matrix_dirs=args.matrix_dir,
        output_dir=args.output_dir,
        objective=args.objective,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
