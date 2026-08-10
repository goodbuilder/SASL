#!/usr/bin/env python3
from __future__ import annotations

import argparse
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

from compare_stereodistance_profiles_on_common_csv import json_safe, parse_bool  # noqa: E402
from run_common_csv_frontier_profile_comparison import run_frontier_profile_comparison  # noqa: E402
from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from stereodistance_profile_replay import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "frontier matrix replay uses calibration-time truth only for offline validation; runtime profiles consume "
    "stereo-center RDF(right,down,forward) plus measured disparity and publish FRD+range"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run common-CSV method-frontier replay across every scenario/seed directory under a sweep root "
            "and aggregate method stability without using truth as a runtime input."
        )
    )
    parser.add_argument("--frontier-csv", type=Path, required=True)
    parser.add_argument("--sweep-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--runtime-profiles-dir",
        type=Path,
        help=(
            "Optional fixed runtime_profiles directory to replay against every scenario/seed CSV. "
            "When omitted, each run's own runtime_profiles directory is used."
        ),
    )
    parser.add_argument("--active-profile-json", type=Path)
    parser.add_argument("--active-label", default="current_active")
    parser.add_argument("--top-n", type=int, default=12)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    return parser.parse_args()


def discover_run_dirs(sweep_root: Path) -> list[Path]:
    run_dirs: list[Path] = []
    for sample_csv in sorted(sweep_root.glob("*/seed_*/calibration_samples.csv")):
        run_dir = sample_csv.parent
        if (run_dir / "runtime_profiles").is_dir():
            run_dirs.append(run_dir)
    return run_dirs


def scenario_seed_from_run_dir(run_dir: Path) -> tuple[str, str]:
    return run_dir.parent.name, run_dir.name


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def run_one(
    *,
    run_dir: Path,
    output_dir: Path,
    frontier_csv: Path,
    runtime_profiles_dir: Path | None,
    active_profile_json: Path | None,
    active_label: str,
    top_n: int,
    split: str,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    scenario, seed = scenario_seed_from_run_dir(run_dir)
    run_output_dir = output_dir / "runs" / scenario / seed
    profile_dir = runtime_profiles_dir if runtime_profiles_dir is not None else run_dir / "runtime_profiles"
    manifest = run_frontier_profile_comparison(
        frontier_csv=frontier_csv,
        runtime_profiles_dir=profile_dir,
        input_csv=run_dir / "calibration_samples.csv",
        output_dir=run_output_dir,
        active_profile_json=active_profile_json,
        active_label=active_label,
        top_n=top_n,
        split=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    summary = read_json(run_output_dir / "common_csv_profile_comparison.json")
    return {
        "scenario": scenario,
        "seed": seed,
        "run_dir": str(run_dir),
        "runtime_profiles_dir": str(profile_dir),
        "output_dir": str(run_output_dir),
        "status": summary.get("status", ""),
        "selected_candidate": summary.get("selected_candidate", ""),
        "selected_method": summary.get("selected_method", ""),
        "candidate_count": summary.get("candidate_count", 0),
        "passing_candidate_count": summary.get("passing_candidate_count", 0),
        "manifest_candidate_count": manifest.get("candidate_count", 0),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "rows": summary.get("rows", []),
    }


def aggregate_method_rows(run_summaries: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for run in run_summaries:
        for row in run.get("rows", []):
            if not isinstance(row, Mapping):
                continue
            method = str(row.get("method", "")).strip()
            if not method:
                method = str(row.get("candidate", "")).strip()
            grouped.setdefault(method, []).append({**row, "_run": run})

    aggregate_rows: list[dict[str, object]] = []
    total_runs = len(run_summaries)
    for method, rows in grouped.items():
        rows_by_run: dict[tuple[str, str], list[Mapping[str, object]]] = {}
        for row in rows:
            run = row.get("_run", {})
            if not isinstance(run, Mapping):
                continue
            run_key = (str(run.get("scenario", "")), str(run.get("seed", "")))
            rows_by_run.setdefault(run_key, []).append(row)
        best_rows = [min(run_rows, key=per_run_method_row_sort_key) for run_rows in rows_by_run.values()]
        corrected_rmse = [
            finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in best_rows
        ]
        position_rmse = [
            finite_float(row.get("validation_position_rmse_corrected_m"), float("nan")) for row in best_rows
        ]
        target_max = [finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")) for row in best_rows]
        scores = [finite_float(row.get("common_csv_score"), float("nan")) for row in best_rows]
        corrected_rmse = [value for value in corrected_rmse if math.isfinite(value)]
        position_rmse = [value for value in position_rmse if math.isfinite(value)]
        target_max = [value for value in target_max if math.isfinite(value)]
        scores = [value for value in scores if math.isfinite(value)]
        pass_count = sum(1 for row in best_rows if parse_bool(row.get("pass_common_replay")))
        axis_pass_count = sum(1 for row in best_rows if parse_bool(row.get("rdf_axis_nonregression_pass")))
        target_pass_count = sum(1 for row in best_rows if parse_bool(row.get("target_max_error_pass")))
        selected_count = sum(
            1
            for row in best_rows
            if str(row.get("candidate", "")) == str(row.get("_run", {}).get("selected_candidate", ""))
        )
        aggregate_rows.append(
            {
                "method": method,
                "evaluated_run_count": len(best_rows),
                "expected_run_count": total_runs,
                "pass_count": pass_count,
                "pass_all_runs": total_runs > 0 and pass_count >= total_runs,
                "axis_pass_count": axis_pass_count,
                "target_pass_count": target_pass_count,
                "selected_count": selected_count,
                "validation_range_rmse_corrected_mean_m": mean_or_blank(corrected_rmse),
                "validation_range_rmse_corrected_max_m": max(corrected_rmse) if corrected_rmse else "",
                "validation_position_rmse_corrected_mean_m": mean_or_blank(position_rmse),
                "validation_position_rmse_corrected_max_m": max(position_rmse) if position_rmse else "",
                "target_max_abs_error_corrected_max_m": max(target_max) if target_max else "",
                "common_csv_score_mean": mean_or_blank(scores),
                "common_csv_score_max": max(scores) if scores else "",
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
        )
    return sorted(aggregate_rows, key=aggregate_sort_key)


def per_run_method_row_sort_key(row: Mapping[str, object]) -> tuple[object, ...]:
    return (
        0 if parse_bool(row.get("pass_common_replay")) else 1,
        finite_float(row.get("common_csv_score"), float("inf")),
        finite_float(row.get("target_max_abs_error_corrected_m"), float("inf")),
        finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
        str(row.get("candidate", "")),
    )


def mean_or_blank(values: Sequence[float]) -> float | str:
    return sum(values) / len(values) if values else ""


def aggregate_sort_key(row: Mapping[str, object]) -> tuple[object, ...]:
    return (
        0 if parse_bool(row.get("pass_all_runs")) else 1,
        -int(finite_float(row.get("pass_count"), 0.0)),
        finite_float(row.get("common_csv_score_mean"), float("inf")),
        finite_float(row.get("target_max_abs_error_corrected_max_m"), float("inf")),
        finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf")),
        str(row.get("method", "")),
    )


def write_matrix_summary_txt(
    path: Path,
    *,
    summary: Mapping[str, object],
    aggregate_rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "Common-CSV method-frontier profile matrix",
        "",
        f"status: {summary.get('status', '')}",
        f"selected_method: {summary.get('selected_method', '')}",
        f"run_count: {summary.get('run_count', '')}",
        f"method_count: {summary.get('method_count', '')}",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        f"runtime_input_contract: {RUNTIME_INPUT_CONTRACT}",
        "runtime_truth_used: False",
        "",
        "Methods:",
    ]
    for row in aggregate_rows:
        lines.append(
            (
                "- {method}: pass={passed}/{expected}, selected={selected}, "
                "rmse_mean={rmse_mean}, rmse_max={rmse_max}, "
                "target_max={target_max}, score_mean={score_mean}"
            ).format(
                method=row.get("method", ""),
                passed=row.get("pass_count", ""),
                expected=row.get("expected_run_count", ""),
                selected=row.get("selected_count", ""),
                rmse_mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                target_max=row.get("target_max_abs_error_corrected_max_m", ""),
                score_mean=row.get("common_csv_score_mean", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_frontier_profile_matrix(
    *,
    frontier_csv: Path,
    sweep_root: Path,
    output_dir: Path,
    runtime_profiles_dir: Path | None = None,
    active_profile_json: Path | None = None,
    active_label: str = "current_active",
    top_n: int = 12,
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
) -> dict[str, object]:
    run_dirs = discover_run_dirs(sweep_root)
    if not run_dirs:
        raise ValueError(f"no seed run dirs with calibration_samples.csv and runtime_profiles under {sweep_root}")
    output_dir.mkdir(parents=True, exist_ok=True)
    run_summaries = [
        run_one(
            run_dir=run_dir,
            output_dir=output_dir,
            frontier_csv=frontier_csv,
            runtime_profiles_dir=runtime_profiles_dir,
            active_profile_json=active_profile_json,
            active_label=active_label,
            top_n=top_n,
            split=split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
        )
        for run_dir in run_dirs
    ]
    aggregate_rows = aggregate_method_rows(run_summaries)
    selected = aggregate_rows[0] if aggregate_rows else {}
    summary = {
        "status": "PASS" if aggregate_rows and parse_bool(selected.get("pass_all_runs")) else "FAIL",
        "selected_method": selected.get("method", ""),
        "reason": "lowest all-run passing common-CSV frontier score" if aggregate_rows else "no aggregate rows",
        "frontier_csv": str(frontier_csv),
        "sweep_root": str(sweep_root),
        "output_dir": str(output_dir),
        "runtime_profiles_dir": str(runtime_profiles_dir or ""),
        "run_count": len(run_summaries),
        "method_count": len(aggregate_rows),
        "top_n": top_n,
        "split": split,
        "target_range_m": target_range_m,
        "target_window_m": target_window_m,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "runs": [{key: json_safe(value) for key, value in row.items() if key != "rows"} for row in run_summaries],
        "methods": [{key: json_safe(value) for key, value in row.items()} for row in aggregate_rows],
    }
    write_csv(output_dir / "frontier_profile_matrix_runs.csv", [{k: v for k, v in row.items() if k != "rows"} for row in run_summaries])
    write_csv(output_dir / "frontier_profile_matrix_methods.csv", aggregate_rows)
    write_json(output_dir / "frontier_profile_matrix_summary.json", summary)
    write_matrix_summary_txt(output_dir / "frontier_profile_matrix_summary.txt", summary=summary, aggregate_rows=aggregate_rows)
    plot_matrix_method_pass_counts(output_dir / "frontier_profile_matrix_pass_counts.png", aggregate_rows)
    plot_matrix_method_rmse(output_dir / "frontier_profile_matrix_rmse.png", aggregate_rows)
    plot_matrix_method_target_errors(output_dir / "frontier_profile_matrix_target_errors.png", aggregate_rows)
    return summary


def short_method_label(method: object, *, max_len: int = 44) -> str:
    text = str(method)
    if len(text) <= max_len:
        return text
    return text[: max_len - 3] + "..."


def top_plot_rows(rows: Sequence[Mapping[str, object]], *, limit: int = 12) -> list[Mapping[str, object]]:
    return list(rows[:limit])


def plot_matrix_method_pass_counts(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    plot_rows = top_plot_rows(rows)
    if not plot_rows:
        return
    labels = [short_method_label(row.get("method", "")) for row in plot_rows]
    pass_counts = np.asarray([finite_float(row.get("pass_count"), 0.0) for row in plot_rows], dtype=float)
    expected_counts = np.asarray([finite_float(row.get("expected_run_count"), 0.0) for row in plot_rows], dtype=float)
    y = np.arange(len(plot_rows), dtype=float)
    plt.figure(figsize=(12, max(5, 0.42 * len(plot_rows) + 1.5)))
    plt.barh(y, expected_counts, color="0.85", label="expected runs")
    plt.barh(y, pass_counts, color="tab:green", label="PASS runs")
    plt.yticks(y, labels)
    plt.xlabel("run count")
    plt.title("Common-CSV frontier matrix PASS coverage")
    plt.grid(True, axis="x", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def plot_matrix_method_rmse(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    plot_rows = top_plot_rows(rows)
    if not plot_rows:
        return
    labels = [short_method_label(row.get("method", "")) for row in plot_rows]
    mean_values = np.asarray(
        [finite_float(row.get("validation_range_rmse_corrected_mean_m"), 0.0) for row in plot_rows],
        dtype=float,
    )
    max_values = np.asarray(
        [finite_float(row.get("validation_range_rmse_corrected_max_m"), 0.0) for row in plot_rows],
        dtype=float,
    )
    y = np.arange(len(plot_rows), dtype=float)
    height = 0.36
    plt.figure(figsize=(12, max(5, 0.42 * len(plot_rows) + 1.5)))
    plt.barh(y - height / 2.0, mean_values, height=height, color="tab:blue", label="mean")
    plt.barh(y + height / 2.0, max_values, height=height, color="tab:orange", label="max")
    plt.yticks(y, labels)
    plt.xlabel("corrected range RMSE (m)")
    plt.title("Common-CSV frontier matrix range RMSE")
    plt.grid(True, axis="x", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def plot_matrix_method_target_errors(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    plot_rows = top_plot_rows(rows)
    if not plot_rows:
        return
    labels = [short_method_label(row.get("method", "")) for row in plot_rows]
    values = np.asarray(
        [finite_float(row.get("target_max_abs_error_corrected_max_m"), 0.0) for row in plot_rows],
        dtype=float,
    )
    y = np.arange(len(plot_rows), dtype=float)
    plt.figure(figsize=(12, max(5, 0.42 * len(plot_rows) + 1.5)))
    plt.barh(y, values, color="tab:purple", label="1852m worst-case")
    plt.axvline(10.0, color="tab:red", linestyle="--", linewidth=1, label="10m limit")
    plt.yticks(y, labels)
    plt.xlabel("target-window max abs range error (m)")
    plt.title("Common-CSV frontier matrix 1852m target error")
    plt.grid(True, axis="x", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def main() -> int:
    args = parse_args()
    summary = run_frontier_profile_matrix(
        frontier_csv=args.frontier_csv,
        sweep_root=args.sweep_root,
        output_dir=args.output_dir,
        runtime_profiles_dir=args.runtime_profiles_dir,
        active_profile_json=args.active_profile_json,
        active_label=args.active_label,
        top_n=args.top_n,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
