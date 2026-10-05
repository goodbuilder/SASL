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

from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    candidate_source_scenarios,
    finite_float,
    read_csv,
    replay_candidate,
    replay_selection_score,
    write_csv,
    write_json,
)
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT, load_catalog  # noqa: E402
from sweep_runtime_measurement_guard_thresholds import read_audit_rows  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a truth-free runtime measurement guard selection against the offline truth-replay "
            "oracle for each calibration CSV. The oracle is for validation only and is not a runtime input."
        )
    )
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--guard-audit-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--candidate-source-mode", choices=("minimal_set", "all_catalog"), default="all_catalog")
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def candidate_sources(
    *,
    catalog: Mapping[str, object],
    minimal_set_json: Path | None,
    candidate_source_mode: str,
) -> list[str]:
    if candidate_source_mode == "minimal_set":
        return candidate_source_scenarios(catalog, minimal_set_json)
    if candidate_source_mode == "all_catalog":
        return candidate_source_scenarios(catalog, None)
    raise ValueError("candidate_source_mode must be 'minimal_set' or 'all_catalog'")


def oracle_sort_key(row: Mapping[str, object]) -> tuple[object, ...]:
    return (
        not bool(row.get("pass_replay")),
        finite_float(row.get("replay_selection_score"), float("inf")),
        finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
        finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")),
        finite_float(row.get("target_max_abs_error_corrected_m"), float("inf")),
        str(row.get("scenario", "")),
    )


def replay_all_candidates(
    *,
    catalog_json: Path,
    catalog: Mapping[str, object],
    source_scenarios: Sequence[str],
    rows: Sequence[Mapping[str, str]],
    split: str,
    target_range_m: float,
    target_window_m: float,
    target_error_limit_m: float,
    allow_unvalidated: bool,
) -> list[dict[str, object]]:
    candidate_rows: list[dict[str, object]] = []
    for scenario in source_scenarios:
        candidate_rows.append(
            replay_candidate(
                catalog_json=catalog_json,
                catalog=catalog,
                scenario=scenario,
                rows=rows,
                split=split,
                target_range_m=target_range_m,
                target_window_m=target_window_m,
                target_error_limit_m=target_error_limit_m,
                allow_unvalidated=allow_unvalidated,
                stability_splits=("train", "validation"),
            )
        )
    candidate_rows = sorted(candidate_rows, key=oracle_sort_key)
    for index, row in enumerate(candidate_rows, start=1):
        row["oracle_candidate_rank"] = index
    return candidate_rows


def choose_oracle(candidates: Sequence[Mapping[str, object]]) -> Mapping[str, object]:
    passing = [row for row in candidates if bool(row.get("pass_replay"))]
    if passing:
        return min(passing, key=oracle_sort_key)
    if not candidates:
        raise ValueError("no oracle candidates were replayed")
    return min(candidates, key=oracle_sort_key)


def summarize_float(rows: Sequence[Mapping[str, object]], key: str, reducer: str) -> float:
    values = [finite_float(row.get(key), float("nan")) for row in rows]
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return float("nan")
    arr = np.asarray(values, dtype=float)
    if reducer == "mean":
        return float(np.mean(arr))
    if reducer == "max":
        return float(np.max(arr))
    if reducer == "min":
        return float(np.min(arr))
    raise ValueError(f"unsupported reducer: {reducer}")


def audit_oracle_regret(args: argparse.Namespace) -> dict[str, object]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    catalog = load_catalog(args.catalog_json)
    sources = candidate_sources(
        catalog=catalog,
        minimal_set_json=args.minimal_set_json,
        candidate_source_mode=args.candidate_source_mode,
    )
    guard_rows = read_audit_rows(args.guard_audit_csv)
    if not guard_rows:
        raise ValueError(f"guard audit CSV has no rows: {args.guard_audit_csv}")
    comparison_rows: list[dict[str, object]] = []
    candidate_rows_all: list[dict[str, object]] = []
    for guard_row in guard_rows:
        input_csv = Path(str(guard_row.get("input_csv", "")))
        if not input_csv.is_file():
            raise ValueError(f"input_csv from guard audit does not exist: {input_csv}")
        sample_rows = read_csv(input_csv)
        candidates = replay_all_candidates(
            catalog_json=args.catalog_json,
            catalog=catalog,
            source_scenarios=sources,
            rows=sample_rows,
            split=args.split,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
            target_error_limit_m=args.target_error_limit_m,
            allow_unvalidated=args.allow_unvalidated,
        )
        oracle = choose_oracle(candidates)
        selected_source = str(guard_row.get("selected_source_scenario", ""))
        selected = next((row for row in candidates if str(row.get("scenario", "")) == selected_source), None)
        if selected is None:
            raise ValueError(f"selected source {selected_source!r} was not replayed for {input_csv}")
        target_scenario = str(guard_row.get("target_scenario", ""))
        target_seed = str(guard_row.get("target_seed", ""))
        for candidate in candidates:
            candidate_rows_all.append(
                {
                    "target_scenario": target_scenario,
                    "target_seed": target_seed,
                    "input_csv": str(input_csv),
                    **candidate,
                }
            )
        selected_rmse = finite_float(selected.get("validation_range_rmse_corrected_m"), float("nan"))
        oracle_rmse = finite_float(oracle.get("validation_range_rmse_corrected_m"), float("nan"))
        selected_target_abs = finite_float(selected.get("target_abs_mean_error_corrected_m"), float("nan"))
        oracle_target_abs = finite_float(oracle.get("target_abs_mean_error_corrected_m"), float("nan"))
        selected_target_max = finite_float(selected.get("target_max_abs_error_corrected_m"), float("nan"))
        oracle_target_max = finite_float(oracle.get("target_max_abs_error_corrected_m"), float("nan"))
        comparison_rows.append(
            {
                "target_scenario": target_scenario,
                "target_seed": target_seed,
                "input_csv": str(input_csv),
                "guard_selection_policy": guard_row.get("selection_policy", ""),
                "guard_preliminary_source_scenario": guard_row.get("preliminary_source_scenario", ""),
                "guard_selected_source_scenario": selected_source,
                "guard_selected_method": selected.get("method", ""),
                "guard_pass_replay": selected.get("pass_replay", ""),
                "guard_replay_rank": selected.get("oracle_candidate_rank", ""),
                "oracle_source_scenario": oracle.get("scenario", ""),
                "oracle_method": oracle.get("method", ""),
                "oracle_pass_replay": oracle.get("pass_replay", ""),
                "oracle_replay_rank": oracle.get("oracle_candidate_rank", ""),
                "selected_is_oracle": selected_source == str(oracle.get("scenario", "")),
                "selected_validation_range_rmse_corrected_m": selected_rmse,
                "oracle_validation_range_rmse_corrected_m": oracle_rmse,
                "rmse_regret_m": selected_rmse - oracle_rmse,
                "selected_target_abs_mean_error_corrected_m": selected_target_abs,
                "oracle_target_abs_mean_error_corrected_m": oracle_target_abs,
                "target_abs_regret_m": selected_target_abs - oracle_target_abs,
                "selected_target_max_abs_error_corrected_m": selected_target_max,
                "oracle_target_max_abs_error_corrected_m": oracle_target_max,
                "target_max_regret_m": selected_target_max - oracle_target_max,
                "selected_replay_selection_score": replay_selection_score(selected),
                "oracle_replay_selection_score": replay_selection_score(oracle),
                "replay_score_regret": replay_selection_score(selected) - replay_selection_score(oracle),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "selection_contract": (
                    "guard selection uses raw RDF/disparity; oracle uses calibration-time truth replay "
                    "for offline regret audit only"
                ),
            }
        )
    summary = {
        "mode": "runtime_measurement_guard_oracle_regret",
        "candidate_source_mode": args.candidate_source_mode,
        "sample_count": len(comparison_rows),
        "selected_oracle_match_count": sum(1 for row in comparison_rows if bool(row["selected_is_oracle"])),
        "selected_pass_count": sum(1 for row in comparison_rows if str(row.get("guard_pass_replay")) == "True"),
        "oracle_pass_count": sum(1 for row in comparison_rows if str(row.get("oracle_pass_replay")) == "True"),
        "rmse_regret_mean_m": summarize_float(comparison_rows, "rmse_regret_m", "mean"),
        "rmse_regret_max_m": summarize_float(comparison_rows, "rmse_regret_m", "max"),
        "target_abs_regret_mean_m": summarize_float(comparison_rows, "target_abs_regret_m", "mean"),
        "target_abs_regret_max_m": summarize_float(comparison_rows, "target_abs_regret_m", "max"),
        "target_max_regret_mean_m": summarize_float(comparison_rows, "target_max_regret_m", "mean"),
        "target_max_regret_max_m": summarize_float(comparison_rows, "target_max_regret_m", "max"),
        "replay_score_regret_mean": summarize_float(comparison_rows, "replay_score_regret", "mean"),
        "replay_score_regret_max": summarize_float(comparison_rows, "replay_score_regret", "max"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": "truth-free guard selection compared to offline truth-replay oracle; oracle is not runtime",
    }
    write_csv(args.output_dir / "runtime_measurement_guard_oracle_regret.csv", comparison_rows)
    write_csv(args.output_dir / "runtime_measurement_guard_oracle_candidates.csv", candidate_rows_all)
    write_json(args.output_dir / "runtime_measurement_guard_oracle_regret_summary.json", summary)
    write_summary_txt(args.output_dir / "runtime_measurement_guard_oracle_regret_summary.txt", summary, comparison_rows)
    plot_regret(args.output_dir / "runtime_measurement_guard_oracle_regret.png", comparison_rows)
    return summary


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime measurement guard oracle regret audit",
        "",
        "Purpose:",
        (
            "Measure how far a truth-free runtime selector is from the offline calibration-time oracle "
            "that can replay every catalog profile using truth."
        ),
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "selection_contract: runtime selector uses raw RDF/disparity; oracle is offline validation only",
        "",
        "Summary:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Rows:"])
    for row in rows:
        lines.append(
            "{target_scenario}/{target_seed}: selected={selected} oracle={oracle} "
            "selected_rank={rank} rmse_regret={rmse} target_abs_regret={target_abs} "
            "target_max_regret={target_max}".format(
                target_scenario=row.get("target_scenario", ""),
                target_seed=row.get("target_seed", ""),
                selected=row.get("guard_selected_source_scenario", ""),
                oracle=row.get("oracle_source_scenario", ""),
                rank=row.get("guard_replay_rank", ""),
                rmse=row.get("rmse_regret_m", ""),
                target_abs=row.get("target_abs_regret_m", ""),
                target_max=row.get("target_max_regret_m", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_regret(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    labels = [f"{row.get('target_scenario', '')}\n{row.get('target_seed', '')}" for row in rows]
    rmse = np.asarray([finite_float(row.get("rmse_regret_m"), 0.0) for row in rows], dtype=float)
    target_abs = np.asarray([finite_float(row.get("target_abs_regret_m"), 0.0) for row in rows], dtype=float)
    target_max = np.asarray([finite_float(row.get("target_max_regret_m"), 0.0) for row in rows], dtype=float)
    x = np.arange(len(rows))
    fig, axes = plt.subplots(3, 1, figsize=(max(10, len(rows) * 0.7), 9), sharex=True, constrained_layout=True)
    for axis, values, title in (
        (axes[0], rmse, "Range RMSE regret"),
        (axes[1], target_abs, "Target mean abs error regret"),
        (axes[2], target_max, "Target max abs error regret"),
    ):
        axis.bar(x, values)
        axis.axhline(0.0, color="black", linewidth=1)
        axis.set_ylabel("m")
        axis.set_title(title)
        axis.grid(True, axis="y", alpha=0.3)
    axes[-1].set_xticks(x, labels, rotation=45, ha="right")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    summary = audit_oracle_regret(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
