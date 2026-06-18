#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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

from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from stereodistance_profile_replay import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    load_profile,
    replay_rows,
)


RUNTIME_INPUT_CONTRACT = (
    "common replay uses calibration-time truth for evaluation only; runtime profiles consume "
    "stereo-center RDF(right,down,forward) plus measured disparity and publish FRD+range"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay multiple StereoDistance active runtime profiles on the same calibration CSV. "
            "This is an offline apples-to-apples comparison; PX4/RTK truth is never a runtime input."
        )
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument(
        "--profile",
        action="append",
        required=True,
        metavar="LABEL:PROFILE_JSON",
        help="Profile candidate. Repeat for every active_runtime_profile.json to compare.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    return parser.parse_args()


def parse_profile_spec(spec: str) -> tuple[str, Path]:
    parts = spec.split(":", 1)
    if len(parts) != 2 or not parts[0].strip() or not parts[1].strip():
        raise ValueError("--profile must be LABEL:PROFILE_JSON")
    return parts[0].strip(), Path(parts[1])


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def json_safe(value: object) -> object:
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return None
    return value


def profile_payload(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def comparison_score(row: Mapping[str, object]) -> float:
    if not parse_bool(row.get("pass_common_replay")):
        return float("inf")
    corrected_rmse = finite_float(row.get("validation_range_rmse_corrected_m"), float("inf"))
    target_max = finite_float(row.get("target_max_abs_error_corrected_m"), float("inf"))
    position_rmse = finite_float(row.get("validation_position_rmse_corrected_m"), float("inf"))
    if not all(math.isfinite(value) for value in (corrected_rmse, target_max, position_rmse)):
        return float("inf")
    return corrected_rmse + target_max + 0.25 * position_rmse


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def axis_nonregression_pass(summary: Mapping[str, object]) -> bool:
    for axis in ("right", "down", "forward"):
        raw = finite_float(summary.get(f"validation_{axis}_rmse_raw_m"), float("inf"))
        corrected = finite_float(summary.get(f"validation_{axis}_rmse_corrected_m"), float("inf"))
        if not math.isfinite(raw) or not math.isfinite(corrected) or corrected > raw + 1.0e-9:
            return False
    return True


def compare_profiles_on_common_csv(
    *,
    input_csv: Path,
    profile_specs: Sequence[str],
    output_dir: Path,
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
) -> dict[str, object]:
    rows = read_csv(input_csv)
    profile_rows: list[dict[str, object]] = []
    prediction_rows: list[dict[str, object]] = []
    for spec in profile_specs:
        label, profile_json = parse_profile_spec(spec)
        payload = profile_payload(profile_json)
        profile = load_profile(profile_json)
        predictions, replay_summary = replay_rows(
            rows,
            profile,
            split_filter=split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
        )
        if not predictions:
            raise ValueError(f"profile {label!r} replayed no rows on {input_csv}")
        range_raw = finite_float(replay_summary.get("validation_range_rmse_raw_m"), float("inf"))
        range_corrected = finite_float(replay_summary.get("validation_range_rmse_corrected_m"), float("inf"))
        target_max = finite_float(replay_summary.get("target_max_abs_error_corrected_m"), float("inf"))
        target_count = int(float(replay_summary.get("target_window_count", 0) or 0))
        range_improvement_pass = (
            math.isfinite(range_raw) and math.isfinite(range_corrected) and range_corrected < range_raw
        )
        axis_pass = axis_nonregression_pass(replay_summary)
        target_pass = target_count > 0 and math.isfinite(target_max) and target_max <= 10.0
        pass_common_replay = range_improvement_pass and axis_pass and target_pass
        profile_row = {
            "candidate": label,
            "profile_json": str(profile_json),
            "method": payload.get("method", ""),
            "profile_id": payload.get("profile_id", ""),
            "profile_mode": payload.get("mode", ""),
            "evaluated_row_count": replay_summary.get("evaluated_row_count", 0),
            "skipped_row_count": replay_summary.get("skipped_row_count", 0),
            "validation_range_rmse_raw_m": replay_summary.get("validation_range_rmse_raw_m"),
            "validation_range_rmse_corrected_m": replay_summary.get("validation_range_rmse_corrected_m"),
            "validation_range_rmse_improvement_m": range_raw - range_corrected,
            "range_rmse_improvement_pass": range_improvement_pass,
            "validation_position_rmse_raw_m": replay_summary.get("validation_position_rmse_raw_m"),
            "validation_position_rmse_corrected_m": replay_summary.get("validation_position_rmse_corrected_m"),
            "validation_right_rmse_raw_m": replay_summary.get("validation_right_rmse_raw_m"),
            "validation_right_rmse_corrected_m": replay_summary.get("validation_right_rmse_corrected_m"),
            "validation_down_rmse_raw_m": replay_summary.get("validation_down_rmse_raw_m"),
            "validation_down_rmse_corrected_m": replay_summary.get("validation_down_rmse_corrected_m"),
            "validation_forward_rmse_raw_m": replay_summary.get("validation_forward_rmse_raw_m"),
            "validation_forward_rmse_corrected_m": replay_summary.get("validation_forward_rmse_corrected_m"),
            "rdf_axis_nonregression_pass": axis_pass,
            "target_window_count": replay_summary.get("target_window_count", 0),
            "target_abs_mean_error_corrected_m": replay_summary.get("target_abs_mean_error_corrected_m"),
            "target_max_abs_error_corrected_m": replay_summary.get("target_max_abs_error_corrected_m"),
            "target_max_error_pass": target_pass,
            "pass_common_replay": pass_common_replay,
            "runtime_truth_used": False,
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        }
        profile_row["common_csv_score"] = comparison_score(profile_row)
        profile_rows.append(profile_row)
        for prediction in predictions:
            prediction_rows.append(
                {
                    "candidate": label,
                    "method": payload.get("method", ""),
                    **prediction,
                }
            )
    ranked = sorted(profile_rows, key=lambda row: (comparison_score(row), str(row.get("candidate", ""))))
    passing = [row for row in ranked if parse_bool(row.get("pass_common_replay"))]
    selected = passing[0] if passing else ranked[0]
    summary = {
        "status": "PASS" if passing else "FAIL",
        "selected_candidate": selected.get("candidate", ""),
        "selected_method": selected.get("method", ""),
        "reason": (
            "lowest passing common-CSV corrected RMSE/1852m target/position score"
            if passing
            else "no profile passed common-CSV range/axis/target gates"
        ),
        "input_csv": str(input_csv),
        "split": split,
        "candidate_count": len(profile_rows),
        "passing_candidate_count": len(passing),
        "target_range_m": target_range_m,
        "target_window_m": target_window_m,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "rows": [{key: json_safe(value) for key, value in row.items()} for row in ranked],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "common_csv_profile_comparison.csv", ranked)
    write_csv(output_dir / "common_csv_profile_predictions.csv", prediction_rows)
    target_stats = target_window_stats(
        prediction_rows,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    write_csv(output_dir / "common_csv_target_window_stats.csv", target_stats)
    write_json(
        output_dir / "common_csv_target_window_stats.json",
        {
            "target_range_m": target_range_m,
            "target_window_m": target_window_m,
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "runtime_truth_used": False,
            "rows": [{key: json_safe(value) for key, value in row.items()} for row in target_stats],
        },
    )
    write_target_stats_txt(output_dir / "common_csv_target_window_stats.txt", target_stats)
    write_json(output_dir / "common_csv_profile_comparison.json", summary)
    write_summary_txt(output_dir / "common_csv_profile_comparison.txt", summary, ranked)
    plot_range_errors(output_dir / "common_csv_profile_range_errors.png", prediction_rows)
    plot_axis_rmse(output_dir / "common_csv_profile_axis_rmse.png", ranked)
    plot_residual_disparity(
        output_dir / "common_csv_residual_disparity_by_range.png",
        rows,
        split=split,
    )
    return summary


def target_window_stats(
    prediction_rows: Sequence[Mapping[str, object]],
    *,
    target_range_m: float,
    target_window_m: float,
) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in prediction_rows:
        grouped.setdefault(str(row.get("candidate", "")), []).append(row)
    rows: list[dict[str, object]] = []
    for label, items in grouped.items():
        target_items = [
            row
            for row in items
            if abs(finite_float(row.get("truth_range_m"), float("inf")) - target_range_m) <= target_window_m
        ]
        corrected_errors = [
            finite_float(row.get("corrected_range_error_m"), float("nan")) for row in target_items
        ]
        raw_errors = [finite_float(row.get("raw_range_error_m"), float("nan")) for row in target_items]
        corrected_errors = [value for value in corrected_errors if math.isfinite(value)]
        raw_errors = [value for value in raw_errors if math.isfinite(value)]
        method = str(items[0].get("method", "")) if items else ""
        rows.append(
            {
                "candidate": label,
                "method": method,
                "target_range_m": target_range_m,
                "target_window_m": target_window_m,
                "target_window_count": len(corrected_errors),
                "raw_abs_mean_error_m": (
                    sum(abs(value) for value in raw_errors) / len(raw_errors) if raw_errors else ""
                ),
                "corrected_abs_mean_error_m": (
                    sum(abs(value) for value in corrected_errors) / len(corrected_errors)
                    if corrected_errors
                    else ""
                ),
                "corrected_mean_error_m": (
                    sum(corrected_errors) / len(corrected_errors) if corrected_errors else ""
                ),
                "corrected_max_abs_error_m": (
                    max(abs(value) for value in corrected_errors) if corrected_errors else ""
                ),
                "target_error_limit_m": 10.0,
                "target_max_error_pass": (
                    bool(corrected_errors) and max(abs(value) for value in corrected_errors) <= 10.0
                ),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_truth_used": False,
            }
        )
    return sorted(
        rows,
        key=lambda row: (
            finite_float(row.get("corrected_max_abs_error_m"), float("inf")),
            str(row.get("candidate", "")),
        ),
    )


def write_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance common-CSV active profile comparison",
        "",
        f"status: {summary.get('status', '')}",
        f"selected_candidate: {summary.get('selected_candidate', '')}",
        f"selected_method: {summary.get('selected_method', '')}",
        f"reason: {summary.get('reason', '')}",
        f"input_csv: {summary.get('input_csv', '')}",
        f"split: {summary.get('split', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "",
        "Candidates:",
    ]
    for row in rows:
        lines.append(
            (
                "- {candidate}: method={method}, corrected_rmse={rmse}, "
                "target_max={target}, position_rmse={position}, axis_pass={axis}, "
                "pass={passed}, score={score}"
            ).format(
                candidate=row.get("candidate", ""),
                method=row.get("method", ""),
                rmse=row.get("validation_range_rmse_corrected_m", ""),
                target=row.get("target_max_abs_error_corrected_m", ""),
                position=row.get("validation_position_rmse_corrected_m", ""),
                axis=row.get("rdf_axis_nonregression_pass", ""),
                passed=row.get("pass_common_replay", ""),
                score=row.get("common_csv_score", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_target_stats_txt(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Common-CSV 1852m target-window range error statistics",
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "runtime_truth_used: False",
        "",
        "Candidates:",
    ]
    for row in rows:
        lines.append(
            (
                "- {candidate}: method={method}, count={count}, corrected_abs_mean={mean}, "
                "corrected_max_abs={max_abs}, pass={passed}"
            ).format(
                candidate=row.get("candidate", ""),
                method=row.get("method", ""),
                count=row.get("target_window_count", ""),
                mean=row.get("corrected_abs_mean_error_m", ""),
                max_abs=row.get("corrected_max_abs_error_m", ""),
                passed=row.get("target_max_error_pass", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_range_errors(path: Path, prediction_rows: Sequence[Mapping[str, object]]) -> None:
    if not prediction_rows:
        return
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in prediction_rows:
        grouped.setdefault(str(row.get("candidate", "")), []).append(row)
    plt.figure(figsize=(11, 6))
    first_group = next(iter(grouped.values()))
    truth_range = np.array([float(row["truth_range_m"]) for row in first_group], dtype=float)
    raw_error = np.array([float(row["raw_range_error_m"]) for row in first_group], dtype=float)
    order = np.argsort(truth_range)
    plt.plot(truth_range[order], raw_error[order], color="0.35", linewidth=1.2, label="raw")
    for label, rows in grouped.items():
        values = np.array([float(row["corrected_range_error_m"]) for row in rows], dtype=float)
        plt.plot(truth_range[order], values[order], linewidth=1.2, label=label)
    plt.axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    plt.axhline(-10.0, color="tab:red", linestyle="--", linewidth=1)
    plt.xlabel("truth range (m)")
    plt.ylabel("range error (m)")
    plt.title("Common-CSV active profile range error")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def plot_axis_rmse(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    axes = ("right", "down", "forward")
    labels = ["raw"] + [str(row.get("candidate", "")) for row in rows]
    values: list[list[float]] = []
    first = rows[0]
    values.append([finite_float(first.get(f"validation_{axis}_rmse_raw_m"), 0.0) for axis in axes])
    for row in rows:
        values.append([finite_float(row.get(f"validation_{axis}_rmse_corrected_m"), 0.0) for axis in axes])
    data = np.asarray(values, dtype=float)
    x = np.arange(len(axes), dtype=float)
    width = min(0.18, 0.8 / max(1, len(labels)))
    plt.figure(figsize=(11, 6))
    for index, label in enumerate(labels):
        offset = (index - (len(labels) - 1) / 2.0) * width
        plt.bar(x + offset, data[index], width=width, label=label)
    plt.xticks(x, axes)
    plt.ylabel("RMSE (m)")
    plt.xlabel("RDF axis")
    plt.title("Common-CSV RDF axis-wise RMSE")
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def residual_disparity_from_row(row: Mapping[str, str]) -> float | None:
    residual = finite_float(row.get("raw_disparity_residual_px"), float("nan"))
    if math.isfinite(residual):
        return residual
    measured = finite_float(row.get("measured_disparity_px"), float("nan"))
    truth = finite_float(row.get("true_disparity_px"), float("nan"))
    if math.isfinite(measured) and math.isfinite(truth):
        return measured - truth
    return None


def plot_residual_disparity(path: Path, rows: Sequence[Mapping[str, str]], *, split: str) -> None:
    truth_ranges: list[float] = []
    residuals: list[float] = []
    for row in rows:
        if split and str(row.get("split", "")) != split:
            continue
        truth_range = finite_float(row.get("truth_range_m"), float("nan"))
        residual = residual_disparity_from_row(row)
        if math.isfinite(truth_range) and residual is not None and math.isfinite(residual):
            truth_ranges.append(truth_range)
            residuals.append(residual)
    if not truth_ranges:
        return
    order = np.argsort(np.asarray(truth_ranges, dtype=float))
    truth_array = np.asarray(truth_ranges, dtype=float)[order]
    residual_array = np.asarray(residuals, dtype=float)[order]
    plt.figure(figsize=(11, 5))
    plt.scatter(truth_array, residual_array, s=18, alpha=0.75)
    plt.axhline(0.0, color="0.35", linewidth=1)
    plt.xlabel("truth range (m)")
    plt.ylabel("raw disparity residual (px)")
    plt.title("Common-CSV residual disparity by range")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def main() -> int:
    args = parse_args()
    summary = compare_profiles_on_common_csv(
        input_csv=args.input_csv,
        profile_specs=args.profile,
        output_dir=args.output_dir,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
