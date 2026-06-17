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
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare StereoDistance selector operating modes: truth-free runtime guard versus "
            "calibration-time truth replay. This is an offline audit; runtime truth remains forbidden."
        )
    )
    parser.add_argument("--runtime-guard-audit-csv", type=Path, required=True)
    parser.add_argument("--calibration-replay-runs-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def seed_key(row: Mapping[str, str]) -> tuple[str, str]:
    scenario = str(row.get("target_scenario", ""))
    seed = str(row.get("target_seed", row.get("seed", "")))
    if seed and not seed.startswith("seed_"):
        seed = f"seed_{seed}"
    return scenario, seed


def summarize(values: Sequence[float], reducer: str) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return float("nan")
    arr = np.asarray(finite, dtype=float)
    if reducer == "mean":
        return float(np.mean(arr))
    if reducer == "max":
        return float(np.max(arr))
    if reducer == "min":
        return float(np.min(arr))
    raise ValueError(f"unsupported reducer: {reducer}")


def mode_summary(mode: str, rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    rmse = [finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows]
    target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")) for row in rows]
    target_max = [finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")) for row in rows]
    pass_values = [str(row.get("pass", "")).strip() == "True" for row in rows]
    return {
        "operating_mode": mode,
        "row_count": len(rows),
        "pass_count": sum(1 for value in pass_values if value),
        "validation_range_rmse_corrected_mean_m": summarize(rmse, "mean"),
        "validation_range_rmse_corrected_max_m": summarize(rmse, "max"),
        "target_abs_mean_error_corrected_mean_m": summarize(target_abs, "mean"),
        "target_abs_mean_error_corrected_max_m": summarize(target_abs, "max"),
        "target_max_abs_error_corrected_max_m": summarize(target_max, "max"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def audit_operating_modes(args: argparse.Namespace) -> dict[str, object]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    runtime_rows_raw = read_csv(args.runtime_guard_audit_csv)
    calibration_rows_raw = read_csv(args.calibration_replay_runs_csv)
    calibration_by_key = {seed_key(row): row for row in calibration_rows_raw}
    comparison_rows: list[dict[str, object]] = []
    runtime_rows: list[dict[str, object]] = []
    calibration_rows: list[dict[str, object]] = []
    for runtime in runtime_rows_raw:
        key = seed_key(runtime)
        calibration = calibration_by_key.get(key)
        if calibration is None:
            raise ValueError(f"missing calibration replay row for {key}")
        runtime_row = {
            "target_scenario": key[0],
            "target_seed": key[1],
            "source_scenario": runtime.get("selected_source_scenario", ""),
            "method": runtime.get("selected_method", ""),
            "pass": runtime.get("pass_runtime_measurement_guard_replay", ""),
            "validation_range_rmse_corrected_m": runtime.get("validation_range_rmse_corrected_m", ""),
            "target_abs_mean_error_corrected_m": runtime.get("target_abs_mean_error_corrected_m", ""),
            "target_max_abs_error_corrected_m": runtime.get("target_max_abs_error_corrected_m", ""),
        }
        calibration_row = {
            "target_scenario": key[0],
            "target_seed": key[1],
            "source_scenario": calibration.get("calibration_source_scenario", ""),
            "method": "calibration_replay_selected_catalog_profile",
            "pass": calibration.get("calibration_pass", ""),
            "validation_range_rmse_corrected_m": calibration.get("calibration_validation_range_rmse_corrected_m", ""),
            "target_abs_mean_error_corrected_m": calibration.get("calibration_target_abs_mean_error_corrected_m", ""),
            "target_max_abs_error_corrected_m": "",
        }
        runtime_rows.append(runtime_row)
        calibration_rows.append(calibration_row)
        runtime_rmse = finite_float(runtime_row["validation_range_rmse_corrected_m"], float("nan"))
        calibration_rmse = finite_float(calibration_row["validation_range_rmse_corrected_m"], float("nan"))
        runtime_target_abs = finite_float(runtime_row["target_abs_mean_error_corrected_m"], float("nan"))
        calibration_target_abs = finite_float(calibration_row["target_abs_mean_error_corrected_m"], float("nan"))
        comparison_rows.append(
            {
                "target_scenario": key[0],
                "target_seed": key[1],
                "runtime_source_scenario": runtime_row["source_scenario"],
                "calibration_source_scenario": calibration_row["source_scenario"],
                "runtime_pass": runtime_row["pass"],
                "calibration_pass": calibration_row["pass"],
                "runtime_validation_range_rmse_corrected_m": runtime_rmse,
                "calibration_validation_range_rmse_corrected_m": calibration_rmse,
                "calibration_minus_runtime_rmse_m": calibration_rmse - runtime_rmse,
                "runtime_target_abs_mean_error_corrected_m": runtime_target_abs,
                "calibration_target_abs_mean_error_corrected_m": calibration_target_abs,
                "calibration_minus_runtime_target_abs_m": calibration_target_abs - runtime_target_abs,
                "recommended_when_truth_available": "calibration_replay_all_catalog",
                "recommended_when_truth_unavailable": "runtime_transfer_risk_guard",
                "selection_contract": (
                    "calibration mode may use calibration-time truth; runtime mode must use raw RDF/disparity only"
                ),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    mode_rows = [
        mode_summary("runtime_truth_unavailable_transfer_risk_guard", runtime_rows),
        mode_summary("calibration_truth_available_replay_all_catalog", calibration_rows),
    ]
    runtime_summary = mode_rows[0]
    calibration_summary = mode_rows[1]
    summary = {
        "mode": "stereodistance_selector_operating_mode_audit",
        "row_count": len(comparison_rows),
        "recommended_when_truth_available": "calibration_replay_all_catalog",
        "recommended_when_truth_unavailable": "runtime_transfer_risk_guard",
        "runtime_pass_count": runtime_summary["pass_count"],
        "calibration_pass_count": calibration_summary["pass_count"],
        "runtime_validation_range_rmse_corrected_mean_m": runtime_summary[
            "validation_range_rmse_corrected_mean_m"
        ],
        "calibration_validation_range_rmse_corrected_mean_m": calibration_summary[
            "validation_range_rmse_corrected_mean_m"
        ],
        "runtime_target_abs_mean_error_corrected_mean_m": runtime_summary[
            "target_abs_mean_error_corrected_mean_m"
        ],
        "calibration_target_abs_mean_error_corrected_mean_m": calibration_summary[
            "target_abs_mean_error_corrected_mean_m"
        ],
        "calibration_lower_rmse_count": sum(
            1 for row in comparison_rows if finite_float(row.get("calibration_minus_runtime_rmse_m"), 0.0) < 0.0
        ),
        "calibration_lower_target_abs_count": sum(
            1
            for row in comparison_rows
            if finite_float(row.get("calibration_minus_runtime_target_abs_m"), 0.0) < 0.0
        ),
        "selection_contract": (
            "Use calibration replay only when calibration-time truth exists. Exported runtime profiles still "
            "consume raw RDF/disparity and must not consume truth."
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }
    write_csv(args.output_dir / "selector_operating_mode_comparison.csv", comparison_rows)
    write_csv(args.output_dir / "selector_operating_mode_summary.csv", mode_rows)
    write_json(args.output_dir / "selector_operating_mode_recommendation.json", summary)
    write_summary_txt(args.output_dir / "selector_operating_mode_summary.txt", summary, comparison_rows)
    plot_operating_modes(args.output_dir / "selector_operating_mode_comparison.png", comparison_rows)
    return summary


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "StereoDistance selector operating mode audit",
        "",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Recommendation:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Rows:"])
    for row in rows:
        lines.append(
            "{scenario}/{seed}: runtime={runtime} calibration={calibration} "
            "rmse_delta={rmse_delta} target_abs_delta={target_delta}".format(
                scenario=row.get("target_scenario", ""),
                seed=row.get("target_seed", ""),
                runtime=row.get("runtime_source_scenario", ""),
                calibration=row.get("calibration_source_scenario", ""),
                rmse_delta=row.get("calibration_minus_runtime_rmse_m", ""),
                target_delta=row.get("calibration_minus_runtime_target_abs_m", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_operating_modes(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    labels = [f"{row.get('target_scenario', '')}\n{row.get('target_seed', '')}" for row in rows]
    runtime_rmse = np.asarray(
        [finite_float(row.get("runtime_validation_range_rmse_corrected_m"), 0.0) for row in rows],
        dtype=float,
    )
    calibration_rmse = np.asarray(
        [finite_float(row.get("calibration_validation_range_rmse_corrected_m"), 0.0) for row in rows],
        dtype=float,
    )
    runtime_target = np.asarray(
        [finite_float(row.get("runtime_target_abs_mean_error_corrected_m"), 0.0) for row in rows],
        dtype=float,
    )
    calibration_target = np.asarray(
        [finite_float(row.get("calibration_target_abs_mean_error_corrected_m"), 0.0) for row in rows],
        dtype=float,
    )
    x = np.arange(len(rows))
    width = 0.42
    fig, axes = plt.subplots(2, 1, figsize=(max(10, len(rows) * 0.75), 8), sharex=True, constrained_layout=True)
    axes[0].bar(x - width / 2, runtime_rmse, width, label="runtime guard")
    axes[0].bar(x + width / 2, calibration_rmse, width, label="calibration replay")
    axes[0].set_ylabel("range RMSE (m)")
    axes[0].set_title("Selector operating modes: validation RMSE")
    axes[0].legend()
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(x - width / 2, runtime_target, width, label="runtime guard")
    axes[1].bar(x + width / 2, calibration_target, width, label="calibration replay")
    axes[1].set_ylabel("target abs mean error (m)")
    axes[1].set_title("Selector operating modes: 1852m target-window error")
    axes[1].legend()
    axes[1].grid(True, axis="y", alpha=0.3)
    axes[1].set_xticks(x, labels, rotation=45, ha="right")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    summary = audit_operating_modes(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
