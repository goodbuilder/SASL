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
from select_stereodistance_profile_by_runtime_measurement_guard import sweep_calibration_csvs  # noqa: E402
from stereodistance_profile_replay import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    load_profile,
    replay_rows,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay one exported StereoDistance active runtime profile across every scenario/seed in a "
            "robustness sweep. Truth is used only for offline replay metrics; the profile consumes raw "
            "stereo-center RDF/disparity only."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument(
        "--target-scenarios",
        default="",
        help="Optional comma-separated scenario names to replay. Empty means every scenario/seed.",
    )
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_scenario_filter(value: str | Sequence[str] | None) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        items = value.split(",")
    else:
        items = list(value)
    scenarios: list[str] = []
    for item in items:
        scenario = str(item).strip()
        if scenario and scenario not in scenarios:
            scenarios.append(scenario)
    return scenarios


def pass_replay_summary(summary: Mapping[str, object], *, target_error_limit_m: float) -> tuple[bool, str]:
    raw_rmse = finite_float(summary.get("validation_range_rmse_raw_m"), float("inf"))
    corrected_rmse = finite_float(summary.get("validation_range_rmse_corrected_m"), float("inf"))
    improvement = raw_rmse - corrected_rmse
    target_count = int(float(summary.get("target_window_count", 0) or 0))
    target_abs = finite_float(summary.get("target_abs_mean_error_corrected_m"), float("inf"))
    target_max = finite_float(summary.get("target_max_abs_error_corrected_m"), float("inf"))
    reasons: list[str] = []
    if int(float(summary.get("evaluated_row_count", 0) or 0)) <= 0:
        reasons.append("no_evaluated_rows")
    if not math.isfinite(raw_rmse) or not math.isfinite(corrected_rmse):
        reasons.append("rmse_not_finite")
    if not math.isfinite(improvement) or improvement <= 0.0:
        reasons.append("no_positive_range_rmse_improvement")
    if target_count <= 0:
        reasons.append("target_window_missing")
    if not math.isfinite(target_abs) or target_abs > target_error_limit_m:
        reasons.append("target_abs_error_over_limit")
    if not math.isfinite(target_max) or target_max > target_error_limit_m:
        reasons.append("target_max_error_over_limit")
    return not reasons, "pass" if not reasons else ";".join(reasons)


def audit_active_profile_transfer(
    *,
    sweep_dir: Path,
    profile_json: Path,
    output_dir: Path,
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
    target_scenarios: Sequence[str] = (),
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    profile = load_profile(profile_json)
    rows_out: list[dict[str, object]] = []
    prediction_root = output_dir / "per_run_predictions"
    target_filter = set(target_scenarios)
    for scenario, seed, sample_csv in sweep_calibration_csvs(sweep_dir):
        if target_filter and scenario not in target_filter:
            continue
        rows = read_csv(sample_csv)
        predictions, summary = replay_rows(
            rows,
            profile,
            split_filter=split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
        )
        passed, reason = pass_replay_summary(summary, target_error_limit_m=target_error_limit_m)
        run_dir = prediction_root / scenario / seed
        write_csv(run_dir / "profile_replay_predictions.csv", predictions)
        write_csv(run_dir / "profile_replay_summary.csv", [summary])
        rows_out.append(
            {
                "target_scenario": scenario,
                "target_seed": seed,
                "sample_csv": str(sample_csv),
                "profile_json": str(profile_json),
                "profile_id": summary.get("profile_id", ""),
                "profile_mode": summary.get("profile_mode", ""),
                "pass_active_profile_transfer": passed,
                "pass_reason": reason,
                "evaluated_row_count": summary.get("evaluated_row_count", ""),
                "validation_range_rmse_raw_m": summary.get("validation_range_rmse_raw_m", ""),
                "validation_range_rmse_corrected_m": summary.get("validation_range_rmse_corrected_m", ""),
                "validation_range_rmse_improvement_m": finite_float(
                    summary.get("validation_range_rmse_raw_m"), float("nan")
                )
                - finite_float(summary.get("validation_range_rmse_corrected_m"), float("nan")),
                "target_window_count": summary.get("target_window_count", ""),
                "target_abs_mean_error_raw_m": summary.get("target_abs_mean_error_raw_m", ""),
                "target_abs_mean_error_corrected_m": summary.get("target_abs_mean_error_corrected_m", ""),
                "target_max_abs_error_raw_m": summary.get("target_max_abs_error_raw_m", ""),
                "target_max_abs_error_corrected_m": summary.get("target_max_abs_error_corrected_m", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": "active profile replay uses raw stereo-center RDF/disparity only; truth is audit-only",
            }
        )
    write_csv(output_dir / "active_profile_transfer_replay.csv", rows_out)
    summary = summarize_transfer(
        rows_out,
        profile_json=profile_json,
        sweep_dir=sweep_dir,
        split=split,
        target_scenarios=target_scenarios,
    )
    write_json(output_dir / "active_profile_transfer_summary.json", summary)
    write_summary_txt(output_dir / "active_profile_transfer_summary.txt", summary, rows_out)
    plot_transfer_summary(output_dir / "active_profile_transfer_summary.png", rows_out)
    return summary


def finite_values(rows: Sequence[Mapping[str, object]], key: str) -> list[float]:
    values = [finite_float(row.get(key), float("nan")) for row in rows]
    return [value for value in values if math.isfinite(value)]


def summarize_transfer(
    rows: Sequence[Mapping[str, object]],
    *,
    profile_json: Path,
    sweep_dir: Path,
    split: str,
    target_scenarios: Sequence[str] = (),
) -> dict[str, object]:
    rmse = finite_values(rows, "validation_range_rmse_corrected_m")
    improvement = finite_values(rows, "validation_range_rmse_improvement_m")
    target_abs = finite_values(rows, "target_abs_mean_error_corrected_m")
    target_max = finite_values(rows, "target_max_abs_error_corrected_m")
    pass_count = sum(1 for row in rows if bool(row.get("pass_active_profile_transfer")))
    return {
        "mode": "active_profile_transfer_replay",
        "profile_json": str(profile_json),
        "sweep_dir": str(sweep_dir),
        "split": split,
        "target_scenarios": ",".join(target_scenarios),
        "transfer_replay_count": len(rows),
        "transfer_replay_pass_count": pass_count,
        "pass_all_transfer_replays": bool(rows) and pass_count == len(rows),
        "validation_range_rmse_corrected_mean_m": float(np.mean(rmse)) if rmse else float("nan"),
        "validation_range_rmse_corrected_max_m": float(np.max(rmse)) if rmse else float("nan"),
        "validation_range_rmse_improvement_min_m": float(np.min(improvement)) if improvement else float("nan"),
        "target_abs_mean_error_mean_m": float(np.mean(target_abs)) if target_abs else float("nan"),
        "target_abs_mean_error_max_m": float(np.max(target_abs)) if target_abs else float("nan"),
        "target_max_abs_error_mean_m": float(np.mean(target_max)) if target_max else float("nan"),
        "target_max_abs_error_max_m": float(np.max(target_max)) if target_max else float("nan"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "active profile replay uses raw stereo-center RDF/disparity only; truth is audit-only",
    }


def write_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance active profile transfer replay",
        "",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "",
        "Summary:",
    ]
    for key in (
        "profile_json",
        "transfer_replay_pass_count",
        "transfer_replay_count",
        "pass_all_transfer_replays",
        "validation_range_rmse_corrected_mean_m",
        "validation_range_rmse_corrected_max_m",
        "validation_range_rmse_improvement_min_m",
        "target_abs_mean_error_mean_m",
        "target_abs_mean_error_max_m",
        "target_max_abs_error_mean_m",
        "target_max_abs_error_max_m",
    ):
        value = summary.get(key, "")
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Rows:"])
    for row in rows:
        lines.append(
            "{scenario}/{seed}: pass={passed}, rmse={rmse}, target_abs={target_abs}, "
            "target_max={target_max}, reason={reason}".format(
                scenario=row.get("target_scenario", ""),
                seed=row.get("target_seed", ""),
                passed=row.get("pass_active_profile_transfer", ""),
                rmse=row.get("validation_range_rmse_corrected_m", ""),
                target_abs=row.get("target_abs_mean_error_corrected_m", ""),
                target_max=row.get("target_max_abs_error_corrected_m", ""),
                reason=row.get("pass_reason", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_transfer_summary(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    labels = [f"{row.get('target_scenario', '')}\n{row.get('target_seed', '')}" for row in rows]
    rmse = np.asarray([finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows])
    target_max = np.asarray([finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")) for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(max(10, len(rows) * 0.55), 7), sharex=True, constrained_layout=True)
    x = np.arange(len(rows))
    axes[0].bar(x, rmse, color="tab:blue")
    axes[0].set_ylabel("RMSE corrected (m)")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(x, target_max, color="tab:orange")
    axes[1].axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    axes[1].set_ylabel("target max abs (m)")
    axes[1].set_xticks(x, labels, rotation=65, ha="right")
    axes[1].grid(True, axis="y", alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    summary = audit_active_profile_transfer(
        sweep_dir=args.sweep_dir,
        profile_json=args.profile_json,
        output_dir=args.output_dir,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        target_scenarios=parse_scenario_filter(args.target_scenarios),
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
