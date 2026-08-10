#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from audit_runtime_measurement_guard_oracle_regret import audit_oracle_regret  # noqa: E402
from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_by_runtime_measurement_guard import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    run_guard,
)
from sweep_runtime_measurement_guard_thresholds import parse_float_list  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep transfer-risk target-error weights and compare each runtime selector against "
            "the offline oracle. Selection remains truth-free; oracle regret is audit-only."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--transfer-matrix-csv", type=Path)
    parser.add_argument("--weights", type=parse_float_list, default=parse_float_list("0.0,0.05,0.1,0.2,0.4,0.8"))
    parser.add_argument("--candidate-source-mode", choices=("minimal_set", "all_catalog"), default="all_catalog")
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--transfer-low-risk-rmse-limit-m", type=float, default=1.3)
    parser.add_argument("--transfer-low-risk-target-limit-m", type=float, default=2.5)
    parser.add_argument("--include-target-in-centroids", action="store_true")
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def weight_run_dir(output_dir: Path, weight: float) -> Path:
    return output_dir / "runs" / f"target_weight_{weight:.3f}".replace(".", "p")


def score_weight_row(row: Mapping[str, object]) -> float:
    pass_count = finite_float(row.get("guard_pass_count"), 0.0)
    audit_count = finite_float(row.get("guard_audit_count"), 0.0)
    fail_count = max(0.0, audit_count - pass_count)
    target_abs_regret_max = finite_float(row.get("target_abs_regret_max_m"), float("inf"))
    target_max_regret_max = finite_float(row.get("target_max_regret_max_m"), float("inf"))
    guard_rmse_max = finite_float(row.get("guard_validation_range_rmse_corrected_max_m"), float("inf"))
    return 1000.0 * fail_count + guard_rmse_max + 0.50 * target_abs_regret_max + 0.20 * target_max_regret_max


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime measurement guard transfer-risk target-weight sweep",
        "",
        "Purpose:",
        (
            "Tune the transfer-risk target-error score weight while preserving truth-free runtime selection "
            "and measuring offline oracle regret."
        ),
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "selection_contract: raw RDF/disparity + transfer metadata for runtime selection; oracle regret is validation-only",
        "",
        "Recommendation:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Weight rows:"])
    for row in rows:
        lines.append(
            "weight={weight} pass={passes}/{total} guard_rmse_max={rmse_max} "
            "target_abs_regret_max={target_abs} target_max_regret_max={target_max} score={score}".format(
                weight=row.get("transfer_risk_target_weight", ""),
                passes=row.get("guard_pass_count", ""),
                total=row.get("guard_audit_count", ""),
                rmse_max=row.get("guard_validation_range_rmse_corrected_max_m", ""),
                target_abs=row.get("target_abs_regret_max_m", ""),
                target_max=row.get("target_max_regret_max_m", ""),
                score=row.get("target_weight_selection_score", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_weight_sweep(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    ordered = sorted(rows, key=lambda row: float(row["transfer_risk_target_weight"]))
    weights = np.asarray([float(row["transfer_risk_target_weight"]) for row in ordered], dtype=float)
    pass_counts = np.asarray([float(row["guard_pass_count"]) for row in ordered], dtype=float)
    target_abs = np.asarray([float(row["target_abs_regret_max_m"]) for row in ordered], dtype=float)
    target_max = np.asarray([float(row["target_max_regret_max_m"]) for row in ordered], dtype=float)
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True, constrained_layout=True)
    axes[0].plot(weights, pass_counts, marker="o")
    axes[0].set_ylabel("pass count")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(weights, target_abs, marker="o", color="tab:orange")
    axes[1].set_ylabel("target abs regret max (m)")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(weights, target_max, marker="o", color="tab:green")
    axes[2].set_ylabel("target max regret max (m)")
    axes[2].set_xlabel("transfer risk target weight")
    axes[2].grid(True, alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def sweep_target_weights(args: argparse.Namespace) -> dict[str, object]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    for weight in args.weights:
        run_dir = weight_run_dir(args.output_dir, weight)
        guard_dir = run_dir / "guard"
        regret_dir = run_dir / "oracle_regret"
        guard_args = SimpleNamespace(
            sweep_dir=args.sweep_dir,
            catalog_json=args.catalog_json,
            minimal_set_json=args.minimal_set_json,
            input_csv=None,
            output_dir=guard_dir,
            output_profile=None,
            split=args.split,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
            target_error_limit_m=args.target_error_limit_m,
            candidate_source_mode=args.candidate_source_mode,
            selection_policy="transfer_risk_fallback",
            transfer_matrix_csv=args.transfer_matrix_csv,
            transfer_low_risk_rmse_limit_m=args.transfer_low_risk_rmse_limit_m,
            transfer_low_risk_target_limit_m=args.transfer_low_risk_target_limit_m,
            transfer_risk_target_weight=weight,
            knn_neighbor_count=5,
            audit_all_sweep_seeds=True,
            include_target_in_centroids=args.include_target_in_centroids,
            allow_unvalidated=args.allow_unvalidated,
        )
        guard_summary = dict(run_guard(guard_args))
        regret_args = SimpleNamespace(
            catalog_json=args.catalog_json,
            guard_audit_csv=guard_dir / "runtime_measurement_guard_audit.csv",
            output_dir=regret_dir,
            minimal_set_json=args.minimal_set_json,
            candidate_source_mode=args.candidate_source_mode,
            split=args.split,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
            target_error_limit_m=args.target_error_limit_m,
            allow_unvalidated=args.allow_unvalidated,
        )
        regret_summary = dict(audit_oracle_regret(regret_args))
        row = {
            "transfer_risk_target_weight": weight,
            "run_dir": str(run_dir),
            "guard_run_dir": str(guard_dir),
            "oracle_regret_run_dir": str(regret_dir),
            "guard_pass_count": guard_summary.get("pass_count"),
            "guard_audit_count": guard_summary.get("audit_count"),
            "guard_validation_range_rmse_corrected_mean_m": guard_summary.get(
                "validation_range_rmse_corrected_mean_m"
            ),
            "guard_validation_range_rmse_corrected_max_m": guard_summary.get(
                "validation_range_rmse_corrected_max_m"
            ),
            "guard_target_abs_mean_error_max_m": guard_summary.get("target_abs_mean_error_max_m"),
            "guard_target_max_abs_error_max_m": guard_summary.get("target_max_abs_error_max_m"),
            "selected_oracle_match_count": regret_summary.get("selected_oracle_match_count"),
            "rmse_regret_mean_m": regret_summary.get("rmse_regret_mean_m"),
            "rmse_regret_max_m": regret_summary.get("rmse_regret_max_m"),
            "target_abs_regret_mean_m": regret_summary.get("target_abs_regret_mean_m"),
            "target_abs_regret_max_m": regret_summary.get("target_abs_regret_max_m"),
            "target_max_regret_mean_m": regret_summary.get("target_max_regret_mean_m"),
            "target_max_regret_max_m": regret_summary.get("target_max_regret_max_m"),
        }
        row["target_weight_selection_score"] = score_weight_row(row)
        rows.append(row)
    rows = sorted(
        rows,
        key=lambda row: (
            finite_float(row.get("target_weight_selection_score"), float("inf")),
            -finite_float(row.get("guard_pass_count"), 0.0),
            finite_float(row.get("transfer_risk_target_weight"), float("inf")),
        ),
    )
    for index, row in enumerate(rows, start=1):
        row["target_weight_selection_rank"] = index
    best = rows[0] if rows else {}
    summary = {
        "mode": "runtime_measurement_guard_target_weight_sweep",
        "candidate_source_mode": args.candidate_source_mode,
        "weight_count": len(rows),
        "recommended_transfer_risk_target_weight": best.get("transfer_risk_target_weight", ""),
        "recommended_guard_pass_count": best.get("guard_pass_count", ""),
        "recommended_guard_audit_count": best.get("guard_audit_count", ""),
        "recommended_guard_validation_range_rmse_corrected_max_m": best.get(
            "guard_validation_range_rmse_corrected_max_m", ""
        ),
        "recommended_target_abs_regret_max_m": best.get("target_abs_regret_max_m", ""),
        "recommended_target_max_regret_max_m": best.get("target_max_regret_max_m", ""),
        "recommended_target_weight_selection_score": best.get("target_weight_selection_score", ""),
        "recommended_run_dir": best.get("run_dir", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": "raw RDF/disparity + transfer metadata for runtime selection; oracle regret is validation-only",
    }
    write_csv(args.output_dir / "runtime_measurement_guard_target_weight_sweep.csv", rows)
    write_json(args.output_dir / "runtime_measurement_guard_target_weight_sweep_summary.json", summary)
    write_summary_txt(args.output_dir / "runtime_measurement_guard_target_weight_sweep_summary.txt", summary, rows)
    plot_weight_sweep(args.output_dir / "runtime_measurement_guard_target_weight_sweep.png", rows)
    return summary


def main() -> int:
    summary = sweep_target_weights(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
