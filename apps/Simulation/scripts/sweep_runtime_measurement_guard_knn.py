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

from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_by_runtime_measurement_guard import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    run_guard,
)


def parse_int_list(value: str | Sequence[int]) -> list[int]:
    if isinstance(value, str):
        raw_items = value.split(",")
    else:
        raw_items = list(value)
    parsed: list[int] = []
    for item in raw_items:
        number = int(str(item).strip())
        if number <= 0:
            raise argparse.ArgumentTypeError("neighbor counts must be positive")
        if number not in parsed:
            parsed.append(number)
    if not parsed:
        raise argparse.ArgumentTypeError("expected at least one neighbor count")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep kNN runtime-measurement guard neighbor counts. Selection uses raw stereo-center "
            "RDF/disparity/image-center features plus transfer metadata only; truth is replay audit-only."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--transfer-matrix-csv", type=Path)
    parser.add_argument("--neighbor-counts", type=parse_int_list, default=parse_int_list("1,3,5,7,9"))
    parser.add_argument("--candidate-source-mode", choices=("minimal_set", "all_catalog"), default="all_catalog")
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--transfer-low-risk-rmse-limit-m", type=float, default=1.3)
    parser.add_argument("--transfer-low-risk-target-limit-m", type=float, default=2.5)
    parser.add_argument("--transfer-risk-target-weight", type=float, default=0.10)
    parser.add_argument("--include-target-in-centroids", action="store_true")
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def knn_run_dir(output_dir: Path, neighbor_count: int) -> Path:
    return output_dir / "runs" / f"k_{neighbor_count}"


def score_knn_row(row: Mapping[str, object]) -> float:
    pass_count = finite_float(row.get("pass_count"), 0.0)
    audit_count = finite_float(row.get("audit_count"), 0.0)
    fail_count = max(0.0, audit_count - pass_count)
    corrected_max = finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf"))
    corrected_mean = finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf"))
    target_max = finite_float(row.get("target_max_abs_error_max_m"), float("inf"))
    fallback_count = finite_float(row.get("fallback_count"), 0.0)
    return 1000.0 * fail_count + corrected_max + 0.25 * corrected_mean + 0.10 * target_max + 0.01 * fallback_count


def fallback_count_from_audit(path: Path) -> int:
    import csv

    if not path.is_file():
        return 0
    with path.open(newline="", encoding="utf-8") as handle:
        return sum(1 for row in csv.DictReader(handle) if str(row.get("fallback_reason", "")).strip())


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime measurement guard kNN sweep",
        "",
        "Purpose:",
        (
            "Tune the raw-measurement kNN selector neighbor count while preserving truth-free runtime "
            "selection and using truth only for offline replay audit."
        ),
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "selection_contract: raw RDF/disparity/image-center features + transfer metadata for runtime selection; truth is audit-only",
        "",
        "Recommendation:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "kNN rows:"])
    for row in rows:
        lines.append(
            "k={k} pass={passes}/{total} rmse_mean/max={mean}/{maxv} target_max={target_max} "
            "fallback={fallback} score={score}".format(
                k=row.get("knn_neighbor_count", ""),
                passes=row.get("pass_count", ""),
                total=row.get("audit_count", ""),
                mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                maxv=row.get("validation_range_rmse_corrected_max_m", ""),
                target_max=row.get("target_max_abs_error_max_m", ""),
                fallback=row.get("fallback_count", ""),
                score=row.get("knn_selection_score", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_knn_sweep(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    ordered = sorted(rows, key=lambda row: int(row["knn_neighbor_count"]))
    counts = np.asarray([int(row["knn_neighbor_count"]) for row in ordered], dtype=int)
    pass_counts = np.asarray([float(row["pass_count"]) for row in ordered], dtype=float)
    rmse_max = np.asarray([float(row["validation_range_rmse_corrected_max_m"]) for row in ordered], dtype=float)
    target_max = np.asarray([float(row["target_max_abs_error_max_m"]) for row in ordered], dtype=float)
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True, constrained_layout=True)
    axes[0].plot(counts, pass_counts, marker="o")
    axes[0].set_ylabel("pass count")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(counts, rmse_max, marker="o", color="tab:orange")
    axes[1].set_ylabel("RMSE max (m)")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(counts, target_max, marker="o", color="tab:green")
    axes[2].axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    axes[2].set_ylabel("target max abs (m)")
    axes[2].set_xlabel("kNN neighbor count")
    axes[2].grid(True, alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def sweep_knn(args: argparse.Namespace) -> dict[str, object]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    for neighbor_count in args.neighbor_counts:
        run_dir = knn_run_dir(args.output_dir, neighbor_count)
        guard_args = SimpleNamespace(
            sweep_dir=args.sweep_dir,
            catalog_json=args.catalog_json,
            minimal_set_json=args.minimal_set_json,
            input_csv=None,
            output_dir=run_dir,
            output_profile=None,
            split=args.split,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
            target_error_limit_m=args.target_error_limit_m,
            candidate_source_mode=args.candidate_source_mode,
            selection_policy="knn_transfer_risk",
            transfer_matrix_csv=args.transfer_matrix_csv,
            transfer_low_risk_rmse_limit_m=args.transfer_low_risk_rmse_limit_m,
            transfer_low_risk_target_limit_m=args.transfer_low_risk_target_limit_m,
            transfer_risk_target_weight=args.transfer_risk_target_weight,
            knn_neighbor_count=neighbor_count,
            audit_all_sweep_seeds=True,
            include_target_in_centroids=args.include_target_in_centroids,
            allow_unvalidated=args.allow_unvalidated,
        )
        row = dict(run_guard(guard_args))
        row["run_dir"] = str(run_dir)
        row["candidate_source_mode"] = args.candidate_source_mode
        row["selection_policy"] = "knn_transfer_risk"
        row["knn_neighbor_count"] = neighbor_count
        row["fallback_count"] = fallback_count_from_audit(run_dir / "runtime_measurement_guard_audit.csv")
        row["knn_selection_score"] = score_knn_row(row)
        rows.append(row)
    rows = sorted(
        rows,
        key=lambda row: (
            finite_float(row.get("knn_selection_score"), float("inf")),
            -finite_float(row.get("pass_count"), 0.0),
            finite_float(row.get("knn_neighbor_count"), float("inf")),
        ),
    )
    for index, row in enumerate(rows, start=1):
        row["knn_selection_rank"] = index
    best = rows[0] if rows else {}
    summary = {
        "mode": "runtime_measurement_guard_knn_sweep",
        "candidate_source_mode": args.candidate_source_mode,
        "neighbor_count_count": len(rows),
        "recommended_knn_neighbor_count": best.get("knn_neighbor_count", ""),
        "recommended_pass_count": best.get("pass_count", ""),
        "recommended_audit_count": best.get("audit_count", ""),
        "recommended_validation_range_rmse_corrected_mean_m": best.get("validation_range_rmse_corrected_mean_m", ""),
        "recommended_validation_range_rmse_corrected_max_m": best.get("validation_range_rmse_corrected_max_m", ""),
        "recommended_target_max_abs_error_max_m": best.get("target_max_abs_error_max_m", ""),
        "recommended_knn_selection_score": best.get("knn_selection_score", ""),
        "recommended_run_dir": best.get("run_dir", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "raw RDF/disparity/image-center features + transfer metadata for runtime selection; "
            "truth is replay audit-only"
        ),
    }
    write_csv(args.output_dir / "runtime_measurement_guard_knn_sweep.csv", rows)
    write_json(args.output_dir / "runtime_measurement_guard_knn_sweep_summary.json", summary)
    write_summary_txt(args.output_dir / "runtime_measurement_guard_knn_sweep_summary.txt", summary, rows)
    plot_knn_sweep(args.output_dir / "runtime_measurement_guard_knn_sweep.png", rows)
    return summary


def main() -> int:
    summary = sweep_knn(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
