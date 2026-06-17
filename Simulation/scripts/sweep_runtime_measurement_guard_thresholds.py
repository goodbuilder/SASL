#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
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


def parse_float_list(value: str) -> list[float]:
    parsed: list[float] = []
    for item in str(value).split(","):
        text = item.strip()
        if not text:
            continue
        parsed.append(float(text))
    if not parsed:
        raise argparse.ArgumentTypeError("expected at least one comma-separated float")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep transfer-risk fallback thresholds for the runtime measurement guard. "
            "Selection uses raw RDF/disparity and transfer-matrix metadata only; truth is audit-only."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--transfer-matrix-csv", type=Path)
    parser.add_argument("--rmse-limits-m", type=parse_float_list, default=parse_float_list("0.9,1.1,1.3,1.5,1.8"))
    parser.add_argument("--target-limits-m", type=parse_float_list, default=parse_float_list("1.5,2.0,2.5,3.0,4.0"))
    parser.add_argument("--candidate-source-mode", choices=("minimal_set", "all_catalog"), default="all_catalog")
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--transfer-risk-target-weight", type=float, default=0.10)
    parser.add_argument("--include-target-in-centroids", action="store_true")
    parser.add_argument(
        "--exclude-target-scenario-from-library",
        action="store_true",
        help=(
            "When auditing a sweep, remove the target scenario from both the raw-feature "
            "library and candidate source scenarios for each guard run."
        ),
    )
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def threshold_run_dir(output_dir: Path, rmse_limit_m: float, target_limit_m: float) -> Path:
    return output_dir / f"rmse_{rmse_limit_m:.3f}_target_{target_limit_m:.3f}".replace(".", "p")


def score_threshold_row(row: Mapping[str, object]) -> float:
    pass_count = finite_float(row.get("pass_count"), 0.0)
    audit_count = finite_float(row.get("audit_count"), 0.0)
    fail_count = max(0.0, audit_count - pass_count)
    corrected_max = finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf"))
    corrected_mean = finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf"))
    target_max = finite_float(row.get("target_max_abs_error_max_m"), float("inf"))
    cluster_mean = finite_float(row.get("mean_transfer_cluster_size"), float("inf"))
    return 1000.0 * fail_count + corrected_max + 0.25 * corrected_mean + 0.10 * target_max + 0.02 * cluster_mean


def read_audit_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    import csv

    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def cluster_stats(audit_rows: Sequence[Mapping[str, str]]) -> tuple[float, int]:
    sizes: list[int] = []
    fallback_count = 0
    for row in audit_rows:
        cluster = [item for item in str(row.get("transfer_confusion_cluster", "")).split(",") if item]
        if cluster:
            sizes.append(len(cluster))
        if str(row.get("fallback_reason", "")).strip():
            fallback_count += 1
    return (float(np.mean(sizes)) if sizes else 0.0, fallback_count)


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime measurement guard transfer-risk threshold sweep",
        "",
        "Purpose:",
        (
            "Find stable low-risk transfer thresholds for selecting StereoDistance runtime profiles from "
            "raw RDF/disparity measurements without runtime truth."
        ),
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "selection_contract: raw RDF/disparity + transfer-matrix metadata for selection; truth is audit-only",
        "",
        "Recommendation:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Threshold rows:"])
    for row in rows:
        lines.append(
            "rmse_limit={rmse} target_limit={target} pass={passes}/{total} "
            "rmse_mean/max={mean}/{maxv} target_max={target_max} score={score}".format(
                rmse=row.get("transfer_low_risk_rmse_limit_m", ""),
                target=row.get("transfer_low_risk_target_limit_m", ""),
                passes=row.get("pass_count", ""),
                total=row.get("audit_count", ""),
                mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                maxv=row.get("validation_range_rmse_corrected_max_m", ""),
                target_max=row.get("target_max_abs_error_max_m", ""),
                score=row.get("threshold_selection_score", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_threshold_heatmap(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    rmse_limits = sorted({float(row["transfer_low_risk_rmse_limit_m"]) for row in rows})
    target_limits = sorted({float(row["transfer_low_risk_target_limit_m"]) for row in rows})
    pass_matrix = np.zeros((len(target_limits), len(rmse_limits)), dtype=float)
    score_matrix = np.zeros_like(pass_matrix)
    for row in rows:
        x = rmse_limits.index(float(row["transfer_low_risk_rmse_limit_m"]))
        y = target_limits.index(float(row["transfer_low_risk_target_limit_m"]))
        pass_matrix[y, x] = float(row["pass_count"])
        score_matrix[y, x] = float(row["threshold_selection_score"])
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    for axis, matrix, title in (
        (axes[0], pass_matrix, "Pass count"),
        (axes[1], score_matrix, "Selection score"),
    ):
        image = axis.imshow(matrix, aspect="auto", origin="lower", cmap="viridis")
        axis.set_xticks(np.arange(len(rmse_limits)), [f"{value:.2f}" for value in rmse_limits])
        axis.set_yticks(np.arange(len(target_limits)), [f"{value:.2f}" for value in target_limits])
        axis.set_xlabel("low-risk RMSE limit (m)")
        axis.set_ylabel("low-risk target limit (m)")
        axis.set_title(title)
        for y in range(matrix.shape[0]):
            for x in range(matrix.shape[1]):
                text = f"{matrix[y, x]:.0f}" if title == "Pass count" else f"{matrix[y, x]:.2f}"
                axis.text(x, y, text, ha="center", va="center", color="white", fontsize=8)
        fig.colorbar(image, ax=axis, shrink=0.85)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def sweep_thresholds(args: argparse.Namespace) -> dict[str, object]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    for rmse_limit_m in args.rmse_limits_m:
        for target_limit_m in args.target_limits_m:
            run_dir = threshold_run_dir(args.output_dir / "runs", rmse_limit_m, target_limit_m)
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
                selection_policy="transfer_risk_fallback",
                transfer_matrix_csv=args.transfer_matrix_csv,
                transfer_low_risk_rmse_limit_m=rmse_limit_m,
                transfer_low_risk_target_limit_m=target_limit_m,
                transfer_risk_target_weight=args.transfer_risk_target_weight,
                knn_neighbor_count=5,
                audit_all_sweep_seeds=True,
                include_target_in_centroids=args.include_target_in_centroids,
                exclude_target_scenario_from_library=getattr(args, "exclude_target_scenario_from_library", False),
                allow_unvalidated=args.allow_unvalidated,
            )
            summary = dict(run_guard(guard_args))
            audit_rows = read_audit_rows(run_dir / "runtime_measurement_guard_audit.csv")
            mean_cluster_size, fallback_count = cluster_stats(audit_rows)
            summary["run_dir"] = str(run_dir)
            summary["mean_transfer_cluster_size"] = mean_cluster_size
            summary["fallback_count"] = fallback_count
            summary["threshold_selection_score"] = score_threshold_row(summary)
            rows.append(summary)
    rows = sorted(
        rows,
        key=lambda row: (
            finite_float(row.get("threshold_selection_score"), float("inf")),
            -finite_float(row.get("pass_count"), 0.0),
            finite_float(row.get("transfer_low_risk_rmse_limit_m"), float("inf")),
            finite_float(row.get("transfer_low_risk_target_limit_m"), float("inf")),
        ),
    )
    for index, row in enumerate(rows, start=1):
        row["threshold_selection_rank"] = index
    best = rows[0] if rows else {}
    summary = {
        "mode": "runtime_measurement_guard_transfer_threshold_sweep",
        "candidate_source_mode": args.candidate_source_mode,
        "sweep_count": len(rows),
        "recommended_rmse_limit_m": best.get("transfer_low_risk_rmse_limit_m", ""),
        "recommended_target_limit_m": best.get("transfer_low_risk_target_limit_m", ""),
        "recommended_pass_count": best.get("pass_count", ""),
        "recommended_audit_count": best.get("audit_count", ""),
        "recommended_validation_range_rmse_corrected_mean_m": best.get(
            "validation_range_rmse_corrected_mean_m", ""
        ),
        "recommended_validation_range_rmse_corrected_max_m": best.get("validation_range_rmse_corrected_max_m", ""),
        "recommended_target_max_abs_error_max_m": best.get("target_max_abs_error_max_m", ""),
        "recommended_threshold_selection_score": best.get("threshold_selection_score", ""),
        "recommended_run_dir": best.get("run_dir", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": "raw RDF/disparity + transfer-matrix metadata for selection; truth is audit-only",
        "exclude_target_scenario_from_library": bool(getattr(args, "exclude_target_scenario_from_library", False)),
    }
    write_csv(args.output_dir / "runtime_measurement_guard_threshold_sweep.csv", rows)
    write_json(args.output_dir / "runtime_measurement_guard_threshold_sweep_summary.json", summary)
    write_summary_txt(args.output_dir / "runtime_measurement_guard_threshold_sweep_summary.txt", summary, rows)
    plot_threshold_heatmap(args.output_dir / "runtime_measurement_guard_threshold_sweep_heatmap.png", rows)
    return summary


def main() -> int:
    summary = sweep_thresholds(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
