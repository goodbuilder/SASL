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
from sweep_runtime_measurement_guard_thresholds import read_audit_rows  # noqa: E402


POLICIES = (
    "nearest_centroid",
    "robust_family_fallback",
    "transfer_risk_fallback",
    "intrinsic_scale_gap_transfer_risk",
    "feature_vote_transfer_risk",
    "knn_transfer_risk",
    "centroid_knn_union_transfer_risk",
    "centroid_knn_veto_transfer_risk",
)

POLICY_COMPLEXITY_PENALTY = {
    "nearest_centroid": 0.00,
    "transfer_risk_fallback": 0.02,
    "intrinsic_scale_gap_transfer_risk": 0.04,
    "robust_family_fallback": 0.05,
    "knn_transfer_risk": 0.08,
    "feature_vote_transfer_risk": 0.10,
    "centroid_knn_veto_transfer_risk": 0.12,
    "centroid_knn_union_transfer_risk": 0.14,
}


def parse_csv_list(value: str) -> list[str]:
    items = [item.strip() for item in str(value).split(",") if item.strip()]
    if not items:
        raise argparse.ArgumentTypeError("expected at least one comma-separated value")
    return items


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare runtime measurement guard selection policies on the same sweep. "
            "Selection uses raw RDF/disparity plus allowed metadata only; truth is audit-only."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--transfer-matrix-csv", type=Path)
    parser.add_argument("--policies", type=parse_csv_list, default=list(POLICIES))
    parser.add_argument(
        "--candidate-source-modes",
        type=parse_csv_list,
        default=["minimal_set", "all_catalog"],
        help="Comma-separated candidate source modes to compare.",
    )
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--transfer-low-risk-rmse-limit-m", type=float, default=1.3)
    parser.add_argument("--transfer-low-risk-target-limit-m", type=float, default=2.5)
    parser.add_argument("--transfer-risk-target-weight", type=float, default=0.10)
    parser.add_argument("--knn-neighbor-count", type=int, default=5)
    parser.add_argument("--intrinsic-scale-gap-source-scenario", default="intrinsic_scale_stress")
    parser.add_argument("--intrinsic-scale-gap-product-ratio-p10-threshold", type=float, default=1.0037)
    parser.add_argument("--include-target-in-centroids", action="store_true")
    parser.add_argument(
        "--exclude-target-scenario-from-library",
        action="store_true",
        help=(
            "Run a stricter leave-one-scenario-out audit by removing each target scenario from "
            "the runtime feature library and candidate source set."
        ),
    )
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def policy_run_dir(output_dir: Path, candidate_source_mode: str, policy: str) -> Path:
    return output_dir / "runs" / f"{candidate_source_mode}_{policy}"


def score_policy_row(row: Mapping[str, object]) -> float:
    pass_count = finite_float(row.get("pass_count"), 0.0)
    audit_count = finite_float(row.get("audit_count"), 0.0)
    fail_count = max(0.0, audit_count - pass_count)
    corrected_max = finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf"))
    corrected_mean = finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf"))
    target_max = finite_float(row.get("target_max_abs_error_max_m"), float("inf"))
    fallback_count = finite_float(row.get("fallback_count"), 0.0)
    complexity = POLICY_COMPLEXITY_PENALTY.get(str(row.get("selection_policy", "")), 0.20)
    return (
        1000.0 * fail_count
        + corrected_max
        + 0.25 * corrected_mean
        + 0.10 * target_max
        + 0.01 * fallback_count
        + complexity
    )


def audit_row_stats(run_dir: Path) -> dict[str, object]:
    rows = read_audit_rows(run_dir / "runtime_measurement_guard_audit.csv")
    fallback_count = sum(1 for row in rows if str(row.get("fallback_reason", "")).strip())
    selected_pairs: dict[str, int] = {}
    for row in rows:
        key = f"{row.get('preliminary_source_scenario', '')}->{row.get('selected_source_scenario', '')}"
        selected_pairs[key] = selected_pairs.get(key, 0) + 1
    return {
        "fallback_count": fallback_count,
        "selection_pair_counts_json": json.dumps(selected_pairs, ensure_ascii=False, sort_keys=True),
    }


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime measurement guard policy audit",
        "",
        "Purpose:",
        (
            "Compare raw-measurement StereoDistance runtime profile selectors under identical "
            "leave-one-out audit conditions."
        ),
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "selection_contract: raw RDF/disparity for selection; transfer metadata only for transfer-risk policy; truth is audit-only",
        "",
        "Recommendation:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Policy rows:"])
    for row in rows:
        lines.append(
            "{rank}. mode={mode} policy={policy} pass={passes}/{total} "
            "rmse_mean/max={mean}/{maxv} target_max={target_max} fallback={fallback} score={score}".format(
                rank=row.get("policy_selection_rank", ""),
                mode=row.get("candidate_source_mode", ""),
                policy=row.get("selection_policy", ""),
                passes=row.get("pass_count", ""),
                total=row.get("audit_count", ""),
                mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                maxv=row.get("validation_range_rmse_corrected_max_m", ""),
                target_max=row.get("target_max_abs_error_max_m", ""),
                fallback=row.get("fallback_count", ""),
                score=row.get("policy_selection_score", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_policy_scores(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    labels = [f"{row['candidate_source_mode']}\n{row['selection_policy']}" for row in rows]
    pass_counts = np.array([float(row["pass_count"]) for row in rows], dtype=float)
    target_max = np.array([float(row["target_max_abs_error_max_m"]) for row in rows], dtype=float)
    fig, axes = plt.subplots(1, 2, figsize=(max(9, len(rows) * 1.3), 4.8), constrained_layout=True)
    axes[0].bar(np.arange(len(rows)), pass_counts, color="tab:blue")
    axes[0].set_title("Pass count")
    axes[0].set_ylabel("audit passes")
    axes[0].set_ylim(0, max(1.0, np.max(pass_counts) + 1.0))
    axes[1].bar(np.arange(len(rows)), target_max, color="tab:orange")
    axes[1].axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    axes[1].set_title("Worst target max abs error")
    axes[1].set_ylabel("m")
    for axis in axes:
        axis.set_xticks(np.arange(len(rows)), labels, rotation=35, ha="right")
        axis.grid(True, axis="y", alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def audit_policies(args: argparse.Namespace) -> dict[str, object]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    for candidate_source_mode in args.candidate_source_modes:
        for policy in args.policies:
            if policy not in POLICIES:
                raise ValueError(f"unsupported policy: {policy}")
            if candidate_source_mode == "minimal_set" and args.minimal_set_json is None:
                continue
            run_dir = policy_run_dir(args.output_dir, candidate_source_mode, policy)
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
                candidate_source_mode=candidate_source_mode,
                selection_policy=policy,
                transfer_matrix_csv=args.transfer_matrix_csv,
                transfer_low_risk_rmse_limit_m=args.transfer_low_risk_rmse_limit_m,
                transfer_low_risk_target_limit_m=args.transfer_low_risk_target_limit_m,
                transfer_risk_target_weight=args.transfer_risk_target_weight,
                knn_neighbor_count=args.knn_neighbor_count,
                intrinsic_scale_gap_source_scenario=getattr(
                    args,
                    "intrinsic_scale_gap_source_scenario",
                    "intrinsic_scale_stress",
                ),
                intrinsic_scale_gap_product_ratio_p10_threshold=getattr(
                    args,
                    "intrinsic_scale_gap_product_ratio_p10_threshold",
                    1.0037,
                ),
                audit_all_sweep_seeds=True,
                include_target_in_centroids=args.include_target_in_centroids,
                exclude_target_scenario_from_library=bool(
                    getattr(args, "exclude_target_scenario_from_library", False)
                ),
                allow_unvalidated=args.allow_unvalidated,
            )
            row = dict(run_guard(guard_args))
            row["run_dir"] = str(run_dir)
            row["candidate_source_mode"] = candidate_source_mode
            row["selection_policy"] = policy
            row.update(audit_row_stats(run_dir))
            row["policy_selection_score"] = score_policy_row(row)
            rows.append(row)
    rows = sorted(
        rows,
        key=lambda row: (
            finite_float(row.get("policy_selection_score"), float("inf")),
            -finite_float(row.get("pass_count"), 0.0),
            finite_float(row.get("target_max_abs_error_max_m"), float("inf")),
            str(row.get("candidate_source_mode", "")),
            str(row.get("selection_policy", "")),
        ),
    )
    for index, row in enumerate(rows, start=1):
        row["policy_selection_rank"] = index
    best = rows[0] if rows else {}
    summary = {
        "mode": "runtime_measurement_guard_policy_audit",
        "policy_count": len(rows),
        "recommended_candidate_source_mode": best.get("candidate_source_mode", ""),
        "recommended_selection_policy": best.get("selection_policy", ""),
        "recommended_pass_count": best.get("pass_count", ""),
        "recommended_audit_count": best.get("audit_count", ""),
        "recommended_validation_range_rmse_corrected_mean_m": best.get(
            "validation_range_rmse_corrected_mean_m", ""
        ),
        "recommended_validation_range_rmse_corrected_max_m": best.get("validation_range_rmse_corrected_max_m", ""),
        "recommended_target_max_abs_error_max_m": best.get("target_max_abs_error_max_m", ""),
        "recommended_policy_selection_score": best.get("policy_selection_score", ""),
        "recommended_run_dir": best.get("run_dir", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": "raw RDF/disparity for selection; transfer metadata only for transfer-risk policy; truth is audit-only",
    }
    write_csv(args.output_dir / "runtime_measurement_guard_policy_audit.csv", rows)
    write_json(args.output_dir / "runtime_measurement_guard_policy_audit_summary.json", summary)
    write_summary_txt(args.output_dir / "runtime_measurement_guard_policy_audit_summary.txt", summary, rows)
    plot_policy_scores(args.output_dir / "runtime_measurement_guard_policy_audit.png", rows)
    return summary


def main() -> int:
    summary = audit_policies(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
