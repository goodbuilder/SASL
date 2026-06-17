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

from audit_stereodistance_active_profile_transfer import audit_active_profile_transfer  # noqa: E402
from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from stereodistance_profile_replay import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare multiple exported StereoDistance active runtime profiles as fixed universal "
            "profiles across a robustness sweep. Profile application consumes raw stereo-center RDF "
            "and disparity only; truth is used only for offline replay scoring."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--profile",
        action="append",
        default=[],
        help="Profile candidate as LABEL=PATH or PATH. Repeat for every active runtime profile candidate.",
    )
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def parse_profile_arg(value: str) -> tuple[str, Path]:
    if "=" in value:
        label, path = value.split("=", 1)
        label = label.strip()
        profile_path = Path(path.strip())
    else:
        profile_path = Path(value.strip())
        label = profile_path.stem
    if not label:
        label = profile_path.stem
    if not str(profile_path):
        raise ValueError("profile path must not be empty")
    return label, profile_path


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _float_sort_value(row: Mapping[str, object], key: str, *, default: float) -> float:
    value = finite_float(row.get(key), default)
    return value if math.isfinite(value) else default


def rank_summaries(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    ranked = sorted(
        rows,
        key=lambda row: (
            not bool(row.get("pass_all_transfer_replays")),
            -int(float(row.get("transfer_replay_pass_count", 0) or 0)),
            _float_sort_value(row, "target_max_abs_error_max_m", default=float("inf")),
            _float_sort_value(row, "validation_range_rmse_corrected_max_m", default=float("inf")),
            _float_sort_value(row, "validation_range_rmse_corrected_mean_m", default=float("inf")),
            str(row.get("profile_label", "")),
        ),
    )
    output: list[dict[str, object]] = []
    for rank, row in enumerate(ranked, start=1):
        enriched = dict(row)
        enriched["portfolio_rank"] = rank
        output.append(enriched)
    return output


def recommendation_from_ranked(ranked_rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    if not ranked_rows:
        return {
            "selected": False,
            "reason": "no_profile_candidates",
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "runtime_input_contract": "profile application uses raw stereo-center RDF/disparity only; truth is audit-only",
        }
    best = ranked_rows[0]
    selected = bool(best.get("pass_all_transfer_replays"))
    return {
        "selected": selected,
        "recommended_profile_label": best.get("profile_label", ""),
        "recommended_profile_json": best.get("profile_json", ""),
        "reason": (
            "passes_all_transfer_replays_with_lowest_target_worst_case_then_validation_rmse"
            if selected
            else "no_candidate_passed_all_transfer_replays; listed profile has best partial score"
        ),
        "transfer_replay_pass_count": best.get("transfer_replay_pass_count", 0),
        "transfer_replay_count": best.get("transfer_replay_count", 0),
        "target_max_abs_error_max_m": best.get("target_max_abs_error_max_m", ""),
        "validation_range_rmse_corrected_max_m": best.get("validation_range_rmse_corrected_max_m", ""),
        "validation_range_rmse_corrected_mean_m": best.get("validation_range_rmse_corrected_mean_m", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "profile application uses raw stereo-center RDF/disparity only; truth is audit-only",
    }


def write_summary_txt(
    path: Path,
    *,
    recommendation: Mapping[str, object],
    ranked_rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance active profile portfolio audit",
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "runtime_input_contract: profile application uses raw stereo-center RDF/disparity only; truth is audit-only",
        "",
        "Recommendation:",
    ]
    for key in (
        "selected",
        "recommended_profile_label",
        "recommended_profile_json",
        "reason",
        "transfer_replay_pass_count",
        "transfer_replay_count",
        "target_max_abs_error_max_m",
        "validation_range_rmse_corrected_max_m",
        "validation_range_rmse_corrected_mean_m",
    ):
        value = recommendation.get(key, "")
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Ranked profiles:"])
    for row in ranked_rows:
        lines.append(
            "rank={rank} label={label} pass={passed}/{total} pass_all={pass_all} "
            "target_max={target_max} rmse_max={rmse_max} rmse_mean={rmse_mean}".format(
                rank=row.get("portfolio_rank", ""),
                label=row.get("profile_label", ""),
                passed=row.get("transfer_replay_pass_count", ""),
                total=row.get("transfer_replay_count", ""),
                pass_all=row.get("pass_all_transfer_replays", ""),
                target_max=row.get("target_max_abs_error_max_m", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                rmse_mean=row.get("validation_range_rmse_corrected_mean_m", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_portfolio(path: Path, ranked_rows: Sequence[Mapping[str, object]]) -> None:
    if not ranked_rows:
        return
    labels = [str(row.get("profile_label", "")) for row in ranked_rows]
    pass_counts = np.asarray([finite_float(row.get("transfer_replay_pass_count"), float("nan")) for row in ranked_rows])
    totals = np.asarray([finite_float(row.get("transfer_replay_count"), float("nan")) for row in ranked_rows])
    target_max = np.asarray([finite_float(row.get("target_max_abs_error_max_m"), float("nan")) for row in ranked_rows])
    rmse_max = np.asarray([finite_float(row.get("validation_range_rmse_corrected_max_m"), float("nan")) for row in ranked_rows])
    x = np.arange(len(ranked_rows))
    fig, axes = plt.subplots(3, 1, figsize=(max(8, len(ranked_rows) * 1.1), 8), sharex=True, constrained_layout=True)
    axes[0].bar(x, pass_counts, color="tab:green")
    axes[0].plot(x, totals, color="black", marker="o", linewidth=1)
    axes[0].set_ylabel("pass count")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(x, target_max, color="tab:orange")
    axes[1].axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    axes[1].set_ylabel("target max abs (m)")
    axes[1].grid(True, axis="y", alpha=0.3)
    axes[2].bar(x, rmse_max, color="tab:blue")
    axes[2].set_ylabel("RMSE max (m)")
    axes[2].set_xticks(x, labels, rotation=45, ha="right")
    axes[2].grid(True, axis="y", alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def audit_active_profile_portfolio(
    *,
    sweep_dir: Path,
    profile_candidates: Sequence[tuple[str, Path]],
    output_dir: Path,
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_rows: list[dict[str, object]] = []
    run_rows: list[dict[str, object]] = []
    seen_labels: set[str] = set()
    for label, profile_json in profile_candidates:
        if label in seen_labels:
            raise ValueError(f"duplicate profile label: {label}")
        seen_labels.add(label)
        profile_output_dir = output_dir / "per_profile_transfer_replay" / label
        summary = audit_active_profile_transfer(
            sweep_dir=sweep_dir,
            profile_json=profile_json,
            output_dir=profile_output_dir,
            split=split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
            target_error_limit_m=target_error_limit_m,
        )
        summary_row = dict(summary)
        summary_row["profile_label"] = label
        summary_rows.append(summary_row)
        for row in read_csv(profile_output_dir / "active_profile_transfer_replay.csv"):
            row_out = dict(row)
            row_out["profile_label"] = label
            run_rows.append(row_out)
    ranked = rank_summaries(summary_rows)
    recommendation = recommendation_from_ranked(ranked)
    write_csv(output_dir / "active_profile_portfolio_runs.csv", run_rows)
    write_csv(output_dir / "active_profile_portfolio_summary.csv", ranked)
    write_json(output_dir / "active_profile_portfolio_recommendation.json", recommendation)
    write_summary_txt(
        output_dir / "active_profile_portfolio_summary.txt",
        recommendation=recommendation,
        ranked_rows=ranked,
    )
    plot_portfolio(output_dir / "active_profile_portfolio_summary.png", ranked)
    return {
        "mode": "active_profile_portfolio_audit",
        "profile_count": len(profile_candidates),
        "recommendation": recommendation,
        "summary_rows": ranked,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "profile application uses raw stereo-center RDF/disparity only; truth is audit-only",
    }


def main() -> int:
    args = parse_args()
    profile_candidates = [parse_profile_arg(value) for value in args.profile]
    if not profile_candidates:
        raise SystemExit("--profile is required at least once")
    result = audit_active_profile_portfolio(
        sweep_dir=args.sweep_dir,
        profile_candidates=profile_candidates,
        output_dir=args.output_dir,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(result["recommendation"], ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result["recommendation"].get("selected") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
