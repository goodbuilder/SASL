#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Mapping, Sequence


def read_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def as_float(row: Mapping[str, object], name: str) -> float:
    return float(row.get(name, "nan"))


def as_int(row: Mapping[str, object], name: str) -> int:
    return int(float(row.get(name, 0)))


def replay_label(replay_dir: Path, payload: Mapping[str, object]) -> str:
    experiment_id = str(payload.get("experiment_id", ""))
    if experiment_id:
        return experiment_id
    return replay_dir.name


def summarize_replay_dir(replay_dir: Path) -> tuple[dict[str, object], list[dict[str, object]]]:
    metrics_path = replay_dir / "fresh_seed_replay_metrics.json"
    scenario_path = replay_dir / "fresh_seed_replay_scenario_summary.csv"
    if not metrics_path.is_file():
        raise FileNotFoundError(f"missing fresh replay metrics: {metrics_path}")
    if not scenario_path.is_file():
        raise FileNotFoundError(f"missing fresh replay scenario summary: {scenario_path}")

    metrics = read_json(metrics_path)
    label = replay_label(replay_dir, metrics)
    row = {
        "replay_label": label,
        "replay_dir": str(replay_dir),
        "profile_json": metrics.get("profile_json", ""),
        "profile_id": metrics.get("profile_id", ""),
        "status": metrics.get("status", ""),
        "pass_count": metrics.get("pass_count", 0),
        "trial_count": metrics.get("trial_count", 0),
        "pass_rate": (
            as_float(metrics, "pass_count") / as_float(metrics, "trial_count")
            if as_float(metrics, "trial_count") > 0.0
            else 0.0
        ),
        "range_rmse_raw_mean_m": metrics.get("range_rmse_raw_mean_m", ""),
        "range_rmse_corrected_mean_m": metrics.get("range_rmse_corrected_mean_m", ""),
        "range_rmse_corrected_max_m": metrics.get("range_rmse_corrected_max_m", ""),
        "position_rmse_raw_mean_m": metrics.get("position_rmse_raw_mean_m", ""),
        "position_rmse_corrected_mean_m": metrics.get("position_rmse_corrected_mean_m", ""),
        "position_rmse_corrected_max_m": metrics.get("position_rmse_corrected_max_m", ""),
        "target_max_abs_error_corrected_max_m": metrics.get("target_max_abs_error_corrected_max_m", ""),
        "target_rmse_corrected_mean_m": metrics.get("target_rmse_corrected_mean_m", ""),
        "invalid_count_total": metrics.get("invalid_count_total", 0),
        "fallback_count_total": metrics.get("fallback_count_total", 0),
        "runtime_truth_used": metrics.get("runtime_truth_used", ""),
        "rdf_axis_contract": metrics.get("rdf_axis_contract", ""),
        "public_topic_contract": metrics.get("public_topic_contract", ""),
    }
    scenario_rows = []
    for scenario_row in read_csv(scenario_path):
        scenario_rows.append(
            {
                "replay_label": label,
                "profile_id": row["profile_id"],
                **scenario_row,
            }
        )
    return row, scenario_rows


def rank_key(row: Mapping[str, object]) -> tuple[float, float, float, float, float]:
    strict_penalty = 0.0 if str(row.get("status", "")) == "STRICT_PASS" else 1.0
    return (
        strict_penalty,
        -float(row.get("pass_rate", 0.0)),
        as_float(row, "target_max_abs_error_corrected_max_m"),
        as_float(row, "range_rmse_corrected_mean_m"),
        as_float(row, "position_rmse_corrected_mean_m"),
    )


def build_summary(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    if not rows:
        return {
            "status": "FAIL",
            "profile_count": 0,
            "recommendation": "no_fresh_seed_replays_found",
            "runtime_truth_used": False,
        }
    sorted_rows = sorted(rows, key=rank_key)
    best = sorted_rows[0]
    strict_pass_count = sum(1 for row in rows if str(row.get("status", "")) == "STRICT_PASS")
    all_truth_free = all(str(row.get("runtime_truth_used", "")).lower() == "false" for row in rows)
    all_target_pass = all(as_float(row, "target_max_abs_error_corrected_max_m") <= 10.0 for row in rows)
    status = "STRICT_PASS" if strict_pass_count == len(rows) and all_truth_free else "FAIL"
    if status == "STRICT_PASS":
        recommendation = "all_profiles_strict_pass_select_best_ranked_candidate"
    elif all_target_pass and all_truth_free:
        recommendation = "no_strict_unanimous_pass_but_all_profiles_meet_target_tail_gate"
    else:
        recommendation = "fresh_seed_generalization_gap_continue_search"
    return {
        "status": status,
        "profile_count": len(rows),
        "strict_pass_count": strict_pass_count,
        "all_target_max_within_10m": all_target_pass,
        "runtime_truth_used": not all_truth_free,
        "recommended_replay_label": best.get("replay_label", ""),
        "recommended_profile_id": best.get("profile_id", ""),
        "recommended_status": best.get("status", ""),
        "recommended_pass_rate": best.get("pass_rate", 0.0),
        "recommended_range_rmse_corrected_mean_m": best.get("range_rmse_corrected_mean_m", ""),
        "recommended_position_rmse_corrected_mean_m": best.get("position_rmse_corrected_mean_m", ""),
        "recommended_target_max_abs_error_corrected_max_m": best.get("target_max_abs_error_corrected_max_m", ""),
        "recommendation": recommendation,
    }


def write_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "Fresh seed runtime profile replay summary",
        "",
        f"status: {summary.get('status', '')}",
        f"profile_count: {summary.get('profile_count', '')}",
        f"strict_pass_count: {summary.get('strict_pass_count', '')}",
        f"all_target_max_within_10m: {summary.get('all_target_max_within_10m', '')}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', '')}",
        f"recommendation: {summary.get('recommendation', '')}",
        f"recommended_replay_label: {summary.get('recommended_replay_label', '')}",
        f"recommended_profile_id: {summary.get('recommended_profile_id', '')}",
        f"recommended_pass_rate: {summary.get('recommended_pass_rate', '')}",
        f"recommended_target_max_abs_error_corrected_max_m: {summary.get('recommended_target_max_abs_error_corrected_max_m', '')}",
        "",
        "Rows:",
    ]
    for row in sorted(rows, key=rank_key):
        lines.append(
            (
                "- {label}: status={status}, pass={passed}/{total}, range_mean={range_mean:.6f}m, "
                "pos_mean={pos_mean:.6f}m, target_max={target_max:.6f}m"
            ).format(
                label=row.get("replay_label", ""),
                status=row.get("status", ""),
                passed=as_int(row, "pass_count"),
                total=as_int(row, "trial_count"),
                range_mean=as_float(row, "range_rmse_corrected_mean_m"),
                pos_mean=as_float(row, "position_rmse_corrected_mean_m"),
                target_max=as_float(row, "target_max_abs_error_corrected_max_m"),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize multiple fresh-seed runtime profile replay outputs.")
    parser.add_argument("--replay-dir", type=Path, action="append", default=[])
    parser.add_argument("--replay-root", type=Path, action="append", default=[])
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def discover_replay_dirs(args: argparse.Namespace) -> list[Path]:
    replay_dirs = [path for path in args.replay_dir]
    for root in args.replay_root:
        if not root.is_dir():
            continue
        for child in sorted(root.iterdir()):
            if child.is_dir() and (child / "fresh_seed_replay_metrics.json").is_file():
                replay_dirs.append(child)
    return list(dict.fromkeys(replay_dirs))


def main() -> int:
    args = parse_args()
    replay_rows: list[dict[str, object]] = []
    scenario_rows: list[dict[str, object]] = []
    for replay_dir in discover_replay_dirs(args):
        row, scenarios = summarize_replay_dir(replay_dir)
        replay_rows.append(row)
        scenario_rows.extend(scenarios)
    summary = build_summary(replay_rows)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "fresh_seed_profile_replay_summary.csv", replay_rows)
    write_csv(args.output_dir / "fresh_seed_profile_replay_scenario_summary.csv", scenario_rows)
    write_json(args.output_dir / "fresh_seed_profile_replay_recommendation.json", summary)
    write_summary_txt(args.output_dir / "fresh_seed_profile_replay_recommendation.txt", summary, replay_rows)
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
