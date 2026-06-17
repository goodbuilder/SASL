#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable, Sequence


RDF_AXES = ("right", "down", "forward")
AXES = ("right", "down", "forward", "position")
ALL_SOURCES = ("anchor", "center", "anchor_rd_center_f", "center_rd_anchor_f")
STRICT_EPSILON_M = 1.0e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Search runtime-available bbox-anchor / center-rotation axis-gain "
            "corrections across multiple simulation report families under the "
            "Strict RDF all-axis gate."
        )
    )
    parser.add_argument("report_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--sources", default="anchor,center,anchor_rd_center_f,center_rd_anchor_f")
    parser.add_argument("--right-min", type=float, default=0.0)
    parser.add_argument("--right-max", type=float, default=1.0)
    parser.add_argument("--right-step", type=float, default=0.05)
    parser.add_argument("--down-min", type=float, default=0.0)
    parser.add_argument("--down-max", type=float, default=1.0)
    parser.add_argument("--down-step", type=float, default=0.05)
    parser.add_argument("--forward-min", type=float, default=0.0)
    parser.add_argument("--forward-max", type=float, default=1.0)
    parser.add_argument("--forward-step", type=float, default=0.05)
    parser.add_argument("--strict-epsilon-m", type=float, default=STRICT_EPSILON_M)
    parser.add_argument("--top-k", type=int, default=100)
    return parser.parse_args()


def parse_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_ints(text: str) -> list[int]:
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_sources(text: str) -> list[str]:
    sources = [part.strip() for part in str(text).split(",") if part.strip()]
    invalid = [source for source in sources if source not in ALL_SOURCES]
    if invalid:
        raise ValueError(f"unsupported sources: {', '.join(invalid)}")
    return sources


def finite_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def frange(start: float, stop: float, step: float) -> list[float]:
    if step <= 0.0:
        raise ValueError("step must be > 0")
    if start > stop:
        raise ValueError("range start must be <= stop")
    values: list[float] = []
    current = start
    epsilon = abs(step) * 1.0e-9
    while current <= stop + epsilon:
        values.append(round(current, 12))
        current += step
    return values


def delta_kind(source: str, axis: str) -> str:
    if source == "anchor":
        return "anchor"
    if source == "center":
        return "center"
    if source == "anchor_rd_center_f":
        return "center" if axis == "forward" else "anchor"
    if source == "center_rd_anchor_f":
        return "anchor" if axis == "forward" else "center"
    raise ValueError(source)


def load_trial_stats(report_dir: Path, trial: int) -> dict[str, object]:
    path = report_dir / f"validation_trial_{trial:02d}_anchor_comparison.csv"
    if not path.exists():
        raise FileNotFoundError(path)
    stats: dict[str, object] = {
        "report_dir": report_dir.name,
        "trial": trial,
        "sample_count": 0,
        "raw_sse": {axis: 0.0 for axis in RDF_AXES},
        "raw_position_sse": 0.0,
        "delta_cross": {name: {axis: 0.0 for axis in RDF_AXES} for name in ("anchor", "center")},
        "delta_sse": {name: {axis: 0.0 for axis in RDF_AXES} for name in ("anchor", "center")},
    }
    for row in parse_csv(path):
        truth = {axis: finite_float(row.get(f"truth_{axis}_m")) for axis in RDF_AXES}
        raw = {axis: finite_float(row.get(f"raw_{axis}_m")) for axis in RDF_AXES}
        anchor = {axis: finite_float(row.get(f"anchor_{axis}_m")) for axis in RDF_AXES}
        center = {axis: finite_float(row.get(f"center_rot_{axis}_m")) for axis in RDF_AXES}
        if any(
            value is None
            for values in (truth.values(), raw.values(), anchor.values(), center.values())
            for value in values
        ):
            continue
        stats["sample_count"] = int(stats["sample_count"]) + 1
        raw_position_sse = 0.0
        raw_sse = stats["raw_sse"]
        delta_cross = stats["delta_cross"]
        delta_sse = stats["delta_sse"]
        for axis in RDF_AXES:
            raw_error = float(raw[axis]) - float(truth[axis])
            anchor_delta = float(anchor[axis]) - float(raw[axis])
            center_delta = float(center[axis]) - float(raw[axis])
            raw_sse[axis] += raw_error * raw_error
            delta_cross["anchor"][axis] += raw_error * anchor_delta
            delta_sse["anchor"][axis] += anchor_delta * anchor_delta
            delta_cross["center"][axis] += raw_error * center_delta
            delta_sse["center"][axis] += center_delta * center_delta
            raw_position_sse += raw_error * raw_error
        stats["raw_position_sse"] = float(stats["raw_position_sse"]) + raw_position_sse
    return stats


def load_trials(report_dirs: Sequence[Path], trials: Sequence[int]) -> list[dict[str, object]]:
    return [load_trial_stats(report_dir, trial) for report_dir in report_dirs for trial in trials]


def rmse_from_sse(sse: float, sample_count: int) -> float:
    return math.sqrt(sse / float(sample_count))


def evaluate_candidate(
    trial_stats: Sequence[dict[str, object]],
    *,
    source: str,
    right_gain: float,
    down_gain: float,
    forward_gain: float,
    strict_epsilon_m: float,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    gains = {"right": right_gain, "down": down_gain, "forward": forward_gain}
    pass_count = 0
    per_trial_rows: list[dict[str, object]] = []
    min_values = {axis: float("inf") for axis in AXES}
    mean_values = {axis: 0.0 for axis in AXES}
    total_sample_count = 0
    for stats in trial_stats:
        sample_count = int(stats["sample_count"])
        total_sample_count += sample_count
        raw_sse = stats["raw_sse"]
        delta_cross = stats["delta_cross"]
        delta_sse = stats["delta_sse"]
        per_axis_improvement: dict[str, float] = {}
        strict_pass = True
        for axis in RDF_AXES:
            gain = gains[axis]
            kind = delta_kind(source, axis)
            corrected_sse = (
                float(raw_sse[axis])
                + 2.0 * gain * float(delta_cross[kind][axis])
                + gain * gain * float(delta_sse[kind][axis])
            )
            improvement = rmse_from_sse(float(raw_sse[axis]), sample_count) - rmse_from_sse(
                corrected_sse, sample_count
            )
            per_axis_improvement[axis] = improvement
            strict_pass = strict_pass and improvement > strict_epsilon_m
        corrected_position_sse = float(stats["raw_position_sse"])
        for axis in RDF_AXES:
            gain = gains[axis]
            kind = delta_kind(source, axis)
            corrected_position_sse += (
                2.0 * gain * float(delta_cross[kind][axis]) + gain * gain * float(delta_sse[kind][axis])
            )
        position_improvement = rmse_from_sse(float(stats["raw_position_sse"]), sample_count) - rmse_from_sse(
            corrected_position_sse, sample_count
        )
        strict_pass = strict_pass and position_improvement > strict_epsilon_m
        if strict_pass:
            pass_count += 1
        row = {
            "report_dir": str(stats["report_dir"]),
            "trial": int(stats["trial"]),
            "sample_count": sample_count,
            "pass_strict_rdf_all_axis": "yes" if strict_pass else "no",
            "right_rmse_improvement_m": f"{per_axis_improvement['right']:.9f}",
            "down_rmse_improvement_m": f"{per_axis_improvement['down']:.9f}",
            "forward_rmse_improvement_m": f"{per_axis_improvement['forward']:.9f}",
            "position_rmse_improvement_m": f"{position_improvement:.9f}",
        }
        per_trial_rows.append(row)
        for axis in RDF_AXES:
            value = per_axis_improvement[axis]
            min_values[axis] = min(min_values[axis], value)
            mean_values[axis] += value
        min_values["position"] = min(min_values["position"], position_improvement)
        mean_values["position"] += position_improvement
    total_trials = len(trial_stats)
    for axis in AXES:
        mean_values[axis] /= float(total_trials)
    summary = {
        "source": source,
        "right_gain": f"{right_gain:.12f}",
        "down_gain": f"{down_gain:.12f}",
        "forward_gain": f"{forward_gain:.12f}",
        "report_count": len({str(stats["report_dir"]) for stats in trial_stats}),
        "trial_count": total_trials,
        "sample_count": total_sample_count,
        "pass_trial_count": pass_count,
        "strict_pass_all_trials": "yes" if pass_count == total_trials else "no",
        "right_min_delta_m": f"{min_values['right']:.9f}",
        "right_mean_delta_m": f"{mean_values['right']:.9f}",
        "down_min_delta_m": f"{min_values['down']:.9f}",
        "down_mean_delta_m": f"{mean_values['down']:.9f}",
        "forward_min_delta_m": f"{min_values['forward']:.9f}",
        "forward_mean_delta_m": f"{mean_values['forward']:.9f}",
        "position_min_delta_m": f"{min_values['position']:.9f}",
        "position_mean_delta_m": f"{mean_values['position']:.9f}",
    }
    return summary, per_trial_rows


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def sort_key(row: dict[str, object]) -> tuple[object, ...]:
    return (
        int(row["pass_trial_count"]),
        float(row["right_min_delta_m"]),
        float(row["down_min_delta_m"]),
        float(row["forward_min_delta_m"]),
        float(row["position_min_delta_m"]),
        float(row["position_mean_delta_m"]),
    )


def main() -> int:
    args = parse_args()
    trials = parse_ints(args.validation_trials)
    sources = parse_sources(args.sources)
    report_dirs = [path.resolve() for path in args.report_dirs]
    trial_stats = load_trials(report_dirs, trials)
    right_values = frange(args.right_min, args.right_max, args.right_step)
    down_values = frange(args.down_min, args.down_max, args.down_step)
    forward_values = frange(args.forward_min, args.forward_max, args.forward_step)

    summaries: list[dict[str, object]] = []
    per_source_best: dict[str, dict[str, object]] = {}
    best_summary: dict[str, object] | None = None
    best_failures: list[dict[str, object]] = []
    for source in sources:
        for right_gain in right_values:
            for down_gain in down_values:
                for forward_gain in forward_values:
                    summary, per_trial_rows = evaluate_candidate(
                        trial_stats,
                        source=source,
                        right_gain=right_gain,
                        down_gain=down_gain,
                        forward_gain=forward_gain,
                        strict_epsilon_m=args.strict_epsilon_m,
                    )
                    summaries.append(summary)
                    if source not in per_source_best or sort_key(summary) > sort_key(per_source_best[source]):
                        per_source_best[source] = summary
                    if best_summary is None or sort_key(summary) > sort_key(best_summary):
                        best_summary = summary
                        best_failures = [row for row in per_trial_rows if row["pass_strict_rdf_all_axis"] != "yes"]

    summaries.sort(key=sort_key, reverse=True)
    strict_full_rows = [row for row in summaries if row["strict_pass_all_trials"] == "yes"]
    top_rows = summaries[: max(0, int(args.top_k))]
    metadata = {
        "report_dirs": [str(path) for path in report_dirs],
        "validation_trials": trials,
        "sources": sources,
        "strict_epsilon_m": args.strict_epsilon_m,
        "right_range": [args.right_min, args.right_max, args.right_step],
        "down_range": [args.down_min, args.down_max, args.down_step],
        "forward_range": [args.forward_min, args.forward_max, args.forward_step],
        "trial_count": len(trial_stats),
        "combo_count": len(summaries),
        "strict_full_count": len(strict_full_rows),
        "best_candidate": best_summary,
        "best_candidate_failure_count": len(best_failures),
        "per_source_best": per_source_best,
    }

    output_dir = args.output_dir
    write_csv(output_dir / "candidate_summary.csv", top_rows)
    write_csv(output_dir / "full_strict_candidates.csv", strict_full_rows)
    write_csv(output_dir / "best_candidate_failures.csv", best_failures)
    write_json(output_dir / "search_metadata.json", metadata)

    print(f"trial_count={len(trial_stats)}")
    print(f"combo_count={len(summaries)}")
    print(f"strict_full_count={len(strict_full_rows)}")
    if best_summary is not None:
        print(
            "best_candidate="
            f"{best_summary['source']} "
            f"R={best_summary['right_gain']} "
            f"D={best_summary['down_gain']} "
            f"F={best_summary['forward_gain']} "
            f"strict={best_summary['pass_trial_count']}/{best_summary['trial_count']} "
            f"mins=({best_summary['right_min_delta_m']},"
            f"{best_summary['down_min_delta_m']},"
            f"{best_summary['forward_min_delta_m']},"
            f"{best_summary['position_min_delta_m']})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
