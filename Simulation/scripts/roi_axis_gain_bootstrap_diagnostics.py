#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import random
from pathlib import Path
from typing import Sequence


AXES = ("right", "down", "forward", "position")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Bootstrap RMSE-improvement diagnostics for a fixed ROI axis-gain "
            "candidate diagnostics CSV. Positive improvement means raw RMSE is "
            "larger than corrected RMSE."
        )
    )
    parser.add_argument("sample_diagnostics_csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20260609)
    parser.add_argument("--groups", default="all,trial,range_bin,screen_bin")
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--min-valid-roi", type=int, default=1)
    return parser.parse_args()


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def quantile(values: Sequence[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, q)) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def improvement(rows: Sequence[dict[str, str]], axis: str) -> float:
    raw = [float(row[f"raw_error_{axis}_m"]) for row in rows]
    corrected = [float(row[f"corrected_error_{axis}_m"]) for row in rows]
    return rmse(raw) - rmse(corrected)


def group_key(row: dict[str, str], group: str) -> tuple[str, ...]:
    if group == "all":
        return ("all",)
    if group == "trial":
        return (row["trial"],)
    if group == "range_bin":
        return (row["range_bin"],)
    if group == "screen_bin":
        return (row["screen_u_bin"], row["screen_v_bin"])
    raise ValueError(f"unsupported group: {group}")


def group_label(group: str, key: tuple[str, ...]) -> dict[str, str]:
    if group == "all":
        return {"group": "all"}
    if group == "trial":
        return {"group": "trial", "trial": key[0]}
    if group == "range_bin":
        return {"group": "range_bin", "range_bin": key[0]}
    if group == "screen_bin":
        return {"group": "screen_bin", "screen_u_bin": key[0], "screen_v_bin": key[1]}
    raise ValueError(f"unsupported group: {group}")


def sample_rows(rng: random.Random, rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    return [rows[rng.randrange(len(rows))] for _ in range(len(rows))]


def sample_trial_clusters(rng: random.Random, rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    by_trial: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_trial.setdefault(row["trial"], []).append(row)
    trials = sorted(by_trial)
    sampled: list[dict[str, str]] = []
    for _ in trials:
        trial = trials[rng.randrange(len(trials))]
        sampled.extend(by_trial[trial])
    return sampled


def bootstrap_summary(
    rows: Sequence[dict[str, str]],
    *,
    rng: random.Random,
    iterations: int,
    resample_mode: str,
) -> dict[str, object]:
    if resample_mode == "rows":
        sampler = sample_rows
    elif resample_mode == "trials":
        sampler = sample_trial_clusters
    else:
        raise ValueError(f"unsupported resample mode: {resample_mode}")
    axis_values = {axis: [] for axis in AXES}
    pass_count = 0
    for _ in range(max(1, iterations)):
        sampled = sampler(rng, rows)
        sample_improvements = {axis: improvement(sampled, axis) for axis in AXES}
        for axis, value in sample_improvements.items():
            axis_values[axis].append(value)
        if all(sample_improvements[axis] >= 0.0 for axis in ("right", "down", "forward")):
            pass_count += 1
    result: dict[str, object] = {
        "resample_mode": resample_mode,
        "bootstrap_iterations": iterations,
        "bootstrap_pass_fraction": f"{pass_count / max(1, iterations):.9f}",
    }
    for axis in AXES:
        values = axis_values[axis]
        result[f"{axis}_p025_m"] = f"{quantile(values, 0.025):.9f}"
        result[f"{axis}_p500_m"] = f"{quantile(values, 0.500):.9f}"
        result[f"{axis}_p975_m"] = f"{quantile(values, 0.975):.9f}"
        result[f"{axis}_negative_fraction"] = f"{sum(value < 0.0 for value in values) / len(values):.9f}"
    return result


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    rows = load_rows(args.sample_diagnostics_csv)
    rng = random.Random(args.seed)
    group_names = [part.strip() for part in str(args.groups).split(",") if part.strip()]
    grouped: dict[tuple[str, tuple[str, ...]], list[dict[str, str]]] = {}
    for group in group_names:
        for row in rows:
            key = group_key(row, group)
            grouped.setdefault((group, key), []).append(row)
    output_rows: list[dict[str, object]] = []
    for (group, key), group_rows in sorted(grouped.items(), key=lambda item: (item[0][0], item[0][1])):
        valid_count = sum(row["fallback"] == "none" for row in group_rows)
        if len(group_rows) < int(args.min_samples) or valid_count < int(args.min_valid_roi):
            continue
        base = group_label(group, key)
        base.update(
            {
                "sample_count": len(group_rows),
                "valid_roi_count": valid_count,
                "base_pass_rdf_nonregression": (
                    "yes" if all(improvement(group_rows, axis) >= 0.0 for axis in ("right", "down", "forward")) else "no"
                ),
            }
        )
        for axis in AXES:
            base[f"base_{axis}_rmse_improvement_m"] = f"{improvement(group_rows, axis):.9f}"
        for mode in ("rows", "trials"):
            if mode == "trials" and group != "all":
                continue
            summary = dict(base)
            summary.update(bootstrap_summary(group_rows, rng=rng, iterations=int(args.iterations), resample_mode=mode))
            output_rows.append(summary)
    write_csv(args.output_dir / "bootstrap_summary.csv", output_rows)
    print(f"Summary: {args.output_dir / 'bootstrap_summary.csv'}")
    if output_rows:
        first = output_rows[0]
        print(
            "First row: "
            f"group={first['group']} mode={first['resample_mode']} "
            f"pass_fraction={first['bootstrap_pass_fraction']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
