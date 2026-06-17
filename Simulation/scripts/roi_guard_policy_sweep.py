#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import itertools
import math
import random
from pathlib import Path
from typing import Iterable, Sequence


AXES = ("right", "down", "forward", "position")
RDF_AXES = ("right", "down", "forward")
NONREGRESSION_EPS = 1.0e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep runtime-available no-op guard policies over an existing "
            "unguarded ROI/range fusion candidate_sample_diagnostics.csv. "
            "Guards may use raw range and screen bins only."
        )
    )
    parser.add_argument("sample_diagnostics_csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--forward-min-ranges-m", default="none,300,500,650,800")
    parser.add_argument("--screen-bin-options", default="observed")
    parser.add_argument("--max-disabled-screen-bins", type=int, default=1)
    parser.add_argument("--bootstrap-iterations", type=int, default=300)
    parser.add_argument("--bootstrap-seed", type=int, default=20260609)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--min-valid-roi", type=int, default=1)
    parser.add_argument("--write-best-samples", action="store_true")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_forward_min_ranges(text: str) -> list[float | None]:
    values: list[float | None] = []
    for part in str(text).split(","):
        item = part.strip()
        if not item:
            continue
        values.append(None if item.lower() in {"none", "no", "off"} else float(item))
    return values


def parse_screen_bins(text: str, rows: Sequence[dict[str, str]]) -> list[tuple[int, int]]:
    if text.strip() == "observed":
        bins = {
            (int(row["screen_u_bin"]), int(row["screen_v_bin"]))
            for row in rows
            if row.get("screen_u_bin", "") != "" and row.get("screen_v_bin", "") != ""
        }
        return sorted(bins)
    result: list[tuple[int, int]] = []
    for part in str(text).split(","):
        item = part.strip()
        if not item:
            continue
        u_text, v_text = item.split(":", 1)
        result.append((int(u_text), int(v_text)))
    return sorted(set(result))


def screen_set_label(items: Sequence[tuple[int, int]]) -> str:
    return ",".join(f"{u}:{v}" for u, v in sorted(items))


def guard_candidates(
    rows: Sequence[dict[str, str]],
    *,
    forward_min_ranges: Sequence[float | None],
    screen_bins: Sequence[tuple[int, int]],
    max_disabled_screen_bins: int,
) -> list[tuple[float | None, tuple[tuple[int, int], ...]]]:
    disabled_sets: list[tuple[tuple[int, int], ...]] = [tuple()]
    for size in range(1, max(0, int(max_disabled_screen_bins)) + 1):
        disabled_sets.extend(tuple(combo) for combo in itertools.combinations(screen_bins, size))
    return [(forward_min, disabled) for forward_min in forward_min_ranges for disabled in disabled_sets]


def apply_guard(
    rows: Sequence[dict[str, str]],
    *,
    forward_min_range: float | None,
    disabled_screen_bins: Sequence[tuple[int, int]],
) -> list[dict[str, str]]:
    disabled = set(disabled_screen_bins)
    guarded_rows: list[dict[str, str]] = []
    for row in rows:
        guarded = dict(row)
        raw_forward = float(row["raw_error_forward_m"])
        corrected_forward = float(row["corrected_error_forward_m"])
        raw_down = float(row["raw_error_down_m"])
        corrected_down = float(row["corrected_error_down_m"])
        raw_range = float(row["raw_range_m"])
        screen_key = (int(row["screen_u_bin"]), int(row["screen_v_bin"]))
        forward_guarded = forward_min_range is not None and raw_range < float(forward_min_range)
        down_guarded = screen_key in disabled
        if forward_guarded:
            corrected_forward = raw_forward
        if down_guarded:
            corrected_down = raw_down
        guarded["corrected_error_down_m"] = f"{corrected_down:.12g}"
        guarded["corrected_error_forward_m"] = f"{corrected_forward:.12g}"
        corrected_right = float(row["corrected_error_right_m"])
        guarded["corrected_error_position_m"] = f"{math.sqrt(corrected_right**2 + corrected_down**2 + corrected_forward**2):.12g}"
        guarded["forward_fallback"] = "range_guarded_noop" if forward_guarded else row.get("forward_fallback", "none")
        guarded["down_guarded_noop"] = "yes" if down_guarded else "no"
        guarded_rows.append(guarded)
    return guarded_rows


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


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


def group_label(group: str, key: tuple[str, ...]) -> str:
    if group == "all":
        return "all"
    if group == "trial":
        return f"trial={key[0]}"
    if group == "range_bin":
        return f"range_bin={key[0]}"
    if group == "screen_bin":
        return f"screen_bin={key[0]}:{key[1]}"
    raise ValueError(f"unsupported group: {group}")


def grouped_rows(rows: Sequence[dict[str, str]], groups: Sequence[str]) -> dict[str, list[dict[str, str]]]:
    result: dict[str, list[dict[str, str]]] = {}
    for group in groups:
        for row in rows:
            label = group_label(group, group_key(row, group))
            result.setdefault(label, []).append(row)
    return result


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


def sample_rows(rng: random.Random, rows: Sequence[dict[str, str]]) -> list[dict[str, str]]:
    return [rows[rng.randrange(len(rows))] for _ in range(len(rows))]


def bootstrap_group(
    rows: Sequence[dict[str, str]],
    *,
    rng: random.Random,
    iterations: int,
) -> dict[str, float]:
    pass_count = 0
    values = {axis: [] for axis in AXES}
    for _ in range(max(1, int(iterations))):
        sampled = sample_rows(rng, rows)
        improvements = {axis: improvement(sampled, axis) for axis in AXES}
        for axis, value in improvements.items():
            values[axis].append(value)
        if all(improvements[axis] >= -NONREGRESSION_EPS for axis in RDF_AXES):
            pass_count += 1
    result = {"pass_fraction": pass_count / max(1, int(iterations))}
    for axis in AXES:
        result[f"{axis}_p025"] = quantile(values[axis], 0.025)
    return result


def summarize_candidate(
    rows: Sequence[dict[str, str]],
    *,
    forward_min_range: float | None,
    disabled_screen_bins: Sequence[tuple[int, int]],
    rng: random.Random,
    iterations: int,
    min_samples: int,
    min_valid_roi: int,
) -> dict[str, object]:
    groups = grouped_rows(rows, ("all", "trial", "range_bin", "screen_bin"))
    scoped_groups = {
        label: group_rows
        for label, group_rows in groups.items()
        if len(group_rows) >= int(min_samples) and sum(row.get("fallback") == "none" for row in group_rows) >= int(min_valid_roi)
    }
    base_improvements = {axis: improvement(rows, axis) for axis in AXES}
    min_group_improvements = {
        axis: min((improvement(group_rows, axis) for group_rows in scoped_groups.values()), default=float("nan"))
        for axis in AXES
    }
    bootstrap_rows: list[dict[str, object]] = []
    worst_pass = 1.0
    worst_p025 = {axis: float("inf") for axis in AXES}
    worst_label = ""
    for label, group_rows in sorted(scoped_groups.items()):
        summary = bootstrap_group(group_rows, rng=rng, iterations=iterations)
        bootstrap_rows.append(
            {
                "group_label": label,
                "sample_count": len(group_rows),
                "valid_roi_count": sum(row.get("fallback") == "none" for row in group_rows),
                "bootstrap_pass_fraction": f"{summary['pass_fraction']:.9f}",
                "right_p025_m": f"{summary['right_p025']:.9f}",
                "down_p025_m": f"{summary['down_p025']:.9f}",
                "forward_p025_m": f"{summary['forward_p025']:.9f}",
            }
        )
        if float(summary["pass_fraction"]) < worst_pass:
            worst_pass = float(summary["pass_fraction"])
            worst_label = label
    for axis in AXES:
            worst_p025[axis] = min(worst_p025[axis], float(summary[f"{axis}_p025"]))
    disabled_label = screen_set_label(disabled_screen_bins)
    forward_label = "" if forward_min_range is None else f"{forward_min_range:g}"
    candidate_id = f"forward_min={forward_label};disable_down={disabled_label}"
    return {
        "candidate_id": candidate_id,
        "forward_min_range_m": forward_label,
        "disable_down_screen_bins": disabled_label,
        "sample_count": len(rows),
        "valid_roi_count": sum(row.get("fallback") == "none" for row in rows),
        "range_guarded_forward_noop_count": sum(row.get("forward_fallback") == "range_guarded_noop" for row in rows),
        "screen_guarded_down_noop_count": sum(row.get("down_guarded_noop") == "yes" for row in rows),
        "base_right_rmse_improvement_m": f"{base_improvements['right']:.9f}",
        "base_down_rmse_improvement_m": f"{base_improvements['down']:.9f}",
        "base_forward_rmse_improvement_m": f"{base_improvements['forward']:.9f}",
        "base_position_rmse_improvement_m": f"{base_improvements['position']:.9f}",
        "min_group_right_rmse_improvement_m": f"{min_group_improvements['right']:.9f}",
        "min_group_down_rmse_improvement_m": f"{min_group_improvements['down']:.9f}",
        "min_group_forward_rmse_improvement_m": f"{min_group_improvements['forward']:.9f}",
        "min_group_position_rmse_improvement_m": f"{min_group_improvements['position']:.9f}",
        "worst_bootstrap_pass_fraction": f"{worst_pass:.9f}",
        "worst_bootstrap_group": worst_label,
        "worst_right_p025_m": f"{worst_p025['right']:.9f}",
        "worst_down_p025_m": f"{worst_p025['down']:.9f}",
        "worst_forward_p025_m": f"{worst_p025['forward']:.9f}",
        "bootstrap_rows": bootstrap_rows,
    }


def main() -> int:
    args = parse_args()
    source_rows = read_rows(args.sample_diagnostics_csv)
    forward_min_ranges = parse_forward_min_ranges(args.forward_min_ranges_m)
    screen_bins = parse_screen_bins(args.screen_bin_options, source_rows)
    rng = random.Random(args.bootstrap_seed)
    candidate_rows: list[dict[str, object]] = []
    best_rows: list[dict[str, str]] = []
    best_bootstrap_rows: list[dict[str, object]] = []
    best_candidate: dict[str, object] | None = None
    for forward_min, disabled in guard_candidates(
        source_rows,
        forward_min_ranges=forward_min_ranges,
        screen_bins=screen_bins,
        max_disabled_screen_bins=int(args.max_disabled_screen_bins),
    ):
        guarded_rows = apply_guard(source_rows, forward_min_range=forward_min, disabled_screen_bins=disabled)
        summary = summarize_candidate(
            guarded_rows,
            forward_min_range=forward_min,
            disabled_screen_bins=disabled,
            rng=rng,
            iterations=int(args.bootstrap_iterations),
            min_samples=int(args.min_samples),
            min_valid_roi=int(args.min_valid_roi),
        )
        bootstrap_rows = summary.pop("bootstrap_rows")
        candidate_rows.append(summary)
        if best_candidate is None or (
            float(summary["worst_bootstrap_pass_fraction"]),
            float(summary["min_group_down_rmse_improvement_m"]),
            float(summary["min_group_forward_rmse_improvement_m"]),
            float(summary["base_position_rmse_improvement_m"]),
        ) > (
            float(best_candidate["worst_bootstrap_pass_fraction"]),
            float(best_candidate["min_group_down_rmse_improvement_m"]),
            float(best_candidate["min_group_forward_rmse_improvement_m"]),
            float(best_candidate["base_position_rmse_improvement_m"]),
        ):
            best_candidate = summary
            best_rows = guarded_rows
            best_bootstrap_rows = list(bootstrap_rows)
    candidate_rows.sort(
        key=lambda row: (
            float(row["worst_bootstrap_pass_fraction"]),
            float(row["min_group_down_rmse_improvement_m"]),
            float(row["min_group_forward_rmse_improvement_m"]),
            float(row["base_position_rmse_improvement_m"]),
        ),
        reverse=True,
    )
    write_csv(args.output_dir / "guard_policy_candidate_summary.csv", candidate_rows)
    if best_candidate is not None:
        write_csv(args.output_dir / "best_guard_policy_bootstrap_summary.csv", best_bootstrap_rows)
        if args.write_best_samples:
            write_csv(args.output_dir / "best_candidate_sample_diagnostics.csv", best_rows)
    print(f"Candidates: {len(candidate_rows)}")
    print(f"Summary: {args.output_dir / 'guard_policy_candidate_summary.csv'}")
    if candidate_rows:
        best = candidate_rows[0]
        print(
            "Best: "
            f"{best['candidate_id']} worst_pass={best['worst_bootstrap_pass_fraction']} "
            f"min_group_rdf=({best['min_group_right_rmse_improvement_m']},"
            f"{best['min_group_down_rmse_improvement_m']},"
            f"{best['min_group_forward_rmse_improvement_m']})"
        )
    return 0 if candidate_rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
