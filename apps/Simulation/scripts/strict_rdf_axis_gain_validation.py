#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path
from typing import Sequence


RDF_AXES = ("right", "down", "forward")
AXES = ("right", "down", "forward", "position")
STRICT_EPSILON_M = 1.0e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a family-local strict RDF bbox-anchor axis-gain correction. "
            "The correction uses runtime-available raw/anchor/center-rotation "
            "geometry values and does not use truth as a runtime input."
        )
    )
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument(
        "--source",
        choices=("anchor", "center", "anchor_rd_center_f", "center_rd_anchor_f"),
        required=True,
    )
    parser.add_argument("--right-gain", type=float, required=True)
    parser.add_argument("--down-gain", type=float, required=True)
    parser.add_argument("--forward-gain", type=float, required=True)
    parser.add_argument("--strict-epsilon-m", type=float, default=STRICT_EPSILON_M)
    parser.add_argument("--bootstrap-iterations", type=int, default=500)
    parser.add_argument("--bootstrap-seed", type=int, default=20260612)
    return parser.parse_args()


def parse_ints(text: str) -> list[int]:
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def finite_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def percentile(values: Sequence[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lo = int(math.floor(position))
    hi = int(math.ceil(position))
    if lo == hi:
        return ordered[lo]
    frac = position - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def source_values(row: dict[str, str], source: str) -> dict[str, float] | None:
    raw = {axis: finite_float(row.get(f"raw_{axis}_m")) for axis in RDF_AXES}
    anchor = {axis: finite_float(row.get(f"anchor_{axis}_m")) for axis in RDF_AXES}
    center = {axis: finite_float(row.get(f"center_rot_{axis}_m")) for axis in RDF_AXES}
    if any(value is None for value in raw.values()):
        return None
    if source in ("anchor", "anchor_rd_center_f", "center_rd_anchor_f") and any(
        value is None for value in anchor.values()
    ):
        return None
    if source in ("center", "anchor_rd_center_f", "center_rd_anchor_f") and any(
        value is None for value in center.values()
    ):
        return None
    if source == "anchor":
        return {axis: float(anchor[axis]) for axis in RDF_AXES}
    if source == "center":
        return {axis: float(center[axis]) for axis in RDF_AXES}
    if source == "anchor_rd_center_f":
        return {
            "right": float(anchor["right"]),
            "down": float(anchor["down"]),
            "forward": float(center["forward"]),
        }
    if source == "center_rd_anchor_f":
        return {
            "right": float(center["right"]),
            "down": float(center["down"]),
            "forward": float(anchor["forward"]),
        }
    raise ValueError(source)


def load_samples(
    report_dir: Path,
    trials: Sequence[int],
    *,
    source: str,
    gains: dict[str, float],
) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    for trial in trials:
        path = report_dir / f"validation_trial_{trial:02d}_anchor_comparison.csv"
        if not path.exists():
            raise FileNotFoundError(path)
        for row_index, row in enumerate(read_csv(path)):
            truth = {axis: finite_float(row.get(f"truth_{axis}_m")) for axis in RDF_AXES}
            raw = {axis: finite_float(row.get(f"raw_{axis}_m")) for axis in RDF_AXES}
            source_point = source_values(row, source)
            if any(value is None for value in truth.values()):
                continue
            if any(value is None for value in raw.values()) or source_point is None:
                continue
            corrected = {
                axis: float(raw[axis]) + gains[axis] * (float(source_point[axis]) - float(raw[axis]))
                for axis in RDF_AXES
            }
            sample: dict[str, object] = {
                "report_dir": report_dir.name,
                "trial": trial,
                "row_index": row_index,
                "source": source,
            }
            for axis in RDF_AXES:
                sample[f"truth_{axis}_m"] = float(truth[axis])
                sample[f"raw_{axis}_m"] = float(raw[axis])
                sample[f"source_{axis}_m"] = float(source_point[axis])
                sample[f"corrected_{axis}_m"] = corrected[axis]
                sample[f"gain_{axis}"] = gains[axis]
                sample[f"raw_error_{axis}_m"] = float(raw[axis]) - float(truth[axis])
                sample[f"corrected_error_{axis}_m"] = corrected[axis] - float(truth[axis])
            samples.append(sample)
    return samples


def summarize(samples: Sequence[dict[str, object]], *, strict_epsilon_m: float) -> dict[str, object]:
    result: dict[str, object] = {"sample_count": len(samples)}
    pass_strict = True
    for axis in RDF_AXES:
        raw_errors = [float(row[f"raw_error_{axis}_m"]) for row in samples]
        corrected_errors = [float(row[f"corrected_error_{axis}_m"]) for row in samples]
        raw_value = rmse(raw_errors)
        corrected_value = rmse(corrected_errors)
        improvement = raw_value - corrected_value
        result[f"raw_{axis}_rmse_m"] = f"{raw_value:.9f}"
        result[f"corrected_{axis}_rmse_m"] = f"{corrected_value:.9f}"
        result[f"{axis}_rmse_improvement_m"] = f"{improvement:.9f}"
        if not (math.isfinite(improvement) and improvement > strict_epsilon_m):
            pass_strict = False
    raw_position = [
        math.sqrt(
            float(row["raw_error_right_m"]) ** 2
            + float(row["raw_error_down_m"]) ** 2
            + float(row["raw_error_forward_m"]) ** 2
        )
        for row in samples
    ]
    corrected_position = [
        math.sqrt(
            float(row["corrected_error_right_m"]) ** 2
            + float(row["corrected_error_down_m"]) ** 2
            + float(row["corrected_error_forward_m"]) ** 2
        )
        for row in samples
    ]
    position_improvement = rmse(raw_position) - rmse(corrected_position)
    result["raw_position_rmse_m"] = f"{rmse(raw_position):.9f}"
    result["corrected_position_rmse_m"] = f"{rmse(corrected_position):.9f}"
    result["position_rmse_improvement_m"] = f"{position_improvement:.9f}"
    if not (math.isfinite(position_improvement) and position_improvement > strict_epsilon_m):
        pass_strict = False
    result["pass_strict_rdf_all_axis"] = "yes" if pass_strict else "no"
    return result


def trial_summary(samples: Sequence[dict[str, object]], *, strict_epsilon_m: float) -> list[dict[str, object]]:
    grouped: dict[int, list[dict[str, object]]] = {}
    for row in samples:
        grouped.setdefault(int(row["trial"]), []).append(row)
    rows: list[dict[str, object]] = []
    for trial, group in sorted(grouped.items()):
        summary = summarize(group, strict_epsilon_m=strict_epsilon_m)
        rows.append(
            {
                "trial": trial,
                "sample_count": summary["sample_count"],
                **{f"{axis}_rmse_improvement_m": summary[f"{axis}_rmse_improvement_m"] for axis in AXES},
                "pass_strict_rdf_all_axis": summary["pass_strict_rdf_all_axis"],
            }
        )
    return rows


def aggregate_summary(per_trial: Sequence[dict[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {
        "trial_count": len(per_trial),
        "strict_pass_trial_count": sum(row["pass_strict_rdf_all_axis"] == "yes" for row in per_trial),
        "strict_pass_all_trials": "yes"
        if all(row["pass_strict_rdf_all_axis"] == "yes" for row in per_trial)
        else "no",
        "sample_count": sum(int(row["sample_count"]) for row in per_trial),
    }
    for axis in AXES:
        values = [float(row[f"{axis}_rmse_improvement_m"]) for row in per_trial]
        result[f"min_{axis}_rmse_improvement_m"] = f"{min(values):.9f}"
        result[f"mean_{axis}_rmse_improvement_m"] = f"{sum(values) / len(values):.9f}"
        result[f"max_{axis}_rmse_improvement_m"] = f"{max(values):.9f}"
    return result


def bootstrap(
    samples: Sequence[dict[str, object]],
    rng: random.Random,
    iterations: int,
    mode: str,
    *,
    strict_epsilon_m: float,
) -> dict[str, object]:
    pass_count = 0
    values: dict[str, list[float]] = {axis: [] for axis in AXES}
    if mode == "trials":
        grouped: dict[int, list[dict[str, object]]] = {}
        for row in samples:
            grouped.setdefault(int(row["trial"]), []).append(row)
        clusters = list(grouped.values())
    else:
        clusters = []
    for _ in range(iterations):
        if mode == "rows":
            picked = [samples[rng.randrange(0, len(samples))] for _ in range(len(samples))]
        else:
            picked = []
            for _ in clusters:
                picked.extend(clusters[rng.randrange(0, len(clusters))])
        summary = summarize(picked, strict_epsilon_m=strict_epsilon_m)
        if summary["pass_strict_rdf_all_axis"] == "yes":
            pass_count += 1
        for axis in AXES:
            values[axis].append(float(summary[f"{axis}_rmse_improvement_m"]))
    result: dict[str, object] = {
        "bootstrap_mode": mode,
        "bootstrap_iterations": iterations,
        "bootstrap_strict_pass_fraction": f"{pass_count / max(1, iterations):.9f}",
    }
    for axis in AXES:
        result[f"{axis}_p025_m"] = f"{percentile(values[axis], 0.025):.9f}"
        result[f"{axis}_mean_m"] = f"{sum(values[axis]) / len(values[axis]):.9f}"
    return result


def main() -> int:
    args = parse_args()
    gains = {
        "right": float(args.right_gain),
        "down": float(args.down_gain),
        "forward": float(args.forward_gain),
    }
    strict_epsilon_m = float(args.strict_epsilon_m)
    samples = load_samples(
        args.report_dir,
        parse_ints(args.validation_trials),
        source=str(args.source),
        gains=gains,
    )
    if not samples:
        raise RuntimeError("no validation samples loaded")
    per_trial = trial_summary(samples, strict_epsilon_m=strict_epsilon_m)
    candidate = aggregate_summary(per_trial)
    rng = random.Random(int(args.bootstrap_seed))
    bootstrap_rows = [
        bootstrap(samples, rng, int(args.bootstrap_iterations), "rows", strict_epsilon_m=strict_epsilon_m),
        bootstrap(samples, rng, int(args.bootstrap_iterations), "trials", strict_epsilon_m=strict_epsilon_m),
    ]
    status = "FAMILY_LOCAL_PASS"
    tags: list[str] = []
    if candidate["strict_pass_all_trials"] != "yes":
        status = "REJECTED_STRICT_RDF_GATE"
        tags.append("strict_rdf_all_axis_failed")
    for row in bootstrap_rows:
        if float(row["bootstrap_strict_pass_fraction"]) < 0.99:
            status = "NEAR_PASS" if status == "FAMILY_LOCAL_PASS" else status
            tags.append("bootstrap_unstable")

    sample_rows = [
        {key: f"{value:.9f}" if isinstance(value, float) else value for key, value in row.items()}
        for row in samples
    ]
    output_dir = args.output_dir
    write_csv(output_dir / "trial_summary.csv", per_trial)
    write_csv(output_dir / "candidate_summary.csv", [candidate])
    write_csv(output_dir / "sample_diagnostics.csv", sample_rows)
    write_csv(output_dir / "bootstrap_summary.csv", bootstrap_rows)
    write_json(
        output_dir / "metadata.json",
        {
            "status": status,
            "failure_tags": sorted(set(tags)),
            "report_dir": str(args.report_dir),
            "method": "strict_rdf_axis_gain",
            "source": args.source,
            "gains": gains,
            "strict_epsilon_m": strict_epsilon_m,
            "axis_contract": "Internal evaluation axes are RDF: right, down, forward.",
            "runtime_input_contract": (
                "Correction uses raw stereo RDF and bbox anchor/center-rotation "
                "RDF geometry columns. Truth is used only for offline validation."
            ),
            "candidate": candidate,
            "bootstrap": bootstrap_rows,
        },
    )
    print(
        f"status={status} report={args.report_dir.name} source={args.source} "
        f"gains=({gains['right']},{gains['down']},{gains['forward']}) "
        f"strict_pass={candidate['strict_pass_trial_count']}/{candidate['trial_count']} "
        f"min_rdf=({candidate['min_right_rmse_improvement_m']},"
        f"{candidate['min_down_rmse_improvement_m']},"
        f"{candidate['min_forward_rmse_improvement_m']}) "
        f"tags={','.join(sorted(set(tags)))}"
    )
    return 0 if status == "FAMILY_LOCAL_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
