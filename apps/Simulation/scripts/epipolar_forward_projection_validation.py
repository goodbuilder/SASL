#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path
from typing import Iterable, Sequence


RDF_AXES = ("right", "down", "forward")
AXES = ("right", "down", "forward", "position")
STRICT_EPSILON_M = 1.0e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate an epipolar-geometry Forward correction that uses the "
            "right-camera rotation retriangulated bbox center Forward estimate "
            "while keeping Right/Down as protected raw axes."
        )
    )
    parser.add_argument("--reports-root", type=Path, default=Path("reports"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--bootstrap-iterations", type=int, default=500)
    parser.add_argument("--bootstrap-seed", type=int, default=20260612)
    parser.add_argument("--strict-epsilon-m", type=float, default=STRICT_EPSILON_M)
    return parser.parse_args()


def parse_ints(text: str) -> set[int]:
    return {int(part.strip()) for part in str(text).split(",") if part.strip()}


def finite_float(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


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


def validation_paths(reports_root: Path, trials: set[int]) -> list[Path]:
    paths: list[Path] = []
    for path in sorted(reports_root.glob("*/validation_trial_*_anchor_comparison.csv")):
        trial_text = path.name.removeprefix("validation_trial_").removesuffix("_anchor_comparison.csv")
        try:
            trial = int(trial_text)
        except ValueError:
            continue
        if trial in trials:
            paths.append(path)
    return paths


def load_samples(paths: Iterable[Path]) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    for path in paths:
        trial = int(path.name.removeprefix("validation_trial_").removesuffix("_anchor_comparison.csv"))
        report_dir = path.parent.name
        for row_index, row in enumerate(read_csv(path)):
            truth = {axis: finite_float(row.get(f"truth_{axis}_m")) for axis in RDF_AXES}
            raw = {axis: finite_float(row.get(f"raw_{axis}_m")) for axis in RDF_AXES}
            center_forward = finite_float(row.get("center_rot_forward_m"))
            if any(value is None for value in truth.values()):
                continue
            if any(value is None for value in raw.values()) or center_forward is None:
                continue
            corrected = {
                "right": float(raw["right"]),
                "down": float(raw["down"]),
                "forward": float(center_forward),
            }
            raw_values = [float(raw[axis]) for axis in RDF_AXES]
            corrected_values = [corrected[axis] for axis in RDF_AXES]
            truth_values = [float(truth[axis]) for axis in RDF_AXES]
            samples.append(
                {
                    "report_dir": report_dir,
                    "trial": trial,
                    "row_index": row_index,
                    "truth_right_m": truth_values[0],
                    "truth_down_m": truth_values[1],
                    "truth_forward_m": truth_values[2],
                    "raw_right_m": raw_values[0],
                    "raw_down_m": raw_values[1],
                    "raw_forward_m": raw_values[2],
                    "corrected_right_m": corrected_values[0],
                    "corrected_down_m": corrected_values[1],
                    "corrected_forward_m": corrected_values[2],
                    "raw_error_right_m": raw_values[0] - truth_values[0],
                    "raw_error_down_m": raw_values[1] - truth_values[1],
                    "raw_error_forward_m": raw_values[2] - truth_values[2],
                    "corrected_error_right_m": corrected_values[0] - truth_values[0],
                    "corrected_error_down_m": corrected_values[1] - truth_values[1],
                    "corrected_error_forward_m": corrected_values[2] - truth_values[2],
                }
            )
    return samples


def summarize(samples: Sequence[dict[str, object]], *, strict_epsilon_m: float = STRICT_EPSILON_M) -> dict[str, object]:
    result: dict[str, object] = {"sample_count": len(samples)}
    pass_rdf = True
    pass_strict_rdf = True
    for axis in RDF_AXES:
        raw_errors = [float(row[f"raw_error_{axis}_m"]) for row in samples]
        corrected_errors = [float(row[f"corrected_error_{axis}_m"]) for row in samples]
        raw_value = rmse(raw_errors)
        corrected_value = rmse(corrected_errors)
        improvement = raw_value - corrected_value
        result[f"raw_{axis}_rmse_m"] = f"{raw_value:.9f}"
        result[f"corrected_{axis}_rmse_m"] = f"{corrected_value:.9f}"
        result[f"{axis}_rmse_improvement_m"] = f"{improvement:.9f}"
        if not (math.isfinite(improvement) and improvement >= -1.0e-12):
            pass_rdf = False
        if not (math.isfinite(improvement) and improvement > strict_epsilon_m):
            pass_strict_rdf = False

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
        pass_strict_rdf = False
    result["pass_rdf_nonregression"] = "yes" if pass_rdf else "no"
    result["pass_strict_rdf_all_axis"] = "yes" if pass_strict_rdf else "no"
    result["right_corrected_count"] = 0
    result["down_corrected_count"] = 0
    result["forward_corrected_count"] = len(samples)
    return result


def trial_summary(samples: Sequence[dict[str, object]], *, strict_epsilon_m: float) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in samples:
        grouped.setdefault((str(row["report_dir"]), int(row["trial"])), []).append(row)
    output: list[dict[str, object]] = []
    for (report_dir, trial), group in sorted(grouped.items()):
        summary = summarize(group, strict_epsilon_m=strict_epsilon_m)
        output.append(
            {
                "report_dir": report_dir,
                "trial": trial,
                **{
                    f"{axis}_rmse_improvement_m": summary[f"{axis}_rmse_improvement_m"]
                    for axis in AXES
                },
                "sample_count": summary["sample_count"],
                "pass_rdf_nonregression": summary["pass_rdf_nonregression"],
                "pass_strict_rdf_all_axis": summary["pass_strict_rdf_all_axis"],
                "right_corrected_count": summary["right_corrected_count"],
                "down_corrected_count": summary["down_corrected_count"],
                "forward_corrected_count": summary["forward_corrected_count"],
            }
        )
    return output


def aggregate_candidate(per_trial: Sequence[dict[str, object]], *, strict_epsilon_m: float) -> dict[str, object]:
    result: dict[str, object] = {
        "candidate_id": "epipolar_forward_projection_centerrot_forward_protected_rd",
        "method": "epipolar_forward_projection",
        "report_count": len({str(row["report_dir"]) for row in per_trial}),
        "trial_count": len(per_trial),
        "pass_trial_count": sum(row["pass_rdf_nonregression"] == "yes" for row in per_trial),
        "pass_all_nonregression": "yes"
        if all(row["pass_rdf_nonregression"] == "yes" for row in per_trial)
        else "no",
        "strict_pass_trial_count": sum(row["pass_strict_rdf_all_axis"] == "yes" for row in per_trial),
        "strict_pass_all_trials": "yes"
        if all(row["pass_strict_rdf_all_axis"] == "yes" for row in per_trial)
        else "no",
        "strict_epsilon_m": f"{strict_epsilon_m:.12f}",
        "right_corrected_count": 0,
        "down_corrected_count": 0,
        "forward_corrected_count": sum(int(row["forward_corrected_count"]) for row in per_trial),
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
    values: dict[str, list[float]] = {axis: [] for axis in RDF_AXES}
    pass_count = 0
    strict_pass_count = 0
    if mode == "trials":
        grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
        for row in samples:
            grouped.setdefault((str(row["report_dir"]), int(row["trial"])), []).append(row)
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
        if summary["pass_rdf_nonregression"] == "yes":
            pass_count += 1
        if summary["pass_strict_rdf_all_axis"] == "yes":
            strict_pass_count += 1
        for axis in RDF_AXES:
            values[axis].append(float(summary[f"{axis}_rmse_improvement_m"]))
    result: dict[str, object] = {
        "bootstrap_mode": mode,
        "bootstrap_iterations": iterations,
        "bootstrap_pass_fraction": f"{pass_count / max(1, iterations):.9f}",
        "bootstrap_strict_pass_fraction": f"{strict_pass_count / max(1, iterations):.9f}",
        "bootstrap_gate": "rdf_nonregression_and_strict_rdf_all_axis",
    }
    for axis in RDF_AXES:
        result[f"{axis}_p025_m"] = f"{percentile(values[axis], 0.025):.9f}"
        result[f"{axis}_mean_m"] = f"{sum(values[axis]) / len(values[axis]):.9f}"
    return result


def main() -> int:
    args = parse_args()
    paths = validation_paths(args.reports_root, parse_ints(args.validation_trials))
    samples = load_samples(paths)
    if not samples:
        raise RuntimeError("no validation samples loaded")
    strict_epsilon_m = float(args.strict_epsilon_m)
    per_trial = trial_summary(samples, strict_epsilon_m=strict_epsilon_m)
    candidate = aggregate_candidate(per_trial, strict_epsilon_m=strict_epsilon_m)
    rng = random.Random(int(args.bootstrap_seed))
    bootstrap_rows = [
        bootstrap(samples, rng, int(args.bootstrap_iterations), "rows", strict_epsilon_m=strict_epsilon_m),
        bootstrap(samples, rng, int(args.bootstrap_iterations), "trials", strict_epsilon_m=strict_epsilon_m),
    ]
    status = "PASS"
    tags: list[str] = []
    if candidate["strict_pass_all_trials"] != "yes":
        status = "REJECTED_STRICT_RDF_GATE"
        tags.append("strict_rdf_all_axis_failed")
    if float(candidate["min_right_rmse_improvement_m"]) <= strict_epsilon_m:
        tags.append("right_not_improved")
    if float(candidate["min_down_rmse_improvement_m"]) <= strict_epsilon_m:
        tags.append("down_not_improved")
    if float(candidate["mean_forward_rmse_improvement_m"]) <= 1.0:
        tags.append("weak_forward_signal")
    for row in bootstrap_rows:
        if float(row["bootstrap_strict_pass_fraction"]) < 0.99:
            tags.append("bootstrap_unstable")

    sample_rows = []
    for row in samples:
        sample_rows.append(
            {
                key: f"{value:.9f}" if isinstance(value, float) else value
                for key, value in row.items()
            }
        )

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
            "validation_contract": (
                "All validation_trial_*_anchor_comparison.csv files are replayed. "
                "Strict RDF PASS requires Right, Down, Forward, and Position RMSE "
                "improvement > strict_epsilon_m in every validation trajectory. "
                "This method keeps Right/Down as protected raw no-op axes and therefore "
                "fails the strict RDF all-axis gate. Forward uses center_rot_forward_m, "
                "which is computed from bbox center triangulation with the configured "
                "right-camera rotation and does not use ground truth at runtime."
            ),
            "axis_contract": "Internal evaluation axes are RDF: right, down, forward.",
            "strict_epsilon_m": strict_epsilon_m,
            "input_files": [str(path) for path in paths],
            "candidate": candidate,
            "bootstrap": bootstrap_rows,
        },
    )
    print(
        "Best: "
        f"{candidate['candidate_id']} status={status} "
        f"pass={candidate['pass_trial_count']}/{candidate['trial_count']} "
        f"strict_pass={candidate['strict_pass_trial_count']}/{candidate['trial_count']} "
        f"min_rdf=({candidate['min_right_rmse_improvement_m']},"
        f"{candidate['min_down_rmse_improvement_m']},"
        f"{candidate['min_forward_rmse_improvement_m']}) "
        f"mean_forward={candidate['mean_forward_rmse_improvement_m']} "
        f"tags={','.join(sorted(set(tags)))}"
    )
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
