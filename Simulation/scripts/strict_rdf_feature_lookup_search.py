#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.optimize import differential_evolution, minimize


RDF_AXES = ("right", "down", "forward")
AXES = ("right", "down", "forward", "position")
STRICT_EPSILON_M = 1.0e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit and replay a runtime-feature lookup correction over "
            "bbox-anchor comparison CSVs under the Strict RDF all-axis gate."
        )
    )
    parser.add_argument("--fit-report", action="append", type=Path, required=True)
    parser.add_argument("--eval-report", action="append", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-abs-gain", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--strict-epsilon-m", type=float, default=STRICT_EPSILON_M)
    return parser.parse_args()


def parse_ints(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def finite_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def load_rows(report_dirs: Sequence[Path], trials: Sequence[int]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for report_dir in report_dirs:
        for trial in trials:
            path = report_dir / f"validation_trial_{trial:02d}_anchor_comparison.csv"
            if not path.exists():
                raise FileNotFoundError(path)
            with path.open(newline="", encoding="utf-8") as handle:
                for row in csv.DictReader(handle):
                    row = dict(row)
                    row["report_dir"] = report_dir.name
                    row["trial"] = trial
                    # Legacy global-anchor reports predate this metadata column.
                    # The CSV axis contract is still RDF; only the profile-family
                    # feature is aliased to global for lookup selection.
                    row["anchor_cell"] = row.get("anchor_cell") or (
                        "global" if "global" in report_dir.name else "unknown"
                    )
                    rows.append(row)
    return rows


def trial_keys(rows: Sequence[dict[str, object]]) -> list[tuple[str, int]]:
    return sorted({(str(row["report_dir"]), int(row["trial"])) for row in rows})


def basis_names(rows: Sequence[dict[str, object]]) -> list[str]:
    cells = sorted({str(row["anchor_cell"]) for row in rows})
    names: list[str] = []
    for source in ("anchor", "center"):
        names.extend(f"{source}:{feature}" for feature in ("all", "raw_p", "raw_n", "u_l", "u_r"))
        names.extend(f"{source}:cell={cell}" for cell in cells)
        for cell in cells:
            names.append(f"{source}:cellraw={cell}_p")
            names.append(f"{source}:cellraw={cell}_n")
    return names


def float_field(row: dict[str, object], key: str) -> float:
    value = finite_float(row.get(key))
    if value is None:
        raise ValueError(f"missing finite field {key!r} in row")
    return value


def source_delta(row: dict[str, object], source: str, axis: str) -> float:
    prefix = "anchor" if source == "anchor" else "center_rot"
    return float_field(row, f"{prefix}_{axis}_m") - float_field(row, f"raw_{axis}_m")


def basis_applies(row: dict[str, object], feature: str, axis: str) -> bool:
    raw_value = float_field(row, f"raw_{axis}_m")
    cell = str(row["anchor_cell"])
    u_px = float_field(row, "anchor_left_u_px")
    if feature == "all":
        return True
    if feature == "raw_p":
        return raw_value > 0.0
    if feature == "raw_n":
        return raw_value <= 0.0
    if feature == "u_l":
        return u_px < 960.0
    if feature == "u_r":
        return u_px >= 960.0
    if feature.startswith("cell="):
        return cell == feature.split("=", 1)[1]
    if feature.startswith("cellraw="):
        expected_cell, sign = feature.split("=", 1)[1].rsplit("_", 1)
        return cell == expected_cell and ((raw_value > 0.0) == (sign == "p"))
    raise ValueError(feature)


def design_matrix(
    rows: Sequence[dict[str, object]],
    keys: Sequence[tuple[str, int]],
    axis: str,
    bases: Sequence[str],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    key_index = {key: index for index, key in enumerate(keys)}
    errors = np.empty(len(rows))
    trial_index = np.empty(len(rows), dtype=int)
    design = np.zeros((len(rows), len(bases)))
    for row_index, row in enumerate(rows):
        errors[row_index] = float_field(row, f"raw_{axis}_m") - float_field(row, f"truth_{axis}_m")
        trial_index[row_index] = key_index[(str(row["report_dir"]), int(row["trial"]))]
        for basis_index, basis in enumerate(bases):
            source, feature = basis.split(":", 1)
            if basis_applies(row, feature, axis):
                design[row_index, basis_index] = source_delta(row, source, axis)
    return errors, design, trial_index


def trial_coefficients(
    errors: np.ndarray,
    design: np.ndarray,
    trial_index: np.ndarray,
    trial_count: int,
) -> tuple[np.ndarray, list[np.ndarray], list[np.ndarray], list[int]]:
    raw_mse = np.empty(trial_count)
    quadratic: list[np.ndarray] = []
    linear: list[np.ndarray] = []
    sample_counts: list[int] = []
    for trial in range(trial_count):
        mask = trial_index == trial
        if not bool(mask.any()):
            raise ValueError(f"empty trial index {trial}")
        sample_counts.append(int(mask.sum()))
        raw_mse[trial] = float(np.mean(errors[mask] ** 2))
        quadratic.append(design[mask].T @ design[mask] / float(mask.sum()))
        linear.append(design[mask].T @ errors[mask] / float(mask.sum()))
    return raw_mse, quadratic, linear, sample_counts


def axis_improvements(
    weights: np.ndarray,
    raw_mse: np.ndarray,
    quadratic: Sequence[np.ndarray],
    linear: Sequence[np.ndarray],
) -> np.ndarray:
    improvements = []
    for raw_value, q_matrix, l_vector in zip(raw_mse, quadratic, linear):
        corrected_mse = raw_value + 2.0 * float(l_vector.dot(weights)) + float(weights.dot(q_matrix).dot(weights))
        improvements.append(math.sqrt(raw_value) - math.sqrt(max(corrected_mse, 0.0)))
    return np.asarray(improvements)


def fit_axis_policy(
    rows: Sequence[dict[str, object]],
    axis: str,
    bases: Sequence[str],
    *,
    max_abs_gain: float,
    seed: int,
) -> dict[str, float]:
    keys = trial_keys(rows)
    errors, design, trial_index = design_matrix(rows, keys, axis, bases)
    raw_mse, quadratic, linear, _ = trial_coefficients(errors, design, trial_index, len(keys))

    def differential_objective(weights: np.ndarray) -> float:
        improvements = axis_improvements(weights, raw_mse, quadratic, linear)
        return -float(np.min(improvements)) + 1.0e-5 * float(np.sum(weights * weights))

    bounds = [(-max_abs_gain, max_abs_gain)] * len(bases)
    rough = differential_evolution(
        differential_objective,
        bounds,
        maxiter=60,
        popsize=6,
        seed=seed,
        polish=False,
        tol=1.0e-6,
        updating="immediate",
    )

    def constrained_objective(values: np.ndarray) -> float:
        weights = values[:-1]
        threshold = values[-1]
        return -float(threshold) + 1.0e-6 * float(weights.dot(weights))

    constraints = [
        {
            "type": "ineq",
            "fun": lambda values, index=index: float(
                axis_improvements(values[:-1], raw_mse, quadratic, linear)[index] - values[-1]
            ),
        }
        for index in range(len(keys))
    ]
    start = np.r_[rough.x, np.min(axis_improvements(rough.x, raw_mse, quadratic, linear))]
    result = minimize(
        constrained_objective,
        start,
        method="SLSQP",
        bounds=bounds + [(-1.0, 1.0)],
        constraints=constraints,
        options={"maxiter": 2000, "ftol": 1.0e-12},
    )
    weights = result.x[:-1]
    return {basis: float(weight) for basis, weight in zip(bases, weights) if abs(float(weight)) > 1.0e-10}


def corrected_value(row: dict[str, object], axis: str, policy: dict[str, float]) -> float:
    corrected = float_field(row, f"raw_{axis}_m")
    for basis, gain in policy.items():
        source, feature = basis.split(":", 1)
        if basis_applies(row, feature, axis):
            corrected += gain * source_delta(row, source, axis)
    return corrected


def evaluate(
    rows: Sequence[dict[str, object]],
    policies: dict[str, dict[str, float]],
    *,
    strict_epsilon_m: float,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["report_dir"]), int(row["trial"]))].append(row)

    per_trial: list[dict[str, object]] = []
    min_delta = {axis: float("inf") for axis in AXES}
    mean_delta = {axis: 0.0 for axis in AXES}
    sample_count = 0
    corrected_sample_count = 0
    pass_count = 0
    for key, trial_rows in sorted(grouped.items()):
        sums = {f"raw_{axis}": 0.0 for axis in RDF_AXES}
        sums.update({f"corrected_{axis}": 0.0 for axis in RDF_AXES})
        sums["raw_position"] = 0.0
        sums["corrected_position"] = 0.0
        for row in trial_rows:
            sample_count += 1
            raw_errors: dict[str, float] = {}
            corrected_errors: dict[str, float] = {}
            for axis in RDF_AXES:
                truth = float_field(row, f"truth_{axis}_m")
                raw_errors[axis] = float_field(row, f"raw_{axis}_m") - truth
                corrected = corrected_value(row, axis, policies[axis])
                corrected_errors[axis] = corrected - truth
                if abs(corrected - float_field(row, f"raw_{axis}_m")) > 1.0e-12:
                    corrected_sample_count += 1
            for axis in RDF_AXES:
                sums[f"raw_{axis}"] += raw_errors[axis] * raw_errors[axis]
                sums[f"corrected_{axis}"] += corrected_errors[axis] * corrected_errors[axis]
            sums["raw_position"] += sum(value * value for value in raw_errors.values())
            sums["corrected_position"] += sum(value * value for value in corrected_errors.values())

        n = float(len(trial_rows))
        deltas = {
            axis: math.sqrt(sums[f"raw_{axis}"] / n) - math.sqrt(sums[f"corrected_{axis}"] / n)
            for axis in RDF_AXES
        }
        deltas["position"] = math.sqrt(sums["raw_position"] / n) - math.sqrt(sums["corrected_position"] / n)
        strict_pass = all(deltas[axis] > strict_epsilon_m for axis in AXES)
        if strict_pass:
            pass_count += 1
        for axis in AXES:
            min_delta[axis] = min(min_delta[axis], deltas[axis])
            mean_delta[axis] += deltas[axis]
        per_trial.append(
            {
                "report_dir": key[0],
                "trial": key[1],
                "sample_count": len(trial_rows),
                "pass_strict_rdf_all_axis": "yes" if strict_pass else "no",
                **{f"{axis}_rmse_improvement_m": f"{deltas[axis]:.12f}" for axis in AXES},
            }
        )

    trial_count = len(per_trial)
    for axis in AXES:
        mean_delta[axis] /= float(trial_count)
    summary = {
        "report_count": len({row["report_dir"] for row in rows}),
        "trial_count": trial_count,
        "sample_count": sample_count,
        "corrected_sample_count": corrected_sample_count,
        "pass_trial_count": pass_count,
        "total_trial_count": trial_count,
        "corrected_sample_coverage": corrected_sample_count / float(sample_count * len(RDF_AXES)),
        **{f"{axis}_min_delta_m": round(min_delta[axis], 12) for axis in AXES},
        **{f"{axis}_mean_delta_m": round(mean_delta[axis], 12) for axis in AXES},
    }
    return summary, per_trial


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


def main() -> int:
    args = parse_args()
    trials = parse_ints(args.validation_trials)
    fit_reports = [path.resolve() for path in args.fit_report]
    eval_reports = [path.resolve() for path in args.eval_report]
    fit_rows = load_rows(fit_reports, trials)
    eval_rows = load_rows(eval_reports, trials)
    bases = basis_names(fit_rows + eval_rows)

    policies = {
        "right": fit_axis_policy(fit_rows, "right", bases, max_abs_gain=args.max_abs_gain, seed=args.seed),
        "down": fit_axis_policy(fit_rows, "down", bases, max_abs_gain=args.max_abs_gain, seed=args.seed + 1),
        "forward": {"center:all": 1.0},
    }
    fit_summary, fit_trials = evaluate(fit_rows, policies, strict_epsilon_m=args.strict_epsilon_m)
    eval_summary, eval_trials = evaluate(eval_rows, policies, strict_epsilon_m=args.strict_epsilon_m)

    output_dir = args.output_dir
    write_json(output_dir / "policy.json", {"basis": bases, "axis_policies": policies})
    write_json(output_dir / "metrics.json", {"fit": fit_summary, "eval": eval_summary})
    write_csv(output_dir / "fit_per_trial.csv", fit_trials)
    write_csv(output_dir / "eval_per_trial.csv", eval_trials)

    print(f"fit_pass={fit_summary['pass_trial_count']}/{fit_summary['total_trial_count']}")
    print(f"eval_pass={eval_summary['pass_trial_count']}/{eval_summary['total_trial_count']}")
    print(
        "eval_mins="
        f"R={eval_summary['right_min_delta_m']} "
        f"D={eval_summary['down_min_delta_m']} "
        f"F={eval_summary['forward_min_delta_m']} "
        f"P={eval_summary['position_min_delta_m']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
