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
FX_PX = 8241.646774127139
BASELINE_M = 25.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay origin-bbox residual correction candidates under Strict RDF. "
            "CSV contracts are stereo-center camera RDF right/down/forward in meters."
        )
    )
    parser.add_argument("report_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument(
        "--mode",
        choices=("validation_fit", "train_trial", "report_trial_holdout"),
        required=True,
        help=(
            "validation_fit is an upper-bound overfit check; train_trial fits only "
            "profile_build_comparison.csv; report_trial_holdout excludes the held-out "
            "report and held-out validation trial number for each replay."
        ),
    )
    parser.add_argument("--fit-method", choices=("ridge", "maxmin"), default="ridge")
    parser.add_argument("--strict-epsilon-m", type=float, default=STRICT_EPSILON_M)
    parser.add_argument("--seed", type=int, default=20260613)
    return parser.parse_args()


def parse_ints(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def float_field(row: dict[str, object], key: str) -> float:
    value = finite_float(row.get(key))
    if value is None:
        if key == "raw_disparity_px":
            forward = float_field(row, "raw_forward_m")
            return BASELINE_M * FX_PX / forward
        raise ValueError(f"missing finite field {key!r}")
    return value


def load_comparison(path: Path, report_name: str, trial: int) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            row = dict(row)
            row["report_dir"] = report_name
            row["trial"] = trial
            rows.append(row)
    return rows


def load_validation_rows(report_dirs: Sequence[Path], trials: Sequence[int]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for report_dir in report_dirs:
        for trial in trials:
            path = report_dir / f"validation_trial_{trial:02d}_comparison.csv"
            if not path.exists():
                raise FileNotFoundError(path)
            rows.extend(load_comparison(path, report_dir.name, trial))
    return rows


def load_train_rows(report_dirs: Sequence[Path]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for report_dir in report_dirs:
        path = report_dir / "profile_build_comparison.csv"
        if not path.exists():
            raise FileNotFoundError(path)
        rows.extend(load_comparison(path, report_dir.name, 1))
    return rows


def feature_vector(row: dict[str, object], axis: str) -> np.ndarray:
    raw = float_field(row, f"raw_{axis}_m")
    corrected = float_field(row, f"origin_bbox_reprojection_corrected_{axis}_m")
    forward = float_field(row, "raw_forward_m")
    disparity = float_field(row, "raw_disparity_px")
    # Output remains the same RDF axis. These are scalar residual basis terms,
    # not frame conversions.
    return np.asarray(
        [
            corrected - raw,
            raw,
            1.0,
            raw / forward,
            disparity / 1000.0,
        ],
        dtype=float,
    )


def raw_error(row: dict[str, object], axis: str) -> float:
    return float_field(row, f"raw_{axis}_m") - float_field(row, f"truth_{axis}_m")


def origin_corrected_error(row: dict[str, object], axis: str) -> float:
    return float_field(row, f"origin_bbox_reprojection_corrected_{axis}_m") - float_field(
        row, f"truth_{axis}_m"
    )


def fit_ridge(rows: Sequence[dict[str, object]], axis: str) -> list[float]:
    design = np.asarray([feature_vector(row, axis) for row in rows])
    target = np.asarray([-raw_error(row, axis) for row in rows])
    best_key: tuple[float, float] | None = None
    best_weights: np.ndarray | None = None
    for regularization in (0.0, 1.0e-8, 1.0e-6, 1.0e-4, 1.0e-2, 1.0, 100.0):
        weights = np.linalg.solve(
            design.T @ design + regularization * np.eye(design.shape[1]),
            design.T @ target,
        )
        improvements = axis_trial_improvements(rows, axis, weights)
        key = (min(improvements), sum(improvements) / len(improvements))
        if best_key is None or key > best_key:
            best_key = key
            best_weights = weights
    assert best_weights is not None
    return [float(value) for value in best_weights]


def axis_trial_improvements(rows: Sequence[dict[str, object]], axis: str, weights: np.ndarray) -> list[float]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["report_dir"]), int(row["trial"]))].append(row)
    improvements: list[float] = []
    for trial_rows in grouped.values():
        raw_sse = 0.0
        corrected_sse = 0.0
        for row in trial_rows:
            error = raw_error(row, axis)
            corrected_error = error + float(feature_vector(row, axis).dot(weights))
            raw_sse += error * error
            corrected_sse += corrected_error * corrected_error
        improvements.append(math.sqrt(raw_sse / len(trial_rows)) - math.sqrt(corrected_sse / len(trial_rows)))
    return improvements


def fit_maxmin(rows: Sequence[dict[str, object]], axis: str, seed: int) -> list[float]:
    grouped = list(defaultdict(list).items())
    by_trial: dict[tuple[str, int], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        by_trial[(str(row["report_dir"]), int(row["trial"]))].append(row)

    raw_mse: list[float] = []
    quadratic: list[np.ndarray] = []
    linear: list[np.ndarray] = []
    for trial_rows in by_trial.values():
        design = np.asarray([feature_vector(row, axis) for row in trial_rows])
        errors = np.asarray([raw_error(row, axis) for row in trial_rows])
        raw_mse.append(float(np.mean(errors * errors)))
        quadratic.append(design.T @ design / len(trial_rows))
        linear.append(design.T @ errors / len(trial_rows))

    def improvements(weights: np.ndarray) -> np.ndarray:
        values = []
        for raw_value, q_matrix, l_vector in zip(raw_mse, quadratic, linear):
            corrected_mse = raw_value + 2.0 * float(l_vector.dot(weights)) + float(
                weights.dot(q_matrix).dot(weights)
            )
            values.append(math.sqrt(raw_value) - math.sqrt(max(corrected_mse, 0.0)))
        return np.asarray(values)

    bounds = [(-3.0, 3.0), (-0.5, 0.5), (-20.0, 20.0), (-100.0, 100.0), (-5.0, 5.0)]

    def objective(weights: np.ndarray) -> float:
        values = improvements(weights)
        return -float(np.min(values)) + 1.0e-6 * float(np.sum(weights * weights))

    rough = differential_evolution(
        objective,
        bounds,
        maxiter=80,
        popsize=10,
        seed=seed,
        polish=False,
        tol=1.0e-6,
    )
    constraints = [
        {"type": "ineq", "fun": lambda values, index=index: float(improvements(values[:-1])[index] - values[-1])}
        for index in range(len(raw_mse))
    ]
    start = np.r_[rough.x, np.min(improvements(rough.x))]
    result = minimize(
        lambda values: -float(values[-1]) + 1.0e-6 * float(values[:-1].dot(values[:-1])),
        start,
        method="SLSQP",
        bounds=bounds + [(-10.0, 10.0)],
        constraints=constraints,
        options={"maxiter": 2000, "ftol": 1.0e-10},
    )
    return [float(value) for value in result.x[:-1]]


def fit_axis(rows: Sequence[dict[str, object]], axis: str, method: str, seed: int) -> list[float]:
    if method == "ridge":
        return fit_ridge(rows, axis)
    return fit_maxmin(rows, axis, seed)


def evaluate_rows(
    rows: Sequence[dict[str, object]],
    weights_by_axis: dict[str, Sequence[float]],
    *,
    strict_epsilon_m: float,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["report_dir"]), int(row["trial"]))].append(row)

    per_trial: list[dict[str, object]] = []
    min_delta = {axis: float("inf") for axis in AXES}
    mean_delta = {axis: 0.0 for axis in AXES}
    pass_count = 0
    sample_count = 0
    for key, trial_rows in sorted(grouped.items()):
        sums = {f"raw_{axis}": 0.0 for axis in RDF_AXES}
        sums.update({f"corrected_{axis}": 0.0 for axis in RDF_AXES})
        sums["raw_position"] = 0.0
        sums["corrected_position"] = 0.0
        for row in trial_rows:
            sample_count += 1
            raw_errors: dict[str, float] = {}
            corrected_errors: dict[str, float] = {}
            for axis in ("right", "down"):
                weights = np.asarray(weights_by_axis[axis], dtype=float)
                raw_errors[axis] = raw_error(row, axis)
                corrected_errors[axis] = raw_errors[axis] + float(feature_vector(row, axis).dot(weights))
            raw_errors["forward"] = raw_error(row, "forward")
            corrected_errors["forward"] = origin_corrected_error(row, "forward")
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
        pass_count += 1 if strict_pass else 0
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

    total_trials = len(per_trial)
    summary = {
        "report_count": len({str(row["report_dir"]) for row in rows}),
        "trial_count": total_trials,
        "sample_count": sample_count,
        "corrected_sample_count": sample_count * len(RDF_AXES),
        "pass_trial_count": pass_count,
        "total_trial_count": total_trials,
        "corrected_sample_coverage": 1.0,
        **{f"{axis}_min_delta_m": round(min_delta[axis], 12) for axis in AXES},
        **{f"{axis}_mean_delta_m": round(mean_delta[axis] / total_trials, 12) for axis in AXES},
    }
    return summary, per_trial


def evaluate_report_trial_holdout(
    validation_rows: Sequence[dict[str, object]],
    report_dirs: Sequence[Path],
    trials: Sequence[int],
    *,
    fit_method: str,
    strict_epsilon_m: float,
    seed: int,
) -> tuple[dict[str, object], list[dict[str, object]], dict[str, object]]:
    policies: dict[str, object] = {}
    all_rows: list[dict[str, object]] = []
    for report_dir in report_dirs:
        for trial in trials:
            train_rows = [
                row
                for row in validation_rows
                if str(row["report_dir"]) != report_dir.name and int(row["trial"]) != int(trial)
            ]
            eval_rows = [
                row
                for row in validation_rows
                if str(row["report_dir"]) == report_dir.name and int(row["trial"]) == int(trial)
            ]
            weights = {
                "right": fit_axis(train_rows, "right", fit_method, seed + trial),
                "down": fit_axis(train_rows, "down", fit_method, seed + trial + 1000),
            }
            summary, per_trial = evaluate_rows(eval_rows, weights, strict_epsilon_m=strict_epsilon_m)
            all_rows.extend(per_trial)
            policies[f"{report_dir.name}:trial_{trial:02d}"] = weights

    # Rebuild aggregate from per-trial rows to avoid reusing fold-specific weights.
    min_delta = {axis: min(float(row[f"{axis}_rmse_improvement_m"]) for row in all_rows) for axis in AXES}
    mean_delta = {
        axis: sum(float(row[f"{axis}_rmse_improvement_m"]) for row in all_rows) / len(all_rows) for axis in AXES
    }
    sample_count = sum(int(row["sample_count"]) for row in all_rows)
    summary = {
        "report_count": len(report_dirs),
        "trial_count": len(all_rows),
        "sample_count": sample_count,
        "corrected_sample_count": sample_count * len(RDF_AXES),
        "pass_trial_count": sum(1 for row in all_rows if row["pass_strict_rdf_all_axis"] == "yes"),
        "total_trial_count": len(all_rows),
        "corrected_sample_coverage": 1.0,
        **{f"{axis}_min_delta_m": round(min_delta[axis], 12) for axis in AXES},
        **{f"{axis}_mean_delta_m": round(mean_delta[axis], 12) for axis in AXES},
    }
    return summary, all_rows, policies


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
    report_dirs = [path.resolve() for path in args.report_dirs]
    trials = parse_ints(args.validation_trials)
    validation_rows = load_validation_rows(report_dirs, trials)

    if args.mode == "validation_fit":
        fit_rows = validation_rows
        policies = {
            "right": fit_axis(fit_rows, "right", args.fit_method, args.seed),
            "down": fit_axis(fit_rows, "down", args.fit_method, args.seed + 1000),
        }
        summary, per_trial = evaluate_rows(validation_rows, policies, strict_epsilon_m=args.strict_epsilon_m)
        policy_payload: dict[str, object] = {"axis_policies": policies}
    elif args.mode == "train_trial":
        fit_rows = load_train_rows(report_dirs)
        policies = {
            "right": fit_axis(fit_rows, "right", args.fit_method, args.seed),
            "down": fit_axis(fit_rows, "down", args.fit_method, args.seed + 1000),
        }
        summary, per_trial = evaluate_rows(validation_rows, policies, strict_epsilon_m=args.strict_epsilon_m)
        policy_payload = {"axis_policies": policies}
    else:
        summary, per_trial, policy_payload = evaluate_report_trial_holdout(
            validation_rows,
            report_dirs,
            trials,
            fit_method=args.fit_method,
            strict_epsilon_m=args.strict_epsilon_m,
            seed=args.seed,
        )

    metadata = {
        "mode": args.mode,
        "fit_method": args.fit_method,
        "strict_epsilon_m": args.strict_epsilon_m,
        "report_dirs": [str(path) for path in report_dirs],
        "validation_trials": trials,
        "coordinate_contract": "stereo-center camera RDF: right/down/forward meters; position is RDF Euclidean RMSE",
        "feature_contract": [
            "origin_bbox_reprojection_corrected_axis_m - raw_axis_m",
            "raw_axis_m",
            "constant",
            "raw_axis_m / raw_forward_m",
            "raw_disparity_px / 1000",
        ],
        "summary": summary,
    }

    output_dir = args.output_dir
    write_json(output_dir / "metadata.json", metadata)
    write_json(output_dir / "policy.json", policy_payload)
    write_csv(output_dir / "per_trial.csv", per_trial)
    print(f"mode={args.mode}")
    print(f"fit_method={args.fit_method}")
    print(f"pass={summary['pass_trial_count']}/{summary['total_trial_count']}")
    print(
        "mins="
        f"R={summary['right_min_delta_m']} "
        f"D={summary['down_min_delta_m']} "
        f"F={summary['forward_min_delta_m']} "
        f"P={summary['position_min_delta_m']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
