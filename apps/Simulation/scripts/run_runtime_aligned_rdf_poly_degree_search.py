#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import rtk_stereo_calibration_pipeline as pipe  # noqa: E402


RDF_AXIS_CONTRACT = pipe.RDF_AXIS_CONTRACT
NED_AXIS_CONTRACT = pipe.NED_AXIS_CONTRACT
PUBLIC_TOPIC_CONTRACT = pipe.STEREODISTANCE_PUBLIC_TOPIC_CONTRACT
NED_TO_RDF_CONTRACT = pipe.SYNTHETIC_NED_TO_RDF_CONTRACT


def parse_int_list(value: str) -> list[int]:
    items = [item.strip() for item in value.split(",") if item.strip()]
    if not items:
        raise argparse.ArgumentTypeError("list must contain at least one integer")
    return [int(item) for item in items]


def parse_float_list(value: str) -> list[float]:
    items = [item.strip() for item in value.split(",") if item.strip()]
    if not items:
        raise argparse.ArgumentTypeError("list must contain at least one float")
    return [float(item) for item in items]


def value_label(value: float) -> str:
    return f"{float(value):g}".replace("-", "m").replace(".", "p")


def zscore(values: np.ndarray, *, mean: float, std: float) -> np.ndarray:
    scale = std if math.isfinite(std) and std > 1.0e-12 else 1.0
    return (np.asarray(values, dtype=float) - float(mean)) / scale


def ridge_polyfit_fixed_degree(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    degree: int,
    alpha: float,
) -> np.ndarray:
    x_values = np.asarray(x_values, dtype=float)
    y_values = np.asarray(y_values, dtype=float)
    valid = np.isfinite(x_values) & np.isfinite(y_values)
    if int(np.sum(valid)) <= int(degree):
        return np.zeros(int(degree) + 1, dtype=float)
    return pipe.fit_ridge_polyfit(x_values[valid], y_values[valid], degree=int(degree), alpha=float(alpha))


def apply_axis_residual(
    sim_points: np.ndarray,
    x_values: np.ndarray,
    coefficients_by_axis: dict[str, np.ndarray],
    *,
    forward_clamp_m: float,
) -> tuple[np.ndarray, int]:
    corrected = np.array(sim_points, dtype=float, copy=True)
    clamped_count = 0
    for axis_index, axis_name in enumerate(("right", "down", "forward")):
        residual = np.polyval(coefficients_by_axis[axis_name], x_values)
        if axis_name == "forward" and math.isfinite(forward_clamp_m) and forward_clamp_m > 0.0:
            before = residual.copy()
            residual = np.clip(residual, -float(forward_clamp_m), float(forward_clamp_m))
            clamped_count += int(np.sum(np.abs(before - residual) > 1.0e-12))
        corrected[:, axis_index] += residual
    return corrected, clamped_count


def build_samples(
    *,
    seed: int,
    sample_count: int,
    target_range_m: float,
    target_window_m: float,
    target_oversample_count: int,
) -> list[pipe.CalibrationSample]:
    source = pipe.SyntheticCalibrationDataSource(
        camera=pipe.CameraModel(),
        injection=pipe.ErrorInjection(),
        sample_count=sample_count,
        seed=seed,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
        target_oversample_count=target_oversample_count,
    )
    return source.load_samples()


def fit_and_evaluate_seed(
    *,
    seed: int,
    train_sample_count: int,
    validation_sample_count: int,
    target_range_m: float,
    target_window_m: float,
    target_oversample_count: int,
    right_degree: int,
    down_degree: int,
    forward_degrees: Sequence[int],
    ridge_alpha: float,
    huber_delta_m: float,
    forward_clamp_m: float,
) -> tuple[list[dict[str, object]], dict[str, np.ndarray]]:
    sample_count = int(train_sample_count) + int(validation_sample_count)
    samples = build_samples(
        seed=seed,
        sample_count=sample_count,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
        target_oversample_count=target_oversample_count,
    )
    train, validation = pipe.split_samples(
        samples,
        train_ratio=float(train_sample_count) / float(sample_count),
        seed=seed + 17,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    raw_train = pipe.stack(train, "raw_rdf_m")
    truth_train = pipe.stack(train, "truth_rdf_m")
    raw_validation = pipe.stack(validation, "raw_rdf_m")
    d_train = pipe.measured_disparity(train)
    d_validation = pipe.measured_disparity(validation)
    d_mean = float(np.mean(d_train))
    d_std = float(np.std(d_train))
    x_train = zscore(d_train, mean=d_mean, std=d_std)
    x_validation = zscore(d_validation, mean=d_mean, std=d_std)

    scale, rotation, translation, weights = pipe.fit_huber_umeyama_sim3(
        raw_train,
        truth_train,
        delta_m=float(huber_delta_m),
    )
    sim_train = pipe.predict_sim3(raw_train, scale, rotation, translation)
    sim_validation = pipe.predict_sim3(raw_validation, scale, rotation, translation)
    residual_train = truth_train - sim_train
    raw_metrics = pipe.evaluate_predictions(
        validation,
        raw_validation,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )

    rows: list[dict[str, object]] = []
    predictions: dict[str, np.ndarray] = {}
    for forward_degree in forward_degrees:
        method = (
            f"rdf_d{right_degree}_d{down_degree}_d{forward_degree}"
            f"_huber{value_label(huber_delta_m)}_ridge{value_label(ridge_alpha)}"
            f"_clamp{value_label(forward_clamp_m)}"
        )
        coeffs = {
            "right": ridge_polyfit_fixed_degree(
                x_train,
                residual_train[:, 0],
                degree=right_degree,
                alpha=ridge_alpha,
            ),
            "down": ridge_polyfit_fixed_degree(
                x_train,
                residual_train[:, 1],
                degree=down_degree,
                alpha=ridge_alpha,
            ),
            "forward": ridge_polyfit_fixed_degree(
                x_train,
                residual_train[:, 2],
                degree=forward_degree,
                alpha=ridge_alpha,
            ),
        }
        corrected, clamped_count = apply_axis_residual(
            sim_validation,
            x_validation,
            coeffs,
            forward_clamp_m=forward_clamp_m,
        )
        metrics = pipe.evaluate_predictions(
            validation,
            corrected,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
        )
        predictions[method] = corrected
        rows.append(
            {
                "seed": seed,
                "method": method,
                "right_degree": right_degree,
                "down_degree": down_degree,
                "forward_degree": forward_degree,
                "ridge_alpha": ridge_alpha,
                "huber_delta_m": huber_delta_m,
                "forward_clamp_m": forward_clamp_m,
                "train_sample_count": len(train),
                "validation_sample_count": len(validation),
                "runtime_truth_used": False,
                "fallback_count": 0,
                "clamped_count": clamped_count,
                "mean_huber_weight": float(np.mean(weights)),
                "min_huber_weight": float(np.min(weights)),
                "raw_range_rmse_m": raw_metrics["range_rmse_m"],
                "raw_position_rmse_m": raw_metrics["position_rmse_m"],
                "range_rmse_m": metrics["range_rmse_m"],
                "position_rmse_m": metrics["position_rmse_m"],
                "right_rmse_m": metrics["right_rmse_m"],
                "down_rmse_m": metrics["down_rmse_m"],
                "forward_rmse_m": metrics["forward_rmse_m"],
                "target_count": metrics["target_window_count"],
                "target_abs_mean_error_m": metrics["target_abs_mean_error_m"],
                "target_rmse_m": metrics["target_rmse_m"],
                "target_max_abs_error_m": metrics["target_max_abs_error_m"],
            }
        )
    return rows, predictions


def aggregate(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    by_method: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        by_method.setdefault(str(row["method"]), []).append(row)
    summary = []
    for method, method_rows in sorted(by_method.items()):
        def mean_field(name: str) -> float:
            values = [float(row[name]) for row in method_rows]
            return float(np.mean(values)) if values else float("nan")

        def max_field(name: str) -> float:
            values = [float(row[name]) for row in method_rows]
            return float(np.max(values)) if values else float("nan")

        first = method_rows[0]
        pass_count = sum(
            1
            for row in method_rows
            if float(row["range_rmse_m"]) < float(row["raw_range_rmse_m"])
            and float(row["position_rmse_m"]) < float(row["raw_position_rmse_m"])
            and float(row["target_max_abs_error_m"]) <= 10.0
            and int(row["target_count"]) > 0
            and int(row["fallback_count"]) == 0
            and not bool(row["runtime_truth_used"])
        )
        summary.append(
            {
                "method": method,
                "status": "STRICT_PASS" if pass_count == len(method_rows) else "FAIL",
                "pass_count": pass_count,
                "trial_count": len(method_rows),
                "right_degree": first["right_degree"],
                "down_degree": first["down_degree"],
                "forward_degree": first["forward_degree"],
                "ridge_alpha": first["ridge_alpha"],
                "huber_delta_m": first["huber_delta_m"],
                "forward_clamp_m": first["forward_clamp_m"],
                "range_rmse_mean_m": mean_field("range_rmse_m"),
                "range_rmse_max_m": max_field("range_rmse_m"),
                "position_rmse_mean_m": mean_field("position_rmse_m"),
                "position_rmse_max_m": max_field("position_rmse_m"),
                "right_rmse_mean_m": mean_field("right_rmse_m"),
                "down_rmse_mean_m": mean_field("down_rmse_m"),
                "forward_rmse_mean_m": mean_field("forward_rmse_m"),
                "target_rmse_mean_m": mean_field("target_rmse_m"),
                "target_rmse_max_m": max_field("target_rmse_m"),
                "target_max_abs_error_max_m": max_field("target_max_abs_error_m"),
                "target_abs_mean_error_mean_m": mean_field("target_abs_mean_error_m"),
                "clamped_count_total": int(sum(int(row["clamped_count"]) for row in method_rows)),
                "runtime_truth_used": False,
            }
        )
    return sorted(
        summary,
        key=lambda row: (
            0 if row["status"] == "STRICT_PASS" else 1,
            float(row["range_rmse_mean_m"]),
            float(row["position_rmse_mean_m"]),
            float(row["target_rmse_mean_m"]),
            int(row["forward_degree"]),
        ),
    )


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(pipe.json_safe(payload), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Runtime-aligned RDF-axis polynomial residual degree search."
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--seed-start", type=int, default=20292859)
    parser.add_argument("--trial-count", type=int, default=300)
    parser.add_argument("--train-sample-count", type=int, default=1260)
    parser.add_argument("--validation-sample-count", type=int, default=420)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    parser.add_argument("--right-degree", type=int, default=6)
    parser.add_argument("--down-degree", type=int, default=6)
    parser.add_argument("--forward-degrees", type=parse_int_list, default=parse_int_list("14,16,18"))
    parser.add_argument("--ridge-alphas", type=parse_float_list, default=parse_float_list("0.1"))
    parser.add_argument("--huber-deltas", type=parse_float_list, default=parse_float_list("8.0"))
    parser.add_argument("--forward-clamp-m", type=float, default=2.0)
    parser.add_argument(
        "--forward-clamps",
        type=parse_float_list,
        default=None,
        help="Optional comma-separated forward residual clamp sweep. Overrides --forward-clamp-m.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    seed_rows: list[dict[str, object]] = []
    forward_clamps = args.forward_clamps if args.forward_clamps is not None else [float(args.forward_clamp_m)]
    for offset in range(int(args.trial_count)):
        seed = int(args.seed_start) + offset
        for huber_delta in args.huber_deltas:
            for ridge_alpha in args.ridge_alphas:
                for forward_clamp in forward_clamps:
                    rows, _ = fit_and_evaluate_seed(
                        seed=seed,
                        train_sample_count=args.train_sample_count,
                        validation_sample_count=args.validation_sample_count,
                        target_range_m=args.target_range_m,
                        target_window_m=args.target_window_m,
                        target_oversample_count=args.target_oversample_count,
                        right_degree=args.right_degree,
                        down_degree=args.down_degree,
                        forward_degrees=args.forward_degrees,
                        ridge_alpha=ridge_alpha,
                        huber_delta_m=huber_delta,
                        forward_clamp_m=forward_clamp,
                    )
                    seed_rows.extend(rows)

    summary_rows = aggregate(seed_rows)
    best = summary_rows[0] if summary_rows else {}
    metrics = {
        "experiment_id": args.experiment_id,
        "status": best.get("status", "FAIL"),
        "stage": "runtime_aligned_rdf_polynomial_degree_search",
        "seed_start": args.seed_start,
        "seed_end": args.seed_start + args.trial_count - 1,
        "trial_count": args.trial_count,
        "train_sample_count": args.train_sample_count,
        "validation_sample_count_per_seed": args.validation_sample_count,
        "coordinate_contracts_explicit": True,
        "px4_truth_frame": "px4_local_position_ned",
        "px4_truth_axis_order": "north,east,down",
        "internal_correction_frame": "stereo_center_opencv_optical",
        "internal_correction_axis_order": "right,down,forward",
        "public_stereodistance_topic_order": "forward,right,down,range",
        "truth_rdf_transform_contract": NED_TO_RDF_CONTRACT,
        "ned_to_rdf_contract": NED_TO_RDF_CONTRACT,
        "runtime_truth_used": False,
        "best_method": best,
        "methods": summary_rows,
    }
    write_csv(args.output_dir / "seed_metrics.csv", seed_rows)
    write_csv(args.output_dir / "method_summary.csv", summary_rows)
    write_json(args.output_dir / "metrics.json", metrics)
    lines = [
        f"# {args.experiment_id}",
        "",
        "Purpose: runtime-aligned RDF-axis polynomial residual degree search for StereoDistance deployment.",
        "",
        (
            "Method: fit calibration-time Huber Sim(3) from raw stereo-center RDF to truth RDF, "
            "then fit per-axis ridge polynomial residuals over measured disparity. Runtime uses only "
            "raw RDF, measured disparity, and fixed fitted parameters."
        ),
        "",
        (
            "Coordinate contract: PX4/local NED truth `north/east/down` is converted to stereo-center "
            "OpenCV optical RDF by an explicit T_source_to_stereo_rdf; direct NED-to-RDF axis aliasing is forbidden. Internal "
            "correction is RDF `right/down/forward`; public StereoDistance topics remain FRD+range "
            "`forward/right/down/range`."
        ),
        "",
        (
            "Pass/fail criteria: STRICT_PASS if corrected validation range RMSE and position RMSE beat raw, "
            "target-window coverage is nonzero, target max absolute range error is within +/-10m, "
            "fallback count is zero, and runtime truth is unused."
        ),
        "",
        (
            f"Result: {metrics['status']}. Best `{best.get('method', 'none')}` range RMSE "
            f"{float(best.get('range_rmse_mean_m', float('nan'))):.9f} m, position RMSE "
            f"{float(best.get('position_rmse_mean_m', float('nan'))):.9f} m, target RMSE "
            f"{float(best.get('target_rmse_mean_m', float('nan'))):.9f} m."
        ),
    ]
    (args.output_dir / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (args.output_dir / "command.md").write_text(
        "# Command\n\nGenerated by `apps/Simulation/scripts/run_runtime_aligned_rdf_poly_degree_search.py`.\n",
        encoding="utf-8",
    )
    print(json.dumps(pipe.json_safe(metrics), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
