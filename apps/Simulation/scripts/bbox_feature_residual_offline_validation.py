#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


DEFAULT_IMAGE_WIDTH_PX = 1280.0
DEFAULT_IMAGE_HEIGHT_PX = 720.0


@dataclass(frozen=True)
class Sample:
    truth: tuple[float, float, float]  # right, down, forward
    raw: tuple[float, float, float]  # right, down, forward
    features: dict[str, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Train/evaluate bbox feature residual profiles without using bbox center as "
            "the corrected image observation."
        )
    )
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--train-trial", type=int, default=1)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--image-width-px", type=float, default=DEFAULT_IMAGE_WIDTH_PX)
    parser.add_argument("--image-height-px", type=float, default=DEFAULT_IMAGE_HEIGHT_PX)
    parser.add_argument("--ridge-lambdas", default="0.001,0.01,0.1,1,10,100,1000")
    parser.add_argument("--blend-grid", default="0,0.02,0.05,0.1,0.2,0.35,0.5,0.75,1.0")
    return parser.parse_args()


def validation_trials(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def float_list(text: str) -> list[float]:
    return [float(part.strip()) for part in text.split(",") if part.strip()]


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def first_float(row: dict[str, str], names: Sequence[str]) -> float | None:
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def feature_base(row: dict[str, str], width_px: float, height_px: float) -> dict[str, float] | None:
    left_x1 = first_float(row, ("left_x1_px", "left_x1", "left_bbox_x1_px"))
    left_y1 = first_float(row, ("left_y1_px", "left_y1", "left_bbox_y1_px"))
    left_x2 = first_float(row, ("left_x2_px", "left_x2", "left_bbox_x2_px"))
    left_y2 = first_float(row, ("left_y2_px", "left_y2", "left_bbox_y2_px"))
    right_x1 = first_float(row, ("right_x1_px", "right_x1", "right_bbox_x1_px"))
    right_y1 = first_float(row, ("right_y1_px", "right_y1", "right_bbox_y1_px"))
    right_x2 = first_float(row, ("right_x2_px", "right_x2", "right_bbox_x2_px"))
    right_y2 = first_float(row, ("right_y2_px", "right_y2", "right_bbox_y2_px"))
    raw_disparity = first_float(row, ("raw_disparity_px",))
    if None in (left_x1, left_y1, left_x2, left_y2, right_x1, right_y1, right_x2, right_y2):
        return None
    left_width = float(left_x2) - float(left_x1)
    left_height = float(left_y2) - float(left_y1)
    right_width = float(right_x2) - float(right_x1)
    right_height = float(right_y2) - float(right_y1)
    if left_width <= 1.0 or left_height <= 1.0 or right_width <= 1.0 or right_height <= 1.0:
        return None
    if raw_disparity is None:
        left_cx = 0.5 * (float(left_x1) + float(left_x2))
        right_cx = 0.5 * (float(right_x1) + float(right_x2))
        raw_disparity = abs(left_cx - right_cx)
    avg_width = 0.5 * (left_width + right_width)
    avg_height = 0.5 * (left_height + right_height)
    left_area = left_width * left_height
    right_area = right_width * right_height
    avg_area = 0.5 * (left_area + right_area)
    confidence_left = first_float(row, ("left_confidence",))
    confidence_right = first_float(row, ("right_confidence",))
    confidence_min = min(
        1.0 if confidence_left is None else float(confidence_left),
        1.0 if confidence_right is None else float(confidence_right),
    )
    return {
        "bias": 1.0,
        "left_width_norm": left_width / width_px,
        "left_height_norm": left_height / height_px,
        "right_width_norm": right_width / width_px,
        "right_height_norm": right_height / height_px,
        "avg_width_norm": avg_width / width_px,
        "avg_height_norm": avg_height / height_px,
        "avg_area_norm": avg_area / (width_px * height_px),
        "width_ratio": left_width / max(right_width, 1.0e-6),
        "height_ratio": left_height / max(right_height, 1.0e-6),
        "area_ratio": left_area / max(right_area, 1.0e-6),
        "disparity_norm": abs(float(raw_disparity)) / width_px,
        "width_over_disparity": avg_width / max(abs(float(raw_disparity)), 1.0e-6),
        "height_over_disparity": avg_height / max(abs(float(raw_disparity)), 1.0e-6),
        "confidence_min": confidence_min,
    }


def load_samples(path: Path, *, width_px: float, height_px: float) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            truth_right = first_float(row, ("truth_right_m", "gt_right_m"))
            truth_down = first_float(row, ("truth_down_m", "gt_down_m"))
            truth_forward = first_float(row, ("truth_forward_m", "gt_forward_m"))
            raw_right = first_float(row, ("estimated_right_m", "raw_right_m"))
            raw_down = first_float(row, ("estimated_down_m", "raw_down_m"))
            raw_forward = first_float(row, ("estimated_forward_m", "raw_forward_m"))
            features = feature_base(row, width_px, height_px)
            if None in (truth_right, truth_down, truth_forward, raw_right, raw_down, raw_forward) or features is None:
                continue
            raw = (float(raw_right), float(raw_down), float(raw_forward))
            raw_range = math.sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2])
            features = {
                **features,
                "raw_right_m": raw[0],
                "raw_down_m": raw[1],
                "raw_forward_m": raw[2],
                "raw_range_m": raw_range,
            }
            samples.append(
                Sample(
                    truth=(float(truth_right), float(truth_down), float(truth_forward)),
                    raw=raw,
                    features=features,
                )
            )
    return samples


def feature_sets() -> dict[str, list[str]]:
    size_shape = [
        "bias",
        "left_width_norm",
        "left_height_norm",
        "right_width_norm",
        "right_height_norm",
        "avg_width_norm",
        "avg_height_norm",
        "avg_area_norm",
        "width_ratio",
        "height_ratio",
        "area_ratio",
        "disparity_norm",
        "width_over_disparity",
        "height_over_disparity",
        "confidence_min",
    ]
    compact = [
        "bias",
        "avg_width_norm",
        "avg_height_norm",
        "avg_area_norm",
        "width_ratio",
        "height_ratio",
        "disparity_norm",
        "width_over_disparity",
        "confidence_min",
    ]
    return {
        "size_shape": size_shape,
        "size_shape_compact": compact,
        "size_shape_raw_range": size_shape + ["raw_forward_m", "raw_range_m"],
        "size_shape_raw_rdf": size_shape + ["raw_right_m", "raw_down_m", "raw_forward_m", "raw_range_m"],
    }


def matrices(samples: Sequence[Sample], names: Sequence[str]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    x = np.array([[sample.features[name] for name in names] for sample in samples], dtype=float)
    truth = np.array([sample.truth for sample in samples], dtype=float)
    raw = np.array([sample.raw for sample in samples], dtype=float)
    residual = truth - raw
    return x, raw, residual


def fit_ridge(samples: Sequence[Sample], names: Sequence[str], ridge_lambda: float) -> dict[str, np.ndarray]:
    x_raw, _, residual = matrices(samples, names)
    mean = np.mean(x_raw, axis=0)
    scale = np.std(x_raw, axis=0)
    for index, name in enumerate(names):
        if name == "bias":
            mean[index] = 0.0
            scale[index] = 1.0
    scale = np.where(np.abs(scale) < 1.0e-12, 1.0, scale)
    x = (x_raw - mean) / scale
    regularizer = float(ridge_lambda) * np.eye(x.shape[1])
    for index, name in enumerate(names):
        if name == "bias":
            regularizer[index, index] = 0.0
    coefficients = np.linalg.solve(x.T @ x + regularizer, x.T @ residual).T
    return {"mean": mean, "scale": scale, "coefficients": coefficients}


def predict_residual(samples: Sequence[Sample], names: Sequence[str], model: dict[str, np.ndarray]) -> np.ndarray:
    x_raw, _, _ = matrices(samples, names)
    x = (x_raw - model["mean"]) / model["scale"]
    return x @ model["coefficients"].T


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if values.size else float("nan")


def choose_blend(train_samples: Sequence[Sample], names: Sequence[str], model: dict[str, np.ndarray], grid: Sequence[float]) -> tuple[float, float, float]:
    _, raw, _ = matrices(train_samples, names)
    truth = np.array([sample.truth for sample in train_samples], dtype=float)
    residual = predict_residual(train_samples, names, model)
    blends: list[float] = []
    for axis in range(3):
        best_blend = 0.0
        best_rmse = rmse(raw[:, axis] - truth[:, axis])
        for blend in grid:
            corrected = raw[:, axis] + float(blend) * residual[:, axis]
            value = rmse(corrected - truth[:, axis])
            if math.isfinite(value) and value < best_rmse:
                best_rmse = value
                best_blend = float(blend)
        blends.append(best_blend)
    return (blends[0], blends[1], blends[2])


def evaluate(
    samples: Sequence[Sample],
    names: Sequence[str],
    model: dict[str, np.ndarray],
    blend: tuple[float, float, float],
) -> dict[str, float | int | str]:
    _, raw, _ = matrices(samples, names)
    truth = np.array([sample.truth for sample in samples], dtype=float)
    residual = predict_residual(samples, names, model)
    corrected = raw + np.array(blend, dtype=float) * residual
    summary: dict[str, float | int | str] = {
        "sample_count": len(samples),
        "evaluated_count": len(samples),
    }
    pass_nonregression = True
    pass_strict = True
    for axis_name, axis in (("right", 0), ("down", 1), ("forward", 2)):
        raw_rmse = rmse(raw[:, axis] - truth[:, axis])
        corrected_rmse = rmse(corrected[:, axis] - truth[:, axis])
        improvement = raw_rmse - corrected_rmse
        summary[f"raw_{axis_name}_rmse_m"] = raw_rmse
        summary[f"corrected_{axis_name}_rmse_m"] = corrected_rmse
        summary[f"{axis_name}_rmse_improvement_m"] = improvement
        if not (math.isfinite(improvement) and improvement >= 0.0):
            pass_nonregression = False
        if not (math.isfinite(improvement) and improvement > 0.0):
            pass_strict = False
    raw_position = np.sqrt(np.sum(np.square(raw - truth), axis=1))
    corrected_position = np.sqrt(np.sum(np.square(corrected - truth), axis=1))
    summary["raw_position_rmse_m"] = rmse(raw_position)
    summary["corrected_position_rmse_m"] = rmse(corrected_position)
    summary["position_rmse_improvement_m"] = float(summary["raw_position_rmse_m"]) - float(
        summary["corrected_position_rmse_m"]
    )
    summary["pass_nonregression"] = "yes" if pass_nonregression else "no"
    summary["pass_strict_improvement"] = "yes" if pass_strict else "no"
    return summary


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


def aggregate(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str], list[dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault((str(row["feature_set"]), str(row["ridge_lambda"])), []).append(row)
    output: list[dict[str, object]] = []
    for (feature_set, ridge_lambda), items in sorted(grouped.items()):
        output.append(
            {
                "feature_set": feature_set,
                "ridge_lambda": ridge_lambda,
                "trial_count": len(items),
                "pass_nonregression_all_trials": "yes"
                if all(item["pass_nonregression"] == "yes" for item in items)
                else "no",
                "pass_strict_all_trials": "yes"
                if all(item["pass_strict_improvement"] == "yes" for item in items)
                else "no",
                "min_right_improvement_m": min(float(item["right_rmse_improvement_m"]) for item in items),
                "min_down_improvement_m": min(float(item["down_rmse_improvement_m"]) for item in items),
                "min_forward_improvement_m": min(float(item["forward_rmse_improvement_m"]) for item in items),
                "mean_right_improvement_m": statistics.fmean(float(item["right_rmse_improvement_m"]) for item in items),
                "mean_down_improvement_m": statistics.fmean(float(item["down_rmse_improvement_m"]) for item in items),
                "mean_forward_improvement_m": statistics.fmean(float(item["forward_rmse_improvement_m"]) for item in items),
                "mean_position_improvement_m": statistics.fmean(
                    float(item["position_rmse_improvement_m"]) for item in items
                ),
                "right_blend": items[0]["right_blend"],
                "down_blend": items[0]["down_blend"],
                "forward_blend": items[0]["forward_blend"],
            }
        )
    return output


def main() -> int:
    args = parse_args()
    normalized_dir = args.report_dir / "normalized"
    output_dir = args.output_dir or Path("reports") / f"bbox_feature_residual_{dt.datetime.now():%Y%m%d_%H%M%S}"
    train_samples = load_samples(
        normalized_dir / f"trial_{args.train_trial:02d}.csv",
        width_px=float(args.image_width_px),
        height_px=float(args.image_height_px),
    )
    if not train_samples:
        raise RuntimeError("no training samples")
    rows: list[dict[str, object]] = []
    for feature_set_name, names in feature_sets().items():
        for ridge_lambda in float_list(args.ridge_lambdas):
            try:
                model = fit_ridge(train_samples, names, ridge_lambda)
            except np.linalg.LinAlgError:
                continue
            blend = choose_blend(train_samples, names, model, float_list(args.blend_grid))
            for trial in validation_trials(args.validation_trials):
                samples = load_samples(
                    normalized_dir / f"trial_{trial:02d}.csv",
                    width_px=float(args.image_width_px),
                    height_px=float(args.image_height_px),
                )
                summary = evaluate(samples, names, model, blend)
                rows.append(
                    {
                        "trial": trial,
                        "feature_set": feature_set_name,
                        "feature_count": len(names),
                        "ridge_lambda": ridge_lambda,
                        "right_blend": blend[0],
                        "down_blend": blend[1],
                        "forward_blend": blend[2],
                        **summary,
                    }
                )
    aggregate_rows = aggregate(rows)
    write_csv(output_dir / "trial_summary.csv", rows)
    write_csv(output_dir / "candidate_aggregate_summary.csv", aggregate_rows)
    write_json(
        output_dir / "metadata.json",
        {
            "source_report_dir": str(args.report_dir),
            "train_trial": args.train_trial,
            "validation_trials": validation_trials(args.validation_trials),
            "contract": (
                "Feature residual profiles use bbox size/shape/disparity fields; "
                "size_shape* variants do not use bbox center position as an image observation."
            ),
        },
    )
    pass_count = sum(1 for row in aggregate_rows if row["pass_nonregression_all_trials"] == "yes")
    strict_count = sum(1 for row in aggregate_rows if row["pass_strict_all_trials"] == "yes")
    print(f"Output: {output_dir}")
    print(f"Aggregate rows: {output_dir / 'candidate_aggregate_summary.csv'}")
    print(f"Non-regression candidates: {pass_count}")
    print(f"Strict-improvement candidates: {strict_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
