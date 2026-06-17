#!/usr/bin/env python3
"""Search same-row Right signals for ROI Down/Forward fusion.

Coordinate contract:
- Input comparison rows use stereo-center camera RDF axes in meters.
- ROI diagnostic rows already contain corrected RDF error fields.
- Output deltas are RMSE(raw_error_axis) - RMSE(composite_error_axis).
- Position is Euclidean RMSE over Right/Down/Forward in the same RDF frame.
- No NED/ENU/PX4/body conversion is performed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


APP_DIR = Path(__file__).resolve().parents[2] / "apps" / "StereoDistance"
sys.path.insert(0, str(APP_DIR))

from wiris_stereo_distance.tracking_filter import (  # noqa: E402
    ConstantVelocityKalmanFilter,
    TrackingFilterConfig,
    measurement_variance_from_stereo,
)


@dataclass
class Trial:
    trial: int
    timestamps: np.ndarray
    raw_right_m: np.ndarray
    raw_down_m: np.ndarray
    raw_forward_m: np.ndarray
    truth_right_m: np.ndarray
    truth_down_m: np.ndarray
    truth_forward_m: np.ndarray
    raw_right_error_m: np.ndarray
    raw_down_error_m: np.ndarray
    raw_forward_error_m: np.ndarray
    origin_right_error_m: np.ndarray
    roi_down_error_m: np.ndarray
    roi_forward_error_m: np.ndarray


def rmse(values: np.ndarray) -> float:
    return math.sqrt(float(np.mean(values * values)))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def load_trials(report_dir: Path, roi_diagnostics: Path) -> list[Trial]:
    roi_by_trial: dict[int, list[dict[str, str]]] = {}
    for row in read_csv(roi_diagnostics):
        roi_by_trial.setdefault(int(row["trial"]), []).append(row)

    trials: list[Trial] = []
    for trial_id in range(2, 7):
        rows = read_csv(report_dir / f"validation_trial_{trial_id:02d}_comparison.csv")
        roi_rows = roi_by_trial[trial_id]
        if len(rows) != len(roi_rows):
            raise ValueError(
                f"trial {trial_id} row mismatch: comparison={len(rows)} roi={len(roi_rows)}"
            )

        def arr(column: str, source: list[dict[str, str]] = rows) -> np.ndarray:
            return np.array([float(row[column]) for row in source])

        trials.append(
            Trial(
                trial=trial_id,
                timestamps=arr("timestamp_s"),
                raw_right_m=arr("raw_right_m"),
                raw_down_m=arr("raw_down_m"),
                raw_forward_m=arr("raw_forward_m"),
                truth_right_m=arr("truth_right_m"),
                truth_down_m=arr("truth_down_m"),
                truth_forward_m=arr("truth_forward_m"),
                raw_right_error_m=arr("raw_error_right_m"),
                raw_down_error_m=arr("raw_error_down_m"),
                raw_forward_error_m=arr("raw_error_forward_m"),
                origin_right_error_m=arr("origin_bbox_reprojection_error_right_m"),
                roi_down_error_m=arr("corrected_error_down_m", roi_rows),
                roi_forward_error_m=arr("corrected_error_forward_m", roi_rows),
            )
        )
    return trials


def trial_deltas(trial: Trial, right_error: np.ndarray) -> dict[str, object]:
    raw_right = rmse(trial.raw_right_error_m)
    raw_down = rmse(trial.raw_down_error_m)
    raw_forward = rmse(trial.raw_forward_error_m)
    corrected_right = rmse(right_error)
    corrected_down = rmse(trial.roi_down_error_m)
    corrected_forward = rmse(trial.roi_forward_error_m)
    raw_position = math.sqrt(raw_right**2 + raw_down**2 + raw_forward**2)
    corrected_position = math.sqrt(
        corrected_right**2 + corrected_down**2 + corrected_forward**2
    )
    deltas = {
        "right_delta_rmse_m": raw_right - corrected_right,
        "down_delta_rmse_m": raw_down - corrected_down,
        "forward_delta_rmse_m": raw_forward - corrected_forward,
        "position_delta_rmse_m": raw_position - corrected_position,
    }
    return {
        "trial": trial.trial,
        "sample_count": int(trial.raw_right_error_m.size),
        "pass_strict_rdf_all_axis": "yes" if all(value > 0.0 for value in deltas.values()) else "no",
        **deltas,
    }


def aggregate(rows: list[dict[str, object]]) -> dict[str, object]:
    keys = [
        "right_delta_rmse_m",
        "down_delta_rmse_m",
        "forward_delta_rmse_m",
        "position_delta_rmse_m",
    ]
    return {
        "trial_count": len(rows),
        "sample_count": sum(int(row["sample_count"]) for row in rows),
        "pass_trial_count": sum(row["pass_strict_rdf_all_axis"] == "yes" for row in rows),
        **{f"min_{key}": min(float(row[key]) for row in rows) for key in keys},
        **{f"mean_{key}": float(np.mean([float(row[key]) for row in rows])) for key in keys},
    }


def origin_rows(trials: list[Trial], gain: float) -> list[dict[str, object]]:
    rows = []
    for trial in trials:
        right = trial.raw_right_error_m + gain * (
            trial.origin_right_error_m - trial.raw_right_error_m
        )
        row = trial_deltas(trial, right)
        row["right_source"] = "origin_bbox_right_gain"
        row["right_gain"] = gain
        rows.append(row)
    return rows


def bias_rows(trials: list[Trial], bias_m: float) -> list[dict[str, object]]:
    rows = []
    for trial in trials:
        row = trial_deltas(trial, trial.raw_right_error_m - bias_m)
        row["right_source"] = "profile_right_bias"
        row["right_bias_m"] = bias_m
        rows.append(row)
    return rows


def tracking_right_errors(
    trial: Trial,
    config: TrackingFilterConfig,
    *,
    focal_length_px: float,
    baseline_m: float,
    confidence: float,
) -> np.ndarray:
    tracker = ConstantVelocityKalmanFilter(config)
    errors: list[float] = []
    for index in range(trial.raw_right_m.size):
        point = (
            float(trial.raw_right_m[index]),
            float(trial.raw_down_m[index]),
            float(trial.raw_forward_m[index]),
        )
        forward = abs(point[2])
        disparity = focal_length_px * baseline_m / forward if forward > 1.0e-6 else 0.0
        if disparity <= 0.0 or not math.isfinite(disparity):
            variance = (
                config.max_lateral_std_m**2,
                config.max_lateral_std_m**2,
                config.max_forward_std_m**2,
            )
        else:
            variance = measurement_variance_from_stereo(
                point_m=point,
                focal_length_px=focal_length_px,
                baseline_m=baseline_m,
                disparity_px=disparity,
                left_confidence=confidence,
                right_confidence=confidence,
                config=config,
            )
        result = tracker.update(
            timestamp_s=float(trial.timestamps[index]),
            point_m=point,
            measurement_variance_m2=variance,
        )
        errors.append(float(result.point_m[0]) - float(trial.truth_right_m[index]))
    return np.array(errors)


def tracking_rows(
    trials: list[Trial],
    *,
    process_accel_std_mps2: float,
    min_lateral_std_m: float,
    min_forward_std_m: float,
) -> list[dict[str, object]]:
    focal_length_px = (1280.0 * 0.5) / math.tan(0.154997710698 * 0.5)
    config = TrackingFilterConfig(
        process_accel_std_mps2=process_accel_std_mps2,
        initial_position_std_m=25.0,
        initial_velocity_std_mps=20.0,
        reset_after_s=3.0,
        max_normalized_innovation=1.0e9,
        min_lateral_std_m=min_lateral_std_m,
        min_forward_std_m=min_forward_std_m,
        max_lateral_std_m=50.0,
        max_forward_std_m=250.0,
    )
    rows = []
    for trial in trials:
        row = trial_deltas(
            trial,
            tracking_right_errors(
                trial,
                config,
                focal_length_px=focal_length_px,
                baseline_m=25.0,
                confidence=0.5,
            ),
        )
        row["right_source"] = "tracking_raw_right"
        row["process_accel_std_mps2"] = process_accel_std_mps2
        row["min_lateral_std_m"] = min_lateral_std_m
        row["min_forward_std_m"] = min_forward_std_m
        rows.append(row)
    return rows


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--roi-diagnostics", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    trials = load_trials(args.report_dir, args.roi_diagnostics)
    profile_rows = read_csv(args.report_dir / "profile_build_comparison.csv")
    profile_right_errors = np.array([float(row["raw_error_right_m"]) for row in profile_rows])

    candidates: list[tuple[dict[str, object], list[dict[str, object]]]] = []
    for gain in [-0.5, -0.1, 0.0, 0.05, 0.1, 0.2, 0.35, 0.5, 0.75, 1.0]:
        rows = origin_rows(trials, gain)
        candidates.append(({"right_source": "origin_bbox_right_gain", "right_gain": gain, **aggregate(rows)}, rows))
    for bias in [
        float(np.mean(profile_right_errors)),
        float(np.median(profile_right_errors)),
        -0.02,
        -0.01,
        -0.001,
        0.001,
        0.01,
        0.02,
    ]:
        rows = bias_rows(trials, bias)
        candidates.append(({"right_source": "profile_right_bias", "right_bias_m": bias, **aggregate(rows)}, rows))
    for accel in [2.0, 4.0, 8.0, 16.0]:
        for min_lateral in [0.01, 0.05, 0.1, 0.25]:
            rows = tracking_rows(
                trials,
                process_accel_std_mps2=accel,
                min_lateral_std_m=min_lateral,
                min_forward_std_m=0.5,
            )
            candidates.append(
                (
                    {
                        "right_source": "tracking_raw_right",
                        "process_accel_std_mps2": accel,
                        "min_lateral_std_m": min_lateral,
                        "min_forward_std_m": 0.5,
                        **aggregate(rows),
                    },
                    rows,
                )
            )

    def sort_key(item: tuple[dict[str, object], list[dict[str, object]]]) -> tuple[object, ...]:
        summary, _ = item
        min_margin = min(
            float(summary[f"min_{key}"])
            for key in [
                "right_delta_rmse_m",
                "down_delta_rmse_m",
                "forward_delta_rmse_m",
                "position_delta_rmse_m",
            ]
        )
        return (
            int(summary["pass_trial_count"]),
            min_margin,
            float(summary["mean_position_delta_rmse_m"]),
        )

    candidates.sort(key=sort_key, reverse=True)
    summary_rows = [item[0] for item in candidates]
    best_summary, best_rows = candidates[0]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "candidate_summary.csv", summary_rows)
    write_csv(args.output_dir / "best_trial_summary.csv", best_rows)
    metadata = {
        "method": "roi_composite_right_signal_validation",
        "coordinate_contract": {
            "frame": "stereo-center camera RDF",
            "axes": ["right_m", "down_m", "forward_m"],
            "position_metric": "Euclidean RMSE over RDF components",
            "frame_conversion": "none",
        },
        "roi_down_forward_source": str(args.roi_diagnostics),
        "best_candidate": best_summary,
    }
    (args.output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
