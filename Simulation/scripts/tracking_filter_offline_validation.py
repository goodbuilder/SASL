#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


def _find_stereo_distance_app() -> Path:
    if os.environ.get("STEREO_DISTANCE_APP"):
        return Path(os.environ["STEREO_DISTANCE_APP"]).expanduser()
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "StereoDistance"
        if (candidate / "wiris_stereo_distance").is_dir():
            return candidate
    return Path(__file__).resolve().parents[2] / "StereoDistance"


APP_DIR = _find_stereo_distance_app()
sys.path.insert(0, str(APP_DIR))

from wiris_stereo_distance.tracking_filter import (  # noqa: E402
    ConstantVelocityKalmanFilter,
    TrackingFilterConfig,
    measurement_variance_from_stereo,
)


DEFAULT_IMAGE_WIDTH_PX = 1280.0
DEFAULT_BASELINE_M = 25.0
DEFAULT_HORIZONTAL_FOV_RAD = 0.154997710698


@dataclass(frozen=True)
class PointSample:
    trial_t_s: float
    truth: tuple[float, float, float]
    raw: tuple[float, float, float]
    measurement: tuple[float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Apply the StereoDistance constant-velocity Kalman tracker offline to "
            "bbox-anchor validation CSVs and report per-trial RDF RMSE gates."
        )
    )
    parser.add_argument("anchor_result_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--source", choices=("anchor", "raw", "center_rot"), default="anchor")
    parser.add_argument("--image-width-px", type=float, default=DEFAULT_IMAGE_WIDTH_PX)
    parser.add_argument("--baseline-m", type=float, default=DEFAULT_BASELINE_M)
    parser.add_argument("--horizontal-fov-rad", type=float, default=DEFAULT_HORIZONTAL_FOV_RAD)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--process-accel-std-mps2", type=float, default=4.0)
    parser.add_argument("--initial-position-std-m", type=float, default=25.0)
    parser.add_argument("--initial-velocity-std-mps", type=float, default=20.0)
    parser.add_argument("--reset-after-s", type=float, default=3.0)
    parser.add_argument("--max-normalized-innovation", type=float, default=12.0)
    parser.add_argument("--min-lateral-std-m", type=float, default=0.25)
    parser.add_argument("--min-forward-std-m", type=float, default=2.0)
    parser.add_argument("--max-lateral-std-m", type=float, default=50.0)
    parser.add_argument("--max-forward-std-m", type=float, default=250.0)
    return parser.parse_args()


def validation_trials(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def focal_px(width_px: float, horizontal_fov_rad: float) -> float:
    return (float(width_px) * 0.5) / math.tan(float(horizontal_fov_rad) * 0.5)


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def point_from_row(row: dict[str, str], prefix: str) -> tuple[float, float, float] | None:
    right = finite_float(row.get(f"{prefix}_right_m"))
    down = finite_float(row.get(f"{prefix}_down_m"))
    forward = finite_float(row.get(f"{prefix}_forward_m"))
    if right is None or down is None or forward is None:
        return None
    return (right, down, forward)


def load_samples(path: Path, source: str) -> list[PointSample]:
    samples: list[PointSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            timestamp = finite_float(row.get("trial_t_s"))
            truth = point_from_row(row, "truth")
            raw = point_from_row(row, "raw")
            measurement = point_from_row(row, source)
            if timestamp is None or truth is None or raw is None or measurement is None:
                continue
            samples.append(PointSample(timestamp, truth, raw, measurement))
    return samples


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def position_errors(axis_errors: dict[str, list[float]], prefix: str) -> list[float]:
    return [
        math.sqrt(right * right + down * down + forward * forward)
        for right, down, forward in zip(
            axis_errors[f"{prefix}_right"],
            axis_errors[f"{prefix}_down"],
            axis_errors[f"{prefix}_forward"],
        )
    ]


def variance_for_point(
    point: tuple[float, float, float],
    *,
    focal_length_px: float,
    baseline_m: float,
    confidence: float,
    config: TrackingFilterConfig,
) -> tuple[float, float, float]:
    forward = abs(point[2])
    disparity = focal_length_px * baseline_m / forward if forward > 1.0e-6 else 0.0
    if disparity <= 0.0 or not math.isfinite(disparity):
        return (
            config.max_lateral_std_m**2,
            config.max_lateral_std_m**2,
            config.max_forward_std_m**2,
        )
    return measurement_variance_from_stereo(
        point_m=point,
        focal_length_px=focal_length_px,
        baseline_m=baseline_m,
        disparity_px=disparity,
        left_confidence=confidence,
        right_confidence=confidence,
        config=config,
    )


def evaluate_trial(
    samples: Sequence[PointSample],
    *,
    source: str,
    focal_length_px: float,
    baseline_m: float,
    confidence: float,
    config: TrackingFilterConfig,
) -> tuple[list[dict[str, str]], dict[str, float | int | str]]:
    tracker = ConstantVelocityKalmanFilter(config)
    rows: list[dict[str, str]] = []
    errors: dict[str, list[float]] = {
        "raw_right": [],
        "raw_down": [],
        "raw_forward": [],
        "measurement_right": [],
        "measurement_down": [],
        "measurement_forward": [],
        "filtered_right": [],
        "filtered_down": [],
        "filtered_forward": [],
    }
    accepted = 0
    rejected = 0
    resets = 0
    for sample in samples:
        variance = variance_for_point(
            sample.measurement,
            focal_length_px=focal_length_px,
            baseline_m=baseline_m,
            confidence=confidence,
            config=config,
        )
        result = tracker.update(
            timestamp_s=sample.trial_t_s,
            point_m=sample.measurement,
            measurement_variance_m2=variance,
        )
        if result.accepted:
            accepted += 1
        else:
            rejected += 1
        if result.reset:
            resets += 1

        filtered = result.point_m
        for prefix, point in (
            ("raw", sample.raw),
            ("measurement", sample.measurement),
            ("filtered", filtered),
        ):
            for axis, index in (("right", 0), ("down", 1), ("forward", 2)):
                errors[f"{prefix}_{axis}"].append(point[index] - sample.truth[index])

        rows.append(
            {
                "trial_t_s": f"{sample.trial_t_s:.6f}",
                "truth_right_m": f"{sample.truth[0]:.9f}",
                "truth_down_m": f"{sample.truth[1]:.9f}",
                "truth_forward_m": f"{sample.truth[2]:.9f}",
                "raw_right_m": f"{sample.raw[0]:.9f}",
                "raw_down_m": f"{sample.raw[1]:.9f}",
                "raw_forward_m": f"{sample.raw[2]:.9f}",
                f"{source}_right_m": f"{sample.measurement[0]:.9f}",
                f"{source}_down_m": f"{sample.measurement[1]:.9f}",
                f"{source}_forward_m": f"{sample.measurement[2]:.9f}",
                "filtered_right_m": f"{filtered[0]:.9f}",
                "filtered_down_m": f"{filtered[1]:.9f}",
                "filtered_forward_m": f"{filtered[2]:.9f}",
                "raw_error_right_m": f"{errors['raw_right'][-1]:.9f}",
                "raw_error_down_m": f"{errors['raw_down'][-1]:.9f}",
                "raw_error_forward_m": f"{errors['raw_forward'][-1]:.9f}",
                f"{source}_error_right_m": f"{errors['measurement_right'][-1]:.9f}",
                f"{source}_error_down_m": f"{errors['measurement_down'][-1]:.9f}",
                f"{source}_error_forward_m": f"{errors['measurement_forward'][-1]:.9f}",
                "filtered_error_right_m": f"{errors['filtered_right'][-1]:.9f}",
                "filtered_error_down_m": f"{errors['filtered_down'][-1]:.9f}",
                "filtered_error_forward_m": f"{errors['filtered_forward'][-1]:.9f}",
                "tracking_accepted": "yes" if result.accepted else "no",
                "tracking_reset": "yes" if result.reset else "no",
                "tracking_normalized_innovation": f"{result.normalized_innovation:.9f}",
                "tracking_std_right_m": f"{result.measurement_std_m[0]:.9f}",
                "tracking_std_down_m": f"{result.measurement_std_m[1]:.9f}",
                "tracking_std_forward_m": f"{result.measurement_std_m[2]:.9f}",
                "tracking_velocity_right_mps": f"{result.velocity_mps[0]:.9f}",
                "tracking_velocity_down_mps": f"{result.velocity_mps[1]:.9f}",
                "tracking_velocity_forward_mps": f"{result.velocity_mps[2]:.9f}",
            }
        )

    summary: dict[str, float | int | str] = {
        "source": source,
        "sample_count": len(samples),
        "evaluated_count": len(rows),
        "tracking_accepted_count": accepted,
        "tracking_rejected_count": rejected,
        "tracking_reset_count": resets,
    }
    pass_vs_raw = True
    pass_vs_measurement = True
    for axis in ("right", "down", "forward"):
        raw_value = rmse(errors[f"raw_{axis}"])
        measurement_value = rmse(errors[f"measurement_{axis}"])
        filtered_value = rmse(errors[f"filtered_{axis}"])
        summary[f"raw_{axis}_rmse_m"] = raw_value
        summary[f"{source}_{axis}_rmse_m"] = measurement_value
        summary[f"filtered_{axis}_rmse_m"] = filtered_value
        summary[f"{source}_{axis}_rmse_improvement_m"] = raw_value - measurement_value
        summary[f"filtered_vs_raw_{axis}_rmse_improvement_m"] = raw_value - filtered_value
        summary[f"filtered_vs_{source}_{axis}_rmse_improvement_m"] = measurement_value - filtered_value
        if not (math.isfinite(raw_value - filtered_value) and raw_value - filtered_value >= 0.0):
            pass_vs_raw = False
        if not (math.isfinite(measurement_value - filtered_value) and measurement_value - filtered_value >= 0.0):
            pass_vs_measurement = False

    for prefix in ("raw", "measurement", "filtered"):
        summary[f"{prefix}_position_rmse_m"] = rmse(position_errors(errors, prefix))
    summary[f"{source}_position_rmse_improvement_m"] = (
        float(summary["raw_position_rmse_m"]) - float(summary["measurement_position_rmse_m"])
    )
    summary["filtered_vs_raw_position_rmse_improvement_m"] = (
        float(summary["raw_position_rmse_m"]) - float(summary["filtered_position_rmse_m"])
    )
    summary[f"filtered_vs_{source}_position_rmse_improvement_m"] = (
        float(summary["measurement_position_rmse_m"]) - float(summary["filtered_position_rmse_m"])
    )
    summary["pass_vs_raw"] = "yes" if pass_vs_raw else "no"
    summary[f"pass_vs_{source}"] = "yes" if pass_vs_measurement else "no"
    return rows, summary


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


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = args.anchor_result_dir.parent / f"{args.anchor_result_dir.name}_tracking_{args.source}"
    output_dir.mkdir(parents=True, exist_ok=True)

    config = TrackingFilterConfig(
        process_accel_std_mps2=float(args.process_accel_std_mps2),
        initial_position_std_m=float(args.initial_position_std_m),
        initial_velocity_std_mps=float(args.initial_velocity_std_mps),
        reset_after_s=float(args.reset_after_s),
        max_normalized_innovation=float(args.max_normalized_innovation),
        min_lateral_std_m=float(args.min_lateral_std_m),
        min_forward_std_m=float(args.min_forward_std_m),
        max_lateral_std_m=float(args.max_lateral_std_m),
        max_forward_std_m=float(args.max_forward_std_m),
    )
    fx_px = focal_px(args.image_width_px, args.horizontal_fov_rad)
    gate_rows: list[dict[str, object]] = []
    final_pass = True
    final_pass_vs_source = True
    for trial in validation_trials(args.validation_trials):
        input_path = args.anchor_result_dir / f"validation_trial_{trial:02d}_anchor_comparison.csv"
        samples = load_samples(input_path, args.source)
        if not samples:
            raise RuntimeError(f"no samples in {input_path}")
        rows, summary = evaluate_trial(
            samples,
            source=args.source,
            focal_length_px=fx_px,
            baseline_m=float(args.baseline_m),
            confidence=float(args.confidence),
            config=config,
        )
        if summary["pass_vs_raw"] != "yes":
            final_pass = False
        if summary[f"pass_vs_{args.source}"] != "yes":
            final_pass_vs_source = False
        write_csv(output_dir / f"validation_trial_{trial:02d}_tracking_comparison.csv", rows)
        write_json(output_dir / f"validation_trial_{trial:02d}_summary.json", summary)
        gate_rows.append(
            {
                "trial": trial,
                "sample_count": summary["sample_count"],
                "evaluated_count": summary["evaluated_count"],
                "filtered_vs_raw_right_rmse_improvement_m": f"{float(summary['filtered_vs_raw_right_rmse_improvement_m']):.9f}",
                "filtered_vs_raw_down_rmse_improvement_m": f"{float(summary['filtered_vs_raw_down_rmse_improvement_m']):.9f}",
                "filtered_vs_raw_forward_rmse_improvement_m": f"{float(summary['filtered_vs_raw_forward_rmse_improvement_m']):.9f}",
                "filtered_vs_raw_position_rmse_improvement_m": f"{float(summary['filtered_vs_raw_position_rmse_improvement_m']):.9f}",
                f"filtered_vs_{args.source}_right_rmse_improvement_m": f"{float(summary[f'filtered_vs_{args.source}_right_rmse_improvement_m']):.9f}",
                f"filtered_vs_{args.source}_down_rmse_improvement_m": f"{float(summary[f'filtered_vs_{args.source}_down_rmse_improvement_m']):.9f}",
                f"filtered_vs_{args.source}_forward_rmse_improvement_m": f"{float(summary[f'filtered_vs_{args.source}_forward_rmse_improvement_m']):.9f}",
                f"filtered_vs_{args.source}_position_rmse_improvement_m": f"{float(summary[f'filtered_vs_{args.source}_position_rmse_improvement_m']):.9f}",
                "tracking_rejected_count": summary["tracking_rejected_count"],
                "tracking_reset_count": summary["tracking_reset_count"],
                "pass_vs_raw": summary["pass_vs_raw"],
                f"pass_vs_{args.source}": summary[f"pass_vs_{args.source}"],
            }
        )
    write_csv(output_dir / "validation_gate_summary.csv", gate_rows)
    write_json(
        output_dir / "tracking_filter_config.json",
        {
            "source": args.source,
            "anchor_result_dir": str(args.anchor_result_dir),
            "config": config.__dict__,
            "confidence": float(args.confidence),
            "focal_length_px": fx_px,
            "baseline_m": float(args.baseline_m),
            "final_pass_vs_raw": "yes" if final_pass else "no",
            f"final_pass_vs_{args.source}": "yes" if final_pass_vs_source else "no",
        },
    )
    (output_dir / "result.txt").write_text(
        "\n".join(
            [
                f"final_pass_vs_raw={'yes' if final_pass else 'no'}",
                f"final_pass_vs_{args.source}={'yes' if final_pass_vs_source else 'no'}",
                f"summary={output_dir / 'validation_gate_summary.csv'}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
