#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import bbox_anchor_offline_validation as anchor_base
import roi_silhouette_offline_validation as roi


@dataclass(frozen=True)
class PreparedSample:
    truth: tuple[float, float, float]
    raw: tuple[float, float, float]
    triangulated: tuple[float, float, float] | None
    fallback: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep conservative axis gains for ROI silhouette triangulation. "
            "ROI images are parsed once per mode/trial, and invalid ROI samples "
            "fall back to raw no-op."
        )
    )
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--modes", default="centroid,bbox_center,top_center,bottom_center")
    parser.add_argument("--right-gains", default="0.0")
    parser.add_argument("--down-gains", default="0.0,0.1,0.25,0.5,0.75,1.0")
    parser.add_argument("--forward-gains", default="0.0,0.25,0.5,0.75,1.0")
    parser.add_argument("--image-width-px", type=float, default=roi.DEFAULT_IMAGE_WIDTH_PX)
    parser.add_argument("--image-height-px", type=float, default=roi.DEFAULT_IMAGE_HEIGHT_PX)
    parser.add_argument("--baseline-m", type=float, default=roi.DEFAULT_BASELINE_M)
    parser.add_argument("--horizontal-fov-rad", type=float, default=roi.DEFAULT_HORIZONTAL_FOV_RAD)
    parser.add_argument("--roi-crop-margin", type=float, default=0.25)
    parser.add_argument("--min-contrast", type=float, default=18.0)
    parser.add_argument("--min-mask-pixels", type=int, default=20)
    parser.add_argument("--max-mask-fraction", type=float, default=0.85)
    return parser.parse_args()


def parse_int_list(text: str) -> list[int]:
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_float_list(text: str) -> list[float]:
    return [float(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_str_list(text: str) -> list[str]:
    return [part.strip() for part in str(text).split(",") if part.strip()]


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def prepare_samples(
    samples: Sequence[roi.Sample],
    *,
    mode: str,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
    args: argparse.Namespace,
) -> list[PreparedSample]:
    prepared: list[PreparedSample] = []
    for sample in samples:
        left_point = roi.image_point(sample.left_roi_path, sample.left_bbox, mode=mode, args=args)
        right_point = roi.image_point(sample.right_roi_path, sample.right_bbox, mode=mode, args=args)
        fallback = "none"
        triangulated: tuple[float, float, float] | None = None
        if left_point is None or right_point is None:
            fallback = "invalid_or_missing_roi"
        else:
            triangulated = anchor_base.triangulate_with_rotation(left_point, right_point, camera, rotation)
            if triangulated is None:
                fallback = "triangulation_failed"
        prepared.append(
            PreparedSample(
                truth=sample.truth,
                raw=sample.raw,
                triangulated=triangulated,
                fallback=fallback,
            )
        )
    return prepared


def evaluate_prepared(
    prepared: Sequence[PreparedSample],
    *,
    right_gain: float,
    down_gain: float,
    forward_gain: float,
) -> dict[str, object]:
    gains = (float(right_gain), float(down_gain), float(forward_gain))
    raw_errors: list[list[float]] = [[], [], []]
    corrected_errors: list[list[float]] = [[], [], []]
    raw_position_errors: list[float] = []
    corrected_position_errors: list[float] = []
    fallback_counts: dict[str, int] = {}
    for sample in prepared:
        corrected = sample.raw
        fallback_counts[sample.fallback] = fallback_counts.get(sample.fallback, 0) + 1
        if sample.triangulated is not None:
            corrected = tuple(
                sample.raw[index] + gains[index] * (sample.triangulated[index] - sample.raw[index])
                for index in range(3)
            )
        raw_error = tuple(sample.raw[index] - sample.truth[index] for index in range(3))
        corrected_error = tuple(corrected[index] - sample.truth[index] for index in range(3))
        for index in range(3):
            raw_errors[index].append(raw_error[index])
            corrected_errors[index].append(corrected_error[index])
        raw_position_errors.append(math.sqrt(sum(value * value for value in raw_error)))
        corrected_position_errors.append(math.sqrt(sum(value * value for value in corrected_error)))
    summary: dict[str, object] = {
        "sample_count": len(prepared),
        "fallback_none_count": fallback_counts.get("none", 0),
        "fallback_invalid_or_missing_roi_count": fallback_counts.get("invalid_or_missing_roi", 0),
        "fallback_triangulation_failed_count": fallback_counts.get("triangulation_failed", 0),
    }
    pass_nonregression = True
    pass_strict = True
    for axis, index in (("right", 0), ("down", 1), ("forward", 2)):
        raw_value = rmse(raw_errors[index])
        corrected_value = rmse(corrected_errors[index])
        improvement = raw_value - corrected_value
        summary[f"raw_{axis}_rmse_m"] = raw_value
        summary[f"corrected_{axis}_rmse_m"] = corrected_value
        summary[f"{axis}_rmse_improvement_m"] = improvement
        if not (math.isfinite(improvement) and improvement >= 0.0):
            pass_nonregression = False
        if not (math.isfinite(improvement) and improvement > 0.0):
            pass_strict = False
    raw_position = rmse(raw_position_errors)
    corrected_position = rmse(corrected_position_errors)
    summary["raw_position_rmse_m"] = raw_position
    summary["corrected_position_rmse_m"] = corrected_position
    summary["position_rmse_improvement_m"] = raw_position - corrected_position
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


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir or args.report_dir / f"roi_silhouette_axis_gain_sweep_{dt.datetime.now():%Y%m%d_%H%M%S}"
    trials = parse_int_list(args.validation_trials)
    modes = parse_str_list(args.modes)
    right_gains = parse_float_list(args.right_gains)
    down_gains = parse_float_list(args.down_gains)
    forward_gains = parse_float_list(args.forward_gains)
    camera = anchor_base.Camera(
        width_px=float(args.image_width_px),
        height_px=float(args.image_height_px),
        fx_px=anchor_base.focal_px(args.image_width_px, args.horizontal_fov_rad),
        baseline_m=float(args.baseline_m),
    )
    rotation = anchor_base.rotation_matrix_from_report(
        argparse.Namespace(report_dir=args.report_dir, right_optical_rotation_deg="")
    )
    normalized_dir = args.report_dir / "normalized"
    trial_rows: list[dict[str, object]] = []
    candidate_accumulator: dict[tuple[str, float, float, float], list[dict[str, object]]] = {}
    for mode in modes:
        for trial in trials:
            samples = roi.load_samples(normalized_dir / f"trial_{trial:02d}.csv", args.report_dir)
            prepared = prepare_samples(samples, mode=mode, camera=camera, rotation=rotation, args=args)
            for right_gain in right_gains:
                for down_gain in down_gains:
                    for forward_gain in forward_gains:
                        summary = evaluate_prepared(
                            prepared,
                            right_gain=right_gain,
                            down_gain=down_gain,
                            forward_gain=forward_gain,
                        )
                        row = {
                            "trial": trial,
                            "mode": mode,
                            "right_gain": f"{right_gain:.6g}",
                            "down_gain": f"{down_gain:.6g}",
                            "forward_gain": f"{forward_gain:.6g}",
                            "sample_count": summary["sample_count"],
                            "right_rmse_improvement_m": f"{float(summary['right_rmse_improvement_m']):.9f}",
                            "down_rmse_improvement_m": f"{float(summary['down_rmse_improvement_m']):.9f}",
                            "forward_rmse_improvement_m": f"{float(summary['forward_rmse_improvement_m']):.9f}",
                            "position_rmse_improvement_m": f"{float(summary['position_rmse_improvement_m']):.9f}",
                            "fallback_none_count": summary["fallback_none_count"],
                            "fallback_invalid_or_missing_roi_count": summary[
                                "fallback_invalid_or_missing_roi_count"
                            ],
                            "fallback_triangulation_failed_count": summary["fallback_triangulation_failed_count"],
                            "pass_nonregression": summary["pass_nonregression"],
                            "pass_strict_improvement": summary["pass_strict_improvement"],
                        }
                        trial_rows.append(row)
                        key = (mode, right_gain, down_gain, forward_gain)
                        candidate_accumulator.setdefault(key, []).append(row)
    candidate_rows: list[dict[str, object]] = []
    for (mode, right_gain, down_gain, forward_gain), rows in candidate_accumulator.items():
        improvements = {
            axis: [float(row[f"{axis}_rmse_improvement_m"]) for row in rows]
            for axis in ("right", "down", "forward", "position")
        }
        pass_count = sum(row["pass_nonregression"] == "yes" for row in rows)
        strict_count = sum(row["pass_strict_improvement"] == "yes" for row in rows)
        candidate_rows.append(
            {
                "mode": mode,
                "right_gain": f"{right_gain:.6g}",
                "down_gain": f"{down_gain:.6g}",
                "forward_gain": f"{forward_gain:.6g}",
                "trial_count": len(rows),
                "pass_nonregression_count": pass_count,
                "pass_strict_improvement_count": strict_count,
                "pass_all_nonregression": "yes" if pass_count == len(rows) else "no",
                "min_right_rmse_improvement_m": f"{min(improvements['right']):.9f}",
                "mean_right_rmse_improvement_m": f"{sum(improvements['right']) / len(rows):.9f}",
                "min_down_rmse_improvement_m": f"{min(improvements['down']):.9f}",
                "mean_down_rmse_improvement_m": f"{sum(improvements['down']) / len(rows):.9f}",
                "min_forward_rmse_improvement_m": f"{min(improvements['forward']):.9f}",
                "mean_forward_rmse_improvement_m": f"{sum(improvements['forward']) / len(rows):.9f}",
                "min_position_rmse_improvement_m": f"{min(improvements['position']):.9f}",
                "mean_position_rmse_improvement_m": f"{sum(improvements['position']) / len(rows):.9f}",
                "fallback_none_count": sum(int(row["fallback_none_count"]) for row in rows),
                "fallback_invalid_or_missing_roi_count": sum(
                    int(row["fallback_invalid_or_missing_roi_count"]) for row in rows
                ),
                "fallback_triangulation_failed_count": sum(
                    int(row["fallback_triangulation_failed_count"]) for row in rows
                ),
            }
        )
    candidate_rows.sort(
        key=lambda row: (
            row["pass_all_nonregression"] == "yes",
            int(row["pass_nonregression_count"]),
            float(row["min_down_rmse_improvement_m"]),
            float(row["min_forward_rmse_improvement_m"]),
            float(row["mean_position_rmse_improvement_m"]),
        ),
        reverse=True,
    )
    write_csv(output_dir / "axis_gain_trial_summary.csv", trial_rows)
    write_csv(output_dir / "axis_gain_candidate_summary.csv", candidate_rows)
    write_json(
        output_dir / "metadata.json",
        {
            "source_report_dir": str(args.report_dir),
            "validation_trials": trials,
            "modes": modes,
            "right_gains": right_gains,
            "down_gains": down_gains,
            "forward_gains": forward_gains,
            "roi_crop_margin": args.roi_crop_margin,
            "min_contrast": args.min_contrast,
            "min_mask_pixels": args.min_mask_pixels,
            "max_mask_fraction": args.max_mask_fraction,
            "contract": "invalid ROI and triangulation failures are raw no-op fallbacks",
        },
    )
    print(f"Trial summary: {output_dir / 'axis_gain_trial_summary.csv'}")
    print(f"Candidate summary: {output_dir / 'axis_gain_candidate_summary.csv'}")
    if candidate_rows:
        best = candidate_rows[0]
        print(
            "Best: "
            f"mode={best['mode']} gains=({best['right_gain']},{best['down_gain']},{best['forward_gain']}) "
            f"pass_all={best['pass_all_nonregression']} "
            f"min_rdf=({best['min_right_rmse_improvement_m']},"
            f"{best['min_down_rmse_improvement_m']},"
            f"{best['min_forward_rmse_improvement_m']})"
        )
        return 0 if best["pass_all_nonregression"] == "yes" else 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
