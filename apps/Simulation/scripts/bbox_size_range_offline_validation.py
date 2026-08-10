#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


DEFAULT_IMAGE_WIDTH_PX = 1280.0
DEFAULT_HORIZONTAL_FOV_RAD = 0.154997710698


@dataclass(frozen=True)
class Sample:
    truth: tuple[float, float, float]
    raw: tuple[float, float, float]
    left_width_px: float
    right_width_px: float

    @property
    def avg_width_px(self) -> float:
        return 0.5 * (self.left_width_px + self.right_width_px)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a 15m known-size bbox range prior. The method calibrates the "
            "apparent physical width on the profile trajectory, estimates range from "
            "bbox width, and keeps only the raw stereo bearing direction."
        )
    )
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--train-trial", type=int, default=1)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--image-width-px", type=float, default=DEFAULT_IMAGE_WIDTH_PX)
    parser.add_argument("--horizontal-fov-rad", type=float, default=DEFAULT_HORIZONTAL_FOV_RAD)
    parser.add_argument("--blend", type=float, default=1.0)
    parser.add_argument(
        "--train-blend",
        action="store_true",
        help="Select a raw/size blend in [0, 1] from the training trajectory by position RMSE.",
    )
    parser.add_argument("--blend-step", type=float, default=0.02)
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


def first_float(row: dict[str, str], names: Sequence[str]) -> float | None:
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def load_samples(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            truth_right = first_float(row, ("truth_right_m", "gt_right_m"))
            truth_down = first_float(row, ("truth_down_m", "gt_down_m"))
            truth_forward = first_float(row, ("truth_forward_m", "gt_forward_m"))
            raw_right = first_float(row, ("estimated_right_m", "raw_right_m"))
            raw_down = first_float(row, ("estimated_down_m", "raw_down_m"))
            raw_forward = first_float(row, ("estimated_forward_m", "raw_forward_m"))
            left_x1 = first_float(row, ("left_x1_px", "left_x1", "left_bbox_x1_px"))
            left_x2 = first_float(row, ("left_x2_px", "left_x2", "left_bbox_x2_px"))
            right_x1 = first_float(row, ("right_x1_px", "right_x1", "right_bbox_x1_px"))
            right_x2 = first_float(row, ("right_x2_px", "right_x2", "right_bbox_x2_px"))
            if None in (
                truth_right,
                truth_down,
                truth_forward,
                raw_right,
                raw_down,
                raw_forward,
                left_x1,
                left_x2,
                right_x1,
                right_x2,
            ):
                continue
            left_width = float(left_x2) - float(left_x1)
            right_width = float(right_x2) - float(right_x1)
            if left_width <= 1.0 or right_width <= 1.0 or abs(float(raw_forward)) <= 1.0e-6:
                continue
            samples.append(
                Sample(
                    truth=(float(truth_right), float(truth_down), float(truth_forward)),
                    raw=(float(raw_right), float(raw_down), float(raw_forward)),
                    left_width_px=left_width,
                    right_width_px=right_width,
                )
            )
    return samples


def train_apparent_width_m(samples: Sequence[Sample], fx_px: float) -> float:
    widths = [
        sample.avg_width_px * sample.truth[2] / fx_px
        for sample in samples
        if sample.avg_width_px > 1.0 and sample.truth[2] > 1.0
    ]
    if not widths:
        raise RuntimeError("no usable width training samples")
    return float(statistics.median(widths))


def size_point(sample: Sample, fx_px: float, apparent_width_m: float) -> tuple[float, float, float]:
    forward = fx_px * apparent_width_m / sample.avg_width_px
    raw_forward = sample.raw[2]
    right = sample.raw[0] / raw_forward * forward
    down = sample.raw[1] / raw_forward * forward
    return (right, down, forward)


def blended_point(
    raw: tuple[float, float, float],
    size: tuple[float, float, float],
    blend: float,
) -> tuple[float, float, float]:
    weight = max(0.0, min(1.0, float(blend)))
    return tuple((1.0 - weight) * raw[index] + weight * size[index] for index in range(3))  # type: ignore[return-value]


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def evaluate_points(
    samples: Sequence[Sample],
    *,
    fx_px: float,
    apparent_width_m: float,
    blend: float,
) -> tuple[list[dict[str, str]], dict[str, float | int | str]]:
    rows: list[dict[str, str]] = []
    errors = {
        "raw_right": [],
        "raw_down": [],
        "raw_forward": [],
        "size_right": [],
        "size_down": [],
        "size_forward": [],
        "blended_right": [],
        "blended_down": [],
        "blended_forward": [],
    }
    for sample in samples:
        size = size_point(sample, fx_px, apparent_width_m)
        blended = blended_point(sample.raw, size, blend)
        for prefix, point in (("raw", sample.raw), ("size", size), ("blended", blended)):
            for axis, index in (("right", 0), ("down", 1), ("forward", 2)):
                errors[f"{prefix}_{axis}"].append(point[index] - sample.truth[index])
        rows.append(
            {
                "truth_right_m": f"{sample.truth[0]:.9f}",
                "truth_down_m": f"{sample.truth[1]:.9f}",
                "truth_forward_m": f"{sample.truth[2]:.9f}",
                "raw_right_m": f"{sample.raw[0]:.9f}",
                "raw_down_m": f"{sample.raw[1]:.9f}",
                "raw_forward_m": f"{sample.raw[2]:.9f}",
                "size_right_m": f"{size[0]:.9f}",
                "size_down_m": f"{size[1]:.9f}",
                "size_forward_m": f"{size[2]:.9f}",
                "blended_right_m": f"{blended[0]:.9f}",
                "blended_down_m": f"{blended[1]:.9f}",
                "blended_forward_m": f"{blended[2]:.9f}",
                "raw_error_right_m": f"{errors['raw_right'][-1]:.9f}",
                "raw_error_down_m": f"{errors['raw_down'][-1]:.9f}",
                "raw_error_forward_m": f"{errors['raw_forward'][-1]:.9f}",
                "size_error_right_m": f"{errors['size_right'][-1]:.9f}",
                "size_error_down_m": f"{errors['size_down'][-1]:.9f}",
                "size_error_forward_m": f"{errors['size_forward'][-1]:.9f}",
                "blended_error_right_m": f"{errors['blended_right'][-1]:.9f}",
                "blended_error_down_m": f"{errors['blended_down'][-1]:.9f}",
                "blended_error_forward_m": f"{errors['blended_forward'][-1]:.9f}",
            }
        )
    summary: dict[str, float | int | str] = {
        "sample_count": len(samples),
        "evaluated_count": len(rows),
        "apparent_width_m": apparent_width_m,
        "blend": blend,
    }
    pass_size = True
    pass_blended = True
    for axis in ("right", "down", "forward"):
        raw_value = rmse(errors[f"raw_{axis}"])
        size_value = rmse(errors[f"size_{axis}"])
        blended_value = rmse(errors[f"blended_{axis}"])
        summary[f"raw_{axis}_rmse_m"] = raw_value
        summary[f"size_{axis}_rmse_m"] = size_value
        summary[f"blended_{axis}_rmse_m"] = blended_value
        summary[f"size_{axis}_rmse_improvement_m"] = raw_value - size_value
        summary[f"blended_{axis}_rmse_improvement_m"] = raw_value - blended_value
        if not (math.isfinite(raw_value - size_value) and raw_value - size_value >= 0.0):
            pass_size = False
        if not (math.isfinite(raw_value - blended_value) and raw_value - blended_value >= 0.0):
            pass_blended = False
    for prefix in ("raw", "size", "blended"):
        position_errors = [
            math.sqrt(right * right + down * down + forward * forward)
            for right, down, forward in zip(
                errors[f"{prefix}_right"],
                errors[f"{prefix}_down"],
                errors[f"{prefix}_forward"],
            )
        ]
        summary[f"{prefix}_position_rmse_m"] = rmse(position_errors)
    summary["size_position_rmse_improvement_m"] = float(summary["raw_position_rmse_m"]) - float(summary["size_position_rmse_m"])
    summary["blended_position_rmse_improvement_m"] = (
        float(summary["raw_position_rmse_m"]) - float(summary["blended_position_rmse_m"])
    )
    summary["pass_size"] = "yes" if pass_size else "no"
    summary["pass_blended"] = "yes" if pass_blended else "no"
    return rows, summary


def train_blend(samples: Sequence[Sample], *, fx_px: float, apparent_width_m: float, step: float) -> float:
    if step <= 0.0:
        raise ValueError("blend step must be positive")
    best_blend = 0.0
    best_score = float("inf")
    count = int(round(1.0 / step))
    for index in range(count + 1):
        blend = min(1.0, index * step)
        _, summary = evaluate_points(samples, fx_px=fx_px, apparent_width_m=apparent_width_m, blend=blend)
        score = float(summary["blended_position_rmse_m"])
        if score < best_score:
            best_score = score
            best_blend = blend
    return best_blend


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
    fx_px = focal_px(args.image_width_px, args.horizontal_fov_rad)
    normalized_dir = args.report_dir / "normalized"
    train_samples = load_samples(normalized_dir / f"trial_{args.train_trial:02d}.csv")
    if not train_samples:
        raise RuntimeError("no training samples")
    apparent_width_m = train_apparent_width_m(train_samples, fx_px)
    blend = train_blend(train_samples, fx_px=fx_px, apparent_width_m=apparent_width_m, step=args.blend_step) if args.train_blend else args.blend

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = args.report_dir / "bbox_size_range_validation"
    output_dir.mkdir(parents=True, exist_ok=True)
    write_json(
        output_dir / "bbox_size_range_profile.json",
        {
            "method": "bbox_known_size_range_prior_v1",
            "source_report_dir": str(args.report_dir),
            "train_trial": args.train_trial,
            "train_sample_count": len(train_samples),
            "apparent_width_m": apparent_width_m,
            "blend": blend,
            "focal_length_px": fx_px,
            "contract": "range from bbox average width; right/down use raw stereo bearing; optional raw/size linear blend",
        },
    )

    gate_rows: list[dict[str, str]] = []
    final_size = True
    final_blended = True
    for trial in validation_trials(args.validation_trials):
        samples = load_samples(normalized_dir / f"trial_{trial:02d}.csv")
        rows, summary = evaluate_points(samples, fx_px=fx_px, apparent_width_m=apparent_width_m, blend=blend)
        write_csv(output_dir / f"validation_trial_{trial:02d}_size_range_comparison.csv", rows)
        write_json(output_dir / f"validation_trial_{trial:02d}_summary.json", summary)
        if summary["pass_size"] != "yes":
            final_size = False
        if summary["pass_blended"] != "yes":
            final_blended = False
        gate_rows.append(
            {
                "trial": str(trial),
                "sample_count": str(summary["sample_count"]),
                "evaluated_count": str(summary["evaluated_count"]),
                "size_right_rmse_improvement_m": f"{float(summary['size_right_rmse_improvement_m']):.9f}",
                "size_down_rmse_improvement_m": f"{float(summary['size_down_rmse_improvement_m']):.9f}",
                "size_forward_rmse_improvement_m": f"{float(summary['size_forward_rmse_improvement_m']):.9f}",
                "size_position_rmse_improvement_m": f"{float(summary['size_position_rmse_improvement_m']):.9f}",
                "blended_right_rmse_improvement_m": f"{float(summary['blended_right_rmse_improvement_m']):.9f}",
                "blended_down_rmse_improvement_m": f"{float(summary['blended_down_rmse_improvement_m']):.9f}",
                "blended_forward_rmse_improvement_m": f"{float(summary['blended_forward_rmse_improvement_m']):.9f}",
                "blended_position_rmse_improvement_m": f"{float(summary['blended_position_rmse_improvement_m']):.9f}",
                "pass_size": str(summary["pass_size"]),
                "pass_blended": str(summary["pass_blended"]),
            }
        )
    write_csv(output_dir / "validation_gate_summary.csv", gate_rows)
    (output_dir / "result.txt").write_text(
        f"final_pass_size={'yes' if final_size else 'no'}\n"
        f"final_pass_blended={'yes' if final_blended else 'no'}\n"
        f"profile={output_dir / 'bbox_size_range_profile.json'}\n"
        f"summary={output_dir / 'validation_gate_summary.csv'}\n",
        encoding="utf-8",
    )
    print(f"Profile: {output_dir / 'bbox_size_range_profile.json'}")
    print(f"Summary: {output_dir / 'validation_gate_summary.csv'}")
    print(f"Final pass size: {'yes' if final_size else 'no'}")
    print(f"Final pass blended: {'yes' if final_blended else 'no'}")
    return 0 if final_blended else 1


if __name__ == "__main__":
    raise SystemExit(main())
