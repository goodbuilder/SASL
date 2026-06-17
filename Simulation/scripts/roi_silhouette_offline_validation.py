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

import numpy as np

import bbox_anchor_offline_validation as anchor_base


DEFAULT_IMAGE_WIDTH_PX = 1280.0
DEFAULT_IMAGE_HEIGHT_PX = 720.0
DEFAULT_BASELINE_M = 25.0
DEFAULT_HORIZONTAL_FOV_RAD = 0.154997710698


@dataclass(frozen=True)
class Sample:
    trial_t_s: float | None
    truth: tuple[float, float, float]
    raw: tuple[float, float, float]
    left_bbox: tuple[float, float, float, float]
    right_bbox: tuple[float, float, float, float]
    left_roi_path: Path | None
    right_roi_path: Path | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate image ROI silhouette observation points instead of YOLO bbox centers. "
            "Invalid/low-contrast ROI samples fall back to raw no-op."
        )
    )
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--image-width-px", type=float, default=DEFAULT_IMAGE_WIDTH_PX)
    parser.add_argument("--image-height-px", type=float, default=DEFAULT_IMAGE_HEIGHT_PX)
    parser.add_argument("--baseline-m", type=float, default=DEFAULT_BASELINE_M)
    parser.add_argument("--horizontal-fov-rad", type=float, default=DEFAULT_HORIZONTAL_FOV_RAD)
    parser.add_argument("--roi-crop-margin", type=float, default=0.25)
    parser.add_argument("--min-contrast", type=float, default=18.0)
    parser.add_argument("--min-mask-pixels", type=int, default=20)
    parser.add_argument("--max-mask-fraction", type=float, default=0.85)
    return parser.parse_args()


def validation_trials(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


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


def resolve_path(report_dir: Path, text: str | None) -> Path | None:
    if not text:
        return None
    path = Path(text)
    if path.is_absolute():
        return path
    candidates = [
        path,
        report_dir / path,
        report_dir.parent.parent / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return path


def load_samples(path: Path, report_dir: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            truth_right = first_float(row, ("truth_right_m", "gt_right_m"))
            truth_down = first_float(row, ("truth_down_m", "gt_down_m"))
            truth_forward = first_float(row, ("truth_forward_m", "gt_forward_m"))
            raw_right = first_float(row, ("estimated_right_m", "raw_right_m"))
            raw_down = first_float(row, ("estimated_down_m", "raw_down_m"))
            raw_forward = first_float(row, ("estimated_forward_m", "raw_forward_m"))
            left_x1 = first_float(row, ("left_x1_px", "left_bbox_x1_px"))
            left_y1 = first_float(row, ("left_y1_px", "left_bbox_y1_px"))
            left_x2 = first_float(row, ("left_x2_px", "left_bbox_x2_px"))
            left_y2 = first_float(row, ("left_y2_px", "left_bbox_y2_px"))
            right_x1 = first_float(row, ("right_x1_px", "right_bbox_x1_px"))
            right_y1 = first_float(row, ("right_y1_px", "right_bbox_y1_px"))
            right_x2 = first_float(row, ("right_x2_px", "right_bbox_x2_px"))
            right_y2 = first_float(row, ("right_y2_px", "right_bbox_y2_px"))
            if None in (
                truth_right,
                truth_down,
                truth_forward,
                raw_right,
                raw_down,
                raw_forward,
                left_x1,
                left_y1,
                left_x2,
                left_y2,
                right_x1,
                right_y1,
                right_x2,
                right_y2,
            ):
                continue
            samples.append(
                Sample(
                    trial_t_s=first_float(row, ("trial_t_s",)),
                    truth=(float(truth_right), float(truth_down), float(truth_forward)),
                    raw=(float(raw_right), float(raw_down), float(raw_forward)),
                    left_bbox=(float(left_x1), float(left_y1), float(left_x2), float(left_y2)),
                    right_bbox=(float(right_x1), float(right_y1), float(right_x2), float(right_y2)),
                    left_roi_path=resolve_path(report_dir, row.get("left_roi_path")),
                    right_roi_path=resolve_path(report_dir, row.get("right_roi_path")),
                )
            )
    return samples


def read_ppm(path: Path) -> np.ndarray | None:
    data = path.read_bytes()
    if not data.startswith(b"P6\n"):
        return None
    index = 3
    while data[index : index + 1] == b"#":
        index = data.index(b"\n", index) + 1
    end = data.index(b"\n", index)
    width, height = (int(part) for part in data[index:end].split())
    index = end + 1
    end = data.index(b"\n", index)
    max_value = int(data[index:end])
    if max_value != 255:
        return None
    index = end + 1
    expected = width * height * 3
    pixels = np.frombuffer(data[index : index + expected], dtype=np.uint8)
    if pixels.size != expected:
        return None
    return pixels.reshape(height, width, 3)


def read_image(path: Path | None) -> np.ndarray | None:
    if path is None or not path.exists():
        return None
    if path.suffix.lower() == ".ppm":
        return read_ppm(path)
    try:
        import cv2

        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    except Exception:
        return None
    if image is None:
        return None
    return image[:, :, ::-1]


def crop_origin(
    bbox: tuple[float, float, float, float],
    *,
    image_width_px: float,
    image_height_px: float,
    margin: float,
) -> tuple[float, float]:
    x1, y1, x2, y2 = bbox
    width = x2 - x1
    height = y2 - y1
    return (
        max(0.0, math.floor(x1 - width * margin)),
        max(0.0, math.floor(y1 - height * margin)),
    )


def silhouette_mask(
    image: np.ndarray,
    *,
    min_contrast: float,
    min_mask_pixels: int,
    max_mask_fraction: float,
) -> np.ndarray | None:
    gray = image.astype(np.float32).mean(axis=2)
    border = np.concatenate([gray[0, :], gray[-1, :], gray[:, 0], gray[:, -1]])
    background = float(np.median(border))
    dark = background - gray
    mask = dark >= float(min_contrast)
    count = int(mask.sum())
    if count < int(min_mask_pixels):
        return None
    if count / float(mask.size) > float(max_mask_fraction):
        return None
    return mask


def mask_point(mask: np.ndarray, mode: str) -> tuple[float, float] | None:
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    x_min = float(xs.min())
    x_max = float(xs.max())
    y_min = float(ys.min())
    y_max = float(ys.max())
    if mode == "centroid":
        return (float(xs.mean()), float(ys.mean()))
    if mode == "bbox_center":
        return ((x_min + x_max) * 0.5, (y_min + y_max) * 0.5)
    if mode == "top_center":
        return ((x_min + x_max) * 0.5, y_min)
    if mode == "bottom_center":
        return ((x_min + x_max) * 0.5, y_max)
    raise ValueError(f"unsupported mode: {mode}")


def image_point(
    path: Path | None,
    bbox: tuple[float, float, float, float],
    *,
    mode: str,
    args: argparse.Namespace,
) -> tuple[float, float] | None:
    image = read_image(path)
    if image is None:
        return None
    mask = silhouette_mask(
        image,
        min_contrast=float(args.min_contrast),
        min_mask_pixels=int(args.min_mask_pixels),
        max_mask_fraction=float(args.max_mask_fraction),
    )
    if mask is None:
        return None
    point = mask_point(mask, mode)
    if point is None:
        return None
    origin_x, origin_y = crop_origin(
        bbox,
        image_width_px=float(args.image_width_px),
        image_height_px=float(args.image_height_px),
        margin=float(args.roi_crop_margin),
    )
    return (origin_x + point[0], origin_y + point[1])


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def evaluate_mode(
    samples: Sequence[Sample],
    *,
    mode: str,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
    args: argparse.Namespace,
) -> tuple[list[dict[str, str]], dict[str, object]]:
    rows: list[dict[str, str]] = []
    errors: dict[str, list[float]] = {
        "raw_right": [],
        "raw_down": [],
        "raw_forward": [],
        "corrected_right": [],
        "corrected_down": [],
        "corrected_forward": [],
    }
    fallback_counts: dict[str, int] = {}
    for sample in samples:
        left_point = image_point(sample.left_roi_path, sample.left_bbox, mode=mode, args=args)
        right_point = image_point(sample.right_roi_path, sample.right_bbox, mode=mode, args=args)
        fallback = "none"
        corrected = sample.raw
        if left_point is None or right_point is None:
            fallback = "invalid_or_missing_roi"
        else:
            triangulated = anchor_base.triangulate_with_rotation(left_point, right_point, camera, rotation)
            if triangulated is None:
                fallback = "triangulation_failed"
            else:
                corrected = triangulated
        fallback_counts[fallback] = fallback_counts.get(fallback, 0) + 1
        raw_error = tuple(sample.raw[index] - sample.truth[index] for index in range(3))
        corrected_error = tuple(corrected[index] - sample.truth[index] for index in range(3))
        for axis, index in (("right", 0), ("down", 1), ("forward", 2)):
            errors[f"raw_{axis}"].append(raw_error[index])
            errors[f"corrected_{axis}"].append(corrected_error[index])
        rows.append(
            {
                "trial_t_s": "" if sample.trial_t_s is None else f"{sample.trial_t_s:.6f}",
                "mode": mode,
                "fallback": fallback,
                "truth_right_m": f"{sample.truth[0]:.9f}",
                "truth_down_m": f"{sample.truth[1]:.9f}",
                "truth_forward_m": f"{sample.truth[2]:.9f}",
                "raw_right_m": f"{sample.raw[0]:.9f}",
                "raw_down_m": f"{sample.raw[1]:.9f}",
                "raw_forward_m": f"{sample.raw[2]:.9f}",
                "corrected_right_m": f"{corrected[0]:.9f}",
                "corrected_down_m": f"{corrected[1]:.9f}",
                "corrected_forward_m": f"{corrected[2]:.9f}",
                "raw_error_right_m": f"{raw_error[0]:.9f}",
                "raw_error_down_m": f"{raw_error[1]:.9f}",
                "raw_error_forward_m": f"{raw_error[2]:.9f}",
                "corrected_error_right_m": f"{corrected_error[0]:.9f}",
                "corrected_error_down_m": f"{corrected_error[1]:.9f}",
                "corrected_error_forward_m": f"{corrected_error[2]:.9f}",
            }
        )
    summary: dict[str, object] = {
        "sample_count": len(samples),
        "evaluated_count": len(rows),
        "mode": mode,
    }
    for key, value in fallback_counts.items():
        summary[f"fallback_{key}_count"] = value
    pass_nonregression = True
    pass_strict = True
    for axis in ("right", "down", "forward"):
        raw_value = rmse(errors[f"raw_{axis}"])
        corrected_value = rmse(errors[f"corrected_{axis}"])
        improvement = raw_value - corrected_value
        summary[f"raw_{axis}_rmse_m"] = raw_value
        summary[f"corrected_{axis}_rmse_m"] = corrected_value
        summary[f"{axis}_rmse_improvement_m"] = improvement
        if not (math.isfinite(improvement) and improvement >= 0.0):
            pass_nonregression = False
        if not (math.isfinite(improvement) and improvement > 0.0):
            pass_strict = False
    raw_position = [
        math.sqrt(r * r + d * d + f * f)
        for r, d, f in zip(errors["raw_right"], errors["raw_down"], errors["raw_forward"])
    ]
    corrected_position = [
        math.sqrt(r * r + d * d + f * f)
        for r, d, f in zip(errors["corrected_right"], errors["corrected_down"], errors["corrected_forward"])
    ]
    summary["raw_position_rmse_m"] = rmse(raw_position)
    summary["corrected_position_rmse_m"] = rmse(corrected_position)
    summary["position_rmse_improvement_m"] = float(summary["raw_position_rmse_m"]) - float(
        summary["corrected_position_rmse_m"]
    )
    summary["pass_nonregression"] = "yes" if pass_nonregression else "no"
    summary["pass_strict_improvement"] = "yes" if pass_strict else "no"
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
    output_dir = args.output_dir or args.report_dir / f"roi_silhouette_validation_{dt.datetime.now():%Y%m%d_%H%M%S}"
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
    gate_rows: list[dict[str, object]] = []
    final_pass = True
    for trial in validation_trials(args.validation_trials):
        samples = load_samples(normalized_dir / f"trial_{trial:02d}.csv", args.report_dir)
        for mode in ("centroid", "bbox_center", "top_center", "bottom_center"):
            rows, summary = evaluate_mode(samples, mode=mode, camera=camera, rotation=rotation, args=args)
            write_csv(output_dir / f"validation_trial_{trial:02d}_{mode}.csv", rows)
            write_json(output_dir / f"validation_trial_{trial:02d}_{mode}_summary.json", summary)
            if summary["pass_nonregression"] != "yes":
                final_pass = False
            gate_rows.append(
                {
                    "trial": trial,
                    "mode": mode,
                    "sample_count": summary["sample_count"],
                    "right_rmse_improvement_m": f"{float(summary['right_rmse_improvement_m']):.9f}",
                    "down_rmse_improvement_m": f"{float(summary['down_rmse_improvement_m']):.9f}",
                    "forward_rmse_improvement_m": f"{float(summary['forward_rmse_improvement_m']):.9f}",
                    "position_rmse_improvement_m": f"{float(summary['position_rmse_improvement_m']):.9f}",
                    "fallback_none_count": summary.get("fallback_none_count", 0),
                    "fallback_invalid_or_missing_roi_count": summary.get("fallback_invalid_or_missing_roi_count", 0),
                    "fallback_triangulation_failed_count": summary.get("fallback_triangulation_failed_count", 0),
                    "pass_nonregression": summary["pass_nonregression"],
                    "pass_strict_improvement": summary["pass_strict_improvement"],
                }
            )
    write_csv(output_dir / "validation_gate_summary.csv", gate_rows)
    write_json(
        output_dir / "metadata.json",
        {
            "source_report_dir": str(args.report_dir),
            "validation_trials": validation_trials(args.validation_trials),
            "roi_crop_margin": args.roi_crop_margin,
            "min_contrast": args.min_contrast,
            "min_mask_pixels": args.min_mask_pixels,
            "max_mask_fraction": args.max_mask_fraction,
            "contract": "low-contrast/missing ROI and triangulation failures are raw no-op fallbacks",
        },
    )
    print(f"Summary: {output_dir / 'validation_gate_summary.csv'}")
    print(f"Final pass: {'yes' if final_pass else 'no'}")
    return 0 if final_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
