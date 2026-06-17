#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

import bbox_anchor_offline_validation as anchor_base
import roi_silhouette_offline_validation as roi


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Diagnose a fixed ROI silhouette axis-gain candidate by trial, "
            "raw range bin, and screen-position bin. Invalid ROI or failed "
            "triangulation samples fall back to raw no-op."
        )
    )
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--mode", default="top_center")
    parser.add_argument("--right-gain", type=float, default=0.0)
    parser.add_argument("--down-gain", type=float, default=0.75)
    parser.add_argument("--forward-gain", type=float, default=0.75)
    parser.add_argument("--range-edges-m", default="0,500,800,1100,1400,2000")
    parser.add_argument("--screen-u-bins", type=int, default=3)
    parser.add_argument("--screen-v-bins", type=int, default=3)
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


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def bin_index(value: float, edges: Sequence[float]) -> int | None:
    if not math.isfinite(value) or value < edges[0] or value > edges[-1]:
        return None
    for index in range(len(edges) - 1):
        if edges[index] <= value < edges[index + 1] or (index == len(edges) - 2 and value <= edges[index + 1]):
            return index
    return None


def screen_bin(value: float, *, bins: int, size: float) -> int | None:
    if bins <= 0 or not math.isfinite(value) or value < 0.0 or value > size:
        return None
    index = int(value / size * bins)
    return min(bins - 1, max(0, index))


def bbox_center(bbox: tuple[float, float, float, float]) -> tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)


def position_error(error: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in error))


def evaluate_sample(
    sample: roi.Sample,
    *,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
    args: argparse.Namespace,
) -> dict[str, object]:
    left_point = roi.image_point(sample.left_roi_path, sample.left_bbox, mode=args.mode, args=args)
    right_point = roi.image_point(sample.right_roi_path, sample.right_bbox, mode=args.mode, args=args)
    fallback = "none"
    triangulated: tuple[float, float, float] | None = None
    if left_point is None or right_point is None:
        fallback = "invalid_or_missing_roi"
    else:
        triangulated = anchor_base.triangulate_with_rotation(left_point, right_point, camera, rotation)
        if triangulated is None:
            fallback = "triangulation_failed"
    corrected = sample.raw
    if triangulated is not None:
        gains = (float(args.right_gain), float(args.down_gain), float(args.forward_gain))
        corrected = tuple(sample.raw[index] + gains[index] * (triangulated[index] - sample.raw[index]) for index in range(3))
    raw_error = tuple(sample.raw[index] - sample.truth[index] for index in range(3))
    corrected_error = tuple(corrected[index] - sample.truth[index] for index in range(3))
    left_cx, left_cy = bbox_center(sample.left_bbox)
    right_cx, right_cy = bbox_center(sample.right_bbox)
    raw_range = math.sqrt(sum(value * value for value in sample.raw))
    return {
        "fallback": fallback,
        "raw_range_m": raw_range,
        "screen_u_px": (left_cx + right_cx) * 0.5,
        "screen_v_px": (left_cy + right_cy) * 0.5,
        "raw_error_right_m": raw_error[0],
        "raw_error_down_m": raw_error[1],
        "raw_error_forward_m": raw_error[2],
        "raw_error_position_m": position_error(raw_error),
        "corrected_error_right_m": corrected_error[0],
        "corrected_error_down_m": corrected_error[1],
        "corrected_error_forward_m": corrected_error[2],
        "corrected_error_position_m": position_error(corrected_error),
    }


def summarize(label: dict[str, object], rows: Sequence[dict[str, object]]) -> dict[str, object]:
    result = dict(label)
    result["sample_count"] = len(rows)
    result["valid_roi_count"] = sum(row["fallback"] == "none" for row in rows)
    result["invalid_or_missing_roi_count"] = sum(row["fallback"] == "invalid_or_missing_roi" for row in rows)
    result["triangulation_failed_count"] = sum(row["fallback"] == "triangulation_failed" for row in rows)
    for axis in ("right", "down", "forward", "position"):
        raw = [float(row[f"raw_error_{axis}_m"]) for row in rows]
        corrected = [float(row[f"corrected_error_{axis}_m"]) for row in rows]
        raw_rmse = rmse(raw)
        corrected_rmse = rmse(corrected)
        result[f"raw_{axis}_rmse_m"] = f"{raw_rmse:.9f}"
        result[f"corrected_{axis}_rmse_m"] = f"{corrected_rmse:.9f}"
        result[f"{axis}_rmse_improvement_m"] = f"{(raw_rmse - corrected_rmse):.9f}"
    result["pass_rdf_nonregression"] = (
        "yes"
        if all(float(result[f"{axis}_rmse_improvement_m"]) >= 0.0 for axis in ("right", "down", "forward"))
        else "no"
    )
    return result


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


def rows_for_trials(args: argparse.Namespace) -> list[dict[str, object]]:
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
    rows: list[dict[str, object]] = []
    range_edges = parse_float_list(args.range_edges_m)
    for trial in parse_int_list(args.validation_trials):
        samples = roi.load_samples(normalized_dir / f"trial_{trial:02d}.csv", args.report_dir)
        for sample in samples:
            row = evaluate_sample(sample, camera=camera, rotation=rotation, args=args)
            row["trial"] = trial
            row["range_bin"] = bin_index(float(row["raw_range_m"]), range_edges)
            row["screen_u_bin"] = screen_bin(
                float(row["screen_u_px"]), bins=int(args.screen_u_bins), size=float(args.image_width_px)
            )
            row["screen_v_bin"] = screen_bin(
                float(row["screen_v_px"]), bins=int(args.screen_v_bins), size=float(args.image_height_px)
            )
            rows.append(row)
    return rows


def group_rows(rows: Iterable[dict[str, object]], keys: Sequence[str]) -> dict[tuple[object, ...], list[dict[str, object]]]:
    grouped: dict[tuple[object, ...], list[dict[str, object]]] = {}
    for row in rows:
        key = tuple(row.get(name) for name in keys)
        grouped.setdefault(key, []).append(row)
    return grouped


def main() -> int:
    args = parse_args()
    rows = rows_for_trials(args)
    range_edges = parse_float_list(args.range_edges_m)
    write_csv(args.output_dir / "candidate_sample_diagnostics.csv", rows)
    summary_rows: list[dict[str, object]] = []
    summary_rows.append(summarize({"group": "all"}, rows))
    for trial_key, group in group_rows(rows, ("trial",)).items():
        summary_rows.append(summarize({"group": "trial", "trial": trial_key[0]}, group))
    for range_key, group in group_rows(rows, ("range_bin",)).items():
        index = range_key[0]
        if index is None:
            label = "outside"
        else:
            label = f"{range_edges[int(index)]:g}-{range_edges[int(index) + 1]:g}"
        summary_rows.append(summarize({"group": "range_bin", "range_bin": index, "range_bin_label": label}, group))
    for screen_key, group in group_rows(rows, ("screen_u_bin", "screen_v_bin")).items():
        summary_rows.append(
            summarize({"group": "screen_bin", "screen_u_bin": screen_key[0], "screen_v_bin": screen_key[1]}, group)
        )
    write_csv(args.output_dir / "candidate_group_summary.csv", summary_rows)
    write_json(
        args.output_dir / "metadata.json",
        {
            "report_dir": str(args.report_dir),
            "mode": args.mode,
            "gains": {
                "right": args.right_gain,
                "down": args.down_gain,
                "forward": args.forward_gain,
            },
            "range_edges_m": range_edges,
            "screen_u_bins": args.screen_u_bins,
            "screen_v_bins": args.screen_v_bins,
            "contract": "invalid ROI and triangulation failures are raw no-op fallbacks",
        },
    )
    overall = summary_rows[0]
    print(f"Summary: {args.output_dir / 'candidate_group_summary.csv'}")
    print(
        "Overall: "
        f"samples={overall['sample_count']} valid_roi={overall['valid_roi_count']} "
        f"pass={overall['pass_rdf_nonregression']} "
        f"improvements=({overall['right_rmse_improvement_m']},"
        f"{overall['down_rmse_improvement_m']},"
        f"{overall['forward_rmse_improvement_m']})"
    )
    return 0 if overall["pass_rdf_nonregression"] == "yes" else 1


if __name__ == "__main__":
    raise SystemExit(main())
