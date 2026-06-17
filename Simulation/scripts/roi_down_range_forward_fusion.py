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


RDF_AXES = ("right", "down", "forward")
POSITION_AXES = ("right", "down", "forward", "position")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fuse two conservative observations on the same validation rows: "
            "Right remains raw, Down is blended from ROI silhouette triangulation, "
            "and Forward is blended from an origin-bbox reprojection/range-only source."
        )
    )
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--modes", default="top_center")
    parser.add_argument("--down-gains", default="0,0.1,0.25,0.5,0.75,1.0")
    parser.add_argument("--forward-gains", default="0,0.25,0.5,0.75,1.0")
    parser.add_argument(
        "--forward-source",
        choices=("origin_bbox_reprojection", "corrected", "raw"),
        default="origin_bbox_reprojection",
    )
    parser.add_argument("--diagnostic-mode", default="top_center")
    parser.add_argument("--diagnostic-down-gain", type=float, default=0.75)
    parser.add_argument("--diagnostic-forward-gain", type=float, default=1.0)
    parser.add_argument("--range-edges-m", default="0,500,800,1100,1400,2000")
    parser.add_argument("--screen-u-bins", type=int, default=3)
    parser.add_argument("--screen-v-bins", type=int, default=3)
    parser.add_argument(
        "--forward-min-range-m",
        type=float,
        help="If set, Forward correction is no-op below this raw range.",
    )
    parser.add_argument(
        "--forward-max-range-m",
        type=float,
        help="If set, Forward correction is no-op above this raw range.",
    )
    parser.add_argument(
        "--forward-min-correction-abs-m",
        type=float,
        help="If set, Forward correction is no-op when abs(source - raw) is below this value.",
    )
    parser.add_argument(
        "--disable-down-screen-bins",
        default="",
        help="Comma-separated screen bins u:v where Down correction is no-op, e.g. 0:1,2:1.",
    )
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


def parse_screen_bin_set(text: str) -> set[tuple[int, int]]:
    bins: set[tuple[int, int]] = set()
    for part in parse_str_list(text):
        u_text, v_text = part.split(":", 1)
        bins.add((int(u_text), int(v_text)))
    return bins


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


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


def forward_source_key(name: str) -> str:
    if name == "origin_bbox_reprojection":
        return "origin_bbox_reprojection_corrected_forward_m"
    if name == "corrected":
        return "corrected_forward_m"
    if name == "raw":
        return "raw_forward_m"
    raise ValueError(f"unsupported forward source: {name}")


def load_forward_sources(report_dir: Path, trial: int, source: str) -> list[float | None]:
    path = report_dir / f"validation_trial_{trial:02d}_comparison.csv"
    key = forward_source_key(source)
    values: list[float | None] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            values.append(finite_float(row.get(key)))
    return values


def bbox_center(bbox: tuple[float, float, float, float]) -> tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)


def position_error(error: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in error))


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


def build_camera(args: argparse.Namespace) -> anchor_base.Camera:
    return anchor_base.Camera(
        width_px=float(args.image_width_px),
        height_px=float(args.image_height_px),
        fx_px=anchor_base.focal_px(args.image_width_px, args.horizontal_fov_rad),
        baseline_m=float(args.baseline_m),
    )


def roi_triangulation(
    sample: roi.Sample,
    *,
    mode: str,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
    args: argparse.Namespace,
) -> tuple[tuple[float, float, float] | None, str]:
    left_point = roi.image_point(sample.left_roi_path, sample.left_bbox, mode=mode, args=args)
    right_point = roi.image_point(sample.right_roi_path, sample.right_bbox, mode=mode, args=args)
    if left_point is None or right_point is None:
        return None, "invalid_or_missing_roi"
    triangulated = anchor_base.triangulate_with_rotation(left_point, right_point, camera, rotation)
    if triangulated is None:
        return None, "triangulation_failed"
    return triangulated, "none"


def corrected_point(
    sample: roi.Sample,
    *,
    triangulated: tuple[float, float, float] | None,
    forward_source: float | None,
    raw_range_m: float,
    screen_key: tuple[int | None, int | None],
    down_gain: float,
    forward_gain: float,
    args: argparse.Namespace,
) -> tuple[tuple[float, float, float], str]:
    corrected_right = sample.raw[0]
    corrected_down = sample.raw[1]
    corrected_forward = sample.raw[2]
    down_disabled_bins = parse_screen_bin_set(str(args.disable_down_screen_bins))
    down_guarded = (
        screen_key[0] is not None
        and screen_key[1] is not None
        and (int(screen_key[0]), int(screen_key[1])) in down_disabled_bins
    )
    if triangulated is not None and not down_guarded:
        corrected_down = sample.raw[1] + float(down_gain) * (triangulated[1] - sample.raw[1])
    forward_fallback = "none"
    if forward_source is None:
        forward_fallback = "missing_forward_source"
    elif args.forward_min_range_m is not None and raw_range_m < float(args.forward_min_range_m):
        forward_fallback = "range_guarded_noop"
    elif args.forward_max_range_m is not None and raw_range_m > float(args.forward_max_range_m):
        forward_fallback = "range_guarded_noop"
    elif (
        args.forward_min_correction_abs_m is not None
        and abs(float(forward_source) - float(sample.raw[2])) < float(args.forward_min_correction_abs_m)
    ):
        forward_fallback = "tiny_correction_guarded_noop"
    else:
        corrected_forward = sample.raw[2] + float(forward_gain) * (float(forward_source) - sample.raw[2])
    return (corrected_right, corrected_down, corrected_forward), forward_fallback


def sample_error_rows(
    report_dir: Path,
    *,
    mode: str,
    down_gain: float,
    forward_gain: float,
    args: argparse.Namespace,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    range_edges = parse_float_list(args.range_edges_m)
    normalized_dir = report_dir / "normalized"
    for trial in parse_int_list(args.validation_trials):
        samples = roi.load_samples(normalized_dir / f"trial_{trial:02d}.csv", report_dir)
        forward_sources = load_forward_sources(report_dir, trial, str(args.forward_source))
        for index, sample in enumerate(samples):
            triangulated, roi_fallback = roi_triangulation(sample, mode=mode, camera=camera, rotation=rotation, args=args)
            forward_source = forward_sources[index] if index < len(forward_sources) else None
            left_cx, left_cy = bbox_center(sample.left_bbox)
            right_cx, right_cy = bbox_center(sample.right_bbox)
            screen_u_px = (left_cx + right_cx) * 0.5
            screen_v_px = (left_cy + right_cy) * 0.5
            raw_range = math.sqrt(sum(value * value for value in sample.raw))
            screen_key = (
                screen_bin(screen_u_px, bins=int(args.screen_u_bins), size=float(args.image_width_px)),
                screen_bin(screen_v_px, bins=int(args.screen_v_bins), size=float(args.image_height_px)),
            )
            corrected, forward_fallback = corrected_point(
                sample,
                triangulated=triangulated,
                forward_source=forward_source,
                raw_range_m=raw_range,
                screen_key=screen_key,
                down_gain=down_gain,
                forward_gain=forward_gain,
                args=args,
            )
            raw_error = tuple(sample.raw[axis] - sample.truth[axis] for axis in range(3))
            corrected_error = tuple(corrected[axis] - sample.truth[axis] for axis in range(3))
            rows.append(
                {
                    "fallback": roi_fallback,
                    "forward_fallback": forward_fallback,
                    "down_guarded_noop": (
                        "yes"
                        if screen_key[0] is not None
                        and screen_key[1] is not None
                        and (int(screen_key[0]), int(screen_key[1]))
                        in parse_screen_bin_set(str(args.disable_down_screen_bins))
                        else "no"
                    ),
                    "mode": mode,
                    "right_gain": "0",
                    "down_gain": f"{float(down_gain):.6g}",
                    "forward_gain": f"{float(forward_gain):.6g}",
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
                    "trial": trial,
                    "range_bin": bin_index(raw_range, range_edges),
                    "screen_u_bin": screen_key[0],
                    "screen_v_bin": screen_key[1],
                }
            )
    return rows


def summarize(label: dict[str, object], rows: Sequence[dict[str, object]]) -> dict[str, object]:
    result = dict(label)
    result["sample_count"] = len(rows)
    result["valid_roi_count"] = sum(row["fallback"] == "none" for row in rows)
    result["invalid_or_missing_roi_count"] = sum(row["fallback"] == "invalid_or_missing_roi" for row in rows)
    result["triangulation_failed_count"] = sum(row["fallback"] == "triangulation_failed" for row in rows)
    result["missing_forward_source_count"] = sum(row["forward_fallback"] != "none" for row in rows)
    result["range_guarded_forward_noop_count"] = sum(row["forward_fallback"] == "range_guarded_noop" for row in rows)
    result["tiny_guarded_forward_noop_count"] = sum(
        row["forward_fallback"] == "tiny_correction_guarded_noop" for row in rows
    )
    result["screen_guarded_down_noop_count"] = sum(row["down_guarded_noop"] == "yes" for row in rows)
    for axis in POSITION_AXES:
        raw = [float(row[f"raw_error_{axis}_m"]) for row in rows]
        corrected = [float(row[f"corrected_error_{axis}_m"]) for row in rows]
        raw_rmse = rmse(raw)
        corrected_rmse = rmse(corrected)
        result[f"raw_{axis}_rmse_m"] = f"{raw_rmse:.9f}"
        result[f"corrected_{axis}_rmse_m"] = f"{corrected_rmse:.9f}"
        result[f"{axis}_rmse_improvement_m"] = f"{(raw_rmse - corrected_rmse):.9f}"
    result["pass_rdf_nonregression"] = (
        "yes" if all(float(result[f"{axis}_rmse_improvement_m"]) >= 0.0 for axis in RDF_AXES) else "no"
    )
    return result


def group_rows(rows: Iterable[dict[str, object]], keys: Sequence[str]) -> dict[tuple[object, ...], list[dict[str, object]]]:
    grouped: dict[tuple[object, ...], list[dict[str, object]]] = {}
    for row in rows:
        key = tuple(row.get(name) for name in keys)
        grouped.setdefault(key, []).append(row)
    return grouped


def trial_summary_rows(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for trial_key, group in sorted(group_rows(rows, ("trial",)).items()):
        result.append(summarize({"trial": trial_key[0]}, group))
    return result


def candidate_key(mode: str, down_gain: float, forward_gain: float) -> tuple[str, str, str, str]:
    return (mode, "0", f"{float(down_gain):.6g}", f"{float(forward_gain):.6g}")


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    camera = build_camera(args)
    rotation = anchor_base.rotation_matrix_from_report(
        argparse.Namespace(report_dir=args.report_dir, right_optical_rotation_deg="")
    )
    candidate_rows: list[dict[str, object]] = []
    all_trial_rows: list[dict[str, object]] = []
    diagnostics: dict[tuple[str, str, str, str], list[dict[str, object]]] = {}
    diagnostic_key = candidate_key(
        str(args.diagnostic_mode),
        float(args.diagnostic_down_gain),
        float(args.diagnostic_forward_gain),
    )

    for mode in parse_str_list(args.modes):
        for down_gain in parse_float_list(args.down_gains):
            for forward_gain in parse_float_list(args.forward_gains):
                rows = sample_error_rows(
                    args.report_dir,
                    mode=mode,
                    down_gain=down_gain,
                    forward_gain=forward_gain,
                    args=args,
                    camera=camera,
                    rotation=rotation,
                )
                key = candidate_key(mode, down_gain, forward_gain)
                diagnostics[key] = rows
                per_trial = trial_summary_rows(rows)
                for row in per_trial:
                    row.update(
                        {
                            "mode": mode,
                            "right_gain": "0",
                            "down_gain": f"{float(down_gain):.6g}",
                            "forward_gain": f"{float(forward_gain):.6g}",
                            "forward_min_range_m": "" if args.forward_min_range_m is None else args.forward_min_range_m,
                            "forward_max_range_m": "" if args.forward_max_range_m is None else args.forward_max_range_m,
                            "forward_min_correction_abs_m": (
                                "" if args.forward_min_correction_abs_m is None else args.forward_min_correction_abs_m
                            ),
                            "disable_down_screen_bins": args.disable_down_screen_bins,
                        }
                    )
                    all_trial_rows.append(row)
                improvements = {
                    axis: [float(row[f"{axis}_rmse_improvement_m"]) for row in per_trial]
                    for axis in POSITION_AXES
                }
                pass_count = sum(row["pass_rdf_nonregression"] == "yes" for row in per_trial)
                candidate_rows.append(
                    {
                        "mode": mode,
                        "right_gain": "0",
                        "down_gain": f"{float(down_gain):.6g}",
                        "forward_gain": f"{float(forward_gain):.6g}",
                        "forward_source": args.forward_source,
                        "trial_count": len(per_trial),
                        "pass_nonregression_count": pass_count,
                        "pass_all_nonregression": "yes" if pass_count == len(per_trial) else "no",
                        "min_right_rmse_improvement_m": f"{min(improvements['right']):.9f}",
                        "mean_right_rmse_improvement_m": f"{sum(improvements['right']) / len(per_trial):.9f}",
                        "min_down_rmse_improvement_m": f"{min(improvements['down']):.9f}",
                        "mean_down_rmse_improvement_m": f"{sum(improvements['down']) / len(per_trial):.9f}",
                        "min_forward_rmse_improvement_m": f"{min(improvements['forward']):.9f}",
                        "mean_forward_rmse_improvement_m": f"{sum(improvements['forward']) / len(per_trial):.9f}",
                        "min_position_rmse_improvement_m": f"{min(improvements['position']):.9f}",
                        "mean_position_rmse_improvement_m": f"{sum(improvements['position']) / len(per_trial):.9f}",
                        "valid_roi_count": sum(int(row["valid_roi_count"]) for row in per_trial),
                        "invalid_or_missing_roi_count": sum(int(row["invalid_or_missing_roi_count"]) for row in per_trial),
                        "triangulation_failed_count": sum(int(row["triangulation_failed_count"]) for row in per_trial),
                        "missing_forward_source_count": sum(int(row["missing_forward_source_count"]) for row in per_trial),
                        "range_guarded_forward_noop_count": sum(
                            int(row["range_guarded_forward_noop_count"]) for row in per_trial
                        ),
                        "tiny_guarded_forward_noop_count": sum(
                            int(row["tiny_guarded_forward_noop_count"]) for row in per_trial
                        ),
                        "screen_guarded_down_noop_count": sum(
                            int(row["screen_guarded_down_noop_count"]) for row in per_trial
                        ),
                        "forward_min_range_m": "" if args.forward_min_range_m is None else args.forward_min_range_m,
                        "forward_max_range_m": "" if args.forward_max_range_m is None else args.forward_max_range_m,
                        "forward_min_correction_abs_m": (
                            "" if args.forward_min_correction_abs_m is None else args.forward_min_correction_abs_m
                        ),
                        "disable_down_screen_bins": args.disable_down_screen_bins,
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
    write_csv(output_dir / "axis_gain_candidate_summary.csv", candidate_rows)
    write_csv(output_dir / "axis_gain_trial_summary.csv", all_trial_rows)
    diagnostic_rows = diagnostics.get(diagnostic_key, [])
    write_csv(output_dir / "candidate_sample_diagnostics.csv", diagnostic_rows)
    group_summary: list[dict[str, object]] = [summarize({"group": "all"}, diagnostic_rows)]
    for trial_key, group in group_rows(diagnostic_rows, ("trial",)).items():
        group_summary.append(summarize({"group": "trial", "trial": trial_key[0]}, group))
    range_edges = parse_float_list(args.range_edges_m)
    for range_key, group in group_rows(diagnostic_rows, ("range_bin",)).items():
        index = range_key[0]
        label = "outside" if index is None else f"{range_edges[int(index)]:g}-{range_edges[int(index) + 1]:g}"
        group_summary.append(summarize({"group": "range_bin", "range_bin": index, "range_bin_label": label}, group))
    for screen_key, group in group_rows(diagnostic_rows, ("screen_u_bin", "screen_v_bin")).items():
        group_summary.append(
            summarize({"group": "screen_bin", "screen_u_bin": screen_key[0], "screen_v_bin": screen_key[1]}, group)
        )
    write_csv(output_dir / "candidate_group_summary.csv", group_summary)
    write_json(
        output_dir / "metadata.json",
        {
            "source_report_dir": str(args.report_dir),
            "forward_source": args.forward_source,
            "diagnostic_candidate": {
                "mode": args.diagnostic_mode,
                "right_gain": 0.0,
                "down_gain": args.diagnostic_down_gain,
                "forward_gain": args.diagnostic_forward_gain,
            },
            "range_edges_m": range_edges,
            "screen_u_bins": args.screen_u_bins,
            "screen_v_bins": args.screen_v_bins,
            "forward_min_range_m": args.forward_min_range_m,
            "forward_max_range_m": args.forward_max_range_m,
            "forward_min_correction_abs_m": args.forward_min_correction_abs_m,
            "disable_down_screen_bins": args.disable_down_screen_bins,
            "contract": (
                "Right remains raw. Down is blended only when ROI silhouette triangulates. "
                "Forward is blended from the configured forward source; missing source or guard match falls back to raw."
            ),
        },
    )
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
        print(f"Candidate summary: {output_dir / 'axis_gain_candidate_summary.csv'}")
        return 0 if best["pass_all_nonregression"] == "yes" else 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
