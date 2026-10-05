#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import bbox_anchor_offline_validation as anchor_base  # noqa: E402


DEFAULT_REPORTS = (
    "reports/origin_bbox_reprojection_validation_attitude_aware_component_gains_longprofile_right_att_0p1_20260609_seed20260625",
)


@dataclass(frozen=True)
class Candidate:
    name: str
    left_u: float
    left_v: float
    right_u: float
    right_v: float
    source: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep bbox non-center point methods on normalized profile/validation trials. "
            "Each method uses bbox geometry instead of the bbox center as the image observation."
        )
    )
    parser.add_argument("report_dirs", nargs="*", type=Path, default=[Path(item) for item in DEFAULT_REPORTS])
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--train-trial", type=int, default=1)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--image-width-px", type=float, default=anchor_base.DEFAULT_IMAGE_WIDTH_PX)
    parser.add_argument("--image-height-px", type=float, default=anchor_base.DEFAULT_IMAGE_HEIGHT_PX)
    parser.add_argument("--baseline-m", type=float, default=anchor_base.DEFAULT_BASELINE_M)
    parser.add_argument("--horizontal-fov-rad", type=float, default=anchor_base.DEFAULT_HORIZONTAL_FOV_RAD)
    parser.add_argument("--include-grid", action="store_true")
    parser.add_argument("--grid-step", type=float, default=0.1)
    parser.add_argument("--gain-step", type=float, default=0.02)
    parser.add_argument("--allow-learned-outside-bbox", action="store_true")
    parser.add_argument("--epsilon", type=float, default=1.0e-9)
    return parser.parse_args()


def validation_trials(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def fixed_candidates() -> list[Candidate]:
    anchors = [
        ("top_left", 0.0, 0.0),
        ("top_center", 0.5, 0.0),
        ("top_right", 1.0, 0.0),
        ("upper_third_center", 0.5, 1.0 / 3.0),
        ("origin_prior_center_u_v042", 0.5, 0.42),
        ("middle_left", 0.0, 0.5),
        ("middle_right", 1.0, 0.5),
        ("lower_third_center", 0.5, 2.0 / 3.0),
        ("bottom_left", 0.0, 1.0),
        ("bottom_center", 0.5, 1.0),
        ("bottom_right", 1.0, 1.0),
    ]
    candidates = [Candidate(name, u, v, u, v, "fixed") for name, u, v in anchors]
    for v_name, v in (
        ("top", 0.0),
        ("origin_prior_v042", 0.42),
        ("middle", 0.5),
        ("bottom", 1.0),
    ):
        candidates.extend(
            [
                Candidate(f"inner_edges_{v_name}", 0.0, v, 1.0, v, "cross_edge"),
                Candidate(f"outer_edges_{v_name}", 1.0, v, 0.0, v, "cross_edge"),
                Candidate(f"left_edge_to_right_center_{v_name}", 0.0, v, 0.5, v, "cross_edge"),
                Candidate(f"right_edge_to_right_center_{v_name}", 1.0, v, 0.5, v, "cross_edge"),
                Candidate(f"left_center_to_right_edge_{v_name}", 0.5, v, 1.0, v, "cross_edge"),
                Candidate(f"left_center_to_left_edge_{v_name}", 0.5, v, 0.0, v, "cross_edge"),
            ]
        )
    return candidates


def grid_candidates(step: float) -> list[Candidate]:
    if step <= 0.0 or step > 1.0:
        raise ValueError("--grid-step must be in (0, 1]")
    count = int(round(1.0 / step))
    values = sorted({0.0, 1.0, *(min(1.0, max(0.0, index * step)) for index in range(count + 1))})
    return [
        Candidate(f"grid_u{u:.2f}_v{v:.2f}", u, v, u, v, "grid")
        for v in values
        for u in values
        if not (abs(u - 0.5) <= 1.0e-12 and abs(v - 0.5) <= 1.0e-12)
    ]


def learned_candidates(
    samples: Sequence[anchor_base.Sample],
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
    *,
    allow_outside_bbox: bool,
) -> list[Candidate]:
    anchor = anchor_base.train_global_anchor(
        samples,
        camera,
        rotation,
        allow_outside_bbox=allow_outside_bbox,
    )
    learned = Candidate(
        "learned_origin_anchor",
        float(anchor["left_u"]),
        float(anchor["left_v"]),
        float(anchor["right_u"]),
        float(anchor["right_v"]),
        "learned",
    )
    center_u = Candidate(
        "learned_vertical_center_u",
        0.5,
        float(anchor["left_v"]),
        0.5,
        float(anchor["right_v"]),
        "learned",
    )
    shared_v = statistics.median([float(anchor["left_v"]), float(anchor["right_v"])])
    shared = Candidate("learned_shared_v_center_u", 0.5, shared_v, 0.5, shared_v, "learned")
    return [learned, center_u, shared]


def candidate_pixels(sample: anchor_base.Sample, candidate: Candidate) -> tuple[tuple[float, float], tuple[float, float]]:
    return (
        sample.left_bbox.anchor(candidate.left_u, candidate.left_v),
        sample.right_bbox.anchor(candidate.right_u, candidate.right_v),
    )


def bearing_raw_forward_point(
    sample: anchor_base.Sample,
    candidate: Candidate,
    camera: anchor_base.Camera,
) -> tuple[float, float, float]:
    left_pixel, right_pixel = candidate_pixels(sample, candidate)
    forward = sample.raw_forward_m
    avg_u = 0.5 * (left_pixel[0] + right_pixel[0])
    avg_v = 0.5 * (left_pixel[1] + right_pixel[1])
    right = (avg_u - camera.cx_px) * forward / camera.fx_px
    down = (avg_v - camera.cy_px) * forward / camera.fx_px
    return (right, down, forward)


def triangulated_point(
    sample: anchor_base.Sample,
    candidate: Candidate,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float] | None:
    left_pixel, right_pixel = candidate_pixels(sample, candidate)
    return anchor_base.triangulate_with_rotation(left_pixel, right_pixel, camera, rotation)


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def candidate_point(
    sample: anchor_base.Sample,
    candidate: Candidate,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
    *,
    method: str,
) -> tuple[float, float, float] | None:
    if method == "triangulated":
        return triangulated_point(sample, candidate, camera, rotation)
    if method == "bearing_raw_forward":
        return bearing_raw_forward_point(sample, candidate, camera)
    raise ValueError(f"unsupported method: {method}")


def train_axis_gains(
    samples: Sequence[anchor_base.Sample],
    candidate: Candidate,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
    *,
    method: str,
    step: float,
) -> tuple[float, float, float]:
    if step <= 0.0 or step > 1.0:
        raise ValueError("--gain-step must be in (0, 1]")
    rows: list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]] = []
    for sample in samples:
        point = candidate_point(sample, candidate, camera, rotation, method=method)
        if point is None:
            continue
        raw = (sample.raw_right_m, sample.raw_down_m, sample.raw_forward_m)
        truth = (sample.truth_right_m, sample.truth_down_m, sample.truth_forward_m)
        rows.append((raw, point, truth))
    if not rows:
        return (0.0, 0.0, 0.0)

    gains: list[float] = []
    count = int(round(1.0 / step))
    for axis in range(3):
        best_gain = 0.0
        best_rmse = rmse([raw[axis] - truth[axis] for raw, _, truth in rows])
        for index in range(1, count + 1):
            gain = min(1.0, index * step)
            value = rmse(
                [
                    (raw[axis] + gain * (point[axis] - raw[axis])) - truth[axis]
                    for raw, point, truth in rows
                ]
            )
            if math.isfinite(value) and value < best_rmse:
                best_rmse = value
                best_gain = gain
        gains.append(best_gain)
    return (gains[0], gains[1], gains[2])


def evaluate_candidate(
    samples: Sequence[anchor_base.Sample],
    candidate: Candidate,
    camera: anchor_base.Camera,
    rotation: tuple[tuple[float, float, float], ...],
    *,
    method: str,
    epsilon: float,
    gains: tuple[float, float, float] | None = None,
) -> dict[str, object]:
    errors: dict[str, list[float]] = {
        "raw_right": [],
        "raw_down": [],
        "raw_forward": [],
        "candidate_right": [],
        "candidate_down": [],
        "candidate_forward": [],
    }
    skipped = 0
    for sample in samples:
        point = candidate_point(sample, candidate, camera, rotation, method=method)
        if point is None:
            skipped += 1
            continue
        raw = (sample.raw_right_m, sample.raw_down_m, sample.raw_forward_m)
        truth = (sample.truth_right_m, sample.truth_down_m, sample.truth_forward_m)
        if gains is not None:
            point = tuple(raw[index] + gains[index] * (point[index] - raw[index]) for index in range(3))  # type: ignore[assignment]
        for axis, index in (("right", 0), ("down", 1), ("forward", 2)):
            errors[f"raw_{axis}"].append(raw[index] - truth[index])
            errors[f"candidate_{axis}"].append(point[index] - truth[index])

    summary: dict[str, object] = {
        "sample_count": len(samples),
        "evaluated_count": len(errors["raw_right"]),
        "skipped_count": skipped,
    }
    pass_nonregression = True
    pass_strict = True
    for axis in ("right", "down", "forward"):
        raw_value = rmse(errors[f"raw_{axis}"])
        candidate_value = rmse(errors[f"candidate_{axis}"])
        improvement = raw_value - candidate_value
        summary[f"raw_{axis}_rmse_m"] = raw_value
        summary[f"candidate_{axis}_rmse_m"] = candidate_value
        summary[f"{axis}_rmse_improvement_m"] = improvement
        if not (math.isfinite(improvement) and improvement >= -epsilon):
            pass_nonregression = False
        if not (math.isfinite(improvement) and improvement > epsilon):
            pass_strict = False

    for prefix in ("raw", "candidate"):
        position_errors = [
            math.sqrt(right * right + down * down + forward * forward)
            for right, down, forward in zip(
                errors[f"{prefix}_right"],
                errors[f"{prefix}_down"],
                errors[f"{prefix}_forward"],
            )
        ]
        summary[f"{prefix}_position_rmse_m"] = rmse(position_errors)
    summary["position_rmse_improvement_m"] = (
        float(summary["raw_position_rmse_m"]) - float(summary["candidate_position_rmse_m"])
    )
    summary["pass_nonregression"] = "yes" if pass_nonregression else "no"
    summary["pass_strict_improvement"] = "yes" if pass_strict else "no"
    if gains is not None:
        summary["right_gain"] = gains[0]
        summary["down_gain"] = gains[1]
        summary["forward_gain"] = gains[2]
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


def aggregate_rows(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str, str], list[dict[str, object]]] = {}
    for row in rows:
        key = (str(row["report"]), str(row["method"]), str(row["candidate"]))
        grouped.setdefault(key, []).append(row)

    aggregates: list[dict[str, object]] = []
    for (report, method, candidate), items in sorted(grouped.items()):
        axis_improvements = {
            axis: [float(item[f"{axis}_rmse_improvement_m"]) for item in items]
            for axis in ("right", "down", "forward")
        }
        aggregates.append(
            {
                "report": report,
                "method": method,
                "candidate": candidate,
                "trial_count": len(items),
                "pass_nonregression_all_trials": "yes"
                if all(str(item["pass_nonregression"]) == "yes" for item in items)
                else "no",
                "pass_strict_all_trials": "yes"
                if all(str(item["pass_strict_improvement"]) == "yes" for item in items)
                else "no",
                "min_right_improvement_m": min(axis_improvements["right"]),
                "min_down_improvement_m": min(axis_improvements["down"]),
                "min_forward_improvement_m": min(axis_improvements["forward"]),
                "mean_right_improvement_m": statistics.fmean(axis_improvements["right"]),
                "mean_down_improvement_m": statistics.fmean(axis_improvements["down"]),
                "mean_forward_improvement_m": statistics.fmean(axis_improvements["forward"]),
                "mean_position_improvement_m": statistics.fmean(
                    float(item["position_rmse_improvement_m"]) for item in items
                ),
                "right_gain": items[0].get("right_gain", ""),
                "down_gain": items[0].get("down_gain", ""),
                "forward_gain": items[0].get("forward_gain", ""),
            }
        )
    return aggregates


def run_report(args: argparse.Namespace, report_dir: Path, output_dir: Path) -> list[dict[str, object]]:
    camera = anchor_base.Camera(
        width_px=float(args.image_width_px),
        height_px=float(args.image_height_px),
        fx_px=anchor_base.focal_px(args.image_width_px, args.horizontal_fov_rad),
        baseline_m=float(args.baseline_m),
    )
    rotation = anchor_base.rotation_matrix_from_report(
        argparse.Namespace(report_dir=report_dir, right_optical_rotation_deg="")
    )
    normalized_dir = report_dir / "normalized"
    train_samples = anchor_base.load_samples(normalized_dir / f"trial_{args.train_trial:02d}.csv", camera)
    if not train_samples:
        raise RuntimeError(f"no training samples in {normalized_dir}")

    candidates = fixed_candidates() + learned_candidates(
        train_samples,
        camera,
        rotation,
        allow_outside_bbox=bool(args.allow_learned_outside_bbox),
    )
    if args.include_grid:
        candidates.extend(grid_candidates(float(args.grid_step)))

    rows: list[dict[str, object]] = []
    trained_gains = {
        (candidate.name, method): train_axis_gains(
            train_samples,
            candidate,
            camera,
            rotation,
            method=method,
            step=float(args.gain_step),
        )
        for candidate in candidates
        for method in ("triangulated", "bearing_raw_forward")
    }
    for trial in validation_trials(args.validation_trials):
        samples = anchor_base.load_samples(normalized_dir / f"trial_{trial:02d}.csv", camera)
        for candidate in candidates:
            for method, gains in (
                ("triangulated", None),
                ("bearing_raw_forward", None),
                ("triangulated_axis_gain", trained_gains[(candidate.name, "triangulated")]),
                ("bearing_raw_forward_axis_gain", trained_gains[(candidate.name, "bearing_raw_forward")]),
            ):
                base_method = method.replace("_axis_gain", "")
                summary = evaluate_candidate(
                    samples,
                    candidate,
                    camera,
                    rotation,
                    method=base_method,
                    epsilon=float(args.epsilon),
                    gains=gains,
                )
                rows.append(
                    {
                        "report": report_dir.name,
                        "trial": trial,
                        "method": method,
                        "candidate": candidate.name,
                        "candidate_source": candidate.source,
                        "left_u": candidate.left_u,
                        "left_v": candidate.left_v,
                        "right_u": candidate.right_u,
                        "right_v": candidate.right_v,
                        **summary,
                    }
                )
    report_output = output_dir / report_dir.name
    write_csv(report_output / "trial_candidate_summary.csv", rows)
    write_csv(report_output / "candidate_aggregate_summary.csv", aggregate_rows(rows))
    return rows


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir or Path("reports") / f"bbox_non_center_sweep_{dt.datetime.now():%Y%m%d_%H%M%S}"
    all_rows: list[dict[str, object]] = []
    for report_dir in args.report_dirs:
        all_rows.extend(run_report(args, report_dir, output_dir))

    aggregate = aggregate_rows(all_rows)
    write_csv(output_dir / "candidate_aggregate_summary.csv", aggregate)
    metadata = {
        "report_dirs": [str(path) for path in args.report_dirs],
        "train_trial": args.train_trial,
        "validation_trials": validation_trials(args.validation_trials),
        "include_grid": bool(args.include_grid),
        "grid_step": args.grid_step,
        "contract": (
            "Non-center bbox candidates are evaluated against raw RDF RMSE. "
            "pass_nonregression allows zero improvement; pass_strict requires positive improvement on every RDF axis."
        ),
    }
    write_json(output_dir / "metadata.json", metadata)
    pass_count = sum(1 for row in aggregate if row["pass_nonregression_all_trials"] == "yes")
    strict_count = sum(1 for row in aggregate if row["pass_strict_all_trials"] == "yes")
    print(f"Output: {output_dir}")
    print(f"Aggregate rows: {output_dir / 'candidate_aggregate_summary.csv'}")
    print(f"Non-regression candidates: {pass_count}")
    print(f"Strict-improvement candidates: {strict_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
