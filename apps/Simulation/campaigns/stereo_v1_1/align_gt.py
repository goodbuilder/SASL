#!/usr/bin/env python3
"""Align source Gazebo GT to every saved image and science control timestamp.

Raw GT/image bags are never changed. V0 location-performance summaries remain
closed until the campaign algorithm and path-selection rule are frozen.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.gt_alignment import GroundTruthSeries


def _rows(path: Path):
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            yield json.loads(line)


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    point = fraction * (len(ordered) - 1)
    lower = int(math.floor(point))
    upper = int(math.ceil(point))
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (point - lower)


def align_attempt(attempt_dir: Path, output_dir: Path, *, reveal_tracking: bool = False,
                  max_bracketing_gap_s: float = 0.04) -> dict:
    if output_dir.exists():
        raise FileExistsError(f"derived GT alignment output already exists: {output_dir}")
    run_manifest_path = attempt_dir / "run_manifest.json"
    manifest = json.loads(run_manifest_path.read_text()) if run_manifest_path.is_file() else {}
    if manifest.get("path_id") == "V0" and reveal_tracking:
        raise ValueError("V0 tracking/location metrics remain closed before algorithm freeze")
    if not math.isfinite(max_bracketing_gap_s) or max_bracketing_gap_s <= 0:
        raise ValueError("maximum GT bracketing gap must be positive and finite")
    series = GroundTruthSeries(
        _rows(attempt_dir / "ground_truth.jsonl"),
        max_bracketing_gap_ns=round(max_bracketing_gap_s * 1_000_000_000),
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    image_counts = Counter()
    science_counts = Counter()
    events_path = attempt_dir / "events.jsonl"
    science_bounds = None
    if events_path.is_file():
        events = list(_rows(events_path))
        starts = [row["simulation_timestamp_ns"] for row in events if row.get("event") == "science_start"]
        ends = [row["simulation_timestamp_ns"] for row in events if row.get("event") == "science_end"]
        if len(starts) == len(ends) == 1:
            science_bounds = (int(starts[0]), int(ends[0]))
    with (output_dir / "gt_at_images.jsonl").open("x", encoding="utf-8") as handle:
        for frame in _rows(attempt_dir / "image_index.jsonl"):
            aligned = series.align(frame["simulation_epoch"], frame["image_timestamp_ns"])
            image_counts[aligned.status] += 1
            if science_bounds and science_bounds[0] <= frame["image_timestamp_ns"] <= science_bounds[1]:
                science_counts[aligned.status] += 1
            row = {
                "run_id": frame["run_id"],
                "attempt_id": frame["attempt_id"],
                "camera_id": frame["camera_id"],
                "frame_id": frame["frame_id"],
                "simulation_epoch": frame["simulation_epoch"],
                "image_timestamp_ns": frame["image_timestamp_ns"],
                "alignment_status": aligned.status,
                "gt_position_frame": "gazebo_world_enu",
                "gt_position_axis_order": "east,north,up",
                "gt_position_world_enu_m": aligned.position_world_enu_m,
                "gt_orientation_frame": "world_from_vehicle_reference_link",
                "gt_quaternion_order": "xyzw",
                "gt_orientation_world_from_reference_xyzw": aligned.orientation_world_from_reference_xyzw,
                "gt_before_timestamp_ns": aligned.before_timestamp_ns,
                "gt_after_timestamp_ns": aligned.after_timestamp_ns,
                "gt_interpolation_alpha": aligned.interpolation_alpha,
                "gt_bracketing_gap_ns": aligned.bracketing_gap_ns,
            }
            handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")

    tracking = None
    if reveal_tracking:
        errors = []
        aligned_count = 0
        total = 0
        for row in _rows(attempt_dir / "flight_state.jsonl"):
            if row["phase"] != "science":
                continue
            total += 1
            aligned = series.align(row["simulation_epoch"], row["simulation_timestamp_ns"])
            if aligned.status not in ("EXACT", "INTERPOLATED") or row["reference_position_world_enu_m"] is None:
                continue
            aligned_count += 1
            errors.append(tuple(aligned.position_world_enu_m[index] - row["reference_position_world_enu_m"][index]
                                for index in range(3)))
        if errors:
            norms = [math.sqrt(sum(component * component for component in error)) for error in errors]
            tracking = {
                "method": "image-time-compatible GT interpolation at 20 Hz science control stamps",
                "input_frame": "gazebo_world_enu:east,north,up",
                "science_control_rows": total,
                "valid_gt_aligned_rows": aligned_count,
                "axis_rmse_enu_m": {
                    name: math.sqrt(statistics.mean(error[index] ** 2 for error in errors))
                    for index, name in enumerate(("east", "north", "up"))
                },
                "position_3d_rmse_m": math.sqrt(statistics.mean(value * value for value in norms)),
                "position_3d_p95_m": _percentile(norms, 0.95),
                "tracking_warning_p95_gt_1m": _percentile(norms, 0.95) > 1.0,
            }
    summary = {
        "schema_version": "stereo_v1_1_gt_alignment_v1",
        "method": "linear_world_enu_position_and_shortest_path_xyzw_slerp",
        "maximum_bracketing_gap_s": max_bracketing_gap_s,
        "image_count_by_alignment_status": dict(image_counts),
        "image_count_total": sum(image_counts.values()),
        "science_image_count_by_alignment_status": dict(science_counts),
        "science_image_count_total": sum(science_counts.values()),
        "tracking": tracking,
        "tracking_metrics_revealed": bool(reveal_tracking),
    }
    (output_dir / "gt_alignment_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--reveal-tracking", action="store_true")
    parser.add_argument("--max-bracketing-gap-s", type=float, default=0.04)
    args = parser.parse_args()
    result = align_attempt(args.attempt_dir, args.output_dir, reveal_tracking=args.reveal_tracking,
                           max_bracketing_gap_s=args.max_bracketing_gap_s)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
