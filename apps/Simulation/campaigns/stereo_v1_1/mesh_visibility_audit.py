#!/usr/bin/env python3
"""Offline source-pixel foreground visibility on an explicitly uniform sky.

This is not YOLO, target identification, calibration, or a positioning metric.
It supports only a uniform background and one small contrasting foreground
object assumed to be the rendered drone. It cannot establish that identity,
recover invisible parts, or prove the complete 3D mesh is visible. Unsupported
scenes fail closed. No frame is removed from science data or campaign counts.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

SIM_ROOT = Path(__file__).resolve().parents[2]
if str(SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(SIM_ROOT))

THRESHOLDS = {"background_tolerance_rgb8": 3, "foreground_contrast_rgb8": 16,
              "maximum_nonbackground_fraction": 0.02,
              "maximum_border_outlier_fraction": 0.02,
              "maximum_foreground_bbox_axis_fraction": 0.25,
              "minimum_foreground_pixels": 4}


def inspect_pixels(pixels: np.ndarray, margin_px: float = 32) -> dict:
    """Return original u/right,v/down pixel bbox; xyxy max edges are exclusive."""
    if (pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3
            or min(pixels.shape[:2]) < 4 or not math.isfinite(margin_px) or margin_px < 0):
        raise ValueError("expected uint8 HxWx3 image and finite nonnegative pixel margin")
    h, w = pixels.shape[:2]
    border = np.concatenate((pixels[0], pixels[-1], pixels[1:-1, 0], pixels[1:-1, -1]))
    background = np.median(border, axis=0)
    difference = np.max(np.abs(pixels.astype(np.int16)-background), axis=2)
    border_difference = np.max(np.abs(border.astype(np.int16)-background), axis=1)
    nonbackground_fraction = float(np.mean(difference > THRESHOLDS["background_tolerance_rgb8"]))
    border_fraction = float(np.mean(border_difference > THRESHOLDS["background_tolerance_rgb8"]))
    foreground = difference >= THRESHOLDS["foreground_contrast_rgb8"]
    ys, xs = np.nonzero(foreground)
    result = {"status": "UNSUPPORTED_SCENE", "bbox_xyxy_original_px": None,
              "minimum_edge_margin_px": None, "foreground_pixel_count": len(xs),
              "image_width_px": w, "image_height_px": h,
              "border_background_color": background.tolist(),
              "nonbackground_fraction": nonbackground_fraction,
              "border_outlier_fraction": border_fraction}
    if (nonbackground_fraction > THRESHOLDS["maximum_nonbackground_fraction"]
            or border_fraction > THRESHOLDS["maximum_border_outlier_fraction"]):
        return result
    if len(xs) < THRESHOLDS["minimum_foreground_pixels"]:
        result["status"] = "NO_FOREGROUND"
        return result
    x1, x2, y1, y2 = int(xs.min()), int(xs.max())+1, int(ys.min()), int(ys.max())+1
    margin = min(x1, y1, w-x2, h-y2)
    result.update(bbox_xyxy_original_px=[x1, y1, x2, y2], minimum_edge_margin_px=margin)
    if ((x2-x1)/w > THRESHOLDS["maximum_foreground_bbox_axis_fraction"]
            or (y2-y1)/h > THRESHOLDS["maximum_foreground_bbox_axis_fraction"]):
        return result
    result["status"] = "CLIPPED" if margin == 0 else "MARGIN_VIOLATION" if margin < margin_px else "VISIBLE"
    return result


def _rows(path: Path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def audit_attempt(attempt_dir: Path, margin_px: float = 32) -> dict:
    from campaigns.stereo_v1_1.postprocess_detections import _source_frames
    if not math.isfinite(margin_px) or margin_px < 0:
        raise ValueError("minimum margin must be finite and nonnegative")
    events = _rows(attempt_dir / "events.jsonl")
    starts = [r for r in events if r.get("event") == "science_start"]
    ends = [r for r in events if r.get("event") == "science_end"]
    if len(starts) != 1 or len(ends) != 1 or starts[0]["simulation_epoch"] != ends[0]["simulation_epoch"]:
        raise ValueError("audit requires one completed, same-epoch science window")
    start, end, epoch = starts[0]["simulation_timestamp_ns"], ends[0]["simulation_timestamp_ns"], starts[0]["simulation_epoch"]
    if start >= end:
        raise ValueError("science window must be positive")
    selected = [r for r in _rows(attempt_dir / "image_index.jsonl")
                if r["simulation_epoch"] == epoch and start <= r["image_timestamp_ns"] <= end]
    report = {"method": "uniform_background_source_pixel_visibility_v1",
              "audit_code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "status": "FAILED", "is_yolo_detection": False, "uses_gt_or_3d_positions": False,
              "changes_science_selection": False,
              "assumptions": ["uniform background", "one small contrasting object assumed to be rendered drone",
                              "visible contrast silhouette only; not proof of full 3D mesh or object identity"],
              "thresholds": {**THRESHOLDS, "minimum_edge_margin_px": margin_px},
              "margin_requirement_source": "engineering_guard_not_protocol_threshold",
              "bbox_contract": "source pixels u right,v down; xyxy upper edges exclusive",
              "simulation_epoch": epoch, "science_start_ns": start, "science_end_ns": end,
              "source_index_sha256": hashlib.sha256((attempt_dir / "image_index.jsonl").read_bytes()).hexdigest(),
              "source_events_sha256": hashlib.sha256((attempt_dir / "events.jsonl").read_bytes()).hexdigest(),
              "cameras": {}}
    all_valid = True
    for side in ("left", "right"):
        expected_rows = [r for r in selected if r["camera_id"] == side]
        expected = {r["original_image_reference"]: r for r in expected_rows}
        if len(expected) != len(expected_rows):
            raise ValueError("duplicate raw image reference in source index")
        results, seen = [], set()
        for reference, _, msg in _source_frames(attempt_dir, side):
            if reference not in expected:
                continue
            source = expected[reference]
            stamp = int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
            if (reference in seen or stamp != source["image_timestamp_ns"]
                    or msg.encoding not in ("rgb8", "bgr8") or msg.step != msg.width*3
                    or msg.width != source["image_width_px"] or msg.height != source["image_height_px"]):
                raise ValueError("raw image differs from source-index contract")
            data = bytes(msg.data)
            if hashlib.sha256(data).hexdigest() != source["image_pixels_sha256"]:
                raise ValueError("raw image pixels differ from recorded source hash")
            pixels = np.frombuffer(data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            result = inspect_pixels(pixels, margin_px)
            results.append({"frame_id": source["frame_id"], "image_timestamp_ns": stamp,
                            "original_image_reference": reference, "source_encoding": msg.encoding, **result})
            seen.add(reference)
        counts = dict(Counter(r["status"] for r in results))
        valid = bool(expected and seen == set(expected) and counts.get("VISIBLE", 0) == len(expected))
        margins = [r["minimum_edge_margin_px"] for r in results if r["minimum_edge_margin_px"] is not None]
        report["cameras"][side] = {"status": "PASS_UNDER_ASSUMPTIONS" if valid else "FAILED",
            "expected_science_images": len(expected), "audited_images": len(results), "status_counts": counts,
            "missing_original_references": sorted(set(expected)-seen),
            "minimum_observed_edge_margin_px": min(margins) if margins else None, "frames": results}
        all_valid = all_valid and valid
    report["status"] = "PASS_UNDER_ASSUMPTIONS" if all_valid else "FAILED"
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--minimum-margin-px", type=float, default=32)
    args = parser.parse_args()
    if args.report.resolve().is_relative_to(args.attempt_dir.resolve()):
        raise ValueError("write the derived audit outside the immutable attempt directory")
    if args.report.exists():
        raise FileExistsError("audit report already exists; refusing overwrite")
    report = audit_attempt(args.attempt_dir, args.minimum_margin_px)
    with args.report.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"status": report["status"], "report_name": args.report.name}))
    return 0 if report["status"] == "PASS_UNDER_ASSUMPTIONS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
