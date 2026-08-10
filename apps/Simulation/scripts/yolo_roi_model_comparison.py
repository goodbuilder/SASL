#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from pathlib import Path
from statistics import mean, median
from time import perf_counter
from typing import Any

from ultralytics import YOLO


TARGET_CLASS_NAMES = {"uav", "drone"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare YOLO model variants on saved stereo ROI image pairs. "
            "This evaluates image-space detection coverage and disparity only; "
            "it does not change RDF/NED coordinates."
        )
    )
    parser.add_argument(
        "--report-dir",
        action="append",
        type=Path,
        required=True,
        help="Report directory containing roi_images/trial_XX and validation_trial_XX_comparison.csv.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-pairs-per-report", type=int, default=60)
    parser.add_argument("--imgsz", type=int, default=1080)
    parser.add_argument("--conf", type=float, default=0.10)
    parser.add_argument(
        "--device",
        default="auto",
        help="Ultralytics device string. Use 'auto' to prefer cuda:0 when torch reports CUDA available.",
    )
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument(
        "--model",
        action="append",
        nargs=2,
        metavar=("NAME", "PATH"),
        required=True,
        help="Model display name and weight path.",
    )
    return parser.parse_args()


def finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def resolve_device(requested: str) -> str:
    device = str(requested).strip()
    if device.lower() != "auto":
        return device
    try:
        import torch

        if torch.cuda.is_available():
            return "cuda:0"
    except Exception:
        pass
    return "cpu"


def trial_number(path: Path) -> int:
    try:
        return int(path.name.split("_")[-1])
    except ValueError:
        return -1


def frame_number(path: Path) -> int:
    try:
        return int(path.stem.split("_")[-1])
    except ValueError:
        return -1


def load_validation_rows(report_dir: Path, trial: int) -> list[dict[str, str]]:
    path = report_dir / f"validation_trial_{trial:02d}_comparison.csv"
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def collect_pairs(report_dirs: list[Path], max_pairs_per_report: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for report_dir in report_dirs:
        image_root = report_dir / "roi_images"
        trial_dirs = sorted(
            [path for path in image_root.glob("trial_*") if path.is_dir()],
            key=trial_number,
        )
        report_pairs: list[dict[str, Any]] = []
        for trial_dir in trial_dirs:
            trial = trial_number(trial_dir)
            left_paths = sorted(trial_dir.glob("left_*.ppm"), key=frame_number)
            right_by_frame = {frame_number(path): path for path in trial_dir.glob("right_*.ppm")}
            validation_rows = load_validation_rows(report_dir, trial)
            for pair_index, left_path in enumerate(left_paths):
                frame = frame_number(left_path)
                right_path = right_by_frame.get(frame)
                if right_path is None:
                    continue
                meta = validation_rows[pair_index] if pair_index < len(validation_rows) else {}
                report_pairs.append(
                    {
                        "report_dir": str(report_dir),
                        "trial": trial,
                        "frame": frame,
                        "left_path": str(left_path),
                        "right_path": str(right_path),
                        "truth_range_m": finite_float(meta.get("truth_range_m")),
                        "truth_right_m": finite_float(meta.get("truth_right_m")),
                        "truth_down_m": finite_float(meta.get("truth_down_m")),
                        "truth_forward_m": finite_float(meta.get("truth_forward_m")),
                        "baseline_left_cx": finite_float(meta.get("left_cx")),
                        "baseline_right_cx": finite_float(meta.get("right_cx")),
                        "baseline_raw_disparity_px": finite_float(meta.get("raw_disparity_px")),
                    }
                )
        if max_pairs_per_report > 0 and len(report_pairs) > max_pairs_per_report:
            step = len(report_pairs) / max_pairs_per_report
            selected = []
            used: set[int] = set()
            for index in range(max_pairs_per_report):
                source_index = min(len(report_pairs) - 1, int(round(index * step)))
                while source_index in used and source_index + 1 < len(report_pairs):
                    source_index += 1
                used.add(source_index)
                selected.append(report_pairs[source_index])
            report_pairs = selected
        rows.extend(report_pairs)
    return rows


def select_detection(result: Any, names: dict[int, str]) -> dict[str, Any]:
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return {
            "detected": False,
            "selection_policy": "none",
            "class_name": "",
            "confidence": math.nan,
            "cx_px": math.nan,
            "cy_px": math.nan,
            "width_px": math.nan,
            "height_px": math.nan,
            "detection_count": 0,
        }
    candidates = []
    for index in range(len(boxes)):
        cls_id = int(boxes.cls[index].item())
        conf = float(boxes.conf[index].item())
        xyxy = boxes.xyxy[index].tolist()
        class_name = str(names.get(cls_id, cls_id))
        candidates.append((class_name.lower(), conf, cls_id, xyxy))
    target_candidates = [item for item in candidates if item[0] in TARGET_CLASS_NAMES]
    if target_candidates:
        class_name, conf, cls_id, xyxy = max(target_candidates, key=lambda item: item[1])
        policy = "target_class"
    else:
        class_name, conf, cls_id, xyxy = max(candidates, key=lambda item: item[1])
        policy = "highest_conf_fallback"
    x1, y1, x2, y2 = [float(value) for value in xyxy]
    return {
        "detected": True,
        "selection_policy": policy,
        "class_name": names.get(cls_id, str(cls_id)),
        "confidence": conf,
        "cx_px": (x1 + x2) * 0.5,
        "cy_px": (y1 + y2) * 0.5,
        "width_px": x2 - x1,
        "height_px": y2 - y1,
        "detection_count": len(candidates),
    }


def run_model(model_name: str, model_path: str, pairs: list[dict[str, Any]], args: argparse.Namespace) -> list[dict[str, Any]]:
    model = YOLO(model_path)
    names = {int(key): str(value) for key, value in model.names.items()}
    has_target_class = any(value.lower() in TARGET_CLASS_NAMES for value in names.values())
    device = resolve_device(args.device)
    image_paths: list[str] = []
    for pair in pairs:
        image_paths.extend([pair["left_path"], pair["right_path"]])
    started = perf_counter()
    results = []
    batch_size = max(1, int(args.batch_size))
    for start in range(0, len(image_paths), batch_size):
        results.extend(
            model.predict(
                source=image_paths[start : start + batch_size],
                imgsz=args.imgsz,
                conf=args.conf,
                device=device,
                verbose=False,
                stream=False,
            )
        )
    elapsed_s = perf_counter() - started
    if len(results) != len(image_paths):
        raise RuntimeError(f"model returned {len(results)} results for {len(image_paths)} images")
    detection_by_path = {
        str(Path(image_path).resolve()): select_detection(result, names)
        for image_path, result in zip(image_paths, results)
    }
    rows = []
    for pair in pairs:
        left = detection_by_path[str(Path(pair["left_path"]).resolve())]
        right = detection_by_path[str(Path(pair["right_path"]).resolve())]
        both = bool(left["detected"] and right["detected"])
        disparity = (
            abs(float(left["cx_px"]) - float(right["cx_px"]))
            if both
            else math.nan
        )
        baseline_disparity = pair.get("baseline_raw_disparity_px")
        disparity_delta = (
            disparity - float(baseline_disparity)
            if both and baseline_disparity is not None
            else math.nan
        )
        rows.append(
            {
                **pair,
                "model_name": model_name,
                "model_path": model_path,
                "class_count": len(names),
                "has_target_class": has_target_class,
                "requested_device": args.device,
                "resolved_device": device,
                "imgsz": args.imgsz,
                "elapsed_s_total_model": elapsed_s,
                "left_detected": left["detected"],
                "right_detected": right["detected"],
                "both_detected": both,
                "left_policy": left["selection_policy"],
                "right_policy": right["selection_policy"],
                "left_class": left["class_name"],
                "right_class": right["class_name"],
                "left_confidence": left["confidence"],
                "right_confidence": right["confidence"],
                "left_cx_px": left["cx_px"],
                "left_cy_px": left["cy_px"],
                "right_cx_px": right["cx_px"],
                "right_cy_px": right["cy_px"],
                "left_width_px": left["width_px"],
                "left_height_px": left["height_px"],
                "right_width_px": right["width_px"],
                "right_height_px": right["height_px"],
                "left_detection_count": left["detection_count"],
                "right_detection_count": right["detection_count"],
                "raw_disparity_px": disparity,
                "disparity_delta_vs_baseline_px": disparity_delta,
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    summary = []
    for model_name in sorted({str(row["model_name"]) for row in rows}):
        selected = [row for row in rows if row["model_name"] == model_name]
        both = [row for row in selected if row["both_detected"]]
        target_window = [
            row
            for row in selected
            if row["truth_range_m"] is not None and abs(float(row["truth_range_m"]) - 1852.0) <= 30.0
        ]
        target_both = [row for row in target_window if row["both_detected"]]
        disparities = [float(row["raw_disparity_px"]) for row in both if math.isfinite(float(row["raw_disparity_px"]))]
        disparity_deltas = [
            float(row["disparity_delta_vs_baseline_px"])
            for row in both
            if math.isfinite(float(row["disparity_delta_vs_baseline_px"]))
        ]
        confs = [
            min(float(row["left_confidence"]), float(row["right_confidence"]))
            for row in both
            if math.isfinite(float(row["left_confidence"])) and math.isfinite(float(row["right_confidence"]))
        ]
        class_pairs = Counter(f"{row['left_class']}|{row['right_class']}" for row in both)
        policies = Counter(f"{row['left_policy']}|{row['right_policy']}" for row in selected)
        summary.append(
            {
                "model_name": model_name,
                "model_path": selected[0]["model_path"],
                "class_count": selected[0]["class_count"],
                "has_target_class": selected[0]["has_target_class"],
                "requested_device": selected[0]["requested_device"],
                "resolved_device": selected[0]["resolved_device"],
                "imgsz": selected[0]["imgsz"],
                "sampled_pair_count": len(selected),
                "both_detected_count": len(both),
                "both_detected_fraction": len(both) / len(selected) if selected else 0.0,
                "target_1852_pair_count": len(target_window),
                "target_1852_both_detected_count": len(target_both),
                "target_1852_both_detected_fraction": len(target_both) / len(target_window) if target_window else 0.0,
                "median_min_pair_confidence": median(confs) if confs else math.nan,
                "mean_disparity_px": mean(disparities) if disparities else math.nan,
                "median_abs_disparity_delta_vs_baseline_px": median(abs(value) for value in disparity_deltas) if disparity_deltas else math.nan,
                "mean_abs_disparity_delta_vs_baseline_px": mean(abs(value) for value in disparity_deltas) if disparity_deltas else math.nan,
                "elapsed_s_total_model": selected[0]["elapsed_s_total_model"],
                "top_class_pairs": ";".join(f"{key}:{value}" for key, value in class_pairs.most_common(5)),
                "selection_policies": ";".join(f"{key}:{value}" for key, value in policies.most_common()),
            }
        )
    return summary


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    pairs = collect_pairs(args.report_dir, args.max_pairs_per_report)
    if not pairs:
        raise RuntimeError("no left/right ROI image pairs found")
    write_csv(args.output_dir / "sampled_pairs.csv", pairs)
    all_rows: list[dict[str, Any]] = []
    for model_name, model_path in args.model:
        all_rows.extend(run_model(model_name, model_path, pairs, args))
    write_csv(args.output_dir / "per_pair_detections.csv", all_rows)
    write_csv(args.output_dir / "model_summary.csv", summarize(all_rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
