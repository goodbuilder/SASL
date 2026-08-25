#!/usr/bin/env python3
"""Benchmark one real stereo video frame pair through Ultralytics batch inference."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Any


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return math.nan
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def evaluate_acceptance(report: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if report.get("requested_device", "").startswith("cuda"):
        if report.get("backend_device", "").startswith("cuda") is False:
            errors.append("actual backend is not CUDA")
        if report.get("backend_precision") != "fp16":
            errors.append("actual backend precision is not FP16")
        if report.get("cuda_tensor_probe") is not True:
            errors.append("CUDA tensor probe failed")
    if float(report.get("pair_update_hz", 0.0)) < float(report["minimum_pair_update_hz"]):
        errors.append("pair update throughput is below the minimum")
    if float(report.get("wall_time_ms_p95", math.inf)) > float(report["maximum_p95_ms"]):
        errors.append("pair inference wall-time p95 exceeds the maximum")
    if int(report.get("timed_iterations", 0)) < 1:
        errors.append("no timed inference iterations were recorded")
    return errors


def _read_video_frame(path: Path, fraction: float):
    import cv2

    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open video: {path}")
    try:
        frame_count = max(1, int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
        frame_index = min(frame_count - 1, max(0, int(round((frame_count - 1) * fraction))))
        capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, bgr = capture.read()
        if not ok or bgr is None:
            raise RuntimeError(f"cannot read frame {frame_index} from {path}")
        return bgr[:, :, ::-1].copy(), frame_index, frame_count
    finally:
        capture.release()


def _target_class_ids(model, class_name: str):
    normalized = class_name.strip().lower()
    if normalized in ("", "any", "all", "*"):
        return None
    matches = [
        int(class_id)
        for class_id, name in (getattr(model, "names", {}) or {}).items()
        if str(name).strip().lower() == normalized
    ]
    return matches or None


def _detection_counts(results) -> list[int]:
    counts = []
    for result in results[:2]:
        boxes = getattr(result, "boxes", None)
        counts.append(len(boxes) if boxes is not None else 0)
    return counts


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure actual Ultralytics batch=2 stereo-pair inference on saved MP4 frames."
    )
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--left-video", type=Path, required=True)
    parser.add_argument("--right-video", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--imgsz", type=int, default=736)
    parser.add_argument("--confidence", type=float, default=0.10)
    parser.add_argument("--class-name", default="UAV")
    parser.add_argument("--frame-fraction", type=float, default=0.5)
    parser.add_argument("--warmup-iterations", type=int, default=10)
    parser.add_argument("--iterations", type=int, default=60)
    parser.add_argument("--minimum-pair-update-hz", type=float, default=15.0)
    parser.add_argument("--maximum-p95-ms", type=float, default=66.7)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not 0.0 <= args.frame_fraction <= 1.0:
        raise SystemExit("--frame-fraction must be between 0 and 1")
    if args.warmup_iterations < 1 or args.iterations < 1:
        raise SystemExit("warmup and timed iteration counts must be positive")
    for path in (args.model, args.left_video, args.right_video):
        if not path.is_file():
            raise SystemExit(f"missing input: {path}")

    import cv2
    import torch
    import torchvision
    import ultralytics
    from ultralytics import YOLO

    os.environ.setdefault("SASL_YOLO_HOST_THREADS", "1")
    torch.set_num_threads(max(1, int(os.environ["SASL_YOLO_HOST_THREADS"])))
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
    cv2.setNumThreads(max(1, int(os.environ["SASL_YOLO_HOST_THREADS"])))

    requested_cuda = args.device.startswith("cuda")
    cuda_tensor_probe = False
    gpu_name = ""
    if requested_cuda:
        if not torch.cuda.is_available():
            raise SystemExit("CUDA is unavailable; refusing a CPU benchmark fallback")
        torch.zeros(1, device=args.device)
        torch.cuda.synchronize()
        cuda_tensor_probe = True
        gpu_name = torch.cuda.get_device_name(0)

    left_rgb, left_index, left_count = _read_video_frame(
        args.left_video, args.frame_fraction
    )
    right_rgb, right_index, right_count = _read_video_frame(
        args.right_video, args.frame_fraction
    )
    model = YOLO(str(args.model.resolve()))
    if hasattr(model, "to"):
        model.to(args.device)
    target_ids = _target_class_ids(model, args.class_name)
    predict_kwargs: dict[str, Any] = {
        "source": [left_rgb, right_rgb],
        "conf": args.confidence,
        "device": args.device,
        "imgsz": args.imgsz,
        "max_det": 1,
        "verbose": False,
    }
    if target_ids is not None:
        predict_kwargs["classes"] = target_ids
    if requested_cuda:
        predict_kwargs["quantize"] = 16

    results = None
    with torch.inference_mode():
        for _ in range(args.warmup_iterations):
            results = model.predict(**predict_kwargs)
        if requested_cuda:
            torch.cuda.synchronize()
        timings_ms: list[float] = []
        for _ in range(args.iterations):
            started = time.perf_counter()
            results = model.predict(**predict_kwargs)
            if requested_cuda:
                torch.cuda.synchronize()
            timings_ms.append((time.perf_counter() - started) * 1000.0)

    predictor = getattr(model, "predictor", None)
    backend = getattr(predictor, "model", None)
    backend_device = str(getattr(backend, "device", "unknown"))
    backend_precision = "fp16" if bool(getattr(backend, "fp16", False)) else "fp32"
    total_s = sum(timings_ms) / 1000.0
    report: dict[str, Any] = {
        "schema_version": "simulation_yolo_pair_benchmark_v1",
        "scope": "ultralytics_batch2_pair_predict_wall_time",
        "python": sys.executable,
        "torch_version": torch.__version__,
        "torchvision_version": torchvision.__version__,
        "ultralytics_version": ultralytics.__version__,
        "opencv_version": cv2.__version__,
        "torch_cuda_build": torch.version.cuda,
        "cuda_available": bool(torch.cuda.is_available()),
        "cuda_tensor_probe": cuda_tensor_probe,
        "gpu_name": gpu_name,
        "requested_device": args.device,
        "backend_device": backend_device,
        "backend_precision": backend_precision,
        "model_path": str(args.model.resolve()),
        "model_sha256": sha256_file(args.model),
        "imgsz": args.imgsz,
        "confidence": args.confidence,
        "class_name": args.class_name,
        "target_class_ids": target_ids,
        "batch_size": 2,
        "left_video": str(args.left_video.resolve()),
        "right_video": str(args.right_video.resolve()),
        "left_video_sha256": sha256_file(args.left_video),
        "right_video_sha256": sha256_file(args.right_video),
        "frame_fraction": args.frame_fraction,
        "left_frame_index": left_index,
        "right_frame_index": right_index,
        "left_video_frame_count": left_count,
        "right_video_frame_count": right_count,
        "warmup_iterations": args.warmup_iterations,
        "timed_iterations": args.iterations,
        "wall_time_ms_median": statistics.median(timings_ms),
        "wall_time_ms_p95": percentile(timings_ms, 0.95),
        "wall_time_ms_max": max(timings_ms),
        "pair_update_hz": args.iterations / total_s if total_s > 0.0 else 0.0,
        "minimum_pair_update_hz": args.minimum_pair_update_hz,
        "maximum_p95_ms": args.maximum_p95_ms,
        "last_detection_counts": _detection_counts(results or []),
    }
    errors = evaluate_acceptance(report)
    report["accepted"] = not errors
    report["errors"] = errors
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(args.output.name + ".tmp")
    temporary.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(args.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
