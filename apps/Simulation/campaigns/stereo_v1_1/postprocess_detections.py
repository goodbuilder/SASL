#!/usr/bin/env python3
"""Run pinned YOLO on every stored source image without changing raw bags.

Original RGB 1280×720 pixels are explicitly letterboxed to a square BGR
input. Inference boxes are converted back to original-image xyxy pixels.
Every source frame, including no detection, keeps a row. Output belongs in a
new derived directory, separate from the immutable original capture.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sqlite3
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
import zstandard
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


CAMPAIGN_ROOT = Path(__file__).resolve().parent
MODEL_MANIFEST = CAMPAIGN_ROOT / "model_manifest.json"


def letterbox_rgb_to_bgr(rgb: np.ndarray, size: int) -> tuple[np.ndarray, dict]:
    if rgb.ndim != 3 or rgb.shape[2] != 3 or size <= 0:
        raise ValueError("expected H×W×3 RGB image and positive square size")
    height, width = rgb.shape[:2]
    scale = min(size / width, size / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    pad_left = (size - resized_width) // 2
    pad_top = (size - resized_height) // 2
    pad_right = size - resized_width - pad_left
    pad_bottom = size - resized_height - pad_top
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    resized = cv2.resize(bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[pad_top:pad_top + resized_height, pad_left:pad_left + resized_width] = resized
    return canvas, {
        "policy": "explicit_square_letterbox_bgr",
        "original_width_px": width,
        "original_height_px": height,
        "inference_width_px": size,
        "inference_height_px": size,
        "scale": scale,
        "resized_width_px": resized_width,
        "resized_height_px": resized_height,
        "pad_left_px": pad_left,
        "pad_top_px": pad_top,
        "pad_right_px": pad_right,
        "pad_bottom_px": pad_bottom,
        "pad_value_bgr": 114,
    }


def unletterbox_xyxy(box, transform: dict) -> list[float]:
    if len(box) != 4:
        raise ValueError("box must be xyxy")
    scale = float(transform["scale"])
    x1 = max(0.0, min(float(transform["original_width_px"]), (float(box[0]) - transform["pad_left_px"]) / scale))
    y1 = max(0.0, min(float(transform["original_height_px"]), (float(box[1]) - transform["pad_top_px"]) / scale))
    x2 = max(0.0, min(float(transform["original_width_px"]), (float(box[2]) - transform["pad_left_px"]) / scale))
    y2 = max(0.0, min(float(transform["original_height_px"]), (float(box[3]) - transform["pad_top_px"]) / scale))
    if x2 <= x1 or y2 <= y1:
        raise ValueError("YOLO box is outside original image after letterbox inverse")
    return [x1, y1, x2, y2]


def inference_due(timestamp_ns: int, next_due_ns: int | None, period_ns: int) -> tuple[bool, int]:
    """Rate-limit on a fixed simulation-time phase, robust to small timestamp jitter."""
    stamp, period = int(timestamp_ns), int(period_ns)
    if period <= 0:
        raise ValueError("inference period must be positive")
    if next_due_ns is None:
        return True, stamp + period
    due = int(next_due_ns)
    jitter_tolerance_ns = max(1_000_000, period // 10)
    if stamp + jitter_tolerance_ns < due:
        return False, due
    missed_periods = max(1, (stamp - due) // period + 1)
    return True, due + missed_periods * period


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_frames(attempt_dir: Path, side: str):
    bag = attempt_dir / "raw_images_or_lossless_bag" / side
    info = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    compression = (info.get("compression_format") or "").lower()
    mode = (info.get("compression_mode") or "").upper()
    if compression not in ("", "zstd") or (compression and mode != "MESSAGE"):
        raise ValueError("unsupported source bag compression")
    decoder = zstandard.ZstdDecompressor() if compression else None
    bags = sorted(bag.glob("*.db3"), key=lambda p: int(p.stem.rsplit("_", 1)[1]))
    for db in bags:
        with sqlite3.connect(f"file:{db}?mode=ro", uri=True) as connection:
            topic = connection.execute("SELECT id FROM topics WHERE name=?", (f"/{side}_camera/image",)).fetchone()
            if topic is None:
                raise ValueError(f"missing {side} image topic in {db.name}")
            for message_id, receive_ns, blob in connection.execute(
                "SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id", (topic[0],)
            ):
                raw = decoder.decompress(blob) if decoder else blob
                yield f"raw_images_or_lossless_bag/{side}/{db.name}#message_id={message_id}", receive_ns, deserialize_message(raw, Image)


def _observation(index: dict, image: Image, model, config: dict, device: str, version: str,
                 *, perform_inference: bool = True) -> dict:
    pixels = bytes(image.data)
    if hashlib.sha256(pixels).hexdigest() != index["image_pixels_sha256"]:
        raise ValueError("raw image pixels differ from image_index hash")
    stamp = int(image.header.stamp.sec) * 1_000_000_000 + int(image.header.stamp.nanosec)
    if stamp != index["image_timestamp_ns"]:
        raise ValueError("raw image source stamp differs from image_index")
    if image.encoding != "rgb8":
        raise ValueError("pinned YOLO input requires rgb8 source image")
    rgb = np.frombuffer(pixels, dtype=np.uint8).reshape(image.height, image.width, 3)
    letterboxed, transform = letterbox_rgb_to_bgr(rgb, int(config["inference_imgsz_px"]))
    result = {
        "run_id": index["run_id"],
        "attempt_id": index["attempt_id"],
        "simulation_epoch": index["simulation_epoch"],
        "camera_id": index["camera_id"],
        "frame_id": index["frame_id"],
        "pair_id": index["pair_id"],
        "image_timestamp_ns": index["image_timestamp_ns"],
        "receive_timestamp_ns": index["receive_timestamp_ns"],
        "receive_time_domain": index["receive_time_domain"],
        "inference_end_timestamp_ns": None,
        "inference_end_time_domain": "offline_replay_utc_ns",
        "detection_status": "NOT_DETECTED",
        "bbox_xyxy_px": None,
        "bbox_center_uv_px": None,
        "confidence": None,
        "image_width_px": int(image.width),
        "image_height_px": int(image.height),
        "original_image_reference": index["original_image_reference"],
        "yolo_weights_sha256": config["sha256"],
        "ultralytics_version": version,
        "inference_mode": ("offline_original_frame" if perform_inference
                           else "offline_original_frame_rate_limited"),
        "inference_device_requested": device,
        "letterbox_transform": transform,
        "inference_error": None,
    }
    if not perform_inference:
        result["detection_status"] = "SKIPPED_RATE_LIMIT"
        return result
    try:
        prediction = model.predict(
            source=letterboxed,
            conf=float(config["confidence"]),
            iou=float(config["nms_iou"]),
            imgsz=int(config["inference_imgsz_px"]),
            max_det=int(config["inference_max_det"]),
            classes=[0], device=device, rect=False, verbose=False,
        )[0]
        if tuple(prediction.orig_shape) != (letterboxed.shape[0], letterboxed.shape[1]):
            raise ValueError("YOLO returned a different inference image shape")
        if prediction.boxes is not None and len(prediction.boxes):
            box = prediction.boxes.xyxy[0].cpu().numpy().tolist()
            xyxy = unletterbox_xyxy(box, transform)
            result["bbox_xyxy_px"] = xyxy
            result["bbox_center_uv_px"] = [(xyxy[0] + xyxy[2]) / 2, (xyxy[1] + xyxy[3]) / 2]
            result["confidence"] = float(prediction.boxes.conf[0].cpu().item())
            result["detection_status"] = "DETECTED"
    except Exception as exc:
        result["detection_status"] = "INFERENCE_ERROR"
        result["inference_error"] = f"{type(exc).__name__}: {exc}"[:500]
    result["inference_end_timestamp_ns"] = time.time_ns()
    return result


def process(attempt_dir: Path, output_dir: Path, model_path: Path,
            device: str, development_version_override: bool = False, *,
            confidence: float | None = None, nms_iou: float | None = None,
            inference_imgsz_px: int | None = None, target_hz: float | None = None,
            parameter_source: str = "pinned_model_manifest") -> dict:
    import ultralytics
    from ultralytics import YOLO

    config = json.loads(MODEL_MANIFEST.read_text(encoding="utf-8"))
    config["confidence"] = config["confidence"] if confidence is None else float(confidence)
    config["nms_iou"] = config["nms_iou"] if nms_iou is None else float(nms_iou)
    config["inference_imgsz_px"] = (config["inference_imgsz_px"] if inference_imgsz_px is None
                                    else int(inference_imgsz_px))
    requested_target_hz = float(target_hz if target_hz is not None else 20.0)
    if (not 0 <= config["confidence"] <= 1 or not 0 < config["nms_iou"] <= 1
            or config["inference_imgsz_px"] <= 0 or not math.isfinite(requested_target_hz)
            or requested_target_hz <= 0):
        raise ValueError("YOLO confidence, IoU, image size, and target rate are invalid")
    if _sha256(model_path) != config["sha256"]:
        raise ValueError("YOLO weight hash differs from pinned manifest")
    version = ultralytics.__version__
    if version != config["ultralytics_version"] and not development_version_override:
        raise ValueError(f"Ultralytics {version} differs from pinned {config['ultralytics_version']}")
    if output_dir.exists():
        raise FileExistsError(f"derived output already exists: {output_dir}")
    image_rows = [json.loads(line) for line in (attempt_dir / "image_index.jsonl").read_text().splitlines()]
    pair_rows = [json.loads(line) for line in (attempt_dir / "pair_index.jsonl").read_text().splitlines()]
    by_source = {row["original_image_reference"]: row for row in image_rows}
    if len(by_source) != len(image_rows):
        raise ValueError("image index contains duplicate original_image_reference")
    model = YOLO(str(model_path))
    if model.names.get(0) != config["class_name"]:
        raise ValueError("YOLO class name differs from pinned manifest")
    output_dir.mkdir(parents=True, exist_ok=False)
    observations = {}
    counts = {}
    inference_stamps = {}
    for side in ("left", "right"):
        observed = []
        next_due_ns = None
        side_inference_stamps = []
        with (output_dir / f"observations_{side}.jsonl").open("x", encoding="utf-8") as handle:
            for reference, receive_ns, image in _source_frames(attempt_dir, side):
                index = by_source.get(reference)
                if index is None or index["camera_id"] != side or index["receive_timestamp_ns"] != receive_ns:
                    raise ValueError("source bag and image index identity differ")
                stamp_ns = int(index["image_timestamp_ns"])
                perform_inference, next_due_ns = inference_due(
                    stamp_ns, next_due_ns, round(1_000_000_000 / requested_target_hz),
                )
                row = _observation(index, image, model, config, device, version,
                                   perform_inference=perform_inference)
                if perform_inference:
                    side_inference_stamps.append(stamp_ns)
                handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
                observed.append(row)
        expected = sum(row["camera_id"] == side for row in image_rows)
        if len(observed) != expected:
            raise ValueError(f"{side} observation rows do not cover every source frame")
        observations.update({row["frame_id"]: row for row in observed})
        inference_stamps[side] = side_inference_stamps
        counts[side] = {
            "source_frames": len(observed),
            "inferred": sum(row["detection_status"] != "SKIPPED_RATE_LIMIT" for row in observed),
            "skipped_rate_limit": sum(row["detection_status"] == "SKIPPED_RATE_LIMIT" for row in observed),
            "detected": sum(row["detection_status"] == "DETECTED" for row in observed),
            "not_detected": sum(row["detection_status"] == "NOT_DETECTED" for row in observed),
            "inference_error": sum(row["detection_status"] == "INFERENCE_ERROR" for row in observed),
        }
    paired_both_detected = 0
    with (output_dir / "paired_observations.jsonl").open("x", encoding="utf-8") as handle:
        for pair in pair_rows:
            left, right = observations[pair["left_frame_id"]], observations[pair["right_frame_id"]]
            both = left["detection_status"] == right["detection_status"] == "DETECTED"
            paired_both_detected += both
            row = {
                **pair,
                "left_detection_status": left["detection_status"],
                "right_detection_status": right["detection_status"],
                "left_bbox_xyxy_px": left["bbox_xyxy_px"],
                "right_bbox_xyxy_px": right["bbox_xyxy_px"],
                "left_confidence": left["confidence"],
                "right_confidence": right["confidence"],
                "valid_yolo_pair": both,
            }
            handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
    summary = {
        "schema_version": "stereo_v1_1_yolo_observation_v1",
        "status": "DEVELOPMENT_ONLY" if development_version_override else "COMPLETE",
        "run_id": image_rows[0]["run_id"],
        "attempt_id": image_rows[0]["attempt_id"],
        "model_sha256": config["sha256"],
        "ultralytics_version": version,
        "pinned_ultralytics_version": config["ultralytics_version"],
        "confidence": config["confidence"],
        "nms_iou": config["nms_iou"],
        "inference_imgsz_px": config["inference_imgsz_px"],
        "requested_target_hz": requested_target_hz,
        "actual_source_simulation_rate_hz": {
            side: ((len(values) - 1) * 1_000_000_000 / (values[-1] - values[0])
                   if len(values) > 1 and values[-1] > values[0] else None)
            for side, values in inference_stamps.items()
        },
        "parameter_source": parameter_source,
        "device_requested": device,
        "counts": counts,
        "nominal_pair_count": len(pair_rows),
        "both_detected_pair_count": paired_both_detected,
        "image_index_sha256": _sha256(attempt_dir / "image_index.jsonl"),
        "pair_index_sha256": _sha256(attempt_dir / "pair_index.jsonl"),
    }
    (output_dir / "observation_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--confidence", type=float)
    parser.add_argument("--nms-iou", type=float)
    parser.add_argument("--imgsz", type=int)
    parser.add_argument("--target-hz", type=float, default=20.0)
    parser.add_argument("--parameter-source", default="pinned_model_manifest")
    parser.add_argument("--development-version-override", action="store_true")
    args = parser.parse_args()
    result = process(args.attempt_dir, args.output_dir, args.model, args.device,
                     args.development_version_override, confidence=args.confidence,
                     nms_iou=args.nms_iou, inference_imgsz_px=args.imgsz,
                     target_hz=args.target_hz, parameter_source=args.parameter_source)
    print(json.dumps({"status": result["status"], "counts": result["counts"],
                      "both_detected_pair_count": result["both_detected_pair_count"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
