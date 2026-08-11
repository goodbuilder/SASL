"""Spawn-process YOLO inference for one RGB stereo image pair.

The IPC boundary accepts RGB uint8 arrays and returns detections in OpenCV
image pixels: ``cx/u`` increases right, ``cy/v`` increases down, and bbox order
is ``(x1, y1, x2, y2)``.  No optical RDF, camera, stereo-center, or world-frame
conversion is performed in this process.
"""

import os
from multiprocessing import shared_memory

import numpy as np


_MODEL = None
_MODEL_SOURCE = None
_MODEL_DEVICE = None


def _configure_host_runtime(torch_module, thread_count):
    try:
        torch_module.set_num_threads(thread_count)
    except Exception:
        pass
    try:
        torch_module.set_num_interop_threads(1)
    except Exception:
        pass
    try:
        import cv2

        cv2.setNumThreads(thread_count)
    except Exception:
        pass
    try:
        torch_module.backends.cudnn.benchmark = True
    except Exception:
        pass


def _target_class_ids(model, class_name):
    normalized = str(class_name).strip().lower()
    if normalized in ("", "any", "all", "*"):
        return None, False
    names = getattr(model, "names", {}) or {}
    target_ids = [
        int(class_id)
        for class_id, name in names.items()
        if str(name).strip().lower() == normalized
    ]
    return (target_ids, False) if target_ids else (None, True)


def _best_detection(result, target_ids):
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return None
    best = None
    for box in boxes:
        try:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            values = box.xyxy[0].detach().cpu().numpy().astype(float).tolist()
        except Exception:
            continue
        if target_ids is not None and class_id not in target_ids:
            continue
        x1, y1, x2, y2 = values
        if best is None or confidence > best["confidence"]:
            best = {
                "cx": (x1 + x2) * 0.5,
                "cy": (y1 + y2) * 0.5,
                "bbox": (x1, y1, x2, y2),
                "confidence": confidence,
                "class_id": class_id,
            }
    return best


def predict_yolo_pair(request):
    """Load/cache the model and infer one serialized RGB stereo pair."""

    global _MODEL, _MODEL_SOURCE, _MODEL_DEVICE
    try:
        import torch
        from ultralytics import YOLO

        try:
            thread_count = max(
                1, int(os.environ.get("SASL_YOLO_HOST_THREADS", "1"))
            )
        except ValueError:
            thread_count = 1
        _configure_host_runtime(torch, thread_count)
        model_source = request["model_source"]
        device = request["device"]
        if _MODEL is None or _MODEL_SOURCE != model_source:
            _MODEL = YOLO(model_source)
            _MODEL_SOURCE = model_source
            _MODEL_DEVICE = None
        if _MODEL_DEVICE != device and hasattr(_MODEL, "to"):
            _MODEL.to(device)
            _MODEL_DEVICE = device
        target_ids, class_missing = _target_class_ids(
            _MODEL, request["class_name"]
        )
        shared_buffers = [
            shared_memory.SharedMemory(name=descriptor["name"])
            for descriptor in request["shared_images"]
        ]
        images = [
            np.ndarray(
                tuple(descriptor["shape"]),
                dtype=np.dtype(descriptor["dtype"]),
                buffer=buffer.buf,
            )
            for descriptor, buffer in zip(
                request["shared_images"], shared_buffers
            )
        ]
        kwargs = {
            "source": images,
            "conf": float(request["confidence"]),
            "device": device,
            "imgsz": int(request["imgsz"]),
            "max_det": 1,
            "verbose": False,
        }
        if target_ids is not None:
            kwargs["classes"] = target_ids
        if str(device).startswith("cuda"):
            kwargs["quantize"] = 16
        try:
            results = _MODEL.predict(**kwargs)
        finally:
            for buffer in shared_buffers:
                buffer.close()
        detections = [_best_detection(result, target_ids) for result in results[:2]]
        while len(detections) < 2:
            detections.append(None)
        predictor = getattr(_MODEL, "predictor", None)
        backend = getattr(predictor, "model", None)
        return {
            "left": detections[0],
            "right": detections[1],
            "class_missing": class_missing,
            "backend_device": str(getattr(backend, "device", "unknown")),
            "backend_precision": (
                "fp16" if bool(getattr(backend, "fp16", False)) else "fp32"
            ),
            "error": "",
        }
    except Exception as exc:
        return {
            "left": None,
            "right": None,
            "class_missing": False,
            "backend_device": "unknown",
            "backend_precision": "unknown",
            "error": f"{type(exc).__name__}: {exc}",
        }
