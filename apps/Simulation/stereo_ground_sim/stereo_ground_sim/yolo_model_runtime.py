"""YOLO model loading, device selection, warmup, and inference runtime.

Input images are RGB uint8 arrays. Returned detection centres and bounding boxes
use OpenCV image coordinates with top-left origin, u/right and v/down pixels.
This component performs no stereo or world-frame coordinate conversion.
"""

import os
import threading
import time

import numpy as np


YOLO_RUNTIME_STATE_FIELDS = (
    "yolo_model",
    "yolo_model_device",
    "yolo_model_loaded_path",
    "yolo_model_lock",
    "yolo_load_error",
    "resolved_yolo_device",
    "yolo_device_warning_logged",
    "yolo_device_info_logged",
    "yolo_runtime_disabled",
    "yolo_runtime_disable_reason",
    "yolo_runtime_warning_logged",
    "yolo_class_warning_logged",
    "yolo_cpu_fallback",
    "yolo_warmed_up",
)


class YoloModelRuntime:
    def __init__(self, node):
        self.node = node
        self.yolo_model = None
        self.yolo_model_device = None
        self.yolo_model_loaded_path = None
        self.yolo_model_lock = threading.RLock()
        self.yolo_load_error = ""
        self.resolved_yolo_device = None
        self.yolo_device_warning_logged = False
        self.yolo_device_info_logged = False
        self.yolo_runtime_disabled = False
        self.yolo_runtime_disable_reason = ""
        self.yolo_runtime_warning_logged = False
        self.yolo_class_warning_logged = False
        self.yolo_cpu_fallback = False
        self.yolo_warmed_up = False

    def _reset_yolo_model(self):
        with self.node.yolo_model_lock:
            self.node.yolo_model = None
            self.node.yolo_model_device = None
            self.node.yolo_model_loaded_path = None
            self.node.yolo_load_error = ""
            self.node.yolo_runtime_disabled = False
            self.node.yolo_runtime_disable_reason = ""
            self.node.yolo_runtime_warning_logged = False
            self.node.yolo_class_warning_logged = False

    def _load_yolo_model(self):
        if self.node.yolo_runtime_disabled:
            self.node.yolo_load_error = self.node.yolo_runtime_disable_reason
            return None
        model_path = self.node.yolo_model_path.strip()
        if not model_path:
            self.node.yolo_load_error = "set yolo_model_path"
            return None
        expanded = os.path.expanduser(model_path)
        allow_named_model = "/" not in expanded and (os.altsep is None or os.altsep not in expanded)
        if not allow_named_model and not os.path.isabs(expanded):
            expanded = os.path.abspath(expanded)
        if not allow_named_model and not os.path.exists(expanded):
            self.node._disable_yolo_runtime(f"YOLO pt not found: {expanded}")
            return None
        model_source = model_path if allow_named_model else expanded
        if self.node.yolo_model is not None and self.node.yolo_model_loaded_path == model_source:
            return self.node.yolo_model
        try:
            from ultralytics import YOLO
        except Exception as exc:
            self.node._disable_yolo_runtime(f"ultralytics import failed: {exc}")
            return None
        try:
            self.node.yolo_model = YOLO(model_source)
            self.node.yolo_model_loaded_path = model_source
            self.node.yolo_load_error = ""
            self.node.get_logger().info(f"Loaded YOLO model: {model_source}")
        except Exception as exc:
            self.node._reset_yolo_model()
            self.node._disable_yolo_runtime(f"YOLO load failed: {exc}")
        return self.node.yolo_model

    def _warm_yolo_model(self):
        if self.node.yolo_warmed_up:
            return
        with self.node.yolo_model_lock:
            if self.node.yolo_warmed_up:
                return
            model = self.node._load_yolo_model()
            if model is None:
                return
            try:
                device = self.node._move_yolo_model(model, self.node.resolved_yolo_device or self.node._resolve_yolo_device())
                warmup_image = np.zeros((self.node.yolo_imgsz, self.node.yolo_imgsz, 3), dtype=np.uint8)
                predict_kwargs = {
                    "source": warmup_image,
                    "conf": float(self.node.yolo_confidence),
                    "device": device,
                    "imgsz": int(self.node.yolo_imgsz),
                    "verbose": False,
                }
                if device.startswith("cuda"):
                    predict_kwargs["quantize"] = 16
                started = time.monotonic()
                model.predict(**predict_kwargs)
                elapsed_ms = (time.monotonic() - started) * 1000.0
                self.node.yolo_warmed_up = True
                self.node.get_logger().info(f"YOLO warmup completed in {elapsed_ms:.1f} ms.")
            except Exception as exc:
                self.node.yolo_load_error = f"YOLO warmup failed: {exc}"
                self.node.get_logger().warning(self.node.yolo_load_error)

    def _target_class_ids(self, model):
        class_name = self.node.yolo_class_name.strip().lower()
        if class_name in ("", "any", "all", "*"):
            return None
        names = getattr(model, "names", {}) or {}
        target_ids = []
        for class_id, name in names.items():
            if str(name).strip().lower() == class_name:
                target_ids.append(int(class_id))
        if not target_ids:
            if not self.node.yolo_class_warning_logged:
                self.node.get_logger().warning(
                    f"class not found in pt: {self.node.yolo_class_name}; using highest-confidence detections from the model."
                )
                self.node.yolo_class_warning_logged = True
            return None
        return target_ids

    def _detect_drone(self, image, device=None):
        with self.node.yolo_model_lock:
            model = self.node._load_yolo_model()
            if model is None:
                return None
            target_ids = self.node._target_class_ids(model)
            if target_ids == []:
                return None
            if device is None:
                device = self.node._resolve_yolo_device()
            device = self.node._move_yolo_model(model, device)
            predict_kwargs = {
                "source": image,
                "conf": float(self.node.yolo_confidence),
                "device": device,
                "imgsz": int(self.node.yolo_imgsz),
                "verbose": False,
            }
            if device.startswith("cuda"):
                predict_kwargs["quantize"] = 16
            try:
                results = model.predict(**predict_kwargs)
            except Exception as exc:
                self.node.yolo_load_error = f"YOLO predict failed: {exc}"
                return None
        if not results:
            return None
        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None

        best = None
        for box in boxes:
            try:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
            except Exception:
                continue
            if target_ids is not None and class_id not in target_ids:
                continue
            x1, y1, x2, y2 = xyxy.tolist()
            if best is None or confidence > best["confidence"]:
                best = {
                    "cx": (x1 + x2) * 0.5,
                    "cy": (y1 + y2) * 0.5,
                    "bbox": (x1, y1, x2, y2),
                    "confidence": confidence,
                    "class_id": class_id,
                }
        return best

    def _detect_drone_pair(self, left_image, right_image, device=None):
        with self.node.yolo_model_lock:
            model = self.node._load_yolo_model()
            if model is None:
                return None, None
            target_ids = self.node._target_class_ids(model)
            if target_ids == []:
                return None, None
            if device is None:
                device = self.node._resolve_yolo_device()
            device = self.node._move_yolo_model(model, device)
            predict_kwargs = {
                "source": [left_image, right_image],
                "conf": float(self.node.yolo_confidence),
                "device": device,
                "imgsz": int(self.node.yolo_imgsz),
                "verbose": False,
            }
            if device.startswith("cuda"):
                predict_kwargs["quantize"] = 16
            try:
                results = model.predict(**predict_kwargs)
            except Exception as exc:
                self.node.yolo_load_error = f"YOLO predict failed: {exc}"
                return None, None
        detections = [self.node._best_detection_from_result(result, target_ids) for result in results[:2]]
        while len(detections) < 2:
            detections.append(None)
        return detections[0], detections[1]

    def _best_detection_from_result(self, result, target_ids):
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None

        best = None
        for box in boxes:
            try:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
            except Exception:
                continue
            if target_ids is not None and class_id not in target_ids:
                continue
            x1, y1, x2, y2 = xyxy.tolist()
            if best is None or confidence > best["confidence"]:
                best = {
                    "cx": (x1 + x2) * 0.5,
                    "cy": (y1 + y2) * 0.5,
                    "bbox": (x1, y1, x2, y2),
                    "confidence": confidence,
                    "class_id": class_id,
                }
        return best

    def _resolve_yolo_device(self):
        device = self.node.yolo_device.strip().lower()
        if not device:
            return "cpu"
        if device == "auto":
            try:
                import torch

                if torch.cuda.is_available():
                    self.node.yolo_cpu_fallback = False
                    return self.node._log_cuda_device("cuda:0", torch)
                if not self.node.yolo_device_warning_logged:
                    self.node.get_logger().warning(
                        "YOLO device is auto, but CUDA is not available; using CPU. "
                        + self.node._cuda_status_message(torch)
                    )
                    self.node.yolo_device_warning_logged = True
                self.node.yolo_cpu_fallback = True
            except Exception as exc:
                if not self.node.yolo_device_warning_logged:
                    self.node.get_logger().warning(
                        f"YOLO device is auto, but torch CUDA check failed: {exc}; using CPU."
                    )
                    self.node.yolo_device_warning_logged = True
                self.node.yolo_cpu_fallback = True
                return "cpu"
        if device.startswith("cuda"):
            try:
                import torch

                if torch.cuda.is_available():
                    self.node.yolo_cpu_fallback = False
                    return self.node._log_cuda_device(device, torch)
            except Exception as exc:
                if not self.node.yolo_device_warning_logged:
                    self.node.get_logger().warning(
                        f"Requested YOLO device '{device}', but torch CUDA check failed: {exc}; "
                        "falling back to CPU."
                    )
                    self.node.yolo_device_warning_logged = True
                self.node.yolo_cpu_fallback = True
                return "cpu"
            if not self.node.yolo_device_warning_logged:
                self.node.get_logger().warning(
                    f"Requested YOLO device '{device}', but CUDA is not available; falling back to CPU. "
                    + self.node._cuda_status_message(torch)
                )
                self.node.yolo_device_warning_logged = True
            self.node.yolo_cpu_fallback = True
            return "cpu"
        self.node.yolo_cpu_fallback = device == "cpu"
        return device

    def _log_cuda_device(self, device, torch_module):
        if not self.node.yolo_device_info_logged:
            try:
                torch_module.backends.cudnn.benchmark = True
            except Exception:
                pass
            try:
                name = torch_module.cuda.get_device_name(0)
            except Exception:
                name = "unknown"
            self.node.get_logger().info(
                f"Using YOLO CUDA device '{device}' ({name}); FP16 inference enabled."
            )
            self.node.yolo_device_info_logged = True
        return device

    def _move_yolo_model(self, model, device):
        if self.node.yolo_model_device == device:
            return device
        if not hasattr(model, "to"):
            self.node.yolo_model_device = device
            return device
        try:
            model.to(device)
            self.node.yolo_model_device = device
            return device
        except Exception as exc:
            self.node.yolo_load_error = f"YOLO model move to {device} failed: {exc}"
            if device.startswith("cuda") and not self.node.yolo_device_warning_logged:
                self.node.get_logger().warning(
                    f"Could not move YOLO model to '{device}'; falling back to CPU: {exc}"
                )
                self.node.yolo_device_warning_logged = True
            try:
                model.to("cpu")
                self.node.yolo_model_device = "cpu"
                return "cpu"
            except Exception:
                pass
        return self.node.yolo_model_device or "cpu"

    def _cuda_status_message(self, torch_module):
        details = []
        try:
            details.append(f"torch={torch_module.__version__}")
        except Exception:
            pass
        try:
            details.append(f"torch_cuda={torch_module.version.cuda}")
        except Exception:
            pass
        try:
            details.append(f"device_count={torch_module.cuda.device_count()}")
        except Exception:
            pass
        if not os.path.exists("/dev/nvidiactl"):
            details.append("/dev/nvidiactl missing")
        return " ".join(details)
