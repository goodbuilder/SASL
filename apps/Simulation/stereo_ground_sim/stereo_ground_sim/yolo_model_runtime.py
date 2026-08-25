"""YOLO model loading, device selection, warmup, and inference runtime.

Input images are RGB uint8 arrays. Returned detection centres and bounding boxes
use OpenCV image coordinates with top-left origin, u/right and v/down pixels.
This component performs no stereo or world-frame coordinate conversion.
"""

import os
import multiprocessing
import threading
import time
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import shared_memory

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
    "yolo_backend_verified",
    "yolo_backend_device",
    "yolo_backend_precision",
    "yolo_host_runtime_configured",
    "yolo_host_intraop_threads",
    "yolo_process_executor",
    "yolo_process_shared_memory",
    "yolo_process_shared_layout",
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
        self.yolo_backend_verified = False
        self.yolo_backend_device = ""
        self.yolo_backend_precision = ""
        self.yolo_host_runtime_configured = False
        self.yolo_host_intraop_threads = 0
        self.yolo_process_executor = None
        self.yolo_process_shared_memory = None
        self.yolo_process_shared_layout = None

    def _reset_yolo_model(self):
        with self.node.yolo_model_lock:
            self._shutdown_yolo_process()
            self.node.yolo_model = None
            self.node.yolo_model_device = None
            self.node.yolo_model_loaded_path = None
            self.node.yolo_load_error = ""
            self.node.yolo_runtime_disabled = False
            self.node.yolo_runtime_disable_reason = ""
            self.node.yolo_runtime_warning_logged = False
            self.node.yolo_class_warning_logged = False
            self.node.yolo_backend_verified = False
            self.node.yolo_backend_device = ""
            self.node.yolo_backend_precision = ""

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
            try:
                device = self.node.resolved_yolo_device or self.node._resolve_yolo_device()
                warmup_image = np.zeros((self.node.yolo_imgsz, self.node.yolo_imgsz, 3), dtype=np.uint8)
                if self._use_yolo_process(device):
                    started = time.monotonic()
                    self._predict_pair_in_process(
                        warmup_image, warmup_image, device, timeout_s=60.0
                    )
                    elapsed_ms = (time.monotonic() - started) * 1000.0
                    if self.node.yolo_load_error:
                        raise RuntimeError(self.node.yolo_load_error or "YOLO process warmup failed")
                    self.node.yolo_warmed_up = True
                    self.node.get_logger().info(
                        f"YOLO process warmup completed in {elapsed_ms:.1f} ms."
                    )
                    return
                model = self.node._load_yolo_model()
                if model is None:
                    return
                device = self.node._move_yolo_model(model, device)
                predict_kwargs = self._prediction_kwargs(
                    warmup_image,
                    device,
                    self.node._target_class_ids(model),
                )
                started = time.monotonic()
                model.predict(**predict_kwargs)
                elapsed_ms = (time.monotonic() - started) * 1000.0
                self._verify_prediction_backend(model, device)
                self.node.yolo_warmed_up = True
                self.node.get_logger().info(f"YOLO warmup completed in {elapsed_ms:.1f} ms.")
            except Exception as exc:
                self.node.yolo_load_error = f"YOLO warmup failed: {exc}"
                self.node.get_logger().warning(self.node.yolo_load_error)

    def _prediction_kwargs(self, source, device, target_ids):
        """Build one best-target inference request for a single stereo pair.

        Stereo triangulation consumes at most one centre per camera. Filtering
        the target class before NMS and limiting retained boxes avoids spending
        CPU post-processing time on detections that the caller would discard.
        Input arrays remain RGB/OpenCV optical images; this helper performs no
        coordinate or frame conversion.
        """

        kwargs = {
            "source": source,
            "conf": float(self.node.yolo_confidence),
            "device": device,
            "imgsz": int(self.node.yolo_imgsz),
            "max_det": 1,
            "verbose": False,
        }
        if target_ids is not None:
            kwargs["classes"] = list(target_ids)
        if device.startswith("cuda"):
            kwargs["quantize"] = 16
        return kwargs

    def _verify_prediction_backend(self, model, requested_device):
        """Record the backend device and precision created by Ultralytics.

        Ultralytics 8.4 uses ``quantize=16`` as the canonical FP16 predict
        option.  Inspect the initialized predictor instead of assuming that a
        CUDA request was honored, because a silent CPU/FP32 path would make
        target-rate tuning misleading.
        """

        predictor = getattr(model, "predictor", None)
        backend = getattr(predictor, "model", None)
        backend_device = str(getattr(backend, "device", "unknown"))
        backend_fp16 = bool(getattr(backend, "fp16", False))
        backend_precision = "fp16" if backend_fp16 else "fp32"
        self.node.yolo_backend_device = backend_device
        self.node.yolo_backend_precision = backend_precision
        self.node.yolo_backend_verified = True

        requested_cuda = str(requested_device).startswith("cuda")
        verified_cuda_fp16 = backend_device.startswith("cuda") and backend_fp16
        message = (
            "YOLO prediction backend verified: "
            f"requested={requested_device}, actual={backend_device}, "
            f"precision={backend_precision}."
        )
        if requested_cuda and not verified_cuda_fp16:
            if not getattr(self.node, "yolo_allow_cpu_fallback", True):
                raise RuntimeError(
                    f"{message} CUDA FP16 was required and CPU/FP32 fallback is disabled."
                )
            self.node.get_logger().warning(message)
        else:
            self.node.get_logger().info(message)

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
            predict_kwargs = self._prediction_kwargs(image, device, target_ids)
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
        if device is None:
            device = self.node._resolve_yolo_device()
        if self._use_yolo_process(device):
            with self.node.yolo_model_lock:
                return self._predict_pair_in_process(
                    left_image, right_image, device
                )
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
            predict_kwargs = self._prediction_kwargs(
                [left_image, right_image], device, target_ids
            )
            try:
                results = model.predict(**predict_kwargs)
            except Exception as exc:
                self.node.yolo_load_error = f"YOLO predict failed: {exc}"
                return None, None
        detections = [self.node._best_detection_from_result(result, target_ids) for result in results[:2]]
        while len(detections) < 2:
            detections.append(None)
        return detections[0], detections[1]

    @staticmethod
    def _use_yolo_process(device):
        enabled = os.environ.get("SASL_YOLO_PROCESS_ISOLATION", "1").strip().lower()
        return str(device).startswith("cuda") and enabled not in (
            "0", "false", "no", "off"
        )

    def _ensure_yolo_process(self):
        executor = self.node.yolo_process_executor
        if executor is None:
            executor = ProcessPoolExecutor(
                max_workers=1,
                mp_context=multiprocessing.get_context("spawn"),
            )
            self.node.yolo_process_executor = executor
        return executor

    def _process_model_source(self):
        model_path = self.node.yolo_model_path.strip()
        if not model_path:
            self.node.yolo_load_error = "set yolo_model_path"
            return None
        expanded = os.path.expanduser(model_path)
        allow_named_model = "/" not in expanded and (
            os.altsep is None or os.altsep not in expanded
        )
        if not allow_named_model and not os.path.isabs(expanded):
            expanded = os.path.abspath(expanded)
        if not allow_named_model and not os.path.exists(expanded):
            self.node._disable_yolo_runtime(f"YOLO pt not found: {expanded}")
            return None
        return model_path if allow_named_model else expanded

    def _predict_pair_in_process(
        self, left_image, right_image, device, *, timeout_s=10.0
    ):
        from stereo_ground_sim.yolo_process_inference import predict_yolo_pair

        model_source = self._process_model_source()
        if model_source is None or self.node.yolo_runtime_disabled:
            return None, None
        shared_images = self._copy_pair_to_shared_memory(left_image, right_image)
        request = {
            "model_source": model_source,
            "class_name": self.node.yolo_class_name,
            "confidence": float(self.node.yolo_confidence),
            "device": str(device),
            "imgsz": int(self.node.yolo_imgsz),
            "shared_images": shared_images,
        }
        try:
            result = self._ensure_yolo_process().submit(
                predict_yolo_pair, request
            ).result(timeout=timeout_s)
        except Exception as exc:
            self.node.yolo_load_error = f"YOLO process predict failed: {exc}"
            return None, None
        error = result.get("error", "")
        if error:
            self.node.yolo_load_error = f"YOLO process predict failed: {error}"
            return None, None
        if result.get("class_missing") and not self.node.yolo_class_warning_logged:
            self.node.get_logger().warning(
                f"class not found in pt: {self.node.yolo_class_name}; "
                "using highest-confidence detections from the model."
            )
            self.node.yolo_class_warning_logged = True
        self._verify_process_backend(result, device)
        self.node.yolo_load_error = ""
        return result.get("left"), result.get("right")

    def _verify_process_backend(self, result, requested_device):
        backend_device = result.get("backend_device", "unknown")
        backend_precision = result.get("backend_precision", "unknown")
        self.node.yolo_backend_device = backend_device
        self.node.yolo_backend_precision = backend_precision
        if self.node.yolo_backend_verified:
            return
        self.node.yolo_backend_verified = True
        message = (
            "YOLO process backend verified: "
            f"requested={requested_device}, actual={backend_device}, "
            f"precision={backend_precision}."
        )
        if str(requested_device).startswith("cuda") and not (
            str(backend_device).startswith("cuda") and backend_precision == "fp16"
        ):
            if not getattr(self.node, "yolo_allow_cpu_fallback", True):
                raise RuntimeError(
                    f"{message} CUDA FP16 was required and CPU/FP32 fallback is disabled."
                )
            self.node.get_logger().warning(message)
        else:
            self.node.get_logger().info(message)

    def _shutdown_yolo_process(self):
        executor = getattr(self.node, "yolo_process_executor", None)
        if executor is None:
            self._release_yolo_shared_memory()
            return
        self.node.yolo_process_executor = None
        executor.shutdown(wait=True, cancel_futures=True)
        self._release_yolo_shared_memory()

    def _copy_pair_to_shared_memory(self, left_image, right_image):
        images = (np.ascontiguousarray(left_image), np.ascontiguousarray(right_image))
        layout = tuple((image.shape, image.dtype.str) for image in images)
        buffers = getattr(self.node, "yolo_process_shared_memory", None)
        if buffers is None or self.node.yolo_process_shared_layout != layout:
            self._release_yolo_shared_memory()
            buffers = tuple(
                shared_memory.SharedMemory(create=True, size=image.nbytes)
                for image in images
            )
            self.node.yolo_process_shared_memory = buffers
            self.node.yolo_process_shared_layout = layout
        descriptors = []
        for image, buffer in zip(images, buffers):
            shared_array = np.ndarray(
                image.shape, dtype=image.dtype, buffer=buffer.buf
            )
            np.copyto(shared_array, image)
            descriptors.append(
                {
                    "name": buffer.name,
                    "shape": image.shape,
                    "dtype": image.dtype.str,
                }
            )
        return descriptors

    def _release_yolo_shared_memory(self):
        buffers = getattr(self.node, "yolo_process_shared_memory", None)
        self.node.yolo_process_shared_memory = None
        self.node.yolo_process_shared_layout = None
        if buffers is None:
            return
        for buffer in buffers:
            try:
                buffer.close()
            except Exception:
                pass
            try:
                buffer.unlink()
            except FileNotFoundError:
                pass
            except Exception:
                pass

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
                if not getattr(self.node, "yolo_allow_cpu_fallback", True):
                    raise RuntimeError(
                        "YOLO device is auto, CUDA is unavailable, and CPU fallback is disabled. "
                        + self.node._cuda_status_message(torch)
                    )
                if not self.node.yolo_device_warning_logged:
                    self.node.get_logger().warning(
                        "YOLO device is auto, but CUDA is not available; using CPU. "
                        + self.node._cuda_status_message(torch)
                    )
                    self.node.yolo_device_warning_logged = True
                self.node.yolo_cpu_fallback = True
            except Exception as exc:
                if not getattr(self.node, "yolo_allow_cpu_fallback", True):
                    raise RuntimeError(
                        f"YOLO auto-device CUDA check failed while CPU fallback is disabled: {exc}"
                    ) from exc
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
                if not getattr(self.node, "yolo_allow_cpu_fallback", True):
                    raise RuntimeError(
                        f"Requested YOLO device '{device}' failed its CUDA check and CPU fallback is disabled: {exc}"
                    ) from exc
                if not self.node.yolo_device_warning_logged:
                    self.node.get_logger().warning(
                        f"Requested YOLO device '{device}', but torch CUDA check failed: {exc}; "
                        "falling back to CPU."
                    )
                    self.node.yolo_device_warning_logged = True
                self.node.yolo_cpu_fallback = True
                return "cpu"
            if not getattr(self.node, "yolo_allow_cpu_fallback", True):
                raise RuntimeError(
                    f"Requested YOLO device '{device}', but CUDA is unavailable and CPU fallback is disabled. "
                    + self.node._cuda_status_message(torch)
                )
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
        self._configure_yolo_host_runtime(torch_module)
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

    def _configure_yolo_host_runtime(self, torch_module):
        """Bound host-side inference pools used by the CUDA YOLO path.

        ROS 2 already owns an executor pool and Ultralytics adds preprocessing
        work.  Letting PyTorch and OpenCV create a CPU-sized pool each causes
        severe oversubscription inside this process even though the tensor
        inference itself runs on CUDA.  The environment override remains a
        machine-local tuning knob; it does not change the RGB/OpenCV optical
        pixel contract or any stereo/world coordinate transform.
        """

        if getattr(self.node, "yolo_host_runtime_configured", False):
            return
        try:
            thread_count = max(1, int(os.environ.get("SASL_YOLO_HOST_THREADS", "1")))
        except ValueError:
            thread_count = 1
        try:
            torch_module.set_num_threads(thread_count)
        except Exception:
            pass
        try:
            # PyTorch permits this call only before inter-op work starts.
            torch_module.set_num_interop_threads(1)
        except RuntimeError:
            pass
        except Exception:
            pass
        try:
            import cv2

            cv2.setNumThreads(thread_count)
        except Exception:
            pass
        self.node.yolo_host_intraop_threads = thread_count
        self.node.yolo_host_runtime_configured = True
        self.node.get_logger().info(
            "YOLO host runtime configured: "
            f"PyTorch/OpenCV threads={thread_count}, PyTorch inter-op threads=1."
        )

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
            if device.startswith("cuda") and not getattr(
                self.node, "yolo_allow_cpu_fallback", True
            ):
                raise RuntimeError(
                    f"{self.node.yolo_load_error}; CPU fallback is disabled"
                ) from exc
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
