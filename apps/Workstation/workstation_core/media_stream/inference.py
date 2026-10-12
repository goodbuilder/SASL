"""inference owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

from typing import Any
from Workstation.workstation_core.error_throttle import should_emit_throttled_error
from .detections import (
    detections_from_ultralytics_result,
    filter_detections_by_class_gate,
    highest_confidence_detection,
)
from .events import _emit_status
from .worker_threads import _configure_torch_threads, _worker_cpu_threads


def _load_yolo_model(config: dict[str, Any], output_queue, enable_detection: bool):
    if not enable_detection:
        return None
    if enable_detection:
        _configure_torch_threads(_worker_cpu_threads(config))
        try:
            from ultralytics import YOLO
        except Exception as exc:
            _emit_status(output_queue, f"YOLO 사용 불가: {exc}")
            return None
        _emit_status(output_queue, "YOLO 모델 로드 중")
        try:
            return YOLO(config["weights_path"])
        except Exception as exc:
            _emit_status(output_queue, f"모델 로드 오류: {exc}")
            return None


def _process_frame_for_output(
    cv2,
    frame,
    *,
    model,
    config: dict[str, Any],
    output_queue,
    now_s: float,
    enable_detection: bool,
    timestamp_metadata: dict[str, Any] | None = None,
):
    height, width = frame.shape[:2]
    best_detection = None
    inferred = False
    if enable_detection:
        class_names = getattr(model, "names", None)
        configured_device = str(config.get("device") or "")
        predict_devices = [configured_device]
        if configured_device and configured_device.lower() != "cpu" and not config.get("_cpu_fallback_attempted"):
            predict_devices.append("cpu")
        for attempt_index, predict_device in enumerate(predict_devices):
            try:
                result = model.predict(
                    frame,
                    imgsz=config["input_size"],
                    conf=config["conf_threshold"],
                    iou=config["iou_threshold"],
                    device=predict_device or None,
                    verbose=False,
                )[0]
                active_device = (predict_device or "auto").strip() or "auto"
                requested_device = str(config.get("_requested_yolo_device") or configured_device or "").strip().lower()
                config["_active_yolo_device"] = active_device
                config["_yolo_cpu_fallback_active"] = active_device.lower() == "cpu" and requested_device not in {
                    "",
                    "cpu",
                }
                if attempt_index > 0:
                    config["device"] = "cpu"
                    config["_cpu_fallback_attempted"] = True
                    _emit_status(output_queue, "CUDA 추론 실패 후 CPU로 전환했습니다.")
                break
            except Exception as exc:
                if attempt_index + 1 < len(predict_devices):
                    config["_cpu_fallback_attempted"] = True
                    config["_yolo_cpu_fallback_active"] = False
                    _emit_status(output_queue, f"추론 오류({exc}); CPU로 재시도합니다.")
                    continue
                throttle_state = config.setdefault("_inference_error_throttle_state", {})
                if should_emit_throttled_error(throttle_state, "inference", str(exc), now_s):
                    _emit_status(output_queue, f"추론 오류: {exc}")
                output_queue.put(
                    (
                        "detection",
                        now_s,
                        width,
                        height,
                        None,
                        [],
                        dict(timestamp_metadata or {}),
                    )
                )
                return frame, None, False
        try:
            detections = filter_detections_by_class_gate(
                detections_from_ultralytics_result(result, names=class_names),
                enabled=bool(config.get("class_gate_enabled", False)),
                target=config.get("class_gate", ""),
            )
            best_detection = highest_confidence_detection(detections)
            inferred = True
            output_queue.put(
                (
                    "detection",
                    now_s,
                    width,
                    height,
                    best_detection.as_dict() if best_detection is not None else None,
                    [detection.as_dict() for detection in detections],
                    dict(timestamp_metadata or {}),
                )
            )
        except Exception as exc:
            _emit_status(output_queue, f"검출 결과 처리 오류: {exc}")
            return frame, None, False
    return frame, best_detection, inferred


def _yolo_device_stats(config: dict[str, Any]) -> tuple[str, str, bool]:
    requested_device = str(config.get("_requested_yolo_device") or config.get("device") or "").strip()
    active_device = str(config.get("_active_yolo_device") or "").strip()
    cpu_fallback = bool(config.get("_yolo_cpu_fallback_active", False))
    return requested_device, active_device, cpu_fallback
