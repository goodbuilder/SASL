"""detections owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
Detection x1/y1/x2/y2 are native pixel u/v corners (u right, v down).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DetectionBox:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int
    class_name: str | None = None

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    def as_dict(self) -> dict[str, float | int | str]:
        center_x, center_y = self.center
        payload: dict[str, float | int | str] = {
            "x1": self.x1,
            "y1": self.y1,
            "x2": self.x2,
            "y2": self.y2,
            "center_x": center_x,
            "center_y": center_y,
            "confidence": self.confidence,
            "class_id": self.class_id,
        }
        if self.class_name:
            payload["class_name"] = self.class_name
        return payload


def _resolve_class_name(names: object | None, class_id: int) -> str:
    if isinstance(names, dict):
        value = names.get(class_id)
        if value is not None:
            return str(value)
    elif isinstance(names, (list, tuple)):
        if 0 <= class_id < len(names):
            value = names[class_id]
            if value is not None:
                return str(value)
    return f"#{class_id}"


def highest_confidence_detection(detections: list[DetectionBox]) -> DetectionBox | None:
    if not detections:
        return None
    return max(detections, key=lambda detection: detection.confidence)


def filter_detections_by_class_gate(
    detections: list[DetectionBox],
    *,
    enabled: bool,
    target: object = "",
) -> list[DetectionBox]:
    """Keep only the operator-selected target class before choosing ``best``.

    The gate is a target-class filter, not the stereo correspondence rule.
    Left/right observations still have to carry the same class downstream;
    this function only prevents a higher-confidence distractor from winning
    before the target-class setting can be applied.
    """

    if not enabled:
        return list(detections)
    text = str(target or "").strip()
    if not text:
        return []
    normalized = text.casefold()
    target_id: int | None = None
    candidate = normalized[1:] if normalized.startswith("#") else normalized
    try:
        target_id = int(candidate)
    except ValueError:
        pass
    if target_id is not None:
        return [detection for detection in detections if detection.class_id == target_id]
    return [
        detection
        for detection in detections
        if detection.class_name is not None and detection.class_name.strip().casefold() == normalized
    ]


def detections_from_ultralytics_result(result, *, names: object | None = None) -> list[DetectionBox]:
    detections: list[DetectionBox] = []
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return detections
    for box in boxes:
        class_id = int(box.cls.item())
        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
        detections.append(
            DetectionBox(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                confidence=float(box.conf.item()),
                class_id=class_id,
                class_name=_resolve_class_name(names, class_id),
            )
        )
    return detections


# Retain legacy pickle names without importing the facade.
DetectionBox.__module__ = "Workstation.workstation_core.yolo_stream"
