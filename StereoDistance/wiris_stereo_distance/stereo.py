from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Mapping


@dataclass(frozen=True)
class DetectionObservation:
    camera: str
    timestamp_s: float
    width: int
    height: int
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int
    class_name: str | None = None
    timestamp_metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @classmethod
    def from_payload(cls, camera: str, payload: Mapping[str, object]) -> "DetectionObservation | None":
        best = payload.get("best_detection")
        if not isinstance(best, Mapping):
            return None
        return cls(
            camera=camera,
            timestamp_s=float(payload["timestamp_s"]),
            width=int(payload["width"]),
            height=int(payload["height"]),
            x1=float(best["x1"]),
            y1=float(best["y1"]),
            x2=float(best["x2"]),
            y2=float(best["y2"]),
            confidence=float(best["confidence"]),
            class_id=int(best["class_id"]),
            class_name=str(best["class_name"]) if best.get("class_name") is not None else None,
            timestamp_metadata=dict(payload.get("timestamp_metadata") or {}),
        )


@dataclass(frozen=True)
class StereoCalibration:
    """Pairing limits only; metric geometry comes exclusively from calibrated rays."""

    max_time_delta_s: float = 0.25
    max_vertical_delta_px: float = 80.0
    require_same_class: bool = True
    timestamp_basis: str = "detection"


@dataclass(frozen=True)
class StereoMatch:
    time_delta_s: float
    vertical_delta_px: float
    left: DetectionObservation
    right: DetectionObservation
    left_extrinsic: Any | None = None
    right_extrinsic: Any | None = None
    extrinsic_time_delta_s: float | None = None
    left_attitude_age_s: float | None = None
    right_attitude_age_s: float | None = None
    attitude_pair_delta_s: float | None = None
    relative_roll_deg: float | None = None
    relative_pitch_deg: float | None = None
    relative_yaw_deg: float | None = None
    sync_quality: str = "unknown"
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class StereoEstimate(StereoMatch):
    range_m: float = 0.0


def detection_timestamp_for_basis(observation: DetectionObservation, timestamp_basis: str = "detection") -> float | None:
    if timestamp_basis in {"camera_rtcp", "camera_rtc_required"}:
        value = observation.timestamp_metadata.get("rtcp_frame_ntp_unix_s")
        if value is not None:
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                parsed = float("nan")
            if isfinite(parsed):
                return parsed
        if timestamp_basis == "camera_rtc_required":
            return None
    return float(observation.timestamp_s)


def match_stereo_observations(
    left: DetectionObservation | None,
    right: DetectionObservation | None,
    calibration: StereoCalibration,
    *,
    left_extrinsic: Any | None = None,
    right_extrinsic: Any | None = None,
    left_attitude_age_s: float | None = None,
    right_attitude_age_s: float | None = None,
    attitude_pair_delta_s: float | None = None,
    relative_roll_deg: float | None = None,
    relative_pitch_deg: float | None = None,
    relative_yaw_deg: float | None = None,
    sync_quality: str = "unknown",
    warnings: tuple[str, ...] = (),
) -> StereoMatch | None:
    if left is None or right is None:
        return None
    _, left_center_y = left.center
    _, right_center_y = right.center
    left_timestamp_s = detection_timestamp_for_basis(left, calibration.timestamp_basis)
    right_timestamp_s = detection_timestamp_for_basis(right, calibration.timestamp_basis)
    if left_timestamp_s is None or right_timestamp_s is None:
        return None
    time_delta = abs(left_timestamp_s - right_timestamp_s)
    if time_delta > calibration.max_time_delta_s:
        return None
    vertical_delta = abs(left_center_y - right_center_y)
    if vertical_delta > calibration.max_vertical_delta_px:
        return None
    if calibration.require_same_class and left.class_id != right.class_id:
        return None
    extrinsic_time_delta_s = None
    if left_extrinsic is not None and right_extrinsic is not None:
        left_ts = getattr(left_extrinsic, "timestamp_s", None)
        right_ts = getattr(right_extrinsic, "timestamp_s", None)
        if left_ts is not None and right_ts is not None:
            extrinsic_time_delta_s = abs(float(left_ts) - float(right_ts))
    return StereoMatch(
        time_delta_s=time_delta,
        vertical_delta_px=vertical_delta,
        left=left,
        right=right,
        left_extrinsic=left_extrinsic,
        right_extrinsic=right_extrinsic,
        extrinsic_time_delta_s=extrinsic_time_delta_s,
        left_attitude_age_s=left_attitude_age_s,
        right_attitude_age_s=right_attitude_age_s,
        attitude_pair_delta_s=attitude_pair_delta_s,
        relative_roll_deg=relative_roll_deg,
        relative_pitch_deg=relative_pitch_deg,
        relative_yaw_deg=relative_yaw_deg,
        sync_quality=sync_quality,
        warnings=warnings,
    )


def stereo_estimate_from_match(match: StereoMatch, *, range_m: float) -> StereoEstimate | None:
    if not isfinite(range_m) or range_m <= 0.0:
        return None
    return StereoEstimate(
        range_m=range_m,
        time_delta_s=match.time_delta_s,
        vertical_delta_px=match.vertical_delta_px,
        left=match.left,
        right=match.right,
        left_extrinsic=match.left_extrinsic,
        right_extrinsic=match.right_extrinsic,
        extrinsic_time_delta_s=match.extrinsic_time_delta_s,
        left_attitude_age_s=match.left_attitude_age_s,
        right_attitude_age_s=match.right_attitude_age_s,
        attitude_pair_delta_s=match.attitude_pair_delta_s,
        relative_roll_deg=match.relative_roll_deg,
        relative_pitch_deg=match.relative_pitch_deg,
        relative_yaw_deg=match.relative_yaw_deg,
        sync_quality=match.sync_quality,
        warnings=match.warnings,
    )
