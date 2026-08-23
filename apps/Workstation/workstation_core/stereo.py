from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from dataclasses import field
from math import isfinite
from typing import Mapping, Any


def _materialize_timestamp_metadata(metadata: Any) -> dict[str, Any]:
    """Detach payload metadata after validating its explicit pair shape."""

    if metadata is None:
        return {}
    if isinstance(metadata, Mapping):
        raw_items = tuple(metadata.items())
    elif type(metadata) in (tuple, list):
        raw_items = metadata
    else:
        raise TypeError("timestamp metadata must be a Mapping, tuple, or list")

    materialized: dict[str, Any] = {}
    for raw_item in raw_items:
        if type(raw_item) not in (tuple, list):
            raise TypeError("each timestamp metadata item must be a tuple or list")
        if len(raw_item) != 2:
            raise ValueError("each timestamp metadata item must contain exactly two values")
        key, value = raw_item
        if type(key) is not str:
            raise TypeError("timestamp metadata keys must be strings")
        if key in materialized:
            raise ValueError(f"duplicate timestamp metadata key: {key}")
        materialized[key] = value
    return deepcopy(materialized)


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
    candidate_class_ids: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "candidate_class_ids",
            tuple(int(class_id) for class_id in self.candidate_class_ids),
        )

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def same_class_candidate_count(self) -> int:
        candidates = self.candidate_class_ids or (self.class_id,)
        return sum(class_id == self.class_id for class_id in candidates)

    @classmethod
    def from_payload(cls, camera: str, payload: Mapping[str, object]) -> "DetectionObservation | None":
        best = payload.get("best_detection")
        if not isinstance(best, Mapping):
            return None
        required_payload = ("timestamp_s", "width", "height")
        required_detection = ("x1", "y1", "x2", "y2", "confidence", "class_id")
        if any(payload.get(name) is None for name in required_payload) or any(
            best.get(name) is None for name in required_detection
        ):
            return None
        candidates = payload.get("detections")
        try:
            candidate_class_ids = tuple(
                int(candidate.get("class_id"))
                for candidate in candidates
                if isinstance(candidate, Mapping)
                and candidate.get("class_id") is not None
            ) if isinstance(candidates, (list, tuple)) else ()
            timestamp_s = float(payload.get("timestamp_s"))
            width = int(payload.get("width"))
            height = int(payload.get("height"))
            x1 = float(best.get("x1"))
            y1 = float(best.get("y1"))
            x2 = float(best.get("x2"))
            y2 = float(best.get("y2"))
            confidence = float(best.get("confidence"))
            class_id = int(best.get("class_id"))
            timestamp_metadata = _materialize_timestamp_metadata(
                payload.get("timestamp_metadata")
            )
        except (TypeError, ValueError, OverflowError):
            return None
        numeric_values = (timestamp_s, x1, y1, x2, y2, confidence)
        if (
            width <= 0
            or height <= 0
            or not all(isfinite(value) for value in numeric_values)
            or x2 < x1
            or y2 < y1
        ):
            return None
        return cls(
            camera=str(camera),
            timestamp_s=timestamp_s,
            width=width,
            height=height,
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
            confidence=confidence,
            class_id=class_id,
            class_name=(
                str(best.get("class_name"))
                if best.get("class_name") is not None
                else None
            ),
            timestamp_metadata=timestamp_metadata,
            candidate_class_ids=candidate_class_ids,
        )


def correspondence_rejection_reason(
    left: DetectionObservation | None,
    right: DetectionObservation | None,
) -> str | None:
    """Return a deterministic fail-closed reason for the minimal B1 matcher."""

    if left is None:
        return "missing_left_detection"
    if right is None:
        return "missing_right_detection"
    if left.same_class_candidate_count > 1:
        return "ambiguous_left_candidates"
    if right.same_class_candidate_count > 1:
        return "ambiguous_right_candidates"
    if left.same_class_candidate_count != 1 or right.same_class_candidate_count != 1:
        return "ambiguous_correspondence"
    if left.class_id != right.class_id:
        return "correspondence_class_mismatch"
    return None


@dataclass(frozen=True)
class StereoCalibration:
    focal_length_px: float
    baseline_m: float
    max_time_delta_s: float = 0.25
    max_vertical_delta_px: float = 80.0
    require_same_class: bool = True
    timestamp_basis: str = "detection"
    quality_gates_advisory: bool = False


@dataclass(frozen=True)
class StereoMatch:
    disparity_px: float
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
    distance_m: float = 0.0


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
    if correspondence_rejection_reason(left, right) is not None:
        return None
    left_center_x, left_center_y = left.center
    right_center_x, right_center_y = right.center
    disparity = abs(left_center_x - right_center_x)
    if not isfinite(disparity) or disparity <= 1e-6:
        return None
    left_timestamp_s = detection_timestamp_for_basis(left, calibration.timestamp_basis)
    right_timestamp_s = detection_timestamp_for_basis(right, calibration.timestamp_basis)
    advisory_warnings = list(warnings)
    if left_timestamp_s is None or right_timestamp_s is None:
        if not calibration.quality_gates_advisory:
            return None
        left_timestamp_s = float(left.timestamp_s)
        right_timestamp_s = float(right.timestamp_s)
        advisory_warnings.append("RTCP/NTP timestamp unavailable; app receive time used")
    time_delta = abs(left_timestamp_s - right_timestamp_s)
    if time_delta > calibration.max_time_delta_s:
        if not calibration.quality_gates_advisory:
            return None
        advisory_warnings.append(
            f"camera pair delta {time_delta:.6f}s exceeds {calibration.max_time_delta_s:.6f}s"
        )
    vertical_delta = abs(left_center_y - right_center_y)
    if vertical_delta > calibration.max_vertical_delta_px:
        if not calibration.quality_gates_advisory:
            return None
        advisory_warnings.append(
            f"vertical bbox delta {vertical_delta:.3f}px exceeds {calibration.max_vertical_delta_px:.3f}px"
        )
    if calibration.require_same_class and left.class_id != right.class_id:
        return None
    extrinsic_time_delta_s = None
    if left_extrinsic is not None and right_extrinsic is not None:
        left_ts = getattr(left_extrinsic, "timestamp_s", None)
        right_ts = getattr(right_extrinsic, "timestamp_s", None)
        if left_ts is not None and right_ts is not None:
            extrinsic_time_delta_s = abs(float(left_ts) - float(right_ts))
    return StereoMatch(
        disparity_px=disparity,
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
        warnings=tuple(dict.fromkeys(advisory_warnings)),
    )


def estimate_disparity_distance(match: StereoMatch, calibration: StereoCalibration) -> StereoEstimate | None:
    if calibration.focal_length_px <= 0.0 or calibration.baseline_m <= 0.0:
        return None
    distance = calibration.focal_length_px * calibration.baseline_m / match.disparity_px
    if not isfinite(distance) or distance <= 0.0:
        return None
    return stereo_estimate_from_match(match, distance_m=distance)


def stereo_estimate_from_match(match: StereoMatch, *, distance_m: float) -> StereoEstimate | None:
    if not isfinite(distance_m) or distance_m <= 0.0:
        return None
    return StereoEstimate(
        distance_m=distance_m,
        disparity_px=match.disparity_px,
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


def estimate_distance(
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
) -> StereoEstimate | None:
    match = match_stereo_observations(
        left,
        right,
        calibration,
        left_extrinsic=left_extrinsic,
        right_extrinsic=right_extrinsic,
        left_attitude_age_s=left_attitude_age_s,
        right_attitude_age_s=right_attitude_age_s,
        attitude_pair_delta_s=attitude_pair_delta_s,
        relative_roll_deg=relative_roll_deg,
        relative_pitch_deg=relative_pitch_deg,
        relative_yaw_deg=relative_yaw_deg,
        sync_quality=sync_quality,
        warnings=warnings,
    )
    if match is None:
        return None
    return estimate_disparity_distance(match, calibration)
