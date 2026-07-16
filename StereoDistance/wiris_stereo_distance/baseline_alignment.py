from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from math import acos, atan2, cos, degrees, radians, sqrt
from statistics import mean, pstdev
from typing import Any

import yaml

from wiris_stereo_distance.rtk_geometry import EARTH_RADIUS_M, GpsPositionState, ned_delta_m


@dataclass(frozen=True)
class BaselineVector:
    east_m: float
    north_m: float
    up_m: float
    length_m: float
    azimuth_deg: float
    elevation_deg: float
    altitude_delta_m: float
    timestamp_delta_ms: float
    left_timestamp_s: float
    right_timestamp_s: float

    @property
    def enu_m(self) -> tuple[float, float, float]:
        return self.east_m, self.north_m, self.up_m

    @property
    def ned_m(self) -> tuple[float, float, float]:
        return self.north_m, self.east_m, -self.up_m


@dataclass(frozen=True)
class BaselineQuality:
    grade: str
    baseline_length_mean_m: float
    baseline_length_std_m: float
    enu_mean_m: tuple[float, float, float]
    enu_std_m: tuple[float, float, float]
    rtk_fixed_ratio: float
    gps_message_rate_hz: float
    timestamp_delta_ms: float
    outlier_count: int
    baseline_jump_count: int
    mean_h_acc_m: float | None
    mean_v_acc_m: float | None
    sample_count: int
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class BaselineQualityThresholds:
    bad_std_m: float = 0.10
    jump_m: float = 0.20
    max_timestamp_delta_ms: float = 200.0
    good_rtk_ratio: float = 0.95
    good_std_m: float = 0.02
    good_h_acc_m: float = 0.03
    good_v_acc_m: float = 0.05
    good_timestamp_delta_ms: float = 50.0
    warning_rtk_ratio: float = 0.70


@dataclass(frozen=True)
class BaselineSourceComparison:
    sensor_baseline: BaselineVector | None
    global_baseline: BaselineVector | None
    left_position_delta_ned_m: tuple[float, float, float] | None
    right_position_delta_ned_m: tuple[float, float, float] | None
    baseline_delta_ned_m: tuple[float, float, float] | None
    baseline_length_delta_m: float | None
    baseline_angle_deg: float | None
    status: str


def _vector_angle_deg(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> float | None:
    first_length = sqrt(sum(value * value for value in first))
    second_length = sqrt(sum(value * value for value in second))
    if first_length <= 1e-9 or second_length <= 1e-9:
        return None
    cosine = sum(a * b for a, b in zip(first, second)) / (first_length * second_length)
    return degrees(acos(max(-1.0, min(1.0, cosine))))


def compare_baseline_sources(
    sensor_left: GpsPositionState | None,
    sensor_right: GpsPositionState | None,
    global_left: GpsPositionState | None,
    global_right: GpsPositionState | None,
    *,
    max_timestamp_delta_ms: float = 200.0,
) -> BaselineSourceComparison:
    sensor_baseline = (
        baseline_from_gps(sensor_left, sensor_right)
        if sensor_left is not None and sensor_right is not None
        else None
    )
    global_baseline = (
        baseline_from_gps(global_left, global_right)
        if global_left is not None and global_right is not None
        else None
    )
    left_delta = (
        ned_delta_m(sensor_left, global_left)
        if sensor_left is not None and global_left is not None
        else None
    )
    right_delta = (
        ned_delta_m(sensor_right, global_right)
        if sensor_right is not None and global_right is not None
        else None
    )
    baseline_delta = None
    baseline_length_delta = None
    baseline_angle = None
    if sensor_baseline is not None and global_baseline is not None:
        baseline_delta = (
            global_baseline.north_m - sensor_baseline.north_m,
            global_baseline.east_m - sensor_baseline.east_m,
            -global_baseline.up_m + sensor_baseline.up_m,
        )
        baseline_length_delta = global_baseline.length_m - sensor_baseline.length_m
        baseline_angle = _vector_angle_deg(sensor_baseline.ned_m, global_baseline.ned_m)

    if sensor_baseline is None or global_baseline is None:
        status = "WAITING_SOURCE"
    elif (
        sensor_baseline.timestamp_delta_ms > max_timestamp_delta_ms
        or global_baseline.timestamp_delta_ms > max_timestamp_delta_ms
    ):
        status = "TIMESTAMP_SKEW"
    else:
        status = "OK"
    return BaselineSourceComparison(
        sensor_baseline=sensor_baseline,
        global_baseline=global_baseline,
        left_position_delta_ned_m=left_delta,
        right_position_delta_ned_m=right_delta,
        baseline_delta_ned_m=baseline_delta,
        baseline_length_delta_m=baseline_length_delta,
        baseline_angle_deg=baseline_angle,
        status=status,
    )


def baseline_from_gps(left: GpsPositionState, right: GpsPositionState) -> BaselineVector | None:
    if not left.has_position or not right.has_position:
        return None
    lat0 = radians(float(left.latitude_deg))
    d_lat = radians(float(right.latitude_deg) - float(left.latitude_deg))
    d_lon = radians(float(right.longitude_deg) - float(left.longitude_deg))
    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * cos(lat0)
    up = float(right.altitude_m) - float(left.altitude_m)
    horizontal_m = sqrt(east * east + north * north)
    length_m = sqrt(horizontal_m * horizontal_m + up * up)
    azimuth_deg = (degrees(atan2(east, north)) + 360.0) % 360.0
    elevation_deg = degrees(atan2(up, horizontal_m)) if horizontal_m > 1e-9 else 0.0
    return BaselineVector(
        east_m=east,
        north_m=north,
        up_m=up,
        length_m=length_m,
        azimuth_deg=azimuth_deg,
        elevation_deg=elevation_deg,
        altitude_delta_m=up,
        timestamp_delta_ms=abs(float(left.timestamp_s) - float(right.timestamp_s)) * 1000.0,
        left_timestamp_s=float(left.timestamp_s),
        right_timestamp_s=float(right.timestamp_s),
    )


def evaluate_baseline_quality(
    samples: list[tuple[BaselineVector, GpsPositionState, GpsPositionState]],
    *,
    duration_s: float,
    thresholds: BaselineQualityThresholds | None = None,
) -> BaselineQuality | None:
    if not samples:
        return None
    limits = thresholds or BaselineQualityThresholds()
    vectors = [sample[0] for sample in samples]
    lengths = [vector.length_m for vector in vectors]
    east_values = [vector.east_m for vector in vectors]
    north_values = [vector.north_m for vector in vectors]
    up_values = [vector.up_m for vector in vectors]
    timestamp_deltas = [vector.timestamp_delta_ms for vector in vectors]
    rtk_fixed = [
        left.fix_type == 6 and right.fix_type == 6
        for _vector, left, right in samples
    ]
    h_acc_values = [
        value
        for _vector, left, right in samples
        for value in (left.eph_m, right.eph_m)
        if value is not None
    ]
    v_acc_values = [
        value
        for _vector, left, right in samples
        for value in (left.epv_m, right.epv_m)
        if value is not None
    ]
    length_mean = mean(lengths)
    length_std = pstdev(lengths) if len(lengths) > 1 else 0.0
    jumps = sum(1 for previous, current in zip(lengths, lengths[1:]) if abs(current - previous) > limits.jump_m)
    outliers = sum(1 for value in lengths if abs(value - length_mean) > max(limits.bad_std_m, length_std * 3.0))
    rtk_ratio = sum(1 for item in rtk_fixed if item) / len(rtk_fixed)
    mean_h_acc = mean(h_acc_values) if h_acc_values else None
    mean_v_acc = mean(v_acc_values) if v_acc_values else None
    max_timestamp_delta = max(timestamp_deltas)
    reasons: list[str] = []

    any_low_fix = any((left.fix_type or 0) < 5 or (right.fix_type or 0) < 5 for _vector, left, right in samples)
    if any_low_fix:
        reasons.append("No Fix/3D Fix present")
    if length_std > limits.bad_std_m:
        reasons.append(f"baseline std > {limits.bad_std_m:.2f} m")
    if max_timestamp_delta > limits.max_timestamp_delta_ms:
        reasons.append(f"timestamp delta > {limits.max_timestamp_delta_ms:.0f} ms")
    if jumps > 0:
        reasons.append(f"baseline jump > {limits.jump_m:.2f} m")

    if reasons:
        grade = "BAD"
    elif (
        rtk_ratio >= limits.good_rtk_ratio
        and length_std < limits.good_std_m
        and mean_h_acc is not None
        and mean_h_acc < limits.good_h_acc_m
        and mean_v_acc is not None
        and mean_v_acc < limits.good_v_acc_m
        and max_timestamp_delta < limits.good_timestamp_delta_ms
    ):
        grade = "GOOD"
    elif rtk_ratio >= limits.warning_rtk_ratio or length_std <= limits.bad_std_m or max_timestamp_delta <= limits.max_timestamp_delta_ms:
        grade = "WARNING"
    else:
        grade = "BAD"

    return BaselineQuality(
        grade=grade,
        baseline_length_mean_m=length_mean,
        baseline_length_std_m=length_std,
        enu_mean_m=(mean(east_values), mean(north_values), mean(up_values)),
        enu_std_m=(
            pstdev(east_values) if len(east_values) > 1 else 0.0,
            pstdev(north_values) if len(north_values) > 1 else 0.0,
            pstdev(up_values) if len(up_values) > 1 else 0.0,
        ),
        rtk_fixed_ratio=rtk_ratio,
        gps_message_rate_hz=len(samples) / max(float(duration_s), 1e-6),
        timestamp_delta_ms=max_timestamp_delta,
        outlier_count=outliers,
        baseline_jump_count=jumps,
        mean_h_acc_m=mean_h_acc,
        mean_v_acc_m=mean_v_acc,
        sample_count=len(samples),
        reasons=tuple(reasons),
    )


def baseline_profile_payload(
    *,
    left_gps1_topic: str,
    left_gps2_topic: str,
    right_gps1_topic: str,
    right_gps2_topic: str,
    baseline: BaselineVector,
    quality: BaselineQuality | None,
) -> dict[str, Any]:
    quality_payload = {
        "rtk_fixed_ratio": 0.0,
        "baseline_std_m": 0.0,
        "mean_h_acc_m": 0.0,
        "mean_v_acc_m": 0.0,
        "timestamp_delta_ms": baseline.timestamp_delta_ms,
        "grade": "UNKNOWN",
    }
    if quality is not None:
        quality_payload = {
            "rtk_fixed_ratio": float(quality.rtk_fixed_ratio),
            "baseline_std_m": float(quality.baseline_length_std_m),
            "mean_h_acc_m": 0.0 if quality.mean_h_acc_m is None else float(quality.mean_h_acc_m),
            "mean_v_acc_m": 0.0 if quality.mean_v_acc_m is None else float(quality.mean_v_acc_m),
            "timestamp_delta_ms": float(quality.timestamp_delta_ms),
            "grade": quality.grade,
        }
    return {
        "baseline_profile": {
            "created_at": datetime.now(timezone.utc).isoformat(),
            "origin": "left_camera_gps1",
            "left_camera": {
                "gps1_topic": left_gps1_topic,
                "gps2_topic": left_gps2_topic,
            },
            "right_camera": {
                "gps1_topic": right_gps1_topic,
                "gps2_topic": right_gps2_topic,
            },
            "baseline": {
                "frame": "ENU",
                "vector_m": [float(baseline.east_m), float(baseline.north_m), float(baseline.up_m)],
                "length_m": float(baseline.length_m),
                "azimuth_deg": float(baseline.azimuth_deg),
                "elevation_deg": float(baseline.elevation_deg),
            },
            "quality": quality_payload,
        }
    }


def dump_baseline_profile(payload: dict[str, Any]) -> str:
    return yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)


def load_baseline_profile(text: str) -> dict[str, Any]:
    payload = yaml.safe_load(text) or {}
    if not isinstance(payload, dict) or "baseline_profile" not in payload:
        raise ValueError("baseline_profile YAML 형식이 아닙니다.")
    return payload
