"""Ideal rectified-stereo disparity derived from the live GNSS baseline.

The result is a planning/reference value, not an observed image disparity.
It assumes a rectified stereo pair and parallel optical axes.  The live
Left/Right/Drone Pixhawk positions define the P1/P2 baseline and the Drone
range from its midpoint; camera pose, lens distortion, and target bearing are
not inferred from GNSS.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from math import cos, radians, sqrt

from Workstation.workstation_core.baseline_alignment import BaselineVector, baseline_from_gps
from Workstation.workstation_core.rtk_geometry import EARTH_RADIUS_M, GpsPositionState


@dataclass(frozen=True)
class IdealDisparityEstimate:
    """A source-labelled ideal disparity estimate in pixels."""

    state: str
    reason: str
    baseline: BaselineVector | None = None
    focal_length_px: float | None = None
    target_distance_m: float | None = None
    disparity_px: float | None = None

    @property
    def baseline_m(self) -> float | None:
        return None if self.baseline is None else self.baseline.length_m


def ideal_disparity_px(*, baseline_m: float, focal_length_px: float, target_distance_m: float) -> float:
    """Return ``f * B / Z`` for a rectified, parallel stereo pair."""

    baseline = float(baseline_m)
    focal = float(focal_length_px)
    distance = float(target_distance_m)
    if not all(math.isfinite(value) for value in (baseline, focal, distance)):
        raise ValueError("baseline, focal length, and target distance must be finite")
    if baseline < 0.0:
        raise ValueError("baseline must be non-negative")
    if focal <= 0.0:
        raise ValueError("focal length must be positive")
    if distance <= 0.0:
        raise ValueError("target distance must be positive")
    return focal * baseline / distance


def target_distance_from_gps(
    left: GpsPositionState,
    right: GpsPositionState,
    drone: GpsPositionState,
) -> float | None:
    """Return the Drone range from the P1/P2 GNSS baseline midpoint.

    The three positions are treated as a local ENU triangle.  This keeps the
    disparity reference explicit: ``B`` is the P1/P2 GNSS baseline and ``Z``
    is the Euclidean range from its midpoint to the Drone GNSS position.
    """

    if not all(_valid_gps_position(state) for state in (left, right, drone)):
        return None
    baseline = baseline_from_gps(left, right)
    if baseline is None:
        return None
    reference_lat_rad = radians((float(left.latitude_deg) + float(right.latitude_deg)) / 2.0)
    drone_north_m = radians(float(drone.latitude_deg) - float(left.latitude_deg)) * EARTH_RADIUS_M
    drone_east_m = (
        radians(float(drone.longitude_deg) - float(left.longitude_deg))
        * EARTH_RADIUS_M
        * cos(reference_lat_rad)
    )
    drone_up_m = float(drone.altitude_m) - float(left.altitude_m)
    midpoint_relative = (
        drone_east_m - baseline.east_m / 2.0,
        drone_north_m - baseline.north_m / 2.0,
        drone_up_m - baseline.up_m / 2.0,
    )
    distance = sqrt(sum(value * value for value in midpoint_relative))
    return distance if math.isfinite(distance) and distance > 0.0 else None


def estimate_ideal_disparity(
    left: GpsPositionState | None,
    right: GpsPositionState | None,
    drone: GpsPositionState | None,
    *,
    focal_length_px: float | None,
    now_s: float | None = None,
    maximum_gps_age_s: float | None = None,
) -> IdealDisparityEstimate:
    """Build a UI-friendly estimate while preserving missing-data reasons."""

    if left is None or right is None or drone is None:
        return IdealDisparityEstimate("WAITING", "P1/P2/Drone GNSS 위치 대기 중")
    baseline = baseline_from_gps(left, right)
    if baseline is None:
        return IdealDisparityEstimate("WAITING", "P1/P2 GNSS 위치가 유효하지 않음")
    target_distance = target_distance_from_gps(left, right, drone)
    if target_distance is None:
        return IdealDisparityEstimate(
            "WAITING",
            "세 GNSS 위치가 유효하지 않음",
            baseline=baseline,
        )

    if maximum_gps_age_s is not None and now_s is not None:
        age_limit = max(0.0, float(maximum_gps_age_s))
        oldest_age = max(
            float(now_s) - float(left.timestamp_s),
            float(now_s) - float(right.timestamp_s),
            float(now_s) - float(drone.timestamp_s),
        )
        if oldest_age > age_limit:
            return IdealDisparityEstimate(
                "STALE",
                f"세 GNSS age가 {age_limit:.1f}s를 초과함",
                baseline=baseline,
                focal_length_px=_finite_or_none(focal_length_px),
                target_distance_m=target_distance,
            )

    focal = _finite_or_none(focal_length_px)
    if focal is None or focal <= 0.0:
        return IdealDisparityEstimate(
            "WAITING",
            "유효한 줌별 카메라 초점거리(fx)가 없음",
            baseline=baseline,
            focal_length_px=focal,
            target_distance_m=target_distance,
        )
    disparity = ideal_disparity_px(
        baseline_m=baseline.length_m,
        focal_length_px=focal,
        target_distance_m=target_distance,
    )
    return IdealDisparityEstimate(
        "READY",
        "three GNSS positions + active zoom focal length; rectified parallel-camera model",
        baseline=baseline,
        focal_length_px=focal,
        target_distance_m=target_distance,
        disparity_px=disparity,
    )


def _finite_or_none(value: float | None) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _valid_gps_position(gps: GpsPositionState) -> bool:
    if not gps.has_position:
        return False
    try:
        latitude = float(gps.latitude_deg)
        longitude = float(gps.longitude_deg)
        altitude = float(gps.altitude_m)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(latitude)
        and -90.0 <= latitude <= 90.0
        and math.isfinite(longitude)
        and -180.0 <= longitude <= 180.0
        and math.isfinite(altitude)
    )
