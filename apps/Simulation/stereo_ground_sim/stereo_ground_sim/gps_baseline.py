"""Raw-GPS camera-baseline geometry for Simulation-only comparison runs."""

from __future__ import annotations

import math
from dataclasses import dataclass


WGS84_A_M = 6_378_137.0
WGS84_E2 = 6.6943799901413165e-3


@dataclass(frozen=True)
class RawGpsFix:
    timestamp_us: int
    latitude_deg: float
    longitude_deg: float
    altitude_msl_m: float
    eph_m: float
    epv_m: float


def _finite(*values: float) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("GPS coordinates must be finite")


def _ecef_from_wgs84(latitude_deg: float, longitude_deg: float, altitude_m: float) -> tuple[float, float, float]:
    _finite(latitude_deg, longitude_deg, altitude_m)
    latitude = math.radians(float(latitude_deg))
    longitude = math.radians(float(longitude_deg))
    sin_latitude = math.sin(latitude)
    radius = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_latitude * sin_latitude)
    return (
        (radius + altitude_m) * math.cos(latitude) * math.cos(longitude),
        (radius + altitude_m) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
    )


def gps_relative_enu_m(origin: RawGpsFix, target: RawGpsFix) -> tuple[float, float, float]:
    """Return ``target - origin`` in the origin's local ENU frame, in metres."""

    origin_ecef = _ecef_from_wgs84(
        origin.latitude_deg, origin.longitude_deg, origin.altitude_msl_m
    )
    target_ecef = _ecef_from_wgs84(
        target.latitude_deg, target.longitude_deg, target.altitude_msl_m
    )
    dx, dy, dz = tuple(target_ecef[index] - origin_ecef[index] for index in range(3))
    latitude = math.radians(origin.latitude_deg)
    longitude = math.radians(origin.longitude_deg)
    east = -math.sin(longitude) * dx + math.cos(longitude) * dy
    north = (
        -math.sin(latitude) * math.cos(longitude) * dx
        - math.sin(latitude) * math.sin(longitude) * dy
        + math.cos(latitude) * dz
    )
    up = (
        math.cos(latitude) * math.cos(longitude) * dx
        + math.cos(latitude) * math.sin(longitude) * dy
        + math.sin(latitude) * dz
    )
    return float(east), float(north), float(up)


def gps_baseline_enu_m(left: RawGpsFix, right: RawGpsFix) -> tuple[float, float, float]:
    """Return the right-minus-left raw-GPS baseline in the left local ENU frame."""

    return gps_relative_enu_m(left, right)


def norm_m(vector: tuple[float, float, float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in vector))
