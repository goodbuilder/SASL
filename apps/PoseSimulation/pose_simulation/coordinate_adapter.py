"""WGS84/ECEF/local-ENU conversion for PoseSimulation.

The conversion is intentionally independent from the existing Simulation
package.  Latitude and longitude are never differenced directly.  A GPS
sample is converted to WGS84 ECEF, differenced against the configured
reference origin, rotated to local ENU, and then translated by the configured
Gazebo-world origin offset.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .pose_math import Vector3, as_vector3


WGS84_SEMI_MAJOR_AXIS_M = 6_378_137.0
WGS84_FIRST_ECCENTRICITY_SQUARED = 6.6943799901413165e-3


@dataclass(frozen=True)
class Wgs84ReferenceOrigin:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    altitude_datum: str = "ellipsoid"

    def validate(self) -> "Wgs84ReferenceOrigin":
        latitude = float(self.latitude_deg)
        longitude = float(self.longitude_deg)
        altitude = float(self.altitude_m)
        if not all(math.isfinite(value) for value in (latitude, longitude, altitude)):
            raise ValueError("WGS84 reference origin must be finite")
        if latitude < -90.0 or latitude > 90.0:
            raise ValueError("reference latitude must be in [-90, 90] degrees")
        if longitude < -180.0 or longitude > 180.0:
            raise ValueError("reference longitude must be in [-180, 180] degrees")
        datum = str(self.altitude_datum).strip().lower()
        if datum not in {"ellipsoid", "msl"}:
            raise ValueError("altitude_datum must be 'ellipsoid' or 'msl'")
        return Wgs84ReferenceOrigin(latitude, longitude, altitude, datum)


@dataclass(frozen=True)
class GazeboOriginOffset:
    """Gazebo world origin expressed in the reference origin's local ENU."""

    east_m: float = 0.0
    north_m: float = 0.0
    up_m: float = 0.0

    def as_vector(self) -> Vector3:
        return as_vector3((self.east_m, self.north_m, self.up_m), "Gazebo origin offset")


def _validated_wgs84(
    latitude_deg: float, longitude_deg: float, altitude_m: float
) -> tuple[float, float, float]:
    latitude = float(latitude_deg)
    longitude = float(longitude_deg)
    altitude = float(altitude_m)
    if not all(math.isfinite(value) for value in (latitude, longitude, altitude)):
        raise ValueError("WGS84 position must be finite")
    if latitude < -90.0 or latitude > 90.0:
        raise ValueError("latitude must be in [-90, 90] degrees")
    if longitude < -180.0 or longitude > 180.0:
        raise ValueError("longitude must be in [-180, 180] degrees")
    return latitude, longitude, altitude


def ecef_from_wgs84(
    latitude_deg: float, longitude_deg: float, altitude_m: float
) -> Vector3:
    """Convert WGS84 geodetic coordinates to ECEF metres."""

    latitude_deg, longitude_deg, altitude_m = _validated_wgs84(
        latitude_deg, longitude_deg, altitude_m
    )
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    radius = WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )
    return (
        (radius + altitude_m) * cos_latitude * math.cos(longitude),
        (radius + altitude_m) * cos_latitude * math.sin(longitude),
        (radius * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED) + altitude_m)
        * sin_latitude,
    )


def enu_from_ecef_delta(
    delta_ecef_m: Sequence[float],
    reference_latitude_deg: float,
    reference_longitude_deg: float,
) -> Vector3:
    """Rotate an ECEF displacement into the reference origin's local ENU."""

    delta = as_vector3(delta_ecef_m, "delta_ecef_m")
    latitude, longitude, _ = _validated_wgs84(
        reference_latitude_deg, reference_longitude_deg, 0.0
    )
    latitude_rad = math.radians(latitude)
    longitude_rad = math.radians(longitude)
    dx, dy, dz = delta
    east = -math.sin(longitude_rad) * dx + math.cos(longitude_rad) * dy
    north = (
        -math.sin(latitude_rad) * math.cos(longitude_rad) * dx
        - math.sin(latitude_rad) * math.sin(longitude_rad) * dy
        + math.cos(latitude_rad) * dz
    )
    up = (
        math.cos(latitude_rad) * math.cos(longitude_rad) * dx
        + math.cos(latitude_rad) * math.sin(longitude_rad) * dy
        + math.sin(latitude_rad) * dz
    )
    return east, north, up


def wgs84_to_reference_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    reference_origin: Wgs84ReferenceOrigin,
) -> Vector3:
    """Convert one WGS84 position to ENU metres relative to ``reference_origin``."""

    origin = reference_origin.validate()
    sample_ecef = ecef_from_wgs84(latitude_deg, longitude_deg, altitude_m)
    origin_ecef = ecef_from_wgs84(
        origin.latitude_deg, origin.longitude_deg, origin.altitude_m
    )
    delta = tuple(sample_ecef[index] - origin_ecef[index] for index in range(3))
    return enu_from_ecef_delta(delta, origin.latitude_deg, origin.longitude_deg)


def wgs84_to_gazebo_world_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    reference_origin: Wgs84ReferenceOrigin,
    gazebo_origin_in_reference_enu_m: Sequence[float] = (0.0, 0.0, 0.0),
) -> Vector3:
    """Convert WGS84 to the configured ``gazebo_world_enu`` frame in metres."""

    local_enu = wgs84_to_reference_enu(
        latitude_deg,
        longitude_deg,
        altitude_m,
        reference_origin,
    )
    offset = as_vector3(
        gazebo_origin_in_reference_enu_m,
        "gazebo_origin_in_reference_enu_m",
    )
    return tuple(local_enu[index] + offset[index] for index in range(3))  # type: ignore[return-value]


__all__ = [
    "GazeboOriginOffset",
    "WGS84_FIRST_ECCENTRICITY_SQUARED",
    "WGS84_SEMI_MAJOR_AXIS_M",
    "Wgs84ReferenceOrigin",
    "ecef_from_wgs84",
    "enu_from_ecef_delta",
    "wgs84_to_gazebo_world_enu",
    "wgs84_to_reference_enu",
]
