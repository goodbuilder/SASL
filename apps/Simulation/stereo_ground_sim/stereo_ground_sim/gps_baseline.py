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


def _bearing_deg(vector_enu: tuple[float, float, float]) -> float:
    """Compass bearing (clockwise from North) of an ENU vector's horizontal part.

    World-ENU vectors in this codebase are ordered (East, North, Up), so
    bearing is ``atan2(East, North)``.
    """

    east, north, _up = vector_enu
    return math.degrees(math.atan2(float(east), float(north)))


def _wrap_180_deg(angle_deg: float) -> float:
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


def declination_from_gps_baseline_rad(
    left_fix: RawGpsFix,
    right_fix: RawGpsFix,
    nominal_baseline_enu_m: tuple[float, float, float],
    nominal_left_camera_pose,
    attitude_left_camera_pose,
) -> float | None:
    """Estimate local magnetic declination from the dual-GPS camera baseline.

    Compares the left/right GPS baseline's true-north bearing (pure WGS84
    geodesy, magnetism-free) against the same baseline direction as predicted
    by the live, uncorrected magnetometer-derived left-camera attitude. The
    angular difference is ``true - magnetic``, matching the convention
    ``true_heading = magnetic_heading + declination`` (the same convention
    PX4's own EKF2 declination fusion uses).

    ``nominal_left_camera_pose`` and ``attitude_left_camera_pose`` must be
    DISTINCT poses: the nominal/design pose (fixed, config-derived, no live
    sensors) is used only to express the known baseline direction in the
    camera's own local frame; the live magnetometer-derived attitude pose is
    then used to rotate that local vector back into world ENU. Passing the
    same pose for both makes the projection-and-back-rotation a mathematical
    no-op that cancels any injected bias -- always returns ~0 regardless of
    the true heading error.

    Returns ``None`` if any input is non-finite or the baseline is
    degenerate, rather than raising -- callers should fail the ray closed.
    """

    try:
        gps_measured_enu = gps_baseline_enu_m(left_fix, right_fix)
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in gps_measured_enu):
        return None
    if norm_m(gps_measured_enu) <= 1.0e-12:
        return None
    if not all(math.isfinite(float(value)) for value in nominal_baseline_enu_m):
        return None
    local_baseline_design = (
        float(_dot3(nominal_left_camera_pose.right_world_enu, nominal_baseline_enu_m)),
        float(_dot3(nominal_left_camera_pose.down_world_enu, nominal_baseline_enu_m)),
        float(_dot3(nominal_left_camera_pose.forward_world_enu, nominal_baseline_enu_m)),
    )
    if norm_m(local_baseline_design) <= 1.0e-12:
        return None
    magnetometer_predicted_enu = attitude_left_camera_pose.ray_to_world(local_baseline_design)
    if not all(math.isfinite(value) for value in magnetometer_predicted_enu):
        return None
    declination_deg = _wrap_180_deg(
        _bearing_deg(gps_measured_enu) - _bearing_deg(magnetometer_predicted_enu)
    )
    return math.radians(declination_deg)


def _dot3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum(float(a[index]) * float(b[index]) for index in range(3))
