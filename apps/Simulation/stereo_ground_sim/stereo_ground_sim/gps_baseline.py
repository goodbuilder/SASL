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


def _wrap_pi(angle_rad: float) -> float:
    """Wrap an angle in radians to (-pi, pi]."""

    return ((float(angle_rad) + math.pi) % (2.0 * math.pi)) - math.pi


def gps_baseline_bearing_rad(left_fix: RawGpsFix, right_fix: RawGpsFix) -> float | None:
    """True-north horizontal bearing of the left->right GNSS baseline.

    Pure WGS84 geodesy (``gps_baseline_enu_m``), magnetism/attitude-free.
    World-ENU vectors in this codebase are ordered (East, North, Up), so
    bearing is ``atan2(East, North)``. Returns ``None`` if the fixes are
    invalid or the horizontal baseline is degenerate (near-vertical/zero),
    rather than raising -- callers should fail closed.
    """

    try:
        baseline_enu = gps_baseline_enu_m(left_fix, right_fix)
    except ValueError:
        return None
    east, north, _up = baseline_enu
    horizontal_norm = math.hypot(east, north)
    if not math.isfinite(horizontal_norm) or horizontal_norm <= 1.0e-6:
        return None
    return math.atan2(east, north)


def true_north_yaw_correction_rad(
    gps_baseline_bearing_rad: float,
    baseline_to_forward_yaw_offset_rad: float,
    camera_forward_world_enu: tuple[float, float, float],
) -> float | None:
    """Per-camera yaw correction from a GNSS-baseline-derived True-North heading.

    ``expected_heading = gps_baseline_bearing_rad + baseline_to_forward_yaw_offset_rad``
    is the camera's Forward bearing implied by the (magnetism-free) GNSS
    baseline plus the known rig calibration offset between the baseline and
    that camera's own boresight. ``camera_forward_world_enu`` is the SAME
    camera's live, uncorrected PX4-attitude-derived Forward vector (from the
    full body-to-world rotation -- never Euler-decomposed); its own horizontal
    bearing is the "live" heading. The correction is ``expected - live``,
    wrapped to (-pi, pi], to be applied as a world-frame (NED Z) yaw
    pre-multiply -- see ``coordinate_frames.camera_optical_pose_from_px4_attitude_wxyz``'s
    ``declination_correction_rad`` kwarg.

    Returns ``None`` if any input is non-finite or the camera's forward
    vector is near-vertical (horizontal heading undefined), rather than
    raising -- callers should fail the ray closed.
    """

    if not math.isfinite(float(gps_baseline_bearing_rad)) or not math.isfinite(
        float(baseline_to_forward_yaw_offset_rad)
    ):
        return None
    expected_heading_rad = float(gps_baseline_bearing_rad) + float(baseline_to_forward_yaw_offset_rad)
    east, north, _up = camera_forward_world_enu
    horizontal_norm = math.hypot(east, north)
    if not math.isfinite(horizontal_norm) or horizontal_norm <= 1.0e-6:
        return None
    live_heading_rad = math.atan2(east, north)
    return _wrap_pi(expected_heading_rad - live_heading_rad)
