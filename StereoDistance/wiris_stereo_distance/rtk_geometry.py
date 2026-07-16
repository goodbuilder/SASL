from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, radians, sin, sqrt
from typing import Any

EARTH_RADIUS_M = 6_378_137.0
GPS_SOURCE_SENSOR_GPS = "sensor_gps"
GPS_SOURCE_GLOBAL_POSITION = "global_position"


@dataclass(frozen=True)
class GpsPositionState:
    camera: str
    namespace: str
    timestamp_s: float
    topic: str
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    altitude_m: float | None = None
    fix_type: int | None = None
    satellites_used: int | None = None
    eph_m: float | None = None
    epv_m: float | None = None
    heading_deg: float | None = None
    heading_accuracy_deg: float | None = None
    rtcm_injection_rate_hz: float | None = None
    px4_timestamp_s: float | None = None
    raw: Any | None = None
    source: str = GPS_SOURCE_SENSOR_GPS
    px4_sample_timestamp_s: float | None = None

    @property
    def has_position(self) -> bool:
        return self.latitude_deg is not None and self.longitude_deg is not None and self.altitude_m is not None

    @property
    def rtk_label(self) -> str:
        # PX4/ublox fix_type commonly uses 5=float RTK, 6=fixed RTK. Keep generic fallback.
        if self.fix_type == 6:
            return "RTK_FIXED"
        if self.fix_type == 5:
            return "RTK_FLOAT"
        if self.fix_type is None:
            return "UNKNOWN"
        if self.fix_type >= 3:
            return "3D_FIX"
        return "NO_FIX"


def gps_topic(namespace: str) -> str:
    normalized = namespace.strip().strip("/")
    return f"/{normalized}/fmu/out/vehicle_gps_position" if normalized else "/fmu/out/vehicle_gps_position"


def global_position_topic(namespace: str) -> str:
    normalized = namespace.strip().strip("/")
    return f"/{normalized}/fmu/out/vehicle_global_position" if normalized else "/fmu/out/vehicle_global_position"


def _scaled_degrees(value: Any) -> float | None:
    if value is None:
        return None
    number = float(value)
    # PX4 vehicle_gps_position lat/lon are usually int32 degrees * 1e7.
    if abs(number) > 1000.0:
        return number * 1e-7
    return number


def _scaled_altitude_m(value: Any) -> float | None:
    if value is None:
        return None
    number = float(value)
    # PX4 altitude is often millimetres. Values above 20 km are almost certainly mm.
    if abs(number) > 20_000.0:
        return number * 1e-3
    return number


def _scaled_accuracy_m(value: Any) -> float | None:
    if value is None:
        return None
    number = float(value)
    # PX4 eph/epv are commonly metres in vehicle_gps_position. Accept mm-like values defensively.
    if abs(number) > 1000.0:
        return number * 1e-3
    return number


def _heading_degrees(value: Any) -> float | None:
    if value is None:
        return None
    number = float(value)
    # Some PX4 fields are radians, some scaled integers depending on msg/version.
    if abs(number) <= 2.0 * 3.141592653589793:
        return number * 180.0 / 3.141592653589793
    if abs(number) > 1000.0:
        return number * 1e-5
    return number


def gps_state_from_message(
    *,
    camera: str,
    namespace: str,
    message: Any,
    timestamp_s: float,
    source: str = GPS_SOURCE_SENSOR_GPS,
    topic: str | None = None,
) -> GpsPositionState:
    px4_timestamp = getattr(message, "timestamp", None)
    px4_sample_timestamp = getattr(message, "timestamp_sample", None)
    lat_lon_valid = getattr(message, "lat_lon_valid", True)
    alt_valid = getattr(message, "alt_valid", True)
    latitude = _scaled_degrees(getattr(message, "lat", getattr(message, "latitude_deg", None)))
    longitude = _scaled_degrees(getattr(message, "lon", getattr(message, "longitude_deg", None)))
    altitude = _scaled_altitude_m(getattr(message, "alt", getattr(message, "altitude_msl_m", None)))
    if lat_lon_valid is False:
        latitude = None
        longitude = None
    if alt_valid is False:
        altitude = None
    return GpsPositionState(
        camera=camera,
        namespace=namespace.strip().strip("/"),
        timestamp_s=float(timestamp_s),
        topic=topic or gps_topic(namespace),
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_m=altitude,
        fix_type=int(getattr(message, "fix_type")) if hasattr(message, "fix_type") else None,
        satellites_used=int(getattr(message, "satellites_used")) if hasattr(message, "satellites_used") else None,
        eph_m=_scaled_accuracy_m(getattr(message, "eph", None)),
        epv_m=_scaled_accuracy_m(getattr(message, "epv", None)),
        heading_deg=_heading_degrees(getattr(message, "heading", getattr(message, "heading_motion", None))),
        heading_accuracy_deg=_heading_degrees(getattr(message, "heading_accuracy", None)),
        rtcm_injection_rate_hz=_scaled_accuracy_m(getattr(message, "rtcm_injection_rate", None)),
        px4_timestamp_s=float(px4_timestamp) * 1e-6 if px4_timestamp is not None else None,
        source=str(source or GPS_SOURCE_SENSOR_GPS),
        px4_sample_timestamp_s=(
            float(px4_sample_timestamp) * 1e-6 if px4_sample_timestamp is not None else None
        ),
        raw=message,
    )


@dataclass(frozen=True)
class RtkBaselineEstimate:
    baseline_m: float
    north_m: float
    east_m: float
    down_m: float
    left_age_s: float
    right_age_s: float
    fix_quality: str
    warnings: tuple[str, ...] = ()


def ned_delta_m(left: GpsPositionState, right: GpsPositionState) -> tuple[float, float, float] | None:
    if not left.has_position or not right.has_position:
        return None
    lat0 = radians((left.latitude_deg + right.latitude_deg) / 2.0)  # type: ignore[operator]
    d_lat = radians(right.latitude_deg - left.latitude_deg)  # type: ignore[operator]
    d_lon = radians(right.longitude_deg - left.longitude_deg)  # type: ignore[operator]
    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * cos(lat0)
    down = -(right.altitude_m - left.altitude_m)  # type: ignore[operator]
    return north, east, down


def estimate_rtk_baseline(
    left: GpsPositionState | None,
    right: GpsPositionState | None,
    *,
    now_s: float,
    require_fixed: bool,
) -> RtkBaselineEstimate | None:
    if left is None or right is None:
        return None
    delta = ned_delta_m(left, right)
    if delta is None:
        return None
    north, east, down = delta
    baseline = sqrt(north * north + east * east + down * down)
    left_age = max(0.0, now_s - left.timestamp_s)
    right_age = max(0.0, now_s - right.timestamp_s)
    warnings: list[str] = []
    if left.fix_type != 6:
        warnings.append(f"left {left.rtk_label}")
    if right.fix_type != 6:
        warnings.append(f"right {right.rtk_label}")
    if require_fixed and any("RTK_FIXED" not in state.rtk_label for state in (left, right)):
        quality = "BAD"
    elif warnings:
        quality = "DEGRADED"
    else:
        quality = "GOOD"
    return RtkBaselineEstimate(
        baseline_m=baseline,
        north_m=north,
        east_m=east,
        down_m=down,
        left_age_s=left_age,
        right_age_s=right_age,
        fix_quality=quality,
        warnings=tuple(warnings),
    )
