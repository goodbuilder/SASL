from __future__ import annotations

import math
from dataclasses import dataclass


LOCAL_NED_FRAME = "px4_local_ned"
LOCAL_NED_AXIS_CONTRACT = "n/north,e/east,d/down"
GLOBAL_LLA_FRAME = "global_lla"
GLOBAL_LLA_AXIS_CONTRACT = "lat_deg,lon_deg,alt_m"


@dataclass(frozen=True)
class LocalNedSample:
    north_m: float
    east_m: float
    down_m: float
    source_frame: str
    source_axis_contract: str
    source: str
    conversion_status: str = "ok"
    raw_lat_deg: float | None = None
    raw_lon_deg: float | None = None
    raw_alt_m: float | None = None
    origin_lat_deg: float | None = None
    origin_lon_deg: float | None = None
    origin_alt_m: float | None = None
    origin_kind: str = ""
    altitude_reference: str = ""


def coerce_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def scaled_degrees(value: object) -> float | None:
    parsed = coerce_float(value)
    if parsed is None:
        return None
    if abs(parsed) > 1000.0:
        return parsed * 1.0e-7
    return parsed


def scaled_altitude_m(value: object) -> float | None:
    parsed = coerce_float(value)
    if parsed is None:
        return None
    if abs(parsed) > 20_000.0:
        return parsed * 1.0e-3
    return parsed


def local_ned_from_xyz(
    x: object,
    y: object,
    z: object,
    *,
    source: str,
    source_frame: str = LOCAL_NED_FRAME,
) -> LocalNedSample | None:
    north_m = coerce_float(x)
    east_m = coerce_float(y)
    down_m = coerce_float(z)
    if None in (north_m, east_m, down_m):
        return None
    return LocalNedSample(
        north_m=float(north_m),
        east_m=float(east_m),
        down_m=float(down_m),
        source_frame=source_frame,
        source_axis_contract=LOCAL_NED_AXIS_CONTRACT,
        source=source,
    )


def gps_lla_delta_to_ned_m(
    *,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    reference_lat_deg: float,
    reference_lon_deg: float,
    reference_alt_m: float,
) -> tuple[float, float, float]:
    earth_radius_m = 6_378_137.0
    lat0_rad = math.radians((lat_deg + reference_lat_deg) * 0.5)
    north_m = math.radians(lat_deg - reference_lat_deg) * earth_radius_m
    east_m = math.radians(lon_deg - reference_lon_deg) * earth_radius_m * math.cos(lat0_rad)
    down_m = -(alt_m - reference_alt_m)
    return north_m, east_m, down_m


def local_ned_from_lla(
    *,
    lat: object,
    lon: object,
    alt: object,
    reference_lat_deg: float,
    reference_lon_deg: float,
    reference_alt_m: float,
    source: str,
    origin_kind: str,
    altitude_reference: str = "",
) -> LocalNedSample | None:
    lat_deg = scaled_degrees(lat)
    lon_deg = scaled_degrees(lon)
    alt_m = scaled_altitude_m(alt)
    if None in (lat_deg, lon_deg, alt_m):
        return None
    north_m, east_m, down_m = gps_lla_delta_to_ned_m(
        lat_deg=float(lat_deg),
        lon_deg=float(lon_deg),
        alt_m=float(alt_m),
        reference_lat_deg=reference_lat_deg,
        reference_lon_deg=reference_lon_deg,
        reference_alt_m=reference_alt_m,
    )
    return LocalNedSample(
        north_m=north_m,
        east_m=east_m,
        down_m=down_m,
        source_frame=GLOBAL_LLA_FRAME,
        source_axis_contract=GLOBAL_LLA_AXIS_CONTRACT,
        source=source,
        raw_lat_deg=float(lat_deg),
        raw_lon_deg=float(lon_deg),
        raw_alt_m=float(alt_m),
        origin_lat_deg=float(reference_lat_deg),
        origin_lon_deg=float(reference_lon_deg),
        origin_alt_m=float(reference_alt_m),
        origin_kind=origin_kind,
        altitude_reference=altitude_reference,
    )
