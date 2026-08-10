"""Validated GNSS coordinate profiles and their immutable provenance."""

from __future__ import annotations

from dataclasses import dataclass, asdict
import hashlib
import json
import math
from statistics import median, pstdev
from typing import Any, Iterable

from Workstation.workstation_core.gnss_operation import HeightType

EARTH_RADIUS_M = 6_378_137.0
WGS84_SEMI_MAJOR_M = 6_378_137.0
WGS84_FIRST_ECCENTRICITY_SQUARED = 6.6943799901413165e-3


def geodetic_to_ecef_m(latitude_deg: float, longitude_deg: float, height_m: float) -> tuple[float, float, float]:
    """Convert WGS84 geodetic latitude/longitude/ellipsoid height to ECEF metres."""

    latitude_rad = math.radians(_finite(latitude_deg, "latitude_deg"))
    longitude_rad = math.radians(_finite(longitude_deg, "longitude_deg"))
    height = _finite(height_m, "height_m")
    sin_lat = math.sin(latitude_rad)
    cos_lat = math.cos(latitude_rad)
    sin_lon = math.sin(longitude_rad)
    cos_lon = math.cos(longitude_rad)
    prime_vertical_radius_m = WGS84_SEMI_MAJOR_M / math.sqrt(
        1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_lat * sin_lat
    )
    return (
        (prime_vertical_radius_m + height) * cos_lat * cos_lon,
        (prime_vertical_radius_m + height) * cos_lat * sin_lon,
        (prime_vertical_radius_m * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED) + height) * sin_lat,
    )


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class GnssCoordinateSample:
    latitude_deg: float
    longitude_deg: float
    ellipsoid_height_m: float | None
    msl_altitude_m: float | None = None
    height_type: HeightType = HeightType.UNKNOWN
    timestamp_s: float = 0.0
    fixed: bool = False
    device_id: str = ""

    def __post_init__(self) -> None:
        lat = _finite(self.latitude_deg, "latitude_deg")
        lon = _finite(self.longitude_deg, "longitude_deg")
        if not -90.0 <= lat <= 90.0:
            raise ValueError("latitude_deg must be between -90 and 90")
        if not -180.0 <= lon <= 180.0:
            raise ValueError("longitude_deg must be between -180 and 180")
        object.__setattr__(self, "latitude_deg", lat)
        object.__setattr__(self, "longitude_deg", lon)
        for name in ("ellipsoid_height_m", "msl_altitude_m", "timestamp_s"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _finite(value, name))
        object.__setattr__(self, "height_type", HeightType(self.height_type))
        object.__setattr__(self, "fixed", bool(self.fixed))
        object.__setattr__(self, "device_id", str(self.device_id))

    @property
    def selected_height_m(self) -> float | None:
        if self.height_type == HeightType.ELLIPSOID:
            return self.ellipsoid_height_m
        if self.height_type == HeightType.MSL:
            return self.msl_altitude_m
        return None


@dataclass(frozen=True)
class GnssCoordinateProfile:
    latitude_deg: float
    longitude_deg: float
    height_m: float
    height_type: HeightType
    acquisition_start_utc: str
    acquisition_end_utc: str
    sample_count: int
    fixed_sample_count: int
    fixed_ratio: float
    fixed_duration_s: float
    collection_duration_s: float
    horizontal_scatter_m: float
    vertical_scatter_m: float
    source_caster: str
    source_mountpoint: str
    coordinate_source_type: str
    antenna_height_m: float | None
    antenna_reference_point: str
    datum: str
    reference_epoch: str | None = None
    msl_altitude_m: float | None = None
    profile_hash: str = ""

    def __post_init__(self) -> None:
        for name in ("latitude_deg", "longitude_deg", "height_m", "fixed_ratio", "fixed_duration_s", "collection_duration_s", "horizontal_scatter_m", "vertical_scatter_m"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("profile latitude_deg is out of range")
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError("profile longitude_deg is out of range")
        if not 0.0 <= self.fixed_ratio <= 1.0:
            raise ValueError("profile fixed_ratio must be between 0 and 1")
        for name in ("sample_count", "fixed_sample_count"):
            value = int(getattr(self, name))
            if value < 0:
                raise ValueError(f"{name} must be >= 0")
            object.__setattr__(self, name, value)
        if self.height_type == HeightType.UNKNOWN:
            raise ValueError("a validated coordinate profile cannot have UNKNOWN height_type")
        object.__setattr__(self, "height_type", HeightType(self.height_type))
        if self.antenna_height_m is not None:
            object.__setattr__(self, "antenna_height_m", _finite(self.antenna_height_m, "antenna_height_m"))
        if self.msl_altitude_m is not None:
            object.__setattr__(self, "msl_altitude_m", _finite(self.msl_altitude_m, "msl_altitude_m"))
        object.__setattr__(self, "profile_hash", self.profile_hash or self.compute_hash())

    def canonical_payload(self) -> dict[str, Any]:
        payload = asdict(self)
        payload.pop("profile_hash", None)
        payload["height_type"] = self.height_type.value
        return payload

    def compute_hash(self) -> str:
        return hashlib.sha256(
            json.dumps(self.canonical_payload(), sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
        ).hexdigest()

    def to_mapping(self) -> dict[str, Any]:
        payload = self.canonical_payload()
        payload["profile_hash"] = self.profile_hash
        return payload


def _local_xy_m(sample: GnssCoordinateSample, *, latitude_origin_deg: float, longitude_origin_deg: float) -> tuple[float, float]:
    lat_scale = math.pi / 180.0 * EARTH_RADIUS_M
    lon_scale = lat_scale * math.cos(math.radians(latitude_origin_deg))
    return (
        (sample.longitude_deg - longitude_origin_deg) * lon_scale,
        (sample.latitude_deg - latitude_origin_deg) * lat_scale,
    )


def build_coordinate_profile(
    samples: Iterable[GnssCoordinateSample],
    *,
    acquisition_start_utc: str,
    acquisition_end_utc: str,
    source_caster: str = "",
    source_mountpoint: str = "",
    coordinate_source_type: str = "NETWORK_ASSISTED_FIXED_BASE",
    antenna_height_m: float | None = None,
    antenna_reference_point: str = "",
    datum: str = "WGS84",
    reference_epoch: str | None = None,
) -> GnssCoordinateProfile:
    rows = tuple(samples)
    if not rows:
        raise ValueError("at least one GNSS coordinate sample is required")
    fixed_rows = tuple(row for row in rows if row.fixed)
    height_types = {row.height_type for row in fixed_rows}
    if len(height_types) != 1 or HeightType.UNKNOWN in height_types:
        raise ValueError("fixed coordinate samples must have one known height_type")
    height_type = next(iter(height_types))
    valid_rows = tuple(row for row in fixed_rows if row.selected_height_m is not None)
    if len(valid_rows) != len(fixed_rows):
        raise ValueError("fixed coordinate samples are missing the selected height field")
    lat = float(median(row.latitude_deg for row in valid_rows))
    lon = float(median(row.longitude_deg for row in valid_rows))
    heights = [float(row.selected_height_m) for row in valid_rows if row.selected_height_m is not None]
    height = float(median(heights))
    xy = [_local_xy_m(row, latitude_origin_deg=lat, longitude_origin_deg=lon) for row in valid_rows]
    horizontal_scatter = max((math.hypot(east, north) for east, north in xy), default=0.0)
    vertical_scatter = pstdev(heights) if len(heights) > 1 else 0.0
    timestamp_values = [row.timestamp_s for row in rows]
    fixed_timestamps = [row.timestamp_s for row in fixed_rows]
    return GnssCoordinateProfile(
        latitude_deg=lat,
        longitude_deg=lon,
        height_m=height,
        height_type=height_type,
        acquisition_start_utc=str(acquisition_start_utc),
        acquisition_end_utc=str(acquisition_end_utc),
        sample_count=len(rows),
        fixed_sample_count=len(fixed_rows),
        fixed_ratio=len(fixed_rows) / len(rows),
        fixed_duration_s=(max(fixed_timestamps) - min(fixed_timestamps)) if len(fixed_timestamps) > 1 else 0.0,
        collection_duration_s=(max(timestamp_values) - min(timestamp_values)) if len(timestamp_values) > 1 else 0.0,
        horizontal_scatter_m=float(horizontal_scatter),
        vertical_scatter_m=float(vertical_scatter),
        source_caster=str(source_caster),
        source_mountpoint=str(source_mountpoint),
        coordinate_source_type=str(coordinate_source_type),
        antenna_height_m=antenna_height_m,
        antenna_reference_point=str(antenna_reference_point),
        datum=str(datum),
        reference_epoch=None if reference_epoch is None else str(reference_epoch),
        msl_altitude_m=(float(median([row.msl_altitude_m for row in valid_rows if row.msl_altitude_m is not None])) if any(row.msl_altitude_m is not None for row in valid_rows) else None),
    )
