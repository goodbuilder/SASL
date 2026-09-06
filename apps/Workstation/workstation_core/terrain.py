"""GeoTIFF metadata and local-NED terrain helpers.

Rasterio and PyProj are imported lazily so raw recording remains available on
machines that have not installed the optional terrain dependencies.  Terrain
pose solving, however, must report ``DEPENDENCY_MISSING`` rather than silently
falling back to an unreferenced image.

Coordinate contract
-------------------
GeoTIFF coordinates remain in the dataset CRS until PyProj transforms them to
WGS84 longitude/latitude/ellipsoid-height.  ``LocalNedFrame`` then converts
ECEF metres to local ``[north,east,down]`` metres.  A raster height is never
assumed to be ellipsoidal unless its vertical datum is explicitly declared.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from Workstation.workstation_core.strict_jsonl import sha256_file


class TerrainDependencyError(RuntimeError):
    pass


def _terrain_dependencies() -> tuple[Any, Any]:
    try:
        import rasterio  # type: ignore[import-not-found]
        from pyproj import Transformer  # type: ignore[import-not-found]
    except ImportError as exc:
        raise TerrainDependencyError(
            "GeoTIFF pose requires rasterio>=1.3 and pyproj>=3.5; raw recording remains available"
        ) from exc
    return rasterio, Transformer


@dataclass(frozen=True)
class TerrainMetadata:
    path: str
    sha256: str
    width: int
    height: int
    band_count: int
    dtype: str
    crs_wkt: str
    affine_gdal: tuple[float, float, float, float, float, float]
    bounds: tuple[float, float, float, float]
    nodata: float | None
    horizontal_datum: str
    vertical_datum: str
    transformer_definition: str
    transformer_accuracy_m: float | None

    @property
    def full_pnp_height_ready(self) -> bool:
        return bool(self.vertical_datum and self.vertical_datum.lower() not in {"unknown", "unspecified"})

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema": "terrain-asset-v1",
            "path": self.path,
            "sha256": self.sha256,
            "width": self.width,
            "height": self.height,
            "band_count": self.band_count,
            "dtype": self.dtype,
            "crs_wkt": self.crs_wkt,
            "affine_gdal": list(self.affine_gdal),
            "bounds": list(self.bounds),
            "nodata": self.nodata,
            "horizontal_datum": self.horizontal_datum,
            "vertical_datum": self.vertical_datum,
            "transformer_definition": self.transformer_definition,
            "transformer_accuracy_m": self.transformer_accuracy_m,
            "full_pnp_height_ready": self.full_pnp_height_ready,
            "coordinate_contract": "dataset CRS -> WGS84 lon/lat/height -> local NED metres",
        }


def inspect_geotiff(path: Path | str, *, vertical_datum: str) -> TerrainMetadata:
    rasterio, Transformer = _terrain_dependencies()
    source = Path(path).expanduser().resolve(strict=True)
    with rasterio.open(source) as dataset:
        if dataset.crs is None:
            raise ValueError("GeoTIFF CRS is required")
        if dataset.count < 1:
            raise ValueError("GeoTIFF must contain an elevation band")
        transformer = Transformer.from_crs(dataset.crs, "EPSG:4979", always_xy=True)
        accuracy = getattr(transformer, "accuracy", None)
        try:
            accuracy_value = float(accuracy)
            if not math.isfinite(accuracy_value) or accuracy_value < 0.0:
                accuracy_value = None
        except (TypeError, ValueError):
            accuracy_value = None
        crs_wkt = dataset.crs.to_wkt()
        datum = ""
        try:
            datum = str(dataset.crs.to_dict().get("datum") or "")
        except AttributeError:
            datum = ""
        return TerrainMetadata(
            path=str(source),
            sha256=sha256_file(source),
            width=int(dataset.width),
            height=int(dataset.height),
            band_count=int(dataset.count),
            dtype=str(dataset.dtypes[0]),
            crs_wkt=crs_wkt,
            affine_gdal=tuple(float(value) for value in dataset.transform.to_gdal()),
            bounds=tuple(float(value) for value in dataset.bounds),
            nodata=None if dataset.nodata is None else float(dataset.nodata),
            horizontal_datum=datum or "encoded_in_crs_wkt",
            vertical_datum=str(vertical_datum or "unknown"),
            transformer_definition=str(getattr(transformer, "definition", "") or ""),
            transformer_accuracy_m=accuracy_value,
        )


@dataclass(frozen=True)
class LocalNedFrame:
    latitude_deg: float
    longitude_deg: float
    ellipsoid_height_m: float
    source: str

    def __post_init__(self) -> None:
        values = (float(self.latitude_deg), float(self.longitude_deg), float(self.ellipsoid_height_m))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("local NED origin must be finite")
        if not -90.0 <= values[0] <= 90.0 or not -180.0 <= values[1] <= 180.0:
            raise ValueError("invalid WGS84 origin")
        object.__setattr__(self, "latitude_deg", values[0])
        object.__setattr__(self, "longitude_deg", values[1])
        object.__setattr__(self, "ellipsoid_height_m", values[2])
        object.__setattr__(self, "source", str(self.source))

    def wgs84_to_ned(self, longitude_deg: float, latitude_deg: float, ellipsoid_height_m: float) -> tuple[float, float, float]:
        _, Transformer = _terrain_dependencies()
        transformer = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        origin = transformer.transform(self.longitude_deg, self.latitude_deg, self.ellipsoid_height_m)
        point = transformer.transform(float(longitude_deg), float(latitude_deg), float(ellipsoid_height_m))
        dx, dy, dz = (float(current) - float(reference) for current, reference in zip(point, origin))
        latitude = math.radians(self.latitude_deg)
        longitude = math.radians(self.longitude_deg)
        north = -math.sin(latitude) * math.cos(longitude) * dx - math.sin(latitude) * math.sin(longitude) * dy + math.cos(latitude) * dz
        east = -math.sin(longitude) * dx + math.cos(longitude) * dy
        down = -math.cos(latitude) * math.cos(longitude) * dx - math.cos(latitude) * math.sin(longitude) * dy - math.sin(latitude) * dz
        return north, east, down


__all__ = [
    "LocalNedFrame",
    "TerrainDependencyError",
    "TerrainMetadata",
    "inspect_geotiff",
]
