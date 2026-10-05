from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from Workstation.workstation_core.terrain import (
    LocalNedFrame,
    TerrainDependencyError,
    inspect_geotiff,
)


def test_missing_terrain_dependencies_fail_explicitly_without_affecting_raw_recording(tmp_path: Path) -> None:
    if importlib.util.find_spec("rasterio") is not None and importlib.util.find_spec("pyproj") is not None:
        pytest.skip("terrain dependencies are installed in this environment")

    with pytest.raises(TerrainDependencyError, match="raw recording remains available"):
        inspect_geotiff(tmp_path / "missing.tif", vertical_datum="unknown")


def test_local_ned_origin_validates_wgs84_contract() -> None:
    with pytest.raises(ValueError, match="invalid WGS84"):
        LocalNedFrame(95.0, 127.0, 10.0, "test")


@pytest.mark.skipif(importlib.util.find_spec("pyproj") is None, reason="pyproj not installed")
def test_local_ned_axes_are_north_east_down() -> None:
    origin = LocalNedFrame(37.0, 127.0, 100.0, "synthetic")

    north = origin.wgs84_to_ned(127.0, 37.00001, 100.0)
    east = origin.wgs84_to_ned(127.00001, 37.0, 100.0)
    up = origin.wgs84_to_ned(127.0, 37.0, 101.0)

    assert north[0] > 1.0 and abs(north[1]) < 0.01
    assert east[1] > 0.8 and abs(east[0]) < 0.01
    assert up[2] == pytest.approx(-1.0, abs=1e-5)

