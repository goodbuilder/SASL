from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from Workstation.workstation_core.terrain import (
    LocalNedFrame,
    TerrainDependencyError,
    inspect_geotiff,
)


@pytest.mark.parametrize("missing", ("rasterio", "pyproj"))
def test_missing_terrain_dependencies_fail_explicitly_without_affecting_raw_recording(tmp_path: Path, monkeypatch, missing) -> None:
    # Exercise the dependency failure even on fully provisioned test machines.
    import builtins
    original_import = builtins.__import__

    def require_available(name, *args, **kwargs):
        if name == missing:
            raise ImportError(f"simulated missing optional dependency: {missing}")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", require_available)
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

