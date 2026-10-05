#!/usr/bin/env python3
"""Verify that a Gazebo world's <magnetic_field> is consistent with the
World Magnetic Model actually built into the checked-out PX4-Autopilot
revision, at that same world's <spherical_coordinates> latitude/longitude.

This does NOT reimplement or copy PX4's WMM lookup table or interpolation.
It compiles and runs PX4's own, unmodified
``src/lib/world_magnetic_model/geo_mag_declination.cpp`` (linked against a
minimal local shim for the one generic ``math::constrain`` utility it needs,
so it can be built standalone without the full PX4 NuttX/posix build system)
and reads back the declination/inclination/strength it actually returns for
the world's coordinates. It also requires the world plugin to declare ENU
earth-frame output explicitly. Body-frame FLU -> FRD delivery is verified
separately from real ULogs by ``validate_px4_magnetometer_frame.py``.

Usage:
    python3 apps/Simulation/stereo_ground_sim/tools/verify_magnetic_field.py \\
        --world apps/Simulation/stereo_ground_sim/worlds/stereo_ground_drone.sdf \\
        --px4-root apps/Simulation/PX4-Autopilot \\
        --max-declination-error-deg 0.05 \\
        --max-inclination-error-deg 0.05 \\
        --max-strength-relative-error 0.005
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET

_PROBE_SOURCE = """
#include <cstdio>
#include <cstdlib>
#include "geo_mag_declination.h"

int main(int argc, char **argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: probe <latitude_deg> <longitude_deg>\\n");
        return 2;
    }
    const double lat = std::atof(argv[1]);
    const double lon = std::atof(argv[2]);

    float decl = get_mag_declination_degrees((float)lat, (float)lon);
    float incl = get_mag_inclination_degrees((float)lat, (float)lon);
    float strength_tesla = get_mag_strength_tesla((float)lat, (float)lon);

    std::printf("declination_deg=%.9f\\n", (double)decl);
    std::printf("inclination_deg=%.9f\\n", (double)incl);
    std::printf("strength_tesla=%.15e\\n", (double)strength_tesla);
    return 0;
}
"""

# geo_mag_declination.cpp only calls math::constrain, a generic clamp with
# no PX4-specific behavior and no dependency on px4_platform_common; this
# shim pulls in the real, unmodified mathlib/math/Limits.hpp (where
# math::constrain actually lives) instead of the full mathlib.h umbrella
# header, which requires the full PX4 platform build.
_MATHLIB_SHIM = """
#include "{limits_hpp}"
"""


class MagneticFieldVerificationError(RuntimeError):
    """Raised for any condition that must abort verification with a clear error."""


def _fail(message: str) -> "SystemExit":
    return SystemExit(f"error: {message}")


def parse_world(world_path: Path) -> tuple[float, float, tuple[float, float, float]]:
    if not world_path.is_file():
        raise _fail(f"world file not found: {world_path}")
    try:
        root = ET.parse(world_path).getroot()
    except ET.ParseError as exc:
        raise _fail(f"world file is not valid XML: {exc}") from exc

    spherical = root.find(".//spherical_coordinates")
    if spherical is None:
        raise _fail("spherical_coordinates is missing from the world file")
    lat_node = spherical.find("latitude_deg")
    lon_node = spherical.find("longitude_deg")
    if lat_node is None or lon_node is None or not lat_node.text or not lon_node.text:
        raise _fail("spherical_coordinates is missing latitude_deg/longitude_deg")
    try:
        latitude_deg = float(lat_node.text)
        longitude_deg = float(lon_node.text)
    except ValueError as exc:
        raise _fail(f"spherical_coordinates latitude/longitude is not numeric: {exc}") from exc
    if not (math.isfinite(latitude_deg) and math.isfinite(longitude_deg)):
        raise _fail("spherical_coordinates latitude/longitude is NaN or Inf")

    mag_node = root.find(".//magnetic_field")
    if mag_node is None or not mag_node.text:
        raise _fail("magnetic_field is missing from the world file")
    parts = mag_node.text.split()
    if len(parts) != 3:
        raise _fail(f"magnetic_field must contain 3 values, got {len(parts)}")
    try:
        values = tuple(float(value) for value in parts)
    except ValueError as exc:
        raise _fail(f"magnetic_field contains a non-numeric value: {exc}") from exc
    if not all(math.isfinite(value) for value in values):
        raise _fail("magnetic_field contains NaN or Inf")
    if math.sqrt(sum(value * value for value in values)) <= 0.0:
        raise _fail("magnetic_field is a zero vector")

    plugins = [
        plugin
        for plugin in root.findall(".//world/plugin")
        if plugin.attrib.get("name") == "gz::sim::systems::Magnetometer"
    ]
    if len(plugins) != 1:
        raise _fail(
            "world must contain exactly one gz::sim::systems::Magnetometer plugin"
        )
    plugin = plugins[0]
    if (plugin.findtext("use_units_gauss") or "").strip().lower() != "true":
        raise _fail("magnetometer plugin must explicitly use gauss units")
    if (plugin.findtext("use_earth_frame_ned") or "").strip().lower() != "false":
        raise _fail("magnetometer plugin must explicitly emit the earth field in ENU")

    return latitude_deg, longitude_deg, values  # type: ignore[return-value]


def sdf_declination_inclination_strength(
    enu_tesla: tuple[float, float, float]
) -> tuple[float, float, float]:
    east_t, north_t, up_t = enu_tesla
    total_t = math.sqrt(east_t**2 + north_t**2 + up_t**2)
    declination_deg = math.degrees(math.atan2(east_t, north_t))
    horizontal_t = math.hypot(east_t, north_t)
    inclination_deg = math.degrees(math.atan2(-up_t, horizontal_t))
    return declination_deg, inclination_deg, total_t


def build_and_run_px4_wmm_probe(px4_root: Path, latitude_deg: float, longitude_deg: float) -> tuple[float, float, float]:
    px4_root = px4_root.resolve()
    wmm_dir = px4_root / "src" / "lib" / "world_magnetic_model"
    geo_mag_cpp = wmm_dir / "geo_mag_declination.cpp"
    geo_mag_h = wmm_dir / "geo_mag_declination.h"
    limits_hpp = px4_root / "src" / "lib" / "mathlib" / "math" / "Limits.hpp"
    for required in (geo_mag_cpp, geo_mag_h, limits_hpp):
        if not required.is_file():
            raise _fail(f"required PX4 WMM source file not found: {required}")

    with tempfile.TemporaryDirectory(prefix="verify_magnetic_field_") as tmp:
        tmp_path = Path(tmp)
        shim_dir = tmp_path / "shim_include" / "mathlib"
        shim_dir.mkdir(parents=True)
        (shim_dir / "mathlib.h").write_text(
            _MATHLIB_SHIM.format(limits_hpp=limits_hpp), encoding="utf-8"
        )
        probe_cpp = tmp_path / "probe.cpp"
        probe_cpp.write_text(_PROBE_SOURCE, encoding="utf-8")
        probe_bin = tmp_path / "probe"

        compile_cmd = [
            "g++", "-std=c++17", "-O2",
            "-I", str(tmp_path / "shim_include"),
            "-I", str(wmm_dir),
            "-o", str(probe_bin),
            str(probe_cpp),
            str(geo_mag_cpp),
        ]
        result = subprocess.run(compile_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            raise _fail(
                "failed to compile the PX4 WMM probe helper "
                f"(this helper only wraps PX4's own geo_mag_declination.cpp; it is not saved to the repository):\n"
                f"{result.stderr}"
            )

        run_result = subprocess.run(
            [str(probe_bin), str(latitude_deg), str(longitude_deg)],
            capture_output=True, text=True,
        )
        if run_result.returncode != 0:
            raise _fail(f"PX4 WMM probe helper exited with an error:\n{run_result.stderr}")

        values: dict[str, float] = {}
        for line in run_result.stdout.splitlines():
            if "=" not in line:
                continue
            key, _, raw_value = line.partition("=")
            try:
                values[key.strip()] = float(raw_value)
            except ValueError:
                continue

    for key in ("declination_deg", "inclination_deg", "strength_tesla"):
        if key not in values:
            raise _fail(f"PX4 WMM probe helper did not report {key}")
        if not math.isfinite(values[key]):
            raise _fail(f"PX4 WMM probe helper reported non-finite {key}")

    return values["declination_deg"], values["inclination_deg"], values["strength_tesla"]


def wrap_deg(angle_deg: float) -> float:
    """Wrap an angle difference into [-180, 180)."""

    return (angle_deg + 180.0) % 360.0 - 180.0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--px4-root", required=True, type=Path)
    parser.add_argument("--max-declination-error-deg", required=True, type=float)
    parser.add_argument("--max-inclination-error-deg", required=True, type=float)
    parser.add_argument("--max-strength-relative-error", required=True, type=float)
    args = parser.parse_args(argv)

    try:
        latitude_deg, longitude_deg, enu_tesla = parse_world(args.world)
        sdf_declination_deg, sdf_inclination_deg, sdf_total_t = sdf_declination_inclination_strength(enu_tesla)
        px4_declination_deg, px4_inclination_deg, px4_strength_t = build_and_run_px4_wmm_probe(
            args.px4_root, latitude_deg, longitude_deg
        )
    except SystemExit as exc:
        print(exc, file=sys.stderr)
        return 2

    declination_error_deg = abs(wrap_deg(sdf_declination_deg - px4_declination_deg))
    inclination_error_deg = abs(sdf_inclination_deg - px4_inclination_deg)
    strength_relative_error = abs(sdf_total_t - px4_strength_t) / px4_strength_t if px4_strength_t else math.inf

    passed = (
        declination_error_deg <= args.max_declination_error_deg
        and inclination_error_deg <= args.max_inclination_error_deg
        and strength_relative_error <= args.max_strength_relative_error
    )

    print(f"world={args.world}")
    print(f"latitude_deg={latitude_deg:.9f}")
    print(f"longitude_deg={longitude_deg:.9f}")
    print(f"sdf_enu_tesla=({enu_tesla[0]:.12e}, {enu_tesla[1]:.12e}, {enu_tesla[2]:.12e})")
    print(f"sdf_declination_deg={sdf_declination_deg:.9f}")
    print(f"sdf_inclination_deg={sdf_inclination_deg:.9f}")
    print(f"sdf_total_intensity_tesla={sdf_total_t:.12e}")
    print(f"px4_declination_deg={px4_declination_deg:.9f}")
    print(f"px4_inclination_deg={px4_inclination_deg:.9f}")
    print(f"px4_total_intensity_tesla={px4_strength_t:.12e}")
    print(f"declination_error_deg={declination_error_deg:.9f}")
    print(f"inclination_error_deg={inclination_error_deg:.9f}")
    print(f"strength_relative_error={strength_relative_error:.9f}")
    print("result=PASS" if passed else "result=FAIL")

    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
