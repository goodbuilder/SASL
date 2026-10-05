from __future__ import annotations

from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.gps_baseline import RawGpsFix, gps_baseline_enu_m, gps_relative_enu_m, norm_m


def _fix(latitude_deg: float, longitude_deg: float, altitude_m: float = 10.0) -> RawGpsFix:
    return RawGpsFix(1, latitude_deg, longitude_deg, altitude_m, 0.9, 1.8)


def test_raw_gps_baseline_is_expressed_as_right_minus_left_enu() -> None:
    left = _fix(47.3979712, 8.5459985)
    right = _fix(47.3979712, 8.5463293)
    east, north, up = gps_baseline_enu_m(left, right)
    assert east == pytest.approx(25.0, abs=0.2)
    assert north == pytest.approx(0.0, abs=0.02)
    assert up == pytest.approx(0.0, abs=0.02)
    assert norm_m((east, north, up)) == pytest.approx(25.0, abs=0.2)


def test_relative_enu_can_use_the_left_camera_fix_as_a_uam_origin() -> None:
    left = _fix(47.3979712, 8.5459985)
    uam = _fix(47.3979712, 8.5463293, altitude_m=22.0)

    east, north, up = gps_relative_enu_m(left, uam)

    assert east == pytest.approx(25.0, abs=0.2)
    assert north == pytest.approx(0.0, abs=0.02)
    assert up == pytest.approx(12.0, abs=0.02)


def test_raw_gps_baseline_rejects_non_finite_coordinates() -> None:
    with pytest.raises(ValueError, match="finite"):
        gps_baseline_enu_m(_fix(float("nan"), 8.0), _fix(47.0, 8.1))
