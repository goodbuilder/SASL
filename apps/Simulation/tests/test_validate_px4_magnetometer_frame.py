from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_px4_magnetometer_frame.py"


def _module():
    spec = importlib.util.spec_from_file_location("validate_px4_magnetometer_frame", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VALIDATOR = _module()


def test_expected_body_field_matches_hand_calculated_pitch_twenty_fixture() -> None:
    actual = VALIDATOR.expected_body_field_gauss(
        declination_deg=3.405765295,
        inclination_deg=63.157466888,
        strength_gauss=0.482388168166,
        roll_deg=0.0,
        pitch_deg=20.0,
        yaw_deg=0.0,
    )

    assert actual == pytest.approx(
        (0.057110739, 0.012939849, 0.478820709), abs=1.0e-8
    )


@pytest.mark.parametrize(
    "rpy_deg",
    ((0.0, 0.0, 0.0), (0.0, 20.0, 0.0), (10.0, 20.0, 15.0)),
)
def test_expected_body_field_preserves_wmm_strength(rpy_deg) -> None:
    strength = 0.482388168166

    body = VALIDATOR.expected_body_field_gauss(
        declination_deg=3.405765295,
        inclination_deg=63.157466888,
        strength_gauss=strength,
        roll_deg=rpy_deg[0],
        pitch_deg=rpy_deg[1],
        yaw_deg=rpy_deg[2],
    )

    assert math.sqrt(sum(value * value for value in body)) == pytest.approx(
        strength, abs=1.0e-12
    )


@pytest.mark.parametrize(
    ("angle", "expected"),
    ((0.0, 0.0), (181.0, -179.0), (-181.0, 179.0), (540.0, -180.0)),
)
def test_wrap_deg_uses_half_open_interval(angle: float, expected: float) -> None:
    assert VALIDATOR.wrap_deg(angle) == pytest.approx(expected)
