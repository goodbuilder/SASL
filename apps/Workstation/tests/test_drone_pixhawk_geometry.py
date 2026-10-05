from __future__ import annotations

import math

import pytest

from Workstation.workstation_core.drone_pixhawk_geometry import (
    calculate_horizontal_triangle_angles,
    horizontal_angle_deg,
    vector_range_m,
)


def test_horizontal_triangle_angles_are_independent_angles_at_p1_and_p2() -> None:
    result = calculate_horizontal_triangle_angles(
        (0.0, 0.0, 100.0),
        (10.0, 0.0, 0.0),
        (0.0, 10.0, -50.0),
    )

    assert result.at_p1_deg == pytest.approx(90.0)
    assert result.at_p2_deg == pytest.approx(45.0)


def test_horizontal_triangle_angles_ignore_down_and_return_collinear_limits() -> None:
    result = calculate_horizontal_triangle_angles(
        (0.0, 0.0, -1000.0),
        (10.0, 0.0, 0.0),
        (20.0, 0.0, 1000.0),
    )

    assert result.at_p1_deg == pytest.approx(0.0)
    assert result.at_p2_deg == pytest.approx(180.0)


def test_horizontal_angle_is_undefined_for_zero_horizontal_ray() -> None:
    assert horizontal_angle_deg((0.0, 0.0, 4.0), (1.0, 0.0, 0.0)) is None
    result = calculate_horizontal_triangle_angles(
        (0.0, 0.0, 0.0),
        (0.0, 10.0, 0.0),
        (0.0, 0.0, 30.0),
    )
    assert result.at_p1_deg is None
    assert result.at_p2_deg == pytest.approx(0.0)


@pytest.mark.parametrize(
    "positions",
    (
        ((math.nan, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        ((0.0, 0.0, 0.0), (math.inf, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ),
)
def test_horizontal_triangle_angles_reject_nonfinite_positions(positions) -> None:
    with pytest.raises(ValueError):
        calculate_horizontal_triangle_angles(*positions)


def test_vector_range_uses_all_frd_components() -> None:
    assert vector_range_m((3.0, 4.0, 12.0)) == pytest.approx(13.0)
