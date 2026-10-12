"""Independent ENU/NED sentinels for the v1.1 continuous reference."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.trajectory_contract import (
    LocalNed,
    LocalNedVelocity,
    WorldEnu,
    heading_clockwise_from_north_to_px4_yaw_rad,
    interpolate_reference,
    observed_origin_world_enu,
    world_position_to_px4_local_ned,
    world_velocity_to_px4_local_ned,
)


def test_observed_origin_and_position_conversion_use_distinct_axis_orders():
    # Hand-built values keep east/north and up/down distinct in every field.
    gt = WorldEnu(east_m=10.0, north_m=400.0, up_m=101.0)
    local = LocalNed(north_m=-15.0, east_m=2.0, down_m=-44.0)
    origin = observed_origin_world_enu(gt, local)
    assert origin == WorldEnu(east_m=8.0, north_m=415.0, up_m=57.0)
    assert world_position_to_px4_local_ned(gt, origin) == local
    assert WorldEnu.from_reference_xyz([10.0, 400.0, 101.0]) == gt
    assert LocalNed.from_px4_xyz([-15.0, 2.0, -44.0]) == local


def test_reference_velocity_heading_and_static_preflight_sample():
    assert world_velocity_to_px4_local_ned([1.0, 2.0, 3.0]) == LocalNedVelocity(2.0, 1.0, -3.0)
    assert heading_clockwise_from_north_to_px4_yaw_rad(0.0) == pytest.approx(0.0)
    assert heading_clockwise_from_north_to_px4_yaw_rad(90.0) == pytest.approx(math.pi / 2)
    assert heading_clockwise_from_north_to_px4_yaw_rad(180.0) == pytest.approx(math.pi)

    # The observed disarmed sample is near world (0,415,1), not configured Z=6.
    observed = observed_origin_world_enu(
        WorldEnu(0.0, 415.0, 1.0264498693713398),
        LocalNed(-0.0017620022408664227, -0.03957267105579376, -0.05333065986633301),
    )
    assert observed.east_m == pytest.approx(0.03957267105579376)
    assert observed.north_m == pytest.approx(415.00176200224087)
    assert observed.up_m == pytest.approx(0.9731192095050067)


def test_reference_interpolates_inside_science_and_rejects_extrapolation():
    samples = [
        {
            "t_science_s": 0.0,
            "position_world_enu_m": [1.0, 400.0, 100.0],
            "velocity_world_enu_mps": [0.0, -2.0, -1.0],
            "acceleration_world_enu_mps2": [1.0, 0.0, 0.0],
            "heading_deg_clockwise_from_north": 180.0,
        },
        {
            "t_science_s": 0.02,
            "position_world_enu_m": [3.0, 398.0, 98.0],
            "velocity_world_enu_mps": [2.0, -4.0, -3.0],
            "acceleration_world_enu_mps2": [3.0, 2.0, -2.0],
            "heading_deg_clockwise_from_north": 180.0,
        },
    ]
    middle = interpolate_reference(samples, 0.01)
    assert middle["position_world_enu_m"] == pytest.approx([2.0, 399.0, 99.0])
    assert middle["velocity_world_enu_mps"] == pytest.approx([1.0, -3.0, -2.0])
    assert middle["acceleration_world_enu_mps2"] == pytest.approx([2.0, 1.0, -1.0])
    assert middle["heading_deg_clockwise_from_north"] == 180.0
    for t in (-0.01, 0.03):
        with pytest.raises(ValueError, match="outside reference"):
            interpolate_reference(samples, t)
