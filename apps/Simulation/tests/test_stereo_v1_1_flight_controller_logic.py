"""Entry lead and heading error boundaries without starting SITL."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.flight_controller import (
    _angle_error_deg,
    _heading_error_deg,
    _quaternion_yaw_deg_xyzw,
    bounded_approach_target,
)
from campaigns.stereo_v1_1.trajectory_contract import LocalNed


def test_entry_target_limits_position_lead_in_local_ned():
    current = LocalNed(north_m=0.0, east_m=1.0, down_m=-1.0)
    distant = LocalNed(north_m=-30.0, east_m=5.0, down_m=-141.0)
    command = bounded_approach_target(current, distant, max_lead_m=5.0)
    assert math.dist(command.as_xyz(), current.as_xyz()) == pytest.approx(5.0)
    assert command.north_m < 0.0
    assert command.east_m > 1.0
    assert command.down_m < -1.0
    near = LocalNed(north_m=-1.0, east_m=1.1, down_m=-2.0)
    assert bounded_approach_target(current, near) == near


def test_heading_error_wraps_north_to_east_yaw_at_180_degrees():
    assert _heading_error_deg(math.radians(-179.0), math.radians(179.0)) == pytest.approx(2.0)
    assert _heading_error_deg(math.pi, math.pi) == pytest.approx(0.0)


def test_spawn_yaw_check_reads_world_yaw_from_xyzw_quaternion():
    yaw_minus_90 = [0.0, 0.0, math.sin(math.radians(-45)), math.cos(math.radians(-45))]
    observed = _quaternion_yaw_deg_xyzw(yaw_minus_90)
    assert observed == pytest.approx(-90.0)
    assert _angle_error_deg(observed, 270.0) == pytest.approx(0.0)
