"""MAVLink field-order and active-mask evidence independent of PX4 dynamics."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
from pymavlink import mavutil


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.campaign_setpoints import (
    send_position_velocity_yaw,
    send_position_yaw,
)
from campaigns.stereo_v1_1.trajectory_contract import LocalNed, LocalNedVelocity


class _Mav:
    def __init__(self):
        self.calls = []

    def set_position_target_local_ned_send(self, *args):
        self.calls.append(args)


class _Master:
    target_system = 7
    target_component = 2

    def __init__(self):
        self.mav = _Mav()


def test_science_setpoint_keeps_position_velocity_yaw_active_in_ned_order():
    master = _Master()
    send_position_velocity_yaw(
        master, 1234,
        LocalNed(north_m=-15.0, east_m=2.0, down_m=-144.0),
        LocalNedVelocity(north_mps=-4.0, east_mps=0.5, down_mps=1.0),
        math.pi,
    )
    args = master.mav.calls[0]
    assert args[:4] == (1234, 7, 2, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
    mask = args[4]
    for active in (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE,
    ):
        assert mask & active == 0
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    assert args[5:8] == (-15.0, 2.0, -144.0)
    assert args[8:11] == (-4.0, 0.5, 1.0)
    assert args[14] == math.pi
    assert args[15] == 0.0


def test_approach_setpoint_ignores_velocity_but_commands_yaw():
    master = _Master()
    send_position_yaw(master, 55, LocalNed(3.0, -2.0, -10.0), math.pi)
    args = master.mav.calls[0]
    mask = args[4]
    assert args[5:8] == (3.0, -2.0, -10.0)
    assert args[8:11] == (0.0, 0.0, 0.0)
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    assert mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE == 0


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_nonfinite_setpoints_fail_before_mavlink_send(bad):
    master = _Master()
    with pytest.raises(ValueError, match="finite"):
        send_position_yaw(master, 1, LocalNed(0.0, 0.0, bad), 0.0)
    assert not master.mav.calls
