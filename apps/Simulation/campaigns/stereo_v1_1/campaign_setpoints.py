"""MAVLink offboard setpoints for the v1.1 continuous science path.

All positions are PX4 local NED [north,east,down] metres; velocities are
PX4 local NED [north,east,down] m/s; yaw is clockwise from North in radians.
This module never accepts world-ENU arrays directly.
"""

from __future__ import annotations

import math

from pymavlink import mavutil

from .trajectory_contract import LocalNed, LocalNedVelocity


def _validate(position: LocalNed, velocity: LocalNedVelocity | None, yaw_rad: float, time_boot_ms: int) -> None:
    if not isinstance(position, LocalNed):
        raise TypeError("position must be LocalNed")
    if velocity is not None and not isinstance(velocity, LocalNedVelocity):
        raise TypeError("velocity must be LocalNedVelocity")
    values = (*position.as_xyz(), *(velocity.as_xyz() if velocity else ()), float(yaw_rad))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("NED setpoint values must be finite")
    if isinstance(time_boot_ms, bool) or not isinstance(time_boot_ms, int) or not 0 <= time_boot_ms <= 0xFFFFFFFF:
        raise ValueError("time_boot_ms must be uint32")


def send_position_yaw(master, time_boot_ms: int, position: LocalNed, yaw_rad: float) -> None:
    """Hold/approach at local NED position with an active heading command."""
    _validate(position, None, yaw_rad, time_boot_ms)
    mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        time_boot_ms, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
        *position.as_xyz(), 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, float(yaw_rad), 0.0,
    )


def send_position_velocity_yaw(
    master,
    time_boot_ms: int,
    position: LocalNed,
    velocity: LocalNedVelocity,
    yaw_rad: float,
) -> None:
    """Track a time-reference position/velocity with active NED heading."""
    _validate(position, velocity, yaw_rad, time_boot_ms)
    mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        time_boot_ms, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
        *position.as_xyz(), *velocity.as_xyz(),
        0.0, 0.0, 0.0, float(yaw_rad), 0.0,
    )
