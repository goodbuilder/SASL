"""Thread-safe SITL observations for a reference follower.

Gazebo PoseStamped position is world ENU [east,north,up] metres and its header
is simulation time. PX4 VehicleLocalPosition `x/y/z` and `vx/vy/vz` are local
NED [north,east,down] in metres and m/s; its uORB timestamp is a separate
clock domain. Pairing for origin observation uses callback monotonic times.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

from .trajectory_contract import LocalNed, LocalNedVelocity, WorldEnu


@dataclass(frozen=True)
class FlightSnapshot:
    simulation_time_ns: int
    simulation_epoch: int
    gt_stamp_ns: int
    gt_world_enu_m: WorldEnu
    gt_orientation_world_from_reference_xyzw: tuple[float, float, float, float]
    gt_receive_monotonic_ns: int
    px4_timestamp_us: int
    px4_local_ned_m: LocalNed
    px4_velocity_ned_mps: LocalNedVelocity
    px4_heading_rad: float
    px4_receive_monotonic_ns: int
    xy_reset_counter: int
    z_reset_counter: int

    @property
    def speed_mps(self) -> float:
        return math.sqrt(sum(value * value for value in self.px4_velocity_ned_mps.as_xyz()))


class ObservationCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._clock_ns: int | None = None
        self._epoch = 0
        self._gt: tuple[int, WorldEnu, tuple[float, float, float, float], int] | None = None
        self._local: tuple[int, LocalNed, LocalNedVelocity, float, int, int, int] | None = None

    def observe_clock(self, simulation_time_ns: int) -> None:
        stamp = int(simulation_time_ns)
        if stamp < 0:
            raise ValueError("simulation clock must be nonnegative")
        with self._lock:
            if self._clock_ns is not None and stamp < self._clock_ns:
                self._epoch += 1
                self._gt = None
                self._local = None
            self._clock_ns = stamp

    def observe_gt(
        self, stamp_ns: int, frame_id: str, world_position_xyz,
        receive_monotonic_ns: int, orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    ) -> None:
        if frame_id != "gazebo_world_enu":
            raise ValueError("GT position frame must be gazebo_world_enu")
        world = WorldEnu.from_reference_xyz(world_position_xyz)
        orientation = tuple(float(value) for value in orientation_xyzw)
        if len(orientation) != 4 or not all(math.isfinite(value) for value in orientation):
            raise ValueError("GT orientation must contain four finite xyzw values")
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm <= 0 or abs(norm - 1.0) > 1e-3:
            raise ValueError("GT orientation quaternion must be unit length")
        with self._lock:
            self._gt = (int(stamp_ns), world, orientation, int(receive_monotonic_ns))

    def observe_local(
        self,
        *,
        timestamp_us: int,
        position_xyz,
        velocity_xyz,
        heading_rad: float,
        xy_valid: bool,
        z_valid: bool,
        xy_reset_counter: int,
        z_reset_counter: int,
        receive_monotonic_ns: int,
    ) -> None:
        if not xy_valid or not z_valid:
            with self._lock:
                self._local = None
            return
        local = LocalNed.from_px4_xyz(position_xyz)
        if len(velocity_xyz) != 3:
            raise ValueError("PX4 velocity must have three NED components")
        velocity = LocalNedVelocity(*(float(value) for value in velocity_xyz))
        heading = float(heading_rad)
        if not all(math.isfinite(value) for value in (*velocity.as_xyz(), heading)):
            raise ValueError("PX4 local velocity/heading must be finite")
        with self._lock:
            self._local = (
                int(timestamp_us), local, velocity, heading,
                int(receive_monotonic_ns), int(xy_reset_counter), int(z_reset_counter),
            )

    def snapshot(
        self,
        now_monotonic_ns: int,
        *,
        max_receive_age_ns: int = 500_000_000,
        max_pair_skew_ns: int = 100_000_000,
        max_gt_clock_lag_ns: int = 200_000_000,
        max_gt_future_ns: int = 40_000_000,
    ) -> FlightSnapshot | None:
        with self._lock:
            if self._clock_ns is None or self._gt is None or self._local is None:
                return None
            clock_ns, epoch, gt, local = self._clock_ns, self._epoch, self._gt, self._local
        gt_stamp, world, orientation, gt_receive = gt
        local_stamp, position, velocity, heading, local_receive, xy_reset, z_reset = local
        now = int(now_monotonic_ns)
        if (
            abs(gt_receive - local_receive) > max_pair_skew_ns
            or now < gt_receive or now < local_receive
            or now - gt_receive > max_receive_age_ns
            or now - local_receive > max_receive_age_ns
            or clock_ns - gt_stamp > max_gt_clock_lag_ns
            or gt_stamp - clock_ns > max_gt_future_ns
        ):
            return None
        return FlightSnapshot(
            clock_ns, epoch, gt_stamp, world, orientation, gt_receive, local_stamp,
            position, velocity, heading, local_receive, xy_reset, z_reset,
        )
