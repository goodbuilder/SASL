"""v1.2 entry-only vertical launch clearance before horizontal transit.

Method: hold launch PX4 local NED north/east, climb to 10 m above launch,
then require local NED and independent world ENU rise within 0.5 m of that
waypoint, horizontal drift <=1 m and speed <=1 m/s for 1 s of simulation
observations (gaps <=0.2 s). GT is SITL entry validation, never a calibration
estimator input. Clock/local resets fail instead of reinterpreting launch.
This engineering stage does not change the science trajectory or controller gains.
"""
from __future__ import annotations

import math

from .trajectory_contract import LocalNed


class VerticalLiftoffGate:
    def __init__(self, launch):
        self.launch_local = launch.px4_local_ned_m
        self.launch_world = launch.gt_world_enu_m
        self.epoch = launch.simulation_epoch
        self.resets = (launch.xy_reset_counter, launch.z_reset_counter)
        self.target_rise_m = 10.0
        self.last_stamp_ns = None
        self.stable_since_ns = None

    def target(self, actual: LocalNed, lead_m: float) -> LocalNed:
        if not math.isfinite(lead_m) or lead_m <= 0:
            raise ValueError('vertical lead must be positive and finite')
        return LocalNed(self.launch_local.north_m, self.launch_local.east_m,
                        max(self.launch_local.down_m - self.target_rise_m,
                            actual.down_m - lead_m))

    def observe(self, snapshot) -> bool:
        if snapshot.simulation_epoch != self.epoch:
            raise RuntimeError('simulation clock reset during vertical liftoff')
        if (snapshot.xy_reset_counter, snapshot.z_reset_counter) != self.resets:
            raise RuntimeError('PX4 local frame reset during vertical liftoff')
        stamp = snapshot.simulation_time_ns
        if self.last_stamp_ns is not None and stamp < self.last_stamp_ns:
            raise RuntimeError('simulation time moved backwards during vertical liftoff')
        if self.last_stamp_ns == stamp:
            return False
        local = snapshot.px4_local_ned_m
        local_rise = self.launch_local.down_m - local.down_m
        world_rise = snapshot.gt_world_enu_m.up_m - self.launch_world.up_m
        ready = (abs(local_rise - self.target_rise_m) <= 0.5
                 and abs(world_rise - self.target_rise_m) <= 0.5
                 and math.hypot(local.north_m - self.launch_local.north_m,
                                local.east_m - self.launch_local.east_m) <= 1.0
                 and snapshot.speed_mps <= 1.0)
        if not ready:
            self.stable_since_ns = None
        elif (self.stable_since_ns is None or self.last_stamp_ns is None
              or stamp - self.last_stamp_ns > 200_000_000):
            self.stable_since_ns = stamp
        self.last_stamp_ns = stamp
        return self.stable_since_ns is not None and stamp - self.stable_since_ns >= 1_000_000_000
