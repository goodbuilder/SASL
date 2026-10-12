"""ROS payload adapter keeps source clock and ENU/NED fields distinct."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace as Obj

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.ros_observation_bridge import RosObservationBridge
from campaigns.stereo_v1_1.runtime_observation import ObservationCache


def test_ros_callback_payloads_preserve_world_and_local_frames(monkeypatch):
    bridge = RosObservationBridge.__new__(RosObservationBridge)
    bridge.cache = ObservationCache()
    bridge.last_error = None
    monkeypatch.setattr("campaigns.stereo_v1_1.ros_observation_bridge.time.monotonic_ns", lambda: 20_000_000_000)
    bridge._clock(Obj(clock=Obj(sec=10, nanosec=0)))
    bridge._gt(Obj(
        header=Obj(stamp=Obj(sec=9, nanosec=990_000_000), frame_id="gazebo_world_enu"),
        pose=Obj(position=Obj(x=2.0, y=415.0, z=1.0),
                 orientation=Obj(x=0.0, y=0.0, z=-0.7071067811865476, w=0.7071067811865476)),
    ))
    bridge._local(Obj(
        timestamp=123456, x=-3.0, y=4.0, z=-5.0,
        vx=0.1, vy=0.2, vz=-0.3, heading=3.14,
        xy_valid=True, z_valid=True,
        xy_reset_counter=4, z_reset_counter=2,
    ))
    snapshot = bridge.cache.snapshot(20_010_000_000)
    assert snapshot is not None
    assert snapshot.gt_stamp_ns == 9_990_000_000
    assert snapshot.gt_world_enu_m.as_xyz() == (2.0, 415.0, 1.0)
    assert snapshot.gt_orientation_world_from_reference_xyzw == pytest.approx(
        (0.0, 0.0, -0.7071067811865476, 0.7071067811865476)
    )
    assert snapshot.px4_local_ned_m.as_xyz() == (-3.0, 4.0, -5.0)
    assert snapshot.px4_velocity_ned_mps.as_xyz() == (0.1, 0.2, -0.3)
    assert snapshot.px4_timestamp_us == 123456
    assert bridge.last_error is None
