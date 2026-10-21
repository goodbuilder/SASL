from __future__ import annotations

import sys
import math
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.runtime_observation import ObservationCache


def _ready(cache, clock=10_000_000_000):
    cache.observe_clock(clock)
    cache.observe_gt(clock - 10_000_000, "gazebo_world_enu", [2.0, 415.0, 1.0], 20_000_000_000)
    cache.observe_local(
        timestamp_us=123456,
        position_xyz=[-3.0, 4.0, -5.0],
        velocity_xyz=[0.1, 0.2, -0.3],
        heading_rad=3.14,
        xy_valid=True, z_valid=True,
        xy_reset_counter=4, z_reset_counter=2,
        receive_monotonic_ns=20_020_000_000,
    )


def test_snapshot_names_distinct_frames_and_rejects_stale_pair():
    cache = ObservationCache()
    _ready(cache)
    snapshot = cache.snapshot(20_050_000_000)
    assert snapshot is not None
    assert snapshot.gt_world_enu_m.as_xyz() == (2.0, 415.0, 1.0)
    assert snapshot.gt_orientation_world_from_reference_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert snapshot.px4_local_ned_m.as_xyz() == (-3.0, 4.0, -5.0)
    assert snapshot.px4_velocity_ned_mps.as_xyz() == (0.1, 0.2, -0.3)
    assert cache.snapshot(21_000_000_000) is None
    with pytest.raises(ValueError, match="frame"):
        cache.observe_gt(10, "baseline_center_frd", [1, 2, 3], 20_000_000_000)


def test_snapshot_preserves_world_from_reference_quaternion():
    cache = ObservationCache()
    clock = 10_000_000_000
    yaw = [0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)]
    cache.observe_clock(clock)
    cache.observe_gt(clock - 10_000_000, "gazebo_world_enu", [0.0, 0.0, 0.0],
                     20_000_000_000, yaw)
    cache.observe_local(
        timestamp_us=123456, position_xyz=[0.0, 0.0, 0.0], velocity_xyz=[0.0, 0.0, 0.0],
        heading_rad=0.0, xy_valid=True, z_valid=True,
        xy_reset_counter=0, z_reset_counter=0, receive_monotonic_ns=20_020_000_000,
    )
    snapshot = cache.snapshot(20_050_000_000)
    assert snapshot is not None
    assert snapshot.gt_orientation_world_from_reference_xyzw == pytest.approx(yaw)


def test_clock_reset_clears_old_gt_and_local_samples():
    cache = ObservationCache()
    _ready(cache)
    assert cache.snapshot(20_050_000_000) is not None
    cache.observe_clock(100)
    assert cache.snapshot(20_050_000_000) is None
    cache.observe_gt(100, "gazebo_world_enu", [0, 0, 0], 20_100_000_000)
    cache.observe_local(
        timestamp_us=123457, position_xyz=[0, 0, 0], velocity_xyz=[0, 0, 0],
        heading_rad=0.0, xy_valid=False, z_valid=True,
        xy_reset_counter=0, z_reset_counter=0, receive_monotonic_ns=20_100_000_000,
    )
    assert cache.snapshot(20_100_000_000) is None


def test_snapshot_diagnostics_share_admission_and_boundary_predicates():
    cache = ObservationCache()
    _ready(cache)
    for now in [20_019_999_999, 20_020_000_000, 20_500_000_000, 20_500_000_001]:
        diagnostic = {}
        ordinary = cache.snapshot(now)
        observed = cache.snapshot(now, diagnostic_details=diagnostic)
        assert ordinary == observed
        assert diagnostic['snapshot_accepted'] == (ordinary is not None)
        assert diagnostic['gt_receive_age_ns'] == now - 20_000_000_000
    assert diagnostic['snapshot_rejection_reasons'] == ['gt_receive_stale']


def test_snapshot_diagnostics_explain_missing_and_gt_future():
    cache = ObservationCache()
    detail = {}
    assert cache.snapshot(0, diagnostic_details=detail) is None
    assert detail['snapshot_rejection_reasons'] == ['missing_clock', 'missing_gt', 'missing_local']
    _ready(cache)
    cache.observe_gt(10_040_000_001, 'gazebo_world_enu', [0, 0, 0], 20_000_000_000)
    detail = {}
    assert cache.snapshot(20_050_000_000, diagnostic_details=detail) is None
    assert detail['snapshot_rejection_reasons'] == ['gt_clock_future']
    assert detail['clock_minus_gt_ns'] == -40_000_001


def test_fresh_clock_observation_does_not_weaken_incoherent_snapshot_gate():
    cache=ObservationCache()
    _ready(cache)
    cache.observe_clock(10_000_000_000,20_130_000_000)
    cache.observe_gt(10_000_000_000,'gazebo_world_enu',[2,415,1],20_127_516_441)
    details={}
    assert cache.snapshot(20_140_000_000,diagnostic_details=details) is None
    assert details['snapshot_rejection_reasons']==['pair_receive_skew']
    assert details['pair_receive_skew_ns']==107_516_441
    evidence=cache.clock_observation(20_140_000_000)
    assert evidence['fresh'] and evidence['receive_age_ns']==10_000_000
    assert evidence['local_reset_counters']==(4,2)
    assert not cache.clock_observation(20_630_000_001)['fresh']
