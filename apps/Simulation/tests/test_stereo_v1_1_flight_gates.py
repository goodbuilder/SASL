from __future__ import annotations

import sys
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.flight_gates import (
    OriginObservation,
    SettleSample,
    fit_observed_origin,
    science_start_ready,
)
from campaigns.stereo_v1_1.trajectory_contract import LocalNed, WorldEnu


def test_origin_gate_uses_live_pairs_and_rejects_resets():
    samples = [
        OriginObservation(
            gt_world_enu_m=WorldEnu(0.0, 415.0, 1.0),
            px4_local_ned_m=LocalNed(0.02, -0.01, -0.03),
            gt_receive_monotonic_ns=10_000_000_000 + i * 50_000_000,
            px4_receive_monotonic_ns=10_020_000_000 + i * 50_000_000,
            simulation_epoch=0,
            xy_reset_counter=4,
            z_reset_counter=2,
        )
        for i in range(20)
    ]
    assert fit_observed_origin(samples) == WorldEnu(0.01, 414.98, 0.97)
    changed = list(samples)
    changed[10] = OriginObservation(
        **{**changed[10].__dict__, "z_reset_counter": 3}
    )
    with pytest.raises(ValueError, match="reset"):
        fit_observed_origin(changed)
    stale = list(samples)
    stale[0] = OriginObservation(
        **{**stale[0].__dict__, "px4_receive_monotonic_ns": 11_000_000_000}
    )
    with pytest.raises(ValueError, match="insufficient"):
        fit_observed_origin(stale)


def test_five_second_settle_rejects_gap_threshold_and_epoch_change():
    samples = [
        SettleSample(i * 100_000_000, 0.1, 0.2, 1.0, 0, 4, 2)
        for i in range(51)
    ]
    assert science_start_ready(samples)
    shifted = [
        SettleSample(sample.simulation_time_ns + 30_000_000, 0.1, 0.2, 1.0, 0, 4, 2)
        for sample in samples
    ]
    assert science_start_ready(shifted)
    assert not science_start_ready(samples[:-1])
    bad_position = list(samples)
    bad_position[25] = SettleSample(2_500_000_000, 0.1, 0.5, 1.0, 0, 4, 2)
    assert not science_start_ready(bad_position)
    reset = list(samples)
    reset[-1] = SettleSample(5_000_000_000, 0.1, 0.2, 1.0, 0, 4, 3)
    assert not science_start_ready(reset)
    gap = samples[:20] + samples[23:]
    assert not science_start_ready(gap)
