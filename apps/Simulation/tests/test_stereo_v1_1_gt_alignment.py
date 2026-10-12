"""Independent 50 Hz GT position/rotation and failure-boundary oracles."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.gt_alignment import GroundTruthSeries, shortest_path_slerp_xyzw


def _row(stamp, north, quaternion, epoch=0):
    return {
        "simulation_epoch": epoch,
        "source_timestamp_ns": stamp,
        "position_world_enu_m": [2.0, north, 100.0],
        "orientation_world_from_reference_xyzw": quaternion,
    }


def test_position_linear_and_quaternion_shortest_path_at_image_time():
    series = GroundTruthSeries([
        _row(1_000_000_000, 400.0, [0, 0, 0, 1]),
        _row(1_020_000_000, 398.0, [0, 0, 1, 0]),
    ])
    aligned = series.align(0, 1_010_000_000)
    assert aligned.status == "INTERPOLATED"
    assert aligned.position_world_enu_m == pytest.approx((2.0, 399.0, 100.0))
    assert aligned.orientation_world_from_reference_xyzw == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    )
    assert aligned.bracketing_gap_ns == 20_000_000
    assert series.align(0, 1_000_000_000).status == "EXACT"


def test_antipodal_quaternion_gap_and_extrapolation_are_not_silent():
    assert shortest_path_slerp_xyzw([0, 0, 0, 1], [0, 0, 0, -1], 0.5) == pytest.approx((0, 0, 0, 1))
    series = GroundTruthSeries([
        _row(1_000_000_000, 400.0, [0, 0, 0, 1]),
        _row(1_040_000_000, 396.0, [0, 0, 0, -1]),
    ])
    assert series.align(0, 1_020_000_000).status == "GAP_GE_40MS"
    assert series.align(0, 999_999_999).status == "EXTRAPOLATION"
    assert series.align(1, 1_020_000_000).status == "UNAVAILABLE"
    with pytest.raises(ValueError, match="duplicate"):
        GroundTruthSeries([_row(1, 1, [0, 0, 0, 1]), _row(1, 2, [0, 0, 0, 1])])
