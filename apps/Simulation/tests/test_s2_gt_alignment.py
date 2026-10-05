from __future__ import annotations

import pytest

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import GroundTruthPositionBuffer


Q = (0.0, 0.0, 0.0, 1.0)


def test_exact_and_interpolated_position_alignment() -> None:
    buffer = GroundTruthPositionBuffer()
    buffer.add(100, (1.0, 2.0, 3.0), Q)
    buffer.add(200, (3.0, 6.0, 9.0), Q)
    exact = buffer.align(100, 0)
    interpolated = buffer.align(150, 0)
    assert exact.gt_alignment_mode == "exact"
    assert exact.position_world_enu_m == (1.0, 2.0, 3.0)
    assert interpolated.gt_alignment_mode == "interpolated"
    assert interpolated.position_world_enu_m == pytest.approx((2.0, 4.0, 6.0), abs=1e-12)
    assert interpolated.gt_interpolation_alpha == pytest.approx(0.5)
    assert interpolated.orientation_world_from_reference_xyzw is None


def test_nearest_modes_are_explicit_flush_fallbacks() -> None:
    buffer = GroundTruthPositionBuffer()
    buffer.add(100, (1.0, 0.0, 0.0), Q)
    before = buffer.align(150, 0, allow_nearest=True)
    after = buffer.align(50, 0, allow_nearest=True)
    assert before.gt_alignment_mode == "nearest_before"
    assert before.gt_nearest_delta_ns == -50
    assert after.gt_alignment_mode == "nearest_after"
    assert after.gt_nearest_delta_ns == 50
    assert buffer.align(150, 0).gt_alignment_mode == "unavailable"
