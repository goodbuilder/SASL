from __future__ import annotations

import pytest

from Workstation.workstation_core.baseline_yaw import (
    baseline_azimuth_deg,
    quaternion_from_roll_pitch_yaw_deg,
    select_baseline_yaw,
)
from Workstation.workstation_core.pixhawk_attitude import quaternion_to_euler_deg


def test_baseline_yaw_builds_the_two_perpendicular_candidates() -> None:
    assert baseline_azimuth_deg(1.0, 0.0) == pytest.approx(0.0)
    selection = select_baseline_yaw(
        north_m=1.0,
        east_m=0.0,
        p1_yaw_deg=270.0,
        p2_yaw_deg=268.0,
    )

    assert selection.ready
    assert selection.candidate_minus90_deg == pytest.approx(270.0)
    assert selection.candidate_plus90_deg == pytest.approx(90.0)
    assert selection.selected_yaw_deg == pytest.approx(270.0)
    assert selection.selected_offset_deg == pytest.approx(-90.0)
    assert selection.consensus is True


def test_baseline_yaw_reverse_flips_auto_selection() -> None:
    selection = select_baseline_yaw(
        north_m=1.0,
        east_m=0.0,
        p1_yaw_deg=270.0,
        p2_yaw_deg=270.0,
        reverse=True,
    )

    assert selection.ready
    assert selection.mode == "auto_reverse"
    assert selection.selected_yaw_deg == pytest.approx(90.0)
    assert selection.selected_offset_deg == pytest.approx(90.0)


def test_baseline_yaw_persistent_reverse_can_force_branch_without_yaw_cache() -> None:
    auto = select_baseline_yaw(north_m=0.0, east_m=1.0)
    forced = select_baseline_yaw(north_m=0.0, east_m=1.0, reverse=True)

    assert not auto.ready
    assert forced.ready
    assert forced.mode == "reverse_forced"
    assert forced.selected_offset_deg == pytest.approx(90.0)


def test_baseline_yaw_quaternion_preserves_rp_and_uses_selected_yaw() -> None:
    quaternion = quaternion_from_roll_pitch_yaw_deg(12.0, -7.0, 90.0)
    euler = quaternion_to_euler_deg(quaternion)

    assert euler.roll_deg == pytest.approx(12.0, abs=1.0e-9)
    assert euler.pitch_deg == pytest.approx(-7.0, abs=1.0e-9)
    assert euler.yaw_deg == pytest.approx(90.0, abs=1.0e-9)
