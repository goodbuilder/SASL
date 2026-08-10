from __future__ import annotations

from math import cos, radians, sin

import pytest

from workstation_core.physical_alignment import (
    PhysicalAlignmentSample,
    angle_error_deg,
    evaluate_physical_alignment,
    make_physical_alignment_sample,
)


def _yaw(degrees: float) -> tuple[float, float, float, float]:
    half = radians(degrees) / 2.0
    return cos(half), 0.0, 0.0, sin(half)


def test_sample_keeps_ned_and_projects_to_p1_body_frd() -> None:
    sample = make_physical_alignment_sample(
        timestamp_s=10.0,
        baseline_ned_m=(1.0, 2.0, 3.0),
        p1_body_to_world_q=_yaw(90.0),
        p2_body_to_world_q=_yaw(95.0),
        left_fix_type=5,
        right_fix_type=3,
    )

    assert sample.baseline_ned_m == pytest.approx((1.0, 2.0, 3.0))
    assert sample.baseline_frd_m == pytest.approx((2.0, -1.0, 3.0))
    assert sample.relative_rpy_deg == pytest.approx((0.0, 0.0, 5.0))
    assert sample.minimum_fix_type == 3
    assert sample.both_rtk_fixed is False


def test_alignment_accepts_numeric_fix_type_without_assuming_rtk_fixed() -> None:
    sample = PhysicalAlignmentSample(
        timestamp_s=1.0,
        baseline_ned_m=(1.0, 2.0, 3.0),
        baseline_frd_m=(1.0, 2.0, 3.0),
        relative_rpy_deg=(0.1, -0.1, 0.2),
        left_fix_type=5,
        right_fix_type=3,
    )

    quality = evaluate_physical_alignment([sample], minimum_fix_type=3)

    assert quality is not None
    assert quality.state == "EVIDENCE"
    assert quality.fixed_ratio == pytest.approx(0.0)
    assert quality.minimum_fix_type_observed == 3
    assert quality.reasons == ("baseline_reference_missing",)


def test_alignment_pass_includes_roll_pitch_and_yaw() -> None:
    sample = PhysicalAlignmentSample(
        timestamp_s=1.0,
        baseline_ned_m=(1.0, 2.0, 3.0),
        baseline_frd_m=(1.0, 2.0, 3.0),
        relative_rpy_deg=(0.5, -0.5, 0.5),
        left_fix_type=6,
        right_fix_type=6,
    )

    quality = evaluate_physical_alignment(
        [sample],
        target_baseline_frd_m=(1.0, 2.0, 3.0),
        rpy_tolerance_deg=1.0,
        minimum_fix_type=3,
        minimum_fixed_ratio=1.0,
    )

    assert quality is not None
    assert quality.state == "PASS"
    assert quality.relative_rpy_error_deg == pytest.approx((0.5, -0.5, 0.5))


def test_angle_error_uses_shortest_path() -> None:
    assert angle_error_deg(179.0, -179.0) == pytest.approx(2.0)


def test_fixed_policy_is_explicit_when_requested() -> None:
    sample = PhysicalAlignmentSample(
        timestamp_s=1.0,
        baseline_ned_m=(1.0, 2.0, 3.0),
        baseline_frd_m=(1.0, 2.0, 3.0),
        relative_rpy_deg=(0.0, 0.0, 0.0),
        left_fix_type=5,
        right_fix_type=6,
    )

    quality = evaluate_physical_alignment(
        [sample],
        target_baseline_frd_m=(1.0, 2.0, 3.0),
        minimum_fix_type=6,
        minimum_fixed_ratio=1.0,
    )

    assert quality is not None
    assert quality.state == "FAIL"
    assert "fix_type_below_minimum" in quality.reasons
    assert "fixed_ratio_below_minimum" in quality.reasons
