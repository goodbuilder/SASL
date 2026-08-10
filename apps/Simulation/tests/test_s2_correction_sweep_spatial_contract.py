from __future__ import annotations

from decimal import Decimal
import math
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import build_baseline_center_frd, nominal_camera_poses
from stereo_ground_sim.correction_sweep import (
    ATTITUDE_AXES,
    CameraContract,
    build_sweep_plan,
    compute_spatial_validation,
    fov_preflight,
    spatial_frd,
    spatial_from_frd,
    wrapped_angle_error_deg,
)


ZERO = {axis: Decimal(0) for axis in ATTITUDE_AXES}


def test_spatial_cardinal_axes_follow_frd_contract() -> None:
    assert spatial_frd(500, 0, 0) == pytest.approx((500, 0, 0), abs=1e-12)
    assert spatial_frd(500, 90, 0) == pytest.approx((0, 500, 0), abs=1e-12)
    assert spatial_frd(500, 0, 90) == pytest.approx((0, 0, -500), abs=1e-12)


def test_representative_spatial_value_and_inverse() -> None:
    point = spatial_frd(500, 1, 0.5)
    expected = (
        500 * math.cos(math.radians(0.5)) * math.cos(math.radians(1)),
        500 * math.cos(math.radians(0.5)) * math.sin(math.radians(1)),
        -500 * math.sin(math.radians(0.5)),
    )
    assert point == pytest.approx(expected, abs=1e-12)
    assert spatial_from_frd(point) == pytest.approx((500, 1, 0.5), abs=1e-12)


def test_frd_world_enu_round_trip_uses_nominal_baseline_frame() -> None:
    left, right = nominal_camera_poses(25.0, 5.0)
    frame = build_baseline_center_frd(
        left.center_world_enu_m,
        right.center_world_enu_m,
        left.forward_world_enu,
        right.forward_world_enu,
    )
    point = (499.8, 8.7, -4.3)
    world = frame.point_to_world(point)
    assert world != point
    assert frame.point_from_world(world) == pytest.approx(point, abs=1e-12)


def test_actual_pose_fov_preflight_valid_and_finite() -> None:
    result = fov_preflight(
        contract=CameraContract(), attitude=ZERO, point_frd=spatial_frd(500, 1, 0.5)
    )
    assert result["valid"] is True
    assert all(math.isfinite(value) for value in result["left_pixel_px"])
    assert all(math.isfinite(value) for value in result["right_pixel_px"])
    assert result["left_forward_depth_m"] > 0
    assert result["right_forward_depth_m"] > 0


@pytest.mark.parametrize(
    ("point", "reason"),
    (
        ((-1, 0, 0), "behind_or_clip_near"),
        ((500, 100, 0), "pixel_out_of_bounds"),
        ((2600, 0, 0), "clip_far"),
    ),
)
def test_fov_preflight_rejects_invalid_geometry(point, reason: str) -> None:
    result = fov_preflight(contract=CameraContract(), attitude=ZERO, point_frd=point)
    assert result["valid"] is False
    assert any(reason in item for item in result["reasons"])


def test_fail_and_skip_invalid_spatial_policy() -> None:
    kwargs = dict(
        sweep_id="s",
        commit_sha="a" * 40,
        branch="sim/test",
        repository_name="repo",
        attitude_mode="ofat",
        axis_ranges=(),
        range_values=(Decimal("500"),),
        bearing_values=(Decimal("30"),),
        elevation_values=(Decimal("0"),),
        repetitions=1,
        seed=1,
        case_order="canonical",
        max_cases=2,
        contract=CameraContract(),
    )
    with pytest.raises(ValueError, match="invalid spatial"):
        build_sweep_plan(**kwargs, invalid_spatial_policy="fail")
    plan, cases = build_sweep_plan(**kwargs, invalid_spatial_policy="skip")
    assert cases == []
    assert plan["total_skipped_by_fov"] == 1
    assert plan["skipped_cases"][0]["reasons"]


def test_requested_range_must_be_positive_and_below_clip_far() -> None:
    base = dict(
        sweep_id="s",
        commit_sha="a" * 40,
        branch="sim/test",
        repository_name="repo",
        attitude_mode="ofat",
        axis_ranges=(),
        bearing_values=(Decimal("0"),),
        elevation_values=(Decimal("0"),),
        repetitions=1,
        seed=1,
        case_order="canonical",
        invalid_spatial_policy="fail",
        max_cases=2,
        contract=CameraContract(),
    )
    with pytest.raises(ValueError, match="positive"):
        build_sweep_plan(**base, range_values=(Decimal("0"),))
    with pytest.raises(ValueError, match="less than camera_clip_far"):
        build_sweep_plan(**base, range_values=(Decimal("2500"),))


def test_attitude_does_not_move_canonical_baseline_frame() -> None:
    changed = dict(ZERO)
    changed["common_optical_roll_error_deg"] = Decimal("0.1")
    zero = fov_preflight(contract=CameraContract(), attitude=ZERO, point_frd=(500, 1, 1))
    nonzero = fov_preflight(contract=CameraContract(), attitude=changed, point_frd=(500, 1, 1))
    assert zero["baseline_origin_world_enu_m"] == nonzero["baseline_origin_world_enu_m"]
    assert zero["baseline_rotation_baseline_from_world"] == nonzero["baseline_rotation_baseline_from_world"]


def _spatial_records(points):
    return [
        {
            "record_sequence": index,
            "sample_eligible": True,
            "gt_forward_m": point[0],
            "gt_right_m": point[1],
            "gt_down_m": point[2],
        }
        for index, point in enumerate(points, start=1)
    ]


def test_per_sample_spatial_validation_uses_max_not_only_median() -> None:
    exact = spatial_frd(500.0, 1.0, 0.5)
    moved = spatial_frd(500.001, 1.0, 0.5)
    result = compute_spatial_validation(
        _spatial_records((exact, exact, moved)),
        warmup_count=0,
        analysis_count=3,
        requested_range_m=500.0,
        requested_bearing_deg=1.0,
        requested_elevation_deg=0.5,
        range_tolerance_m=0.0001,
        angle_tolerance_deg=0.00001,
    )
    assert result["actual_range_m_median"] == pytest.approx(500.0, abs=1e-12)
    assert result["spatial_range_error_abs_max_m"] > 0.0001
    assert result["spatial_validation_pass"] is False
    assert result["spatial_validation_sample_count"] == 3


def test_spatial_validation_exact_point_wrap_and_strict_boundary() -> None:
    point = spatial_frd(500.0, 179.999999, -0.5)
    result = compute_spatial_validation(
        _spatial_records((point,)),
        warmup_count=0,
        analysis_count=1,
        requested_range_m=500.0,
        requested_bearing_deg=179.999999,
        requested_elevation_deg=-0.5,
        range_tolerance_m=0.0001,
        angle_tolerance_deg=0.00001,
    )
    assert result["spatial_validation_pass"] is True
    assert abs(wrapped_angle_error_deg(-179.999999, 179.999999)) < 0.00001


@pytest.mark.parametrize("bad", (None, float("nan"), float("inf")))
def test_spatial_validation_rejects_null_and_nonfinite_gt(bad) -> None:
    point = list(spatial_frd(500.0, 0.0, 0.0))
    point[2] = bad
    with pytest.raises(ValueError, match="gt_down_m"):
        compute_spatial_validation(
            _spatial_records((point,)),
            warmup_count=0,
            analysis_count=1,
            requested_range_m=500.0,
            requested_bearing_deg=0.0,
            requested_elevation_deg=0.0,
            range_tolerance_m=0.0001,
            angle_tolerance_deg=0.00001,
        )


def test_spatial_tolerances_change_plan_identity() -> None:
    kwargs = dict(
        sweep_id="s",
        commit_sha="a" * 40,
        branch="sim/test",
        repository_name="repo",
        attitude_mode="ofat",
        axis_ranges=(),
        range_values=(Decimal("500"),),
        bearing_values=(Decimal("0"),),
        elevation_values=(Decimal("0"),),
        repetitions=1,
        seed=1,
        case_order="canonical",
        invalid_spatial_policy="fail",
        max_cases=2,
        contract=CameraContract(),
    )
    first, cases = build_sweep_plan(**kwargs)
    second, second_cases = build_sweep_plan(
        **kwargs, spatial_range_tolerance_m=0.0002
    )
    assert first["plan_hash"] != second["plan_hash"]
    assert cases[0]["case_hash"] != second_cases[0]["case_hash"]
    assert cases[0]["payload"]["target_static_for_research"] is True
