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
    fov_preflight,
    spatial_frd,
    spatial_from_frd,
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
