from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import (  # noqa: E402
    _ned_yaw_correction_rotation,
    camera_optical_pose_from_px4_attitude_wxyz,
    nominal_camera_poses,
    relative_rotation_angle_deg,
    validate_rotation_matrix,
)
from stereo_ground_sim.gps_baseline import (  # noqa: E402
    RawGpsFix,
    gps_baseline_bearing_rad,
    true_north_yaw_correction_rad,
)

WGS84_A_M = 6_378_137.0


def _pure_yaw_quaternion_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2.0
    return math.cos(half), 0.0, 0.0, math.sin(half)


def _gps_fixes_for_enu_baseline(baseline_enu_m, *, lat0_deg=47.398, lon0_deg=8.546):
    """Build (left, right) RawGpsFix pairs whose true-north geodetic baseline
    matches ``baseline_enu_m`` (E, N, U) to a small-baseline flat-Earth
    approximation -- good enough for a ~25 m separation test fixture."""

    east, north, _up = baseline_enu_m
    dlat_deg = math.degrees(north / WGS84_A_M)
    dlon_deg = math.degrees(east / (WGS84_A_M * math.cos(math.radians(lat0_deg))))
    left_fix = RawGpsFix(
        timestamp_us=0, latitude_deg=lat0_deg, longitude_deg=lon0_deg,
        altitude_msl_m=0.0, eph_m=0.5, epv_m=0.5,
    )
    right_fix = RawGpsFix(
        timestamp_us=0, latitude_deg=lat0_deg + dlat_deg, longitude_deg=lon0_deg + dlon_deg,
        altitude_msl_m=0.0, eph_m=0.5, epv_m=0.5,
    )
    return left_fix, right_fix


# ---------------------------------------------------------------------------
# NED yaw correction rotation (unchanged from Phase 1, still the application
# mechanism for whatever yaw-correction radians value is computed)
# ---------------------------------------------------------------------------


def test_ned_yaw_correction_rotation_is_a_valid_rotation_matrix() -> None:
    rotation = _ned_yaw_correction_rotation(math.radians(12.5))
    validate_rotation_matrix(rotation)  # raises on failure


@pytest.mark.parametrize("yaw_deg", (0.0, 5.0, -30.0, 179.0))
def test_ned_yaw_correction_rotation_rotates_north_toward_east_by_exactly_yaw(
    yaw_deg,
) -> None:
    rotation = _ned_yaw_correction_rotation(math.radians(yaw_deg))
    north = (1.0, 0.0, 0.0)
    rotated = tuple(sum(rotation[row][col] * north[col] for col in range(3)) for row in range(3))
    bearing_deg = math.degrees(math.atan2(rotated[1], rotated[0]))
    assert abs(bearing_deg - yaw_deg) < 1.0e-9


# ---------------------------------------------------------------------------
# gps_baseline_bearing_rad
# ---------------------------------------------------------------------------


def test_gps_baseline_bearing_matches_known_geometry() -> None:
    nominal_left, nominal_right = nominal_camera_poses(baseline_m=25.0, camera_height_m=0.1)
    nominal_baseline = tuple(
        nominal_right.center_world_enu_m[index] - nominal_left.center_world_enu_m[index]
        for index in range(3)
    )
    left_fix, right_fix = _gps_fixes_for_enu_baseline(nominal_baseline)

    bearing_rad = gps_baseline_bearing_rad(left_fix, right_fix)

    assert bearing_rad is not None
    assert abs(math.degrees(bearing_rad) - 90.0) < 1.0e-2  # pure East baseline


def test_gps_baseline_bearing_returns_none_for_degenerate_baseline() -> None:
    left_fix, _right_fix = _gps_fixes_for_enu_baseline((25.0, 0.0, 0.0))

    assert gps_baseline_bearing_rad(left_fix, left_fix) is None


# ---------------------------------------------------------------------------
# true_north_yaw_correction_rad
# ---------------------------------------------------------------------------


def test_true_north_yaw_correction_is_zero_when_forward_already_matches_expected() -> None:
    bearing_rad = math.radians(90.0)
    offset_rad = math.radians(-90.0)
    expected_heading_rad = bearing_rad + offset_rad  # 0 deg -> pure North
    matching_forward = (math.sin(expected_heading_rad), math.cos(expected_heading_rad), 0.0)

    correction_rad = true_north_yaw_correction_rad(bearing_rad, offset_rad, matching_forward)

    assert correction_rad is not None
    assert abs(math.degrees(correction_rad)) < 1.0e-6


@pytest.mark.parametrize("injected_live_bias_deg", (5.0, -12.0, 179.0, -179.0))
def test_true_north_yaw_correction_recovers_injected_live_heading_bias(
    injected_live_bias_deg,
) -> None:
    bearing_rad = math.radians(90.0)
    offset_rad = math.radians(-90.0)
    expected_heading_rad = bearing_rad + offset_rad
    biased_heading_rad = expected_heading_rad + math.radians(injected_live_bias_deg)
    biased_forward = (math.sin(biased_heading_rad), math.cos(biased_heading_rad), 0.0)

    correction_rad = true_north_yaw_correction_rad(bearing_rad, offset_rad, biased_forward)

    assert correction_rad is not None
    # expected - live = -injected_bias
    assert abs(math.degrees(correction_rad) - (-injected_live_bias_deg)) < 1.0e-6


def test_true_north_yaw_correction_returns_none_for_near_vertical_forward() -> None:
    bearing_rad = math.radians(90.0)
    offset_rad = math.radians(-90.0)

    assert true_north_yaw_correction_rad(bearing_rad, offset_rad, (0.0, 0.0, 1.0)) is None


def test_true_north_yaw_correction_applied_back_cancels_the_injected_bias() -> None:
    """End-to-end: estimate a correction from a biased live PX4 attitude pose
    and confirm applying it via camera_optical_pose_from_px4_attitude_wxyz's
    declination_correction_rad recovers the unbiased pose."""

    nominal_left, _nominal_right = nominal_camera_poses(baseline_m=25.0, camera_height_m=0.1)
    bearing_rad = math.radians(90.0)
    offset_rad = math.radians(-90.0)

    unbiased_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m, _pure_yaw_quaternion_wxyz(0.0)
    )
    injected_bias_deg = 6.5
    biased_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m, _pure_yaw_quaternion_wxyz(injected_bias_deg)
    )

    correction_rad = true_north_yaw_correction_rad(
        bearing_rad, offset_rad, biased_pose.forward_world_enu
    )
    assert correction_rad is not None
    corrected_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m,
        _pure_yaw_quaternion_wxyz(injected_bias_deg),
        declination_correction_rad=correction_rad,
    )

    residual_deg = relative_rotation_angle_deg(corrected_pose, unbiased_pose)
    assert residual_deg < 1.0e-2
