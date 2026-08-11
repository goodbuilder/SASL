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
    declination_from_gps_baseline_rad,
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
# NED yaw correction rotation
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
# declination_from_gps_baseline_rad
# ---------------------------------------------------------------------------


def test_declination_from_gps_baseline_is_near_zero_with_no_injected_bias() -> None:
    nominal_left, nominal_right = nominal_camera_poses(baseline_m=25.0, camera_height_m=0.1)
    nominal_baseline = tuple(
        nominal_right.center_world_enu_m[index] - nominal_left.center_world_enu_m[index]
        for index in range(3)
    )
    left_fix, right_fix = _gps_fixes_for_enu_baseline(nominal_baseline)
    unbiased_attitude_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m, _pure_yaw_quaternion_wxyz(0.0)
    )

    declination_rad = declination_from_gps_baseline_rad(
        left_fix, right_fix, nominal_baseline, nominal_left, unbiased_attitude_pose
    )

    assert declination_rad is not None
    assert abs(math.degrees(declination_rad)) < 1.0e-3


@pytest.mark.parametrize("injected_yaw_bias_deg", (3.7, -2.1, 10.0, -25.0))
def test_declination_from_gps_baseline_recovers_injected_yaw_bias(
    injected_yaw_bias_deg,
) -> None:
    nominal_left, nominal_right = nominal_camera_poses(baseline_m=25.0, camera_height_m=0.1)
    nominal_baseline = tuple(
        nominal_right.center_world_enu_m[index] - nominal_left.center_world_enu_m[index]
        for index in range(3)
    )
    left_fix, right_fix = _gps_fixes_for_enu_baseline(nominal_baseline)
    biased_attitude_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m,
        _pure_yaw_quaternion_wxyz(injected_yaw_bias_deg),
    )

    declination_rad = declination_from_gps_baseline_rad(
        left_fix, right_fix, nominal_baseline, nominal_left, biased_attitude_pose
    )

    assert declination_rad is not None
    # true = magnetic + declination, so declination should recover -bias.
    assert abs(math.degrees(declination_rad) - (-injected_yaw_bias_deg)) < 1.0e-2


def test_declination_correction_applied_back_cancels_the_injected_bias() -> None:
    nominal_left, nominal_right = nominal_camera_poses(baseline_m=25.0, camera_height_m=0.1)
    nominal_baseline = tuple(
        nominal_right.center_world_enu_m[index] - nominal_left.center_world_enu_m[index]
        for index in range(3)
    )
    left_fix, right_fix = _gps_fixes_for_enu_baseline(nominal_baseline)
    injected_yaw_bias_deg = 7.25
    biased_quaternion = _pure_yaw_quaternion_wxyz(injected_yaw_bias_deg)
    biased_attitude_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m, biased_quaternion
    )
    unbiased_attitude_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m, _pure_yaw_quaternion_wxyz(0.0)
    )

    declination_rad = declination_from_gps_baseline_rad(
        left_fix, right_fix, nominal_baseline, nominal_left, biased_attitude_pose
    )
    corrected_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m,
        biased_quaternion,
        declination_correction_rad=declination_rad,
    )

    residual_deg = relative_rotation_angle_deg(corrected_pose, unbiased_attitude_pose)
    assert residual_deg < 1.0e-2


def test_declination_from_gps_baseline_returns_none_on_degenerate_baseline() -> None:
    nominal_left, nominal_right = nominal_camera_poses(baseline_m=25.0, camera_height_m=0.1)
    left_fix, right_fix = _gps_fixes_for_enu_baseline((25.0, 0.0, 0.0))
    unbiased_attitude_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m, _pure_yaw_quaternion_wxyz(0.0)
    )

    declination_rad = declination_from_gps_baseline_rad(
        left_fix, right_fix, (0.0, 0.0, 0.0), nominal_left, unbiased_attitude_pose
    )

    assert declination_rad is None


def test_declination_estimate_requires_distinct_design_and_live_poses_a_regression_guard() -> None:
    """Regression guard for a real bug found during implementation: passing
    the SAME pose as both the design projection basis and the live rotation
    makes the round-trip a mathematical no-op, silently returning ~0
    regardless of the true heading error."""

    nominal_left, nominal_right = nominal_camera_poses(baseline_m=25.0, camera_height_m=0.1)
    nominal_baseline = tuple(
        nominal_right.center_world_enu_m[index] - nominal_left.center_world_enu_m[index]
        for index in range(3)
    )
    left_fix, right_fix = _gps_fixes_for_enu_baseline(nominal_baseline)
    biased_attitude_pose = camera_optical_pose_from_px4_attitude_wxyz(
        nominal_left.center_world_enu_m, _pure_yaw_quaternion_wxyz(9.0)
    )

    same_pose_declination_rad = declination_from_gps_baseline_rad(
        left_fix, right_fix, nominal_baseline, biased_attitude_pose, biased_attitude_pose
    )
    distinct_pose_declination_rad = declination_from_gps_baseline_rad(
        left_fix, right_fix, nominal_baseline, nominal_left, biased_attitude_pose
    )

    assert abs(math.degrees(same_pose_declination_rad)) < 1.0e-3
    assert abs(math.degrees(distinct_pose_declination_rad)) > 1.0
