from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import (
    apply_optical_rpy_error,
    camera_optical_pose_from_px4_attitude_wxyz,
    nominal_camera_poses,
    relative_rotation_angle_deg,
    sdf_pose_rpy_from_camera_pose,
)


def _quaternion_wxyz_from_rotation(rotation) -> tuple[float, float, float, float]:
    """Minimal rotation-matrix -> (w, x, y, z) quaternion, test-only helper."""

    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2][1] - rotation[1][2]) / s
        y = (rotation[0][2] - rotation[2][0]) / s
        z = (rotation[1][0] - rotation[0][1]) / s
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        s = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        w = (rotation[2][1] - rotation[1][2]) / s
        x = 0.25 * s
        y = (rotation[0][1] + rotation[1][0]) / s
        z = (rotation[0][2] + rotation[2][0]) / s
    elif rotation[1][1] > rotation[2][2]:
        s = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        w = (rotation[0][2] - rotation[2][0]) / s
        x = (rotation[0][1] + rotation[1][0]) / s
        y = 0.25 * s
        z = (rotation[1][2] + rotation[2][1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
        w = (rotation[1][0] - rotation[0][1]) / s
        x = (rotation[0][2] + rotation[2][0]) / s
        y = (rotation[1][2] + rotation[2][1]) / s
        z = 0.25 * s
    return w, x, y, z


def _rotation_from_body_frd_world_from_optical(optical_pose):
    """R_NED_from_Body_true, i.e. what PX4 would report at zero error, built
    from the SAME physical directions as the true optical pose (fixed
    zero-relative-pose body<->optical extrinsic: F=F, R=R, D=D)."""

    from stereo_ground_sim.coordinate_frames import R_ENU_FROM_NED, PX4_BODY_FRD_FROM_OPTICAL_RDF

    def _mat_mul(left, right):
        return tuple(
            tuple(sum(left[row][k] * right[k][col] for k in range(3)) for col in range(3))
            for row in range(3)
        )

    def _transpose(matrix):
        return tuple(tuple(matrix[row][col] for row in range(3)) for col in range(3))

    r_enu_from_optical = tuple(
        (optical_pose.right_world_enu[i], optical_pose.down_world_enu[i], optical_pose.forward_world_enu[i])
        for i in range(3)
    )
    r_ned_from_enu = _transpose(R_ENU_FROM_NED)
    r_body_from_optical = PX4_BODY_FRD_FROM_OPTICAL_RDF
    r_optical_from_body = _transpose(r_body_from_optical)
    r_ned_from_body = _mat_mul(_mat_mul(r_ned_from_enu, r_enu_from_optical), r_optical_from_body)
    return r_ned_from_body


def _true_optical_pose(side: str, roll_deg: float, pitch_deg: float, yaw_deg: float):
    left_nominal, right_nominal = nominal_camera_poses(25.0, 30.0)
    nominal = left_nominal if side == "left" else right_nominal
    return apply_optical_rpy_error(
        nominal, math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    )


def _px4_quaternion_wxyz_for(optical_pose, error_roll_deg=0.0, error_pitch_deg=0.0, error_yaw_deg=0.0):
    """True body rotation (from optical), with an optional body-local error,
    packaged as a PX4 (w, x, y, z) quaternion -- as if PX4 SITL reported it."""

    reported_optical_pose = apply_optical_rpy_error(
        optical_pose,
        math.radians(error_roll_deg),
        math.radians(error_pitch_deg),
        math.radians(error_yaw_deg),
    )
    r_ned_from_body = _rotation_from_body_frd_world_from_optical(reported_optical_pose)
    return _quaternion_wxyz_from_rotation(r_ned_from_body)


# ---------------------------------------------------------------------------
# 20.1 zero-error round trip
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("side", ("left", "right"))
def test_zero_error_optical_round_trip_matches_to_numeric_precision(side) -> None:
    true_pose = _true_optical_pose(side, 1.0, 2.0, -0.5)
    quaternion_wxyz = _px4_quaternion_wxyz_for(true_pose)

    reconstructed = camera_optical_pose_from_px4_attitude_wxyz(
        true_pose.center_world_enu_m, quaternion_wxyz
    )

    residual_deg = relative_rotation_angle_deg(true_pose, reconstructed)
    assert residual_deg <= 1.0e-9

    r_true = tuple(
        (true_pose.right_world_enu[i], true_pose.down_world_enu[i], true_pose.forward_world_enu[i])
        for i in range(3)
    )
    r_reconstructed = tuple(
        (
            reconstructed.right_world_enu[i],
            reconstructed.down_world_enu[i],
            reconstructed.forward_world_enu[i],
        )
        for i in range(3)
    )
    max_abs_error = max(
        abs(r_true[row][col] - r_reconstructed[row][col]) for row in range(3) for col in range(3)
    )
    assert max_abs_error <= 1.0e-12


def test_zero_rotation_round_trip_is_exact_identity() -> None:
    true_pose = _true_optical_pose("left", 0.0, 0.0, 0.0)
    quaternion_wxyz = _px4_quaternion_wxyz_for(true_pose)

    reconstructed = camera_optical_pose_from_px4_attitude_wxyz(
        true_pose.center_world_enu_m, quaternion_wxyz
    )

    assert relative_rotation_angle_deg(true_pose, reconstructed) <= 1.0e-9


# ---------------------------------------------------------------------------
# 20.2 non-zero mixed rotation, 20.3 left/right independence
# ---------------------------------------------------------------------------


def test_nonzero_mixed_optical_rotation_round_trips() -> None:
    true_pose = _true_optical_pose("left", 1.0, 2.0, -0.5)
    quaternion_wxyz = _px4_quaternion_wxyz_for(true_pose)

    reconstructed = camera_optical_pose_from_px4_attitude_wxyz(
        true_pose.center_world_enu_m, quaternion_wxyz
    )

    assert relative_rotation_angle_deg(true_pose, reconstructed) <= 1.0e-9


def test_left_and_right_attitudes_are_derived_independently() -> None:
    left_true = _true_optical_pose("left", 0.3, 2.0, -0.4)
    right_true = _true_optical_pose("right", -0.2, -1.0, 0.6)

    left_q = _px4_quaternion_wxyz_for(left_true)
    right_q = _px4_quaternion_wxyz_for(right_true)

    left_reconstructed = camera_optical_pose_from_px4_attitude_wxyz(
        left_true.center_world_enu_m, left_q
    )
    right_reconstructed = camera_optical_pose_from_px4_attitude_wxyz(
        right_true.center_world_enu_m, right_q
    )

    assert relative_rotation_angle_deg(left_true, left_reconstructed) <= 1.0e-9
    assert relative_rotation_angle_deg(right_true, right_reconstructed) <= 1.0e-9
    # The two sides' true attitudes actually differ, proving one did not leak
    # into the other's reconstruction.
    assert relative_rotation_angle_deg(left_true, right_true) > 1.0


# ---------------------------------------------------------------------------
# 20.4 fixed body-local error reproduction
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("error_roll_deg", "error_pitch_deg", "error_yaw_deg"),
    (
        (0.0, 0.0, 0.05),
        (0.05, 0.0, 0.0),
        (0.0, 0.05, 0.0),
        (0.02, -0.03, 0.04),
    ),
)
def test_fixed_body_local_error_is_reproduced_within_tolerance(
    error_roll_deg, error_pitch_deg, error_yaw_deg
) -> None:
    true_pose = _true_optical_pose("right", 0.3, 2.0, -0.4)
    reported_quaternion_wxyz = _px4_quaternion_wxyz_for(
        true_pose, error_roll_deg, error_pitch_deg, error_yaw_deg
    )

    reconstructed_reported = camera_optical_pose_from_px4_attitude_wxyz(
        true_pose.center_world_enu_m, reported_quaternion_wxyz
    )

    residual_deg = relative_rotation_angle_deg(true_pose, reconstructed_reported)
    magnitude = math.sqrt(error_roll_deg**2 + error_pitch_deg**2 + error_yaw_deg**2)
    # Small-angle approximation is accurate to well under 1e-4 deg at these
    # magnitudes; the real check is against the directly-injected rotation.
    assert abs(residual_deg - magnitude) < 1.0e-3

    direct_pose = apply_optical_rpy_error(
        true_pose,
        math.radians(error_roll_deg),
        math.radians(error_pitch_deg),
        math.radians(error_yaw_deg),
    )
    exact_residual_deg = relative_rotation_angle_deg(direct_pose, reconstructed_reported)
    assert exact_residual_deg <= 1.0e-8


# ---------------------------------------------------------------------------
# 20.5 quaternion order trap
# ---------------------------------------------------------------------------


def test_quaternion_order_trap_wxyz_vs_xyzw() -> None:
    true_pose = _true_optical_pose("left", 3.0, -4.0, 6.0)
    w, x, y, z = _px4_quaternion_wxyz_for(true_pose)

    correct = camera_optical_pose_from_px4_attitude_wxyz(
        true_pose.center_world_enu_m, (w, x, y, z)
    )
    assert relative_rotation_angle_deg(true_pose, correct) <= 1.0e-9

    # Feeding the same four components as if they were already (x, y, z, w)
    # (i.e. silently treating wxyz input as xyzw) must NOT reconstruct the
    # true pose for a non-trivial rotation.
    swapped = camera_optical_pose_from_px4_attitude_wxyz(
        true_pose.center_world_enu_m, (x, y, z, w)
    )
    assert relative_rotation_angle_deg(true_pose, swapped) > 1.0


# ---------------------------------------------------------------------------
# 20.6 ENU/NED (and RDF/FRD) trap
# ---------------------------------------------------------------------------


def test_enu_ned_axis_swap_trap() -> None:
    true_pose = _true_optical_pose("left", 2.0, -3.0, 5.0)
    quaternion_wxyz = _px4_quaternion_wxyz_for(true_pose)

    correct = camera_optical_pose_from_px4_attitude_wxyz(
        true_pose.center_world_enu_m, quaternion_wxyz
    )
    assert relative_rotation_angle_deg(true_pose, correct) <= 1.0e-9

    # A naive East/North swap plus Down-sign error on the *result* basis
    # would silently pass an all-zero-rotation smoke test but must fail this
    # all-axes-nonzero one.
    swapped_forward = (correct.forward_world_enu[1], correct.forward_world_enu[0], correct.forward_world_enu[2])
    assert swapped_forward != pytest.approx(correct.forward_world_enu, abs=1.0e-6)


# ---------------------------------------------------------------------------
# 20.7 invalid input fails closed
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "quaternion_wxyz",
    (
        (float("nan"), 0.0, 0.0, 0.0),
        (float("inf"), 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
    ),
)
def test_invalid_quaternion_fails_closed(quaternion_wxyz) -> None:
    with pytest.raises(ValueError):
        camera_optical_pose_from_px4_attitude_wxyz((0.0, 0.0, 0.0), quaternion_wxyz)


def test_wrong_length_quaternion_fails_closed() -> None:
    with pytest.raises(ValueError):
        camera_optical_pose_from_px4_attitude_wxyz((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# sdf_pose_rpy_from_camera_pose <-> camera_optical_pose_from_sdf_pose inverse
# ---------------------------------------------------------------------------


def test_sdf_pose_rpy_from_camera_pose_round_trips_through_sdf_pose() -> None:
    from stereo_ground_sim.coordinate_frames import camera_optical_pose_from_sdf_pose

    for rpy_deg in ((0.0, 0.0, 0.0), (1.0, 2.0, -0.5), (10.0, -5.0, 15.0)):
        rpy = tuple(math.radians(value) for value in rpy_deg)
        pose = camera_optical_pose_from_sdf_pose((1.0, 2.0, 3.0), rpy)
        recovered = sdf_pose_rpy_from_camera_pose(pose)
        assert recovered == pytest.approx(rpy, abs=1.0e-9)
