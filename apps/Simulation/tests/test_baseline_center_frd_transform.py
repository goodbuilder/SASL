from __future__ import annotations

import math
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import (
    BASELINE_CENTER_FRD_FRAME,
    build_baseline_center_frd,
    camera_optical_pose_from_sdf_pose,
    ground_truth_world_to_baseline,
    nominal_camera_poses,
)


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def test_default_frame_is_orthonormal_right_handed_and_origin_centered() -> None:
    left, right = nominal_camera_poses(25.0, 5.0)
    frame = build_baseline_center_frd(
        left.center_world_enu_m,
        right.center_world_enu_m,
        left.forward_world_enu,
        right.forward_world_enu,
    )
    rotation = frame.rotation_baseline_from_world
    transpose = tuple(tuple(rotation[j][i] for j in range(3)) for i in range(3))
    product = _matmul(rotation, transpose)
    assert frame.frame_id == BASELINE_CENTER_FRD_FRAME
    assert max(abs(product[i][j] - (1.0 if i == j else 0.0)) for i in range(3) for j in range(3)) < 1.0e-12
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    assert abs(determinant - 1.0) < 1.0e-12
    assert max(abs(value) for value in frame.point_from_world(frame.origin_world_enu_m)) < 1.0e-12
    assert frame.point_from_world((0.0, 10.0, 4.0)) == (10.0, 0.0, 1.0)


def test_arbitrary_rig_yaw_and_translation_round_trip() -> None:
    yaw = math.radians(37.0)
    translation = (123.4, -987.6, 42.0)
    baseline_right = (math.sin(yaw), -math.cos(yaw))
    left = camera_optical_pose_from_sdf_pose(
        (translation[0] - 8.0 * baseline_right[0], translation[1] - 8.0 * baseline_right[1], translation[2]),
        (0.0, 0.0, yaw),
    )
    right = camera_optical_pose_from_sdf_pose(
        (translation[0] + 8.0 * baseline_right[0], translation[1] + 8.0 * baseline_right[1], translation[2]),
        (0.0, 0.0, yaw),
    )
    frame = build_baseline_center_frd(
        left.center_world_enu_m,
        right.center_world_enu_m,
        left.forward_world_enu,
        right.forward_world_enu,
    )
    expected = (37.0, -2.5, 11.0)
    world = frame.point_to_world(expected)
    recovered = frame.point_from_world(world)
    assert max(abs(a - b) for a, b in zip(recovered, expected)) < 1.0e-9
    gt = ground_truth_world_to_baseline(world, frame)
    assert max(abs(gt[key] - value) for key, value in zip(("forward_m", "right_m", "down_m"), expected)) < 1.0e-9
