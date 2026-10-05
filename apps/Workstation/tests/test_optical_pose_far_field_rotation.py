"""Far-field relative rotation and unlimited inlier retention for Mountain/Cloud.

The stored suggestion used to keep at most 12 spread correspondences and
discarded the RANSAC geometry. It now keeps every inlier by default and fits a
rotation-only (homography-at-infinity) model over all of them, reporting the
right camera's roll/pitch/yaw in the left camera's virtual FRD frame.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest

from Workstation.workstation_core.optical_pose_auto_select import (
    _fit_far_field_relative_rotation,
    detect_mountain,
)

_FRD_FROM_RDF = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def _rotation_left_from_right_rdf(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """R_left_from_right in optical RDF from the right camera's FRD 3-2-1 rpy."""

    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    frd = rz @ ry @ rx
    return _FRD_FROM_RDF.T @ frd @ _FRD_FROM_RDF


def test_fit_recovers_known_rpy_from_bearings() -> None:
    rng = np.random.default_rng(3)
    focal = 8000.0
    left_norm = rng.uniform(-0.08, 0.08, (300, 2))
    truth = (0.3, -0.7, -2.1)  # roll, pitch, yaw of the right camera in left FRD
    rotation_right_from_left = _rotation_left_from_right_rdf(*truth).T
    homogeneous = np.concatenate([left_norm, np.ones((300, 1))], axis=1)
    rotated = (rotation_right_from_left @ homogeneous.T).T
    right_norm = rotated[:, :2] / rotated[:, 2:3]
    right_norm += rng.normal(0.0, 0.3 / focal, right_norm.shape)  # 0.3 px noise
    # a handful of gross outliers must be trimmed
    right_norm[:6] += 40.0 / focal
    result = _fit_far_field_relative_rotation(
        left_norm, right_norm, left_norm * focal, right_norm * focal, focal_px=focal, np=np
    )
    assert result["available"] is True
    assert result["fit_inlier_count"] >= 280
    assert result["roll_right_minus_left_deg"] == pytest.approx(truth[0], abs=0.02)
    assert result["pitch_right_minus_left_deg"] == pytest.approx(truth[1], abs=0.02)
    assert result["yaw_right_minus_left_deg"] == pytest.approx(truth[2], abs=0.02)
    assert result["fit_residual_px"] < 1.0
    # toe-in (negative yaw) shifts far features to +u in the right image
    assert result["far_field_du_median_px"] > 0


def _textured_frame(rng: np.random.Generator) -> np.ndarray:
    base = np.repeat(np.linspace(200.0, 120.0, 480)[:, None], 640, axis=1)
    blobs = np.zeros((480, 640), np.float32)
    for _ in range(60):
        center = (int(rng.integers(0, 640)), int(rng.integers(0, 480)))
        cv2.circle(blobs, center, int(rng.integers(10, 60)), float(rng.uniform(-70, 70)), -1)
    blobs = cv2.GaussianBlur(blobs, (0, 0), 6)
    detail = cv2.GaussianBlur(rng.normal(0.0, 30.0, (480, 640)).astype(np.float32), (0, 0), 2)
    return np.clip(base + blobs + detail, 0, 255).astype(np.uint8)


def test_detect_mountain_keeps_all_inliers_and_reports_rotation() -> None:
    rng = np.random.default_rng(11)
    left = _textured_frame(rng)
    focal = 900.0
    camera = np.array([[focal, 0.0, 320.0], [0.0, focal, 240.0], [0.0, 0.0, 1.0]])
    truth_yaw = -1.2  # right camera toed in by 1.2 deg
    rotation_right_from_left = _rotation_left_from_right_rdf(0.0, 0.0, truth_yaw).T
    homography = camera @ rotation_right_from_left @ np.linalg.inv(camera)
    right = cv2.warpPerspective(left, homography, (640, 480), flags=cv2.INTER_LINEAR)

    result = detect_mountain(
        left,
        right,
        left_camera_matrix=camera,
        right_camera_matrix=camera,
        far_field_only=True,
    )
    assert result.inlier_count >= 20
    # default max_points=0 keeps every inlier on both sides
    assert len(result.left.points) == result.inlier_count
    assert len(result.right.points) == result.inlier_count
    assert len(result.left.point_confidences) == result.inlier_count

    rotation = result.metadata["relative_rotation"]
    assert rotation["available"] is True
    assert rotation["yaw_right_minus_left_deg"] == pytest.approx(truth_yaw, abs=0.1)
    assert abs(rotation["pitch_right_minus_left_deg"]) < 0.1
    assert abs(rotation["roll_right_minus_left_deg"]) < 0.1
    assert rotation["far_field_du_median_px"] > 0
    assert result.left.metadata["relative_rotation"] == rotation

    capped = detect_mountain(
        left, right, left_camera_matrix=camera, right_camera_matrix=camera, far_field_only=True, max_points=12
    )
    assert 4 <= len(capped.left.points) <= 12
    assert capped.metadata["relative_rotation"]["fit_point_count"] == capped.inlier_count
