from __future__ import annotations

from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.annotation_image_view import qimage_pixel_sha256
from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.workspaces import OpticalPosePane
from Workstation.workstation_core.optical_pose_auto_select import (
    detect_horizon,
    detect_mountain,
)


def _camera_matrix() -> tuple[tuple[float, float, float], ...]:
    return ((800.0, 0.0, 320.0), (0.0, 800.0, 240.0), (0.0, 0.0, 1.0))


def test_synthetic_horizon_returns_native_endpoints_and_quality_metrics() -> None:
    np = pytest.importorskip("numpy")
    cv2 = pytest.importorskip("cv2")
    image = np.zeros((480, 640), dtype=np.uint8)
    for u in range(640):
        v = int(round(208.0 + 0.08 * u))
        image[:v, u] = 215
        image[v:, u] = 55
    image[250:, :] = np.clip(
        image[250:, :].astype(np.int16)
        + np.random.default_rng(12).normal(0.0, 7.0, image[250:, :].shape),
        0,
        255,
    ).astype(np.uint8)

    result = detect_horizon(
        image,
        camera_matrix=_camera_matrix(),
        distortion_coefficients=(0.0,) * 5,
    )

    assert result.state == "GOOD"
    assert result.eligible is True
    assert len(result.points) == 2
    assert result.inlier_count >= 4
    assert result.residual_px is not None
    assert result.confidence >= 0.58
    assert result.points[0][1] == pytest.approx(208.0, abs=4.0)
    assert result.points[1][1] == pytest.approx(259.0, abs=4.0)
    assert cv2 is not None  # documents the explicit OpenCV-only test path


def test_blank_horizon_is_no_data_and_never_eligible() -> None:
    np = pytest.importorskip("numpy")
    result = detect_horizon(
        np.zeros((480, 640), dtype=np.uint8),
        camera_matrix=_camera_matrix(),
        distortion_coefficients=(0.0,) * 5,
    )

    assert result.state == "NO_DATA"
    assert result.eligible is False
    assert result.failure_reason


def test_synthetic_stereo_features_are_distributed_and_unique() -> None:
    np = pytest.importorskip("numpy")
    cv2 = pytest.importorskip("cv2")
    rng = np.random.default_rng(21)
    left = np.zeros((480, 640), dtype=np.uint8)
    for _ in range(90):
        center = (int(rng.integers(20, 620)), int(rng.integers(35, 445)))
        cv2.circle(left, center, int(rng.integers(2, 6)), int(rng.integers(100, 255)), -1)
        cv2.line(
            left,
            center,
            (center[0] + int(rng.integers(-9, 10)), center[1] + int(rng.integers(-9, 10))),
            int(rng.integers(80, 220)),
            1,
        )
    right = np.roll(left, -18, axis=1)
    result = detect_mountain(
        left,
        right,
        left_camera_matrix=_camera_matrix(),
        right_camera_matrix=_camera_matrix(),
        left_distortion_coefficients=(0.0,) * 5,
        right_distortion_coefficients=(0.0,) * 5,
        baseline_ned_m=(0.0, 2.0, 0.0),
    )

    assert result.inlier_count >= 4
    assert len(result.left.points) >= 4
    assert len(result.left.points) == len(result.right.points)
    assert result.left.spatial_rank >= 2
    assert len(set(result.left.points)) == len(result.left.points)
    assert result.left.eligible is True
    assert result.residual_px is not None


def test_mountain_solver_gate_rejects_near_collinear_points() -> None:
    with pytest.raises(ValueError, match="near-collinear"):
        OpticalPoseController._validate_mountain_point_distribution(
            tuple((float(index * 100), float(index * 20 + 10)) for index in range(4)),
            side="left",
        )


def _suggestion_payload(left: QtGui.QImage, right: QtGui.QImage) -> dict[str, object]:
    return {
        "suggestion_id": "auto-horizon-test",
        "mode": "horizon",
        "state": "GOOD",
        "provenance_state": "AUTO_PROPOSED",
        "detection_method": "synthetic-gradient",
        "confidence": 0.91,
        "inlier_count": 128,
        "residual_px": 0.7,
        "eligible": True,
        "sides": {
            "left": {
                "frame_id": "frame-left",
                "image_sha256": qimage_pixel_sha256(left),
                "points": [{"u_px": 20.0, "v_px": 210.0}, {"u_px": 620.0, "v_px": 230.0}],
            },
            "right": {
                "frame_id": "frame-right",
                "image_sha256": qimage_pixel_sha256(right),
                "points": [{"u_px": 24.0, "v_px": 211.0}, {"u_px": 624.0, "v_px": 231.0}],
            },
        },
    }


def test_auto_overlay_acceptance_and_manual_edit_provenance() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = OpticalPosePane()
    commands: list[tuple[str, dict[str, object]]] = []
    pane.command_requested.connect(
        lambda command_id, payload: commands.append((str(command_id), dict(payload)))
    )
    left = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    right = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    left.fill(QtCore.Qt.darkGray)
    right.fill(QtCore.Qt.lightGray)
    pane.apply_snapshot(
        {
            "frozen_frames": {
                "left": {"image": left, "frame_id": "frame-left", "image_sha256": qimage_pixel_sha256(left)},
                "right": {"image": right, "frame_id": "frame-right", "image_sha256": qimage_pixel_sha256(right)},
            }
        }
    )
    suggestion = _suggestion_payload(left, right)
    pane.set_auto_suggestions([suggestion])

    assert pane.annotation_views["left"].annotation_points("horizon") == ()
    assert len(pane.annotation_views["left"].suggestion_points("horizon")) == 2
    assert pane.use_suggestion_button.isEnabled()

    accepted = dict(suggestion)
    accepted["provenance_state"] = "AUTO_ACCEPTED"
    pane.apply_snapshot(
        {
            "auto_suggestions": [suggestion],
            "accepted_auto_suggestion": accepted,
        }
    )
    assert pane.annotation_views["left"].annotation_points("horizon")
    assert "AUTO_ACCEPTED" in pane.auto_suggestion_status_label.text()

    pane.annotation_views["left"].add_source_point((320.0, 240.0), mode="horizon")
    assert "MANUAL_EDITED" in pane.auto_suggestion_status_label.text()
    assert commands[-1][0] == "alignment.pose.annotation_changed"
    assert commands[-1][1]["provenance_state"] == "MANUAL_EDITED"
    assert commands[-1][1]["annotations"]["left"].to_mapping()["horizon_line"]

    pane.deleteLater()
    app.processEvents()
