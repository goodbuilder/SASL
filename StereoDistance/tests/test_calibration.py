import json
import sys
from pathlib import Path

import numpy as np

from wiris_stereo_distance import calibration
from wiris_stereo_distance.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoIntrinsicsResult,
    load_intrinsics_json,
    opencv_available,
    save_intrinsics_json,
    stereo_calibration_extrinsics_from_result,
)


def test_checkerboard_config_pattern_and_square_size() -> None:
    config = CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03)

    assert config.pattern_size == (9, 6)
    assert config.square_size_m == 0.03


def test_stereo_calibration_extrinsics_convert_opencv_pose_to_right_in_left() -> None:
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=12,
    )
    result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        stereo_calibration={
            "rotation_left_to_right": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "translation_left_to_right_m": [-0.3, 0.0, 0.0],
            "pair_count": 12,
            "rms_error_px": 0.4,
            "epipolar_mean_error_px": 0.08,
            "epipolar_max_error_px": 0.2,
        },
    )

    extrinsics = stereo_calibration_extrinsics_from_result(result)

    assert extrinsics.rotation_right_to_left == [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    assert extrinsics.translation_right_in_left_m == (0.3, -0.0, -0.0)
    assert extrinsics.baseline_m == 0.3
    assert extrinsics.pair_count == 12


def test_save_intrinsics_json(tmp_path: Path) -> None:
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.1, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.4,
        image_count=20,
        image_results=[
            {
                "path": "frame001.png",
                "accepted": True,
                "detector_method": "findChessboardCornersSB",
                "corner_count": 54,
                "image_size": {"width": 1280, "height": 720},
                "reprojection_error_px": 0.31,
                "reject_reason": None,
            }
        ],
        coverage_summary={
            "status": "warning",
            "accepted_count": 1,
            "warnings": ["insufficient screen-position coverage"],
        },
    )
    result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        stereo_calibration={
            "method": "cv2.stereoCalibrate",
            "pair_count": 18,
            "rms_error_px": 0.22,
            "epipolar_mean_error_px": 0.08,
            "pair_results": [],
        },
    )
    output = tmp_path / "intrinsics.json"

    save_intrinsics_json(output, result, metadata={"test": True})

    text = output.read_text()
    assert '"fx"' not in text  # Matrix form is stored for direct OpenCV use.
    assert '"camera_matrix"' in text
    assert '"reprojection_error_px": 0.4' in text
    assert '"image_results"' in text
    assert '"frame001.png"' in text
    assert '"coverage_summary"' in text
    assert '"insufficient screen-position coverage"' in text
    assert '"stereo_calibration"' in text
    assert '"epipolar_mean_error_px": 0.08' in text
    assert '"extrinsics"' not in text


def test_load_intrinsics_json_ignores_legacy_extrinsics(tmp_path: Path) -> None:
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.4,
        image_count=20,
    )
    output = tmp_path / "intrinsics.json"
    result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )

    save_intrinsics_json(output, result)
    payload = json.loads(output.read_text())
    payload["extrinsics"] = {
        "rotation_left_to_right": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "rotation_right_to_left": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "translation_left_to_right_m": [-0.3, 0.0, 0.0],
        "translation_right_in_left_m": [0.3, 0.0, 0.0],
        "reprojection_error_px": 0.22,
        "pair_count": 18,
    }
    output.write_text(json.dumps(payload))

    loaded = load_intrinsics_json(output)

    assert loaded.left.camera_matrix == intrinsics.camera_matrix
    assert loaded.right.image_count == 20
    assert loaded.stereo_calibration is None
    assert not hasattr(loaded, "extrinsics")


def test_opencv_available_returns_status_tuple() -> None:
    available, error = opencv_available()

    assert isinstance(available, bool)
    assert error is None or isinstance(error, str)


def test_find_checkerboard_prefers_sb_detector() -> None:
    class FakeCv2:
        CALIB_CB_EXHAUSTIVE = 1
        CALIB_CB_ACCURACY = 2

        def findChessboardCornersSB(self, image, pattern_size, flags):
            assert image == "gray"
            assert pattern_size == (9, 6)
            assert flags == 3
            return True, ["corner"]

        def findChessboardCorners(self, image, pattern_size, flags):
            raise AssertionError("classic detector should not run after SB succeeds")

    found, corners, method = calibration._find_checkerboard(FakeCv2(), "gray", (9, 6))

    assert found is True
    assert corners == ["corner"]
    assert method == "findChessboardCornersSB"


def test_find_checkerboard_falls_back_to_classic_detector() -> None:
    class FakeCv2:
        CALIB_CB_EXHAUSTIVE = 1
        CALIB_CB_ACCURACY = 2

        def findChessboardCornersSB(self, image, pattern_size, flags):
            return False, None

        def findChessboardCorners(self, image, pattern_size, flags):
            assert flags is None
            return True, ["classic-corner"]

    found, corners, method = calibration._find_checkerboard(FakeCv2(), "gray", (9, 6))

    assert found is True
    assert corners == ["classic-corner"]
    assert method == "findChessboardCorners"


def test_calibrate_camera_records_per_image_evidence(monkeypatch) -> None:
    class FakeCv2:
        TERM_CRITERIA_EPS = 1
        TERM_CRITERIA_MAX_ITER = 2
        NORM_L2 = 4

        def __init__(self) -> None:
            self.calls = 0

        def findChessboardCorners(self, image, pattern_size, flags):
            self.calls += 1
            if self.calls == 4:
                return False, None
            return True, np.zeros((4, 1, 2), dtype=np.float32)

        def cornerSubPix(self, image, corners, win_size, zero_zone, criteria):
            return corners

        def calibrateCamera(self, object_points, image_points, image_size, camera_matrix, distortion):
            assert len(object_points) == 3
            return (
                1.0,
                np.eye(3, dtype=np.float64),
                np.zeros((5, 1), dtype=np.float64),
                [np.zeros((3, 1), dtype=np.float64) for _ in object_points],
                [np.zeros((3, 1), dtype=np.float64) for _ in object_points],
            )

        def projectPoints(self, obj, rvec, tvec, camera_matrix, distortion):
            return np.zeros((4, 1, 2), dtype=np.float32), None

        def norm(self, img, projected, norm_type):
            return 2.0

    fake_cv2 = FakeCv2()
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)
    monkeypatch.setattr(calibration, "_load_grayscale", lambda path: np.zeros((12, 16), dtype=np.uint8))

    result = calibration.calibrate_camera_from_images(
        ["accepted-1.png", "accepted-2.png", "accepted-3.png", "rejected.png"],
        CheckerboardConfig(inner_cols=2, inner_rows=2, square_size_m=0.03),
    )

    assert result.image_count == 3
    assert result.reprojection_error_px == 1.0
    assert [entry["accepted"] for entry in result.image_results] == [True, True, True, False]
    assert result.image_results[0]["detector_method"] == "findChessboardCorners"
    assert result.image_results[0]["corner_count"] == 4
    assert result.image_results[0]["reprojection_error_px"] == 1.0
    assert result.image_results[0]["board_coverage"]["center_norm"] == {"x": 0.0, "y": 0.0}
    assert result.coverage_summary["status"] == "warning"
    assert "insufficient screen-position coverage" in result.coverage_summary["warnings"]
    assert "insufficient checkerboard scale variation" in result.coverage_summary["warnings"]
    assert result.image_results[3]["reject_reason"] == "checkerboard not found"


def test_coverage_summary_accepts_distributed_checkerboards() -> None:
    image_results = []
    for index, (x, y, area) in enumerate(
        [
            (0.15, 0.15, 0.05),
            (0.85, 0.15, 0.08),
            (0.15, 0.85, 0.11),
            (0.85, 0.85, 0.14),
            (0.50, 0.50, 0.20),
            (0.25, 0.50, 0.09),
            (0.75, 0.50, 0.12),
            (0.50, 0.25, 0.07),
            (0.50, 0.75, 0.16),
            (0.35, 0.35, 0.10),
        ]
    ):
        image_results.append(
            {
                "accepted": True,
                "board_coverage": {
                    "center_norm": {"x": x, "y": y},
                    "area_fraction": area,
                },
            }
        )

    summary = calibration._coverage_summary(image_results)

    assert summary["status"] == "ok"
    assert summary["warnings"] == []
    assert summary["center_span_norm"] == {"x": 0.7, "y": 0.7}


def test_calibrate_stereo_records_pair_and_epipolar_evidence(monkeypatch) -> None:
    class FakeCv2:
        TERM_CRITERIA_EPS = 1
        TERM_CRITERIA_MAX_ITER = 2
        CALIB_FIX_INTRINSIC = 4

        def __init__(self) -> None:
            self.stereo_flags = None

        def findChessboardCorners(self, image, pattern_size, flags):
            corners = np.array([[[0.0, 0.0]], [[1.0, 0.0]], [[0.0, 1.0]], [[1.0, 1.0]]], dtype=np.float32)
            return True, corners

        def cornerSubPix(self, image, corners, win_size, zero_zone, criteria):
            return corners

        def stereoCalibrate(
            self,
            object_points,
            left_image_points,
            right_image_points,
            left_camera,
            left_distortion,
            right_camera,
            right_distortion,
            image_size,
            flags=0,
        ):
            self.stereo_flags = flags
            assert len(object_points) == 3
            assert image_size == (16, 12)
            fundamental = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]], dtype=np.float64)
            return (
                0.25,
                left_camera,
                left_distortion,
                right_camera,
                right_distortion,
                np.eye(3, dtype=np.float64),
                np.array([[0.3], [0.0], [0.0]], dtype=np.float64),
                np.eye(3, dtype=np.float64),
                fundamental,
            )

    fake_cv2 = FakeCv2()
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)
    monkeypatch.setattr(calibration, "_load_grayscale", lambda path: np.zeros((12, 16), dtype=np.uint8))
    intrinsics = CameraIntrinsics(
        width=16,
        height=12,
        camera_matrix=[[10.0, 0.0, 8.0], [0.0, 10.0, 6.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.1,
        image_count=3,
    )

    evidence = calibration.calibrate_stereo_from_image_pairs(
        ["left-1.png", "left-2.png", "left-3.png", "left-4.png"],
        ["right-1.png", "right-2.png", "right-3.png", "right-5.png"],
        CheckerboardConfig(inner_cols=2, inner_rows=2, square_size_m=0.03),
        intrinsics,
        intrinsics,
    )

    assert fake_cv2.stereo_flags == fake_cv2.CALIB_FIX_INTRINSIC
    assert evidence["method"] == "cv2.stereoCalibrate"
    assert evidence["pair_count"] == 3
    assert evidence["rms_error_px"] == 0.25
    assert evidence["translation_left_to_right_m"] == [0.3, 0.0, 0.0]
    assert evidence["epipolar_mean_error_px"] == 0.0
    assert evidence["epipolar_max_error_px"] == 0.0
    assert [entry["accepted"] for entry in evidence["pair_results"]] == [True, True, True, False, False]
    assert evidence["pair_results"][3]["left_path"] == "left-4.png"
    assert evidence["pair_results"][3]["right_path"] is None
    assert evidence["pair_results"][4]["left_path"] is None
    assert evidence["pair_results"][4]["right_path"] == "right-5.png"
    assert evidence["pair_results"][0]["epipolar_mean_error_px"] == 0.0
