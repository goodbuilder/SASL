from dataclasses import replace
from types import SimpleNamespace

import pytest

from wiris_stereo_distance.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
    StereoIntrinsicsResult,
)
from wiris_stereo_distance.measurement_controller import MeasurementGeometryInput, calculate_measurement_geometry
from wiris_stereo_distance.stereo import DetectionObservation, StereoCalibration


def detection(camera: str, *, x1: float, x2: float, t: float = 10.0) -> DetectionObservation:
    return DetectionObservation(
        camera=camera,
        timestamp_s=t,
        width=1280,
        height=720,
        x1=x1,
        y1=340.0,
        x2=x2,
        y2=380.0,
        confidence=0.9,
        class_id=1,
        class_name="target",
    )


def intrinsics_result(*, extrinsics: StereoExtrinsics | None = None) -> StereoIntrinsicsResult:
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    return StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        extrinsics=extrinsics,
    )


def pixhawk_attitude(timestamp_s: float = 10.0) -> SimpleNamespace:
    return SimpleNamespace(q=(1.0, 0.0, 0.0, 0.0), timestamp_s=timestamp_s)


def test_measurement_controller_pixhawk_rtk_triangulates_valid_pair() -> None:
    result = calculate_measurement_geometry(
        MeasurementGeometryInput(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(),
            intrinsics=intrinsics_result(),
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=pixhawk_attitude(),
            translation_right_in_left_m=(0.3, 0.0, 0.0),
            sync_quality="GOOD",
        )
    )

    assert result.match is not None
    assert result.triangulated is not None
    assert result.estimate is not None
    assert result.estimate.range_m == result.triangulated.range_m
    assert result.triangulated.point_center_frd_m[0] > 0.0
    assert result.triangulation_reason == "ok"
    assert result.warnings == ()


def test_measurement_controller_reports_pixhawk_geometry_waiting() -> None:
    result = calculate_measurement_geometry(
        MeasurementGeometryInput(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(),
            intrinsics=intrinsics_result(),
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=None,
            sync_quality="DEGRADED",
        )
    )

    assert result.match is not None
    assert result.triangulated is None
    assert result.estimate is None
    assert "triangulation waiting for Pixhawk R" in result.warnings
    assert result.triangulation_reason == "missing_pixhawk_rotation"


def test_measurement_controller_reports_intrinsics_waiting() -> None:
    result = calculate_measurement_geometry(
        MeasurementGeometryInput(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(),
            intrinsics=None,
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=pixhawk_attitude(),
            translation_right_in_left_m=(0.3, 0.0, 0.0),
            sync_quality="GOOD",
        )
    )

    assert result.match is not None
    assert result.triangulated is None
    assert result.estimate is None
    assert "triangulation waiting for intrinsics" in result.warnings


@pytest.mark.parametrize(
    ("camera", "expected_reason"),
    (
        ("left", "left_pixel_grid_mismatch"),
        ("right", "right_pixel_grid_mismatch"),
    ),
)
def test_measurement_controller_fails_closed_when_detection_grid_does_not_match_camera_k(
    camera: str,
    expected_reason: str,
) -> None:
    left = detection("left", x1=690.0, x2=710.0)
    right = detection("right", x1=640.0, x2=660.0)
    if camera == "left":
        left = replace(left, width=1920, height=1080)
    else:
        right = replace(right, width=1920, height=1080)

    result = calculate_measurement_geometry(
        MeasurementGeometryInput(
            left=left,
            right=right,
            calibration=StereoCalibration(),
            intrinsics=intrinsics_result(),
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=pixhawk_attitude(),
            translation_right_in_left_m=(0.3, 0.0, 0.0),
            sync_quality="GOOD",
        )
    )

    assert result.match is not None
    assert result.triangulated is None
    assert result.estimate is None
    assert result.triangulation_reason == expected_reason
    assert result.warnings == (
        f"{camera} detection pixel grid 1920x1080 "
        f"does not match {camera} intrinsics grid 1280x720",
    )


def test_measurement_controller_keeps_match_validation_separate_from_triangulation() -> None:
    result = calculate_measurement_geometry(
        MeasurementGeometryInput(
            left=detection("left", x1=690.0, x2=710.0, t=10.0),
            right=detection("right", x1=640.0, x2=660.0, t=12.0),
            calibration=StereoCalibration(max_time_delta_s=0.25),
            intrinsics=intrinsics_result(),
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=pixhawk_attitude(),
            translation_right_in_left_m=(0.3, 0.0, 0.0),
            sync_quality="BAD",
        )
    )

    assert result.match is None
    assert result.triangulated is not None
    assert result.estimate is None


def test_measurement_controller_checkerboard_extrinsics_triangulates_without_pixhawk() -> None:
    result = calculate_measurement_geometry(
        MeasurementGeometryInput(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(),
            intrinsics=intrinsics_result(
                extrinsics=StereoExtrinsics(
                    rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                    translation_left_to_right_m=[-0.3, 0.0, 0.0],
                    reprojection_error_px=0.2,
                    pair_count=12,
                )
            ),
            triangulation_source="checkerboard",
            sync_quality="GOOD",
        )
    )

    assert result.match is not None
    assert result.triangulated is not None
    assert result.estimate is not None
    assert round(result.triangulated.baseline_m, 1) == 0.3
    assert result.warnings == ()


def test_measurement_controller_checkerboard_mode_waits_for_stereo_extrinsics() -> None:
    result = calculate_measurement_geometry(
        MeasurementGeometryInput(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(),
            intrinsics=intrinsics_result(),
            triangulation_source="checkerboard",
            sync_quality="GOOD",
        )
    )

    assert result.triangulated is None
    assert result.estimate is None
    assert "triangulation waiting for checkerboard stereo extrinsics" in result.warnings
