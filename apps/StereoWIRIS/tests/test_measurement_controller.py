from dataclasses import replace
from types import SimpleNamespace

import pytest

from wiris_stereo_distance.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
    StereoIntrinsicsResult,
)
from wiris_stereo_distance.measurement_attempt import (
    AttitudeSnapshot,
    BaselineSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
    build_checkerboard_baseline_snapshot,
    detection_pair_key,
)
from wiris_stereo_distance.measurement_controller import calculate_measurement_geometry
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


def geometry_attempt(
    *,
    left,
    right,
    calibration,
    intrinsics,
    triangulation_source,
    left_extrinsic=None,
    right_extrinsic=None,
    left_attitude_age_s=None,
    right_attitude_age_s=None,
    attitude_pair_delta_s=None,
    relative_roll_deg=None,
    relative_pitch_deg=None,
    relative_yaw_deg=None,
    sync_quality="unknown",
    warnings=(),
    translation_right_in_left_m=None,
) -> MeasurementAttempt:
    left_detection = DetectionSnapshot.from_observation(
        left, pairing_timestamp_basis=calibration.timestamp_basis, pairing_timestamp_s=left.timestamp_s
    )
    right_detection = DetectionSnapshot.from_observation(
        right, pairing_timestamp_basis=calibration.timestamp_basis, pairing_timestamp_s=right.timestamp_s
    )

    def attitude(camera, state):
        if state is None:
            return None
        return AttitudeSnapshot(
            camera=camera,
            app_receive_monotonic_s=state.timestamp_s,
            quaternion_wxyz=tuple(state.q),
            namespace=camera,
            topic=f"/{camera}/attitude",
            px4_timestamp_s=None,
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
        )

    left_attitude = attitude("left", left_extrinsic)
    right_attitude = attitude("right", right_extrinsic)
    intrinsics_snapshot = (
        None
        if intrinsics is None
        else StereoIntrinsicsSnapshot.from_result(
            intrinsics, active_zoom_level=8, selected_profile_path="fixture.json"
        )
    )
    if triangulation_source == "checkerboard":
        baseline = build_checkerboard_baseline_snapshot(intrinsics_snapshot)
    else:
        basis = None if translation_right_in_left_m is None or left_attitude is None else left_attitude.q
        baseline = BaselineSnapshot(
            source="gps-rtk",
            baseline_m=calibration.baseline_m,
            ned_right_from_left_m=(calibration.baseline_m, 0.0, 0.0),
            translation_right_in_left_rdf_m=translation_right_in_left_m,
            left_basis_quaternion_wxyz=basis,
            fix_quality="GOOD",
            warnings=(),
            usable_for_translation=translation_right_in_left_m is not None,
            unusable_reason="" if translation_right_in_left_m is not None else "missing fixture translation",
            gps_pair_delta_s=0.0,
            left_age_at_capture_s=0.0,
            right_age_at_capture_s=0.0,
            left_age_from_detection_midpoint_s=0.0,
            right_age_from_detection_midpoint_s=0.0,
            left_basis_attitude_monotonic_s=(
                None if left_attitude is None or translation_right_in_left_m is None else left_attitude.timestamp_s
            ),
        )
    key = detection_pair_key(
        triangulation_source=triangulation_source,
        timestamp_basis=calibration.timestamp_basis,
        left=left,
        right=right,
        left_pairing_timestamp_s=left.timestamp_s,
        right_pairing_timestamp_s=right.timestamp_s,
    )
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="test",
        detection_pair_key=key,
        created_monotonic_s=20.0,
        created_wall_s=100.0,
        detection_midpoint_monotonic_s=(left.timestamp_s + right.timestamp_s) / 2.0,
        timestamp_basis=calibration.timestamp_basis,
        triangulation_source=triangulation_source,
        left_detection=left_detection,
        right_detection=right_detection,
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=None,
        right_gps=None,
        intrinsics=intrinsics_snapshot,
        baseline=baseline,
        geometry_config=GeometryConfigSnapshot(
            focal_length_px=calibration.focal_length_px,
            max_time_delta_s=(
                None if calibration.max_time_delta_s == float("inf") else calibration.max_time_delta_s
            ),
            max_vertical_delta_px=(
                None
                if calibration.max_vertical_delta_px == float("inf")
                else calibration.max_vertical_delta_px
            ),
            require_same_class=calibration.require_same_class,
            timestamp_basis=calibration.timestamp_basis,
            manual_focal_override=False,
        ),
        left_attitude_age_s=left_attitude_age_s,
        right_attitude_age_s=right_attitude_age_s,
        attitude_pair_delta_s=attitude_pair_delta_s,
        relative_roll_deg=relative_roll_deg,
        relative_pitch_deg=relative_pitch_deg,
        relative_yaw_deg=relative_yaw_deg,
        source_quality=sync_quality,
        warnings=tuple(warnings),
    )


def test_measurement_controller_pixhawk_rtk_triangulates_valid_pair() -> None:
    result = calculate_measurement_geometry(
        geometry_attempt(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3),
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
    assert result.estimate.distance_m == result.triangulated.distance_m
    assert result.triangulated.point_frame == "left_camera_rdf"
    assert result.triangulated.output_frame == "baseline_center_frd"
    assert result.triangulated.output_axis_contract == "baseline_center_frd:forward_right_down"
    assert result.triangulated.output_vector_contract == "baseline_center_frd:[forward,right,down,range]"
    assert result.triangulated.point_m == result.triangulated.output_point_m
    assert result.triangulated.source_point_left_rdf_m is not None
    assert result.triangulation_reason == "ok"
    assert result.warnings == ()


def test_measurement_controller_reports_pixhawk_geometry_waiting() -> None:
    result = calculate_measurement_geometry(
        geometry_attempt(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3),
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
        geometry_attempt(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3),
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
    assert result.triangulation_reason == "missing_intrinsics"


def test_measurement_controller_rejects_zero_disparity_before_ray_geometry() -> None:
    result = calculate_measurement_geometry(
        geometry_attempt(
            left=detection("left", x1=640.0, x2=660.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3),
            intrinsics=intrinsics_result(),
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=pixhawk_attitude(),
            translation_right_in_left_m=(0.3, 0.0, 0.0),
            sync_quality="GOOD",
        )
    )

    assert result.triangulated is None
    assert result.estimate is None
    assert result.triangulation_reason == "correspondence_rejected"


@pytest.mark.parametrize(
    ("camera", "expected_reason"),
    (
        ("left", "left_pixel_grid_mismatch"),
        ("right", "right_pixel_grid_mismatch"),
    ),
)
def test_measurement_controller_fails_closed_on_detection_intrinsics_grid_mismatch(
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
        geometry_attempt(
            left=left,
            right=right,
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3),
            intrinsics=intrinsics_result(),
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=pixhawk_attitude(),
            translation_right_in_left_m=(0.3, 0.0, 0.0),
            sync_quality="GOOD",
        )
    )

    assert result.triangulated is None
    assert result.estimate is None
    assert result.triangulation_reason == expected_reason
    assert result.warnings == (
        f"{camera} detection pixel grid 1920x1080 "
        f"does not match {camera} intrinsics grid 1280x720",
    )


def test_measurement_controller_rejects_correspondence_before_triangulation() -> None:
    result = calculate_measurement_geometry(
        geometry_attempt(
            left=detection("left", x1=690.0, x2=710.0, t=10.0),
            right=detection("right", x1=640.0, x2=660.0, t=12.0),
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3, max_time_delta_s=0.25),
            intrinsics=intrinsics_result(),
            triangulation_source="pixhawk",
            left_extrinsic=pixhawk_attitude(),
            right_extrinsic=pixhawk_attitude(),
            translation_right_in_left_m=(0.3, 0.0, 0.0),
            sync_quality="BAD",
        )
    )

    assert result.match is None
    assert result.triangulated is None
    assert result.estimate is None
    assert result.triangulation_reason == "correspondence_rejected"


def test_measurement_controller_checkerboard_extrinsics_triangulates_without_pixhawk() -> None:
    result = calculate_measurement_geometry(
        geometry_attempt(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3),
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
        geometry_attempt(
            left=detection("left", x1=690.0, x2=710.0),
            right=detection("right", x1=640.0, x2=660.0),
            calibration=StereoCalibration(focal_length_px=900.0, baseline_m=0.3),
            intrinsics=intrinsics_result(),
            triangulation_source="checkerboard",
            sync_quality="GOOD",
        )
    )

    assert result.triangulated is None
    assert result.estimate is None
    assert "triangulation waiting for checkerboard stereo extrinsics" in result.warnings
