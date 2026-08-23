from __future__ import annotations

import json
import math

import pytest

from Workstation.workstation_core.measurement_attempt import (
    CameraIntrinsicsSnapshot,
    CheckerboardExtrinsicsSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
)
from Workstation.workstation_core.measurement_controller import calculate_measurement_geometry


def _detection(side: str, u_px: float, timestamp_s: float) -> DetectionSnapshot:
    return DetectionSnapshot(
        camera=side,
        app_receive_monotonic_s=timestamp_s,
        width_px=640,
        height_px=480,
        bbox_xyxy_px=(u_px - 10.0, 230.0, u_px + 10.0, 250.0),
        center_px=(u_px, 240.0),
        confidence=0.95,
        class_id=1,
        class_name="target",
        pairing_timestamp_basis="app_receive_monotonic",
        pairing_timestamp_s=timestamp_s,
    )


def _attempt() -> MeasurementAttempt:
    camera = CameraIntrinsicsSnapshot(
        width_px=640,
        height_px=480,
        camera_matrix=((800.0, 0.0, 320.0), (0.0, 800.0, 240.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        reprojection_error_px=0.1,
        image_count=20,
    )
    extrinsics = CheckerboardExtrinsicsSnapshot(
        rotation_left_to_right=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        translation_left_to_right_m=(-1.0, 0.0, 0.0),
        reprojection_error_px=0.1,
        pair_count=20,
        frame_contract=(
            "opencv_optical:RDF(right,down,forward); "
            "X_right = R_left_to_right * X_left + T_left_to_right_m"
        ),
    )
    intrinsics = StereoIntrinsicsSnapshot(
        left=camera,
        right=camera,
        checkerboard_extrinsics=extrinsics,
        active_zoom_level=8,
        selected_profile_path="calibration/intrinsics/profile.json",
        content_sha256="a" * 64,
        intrinsics_content_sha256="b" * 64,
        selected_extrinsics_profile_path="calibration/extrinsics/profile.json",
        extrinsics_content_sha256="c" * 64,
        intrinsics_source_zoom_level=8,
        extrinsics_source_zoom_level=8,
        selection_mode="manual_same_zoom",
    )
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="integrity-replay",
        detection_pair_key=("checkerboard_stereo", 1.0, 1.01),
        created_monotonic_s=1.02,
        created_wall_s=2.0,
        detection_midpoint_monotonic_s=1.005,
        timestamp_basis="app_receive_monotonic",
        triangulation_source="checkerboard",
        left_detection=_detection("left", 320.0, 1.0),
        right_detection=_detection("right", 240.0, 1.01),
        left_attitude=None,
        right_attitude=None,
        left_gps=None,
        right_gps=None,
        intrinsics=intrinsics,
        baseline=None,
        geometry_config=GeometryConfigSnapshot(
            focal_length_px=800.0,
            max_time_delta_s=0.1,
            max_vertical_delta_px=5.0,
            require_same_class=True,
            timestamp_basis="app_receive_monotonic",
            manual_focal_override=False,
        ),
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        attitude_pair_delta_s=None,
        relative_roll_deg=None,
        relative_pitch_deg=None,
        relative_yaw_deg=None,
        source_quality="GOOD",
        warnings=(),
        measurement_method_id="checkerboard_stereo",
        measurement_method_instance_id="profile:checkerboard-replay",
        measurement_geometry_kind="relative_pose",
        measurement_geometry_sha256="d" * 64,
        measurement_selection_reason="test_fixture",
        measurement_component_sources=(("relative_pose", "checkerboard_profile"),),
    )


def test_canonical_measurement_attempt_replays_identical_geometry_with_numeric_oracle() -> None:
    original = _attempt()
    first = calculate_measurement_geometry(original)
    recovered = MeasurementAttempt.from_canonical_payload(
        json.loads(original.canonical_json())
    )
    replay = calculate_measurement_geometry(recovered)

    assert first.triangulated is not None
    assert replay.triangulated is not None
    assert first.estimate is not None
    assert replay.estimate is not None
    for actual, repeated in zip(
        first.triangulated.point_m,
        replay.triangulated.point_m,
        strict=True,
    ):
        assert abs(actual - repeated) < 1.0e-12
    assert abs(first.triangulated.range_m - replay.triangulated.range_m) < 1.0e-12
    assert abs(first.triangulated.closest_gap_m - replay.triangulated.closest_gap_m) < 1.0e-12

    # Independent hand projection: target is 10 m forward from Left, the
    # baseline-center origin is 0.5 m to the right of Left.
    assert first.triangulated.point_m == pytest.approx((10.0, -0.5, 0.0), abs=1.0e-9)
    assert first.triangulated.range_m == pytest.approx(math.sqrt(100.25), abs=1.0e-9)
    assert first.triangulated.closest_gap_m == pytest.approx(0.0, abs=1.0e-9)
