from dataclasses import replace
from math import cos, degrees, radians, sin
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
    CameraIntrinsicsSnapshot,
    CheckerboardExtrinsicsSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
    build_checkerboard_baseline_snapshot,
    build_pixhawk_baseline_snapshot,
    detection_pair_key,
)
from wiris_stereo_distance.measurement_controller import calculate_measurement_geometry
from wiris_stereo_distance.rtk_geometry import EARTH_RADIUS_M
from wiris_stereo_distance.stereo import DetectionObservation, correspondence_rejection_reason


def detection(camera: str, center_x: float, *, timestamp_s: float = 10.0, metadata=None):
    return DetectionObservation(
        camera=camera,
        timestamp_s=timestamp_s,
        width=1280,
        height=720,
        x1=center_x - 10.0,
        y1=350.0,
        x2=center_x + 10.0,
        y2=370.0,
        confidence=0.95,
        class_id=2,
        class_name="drone",
        timestamp_metadata={} if metadata is None else metadata,
    )


def calibration_result(*, right_fx: float = 900.0, checkerboard: bool = False, distorted: bool = False):
    left_k = [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
    right_k = [[right_fx, 0.0, 640.0], [0.0, right_fx, 360.0], [0.0, 0.0, 1.0]]
    left_distortion = [0.01, -0.02, 0.0, 0.0, 0.0] if distorted else [0.0]
    right_distortion = [0.03, -0.04, 0.0, 0.0, 0.0] if distorted else [0.0]
    left = CameraIntrinsics(1280, 720, left_k, left_distortion, 0.2, 20)
    right = CameraIntrinsics(1280, 720, right_k, right_distortion, 0.3, 21)
    extrinsics = None
    if checkerboard:
        extrinsics = StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.1,
            pair_count=12,
        )
    return StereoIntrinsicsResult(
        left=left,
        right=right,
        board=CheckerboardConfig(9, 6, 0.03),
        extrinsics=extrinsics,
    )


def attitude(camera: str, q=(1.0, 0.0, 0.0, 0.0), timestamp_s: float = 10.0):
    return AttitudeSnapshot(
        camera=camera,
        app_receive_monotonic_s=timestamp_s,
        quaternion_wxyz=tuple(q),
        namespace=camera,
        topic=f"/{camera}/fmu/out/vehicle_attitude",
        px4_timestamp_s=timestamp_s - 1.0,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )


def gps(camera: str, *, longitude_deg: float, timestamp_s: float):
    return GpsSnapshot(
        camera=camera,
        app_receive_monotonic_s=timestamp_s,
        latitude_deg=0.0,
        longitude_deg=longitude_deg,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
        eph_m=0.01,
        epv_m=0.02,
        rtk_label="RTK_FIXED",
        namespace=camera,
        topic=f"/{camera}/fmu/out/vehicle_gps_position",
        px4_timestamp_s=timestamp_s - 1.0,
    )


def pixhawk_attempt(*, intrinsics_source=None) -> MeasurementAttempt:
    left_source = detection("left", 640.0)
    right_source = detection("right", 613.0)
    left_detection = DetectionSnapshot.from_observation(
        left_source, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
    )
    right_detection = DetectionSnapshot.from_observation(
        right_source, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
    )
    left_attitude = attitude("left")
    right_attitude = attitude("right")
    longitude_delta_deg = degrees(0.3 / EARTH_RADIUS_M)
    left_gps = gps("left", longitude_deg=0.0, timestamp_s=9.8)
    right_gps = gps("right", longitude_deg=longitude_delta_deg, timestamp_s=9.9)
    baseline = build_pixhawk_baseline_snapshot(
        left_gps=left_gps,
        right_gps=right_gps,
        selected_left_quaternion_wxyz=left_attitude.q,
        selected_right_quaternion_wxyz=right_attitude.q,
        selected_left_attitude_monotonic_s=left_attitude.timestamp_s,
        created_monotonic_s=10.2,
        detection_midpoint_monotonic_s=10.0,
    )
    result = calibration_result() if intrinsics_source is None else intrinsics_source
    intrinsics = StereoIntrinsicsSnapshot.from_result(
        result, active_zoom_level=8, selected_profile_path="zoom_08.json"
    )
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="session-a",
        detection_pair_key=detection_pair_key(
            triangulation_source="pixhawk",
            timestamp_basis="detection",
            left=left_source,
            right=right_source,
            left_pairing_timestamp_s=10.0,
            right_pairing_timestamp_s=10.0,
        ),
        created_monotonic_s=10.2,
        created_wall_s=1000.0,
        detection_midpoint_monotonic_s=10.0,
        timestamp_basis="detection",
        triangulation_source="pixhawk",
        left_detection=left_detection,
        right_detection=right_detection,
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
        intrinsics=intrinsics,
        baseline=baseline,
        geometry_config=GeometryConfigSnapshot(900.0, None, None, False, "detection", False),
        left_attitude_age_s=0.0,
        right_attitude_age_s=0.0,
        attitude_pair_delta_s=0.0,
        relative_roll_deg=0.0,
        relative_pitch_deg=0.0,
        relative_yaw_deg=0.0,
        source_quality="GOOD",
        warnings=(),
    )


def test_snapshots_are_deeply_immutable_and_hash_stable_after_source_mutation() -> None:
    metadata = {"rtp_timestamp": 123, "timestamp_source": "rtcp", "ignored": [1, 2]}
    observation = detection("left", 640.0, metadata=metadata)
    camera_matrix = [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
    distortion = [0.01, -0.02]
    raw_message = {"mutable": [1, 2]}
    attitude_q = [1.0, 0.0, 0.0, 0.0]
    attitude_state = SimpleNamespace(
        camera="left",
        timestamp_s=9.7,
        q=attitude_q,
        namespace="left",
        topic="/attitude",
        px4_timestamp_s=8.7,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )
    gps_state = SimpleNamespace(
        camera="left",
        timestamp_s=9.8,
        latitude_deg=0.0,
        longitude_deg=0.0,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
        eph_m=0.01,
        epv_m=0.02,
        rtk_label="RTK_FIXED",
        namespace="left",
        topic="/gps",
        px4_timestamp_s=8.8,
        heading_deg=123.0,
        heading_accuracy_deg=0.5,
        raw=raw_message,
    )
    detection_snapshot = DetectionSnapshot.from_observation(
        observation, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
    )
    intrinsics_snapshot = StereoIntrinsicsSnapshot.from_result(
        StereoIntrinsicsResult(
            left=CameraIntrinsics(1280, 720, camera_matrix, distortion, 0.2, 20),
            right=CameraIntrinsics(1280, 720, camera_matrix, distortion, 0.2, 20),
            board=CheckerboardConfig(9, 6, 0.03),
        ),
        active_zoom_level=8,
        selected_profile_path="zoom_08.json",
    )
    gps_snapshot = GpsSnapshot.from_state(gps_state)
    attitude_snapshot = AttitudeSnapshot.from_state(attitude_state)
    before = (
        hash(detection_snapshot),
        hash(intrinsics_snapshot),
        hash(gps_snapshot),
        hash(attitude_snapshot),
    )

    metadata["rtp_timestamp"] = 999
    metadata["ignored"].append(3)
    camera_matrix[0][0] = 1.0
    distortion.append(99.0)
    raw_message["mutable"].append(3)
    gps_state.longitude_deg = 12.0
    gps_state.altitude_m = -50.0
    gps_state.heading_deg = 321.0
    attitude_q[0] = 0.0
    attitude_q[3] = 1.0

    assert detection_snapshot.timestamp_metadata["rtp_timestamp"] == 123
    assert intrinsics_snapshot.left.camera_matrix[0][0] == 900.0
    assert intrinsics_snapshot.left.distortion_coefficients == (0.01, -0.02)
    assert not hasattr(gps_snapshot, "raw")
    assert gps_snapshot.longitude_deg == 0.0
    assert gps_snapshot.altitude_m == 100.0
    assert gps_snapshot.heading_deg == 123.0
    assert attitude_snapshot.quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert before == (
        hash(detection_snapshot),
        hash(intrinsics_snapshot),
        hash(gps_snapshot),
        hash(attitude_snapshot),
    )


def test_pixhawk_attempt_uses_one_left_quaternion_basis_and_reconstructs_known_point() -> None:
    attempt = pixhawk_attempt()
    assert attempt.baseline is not None
    assert attempt.baseline.left_basis_quaternion_wxyz == attempt.left_attitude.quaternion_wxyz
    assert attempt.translation_right_in_left_m == (0.3, 0.0, 0.0)

    geometry = calculate_measurement_geometry(attempt)

    assert geometry.triangulated is not None
    reconstructed = geometry.triangulated.point_left_rdf_m
    expected = (0.0, 0.0, 10.0)
    error_m = sum((actual - target) ** 2 for actual, target in zip(reconstructed, expected)) ** 0.5
    assert error_m <= 1e-9
    assert geometry.triangulated.range_m == 10.0
    assert geometry.triangulated.output_frame == "left_camera_rdf"


def test_selected_gps_pair_and_intrinsics_content_are_stable_and_independent() -> None:
    source = calibration_result(right_fx=910.0, distorted=True)
    attempt = pixhawk_attempt(intrinsics_source=source)
    before_hash = attempt.canonical_sha256()
    source.left.camera_matrix[0][0] = 1.0
    source.right.camera_matrix[0][0] = 2.0
    source.left.distortion_coefficients.append(88.0)

    assert attempt.left_gps.timestamp_s == 9.8
    assert attempt.right_gps.timestamp_s == 9.9
    assert attempt.baseline.ned_right_from_left_m == (0.0, 0.3, 0.0)
    assert attempt.intrinsics.left.camera_matrix[0][0] == 900.0
    assert attempt.intrinsics.right.camera_matrix[0][0] == 910.0
    assert attempt.intrinsics.left.distortion_coefficients == (0.01, -0.02, 0.0, 0.0, 0.0)
    assert attempt.intrinsics.right.distortion_coefficients == (0.03, -0.04, 0.0, 0.0, 0.0)
    assert attempt.canonical_sha256() == before_hash


def test_intrinsics_content_hash_is_deterministic_and_checkerboard_snapshot_is_self_contained() -> None:
    source = calibration_result(checkerboard=True)
    first = StereoIntrinsicsSnapshot.from_result(source, active_zoom_level=8, selected_profile_path="a.json")
    second = StereoIntrinsicsSnapshot.from_result(source, active_zoom_level=8, selected_profile_path="b.json")
    baseline = build_checkerboard_baseline_snapshot(first)

    assert first.content_sha256 == second.content_sha256
    assert first.left is not first.right
    assert baseline.translation_right_in_left_rdf_m == (0.3, 0.0, 0.0)
    source.extrinsics.translation_left_to_right_m[0] = -99.0
    assert first.checkerboard_extrinsics.translation_left_to_right_m == (-0.3, 0.0, 0.0)


def test_pair_id_is_collision_free_for_10000_sequences_and_scoped_by_session() -> None:
    template = pixhawk_attempt()
    same_session_ids = {
        replace(template, attempt_id=sequence).pair_id for sequence in range(1, 10_001)
    }

    assert len(same_session_ids) == 10_000
    assert replace(template, attempt_id=1, attempt_session_id="session-a").pair_id != replace(
        template, attempt_id=1, attempt_session_id="session-b"
    ).pair_id


@pytest.mark.parametrize(
    ("field_name", "wrong_side"),
    (
        ("left_attitude", "right"),
        ("right_attitude", "left"),
        ("left_gps", "right"),
        ("right_gps", "left"),
    ),
)
def test_attempt_rejects_cross_camera_state_slots(field_name: str, wrong_side: str) -> None:
    attempt = pixhawk_attempt()
    current = getattr(attempt, field_name)

    with pytest.raises(ValueError, match=f"{field_name} must preserve"):
        replace(attempt, **{field_name: replace(current, camera=wrong_side)})


def test_detection_payload_preserves_candidate_identity_for_fail_closed_matching() -> None:
    payload = {
        "timestamp_s": 10.0,
        "width": 1280,
        "height": 720,
        "best_detection": {
            "x1": 630.0,
            "y1": 350.0,
            "x2": 650.0,
            "y2": 370.0,
            "confidence": 0.95,
            "class_id": 2,
            "class_name": "drone",
        },
        "detections": [
            {"class_id": 2},
            {"class_id": 2},
            {"class_id": 7},
        ],
    }
    left = DetectionObservation.from_payload("left", payload)
    right = DetectionObservation.from_payload(
        "right",
        {
            **payload,
            "detections": [{"class_id": 2}],
        },
    )

    assert left is not None and right is not None
    assert left.candidate_class_ids == (2, 2, 7)
    assert correspondence_rejection_reason(left, right) == "ambiguous_left_candidates"
    assert correspondence_rejection_reason(None, right) == "missing_left_detection"
    assert correspondence_rejection_reason(left, None) == "missing_right_detection"


def test_nonidentity_selected_left_attitude_is_shared_by_ray_and_origin() -> None:
    def transpose(matrix):
        return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))

    def matrix_vector(matrix, vector):
        return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))

    def yaw_matrix(angle_deg):
        angle = radians(angle_deg)
        return (
            (cos(angle), -sin(angle), 0.0),
            (sin(angle), cos(angle), 0.0),
            (0.0, 0.0, 1.0),
        )

    camera_to_body = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    body_to_camera = transpose(camera_to_body)
    left_yaw_deg = 30.0
    right_yaw_deg = 20.0
    left_world_from_body = yaw_matrix(left_yaw_deg)
    right_world_from_body = yaw_matrix(right_yaw_deg)
    baseline_ned = (0.2, 0.3, -0.1)
    translation_left_rdf = matrix_vector(
        body_to_camera,
        matrix_vector(transpose(left_world_from_body), baseline_ned),
    )
    expected_point = (1.2, 0.4, 12.0)
    right_to_target_left = tuple(
        expected_point[index] - translation_left_rdf[index] for index in range(3)
    )
    right_ray_camera = matrix_vector(
        body_to_camera,
        matrix_vector(
            transpose(right_world_from_body),
            matrix_vector(
                left_world_from_body,
                matrix_vector(camera_to_body, right_to_target_left),
            ),
        ),
    )
    left_pixel = (
        640.0 + 900.0 * expected_point[0] / expected_point[2],
        360.0 + 900.0 * expected_point[1] / expected_point[2],
    )
    right_pixel = (
        640.0 + 900.0 * right_ray_camera[0] / right_ray_camera[2],
        360.0 + 900.0 * right_ray_camera[1] / right_ray_camera[2],
    )
    left_source = detection("left", left_pixel[0])
    right_source = detection("right", right_pixel[0])
    left_source = replace(left_source, y1=left_pixel[1] - 10.0, y2=left_pixel[1] + 10.0)
    right_source = replace(right_source, y1=right_pixel[1] - 10.0, y2=right_pixel[1] + 10.0)
    left_q = (cos(radians(left_yaw_deg) / 2.0), 0.0, 0.0, sin(radians(left_yaw_deg) / 2.0))
    right_q = (cos(radians(right_yaw_deg) / 2.0), 0.0, 0.0, sin(radians(right_yaw_deg) / 2.0))
    left_attitude = attitude("left", left_q, timestamp_s=9.7)
    right_attitude = attitude("right", right_q, timestamp_s=9.8)
    left_gps = GpsSnapshot(
        "left", 9.6, 0.0, 0.0, 100.0, 6, 20, 0.01, 0.02,
        "RTK_FIXED", "left", "/left/gps", 8.6,
    )
    right_gps = GpsSnapshot(
        "right",
        9.65,
        degrees(baseline_ned[0] / EARTH_RADIUS_M),
        degrees(baseline_ned[1] / EARTH_RADIUS_M),
        100.0 - baseline_ned[2],
        6,
        20,
        0.01,
        0.02,
        "RTK_FIXED",
        "right",
        "/right/gps",
        8.65,
    )
    baseline = build_pixhawk_baseline_snapshot(
        left_gps=left_gps,
        right_gps=right_gps,
        selected_left_quaternion_wxyz=left_q,
        selected_right_quaternion_wxyz=right_q,
        selected_left_attitude_monotonic_s=left_attitude.timestamp_s,
        created_monotonic_s=10.1,
        detection_midpoint_monotonic_s=10.0,
    )
    attempt = replace(
        pixhawk_attempt(),
        left_detection=DetectionSnapshot.from_observation(
            left_source, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
        ),
        right_detection=DetectionSnapshot.from_observation(
            right_source, pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
        ),
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
        baseline=baseline,
    )

    geometry = calculate_measurement_geometry(attempt)

    assert attempt.baseline.left_basis_quaternion_wxyz == left_attitude.quaternion_wxyz
    assert attempt.baseline.left_basis_attitude_monotonic_s == left_attitude.timestamp_s
    assert geometry.triangulated is not None
    error_m = sum(
        (actual - expected) ** 2
        for actual, expected in zip(geometry.triangulated.point_left_rdf_m, expected_point)
    ) ** 0.5
    assert error_m <= 1e-9
    assert geometry.triangulated.output_frame == "left_camera_rdf"
    assert attempt.measurement_unit == "m"


def test_checkerboard_attempt_keeps_not_used_inputs_and_golden_point() -> None:
    source = calibration_result(checkerboard=True)
    intrinsics = StereoIntrinsicsSnapshot.from_result(
        source, active_zoom_level=8, selected_profile_path="zoom_08.json"
    )
    baseline = build_checkerboard_baseline_snapshot(intrinsics)
    template = pixhawk_attempt()
    checkerboard = replace(
        template,
        triangulation_source="checkerboard",
        left_detection=DetectionSnapshot.from_observation(
            detection("left", 640.0), pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
        ),
        right_detection=DetectionSnapshot.from_observation(
            detection("right", 613.0), pairing_timestamp_basis="detection", pairing_timestamp_s=10.0
        ),
        left_attitude=None,
        right_attitude=None,
        left_gps=None,
        right_gps=None,
        intrinsics=intrinsics,
        baseline=baseline,
    )

    geometry = calculate_measurement_geometry(checkerboard)

    assert checkerboard.left_attitude is None and checkerboard.right_attitude is None
    assert checkerboard.left_gps is None and checkerboard.right_gps is None
    assert checkerboard.baseline.fix_quality == "NOT_USED"
    assert geometry.triangulated is not None
    assert geometry.triangulated.point_left_rdf_m == pytest.approx((0.0, 0.0, 10.0), abs=1e-9)
    assert geometry.triangulated.range_m == pytest.approx(10.0, abs=1e-9)


@pytest.mark.parametrize(
    "factory",
    (
        lambda: replace(attitude("left"), quaternion_wxyz=(0.0, 0.0, 0.0, 0.0)),
        lambda: GpsSnapshot(
            "left", 10.0, float("nan"), 0.0, 100.0, 6, 20, 0.01, 0.02,
            "RTK_FIXED", "left", "/gps", 9.0,
        ),
        lambda: CameraIntrinsicsSnapshot(
            1280,
            720,
            ((900.0, 0.0, 640.0), (0.0, 900.0, 360.0), (0.0, 0.0, 1.0)),
            (),
            0.2,
            20,
        ),
        lambda: CheckerboardExtrinsicsSnapshot(
            ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (-0.3, 0.0, 0.0),
            0.1,
            10,
            "opencv-rdf",
        ),
        lambda: CheckerboardExtrinsicsSnapshot(
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (-0.3, 0.0, 0.0),
            0.1,
            10,
            "ned",
        ),
        lambda: replace(
            pixhawk_attempt(),
            measurement_frame="baseline_center_frd",
        ),
        lambda: replace(
            pixhawk_attempt(),
            baseline=replace(pixhawk_attempt().baseline, translation_frame="ned"),
        ),
    ),
)
def test_attempt_inputs_fail_closed_on_invalid_semantics(factory) -> None:
    with pytest.raises(ValueError):
        factory()
