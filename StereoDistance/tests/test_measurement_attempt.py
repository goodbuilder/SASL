from math import degrees
from types import SimpleNamespace

from wiris_stereo_distance.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
    StereoIntrinsicsResult,
)
from wiris_stereo_distance.measurement_attempt import (
    AttitudeSnapshot,
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
from wiris_stereo_distance.stereo import DetectionObservation


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
    before = (hash(detection_snapshot), hash(intrinsics_snapshot), hash(gps_snapshot))

    metadata["rtp_timestamp"] = 999
    metadata["ignored"].append(3)
    camera_matrix[0][0] = 1.0
    distortion.append(99.0)
    raw_message["mutable"].append(3)

    assert detection_snapshot.timestamp_metadata["rtp_timestamp"] == 123
    assert intrinsics_snapshot.left.camera_matrix[0][0] == 900.0
    assert intrinsics_snapshot.left.distortion_coefficients == (0.01, -0.02)
    assert not hasattr(gps_snapshot, "raw")
    assert before == (hash(detection_snapshot), hash(intrinsics_snapshot), hash(gps_snapshot))


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
