"""Shared synthetic Measurement inputs built with the real solver path."""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
from typing import Any

from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    BaselineSnapshot,
    CameraIntrinsicsSnapshot,
    CheckerboardExtrinsicsSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
)
from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    PIXHAWK_RTK_LIVE,
)
from Workstation.workstation_core.method_evaluation import (
    FrozenMeasurementPairSnapshot,
    MethodEvaluationOutcome,
    MethodEvaluationRequest,
    evaluate_method_request,
)
from Workstation.workstation_core.stereo import DetectionCandidate, DetectionObservation

MEASUREMENT_ID = "run-abc123"
STARTED_LOCAL = datetime(2026, 9, 6, 14, 30, 5, tzinfo=timezone.utc)


def detection_snapshot(
    camera: str,
    center_x: float,
    timestamp_s: float,
    *,
    center_y: float = 240.0,
    frame_key: str = "",
    selected_index: int = 0,
) -> DetectionSnapshot:
    app_ns = int(round(timestamp_s * 1_000_000_000))
    return DetectionSnapshot(
        camera=camera,
        app_receive_monotonic_s=timestamp_s,
        width_px=640,
        height_px=480,
        bbox_xyxy_px=(center_x - 5.0, center_y - 5.0, center_x + 5.0, center_y + 5.0),
        center_px=(center_x, center_y),
        confidence=0.95,
        class_id=1,
        class_name="target",
        pairing_timestamp_basis="app_receive_monotonic",
        pairing_timestamp_s=timestamp_s,
        timestamp_metadata_items=(
            ("frame_key", frame_key or f"{camera}:1:{app_ns}"),
            ("frame_id", 7),
            ("app_receive_monotonic_ns", app_ns),
            ("stream_generation", 1),
        ),
        selected_candidate_index=selected_index,
    )


def detection_observation(
    camera: str,
    center_x: float,
    timestamp_s: float,
    *,
    candidates: tuple[tuple[float, float, int], ...] = (),
    frame_key: str = "",
) -> DetectionObservation:
    app_ns = int(round(timestamp_s * 1_000_000_000))
    boxes = tuple(
        DetectionCandidate(
            index=index,
            x1=cx - 5.0,
            y1=235.0,
            x2=cx + 5.0,
            y2=245.0,
            confidence=confidence,
            class_id=class_id,
            class_name="target",
        )
        for index, (cx, confidence, class_id) in enumerate(candidates or ((center_x, 0.95, 1),))
    )
    best = max(boxes, key=lambda item: item.confidence)
    return DetectionObservation(
        camera=camera,
        timestamp_s=timestamp_s,
        width=640,
        height=480,
        x1=best.x1,
        y1=best.y1,
        x2=best.x2,
        y2=best.y2,
        confidence=best.confidence,
        class_id=best.class_id,
        class_name="target",
        timestamp_metadata={
            "frame_key": frame_key or f"{camera}:1:{app_ns}",
            "frame_id": 7,
            "app_receive_monotonic_ns": app_ns,
            "stream_generation": 1,
        },
        candidates=boxes,
        selected_candidate_index=best.index,
    )


def intrinsics(
    *,
    translation_left_to_right_m: tuple[float, float, float] = (-0.4, 0.0, 0.0),
    distortion: tuple[float, ...] = (0.01, -0.002, 0.0001, -0.0002, 0.0),
) -> StereoIntrinsicsSnapshot:
    camera = CameraIntrinsicsSnapshot(
        width_px=640,
        height_px=480,
        camera_matrix=((900.0, 0.0, 320.0), (0.0, 901.5, 240.25), (0.0, 0.0, 1.0)),
        distortion_coefficients=distortion,
        reprojection_error_px=0.2,
        image_count=20,
    )
    extrinsics = CheckerboardExtrinsicsSnapshot(
        rotation_left_to_right=(
            (0.9998476951563913, 0.0, 0.01745240643728351),
            (0.0, 1.0, 0.0),
            (-0.01745240643728351, 0.0, 0.9998476951563913),
        ),
        translation_left_to_right_m=translation_left_to_right_m,
        reprojection_error_px=0.3,
        pair_count=20,
        frame_contract=(
            "opencv_optical:RDF(right,down,forward); "
            "X_right = R_left_to_right * X_left + T_left_to_right_m"
        ),
    )
    return StereoIntrinsicsSnapshot(
        left=camera,
        right=camera,
        checkerboard_extrinsics=extrinsics,
        active_zoom_level=1,
        selected_profile_path="calibration/profile.json",
        content_sha256="a" * 64,
    )


def checkerboard_attempt(
    pair_sequence: int = 1,
    *,
    left_x: float = 340.0,
    right_x: float = 300.0,
    timestamp_s: float = 1.0,
    intrinsics_snapshot: StereoIntrinsicsSnapshot | None = None,
) -> MeasurementAttempt:
    return MeasurementAttempt(
        attempt_id=pair_sequence,
        attempt_session_id=MEASUREMENT_ID,
        detection_pair_key=("pair", pair_sequence),
        created_monotonic_s=timestamp_s + 0.02,
        created_wall_s=1_757_000_000.0 + timestamp_s,
        detection_midpoint_monotonic_s=timestamp_s + 0.005,
        timestamp_basis="app_receive_monotonic",
        triangulation_source="checkerboard",
        left_detection=detection_snapshot("left", left_x, timestamp_s),
        right_detection=detection_snapshot("right", right_x, timestamp_s + 0.01),
        left_attitude=None,
        right_attitude=None,
        left_gps=None,
        right_gps=None,
        intrinsics=intrinsics_snapshot or intrinsics(),
        baseline=None,
        geometry_config=GeometryConfigSnapshot(
            focal_length_px=900.0,
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
        measurement_method_id=CHECKERBOARD_STEREO,
        measurement_method_instance_id="checkerboard:profile-a",
        measurement_component_sources=(("rotation", "checkerboard_profile"),),
    )


def pixhawk_attempt(pair_sequence: int = 1, *, timestamp_s: float = 1.0) -> MeasurementAttempt:
    left_attitude = AttitudeSnapshot(
        camera="left",
        app_receive_monotonic_s=timestamp_s,
        quaternion_wxyz=(0.9990482215818578, 0.0, 0.04361938736533599, 0.0),
        namespace="/left",
        topic="mavlink/ATTITUDE",
        px4_timestamp_s=0.9,
        roll_deg=0.0,
        pitch_deg=5.0,
        yaw_deg=0.0,
        px4_timestamp_us=900_000,
        mavlink_source_system=1,
        mavlink_source_component=1,
        mavlink_sequence=42,
        mavlink_message_id=30,
    )
    right_attitude = replace(
        left_attitude,
        camera="right",
        namespace="/right",
        quaternion_wxyz=(0.9986295347545738, 0.0, 0.05233595624294383, 0.0),
        pitch_deg=6.0,
    )
    left_gps = GpsSnapshot(
        camera="left",
        app_receive_monotonic_s=timestamp_s,
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
        eph_m=0.01,
        epv_m=0.02,
        rtk_label="RTK_FIXED",
        namespace="/left",
        topic="mavlink/GPS_RAW_INT",
        px4_timestamp_s=0.9,
        gps_utc_us=1_757_000_000_123_456,
    )
    right_gps = replace(left_gps, camera="right", namespace="/right", longitude_deg=127.0000045)
    baseline = BaselineSnapshot(
        source="gps-rtk-live",
        baseline_m=0.4,
        ned_right_from_left_m=(0.0, 0.4, 0.0),
        translation_right_in_left_rdf_m=(0.4, 0.0, 0.0),
        left_basis_quaternion_wxyz=left_attitude.quaternion_wxyz,
        fix_quality="RTK_FIXED",
        warnings=(),
        usable_for_translation=True,
        unusable_reason="",
        gps_pair_delta_s=0.0,
        left_age_at_capture_s=0.02,
        right_age_at_capture_s=0.02,
        left_age_from_detection_midpoint_s=0.005,
        right_age_from_detection_midpoint_s=0.005,
        left_basis_attitude_monotonic_s=left_attitude.timestamp_s,
    )
    base = checkerboard_attempt(pair_sequence, timestamp_s=timestamp_s)
    return replace(
        base,
        triangulation_source="pixhawk",
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
        baseline=baseline,
        left_attitude_age_s=0.0,
        right_attitude_age_s=0.0,
        attitude_pair_delta_s=0.0,
        relative_roll_deg=0.0,
        relative_pitch_deg=1.0,
        relative_yaw_deg=0.0,
        measurement_method_id=PIXHAWK_RTK_LIVE,
        measurement_method_instance_id="live:pixhawk-rtk",
        measurement_component_sources=(
            ("left.roll_deg", "live_pixhawk"),
            ("left.north_m", "live_gnss"),
        ),
    )


def frozen_snapshot(attempt: MeasurementAttempt) -> FrozenMeasurementPairSnapshot:
    from dataclasses import asdict

    return FrozenMeasurementPairSnapshot(
        measurement_id=MEASUREMENT_ID,
        pair_id=f"{MEASUREMENT_ID}:{attempt.attempt_id:020d}",
        pair_sequence=attempt.attempt_id,
        created_monotonic_ns=int(attempt.created_monotonic_s * 1e9),
        created_unix_ns=int(attempt.created_wall_s * 1e9),
        timestamp_basis="app_receive_monotonic",
        left_detection=asdict(attempt.left_detection),
        right_detection=asdict(attempt.right_detection),
        left_attitude=None if attempt.left_attitude is None else asdict(attempt.left_attitude),
        right_attitude=None if attempt.right_attitude is None else asdict(attempt.right_attitude),
        left_gps=None if attempt.left_gps is None else asdict(attempt.left_gps),
        right_gps=None if attempt.right_gps is None else asdict(attempt.right_gps),
        capture_sync={"pair_delta_s": 0.01, "time_basis": "app_receive_monotonic"},
    )


def request_for(attempt: MeasurementAttempt, snapshot: FrozenMeasurementPairSnapshot) -> MethodEvaluationRequest:
    return MethodEvaluationRequest(
        measurement_id=snapshot.measurement_id,
        pair_id=snapshot.pair_id,
        pair_sequence=snapshot.pair_sequence,
        snapshot_sha256=snapshot.canonical_sha256(),
        request_generation=snapshot.pair_sequence,
        method_id=attempt.measurement_method_id,
        method_instance_id=attempt.measurement_method_instance_id,
        attempt_payload=attempt.canonical_payload(),
        enqueued_monotonic_ns=1_000,
    )


def outcome_for(attempt: MeasurementAttempt, snapshot: FrozenMeasurementPairSnapshot) -> MethodEvaluationOutcome:
    return evaluate_method_request(request_for(attempt, snapshot), worker_started_monotonic_ns=2_000)


def px4_row(side: str, sample_type: str, sequence: int, **values: Any) -> dict[str, Any]:
    row = {
        "sequence": sequence,
        "session_id": MEASUREMENT_ID,
        "measurement_id": MEASUREMENT_ID,
        "side": side,
        "sample_type": sample_type,
        "row_contract": "one row per PX4 telemetry callback received by the GUI",
        "sample_monotonic_ns": 1_000_000_000 + sequence * 20_000_000,
        "host_receive_utc_ns": 1_757_000_000_000_000_000 + sequence * 20_000_000,
        "session_utc_ns": 1_757_000_000_000_000_000 + sequence * 20_000_000,
        "source_utc_ns": None,
        "namespace": f"/{side}",
        "topic": f"mavlink/{sample_type}",
        "mavlink_source_system": 1,
        "mavlink_source_component": 1,
        "mavlink_sequence": sequence,
        "mavlink_message_id": 30,
    }
    row.update(values)
    return row
