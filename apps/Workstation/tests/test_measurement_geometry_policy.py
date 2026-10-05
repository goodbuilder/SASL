"""Poor-quality stereo pairs are measured and kept; only undefined geometry is N/A."""

from __future__ import annotations

import math

import pytest

from Workstation.workstation_core.canonical_transform import (
    kernel_inputs_from_right_from_left,
    right_from_left_from_kernel_inputs,
    rotation_left_from_right_from_body_quaternions,
)
from Workstation.workstation_core.measurement_attempt import (
    CameraIntrinsicsSnapshot,
    CheckerboardExtrinsicsSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
)
from Workstation.workstation_core.measurement_controller import (
    calculate_measurement_geometry,
)
from Workstation.workstation_core.method_evaluation import (
    MethodEvaluationRequest,
    evaluate_method_request,
)
from Workstation.workstation_core.triangulation import (
    invert_left_to_right_camera_transform,
)


def _detection(
    camera: str,
    center_x: float,
    timestamp_s: float,
    *,
    center_y: float = 240.0,
    class_id: int = 1,
    candidate_class_ids: tuple[int, ...] = (),
) -> DetectionSnapshot:
    return DetectionSnapshot(
        camera=camera,
        app_receive_monotonic_s=timestamp_s,
        width_px=640,
        height_px=480,
        bbox_xyxy_px=(center_x - 5.0, center_y - 5.0, center_x + 5.0, center_y + 5.0),
        center_px=(center_x, center_y),
        confidence=0.95,
        class_id=class_id,
        class_name="target",
        pairing_timestamp_basis="app_receive_monotonic",
        pairing_timestamp_s=timestamp_s,
        candidate_class_ids=candidate_class_ids,
    )


def _intrinsics(translation_left_to_right_m=(-0.4, 0.0, 0.0)) -> StereoIntrinsicsSnapshot:
    camera = CameraIntrinsicsSnapshot(
        width_px=640,
        height_px=480,
        camera_matrix=((900.0, 0.0, 320.0), (0.0, 900.0, 240.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        reprojection_error_px=0.2,
        image_count=20,
    )
    extrinsics = CheckerboardExtrinsicsSnapshot(
        rotation_left_to_right=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
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


def _attempt(
    *,
    left: DetectionSnapshot | None = None,
    right: DetectionSnapshot | None = None,
    intrinsics: StereoIntrinsicsSnapshot | None = None,
) -> MeasurementAttempt:
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="measurement-1",
        detection_pair_key=("pair", 1),
        created_monotonic_s=1.02,
        created_wall_s=2.0,
        detection_midpoint_monotonic_s=1.005,
        timestamp_basis="app_receive_monotonic",
        triangulation_source="checkerboard",
        left_detection=left or _detection("left", 340.0, 1.0),
        right_detection=right or _detection("right", 300.0, 1.01),
        left_attitude=None,
        right_attitude=None,
        left_gps=None,
        right_gps=None,
        intrinsics=intrinsics or _intrinsics(),
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
        measurement_method_id="checkerboard_stereo",
        measurement_method_instance_id="checkerboard:profile-a",
        measurement_component_sources=(("rotation", "checkerboard_profile"),),
    )


def _request(attempt: MeasurementAttempt) -> MethodEvaluationRequest:
    return MethodEvaluationRequest(
        measurement_id="measurement-1",
        pair_id="measurement-1:00000000000000000001",
        pair_sequence=1,
        snapshot_sha256="b" * 64,
        request_generation=1,
        method_id="checkerboard_stereo",
        method_instance_id="checkerboard:profile-a",
        attempt_payload=attempt.canonical_payload(),
        enqueued_monotonic_ns=1_000,
    )


def test_good_pair_is_valid_with_positive_depths() -> None:
    geometry = calculate_measurement_geometry(_attempt())

    assert geometry.triangulated is not None
    assert geometry.triangulated.left_depth_m > 0.0
    assert geometry.triangulated.right_depth_m > 0.0
    assert geometry.triangulated.range_m == pytest.approx(9.0, rel=1e-3)
    assert geometry.triangulation_reason == "ok"


def test_negative_depth_is_kept_as_signed_evidence() -> None:
    # Left target appears left of the right target: rays cross behind the rig.
    geometry = calculate_measurement_geometry(
        _attempt(left=_detection("left", 300.0, 1.0), right=_detection("right", 340.0, 1.01))
    )

    assert geometry.triangulated is not None
    assert geometry.triangulated.left_depth_m < 0.0
    assert geometry.triangulated.right_depth_m < 0.0
    assert geometry.estimate is not None
    assert math.isfinite(geometry.estimate.distance_m)
    outcome = evaluate_method_request(_request(_attempt(
        left=_detection("left", 300.0, 1.0), right=_detection("right", 340.0, 1.01)
    )), worker_started_monotonic_ns=2_000)
    assert outcome.status in {"VALID", "ESTIMATED"}
    assert outcome.result["triangulated"]["range_m"] == pytest.approx(
        geometry.triangulated.range_m
    )


def test_large_ray_gap_from_vertical_delta_is_kept() -> None:
    geometry = calculate_measurement_geometry(
        _attempt(
            left=_detection("left", 340.0, 1.0, center_y=100.0),
            right=_detection("right", 300.0, 1.01, center_y=400.0),
        )
    )

    assert geometry.match is not None
    assert geometry.match.vertical_delta_px == 300.0
    assert geometry.triangulated is not None
    assert geometry.triangulated.closest_gap_m > 0.3
    assert any("vertical bbox delta" in warning for warning in geometry.warnings)


def test_large_time_delta_is_kept_with_warning() -> None:
    geometry = calculate_measurement_geometry(
        _attempt(left=_detection("left", 340.0, 1.0), right=_detection("right", 300.0, 6.0))
    )

    assert geometry.match is not None
    assert geometry.match.time_delta_s == pytest.approx(5.0)
    assert geometry.triangulated is not None
    assert any("camera pair delta" in warning for warning in geometry.warnings)


def test_multiple_candidates_are_kept_with_warnings() -> None:
    attempt = _attempt(
        left=_detection("left", 340.0, 1.0, candidate_class_ids=(1, 1, 3)),
        right=_detection("right", 300.0, 1.01, candidate_class_ids=(1, 1)),
    )
    geometry = calculate_measurement_geometry(attempt)

    assert geometry.triangulated is not None
    assert any(warning.startswith("ambiguous_left_candidates") for warning in geometry.warnings)
    assert any(warning.startswith("ambiguous_right_candidates") for warning in geometry.warnings)
    outcome = evaluate_method_request(_request(attempt), worker_started_monotonic_ns=2_000)
    assert outcome.status == "ESTIMATED"
    assert outcome.result["triangulated"] is not None


def test_class_mismatch_is_a_hard_gate_with_reason() -> None:
    attempt = _attempt(
        left=_detection("left", 340.0, 1.0, class_id=1),
        right=_detection("right", 300.0, 1.01, class_id=2),
    )
    geometry = calculate_measurement_geometry(attempt)
    outcome = evaluate_method_request(_request(attempt), worker_started_monotonic_ns=2_000)

    assert geometry.match is None
    assert geometry.triangulated is None
    assert geometry.triangulation_reason == "correspondence_class_mismatch"
    assert outcome.status == "BLOCKED"
    assert outcome.reasons == ("correspondence_class_mismatch",)


def test_zero_disparity_is_still_computed_when_rays_are_not_parallel() -> None:
    geometry = calculate_measurement_geometry(
        _attempt(
            left=_detection("left", 320.0, 1.0, center_y=100.0),
            right=_detection("right", 320.0, 1.01, center_y=300.0),
        )
    )

    assert geometry.match is not None
    assert geometry.match.disparity_px == 0.0
    assert geometry.triangulated is not None


def test_zero_baseline_leaves_result_empty_but_keeps_inputs_and_reason() -> None:
    attempt = _attempt(intrinsics=_intrinsics(translation_left_to_right_m=(0.0, 0.0, 0.0)))

    geometry = calculate_measurement_geometry(attempt)
    outcome = evaluate_method_request(_request(attempt), worker_started_monotonic_ns=2_000)

    assert geometry.match is not None
    assert geometry.triangulated is None
    assert geometry.triangulation_reason == "zero_baseline"
    assert outcome.status == "BLOCKED"
    assert outcome.reasons == ("zero_baseline",)
    assert tuple(outcome.attempt_payload["left_detection"]["center_px"]) == (340.0, 240.0)
    assert outcome.result["match"]["disparity_px"] == 40.0


def test_exactly_parallel_rays_are_rank_deficient_not_dropped() -> None:
    attempt = _attempt(left=_detection("left", 320.0, 1.0), right=_detection("right", 320.0, 1.01))

    geometry = calculate_measurement_geometry(attempt)
    outcome = evaluate_method_request(_request(attempt), worker_started_monotonic_ns=2_000)

    assert geometry.match is not None
    assert geometry.triangulated is None
    assert geometry.triangulation_reason == "rank_deficient"
    assert outcome.status == "BLOCKED"
    assert outcome.reasons == ("rank_deficient",)
    assert outcome.result["match"] is not None


def test_canonical_transform_round_trips_kernel_inputs() -> None:
    rotation_left_to_right = (
        (0.9998476951563913, 0.0, 0.01745240643728351),
        (0.0, 1.0, 0.0),
        (-0.01745240643728351, 0.0, 0.9998476951563913),
    )
    translation_left_to_right_m = (-0.4, 0.01, 0.002)
    kernel_rotation, kernel_origin = invert_left_to_right_camera_transform(
        [list(row) for row in rotation_left_to_right],
        translation_left_to_right_m,
    )

    canonical_rotation, canonical_translation = right_from_left_from_kernel_inputs(
        kernel_rotation,
        kernel_origin,
    )
    for row in range(3):
        for column in range(3):
            assert canonical_rotation[row][column] == pytest.approx(
                rotation_left_to_right[row][column], abs=1e-15
            )
    assert canonical_translation == pytest.approx(translation_left_to_right_m, abs=1e-15)

    back_rotation, back_origin = kernel_inputs_from_right_from_left(
        canonical_rotation,
        canonical_translation,
    )
    for row in range(3):
        assert back_rotation[row] == pytest.approx(tuple(kernel_rotation[row]), abs=1e-15)
    assert back_origin == pytest.approx(kernel_origin, abs=1e-15)


def test_identical_body_attitudes_give_identity_camera_rotation() -> None:
    rotation = rotation_left_from_right_from_body_quaternions(
        (1.0, 0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    )

    for row in range(3):
        for column in range(3):
            assert rotation[row][column] == pytest.approx(1.0 if row == column else 0.0, abs=1e-15)
