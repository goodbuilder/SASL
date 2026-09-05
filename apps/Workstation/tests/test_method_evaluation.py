from __future__ import annotations

import json
import multiprocessing
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest
from Workstation.measurement_view_controller import MeasurementViewController
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
    build_pixhawk_baseline_snapshot,
)
from Workstation.workstation_core.baseline_yaw import (
    quaternion_from_roll_pitch_yaw_deg,
    select_baseline_yaw,
)
from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS,
    MOUNTAIN_GNSS,
    PIXHAWK_RTK_LIVE,
    PIXHAWK_RP_GPS_BASELINE_YAW,
    MeasurementMethodStatus,
)
from Workstation.workstation_core.method_evaluation import (
    METHOD_EVALUATION_QUEUE_LIMIT,
    FrozenMeasurementPairSnapshot,
    MethodEvaluationBundle,
    MethodEvaluationOutcome,
    MethodEvaluationRequest,
    evaluate_method_request,
)
from Workstation.workstation_core.method_evaluation_workers import (
    MethodEvaluationCoordinator,
    MethodEvaluationPoll,
)
from Workstation.workstation_core.preview_measurement_worker import (
    PreviewMeasurementCoordinator,
    PreviewMeasurementResult,
)
from Workstation.workstation_core.measurement_controller import (
    calculate_measurement_geometry,
)
from Workstation.workstation_core.stereo import stereo_estimate_from_match
from Workstation.workstation_core.measurement_data_store import (
    ROW_WRITE_ADMITTED,
    ROW_WRITE_UNAVAILABLE,
    LiveDataRows,
)
from Workstation.workstation_core.recording_reader import read_measurement_stream
from Workstation.workstation_core.recording_session import (
    SpoolRows,
    _measurement_review_mapping,
    load_retryable_session,
    save_session_files,
    write_recovery_metadata,
)


def _detection(camera: str, center_x: float, timestamp_s: float) -> DetectionSnapshot:
    return DetectionSnapshot(
        camera=camera,
        app_receive_monotonic_s=timestamp_s,
        width_px=640,
        height_px=480,
        bbox_xyxy_px=(center_x - 5.0, 235.0, center_x + 5.0, 245.0),
        center_px=(center_x, 240.0),
        confidence=0.95,
        class_id=1,
        class_name="target",
        pairing_timestamp_basis="app_receive_monotonic",
        pairing_timestamp_s=timestamp_s,
    )


def _intrinsics() -> StereoIntrinsicsSnapshot:
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
        translation_left_to_right_m=(-0.4, 0.0, 0.0),
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


def _attempt() -> MeasurementAttempt:
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="measurement-1",
        detection_pair_key=("pair", 1),
        created_monotonic_s=1.02,
        created_wall_s=2.0,
        detection_midpoint_monotonic_s=1.005,
        timestamp_basis="app_receive_monotonic",
        triangulation_source="checkerboard",
        left_detection=_detection("left", 340.0, 1.0),
        right_detection=_detection("right", 300.0, 1.01),
        left_attitude=None,
        right_attitude=None,
        left_gps=None,
        right_gps=None,
        intrinsics=_intrinsics(),
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


def _request(
    method_id: str = "checkerboard_stereo",
    *,
    attempt_payload: dict[str, object] | None = None,
    blocked_reasons: tuple[str, ...] = (),
    pair_sequence: int = 1,
    request_generation: int = 3,
) -> MethodEvaluationRequest:
    return MethodEvaluationRequest(
        measurement_id="measurement-1",
        pair_id=f"measurement-1:{pair_sequence:020d}",
        pair_sequence=pair_sequence,
        snapshot_sha256="b" * 64,
        request_generation=request_generation,
        method_id=method_id,
        method_instance_id=f"{method_id}:frozen-1",
        attempt_payload=attempt_payload,
        blocked_reasons=blocked_reasons,
        enqueued_monotonic_ns=1_000,
    )


def _pixhawk_fallback_attempt() -> MeasurementAttempt:
    left_attitude = AttitudeSnapshot(
        camera="left",
        app_receive_monotonic_s=1.0,
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        namespace="/left",
        topic="vehicle_attitude",
        px4_timestamp_s=0.9,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )
    right_attitude = replace(left_attitude, camera="right", namespace="/right")
    left_gps = GpsSnapshot(
        camera="left",
        app_receive_monotonic_s=1.0,
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
        eph_m=0.01,
        epv_m=0.02,
        rtk_label="RTK_FIXED",
        namespace="/left",
        topic="vehicle_gps_position",
        px4_timestamp_s=0.9,
    )
    right_gps = replace(
        left_gps,
        camera="right",
        namespace="/right",
        longitude_deg=127.0000045,
    )
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
    base = _attempt()
    return replace(
        base,
        triangulation_source="pixhawk",
        timestamp_basis="app_receive_monotonic",
        requested_timestamp_basis="camera_rtcp",
        timing_fallback_reason="RTCP_TIMESTAMP_UNAVAILABLE",
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
        baseline=baseline,
        left_attitude_age_s=0.0,
        right_attitude_age_s=0.0,
        attitude_pair_delta_s=0.0,
        relative_roll_deg=0.0,
        relative_pitch_deg=0.0,
        relative_yaw_deg=0.0,
        measurement_method_id=PIXHAWK_RTK_LIVE,
        measurement_method_instance_id="live:pixhawk-rtk",
        measurement_component_sources=(
            ("rotation", "live_pixhawk"),
            ("translation", "live_gnss"),
        ),
    )


def test_preview_geometry_and_optical_math_run_in_latest_only_worker() -> None:
    coordinator = PreviewMeasurementCoordinator()
    attempt = _attempt()
    try:
        coordinator.submit(
            key=attempt.canonical_sha256(),
            attempt=attempt,
            optical_payload={
                "left_center": (340.0, 240.0),
                "right_center": (300.0, 238.0),
                "expected_dy_px": 0.0,
                "mean_fy_px": 900.0,
                "mean_fx_px": 900.0,
                "target_distance_m": 9.0,
            },
        )
        deadline = time.monotonic() + 2.0
        result = None
        while time.monotonic() < deadline and result is None:
            result = coordinator.poll_latest()
            time.sleep(0.005)
        assert result is not None
        assert result.error == ""
        assert result.geometry_result is not None
        assert result.optical_metrics["observed_dx_px"] == 40.0
        assert result.optical_metrics["observed_dy_px"] == 2.0
        assert result.optical_metrics["vertical_error_mrad"] == pytest.approx(
            2.0 / 900.0 * 1000.0
        )
        assert coordinator.metrics()["request_capacity"] == 1
    finally:
        assert coordinator.close()


def test_px4_attitude_distance_remains_valid_on_app_receive_timing_fallback() -> None:
    attempt = _pixhawk_fallback_attempt()

    outcome = evaluate_method_request(
        _request(
            method_id=PIXHAWK_RTK_LIVE,
            attempt_payload=attempt.canonical_payload(),
        ),
        worker_started_monotonic_ns=2_000,
    )

    assert outcome.status == "VALID"
    assert outcome.result["triangulated"]["range_m"] == pytest.approx(9.0, rel=1e-3)
    assert all("RTCP" not in warning for warning in outcome.warnings)


def test_px4_attitude_distance_uses_frozen_healthy_rtcp_pair() -> None:
    base = _pixhawk_fallback_attempt()

    def rtcp_detection(
        detection: DetectionSnapshot,
        ntp_s: float,
    ) -> DetectionSnapshot:
        return replace(
            detection,
            pairing_timestamp_basis="camera_rtcp",
            pairing_timestamp_s=ntp_s,
            timestamp_metadata_items=(
                *detection.timestamp_metadata_items,
                ("rtcp_frame_ntp_unix_s", ntp_s),
                ("clock_epoch_id", "epoch-1"),
                ("clock_epoch_state", "current"),
            ),
        )

    attempt = replace(
        base,
        timestamp_basis="camera_rtcp",
        timing_fallback_reason="",
        left_detection=rtcp_detection(base.left_detection, 1_000.000),
        right_detection=rtcp_detection(base.right_detection, 1_000.010),
        geometry_config=replace(base.geometry_config, timestamp_basis="camera_rtcp"),
    )

    outcome = evaluate_method_request(
        _request(
            method_id=PIXHAWK_RTK_LIVE,
            attempt_payload=attempt.canonical_payload(),
        ),
        worker_started_monotonic_ns=2_000,
    )

    assert outcome.status == "VALID"
    assert outcome.result["match"]["time_delta_s"] == pytest.approx(0.010)
    assert outcome.result["triangulated"]["range_m"] == pytest.approx(9.0, rel=1e-3)


def test_baseline_yaw_method_uses_effective_rp_and_selected_gps_yaw() -> None:
    base = _pixhawk_fallback_attempt()
    selection = select_baseline_yaw(
        north_m=base.baseline.ned_right_from_left_m[0],
        east_m=base.baseline.ned_right_from_left_m[1],
        p1_yaw_deg=0.0,
        p2_yaw_deg=0.0,
    )
    assert selection.ready
    left_q = quaternion_from_roll_pitch_yaw_deg(
        base.left_attitude.roll_deg,
        base.left_attitude.pitch_deg,
        selection.selected_yaw_deg,
    )
    right_q = quaternion_from_roll_pitch_yaw_deg(
        base.right_attitude.roll_deg,
        base.right_attitude.pitch_deg,
        selection.selected_yaw_deg,
    )
    baseline = build_pixhawk_baseline_snapshot(
        left_gps=base.left_gps,
        right_gps=base.right_gps,
        selected_left_quaternion_wxyz=left_q,
        selected_left_attitude_monotonic_s=base.left_attitude.timestamp_s,
        created_monotonic_s=base.created_monotonic_s,
        detection_midpoint_monotonic_s=base.detection_midpoint_monotonic_s,
    )
    attempt = replace(
        base,
        measurement_method_id=PIXHAWK_RP_GPS_BASELINE_YAW,
        measurement_method_instance_id="live:pixhawk-rp-gps-baseline-yaw",
        measurement_geometry_kind="live_pixhawk_baseline_yaw",
        measurement_geometry_sha256="a" * 64,
        baseline=baseline,
        geometry_left_quaternion_wxyz=left_q,
        geometry_right_quaternion_wxyz=right_q,
        measurement_yaw_source="gnss_baseline_perpendicular_90deg",
        measurement_yaw_selection_mode=selection.mode,
        measurement_yaw_branch_offset_deg=selection.selected_offset_deg,
        measurement_yaw_baseline_azimuth_deg=selection.baseline_azimuth_deg,
        measurement_yaw_selected_deg=selection.selected_yaw_deg,
        measurement_yaw_p1_reference_deg=selection.p1_reference_yaw_deg,
        measurement_yaw_p2_reference_deg=selection.p2_reference_yaw_deg,
        measurement_yaw_score_minus90_deg=selection.score_minus90_deg,
        measurement_yaw_score_plus90_deg=selection.score_plus90_deg,
        measurement_yaw_consensus=selection.consensus,
        measurement_yaw_low_margin=selection.low_margin,
    )

    outcome = evaluate_method_request(
        _request(
            method_id=PIXHAWK_RP_GPS_BASELINE_YAW,
            attempt_payload=attempt.canonical_payload(),
        ),
        worker_started_monotonic_ns=2_000,
    )

    assert outcome.status == "VALID"
    assert outcome.result["triangulated"]["range_m"] == pytest.approx(9.0, rel=1e-3)
    assert outcome.attempt_payload["measurement_yaw_source"] == (
        "gnss_baseline_perpendicular_90deg"
    )


@pytest.mark.parametrize("distance_m", (0.0, -1.25))
def test_finite_non_positive_distance_is_retained_as_measurement_evidence(
    distance_m: float,
) -> None:
    geometry = calculate_measurement_geometry(_attempt())
    assert geometry.match is not None

    estimate = stereo_estimate_from_match(geometry.match, distance_m=distance_m)

    assert estimate is not None
    assert estimate.distance_m == distance_m


@pytest.mark.parametrize("distance_m", (0.0, -1.25))
def test_non_positive_primary_distance_is_committed_without_blocking_other_data(
    distance_m: float,
) -> None:
    primary = _primary_outcome()
    result = dict(primary.result)
    triangulated = dict(result["triangulated"])
    triangulated["range_m"] = distance_m
    estimate = dict(result["estimate"])
    estimate["distance_m"] = distance_m
    primary = replace(
        primary,
        result={**result, "triangulated": triangulated, "estimate": estimate},
    )
    controller, _panel, events, commits = _primary_poll_controller(
        [MethodEvaluationPoll(outcomes=(primary,))]
    )

    controller._poll_method_evaluations()

    assert events == ["commit", "display"]
    assert commits[0]["distance_m"] == distance_m


def test_primary_status_includes_match_advisories_used_by_measurement_row() -> None:
    base = _pixhawk_fallback_attempt()
    attempt = replace(
        base,
        geometry_config=replace(
            base.geometry_config,
            max_time_delta_s=0.001,
            field_quality_gates_advisory=True,
        ),
    )

    outcome = evaluate_method_request(
        _request(
            PIXHAWK_RTK_LIVE,
            attempt_payload=attempt.canonical_payload(),
        ),
        worker_started_monotonic_ns=2_000,
    )

    assert outcome.status == "ESTIMATED"
    assert any("camera pair delta" in warning for warning in outcome.warnings)
    assert outcome.warnings == tuple(outcome.result["estimate"]["warnings"])


def _blocked_requests(
    pair_sequence: int,
    *,
    request_generation: int = 3,
) -> tuple[MethodEvaluationRequest, ...]:
    return tuple(
        _request(
            method_id,
            blocked_reasons=("frozen input unavailable",),
            pair_sequence=pair_sequence,
            request_generation=request_generation,
        )
        for method_id in MEASUREMENT_METHOD_IDS
    )


def _slow_horizon_worker(
    method_id: str,
    _expected_parent_pid: int,
    input_queue: object,
    output_queue: object,
) -> None:
    """Spawn-safe test worker that makes only Horizon exceed its policy."""

    while True:
        payload = input_queue.get()
        if payload is None:
            return
        request = MethodEvaluationRequest.from_mapping(payload)
        started_ns = time.monotonic_ns()
        output_queue.put(
            {
                "kind": "started",
                "measurement_id": request.measurement_id,
                "pair_id": request.pair_id,
                "pair_sequence": request.pair_sequence,
                "snapshot_sha256": request.snapshot_sha256,
                "request_generation": request.request_generation,
                "method_id": request.method_id,
                "worker_started_monotonic_ns": started_ns,
            }
        )
        if method_id == "horizon_hybrid":
            time.sleep(2.0)
        outcome = evaluate_method_request(
            request,
            worker_started_monotonic_ns=started_ns,
        )
        output_queue.put({"kind": "outcome", "outcome": outcome.to_record()})


def _slow_primary_worker(
    method_id: str,
    _expected_parent_pid: int,
    input_queue: object,
    output_queue: object,
) -> None:
    while True:
        payload = input_queue.get()
        if payload is None:
            return
        request = MethodEvaluationRequest.from_mapping(payload)
        started_ns = time.monotonic_ns()
        output_queue.put(
            {
                "kind": "started",
                "measurement_id": request.measurement_id,
                "pair_id": request.pair_id,
                "pair_sequence": request.pair_sequence,
                "snapshot_sha256": request.snapshot_sha256,
                "request_generation": request.request_generation,
                "method_id": request.method_id,
                "worker_started_monotonic_ns": started_ns,
            }
        )
        if method_id == PIXHAWK_RTK_LIVE:
            time.sleep(2.0)
        outcome = evaluate_method_request(
            request,
            worker_started_monotonic_ns=started_ns,
        )
        output_queue.put({"kind": "outcome", "outcome": outcome.to_record()})


def _crashing_horizon_worker(
    method_id: str,
    _expected_parent_pid: int,
    input_queue: object,
    output_queue: object,
) -> None:
    """Spawn-safe test worker whose Horizon process dies mid-evaluation."""

    import os as _os

    while True:
        payload = input_queue.get()
        if payload is None:
            return
        request = MethodEvaluationRequest.from_mapping(payload)
        started_ns = time.monotonic_ns()
        output_queue.put(
            {
                "kind": "started",
                "measurement_id": request.measurement_id,
                "pair_id": request.pair_id,
                "pair_sequence": request.pair_sequence,
                "snapshot_sha256": request.snapshot_sha256,
                "request_generation": request.request_generation,
                "method_id": request.method_id,
                "worker_started_monotonic_ns": started_ns,
            }
        )
        if method_id == "horizon_hybrid":
            _os._exit(3)
        outcome = evaluate_method_request(
            request,
            worker_started_monotonic_ns=started_ns,
        )
        output_queue.put({"kind": "outcome", "outcome": outcome.to_record()})


def _primary_outcome(status: str = "VALID") -> MethodEvaluationOutcome:
    attempt = _pixhawk_fallback_attempt()
    if status == "ESTIMATED":
        attempt = replace(attempt, warnings=("RTK fix advisory",))
    if status == "BLOCKED":
        return evaluate_method_request(
            _request(
                PIXHAWK_RTK_LIVE,
                attempt_payload=attempt.canonical_payload(),
                blocked_reasons=("primary input blocked",),
            ),
            worker_started_monotonic_ns=2_000,
        )
    outcome = evaluate_method_request(
        _request(
            PIXHAWK_RTK_LIVE,
            attempt_payload=attempt.canonical_payload(),
        ),
        worker_started_monotonic_ns=2_000,
    )
    if status in {"FAILED", "TIMEOUT", "BACKPRESSURE_BLOCKED"}:
        return replace(
            outcome,
            status=status,
            result={},
            reasons=(f"primary {status.lower()}",),
        )
    assert outcome.status == status
    return outcome


def _bundle_with_primary(
    primary: MethodEvaluationOutcome,
) -> MethodEvaluationBundle:
    outcomes = []
    for method_id in MEASUREMENT_METHOD_IDS:
        if method_id == PIXHAWK_RTK_LIVE:
            outcomes.append(primary)
            continue
        outcomes.append(
            evaluate_method_request(
                _request(
                    method_id,
                    attempt_payload=None,
                    blocked_reasons=("unused comparison input unavailable",),
                ),
                worker_started_monotonic_ns=2_000,
            )
        )
    return MethodEvaluationBundle(
        measurement_id=primary.measurement_id,
        pair_id=primary.pair_id,
        pair_sequence=primary.pair_sequence,
        snapshot_sha256=primary.snapshot_sha256,
        request_generation=primary.request_generation,
        outcomes=tuple(outcomes),
    )


def _primary_poll_controller(
    polls: list[MethodEvaluationPoll],
    *,
    admissions: tuple[bool, ...] = (True,),
    run_state: str = "RUNNING",
) -> tuple[
    MeasurementViewController,
    SimpleNamespace,
    list[str],
    list[dict[str, object]],
]:
    events: list[str] = []
    commits: list[dict[str, object]] = []
    remaining_admissions = list(admissions)

    def commit_primary(**kwargs):
        events.append("commit")
        commits.append(dict(kwargs))
        admitted = remaining_admissions.pop(0)
        return SimpleNamespace(
            history_added=bool(kwargs["publish_history"]),
            row_commit=SimpleNamespace(enqueue_admitted=admitted),
        )

    panel = SimpleNamespace(
        distance_recording_enabled=True,
        _measurement_session_hierarchy_enabled=False,
        _measurement_run_generation="measurement-1",
        _measurement_id="measurement-1",
        _measurement_run_state=run_state,
        _method_evaluation_committed=set(),
        _measurement_method_result_counters={},
        method_evaluation_bundle_rows=[],
        _commit_primary_distance_outcome=commit_primary,
        _set_distance_text=lambda *_args, **_kwargs: events.append("display"),
        _log=lambda *_args, **_kwargs: None,
    )
    controller = MeasurementViewController(panel)
    controller._method_evaluation_coordinator = SimpleNamespace(
        poll=lambda: polls.pop(0) if polls else MethodEvaluationPoll()
    )
    return controller, panel, events, commits


class _BundleAdmissionStore:
    def __init__(self, admissions: tuple[bool, ...]) -> None:
        self._admissions = list(admissions)
        self.calls: list[dict[str, object]] = []
        self.lifecycle_state = "OPEN"
        self.writer_alive = True

    def enqueue(self, stream, payload, *, priority=None):
        self.calls.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return self._admissions.pop(0)

    def snapshot(self) -> dict[str, object]:
        return {
            "lifecycle_state": self.lifecycle_state,
            "writer_alive": self.writer_alive,
            "streams": {"method_evaluations": {"last_error": ""}},
        }


def test_pure_evaluator_round_trips_a_frozen_valid_attempt() -> None:
    attempt = _attempt()
    outcome = evaluate_method_request(
        _request(attempt_payload=attempt.canonical_payload()),
        worker_started_monotonic_ns=2_000,
    )

    assert outcome.status == "VALID"
    assert outcome.result["triangulated"]["range_m"] == pytest.approx(9.0, rel=1e-3)
    assert outcome.result["estimate"]["sync_quality"] == "GOOD"
    assert outcome.attempt_sha256 == attempt.canonical_sha256()
    assert MethodEvaluationOutcome.from_mapping(outcome.to_record()) == outcome


def test_nonfixed_3d_mountain_warning_persists_as_estimated_outcome() -> None:
    warning = (
        "GNSS + Mountain translation accepted with 3D fix or better without "
        "RTK_FIXED on both sides; the Mountain measurement remains ESTIMATED"
    )
    attempt = replace(
        _attempt(),
        measurement_method_id=MOUNTAIN_GNSS,
        measurement_method_instance_id="frozen-draft:mountain-3d-fix",
        warnings=(warning,),
    )

    outcome = evaluate_method_request(
        _request(MOUNTAIN_GNSS, attempt_payload=attempt.canonical_payload()),
        worker_started_monotonic_ns=2_000,
    )
    record = outcome.to_record()

    assert MEASUREMENT_METHOD_IDS[0] == "pixhawk_rtk_live"
    assert outcome.status == "ESTIMATED"
    assert outcome.warnings == (warning,)
    assert record["method_id"] == MOUNTAIN_GNSS
    assert record["status"] == "ESTIMATED"
    assert record["warnings"] == [warning]


def test_missing_frozen_input_produces_an_explicit_blocked_outcome() -> None:
    outcome = evaluate_method_request(
        _request(attempt_payload=None, blocked_reasons=("frozen comparison input missing",)),
        worker_started_monotonic_ns=2_000,
    )

    assert outcome.status == "BLOCKED"
    assert outcome.reasons == ("frozen comparison input missing",)
    assert outcome.queue_wait_ms == pytest.approx(0.001)


def test_bundle_accepts_canonical_subsets_and_verifies_hashes() -> None:
    base = evaluate_method_request(
        _request(attempt_payload=None, blocked_reasons=("not ready",)),
        worker_started_monotonic_ns=2_000,
    )
    outcomes = tuple(
        replace(
            base,
            method_id=method_id,
            method_instance_id=f"{method_id}:frozen-1",
        )
        for method_id in MEASUREMENT_METHOD_IDS
    )
    bundle = MethodEvaluationBundle(
        measurement_id=base.measurement_id,
        pair_id=base.pair_id,
        pair_sequence=base.pair_sequence,
        snapshot_sha256=base.snapshot_sha256,
        request_generation=base.request_generation,
        outcomes=outcomes,
        planned_methods={"terrain_pnp": "no Terrain PnP candidate"},
    )

    assert MethodEvaluationBundle.from_mapping(bundle.to_record()) == bundle
    assert bundle.primary is outcomes[0]
    with pytest.raises(ValueError, match="canonical order"):
        replace(bundle, outcomes=tuple(reversed(outcomes)))
    with pytest.raises(ValueError, match="duplicate"):
        replace(bundle, outcomes=(outcomes[2], outcomes[2]))
    with pytest.raises(ValueError, match="at least one"):
        replace(bundle, outcomes=())
    tampered = bundle.to_record()
    tampered["outcomes"][0]["status"] = "FAILED"
    with pytest.raises(ValueError, match="hash mismatch"):
        MethodEvaluationBundle.from_mapping(tampered)

    subset = replace(bundle, outcomes=(outcomes[2], outcomes[5]))
    assert subset.primary is None
    assert subset.submitted_method_ids == (outcomes[2].method_id, outcomes[5].method_id)
    assert subset.outcome_for(outcomes[5].method_id) is outcomes[5]
    record = subset.to_record()
    assert record["primary_outcome_sha256"] == ""
    assert record["submitted_method_ids"] == list(subset.submitted_method_ids)
    assert MethodEvaluationBundle.from_mapping(record) == subset


def test_worker_policy_has_no_compute_timeout_and_twenty_slots_per_method() -> None:
    from Workstation.workstation_core import method_evaluation

    assert not hasattr(method_evaluation, "METHOD_EVALUATION_TIMEOUT_S")
    assert METHOD_EVALUATION_QUEUE_LIMIT == 20


def test_frozen_pair_queue_payload_uses_native_paths_and_rejects_image_bytes() -> None:
    snapshot = FrozenMeasurementPairSnapshot(
        measurement_id="measurement-1",
        pair_id="measurement-1:00000000000000000001",
        pair_sequence=1,
        created_monotonic_ns=100,
        created_unix_ns=200,
        timestamp_basis="app_receive_monotonic",
        left_detection={"bbox": [1, 2, 3, 4]},
        right_detection={"bbox": [5, 6, 7, 8]},
        native_frames={
            "native_left_png_path": "frames/left.png",
            "native_left_png_sha256": "a" * 64,
        },
    )
    request = replace(
        _request(blocked_reasons=("not ready",)),
        snapshot_sha256=snapshot.canonical_sha256(),
        snapshot_payload=snapshot.canonical_payload(),
    )

    assert MethodEvaluationRequest.from_mapping(request.to_mapping()) == request
    with pytest.raises(TypeError, match="instead of carrying bytes"):
        FrozenMeasurementPairSnapshot(
            measurement_id="measurement-1",
            pair_id="pair-2",
            pair_sequence=2,
            created_monotonic_ns=100,
            created_unix_ns=200,
            timestamp_basis="app_receive_monotonic",
            left_detection={},
            right_detection={},
            native_frames={"left_image": b"raw pixels"},
        )


def test_ftp_same_command_round_is_recorded_as_operator_sync_assumption() -> None:
    metadata = {
        "capture_id": "hr-001",
        "command_round": "round-7",
        "ftp_downloaded": True,
        "decoded": True,
    }
    pair = SimpleNamespace(
        left=SimpleNamespace(timestamp_metadata=dict(metadata)),
        right=SimpleNamespace(timestamp_metadata=dict(metadata)),
    )

    sync = MeasurementViewController._ftp_command_round_sync(pair)

    assert sync["time_basis"] == "command_round_assumed_sync"
    assert sync["pair_delta_s"] is None
    assert sync["operator_approved_sync_assumption"] is True
    assert "not a measured exposure timestamp" in sync["warning"]


def test_request_fanout_uses_one_snapshot_and_canonical_method_order() -> None:
    snapshot = FrozenMeasurementPairSnapshot(
        measurement_id="measurement-1",
        pair_id="measurement-1:00000000000000000009",
        pair_sequence=9,
        created_monotonic_ns=100,
        created_unix_ns=200,
        timestamp_basis="app_receive_monotonic",
        left_detection={},
        right_detection={},
        reused_inputs=("right_gps",),
    )
    frozen = {"snapshot": snapshot}
    panel = SimpleNamespace()
    controller = MeasurementViewController(panel)
    controller._freeze_measurement_pair_inputs = lambda _pair: frozen
    controller._measurement_method_statuses = lambda: tuple(
        MeasurementMethodStatus(
            method_id=method_id,
            instance_id=f"{method_id}:frozen",
            ready=True,
            component_sources=(
                ("left.north_m", "live_gnss"),
            )
            if index in {0, 2, 4}
            else (),
        )
        for index, method_id in enumerate(MEASUREMENT_METHOD_IDS)
    )
    controller._build_measurement_attempt = lambda _pair, **kwargs: SimpleNamespace(
        canonical_payload=lambda: {
            "measurement_method_id": kwargs["method_status"].method_id
        }
    )

    requests = controller._method_evaluation_requests_for_pair(object())

    assert tuple(request.method_id for request in requests) == MEASUREMENT_METHOD_IDS
    assert {request.snapshot_sha256 for request in requests} == {
        snapshot.canonical_sha256()
    }
    assert {request.pair_id for request in requests} == {snapshot.pair_id}
    assert requests[0].reused_inputs == ("right_gps",)
    assert requests[1].reused_inputs == ()


def test_saturated_method_gets_backpressure_while_other_methods_stay_admitted() -> None:
    coordinator = MethodEvaluationCoordinator(start_workers=False)
    try:
        for sequence in range(1, METHOD_EVALUATION_QUEUE_LIMIT + 1):
            assert coordinator.submit(_blocked_requests(sequence)).accepted is True

        overflow = coordinator.submit(_blocked_requests(21))
        assert overflow.accepted is True
        assert overflow.submitted_method_ids == MEASUREMENT_METHOD_IDS

        blocked: dict[str, MethodEvaluationOutcome] = {}
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and len(blocked) < len(MEASUREMENT_METHOD_IDS):
            for outcome in coordinator.poll().outcomes:
                if outcome.pair_sequence == 21:
                    blocked[outcome.method_id] = outcome
            time.sleep(0.01)
        assert {row.status for row in blocked.values()} == {"BACKPRESSURE_BLOCKED"}
        assert set(blocked) == set(MEASUREMENT_METHOD_IDS)
        assert (blocked["horizon_hybrid"].pair_id, 3) not in coordinator.pending_jobs()
    finally:
        coordinator.close()
        assert coordinator.wait_closed(10.0)


def test_slow_method_backlog_only_blocks_that_method() -> None:
    coordinator = MethodEvaluationCoordinator(
        mp_context=multiprocessing.get_context("spawn"),
        process_target=_slow_horizon_worker,
    )
    try:
        for sequence in range(1, METHOD_EVALUATION_QUEUE_LIMIT + 1):
            assert coordinator.submit(_blocked_requests(sequence)).accepted is True

        primary_pairs: set[str] = set()
        deadline = time.monotonic() + 6.0
        while (
            len(primary_pairs) < METHOD_EVALUATION_QUEUE_LIMIT
            and time.monotonic() < deadline
        ):
            polled = coordinator.poll()
            primary_pairs.update(
                outcome.pair_id for outcome in polled.primary_outcomes
            )
            time.sleep(0.01)
        assert len(primary_pairs) == METHOD_EVALUATION_QUEUE_LIMIT
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and any(
            methods != ("horizon_hybrid",)
            for methods in coordinator.pending_jobs().values()
        ):
            coordinator.poll()
            time.sleep(0.01)
        assert coordinator.pending_pair_count >= 1
        assert all(
            methods == ("horizon_hybrid",)
            for methods in coordinator.pending_jobs().values()
        )

        admitted = coordinator.submit(_blocked_requests(21))
        assert admitted.accepted is True

        outcomes_21: dict[str, MethodEvaluationOutcome] = {}
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and len(outcomes_21) < len(MEASUREMENT_METHOD_IDS) - 1:
            for outcome in coordinator.poll().outcomes:
                if outcome.pair_sequence == 21:
                    outcomes_21[outcome.method_id] = outcome
            time.sleep(0.01)
        assert outcomes_21[PIXHAWK_RTK_LIVE].status == "BLOCKED"
        assert outcomes_21["horizon_hybrid"].status == "BACKPRESSURE_BLOCKED"
        assert all(
            outcome.status == "BLOCKED"
            for method_id, outcome in outcomes_21.items()
            if method_id != "horizon_hybrid"
        )
    finally:
        coordinator.close()
        assert coordinator.wait_closed(10.0)


def test_slow_primary_is_never_timed_out_and_completes() -> None:
    coordinator = MethodEvaluationCoordinator(
        mp_context=multiprocessing.get_context("spawn"),
        process_target=_slow_primary_worker,
    )
    primary = None
    bundle = None
    try:
        assert coordinator.submit(_blocked_requests(1)).accepted is True
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and bundle is None:
            polled = coordinator.poll()
            if polled.primary_outcomes:
                primary = polled.primary_outcomes[0]
            if polled.completed_bundles:
                bundle = polled.completed_bundles[0]
            time.sleep(0.01)
        assert primary is not None
        assert primary.status == "BLOCKED"
        assert primary.evaluation_duration_ms is not None
        assert primary.evaluation_duration_ms >= 1_900.0
        assert bundle is not None
        assert bundle.primary == primary
        assert coordinator.worker_generations[PIXHAWK_RTK_LIVE] == 1
    finally:
        coordinator.close()
        assert coordinator.wait_closed(10.0)


def test_subset_submission_runs_only_requested_methods() -> None:
    coordinator = MethodEvaluationCoordinator(
        mp_context=multiprocessing.get_context("spawn"),
        process_target=_slow_horizon_worker,
    )
    bundle = None
    try:
        requests = tuple(
            request
            for request in _blocked_requests(1)
            if request.method_id in {PIXHAWK_RTK_LIVE, "checkerboard_stereo"}
        )
        submission = coordinator.submit(
            requests,
            planned_methods={"terrain_pnp": "no Terrain PnP candidate"},
        )
        assert submission.submitted_method_ids == (PIXHAWK_RTK_LIVE, "checkerboard_stereo")
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and bundle is None:
            polled = coordinator.poll()
            if polled.completed_bundles:
                bundle = polled.completed_bundles[0]
            time.sleep(0.01)
        assert bundle is not None
        assert bundle.submitted_method_ids == (PIXHAWK_RTK_LIVE, "checkerboard_stereo")
        assert bundle.planned_methods == {"terrain_pnp": "no Terrain PnP candidate"}
        assert coordinator.pending_pair_count == 0
        with pytest.raises(ValueError, match="canonical order"):
            coordinator.submit(tuple(reversed(requests)))
    finally:
        coordinator.close()
        assert coordinator.wait_closed(10.0)


def test_crashed_worker_yields_failed_outcome_and_restarts() -> None:
    coordinator = MethodEvaluationCoordinator(
        mp_context=multiprocessing.get_context("spawn"),
        process_target=_crashing_horizon_worker,
    )
    bundle = None
    try:
        assert coordinator.submit(_blocked_requests(1)).accepted is True
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and bundle is None:
            polled = coordinator.poll()
            if polled.completed_bundles:
                bundle = polled.completed_bundles[0]
            time.sleep(0.01)
        assert bundle is not None
        horizon = bundle.outcome_for("horizon_hybrid")
        assert horizon is not None
        assert horizon.status == "FAILED"
        assert "exited unexpectedly" in horizon.reasons[0]
        assert all(
            outcome.status == "BLOCKED"
            for outcome in bundle.outcomes
            if outcome.method_id != "horizon_hybrid"
        )
        assert coordinator.worker_generations["horizon_hybrid"] >= 2
    finally:
        coordinator.close()
        assert coordinator.wait_closed(10.0)


def test_close_marks_unfinished_methods_failed_instead_of_dropping_them() -> None:
    coordinator = MethodEvaluationCoordinator(
        mp_context=multiprocessing.get_context("spawn"),
        process_target=_slow_horizon_worker,
    )
    assert coordinator.submit(_blocked_requests(1)).accepted is True
    primary_seen = False
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline and not primary_seen:
        primary_seen = bool(coordinator.poll().primary_outcomes)
        time.sleep(0.01)
    assert primary_seen

    close_started_s = time.monotonic()
    coordinator.close()
    assert time.monotonic() - close_started_s < 0.05
    assert coordinator.wait_closed(10.0)

    polled = coordinator.poll()
    failed = {
        outcome.method_id: outcome
        for outcome in polled.outcomes
        if outcome.status == "FAILED"
    }
    assert "horizon_hybrid" in failed
    assert "closed before completion" in failed["horizon_hybrid"].reasons[0]
    assert polled.completed_bundles and polled.completed_bundles[0].outcome_for("horizon_hybrid") is not None


@pytest.mark.parametrize("status", ("VALID", "ESTIMATED"))
def test_running_sessionless_primary_success_commits_before_display_without_bundle(
    status: str,
) -> None:
    primary = _primary_outcome(status)
    controller, _panel, events, commits = _primary_poll_controller(
        [MethodEvaluationPoll(outcomes=(primary,))]
    )

    controller._poll_method_evaluations()

    assert events == ["commit", "display"]
    assert len(commits) == 1
    commit = commits[0]
    assert commit["distance_m"] == pytest.approx(9.0, rel=1e-3)
    assert commit["publish_history"] is True
    assert commit["outcome_metadata"] == {
        "runtime_generation": primary.measurement_id,
        "method_evaluation_pair_id": primary.pair_id,
        "method_evaluation_pair_sequence": primary.pair_sequence,
        "method_evaluation_request_generation": primary.request_generation,
        "method_evaluation_snapshot_sha256": primary.snapshot_sha256,
        "primary_outcome_status": status,
        "primary_outcome_sha256": primary.canonical_sha256(),
        "primary_committed_before_bundle": True,
        "primary_commit_source": "early_outcome",
    }


def test_duplicate_primary_and_blocked_comparison_bundle_commit_measurement_once() -> None:
    primary = _primary_outcome()
    bundle = _bundle_with_primary(primary)
    controller, panel, _events, commits = _primary_poll_controller(
        [
            MethodEvaluationPoll(outcomes=(primary,)),
            MethodEvaluationPoll(
                outcomes=(primary,),
                completed_bundles=(bundle,),
            ),
        ]
    )

    controller._poll_method_evaluations()
    controller._poll_method_evaluations()

    assert len(commits) == 1
    assert commits[0]["publish_history"] is True
    assert len(panel.method_evaluation_bundle_rows) == 1
    persisted_bundle = MethodEvaluationBundle.from_mapping(
        panel.method_evaluation_bundle_rows[0]
    )
    assert persisted_bundle.primary == primary
    assert [outcome.status for outcome in persisted_bundle.outcomes[1:]] == [
        "BLOCKED"
    ] * (len(MEASUREMENT_METHOD_IDS) - 1)
    assert all(
        outcome.reasons == ("unused comparison input unavailable",)
        for outcome in persisted_bundle.outcomes[1:]
    )


def test_selected_baseline_yaw_comparison_outcome_becomes_saved_measurement() -> None:
    fixed_primary = _primary_outcome()
    attempt = MeasurementAttempt.from_canonical_payload(fixed_primary.attempt_payload)
    new_attempt = replace(
        attempt,
        measurement_method_id=PIXHAWK_RP_GPS_BASELINE_YAW,
        measurement_method_instance_id="live:pixhawk-rp-gps-baseline-yaw",
        measurement_geometry_kind="live_pixhawk_baseline_yaw",
        measurement_geometry_sha256="b" * 64,
    )
    selected = replace(
        fixed_primary,
        method_id=PIXHAWK_RP_GPS_BASELINE_YAW,
        method_instance_id="live:pixhawk-rp-gps-baseline-yaw",
        attempt_sha256=new_attempt.canonical_sha256(),
        attempt_payload=new_attempt.canonical_payload(),
        geometry_sha256=new_attempt.measurement_geometry_sha256,
    )
    bundle = _bundle_with_primary(fixed_primary)
    outcomes = list(bundle.outcomes)
    outcomes[MEASUREMENT_METHOD_IDS.index(PIXHAWK_RP_GPS_BASELINE_YAW)] = selected
    bundle = replace(bundle, outcomes=tuple(outcomes))
    controller, panel, _events, commits = _primary_poll_controller(
        [MethodEvaluationPoll(completed_bundles=(bundle,))]
    )
    panel._measurement_recording_method_id = PIXHAWK_RP_GPS_BASELINE_YAW

    controller._poll_method_evaluations()

    assert len(commits) == 1
    assert commits[0]["method"].endswith(PIXHAWK_RP_GPS_BASELINE_YAW)
    assert commits[0]["distance_m"] == pytest.approx(9.0, rel=1.0e-3)


def test_bundle_only_success_falls_back_to_one_primary_measurement_commit() -> None:
    primary = _primary_outcome()
    bundle = _bundle_with_primary(primary)
    controller, panel, events, commits = _primary_poll_controller(
        [MethodEvaluationPoll(completed_bundles=(bundle,))]
    )

    controller._poll_method_evaluations()

    assert events == ["commit", "display"]
    assert len(commits) == 1
    assert commits[0]["publish_history"] is True
    assert commits[0]["outcome_metadata"]["primary_commit_source"] == (
        "bundle_fallback"
    )
    assert commits[0]["outcome_metadata"]["primary_committed_before_bundle"] is False
    assert len(panel.method_evaluation_bundle_rows) == 1


@pytest.mark.parametrize("run_state", ("STOPPING", "IDLE"))
def test_stopped_or_idle_primary_outcome_has_no_measurement_commit_or_ui(
    run_state: str,
) -> None:
    primary = _primary_outcome()
    controller, _panel, events, commits = _primary_poll_controller(
        [MethodEvaluationPoll(outcomes=(primary,))],
        admissions=(),
        run_state=run_state,
    )

    controller._poll_method_evaluations()

    assert commits == []
    assert events == []


@pytest.mark.parametrize(
    "status",
    ("BLOCKED", "FAILED", "TIMEOUT", "BACKPRESSURE_BLOCKED"),
)
def test_nonsuccess_primary_outcome_never_commits_a_measurement(
    status: str,
) -> None:
    primary = _primary_outcome(status)
    controller, _panel, events, commits = _primary_poll_controller(
        [MethodEvaluationPoll(outcomes=(primary,))],
        admissions=(),
    )

    controller._poll_method_evaluations()

    assert commits == []
    assert events == ["display"]


def test_rejected_early_primary_enqueue_retries_in_background_without_second_history() -> None:
    primary = _primary_outcome()
    bundle = _bundle_with_primary(primary)
    controller, panel, events, commits = _primary_poll_controller(
        [
            MethodEvaluationPoll(outcomes=(primary,)),
            MethodEvaluationPoll(completed_bundles=(bundle,)),
        ],
        admissions=(False, True),
    )

    controller._poll_method_evaluations()
    controller._poll_method_evaluations()

    assert [commit["publish_history"] for commit in commits] == [True, False]
    assert [
        commit["outcome_metadata"]["primary_commit_source"] for commit in commits
    ] == ["early_outcome", "background_retry"]
    assert [
        commit["outcome_metadata"]["primary_committed_before_bundle"]
        for commit in commits
    ] == [True, False]
    assert {
        commit["outcome_metadata"]["primary_outcome_sha256"] for commit in commits
    } == {primary.canonical_sha256()}
    assert events == ["commit", "display", "commit", "display"]
    assert len(panel.method_evaluation_bundle_rows) == 1
    assert controller._primary_measurement_persisted == {
        controller._primary_outcome_key(primary)
    }


def test_saved_bundle_keeps_rejected_primary_retry_owned_and_ui_degraded() -> None:
    primary = _primary_outcome()
    bundle = _bundle_with_primary(primary)
    controller, panel, _events, commits = _primary_poll_controller(
        [
            MethodEvaluationPoll(outcomes=(primary,)),
            MethodEvaluationPoll(completed_bundles=(bundle,)),
        ],
        admissions=(False, False, False, False, True),
    )
    displays: list[dict[str, object]] = []
    panel._set_distance_text = lambda text, **kwargs: displays.append(
        {"text": str(text), **kwargs}
    )
    primary_key = controller._primary_outcome_key(primary)
    bundle_key = controller._method_bundle_key(bundle)

    controller._poll_method_evaluations()
    controller._poll_method_evaluations()

    assert [commit["publish_history"] for commit in commits] == [
        True,
        False,
        False,
    ]
    assert primary_key not in controller._primary_measurement_persisted
    assert controller._pending_primary_measurement_commits == {
        primary_key: primary
    }
    assert bundle_key in controller._persisted_method_bundle_keys
    assert controller._pending_method_bundle_commits == {}
    assert len(panel.method_evaluation_bundle_rows) == 1
    assert "EVIDENCE DEGRADED" in displays[-1]["text"]
    assert displays[-1]["quality"] == "WARNING"
    counters_after_bundle = {
        method_id: dict(counts)
        for method_id, counts in panel._measurement_method_result_counters.items()
    }
    displays_after_bundle = list(displays)
    controller._method_evaluation_coordinator.pending_pair_count = 0

    assert controller.method_evaluation_pending_count() == 0
    assert primary_key in controller._pending_primary_measurement_commits

    controller._retry_pending_primary_measurements()

    assert [commit["publish_history"] for commit in commits] == [
        True,
        False,
        False,
        False,
        False,
    ]
    assert primary_key in controller._primary_measurement_persisted
    assert controller._pending_primary_measurement_commits == {}
    assert controller._persisted_method_bundle_keys == {bundle_key}
    assert len(panel.method_evaluation_bundle_rows) == 1
    assert panel._measurement_method_result_counters == counters_after_bundle
    assert displays == displays_after_bundle


def test_rejected_old_primary_retries_late_direct_without_blocking_next_run() -> None:
    primary = _primary_outcome()
    controller, panel, _events, commits = _primary_poll_controller(
        [MethodEvaluationPoll(outcomes=(primary,))],
        admissions=(False, False),
    )
    displays: list[dict[str, object]] = []
    panel._set_distance_text = lambda text, **kwargs: displays.append(
        {"text": str(text), **kwargs}
    )
    primary_key = controller._primary_outcome_key(primary)

    controller._poll_method_evaluations()
    controller._method_evaluation_coordinator.pending_pair_count = 0

    assert controller.method_evaluation_pending_count() == 0
    assert primary_key in controller._pending_primary_measurement_commits

    panel._measurement_run_generation = "measurement-2"
    panel._measurement_id = "measurement-2"
    panel._measurement_run_state = "RUNNING"
    panel._measurement_method_result_counters = {}
    writes: list[dict[str, object]] = []
    admissions = [False, True]

    def append_workstation_data(stream, payload, *, priority=None):
        writes.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return admissions.pop(0)

    panel.append_workstation_data = append_workstation_data
    displays_before_late_retry = list(displays)
    commits_before_late_retry = list(commits)

    controller._retry_pending_primary_measurements()
    controller._retry_pending_primary_measurements()

    assert len(writes) == 2
    assert all(write["stream"] == "measurements" for write in writes)
    assert all(write["priority"] is True for write in writes)
    late = writes[-1]["payload"]
    assert late["event_type"] == "late_primary_measurement_outcome"
    assert late["runtime_generation"] == "measurement-1"
    assert late["current_runtime_generation"] == "measurement-2"
    assert late["late"] is True
    assert late["primary_outcome_sha256"] == primary.canonical_sha256()
    assert MethodEvaluationOutcome.from_mapping(late["primary_outcome"]) == primary
    assert MethodEvaluationOutcome.from_mapping(
        json.loads(late["primary_outcome_canonical_json"])
    ).canonical_sha256() == primary.canonical_sha256()
    assert primary_key in controller._primary_measurement_persisted
    assert controller._pending_primary_measurement_commits == {}
    assert panel._measurement_method_result_counters == {}
    assert displays == displays_before_late_retry
    assert commits == commits_before_late_retry


def test_pending_primary_retry_budget_rotates_one_key_per_tick() -> None:
    base = _primary_outcome()
    outcomes = tuple(
        replace(
            base,
            pair_id=f"measurement-1:pair-{index}",
            pair_sequence=index,
            request_generation=index,
        )
        for index in range(1, 4)
    )
    controller, panel, events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    controller._method_evaluation_coordinator = None
    controller._pending_primary_measurement_commits = {
        controller._primary_outcome_key(outcome): outcome for outcome in outcomes
    }
    panel._measurement_run_generation = "measurement-2"
    panel._measurement_id = "measurement-2"
    writes: list[str] = []

    def append_workstation_data(_stream, payload, *, priority=None):
        assert priority is True
        writes.append(str(payload["method_evaluation_pair_id"]))
        return False

    panel.append_workstation_data = append_workstation_data

    for expected_count in range(1, 4):
        controller._retry_pending_primary_measurements()
        assert len(writes) == expected_count

    assert writes == [outcome.pair_id for outcome in outcomes]
    assert list(controller._pending_primary_measurement_commits) == [
        controller._primary_outcome_key(outcome) for outcome in outcomes
    ]
    assert panel._measurement_method_result_counters == {}
    assert events == []
    assert commits == []


def test_pending_primary_ownership_is_bounded_with_explicit_overflow_status(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import measurement_view_controller as measurement_module

    monkeypatch.setattr(measurement_module, "PERSISTENCE_PENDING_MAX_COUNT", 2)
    monkeypatch.setattr(
        measurement_module,
        "PERSISTENCE_PENDING_MAX_BYTES",
        10_000_000,
    )
    controller, panel, _events, _commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    overflow_events: list[dict[str, object]] = []
    released_record_keys: list[tuple[object, ...]] = []
    panel._recording_controller_owner = SimpleNamespace(
        _release_primary_retry_ownership=lambda key: released_record_keys.append(
            tuple(key)
        )
    )
    panel.append_workstation_data = lambda stream, payload, **_kwargs: (
        overflow_events.append({"stream": str(stream), "payload": dict(payload)})
        or False
    )
    base = _primary_outcome()
    outcomes = tuple(
        replace(
            base,
            pair_id=f"measurement-1:bounded-{index}",
            pair_sequence=index,
            request_generation=index,
        )
        for index in range(1, 4)
    )

    for outcome in outcomes:
        controller._retain_pending_primary_measurement(
            controller._primary_outcome_key(outcome),
            outcome,
        )

    assert list(controller._pending_primary_measurement_commits) == [
        controller._primary_outcome_key(outcome) for outcome in outcomes[1:]
    ]
    assert len(controller._pending_primary_measurement_commits) == 2
    assert controller._pending_primary_measurement_bytes == sum(
        controller._pending_primary_measurement_sizes.values()
    )
    assert controller._pending_primary_measurement_bytes <= 10_000_000
    assert controller._persistence_pending_overflow_count == 1
    assert controller._persistence_pending_overflow_evidence_failures == 1
    assert overflow_events[-1]["stream"] == "events"
    assert overflow_events[-1]["payload"]["pair_id"] == outcomes[0].pair_id
    status = panel._primary_measurement_retry_status
    assert status["state"] == "OVERFLOW"
    assert status["primary_pending_count"] == 2
    assert status["overflow_count"] == 1
    assert status["overflow_evidence_failures"] == 1
    assert released_record_keys == [
        tuple(_pixhawk_fallback_attempt().detection_pair_key)
    ]


def test_pending_primary_single_record_byte_overflow_is_explicit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import measurement_view_controller as measurement_module

    monkeypatch.setattr(measurement_module, "PERSISTENCE_PENDING_MAX_COUNT", 10)
    monkeypatch.setattr(measurement_module, "PERSISTENCE_PENDING_MAX_BYTES", 1)
    controller, panel, _events, _commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    overflow_events: list[dict[str, object]] = []
    panel.append_workstation_data = lambda stream, payload, **_kwargs: (
        overflow_events.append({"stream": str(stream), "payload": dict(payload)})
        or True
    )
    outcome = _primary_outcome()

    retained = controller._retain_pending_primary_measurement(
        controller._primary_outcome_key(outcome),
        outcome,
    )

    assert retained is False
    assert controller._pending_primary_measurement_commits == {}
    assert controller._pending_primary_measurement_bytes == 0
    assert controller._persistence_pending_overflow_count == 1
    assert controller._persistence_pending_overflow_evidence_failures == 0
    assert "exceeds byte_limit=1" in overflow_events[-1]["payload"]["reason"]
    assert panel._primary_measurement_retry_status["state"] == "OVERFLOW"


def test_pending_bundle_ownership_and_retry_work_are_bounded(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import measurement_view_controller as measurement_module

    monkeypatch.setattr(measurement_module, "PERSISTENCE_PENDING_MAX_COUNT", 2)
    monkeypatch.setattr(
        measurement_module,
        "PERSISTENCE_PENDING_MAX_BYTES",
        10_000_000,
    )
    controller, panel, _events, commits = _primary_poll_controller(
        [],
        admissions=(),
        run_state="IDLE",
    )
    controller._method_evaluation_coordinator = None
    panel._measurement_run_generation = ""
    method_writes: list[dict[str, object]] = []
    overflow_events: list[dict[str, object]] = []

    def append_workstation_data(stream, payload, *, priority=None):
        assert priority is True
        target = overflow_events if stream == "events" else method_writes
        target.append(dict(payload))
        return stream == "events"

    panel.append_workstation_data = append_workstation_data
    base = _bundle_with_primary(_primary_outcome("BLOCKED"))
    bundles = []
    for index in range(1, 4):
        pair_id = f"measurement-1:bundle-{index}"
        outcomes = tuple(
            replace(
                outcome,
                pair_id=pair_id,
                pair_sequence=index,
                request_generation=index,
            )
            for outcome in base.outcomes
        )
        bundles.append(
            replace(
                base,
                pair_id=pair_id,
                pair_sequence=index,
                request_generation=index,
                outcomes=outcomes,
            )
        )

    for bundle in bundles:
        controller._commit_method_evaluation_bundle(bundle)

    assert list(controller._pending_method_bundle_commits) == [
        controller._method_bundle_key(bundle) for bundle in bundles[1:]
    ]
    assert len(controller._pending_method_bundle_commits) == 2
    assert controller._pending_method_bundle_bytes == sum(
        controller._pending_method_bundle_sizes.values()
    )
    assert controller._persistence_pending_overflow_count == 1
    assert controller._persistence_pending_overflow_evidence_failures == 0
    assert overflow_events[-1]["pending_kind"] == "method_bundle"
    writes_before_retry = len(method_writes)

    controller._retry_pending_method_bundles()

    assert len(method_writes) == writes_before_retry + 1
    assert len(controller._pending_method_bundle_commits) == 2
    assert panel._measurement_method_result_counters == {}
    assert commits == []


def test_shutdown_terminalizes_permanently_rejected_pending_after_close_budget(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import measurement_view_controller as measurement_module

    monkeypatch.setattr(
        measurement_module,
        "METHOD_EVALUATION_SHUTDOWN_CLOSE_RETRY_LIMIT",
        2,
    )
    controller, panel, _events, commits = _primary_poll_controller(
        [],
        admissions=(),
        run_state="IDLE",
    )
    controller._method_evaluation_coordinator = None
    panel._measurement_run_generation = ""
    panel._shutdown_in_progress = True
    writes: list[dict[str, object]] = []

    def append_workstation_data(stream, payload, *, priority=None):
        writes.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return False

    panel.append_workstation_data = append_workstation_data
    primary = _primary_outcome()
    primary_key = controller._primary_outcome_key(primary)
    assert controller._retain_pending_primary_measurement(primary_key, primary)
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    bundle_key = controller._method_bundle_key(bundle)
    controller._commit_method_evaluation_bundle(bundle)
    assert primary_key in controller._pending_primary_measurement_commits
    assert bundle_key in controller._pending_method_bundle_commits

    assert controller.close_method_evaluation_workers() is False
    assert primary_key in controller._pending_primary_measurement_commits
    assert bundle_key in controller._pending_method_bundle_commits

    assert controller.close_method_evaluation_workers() is True

    assert controller._pending_primary_measurement_commits == {}
    assert controller._pending_method_bundle_commits == {}
    assert controller._persistence_pending_overflow_count == 2
    assert controller._persistence_pending_overflow_evidence_failures == 2
    shutdown_drops = [
        write
        for write in writes
        if write["stream"] == "events"
        and str(write["payload"].get("pending_kind", "")).endswith(
            "_shutdown_drop"
        )
    ]
    assert len(shutdown_drops) == 2
    assert all(
        "shutdown close retry budget exhausted"
        in write["payload"]["reason"]
        for write in shutdown_drops
    )
    status = panel._primary_measurement_retry_status
    assert status["state"] == "OVERFLOW"
    assert status["primary_pending_count"] == 0
    assert status["bundle_pending_count"] == 0
    assert status["overflow_count"] == 2
    assert status["overflow_evidence_failures"] == 2
    assert commits == []


def test_conflicting_duplicate_keeps_first_normalized_primary_outcome() -> None:
    primary = _primary_outcome("VALID")
    controller, _panel, _events, _commits = _primary_poll_controller([])
    first = controller._normalize_primary_outcome(primary)
    conflicting = replace(
        primary,
        completed_monotonic_ns=primary.completed_monotonic_ns + 1,
    )

    second = controller._normalize_primary_outcome(conflicting)

    assert first == primary
    assert second == primary
    assert second.status == "VALID"


def test_conflicting_bundle_preserves_first_primary_commit_and_status() -> None:
    primary = _primary_outcome("VALID")
    conflicting = replace(
        primary,
        completed_monotonic_ns=primary.completed_monotonic_ns + 1,
    )
    bundle = _bundle_with_primary(conflicting)
    controller, panel, events, commits = _primary_poll_controller(
        [
            MethodEvaluationPoll(outcomes=(primary,)),
            MethodEvaluationPoll(completed_bundles=(bundle,)),
        ]
    )

    controller._poll_method_evaluations()
    controller._poll_method_evaluations()

    assert len(commits) == 1
    assert commits[0]["outcome_metadata"]["primary_outcome_sha256"] == (
        primary.canonical_sha256()
    )
    assert len(panel.method_evaluation_bundle_rows) == 1
    persisted = MethodEvaluationBundle.from_mapping(
        panel.method_evaluation_bundle_rows[0]
    )
    assert persisted.primary.canonical_sha256() == primary.canonical_sha256()
    assert persisted.primary.status == "VALID"
    assert panel._last_method_evaluation_bundle.primary.status == "VALID"
    assert events == ["commit", "display", "display"]


def test_active_bundle_commits_only_after_live_rows_retry_and_counts_once() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    store = _BundleAdmissionStore((False, True))
    rows = LiveDataRows(store, "method_evaluations", priority=True)
    controller, panel, _events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    panel.method_evaluation_bundle_rows = rows
    identity = (bundle.pair_id, bundle.request_generation)
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)

    assert commits == []
    assert len(rows) == 1
    assert [call["payload"]["_record_operation"] for call in store.calls] == [
        "append"
    ]
    assert identity not in panel._method_evaluation_committed
    assert bundle_key not in controller._persisted_method_bundle_keys
    assert bundle_key in controller._pending_method_bundle_commits
    assert panel._measurement_method_result_counters == {}

    controller._retry_pending_method_bundles()

    assert len(rows) == 1
    assert [call["payload"]["_record_operation"] for call in store.calls] == [
        "append",
        "append",
    ]
    assert (
        MethodEvaluationBundle.from_mapping(rows[0]).canonical_sha256()
        == bundle.canonical_sha256()
    )
    assert identity in panel._method_evaluation_committed
    assert bundle_key in controller._persisted_method_bundle_keys
    assert bundle_key not in controller._pending_method_bundle_commits
    assert sum(
        counts["BLOCKED"]
        for counts in panel._measurement_method_result_counters.values()
    ) == len(MEASUREMENT_METHOD_IDS)

    calls_after_success = list(store.calls)
    counters_after_success = {
        method_id: dict(counts)
        for method_id, counts in panel._measurement_method_result_counters.items()
    }
    controller._commit_method_evaluation_bundle(bundle)
    controller._retry_pending_method_bundles()

    assert store.calls == calls_after_success
    assert panel._measurement_method_result_counters == counters_after_success


def test_late_bundle_writer_rejection_retries_once_then_deduplicates() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    writes: list[dict[str, object]] = []
    admissions = [False, True]
    controller, panel, _events, commits = _primary_poll_controller(
        [],
        admissions=(),
        run_state="IDLE",
    )
    panel._measurement_run_generation = ""
    last_bundle_sentinel = object()
    panel._last_method_evaluation_bundle = last_bundle_sentinel

    def append_workstation_data(stream, payload, *, priority=None):
        writes.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return admissions.pop(0)

    panel.append_workstation_data = append_workstation_data
    identity = (bundle.pair_id, bundle.request_generation)
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)

    assert commits == []
    assert len(writes) == 1
    assert writes[0]["stream"] == "method_evaluations"
    assert writes[0]["priority"] is True
    assert writes[0]["payload"]["late"] is True
    assert writes[0]["payload"]["current_runtime_generation"] == ""
    assert identity not in panel._method_evaluation_committed
    assert panel._measurement_method_result_counters == {}

    controller._retry_pending_method_bundles()

    assert len(writes) == 2
    assert writes[1]["payload"]["late"] is True
    assert writes[1]["payload"]["current_runtime_generation"] == ""
    assert identity not in panel._method_evaluation_committed
    assert bundle_key in controller._persisted_method_bundle_keys
    assert bundle_key not in controller._pending_method_bundle_commits
    assert panel._measurement_method_result_counters == {}
    assert panel._last_method_evaluation_bundle is last_bundle_sentinel

    calls_after_success = list(writes)
    counters_after_success = {
        method_id: dict(counts)
        for method_id, counts in panel._measurement_method_result_counters.items()
    }
    controller._commit_method_evaluation_bundle(bundle)
    controller._retry_pending_method_bundles()

    assert writes == calls_after_success
    assert panel._measurement_method_result_counters == counters_after_success


def test_pending_active_bundle_becomes_late_on_same_row_after_stop() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    store = _BundleAdmissionStore((False, True))
    rows = LiveDataRows(store, "method_evaluations", priority=True)
    controller, panel, _events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    panel.method_evaluation_bundle_rows = rows
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)

    assert commits == []
    assert len(rows) == 1
    assert "late" not in rows[0]
    assert bundle_key in controller._pending_method_bundle_commits

    panel._measurement_run_state = "IDLE"
    panel._measurement_run_generation = ""
    controller._retry_pending_method_bundles()

    assert len(rows) == 1
    assert rows[0]["late"] is True
    assert rows[0]["current_runtime_generation"] == ""
    assert [call["payload"]["_record_operation"] for call in store.calls] == [
        "append",
        "append",
    ]
    assert "late" not in store.calls[0]["payload"]
    assert store.calls[1]["payload"]["late"] is True
    assert bundle_key in controller._persisted_method_bundle_keys
    assert bundle_key not in controller._pending_method_bundle_commits
    assert panel._method_evaluation_committed == set()
    assert panel._measurement_method_result_counters == {}
    assert not hasattr(panel, "_last_method_evaluation_bundle")

    calls_after_success = list(store.calls)
    controller._commit_method_evaluation_bundle(bundle)
    assert store.calls == calls_after_success


@pytest.mark.parametrize("owner_failure", ("cleared", "closed"))
def test_late_pending_unusable_row_owner_falls_back_to_direct_writer(
    owner_failure: str,
) -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    store = _BundleAdmissionStore(
        (False, True) if owner_failure == "cleared" else (False,)
    )
    rows = LiveDataRows(store, "method_evaluations", priority=True)
    controller, panel, events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    panel.method_evaluation_bundle_rows = rows
    writes: list[dict[str, object]] = []

    def append_workstation_data(stream, payload, *, priority=None):
        writes.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return True

    panel.append_workstation_data = append_workstation_data
    last_bundle_sentinel = object()
    panel._last_method_evaluation_bundle = last_bundle_sentinel
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)

    assert len(rows) == 1
    assert bundle_key in controller._pending_method_bundle_commits
    old_lease = controller._pending_method_bundle_commits[bundle_key].row_lease
    assert old_lease is not None
    if owner_failure == "cleared":
        rows.clear()
        new_row = rows.append_with_admission({"new_generation": True})
        assert new_row.state == ROW_WRITE_ADMITTED
        assert new_row.lease != old_lease
    else:
        store.lifecycle_state = "CLOSED"
        store.writer_alive = False
    panel._measurement_run_state = "IDLE"
    panel._measurement_run_generation = ""

    controller._retry_pending_method_bundles()

    assert len(store.calls) == (2 if owner_failure == "cleared" else 1)
    if owner_failure == "cleared":
        assert rows[0] == {"new_generation": True}
    assert len(writes) == 1
    assert writes[0]["stream"] == "method_evaluations"
    assert writes[0]["priority"] is True
    assert writes[0]["payload"]["late"] is True
    assert writes[0]["payload"]["current_runtime_generation"] == ""
    assert bundle_key in controller._persisted_method_bundle_keys
    assert bundle_key not in controller._pending_method_bundle_commits
    assert panel._method_evaluation_committed == set()
    assert panel._measurement_method_result_counters == {}
    assert panel._last_method_evaluation_bundle is last_bundle_sentinel
    assert events == []
    assert commits == []

    calls_after_success = list(writes)
    controller._commit_method_evaluation_bundle(bundle)
    assert writes == calls_after_success


def test_failed_spool_initial_append_falls_back_to_direct_writer(
    tmp_path,
) -> None:
    class FailingHandle:
        def write(self, _value):
            raise OSError("injected closed spool")

        def close(self):
            return None

    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    rows = SpoolRows(supports_updates=True)
    rows.bind(tmp_path / "method_evaluation_bundles.jsonl")
    assert rows._handle is not None
    rows._handle.close()
    rows._handle = FailingHandle()
    controller, panel, events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    panel.method_evaluation_bundle_rows = rows
    writes: list[dict[str, object]] = []

    def append_workstation_data(stream, payload, *, priority=None):
        writes.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return True

    panel.append_workstation_data = append_workstation_data
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)

    assert len(rows) == 0
    assert list(rows) == []
    assert rows.write_error == "injected closed spool"
    assert len(writes) == 1
    assert writes[0]["stream"] == "method_evaluations"
    assert writes[0]["priority"] is True
    assert "late" not in writes[0]["payload"]
    assert bundle_key in controller._persisted_method_bundle_keys
    assert bundle_key not in controller._pending_method_bundle_commits
    assert panel._method_evaluation_committed == {
        (bundle.pair_id, bundle.request_generation)
    }
    assert sum(
        counts["BLOCKED"]
        for counts in panel._measurement_method_result_counters.values()
    ) == len(MEASUREMENT_METHOD_IDS)
    assert panel._last_method_evaluation_bundle == bundle
    assert events == ["display"]
    assert commits == []


def test_closed_bound_spool_atomic_replace_reports_unavailable(tmp_path) -> None:
    path = tmp_path / "closed-method-evaluation-bundles.jsonl"
    rows = SpoolRows(supports_updates=True)
    rows.bind(path)
    assert rows.append({"state": "ACTIVE"}) is True
    assert rows.close() is True
    before = path.read_bytes()

    admission = rows.replace_with_admission(0, {"state": "LATE"})

    assert admission.state == ROW_WRITE_UNAVAILABLE
    assert "closed" in admission.reason
    assert path.read_bytes() == before
    assert rows[0] == {"state": "ACTIVE"}


@pytest.mark.parametrize("bound_then_closed", (False, True))
def test_unwritable_spool_atomic_append_is_unavailable_with_lease(
    tmp_path,
    bound_then_closed: bool,
) -> None:
    rows = SpoolRows(supports_updates=True)
    path = tmp_path / "unwritable-method-evaluation-bundles.jsonl"
    if bound_then_closed:
        rows.bind(path)
        assert rows.close() is True
    before = b"" if not path.exists() else path.read_bytes()

    admission = rows.append_with_admission({"state": "PENDING"})

    assert admission.state == ROW_WRITE_UNAVAILABLE
    assert admission.row_index == 0
    assert admission.lease is not None
    assert len(rows) == 0
    assert list(rows) == []
    assert (b"" if not path.exists() else path.read_bytes()) == before


def test_pending_old_bundle_becomes_late_after_next_run_starts() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    store = _BundleAdmissionStore((False, True))
    rows = LiveDataRows(store, "method_evaluations", priority=True)
    controller, panel, events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    panel.method_evaluation_bundle_rows = rows
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)

    assert commits == []
    assert len(rows) == 1
    assert "late" not in rows[0]
    assert bundle_key in controller._pending_method_bundle_commits

    panel._measurement_run_state = "RUNNING"
    panel._measurement_run_generation = "measurement-2"
    panel._measurement_id = "measurement-2"
    controller._retry_pending_method_bundles()

    assert panel._measurement_run_generation == "measurement-2"
    assert len(rows) == 1
    assert rows[0]["late"] is True
    assert rows[0]["current_runtime_generation"] == "measurement-2"
    assert [call["payload"]["_record_operation"] for call in store.calls] == [
        "append",
        "append",
    ]
    assert "late" not in store.calls[0]["payload"]
    assert store.calls[1]["payload"]["late"] is True
    assert store.calls[1]["payload"]["current_runtime_generation"] == (
        "measurement-2"
    )
    assert bundle_key in controller._persisted_method_bundle_keys
    assert bundle_key not in controller._pending_method_bundle_commits
    assert panel._method_evaluation_committed == set()
    assert panel._measurement_method_result_counters == {}
    assert not hasattr(panel, "_last_method_evaluation_bundle")
    assert events == []

    calls_after_success = list(store.calls)
    controller._commit_method_evaluation_bundle(bundle)
    assert store.calls == calls_after_success


def test_hierarchy_pending_old_bundle_does_not_project_into_new_measurement() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    store = _BundleAdmissionStore((False, True))
    rows = LiveDataRows(store, "method_evaluations", priority=True)
    controller, panel, events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    panel._measurement_session_hierarchy_enabled = True
    panel.method_evaluation_bundle_rows = rows
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)
    assert bundle_key in controller._pending_method_bundle_commits

    panel._measurement_id = "measurement-2"
    panel._measurement_run_generation = "measurement-2"
    panel._method_evaluation_committed = set()
    panel._measurement_method_result_counters = {}
    last_bundle_sentinel = object()
    panel._last_method_evaluation_bundle = last_bundle_sentinel
    events.clear()

    controller._retry_pending_method_bundles()

    assert bundle_key in controller._persisted_method_bundle_keys
    assert bundle_key not in controller._pending_method_bundle_commits
    assert rows[0]["late"] is True
    assert rows[0]["current_runtime_generation"] == "measurement-2"
    assert panel._method_evaluation_committed == set()
    assert panel._measurement_method_result_counters == {}
    assert panel._last_method_evaluation_bundle is last_bundle_sentinel
    assert events == []
    assert commits == []


def test_hierarchy_old_primary_cannot_commit_into_new_measurement() -> None:
    outcome = _primary_outcome("VALID")
    controller, panel, events, commits = _primary_poll_controller(
        [],
        admissions=(True,),
    )
    panel._measurement_session_hierarchy_enabled = True
    panel._measurement_id = "measurement-2"

    _normalized, current, persistence_state = (
        controller._commit_primary_method_outcome(
            outcome,
            commit_source="bundle_fallback",
        )
    )

    assert current is False
    assert persistence_state == "NOT_REQUESTED"
    assert commits == []
    assert events == []


def test_bundle_write_generation_switch_does_not_project_into_new_run() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    controller, panel, events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    writes: list[dict[str, object]] = []

    class SwitchingRows(list):
        def append(self, record):
            writes.append(dict(record))
            super().append(dict(record))
            panel._measurement_run_generation = "measurement-2"
            panel._measurement_id = "measurement-2"
            return True

    panel.method_evaluation_bundle_rows = SwitchingRows()
    last_bundle_sentinel = object()
    panel._last_method_evaluation_bundle = last_bundle_sentinel
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)

    assert len(writes) == 1
    assert bundle_key in controller._persisted_method_bundle_keys
    assert controller._pending_method_bundle_commits == {}
    assert panel._measurement_run_generation == "measurement-2"
    assert panel._method_evaluation_committed == set()
    assert panel._measurement_method_result_counters == {}
    assert panel._last_method_evaluation_bundle is last_bundle_sentinel
    assert events == []
    assert commits == []


def test_persisted_old_bundle_dedupe_survives_next_pairing_generation() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    controller, panel, events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    writes: list[dict[str, object]] = []

    class Rows(list):
        def append(self, record):
            writes.append(dict(record))
            super().append(dict(record))
            return True

    panel.method_evaluation_bundle_rows = Rows()
    bundle_key = controller._method_bundle_key(bundle)
    controller._commit_method_evaluation_bundle(bundle)
    assert bundle_key in controller._persisted_method_bundle_keys
    assert len(writes) == 1

    controller.begin_pairing_generation()
    panel._measurement_run_generation = "measurement-2"
    panel._measurement_id = "measurement-2"
    panel._measurement_run_state = "RUNNING"
    panel._method_evaluation_committed = set()
    panel._measurement_method_result_counters = {}
    last_bundle_sentinel = object()
    panel._last_method_evaluation_bundle = last_bundle_sentinel

    controller._commit_method_evaluation_bundle(bundle)

    assert bundle_key in controller._persisted_method_bundle_keys
    assert len(writes) == 1
    assert panel._method_evaluation_committed == set()
    assert panel._measurement_method_result_counters == {}
    assert panel._last_method_evaluation_bundle is last_bundle_sentinel
    assert events == ["display"]
    assert commits == []


def test_close_keeps_retry_timer_until_pending_direct_bundle_is_admitted() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    admissions = [False, False, True]
    writes: list[dict[str, object]] = []
    controller, panel, _events, commits = _primary_poll_controller(
        [],
        admissions=(),
        run_state="IDLE",
    )
    panel._measurement_run_generation = ""

    def append_workstation_data(stream, payload, *, priority=None):
        writes.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return admissions.pop(0)

    panel.append_workstation_data = append_workstation_data
    controller._commit_method_evaluation_bundle(bundle)
    assert controller._pending_method_bundle_commits
    controller._method_evaluation_coordinator = None
    controller._ensure_method_evaluation_timer()
    assert controller._method_evaluation_timer is not None
    assert controller._method_evaluation_timer.isActive()

    assert controller.close_method_evaluation_workers() is False

    assert len(writes) == 2
    assert controller._pending_method_bundle_commits
    assert controller._method_evaluation_timer is not None
    assert controller._method_evaluation_timer.isActive()

    controller._retry_pending_method_bundles()

    assert len(writes) == 3
    assert controller._pending_method_bundle_commits == {}
    assert controller._method_evaluation_timer is None
    assert commits == []


def test_pending_bundle_is_included_in_legacy_save_barrier() -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    controller, panel, _events, _commits = _primary_poll_controller(
        [],
        admissions=(),
        run_state="IDLE",
    )
    panel._measurement_run_generation = ""
    panel.append_workstation_data = lambda *_args, **_kwargs: False
    controller._method_evaluation_coordinator = None

    controller._commit_method_evaluation_bundle(bundle)

    assert controller.method_evaluation_pending_count() == 1
    assert controller._pending_method_bundle_commits


def test_unavailable_current_owner_retries_only_direct_writer(tmp_path) -> None:
    bundle = _bundle_with_primary(_primary_outcome("BLOCKED"))
    controller, panel, _events, commits = _primary_poll_controller(
        [],
        admissions=(),
    )
    rows = SpoolRows(supports_updates=True)
    panel.method_evaluation_bundle_rows = rows
    admissions = [False, False, True]
    writes: list[dict[str, object]] = []

    def append_workstation_data(stream, payload, *, priority=None):
        writes.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return admissions.pop(0)

    panel.append_workstation_data = append_workstation_data
    bundle_key = controller._method_bundle_key(bundle)

    controller._commit_method_evaluation_bundle(bundle)
    assert len(rows) == 0
    assert len(writes) == 1
    assert controller._pending_method_bundle_commits[bundle_key].row_owner is None

    controller._retry_pending_method_bundles()
    controller._retry_pending_method_bundles()

    assert len(rows) == 0
    assert len(writes) == 3
    assert bundle_key in controller._persisted_method_bundle_keys
    assert bundle_key not in controller._pending_method_bundle_commits
    assert commits == []


def test_deferred_start_actually_starts_workers() -> None:
    """``start_workers=False`` defers spawning until ``.start()`` is called.

    Regression guard: ``start()`` on the actor facade previously only raised
    when already closed and otherwise did nothing, so a coordinator built
    with ``start_workers=False`` could never actually start its workers.
    """

    coordinator = MethodEvaluationCoordinator(
        mp_context=multiprocessing.get_context("spawn"),
        process_target=_slow_horizon_worker,
        start_workers=False,
    )
    bundle = None
    try:
        coordinator.start()
        assert coordinator.submit(_blocked_requests(1)).accepted is True
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and bundle is None:
            polled = coordinator.poll()
            if polled.completed_bundles:
                bundle = polled.completed_bundles[0]
                break
            time.sleep(0.01)

        assert bundle is not None
        assert len(bundle.outcomes) == len(MEASUREMENT_METHOD_IDS)
    finally:
        coordinator.close()
        assert coordinator.wait_closed(10.0)


def test_context_manager_starts_deferred_workers_once() -> None:
    coordinator = MethodEvaluationCoordinator(
        mp_context=multiprocessing.get_context("spawn"),
        process_target=_slow_horizon_worker,
        start_workers=False,
    )

    with coordinator:
        coordinator.start()
        deadline = time.monotonic() + 3.0
        while (
            coordinator.worker_generations[PIXHAWK_RTK_LIVE] < 1
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert coordinator.worker_generations[PIXHAWK_RTK_LIVE] == 1
        assert all(
            value == 0
            for method_id, value in coordinator.worker_generations.items()
            if method_id != PIXHAWK_RTK_LIVE
        )

        assert coordinator.submit(_blocked_requests(1)).accepted is True
        primary_seen = False
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            polled = coordinator.poll()
            primary_seen = primary_seen or bool(polled.primary_outcomes)
            if primary_seen and all(
                value >= 1 for value in coordinator.worker_generations.values()
            ):
                break
            time.sleep(0.01)
        assert primary_seen
        assert all(value == 1 for value in coordinator.worker_generations.values())

    assert coordinator.closed


def test_gui_bundle_commit_reuses_worker_geometry_without_recalculation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import measurement_view_controller

    attempt = _pixhawk_fallback_attempt()
    base = evaluate_method_request(
        _request(
            PIXHAWK_RTK_LIVE,
            attempt_payload=attempt.canonical_payload(),
        ),
        worker_started_monotonic_ns=2_000,
    )
    outcomes = tuple(
        replace(
            base,
            method_id=method_id,
            method_instance_id=f"{method_id}:worker",
        )
        for method_id in MEASUREMENT_METHOD_IDS
    )
    bundle = MethodEvaluationBundle(
        measurement_id=base.measurement_id,
        pair_id=base.pair_id,
        pair_sequence=base.pair_sequence,
        snapshot_sha256=base.snapshot_sha256,
        request_generation=base.request_generation,
        outcomes=outcomes,
    )
    history: list[dict[str, object]] = []
    panel = SimpleNamespace(
        _method_evaluation_committed=set(),
        method_evaluation_bundle_rows=[],
        distance_recording_enabled=True,
        _measurement_method_result_counters={},
        _append_distance_history=lambda **kwargs: history.append(kwargs),
        _record_unresolved_pair=lambda *_args, **_kwargs: None,
        _set_distance_text=lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        measurement_view_controller,
        "calculate_measurement_geometry",
        lambda _attempt: (_ for _ in ()).throw(AssertionError("GUI recalculation forbidden")),
    )

    MeasurementViewController(panel)._commit_method_evaluation_bundle(bundle)

    assert len(history) == 1
    assert history[0]["distance_m"] == pytest.approx(9.0, rel=1e-3)
    assert len(panel.method_evaluation_bundle_rows) == 1


def test_successful_preview_distance_formats_input_integrity_without_scope_error() -> None:
    attempt = _attempt()
    pair = SimpleNamespace(
        left=attempt.left_detection,
        right=attempt.right_detection,
    )
    preview = PreviewMeasurementResult(
        key=attempt.canonical_sha256(),
        attempt=attempt,
        geometry_result=calculate_measurement_geometry(attempt),
        optical_metrics={},
        error="",
        submitted_monotonic_s=1.0,
        completed_monotonic_s=1.1,
    )
    distance_text: list[str] = []
    panel = SimpleNamespace(
        geometry_comparison_rows=(),
        _rtsp_urls_are_same_stream=lambda: False,
        _matched_detection_pair=lambda: pair,
        _geometry_comparison_rows=lambda _pair: (),
        gnss_operation_gate_active=False,
        _measurement_display_mode=lambda: "detection",
        _show_measurement_pair=lambda *_args: None,
        _clear_measurement_pair_display=lambda: None,
        _build_measurement_attempt=lambda _pair: attempt,
        _update_triangulation_quality=lambda **_kwargs: None,
        _set_distance_text=lambda text, **_kwargs: distance_text.append(text),
        _append_distance_history=lambda **_kwargs: True,
        _record_unresolved_pair=lambda *_args, **_kwargs: None,
    )
    controller = MeasurementViewController(panel)
    controller._multi_method_measurement_active = lambda: False
    controller._select_measurement_method_for_attempt = lambda: SimpleNamespace(
        requires_live_gnss=False
    )
    controller._preview_measurement_result = lambda *_args: preview

    controller._refresh_distance()

    assert distance_text
    assert "input=VERIFIED" in distance_text[-1]


def test_atomic_bundle_materializes_eight_verified_method_jsonl_streams(tmp_path) -> None:
    requests = _blocked_requests(1)
    outcomes = tuple(
        evaluate_method_request(request, worker_started_monotonic_ns=2_000)
        for request in requests
    )
    bundle = MethodEvaluationBundle(
        measurement_id=requests[0].measurement_id,
        pair_id=requests[0].pair_id,
        pair_sequence=requests[0].pair_sequence,
        snapshot_sha256=requests[0].snapshot_sha256,
        request_generation=requests[0].request_generation,
        outcomes=outcomes,
    )
    measurement_root = (
        tmp_path / "session-1" / "measurements" / "measurement-1"
    )

    saved = save_session_files(
        measurement_root,
        metadata={"session_id": "session-1", "measurement_id": "measurement-1"},
        pair_rows=[],
        method_evaluation_bundle_rows=[bundle.to_record()],
    )

    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    assert manifest["completeness"]["complete"] is True
    bundle_stream = read_measurement_stream(
        measurement_root,
        "raw/method_evaluation_bundles",
    )
    assert len(bundle_stream.rows) == 1
    assert MethodEvaluationBundle.from_mapping(bundle_stream.rows[0]) == bundle
    for method_id in MEASUREMENT_METHOD_IDS:
        stream = read_measurement_stream(
            measurement_root,
            f"raw/method_results/{method_id}",
        )
        assert stream.path == (
            measurement_root / f"data/raw/method_results/{method_id}.jsonl"
        ).resolve()
        assert len(stream.rows) == 1
        assert stream.rows[0]["method_id"] == method_id
        assert stream.rows[0]["method_evaluation_bundle_sha256"] == (
            bundle.canonical_sha256()
        )
    review = _measurement_review_mapping(
        measurement_root,
        session_root=measurement_root.parent.parent,
    )
    assert review is not None
    assert all(
        len(review["method_results"][method_id]) == 1
        for method_id in MEASUREMENT_METHOD_IDS
    )


def test_atomic_bundle_spool_recovers_after_application_restart(tmp_path) -> None:
    requests = _blocked_requests(1)
    bundle = MethodEvaluationBundle(
        measurement_id="measurement-1",
        pair_id=requests[0].pair_id,
        pair_sequence=1,
        snapshot_sha256=requests[0].snapshot_sha256,
        request_generation=requests[0].request_generation,
        outcomes=tuple(
            evaluate_method_request(request, worker_started_monotonic_ns=2_000)
            for request in requests
        ),
    )
    output = tmp_path / "measurement-1"
    write_recovery_metadata(
        output,
        {"session_id": "measurement-1", "measurement_id": "measurement-1"},
    )
    spool = SpoolRows()
    spool.bind(output / ".spool" / "method_evaluation_bundles.jsonl")
    spool.append(bundle.to_record())
    spool.close()

    recovered = load_retryable_session(output)

    assert len(recovered.method_evaluation_bundle_rows) == 1
    assert MethodEvaluationBundle.from_mapping(
        recovered.method_evaluation_bundle_rows[0]
    ) == bundle
