from __future__ import annotations

import json
import multiprocessing
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest
from Workstation.measurement_view_controller import MeasurementViewController
from Workstation.workstation_core.measurement_attempt import (
    CameraIntrinsicsSnapshot,
    CheckerboardExtrinsicsSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
)
from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS,
    MeasurementMethodStatus,
)
from Workstation.workstation_core.method_evaluation import (
    METHOD_EVALUATION_QUEUE_LIMIT,
    METHOD_EVALUATION_TIMEOUT_S,
    FrozenMeasurementPairSnapshot,
    MethodEvaluationBundle,
    MethodEvaluationOutcome,
    MethodEvaluationRequest,
    evaluate_method_request,
)
from Workstation.workstation_core.method_evaluation_workers import (
    MethodEvaluationCoordinator,
)
from Workstation.workstation_core.preview_measurement_worker import (
    PreviewMeasurementCoordinator,
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


def test_missing_frozen_input_produces_an_explicit_blocked_outcome() -> None:
    outcome = evaluate_method_request(
        _request(attempt_payload=None, blocked_reasons=("frozen comparison input missing",)),
        worker_started_monotonic_ns=2_000,
    )

    assert outcome.status == "BLOCKED"
    assert outcome.reasons == ("frozen comparison input missing",)
    assert outcome.queue_wait_ms == pytest.approx(0.001)


def test_bundle_requires_all_methods_in_canonical_order_and_verifies_hashes() -> None:
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
    )

    assert MethodEvaluationBundle.from_mapping(bundle.to_record()) == bundle
    with pytest.raises(ValueError, match="canonical order"):
        replace(bundle, outcomes=tuple(reversed(outcomes)))
    tampered = bundle.to_record()
    tampered["outcomes"][0]["status"] = "FAILED"
    with pytest.raises(ValueError, match="hash mismatch"):
        MethodEvaluationBundle.from_mapping(tampered)


def test_worker_policy_has_seven_timeouts_and_twenty_pair_queue_slots() -> None:
    assert tuple(METHOD_EVALUATION_TIMEOUT_S) == MEASUREMENT_METHOD_IDS[1:]
    assert METHOD_EVALUATION_TIMEOUT_S["horizon_hybrid"] == 1.0
    assert METHOD_EVALUATION_TIMEOUT_S["terrain_pnp"] == 10.0
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


def test_pair_queue_rejects_the_twenty_first_pair_as_persistable_backpressure() -> None:
    coordinator = MethodEvaluationCoordinator(start_workers=False)
    try:
        for sequence in range(1, METHOD_EVALUATION_QUEUE_LIMIT + 1):
            assert coordinator.submit(_blocked_requests(sequence)).accepted is True

        overflow = coordinator.submit(_blocked_requests(21))

        assert overflow.accepted is False
        assert overflow.backpressure_bundle is not None
        assert {row.status for row in overflow.backpressure_bundle.outcomes} == {
            "BACKPRESSURE_BLOCKED"
        }
    finally:
        coordinator.close()


def test_seven_comparison_workers_run_independently_and_timed_out_worker_restarts() -> None:
    coordinator = MethodEvaluationCoordinator(
        mp_context=multiprocessing.get_context("spawn"),
        process_target=_slow_horizon_worker,
    )
    primary_seen_at: float | None = None
    bundle = None
    poll_durations: list[float] = []
    started_at = time.monotonic()
    try:
        assert coordinator.submit(_blocked_requests(1)).accepted is True
        deadline = started_at + 8.0
        while time.monotonic() < deadline and bundle is None:
            poll_started_s = time.monotonic()
            polled = coordinator.poll()
            poll_durations.append(time.monotonic() - poll_started_s)
            if polled.primary_outcomes and primary_seen_at is None:
                primary_seen_at = time.monotonic()
            if polled.completed_bundles:
                bundle = polled.completed_bundles[0]
                break
            time.sleep(0.01)

        assert bundle is not None
        assert primary_seen_at is not None
        assert primary_seen_at - started_at < 1.0
        assert bundle.outcomes[MEASUREMENT_METHOD_IDS.index("horizon_hybrid")].status == "TIMEOUT"
        assert all(
            outcome.status == "BLOCKED"
            for outcome in bundle.outcomes
            if outcome.method_id != "horizon_hybrid"
        )
        assert coordinator.worker_generations["horizon_hybrid"] >= 2
        assert max(poll_durations, default=0.0) < 0.05
    finally:
        close_started_s = time.monotonic()
        coordinator.close()
        assert time.monotonic() - close_started_s < 0.05
        assert coordinator.wait_closed(10.0)


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
            not all(value >= 1 for value in coordinator.worker_generations.values())
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert all(value == 1 for value in coordinator.worker_generations.values())

    assert coordinator.closed


def test_gui_bundle_commit_reuses_worker_geometry_without_recalculation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import measurement_view_controller

    attempt = _attempt()
    base = evaluate_method_request(
        _request(attempt_payload=attempt.canonical_payload()),
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
