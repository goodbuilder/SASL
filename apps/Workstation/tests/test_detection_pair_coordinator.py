from __future__ import annotations

import multiprocessing
import time
from collections.abc import Mapping
from types import SimpleNamespace

import pytest

from Workstation.workstation_core.detection_pair_coordinator import (
    DetectionPairCoordinator,
    PairingPolicy,
)
from Workstation.workstation_core.stereo import DetectionObservation
from Workstation.workstation_core.native_evidence_writer import (
    RingPairReservation,
    reserve_ring_pair,
)
from Workstation.workstation_core.native_frame_ring import (
    FrameKey,
    SharedNativeFrameRing,
)


def _detection(
    side: str,
    timestamp_s: float,
    *,
    generation: int = 1,
    frame_id: int = 1,
    rtc_s: float | None = None,
    timestamp_metadata: Mapping[str, object] | None = None,
) -> DetectionObservation:
    metadata = (
        dict(timestamp_metadata)
        if timestamp_metadata is not None
        else {
            "camera_id": side,
            "stream_generation": generation,
            "frame_id": frame_id,
            "frame_key": f"{side}:{generation}:{frame_id}",
            "native_ring_available": True,
        }
    )
    if rtc_s is not None:
        metadata["rtcp_frame_ntp_unix_s"] = rtc_s
    return DetectionObservation(
        camera=side,
        timestamp_s=timestamp_s,
        width=1280,
        height=720,
        x1=100.0,
        y1=100.0,
        x2=200.0,
        y2=200.0,
        confidence=0.9,
        class_id=0,
        class_name="person",
        timestamp_metadata=metadata,
        candidate_class_ids=(0,),
    )


def _await_result(coordinator: DetectionPairCoordinator):
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        result = coordinator.poll_latest()
        if result is not None:
            return result
        time.sleep(0.005)
    raise AssertionError("pairing result timeout")


def _await_result_where(coordinator: DetectionPairCoordinator, predicate):
    deadline = time.monotonic() + 2.0
    last = None
    while time.monotonic() < deadline:
        result = coordinator.poll_latest()
        if result is not None:
            last = result
            if predicate(result):
                return result
        time.sleep(0.005)
    raise AssertionError(f"matching pairing result timeout; last={last!r}")


def test_pairing_runs_off_gui_and_preserves_exact_frame_keys() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(timestamp_basis="camera_rtc_required", revision=4)
        coordinator.submit(
            "left",
            _detection("left", 1.0, frame_id=10, rtc_s=100.0),
            policy,
        )
        coordinator.submit(
            "right",
            _detection("right", 1.1, frame_id=20, rtc_s=100.01),
            policy,
        )
        result = _await_result(coordinator)
        assert result.pair is not None
        assert result.pair.left.timestamp_metadata["frame_key"] == "left:1:10"
        assert result.pair.right.timestamp_metadata["frame_key"] == "right:1:20"
        assert result.uses_app_fallback is False
        assert result.policy_revision == 4
    finally:
        assert coordinator.close()


def test_missing_camera_rtc_uses_explicit_app_fallback() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="camera_rtc_required",
            allow_app_fallback=True,
        )
        coordinator.submit("left", _detection("left", 5.0), policy)
        coordinator.submit("right", _detection("right", 5.01), policy)
        result = _await_result(coordinator)
        assert result.pair is not None
        assert result.uses_app_fallback is True
        assert result.timing is not None
        assert result.timing.requested_basis == "camera_rtc_required"
        assert result.timing.effective_basis == "app_receive_monotonic"
        assert result.timing.fallback_reason == "RTCP_TIMESTAMP_UNAVAILABLE"
        assert result.advisories == ()
    finally:
        assert coordinator.close()


def test_healthy_rtcp_preference_actually_selects_rtcp_pair() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="camera_rtcp",
            max_delta_s=0.100,
            allow_app_fallback=True,
            rtcp_clock_verified=True,
            require_current_clock_epoch=True,
        )
        metadata = {
            "clock_epoch_id": "epoch-7",
            "clock_epoch_state": "current",
        }
        coordinator.submit(
            "left",
            _detection(
                "left",
                10.000,
                rtc_s=1_000.000,
                timestamp_metadata={
                    **metadata,
                    "app_receive_monotonic_ns": 10_000_000_000,
                },
            ),
            policy,
        )
        coordinator.submit(
            "right",
            _detection(
                "right",
                10.090,
                rtc_s=1_000.010,
                timestamp_metadata={
                    **metadata,
                    "app_receive_monotonic_ns": 10_090_000_000,
                },
            ),
            policy,
        )

        result = _await_result(coordinator)

        assert result.pair is not None
        assert result.timing is not None
        assert result.timing.requested_basis == "camera_rtcp"
        assert result.timing.effective_basis == "camera_rtcp"
        assert result.timing.signed_delta_ns == 10_000_000
        assert result.timing.fallback_reason == ""
        assert result.uses_app_fallback is False
    finally:
        assert coordinator.close()


@pytest.mark.parametrize(
    ("clock_verified", "left_metadata", "right_metadata", "rtc_pair", "reason"),
    (
        (
            False,
            {"clock_epoch_id": "epoch", "clock_epoch_state": "current"},
            {"clock_epoch_id": "epoch", "clock_epoch_state": "current"},
            (100.0, 100.01),
            "CLOCK_NOT_VERIFIED",
        ),
        (
            True,
            {"clock_epoch_id": "epoch", "clock_epoch_state": "transition"},
            {"clock_epoch_id": "epoch", "clock_epoch_state": "current"},
            (100.0, 100.01),
            "CLOCK_EPOCH_TRANSITION",
        ),
        (
            True,
            {"clock_epoch_id": "left", "clock_epoch_state": "current"},
            {"clock_epoch_id": "right", "clock_epoch_state": "current"},
            (100.0, 100.01),
            "CLOCK_EPOCH_MISMATCH",
        ),
        (
            True,
            {"clock_epoch_id": "epoch", "clock_epoch_state": "current"},
            {"clock_epoch_id": "epoch", "clock_epoch_state": "current"},
            (100.0, None),
            "RTCP_TIMESTAMP_UNAVAILABLE",
        ),
        (
            True,
            {"clock_epoch_id": "epoch", "clock_epoch_state": "current"},
            {"clock_epoch_id": "epoch", "clock_epoch_state": "current"},
            (100.0, 100.101),
            "RTCP_DELTA_EXCEEDED",
        ),
    ),
)
def test_rtcp_timing_faults_fall_back_to_exact_app_receive(
    clock_verified,
    left_metadata,
    right_metadata,
    rtc_pair,
    reason,
) -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="camera_rtcp",
            max_delta_s=0.100,
            allow_app_fallback=True,
            rtcp_clock_verified=clock_verified,
            require_current_clock_epoch=True,
        )
        for side, timestamp_s, app_ns, rtc_s, metadata in (
            ("left", 5.000, 5_000_000_000, rtc_pair[0], left_metadata),
            ("right", 5.020, 5_020_000_000, rtc_pair[1], right_metadata),
        ):
            coordinator.submit(
                side,
                _detection(
                    side,
                    timestamp_s,
                    rtc_s=rtc_s,
                    timestamp_metadata={
                        **metadata,
                        "app_receive_monotonic_ns": app_ns,
                    },
                ),
                policy,
            )

        result = _await_result(coordinator)

        assert result.pair is not None
        assert result.timing is not None
        assert result.timing.effective_basis == "app_receive_monotonic"
        assert result.timing.signed_delta_ns == 20_000_000
        assert result.timing.fallback_reason == reason
        assert result.primary_rejection_reason == reason
        assert result.uses_app_fallback is True
        assert result.advisories == ()
    finally:
        assert coordinator.close()


def test_rtcp_and_app_timing_failure_returns_precise_final_rejection() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="camera_rtcp",
            max_delta_s=0.100,
            allow_app_fallback=True,
            rtcp_clock_verified=False,
            rtcp_unavailable_reason="CLOCK_NOT_VERIFIED",
            require_current_clock_epoch=True,
        )
        coordinator.submit("left", _detection("left", 1.0), policy)
        coordinator.submit("right", _detection("right", 1.100_000_001), policy)

        result = _await_result(coordinator)

        assert result.pair is None
        assert result.primary_rejection_reason == "CLOCK_NOT_VERIFIED"
        assert result.fallback_rejection_reason == "APP_RECEIVE_DELTA_EXCEEDED"
        assert result.rejection_reason == "APP_RECEIVE_DELTA_EXCEEDED"
    finally:
        assert coordinator.close()


def test_latest_missing_rtcp_does_not_reuse_an_older_healthy_rtcp_pair() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="camera_rtcp",
            allow_app_fallback=True,
            rtcp_clock_verified=True,
            require_current_clock_epoch=True,
        )
        epoch = {"clock_epoch_id": "epoch", "clock_epoch_state": "current"}
        coordinator.submit(
            "left",
            _detection("left", 1.0, frame_id=1, rtc_s=100.0, timestamp_metadata=epoch),
            policy,
        )
        coordinator.submit(
            "right",
            _detection("right", 1.01, frame_id=1, rtc_s=100.01, timestamp_metadata=epoch),
            policy,
        )
        healthy = _await_result_where(
            coordinator,
            lambda result: result.pair is not None and not result.uses_app_fallback,
        )
        assert healthy.pair.left.timestamp_s == pytest.approx(1.0)

        coordinator.submit(
            "left",
            _detection("left", 2.0, frame_id=2, timestamp_metadata=epoch),
            policy,
        )
        coordinator.submit(
            "right",
            _detection("right", 2.01, frame_id=2, timestamp_metadata=epoch),
            policy,
        )
        fallback = _await_result_where(
            coordinator,
            lambda result: result.pair is not None and result.uses_app_fallback,
        )

        assert fallback.pair.left.timestamp_s == pytest.approx(2.0)
        assert fallback.pair.right.timestamp_s == pytest.approx(2.01)
        assert fallback.timing.fallback_reason == "RTCP_TIMESTAMP_UNAVAILABLE"
    finally:
        assert coordinator.close()


def test_time_dependent_pairing_rejects_stus_clock_epoch_transition() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(timestamp_basis="camera_rtc_required")
        coordinator.submit(
            "left",
            _detection(
                "left",
                1.0,
                rtc_s=100.0,
                timestamp_metadata={
                    "clock_epoch_id": "epoch-2",
                    "clock_epoch_state": "transition",
                },
            ),
            policy,
        )
        coordinator.submit(
            "right",
            _detection(
                "right",
                1.0,
                rtc_s=100.0,
                timestamp_metadata={
                    "clock_epoch_id": "epoch-1",
                    "clock_epoch_state": "current",
                },
            ),
            policy,
        )
        result = _await_result(coordinator)
        assert result.pair is None
        assert result.rejection_reason == "CLOCK_EPOCH_TRANSITION"
        assert result.uses_app_fallback is False
    finally:
        assert coordinator.close()


def test_time_dependent_pairing_requires_same_current_clock_epoch() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(timestamp_basis="camera_rtc_required")
        for side, epoch in (("left", "epoch-left"), ("right", "epoch-right")):
            coordinator.submit(
                side,
                _detection(
                    side,
                    1.0,
                    rtc_s=100.0,
                    timestamp_metadata={
                        "clock_epoch_id": epoch,
                        "clock_epoch_state": "current",
                    },
                ),
                policy,
            )
        result = _await_result(coordinator)
        assert result.pair is None
        assert result.rejection_reason == "CLOCK_EPOCH_MISMATCH"
    finally:
        assert coordinator.close()


def test_app_receive_pairing_ignores_ntp_value_and_clock_epoch() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="app_receive_monotonic",
            max_delta_s=0.100,
            allow_app_fallback=False,
        )
        coordinator.submit(
            "left",
            _detection(
                "left",
                10.000,
                rtc_s=100.0,
                timestamp_metadata={
                    "clock_epoch_id": "epoch-left",
                    "clock_epoch_state": "transition",
                    "rtcp_frame_ntp_unix_s": 100.0,
                },
            ),
            policy,
        )
        coordinator.submit(
            "right",
            _detection(
                "right",
                10.020,
                rtc_s=32_500.0,
                timestamp_metadata={
                    "clock_epoch_id": "epoch-right",
                    "clock_epoch_state": "current",
                    "rtcp_frame_ntp_unix_s": 32_500.0,
                },
            ),
            policy,
        )

        result = _await_result(coordinator)

        assert result.pair is not None
        assert result.pair.delta_s == pytest.approx(0.020)
        assert result.uses_app_fallback is False
    finally:
        assert coordinator.close()


def test_app_receive_pairing_hard_rejects_over_100ms_even_when_ntp_is_close() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="app_receive_monotonic",
            max_delta_s=0.100,
            allow_app_fallback=False,
        )
        coordinator.submit(
            "left",
            _detection("left", 20.0, rtc_s=1_000.000),
            policy,
        )
        coordinator.submit(
            "right",
            _detection("right", 20.1001, rtc_s=1_000.001),
            policy,
        )

        result = _await_result(coordinator)

        assert result.pair is None
        assert result.rejection_reason == "APP_RECEIVE_DELTA_EXCEEDED"
    finally:
        assert coordinator.close()


@pytest.mark.parametrize("configured_limit_s", [0.100, 0.500, float("inf")])
def test_app_receive_pairing_rejects_exact_100ms_plus_one_ns(
    configured_limit_s: float,
) -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="app_receive_monotonic",
            max_delta_s=configured_limit_s,
        )
        coordinator.submit(
            "left",
            _detection(
                "left",
                10.0,
                timestamp_metadata={"app_receive_monotonic_ns": 10_000_000_000},
            ),
            policy,
        )
        coordinator.submit(
            "right",
            _detection(
                "right",
                10.100000001,
                timestamp_metadata={"app_receive_monotonic_ns": 10_100_000_001},
            ),
            policy,
        )

        result = _await_result(coordinator)

        assert result.pair is None
        assert result.rejection_reason == "APP_RECEIVE_DELTA_EXCEEDED"
    finally:
        assert coordinator.close()


def test_app_receive_pairing_accepts_exact_100ms_without_ntp() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="app_receive_monotonic",
            max_delta_s=0.100,
            allow_app_fallback=False,
        )
        coordinator.submit("left", _detection("left", 30.0), policy)
        coordinator.submit("right", _detection("right", 30.1), policy)

        result = _await_result(coordinator)

        assert result.pair is not None
        assert result.pair.delta_s == pytest.approx(0.100)
    finally:
        assert coordinator.close()


def test_app_receive_pairing_rejects_invalid_exact_monotonic_timestamp() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="app_receive_monotonic",
            max_delta_s=0.100,
        )
        coordinator.submit(
            "left",
            _detection(
                "left",
                40.0,
                timestamp_metadata={"app_receive_monotonic_ns": -1},
            ),
            policy,
        )
        coordinator.submit("right", _detection("right", 40.01), policy)

        result = _await_result(coordinator)

        assert result.pair is None
        assert result.rejection_reason == "APP_RECEIVE_TIMESTAMP_INVALID"
    finally:
        assert coordinator.close()


@pytest.mark.parametrize("invalid_exact_ns", [True, 1.5, "1000000000"])
def test_app_receive_pairing_rejects_noninteger_exact_timestamp(
    invalid_exact_ns: object,
) -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="app_receive_monotonic",
            max_delta_s=0.100,
        )
        coordinator.submit(
            "left",
            _detection(
                "left",
                1.0,
                timestamp_metadata={"app_receive_monotonic_ns": invalid_exact_ns},
            ),
            policy,
        )
        coordinator.submit(
            "right",
            _detection(
                "right",
                1.02,
                timestamp_metadata={"app_receive_monotonic_ns": 1_020_000_000},
            ),
            policy,
        )

        result = _await_result(coordinator)

        assert result.pair is None
        assert result.rejection_reason == "APP_RECEIVE_TIMESTAMP_INVALID"
    finally:
        assert coordinator.close()


def test_workstation_live_pairing_policy_defaults_to_app_receive_and_caps_100ms() -> None:
    from Workstation.measurement_view_controller import MeasurementViewController

    panel = SimpleNamespace(
        _yolo_pair_max_delta_s=lambda: float("inf"),
        _yolo_vertical_max_delta_px=lambda: 25.0,
    )
    controller = MeasurementViewController(panel)

    policy = controller._pairing_policy_snapshot()

    assert policy.timestamp_basis == "app_receive_monotonic"
    assert policy.max_delta_s == pytest.approx(0.100)
    assert policy.max_vertical_delta_px == pytest.approx(25.0)
    assert policy.allow_app_fallback is False


def test_workstation_live_pairing_policy_uses_verified_rtcp_preference() -> None:
    from Workstation.measurement_view_controller import MeasurementViewController

    panel = SimpleNamespace(
        yolo_gate_time_basis_combo=SimpleNamespace(currentData=lambda: "camera_rtcp"),
        _yolo_pair_max_delta_s=lambda: 0.080,
        _yolo_vertical_max_delta_px=lambda: 25.0,
        _measurement_id="",
        camera_sync_status="verified",
        frame_timing_status="current",
    )
    controller = MeasurementViewController(panel)

    policy = controller._pairing_policy_snapshot()

    assert policy.timestamp_basis == "camera_rtcp"
    assert policy.max_delta_s == pytest.approx(0.080)
    assert policy.allow_app_fallback is True
    assert policy.rtcp_clock_verified is True
    assert policy.require_current_clock_epoch is True


def test_running_measurement_keeps_frozen_request_until_next_measurement() -> None:
    from Workstation.measurement_view_controller import MeasurementViewController

    run_metadata: dict[str, object] = {}
    panel = SimpleNamespace(
        yolo_gate_time_basis_combo=SimpleNamespace(currentData=lambda: "camera_rtcp"),
        _measurement_id="run-1",
        _measurement_package=None,
        _measurement_run_metadata=run_metadata,
        _measurement_method_config_token=("run-1", id(run_metadata)),
        _measurement_active_time_basis_requested="app_receive_monotonic",
    )
    controller = MeasurementViewController(panel)

    assert controller.requested_time_basis() == "app_receive_monotonic"
    panel._measurement_id = ""
    assert controller.requested_time_basis() == "camera_rtcp"


def test_freeze_consumes_coordinator_rtcp_decision_but_attitude_uses_app_time() -> None:
    from Workstation.measurement_view_controller import MeasurementViewController
    from Workstation.workstation_core.detection_pair_coordinator import PairTimingDecision

    left = _detection(
        "left",
        10.000,
        rtc_s=1_000.000,
        timestamp_metadata={
            "app_receive_monotonic_ns": 10_000_000_000,
            "rtcp_frame_ntp_unix_s": 1_000.000,
            "clock_epoch_id": "epoch-1",
            "clock_epoch_state": "current",
        },
    )
    right = _detection(
        "right",
        10.090,
        rtc_s=1_000.010,
        timestamp_metadata={
            "app_receive_monotonic_ns": 10_090_000_000,
            "rtcp_frame_ntp_unix_s": 1_000.010,
            "clock_epoch_id": "epoch-1",
            "clock_epoch_state": "current",
        },
    )
    pair = SimpleNamespace(left=left, right=right)
    timing = PairTimingDecision(
        requested_basis="camera_rtcp",
        effective_basis="camera_rtcp",
        left_timestamp_s=1_000.000,
        right_timestamp_s=1_000.010,
        signed_delta_ns=10_000_000,
        abs_delta_ns=10_000_000,
    )
    attitude_queries: dict[str, list[float]] = {"left": [], "right": []}

    def attitude_buffer(side: str):
        return SimpleNamespace(
            nearest=lambda timestamp_s: (
                attitude_queries[side].append(timestamp_s) or None
            )
        )

    panel = SimpleNamespace(
        _coordinated_pairing_result=SimpleNamespace(pair=pair, timing=timing),
        _detection_pair_coordinator_active=True,
        _last_frozen_measurement_pair=None,
        _coordinated_detection_pair_reservation=None,
        attitude_buffers={
            "left": attitude_buffer("left"),
            "right": attitude_buffer("right"),
        },
        _measurement_started_monotonic_s=None,
        _measurement_attempt_counter=0,
        _measurement_attempt_generation_id="generation-1",
        _measurement_id="measurement-1",
        _yolo_pair_max_delta_s=lambda: 0.100,
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_gps_states = lambda: (None, None, ())
    controller._ftp_command_round_sync = lambda _pair: {}

    frozen = controller._freeze_measurement_pair_inputs(pair)

    assert frozen["requested_timestamp_basis"] == "camera_rtcp"
    assert frozen["timestamp_basis"] == "camera_rtcp"
    assert frozen["left_pairing_timestamp_s"] == pytest.approx(1_000.000)
    assert frozen["right_pairing_timestamp_s"] == pytest.approx(1_000.010)
    assert frozen["snapshot"].capture_sync["selected_pair_delta_ns"] == 10_000_000
    assert frozen["snapshot"].capture_sync["app_receive_pair_delta_ns"] == 90_000_000
    assert attitude_queries == {"left": [10.0], "right": [10.09]}


def test_policy_revision_flushes_old_candidates_and_rejects_rollback() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        old = PairingPolicy(timestamp_basis="app_receive_monotonic", revision=1)
        new = PairingPolicy(timestamp_basis="app_receive_monotonic", revision=2)
        coordinator.submit("left", _detection("left", 1.0), old)
        _await_result_where(
            coordinator,
            lambda result: result.rejection_reason == "missing_right_detection",
        )

        coordinator.submit("right", _detection("right", 1.01), new)
        flushed = _await_result_where(
            coordinator,
            lambda result: result.rejection_reason == "missing_left_detection",
        )
        assert flushed.policy_revision == 2

        coordinator.submit("left", _detection("left", 1.0, frame_id=99), old)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and coordinator.metrics()["stale_policy_inputs"] < 1:
            time.sleep(0.005)
        assert coordinator.metrics()["stale_policy_inputs"] == 1

        coordinator.submit("left", _detection("left", 1.0, frame_id=2), new)
        paired = _await_result_where(coordinator, lambda result: result.pair is not None)
        assert paired.policy_revision == 2
        assert coordinator.metrics()["policy_revisions"] >= 2
    finally:
        assert coordinator.close()


def test_wait_status_distinguishes_timing_fallback_failure_from_correspondence() -> None:
    from Workstation.measurement_view_controller import MeasurementViewController

    panel = SimpleNamespace(
        _last_correspondence_rejection_reason="APP_RECEIVE_DELTA_EXCEEDED",
        _coordinated_pairing_result=SimpleNamespace(
            primary_rejection_reason="CLOCK_NOT_VERIFIED"
        ),
        detection_buffers={"left": [object()], "right": [object()]},
        camera_stream_stats={"left": {}, "right": {}},
        _yolo_time_delta_gate_enabled=lambda: True,
        _yolo_time_basis=lambda: "camera_rtcp",
    )
    controller = MeasurementViewController(panel)

    assert controller._detection_wait_status() == (
        "거리: timing pair 대기 (APP_RECEIVE_DELTA_EXCEEDED)"
    )
    detail = controller._detection_wait_detail()
    assert "timing" in detail
    assert "preferred=CLOCK_NOT_VERIFIED" in detail
    assert "correspondence" not in detail


def test_new_stream_generation_evicts_old_camera_detections() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(timestamp_basis="detection")
        coordinator.submit(
            "left",
            _detection("left", 1.0, generation=1, frame_id=1),
            policy,
        )
        coordinator.submit(
            "left",
            _detection("left", 2.0, generation=2, frame_id=1),
            policy,
        )
        coordinator.submit(
            "right",
            _detection("right", 2.01, generation=1, frame_id=1),
            policy,
        )
        result = _await_result(coordinator)
        assert result.pair is not None
        assert result.pair.left.timestamp_metadata["stream_generation"] == 2
        assert coordinator.metrics()["left_buffered"] == 1
    finally:
        assert coordinator.close()


def test_delayed_old_generation_is_rejected_without_rolling_buffer_back() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(timestamp_basis="detection")
        coordinator.submit(
            "left",
            _detection("left", 2.0, generation=2, frame_id=1),
            policy,
        )
        coordinator.submit(
            "right",
            _detection("right", 2.01, generation=2, frame_id=1),
            policy,
        )
        paired = _await_result_where(coordinator, lambda result: result.pair is not None)
        assert paired.pair.left.timestamp_metadata["stream_generation"] == 2

        coordinator.submit(
            "left",
            _detection("left", 1.0, generation=1, frame_id=99),
            policy,
        )
        rejected = _await_result_where(
            coordinator,
            lambda result: result.rejection_reason == "stream_generation_rollback",
        )
        assert rejected.pair is None
        metrics = coordinator.metrics()
        assert metrics["generation_rollbacks"] == 1
        assert metrics["left_buffered"] == 1
    finally:
        assert coordinator.close()


def test_stale_and_explicit_no_detection_inputs_evict_old_pairs() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="detection",
            max_observation_age_s=0.02,
        )
        coordinator.submit("left", _detection("left", 1.0), policy)
        time.sleep(0.04)
        coordinator.submit("right", _detection("right", 1.01), policy)
        stale = _await_result_where(
            coordinator,
            lambda result: result.rejection_reason == "missing_left_detection",
        )
        assert stale.pair is None
        assert coordinator.metrics()["stale_evicted"] >= 1

        coordinator.submit("left", _detection("left", 1.011, frame_id=2), policy)
        assert _await_result_where(
            coordinator,
            lambda result: result.pair is not None,
        ).pair is not None

        coordinator.submit("left", None, policy)
        no_detection = _await_result_where(
            coordinator,
            lambda result: result.rejection_reason == "missing_left_detection",
        )
        assert no_detection.pair is None
        assert coordinator.metrics()["no_detection_evicted"] >= 1
    finally:
        assert coordinator.close()


def test_silent_stream_still_expires_buffer_on_explicit_age_deadline() -> None:
    coordinator = DetectionPairCoordinator()
    try:
        policy = PairingPolicy(
            timestamp_basis="detection",
            max_observation_age_s=0.02,
        )
        coordinator.submit("left", _detection("left", 1.0), policy)
        _await_result_where(
            coordinator,
            lambda result: result.rejection_reason == "missing_right_detection",
        )

        expired = _await_result_where(
            coordinator,
            lambda result: result.rejection_reason == "missing_left_detection",
        )
        assert expired.pair is None
        assert coordinator.metrics()["stale_evicted"] == 1
        assert coordinator.metrics()["left_buffered"] == 0
    finally:
        assert coordinator.close()


def test_accepted_pair_is_reserved_before_more_than_ring_depth_gui_delay() -> None:
    context = multiprocessing.get_context("spawn")
    rings = {
        side: SharedNativeFrameRing.create(
            camera_id=side,
            stream_generation=5,
            capacity=2,
            max_width=2,
            max_height=1,
            channels=3,
            mp_context=context,
        )
        for side in ("left", "right")
    }

    def reserve(pair, sources):
        frames = []
        for side, observation in (("left", pair.left), ("right", pair.right)):
            key = FrameKey.from_mapping(observation.timestamp_metadata)
            frames.append((side, sources[side], key, observation.timestamp_metadata))
        return reserve_ring_pair(frames)

    coordinator = DetectionPairCoordinator(reservation_factory=reserve)
    try:
        descriptors = {
            side: ring.publish_bytes(
                bytes([index]) * 6,
                width=2,
                height=1,
                channels=3,
                timestamp_s=1.0 + index * 0.01,
            )
            for index, (side, ring) in enumerate(rings.items(), start=1)
        }
        policy = PairingPolicy(timestamp_basis="detection")
        for side in ("left", "right"):
            descriptor = descriptors[side]
            coordinator.submit(
                side,
                _detection(
                    side,
                    descriptor.timestamp_s or 0.0,
                    generation=5,
                    frame_id=descriptor.key.frame_id,
                    timestamp_metadata=descriptor.to_mapping(),
                ),
                policy,
                reservation_source=rings[side],
            )

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and coordinator.metrics()["paired"] < 1:
            time.sleep(0.005)
        assert coordinator.metrics()["paired"] == 1

        # Simulate a GUI callback delayed beyond the two-slot ring depth.
        for value in range(10, 16):
            for ring in rings.values():
                ring.publish_bytes(
                    bytes([value]) * 6,
                    width=2,
                    height=1,
                    channels=3,
                    timestamp_s=float(value),
                )

        result = _await_result_where(coordinator, lambda item: item.pair is not None)
        assert isinstance(result.reservation, RingPairReservation)
        assert result.reservation.active is True
        for side, ring in rings.items():
            assert ring.descriptor(descriptors[side].key) is not None
            assert ring.metrics()["active_leases"] == 1
        assert result.reservation.release() is True
    finally:
        assert coordinator.close()
        for ring in rings.values():
            ring.close()
            ring.unlink()


def test_coordinator_stop_releases_unpolled_pair_reservation() -> None:
    context = multiprocessing.get_context("spawn")
    rings = {
        side: SharedNativeFrameRing.create(
            camera_id=side,
            stream_generation=6,
            capacity=1,
            max_width=2,
            max_height=1,
            channels=3,
            mp_context=context,
        )
        for side in ("left", "right")
    }

    def reserve(pair, sources):
        return reserve_ring_pair(
            tuple(
                (
                    side,
                    sources[side],
                    FrameKey.from_mapping(observation.timestamp_metadata),
                    observation.timestamp_metadata,
                )
                for side, observation in (("left", pair.left), ("right", pair.right))
            )
        )

    coordinator = DetectionPairCoordinator(reservation_factory=reserve)
    try:
        policy = PairingPolicy(timestamp_basis="detection")
        for index, (side, ring) in enumerate(rings.items(), start=1):
            descriptor = ring.publish_bytes(
                bytes([index]) * 6,
                width=2,
                height=1,
                channels=3,
                timestamp_s=1.0 + index * 0.01,
            )
            coordinator.submit(
                side,
                _detection(
                    side,
                    descriptor.timestamp_s or 0.0,
                    generation=6,
                    timestamp_metadata=descriptor.to_mapping(),
                ),
                policy,
                reservation_source=ring,
            )
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and coordinator.metrics()["paired"] < 1:
            time.sleep(0.005)
        assert all(ring.metrics()["active_leases"] == 1 for ring in rings.values())
        assert coordinator.close() is True
        assert all(ring.metrics()["active_leases"] == 0 for ring in rings.values())
    finally:
        coordinator.close()
        for ring in rings.values():
            ring.close()
            ring.unlink()


def test_input_and_result_queues_are_bounded_latest_state() -> None:
    coordinator = DetectionPairCoordinator(input_capacity=1, buffer_capacity=2)
    try:
        policy = PairingPolicy(timestamp_basis="detection")
        for frame_id in range(50):
            coordinator.submit(
                "left",
                _detection("left", float(frame_id), frame_id=frame_id + 1),
                policy,
            )
        time.sleep(0.05)
        metrics = coordinator.metrics()
        assert metrics["input_capacity"] == 1
        assert metrics["left_buffered"] <= 2
        assert metrics["input_replaced"] > 0
    finally:
        assert coordinator.close()
