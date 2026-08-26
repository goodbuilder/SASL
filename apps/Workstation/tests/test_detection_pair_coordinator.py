from __future__ import annotations

import multiprocessing
import time
from collections.abc import Mapping

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
        assert "app receive time used" in result.advisories[0]
    finally:
        assert coordinator.close()


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
