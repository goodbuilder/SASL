from __future__ import annotations

import time

from Workstation.workstation_core.detection_pair_coordinator import (
    DetectionPairCoordinator,
    PairingPolicy,
)
from Workstation.workstation_core.stereo import DetectionObservation


def _detection(
    side: str,
    timestamp_s: float,
    *,
    generation: int = 1,
    frame_id: int = 1,
    rtc_s: float | None = None,
) -> DetectionObservation:
    metadata = {
        "camera_id": side,
        "stream_generation": generation,
        "frame_id": frame_id,
        "frame_key": f"{side}:{generation}:{frame_id}",
        "native_ring_available": True,
    }
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
