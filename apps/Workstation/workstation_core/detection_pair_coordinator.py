"""Bounded off-GUI stereo detection pairing coordinator."""

from __future__ import annotations

import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass

from Workstation.workstation_core.stereo import (
    DetectionObservation,
    correspondence_rejection_reason,
)
from Workstation.workstation_core.sync import match_nearest_pair


DEFAULT_DETECTION_INPUT_CAPACITY = 64
DEFAULT_DETECTION_BUFFER_CAPACITY = 64


@dataclass(frozen=True)
class PairingPolicy:
    timestamp_basis: str = "camera_rtc_required"
    max_delta_s: float = float("inf")
    max_vertical_delta_px: float = float("inf")
    allow_app_fallback: bool = True
    revision: int = 0


@dataclass(frozen=True)
class PairingResult:
    sequence: int
    pair: object | None
    rejection_reason: str
    advisories: tuple[str, ...]
    uses_app_fallback: bool
    policy_revision: int
    input_replaced: int
    result_replaced: int
    created_monotonic_s: float


def _finite_timestamp(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _camera_timestamp(observation: DetectionObservation) -> float | None:
    return _finite_timestamp(
        observation.timestamp_metadata.get("rtcp_frame_ntp_unix_s")
    )


def _stream_generation(observation: DetectionObservation) -> int:
    try:
        return int(observation.timestamp_metadata.get("stream_generation") or 0)
    except (TypeError, ValueError):
        return 0


class DetectionPairCoordinator:
    """Pair lightweight detections without running matcher logic on Qt's thread."""

    def __init__(
        self,
        *,
        input_capacity: int = DEFAULT_DETECTION_INPUT_CAPACITY,
        buffer_capacity: int = DEFAULT_DETECTION_BUFFER_CAPACITY,
    ) -> None:
        self.input_capacity = max(1, int(input_capacity))
        self.buffer_capacity = max(1, int(buffer_capacity))
        self._input: queue.Queue[
            tuple[str, DetectionObservation | None, PairingPolicy] | None
        ] = queue.Queue(maxsize=self.input_capacity)
        self._results: queue.Queue[PairingResult] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._input_replaced = 0
        self._result_replaced = 0
        self._submitted = 0
        self._paired = 0
        self._rejected = 0
        self._sequence = 0
        self._buffers: dict[str, deque[DetectionObservation]] = {
            "left": deque(maxlen=self.buffer_capacity),
            "right": deque(maxlen=self.buffer_capacity),
        }
        self._generation_by_camera = {"left": 0, "right": 0}
        self._thread = threading.Thread(
            target=self._run,
            name="detection-pair-coordinator",
            daemon=True,
        )
        self._thread.start()

    def submit(
        self,
        camera: str,
        detection: DetectionObservation | None,
        policy: PairingPolicy,
    ) -> None:
        message = (str(camera), detection, policy)
        try:
            self._input.put_nowait(message)
        except queue.Full:
            try:
                self._input.get_nowait()
                with self._lock:
                    self._input_replaced += 1
            except queue.Empty:
                pass
            try:
                self._input.put_nowait(message)
            except queue.Full:
                with self._lock:
                    self._input_replaced += 1
        with self._lock:
            self._submitted += 1

    def _timestamp_for_policy(
        self,
        observation: DetectionObservation,
        policy: PairingPolicy,
    ) -> float | None:
        basis = str(policy.timestamp_basis or "detection")
        if basis in {"camera_rtc_required", "camera_rtcp"}:
            return _camera_timestamp(observation)
        return float(observation.timestamp_s)

    def _match(self, policy: PairingPolicy) -> PairingResult:
        left = list(self._buffers["left"])
        right = list(self._buffers["right"])
        uses_app_fallback = False
        pair = match_nearest_pair(
            left,
            right,
            max_delta_s=float("inf"),
            timestamp=lambda observation: self._timestamp_for_policy(
                observation,
                policy,
            ),
        )
        advisories: list[str] = []
        if pair is None and policy.allow_app_fallback:
            pair = match_nearest_pair(
                left,
                right,
                max_delta_s=float("inf"),
                timestamp=lambda observation: float(observation.timestamp_s),
            )
            uses_app_fallback = pair is not None
            if uses_app_fallback:
                advisories.append(
                    "RTCP/NTP pairing unavailable; app receive time used"
                )

        reason = ""
        if pair is None:
            reason = (
                "missing_left_detection"
                if not left
                else "missing_right_detection"
                if not right
                else "no_timestamp_pair"
            )
        else:
            reason = correspondence_rejection_reason(pair.left, pair.right) or ""
            if not reason:
                if pair.delta_s > float(policy.max_delta_s):
                    advisories.append(
                        f"camera pair delta {pair.delta_s:.6f}s exceeds configured limit"
                    )
                vertical_delta = abs(pair.left.center[1] - pair.right.center[1])
                if vertical_delta > float(policy.max_vertical_delta_px):
                    advisories.append(
                        f"vertical bbox delta {vertical_delta:.3f}px exceeds configured limit"
                    )
            else:
                pair = None

        with self._lock:
            self._sequence += 1
            sequence = self._sequence
            if pair is None:
                self._rejected += 1
            else:
                self._paired += 1
            input_replaced = self._input_replaced
            result_replaced = self._result_replaced
        return PairingResult(
            sequence=sequence,
            pair=pair,
            rejection_reason=reason,
            advisories=tuple(dict.fromkeys(advisories)),
            uses_app_fallback=uses_app_fallback,
            policy_revision=int(policy.revision),
            input_replaced=input_replaced,
            result_replaced=result_replaced,
            created_monotonic_s=time.monotonic(),
        )

    def _publish_result(self, result: PairingResult) -> None:
        try:
            self._results.put_nowait(result)
            return
        except queue.Full:
            pass
        try:
            self._results.get_nowait()
            with self._lock:
                self._result_replaced += 1
        except queue.Empty:
            pass
        try:
            self._results.put_nowait(result)
        except queue.Full:
            with self._lock:
                self._result_replaced += 1

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                message = self._input.get(timeout=0.1)
            except queue.Empty:
                continue
            if message is None:
                return
            camera, detection, policy = message
            if camera not in self._buffers:
                continue
            if detection is not None:
                generation = _stream_generation(detection)
                previous_generation = self._generation_by_camera[camera]
                if generation > 0 and generation != previous_generation:
                    self._buffers[camera].clear()
                    self._generation_by_camera[camera] = generation
                if generation <= 0 or generation >= self._generation_by_camera[camera]:
                    self._buffers[camera].append(detection)
            self._publish_result(self._match(policy))

    def poll_latest(self) -> PairingResult | None:
        latest = None
        while True:
            try:
                latest = self._results.get_nowait()
            except queue.Empty:
                return latest

    def metrics(self) -> dict[str, int]:
        with self._lock:
            return {
                "input_capacity": self.input_capacity,
                "buffer_capacity": self.buffer_capacity,
                "submitted": self._submitted,
                "paired": self._paired,
                "rejected": self._rejected,
                "input_replaced": self._input_replaced,
                "result_replaced": self._result_replaced,
                "left_buffered": len(self._buffers["left"]),
                "right_buffered": len(self._buffers["right"]),
            }

    def close(self) -> bool:
        self._stop.set()
        try:
            self._input.put_nowait(None)
        except queue.Full:
            try:
                self._input.get_nowait()
            except queue.Empty:
                pass
            try:
                self._input.put_nowait(None)
            except queue.Full:
                pass
        self._thread.join(timeout=1.0)
        return not self._thread.is_alive()


__all__ = [
    "DEFAULT_DETECTION_BUFFER_CAPACITY",
    "DEFAULT_DETECTION_INPUT_CAPACITY",
    "DetectionPairCoordinator",
    "PairingPolicy",
    "PairingResult",
]
