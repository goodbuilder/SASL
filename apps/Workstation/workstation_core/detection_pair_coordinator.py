"""Bounded off-GUI stereo detection pairing coordinator."""

from __future__ import annotations

import math
import queue
import threading
import time
from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass

from Workstation.workstation_core.stereo import (
    DetectionObservation,
    correspondence_rejection_reason,
)
from Workstation.workstation_core.sync import match_nearest_pair


DEFAULT_DETECTION_INPUT_CAPACITY = 64
DEFAULT_DETECTION_BUFFER_CAPACITY = 64
DEFAULT_DETECTION_MAX_OBSERVATION_AGE_S = 1.0


@dataclass(frozen=True)
class PairingPolicy:
    timestamp_basis: str = "camera_rtc_required"
    max_delta_s: float = float("inf")
    max_vertical_delta_px: float = float("inf")
    allow_app_fallback: bool = True
    max_observation_age_s: float = DEFAULT_DETECTION_MAX_OBSERVATION_AGE_S
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
    reservation: object | None = None
    reservation_error: str = ""


@dataclass(frozen=True)
class _BufferedDetection:
    observation: DetectionObservation
    reservation_source: object | None
    submitted_monotonic_s: float


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
        reservation_factory: Callable[
            [object, Mapping[str, object | None]], object | None
        ]
        | None = None,
    ) -> None:
        self.input_capacity = max(1, int(input_capacity))
        self.buffer_capacity = max(1, int(buffer_capacity))
        self._input: queue.Queue[
            tuple[
                str,
                DetectionObservation | None,
                PairingPolicy,
                object | None,
                float,
            ]
            | None
        ] = queue.Queue(maxsize=self.input_capacity)
        self._results: queue.Queue[PairingResult] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._input_replaced = 0
        self._result_replaced = 0
        self._submitted = 0
        self._paired = 0
        self._rejected = 0
        self._stale_evicted = 0
        self._no_detection_evicted = 0
        self._generation_rollbacks = 0
        self._reservation_failed = 0
        self._sequence = 0
        self._buffers: dict[str, deque[_BufferedDetection]] = {
            "left": deque(maxlen=self.buffer_capacity),
            "right": deque(maxlen=self.buffer_capacity),
        }
        self._generation_by_camera = {"left": 0, "right": 0}
        self._reservation_factory = reservation_factory
        self._latest_policy = PairingPolicy()
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
        *,
        reservation_source: object | None = None,
    ) -> None:
        message = (
            str(camera),
            detection,
            policy,
            reservation_source,
            time.monotonic(),
        )
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

    @staticmethod
    def _release_reservation(reservation: object | None) -> None:
        release = getattr(reservation, "release", None)
        if callable(release):
            release()

    def _evict_stale(
        self,
        policy: PairingPolicy,
        *,
        now_monotonic_s: float,
    ) -> int:
        try:
            max_age_s = float(policy.max_observation_age_s)
        except (TypeError, ValueError):
            max_age_s = DEFAULT_DETECTION_MAX_OBSERVATION_AGE_S
        if not math.isfinite(max_age_s):
            return 0
        max_age_s = max(0.0, max_age_s)
        evicted = 0
        for camera in ("left", "right"):
            current = self._buffers[camera]
            retained = [
                entry
                for entry in current
                if now_monotonic_s - entry.submitted_monotonic_s <= max_age_s
            ]
            evicted += len(current) - len(retained)
            if len(retained) != len(current):
                self._buffers[camera] = deque(
                    retained,
                    maxlen=self.buffer_capacity,
                )
        if evicted:
            with self._lock:
                self._stale_evicted += evicted
        return evicted

    def _result(
        self,
        *,
        policy: PairingPolicy,
        pair: object | None,
        reason: str,
        advisories: list[str] | tuple[str, ...] = (),
        uses_app_fallback: bool = False,
        reservation: object | None = None,
        reservation_error: str = "",
    ) -> PairingResult:
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
            reservation=reservation,
            reservation_error=reservation_error,
        )

    def _match(self, policy: PairingPolicy) -> PairingResult:
        left_entries = list(self._buffers["left"])
        right_entries = list(self._buffers["right"])
        left = [entry.observation for entry in left_entries]
        right = [entry.observation for entry in right_entries]
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

        reservation = None
        reservation_error = ""
        if pair is not None and self._reservation_factory is not None:
            left_entry = next(
                entry for entry in left_entries if entry.observation is pair.left
            )
            right_entry = next(
                entry for entry in right_entries if entry.observation is pair.right
            )
            try:
                reservation = self._reservation_factory(
                    pair,
                    {
                        "left": left_entry.reservation_source,
                        "right": right_entry.reservation_source,
                    },
                )
                if reservation is None:
                    raise RuntimeError("exact native frames are unavailable")
            except Exception as exc:  # noqa: BLE001 - reject the pair fail-closed.
                reservation_error = f"{type(exc).__name__}: {exc}"
                pair = None
                reason = "EVIDENCE_FRAME_UNAVAILABLE"
                advisories.append(reservation_error)
                with self._lock:
                    self._reservation_failed += 1

        return self._result(
            policy=policy,
            pair=pair,
            reason=reason,
            advisories=advisories,
            uses_app_fallback=uses_app_fallback,
            reservation=reservation,
            reservation_error=reservation_error,
        )

    def _publish_result(self, result: PairingResult) -> None:
        try:
            self._results.put_nowait(result)
            return
        except queue.Full:
            pass
        try:
            replaced = self._results.get_nowait()
            with self._lock:
                self._result_replaced += 1
        except queue.Empty:
            pass
        else:
            self._release_reservation(replaced.reservation)
        try:
            self._results.put_nowait(result)
        except queue.Full:
            with self._lock:
                self._result_replaced += 1
            self._release_reservation(result.reservation)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                message = self._input.get(timeout=0.1)
            except queue.Empty:
                if self._evict_stale(
                    self._latest_policy,
                    now_monotonic_s=time.monotonic(),
                ):
                    self._publish_result(self._match(self._latest_policy))
                continue
            if message is None:
                return
            (
                camera,
                detection,
                policy,
                reservation_source,
                submitted_monotonic_s,
            ) = message
            if camera not in self._buffers:
                continue
            self._latest_policy = policy
            self._evict_stale(policy, now_monotonic_s=time.monotonic())
            if detection is None:
                evicted = len(self._buffers[camera])
                self._buffers[camera].clear()
                if evicted:
                    with self._lock:
                        self._no_detection_evicted += evicted
                self._publish_result(self._match(policy))
                continue

            generation = _stream_generation(detection)
            previous_generation = self._generation_by_camera[camera]
            if generation <= 0 and previous_generation > 0:
                with self._lock:
                    self._generation_rollbacks += 1
                self._publish_result(
                    self._result(
                        policy=policy,
                        pair=None,
                        reason="stream_generation_rollback",
                    )
                )
                continue
            if generation > 0 and previous_generation > 0 and generation < previous_generation:
                with self._lock:
                    self._generation_rollbacks += 1
                self._publish_result(
                    self._result(
                        policy=policy,
                        pair=None,
                        reason="stream_generation_rollback",
                    )
                )
                continue
            if generation > previous_generation:
                self._buffers[camera].clear()
                self._generation_by_camera[camera] = generation
            if generation <= 0 or generation == self._generation_by_camera[camera]:
                self._buffers[camera].append(
                    _BufferedDetection(
                        observation=detection,
                        reservation_source=reservation_source,
                        submitted_monotonic_s=float(submitted_monotonic_s),
                    )
                )
            self._publish_result(self._match(policy))

    def poll_latest(self) -> PairingResult | None:
        latest = None
        while True:
            try:
                candidate = self._results.get_nowait()
            except queue.Empty:
                return latest
            if latest is not None:
                self._release_reservation(latest.reservation)
            latest = candidate

    def metrics(self) -> dict[str, int]:
        with self._lock:
            return {
                "input_capacity": self.input_capacity,
                "buffer_capacity": self.buffer_capacity,
                "submitted": self._submitted,
                "paired": self._paired,
                "rejected": self._rejected,
                "stale_evicted": self._stale_evicted,
                "no_detection_evicted": self._no_detection_evicted,
                "generation_rollbacks": self._generation_rollbacks,
                "reservation_failed": self._reservation_failed,
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
        while True:
            try:
                result = self._results.get_nowait()
            except queue.Empty:
                break
            self._release_reservation(result.reservation)
        return not self._thread.is_alive()


__all__ = [
    "DEFAULT_DETECTION_BUFFER_CAPACITY",
    "DEFAULT_DETECTION_INPUT_CAPACITY",
    "DEFAULT_DETECTION_MAX_OBSERVATION_AGE_S",
    "DetectionPairCoordinator",
    "PairingPolicy",
    "PairingResult",
]
