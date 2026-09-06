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
    TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC,
    correspondence_advisories,
    correspondence_rejection_reason,
)
from Workstation.workstation_core.sync import match_nearest_pair


DEFAULT_DETECTION_INPUT_CAPACITY = 64
DEFAULT_DETECTION_BUFFER_CAPACITY = 64
DEFAULT_DETECTION_MAX_OBSERVATION_AGE_S = 1.0
PAIR_FLOAT_COMPARISON_TOLERANCE_S = 1.0e-9
LIVE_DISTANCE_HARD_MAX_DELTA_NS = 100_000_000


@dataclass(frozen=True)
class PairingPolicy:
    timestamp_basis: str = TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC
    max_delta_s: float = 0.100
    max_vertical_delta_px: float = float("inf")
    allow_app_fallback: bool = False
    rtcp_clock_verified: bool | None = None
    rtcp_unavailable_reason: str = ""
    require_current_clock_epoch: bool = False
    measurement_generation: str = ""
    max_observation_age_s: float = DEFAULT_DETECTION_MAX_OBSERVATION_AGE_S
    revision: int = 0


@dataclass(frozen=True)
class PairTimingDecision:
    """The immutable timing choice consumed by freeze, workers, UI and storage."""

    requested_basis: str
    effective_basis: str
    left_timestamp_s: float
    right_timestamp_s: float
    signed_delta_ns: int
    abs_delta_ns: int
    fallback_reason: str = ""

    def __post_init__(self) -> None:
        left = float(self.left_timestamp_s)
        right = float(self.right_timestamp_s)
        signed = int(self.signed_delta_ns)
        absolute = int(self.abs_delta_ns)
        if not math.isfinite(left) or not math.isfinite(right):
            raise ValueError("pair timing timestamps must be finite")
        if absolute != abs(signed):
            raise ValueError("pair timing absolute delta must match signed delta")
        object.__setattr__(self, "requested_basis", str(self.requested_basis))
        object.__setattr__(self, "effective_basis", str(self.effective_basis))
        object.__setattr__(self, "left_timestamp_s", left)
        object.__setattr__(self, "right_timestamp_s", right)
        object.__setattr__(self, "signed_delta_ns", signed)
        object.__setattr__(self, "abs_delta_ns", absolute)
        object.__setattr__(self, "fallback_reason", str(self.fallback_reason or ""))


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
    timing: PairTimingDecision | None = None
    primary_rejection_reason: str = ""
    fallback_rejection_reason: str = ""


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


def _app_timestamp_ns(observation: DetectionObservation) -> int | None:
    try:
        value = observation.app_receive_monotonic_ns
    except (TypeError, ValueError, OverflowError):
        return None
    return value if type(value) is int and value >= 0 else None


def _app_timestamp(observation: DetectionObservation) -> float | None:
    value = _app_timestamp_ns(observation)
    return None if value is None else value * 1.0e-9


def _stream_generation(observation: DetectionObservation) -> int:
    try:
        return int(observation.timestamp_metadata.get("stream_generation") or 0)
    except (TypeError, ValueError):
        return 0


def _clock_epoch_state(observation: DetectionObservation) -> tuple[str, str]:
    metadata = observation.timestamp_metadata
    return (
        str(metadata.get("clock_epoch_state") or ""),
        str(metadata.get("clock_epoch_id") or ""),
    )


def _has_clock_epoch_evidence(observation: DetectionObservation) -> bool:
    # "unverified" is the default stamp on every frame once STUS epoch
    # tracking is decorating metadata at all, so treating it as evidence
    # would force epoch gating unconditionally and silently override an
    # explicit require_current_clock_epoch=False. Only a real signal that
    # an epoch is actually in flux or established should auto-enable it.
    state, epoch_id = _clock_epoch_state(observation)
    return bool(epoch_id) or state not in ("", "unverified")


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
        self._input_lost = 0
        self._result_replaced = 0
        self._submitted = 0
        self._paired = 0
        self._rejected = 0
        self._stale_evicted = 0
        self._no_detection_evicted = 0
        self._generation_rollbacks = 0
        self._policy_revisions = 0
        self._stale_policy_inputs = 0
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
        # The full-queue fallback (evict oldest, then insert) must be atomic
        # as a unit. Without the lock, two concurrent producers (left/right
        # camera threads) can each observe a full queue, each successfully
        # evict a slot, and then race for the freed slots: the loser's final
        # put_nowait() fails even though it already evicted something, so its
        # own submission is dropped entirely -- neither queued nor recorded
        # as displacing anything. Serializing the whole sequence under
        # ``self._lock`` removes that race, and the two outcomes below are
        # now tracked as genuinely distinct: ``_input_replaced`` only counts
        # a submission that made it in after displacing an older one, while
        # ``_input_lost`` counts a submission that ended up neither queued
        # nor recorded as a replacement (true data loss).
        with self._lock:
            evicted = False
            enqueued = False
            try:
                self._input.put_nowait(message)
                enqueued = True
            except queue.Full:
                try:
                    self._input.get_nowait()
                    evicted = True
                except queue.Empty:
                    pass
                try:
                    self._input.put_nowait(message)
                    enqueued = True
                except queue.Full:
                    enqueued = False
            if enqueued and evicted:
                self._input_replaced += 1
            elif not enqueued:
                self._input_lost += 1
            self._submitted += 1

    def _timestamp_for_policy(
        self,
        observation: DetectionObservation,
        policy: PairingPolicy,
    ) -> float | None:
        basis = str(policy.timestamp_basis or "detection")
        if basis in {"camera_rtc_required", "camera_rtcp"}:
            return _camera_timestamp(observation)
        if basis == TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC:
            return _app_timestamp(observation)
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
        timing: PairTimingDecision | None = None,
        primary_rejection_reason: str = "",
        fallback_rejection_reason: str = "",
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
            timing=timing,
            primary_rejection_reason=str(primary_rejection_reason or ""),
            fallback_rejection_reason=str(fallback_rejection_reason or ""),
        )

    @staticmethod
    def _delta_limit_ns(policy: PairingPolicy) -> int:
        try:
            configured_delta_s = float(policy.max_delta_s)
        except (TypeError, ValueError):
            configured_delta_s = 0.100
        if not math.isfinite(configured_delta_s):
            return LIVE_DISTANCE_HARD_MAX_DELTA_NS
        return min(
            LIVE_DISTANCE_HARD_MAX_DELTA_NS,
            max(0, int(round(configured_delta_s * 1_000_000_000.0))),
        )

    @staticmethod
    def _timing_decision(
        pair: object,
        *,
        requested_basis: str,
        effective_basis: str,
        fallback_reason: str = "",
    ) -> PairTimingDecision:
        if effective_basis == TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC:
            left_ns = _app_timestamp_ns(pair.left)
            right_ns = _app_timestamp_ns(pair.right)
            if left_ns is None or right_ns is None:
                raise ValueError("selected app pair has no exact monotonic timestamp")
            signed_delta_ns = right_ns - left_ns
            left_timestamp_s = left_ns * 1.0e-9
            right_timestamp_s = right_ns * 1.0e-9
        else:
            left_timestamp_s = _camera_timestamp(pair.left)
            right_timestamp_s = _camera_timestamp(pair.right)
            if left_timestamp_s is None or right_timestamp_s is None:
                raise ValueError("selected RTCP pair has no finite timestamp")
            signed_delta_ns = int(
                round((right_timestamp_s - left_timestamp_s) * 1_000_000_000.0)
            )
        return PairTimingDecision(
            requested_basis=requested_basis,
            effective_basis=effective_basis,
            left_timestamp_s=left_timestamp_s,
            right_timestamp_s=right_timestamp_s,
            signed_delta_ns=signed_delta_ns,
            abs_delta_ns=abs(signed_delta_ns),
            fallback_reason=fallback_reason,
        )

    def _match_app_receive(
        self,
        left: list[DetectionObservation],
        right: list[DetectionObservation],
        policy: PairingPolicy,
    ) -> tuple[object | None, str]:
        left_valid = [item for item in left if _app_timestamp_ns(item) is not None]
        right_valid = [item for item in right if _app_timestamp_ns(item) is not None]
        if not left_valid or not right_valid:
            return None, "APP_RECEIVE_TIMESTAMP_INVALID"
        limit_ns = self._delta_limit_ns(policy)
        pair = match_nearest_pair(
            left_valid,
            right_valid,
            max_delta_s=(
                limit_ns * 1.0e-9 + PAIR_FLOAT_COMPARISON_TOLERANCE_S
            ),
            timestamp=_app_timestamp,
        )
        if pair is None:
            # The window only prefers close partners. Without one, the nearest
            # available partner is still paired and the excess is reported.
            pair = match_nearest_pair(
                left_valid,
                right_valid,
                max_delta_s=float("inf"),
                timestamp=_app_timestamp,
            )
            if pair is None:
                return None, "APP_RECEIVE_TIMESTAMP_INVALID"
        left_ns = _app_timestamp_ns(pair.left)
        right_ns = _app_timestamp_ns(pair.right)
        if left_ns is None or right_ns is None:
            return None, "APP_RECEIVE_TIMESTAMP_INVALID"
        return pair, ""

    def _match_rtcp(
        self,
        left: list[DetectionObservation],
        right: list[DetectionObservation],
        policy: PairingPolicy,
    ) -> tuple[object | None, str]:
        if policy.rtcp_clock_verified is False:
            return None, str(policy.rtcp_unavailable_reason or "CLOCK_NOT_VERIFIED")

        if _camera_timestamp(left[-1]) is None or _camera_timestamp(right[-1]) is None:
            return None, "RTCP_TIMESTAMP_UNAVAILABLE"
        left_valid = [item for item in left if _camera_timestamp(item) is not None]
        right_valid = [item for item in right if _camera_timestamp(item) is not None]
        if not left_valid or not right_valid:
            return None, "RTCP_TIMESTAMP_UNAVAILABLE"

        require_epoch = bool(policy.require_current_clock_epoch) or any(
            _has_clock_epoch_evidence(item) for item in (*left_valid, *right_valid)
        )
        epoch_groups: list[tuple[list[DetectionObservation], list[DetectionObservation]]] = []
        if require_epoch:
            latest_left_state, latest_left_epoch = _clock_epoch_state(left[-1])
            latest_right_state, latest_right_epoch = _clock_epoch_state(right[-1])
            if {latest_left_state, latest_right_state} != {"current"}:
                return None, "CLOCK_EPOCH_TRANSITION"
            if (
                not latest_left_epoch
                or not latest_right_epoch
                or latest_left_epoch != latest_right_epoch
            ):
                return None, "CLOCK_EPOCH_MISMATCH"
            left_current: dict[str, list[DetectionObservation]] = {}
            right_current: dict[str, list[DetectionObservation]] = {}
            for target, observations in (
                (left_current, left_valid),
                (right_current, right_valid),
            ):
                for observation in observations:
                    state, epoch_id = _clock_epoch_state(observation)
                    if state == "current" and epoch_id:
                        target.setdefault(epoch_id, []).append(observation)
            common_epochs = set(left_current) & set(right_current)
            if not common_epochs:
                if left_current and right_current:
                    return None, "CLOCK_EPOCH_MISMATCH"
                return None, "CLOCK_EPOCH_TRANSITION"
            epoch_groups = [
                (left_current[epoch], right_current[epoch])
                for epoch in sorted(common_epochs)
            ]
        else:
            epoch_groups = [(left_valid, right_valid)]

        limit_ns = self._delta_limit_ns(policy)
        limit_s = limit_ns * 1.0e-9
        candidates = [
            pair
            for left_group, right_group in epoch_groups
            if (
                pair := match_nearest_pair(
                    left_group,
                    right_group,
                    max_delta_s=(
                        limit_s + PAIR_FLOAT_COMPARISON_TOLERANCE_S
                    ),
                    timestamp=_camera_timestamp,
                )
            )
            is not None
        ]
        if not candidates:
            # No partner inside the preferred window: still pair the nearest
            # RTCP-timestamped partner and let the excess become an advisory.
            candidates = [
                pair
                for left_group, right_group in epoch_groups
                if (
                    pair := match_nearest_pair(
                        left_group,
                        right_group,
                        max_delta_s=float("inf"),
                        timestamp=_camera_timestamp,
                    )
                )
                is not None
            ]
        if not candidates:
            return None, "RTCP_TIMESTAMP_UNAVAILABLE"
        return min(candidates, key=lambda item: (item.delta_s, -item.midpoint_s)), ""

    def _match(self, policy: PairingPolicy) -> PairingResult:
        left_entries = list(self._buffers["left"])
        right_entries = list(self._buffers["right"])
        left = [entry.observation for entry in left_entries]
        right = [entry.observation for entry in right_entries]
        requested_basis = str(policy.timestamp_basis or "detection")
        primary_rejection_reason = ""
        fallback_rejection_reason = ""
        timing: PairTimingDecision | None = None
        advisories: list[str] = []

        if not left or not right:
            return self._result(
                policy=policy,
                pair=None,
                reason=(
                    "missing_left_detection"
                    if not left
                    else "missing_right_detection"
                ),
            )

        if requested_basis == TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC:
            pair, reason = self._match_app_receive(left, right, policy)
            if pair is not None:
                timing = self._timing_decision(
                    pair,
                    requested_basis=requested_basis,
                    effective_basis=TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC,
                )
        elif requested_basis in {"camera_rtc_required", "camera_rtcp"}:
            pair, primary_rejection_reason = self._match_rtcp(left, right, policy)
            reason = primary_rejection_reason
            if pair is not None:
                timing = self._timing_decision(
                    pair,
                    requested_basis=requested_basis,
                    effective_basis="camera_rtcp",
                )
            elif requested_basis == "camera_rtcp" or policy.allow_app_fallback:
                pair, fallback_rejection_reason = self._match_app_receive(
                    left,
                    right,
                    policy,
                )
                reason = fallback_rejection_reason
                if pair is not None:
                    timing = self._timing_decision(
                        pair,
                        requested_basis=requested_basis,
                        effective_basis=TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC,
                        fallback_reason=primary_rejection_reason,
                    )
                    reason = ""
        else:
            try:
                configured_delta_s = float(policy.max_delta_s)
            except (TypeError, ValueError):
                configured_delta_s = float("inf")
            pair = match_nearest_pair(
                left,
                right,
                max_delta_s=float("inf"),
                timestamp=lambda observation: float(observation.timestamp_s),
            )
            reason = "" if pair is not None else "no_timestamp_pair"
            if pair is not None:
                signed_delta_ns = int(
                    round(
                        (float(pair.right.timestamp_s) - float(pair.left.timestamp_s))
                        * 1_000_000_000.0
                    )
                )
                timing = PairTimingDecision(
                    requested_basis=requested_basis,
                    effective_basis=requested_basis,
                    left_timestamp_s=float(pair.left.timestamp_s),
                    right_timestamp_s=float(pair.right.timestamp_s),
                    signed_delta_ns=signed_delta_ns,
                    abs_delta_ns=abs(signed_delta_ns),
                )
                if math.isfinite(configured_delta_s) and pair.delta_s > configured_delta_s:
                    advisories.append(
                        f"camera pair delta {pair.delta_s:.6f}s exceeds configured limit"
                    )

        if pair is not None:
            correspondence_reason = correspondence_rejection_reason(pair.left, pair.right)
            if correspondence_reason:
                reason = correspondence_reason
                pair = None
                timing = None
        if pair is not None:
            advisories.extend(correspondence_advisories(pair.left, pair.right))
            if timing is not None and timing.abs_delta_ns > self._delta_limit_ns(policy):
                advisories.append(
                    f"pair_delta_exceeds_window: {timing.abs_delta_ns}ns > "
                    f"{self._delta_limit_ns(policy)}ns"
                )
            vertical_delta = abs(pair.left.center[1] - pair.right.center[1])
            if vertical_delta > float(policy.max_vertical_delta_px):
                advisories.append(
                    f"vertical bbox delta {vertical_delta:.3f}px exceeds configured limit"
                )

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
                timing = None
                reason = "EVIDENCE_FRAME_UNAVAILABLE"
                advisories.append(reservation_error)
                with self._lock:
                    self._reservation_failed += 1

        return self._result(
            policy=policy,
            pair=pair,
            reason=reason,
            advisories=advisories,
            uses_app_fallback=bool(
                pair is not None
                and timing is not None
                and timing.effective_basis == TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC
                and timing.requested_basis != TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC
            ),
            reservation=reservation,
            reservation_error=reservation_error,
            timing=timing,
            primary_rejection_reason=primary_rejection_reason,
            fallback_rejection_reason=fallback_rejection_reason,
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

    def _discard_pending_results(self) -> None:
        while True:
            try:
                result = self._results.get_nowait()
            except queue.Empty:
                return
            self._release_reservation(result.reservation)

    def _accept_policy_revision(self, policy: PairingPolicy) -> bool:
        incoming = int(policy.revision)
        current = int(self._latest_policy.revision)
        if incoming < current:
            with self._lock:
                self._stale_policy_inputs += 1
            return False
        if incoming > current:
            for buffer in self._buffers.values():
                buffer.clear()
            self._discard_pending_results()
            with self._lock:
                self._policy_revisions += 1
        self._latest_policy = policy
        return True

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
            if not self._accept_policy_revision(policy):
                continue
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
                "policy_revisions": self._policy_revisions,
                "stale_policy_inputs": self._stale_policy_inputs,
                "reservation_failed": self._reservation_failed,
                "input_replaced": self._input_replaced,
                "input_lost": self._input_lost,
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
    "PairTimingDecision",
]
