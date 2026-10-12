"""Per-link PX4 boot epoch tracking with explicit host receive timestamps.

Inputs remain uint32 boot milliseconds and host monotonic seconds. This owner
has no clock, timer, socket or Qt dependency; observe() only advances its own
reorder/reboot evidence. Subscriber policy decides what to do with a reboot.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


MAVLINK_BOOT_TIME_UINT32_MODULUS = 1 << 32


MAVLINK_BOOT_TIME_UINT32_HALF_RANGE = 1 << 31


MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS = 2_000


MAVLINK_BOOT_TIME_REBOOT_REGRESSION_MS = MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS


MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_SAMPLES = 2


MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_WINDOW_S = 2.0


MAVLINK_BOOT_TIME_FORWARD_SLACK_MS = 5_000


@dataclass(frozen=True)
class _MavlinkBootTimeSample:
    time_boot_ms: int
    received_at_s: float


@dataclass
class _MavlinkBootTimeRegressionCandidate:
    latest: _MavlinkBootTimeSample
    sample_count: int = 1


@dataclass(frozen=True)
class _MavlinkRemoteRebootEvidence:
    previous_time_boot_ms: int
    current_time_boot_ms: int


@dataclass(frozen=True)
class _MavlinkBootTimeObservation:
    accept_for_downstream: bool = True
    reboot: _MavlinkRemoteRebootEvidence | None = None


class _MavlinkBootTimeTracker:
    """Confirm remote PX4 boot epochs from one side's ATTITUDE clock.

    MAVLink ``time_boot_ms`` is uint32.  Forward modular movement includes the
    natural roughly 49.7-day wrap.  Small backward movement is packet
    reordering, while movement beyond the two-second reorder tolerance requires
    two increasing low-epoch samples to confirm a reboot.  The host receive
    clock also rejects a delayed prior-boot high-time packet after confirmation.
    """

    def __init__(self) -> None:
        self._latest: _MavlinkBootTimeSample | None = None
        self._candidate: _MavlinkBootTimeRegressionCandidate | None = None
        self._confirmed_reboots = 0

    def reset(self) -> None:
        self._latest = None
        self._candidate = None
        self._confirmed_reboots = 0

    @staticmethod
    def _forward_delta(previous_ms: int, current_ms: int) -> int:
        return (int(current_ms) - int(previous_ms)) % MAVLINK_BOOT_TIME_UINT32_MODULUS

    @staticmethod
    def _plausible_forward(
        previous: _MavlinkBootTimeSample,
        current: _MavlinkBootTimeSample,
    ) -> bool:
        delta_ms = _MavlinkBootTimeTracker._forward_delta(
            previous.time_boot_ms,
            current.time_boot_ms,
        )
        if not 0 < delta_ms < MAVLINK_BOOT_TIME_UINT32_HALF_RANGE:
            return False
        host_elapsed_ms = max(
            0,
            int(round((current.received_at_s - previous.received_at_s) * 1_000.0)),
        )
        return delta_ms <= host_elapsed_ms + MAVLINK_BOOT_TIME_FORWARD_SLACK_MS

    def observe(
        self,
        time_boot_ms: Any,
        *,
        received_at_s: float,
    ) -> _MavlinkBootTimeObservation:
        if isinstance(time_boot_ms, bool):
            return _MavlinkBootTimeObservation()
        try:
            boot_ms = int(time_boot_ms)
            observed_s = float(received_at_s)
        except (TypeError, ValueError, OverflowError):
            return _MavlinkBootTimeObservation()
        if (
            not 0 <= boot_ms < MAVLINK_BOOT_TIME_UINT32_MODULUS
            or not math.isfinite(observed_s)
        ):
            return _MavlinkBootTimeObservation()

        sample = _MavlinkBootTimeSample(boot_ms, observed_s)
        previous = self._latest
        if previous is None:
            self._latest = sample
            return _MavlinkBootTimeObservation()

        candidate = self._candidate
        if candidate is not None:
            candidate_age_s = sample.received_at_s - candidate.latest.received_at_s
            if not 0.0 <= candidate_age_s <= MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_WINDOW_S:
                self._candidate = None
            else:
                baseline_forward = self._forward_delta(
                    previous.time_boot_ms,
                    sample.time_boot_ms,
                )
                raw_regression_ms = max(
                    0,
                    previous.time_boot_ms - sample.time_boot_ms,
                )
                if baseline_forward == 0 or self._plausible_forward(previous, sample):
                    self._candidate = None
                    if baseline_forward != 0:
                        self._latest = sample
                    return _MavlinkBootTimeObservation()
                if raw_regression_ms <= MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS:
                    self._candidate = None
                    return _MavlinkBootTimeObservation()
                if (
                    self._plausible_forward(candidate.latest, sample)
                    and raw_regression_ms >= MAVLINK_BOOT_TIME_REBOOT_REGRESSION_MS
                ):
                    candidate.latest = sample
                    candidate.sample_count += 1
                    if (
                        candidate.sample_count
                        >= MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_SAMPLES
                    ):
                        evidence = _MavlinkRemoteRebootEvidence(
                            previous_time_boot_ms=previous.time_boot_ms,
                            current_time_boot_ms=sample.time_boot_ms,
                        )
                        self._latest = sample
                        self._candidate = None
                        self._confirmed_reboots += 1
                        return _MavlinkBootTimeObservation(reboot=evidence)
                    return _MavlinkBootTimeObservation()
                self._candidate = None

        forward_delta = self._forward_delta(
            previous.time_boot_ms,
            sample.time_boot_ms,
        )
        if forward_delta == 0:
            return _MavlinkBootTimeObservation()
        if self._plausible_forward(previous, sample):
            self._latest = sample
            return _MavlinkBootTimeObservation()

        raw_regression_ms = max(
            0,
            previous.time_boot_ms - sample.time_boot_ms,
        )
        if 0 < raw_regression_ms <= MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS:
            return _MavlinkBootTimeObservation()
        if raw_regression_ms >= MAVLINK_BOOT_TIME_REBOOT_REGRESSION_MS:
            self._candidate = _MavlinkBootTimeRegressionCandidate(
                latest=sample,
            )
            return _MavlinkBootTimeObservation()

        # Once a lower boot epoch is confirmed, a large forward leap that
        # cannot fit the host elapsed time is a delayed prior-epoch packet.
        # Before any confirmed reboot, retain compatibility and merely exclude
        # the implausible value from the tracker baseline.
        return _MavlinkBootTimeObservation(
            accept_for_downstream=self._confirmed_reboots == 0
        )


# Retain the historical pickle path for existing tracker snapshots, if any.
for _legacy_type in (
    _MavlinkBootTimeSample,
    _MavlinkBootTimeRegressionCandidate,
    _MavlinkRemoteRebootEvidence,
    _MavlinkBootTimeObservation,
    _MavlinkBootTimeTracker,
):
    _legacy_type.__module__ = "Workstation.workstation_core.pixhawk_attitude"
del _legacy_type
