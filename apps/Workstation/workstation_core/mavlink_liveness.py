"""Deterministic per-side MAVLink liveness and qualification state.

This module deliberately has no Qt or pymavlink dependency.  The transport
owner feeds it validated packet provenance and monotonic receive times; UI,
RTCM, and command owners consume immutable snapshots.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable


MAVLINK_ATTITUDE_FRESH_S = 2.0
MAVLINK_HEARTBEAT_FRESH_S = 3.0
MAVLINK_ALL_RX_STALL_S = 30.0
MAVLINK_INITIAL_RX_TIMEOUT_S = 30.0
MAVLINK_QUALIFICATION_S = 10.0


class MavlinkLivenessState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    QUALIFYING = "QUALIFYING"
    LIVE = "LIVE"
    ATTITUDE_STALE = "ATTITUDE STALE"
    INITIAL_RX_TIMEOUT = "MAVLINK INITIAL RX TIMEOUT"
    RX_STALLED = "ETH MAVLINK RX STALLED"
    RECOVERING = "RECOVERING"
    ERROR = "ERROR"


@dataclass(frozen=True)
class MavlinkSideHealthSnapshot:
    side: str
    connection_generation: int
    endpoint: str
    peer_host: str
    expected_system_id: int
    observed_system_id: int | None
    observed_component_id: int | None
    identity_verified: bool
    listener_open: bool
    listener_opened_monotonic_s: float | None
    listener_age_s: float | None
    last_any_rx_monotonic_s: float | None
    last_heartbeat_rx_monotonic_s: float | None
    last_attitude_rx_monotonic_s: float | None
    last_gcs_heartbeat_tx_monotonic_s: float | None
    any_rx_age_s: float | None
    heartbeat_rx_age_s: float | None
    attitude_rx_age_s: float | None
    active_command: str | None
    state: MavlinkLivenessState
    qualification_started_monotonic_s: float | None
    qualification_elapsed_s: float
    qualification_required_s: float
    rtcm_eligible: bool
    command_eligible: bool
    stall_reason: str | None


class MavlinkSideLiveness:
    """Thread-safe event reducer for one expected MAVLink system ID."""

    def __init__(
        self,
        *,
        side: str,
        expected_system_id: int,
        endpoint: str = "",
        peer_host: str = "",
        clock: Callable[[], float] = time.monotonic,
        attitude_fresh_s: float = MAVLINK_ATTITUDE_FRESH_S,
        heartbeat_fresh_s: float = MAVLINK_HEARTBEAT_FRESH_S,
        all_rx_stall_s: float = MAVLINK_ALL_RX_STALL_S,
        initial_rx_timeout_s: float = MAVLINK_INITIAL_RX_TIMEOUT_S,
        qualification_s: float = MAVLINK_QUALIFICATION_S,
    ) -> None:
        expected_system_id = int(expected_system_id)
        if not 1 <= expected_system_id <= 255:
            raise ValueError("expected_system_id must be in MAVLink range 1..255")
        thresholds = (
            float(attitude_fresh_s),
            float(heartbeat_fresh_s),
            float(all_rx_stall_s),
            float(initial_rx_timeout_s),
            float(qualification_s),
        )
        if any(value <= 0.0 for value in thresholds):
            raise ValueError("liveness thresholds must be positive")
        self.side = str(side)
        self.expected_system_id = expected_system_id
        self.endpoint = str(endpoint)
        self.peer_host = str(peer_host).strip()
        self.attitude_fresh_s = thresholds[0]
        self.heartbeat_fresh_s = thresholds[1]
        self.all_rx_stall_s = thresholds[2]
        self.initial_rx_timeout_s = thresholds[3]
        self.qualification_s = thresholds[4]
        self._clock = clock
        self._lock = threading.RLock()
        self._connection_generation = 0
        self._listener_open = False
        self._listener_opened_s: float | None = None
        self._identity_wait_started_s: float | None = None
        self._observed_system_id: int | None = None
        self._observed_component_id: int | None = None
        self._identity_verified = False
        self._last_any_rx_s: float | None = None
        self._last_heartbeat_rx_s: float | None = None
        self._last_attitude_rx_s: float | None = None
        self._last_gcs_heartbeat_tx_s: float | None = None
        self._qualification_started_s: float | None = None
        self._active_command: str | None = None
        self._preserve_qualification_command = False
        self._command_ended_s: float | None = None
        self._stalled = False
        self._recovering = False
        self._error: str | None = None

    @property
    def connection_generation(self) -> int:
        with self._lock:
            return self._connection_generation

    def _time(self, value: float | None) -> float:
        return float(self._clock() if value is None else value)

    def open_listener(
        self,
        *,
        endpoint: str | None = None,
        peer_host: str | None = None,
        now_s: float | None = None,
        recovering: bool = False,
    ) -> int:
        """Start a new connection generation and discard all old evidence."""

        opened_s = self._time(now_s)
        with self._lock:
            self._connection_generation += 1
            if endpoint is not None:
                self.endpoint = str(endpoint)
            if peer_host is not None:
                self.peer_host = str(peer_host).strip()
            self._listener_open = True
            self._listener_opened_s = opened_s
            self._identity_wait_started_s = opened_s
            self._observed_system_id = None
            self._observed_component_id = None
            self._identity_verified = False
            self._last_any_rx_s = None
            self._last_heartbeat_rx_s = None
            self._last_attitude_rx_s = None
            self._last_gcs_heartbeat_tx_s = None
            self._qualification_started_s = None
            self._active_command = None
            self._preserve_qualification_command = False
            self._command_ended_s = None
            self._stalled = False
            self._recovering = bool(recovering)
            self._error = None
            return self._connection_generation

    def close_listener(self, *, now_s: float | None = None) -> None:
        self._time(now_s)
        with self._lock:
            self._listener_open = False
            self._listener_opened_s = None
            self._identity_wait_started_s = None
            self._active_command = None
            self._preserve_qualification_command = False
            self._qualification_started_s = None
            self._stalled = False
            self._recovering = False

    def mark_error(self, error: str, *, now_s: float | None = None) -> None:
        self._time(now_s)
        with self._lock:
            self._error = str(error)
            self._qualification_started_s = None

    def clear_error(self) -> None:
        with self._lock:
            self._error = None

    def note_remote_reboot(self, *, now_s: float | None = None) -> bool:
        """Invalidate old-boot evidence without replacing the host listener.

        A PX4 power-cycle can restart ``time_boot_ms`` while pymavlink keeps the
        same UDP socket object.  That is a new remote boot, not a new host
        connection generation.  Require a fresh HEARTBEAT identity and a new
        qualification interval while preserving the listener generation used
        by in-flight host routing callbacks.
        """

        reboot_s = self._time(now_s)
        with self._lock:
            if not self._listener_open:
                return False
            self._observed_system_id = None
            self._observed_component_id = None
            self._identity_verified = False
            self._identity_wait_started_s = reboot_s
            self._last_any_rx_s = None
            self._last_heartbeat_rx_s = None
            self._last_attitude_rx_s = None
            self._qualification_started_s = None
            self._stalled = False
            self._recovering = True
            return True

    def begin_command(
        self,
        command: str,
        *,
        now_s: float | None = None,
        preserve_qualification: bool = False,
    ) -> None:
        self._time(now_s)
        with self._lock:
            if self._active_command is not None:
                raise RuntimeError(f"MAVLink command already active: {self._active_command}")
            self._active_command = str(command)
            self._preserve_qualification_command = bool(preserve_qualification)
            if not self._preserve_qualification_command:
                self._qualification_started_s = None

    def end_command(self, *, now_s: float | None = None) -> None:
        ended_s = self._time(now_s)
        with self._lock:
            if self._active_command is None:
                return
            preserve_qualification = self._preserve_qualification_command
            self._active_command = None
            self._preserve_qualification_command = False
            if not preserve_qualification:
                self._command_ended_s = ended_s
                self._qualification_started_s = None

    def observe_packet(
        self,
        *,
        system_id: int | None,
        component_id: int | None,
        message_type: str,
        received_at_s: float | None = None,
        generation: int | None = None,
    ) -> bool:
        """Accept one parseable expected-system packet as transport evidence.

        Returns ``False`` for a closed/replaced generation, BAD_DATA, local
        echo, or any unexpected system ID.  Such packets update no clock.
        """

        received_s = self._time(received_at_s)
        message_name = str(message_type).strip().upper()
        with self._lock:
            if not self._listener_open:
                return False
            if generation is not None and int(generation) != self._connection_generation:
                return False
            if message_name in {"", "BAD_DATA"}:
                return False
            try:
                parsed_system_id = int(system_id) if system_id is not None else None
            except (TypeError, ValueError):
                return False
            if parsed_system_id != self.expected_system_id:
                return False

            self._mark_stalled_if_due(received_s)
            if self._stalled:
                self._stalled = False
                self._recovering = True
                self._qualification_started_s = None

            self._observed_system_id = parsed_system_id
            try:
                self._observed_component_id = int(component_id) if component_id is not None else None
            except (TypeError, ValueError):
                self._observed_component_id = None
            self._last_any_rx_s = self._latest(self._last_any_rx_s, received_s)
            if message_name == "HEARTBEAT":
                self._identity_verified = True
                self._identity_wait_started_s = None
                self._last_heartbeat_rx_s = self._latest(self._last_heartbeat_rx_s, received_s)
            elif message_name == "ATTITUDE":
                self._last_attitude_rx_s = self._latest(self._last_attitude_rx_s, received_s)
            self._refresh_qualification(received_s)
            return True

    def note_gcs_heartbeat_sent(
        self,
        *,
        sent_at_s: float | None = None,
        generation: int | None = None,
    ) -> bool:
        sent_s = self._time(sent_at_s)
        with self._lock:
            if not self._listener_open:
                return False
            if generation is not None and int(generation) != self._connection_generation:
                return False
            self._last_gcs_heartbeat_tx_s = self._latest(self._last_gcs_heartbeat_tx_s, sent_s)
            return True

    @staticmethod
    def _latest(previous: float | None, candidate: float) -> float:
        return candidate if previous is None else max(previous, candidate)

    @staticmethod
    def _age(now_s: float, observed_s: float | None) -> float | None:
        return None if observed_s is None else max(0.0, now_s - observed_s)

    def _post_command_streams_observed(self) -> bool:
        if self._command_ended_s is None:
            return True
        return (
            self._last_heartbeat_rx_s is not None
            and self._last_heartbeat_rx_s >= self._command_ended_s
            and self._last_attitude_rx_s is not None
            and self._last_attitude_rx_s >= self._command_ended_s
        )

    def _streams_fresh(self, now_s: float) -> tuple[bool, bool]:
        heartbeat_age = self._age(now_s, self._last_heartbeat_rx_s)
        attitude_age = self._age(now_s, self._last_attitude_rx_s)
        post_command = self._post_command_streams_observed()
        return (
            post_command and heartbeat_age is not None and heartbeat_age <= self.heartbeat_fresh_s,
            post_command and attitude_age is not None and attitude_age <= self.attitude_fresh_s,
        )

    def _effective_last_any_s(self) -> float | None:
        if self._command_ended_s is None:
            return self._last_any_rx_s
        if self._last_any_rx_s is None:
            return self._command_ended_s
        return max(self._last_any_rx_s, self._command_ended_s)

    def _mark_stalled_if_due(self, now_s: float) -> None:
        if (
            not self._listener_open
            or not self._identity_verified
            or self._active_command is not None
            or self._error is not None
        ):
            return
        any_age = self._age(now_s, self._effective_last_any_s())
        if any_age is not None and any_age >= self.all_rx_stall_s:
            self._stalled = True
            self._qualification_started_s = None

    def _refresh_qualification(self, now_s: float) -> None:
        if (
            not self._listener_open
            or not self._identity_verified
            or self._active_command is not None
            or self._stalled
            or self._error is not None
        ):
            return
        heartbeat_fresh, attitude_fresh = self._streams_fresh(now_s)
        if not (heartbeat_fresh and attitude_fresh):
            self._qualification_started_s = None
        elif self._qualification_started_s is None:
            self._qualification_started_s = now_s

    def snapshot(self, *, now_s: float | None = None) -> MavlinkSideHealthSnapshot:
        observed_s = self._time(now_s)
        with self._lock:
            self._mark_stalled_if_due(observed_s)
            self._refresh_qualification(observed_s)
            any_age = self._age(observed_s, self._last_any_rx_s)
            listener_age = self._age(observed_s, self._listener_opened_s)
            identity_wait_age = self._age(
                observed_s,
                self._identity_wait_started_s,
            )
            heartbeat_age = self._age(observed_s, self._last_heartbeat_rx_s)
            attitude_age = self._age(observed_s, self._last_attitude_rx_s)
            heartbeat_fresh, attitude_fresh = self._streams_fresh(observed_s)
            qualification_elapsed = (
                0.0
                if self._qualification_started_s is None
                else max(0.0, observed_s - self._qualification_started_s)
            )
            qualification_complete = (
                heartbeat_fresh
                and attitude_fresh
                and qualification_elapsed >= self.qualification_s
            )

            if self._error is not None:
                state = MavlinkLivenessState.ERROR
                reason = self._error
            elif not self._listener_open:
                state = MavlinkLivenessState.DISCONNECTED
                reason = "LISTENER_CLOSED"
            elif not self._identity_verified:
                if (
                    identity_wait_age is not None
                    and identity_wait_age >= self.initial_rx_timeout_s
                ):
                    state = MavlinkLivenessState.INITIAL_RX_TIMEOUT
                    reason = "EXPECTED_HEARTBEAT_NOT_RECEIVED"
                else:
                    state = MavlinkLivenessState.CONNECTING
                    reason = "AWAITING_EXPECTED_HEARTBEAT"
            elif self._active_command is not None:
                state = MavlinkLivenessState.RECOVERING if self._recovering else MavlinkLivenessState.QUALIFYING
                reason = "ACTIVE_COMMAND"
            elif self._stalled:
                state = MavlinkLivenessState.RX_STALLED
                reason = "ALL_EXPECTED_RX_SILENT"
            elif qualification_complete:
                state = MavlinkLivenessState.LIVE
                reason = None
                self._recovering = False
            elif self._recovering:
                state = MavlinkLivenessState.RECOVERING
                reason = "RECOVERY_QUALIFICATION"
            elif (
                self._last_attitude_rx_s is not None
                and not attitude_fresh
                and heartbeat_fresh
                and any_age is not None
                and any_age < self.all_rx_stall_s
            ):
                state = MavlinkLivenessState.ATTITUDE_STALE
                reason = "ATTITUDE_STALE"
            else:
                state = MavlinkLivenessState.QUALIFYING
                if not self._post_command_streams_observed():
                    reason = "AWAITING_POST_COMMAND_HEARTBEAT_ATTITUDE"
                elif not heartbeat_fresh and self._last_heartbeat_rx_s is not None:
                    reason = "HEARTBEAT_STALE"
                elif self._last_heartbeat_rx_s is None or self._last_attitude_rx_s is None:
                    reason = "AWAITING_HEARTBEAT_ATTITUDE"
                else:
                    reason = "QUALIFICATION"

            transport_current = (
                self._listener_open
                and self._identity_verified
                and self._error is None
                and not self._stalled
                and any_age is not None
                and any_age < self.all_rx_stall_s
            )
            return MavlinkSideHealthSnapshot(
                side=self.side,
                connection_generation=self._connection_generation,
                endpoint=self.endpoint,
                peer_host=self.peer_host,
                expected_system_id=self.expected_system_id,
                observed_system_id=self._observed_system_id,
                observed_component_id=self._observed_component_id,
                identity_verified=self._identity_verified,
                listener_open=self._listener_open,
                listener_opened_monotonic_s=self._listener_opened_s,
                listener_age_s=listener_age,
                last_any_rx_monotonic_s=self._last_any_rx_s,
                last_heartbeat_rx_monotonic_s=self._last_heartbeat_rx_s,
                last_attitude_rx_monotonic_s=self._last_attitude_rx_s,
                last_gcs_heartbeat_tx_monotonic_s=self._last_gcs_heartbeat_tx_s,
                any_rx_age_s=any_age,
                heartbeat_rx_age_s=heartbeat_age,
                attitude_rx_age_s=attitude_age,
                active_command=self._active_command,
                state=state,
                qualification_started_monotonic_s=self._qualification_started_s,
                qualification_elapsed_s=qualification_elapsed,
                qualification_required_s=self.qualification_s,
                rtcm_eligible=transport_current,
                command_eligible=transport_current and self._active_command is None,
                stall_reason=reason,
            )
