"""rtp timing owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Callable
from .policy import (
    H264_ACCESS_UNIT_KIND_NAL1_NON_IDR_INTRA,
    H264_NAL_TYPE_IDR,
    H264_NAL_TYPE_NON_IDR_SLICE,
    H264_NAL_TYPE_PPS,
    H264_NAL_TYPE_SPS,
    H264_RECOVERY_BASIS_IDR,
    H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL,
    MAX_RTCP_RATE_SAMPLES,
    MAX_RTCP_SR_ARRIVAL_CACHE,
    MAX_RTP_DISCONTINUITY_CACHE,
    MAX_RTP_PTS_MATCH_DELTA_NS,
    MAX_RTP_TIMING_CACHE,
    NTP_UNIX_EPOCH_OFFSET_S,
    RTP_CLOCK_RATE_HZ,
    UINT32_HALF_RANGE,
    UINT32_MODULUS,
)


def format_rtp_sequence_discontinuity(discontinuity: "RtpSequenceDiscontinuity") -> str:
    return (
        "RTP sequence discontinuity: "
        f"ssrc={discontinuity.ssrc}, "
        f"seq={discontinuity.previous_sequence}->{discontinuity.sequence}, "
        f"missing={discontinuity.missing_packets}"
    )


def h264_byte_stream_nal_unit_types(
    access_unit: bytes | bytearray | memoryview,
) -> tuple[int, ...]:
    """Return NAL types from a normalized Annex-B H.264 access unit.

    The live pipeline forces ``h264parse`` to emit byte-stream/alignment=au,
    so start codes are an independent boundary for inspecting NAL headers.
    Payload bytes are never searched as candidate headers: H.264 byte-stream
    emulation prevention guarantees that a real ``00 00 01`` begins a NAL.
    Malformed or non-Annex-B input therefore returns no types and fails closed.
    """

    try:
        data = (
            access_unit if isinstance(access_unit, (bytes, bytearray)) else memoryview(access_unit).cast("B").tobytes()
        )
    except (TypeError, ValueError):
        return ()
    nal_types: list[int] = []
    cursor = 0
    while True:
        # Searching for the three-byte suffix also finds a four-byte start
        # code at its second zero. bytes.find() keeps this inspection in C;
        # the Python camera thread does not walk multi-megabyte slice payloads.
        start_index = data.find(b"\x00\x00\x01", cursor)
        if start_index < 0:
            break
        header_index = start_index + 3
        if header_index >= len(data):
            break
        header = int(data[header_index])
        nal_type = header & 0x1F
        if not (header & 0x80) and not (nal_type == H264_NAL_TYPE_IDR and not (header & 0x60)):
            nal_types.append(nal_type)
        cursor = header_index + 1
    return tuple(nal_types)


def h264_access_unit_has_idr(access_unit: bytes | bytearray | memoryview) -> bool:
    """Prove that an AU contains an H.264 IDR slice (NAL unit type 5)."""

    return H264_NAL_TYPE_IDR in h264_byte_stream_nal_unit_types(access_unit)


def h264_nal1_non_idr_intra_metadata(
    access_unit: bytes | bytearray | memoryview,
    *,
    pts_ns: int | None,
    is_delta: bool,
    is_corrupted: bool = False,
) -> dict[str, object] | None:
    """Return positive Auto-capture provenance for one exact NAL1 AU.

    NAL type 1 alone also covers inter-predicted non-IDR pictures. The
    parser-owned ``DELTA_UNIT`` flag is therefore part of the operational
    capture contract. This evidence deliberately does not claim IDR or a
    random-access integrity boundary.
    """

    if pts_ns is None or bool(is_delta) or bool(is_corrupted):
        return None
    nal_types = h264_byte_stream_nal_unit_types(access_unit)
    if H264_NAL_TYPE_NON_IDR_SLICE not in nal_types or H264_NAL_TYPE_IDR in nal_types:
        return None
    return {
        "h264_access_unit_kind": H264_ACCESS_UNIT_KIND_NAL1_NON_IDR_INTRA,
        "h264_access_unit_nal_types": list(nal_types),
        "h264_access_unit_is_delta": False,
        "h264_access_unit_pts_ns": int(pts_ns),
        "h264_nal1_non_idr_intra": True,
        "h264_random_access_guaranteed": False,
    }


def h264_access_unit_recovery_basis(
    access_unit: bytes | bytearray | memoryview,
    *,
    is_delta: bool,
    allow_non_idr_intra_experimental: bool = False,
) -> str | None:
    """Classify one parser-normalized AU without relabelling NAL1 as IDR.

    ``h264parse`` owns the AU/non-delta classification.  The experimental
    WIRIS branch additionally requires in-band SPS and PPS in that exact AU;
    a bare NAL1 header or a parser flag alone is never a recovery candidate.
    """

    nal_types = h264_byte_stream_nal_unit_types(access_unit)
    if H264_NAL_TYPE_IDR in nal_types:
        return H264_RECOVERY_BASIS_IDR
    if not allow_non_idr_intra_experimental or bool(is_delta):
        return None
    required_types = {
        H264_NAL_TYPE_NON_IDR_SLICE,
        H264_NAL_TYPE_SPS,
        H264_NAL_TYPE_PPS,
    }
    if required_types.issubset(nal_types):
        return H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL
    return None


def ntp64_to_unix_seconds(ntp_timestamp: int) -> float | None:
    if not ntp_timestamp:
        return None
    seconds = int(ntp_timestamp) >> 32
    fraction = int(ntp_timestamp) & 0xFFFFFFFF
    return seconds - NTP_UNIX_EPOCH_OFFSET_S + fraction / float(UINT32_MODULUS)


def rtp_timestamp_delta(timestamp: int, reference: int) -> int:
    return (int(timestamp) - int(reference) + UINT32_HALF_RANGE) % UINT32_MODULUS - UINT32_HALF_RANGE


def estimate_rtcp_ntp_unix_seconds(
    *,
    rtp_timestamp: int,
    sr_rtp_timestamp: int,
    sr_ntp_unix_s: float,
    clock_rate_hz: int,
) -> float | None:
    if clock_rate_hz <= 0:
        return None
    delta_ticks = rtp_timestamp_delta(rtp_timestamp, sr_rtp_timestamp)
    return float(sr_ntp_unix_s) + delta_ticks / float(clock_rate_hz)


def _gst_structure_value(structure, key: str, default=None):
    try:
        return structure.get_value(key)
    except Exception:
        return default


@dataclass(frozen=True)
class RtpFrameTiming:
    rtp_timestamp: int
    rtp_ssrc: int
    rtp_marker_seq: int
    rtp_pts_s: float | None


@dataclass(frozen=True)
class RtpSequenceDiscontinuity:
    ssrc: int
    previous_sequence: int
    sequence: int
    missing_packets: int
    observed_monotonic_s: float
    rtp_pts_ns: int | None = None


@dataclass(frozen=True)
class RtpSequenceRecovery:
    ssrc: int
    resume_pts_ns: int
    observed_monotonic_s: float
    blocked_duration_s: float
    recovery_basis: str = H264_RECOVERY_BASIS_IDR


@dataclass(frozen=True)
class H264AccessUnitDiscontinuity:
    reason: str
    pts_ns: int | None
    observed_monotonic_s: float


@dataclass
class RtpDiscontinuityReporter:
    interval_s: float = 5.0
    total_events: int = 0
    total_missing_packets: int = 0
    _reported_events: int = 0
    _reported_missing_packets: int = 0
    _last_report_s: float | None = None

    def observe(
        self,
        discontinuity: RtpSequenceDiscontinuity,
        *,
        now_s: float | None = None,
    ) -> str | None:
        observed_s = discontinuity.observed_monotonic_s if now_s is None else float(now_s)
        self.total_events += 1
        self.total_missing_packets += int(discontinuity.missing_packets)
        if self._last_report_s is None:
            self._last_report_s = observed_s
            return None
        if observed_s - self._last_report_s < max(0.1, float(self.interval_s)):
            return None
        return self._render(observed_s)

    def flush(self, *, now_s: float | None = None) -> str | None:
        observed_s = time.monotonic() if now_s is None else float(now_s)
        return self._render(observed_s)

    def _render(self, observed_s: float) -> str | None:
        event_delta = self.total_events - self._reported_events
        missing_delta = self.total_missing_packets - self._reported_missing_packets
        if event_delta <= 0:
            return None
        self._reported_events = self.total_events
        self._reported_missing_packets = self.total_missing_packets
        self._last_report_s = observed_s
        return (
            "RTP discontinuity summary: "
            f"events={self.total_events} (+{event_delta}), "
            f"missing={self.total_missing_packets} (+{missing_delta})"
        )


@dataclass(frozen=True)
class RtcpSenderReportTiming:
    ssrc: int
    ntp_raw: int
    ntp_unix_s: float
    rtp_timestamp: int
    clock_rate_hz: int
    received_monotonic_s: float
    received_wall_s: float | None = None
    arrival_basis: str = "poll-detected"


def rtcp_sender_report_pc_offset_s(
    *,
    sr_ntp_unix_s: float,
    received_wall_s: float,
) -> float | None:
    offset_s = float(sr_ntp_unix_s) - float(received_wall_s)
    return offset_s if abs(offset_s) <= 60.0 else None


def sender_report_clock_rates(
    previous: RtcpSenderReportTiming,
    current: RtcpSenderReportTiming,
) -> dict[str, float] | None:
    """Compare the camera's NTP and media clocks against the PC clock.

    Both rates are "camera seconds advanced per PC second" between two Sender
    Reports, so 1.0 means agreement. ``ntp_rate`` below 1.0 means the camera's
    wall clock is losing time (the drift that inflates transit delay);
    ``media_rate`` below 1.0 means its RTP clock ticks slower than the declared
    rate, which inflates transport delay because the media time between two
    reports is then undercounted.
    """

    pc_elapsed_s = current.received_monotonic_s - previous.received_monotonic_s
    if pc_elapsed_s <= 0.0 or current.clock_rate_hz <= 0:
        return None
    ntp_elapsed_s = current.ntp_unix_s - previous.ntp_unix_s
    media_ticks = rtp_timestamp_delta(current.rtp_timestamp, previous.rtp_timestamp)
    media_elapsed_s = media_ticks / float(current.clock_rate_hz)
    return {
        "pc_elapsed_s": pc_elapsed_s,
        "ntp_elapsed_s": ntp_elapsed_s,
        "media_elapsed_s": media_elapsed_s,
        "ntp_rate": ntp_elapsed_s / pc_elapsed_s,
        "media_rate": media_elapsed_s / pc_elapsed_s,
    }


def robust_sender_report_clock_rates(
    samples: deque[dict[str, float]] | list[dict[str, float]],
) -> dict[str, float] | None:
    """Return median clock rates over recent Sender Report intervals."""

    valid = [
        sample
        for sample in samples
        if all(
            key in sample
            and isinstance(sample[key], (int, float))
            and math.isfinite(float(sample[key]))
            and float(sample[key]) > 0.0
            for key in ("ntp_rate", "media_rate")
        )
    ]
    if not valid:
        return None
    return {
        "ntp_rate": float(median(float(sample["ntp_rate"]) for sample in valid)),
        "media_rate": float(median(float(sample["media_rate"]) for sample in valid)),
    }


class GstRtpTimingTracker:
    def __init__(
        self,
        Gst,
        GstRtp,
        *,
        status_callback: Callable[[str], None] | None = None,
        allow_non_idr_intra_experimental: bool = False,
        allow_unverified_encoded_output: bool = False,
    ) -> None:
        self.Gst = Gst
        self.GstRtp = GstRtp
        self._status_callback = status_callback
        self._allow_non_idr_intra_experimental = bool(allow_non_idr_intra_experimental)
        self._allow_unverified_encoded_output = bool(allow_unverified_encoded_output)
        self._lock = threading.Lock()
        self._pending_rtp_frames: deque[RtpFrameTiming] = deque(maxlen=MAX_RTP_TIMING_CACHE)
        self._timing_by_pts_ns: dict[int, RtpFrameTiming] = {}
        self._pts_order: deque[int] = deque(maxlen=MAX_RTP_TIMING_CACHE)
        self._nal1_metadata_by_pts_ns: dict[int, dict[str, object]] = {}
        self._nal1_metadata_pts_order: deque[int] = deque()
        self._active_rtp_ssrc: int | None = None
        self._last_rtp_sequence: int | None = None
        self._rtp_packet_count = 0
        self._rtp_gap_events_total = 0
        self._rtp_missing_packets_total = 0
        self._h264_access_units_total = 0
        self._last_rtp_packet_monotonic_s: float | None = None
        self._rtp_discontinuities: deque[RtpSequenceDiscontinuity] = deque(maxlen=MAX_RTP_DISCONTINUITY_CACHE)
        self._rtp_recoveries: deque[RtpSequenceRecovery] = deque(maxlen=MAX_RTP_DISCONTINUITY_CACHE)
        self._h264_access_unit_discontinuities: deque[H264AccessUnitDiscontinuity] = deque(
            maxlen=MAX_RTP_DISCONTINUITY_CACHE
        )
        # A newly attached decoder can start in the middle of a dependency
        # chain. Treat startup exactly like post-loss recovery: no compressed
        # or decoded output is trusted until a parser-normalized IDR AU enters
        # the decoder and that exact PTS returns from it.
        self._decoded_output_blocked = True
        self._encoded_output_blocked = True
        self._blocked_after_pts_ns: int | None = None
        self._blocked_since_monotonic_s: float | None = time.monotonic()
        self._recovery_candidate_pts_ns: int | None = None
        self._recovery_candidate_observed_s: float | None = None
        self._recovery_candidate_basis: str | None = None
        self._output_recovery_basis: str | None = None
        self._encoded_recovery_marker_pts_ns: int | None = None
        self._accepted_access_unit_seen = False
        self._access_unit_discontinuity_reported = False
        self._sr_by_ssrc: dict[int, RtcpSenderReportTiming] = {}
        self._sr_count_by_ssrc: dict[int, int] = {}
        self._sr_history_by_ssrc: dict[int, deque[RtcpSenderReportTiming]] = {}
        self._clock_rate_samples_by_ssrc: dict[int, deque[dict[str, float]]] = {}
        self._clock_rates_by_ssrc: dict[int, dict[str, float]] = {}
        self._sr_arrival_by_key: dict[tuple[int, int], tuple[float, float]] = {}
        self._sr_arrival_order: deque[tuple[int, int]] = deque()
        self._rtcp_probe_pads: set[str] = set()
        self._manager = None
        self._last_rtcp_poll_s = 0.0
        self._last_clock_rates: dict[str, float] | None = None

    def _emit_status(self, text: str) -> None:
        if self._status_callback is not None:
            self._status_callback(text)

    def _remember_nal1_metadata(self, metadata: dict[str, object]) -> None:
        pts_ns = int(metadata["h264_access_unit_pts_ns"])
        with self._lock:
            if pts_ns not in self._nal1_metadata_by_pts_ns:
                self._nal1_metadata_pts_order.append(pts_ns)
            self._nal1_metadata_by_pts_ns[pts_ns] = dict(metadata)
            while len(self._nal1_metadata_pts_order) > MAX_RTP_TIMING_CACHE:
                stale_pts_ns = self._nal1_metadata_pts_order.popleft()
                self._nal1_metadata_by_pts_ns.pop(stale_pts_ns, None)

    def _pop_nal1_metadata(self, pts_ns: int | None) -> dict[str, object] | None:
        if pts_ns is None:
            return None
        with self._lock:
            normalized_pts_ns = int(pts_ns)
            metadata = self._nal1_metadata_by_pts_ns.pop(normalized_pts_ns, None)
            if metadata is not None:
                try:
                    self._nal1_metadata_pts_order.remove(normalized_pts_ns)
                except ValueError:
                    pass
        return None if metadata is None else dict(metadata)

    def register_rtp_sequence(
        self,
        *,
        ssrc: int,
        sequence: int,
        observed_monotonic_s: float | None = None,
        rtp_pts_ns: int | None = None,
    ) -> RtpSequenceDiscontinuity | None:
        """Record forward gaps in the active SSRC's 16-bit sequence space."""

        normalized_ssrc = int(ssrc)
        normalized_sequence = int(sequence) & 0xFFFF
        observed_s = time.monotonic() if observed_monotonic_s is None else float(observed_monotonic_s)
        normalized_pts_ns = None if rtp_pts_ns is None else int(rtp_pts_ns)
        with self._lock:
            self._rtp_packet_count += 1
            self._last_rtp_packet_monotonic_s = observed_s
            if self._active_rtp_ssrc != normalized_ssrc:
                previous_ssrc = self._active_rtp_ssrc
                self._active_rtp_ssrc = normalized_ssrc
                self._last_rtp_sequence = normalized_sequence
                if previous_ssrc is not None:
                    self._h264_access_unit_discontinuities.append(
                        H264AccessUnitDiscontinuity(
                            reason="ssrc-change",
                            pts_ns=normalized_pts_ns,
                            observed_monotonic_s=observed_s,
                        )
                    )
                    self._access_unit_discontinuity_reported = True
                    # A new SSRC owns a new RTP timestamp domain. Do not carry
                    # the prior stream's high PTS threshold into its recovery.
                    self._blocked_after_pts_ns = None
                    self._latch_compressed_discontinuity_locked(
                        observed_s=observed_s,
                        pts_ns=normalized_pts_ns,
                    )
                self._clear_frame_timing_locked()
                return None
            previous = self._last_rtp_sequence
            if previous is None:
                self._last_rtp_sequence = normalized_sequence
                return None
            delta = (normalized_sequence - previous) & 0xFFFF
            if delta == 0 or delta >= 0x8000:
                # Duplicate or older/reordered packets do not move the forward
                # baseline and therefore do not create a false discontinuity.
                return None
            self._last_rtp_sequence = normalized_sequence
            if delta == 1:
                return None
            discontinuity = RtpSequenceDiscontinuity(
                ssrc=normalized_ssrc,
                previous_sequence=previous,
                sequence=normalized_sequence,
                missing_packets=delta - 1,
                observed_monotonic_s=observed_s,
                rtp_pts_ns=normalized_pts_ns,
            )
            self._rtp_discontinuities.append(discontinuity)
            self._rtp_gap_events_total += 1
            self._rtp_missing_packets_total += int(discontinuity.missing_packets)
            # Popping the status event must not release this latch. Only a
            # parser-proven IDR followed by that exact decoded PTS can do so.
            self._latch_compressed_discontinuity_locked(
                observed_s=observed_s,
                pts_ns=normalized_pts_ns,
            )
            self._clear_frame_timing_locked()
            return discontinuity

    def has_recent_rtp_input(
        self,
        *,
        now_s: float | None = None,
        max_age_s: float = 2.0,
    ) -> bool:
        """True only when the compressed-input boundary is still receiving RTP."""

        observed_s = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            last_s = self._last_rtp_packet_monotonic_s
            return bool(
                self._rtp_packet_count > 0 and last_s is not None and observed_s - last_s <= max(0.0, float(max_age_s))
            )

    def rtp_metrics_snapshot(self) -> dict[str, int | None]:
        """Return transport and parser counters for the UI rate display."""

        with self._lock:
            return {
                "rtp_packets_received_total": int(self._rtp_packet_count),
                "rtp_gap_events_total": int(self._rtp_gap_events_total),
                "rtp_missing_packets_total": int(self._rtp_missing_packets_total),
                "h264_access_units_total": int(self._h264_access_units_total),
                "rtp_active_ssrc": self._active_rtp_ssrc,
            }

    def pop_rtp_discontinuity(self) -> RtpSequenceDiscontinuity | None:
        with self._lock:
            if not self._rtp_discontinuities:
                return None
            return self._rtp_discontinuities.popleft()

    def decoded_output_blocked(self) -> bool:
        with self._lock:
            return self._decoded_output_blocked

    def encoded_output_blocked(self) -> bool:
        with self._lock:
            return self._encoded_output_blocked

    def blocked_duration_s(self, *, now_s: float | None = None) -> float:
        observed_s = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            if not self._decoded_output_blocked or self._blocked_since_monotonic_s is None:
                return 0.0
            return max(0.0, observed_s - self._blocked_since_monotonic_s)

    def _latch_compressed_discontinuity_locked(
        self,
        *,
        observed_s: float,
        pts_ns: int | None,
    ) -> None:
        self._decoded_output_blocked = True
        self._encoded_output_blocked = True
        if self._blocked_since_monotonic_s is None:
            self._blocked_since_monotonic_s = observed_s
        if pts_ns is None:
            self._blocked_after_pts_ns = None
        elif self._blocked_after_pts_ns is None:
            self._blocked_after_pts_ns = int(pts_ns)
        else:
            self._blocked_after_pts_ns = max(self._blocked_after_pts_ns, int(pts_ns))
        self._recovery_candidate_pts_ns = None
        self._recovery_candidate_observed_s = None
        self._recovery_candidate_basis = None
        self._encoded_recovery_marker_pts_ns = None

    def _register_access_unit_locked(
        self,
        *,
        pts_ns: int | None,
        is_delta: bool,
        is_corrupted: bool,
        is_random_access: bool,
        recovery_basis: str | None,
        observed_s: float,
    ) -> bool:
        _ = is_delta
        normalized_basis = str(recovery_basis or "").strip()
        if not normalized_basis and is_random_access:
            normalized_basis = H264_RECOVERY_BASIS_IDR
        allowed_basis = normalized_basis == H264_RECOVERY_BASIS_IDR or (
            self._allow_non_idr_intra_experimental
            and normalized_basis == H264_RECOVERY_BASIS_NON_IDR_INTRA_EXPERIMENTAL
        )
        if not self._decoded_output_blocked or not allowed_basis or is_corrupted or pts_ns is None:
            return False
        # The AU containing the missing RTP packet is itself suspect even when
        # it contains an IDR. Recovery must be a later, complete IDR AU.
        if self._blocked_after_pts_ns is not None and pts_ns <= self._blocked_after_pts_ns:
            return False
        self._recovery_candidate_pts_ns = int(pts_ns)
        self._recovery_candidate_observed_s = observed_s
        self._recovery_candidate_basis = normalized_basis
        self._encoded_recovery_marker_pts_ns = int(pts_ns)
        self._encoded_output_blocked = False
        return True

    def register_access_unit(
        self,
        *,
        pts_ns: int | None,
        is_delta: bool,
        is_corrupted: bool,
        is_random_access: bool = False,
        recovery_basis: str | None = None,
        observed_monotonic_s: float | None = None,
    ) -> bool:
        observed_s = time.monotonic() if observed_monotonic_s is None else float(observed_monotonic_s)
        normalized_pts_ns = None if pts_ns is None else int(pts_ns)
        with self._lock:
            return self._register_access_unit_locked(
                pts_ns=normalized_pts_ns,
                is_delta=bool(is_delta),
                is_corrupted=bool(is_corrupted),
                is_random_access=bool(is_random_access),
                recovery_basis=recovery_basis,
                observed_s=observed_s,
            )

    def should_drop_encoded_access_unit(
        self,
        *,
        pts_ns: int | None,
        is_delta: bool,
        is_corrupted: bool,
        is_discontinuous: bool,
        is_random_access: bool = False,
        recovery_basis: str | None = None,
        observed_monotonic_s: float | None = None,
    ) -> bool:
        """Keep an invalid AU and every dependent AU out of all consumers."""

        observed_s = time.monotonic() if observed_monotonic_s is None else float(observed_monotonic_s)
        normalized_pts_ns = None if pts_ns is None else int(pts_ns)
        normalized_basis = str(recovery_basis or "").strip()
        random_access_candidate = bool(is_random_access or normalized_basis)
        with self._lock:
            # GstBaseParse marks the first valid IDR DISCONT at a normal stream
            # or segment start. An explicitly parsed IDR is a safe reset for
            # that marker; DISCONT on every other accepted chain remains strict.
            reason = (
                "missing-pts"
                if normalized_pts_ns is None
                else "corrupted"
                if is_corrupted
                else "discont"
                if (is_discontinuous and self._accepted_access_unit_seen and not random_access_candidate)
                else None
            )
            if reason is not None:
                is_new_event = (
                    not self._access_unit_discontinuity_reported
                    or not self._encoded_output_blocked
                    or self._recovery_candidate_pts_ns is not None
                )
                if is_new_event:
                    self._h264_access_unit_discontinuities.append(
                        H264AccessUnitDiscontinuity(
                            reason=reason,
                            pts_ns=normalized_pts_ns,
                            observed_monotonic_s=observed_s,
                        )
                    )
                    self._access_unit_discontinuity_reported = True
                self._latch_compressed_discontinuity_locked(
                    observed_s=observed_s,
                    pts_ns=normalized_pts_ns,
                )
                return True
            if not self._encoded_output_blocked:
                self._accepted_access_unit_seen = True
                return False
            if self._register_access_unit_locked(
                pts_ns=normalized_pts_ns,
                is_delta=bool(is_delta),
                is_corrupted=bool(is_corrupted),
                is_random_access=random_access_candidate,
                recovery_basis=normalized_basis,
                observed_s=observed_s,
            ):
                self._accepted_access_unit_seen = True
                return False
            return True

    def pop_h264_access_unit_discontinuity(self) -> H264AccessUnitDiscontinuity | None:
        with self._lock:
            if not self._h264_access_unit_discontinuities:
                return None
            return self._h264_access_unit_discontinuities.popleft()

    def consume_encoded_recovery_marker(self, pts_ns: int | None) -> bool:
        normalized_pts_ns = None if pts_ns is None else int(pts_ns)
        with self._lock:
            if normalized_pts_ns is None or self._encoded_recovery_marker_pts_ns != normalized_pts_ns:
                return False
            self._encoded_recovery_marker_pts_ns = None
            return True

    def should_suppress_decoded_buffer(self, pts_ns: int | None) -> bool:
        normalized_pts_ns = None if pts_ns is None else int(pts_ns)
        with self._lock:
            if not self._decoded_output_blocked:
                return False
            resume_pts_ns = self._recovery_candidate_pts_ns
            if resume_pts_ns is None or normalized_pts_ns is None or normalized_pts_ns != resume_pts_ns:
                return True
            observed_s = self._recovery_candidate_observed_s or time.monotonic()
            blocked_since_s = self._blocked_since_monotonic_s or observed_s
            recovery_basis = self._recovery_candidate_basis or H264_RECOVERY_BASIS_IDR
            self._rtp_recoveries.append(
                RtpSequenceRecovery(
                    ssrc=int(self._active_rtp_ssrc or 0),
                    resume_pts_ns=resume_pts_ns,
                    observed_monotonic_s=observed_s,
                    blocked_duration_s=max(0.0, observed_s - blocked_since_s),
                    recovery_basis=recovery_basis,
                )
            )
            self._decoded_output_blocked = False
            self._encoded_output_blocked = False
            self._blocked_after_pts_ns = None
            self._blocked_since_monotonic_s = None
            self._recovery_candidate_pts_ns = None
            self._recovery_candidate_observed_s = None
            self._recovery_candidate_basis = None
            self._encoded_recovery_marker_pts_ns = None
            self._access_unit_discontinuity_reported = False
            self._output_recovery_basis = recovery_basis
            return False

    def current_output_recovery_basis(self) -> str | None:
        with self._lock:
            return self._output_recovery_basis

    def pop_rtp_recovery(self) -> RtpSequenceRecovery | None:
        with self._lock:
            if not self._rtp_recoveries:
                return None
            return self._rtp_recoveries.popleft()

    def _clear_frame_timing_locked(self) -> None:
        self._pending_rtp_frames.clear()
        self._timing_by_pts_ns.clear()
        self._pts_order.clear()
        self._nal1_metadata_by_pts_ns.clear()
        self._nal1_metadata_pts_order.clear()

    def _register_sender_report(self, timing: RtcpSenderReportTiming) -> str | None:
        with self._lock:
            previous = self._sr_by_ssrc.get(timing.ssrc)
            if previous is not None and previous.ntp_raw == timing.ntp_raw:
                # The poll re-reads the same report from the GStreamer session
                # stats every 250 ms. Keep the timestamps from when this report
                # first appeared, so rtcp_sr_age_s measures how stale the anchor
                # is instead of how recently we polled.
                return None
            self._sr_count_by_ssrc[timing.ssrc] = self._sr_count_by_ssrc.get(timing.ssrc, 0) + 1
            self._sr_by_ssrc[timing.ssrc] = timing
            self._sr_history_by_ssrc.setdefault(timing.ssrc, deque(maxlen=8)).append(timing)
        if previous is None:
            return None
        interval_s = timing.received_monotonic_s - previous.received_monotonic_s
        if interval_s <= 0.0:
            return None
        rates = sender_report_clock_rates(previous, timing)
        if rates is not None:
            with self._lock:
                samples = self._clock_rate_samples_by_ssrc.setdefault(
                    timing.ssrc,
                    deque(maxlen=MAX_RTCP_RATE_SAMPLES),
                )
                samples.append(rates)
                robust_rates = robust_sender_report_clock_rates(samples)
                if robust_rates is not None:
                    self._clock_rates_by_ssrc[timing.ssrc] = robust_rates
                    self._last_clock_rates = robust_rates
        rate_text = ""
        robust_rates = self._clock_rates_by_ssrc.get(timing.ssrc)
        if robust_rates is not None:
            rate_text = f", ntp_rate={robust_rates['ntp_rate']:.6f}, media_rate={robust_rates['media_rate']:.6f}"
        return (
            "[RTCP] Sender Report 갱신: "
            f"ssrc={timing.ssrc}, interval={interval_s:.2f}s, "
            f"ntp={timing.ntp_unix_s:.6f}{rate_text}"
        )

    def poll_sender_reports(self, *, now_s: float | None = None) -> None:
        """Refresh RTCP statistics without requiring a decoded frame."""

        self._poll_rtcp_sender_reports(time.monotonic() if now_s is None else float(now_s))

    def sender_report_observations(
        self,
        *,
        now_s: float | None = None,
    ) -> list[dict[str, int | float | str | None]]:
        """Return bounded pad-arrival SR evidence independent of decode output."""

        observed_s = time.monotonic() if now_s is None else float(now_s)
        with self._lock:
            histories = {ssrc: tuple(rows) for ssrc, rows in self._sr_history_by_ssrc.items()}
            counts = dict(self._sr_count_by_ssrc)
            rates = {ssrc: dict(values) for ssrc, values in self._clock_rates_by_ssrc.items()}
        observations: list[dict[str, int | float | str | None]] = []
        for ssrc in sorted(histories):
            clock_rates = rates.get(ssrc, {})
            previous_timing: RtcpSenderReportTiming | None = None
            for timing in histories[ssrc]:
                offset_s = (
                    None
                    if timing.received_wall_s is None
                    else rtcp_sender_report_pc_offset_s(
                        sr_ntp_unix_s=timing.ntp_unix_s,
                        received_wall_s=timing.received_wall_s,
                    )
                )
                raw_offset_s = (
                    None if timing.received_wall_s is None else float(timing.ntp_unix_s) - float(timing.received_wall_s)
                )
                host_clock_step_s = None
                if (
                    previous_timing is not None
                    and previous_timing.received_wall_s is not None
                    and timing.received_wall_s is not None
                ):
                    host_clock_step_s = (
                        float(timing.received_wall_s)
                        - float(previous_timing.received_wall_s)
                        - (float(timing.received_monotonic_s) - float(previous_timing.received_monotonic_s))
                    )
                observations.append(
                    {
                        "ssrc": int(timing.ssrc),
                        "ntp_raw": int(timing.ntp_raw),
                        "ntp_unix_s": float(timing.ntp_unix_s),
                        "rtp_timestamp": int(timing.rtp_timestamp),
                        "clock_rate_hz": int(timing.clock_rate_hz),
                        "received_monotonic_s": float(timing.received_monotonic_s),
                        "received_wall_s": timing.received_wall_s,
                        "received_count": int(counts.get(ssrc, 0)),
                        "age_s": max(0.0, observed_s - timing.received_monotonic_s),
                        "arrival_basis": str(timing.arrival_basis),
                        "sr_pc_offset_s": offset_s,
                        "sr_pc_offset_raw_s": raw_offset_s,
                        "ntp_rate": clock_rates.get("ntp_rate"),
                        "media_rate": clock_rates.get("media_rate"),
                        "host_clock_step_s": host_clock_step_s,
                    }
                )
                previous_timing = timing
        observations.sort(
            key=lambda row: (
                float(row["received_monotonic_s"]),
                int(row["ssrc"]),
                int(row["ntp_raw"]),
            )
        )
        return observations

    def attach(self, pipeline) -> None:
        source = pipeline.get_by_name("src")
        depay = pipeline.get_by_name("depay")
        parser = pipeline.get_by_name("parse")
        if source is not None:
            source.connect("new-manager", self._handle_new_manager)
        if depay is None:
            return
        sink_pad = depay.get_static_pad("sink")
        src_pad = depay.get_static_pad("src")
        if sink_pad is not None:
            sink_pad.add_probe(self.Gst.PadProbeType.BUFFER, self._capture_rtp_packet)
        if src_pad is not None:
            src_pad.add_probe(self.Gst.PadProbeType.BUFFER, self._map_depay_buffer_pts)
        if parser is not None:
            parser_src_pad = parser.get_static_pad("src")
            if parser_src_pad is not None:
                parser_src_pad.add_probe(self.Gst.PadProbeType.BUFFER, self._observe_access_unit)

    def frame_metadata(
        self,
        buffer,
        now_s: float,
        *,
        received_wall_s: float | None = None,
    ) -> dict[str, object] | None:
        received_wall_s = time.time() if received_wall_s is None else float(received_wall_s)
        self._poll_rtcp_sender_reports(now_s)
        pts_ns = None if buffer.pts == self.Gst.CLOCK_TIME_NONE else int(buffer.pts)
        metadata = self._pop_nal1_metadata(pts_ns) or {}
        if metadata:
            metadata["decoded_buffer_pts_ns"] = pts_ns
        frame_timing = self._pop_frame_timing(pts_ns)
        if frame_timing is None:
            return metadata or None
        metadata.update(
            {
                "rtp_timestamp": frame_timing.rtp_timestamp,
                "rtp_ssrc": frame_timing.rtp_ssrc,
                "rtp_marker_seq": frame_timing.rtp_marker_seq,
            }
        )
        recovery_basis = self.current_output_recovery_basis()
        if recovery_basis:
            metadata.update(
                {
                    "media_recovery_basis": recovery_basis,
                    "media_integrity_mode": (
                        "experimental_non_idr_intra" if self._allow_non_idr_intra_experimental else "strict_idr"
                    ),
                    "media_random_access_guaranteed": (recovery_basis == H264_RECOVERY_BASIS_IDR),
                }
            )
        if frame_timing.rtp_pts_s is not None:
            metadata["rtp_pts_s"] = frame_timing.rtp_pts_s
        sr_timing = self._sr_by_ssrc.get(frame_timing.rtp_ssrc)
        if sr_timing is None:
            return metadata
        sr_received_count = self._sr_count_by_ssrc.get(frame_timing.rtp_ssrc, 0)
        frame_ntp_s = estimate_rtcp_ntp_unix_seconds(
            rtp_timestamp=frame_timing.rtp_timestamp,
            sr_rtp_timestamp=sr_timing.rtp_timestamp,
            sr_ntp_unix_s=sr_timing.ntp_unix_s,
            clock_rate_hz=sr_timing.clock_rate_hz,
        )
        metadata.update(
            {
                "rtcp_sr_ntp_raw": sr_timing.ntp_raw,
                "rtcp_sr_ntp_unix_s": sr_timing.ntp_unix_s,
                "rtcp_sr_rtp_timestamp": sr_timing.rtp_timestamp,
                "rtcp_clock_rate_hz": sr_timing.clock_rate_hz,
                "rtcp_sr_age_s": max(0.0, now_s - sr_timing.received_monotonic_s),
                "rtcp_sr_received_wall_s": sr_timing.received_wall_s,
                "rtcp_sr_received_count": sr_received_count,
            }
        )
        clock_rates = self._clock_rates_by_ssrc.get(frame_timing.rtp_ssrc) or self._last_clock_rates
        if clock_rates is not None:
            metadata["rtcp_camera_ntp_rate"] = clock_rates["ntp_rate"]
            metadata["rtcp_camera_media_rate"] = clock_rates["media_rate"]
        metadata["rtcp_sr_arrival_basis"] = sr_timing.arrival_basis
        sr_pc_offset_s = None
        if sr_timing.received_wall_s is not None:
            sr_pc_offset_s = rtcp_sender_report_pc_offset_s(
                sr_ntp_unix_s=sr_timing.ntp_unix_s,
                received_wall_s=sr_timing.received_wall_s,
            )
            if sr_pc_offset_s is not None:
                metadata["rtcp_sr_pc_offset_s"] = sr_pc_offset_s
        if frame_ntp_s is not None:
            # An RTCP Sender Report can describe an older encoded frame. Its
            # arrival delay is a transport/encoder diagnostic, not proof that
            # the camera clock is wrong, so keep the RTP->NTP mapping intact.
            metadata["rtcp_frame_ntp_unix_s"] = frame_ntp_s
            metadata["rtcp_frame_received_wall_s"] = received_wall_s
            receive_delay_s = received_wall_s - frame_ntp_s
            metadata["rtcp_frame_receive_delay_s"] = receive_delay_s
            if sr_pc_offset_s is not None:
                # receive_delay carries the camera's absolute clock error, so it
                # grows without bound as that clock drifts. Adding the report's
                # own PC offset cancels the sr_ntp term, leaving
                #   (frame arrival - SR arrival) - (media time between them),
                # a transport/encoder figure that stays flat under clock drift.
                metadata["rtcp_frame_transport_delay_s"] = receive_delay_s + sr_pc_offset_s
        return metadata

    def _handle_new_manager(self, _source, manager) -> None:
        with self._lock:
            self._manager = manager
        try:
            manager.connect("pad-added", self._handle_manager_pad_added)
            iterator = manager.iterate_pads()
            while True:
                result, pad = iterator.next()
                if result != self.Gst.IteratorResult.OK:
                    break
                self._handle_manager_pad_added(manager, pad)
        except Exception as exc:
            self._emit_status(f"[RTCP] 실제 도착 probe 연결 실패, 통계 폴링 fallback: {exc}")

    def _handle_manager_pad_added(self, _manager, pad) -> None:
        try:
            pad_name = str(pad.get_name())
        except Exception:
            return
        if not pad_name.startswith("recv_rtcp_sink_") or pad_name in self._rtcp_probe_pads:
            return
        try:
            pad.add_probe(self.Gst.PadProbeType.BUFFER, self._capture_rtcp_packet)
        except Exception as exc:
            self._emit_status(f"[RTCP] {pad_name} probe 연결 실패, 통계 폴링 fallback: {exc}")
            return
        self._rtcp_probe_pads.add(pad_name)
        self._emit_status(f"[RTCP] 실제 Sender Report 도착시각 probe 연결: {pad_name}")

    def _remember_sender_report_arrival(
        self,
        *,
        ssrc: int,
        ntp_raw: int,
        received_monotonic_s: float,
        received_wall_s: float,
    ) -> None:
        key = (int(ssrc), int(ntp_raw))
        with self._lock:
            if key not in self._sr_arrival_by_key:
                self._sr_arrival_order.append(key)
            self._sr_arrival_by_key[key] = (float(received_monotonic_s), float(received_wall_s))
            while len(self._sr_arrival_by_key) > MAX_RTCP_SR_ARRIVAL_CACHE and self._sr_arrival_order:
                old_key = self._sr_arrival_order.popleft()
                self._sr_arrival_by_key.pop(old_key, None)

    def _sender_report_arrival(self, *, ssrc: int, ntp_raw: int) -> tuple[float, float] | None:
        with self._lock:
            return self._sr_arrival_by_key.get((int(ssrc), int(ntp_raw)))

    def _capture_rtcp_packet(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None:
            return self.Gst.PadProbeReturn.OK
        received_monotonic_s = time.monotonic()
        received_wall_s = time.time()
        rtcp_buffer = self.GstRtp.RTCPBuffer()
        try:
            success = self.GstRtp.RTCPBuffer.map(
                buffer,
                self.Gst.MapFlags.READ,
                rtcp_buffer,
            )
        except Exception:
            return self.Gst.PadProbeReturn.OK
        if not success:
            return self.Gst.PadProbeReturn.OK
        try:
            packet = self.GstRtp.RTCPPacket()
            if not rtcp_buffer.get_first_packet(packet):
                return self.Gst.PadProbeReturn.OK
            while True:
                if packet.get_type() == self.GstRtp.RTCPType.SR:
                    ssrc, ntp_raw, _rtp_timestamp, _packet_count, _octet_count = packet.sr_get_sender_info()
                    self._remember_sender_report_arrival(
                        ssrc=int(ssrc),
                        ntp_raw=int(ntp_raw),
                        received_monotonic_s=received_monotonic_s,
                        received_wall_s=received_wall_s,
                    )
                if not packet.move_to_next():
                    break
        except Exception:
            # Keep the stats-based path alive if a platform's GI bindings do
            # not expose one of the RTCP packet accessors.
            return self.Gst.PadProbeReturn.OK
        finally:
            rtcp_buffer.unmap()
        return self.Gst.PadProbeReturn.OK

    def _capture_rtp_packet(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None:
            return self.Gst.PadProbeReturn.OK
        success, rtp_buffer = self.GstRtp.RTPBuffer.map(buffer, self.Gst.MapFlags.READ)
        if not success:
            return self.Gst.PadProbeReturn.OK
        try:
            ssrc = int(rtp_buffer.get_ssrc())
            sequence = int(rtp_buffer.get_seq())
            rtp_pts_ns = None if buffer.pts == self.Gst.CLOCK_TIME_NONE else int(buffer.pts)
            self.register_rtp_sequence(
                ssrc=ssrc,
                sequence=sequence,
                rtp_pts_ns=rtp_pts_ns,
            )
            if rtp_buffer.get_marker():
                timing = RtpFrameTiming(
                    rtp_timestamp=int(rtp_buffer.get_timestamp()),
                    rtp_ssrc=ssrc,
                    rtp_marker_seq=sequence,
                    rtp_pts_s=(None if buffer.pts == self.Gst.CLOCK_TIME_NONE else int(buffer.pts) / 1e9),
                )
                with self._lock:
                    self._pending_rtp_frames.append(timing)
        finally:
            rtp_buffer.unmap()
        return self.Gst.PadProbeReturn.OK

    def _observe_access_unit(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None:
            return self.Gst.PadProbeReturn.OK
        with self._lock:
            self._h264_access_units_total += 1
        flags = buffer.get_flags()
        pts_ns = None if buffer.pts == self.Gst.CLOCK_TIME_NONE else int(buffer.pts)
        is_delta = bool(flags & self.Gst.BufferFlags.DELTA_UNIT)
        is_corrupted = bool(flags & self.Gst.BufferFlags.CORRUPTED)
        is_discontinuous = bool(flags & self.Gst.BufferFlags.DISCONT)
        recovery_basis = None
        nal1_metadata = None
        map_info = None
        if self.encoded_output_blocked() or is_discontinuous or not is_delta:
            try:
                mapped, map_info = buffer.map(self.Gst.MapFlags.READ)
                if mapped:
                    nal1_metadata = h264_nal1_non_idr_intra_metadata(
                        map_info.data,
                        pts_ns=pts_ns,
                        is_delta=is_delta,
                        is_corrupted=is_corrupted,
                    )
                    recovery_basis = h264_access_unit_recovery_basis(
                        map_info.data,
                        is_delta=is_delta,
                        allow_non_idr_intra_experimental=(self._allow_non_idr_intra_experimental),
                    )
            except Exception:
                # A boundary that cannot be inspected is not safe to recover on.
                recovery_basis = None
            finally:
                if map_info is not None:
                    try:
                        buffer.unmap(map_info)
                    except Exception:
                        pass
        if nal1_metadata is not None:
            self._remember_nal1_metadata(nal1_metadata)
        should_drop = self.should_drop_encoded_access_unit(
            pts_ns=pts_ns,
            is_delta=is_delta,
            is_corrupted=is_corrupted,
            is_discontinuous=is_discontinuous,
            is_random_access=bool(recovery_basis),
            recovery_basis=recovery_basis,
        )
        if should_drop:
            if self._allow_unverified_encoded_output:
                # Preview-only tolerant mode deliberately lets the decoder see
                # the same incomplete AUs that the initial StereoWIRIS path
                # exposed to its latest-frame consumer. The reader marks the
                # resulting BGR sample UNVERIFIED; recording remains strict.
                return self.Gst.PadProbeReturn.OK
            return self.Gst.PadProbeReturn.DROP
        if self.consume_encoded_recovery_marker(pts_ns):
            buffer.set_flags(self.Gst.BufferFlags.DISCONT)
        return self.Gst.PadProbeReturn.OK

    def _map_depay_buffer_pts(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None or buffer.pts == self.Gst.CLOCK_TIME_NONE:
            return self.Gst.PadProbeReturn.OK
        pts_ns = int(buffer.pts)
        pts_s = pts_ns / 1e9
        with self._lock:
            timing = self._pop_pending_rtp_timing_locked(pts_ns)
            if timing is None:
                return self.Gst.PadProbeReturn.OK
            timing = RtpFrameTiming(
                rtp_timestamp=timing.rtp_timestamp,
                rtp_ssrc=timing.rtp_ssrc,
                rtp_marker_seq=timing.rtp_marker_seq,
                rtp_pts_s=pts_s,
            )
            self._timing_by_pts_ns[pts_ns] = timing
            self._pts_order.append(pts_ns)
            self._trim_timing_cache_locked()
        return self.Gst.PadProbeReturn.OK

    def _pop_pending_rtp_timing_locked(self, pts_ns: int) -> RtpFrameTiming | None:
        """Match a depayloaded access unit to its RTP marker by PTS.

        A camera can emit marker packets that ``rtph264depay`` later merges or
        drops.  Consuming marker timings as a strict FIFO therefore leaves one
        extra entry behind whenever that happens; each extra entry makes every
        following frame appear one frame older and produces a steadily growing
        RTCP transit delay.  Packet and depay-buffer PTS are preserved by
        ``rtspsrc``, so discard unmatched older markers and take the closest
        matching PTS instead.
        """

        if not self._pending_rtp_frames:
            return None
        known_pts = [
            (index, int(round(float(timing.rtp_pts_s) * 1e9)))
            for index, timing in enumerate(self._pending_rtp_frames)
            if timing.rtp_pts_s is not None
        ]
        if not known_pts:
            # Keep compatibility with GStreamer builds that do not put a PTS
            # on RTP packet buffers.
            return self._pending_rtp_frames.popleft()

        match_index, match_pts_ns = min(
            known_pts,
            key=lambda item: abs(item[1] - int(pts_ns)),
        )
        if abs(match_pts_ns - int(pts_ns)) <= MAX_RTP_PTS_MATCH_DELTA_NS:
            for _ in range(match_index):
                self._pending_rtp_frames.popleft()
            return self._pending_rtp_frames.popleft()

        stale_before_ns = int(pts_ns) - MAX_RTP_PTS_MATCH_DELTA_NS
        while self._pending_rtp_frames:
            timing = self._pending_rtp_frames[0]
            if timing.rtp_pts_s is None:
                break
            timing_pts_ns = int(round(float(timing.rtp_pts_s) * 1e9))
            if timing_pts_ns >= stale_before_ns:
                break
            self._pending_rtp_frames.popleft()
        return None

    def _pop_frame_timing(self, pts_ns: int | None) -> RtpFrameTiming | None:
        with self._lock:
            if pts_ns is not None:
                timing = self._timing_by_pts_ns.pop(pts_ns, None)
                if timing is not None:
                    return timing
                timing = self._pop_nearest_timing_locked(pts_ns)
                if timing is not None:
                    return timing
                return None
            if self._timing_by_pts_ns:
                oldest_pts = min(self._timing_by_pts_ns)
                return self._timing_by_pts_ns.pop(oldest_pts, None)
            return None

    def _pop_nearest_timing_locked(self, pts_ns: int) -> RtpFrameTiming | None:
        if not self._timing_by_pts_ns:
            return None
        nearest_pts = min(self._timing_by_pts_ns, key=lambda candidate: abs(candidate - pts_ns))
        if abs(nearest_pts - pts_ns) > MAX_RTP_PTS_MATCH_DELTA_NS:
            return None
        return self._timing_by_pts_ns.pop(nearest_pts, None)

    def _trim_timing_cache_locked(self) -> None:
        while len(self._timing_by_pts_ns) > MAX_RTP_TIMING_CACHE and self._pts_order:
            old_pts = self._pts_order.popleft()
            self._timing_by_pts_ns.pop(old_pts, None)

    def _poll_rtcp_sender_reports(self, now_s: float) -> None:
        if now_s - self._last_rtcp_poll_s < 0.25:
            return
        self._last_rtcp_poll_s = now_s
        with self._lock:
            manager = self._manager
        if manager is None:
            return
        try:
            session = manager.emit("get-internal-session", 0)
            stats = session.get_property("stats")
            source_stats = stats.get_value("source-stats")
        except Exception:
            return
        if not isinstance(source_stats, list):
            return
        emitted_messages: list[str] = []
        for source in source_stats:
            if not bool(_gst_structure_value(source, "is-sender", False)):
                continue
            if not bool(_gst_structure_value(source, "have-sr", False)):
                continue
            ntp_raw = int(_gst_structure_value(source, "sr-ntptime", 0) or 0)
            sr_ntp_unix_s = ntp64_to_unix_seconds(ntp_raw)
            if sr_ntp_unix_s is None:
                continue
            ssrc = int(_gst_structure_value(source, "ssrc", 0) or 0)
            sr_rtp_timestamp = int(_gst_structure_value(source, "sr-rtptime", 0) or 0)
            clock_rate_hz = int(_gst_structure_value(source, "clock-rate", RTP_CLOCK_RATE_HZ) or RTP_CLOCK_RATE_HZ)
            timing = RtcpSenderReportTiming(
                ssrc=ssrc,
                ntp_raw=ntp_raw,
                ntp_unix_s=sr_ntp_unix_s,
                rtp_timestamp=sr_rtp_timestamp,
                clock_rate_hz=clock_rate_hz,
                received_monotonic_s=now_s,
                received_wall_s=time.time(),
            )
            arrival = self._sender_report_arrival(ssrc=ssrc, ntp_raw=ntp_raw)
            if arrival is not None:
                timing = RtcpSenderReportTiming(
                    ssrc=timing.ssrc,
                    ntp_raw=timing.ntp_raw,
                    ntp_unix_s=timing.ntp_unix_s,
                    rtp_timestamp=timing.rtp_timestamp,
                    clock_rate_hz=timing.clock_rate_hz,
                    received_monotonic_s=arrival[0],
                    received_wall_s=arrival[1],
                    arrival_basis="rtcp-pad-probe",
                )
            message = self._register_sender_report(timing)
            if message is not None:
                emitted_messages.append(message)
        for message in emitted_messages:
            self._emit_status(message)


# Retain legacy pickle names without importing the facade.
RtpFrameTiming.__module__ = "Workstation.workstation_core.yolo_stream"
RtpSequenceDiscontinuity.__module__ = "Workstation.workstation_core.yolo_stream"
RtpSequenceRecovery.__module__ = "Workstation.workstation_core.yolo_stream"
H264AccessUnitDiscontinuity.__module__ = "Workstation.workstation_core.yolo_stream"
RtpDiscontinuityReporter.__module__ = "Workstation.workstation_core.yolo_stream"
RtcpSenderReportTiming.__module__ = "Workstation.workstation_core.yolo_stream"
GstRtpTimingTracker.__module__ = "Workstation.workstation_core.yolo_stream"
