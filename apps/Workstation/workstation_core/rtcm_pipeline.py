"""UI-independent RTCM routing, integrity accounting, and evidence writes.

The Workstation receives RTCM from two producers: an NTRIP transport stream
and the local Base serial receiver.  This module deliberately has no Qt or
panel dependency so the full-rate data path can live outside the GUI thread.
Only snapshots produced by :class:`RtcmDataPlane` belong in the UI.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import time
from typing import Mapping

from Workstation.workstation_core.gnss_protocol import (
    Rtcm3StreamParser,
    parse_rtcm3_message_type,
)


RTCM_EVIDENCE_ACTIVE_STATES = frozenset(
    {"COLLECTING", "POST_CAPTURE", "VERIFYING", "RECOVERABLE"}
)
RTCM_PIPELINE_SNAPSHOT_SCHEMA = "workstation-rtcm-pipeline-snapshot-v1"
RTCM_EVIDENCE_SCHEMA = "workstation-rtcm-byte-record-v1"
RTCM_ALLOWED_TARGETS = frozenset({"left", "right", "drone"})


def _normalized_targets(values: tuple[str, ...] | list[str]) -> tuple[str, ...]:
    targets = tuple(dict.fromkeys(str(value).strip().lower() for value in values))
    invalid = tuple(value for value in targets if value not in RTCM_ALLOWED_TARGETS)
    if invalid:
        raise ValueError(f"invalid RTCM targets: {', '.join(invalid)}")
    return targets


@dataclass(frozen=True)
class RtcmRouteConfig:
    """Immutable routing snapshot supplied by the GUI/control plane."""

    ntrip_targets: tuple[str, ...] = ()
    base_targets: tuple[str, ...] = ()
    forward_ntrip_to_base: bool = False
    base_serial_available: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "ntrip_targets", _normalized_targets(self.ntrip_targets))
        object.__setattr__(self, "base_targets", _normalized_targets(self.base_targets))

    @classmethod
    def from_payload(cls, payload: object) -> "RtcmRouteConfig":
        if isinstance(payload, cls):
            return payload
        values = dict(payload) if isinstance(payload, Mapping) else {}
        return cls(
            ntrip_targets=tuple(values.get("ntrip_targets") or ()),
            base_targets=tuple(values.get("base_targets") or ()),
            forward_ntrip_to_base=bool(values.get("forward_ntrip_to_base", False)),
            base_serial_available=bool(values.get("base_serial_available", False)),
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "ntrip_targets": self.ntrip_targets,
            "base_targets": self.base_targets,
            "forward_ntrip_to_base": self.forward_ntrip_to_base,
            "base_serial_available": self.base_serial_available,
        }


@dataclass(frozen=True)
class RtcmEvidenceContext:
    """Session-owned destination snapshot; disabled contexts never write."""

    state: str = "IDLE"
    session_id: str = ""
    session_root: Path | None = None
    context_token: str = ""

    def __post_init__(self) -> None:
        state = str(self.state or "IDLE").strip().upper()
        session_id = str(self.session_id or "").strip()
        root = None if self.session_root is None else Path(self.session_root)
        context_token = str(self.context_token or "").strip()
        if not context_token and session_id and root is not None:
            identity = f"{session_id}\0{root}".encode("utf-8", "surrogatepass")
            context_token = "session-" + hashlib.sha256(identity).hexdigest()[:24]
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "session_id", session_id)
        object.__setattr__(self, "session_root", root)
        object.__setattr__(self, "context_token", context_token)

    @property
    def active(self) -> bool:
        return bool(
            self.state in RTCM_EVIDENCE_ACTIVE_STATES
            and self.session_id
            and self.session_root is not None
        )

    @classmethod
    def from_payload(cls, payload: object) -> "RtcmEvidenceContext":
        if isinstance(payload, cls):
            return payload
        values = dict(payload) if isinstance(payload, Mapping) else {}
        return cls(
            state=str(values.get("state") or "IDLE"),
            session_id=str(values.get("session_id") or ""),
            session_root=(
                None
                if not values.get("session_root")
                else Path(str(values["session_root"]))
            ),
            context_token=str(values.get("context_token") or ""),
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "state": self.state,
            "session_id": self.session_id,
            "session_root": None if self.session_root is None else str(self.session_root),
            "context_token": self.context_token,
            "active": self.active,
        }


@dataclass(frozen=True)
class RtcmEvidenceRecord:
    sequence: int
    source_sequence: int
    source: str
    record_kind: str
    payload: bytes
    receive_utc_ns: int
    receive_monotonic_ns: int
    message_type: int | None = None

    def __post_init__(self) -> None:
        sequence = int(self.sequence)
        source_sequence = int(self.source_sequence)
        payload = bytes(self.payload)
        if sequence <= 0 or source_sequence <= 0:
            raise ValueError("RTCM evidence sequences must be positive")
        if not payload:
            raise ValueError("RTCM evidence payload cannot be empty")
        object.__setattr__(self, "sequence", sequence)
        object.__setattr__(self, "source_sequence", source_sequence)
        object.__setattr__(self, "source", str(self.source).strip() or "unknown")
        object.__setattr__(self, "record_kind", str(self.record_kind).strip() or "bytes")
        object.__setattr__(self, "payload", payload)
        object.__setattr__(self, "receive_utc_ns", int(self.receive_utc_ns))
        object.__setattr__(self, "receive_monotonic_ns", int(self.receive_monotonic_ns))

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.payload).hexdigest()

    def envelope(self) -> dict[str, object]:
        return {
            "sequence": self.sequence,
            "source_sequence": self.source_sequence,
            "source": self.source,
            "record_kind": self.record_kind,
            "payload": self.payload,
            "payload_bytes": len(self.payload),
            "sha256": self.sha256,
            "receive_utc_ns": self.receive_utc_ns,
            "receive_monotonic_ns": self.receive_monotonic_ns,
            "message_type": self.message_type,
        }


@dataclass(frozen=True)
class RtcmInjectionRequest:
    sequence: int
    source: str
    targets: tuple[str, ...]
    payload: bytes
    receive_monotonic_ns: int
    message_type: int | None

    def as_dict(self) -> dict[str, object]:
        return {
            "pipeline_sequence": int(self.sequence),
            "source": str(self.source),
            "targets": tuple(self.targets),
            "payload": bytes(self.payload),
            "payload_bytes": len(self.payload),
            "payload_sha256": hashlib.sha256(self.payload).hexdigest(),
            "receive_monotonic_ns": int(self.receive_monotonic_ns),
            "message_type": self.message_type,
        }


@dataclass(frozen=True)
class RtcmProcessResult:
    evidence_records: tuple[RtcmEvidenceRecord, ...] = ()
    injection_requests: tuple[RtcmInjectionRequest, ...] = ()
    base_writes: tuple[tuple[str, bytes], ...] = ()
    base_reference_seen: bool = False
    base_forward_unavailable: bool = False


class RtcmDataPlane:
    """Single-owner RTCM parser, router, and integrity high-water ledger."""

    def __init__(self, *, started_monotonic_s: float | None = None) -> None:
        self.route = RtcmRouteConfig()
        self.parser = Rtcm3StreamParser()
        self.started_monotonic_s = (
            time.monotonic() if started_monotonic_s is None else float(started_monotonic_s)
        )
        self._sequence = 0
        self._source_sequences: Counter[tuple[str, str]] = Counter()
        self._transport_chunks = 0
        self._transport_bytes = 0
        self._frames = 0
        self._frame_bytes = 0
        self._max_frame_bytes = 0
        self._last_frame_monotonic_ns: int | None = None
        self._message_types: Counter[int] = Counter()
        self._injection = {
            side: {"frames": 0, "bytes": 0, "packets": 0, "fail": 0}
            for side in ("left", "right", "drone")
        }
        self._injection_dropped_stale = 0
        self._last_injection_drop: dict[str, object] | None = None
        self._injection_expected = 0
        self._injection_outcomes = 0
        self._injection_unconfirmed = 0
        self._pending_injection_sequences: set[int] = set()
        self._last_injected_sequence = 0
        self._last_received_sequence = 0
        self._last_persisted_sequence = 0
        self._records_received = 0
        self._evidence_expected = 0
        self._evidence_persisted = 0
        self._evidence_failed = 0
        self._last_evidence_error = ""
        self._base_forward_unavailable = 0

    def configure(self, config: RtcmRouteConfig | Mapping[str, object]) -> None:
        self.route = RtcmRouteConfig.from_payload(config)

    def reset_runtime(
        self,
        config: RtcmRouteConfig | Mapping[str, object],
        *,
        started_monotonic_s: float | None = None,
    ) -> None:
        """Reset connection-local parsing/stats without reusing a sequence.

        Sequence and evidence high-water values deliberately survive NTRIP or
        Base reconnects so one session never receives duplicate identities.
        """

        self.configure(config)
        self.parser = Rtcm3StreamParser()
        self.started_monotonic_s = (
            time.monotonic() if started_monotonic_s is None else float(started_monotonic_s)
        )
        self._transport_chunks = 0
        self._transport_bytes = 0
        self._frames = 0
        self._frame_bytes = 0
        self._max_frame_bytes = 0
        self._last_frame_monotonic_ns = None
        self._message_types.clear()
        self._injection = {
            side: {"frames": 0, "bytes": 0, "packets": 0, "fail": 0}
            for side in ("left", "right", "drone")
        }
        self._injection_dropped_stale = 0
        self._last_injection_drop = None
        self._base_forward_unavailable = 0

    def restore_evidence_high_water(
        self,
        *,
        last_sequence: int,
        source_sequences: Mapping[tuple[str, str], int],
    ) -> None:
        self._sequence = max(self._sequence, int(last_sequence))
        for key, value in source_sequences.items():
            source, record_kind = key
            normalized = (str(source), str(record_kind))
            self._source_sequences[normalized] = max(
                self._source_sequences[normalized],
                int(value),
            )
        self._last_persisted_sequence = max(
            self._last_persisted_sequence,
            int(last_sequence),
        )

    def _record(
        self,
        *,
        source: str,
        record_kind: str,
        payload: bytes,
        receive_utc_ns: int,
        receive_monotonic_ns: int,
        message_type: int | None = None,
    ) -> RtcmEvidenceRecord:
        self._sequence += 1
        key = (str(source), str(record_kind))
        self._source_sequences[key] += 1
        record = RtcmEvidenceRecord(
            sequence=self._sequence,
            source_sequence=self._source_sequences[key],
            source=source,
            record_kind=record_kind,
            payload=payload,
            receive_utc_ns=receive_utc_ns,
            receive_monotonic_ns=receive_monotonic_ns,
            message_type=message_type,
        )
        self._last_received_sequence = record.sequence
        self._records_received += 1
        return record

    def _process_frame(
        self,
        payload: bytes,
        *,
        source: str,
        targets: tuple[str, ...],
        receive_utc_ns: int,
        receive_monotonic_ns: int,
    ) -> tuple[RtcmEvidenceRecord, RtcmInjectionRequest | None, bool]:
        frame = bytes(payload)
        message_type = parse_rtcm3_message_type(frame)
        record = self._record(
            source=source,
            record_kind="rtcm_frame",
            payload=frame,
            receive_utc_ns=receive_utc_ns,
            receive_monotonic_ns=receive_monotonic_ns,
            message_type=message_type,
        )
        self._frames += 1
        self._frame_bytes += len(frame)
        self._max_frame_bytes = max(self._max_frame_bytes, len(frame))
        self._last_frame_monotonic_ns = int(receive_monotonic_ns)
        if message_type is not None:
            self._message_types[int(message_type)] += 1
        request = None
        if targets:
            request = RtcmInjectionRequest(
                sequence=record.sequence,
                source=source,
                targets=targets,
                payload=frame,
                receive_monotonic_ns=receive_monotonic_ns,
                message_type=message_type,
            )
        return record, request, message_type in {1005, 1006}

    def process_ntrip_chunk(
        self,
        payload: bytes,
        *,
        receive_utc_ns: int | None = None,
        receive_monotonic_ns: int | None = None,
    ) -> RtcmProcessResult:
        chunk = bytes(payload)
        if not chunk:
            return RtcmProcessResult()
        utc_ns = time.time_ns() if receive_utc_ns is None else int(receive_utc_ns)
        monotonic_ns = (
            time.monotonic_ns()
            if receive_monotonic_ns is None
            else int(receive_monotonic_ns)
        )
        records: list[RtcmEvidenceRecord] = [
            self._record(
                source="ntrip",
                record_kind="transport_chunk",
                payload=chunk,
                receive_utc_ns=utc_ns,
                receive_monotonic_ns=monotonic_ns,
            )
        ]
        self._transport_chunks += 1
        self._transport_bytes += len(chunk)
        requests: list[RtcmInjectionRequest] = []
        base_reference_seen = False
        for frame in self.parser.feed(chunk):
            record, request, reference_seen = self._process_frame(
                frame,
                source="ntrip",
                targets=self.route.ntrip_targets,
                receive_utc_ns=utc_ns,
                receive_monotonic_ns=monotonic_ns,
            )
            records.append(record)
            if request is not None:
                requests.append(request)
            base_reference_seen = base_reference_seen or reference_seen
        base_writes: tuple[tuple[str, bytes], ...] = ()
        unavailable = False
        if self.route.forward_ntrip_to_base:
            if self.route.base_serial_available:
                base_writes = (("NTRIP RTCM -> fixed base", chunk),)
            else:
                unavailable = True
                self._base_forward_unavailable += 1
        return RtcmProcessResult(
            evidence_records=tuple(records),
            injection_requests=tuple(requests),
            base_writes=base_writes,
            base_reference_seen=base_reference_seen,
            base_forward_unavailable=unavailable,
        )

    def process_base_frame(
        self,
        payload: bytes,
        *,
        receive_utc_ns: int | None = None,
        receive_monotonic_ns: int | None = None,
    ) -> RtcmProcessResult:
        frame = bytes(payload)
        if not frame:
            return RtcmProcessResult()
        utc_ns = time.time_ns() if receive_utc_ns is None else int(receive_utc_ns)
        monotonic_ns = (
            time.monotonic_ns()
            if receive_monotonic_ns is None
            else int(receive_monotonic_ns)
        )
        record, request, reference_seen = self._process_frame(
            frame,
            source="base",
            targets=self.route.base_targets,
            receive_utc_ns=utc_ns,
            receive_monotonic_ns=monotonic_ns,
        )
        return RtcmProcessResult(
            evidence_records=(record,),
            injection_requests=() if request is None else (request,),
            base_reference_seen=reference_seen,
        )

    def record_injection_result(self, result: Mapping[str, object]) -> None:
        payload_bytes = int(result.get("payload_bytes") or 0)
        pipeline_sequence = int(result.get("pipeline_sequence") or 0)
        for side, packet_count in tuple(result.get("sent") or ()):
            key = str(side)
            if key not in self._injection:
                continue
            self._injection[key]["frames"] += 1
            self._injection[key]["bytes"] += payload_bytes
            self._injection[key]["packets"] += int(packet_count or 0)
            delivery_by_target = result.get("delivery_monotonic_s_by_target")
            delivery_s = (
                delivery_by_target.get(key)
                if isinstance(delivery_by_target, Mapping)
                else result.get("last_delivery_s")
            )
            if delivery_s is not None:
                self._injection[key]["last_delivery_s"] = float(delivery_s)
        for side, _error in tuple(result.get("failed") or ()):
            key = str(side)
            if key in self._injection:
                self._injection[key]["fail"] += 1
        dropped = int(result.get("dropped") or 0)
        self._injection_dropped_stale += dropped
        if dropped and str(result.get("drop_reason") or "").startswith("stale-"):
            self._last_injection_drop = {
                "pipeline_sequence": pipeline_sequence,
                "source": str(result.get("source") or ""),
                "payload_bytes": payload_bytes,
                "payload_sha256": str(result.get("payload_sha256") or ""),
                "age_s": max(0.0, float(result.get("age_s") or 0.0)),
                "reason": str(result.get("drop_reason") or "stale-before-send"),
                "dropped_targets": tuple(result.get("dropped_targets") or ()),
            }
        completed_sequences: set[int] = set()
        if pipeline_sequence > 0:
            completed_sequences.add(pipeline_sequence)
        for envelope in tuple(result.get("dropped_envelopes") or ()):
            if isinstance(envelope, Mapping):
                sequence = int(envelope.get("pipeline_sequence") or 0)
                if sequence > 0:
                    completed_sequences.add(sequence)
        completed = completed_sequences & self._pending_injection_sequences
        self._pending_injection_sequences.difference_update(completed)
        self._injection_outcomes += len(completed)
        if pipeline_sequence > 0 and bool(result.get("sent")):
            self._last_injected_sequence = max(self._last_injected_sequence, pipeline_sequence)

    def record_injection_expected(self, request: RtcmInjectionRequest) -> None:
        if request.sequence in self._pending_injection_sequences:
            raise ValueError(f"duplicate pending RTCM injection sequence: {request.sequence}")
        self._pending_injection_sequences.add(request.sequence)
        self._injection_expected += 1

    @property
    def pending_injection_count(self) -> int:
        return len(self._pending_injection_sequences)

    @property
    def pending_injection_sequences(self) -> tuple[int, ...]:
        return tuple(sorted(self._pending_injection_sequences))

    def mark_injections_unconfirmed(
        self,
        sequences: object | None = None,
    ) -> tuple[int, ...]:
        if sequences is None:
            selected = set(self._pending_injection_sequences)
        else:
            selected = {
                int(value)
                for value in tuple(sequences or ())
                if int(value) > 0
            }
            selected.intersection_update(self._pending_injection_sequences)
        ordered = tuple(sorted(selected))
        self._pending_injection_sequences.difference_update(selected)
        self._injection_unconfirmed += len(ordered)
        return ordered

    def mark_pending_injections_unconfirmed(self) -> tuple[int, ...]:
        return self.mark_injections_unconfirmed()

    def record_stale_injection_drop(
        self,
        request: RtcmInjectionRequest,
        *,
        age_s: float,
    ) -> None:
        self._injection_dropped_stale += 1
        self._last_injection_drop = {
            "pipeline_sequence": request.sequence,
            "source": request.source,
            "targets": request.targets,
            "payload_bytes": len(request.payload),
            "payload_sha256": hashlib.sha256(request.payload).hexdigest(),
            "age_s": max(0.0, float(age_s)),
            "reason": "stale-before-injection",
        }

    def record_evidence_expected(self, record: RtcmEvidenceRecord) -> None:
        self._evidence_expected += 1

    def record_evidence_written(self, result: Mapping[str, object]) -> None:
        sequence = int(result.get("sequence") or 0)
        self._evidence_persisted += 1
        self._last_persisted_sequence = max(self._last_persisted_sequence, sequence)

    def record_evidence_failure(self, *, sequence: int, error: str) -> None:
        self._evidence_failed += 1
        self._last_evidence_error = f"sequence={int(sequence)}: {error}"

    def snapshot(self, *, now_monotonic_s: float | None = None) -> dict[str, object]:
        now_s = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        last_frame_s = (
            None
            if self._last_frame_monotonic_ns is None
            else self._last_frame_monotonic_ns / 1_000_000_000.0
        )
        return {
            "schema": RTCM_PIPELINE_SNAPSHOT_SCHEMA,
            "start_s": self.started_monotonic_s,
            "last_frame_s": last_frame_s,
            "last_frame_age_s": (
                None if last_frame_s is None else max(0.0, now_s - last_frame_s)
            ),
            "frames": self._frames,
            "bytes": self._frame_bytes,
            "max_frame_bytes": self._max_frame_bytes,
            "message_types": dict(sorted(self._message_types.items())),
            "transport": {
                "chunks": self._transport_chunks,
                "bytes": self._transport_bytes,
                "base_forward_unavailable": self._base_forward_unavailable,
            },
            "injection": {
                side: dict(values) for side, values in self._injection.items()
            },
            "integrity": {
                "last_received_sequence": self._last_received_sequence,
                "last_persisted_sequence": self._last_persisted_sequence,
                "last_injected_sequence": self._last_injected_sequence,
                "records_received": self._records_received,
                "evidence_expected": self._evidence_expected,
                "evidence_persisted": self._evidence_persisted,
                "evidence_failed": self._evidence_failed,
                "injection_dropped_stale": self._injection_dropped_stale,
                "last_injection_drop": self._last_injection_drop,
                "injection_expected": self._injection_expected,
                "injection_outcomes": self._injection_outcomes,
                "injection_pending": self.pending_injection_count,
                "pending_injection_sequences": tuple(
                    sorted(self._pending_injection_sequences)[:64]
                ),
                "injection_unconfirmed": self._injection_unconfirmed,
                "last_evidence_error": self._last_evidence_error,
            },
            "route": self.route.as_dict(),
        }


def _safe_component(value: str, fallback: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "-", str(value)).strip(".-") or fallback


def _write_all_at(descriptor: int, payload: bytes, offset: int) -> None:
    """Write every byte at an explicit offset, including short-write retries."""

    view = memoryview(payload)
    written = 0
    while written < len(view):
        count = os.pwrite(descriptor, view[written:], int(offset) + written)
        if count <= 0:
            raise OSError(
                f"short RTCM evidence write: expected={len(view)}, actual={written}"
            )
        written += int(count)


def _read_last_jsonl_row(descriptor: int, size: int) -> dict[str, object]:
    if size <= 0:
        raise ValueError("RTCM evidence index is empty")
    if os.pread(descriptor, 1, size - 1) != b"\n":
        raise ValueError("RTCM evidence index does not end with a newline")
    end = size - 1
    position = end
    chunks: list[bytes] = []
    while position > 0:
        start = max(0, position - 4096)
        chunk = os.pread(descriptor, position - start, start)
        separator = chunk.rfind(b"\n")
        if separator >= 0:
            chunks.append(chunk[separator + 1 :])
            break
        chunks.append(chunk)
        position = start
    encoded = b"".join(reversed(chunks))
    try:
        row = json.loads(encoded.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError(f"malformed final RTCM evidence index row: {exc}") from exc
    if not isinstance(row, dict):
        raise ValueError("malformed final RTCM evidence index row: row is not an object")
    return row


def _validate_evidence_append_boundary(
    binary_descriptor: int,
    index_descriptor: int,
    *,
    context: RtcmEvidenceContext,
    record: RtcmEvidenceRecord,
) -> tuple[int, int]:
    """Validate the existing pair and return immutable pre-write offsets."""

    binary_offset = int(os.fstat(binary_descriptor).st_size)
    index_offset = int(os.fstat(index_descriptor).st_size)
    if binary_offset == 0 and index_offset == 0:
        return binary_offset, index_offset
    if binary_offset == 0 or index_offset == 0:
        raise ValueError("RTCM evidence binary/index pair is incomplete")
    previous = _read_last_jsonl_row(index_descriptor, index_offset)
    try:
        previous_offset = int(previous["offset_bytes"])
        previous_size = int(previous["size_bytes"])
        previous_sha256 = str(previous["sha256"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("incomplete final RTCM evidence index row") from exc
    if previous_offset < 0 or previous_size <= 0:
        raise ValueError("invalid final RTCM evidence offset or size")
    if previous_offset + previous_size != binary_offset:
        raise ValueError(
            "RTCM evidence binary contains unindexed trailing bytes or a short indexed payload"
        )
    previous_payload = os.pread(binary_descriptor, previous_size, previous_offset)
    if len(previous_payload) != previous_size:
        raise ValueError("short final RTCM evidence payload")
    if hashlib.sha256(previous_payload).hexdigest() != previous_sha256:
        raise ValueError("final RTCM evidence payload hash mismatch")
    if str(previous.get("schema") or "") != RTCM_EVIDENCE_SCHEMA:
        raise ValueError("unexpected final RTCM evidence schema")
    if str(previous.get("session_id") or "") != context.session_id:
        raise ValueError("RTCM evidence session identity changed within one file pair")
    if str(previous.get("source") or "") != record.source or str(
        previous.get("record_kind") or ""
    ) != record.record_kind:
        raise ValueError("RTCM evidence source identity does not match its file pair")
    previous_sequence = int(previous.get("sequence") or 0)
    previous_source_sequence = int(previous.get("source_sequence") or 0)
    if record.sequence <= previous_sequence or record.source_sequence <= previous_source_sequence:
        raise ValueError("RTCM evidence sequence did not advance")
    return binary_offset, index_offset


def _rollback_evidence_pair(
    binary_descriptor: int,
    index_descriptor: int,
    *,
    binary_offset: int,
    index_offset: int,
) -> None:
    """Restore the exact pre-append pair; index always rolls back first."""

    errors: list[str] = []
    for label, descriptor, offset in (
        ("index", index_descriptor, index_offset),
        ("binary", binary_descriptor, binary_offset),
    ):
        try:
            os.ftruncate(descriptor, offset)
            os.fsync(descriptor)
            actual = int(os.fstat(descriptor).st_size)
            if actual != offset:
                raise OSError(f"expected={offset}, actual={actual}")
        except Exception as exc:
            errors.append(f"{label}: {exc}")
    if errors:
        raise OSError("RTCM evidence rollback failed: " + "; ".join(errors))


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def append_rtcm_evidence(
    context: RtcmEvidenceContext,
    record: RtcmEvidenceRecord,
) -> dict[str, object] | None:
    """Append one exact payload and index row, returning its durable identity.

    The caller must serialize calls for a given context.  Runtime ownership is
    the RTCM data-plane worker; direct calls remain useful for recovery tests.
    """

    if not context.active or context.session_root is None:
        return None
    safe_source = _safe_component(record.source, "unknown")
    safe_kind = _safe_component(record.record_kind, "bytes")
    root = context.session_root / "raw" / "gnss"
    binary_path = root / f"{safe_source}_{safe_kind}.rtcm"
    index_path = binary_path.with_suffix(binary_path.suffix + ".jsonl")
    root.mkdir(parents=True, exist_ok=True)
    binary_existed = binary_path.exists()
    index_existed = index_path.exists()
    if binary_existed != index_existed:
        raise ValueError("RTCM evidence binary/index pair is incomplete")
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_CLOEXEC", 0)
    binary_descriptor = os.open(binary_path, flags, 0o644)
    try:
        index_descriptor = os.open(index_path, flags, 0o644)
    except Exception:
        os.close(binary_descriptor)
        if not binary_existed:
            try:
                binary_path.unlink()
            except OSError:
                pass
        raise
    binary_offset = 0
    index_offset = 0
    try:
        binary_offset, index_offset = _validate_evidence_append_boundary(
            binary_descriptor,
            index_descriptor,
            context=context,
            record=record,
        )
        row = {
            "schema": RTCM_EVIDENCE_SCHEMA,
            "session_id": context.session_id,
            "context_token": context.context_token,
            "sequence": record.sequence,
            "source_sequence": record.source_sequence,
            "source": record.source,
            "record_kind": record.record_kind,
            "offset_bytes": binary_offset,
            "size_bytes": len(record.payload),
            "sha256": record.sha256,
            "receive_utc_ns": record.receive_utc_ns,
            "receive_monotonic_ns": record.receive_monotonic_ns,
            "message_type": record.message_type,
        }
        encoded = (
            json.dumps(
                row,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")
        try:
            _write_all_at(binary_descriptor, record.payload, binary_offset)
            os.fsync(binary_descriptor)
            binary_size = int(os.fstat(binary_descriptor).st_size)
            persisted_payload = os.pread(
                binary_descriptor,
                len(record.payload),
                binary_offset,
            )
            if binary_size != binary_offset + len(record.payload):
                raise OSError(
                    f"RTCM evidence binary size mismatch: expected="
                    f"{binary_offset + len(record.payload)}, actual={binary_size}"
                )
            if persisted_payload != record.payload or hashlib.sha256(
                persisted_payload
            ).hexdigest() != record.sha256:
                raise OSError("RTCM evidence binary readback mismatch")

            _write_all_at(index_descriptor, encoded, index_offset)
            os.fsync(index_descriptor)
            index_size = int(os.fstat(index_descriptor).st_size)
            persisted_index = os.pread(index_descriptor, len(encoded), index_offset)
            if index_size != index_offset + len(encoded) or persisted_index != encoded:
                raise OSError("RTCM evidence index readback mismatch")
            if json.loads(persisted_index.decode("utf-8")) != row:
                raise OSError("RTCM evidence index semantic readback mismatch")
            if not binary_existed:
                _fsync_directory(root)
        except Exception as exc:
            try:
                _rollback_evidence_pair(
                    binary_descriptor,
                    index_descriptor,
                    binary_offset=binary_offset,
                    index_offset=index_offset,
                )
            except Exception as rollback_exc:
                raise OSError(f"{exc}; {rollback_exc}") from exc
            raise
    finally:
        os.close(index_descriptor)
        os.close(binary_descriptor)
    return {
        **row,
        "binary_path": str(binary_path),
        "index_path": str(index_path),
    }


def load_rtcm_evidence_high_water(
    context: RtcmEvidenceContext,
) -> tuple[int, dict[tuple[str, str], int]]:
    """Recover sequence floors from existing append-only RTCM indexes.

    Legacy rows written before pipeline sequences existed are counted in stable
    path/line order.  Malformed rows fail closed rather than allowing a new
    writer to append identities to an index it cannot audit.
    """

    if not context.active or context.session_root is None:
        return 0, {}
    root = context.session_root / "raw" / "gnss"
    if not root.is_dir():
        return 0, {}
    for binary_path in sorted(root.glob("*.rtcm")):
        index_path = binary_path.with_suffix(binary_path.suffix + ".jsonl")
        if not index_path.is_file():
            raise ValueError(f"RTCM evidence binary has no index: {binary_path}")
    explicit_sequences: set[int] = set()
    last_sequence = 0
    row_count = 0
    source_sequences: Counter[tuple[str, str]] = Counter()
    for index_path in sorted(root.glob("*.rtcm.jsonl")):
        binary_path = index_path.with_suffix("")
        if not binary_path.is_file():
            raise ValueError(f"RTCM evidence index has no binary: {index_path}")
        try:
            lines = index_path.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            raise OSError(f"cannot read RTCM evidence index {index_path}: {exc}") from exc
        expected_offset = 0
        with binary_path.open("rb") as binary_handle:
            for line_number, line in enumerate(lines, start=1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"malformed RTCM evidence index {index_path}:{line_number}: {exc}"
                    ) from exc
                if not isinstance(row, dict):
                    raise ValueError(
                        f"malformed RTCM evidence index {index_path}:{line_number}: row is not an object"
                    )
                if str(row.get("schema") or "") != RTCM_EVIDENCE_SCHEMA:
                    raise ValueError(
                        f"unexpected RTCM evidence schema at {index_path}:{line_number}"
                    )
                if str(row.get("session_id") or "") != context.session_id:
                    raise ValueError(
                        f"RTCM evidence session mismatch at {index_path}:{line_number}"
                    )
                try:
                    offset = int(row["offset_bytes"])
                    size = int(row["size_bytes"])
                    expected_sha256 = str(row["sha256"])
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(
                        f"incomplete RTCM evidence index {index_path}:{line_number}"
                    ) from exc
                if offset != expected_offset or size <= 0:
                    raise ValueError(
                        f"non-contiguous RTCM evidence index {index_path}:{line_number}: "
                        f"expected_offset={expected_offset}, offset={offset}, size={size}"
                    )
                payload = binary_handle.read(size)
                if len(payload) != size:
                    raise ValueError(
                        f"short RTCM evidence binary at {index_path}:{line_number}: "
                        f"expected={size}, actual={len(payload)}"
                    )
                actual_sha256 = hashlib.sha256(payload).hexdigest()
                if actual_sha256 != expected_sha256:
                    raise ValueError(
                        f"RTCM evidence hash mismatch at {index_path}:{line_number}"
                    )
                expected_offset += size
                row_count += 1
                source = str(row.get("source") or "unknown")
                record_kind = str(row.get("record_kind") or "bytes")
                expected_name = (
                    f"{_safe_component(source, 'unknown')}_"
                    f"{_safe_component(record_kind, 'bytes')}.rtcm"
                )
                if binary_path.name != expected_name:
                    raise ValueError(
                        f"RTCM evidence source/path mismatch at {index_path}:{line_number}"
                    )
                key = (source, record_kind)
                source_sequence = row.get("source_sequence")
                if source_sequence is None:
                    source_sequences[key] += 1
                else:
                    value = int(source_sequence)
                    if value <= 0:
                        raise ValueError(
                            f"invalid RTCM source_sequence at {index_path}:{line_number}"
                        )
                    source_sequences[key] = max(source_sequences[key], value)
                sequence = row.get("sequence")
                if sequence is None:
                    continue
                value = int(sequence)
                if value <= 0 or value in explicit_sequences:
                    raise ValueError(
                        f"invalid or duplicate RTCM sequence {value} at {index_path}:{line_number}"
                    )
                explicit_sequences.add(value)
                last_sequence = max(last_sequence, value)
            trailing = binary_handle.read(1)
            if trailing:
                raise ValueError(f"RTCM evidence binary contains unindexed trailing bytes: {binary_path}")
    return max(last_sequence, row_count), dict(source_sequences)


__all__ = [
    "RTCM_EVIDENCE_ACTIVE_STATES",
    "RTCM_EVIDENCE_SCHEMA",
    "RTCM_PIPELINE_SNAPSHOT_SCHEMA",
    "RtcmDataPlane",
    "RtcmEvidenceContext",
    "RtcmEvidenceRecord",
    "RtcmInjectionRequest",
    "RtcmProcessResult",
    "RtcmRouteConfig",
    "append_rtcm_evidence",
    "load_rtcm_evidence_high_water",
]
