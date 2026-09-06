"""Workstation GNSS base serial, NTRIP, and discovery workers."""

from __future__ import annotations

from dataclasses import dataclass
import errno as errno_module
import logging
import queue
import socket
import threading
import time

from PyQt5 import QtCore

_LOGGER = logging.getLogger(__name__)

from Workstation.workstation_core.gnss_network import NtripClient
from Workstation.workstation_core.gnss_operation import HeightType, NetworkConfig
from Workstation.workstation_core.gnss_protocol import (
    Rtcm3StreamParser,
    UbxStreamParser,
    gnss_base_serial_port_candidates,
    parse_nmea_gga_satellite_count,
    parse_ubx_base_satellite_count,
    parse_ubx_nav_pvt_coordinate_sample,
    parse_ubx_nav_sat_quality,
    parse_ubx_nav_svin,
    probe_gnss_base_serial_ports,
    ubx_nav_pvt_poll_frame,
    ubx_nav_svin_poll_frame,
)
from Workstation.workstation_core.rtcm_pipeline import (
    RtcmDataPlane,
    RtcmEvidenceContext,
    RtcmRouteConfig,
    append_rtcm_evidence,
    load_rtcm_evidence_high_water,
)


@dataclass
class _EvidenceDrainBarrier:
    context: RtcmEvidenceContext
    pending_injection_sequences: set[int]
    deadline_s: float


class RtcmDataPlaneWorker(QtCore.QObject):
    """Own full-rate RTCM parsing, routing, and evidence outside the GUI.

    NTRIP and Base-serial producers submit timestamped envelopes directly to a
    bounded thread-safe ingress queue. Parsing, routing, and evidence writes
    run on this object's QThread; only ``snapshot_ready`` crosses back to the
    panel, at a bounded cadence.
    """

    SNAPSHOT_INTERVAL_S = 0.5
    MAX_INGRESS_QUEUE = 4096
    INGRESS_BATCH_SIZE = 64
    MAX_INJECTION_AGE_S = 2.0
    STOP_DRAIN_TIMEOUT_S = 3.0
    EVIDENCE_DRAIN_TIMEOUT_S = 3.0

    configure_requested = QtCore.pyqtSignal(object)
    reset_requested = QtCore.pyqtSignal(object)
    evidence_context_requested = QtCore.pyqtSignal(object)
    ntrip_stats_received = QtCore.pyqtSignal(object)
    ntrip_chunk_received = QtCore.pyqtSignal(object)
    base_frame_received = QtCore.pyqtSignal(object)
    injection_result_received = QtCore.pyqtSignal(object)
    evidence_drain_requested = QtCore.pyqtSignal(object)
    stop_requested = QtCore.pyqtSignal()

    base_write_requested = QtCore.pyqtSignal(str, object)
    injection_requested = QtCore.pyqtSignal(object)
    snapshot_ready = QtCore.pyqtSignal(object)
    evidence_started = QtCore.pyqtSignal(object)
    evidence_drained = QtCore.pyqtSignal(object)
    integrity_failed = QtCore.pyqtSignal(object)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        snapshot_interval_s: float = SNAPSHOT_INTERVAL_S,
        max_ingress_queue: int = MAX_INGRESS_QUEUE,
        max_injection_age_s: float = MAX_INJECTION_AGE_S,
        stop_drain_timeout_s: float = STOP_DRAIN_TIMEOUT_S,
        evidence_drain_timeout_s: float = EVIDENCE_DRAIN_TIMEOUT_S,
    ) -> None:
        super().__init__()
        self.snapshot_interval_s = max(0.05, float(snapshot_interval_s))
        self._plane = RtcmDataPlane()
        self._context = RtcmEvidenceContext()
        self._requested_context = self._context
        self._context_lock = threading.Lock()
        self._ingress_submission_lock = threading.Lock()
        self._validated_context_tokens: dict[str, bool] = {}
        self._closing_evidence_contexts: dict[str, RtcmEvidenceContext] = {}
        self._evidence_errors_by_context: dict[str, list[str]] = {}
        self._pending_evidence_barriers: list[_EvidenceDrainBarrier] = []
        self._evidence_barrier_poll_scheduled = False
        self._injection_context_by_sequence: dict[int, str] = {}
        self._latest_ntrip_stats: object | None = None
        self._latest_injection_result: dict[str, object] | None = None
        self._last_snapshot_s = float("-inf")
        self._base_reference_seen_since_snapshot = False
        self._evidence_started_key: tuple[str, str] | None = None
        self._accepting = True
        self._finished = False
        self._stop_after_drain = False
        self._ingress: queue.Queue[tuple[str, dict[str, object]]] = queue.Queue(
            maxsize=max(1, int(max_ingress_queue))
        )
        self._ingress_lock = threading.Lock()
        self._drain_scheduled = False
        self._backpressure_waits = 0
        self.max_injection_age_s = max(0.0, float(max_injection_age_s))
        self.stop_drain_timeout_s = max(0.05, float(stop_drain_timeout_s))
        self.evidence_drain_timeout_s = max(0.05, float(evidence_drain_timeout_s))
        self._stop_deadline_s: float | None = None

        queued = QtCore.Qt.QueuedConnection
        self.configure_requested.connect(self.configure, queued)
        self.reset_requested.connect(self.reset, queued)
        # Context changes share the same FIFO as ingress bytes. The direct
        # adapter only snapshots an immutable dataclass and enqueues it; file
        # recovery and data-plane mutation still run on this object's thread.
        self.evidence_context_requested.connect(
            self._queue_evidence_context,
            QtCore.Qt.DirectConnection,
        )
        self.ntrip_stats_received.connect(self.handle_ntrip_stats, queued)
        self.ntrip_chunk_received.connect(self.handle_ntrip_chunk, queued)
        self.base_frame_received.connect(self.handle_base_frame, queued)
        self.injection_result_received.connect(self.handle_injection_result, queued)
        self.evidence_drain_requested.connect(
            self.close_evidence_context_and_drain,
            QtCore.Qt.DirectConnection,
        )
        self.stop_requested.connect(self.request_stop, queued)

    @QtCore.pyqtSlot(object)
    def configure(self, payload: object) -> None:
        self._plane.configure(RtcmRouteConfig.from_payload(payload))
        self._emit_snapshot(force=True)

    @QtCore.pyqtSlot(object)
    def reset(self, payload: object) -> None:
        route = RtcmRouteConfig.from_payload(payload)
        self._plane.reset_runtime(route)
        self._latest_ntrip_stats = None
        self._latest_injection_result = None
        self._last_snapshot_s = float("-inf")
        self._base_reference_seen_since_snapshot = False
        self._accepting = True
        self._emit_snapshot(force=True)

    @QtCore.pyqtSlot(object)
    def set_evidence_context(self, payload: object) -> None:
        context = RtcmEvidenceContext.from_payload(payload)
        with self._context_lock:
            self._requested_context = context
        context_key = (context.session_id, str(context.session_root))
        previous = self._evidence_context_snapshot()
        previous_key = (previous.session_id, str(previous.session_root))
        if context.active and (not previous.active or context_key != previous_key):
            try:
                last_sequence, source_sequences = load_rtcm_evidence_high_water(context)
                self._plane.restore_evidence_high_water(
                    last_sequence=last_sequence,
                    source_sequences=source_sequences,
                )
            except Exception as exc:
                if context.context_token:
                    self._validated_context_tokens[context.context_token] = False
                self.integrity_failed.emit(
                    {
                        "schema": "workstation-rtcm-integrity-failure-v1",
                        "sequence": 0,
                        "source": "evidence-index",
                        "record_kind": "recovery",
                        "payload_bytes": 0,
                        "session_id": context.session_id,
                        "context_token": context.context_token,
                        "session_root": (
                            None if context.session_root is None else str(context.session_root)
                        ),
                        "error": str(exc),
                    }
                )
                failed_token = context.context_token
                context = RtcmEvidenceContext()
                with self._context_lock:
                    if self._requested_context.context_token == failed_token:
                        self._requested_context = context
        if context.context_token:
            self._validated_context_tokens[context.context_token] = True
        with self._context_lock:
            self._context = context
        if not context.active:
            self._evidence_started_key = None
        self._emit_snapshot(force=True)

    def _evidence_context_snapshot(self) -> RtcmEvidenceContext:
        with self._context_lock:
            return self._context

    def _requested_evidence_context_snapshot(self) -> RtcmEvidenceContext:
        with self._context_lock:
            return self._requested_context

    @QtCore.pyqtSlot(object)
    def _queue_evidence_context(self, payload: object) -> None:
        context = RtcmEvidenceContext.from_payload(payload)
        queued = False
        apply_directly = False
        with self._ingress_submission_lock:
            with self._context_lock:
                self._requested_context = context
            if QtCore.QThread.currentThread() == self.thread():
                apply_directly = True
            else:
                queued = self._put_ingress_locked("context", {"context": context})
        if apply_directly:
            self.set_evidence_context(context)
            return
        if queued:
            self._schedule_ingress_drain()

    def _put_ingress_locked(self, kind: str, item: dict[str, object]) -> bool:
        """Put while the caller owns _ingress_submission_lock."""

        while self._accepting:
            try:
                self._ingress.put((kind, item), timeout=0.05)
                return True
            except queue.Full:
                with self._ingress_lock:
                    self._backpressure_waits += 1
        return False

    def close_evidence_context_and_drain(self, request: object) -> bool:
        """Close one evidence token and enqueue its ordered durable barrier.

        This method is safe to call through ``evidence_drain_requested`` with
        a DirectConnection. The submission lock linearizes context closure and
        every producer enqueue: bytes accepted before this call remain owned
        by the closing context, while every later byte captures an inactive or
        newly requested context and can never be appended behind this token's
        barrier.
        """

        values = dict(request) if isinstance(request, dict) else {}
        raw_token = (
            values.get("context_token")
            if values
            else request if isinstance(request, str) else ""
        )
        token = str(raw_token or "").strip()
        rejection = ""
        closing_context = RtcmEvidenceContext()
        queued = False
        with self._ingress_submission_lock:
            with self._context_lock:
                requested = self._requested_context
                already_closing = self._closing_evidence_contexts.get(token)
                if already_closing is not None:
                    return True
                if not token:
                    rejection = "RTCM evidence drain requires a context token"
                elif not requested.active or requested.context_token != token:
                    rejection = (
                        "RTCM evidence drain token is not the active ingress owner: "
                        f"requested={token}, active={requested.context_token or '<none>'}"
                    )
                elif values.get("session_id") and str(values["session_id"]) != requested.session_id:
                    rejection = "RTCM evidence drain session identity mismatch"
                elif values.get("session_root") and str(values["session_root"]) != str(
                    requested.session_root
                ):
                    rejection = "RTCM evidence drain root identity mismatch"
                else:
                    closing_context = requested
                    # Subsequent producers snapshot this disabled owner before
                    # the barrier is inserted, all under one lock.
                    self._requested_context = RtcmEvidenceContext()
                    self._closing_evidence_contexts[token] = closing_context
            if not rejection:
                queued = self._put_ingress_locked(
                    "evidence_barrier",
                    {"context": closing_context},
                )
                if not queued:
                    self._closing_evidence_contexts.pop(token, None)
                    rejection = "RTCM pipeline stopped before evidence barrier enqueue"
        if rejection:
            self.evidence_drained.emit(
                self._evidence_drain_payload(
                    closing_context,
                    valid=False,
                    error=rejection,
                    last_sequence=0,
                    source_sequences={},
                    unconfirmed_sequences=(),
                    requested_token=token,
                )
            )
            return False
        self._schedule_ingress_drain()
        return True

    @QtCore.pyqtSlot(object)
    def handle_ntrip_stats(self, stats: object) -> None:
        self._latest_ntrip_stats = stats
        self._emit_snapshot()

    @QtCore.pyqtSlot(object)
    def handle_ntrip_chunk(self, chunk: object) -> None:
        if not self._accepting or not isinstance(chunk, (bytes, bytearray)):
            return
        context = self._requested_evidence_context_snapshot()
        result = self._plane.process_ntrip_chunk(bytes(chunk))
        self._dispatch(result, evidence_context=context)

    @QtCore.pyqtSlot(object)
    def submit_ntrip_envelope(self, envelope: object) -> None:
        self._submit_ingress("ntrip", envelope)

    @QtCore.pyqtSlot(object)
    def handle_base_frame(self, frame: object) -> None:
        if not self._accepting or not isinstance(frame, (bytes, bytearray)):
            return
        context = self._requested_evidence_context_snapshot()
        result = self._plane.process_base_frame(bytes(frame))
        self._dispatch(result, evidence_context=context)

    @QtCore.pyqtSlot(object)
    def submit_base_envelope(self, envelope: object) -> None:
        self._submit_ingress("base", envelope)

    def _submit_ingress(self, kind: str, envelope: object) -> None:
        if isinstance(envelope, dict):
            values = dict(envelope)
            payload = values.get("payload")
        else:
            values = {}
            payload = envelope
        if not isinstance(payload, (bytes, bytearray)) or not payload:
            return
        with self._ingress_submission_lock:
            context = self._requested_evidence_context_snapshot()
            item = {
                "payload": bytes(payload),
                "receive_utc_ns": int(values.get("receive_utc_ns") or time.time_ns()),
                "receive_monotonic_ns": int(
                    values.get("receive_monotonic_ns") or time.monotonic_ns()
                ),
                # This immutable dataclass is captured and queued under the
                # same lock used by the close barrier. No old owner can appear
                # behind its barrier.
                "evidence_context": context,
                "evidence_context_token": context.context_token,
                "evidence_session_id": context.session_id,
                "evidence_session_root": (
                    None if context.session_root is None else str(context.session_root)
                ),
            }
            queued = self._put_ingress_locked(kind, item)
        if not queued:
            self.integrity_failed.emit(
                {
                    "schema": "workstation-rtcm-integrity-failure-v1",
                    "sequence": 0,
                    "source": kind,
                    "record_kind": "ingress_envelope",
                    "payload_bytes": len(item["payload"]),
                    "session_id": context.session_id,
                    "context_token": context.context_token,
                    "session_root": (
                        None if context.session_root is None else str(context.session_root)
                    ),
                    "error": "RTCM envelope arrived after pipeline intake closed",
                }
            )
            return
        self._schedule_ingress_drain()

    def _schedule_ingress_drain(self) -> None:
        with self._ingress_lock:
            if self._drain_scheduled:
                return
            self._drain_scheduled = True
        QtCore.QMetaObject.invokeMethod(
            self,
            "_drain_ingress",
            QtCore.Qt.QueuedConnection,
        )

    @QtCore.pyqtSlot()
    def _drain_ingress(self) -> None:
        with self._ingress_lock:
            self._drain_scheduled = False
        for _index in range(self.INGRESS_BATCH_SIZE):
            try:
                kind, envelope = self._ingress.get_nowait()
            except queue.Empty:
                break
            if kind == "context":
                try:
                    self.set_evidence_context(envelope.get("context"))
                finally:
                    self._ingress.task_done()
                continue
            if kind == "evidence_barrier":
                try:
                    context = envelope.get("context")
                    if isinstance(context, RtcmEvidenceContext):
                        pending = {
                            sequence
                            for sequence, context_token in self._injection_context_by_sequence.items()
                            if context_token == context.context_token
                        }
                        self._pending_evidence_barriers.append(
                            _EvidenceDrainBarrier(
                                context=context,
                                pending_injection_sequences=pending,
                                deadline_s=time.monotonic()
                                + self.evidence_drain_timeout_s,
                            )
                        )
                        with self._context_lock:
                            if self._context.context_token == context.context_token:
                                self._context = RtcmEvidenceContext()
                        if self._evidence_started_key is not None and (
                            self._evidence_started_key[0] == context.context_token
                        ):
                            self._evidence_started_key = None
                        self._maybe_complete_evidence_barriers()
                finally:
                    self._ingress.task_done()
                continue
            payload = bytes(envelope["payload"])
            kwargs = {
                "receive_utc_ns": int(envelope["receive_utc_ns"]),
                "receive_monotonic_ns": int(envelope["receive_monotonic_ns"]),
            }
            evidence_context = envelope.get("evidence_context")
            if not isinstance(evidence_context, RtcmEvidenceContext):
                evidence_context = RtcmEvidenceContext()
            if evidence_context.context_token and not self._validated_context_tokens.get(
                evidence_context.context_token,
                False,
            ):
                evidence_context = RtcmEvidenceContext()
            try:
                if kind == "ntrip":
                    result = self._plane.process_ntrip_chunk(payload, **kwargs)
                else:
                    result = self._plane.process_base_frame(payload, **kwargs)
                self._dispatch(result, evidence_context=evidence_context)
            except Exception as exc:
                if evidence_context.context_token:
                    self._evidence_errors_by_context.setdefault(
                        evidence_context.context_token,
                        [],
                    ).append(f"pipeline-processing: {exc}")
                self.integrity_failed.emit(
                    {
                        "schema": "workstation-rtcm-integrity-failure-v1",
                        "sequence": 0,
                        "source": kind,
                        "record_kind": "pipeline-processing",
                        "payload_bytes": len(payload),
                        "session_id": evidence_context.session_id,
                        "context_token": evidence_context.context_token,
                        "session_root": (
                            None
                            if evidence_context.session_root is None
                            else str(evidence_context.session_root)
                        ),
                        "error": str(exc),
                    }
                )
            finally:
                self._ingress.task_done()
        if not self._ingress.empty():
            self._schedule_ingress_drain()
        elif self._stop_after_drain:
            self._maybe_finish_stop()

    def _evidence_drain_payload(
        self,
        context: RtcmEvidenceContext,
        *,
        valid: bool,
        error: str,
        last_sequence: int,
        source_sequences: dict[tuple[str, str], int],
        unconfirmed_sequences: tuple[int, ...],
        requested_token: str = "",
    ) -> dict[str, object]:
        integrity = (
            dict(self._plane.snapshot(now_monotonic_s=time.monotonic())["integrity"])
            if QtCore.QThread.currentThread() == self.thread()
            else {}
        )
        return {
            "schema": "workstation-rtcm-evidence-drain-v1",
            "context_token": context.context_token or str(requested_token),
            "session_id": context.session_id,
            "session_root": (
                None if context.session_root is None else str(context.session_root)
            ),
            "valid": bool(valid),
            "error": str(error),
            "high_water": {
                "last_sequence": int(last_sequence),
                "source_sequences": tuple(
                    {
                        "source": source,
                        "record_kind": record_kind,
                        "sequence": int(sequence),
                    }
                    for (source, record_kind), sequence in sorted(
                        source_sequences.items()
                    )
                ),
            },
            "unconfirmed_injection_sequences": tuple(unconfirmed_sequences),
            "integrity": integrity,
        }

    def _maybe_complete_evidence_barriers(self) -> None:
        while self._pending_evidence_barriers:
            barrier = self._pending_evidence_barriers[0]
            still_pending = {
                sequence
                for sequence in barrier.pending_injection_sequences
                if sequence in self._injection_context_by_sequence
            }
            unconfirmed: tuple[int, ...] = ()
            errors = list(
                self._evidence_errors_by_context.get(
                    barrier.context.context_token,
                    (),
                )
            )
            if still_pending and time.monotonic() < barrier.deadline_s:
                self._schedule_evidence_barrier_poll()
                return
            if still_pending:
                unconfirmed = self._plane.mark_injections_unconfirmed(still_pending)
                for sequence in unconfirmed:
                    self._injection_context_by_sequence.pop(sequence, None)
                error = (
                    "RTCM injection outcomes did not drain before evidence barrier timeout: "
                    + ",".join(str(value) for value in unconfirmed)
                )
                errors.append(error)
                self.integrity_failed.emit(
                    {
                        "schema": "workstation-rtcm-integrity-failure-v1",
                        "sequence": unconfirmed[0] if unconfirmed else 0,
                        "source": "pixhawk-injection",
                        "record_kind": "delivery-outcome",
                        "payload_bytes": 0,
                        "session_id": barrier.context.session_id,
                        "context_token": barrier.context.context_token,
                        "session_root": str(barrier.context.session_root),
                        "unconfirmed_sequences": unconfirmed,
                        "error": error,
                    }
                )
            last_sequence = 0
            source_sequences: dict[tuple[str, str], int] = {}
            try:
                last_sequence, source_sequences = load_rtcm_evidence_high_water(
                    barrier.context
                )
            except Exception as exc:
                errors.append(f"RTCM evidence semantic validation failed: {exc}")
            payload = self._evidence_drain_payload(
                barrier.context,
                valid=not errors,
                error="; ".join(errors),
                last_sequence=last_sequence,
                source_sequences=source_sequences,
                unconfirmed_sequences=unconfirmed,
            )
            self._pending_evidence_barriers.pop(0)
            with self._ingress_submission_lock:
                self._closing_evidence_contexts.pop(
                    barrier.context.context_token,
                    None,
                )
            self._evidence_errors_by_context.pop(
                barrier.context.context_token,
                None,
            )
            self.evidence_drained.emit(payload)

    def _schedule_evidence_barrier_poll(self) -> None:
        if self._evidence_barrier_poll_scheduled:
            return
        self._evidence_barrier_poll_scheduled = True
        QtCore.QTimer.singleShot(25, self._poll_evidence_barriers)

    def _poll_evidence_barriers(self) -> None:
        self._evidence_barrier_poll_scheduled = False
        self._maybe_complete_evidence_barriers()

    def _dispatch(
        self,
        result,
        *,
        evidence_context: RtcmEvidenceContext | None = None,
    ) -> None:
        context = (
            self._evidence_context_snapshot()
            if evidence_context is None
            else evidence_context
        )
        for label, payload in result.base_writes:
            self.base_write_requested.emit(str(label), bytes(payload))
        for request in result.injection_requests:
            age_s = max(
                0.0,
                (time.monotonic_ns() - request.receive_monotonic_ns) / 1_000_000_000.0,
            )
            if self.max_injection_age_s > 0.0 and age_s > self.max_injection_age_s:
                self._plane.record_stale_injection_drop(request, age_s=age_s)
                self._latest_injection_result = {
                    "pipeline_sequence": request.sequence,
                    "payload_bytes": len(request.payload),
                    "sent": (),
                    "failed": (),
                    "dropped": 1,
                    "drop_reason": "stale-before-injection",
                    "age_s": age_s,
                }
                continue
            self._plane.record_injection_expected(request)
            self._injection_context_by_sequence[request.sequence] = context.context_token
            envelope = request.as_dict()
            envelope.update(
                {
                    "evidence_context_token": context.context_token,
                    "evidence_session_id": context.session_id,
                    "evidence_session_root": (
                        None if context.session_root is None else str(context.session_root)
                    ),
                }
            )
            self.injection_requested.emit(envelope)
        self._base_reference_seen_since_snapshot = bool(
            self._base_reference_seen_since_snapshot or result.base_reference_seen
        )
        for record in result.evidence_records:
            if not context.active:
                continue
            self._plane.record_evidence_expected(record)
            try:
                written = append_rtcm_evidence(context, record)
            except Exception as exc:
                self._plane.record_evidence_failure(sequence=record.sequence, error=str(exc))
                if context.context_token:
                    self._evidence_errors_by_context.setdefault(
                        context.context_token,
                        [],
                    ).append(f"sequence={record.sequence}: {exc}")
                self.integrity_failed.emit(
                    {
                        "schema": "workstation-rtcm-integrity-failure-v1",
                        "sequence": record.sequence,
                        "source": record.source,
                        "record_kind": record.record_kind,
                        "payload_bytes": len(record.payload),
                        "payload_sha256": record.sha256,
                        "session_id": context.session_id,
                        "context_token": context.context_token,
                        "session_root": (
                            None if context.session_root is None else str(context.session_root)
                        ),
                        "error": str(exc),
                    }
                )
                continue
            if written is None:
                self._plane.record_evidence_failure(
                    sequence=record.sequence,
                    error="active evidence context produced no write",
                )
                if context.context_token:
                    self._evidence_errors_by_context.setdefault(
                        context.context_token,
                        [],
                    ).append(
                        f"sequence={record.sequence}: active evidence context produced no write"
                    )
                continue
            self._plane.record_evidence_written(written)
            evidence_key = (context.context_token, str(context.session_root))
            if evidence_key != self._evidence_started_key:
                self._evidence_started_key = evidence_key
                self.evidence_started.emit(
                    {
                        "schema": "workstation-rtcm-evidence-start-v1",
                        "session_id": context.session_id,
                        "session_root": str(context.session_root),
                        "context_token": context.context_token,
                        "first_sequence": record.sequence,
                        "first_receive_utc_ns": record.receive_utc_ns,
                    }
                )
        self._emit_snapshot()

    @QtCore.pyqtSlot(object)
    def handle_injection_result(self, result: object) -> None:
        if isinstance(result, dict):
            self._latest_injection_result = dict(result)
            self._plane.record_injection_result(result)
            completed_sequences = {
                int(result.get("pipeline_sequence") or 0),
            }
            for envelope in tuple(result.get("dropped_envelopes") or ()):
                if isinstance(envelope, dict):
                    completed_sequences.add(
                        int(envelope.get("pipeline_sequence") or 0)
                    )
            for sequence in completed_sequences:
                if sequence > 0:
                    self._injection_context_by_sequence.pop(sequence, None)
            self._maybe_complete_evidence_barriers()
        self._emit_snapshot()
        if self._stop_after_drain:
            self._maybe_finish_stop()

    def _emit_snapshot(self, *, force: bool = False) -> None:
        now_s = time.monotonic()
        if not force and now_s - self._last_snapshot_s < self.snapshot_interval_s:
            return
        snapshot = self._plane.snapshot(now_monotonic_s=now_s)
        snapshot["ntrip_stats"] = self._latest_ntrip_stats
        snapshot["latest_injection_result"] = self._latest_injection_result
        snapshot["base_reference_seen"] = self._base_reference_seen_since_snapshot
        snapshot["evidence_context"] = self._evidence_context_snapshot().as_dict()
        with self._ingress_lock:
            backpressure_waits = self._backpressure_waits
        snapshot["ingress"] = {
            "queued": self._ingress.qsize(),
            "capacity": self._ingress.maxsize,
            "backpressure_waits": backpressure_waits,
        }
        self._base_reference_seen_since_snapshot = False
        self._last_snapshot_s = now_s
        self.snapshot_ready.emit(snapshot)

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        if self._finished:
            return
        if self._stop_after_drain:
            self._maybe_finish_stop()
            return
        self._accepting = False
        self._stop_after_drain = True
        self._stop_deadline_s = time.monotonic() + self.stop_drain_timeout_s
        if not self._ingress.empty():
            self._schedule_ingress_drain()
            return
        self._maybe_finish_stop()

    def _maybe_finish_stop(self) -> None:
        if not self._ingress.empty():
            self._schedule_ingress_drain()
            return
        if self._plane.pending_injection_count <= 0:
            self._finish_stop()
            return
        if self._stop_deadline_s is not None and time.monotonic() >= self._stop_deadline_s:
            sequences = self._plane.mark_pending_injections_unconfirmed()
            for sequence in sequences:
                self._injection_context_by_sequence.pop(sequence, None)
            self._maybe_complete_evidence_barriers()
            self.integrity_failed.emit(
                {
                    "schema": "workstation-rtcm-integrity-failure-v1",
                    "sequence": sequences[0] if sequences else 0,
                    "source": "pixhawk-injection",
                    "record_kind": "delivery-outcome",
                    "payload_bytes": 0,
                    "session_id": self._evidence_context_snapshot().session_id,
                    "context_token": self._evidence_context_snapshot().context_token,
                    "unconfirmed_sequences": sequences,
                    "error": "RTCM injection outcomes did not drain before shutdown timeout",
                }
            )
            self._finish_stop()
            return
        QtCore.QTimer.singleShot(25, self._maybe_finish_stop)

    def _finish_stop(self) -> None:
        if self._finished:
            return
        self._emit_snapshot(force=True)
        self._finished = True
        self.finished.emit()


class GnssBaseSerialWorker(QtCore.QObject):
    WRITE_TIMEOUT_S = 1.0
    MAX_WRITE_QUEUE = 64
    MAX_NMEA_BUFFER_BYTES = 4096

    connected = QtCore.pyqtSignal(str, int)
    stats_changed = QtCore.pyqtSignal(int, float, float)
    survey_in_changed = QtCore.pyqtSignal(object)
    satellite_count_changed = QtCore.pyqtSignal(int, str)
    satellite_quality_changed = QtCore.pyqtSignal(object)
    position_changed = QtCore.pyqtSignal(object)
    ubx_message_received = QtCore.pyqtSignal(int, int, object)
    rtcm_frame_received = QtCore.pyqtSignal(object)
    rtcm_frame_envelope_received = QtCore.pyqtSignal(object)
    command_sent = QtCore.pyqtSignal(str, int)
    failed = QtCore.pyqtSignal(str)
    disconnected = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, port: str, baud: int, height_type: HeightType = HeightType.ELLIPSOID) -> None:
        super().__init__()
        self.port = port
        self.baud = int(baud)
        self.height_type = HeightType(height_type)
        self._stop_requested = False
        self._write_queue: queue.Queue[tuple[str, bytes]] = queue.Queue(
            maxsize=self.MAX_WRITE_QUEUE
        )

    @QtCore.pyqtSlot()
    def run(self) -> None:
        serial_obj = None
        total_bytes = 0
        started_s = time.monotonic()
        last_emit_s = started_s
        last_rx_s: float | None = None
        last_nav_svin_poll_s = 0.0
        ubx_parser = UbxStreamParser()
        rtcm_parser = Rtcm3StreamParser()
        nmea_buffer = ""
        try:
            try:
                import serial  # type: ignore[import-not-found]
            except Exception as exc:
                self.failed.emit(f"pyserial 필요: {exc}")
                return
            serial_obj = serial.Serial(
                self.port,
                self.baud,
                timeout=0.2,
                write_timeout=self.WRITE_TIMEOUT_S,
            )
            self.connected.emit(self.port, self.baud)
            while not self._stop_requested:
                self._drain_writes(serial_obj)
                now_s = time.monotonic()
                if now_s - last_nav_svin_poll_s >= 2.0:
                    serial_obj.write(ubx_nav_svin_poll_frame())
                    serial_obj.write(ubx_nav_pvt_poll_frame())
                    try:
                        serial_obj.flush()
                    except Exception:
                        pass
                    last_nav_svin_poll_s = now_s
                chunk = serial_obj.read(4096)
                if chunk:
                    total_bytes += len(chunk)
                    last_rx_s = now_s
                    for rtcm_frame in rtcm_parser.feed(chunk):
                        self.rtcm_frame_envelope_received.emit(
                            {
                                "payload": rtcm_frame,
                                "receive_monotonic_ns": int(now_s * 1_000_000_000),
                                "receive_utc_ns": time.time_ns(),
                            }
                        )
                        self.rtcm_frame_received.emit(rtcm_frame)
                    nmea_buffer += chunk.decode("ascii", "ignore")
                    if "\n" in nmea_buffer:
                        lines = nmea_buffer.splitlines()
                        nmea_buffer = "" if nmea_buffer.endswith(("\n", "\r")) else lines.pop() if lines else nmea_buffer
                        for line in lines:
                            gga = parse_nmea_gga_satellite_count(line)
                            if gga is not None:
                                satellites, fix_quality = gga
                                self.satellite_count_changed.emit(satellites, f"NMEA-GGA fix={fix_quality}")
                    elif len(nmea_buffer) > self.MAX_NMEA_BUFFER_BYTES:
                        # A stream with no newline bytes for a long stretch
                        # (e.g. UBX/RTCM-only traffic) would otherwise grow
                        # this buffer unbounded. Discard and resync on the
                        # next newline.
                        _LOGGER.warning(
                            "GnssBaseSerialWorker: discarding %d-byte NMEA buffer with no newline (port=%s)",
                            len(nmea_buffer),
                            self.port,
                        )
                        nmea_buffer = ""
                    for message_class, message_id, payload in ubx_parser.feed(chunk):
                        self.ubx_message_received.emit(message_class, message_id, payload)
                        if message_class == 0x01 and message_id == 0x3B:
                            survey_in = parse_ubx_nav_svin(payload)
                            if survey_in is not None:
                                self.survey_in_changed.emit(survey_in)
                        if message_class == 0x01 and message_id == 0x07:
                            coordinate = parse_ubx_nav_pvt_coordinate_sample(
                                message_class,
                                message_id,
                                payload,
                                timestamp_s=now_s,
                                height_type=self.height_type,
                            )
                            if coordinate is not None:
                                self.position_changed.emit(coordinate)
                        satellite_count = parse_ubx_base_satellite_count(message_class, message_id, payload)
                        if satellite_count is not None:
                            source = "NAV-PVT" if message_id == 0x07 else "NAV-SAT"
                            self.satellite_count_changed.emit(satellite_count, source)
                        satellite_quality = parse_ubx_nav_sat_quality(message_class, message_id, payload)
                        if satellite_quality is not None:
                            self.satellite_quality_changed.emit(satellite_quality)
                if now_s - last_emit_s >= 0.5:
                    elapsed_s = max(1e-6, now_s - started_s)
                    rate_bps = total_bytes / elapsed_s
                    age_s = -1.0 if last_rx_s is None else max(0.0, now_s - last_rx_s)
                    self.stats_changed.emit(total_bytes, rate_bps, age_s)
                    last_emit_s = now_s
            self.disconnected.emit("disconnected")
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if serial_obj is not None:
                try:
                    serial_obj.close()
                except Exception:
                    pass
            self.finished.emit()
    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True

    @QtCore.pyqtSlot(str, bytes)
    def send_bytes(self, label: str, payload: bytes) -> None:
        item = (str(label), bytes(payload))
        try:
            self._write_queue.put_nowait(item)
            return
        except queue.Full:
            pass
        # The base output is a live correction/command stream.  Never block a
        # producer behind a stalled USB writer: discard the oldest queued item
        # and retain the newest request.  A racing consumer may free the slot
        # between operations, so retry until either operation succeeds.
        while True:
            try:
                self._write_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._write_queue.put_nowait(item)
                return
            except queue.Full:
                continue

    def _drain_writes(self, serial_obj) -> None:
        while True:
            try:
                label, payload = self._write_queue.get_nowait()
            except queue.Empty:
                return
            serial_obj.write(payload)
            try:
                serial_obj.flush()
            except Exception:
                pass
            self.command_sent.emit(label, len(payload))


def _is_transient_network_error(exc: BaseException) -> bool:
    """Tell a link outage that clears itself from a fault that never will.

    Field uplinks are phone hotspots and LTE modems: they drop the uplink DNS
    server mid-session and come back seconds later. ``EAI_AGAIN`` ("Temporary
    failure in name resolution") is exactly that case and is worth waiting out.

    A wrong hostname (``EAI_NONAME``), rejected credentials, or an unknown
    mountpoint are the opposite -- retrying repeats the same rejection forever
    -- so those stay on the short fatal count.
    """

    if isinstance(exc, socket.gaierror):
        # EAI_AGAIN only. EAI_NONAME means the caster host itself is wrong.
        return exc.errno == socket.EAI_AGAIN
    if isinstance(exc, (socket.timeout, TimeoutError, ConnectionError)):
        return True
    if isinstance(exc, OSError):
        return exc.errno in {
            errno_module.ENETUNREACH,
            errno_module.EHOSTUNREACH,
            errno_module.ENETDOWN,
            errno_module.ENETRESET,
        }
    return False


class NtripNetworkWorker(QtCore.QObject):
    """Qt adapter for the blocking, dependency-free NTRIP client."""

    # A caster rejection (bad credentials, unknown mountpoint, wrong host)
    # never clears itself by retrying -- it just repeats the same
    # "reconnecting" message forever, leaving the UI's Start control disabled
    # (see start_gnss_network's early-return guard) with no visible fault.
    # Once a failure repeats this many times in a row, treat it as fatal:
    # stop retrying and let ``failed`` surface it instead.
    MAX_CONSECUTIVE_FAILURES = 3
    # A dropped uplink is not a rejection. It clears itself once the hotspot
    # or modem comes back, so give it the length of a real field outage
    # instead of the three strikes meant for a caster that keeps saying no.
    # At the default 5 s reconnect interval this is roughly five minutes,
    # and every attempt still reports progress through ``reconnecting``.
    MAX_CONSECUTIVE_TRANSIENT_FAILURES = 60

    connected = QtCore.pyqtSignal(str, object)
    stats_changed = QtCore.pyqtSignal(object)
    stats_snapshot_changed = QtCore.pyqtSignal(object)
    chunk_received = QtCore.pyqtSignal(object)
    chunk_envelope_received = QtCore.pyqtSignal(object)
    reconnecting = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()
    # Per-event evidence from NtripClient (connect success, initial/periodic
    # GGA TX outcome) that has no other signal to ride on -- unlike a stream
    # failure, these do not stop or restart anything, so they need their own
    # channel to reach the visible log at all.
    diagnostic = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        config: NetworkConfig,
        gga_provider=None,
        username: str | None = None,
        password: str | None = None,
    ) -> None:
        super().__init__()
        self.config = config
        self.gga_provider = gga_provider
        # Explicit credentials resolved once by the caller (session entry,
        # OS keyring, or legacy env-var fallback -- see
        # gnss_runtime_controller._resolve_ntrip_credentials). Passing them
        # through explicitly means every reconnect attempt below reuses the
        # exact same credential instead of re-reading os.environ, and keeps
        # this worker from ever needing to know which tier they came from.
        self._username = username
        self._password = password
        self._stop_event = threading.Event()
        self._client: NtripClient | None = None

    @QtCore.pyqtSlot()
    def run(self) -> None:
        consecutive_failures = 0
        last_stats_snapshot_s = float("-inf")
        try:
            while not self._stop_event.is_set():
                client = NtripClient(
                    self.config,
                    username=self._username,
                    password=self._password,
                    diagnostics=self.diagnostic.emit,
                )
                self._client = client
                try:
                    initial_gga = self.gga_provider() if self.gga_provider is not None else None
                    status_line, initial_payload = client.connect(gga_sentence=initial_gga)
                    consecutive_failures = 0
                    self.connected.emit(status_line, client.stats)
                    for chunk in client.iter_chunks(
                        self._stop_event,
                        gga_provider=self.gga_provider,
                        initial_payload=initial_payload,
                    ):
                        receive_monotonic_ns = time.monotonic_ns()
                        receive_utc_ns = time.time_ns()
                        if client.stats is not None:
                            self.stats_changed.emit(client.stats)
                            now_s = receive_monotonic_ns / 1_000_000_000.0
                            if now_s - last_stats_snapshot_s >= 0.5:
                                self.stats_snapshot_changed.emit(client.stats)
                                last_stats_snapshot_s = now_s
                        self.chunk_envelope_received.emit(
                            {
                                "payload": bytes(chunk),
                                "receive_monotonic_ns": receive_monotonic_ns,
                                "receive_utc_ns": receive_utc_ns,
                            }
                        )
                        self.chunk_received.emit(chunk)
                    if self._stop_event.is_set():
                        break
                    raise RuntimeError("NTRIP stream ended without a stop request")
                except Exception as exc:
                    if self._stop_event.is_set():
                        break
                    error_text = str(exc)
                    if "401" in error_text or "unauthorized" in error_text.lower():
                        self.failed.emit(error_text)
                        break
                    consecutive_failures += 1
                    transient = _is_transient_network_error(exc)
                    failure_limit = (
                        self.MAX_CONSECUTIVE_TRANSIENT_FAILURES
                        if transient
                        else self.MAX_CONSECUTIVE_FAILURES
                    )
                    if consecutive_failures >= failure_limit:
                        self.failed.emit(error_text)
                        break
                    if transient:
                        self.reconnecting.emit(
                            f"{error_text} (uplink 대기 {consecutive_failures}/{failure_limit})"
                        )
                    else:
                        self.reconnecting.emit(error_text)
                    # The failed client is closed before retrying.  A fresh
                    # client owns the interruptible reconnect delay because a
                    # closed client intentionally never retries.
                    client.close()
                    self._client = None
                    retry_waiter = NtripClient(self.config)
                    if not retry_waiter.wait_for_reconnect(self._stop_event):
                        break
                    continue
                finally:
                    client.close()
                    if self._client is client:
                        self._client = None
        finally:
            self._client = None
            self.finished.emit()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_event.set()
        client = self._client
        if client is not None:
            client.close()


class GnssBaseSerialDetectWorker(QtCore.QObject):
    detected = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, baud: int) -> None:
        super().__init__()
        self.baud = int(baud)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            candidates = gnss_base_serial_port_candidates()
            rows = probe_gnss_base_serial_ports(candidates, baud=self.baud, timeout_s=0.6)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.detected.emit(rows)
        finally:
            self.finished.emit()
