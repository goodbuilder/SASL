"""Durable accepted-pair evidence writer backed by native shared-memory rings."""

from __future__ import annotations

import hashlib
import hmac
import json
import queue
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from PyQt5 import QtCore, QtGui

from Workstation.workstation_core.measurement_data_store import (
    _safe_token,
    creation_timestamp_text,
)
from Workstation.workstation_core.native_frame_ring import (
    FrameKey,
    NativeFrameLease,
    SharedNativeFrameRing,
)
from Workstation.workstation_core.native_frame_store import NativeFrameStore
from Workstation.workstation_core.qimage_convert import qimage_from_bgr as _qimage_from_bgr


EVIDENCE_FRAME_UNAVAILABLE = "EVIDENCE_FRAME_UNAVAILABLE"


@dataclass(frozen=True)
class RingFrameRequest:
    side: str
    ring: SharedNativeFrameRing
    key: FrameKey
    lease: NativeFrameLease
    metadata: dict[str, Any]
    pinned_descriptor_sha256: str


@dataclass(frozen=True)
class RingPairJob:
    request_id: str
    pair_id: str
    purpose: str
    frames: tuple[RingFrameRequest, ...]
    context: dict[str, object]


class RingPairReservation:
    """Single-owner lease bundle transferred from pairer to writer.

    The pair coordinator owns a new reservation until the GUI consumes its
    result.  ``transfer`` moves the already-pinned leases to exactly one writer
    job; ``release`` is idempotent and only releases leases that were not
    transferred.  This keeps GUI polling latency outside the ring overwrite
    window without sharing mutable GUI objects with the writer thread.
    """

    def __init__(self, frames: Sequence[RingFrameRequest]) -> None:
        self.frames = tuple(frames)
        self._lock = threading.Lock()
        self._state = "owned"

    @property
    def active(self) -> bool:
        with self._lock:
            return self._state == "owned"

    @property
    def frame_keys(self) -> tuple[FrameKey, ...]:
        return tuple(frame.key for frame in self.frames)

    def transfer(self) -> tuple[RingFrameRequest, ...] | None:
        with self._lock:
            if self._state != "owned":
                return None
            self._state = "transferred"
            return self.frames

    def release(self) -> bool:
        with self._lock:
            if self._state != "owned":
                return False
            self._state = "released"
        released = True
        for frame in self.frames:
            released = frame.ring.release(frame.lease) and released
        return released


def reserve_ring_pair(
    frames: Sequence[
        tuple[str, SharedNativeFrameRing, FrameKey, Mapping[str, Any]]
    ],
) -> RingPairReservation | None:
    """Pin every exact frame atomically from the caller's point of view."""

    requested = tuple(frames)
    if len(requested) != 2 or {str(item[0]) for item in requested} != {
        "left",
        "right",
    }:
        raise ValueError("native pair reservation requires one left and one right frame")
    pinned: list[RingFrameRequest] = []
    try:
        for side, ring, key, metadata in requested:
            if str(side) != key.camera_id or ring.camera_id != key.camera_id:
                raise ValueError("native pair side, ring, and FrameKey must agree")
            lease = ring.pin(key)
            if lease is None:
                for frame in pinned:
                    frame.ring.release(frame.lease)
                return None
            try:
                descriptor = ring.descriptor(key)
                if descriptor is None:
                    ring.release(lease)
                    for frame in pinned:
                        frame.ring.release(frame.lease)
                    return None
                descriptor_sha256 = str(
                    descriptor.to_mapping().get("frame_descriptor_sha256") or ""
                )
                request = RingFrameRequest(
                    side=str(side),
                    ring=ring,
                    key=key,
                    lease=lease,
                    metadata=dict(metadata),
                    pinned_descriptor_sha256=descriptor_sha256,
                )
            except Exception:
                ring.release(lease)
                raise
            pinned.append(request)
    except Exception:
        for frame in pinned:
            frame.ring.release(frame.lease)
        raise
    if len(pinned) != len(requested):
        for frame in pinned:
            frame.ring.release(frame.lease)
        return None
    return RingPairReservation(pinned)



class AsyncNativeEvidenceWriter(QtCore.QObject):
    """Bounded writer whose progress never depends on GUI signal delivery."""

    completed = QtCore.pyqtSignal(object)
    DEFAULT_MAX_PENDING = 16
    MAX_UNACKNOWLEDGED_RESULTS = 128

    def __init__(
        self,
        session_root: Path | str,
        *,
        max_pending: int = DEFAULT_MAX_PENDING,
        timestamp_filenames: bool = False,
        events_path: Path | str | None = None,
        parent: QtCore.QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.root = Path(session_root).expanduser().resolve()
        self.max_pending = max(1, int(max_pending))
        self.timestamp_filenames = bool(timestamp_filenames)
        self.events_path = (
            None
            if events_path is None
            else Path(events_path).expanduser().resolve()
        )
        self._lock = threading.RLock()
        self._pending: set[str] = set()
        self._results: dict[str, dict[str, object]] = {}
        self._result_order: list[str] = []
        self._queue: queue.Queue[RingPairJob | None] = queue.Queue(
            maxsize=self.max_pending
        )
        self._store: NativeFrameStore | None = None
        self._closed = False
        self._metrics = {
            "enqueued": 0,
            "saved": 0,
            "failed": 0,
            "frame_unavailable": 0,
            "queue_full": 0,
            "pending_high_water": 0,
        }
        self._thread = threading.Thread(
            target=self._run,
            name="native-evidence-writer",
            daemon=True,
        )
        self._thread.start()

    @property
    def pending_count(self) -> int:
        with self._lock:
            return len(self._pending)

    def metrics(self) -> dict[str, int]:
        with self._lock:
            return {
                **{key: int(value) for key, value in self._metrics.items()},
                "pending": len(self._pending),
                "unacknowledged_results": len(self._results),
                "capacity": self.max_pending,
            }

    def _release_requests(self, frames: Sequence[RingFrameRequest]) -> None:
        for frame in frames:
            frame.ring.release(frame.lease)

    def enqueue_ring_pair(
        self,
        *,
        request_id: str,
        pair_id: str,
        purpose: str,
        frames: Sequence[
            tuple[str, SharedNativeFrameRing, FrameKey, Mapping[str, Any]]
        ],
        context: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        request = str(request_id)
        with self._lock:
            if self._closed:
                return {
                    "native_pair_status": "writer_closed",
                    "evidence_state": "FAILED",
                }
            if request in self._pending:
                return {
                    "native_pair_status": "duplicate_request",
                    "evidence_state": "FAILED",
                }
            if len(self._pending) >= self.max_pending:
                self._metrics["queue_full"] += 1
                return {
                    "native_pair_status": "queue_full",
                    "native_pair_error": (
                        f"native evidence queue limit {self.max_pending} reached"
                    ),
                    "evidence_state": "FAILED",
                }
        try:
            reservation = reserve_ring_pair(frames)
        except (TypeError, ValueError) as exc:
            with self._lock:
                self._metrics["frame_unavailable"] += 1
            return {
                "native_pair_status": EVIDENCE_FRAME_UNAVAILABLE,
                "native_pair_error": f"{type(exc).__name__}: {exc}",
                "evidence_state": "REJECTED",
            }
        if reservation is None:
            with self._lock:
                self._metrics["frame_unavailable"] += 1
            return {
                "native_pair_status": EVIDENCE_FRAME_UNAVAILABLE,
                "native_pair_error": "one or more exact native frames are unavailable",
                "evidence_state": "REJECTED",
            }
        return self.enqueue_reserved_pair(
            request_id=request_id,
            pair_id=pair_id,
            purpose=purpose,
            reservation=reservation,
            context=context,
        )

    def enqueue_reserved_pair(
        self,
        *,
        request_id: str,
        pair_id: str,
        purpose: str,
        reservation: RingPairReservation,
        context: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        """Consume one pre-pinned pair reservation and schedule its writer job."""

        request = str(request_id)
        with self._lock:
            if self._closed:
                reservation.release()
                return {
                    "native_pair_status": "writer_closed",
                    "evidence_state": "FAILED",
                }
            if request in self._pending:
                reservation.release()
                return {
                    "native_pair_status": "duplicate_request",
                    "evidence_state": "FAILED",
                }
            if len(self._pending) >= self.max_pending:
                reservation.release()
                self._metrics["queue_full"] += 1
                return {
                    "native_pair_status": "queue_full",
                    "native_pair_error": (
                        f"native evidence queue limit {self.max_pending} reached"
                    ),
                    "evidence_state": "FAILED",
                }

            pinned = reservation.transfer()
            if pinned is None:
                self._metrics["frame_unavailable"] += 1
                return {
                    "native_pair_status": EVIDENCE_FRAME_UNAVAILABLE,
                    "native_pair_error": "native pair reservation is no longer owned",
                    "evidence_state": "REJECTED",
                }
            stored_pair_id = str(pair_id)
            if self.timestamp_filenames:
                stored_pair_id = (
                    f"{creation_timestamp_text()}_"
                    f"{_safe_token(pair_id, 'frame')}"
                )
            job = RingPairJob(
                request_id=request,
                pair_id=stored_pair_id,
                purpose=str(purpose),
                frames=tuple(pinned),
                context=dict(context or {}),
            )
            self._pending.add(request)
            try:
                self._queue.put_nowait(job)
            except queue.Full:
                self._pending.discard(request)
                self._release_requests(pinned)
                self._metrics["queue_full"] += 1
                return {
                    "native_pair_status": "queue_full",
                    "native_pair_error": (
                        f"native evidence queue limit {self.max_pending} reached"
                    ),
                    "evidence_state": "FAILED",
                }
            self._metrics["enqueued"] += 1
            self._metrics["pending_high_water"] = max(
                self._metrics["pending_high_water"],
                len(self._pending),
            )

        scheduled: dict[str, object] = {
            "native_pair_status": "pending",
            "native_pair_request_id": request,
            "evidence_state": "WRITING",
        }
        identifier = _safe_token(stored_pair_id, "frame")
        purpose_value = _safe_token(purpose, "unspecified")
        for frame in job.frames:
            scheduled.update(
                {
                    f"native_{frame.side}_frame_id": identifier,
                    f"native_{frame.side}_source_frame_id": frame.key.text,
                    f"native_{frame.side}_png_path": (
                        Path("frames")
                        / purpose_value
                        / f"{identifier}-{frame.side}.png"
                    ).as_posix(),
                }
            )
        return scheduled

    @staticmethod
    def _validate_pinned_snapshot(frame: RingFrameRequest, snapshot) -> None:
        descriptor = snapshot.descriptor
        if frame.side != frame.key.camera_id:
            raise ValueError("native evidence side differs from pinned FrameKey camera")
        if descriptor.key != frame.key or descriptor.channels != 3:
            raise ValueError("pinned native descriptor identity mismatch")
        if len(snapshot.data) != descriptor.byte_length:
            raise ValueError("pinned native descriptor byte length mismatch")

        incoming_key = FrameKey.from_mapping(frame.metadata)
        if incoming_key != frame.key:
            raise ValueError("incoming native frame identity differs from pinned frame")
        incoming_sha256 = str(
            frame.metadata.get("frame_descriptor_sha256") or ""
        ).strip().lower()
        current_sha256 = str(
            descriptor.to_mapping().get("frame_descriptor_sha256") or ""
        ).strip().lower()
        pinned_sha256 = str(frame.pinned_descriptor_sha256 or "").strip().lower()
        if not incoming_sha256 or not hmac.compare_digest(
            incoming_sha256,
            pinned_sha256,
        ):
            raise ValueError(
                "incoming native frame descriptor hash differs from pinned descriptor"
            )
        if not current_sha256 or not hmac.compare_digest(
            current_sha256,
            pinned_sha256,
        ):
            raise ValueError("pinned native frame descriptor changed after reservation")

    def _persist(self, job: RingPairJob) -> dict[str, object]:
        if self._store is None:
            self._store = NativeFrameStore(
                self.root,
                events_path=self.events_path,
            )
        fields: dict[str, object] = {
            "native_pair_status": "complete",
            "evidence_state": "SAVED",
        }
        ack_rows: list[dict[str, object]] = []
        failed: list[str] = []
        snapshots: list[tuple[RingFrameRequest, object]] = []
        try:
            for frame in job.frames:
                try:
                    snapshot = frame.ring.read(frame.lease)
                    self._validate_pinned_snapshot(frame, snapshot)
                    snapshots.append((frame, snapshot))
                except Exception as exc:  # noqa: BLE001 - pair validation fails closed.
                    failed.append(frame.side)
                    fields[f"native_{frame.side}_error"] = (
                        f"{type(exc).__name__}: {exc}"
                    )

            # Descriptor/key validation is an all-pair preflight. Never write
            # the other side when one incoming identity is contradictory.
            if not failed:
                for frame, snapshot in snapshots:
                    descriptor = snapshot.descriptor
                    image = _qimage_from_bgr(
                        snapshot.data,
                        width=descriptor.width,
                        height=descriptor.height,
                    )
                    metadata = dict(frame.metadata)
                    metadata.update(descriptor.to_mapping())
                    metadata.update(
                        {
                            "source_native_frame_id": frame.key.text,
                            "native_lossless_source": True,
                            "overlay_applied": False,
                        }
                    )
                    try:
                        artifact = self._store.persist(
                            side=frame.side,
                            purpose=job.purpose,
                            frame_id=job.pair_id,
                            image=image,
                            metadata=metadata,
                        )
                    except Exception as exc:  # noqa: BLE001 - each side stays explicit.
                        failed.append(frame.side)
                        fields[f"native_{frame.side}_error"] = (
                            f"{type(exc).__name__}: {exc}"
                        )
                        continue
                    row = {
                        "side": frame.side,
                        "source_frame_key": frame.key.text,
                        "frame_descriptor_sha256": frame.pinned_descriptor_sha256,
                        "relative_path": artifact.relative_path,
                        "size_bytes": artifact.size_bytes,
                        "sha256": artifact.sha256,
                        "timestamp_s": artifact.timestamp_s,
                        "source_timestamp_s": artifact.source_timestamp_s,
                    }
                    ack_rows.append(row)
                    fields.update(
                        {
                            f"native_{frame.side}_frame_id": artifact.frame_id,
                            f"native_{frame.side}_source_frame_id": frame.key.text,
                            f"native_{frame.side}_frame_descriptor_sha256": (
                                frame.pinned_descriptor_sha256
                            ),
                            f"native_{frame.side}_png_path": artifact.relative_path,
                            f"native_{frame.side}_png_sha256": artifact.sha256,
                            f"native_{frame.side}_width_px": artifact.width,
                            f"native_{frame.side}_height_px": artifact.height,
                            f"native_{frame.side}_timestamp_reference": (
                                artifact.timestamp_reference
                            ),
                        }
                    )
        finally:
            self._release_requests(job.frames)

        if failed:
            fields["native_pair_status"] = "failed:" + ",".join(failed)
            fields["evidence_state"] = "FAILED"
        canonical_ack = json.dumps(
            {
                "request_id": job.request_id,
                "pair_id": job.pair_id,
                "artifacts": ack_rows,
                "state": fields["evidence_state"],
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        fields["evidence_ack_sha256"] = hashlib.sha256(canonical_ack).hexdigest()
        fields["evidence_artifact_count"] = len(ack_rows)
        return {
            "request_id": job.request_id,
            "context": job.context,
            "fields": fields,
        }

    def _remember_result(self, payload: dict[str, object]) -> None:
        request_id = str(payload.get("request_id") or "")
        with self._lock:
            self._pending.discard(request_id)
            self._results[request_id] = payload
            self._result_order.append(request_id)
            while len(self._result_order) > self.MAX_UNACKNOWLEDGED_RESULTS:
                old_id = self._result_order.pop(0)
                self._results.pop(old_id, None)
            fields = payload.get("fields")
            state = (
                str(fields.get("evidence_state") or "FAILED")
                if isinstance(fields, Mapping)
                else "FAILED"
            )
            self._metrics["saved" if state == "SAVED" else "failed"] += 1

    def _run(self) -> None:
        while True:
            job = self._queue.get()
            if job is None:
                return
            try:
                payload = self._persist(job)
            except Exception as exc:  # noqa: BLE001 - release and surface setup failure.
                self._release_requests(job.frames)
                payload = {
                    "request_id": job.request_id,
                    "context": job.context,
                    "fields": {
                        "native_pair_status": "failed",
                        "native_pair_error": f"{type(exc).__name__}: {exc}",
                        "evidence_state": "FAILED",
                    },
                }
            self._remember_result(payload)
            self.completed.emit(payload)

    def result(self, request_id: str) -> dict[str, object] | None:
        with self._lock:
            payload = self._results.get(str(request_id))
            return None if payload is None else dict(payload)

    def acknowledge(self, request_id: str) -> None:
        request = str(request_id)
        with self._lock:
            self._results.pop(request, None)
            try:
                self._result_order.remove(request)
            except ValueError:
                pass

    def close(self) -> bool:
        with self._lock:
            if self._pending:
                return False
            if self._closed:
                return True
            self._closed = True
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            return False
        return True


__all__ = [
    "AsyncNativeEvidenceWriter",
    "EVIDENCE_FRAME_UNAVAILABLE",
    "RingFrameRequest",
    "RingPairJob",
    "RingPairReservation",
    "reserve_ring_pair",
]
