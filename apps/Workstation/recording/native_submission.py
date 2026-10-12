"""Own exact native-frame writer admission and per-session writer lifetimes.

FrameKey identity is mandatory; missing pixels never use nearest-frame substitution.
The native completion collaborator retains independent durable retry ownership."""
from __future__ import annotations
import re
import uuid
from collections.abc import Mapping
from pathlib import Path
from PyQt5 import QtCore, QtGui
from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from Workstation.workstation_core.native_frame_store import AsyncNativeFrameStore
from Workstation.workstation_core.native_evidence_writer import AsyncNativeEvidenceWriter, EVIDENCE_FRAME_UNAVAILABLE, RingPairReservation
from Workstation.workstation_core.native_frame_ring import FrameKey
from .contracts import (NATIVE_COMPLETION_FAST_RETRY_LIMIT, NATIVE_COMPLETION_PENDING_SOFT_LIMIT, NATIVE_COMPLETION_RETRY_INTERVAL_MS, NATIVE_COMPLETION_SLOW_RETRY_INTERVAL_MS)
from .state import (NativeEvidenceState, RecordingSessionState)
from .ports import NativeSubmissionPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .native_completion import NativeCompletion
    from .native_retry import NativeRetry


class NativeSubmission:
    def __init__(self, *,
        native_state: NativeEvidenceState,
        session: RecordingSessionState,
        boundary: NativeSubmissionPort,
    ) -> None:
        self.native_state = native_state
        self.session = session
        self.boundary = boundary

    def bind_collaborators(self, *,
        native_completion: "NativeCompletion",
        native_retry: "NativeRetry",
    ) -> None:
        self.native_completion = native_completion
        self.native_retry = native_retry

    def _persist_native_detection_pair(
        self,
        *,
        left_detection: object,
        right_detection: object,
        pair_id: str,
        purpose: str,
        record_key: tuple[object, ...] | None = None,
        reservation: RingPairReservation | None = None,
    ) -> dict[str, object]:
        """Persist evaluated-pair source pixels without resizing or overlays.

        The producer's OpenCV/native image coordinates are retained.  Missing
        cached frames do not block raw attempt recording; the explicit status
        lets offline analysis distinguish absence from a hash/path mismatch.
        """

        if not self.session.recording_session_id:
            if reservation is not None:
                reservation.release()
            return {"native_pair_status": "session_id_missing"}
        resolver = getattr(self.boundary, "_recording_session_dir", None)
        if not callable(resolver):
            if reservation is not None:
                reservation.release()
            return {"native_pair_status": "session_directory_owner_missing"}
        session_id = str(self.session.recording_session_id)
        root = Path(resolver(session_id))
        exact_ring_frames: list[
            tuple[str, object, FrameKey, Mapping[str, object]]
        ] = []
        exact_ring_mode = True
        frames: list[tuple[str, QtGui.QImage, Mapping[str, object]]] = []
        missing: list[str] = []
        for side, detection in (
            ("left", left_detection),
            ("right", right_detection),
        ):
            pane = getattr(self.boundary, f"{side}_pane", None)
            ring_getter = getattr(pane, "native_ring_for_frame_key", None)
            detection_metadata = getattr(detection, "timestamp_metadata", {})
            detection_metadata = (
                dict(detection_metadata)
                if isinstance(detection_metadata, Mapping)
                else {}
            )
            try:
                frame_key = FrameKey.from_mapping(detection_metadata)
            except (TypeError, ValueError):
                frame_key = FrameKey("", 0, 0)
            if callable(ring_getter):
                ring = (
                    ring_getter(frame_key)
                    if frame_key.camera_id
                    and frame_key.stream_generation > 0
                    and frame_key.frame_id > 0
                    and detection_metadata.get("native_ring_available") is not False
                    else None
                )
                if ring is None:
                    missing.append(side)
                    continue
                exact_ring_frames.append(
                    (side, ring, frame_key, detection_metadata)
                )
                continue

            exact_ring_mode = False
            legacy_getter = getattr(pane, "native_measurement_frame", None)
            legacy_cached = (
                legacy_getter(detection.app_receive_monotonic_s)
                if callable(legacy_getter)
                else None
            )
            cached = (
                None
                if legacy_cached is None
                else (legacy_cached[1], legacy_cached[2])
            )
            if cached is None:
                missing.append(side)
                continue
            image, metadata = cached
            frames.append((side, image, dict(metadata)))

        if exact_ring_mode and len(exact_ring_frames) != 2:
            missing = missing or ["left", "right"]
        if missing:
            if reservation is not None:
                reservation.release()
            return {
                "native_pair_status": EVIDENCE_FRAME_UNAVAILABLE,
                "native_pair_error": (
                    "exact FrameKey is absent or was overwritten; nearest-frame "
                    "substitution is forbidden"
                ),
                "native_pair_missing_sides": ",".join(dict.fromkeys(missing)),
                "evidence_state": "REJECTED",
            }

        if len(self.native_state._pending_native_completions) >= NATIVE_COMPLETION_PENDING_SOFT_LIMIT:
            if reservation is not None:
                reservation.release()
            return {
                "native_pair_status": "completion_retry_backpressure",
                "native_pair_error": (
                    "native completion persistence retry ownership is at capacity"
                ),
                "evidence_state": "FAILED",
            }
        request_id = f"native-{uuid.uuid4().hex}"
        context = {
            "session_id": session_id,
            "record_key": list(record_key) if record_key is not None else None,
            "writer_kind": "ring" if exact_ring_mode else "legacy",
            "native_root": str(root),
            "native_events_path": str(self._native_events_path(root)),
        }
        context_reader = getattr(self.boundary, "measurement_storage_context", None)
        if callable(context_reader):
            context.update(context_reader(session_id))
        if exact_ring_mode:
            writer = self._native_evidence_writer_for_session(session_id, root)
            if reservation is not None:
                expected_keys = tuple(
                    frame_key for _side, _ring, frame_key, _metadata in exact_ring_frames
                )
                if reservation.frame_keys != expected_keys:
                    reservation.release()
                    return {
                        "native_pair_status": EVIDENCE_FRAME_UNAVAILABLE,
                        "native_pair_error": (
                            "pre-pinned pair reservation does not match selected FrameKeys"
                        ),
                        "evidence_state": "REJECTED",
                    }
                return writer.enqueue_reserved_pair(
                    request_id=request_id,
                    pair_id=pair_id,
                    purpose=purpose,
                    reservation=reservation,
                    context=context,
                )
            return writer.enqueue_ring_pair(
                request_id=request_id,
                pair_id=pair_id,
                purpose=purpose,
                frames=exact_ring_frames,
                context=context,
            )

        if reservation is not None:
            reservation.release()
        store = self._native_frame_store_for_session(session_id, root)
        scheduled = store.enqueue_pair(
            request_id=request_id,
            pair_id=pair_id,
            purpose=purpose,
            frames=frames,
            missing_sides=(),
            context=context,
        )
        if scheduled is None:
            return {
                "native_pair_status": "queue_full",
                "native_pair_error": (
                    f"native frame queue limit {store.max_pending} reached"
                ),
                "evidence_state": "FAILED",
            }
        return scheduled

    def _native_frame_store_for_session(
        self,
        session_id: str,
        root: Path,
    ) -> AsyncNativeFrameStore:
        stores = getattr(self.native_state, "_native_frame_async_stores", None)
        if not isinstance(stores, dict):
            stores = {}
            self.native_state._native_frame_async_stores = stores
        store_key = (
            "measurement-data"
            if not bool(
                getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
            )
            else session_id
        )
        store = stores.get(store_key)
        if isinstance(store, AsyncNativeFrameStore):
            return store
        parent = self.boundary.qobject_parent if isinstance(self.boundary.qobject_parent, QtCore.QObject) else None
        store = AsyncNativeFrameStore(
            root,
            timestamp_filenames=not bool(
                getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
            ),
            events_path=self._native_events_path(root),
            event_sink=self._persist_native_artifact_record,
            parent=parent,
        )
        callback = getattr(self.boundary, "_handle_async_native_frame_result", None)
        if callable(callback):
            store.completed.connect(callback, QtCore.Qt.QueuedConnection)
        else:
            store.completed.connect(
                self.native_completion._handle_async_native_frame_result,
                QtCore.Qt.DirectConnection,
            )
        stores[store_key] = store
        return store

    def _native_evidence_writer_for_session(
        self,
        session_id: str,
        root: Path,
    ) -> AsyncNativeEvidenceWriter:
        writers = getattr(self.native_state, "_native_evidence_writers", None)
        if not isinstance(writers, dict):
            writers = {}
            self.native_state._native_evidence_writers = writers
        writer_key = (
            "measurement-data"
            if not bool(
                getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
            )
            else session_id
        )
        writer = writers.get(writer_key)
        if isinstance(writer, AsyncNativeEvidenceWriter):
            return writer
        parent = self.boundary.qobject_parent if isinstance(self.boundary.qobject_parent, QtCore.QObject) else None
        writer = AsyncNativeEvidenceWriter(
            root,
            timestamp_filenames=not bool(
                getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
            ),
            events_path=self._native_events_path(root),
            event_sink=self._persist_native_artifact_record,
            parent=parent,
        )
        callback = getattr(self.boundary, "_handle_async_native_frame_result", None)
        if callable(callback):
            writer.completed.connect(callback, QtCore.Qt.QueuedConnection)
        else:
            writer.completed.connect(
                self.native_completion._handle_async_native_frame_result,
                QtCore.Qt.DirectConnection,
            )
        writers[writer_key] = writer
        return writer

    def _persist_native_artifact_record(self, payload: Mapping[str, object], context: Mapping[str, object]) -> bool:
        writer = getattr(self.boundary, "append_workstation_data", None)
        if not callable(writer):
            return False
        return bool(writer("native.frames", {
            **dict(payload), "measurement_id": str(context.get("measurement_id") or ""),
            "event_type": "native_frame_saved",
        }, priority=True))

    def _native_events_path(self, root: Path) -> Path:
        """Use the timestamped native ledger selected by the data store when co-located."""

        if re.fullmatch(r"\d{8}_\d{6}_\d{9}\+0900(?:-\d+)?", root.name):
            return root / "raw" / f"{root.name}_native_frames.jsonl"
        candidate: Path | None = None
        path_provider = getattr(self.boundary, "native_frames_path", None)
        if callable(path_provider):
            try:
                selected_path = path_provider()
                if selected_path is not None:
                    candidate = Path(selected_path)
            except (OSError, RuntimeError, TypeError, ValueError):
                candidate = None
        if candidate is None:
            return root / "native_frames.jsonl"
        try:
            candidate.relative_to(root)
        except ValueError:
            return root / candidate.name
        return candidate

    def _native_result_store(
        self,
        context: Mapping[str, object],
    ) -> AsyncNativeFrameStore | AsyncNativeEvidenceWriter | None:
        writer_kind = str(context.get("writer_kind") or "legacy")
        stores = getattr(
            self.native_state,
            "_native_evidence_writers"
            if writer_kind == "ring"
            else "_native_frame_async_stores",
            None,
        )
        session_id = str(context.get("session_id") or "")
        store_key = (
            "measurement-data"
            if not bool(
                getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
            )
            else session_id
        )
        store = stores.get(store_key) if isinstance(stores, dict) else None
        return (
            store
            if isinstance(store, (AsyncNativeFrameStore, AsyncNativeEvidenceWriter))
            else None
        )

    @staticmethod
    def _acknowledge_native_result(
        store: AsyncNativeFrameStore | AsyncNativeEvidenceWriter | None,
        request_id: str,
    ) -> None:
        if store is not None:
            store.acknowledge(request_id)

    @staticmethod
    def _native_completion_retry_delay_s(attempts: int) -> float:
        interval_ms = (
            NATIVE_COMPLETION_RETRY_INTERVAL_MS
            if attempts < NATIVE_COMPLETION_FAST_RETRY_LIMIT
            else NATIVE_COMPLETION_SLOW_RETRY_INTERVAL_MS
        )
        return interval_ms / 1000.0

    def native_frame_persistence_pending_count(self) -> int:
        self.native_retry._retry_pending_native_completions()
        pending = 0
        for attribute, writer_type in (
            ("_native_frame_async_stores", AsyncNativeFrameStore),
            ("_native_evidence_writers", AsyncNativeEvidenceWriter),
        ):
            stores = getattr(self.native_state, attribute, None)
            if not isinstance(stores, dict):
                continue
            pending += sum(
                store.pending_count
                for store in stores.values()
                if isinstance(store, writer_type)
            )
        return (
            pending
            + len(self.native_state._pending_native_completions)
            + len(self.native_state._pending_native_conflicts)
        )

    def _shutdown_native_frame_stores(self) -> bool:
        # Application shutdown may outlive the Qt retry timer. Force one bounded
        # retry slice per lifecycle pass so permanent rejection reaches the
        # explicit terminal-drop state instead of blocking close forever.
        self.native_retry._retry_pending_native_completions(force=True)
        all_closed = True
        for attribute, writer_type in (
            ("_native_frame_async_stores", AsyncNativeFrameStore),
            ("_native_evidence_writers", AsyncNativeEvidenceWriter),
        ):
            stores = getattr(self.native_state, attribute, None)
            if not isinstance(stores, dict):
                continue
            for session_id, store in list(stores.items()):
                persistence_required = any(
                    str(session_id) == "measurement-data"
                    or str(
                        record.payload.get("context", {}).get("session_id", "")
                        if isinstance(record.payload.get("context"), Mapping)
                        else ""
                    )
                    == str(session_id)
                    for record in self.native_state._pending_native_completions.values()
                )
                persistence_required = persistence_required or any(
                    str(session_id) == "measurement-data"
                    or str(record.context.get("session_id") or "")
                    == str(session_id)
                    for record in self.native_state._pending_native_conflicts.values()
                )
                if persistence_required:
                    all_closed = False
                    continue
                if not isinstance(store, writer_type) or store.close():
                    stores.pop(session_id, None)
                    orphaned = []
                    for request_id, payload in self.native_state._deferred_native_frame_results.items():
                        context = payload.get("context")
                        context = (
                            dict(context) if isinstance(context, Mapping) else {}
                        )
                        if request_id in self.native_state._pending_native_completions:
                            continue
                        if (
                            str(session_id) == "measurement-data"
                            or str(context.get("session_id") or "")
                            == str(session_id)
                        ):
                            orphaned.append(request_id)
                    for request_id in orphaned:
                        self.native_state._deferred_native_frame_results.pop(request_id, None)
                    if orphaned:
                        logger = getattr(self.boundary, "_log", None)
                        if callable(logger):
                            logger(
                                "[저장] 행 등록 전 완료된 native frame 결과를 "
                                f"세션 종료 시 정리했습니다: {len(orphaned)} requests"
                            )
                    if isinstance(store, QtCore.QObject):
                        store.deleteLater()
                else:
                    all_closed = False
        return all_closed

    def _persist_native_measurement_pair(
        self,
        measurement_attempt: MeasurementAttempt,
    ) -> dict[str, object]:
        return self._persist_native_detection_pair(
            left_detection=measurement_attempt.left_detection,
            right_detection=measurement_attempt.right_detection,
            pair_id=f"pair-{measurement_attempt.attempt_id}",
            purpose="accepted-measurement",
            record_key=tuple(measurement_attempt.detection_pair_key),
        )
