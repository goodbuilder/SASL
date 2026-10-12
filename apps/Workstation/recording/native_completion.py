"""Apply canonical native completions to their original leased row owner.

Request fingerprints enforce first-terminal-wins. The shared primary ledger lock
serializes full-row projection against compact late-primary admission."""
from __future__ import annotations
import copy
import hashlib
import json
import time
from collections.abc import Mapping
from Workstation.workstation_core.measurement_data_store import LiveDataRows, ROW_WRITE_ADMITTED, ROW_WRITE_RETRYABLE, ROW_WRITE_UNAVAILABLE, RowWriteAdmission
from Workstation.workstation_core.session_storage.spool import SpoolRows
from .contracts import (NATIVE_TERMINAL_FINGERPRINT_LIMIT, _NativeRowOwner, _NativeTerminalFingerprint, _PendingNativeCompletion)
from .state import (PrimaryPersistenceLedger, NativeEvidenceState, RecordingRowsState, RecordingSessionState)
from .ports import NativeCompletionPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .native_retry import NativeRetry
    from .native_submission import NativeSubmission
    from .presentation import Presentation
    from .primary_persistence import PrimaryPersistence


class NativeCompletion:
    def __init__(self, *,
        ledger: PrimaryPersistenceLedger,
        native_state: NativeEvidenceState,
        rows: RecordingRowsState,
        session: RecordingSessionState,
        boundary: NativeCompletionPort,
    ) -> None:
        self.ledger = ledger
        self.native_state = native_state
        self.rows = rows
        self.session = session
        self.boundary = boundary

    def bind_collaborators(self, *,
        native_retry: "NativeRetry",
        native_submission: "NativeSubmission",
        presentation: "Presentation",
        primary_persistence: "PrimaryPersistence",
    ) -> None:
        self.native_retry = native_retry
        self.native_submission = native_submission
        self.presentation = presentation
        self.primary_persistence = primary_persistence

    @staticmethod
    def _native_completion_fingerprint(
        *,
        request_id: str,
        context: Mapping[str, object],
        fields: Mapping[str, object],
    ) -> tuple[str, str, dict[str, object]]:
        canonical_fields = dict(fields)
        canonical_fields.pop("native_completion_sha256", None)
        payload = {
            "request_id": str(request_id),
            "context": dict(context),
            "fields": canonical_fields,
        }
        canonical_json = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        return (
            hashlib.sha256(canonical_json.encode("utf-8")).hexdigest(),
            canonical_json,
            payload,
        )

    def _remember_native_terminal_fingerprint(
        self,
        request_id: str,
        fingerprint: _NativeTerminalFingerprint,
    ) -> None:
        self.native_state._native_terminal_fingerprints[request_id] = fingerprint
        self.native_state._native_terminal_fingerprint_order.append(request_id)
        protected = set(self.native_state._pending_native_completions)
        protected.update(self.native_state._deferred_native_frame_results)
        protected.update(self.native_state._native_row_owners)
        protected.add(request_id)
        while (
            len(self.native_state._native_terminal_fingerprints)
            > NATIVE_TERMINAL_FINGERPRINT_LIMIT
        ):
            removed = False
            for _ in range(len(self.native_state._native_terminal_fingerprint_order)):
                candidate = self.native_state._native_terminal_fingerprint_order.popleft()
                if candidate in protected:
                    self.native_state._native_terminal_fingerprint_order.append(candidate)
                    continue
                self.native_state._native_terminal_fingerprints.pop(candidate, None)
                self.native_state._native_conflicts_recorded.discard(candidate)
                removed = True
                break
            if not removed:
                # Persistence-required identities are never evicted merely to
                # satisfy the diagnostic fingerprint cache bound.
                break

    def _finish_native_completion(self, request_id: str) -> None:
        self.native_state._pending_native_completions.pop(request_id, None)
        self.native_state._native_row_owners.pop(request_id, None)
        pending = getattr(
            self.native_state,
            "_recording_pending_native_request_rows",
            None,
        )
        if isinstance(pending, dict):
            pending.pop(request_id, None)

    def _finish_native_completion_from_admitted_row(
        self,
        values: Mapping[str, object],
    ) -> None:
        request_id = str(values.get("native_pair_request_id") or "")
        if not request_id:
            return
        record = self.native_state._pending_native_completions.get(request_id)
        if record is None:
            return
        status = str(values.get("native_pair_status") or "")
        row_sha256 = str(values.get("native_completion_sha256") or "")
        if status == "pending" or not row_sha256:
            return
        if row_sha256 != record.canonical_sha256:
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                logger(
                    "[저장] admitted native completion row fingerprint 불일치; "
                    "first terminal retry ownership을 유지합니다: "
                    f"request={request_id}, expected={record.canonical_sha256}, "
                    f"actual={row_sha256}"
                )
            return
        self.native_state._deferred_native_frame_results.pop(request_id, None)
        self._finish_native_completion(request_id)

    def _append_native_frame_completion_event(
        self,
        *,
        request_id: str,
        context: Mapping[str, object],
        fields: Mapping[str, object],
        force: bool = False,
        event_type: str = "native_frame_pair_persisted",
    ) -> bool:
        if not force and bool(
            getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
        ):
            return False
        writer = getattr(self.boundary, "append_workstation_data", None)
        if not callable(writer):
            return False
        runtime_generation = str(context.get("session_id") or "")
        try:
            accepted = writer(
                "native.frames",
                {
                    **dict(fields),
                    "event_type": str(event_type),
                    "runtime_generation": runtime_generation,
                    "request_id": request_id,
                    "native_request_context": {
                        "owner_runtime_generation": runtime_generation,
                        "request_id": request_id,
                        "record_key": context.get("record_key"),
                        "writer_kind": str(context.get("writer_kind") or "legacy"),
                    },
                },
                priority=True,
            )
        except Exception as exc:  # noqa: BLE001 - evidence failure must not block row completion.
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                logger(f"[저장] native frame completion event 기록 실패: {exc}")
            return False
        return accepted is not False

    def _handle_async_native_frame_result(self, payload: object) -> None:
        data = dict(payload or {})
        retrying = bool(data.pop("_native_completion_retry", False))
        internal_replay = bool(data.pop("_native_completion_internal_replay", False))
        supplied_owner = data.pop("_native_completion_owner", None)
        supplied_owner = (
            supplied_owner if isinstance(supplied_owner, _NativeRowOwner) else None
        )
        request_id = str(data.get("request_id") or "")
        context = data.get("context")
        context = dict(context) if isinstance(context, Mapping) else {}
        session_id = str(context.get("session_id") or "")
        fields = data.get("fields")
        fields = dict(fields) if isinstance(fields, Mapping) else {
            "native_pair_status": "failed",
            "native_pair_error": "async native frame result is malformed",
        }
        event_recorded = bool(data.get("_completion_event_recorded", False))
        try:
            completion_sha256, canonical_json, canonical_payload = (
                self._native_completion_fingerprint(
                    request_id=request_id,
                    context=context,
                    fields=fields,
                )
            )
        except (TypeError, ValueError) as exc:
            fields = {
                "native_pair_status": "failed",
                "native_pair_error": (
                    "async native frame result is not canonical JSON: "
                    f"{type(exc).__name__}: {exc}"
                ),
                "evidence_state": "FAILED",
            }
            completion_sha256, canonical_json, canonical_payload = (
                self._native_completion_fingerprint(
                    request_id=request_id,
                    context=context,
                    fields=fields,
                )
            )
        fields = {
            **fields,
            "native_completion_sha256": completion_sha256,
        }
        store = self.native_submission._native_result_store(context)
        if not request_id:
            self.native_submission._acknowledge_native_result(store, request_id)
            return
        existing_fingerprint = self.native_state._native_terminal_fingerprints.get(request_id)
        if not retrying and not internal_replay and existing_fingerprint is not None:
            if existing_fingerprint.canonical_sha256 == completion_sha256:
                # The writer may deliver the same terminal signal more than
                # once. The first callback already owns projection/persistence.
                self.native_submission._acknowledge_native_result(store, request_id)
                return
            self.native_retry._record_native_completion_conflict(
                request_id=request_id,
                authoritative=existing_fingerprint,
                conflicting_sha256=completion_sha256,
                conflicting_canonical_json=canonical_json,
            )
            self.native_submission._acknowledge_native_result(store, request_id)
            return
        if existing_fingerprint is None:
            existing_fingerprint = _NativeTerminalFingerprint(
                canonical_sha256=completion_sha256,
                canonical_json=canonical_json,
                context=dict(context),
            )
            self._remember_native_terminal_fingerprint(
                request_id,
                existing_fingerprint,
            )
        raw_key = context.get("record_key")
        record_key = tuple(raw_key) if isinstance(raw_key, (tuple, list)) else None
        current_rows = getattr(self.rows, "distance_recording_rows", None)
        rows = current_rows
        pending = getattr(self.native_state, "_recording_pending_native_request_rows", {})
        owner = supplied_owner or self.native_state._native_row_owners.pop(request_id, None)
        if owner is not None:
            rows = owner.rows
            target = (owner.runtime_generation, owner.row_index)
            if isinstance(pending, dict):
                pending.pop(request_id, None)
        else:
            target = pending.pop(request_id, None) if isinstance(pending, dict) else None
        if target is None and rows is not None and request_id:
            for candidate_index, candidate in enumerate(rows):
                if str(candidate.get("native_pair_request_id") or "") == request_id:
                    target = (session_id, candidate_index)
                    break
        if owner is None and target is not None and rows is not None:
            owner = _NativeRowOwner(
                runtime_generation=str(target[0]),
                rows=rows,
                row_index=int(target[1]),
            )
        retry_record = self.native_state._pending_native_completions.get(request_id)
        if retry_record is None:
            retry_record = _PendingNativeCompletion(
                request_id=request_id,
                canonical_sha256=existing_fingerprint.canonical_sha256,
                payload=copy.deepcopy(canonical_payload),
                owner=owner,
                event_recorded=event_recorded,
                first_seen_monotonic_s=time.monotonic(),
            )
        else:
            if owner is not None:
                retry_record.owner = owner
            retry_record.event_recorded = bool(
                retry_record.event_recorded or event_recorded
            )
        self.native_state._pending_native_completions[request_id] = retry_record
        row_admitted = False
        event_attempted = False
        if not event_recorded:
            event_attempted = bool(
                target is None
                or not bool(
                    getattr(
                        self.boundary,
                        "_measurement_session_hierarchy_enabled",
                        True,
                    )
                )
            )
            event_recorded = self._append_native_frame_completion_event(
                request_id=request_id,
                context=context,
                fields=fields,
                force=target is None,
            )
            data["_completion_event_recorded"] = event_recorded
        if target is None and record_key is not None:
            # A non-QObject test adapter can deliver directly from the writer
            # thread before the GUI-side append/upsert binds the request ID.
            # Disk persistence is already complete, so release the store's I/O
            # barrier now. The binding path drains this exact payload if the
            # row is committed; a later row-building exception must never leave
            # session shutdown waiting on work that has already finished.
            data = copy.deepcopy(canonical_payload)
            data["fields"] = dict(fields)
            data["_completion_event_recorded"] = event_recorded
            self.native_state._deferred_native_frame_results[request_id] = data
            retry_record.event_recorded = event_recorded
            if event_recorded:
                self._finish_native_completion(request_id)
            else:
                self.native_retry._defer_native_completion_retry(retry_record)
            self.native_submission._acknowledge_native_result(store, request_id)
            return
        if target is None and record_key is None:
            # Diagnostic/native lookup callers have no Measurement row join.
            # Their NativeFrameStore ledger is already the durable authority;
            # an optional workstation event failure must not create an orphan
            # retry that can never acquire a row owner.
            self._finish_native_completion(request_id)
            self.native_submission._acknowledge_native_result(store, request_id)
            return
        self.ledger._primary_persistence_lock.acquire()
        try:
            if (
                record_key is not None
                and record_key in self.ledger._compact_primary_persisted_keys
            ):
                event_attempted = True
                suppression_event_admitted = self._append_native_frame_completion_event(
                    request_id=request_id,
                    context=context,
                    fields={
                        **fields,
                        "native_row_projection_state": "SUPPRESSED",
                        "native_row_projection_reason": (
                            "compact primary measurement already persisted"
                        ),
                    },
                    force=True,
                    event_type="native_frame_row_projection_suppressed",
                )
                event_recorded = bool(
                    event_recorded or suppression_event_admitted
                )
                retry_record.event_recorded = event_recorded
                if event_recorded:
                    self.native_state._deferred_native_frame_results.pop(request_id, None)
                    self._finish_native_completion(request_id)
                else:
                    self.native_retry._defer_native_completion_retry(retry_record)
                return
            if target is not None and rows is not None:
                bound_session_id, row_index = target
                try:
                    current = dict(rows[row_index])
                except (IndexError, KeyError, TypeError):
                    current = {}
                current_request_id = str(
                    current.get("native_pair_request_id") or ""
                )
                if (
                    current
                    and bound_session_id == session_id
                    and current_request_id in {"", request_id}
                ):
                    row_needed_primary_retry = bool(
                        isinstance(rows, LiveDataRows)
                        and row_index
                        not in getattr(rows, "_admitted_indices", set())
                    )
                    current.update(fields)
                    atomic_replacer = getattr(rows, "replace_with_admission", None)
                    if callable(atomic_replacer):
                        if owner is None or owner.row_lease is None:
                            # An atomic row owner without its append lease cannot
                            # prove that this numeric index still belongs to the
                            # completed request.  Preserve the completion as a
                            # canonical native.frames event and abandon the row
                            # projection instead of permitting an ABA overwrite.
                            replacement = RowWriteAdmission(
                                ROW_WRITE_UNAVAILABLE,
                                "native row append lease is unavailable",
                                row_index=row_index,
                            )
                        else:
                            replacement = atomic_replacer(
                                row_index,
                                current,
                                lease=owner.row_lease,
                            )
                        replacement_state = str(
                            getattr(replacement, "state", "") or ""
                        ).upper()
                        row_admitted = replacement_state == ROW_WRITE_ADMITTED
                        if (
                            row_admitted
                            and record_key is not None
                            and row_needed_primary_retry
                            and record_key in self.ledger._primary_retry_required_keys
                        ):
                            self.primary_persistence._remember_native_primary_persisted(record_key)
                        if replacement_state == ROW_WRITE_UNAVAILABLE:
                            projection_fields = {
                                **fields,
                                "native_row_projection_state": ROW_WRITE_UNAVAILABLE,
                                "native_row_projection_reason": str(
                                    getattr(replacement, "reason", "")
                                    or "native row owner is unavailable"
                                ),
                                "native_row_expected_lease": (
                                    None
                                    if owner is None or owner.row_lease is None
                                    else list(owner.row_lease)
                                ),
                                "native_row_current_lease": (
                                    None
                                    if getattr(replacement, "lease", None) is None
                                    else list(replacement.lease)
                                ),
                            }
                            if event_recorded:
                                event_attempted = True
                                self._append_native_frame_completion_event(
                                    request_id=request_id,
                                    context=context,
                                    fields=projection_fields,
                                    force=True,
                                    event_type=(
                                        "native_frame_row_projection_unavailable"
                                    ),
                                )
                            else:
                                event_attempted = True
                                event_recorded = (
                                    self._append_native_frame_completion_event(
                                        request_id=request_id,
                                        context=context,
                                        fields=projection_fields,
                                        force=True,
                                    )
                                )
                            logger = getattr(self.boundary, "_log", None)
                            if callable(logger):
                                logger(
                                    "[저장] native frame row projection 포기: "
                                    f"{projection_fields['native_row_projection_reason']}"
                                )
                        elif replacement_state == ROW_WRITE_RETRYABLE:
                            if not event_recorded:
                                event_attempted = True
                                event_recorded = (
                                    self._append_native_frame_completion_event(
                                        request_id=request_id,
                                        context=context,
                                        fields={
                                            **fields,
                                            "native_row_projection_state": (
                                                ROW_WRITE_RETRYABLE
                                            ),
                                            "native_row_projection_reason": str(
                                                getattr(replacement, "reason", "")
                                                or "native row projection enqueue was not admitted"
                                            ),
                                            "native_row_expected_lease": (
                                                None
                                                if owner is None
                                                or owner.row_lease is None
                                                else list(owner.row_lease)
                                            ),
                                            "native_row_current_lease": (
                                                None
                                                if getattr(replacement, "lease", None)
                                                is None
                                                else list(replacement.lease)
                                            ),
                                        },
                                        force=True,
                                    )
                                )
                            logger = getattr(self.boundary, "_log", None)
                            if callable(logger):
                                logger(
                                    "[저장] native frame row projection enqueue 지연; "
                                    "native.frames completion event를 권위 증거로 유지합니다."
                                )
                        elif rows is current_rows:
                            refresher = getattr(
                                self.presentation,
                                "_refresh_recording_table",
                                None,
                            )
                            if callable(refresher):
                                refresher()
                    elif isinstance(rows, (SpoolRows, LiveDataRows)):
                        row_admitted = bool(rows.replace(row_index, current))
                        refresher = getattr(self.presentation, "_refresh_recording_table", None)
                        if row_admitted and rows is current_rows and callable(refresher):
                            refresher()
                    else:
                        rows[row_index] = current
                        row_admitted = True
                        refresher = getattr(self.presentation, "_refresh_recording_table", None)
                        if rows is current_rows and callable(refresher):
                            refresher()
            if not row_admitted and not event_recorded and not event_attempted:
                event_attempted = True
                event_recorded = self._append_native_frame_completion_event(
                    request_id=request_id,
                    context=context,
                    fields={
                        **fields,
                        "native_row_projection_state": ROW_WRITE_UNAVAILABLE,
                        "native_row_projection_reason": (
                            "native row target is missing or no longer owns request"
                        ),
                    },
                    force=True,
                )
            if str(fields.get("native_pair_status") or "").startswith(("failed", "missing", "queue")):
                logger = getattr(self.boundary, "_log", None)
                if callable(logger):
                    logger(
                        "[저장] native frame 비동기 저장 실패/누락: "
                        f"{fields.get('native_pair_status')} "
                        f"{fields.get('native_pair_error', '')}"
                    )
        finally:
            self.ledger._primary_persistence_lock.release()
            retained = self.native_state._pending_native_completions.get(request_id)
            if retained is not None:
                retained.event_recorded = bool(
                    retained.event_recorded or event_recorded
                )
                if row_admitted or retained.event_recorded:
                    self._finish_native_completion(request_id)
                elif retained.next_retry_monotonic_s <= time.monotonic():
                    self.native_retry._defer_native_completion_retry(retained)
            self.native_submission._acknowledge_native_result(store, request_id)

    def _track_pending_native_row(
        self,
        row_index: int,
        values: Mapping[str, object],
        *,
        row_lease: tuple[int, int] | None = None,
    ) -> None:
        """Bind one in-flight native request to its permanent append-only row.

        ``_recording_pair_row_index`` is LRU-pruned at
        ``RECORDING_KEY_CACHE_LIMIT``. Request IDs are unique and remain bound
        until their own completion, so repeated writes for one pair cannot
        consume each other's lookup or overwrite a newer pending request.
        """

        request_id = str(values.get("native_pair_request_id") or "")
        if not request_id:
            return
        pending = getattr(
            self.native_state,
            "_recording_pending_native_request_rows",
            None,
        )
        if not isinstance(pending, dict):
            return
        if str(values.get("native_pair_status") or "") == "pending":
            session_id = str(
                values.get("session_id")
                or getattr(self.session, "recording_session_id", "")
                or ""
            )
            pending[request_id] = (session_id, row_index)
            rows = getattr(self.rows, "distance_recording_rows", None)
            row_owner = None
            if rows is not None:
                row_owner = _NativeRowOwner(
                    runtime_generation=session_id,
                    rows=rows,
                    row_index=int(row_index),
                    row_lease=row_lease,
                )
                self.native_state._native_row_owners[request_id] = row_owner
            deferred = self.native_state._deferred_native_frame_results.pop(request_id, None)
            if deferred is not None:
                deferred = dict(deferred)
                deferred["_native_completion_internal_replay"] = True
                deferred["_native_completion_owner"] = row_owner
                self._handle_async_native_frame_result(deferred)
            else:
                retry_record = self.native_state._pending_native_completions.get(request_id)
                if retry_record is not None and row_owner is not None:
                    retry_record.owner = row_owner
                    retry_record.next_retry_monotonic_s = 0.0
                    retry_payload = copy.deepcopy(retry_record.payload)
                    retry_payload["_native_completion_retry"] = True
                    retry_payload["_native_completion_owner"] = row_owner
                    retry_payload["_completion_event_recorded"] = (
                        retry_record.event_recorded
                    )
                    self._handle_async_native_frame_result(retry_payload)
        else:
            pending.pop(request_id, None)
            self.native_state._native_row_owners.pop(request_id, None)
