"""Own primary-row admission, leases, deduplication, and late compact handshakes.

One PrimaryPersistenceLedger is shared with native completion. Protected keys
survive cache pruning until their actual persistence handshake is consumed."""
from __future__ import annotations
import json
from collections.abc import Mapping
from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from Workstation.workstation_core.measurement_data_store import ROW_WRITE_ADMITTED, ROW_WRITE_RETRYABLE, ROW_WRITE_UNAVAILABLE, RowWriteAdmission
from .contracts import (PairRecordingCommitResult, RECORDING_KEY_CACHE_LIMIT)
from .state import (PrimaryPersistenceLedger, NativeEvidenceState, RecordingRowsState, RecordingSessionState)
from .ports import PrimaryPersistencePort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .native_completion import NativeCompletion
    from .presentation import Presentation


class PrimaryPersistence:
    def __init__(self, *,
        ledger: PrimaryPersistenceLedger,
        native_state: NativeEvidenceState,
        rows: RecordingRowsState,
        session: RecordingSessionState,
        boundary: PrimaryPersistencePort,
    ) -> None:
        self.ledger = ledger
        self.native_state = native_state
        self.rows = rows
        self.session = session
        self.boundary = boundary

    def bind_collaborators(self, *,
        native_completion: "NativeCompletion",
        presentation: "Presentation",
    ) -> None:
        self.native_completion = native_completion
        self.presentation = presentation

    def _append_measurement_row_direct(
        self,
        values: Mapping[str, object],
        *,
        reason: str,
        persistence_mode: str = "direct_owner_unavailable",
    ) -> RowWriteAdmission:
        writer = getattr(self.boundary, "append_workstation_data", None)
        if not callable(writer):
            return RowWriteAdmission(
                ROW_WRITE_UNAVAILABLE,
                reason or "measurement row owner and direct writer are unavailable",
            )
        record = {
            **dict(values),
            "row_persistence_mode": str(persistence_mode),
            "row_owner_unavailable_reason": str(reason or "row owner unavailable"),
        }
        try:
            admitted = writer("measurements", record, priority=True)
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            return RowWriteAdmission(
                ROW_WRITE_RETRYABLE,
                f"direct measurement enqueue failed: {exc}",
            )
        if admitted is False:
            return RowWriteAdmission(
                ROW_WRITE_RETRYABLE,
                "direct measurement enqueue was not admitted",
            )
        return RowWriteAdmission(ROW_WRITE_ADMITTED)

    def _persist_compact_primary_measurement(
        self,
        record_key: tuple[object, ...],
        record: Mapping[str, object],
    ) -> bool:
        """Serialize late compact evidence against native full-row projection."""

        key = tuple(record_key)
        with self.ledger._primary_persistence_lock:
            if self._consume_native_primary_persisted(key):
                return True
            admission = self._append_measurement_row_direct(
                record,
                reason="primary row retired before background retry admission",
                persistence_mode=str(
                    record.get("primary_persistence_mode")
                    or "late_direct_retry"
                ),
            )
            if admission.admitted:
                self._remember_compact_primary_persisted(key)
                self.ledger._primary_retry_required_keys.discard(key)
            return admission.admitted

    def _release_primary_retry_ownership(
        self,
        record_key: tuple[object, ...],
    ) -> None:
        """Release controller-side handshake after explicit View overflow/drop."""

        key = tuple(record_key)
        with self.ledger._primary_persistence_lock:
            self.ledger._primary_retry_required_keys.discard(key)
            self.ledger._pending_primary_record_rows.pop(key, None)
            self._consume_native_primary_persisted(key)

    def _remember_pending_primary_row(
        self,
        key: tuple[object, ...],
        row: Mapping[str, object],
    ) -> None:
        """Retain one already-built row for side-effect-free admission retry."""

        self.ledger._pending_primary_record_rows.pop(key, None)
        self.ledger._pending_primary_record_rows[key] = dict(row)
        while len(self.ledger._pending_primary_record_rows) > RECORDING_KEY_CACHE_LIMIT:
            oldest = next(iter(self.ledger._pending_primary_record_rows))
            self.ledger._pending_primary_record_rows.pop(oldest, None)

    def _remember_compact_primary_persisted(
        self,
        key: tuple[object, ...],
    ) -> None:
        """Keep projection suppression terminal and bounded across generations."""

        if key in self.ledger._compact_primary_persisted_keys:
            return
        self.ledger._compact_primary_persisted_keys.add(key)
        self.ledger._compact_primary_persisted_order.append(key)
        protected: set[tuple[object, ...]] = set()
        for pending in self.native_state._pending_native_completions.values():
            context = pending.payload.get("context")
            raw_key = (
                context.get("record_key")
                if isinstance(context, Mapping)
                else None
            )
            if isinstance(raw_key, (tuple, list)):
                protected.add(tuple(raw_key))
        for owner in self.native_state._native_row_owners.values():
            try:
                row = owner.rows[owner.row_index]  # type: ignore[index]
                raw_key = json.loads(
                    str(row.get("measurement_detection_pair_key") or "null")
                )
            except (AttributeError, IndexError, KeyError, TypeError, ValueError):
                continue
            if isinstance(raw_key, list):
                protected.add(tuple(raw_key))
        while (
            len(self.ledger._compact_primary_persisted_order)
            > RECORDING_KEY_CACHE_LIMIT
        ):
            evicted: tuple[object, ...] | None = None
            for _ in range(len(self.ledger._compact_primary_persisted_order)):
                candidate = self.ledger._compact_primary_persisted_order.popleft()
                if candidate in protected:
                    self.ledger._compact_primary_persisted_order.append(candidate)
                    continue
                self.ledger._compact_primary_persisted_keys.discard(candidate)
                evicted = candidate
                break
            if evicted is None:
                # Active native owners are persistence state, not diagnostics;
                # temporarily exceed the cache bound rather than reopen an ABA
                # projection after compact primary evidence was already admitted.
                break
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                logger(
                    "[측정] compact primary suppression terminal cache eviction: "
                    f"key={evicted}"
                )

    def _remember_native_primary_persisted(
        self,
        key: tuple[object, ...],
    ) -> None:
        if key in self.ledger._native_primary_persisted_keys:
            return
        self.ledger._native_primary_persisted_keys.add(key)
        self.ledger._native_primary_persisted_order.append(key)
        while len(self.ledger._native_primary_persisted_order) > RECORDING_KEY_CACHE_LIMIT:
            evicted: tuple[object, ...] | None = None
            for _ in range(len(self.ledger._native_primary_persisted_order)):
                candidate = self.ledger._native_primary_persisted_order.popleft()
                if candidate in self.ledger._primary_retry_required_keys:
                    self.ledger._native_primary_persisted_order.append(candidate)
                    continue
                self.ledger._native_primary_persisted_keys.discard(candidate)
                evicted = candidate
                break
            if evicted is None:
                # A full row admission remains a live handshake until the View
                # consumes that primary retry. Never turn a size bound into a
                # false "not persisted" result and duplicate compact evidence.
                break

    def _consume_native_primary_persisted(
        self,
        key: tuple[object, ...],
    ) -> bool:
        if key not in self.ledger._native_primary_persisted_keys:
            return False
        self.ledger._native_primary_persisted_keys.discard(key)
        try:
            self.ledger._native_primary_persisted_order.remove(key)
        except ValueError:
            pass
        self.ledger._primary_retry_required_keys.discard(key)
        return True

    def _pending_primary_retry_row(
        self,
        key: tuple[object, ...],
    ) -> dict[str, object] | None:
        cached = self.ledger._pending_primary_record_rows.get(key)
        if cached is None:
            return None
        retry_row = dict(cached)
        deferred_projection: dict[str, object] = {}
        cached_request_id = str(cached.get("native_pair_request_id") or "")
        if cached_request_id:
            deferred = self.native_state._deferred_native_frame_results.pop(
                cached_request_id,
                None,
            )
            deferred_fields = (
                deferred.get("fields") if isinstance(deferred, Mapping) else None
            )
            if isinstance(deferred_fields, Mapping):
                deferred_projection = dict(deferred_fields)
                retry_row.update(deferred_projection)
        row_index = self.ledger._recording_pair_row_index.get(key)
        if row_index is None:
            return retry_row
        try:
            current = dict(self.rows.distance_recording_rows[row_index])
        except (IndexError, KeyError, TypeError):
            return retry_row
        current_request_id = str(current.get("native_pair_request_id") or "")
        if cached_request_id and current_request_id == cached_request_id:
            # Preserve a terminal async native-frame projection that completed
            # after the first row enqueue was rejected.
            current.update(deferred_projection)
            return current
        return retry_row

    def _upsert_pair_recording_row(
        self,
        measurement_attempt: MeasurementAttempt,
        values: dict[str, object],
    ) -> PairRecordingCommitResult:
        if not self.session.distance_recording_enabled:
            return PairRecordingCommitResult(
                inserted=False,
                enqueue_admitted=False,
                row_index=-1,
                admission_state=ROW_WRITE_UNAVAILABLE,
                admission_reason="distance recording is not active",
            )
        key = tuple(measurement_attempt.detection_pair_key)
        row_index = self.ledger._recording_pair_row_index.get(key)
        owner = self.rows.distance_recording_rows
        persisted_values: Mapping[str, object] = values
        if row_index is None:
            row_index = len(owner)
            values["sample_index"] = row_index
            values["pair_sequence"] = row_index
            content_sha256 = values.pop("measurement_attempt_content_sha256", None)
            instance_sha256 = values.pop("measurement_attempt_instance_sha256", None)
            values["measurement_attempt_content_sha256"] = content_sha256
            values["measurement_attempt_instance_sha256"] = instance_sha256
            atomic_appender = getattr(owner, "append_with_admission", None)
            if callable(atomic_appender):
                admission = atomic_appender(values)
            else:
                appender = getattr(owner, "append", None)
                if not callable(appender):
                    admission = RowWriteAdmission(
                        ROW_WRITE_UNAVAILABLE,
                        "measurement row owner has no append contract",
                    )
                else:
                    try:
                        appended = appender(values)
                    except (OSError, RuntimeError, TypeError, ValueError) as exc:
                        admission = RowWriteAdmission(
                            ROW_WRITE_UNAVAILABLE,
                            f"legacy measurement append failed: {exc}",
                        )
                    else:
                        admission = RowWriteAdmission(
                            ROW_WRITE_ADMITTED if appended is not False else ROW_WRITE_RETRYABLE,
                            "" if appended is not False else "legacy measurement append was not admitted",
                            row_index=row_index,
                        )
            owner_row_index = admission.row_index
            if owner_row_index is None:
                owner_row_index = row_index
            owner_has_row = admission.state in {
                ROW_WRITE_ADMITTED,
                ROW_WRITE_RETRYABLE,
            }
            if owner_has_row:
                row_index = int(owner_row_index)
                self.ledger._recording_pair_row_index[key] = row_index
                if admission.lease is not None:
                    self.ledger._pair_recording_row_leases[key] = admission.lease
                self.rows.distance_recording_keys.append(key)
                if len(self.rows.distance_recording_keys) > RECORDING_KEY_CACHE_LIMIT:
                    self.rows.distance_recording_keys.pop(0)
                minimum_cached_index = max(
                    0, len(owner) - RECORDING_KEY_CACHE_LIMIT
                )
                self.ledger._recording_pair_row_index = {
                    cached_key: cached_index
                    for cached_key, cached_index in self.ledger._recording_pair_row_index.items()
                    if cached_index >= minimum_cached_index
                }
                self.ledger._pair_recording_row_leases = {
                    cached_key: lease
                    for cached_key, lease in self.ledger._pair_recording_row_leases.items()
                    if cached_key in self.ledger._recording_pair_row_index
                }
                self.native_completion._track_pending_native_row(
                    row_index,
                    values,
                    row_lease=admission.lease,
                )
                self.presentation._append_recording_table_row(values)
                inserted = True
                direct_persistence = False
            else:
                self.ledger._recording_pair_row_index.pop(key, None)
                self.ledger._pair_recording_row_leases.pop(key, None)
                direct = self._append_measurement_row_direct(
                    values,
                    reason=admission.reason,
                )
                admission = direct
                row_index = -1
                inserted = False
                direct_persistence = direct.admitted
        else:
            try:
                current = dict(owner[row_index])
            except (IndexError, KeyError, TypeError):
                # The owner may have been cleared between the retryable append
                # and this bundle fallback.  Let replace_with_admission classify
                # the stale index/lease atomically so the canonical row can be
                # rerouted to direct persistence instead of escaping as an
                # indexing exception.
                current = {}
            sample_index = current.get("sample_index", row_index)
            pair_sequence = current.get("pair_sequence", row_index)
            replacement = current
            replacement.update(values)
            replacement["sample_index"] = sample_index
            replacement["pair_sequence"] = pair_sequence
            persisted_values = replacement
            atomic_replacer = getattr(owner, "replace_with_admission", None)
            if callable(atomic_replacer):
                admission = atomic_replacer(
                    row_index,
                    replacement,
                    lease=self.ledger._pair_recording_row_leases.get(key),
                )
            else:
                replacer = getattr(owner, "replace", None)
                try:
                    replaced = (
                        replacer(row_index, replacement)
                        if callable(replacer)
                        else owner.__setitem__(row_index, replacement)
                    )
                except (IndexError, OSError, RuntimeError, TypeError, ValueError) as exc:
                    admission = RowWriteAdmission(
                        ROW_WRITE_UNAVAILABLE,
                        f"legacy measurement replacement failed: {exc}",
                        row_index=row_index,
                    )
                else:
                    admission = RowWriteAdmission(
                        ROW_WRITE_ADMITTED if replaced is not False else ROW_WRITE_RETRYABLE,
                        "" if replaced is not False else "legacy measurement replacement was not admitted",
                        row_index=row_index,
                    )
            if admission.state == ROW_WRITE_UNAVAILABLE:
                self.ledger._recording_pair_row_index.pop(key, None)
                self.ledger._pair_recording_row_leases.pop(key, None)
                direct_values = dict(values)
                direct_values["sample_index"] = sample_index
                direct_values["pair_sequence"] = pair_sequence
                persisted_values = direct_values
                direct = self._append_measurement_row_direct(
                    direct_values,
                    reason=admission.reason,
                )
                admission = direct
                row_index = -1
                direct_persistence = direct.admitted
            else:
                if admission.lease is not None:
                    self.ledger._pair_recording_row_leases[key] = admission.lease
                self.native_completion._track_pending_native_row(
                    row_index,
                    values,
                    row_lease=admission.lease
                    or self.ledger._pair_recording_row_leases.get(key),
                )
                self.presentation._refresh_recording_table()
                direct_persistence = False
            inserted = False
        enqueue_admitted = admission.admitted
        if enqueue_admitted:
            self.native_completion._finish_native_completion_from_admitted_row(persisted_values)
        self.presentation._refresh_recording_status_label()
        self.boundary._refresh_operation_header()
        return PairRecordingCommitResult(
            inserted=inserted,
            enqueue_admitted=enqueue_admitted,
            row_index=row_index,
            admission_state=admission.state,
            admission_reason=admission.reason,
            row_lease=admission.lease,
            direct_persistence=direct_persistence,
        )
