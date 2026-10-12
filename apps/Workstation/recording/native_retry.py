"""Retry native evidence publication within bounded count, age, and batch limits.

Terminal drops preserve explicit diagnostics and release shutdown barriers."""
from __future__ import annotations
import copy
import json
import time
from collections.abc import Mapping
from PyQt5 import QtCore
from .contracts import (NATIVE_COMPLETION_FAST_RETRY_LIMIT, NATIVE_COMPLETION_MAX_RETRY_AGE_S, NATIVE_COMPLETION_RETRY_BATCH_LIMIT, NATIVE_COMPLETION_RETRY_INTERVAL_MS, NATIVE_COMPLETION_TOTAL_RETRY_LIMIT, _NativeTerminalFingerprint, _PendingNativeCompletion, _PendingNativeConflict)
from .state import (NativeEvidenceState)
from .ports import NativeRetryPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .native_completion import NativeCompletion
    from .native_submission import NativeSubmission


class NativeRetry:
    def __init__(self, *,
        native_state: NativeEvidenceState,
        boundary: NativeRetryPort,
    ) -> None:
        self.native_state = native_state
        self.boundary = boundary

    def bind_collaborators(self, *,
        native_completion: "NativeCompletion",
        native_submission: "NativeSubmission",
    ) -> None:
        self.native_completion = native_completion
        self.native_submission = native_submission

    def _log_native_completion_slow_retry(
        self,
        *,
        request_id: str,
        attempts: int,
    ) -> None:
        logger = getattr(self.boundary, "_log", None)
        if callable(logger):
            logger(
                "[저장] native completion fast retry budget 소진; "
                "payload ownership을 유지하고 저속 background 재시도를 계속합니다: "
                f"request={request_id}, attempts={attempts}"
            )

    @staticmethod
    def _native_retry_is_terminal(
        *,
        attempts: int,
        first_seen_monotonic_s: float,
    ) -> bool:
        age_s = max(0.0, time.monotonic() - first_seen_monotonic_s)
        return bool(
            attempts >= NATIVE_COMPLETION_TOTAL_RETRY_LIMIT
            or age_s >= NATIVE_COMPLETION_MAX_RETRY_AGE_S
        )

    def _remember_native_completion_terminal_drop(
        self,
        record: Mapping[str, object],
    ) -> None:
        if (
            self.native_state._native_completion_terminal_drops.maxlen is not None
            and len(self.native_state._native_completion_terminal_drops)
            >= self.native_state._native_completion_terminal_drops.maxlen
        ):
            self.native_state._native_completion_terminal_drop_evictions += 1
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                logger(
                    "[저장] native completion terminal-drop 진단 cache가 가득 차 "
                    "가장 오래된 항목을 명시적으로 교체합니다: "
                    f"evictions={self.native_state._native_completion_terminal_drop_evictions}"
                )
        frozen = copy.deepcopy(dict(record))
        self.native_state._native_completion_terminal_drops.append(frozen)
        self.native_state._native_completion_terminal_drop_count += 1
        setattr(
            self.native_state,
            "_native_completion_terminal_drop_count",
            self.native_state._native_completion_terminal_drop_count,
        )
        setattr(
            self.native_state,
            "_native_completion_last_terminal_drop",
            frozen,
        )
        logger = getattr(self.boundary, "_log", None)
        if callable(logger):
            logger(
                "[저장] native completion 영구 admission 실패를 terminal drop으로 "
                "전환했습니다; 원 payload/hash는 bounded 진단 cache에 유지됩니다: "
                f"request={frozen.get('request_id')}, "
                f"sha256={frozen.get('native_completion_sha256')}, "
                f"attempts={frozen.get('attempts')}"
            )

    def _terminal_drop_native_completion(
        self,
        record: _PendingNativeCompletion,
    ) -> None:
        context = record.payload.get("context")
        context = dict(context) if isinstance(context, Mapping) else {}
        fields = record.payload.get("fields")
        fields = dict(fields) if isinstance(fields, Mapping) else {}
        age_s = max(0.0, time.monotonic() - record.first_seen_monotonic_s)
        drop_fields = {
            **fields,
            "native_completion_sha256": record.canonical_sha256,
            "native_completion_persistence_state": "TERMINAL_DROPPED",
            "native_completion_persistence_reason": (
                "row projection and native.frames event admission remained unavailable"
            ),
            "native_completion_retry_attempts": record.attempts,
            "native_completion_retry_age_s": age_s,
            "native_completion_canonical_json": json.dumps(
                record.payload,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ),
        }
        status_event_admitted = self.native_completion._append_native_frame_completion_event(
            request_id=record.request_id,
            context=context,
            fields=drop_fields,
            force=True,
            event_type="native_frame_completion_persistence_dropped",
        )
        self._remember_native_completion_terminal_drop(
            {
                "event_type": "native_frame_completion_persistence_dropped",
                "request_id": record.request_id,
                "runtime_generation": str(context.get("session_id") or ""),
                "native_completion_sha256": record.canonical_sha256,
                "attempts": record.attempts,
                "age_s": age_s,
                "status_event_admitted": status_event_admitted,
                "payload": record.payload,
                "owner_runtime_generation": (
                    "" if record.owner is None else record.owner.runtime_generation
                ),
                "owner_row_index": (
                    None if record.owner is None else record.owner.row_index
                ),
                "owner_row_lease": (
                    None
                    if record.owner is None or record.owner.row_lease is None
                    else list(record.owner.row_lease)
                ),
            }
        )
        self.native_completion._finish_native_completion(record.request_id)

    def _terminal_drop_native_conflict(
        self,
        record: _PendingNativeConflict,
    ) -> None:
        age_s = max(0.0, time.monotonic() - record.first_seen_monotonic_s)
        fields = {
            "native_completion_conflict": True,
            "native_completion_authority": "first_terminal_wins",
            "native_completion_authoritative_sha256": record.authoritative_sha256,
            "native_completion_rejected_sha256": record.conflicting_sha256,
            "native_completion_rejected_canonical_json": (
                record.conflicting_canonical_json
            ),
            "native_completion_persistence_state": "TERMINAL_DROPPED",
            "native_completion_retry_attempts": record.attempts,
            "native_completion_retry_age_s": age_s,
        }
        status_event_admitted = self.native_completion._append_native_frame_completion_event(
            request_id=record.request_id,
            context=record.context,
            fields=fields,
            force=True,
            event_type="native_frame_completion_conflict_persistence_dropped",
        )
        self._remember_native_completion_terminal_drop(
            {
                "event_type": "native_frame_completion_conflict_persistence_dropped",
                "request_id": record.request_id,
                "runtime_generation": str(
                    record.context.get("session_id") or ""
                ),
                "native_completion_sha256": record.authoritative_sha256,
                "conflicting_sha256": record.conflicting_sha256,
                "attempts": record.attempts,
                "age_s": age_s,
                "status_event_admitted": status_event_admitted,
            }
        )
        self.native_state._pending_native_conflicts.pop(record.request_id, None)
        self.native_state._native_conflicts_recorded.add(record.request_id)

    def _defer_native_completion_retry(
        self,
        record: _PendingNativeCompletion,
    ) -> None:
        record.attempts += 1
        record.next_retry_monotonic_s = (
            time.monotonic()
            + self.native_submission._native_completion_retry_delay_s(record.attempts)
        )
        if (
            record.attempts >= NATIVE_COMPLETION_FAST_RETRY_LIMIT
            and not record.slow_retry_logged
        ):
            record.slow_retry_logged = True
            self._log_native_completion_slow_retry(
                request_id=record.request_id,
                attempts=record.attempts,
            )
        if self._native_retry_is_terminal(
            attempts=record.attempts,
            first_seen_monotonic_s=record.first_seen_monotonic_s,
        ):
            self._terminal_drop_native_completion(record)
            return
        self.native_state._pending_native_completions[record.request_id] = record
        self._schedule_native_completion_retry()

    def _attempt_native_conflict_diagnostic(
        self,
        record: _PendingNativeConflict,
    ) -> bool:
        admitted = self.native_completion._append_native_frame_completion_event(
            request_id=record.request_id,
            context=record.context,
            fields={
                "native_completion_conflict": True,
                "native_completion_authority": "first_terminal_wins",
                "native_completion_authoritative_sha256": (
                    record.authoritative_sha256
                ),
                "native_completion_rejected_sha256": record.conflicting_sha256,
                "native_completion_rejected_canonical_json": (
                    record.conflicting_canonical_json
                ),
            },
            force=True,
            event_type="native_frame_completion_conflict",
        )
        if admitted:
            self.native_state._pending_native_conflicts.pop(record.request_id, None)
            self.native_state._native_conflicts_recorded.add(record.request_id)
            return True
        record.attempts += 1
        record.next_retry_monotonic_s = (
            time.monotonic()
            + self.native_submission._native_completion_retry_delay_s(record.attempts)
        )
        if (
            record.attempts >= NATIVE_COMPLETION_FAST_RETRY_LIMIT
            and not record.slow_retry_logged
        ):
            record.slow_retry_logged = True
            self._log_native_completion_slow_retry(
                request_id=record.request_id,
                attempts=record.attempts,
            )
        if self._native_retry_is_terminal(
            attempts=record.attempts,
            first_seen_monotonic_s=record.first_seen_monotonic_s,
        ):
            self._terminal_drop_native_conflict(record)
            return False
        self.native_state._pending_native_conflicts[record.request_id] = record
        self._schedule_native_completion_retry()
        return False

    def _record_native_completion_conflict(
        self,
        *,
        request_id: str,
        authoritative: _NativeTerminalFingerprint,
        conflicting_sha256: str,
        conflicting_canonical_json: str,
    ) -> None:
        if (
            request_id in self.native_state._native_conflicts_recorded
            or request_id in self.native_state._pending_native_conflicts
        ):
            return
        record = _PendingNativeConflict(
            request_id=request_id,
            context=dict(authoritative.context),
            authoritative_sha256=authoritative.canonical_sha256,
            conflicting_sha256=conflicting_sha256,
            conflicting_canonical_json=conflicting_canonical_json,
            first_seen_monotonic_s=time.monotonic(),
        )
        self.native_state._pending_native_conflicts[request_id] = record
        self._attempt_native_conflict_diagnostic(record)

    def _schedule_native_completion_retry(self) -> None:
        if self.native_state._native_completion_retry_scheduled or not (
            self.native_state._pending_native_completions or self.native_state._pending_native_conflicts
        ):
            return
        self.native_state._native_completion_retry_scheduled = True
        QtCore.QTimer.singleShot(
            NATIVE_COMPLETION_RETRY_INTERVAL_MS,
            self._retry_pending_native_completions,
        )

    def _retry_pending_native_completions(self, *, force: bool = False) -> None:
        self.native_state._native_completion_retry_scheduled = False
        now_s = time.monotonic()
        attempted = 0
        for request_id, record in tuple(self.native_state._pending_native_completions.items()):
            if attempted >= NATIVE_COMPLETION_RETRY_BATCH_LIMIT:
                break
            if not force and record.next_retry_monotonic_s > now_s:
                continue
            payload = copy.deepcopy(record.payload)
            payload["_native_completion_retry"] = True
            payload["_native_completion_owner"] = record.owner
            payload["_completion_event_recorded"] = record.event_recorded
            attempted += 1
            try:
                self.native_completion._handle_async_native_frame_result(payload)
            except Exception as exc:  # noqa: BLE001 - retry ownership must survive.
                logger = getattr(self.boundary, "_log", None)
                if callable(logger):
                    logger(
                        "[저장] native completion background retry 실패: "
                        f"request={request_id}, error={exc}"
                    )
                self._defer_native_completion_retry(record)
        now_s = time.monotonic()
        for request_id, record in tuple(self.native_state._pending_native_conflicts.items()):
            if attempted >= NATIVE_COMPLETION_RETRY_BATCH_LIMIT:
                break
            if not force and record.next_retry_monotonic_s > now_s:
                continue
            attempted += 1
            self._attempt_native_conflict_diagnostic(record)
        if self.native_state._pending_native_completions or self.native_state._pending_native_conflicts:
            self._schedule_native_completion_retry()
