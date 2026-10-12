"""Result identity, evidence admission, bounded retries and original-run routing.

Attitudes: body FRD -> earth NED, wxyz. GPS baselines: NED metres.
Optical R,t: X_right_RDF = R_right_from_left X_left_RDF + t_m.
Public point_m: baseline-center FRD; source_point_left_rdf_m: left optical RDF.
"""

from __future__ import annotations
import hashlib
import json
import time
from collections.abc import Mapping
from dataclasses import dataclass, replace
from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS,
    PIXHAWK_RTK_LIVE,
    latest_status_by_method,
)
from Workstation.workstation_core.measurement_attempt import (
    BlockedMeasurementPair,
    DetectionSnapshot,
    MeasurementAttempt,
    detection_pair_key,
)
from Workstation.workstation_core.measurement_data_store import (
    ROW_WRITE_ADMITTED,
    ROW_WRITE_UNAVAILABLE,
)
from Workstation.workstation_core.method_evaluation import (
    MethodEvaluationBundle,
    MethodEvaluationOutcome,
)
from typing import Any, Callable
from Workstation.measurement import limits
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.state import (
    UNSET,
    read,
    ResultCommitState,
    MethodSelectionState,
    MeasurementPresentationState,
    WorkerRuntimeState,
)
from Workstation.measurement.pending_commit import (
    PendingMethodBundleCommit as _PendingMethodBundleCommit,
)


@dataclass(frozen=True)
class ResultCommitInputs:
    append_workstation_data: ValueInput
    _log: ValueInput
    release_primary_retry: ValueInput
    persist_compact_primary: ValueInput
    measurement_registry: ValueInput
    _record_blocked_measurement_pair: ValueInput
    distance_recording_enabled: ValueInput
    _measurement_session_hierarchy_enabled: ValueInput
    _measurement_id: ValueInput
    _measurement_run_generation: ValueInput
    _measurement_run_state: ValueInput
    _commit_primary_distance_outcome: ValueInput
    _append_distance_history: ValueInput
    method_evaluation_bundle_rows: ValueInput
    method_result_rows: ValueInput
    _recording_pair_row_index: ValueInput
    _upsert_pair_recording_row: ValueInput


@dataclass(frozen=True)
class ResultCommitActions:
    _pair_timing_decision: Callable[..., Any]
    _measurement_method_statuses: Callable[..., Any]
    _measurement_fallback_order: Callable[..., Any]
    _primary_outcome_key: Callable[..., Any]
    _failed_primary_outcome: Callable[..., Any]
    _method_history_payload: Callable[..., Any]
    _ensure_method_evaluation_timer: Callable[..., Any]
    _stop_method_evaluation_timer: Callable[..., Any]
    _method_bundle_key: Callable[..., Any]
    _method_result_row: Callable[..., Any]
    _bundle_flat_fields: Callable[..., Any]
    _optical_relative_rotation_fields: Callable[..., Any]
    _display_primary_method_outcome: Callable[..., Any]


class ResultCommitService:
    """Result identity, evidence admission, bounded retries and original-run routing."""

    def __init__(
        self,
        *,
        state: ResultCommitState,
        inputs: ResultCommitInputs,
        actions: ResultCommitActions,
        methods_state: MethodSelectionState,
        presentation_state: MeasurementPresentationState,
        runtime_state: WorkerRuntimeState,
    ) -> None:
        self.state = state
        self.inputs = inputs
        self.actions = actions
        self.methods_state = methods_state
        self.presentation_state = presentation_state
        self.runtime_state = runtime_state

    @staticmethod
    def _pending_record_size(record: Mapping[str, object]) -> int:
        try:
            return len(
                json.dumps(
                    dict(record),
                    ensure_ascii=False,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            )
        except (TypeError, ValueError):
            return len(repr(dict(record)).encode("utf-8", errors="replace"))

    def _sync_persistence_pending_status(self) -> None:
        self.state._primary_measurement_retry_status = {
            "state": (
                "OVERFLOW"
                if self.state._persistence_pending_overflow_count
                else (
                    "PENDING"
                    if self.state._pending_primary_measurement_commits
                    or self.state._pending_method_bundle_commits
                    else "IDLE"
                )
            ),
            "primary_pending_count": len(
                self.state._pending_primary_measurement_commits
            ),
            "primary_pending_bytes": self.state._pending_primary_measurement_bytes,
            "bundle_pending_count": len(self.state._pending_method_bundle_commits),
            "bundle_pending_bytes": self.state._pending_method_bundle_bytes,
            "max_count_per_kind": max(1, int(limits.PERSISTENCE_PENDING_MAX_COUNT)),
            "max_bytes_per_kind": max(1, int(limits.PERSISTENCE_PENDING_MAX_BYTES)),
            "overflow_count": self.state._persistence_pending_overflow_count,
            "overflow_evidence_failures": self.state._persistence_pending_overflow_evidence_failures,
            "shutdown_close_retry_calls": self.runtime_state._method_evaluation_shutdown_close_retry_calls,
            "shutdown_close_retry_limit": max(
                1, int(limits.METHOD_EVALUATION_SHUTDOWN_CLOSE_RETRY_LIMIT)
            ),
            "last_overflow": dict(self.state._last_persistence_pending_overflow),
        }

    def _record_persistence_pending_overflow(
        self, *, kind: str, key: tuple[str, str, int], digest: str, reason: str
    ) -> None:
        self.state._persistence_pending_overflow_count += 1
        record = {
            "event_type": "measurement_persistence_retry_overflow",
            "pending_kind": str(kind),
            "eviction_policy": "oldest_first_keep_newest",
            "reason": str(reason),
            "measurement_id": str(key[0]),
            "pair_id": str(key[1]),
            "request_generation": int(key[2]),
            "evidence_sha256": str(digest),
            "primary_pending_count": len(
                self.state._pending_primary_measurement_commits
            ),
            "primary_pending_bytes": self.state._pending_primary_measurement_bytes,
            "bundle_pending_count": len(self.state._pending_method_bundle_commits),
            "bundle_pending_bytes": self.state._pending_method_bundle_bytes,
            "max_count_per_kind": max(1, int(limits.PERSISTENCE_PENDING_MAX_COUNT)),
            "max_bytes_per_kind": max(1, int(limits.PERSISTENCE_PENDING_MAX_BYTES)),
        }
        writer = self.inputs.append_workstation_data.get(None)
        try:
            evidence_admitted = bool(
                callable(writer) and writer("events", record, priority=True)
            )
        except Exception:
            evidence_admitted = False
        if not evidence_admitted:
            self.state._persistence_pending_overflow_evidence_failures += 1
        record["overflow_evidence_admitted"] = evidence_admitted
        self.state._last_persistence_pending_overflow = record
        logger = self.inputs._log.get(None)
        if callable(logger):
            logger(
                f"[측정] persistence retry overflow: kind={kind}, pair={key[1]}, reason={reason}, evidence={('QUEUED' if evidence_admitted else 'REJECTED')}"
            )
        self._sync_persistence_pending_status()

    def _release_recording_primary_retry(
        self, outcome: MethodEvaluationOutcome
    ) -> None:
        releaser = self.inputs.release_primary_retry.get(None)
        if not callable(releaser):
            return
        try:
            attempt = MeasurementAttempt.from_canonical_payload(outcome.attempt_payload)
            releaser(tuple(attempt.detection_pair_key))
        except (KeyError, RuntimeError, TypeError, ValueError):
            return

    def _remove_pending_primary_measurement(self, key: tuple[str, str, int]) -> None:
        self.state._pending_primary_measurement_commits.pop(key, None)
        self.state._pending_primary_measurement_bytes -= (
            self.state._pending_primary_measurement_sizes.pop(key, 0)
        )
        self.state._pending_primary_measurement_bytes = max(
            0, self.state._pending_primary_measurement_bytes
        )
        self._sync_persistence_pending_status()

    def _retain_pending_primary_measurement(
        self, key: tuple[str, str, int], outcome: MethodEvaluationOutcome
    ) -> bool:
        self._remove_pending_primary_measurement(key)
        size = self._pending_record_size(outcome.to_record())
        count_limit = max(1, int(limits.PERSISTENCE_PENDING_MAX_COUNT))
        byte_limit = max(1, int(limits.PERSISTENCE_PENDING_MAX_BYTES))
        if size > byte_limit:
            self._release_recording_primary_retry(outcome)
            self._record_persistence_pending_overflow(
                kind="primary_measurement",
                key=key,
                digest=outcome.canonical_sha256(),
                reason=f"single_record_bytes={size} exceeds byte_limit={byte_limit}",
            )
            return False
        while (
            len(self.state._pending_primary_measurement_commits) >= count_limit
            or self.state._pending_primary_measurement_bytes + size > byte_limit
        ):
            oldest_key = next(iter(self.state._pending_primary_measurement_commits))
            oldest = self.state._pending_primary_measurement_commits.get(oldest_key)
            oldest_size = self.state._pending_primary_measurement_sizes.pop(
                oldest_key, 0
            )
            self.state._pending_primary_measurement_commits.pop(oldest_key, None)
            self.state._pending_primary_measurement_bytes = max(
                0, self.state._pending_primary_measurement_bytes - oldest_size
            )
            if oldest is not None:
                self._release_recording_primary_retry(oldest)
                self._record_persistence_pending_overflow(
                    kind="primary_measurement",
                    key=oldest_key,
                    digest=oldest.canonical_sha256(),
                    reason="oldest evicted to admit newer primary retry",
                )
        self.state._pending_primary_measurement_commits[key] = outcome
        self.state._pending_primary_measurement_sizes[key] = size
        self.state._pending_primary_measurement_bytes += size
        self._sync_persistence_pending_status()
        return True

    def _remove_pending_method_bundle(self, key: tuple[str, str, int]) -> None:
        self.state._pending_method_bundle_commits.pop(key, None)
        self.state._pending_method_bundle_bytes -= (
            self.state._pending_method_bundle_sizes.pop(key, 0)
        )
        self.state._pending_method_bundle_bytes = max(
            0, self.state._pending_method_bundle_bytes
        )
        self._sync_persistence_pending_status()

    def _retain_pending_method_bundle(
        self, key: tuple[str, str, int], pending: _PendingMethodBundleCommit
    ) -> bool:
        self._remove_pending_method_bundle(key)
        size = self._pending_record_size(pending.bundle.to_record())
        count_limit = max(1, int(limits.PERSISTENCE_PENDING_MAX_COUNT))
        byte_limit = max(1, int(limits.PERSISTENCE_PENDING_MAX_BYTES))
        if size > byte_limit:
            self._record_persistence_pending_overflow(
                kind="method_bundle",
                key=key,
                digest=pending.bundle.canonical_sha256(),
                reason=f"single_record_bytes={size} exceeds byte_limit={byte_limit}",
            )
            return False
        while (
            len(self.state._pending_method_bundle_commits) >= count_limit
            or self.state._pending_method_bundle_bytes + size > byte_limit
        ):
            oldest_key = next(iter(self.state._pending_method_bundle_commits))
            oldest = self.state._pending_method_bundle_commits.get(oldest_key)
            oldest_size = self.state._pending_method_bundle_sizes.pop(oldest_key, 0)
            self.state._pending_method_bundle_commits.pop(oldest_key, None)
            self.state._pending_method_bundle_bytes = max(
                0, self.state._pending_method_bundle_bytes - oldest_size
            )
            if oldest is not None:
                self._record_persistence_pending_overflow(
                    kind="method_bundle",
                    key=oldest_key,
                    digest=oldest.bundle.canonical_sha256(),
                    reason="oldest evicted to admit newer bundle retry",
                )
        self.state._pending_method_bundle_commits[key] = pending
        self.state._pending_method_bundle_sizes[key] = size
        self.state._pending_method_bundle_bytes += size
        self._sync_persistence_pending_status()
        return True

    def _terminalize_shutdown_pending_retries(self) -> None:
        attempts = self.runtime_state._method_evaluation_shutdown_close_retry_calls
        reason = (
            f"application shutdown close retry budget exhausted: attempts={attempts}"
        )
        for key, outcome in tuple(
            self.state._pending_primary_measurement_commits.items()
        ):
            self._release_recording_primary_retry(outcome)
            self._record_persistence_pending_overflow(
                kind="primary_measurement_shutdown_drop",
                key=key,
                digest=outcome.canonical_sha256(),
                reason=reason,
            )
            self._remove_pending_primary_measurement(key)
        for key, pending in tuple(self.state._pending_method_bundle_commits.items()):
            self._record_persistence_pending_overflow(
                kind="method_bundle_shutdown_drop",
                key=key,
                digest=pending.bundle.canonical_sha256(),
                reason=reason,
            )
            self._remove_pending_method_bundle(key)
        self._sync_persistence_pending_status()

    def _measurement_registry(self):
        getter = self.inputs.measurement_registry.get(None)
        if not callable(getter):
            return None
        try:
            registry = getter()
        except Exception:
            return None
        return registry if hasattr(registry, "route_outcome") else None

    def _registry_is_current(self, measurement_id: str) -> bool | None:
        registry = self._measurement_registry()
        if registry is None:
            return None
        try:
            return registry.is_current(measurement_id)
        except Exception:
            return None

    def _selected_method_id_for(self, measurement_id: str) -> str:
        registry = self._measurement_registry()
        record = None if registry is None else registry.record_for(measurement_id)
        selected = (
            ""
            if record is None
            else str(getattr(record, "selected_method_id", "") or "")
        )
        if not selected:
            selected = str(
                read(self.runtime_state._measurement_recording_method_id, "")
                or PIXHAWK_RTK_LIVE
            )
        return selected if selected in MEASUREMENT_METHOD_IDS else PIXHAWK_RTK_LIVE

    def _record_no_ready_measurement_pair(self, pair) -> None:
        timing = self.actions._pair_timing_decision(pair)
        timestamp_basis = timing.effective_basis
        left_pairing_timestamp_s = timing.left_timestamp_s
        right_pairing_timestamp_s = timing.right_timestamp_s
        statuses = self.actions._measurement_method_statuses()
        readiness_payload = {
            "schema": "measurement-readiness-block-v1",
            "preferred_method_id": str(
                read(self.methods_state._measurement_preferred_method_id, "") or ""
            ),
            "fallback_order": list(self.actions._measurement_fallback_order()),
            "methods": [row.to_mapping() for row in statuses],
        }
        readiness_sha256 = hashlib.sha256(
            json.dumps(
                readiness_payload, sort_keys=True, separators=(",", ":")
            ).encode()
        ).hexdigest()
        reasons = tuple(
            (
                f"{row.method_id}: {', '.join(row.reasons) or 'not ready'}"
                for row in latest_status_by_method(statuses).values()
                if not row.ready
            )
        ) or ("no ready measurement method",)
        blocked = BlockedMeasurementPair(
            detection_pair_key=detection_pair_key(
                triangulation_source="no_ready_measurement_method",
                timestamp_basis=timestamp_basis,
                left=pair.left,
                right=pair.right,
                left_pairing_timestamp_s=left_pairing_timestamp_s,
                right_pairing_timestamp_s=right_pairing_timestamp_s,
            ),
            created_monotonic_s=time.monotonic(),
            created_wall_s=time.time(),
            timestamp_basis=timestamp_basis,
            left_detection=DetectionSnapshot.from_observation(
                pair.left,
                pairing_timestamp_basis=timestamp_basis,
                pairing_timestamp_s=left_pairing_timestamp_s,
            ),
            right_detection=DetectionSnapshot.from_observation(
                pair.right,
                pairing_timestamp_basis=timestamp_basis,
                pairing_timestamp_s=right_pairing_timestamp_s,
            ),
            status="matched_no_ready_measurement_method",
            reasons=reasons,
            readiness_sha256=readiness_sha256,
            readiness_payload_json=json.dumps(
                readiness_payload, sort_keys=True, separators=(",", ":")
            ),
            requested_timestamp_basis=timing.requested_basis,
            timing_fallback_reason=timing.fallback_reason,
        )
        self.inputs._record_blocked_measurement_pair.get()(blocked)

    def _primary_outcome_is_current(self, outcome: MethodEvaluationOutcome) -> bool:
        registry_current = self._registry_is_current(outcome.measurement_id)
        if registry_current is not None:
            return registry_current
        if not bool(self.inputs.distance_recording_enabled.get(False)):
            return False
        sessionless = not bool(
            self.inputs._measurement_session_hierarchy_enabled.get(True)
        )
        if not sessionless:
            raw_measurement_id = self.inputs._measurement_id.get(None)
            if raw_measurement_id is None:
                return True
            current_measurement_id = str(raw_measurement_id or "")
            return bool(
                current_measurement_id
                and outcome.measurement_id == current_measurement_id
            )
        current_generation = str(
            self.inputs._measurement_run_generation.get("")
            or self.inputs._measurement_id.get("")
            or ""
        )
        run_state = str(
            self.inputs._measurement_run_state.get("IDLE") or "IDLE"
        ).upper()
        return bool(
            run_state == "RUNNING"
            and current_generation
            and (outcome.measurement_id == current_generation)
        )

    def _normalize_primary_outcome(
        self,
        outcome: MethodEvaluationOutcome,
        *,
        expected_method_id: str = PIXHAWK_RTK_LIVE,
    ) -> MethodEvaluationOutcome:
        key = self.actions._primary_outcome_key(outcome)
        outcome_sha256 = outcome.canonical_sha256()
        previous_sha256 = self.state._primary_outcome_hash_by_key.get(key)
        if previous_sha256 is not None and previous_sha256 != outcome_sha256:
            logger = self.inputs._log.get(None)
            if callable(logger):
                logger(
                    f"[측정] conflicting primary outcome ignored after first accepted identity: pair={outcome.pair_id}"
                )
            cached = self.state._primary_normalized_outcome_by_key.get(key)
            return (
                cached
                if cached is not None
                else self.actions._failed_primary_outcome(
                    outcome,
                    "conflicting primary method outcome for one request identity",
                )
            )
        self.state._primary_outcome_hash_by_key.setdefault(key, outcome_sha256)
        cached = self.state._primary_normalized_outcome_by_key.get(key)
        if cached is not None:
            return cached
        normalized = outcome
        try:
            if outcome.method_id != expected_method_id:
                raise ValueError(f"primary outcome method is not {expected_method_id}")
            if outcome.status in {"VALID", "ESTIMATED"}:
                if not isinstance(outcome.attempt_payload, Mapping):
                    raise ValueError(
                        "successful primary outcome has no attempt payload"
                    )
                attempt = MeasurementAttempt.from_canonical_payload(
                    outcome.attempt_payload
                )
                if attempt.canonical_sha256() != outcome.attempt_sha256:
                    raise ValueError("primary outcome attempt hash mismatch")
                if attempt.measurement_method_id != expected_method_id:
                    raise ValueError(
                        f"primary attempt method is not {expected_method_id}"
                    )
                if attempt.measurement_geometry_sha256 != outcome.geometry_sha256:
                    raise ValueError("primary outcome geometry hash mismatch")
                self.actions._method_history_payload(outcome, attempt)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            normalized = self.actions._failed_primary_outcome(
                outcome, f"primary outcome validation failed: {exc}"
            )
        self.state._primary_normalized_outcome_by_key[key] = normalized
        return normalized

    def _commit_primary_method_outcome(
        self,
        outcome: MethodEvaluationOutcome,
        *,
        commit_source: str,
        expected_method_id: str = PIXHAWK_RTK_LIVE,
    ) -> tuple[MethodEvaluationOutcome, bool, str]:
        normalized = self._normalize_primary_outcome(
            outcome, expected_method_id=expected_method_id
        )
        key = self.actions._primary_outcome_key(normalized)
        current = self._primary_outcome_is_current(normalized)
        if not current:
            return (normalized, False, "NOT_REQUESTED")
        if normalized.status not in {"VALID", "ESTIMATED"}:
            return (normalized, True, "NOT_REQUESTED")
        if key in self.state._primary_measurement_persisted:
            self._remove_pending_primary_measurement(key)
            return (normalized, True, "QUEUED")
        try:
            attempt = MeasurementAttempt.from_canonical_payload(
                normalized.attempt_payload
            )
            history_payload = self.actions._method_history_payload(normalized, attempt)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            failed = self.actions._failed_primary_outcome(
                normalized, f"primary persistence payload invalid: {exc}"
            )
            self.state._primary_normalized_outcome_by_key[key] = failed
            return (failed, True, "NOT_REQUESTED")
        publish_history = key not in self.state._primary_history_projected
        outcome_sha256 = normalized.canonical_sha256()
        metadata = {
            "runtime_generation": normalized.measurement_id,
            "method_evaluation_pair_id": normalized.pair_id,
            "method_evaluation_pair_sequence": normalized.pair_sequence,
            "method_evaluation_request_generation": normalized.request_generation,
            "method_evaluation_snapshot_sha256": normalized.snapshot_sha256,
            "primary_outcome_status": normalized.status,
            "primary_outcome_sha256": outcome_sha256,
            "primary_committed_before_bundle": commit_source == "early_outcome",
            "primary_commit_source": str(commit_source),
        }
        method = f"{history_payload['method']}-{expected_method_id}"
        committer = self.inputs._commit_primary_distance_outcome.get(None)
        if callable(committer):
            commit_result = committer(
                estimate=history_payload["estimate"],
                distance_m=float(history_payload["distance_m"]),
                method=method,
                baseline_source=attempt.baseline_source,
                baseline_m=attempt.baseline_m,
                point_m=history_payload["point_m"],
                raw_point_m=history_payload["point_m"],
                raw_point_left_m=history_payload["source_point_left_rdf_m"],
                measurement_attempt=attempt,
                outcome_metadata=metadata,
                publish_history=publish_history,
            )
            history_added = bool(getattr(commit_result, "history_added", False))
            row_commit = getattr(commit_result, "row_commit", None)
            enqueue_admitted = bool(getattr(row_commit, "enqueue_admitted", False))
        else:
            legacy_append = self.inputs._append_distance_history.get(None)
            enqueue_admitted = bool(
                callable(legacy_append)
                and legacy_append(
                    estimate=history_payload["estimate"],
                    distance_m=float(history_payload["distance_m"]),
                    method=method,
                    baseline_source=attempt.baseline_source,
                    baseline_m=attempt.baseline_m,
                    point_m=history_payload["point_m"],
                    raw_point_m=history_payload["point_m"],
                    raw_point_left_m=history_payload["source_point_left_rdf_m"],
                    measurement_attempt=attempt,
                )
            )
            history_added = enqueue_admitted
        if history_added:
            self.state._primary_history_projected.add(key)
        if enqueue_admitted:
            self.state._primary_measurement_persisted.add(key)
            self._remove_pending_primary_measurement(key)
            return (normalized, True, "QUEUED")
        first_rejection = key not in self.state._pending_primary_measurement_commits
        retained = self._retain_pending_primary_measurement(key, normalized)
        if retained:
            self.actions._ensure_method_evaluation_timer()
        logger = self.inputs._log.get(None)
        if first_rejection and callable(logger):
            logger(
                f"[측정] PX4+RTK 대표 결과 저장 enqueue 실패; background에서 재시도합니다. pair={normalized.pair_id}"
            )
        return (normalized, True, "REJECTED")

    def _persist_late_primary_measurement(
        self, outcome: MethodEvaluationOutcome
    ) -> bool:
        """Persist a retired primary result without touching the next run's rows/UI."""
        writer = self.inputs.append_workstation_data.get(None)
        if not callable(writer):
            return False
        current_generation = str(
            self.inputs._measurement_run_generation.get("")
            or self.inputs._measurement_id.get("")
            or ""
        )
        primary_record = outcome.to_record()
        primary_canonical_json = json.dumps(
            primary_record,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        record = {
            "event_type": "late_primary_measurement_outcome",
            "record_kind": "primary_measurement_evidence",
            "runtime_generation": str(outcome.measurement_id),
            "measurement_id": str(outcome.measurement_id),
            "method_evaluation_pair_id": str(outcome.pair_id),
            "method_evaluation_pair_sequence": int(outcome.pair_sequence),
            "method_evaluation_request_generation": int(outcome.request_generation),
            "primary_outcome_status": str(outcome.status),
            "primary_outcome_sha256": outcome.canonical_sha256(),
            "primary_outcome": primary_record,
            "primary_outcome_canonical_json": primary_canonical_json,
            "primary_persistence_mode": "late_direct_retry",
            "late": True,
            "current_runtime_generation": current_generation,
        }
        compact_persister = self.inputs.persist_compact_primary.get(None)
        if callable(compact_persister):
            try:
                attempt = MeasurementAttempt.from_canonical_payload(
                    outcome.attempt_payload
                )
                return bool(
                    compact_persister(tuple(attempt.detection_pair_key), record)
                )
            except (KeyError, OSError, RuntimeError, TypeError, ValueError):
                return False
        try:
            return bool(writer("measurements", record, priority=True))
        except (OSError, RuntimeError, TypeError, ValueError):
            return False

    def _retry_pending_primary_measurements(self) -> None:
        """Retry primary evidence independently from the already-saved bundle."""
        if not self.state._pending_primary_measurement_commits:
            return
        logger = self.inputs._log.get(None)
        retry_batch = tuple(self.state._pending_primary_measurement_commits.items())[
            : max(1, int(limits.PRIMARY_MEASUREMENT_RETRY_BUDGET))
        ]
        for key, outcome in retry_batch:
            self.state._pending_primary_measurement_commits.pop(key, None)
            if key in self.state._primary_measurement_persisted:
                self._remove_pending_primary_measurement(key)
                continue
            self.state._pending_primary_measurement_commits[key] = outcome
            try:
                if self._primary_outcome_is_current(outcome):
                    self._commit_primary_method_outcome(
                        outcome, commit_source="background_retry"
                    )
                    continue
                if self._persist_late_primary_measurement(outcome):
                    self.state._primary_measurement_persisted.add(key)
                    self._remove_pending_primary_measurement(key)
            except Exception as exc:
                if callable(logger):
                    logger(f"[측정] primary 저장 재시도 실패: {exc}")
        if (
            not self.state._pending_primary_measurement_commits
            and (not self.state._pending_method_bundle_commits)
            and (self.runtime_state._method_evaluation_coordinator is None)
        ):
            self.actions._stop_method_evaluation_timer()

    def _update_method_evaluation_counters(
        self, bundle: MethodEvaluationBundle
    ) -> None:
        counters = read(self.state._measurement_method_result_counters, None)
        if not isinstance(counters, dict):
            counters = {
                method_id: {"VALID": 0, "BLOCKED": 0, "FAILED": 0}
                for method_id in MEASUREMENT_METHOD_IDS
            }
            self.state._measurement_method_result_counters = counters
        for outcome in bundle.outcomes:
            bucket = (
                "VALID"
                if outcome.status in {"VALID", "ESTIMATED"}
                else (
                    "BLOCKED"
                    if outcome.status in {"BLOCKED", "BACKPRESSURE_BLOCKED"}
                    else "FAILED"
                )
            )
            counters.setdefault(
                outcome.method_id, {"VALID": 0, "BLOCKED": 0, "FAILED": 0}
            )[bucket] += 1

    def _method_bundle_is_current(
        self, bundle: MethodEvaluationBundle
    ) -> tuple[bool, str]:
        sessionless = not bool(
            self.inputs._measurement_session_hierarchy_enabled.get(True)
        )
        current_generation = str(self.inputs._measurement_run_generation.get("") or "")
        registry_current = self._registry_is_current(bundle.measurement_id)
        if registry_current is not None:
            return (registry_current, str(current_generation or bundle.measurement_id))
        run_state = str(
            self.inputs._measurement_run_state.get("IDLE") or "IDLE"
        ).upper()
        if not sessionless:
            raw_measurement_id = self.inputs._measurement_id.get(None)
            if raw_measurement_id is None:
                return (
                    bool(self.inputs.distance_recording_enabled.get(False)),
                    str(bundle.measurement_id),
                )
            hierarchy_generation = str(raw_measurement_id or current_generation or "")
            return (
                bool(
                    self.inputs.distance_recording_enabled.get(False)
                    and hierarchy_generation
                    and (bundle.measurement_id == hierarchy_generation)
                ),
                hierarchy_generation,
            )
        return (
            bool(
                bundle.measurement_id == current_generation and run_state == "RUNNING"
            ),
            current_generation,
        )

    def _method_bundle_record(
        self, bundle: MethodEvaluationBundle, *, current: bool, current_generation: str
    ) -> dict[str, object]:
        record = dict(bundle.to_record())
        if not current:
            record.update(
                {"late": True, "current_runtime_generation": str(current_generation)}
            )
        return record

    @staticmethod
    def _write_admitted(result: object) -> bool:
        return True if result is None else bool(result)

    @staticmethod
    def _method_bundle_row_owner_usable(owner: object, index: int | None) -> bool:
        if index is None:
            return False
        try:
            if index < 0 or index >= len(owner):
                return False
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return False
        store = getattr(owner, "store", None)
        snapshot_reader = getattr(store, "snapshot", None)
        if callable(snapshot_reader):
            try:
                snapshot = snapshot_reader()
            except (KeyError, OSError, RuntimeError, TypeError, ValueError):
                return False
            if isinstance(snapshot, Mapping):
                lifecycle = str(snapshot.get("lifecycle_state") or "").upper()
                if lifecycle and lifecycle != "OPEN":
                    return False
                if snapshot.get("writer_alive") is False:
                    return False
        try:
            write_error = getattr(owner, "write_error", None)
        except (KeyError, OSError, RuntimeError, TypeError, ValueError):
            return False
        if write_error:
            return False
        return True

    def _persist_method_evaluation_bundle(self, bundle: MethodEvaluationBundle) -> bool:
        key = self.actions._method_bundle_key(bundle)
        current, current_generation = self._method_bundle_is_current(bundle)
        record = self._method_bundle_record(
            bundle, current=current, current_generation=current_generation
        )

        def append_direct() -> bool:
            writer = self.inputs.append_workstation_data.get(None)
            try:
                return bool(
                    callable(writer)
                    and writer("method_evaluations", record, priority=True)
                )
            except (OSError, RuntimeError, TypeError, ValueError):
                return False

        pending = self.state._pending_method_bundle_commits.get(key)
        pending_candidate: _PendingMethodBundleCommit | None = None
        if pending is not None:
            admitted = False
            if pending.row_owner is None:
                admitted = append_direct()
            else:
                owner = pending.row_owner
                index = pending.row_index
                owner_usable = self._method_bundle_row_owner_usable(owner, index)
                owner_unavailable = not owner_usable
                if owner_usable:
                    atomic_replacer = getattr(owner, "replace_with_admission", None)
                    replacer = getattr(owner, "replace", None)
                    try:
                        if callable(atomic_replacer):
                            replacement = atomic_replacer(
                                index, record, lease=pending.row_lease
                            )
                            replacement_state = str(
                                getattr(replacement, "state", "") or ""
                            ).upper()
                            admitted = replacement_state == ROW_WRITE_ADMITTED
                            owner_unavailable = (
                                replacement_state == ROW_WRITE_UNAVAILABLE
                            )
                        elif callable(replacer):
                            admitted = self._write_admitted(replacer(index, record))
                        else:
                            owner[index] = dict(record)
                            admitted = True
                    except (IndexError, OSError, RuntimeError, TypeError, ValueError):
                        admitted = False
                        owner_unavailable = True
                if not admitted and owner_unavailable:
                    pending.row_owner = None
                    pending.row_index = None
                    pending.row_lease = None
                    admitted = append_direct()
        elif current:
            owner = self.inputs.method_evaluation_bundle_rows.get(None)
            atomic_appender = getattr(owner, "append_with_admission", None)
            appender = getattr(owner, "append", None)
            if callable(atomic_appender):
                index = len(owner)
                appended = atomic_appender(record)
                append_state = str(getattr(appended, "state", "") or "").upper()
                admitted = append_state == ROW_WRITE_ADMITTED
                owner_unavailable = append_state == ROW_WRITE_UNAVAILABLE
                row_index = getattr(appended, "row_index", index)
                row_lease = getattr(appended, "lease", None)
                if not admitted:
                    if owner_unavailable:
                        admitted = append_direct()
                    else:
                        pending_candidate = _PendingMethodBundleCommit(
                            bundle=bundle,
                            row_owner=owner,
                            row_index=row_index,
                            row_lease=row_lease,
                        )
            elif callable(appender):
                index = len(owner)
                admitted = self._write_admitted(appender(record))
                if not admitted:
                    pending_candidate = _PendingMethodBundleCommit(
                        bundle=bundle, row_owner=owner, row_index=index
                    )
            else:
                admitted = False
        else:
            admitted = append_direct()
        if admitted:
            self._remove_pending_method_bundle(key)
            self.state._persisted_method_bundle_keys.add(key)
            committed = read(self.state._method_evaluation_committed, None)
            current_after_write, _generation_after_write = (
                self._method_bundle_is_current(bundle)
            )
            if current_after_write and isinstance(committed, set):
                committed.add((bundle.pair_id, bundle.request_generation))
            return True
        existing = pending_candidate or self.state._pending_method_bundle_commits.get(
            key
        )
        if existing is None:
            existing = _PendingMethodBundleCommit(bundle=bundle)
        else:
            existing.bundle = bundle
        self._retain_pending_method_bundle(key, existing)
        return False

    def _retry_pending_method_bundles(self) -> None:
        if not self.state._pending_method_bundle_commits:
            return
        logger = self.inputs._log.get(None)
        retry_batch = tuple(self.state._pending_method_bundle_commits.items())[
            : max(1, int(limits.METHOD_BUNDLE_RETRY_BUDGET))
        ]
        for key, pending in retry_batch:
            self.state._pending_method_bundle_commits.pop(key, None)
            self.state._pending_method_bundle_commits[key] = pending
            try:
                self._commit_method_evaluation_bundle(pending.bundle)
            except Exception as exc:
                if callable(logger):
                    logger(f"[측정] method bundle 저장 재시도 실패: {exc}")
        if (
            not self.state._pending_method_bundle_commits
            and (not self.state._pending_primary_measurement_commits)
            and (self.runtime_state._method_evaluation_coordinator is None)
        ):
            self.actions._stop_method_evaluation_timer()

    def _persist_method_result_rows(self, bundle: MethodEvaluationBundle) -> int:
        """Append one thin row per outcome to the ``method_results`` stream."""
        owner = self.inputs.method_result_rows.get(None)
        appender = getattr(owner, "append", None)
        if not callable(appender):
            return 0
        written = 0
        for outcome in bundle.outcomes:
            try:
                if appender(self.actions._method_result_row(bundle, outcome)):
                    written += 1
            except (OSError, RuntimeError, TypeError, ValueError):
                continue
        return written

    def _flatten_bundle_into_measurement_row(
        self, bundle: MethodEvaluationBundle
    ) -> bool:
        """Merge every method's result into the pair's ``measurements`` row.

        The row is created by the primary commit; this re-writes it (a
        ``replace`` record) once the whole bundle is known so one row carries
        RTK, baseline-yaw, the yaw selection evidence and the optical relative
        yaw side by side. Pairs without a primary row are left to
        ``method_results``.
        """
        if not bundle.outcomes:
            return False
        index_map = self.inputs._recording_pair_row_index.get(None)
        upsert = self.inputs._upsert_pair_recording_row.get(None)
        if not isinstance(index_map, Mapping) or not callable(upsert):
            return False
        source = bundle.primary if bundle.primary is not None else bundle.outcomes[0]
        try:
            attempt = MeasurementAttempt.from_canonical_payload(source.attempt_payload)
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        if tuple(attempt.detection_pair_key) not in index_map:
            return False
        values = self.actions._bundle_flat_fields(bundle)
        values.update(
            self.actions._optical_relative_rotation_fields(source.attempt_payload)
        )
        try:
            commit = upsert(attempt, values)
        except (OSError, RuntimeError, TypeError, ValueError):
            return False
        return bool(getattr(commit, "enqueue_admitted", False))

    def _commit_method_evaluation_bundle(self, bundle: MethodEvaluationBundle) -> None:
        bundle_key = self.actions._method_bundle_key(bundle)
        if bundle_key in self.state._persisted_method_bundle_keys:
            return
        committed = read(self.state._method_evaluation_committed, None)
        if not isinstance(committed, set):
            committed = set()
            self.state._method_evaluation_committed = committed
        identity = (bundle.pair_id, bundle.request_generation)
        if identity in committed:
            return
        selected_method_id = self._selected_method_id_for(bundle.measurement_id)
        selected_outcome = bundle.outcome_for(selected_method_id)
        primary = None
        current = False
        persistence_state = "NOT_REQUESTED"
        if selected_outcome is not None:
            primary, current, persistence_state = self._commit_primary_method_outcome(
                selected_outcome,
                commit_source="bundle_fallback",
                expected_method_id=selected_method_id,
            )
            if primary != selected_outcome:
                outcomes = tuple(
                    (
                        primary if outcome.method_id == selected_method_id else outcome
                        for outcome in bundle.outcomes
                    )
                )
                bundle = replace(bundle, outcomes=outcomes)
        if not self._persist_method_evaluation_bundle(bundle):
            return
        self._persist_method_result_rows(bundle)
        self._flatten_bundle_into_measurement_row(bundle)
        current_after_write, _generation_after_write = self._method_bundle_is_current(
            bundle
        )
        if not current_after_write:
            return
        self.state._last_method_evaluation_bundle = bundle
        self._update_method_evaluation_counters(bundle)
        primary_key = self.actions._primary_outcome_key(primary)
        if current and (
            self.presentation_state._primary_displayed_key == primary_key
            or primary.pair_sequence
            >= self.presentation_state._primary_display_high_water
        ):
            self.presentation_state._primary_display_high_water = max(
                self.presentation_state._primary_display_high_water,
                primary.pair_sequence,
            )
            self.presentation_state._primary_displayed_key = primary_key
            self.actions._display_primary_method_outcome(
                primary, waiting_for_barrier=False, persistence_state=persistence_state
            )
