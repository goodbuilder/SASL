"""App-lifetime worker scheduling, recovery routing and Qt timer ownership.

Attitudes: body FRD -> earth NED, wxyz. GPS baselines: NED metres.
Optical R,t: X_right_RDF = R_right_from_left X_left_RDF + t_m.
Public point_m: baseline-center FRD; source_point_left_rdf_m: left optical RDF.
"""

from __future__ import annotations
from collections.abc import Mapping
from dataclasses import dataclass
from PyQt5 import QtCore
from Workstation.workstation_core.measurement_methods import PIXHAWK_RTK_LIVE
from Workstation.workstation_core.method_evaluation_workers import (
    MethodEvaluationCoordinator,
    MethodEvaluationPoll,
)
from typing import Any, Callable
from Workstation.measurement import limits
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.state import (
    UNSET,
    read,
    PairCaptureState,
    ResultCommitState,
    MeasurementPresentationState,
    WorkerRuntimeState,
)


@dataclass(frozen=True)
class WorkerRuntimeInputs:
    qt_parent: ValueInput
    append_workstation_data: ValueInput
    _log: ValueInput
    _shutdown_in_progress: ValueInput
    _measurement_id: ValueInput
    distance_recording_enabled: ValueInput


@dataclass(frozen=True)
class WorkerRuntimeActions:
    _retry_pending_primary_measurements: Callable[..., Any]
    _retry_pending_method_bundles: Callable[..., Any]
    _measurement_registry: Callable[..., Any]
    _terminalize_shutdown_pending_retries: Callable[..., Any]
    _sync_persistence_pending_status: Callable[..., Any]
    _release_detection_pair_reservation: Callable[..., Any]
    _plan_method_evaluations: Callable[..., Any]
    panel__set_distance_text: ValueInput
    _selected_method_id_for: Callable[..., Any]
    _commit_primary_method_outcome: Callable[..., Any]
    _primary_outcome_key: Callable[..., Any]
    _display_primary_method_outcome: Callable[..., Any]
    _commit_method_evaluation_bundle: Callable[..., Any]
    _method_bundle_with_primary_failure: Callable[..., Any]


class WorkerRuntimeService:
    """App-lifetime worker scheduling, recovery routing and Qt timer ownership."""

    def __init__(
        self,
        *,
        state: WorkerRuntimeState,
        inputs: WorkerRuntimeInputs,
        actions: WorkerRuntimeActions,
        capture_state: PairCaptureState,
        commit_state: ResultCommitState,
        presentation_state: MeasurementPresentationState,
    ) -> None:
        self.state = state
        self.inputs = inputs
        self.actions = actions
        self.capture_state = capture_state
        self.commit_state = commit_state
        self.presentation_state = presentation_state

    def _ensure_method_evaluation_timer(self) -> None:
        if (
            self.state._method_evaluation_timer is not None
            or QtCore.QCoreApplication.instance() is None
        ):
            return
        parent = (
            self.inputs.qt_parent.get(None)
            if self.inputs.qt_parent.get(None) is not None
            else None
        )
        timer = QtCore.QTimer(parent)
        timer.setInterval(20)
        timer.timeout.connect(self._poll_method_evaluations)
        timer.start()
        self.state._method_evaluation_timer = timer

    def _stop_method_evaluation_timer(self) -> None:
        timer = self.state._method_evaluation_timer
        if timer is None:
            return
        timer.stop()
        timer.deleteLater()
        self.state._method_evaluation_timer = None

    def _ensure_method_evaluation_coordinator(self) -> MethodEvaluationCoordinator:
        coordinator = self.state._method_evaluation_coordinator
        if coordinator is None:
            coordinator = MethodEvaluationCoordinator()
            self.state._method_evaluation_coordinator = coordinator
        self._ensure_method_evaluation_timer()
        return coordinator

    def measurement_worker_snapshot(self) -> dict[str, object]:
        coordinator = self.state._method_evaluation_coordinator
        return (
            {"state": "NOT_STARTED", "workers": []}
            if coordinator is None
            else coordinator.status_snapshot()
        )

    def _record_method_worker_recovery(self, event: Mapping[str, object]) -> None:
        append = self.inputs.append_workstation_data.get(None)
        stored = False
        if callable(append):
            try:
                stored = bool(append("events", dict(event), priority=True))
            except Exception:
                pass
        logger = self.inputs._log.get(None)
        if callable(logger):
            logger(
                f"[측정] {event.get('event_type')}: {event.get('method_id')} {event.get('status', event.get('reason', ''))}; evidence_enqueued={stored}"
            )

    def restart_measurement_method_worker(
        self, payload: Mapping[str, object]
    ) -> dict[str, object]:
        coordinator = self.state._method_evaluation_coordinator
        if coordinator is None:
            return {"accepted": False, "reason": "method runtime has not started"}
        result = coordinator.restart_method(
            str(payload.get("method_id") or ""),
            expected_worker_generation=int(payload["expected_worker_generation"]),
            expected_pair_id=str(payload.get("expected_pair_id") or ""),
            expected_request_generation=(
                None
                if payload.get("expected_request_generation") is None
                else int(payload["expected_request_generation"])
            ),
        )
        return result

    def close_method_evaluation_workers(self) -> bool:
        """Shutdown-only: stop the app-lifetime workers and drain their evidence."""
        self.actions._retry_pending_primary_measurements()
        self.actions._retry_pending_method_bundles()
        coordinator = self.state._method_evaluation_coordinator
        self.state._method_evaluation_coordinator = None
        remaining: list[MethodEvaluationCoordinator] = []
        if coordinator is not None:
            coordinator.close()
            if not coordinator.wait_closed(2.0):
                remaining.append(coordinator)
            try:
                polled = coordinator.poll()
            except Exception:
                polled = MethodEvaluationPoll()
            registry = self.actions._measurement_registry()
            if registry is not None:
                for outcome in polled.outcomes:
                    try:
                        registry.route_outcome(outcome)
                    except Exception:
                        pass
        pending_persistence = bool(
            self.commit_state._pending_primary_measurement_commits
            or self.commit_state._pending_method_bundle_commits
        )
        if bool(self.inputs._shutdown_in_progress.get(False)):
            if pending_persistence:
                self.state._method_evaluation_shutdown_close_retry_calls += 1
                if self.state._method_evaluation_shutdown_close_retry_calls >= max(
                    1, int(limits.METHOD_EVALUATION_SHUTDOWN_CLOSE_RETRY_LIMIT)
                ):
                    self.actions._terminalize_shutdown_pending_retries()
            else:
                self.state._method_evaluation_shutdown_close_retry_calls = 0
        else:
            self.state._method_evaluation_shutdown_close_retry_calls = 0
        self.actions._sync_persistence_pending_status()
        if (
            self.commit_state._pending_primary_measurement_commits
            or self.commit_state._pending_method_bundle_commits
        ):
            self._ensure_method_evaluation_timer()
        else:
            self._stop_method_evaluation_timer()
        detection_timer = self.capture_state._detection_pair_timer
        if detection_timer is not None:
            detection_timer.stop()
            detection_timer.deleteLater()
            self.capture_state._detection_pair_timer = None
        detection_coordinator = self.capture_state._detection_pair_coordinator
        self.capture_state._detection_pair_coordinator = None
        detection_closed = (
            True if detection_coordinator is None else detection_coordinator.close()
        )
        self.actions._release_detection_pair_reservation(
            read(self.capture_state._coordinated_detection_pair_reservation, None)
        )
        self.capture_state._coordinated_detection_pair_reservation = None
        preview_timer = self.capture_state._preview_measurement_timer
        if preview_timer is not None:
            preview_timer.stop()
            preview_timer.deleteLater()
            self.capture_state._preview_measurement_timer = None
        preview_coordinator = self.capture_state._preview_measurement_coordinator
        self.capture_state._preview_measurement_coordinator = None
        preview_closed = (
            True if preview_coordinator is None else preview_coordinator.close()
        )
        self.capture_state._preview_measurement_results.clear()
        self.capture_state._preview_measurement_pending.clear()
        return bool(
            not remaining
            and detection_closed
            and preview_closed
            and (not self.commit_state._pending_primary_measurement_commits)
            and (not self.commit_state._pending_method_bundle_commits)
        )

    def _multi_method_measurement_active(self) -> bool:
        return bool(
            str(self.inputs._measurement_id.get("") or "")
            and bool(self.inputs.distance_recording_enabled.get(False))
            and bool(read(self.state._method_evaluation_accepting, True))
        )

    def method_evaluation_pending_count(self) -> int:
        self._poll_method_evaluations()
        return len(self.commit_state._pending_method_bundle_commits)

    def _submit_method_evaluations(self, pair: object) -> None:
        requests, not_attempted, evidence, snapshot = (
            self.actions._plan_method_evaluations(pair)
        )
        submitted = read(self.state._method_evaluation_submitted, None)
        if not isinstance(submitted, set):
            submitted = set()
            self.state._method_evaluation_submitted = submitted
        identity = (snapshot.pair_id, snapshot.pair_sequence)
        if identity in submitted:
            return
        submitted.add(identity)
        registry = self.actions._measurement_registry()
        record = (
            None if registry is None else registry.record_for(snapshot.measurement_id)
        )
        if record is not None:
            if record.stopped:
                return
            if not record.selected_method_id:
                record.selected_method_id = str(
                    read(self.state._measurement_recording_method_id, "") or ""
                )
            try:
                record.register_pair(
                    snapshot.canonical_payload(),
                    submitted_method_ids=tuple(
                        (request.method_id for request in requests)
                    ),
                    not_attempted=not_attempted,
                    method_evidence=evidence,
                )
            except Exception as exc:
                logger = self.inputs._log.get(None)
                if callable(logger):
                    logger(f"[측정] pair 기록 실패(계속 진행): {exc}")
        if not requests:
            setter = self.actions.panel__set_distance_text.get(None)
            if callable(setter):
                setter(
                    "거리: 준비된 측정 방식 없음 (pair는 기록됨)",
                    quality="WARNING",
                    detail="; ".join((f"{k}: {v}" for (k, v) in not_attempted.items()))[
                        :400
                    ],
                )
            return
        self._ensure_method_evaluation_coordinator().submit(
            requests, planned_methods=not_attempted
        )

    def _poll_method_evaluations(self) -> None:
        self.actions._retry_pending_primary_measurements()
        self.actions._retry_pending_method_bundles()
        coordinator = self.state._method_evaluation_coordinator
        if coordinator is None:
            return
        try:
            polled = coordinator.poll()
        except Exception as exc:
            logger = self.inputs._log.get(None)
            if callable(logger):
                logger(f"[측정] method worker poll 실패(타이머 유지): {exc}")
            return
        for event in getattr(polled, "recovery_events", ()):
            self._record_method_worker_recovery(event)
        registry = self.actions._measurement_registry()
        if registry is not None:
            for outcome in polled.outcomes:
                try:
                    registry.route_outcome(outcome)
                except Exception as exc:
                    logger = self.inputs._log.get(None)
                    if callable(logger):
                        logger(f"[측정] workbook outcome 기록 실패: {exc}")
        for primary in polled.primary_outcomes:
            selected_method_id = self.actions._selected_method_id_for(
                primary.measurement_id
            )
            if selected_method_id != PIXHAWK_RTK_LIVE:
                continue
            try:
                primary, current, persistence_state = (
                    self.actions._commit_primary_method_outcome(
                        primary, commit_source="early_outcome"
                    )
                )
                if (
                    current
                    and primary.pair_sequence
                    >= self.presentation_state._primary_display_high_water
                ):
                    self.presentation_state._primary_display_high_water = (
                        primary.pair_sequence
                    )
                    self.presentation_state._primary_displayed_key = (
                        self.actions._primary_outcome_key(primary)
                    )
                    self.actions._display_primary_method_outcome(
                        primary,
                        waiting_for_barrier=True,
                        persistence_state=persistence_state,
                    )
            except Exception as exc:
                logger = self.inputs._log.get(None)
                if callable(logger):
                    logger(f"[측정] primary method 저장/표시 실패: {exc}")
        for bundle in polled.completed_bundles:
            try:
                self.actions._commit_method_evaluation_bundle(bundle)
            except Exception as exc:
                failed_bundle = self.actions._method_bundle_with_primary_failure(
                    bundle,
                    f"GUI method bundle commit failure: {type(exc).__name__}: {exc}",
                    primary_method_id=str(
                        read(self.state._measurement_recording_method_id, "")
                        or PIXHAWK_RTK_LIVE
                    ),
                )
                logger = self.inputs._log.get(None)
                if callable(logger):
                    logger(f"[측정] method bundle FAILED 처리: {exc}")
                try:
                    self.actions._commit_method_evaluation_bundle(failed_bundle)
                except Exception as nested_exc:
                    if callable(logger):
                        logger(f"[측정] FAILED bundle 기록도 실패: {nested_exc}")
        if registry is not None:
            try:
                registry.tick()
            except Exception:
                pass

    def resource_inventory(self) -> dict[str, object]:
        """Live owner handles, including queues whose names are not GUI suffixes.

        Worker processes remain actor-owned; exposing the coordinator preserves
        its sole authority to stop/join/restart them. Diagnostics may inspect
        status_snapshot(), but must not close child handles independently.
        """
        resources: dict[str, object] = {}
        for name, value in (
            ("method.timer", self.state._method_evaluation_timer),
            ("method.coordinator", self.state._method_evaluation_coordinator),
            ("pairing.timer", self.capture_state._detection_pair_timer),
            ("pairing.coordinator", self.capture_state._detection_pair_coordinator),
            ("preview.timer", self.capture_state._preview_measurement_timer),
            (
                "preview.coordinator",
                self.capture_state._preview_measurement_coordinator,
            ),
        ):
            if value is not None:
                resources[name] = value
        for name, owner, members in (
            (
                "method",
                self.state._method_evaluation_coordinator,
                ("_actor", "_commands", "_results"),
            ),
            (
                "pairing",
                self.capture_state._detection_pair_coordinator,
                ("_thread", "_input", "_results"),
            ),
            (
                "preview",
                self.capture_state._preview_measurement_coordinator,
                ("_thread", "_requests", "_results"),
            ),
        ):
            if owner is not None:
                for member in members:
                    value = getattr(owner, member, None)
                    if value is not None:
                        key = (
                            "actor_thread"
                            if member == "_actor"
                            else (
                                "worker_thread"
                                if member == "_thread"
                                else member.lstrip("_")
                            )
                        )
                        resources[f"{name}.{key}"] = value
        return resources
