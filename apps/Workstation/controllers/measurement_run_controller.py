"""Transient, sessionless Measurement lifecycle for Workstation."""

from __future__ import annotations

import time
import uuid
from collections.abc import Mapping, Sequence
from typing import Any

from Workstation.controllers.recording_controller import new_measurement_attempt_generation_id
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS


from Workstation.workstation_core.immutable_json import thaw_json
from Workstation.workstation_core.measurement_run import (
    MEASUREMENT_RUN_STATES, MEASUREMENT_SOURCE_STATES, MEASUREMENT_SOURCE_IDS,
    MeasurementRunService,
)


MEASUREMENT_RUN_METHODS = (
    "measurement_run_snapshot",
    "queue_measurement_start",
    "begin_measurement_run",
    "measurement_source_transition",
    "confirm_measurement_running",
    "request_measurement_stop",
    "finish_measurement_run",
    "fail_measurement_start",
    "cancel_measurement_start",
    # Compatibility names used by the camera/measurement view during the
    # vertical migration. They own no persisted package or Session.
    "begin_measurement",
    "begin_next_measurement",
    "begin_measurement_detection",
    "complete_measurement_detection",
    "cancel_measurement_detection_start",
    "measurement_package_snapshot",
    "measurement_session_snapshot",
)


class MeasurementRunController:
    """Own only live command state; persistence never participates in admission."""

    def __init__(self, panel: object) -> None:
        self.panel = panel
        # Compatibility delegates can construct adapters lazily. All adapters
        # share one service; legacy panel fields are projections, never input
        # to lifecycle decisions. Remove this bridge as consumers migrate to
        # measurement_run_snapshot(), without creating a second state owner.
        owner = getattr(panel, "_measurement_run_service_owner", None)
        if owner is None:
            owner = MeasurementRunService()
            setattr(panel, "_measurement_run_service_owner", owner)
        self.run: MeasurementRunService = owner
        self._publish_run_projection()

    @property
    def state(self) -> str:
        return self.run.snapshot.state

    def _publish_run_projection(self) -> None:
        snapshot = self.run.snapshot
        for name in (
            "state", "detail", "error", "generation", "last_generation",
            "queued_reason", "metadata", "started_utc_ns", "started_monotonic_ns",
            "active_sides", "sources", "last_sources",
        ):
            setattr(self.panel, f"_measurement_run_{name}", getattr(snapshot, name))

    def _notify_run_changed(self) -> None:
        self._publish_run_projection()
        signal = getattr(self.panel, "measurement_package_changed", None)
        emit = getattr(signal, "emit", None)
        if callable(emit):
            emit(self.measurement_run_snapshot())
        refresher = getattr(self.panel, "_refresh_operation_header", None)
        if callable(refresher):
            refresher()

    def _set_state(self, state: str, detail: str = "", error: str = "") -> None:
        self.run.set_state(state, detail, error)
        self._notify_run_changed()

    def _storage_snapshot(self) -> dict[str, Any]:
        reader = getattr(self.panel, "data_storage_snapshot", None)
        if not callable(reader):
            return {"state": "UNAVAILABLE", "streams": {}}
        try:
            value = reader()
        except Exception as exc:  # storage cannot become a lifecycle exception.
            return {"state": "UNAVAILABLE", "streams": {}, "error": str(exc)}
        return dict(value) if isinstance(value, Mapping) else {"state": "UNAVAILABLE", "streams": {}}

    def measurement_run_snapshot(self) -> dict[str, Any]:
        return {
            **self.run.snapshot.to_mapping(),
            "manual_recording_active": bool(getattr(self.panel, "_manual_recording_active", False)),
            "storage": self._storage_snapshot(),
        }

    @staticmethod
    def _initial_sources() -> dict[str, dict[str, Any]]:
        return MeasurementRunService.initial_sources()

    def measurement_source_transition(
        self,
        source_id: str,
        state: str,
        detail: str = "",
        *,
        observed_utc_ns: int | None = None,
        observed_monotonic_ns: int | None = None,
    ) -> bool:
        transition = self.run.source_transition(
            source_id, state, detail,
            time.time_ns() if observed_utc_ns is None else int(observed_utc_ns),
            time.monotonic_ns() if observed_monotonic_ns is None else int(observed_monotonic_ns),
        )
        if not transition.accepted or transition.event is None:
            return transition.accepted
        self._publish_run_projection()
        append = getattr(self.panel, "append_workstation_data", None)
        if callable(append):
            try:
                append("events", thaw_json(transition.event), priority=True)
            except Exception:
                pass  # Optional evidence cannot own observation admission.
        self._notify_run_changed()
        return True

    def queue_measurement_start(self, reason: str) -> bool:
        if not self.run.queue(reason):
            return False
        self._notify_run_changed()
        return True

    def begin_measurement_run(
        self,
        metadata: Mapping[str, object] | None = None,
    ) -> bool:
        if self.state in {"STARTING", "RUNNING", "STOPPING"}:
            return False
        click_clocks = getattr(self.panel, "_measurement_start_click_clocks", None)
        self.panel._measurement_start_click_clocks = None
        if click_clocks is None:
            monotonic_before = time.monotonic_ns()
            started_utc_ns = time.time_ns()
            started_monotonic_ns = (monotonic_before + time.monotonic_ns()) // 2
        else:
            started_utc_ns, started_monotonic_ns = click_clocks
        # Method workers are application-lifetime: a previous Measurement's
        # in-flight jobs keep running and their outcomes are routed to that
        # Measurement's own record by measurement_id.
        method_config: dict[str, object] = {}
        factory = getattr(self.panel, "_measurement_view_controller", None)
        try:
            method_owner = factory() if callable(factory) else None
        except Exception:
            method_owner = None
        builder = getattr(method_owner, "measurement_method_config_for_new_measurement", None)
        begin_pairing_generation = getattr(
            method_owner,
            "begin_pairing_generation",
            None,
        )
        if callable(begin_pairing_generation):
            begin_pairing_generation()
        if callable(builder):
            try:
                value = builder()
                if isinstance(value, Mapping):
                    method_config = dict(value)
            except (AttributeError, TypeError, ValueError):
                method_config = {}

        generation = f"run-{uuid.uuid4().hex}"
        run_metadata = {**dict(metadata or {}), **method_config}
        mag_gate = getattr(self.panel, "_last_measurement_mag_gate", None)
        if isinstance(mag_gate, Mapping) and bool(mag_gate.get("required")):
            run_metadata["pixhawk_mag_selection"] = dict(mag_gate)
        if not self.run.begin(generation, started_utc_ns, started_monotonic_ns, run_metadata):
            return False
        self._publish_run_projection()
        # Compatibility-only runtime fields used by existing calculation and
        # asynchronous worker contracts. None owns storage or restart recovery.
        self.panel._measurement_id = generation
        self.panel._measurement_attempt_generation_id = new_measurement_attempt_generation_id(
            generation
        )
        self.panel._measurement_started_monotonic_s = started_monotonic_ns * 1e-9
        self.panel._measurement_detection_started_monotonic_s = None
        self.panel._measurement_package = None
        self.panel._measurement_package_state = "COLLECTING"
        self.panel._measurement_package_detection_started = False
        self.panel._measurement_package_detection_completed = False
        self.panel._measurement_package_finalize_requested = False
        self.panel._measurement_method_config_token = None
        self.panel._last_frozen_measurement_pair = None
        self.panel._method_evaluation_submitted = set()
        self.panel._method_evaluation_committed = set()
        self.panel._method_evaluation_accepting = True
        self.panel._last_method_evaluation_bundle = None
        self.panel._measurement_method_result_counters = {
            method_id: {"VALID": 0, "BLOCKED": 0, "FAILED": 0}
            for method_id in MEASUREMENT_METHOD_IDS
        }
        self._notify_run_changed()
        self._begin_measurement_record(
            generation,
            started_utc_ns=started_utc_ns,
            started_monotonic_ns=started_monotonic_ns,
        )
        retry = getattr(self.panel, "retry_data_storage", None)
        if callable(retry):
            try:
                retry()
            except Exception:
                pass
        append = getattr(self.panel, "append_workstation_data", None)
        if callable(append):
            try:
                append(
                    "events",
                    {
                        "event_type": "measurement_start_requested",
                        "runtime_generation": generation,
                        "started_utc_ns": started_utc_ns,
                        "started_monotonic_ns": started_monotonic_ns,
                        "recording_mode": "measurement",
                    },
                    priority=True,
                )
            except Exception:
                pass
        self.panel._measurement_package_detection_started = True
        self.panel._measurement_detection_started_monotonic_s = (
            started_monotonic_ns * 1.0e-9
        )
        self._set_state(
            "RUNNING",
            "Measurement observing; sources may join independently",
        )
        return True

    def confirm_measurement_running(self, sides: Sequence[str]) -> bool:
        active = tuple(side for side in (str(value) for value in sides) if side in {"left", "right"})
        if self.state not in {"STARTING", "QUEUED", "RUNNING"}:
            return False
        for side in active:
            self.measurement_source_transition(
                f"camera.{side}",
                "LIVE",
                "current-generation decoded frame confirmed",
            )
        return True

    def request_measurement_stop(self) -> bool:
        if self.state == "IDLE":
            return True
        if not self.run.mark_stopping():
            return False
        self.panel._measurement_package_detection_completed = True
        self.panel._measurement_package_state = "POST_CAPTURE"
        self.panel._method_evaluation_accepting = False
        self._notify_run_changed()
        # The observation boundary is complete once its end timestamp and
        # generation are frozen. Camera/video teardown continues under its
        # own owner and must not hold admission for the next run.
        return self.finish_measurement_run()

    def finish_measurement_run(self, generation: str | None = None) -> bool:
        current_generation = self.run.snapshot.generation
        expected_generation = str(generation or current_generation)
        if generation is not None and expected_generation != current_generation:
            return True
        generation = current_generation
        if not generation:
            return self.state == "IDLE"
        stopped_utc_ns = time.time_ns()
        stopped_monotonic_ns = time.monotonic_ns()
        # Release admission before touching optional persistence. Even a
        # synchronous writer/flush exception cannot strand the lifecycle in
        # STOPPING or retain the old generation.
        if self.run.finish(generation) is None:
            return self.state == "IDLE"
        self.panel._measurement_id = ""
        self.panel._measurement_package_state = "IDLE"
        self.panel._measurement_package_detection_started = False
        self.panel._measurement_package_detection_completed = False
        self.panel._measurement_package_finalize_requested = False
        self.panel._measurement_method_config_token = None
        self._notify_run_changed()
        self._stop_measurement_record(
            generation,
            stopped_utc_ns=stopped_utc_ns,
            stopped_monotonic_ns=stopped_monotonic_ns,
        )
        append = getattr(self.panel, "append_workstation_data", None)
        if callable(append):
            try:
                append(
                    "events",
                    {
                        "event_type": "measurement_stopped",
                        "runtime_generation": generation,
                        "stopped_utc_ns": stopped_utc_ns,
                        "stopped_monotonic_ns": stopped_monotonic_ns,
                        "recording_mode": "measurement",
                    },
                    priority=True,
                )
            except Exception:
                pass
        flush = getattr(self.panel, "flush_workstation_data", None)
        if callable(flush):
            try:
                flush()
            except Exception:
                pass
        return True

    def _measurement_registry(self) -> Any | None:
        getter = getattr(self.panel, "measurement_registry", None)
        if not callable(getter):
            return None
        try:
            registry = getter()
        except Exception:
            return None
        return registry if hasattr(registry, "begin") and hasattr(registry, "stop") else None

    def _begin_measurement_record(
        self,
        generation: str,
        *,
        started_utc_ns: int,
        started_monotonic_ns: int,
    ) -> Any | None:
        """Open the per-Measurement workbook record; never gates the Start."""

        self.panel._measurement_file_prefix = ""
        registry = self._measurement_registry()
        if registry is None:
            return None
        owner = getattr(self.panel, "_measurement_view_controller_owner", None)
        calibration: dict[str, object] = {}
        alignment_rows: list[Mapping[str, object]] = []
        try:
            snapshot = getattr(owner, "measurement_calibration_snapshot", None)
            if callable(snapshot):
                calibration = dict(snapshot() or {})
        except Exception:
            calibration = {}
        try:
            rows = getattr(owner, "measurement_alignment_rows", None)
            if callable(rows):
                alignment_rows = list(rows() or ())
        except Exception:
            alignment_rows = []
        try:
            record = registry.begin(
                generation,
                started_utc_ns=started_utc_ns,
                started_monotonic_ns=started_monotonic_ns,
                metadata=thaw_json(self.run.snapshot.metadata),
                calibration=calibration,
            )
        except Exception:
            return None
        for row in alignment_rows:
            try:
                record.add_alignment_row(row)
            except Exception:
                continue
        self.panel._measurement_file_prefix = str(record.file_prefix)
        self.panel._measurement_method_result_counters = record.counters
        return record

    def _stop_measurement_record(
        self,
        generation: str,
        *,
        stopped_utc_ns: int,
        stopped_monotonic_ns: int,
    ) -> None:
        self.panel._measurement_file_prefix = ""
        registry = self._measurement_registry()
        if registry is None:
            return
        try:
            registry.stop(
                generation,
                stopped_utc_ns=stopped_utc_ns,
                stopped_monotonic_ns=stopped_monotonic_ns,
            )
        except Exception:
            # The workbook is evidence, never an admission gate.
            pass

    def fail_measurement_start(self, reason: str) -> bool:
        """Keep the observation open when camera admission fails."""

        if self.state not in {"QUEUED", "STARTING", "RUNNING"}:
            return False
        failure = str(reason or "Measurement camera start failed")
        for side in ("left", "right"):
            source_id = f"camera.{side}"
            current = self.run.snapshot.sources.get(source_id, {})
            if current.get("state") != "LIVE":
                self.measurement_source_transition(source_id, "UNAVAILABLE", failure)
                self.measurement_source_transition(
                    f"recording.{side}", "UNAVAILABLE", failure
                )
        self._set_state("RUNNING", f"Measurement observing; camera unavailable: {failure}")
        return True

    def cancel_measurement_start(self, reason: str = "Measurement start cancelled") -> bool:
        if not self.run.cancel(reason):
            return False
        self.panel._measurement_id = ""
        self.panel._measurement_package_state = "IDLE"
        self.panel._measurement_package_detection_started = False
        self.panel._method_evaluation_accepting = False
        self._notify_run_changed()
        return True

    def begin_measurement(self, metadata: Mapping[str, object] | None = None) -> bool:
        return self.begin_measurement_run(metadata)

    def begin_next_measurement(self, metadata: Mapping[str, object] | None = None) -> bool:
        return self.begin_measurement_run(metadata)

    def begin_measurement_detection(self) -> bool:
        return self.state in {"STARTING", "QUEUED", "RUNNING"}

    def complete_measurement_detection(self) -> bool:
        return self.request_measurement_stop()

    def cancel_measurement_detection_start(self, reason: str = "Detection did not start") -> bool:
        return self.cancel_measurement_start(reason)

    def measurement_package_snapshot(self) -> dict[str, Any]:
        run = self.measurement_run_snapshot()
        state = {
            "IDLE": "IDLE",
            "QUEUED": "COLLECTING",
            "STARTING": "COLLECTING",
            "RUNNING": "COLLECTING",
            "STOPPING": "POST_CAPTURE",
            "ERROR": "RECOVERABLE",
        }[run["state"]]
        return {
            **run,
            "schema": "workstation-sessionless-measurement-compat-v1",
            "state": state,
            "measurement_id": run["generation"],
            "detection_started": run["state"] in {"RUNNING", "STOPPING"},
            "detection_completed": run["state"] == "STOPPING",
            "evidence_completeness": {"status": "NOT_APPLICABLE", "complete": None},
        }

    def measurement_session_snapshot(self) -> dict[str, Any]:
        return {
            "schema": "workstation-sessionless-session-compat-v1",
            "state": "REMOVED",
            "session_id": "",
            "active_measurement_id": None,
            "measurements": [],
            "current_measurement": self.measurement_package_snapshot(),
        }


__all__ = [
    "MEASUREMENT_RUN_METHODS",
    "MEASUREMENT_SOURCE_IDS",
    "MEASUREMENT_SOURCE_STATES",
    "MEASUREMENT_RUN_STATES",
    "MeasurementRunController",
]
