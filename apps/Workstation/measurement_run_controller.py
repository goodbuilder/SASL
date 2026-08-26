"""Transient, sessionless Measurement lifecycle for Workstation."""

from __future__ import annotations

import time
import uuid
from collections.abc import Mapping, Sequence
from typing import Any

from Workstation.recording_controller import new_measurement_attempt_generation_id
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS


MEASUREMENT_RUN_STATES = frozenset(
    {"IDLE", "QUEUED", "STARTING", "RUNNING", "STOPPING", "ERROR"}
)

MEASUREMENT_RUN_METHODS = (
    "measurement_run_snapshot",
    "queue_measurement_start",
    "begin_measurement_run",
    "confirm_measurement_running",
    "request_measurement_stop",
    "finish_measurement_run",
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
        defaults = {
            "_measurement_run_state": "IDLE",
            "_measurement_run_detail": "Ready for Measurement",
            "_measurement_run_error": "",
            "_measurement_run_generation": "",
            "_measurement_run_last_generation": "",
            "_measurement_run_queued_reason": "",
            "_measurement_run_metadata": {},
            "_measurement_run_started_utc_ns": None,
            "_measurement_run_started_monotonic_ns": None,
            "_measurement_run_active_sides": (),
        }
        for name, value in defaults.items():
            if not hasattr(panel, name):
                setattr(panel, name, value)

    @property
    def state(self) -> str:
        value = str(getattr(self.panel, "_measurement_run_state", "IDLE") or "IDLE").upper()
        return value if value in MEASUREMENT_RUN_STATES else "ERROR"

    def _set_state(self, state: str, detail: str = "", error: str = "") -> None:
        normalized = str(state).upper()
        if normalized not in MEASUREMENT_RUN_STATES:
            raise ValueError(f"unsupported Measurement run state: {state}")
        self.panel._measurement_run_state = normalized
        self.panel._measurement_run_detail = str(detail)
        self.panel._measurement_run_error = str(error)
        signal = getattr(self.panel, "measurement_package_changed", None)
        emit = getattr(signal, "emit", None)
        if callable(emit):
            emit(self.measurement_run_snapshot())
        refresher = getattr(self.panel, "_refresh_operation_header", None)
        if callable(refresher):
            refresher()

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
            "schema": "workstation-measurement-run-v1",
            "state": self.state,
            "detail": str(getattr(self.panel, "_measurement_run_detail", "") or ""),
            "error": str(getattr(self.panel, "_measurement_run_error", "") or ""),
            "queued_reason": str(
                getattr(self.panel, "_measurement_run_queued_reason", "") or ""
            ),
            "generation": str(
                getattr(self.panel, "_measurement_run_generation", "") or ""
            ),
            "started_utc_ns": getattr(
                self.panel, "_measurement_run_started_utc_ns", None
            ),
            "started_monotonic_ns": getattr(
                self.panel, "_measurement_run_started_monotonic_ns", None
            ),
            "active_sides": list(
                getattr(self.panel, "_measurement_run_active_sides", ()) or ()
            ),
            "manual_recording_active": bool(
                getattr(self.panel, "_manual_recording_active", False)
            ),
            "storage": self._storage_snapshot(),
        }

    def queue_measurement_start(self, reason: str) -> bool:
        if self.state in {"RUNNING", "STOPPING"}:
            return False
        self.panel._measurement_run_queued_reason = str(reason)
        self._set_state("QUEUED", f"Measurement start queued: {reason}")
        return True

    def begin_measurement_run(
        self,
        metadata: Mapping[str, object] | None = None,
    ) -> bool:
        if self.state in {"STARTING", "RUNNING", "STOPPING"}:
            return False
        owner = getattr(self.panel, "_measurement_view_controller_owner", None)
        if owner is not None:
            # Retire prior-generation workers without waiting; their callbacks
            # are generation-filtered and cannot block this start.
            owner.close_method_evaluation_workers()
        method_config: dict[str, object] = {}
        factory = getattr(self.panel, "_measurement_view_controller", None)
        method_owner = factory() if callable(factory) else None
        builder = getattr(method_owner, "measurement_method_config_for_new_measurement", None)
        if callable(builder):
            try:
                value = builder()
                if isinstance(value, Mapping):
                    method_config = dict(value)
            except (AttributeError, TypeError, ValueError):
                method_config = {}

        generation = f"run-{uuid.uuid4().hex}"
        monotonic_before = time.monotonic_ns()
        started_utc_ns = time.time_ns()
        monotonic_after = time.monotonic_ns()
        started_monotonic_ns = (monotonic_before + monotonic_after) // 2
        self.panel._measurement_run_generation = generation
        self.panel._measurement_run_queued_reason = ""
        self.panel._measurement_run_metadata = {
            **dict(metadata or {}),
            **method_config,
        }
        self.panel._measurement_run_started_utc_ns = started_utc_ns
        self.panel._measurement_run_started_monotonic_ns = started_monotonic_ns
        self.panel._measurement_run_active_sides = ()
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
        retry = getattr(self.panel, "retry_data_storage", None)
        if callable(retry):
            try:
                retry()
            except Exception:
                pass
        append = getattr(self.panel, "append_workstation_data", None)
        if callable(append):
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
        self._set_state("STARTING", "Measurement camera/Detection start requested")
        return True

    def confirm_measurement_running(self, sides: Sequence[str]) -> bool:
        active = tuple(side for side in (str(value) for value in sides) if side in {"left", "right"})
        if self.state not in {"STARTING", "QUEUED"} or not active:
            if not active:
                self._set_state("ERROR", "No camera side started", "camera start failed")
            return False
        self.panel._measurement_run_active_sides = active
        self.panel._measurement_package_detection_started = True
        self.panel._measurement_detection_started_monotonic_s = time.monotonic()
        self._set_state(
            "RUNNING",
            "Measurement running" if len(active) == 2 else f"Measurement partial: {active[0]} only",
        )
        return True

    def request_measurement_stop(self) -> bool:
        if self.state == "STOPPING":
            return True
        if self.state not in {"STARTING", "RUNNING", "ERROR"}:
            return False
        self.panel._measurement_package_detection_completed = True
        self.panel._measurement_package_state = "POST_CAPTURE"
        self.panel._method_evaluation_accepting = False
        self._set_state("STOPPING", "Releasing camera/Detection/video owners")
        return True

    def finish_measurement_run(self) -> bool:
        generation = str(getattr(self.panel, "_measurement_run_generation", "") or "")
        append = getattr(self.panel, "append_workstation_data", None)
        if callable(append):
            append(
                "events",
                {
                    "event_type": "measurement_stopped",
                    "runtime_generation": generation,
                    "stopped_utc_ns": time.time_ns(),
                    "stopped_monotonic_ns": time.monotonic_ns(),
                    "recording_mode": "measurement",
                },
                priority=True,
            )
        flush = getattr(self.panel, "flush_workstation_data", None)
        if callable(flush):
            flush()
        self.panel._measurement_run_last_generation = generation
        self.panel._measurement_run_generation = ""
        self.panel._measurement_run_active_sides = ()
        self.panel._measurement_id = ""
        self.panel._measurement_package_state = "IDLE"
        self.panel._measurement_package_detection_started = False
        self.panel._measurement_package_detection_completed = False
        self.panel._measurement_package_finalize_requested = False
        self.panel._measurement_method_config_token = None
        self._set_state("IDLE", "Ready for next Measurement")
        return True

    def cancel_measurement_start(self, reason: str = "Measurement start cancelled") -> bool:
        if self.state not in {"QUEUED", "STARTING"}:
            return False
        self.panel._measurement_run_queued_reason = ""
        self.panel._measurement_run_generation = ""
        self.panel._measurement_id = ""
        self.panel._measurement_package_state = "IDLE"
        self._set_state("IDLE", str(reason))
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
    "MEASUREMENT_RUN_STATES",
    "MeasurementRunController",
]
