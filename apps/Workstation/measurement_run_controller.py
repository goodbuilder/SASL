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

MEASUREMENT_SOURCE_STATES = frozenset(
    {"PENDING", "WAITING_RESOURCE", "LIVE", "UNAVAILABLE", "LEFT"}
)
MEASUREMENT_SOURCE_IDS = (
    "camera.left",
    "camera.right",
    "recording.left",
    "recording.right",
    "pixhawk.left",
    "pixhawk.right",
    "gnss.base",
    "ntrip",
    "target",
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
            "_measurement_run_sources": {},
            "_measurement_run_last_sources": {},
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
        sources = getattr(self.panel, "_measurement_run_sources", {})
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
            "sources": {
                str(source_id): dict(value)
                for source_id, value in dict(sources or {}).items()
                if isinstance(value, Mapping)
            },
            "manual_recording_active": bool(
                getattr(self.panel, "_manual_recording_active", False)
            ),
            "storage": self._storage_snapshot(),
        }

    @staticmethod
    def _initial_sources() -> dict[str, dict[str, Any]]:
        return {
            source_id: {
                "state": "PENDING",
                "detail": "awaiting source evidence",
                "joined_utc_ns": None,
                "joined_monotonic_ns": None,
                "left_utc_ns": None,
                "left_monotonic_ns": None,
            }
            for source_id in MEASUREMENT_SOURCE_IDS
        }

    def measurement_source_transition(
        self,
        source_id: str,
        state: str,
        detail: str = "",
        *,
        observed_utc_ns: int | None = None,
        observed_monotonic_ns: int | None = None,
    ) -> bool:
        """Publish one source transition without changing run admission.

        Source evidence is meaningful only while an observation generation is
        live.  Device owners may fail or recover independently; none owns the
        Measurement RUNNING state.
        """

        generation = str(
            getattr(self.panel, "_measurement_run_generation", "") or ""
        )
        if not generation or self.state not in {"RUNNING", "STOPPING"}:
            return False
        source = str(source_id).strip()
        normalized = str(state).strip().upper()
        if not source:
            raise ValueError("measurement source_id is required")
        if source not in MEASUREMENT_SOURCE_IDS:
            raise ValueError(f"unsupported Measurement source_id: {source}")
        if normalized not in MEASUREMENT_SOURCE_STATES:
            raise ValueError(f"unsupported Measurement source state: {state}")
        utc_ns = time.time_ns() if observed_utc_ns is None else int(observed_utc_ns)
        monotonic_ns = (
            time.monotonic_ns()
            if observed_monotonic_ns is None
            else int(observed_monotonic_ns)
        )
        sources = getattr(self.panel, "_measurement_run_sources", None)
        if not isinstance(sources, dict):
            sources = self._initial_sources()
            self.panel._measurement_run_sources = sources
        previous = dict(sources.get(source) or {})
        if (
            previous.get("state") == normalized
            and str(previous.get("detail") or "") == str(detail)
        ):
            return True
        row = {
            "state": normalized,
            "detail": str(detail),
            "joined_utc_ns": previous.get("joined_utc_ns"),
            "joined_monotonic_ns": previous.get("joined_monotonic_ns"),
            "left_utc_ns": previous.get("left_utc_ns"),
            "left_monotonic_ns": previous.get("left_monotonic_ns"),
        }
        if normalized == "LIVE" and previous.get("state") != "LIVE":
            row["joined_utc_ns"] = utc_ns
            row["joined_monotonic_ns"] = monotonic_ns
            row["left_utc_ns"] = None
            row["left_monotonic_ns"] = None
        elif normalized in {"UNAVAILABLE", "LEFT"}:
            row["left_utc_ns"] = utc_ns
            row["left_monotonic_ns"] = monotonic_ns
        sources[source] = row
        active_sides = tuple(
            side
            for side in ("left", "right")
            if dict(sources.get(f"camera.{side}") or {}).get("state") == "LIVE"
        )
        self.panel._measurement_run_active_sides = active_sides
        append = getattr(self.panel, "append_workstation_data", None)
        if callable(append):
            try:
                append(
                    "events",
                    {
                        "event_type": "measurement_source_transition",
                        "runtime_generation": generation,
                        "source_id": source,
                        "source_state": normalized,
                        "detail": str(detail),
                        "observed_utc_ns": utc_ns,
                        "observed_monotonic_ns": monotonic_ns,
                    },
                    priority=True,
                )
            except Exception:
                pass
        if self.state == "RUNNING":
            live_count = sum(
                1
                for value in sources.values()
                if isinstance(value, Mapping) and value.get("state") == "LIVE"
            )
            self._set_state(
                "RUNNING",
                f"Measurement observing; live sources={live_count}",
            )
        return True

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
        mag_gate = getattr(self.panel, "_last_measurement_mag_gate", None)
        if isinstance(mag_gate, Mapping) and bool(mag_gate.get("required")):
            self.panel._measurement_run_metadata["pixhawk_mag_selection"] = dict(mag_gate)
        self.panel._measurement_run_started_utc_ns = started_utc_ns
        self.panel._measurement_run_started_monotonic_ns = started_monotonic_ns
        self.panel._measurement_run_active_sides = ()
        self.panel._measurement_run_sources = self._initial_sources()
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
        if self.state not in {"STARTING", "RUNNING", "ERROR"}:
            return False
        self.panel._measurement_package_detection_completed = True
        self.panel._measurement_package_state = "POST_CAPTURE"
        self.panel._method_evaluation_accepting = False
        self._set_state("STOPPING", "Releasing camera/Detection/video owners")
        # The observation boundary is complete once its end timestamp and
        # generation are frozen. Camera/video teardown continues under its
        # own owner and must not hold admission for the next run.
        return self.finish_measurement_run()

    def finish_measurement_run(self, generation: str | None = None) -> bool:
        current_generation = str(
            getattr(self.panel, "_measurement_run_generation", "") or ""
        )
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
        self.panel._measurement_run_last_generation = generation
        self.panel._measurement_run_last_sources = {
            source_id: dict(value)
            for source_id, value in dict(
                getattr(self.panel, "_measurement_run_sources", {}) or {}
            ).items()
            if isinstance(value, Mapping)
        }
        self.panel._measurement_run_generation = ""
        self.panel._measurement_run_active_sides = ()
        self.panel._measurement_run_sources = {}
        self.panel._measurement_id = ""
        self.panel._measurement_package_state = "IDLE"
        self.panel._measurement_package_detection_started = False
        self.panel._measurement_package_detection_completed = False
        self.panel._measurement_package_finalize_requested = False
        self.panel._measurement_method_config_token = None
        self._set_state("IDLE", "Ready for next Measurement")
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
                metadata=dict(getattr(self.panel, "_measurement_run_metadata", {}) or {}),
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
            current = dict(
                getattr(self.panel, "_measurement_run_sources", {}).get(
                    source_id, {}
                )
            )
            if current.get("state") != "LIVE":
                self.measurement_source_transition(source_id, "UNAVAILABLE", failure)
                self.measurement_source_transition(
                    f"recording.{side}", "UNAVAILABLE", failure
                )
        self._set_state("RUNNING", f"Measurement observing; camera unavailable: {failure}")
        return True

    def cancel_measurement_start(self, reason: str = "Measurement start cancelled") -> bool:
        if self.state not in {"QUEUED", "STARTING"}:
            return False
        self.panel._measurement_run_queued_reason = ""
        self.panel._measurement_run_generation = ""
        self.panel._measurement_run_active_sides = ()
        self.panel._measurement_id = ""
        self.panel._measurement_package_state = "IDLE"
        self.panel._measurement_package_detection_started = False
        self.panel._method_evaluation_accepting = False
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
    "MEASUREMENT_SOURCE_IDS",
    "MEASUREMENT_SOURCE_STATES",
    "MEASUREMENT_RUN_STATES",
    "MeasurementRunController",
]
