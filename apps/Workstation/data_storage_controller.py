"""GUI-facing owner for sessionless Workstation measurement storage."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path
import threading
import time
from typing import Any

from Workstation.runtime_paths import WORKSTATION_MEASUREMENT_DATA_ROOT
from Workstation.workstation_core.measurement_data_store import (
    MeasurementDataStore,
    allocate_timestamped_path,
)
from Workstation.workstation_core.latest_measurement_xlsx import (
    LATEST_MEASUREMENT_WORKBOOK_NAME,
    LatestMeasurementExportCancelled,
    workbook_identity,
    timestamped_workbook_name,
    write_latest_measurement_xlsx,
)


DEFAULT_MEASUREMENT_DATA_ROOT = ""
LEGACY_DEFAULT_MEASUREMENT_DATA_ROOT = "measurement_data"
LATEST_MEASUREMENT_XLSX_STATUS_SCHEMA = (
    "workstation-latest-measurement-xlsx-status-v1"
)
LATEST_MEASUREMENT_XLSX_QUIET_S = 2.0
LATEST_MEASUREMENT_XLSX_CHECKPOINT_TIMEOUT_S = 5.0
LATEST_MEASUREMENT_XLSX_PARTIAL_RETRY_S = 5.0

DATA_STORAGE_CONTROLLER_METHODS = (
    "append_workstation_data",
    "data_storage_snapshot",
    "retry_data_storage",
    "allocate_workstation_data_artifact",
    "flush_workstation_data",
    "close_workstation_data",
    "request_latest_measurement_xlsx",
    "latest_measurement_xlsx_snapshot",
    "cancel_latest_measurement_xlsx_export",
)


@dataclass(frozen=True)
class _LatestMeasurementXlsxTarget:
    store: MeasurementDataStore
    generation: str | None
    output_name: str
    started_utc_ns: int | None
    stopped_utc_ns: int | None
    source_states: Mapping[str, object]
    automatic: bool
    requested_utc_ns: int
    revision: int


def _payload_generation(payload: Mapping[str, object]) -> str:
    for key in ("runtime_generation", "measurement_id"):
        value = payload.get(key)
        if value not in (None, ""):
            return str(value)
    snapshot = payload.get("snapshot_payload")
    if isinstance(snapshot, Mapping) and snapshot.get("measurement_id") not in (
        None,
        "",
    ):
        return str(snapshot["measurement_id"])
    return ""


def _allocate_timestamped_workbook_name(root: Path, utc_ns: int) -> str:
    sequence = 0
    while True:
        name = timestamped_workbook_name(utc_ns, sequence=sequence)
        candidate = root / name
        if not candidate.exists() and not candidate.is_symlink():
            return name
        sequence += 1


class _LatestMeasurementXlsxCoordinator:
    """Own one coalesced, cancellable latest-Measurement export worker."""

    def __init__(self, controller: "DataStorageController") -> None:
        self.controller = controller
        self.panel = controller.panel
        self._condition = threading.Condition(threading.RLock())
        self._thread: threading.Thread | None = None
        self._stop_requested = False
        self._target: _LatestMeasurementXlsxTarget | None = None
        self._revision = 0
        self._dirty = False
        self._last_matching_enqueue_s = 0.0
        self._next_attempt_s = 0.0
        self._cancel_event = threading.Event()
        root = controller._configured_root()
        candidates = [
            path
            for path in root.glob("latest_measurement_*.xlsx")
            if path.is_file() and not path.is_symlink()
        ]
        fixed = root / LATEST_MEASUREMENT_WORKBOOK_NAME
        if fixed.is_file() and not fixed.is_symlink():
            candidates.append(fixed)
        candidates.sort(key=lambda path: (path.stat().st_mtime_ns, path.name), reverse=True)
        output = candidates[0] if candidates else fixed
        state = "NOT_CREATED"
        published_generation = ""
        error = ""
        owned = next(
            (
                (candidate, workbook_identity(candidate))
                for candidate in candidates
                if workbook_identity(candidate).get("runtime_generation")
            ),
            None,
        )
        if owned is not None:
            output, identity = owned
            published_generation = str(identity.get("runtime_generation") or "")
            state = "READY"
        elif candidates:
            state = "FAILED"
            error = "existing latest Measurement XLSX is not Workstation-owned"
        self._status: dict[str, object] = {
            "schema": LATEST_MEASUREMENT_XLSX_STATUS_SCHEMA,
            "state": state,
            "output_path": str(output),
            "target_generation": published_generation,
            "published_generation": published_generation,
            "requested_utc_ns": None,
            "generated_utc_ns": None,
            "row_counts": {},
            "issue_count": 0,
            "checkpoint_settled": None,
            "checkpoint_pending": 0,
            "completion_evidence": "existing_workbook" if published_generation else "",
            "error": error,
        }

    def _recording_active(self) -> bool:
        state = str(
            getattr(self.panel, "_measurement_run_state", "IDLE") or "IDLE"
        ).upper()
        return bool(
            state in {"QUEUED", "STARTING", "RUNNING", "STOPPING"}
            or getattr(self.panel, "_manual_recording_active", False)
            or getattr(self.panel, "distance_recording_enabled", False)
            or getattr(self.panel, "video_recording_enabled", False)
        )

    def _ensure_thread_unlocked(self) -> bool:
        if self._stop_requested:
            return False
        thread = self._thread
        if thread is not None and thread.is_alive():
            return True
        thread = threading.Thread(
            target=self._run,
            name="workstation-latest-measurement-xlsx",
            daemon=True,
        )
        self._thread = thread
        try:
            setattr(self.panel, "_latest_measurement_xlsx_thread", thread)
        except (AttributeError, TypeError):
            pass
        thread.start()
        return True

    def request(
        self,
        generation: str | None = None,
        *,
        started_utc_ns: int | None = None,
        stopped_utc_ns: int | None = None,
        source_states: Mapping[str, object] | None = None,
        automatic: bool = False,
    ) -> bool:
        selected = str(generation or "").strip() or str(
            getattr(self.panel, "_measurement_run_last_generation", "") or ""
        ).strip()
        selected_or_none = selected or None
        store = self.controller.store
        now_ns = time.time_ns()
        output_name = _allocate_timestamped_workbook_name(store.root, now_ns)
        now_s = time.monotonic()
        with self._condition:
            if self._stop_requested:
                return False
            was_generating = str(self._status.get("state")) == "GENERATING"
            self._revision += 1
            previous = self._target
            self._target = _LatestMeasurementXlsxTarget(
                store=store,
                generation=selected_or_none,
                output_name=output_name,
                started_utc_ns=(
                    None if started_utc_ns is None else int(started_utc_ns)
                ),
                stopped_utc_ns=(
                    None if stopped_utc_ns is None else int(stopped_utc_ns)
                ),
                source_states=dict(source_states or {}),
                automatic=bool(automatic),
                requested_utc_ns=now_ns,
                revision=self._revision,
            )
            self._dirty = True
            self._last_matching_enqueue_s = (
                now_s if automatic else now_s - LATEST_MEASUREMENT_XLSX_QUIET_S
            )
            self._next_attempt_s = now_s
            published = str(self._status.get("published_generation") or "")
            self._status.update(
                state="STALE" if published else "WAITING",
                output_path=str(store.root / output_name),
                target_generation=selected_or_none or "",
                requested_utc_ns=now_ns,
                error="",
            )
            if (
                previous is not None
                and previous.generation != selected_or_none
                and was_generating
            ):
                self._cancel_event.set()
            if not self._ensure_thread_unlocked():
                return False
            self._condition.notify_all()
        return True

    def note_record(
        self,
        payload: Mapping[str, object],
        *,
        admitted: bool,
    ) -> None:
        generation = _payload_generation(payload)
        now_s = time.monotonic()
        with self._condition:
            target = self._target
            if target is None:
                return
            if self._recording_active() and str(self._status.get("state")) == "GENERATING":
                self._cancel_event.set()
            if not target.generation or generation != target.generation:
                return
            self._dirty = True
            self._last_matching_enqueue_s = now_s
            if str(self._status.get("state")) in {"READY", "GENERATING"}:
                self._status["state"] = "STALE"
            if not admitted:
                self._status["error"] = "a matching source record was not admitted"
            self._condition.notify_all()

    def snapshot(self) -> dict[str, object]:
        with self._condition:
            value = dict(self._status)
            value["row_counts"] = dict(value.get("row_counts") or {})
            value["worker_alive"] = bool(
                self._thread is not None and self._thread.is_alive()
            )
            value["recording_active"] = self._recording_active()
            return value

    def uses_store(self, store: MeasurementDataStore | None) -> bool:
        if store is None:
            return False
        with self._condition:
            return bool(
                self._target is not None
                and self._target.store is store
                and (
                    self._dirty
                    or str(self._status.get("state"))
                    in {"WAITING", "GENERATING", "STALE"}
                )
            )

    def close(self, *, timeout_s: float = 2.0) -> bool:
        with self._condition:
            self._stop_requested = True
            self._dirty = False
            self._cancel_event.set()
            thread = self._thread
            self._condition.notify_all()
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, float(timeout_s)))
        return thread is None or not thread.is_alive()

    def _run(self) -> None:
        current_thread = threading.current_thread()
        try:
            while True:
                with self._condition:
                    target: _LatestMeasurementXlsxTarget | None = None
                    while not self._stop_requested:
                        candidate = self._target
                        if candidate is None or not self._dirty:
                            self._condition.wait(timeout=0.5)
                            continue
                        if self._recording_active():
                            self._status["state"] = (
                                "STALE"
                                if self._status.get("published_generation")
                                else "WAITING"
                            )
                            self._condition.wait(timeout=0.25)
                            continue
                        now_s = time.monotonic()
                        quiet_s = (
                            LATEST_MEASUREMENT_XLSX_QUIET_S
                            if candidate.automatic
                            else 0.0
                        )
                        ready_s = max(
                            self._next_attempt_s,
                            self._last_matching_enqueue_s + quiet_s,
                        )
                        if now_s < ready_s:
                            self._condition.wait(timeout=min(0.5, ready_s - now_s))
                            continue
                        target = candidate
                        self._dirty = False
                        self._cancel_event = threading.Event()
                        self._status.update(
                            state="GENERATING",
                            target_generation=target.generation or "",
                            error="",
                        )
                        break
                    if self._stop_requested:
                        return
                    assert target is not None
                    cancel_event = self._cancel_event
                try:
                    checkpoint = target.store.export_checkpoint(
                        timeout_s=LATEST_MEASUREMENT_XLSX_CHECKPOINT_TIMEOUT_S
                    )
                    result = write_latest_measurement_xlsx(
                        checkpoint.root,
                        generation=target.generation,
                        output_name=target.output_name,
                        started_utc_ns=target.started_utc_ns,
                        stopped_utc_ns=target.stopped_utc_ns,
                        completion_evidence=(
                            "measurement_stopped"
                            if target.generation is not None
                            else "discovered"
                        ),
                        stream_byte_limits=checkpoint.stream_byte_limits,
                        stream_paths=checkpoint.stream_paths,
                        stream_byte_limits_by_path=(
                            checkpoint.stream_byte_limits_by_path
                        ),
                        storage_snapshot=checkpoint.storage_snapshot,
                        source_states=target.source_states,
                        checkpoint_settled=checkpoint.settled,
                        checkpoint_pending=checkpoint.pending_before_cutoff,
                        cancel_event=cancel_event,
                    )
                except LatestMeasurementExportCancelled:
                    with self._condition:
                        if self._stop_requested:
                            self._status["state"] = "CANCELLED"
                            return
                        self._dirty = True
                        self._next_attempt_s = time.monotonic() + 0.5
                        self._status["state"] = (
                            "STALE"
                            if self._status.get("published_generation")
                            else "WAITING"
                        )
                        self._condition.notify_all()
                    continue
                except Exception as exc:  # noqa: BLE001 - failure is a UI read model.
                    with self._condition:
                        if self._target is not None and self._target.revision != target.revision:
                            self._dirty = True
                            self._status["state"] = (
                                "STALE"
                                if self._status.get("published_generation")
                                else "WAITING"
                            )
                        else:
                            self._dirty = False
                            self._status["state"] = "FAILED"
                        self._status["error"] = str(exc)
                        self._condition.notify_all()
                    continue
                with self._condition:
                    self._status.update(
                        output_path=str(result.path),
                        published_generation=result.generation,
                        generated_utc_ns=result.generated_utc_ns,
                        row_counts=dict(result.row_counts),
                        issue_count=result.issue_count,
                        checkpoint_settled=result.checkpoint_settled,
                        checkpoint_pending=result.checkpoint_pending,
                        completion_evidence=result.completion_evidence,
                        error="",
                    )
                    if self._target is not None and self._target.revision == target.revision:
                        if self._target.generation is None:
                            self._target = replace(
                                self._target,
                                generation=result.generation,
                            )
                        self._status["target_generation"] = result.generation
                    superseded = bool(
                        self._target is not None
                        and self._target.revision != target.revision
                    )
                    if not result.checkpoint_settled:
                        self._dirty = True
                        self._next_attempt_s = (
                            time.monotonic()
                            + LATEST_MEASUREMENT_XLSX_PARTIAL_RETRY_S
                        )
                    if superseded or self._dirty:
                        self._status["state"] = "STALE"
                    else:
                        self._status["state"] = "READY"
                    self._condition.notify_all()
        finally:
            with self._condition:
                if self._thread is current_thread:
                    self._thread = None
                try:
                    if getattr(self.panel, "_latest_measurement_xlsx_thread", None) is current_thread:
                        setattr(self.panel, "_latest_measurement_xlsx_thread", None)
                except (AttributeError, TypeError):
                    pass
                self._condition.notify_all()


class DataStorageController:
    """Keep storage failures observable and outside Measurement admission."""

    def __init__(self, panel: object) -> None:
        self.panel = panel
        self._store: MeasurementDataStore | None = None
        self._root: Path | None = None
        self._configuration_error = ""
        self._latest_xlsx = _LatestMeasurementXlsxCoordinator(self)

    def _default_root(self) -> Path:
        configured = getattr(
            self.panel,
            "default_measurement_data_root",
            WORKSTATION_MEASUREMENT_DATA_ROOT,
        )
        return Path(configured).expanduser().resolve(strict=False)

    def _configured_root(self) -> Path:
        setting_reader = getattr(self.panel, "_workstation_setting", None)
        settings = getattr(self.panel, "settings", None)
        default = getattr(settings, "measurement_data_root", DEFAULT_MEASUREMENT_DATA_ROOT)
        raw = (
            setting_reader("measurement_data_root", default)
            if callable(setting_reader)
            else default
        )
        text = str(raw or DEFAULT_MEASUREMENT_DATA_ROOT).strip()
        if not text or text == LEGACY_DEFAULT_MEASUREMENT_DATA_ROOT:
            self._configuration_error = ""
            return self._default_root()
        base = Path(getattr(self.panel, "app_root", Path.cwd())).expanduser().resolve()
        path = Path(text).expanduser()
        if path.is_absolute():
            self._configuration_error = (
                "measurement_data_root must be a portable relative path; "
                f"using default recordings root instead of {text!r}"
            )
            return self._default_root()
        resolved = (base / path).resolve(strict=False)
        try:
            resolved.relative_to(base)
        except ValueError:
            # The public setting is intentionally portable and app-relative.
            # A stale/hand-edited invalid value must never turn into a
            # Measurement admission failure, so fall back to the safe default
            # and expose the problem only as storage diagnostics.
            self._configuration_error = (
                "measurement_data_root must stay inside the Workstation app root; "
                f"using default recordings root instead of {text!r}"
            )
            return self._default_root()
        self._configuration_error = ""
        return resolved

    @property
    def store(self) -> MeasurementDataStore:
        root = self._configured_root().resolve(strict=False)
        if self._store is None:
            self._root = root
            self._store = MeasurementDataStore(root, timestamped_filenames=True)
        elif self._root != root:
            active = bool(
                getattr(self.panel, "distance_recording_enabled", False)
                or getattr(self.panel, "video_recording_enabled", False)
            )
            if not active and not self._latest_xlsx.uses_store(self._store):
                self._store.close(timeout_s=2.0)
                self._root = root
                self._store = MeasurementDataStore(root, timestamped_filenames=True)
        return self._store

    def append_workstation_data(
        self,
        stream: str,
        payload: Mapping[str, Any],
        *,
        priority: bool | None = None,
    ) -> bool:
        admitted = False
        try:
            admitted = bool(self.store.enqueue(stream, dict(payload), priority=priority))
        except (OSError, TypeError, ValueError):
            # Storage is explicitly best-effort. The per-stream snapshot owns
            # detailed failures once a store exists; setup errors cannot become
            # a Measurement command rejection.
            admitted = False
        self._latest_xlsx.note_record(payload, admitted=admitted)
        return admitted

    def data_storage_snapshot(self) -> dict[str, Any]:
        try:
            snapshot = self.store.snapshot()
            if self._configuration_error:
                snapshot = dict(snapshot)
                snapshot["configuration_error"] = self._configuration_error
                if snapshot.get("state") == "OK":
                    snapshot["state"] = "DEGRADED"
            return snapshot
        except (OSError, TypeError, ValueError) as exc:
            return {
                "schema": "workstation-measurement-data-store-v1",
                "state": "UNAVAILABLE",
                "root": str(self._configured_root()),
                "priority_pending": 0,
                "bulk_pending": 0,
                "dropped_records": 0,
                "csv_retry_pending": 0,
                "csv_retry_bytes": 0,
                "csv_dropped_records": 0,
                "writer_alive": False,
                "streams": {},
                "error": str(exc),
            }

    def retry_data_storage(self) -> dict[str, Any]:
        try:
            self.store.retry_failed()
        except (OSError, TypeError, ValueError):
            pass
        return self.data_storage_snapshot()

    def allocate_workstation_data_artifact(
        self,
        category: str,
        *,
        label: str,
        suffix: str,
        utc_ns: int | None = None,
    ) -> Path:
        return allocate_timestamped_path(
            self._configured_root(),
            category,
            label=label,
            suffix=suffix,
            utc_ns=utc_ns,
        )

    def flush_workstation_data(self) -> bool:
        store = self._store
        if store is None:
            return True
        store.request_flush()
        return True

    def close_workstation_data(self, *, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        if not self._latest_xlsx.close(timeout_s=max(0.0, deadline - time.monotonic())):
            return False
        store = self._store
        if store is None:
            return True
        return bool(store.close(timeout_s=max(0.0, deadline - time.monotonic())))

    def request_latest_measurement_xlsx(
        self,
        generation: str | None = None,
        *,
        started_utc_ns: int | None = None,
        stopped_utc_ns: int | None = None,
        source_states: Mapping[str, object] | None = None,
        automatic: bool = False,
    ) -> bool:
        return self._latest_xlsx.request(
            generation,
            started_utc_ns=started_utc_ns,
            stopped_utc_ns=stopped_utc_ns,
            source_states=source_states,
            automatic=automatic,
        )

    def latest_measurement_xlsx_snapshot(self) -> dict[str, object]:
        return self._latest_xlsx.snapshot()

    def cancel_latest_measurement_xlsx_export(
        self,
        *,
        timeout_s: float = 0.0,
    ) -> bool:
        return self._latest_xlsx.close(timeout_s=timeout_s)


__all__ = [
    "DATA_STORAGE_CONTROLLER_METHODS",
    "DEFAULT_MEASUREMENT_DATA_ROOT",
    "LEGACY_DEFAULT_MEASUREMENT_DATA_ROOT",
    "LATEST_MEASUREMENT_XLSX_STATUS_SCHEMA",
    "DataStorageController",
]
