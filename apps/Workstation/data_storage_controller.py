"""GUI-facing owner for sessionless Workstation measurement storage.

Owns the append-only JSONL store, the per-Measurement record registry and
the background workbook writer. None of them ever gates Measurement
admission: failures surface only as diagnostics and workbook events.
"""

from __future__ import annotations

import os
import threading
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from Workstation.runtime_paths import WORKSTATION_MEASUREMENT_DATA_ROOT
from Workstation.workstation_core.measurement_data_store import (
    MeasurementDataStore,
    allocate_timestamped_path,
)
from Workstation.workstation_core.measurement_record import MeasurementRegistry
from Workstation.workstation_core.measurement_artifacts import measurement_path_sort_key
from Workstation.workstation_core.measurement_workbook import MeasurementWorkbookWriter


DEFAULT_MEASUREMENT_DATA_ROOT = ""
LEGACY_DEFAULT_MEASUREMENT_DATA_ROOT = "measurement_data"
MEASUREMENT_WORKBOOK_STATUS_SCHEMA = "workstation-measurement-workbook-status-v1"
MEASUREMENT_WORKBOOK_SUFFIX = "_measurement.xlsx"

DATA_STORAGE_CONTROLLER_METHODS = (
    "append_workstation_data",
    "data_storage_snapshot",
    "retry_data_storage",
    "allocate_workstation_data_artifact",
    "allocate_measurement_artifact",
    "measurement_storage_context",
    "rebuild_latest_measurement_workbook",
    "flush_workstation_data",
    "close_workstation_data",
    "archive_session_recordings",
    "measurement_registry",
    "measurement_results_snapshot",
    "measurement_workbook_snapshot",
    "latest_measurement_workbook_path",
    "finalize_measurement_workbooks",
)


class DataStorageController:
    """Keep storage failures observable and outside Measurement admission."""

    def __init__(self, panel: object) -> None:
        self.panel = panel
        self._store: MeasurementDataStore | None = None
        self._root: Path | None = None
        self._configuration_error = ""
        self._workbook_writer: MeasurementWorkbookWriter | None = None
        self._registry = MeasurementRegistry(
            data_root_provider=self._configured_root,
            per_run_storage=True,
            context_started=self._start_storage_context,
            context_stopped=lambda measurement_id: self.store.finish_measurement(measurement_id),
            journal_sink=lambda payload: self.append_workstation_data("measurement.journal", payload, priority=True),
        )
        self._rebuild_thread: threading.Thread | None = None
        self._rebuild_error = ""
        self._repair_thread: threading.Thread | None = None
        self._repair_errors: dict[str, str] = {}

    def _start_storage_context(self, context) -> None:
        self._rebuild_error = ""
        self.store.register_measurement(context)

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
            self._store = self._new_store(root)
        elif self._root != root:
            active = bool(
                getattr(self.panel, "distance_recording_enabled", False)
                or getattr(self.panel, "video_recording_enabled", False)
            )
            if not active:
                # This property is read from the GUI thread's status-poll
                # timer (see StatusModel._refresh_recording ->
                # data_storage_snapshot -> store). Closing the old store
                # synchronously can block for up to close()'s own timeout,
                # which would freeze the UI whenever
                # measurement_data_root changes while idle. Swap in the new
                # store immediately and let the superseded store finish
                # closing on a background thread instead.
                old_store = self._store
                self._root = root
                self._store = self._new_store(root)
                self._close_store_in_background(old_store)
        return self._store

    def _new_store(self, root: Path) -> MeasurementDataStore:
        store = MeasurementDataStore(root, timestamped_filenames=True)
        store.add_enqueue_observer(self._registry.observe_store_record)
        return store

    @staticmethod
    def _close_store_in_background(store: MeasurementDataStore) -> None:
        def _do_close() -> None:
            try:
                store.close(timeout_s=2.0)
            except Exception:
                pass

        threading.Thread(
            target=_do_close,
            name="workstation-data-store-close",
            daemon=True,
        ).start()

    def measurement_registry(self) -> MeasurementRegistry:
        if self._workbook_writer is None:
            self._workbook_writer = MeasurementWorkbookWriter(source_storage_reader=self._source_storage_for_workbook)
            self._registry.set_writer(self._workbook_writer)
        return self._registry

    def measurement_results_snapshot(self, measurement_id=None, pair_id=None):
        """Read existing records without starting the workbook writer or storage."""
        return self._registry.results_snapshot(measurement_id, pair_id)

    def _source_storage_for_workbook(self, measurement_id: str) -> dict[str, Any]:
        checkpoint = self.store.export_checkpoint(timeout_s=1.0, measurement_id=measurement_id)
        snapshot = checkpoint.storage_snapshot
        streams = {
            value["stream"]: value for value in snapshot.get("stream_instances", {}).values()
            if value.get("measurement_id") == measurement_id
        }
        degraded = any(v.get("state") == "FAILED" or v.get("dropped_records") or (v.get("csv") or {}).get("state") == "FAILED" or (v.get("csv") or {}).get("dropped_records") for v in streams.values())
        state = "DEGRADED" if degraded else "OK"
        return {"state": state if checkpoint.settled else "PENDING", "streams": streams,
                "pending_records": checkpoint.pending_before_cutoff, "settled": checkpoint.settled}

    def append_workstation_data(
        self,
        stream: str,
        payload: Mapping[str, Any],
        *,
        priority: bool | None = None,
    ) -> bool:
        try:
            return bool(self.store.enqueue(stream, dict(payload), priority=priority))
        except (OSError, TypeError, ValueError):
            # Storage is explicitly best-effort. The per-stream snapshot owns
            # detailed failures once a store exists; setup errors cannot become
            # a Measurement command rejection.
            return False

    def data_storage_snapshot(self) -> dict[str, Any]:
        try:
            snapshot = self.store.snapshot()
            from Workstation.workstation_core.measurement_csv_mirror import failed_mirrors
            external = {str(path): error for path, error in failed_mirrors().items() if path.is_relative_to(self._configured_root())}
            external.update(self._repair_errors)
            if external:
                snapshot["state"] = "DEGRADED"
                for name, error in external.items():
                    path = Path(name)
                    snapshot["streams"][f"rtcm/{path.stem}"] = {
                        "path": str(path), "state": "ACTIVE", "dropped_records": 0,
                        "last_error": "", "csv": {"path": str(path.with_suffix('.csv')), "state": "FAILED", "last_error": error, "dropped_records": 0},
                    }
            snapshot["csv_repairing"] = self._repair_thread is not None and self._repair_thread.is_alive()
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
        self._retry_external_mirrors()
        return self.data_storage_snapshot()

    def _retry_external_mirrors(self) -> None:
        if self._registry.current() is not None or bool(getattr(self.panel, "_manual_recording_active", False)):
            return  # the active RTCM owner retries its own mirror on the next row
        if self._repair_thread is not None and self._repair_thread.is_alive():
            return
        root = self._configured_root()
        def repair() -> None:
            from Workstation.workstation_core.measurement_csv_mirror import failed_mirrors, repair_owned_jsonl_mirror
            journals = sorted(root.glob("*/raw/*_measurement_journal.*"), key=measurement_path_sort_key)
            paths = set(journals[-1].parent.parent.glob("raw/gnss/*.rtcm.jsonl")) if journals else set()
            paths.update(path for path in failed_mirrors() if path.is_relative_to(root))
            errors = {}
            for path in paths:
                try:
                    repair_owned_jsonl_mirror(path)
                except Exception as exc:
                    errors[str(path)] = str(exc)
            self._repair_errors = errors
        self._repair_thread = threading.Thread(target=repair, name="measurement-csv-repair", daemon=True)
        self._repair_thread.start()

    def allocate_workstation_data_artifact(
        self,
        category: str,
        *,
        label: str,
        suffix: str,
        utc_ns: int | None = None,
        measurement_id: str | None = None,
    ) -> Path:
        context = self._artifact_context(measurement_id)
        if context is not None:
            return context.allocate(category, label, suffix)
        return allocate_timestamped_path(
            self._configured_root(),
            category,
            label=label,
            suffix=suffix,
            utc_ns=utc_ns,
        )

    def _artifact_context(self, measurement_id: str | None = None):
        registry = getattr(self, "_registry", None)
        if registry is None:
            return None
        if measurement_id is None:
            record = registry.current()
            if record is None or not record.running:
                return None
            measurement_id = record.measurement_id
        return registry.context_for(measurement_id)

    def measurement_storage_context(self, measurement_id: str | None = None) -> dict[str, Any]:
        context = self._artifact_context(measurement_id)
        return {} if context is None else {**context.metadata(), "root": str(context.root)}

    def allocate_measurement_artifact(self, *, label: str, suffix: str, measurement_id: str | None = None) -> Path | None:
        """``<root>/<measurement prefix>_<label>[-NN]<suffix>`` for the running Measurement."""

        context = self._artifact_context(measurement_id)
        if context is not None:
            return context.allocate("media", label, suffix)
        if measurement_id is not None:
            return None
        record = self._registry.current()
        if record is None or not record.running or not record.file_prefix:
            return None
        suffix_text = str(suffix or "").strip().lower()
        if not suffix_text.startswith(".") or "/" in suffix_text:
            raise ValueError("artifact suffix must be a short extension")
        safe_label = "".join(
            character if character.isalnum() or character in "-_" else "-"
            for character in str(label or "artifact")
        ) or "artifact"
        root = self._configured_root()
        root.mkdir(parents=True, exist_ok=True)
        stem = f"{record.file_prefix}_{safe_label}"
        sequence = 1
        candidate = root / f"{stem}{suffix_text}"
        while True:
            # Reserve the slot atomically (O_CREAT|O_EXCL) instead of
            # checking exists()/is_symlink() and returning the path for the
            # caller to open later -- the check-then-return form lets two
            # concurrent callers observe the same "free" candidate and both
            # end up writing to it. The reservation leaves an empty file in
            # place, which the eventual writer (video/ffmpeg/etc.) opens and
            # truncates as normal.
            try:
                fd = os.open(
                    str(candidate),
                    os.O_CREAT | os.O_EXCL | os.O_WRONLY,
                    0o644,
                )
            except FileExistsError:
                sequence += 1
                candidate = root / f"{stem}-{sequence:02d}{suffix_text}"
                continue
            os.close(fd)
            return candidate

    def latest_measurement_workbook_path(self) -> Path | None:
        root = self._configured_root()
        try:
            candidates = sorted(
                (path for pattern in (f"*{MEASUREMENT_WORKBOOK_SUFFIX}", f"*/*{MEASUREMENT_WORKBOOK_SUFFIX}") for path in root.glob(pattern) if path.is_file()),
                key=measurement_path_sort_key,
            )
        except OSError:
            return None
        return candidates[-1] if candidates else None

    def rebuild_latest_measurement_workbook(self) -> bool:
        """Queue a rebuild from saved input only; never block the Qt thread."""
        if self._registry.current() is not None:
            return False
        if self._registry.pending_job_count() or any(r.pending_method_count() for r in self._registry.records()):
            return False
        if bool(getattr(self.panel, "_manual_recording_active", False)):
            return False
        if self._workbook_writer is not None and self._workbook_writer.snapshot().get("state") == "WRITING":
            return False
        if self._rebuild_thread is not None and self._rebuild_thread.is_alive():
            return False
        root = self._configured_root()
        journals = sorted({p.with_suffix(".jsonl") for pattern in ("*/raw/*_measurement_journal.jsonl", "*/raw/*_measurement_journal.csv") for p in root.glob(pattern)}, key=measurement_path_sort_key)
        if not journals:
            self._rebuild_error = "재생성할 Measurement 원본 기록이 없습니다."
            return False
        source = journals[-1]
        self._rebuild_error = ""

        def rebuild() -> None:
            from Workstation.workstation_core.measurement_journal import load_measurement_record
            from Workstation.workstation_core.measurement_workbook import write_measurement_workbook
            try:
                if self._store is not None and self._store.alive:
                    self._store.export_checkpoint(timeout_s=5.0)
                record = load_measurement_record(source)
                write_measurement_workbook(record.snapshot(), record.workbook_path)
            except Exception as exc:
                self._rebuild_error = f"{type(exc).__name__}: {exc}"

        self._rebuild_thread = threading.Thread(target=rebuild, name="measurement-workbook-rebuild", daemon=True)
        self._rebuild_thread.start()
        return True

    def measurement_workbook_snapshot(self) -> dict[str, Any]:
        writer = self._workbook_writer
        registry_status = self._registry.status_snapshot()
        latest = self.latest_measurement_workbook_path()
        writer_status = {} if writer is None else writer.snapshot()
        state = str(writer_status.get("state") or "IDLE")
        if state == "IDLE" and latest is not None:
            state = "READY"
        pending_jobs = sum(int(row.get("pending_method_jobs") or 0) for row in registry_status["records"])
        return {
            "schema": MEASUREMENT_WORKBOOK_STATUS_SCHEMA,
            "state": state,
            "current_measurement_id": registry_status["current_measurement_id"],
            "records": registry_status["records"],
            "pending_method_jobs": pending_jobs,
            "latest_path": "" if latest is None else str(latest),
            "last_path": str(writer_status.get("last_path") or ""),
            "last_write_utc_ns": writer_status.get("last_write_utc_ns"),
            "error": str(writer_status.get("last_error") or ""),
            "write_count": int(writer_status.get("write_count") or 0),
            "failure_count": int(writer_status.get("failure_count") or 0),
            "writer_alive": bool(writer_status.get("writer_alive", False)),
            "recording_active": bool(registry_status["current_measurement_id"]),
            "rebuilding": self._rebuild_thread is not None and self._rebuild_thread.is_alive(),
            "rebuild_error": self._rebuild_error,
        }

    def flush_workstation_data(self) -> bool:
        store = self._store
        if store is None:
            return True
        store.request_flush()
        return True

    def finalize_measurement_workbooks(self, *, timeout_s: float = 5.0) -> bool:
        """Shutdown path: fail pending jobs, write every record, stop the writer."""

        deadline = time.monotonic() + max(0.0, float(timeout_s))
        writer = self._workbook_writer
        try:
            self._registry.finalize_all(
                "application shutdown before method completion",
                timeout_s=max(0.0, deadline - time.monotonic()),
            )
        except Exception:  # noqa: BLE001 - shutdown must proceed.
            pass
        if writer is None:
            return True
        closed = writer.close(timeout_s=max(0.0, deadline - time.monotonic()))
        self._workbook_writer = None
        self._registry.set_writer(None)
        return bool(closed)

    def archive_session_recordings(self) -> bool:
        """Move this session's files under ``<root>/<YYYYMMDD_HHMMSS>/`` after close.

        Every writer of one process shares the store's creation token, so the
        session is identifiable by filename. Once the store is closed the
        token-stamped stream files, the ``frames``/``raw`` trees they reference
        by relative path, and the run artifacts (``<prefix>_measurement.xlsx``,
        ``<prefix>_left/right.mp4``) started during the session are renamed
        into one folder named by the token time. Idempotent: an already
        archived session (or nothing to move) returns ``True``; an open store
        returns ``False`` so the lifecycle retries after ``close``.
        """

        store = self._store
        if store is None or not bool(getattr(store, "timestamped_filenames", False)):
            return True
        stamp = str(getattr(store, "file_stamp", "") or "")
        if len(stamp) < 15:
            return True
        try:
            lifecycle = str((store.snapshot() or {}).get("lifecycle_state") or "").upper()
        except (OSError, RuntimeError, TypeError, ValueError):
            lifecycle = ""
        if lifecycle == "OPEN":
            return False
        root = Path(getattr(store, "root", self._root or self._configured_root()))
        folder_name = stamp[:15]
        target = root / folder_name
        moved: list[str] = []
        try:
            entries = list(root.iterdir())
        except OSError:
            return True

        def _move(source: Path, destination: Path) -> None:
            if destination.exists():
                return
            destination.parent.mkdir(parents=True, exist_ok=True)
            source.rename(destination)
            moved.append(source.name)

        try:
            for entry in entries:
                name = entry.name
                if entry.is_dir():
                    if name in {"frames", "raw"}:
                        _move(entry, target / name)
                    continue
                if not entry.is_file():
                    continue
                if f"_{stamp}." in name or name.endswith(f"_{stamp}"):
                    _move(entry, target / name)
                    continue
                prefix = name[:15]
                if (
                    len(name) > 16
                    and prefix[:8].isdigit()
                    and prefix[8] == "_"
                    and prefix[9:15].isdigit()
                    and prefix >= folder_name
                    and name[15] in "_-"
                ):
                    run_dir = target / "runs" / prefix
                    _move(entry, run_dir / name)
        except OSError as exc:
            logger = getattr(self.panel, "_log", None)
            if callable(logger):
                logger(f"[저장] 세션 폴더 정리 실패 ({folder_name}): {exc}")
            return False
        if moved:
            logger = getattr(self.panel, "_log", None)
            if callable(logger):
                logger(f"[저장] 세션 파일 {len(moved)}개를 {target} 아래로 정리했습니다")
        return True

    def close_workstation_data(self, *, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        repair = getattr(self, "_repair_thread", None)
        if repair is not None and repair.is_alive():
            repair.join(timeout=max(0.0, deadline - time.monotonic()))
            if repair.is_alive():
                return False
        rebuild = getattr(self, "_rebuild_thread", None)
        if rebuild is not None and rebuild.is_alive():
            rebuild.join(timeout=max(0.0, deadline - time.monotonic()))
            if rebuild.is_alive():
                return False
        workbooks_closed = self.finalize_measurement_workbooks(
            timeout_s=max(0.0, deadline - time.monotonic())
        )
        store = self._store
        if store is None:
            return workbooks_closed
        return bool(store.close(timeout_s=max(0.0, deadline - time.monotonic()))) and workbooks_closed


__all__ = [
    "DATA_STORAGE_CONTROLLER_METHODS",
    "DEFAULT_MEASUREMENT_DATA_ROOT",
    "LEGACY_DEFAULT_MEASUREMENT_DATA_ROOT",
    "MEASUREMENT_WORKBOOK_STATUS_SCHEMA",
    "DataStorageController",
]
