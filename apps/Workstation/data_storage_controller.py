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
    "flush_workstation_data",
    "close_workstation_data",
    "measurement_registry",
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
        self._registry = MeasurementRegistry(data_root_provider=self._configured_root)

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
            self._workbook_writer = MeasurementWorkbookWriter()
            self._registry.set_writer(self._workbook_writer)
        return self._registry

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

    def allocate_measurement_artifact(self, *, label: str, suffix: str) -> Path | None:
        """``<root>/<measurement prefix>_<label>[-NN]<suffix>`` for the running Measurement."""

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
                (path for path in root.glob(f"*{MEASUREMENT_WORKBOOK_SUFFIX}") if path.is_file()),
                key=lambda path: path.name,
            )
        except OSError:
            return None
        return candidates[-1] if candidates else None

    def measurement_workbook_snapshot(self) -> dict[str, Any]:
        writer = self._workbook_writer
        registry_status = self._registry.status_snapshot()
        latest = self.latest_measurement_workbook_path()
        writer_status = {} if writer is None else writer.snapshot()
        state = str(writer_status.get("state") or "IDLE")
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

    def close_workstation_data(self, *, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
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
