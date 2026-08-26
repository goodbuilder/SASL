"""GUI-facing owner for sessionless Workstation measurement storage."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from Workstation.workstation_core.measurement_data_store import (
    MeasurementDataStore,
    allocate_timestamped_path,
)


DEFAULT_MEASUREMENT_DATA_ROOT = "measurement_data"

DATA_STORAGE_CONTROLLER_METHODS = (
    "append_workstation_data",
    "data_storage_snapshot",
    "retry_data_storage",
    "allocate_workstation_data_artifact",
    "flush_workstation_data",
    "close_workstation_data",
)


class DataStorageController:
    """Keep storage failures observable and outside Measurement admission."""

    def __init__(self, panel: object) -> None:
        self.panel = panel
        self._store: MeasurementDataStore | None = None
        self._root: Path | None = None
        self._configuration_error = ""

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
        base = Path(getattr(self.panel, "app_root", Path.cwd())).expanduser().resolve()
        path = Path(text or DEFAULT_MEASUREMENT_DATA_ROOT).expanduser()
        if path.is_absolute():
            self._configuration_error = (
                "measurement_data_root must be a portable relative path; "
                f"using {DEFAULT_MEASUREMENT_DATA_ROOT!r} instead of {text!r}"
            )
            return (base / DEFAULT_MEASUREMENT_DATA_ROOT).resolve(strict=False)
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
                f"using {DEFAULT_MEASUREMENT_DATA_ROOT!r} instead of {text!r}"
            )
            return (base / DEFAULT_MEASUREMENT_DATA_ROOT).resolve(strict=False)
        self._configuration_error = ""
        return resolved

    @property
    def store(self) -> MeasurementDataStore:
        root = self._configured_root().resolve(strict=False)
        if self._store is None:
            self._root = root
            self._store = MeasurementDataStore(root)
        elif self._root != root:
            active = bool(
                getattr(self.panel, "distance_recording_enabled", False)
                or getattr(self.panel, "video_recording_enabled", False)
            )
            if not active:
                self._store.close(timeout_s=2.0)
                self._root = root
                self._store = MeasurementDataStore(root)
        return self._store

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
        store = self._store
        if store is None:
            return True
        return bool(store.close(timeout_s=timeout_s))


__all__ = [
    "DATA_STORAGE_CONTROLLER_METHODS",
    "DEFAULT_MEASUREMENT_DATA_ROOT",
    "DataStorageController",
]
