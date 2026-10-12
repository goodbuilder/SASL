"""Allocate bounded recording previews backed by the measurement data store.

Unavailable storage preserves the established in-memory recording fallback."""
from __future__ import annotations
from Workstation.workstation_core.measurement_data_store import LiveDataRows
from Workstation.workstation_core.session_storage.spool import SpoolRows
from .contracts import (RECORDING_PREVIEW_LIMIT)
from .ports import RowCollectionsPort


class RowCollections:
    def __init__(self, *,
        boundary: RowCollectionsPort,
    ) -> None:
        self.boundary = boundary

    def _handle_recording_spool_error(self, exc: OSError) -> None:
        message = f"recording spool write failed: {exc}"
        self.boundary._log(f"[저장] {message}; Measurement continues")

    def _new_recording_spool(
        self,
        *,
        preview_limit: int = 256,
        supports_updates: bool = False,
    ) -> SpoolRows:
        return SpoolRows(
            preview_limit=preview_limit,
            supports_updates=supports_updates,
            on_write_error=self._handle_recording_spool_error,
        )

    def _new_live_rows(
        self,
        stream: str,
        *,
        preview_limit: int = RECORDING_PREVIEW_LIMIT,
        priority: bool | None = None,
    ) -> LiveDataRows | SpoolRows:
        provider = getattr(self.boundary, "recording_store", None)
        try:
            store = provider() if callable(provider) else None
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            store = None
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                logger(
                    f"[저장] {stream} writer 준비 실패; 메모리 기록으로 측정 계속: {exc}"
                )
        if store is None:
            # Lightweight legacy/test callers without the Workstation data
            # owner still receive an in-memory row sequence. Production
            # Workstation always exposes DataStorageController.
            return SpoolRows(preview_limit=preview_limit, supports_updates=True)
        return LiveDataRows(
            store,
            stream,
            preview_limit=preview_limit,
            priority=priority,
        )
