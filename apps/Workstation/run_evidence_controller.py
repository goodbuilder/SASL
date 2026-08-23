"""Workstation owner for append-only session evidence."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from Workstation.workstation_core.run_evidence import RunEvidenceLedger


RUN_EVIDENCE_CONTROLLER_METHODS = (
    "ensure_workstation_run_evidence",
    "append_workstation_run_event",
    "import_workstation_session_artifact",
    "register_workstation_session_artifact",
    "flush_workstation_run_evidence",
)


class RunEvidenceController:
    def __init__(self, panel: object) -> None:
        self.panel = panel
        self._ledger: RunEvidenceLedger | None = None

    @property
    def ledger(self) -> RunEvidenceLedger | None:
        return self._ledger

    def _session_root(self) -> Path:
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        if not session_id:
            raise RuntimeError("recording session ID is not prepared")
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        return Path(resolver(session_id))

    def _ensure_session_ledger(self) -> RunEvidenceLedger | None:
        """Bind the evidence ledger to the current session."""

        if not str(getattr(self.panel, "recording_session_id", "") or ""):
            return None
        root = self._session_root().resolve()
        ledger = self._ledger
        if ledger is None or ledger.root != root:
            if ledger is not None:
                try:
                    ledger.flush()
                except OSError as exc:
                    self._record_persistence_failure(exc)
            ledger = RunEvidenceLedger(root)
            self._ledger = ledger
        return ledger

    def ensure_workstation_run_evidence(
        self,
        event_type: str = "session_prepared",
        payload: Mapping[str, Any] | None = None,
        *,
        source: str = "Workstation",
        coordinate_contract: str = "not_applicable",
    ) -> dict[str, Any]:
        try:
            ledger = self._ensure_session_ledger()
            if ledger is None:
                ledger = RunEvidenceLedger(self._session_root())
                self._ledger = ledger
        except OSError as exc:
            self._record_persistence_failure(exc)
            return self._failed_write_result("run_event_setup", exc)
        try:
            return ledger.append_event(
                event_type,
                dict(payload or {}),
                source=source,
                coordinate_contract=coordinate_contract,
            )
        except OSError as exc:
            self._record_persistence_failure(exc)
            return self._failed_write_result("run_event", exc)

    def append_workstation_run_event(
        self,
        event_type: str,
        payload: Mapping[str, Any],
        *,
        source: str,
        coordinate_contract: str,
        source_timestamp_ns: int | None = None,
    ) -> dict[str, Any]:
        if self._ledger is None or not self._ledger.locked:
            setup = self.ensure_workstation_run_evidence(
                "session_prepared",
                {},
                source="RunEvidenceController",
            )
            if self._ledger is None:
                return dict(setup)
        try:
            return self._ledger.append_event(
                event_type,
                payload,
                source=source,
                source_timestamp_ns=source_timestamp_ns,
                coordinate_contract=coordinate_contract,
            )
        except OSError as exc:
            self._record_persistence_failure(exc)
            return self._failed_write_result("run_event", exc)

    def import_workstation_session_artifact(
        self,
        source_path: str | Path,
        *,
        role: str,
        media_type: str = "application/octet-stream",
        metadata: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self._ledger is None or not self._ledger.locked:
            setup = self.ensure_workstation_run_evidence(
                "session_prepared",
                {},
                source="RunEvidenceController",
            )
            if self._ledger is None:
                return dict(setup)
        try:
            return self._ledger.import_artifact(
                source_path,
                role=role,
                media_type=media_type,
                metadata=metadata,
            ).to_mapping()
        except OSError as exc:
            self._record_persistence_failure(exc)
            return self._failed_write_result("artifact_import", exc)

    def register_workstation_session_artifact(
        self,
        session_path: str | Path,
        *,
        role: str,
        media_type: str = "application/octet-stream",
        metadata: Mapping[str, Any] | None = None,
        expected_size_bytes: int | None = None,
        expected_sha256: str | None = None,
    ) -> dict[str, Any]:
        """Inventory an already sealed file below the active session root."""

        if self._ledger is None or not self._ledger.locked:
            setup = self.ensure_workstation_run_evidence(
                "session_prepared",
                {},
                source="RunEvidenceController",
            )
            if self._ledger is None:
                return dict(setup)
        try:
            return self._ledger.register_session_artifact(
                session_path,
                role=role,
                media_type=media_type,
                metadata=metadata,
                expected_size_bytes=expected_size_bytes,
                expected_sha256=expected_sha256,
            ).to_mapping()
        except OSError as exc:
            self._record_persistence_failure(exc)
            return self._failed_write_result("artifact_registration", exc)

    def flush_workstation_run_evidence(self) -> bool:
        ledger = self._ledger
        if ledger is None:
            return True
        try:
            ledger.flush()
        except OSError as exc:
            self._record_persistence_failure(exc)
            return False
        return True

    def _record_persistence_failure(self, exc: OSError) -> None:
        message = f"run evidence persistence failed: {exc}"
        setattr(self.panel, "_session_save_retry_required", True)
        setattr(self.panel, "_session_save_last_error", message)
        label = getattr(self.panel, "recording_status_label", None)
        setter = getattr(label, "setText", None)
        if callable(setter):
            setter(f"세션 기록 실패(복구 가능): {message}")
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            logger(f"[저장] {message}")

    @staticmethod
    def _failed_write_result(kind: str, exc: OSError) -> dict[str, Any]:
        return {
            "schema": "workstation-persistence-failure-v1",
            "status": "failed",
            "kind": str(kind),
            "error": str(exc),
        }


__all__ = ["RUN_EVIDENCE_CONTROLLER_METHODS", "RunEvidenceController"]
