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
        ledger = self._ensure_session_ledger()
        if ledger is None:
            ledger = RunEvidenceLedger(self._session_root())
            self._ledger = ledger
        event = ledger.append_event(
            event_type,
            dict(payload or {}),
            source=source,
            coordinate_contract=coordinate_contract,
        )
        return event

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
            self.ensure_workstation_run_evidence(
                "session_prepared",
                {},
                source="RunEvidenceController",
            )
        assert self._ledger is not None
        return self._ledger.append_event(
            event_type,
            payload,
            source=source,
            source_timestamp_ns=source_timestamp_ns,
            coordinate_contract=coordinate_contract,
        )

    def import_workstation_session_artifact(
        self,
        source_path: str | Path,
        *,
        role: str,
        media_type: str = "application/octet-stream",
        metadata: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self._ledger is None or not self._ledger.locked:
            self.ensure_workstation_run_evidence(
                "session_prepared",
                {},
                source="RunEvidenceController",
            )
        assert self._ledger is not None
        return self._ledger.import_artifact(
            source_path,
            role=role,
            media_type=media_type,
            metadata=metadata,
        ).to_mapping()

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
            self.ensure_workstation_run_evidence(
                "session_prepared",
                {},
                source="RunEvidenceController",
            )
        assert self._ledger is not None
        return self._ledger.register_session_artifact(
            session_path,
            role=role,
            media_type=media_type,
            metadata=metadata,
            expected_size_bytes=expected_size_bytes,
            expected_sha256=expected_sha256,
        ).to_mapping()


__all__ = ["RUN_EVIDENCE_CONTROLLER_METHODS", "RunEvidenceController"]
