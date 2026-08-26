"""Workstation owner for append-only session evidence."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Mapping

from Workstation.workstation_core.run_evidence import RunEvidenceLedger


PERSISTENCE_FAILURE_SCHEMA = "workstation-persistence-failure-v1"


def persistence_failure_error(result: object) -> str | None:
    """Return the persisted error text for the controller's failure contract."""

    if not isinstance(result, Mapping):
        return None
    if (
        result.get("schema") != PERSISTENCE_FAILURE_SCHEMA
        or result.get("status") != "failed"
    ):
        return None
    return str(result.get("error") or "persistence operation failed")


def report_recoverable_persistence_failure(panel: object, message: str) -> None:
    """Publish one recoverable storage failure through the shared panel state."""

    text = str(message)
    if not bool(getattr(panel, "_measurement_session_hierarchy_enabled", True)):
        label = getattr(panel, "recording_status_label", None)
        setter = getattr(label, "setText", None)
        if callable(setter):
            setter(f"데이터 저장 저하(측정 계속): {text}")
        logger = getattr(panel, "_log", None)
        if callable(logger):
            logger(f"[저장] {text}; Measurement continues")
        return
    setattr(panel, "_session_save_retry_required", True)
    setattr(panel, "_session_save_last_error", text)
    label = getattr(panel, "recording_status_label", None)
    setter = getattr(label, "setText", None)
    if callable(setter):
        setter(f"세션 기록 실패(복구 가능): {text}")
    logger = getattr(panel, "_log", None)
    if callable(logger):
        logger(f"[저장] {text}")


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
                # Do not abandon the old dirty ledger when its durability
                # barrier fails.  The public wrapper records the failure and
                # leaves this binding intact so the same session can retry.
                ledger.flush()
            ledger = RunEvidenceLedger(root)
            self._ledger = ledger
        return ledger

    def _get_or_create_session_ledger(self) -> RunEvidenceLedger:
        ledger = self._ensure_session_ledger()
        if ledger is None:
            ledger = RunEvidenceLedger(self._session_root())
            self._ledger = ledger
        return ledger

    def _run_persistence(
        self,
        kind: str,
        operation: Callable[[], Any],
    ) -> Any:
        try:
            return operation()
        except OSError as exc:
            self._record_persistence_failure(exc)
            return self._failed_write_result(kind, exc)

    def _locked_ledger_or_failure(self) -> RunEvidenceLedger | dict[str, Any]:
        ledger = self._run_persistence(
            "run_event_setup",
            self._get_or_create_session_ledger,
        )
        if self.is_persistence_failure(ledger):
            return dict(ledger)
        assert isinstance(ledger, RunEvidenceLedger)
        if ledger.locked:
            return ledger
        setup = self._run_persistence(
            "run_event",
            lambda: ledger.append_event(
                "session_prepared",
                {},
                source="RunEvidenceController",
            ),
        )
        if self.is_persistence_failure(setup):
            return dict(setup)
        return ledger

    def ensure_workstation_run_evidence(
        self,
        event_type: str = "session_prepared",
        payload: Mapping[str, Any] | None = None,
        *,
        source: str = "Workstation",
        coordinate_contract: str = "not_applicable",
    ) -> dict[str, Any]:
        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            event = {
                "event_type": str(event_type),
                "source": str(source),
                "coordinate_contract": str(coordinate_contract),
                "payload": dict(payload or {}),
            }
            writer = getattr(self.panel, "append_workstation_data", None)
            persisted = bool(writer("events", event, priority=True)) if callable(writer) else False
            return {**event, "persisted": persisted}
        ledger = self._run_persistence(
            "run_event_setup",
            self._get_or_create_session_ledger,
        )
        if self.is_persistence_failure(ledger):
            return dict(ledger)
        assert isinstance(ledger, RunEvidenceLedger)
        return self._run_persistence(
            "run_event",
            lambda: ledger.append_event(
                event_type,
                dict(payload or {}),
                source=source,
                coordinate_contract=coordinate_contract,
            ),
        )

    def append_workstation_run_event(
        self,
        event_type: str,
        payload: Mapping[str, Any],
        *,
        source: str,
        coordinate_contract: str,
        source_timestamp_ns: int | None = None,
    ) -> dict[str, Any]:
        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            event = {
                "event_type": str(event_type),
                "source": str(source),
                "source_timestamp_ns": source_timestamp_ns,
                "coordinate_contract": str(coordinate_contract),
                "payload": dict(payload),
            }
            writer = getattr(self.panel, "append_workstation_data", None)
            persisted = bool(writer("events", event, priority=True)) if callable(writer) else False
            return {**event, "persisted": persisted}
        ledger = self._locked_ledger_or_failure()
        if self.is_persistence_failure(ledger):
            return dict(ledger)
        assert isinstance(ledger, RunEvidenceLedger)
        return self._run_persistence(
            "run_event",
            lambda: ledger.append_event(
                event_type,
                payload,
                source=source,
                source_timestamp_ns=source_timestamp_ns,
                coordinate_contract=coordinate_contract,
            ),
        )

    def import_workstation_session_artifact(
        self,
        source_path: str | Path,
        *,
        role: str,
        media_type: str = "application/octet-stream",
        metadata: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            source = Path(source_path).expanduser().resolve(strict=True)
            record = {
                "event_type": "artifact_import_reference",
                "role": str(role),
                "media_type": str(media_type),
                "source_path": str(source),
                "size_bytes": source.stat().st_size,
                "metadata": dict(metadata or {}),
            }
            writer = getattr(self.panel, "append_workstation_data", None)
            persisted = bool(writer("artifacts", record, priority=True)) if callable(writer) else False
            return {**record, "persisted": persisted}
        ledger = self._locked_ledger_or_failure()
        if self.is_persistence_failure(ledger):
            return dict(ledger)
        assert isinstance(ledger, RunEvidenceLedger)
        return self._run_persistence(
            "artifact_import",
            lambda: ledger.import_artifact(
                source_path,
                role=role,
                media_type=media_type,
                metadata=metadata,
            ).to_mapping(),
        )

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

        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            path = Path(session_path).expanduser().resolve(strict=True)
            record = {
                "event_type": "artifact_registered",
                "role": str(role),
                "media_type": str(media_type),
                "path": str(path),
                "size_bytes": path.stat().st_size,
                "expected_size_bytes": expected_size_bytes,
                "expected_sha256": expected_sha256,
                "metadata": dict(metadata or {}),
            }
            writer = getattr(self.panel, "append_workstation_data", None)
            persisted = bool(writer("artifacts", record, priority=True)) if callable(writer) else False
            return {**record, "persisted": persisted}
        ledger = self._locked_ledger_or_failure()
        if self.is_persistence_failure(ledger):
            return dict(ledger)
        assert isinstance(ledger, RunEvidenceLedger)
        return self._run_persistence(
            "artifact_registration",
            lambda: ledger.register_session_artifact(
                session_path,
                role=role,
                media_type=media_type,
                metadata=metadata,
                expected_size_bytes=expected_size_bytes,
                expected_sha256=expected_sha256,
            ).to_mapping(),
        )

    def flush_workstation_run_evidence(self) -> bool:
        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            flusher = getattr(self.panel, "flush_workstation_data", None)
            return True if not callable(flusher) else bool(flusher())
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
        report_recoverable_persistence_failure(self.panel, message)

    @staticmethod
    def _failed_write_result(kind: str, exc: OSError) -> dict[str, Any]:
        return {
            "schema": PERSISTENCE_FAILURE_SCHEMA,
            "status": "failed",
            "kind": str(kind),
            "error": str(exc),
        }

    @staticmethod
    def is_persistence_failure(result: object) -> bool:
        """True when a helper above swallowed an OSError and returned a failure dict."""

        return persistence_failure_error(result) is not None


__all__ = [
    "PERSISTENCE_FAILURE_SCHEMA",
    "RUN_EVIDENCE_CONTROLLER_METHODS",
    "RunEvidenceController",
    "persistence_failure_error",
    "report_recoverable_persistence_failure",
]
