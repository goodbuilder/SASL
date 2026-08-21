"""Workstation owner for per-run identity and append-only session evidence."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Mapping

from Workstation.workstation_core.run_evidence import RunEvidenceLedger, RunMode, RunSetup


RUN_EVIDENCE_CONTROLLER_METHODS = (
    "apply_workstation_run_setup",
    "workstation_run_setup_snapshot",
    "ensure_workstation_run_evidence",
    "append_workstation_run_event",
    "import_workstation_session_artifact",
    "register_workstation_session_artifact",
)


class RunEvidenceController:
    def __init__(self, panel: object) -> None:
        self.panel = panel
        self._setup: RunSetup | None = None
        self._ledger: RunEvidenceLedger | None = None
        self._state = "PENDING"
        self._detail = "run setup not applied"

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
        """Bind to the current session and hydrate its durable identity."""

        if not str(getattr(self.panel, "recording_session_id", "") or ""):
            return None
        root = self._session_root().resolve()
        ledger = self._ledger
        if ledger is None or ledger.root != root:
            ledger = RunEvidenceLedger(root)
            self._ledger = ledger
        if ledger.setup is not None:
            self._setup = ledger.setup
            if ledger.locked:
                self._state = "LOCKED"
                self._detail = "rehydrated from locked session run evidence"
            else:
                self._state = "APPLIED"
                self._detail = "rehydrated from session run setup"
        return ledger

    @staticmethod
    def _zoom_index(value: object) -> int | None:
        try:
            result = int(value)
            return result if result >= 0 else None
        except (TypeError, ValueError):
            return None

    def _observed_zoom(self, side: str) -> int | None:
        actual = getattr(self.panel, "actual_visible_zoom", {}) or {}
        row = actual.get(side, {}) if isinstance(actual, Mapping) else {}
        return self._zoom_index(row.get("index") if isinstance(row, Mapping) else None)

    def apply_workstation_run_setup(self, values: Mapping[str, Any]) -> dict[str, Any]:
        mode = values.get("mode", values.get("target_platform", "drone"))
        condition = values.get("condition")
        condition_mapping = dict(condition) if isinstance(condition, Mapping) else {}
        setup = RunSetup(
            experiment_id=str(values.get("experiment_id") or ""),
            run_id=str(values.get("run_id") or ""),
            condition_id=str(values.get("condition_id") or ""),
            mode=RunMode(str(mode).lower()),
            condition=tuple(condition_mapping.items()),
            requested_zoom_left=self._zoom_index(
                values.get("requested_zoom_left", self._observed_zoom("left"))
            ),
            requested_zoom_right=self._zoom_index(
                values.get("requested_zoom_right", self._observed_zoom("right"))
            ),
        )
        ledger = self._ensure_session_ledger()
        if ledger is not None and ledger.setup is not None:
            if ledger.setup == setup:
                self._setup = ledger.setup
                self._state = "LOCKED" if ledger.locked else "APPLIED"
                self._detail = (
                    "existing locked run setup confirmed"
                    if ledger.locked
                    else "existing run setup confirmed"
                )
                return self.workstation_run_setup_snapshot()
            if ledger.locked:
                raise RuntimeError("run setup is locked after the first evidence event")
        self._setup = setup
        self._state = "APPLIED"
        self._detail = "identity will lock on first HR/pose/recording evidence"
        if ledger is not None:
            ledger.apply_setup(setup)
        return self.workstation_run_setup_snapshot()

    def workstation_run_setup_snapshot(self) -> dict[str, Any]:
        self._ensure_session_ledger()
        setup = self._setup
        locked = bool(self._ledger and self._ledger.locked)
        payload: dict[str, Any] = {
            "state": "LOCKED" if locked else self._state,
            "locked": locked,
            "source": self._detail,
            "schema": "workstation-run-setup-v1",
        }
        if setup is not None:
            payload.update(setup.to_mapping())
            payload["target_platform"] = setup.mode.value
        return payload

    def ensure_workstation_run_evidence(
        self,
        event_type: str = "session_prepared",
        payload: Mapping[str, Any] | None = None,
        *,
        source: str = "Workstation",
        coordinate_contract: str = "not_applicable",
    ) -> dict[str, Any]:
        ledger = self._ensure_session_ledger()
        if self._setup is None:
            # Raw evidence must never be lost merely because an operator did
            # not finish the join labels.  Use explicit, searchable fallback
            # IDs and lock them; the UI reports AUTO_DEFAULT rather than
            # pretending they are an operator-confirmed experiment identity.
            session_id = str(getattr(self.panel, "recording_session_id", "") or "runtime")
            self._setup = RunSetup(
                experiment_id="unassigned",
                run_id=session_id,
                condition_id="unspecified",
                mode=RunMode.DRONE,
                requested_zoom_left=self._observed_zoom("left"),
                requested_zoom_right=self._observed_zoom("right"),
            )
            self._state = "AUTO_DEFAULT"
            self._detail = "operator run setup missing; explicit fallback IDs locked to preserve raw evidence"
        if ledger is None:
            ledger = RunEvidenceLedger(self._session_root())
            self._ledger = ledger
        if ledger.setup is None:
            ledger.apply_setup(self._setup)
        elif ledger.setup != self._setup:
            raise RuntimeError("session run setup differs from the active setup")
        event = ledger.append_event(
            event_type,
            dict(payload or {}),
            source=source,
            coordinate_contract=coordinate_contract,
        )
        self._state = "LOCKED"
        self._detail = f"locked by {event_type} at UTC ns {event['receive_utc_ns']}"
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
