"""Own the session ULog importer/finalizer and register verified artifacts.

ULog data remains opaque PX4 evidence, with no optical-frame conversion."""
from __future__ import annotations
from collections.abc import Mapping
from pathlib import Path
from Workstation.controllers.run_evidence_controller import persistence_failure_error
from Workstation.controllers.ulog_controller import ManualOwnershipClaim, ULogController
from Workstation.workstation_core.mavlink_sources import MavlinkSourceId
from .state import (RecordingSessionState, ULogState)
from .ports import UlogPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .photo_capture import PhotoCapture


class Ulog:
    def __init__(self, *,
        session: RecordingSessionState,
        ulog_state: ULogState,
        boundary: UlogPort,
    ) -> None:
        self.session = session
        self.ulog_state = ulog_state
        self.boundary = boundary
        self.controller_type = lambda: ULogController

    def bind_collaborators(self, *,
        photo_capture: "PhotoCapture",
    ) -> None:
        self.photo_capture = photo_capture

    def _finalize_workstation_ulogs(self) -> None:
        """Stop/download only ULogs whose injected adapter proves ownership."""

        controller = getattr(self.ulog_state, "_workstation_ulog_controller", None)
        if not isinstance(controller, self.controller_type()):
            return
        snapshot = controller.ulog_snapshot()
        for source in MavlinkSourceId:
            row = snapshot.get("sources", {}).get(source.value, {})
            if not isinstance(row, dict) or row.get("owner") != "WORKSTATION":
                continue
            controller.stop_automatic_ulog_and_collect(source)
        refreshed = controller.ulog_snapshot()
        for artifact in refreshed.get("artifacts", ()) or ():
            if not isinstance(artifact, dict):
                continue
            relative = str(artifact.get("relative_path") or "")
            if not relative:
                continue
            registration = self._register_session_owned_artifact(
                controller.session_root / relative,
                role=f"ulog/{artifact.get('source_id', 'unknown')}",
                media_type="application/vnd.px4.ulog",
                metadata=dict(artifact),
                expected_size_bytes=artifact.get("size_bytes"),
                expected_sha256=artifact.get("sha256"),
            )
            error = persistence_failure_error(registration)
            if error is not None:
                raise OSError(f"ULog artifact registration failed: {error}")

    def _register_session_owned_artifact(
        self,
        path: Path | str,
        *,
        role: str,
        media_type: str,
        metadata: Mapping[str, object],
        expected_size_bytes: object = None,
        expected_sha256: object = None,
    ) -> object | None:
        """Register session-owned sealed bytes, with a legacy copy fallback."""

        registrar = getattr(self.boundary, "register_workstation_session_artifact", None)
        if callable(registrar):
            return registrar(
                path,
                role=role,
                media_type=media_type,
                metadata=dict(metadata),
                expected_size_bytes=expected_size_bytes,
                expected_sha256=expected_sha256,
            )
        importer = getattr(self.boundary, "import_workstation_session_artifact", None)
        if callable(importer):
            return importer(
                path,
                role=role,
                media_type=media_type,
                metadata=dict(metadata),
            )
        return None

    def _ensure_ulog_controller(self) -> ULogController:
        sessionless = not bool(
            getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
        )
        session_id = (
            "measurement-data"
            if sessionless
            else str(getattr(self.session, "recording_session_id", "") or "")
        )
        if not session_id:
            raise RuntimeError("recording session ID is not prepared")
        root_resolver = getattr(self.boundary, "_recording_session_dir", None)
        if not callable(root_resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        root = Path(root_resolver(session_id)).resolve()
        current = getattr(self.ulog_state, "_workstation_ulog_controller", None)
        if (
            not isinstance(current, self.controller_type())
            or current.session_id != session_id
            or current.session_root != root
        ):
            automatic_sources = getattr(self.boundary, "ulog_automatic_sources", None)
            current = self.controller_type()(
                root,
                session_id,
                timestamp_filenames=sessionless,
                storage_context_provider=getattr(self.boundary, "measurement_storage_context", None),
                artifact_sink=lambda payload: bool(self.boundary.append_workstation_data("artifacts", payload, priority=True)),
                automatic_sources=(
                    automatic_sources if isinstance(automatic_sources, dict) else None
                ),
            )
            setattr(self.ulog_state, "_workstation_ulog_controller", current)
        return current

    def workstation_ulog_snapshot(self) -> dict[str, object]:
        controller = getattr(self.ulog_state, "_workstation_ulog_controller", None)
        if not isinstance(controller, self.controller_type()):
            return {
                "schema": "workstation-ulog-controller-v1",
                "state": "NOT_PREPARED",
                "artifacts": [],
                "sources": {},
            }
        return controller.ulog_snapshot()

    def import_workstation_ulog(
        self,
        source: str,
        path: str,
        *,
        ownership: str = "UNKNOWN",
        expected_sha256: str | None = None,
    ) -> dict[str, object]:
        """Import one exact P1/P2/target ULog into this session."""

        self.photo_capture._prepare_pnp_evidence_session()
        controller = self._ensure_ulog_controller()
        source_value = str(source).strip().lower()
        if source_value in {"target", "drone", "target_drone"}:
            source_value = MavlinkSourceId.TARGET_DRONE.value
        resolved_source = MavlinkSourceId(source_value)
        artifact = controller.import_manual_ulog(
            resolved_source,
            path,
            ownership=ManualOwnershipClaim(str(ownership).upper()),
            expected_sha256=expected_sha256,
        )
        absolute = controller.session_root / str(artifact["relative_path"])
        registration = self._register_session_owned_artifact(
            absolute,
            role=f"ulog/{resolved_source.value}",
            media_type="application/vnd.px4.ulog",
            metadata=dict(artifact),
            expected_size_bytes=artifact.get("size_bytes"),
            expected_sha256=artifact.get("sha256"),
        )
        registration_error = persistence_failure_error(registration)
        if registration_error is not None:
            raise OSError(f"ULog artifact registration failed: {registration_error}")
        append_event = getattr(self.boundary, "append_workstation_run_event", None)
        if callable(append_event):
            append_event(
                "ulog_imported",
                dict(artifact),
                source="RecordingController",
                coordinate_contract="opaque PX4 ULog; no optical RDF conversion",
            )
        return controller.ulog_snapshot()
