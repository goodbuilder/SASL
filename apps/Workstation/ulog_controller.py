"""Session-owned PX4 ULog acquisition with fail-closed logger ownership.

The controller deliberately keeps MAVLink/MAVFTP I/O behind
``ULogHardwareAdapter``.  It uses :mod:`workstation_core.mavlink_sources` for
the logger ownership state machine and ULog framing validation, while owning
only the command ordering and durable local artifact boundary.

Coordinate contract
-------------------
ULog bytes are stored without interpreting their message payloads.  A ULog
may contain WGS84, world NED, PX4 body FRD, sensor, and estimator-local fields;
their individual PX4/uORB message definitions remain authoritative.  The
controller never converts these bytes to OpenCV optical RDF or stereo-centre
coordinates and never labels runtime telemetry as calibration truth.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, BinaryIO, Mapping, Protocol, Sequence

from Workstation.workstation_core.mavlink_sources import (
    LoggerCommandReceipt,
    LoggerControlAction,
    LoggerControlDecision,
    LoggerLifecyclePhase,
    LoggerLifecycleState,
    LoggerOwner,
    LoggerRuntimeIdentity,
    LoggerStatusEvidence,
    MavlinkSourceConfig,
    MavlinkSourceId,
    ULogArtifactValidation,
    ULogInventoryEntry,
    decide_logger_start,
    decide_logger_stop,
    observe_logger_preflight,
    record_logger_start_result,
    record_logger_stop_result,
    session_ulog_candidates,
    validate_ulog_artifact,
)


ULOG_CONTROLLER_METHODS = (
    "start_automatic_ulog",
    "stop_automatic_ulog_and_collect",
    "import_manual_ulog",
    "ulog_snapshot",
)
ULOG_ARTIFACT_SCHEMA = "workstation-session-ulog-v1"
ULOG_SNAPSHOT_SCHEMA = "workstation-ulog-controller-v1"
ULOG_COORDINATE_CONTRACT = (
    "opaque PX4 ULog bytes; per-message PX4/uORB definitions declare WGS84, "
    "world NED, PX4 body FRD, sensor, or estimator-local axes; no optical-RDF "
    "conversion; runtime reference is not calibration truth"
)

_SAFE_NAME = re.compile(r"[^A-Za-z0-9._-]+")


class ULogControllerError(RuntimeError):
    """Base error for fail-closed ULog acquisition."""


class ULogArtifactValidationError(ULogControllerError):
    """Downloaded or imported bytes failed ULog integrity validation."""


class ULogAcquisitionKind(str, Enum):
    AUTOMATIC = "automatic"
    MANUAL = "manual"


class ManualOwnershipClaim(str, Enum):
    """Operator-declared provenance for a manually supplied file.

    A manual claim is never promoted to a logger ownership token.  In
    particular, choosing ``WORKSTATION`` does not authorize a later logger
    stop command.
    """

    UNKNOWN = "UNKNOWN"
    WORKSTATION = "WORKSTATION"
    EXTERNAL = "EXTERNAL"


class ULogHardwareAdapter(Protocol):
    """Narrow injected boundary for one MAVLink source.

    Implementations must bind every returned status, receipt, inventory row,
    and download to the same MAVLink system/component and connection
    generation.  The controller verifies those bindings before mutation.
    """

    def current_runtime_identity(self) -> LoggerRuntimeIdentity:
        ...

    def query_logger_status(self, *, query_id: str) -> LoggerStatusEvidence:
        ...

    def execute_logger_command(
        self,
        decision: LoggerControlDecision,
    ) -> LoggerCommandReceipt:
        ...

    def list_ulog_inventory(
        self,
        *,
        session_id: str,
    ) -> Sequence[ULogInventoryEntry]:
        ...

    def download_ulog(self, entry: ULogInventoryEntry, writer: BinaryIO) -> None:
        ...


@dataclass(frozen=True)
class AutomaticULogBinding:
    config: MavlinkSourceConfig
    adapter: ULogHardwareAdapter

    def __post_init__(self) -> None:
        if not isinstance(self.config, MavlinkSourceConfig):
            raise TypeError("config must be MavlinkSourceConfig")
        required = (
            "current_runtime_identity",
            "query_logger_status",
            "execute_logger_command",
            "list_ulog_inventory",
            "download_ulog",
        )
        missing = [name for name in required if not callable(getattr(self.adapter, name, None))]
        if missing:
            raise TypeError("adapter is missing methods: " + ", ".join(missing))


def _source_id(value: MavlinkSourceId | str) -> MavlinkSourceId:
    text = str(getattr(value, "value", value)).strip().lower()
    aliases = {
        "p1": MavlinkSourceId.LEFT,
        "rig_p1": MavlinkSourceId.LEFT,
        "left": MavlinkSourceId.LEFT,
        "p2": MavlinkSourceId.RIGHT,
        "rig_p2": MavlinkSourceId.RIGHT,
        "right": MavlinkSourceId.RIGHT,
        "target": MavlinkSourceId.TARGET_DRONE,
        "target_reference": MavlinkSourceId.TARGET_DRONE,
        "target_drone": MavlinkSourceId.TARGET_DRONE,
    }
    try:
        return aliases[text]
    except KeyError as exc:
        raise ValueError(f"unknown ULog source role: {value!r}") from exc


def _source_role(source: MavlinkSourceId) -> str:
    return {
        MavlinkSourceId.LEFT: "RIG_P1",
        MavlinkSourceId.RIGHT: "RIG_P2",
        MavlinkSourceId.TARGET_DRONE: "TARGET_REFERENCE",
    }[source]


def _required_text(value: object, name: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be str")
    text = value.strip()
    if not text:
        raise ValueError(f"{name} must not be empty")
    if any(character in text for character in ("/", "\\", "\x00")):
        raise ValueError(f"{name} must not contain path separators")
    return text


def _runtime_mapping(identity: LoggerRuntimeIdentity) -> dict[str, Any]:
    return {
        "source_id": identity.source_id.value,
        "connection_generation": identity.connection_generation,
        "boot_id": identity.boot_id,
        "source_system": identity.source_system,
        "source_component": identity.source_component,
    }


def _decision_mapping(decision: LoggerControlDecision | None) -> dict[str, Any] | None:
    if decision is None:
        return None
    return {
        "action": decision.action.value,
        "reason": decision.reason.value,
        "runtime_identity": (
            None
            if decision.runtime_identity is None
            else _runtime_mapping(decision.runtime_identity)
        ),
        "session_id": decision.session_id,
        "observed_remote_log_path": decision.observed_remote_log_path,
        "command_id": decision.command_id,
        "status_observed_at_monotonic_ns": decision.status_observed_at_monotonic_ns,
        "status_query_id": decision.status_query_id,
    }


def _validation_mapping(validation: ULogArtifactValidation) -> dict[str, Any]:
    return {
        "valid": validation.valid,
        "expected_size_bytes": validation.expected_size_bytes,
        "actual_size_bytes": validation.actual_size_bytes,
        "sha256": validation.sha256,
        "header_valid": validation.header_valid,
        "record_structure_valid": validation.record_structure_valid,
        "record_count": validation.record_count,
        "version": validation.version,
        "start_timestamp_us": validation.start_timestamp_us,
        "errors": [error.value for error in validation.errors],
    }


@dataclass(frozen=True)
class SessionULogArtifact:
    artifact_id: str
    session_id: str
    source_id: MavlinkSourceId
    source_role: str
    acquisition: ULogAcquisitionKind
    ownership: str
    ownership_basis: str
    relative_path: str
    metadata_relative_path: str
    original_name: str
    remote_path: str | None
    size_bytes: int
    sha256: str
    imported_utc_ns: int
    imported_monotonic_ns: int
    source_mtime_ns: int | None
    runtime_identity: LoggerRuntimeIdentity
    runtime_identity_basis: str
    listed_at_utc_ns: int | None
    record_count: int
    ulog_version: int
    ulog_start_timestamp_us: int
    ownership_token_sha256: str | None = None
    coordinate_contract: str = ULOG_COORDINATE_CONTRACT

    def __post_init__(self) -> None:
        object.__setattr__(self, "source_id", MavlinkSourceId(self.source_id))
        object.__setattr__(self, "acquisition", ULogAcquisitionKind(self.acquisition))
        if not isinstance(self.runtime_identity, LoggerRuntimeIdentity):
            raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
        if self.runtime_identity.source_id != self.source_id:
            raise ValueError("artifact source and runtime identity source differ")
        if not re.fullmatch(r"[0-9a-f]{64}", self.sha256):
            raise ValueError("sha256 must be a lowercase SHA-256 digest")
        if self.ownership_token_sha256 is not None and not re.fullmatch(
            r"[0-9a-f]{64}", self.ownership_token_sha256
        ):
            raise ValueError("ownership_token_sha256 must be a SHA-256 digest")

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema": ULOG_ARTIFACT_SCHEMA,
            "artifact_id": self.artifact_id,
            "session_id": self.session_id,
            "source_id": self.source_id.value,
            "source_role": self.source_role,
            "acquisition": self.acquisition.value,
            "ownership": self.ownership,
            "ownership_basis": self.ownership_basis,
            "relative_path": self.relative_path,
            "metadata_relative_path": self.metadata_relative_path,
            "original_name": self.original_name,
            "remote_path": self.remote_path,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "imported_utc_ns": self.imported_utc_ns,
            "imported_monotonic_ns": self.imported_monotonic_ns,
            "source_mtime_ns": self.source_mtime_ns,
            "runtime_identity": _runtime_mapping(self.runtime_identity),
            "runtime_identity_basis": self.runtime_identity_basis,
            "listed_at_utc_ns": self.listed_at_utc_ns,
            "record_count": self.record_count,
            "ulog_version": self.ulog_version,
            "ulog_start_timestamp_us": self.ulog_start_timestamp_us,
            "ownership_token_sha256": self.ownership_token_sha256,
            "coordinate_contract": self.coordinate_contract,
            "truth_semantics": "runtime_reference_not_calibration_truth",
            "media_type": "application/vnd.px4.ulog",
            "validated": True,
        }


@dataclass
class _AutomaticState:
    binding: AutomaticULogBinding
    lifecycle: LoggerLifecycleState | None = None
    inventory_before: tuple[ULogInventoryEntry, ...] = ()
    last_decision: LoggerControlDecision | None = None
    last_error: str | None = None


class ULogController:
    """Own automatic logger lifecycles and exact session ULog bytes."""

    def __init__(
        self,
        session_root: Path | str,
        session_id: str,
        *,
        automatic_sources: Mapping[
            MavlinkSourceId | str,
            AutomaticULogBinding | tuple[MavlinkSourceConfig, ULogHardwareAdapter],
        ]
        | None = None,
    ) -> None:
        self.session_root = Path(session_root).expanduser().resolve()
        self.session_root.mkdir(parents=True, exist_ok=True)
        self.session_id = _required_text(session_id, "session_id")
        self._states: dict[MavlinkSourceId, _AutomaticState] = {}
        self._artifacts: list[SessionULogArtifact] = []
        self._manifest_path = self.session_root / "ulog_artifacts.jsonl"
        self._lock = threading.RLock()
        for source, value in (automatic_sources or {}).items():
            binding = (
                value
                if isinstance(value, AutomaticULogBinding)
                else AutomaticULogBinding(value[0], value[1])
            )
            self.bind_automatic_source(source, binding.config, binding.adapter)

    def bind_automatic_source(
        self,
        source: MavlinkSourceId | str,
        config: MavlinkSourceConfig,
        adapter: ULogHardwareAdapter,
    ) -> None:
        resolved = _source_id(source)
        binding = AutomaticULogBinding(config, adapter)
        if binding.config.source_id != resolved:
            raise ValueError("binding source differs from its MAVLink source config")
        with self._lock:
            existing = self._states.get(resolved)
            if existing is not None and existing.lifecycle is not None:
                raise RuntimeError("cannot replace a source after logger preflight")
            self._states[resolved] = _AutomaticState(binding=binding)

    def start_automatic_ulog(self, source: MavlinkSourceId | str) -> dict[str, Any]:
        """Preflight and start only an idle logger with a verified identity."""

        resolved = _source_id(source)
        with self._lock:
            state = self._require_automatic_state(resolved)
            if (
                state.lifecycle is not None
                and state.lifecycle.owner == LoggerOwner.WORKSTATION
            ):
                raise RuntimeError("Workstation already owns this source logger")
            try:
                adapter = state.binding.adapter
                identity = self._current_identity(state)
                preflight = self._query_status(state, "preflight")
                if preflight.runtime_identity != identity:
                    raise ULogControllerError("preflight status runtime identity mismatch")
                lifecycle = observe_logger_preflight(identity, preflight.status)
                state.lifecycle = lifecycle
                state.inventory_before = self._validated_inventory(state, identity)
                decision = decide_logger_start(
                    state.binding.config,
                    lifecycle,
                    session_id=self.session_id,
                    command_id=self._command_id(resolved, "start"),
                )
                state.last_decision = decision
                state.last_error = None
                if decision.action == LoggerControlAction.NONE:
                    # An already-running logger remains externally owned.  No
                    # command, ownership token, or automatic download follows.
                    return self.ulog_snapshot()
                receipt = adapter.execute_logger_command(decision)
                post_start = self._query_status(state, "post-start")
                state.lifecycle = record_logger_start_result(
                    lifecycle,
                    decision,
                    post_start,
                    receipt=receipt,
                )
                return self.ulog_snapshot()
            except Exception as exc:
                state.last_error = f"{type(exc).__name__}: {exc}"
                raise

    def stop_automatic_ulog_and_collect(
        self,
        source: MavlinkSourceId | str,
        *,
        expected_sha256: str | None = None,
    ) -> dict[str, Any]:
        """Stop exactly the logger we own, then validate and seal its ULog.

        The stop command is never issued for a pre-existing logger, a changed
        boot or connection generation, a different session, or a changed
        remote log path.  A denied decision returns a snapshot and performs no
        download.
        """

        resolved = _source_id(source)
        with self._lock:
            state = self._require_automatic_state(resolved)
            if state.lifecycle is None:
                raise RuntimeError("logger preflight has not run")
            try:
                identity = self._current_identity(state)
                fresh = self._query_status(state, "pre-stop")
                decision = decide_logger_stop(
                    state.binding.config,
                    state.lifecycle,
                    current_runtime_identity=identity,
                    session_id=self.session_id,
                    fresh_status=fresh,
                    command_id=self._command_id(resolved, "stop"),
                )
                state.last_decision = decision
                state.last_error = None
                if decision.action == LoggerControlAction.NONE:
                    return self.ulog_snapshot()

                token = state.lifecycle.ownership_token
                if token is None:
                    raise ULogControllerError("STOP decision has no ownership token")
                receipt = state.binding.adapter.execute_logger_command(decision)
                post_stop = self._query_status(state, "post-stop")
                state.lifecycle = record_logger_stop_result(
                    state.lifecycle,
                    decision,
                    post_stop,
                    receipt=receipt,
                )
                if state.lifecycle.phase != LoggerLifecyclePhase.STOPPED:
                    # The token is deliberately consumed by the pure state
                    # machine after one stop attempt.  A still-active or
                    # unknown logger is not downloaded as a final artifact.
                    return self.ulog_snapshot()

                inventory_after = self._validated_inventory(state, identity)
                candidates = session_ulog_candidates(
                    state.inventory_before,
                    inventory_after,
                )
                exact = [
                    entry
                    for entry in candidates
                    if entry.remote_path == token.initial_remote_log_path
                    and entry.runtime_identity == token.runtime_identity
                    and entry.session_id == token.session_id
                ]
                if len(exact) != 1:
                    raise ULogControllerError(
                        "stopped logger path is not one exact NEW/GROWN session ULog"
                    )
                token_payload = json.dumps(
                    {
                        "runtime_identity": _runtime_mapping(token.runtime_identity),
                        "session_id": token.session_id,
                        "initial_remote_log_path": token.initial_remote_log_path,
                        "start_command_id": token.start_command_id,
                        "verified_at_monotonic_ns": token.verified_at_monotonic_ns,
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
                artifact = self._download_automatic(
                    state,
                    exact[0],
                    expected_sha256=expected_sha256,
                    ownership_token_sha256=hashlib.sha256(token_payload).hexdigest(),
                )
                self._artifacts.append(artifact)
                return self.ulog_snapshot()
            except Exception as exc:
                state.last_error = f"{type(exc).__name__}: {exc}"
                raise

    def import_manual_ulog(
        self,
        source: MavlinkSourceId | str,
        source_path: Path | str,
        *,
        ownership: ManualOwnershipClaim | str = ManualOwnershipClaim.UNKNOWN,
        runtime_identity: LoggerRuntimeIdentity | None = None,
        connection_generation: int = 0,
        boot_id: str = "manual-import-unverified",
        source_system: int | None = None,
        source_component: int | None = None,
        expected_sha256: str | None = None,
        source_observed_utc_ns: int | None = None,
    ) -> dict[str, Any]:
        """Copy and validate a P1/P2/target ULog without mutating a vehicle.

        Source and ownership are operator-declared metadata.  They never
        create an automatic logger ownership token, even if ``WORKSTATION`` is
        selected.
        """

        resolved = _source_id(source)
        claim = ManualOwnershipClaim(str(getattr(ownership, "value", ownership)).upper())
        source_file = Path(source_path).expanduser().resolve(strict=True)
        if not source_file.is_file():
            raise ValueError("manual ULog source must be a regular file")
        with self._lock:
            identity, identity_basis = self._manual_identity(
                resolved,
                runtime_identity=runtime_identity,
                connection_generation=connection_generation,
                boot_id=boot_id,
                source_system=source_system,
                source_component=source_component,
            )
            stat_result = source_file.stat()
            entry = ULogInventoryEntry(
                runtime_identity=identity,
                session_id=self.session_id,
                remote_path=f"/manual/{self._safe_ulog_name(source_file.name)}",
                size_bytes=stat_result.st_size,
                listed_at_utc_ns=source_observed_utc_ns,
            )
            artifact = self._copy_validate_commit(
                entry=entry,
                original_name=source_file.name,
                acquisition=ULogAcquisitionKind.MANUAL,
                ownership=claim.value,
                ownership_basis="operator_declared_unverified",
                runtime_identity_basis=identity_basis,
                source_mtime_ns=stat_result.st_mtime_ns,
                expected_sha256=expected_sha256,
                copy=lambda writer: self._copy_file(source_file, writer),
                ownership_token_sha256=None,
            )
            self._artifacts.append(artifact)
            return artifact.to_mapping()

    def ulog_snapshot(self) -> dict[str, Any]:
        with self._lock:
            sources: dict[str, Any] = {}
            for source in MavlinkSourceId:
                state = self._states.get(source)
                lifecycle = None if state is None else state.lifecycle
                sources[source.value] = {
                    "configured": state is not None,
                    "source_role": _source_role(source),
                    "phase": None if lifecycle is None else lifecycle.phase.value,
                    "owner": None if lifecycle is None else lifecycle.owner.value,
                    "runtime_identity": (
                        None
                        if lifecycle is None
                        else _runtime_mapping(lifecycle.runtime_identity)
                    ),
                    "remote_log_path": (
                        None
                        if lifecycle is None
                        else lifecycle.current_status.remote_log_path
                    ),
                    "ownership_token_present": bool(
                        lifecycle is not None and lifecycle.ownership_token is not None
                    ),
                    "last_decision": (
                        None if state is None else _decision_mapping(state.last_decision)
                    ),
                    "last_error": None if state is None else state.last_error,
                    "artifact_count": sum(
                        artifact.source_id == source for artifact in self._artifacts
                    ),
                }
            return {
                "schema": ULOG_SNAPSHOT_SCHEMA,
                "session_id": self.session_id,
                "session_root": str(self.session_root),
                "coordinate_contract": ULOG_COORDINATE_CONTRACT,
                "sources": sources,
                "artifacts": [artifact.to_mapping() for artifact in self._artifacts],
            }

    def _require_automatic_state(self, source: MavlinkSourceId) -> _AutomaticState:
        try:
            return self._states[source]
        except KeyError as exc:
            raise RuntimeError(f"automatic ULog source is not configured: {source.value}") from exc

    @staticmethod
    def _command_id(source: MavlinkSourceId, action: str) -> str:
        return f"workstation-{source.value}-{action}-{uuid.uuid4().hex}"

    @staticmethod
    def _query_id(source: MavlinkSourceId, purpose: str) -> str:
        return f"workstation-{source.value}-{purpose}-{uuid.uuid4().hex}"

    def _current_identity(self, state: _AutomaticState) -> LoggerRuntimeIdentity:
        identity = state.binding.adapter.current_runtime_identity()
        if not isinstance(identity, LoggerRuntimeIdentity):
            raise TypeError("adapter returned a non-LoggerRuntimeIdentity")
        if identity.source_id != state.binding.config.source_id:
            raise ULogControllerError("adapter runtime source differs from configured source")
        return identity

    def _query_status(self, state: _AutomaticState, purpose: str) -> LoggerStatusEvidence:
        query_id = self._query_id(state.binding.config.source_id, purpose)
        evidence = state.binding.adapter.query_logger_status(query_id=query_id)
        if not isinstance(evidence, LoggerStatusEvidence):
            raise TypeError("adapter returned a non-LoggerStatusEvidence")
        if evidence.query_id != query_id:
            raise ULogControllerError("logger status query correlation mismatch")
        return evidence

    def _validated_inventory(
        self,
        state: _AutomaticState,
        identity: LoggerRuntimeIdentity,
    ) -> tuple[ULogInventoryEntry, ...]:
        entries = tuple(
            state.binding.adapter.list_ulog_inventory(session_id=self.session_id)
        )
        for entry in entries:
            if not isinstance(entry, ULogInventoryEntry):
                raise TypeError("adapter returned a non-ULogInventoryEntry")
            if entry.runtime_identity != identity:
                raise ULogControllerError("ULog inventory runtime identity mismatch")
            if entry.session_id != self.session_id:
                raise ULogControllerError("ULog inventory session mismatch")
        return entries

    def _download_automatic(
        self,
        state: _AutomaticState,
        entry: ULogInventoryEntry,
        *,
        expected_sha256: str | None,
        ownership_token_sha256: str,
    ) -> SessionULogArtifact:
        return self._copy_validate_commit(
            entry=entry,
            original_name=Path(entry.remote_path).name,
            acquisition=ULogAcquisitionKind.AUTOMATIC,
            ownership=LoggerOwner.WORKSTATION.value,
            ownership_basis="verified_logger_ownership_token",
            runtime_identity_basis="heartbeat_boot_and_connection_verified",
            source_mtime_ns=None,
            expected_sha256=expected_sha256,
            copy=lambda writer: state.binding.adapter.download_ulog(entry, writer),
            ownership_token_sha256=ownership_token_sha256,
        )

    def _manual_identity(
        self,
        source: MavlinkSourceId,
        *,
        runtime_identity: LoggerRuntimeIdentity | None,
        connection_generation: int,
        boot_id: str,
        source_system: int | None,
        source_component: int | None,
    ) -> tuple[LoggerRuntimeIdentity, str]:
        if runtime_identity is not None:
            if not isinstance(runtime_identity, LoggerRuntimeIdentity):
                raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
            if runtime_identity.source_id != source:
                raise ValueError("manual runtime identity source mismatch")
            return runtime_identity, "caller_supplied_operator_evidence"
        config = self._states.get(source)
        configured_system = (
            None if config is None else config.binding.config.expected_system_id
        )
        configured_component = (
            None if config is None else config.binding.config.expected_component_id
        )
        resolved_system = (
            source_system
            if source_system is not None
            else configured_system if configured_system is not None else 255
        )
        resolved_component = (
            source_component
            if source_component is not None
            else configured_component if configured_component is not None else 255
        )
        placeholder = source_system is None or source_component is None
        return (
            LoggerRuntimeIdentity(
                source_id=source,
                connection_generation=connection_generation,
                boot_id=boot_id,
                source_system=resolved_system,
                source_component=resolved_component,
            ),
            (
                "operator_declared_with_unknown_fields_placeholder_255"
                if placeholder
                else "operator_declared"
            ),
        )

    def _copy_validate_commit(
        self,
        *,
        entry: ULogInventoryEntry,
        original_name: str,
        acquisition: ULogAcquisitionKind,
        ownership: str,
        ownership_basis: str,
        runtime_identity_basis: str,
        source_mtime_ns: int | None,
        expected_sha256: str | None,
        copy: Any,
        ownership_token_sha256: str | None,
    ) -> SessionULogArtifact:
        source_dir = self.session_root / "ulog" / entry.source_id.value
        source_dir.mkdir(parents=True, exist_ok=True)
        part = source_dir / f".{uuid.uuid4().hex}.ulg.part"
        try:
            with part.open("xb") as writer:
                copy(writer)
                writer.flush()
                os.fsync(writer.fileno())

            payload = part.read_bytes()
            validation = validate_ulog_artifact(
                entry,
                payload,
                expected_sha256=expected_sha256,
            )
            if not validation.valid:
                details = ",".join(error.value for error in validation.errors)
                raise ULogArtifactValidationError(
                    f"ULog validation failed: {details or 'unknown error'}"
                )
            safe_name = self._safe_ulog_name(original_name)
            final = source_dir / f"{Path(safe_name).stem}-{validation.sha256[:16]}.ulg"
            if final.exists():
                if (
                    final.stat().st_size != validation.actual_size_bytes
                    or self._sha256_file(final) != validation.sha256
                ):
                    raise ULogControllerError("content-addressed ULog collision")
                part.unlink()
            else:
                os.replace(part, final)
                self._fsync_directory(source_dir)
            if (
                final.stat().st_size != validation.actual_size_bytes
                or self._sha256_file(final) != validation.sha256
            ):
                raise ULogControllerError("sealed ULog changed after atomic rename")

            imported_utc_ns = time.time_ns()
            imported_monotonic_ns = time.monotonic_ns()
            artifact_id = str(uuid.uuid4())
            metadata_path = final.with_name(f"{final.name}.{artifact_id}.json")
            artifact = SessionULogArtifact(
                artifact_id=artifact_id,
                session_id=self.session_id,
                source_id=entry.source_id,
                source_role=_source_role(entry.source_id),
                acquisition=acquisition,
                ownership=ownership,
                ownership_basis=ownership_basis,
                relative_path=final.relative_to(self.session_root).as_posix(),
                metadata_relative_path=metadata_path.relative_to(self.session_root).as_posix(),
                original_name=original_name,
                remote_path=(
                    entry.remote_path if acquisition == ULogAcquisitionKind.AUTOMATIC else None
                ),
                size_bytes=validation.actual_size_bytes,
                sha256=validation.sha256,
                imported_utc_ns=imported_utc_ns,
                imported_monotonic_ns=imported_monotonic_ns,
                source_mtime_ns=source_mtime_ns,
                runtime_identity=entry.runtime_identity,
                runtime_identity_basis=runtime_identity_basis,
                listed_at_utc_ns=entry.listed_at_utc_ns,
                record_count=validation.record_count,
                ulog_version=int(validation.version),
                ulog_start_timestamp_us=int(validation.start_timestamp_us),
                ownership_token_sha256=ownership_token_sha256,
            )
            metadata = artifact.to_mapping()
            metadata["validation"] = _validation_mapping(validation)
            self._atomic_write_json(metadata_path, metadata)
            self._append_json_line(self._manifest_path, metadata)
            return artifact
        except Exception:
            try:
                part.unlink()
            except FileNotFoundError:
                pass
            raise

    @staticmethod
    def _copy_file(source: Path, writer: BinaryIO) -> None:
        with source.open("rb") as reader:
            while True:
                chunk = reader.read(1024 * 1024)
                if not chunk:
                    return
                writer.write(chunk)

    @staticmethod
    def _safe_ulog_name(value: str) -> str:
        name = Path(str(value)).name
        safe = _SAFE_NAME.sub("_", name).strip("._") or "artifact.ulg"
        if not safe.lower().endswith(".ulg"):
            safe += ".ulg"
        return safe

    @staticmethod
    def _sha256_file(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    return digest.hexdigest()
                digest.update(chunk)

    @staticmethod
    def _append_json_line(path: Path, value: Mapping[str, Any]) -> None:
        data = json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8") + b"\n"
        with path.open("ab") as writer:
            writer.write(data)
            writer.flush()
            os.fsync(writer.fileno())

    @staticmethod
    def _atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
        temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.part")
        try:
            with temporary.open("xb") as writer:
                writer.write(
                    json.dumps(
                        value,
                        ensure_ascii=False,
                        allow_nan=False,
                        sort_keys=True,
                        separators=(",", ":"),
                    ).encode("utf-8")
                    + b"\n"
                )
                writer.flush()
                os.fsync(writer.fileno())
            os.replace(temporary, path)
            ULogController._fsync_directory(path.parent)
        except Exception:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
            raise

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        descriptor = os.open(path, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


__all__ = [
    "AutomaticULogBinding",
    "ManualOwnershipClaim",
    "SessionULogArtifact",
    "ULOG_ARTIFACT_SCHEMA",
    "ULOG_CONTROLLER_METHODS",
    "ULOG_COORDINATE_CONTRACT",
    "ULOG_SNAPSHOT_SCHEMA",
    "ULogAcquisitionKind",
    "ULogArtifactValidationError",
    "ULogController",
    "ULogControllerError",
    "ULogHardwareAdapter",
]
