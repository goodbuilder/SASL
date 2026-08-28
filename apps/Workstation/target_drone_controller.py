"""Session owner for the receive-only target-drone MAVLink stream.

The controller deliberately exposes no command or mutation API.  In Drone
mode it owns one :class:`TargetDroneReceiver` and its worker, drains the
receive queue during stop, and then seals the session-scoped JSONL spool.
In UAM mode live RTK is explicitly deferred: no worker thread and no MAVLink
connection are created.

Coordinate contract
-------------------
The controller does not convert telemetry.  Rows retain the producer
contract declared by ``target_drone_stream``: WGS84 geodetic position, world
NED, and PX4 body FRD.  ATTITUDE yaw, GNSS yaw, and course over ground remain
separate observations.  The sealed artifact is runtime reference evidence,
never calibration-time truth and never an OpenCV optical-RDF pose.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import stat
import threading
import time
from typing import Any, Callable, Mapping
import uuid

from Workstation.workstation_core.mavlink_sources import MavlinkSourceConfig
from Workstation.workstation_core.target_drone_stream import (
    TARGET_DRONE_SCHEMA_VERSION,
    TargetDroneMode,
    TargetDroneReceiver,
    TargetDroneStreamSnapshot,
    TargetDroneStreamState,
    TargetDroneWorker,
)


TARGET_DRONE_CONTROLLER_METHODS = (
    "configure_target_drone_stream",
    "connect_target_drone_stream",
    "start_target_drone_stream",
    "drain_target_drone_stream",
    "stop_target_drone_stream",
    "target_drone_stream_snapshot",
    "target_drone_spool_artifact",
)

TARGET_DRONE_ARTIFACT_SCHEMA = "workstation-target-drone-spool-v1"
TARGET_DRONE_SOURCE_ID = "target_drone"
TARGET_DRONE_SOURCE_ROLE = "TARGET_REFERENCE"
TARGET_DRONE_COORDINATE_CONTRACT = (
    "position=WGS84 lat/lon + MSL/ellipsoid altitude; world=NED; body=PX4 FRD; "
    "ATTITUDE yaw, GNSS yaw, and COG are distinct clockwise-from-north observations"
)


class TargetDroneControllerError(RuntimeError):
    """Base controller ownership error."""


class TargetDroneArtifactIntegrityError(TargetDroneControllerError):
    """A stopped spool could not be proven complete and unchanged."""


class TargetDroneControllerPhase(str, Enum):
    UNCONFIGURED = "UNCONFIGURED"
    CONFIGURED = "CONFIGURED"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    DEFERRED = "DEFERRED"
    STOPPED = "STOPPED"
    SEALED = "SEALED"
    FAILED = "FAILED"


class _MeasurementDataTargetSpool:
    """Spool-compatible adapter for the sessionless bounded data writer.

    TargetDroneReceiver can keep its receive/identity state machine while
    persistence becomes best-effort and type-owned. Rows outside Measurement
    or Manual record intervals are deliberately ignored.
    """

    def __init__(self, panel: object, path: Path) -> None:
        self.panel = panel
        self.path = Path(path)
        self._open = False
        self._row_count = 0
        self._size_bytes = 0
        self._lock = threading.RLock()

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._open

    @property
    def row_count(self) -> int:
        with self._lock:
            return self._row_count

    @property
    def size_bytes(self) -> int:
        with self._lock:
            return self._size_bytes

    def open(self) -> None:
        with self._lock:
            self._open = True

    def append(self, row: Mapping[str, Any]) -> None:
        with self._lock:
            if not self._open:
                raise RuntimeError("target-drone data sink is not open")
        active_reader = getattr(self.panel, "_session_recording_active", None)
        if not (callable(active_reader) and bool(active_reader())):
            return
        writer = getattr(self.panel, "append_workstation_data", None)
        if not callable(writer):
            return
        try:
            payload = dict(row)
            encoded_size = len(
                json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ).encode("utf-8")
            ) + 1
            accepted = bool(writer("px4.drone", payload, priority=False))
        except (TypeError, ValueError, OSError):
            accepted = False
            encoded_size = 0
        if accepted:
            with self._lock:
                self._row_count += 1
                self._size_bytes += encoded_size

    def flush(self) -> None:
        flusher = getattr(self.panel, "flush_workstation_data", None)
        if callable(flusher):
            try:
                flusher()
            except Exception:
                pass

    def close(self) -> None:
        with self._lock:
            self._open = False
        self.flush()


def _mode(value: object) -> TargetDroneMode:
    return TargetDroneMode(str(getattr(value, "value", value)).strip().upper())


def _optional_mavlink_id(value: object, name: str) -> int | None:
    if value in (None, ""):
        return None
    if isinstance(value, bool):
        raise TypeError(f"{name} must be an integer or None")
    parsed = int(value)
    if not 1 <= parsed <= 255:
        raise ValueError(f"{name} must be in [1, 255]")
    return parsed


def _positive_int(value: object, name: str) -> int:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a positive integer")
    parsed = int(value)
    if parsed <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return parsed


def _positive_float(value: object, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return parsed


def _enum_value(value: object) -> str:
    return str(getattr(value, "value", value))


def _source_id(config: object) -> str:
    return _enum_value(getattr(config, "source_id", ""))


def _source_role(config: object) -> str:
    return _enum_value(getattr(config, "role", ""))


def _validate_recv_only_config(
    config: object,
    request: "TargetDroneRuntimeRequest | None" = None,
) -> None:
    """Validate by capability, minimizing coupling to policy enum versions."""

    if _source_id(config) != TARGET_DRONE_SOURCE_ID:
        raise ValueError("target stream must use source_id=target_drone")
    if _source_role(config) != TARGET_DRONE_SOURCE_ROLE:
        raise ValueError("target stream must use role=TARGET_REFERENCE")
    policy = getattr(config, "policy", None)
    if policy is None:
        raise ValueError("target stream config has no mutation policy")
    forbidden = {
        "stream setup": getattr(policy, "allow_stream_setup", True),
        "parameter write": getattr(policy, "allow_parameter_write", True),
        "RTCM injection": getattr(policy, "allow_rtcm_injection", True),
        "logger control": getattr(policy, "allow_logger_control", True),
    }
    enabled = [name for name, allowed in forbidden.items() if bool(allowed)]
    if enabled:
        raise ValueError("target stream must be receive-only; forbidden: " + ", ".join(enabled))
    ftp_access = _enum_value(getattr(policy, "ftp_access", "DISABLED")).upper()
    if ftp_access == "READ_WRITE":
        raise ValueError("target stream must not have MAVFTP write access")
    if request is not None:
        claimed = {
            "endpoint": str(getattr(config, "endpoint", "")),
            "baudrate": getattr(config, "baudrate", None),
            "expected_system_id": getattr(config, "expected_system_id", None),
            "expected_component_id": getattr(config, "expected_component_id", None),
        }
        requested = {
            "endpoint": str(request.endpoint),
            "baudrate": request.baudrate,
            "expected_system_id": request.expected_system_id,
            "expected_component_id": request.expected_component_id,
        }
        if claimed != requested:
            raise ValueError(
                "target source config identity differs from the operator runtime request"
            )


@dataclass(frozen=True)
class TargetDroneRuntimeRequest:
    """Validated operator request for one session-bound target source."""

    mode: TargetDroneMode | str
    endpoint: str | None = None
    baudrate: int = 57_600
    expected_system_id: int | None = None
    expected_component_id: int | None = None
    heartbeat_timeout_s: float = 3.0
    spool_max_rows: int = 1_000_000
    spool_max_bytes: int = 512 * 1024 * 1024
    spool_fsync_every_rows: int = 1

    def __post_init__(self) -> None:
        resolved_mode = _mode(self.mode)
        endpoint = str(self.endpoint or "").strip()
        if resolved_mode == TargetDroneMode.DRONE and not endpoint:
            raise ValueError("endpoint is required in Drone mode")
        if resolved_mode == TargetDroneMode.UAM and not endpoint:
            endpoint = "deferred://uam-post-flight-rtk"
        object.__setattr__(self, "mode", resolved_mode)
        object.__setattr__(self, "endpoint", endpoint)
        object.__setattr__(self, "baudrate", _positive_int(self.baudrate, "baudrate"))
        object.__setattr__(
            self,
            "expected_system_id",
            _optional_mavlink_id(self.expected_system_id, "expected_system_id"),
        )
        object.__setattr__(
            self,
            "expected_component_id",
            _optional_mavlink_id(self.expected_component_id, "expected_component_id"),
        )
        object.__setattr__(
            self,
            "heartbeat_timeout_s",
            _positive_float(self.heartbeat_timeout_s, "heartbeat_timeout_s"),
        )
        for name in ("spool_max_rows", "spool_max_bytes", "spool_fsync_every_rows"):
            object.__setattr__(self, name, _positive_int(getattr(self, name), name))

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "TargetDroneRuntimeRequest":
        return cls(
            mode=values.get("mode", values.get("target_platform", "DRONE")),
            endpoint=values.get("endpoint"),
            baudrate=values.get("baudrate", 57_600),
            expected_system_id=values.get("expected_system_id"),
            expected_component_id=values.get("expected_component_id"),
            heartbeat_timeout_s=values.get("heartbeat_timeout_s", 3.0),
            spool_max_rows=values.get("spool_max_rows", 1_000_000),
            spool_max_bytes=values.get("spool_max_bytes", 512 * 1024 * 1024),
            spool_fsync_every_rows=values.get("spool_fsync_every_rows", 1),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "mode": self.mode.value,
            "endpoint": self.endpoint,
            "baudrate": self.baudrate,
            "expected_system_id": self.expected_system_id,
            "expected_component_id": self.expected_component_id,
            "heartbeat_timeout_s": self.heartbeat_timeout_s,
            "spool_max_rows": self.spool_max_rows,
            "spool_max_bytes": self.spool_max_bytes,
            "spool_fsync_every_rows": self.spool_fsync_every_rows,
            "source_id": TARGET_DRONE_SOURCE_ID,
            "source_role": TARGET_DRONE_SOURCE_ROLE,
            "mutation_policy": "recv_only",
        }


@dataclass(frozen=True)
class TargetDroneSpoolArtifact:
    """Immutable description of a closed, content-verified JSONL spool."""

    artifact_id: str
    absolute_path: str
    relative_path: str
    session_id: str
    runtime_id: str
    mode: TargetDroneMode
    endpoint: str
    expected_system_id: int | None
    expected_component_id: int | None
    verified_source_system: int | None
    verified_source_component: int | None
    row_count: int
    accepted_message_count: int
    size_bytes: int
    sha256: str
    stopped_utc_ns: int
    read_only: bool = True

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema": TARGET_DRONE_ARTIFACT_SCHEMA,
            "artifact_id": self.artifact_id,
            "absolute_path": self.absolute_path,
            "relative_path": self.relative_path,
            "session_id": self.session_id,
            "runtime_id": self.runtime_id,
            "mode": self.mode.value,
            "source_id": TARGET_DRONE_SOURCE_ID,
            "source_role": TARGET_DRONE_SOURCE_ROLE,
            "endpoint": self.endpoint,
            "expected_system_id": self.expected_system_id,
            "expected_component_id": self.expected_component_id,
            "verified_source_system": self.verified_source_system,
            "verified_source_component": self.verified_source_component,
            "row_count": self.row_count,
            "accepted_message_count": self.accepted_message_count,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "media_type": "application/x-ndjson",
            "ledger_role": "target_drone_raw_mavlink_jsonl",
            "mutation_policy": "recv_only",
            "coordinate_contract": TARGET_DRONE_COORDINATE_CONTRACT,
            "truth_semantics": "runtime_reference_not_calibration_truth",
            "stopped_utc_ns": self.stopped_utc_ns,
            "closed": True,
            "verified": True,
            "read_only": self.read_only,
        }

    def run_evidence_import_kwargs(self) -> dict[str, Any]:
        """Arguments accepted by ``RunEvidenceLedger.import_artifact``.

        The metadata intentionally uses the session-relative path.  The
        absolute source path is supplied only as the copy input and need not
        be persisted by the ledger.
        """

        return {
            "source_path": Path(self.absolute_path),
            "role": "target_drone_raw_mavlink_jsonl",
            "media_type": "application/x-ndjson",
            "metadata": {
                "schema": TARGET_DRONE_ARTIFACT_SCHEMA,
                "session_source_relative_path": self.relative_path,
                "session_source_sha256": self.sha256,
                "session_source_size_bytes": self.size_bytes,
                "row_count": self.row_count,
                "accepted_message_count": self.accepted_message_count,
                "mode": self.mode.value,
                "source_id": TARGET_DRONE_SOURCE_ID,
                "source_role": TARGET_DRONE_SOURCE_ROLE,
                "endpoint": self.endpoint,
                "expected_system_id": self.expected_system_id,
                "expected_component_id": self.expected_component_id,
                "verified_source_system": self.verified_source_system,
                "verified_source_component": self.verified_source_component,
                "mutation_policy": "recv_only",
                "coordinate_contract": TARGET_DRONE_COORDINATE_CONTRACT,
                "truth_semantics": "runtime_reference_not_calibration_truth",
                "stopped_utc_ns": self.stopped_utc_ns,
            },
        }

    def verify(self) -> bool:
        """Return whether the bytes and row contract still match the seal."""

        try:
            _verify_sealed_artifact(self)
        except (OSError, ValueError, TargetDroneArtifactIntegrityError):
            return False
        return True


@dataclass(frozen=True)
class TargetDroneControllerSnapshot:
    phase: TargetDroneControllerPhase
    configured: bool
    session_id: str
    runtime_id: str
    mode: TargetDroneMode | None
    endpoint: str
    stream_state: TargetDroneStreamState | None
    worker_alive: bool
    expected_system_id: int | None
    expected_component_id: int | None
    verified_source_system: int | None
    verified_source_component: int | None
    accepted_message_count: int
    ignored_message_count: int
    rejected_cross_source_count: int
    rejected_invalid_provenance_count: int
    spool_row_count: int
    spool_size_bytes: int
    last_error: str | None
    artifact: TargetDroneSpoolArtifact | None

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema": "workstation-target-drone-runtime-v1",
            "phase": self.phase.value,
            "configured": self.configured,
            "session_id": self.session_id,
            "runtime_id": self.runtime_id,
            "mode": None if self.mode is None else self.mode.value,
            "source_id": TARGET_DRONE_SOURCE_ID,
            "source_role": TARGET_DRONE_SOURCE_ROLE,
            "endpoint": self.endpoint,
            "stream_state": None if self.stream_state is None else self.stream_state.value,
            "worker_alive": self.worker_alive,
            "expected_system_id": self.expected_system_id,
            "expected_component_id": self.expected_component_id,
            "verified_source_system": self.verified_source_system,
            "verified_source_component": self.verified_source_component,
            "accepted_message_count": self.accepted_message_count,
            "ignored_message_count": self.ignored_message_count,
            "rejected_cross_source_count": self.rejected_cross_source_count,
            "rejected_invalid_provenance_count": self.rejected_invalid_provenance_count,
            "spool_row_count": self.spool_row_count,
            "spool_size_bytes": self.spool_size_bytes,
            "last_error": self.last_error,
            "mutation_policy": "recv_only",
            "coordinate_contract": TARGET_DRONE_COORDINATE_CONTRACT,
            "truth_semantics": "runtime_reference_not_calibration_truth",
            "artifact": None if self.artifact is None else self.artifact.to_mapping(),
        }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _jsonl_row_count_and_validate(
    path: Path,
    *,
    verified_source_system: int | None,
    verified_source_component: int | None,
) -> int:
    rows = 0
    with path.open("rb") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.endswith(b"\n"):
                raise TargetDroneArtifactIntegrityError(
                    f"spool row {line_number} is not newline terminated"
                )
            try:
                row = json.loads(
                    line,
                    parse_constant=lambda token: (_ for _ in ()).throw(
                        ValueError(f"non-finite JSON constant: {token}")
                    ),
                )
            except Exception as exc:
                raise TargetDroneArtifactIntegrityError(
                    f"spool row {line_number} is invalid JSON"
                ) from exc
            if not isinstance(row, dict):
                raise TargetDroneArtifactIntegrityError(
                    f"spool row {line_number} is not an object"
                )
            if row.get("schema_version") != TARGET_DRONE_SCHEMA_VERSION:
                raise TargetDroneArtifactIntegrityError(
                    f"spool row {line_number} has the wrong schema"
                )
            if row.get("source_id") != TARGET_DRONE_SOURCE_ID:
                raise TargetDroneArtifactIntegrityError(
                    f"spool row {line_number} has the wrong source identity"
                )
            if row.get("source_role") != TARGET_DRONE_SOURCE_ROLE:
                raise TargetDroneArtifactIntegrityError(
                    f"spool row {line_number} has the wrong source role"
                )
            if row.get("record_kind") == "telemetry":
                provenance = row.get("provenance")
                if not isinstance(provenance, Mapping):
                    raise TargetDroneArtifactIntegrityError(
                        f"telemetry row {line_number} has no provenance"
                    )
                if provenance.get("source_id") != TARGET_DRONE_SOURCE_ID:
                    raise TargetDroneArtifactIntegrityError(
                        f"telemetry row {line_number} has cross-source provenance"
                    )
                if (
                    verified_source_system is not None
                    and provenance.get("source_system") != verified_source_system
                ):
                    raise TargetDroneArtifactIntegrityError(
                        f"telemetry row {line_number} has a different source system"
                    )
                if (
                    verified_source_component is not None
                    and provenance.get("source_component") != verified_source_component
                ):
                    raise TargetDroneArtifactIntegrityError(
                        f"telemetry row {line_number} has a different source component"
                    )
            rows += 1
    return rows


def _verify_sealed_artifact(artifact: TargetDroneSpoolArtifact) -> None:
    path = Path(artifact.absolute_path)
    if not path.is_file() or path.is_symlink():
        raise TargetDroneArtifactIntegrityError("sealed spool is missing or is not a regular file")
    stat_result = path.stat()
    if stat_result.st_size != artifact.size_bytes:
        raise TargetDroneArtifactIntegrityError("sealed spool byte size changed")
    if _sha256(path) != artifact.sha256:
        raise TargetDroneArtifactIntegrityError("sealed spool hash changed")
    rows = _jsonl_row_count_and_validate(
        path,
        verified_source_system=artifact.verified_source_system,
        verified_source_component=artifact.verified_source_component,
    )
    if rows != artifact.row_count:
        raise TargetDroneArtifactIntegrityError("sealed spool row count changed")


class TargetDroneController:
    """Own exactly one session-scoped target telemetry runtime at a time."""

    def __init__(
        self,
        panel: object,
        *,
        receiver_factory: Callable[..., TargetDroneReceiver] = TargetDroneReceiver,
        worker_factory: Callable[..., TargetDroneWorker] = TargetDroneWorker,
        source_config_factory: Callable[[TargetDroneRuntimeRequest], object] | None = None,
        utc_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        self.panel = panel
        self._receiver_factory = receiver_factory
        self._worker_factory = worker_factory
        self._source_config_factory = source_config_factory or self._default_source_config
        self._utc_ns = utc_ns
        self._request: TargetDroneRuntimeRequest | None = None
        self._receiver: TargetDroneReceiver | None = None
        self._worker: TargetDroneWorker | None = None
        self._session_id = ""
        self._session_root: Path | None = None
        self._runtime_id = ""
        self._partial_spool_path: Path | None = None
        self._phase = TargetDroneControllerPhase.UNCONFIGURED
        self._artifact: TargetDroneSpoolArtifact | None = None

    def _publish_measurement_source(self, state: str, detail: str) -> None:
        publisher = getattr(self.panel, "measurement_source_transition", None)
        if callable(publisher):
            try:
                publisher("target", state, detail)
            except (RuntimeError, TypeError, ValueError):
                pass

    @staticmethod
    def _default_source_config(request: TargetDroneRuntimeRequest) -> MavlinkSourceConfig:
        return MavlinkSourceConfig.target_drone(
            str(request.endpoint),
            baudrate=request.baudrate,
            expected_system_id=request.expected_system_id,
            expected_component_id=request.expected_component_id,
        )

    def _current_session(self) -> tuple[str, Path]:
        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True)):
            resolver = getattr(self.panel, "_recording_session_dir", None)
            if not callable(resolver):
                raise TargetDroneControllerError("measurement data directory owner is unavailable")
            root = Path(resolver("target-drone")).expanduser().resolve()
            root.mkdir(parents=True, exist_ok=True)
            return "measurement-data", root
        session_id = str(getattr(self.panel, "recording_session_id", "") or "").strip()
        if not session_id:
            raise TargetDroneControllerError("recording session ID is not prepared")
        if any(character in session_id for character in ("/", "\\", "\x00")):
            raise TargetDroneControllerError("recording session ID contains a path separator")
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise TargetDroneControllerError("recording session directory owner is unavailable")
        root = Path(resolver(session_id)).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        return session_id, root

    def _assert_session_unchanged(self) -> None:
        if self._request is None:
            raise TargetDroneControllerError("target-drone stream is not configured")
        current_id, current_root = self._current_session()
        if current_id != self._session_id or current_root != self._session_root:
            raise TargetDroneControllerError(
                "recording session changed after target-drone configuration; stop and reconfigure"
            )

    def configure(self, request: TargetDroneRuntimeRequest) -> TargetDroneControllerSnapshot:
        if not isinstance(request, TargetDroneRuntimeRequest):
            raise TypeError("request must be TargetDroneRuntimeRequest")
        if self._worker is not None and self._worker.is_alive:
            raise TargetDroneControllerError("cannot reconfigure a running target-drone worker")
        if self._receiver is not None and self._receiver.state in (
            TargetDroneStreamState.CONNECTING,
            TargetDroneStreamState.STREAMING,
        ):
            raise TargetDroneControllerError("stop the target-drone stream before reconfiguring")
        session_id, session_root = self._current_session()
        runtime_id = f"target-{uuid.uuid4().hex}"
        spool_dir = session_root / ".spool" / "target_drone"
        partial_path = spool_dir / f"{runtime_id}.jsonl.part"
        sessionless = not bool(
            getattr(self.panel, "_measurement_session_hierarchy_enabled", True)
        )
        source_config = self._source_config_factory(request)
        _validate_recv_only_config(source_config, request)
        receiver = self._receiver_factory(
            source_config,
            mode=request.mode,
            spool_path=partial_path,
            heartbeat_timeout_s=request.heartbeat_timeout_s,
            spool_max_rows=request.spool_max_rows,
            spool_max_bytes=request.spool_max_bytes,
            spool_fsync_every_rows=request.spool_fsync_every_rows,
            spool=(
                _MeasurementDataTargetSpool(
                    self.panel,
                    session_root / "px4_drone.jsonl",
                )
                if sessionless
                else None
            ),
        )
        self._request = request
        self._receiver = receiver
        self._worker = None
        self._session_id = session_id
        self._session_root = session_root
        self._runtime_id = runtime_id
        self._partial_spool_path = partial_path
        self._artifact = None
        self._phase = TargetDroneControllerPhase.CONFIGURED
        return self.snapshot()

    def start(self) -> TargetDroneControllerSnapshot:
        self._assert_session_unchanged()
        if self._artifact is not None:
            raise TargetDroneControllerError("sealed target-drone runtime must be reconfigured before restart")
        assert self._request is not None and self._receiver is not None
        if self._request.mode == TargetDroneMode.UAM:
            state = self._receiver.start()
            if state != TargetDroneStreamState.DEFERRED:
                raise TargetDroneControllerError("UAM live target stream did not enter DEFERRED state")
            self._phase = TargetDroneControllerPhase.DEFERRED
            return self.snapshot()
        if self._worker is None:
            self._worker = self._worker_factory(self._receiver)
        self._phase = TargetDroneControllerPhase.STARTING
        self._publish_measurement_source(
            "WAITING_RESOURCE", "Target telemetry worker start requested"
        )
        self._worker.start()
        return self.snapshot()

    def drain(
        self,
        *,
        max_messages: int = 4096,
        max_duration_s: float = 0.250,
    ) -> int:
        self._assert_session_unchanged()
        assert self._request is not None and self._receiver is not None
        if self._request.mode == TargetDroneMode.UAM:
            return 0
        if self._artifact is not None:
            raise TargetDroneControllerError("sealed target-drone spool cannot be drained")
        accepted_before = self._receiver.snapshot().accepted_message_count
        drained = self._receiver.drain(
            max_messages=_positive_int(max_messages, "max_messages"),
            max_duration_s=_positive_float(max_duration_s, "max_duration_s"),
        )
        accepted_after = self._receiver.snapshot().accepted_message_count
        if accepted_after > accepted_before:
            self._publish_measurement_source(
                "LIVE", "verified Target telemetry message received"
            )
        return drained

    def stop(self, *, timeout_s: float | None = None) -> TargetDroneControllerSnapshot:
        self._assert_session_unchanged()
        assert self._request is not None and self._receiver is not None
        if self._artifact is not None:
            return self.snapshot()
        if self._request.mode == TargetDroneMode.UAM:
            self._receiver.stop(drain=False)
            self._phase = TargetDroneControllerPhase.DEFERRED
            self._publish_measurement_source("LEFT", "Target telemetry stopped")
            return self.snapshot()
        try:
            if self._worker is None:
                stream_snapshot = self._receiver.stop(drain=True)
            else:
                stream_snapshot = self._worker.stop(timeout_s=timeout_s)
        except Exception:
            self._phase = TargetDroneControllerPhase.FAILED
            self._publish_measurement_source(
                "UNAVAILABLE", "Target telemetry stop failed"
            )
            raise
        self._phase = (
            TargetDroneControllerPhase.FAILED
            if stream_snapshot.state in (
                TargetDroneStreamState.FAILED,
                TargetDroneStreamState.IDENTITY_MISMATCH,
            )
            else TargetDroneControllerPhase.STOPPED
        )
        self._publish_measurement_source(
            "UNAVAILABLE"
            if self._phase == TargetDroneControllerPhase.FAILED
            else "LEFT",
            "Target telemetry failed"
            if self._phase == TargetDroneControllerPhase.FAILED
            else "Target telemetry stopped",
        )
        if self._partial_spool_path is not None and self._partial_spool_path.is_file():
            self._artifact = self._seal_spool(stream_snapshot)
            if self._phase != TargetDroneControllerPhase.FAILED:
                self._phase = TargetDroneControllerPhase.SEALED
        return self.snapshot()

    def _seal_spool(self, stream: TargetDroneStreamSnapshot) -> TargetDroneSpoolArtifact:
        assert self._request is not None
        assert self._session_root is not None
        assert self._partial_spool_path is not None
        source = self._partial_spool_path
        if source.is_symlink() or not source.is_file():
            raise TargetDroneArtifactIntegrityError("target-drone spool is not a regular file")
        rows = _jsonl_row_count_and_validate(
            source,
            verified_source_system=stream.verified_source_system,
            verified_source_component=stream.verified_source_component,
        )
        if rows != stream.spool_row_count:
            raise TargetDroneArtifactIntegrityError(
                f"target-drone spool row mismatch: receiver={stream.spool_row_count}, disk={rows}"
            )
        initial_size = source.stat().st_size
        initial_sha256 = _sha256(source)
        final_dir = self._session_root / "raw" / "target_drone"
        final_dir.mkdir(parents=True, exist_ok=True)
        final_path = final_dir / f"{self._runtime_id}.{initial_sha256[:16]}.jsonl"
        if final_path.exists():
            raise TargetDroneArtifactIntegrityError("sealed target-drone artifact path already exists")
        os.replace(source, final_path)
        os.chmod(
            final_path,
            stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH,
        )
        directory_fd = os.open(final_dir, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        artifact = TargetDroneSpoolArtifact(
            artifact_id=f"target-drone-spool-{uuid.uuid4().hex}",
            absolute_path=str(final_path),
            relative_path=final_path.relative_to(self._session_root).as_posix(),
            session_id=self._session_id,
            runtime_id=self._runtime_id,
            mode=self._request.mode,
            endpoint=str(self._request.endpoint),
            expected_system_id=self._request.expected_system_id,
            expected_component_id=self._request.expected_component_id,
            verified_source_system=stream.verified_source_system,
            verified_source_component=stream.verified_source_component,
            row_count=rows,
            accepted_message_count=stream.accepted_message_count,
            size_bytes=initial_size,
            sha256=initial_sha256,
            stopped_utc_ns=int(self._utc_ns()),
        )
        _verify_sealed_artifact(artifact)
        return artifact

    def snapshot(self) -> TargetDroneControllerSnapshot:
        request = self._request
        receiver = self._receiver
        worker = self._worker
        stream = None if receiver is None else receiver.snapshot()
        worker_error = None if worker is None else worker.worker_error
        phase = self._phase
        if worker_error is not None:
            phase = TargetDroneControllerPhase.FAILED
        elif stream is not None and stream.state == TargetDroneStreamState.STREAMING:
            phase = TargetDroneControllerPhase.RUNNING
        elif stream is not None and stream.state == TargetDroneStreamState.DEFERRED:
            phase = TargetDroneControllerPhase.DEFERRED
        last_error = None if stream is None else stream.last_error
        if worker_error is not None:
            last_error = str(worker_error)
        return TargetDroneControllerSnapshot(
            phase=phase,
            configured=request is not None,
            session_id=self._session_id,
            runtime_id=self._runtime_id,
            mode=None if request is None else request.mode,
            endpoint="" if request is None else str(request.endpoint),
            stream_state=None if stream is None else stream.state,
            worker_alive=bool(worker and worker.is_alive),
            expected_system_id=None if request is None else request.expected_system_id,
            expected_component_id=None if request is None else request.expected_component_id,
            verified_source_system=None if stream is None else stream.verified_source_system,
            verified_source_component=None if stream is None else stream.verified_source_component,
            accepted_message_count=0 if stream is None else stream.accepted_message_count,
            ignored_message_count=0 if stream is None else stream.ignored_message_count,
            rejected_cross_source_count=0 if stream is None else stream.rejected_cross_source_count,
            rejected_invalid_provenance_count=(
                0 if stream is None else stream.rejected_invalid_provenance_count
            ),
            spool_row_count=0 if stream is None else stream.spool_row_count,
            spool_size_bytes=0 if stream is None else stream.spool_size_bytes,
            last_error=last_error,
            artifact=self._artifact,
        )

    # Mapping-oriented methods are intentionally thin so StandaloneBackend
    # can expose them without owning any MAVLink or filesystem state.
    def _public_snapshot_mapping(
        self,
        snapshot: TargetDroneControllerSnapshot,
    ) -> dict[str, Any]:
        result = snapshot.to_mapping()
        if not bool(
            getattr(self.panel, "_measurement_session_hierarchy_enabled", True)
        ):
            result.pop("session_id", None)
            result["data_owner"] = "measurement_data_root"
        return result

    def configure_target_drone_stream(self, values: Mapping[str, Any]) -> dict[str, Any]:
        return self._public_snapshot_mapping(
            self.configure(TargetDroneRuntimeRequest.from_mapping(values))
        )

    def connect_target_drone_stream(
        self,
        values: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if values is not None:
            self.configure(TargetDroneRuntimeRequest.from_mapping(values))
        return self._public_snapshot_mapping(self.start())

    def start_target_drone_stream(self) -> dict[str, Any]:
        return self._public_snapshot_mapping(self.start())

    def drain_target_drone_stream(
        self,
        *,
        max_messages: int = 4096,
        max_duration_s: float = 0.250,
    ) -> int:
        return self.drain(max_messages=max_messages, max_duration_s=max_duration_s)

    def stop_target_drone_stream(self, *, timeout_s: float | None = None) -> dict[str, Any]:
        return self._public_snapshot_mapping(self.stop(timeout_s=timeout_s))

    def target_drone_stream_snapshot(self) -> dict[str, Any]:
        return self._public_snapshot_mapping(self.snapshot())

    def target_drone_spool_artifact(self) -> dict[str, Any] | None:
        return None if self._artifact is None else self._artifact.to_mapping()


__all__ = [
    "TARGET_DRONE_ARTIFACT_SCHEMA",
    "TARGET_DRONE_CONTROLLER_METHODS",
    "TARGET_DRONE_COORDINATE_CONTRACT",
    "TargetDroneArtifactIntegrityError",
    "TargetDroneController",
    "TargetDroneControllerError",
    "TargetDroneControllerPhase",
    "TargetDroneControllerSnapshot",
    "TargetDroneRuntimeRequest",
    "TargetDroneSpoolArtifact",
]
