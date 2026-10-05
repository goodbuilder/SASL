"""Immutable MAVLink source, heading, logger, and ULog contracts.

Coordinate and source contract
------------------------------
``left`` and ``right`` are the two rig Pixhawks (P1/P2); ``target_drone``
is a reference aircraft and is never a third stereo camera.  A course over
ground is the horizontal velocity direction clockwise from north in the NED
world frame.  GNSS yaw is the GNSS-reported earth-frame direction from north.
ATTITUDE yaw is the MAVLink/PX4 body-FRD to world-NED Euler yaw.  These three
observations are intentionally different types and must not be substituted for
one another.  This module performs no conversion to OpenCV optical RDF,
left/right-camera, or stereo-centre frames, and none of its values are
calibration-time ground truth.

Every observation can retain the producing source and MAVLink packet identity.
All models and transition helpers are frozen and pure: network, clock, MAVFTP,
logger-command, and filesystem I/O remain responsibilities of their callers.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import hashlib
import math
import re
import struct
from typing import Iterable


GPS_COG_MESSAGE_IDS = frozenset((24, 113, 124))
GNSS_YAW_MESSAGE_IDS = frozenset((24, 113, 124, 232))
ATTITUDE_YAW_MESSAGE_IDS = frozenset((30,))


def _strict_bool(value: bool, name: str) -> bool:
    if type(value) is not bool:
        raise TypeError(f"{name} must be bool")
    return value


def _strict_int(value: int, name: str, *, minimum: int = 0, maximum: int | None = None) -> int:
    if type(value) is not int:
        raise TypeError(f"{name} must be int")
    if value < minimum or (maximum is not None and value > maximum):
        contract = f"between {minimum} and {maximum}" if maximum is not None else f">= {minimum}"
        raise ValueError(f"{name} must be {contract}")
    return value


def _nonempty_text(value: str, name: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be str")
    parsed = value.strip()
    if not parsed:
        raise ValueError(f"{name} must not be empty")
    return parsed


class MavlinkSourceId(str, Enum):
    LEFT = "left"
    RIGHT = "right"
    TARGET_DRONE = "target_drone"


class MavlinkSourceRole(str, Enum):
    RIG_P1 = "RIG_P1"
    RIG_P2 = "RIG_P2"
    TARGET_REFERENCE = "TARGET_REFERENCE"


class MavlinkFtpAccess(str, Enum):
    DISABLED = "DISABLED"
    READ_ONLY = "READ_ONLY"
    READ_WRITE = "READ_WRITE"


class MavlinkOperation(str, Enum):
    STREAM_SETUP = "STREAM_SETUP"
    PARAMETER_WRITE = "PARAMETER_WRITE"
    RTCM_INJECTION = "RTCM_INJECTION"
    LOGGER_CONTROL = "LOGGER_CONTROL"
    FTP_READ = "FTP_READ"
    FTP_WRITE = "FTP_WRITE"


@dataclass(frozen=True)
class MavlinkMutationPolicy:
    """Role-level command permissions; FTP read access is explicit and separate."""

    allow_stream_setup: bool
    allow_parameter_write: bool
    allow_rtcm_injection: bool
    allow_logger_control: bool
    ftp_access: MavlinkFtpAccess

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "allow_stream_setup",
            _strict_bool(self.allow_stream_setup, "allow_stream_setup"),
        )
        object.__setattr__(
            self,
            "allow_parameter_write",
            _strict_bool(self.allow_parameter_write, "allow_parameter_write"),
        )
        object.__setattr__(
            self,
            "allow_rtcm_injection",
            _strict_bool(self.allow_rtcm_injection, "allow_rtcm_injection"),
        )
        object.__setattr__(
            self,
            "allow_logger_control",
            _strict_bool(self.allow_logger_control, "allow_logger_control"),
        )
        object.__setattr__(self, "ftp_access", MavlinkFtpAccess(self.ftp_access))

    @classmethod
    def rig_default(cls) -> "MavlinkMutationPolicy":
        return cls(
            allow_stream_setup=True,
            allow_parameter_write=True,
            allow_rtcm_injection=True,
            allow_logger_control=True,
            ftp_access=MavlinkFtpAccess.READ_WRITE,
        )

    @classmethod
    def target_read_only(cls) -> "MavlinkMutationPolicy":
        return cls(
            allow_stream_setup=False,
            allow_parameter_write=False,
            allow_rtcm_injection=False,
            allow_logger_control=False,
            ftp_access=MavlinkFtpAccess.READ_ONLY,
        )

    @property
    def ftp_is_read_only(self) -> bool:
        return self.ftp_access == MavlinkFtpAccess.READ_ONLY

    def allows(self, operation: MavlinkOperation | str) -> bool:
        requested = MavlinkOperation(operation)
        return {
            MavlinkOperation.STREAM_SETUP: self.allow_stream_setup,
            MavlinkOperation.PARAMETER_WRITE: self.allow_parameter_write,
            MavlinkOperation.RTCM_INJECTION: self.allow_rtcm_injection,
            MavlinkOperation.LOGGER_CONTROL: self.allow_logger_control,
            MavlinkOperation.FTP_READ: self.ftp_access in (
                MavlinkFtpAccess.READ_ONLY,
                MavlinkFtpAccess.READ_WRITE,
            ),
            MavlinkOperation.FTP_WRITE: self.ftp_access == MavlinkFtpAccess.READ_WRITE,
        }[requested]


def _role_for_source(source_id: MavlinkSourceId) -> MavlinkSourceRole:
    return {
        MavlinkSourceId.LEFT: MavlinkSourceRole.RIG_P1,
        MavlinkSourceId.RIGHT: MavlinkSourceRole.RIG_P2,
        MavlinkSourceId.TARGET_DRONE: MavlinkSourceRole.TARGET_REFERENCE,
    }[source_id]


@dataclass(frozen=True)
class MavlinkSourceConfig:
    """A validated source/role pair and the commands Workstation may issue."""

    source_id: MavlinkSourceId
    role: MavlinkSourceRole
    endpoint: str
    baudrate: int
    expected_system_id: int | None
    expected_component_id: int | None
    policy: MavlinkMutationPolicy

    def __post_init__(self) -> None:
        source_id = MavlinkSourceId(self.source_id)
        role = MavlinkSourceRole(self.role)
        endpoint = _nonempty_text(self.endpoint, "endpoint")
        baudrate = _strict_int(self.baudrate, "baudrate", minimum=1)
        if role != _role_for_source(source_id):
            raise ValueError(f"{source_id.value} cannot use role {role.value}")
        _validate_mavlink_id(self.expected_system_id, "expected_system_id", minimum=1)
        _validate_mavlink_id(self.expected_component_id, "expected_component_id", minimum=1)
        if not isinstance(self.policy, MavlinkMutationPolicy):
            raise TypeError("policy must be a MavlinkMutationPolicy")
        if source_id == MavlinkSourceId.TARGET_DRONE and (
            self.policy.allow_stream_setup
            or self.policy.allow_parameter_write
            or self.policy.allow_rtcm_injection
            or self.policy.allow_logger_control
            or self.policy.ftp_access == MavlinkFtpAccess.READ_WRITE
        ):
            raise ValueError("target_drone must use a read-only target mutation policy")
        object.__setattr__(self, "source_id", source_id)
        object.__setattr__(self, "role", role)
        object.__setattr__(self, "endpoint", endpoint)
        object.__setattr__(self, "baudrate", baudrate)
        if self.expected_system_id is not None:
            object.__setattr__(self, "expected_system_id", int(self.expected_system_id))
        if self.expected_component_id is not None:
            object.__setattr__(self, "expected_component_id", int(self.expected_component_id))

    @classmethod
    def for_source(
        cls,
        source_id: MavlinkSourceId | str,
        endpoint: str,
        *,
        baudrate: int = 57_600,
        expected_system_id: int | None = None,
        expected_component_id: int | None = None,
        policy: MavlinkMutationPolicy | None = None,
    ) -> "MavlinkSourceConfig":
        source = MavlinkSourceId(source_id)
        default_policy = (
            MavlinkMutationPolicy.target_read_only()
            if source == MavlinkSourceId.TARGET_DRONE
            else MavlinkMutationPolicy.rig_default()
        )
        return cls(
            source_id=source,
            role=_role_for_source(source),
            endpoint=endpoint,
            baudrate=baudrate,
            expected_system_id=expected_system_id,
            expected_component_id=expected_component_id,
            policy=default_policy if policy is None else policy,
        )

    @classmethod
    def left(cls, endpoint: str, **kwargs: object) -> "MavlinkSourceConfig":
        return cls.for_source(MavlinkSourceId.LEFT, endpoint, **kwargs)

    @classmethod
    def right(cls, endpoint: str, **kwargs: object) -> "MavlinkSourceConfig":
        return cls.for_source(MavlinkSourceId.RIGHT, endpoint, **kwargs)

    @classmethod
    def target_drone(cls, endpoint: str, **kwargs: object) -> "MavlinkSourceConfig":
        return cls.for_source(MavlinkSourceId.TARGET_DRONE, endpoint, **kwargs)


def _validate_mavlink_id(value: int | None, name: str, *, minimum: int) -> None:
    if value is None:
        return
    _strict_int(value, name, minimum=minimum, maximum=255)


@dataclass(frozen=True)
class MavlinkPacketProvenance:
    """Packet identity without interpreting its coordinate payload."""

    source_id: MavlinkSourceId
    source_system: int
    source_component: int
    sequence: int
    message_id: int
    host_received_monotonic_ns: int
    host_received_utc_ns: int | None = None
    source_timestamp_us: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "source_id", MavlinkSourceId(self.source_id))
        _validate_mavlink_id(self.source_system, "source_system", minimum=1)
        _validate_mavlink_id(self.source_component, "source_component", minimum=1)
        _validate_mavlink_id(self.sequence, "sequence", minimum=0)
        object.__setattr__(self, "source_system", int(self.source_system))
        object.__setattr__(self, "source_component", int(self.source_component))
        object.__setattr__(self, "sequence", int(self.sequence))
        object.__setattr__(
            self,
            "message_id",
            _strict_int(self.message_id, "message_id", maximum=16_777_215),
        )
        for name in ("host_received_monotonic_ns", "host_received_utc_ns", "source_timestamp_us"):
            value = getattr(self, name)
            if value is not None:
                parsed = _strict_int(value, name)
                object.__setattr__(self, name, parsed)


def _finite_angle(value: float, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


@dataclass(frozen=True)
class CourseOverGround:
    """Velocity direction in the NED horizontal plane, clockwise from north."""

    degrees_true_north: float
    provenance: MavlinkPacketProvenance

    def __post_init__(self) -> None:
        if not isinstance(self.provenance, MavlinkPacketProvenance):
            raise TypeError("provenance must be MavlinkPacketProvenance")
        if self.provenance.message_id not in GPS_COG_MESSAGE_IDS:
            raise ValueError("course over ground provenance is not a GPS COG message")
        angle = _finite_angle(self.degrees_true_north, "degrees_true_north")
        if not 0.0 <= angle < 360.0:
            raise ValueError("course over ground must be in [0, 360) degrees")
        object.__setattr__(self, "degrees_true_north", angle)

    @classmethod
    def from_mavlink_cdeg(
        cls,
        value_cdeg: int | None,
        provenance: MavlinkPacketProvenance,
    ) -> "CourseOverGround | None":
        if value_cdeg is None:
            return None
        value = _strict_int(value_cdeg, "value_cdeg", maximum=65_535)
        if value == 65_535:
            return None
        if not 0 <= value < 36_000:
            raise ValueError("MAVLink COG must be 0..35999 cdeg or UINT16_MAX")
        return cls(value / 100.0, provenance)


@dataclass(frozen=True)
class GnssYaw:
    """GNSS-reported earth-frame yaw clockwise from north; not COG."""

    degrees_true_north: float
    provenance: MavlinkPacketProvenance

    def __post_init__(self) -> None:
        if not isinstance(self.provenance, MavlinkPacketProvenance):
            raise TypeError("provenance must be MavlinkPacketProvenance")
        if self.provenance.message_id not in GNSS_YAW_MESSAGE_IDS:
            raise ValueError("GNSS yaw provenance is not a GNSS-yaw message")
        angle = _finite_angle(self.degrees_true_north, "degrees_true_north")
        if not 0.0 <= angle < 360.0:
            raise ValueError("GNSS yaw must be in [0, 360) degrees")
        object.__setattr__(self, "degrees_true_north", angle)

    @classmethod
    def from_mavlink_cdeg(
        cls,
        value_cdeg: int | None,
        provenance: MavlinkPacketProvenance,
    ) -> "GnssYaw | None":
        if value_cdeg is None:
            return None
        value = _strict_int(value_cdeg, "value_cdeg", maximum=65_535)
        if value in (0, 65_535):
            return None
        if not 1 <= value <= 36_000:
            raise ValueError(
                "MAVLink GNSS yaw must be 1..36000 cdeg, zero when unsupported, "
                "or UINT16_MAX when temporarily unavailable"
            )
        return cls((value % 36_000) / 100.0, provenance)


@dataclass(frozen=True)
class AttitudeYaw:
    """Body-FRD to world-NED Euler yaw in the canonical [-180, 180) range."""

    degrees_ned: float
    provenance: MavlinkPacketProvenance

    def __post_init__(self) -> None:
        if not isinstance(self.provenance, MavlinkPacketProvenance):
            raise TypeError("provenance must be MavlinkPacketProvenance")
        if self.provenance.message_id not in ATTITUDE_YAW_MESSAGE_IDS:
            raise ValueError("ATTITUDE yaw provenance is not an ATTITUDE message")
        angle = _finite_angle(self.degrees_ned, "degrees_ned")
        if not -180.0 <= angle < 180.0:
            raise ValueError("ATTITUDE yaw must be in [-180, 180) degrees")
        object.__setattr__(self, "degrees_ned", angle)

    @classmethod
    def from_mavlink_rad(
        cls,
        value_rad: float | None,
        provenance: MavlinkPacketProvenance,
    ) -> "AttitudeYaw | None":
        if value_rad is None:
            return None
        radians = _finite_angle(value_rad, "value_rad")
        if radians < -math.pi - 1e-6 or radians > math.pi + 1e-6:
            raise ValueError("MAVLink ATTITUDE yaw must be in [-pi, pi] radians")
        radians = min(math.pi, max(-math.pi, radians))
        degrees = math.degrees(radians)
        normalized = (degrees + 180.0) % 360.0 - 180.0
        return cls(normalized, provenance)


@dataclass(frozen=True)
class MavlinkYawObservations:
    """Named, non-interchangeable yaw-like observations from one source."""

    course_over_ground: CourseOverGround | None = None
    gnss_yaw: GnssYaw | None = None
    attitude_yaw: AttitudeYaw | None = None

    def __post_init__(self) -> None:
        for name, expected_type in (
            ("course_over_ground", CourseOverGround),
            ("gnss_yaw", GnssYaw),
            ("attitude_yaw", AttitudeYaw),
        ):
            observation = getattr(self, name)
            if observation is not None and not isinstance(observation, expected_type):
                raise TypeError(f"{name} must be {expected_type.__name__}")
        source_identities = {
            (
                observation.provenance.source_id,
                observation.provenance.source_system,
                observation.provenance.source_component,
            )
            for observation in (
                self.course_over_ground,
                self.gnss_yaw,
                self.attitude_yaw,
            )
            if observation is not None
        }
        if len(source_identities) > 1:
            raise ValueError("yaw observations from different MAVLink identities cannot be merged")

    @property
    def source_id(self) -> MavlinkSourceId | None:
        for observation in (self.course_over_ground, self.gnss_yaw, self.attitude_yaw):
            if observation is not None:
                return observation.provenance.source_id
        return None


class LoggerStatusKind(str, Enum):
    FILE_RUNNING = "FILE_RUNNING"
    NOT_LOGGING = "NOT_LOGGING"
    MAVLINK_ONLY = "MAVLINK_ONLY"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class LoggerStatusObservation:
    kind: LoggerStatusKind
    raw_text: str
    remote_log_path: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.raw_text, str):
            raise TypeError("raw_text must be str")
        kind = LoggerStatusKind(self.kind)
        path = (
            None
            if self.remote_log_path is None
            else _nonempty_text(self.remote_log_path, "remote_log_path")
        )
        if path is not None and not path.startswith("/"):
            raise ValueError("remote_log_path must be an absolute PX4 path")
        if kind == LoggerStatusKind.FILE_RUNNING and path is None:
            raise ValueError("FILE_RUNNING status requires a remote log path")
        if kind != LoggerStatusKind.FILE_RUNNING and path is not None:
            raise ValueError(f"{kind.value} status cannot carry a file path")
        object.__setattr__(self, "kind", kind)
        object.__setattr__(self, "raw_text", self.raw_text)
        object.__setattr__(self, "remote_log_path", path)


_LOG_PATH_PATTERN = re.compile(r"Log file:\s*(/\S+)", re.IGNORECASE)


def parse_logger_status(raw_text: str) -> LoggerStatusObservation:
    """Classify PX4 ``logger status`` output while retaining its raw evidence."""

    if not isinstance(raw_text, str):
        raise TypeError("raw_text must be str")
    raw = raw_text
    lowered = raw.lower()
    paths = tuple(match.rstrip(".,") for match in _LOG_PATH_PATTERN.findall(raw))
    path = paths[0] if len(set(paths)) == 1 and paths else None
    not_logging = "not logging" in lowered
    file_running = "full file logging running" in lowered
    mavlink_running = (
        "mavlink" in lowered
        and "running" in lowered
        and not not_logging
        and not file_running
    )
    if (
        len(set(paths)) > 1
        or (bool(paths) and not file_running)
        or (not_logging and (file_running or mavlink_running))
    ):
        kind = LoggerStatusKind.UNKNOWN
        path = None
    elif not_logging:
        kind = LoggerStatusKind.NOT_LOGGING
    elif file_running and path is not None:
        kind = LoggerStatusKind.FILE_RUNNING
    elif mavlink_running:
        kind = LoggerStatusKind.MAVLINK_ONLY
    else:
        kind = LoggerStatusKind.UNKNOWN
    return LoggerStatusObservation(kind=kind, raw_text=raw, remote_log_path=path)


@dataclass(frozen=True)
class LoggerRuntimeIdentity:
    """Identity boundary across which a logger ownership token is invalid."""

    source_id: MavlinkSourceId
    connection_generation: int
    boot_id: str
    source_system: int
    source_component: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "source_id", MavlinkSourceId(self.source_id))
        generation = _strict_int(
            self.connection_generation,
            "connection_generation",
        )
        boot_id = _nonempty_text(self.boot_id, "boot_id")
        _validate_mavlink_id(self.source_system, "source_system", minimum=1)
        _validate_mavlink_id(self.source_component, "source_component", minimum=1)
        object.__setattr__(self, "connection_generation", generation)
        object.__setattr__(self, "boot_id", boot_id)
        object.__setattr__(self, "source_system", int(self.source_system))
        object.__setattr__(self, "source_component", int(self.source_component))


@dataclass(frozen=True)
class LoggerStatusEvidence:
    """A logger status observation bound to one boot/link and receive time."""

    runtime_identity: LoggerRuntimeIdentity
    status: LoggerStatusObservation
    observed_at_monotonic_ns: int
    query_id: str

    def __post_init__(self) -> None:
        if not isinstance(self.runtime_identity, LoggerRuntimeIdentity):
            raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
        if not isinstance(self.status, LoggerStatusObservation):
            raise TypeError("status must be LoggerStatusObservation")
        object.__setattr__(
            self,
            "observed_at_monotonic_ns",
            _strict_int(self.observed_at_monotonic_ns, "observed_at_monotonic_ns"),
        )
        object.__setattr__(self, "query_id", _nonempty_text(self.query_id, "query_id"))


def _runtime_matches_config(
    config: MavlinkSourceConfig,
    runtime_identity: LoggerRuntimeIdentity,
) -> bool:
    """Require a role label and explicitly configured source packet identity."""

    return (
        config.source_id == runtime_identity.source_id
        and config.expected_system_id is not None
        and config.expected_component_id is not None
        and config.expected_system_id == runtime_identity.source_system
        and config.expected_component_id == runtime_identity.source_component
    )


@dataclass(frozen=True)
class LoggerOwnershipToken:
    runtime_identity: LoggerRuntimeIdentity
    session_id: str
    initial_remote_log_path: str
    start_command_id: str
    verified_at_monotonic_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.runtime_identity, LoggerRuntimeIdentity):
            raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
        session_id = _nonempty_text(self.session_id, "session_id")
        path = _nonempty_text(self.initial_remote_log_path, "initial_remote_log_path")
        command_id = _nonempty_text(self.start_command_id, "start_command_id")
        verified_at = _strict_int(self.verified_at_monotonic_ns, "verified_at_monotonic_ns")
        if not path.startswith("/"):
            raise ValueError("initial_remote_log_path must be an absolute PX4 path")
        object.__setattr__(self, "session_id", session_id)
        object.__setattr__(self, "initial_remote_log_path", path)
        object.__setattr__(self, "start_command_id", command_id)
        object.__setattr__(self, "verified_at_monotonic_ns", verified_at)


class LoggerOwner(str, Enum):
    NONE = "NONE"
    WORKSTATION = "WORKSTATION"
    EXTERNAL_ALREADY_RUNNING = "EXTERNAL_ALREADY_RUNNING"
    UNKNOWN = "UNKNOWN"


class LoggerLifecyclePhase(str, Enum):
    PREFLIGHT_IDLE = "PREFLIGHT_IDLE"
    EXTERNAL_ACTIVE = "EXTERNAL_ACTIVE"
    PREFLIGHT_UNKNOWN = "PREFLIGHT_UNKNOWN"
    WORKSTATION_ACTIVE = "WORKSTATION_ACTIVE"
    START_FAILED = "START_FAILED"
    START_UNVERIFIED = "START_UNVERIFIED"
    STOPPED = "STOPPED"
    PENDING_ACTIVE = "PENDING_ACTIVE"
    STOP_UNVERIFIED = "STOP_UNVERIFIED"


@dataclass(frozen=True)
class LoggerLifecycleState:
    runtime_identity: LoggerRuntimeIdentity
    preflight_status: LoggerStatusObservation
    current_status: LoggerStatusObservation
    owner: LoggerOwner
    phase: LoggerLifecyclePhase
    ownership_token: LoggerOwnershipToken | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.runtime_identity, LoggerRuntimeIdentity):
            raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
        if not isinstance(self.preflight_status, LoggerStatusObservation):
            raise TypeError("preflight_status must be LoggerStatusObservation")
        if not isinstance(self.current_status, LoggerStatusObservation):
            raise TypeError("current_status must be LoggerStatusObservation")
        owner = LoggerOwner(self.owner)
        phase = LoggerLifecyclePhase(self.phase)
        token = self.ownership_token
        if owner == LoggerOwner.WORKSTATION:
            if token is None:
                raise ValueError("WORKSTATION logger ownership requires a token")
            if token.runtime_identity != self.runtime_identity:
                raise ValueError("logger ownership token runtime identity mismatch")
            if phase != LoggerLifecyclePhase.WORKSTATION_ACTIVE:
                raise ValueError("WORKSTATION ownership requires WORKSTATION_ACTIVE phase")
            if self.current_status.kind != LoggerStatusKind.FILE_RUNNING:
                raise ValueError("WORKSTATION ownership requires a running file logger")
            if self.current_status.remote_log_path != token.initial_remote_log_path:
                raise ValueError("WORKSTATION logger path must match its ownership token")
        elif token is not None:
            raise ValueError("only WORKSTATION ownership may retain an ownership token")
        expected_status_by_phase = {
            LoggerLifecyclePhase.PREFLIGHT_IDLE: LoggerStatusKind.NOT_LOGGING,
            LoggerLifecyclePhase.EXTERNAL_ACTIVE: (
                LoggerStatusKind.FILE_RUNNING,
                LoggerStatusKind.MAVLINK_ONLY,
            ),
            LoggerLifecyclePhase.PREFLIGHT_UNKNOWN: LoggerStatusKind.UNKNOWN,
            LoggerLifecyclePhase.WORKSTATION_ACTIVE: LoggerStatusKind.FILE_RUNNING,
            LoggerLifecyclePhase.START_FAILED: LoggerStatusKind.NOT_LOGGING,
            LoggerLifecyclePhase.START_UNVERIFIED: (
                LoggerStatusKind.UNKNOWN,
                LoggerStatusKind.MAVLINK_ONLY,
                LoggerStatusKind.FILE_RUNNING,
            ),
            LoggerLifecyclePhase.STOPPED: LoggerStatusKind.NOT_LOGGING,
            LoggerLifecyclePhase.PENDING_ACTIVE: (
                LoggerStatusKind.FILE_RUNNING,
                LoggerStatusKind.MAVLINK_ONLY,
            ),
            LoggerLifecyclePhase.STOP_UNVERIFIED: LoggerStatusKind.UNKNOWN,
        }
        expected = expected_status_by_phase[phase]
        allowed_statuses = expected if isinstance(expected, tuple) else (expected,)
        if self.current_status.kind not in allowed_statuses:
            raise ValueError(
                f"{phase.value} is inconsistent with {self.current_status.kind.value}"
            )
        expected_owner_by_phase = {
            LoggerLifecyclePhase.PREFLIGHT_IDLE: LoggerOwner.NONE,
            LoggerLifecyclePhase.EXTERNAL_ACTIVE: LoggerOwner.EXTERNAL_ALREADY_RUNNING,
            LoggerLifecyclePhase.PREFLIGHT_UNKNOWN: LoggerOwner.UNKNOWN,
            LoggerLifecyclePhase.WORKSTATION_ACTIVE: LoggerOwner.WORKSTATION,
            LoggerLifecyclePhase.START_FAILED: LoggerOwner.NONE,
            LoggerLifecyclePhase.START_UNVERIFIED: LoggerOwner.UNKNOWN,
            LoggerLifecyclePhase.STOPPED: LoggerOwner.NONE,
            LoggerLifecyclePhase.PENDING_ACTIVE: LoggerOwner.EXTERNAL_ALREADY_RUNNING,
            LoggerLifecyclePhase.STOP_UNVERIFIED: LoggerOwner.UNKNOWN,
        }
        if owner != expected_owner_by_phase[phase]:
            raise ValueError(f"{phase.value} is inconsistent with owner {owner.value}")
        expected_preflight_by_phase = {
            LoggerLifecyclePhase.PREFLIGHT_IDLE: (LoggerStatusKind.NOT_LOGGING,),
            LoggerLifecyclePhase.EXTERNAL_ACTIVE: (
                LoggerStatusKind.FILE_RUNNING,
                LoggerStatusKind.MAVLINK_ONLY,
            ),
            LoggerLifecyclePhase.PREFLIGHT_UNKNOWN: (LoggerStatusKind.UNKNOWN,),
            LoggerLifecyclePhase.WORKSTATION_ACTIVE: (LoggerStatusKind.NOT_LOGGING,),
            LoggerLifecyclePhase.START_FAILED: (LoggerStatusKind.NOT_LOGGING,),
            LoggerLifecyclePhase.START_UNVERIFIED: (LoggerStatusKind.NOT_LOGGING,),
            LoggerLifecyclePhase.STOPPED: (LoggerStatusKind.NOT_LOGGING,),
            LoggerLifecyclePhase.PENDING_ACTIVE: (LoggerStatusKind.NOT_LOGGING,),
            LoggerLifecyclePhase.STOP_UNVERIFIED: (LoggerStatusKind.NOT_LOGGING,),
        }
        if self.preflight_status.kind not in expected_preflight_by_phase[phase]:
            raise ValueError(
                f"{phase.value} is inconsistent with preflight "
                f"{self.preflight_status.kind.value}"
            )
        object.__setattr__(self, "owner", owner)
        object.__setattr__(self, "phase", phase)


def observe_logger_preflight(
    runtime_identity: LoggerRuntimeIdentity,
    status: LoggerStatusObservation,
) -> LoggerLifecycleState:
    """Create safe initial state; any already-running logger is externally owned."""

    if not isinstance(runtime_identity, LoggerRuntimeIdentity):
        raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
    if not isinstance(status, LoggerStatusObservation):
        raise TypeError("status must be LoggerStatusObservation")
    if status.kind == LoggerStatusKind.NOT_LOGGING:
        owner = LoggerOwner.NONE
        phase = LoggerLifecyclePhase.PREFLIGHT_IDLE
    elif status.kind in (LoggerStatusKind.FILE_RUNNING, LoggerStatusKind.MAVLINK_ONLY):
        owner = LoggerOwner.EXTERNAL_ALREADY_RUNNING
        phase = LoggerLifecyclePhase.EXTERNAL_ACTIVE
    else:
        owner = LoggerOwner.UNKNOWN
        phase = LoggerLifecyclePhase.PREFLIGHT_UNKNOWN
    return LoggerLifecycleState(
        runtime_identity=runtime_identity,
        preflight_status=status,
        current_status=status,
        owner=owner,
        phase=phase,
    )


class LoggerControlAction(str, Enum):
    NONE = "NONE"
    START = "START"
    STOP = "STOP"


class LoggerDecisionReason(str, Enum):
    PREFLIGHT_NOT_LOGGING = "PREFLIGHT_NOT_LOGGING"
    WORKSTATION_OWNED_SAME_RUNTIME = "WORKSTATION_OWNED_SAME_RUNTIME"
    LOGGER_CONTROL_DISABLED = "LOGGER_CONTROL_DISABLED"
    SOURCE_MISMATCH = "SOURCE_MISMATCH"
    ALREADY_RUNNING_EXTERNALLY = "ALREADY_RUNNING_EXTERNALLY"
    STATUS_NOT_SAFE = "STATUS_NOT_SAFE"
    NOT_WORKSTATION_OWNED = "NOT_WORKSTATION_OWNED"
    RUNTIME_IDENTITY_MISMATCH = "RUNTIME_IDENTITY_MISMATCH"
    SESSION_MISMATCH = "SESSION_MISMATCH"
    REMOTE_LOG_PATH_MISMATCH = "REMOTE_LOG_PATH_MISMATCH"
    FRESH_STATUS_REQUIRED = "FRESH_STATUS_REQUIRED"
    EXPECTED_IDENTITY_REQUIRED = "EXPECTED_IDENTITY_REQUIRED"


@dataclass(frozen=True)
class LoggerControlDecision:
    action: LoggerControlAction
    reason: LoggerDecisionReason
    runtime_identity: LoggerRuntimeIdentity | None = None
    session_id: str | None = None
    observed_remote_log_path: str | None = None
    command_id: str | None = None
    status_observed_at_monotonic_ns: int | None = None
    status_query_id: str | None = None

    def __post_init__(self) -> None:
        action = LoggerControlAction(self.action)
        reason = LoggerDecisionReason(self.reason)
        if self.runtime_identity is not None and not isinstance(
            self.runtime_identity,
            LoggerRuntimeIdentity,
        ):
            raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
        if action in (LoggerControlAction.START, LoggerControlAction.STOP):
            if self.runtime_identity is None:
                raise ValueError("logger command decisions require runtime identity")
            session_id = _nonempty_text(self.session_id, "session_id")
            command_id = _nonempty_text(self.command_id, "command_id")
        else:
            if any(
                value is not None
                for value in (
                    self.runtime_identity,
                    self.session_id,
                    self.command_id,
                    self.observed_remote_log_path,
                    self.status_observed_at_monotonic_ns,
                    self.status_query_id,
                )
            ):
                raise ValueError("NONE decisions cannot carry command bindings")
            session_id = None
            command_id = None
        path = (
            None
            if self.observed_remote_log_path is None
            else _nonempty_text(self.observed_remote_log_path, "observed_remote_log_path")
        )
        if path is not None and not path.startswith("/"):
            raise ValueError("observed_remote_log_path must be an absolute PX4 path")
        status_time = self.status_observed_at_monotonic_ns
        if status_time is not None:
            status_time = _strict_int(status_time, "status_observed_at_monotonic_ns")
        status_query_id = (
            None
            if self.status_query_id is None
            else _nonempty_text(self.status_query_id, "status_query_id")
        )
        if action == LoggerControlAction.STOP and (
            path is None or status_time is None or status_query_id is None
        ):
            raise ValueError("STOP decisions require fresh logger status path and time")
        if action == LoggerControlAction.START and any(
            value is not None for value in (path, status_time, status_query_id)
        ):
            raise ValueError("START decisions cannot carry post-command status evidence")
        object.__setattr__(self, "action", action)
        object.__setattr__(self, "reason", reason)
        object.__setattr__(self, "session_id", session_id)
        object.__setattr__(self, "command_id", command_id)
        object.__setattr__(self, "observed_remote_log_path", path)
        object.__setattr__(self, "status_observed_at_monotonic_ns", status_time)
        object.__setattr__(self, "status_query_id", status_query_id)


@dataclass(frozen=True)
class LoggerCommandReceipt:
    """Correlated result of one logger command, supplied by the I/O layer.

    ``acknowledged`` must mean that the command executor positively correlated
    PX4's acceptance with this command ID; transport completion alone is not an
    acknowledgement.  Without this evidence a running logger is never claimed.
    """

    action: LoggerControlAction
    runtime_identity: LoggerRuntimeIdentity
    session_id: str
    command_id: str
    acknowledged: bool
    completed_at_monotonic_ns: int

    def __post_init__(self) -> None:
        action = LoggerControlAction(self.action)
        if action == LoggerControlAction.NONE:
            raise ValueError("a receipt must identify START or STOP")
        if not isinstance(self.runtime_identity, LoggerRuntimeIdentity):
            raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
        object.__setattr__(self, "action", action)
        object.__setattr__(self, "session_id", _nonempty_text(self.session_id, "session_id"))
        object.__setattr__(self, "command_id", _nonempty_text(self.command_id, "command_id"))
        object.__setattr__(self, "acknowledged", _strict_bool(self.acknowledged, "acknowledged"))
        object.__setattr__(
            self,
            "completed_at_monotonic_ns",
            _strict_int(self.completed_at_monotonic_ns, "completed_at_monotonic_ns"),
        )


def _receipt_matches_decision(
    receipt: LoggerCommandReceipt,
    decision: LoggerControlDecision,
) -> bool:
    return (
        receipt.action == decision.action
        and receipt.runtime_identity == decision.runtime_identity
        and receipt.session_id == decision.session_id
        and receipt.command_id == decision.command_id
    )


def decide_logger_start(
    config: MavlinkSourceConfig,
    state: LoggerLifecycleState,
    *,
    session_id: str,
    command_id: str,
) -> LoggerControlDecision:
    if not isinstance(config, MavlinkSourceConfig):
        raise TypeError("config must be MavlinkSourceConfig")
    if not isinstance(state, LoggerLifecycleState):
        raise TypeError("state must be LoggerLifecycleState")
    session = _nonempty_text(session_id, "session_id")
    command = _nonempty_text(command_id, "command_id")
    if config.source_id != state.runtime_identity.source_id:
        return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.SOURCE_MISMATCH)
    if not config.policy.allow_logger_control:
        return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.LOGGER_CONTROL_DISABLED)
    if config.expected_system_id is None or config.expected_component_id is None:
        return LoggerControlDecision(
            LoggerControlAction.NONE,
            LoggerDecisionReason.EXPECTED_IDENTITY_REQUIRED,
        )
    if not _runtime_matches_config(config, state.runtime_identity):
        return LoggerControlDecision(
            LoggerControlAction.NONE,
            LoggerDecisionReason.RUNTIME_IDENTITY_MISMATCH,
        )
    if state.owner == LoggerOwner.EXTERNAL_ALREADY_RUNNING:
        return LoggerControlDecision(
            LoggerControlAction.NONE,
            LoggerDecisionReason.ALREADY_RUNNING_EXTERNALLY,
        )
    if state.owner == LoggerOwner.NONE and state.phase == LoggerLifecyclePhase.PREFLIGHT_IDLE:
        return LoggerControlDecision(
            LoggerControlAction.START,
            LoggerDecisionReason.PREFLIGHT_NOT_LOGGING,
            runtime_identity=state.runtime_identity,
            session_id=session,
            command_id=command,
        )
    return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.STATUS_NOT_SAFE)


def record_logger_start_result(
    state: LoggerLifecycleState,
    decision: LoggerControlDecision,
    observed_status: LoggerStatusEvidence,
    *,
    receipt: LoggerCommandReceipt,
) -> LoggerLifecycleState:
    """Claim only a correlated, acknowledged start with a later status query.

    The pure layer cannot prove that a shell string caused the transition.  Its
    caller must therefore issue a unique command ID and supply a positively
    correlated receipt.  If PX4 shell transport cannot provide that evidence,
    ``acknowledged`` must be false and Workstation deliberately never claims or
    later stops the logger; the resulting ULog remains externally owned.
    """

    if not isinstance(state, LoggerLifecycleState):
        raise TypeError("state must be LoggerLifecycleState")
    if not isinstance(decision, LoggerControlDecision):
        raise TypeError("decision must be LoggerControlDecision")
    if decision.action != LoggerControlAction.START:
        raise ValueError("a logger START decision is required")
    if decision.reason != LoggerDecisionReason.PREFLIGHT_NOT_LOGGING:
        raise ValueError("logger START decision does not describe an idle preflight")
    if decision.runtime_identity != state.runtime_identity:
        raise ValueError("logger START decision runtime identity mismatch")
    if not isinstance(receipt, LoggerCommandReceipt):
        raise TypeError("receipt must be LoggerCommandReceipt")
    if not _receipt_matches_decision(receipt, decision):
        raise ValueError("logger START receipt does not match its decision")
    if not isinstance(observed_status, LoggerStatusEvidence):
        raise TypeError("observed_status must be LoggerStatusEvidence")
    if observed_status.runtime_identity != state.runtime_identity:
        raise ValueError("post-start status runtime identity mismatch")
    if observed_status.observed_at_monotonic_ns < receipt.completed_at_monotonic_ns:
        raise ValueError("post-start status must be observed after command completion")
    if state.owner != LoggerOwner.NONE or state.phase != LoggerLifecyclePhase.PREFLIGHT_IDLE:
        raise ValueError("only a preflight-idle logger can be claimed")
    if (
        receipt.acknowledged
        and observed_status.status.kind == LoggerStatusKind.FILE_RUNNING
        and observed_status.status.remote_log_path
    ):
        token = LoggerOwnershipToken(
            runtime_identity=state.runtime_identity,
            session_id=decision.session_id or "",
            initial_remote_log_path=observed_status.status.remote_log_path,
            start_command_id=decision.command_id or "",
            verified_at_monotonic_ns=observed_status.observed_at_monotonic_ns,
        )
        return replace(
            state,
            current_status=observed_status.status,
            owner=LoggerOwner.WORKSTATION,
            phase=LoggerLifecyclePhase.WORKSTATION_ACTIVE,
            ownership_token=token,
        )
    owner = (
        LoggerOwner.NONE
        if observed_status.status.kind == LoggerStatusKind.NOT_LOGGING
        else LoggerOwner.UNKNOWN
    )
    phase = (
        LoggerLifecyclePhase.START_FAILED
        if observed_status.status.kind == LoggerStatusKind.NOT_LOGGING
        else LoggerLifecyclePhase.START_UNVERIFIED
    )
    return replace(
        state,
        current_status=observed_status.status,
        owner=owner,
        phase=phase,
        ownership_token=None,
    )


def decide_logger_stop(
    config: MavlinkSourceConfig,
    state: LoggerLifecycleState,
    *,
    current_runtime_identity: LoggerRuntimeIdentity,
    session_id: str,
    fresh_status: LoggerStatusEvidence | None,
    command_id: str,
) -> LoggerControlDecision:
    """Permit ``logger off`` only for this session's logger on the same boot/link."""

    if not isinstance(config, MavlinkSourceConfig):
        raise TypeError("config must be MavlinkSourceConfig")
    if not isinstance(state, LoggerLifecycleState):
        raise TypeError("state must be LoggerLifecycleState")
    if not isinstance(current_runtime_identity, LoggerRuntimeIdentity):
        raise TypeError("current_runtime_identity must be LoggerRuntimeIdentity")
    if fresh_status is not None and not isinstance(fresh_status, LoggerStatusEvidence):
        raise TypeError("fresh_status must be LoggerStatusEvidence")
    if config.source_id != state.runtime_identity.source_id:
        return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.SOURCE_MISMATCH)
    if not config.policy.allow_logger_control:
        return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.LOGGER_CONTROL_DISABLED)
    if config.expected_system_id is None or config.expected_component_id is None:
        return LoggerControlDecision(
            LoggerControlAction.NONE,
            LoggerDecisionReason.EXPECTED_IDENTITY_REQUIRED,
        )
    if not _runtime_matches_config(config, current_runtime_identity):
        return LoggerControlDecision(
            LoggerControlAction.NONE,
            LoggerDecisionReason.RUNTIME_IDENTITY_MISMATCH,
        )
    if state.owner != LoggerOwner.WORKSTATION or state.ownership_token is None:
        return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.NOT_WORKSTATION_OWNED)
    token = state.ownership_token
    if token.runtime_identity != current_runtime_identity:
        return LoggerControlDecision(
            LoggerControlAction.NONE,
            LoggerDecisionReason.RUNTIME_IDENTITY_MISMATCH,
        )
    if token.session_id != _nonempty_text(session_id, "session_id"):
        return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.SESSION_MISMATCH)
    if state.phase != LoggerLifecyclePhase.WORKSTATION_ACTIVE:
        return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.STATUS_NOT_SAFE)
    if (
        fresh_status is None
        or fresh_status.runtime_identity != current_runtime_identity
        or fresh_status.status.kind != LoggerStatusKind.FILE_RUNNING
        or fresh_status.observed_at_monotonic_ns <= token.verified_at_monotonic_ns
    ):
        return LoggerControlDecision(LoggerControlAction.NONE, LoggerDecisionReason.FRESH_STATUS_REQUIRED)
    if fresh_status.status.remote_log_path != token.initial_remote_log_path:
        return LoggerControlDecision(
            LoggerControlAction.NONE,
            LoggerDecisionReason.REMOTE_LOG_PATH_MISMATCH,
        )
    return LoggerControlDecision(
        LoggerControlAction.STOP,
        LoggerDecisionReason.WORKSTATION_OWNED_SAME_RUNTIME,
        runtime_identity=current_runtime_identity,
        session_id=token.session_id,
        observed_remote_log_path=fresh_status.status.remote_log_path,
        command_id=_nonempty_text(command_id, "command_id"),
        status_observed_at_monotonic_ns=fresh_status.observed_at_monotonic_ns,
        status_query_id=fresh_status.query_id,
    )


def record_logger_stop_result(
    state: LoggerLifecycleState,
    decision: LoggerControlDecision,
    observed_status: LoggerStatusEvidence,
    *,
    receipt: LoggerCommandReceipt,
) -> LoggerLifecycleState:
    """Consume ownership after one off attempt, including armed-still-active cases."""

    if not isinstance(state, LoggerLifecycleState):
        raise TypeError("state must be LoggerLifecycleState")
    if not isinstance(decision, LoggerControlDecision):
        raise TypeError("decision must be LoggerControlDecision")
    if decision.action != LoggerControlAction.STOP:
        raise ValueError("a logger STOP decision is required")
    if decision.reason != LoggerDecisionReason.WORKSTATION_OWNED_SAME_RUNTIME:
        raise ValueError("logger STOP decision lacks matching ownership evidence")
    if state.owner != LoggerOwner.WORKSTATION or state.ownership_token is None:
        raise ValueError("logger is not Workstation-owned")
    token = state.ownership_token
    if (
        decision.runtime_identity != token.runtime_identity
        or decision.session_id != token.session_id
        or decision.observed_remote_log_path != token.initial_remote_log_path
    ):
        raise ValueError("logger STOP decision does not match the ownership token")
    if not isinstance(receipt, LoggerCommandReceipt):
        raise TypeError("receipt must be LoggerCommandReceipt")
    if not _receipt_matches_decision(receipt, decision):
        raise ValueError("logger STOP receipt does not match its decision")
    if receipt.completed_at_monotonic_ns < (decision.status_observed_at_monotonic_ns or 0):
        raise ValueError("logger STOP receipt predates its fresh status evidence")
    if not isinstance(observed_status, LoggerStatusEvidence):
        raise TypeError("observed_status must be LoggerStatusEvidence")
    if observed_status.runtime_identity != decision.runtime_identity:
        raise ValueError("post-stop status runtime identity mismatch")
    if observed_status.observed_at_monotonic_ns < receipt.completed_at_monotonic_ns:
        raise ValueError("post-stop status must be observed after command completion")
    if not receipt.acknowledged:
        unknown_status = LoggerStatusObservation(
            kind=LoggerStatusKind.UNKNOWN,
            raw_text=observed_status.status.raw_text,
        )
        return replace(
            state,
            current_status=unknown_status,
            owner=LoggerOwner.UNKNOWN,
            phase=LoggerLifecyclePhase.STOP_UNVERIFIED,
            ownership_token=None,
        )
    if observed_status.status.kind == LoggerStatusKind.NOT_LOGGING:
        owner = LoggerOwner.NONE
        phase = LoggerLifecyclePhase.STOPPED
    elif observed_status.status.kind in (
        LoggerStatusKind.FILE_RUNNING,
        LoggerStatusKind.MAVLINK_ONLY,
    ):
        owner = LoggerOwner.EXTERNAL_ALREADY_RUNNING
        phase = LoggerLifecyclePhase.PENDING_ACTIVE
    else:
        owner = LoggerOwner.UNKNOWN
        phase = LoggerLifecyclePhase.STOP_UNVERIFIED
    return replace(
        state,
        current_status=observed_status.status,
        owner=owner,
        phase=phase,
        ownership_token=None,
    )


ULOG_MAGIC = b"ULog\x01\x12\x35"
ULOG_HEADER_SIZE_BYTES = 16
ULOG_FLAG_BITS_TYPE = ord("B")
ULOG_FLAG_BITS_MIN_PAYLOAD_BYTES = 40
ULOG_SUPPORTED_VERSIONS = (1,)
ULOG_APPEND_INCOMPAT_FLAG = 0x01


@dataclass(frozen=True)
class ULogInventoryEntry:
    """One remote ULog listing bound to a boot/link and recording session."""

    runtime_identity: LoggerRuntimeIdentity
    session_id: str
    remote_path: str
    size_bytes: int
    listed_at_utc_ns: int | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.runtime_identity, LoggerRuntimeIdentity):
            raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
        session_id = _nonempty_text(self.session_id, "session_id")
        path = _nonempty_text(self.remote_path, "remote_path")
        size = _strict_int(self.size_bytes, "size_bytes")
        if not path.startswith("/"):
            raise ValueError("remote_path must be an absolute PX4 path")
        if not path.lower().endswith(".ulg"):
            raise ValueError("remote_path must identify a .ulg file")
        listed = self.listed_at_utc_ns
        if listed is not None:
            listed = _strict_int(listed, "listed_at_utc_ns")
        object.__setattr__(self, "session_id", session_id)
        object.__setattr__(self, "remote_path", path)
        object.__setattr__(self, "size_bytes", size)
        if listed is not None:
            object.__setattr__(self, "listed_at_utc_ns", listed)

    @property
    def source_id(self) -> MavlinkSourceId:
        return self.runtime_identity.source_id


class ULogInventoryChangeKind(str, Enum):
    NEW = "NEW"
    GROWN = "GROWN"
    UNCHANGED = "UNCHANGED"
    SHRUNK = "SHRUNK"
    REMOVED = "REMOVED"


@dataclass(frozen=True)
class ULogInventoryChange:
    kind: ULogInventoryChangeKind
    before: ULogInventoryEntry | None
    after: ULogInventoryEntry | None

    def __post_init__(self) -> None:
        kind = ULogInventoryChangeKind(self.kind)
        if self.before is None and self.after is None:
            raise ValueError("an inventory change requires a before or after entry")
        for name in ("before", "after"):
            value = getattr(self, name)
            if value is not None and not isinstance(value, ULogInventoryEntry):
                raise TypeError(f"{name} must be ULogInventoryEntry")
        if self.before is not None and self.after is not None:
            if self.before.remote_path != self.after.remote_path:
                raise ValueError("before and after inventory paths must match")
            if self.before.runtime_identity != self.after.runtime_identity:
                raise ValueError("before and after runtime identities must match")
            if self.before.session_id != self.after.session_id:
                raise ValueError("before and after session identities must match")
        object.__setattr__(self, "kind", kind)

    @property
    def is_session_candidate(self) -> bool:
        return self.kind in (ULogInventoryChangeKind.NEW, ULogInventoryChangeKind.GROWN)


def _inventory_key(entry: ULogInventoryEntry) -> tuple[str, int, str, int, int, str, str]:
    identity = entry.runtime_identity
    return (
        identity.source_id.value,
        identity.connection_generation,
        identity.boot_id,
        identity.source_system,
        identity.source_component,
        entry.session_id,
        entry.remote_path,
    )


def _inventory_index(
    entries: Iterable[ULogInventoryEntry],
) -> dict[tuple[str, int, str, int, int, str, str], ULogInventoryEntry]:
    indexed: dict[tuple[str, int, str, int, int, str, str], ULogInventoryEntry] = {}
    for entry in entries:
        if not isinstance(entry, ULogInventoryEntry):
            raise TypeError("inventory entries must be ULogInventoryEntry")
        key = _inventory_key(entry)
        if key in indexed:
            raise ValueError(
                f"duplicate ULog inventory path for {entry.source_id.value}/"
                f"{entry.runtime_identity.boot_id}/{entry.session_id}: {entry.remote_path}"
            )
        indexed[key] = entry
    return indexed


def diff_ulog_inventories(
    before: Iterable[ULogInventoryEntry],
    after: Iterable[ULogInventoryEntry],
) -> tuple[ULogInventoryChange, ...]:
    """Return a deterministic path-wise diff; NEW/GROWN entries are candidates."""

    before_by_path = _inventory_index(before)
    after_by_path = _inventory_index(after)
    changes: list[ULogInventoryChange] = []
    for key in sorted(set(before_by_path) | set(after_by_path)):
        old = before_by_path.get(key)
        new = after_by_path.get(key)
        if old is None:
            kind = ULogInventoryChangeKind.NEW
        elif new is None:
            kind = ULogInventoryChangeKind.REMOVED
        elif new.size_bytes > old.size_bytes:
            kind = ULogInventoryChangeKind.GROWN
        elif new.size_bytes < old.size_bytes:
            kind = ULogInventoryChangeKind.SHRUNK
        else:
            kind = ULogInventoryChangeKind.UNCHANGED
        changes.append(ULogInventoryChange(kind=kind, before=old, after=new))
    return tuple(changes)


def session_ulog_candidates(
    before: Iterable[ULogInventoryEntry],
    after: Iterable[ULogInventoryEntry],
) -> tuple[ULogInventoryEntry, ...]:
    return tuple(
        change.after
        for change in diff_ulog_inventories(before, after)
        if change.is_session_candidate and change.after is not None
    )


class ULogValidationError(str, Enum):
    SIZE_MISMATCH = "SIZE_MISMATCH"
    HEADER_TOO_SHORT = "HEADER_TOO_SHORT"
    MAGIC_MISMATCH = "MAGIC_MISMATCH"
    UNSUPPORTED_VERSION = "UNSUPPORTED_VERSION"
    RECORD_HEADER_TRUNCATED = "RECORD_HEADER_TRUNCATED"
    RECORD_PAYLOAD_TRUNCATED = "RECORD_PAYLOAD_TRUNCATED"
    FIRST_RECORD_NOT_FLAG_BITS = "FIRST_RECORD_NOT_FLAG_BITS"
    FLAG_BITS_NOT_FIRST = "FLAG_BITS_NOT_FIRST"
    FLAG_BITS_SIZE_INVALID = "FLAG_BITS_SIZE_INVALID"
    UNKNOWN_INCOMPATIBILITY_FLAG = "UNKNOWN_INCOMPATIBILITY_FLAG"
    APPENDED_OFFSET_INVALID = "APPENDED_OFFSET_INVALID"
    SHA256_MISMATCH = "SHA256_MISMATCH"


@dataclass(frozen=True)
class ULogArtifactValidation:
    runtime_identity: LoggerRuntimeIdentity
    session_id: str
    remote_path: str
    expected_size_bytes: int
    actual_size_bytes: int
    sha256: str
    header_valid: bool
    record_structure_valid: bool
    record_count: int
    version: int | None
    start_timestamp_us: int | None
    errors: tuple[ULogValidationError, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.runtime_identity, LoggerRuntimeIdentity):
            raise TypeError("runtime_identity must be LoggerRuntimeIdentity")
        session_id = _nonempty_text(self.session_id, "session_id")
        errors = tuple(ULogValidationError(error) for error in self.errors)
        _strict_int(self.expected_size_bytes, "expected_size_bytes")
        _strict_int(self.actual_size_bytes, "actual_size_bytes")
        _strict_int(self.record_count, "record_count")
        _strict_bool(self.header_valid, "header_valid")
        _strict_bool(self.record_structure_valid, "record_structure_valid")
        if not re.fullmatch(r"[0-9a-f]{64}", self.sha256):
            raise ValueError("sha256 must be a lowercase SHA-256 digest")
        if self.version is not None:
            _strict_int(self.version, "version", maximum=255)
        if self.start_timestamp_us is not None:
            _strict_int(self.start_timestamp_us, "start_timestamp_us")
        object.__setattr__(self, "session_id", session_id)
        object.__setattr__(self, "errors", errors)

    @property
    def source_id(self) -> MavlinkSourceId:
        return self.runtime_identity.source_id

    @property
    def valid(self) -> bool:
        return not self.errors

    @property
    def size_matches(self) -> bool:
        return self.actual_size_bytes == self.expected_size_bytes


def validate_ulog_artifact(
    inventory_entry: ULogInventoryEntry,
    payload: bytes | bytearray | memoryview,
    *,
    expected_sha256: str | None = None,
) -> ULogArtifactValidation:
    """Validate already-downloaded bytes without performing filesystem/MAVFTP I/O."""

    if not isinstance(inventory_entry, ULogInventoryEntry):
        raise TypeError("inventory_entry must be ULogInventoryEntry")
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("payload must be bytes-like")
    data = bytes(payload)
    digest = hashlib.sha256(data).hexdigest()
    errors: list[ULogValidationError] = []
    if len(data) != inventory_entry.size_bytes:
        errors.append(ULogValidationError.SIZE_MISMATCH)
    version: int | None = None
    start_timestamp_us: int | None = None
    record_count = 0
    record_structure_valid = False
    if len(data) < ULOG_HEADER_SIZE_BYTES:
        errors.append(ULogValidationError.HEADER_TOO_SHORT)
        header_valid = False
    elif data[: len(ULOG_MAGIC)] != ULOG_MAGIC:
        errors.append(ULogValidationError.MAGIC_MISMATCH)
        header_valid = False
    else:
        version = data[len(ULOG_MAGIC)]
        start_timestamp_us = struct.unpack_from("<Q", data, 8)[0]
        if version not in ULOG_SUPPORTED_VERSIONS:
            errors.append(ULogValidationError.UNSUPPORTED_VERSION)
            header_valid = False
        else:
            header_valid = True
        offset = ULOG_HEADER_SIZE_BYTES
        first_record = True
        structural_errors: list[ULogValidationError] = []
        record_offsets: set[int] = set()
        appended_offsets_to_validate: tuple[int, ...] = ()
        while offset < len(data):
            record_offsets.add(offset)
            if len(data) - offset < 3:
                structural_errors.append(ULogValidationError.RECORD_HEADER_TRUNCATED)
                break
            message_size, message_type = struct.unpack_from("<HB", data, offset)
            record_count += 1
            payload_start = offset + 3
            payload_end = payload_start + message_size
            if payload_end > len(data):
                structural_errors.append(ULogValidationError.RECORD_PAYLOAD_TRUNCATED)
                break
            if first_record:
                if message_type != ULOG_FLAG_BITS_TYPE:
                    structural_errors.append(ULogValidationError.FIRST_RECORD_NOT_FLAG_BITS)
                if message_size < ULOG_FLAG_BITS_MIN_PAYLOAD_BYTES:
                    structural_errors.append(ULogValidationError.FLAG_BITS_SIZE_INVALID)
                elif message_type == ULOG_FLAG_BITS_TYPE:
                    flag_payload = data[payload_start:payload_end]
                    incompat_flags = flag_payload[8:16]
                    if (
                        incompat_flags[0] & ~ULOG_APPEND_INCOMPAT_FLAG
                        or any(incompat_flags[1:])
                    ):
                        structural_errors.append(
                            ULogValidationError.UNKNOWN_INCOMPATIBILITY_FLAG
                        )
                    append_flag_set = bool(
                        incompat_flags[0] & ULOG_APPEND_INCOMPAT_FLAG
                    )
                    appended_offsets = struct.unpack_from("<QQQ", flag_payload, 16)
                    nonzero_offsets = tuple(
                        offset_value
                        for offset_value in appended_offsets
                        if offset_value != 0
                    )
                    appended_offsets_to_validate = nonzero_offsets
                    if (
                        (nonzero_offsets and not append_flag_set)
                        or any(
                            offset_value == 0 and any(appended_offsets[index + 1 :])
                            for index, offset_value in enumerate(appended_offsets)
                        )
                        or tuple(sorted(set(nonzero_offsets))) != nonzero_offsets
                        or any(
                            offset_value < payload_end or offset_value >= len(data)
                            for offset_value in nonzero_offsets
                        )
                    ):
                        structural_errors.append(
                            ULogValidationError.APPENDED_OFFSET_INVALID
                        )
                first_record = False
            elif message_type == ULOG_FLAG_BITS_TYPE:
                structural_errors.append(ULogValidationError.FLAG_BITS_NOT_FIRST)
            offset = payload_end
        if first_record:
            structural_errors.append(ULogValidationError.RECORD_HEADER_TRUNCATED)
        if any(
            appended_offset not in record_offsets
            for appended_offset in appended_offsets_to_validate
        ):
            structural_errors.append(ULogValidationError.APPENDED_OFFSET_INVALID)
        errors.extend(structural_errors)
        record_structure_valid = not structural_errors
    if expected_sha256 is not None:
        if not isinstance(expected_sha256, str):
            raise TypeError("expected_sha256 must be str")
        expected_digest = expected_sha256.strip().lower()
        if not re.fullmatch(r"[0-9a-f]{64}", expected_digest):
            raise ValueError("expected_sha256 must be a 64-character hexadecimal digest")
        if digest != expected_digest:
            errors.append(ULogValidationError.SHA256_MISMATCH)
    return ULogArtifactValidation(
        runtime_identity=inventory_entry.runtime_identity,
        session_id=inventory_entry.session_id,
        remote_path=inventory_entry.remote_path,
        expected_size_bytes=inventory_entry.size_bytes,
        actual_size_bytes=len(data),
        sha256=digest,
        header_valid=header_valid,
        record_structure_valid=record_structure_valid,
        record_count=record_count,
        version=version,
        start_timestamp_us=start_timestamp_us,
        errors=tuple(dict.fromkeys(errors)),
    )
