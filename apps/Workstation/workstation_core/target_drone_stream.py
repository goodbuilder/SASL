"""Read-only MAVLink runtime for a third, target-drone telemetry source.

This module deliberately does *not* reuse the rig Pixhawk command path.  The
only calls made on a pymavlink connection are ``wait_heartbeat``,
``recv_match``, and ``close``.  In particular it never sends a heartbeat,
requests a stream, changes ``MAV_SYS_ID`` or a parameter, injects RTCM,
controls a logger, or writes through MAVFTP.

Coordinate contract
-------------------
``ATTITUDE`` roll/pitch/yaw describe PX4 body FRD orientation in world NED.
``GPS_RAW_INT``/``GPS2_RAW`` latitude/longitude are WGS84 degrees and ``alt``
is metres above mean sea level.  GPS course over ground (COG) and the MAVLink
GNSS-yaw extension are clockwise from true north.  COG, GNSS yaw, and
ATTITUDE yaw remain separately named observations and are never substituted
for one another.  Nothing in this module is an OpenCV optical-RDF pose,
left/right-camera pose, stereo-centre pose, or calibration-time truth.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping

from Workstation.workstation_core.mavlink_sources import (
    AttitudeYaw,
    CourseOverGround,
    GnssYaw,
    MavlinkPacketProvenance,
    MavlinkSourceConfig,
    MavlinkSourceId,
    MavlinkYawObservations,
)


TARGET_DRONE_SCHEMA_VERSION = "target_drone_mavlink_v1"
_ALLOWED_MESSAGE_IDS: dict[str, int] = {
    "ATTITUDE": 30,
    "GPS_RAW_INT": 24,
    "GPS2_RAW": 124,
}
_GPS_UNAVAILABLE_UINT16 = 65_535
_UNIX_2000_EPOCH_US = 946_684_800_000_000


class TargetDroneMode(str, Enum):
    DRONE = "DRONE"
    UAM = "UAM"


class TargetDroneStreamState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    STREAMING = "STREAMING"
    DEFERRED = "DEFERRED"
    IDENTITY_MISMATCH = "IDENTITY_MISMATCH"
    FAILED = "FAILED"
    STOPPED = "STOPPED"


class TargetDroneStreamError(RuntimeError):
    """Base class for target-drone runtime failures."""


class HeartbeatTimeoutError(TargetDroneStreamError):
    """No heartbeat was received before the configured timeout."""


class HeartbeatIdentityError(TargetDroneStreamError):
    """The endpoint heartbeat did not match the configured aircraft."""


class SpoolCapacityError(TargetDroneStreamError):
    """The bounded spool cannot accept another complete JSONL row."""


class SpoolIntegrityError(TargetDroneStreamError):
    """An existing append-only spool is not safe to resume."""


def _target_drone_mode(value: Any) -> TargetDroneMode:
    raw = getattr(value, "value", value)
    return TargetDroneMode(str(raw).strip().upper())


def _strict_positive_int(value: int, name: str) -> int:
    if type(value) is not int or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _reject_nonfinite_json_constant(token: str) -> None:
    raise ValueError(f"non-finite JSON constant: {token}")


def _finite_optional(value: Any, *, scale: float = 1.0) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value) * scale
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


def _optional_int(value: Any) -> int | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _message_value(message: Any, name: str) -> Any:
    try:
        return getattr(message, name)
    except Exception:
        return None


def _message_method_or_attribute(message: Any, *names: str) -> Any:
    for name in names:
        try:
            value = getattr(message, name)
        except Exception:
            continue
        try:
            return value() if callable(value) else value
        except Exception:
            continue
    return None


def _bounded_packet_int(value: Any, *, minimum: int, maximum: int) -> int | None:
    parsed = _optional_int(value)
    if parsed is None or not minimum <= parsed <= maximum:
        return None
    return parsed


def _message_identity(message: Any) -> tuple[int | None, int | None]:
    return (
        _bounded_packet_int(
            _message_method_or_attribute(message, "get_srcSystem", "srcSystem"),
            minimum=1,
            maximum=255,
        ),
        _bounded_packet_int(
            _message_method_or_attribute(message, "get_srcComponent", "srcComponent"),
            minimum=1,
            maximum=255,
        ),
    )


def _message_type(message: Any) -> str:
    raw = _message_method_or_attribute(message, "get_type")
    return "" if raw is None else str(raw).strip().upper()


def _message_provenance(
    message: Any,
    *,
    monotonic_ns: int,
    utc_ns: int,
) -> MavlinkPacketProvenance | None:
    message_type = _message_type(message)
    source_system, source_component = _message_identity(message)
    sequence = _bounded_packet_int(
        _message_method_or_attribute(message, "get_seq", "seq"),
        minimum=0,
        maximum=255,
    )
    message_id = _bounded_packet_int(
        _message_method_or_attribute(message, "get_msgId", "msgid", "msg_id"),
        minimum=0,
        maximum=16_777_215,
    )
    if message_id is None:
        message_id = _ALLOWED_MESSAGE_IDS.get(message_type)
    if None in (source_system, source_component, sequence, message_id):
        return None
    source_timestamp_us: int | None
    if message_type == "ATTITUDE":
        time_boot_ms = _optional_int(_message_value(message, "time_boot_ms"))
        source_timestamp_us = None if time_boot_ms is None or time_boot_ms < 0 else time_boot_ms * 1000
    else:
        time_usec = _optional_int(_message_value(message, "time_usec"))
        source_timestamp_us = None if time_usec is None or time_usec < 0 else time_usec
    return MavlinkPacketProvenance(
        source_id=MavlinkSourceId.TARGET_DRONE,
        source_system=source_system,
        source_component=source_component,
        sequence=sequence,
        message_id=message_id,
        host_received_monotonic_ns=int(monotonic_ns),
        host_received_utc_ns=int(utc_ns),
        source_timestamp_us=source_timestamp_us,
    )


def _provenance_dict(provenance: MavlinkPacketProvenance) -> dict[str, Any]:
    return {
        "source_id": provenance.source_id.value,
        "source_system": provenance.source_system,
        "source_component": provenance.source_component,
        "sequence": provenance.sequence,
        "message_id": provenance.message_id,
        "host_received_monotonic_ns": provenance.host_received_monotonic_ns,
        "host_received_utc_ns": provenance.host_received_utc_ns,
        "source_timestamp_us": provenance.source_timestamp_us,
    }


def _yaw_dict(observations: MavlinkYawObservations) -> dict[str, float | None]:
    return {
        "course_over_ground_deg_true_north": (
            None
            if observations.course_over_ground is None
            else observations.course_over_ground.degrees_true_north
        ),
        "gnss_yaw_deg_true_north": (
            None if observations.gnss_yaw is None else observations.gnss_yaw.degrees_true_north
        ),
        "attitude_yaw_deg_body_frd_to_world_ned": (
            None if observations.attitude_yaw is None else observations.attitude_yaw.degrees_ned
        ),
    }


def _parse_cog(value: Any, provenance: MavlinkPacketProvenance) -> tuple[CourseOverGround | None, str | None]:
    parsed = _optional_int(value)
    try:
        return CourseOverGround.from_mavlink_cdeg(parsed, provenance), None
    except (TypeError, ValueError) as exc:
        return None, str(exc)


def _parse_gnss_yaw(value: Any, provenance: MavlinkPacketProvenance) -> tuple[GnssYaw | None, str | None]:
    parsed = _optional_int(value)
    try:
        return GnssYaw.from_mavlink_cdeg(parsed, provenance), None
    except (TypeError, ValueError) as exc:
        return None, str(exc)


def _parse_attitude_yaw(
    value: Any,
    provenance: MavlinkPacketProvenance,
) -> tuple[AttitudeYaw | None, str | None]:
    parsed = _finite_optional(value)
    try:
        return AttitudeYaw.from_mavlink_rad(parsed, provenance), None
    except (TypeError, ValueError) as exc:
        return None, str(exc)


@dataclass(frozen=True)
class TargetDroneTelemetryRecord:
    """One validated packet from the heartbeat-bound target aircraft."""

    connection_generation: int
    endpoint: str
    message_type: str
    sensor_slot: str | None
    provenance: MavlinkPacketProvenance
    fields: Mapping[str, Any]
    yaw_observations: MavlinkYawObservations
    parse_warnings: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.connection_generation <= 0:
            raise ValueError("connection_generation must be positive")
        if self.message_type not in _ALLOWED_MESSAGE_IDS:
            raise ValueError(f"unsupported target-drone message: {self.message_type}")
        if self.provenance.source_id != MavlinkSourceId.TARGET_DRONE:
            raise ValueError("target-drone record provenance has the wrong source")
        if self.yaw_observations.source_id not in (None, MavlinkSourceId.TARGET_DRONE):
            raise ValueError("target-drone yaw observation has the wrong source")
        object.__setattr__(self, "fields", dict(self.fields))
        object.__setattr__(self, "parse_warnings", tuple(str(item) for item in self.parse_warnings))

    def to_json_dict(self) -> dict[str, Any]:
        source_time_basis = "unavailable"
        if self.provenance.source_timestamp_us is not None:
            if self.message_type == "ATTITUDE":
                source_time_basis = "px4_boot_time"
            elif self.provenance.source_timestamp_us >= _UNIX_2000_EPOCH_US:
                source_time_basis = "utc_epoch"
            else:
                source_time_basis = "gnss_receiver_or_boot_time_unresolved"
        return {
            "schema_version": TARGET_DRONE_SCHEMA_VERSION,
            "record_kind": "telemetry",
            "source_id": MavlinkSourceId.TARGET_DRONE.value,
            "source_role": "TARGET_REFERENCE",
            "connection_generation": self.connection_generation,
            "endpoint": self.endpoint,
            "message_type": self.message_type,
            "sensor_slot": self.sensor_slot,
            "provenance": _provenance_dict(self.provenance),
            "source_time_basis": source_time_basis,
            "coordinate_contract": {
                "world": "NED",
                "body": "FRD",
                "geodetic": "WGS84",
                "attitude": "body_FRD_to_world_NED",
                "cog": "clockwise_from_true_north",
                "gnss_yaw": "clockwise_from_true_north",
                "gps_altitude": "metres_above_mean_sea_level",
            },
            "fields": dict(self.fields),
            "yaw_observations": _yaw_dict(self.yaw_observations),
            "parse_warnings": list(self.parse_warnings),
        }


@dataclass(frozen=True)
class TargetDroneStreamSnapshot:
    state: TargetDroneStreamState
    connection_generation: int
    verified_source_system: int | None
    verified_source_component: int | None
    accepted_message_count: int
    ignored_message_count: int
    rejected_cross_source_count: int
    rejected_invalid_provenance_count: int
    spool_row_count: int
    spool_size_bytes: int
    last_error: str | None


class BoundedAppendOnlyJsonlSpool:
    """Single-file O_APPEND spool with explicit row and byte capacities.

    Capacity is enforced before each append.  The existing prefix is never
    truncated or rewritten, so a full spool fails closed instead of silently
    evicting previously captured evidence.
    """

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        max_rows: int = 1_000_000,
        max_bytes: int = 512 * 1024 * 1024,
        max_row_bytes: int = 1024 * 1024,
        fsync_every_rows: int = 1,
    ) -> None:
        self.path = Path(path)
        self.max_rows = _strict_positive_int(max_rows, "max_rows")
        self.max_bytes = _strict_positive_int(max_bytes, "max_bytes")
        self.max_row_bytes = _strict_positive_int(max_row_bytes, "max_row_bytes")
        self.fsync_every_rows = _strict_positive_int(fsync_every_rows, "fsync_every_rows")
        self._fd: int | None = None
        self._row_count = 0
        self._size_bytes = 0
        self._rows_since_fsync = 0
        self._lock = threading.RLock()

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._fd is not None

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
            if self._fd is not None:
                return
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.exists():
                size = self.path.stat().st_size
                if size > self.max_bytes:
                    raise SpoolCapacityError("existing spool exceeds max_bytes")
                rows = 0
                with self.path.open("rb") as existing:
                    for line_number, line in enumerate(existing, start=1):
                        if len(line) > self.max_row_bytes:
                            raise SpoolCapacityError(
                                f"existing JSONL spool row {line_number} exceeds max_row_bytes"
                            )
                        if not line.endswith(b"\n"):
                            raise SpoolIntegrityError(
                                "existing JSONL spool does not end on a row boundary"
                            )
                        rows += 1
                        if rows > self.max_rows:
                            raise SpoolCapacityError("existing spool exceeds max_rows")
                        try:
                            decoded = json.loads(line, parse_constant=_reject_nonfinite_json_constant)
                            if not isinstance(decoded, dict):
                                raise ValueError("JSONL row is not an object")
                        except Exception as exc:
                            raise SpoolIntegrityError(
                                f"existing JSONL spool row {line_number} is invalid"
                            ) from exc
                if rows > self.max_rows:
                    raise SpoolCapacityError("existing spool exceeds max_rows")
                self._row_count = rows
                self._size_bytes = size
            else:
                self._row_count = 0
                self._size_bytes = 0
            self._rows_since_fsync = 0
            self._fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)

    def append(self, row: Mapping[str, Any]) -> None:
        try:
            encoded = (
                json.dumps(dict(row), ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)
                + "\n"
            ).encode("utf-8")
        except (TypeError, ValueError) as exc:
            raise SpoolIntegrityError(f"JSONL row is not finite/serializable: {exc}") from exc
        if len(encoded) > self.max_row_bytes:
            raise SpoolCapacityError("JSONL row exceeds max_row_bytes")
        with self._lock:
            if self._fd is None:
                raise RuntimeError("spool is not open")
            if self._row_count + 1 > self.max_rows:
                raise SpoolCapacityError("JSONL spool reached max_rows")
            if self._size_bytes + len(encoded) > self.max_bytes:
                raise SpoolCapacityError("JSONL spool reached max_bytes")
            written = 0
            while written < len(encoded):
                count = os.write(self._fd, encoded[written:])
                if count <= 0:
                    raise OSError("short write while appending JSONL spool")
                written += count
            self._row_count += 1
            self._size_bytes += len(encoded)
            self._rows_since_fsync += 1
            if self._rows_since_fsync >= self.fsync_every_rows:
                os.fsync(self._fd)
                self._rows_since_fsync = 0

    def flush(self) -> None:
        with self._lock:
            if self._fd is not None:
                os.fsync(self._fd)
                self._rows_since_fsync = 0

    def close(self) -> None:
        with self._lock:
            if self._fd is None:
                return
            try:
                os.fsync(self._fd)
            finally:
                os.close(self._fd)
                self._fd = None
                self._rows_since_fsync = 0

    def sha256(self) -> str:
        digest = hashlib.sha256()
        with self.path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()


class TargetDroneReceiver:
    """Synchronous recv-only receiver; call ``poll_once`` from one thread."""

    def __init__(
        self,
        config: MavlinkSourceConfig,
        *,
        mode: TargetDroneMode | str,
        spool_path: str | os.PathLike[str],
        heartbeat_timeout_s: float = 3.0,
        spool_max_rows: int = 1_000_000,
        spool_max_bytes: int = 512 * 1024 * 1024,
        spool_fsync_every_rows: int = 1,
        mavutil_module: Any | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        utc_ns: Callable[[], int] = time.time_ns,
        on_record: Callable[[TargetDroneTelemetryRecord], None] | None = None,
    ) -> None:
        if config.source_id != MavlinkSourceId.TARGET_DRONE:
            raise ValueError("TargetDroneReceiver requires a target_drone source config")
        if any(
            (
                config.policy.allow_stream_setup,
                config.policy.allow_parameter_write,
                config.policy.allow_rtcm_injection,
                config.policy.allow_logger_control,
            )
        ):
            raise ValueError("target-drone receiver requires a recv-only mutation policy")
        self.config = config
        self.mode = _target_drone_mode(mode)
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        if not math.isfinite(self.heartbeat_timeout_s) or self.heartbeat_timeout_s <= 0.0:
            raise ValueError("heartbeat_timeout_s must be finite and positive")
        self.spool = BoundedAppendOnlyJsonlSpool(
            spool_path,
            max_rows=spool_max_rows,
            max_bytes=spool_max_bytes,
            fsync_every_rows=spool_fsync_every_rows,
        )
        self._mavutil_module = mavutil_module
        self._monotonic_ns = monotonic_ns
        self._utc_ns = utc_ns
        self._on_record = on_record
        self._connection: Any | None = None
        self._state = TargetDroneStreamState.DISCONNECTED
        self._state_lock = threading.RLock()
        self._poll_lock = threading.RLock()
        self._connection_generation = 0
        self._verified_source_system: int | None = None
        self._verified_source_component: int | None = None
        self._accepted_message_count = 0
        self._ignored_message_count = 0
        self._rejected_cross_source_count = 0
        self._rejected_invalid_provenance_count = 0
        self._rejection_audit_keys: set[tuple[Any, ...]] = set()
        self._last_error: str | None = None
        self._last_attitude: TargetDroneTelemetryRecord | None = None
        self._last_gps: dict[str, TargetDroneTelemetryRecord] = {}

    @property
    def state(self) -> TargetDroneStreamState:
        with self._state_lock:
            return self._state

    @property
    def last_attitude(self) -> TargetDroneTelemetryRecord | None:
        with self._state_lock:
            return self._last_attitude

    def last_gps(self, sensor_slot: str) -> TargetDroneTelemetryRecord | None:
        with self._state_lock:
            return self._last_gps.get(str(sensor_slot).upper())

    def _set_state(self, state: TargetDroneStreamState, error: str | None = None) -> None:
        with self._state_lock:
            self._state = state
            self._last_error = error

    def _mavutil(self) -> Any:
        if self._mavutil_module is not None:
            return self._mavutil_module
        try:
            from pymavlink import mavutil
        except Exception as exc:
            raise TargetDroneStreamError(f"pymavlink is required for target-drone telemetry: {exc}") from exc
        self._mavutil_module = mavutil
        return mavutil

    def _lifecycle_row(self, event: str, **fields: Any) -> dict[str, Any]:
        return {
            "schema_version": TARGET_DRONE_SCHEMA_VERSION,
            "record_kind": "lifecycle",
            "source_id": MavlinkSourceId.TARGET_DRONE.value,
            "source_role": "TARGET_REFERENCE",
            "connection_generation": self._connection_generation,
            "endpoint": self.config.endpoint,
            "host_recorded_monotonic_ns": int(self._monotonic_ns()),
            "host_recorded_utc_ns": int(self._utc_ns()),
            "event": event,
            **fields,
        }

    def start(self) -> TargetDroneStreamState:
        with self._poll_lock:
            if self.mode == TargetDroneMode.UAM:
                # UAM RTK is intentionally post-flight: not starting a live
                # worker is a successful, explicit state rather than an error.
                self._set_state(TargetDroneStreamState.DEFERRED)
                return self.state
            if self._connection is not None:
                return self.state
            with self._state_lock:
                self._connection_generation += 1
                self._verified_source_system = None
                self._verified_source_component = None
                self._last_attitude = None
                self._last_gps.clear()
            self._rejection_audit_keys.clear()
            self._set_state(TargetDroneStreamState.CONNECTING)
            connection = None
            try:
                self.spool.open()
                self.spool.append(
                    self._lifecycle_row(
                        "connection_attempt",
                        expected_source_system=self.config.expected_system_id,
                        expected_source_component=self.config.expected_component_id,
                        mutation_policy="recv_only",
                    )
                )
                mavutil = self._mavutil()
                connection = mavutil.mavlink_connection(
                    self.config.endpoint,
                    baud=self.config.baudrate,
                    autoreconnect=False,
                    robust_parsing=True,
                )
                self._connection = connection
                heartbeat = connection.wait_heartbeat(timeout=self.heartbeat_timeout_s)
                if heartbeat is None:
                    raise HeartbeatTimeoutError("target-drone heartbeat timed out")
                source_system, source_component = _message_identity(heartbeat)
                if source_system is None or source_component is None:
                    raise HeartbeatIdentityError("target-drone heartbeat has no valid MAVLink identity")
                expected_system = self.config.expected_system_id
                expected_component = self.config.expected_component_id
                if expected_system is not None and source_system != expected_system:
                    raise HeartbeatIdentityError(
                        f"target-drone heartbeat sysid mismatch: expected {expected_system}, got {source_system}"
                    )
                if expected_component is not None and source_component != expected_component:
                    raise HeartbeatIdentityError(
                        "target-drone heartbeat compid mismatch: "
                        f"expected {expected_component}, got {source_component}"
                    )
                with self._state_lock:
                    self._verified_source_system = source_system
                    self._verified_source_component = source_component
                self.spool.append(
                    self._lifecycle_row(
                        "heartbeat_verified",
                        verified_source_system=source_system,
                        verified_source_component=source_component,
                    )
                )
                self._set_state(TargetDroneStreamState.STREAMING)
                return self.state
            except HeartbeatIdentityError as exc:
                self._set_state(TargetDroneStreamState.IDENTITY_MISMATCH, str(exc))
                self._append_failure_lifecycle("heartbeat_identity_rejected", str(exc))
                self._close_connection(connection)
                self.spool.close()
                raise
            except Exception as exc:
                self._set_state(TargetDroneStreamState.FAILED, str(exc))
                self._append_failure_lifecycle("connection_failed", str(exc))
                self._close_connection(connection)
                self.spool.close()
                if isinstance(exc, TargetDroneStreamError):
                    raise
                raise TargetDroneStreamError(str(exc)) from exc

    def _append_failure_lifecycle(self, event: str, error: str) -> None:
        try:
            if self.spool.is_open:
                self.spool.append(self._lifecycle_row(event, error=error))
        except Exception:
            pass

    def _close_connection(self, connection: Any | None = None) -> None:
        target = self._connection if connection is None else connection
        if target is not None:
            try:
                target.close()
            except Exception:
                pass
        if target is self._connection or connection is None:
            self._connection = None

    def poll_once(self, *, max_messages: int = 256, max_duration_s: float = 0.020) -> int:
        _strict_positive_int(max_messages, "max_messages")
        duration = float(max_duration_s)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("max_duration_s must be finite and positive")
        with self._poll_lock:
            if self.state != TargetDroneStreamState.STREAMING or self._connection is None:
                return 0
            deadline_ns = self._monotonic_ns() + int(duration * 1_000_000_000)
            handled = 0
            try:
                for _ in range(max_messages):
                    message = self._connection.recv_match(blocking=False)
                    if message is None:
                        break
                    if self._handle_message(message):
                        handled += 1
                    if self._monotonic_ns() >= deadline_ns:
                        break
            except Exception as exc:
                self._set_state(TargetDroneStreamState.FAILED, str(exc))
                self._append_failure_lifecycle("receive_failed", str(exc))
                raise
            return handled

    def _handle_message(self, message: Any) -> bool:
        message_type = _message_type(message)
        if message_type not in _ALLOWED_MESSAGE_IDS:
            with self._state_lock:
                self._ignored_message_count += 1
            return False
        received_monotonic_ns = int(self._monotonic_ns())
        received_utc_ns = int(self._utc_ns())
        source_system, source_component = _message_identity(message)
        if (
            source_system != self._verified_source_system
            or source_component != self._verified_source_component
        ):
            with self._state_lock:
                self._rejected_cross_source_count += 1
            key = (source_system, source_component, message_type)
            if key not in self._rejection_audit_keys:
                self._rejection_audit_keys.add(key)
                self.spool.append(
                    self._lifecycle_row(
                        "packet_identity_rejected",
                        message_type=message_type,
                        observed_source_system=source_system,
                        observed_source_component=source_component,
                        verified_source_system=self._verified_source_system,
                        verified_source_component=self._verified_source_component,
                    )
                )
            return False
        provenance = _message_provenance(
            message,
            monotonic_ns=received_monotonic_ns,
            utc_ns=received_utc_ns,
        )
        if provenance is None:
            with self._state_lock:
                self._rejected_invalid_provenance_count += 1
            return False
        record = self._record_from_message(message_type, message, provenance)
        # Persist before publishing/updating latest state.  A spool error thus
        # fails closed and no unrecorded sample appears live to a consumer.
        self.spool.append(record.to_json_dict())
        with self._state_lock:
            self._accepted_message_count += 1
            if message_type == "ATTITUDE":
                self._last_attitude = record
            elif record.sensor_slot is not None:
                self._last_gps[record.sensor_slot] = record
        if self._on_record is not None:
            self._on_record(record)
        return True

    def _record_from_message(
        self,
        message_type: str,
        message: Any,
        provenance: MavlinkPacketProvenance,
    ) -> TargetDroneTelemetryRecord:
        warnings: list[str] = []
        if message_type == "ATTITUDE":
            attitude_yaw, warning = _parse_attitude_yaw(_message_value(message, "yaw"), provenance)
            if warning is not None:
                warnings.append(f"attitude_yaw: {warning}")
            observations = MavlinkYawObservations(attitude_yaw=attitude_yaw)
            fields: dict[str, Any] = {
                "roll_rad_body_frd_to_world_ned": _finite_optional(_message_value(message, "roll")),
                "pitch_rad_body_frd_to_world_ned": _finite_optional(_message_value(message, "pitch")),
                "yaw_rad_body_frd_to_world_ned": _finite_optional(_message_value(message, "yaw")),
                "rollspeed_rad_s_body_frd": _finite_optional(_message_value(message, "rollspeed")),
                "pitchspeed_rad_s_body_frd": _finite_optional(_message_value(message, "pitchspeed")),
                "yawspeed_rad_s_body_frd": _finite_optional(_message_value(message, "yawspeed")),
                "time_boot_ms": _optional_int(_message_value(message, "time_boot_ms")),
            }
            sensor_slot = None
        else:
            cog, cog_warning = _parse_cog(_message_value(message, "cog"), provenance)
            gnss_yaw, yaw_warning = _parse_gnss_yaw(_message_value(message, "yaw"), provenance)
            if cog_warning is not None:
                warnings.append(f"course_over_ground: {cog_warning}")
            if yaw_warning is not None:
                warnings.append(f"gnss_yaw: {yaw_warning}")
            observations = MavlinkYawObservations(course_over_ground=cog, gnss_yaw=gnss_yaw)
            lat_raw = _optional_int(_message_value(message, "lat"))
            lon_raw = _optional_int(_message_value(message, "lon"))
            alt_raw = _optional_int(_message_value(message, "alt"))
            alt_ellipsoid_raw = _optional_int(_message_value(message, "alt_ellipsoid"))
            eph_raw = _optional_int(_message_value(message, "eph"))
            epv_raw = _optional_int(_message_value(message, "epv"))
            velocity_raw = _optional_int(_message_value(message, "vel"))
            cog_raw = _optional_int(_message_value(message, "cog"))
            yaw_raw = _optional_int(_message_value(message, "yaw"))
            horizontal_accuracy_raw = _optional_int(_message_value(message, "h_acc"))
            vertical_accuracy_raw = _optional_int(_message_value(message, "v_acc"))
            velocity_accuracy_raw = _optional_int(_message_value(message, "vel_acc"))
            heading_accuracy_raw = _optional_int(_message_value(message, "hdg_acc"))
            fix_type = _optional_int(_message_value(message, "fix_type"))
            fields = {
                # Lossless MAVLink integer values are retained next to the
                # explicit SI/WGS84 projections used by the UI and analysis.
                "latitude_int_deg_e7": lat_raw,
                "longitude_int_deg_e7": lon_raw,
                "altitude_mm_msl": alt_raw,
                "altitude_mm_wgs84_ellipsoid": alt_ellipsoid_raw,
                "eph_uint16_cm": eph_raw,
                "epv_uint16_cm": epv_raw,
                "ground_speed_uint16_cm_s": velocity_raw,
                "course_over_ground_uint16_cdeg": cog_raw,
                "gnss_yaw_uint16_cdeg": yaw_raw,
                "horizontal_accuracy_uint32_mm": horizontal_accuracy_raw,
                "vertical_accuracy_uint32_mm": vertical_accuracy_raw,
                "velocity_accuracy_uint32_mm_s": velocity_accuracy_raw,
                "heading_accuracy_uint32_deg_e5": heading_accuracy_raw,
                "latitude_deg_wgs84": None if lat_raw is None else lat_raw * 1e-7,
                "longitude_deg_wgs84": None if lon_raw is None else lon_raw * 1e-7,
                "altitude_m_msl": None if alt_raw is None else alt_raw * 1e-3,
                "altitude_m_wgs84_ellipsoid": (
                    None if alt_ellipsoid_raw is None else alt_ellipsoid_raw * 1e-3
                ),
                "fix_type": fix_type,
                "rtk_fix_label": (
                    "RTK_FIXED" if fix_type == 6 else "RTK_FLOAT" if fix_type == 5 else "NO_RTK"
                ),
                "satellites_visible": _optional_int(_message_value(message, "satellites_visible")),
                "horizontal_dilution_m": (
                    None if eph_raw in (None, _GPS_UNAVAILABLE_UINT16) else eph_raw * 1e-2
                ),
                "vertical_dilution_m": (
                    None if epv_raw in (None, _GPS_UNAVAILABLE_UINT16) else epv_raw * 1e-2
                ),
                "ground_speed_m_s": (
                    None if velocity_raw in (None, _GPS_UNAVAILABLE_UINT16) else velocity_raw * 1e-2
                ),
                "horizontal_accuracy_m": (
                    None
                    if horizontal_accuracy_raw in (None, 4_294_967_295)
                    else horizontal_accuracy_raw * 1e-3
                ),
                "vertical_accuracy_m": (
                    None
                    if vertical_accuracy_raw in (None, 4_294_967_295)
                    else vertical_accuracy_raw * 1e-3
                ),
                "velocity_accuracy_m_s": (
                    None
                    if velocity_accuracy_raw in (None, 4_294_967_295)
                    else velocity_accuracy_raw * 1e-3
                ),
                "time_usec": _optional_int(_message_value(message, "time_usec")),
                "heading_accuracy_deg": (
                    None
                    if heading_accuracy_raw in (None, 4_294_967_295)
                    else heading_accuracy_raw * 1e-5
                ),
                "dgps_age_ms": _optional_int(_message_value(message, "dgps_age")),
                "dgps_num_channels": _optional_int(_message_value(message, "dgps_numch")),
            }
            sensor_slot = "GPS1" if message_type == "GPS_RAW_INT" else "GPS2"
        return TargetDroneTelemetryRecord(
            connection_generation=self._connection_generation,
            endpoint=self.config.endpoint,
            message_type=message_type,
            sensor_slot=sensor_slot,
            provenance=provenance,
            fields=fields,
            yaw_observations=observations,
            parse_warnings=tuple(warnings),
        )

    def drain(self, *, max_messages: int = 4096, max_duration_s: float = 0.250) -> int:
        """Drain already-buffered packets without blocking, under a hard bound."""

        return self.poll_once(max_messages=max_messages, max_duration_s=max_duration_s)

    def stop(
        self,
        *,
        drain: bool = True,
        drain_max_messages: int = 4096,
        drain_max_duration_s: float = 0.250,
    ) -> TargetDroneStreamSnapshot:
        with self._poll_lock:
            if self.mode == TargetDroneMode.UAM:
                self._set_state(TargetDroneStreamState.DEFERRED)
                return self.snapshot()
            pending_error: BaseException | None = None
            try:
                if self._connection is not None and self.state == TargetDroneStreamState.STREAMING and drain:
                    self.poll_once(
                        max_messages=drain_max_messages,
                        max_duration_s=drain_max_duration_s,
                    )
            except BaseException as exc:
                pending_error = exc
            try:
                if self.spool.is_open:
                    try:
                        self.spool.append(
                            self._lifecycle_row(
                                "connection_stopped",
                                accepted_message_count=self._accepted_message_count,
                                ignored_message_count=self._ignored_message_count,
                                rejected_cross_source_count=self._rejected_cross_source_count,
                                rejected_invalid_provenance_count=self._rejected_invalid_provenance_count,
                                drain_error=None if pending_error is None else str(pending_error),
                            )
                        )
                    except SpoolCapacityError:
                        # The evidence prefix remains valid; capacity failure
                        # is already represented by counters/state at caller.
                        pass
            except BaseException as exc:
                if pending_error is None:
                    pending_error = exc
            finally:
                self._close_connection()
                try:
                    self.spool.close()
                except BaseException as exc:
                    if pending_error is None:
                        pending_error = exc
            if pending_error is not None:
                self._set_state(TargetDroneStreamState.FAILED, str(pending_error))
                raise pending_error
            if self.state not in (
                TargetDroneStreamState.FAILED,
                TargetDroneStreamState.IDENTITY_MISMATCH,
            ):
                self._set_state(TargetDroneStreamState.STOPPED)
            return self.snapshot()

    def snapshot(self) -> TargetDroneStreamSnapshot:
        with self._state_lock:
            return TargetDroneStreamSnapshot(
                state=self._state,
                connection_generation=self._connection_generation,
                verified_source_system=self._verified_source_system,
                verified_source_component=self._verified_source_component,
                accepted_message_count=self._accepted_message_count,
                ignored_message_count=self._ignored_message_count,
                rejected_cross_source_count=self._rejected_cross_source_count,
                rejected_invalid_provenance_count=self._rejected_invalid_provenance_count,
                spool_row_count=self.spool.row_count,
                spool_size_bytes=self.spool.size_bytes,
                last_error=self._last_error,
            )


class TargetDroneWorker:
    """Small thread boundary around :class:`TargetDroneReceiver`.

    UAM mode remains ``DEFERRED`` and creates no thread.  ``request_stop`` is
    cooperative; the receive loop is non-blocking after the one bounded
    heartbeat wait.  The worker drains the socket queue before closing it.
    """

    def __init__(
        self,
        receiver: TargetDroneReceiver,
        *,
        idle_wait_s: float = 0.010,
        drain_max_duration_s: float = 1.0,
    ) -> None:
        self.receiver = receiver
        self.idle_wait_s = float(idle_wait_s)
        if not math.isfinite(self.idle_wait_s) or self.idle_wait_s <= 0.0:
            raise ValueError("idle_wait_s must be finite and positive")
        self.drain_max_duration_s = float(drain_max_duration_s)
        if not math.isfinite(self.drain_max_duration_s) or self.drain_max_duration_s <= 0.0:
            raise ValueError("drain_max_duration_s must be finite and positive")
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._worker_error: BaseException | None = None

    @property
    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def worker_error(self) -> BaseException | None:
        return self._worker_error

    def start(self) -> TargetDroneStreamState:
        if self.receiver.mode == TargetDroneMode.UAM:
            return self.receiver.start()
        if self.is_alive:
            return self.receiver.state
        self._stop_event.clear()
        self._worker_error = None
        self._thread = threading.Thread(
            target=self._run,
            name="target-drone-mavlink-recv-only",
            daemon=True,
        )
        self._thread.start()
        return self.receiver.state

    def _run(self) -> None:
        try:
            self.receiver.start()
            while not self._stop_event.is_set():
                handled = self.receiver.poll_once()
                if handled == 0:
                    self._stop_event.wait(self.idle_wait_s)
        except BaseException as exc:
            self._worker_error = exc
        finally:
            try:
                self.receiver.stop(
                    drain=True,
                    drain_max_duration_s=self.drain_max_duration_s,
                )
            except BaseException as exc:
                if self._worker_error is None:
                    self._worker_error = exc

    def request_stop(self) -> None:
        self._stop_event.set()

    def stop(self, *, timeout_s: float | None = None) -> TargetDroneStreamSnapshot:
        self.request_stop()
        thread = self._thread
        if thread is None:
            return self.receiver.stop(
                drain=True,
                drain_max_duration_s=self.drain_max_duration_s,
            )
        timeout = (
            self.receiver.heartbeat_timeout_s + 1.0
            if timeout_s is None
            else float(timeout_s)
        )
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        thread.join(timeout)
        if thread.is_alive():
            raise TimeoutError("target-drone receiver did not stop before timeout")
        return self.receiver.snapshot()
