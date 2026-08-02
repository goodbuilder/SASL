"""State and provenance contracts for the StereoWIRIS GNSS workflows.

This module deliberately contains no socket, serial, ROS 2, or Qt code.  It is
the single owner of the operation mode/state contract so that a connected
correction stream cannot be mistaken for a measurement-ready GNSS solution.

Coordinate contract: inputs are WGS84 geodetic degrees/metres; the live
baseline remains ENU/NED metres in the existing geometry modules; this module
only records provenance and never converts it to ``baseline_center_frd``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
import math
import threading
import time
from typing import Any, Mapping


class GnssOperationMode(str, Enum):
    LOCAL_SURVEY_IN = "LOCAL_SURVEY_IN"
    NETWORK_ASSISTED_FIXED_BASE = "NETWORK_ASSISTED_FIXED_BASE"
    NETWORK_RTK_DIRECT = "NETWORK_RTK_DIRECT"


class GnssOperationState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONFIGURED = "CONFIGURED"
    INITIALIZING = "INITIALIZING"
    NTRIP_CONNECTING = "NTRIP_CONNECTING"
    NTRIP_ACTIVE = "NTRIP_ACTIVE"
    BASE_SURVEYING = "BASE_SURVEYING"
    BASE_AS_ROVER_FLOAT = "BASE_AS_ROVER_FLOAT"
    BASE_AS_ROVER_FIXED = "BASE_AS_ROVER_FIXED"
    FIXED_COORDINATE_COLLECTING = "FIXED_COORDINATE_COLLECTING"
    FIXED_COORDINATE_VALIDATED = "FIXED_COORDINATE_VALIDATED"
    BASE_RECONFIGURING = "BASE_RECONFIGURING"
    LOCAL_RTCM_ACTIVE = "LOCAL_RTCM_ACTIVE"
    ROVERS_CONVERGING = "ROVERS_CONVERGING"
    ALIGNMENT_COLLECTING = "ALIGNMENT_COLLECTING"
    ALIGNMENT_VALID = "ALIGNMENT_VALID"
    READY = "READY"
    MEASURING = "MEASURING"
    DEGRADED = "DEGRADED"
    FAULT = "FAULT"
    INVALIDATED = "INVALIDATED"
    STOPPED = "STOPPED"


class GgaPolicy(str, Enum):
    SYSTEM_CENTER = "SYSTEM_CENTER"
    LEFT_GNSS = "LEFT_GNSS"
    MANUAL = "MANUAL"
    PER_ROVER_SESSION = "PER_ROVER_SESSION"


class HeightType(str, Enum):
    ELLIPSOID = "ELLIPSOID"
    MSL = "MSL"
    UNKNOWN = "UNKNOWN"


_READINESS_MESSAGES: dict[str, str] = {
    "gate_disabled": "GNSS readiness gate is disabled by configuration.",
    "state_not_ready": "GNSS operation state is not READY or MEASURING.",
    "correction_missing": "No correction frame has been observed.",
    "correction_stale": "Correction frame age exceeds the configured limit.",
    "route_missing": "A required correction route is missing.",
    "route_disabled": "A required correction route is not active.",
    "route_endpoint_missing": "A required correction route has no endpoint identity.",
    "route_not_injected": "A required correction route has not injected corrections.",
    "route_stale": "A required correction route delivery is stale.",
    "route_fault": "A required correction route is faulted.",
    "rover_missing": "A required rover quality sample is missing.",
    "rover_not_fixed": "A required rover is not reporting RTK Fixed.",
    "rover_fixed_duration_missing": "A required rover has no continuous RTK Fixed timer.",
    "rover_fixed_duration_short": "A required rover has not been RTK Fixed long enough.",
    "gps_sample_missing": "A required rover GPS sample timestamp is missing.",
    "gps_sample_stale": "A required rover GPS sample is stale.",
    "eph_missing": "A required horizontal accuracy value is missing.",
    "eph_high": "A required horizontal accuracy value exceeds the configured limit.",
    "epv_missing": "A required vertical accuracy value is missing.",
    "epv_high": "A required vertical accuracy value exceeds the configured limit.",
    "left_right_timestamp_delta_missing": "Left/right rover timestamp delta is missing.",
    "left_right_timestamp_delta_high": "Left/right rover timestamp delta exceeds the configured limit.",
    "baseline_std_missing": "Baseline standard deviation is missing.",
    "baseline_std_high": "Baseline standard deviation exceeds the configured limit.",
    "drone_missing": "Drone RTK Fixed is required but no drone sample is available.",
    "drone_not_fixed": "Drone RTK Fixed is required but the drone is not fixed.",
}


def _finite(value: Any, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _nonnegative(value: Any, name: str) -> float:
    parsed = _finite(value, name)
    if parsed < 0.0:
        raise ValueError(f"{name} must be >= 0")
    return parsed


def _positive(value: Any, name: str) -> float:
    parsed = _finite(value, name)
    if parsed <= 0.0:
        raise ValueError(f"{name} must be > 0")
    return parsed


@dataclass(frozen=True)
class ReadinessConfig:
    minimum_fixed_duration_s: float = 30.0
    maximum_correction_age_s: float = 3.0
    maximum_gps_sample_age_s: float = 1.0
    maximum_horizontal_accuracy_m: float = 0.03
    maximum_vertical_accuracy_m: float = 0.05
    maximum_left_right_timestamp_delta_ms: float = 50.0
    maximum_baseline_std_m: float = 0.02

    def __post_init__(self) -> None:
        for name in (
            "minimum_fixed_duration_s",
            "maximum_correction_age_s",
            "maximum_gps_sample_age_s",
            "maximum_horizontal_accuracy_m",
            "maximum_vertical_accuracy_m",
            "maximum_left_right_timestamp_delta_ms",
            "maximum_baseline_std_m",
        ):
            object.__setattr__(self, name, _nonnegative(getattr(self, name), name))

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "ReadinessConfig":
        value = value or {}
        defaults = cls()
        return cls(
            minimum_fixed_duration_s=value.get("minimum_fixed_duration_s", defaults.minimum_fixed_duration_s),
            maximum_correction_age_s=value.get("maximum_correction_age_s", defaults.maximum_correction_age_s),
            maximum_gps_sample_age_s=value.get("maximum_gps_sample_age_s", defaults.maximum_gps_sample_age_s),
            maximum_horizontal_accuracy_m=value.get("maximum_horizontal_accuracy_m", defaults.maximum_horizontal_accuracy_m),
            maximum_vertical_accuracy_m=value.get("maximum_vertical_accuracy_m", defaults.maximum_vertical_accuracy_m),
            maximum_left_right_timestamp_delta_ms=value.get(
                "maximum_left_right_timestamp_delta_ms", defaults.maximum_left_right_timestamp_delta_ms
            ),
            maximum_baseline_std_m=value.get("maximum_baseline_std_m", defaults.maximum_baseline_std_m),
        )

    def to_mapping(self) -> dict[str, float]:
        return {
            "minimum_fixed_duration_s": self.minimum_fixed_duration_s,
            "maximum_correction_age_s": self.maximum_correction_age_s,
            "maximum_gps_sample_age_s": self.maximum_gps_sample_age_s,
            "maximum_horizontal_accuracy_m": self.maximum_horizontal_accuracy_m,
            "maximum_vertical_accuracy_m": self.maximum_vertical_accuracy_m,
            "maximum_left_right_timestamp_delta_ms": self.maximum_left_right_timestamp_delta_ms,
            "maximum_baseline_std_m": self.maximum_baseline_std_m,
        }


@dataclass(frozen=True)
class LocalBaseConfig:
    device: str = ""
    baud: int = 115200
    survey_in_min_duration_s: float = 300.0
    survey_in_accuracy_limit_m: float = 0.05

    def __post_init__(self) -> None:
        object.__setattr__(self, "device", str(self.device).strip())
        object.__setattr__(self, "baud", int(self.baud))
        if self.baud <= 0:
            raise ValueError("local_base.baud must be > 0")
        object.__setattr__(self, "survey_in_min_duration_s", _positive(self.survey_in_min_duration_s, "survey_in_min_duration_s"))
        object.__setattr__(self, "survey_in_accuracy_limit_m", _positive(self.survey_in_accuracy_limit_m, "survey_in_accuracy_limit_m"))

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "LocalBaseConfig":
        value = value or {}
        defaults = cls()
        return cls(
            device=value.get("device", defaults.device),
            baud=value.get("baud", defaults.baud),
            survey_in_min_duration_s=value.get("survey_in_min_duration_s", defaults.survey_in_min_duration_s),
            survey_in_accuracy_limit_m=value.get("survey_in_accuracy_limit_m", defaults.survey_in_accuracy_limit_m),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "device": self.device,
            "baud": self.baud,
            "survey_in_min_duration_s": self.survey_in_min_duration_s,
            "survey_in_accuracy_limit_m": self.survey_in_accuracy_limit_m,
        }


@dataclass(frozen=True)
class NetworkConfig:
    caster_host: str = ""
    caster_port: int = 2101
    mountpoint: str = ""
    username_env: str = "NTRIP_USERNAME"
    password_env: str = "NTRIP_PASSWORD"
    tls: bool = False
    gga_policy: GgaPolicy = GgaPolicy.SYSTEM_CENTER
    reconnect_interval_s: float = 5.0
    connect_timeout_s: float = 10.0
    read_timeout_s: float = 10.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "caster_host", str(self.caster_host).strip())
        object.__setattr__(self, "caster_port", int(self.caster_port))
        if self.caster_port <= 0 or self.caster_port > 65535:
            raise ValueError("network.caster_port must be between 1 and 65535")
        object.__setattr__(self, "mountpoint", str(self.mountpoint).strip().lstrip("/"))
        object.__setattr__(self, "username_env", str(self.username_env).strip())
        object.__setattr__(self, "password_env", str(self.password_env).strip())
        if not self.username_env or not self.password_env:
            raise ValueError("network credential environment variable names are required")
        object.__setattr__(self, "gga_policy", GgaPolicy(self.gga_policy))
        object.__setattr__(self, "reconnect_interval_s", _nonnegative(self.reconnect_interval_s, "reconnect_interval_s"))
        object.__setattr__(self, "connect_timeout_s", _positive(self.connect_timeout_s, "connect_timeout_s"))
        object.__setattr__(self, "read_timeout_s", _positive(self.read_timeout_s, "read_timeout_s"))

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "NetworkConfig":
        value = value or {}
        defaults = cls()
        return cls(
            caster_host=value.get("caster_host", defaults.caster_host),
            caster_port=value.get("caster_port", defaults.caster_port),
            mountpoint=value.get("mountpoint", defaults.mountpoint),
            username_env=value.get("username_env", defaults.username_env),
            password_env=value.get("password_env", defaults.password_env),
            tls=value.get("tls", defaults.tls),
            gga_policy=value.get("gga_policy", defaults.gga_policy.value),
            reconnect_interval_s=value.get("reconnect_interval_s", defaults.reconnect_interval_s),
            connect_timeout_s=value.get("connect_timeout_s", defaults.connect_timeout_s),
            read_timeout_s=value.get("read_timeout_s", defaults.read_timeout_s),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "caster_host": self.caster_host,
            "caster_port": self.caster_port,
            "mountpoint": self.mountpoint,
            "username_env": self.username_env,
            "password_env": self.password_env,
            "tls": self.tls,
            "gga_policy": self.gga_policy.value,
            "reconnect_interval_s": self.reconnect_interval_s,
            "connect_timeout_s": self.connect_timeout_s,
            "read_timeout_s": self.read_timeout_s,
        }


@dataclass(frozen=True)
class FixedBaseConfig:
    coordinate_profile_path: str = ""
    acquisition_duration_s: float = 180.0
    minimum_fixed_duration_s: float = 60.0
    minimum_fixed_ratio: float = 0.95
    maximum_horizontal_scatter_m: float = 0.03
    maximum_vertical_scatter_m: float = 0.05
    permit_profile_reuse: bool = False
    height_type: HeightType = HeightType.UNKNOWN

    def __post_init__(self) -> None:
        object.__setattr__(self, "coordinate_profile_path", str(self.coordinate_profile_path).strip())
        for name in ("acquisition_duration_s", "minimum_fixed_duration_s", "maximum_horizontal_scatter_m", "maximum_vertical_scatter_m"):
            object.__setattr__(self, name, _positive(getattr(self, name), name))
        ratio = _finite(self.minimum_fixed_ratio, "minimum_fixed_ratio")
        if not 0.0 < ratio <= 1.0:
            raise ValueError("minimum_fixed_ratio must be in (0, 1]")
        object.__setattr__(self, "minimum_fixed_ratio", ratio)
        object.__setattr__(self, "height_type", HeightType(self.height_type))

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "FixedBaseConfig":
        value = value or {}
        defaults = cls()
        return cls(
            coordinate_profile_path=value.get("coordinate_profile_path", defaults.coordinate_profile_path),
            acquisition_duration_s=value.get("acquisition_duration_s", defaults.acquisition_duration_s),
            minimum_fixed_duration_s=value.get("minimum_fixed_duration_s", defaults.minimum_fixed_duration_s),
            minimum_fixed_ratio=value.get("minimum_fixed_ratio", defaults.minimum_fixed_ratio),
            maximum_horizontal_scatter_m=value.get("maximum_horizontal_scatter_m", defaults.maximum_horizontal_scatter_m),
            maximum_vertical_scatter_m=value.get("maximum_vertical_scatter_m", defaults.maximum_vertical_scatter_m),
            permit_profile_reuse=value.get("permit_profile_reuse", defaults.permit_profile_reuse),
            height_type=value.get("height_type", defaults.height_type.value),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "coordinate_profile_path": self.coordinate_profile_path,
            "acquisition_duration_s": self.acquisition_duration_s,
            "minimum_fixed_duration_s": self.minimum_fixed_duration_s,
            "minimum_fixed_ratio": self.minimum_fixed_ratio,
            "maximum_horizontal_scatter_m": self.maximum_horizontal_scatter_m,
            "maximum_vertical_scatter_m": self.maximum_vertical_scatter_m,
            "permit_profile_reuse": self.permit_profile_reuse,
            "height_type": self.height_type.value,
        }


@dataclass(frozen=True)
class RoutingConfig:
    left_enabled: bool = True
    right_enabled: bool = True
    drone_enabled: bool = False
    field_base_enabled: bool = False

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "RoutingConfig":
        value = value or {}
        defaults = cls()
        return cls(
            left_enabled=bool(value.get("left_enabled", defaults.left_enabled)),
            right_enabled=bool(value.get("right_enabled", defaults.right_enabled)),
            drone_enabled=bool(value.get("drone_enabled", defaults.drone_enabled)),
            field_base_enabled=bool(value.get("field_base_enabled", defaults.field_base_enabled)),
        )

    def to_mapping(self) -> dict[str, bool]:
        return {
            "left_enabled": self.left_enabled,
            "right_enabled": self.right_enabled,
            "drone_enabled": self.drone_enabled,
            "field_base_enabled": self.field_base_enabled,
        }


@dataclass(frozen=True)
class GnssOperationConfig:
    operation_mode: GnssOperationMode = GnssOperationMode.LOCAL_SURVEY_IN
    require_drone_rtk_fixed: bool = False
    readiness: ReadinessConfig = field(default_factory=ReadinessConfig)
    local_base: LocalBaseConfig = field(default_factory=LocalBaseConfig)
    network: NetworkConfig = field(default_factory=NetworkConfig)
    fixed_base: FixedBaseConfig = field(default_factory=FixedBaseConfig)
    routing: RoutingConfig = field(default_factory=RoutingConfig)
    readiness_gate_enabled: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "operation_mode", GnssOperationMode(self.operation_mode))
        if not isinstance(self.readiness, ReadinessConfig):
            object.__setattr__(self, "readiness", ReadinessConfig.from_mapping(self.readiness))
        if not isinstance(self.local_base, LocalBaseConfig):
            object.__setattr__(self, "local_base", LocalBaseConfig.from_mapping(self.local_base))
        if not isinstance(self.network, NetworkConfig):
            object.__setattr__(self, "network", NetworkConfig.from_mapping(self.network))
        if not isinstance(self.fixed_base, FixedBaseConfig):
            object.__setattr__(self, "fixed_base", FixedBaseConfig.from_mapping(self.fixed_base))
        if not isinstance(self.routing, RoutingConfig):
            object.__setattr__(self, "routing", RoutingConfig.from_mapping(self.routing))

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "GnssOperationConfig":
        value = value or {}
        if not isinstance(value, Mapping):
            raise ValueError("gnss settings must be an object")
        return cls(
            operation_mode=value.get("operation_mode", cls.operation_mode.value),
            require_drone_rtk_fixed=bool(value.get("require_drone_rtk_fixed", False)),
            readiness=ReadinessConfig.from_mapping(value.get("readiness")),
            local_base=LocalBaseConfig.from_mapping(value.get("local_base")),
            network=NetworkConfig.from_mapping(value.get("network")),
            fixed_base=FixedBaseConfig.from_mapping(value.get("fixed_base")),
            routing=RoutingConfig.from_mapping(value.get("routing")),
            readiness_gate_enabled=bool(value.get("readiness_gate_enabled", True)),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "operation_mode": self.operation_mode.value,
            "require_drone_rtk_fixed": self.require_drone_rtk_fixed,
            "readiness_gate_enabled": self.readiness_gate_enabled,
            "readiness": self.readiness.to_mapping(),
            "local_base": self.local_base.to_mapping(),
            "network": self.network.to_mapping(),
            "fixed_base": self.fixed_base.to_mapping(),
            "routing": self.routing.to_mapping(),
        }


@dataclass(frozen=True)
class CorrectionSourceIdentity:
    source_kind: str
    caster_host: str = ""
    caster_port: int | None = None
    mountpoint: str = ""
    gga_policy: GgaPolicy = GgaPolicy.SYSTEM_CENTER
    stream_id: str = ""
    base_device: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "source_kind", str(self.source_kind).strip())
        object.__setattr__(self, "caster_host", str(self.caster_host).strip())
        object.__setattr__(self, "mountpoint", str(self.mountpoint).strip().lstrip("/"))
        object.__setattr__(self, "gga_policy", GgaPolicy(self.gga_policy))
        object.__setattr__(self, "stream_id", str(self.stream_id).strip())
        object.__setattr__(self, "base_device", str(self.base_device).strip())
        if self.caster_port is not None:
            port = int(self.caster_port)
            if port <= 0 or port > 65535:
                raise ValueError("caster_port must be between 1 and 65535")
            object.__setattr__(self, "caster_port", port)
        if not self.source_kind:
            raise ValueError("source_kind must not be empty")

    @property
    def stable_id(self) -> str:
        payload = {
            "source_kind": self.source_kind,
            "caster_host": self.caster_host,
            "caster_port": self.caster_port,
            "mountpoint": self.mountpoint,
            "gga_policy": self.gga_policy.value,
            "stream_id": self.stream_id,
            "base_device": self.base_device,
        }
        return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()

    def to_metadata(self) -> dict[str, Any]:
        return {
            "source_kind": self.source_kind,
            "caster_host": self.caster_host,
            "caster_port": self.caster_port,
            "mountpoint": self.mountpoint,
            "gga_policy": self.gga_policy.value,
            "stream_id": self.stream_id,
            "base_device": self.base_device,
            "correction_source_id": self.stable_id,
        }


@dataclass(frozen=True)
class RoverCorrectionStatus:
    target: str
    enabled: bool = False
    endpoint_identity: str = ""
    bytes_sent: int = 0
    packets_sent: int = 0
    last_delivery_monotonic_s: float | None = None
    queue_depth: int = 0
    drop_count: int = 0
    reconnect_count: int = 0
    sequence_error_count: int = 0
    fault_reason: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "target", str(self.target))
        object.__setattr__(self, "endpoint_identity", str(self.endpoint_identity))
        for name in ("bytes_sent", "packets_sent", "queue_depth", "drop_count", "reconnect_count", "sequence_error_count"):
            value = int(getattr(self, name))
            if value < 0:
                raise ValueError(f"{name} must be >= 0")
            object.__setattr__(self, name, value)
        if self.last_delivery_monotonic_s is not None:
            object.__setattr__(self, "last_delivery_monotonic_s", _finite(self.last_delivery_monotonic_s, "last_delivery_monotonic_s"))

    def delivery_age_s(self, now_s: float | None = None) -> float | None:
        if self.last_delivery_monotonic_s is None:
            return None
        return max(0.0, (time.monotonic() if now_s is None else float(now_s)) - self.last_delivery_monotonic_s)


@dataclass(frozen=True)
class RoverReadinessInput:
    target: str
    fix_type: int | None = None
    gps_sample_monotonic_s: float | None = None
    eph_m: float | None = None
    epv_m: float | None = None
    rtk_fixed_since_monotonic_s: float | None = None
    device_id: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "target", str(self.target))
        object.__setattr__(self, "device_id", str(self.device_id))
        if self.fix_type is not None:
            object.__setattr__(self, "fix_type", int(self.fix_type))
        for name in ("gps_sample_monotonic_s", "eph_m", "epv_m", "rtk_fixed_since_monotonic_s"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _finite(value, name))

    @property
    def rtk_fixed(self) -> bool:
        return self.fix_type == 6

    def gps_age_s(self, now_s: float) -> float | None:
        if self.gps_sample_monotonic_s is None:
            return None
        return max(0.0, float(now_s) - self.gps_sample_monotonic_s)

    def rtk_fixed_duration_s(self, now_s: float) -> float | None:
        if self.rtk_fixed_since_monotonic_s is None:
            return None
        return max(0.0, float(now_s) - self.rtk_fixed_since_monotonic_s)


@dataclass(frozen=True)
class GnssReadinessInputs:
    state: GnssOperationState
    now_monotonic_s: float
    correction_received_monotonic_s: float | None = None
    correction_source_id: str = ""
    routes: Mapping[str, RoverCorrectionStatus] = field(default_factory=dict)
    rover_samples: Mapping[str, RoverReadinessInput] = field(default_factory=dict)
    left_right_timestamp_delta_ms: float | None = None
    baseline_std_m: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "state", GnssOperationState(self.state))
        object.__setattr__(self, "now_monotonic_s", _finite(self.now_monotonic_s, "now_monotonic_s"))
        if self.correction_received_monotonic_s is not None:
            object.__setattr__(
                self,
                "correction_received_monotonic_s",
                _finite(self.correction_received_monotonic_s, "correction_received_monotonic_s"),
            )
        object.__setattr__(self, "correction_source_id", str(self.correction_source_id))
        routes: dict[str, RoverCorrectionStatus] = {}
        for key, value in self.routes.items():
            target = str(key)
            routes[target] = value if isinstance(value, RoverCorrectionStatus) else RoverCorrectionStatus(target=target, **dict(value))
        object.__setattr__(self, "routes", routes)
        samples: dict[str, RoverReadinessInput] = {}
        for key, value in self.rover_samples.items():
            target = str(key)
            samples[target] = value if isinstance(value, RoverReadinessInput) else RoverReadinessInput(target=target, **dict(value))
        object.__setattr__(self, "rover_samples", samples)
        for name in ("left_right_timestamp_delta_ms", "baseline_std_m"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _nonnegative(value, name))

    def correction_age_s(self) -> float | None:
        if self.correction_received_monotonic_s is None:
            return None
        return max(0.0, self.now_monotonic_s - self.correction_received_monotonic_s)


@dataclass(frozen=True)
class GnssReadinessSnapshot:
    ready: bool
    state: GnssOperationState
    reasons: tuple[str, ...]
    warnings: tuple[str, ...]
    mode: GnssOperationMode
    correction_source_id: str
    timestamp_s: float
    reason_messages: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "ready", bool(self.ready))
        object.__setattr__(self, "state", GnssOperationState(self.state))
        object.__setattr__(self, "mode", GnssOperationMode(self.mode))
        object.__setattr__(self, "reasons", tuple(str(value) for value in self.reasons))
        object.__setattr__(self, "warnings", tuple(str(value) for value in self.warnings))
        object.__setattr__(self, "correction_source_id", str(self.correction_source_id))
        object.__setattr__(self, "timestamp_s", _finite(self.timestamp_s, "timestamp_s"))
        object.__setattr__(self, "reason_messages", {str(k): str(v) for k, v in self.reason_messages.items()})

    def to_metadata(self) -> dict[str, Any]:
        return {
            "ready": self.ready,
            "state": self.state.value,
            "reasons": list(self.reasons),
            "warnings": list(self.warnings),
            "mode": self.mode.value,
            "correction_source_id": self.correction_source_id,
            "timestamp_s": self.timestamp_s,
            "reason_messages": dict(self.reason_messages),
        }


def _required_route_targets(config: GnssOperationConfig) -> tuple[str, ...]:
    targets: list[str] = []
    if config.routing.left_enabled:
        targets.append("left")
    if config.routing.right_enabled:
        targets.append("right")
    if config.routing.drone_enabled or config.require_drone_rtk_fixed:
        targets.append("drone")
    return tuple(targets)


def _required_rover_targets(config: GnssOperationConfig) -> tuple[str, ...]:
    targets: list[str] = []
    if config.routing.left_enabled:
        targets.append("left")
    if config.routing.right_enabled:
        targets.append("right")
    if config.require_drone_rtk_fixed:
        targets.append("drone")
    return tuple(targets)


def _append_reason(reasons: list[str], code: str) -> None:
    if code not in reasons:
        reasons.append(code)


def evaluate_gnss_readiness(config: GnssOperationConfig, inputs: GnssReadinessInputs) -> GnssReadinessSnapshot:
    """Evaluate GNSS measurement readiness from immutable config and pure inputs."""

    reasons: list[str] = []
    warnings: list[str] = []
    readiness = config.readiness
    now_s = inputs.now_monotonic_s

    if inputs.state not in {GnssOperationState.READY, GnssOperationState.MEASURING}:
        _append_reason(reasons, "state_not_ready")

    correction_age_s = inputs.correction_age_s()
    if correction_age_s is None:
        _append_reason(reasons, "correction_missing")
    elif correction_age_s > readiness.maximum_correction_age_s:
        _append_reason(reasons, "correction_stale")

    for target in _required_route_targets(config):
        route = inputs.routes.get(target)
        if route is None:
            _append_reason(reasons, "route_missing")
            continue
        if not route.enabled:
            _append_reason(reasons, "route_disabled")
        if not route.endpoint_identity:
            _append_reason(reasons, "route_endpoint_missing")
        if route.bytes_sent <= 0 or route.packets_sent <= 0:
            _append_reason(reasons, "route_not_injected")
        delivery_age_s = route.delivery_age_s(now_s)
        if delivery_age_s is None:
            _append_reason(reasons, "route_not_injected")
        elif delivery_age_s > readiness.maximum_correction_age_s:
            _append_reason(reasons, "route_stale")
        if route.fault_reason:
            _append_reason(reasons, "route_fault")

    for target in _required_rover_targets(config):
        sample = inputs.rover_samples.get(target)
        if sample is None:
            _append_reason(reasons, "drone_missing" if target == "drone" else "rover_missing")
            continue
        if not sample.rtk_fixed:
            _append_reason(reasons, "drone_not_fixed" if target == "drone" else "rover_not_fixed")
        fixed_duration_s = sample.rtk_fixed_duration_s(now_s)
        if fixed_duration_s is None:
            _append_reason(reasons, "rover_fixed_duration_missing")
        elif fixed_duration_s < readiness.minimum_fixed_duration_s:
            _append_reason(reasons, "rover_fixed_duration_short")
        gps_age_s = sample.gps_age_s(now_s)
        if gps_age_s is None:
            _append_reason(reasons, "gps_sample_missing")
        elif gps_age_s > readiness.maximum_gps_sample_age_s:
            _append_reason(reasons, "gps_sample_stale")
        if sample.eph_m is None:
            _append_reason(reasons, "eph_missing")
        elif sample.eph_m > readiness.maximum_horizontal_accuracy_m:
            _append_reason(reasons, "eph_high")
        if sample.epv_m is None:
            _append_reason(reasons, "epv_missing")
        elif sample.epv_m > readiness.maximum_vertical_accuracy_m:
            _append_reason(reasons, "epv_high")

    if config.routing.left_enabled and config.routing.right_enabled:
        if inputs.left_right_timestamp_delta_ms is None:
            _append_reason(reasons, "left_right_timestamp_delta_missing")
        elif inputs.left_right_timestamp_delta_ms > readiness.maximum_left_right_timestamp_delta_ms:
            _append_reason(reasons, "left_right_timestamp_delta_high")
        if inputs.baseline_std_m is None:
            _append_reason(reasons, "baseline_std_missing")
        elif inputs.baseline_std_m > readiness.maximum_baseline_std_m:
            _append_reason(reasons, "baseline_std_high")

    if not config.readiness_gate_enabled:
        warnings.append("gate_disabled")

    codes = tuple(reasons)
    warning_codes = tuple(warnings)
    reason_messages = {code: _READINESS_MESSAGES[code] for code in (*codes, *warning_codes) if code in _READINESS_MESSAGES}
    return GnssReadinessSnapshot(
        ready=not codes,
        state=inputs.state,
        reasons=codes,
        warnings=warning_codes,
        mode=config.operation_mode,
        correction_source_id=inputs.correction_source_id,
        timestamp_s=now_s,
        reason_messages=reason_messages,
    )


@dataclass(frozen=True)
class MeasurementGnssContract:
    gnss_operation_mode: GnssOperationMode
    correction_source_id: str
    caster_identity: str
    mountpoint: str
    gga_policy: GgaPolicy
    base_coordinate_profile_hash: str
    baseline_profile_hash: str
    device_ids: tuple[tuple[str, str], ...]
    route_endpoints: tuple[tuple[str, str], ...]
    datum: str
    height_type: HeightType

    def __post_init__(self) -> None:
        object.__setattr__(self, "gnss_operation_mode", GnssOperationMode(self.gnss_operation_mode))
        object.__setattr__(self, "gga_policy", GgaPolicy(self.gga_policy))
        object.__setattr__(self, "height_type", HeightType(self.height_type))
        for name in (
            "correction_source_id",
            "caster_identity",
            "mountpoint",
            "base_coordinate_profile_hash",
            "baseline_profile_hash",
            "datum",
        ):
            object.__setattr__(self, name, str(getattr(self, name)))
        object.__setattr__(self, "device_ids", tuple((str(k), str(v)) for k, v in self.device_ids))
        object.__setattr__(self, "route_endpoints", tuple((str(k), str(v)) for k, v in self.route_endpoints))

    @property
    def stable_id(self) -> str:
        return hashlib.sha256(
            json.dumps(self._canonical_payload(), sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()

    def _canonical_payload(self) -> dict[str, Any]:
        return {
            "gnss_operation_mode": self.gnss_operation_mode.value,
            "correction_source_id": self.correction_source_id,
            "caster_identity": self.caster_identity,
            "mountpoint": self.mountpoint,
            "gga_policy": self.gga_policy.value,
            "base_coordinate_profile_hash": self.base_coordinate_profile_hash,
            "baseline_profile_hash": self.baseline_profile_hash,
            "device_ids": sorted(self.device_ids),
            "route_endpoints": sorted(self.route_endpoints),
            "datum": self.datum,
            "height_type": self.height_type.value,
        }

    def to_metadata(self) -> dict[str, Any]:
        return {
            "gnss_contract_id": self.stable_id,
            "gnss_operation_mode": self.gnss_operation_mode.value,
            "correction_source_id": self.correction_source_id,
            "caster_identity": self.caster_identity,
            "mountpoint": self.mountpoint,
            "gga_policy": self.gga_policy.value,
            "base_coordinate_profile_hash": self.base_coordinate_profile_hash,
            "baseline_profile_hash": self.baseline_profile_hash,
            "gnss_device_ids": dict(self.device_ids),
            "gnss_route_endpoints": dict(self.route_endpoints),
            "datum": self.datum,
            "height_type": self.height_type.value,
        }


def build_measurement_gnss_contract(
    config: GnssOperationConfig,
    source_identity: CorrectionSourceIdentity,
    readiness: GnssReadinessSnapshot,
    *,
    base_coordinate_profile_hash: str = "",
    baseline_profile_hash: str = "",
    device_ids: Mapping[str, str] | None = None,
    route_endpoints: Mapping[str, str] | None = None,
    datum: str = "WGS84",
) -> MeasurementGnssContract:
    """Build a deterministic measurement contract from pure operation metadata."""

    if readiness.correction_source_id and readiness.correction_source_id != source_identity.stable_id:
        raise ValueError("readiness source identity does not match correction source")
    caster_identity = ""
    if source_identity.caster_host or source_identity.caster_port is not None:
        caster_identity = f"{source_identity.caster_host}:{source_identity.caster_port or ''}"
    return MeasurementGnssContract(
        gnss_operation_mode=config.operation_mode,
        correction_source_id=source_identity.stable_id,
        caster_identity=caster_identity,
        mountpoint=source_identity.mountpoint or config.network.mountpoint,
        gga_policy=source_identity.gga_policy,
        base_coordinate_profile_hash=str(base_coordinate_profile_hash),
        baseline_profile_hash=str(baseline_profile_hash),
        device_ids=tuple(sorted((device_ids or {}).items())),
        route_endpoints=tuple(sorted((route_endpoints or {}).items())),
        datum=str(datum),
        height_type=config.fixed_base.height_type,
    )


_ALLOWED_TRANSITIONS: dict[GnssOperationState, frozenset[GnssOperationState]] = {
    GnssOperationState.DISCONNECTED: frozenset({GnssOperationState.CONFIGURED, GnssOperationState.STOPPED}),
    GnssOperationState.CONFIGURED: frozenset({GnssOperationState.INITIALIZING, GnssOperationState.DISCONNECTED, GnssOperationState.STOPPED}),
    GnssOperationState.INITIALIZING: frozenset({GnssOperationState.NTRIP_CONNECTING, GnssOperationState.BASE_SURVEYING, GnssOperationState.NTRIP_ACTIVE, GnssOperationState.FAULT, GnssOperationState.STOPPED}),
    GnssOperationState.NTRIP_CONNECTING: frozenset({GnssOperationState.NTRIP_ACTIVE, GnssOperationState.FAULT, GnssOperationState.STOPPED}),
    GnssOperationState.NTRIP_ACTIVE: frozenset({GnssOperationState.BASE_AS_ROVER_FLOAT, GnssOperationState.ROVERS_CONVERGING, GnssOperationState.FAULT, GnssOperationState.DEGRADED, GnssOperationState.STOPPED}),
    GnssOperationState.BASE_SURVEYING: frozenset({GnssOperationState.LOCAL_RTCM_ACTIVE, GnssOperationState.FAULT, GnssOperationState.DEGRADED, GnssOperationState.STOPPED}),
    GnssOperationState.BASE_AS_ROVER_FLOAT: frozenset({GnssOperationState.BASE_AS_ROVER_FIXED, GnssOperationState.FAULT, GnssOperationState.DEGRADED, GnssOperationState.STOPPED}),
    GnssOperationState.BASE_AS_ROVER_FIXED: frozenset({GnssOperationState.FIXED_COORDINATE_COLLECTING, GnssOperationState.FAULT, GnssOperationState.DEGRADED, GnssOperationState.STOPPED}),
    GnssOperationState.FIXED_COORDINATE_COLLECTING: frozenset({GnssOperationState.FIXED_COORDINATE_VALIDATED, GnssOperationState.BASE_AS_ROVER_FLOAT, GnssOperationState.FAULT, GnssOperationState.DEGRADED, GnssOperationState.STOPPED}),
    GnssOperationState.FIXED_COORDINATE_VALIDATED: frozenset({GnssOperationState.BASE_RECONFIGURING, GnssOperationState.FAULT, GnssOperationState.STOPPED}),
    GnssOperationState.BASE_RECONFIGURING: frozenset({GnssOperationState.LOCAL_RTCM_ACTIVE, GnssOperationState.FAULT, GnssOperationState.STOPPED}),
    GnssOperationState.LOCAL_RTCM_ACTIVE: frozenset({GnssOperationState.ROVERS_CONVERGING, GnssOperationState.FAULT, GnssOperationState.DEGRADED, GnssOperationState.STOPPED}),
    GnssOperationState.ROVERS_CONVERGING: frozenset({GnssOperationState.ALIGNMENT_COLLECTING, GnssOperationState.FAULT, GnssOperationState.DEGRADED, GnssOperationState.STOPPED}),
    GnssOperationState.ALIGNMENT_COLLECTING: frozenset({GnssOperationState.ALIGNMENT_VALID, GnssOperationState.FAULT, GnssOperationState.DEGRADED, GnssOperationState.STOPPED}),
    GnssOperationState.ALIGNMENT_VALID: frozenset({GnssOperationState.READY, GnssOperationState.FAULT, GnssOperationState.INVALIDATED, GnssOperationState.STOPPED}),
    GnssOperationState.READY: frozenset({GnssOperationState.MEASURING, GnssOperationState.INVALIDATED, GnssOperationState.DEGRADED, GnssOperationState.FAULT, GnssOperationState.STOPPED}),
    GnssOperationState.MEASURING: frozenset({GnssOperationState.INVALIDATED, GnssOperationState.DEGRADED, GnssOperationState.FAULT, GnssOperationState.STOPPED}),
    GnssOperationState.DEGRADED: frozenset({GnssOperationState.INITIALIZING, GnssOperationState.READY, GnssOperationState.FAULT, GnssOperationState.STOPPED}),
    GnssOperationState.FAULT: frozenset({GnssOperationState.INITIALIZING, GnssOperationState.CONFIGURED, GnssOperationState.STOPPED}),
    GnssOperationState.INVALIDATED: frozenset({GnssOperationState.INITIALIZING, GnssOperationState.CONFIGURED, GnssOperationState.STOPPED}),
    GnssOperationState.STOPPED: frozenset({GnssOperationState.CONFIGURED, GnssOperationState.INITIALIZING, GnssOperationState.DISCONNECTED}),
}


class GnssOperationManager:
    """Thread-safe owner of operation state and measurement-time contracts."""

    def __init__(self, config: GnssOperationConfig | None = None) -> None:
        self._lock = threading.RLock()
        self._config = config or GnssOperationConfig()
        self._state = GnssOperationState.DISCONNECTED
        self._source_identity: CorrectionSourceIdentity | None = None
        self._readiness: GnssReadinessSnapshot | None = None
        self._last_transition_reason = ""
        self._invalidation_reason = ""
        self._contract: MeasurementGnssContract | None = None
        self._history: list[tuple[float, GnssOperationState, str]] = []

    @property
    def config(self) -> GnssOperationConfig:
        with self._lock:
            return self._config

    @property
    def state(self) -> GnssOperationState:
        with self._lock:
            return self._state

    @property
    def source_identity(self) -> CorrectionSourceIdentity | None:
        with self._lock:
            return self._source_identity

    @property
    def measurement_contract(self) -> MeasurementGnssContract | None:
        with self._lock:
            return self._contract

    @property
    def invalidation_reason(self) -> str:
        with self._lock:
            return self._invalidation_reason

    @property
    def history(self) -> tuple[tuple[float, GnssOperationState, str], ...]:
        with self._lock:
            return tuple(self._history)

    def configure(self, config: GnssOperationConfig) -> None:
        with self._lock:
            if self._state == GnssOperationState.MEASURING:
                raise RuntimeError("GNSS operation configuration is immutable while measuring")
            self._config = config
            if self._state in {GnssOperationState.DISCONNECTED, GnssOperationState.STOPPED, GnssOperationState.FAULT, GnssOperationState.INVALIDATED}:
                self._transition_unlocked(GnssOperationState.CONFIGURED, "configuration loaded")

    def set_source_identity(self, identity: CorrectionSourceIdentity | None) -> None:
        with self._lock:
            old_id = None if self._source_identity is None else self._source_identity.stable_id
            new_id = None if identity is None else identity.stable_id
            self._source_identity = identity
            if self._state == GnssOperationState.MEASURING and old_id != new_id:
                self._invalidate_unlocked(f"correction source changed: {old_id or 'none'} -> {new_id or 'none'}")

    def transition(self, state: GnssOperationState, reason: str = "") -> None:
        with self._lock:
            self._transition_unlocked(GnssOperationState(state), reason)

    def _transition_unlocked(self, state: GnssOperationState, reason: str) -> None:
        if state == self._state:
            self._last_transition_reason = str(reason)
            return
        if state not in _ALLOWED_TRANSITIONS.get(self._state, frozenset()):
            raise ValueError(f"invalid GNSS state transition: {self._state.value} -> {state.value}")
        self._state = state
        self._last_transition_reason = str(reason)
        self._history.append((time.monotonic(), state, str(reason)))

    def update_readiness(self, snapshot: GnssReadinessSnapshot) -> None:
        with self._lock:
            self._readiness = snapshot

    def readiness(self) -> GnssReadinessSnapshot | None:
        with self._lock:
            return self._readiness

    def begin_measurement(self, contract: MeasurementGnssContract | None, snapshot: GnssReadinessSnapshot) -> None:
        with self._lock:
            if self._state != GnssOperationState.READY:
                raise RuntimeError(f"measurement blocked: GNSS state is {self._state.value}")
            if not snapshot.ready and self._config.readiness_gate_enabled:
                raise RuntimeError("measurement blocked: GNSS readiness is false")
            if not snapshot.ready and not self._config.readiness_gate_enabled:
                self._contract = None
                self._invalidation_reason = ""
                self._readiness = snapshot
                self._transition_unlocked(GnssOperationState.MEASURING, "measurement started with GNSS gate disabled")
                return
            if contract is None:
                raise RuntimeError("measurement blocked: GNSS contract is required")
            source_id = None if self._source_identity is None else self._source_identity.stable_id
            if source_id and source_id != contract.correction_source_id:
                raise RuntimeError("measurement blocked: correction source identity mismatch")
            self._contract = contract
            self._invalidation_reason = ""
            self._readiness = snapshot
            self._transition_unlocked(GnssOperationState.MEASURING, "measurement contract captured")

    def capture_measurement_contract(
        self,
        readiness: GnssReadinessSnapshot,
        *,
        base_coordinate_profile_hash: str = "",
        baseline_profile_hash: str = "",
        device_ids: Mapping[str, str] | None = None,
        route_endpoints: Mapping[str, str] | None = None,
        datum: str = "WGS84",
    ) -> MeasurementGnssContract | None:
        with self._lock:
            if self._source_identity is None:
                raise RuntimeError("measurement blocked: correction source identity is missing")
            if not readiness.ready and not self._config.readiness_gate_enabled:
                return None
            return build_measurement_gnss_contract(
                self._config,
                self._source_identity,
                readiness,
                base_coordinate_profile_hash=base_coordinate_profile_hash,
                baseline_profile_hash=baseline_profile_hash,
                device_ids=device_ids,
                route_endpoints=route_endpoints,
                datum=datum,
            )

    def validate_measurement_contract(self, contract: MeasurementGnssContract | None, snapshot: GnssReadinessSnapshot) -> bool:
        with self._lock:
            if self._state != GnssOperationState.MEASURING:
                raise RuntimeError(f"measurement blocked: GNSS state is {self._state.value}")
            if not snapshot.ready and self._config.readiness_gate_enabled:
                self._invalidate_unlocked("GNSS readiness changed during measurement")
                return False
            if not self._config.readiness_gate_enabled and not snapshot.ready:
                return self._contract is None and contract is None
            if self._contract is None or contract is None:
                self._invalidate_unlocked("GNSS measurement contract missing")
                return False
            if self._contract.stable_id != contract.stable_id:
                self._invalidate_unlocked("GNSS measurement contract changed")
                return False
            source_id = None if self._source_identity is None else self._source_identity.stable_id
            if source_id and source_id != contract.correction_source_id:
                self._invalidate_unlocked("GNSS correction source changed during measurement")
                return False
            self._readiness = snapshot
            return True

    def invalidate(self, reason: str) -> None:
        with self._lock:
            self._invalidate_unlocked(reason)

    def _invalidate_unlocked(self, reason: str) -> None:
        self._invalidation_reason = str(reason)
        if self._state != GnssOperationState.INVALIDATED:
            if GnssOperationState.INVALIDATED in _ALLOWED_TRANSITIONS.get(self._state, frozenset()):
                self._transition_unlocked(GnssOperationState.INVALIDATED, self._invalidation_reason)
            else:
                self._state = GnssOperationState.INVALIDATED
                self._history.append((time.monotonic(), self._state, self._invalidation_reason))

    def stop(self) -> None:
        with self._lock:
            if self._state != GnssOperationState.STOPPED:
                if GnssOperationState.STOPPED in _ALLOWED_TRANSITIONS.get(self._state, frozenset()):
                    self._transition_unlocked(GnssOperationState.STOPPED, "operation stopped")
                else:
                    self._state = GnssOperationState.STOPPED
                    self._history.append((time.monotonic(), self._state, "operation stopped"))

    def metadata(self) -> dict[str, Any]:
        with self._lock:
            source = None if self._source_identity is None else self._source_identity.to_metadata()
            return {
                "gnss_operation_mode": self._config.operation_mode.value,
                "gnss_operation_state": self._state.value,
                "correction_source": source,
                "correction_source_id": None if source is None else source["correction_source_id"],
                "invalidation_reason": self._invalidation_reason,
                "measurement_contract": None if self._contract is None else self._contract.to_metadata(),
                "gnss_contract_id": None if self._contract is None else self._contract.stable_id,
                "readiness": None if self._readiness is None else self._readiness.to_metadata(),
            }
