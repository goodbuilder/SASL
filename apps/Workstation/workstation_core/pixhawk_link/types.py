"""Pixhawk telemetry values with their existing fields and serialization identity.

Attitude is q(wxyz), body FRD -> world NED; Euler displays use degrees.
Receipt timestamps are host monotonic seconds, never PX4 boot or GPS UTC time.
MAG x/y/z retain the producer sensor axes and topic units (scaled IMU mG,
CAN/sensor_mag G); they are not world positions or camera RDF coordinates.
GNSS relative headings use radians, lengths/accuracies use metres.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EulerAttitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class CameraExtrinsicState:
    camera: str
    namespace: str
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    timestamp_s: float
    topic: str
    q: tuple[float, float, float, float]
    rollspeed_rad_s: float | None = None
    pitchspeed_rad_s: float | None = None
    yawspeed_rad_s: float | None = None
    px4_timestamp_s: float | None = None
    px4_timestamp_us: int | None = None
    px4_timestamp_sample_us: int | None = None
    delta_q_reset: tuple[float, float, float, float] | None = None
    quat_reset_counter: int | None = None
    mavlink_source_system: int | None = None
    mavlink_source_component: int | None = None
    mavlink_sequence: int | None = None
    mavlink_message_id: int | None = None


@dataclass(frozen=True)
class RadioStatusState:
    timestamp_s: float
    rssi: int | None
    remote_rssi: int | None
    noise: int | None
    remote_noise: int | None
    txbuf_pct: int | None
    rxerrors: int | None
    fixed: int | None


@dataclass(frozen=True)
class MagSensorState:
    camera: str
    namespace: str
    sensor_slot: str
    timestamp_s: float
    topic: str
    device_id: int | None = None
    node_id: int | None = None
    x: float | None = None
    y: float | None = None
    z: float | None = None
    magnitude: float | None = None
    temperature_c: float | None = None
    error_count: int | None = None
    px4_timestamp_s: float | None = None
    accel_x: float | None = None
    accel_y: float | None = None
    accel_z: float | None = None
    gyro_x: float | None = None
    gyro_y: float | None = None
    gyro_z: float | None = None
    time_boot_ms: int | None = None
    mavlink_source_system: int | None = None
    mavlink_source_component: int | None = None
    mavlink_sequence: int | None = None
    mavlink_message_id: int | None = None


@dataclass(frozen=True)
class GpsSensorState:
    camera: str
    namespace: str
    sensor_slot: str
    timestamp_s: float
    topic: str
    device_id: int | None = None
    fix_type: int | None = None
    satellites_used: int | None = None
    eph_m: float | None = None
    epv_m: float | None = None
    hdop: float | None = None
    vdop: float | None = None
    px4_timestamp_s: float | None = None
    px4_timestamp_us: int | None = None
    px4_timestamp_sample_us: int | None = None
    gps_utc_us: int | None = None
    gps_timestamp_time_relative_us: int | None = None
    mavlink_source_system: int | None = None
    mavlink_source_component: int | None = None
    mavlink_sequence: int | None = None
    mavlink_message_id: int | None = None


@dataclass(frozen=True)
class GnssRelativeState:
    camera: str
    namespace: str
    timestamp_s: float
    topic: str
    device_id: int | None = None
    heading_rad: float | None = None
    heading_accuracy_rad: float | None = None
    position_length_m: float | None = None
    accuracy_length_m: float | None = None
    gnss_fix_ok: bool | None = None
    differential_solution: bool | None = None
    relative_position_valid: bool | None = None
    carrier_solution_floating: bool | None = None
    carrier_solution_fixed: bool | None = None
    moving_base_mode: bool | None = None
    heading_valid: bool | None = None
    px4_timestamp_s: float | None = None
    px4_timestamp_us: int | None = None
    px4_timestamp_sample_us: int | None = None
    gps_utc_us: int | None = None
    mavlink_source_system: int | None = None
    mavlink_source_component: int | None = None
    mavlink_sequence: int | None = None
    mavlink_message_id: int | None = None


@dataclass(frozen=True)
class CustomTopicSample:
    key: str
    topic: str
    field_path: str
    value_text: str
    numeric_value: float | None
    timestamp_s: float


@dataclass(frozen=True)
class UsbMavlinkIdentity:
    device: str
    resolved_device: str
    baud: int
    system_id: int
    component_id: int
    observed_monotonic_s: float

    def to_mapping(self) -> dict[str, object]:
        return {
            "device": self.device,
            "resolved_device": self.resolved_device,
            "baud": self.baud,
            "system_id": self.system_id,
            "component_id": self.component_id,
            "observed_monotonic_s": self.observed_monotonic_s,
        }


@dataclass(frozen=True)
class MavlinkShellCommandResult:
    command: str
    output: str
    flag_label: str


# Preserve the existing pickle GLOBAL path during the staged module migration.
# This string does not import the Qt subscriber; the facade reexports these
# exact classes so older IPC/serialized objects keep their identity.
for _legacy_type in (
    EulerAttitude,
    CameraExtrinsicState,
    RadioStatusState,
    MagSensorState,
    GpsSensorState,
    GnssRelativeState,
    CustomTopicSample,
    UsbMavlinkIdentity,
    MavlinkShellCommandResult,
):
    _legacy_type.__module__ = "Workstation.workstation_core.pixhawk_attitude"
del _legacy_type
