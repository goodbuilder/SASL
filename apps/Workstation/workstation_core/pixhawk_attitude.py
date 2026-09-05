from __future__ import annotations

import errno
import fcntl
import glob
import hashlib
import io
from ipaddress import IPv4Address
import math
import os
import re
import socket
import struct
import threading
import time
from contextlib import contextmanager, nullcontext
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable

from Workstation.workstation_core.error_throttle import (
    DEFAULT_ERROR_LOG_THROTTLE_S,
    should_emit_throttled_error,
)
from Workstation.workstation_core.mavlink_liveness import (
    MavlinkLivenessState,
    MavlinkSideHealthSnapshot,
    MavlinkSideLiveness,
)
from Workstation.workstation_core.pixhawk_radio_policy import (
    RadioStreamPolicy,
    RadioStreamTier,
    radio_stream_requests,
)
from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
)
from Workstation.workstation_core.rtk_geometry import GpsPositionState, gps_state_from_message, gps_topic

from PyQt5 import QtCore


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


def parse_px4_sensor_mag_shell_output(
    output: str,
    *,
    camera: str,
    timestamp_s: float | None = None,
    known_only: bool = True,
) -> tuple[MagSensorState, ...]:
    """Parse PX4 ``listener sensor_mag`` output for known CAN mags.

    PX4 prints producer values in sensor axes (FRD).  Shell output has no
    transport timestamp, so ``timestamp_s`` is the local receipt time used
    only for freshness; the PX4 ``timestamp`` field is retained separately.
    """
    if not output:
        return ()
    received_s = time.monotonic() if timestamp_s is None else float(timestamp_s)
    blocks = re.split(r"(?=TOPIC:\s*sensor_mag\b)", str(output), flags=re.IGNORECASE)
    states: list[MagSensorState] = []
    for block in blocks:
        if "TOPIC:" not in block or "sensor_mag" not in block.lower():
            continue

        def number(field: str) -> float | None:
            match = re.search(rf"^\s*{re.escape(field)}\s*:\s*([-+0-9.eE]+)", block, re.MULTILINE)
            return None if match is None else float(match.group(1))

        device_match = re.search(r"^\s*device_id\s*:\s*(-?\d+)", block, re.MULTILINE)
        if device_match is None:
            continue
        device_id = int(device_match.group(1))
        instance_match = re.search(r"TOPIC:\s*sensor_mag\s+instance\s+(\d+)", block, re.IGNORECASE)
        if device_id in RM3100_PRIMARY_IDS:
            sensor_slot = "CAN1"
            node_id = 0x7D if device_id == RM3100_PRIMARY_IDS[0] else 0x7C
        elif device_id in RM3100_BACKUP_IDS:
            sensor_slot = "CAN2"
            node_id = 0x7B if device_id == RM3100_BACKUP_IDS[0] else None
        else:
            if known_only:
                continue
            sensor_slot = f"MAG{instance_match.group(1) if instance_match else '?'}"
            node_id = None
        x, y, z = number("x"), number("y"), number("z")
        timestamp_us = number("timestamp")
        states.append(
            MagSensorState(
                camera=camera,
                namespace="px4_shell",
                sensor_slot=sensor_slot,
                timestamp_s=received_s,
                topic=f"px4-shell/sensor_mag/{instance_match.group(1) if instance_match else '?'}",
                device_id=device_id,
                node_id=node_id,
                x=x,
                y=y,
                z=z,
                magnitude=(math.sqrt(x * x + y * y + z * z) if None not in (x, y, z) else None),
                px4_timestamp_s=(timestamp_us * 1e-6 if timestamp_us is not None else None),
            )
        )
    return tuple(states)


def parse_px4_vehicle_magnetometer_shell_device_id(output: str) -> int | None:
    """Parse the selected ``vehicle_magnetometer`` device ID from PX4 Shell.

    Stable PX4 exposes the selected magnetometer through the uORB
    ``vehicle_magnetometer`` topic, not through the standard SCALED_IMU
    MAVLink streams.  The device ID is the only field needed here: raw
    per-instance vectors come from ``sensor_mag`` and the existing yaw
    projection owns the exact-identity/fresh-ATTITUDE gate.
    """
    if not output:
        return None
    blocks = re.split(
        r"(?=TOPIC:\s*vehicle_magnetometer\b)",
        str(output),
        flags=re.IGNORECASE,
    )
    candidates: list[tuple[int, int, int]] = []
    for block_index, block in enumerate(blocks):
        if "TOPIC:" not in block or "vehicle_magnetometer" not in block.lower():
            continue
        device_match = re.search(r"^\s*device_id\s*:\s*(-?\d+)", block, re.MULTILINE)
        if device_match is None:
            continue
        timestamp_match = re.search(r"^\s*timestamp\s*:\s*(-?\d+)", block, re.MULTILINE)
        timestamp_us = int(timestamp_match.group(1)) if timestamp_match is not None else -1
        candidates.append((timestamp_us, block_index, int(device_match.group(1))))
    if not candidates:
        return None
    # Radio SERIAL_CONTROL can deliver a queued older listener block before
    # the current one. PX4's uORB timestamp is the independent ordering key;
    # block order is only a tie-breaker for samples without distinct time.
    return max(candidates, key=lambda item: (item[0], item[1]))[2]


def magnetometer_value_units(topic: object) -> str:
    """Return the unit carried by a raw magnetometer telemetry topic."""
    topic_text = str(topic)
    if topic_text.startswith("mavlink/SCALED_IMU"):
        return "mG"
    if "sensor_mag" in topic_text or "vehicle_magnetometer" in topic_text:
        return "G"
    if "CAN_MAG" in topic_text.upper():
        return "G"
    return ""


def magnetometer_value_to_gauss(value: object, topic: object) -> float | None:
    """Convert a raw magnetometer magnitude to the canonical G unit."""
    if value is None:
        return None
    magnitude = float(value)
    return magnitude * 1e-3 if magnetometer_value_units(topic) == "mG" else magnitude


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


def quaternion_to_euler_deg(q: list[float] | tuple[float, float, float, float]) -> EulerAttitude:
    w, x, y, z = (float(value) for value in q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return EulerAttitude(roll, pitch, yaw)


def euler_rad_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def camera_extrinsic_from_euler_rad(
    *,
    camera: str,
    namespace: str,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    timestamp_s: float | None = None,
    topic: str = "mavlink/ATTITUDE",
) -> CameraExtrinsicState:
    q = euler_rad_to_quaternion(roll_rad, pitch_rad, yaw_rad)
    attitude = quaternion_to_euler_deg(q)
    return CameraExtrinsicState(
        camera=camera,
        namespace=namespace.strip(),
        roll_deg=attitude.roll_deg,
        pitch_deg=attitude.pitch_deg,
        yaw_deg=attitude.yaw_deg,
        timestamp_s=time.monotonic() if timestamp_s is None else float(timestamp_s),
        topic=topic,
        q=q,
        px4_timestamp_s=None,
    )




MAVLINK_MESSAGE_IDS = {
    "ATTITUDE": 30,
    "SYS_STATUS": 1,
    "GPS_RAW_INT": 24,
    "RAW_IMU": 27,
    "LOCAL_POSITION_NED": 32,
    "GLOBAL_POSITION_INT": 33,
    "VFR_HUD": 74,
    "GPS_RTK": 127,
    "GPS2_RAW": 124,
    "BATTERY_STATUS": 147,
    "GPS2_RTK": 128,
    "SCALED_IMU": 26,
    "SCALED_IMU2": 116,
    "SCALED_IMU3": 129,
    "DEBUG_VECT": 250,
    "SASL_CAN_MAG": 180,
}
MAVLINK_MAG_STREAMS = (
    ("CAN1", "IMU stream 1", "SCALED_IMU"),
    ("CAN2", "IMU stream 2", "SCALED_IMU2"),
    ("CAN3", "IMU stream 3", "SCALED_IMU3"),
)
MAVLINK_UI_IMU_STREAM_NAMES = tuple(
    message_name for _sensor_slot, _imu_label, message_name in MAVLINK_MAG_STREAMS
)
MAVLINK_CAN_MAG_MESSAGE_TYPES = frozenset(("CAN_MAG", "CAN_MAGNETOMETER", "SASL_CAN_MAG"))
MAVLINK_MAG_SLOT_BY_MESSAGE = {
    message_name: sensor_slot
    for sensor_slot, _imu_label, message_name in MAVLINK_MAG_STREAMS
}
MAVLINK_STOCK_STREAM_REQUESTS = (
    ("ATTITUDE", 30.0),
    ("SYS_STATUS", 1.0),
    ("GPS_RAW_INT", 5.0),
    ("GPS2_RAW", 5.0),
    ("GPS_RTK", 5.0),
    ("GPS2_RTK", 5.0),
    ("SCALED_IMU", 20.0),
    ("SCALED_IMU2", 20.0),
    ("SCALED_IMU3", 20.0),
    ("DEBUG_VECT", 20.0),
    ("RAW_IMU", 20.0),
    ("LOCAL_POSITION_NED", 10.0),
    ("GLOBAL_POSITION_INT", 10.0),
    ("VFR_HUD", 5.0),
    ("BATTERY_STATUS", 2.0),
)
MAVLINK_STOCK_STREAM_CONSUMERS = {
    "ATTITUDE": ("extrinsic/status/measurement",),
    "SYS_STATUS": ("optional timeline", "battery evidence"),
    "GPS_RAW_INT": ("GNSS position/source selection/measurement",),
    "GPS2_RAW": ("GNSS position/source selection/measurement",),
    "GPS_RTK": ("GNSS relative baseline",),
    "GPS2_RTK": ("GNSS relative baseline",),
    "SCALED_IMU": ("mag sensor presentation/evidence",),
    "SCALED_IMU2": ("mag sensor presentation/evidence",),
    "SCALED_IMU3": ("mag sensor presentation/evidence",),
    "DEBUG_VECT": ("CAN-mag compatibility presentation/evidence",),
    "RAW_IMU": ("optional timeline/session evidence",),
    "LOCAL_POSITION_NED": ("optional timeline/session evidence",),
    "GLOBAL_POSITION_INT": ("optional timeline/session evidence",),
    "VFR_HUD": ("optional timeline/session evidence",),
    "BATTERY_STATUS": ("optional timeline/battery evidence",),
}
# Compatibility name for existing callers; the default is now explicitly the
# Stock profile and contains no private SASL_CAN_MAG request.
MAVLINK_RM3100_STREAM_REQUESTS = MAVLINK_STOCK_STREAM_REQUESTS
MAVLINK_MAX_MESSAGES_PER_TICK = 256
MAVLINK_MAX_PROCESS_TIME_S = 0.020
MAVLINK_RECEIVE_ERROR_INTERVAL_S = DEFAULT_ERROR_LOG_THROTTLE_S
MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS = 3
MAVLINK_STREAM_REQUEST_RETRY_S = 2.0
MAVLINK_DUPLICATE_TTL_S = 0.250
MAVLINK_DUPLICATE_CACHE_SIZE = 512
MAVLINK_ATTITUDE_STALE_S = 2.0
MAVLINK_ATTITUDE_RECOVERY_COOLDOWN_S = 10.0
MAVLINK_IMU_STREAM_STALE_S = 1.0
MAVLINK_IMU_STREAM_RECOVERY_COOLDOWN_S = 10.0
MAVLINK_GCS_SOURCE_SYSTEM = 255
MAVLINK_GCS_SOURCE_COMPONENT = 190
MAVLINK_GCS_HEARTBEAT_INTERVAL_S = 1.0
MAVLINK_COMMAND_RECEIVE_POLL_S = 0.05
MAVLINK_RTCM_STATUS_INTERVAL_S = 5.0
MAVLINK_EXPECTED_SYSTEM_IDS = {"left": 1, "right": 2, "drone": 3}
MAVLINK_BOOT_TIME_UINT32_MODULUS = 1 << 32
MAVLINK_BOOT_TIME_UINT32_HALF_RANGE = 1 << 31
MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS = 2_000
MAVLINK_BOOT_TIME_REBOOT_REGRESSION_MS = MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS
MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_SAMPLES = 2
MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_WINDOW_S = 2.0
MAVLINK_BOOT_TIME_FORWARD_SLACK_MS = 5_000


@dataclass(frozen=True)
class _MavlinkBootTimeSample:
    time_boot_ms: int
    received_at_s: float


@dataclass
class _MavlinkBootTimeRegressionCandidate:
    latest: _MavlinkBootTimeSample
    sample_count: int = 1


@dataclass(frozen=True)
class _MavlinkRemoteRebootEvidence:
    previous_time_boot_ms: int
    current_time_boot_ms: int


@dataclass(frozen=True)
class _MavlinkBootTimeObservation:
    accept_for_downstream: bool = True
    reboot: _MavlinkRemoteRebootEvidence | None = None


class _MavlinkBootTimeTracker:
    """Confirm remote PX4 boot epochs from one side's ATTITUDE clock.

    MAVLink ``time_boot_ms`` is uint32.  Forward modular movement includes the
    natural roughly 49.7-day wrap.  Small backward movement is packet
    reordering, while movement beyond the two-second reorder tolerance requires
    two increasing low-epoch samples to confirm a reboot.  The host receive
    clock also rejects a delayed prior-boot high-time packet after confirmation.
    """

    def __init__(self) -> None:
        self._latest: _MavlinkBootTimeSample | None = None
        self._candidate: _MavlinkBootTimeRegressionCandidate | None = None
        self._confirmed_reboots = 0

    def reset(self) -> None:
        self._latest = None
        self._candidate = None
        self._confirmed_reboots = 0

    @staticmethod
    def _forward_delta(previous_ms: int, current_ms: int) -> int:
        return (int(current_ms) - int(previous_ms)) % MAVLINK_BOOT_TIME_UINT32_MODULUS

    @staticmethod
    def _plausible_forward(
        previous: _MavlinkBootTimeSample,
        current: _MavlinkBootTimeSample,
    ) -> bool:
        delta_ms = _MavlinkBootTimeTracker._forward_delta(
            previous.time_boot_ms,
            current.time_boot_ms,
        )
        if not 0 < delta_ms < MAVLINK_BOOT_TIME_UINT32_HALF_RANGE:
            return False
        host_elapsed_ms = max(
            0,
            int(round((current.received_at_s - previous.received_at_s) * 1_000.0)),
        )
        return delta_ms <= host_elapsed_ms + MAVLINK_BOOT_TIME_FORWARD_SLACK_MS

    def observe(
        self,
        time_boot_ms: Any,
        *,
        received_at_s: float,
    ) -> _MavlinkBootTimeObservation:
        if isinstance(time_boot_ms, bool):
            return _MavlinkBootTimeObservation()
        try:
            boot_ms = int(time_boot_ms)
            observed_s = float(received_at_s)
        except (TypeError, ValueError, OverflowError):
            return _MavlinkBootTimeObservation()
        if (
            not 0 <= boot_ms < MAVLINK_BOOT_TIME_UINT32_MODULUS
            or not math.isfinite(observed_s)
        ):
            return _MavlinkBootTimeObservation()

        sample = _MavlinkBootTimeSample(boot_ms, observed_s)
        previous = self._latest
        if previous is None:
            self._latest = sample
            return _MavlinkBootTimeObservation()

        candidate = self._candidate
        if candidate is not None:
            candidate_age_s = sample.received_at_s - candidate.latest.received_at_s
            if not 0.0 <= candidate_age_s <= MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_WINDOW_S:
                self._candidate = None
            else:
                baseline_forward = self._forward_delta(
                    previous.time_boot_ms,
                    sample.time_boot_ms,
                )
                raw_regression_ms = max(
                    0,
                    previous.time_boot_ms - sample.time_boot_ms,
                )
                if baseline_forward == 0 or self._plausible_forward(previous, sample):
                    self._candidate = None
                    if baseline_forward != 0:
                        self._latest = sample
                    return _MavlinkBootTimeObservation()
                if raw_regression_ms <= MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS:
                    self._candidate = None
                    return _MavlinkBootTimeObservation()
                if (
                    self._plausible_forward(candidate.latest, sample)
                    and raw_regression_ms >= MAVLINK_BOOT_TIME_REBOOT_REGRESSION_MS
                ):
                    candidate.latest = sample
                    candidate.sample_count += 1
                    if (
                        candidate.sample_count
                        >= MAVLINK_BOOT_TIME_REBOOT_CONFIRMATION_SAMPLES
                    ):
                        evidence = _MavlinkRemoteRebootEvidence(
                            previous_time_boot_ms=previous.time_boot_ms,
                            current_time_boot_ms=sample.time_boot_ms,
                        )
                        self._latest = sample
                        self._candidate = None
                        self._confirmed_reboots += 1
                        return _MavlinkBootTimeObservation(reboot=evidence)
                    return _MavlinkBootTimeObservation()
                self._candidate = None

        forward_delta = self._forward_delta(
            previous.time_boot_ms,
            sample.time_boot_ms,
        )
        if forward_delta == 0:
            return _MavlinkBootTimeObservation()
        if self._plausible_forward(previous, sample):
            self._latest = sample
            return _MavlinkBootTimeObservation()

        raw_regression_ms = max(
            0,
            previous.time_boot_ms - sample.time_boot_ms,
        )
        if 0 < raw_regression_ms <= MAVLINK_BOOT_TIME_REORDER_TOLERANCE_MS:
            return _MavlinkBootTimeObservation()
        if raw_regression_ms >= MAVLINK_BOOT_TIME_REBOOT_REGRESSION_MS:
            self._candidate = _MavlinkBootTimeRegressionCandidate(
                latest=sample,
            )
            return _MavlinkBootTimeObservation()

        # Once a lower boot epoch is confirmed, a large forward leap that
        # cannot fit the host elapsed time is a delayed prior-epoch packet.
        # Before any confirmed reboot, retain compatibility and merely exclude
        # the implausible value from the tracker baseline.
        return _MavlinkBootTimeObservation(
            accept_for_downstream=self._confirmed_reboots == 0
        )


def _mavlink_udpin_endpoint(url: str) -> tuple[str, int] | None:
    """Return the IPv4 bind endpoint for a pymavlink ``udpin`` URL."""

    text = str(url).strip()
    if not text.lower().startswith("udpin:"):
        return None
    address = text[6:]
    try:
        host, port_text = address.rsplit(":", 1)
        port = int(port_text)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid MAVLink udpin URL: {url!r}") from exc
    if not host or port <= 0 or port > 65535:
        raise ValueError(f"Invalid MAVLink udpin URL: {url!r}")
    return host, port


def _assert_mavlink_udpin_available(
    url: str,
    *,
    socket_factory: Callable[..., Any] = socket.socket,
) -> None:
    """Fail before pymavlink silently shares an existing UDP input port.

    pymavlink enables ``SO_REUSEADDR`` before binding every ``udpin`` socket.
    That is convenient for generic telemetry tools, but unsafe for this
    owner: a leaked worker or another application can bind the same endpoint
    and make packet delivery non-deterministic while both UIs claim success.
    """

    endpoint = _mavlink_udpin_endpoint(url)
    if endpoint is None:
        return
    probe = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(endpoint)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            host, port = endpoint
            raise RuntimeError(
                f"MAVLink UDP input {host}:{port} is already in use; "
                "close the other MAVLink session or leaked worker before reconnecting"
            ) from exc
        raise
    finally:
        probe.close()


def _disable_mavlink_udpin_reuse(connection: Any, url: str) -> None:
    """Make the live pymavlink socket exclusive after its successful bind."""

    if _mavlink_udpin_endpoint(url) is None:
        return
    port_socket = getattr(connection, "port", None)
    setsockopt = getattr(port_socket, "setsockopt", None)
    if callable(setsockopt):
        setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)


def _seed_mavlink_udpin_peer(
    connection: Any,
    url: str,
    peer_host: str,
    *,
    observed_wall_s: float | None = None,
) -> tuple[str, int] | None:
    """Seed pymavlink's input-server destination before the first RX packet.

    ``mavudp(input=True)`` sends only to addresses learned by ``recvfrom``.
    The Workstation already owns a fixed PX4 Ethernet peer per side, so seed
    that peer on the same bound ``udpin`` socket and let the normal pymavlink
    encoder/write path send the first GCS heartbeat. The peer UDP port is the
    port in the side's approved ``udpin`` endpoint.
    """

    endpoint = _mavlink_udpin_endpoint(url)
    if endpoint is None:
        return None
    host_text = str(peer_host).strip()
    if not host_text:
        return None
    try:
        normalized_host = str(IPv4Address(host_text))
    except ValueError as exc:
        raise ValueError(f"Invalid MAVLink Ethernet peer IPv4 address: {peer_host!r}") from exc
    if not bool(getattr(connection, "udp_server", False)):
        raise RuntimeError("MAVLink udpin connection is not an input UDP server")
    clients = getattr(connection, "clients", None)
    if not isinstance(clients, set):
        raise RuntimeError("MAVLink udpin connection does not expose its peer set")
    peer = (normalized_host, int(endpoint[1]))
    clients.add(peer)
    last_alive = getattr(connection, "clients_last_alive", None)
    if isinstance(last_alive, dict):
        last_alive[peer] = time.time() if observed_wall_s is None else float(observed_wall_s)
    return peer


def camera_label_ko(camera: str) -> str:
    return {"left": "왼쪽", "right": "오른쪽", "drone": "Drone"}.get(camera, camera)


def _normalize_transport(value: Any) -> str:
    transport = str(value).strip().lower()
    return transport if transport in {"radio", "usb"} else "ethernet"


def _is_serial_transport(transport: Any) -> bool:
    """Radio and USB open one exclusive serial device; Ethernet does not.

    Only the bandwidth policy separates the two: Radio throttles optional
    streams, USB carries the stock Ethernet-class stream set.
    """

    return str(transport).strip().lower() in {"radio", "usb"}


def _finite_float(value: Any) -> float | None:
    """Return a source numeric only when it has a finite representation."""

    if isinstance(value, (list, tuple)):
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return numeric if math.isfinite(numeric) else None


def _nonfinite_numeric_reason(value: Any) -> str | None:
    """Describe an explicitly non-finite source value without coercing it valid."""

    if isinstance(value, (list, tuple)):
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if math.isnan(numeric):
        return "nan"
    if numeric == math.inf:
        return "positive_infinity"
    if numeric == -math.inf:
        return "negative_infinity"
    return None



def _optional_float(message: Any, *names: str) -> float | None:
    for name in names:
        if hasattr(message, name):
            return _finite_float(getattr(message, name))
    return None


def _optional_int(message: Any, *names: str) -> int | None:
    for name in names:
        if hasattr(message, name):
            value = getattr(message, name)
            if _nonfinite_numeric_reason(value) is not None:
                return None
            try:
                return int(value)
            except (TypeError, ValueError, OverflowError):
                return None
    return None


def _optional_bool(message: Any, *names: str) -> bool | None:
    for name in names:
        if hasattr(message, name):
            return bool(getattr(message, name))
    return None


def _bounded_mavlink_id(value: Any, *, minimum: int, maximum: int) -> int | None:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if minimum <= parsed <= maximum else None


def mavlink_message_provenance(message: Any) -> dict[str, int | None]:
    """Return source identity carried by a pymavlink message.

    These fields identify the received MAVLink packet only.  They do not
    alter the producer frame, units, or any of the app/PX4/GPS time axes.
    ``pymavlink`` exposes the values as methods on ``MAVLink_message``;
    accepting attributes as a fallback keeps this helper testable with small
    message doubles and with compatible dialect implementations.
    """

    def read(*names: str) -> Any:
        for name in names:
            value = getattr(message, name, None)
            if value is None:
                continue
            try:
                return value() if callable(value) else value
            except Exception:
                continue
        return None

    return {
        "mavlink_source_system": _bounded_mavlink_id(read("get_srcSystem", "srcSystem"), minimum=1, maximum=255),
        "mavlink_source_component": _bounded_mavlink_id(
            read("get_srcComponent", "srcComponent"), minimum=0, maximum=255
        ),
        "mavlink_sequence": _bounded_mavlink_id(read("get_seq", "seq"), minimum=0, maximum=255),
        "mavlink_message_id": _bounded_mavlink_id(
            read("get_msgId", "msgid", "msg_id"), minimum=0, maximum=16_777_215
        ),
    }


DEFAULT_USB_MAVLINK_BAUDRATES = (57600, 115200, 921600)
USB_MAVLINK_IDENTITY_MAX_AGE_S = 30.0


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


def usb_mavlink_device_candidates(
    patterns: tuple[str, ...] = ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"),
) -> list[str]:
    candidates: list[str] = []
    targets_seen: set[str] = set()
    for pattern in patterns:
        for candidate in sorted(glob.glob(pattern)):
            target = os.path.realpath(candidate)
            if target in targets_seen:
                continue
            targets_seen.add(target)
            candidates.append(candidate)
    return candidates


def probe_usb_mavlink_devices(
    candidates: list[str] | None = None,
    *,
    baudrates: tuple[int, ...] = DEFAULT_USB_MAVLINK_BAUDRATES,
    timeout_s: float = 1.0,
) -> list[UsbMavlinkIdentity]:
    try:
        from pymavlink import mavutil
    except Exception as exc:
        raise RuntimeError(f"pymavlink를 사용할 수 없어 USB Pixhawk 자동 탐지를 할 수 없습니다. ({exc})") from exc
    found: list[UsbMavlinkIdentity] = []
    for device in candidates if candidates is not None else usb_mavlink_device_candidates():
        for baud in baudrates:
            connection = None
            try:
                connection = mavutil.mavlink_connection(
                    device,
                    baud=int(baud),
                    autoreconnect=False,
                    robust_parsing=True,
                    source_system=MAVLINK_GCS_SOURCE_SYSTEM,
                    source_component=MAVLINK_GCS_SOURCE_COMPONENT,
                )
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                heartbeat = connection.wait_heartbeat(timeout=timeout_s)
                if heartbeat is not None:
                    provenance = mavlink_message_provenance(heartbeat)
                    system_id = provenance["mavlink_source_system"]
                    component_id = provenance["mavlink_source_component"]
                    if system_id is None or component_id is None:
                        continue
                    found.append(
                        UsbMavlinkIdentity(
                            device=str(device),
                            resolved_device=os.path.realpath(device),
                            baud=int(baud),
                            system_id=int(system_id),
                            component_id=int(component_id),
                            observed_monotonic_s=time.monotonic(),
                        )
                    )
                    break
            except Exception:
                continue
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
    return found


def extract_field_path(message: Any, field_path: str) -> Any:
    value: Any = message
    for part in re.split(r"[.:]", field_path):
        part = part.strip()
        if not part:
            continue
        match = re.fullmatch(r"([^\[]+)(?:\[(\d+)\])?", part)
        if not match:
            raise ValueError(f"지원하지 않는 field segment: {part}")
        name, index_text = match.groups()
        value = getattr(value, name)
        if index_text is not None:
            value = value[int(index_text)]
    return value


def _coerce_numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return float(int(value))
    if isinstance(value, (int, float)):
        return _finite_float(value)
    return None


def _format_custom_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_format_custom_value(item) for item in value[:6]) + (", ..." if len(value) > 6 else "") + "]"
    return str(value)


_MAVLINK_OPTIONAL_TIMELINE_NUMERIC_SOURCE_FIELDS: dict[str, tuple[str, ...]] = {
    "GLOBAL_POSITION_INT": (
        "lat",
        "lon",
        "alt",
        "relative_alt",
        "vx",
        "vy",
        "vz",
        "hdg",
        "time_boot_ms",
    ),
    "LOCAL_POSITION_NED": ("x", "y", "z", "vx", "vy", "vz", "time_boot_ms"),
    "VFR_HUD": ("airspeed", "groundspeed", "heading", "throttle", "alt", "climb"),
    "SYS_STATUS": (
        "voltage_battery",
        "current_battery",
        "battery_remaining",
        "drop_rate_comm",
        "errors_comm",
    ),
    "BATTERY_STATUS": (
        "current_battery",
        "battery_remaining",
        "temperature",
        "voltages[0]",
        "voltages[1]",
        "voltages[2]",
    ),
    "RAW_IMU": (
        "xacc",
        "yacc",
        "zacc",
        "xgyro",
        "ygyro",
        "zgyro",
        "xmag",
        "ymag",
        "zmag",
        "time_usec",
    ),
}


def _mavlink_nonfinite_source_reasons(
    message_type: str,
    message: Any,
) -> dict[str, str]:
    reasons: dict[str, str] = {}
    for field_path in _MAVLINK_OPTIONAL_TIMELINE_NUMERIC_SOURCE_FIELDS.get(
        message_type,
        (),
    ):
        try:
            value = extract_field_path(message, field_path)
        except (AttributeError, IndexError, TypeError, ValueError):
            continue
        reason = _nonfinite_numeric_reason(value)
        if reason is not None:
            reasons[f"{message_type}.{field_path}"] = reason
    return reasons



class DualPixhawkMavlinkSubscriber(QtCore.QObject):
    extrinsic_changed = QtCore.pyqtSignal(str, object)
    gps_changed = QtCore.pyqtSignal(str, object)
    mag_sensor_changed = QtCore.pyqtSignal(str, object)
    gps_sensor_changed = QtCore.pyqtSignal(str, object)
    gnss_relative_changed = QtCore.pyqtSignal(str, object)
    custom_watch_changed = QtCore.pyqtSignal(object)
    optional_timeline_changed = QtCore.pyqtSignal(str, object)
    stream_setup_required = QtCore.pyqtSignal(str)
    shell_output = QtCore.pyqtSignal(str, str)
    status_changed = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        left_url: str,
        right_url: str,
        left_baud: int = 57600,
        right_baud: int = 57600,
        drone_url: str = "",
        drone_baud: int = 57600,
        custom_watches: tuple[dict[str, str], ...] = (),
        expected_system_ids: dict[str, int] | None = None,
        peer_hosts: dict[str, str] | None = None,
        transports: dict[str, str] | None = None,
    ) -> None:
        super().__init__()
        self.urls = {
            "left": left_url.strip(),
            "right": right_url.strip(),
            "drone": drone_url.strip(),
        }
        self.baudrates = {
            "left": int(left_baud),
            "right": int(right_baud),
            "drone": int(drone_baud),
        }
        self.peer_hosts = {side: "" for side in self.urls}
        if peer_hosts is not None:
            self.peer_hosts.update(
                {str(side): str(host).strip() for side, host in peer_hosts.items()}
            )
        self.expected_system_ids = dict(MAVLINK_EXPECTED_SYSTEM_IDS)
        if expected_system_ids is not None:
            self.expected_system_ids.update(
                {str(side): int(system_id) for side, system_id in expected_system_ids.items()}
            )
        self.transports = {side: "ethernet" for side in self.urls}
        if transports is not None:
            self.transports.update(
                {
                    str(side): _normalize_transport(value)
                    for side, value in transports.items()
                }
            )
        self._radio_stream_policies = {side: RadioStreamPolicy() for side in self.urls}
        self._radio_rtcm_healthy = {side: True for side in self.urls}
        self._radio_status: dict[str, RadioStatusState] = {}
        self._radio_lock_handles: dict[str, Any] = {}
        self.connections: dict[str, Any] = {}
        self.last_connect_errors: dict[str, str] = {}
        self.custom_watches = list(custom_watches)
        self._heartbeat_logged: set[str] = set()
        self._unrecognized_debug_vect_names: set[str] = set()
        self._mavlink_stream_requests_sent: set[str] = set()
        self._mavlink_stream_request_attempts: dict[str, int] = {}
        self._mavlink_stream_last_request_s: dict[str, float] = {}
        self._mavlink_boot_time_trackers = {
            side: _MavlinkBootTimeTracker() for side in self.urls
        }
        self._sasl_can_mag_stream_confirmed: set[str] = set()
        self._last_attitude_message_s: dict[str, float] = {}
        self._attitude_stream_request_attempts: dict[str, int] = {}
        self._attitude_stream_last_request_s: dict[str, float] = {}
        self._attitude_stream_outage_reported: set[str] = set()
        self._last_imu_stream_message_s: dict[tuple[str, str], float] = {}
        self._imu_stream_request_attempts: dict[tuple[str, str], int] = {}
        self._imu_stream_last_request_s: dict[tuple[str, str], float] = {}
        self._imu_stream_outage_reported: set[tuple[str, str]] = set()
        self._mavlink_stream_config: dict[str, Any] | None = None
        self._mavlink_stream_configured: set[str] = set()
        self._latest_gnss_relative: dict[str, GnssRelativeState] = {}
        self._command_state_lock = threading.RLock()
        self._mavlink_send_locks = {side: threading.RLock() for side in self.urls}
        self._active_command_cameras: set[str] = set()
        self._deferred_command_messages: dict[str, deque[Any]] = {
            side: deque() for side in self.urls
        }
        self._recent_raw_packets: dict[str, deque[tuple[float, bytes]]] = {
            side: deque(maxlen=MAVLINK_DUPLICATE_CACHE_SIZE) for side in self.urls
        }
        self._last_receive_errors: dict[str, tuple[str, float]] = {}
        self._last_gcs_heartbeat_errors: dict[str, tuple[str, float]] = {}
        self._last_rtcm_status_emit_s: dict[str, float] = {}
        self._gcs_heartbeat_last_attempt_s: dict[str, float] = {}
        self._identity_mismatch_reported: set[tuple[str, int]] = set()
        self._liveness_connection_ids: dict[str, int] = {}
        self._seeded_udp_peers: dict[str, tuple[str, int]] = {}
        self._last_health_states: dict[str, MavlinkLivenessState] = {}
        self._mavlink_liveness = {
            side: MavlinkSideLiveness(
                side=side,
                expected_system_id=self.expected_system_ids[side],
                endpoint=self.urls.get(side, ""),
                peer_host=self.peer_hosts.get(side, ""),
                clock=lambda: time.monotonic(),
            )
            for side in self.urls
        }
        self._stream_setup_requests_sent: set[str] = set()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.spin_once)

    @contextmanager
    def exclusive_command(
        self,
        camera: str,
        *,
        hold_send_lock: bool = True,
        command_label: str = "MAVLINK COMMAND",
        preserve_qualification: bool = False,
    ):
        """Reserve one MAVLink receive path for a blocking command worker.

        ``spin_once`` is normally the sole receiver.  A PARAM or SERIAL_CONTROL
        request must temporarily exclude that timer or the timer can consume
        the command response first.  The reservation is intentionally narrow:
        callers still own the worker/thread and this method does not touch Qt
        objects other than the existing subscriber state.
        """

        camera = str(camera)
        send_lock = self._mavlink_send_locks.setdefault(camera, threading.RLock())
        with (send_lock if hold_send_lock else nullcontext()):
            with self._command_state_lock:
                if camera not in self.connections:
                    raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
                if camera in self._active_command_cameras:
                    raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 명령이 이미 진행 중입니다.")
                model = self._ensure_liveness_listener(camera)
                model.begin_command(
                    str(command_label),
                    now_s=time.monotonic(),
                    preserve_qualification=bool(preserve_qualification),
                )
                self._active_command_cameras.add(camera)
            try:
                # The command worker owns both recv_match and this side's
                # re-entrant send lock until the reservation ends. Keep the
                # PX4 peer current from that owner; the GUI timer deliberately
                # skips an active side and must never wait on this lock.
                self._send_due_gcs_heartbeat(
                    camera,
                    self.connections[camera],
                    now_s=time.monotonic(),
                )
                yield
            finally:
                with self._command_state_lock:
                    self._active_command_cameras.discard(camera)
                    model.end_command(now_s=time.monotonic())

    def start(
        self,
        camera: str | None = None,
        *,
        start_timer: bool = True,
        apply_pending_setup: bool = False,
    ) -> None:
        try:
            from pymavlink import mavutil
        except Exception as exc:
            raise RuntimeError(f"pymavlink를 사용할 수 없습니다. MAVLink 연결에는 pymavlink 설치가 필요합니다. ({exc})") from exc
        from Workstation.workstation_core.mavlink_dialects import ensure_registered as _ensure_sasl_dialect
        sasl_dialect = _ensure_sasl_dialect()
        cameras = (camera,) if camera is not None else tuple(self.urls.keys())
        started_any = False
        for side in cameras:
            url = self.urls.get(side, "").strip()
            if not url:
                self.last_connect_errors[side] = "MAVLink URL이 설정되지 않았습니다."
                continue
            if side in self.connections:
                self._ensure_liveness_listener(side)
                started_any = True
                self.last_connect_errors.pop(side, None)
                continue
            connection = None
            radio_lock = None
            try:
                _assert_mavlink_udpin_available(url)
                if _is_serial_transport(self.transports.get(side)):
                    transport_label = str(self.transports.get(side) or "").upper()
                    for other, active_url in self.urls.items():
                        if other != side and other in self.connections and active_url == url:
                            raise RuntimeError(
                                f"{transport_label} endpoint is already owned by "
                                f"{camera_label_ko(other)}"
                            )
                    radio_lock = self._acquire_radio_endpoint_lock(side, url)
                connection = mavutil.mavlink_connection(
                    url,
                    baud=self.baudrates.get(side, 57600),
                    autoreconnect=False,
                    robust_parsing=True,
                    dialect=sasl_dialect,
                    source_system=MAVLINK_GCS_SOURCE_SYSTEM,
                    source_component=MAVLINK_GCS_SOURCE_COMPONENT,
                )
                _disable_mavlink_udpin_reuse(connection, url)
                seeded_peer = _seed_mavlink_udpin_peer(
                    connection,
                    url,
                    self.peer_hosts.get(side, ""),
                )
            except Exception as exc:
                # A multi-side request (``camera=None``) must not fail the sides
                # that did connect just because one side raised; the caller reads
                # ``last_connect_errors`` afterwards to surface a partial-connect
                # status instead of a misleading full-success label.
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
                if radio_lock is not None:
                    self._release_radio_endpoint_lock(side)
                self.last_connect_errors[side] = str(exc)
                self.status_changed.emit(f"{camera_label_ko(side)} MAVLink 연결 실패: {exc}")
                continue
            with self._command_state_lock:
                self.connections[side] = connection
                self._open_liveness_generation(side, connection)
                if seeded_peer is None:
                    self._seeded_udp_peers.pop(side, None)
                else:
                    self._seeded_udp_peers[side] = seeded_peer
            started_any = True
            self.last_connect_errors.pop(side, None)
            self.status_changed.emit(
                f"{camera_label_ko(side)} MAVLink 연결 시작: {url} @ {self.baudrates.get(side, 57600)}"
            )
            self._send_due_gcs_heartbeat(side, connection, now_s=time.monotonic())
            if apply_pending_setup:
                self._apply_pending_connection_setup(side, connection)
        if not started_any:
            label = f"{camera_label_ko(camera)} " if camera is not None else ""
            raise RuntimeError(f"{label}MAVLink 연결 URL이 없거나 연결 시작에 실패했습니다.")
        if not self.connections:
            raise RuntimeError("MAVLink 연결 URL이 없거나 모든 연결 시작에 실패했습니다.")
        if start_timer:
            self.start_receive_timer()

    @QtCore.pyqtSlot()
    def start_receive_timer(self) -> None:
        """Start the Qt receive timer from the subscriber's owning Qt thread."""

        if QtCore.QThread.currentThread() is not self.thread():
            QtCore.QMetaObject.invokeMethod(
                self,
                "start_receive_timer",
                QtCore.Qt.QueuedConnection,
            )
            return
        if self.connections:
            self.timer.start()

    @QtCore.pyqtSlot()
    def stop_receive_timer(self) -> None:
        """Stop the receive timer only from the subscriber's Qt owner thread."""

        if QtCore.QThread.currentThread() is not self.thread():
            QtCore.QMetaObject.invokeMethod(
                self,
                "stop_receive_timer",
                QtCore.Qt.QueuedConnection,
            )
            return
        self.timer.stop()

    def _acquire_radio_endpoint_lock(self, side: str, endpoint: str):
        side = str(side)
        if side in self._radio_lock_handles:
            return self._radio_lock_handles[side]
        resolved = os.path.realpath(str(endpoint))
        digest = hashlib.sha256(resolved.encode("utf-8")).hexdigest()[:20]
        runtime_root = Path(f"/run/user/{os.getuid()}")
        if not runtime_root.is_dir():
            runtime_root = Path("/tmp")
        lock_path = runtime_root / f"workstation-pixhawk-radio-{digest}.lock"
        handle = lock_path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            handle.close()
            raise RuntimeError(f"Radio endpoint is busy: {endpoint}") from exc
        handle.seek(0)
        handle.truncate()
        handle.write(f"pid={os.getpid()} role={side} endpoint={resolved}\n")
        handle.flush()
        self._radio_lock_handles[side] = handle
        return handle

    def _release_radio_endpoint_lock(self, side: str) -> None:
        handle = self._radio_lock_handles.pop(str(side), None)
        if handle is None:
            return
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()

    def _open_liveness_generation(self, side: str, connection: Any) -> MavlinkSideLiveness:
        side = str(side)
        model = self._mavlink_liveness[side]
        model.open_listener(
            endpoint=self.urls.get(side, ""),
            peer_host=self.peer_hosts.get(side, ""),
            now_s=time.monotonic(),
        )
        self._liveness_connection_ids[side] = id(connection)
        self._gcs_heartbeat_last_attempt_s.pop(side, None)
        self._mavlink_boot_time_trackers.setdefault(
            side, _MavlinkBootTimeTracker()
        ).reset()
        return model

    def _ensure_liveness_listener(self, side: str) -> MavlinkSideLiveness:
        side = str(side)
        model = self._mavlink_liveness[side]
        connection = self.connections.get(side)
        if connection is not None and self._liveness_connection_ids.get(side) != id(connection):
            return self._open_liveness_generation(side, connection)
        return model

    def mavlink_health_snapshot(
        self,
        side: str,
        *,
        now_s: float | None = None,
    ) -> MavlinkSideHealthSnapshot:
        model = self._ensure_liveness_listener(str(side))
        return model.snapshot(now_s=now_s)

    def mavlink_health_snapshots(
        self,
        *,
        now_s: float | None = None,
    ) -> dict[str, MavlinkSideHealthSnapshot]:
        observed_s = time.monotonic() if now_s is None else float(now_s)
        return {
            side: self.mavlink_health_snapshot(side, now_s=observed_s)
            for side in self.urls
        }

    def _clear_mavlink_imu_stream_state(
        self,
        camera: str,
        *,
        clear_samples: bool,
    ) -> None:
        camera = str(camera)
        keys = {
            (camera, message_name)
            for message_name in MAVLINK_UI_IMU_STREAM_NAMES
        }
        if clear_samples:
            for key in keys:
                self._last_imu_stream_message_s.pop(key, None)
        for key in keys:
            self._imu_stream_request_attempts.pop(key, None)
            self._imu_stream_last_request_s.pop(key, None)
        self._imu_stream_outage_reported.difference_update(keys)

    def stop(self, camera: str | None = None) -> None:
        if camera is None:
            connections = list(self.connections.items())
        else:
            connection = self.connections.get(camera)
            connections = [] if connection is None else [(camera, connection)]
        for side, connection in connections:
            with self._command_state_lock:
                try:
                    connection.close()
                except Exception:
                    pass
                self.connections.pop(side, None)
                self._mavlink_liveness[side].close_listener(now_s=time.monotonic())
                self._liveness_connection_ids.pop(side, None)
                self._seeded_udp_peers.pop(side, None)
            self._heartbeat_logged.discard(side)
            self._mavlink_stream_requests_sent.discard(side)
            self._mavlink_stream_request_attempts.pop(side, None)
            self._mavlink_stream_last_request_s.pop(side, None)
            self._mavlink_boot_time_trackers.setdefault(
                side, _MavlinkBootTimeTracker()
            ).reset()
            self._sasl_can_mag_stream_confirmed.discard(side)
            self._last_attitude_message_s.pop(side, None)
            self._attitude_stream_request_attempts.pop(side, None)
            self._attitude_stream_last_request_s.pop(side, None)
            self._attitude_stream_outage_reported.discard(side)
            self._clear_mavlink_imu_stream_state(side, clear_samples=True)
            self._mavlink_stream_configured.discard(side)
            self._stream_setup_requests_sent.discard(side)
            self._latest_gnss_relative.pop(side, None)
            self._deferred_command_messages.setdefault(side, deque()).clear()
            self._recent_raw_packets.setdefault(
                side, deque(maxlen=MAVLINK_DUPLICATE_CACHE_SIZE)
            ).clear()
            self._last_receive_errors.pop(side, None)
            self._last_gcs_heartbeat_errors.pop(side, None)
            self._last_rtcm_status_emit_s.pop(side, None)
            self._gcs_heartbeat_last_attempt_s.pop(side, None)
            self._identity_mismatch_reported = {
                item for item in self._identity_mismatch_reported if item[0] != side
            }
            self._last_health_states.pop(side, None)
            self._radio_stream_policies.setdefault(side, RadioStreamPolicy()).reset()
            self._radio_status.pop(side, None)
            self._release_radio_endpoint_lock(side)
        if not self.connections:
            self.stop_receive_timer()
        if camera is None:
            self.status_changed.emit("MAVLink 연결 중지됨")
        else:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink 연결 중지됨")

    def add_custom_watch(self, spec: dict[str, str]) -> None:
        if spec not in self.custom_watches:
            self.custom_watches.append(spec)

    def set_transport(self, side: str, transport: str) -> None:
        side = str(side)
        if side not in self.urls:
            raise ValueError(f"Unknown Pixhawk role: {side}")
        normalized = _normalize_transport(transport)
        if side in self.connections and normalized != self.transports.get(side):
            raise RuntimeError(f"{camera_label_ko(side)} transport cannot change while connected")
        self.transports[side] = normalized
        self._radio_stream_policies.setdefault(side, RadioStreamPolicy()).reset()

    def set_radio_rtcm_health(self, side: str, healthy: bool) -> None:
        self._radio_rtcm_healthy[str(side)] = bool(healthy)

    def radio_stream_policy_snapshot(self, side: str, *, now_s: float | None = None):
        observed_s = time.monotonic() if now_s is None else float(now_s)
        return self._radio_stream_policies[str(side)].snapshot(now_s=observed_s)

    def radio_status_snapshot(self, side: str) -> RadioStatusState | None:
        return self._radio_status.get(str(side))

    def _active_stream_requests(self, camera: str) -> tuple[tuple[str, float], ...]:
        if self.transports.get(str(camera)) != "radio":
            return MAVLINK_STOCK_STREAM_REQUESTS
        policy = self._radio_stream_policies.setdefault(str(camera), RadioStreamPolicy())
        return radio_stream_requests(policy.tier)

    def _advance_radio_stream_policy(self, camera: str, *, now_s: float) -> None:
        camera = str(camera)
        if self.transports.get(camera) != "radio":
            return
        health = self.mavlink_health_snapshot(camera, now_s=now_s)
        qualified = health.state is MavlinkLivenessState.LIVE
        healthy = qualified and self._radio_rtcm_healthy.get(camera, True)
        policy = self._radio_stream_policies.setdefault(camera, RadioStreamPolicy())
        if not policy.observe(qualified=qualified, healthy=healthy, now_s=now_s):
            return
        self._mavlink_stream_requests_sent.discard(camera)
        self._clear_mavlink_imu_stream_state(camera, clear_samples=False)
        self._request_mavlink_sensor_streams(camera)
        self.status_changed.emit(
            f"{camera_label_ko(camera)} Radio optional stream tier={policy.tier.name}"
        )

    def remove_custom_watch(self, key: str) -> None:
        self.custom_watches = [spec for spec in self.custom_watches if str(spec.get("key", "")) != key]

    def configure_mavlink_streams(
        self,
        *,
        host_ip: str,
        ports: dict[str, int],
        sysids: dict[str, int] | None = None,
        rate: int = 2_000_000,
        cameras: set[str] | tuple[str, ...] | list[str] | None = None,
    ) -> None:
        sysids = sysids or dict(MAVLINK_EXPECTED_SYSTEM_IDS)
        self._mavlink_stream_config = {
            "host_ip": host_ip,
            "ports": dict(ports),
            "sysids": dict(sysids),
            "rate": int(rate),
        }
        target_cameras = set(cameras) if cameras is not None else set(self.connections.keys())
        for camera, connection in self.connections.items():
            if camera not in target_cameras:
                continue
            try:
                with self.exclusive_command(camera):
                    self._apply_mavlink_stream_config(camera, connection)
            except Exception as exc:
                self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink stream 명령 슬롯 실패: {exc}")

    def read_param_int(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | None:
        # PARAM reads must reserve the receive path (see exclusive_command's
        # docstring) or spin_once's 20ms timer silently drains the
        # PARAM_VALUE response before this blocking read ever sees it.
        with self.exclusive_command(camera):
            connection = self.connections[camera]
            return _mavlink_read_param_int(
                connection,
                name,
                timeout_s=timeout_s,
                on_unmatched=lambda message: self._defer_command_message(camera, message),
                on_receive_poll=lambda: self._send_due_gcs_heartbeat(
                    camera,
                    connection,
                    now_s=time.monotonic(),
                ),
            )

    def read_param(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | float | None:
        with self.exclusive_command(camera):
            connection = self.connections[camera]
            return _mavlink_read_param(
                connection,
                name,
                timeout_s=timeout_s,
                on_unmatched=lambda message: self._defer_command_message(camera, message),
                on_receive_poll=lambda: self._send_due_gcs_heartbeat(
                    camera,
                    connection,
                    now_s=time.monotonic(),
                ),
            )

    def set_param_int(self, camera: str, name: str, value: int, *, timeout_s: float = 2.0) -> int | None:
        with self.exclusive_command(camera):
            connection = self.connections[camera]
            actual = _mavlink_set_param_int(
                connection,
                name,
                int(value),
                timeout_s=timeout_s,
                on_unmatched=lambda message: self._defer_command_message(camera, message),
                on_receive_poll=lambda: self._send_due_gcs_heartbeat(
                    camera,
                    connection,
                    now_s=time.monotonic(),
                ),
            )
        if actual == int(value):
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 param 설정 확인: {name}={actual}")
        else:
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 param 설정 불일치: {name} expected={int(value)} actual={actual}")
        return actual

    def send_rtcm_data(self, camera: str, payload: bytes) -> int:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        with self._mavlink_send_locks.setdefault(str(camera), threading.RLock()):
            packet_count = _send_mavlink_rtcm_data(connection, payload)
        self._emit_rtcm_status_if_due(camera, payload, packet_count)
        return packet_count

    def send_rtcm_data_if_eligible(
        self,
        camera: str,
        payload: bytes,
        *,
        expected_generation: int | None = None,
    ) -> dict[str, object]:
        """Atomically recheck side health and send one complete RTCM frame."""

        camera = str(camera)
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        with self._mavlink_send_locks.setdefault(camera, threading.RLock()):
            health = self.mavlink_health_snapshot(camera, now_s=time.monotonic())
            if (
                expected_generation is not None
                and health.connection_generation != int(expected_generation)
            ):
                return {
                    "sent": False,
                    "packet_count": 0,
                    "health": health,
                    "block_reason": "connection-generation-changed",
                }
            if not health.rtcm_eligible:
                return {
                    "sent": False,
                    "packet_count": 0,
                    "health": health,
                    "block_reason": "target-mavlink-ineligible",
                }
            packet_count = int(_send_mavlink_rtcm_data(connection, payload) or 0)
        self._emit_rtcm_status_if_due(camera, payload, packet_count)
        return {
            "sent": True,
            "packet_count": packet_count,
            "health": health,
            "block_reason": "",
        }

    def _emit_rtcm_status_if_due(
        self,
        camera: str,
        payload: bytes,
        packet_count: int,
    ) -> None:
        """Bound progress logging while the RTCM worker keeps exact counters."""

        camera = str(camera)
        now_s = time.monotonic()
        last_s = self._last_rtcm_status_emit_s.get(camera)
        if last_s is not None and now_s - last_s < MAVLINK_RTCM_STATUS_INTERVAL_S:
            return
        self._last_rtcm_status_emit_s[camera] = now_s
        self.status_changed.emit(
            f"{camera_label_ko(camera)} RTCM 주입: {len(payload)} bytes / "
            f"{int(packet_count)} MAVLink packets"
        )

    def _apply_pending_connection_setup(self, camera: str, connection: Any) -> None:
        if self._mavlink_stream_config is not None and camera not in self._mavlink_stream_configured:
            self._apply_mavlink_stream_config(camera, connection)

    def _apply_mavlink_stream_config(self, camera: str, connection: Any) -> None:
        config = self._mavlink_stream_config
        if config is None:
            return
        port = config["ports"].get(camera)
        if port is None:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink stream 설정 건너뜀: 포트 없음")
            return
        host_ip = config["host_ip"]
        rate = int(config["rate"])
        sysid = int(config["sysids"].get(camera, 1))
        command = (
            f"param set MAV_SYS_ID {sysid}\n"
            "param set MAV_2_CONFIG 1000\n"
            "param set MAV_2_MODE 0\n"
            "param set MAV_2_BROADCAST 1\n"
            f"param set MAV_2_REMOTE_PRT {int(port)}\n"
            f"param set MAV_2_UDP_PRT {int(port)}\n"
            f"param set MAV_2_RATE {rate}\n"
            "param save"
        )
        try:
            _send_mavlink_shell_command(
                connection,
                command,
                on_unmatched=lambda message: self._defer_command_message(camera, message),
                on_receive_poll=lambda: self._send_due_gcs_heartbeat(
                    camera,
                    connection,
                    now_s=time.monotonic(),
                ),
            )
        except Exception as exc:
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink stream 설정 명령 실패: {exc}")
            return
        self._mavlink_stream_configured.add(camera)
        self._stream_setup_requests_sent.discard(camera)
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink 단일-instance 영구 설정 저장: sysid={sysid}, "
            f"host={host_ip}, port={int(port)} (재부팅 후 적용)"
        )

    def _request_mavlink_sensor_streams(self, camera: str) -> None:
        connection = self.connections.get(camera)
        if connection is None:
            return
        health = self.mavlink_health_snapshot(camera, now_s=time.monotonic())
        if health.state is MavlinkLivenessState.RX_STALLED:
            return
        if camera in self._mavlink_stream_requests_sent:
            return
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            return
        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", 511)
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        now_s = time.monotonic()
        requests = self._active_stream_requests(camera)
        requested = []
        self._mavlink_stream_requests_sent.add(camera)
        self._mavlink_stream_request_attempts[camera] = 1
        self._mavlink_stream_last_request_s[camera] = now_s
        try:
            with self._mavlink_send_locks.setdefault(camera, threading.RLock()):
                for message_name, rate_hz in requests:
                    message_id = MAVLINK_MESSAGE_IDS[message_name]
                    interval_us = int(1_000_000 / float(rate_hz))
                    mav.command_long_send(
                        target_system,
                        target_component,
                        command_id,
                        0,
                        int(message_id),
                        interval_us,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                    requested.append(message_name)
        except Exception as exc:
            self._mavlink_stream_requests_sent.discard(camera)
            self._mavlink_stream_request_attempts.pop(camera, None)
            self._mavlink_stream_last_request_s.pop(camera, None)
            self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink sensor stream 요청 실패: {exc}")
            return
        if "ATTITUDE" in requested:
            self._attitude_stream_request_attempts[camera] = 1
            self._attitude_stream_last_request_s[camera] = now_s
        for message_name in MAVLINK_UI_IMU_STREAM_NAMES:
            if message_name not in requested:
                continue
            key = (camera, message_name)
            self._imu_stream_request_attempts[key] = 1
            self._imu_stream_last_request_s[key] = now_s
            self._imu_stream_outage_reported.discard(key)
        profile = (
            f"Radio {self._radio_stream_policies[camera].tier.name}"
            if self.transports.get(camera) == "radio"
            else "Ethernet Stock"
        )
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink {profile} sensor stream 요청: {', '.join(requested)}"
        )

    def _request_missing_mavlink_imu_streams(self, camera: str) -> None:
        """Bound retries to UI-backed IMU streams with no fresh sample."""

        camera = str(camera)
        connection = self.connections.get(camera)
        if connection is None or camera not in self._mavlink_stream_requests_sent:
            return
        now_s = time.monotonic()
        health = self.mavlink_health_snapshot(camera, now_s=now_s)
        if health.state is MavlinkLivenessState.RX_STALLED:
            return
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            return

        due: list[tuple[str, tuple[str, str], int]] = []
        active_rates = dict(self._active_stream_requests(camera))
        for message_name in MAVLINK_UI_IMU_STREAM_NAMES:
            if message_name not in active_rates:
                continue
            key = (camera, message_name)
            last_sample_s = self._last_imu_stream_message_s.get(key)
            if (
                last_sample_s is not None
                and now_s - last_sample_s <= MAVLINK_IMU_STREAM_STALE_S
            ):
                self._imu_stream_request_attempts.pop(key, None)
                self._imu_stream_last_request_s.pop(key, None)
                self._imu_stream_outage_reported.discard(key)
                continue
            attempts = self._imu_stream_request_attempts.get(key, 0)
            last_request_s = self._imu_stream_last_request_s.get(
                key,
                float("-inf"),
            )
            if attempts >= MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS:
                if now_s - last_request_s < MAVLINK_IMU_STREAM_RECOVERY_COOLDOWN_S:
                    if key not in self._imu_stream_outage_reported:
                        self._imu_stream_outage_reported.add(key)
                        self.status_changed.emit(
                            f"{camera_label_ko(camera)} {message_name} stream 응답이 없습니다. "
                            "재시도를 대기합니다."
                        )
                    continue
                attempts = 0
                self._imu_stream_outage_reported.discard(key)
            if now_s - last_request_s < MAVLINK_STREAM_REQUEST_RETRY_S:
                continue
            due.append((message_name, key, attempts))
        if not due:
            return

        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(
            mavlink,
            "MAV_CMD_SET_MESSAGE_INTERVAL",
            511,
        )
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        requested: list[str] = []
        try:
            with self._mavlink_send_locks.setdefault(camera, threading.RLock()):
                for message_name, key, attempts in due:
                    rate_hz = active_rates[message_name]
                    mav.command_long_send(
                        target_system,
                        target_component,
                        command_id,
                        0,
                        MAVLINK_MESSAGE_IDS[message_name],
                        int(1_000_000 / float(rate_hz)),
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                    self._imu_stream_request_attempts[key] = attempts + 1
                    self._imu_stream_last_request_s[key] = now_s
                    requested.append(message_name)
        except Exception as exc:
            self.status_changed.emit(
                f"{camera_label_ko(camera)} MAVLink IMU stream 요청 실패: {exc}"
            )
            return
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink 누락 IMU stream 재요청: "
            f"{', '.join(requested)}"
        )

    def _request_stale_attitude_stream(self, camera: str) -> None:
        """Re-request ATTITUDE while heartbeat proves the transport is alive."""

        connection = self.connections.get(camera)
        if connection is None:
            return
        health = self.mavlink_health_snapshot(camera, now_s=time.monotonic())
        if health.state is MavlinkLivenessState.RX_STALLED:
            return
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            return
        now_s = time.monotonic()
        last_attitude_s = self._last_attitude_message_s.get(camera)
        if last_attitude_s is not None and now_s - last_attitude_s <= MAVLINK_ATTITUDE_STALE_S:
            self._attitude_stream_request_attempts.pop(camera, None)
            self._attitude_stream_last_request_s.pop(camera, None)
            self._attitude_stream_outage_reported.discard(camera)
            return
        attempts = self._attitude_stream_request_attempts.get(camera, 0)
        last_request_s = self._attitude_stream_last_request_s.get(camera, float("-inf"))
        if attempts >= MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS:
            if now_s - last_request_s < MAVLINK_ATTITUDE_RECOVERY_COOLDOWN_S:
                if camera not in self._attitude_stream_outage_reported:
                    self._attitude_stream_outage_reported.add(camera)
                    self.status_changed.emit(
                        f"{camera_label_ko(camera)} MAVLink heartbeat는 수신 중이나 "
                        "ATTITUDE stream 복구 응답이 없습니다. 재시도를 대기합니다."
                    )
                return
            attempts = 0
            self._attitude_stream_outage_reported.discard(camera)
        if now_s - last_request_s < MAVLINK_STREAM_REQUEST_RETRY_S:
            return
        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", 511)
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        self._attitude_stream_request_attempts[camera] = attempts + 1
        self._attitude_stream_last_request_s[camera] = now_s
        try:
            mav.command_long_send(
                target_system,
                target_component,
                command_id,
                0,
                MAVLINK_MESSAGE_IDS["ATTITUDE"],
                int(1_000_000 / 30.0),
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            self.status_changed.emit(
                f"{camera_label_ko(camera)} MAVLink ATTITUDE stream 복구 요청 실패: {exc}"
            )
            return
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink ATTITUDE stream 복구 요청 "
            f"({attempts + 1}/{MAVLINK_STREAM_REQUEST_MAX_ATTEMPTS})"
        )

    def send_shell_command(self, camera: str, command: str) -> None:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        result = _send_mavlink_shell_command(
            connection,
            command,
            on_unmatched=lambda message: self._defer_command_message(camera, message),
            on_receive_poll=lambda: self._send_due_gcs_heartbeat(
                camera,
                connection,
                now_s=time.monotonic(),
            ),
        )
        if result.flag_label != "respond+exclusive":
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell fallback 사용: {result.flag_label}")
        self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell 명령 전송: {command}")

    def send_shell_command_with_result(self, camera: str, command: str, *, timeout_s: float = 2.0) -> MavlinkShellCommandResult:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        result = _send_mavlink_shell_command(
            connection,
            command,
            timeout_s=timeout_s,
            on_unmatched=lambda message: self._defer_command_message(camera, message),
            on_receive_poll=lambda: self._send_due_gcs_heartbeat(
                camera,
                connection,
                now_s=time.monotonic(),
            ),
        )
        if result.flag_label != "respond+exclusive":
            self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell fallback 사용: {result.flag_label}")
        self.status_changed.emit(f"{camera_label_ko(camera)} PX4 shell 명령 전송: {command}")
        return result

    def write_remote_file_mavftp(self, camera: str, remote_path: str, content: str | bytes) -> None:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        _write_mavlink_ftp_file(connection, remote_path, content)
        self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink FTP 파일 저장: {remote_path}")

    def reboot_autopilot(self, camera: str) -> None:
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            raise RuntimeError("MAVLink COMMAND_LONG을 지원하지 않는 연결입니다.")
        mavlink = getattr(connection, "mavlink", None)
        command_id = _mavlink_constant(mavlink, "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN", 246)
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        mav.command_long_send(
            target_system,
            target_component,
            command_id,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.status_changed.emit(f"{camera_label_ko(camera)} PX4 MAVLink 재부팅 명령 전송")

    def spin_once(self) -> None:
        for camera, connection in list(self.connections.items()):
            self._ensure_liveness_listener(camera)
            # ``spin_once`` runs from a Qt timer.  A continuously busy serial
            # link must not monopolize the GUI thread while we drain it.
            deadline = time.monotonic() + MAVLINK_MAX_PROCESS_TIME_S
            for _ in range(MAVLINK_MAX_MESSAGES_PER_TICK):
                deferred = False
                received_at_s: float | None = None
                with self._command_state_lock:
                    try:
                        pending = self._deferred_command_messages.setdefault(
                            camera, deque()
                        )
                        if pending:
                            received_at_s, message = pending.popleft()
                            deferred = True
                        elif camera in self._active_command_cameras:
                            # A blocking PARAM/SERIAL_CONTROL worker owns
                            # ``recv_match`` so it can receive its response.
                            # It still forwards non-command telemetry into
                            # this queue.  Drain that queue while the worker
                            # is active; otherwise a Radio Shell poll can
                            # make ATTITUDE age for the entire six-command
                            # sequence even though packets are arriving.
                            message = None
                        else:
                            message = connection.recv_match(blocking=False)
                            if message is not None:
                                received_at_s = time.monotonic()
                    except Exception as exc:
                        self._emit_receive_error(camera, exc)
                        # A serial read error means the device node is gone
                        # (radio unpowered, USB cable pulled); the fd cannot
                        # recover, so close and wait for an explicit Connect.
                        if _is_serial_transport(self.transports.get(camera)):
                            transport_label = str(self.transports.get(camera) or "").upper()
                            self.stop(camera)
                            self.status_changed.emit(
                                f"{camera_label_ko(camera)} {transport_label} DISCONNECTED · PRESS CONNECT"
                            )
                        break
                if message is None:
                    break
                if not deferred and not self._observe_mavlink_transport_message(
                    camera,
                    message,
                    received_at_s=received_at_s,
                ):
                    continue
                if self._is_duplicate_packet(camera, message):
                    continue
                self._handle_mavlink_message(
                    camera,
                    message,
                    received_at_s=received_at_s,
                    allow_heartbeat_actions=not deferred,
                )
                if time.monotonic() >= deadline:
                    break
            self._send_due_gcs_heartbeat(
                camera,
                connection,
                now_s=time.monotonic(),
                wait_for_send_lock=False,
            )
            self._emit_liveness_transition(camera, now_s=time.monotonic())
            health = self.mavlink_health_snapshot(camera, now_s=time.monotonic())
            if health.state in {
                MavlinkLivenessState.INITIAL_RX_TIMEOUT,
                MavlinkLivenessState.RX_STALLED,
            }:
                transport = self.transports.get(camera, "ethernet").upper()
                self.stop(camera)
                self.status_changed.emit(
                    f"{camera_label_ko(camera)} {transport} DISCONNECTED · PRESS CONNECT"
                )
                continue
            self._advance_radio_stream_policy(camera, now_s=time.monotonic())

    def _defer_command_message(self, camera: str, message: Any) -> None:
        """Retain telemetry consumed while a blocking command owns recv_match."""

        if message is None:
            return
        received_at_s = time.monotonic()
        if not self._observe_mavlink_transport_message(
            str(camera),
            message,
            received_at_s=received_at_s,
        ):
            return
        with self._command_state_lock:
            self._deferred_command_messages.setdefault(str(camera), deque()).append(
                (received_at_s, message)
            )

    def _observe_mavlink_transport_message(
        self,
        camera: str,
        message: Any,
        *,
        received_at_s: float | None = None,
    ) -> bool:
        """Record arrival before duplicate filtering and reject known wrong peers."""

        camera = str(camera)
        model = self._ensure_liveness_listener(camera)
        provenance = mavlink_message_provenance(message)
        system_id = provenance["mavlink_source_system"]
        component_id = provenance["mavlink_source_component"]
        message_type = message.get_type() if hasattr(message, "get_type") else ""
        # SiK/RFD local status frames are link diagnostics, not vehicle
        # identity. Accept them without allowing them to establish or refresh
        # expected-system liveness.
        if message_type == "RADIO_STATUS":
            return True
        expected_system_id = self.expected_system_ids[camera]
        if system_id is not None and int(system_id) != expected_system_id:
            mismatch = (camera, int(system_id))
            if mismatch not in self._identity_mismatch_reported:
                self._identity_mismatch_reported.add(mismatch)
                self.status_changed.emit(
                    f"{camera_label_ko(camera)} MAVLink identity 불일치: "
                    f"expected sysid {expected_system_id}, got {int(system_id)}"
                )
            return False
        liveness_message_type = message_type
        if message_type == "ATTITUDE" and getattr(message, "time_boot_ms", None) is not None:
            # Count parseable transport now, but let the Qt receive owner
            # classify the PX4 boot epoch before this packet refreshes ATTITUDE
            # liveness. Deferred command telemetry follows the same owner path.
            liveness_message_type = "ATTITUDE_PENDING_BOOT_CHECK"
        accepted = model.observe_packet(
            system_id=system_id,
            component_id=component_id,
            message_type=liveness_message_type,
            received_at_s=received_at_s,
            generation=model.connection_generation,
        )
        # Small legacy/test doubles may not expose MAVLink provenance.  They
        # can still exercise downstream decoders, but never establish link
        # identity or qualify the transport.
        return accepted or system_id is None

    def _send_due_gcs_heartbeat(
        self,
        camera: str,
        connection: Any,
        *,
        now_s: float,
        wait_for_send_lock: bool = True,
    ) -> None:
        camera = str(camera)
        model = self._ensure_liveness_listener(camera)
        health = model.snapshot(now_s=now_s)
        if not health.listener_open:
            return
        if (
            not health.identity_verified
            and _mavlink_udpin_endpoint(self.urls.get(camera, "")) is not None
            and camera not in self._seeded_udp_peers
        ):
            return
        mav = getattr(connection, "mav", None)
        sender = getattr(mav, "heartbeat_send", None)
        if not callable(sender):
            return
        mavlink = getattr(connection, "mavlink", None)
        send_lock = self._mavlink_send_locks.setdefault(camera, threading.RLock())
        acquired = send_lock.acquire(blocking=bool(wait_for_send_lock))
        if not acquired:
            return
        send_error: Exception | None = None
        try:
            # The due check and attempt marker share the same side lock as all
            # MAVLink writers. This prevents a command-owner poll and a GUI
            # timer edge from both emitting the same one-second heartbeat.
            last_attempt_s = self._gcs_heartbeat_last_attempt_s.get(camera)
            if (
                last_attempt_s is not None
                and now_s - last_attempt_s < MAVLINK_GCS_HEARTBEAT_INTERVAL_S
            ):
                return
            self._gcs_heartbeat_last_attempt_s[camera] = now_s
            try:
                sender(
                    _mavlink_constant(mavlink, "MAV_TYPE_GCS", 6),
                    _mavlink_constant(mavlink, "MAV_AUTOPILOT_INVALID", 8),
                    0,
                    0,
                    _mavlink_constant(mavlink, "MAV_STATE_ACTIVE", 4),
                )
            except Exception as exc:
                send_error = exc
        finally:
            send_lock.release()
        if send_error is not None:
            exc = send_error
            message = f"{type(exc).__name__}: {exc}"
            if should_emit_throttled_error(
                self._last_gcs_heartbeat_errors,
                camera,
                message,
                now_s,
                MAVLINK_GCS_HEARTBEAT_INTERVAL_S,
            ):
                self.status_changed.emit(
                    f"{camera_label_ko(camera)} GCS heartbeat 전송 실패: {exc}"
                )
            return
        self._last_gcs_heartbeat_errors.pop(camera, None)
        model.note_gcs_heartbeat_sent(
            sent_at_s=now_s,
            generation=health.connection_generation,
        )

    def _emit_liveness_transition(self, camera: str, *, now_s: float) -> None:
        health = self.mavlink_health_snapshot(str(camera), now_s=now_s)
        previous = self._last_health_states.get(str(camera))
        if previous is health.state:
            return
        self._last_health_states[str(camera)] = health.state
        detail = ""
        if health.state is MavlinkLivenessState.INITIAL_RX_TIMEOUT:
            detail = (
                f"; peer={health.peer_host or '--'}; listener age="
                f"{float(health.listener_age_s or 0.0):.1f}s; "
                "expected-sysid HEARTBEAT not received"
            )
        elif health.state is MavlinkLivenessState.RX_STALLED:
            detail = (
                f"; ANY age={health.any_rx_age_s:.1f}s; RTCM/commands blocked; "
                "PX4/NuttX UDP task deadlock suspected"
            )
        elif health.state in {MavlinkLivenessState.QUALIFYING, MavlinkLivenessState.RECOVERING}:
            elapsed_s = min(
                max(0, int(health.qualification_elapsed_s)),
                max(1, int(health.qualification_required_s)),
            )
            required_s = max(1, int(health.qualification_required_s))
            detail = (
                f"; qualification={elapsed_s // 60:02d}:{elapsed_s % 60:02d}/"
                f"{required_s // 60:02d}:{required_s % 60:02d}"
            )
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink liveness: {health.state.value}{detail}"
        )

    def _is_duplicate_packet(self, camera: str, message: Any) -> bool:
        """Drop an exact duplicate transport packet before any downstream signal."""

        reader = getattr(message, "get_msgbuf", None)
        if not callable(reader):
            return False
        try:
            raw = bytes(reader())
        except Exception:
            return False
        if not raw:
            return False
        now_s = time.monotonic()
        recent = self._recent_raw_packets.setdefault(
            str(camera), deque(maxlen=MAVLINK_DUPLICATE_CACHE_SIZE)
        )
        cutoff_s = now_s - MAVLINK_DUPLICATE_TTL_S
        while recent and recent[0][0] < cutoff_s:
            recent.popleft()
        duplicate = any(previous == raw for _timestamp_s, previous in recent)
        if not duplicate:
            recent.append((now_s, raw))
        return duplicate

    def _emit_receive_error(self, camera: str, exc: BaseException) -> None:
        message = f"{type(exc).__name__}: {exc}"
        now_s = time.monotonic()
        if not should_emit_throttled_error(
            self._last_receive_errors,
            str(camera),
            message,
            now_s,
            MAVLINK_RECEIVE_ERROR_INTERVAL_S,
        ):
            return
        self.status_changed.emit(
            f"{camera_label_ko(camera)} MAVLink 수신 실패: {exc}"
        )

    def _log_unrecognized_debug_vect(self, camera: str, message: Any) -> None:
        """Surface DEBUG_VECT names the CAN-mag decoder doesn't recognize.

        Only "CAN1_MAG"/"CAN2_MAG" are decoded (see
        _can_mag_sensor_state_from_debug_vect); any other firmware naming was
        previously dropped silently, making a firmware-side naming mismatch
        indistinguishable from no CAN mag data being sent at all.
        """

        name = getattr(message, "name", "")
        if isinstance(name, bytes):
            name = name.split(b"\0", 1)[0].decode("ascii", errors="ignore")
        name = str(name).strip(" \0")
        key = f"{camera}:{name}"
        if key in self._unrecognized_debug_vect_names:
            return
        self._unrecognized_debug_vect_names.add(key)
        self.status_changed.emit(
            f"{camera_label_ko(camera)} 인식되지 않는 DEBUG_VECT 이름 수신: '{name}' "
            "(CAN1_MAG/CAN2_MAG 아님 - CAN 자력계 데이터가 아닐 수 있음)"
        )

    def _observe_remote_px4_reboot(
        self,
        camera: str,
        message: Any,
        *,
        received_at_s: float,
        provenance: dict[str, int | None],
    ) -> bool:
        """Classify ATTITUDE and rearm after authoritative new-boot evidence.

        This method is called only from the subscriber's receive owner.  It
        never emits the persistent ``stream_setup_required`` signal and sends
        no command itself.  Clearing the ephemeral request marker lets the
        next expected-system HEARTBEAT use the existing one-shot Stock-profile
        path.
        """

        camera = str(camera)
        source_system = provenance.get("mavlink_source_system")
        if source_system != self.expected_system_ids.get(camera):
            return True
        time_boot_ms = getattr(message, "time_boot_ms", None)
        if time_boot_ms is None:
            return True
        connection = self.connections.get(camera)
        if connection is None:
            return True
        model = self._ensure_liveness_listener(camera)
        observation = self._mavlink_boot_time_trackers.setdefault(
            camera, _MavlinkBootTimeTracker()
        ).observe(time_boot_ms, received_at_s=received_at_s)
        if not observation.accept_for_downstream:
            return False
        evidence = observation.reboot
        if evidence is not None:
            self._mavlink_stream_requests_sent.discard(camera)
            self._mavlink_stream_request_attempts.pop(camera, None)
            self._mavlink_stream_last_request_s.pop(camera, None)
            self._clear_mavlink_imu_stream_state(camera, clear_samples=True)
            model.note_remote_reboot(now_s=received_at_s)
        model.observe_packet(
            system_id=source_system,
            component_id=provenance.get("mavlink_source_component"),
            message_type="ATTITUDE",
            received_at_s=received_at_s,
            generation=model.connection_generation,
        )
        if evidence is None:
            return True
        self._last_health_states.pop(camera, None)
        self.status_changed.emit(
            f"{camera_label_ko(camera)} PX4 재부팅 감지: ATTITUDE time_boot_ms "
            f"{evidence.previous_time_boot_ms}->{evidence.current_time_boot_ms}; "
            "liveness 재검증/Stock stream profile 1회 재요청 대기"
        )
        return True

    def _handle_mavlink_message(
        self,
        camera: str,
        message: Any,
        *,
        received_at_s: float | None = None,
        allow_heartbeat_actions: bool = True,
    ) -> None:
        message_type = message.get_type() if hasattr(message, "get_type") else ""
        namespace = self.urls.get(camera, "")
        now_s = time.monotonic() if received_at_s is None else float(received_at_s)
        provenance = mavlink_message_provenance(message)
        if message_type == "ATTITUDE" and not self._observe_remote_px4_reboot(
            camera,
            message,
            received_at_s=now_s,
            provenance=provenance,
        ):
            return
        self._emit_custom_watch_samples(camera, message, message_type, now_s)
        optional_row = _mavlink_optional_timeline_row(
            message_type,
            message,
            now_s,
            provenance=provenance,
        )
        if optional_row is not None:
            self.optional_timeline_changed.emit(camera, optional_row)
        if message_type == "RADIO_STATUS":
            self._radio_status[camera] = RadioStatusState(
                timestamp_s=now_s,
                rssi=_optional_int(message, "rssi"),
                remote_rssi=_optional_int(message, "remrssi"),
                noise=_optional_int(message, "noise"),
                remote_noise=_optional_int(message, "remnoise"),
                txbuf_pct=_optional_int(message, "txbuf"),
                rxerrors=_optional_int(message, "rxerrors"),
                fixed=_optional_int(message, "fixed"),
            )
            return
        if message_type == "ATTITUDE":
            self._last_attitude_message_s[camera] = now_s
            self._attitude_stream_request_attempts.pop(camera, None)
            self._attitude_stream_last_request_s.pop(camera, None)
            self._attitude_stream_outage_reported.discard(camera)
            extrinsic = camera_extrinsic_from_euler_rad(
                camera=camera,
                namespace=namespace,
                roll_rad=float(getattr(message, "roll")),
                pitch_rad=float(getattr(message, "pitch")),
                yaw_rad=float(getattr(message, "yaw")),
                timestamp_s=now_s,
            )
            time_boot_ms = getattr(message, "time_boot_ms", None)
            if time_boot_ms is not None:
                px4_timestamp_us = int(time_boot_ms) * 1000
                extrinsic = replace(
                    extrinsic,
                    px4_timestamp_s=float(px4_timestamp_us) * 1e-6,
                    px4_timestamp_us=px4_timestamp_us,
                )
            extrinsic = replace(
                extrinsic,
                rollspeed_rad_s=_optional_float(message, "rollspeed"),
                pitchspeed_rad_s=_optional_float(message, "pitchspeed"),
                yawspeed_rad_s=_optional_float(message, "yawspeed"),
                **provenance,
            )
            self.extrinsic_changed.emit(camera, extrinsic)
            return
        if message_type in {"GPS_RAW_INT", "GPS2_RAW"}:
            topic = f"mavlink/{message_type}"
            source_time_usec = getattr(message, "time_usec", None)
            source_time_us = int(source_time_usec) if source_time_usec is not None else None
            gps_utc_us = (
                source_time_us
                if source_time_us is not None and source_time_us >= 946_684_800_000_000
                else None
            )
            gps = GpsPositionState(
                camera=camera,
                namespace=namespace,
                timestamp_s=now_s,
                topic=topic,
                latitude_deg=_mavlink_scaled_degrees(getattr(message, "lat", None)),
                longitude_deg=_mavlink_scaled_degrees(getattr(message, "lon", None)),
                altitude_m=_mavlink_scaled_millimetres(getattr(message, "alt", None)),
                fix_type=_optional_int(message, "fix_type"),
                satellites_used=_optional_int(message, "satellites_visible"),
                # PX4 GPS_RAW_INT.eph/epv are HDOP/VDOP ×100. The actual
                # metre accuracies are carried by the h_acc/v_acc extensions.
                eph_m=_mavlink_scaled_accuracy_mm(getattr(message, "h_acc", None)),
                epv_m=_mavlink_scaled_accuracy_mm(getattr(message, "v_acc", None)),
                heading_deg=_mavlink_heading_cdeg(getattr(message, "cog", None)),
                raw=message,
                gps_utc_us=gps_utc_us,
                altitude_ellipsoid_m=_mavlink_scaled_millimetres(getattr(message, "alt_ellipsoid", None)),
                hdop=_mavlink_scaled_dop(getattr(message, "eph", None)),
                vdop=_mavlink_scaled_dop(getattr(message, "epv", None)),
                time_source="mavlink_time_usec_epoch" if gps_utc_us is not None else "mavlink_time_usec_unresolved",
                **provenance,
            )
            if message_type == "GPS_RAW_INT":
                self.gps_changed.emit(camera, gps)
            else:
                self.gps_changed.emit(f"{camera}_GPS2", gps)
            self.gps_sensor_changed.emit(
                camera,
                replace(
                    _gps_sensor_state_from_mavlink(camera, namespace, message, topic, now_s),
                    **provenance,
                ),
            )
            gnss_relative = _gnss_relative_state_from_mavlink_gps_raw(camera, namespace, message, topic, now_s)
            if gnss_relative is not None:
                self._emit_mavlink_gnss_relative(camera, replace(gnss_relative, **provenance))
            return
        if message_type in {"GPS_RTK", "GPS2_RTK"}:
            gnss_relative = _gnss_relative_state_from_mavlink_rtk(
                camera,
                namespace,
                message,
                f"mavlink/{message_type}",
                now_s,
            )
            if gnss_relative is not None:
                self._emit_mavlink_gnss_relative(camera, replace(gnss_relative, **provenance))
            return
        if message_type in {"SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"}:
            mag = _mag_sensor_state_from_mavlink(camera, namespace, message, message_type, now_s)
            if mag is not None:
                key = (camera, message_type)
                self._last_imu_stream_message_s[key] = now_s
                self._imu_stream_request_attempts.pop(key, None)
                self._imu_stream_last_request_s.pop(key, None)
                self._imu_stream_outage_reported.discard(key)
                self.mag_sensor_changed.emit(camera, replace(mag, **provenance))
            return
        if message_type in MAVLINK_CAN_MAG_MESSAGE_TYPES:
            if message_type == "SASL_CAN_MAG":
                self._sasl_can_mag_stream_confirmed.add(camera)
            mag = _can_mag_sensor_state_from_mavlink(camera, namespace, message, message_type, now_s)
            if mag is not None:
                self.mag_sensor_changed.emit(camera, replace(mag, **provenance))
            return
        if message_type == "DEBUG_VECT":
            mag = _can_mag_sensor_state_from_debug_vect(camera, namespace, message, now_s)
            if mag is not None:
                self.mag_sensor_changed.emit(camera, replace(mag, **provenance))
            else:
                self._log_unrecognized_debug_vect(camera, message)
            return
        if message_type == "HEARTBEAT":
            # Command-owned recv_match records liveness at ingress and queues
            # telemetry with that exact receive timestamp. Replaying an old
            # HEARTBEAT must preserve downstream order/logging, but must not
            # run current-time stream setup or recovery for every historical
            # packet. The next live HEARTBEAT owns those actions.
            if allow_heartbeat_actions:
                self._request_mavlink_sensor_streams(camera)
                self._request_missing_mavlink_imu_streams(camera)
                self._request_stale_attitude_stream(camera)
            if camera not in self._heartbeat_logged:
                self._heartbeat_logged.add(camera)
                self.status_changed.emit(f"{camera_label_ko(camera)} MAVLink heartbeat 수신")
            return
        if message_type == "SERIAL_CONTROL":
            output = _decode_mavlink_serial_control(message)
            if output:
                self.shell_output.emit(camera, output)

    def _emit_mavlink_gnss_relative(self, camera: str, state: GnssRelativeState) -> None:
        merged = _merge_gnss_relative_state(self._latest_gnss_relative.get(camera), state)
        self._latest_gnss_relative[camera] = merged
        self.gnss_relative_changed.emit(camera, merged)


    def _emit_custom_watch_samples(self, camera: str, message: Any, message_type: str, timestamp_s: float) -> None:
        if not self.custom_watches:
            return
        for spec in self.custom_watches:
            source = str(spec.get("source", "")).strip().lower()
            if source and source != "mavlink":
                continue
            target_message = str(spec.get("message", "") or spec.get("topic", "")).strip().upper()
            if target_message and target_message != message_type.upper():
                continue
            field_path = str(spec.get("field", "")).strip()
            if not field_path:
                continue
            try:
                value = extract_field_path(message, field_path)
            except Exception:
                continue
            numeric_value = _coerce_numeric(value)
            key = str(spec.get("key") or f"mavlink:{message_type}.{field_path}")
            self.custom_watch_changed.emit(
                CustomTopicSample(
                    key=f"{camera}:{key}",
                    topic=f"mavlink/{message_type}",
                    field_path=field_path,
                    value_text=_format_custom_value(value),
                    numeric_value=numeric_value,
                    timestamp_s=timestamp_s,
                )
            )



def _mavlink_optional_timeline_row(
    message_type: str,
    message: Any,
    timestamp_s: float,
    *,
    provenance: dict[str, int | None] | None = None,
) -> dict[str, object] | None:
    """Normalize optional MAVLink state messages for the session timeline."""
    row: dict[str, object] = {
        "sample_type": f"mavlink_{message_type.lower()}",
        "topic": f"mavlink/{message_type}",
        "sample_app_timestamp_s": timestamp_s,
        "mavlink_message_name": message_type,
    }
    row.update(provenance or mavlink_message_provenance(message))
    if message_type == "GLOBAL_POSITION_INT":
        row.update(
            {
                "mavlink_global_latitude_deg": _mavlink_scaled_degrees(getattr(message, "lat", None)),
                "mavlink_global_longitude_deg": _mavlink_scaled_degrees(getattr(message, "lon", None)),
                "mavlink_global_altitude_m": _mavlink_scaled_millimetres(getattr(message, "alt", None)),
                "mavlink_global_relative_altitude_m": _mavlink_scaled_millimetres(getattr(message, "relative_alt", None)),
                "mavlink_global_velocity_n_m_s": _mavlink_scaled_centimetres(getattr(message, "vx", None)),
                "mavlink_global_velocity_e_m_s": _mavlink_scaled_centimetres(getattr(message, "vy", None)),
                "mavlink_global_velocity_d_m_s": _mavlink_scaled_centimetres(getattr(message, "vz", None)),
                "mavlink_global_heading_deg": _mavlink_heading_cdeg(getattr(message, "hdg", None)),
                "mavlink_time_boot_ms": _optional_int(message, "time_boot_ms"),
            }
        )
    elif message_type == "LOCAL_POSITION_NED":
        row.update(
            {
                "mavlink_local_x_m": _optional_float(message, "x"),
                "mavlink_local_y_m": _optional_float(message, "y"),
                "mavlink_local_z_m": _optional_float(message, "z"),
                "mavlink_local_velocity_x_m_s": _optional_float(message, "vx"),
                "mavlink_local_velocity_y_m_s": _optional_float(message, "vy"),
                "mavlink_local_velocity_z_m_s": _optional_float(message, "vz"),
                "mavlink_time_boot_ms": _optional_int(message, "time_boot_ms"),
            }
        )
    elif message_type == "VFR_HUD":
        row.update(
            {
                "mavlink_vfr_airspeed_m_s": _optional_float(message, "airspeed"),
                "mavlink_vfr_groundspeed_m_s": _optional_float(message, "groundspeed"),
                "mavlink_vfr_heading_deg": _optional_float(message, "heading"),
                "mavlink_vfr_throttle_pct": _optional_float(message, "throttle"),
                "mavlink_vfr_altitude_m": _optional_float(message, "alt"),
                "mavlink_vfr_climb_m_s": _optional_float(message, "climb"),
            }
        )
    elif message_type == "SYS_STATUS":
        row.update(
            {
                "mavlink_sys_voltage_battery_v": _scaled_mavlink_value(message, "voltage_battery", 1e-3),
                "mavlink_sys_current_battery_a": _scaled_mavlink_value(message, "current_battery", 1e-2),
                "mavlink_sys_battery_remaining_pct": _optional_float(message, "battery_remaining"),
                "mavlink_sys_drop_rate_comm_pct": _scaled_mavlink_value(message, "drop_rate_comm", 1e-2),
                "mavlink_sys_errors_comm": _optional_int(message, "errors_comm"),
            }
        )
    elif message_type == "BATTERY_STATUS":
        row.update(
            {
                "mavlink_battery_current_a": _scaled_mavlink_value(message, "current_battery", 1e-2),
                "mavlink_battery_remaining_pct": _optional_float(message, "battery_remaining"),
                "mavlink_battery_temperature_c": _scaled_mavlink_value(message, "temperature", 1e-2),
                "mavlink_battery_voltage_0_v": _scaled_mavlink_array_value(message, "voltages", 0, 1e-3),
                "mavlink_battery_voltage_1_v": _scaled_mavlink_array_value(message, "voltages", 1, 1e-3),
                "mavlink_battery_voltage_2_v": _scaled_mavlink_array_value(message, "voltages", 2, 1e-3),
            }
        )
    elif message_type == "RAW_IMU":
        row.update(
            {
                "mavlink_raw_accel_x": _optional_float(message, "xacc"),
                "mavlink_raw_accel_y": _optional_float(message, "yacc"),
                "mavlink_raw_accel_z": _optional_float(message, "zacc"),
                "mavlink_raw_gyro_x": _optional_float(message, "xgyro"),
                "mavlink_raw_gyro_y": _optional_float(message, "ygyro"),
                "mavlink_raw_gyro_z": _optional_float(message, "zgyro"),
                "mavlink_raw_mag_x": _optional_float(message, "xmag"),
                "mavlink_raw_mag_y": _optional_float(message, "ymag"),
                "mavlink_raw_mag_z": _optional_float(message, "zmag"),
                "mavlink_time_usec": _optional_int(message, "time_usec"),
            }
        )
    elif message_type == "RADIO_STATUS":
        row.update(
            {
                "mavlink_radio_rssi": _optional_int(message, "rssi"),
                "mavlink_radio_remote_rssi": _optional_int(message, "remrssi"),
                "mavlink_radio_noise": _optional_int(message, "noise"),
                "mavlink_radio_remote_noise": _optional_int(message, "remnoise"),
                "mavlink_radio_txbuf_pct": _optional_int(message, "txbuf"),
                "mavlink_radio_rxerrors": _optional_int(message, "rxerrors"),
                "mavlink_radio_fixed": _optional_int(message, "fixed"),
            }
        )
    else:
        return None
    nonfinite_reasons = _mavlink_nonfinite_source_reasons(message_type, message)
    if nonfinite_reasons:
        row["nonfinite_source_fields"] = sorted(nonfinite_reasons)
        row["nonfinite_source_reasons"] = {
            name: nonfinite_reasons[name] for name in sorted(nonfinite_reasons)
        }
    return row


def _scaled_mavlink_value(message: Any, name: str, scale: float) -> float | None:
    value = _optional_float(message, name)
    return None if value is None or value < 0 else value * scale


def _scaled_mavlink_array_value(message: Any, name: str, index: int, scale: float) -> float | None:
    values = getattr(message, name, None)
    if not isinstance(values, (list, tuple)) or index >= len(values):
        return None
    value = _finite_float(values[index])
    if value is None:
        return None
    return None if value in (0, 65535) else value * scale


def mavlink_graphable_fields_for_message(message_name: str) -> list[str]:
    name = message_name.upper().strip()
    known = {
        "ATTITUDE": ["roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed", "time_boot_ms"],
        "GPS_RAW_INT": ["lat", "lon", "alt", "eph", "epv", "vel", "cog", "fix_type", "satellites_visible", "yaw", "hdg_acc", "time_usec"],
        "GPS2_RAW": ["lat", "lon", "alt", "eph", "epv", "vel", "cog", "fix_type", "satellites_visible", "yaw", "hdg_acc", "time_usec"],
        "GPS_RTK": ["baseline_a_mm", "baseline_b_mm", "baseline_c_mm", "baseline_coords_type", "accuracy", "rtk_receiver_id", "time_last_baseline_ms"],
        "GPS2_RTK": ["baseline_a_mm", "baseline_b_mm", "baseline_c_mm", "baseline_coords_type", "accuracy", "rtk_receiver_id", "time_last_baseline_ms"],
        "GLOBAL_POSITION_INT": ["lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg", "time_boot_ms"],
        "SCALED_IMU": ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "temperature", "time_boot_ms"],
        "SCALED_IMU2": ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "temperature", "time_boot_ms"],
        "SCALED_IMU3": ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "temperature", "time_boot_ms"],
        "LOCAL_POSITION_NED": ["x", "y", "z", "vx", "vy", "vz", "time_boot_ms"],
        "VFR_HUD": ["airspeed", "groundspeed", "heading", "throttle", "alt", "climb"],
        "SYS_STATUS": ["voltage_battery", "current_battery", "battery_remaining", "drop_rate_comm", "errors_comm"],
        "BATTERY_STATUS": ["current_battery", "battery_remaining", "temperature", "voltages[0]", "voltages[1]", "voltages[2]"],
        "RAW_IMU": ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "time_usec"],
    }
    return known.get(name, [])


def discover_mavlink_messages() -> list[dict[str, str]]:
    names = (
        "ATTITUDE",
        "GPS_RAW_INT",
        "GPS2_RAW",
        "GPS_RTK",
        "GPS2_RTK",
        "GLOBAL_POSITION_INT",
        "SCALED_IMU",
        "SCALED_IMU2",
        "SCALED_IMU3",
        "LOCAL_POSITION_NED",
        "VFR_HUD",
        "SYS_STATUS",
        "BATTERY_STATUS",
        "RAW_IMU",
    )
    return [{"name": name, "type": "MAVLink"} for name in names]


def _merge_gnss_relative_state(
    previous: GnssRelativeState | None,
    current: GnssRelativeState,
    *,
    max_merge_age_s: float = 3.0,
) -> GnssRelativeState:
    if previous is None or current.timestamp_s - previous.timestamp_s > max_merge_age_s:
        return current
    preserved = {}
    for field_name in (
        "device_id",
        "heading_rad",
        "heading_accuracy_rad",
        "position_length_m",
        "accuracy_length_m",
        "gnss_fix_ok",
        "differential_solution",
        "relative_position_valid",
        "carrier_solution_floating",
        "carrier_solution_fixed",
        "moving_base_mode",
        "heading_valid",
        "px4_timestamp_s",
        "px4_timestamp_us",
        "px4_timestamp_sample_us",
        "gps_utc_us",
        "mavlink_source_system",
        "mavlink_source_component",
        "mavlink_sequence",
        "mavlink_message_id",
    ):
        if getattr(current, field_name) is None:
            preserved[field_name] = getattr(previous, field_name)
    return replace(current, **preserved) if preserved else current


def _gnss_relative_state_from_mavlink_gps_raw(
    camera: str,
    namespace: str,
    message: Any,
    topic: str,
    timestamp_s: float,
) -> GnssRelativeState | None:
    fix_type = _optional_int(message, "fix_type")
    heading_rad = _mavlink_yaw_cdeg_to_rad(getattr(message, "yaw", None))
    heading_valid = heading_rad is not None
    if fix_type is None and not heading_valid:
        return None
    return GnssRelativeState(
        camera=camera,
        namespace=namespace,
        timestamp_s=timestamp_s,
        topic=topic,
        heading_rad=heading_rad,
        heading_accuracy_rad=_mavlink_hdg_acc_deg_e5_to_rad(getattr(message, "hdg_acc", None)),
        gnss_fix_ok=None if fix_type is None else fix_type >= 3,
        differential_solution=None if fix_type is None else fix_type >= 4,
        relative_position_valid=heading_valid,
        carrier_solution_floating=None if fix_type is None else fix_type == 5,
        carrier_solution_fixed=None if fix_type is None else fix_type == 6,
        moving_base_mode=heading_valid,
        heading_valid=heading_valid,
    )


def _gnss_relative_state_from_mavlink_rtk(
    camera: str,
    namespace: str,
    message: Any,
    topic: str,
    timestamp_s: float,
) -> GnssRelativeState | None:
    baseline_a_mm = _optional_float(message, "baseline_a_mm")
    baseline_b_mm = _optional_float(message, "baseline_b_mm")
    baseline_c_mm = _optional_float(message, "baseline_c_mm")
    if baseline_a_mm is None or baseline_b_mm is None or baseline_c_mm is None:
        return None
    position_length_m = math.sqrt(
        baseline_a_mm * baseline_a_mm + baseline_b_mm * baseline_b_mm + baseline_c_mm * baseline_c_mm
    ) * 1e-3
    baseline_coords_type = _optional_int(message, "baseline_coords_type")
    heading_rad = None
    if baseline_coords_type == 1:
        heading_rad = math.atan2(baseline_b_mm, baseline_a_mm)
        if heading_rad < 0.0:
            heading_rad += math.tau
    accuracy_mm = _optional_float(message, "accuracy")
    relative_position_valid = position_length_m > 0.0
    return GnssRelativeState(
        camera=camera,
        namespace=namespace,
        timestamp_s=timestamp_s,
        topic=topic,
        device_id=_optional_int(message, "rtk_receiver_id"),
        heading_rad=heading_rad,
        position_length_m=position_length_m,
        accuracy_length_m=None if accuracy_mm is None else accuracy_mm * 1e-3,
        gnss_fix_ok=relative_position_valid,
        differential_solution=relative_position_valid,
        relative_position_valid=relative_position_valid,
        moving_base_mode=relative_position_valid,
        heading_valid=heading_rad is not None,
    )


def _gps_sensor_state_from_mavlink(
    camera: str, namespace: str, message: Any, topic: str, timestamp_s: float
) -> GpsSensorState:
    slot = "GPS2" if topic.endswith("GPS2_RAW") else "GPS1"
    source_time_usec = getattr(message, "time_usec", None)
    source_time_us = int(source_time_usec) if source_time_usec is not None else None
    gps_utc_us = (
        source_time_us
        if source_time_us is not None and source_time_us >= 946_684_800_000_000
        else None
    )
    return GpsSensorState(
        camera=camera,
        namespace=namespace,
        sensor_slot=slot,
        timestamp_s=timestamp_s,
        topic=topic,
        device_id=None,
        fix_type=_optional_int(message, "fix_type"),
        satellites_used=_optional_int(message, "satellites_visible"),
        eph_m=_mavlink_scaled_accuracy_mm(getattr(message, "h_acc", None)),
        epv_m=_mavlink_scaled_accuracy_mm(getattr(message, "v_acc", None)),
        hdop=_mavlink_scaled_dop(getattr(message, "eph", None)),
        vdop=_mavlink_scaled_dop(getattr(message, "epv", None)),
        px4_timestamp_s=None,
        gps_utc_us=gps_utc_us,
    )


def _mag_sensor_state_from_mavlink(
    camera: str, namespace: str, message: Any, message_type: str, timestamp_s: float
) -> MagSensorState | None:
    x = _optional_float(message, "xmag")
    y = _optional_float(message, "ymag")
    z = _optional_float(message, "zmag")
    if x is None or y is None or z is None:
        return None
    slot = MAVLINK_MAG_SLOT_BY_MESSAGE.get(message_type, "CAN?")
    return MagSensorState(
        camera=camera,
        namespace=namespace,
        sensor_slot=slot,
        timestamp_s=timestamp_s,
        topic=f"mavlink/{message_type}",
        device_id=None,
        x=x,
        y=y,
        z=z,
        magnitude=math.sqrt(x * x + y * y + z * z),
        temperature_c=_optional_float(message, "temperature"),
        px4_timestamp_s=None,
        accel_x=_optional_float(message, "xacc"),
        accel_y=_optional_float(message, "yacc"),
        accel_z=_optional_float(message, "zacc"),
        gyro_x=_optional_float(message, "xgyro"),
        gyro_y=_optional_float(message, "ygyro"),
        gyro_z=_optional_float(message, "zgyro"),
        time_boot_ms=_optional_int(message, "time_boot_ms"),
    )


def _can_mag_sensor_state_from_mavlink(
    camera: str, namespace: str, message: Any, message_type: str, timestamp_s: float
) -> MagSensorState | None:
    """Decode a firmware-provided per-CAN magnetometer MAVLink message.

    This deliberately accepts only a dedicated CAN message family.  The
    standard SCALED_IMU* messages are handled separately because their stream
    number identifies an onboard IMU, not a CAN port.
    """

    x = _optional_float(message, "xmag")
    y = _optional_float(message, "ymag")
    z = _optional_float(message, "zmag")
    if x is None or y is None or z is None:
        return None

    raw_port = getattr(message, "can_port", getattr(message, "bus", getattr(message, "port", None)))
    port_text = str(raw_port or "").strip().upper()
    if port_text.startswith("CAN"):
        sensor_slot = port_text
    else:
        port_number = _optional_int(message, "can_port")
        if port_number is None:
            port_number = _optional_int(message, "bus")
        sensor_slot = f"CAN{port_number}" if port_number is not None else "CAN?"

    timestamp_us = _optional_int(message, "time_usec")
    if timestamp_us is None:
        timestamp_us = _optional_int(message, "timestamp")
    return MagSensorState(
        camera=camera,
        namespace=namespace,
        sensor_slot=sensor_slot,
        timestamp_s=timestamp_s,
        topic=f"mavlink/{message_type}",
        device_id=_optional_int(message, "device_id"),
        node_id=_optional_int(message, "node_id"),
        x=x,
        y=y,
        z=z,
        magnitude=math.sqrt(x * x + y * y + z * z),
        temperature_c=_optional_float(message, "temperature"),
        px4_timestamp_s=None if timestamp_us is None else float(timestamp_us) * 1e-6,
    )


def _can_mag_sensor_state_from_debug_vect(
    camera: str, namespace: str, message: Any, timestamp_s: float
) -> MagSensorState | None:
    """Decode the PX4 1.17 CAN-RM3100 DEBUG_VECT compatibility stream."""

    name = getattr(message, "name", "")
    if isinstance(name, bytes):
        name = name.split(b"\0", 1)[0].decode("ascii", errors="ignore")
    name = str(name).strip(" \0")
    known = {
        "CAN1_MAG": ("CAN1", RM3100_PRIMARY_IDS[0], 0x7D),
        "CAN2_MAG": ("CAN2", RM3100_BACKUP_IDS[0], 0x7B),
    }
    identity = known.get(name)
    if identity is None:
        return None
    sensor_slot, device_id, node_id = identity
    x = _optional_float(message, "x")
    y = _optional_float(message, "y")
    z = _optional_float(message, "z")
    if x is None or y is None or z is None:
        return None
    timestamp_us = _optional_int(message, "time_usec")
    return MagSensorState(
        camera=camera,
        namespace=namespace,
        sensor_slot=sensor_slot,
        timestamp_s=timestamp_s,
        topic="mavlink/DEBUG_VECT",
        device_id=device_id,
        node_id=node_id,
        x=x,
        y=y,
        z=z,
        magnitude=math.sqrt(x * x + y * y + z * z),
        px4_timestamp_s=None if timestamp_us is None else float(timestamp_us) * 1e-6,
    )


def _mavlink_scaled_degrees(value: Any) -> float | None:
    numeric = _finite_float(value)
    return None if numeric is None else numeric * 1e-7


def _mavlink_scaled_millimetres(value: Any) -> float | None:
    numeric = _finite_float(value)
    return None if numeric is None else numeric * 1e-3


def _mavlink_scaled_centimetres(value: Any) -> float | None:
    numeric = _finite_float(value)
    if numeric is None or int(numeric) == 65535:
        return None
    return numeric * 1e-2


def _mavlink_scaled_dop(value: Any) -> float | None:
    """Decode PX4 GPS_RAW_INT eph/epv, which carry HDOP/VDOP multiplied by 100."""

    numeric = _finite_float(value)
    if numeric is None or int(numeric) == 65535:
        return None
    return numeric * 1e-2


def _mavlink_scaled_accuracy_mm(value: Any) -> float | None:
    """Decode GPS_RAW_INT h_acc/v_acc from millimetres to metres."""

    numeric = _finite_float(value)
    if numeric is None or int(numeric) in {0, 4294967295}:
        return None
    return numeric * 1e-3


def _mavlink_heading_cdeg(value: Any) -> float | None:
    numeric = _finite_float(value)
    if numeric is None or int(numeric) == 65535:
        return None
    return numeric * 1e-2


def _mavlink_yaw_cdeg_to_rad(value: Any) -> float | None:
    numeric = _finite_float(value)
    if numeric is None:
        return None
    yaw_cdeg = int(numeric)
    if yaw_cdeg == 0 or yaw_cdeg == 65535:
        return None
    if yaw_cdeg == 36000:
        yaw_cdeg = 0
    return math.radians((yaw_cdeg % 36000) * 1e-2)


def _mavlink_hdg_acc_deg_e5_to_rad(value: Any) -> float | None:
    numeric = _finite_float(value)
    if numeric is None or int(numeric) == 4294967295:
        return None
    return math.radians(numeric * 1e-5)


def _serial_control_output_from_message(message: Any) -> str:
    try:
        count = int(getattr(message, "count", 0) or 0)
        data = bytes(getattr(message, "data", [])[:count])
    except Exception:
        return ""
    return data.decode("utf-8", errors="replace")


def _mavlink_message_type(message: Any) -> str:
    getter = getattr(message, "get_type", None)
    if callable(getter):
        try:
            return str(getter() or "")
        except Exception:
            return ""
    return str(getattr(message, "type", "") or "")


def _read_mavlink_shell_output(
    connection: Any,
    *,
    timeout_s: float,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> str:
    recv_match = getattr(connection, "recv_match", None)
    if recv_match is None:
        return ""
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    chunks: list[str] = []
    while time.monotonic() < deadline:
        if on_receive_poll is not None:
            on_receive_poll()
        message = recv_match(blocking=True, timeout=MAVLINK_COMMAND_RECEIVE_POLL_S)
        if on_receive_poll is not None:
            on_receive_poll()
        if message is None:
            continue
        if _mavlink_message_type(message) != "SERIAL_CONTROL":
            if on_unmatched is not None:
                on_unmatched(message)
            continue
        text = _serial_control_output_from_message(message)
        if text:
            chunks.append(text)
    return "".join(chunks)


def _send_mavlink_shell_payload(mav: Any, *, device: int, flags: int, payload: bytes) -> None:
    for offset in range(0, len(payload), 70):
        chunk = payload[offset : offset + 70]
        data = list(chunk) + [0] * (70 - len(chunk))
        mav.serial_control_send(device, flags, 0, 0, len(chunk), data)


def _send_mavlink_shell_command(
    connection: Any,
    command: str,
    *,
    timeout_s: float = 0.8,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> MavlinkShellCommandResult:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "serial_control_send"):
        raise RuntimeError("MAVLink serial control을 지원하지 않는 연결입니다.")
    mavlink = getattr(connection, "mavlink", None)
    device = _mavlink_constant(mavlink, "SERIAL_CONTROL_DEV_SHELL", 10)
    respond_flag = _mavlink_constant(mavlink, "SERIAL_CONTROL_FLAG_RESPOND", 2)
    exclusive_flag = _mavlink_constant(mavlink, "SERIAL_CONTROL_FLAG_EXCLUSIVE", 4)
    variants = (
        ("respond+exclusive", respond_flag | exclusive_flag),
        ("respond", respond_flag),
    )
    try:
        payload = (command.rstrip() + "\n").encode("utf-8")
        selected_label, selected_flags = variants[-1]
        probe_output = ""
        for label, flags in variants:
            # Opening the shell first matters on PX4 NSH links that only respond after a read request.
            mav.serial_control_send(device, flags, 0, 0, 0, [0] * 70)
            output = _read_mavlink_shell_output(
                connection,
                timeout_s=min(timeout_s, 0.25),
                on_unmatched=on_unmatched,
                on_receive_poll=on_receive_poll,
            )
            if output or label == variants[-1][0]:
                selected_label = label
                selected_flags = flags
                probe_output = output
                break
        _send_mavlink_shell_payload(mav, device=device, flags=selected_flags, payload=payload)
        command_output = _read_mavlink_shell_output(
            connection,
            timeout_s=timeout_s,
            on_unmatched=on_unmatched,
            on_receive_poll=on_receive_poll,
        )
        return MavlinkShellCommandResult(
            command=command,
            output=probe_output + command_output,
            flag_label=selected_label,
        )
    finally:
        # SERIAL_CONTROL_FLAG_EXCLUSIVE must be explicitly released. Closing
        # the host-side serial descriptor alone does not hand the PX4 shell
        # back and can leave later USB diagnostics with a dead-looking link.
        try:
            mav.serial_control_send(device, 0, 0, 0, 0, [0] * 70)
        except Exception:
            pass


def _send_mavlink_rtcm_data(connection: Any, payload: bytes) -> int:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "gps_rtcm_data_send"):
        raise RuntimeError("MAVLink GPS_RTCM_DATA를 지원하지 않는 연결입니다.")
    data = bytes(payload)
    if not data:
        return 0
    max_chunk = 180
    max_fragmented_payload = max_chunk * 4
    if len(data) > max_fragmented_payload:
        raise ValueError(f"RTCM frame too large for MAVLink GPS_RTCM_DATA: {len(data)} > {max_fragmented_payload} bytes")
    chunks = [data[offset : offset + max_chunk] for offset in range(0, len(data), max_chunk)]
    sequence_id = int(time.monotonic() * 1000) & 0x1F
    fragmented = len(chunks) > 1
    for fragment_id, chunk in enumerate(chunks):
        flags = 0
        if fragmented:
            flags = 1 | ((fragment_id & 0x03) << 1) | ((sequence_id & 0x1F) << 3)
        mav.gps_rtcm_data_send(flags, len(chunk), list(chunk) + [0] * (max_chunk - len(chunk)))
    return len(chunks)


def _write_mavlink_ftp_file(connection: Any, remote_path: str, content: str | bytes) -> None:
    try:
        from pymavlink import mavftp
        from pymavlink.mavftp import FtpError
    except Exception as exc:
        raise RuntimeError(f"pymavlink MAVFTP를 사용할 수 없습니다. ({exc})") from exc
    if not remote_path.startswith("/"):
        raise ValueError(f"MAVLink FTP remote path must be absolute: {remote_path}")
    payload = content.encode("utf-8") if isinstance(content, str) else bytes(content)
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    ftp = mavftp.MAVFTP(connection, target_system, target_component)
    directory = str(Path(remote_path).parent)
    if directory and directory not in {"/", "/fs/microsd"}:
        mkdir_result = ftp.cmd_mkdir([directory])
        if mkdir_result.return_code not in (FtpError.Success, FtpError.FileExists):
            raise RuntimeError(f"MAVLink FTP mkdir 실패: {directory} ({mkdir_result.return_code})")
    rm_result = ftp.cmd_rm([remote_path])
    if rm_result.return_code not in (FtpError.Success, FtpError.FileNotFound):
        raise RuntimeError(f"MAVLink FTP 기존 파일 삭제 실패: {remote_path} ({rm_result.return_code})")
    handle = io.BytesIO(payload)
    create_result = ftp.cmd_put(["extras.txt", remote_path], fh=handle)
    if create_result.return_code != FtpError.Success:
        raise RuntimeError(f"MAVLink FTP 파일 생성 실패: {remote_path} ({create_result.return_code})")
    write_result = ftp.process_ftp_reply("WriteFile", timeout=8)
    if write_result.return_code != FtpError.Success:
        raise RuntimeError(f"MAVLink FTP 파일 쓰기 실패: {remote_path} ({write_result.return_code})")


def _mavlink_param_id(name: str) -> bytes:
    encoded = name.encode("ascii", errors="strict")
    if len(encoded) > 16:
        raise ValueError(f"MAVLink PARAM id is longer than 16 bytes: {name}")
    return encoded


def _mavlink_param_id_text(message: Any) -> str:
    raw = getattr(message, "param_id", "")
    if isinstance(raw, bytes):
        return raw.rstrip(b"\x00").decode("ascii", errors="replace")
    return str(raw).rstrip("\x00")


def _mavlink_int_to_param_float(value: int) -> float:
    return struct.unpack("<f", struct.pack("<I", int(value) & 0xFFFFFFFF))[0]


def _mavlink_param_float_to_int(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def _mavlink_recv_param_value_loop(
    connection: Any,
    name: str,
    *,
    timeout_s: float,
    decode: Callable[[float, int | None], int | float],
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | float | None:
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        if on_receive_poll is not None:
            on_receive_poll()
        message = connection.recv_match(
            blocking=True,
            timeout=min(MAVLINK_COMMAND_RECEIVE_POLL_S, remaining),
        )
        if on_receive_poll is not None:
            on_receive_poll()
        if message is None:
            continue
        if (
            _mavlink_message_type(message) != "PARAM_VALUE"
            or _mavlink_param_id_text(message) != name
        ):
            if on_unmatched is not None:
                on_unmatched(message)
            continue
        return decode(getattr(message, "param_value"), getattr(message, "param_type", None))
    return None


def _mavlink_recv_param_value(
    connection: Any,
    name: str,
    *,
    timeout_s: float,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | None:
    return _mavlink_recv_param_value_loop(
        connection,
        name,
        timeout_s=timeout_s,
        decode=lambda raw_param_value, _param_type: _mavlink_param_float_to_int(raw_param_value),
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


# MAV_PARAM_TYPE_REAL32 = 9, MAV_PARAM_TYPE_REAL64 = 10. Every other MAV_PARAM_TYPE is an
# integer whose bits are reinterpreted (not numerically cast) into the PARAM_VALUE float field,
# per the MAVLink param protocol convention PX4 uses.
_MAVLINK_FLOAT_PARAM_TYPES = (9, 10)


def _mavlink_decode_param_value(raw_param_value: float, param_type: int | None) -> int | float:
    if param_type in _MAVLINK_FLOAT_PARAM_TYPES:
        return float(raw_param_value)
    return _mavlink_param_float_to_int(raw_param_value)


def _mavlink_recv_param_value_typed(
    connection: Any,
    name: str,
    *,
    timeout_s: float,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | float | None:
    return _mavlink_recv_param_value_loop(
        connection,
        name,
        timeout_s=timeout_s,
        decode=_mavlink_decode_param_value,
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _mavlink_read_param(
    connection: Any,
    name: str,
    *,
    timeout_s: float = 2.0,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | float | None:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "param_request_read_send"):
        return None
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    mav.param_request_read_send(target_system, target_component, _mavlink_param_id(name), -1)
    return _mavlink_recv_param_value_typed(
        connection,
        name,
        timeout_s=timeout_s,
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _mavlink_read_param_int(
    connection: Any,
    name: str,
    *,
    timeout_s: float = 2.0,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | None:
    if on_unmatched is None and hasattr(connection, "param_fetch_one"):
        try:
            value = connection.param_fetch_one(name, timeout=timeout_s)
        except TypeError:
            value = connection.param_fetch_one(name)
        except Exception:
            value = None
        if value is not None:
            try:
                return _mavlink_param_float_to_int(value)
            except Exception:
                try:
                    return int(value)
                except Exception:
                    pass
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "param_request_read_send"):
        return None
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    mav.param_request_read_send(target_system, target_component, _mavlink_param_id(name), -1)
    return _mavlink_recv_param_value(
        connection,
        name,
        timeout_s=timeout_s,
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _mavlink_set_param_int(
    connection: Any,
    name: str,
    value: int,
    *,
    timeout_s: float = 2.0,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | None:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "param_set_send"):
        raise RuntimeError("MAVLink PARAM_SET을 지원하지 않는 연결입니다.")
    mavlink = getattr(connection, "mavlink", None)
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    param_type = _mavlink_constant(mavlink, "MAV_PARAM_TYPE_INT32", 6)
    mav.param_set_send(
        target_system,
        target_component,
        _mavlink_param_id(name),
        _mavlink_int_to_param_float(int(value)),
        param_type,
    )
    return _mavlink_recv_param_value(
        connection,
        name,
        timeout_s=timeout_s,
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _decode_mavlink_serial_control(message: Any) -> str:
    count = int(getattr(message, "count", 0) or 0)
    data = getattr(message, "data", b"")
    if isinstance(data, bytes):
        payload = data[:count]
    else:
        payload = bytes(int(value) & 0xFF for value in list(data)[:count])
    return payload.replace(b"\x00", b"").decode("utf-8", errors="replace")


def _mavlink_constant(mavlink: Any, name: str, fallback: int) -> int:
    if mavlink is not None and hasattr(mavlink, name):
        return int(getattr(mavlink, name))
    return fallback
