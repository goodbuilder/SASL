"""Read-only Pixhawk message projections without Qt or transport ownership.

Attitude inputs are radians; q is wxyz rotating body FRD to NED and Euler
outputs are degrees. LOCAL_POSITION_NED x/y/z alias north/east/down in metres.
GPS_RTK a/b/c in millimetres yield a heading only for NED (coords_type=1);
other frames retain a length only. MAG axes remain the source sensor axes.
GPS integer angular/length scaling, sentinels and source timestamp provenance
are preserved. No camera RDF or baseline-center conversion happens here.

Callers pass receipt timestamps to decode captured inputs deterministically.
The two legacy convenience helpers still default to host monotonic time when
timestamp_s is omitted; this is not an exposure, boot or GPS UTC timestamp.
"""
from __future__ import annotations

import math
import re
import time
from dataclasses import replace
from typing import Any

from Workstation.workstation_core.px4_mag_manager import (
    RM3100_BACKUP_IDS,
    RM3100_PRIMARY_IDS,
)
from .types import CameraExtrinsicState, EulerAttitude, GnssRelativeState, GpsSensorState, MagSensorState


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
