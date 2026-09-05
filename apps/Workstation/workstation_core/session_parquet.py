"""Typed, lossless derived Parquet artifacts for finalized measurements.

The real-time recorder may append JSONL rows while a session is active.  This
module owns the other side of that boundary: it converts finalized JSON-safe
rows into typed Parquet files, verifies a complete readback, and only then
atomically replaces the published destination.

Every Parquet row contains two representations of the same logical record:

* stable typed columns for common filtering and analysis; and
* ``row_json``, a canonical JSON object containing every source field.

The canonical JSONL fragments owned by ``recording_session`` are authoritative.
The typed projection is deliberately a derived analysis view; ``row_json``
still prevents a newly added device or solver field from disappearing merely
because an older writer does not yet have a dedicated Arrow column for it.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

import pyarrow as pa
import pyarrow.parquet as pq


ROW_JSON_CONTRACT = "workstation-canonical-json-object-v1"
PARQUET_FORMAT_VERSION = 1
PARQUET_BATCH_ROWS = 2_048

MEASUREMENT_PAIRS_FILENAME = "measurement_pairs.parquet"
CAMERA_TIMELINE_FILENAME = "camera_timeline.parquet"
PX4_TIMELINE_FILENAME = "pixhawk_timeline.parquet"
GNSS_BASE_TIMELINE_FILENAME = "gnss_base_timeline.parquet"
GEOMETRY_COMPARISON_FILENAME = "geometry_comparison.parquet"


class ParquetFinalizationError(OSError):
    """Raised when a staged Parquet artifact cannot be verified."""


@dataclass(frozen=True)
class ParquetArtifactMetadata:
    """Verified metadata for one published Parquet artifact."""

    path: Path
    role: str
    row_count: int
    size_bytes: int
    sha256: str
    schema: dict[str, object]
    schema_sha256: str
    authoritative: bool = False
    verified: bool = True

    def to_mapping(self, *, relative_to: Path | None = None) -> dict[str, object]:
        path = self.path
        if relative_to is not None:
            try:
                path = path.relative_to(relative_to)
            except ValueError:
                pass
        return {
            "role": self.role,
            "media_type": "application/vnd.apache.parquet",
            "relative_path": path.as_posix(),
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "row_count": self.row_count,
            "schema_version": self.schema.get("version"),
            "authoritative": self.authoritative,
            "derived_from": [],
            "schema": self.schema,
            "schema_sha256": self.schema_sha256,
            "verified": self.verified,
        }


@dataclass(frozen=True)
class SessionParquetArtifacts:
    """The five typed artifacts produced for one finalized measurement."""

    measurement_pairs: ParquetArtifactMetadata
    camera_timeline: ParquetArtifactMetadata
    pixhawk_timeline: ParquetArtifactMetadata
    gnss_base_timeline: ParquetArtifactMetadata
    geometry_comparison: ParquetArtifactMetadata

    def to_mapping(self, *, relative_to: Path | None = None) -> dict[str, object]:
        return {
            "measurement_pairs": self.measurement_pairs.to_mapping(relative_to=relative_to),
            "camera_timeline": self.camera_timeline.to_mapping(relative_to=relative_to),
            "pixhawk_timeline": self.pixhawk_timeline.to_mapping(relative_to=relative_to),
            "gnss_base_timeline": self.gnss_base_timeline.to_mapping(relative_to=relative_to),
            "geometry_comparison": self.geometry_comparison.to_mapping(relative_to=relative_to),
        }


@dataclass(frozen=True)
class _Column:
    name: str
    arrow_type: pa.DataType
    converter: Callable[[object, str], object]
    aliases: tuple[str, ...] = ()
    nullable: bool = True
    unit: str = ""

    def field(self) -> pa.Field:
        metadata = {b"unit": self.unit.encode("utf-8")} if self.unit else None
        return pa.field(
            self.name,
            self.arrow_type,
            nullable=self.nullable,
            metadata=metadata,
        )


@dataclass(frozen=True)
class _DatasetSpec:
    key: str
    role: str
    schema_id: str
    columns: tuple[_Column, ...]
    schema_version: int = PARQUET_FORMAT_VERSION

    @property
    def schema(self) -> pa.Schema:
        metadata = {
            b"workstation.schema_id": self.schema_id.encode("utf-8"),
            b"workstation.schema_version": str(self.schema_version).encode("ascii"),
            b"workstation.artifact_role": self.role.encode("utf-8"),
            b"workstation.row_json_contract": ROW_JSON_CONTRACT.encode("utf-8"),
        }
        return pa.schema(
            [column.field() for column in self.columns]
            + [
                pa.field(
                    "row_json",
                    pa.large_string(),
                    nullable=False,
                    metadata={b"contract": ROW_JSON_CONTRACT.encode("utf-8")},
                )
            ],
            metadata=metadata,
        )


def _json_safe(value: object, *, location: str = "row") -> object:
    """Normalize one value without silently stringifying structured data."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{location} contains a non-finite number")
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        normalized: dict[str, object] = {}
        for raw_key, child in value.items():
            if not isinstance(raw_key, str):
                raise TypeError(f"{location} contains a non-string object key")
            normalized[raw_key] = _json_safe(child, location=f"{location}.{raw_key}")
        return normalized
    if isinstance(value, (list, tuple)):
        return [
            _json_safe(child, location=f"{location}[{index}]")
            for index, child in enumerate(value)
        ]
    raise TypeError(f"{location} contains unsupported value type {type(value).__name__}")


def _canonical_row(row: Mapping[str, object], *, row_index: int) -> dict[str, object]:
    normalized = _json_safe(row, location=f"row[{row_index}]")
    if not isinstance(normalized, dict):  # Mapping above makes this defensive only.
        raise TypeError(f"row[{row_index}] is not a JSON object")
    return normalized


def _canonical_json(row: Mapping[str, object]) -> str:
    return json.dumps(
        row,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


def _missing(value: object) -> bool:
    return value is None or (isinstance(value, str) and not value.strip())


def _as_int(value: object, name: str) -> int | None:
    if _missing(value):
        return None
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer, not bool")
    if isinstance(value, int):
        result = value
    elif isinstance(value, float):
        if not math.isfinite(value) or not value.is_integer():
            raise ValueError(f"{name} must be an integer")
        result = int(value)
    elif isinstance(value, str):
        try:
            result = int(value.strip(), 10)
        except ValueError as exc:
            raise ValueError(f"{name} must be an integer") from exc
    else:
        raise ValueError(f"{name} must be an integer")
    if result < -(2**63) or result > 2**63 - 1:
        raise ValueError(f"{name} is outside signed int64 range")
    return result


def _as_float(value: object, name: str) -> float | None:
    if _missing(value):
        return None
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not bool")
    if isinstance(value, (int, float, str)):
        try:
            result = float(value)
        except ValueError as exc:
            raise ValueError(f"{name} must be a number") from exc
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result
    raise ValueError(f"{name} must be a number")


def _as_bool(value: object, name: str) -> bool | None:
    if _missing(value):
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes"}:
            return True
        if normalized in {"false", "0", "no"}:
            return False
    raise ValueError(f"{name} must be a boolean")


def _as_string(value: object, name: str) -> str | None:
    if value is None:
        return None
    if isinstance(value, (str, int, float, bool)):
        return str(value)
    raise ValueError(f"{name} must be a scalar string value")


def _required_string(value: object, name: str) -> str:
    result = _as_string(value, name)
    if result is None or not result.strip():
        raise ValueError(f"{name} is required")
    return result


def _column(
    name: str,
    arrow_type: pa.DataType,
    converter: Callable[[object, str], object],
    *,
    aliases: tuple[str, ...] = (),
    nullable: bool = True,
    unit: str = "",
) -> _Column:
    return _Column(name, arrow_type, converter, aliases, nullable, unit)


I64 = pa.int64()
F64 = pa.float64()
TEXT = pa.string()
BOOL = pa.bool_()


MEASUREMENT_PAIRS = _DatasetSpec(
    key="measurement_pairs",
    role="measurement_pairs_parquet",
    schema_id="workstation-measurement-pairs-parquet-v4",
    schema_version=4,
    columns=(
        _column("sample_index", I64, _as_int),
        _column("recording_time_ns", I64, _as_int, unit="ns"),
        _column("estimated_utc_time_ns", I64, _as_int, unit="ns"),
        _column("device_utc_time_ns", I64, _as_int, unit="ns"),
        _column(
            "pair_id",
            TEXT,
            _as_string,
            aliases=(
                "measurement_attempt_id",
                "measurement_detection_pair_key",
                "pair_sequence",
            ),
        ),
        _column("measurement_valid", BOOL, _as_bool),
        _column("pair_status", TEXT, _as_string),
        _column("class_name", TEXT, _as_string),
        _column("left_bbox_center_x_px", F64, _as_float, aliases=("left_cx",), unit="px"),
        _column("left_bbox_center_y_px", F64, _as_float, aliases=("left_cy",), unit="px"),
        _column("right_bbox_center_x_px", F64, _as_float, aliases=("right_cx",), unit="px"),
        _column("right_bbox_center_y_px", F64, _as_float, aliases=("right_cy",), unit="px"),
        _column("left_confidence", F64, _as_float, aliases=("left_bbox_confidence",)),
        _column("right_confidence", F64, _as_float, aliases=("right_bbox_confidence",)),
        _column("distance_m", F64, _as_float, unit="m"),
        _column("baseline_center_forward_m", F64, _as_float, unit="m"),
        _column("baseline_center_right_m", F64, _as_float, unit="m"),
        _column("baseline_center_down_m", F64, _as_float, unit="m"),
        _column("baseline_m", F64, _as_float, aliases=("baseline_length_m",), unit="m"),
        _column("pairing_time_basis_requested", TEXT, _as_string),
        _column("pairing_time_basis", TEXT, _as_string),
        _column("pairing_time_basis_role", TEXT, _as_string),
        _column("pairing_fallback_active", BOOL, _as_bool),
        _column("pairing_fallback_reason", TEXT, _as_string),
        _column("selected_pair_delta_ns", I64, _as_int, unit="ns"),
        _column("selected_pair_abs_delta_ns", I64, _as_int, unit="ns"),
        _column("selected_pair_midpoint_ns", I64, _as_int, unit="ns"),
        _column("app_receive_pair_delta_ns", I64, _as_int, unit="ns"),
        _column("app_receive_evidence_role", TEXT, _as_string),
        _column("rtcp_pair_delta_ns", I64, _as_int, unit="ns"),
        _column("ntp_evidence_role", TEXT, _as_string),
        _column("left_media_recovery_basis", TEXT, _as_string),
        _column("right_media_recovery_basis", TEXT, _as_string),
        _column("left_media_integrity_mode", TEXT, _as_string),
        _column("right_media_integrity_mode", TEXT, _as_string),
        _column("left_media_random_access_guaranteed", BOOL, _as_bool),
        _column("right_media_random_access_guaranteed", BOOL, _as_bool),
        _column(
            "sync_delta_ns",
            I64,
            _as_int,
            aliases=("selected_pair_delta_ns", "app_receive_pair_delta_ns", "rtcp_pair_delta_ns"),
            unit="ns",
        ),
        _column("quality", TEXT, _as_string, aliases=("pair_quality_status",)),
        _column("warning_code", TEXT, _as_string),
    ),
)


CAMERA_TIMELINE = _DatasetSpec(
    key="camera_timeline",
    role="camera_timeline_parquet",
    schema_id="workstation-camera-timeline-parquet-v3",
    schema_version=3,
    columns=(
        _column("side", TEXT, _required_string, nullable=False),
        _column("sequence", I64, _as_int),
        _column("recording_time_ns", I64, _as_int, unit="ns"),
        _column("app_receive_monotonic_ns", I64, _as_int, unit="ns"),
        _column("app_receive_timestamp_role", TEXT, _as_string),
        _column("estimated_utc_time_ns", I64, _as_int, unit="ns"),
        _column("device_utc_time_ns", I64, _as_int, unit="ns"),
        _column("image_width_px", I64, _as_int, unit="px"),
        _column("image_height_px", I64, _as_int, unit="px"),
        _column("timestamp_source", TEXT, _as_string),
        _column("rtp_timestamp", I64, _as_int),
        _column("rtp_ssrc", I64, _as_int),
        _column("rtp_marker_seq", I64, _as_int),
        _column("rtcp_sr_received_count", I64, _as_int),
        _column("rtcp_sr_rtp_timestamp", I64, _as_int),
        _column("rtcp_clock_rate_hz", F64, _as_float, unit="Hz"),
        _column("rtcp_sr_age_ns", I64, _as_int, unit="ns"),
        _column("rtcp_sr_pc_offset_ns", I64, _as_int, unit="ns"),
        _column("rtcp_receive_delay_ns", I64, _as_int, unit="ns"),
        _column("rtcp_transport_delay_ns", I64, _as_int, unit="ns"),
        _column("rtcp_camera_ntp_rate", F64, _as_float),
        _column("rtcp_camera_media_rate", F64, _as_float),
        _column("rtsp_transport", TEXT, _as_string),
        _column("ntp_evidence_role", TEXT, _as_string),
        _column("media_recovery_basis", TEXT, _as_string),
        _column("media_integrity_mode", TEXT, _as_string),
        _column("media_random_access_guaranteed", BOOL, _as_bool),
        _column("timing_status", TEXT, _as_string),
    ),
)


PX4_TIMELINE = _DatasetSpec(
    key="pixhawk_timeline",
    role="pixhawk_timeline_parquet",
    schema_id="workstation-pixhawk-timeline-parquet-v1",
    columns=(
        _column("source_id", TEXT, _required_string, nullable=False),
        _column("sequence", I64, _as_int),
        _column("recording_time_ns", I64, _as_int, unit="ns"),
        _column("session_utc_time_ns", I64, _as_int, unit="ns"),
        _column("gps_utc_time_ns", I64, _as_int, unit="ns"),
        _column("gps_utc_source", TEXT, _as_string),
        _column("sample_type", TEXT, _as_string),
        _column("topic", TEXT, _as_string),
        _column("attitude_rollspeed_rad_s", F64, _as_float, unit="rad/s"),
        _column("attitude_pitchspeed_rad_s", F64, _as_float, unit="rad/s"),
        _column("attitude_yawspeed_rad_s", F64, _as_float, unit="rad/s"),
        _column("attitude_time_boot_ms", I64, _as_int, unit="ms"),
        _column("roll_deg", F64, _as_float, unit="deg"),
        _column("pitch_deg", F64, _as_float, unit="deg"),
        _column("yaw_deg", F64, _as_float, unit="deg"),
        _column("latitude_deg", F64, _as_float, aliases=("lat_deg",), unit="deg"),
        _column("longitude_deg", F64, _as_float, aliases=("lon_deg",), unit="deg"),
        _column("altitude_m", F64, _as_float, aliases=("alt_m",), unit="m"),
        _column("fix_type", I64, _as_int),
        _column("satellites_used", I64, _as_int, aliases=("satellites",)),
        _column("eph_m", F64, _as_float, unit="m"),
        _column("epv_m", F64, _as_float, unit="m"),
        _column("gps1_latitude_deg", F64, _as_float, unit="deg"),
        _column("gps1_longitude_deg", F64, _as_float, unit="deg"),
        _column("gps1_altitude_m", F64, _as_float, unit="m"),
        _column("gps1_fix_type", I64, _as_int),
        _column("gps1_satellites_used", I64, _as_int),
        _column("gps1_eph_m", F64, _as_float, unit="m"),
        _column("gps1_epv_m", F64, _as_float, unit="m"),
        _column("gps1_heading_deg", F64, _as_float, unit="deg"),
        _column("gps1_ground_speed_m_s", F64, _as_float, unit="m/s"),
        _column("gps1_yaw_deg", F64, _as_float, unit="deg"),
        _column("gps1_heading_accuracy_deg", F64, _as_float, unit="deg"),
        _column("gps2_latitude_deg", F64, _as_float, unit="deg"),
        _column("gps2_longitude_deg", F64, _as_float, unit="deg"),
        _column("gps2_altitude_m", F64, _as_float, unit="m"),
        _column("gps2_fix_type", I64, _as_int),
        _column("gps2_satellites_used", I64, _as_int),
        _column("gps2_eph_m", F64, _as_float, unit="m"),
        _column("gps2_epv_m", F64, _as_float, unit="m"),
        _column("gps2_heading_deg", F64, _as_float, unit="deg"),
        _column("gps2_ground_speed_m_s", F64, _as_float, unit="m/s"),
        _column("gps2_yaw_deg", F64, _as_float, unit="deg"),
        _column("gps2_heading_accuracy_deg", F64, _as_float, unit="deg"),
        _column("heading_deg", F64, _as_float, unit="deg"),
        _column("mag_sensor_slot", TEXT, _as_string),
        _column("mag_slot", I64, _as_int),
        _column("mag_cal_slot", TEXT, _as_string),
        _column("mag_value_units", TEXT, _as_string),
        _column("mag_x", F64, _as_float),
        _column("mag_y", F64, _as_float),
        _column("mag_z", F64, _as_float),
        _column("mag_vector_norm", F64, _as_float),
        _column("mag_temperature_c", F64, _as_float, unit="degC"),
        _column("mag_device_id", I64, _as_int),
        _column("mag_node_id", I64, _as_int),
        _column("mag_priority", I64, _as_int),
        _column("mag_rotation", I64, _as_int),
        _column("mag_enabled", BOOL, _as_bool),
        _column("mag_expected_selected", BOOL, _as_bool),
        _column("mag_actual_selected", BOOL, _as_bool),
        _column("mag_armed", BOOL, _as_bool),
        _column("mag_status", TEXT, _as_string),
        _column("mag_warning_text", TEXT, _as_string),
        _column("mag_param_uavcan_enable", I64, _as_int),
        _column("mag_param_uavcan_sub_mag", I64, _as_int),
        _column("mag_param_ekf2_mag_type", I64, _as_int),
        _column("mag_param_snapshot_json", TEXT, _as_string),
        _column("mag_scaled_imu_x", F64, _as_float),
        _column("mag_scaled_imu_y", F64, _as_float),
        _column("mag_scaled_imu_z", F64, _as_float),
        _column("mag_scaled_imu_unit", TEXT, _as_string),
        _column("mag_scaled_imu2_x", F64, _as_float),
        _column("mag_scaled_imu2_y", F64, _as_float),
        _column("mag_scaled_imu2_z", F64, _as_float),
        _column("mag_scaled_imu2_unit", TEXT, _as_string),
        _column("mag_scaled_imu3_x", F64, _as_float),
        _column("mag_scaled_imu3_y", F64, _as_float),
        _column("mag_scaled_imu3_z", F64, _as_float),
        _column("mag_scaled_imu3_unit", TEXT, _as_string),
        _column("mag_scaled_imu_accel_x", F64, _as_float),
        _column("mag_scaled_imu_accel_y", F64, _as_float),
        _column("mag_scaled_imu_accel_z", F64, _as_float),
        _column("mag_scaled_imu_gyro_x", F64, _as_float),
        _column("mag_scaled_imu_gyro_y", F64, _as_float),
        _column("mag_scaled_imu_gyro_z", F64, _as_float),
        _column("mag_scaled_imu_time_boot_ms", I64, _as_int, unit="ms"),
        _column("mag_scaled_imu2_accel_x", F64, _as_float),
        _column("mag_scaled_imu2_accel_y", F64, _as_float),
        _column("mag_scaled_imu2_accel_z", F64, _as_float),
        _column("mag_scaled_imu2_gyro_x", F64, _as_float),
        _column("mag_scaled_imu2_gyro_y", F64, _as_float),
        _column("mag_scaled_imu2_gyro_z", F64, _as_float),
        _column("mag_scaled_imu2_time_boot_ms", I64, _as_int, unit="ms"),
        _column("mag_scaled_imu3_accel_x", F64, _as_float),
        _column("mag_scaled_imu3_accel_y", F64, _as_float),
        _column("mag_scaled_imu3_accel_z", F64, _as_float),
        _column("mag_scaled_imu3_gyro_x", F64, _as_float),
        _column("mag_scaled_imu3_gyro_y", F64, _as_float),
        _column("mag_scaled_imu3_gyro_z", F64, _as_float),
        _column("mag_scaled_imu3_time_boot_ms", I64, _as_int, unit="ms"),
        _column("mavlink_message_name", TEXT, _as_string),
        _column("mavlink_time_boot_ms", I64, _as_int, unit="ms"),
        _column("mavlink_time_usec", I64, _as_int, unit="us"),
        _column("mavlink_global_latitude_deg", F64, _as_float, unit="deg"),
        _column("mavlink_global_longitude_deg", F64, _as_float, unit="deg"),
        _column("mavlink_global_altitude_m", F64, _as_float, unit="m"),
        _column("mavlink_global_relative_altitude_m", F64, _as_float, unit="m"),
        _column("mavlink_global_velocity_n_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_global_velocity_e_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_global_velocity_d_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_global_heading_deg", F64, _as_float, unit="deg"),
        _column("mavlink_local_x_m", F64, _as_float, unit="m"),
        _column("mavlink_local_y_m", F64, _as_float, unit="m"),
        _column("mavlink_local_z_m", F64, _as_float, unit="m"),
        _column("mavlink_local_velocity_x_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_local_velocity_y_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_local_velocity_z_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_vfr_airspeed_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_vfr_groundspeed_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_vfr_heading_deg", F64, _as_float, unit="deg"),
        _column("mavlink_vfr_throttle_pct", F64, _as_float, unit="percent"),
        _column("mavlink_vfr_altitude_m", F64, _as_float, unit="m"),
        _column("mavlink_vfr_climb_m_s", F64, _as_float, unit="m/s"),
        _column("mavlink_sys_voltage_battery_v", F64, _as_float, unit="V"),
        _column("mavlink_sys_current_battery_a", F64, _as_float, unit="A"),
        _column("mavlink_sys_battery_remaining_pct", F64, _as_float, unit="percent"),
        _column("mavlink_sys_drop_rate_comm_pct", F64, _as_float, unit="percent"),
        _column("mavlink_sys_errors_comm", I64, _as_int),
        _column("mavlink_battery_current_a", F64, _as_float, unit="A"),
        _column("mavlink_battery_remaining_pct", F64, _as_float, unit="percent"),
        _column("mavlink_battery_temperature_c", F64, _as_float, unit="degC"),
        _column("mavlink_battery_voltage_0_v", F64, _as_float, unit="V"),
        _column("mavlink_battery_voltage_1_v", F64, _as_float, unit="V"),
        _column("mavlink_battery_voltage_2_v", F64, _as_float, unit="V"),
        _column("mavlink_raw_accel_x", F64, _as_float),
        _column("mavlink_raw_accel_y", F64, _as_float),
        _column("mavlink_raw_accel_z", F64, _as_float),
        _column("mavlink_raw_gyro_x", F64, _as_float),
        _column("mavlink_raw_gyro_y", F64, _as_float),
        _column("mavlink_raw_gyro_z", F64, _as_float),
        _column("mavlink_raw_mag_x", F64, _as_float),
        _column("mavlink_raw_mag_y", F64, _as_float),
        _column("mavlink_raw_mag_z", F64, _as_float),
    ),
)


GNSS_BASE_TIMELINE = _DatasetSpec(
    key="gnss_base_timeline",
    role="gnss_base_timeline_parquet",
    schema_id="workstation-gnss-base-timeline-parquet-v1",
    columns=(
        _column("sequence", I64, _as_int),
        _column("session_id", TEXT, _as_string),
        _column("side", TEXT, _as_string),
        _column("sample_type", TEXT, _as_string),
        _column("row_contract", TEXT, _as_string),
        _column("sample_monotonic_ns", I64, _as_int, unit="ns"),
        _column("sample_monotonic_s", F64, _as_float, unit="s"),
        _column("session_elapsed_s", F64, _as_float, unit="s"),
        _column("session_utc_ns", I64, _as_int, unit="ns"),
        _column("session_utc_s", F64, _as_float, unit="s"),
        _column("session_utc_method", TEXT, _as_string),
        _column(
            "session_utc_anchor_uncertainty_ns",
            I64,
            _as_int,
            unit="ns",
        ),
        _column("source_utc_ns", I64, _as_int, unit="ns"),
        _column("source_utc_s", F64, _as_float, unit="s"),
        _column("source_utc_valid", BOOL, _as_bool),
        _column("source_utc_method", TEXT, _as_string),
        _column("host_receive_monotonic_ns", I64, _as_int, unit="ns"),
        _column("host_receive_utc_ns", I64, _as_int, unit="ns"),
        _column("latitude_deg", F64, _as_float, unit="deg"),
        _column("longitude_deg", F64, _as_float, unit="deg"),
        _column("ellipsoid_height_m", F64, _as_float, unit="m"),
        _column("msl_altitude_m", F64, _as_float, unit="m"),
        _column("height_type", TEXT, _as_string),
        _column("fixed", BOOL, _as_bool),
        _column("device_id", TEXT, _as_string),
        _column("gnss_operation_state", TEXT, _as_string),
    ),
)


GEOMETRY_COMPARISON = _DatasetSpec(
    key="geometry_comparison",
    role="geometry_comparison_parquet",
    schema_id="workstation-geometry-comparison-parquet-v2",
    columns=(
        _column("pair_sequence", I64, _as_int),
        _column("pair_id", TEXT, _as_string),
        _column("timestamp_s", F64, _as_float, unit="s"),
        _column("schema_version", TEXT, _as_string),
        _column("evidence_namespace", TEXT, _as_string),
        _column("result_class", TEXT, _as_string),
        _column("not_measurement_result", BOOL, _as_bool),
        _column("source_state", TEXT, _as_string),
        _column("source_profile_id", TEXT, _as_string),
        _column("source_profile_hash", TEXT, _as_string),
        _column("candidate_profile_id", TEXT, _as_string),
        _column("candidate_profile_hash", TEXT, _as_string),
        _column("candidate_intrinsics_hash", TEXT, _as_string),
        _column("active_intrinsics_hash", TEXT, _as_string),
        _column("candidate_applied", BOOL, _as_bool),
        _column("candidate_applicability", TEXT, _as_string),
        _column("candidate_quality", TEXT, _as_string),
        _column("method", TEXT, _as_string),
        _column("forward_m", F64, _as_float, unit="m"),
        _column("right_m", F64, _as_float, unit="m"),
        _column("down_m", F64, _as_float, unit="m"),
        _column("range_m", F64, _as_float, unit="m"),
        _column("status", TEXT, _as_string),
    ),
)


_SPECS = {
    spec.key: spec
    for spec in (
        MEASUREMENT_PAIRS,
        CAMERA_TIMELINE,
        PX4_TIMELINE,
        GNSS_BASE_TIMELINE,
        GEOMETRY_COMPARISON,
    )
}


def _source_value(row: Mapping[str, object], column: _Column) -> object:
    for name in (column.name, *column.aliases):
        value = row.get(name)
        if not _missing(value):
            return value
    return None


def _project_row(row: Mapping[str, object], spec: _DatasetSpec) -> dict[str, object]:
    projected: dict[str, object] = {}
    for column in spec.columns:
        source = _source_value(row, column)
        projected[column.name] = column.converter(source, column.name)
    projected["row_json"] = _canonical_json(row)
    return projected


def _iter_canonical_rows(
    rows: Iterable[Mapping[str, object]],
) -> Iterable[dict[str, object]]:
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping):
            raise TypeError(f"row[{index}] is not a mapping")
        yield _canonical_row(row, row_index=index)


def _partitioned_rows(
    rows_by_source: Mapping[str, Iterable[Mapping[str, object]]],
    *,
    discriminator: str,
) -> Iterable[dict[str, object]]:
    row_index = 0
    for raw_source, rows in rows_by_source.items():
        source = _required_string(raw_source, discriminator)
        for row in rows:
            if not isinstance(row, Mapping):
                raise TypeError(f"row[{row_index}] is not a mapping")
            normalized = _canonical_row(row, row_index=row_index)
            existing = normalized.get(discriminator)
            if existing is not None and str(existing) != source:
                raise ValueError(
                    f"{discriminator} conflicts with partition key: "
                    f"expected={source!r}, actual={existing!r}"
                )
            normalized[discriminator] = source
            yield normalized
            row_index += 1


def _schema_mapping(spec: _DatasetSpec) -> dict[str, object]:
    schema = spec.schema
    schema_metadata = {
        key.decode("utf-8"): value.decode("utf-8")
        for key, value in sorted((schema.metadata or {}).items())
    }
    fields: list[dict[str, object]] = []
    for field in schema:
        field_entry: dict[str, object] = {
            "name": field.name,
            "type": str(field.type),
            "nullable": field.nullable,
        }
        if field.metadata:
            field_entry["metadata"] = {
                key.decode("utf-8"): value.decode("utf-8")
                for key, value in sorted(field.metadata.items())
            }
        fields.append(field_entry)
    return {
        "id": spec.schema_id,
        "version": spec.schema_version,
        "fields": fields,
        "metadata": schema_metadata,
    }


def _schema_sha256(schema: Mapping[str, object]) -> str:
    payload = json.dumps(
        schema,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fsync_file(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _verify_table(
    path: Path,
    spec: _DatasetSpec,
    *,
    expected_row_count: int | None = None,
    expected_row_json_sha256: str | None = None,
) -> ParquetArtifactMetadata:
    try:
        parquet_file = pq.ParquetFile(path)
        if parquet_file.metadata.num_rows < 0:
            raise ParquetFinalizationError(f"invalid Parquet row count: {path}")
    except (OSError, pa.ArrowException) as exc:
        raise ParquetFinalizationError(f"cannot read Parquet artifact: {path}: {exc}") from exc

    expected_schema = spec.schema
    if not parquet_file.schema_arrow.equals(expected_schema, check_metadata=True):
        raise ParquetFinalizationError(
            f"Parquet schema mismatch for {spec.key}: path={path}"
        )
    row_count = 0
    row_json_digest = hashlib.sha256()
    for batch in parquet_file.iter_batches(batch_size=PARQUET_BATCH_ROWS):
        records = batch.to_pylist()
        for record in records:
            index = row_count
            row_json = record.get("row_json")
            row_count += 1
            if not isinstance(row_json, str):
                raise ParquetFinalizationError(
                    f"Parquet row_json is null or non-string: path={path}, row={index}"
                )
            row_json_digest.update(row_json.encode("utf-8") + b"\n")
            try:
                decoded = json.loads(row_json)
            except json.JSONDecodeError as exc:
                raise ParquetFinalizationError(
                    f"Parquet row_json is invalid JSON: path={path}, row={index}"
                ) from exc
            if not isinstance(decoded, dict):
                raise ParquetFinalizationError(
                    f"Parquet row_json is not an object: path={path}, row={index}"
                )
            if _canonical_json(decoded) != row_json:
                raise ParquetFinalizationError(
                    f"Parquet row_json is not canonical: path={path}, row={index}"
                )
            try:
                expected_projection = _project_row(decoded, spec)
            except (TypeError, ValueError) as exc:
                raise ParquetFinalizationError(
                    f"Parquet row_json cannot reproduce typed columns: path={path}, row={index}: {exc}"
                ) from exc
            for column in spec.columns:
                if record[column.name] != expected_projection[column.name]:
                    raise ParquetFinalizationError(
                        f"Parquet typed column disagrees with row_json: "
                        f"path={path}, row={index}, column={column.name}"
                    )

    if row_count != parquet_file.metadata.num_rows:
        raise ParquetFinalizationError(
            f"Parquet metadata/readback row mismatch: path={path}, "
            f"metadata={parquet_file.metadata.num_rows}, read={row_count}"
        )
    if expected_row_count is not None and row_count != expected_row_count:
        raise ParquetFinalizationError(
            f"Parquet source/readback row mismatch: path={path}, "
            f"source={expected_row_count}, read={row_count}"
        )
    if (
        expected_row_json_sha256 is not None
        and row_json_digest.hexdigest() != expected_row_json_sha256
    ):
        raise ParquetFinalizationError(f"Parquet row_json readback mismatch: path={path}")

    schema = _schema_mapping(spec)
    return ParquetArtifactMetadata(
        path=path,
        role=spec.role,
        row_count=row_count,
        size_bytes=path.stat().st_size,
        sha256=_file_sha256(path),
        schema=schema,
        schema_sha256=_schema_sha256(schema),
    )


def _atomic_write(
    output: Path,
    rows: Iterable[Mapping[str, object]],
    spec: _DatasetSpec,
) -> ParquetArtifactMetadata:
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)

    descriptor, temporary_name = tempfile.mkstemp(
        dir=output.parent,
        prefix=f".{output.name}.",
        suffix=".tmp",
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        writer = pq.ParquetWriter(
            temporary,
            spec.schema,
            compression="zstd",
            use_dictionary=True,
            write_statistics=True,
        )
        row_count = 0
        row_json_digest = hashlib.sha256()
        batch: list[dict[str, object]] = []
        try:
            for row in _iter_canonical_rows(rows):
                record = _project_row(row, spec)
                batch.append(record)
                row_count += 1
                row_json_digest.update(
                    str(record["row_json"]).encode("utf-8") + b"\n"
                )
                if len(batch) >= PARQUET_BATCH_ROWS:
                    writer.write_table(pa.Table.from_pylist(batch, schema=spec.schema))
                    batch.clear()
            if batch:
                writer.write_table(pa.Table.from_pylist(batch, schema=spec.schema))
            elif row_count == 0:
                writer.write_table(pa.Table.from_pylist([], schema=spec.schema))
        finally:
            writer.close()
        _fsync_file(temporary)
        verified = _verify_table(
            temporary,
            spec,
            expected_row_count=row_count,
            expected_row_json_sha256=row_json_digest.hexdigest(),
        )
        os.replace(temporary, output)
        _fsync_directory(output.parent)
        return replace(verified, path=output)
    finally:
        temporary.unlink(missing_ok=True)


def finalize_measurement_pairs(
    output: Path,
    rows: Iterable[Mapping[str, object]],
) -> ParquetArtifactMetadata:
    """Write and verify the derived measurement-pair Parquet file."""

    return _atomic_write(output, rows, MEASUREMENT_PAIRS)


def finalize_camera_timeline(
    output: Path,
    rows_by_side: Mapping[str, Iterable[Mapping[str, object]]],
) -> ParquetArtifactMetadata:
    """Combine camera partitions, adding a required ``side`` discriminator."""

    rows = _partitioned_rows(rows_by_side, discriminator="side")
    return _atomic_write(output, rows, CAMERA_TIMELINE)


def finalize_px4_timeline(
    output: Path,
    rows_by_source: Mapping[str, Iterable[Mapping[str, object]]],
) -> ParquetArtifactMetadata:
    """Combine Pixhawk partitions, adding a required ``source_id`` field."""

    rows = _partitioned_rows(rows_by_source, discriminator="source_id")
    return _atomic_write(output, rows, PX4_TIMELINE)


def finalize_gnss_base_timeline(
    output: Path,
    rows: Iterable[Mapping[str, object]],
) -> ParquetArtifactMetadata:
    """Write and verify the Base-GNSS timeline Parquet file."""

    return _atomic_write(output, rows, GNSS_BASE_TIMELINE)


def finalize_geometry_comparison(
    output: Path,
    rows: Iterable[Mapping[str, object]],
) -> ParquetArtifactMetadata:
    """Write and verify method-by-method geometry comparison rows."""

    return _atomic_write(output, rows, GEOMETRY_COMPARISON)


def finalize_session_parquet_artifacts(
    output_dir: Path,
    *,
    measurement_pairs: Iterable[Mapping[str, object]],
    camera_rows: Mapping[str, Iterable[Mapping[str, object]]],
    px4_rows: Mapping[str, Iterable[Mapping[str, object]]],
    gnss_base_rows: Iterable[Mapping[str, object]] = (),
    geometry_comparison_rows: Iterable[Mapping[str, object]] = (),
) -> SessionParquetArtifacts:
    """Finalize the standard five Parquet artifacts below ``output_dir``.

    Each file is independently atomic.  Callers that require an all-five-files
    transaction should invoke this function inside their existing generation
    staging directory and atomically publish that generation afterwards.
    """

    output_dir = Path(output_dir)
    return SessionParquetArtifacts(
        measurement_pairs=finalize_measurement_pairs(
            output_dir / MEASUREMENT_PAIRS_FILENAME,
            measurement_pairs,
        ),
        camera_timeline=finalize_camera_timeline(
            output_dir / CAMERA_TIMELINE_FILENAME,
            camera_rows,
        ),
        pixhawk_timeline=finalize_px4_timeline(
            output_dir / PX4_TIMELINE_FILENAME,
            px4_rows,
        ),
        gnss_base_timeline=finalize_gnss_base_timeline(
            output_dir / GNSS_BASE_TIMELINE_FILENAME,
            gnss_base_rows,
        ),
        geometry_comparison=finalize_geometry_comparison(
            output_dir / GEOMETRY_COMPARISON_FILENAME,
            geometry_comparison_rows,
        ),
    )


def verify_parquet_artifact(
    path: Path,
    artifact_key: str,
) -> ParquetArtifactMetadata:
    """Re-verify a previously finalized artifact by its stable artifact key."""

    try:
        spec = _SPECS[artifact_key]
    except KeyError as exc:
        raise ValueError(f"unknown Parquet artifact key: {artifact_key}") from exc
    return _verify_table(Path(path), spec)


def read_lossless_rows(path: Path, artifact_key: str) -> list[dict[str, object]]:
    """Verify an artifact and return the complete source objects from row_json."""

    return list(iter_lossless_rows(path, artifact_key))


def iter_lossless_rows(
    path: Path,
    artifact_key: str,
    *,
    verify: bool = True,
) -> Iterable[dict[str, object]]:
    """Verify, then stream complete source objects in bounded Arrow batches."""

    if verify:
        verify_parquet_artifact(path, artifact_key)
    parquet_file = pq.ParquetFile(path)
    for batch in parquet_file.iter_batches(
        batch_size=PARQUET_BATCH_ROWS,
        columns=["row_json"],
    ):
        for value in batch.column(0).to_pylist():
            yield json.loads(value)


__all__ = [
    "CAMERA_TIMELINE_FILENAME",
    "GEOMETRY_COMPARISON_FILENAME",
    "GNSS_BASE_TIMELINE_FILENAME",
    "MEASUREMENT_PAIRS_FILENAME",
    "PARQUET_FORMAT_VERSION",
    "PX4_TIMELINE_FILENAME",
    "ParquetArtifactMetadata",
    "ParquetFinalizationError",
    "ROW_JSON_CONTRACT",
    "SessionParquetArtifacts",
    "finalize_camera_timeline",
    "finalize_geometry_comparison",
    "finalize_gnss_base_timeline",
    "finalize_measurement_pairs",
    "finalize_px4_timeline",
    "finalize_session_parquet_artifacts",
    "iter_lossless_rows",
    "read_lossless_rows",
    "verify_parquet_artifact",
]
