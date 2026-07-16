"""Independent Phase A session recording core.

This module deliberately has no dependency on the GUI, ROS 2, MAVLink, or
the live camera workers.  It provides typed records, explicit PyArrow
schemas, and a small lifecycle wrapper that can be exercised with synthetic
inputs before Phase B integration.
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
import re
import sqlite3
import threading
import time
from functools import lru_cache
from numbers import Integral
from operator import index as integer_index
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import pyarrow as pa
import pyarrow.parquet as pq


SCHEMA_VERSION = "stereo-distance-session-phase-a-v1"
_SESSION_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1
_CAMERA_SIDES = ("left", "right")
_VIDEO_MAPPING_STATUSES = {
    "pending",
    "stored",
    "dropped",
    "writer_failure",
}
_ALIGNMENT_REFERENCE_DOMAINS = {
    "pc_receive_monotonic",
    "camera_rtcp_utc",
    "px4_utc",
    "unavailable",
}
_ALIGNMENT_STATUSES = {"receive_time_fallback", "utc_aligned", "missing", "rejected"}
_PAIR_TIMESTAMP_BASES = {
    "pc_receive_monotonic",
    "pc_receive_utc",
    "camera_rtp",
    "camera_pts",
    "px4_timestamp",
    "px4_utc",
}
_SOURCE_MAPPING_FINAL_STATUSES = {"stored", "dropped", "writer_failure"}
_CANONICAL_ARTIFACT_COUNT = 7


class SessionRecordingError(RuntimeError):
    """Base error for the independent session recorder."""


class SchemaValidationError(SessionRecordingError, ValueError):
    """Raised when a record violates the declared schema contract."""


class SessionStateError(SessionRecordingError):
    """Raised when an operation is attempted in the wrong lifecycle state."""


def _metadata(**values: str) -> dict[bytes, bytes]:
    return {key.encode("utf-8"): value.encode("utf-8") for key, value in values.items()}


def _field(
    name: str,
    data_type: pa.DataType,
    *,
    nullable: bool,
    unit: str = "",
    domain: str = "",
    frame: str = "",
    source: str = "",
) -> pa.Field:
    return pa.field(
        name,
        data_type,
        nullable=nullable,
        metadata=_metadata(unit=unit, domain=domain, frame=frame, source=source),
    )


@lru_cache(maxsize=1)
def source_records_schema() -> pa.Schema:
    """Return the immutable schema for ``time/source_records.parquet``."""

    return pa.schema(
        [
            _field("source_record_id", pa.string(), nullable=False, source="Phase A recorder"),
            _field("session_id", pa.string(), nullable=False, source="Phase A recorder"),
            _field("camera_side", pa.string(), nullable=False, source="camera adapter"),
            _field("camera_stream_id", pa.string(), nullable=False, source="camera adapter"),
            _field("frame_id", pa.int64(), nullable=False, source="existing camera sequence"),
            _field("camera_sequence", pa.int64(), nullable=False, source="camera adapter"),
            _field(
                "pc_receive_monotonic_ns",
                pa.int64(),
                nullable=False,
                unit="ns",
                domain="pc_monotonic",
                source="camera receive callback",
            ),
            _field(
                "pc_receive_utc_ns",
                pa.int64(),
                nullable=True,
                unit="ns",
                domain="pc_utc",
                source="camera receive callback",
            ),
            _field(
                "rtp_timestamp",
                pa.int64(),
                nullable=True,
                unit="clock ticks",
                domain="camera_rtp",
                source="RTP timing metadata",
            ),
            _field(
                "rtp_ssrc",
                pa.int64(),
                nullable=True,
                domain="camera_rtp",
                source="RTP timing metadata",
            ),
            _field(
                "rtp_marker_seq",
                pa.int64(),
                nullable=True,
                domain="camera_rtp",
                source="RTP timing metadata",
            ),
            _field(
                "rtcp_sr_ntp_raw",
                pa.int64(),
                nullable=True,
                domain="camera_rtcp_ntp",
                source="RTCP sender report",
            ),
            _field(
                "rtcp_frame_ntp_unix_ns",
                pa.int64(),
                nullable=True,
                unit="ns",
                domain="pc_utc",
                source="RTCP sender report conversion",
            ),
            _field(
                "pts_ticks",
                pa.int64(),
                nullable=True,
                unit="time-base ticks",
                domain="camera_media",
                source="decoded/encoded media buffer",
            ),
            _field(
                "dts_ticks",
                pa.int64(),
                nullable=True,
                unit="time-base ticks",
                domain="camera_media",
                source="encoded media buffer",
            ),
            _field(
                "time_base_num",
                pa.int64(),
                nullable=True,
                domain="camera_media",
                source="media backend",
            ),
            _field(
                "time_base_den",
                pa.int64(),
                nullable=True,
                domain="camera_media",
                source="media backend",
            ),
            _field("width_px", pa.int64(), nullable=False, unit="px", source="decoded frame"),
            _field("height_px", pa.int64(), nullable=False, unit="px", source="decoded frame"),
            _field("video_path", pa.string(), nullable=False, source="Phase A layout"),
            _field("video_frame_index", pa.int64(), nullable=True, source="MKV mapping"),
            _field(
                "video_packet_pts_ticks",
                pa.int64(),
                nullable=True,
                unit="time-base ticks",
                domain="mkv_media",
                source="MKV mapping",
            ),
            _field(
                "video_packet_dts_ticks",
                pa.int64(),
                nullable=True,
                unit="time-base ticks",
                domain="mkv_media",
                source="MKV mapping",
            ),
            _field("stored_in_video", pa.bool_(), nullable=False, source="MKV mapping"),
            _field("video_mapping_status", pa.string(), nullable=False, source="MKV mapping"),
            _field("drop_reason", pa.string(), nullable=True, source="MKV mapping"),
        ],
    )


@lru_cache(maxsize=1)
def time_alignment_schema() -> pa.Schema:
    """Return the schema for domain-separated runtime time association."""

    return pa.schema(
        [
            _field("alignment_id", pa.string(), nullable=False, source="Phase A recorder"),
            _field("record_type", pa.string(), nullable=False, source="Phase A recorder"),
            _field("record_id", pa.string(), nullable=False, source="source/sample/result ID"),
            _field("camera_side", pa.string(), nullable=True, source="camera record"),
            _field("pixhawk_id", pa.string(), nullable=True, source="Pixhawk record"),
            _field("alignment_reference_domain", pa.string(), nullable=False, source="alignment adapter"),
            _field("alignment_status", pa.string(), nullable=False, source="alignment adapter"),
            _field(
                "aligned_pc_monotonic_ns",
                pa.int64(),
                nullable=False,
                unit="ns",
                domain="pc_monotonic",
                source="receive-time association",
            ),
            _field(
                "pc_receive_monotonic_ns",
                pa.int64(),
                nullable=False,
                unit="ns",
                domain="pc_monotonic",
                source="receive callback",
            ),
            _field(
                "pc_receive_utc_ns",
                pa.int64(),
                nullable=True,
                unit="ns",
                domain="pc_utc",
                source="receive callback",
            ),
            _field(
                "camera_rtp_timestamp",
                pa.int64(),
                nullable=True,
                unit="clock ticks",
                domain="camera_rtp",
                source="RTP timing metadata",
            ),
            _field(
                "camera_pts_ticks",
                pa.int64(),
                nullable=True,
                unit="time-base ticks",
                domain="camera_media",
                source="media buffer",
            ),
            _field(
                "camera_dts_ticks",
                pa.int64(),
                nullable=True,
                unit="time-base ticks",
                domain="camera_media",
                source="encoded media buffer",
            ),
            _field(
                "camera_rtcp_ntp_unix_ns",
                pa.int64(),
                nullable=True,
                unit="ns",
                domain="pc_utc",
                source="RTCP sender report conversion",
            ),
            _field(
                "px4_timestamp_us",
                pa.int64(),
                nullable=True,
                unit="us",
                domain="px4_boot",
                source="ROS 2 timestamp field",
            ),
            _field(
                "px4_timestamp_sample_us",
                pa.int64(),
                nullable=True,
                unit="us",
                domain="px4_boot",
                source="ROS 2 timestamp_sample field",
            ),
            _field(
                "px4_time_utc_usec",
                pa.int64(),
                nullable=True,
                unit="us",
                domain="px4_utc",
                source="ROS 2 time_utc_usec field",
            ),
            _field(
                "mavlink_time_usec",
                pa.int64(),
                nullable=True,
                unit="us",
                domain="mavlink_source",
                source="MAVLink time_usec field",
            ),
            _field(
                "mavlink_time_boot_ms",
                pa.int64(),
                nullable=True,
                unit="ms",
                domain="mavlink_boot",
                source="MAVLink time_boot_ms field",
            ),
            _field(
                "camera_rtcp_to_pc_utc_delta_ns",
                pa.int64(),
                nullable=True,
                unit="ns",
                domain="pc_utc",
                source="UTC-to-UTC association",
            ),
            _field(
                "px4_utc_to_pc_utc_delta_ns",
                pa.int64(),
                nullable=True,
                unit="ns",
                domain="pc_utc",
                source="UTC-to-UTC association",
            ),
        ],
    )


@lru_cache(maxsize=1)
def pixhawk_samples_schema() -> pa.Schema:
    """Return the schema for raw/normalized synthetic Pixhawk samples."""

    return pa.schema(
        [
            _field("sample_id", pa.string(), nullable=False, source="Phase A recorder"),
            _field("session_id", pa.string(), nullable=False, source="Phase A recorder"),
            _field("pixhawk_id", pa.string(), nullable=False, source="configured namespace"),
            _field("camera_side", pa.string(), nullable=True, source="configured side alias"),
            _field("transport", pa.string(), nullable=False, source="sample adapter"),
            _field("namespace", pa.string(), nullable=True, source="ROS 2 namespace"),
            _field("topic", pa.string(), nullable=False, source="actual topic"),
            _field("message_type", pa.string(), nullable=False, source="actual message type"),
            _field("source_slot", pa.string(), nullable=False, source="topic/source slot"),
            _field("sample_sequence", pa.int64(), nullable=False, source="sample adapter"),
            _field(
                "pc_receive_monotonic_ns",
                pa.int64(),
                nullable=False,
                unit="ns",
                domain="pc_monotonic",
                source="callback receive time",
            ),
            _field(
                "pc_receive_utc_ns",
                pa.int64(),
                nullable=True,
                unit="ns",
                domain="pc_utc",
                source="callback receive time",
            ),
            _field("px4_timestamp_us", pa.int64(), nullable=True, unit="us", domain="px4_boot", source="timestamp"),
            _field(
                "px4_timestamp_sample_us",
                pa.int64(),
                nullable=True,
                unit="us",
                domain="px4_boot",
                source="timestamp_sample",
            ),
            _field(
                "px4_time_utc_usec",
                pa.int64(),
                nullable=True,
                unit="us",
                domain="px4_utc",
                source="time_utc_usec",
            ),
            _field("mavlink_time_usec", pa.int64(), nullable=True, unit="us", domain="mavlink_source", source="time_usec"),
            _field(
                "mavlink_time_boot_ms",
                pa.int64(),
                nullable=True,
                unit="ms",
                domain="mavlink_boot",
                source="time_boot_ms",
            ),
            _field("device_id", pa.int64(), nullable=True, source="message.device_id"),
            _field("attitude_q_w", pa.float64(), nullable=True, source="VehicleAttitude.q[0]", frame="PX4 body FRD"),
            _field("attitude_q_x", pa.float64(), nullable=True, source="VehicleAttitude.q[1]", frame="PX4 body FRD"),
            _field("attitude_q_y", pa.float64(), nullable=True, source="VehicleAttitude.q[2]", frame="PX4 body FRD"),
            _field("attitude_q_z", pa.float64(), nullable=True, source="VehicleAttitude.q[3]", frame="PX4 body FRD"),
            _field("quat_reset_counter", pa.int64(), nullable=True, source="VehicleAttitude.quat_reset_counter"),
            _field("delta_q_reset_json", pa.string(), nullable=True, source="VehicleAttitude.delta_q_reset"),
            _field("latitude_deg", pa.float64(), nullable=True, unit="deg", source="SensorGps/VehicleGlobalPosition"),
            _field("longitude_deg", pa.float64(), nullable=True, unit="deg", source="SensorGps/VehicleGlobalPosition"),
            _field("altitude_msl_m", pa.float64(), nullable=True, unit="m", source="SensorGps/VehicleGlobalPosition"),
            _field("altitude_ellipsoid_m", pa.float64(), nullable=True, unit="m", source="SensorGps/VehicleGlobalPosition"),
            _field("fix_type", pa.int64(), nullable=True, source="SensorGps.fix_type"),
            _field("satellites_used", pa.int64(), nullable=True, source="SensorGps.satellites_used"),
            _field("jamming_state", pa.int64(), nullable=True, source="SensorGps.jamming_state"),
            _field("spoofing_state", pa.int64(), nullable=True, source="SensorGps.spoofing_state"),
            _field("eph_m", pa.float64(), nullable=True, unit="m", source="SensorGps/VehicleGlobalPosition.eph"),
            _field("epv_m", pa.float64(), nullable=True, unit="m", source="SensorGps/VehicleGlobalPosition.epv"),
            _field("velocity_m_s", pa.float64(), nullable=True, unit="m/s", source="SensorGps velocity"),
            _field("velocity_north_m_s", pa.float64(), nullable=True, unit="m/s", frame="PX4 local NED", source="SensorGps.vel_ned"),
            _field("velocity_east_m_s", pa.float64(), nullable=True, unit="m/s", frame="PX4 local NED", source="SensorGps.vel_ned"),
            _field("velocity_down_m_s", pa.float64(), nullable=True, unit="m/s", frame="PX4 local NED", source="SensorGps.vel_ned"),
            _field("velocity_ned_valid", pa.bool_(), nullable=True, frame="PX4 local NED", source="SensorGps.vel_ned_valid"),
            _field("cog_rad", pa.float64(), nullable=True, unit="rad", source="SensorGps.cog_rad"),
            _field("heading_rad", pa.float64(), nullable=True, unit="rad", source="SensorGps/SensorGnssRelative.heading"),
            _field("heading_accuracy_rad", pa.float64(), nullable=True, unit="rad", source="SensorGps/SensorGnssRelative.heading_accuracy"),
            _field("rtcm_injection_rate_hz", pa.float64(), nullable=True, unit="Hz", source="SensorGps.rtcm_injection_rate"),
            _field("gnss_relative_position_json", pa.string(), nullable=True, frame="message-defined", source="SensorGnssRelative.position"),
            _field("gnss_relative_accuracy_json", pa.string(), nullable=True, frame="message-defined", source="SensorGnssRelative.position_accuracy"),
            _field("position_length_m", pa.float64(), nullable=True, unit="m", source="SensorGnssRelative.position_length"),
            _field("accuracy_length_m", pa.float64(), nullable=True, unit="m", source="SensorGnssRelative.accuracy_length"),
            _field("relative_position_valid", pa.bool_(), nullable=True, source="SensorGnssRelative.relative_position_valid"),
            _field("gnss_fix_ok", pa.bool_(), nullable=True, source="SensorGnssRelative.gnss_fix_ok"),
            _field("differential_solution", pa.bool_(), nullable=True, source="SensorGnssRelative.differential_solution"),
            _field("carrier_solution_floating", pa.bool_(), nullable=True, source="SensorGnssRelative.carrier_solution_floating"),
            _field("carrier_solution_fixed", pa.bool_(), nullable=True, source="SensorGnssRelative.carrier_solution_fixed"),
            _field("moving_base_mode", pa.bool_(), nullable=True, source="SensorGnssRelative.moving_base_mode"),
            _field("heading_valid", pa.bool_(), nullable=True, source="SensorGnssRelative.heading_valid"),
            _field("angular_velocity_xyz_json", pa.string(), nullable=True, frame="PX4 body FRD", source="VehicleAngularVelocity.xyz"),
            _field("angular_velocity_derivative_xyz_json", pa.string(), nullable=True, frame="PX4 body FRD", source="VehicleAngularVelocity.xyz_derivative"),
            _field("local_position_xyz_json", pa.string(), nullable=True, frame="PX4 local NED", source="VehicleLocalPosition.x/y/z"),
            _field("local_velocity_xyz_json", pa.string(), nullable=True, frame="PX4 local NED", source="VehicleLocalPosition.vx/vy/vz"),
            _field("raw_fields_json", pa.string(), nullable=False, source="raw actual message fields"),
        ],
    )


@lru_cache(maxsize=1)
def triangulation_schema() -> pa.Schema:
    """Return the schema for base synthetic triangulation records."""

    return pa.schema(
        [
            _field("triangulation_record_id", pa.string(), nullable=False, source="Phase A recorder"),
            _field("session_id", pa.string(), nullable=False, source="Phase A recorder"),
            _field("left_source_record_id", pa.string(), nullable=True, source="camera source record"),
            _field("right_source_record_id", pa.string(), nullable=True, source="camera source record"),
            _field("left_frame_id", pa.int64(), nullable=True, source="camera frame identity"),
            _field("right_frame_id", pa.int64(), nullable=True, source="camera frame identity"),
            _field("pair_timestamp_basis", pa.string(), nullable=False, source="pairing adapter"),
            _field("pair_timestamp_delta_ns", pa.int64(), nullable=True, unit="ns", domain="pairing", source="pairing adapter"),
            _field("left_pose_sample_id", pa.string(), nullable=True, source="Pixhawk sample link"),
            _field("right_pose_sample_id", pa.string(), nullable=True, source="Pixhawk sample link"),
            _field("intrinsics_path", pa.string(), nullable=True, source="calibration snapshot"),
            _field("intrinsics_zoom_level", pa.int64(), nullable=True, source="calibration snapshot"),
            _field("calibration_source_id", pa.string(), nullable=True, source="calibration snapshot"),
            _field("stereo_extrinsics_source", pa.string(), nullable=True, source="existing geometry source"),
            _field("stereo_extrinsics_frame", pa.string(), nullable=True, frame="left camera optical", source="existing geometry source"),
            _field("stereo_extrinsics_axis_contract", pa.string(), nullable=True, frame="camera optical RDF", source="existing geometry source"),
            _field("left_bbox_center_u_px", pa.float64(), nullable=True, unit="px", source="synthetic observation"),
            _field("left_bbox_center_v_px", pa.float64(), nullable=True, unit="px", source="synthetic observation"),
            _field("right_bbox_center_u_px", pa.float64(), nullable=True, unit="px", source="synthetic observation"),
            _field("right_bbox_center_v_px", pa.float64(), nullable=True, unit="px", source="synthetic observation"),
            _field("left_confidence", pa.float64(), nullable=True, source="synthetic observation"),
            _field("right_confidence", pa.float64(), nullable=True, source="synthetic observation"),
            _field("class_id", pa.int64(), nullable=True, source="synthetic observation"),
            _field("class_name", pa.string(), nullable=True, source="synthetic observation"),
            _field("stereo_center_forward_m", pa.float64(), nullable=True, unit="m", frame="stereo_center_frd", source="RDF z -> FRD forward"),
            _field("stereo_center_right_m", pa.float64(), nullable=True, unit="m", frame="stereo_center_frd", source="RDF x -> FRD right"),
            _field("stereo_center_down_m", pa.float64(), nullable=True, unit="m", frame="stereo_center_frd", source="RDF y -> FRD down"),
            _field("stereo_range_m", pa.float64(), nullable=True, unit="m", frame="stereo_center_frd", source="RayTriangulationResult.distance_m"),
            _field("ray_gap_m", pa.float64(), nullable=True, unit="m", source="RayTriangulationResult.closest_gap_m"),
            _field("intersection_angle_rad", pa.float64(), nullable=True, unit="rad", frame="left camera optical", source="angle between ray vectors"),
            _field("baseline_m", pa.float64(), nullable=True, unit="m", frame="left camera optical", source="base triangulation"),
            _field("triangulation_valid", pa.bool_(), nullable=False, source="base validation"),
            _field("triangulation_status", pa.string(), nullable=False, source="base validation/adapter"),
            _field("triangulation_source", pa.string(), nullable=True, source="existing geometry source"),
            _field("triangulation_method", pa.string(), nullable=True, source="RayTriangulationResult.method"),
            _field("warnings_json", pa.string(), nullable=True, source="base geometry warnings"),
        ],
    )



# Corrective Phase A implementation
#
# The first draft of this module intentionally kept the API small, but it
# buffered every source row and treated the final JSON write as part of an
# already-completed close.  The definitions below are the authoritative
# Phase A implementation.  They retain the public record names and schemas
# while adding strict validation, a disk-backed integrity index, bounded
# source batching, and retryable metadata finalization.
# ---------------------------------------------------------------------------

import uuid
from numbers import Real


def _validate_session_id(session_id: str) -> None:
    if not isinstance(session_id, str) or not _SESSION_ID_RE.fullmatch(session_id):
        raise SchemaValidationError("session_id must be a safe non-empty path component")


def _required_string(value: object, field_name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SchemaValidationError(f"{field_name} must be a non-empty string")
    return value


def validate_int64(field_name: str, value: object, *, nullable: bool = True) -> int | None:
    """Validate an Arrow ``int64`` value without implicit Python casts."""

    if value is None:
        if nullable:
            return None
        raise SchemaValidationError(f"{field_name} is required")
    if isinstance(value, bool):
        raise SchemaValidationError(f"{field_name} must be an integer, not bool")
    try:
        converted = integer_index(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise SchemaValidationError(f"{field_name} must be an int64 integer") from exc
    if converted < _INT64_MIN or converted > _INT64_MAX:
        raise SchemaValidationError(f"{field_name} is outside int64 range")
    return int(converted)


def _int64(value: object, field_name: str, *, nullable: bool = True) -> int | None:
    """Backward-compatible argument order for the strict int64 validator."""

    return validate_int64(field_name, value, nullable=nullable)


def _strict_float(field_name: str, value: object, *, nullable: bool = True) -> float | None:
    if value is None:
        if nullable:
            return None
        raise SchemaValidationError(f"{field_name} is required")
    if isinstance(value, bool) or not isinstance(value, Real):
        # NumPy scalar values expose ``item`` but are not consistently
        # registered as ``Real`` across NumPy versions.
        item = getattr(value, "item", None)
        if callable(item):
            value = item()
    if isinstance(value, bool) or not isinstance(value, Real):
        raise SchemaValidationError(f"{field_name} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise SchemaValidationError(f"{field_name} must be finite")
    return converted


def _finite(value: object, field_name: str, *, nullable: bool = True) -> float | None:
    return _strict_float(field_name, value, nullable=nullable)


def _finite_sequence(value: Sequence[float] | None, field_name: str, *, length: int | None = None) -> list[float] | None:
    if value is None:
        return None
    if isinstance(value, (str, bytes, bytearray)):
        raise SchemaValidationError(f"{field_name} must be a numeric sequence")
    result = [_strict_float(f"{field_name}[{index}]", item, nullable=False) for index, item in enumerate(value)]
    if length is not None and len(result) != length:
        raise SchemaValidationError(f"{field_name} must contain {length} values")
    return [float(item) for item in result]


def _strict_bool(field_name: str, value: object, *, nullable: bool = True) -> bool | None:
    if value is None:
        if nullable:
            return None
        raise SchemaValidationError(f"{field_name} is required")
    if type(value) is not bool:
        item = getattr(value, "item", None)
        if callable(item):
            candidate = item()
            if type(candidate) is bool:
                value = candidate
    if type(value) is not bool:
        raise SchemaValidationError(f"{field_name} must be a boolean")
    return value


def _json_safe(value: Any, field_name: str = "raw_fields") -> Any:
    """Convert telemetry-shaped values to finite, deterministic JSON values."""

    from enum import Enum

    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Integral):
        return validate_int64(field_name, value, nullable=False)
    if isinstance(value, Real):
        return _strict_float(field_name, value, nullable=False)
    if isinstance(value, Enum):
        return _json_safe(value.value, field_name)
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {"__type__": "bytes", "hex": bytes(value).hex()}
    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for key, item in value.items():
            key_text = str(key)
            lowered = key_text.lower()
            if any(token in lowered for token in ("password", "passwd", "token", "secret", "private_key", "ssh_key")):
                result[key_text] = "<redacted>"
            else:
                result[key_text] = _json_safe(item, f"{field_name}.{key_text}")
        return result
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_json_safe(item, f"{field_name}[{index}]") for index, item in enumerate(value)]
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _json_safe(tolist(), field_name)
    item = getattr(value, "item", None)
    if callable(item):
        return _json_safe(item(), field_name)
    slots = getattr(value, "__slots__", ())
    if slots:
        return {
            str(slot): _json_safe(getattr(value, slot), f"{field_name}.{slot}")
            for slot in slots
            if hasattr(value, slot)
        }
    try:
        representation = repr(value)
    except Exception as exc:  # pragma: no cover - hostile user object.
        raise SchemaValidationError(f"{field_name} contains an unsupported object") from exc
    return {"__type__": type(value).__name__, "__repr__": representation}


def _json_text(value: Any, field_name: str) -> str:
    return json.dumps(_json_safe(value, field_name), ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _normalise_row(schema: pa.Schema, row: Mapping[str, Any]) -> dict[str, Any]:
    """Validate every value from the actual Arrow schema before conversion."""

    schema_names = {field.name for field in schema}
    unexpected = set(row) - schema_names
    if unexpected:
        raise SchemaValidationError(f"row contains fields outside schema: {sorted(unexpected)}")
    normalized: dict[str, Any] = {}
    for field in schema:
        value = row.get(field.name)
        if pa.types.is_int64(field.type):
            normalized[field.name] = validate_int64(field.name, value, nullable=field.nullable)
        elif pa.types.is_boolean(field.type):
            normalized[field.name] = _strict_bool(field.name, value, nullable=field.nullable)
        elif pa.types.is_float64(field.type):
            normalized[field.name] = _strict_float(field.name, value, nullable=field.nullable)
        elif pa.types.is_string(field.type):
            if value is None and field.nullable:
                normalized[field.name] = None
            elif not isinstance(value, str):
                raise SchemaValidationError(f"{field.name} must be a string")
            else:
                normalized[field.name] = value
        else:  # pragma: no cover - all Phase A schemas are primitive.
            normalized[field.name] = value
    return normalized


def _validate_nonnegative(field_name: str, value: object, *, nullable: bool = True) -> int | None:
    converted = validate_int64(field_name, value, nullable=nullable)
    if converted is not None and converted < 0:
        raise SchemaValidationError(f"{field_name} must be non-negative")
    return converted


def _canonical_source_record(record: SourceRecord) -> SourceRecord:
    integer_fields = {
        field.name
        for field in source_records_schema()
        if pa.types.is_int64(field.type)
    }
    values = {
        name: validate_int64(name, getattr(record, name), nullable=True)
        for name in integer_fields
    }
    return replace(record, **values)


def _canonical_video_mapping(mapping: VideoMapping) -> VideoMapping:
    return replace(
        mapping,
        video_frame_index=validate_int64("video_frame_index", mapping.video_frame_index),
        video_packet_pts_ticks=validate_int64("video_packet_pts_ticks", mapping.video_packet_pts_ticks),
        video_packet_dts_ticks=validate_int64("video_packet_dts_ticks", mapping.video_packet_dts_ticks),
        source_frame_id=validate_int64("source_frame_id", mapping.source_frame_id),
    )


@dataclass(frozen=True)
class SessionRecordingConfig:
    root: Path
    session_id: str
    application: str = "StereoDistance"
    recording_scope: str = "camera_pixhawk_stereo"
    # Retained only as a compatibility field.  It is deliberately ignored:
    # production completion always requires all seven canonical artifacts.
    require_video_artifacts: bool = True
    started_monotonic_ns: int | None = None
    started_utc_ns: int | None = None
    source_batch_size: int = 256

    def __post_init__(self) -> None:
        _validate_session_id(self.session_id)
        _required_string(self.application, "application")
        _required_string(self.recording_scope, "recording_scope")
        _strict_bool("require_video_artifacts", self.require_video_artifacts, nullable=False)
        validate_int64("started_monotonic_ns", self.started_monotonic_ns)
        validate_int64("started_utc_ns", self.started_utc_ns)
        batch_size = validate_int64("source_batch_size", self.source_batch_size, nullable=False)
        assert batch_size is not None
        if batch_size <= 0:
            raise SchemaValidationError("source_batch_size must be positive")


@dataclass(frozen=True)
class SessionPaths:
    session_dir: Path
    metadata: Path
    source_records: Path
    time_alignment: Path
    left_video: Path
    right_video: Path
    pixhawk_samples: Path
    triangulation: Path

    @classmethod
    def from_config(cls, config: SessionRecordingConfig) -> "SessionPaths":
        session_dir = Path(config.root).expanduser() / f"session_{config.session_id}"
        return cls(
            session_dir=session_dir,
            metadata=session_dir / "session_metadata.json",
            source_records=session_dir / "time" / "source_records.parquet",
            time_alignment=session_dir / "time" / "time_alignment.parquet",
            left_video=session_dir / "camera" / "left_video.mkv",
            right_video=session_dir / "camera" / "right_video.mkv",
            pixhawk_samples=session_dir / "pixhawk" / "samples.parquet",
            triangulation=session_dir / "stereo" / "triangulation.parquet",
        )

    @property
    def manifest(self) -> tuple[str, ...]:
        return (
            "session_metadata.json",
            "time/source_records.parquet",
            "time/time_alignment.parquet",
            "camera/left_video.mkv",
            "camera/right_video.mkv",
            "pixhawk/samples.parquet",
            "stereo/triangulation.parquet",
        )

    @property
    def all_artifacts(self) -> tuple[Path, ...]:
        return (
            self.metadata,
            self.source_records,
            self.time_alignment,
            self.left_video,
            self.right_video,
            self.pixhawk_samples,
            self.triangulation,
        )

    @property
    def integrity_index(self) -> Path:
        return self.session_dir / ".session_integrity.sqlite3"

    def video_path(self, camera_side: str) -> Path:
        if camera_side == "left":
            return self.left_video
        if camera_side == "right":
            return self.right_video
        raise SchemaValidationError(f"unknown camera_side: {camera_side!r}")


@dataclass(frozen=True)
class SourceRecord:
    source_record_id: str
    session_id: str
    camera_side: str
    camera_stream_id: str
    frame_id: int
    camera_sequence: int
    pc_receive_monotonic_ns: int
    width_px: int
    height_px: int
    pc_receive_utc_ns: int | None = None
    rtp_timestamp: int | None = None
    rtp_ssrc: int | None = None
    rtp_marker_seq: int | None = None
    rtcp_sr_ntp_raw: int | None = None
    rtcp_frame_ntp_unix_ns: int | None = None
    pts_ticks: int | None = None
    dts_ticks: int | None = None
    time_base_num: int | None = None
    time_base_den: int | None = None
    video_frame_index: int | None = None
    video_packet_pts_ticks: int | None = None
    video_packet_dts_ticks: int | None = None
    stored_in_video: bool = False
    video_mapping_status: str = "pending"
    drop_reason: str | None = None


@dataclass(frozen=True)
class TimeAlignmentRecord:
    alignment_id: str
    record_type: str
    record_id: str
    alignment_reference_domain: str
    alignment_status: str
    aligned_pc_monotonic_ns: int
    pc_receive_monotonic_ns: int
    camera_side: str | None = None
    pixhawk_id: str | None = None
    pc_receive_utc_ns: int | None = None
    camera_rtp_timestamp: int | None = None
    camera_pts_ticks: int | None = None
    camera_dts_ticks: int | None = None
    camera_rtcp_ntp_unix_ns: int | None = None
    px4_timestamp_us: int | None = None
    px4_timestamp_sample_us: int | None = None
    px4_time_utc_usec: int | None = None
    mavlink_time_usec: int | None = None
    mavlink_time_boot_ms: int | None = None
    camera_rtcp_to_pc_utc_delta_ns: int | None = None
    px4_utc_to_pc_utc_delta_ns: int | None = None


@dataclass(frozen=True)
class PixhawkSampleRecord:
    sample_id: str
    session_id: str
    pixhawk_id: str
    transport: str
    topic: str
    message_type: str
    source_slot: str
    sample_sequence: int
    pc_receive_monotonic_ns: int
    camera_side: str | None = None
    namespace: str | None = None
    pc_receive_utc_ns: int | None = None
    px4_timestamp_us: int | None = None
    px4_timestamp_sample_us: int | None = None
    px4_time_utc_usec: int | None = None
    mavlink_time_usec: int | None = None
    mavlink_time_boot_ms: int | None = None
    device_id: int | None = None
    attitude_q_w: float | None = None
    attitude_q_x: float | None = None
    attitude_q_y: float | None = None
    attitude_q_z: float | None = None
    quat_reset_counter: int | None = None
    delta_q_reset: Sequence[float] | None = None
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    altitude_msl_m: float | None = None
    altitude_ellipsoid_m: float | None = None
    fix_type: int | None = None
    satellites_used: int | None = None
    jamming_state: int | None = None
    spoofing_state: int | None = None
    eph_m: float | None = None
    epv_m: float | None = None
    velocity_m_s: float | None = None
    velocity_north_m_s: float | None = None
    velocity_east_m_s: float | None = None
    velocity_down_m_s: float | None = None
    velocity_ned_valid: bool | None = None
    cog_rad: float | None = None
    heading_rad: float | None = None
    heading_accuracy_rad: float | None = None
    rtcm_injection_rate_hz: float | None = None
    gnss_relative_position: Sequence[float] | None = None
    gnss_relative_accuracy: Sequence[float] | None = None
    position_length_m: float | None = None
    accuracy_length_m: float | None = None
    relative_position_valid: bool | None = None
    gnss_fix_ok: bool | None = None
    differential_solution: bool | None = None
    carrier_solution_floating: bool | None = None
    carrier_solution_fixed: bool | None = None
    moving_base_mode: bool | None = None
    heading_valid: bool | None = None
    angular_velocity_xyz: Sequence[float] | None = None
    angular_velocity_derivative_xyz: Sequence[float] | None = None
    local_position_xyz: Sequence[float] | None = None
    local_velocity_xyz: Sequence[float] | None = None
    raw_fields: Mapping[str, Any] = field(default_factory=dict)


def intersection_angle_from_rays(left_ray: Sequence[float], right_ray: Sequence[float]) -> float:
    left = _finite_sequence(left_ray, "left_ray_optical", length=3)
    right = _finite_sequence(right_ray, "right_ray_optical", length=3)
    assert left is not None and right is not None
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    if left_norm <= 1e-12 or right_norm <= 1e-12:
        raise SchemaValidationError("ray vectors must be non-zero")
    dot = sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm)
    return math.acos(max(-1.0, min(1.0, dot)))


@dataclass(frozen=True)
class TriangulationRecord:
    triangulation_record_id: str
    session_id: str
    triangulation_valid: bool
    triangulation_status: str
    left_source_record_id: str | None = None
    right_source_record_id: str | None = None
    left_frame_id: int | None = None
    right_frame_id: int | None = None
    pair_timestamp_basis: str = "pc_receive_monotonic"
    pair_timestamp_delta_ns: int | None = None
    left_pose_sample_id: str | None = None
    right_pose_sample_id: str | None = None
    intrinsics_path: str | None = None
    intrinsics_zoom_level: int | None = None
    calibration_source_id: str | None = None
    stereo_extrinsics_source: str | None = None
    stereo_extrinsics_frame: str | None = None
    stereo_extrinsics_axis_contract: str | None = None
    left_bbox_center_u_px: float | None = None
    left_bbox_center_v_px: float | None = None
    right_bbox_center_u_px: float | None = None
    right_bbox_center_v_px: float | None = None
    left_confidence: float | None = None
    right_confidence: float | None = None
    class_id: int | None = None
    class_name: str | None = None
    stereo_center_optical_m: Sequence[float] | None = None
    stereo_range_m: float | None = None
    ray_gap_m: float | None = None
    intersection_angle_rad: float | None = None
    baseline_m: float | None = None
    triangulation_source: str | None = None
    triangulation_method: str | None = None
    warnings: Sequence[str] | None = None
    left_ray_optical: Sequence[float] | None = None
    right_ray_optical: Sequence[float] | None = None

    def __post_init__(self) -> None:
        if self.stereo_center_optical_m is not None:
            forward, right, down = map_optical_rdf_to_stereo_frd(self.stereo_center_optical_m)
            object.__setattr__(self, "_mapped_stereo_center_frd", (forward, right, down))
        if self.left_ray_optical is not None and self.right_ray_optical is not None:
            object.__setattr__(self, "intersection_angle_rad", intersection_angle_from_rays(self.left_ray_optical, self.right_ray_optical))

    @classmethod
    def from_optical_rdf(cls, **kwargs: Any) -> "TriangulationRecord":
        return cls(**kwargs)


@dataclass(frozen=True)
class VideoMapping:
    source_record_id: str
    stored_in_video: bool
    video_mapping_status: str
    video_frame_index: int | None = None
    video_packet_pts_ticks: int | None = None
    video_packet_dts_ticks: int | None = None
    drop_reason: str | None = None
    camera_side: str | None = None
    source_frame_id: int | None = None


@dataclass(frozen=True)
class VideoArtifact:
    camera_side: str
    frame_count: int
    writer_status: str = "complete"
    mapping_missing_count: int | None = None
    packet_count: int | None = None


@dataclass(frozen=True)
class SessionCloseResult:
    paths: SessionPaths
    session_state: str
    clean_shutdown: bool
    incomplete: bool
    counts: Mapping[str, int]


def map_optical_rdf_to_stereo_frd(point: Sequence[float]) -> tuple[float, float, float]:
    values = _finite_sequence(point, "stereo_center_optical_m", length=3)
    assert values is not None
    x_right, y_down, z_forward = values
    return z_forward, x_right, y_down


def validate_video_mapping(mapping: VideoMapping) -> None:
    _required_string(mapping.source_record_id, "source_record_id")
    if mapping.camera_side is not None and mapping.camera_side not in _CAMERA_SIDES:
        raise SchemaValidationError(f"unknown camera_side: {mapping.camera_side!r}")
    status = _required_string(mapping.video_mapping_status, "video_mapping_status")
    if status not in _VIDEO_MAPPING_STATUSES:
        raise SchemaValidationError(f"invalid video_mapping_status: {status!r}")
    stored = _strict_bool("stored_in_video", mapping.stored_in_video, nullable=False)
    assert stored is not None
    frame_index = validate_int64("video_frame_index", mapping.video_frame_index)
    if frame_index is not None and frame_index < 0:
        raise SchemaValidationError("video_frame_index must be non-negative")
    validate_int64("video_packet_pts_ticks", mapping.video_packet_pts_ticks)
    validate_int64("video_packet_dts_ticks", mapping.video_packet_dts_ticks)
    validate_int64("source_frame_id", mapping.source_frame_id)
    if mapping.source_frame_id is not None and mapping.source_frame_id < 0:
        raise SchemaValidationError("source_frame_id must be non-negative")
    if status == "pending":
        if stored or frame_index is not None or mapping.drop_reason is not None:
            raise SchemaValidationError("pending mapping must not contain a stored frame or drop reason")
        return
    if status == "stored":
        if not stored or frame_index is None:
            raise SchemaValidationError("stored mapping requires stored_in_video=true and video_frame_index")
        if mapping.drop_reason is not None:
            raise SchemaValidationError("stored mapping cannot have drop_reason")
        return
    if stored or frame_index is not None:
        raise SchemaValidationError(f"{status} mapping cannot contain a stored frame")
    _required_string(mapping.drop_reason, "drop_reason")


class SessionRecorder:
    """Synchronous, thread-safe recorder with bounded source row batching.

    All public methods serialize through one lock.  This is intentionally a
    synchronous Phase A contract; callers must not call ParquetWriter
    directly.  A future bounded producer queue can feed these same methods.
    """

    _TABLES = ("source_records", "time_alignment", "pixhawk_samples", "triangulation")

    def __init__(self, config: SessionRecordingConfig, paths: SessionPaths) -> None:
        self.config = config
        self.paths = paths
        self._schemas = {
            "source_records": source_records_schema(),
            "time_alignment": time_alignment_schema(),
            "pixhawk_samples": pixhawk_samples_schema(),
            "triangulation": triangulation_schema(),
        }
        self._writers: dict[str, pq.ParquetWriter] = {}
        self._lock = threading.RLock()
        self._index: sqlite3.Connection | None = None
        self._state = "recording"
        self._closed = False
        self._writers_closed = False
        self._index_closed = False
        self._metadata_finalized = False
        self._pending_final_state: str | None = None
        self._failure_stage: str | None = None
        self._failure_message: str | None = None
        self._ended_monotonic_ns: int | None = None
        self._ended_utc_ns: int | None = None
        self._counts = {name: 0 for name in self._TABLES}
        self._source_batch: list[tuple[str, dict[str, Any]]] = []
        self._uncommitted_source_index_rows = 0
        self._video_artifacts: dict[str, VideoArtifact] = {}
        self._video_probes: dict[str, Any] = {}

    @classmethod
    def open(cls, config: SessionRecordingConfig) -> "SessionRecorder":
        started_monotonic = config.started_monotonic_ns if config.started_monotonic_ns is not None else time.monotonic_ns()
        started_utc = config.started_utc_ns if config.started_utc_ns is not None else time.time_ns()
        config = replace(
            config,
            started_monotonic_ns=validate_int64("started_monotonic_ns", started_monotonic, nullable=False),
            started_utc_ns=validate_int64("started_utc_ns", started_utc, nullable=False),
        )
        paths = SessionPaths.from_config(config)
        if paths.session_dir.exists():
            raise SessionRecordingError(f"refusing to overwrite existing session: {paths.session_dir}")
        paths.session_dir.mkdir(parents=True, exist_ok=False)
        for directory in (paths.source_records.parent, paths.left_video.parent, paths.pixhawk_samples.parent, paths.triangulation.parent):
            directory.mkdir(parents=True, exist_ok=True)

        recorder = cls(config, paths)
        # Provisional metadata is deliberately the first artifact write.
        try:
            recorder._write_metadata_locked()
        except Exception:
            raise
        try:
            recorder._open_integrity_index_locked()
            for table_name in cls._TABLES:
                recorder._writers[table_name] = pq.ParquetWriter(
                    str(recorder._path_for_table(table_name)),
                    recorder._schemas[table_name],
                    compression="zstd",
                )
        except Exception as exc:
            recorder._set_failure_locked(f"open_{table_name if 'table_name' in locals() else 'integrity_index'}", str(exc), state="open_failed")
            for writer in tuple(recorder._writers.values()):
                try:
                    writer.close()
                except Exception:
                    pass
            recorder._writers.clear()
            recorder._writers_closed = True
            recorder._close_index_locked()
            try:
                recorder._write_metadata_locked()
            except Exception:
                pass
            raise SessionRecordingError(f"failed to open session resources: {exc}") from exc
        return recorder

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._closed

    @property
    def source_schema(self) -> pa.Schema:
        return self._schemas["source_records"]

    @property
    def time_alignment_schema(self) -> pa.Schema:
        return self._schemas["time_alignment"]

    @property
    def pixhawk_schema(self) -> pa.Schema:
        return self._schemas["pixhawk_samples"]

    @property
    def triangulation_schema(self) -> pa.Schema:
        return self._schemas["triangulation"]

    def resolve_source_record(self, source_record_id: str) -> SourceRecord | None:
        """Resolve a source record for an authoritative video association."""

        with self._lock:
            if self._index is None:
                return None
            row = self._index.execute(
                "SELECT record_json FROM source_index WHERE source_record_id=?",
                (source_record_id,),
            ).fetchone()
            if row is None:
                return None
            return SourceRecord(**json.loads(row[0]))

    def _path_for_table(self, table_name: str) -> Path:
        return {
            "source_records": self.paths.source_records,
            "time_alignment": self.paths.time_alignment,
            "pixhawk_samples": self.paths.pixhawk_samples,
            "triangulation": self.paths.triangulation,
        }[table_name]

    def _open_integrity_index_locked(self) -> None:
        self._index = sqlite3.connect(str(self.paths.integrity_index), check_same_thread=False)
        self._index.execute("PRAGMA journal_mode=DELETE")
        self._index.executescript(
            """
            CREATE TABLE source_index (
                source_record_id TEXT PRIMARY KEY,
                camera_side TEXT NOT NULL,
                frame_id INTEGER NOT NULL,
                camera_sequence INTEGER NOT NULL,
                record_json TEXT NOT NULL,
                video_frame_index INTEGER,
                stored_in_video INTEGER NOT NULL,
                video_mapping_status TEXT NOT NULL,
                video_packet_pts_ticks INTEGER,
                video_packet_dts_ticks INTEGER,
                drop_reason TEXT,
                emitted INTEGER NOT NULL DEFAULT 0,
                queued INTEGER NOT NULL DEFAULT 0
            );
            CREATE UNIQUE INDEX source_video_frame_unique
                ON source_index(camera_side, video_frame_index)
                WHERE stored_in_video = 1;
            CREATE TABLE pending_mappings (
                source_record_id TEXT PRIMARY KEY,
                camera_side TEXT,
                mapping_json TEXT NOT NULL
            );
            CREATE TABLE alignment_index (alignment_id TEXT PRIMARY KEY);
            CREATE TABLE pixhawk_index (
                sample_id TEXT PRIMARY KEY,
                pixhawk_id TEXT NOT NULL,
                camera_side TEXT,
                source_slot TEXT NOT NULL
            );
            CREATE TABLE triangulation_index (triangulation_record_id TEXT PRIMARY KEY);
            """
        )
        self._index.commit()

    def _close_index_locked(self) -> None:
        if self._index is not None and not self._index_closed:
            self._index.close()
            self._index_closed = True
            self._index = None

    def _require_recording_locked(self) -> None:
        if self._closed or self._state != "recording":
            raise SessionStateError(f"session is not recording: {self._state}")
        if self._index is None or self._writers_closed:
            raise SessionStateError("session resources are closed")

    def _set_failure_locked(self, stage: str, message: str, *, state: str = "incomplete") -> None:
        if self._failure_stage is None:
            self._failure_stage = _required_string(stage, "failure_stage")
            self._failure_message = _required_string(message, "failure_message")[-4000:]
        if state != "metadata_finalize_failed" or self._state != "metadata_finalize_failed":
            self._state = state

    def record_camera_source(self, record: SourceRecord) -> None:
        with self._lock:
            self._require_recording_locked()
            self._validate_source_record(record)
            record = _canonical_source_record(record)
            assert self._index is not None
            if self._index.execute("SELECT 1 FROM source_index WHERE source_record_id=?", (record.source_record_id,)).fetchone():
                raise SchemaValidationError(f"duplicate source_record_id: {record.source_record_id}")
            mapping_row = self._index.execute(
                "SELECT camera_side, mapping_json FROM pending_mappings WHERE source_record_id=?",
                (record.source_record_id,),
            ).fetchone()
            if mapping_row is not None:
                mapping = VideoMapping(**json.loads(mapping_row[1]))
                if mapping.camera_side is not None and mapping.camera_side != record.camera_side:
                    raise SchemaValidationError("pending mapping camera_side does not match source record")
                if mapping.source_frame_id is not None and mapping.source_frame_id != record.frame_id:
                    raise SchemaValidationError("pending mapping source_frame_id does not match source record")
                record = self._record_with_mapping(record, mapping)
            mapping = VideoMapping(
                source_record_id=record.source_record_id,
                camera_side=record.camera_side,
                stored_in_video=record.stored_in_video,
                video_mapping_status=record.video_mapping_status,
                video_frame_index=record.video_frame_index,
                video_packet_pts_ticks=record.video_packet_pts_ticks,
                video_packet_dts_ticks=record.video_packet_dts_ticks,
                drop_reason=record.drop_reason,
            )
            validate_video_mapping(mapping)
            record_json = json.dumps(_json_safe(dataclasses.asdict(record), "source_record"), sort_keys=True, separators=(",", ":"))
            try:
                self._index.execute(
                    """INSERT INTO source_index(
                        source_record_id,camera_side,frame_id,camera_sequence,record_json,
                        video_frame_index,stored_in_video,video_mapping_status,
                        video_packet_pts_ticks,video_packet_dts_ticks,drop_reason
                    ) VALUES(?,?,?,?,?,?,?,?,?,?,?)""",
                    (
                        record.source_record_id, record.camera_side, record.frame_id, record.camera_sequence, record_json,
                        record.video_frame_index, int(record.stored_in_video), record.video_mapping_status,
                        record.video_packet_pts_ticks, record.video_packet_dts_ticks, record.drop_reason,
                    ),
                )
            except sqlite3.IntegrityError as exc:
                raise SchemaValidationError(f"duplicate source association: {record.source_record_id}") from exc
            if mapping_row is not None:
                self._index.execute("DELETE FROM pending_mappings WHERE source_record_id=?", (record.source_record_id,))
            self._uncommitted_source_index_rows += 1
            if self._uncommitted_source_index_rows >= self.config.source_batch_size:
                self._index.commit()
                self._uncommitted_source_index_rows = 0
            if record.video_mapping_status in _SOURCE_MAPPING_FINAL_STATUSES:
                self._queue_source_locked(record.source_record_id)
            if record.video_mapping_status == "writer_failure":
                self._set_failure_locked("video_mapping", record.drop_reason or "writer failure")

    def mark_video_mapping(self, mapping: VideoMapping) -> None:
        with self._lock:
            self._require_recording_locked()
            validate_video_mapping(mapping)
            mapping = _canonical_video_mapping(mapping)
            if mapping.video_mapping_status == "pending":
                raise SchemaValidationError("mark_video_mapping requires a final mapping status")
            assert self._index is not None
            source = self._index.execute(
                "SELECT camera_side,frame_id,record_json,video_mapping_status FROM source_index WHERE source_record_id=?",
                (mapping.source_record_id,),
            ).fetchone()
            if source is None:
                if self._index.execute("SELECT 1 FROM pending_mappings WHERE source_record_id=?", (mapping.source_record_id,)).fetchone():
                    raise SchemaValidationError(f"duplicate video mapping: {mapping.source_record_id}")
                self._index.execute(
                    "INSERT INTO pending_mappings(source_record_id,camera_side,mapping_json) VALUES(?,?,?)",
                    (mapping.source_record_id, mapping.camera_side, json.dumps(_json_safe(dataclasses.asdict(mapping), "video_mapping"), sort_keys=True)),
                )
                self._index.commit()
                if mapping.video_mapping_status == "writer_failure":
                    self._set_failure_locked("video_mapping", mapping.drop_reason or "writer failure")
                return
            source_side, source_frame_id, source_json, current_status = source
            if mapping.camera_side is not None and mapping.camera_side != source_side:
                raise SchemaValidationError("video mapping camera_side does not match source record")
            if mapping.source_frame_id is not None and mapping.source_frame_id != source_frame_id:
                raise SchemaValidationError("video mapping source_frame_id does not match source record")
            if current_status != "pending":
                raise SchemaValidationError(f"duplicate video mapping: {mapping.source_record_id}")
            record = json.loads(source_json)
            record = self._record_with_mapping_dict(record, mapping)
            try:
                self._index.execute(
                    """UPDATE source_index SET record_json=?,video_frame_index=?,stored_in_video=?,
                       video_mapping_status=?,video_packet_pts_ticks=?,video_packet_dts_ticks=?,drop_reason=?
                       WHERE source_record_id=?""",
                    (
                        json.dumps(_json_safe(record, "source_record"), sort_keys=True, separators=(",", ":")),
                        mapping.video_frame_index, int(mapping.stored_in_video), mapping.video_mapping_status,
                        mapping.video_packet_pts_ticks, mapping.video_packet_dts_ticks, mapping.drop_reason,
                        mapping.source_record_id,
                    ),
                )
                self._index.commit()
            except sqlite3.IntegrityError as exc:
                self._index.rollback()
                raise SchemaValidationError("duplicate video_frame_index for camera side") from exc
            self._queue_source_locked(mapping.source_record_id)
            if mapping.video_mapping_status == "writer_failure":
                self._set_failure_locked("video_mapping", mapping.drop_reason or "writer failure")

    def record_time_alignment(self, record: TimeAlignmentRecord) -> None:
        with self._lock:
            self._require_recording_locked()
            self._validate_time_alignment(record)
            assert self._index is not None
            try:
                self._index.execute("INSERT INTO alignment_index(alignment_id) VALUES(?)", (record.alignment_id,))
                self._index.commit()
            except sqlite3.IntegrityError as exc:
                self._index.rollback()
                raise SchemaValidationError(f"duplicate alignment_id: {record.alignment_id}") from exc
            self._write_row_locked("time_alignment", self._time_alignment_row(record))

    def record_pixhawk_sample(self, record: PixhawkSampleRecord) -> None:
        with self._lock:
            self._require_recording_locked()
            self._validate_pixhawk_sample(record)
            assert self._index is not None
            try:
                self._index.execute(
                    "INSERT INTO pixhawk_index(sample_id,pixhawk_id,camera_side,source_slot) VALUES(?,?,?,?)",
                    (record.sample_id, record.pixhawk_id, record.camera_side, record.source_slot),
                )
                self._index.commit()
            except sqlite3.IntegrityError as exc:
                self._index.rollback()
                raise SchemaValidationError(f"duplicate sample_id: {record.sample_id}") from exc
            self._write_row_locked("pixhawk_samples", self._pixhawk_row(record))

    def record_triangulation(self, record: TriangulationRecord) -> None:
        with self._lock:
            self._require_recording_locked()
            self._validate_triangulation(record)
            assert self._index is not None
            try:
                self._index.execute("INSERT INTO triangulation_index(triangulation_record_id) VALUES(?)", (record.triangulation_record_id,))
                self._index.commit()
            except sqlite3.IntegrityError as exc:
                self._index.rollback()
                raise SchemaValidationError(f"duplicate triangulation_record_id: {record.triangulation_record_id}") from exc
            self._write_row_locked("triangulation", self._triangulation_row(record))

    def register_video_artifact(self, artifact: VideoArtifact, *, probe: Any | None = None) -> None:
        with self._lock:
            self._require_recording_locked()
            if artifact.camera_side not in _CAMERA_SIDES:
                raise SchemaValidationError(f"unknown camera_side: {artifact.camera_side!r}")
            validate_int64("frame_count", artifact.frame_count, nullable=False)
            if artifact.writer_status not in {"complete", "failed", "incomplete"}:
                raise SchemaValidationError(f"invalid writer_status: {artifact.writer_status!r}")
            if artifact.mapping_missing_count is not None:
                _validate_nonnegative("mapping_missing_count", artifact.mapping_missing_count, nullable=False)
            validate_int64("packet_count", artifact.packet_count)
            if artifact.writer_status == "complete":
                from .session_video import probe_mkv

                path = self.paths.video_path(artifact.camera_side)
                try:
                    actual_probe = probe_mkv(path, include_packets=True)
                except Exception as exc:
                    raise SessionRecordingError(f"completed video artifact failed authoritative probe: {path}: {exc}") from exc
                if actual_probe.decoded_frame_count is None:
                    raise SessionRecordingError(f"completed video artifact has no decoded frame count: {path}")
                if "matroska" not in actual_probe.format_name.split(",") or actual_probe.codec_name != "h264" or actual_probe.decoded_frame_count < 1:
                    raise SessionRecordingError(f"video artifact is not a non-empty H.264 Matroska file: {path}")
                self._video_probes[artifact.camera_side] = actual_probe
                self._video_artifacts[artifact.camera_side] = replace(
                    artifact,
                    frame_count=actual_probe.decoded_frame_count,
                    packet_count=actual_probe.packet_count,
                    mapping_missing_count=None,
                )
            else:
                self._video_artifacts[artifact.camera_side] = artifact

    def mark_failure(self, stage: str, message: str) -> None:
        with self._lock:
            self._require_recording_locked()
            self._set_failure_locked(stage, message)

    def close(self) -> SessionCloseResult:
        with self._lock:
            if self._closed:
                return self._close_result_locked()
            if self._state == "metadata_finalize_failed":
                return self._retry_metadata_finalize_locked()

            self._state = "finalizing"
            if self._failure_stage is None:
                try:
                    self._validate_normal_close_locked()
                except Exception as exc:
                    self._set_failure_locked("finalize", str(exc))
            try:
                self._flush_all_source_rows_locked()
            except Exception as exc:
                self._set_failure_locked("source_records_flush", str(exc))
            self._close_writers_locked()
            if self._failure_stage is None:
                try:
                    self._validate_complete_artifacts_locked()
                except Exception as exc:
                    self._set_failure_locked("artifact_integrity", str(exc))
            self._ended_monotonic_ns = time.monotonic_ns()
            self._ended_utc_ns = time.time_ns()
            target_state = "complete" if self._failure_stage is None else "incomplete"
            self._pending_final_state = target_state
            try:
                self._write_metadata_locked(state_override=target_state)
            except Exception as exc:
                self._state = "metadata_finalize_failed"
                self._metadata_finalized = False
                self._failure_stage = self._failure_stage or "metadata_finalize"
                self._failure_message = self._failure_message or str(exc)
                raise SessionRecordingError(f"metadata finalization failed: {exc}") from exc
            return self._finish_metadata_finalize_locked(target_state)

    def _retry_metadata_finalize_locked(self) -> SessionCloseResult:
        target = self._pending_final_state or "incomplete"
        try:
            self._write_metadata_locked(state_override=target)
        except Exception as exc:
            self._failure_stage = self._failure_stage or "metadata_finalize"
            self._failure_message = self._failure_message or str(exc)
            self._state = "metadata_finalize_failed"
            raise SessionRecordingError(f"metadata finalization retry failed: {exc}") from exc
        return self._finish_metadata_finalize_locked(target)

    def _finish_metadata_finalize_locked(self, target_state: str) -> SessionCloseResult:
        """Commit metadata state, then remove diagnostics only after a complete write."""

        if target_state != "complete":
            self._state = target_state
            self._metadata_finalized = True
            self._close_index_locked()
            self._closed = True
            return self._close_result_locked()

        try:
            cleanup_succeeded = self._remove_integrity_index_locked()
        except Exception as exc:
            cleanup_succeeded = False
            self._set_failure_locked("integrity_index_cleanup", str(exc))
        if not cleanup_succeeded:
            # The first complete candidate has already been written, but the
            # diagnostic index is evidence that completion was not fully
            # committed.  Replace it with an incomplete state before return.
            self._pending_final_state = "incomplete"
            self._metadata_finalized = False
            self._state = "incomplete"
            try:
                self._write_metadata_locked(state_override="incomplete")
            except Exception as exc:
                self._state = "metadata_finalize_failed"
                self._metadata_finalized = False
                self._failure_stage = self._failure_stage or "metadata_finalize"
                self._failure_message = self._failure_message or str(exc)
                raise SessionRecordingError(f"metadata finalization after index cleanup failure failed: {exc}") from exc
            self._metadata_finalized = True
            self._close_index_locked()
            self._closed = True
            return self._close_result_locked()

        self._state = "complete"
        self._metadata_finalized = True
        self._closed = True
        return self._close_result_locked()

    def _close_writers_locked(self) -> None:
        if self._writers_closed:
            return
        for name, writer in tuple(self._writers.items()):
            try:
                writer.close()
            except Exception as exc:
                self._set_failure_locked("parquet_close", f"{name}: {exc}")
        self._writers.clear()
        self._writers_closed = True

    def _remove_integrity_index_locked(self) -> bool:
        self._close_index_locked()
        try:
            self.paths.integrity_index.unlink(missing_ok=True)
        except OSError as exc:
            self._set_failure_locked("integrity_index_cleanup", str(exc))
            return False
        return True

    def _write_row_locked(self, table_name: str, row: Mapping[str, Any]) -> None:
        writer = self._writers.get(table_name)
        if writer is None:
            raise SessionStateError(f"writer is closed: {table_name}")
        normalized = _normalise_row(self._schemas[table_name], row)
        try:
            writer.write_table(pa.Table.from_pylist([normalized], schema=self._schemas[table_name]))
        except Exception as exc:
            self._set_failure_locked(f"{table_name}_write", str(exc))
            raise SessionRecordingError(f"failed to write {table_name}: {exc}") from exc
        self._counts[table_name] += 1

    @staticmethod
    def _record_with_mapping(record: SourceRecord, mapping: VideoMapping) -> SourceRecord:
        return replace(
            record,
            stored_in_video=mapping.stored_in_video,
            video_mapping_status=mapping.video_mapping_status,
            video_frame_index=mapping.video_frame_index,
            video_packet_pts_ticks=mapping.video_packet_pts_ticks,
            video_packet_dts_ticks=mapping.video_packet_dts_ticks,
            drop_reason=mapping.drop_reason,
        )

    @staticmethod
    def _record_with_mapping_dict(record: dict[str, Any], mapping: VideoMapping) -> dict[str, Any]:
        record.update(
            stored_in_video=mapping.stored_in_video,
            video_mapping_status=mapping.video_mapping_status,
            video_frame_index=mapping.video_frame_index,
            video_packet_pts_ticks=mapping.video_packet_pts_ticks,
            video_packet_dts_ticks=mapping.video_packet_dts_ticks,
            drop_reason=mapping.drop_reason,
        )
        return record

    def _queue_source_locked(self, source_record_id: str) -> None:
        assert self._index is not None
        row = self._index.execute("SELECT record_json,queued,emitted FROM source_index WHERE source_record_id=?", (source_record_id,)).fetchone()
        if row is None:
            raise SchemaValidationError(f"unknown source_record_id: {source_record_id}")
        record_json, queued, emitted = row
        if queued or emitted:
            return
        record = json.loads(record_json)
        record["video_path"] = f"camera/{record['camera_side']}_video.mkv"
        self._index.execute("UPDATE source_index SET queued=1 WHERE source_record_id=?", (source_record_id,))
        self._source_batch.append((source_record_id, record))
        if len(self._source_batch) >= self.config.source_batch_size:
            self._flush_source_batch_locked()

    def _flush_source_batch_locked(self) -> None:
        if not self._source_batch:
            return
        batch = list(self._source_batch)
        rows = [row for _, row in batch]
        normalized = [_normalise_row(self._schemas["source_records"], row) for row in rows]
        writer = self._writers.get("source_records")
        if writer is None:
            raise SessionStateError("source_records writer is closed")
        try:
            writer.write_table(pa.Table.from_pylist(normalized, schema=self._schemas["source_records"]))
        except Exception as exc:
            self._set_failure_locked("source_records_write", str(exc))
            raise SessionRecordingError(f"failed to write source_records: {exc}") from exc
        assert self._index is not None
        self._index.executemany("UPDATE source_index SET emitted=1 WHERE source_record_id=?", [(source_id,) for source_id, _ in batch])
        self._index.commit()
        self._uncommitted_source_index_rows = 0
        self._counts["source_records"] += len(batch)
        self._source_batch.clear()

    def _flush_all_source_rows_locked(self) -> None:
        self._flush_source_batch_locked()
        if self._index is None or self._writers_closed:
            return
        cursor = self._index.execute("SELECT source_record_id,record_json FROM source_index WHERE emitted=0 ORDER BY rowid")
        while True:
            rows = cursor.fetchmany(self.config.source_batch_size)
            if not rows:
                break
            for source_id, record_json in rows:
                record = json.loads(record_json)
                record["video_path"] = f"camera/{record['camera_side']}_video.mkv"
                self._source_batch.append((source_id, record))
                if len(self._source_batch) >= self.config.source_batch_size:
                    self._flush_source_batch_locked()
        self._flush_source_batch_locked()

    def _validate_source_record(self, record: SourceRecord) -> None:
        if record.session_id != self.config.session_id:
            raise SchemaValidationError("source record session_id does not match recorder")
        for field_name in ("source_record_id", "camera_stream_id"):
            _required_string(getattr(record, field_name), field_name)
        if record.camera_side not in _CAMERA_SIDES:
            raise SchemaValidationError(f"unknown camera_side: {record.camera_side!r}")
        for field_name in ("frame_id", "camera_sequence", "pc_receive_monotonic_ns", "width_px", "height_px"):
            _validate_nonnegative(field_name, getattr(record, field_name), nullable=False)
        for field_name in (
            "pc_receive_utc_ns", "rtp_timestamp", "rtp_ssrc", "rtp_marker_seq", "rtcp_sr_ntp_raw",
            "rtcp_frame_ntp_unix_ns", "pts_ticks", "dts_ticks", "time_base_num", "time_base_den",
            "video_frame_index", "video_packet_pts_ticks", "video_packet_dts_ticks",
        ):
            validate_int64(field_name, getattr(record, field_name))
        if (record.time_base_num is None) != (record.time_base_den is None):
            raise SchemaValidationError("time_base_num and time_base_den must both be present or null")
        if record.time_base_num is not None and (record.time_base_num <= 0 or record.time_base_den is None or record.time_base_den <= 0):
            raise SchemaValidationError("time_base_num/time_base_den must be positive")
        _strict_bool("stored_in_video", record.stored_in_video, nullable=False)
        validate_video_mapping(VideoMapping(
            source_record_id=record.source_record_id,
            camera_side=record.camera_side,
            stored_in_video=record.stored_in_video,
            video_mapping_status=record.video_mapping_status,
            video_frame_index=record.video_frame_index,
            video_packet_pts_ticks=record.video_packet_pts_ticks,
            video_packet_dts_ticks=record.video_packet_dts_ticks,
            drop_reason=record.drop_reason,
        ))

    def _validate_time_alignment(self, record: TimeAlignmentRecord) -> None:
        for field_name in ("alignment_id", "record_type", "record_id"):
            _required_string(getattr(record, field_name), field_name)
        if record.alignment_reference_domain not in _ALIGNMENT_REFERENCE_DOMAINS:
            raise SchemaValidationError(f"invalid alignment_reference_domain: {record.alignment_reference_domain!r}")
        if record.alignment_status not in _ALIGNMENT_STATUSES:
            raise SchemaValidationError(f"invalid alignment_status: {record.alignment_status!r}")
        if record.camera_side is not None and record.camera_side not in _CAMERA_SIDES:
            raise SchemaValidationError(f"unknown camera_side: {record.camera_side!r}")
        for field in time_alignment_schema():
            if pa.types.is_int64(field.type):
                validate_int64(field.name, getattr(record, field.name))
        _validate_nonnegative("aligned_pc_monotonic_ns", record.aligned_pc_monotonic_ns, nullable=False)
        _validate_nonnegative("pc_receive_monotonic_ns", record.pc_receive_monotonic_ns, nullable=False)
        if record.alignment_status == "receive_time_fallback":
            if record.alignment_reference_domain != "pc_receive_monotonic" or record.aligned_pc_monotonic_ns != record.pc_receive_monotonic_ns:
                raise SchemaValidationError("receive_time_fallback must use the PC receive monotonic timestamp")
        elif record.alignment_status == "utc_aligned":
            if record.alignment_reference_domain == "camera_rtcp_utc" and record.camera_rtcp_ntp_unix_ns is None:
                raise SchemaValidationError("camera_rtcp_utc alignment requires camera_rtcp_ntp_unix_ns")
            if record.alignment_reference_domain == "px4_utc" and record.px4_time_utc_usec is None:
                raise SchemaValidationError("px4_utc alignment requires px4_time_utc_usec")
            if record.alignment_reference_domain not in {"camera_rtcp_utc", "px4_utc"}:
                raise SchemaValidationError("utc_aligned requires a UTC reference domain")
        elif record.alignment_status == "missing" and record.alignment_reference_domain != "unavailable":
            raise SchemaValidationError("missing alignment must use unavailable domain")
        if record.camera_rtcp_to_pc_utc_delta_ns is not None:
            if record.pc_receive_utc_ns is None or record.camera_rtcp_ntp_unix_ns is None:
                raise SchemaValidationError("camera UTC delta requires both UTC operands")
            expected = validate_int64(
                "camera_rtcp_to_pc_utc_delta_ns",
                record.pc_receive_utc_ns - record.camera_rtcp_ntp_unix_ns,
                nullable=False,
            )
            assert expected is not None
            if record.camera_rtcp_to_pc_utc_delta_ns != expected:
                raise SchemaValidationError("camera_rtcp_to_pc_utc_delta_ns does not match UTC operands")
        if record.px4_utc_to_pc_utc_delta_ns is not None:
            if record.pc_receive_utc_ns is None or record.px4_time_utc_usec is None:
                raise SchemaValidationError("PX4 UTC delta requires both UTC operands")
            expected = validate_int64(
                "px4_utc_to_pc_utc_delta_ns",
                record.pc_receive_utc_ns - record.px4_time_utc_usec * 1000,
                nullable=False,
            )
            assert expected is not None
            if record.px4_utc_to_pc_utc_delta_ns != expected:
                raise SchemaValidationError("px4_utc_to_pc_utc_delta_ns does not match UTC operands")

    def _validate_pixhawk_sample(self, record: PixhawkSampleRecord) -> None:
        if record.session_id != self.config.session_id:
            raise SchemaValidationError("Pixhawk sample session_id does not match recorder")
        for field_name in ("sample_id", "pixhawk_id", "transport", "topic", "message_type", "source_slot"):
            _required_string(getattr(record, field_name), field_name)
        if record.camera_side is not None and record.camera_side not in _CAMERA_SIDES:
            raise SchemaValidationError(f"unknown camera_side: {record.camera_side!r}")
        for field in pixhawk_samples_schema():
            if pa.types.is_int64(field.type):
                validate_int64(field.name, getattr(record, field.name, None))
            elif pa.types.is_boolean(field.type):
                _strict_bool(field.name, getattr(record, field.name, None))
        _validate_nonnegative("sample_sequence", record.sample_sequence, nullable=False)
        _validate_nonnegative("pc_receive_monotonic_ns", record.pc_receive_monotonic_ns, nullable=False)
        q = [record.attitude_q_w, record.attitude_q_x, record.attitude_q_y, record.attitude_q_z]
        if any(value is not None for value in q):
            if any(value is None for value in q):
                raise SchemaValidationError("attitude quaternion must contain all four wxyz values")
            q_values = [_strict_float(f"attitude_q_{axis}", value, nullable=False) for axis, value in zip("wxyz", q)]
            assert all(value is not None for value in q_values)
            if math.sqrt(sum(value * value for value in q_values)) <= 1e-12:
                raise SchemaValidationError("attitude quaternion norm must be positive")
        for field_name in (
            "attitude_q_w", "attitude_q_x", "attitude_q_y", "attitude_q_z", "latitude_deg", "longitude_deg",
            "altitude_msl_m", "altitude_ellipsoid_m", "eph_m", "epv_m", "velocity_m_s", "velocity_north_m_s",
            "velocity_east_m_s", "velocity_down_m_s", "cog_rad", "heading_rad", "heading_accuracy_rad",
            "rtcm_injection_rate_hz", "position_length_m", "accuracy_length_m",
        ):
            _strict_float(field_name, getattr(record, field_name))
        for field_name in (
            "delta_q_reset", "gnss_relative_position", "gnss_relative_accuracy", "angular_velocity_xyz",
            "angular_velocity_derivative_xyz", "local_position_xyz", "local_velocity_xyz",
        ):
            _finite_sequence(getattr(record, field_name), field_name)
        _json_text(record.raw_fields, "raw_fields")

    def _source_lookup(self, source_id: str) -> tuple[Any, ...] | None:
        assert self._index is not None
        return self._index.execute(
            "SELECT camera_side,frame_id,camera_sequence,video_frame_index,stored_in_video,video_mapping_status FROM source_index WHERE source_record_id=?",
            (source_id,),
        ).fetchone()

    def _pose_lookup(self, sample_id: str) -> tuple[Any, ...] | None:
        assert self._index is not None
        return self._index.execute("SELECT pixhawk_id,camera_side,source_slot FROM pixhawk_index WHERE sample_id=?", (sample_id,)).fetchone()

    def _validate_triangulation(self, record: TriangulationRecord) -> None:
        if record.session_id != self.config.session_id:
            raise SchemaValidationError("triangulation session_id does not match recorder")
        _required_string(record.triangulation_record_id, "triangulation_record_id")
        _required_string(record.triangulation_status, "triangulation_status")
        if record.pair_timestamp_basis not in _PAIR_TIMESTAMP_BASES:
            raise SchemaValidationError(f"invalid pair_timestamp_basis: {record.pair_timestamp_basis!r}")
        _strict_bool("triangulation_valid", record.triangulation_valid, nullable=False)
        for source_id, expected_side, frame_id, label in (
            (record.left_source_record_id, "left", record.left_frame_id, "left"),
            (record.right_source_record_id, "right", record.right_frame_id, "right"),
        ):
            if source_id is None:
                continue
            source = self._source_lookup(source_id)
            if source is None:
                raise SchemaValidationError(f"triangulation references unknown {label} source_record_id: {source_id}")
            source_side, source_frame_id, *_ = source
            if source_side != expected_side:
                raise SchemaValidationError(f"{label} source record has camera_side={source_side!r}")
            if frame_id is not None and frame_id != source_frame_id:
                raise SchemaValidationError(f"{label}_frame_id does not match source record")
        for sample_id, expected_side, label in (
            (record.left_pose_sample_id, "left", "left"),
            (record.right_pose_sample_id, "right", "right"),
        ):
            if sample_id is None:
                continue
            pose = self._pose_lookup(sample_id)
            if pose is None:
                raise SchemaValidationError(f"triangulation references unknown {label} pose sample_id: {sample_id}")
            _, pose_side, source_slot = pose
            if pose_side != expected_side or source_slot != "vehicle_attitude":
                raise SchemaValidationError(f"{label} pose must be the matching-side vehicle_attitude sample")
        for field in triangulation_schema():
            if pa.types.is_int64(field.type):
                validate_int64(field.name, getattr(record, field.name, None))
            elif pa.types.is_float64(field.type):
                # Output fields are built below; validate input counterparts.
                if hasattr(record, field.name):
                    _strict_float(field.name, getattr(record, field.name))
        if record.stereo_center_optical_m is not None:
            map_optical_rdf_to_stereo_frd(record.stereo_center_optical_m)
        if record.triangulation_valid:
            if record.stereo_center_optical_m is None or record.stereo_range_m is None:
                raise SchemaValidationError("valid triangulation requires optical center and stereo_range_m")
        _strict_float("stereo_range_m", record.stereo_range_m)
        _strict_float("ray_gap_m", record.ray_gap_m)
        _strict_float("intersection_angle_rad", record.intersection_angle_rad)
        _strict_float("baseline_m", record.baseline_m)
        _json_text(record.warnings or [], "warnings")

    @staticmethod
    def _source_row_from_json(record: Mapping[str, Any]) -> dict[str, Any]:
        row = dict(record)
        row["video_path"] = f"camera/{row['camera_side']}_video.mkv"
        return row

    @staticmethod
    def _time_alignment_row(record: TimeAlignmentRecord) -> dict[str, Any]:
        return dataclasses.asdict(record)

    @staticmethod
    def _pixhawk_row(record: PixhawkSampleRecord) -> dict[str, Any]:
        row = dataclasses.asdict(record)
        for source_name, target_name in (
            ("delta_q_reset", "delta_q_reset_json"),
            ("gnss_relative_position", "gnss_relative_position_json"),
            ("gnss_relative_accuracy", "gnss_relative_accuracy_json"),
            ("angular_velocity_xyz", "angular_velocity_xyz_json"),
            ("angular_velocity_derivative_xyz", "angular_velocity_derivative_xyz_json"),
            ("local_position_xyz", "local_position_xyz_json"),
            ("local_velocity_xyz", "local_velocity_xyz_json"),
        ):
            value = row.pop(source_name)
            row[target_name] = _json_text(value, source_name) if value is not None else None
        row.pop("raw_fields", None)
        row["raw_fields_json"] = _json_text(record.raw_fields, "raw_fields")
        return row

    @staticmethod
    def _triangulation_row(record: TriangulationRecord) -> dict[str, Any]:
        row = dataclasses.asdict(record)
        mapped = getattr(record, "_mapped_stereo_center_frd", None)
        if mapped is None and record.stereo_center_optical_m is not None:
            mapped = map_optical_rdf_to_stereo_frd(record.stereo_center_optical_m)
        forward, right, down = mapped if mapped is not None else (None, None, None)
        row["stereo_center_forward_m"] = forward
        row["stereo_center_right_m"] = right
        row["stereo_center_down_m"] = down
        row["warnings_json"] = _json_text(record.warnings or [], "warnings") if record.warnings is not None else None
        for key in ("stereo_center_optical_m", "warnings", "left_ray_optical", "right_ray_optical", "_mapped_stereo_center_frd"):
            row.pop(key, None)
        return row

    def _validate_normal_close_locked(self) -> None:
        assert self._index is not None
        pending_mapping = self._index.execute("SELECT COUNT(*) FROM pending_mappings").fetchone()[0]
        pending_source = self._index.execute("SELECT COUNT(*) FROM source_index WHERE video_mapping_status='pending'").fetchone()[0]
        writer_failure = self._index.execute("SELECT COUNT(*) FROM source_index WHERE video_mapping_status='writer_failure'").fetchone()[0]
        if pending_mapping:
            raise SessionRecordingError(f"unresolved video mappings: {pending_mapping}")
        if pending_source:
            raise SessionRecordingError(f"source records without video mapping: {pending_source}")
        if writer_failure:
            raise SessionRecordingError(f"writer_failure mappings: {writer_failure}")
        if self._mapping_missing_count("left") or self._mapping_missing_count("right"):
            raise SessionRecordingError("mapping_missing_count must be zero for complete session")
        for side in _CAMERA_SIDES:
            artifact = self._video_artifacts.get(side)
            if artifact is None or artifact.writer_status != "complete":
                raise SessionRecordingError(f"missing complete video artifact registration: {side}")

    def _validate_complete_artifacts_locked(self) -> None:
        for path in self.paths.all_artifacts:
            if not path.is_file() or path.stat().st_size <= 0:
                raise SessionRecordingError(f"canonical artifact is missing or empty: {path}")
        for table_name in self._TABLES:
            parquet_file = pq.ParquetFile(self._path_for_table(table_name))
            expected = self._schemas[table_name]
            actual_schema = parquet_file.schema_arrow
            if actual_schema.names != expected.names or any(actual_schema.field(name).type != expected.field(name).type for name in expected.names):
                raise SessionRecordingError(f"Parquet schema mismatch: {table_name}")
            if parquet_file.metadata is None or parquet_file.metadata.num_rows != self._counts[table_name]:
                raise SessionRecordingError(f"Parquet row count mismatch: {table_name}")
        from .session_video import probe_mkv

        if self._index is None:
            raise SessionRecordingError("integrity index is unavailable during complete validation")
        for side in _CAMERA_SIDES:
            path = self.paths.video_path(side)
            probe = probe_mkv(path, include_packets=True)
            if probe.decoded_frame_count is None:
                raise SessionRecordingError(f"decoded frame count is unavailable for authoritative MKV artifact: {path}")
            if "matroska" not in probe.format_name.split(",") or probe.codec_name != "h264" or probe.decoded_frame_count < 1:
                raise SessionRecordingError(f"invalid authoritative MKV artifact: {path}")
            cursor = self._index.execute(
                "SELECT video_frame_index FROM source_index WHERE camera_side=? AND stored_in_video=1 ORDER BY video_frame_index",
                (side,),
            )
            expected_index = 0
            for (frame_index,) in cursor:
                if frame_index != expected_index:
                    raise SessionRecordingError(f"stored video mapping has a gap or duplicate for {side}")
                expected_index += 1
            if expected_index != probe.decoded_frame_count:
                raise SessionRecordingError(f"stored video mapping does not cover actual {side} MKV frames")
        actual_files = {
            path.relative_to(self.paths.session_dir).as_posix()
            for path in self.paths.session_dir.rglob("*")
            if path.is_file() and path != self.paths.integrity_index
        }
        if actual_files != set(self.paths.manifest):
            raise SessionRecordingError(f"canonical manifest mismatch: {sorted(actual_files ^ set(self.paths.manifest))}")

    def _mapping_missing_count(self, camera_side: str) -> int:
        if self._index is None:
            return 0
        source_count = int(self._index.execute(
            "SELECT COUNT(*) FROM source_index WHERE camera_side=? AND video_mapping_status='pending'",
            (camera_side,),
        ).fetchone()[0])
        orphan_count = int(self._index.execute(
            "SELECT COUNT(*) FROM pending_mappings WHERE camera_side=?",
            (camera_side,),
        ).fetchone()[0])
        return source_count + orphan_count

    def _write_metadata_locked(self, *, state_override: str | None = None) -> None:
        state = state_override or self._state
        camera_manifest: dict[str, dict[str, Any]] = {}
        for side in _CAMERA_SIDES:
            artifact = self._video_artifacts.get(side)
            camera_manifest[side] = {
                "video_path": f"camera/{side}_video.mkv",
                "writer_status": artifact.writer_status if artifact else "not_started",
                "frame_count": artifact.frame_count if artifact else 0,
                "decoded_frame_count": artifact.frame_count if artifact else None,
                "packet_count": artifact.packet_count if artifact else None,
                "mapping_missing_count": self._mapping_missing_count(side),
            }
        payload: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "session_id": self.config.session_id,
            "session_dir_name": self.paths.session_dir.name,
            "application": self.config.application,
            "recording_scope": self.config.recording_scope,
            "session_state": state,
            "clean_shutdown": state == "complete",
            "incomplete": state != "complete",
            "started_monotonic_ns": self.config.started_monotonic_ns,
            "started_utc_ns": self.config.started_utc_ns,
            "ended_monotonic_ns": self._ended_monotonic_ns,
            "ended_utc_ns": self._ended_utc_ns,
            "failure_stage": self._failure_stage,
            "failure_message": self._failure_message,
            "counts": dict(self._counts),
            "camera": camera_manifest,
            "coordinate_contract": {
                "world_frame": "ned",
                "body_frame": "px4_frd",
                "camera_frame": "opencv_optical",
                "measurement_frame": "stereo_center_opencv_optical",
                "measurement_axis_contract": "x/right,y/down,z/forward",
                "output_frame": "stereo_center_frd",
                "output_axis_contract": "forward,right,down",
                "optical_to_frd_axis_map": {"forward": "optical_z_forward", "right": "optical_x_right", "down": "optical_y_down"},
            },
            "quaternion_order": ["w", "x", "y", "z"],
            "time_domains": {
                "pc_receive_monotonic_ns": "pc_monotonic",
                "pc_receive_utc_ns": "pc_utc",
                "px4_timestamp_us": "px4_boot",
                "px4_time_utc_usec": "px4_utc",
                "camera_pts_ticks": "camera_media",
                "camera_dts_ticks": "camera_media",
            },
            "triangulation_provenance": {
                "stereo_range_m": "RayTriangulationResult.distance_m",
                "ray_gap_m": "RayTriangulationResult.closest_gap_m",
                "input_frame": "stereo_center_opencv_optical RDF (right,down,forward)",
                "stored_frame": "stereo_center_frd (forward,right,down)",
            },
            "file_manifest": list(self.paths.manifest),
        }
        _atomic_json_write(self.paths.metadata, payload)

    def _close_result_locked(self) -> SessionCloseResult:
        return SessionCloseResult(
            paths=self.paths,
            session_state=self._state,
            clean_shutdown=self._state == "complete" and self._metadata_finalized,
            incomplete=not (self._state == "complete" and self._metadata_finalized),
            counts=dict(self._counts),
        )

    # ---- row validators and conversions ---------------------------------

    def _time_alignment_row(self, record: TimeAlignmentRecord) -> dict[str, Any]:
        return dataclasses.asdict(record)


def _atomic_json_write(path: Path, payload: Mapping[str, Any]) -> None:
    """Write metadata atomically and flush the file before promotion."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    try:
        encoded = (json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
        with temporary.open("wb") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        try:
            directory_fd = os.open(path.parent, os.O_DIRECTORY)
        except OSError:
            directory_fd = None
        if directory_fd is not None:
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    except Exception:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise
