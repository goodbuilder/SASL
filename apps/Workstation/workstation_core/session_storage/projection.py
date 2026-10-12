"""Session projection responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

from collections.abc import Iterator, Sequence
from dataclasses import field
from typing import Iterable
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.runtime_camera_status import is_sr_pc_diagnostic_reason
from . import schema as _storage_schema
from . import values as _storage_values


def _ordered_fieldnames(
    rows: Iterable[dict[str, object]],
    required_columns: Iterable[str] = _storage_schema.PAIR_AXIS_COLUMNS,
) -> list[str]:
    observed = dict.fromkeys(required_columns)
    for row in rows:
        observed.update(dict.fromkeys(row))
    return list(observed)


def _normalized_pair_rows(pair_rows: Iterable[dict[str, object]]) -> list[dict[str, object]]:
    normalized: list[dict[str, object]] = []
    for index, row in enumerate(pair_rows):
        normalized.append(
            {
                **{key: _storage_values._serializable_value(value) for key, value in row.items()},
                "sample_index": _storage_values._serializable_value(row.get("sample_index", index)),
                "session_elapsed_s": _storage_values._serializable_value(row.get("session_elapsed_s")),
            }
        )
    return normalized


def _normalized_generic_rows(rows: Iterable[dict[str, object]]) -> list[dict[str, object]]:
    return [
        {key: _storage_values._serializable_value(value) for key, value in row.items()}
        for row in rows
    ]


def _materialize_method_evaluation_rows(
    rows: Iterable[dict[str, object]],
) -> tuple[list[dict[str, object]], dict[str, list[dict[str, object]]]]:
    """Validate atomic bundle rows and split them into eight method streams."""

    from Workstation.workstation_core.method_evaluation import MethodEvaluationBundle

    bundles: list[dict[str, object]] = []
    method_rows = {method_id: [] for method_id in MEASUREMENT_METHOD_IDS}
    seen: set[tuple[str, int]] = set()
    last_sequence_by_measurement: dict[str, int] = {}
    for raw_row in rows:
        normalized = {
            key: _storage_values._serializable_value(value) for key, value in raw_row.items()
        }
        bundle = MethodEvaluationBundle.from_mapping(normalized)
        identity = (bundle.pair_id, bundle.request_generation)
        if identity in seen:
            raise ValueError(f"duplicate method evaluation bundle: {bundle.pair_id}")
        seen.add(identity)
        previous_sequence = last_sequence_by_measurement.get(bundle.measurement_id, -1)
        if bundle.pair_sequence <= previous_sequence:
            raise ValueError(
                "method evaluation bundles must have strictly increasing pair_sequence"
            )
        last_sequence_by_measurement[bundle.measurement_id] = bundle.pair_sequence
        bundle_record = bundle.to_record()
        bundles.append(bundle_record)
        bundle_sha256 = str(bundle_record["method_evaluation_bundle_sha256"])
        primary = bundle.primary
        primary_sha256 = "" if primary is None else primary.canonical_sha256()
        for outcome in bundle.outcomes:
            method_rows[outcome.method_id].append(
                {
                    **outcome.to_record(),
                    "method_evaluation_bundle_sha256": bundle_sha256,
                    "primary_method_id": "" if primary is None else primary.method_id,
                    "primary_outcome_sha256": primary_sha256,
                }
            )
    return bundles, method_rows


def _iter_normalized_generic_rows(
    rows: Iterable[dict[str, object]],
) -> Iterator[dict[str, object]]:
    for row in rows:
        yield {key: _storage_values._serializable_value(value) for key, value in row.items()}


def _copy_first_present(
    row: dict[str, object],
    destination: str,
    *sources: str,
) -> None:
    if row.get(destination) not in (None, ""):
        return
    for source in sources:
        value = row.get(source)
        if value not in (None, ""):
            row[destination] = value
            return


def _project_pair_typed_fields(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    for row in rows:
        _copy_first_present(row, "recording_time_ns", "session_elapsed_ns")
        _copy_first_present(row, "estimated_utc_time_ns", "recorded_unix_ns")
        _copy_first_present(row, "device_utc_time_ns", "pair_mid_utc_ns")
        _copy_first_present(
            row,
            "sync_delta_ns",
            "selected_pair_delta_ns",
            "app_receive_pair_delta_ns",
            "rtcp_pair_delta_ns",
        )
    return rows


def _project_camera_typed_fields(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    for row in rows:
        if row.get("recording_time_ns") in (None, ""):
            value = _storage_values._seconds_to_ns(row.get("session_elapsed_s"))
            if value is not None:
                row["recording_time_ns"] = value
        _copy_first_present(row, "estimated_utc_time_ns", "session_utc_ns")
        _copy_first_present(row, "device_utc_time_ns", "source_utc_ns")
        for destination, source in (
            ("rtcp_sr_age_ns", "rtcp_sr_age_s"),
            ("rtcp_sr_pc_offset_ns", "rtcp_sr_pc_offset_s"),
            ("rtcp_receive_delay_ns", "rtcp_frame_receive_delay_s"),
            ("rtcp_transport_delay_ns", "rtcp_frame_transport_delay_s"),
        ):
            if row.get(destination) in (None, ""):
                value = _storage_values._seconds_to_ns(row.get(source))
                if value is not None:
                    row[destination] = value
    return rows


def _project_px4_typed_fields(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    for row in rows:
        if row.get("recording_time_ns") in (None, ""):
            value = _storage_values._seconds_to_ns(row.get("session_elapsed_s"))
            if value is not None:
                row["recording_time_ns"] = value
        _copy_first_present(row, "session_utc_time_ns", "session_utc_ns")
        _copy_first_present(row, "gps_utc_time_ns", "source_utc_ns")
        _copy_first_present(row, "gps_utc_source", "source_utc_method")
    return rows


def _iter_project_camera_typed_fields(
    rows: Iterable[dict[str, object]],
) -> Iterator[dict[str, object]]:
    for row in rows:
        yield _project_camera_typed_fields([row])[0]


def _iter_project_pair_typed_fields(
    rows: Iterable[dict[str, object]],
) -> Iterator[dict[str, object]]:
    for row in rows:
        yield _project_pair_typed_fields([row])[0]


def _iter_project_px4_typed_fields(
    rows: Iterable[dict[str, object]],
) -> Iterator[dict[str, object]]:
    for row in rows:
        yield _project_px4_typed_fields([row])[0]


def _recording_time_ns(row: dict[str, object], metadata: dict[str, object]) -> int | None:
    direct = _storage_values._optional_int(_storage_values._first_present(row, "recording_time_ns", "session_elapsed_ns"))
    if direct is not None:
        return direct
    from_seconds = _storage_values._ns_from_seconds(
        _storage_values._first_present(row, "session_elapsed_s", "recording_elapsed_s")
    )
    if from_seconds is not None:
        return from_seconds
    sample_monotonic_ns = _storage_values._optional_int(
        _storage_values._first_present(row, "sample_monotonic_ns", "pair_app_mid_monotonic_ns")
    )
    started_monotonic_ns = _storage_values._optional_int(metadata.get("session_started_monotonic_ns"))
    if sample_monotonic_ns is not None and started_monotonic_ns is not None:
        return max(0, sample_monotonic_ns - started_monotonic_ns)
    return None


def _estimated_utc_time_ns(
    row: dict[str, object],
    metadata: dict[str, object],
    recording_time_ns: int | None,
) -> int | None:
    direct = _storage_values._optional_int(
        _storage_values._first_present(row, "estimated_utc_time_ns", "session_utc_ns")
    )
    if direct is not None:
        return direct
    started_unix_ns = _storage_values._optional_int(metadata.get("session_started_unix_ns"))
    if started_unix_ns is not None and recording_time_ns is not None:
        return started_unix_ns + recording_time_ns
    return None


def _distance_device_utc_time_ns(row: dict[str, object]) -> int | None:
    return _storage_values._optional_int(
        _storage_values._first_present(
            row,
            "device_utc_time_ns",
            "pair_mid_utc_ns",
            "bbox_mid_utc_ns",
        )
    )


def _timeline_device_utc_time_ns(row: dict[str, object]) -> int | None:
    direct = _storage_values._optional_int(_storage_values._first_present(row, "device_utc_time_ns", "source_utc_ns"))
    if direct is not None:
        return direct
    gps_utc_us = _storage_values._optional_int(row.get("gps_utc_us"))
    if gps_utc_us is not None:
        return gps_utc_us * 1000
    return None


def _paired_class_name(row: dict[str, object]) -> str:
    left = str(_storage_values._first_present(row, "left_bbox_class_name", "left_class_name") or "")
    right = str(_storage_values._first_present(row, "right_bbox_class_name", "right_class_name") or "")
    if left and right:
        return left if left == right else f"{left}/{right}"
    return left or right


def _warning_code(row: dict[str, object]) -> str:
    text = str(_storage_values._first_present(row, "warnings", "measurement_attempt_warnings") or "")
    return text.split("; ", 1)[0].strip()


def _compact_distance_measurement_row(
    row: dict[str, object],
    metadata: dict[str, object],
) -> dict[str, object]:
    recording_time_ns = _recording_time_ns(row, metadata)
    selected_pair_delta_ns = _storage_values._optional_int(
        _storage_values._first_present(
            row,
            "selected_pair_delta_ns",
            "app_receive_pair_delta_ns",
            "sync_delta_ns",
            "rtcp_pair_delta_ns",
        )
    )
    sync_delta_ns = selected_pair_delta_ns
    if sync_delta_ns is None:
        sync_delta_ns = _storage_values._ns_from_seconds(_storage_values._first_present(row, "camera_delta_s", "rtcp_pair_delta_s"))
    compact = {
        "sample_index": _storage_values._optional_int(row.get("sample_index")),
        "recording_time_ns": recording_time_ns,
        "app_receive_monotonic_ns": _storage_values._optional_int(row.get("app_receive_monotonic_ns")),
        "app_receive_timestamp_role": str(row.get("app_receive_timestamp_role") or ""),
        "estimated_utc_time_ns": _estimated_utc_time_ns(row, metadata, recording_time_ns),
        "device_utc_time_ns": _distance_device_utc_time_ns(row),
        "pair_id": str(
            _storage_values._first_present(
                row,
                "measurement_attempt_id",
                "measurement_attempt_session_id",
                "measurement_detection_pair_key",
                "pair_sequence",
                "sample_index",
            )
            or ""
        ),
        "measurement_valid": _storage_values._optional_bool(row.get("measurement_valid")),
        "pair_status": str(row.get("pair_status") or ""),
        "class_name": _paired_class_name(row),
        "left_bbox_center_x_px": _storage_values._optional_float(_storage_values._first_present(row, "left_bbox_center_x_px", "left_cx")),
        "left_bbox_center_y_px": _storage_values._optional_float(_storage_values._first_present(row, "left_bbox_center_y_px", "left_cy")),
        "right_bbox_center_x_px": _storage_values._optional_float(_storage_values._first_present(row, "right_bbox_center_x_px", "right_cx")),
        "right_bbox_center_y_px": _storage_values._optional_float(_storage_values._first_present(row, "right_bbox_center_y_px", "right_cy")),
        "left_confidence": _storage_values._optional_float(_storage_values._first_present(row, "left_bbox_confidence", "left_confidence")),
        "right_confidence": _storage_values._optional_float(_storage_values._first_present(row, "right_bbox_confidence", "right_confidence")),
        "distance_m": _storage_values._optional_float(row.get("distance_m")),
        "baseline_center_forward_m": _storage_values._optional_float(row.get("baseline_center_forward_m")),
        "baseline_center_right_m": _storage_values._optional_float(row.get("baseline_center_right_m")),
        "baseline_center_down_m": _storage_values._optional_float(row.get("baseline_center_down_m")),
        **{name: _storage_values._optional_float(row.get(name)) for name in (
            "target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m",
        )},
        "baseline_m": _storage_values._optional_float(_storage_values._first_present(row, "baseline_m", "baseline_length_m")),
        "pairing_time_basis_requested": str(
            row.get("pairing_time_basis_requested")
            or row.get("pairing_time_basis")
            or ""
        ),
        "pairing_time_basis": str(row.get("pairing_time_basis") or ""),
        "pairing_time_basis_role": str(row.get("pairing_time_basis_role") or ""),
        "pairing_fallback_active": _storage_values._optional_bool(
            row.get("pairing_fallback_active")
        ),
        "pairing_fallback_reason": str(
            row.get("pairing_fallback_reason") or ""
        ),
        "selected_pair_delta_ns": selected_pair_delta_ns,
        "selected_pair_abs_delta_ns": _storage_values._optional_int(row.get("selected_pair_abs_delta_ns")),
        "selected_pair_midpoint_ns": _storage_values._optional_int(
            row.get("selected_pair_midpoint_ns")
        ),
        "app_receive_pair_delta_ns": _storage_values._optional_int(row.get("app_receive_pair_delta_ns")),
        "app_receive_evidence_role": str(
            row.get("app_receive_evidence_role") or ""
        ),
        "rtcp_pair_delta_ns": _storage_values._optional_int(row.get("rtcp_pair_delta_ns")),
        "ntp_evidence_role": str(row.get("ntp_evidence_role") or ""),
        "left_media_recovery_basis": str(
            row.get("left_media_recovery_basis") or ""
        ),
        "right_media_recovery_basis": str(
            row.get("right_media_recovery_basis") or ""
        ),
        "left_media_integrity_mode": str(
            row.get("left_media_integrity_mode") or ""
        ),
        "right_media_integrity_mode": str(
            row.get("right_media_integrity_mode") or ""
        ),
        "left_media_random_access_guaranteed": _storage_values._optional_bool(
            row.get("left_media_random_access_guaranteed")
        ),
        "right_media_random_access_guaranteed": _storage_values._optional_bool(
            row.get("right_media_random_access_guaranteed")
        ),
        "sync_delta_ns": sync_delta_ns,
        "quality": str(_storage_values._first_present(row, "quality", "pair_quality_status") or ""),
        "warning_code": _warning_code(row),
    }
    return {column: compact.get(column) for column in _storage_schema.DISTANCE_MEASUREMENT_COLUMNS}


def compact_distance_measurement_rows(
    rows: Iterable[dict[str, object]],
    metadata: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    metadata = metadata or {}
    return [_compact_distance_measurement_row(row, metadata) for row in rows]


def _compact_camera_timeline_row(
    row: dict[str, object],
    metadata: dict[str, object],
) -> dict[str, object]:
    recording_time_ns = _recording_time_ns(row, metadata)
    receive_delay_ns = _storage_values._ns_from_seconds(row.get("rtcp_frame_receive_delay_s"))
    invalid_reason = str(row.get("rtcp_timing_invalid_reason") or "")
    timing_status = (
        "valid_sr_pc_diagnostic"
        if is_sr_pc_diagnostic_reason(invalid_reason)
        else invalid_reason or "valid"
    )
    compact = {
        "sequence": _storage_values._optional_int(row.get("sequence")),
        "recording_time_ns": recording_time_ns,
        "estimated_utc_time_ns": _estimated_utc_time_ns(row, metadata, recording_time_ns),
        "device_utc_time_ns": _timeline_device_utc_time_ns(row),
        "image_width_px": _storage_values._optional_int(row.get("image_width_px")),
        "image_height_px": _storage_values._optional_int(row.get("image_height_px")),
        "timestamp_source": str(row.get("timestamp_source") or ""),
        "rtp_timestamp": _storage_values._optional_int(row.get("rtp_timestamp")),
        "rtp_ssrc": _storage_values._optional_int(row.get("rtp_ssrc")),
        "rtp_marker_seq": _storage_values._optional_int(row.get("rtp_marker_seq")),
        "rtcp_sr_received_count": _storage_values._optional_int(row.get("rtcp_sr_received_count")),
        "rtcp_sr_rtp_timestamp": _storage_values._optional_int(row.get("rtcp_sr_rtp_timestamp")),
        "rtcp_clock_rate_hz": _storage_values._optional_float(row.get("rtcp_clock_rate_hz")),
        "rtcp_sr_age_ns": _storage_values._ns_from_seconds(row.get("rtcp_sr_age_s")),
        "rtcp_sr_pc_offset_ns": _storage_values._ns_from_seconds(row.get("rtcp_sr_pc_offset_s")),
        "rtcp_receive_delay_ns": receive_delay_ns,
        "rtcp_transport_delay_ns": _storage_values._ns_from_seconds(row.get("rtcp_frame_transport_delay_s")),
        "rtcp_camera_ntp_rate": _storage_values._optional_float(row.get("rtcp_camera_ntp_rate")),
        "rtcp_camera_media_rate": _storage_values._optional_float(row.get("rtcp_camera_media_rate")),
        "rtsp_transport": str(row.get("rtsp_transport") or ""),
        "ntp_evidence_role": str(row.get("ntp_evidence_role") or ""),
        "media_recovery_basis": str(row.get("media_recovery_basis") or ""),
        "media_integrity_mode": str(row.get("media_integrity_mode") or ""),
        "media_random_access_guaranteed": _storage_values._optional_bool(
            row.get("media_random_access_guaranteed")
        ),
        "timing_status": timing_status,
    }
    return {column: compact.get(column) for column in _storage_schema.CAMERA_TIMELINE_COMPACT_COLUMNS}


def compact_camera_timeline_rows(
    rows: Iterable[dict[str, object]],
    metadata: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    metadata = metadata or {}
    return [_compact_camera_timeline_row(row, metadata) for row in rows]


def _compact_px4_timeline_row(
    row: dict[str, object],
    metadata: dict[str, object],
) -> dict[str, object]:
    recording_time_ns = _recording_time_ns(row, metadata)
    compact = {
        "sequence": _storage_values._optional_int(row.get("sequence")),
        "recording_time_ns": recording_time_ns,
        "session_utc_time_ns": _estimated_utc_time_ns(row, metadata, recording_time_ns),
        "gps_utc_time_ns": _gps_utc_time_ns(row),
        "gps_utc_source": _gps_utc_source(row),
        "sample_type": str(row.get("sample_type") or ""),
        "topic": str(row.get("topic") or ""),
        "attitude_rollspeed_rad_s": _storage_values._optional_float(row.get("attitude_rollspeed_rad_s")),
        "attitude_pitchspeed_rad_s": _storage_values._optional_float(row.get("attitude_pitchspeed_rad_s")),
        "attitude_yawspeed_rad_s": _storage_values._optional_float(row.get("attitude_yawspeed_rad_s")),
        "attitude_time_boot_ms": _storage_values._optional_int(row.get("attitude_time_boot_ms")),
        "roll_deg": _storage_values._optional_float(row.get("roll_deg")),
        "pitch_deg": _storage_values._optional_float(row.get("pitch_deg")),
        "latitude_deg": _storage_values._optional_float(_storage_values._first_present(row, "latitude_deg", "lat_deg")),
        "longitude_deg": _storage_values._optional_float(_storage_values._first_present(row, "longitude_deg", "lon_deg")),
        "altitude_m": _storage_values._optional_float(_storage_values._first_present(row, "altitude_m", "alt_m")),
        "altitude_ellipsoid_m": _storage_values._optional_float(row.get("altitude_ellipsoid_m")),
        "fix_type": _storage_values._optional_int(row.get("fix_type")),
        "satellites_used": _storage_values._optional_int(_storage_values._first_present(row, "satellites_used", "satellites")),
        "eph_m": _storage_values._optional_float(row.get("eph_m")),
        "epv_m": _storage_values._optional_float(row.get("epv_m")),
        "rtk_label": str(row.get("rtk_label") or ""),
        "heading_deg": _storage_values._optional_float(row.get("heading_deg")),
        "ground_speed_m_s": _storage_values._optional_float(row.get("ground_speed_m_s")),
        "yaw_deg": _storage_values._optional_float(row.get("gps_yaw_deg", row.get("yaw_deg"))),
        "heading_accuracy_deg": _storage_values._optional_float(row.get("heading_accuracy_deg")),
        "velocity_n_m_s": _storage_values._optional_float(row.get("velocity_n_m_s")),
        "velocity_e_m_s": _storage_values._optional_float(row.get("velocity_e_m_s")),
        "velocity_d_m_s": _storage_values._optional_float(row.get("velocity_d_m_s")),
        "velocity_ned_valid": _storage_values._optional_bool(row.get("velocity_ned_valid")),
        "mag_sensor_slot": str(_storage_values._first_present(row, "mag_sensor_slot", "sensor_slot") or ""),
        "mag_slot": _storage_values._optional_int(row.get("mag_slot")),
        "mag_cal_slot": str(row.get("mag_cal_slot") or ""),
        "mag_value_units": str(row.get("mag_value_units") or ""),
        "mag_x": _storage_values._optional_float(row.get("mag_x")),
        "mag_y": _storage_values._optional_float(row.get("mag_y")),
        "mag_z": _storage_values._optional_float(row.get("mag_z")),
        "mag_vector_norm": _storage_values._optional_float(
            _storage_values._first_present(row, "mag_vector_norm", "mag_magnitude")
        ),
        "mag_temperature_c": _storage_values._optional_float(row.get("mag_temperature_c")),
        "mag_device_id": _storage_values._optional_int(_storage_values._first_present(row, "mag_device_id", "device_id")),
        "mag_node_id": _storage_values._optional_int(_storage_values._first_present(row, "mag_node_id", "node_id")),
        "mag_device_type": str(row.get("mag_device_type") or ""),
        "mag_interface": str(row.get("mag_interface") or ""),
        "mag_is_rm3100": _storage_values._optional_bool(row.get("mag_is_rm3100")),
        "mag_priority": _storage_values._optional_int(row.get("mag_priority")),
        "mag_rotation": _storage_values._optional_int(row.get("mag_rotation")),
        "mag_enabled": _storage_values._optional_bool(row.get("mag_enabled")),
        "mag_expected_selected": _storage_values._optional_bool(row.get("mag_expected_selected")),
        "mag_actual_selected": _storage_values._optional_bool(row.get("mag_actual_selected")),
        "mag_expected_selected_device_id": _storage_values._optional_int(row.get("mag_expected_selected_device_id")),
        "mag_actual_selected_device_id": _storage_values._optional_int(row.get("mag_actual_selected_device_id")),
        "mag_is_actual_rm3100": _storage_values._optional_bool(row.get("mag_is_actual_rm3100")),
        "mag_armed": _storage_values._optional_bool(row.get("mag_armed")),
        "mag_status": str(row.get("mag_status") or ""),
        "mag_warning_text": str(row.get("mag_warning_text") or ""),
        "mag_rm3100_primary_id": _storage_values._optional_int(row.get("mag_rm3100_primary_id")),
        "mag_rm3100_backup_id": _storage_values._optional_int(row.get("mag_rm3100_backup_id")),
        "mag_param_uavcan_enable": _storage_values._optional_int(row.get("mag_param_uavcan_enable")),
        "mag_param_uavcan_sub_mag": _storage_values._optional_int(row.get("mag_param_uavcan_sub_mag")),
        "mag_param_ekf2_mag_type": _storage_values._optional_int(row.get("mag_param_ekf2_mag_type")),
        "mag_param_snapshot_json": str(row.get("mag_param_snapshot_json") or ""),
    }
    for stream_name in ("mag_scaled_imu", "mag_scaled_imu2", "mag_scaled_imu3"):
        for suffix in (
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "time_boot_ms",
        ):
            compact[f"{stream_name}_{suffix}"] = row.get(f"{stream_name}_{suffix}")
    for field in (
        "mavlink_message_name",
        "mavlink_time_boot_ms",
        "mavlink_time_usec",
        "mavlink_global_latitude_deg",
        "mavlink_global_longitude_deg",
        "mavlink_global_altitude_m",
        "mavlink_global_relative_altitude_m",
        "mavlink_global_velocity_n_m_s",
        "mavlink_global_velocity_e_m_s",
        "mavlink_global_velocity_d_m_s",
        "mavlink_global_heading_deg",
        "mavlink_local_x_m",
        "mavlink_local_y_m",
        "mavlink_local_z_m",
        "mavlink_local_velocity_x_m_s",
        "mavlink_local_velocity_y_m_s",
        "mavlink_local_velocity_z_m_s",
        "mavlink_vfr_airspeed_m_s",
        "mavlink_vfr_groundspeed_m_s",
        "mavlink_vfr_heading_deg",
        "mavlink_vfr_throttle_pct",
        "mavlink_vfr_altitude_m",
        "mavlink_vfr_climb_m_s",
        "mavlink_sys_voltage_battery_v",
        "mavlink_sys_current_battery_a",
        "mavlink_sys_battery_remaining_pct",
        "mavlink_sys_drop_rate_comm_pct",
        "mavlink_sys_errors_comm",
        "mavlink_battery_current_a",
        "mavlink_battery_remaining_pct",
        "mavlink_battery_temperature_c",
        "mavlink_battery_voltage_0_v",
        "mavlink_battery_voltage_1_v",
        "mavlink_battery_voltage_2_v",
        "mavlink_raw_accel_x",
        "mavlink_raw_accel_y",
        "mavlink_raw_accel_z",
        "mavlink_raw_gyro_x",
        "mavlink_raw_gyro_y",
        "mavlink_raw_gyro_z",
        "mavlink_raw_mag_x",
        "mavlink_raw_mag_y",
        "mavlink_raw_mag_z",
    ):
        compact[field] = row.get(field)
    gps_prefix = _gps_wide_prefix(compact["topic"], compact["sample_type"])
    if gps_prefix is not None:
        for field in _storage_schema.PX4_GPS_WIDE_FIELDS:
            compact[f"{gps_prefix}_{field}"] = compact[field]
    return {column: compact.get(column) for column in _storage_schema.PX4_TIMELINE_COMPACT_COLUMNS}


def _gps_utc_time_ns(row: dict[str, object]) -> int | None:
    gps_utc_us = _storage_values._optional_int(row.get("gps_utc_us"))
    return gps_utc_us * 1000 if gps_utc_us is not None else None


def _gps_utc_source(row: dict[str, object]) -> str:
    topic = str(row.get("topic") or "").lower()
    if any(token in topic for token in ("gps2", "sensor_gps_2", "vehicle_gps_position_2")):
        return "GPS2"
    if _gps_utc_time_ns(row) is not None and str(row.get("sample_type") or "").lower().startswith(("gps", "gnss")):
        return "GPS1"
    return ""


def _gps_wide_prefix(topic: object, sample_type: object) -> str | None:
    """Return the explicit GPS1/GPS2 column family for a timeline row."""
    topic_text = str(topic or "").lower()
    sample_text = str(sample_type or "").lower()
    if not any(token in sample_text for token in ("gps", "gnss")):
        return None
    if any(token in topic_text for token in ("gps2", "sensor_gps_2", "vehicle_gps_position_2")):
        return "gps2"
    return "gps1"


def compact_px4_timeline_rows(
    rows: Iterable[dict[str, object]],
    metadata: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    metadata = metadata or {}
    compact_rows = [_compact_px4_timeline_row(row, metadata) for row in rows]
    return _wide_mavlink_mag_rows(compact_rows)


def _compact_gnss_base_row(row: dict[str, object]) -> dict[str, object]:
    # Unlike PX4 rows, GNSS Base rows are already built from
    # ``_raw_timeline_row`` with every DEVICE_TIMELINE_COLUMNS field
    # (including session_utc_ns/host_receive_utc_ns) resolved at record
    # time, so no metadata-driven timestamp reconstruction is needed here.
    compact = {column: row.get(column) for column in _storage_schema.DEVICE_TIMELINE_COLUMNS}
    compact.update(
        {
            "latitude_deg": _storage_values._optional_float(row.get("latitude_deg")),
            "longitude_deg": _storage_values._optional_float(row.get("longitude_deg")),
            "ellipsoid_height_m": _storage_values._optional_float(row.get("ellipsoid_height_m")),
            "msl_altitude_m": _storage_values._optional_float(row.get("msl_altitude_m")),
            "height_type": str(row.get("height_type") or ""),
            "fixed": _storage_values._optional_bool(row.get("fixed")),
            "device_id": str(row.get("device_id") or ""),
            "gnss_operation_state": str(row.get("gnss_operation_state") or ""),
        }
    )
    return {column: compact.get(column) for column in _storage_schema.GNSS_BASE_COMPACT_COLUMNS}


def compact_gnss_base_rows(
    rows: Iterable[dict[str, object]],
    metadata: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    del metadata  # kept for signature parity with the other compact_*_rows() helpers
    return [_compact_gnss_base_row(row) for row in rows]


def _mavlink_mag_stream_name(topic: object) -> str | None:
    topic_text = str(topic or "")
    if topic_text in _storage_schema._PX4_MAVLINK_MAG_STREAMS:
        return _storage_schema._PX4_MAVLINK_MAG_STREAMS[topic_text]
    return _storage_schema._PX4_MAVLINK_MAG_STREAMS.get(
        f"mavlink/{topic_text.rsplit('/', 1)[-1]}"
    )


def _wide_mavlink_mag_rows(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    """Combine nearby SCALED_IMU* callbacks into fixed-column PX4 rows.

    MAVLink delivers the three streams independently, so their receive times
    are not required to be byte-for-byte equal.  A 25 ms window is narrow
    enough for the requested 20 Hz stream and avoids merging adjacent samples.
    Non-MAVLink rows and the existing generic ``mag_x/y/z`` fields are kept.
    """
    events: list[dict[str, object]] = []
    pending: list[dict[str, object]] = []

    for row in rows:
        stream_name = _mavlink_mag_stream_name(row.get("topic"))
        if stream_name is None or row.get("sample_type") != "magnetometer":
            events.append({"kind": "row", "row": row})
            continue

        timestamp = row.get("recording_time_ns")
        timestamp_int = timestamp if isinstance(timestamp, int) else None
        candidate = None
        if timestamp_int is not None:
            candidates = [
                group
                for group in pending
                if group["row"].get(f"{stream_name}_x") is None
                and isinstance(group["recording_time_ns"], int)
                and abs(timestamp_int - group["recording_time_ns"])
                <= _storage_schema._PX4_MAVLINK_MAG_MATCH_TOLERANCE_NS
            ]
            if candidates:
                candidate = min(
                    candidates,
                    key=lambda group: abs(timestamp_int - group["recording_time_ns"]),
                )

        if candidate is None:
            wide_row = dict(row)
            for column in _storage_schema.PX4_MAVLINK_MAG_WIDE_COLUMNS:
                wide_row[column] = None
            event = {"kind": "mag", "row": wide_row}
            events.append(event)
            candidate = {
                "row": wide_row,
                "recording_time_ns": timestamp_int,
                "event": event,
            }
            pending.append(candidate)

        wide_row = candidate["row"]
        wide_row[f"{stream_name}_x"] = row.get("mag_x")
        wide_row[f"{stream_name}_y"] = row.get("mag_y")
        wide_row[f"{stream_name}_z"] = row.get("mag_z")
        wide_row[f"{stream_name}_unit"] = row.get("mag_value_units")

        if all(
            wide_row[f"{name}_x"] is not None
            for name in _storage_schema._PX4_MAVLINK_MAG_STREAMS.values()
        ):
            pending.remove(candidate)

    return [event["row"] for event in events]
