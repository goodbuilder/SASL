"""Session schema responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS

from dataclasses import field


WRITE_PARQUET_ARTIFACTS = True


PAIR_AXIS_COLUMNS = ("sample_index", "session_elapsed_s")


DISTANCE_MEASUREMENT_COLUMNS = (
    "sample_index",
    "recording_time_ns",
    "app_receive_monotonic_ns",
    "app_receive_timestamp_role",
    "estimated_utc_time_ns",
    "device_utc_time_ns",
    "pair_id",
    "measurement_valid",
    "pair_status",
    "class_name",
    "left_bbox_center_x_px",
    "left_bbox_center_y_px",
    "right_bbox_center_x_px",
    "right_bbox_center_y_px",
    "left_confidence",
    "right_confidence",
    "distance_m",
    "baseline_center_forward_m",
    "baseline_center_right_m",
    "baseline_center_down_m",
    "target_latitude_deg", "target_longitude_deg", "target_altitude_ellipsoid_m", "target_altitude_msl_m",
    "baseline_m",
    "pairing_time_basis_requested",
    "pairing_time_basis",
    "pairing_time_basis_role",
    "pairing_fallback_active",
    "pairing_fallback_reason",
    "selected_pair_delta_ns",
    "selected_pair_abs_delta_ns",
    "selected_pair_midpoint_ns",
    "app_receive_pair_delta_ns",
    "app_receive_evidence_role",
    "rtcp_pair_delta_ns",
    "ntp_evidence_role",
    "left_media_recovery_basis",
    "right_media_recovery_basis",
    "left_media_integrity_mode",
    "right_media_integrity_mode",
    "left_media_random_access_guaranteed",
    "right_media_random_access_guaranteed",
    "sync_delta_ns",
    "quality",
    "warning_code",
)


CAMERA_TIMELINE_COMPACT_COLUMNS = (
    "sequence",
    "recording_time_ns",
    "estimated_utc_time_ns",
    "device_utc_time_ns",
    "image_width_px",
    "image_height_px",
    "timestamp_source",
    "rtp_timestamp",
    "rtp_ssrc",
    "rtp_marker_seq",
    "rtcp_sr_received_count",
    "rtcp_sr_rtp_timestamp",
    "rtcp_clock_rate_hz",
    "rtcp_sr_age_ns",
    "rtcp_sr_pc_offset_ns",
    "rtcp_receive_delay_ns",
    "rtcp_transport_delay_ns",
    "rtcp_camera_ntp_rate",
    "rtcp_camera_media_rate",
    "rtsp_transport",
    "ntp_evidence_role",
    "media_recovery_basis",
    "media_integrity_mode",
    "media_random_access_guaranteed",
    "timing_status",
)


PX4_TIMELINE_COMPACT_COLUMNS = (
    "sequence",
    "recording_time_ns",
    "session_utc_time_ns",
    "gps_utc_time_ns",
    "gps_utc_source",
    "sample_type",
    "topic",
    "attitude_rollspeed_rad_s",
    "attitude_pitchspeed_rad_s",
    "attitude_yawspeed_rad_s",
    "attitude_time_boot_ms",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "fix_type",
    "satellites_used",
    "eph_m",
    "epv_m",
    "gps1_latitude_deg",
    "gps1_longitude_deg",
    "gps1_altitude_m",
    "gps1_fix_type",
    "gps1_satellites_used",
    "gps1_eph_m",
    "gps1_epv_m",
    "gps1_heading_deg",
    "gps1_ground_speed_m_s",
    "gps1_yaw_deg",
    "gps1_heading_accuracy_deg",
    "gps2_latitude_deg",
    "gps2_longitude_deg",
    "gps2_altitude_m",
    "gps2_fix_type",
    "gps2_satellites_used",
    "gps2_eph_m",
    "gps2_epv_m",
    "gps2_heading_deg",
    "gps2_ground_speed_m_s",
    "gps2_yaw_deg",
    "gps2_heading_accuracy_deg",
    "heading_deg",
    "mag_sensor_slot",
    "mag_slot",
    "mag_cal_slot",
    "mag_value_units",
    "mag_x",
    "mag_y",
    "mag_z",
    "mag_vector_norm",
    "mag_temperature_c",
    "mag_device_id",
    "mag_node_id",
    "mag_priority",
    "mag_rotation",
    "mag_enabled",
    "mag_expected_selected",
    "mag_actual_selected",
    "mag_armed",
    "mag_status",
    "mag_warning_text",
    "mag_param_uavcan_enable",
    "mag_param_uavcan_sub_mag",
    "mag_param_ekf2_mag_type",
    "mag_param_snapshot_json",
    "mag_scaled_imu_x",
    "mag_scaled_imu_y",
    "mag_scaled_imu_z",
    "mag_scaled_imu_unit",
    "mag_scaled_imu2_x",
    "mag_scaled_imu2_y",
    "mag_scaled_imu2_z",
    "mag_scaled_imu2_unit",
    "mag_scaled_imu3_x",
    "mag_scaled_imu3_y",
    "mag_scaled_imu3_z",
    "mag_scaled_imu3_unit",
    "mag_scaled_imu_accel_x",
    "mag_scaled_imu_accel_y",
    "mag_scaled_imu_accel_z",
    "mag_scaled_imu_gyro_x",
    "mag_scaled_imu_gyro_y",
    "mag_scaled_imu_gyro_z",
    "mag_scaled_imu_time_boot_ms",
    "mag_scaled_imu2_accel_x",
    "mag_scaled_imu2_accel_y",
    "mag_scaled_imu2_accel_z",
    "mag_scaled_imu2_gyro_x",
    "mag_scaled_imu2_gyro_y",
    "mag_scaled_imu2_gyro_z",
    "mag_scaled_imu2_time_boot_ms",
    "mag_scaled_imu3_accel_x",
    "mag_scaled_imu3_accel_y",
    "mag_scaled_imu3_accel_z",
    "mag_scaled_imu3_gyro_x",
    "mag_scaled_imu3_gyro_y",
    "mag_scaled_imu3_gyro_z",
    "mag_scaled_imu3_time_boot_ms",
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
)


PX4_MAVLINK_MAG_WIDE_COLUMNS = (
    "mag_scaled_imu_x",
    "mag_scaled_imu_y",
    "mag_scaled_imu_z",
    "mag_scaled_imu_unit",
    "mag_scaled_imu2_x",
    "mag_scaled_imu2_y",
    "mag_scaled_imu2_z",
    "mag_scaled_imu2_unit",
    "mag_scaled_imu3_x",
    "mag_scaled_imu3_y",
    "mag_scaled_imu3_z",
    "mag_scaled_imu3_unit",
)


_PX4_MAVLINK_MAG_STREAMS = {
    "mavlink/SCALED_IMU": "mag_scaled_imu",
    "mavlink/SCALED_IMU2": "mag_scaled_imu2",
    "mavlink/SCALED_IMU3": "mag_scaled_imu3",
}


_PX4_MAVLINK_MAG_MATCH_TOLERANCE_NS = 25_000_000


PX4_GPS_WIDE_FIELDS = (
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "fix_type",
    "satellites_used",
    "eph_m",
    "epv_m",
    "heading_deg",
    "ground_speed_m_s",
    "yaw_deg",
    "heading_accuracy_deg",
)


DEVICE_TIMELINE_COLUMNS = (
    "sequence",
    "session_id",
    "side",
    "sample_type",
    "row_contract",
    "sample_monotonic_ns",
    "sample_monotonic_s",
    "session_elapsed_s",
    "session_utc_ns",
    "session_utc_s",
    "session_utc_method",
    "session_utc_anchor_uncertainty_ns",
    "source_utc_ns",
    "source_utc_s",
    "source_utc_valid",
    "source_utc_method",
    "host_receive_monotonic_ns",
    "host_receive_utc_ns",
)


GNSS_BASE_COMPACT_COLUMNS = DEVICE_TIMELINE_COLUMNS + (
    "latitude_deg",
    "longitude_deg",
    "ellipsoid_height_m",
    "msl_altitude_m",
    "height_type",
    "fixed",
    "device_id",
    "gnss_operation_state",
)


EXTRINSICS_COMPARISON_COMPACT_COLUMNS = (
    "pair_sequence",
    "timestamp_s",
    "method",
    "forward_m",
    "right_m",
    "down_m",
    "range_m",
    "status",
)


DYNAMIC_SHEET_RULES: dict[str, tuple[str, ...]] = {
    "Measurements": (
        "measurement",
        "distance",
        "range_m",
        "baseline_center",
        "source_left_camera_rdf",
        "object",
        "class",
        "track",
        "frame",
        "utc",
        "timestamp",
    ),
    "GPS": ("gps", "latitude", "longitude", "lat", "lon", "altitude", "alt"),
    "Attitude": (
        "attitude",
        "roll",
        "pitch",
        "yaw",
        "quaternion",
        "heading",
        "px4_timestamp",
    ),
    "Baseline": ("baseline",),
    "Quality": (
        "quality",
        "confidence",
        "valid",
        "status",
        "error",
        "delta",
        "sync",
        "matched",
        "interpolation",
    ),
}


CALIBRATION_KEYWORDS = (
    "calibration",
    "intrinsic",
    "extrinsic",
    "distortion",
    "camera_matrix",
    "projection_matrix",
    "rectification",
)


AUTHORITATIVE_JSONL_SCHEMA_VERSION = "stereowiris-authoritative-jsonl-v1"


MEASUREMENT_STORAGE_SCHEMA = "workstation-measurement-storage-v4"


MEASUREMENT_STORAGE_SCHEMA_VERSION = 4


AUTHORITATIVE_JSONL_STREAMS = {
    "raw_stereo_measurements": (
        "raw/stereo_measurements",
        "data/raw/stereo_measurements.jsonl",
    ),
    "raw_camera_left": ("raw/camera_left", "data/raw/camera_left.jsonl"),
    "raw_camera_right": ("raw/camera_right", "data/raw/camera_right.jsonl"),
    "raw_px4_left": ("raw/px4_left", "data/raw/px4_left.jsonl"),
    "raw_px4_right": ("raw/px4_right", "data/raw/px4_right.jsonl"),
    "raw_gnss_base": ("raw/gnss_base", "data/raw/gnss_base.jsonl"),
    "geometry_contexts": (
        "context/geometry_contexts",
        "data/context/geometry_contexts.jsonl",
    ),
    "time_sync_epochs": (
        "context/time_sync_epochs",
        "data/context/time_sync_epochs.jsonl",
    ),
    "raw_px4_drone": ("raw/px4_drone", "data/raw/px4_drone.jsonl"),
    "raw_extrinsics_comparison": (
        "raw/extrinsics_comparison",
        "data/raw/extrinsics_comparison.jsonl",
    ),
    "method_evaluation_bundles": (
        "raw/method_evaluation_bundles",
        "data/raw/method_evaluation_bundles.jsonl",
    ),
}

for _method_result_id in MEASUREMENT_METHOD_IDS:
    AUTHORITATIVE_JSONL_STREAMS[f"method_result_{_method_result_id}"] = (
        f"raw/method_results/{_method_result_id}",
        f"data/raw/method_results/{_method_result_id}.jsonl",
    )



_INVALID_XLSX_SHEET_CHARS = frozenset("[]:\\/?*")


PX4_XLSX_TIME_COLUMNS = (
    "recording_time_ns",
    "session_utc_time_ns",
    "gps_utc_time_ns",
)


PX4_XLSX_TAB_COLUMNS: dict[str, tuple[str, ...]] = {
    "ATTITUDE": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "attitude_rollspeed_rad_s",
        "attitude_pitchspeed_rad_s",
        "attitude_yawspeed_rad_s",
        "attitude_time_boot_ms",
    ),
    "GPS1": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "gps1_latitude_deg",
        "gps1_longitude_deg",
        "gps1_altitude_m",
        "gps1_fix_type",
        "gps1_satellites_used",
        "gps1_eph_m",
        "gps1_epv_m",
        "gps1_heading_deg",
        "gps1_ground_speed_m_s",
        "gps1_yaw_deg",
        "gps1_heading_accuracy_deg",
    ),
    "GPS2": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "gps2_latitude_deg",
        "gps2_longitude_deg",
        "gps2_altitude_m",
        "gps2_fix_type",
        "gps2_satellites_used",
        "gps2_eph_m",
        "gps2_epv_m",
        "gps2_heading_deg",
        "gps2_ground_speed_m_s",
        "gps2_yaw_deg",
        "gps2_heading_accuracy_deg",
    ),
    "MAG": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "mag_sensor_slot",
        "mag_value_units",
        "mag_x",
        "mag_y",
        "mag_z",
        "mag_vector_norm",
        "mag_temperature_c",
    )
    + tuple(
        field
        for stream in ("mag_scaled_imu", "mag_scaled_imu2", "mag_scaled_imu3")
        for field in (
            f"{stream}_x",
            f"{stream}_y",
            f"{stream}_z",
            f"{stream}_unit",
            f"{stream}_accel_x",
            f"{stream}_accel_y",
            f"{stream}_accel_z",
            f"{stream}_gyro_x",
            f"{stream}_gyro_y",
            f"{stream}_gyro_z",
            f"{stream}_time_boot_ms",
        )
    ),
    "GLOBAL_POSITION": PX4_XLSX_TIME_COLUMNS
    + tuple(
        (
            "topic",
            "mavlink_message_name",
            "mavlink_time_boot_ms",
            "mavlink_global_latitude_deg",
            "mavlink_global_longitude_deg",
            "mavlink_global_altitude_m",
            "mavlink_global_relative_altitude_m",
            "mavlink_global_velocity_n_m_s",
            "mavlink_global_velocity_e_m_s",
            "mavlink_global_velocity_d_m_s",
            "mavlink_global_heading_deg",
        )
    ),
    "LOCAL_POSITION_NED": PX4_XLSX_TIME_COLUMNS
    + tuple(
        (
            "topic",
            "mavlink_message_name",
            "mavlink_time_boot_ms",
            "mavlink_local_x_m",
            "mavlink_local_y_m",
            "mavlink_local_z_m",
            "mavlink_local_velocity_x_m_s",
            "mavlink_local_velocity_y_m_s",
            "mavlink_local_velocity_z_m_s",
        )
    ),
    "RAW_IMU": PX4_XLSX_TIME_COLUMNS
    + tuple(
        (
            "topic",
            "mavlink_message_name",
            "mavlink_time_usec",
            "mavlink_raw_accel_x",
            "mavlink_raw_accel_y",
            "mavlink_raw_accel_z",
            "mavlink_raw_gyro_x",
            "mavlink_raw_gyro_y",
            "mavlink_raw_gyro_z",
            "mavlink_raw_mag_x",
            "mavlink_raw_mag_y",
            "mavlink_raw_mag_z",
        )
    ),
    "MAG_CONFIG": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "mag_sensor_slot",
        "mag_slot",
        "mag_cal_slot",
        "mag_priority",
        "mag_rotation",
        "mag_enabled",
        "mag_expected_selected",
        "mag_actual_selected",
        "mag_armed",
        "mag_status",
        "mag_warning_text",
        "mag_param_uavcan_enable",
        "mag_param_uavcan_sub_mag",
        "mag_param_ekf2_mag_type",
        "mag_param_snapshot_json",
    ),
}


RTK_XLSX_COLUMNS = (
    "side",
    "recording_time_ns",
    "session_utc_time_ns",
    "topic",
    "heading_deg",
    "heading_accuracy_deg",
    "position_length_m",
    "accuracy_length_m",
    "gnss_fix_ok",
    "differential_solution",
    "relative_position_valid",
    "carrier_solution_floating",
    "carrier_solution_fixed",
    "moving_base_mode",
    "heading_valid",
    "device_id",
)
