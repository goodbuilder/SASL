import argparse
import csv
import hashlib
import json
import math
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32, Float32MultiArray, String

try:
    from px4_msgs.msg import SensorGps, VehicleGlobalPosition, VehicleOdometry
except ImportError:  # pragma: no cover - allows recorder shell/tests without PX4 messages.
    SensorGps = None
    VehicleGlobalPosition = None
    VehicleOdometry = None

cv2 = None
_CLOSING = False
AXIS_CONTRACT = "stereodistance_public_frd:forward,right,down,range"
XLSX_SCHEMA_VERSION = "2"


NUMERIC_TOPICS = {
    "/ground_truth": "actual_distance_m",
    "/target_distance": "target_distance_m",
    "/stereo/distance_error": "distance_error_m",
    "/stereo/disparity": "disparity_px",
    "/stereo/yolo_inference_ms": "yolo_inference_ms",
    "/stereo/yolo_update_hz": "yolo_update_hz",
}

COMPARISON_FIELDS = [
    "comparison_actual_distance_m",
    "comparison_target_distance_m",
    "comparison_distance_error_m",
    "comparison_disparity_px",
]

GROUND_TRUTH_POSITION_FIELDS = [
    "drone_forward_m",
    "drone_right_m",
    "drone_down_m",
    "ground_truth_m",
]

ESTIMATED_POSITION_FIELDS = [
    "raw_forward_m",
    "raw_right_m",
    "raw_down_m",
    "raw_range_m",
    "estimated_forward_m",
    "estimated_right_m",
    "estimated_down_m",
    "estimated_range_m",
]

CORRECTED_POSITION_FIELDS = [
    "corrected_forward_m",
    "corrected_right_m",
    "corrected_down_m",
    "corrected_range_m",
]

OPTICAL_POSITION_FIELDS = [
    "optical_forward_m",
    "optical_right_m",
    "optical_down_m",
    "optical_range_m",
]

IDEAL_POSITION_FIELDS = [
    "ideal_forward_m",
    "ideal_right_m",
    "ideal_down_m",
    "ideal_range_m",
]

GPS_BASELINE_POSITION_FIELDS = [
    "raw_gps_baseline_forward_m",
    "raw_gps_baseline_right_m",
    "raw_gps_baseline_down_m",
    "raw_gps_baseline_position_error_m",
    "global_position_baseline_forward_m",
    "global_position_baseline_right_m",
    "global_position_baseline_down_m",
    "global_position_baseline_position_error_m",
    "raw_minus_global_baseline_forward_m",
    "raw_minus_global_baseline_right_m",
    "raw_minus_global_baseline_down_m",
    "raw_minus_global_baseline_delta_m",
    "uam_raw_gps_reference_forward_m",
    "uam_raw_gps_reference_right_m",
    "uam_raw_gps_reference_down_m",
    "uam_raw_gps_reference_range_m",
    "uam_raw_gps_reference_position_error_m",
    "truth_range_bin_100m",
]

BBOX_ADJUSTMENT_FIELDS = [
    "left_corrected_cx",
    "left_corrected_cy",
    "right_corrected_cx",
    "right_corrected_cy",
    "left_bbox_du_px",
    "left_bbox_dv_px",
    "right_bbox_du_px",
    "right_bbox_dv_px",
    "profile_id_hash",
    "fallback_level",
    "profile_cell_sample_count",
    "bbox_adjustment_raw_disparity_px",
    "bbox_adjustment_raw_range_m",
]

DETECTION_FIELDS = [
    "left_valid",
    "left_x1",
    "left_y1",
    "left_x2",
    "left_y2",
    "left_confidence",
    "left_cx",
    "left_cy",
    "left_width_px",
    "left_height_px",
    "right_valid",
    "right_x1",
    "right_y1",
    "right_x2",
    "right_y2",
    "right_confidence",
    "right_cx",
    "right_cy",
    "right_width_px",
    "right_height_px",
    "raw_disparity_px",
    "yolo_measurement_raw_range_m",
]

DETECTION_MESSAGE_FIELDS = [
    "left_valid",
    "left_x1",
    "left_y1",
    "left_x2",
    "left_y2",
    "left_confidence",
    "left_cx",
    "left_cy",
    "right_valid",
    "right_x1",
    "right_y1",
    "right_x2",
    "right_y2",
    "right_confidence",
    "right_cx",
    "right_cy",
]

IDEAL_OBSERVATION_FIELDS = [
    "ideal_left_u_px",
    "ideal_left_v_px",
    "ideal_right_u_px",
    "ideal_right_v_px",
    "ideal_disparity_px",
    "ideal_pixel_source",
]

RAY_TYPES = ("nominal", "optical", "ideal")


def _ray_fields():
    fields = []
    for prefix in RAY_TYPES:
        fields.extend(
            (
                f"{prefix}_ray_computation_status",
                f"{prefix}_ray_forward_m",
                f"{prefix}_ray_right_m",
                f"{prefix}_ray_down_m",
                f"{prefix}_ray_range_m",
                f"{prefix}_ray_gap_m",
                f"{prefix}_ray_intersection_angle_deg",
                f"{prefix}_ray_left_depth_m",
                f"{prefix}_ray_right_depth_m",
                f"{prefix}_ray_left_point_forward_m",
                f"{prefix}_ray_left_point_right_m",
                f"{prefix}_ray_left_point_down_m",
                f"{prefix}_ray_right_point_forward_m",
                f"{prefix}_ray_right_point_right_m",
                f"{prefix}_ray_right_point_down_m",
                f"{prefix}_ray_midpoint_forward_m",
                f"{prefix}_ray_midpoint_right_m",
                f"{prefix}_ray_midpoint_down_m",
            )
        )
    return fields


RAY_FIELDS = _ray_fields()

PROFILE_STATUS_FIELDS = [
    "profile_id",
    "profile_mode",
    "profile_fallback_level",
    "profile_fallback_reason",
]

TEXT_TELEMETRY_FIELDS = {
    "profile_id",
    "profile_mode",
    "profile_fallback_reason",
    "ideal_pixel_source",
    *(f"{prefix}_ray_computation_status" for prefix in RAY_TYPES),
}


def _camera_px4_fields():
    fields = []
    for label in ("left", "right"):
        fields.extend(
            (
                f"{label}_raw_gps_timestamp_us",
                f"{label}_raw_gps_lat_deg",
                f"{label}_raw_gps_lon_deg",
                f"{label}_raw_gps_alt_msl_m",
                f"{label}_raw_gps_eph_m",
                f"{label}_raw_gps_epv_m",
                f"{label}_raw_gps_vel_n_m_s",
                f"{label}_raw_gps_vel_e_m_s",
                f"{label}_raw_gps_vel_d_m_s",
                f"{label}_global_position_timestamp_us",
                f"{label}_global_position_lat_deg",
                f"{label}_global_position_lon_deg",
                f"{label}_global_position_alt_msl_m",
                f"{label}_global_position_eph_m",
                f"{label}_global_position_epv_m",
                f"{label}_px4_odometry_timestamp_us",
                f"{label}_px4_position_n_m",
                f"{label}_px4_position_e_m",
                f"{label}_px4_position_d_m",
                f"{label}_px4_attitude_world_enu_roll_deg",
                f"{label}_px4_attitude_world_enu_pitch_deg",
                f"{label}_px4_attitude_world_enu_yaw_deg",
                f"{label}_px4_velocity_n_m_s",
                f"{label}_px4_velocity_e_m_s",
                f"{label}_px4_velocity_d_m_s",
                f"{label}_px4_odometry_reset_counter",
                f"{label}_px4_estimated_optical_world_enu_x_m",
                f"{label}_px4_estimated_optical_world_enu_y_m",
                f"{label}_px4_estimated_optical_world_enu_z_m",
                f"{label}_px4_estimated_optical_world_enu_roll_deg",
                f"{label}_px4_estimated_optical_world_enu_pitch_deg",
                f"{label}_px4_estimated_optical_world_enu_yaw_deg",
            )
        )
    fields.extend(
        (
            "uam_raw_gps_timestamp_us",
            "uam_raw_gps_lat_deg",
            "uam_raw_gps_lon_deg",
            "uam_raw_gps_alt_msl_m",
            "uam_raw_gps_eph_m",
            "uam_raw_gps_epv_m",
            "uam_raw_gps_vel_n_m_s",
            "uam_raw_gps_vel_e_m_s",
            "uam_raw_gps_vel_d_m_s",
            "uam_global_position_timestamp_us",
            "uam_global_position_lat_deg",
            "uam_global_position_lon_deg",
            "uam_global_position_alt_msl_m",
            "uam_global_position_eph_m",
            "uam_global_position_epv_m",
            "uam_px4_odometry_timestamp_us",
            "uam_px4_position_n_m",
            "uam_px4_position_e_m",
            "uam_px4_position_d_m",
            "uam_px4_attitude_world_enu_roll_deg",
            "uam_px4_attitude_world_enu_pitch_deg",
            "uam_px4_attitude_world_enu_yaw_deg",
            "uam_px4_velocity_n_m_s",
            "uam_px4_velocity_e_m_s",
            "uam_px4_velocity_d_m_s",
            "uam_px4_odometry_reset_counter",
        )
    )
    return fields


CAMERA_PX4_FIELDS = _camera_px4_fields()


TELEMETRY_FIELDS = (
    ["time_sec", "ros_time_sec", "axis_contract"]
    + list(NUMERIC_TOPICS.values())
    + COMPARISON_FIELDS
    + GROUND_TRUTH_POSITION_FIELDS
    + ESTIMATED_POSITION_FIELDS
    + CORRECTED_POSITION_FIELDS
    + GPS_BASELINE_POSITION_FIELDS
    + BBOX_ADJUSTMENT_FIELDS
    + DETECTION_FIELDS
    + PROFILE_STATUS_FIELDS
    + CAMERA_PX4_FIELDS
    + OPTICAL_POSITION_FIELDS
    + IDEAL_POSITION_FIELDS
    + IDEAL_OBSERVATION_FIELDS
    + RAY_FIELDS
)

PROGRESS_VALIDITY_FIELDS = (
    "ground_truth_m",
    "raw_range_m",
    "raw_gps_baseline_forward_m",
    "global_position_baseline_forward_m",
    "raw_minus_global_baseline_delta_m",
    "left_x1",
    "right_x1",
)

def _px4_sheet_fields(label):
    prefix = "uam" if label == "uam" else label
    fields = [
        "time_sec",
        "ros_time_sec",
        f"{prefix}_raw_gps_timestamp_us",
        f"{prefix}_raw_gps_lat_deg",
        f"{prefix}_raw_gps_lon_deg",
        f"{prefix}_raw_gps_alt_msl_m",
        f"{prefix}_raw_gps_eph_m",
        f"{prefix}_raw_gps_epv_m",
        f"{prefix}_raw_gps_vel_n_m_s",
        f"{prefix}_raw_gps_vel_e_m_s",
        f"{prefix}_raw_gps_vel_d_m_s",
        f"{prefix}_global_position_timestamp_us",
        f"{prefix}_global_position_lat_deg",
        f"{prefix}_global_position_lon_deg",
        f"{prefix}_global_position_alt_msl_m",
        f"{prefix}_global_position_eph_m",
        f"{prefix}_global_position_epv_m",
        f"{prefix}_px4_odometry_timestamp_us",
        f"{prefix}_px4_position_n_m",
        f"{prefix}_px4_position_e_m",
        f"{prefix}_px4_position_d_m",
        f"{prefix}_px4_attitude_world_enu_roll_deg",
        f"{prefix}_px4_attitude_world_enu_pitch_deg",
        f"{prefix}_px4_attitude_world_enu_yaw_deg",
        f"{prefix}_px4_velocity_n_m_s",
        f"{prefix}_px4_velocity_e_m_s",
        f"{prefix}_px4_velocity_d_m_s",
        f"{prefix}_px4_odometry_reset_counter",
    ]
    if label in ("left", "right"):
        fields.extend(
            [
                f"{prefix}_px4_estimated_optical_world_enu_x_m",
                f"{prefix}_px4_estimated_optical_world_enu_y_m",
                f"{prefix}_px4_estimated_optical_world_enu_z_m",
                f"{prefix}_px4_estimated_optical_world_enu_roll_deg",
                f"{prefix}_px4_estimated_optical_world_enu_pitch_deg",
                f"{prefix}_px4_estimated_optical_world_enu_yaw_deg",
            ]
        )
    return tuple(fields)


XLSX_DIRECT_SHEET_FIELDS = (
    (
        "Summary",
        (
            "time_sec",
            "ros_time_sec",
            "axis_contract",
            "ground_truth_m",
            "raw_range_m",
            "corrected_range_m",
            "optical_range_m",
            "ideal_range_m",
            "target_distance_m",
            "distance_error_m",
            "disparity_px",
            "raw_disparity_px",
            "yolo_update_hz",
            "yolo_inference_ms",
        ),
    ),
    (
        "UAV PX4",
        _px4_sheet_fields("uam"),
    ),
    (
        "Left PX4",
        _px4_sheet_fields("left"),
    ),
    (
        "Right PX4",
        _px4_sheet_fields("right"),
    ),
    (
        "Camera Observations",
        (
            "time_sec",
            "ros_time_sec",
            "left_valid",
            "left_x1",
            "left_y1",
            "left_x2",
            "left_y2",
            "left_confidence",
            "right_valid",
            "right_x1",
            "right_y1",
            "right_x2",
            "right_y2",
            "right_confidence",
            "left_cx",
            "left_cy",
            "right_cx",
            "right_cy",
            "left_width_px",
            "left_height_px",
            "right_width_px",
            "right_height_px",
            "raw_disparity_px",
            "yolo_measurement_raw_range_m",
            "ideal_left_u_px",
            "ideal_left_v_px",
            "ideal_right_u_px",
            "ideal_right_v_px",
            "ideal_disparity_px",
            "ideal_pixel_source",
            "left_corrected_cx",
            "left_corrected_cy",
            "right_corrected_cx",
            "right_corrected_cy",
            "left_bbox_du_px",
            "left_bbox_dv_px",
            "right_bbox_du_px",
            "right_bbox_dv_px",
            "profile_id",
            "profile_mode",
            "profile_fallback_level",
            "profile_fallback_reason",
        ),
    ),
    (
        "Stereo Estimates",
        (
            "time_sec",
            "ros_time_sec",
            "axis_contract",
            "drone_forward_m",
            "drone_right_m",
            "drone_down_m",
            "ground_truth_m",
            "raw_forward_m",
            "raw_right_m",
            "raw_down_m",
            "raw_range_m",
            "corrected_forward_m",
            "corrected_right_m",
            "corrected_down_m",
            "corrected_range_m",
            "optical_forward_m",
            "optical_right_m",
            "optical_down_m",
            "optical_range_m",
            "ideal_forward_m",
            "ideal_right_m",
            "ideal_down_m",
            "ideal_range_m",
            "raw_gps_baseline_forward_m",
            "raw_gps_baseline_right_m",
            "raw_gps_baseline_down_m",
            "raw_gps_baseline_position_error_m",
            "uam_raw_gps_reference_forward_m",
            "uam_raw_gps_reference_right_m",
            "uam_raw_gps_reference_down_m",
            "uam_raw_gps_reference_range_m",
            "uam_raw_gps_reference_position_error_m",
            "global_position_baseline_forward_m",
            "global_position_baseline_right_m",
            "global_position_baseline_down_m",
            "global_position_baseline_position_error_m",
            "raw_minus_global_baseline_forward_m",
            "raw_minus_global_baseline_right_m",
            "raw_minus_global_baseline_down_m",
            "raw_minus_global_baseline_delta_m",
        ),
    ),
    (
        "Quality Events",
        (
            "time_sec",
            "ros_time_sec",
            "yolo_update_hz",
            "yolo_inference_ms",
            "disparity_px",
            "distance_error_m",
            "comparison_distance_error_m",
            "profile_id",
            "profile_mode",
            "profile_fallback_level",
            "profile_fallback_reason",
            "nominal_ray_computation_status",
            "optical_ray_computation_status",
            "ideal_ray_computation_status",
            "nominal_ray_gap_m",
            "optical_ray_gap_m",
            "ideal_ray_gap_m",
            "nominal_ray_intersection_angle_deg",
            "optical_ray_intersection_angle_deg",
            "ideal_ray_intersection_angle_deg",
        ),
    ),
)

RAYS_SHEET_FIELDS = (
    "time_sec",
    "ros_time_sec",
    "ray_type",
    "computation_status",
    "forward_m",
    "right_m",
    "down_m",
    "range_m",
    "ray_gap_m",
    "intersection_angle_deg",
    "left_depth_m",
    "right_depth_m",
    "left_point_forward_m",
    "left_point_right_m",
    "left_point_down_m",
    "right_point_forward_m",
    "right_point_right_m",
    "right_point_down_m",
    "midpoint_forward_m",
    "midpoint_right_m",
    "midpoint_down_m",
)

COMPACT_DISTANCE_FIELDS = (
    "time_sec",
    "ros_time_sec",
    "ground_truth_m",
    "drone_forward_m",
    "drone_right_m",
    "drone_down_m",
    "raw_forward_m",
    "raw_right_m",
    "raw_down_m",
    "raw_range_m",
    "corrected_forward_m",
    "corrected_right_m",
    "corrected_down_m",
    "corrected_range_m",
    "optical_forward_m",
    "optical_right_m",
    "optical_down_m",
    "optical_range_m",
    "ideal_forward_m",
    "ideal_right_m",
    "ideal_down_m",
    "ideal_range_m",
    "raw_gps_baseline_forward_m",
    "raw_gps_baseline_right_m",
    "raw_gps_baseline_down_m",
    "raw_gps_baseline_position_error_m",
)

COMPACT_RAY_SUMMARY_FIELDS = (
    "time_sec",
    "ros_time_sec",
    "truth_forward_m",
    "nominal_forward_m",
    "optical_forward_m",
    "ideal_forward_m",
    "truth_right_m",
    "nominal_right_m",
    "optical_right_m",
    "ideal_right_m",
    "truth_down_m",
    "nominal_down_m",
    "optical_down_m",
    "ideal_down_m",
    "truth_Rg_m",
    "nominal_Rg_m",
    "optical_Rg_m",
    "ideal_Rg_m",
    "nominal_ray_gap_m",
    "optical_ray_gap_m",
    "ideal_ray_gap_m",
    "nominal_ray_intersection_angle_deg",
    "optical_ray_intersection_angle_deg",
    "ideal_ray_intersection_angle_deg",
)

COMPACT_DETECTION_FIELDS = (
    "time_sec",
    "ros_time_sec",
    "left_valid",
    "left_x1",
    "left_y1",
    "left_x2",
    "left_y2",
    "left_confidence",
    "left_cx",
    "left_cy",
    "right_valid",
    "right_x1",
    "right_y1",
    "right_x2",
    "right_y2",
    "right_confidence",
    "right_cx",
    "right_cy",
    "raw_disparity_px",
    "yolo_measurement_raw_range_m",
)


def _compact_px4_fields(label):
    prefix = "uam" if label == "uam" else label
    fields = [
        "time_sec",
        "ros_time_sec",
        f"{prefix}_raw_gps_timestamp_us",
        f"{prefix}_raw_gps_lat_deg",
        f"{prefix}_raw_gps_lon_deg",
        f"{prefix}_raw_gps_alt_msl_m",
        f"{prefix}_global_position_timestamp_us",
        f"{prefix}_global_position_lat_deg",
        f"{prefix}_global_position_lon_deg",
        f"{prefix}_global_position_alt_msl_m",
        f"{prefix}_px4_odometry_timestamp_us",
        f"{prefix}_px4_position_n_m",
        f"{prefix}_px4_position_e_m",
        f"{prefix}_px4_position_d_m",
        f"{prefix}_px4_attitude_world_enu_roll_deg",
        f"{prefix}_px4_attitude_world_enu_pitch_deg",
        f"{prefix}_px4_attitude_world_enu_yaw_deg",
    ]
    if label in ("left", "right"):
        fields.extend(
            [
                f"{prefix}_px4_estimated_optical_world_enu_x_m",
                f"{prefix}_px4_estimated_optical_world_enu_y_m",
                f"{prefix}_px4_estimated_optical_world_enu_z_m",
                f"{prefix}_px4_estimated_optical_world_enu_roll_deg",
                f"{prefix}_px4_estimated_optical_world_enu_pitch_deg",
                f"{prefix}_px4_estimated_optical_world_enu_yaw_deg",
            ]
        )
    return tuple(fields)


COMPACT_PX4_FIELDS = {
    "uam": _compact_px4_fields("uam"),
    "left": _compact_px4_fields("left"),
    "right": _compact_px4_fields("right"),
}

XLSX_SHEET_FIELDS = XLSX_DIRECT_SHEET_FIELDS + (("Rays", RAYS_SHEET_FIELDS),)

DEBUG_BAG_TOPICS = [
    "/left_camera/image",
    "/left_camera/detection_image",
    "/right_camera/image",
    "/right_camera/detection_image",
    "/left_camera/camera_info",
    "/right_camera/camera_info",
    "/stereo/detections",
    "/stereo/disparity",
    "/ground_truth",
    "/stereo/distance_error",
    "/stereo/distance_comparison",
    "/ground_truth_position",
    "/stereo/estimated_position",
    "/stereo/corrected_position",
    "/stereo/optical_position",
    "/stereo/ideal_position",
    "/stereo/ray_metrics",
    "/stereo/bbox_adjustment",
    "/stereo/profile_status",
    "/stereo/raw_gps_baseline_position",
    "/stereo/global_position_baseline_position",
    "/stereo/uam_raw_gps_reference_position",
    "/stereo/distance_marker",
    "/target_distance",
    "/stereo/yolo_inference_ms",
    "/stereo/yolo_update_hz",
    "/camera_left/fmu/out/vehicle_gps_position",
    "/camera_right/fmu/out/vehicle_gps_position",
    "/fmu/out/vehicle_gps_position",
    "/camera_left/fmu/out/vehicle_global_position",
    "/camera_right/fmu/out/vehicle_global_position",
    "/fmu/out/vehicle_global_position",
    "/camera_left/fmu/out/vehicle_odometry",
    "/camera_right/fmu/out/vehicle_odometry",
    "/fmu/out/vehicle_odometry",
    "/simulation/camera_pose_diagnostics/left",
    "/simulation/camera_pose_diagnostics/right",
]


def _stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _finite_or_empty(value):
    if value is None:
        return ""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(number):
        return ""
    return f"{number:.9g}"


def _finite_vector(values, size):
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"expected {size} finite values")
    return result


def _rotation_from_wxyz(quaternion_wxyz):
    w, x, y, z = _finite_vector(quaternion_wxyz, 4)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _rotation_from_xyzw(quaternion_xyzw):
    x, y, z, w = _finite_vector(quaternion_xyzw, 4)
    return _rotation_from_wxyz((w, x, y, z))


def _rpy_deg_from_rotation(rotation):
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    pitch = math.asin(max(-1.0, min(1.0, -float(matrix[2, 0]))))
    roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
    yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


NED_TO_ENU_ROTATION = np.array(
    ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)), dtype=float
)


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _ray_sheet_values(source_row, ray_type):
    prefix = ray_type
    values = [
        source_row.get("time_sec", ""),
        source_row.get("ros_time_sec", ""),
        ray_type,
        source_row.get(f"{prefix}_ray_computation_status", ""),
        source_row.get(f"{prefix}_ray_forward_m", ""),
        source_row.get(f"{prefix}_ray_right_m", ""),
        source_row.get(f"{prefix}_ray_down_m", ""),
        source_row.get(f"{prefix}_ray_range_m", ""),
        source_row.get(f"{prefix}_ray_gap_m", ""),
        source_row.get(f"{prefix}_ray_intersection_angle_deg", ""),
        source_row.get(f"{prefix}_ray_left_depth_m", ""),
        source_row.get(f"{prefix}_ray_right_depth_m", ""),
        source_row.get(f"{prefix}_ray_left_point_forward_m", ""),
        source_row.get(f"{prefix}_ray_left_point_right_m", ""),
        source_row.get(f"{prefix}_ray_left_point_down_m", ""),
        source_row.get(f"{prefix}_ray_right_point_forward_m", ""),
        source_row.get(f"{prefix}_ray_right_point_right_m", ""),
        source_row.get(f"{prefix}_ray_right_point_down_m", ""),
        source_row.get(f"{prefix}_ray_midpoint_forward_m", ""),
        source_row.get(f"{prefix}_ray_midpoint_right_m", ""),
        source_row.get(f"{prefix}_ray_midpoint_down_m", ""),
    ]
    return values


def _ray_sheet_has_data(values):
    return any(str(value).strip() for value in values[3:])


def _write_json_atomic(path, payload):
    output = Path(path)
    temporary = output.with_name(f"{output.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)


def _write_csv_atomic(path, rows, fields):
    output = Path(path)
    temporary = output.with_name(f"{output.name}.tmp")
    row_count = 0
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})
            row_count += 1
    temporary.replace(output)
    return row_count


def _has_any_compact_data(row, fields):
    return any(str(row.get(field, "")).strip() for field in fields[2:])


def _compact_distance_row(row):
    return {field: row.get(field, "") for field in COMPACT_DISTANCE_FIELDS}


def _compact_detection_row(row):
    return {field: row.get(field, "") for field in COMPACT_DETECTION_FIELDS}


def _compact_px4_row(row, label):
    return {field: row.get(field, "") for field in COMPACT_PX4_FIELDS[label]}


def _compact_ray_summary_row(row):
    compact = {
        "time_sec": row.get("time_sec", ""),
        "ros_time_sec": row.get("ros_time_sec", ""),
        "truth_forward_m": row.get("drone_forward_m", ""),
        "truth_right_m": row.get("drone_right_m", ""),
        "truth_down_m": row.get("drone_down_m", ""),
        "truth_Rg_m": row.get("ground_truth_m", ""),
    }
    for ray_type in RAY_TYPES:
        compact[f"{ray_type}_forward_m"] = row.get(f"{ray_type}_ray_midpoint_forward_m", "")
        compact[f"{ray_type}_right_m"] = row.get(f"{ray_type}_ray_midpoint_right_m", "")
        compact[f"{ray_type}_down_m"] = row.get(f"{ray_type}_ray_midpoint_down_m", "")
        compact[f"{ray_type}_Rg_m"] = row.get(f"{ray_type}_ray_range_m", "")
        compact[f"{ray_type}_ray_gap_m"] = row.get(f"{ray_type}_ray_gap_m", "")
        compact[f"{ray_type}_ray_intersection_angle_deg"] = row.get(
            f"{ray_type}_ray_intersection_angle_deg", ""
        )
    return compact


def _session_static_payload(*, mode, csv_hz=None, video_fps=None, manifest_rows=()):
    static_values = {str(key): value for key, value in manifest_rows}
    env_keys = (
        "SIM_CONFIG",
        "SIM_SCENARIO_CONFIG",
        "DRONE_SPAWN_X_M",
        "DRONE_SPAWN_Y_M",
        "DRONE_SPAWN_Z_M",
        "DRONE_SPAWN_YAW_DEG",
        "DRONE_TAKEOFF_ALTITUDE_M",
        "DRONE_FORWARD_SPEED_MPS",
        "WAYPOINTS_XY_ALT_M",
        "WAYPOINT_SPEED_MPS",
        "WAYPOINT_ACCEPTANCE_RADIUS_M",
        "RQT_MISSION_LABEL",
        "RQT_COORD_CONTRACT",
    )
    env_snapshot = {
        key: os.environ.get(key, "")
        for key in env_keys
        if os.environ.get(key, "") != ""
    }
    return {
        "file_type": "simulation_operator_session_static",
        "schema_version": "1",
        "generated_utc": datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "mode": mode,
        "axis_contract": AXIS_CONTRACT,
        "csv_hz": csv_hz,
        "video_fps": video_fps,
        "static_values": static_values,
        "environment": env_snapshot,
        "compact_outputs": {
            "distance_measurements": list(COMPACT_DISTANCE_FIELDS),
            "ray_summary": list(COMPACT_RAY_SUMMARY_FIELDS),
            "detections": list(COMPACT_DETECTION_FIELDS),
            "px4_uam": list(COMPACT_PX4_FIELDS["uam"]),
            "px4_left": list(COMPACT_PX4_FIELDS["left"]),
            "px4_right": list(COMPACT_PX4_FIELDS["right"]),
        },
    }


def export_compact_session_files(csv_path, session_dir, *, mode, csv_hz=None, video_fps=None, manifest_rows=()):
    """Export minimal time-varying CSVs plus one static JSON from telemetry.csv."""
    source = Path(csv_path)
    output_dir = Path(session_dir)
    outputs = {
        "session_static": output_dir / "session_static.json",
        "distance_measurements": output_dir / "distance_measurements.csv",
        "ray_summary": output_dir / "ray_summary.csv",
        "detections": output_dir / "detections.csv",
        "px4_uam": output_dir / "px4_uam.csv",
        "px4_left": output_dir / "px4_left.csv",
        "px4_right": output_dir / "px4_right.csv",
        "manifest": output_dir / "session_manifest.json",
    }
    with source.open(newline="", encoding="utf-8") as handle:
        source_rows = list(csv.DictReader(handle))

    distance_rows = [_compact_distance_row(row) for row in source_rows]
    ray_rows = [
        compact
        for compact in (_compact_ray_summary_row(row) for row in source_rows)
        if _has_any_compact_data(compact, COMPACT_RAY_SUMMARY_FIELDS)
    ]
    detection_rows = [
        compact
        for compact in (_compact_detection_row(row) for row in source_rows)
        if _has_any_compact_data(compact, COMPACT_DETECTION_FIELDS)
    ]
    px4_rows = {
        label: [
            compact
            for compact in (_compact_px4_row(row, label) for row in source_rows)
            if _has_any_compact_data(compact, COMPACT_PX4_FIELDS[label])
        ]
        for label in ("uam", "left", "right")
    }

    _write_json_atomic(
        outputs["session_static"],
        _session_static_payload(
            mode=mode,
            csv_hz=csv_hz,
            video_fps=video_fps,
            manifest_rows=manifest_rows,
        ),
    )
    row_counts = {
        "source_telemetry": len(source_rows),
        "distance_measurements": _write_csv_atomic(
            outputs["distance_measurements"], distance_rows, COMPACT_DISTANCE_FIELDS
        ),
        "ray_summary": _write_csv_atomic(outputs["ray_summary"], ray_rows, COMPACT_RAY_SUMMARY_FIELDS),
        "detections": _write_csv_atomic(outputs["detections"], detection_rows, COMPACT_DETECTION_FIELDS),
        "px4_uam": _write_csv_atomic(outputs["px4_uam"], px4_rows["uam"], COMPACT_PX4_FIELDS["uam"]),
        "px4_left": _write_csv_atomic(outputs["px4_left"], px4_rows["left"], COMPACT_PX4_FIELDS["left"]),
        "px4_right": _write_csv_atomic(outputs["px4_right"], px4_rows["right"], COMPACT_PX4_FIELDS["right"]),
    }
    artifact_roles = {
        outputs["session_static"].name: "session_static_json",
        outputs["distance_measurements"].name: "compact_distance_measurements_csv",
        outputs["ray_summary"].name: "compact_ray_summary_csv",
        outputs["detections"].name: "compact_detections_csv",
        outputs["px4_uam"].name: "compact_px4_uam_csv",
        outputs["px4_left"].name: "compact_px4_left_csv",
        outputs["px4_right"].name: "compact_px4_right_csv",
        source.name: "legacy_wide_telemetry_csv",
    }
    manifest = {
        "status": "finalized",
        "schema_version": "1",
        "mode": mode,
        "axis_contract": AXIS_CONTRACT,
        "source_csv": source.name,
        "source_csv_sha256": _sha256(source),
        "row_counts": row_counts,
        "artifact_roles": artifact_roles,
    }
    _write_json_atomic(outputs["manifest"], manifest)
    return {
        "outputs": outputs,
        "row_counts": row_counts,
        "manifest": manifest,
    }


def export_session_xlsx(csv_path, xlsx_path, *, mode, manifest_rows=()):
    """Create an atomic, purpose-tab XLSX mirror of finalized telemetry.csv."""
    try:
        import xlsxwriter
        from xlsxwriter.exceptions import FileCreateError, FileSizeError
    except ModuleNotFoundError as exc:
        raise RuntimeError("XlsxWriter is required to generate session.xlsx") from exc

    source = Path(csv_path)
    output = Path(xlsx_path)
    temporary = output.with_name(f"{output.name}.tmp")
    try:
        workbook = xlsxwriter.Workbook(str(temporary), {"constant_memory": True})
        workbook.set_properties(
            {
                "title": "Stereo operator recording",
                "subject": "Finalized telemetry CSV mirror",
                "comments": "Generated from the authoritative telemetry.csv at Stop.",
                "created": datetime(1980, 1, 1),
            }
        )
        header_format = workbook.add_format({"bold": True})
        sheets = []
        for sheet_name, fields in XLSX_DIRECT_SHEET_FIELDS:
            worksheet = workbook.add_worksheet(sheet_name)
            worksheet.freeze_panes(1, 0)
            worksheet.write_row(0, 0, fields, header_format)
            sheets.append((worksheet, fields))
        rays_worksheet = workbook.add_worksheet("Rays")
        rays_worksheet.freeze_panes(1, 0)
        rays_worksheet.write_row(0, 0, RAYS_SHEET_FIELDS, header_format)
        rays_row_count = 0

        row_count = 0
        axis_contract = AXIS_CONTRACT
        with source.open(newline="", encoding="utf-8") as source_file:
            reader = csv.DictReader(source_file)
            for row in reader:
                for worksheet, fields in sheets:
                    worksheet.write_row(
                        row_count + 1,
                        0,
                        [row.get(field, "") for field in fields],
                    )
                for ray_type in RAY_TYPES:
                    ray_values = _ray_sheet_values(row, ray_type)
                    if not _ray_sheet_has_data(ray_values):
                        continue
                    rays_worksheet.write_row(rays_row_count + 1, 0, ray_values)
                    rays_row_count += 1
                if row_count == 0 and row.get("axis_contract"):
                    axis_contract = row["axis_contract"]
                row_count += 1

        csv_sha256 = _sha256(source)
        manifest = workbook.add_worksheet("Config")
        manifest.freeze_panes(1, 0)
        manifest.write_row(0, 0, ("key", "value"), header_format)
        manifest_rows = (
                ("xlsx_schema_version", XLSX_SCHEMA_VERSION),
                ("source_csv", source.name),
                ("source_csv_sha256", csv_sha256),
                ("source_csv_row_count", row_count),
                ("rays_sheet_row_count", rays_row_count),
                ("mode", mode),
                ("axis_contract", axis_contract),
                ("sheet_order", ",".join([name for name, _fields in XLSX_SHEET_FIELDS] + ["Config"])),
            ) + tuple(manifest_rows)
        for index, row in enumerate(
            manifest_rows,
            start=1,
        ):
            manifest.write_row(index, 0, row)
        workbook.close()
        temporary.replace(output)
    except (FileCreateError, FileSizeError, OSError) as exc:
        temporary.unlink(missing_ok=True)
        raise RuntimeError(f"XLSX export failed: {exc}") from exc
    return {"path": output, "row_count": row_count, "csv_sha256": csv_sha256}


class OperatorRecorder(Node):
    def __init__(self, args):
        super().__init__("operator_recorder")
        self.mode = args.mode
        self.session_dir = Path(args.output_dir).expanduser().resolve()
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.csv_hz = max(0.5, float(args.csv_hz))
        self.video_fps = max(1.0, float(args.video_fps))
        self.start_monotonic = time.monotonic()
        self.latest = {}
        self.last_stamp = {}
        self.camera_runtime = {
            label: {
                "gazebo_base_position_world_enu_m": None,
                "gazebo_base_rotation_world_from_body": None,
                "mount_translation_body_m": None,
                "mount_rotation_body_to_optical": None,
                "mount_rpy_deg": None,
                "px4_position_ned_m": None,
                "px4_rotation_ned_from_body": None,
                "bootstrap_position_ned_m": None,
                "bootstrap_position_world_enu_m": None,
            }
            for label in ("left", "right")
        }
        self.row_count = 0
        self.valid_row_counts = {field: 0 for field in PROGRESS_VALIDITY_FIELDS}
        self.last_elapsed_sec = 0.0
        self.xlsx_status = "pending"
        self.xlsx_error = ""
        self.compact_status = "pending"
        self.compact_error = ""
        self.csv_file = None
        self.csv_writer = None
        self.video_writers = {}
        self.bag_process = None
        self.bag_storage = (
            self._resolve_bag_storage(args.bag_storage) if self.mode == "debug" else args.bag_storage
        )
        self._open_csv()
        self._write_manifest()
        self._write_progress()
        self._create_subscriptions()
        self.create_timer(1.0 / self.csv_hz, self._write_csv_row)
        self.create_timer(1.0, self._write_progress)
        if self.mode in ("full", "debug"):
            self._create_video_subscriptions()
        if self.mode == "debug":
            self._start_debug_bag()
        self.get_logger().info(f"Recording {self.mode} session to {self.session_dir}")

    def _open_csv(self):
        path = self.session_dir / "telemetry.csv"
        self.csv_file = path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=TELEMETRY_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def _write_manifest(self):
        lines = [
            f"mode={self.mode}",
            f"csv=telemetry.csv",
            f"csv_hz={self.csv_hz:g}",
            "progress=progress.json",
            "session_static=session_static.json (fixed session/config facts written once at Stop)",
            "compact_csvs=distance_measurements.csv,ray_summary.csv,detections.csv,px4_uam.csv,px4_left.csv,px4_right.csv",
            "session_manifest=session_manifest.json",
            "xlsx=session.xlsx (generated from finalized telemetry.csv at Stop)",
            "analysis_export=analysis.csv (optional compact user export; telemetry.csv remains raw)",
            f"axis_contract={AXIS_CONTRACT}",
            "axis_public_fields=drone_forward_m/drone_right_m/drone_down_m, raw_forward_m/raw_right_m/raw_down_m, estimated_forward_m/estimated_right_m/estimated_down_m, corrected_forward_m/corrected_right_m/corrected_down_m",
            "axis_aliases=public CSV uses forward/right/down; legacy x/y/z is intentionally not emitted",
            "estimated_position_topic=/stereo/estimated_position publishes raw forward/right/down/range",
            "corrected_position_topic=/stereo/corrected_position publishes bbox-adjusted forward/right/down/range",
            "optical_position_topic=/stereo/optical_position publishes YOLO pixels triangulated with Gazebo-observed optical camera poses",
            "ideal_position_topic=/stereo/ideal_position publishes GT-projected nominal-pixel triangulation for simulation reference only",
            "ray_metrics_topic=/stereo/ray_metrics publishes numeric nominal/optical/ideal ray diagnostics for the XLSX Rays sheet",
            "raw_gps_baseline_topic=/stereo/raw_gps_baseline_position publishes raw-GPS-baseline forward/right/down",
            "global_position_baseline_topic=/stereo/global_position_baseline_position publishes fused-PX4-global-baseline forward/right/down",
            "gps_baseline_errors=3D FRD error versus /ground_truth_position; truth_range_bin_100m is floor(truth forward / 100) * 100",
            "gps_baseline_comparison=raw_minus_global_baseline_{forward,right,down}_m and raw_minus_global_baseline_delta_m are raw GPS minus PX4 GlobalPosition in public FRD metres",
            "uam_raw_gps_reference_position=/stereo/uam_raw_gps_reference_position publishes the UAM raw-GPS target reference in public FRD metres; it is not simulator truth",
            "detection_bbox_contract=left/right bbox source fields are x1,y1,x2,y2 in image pixels; cx/cy and width/height are retained only as derived compatibility fields",
            "bbox_adjustment_topic=/stereo/bbox_adjustment publishes corrected centers, center deltas, profile hash, fallback, sample count, raw disparity/range",
            "profile_status_topic=/stereo/profile_status publishes profile_id, mode, fallback_level, fallback_reason",
            "px4_recording=UAM target and left/right camera raw GPS and GlobalPosition are WGS84 samples; PX4 odometry is local NED; camera estimated optical exterior pose is world ENU and bootstrapped from Gazebo camera diagnostics",
        ]
        for label, runtime in self.camera_runtime.items():
            translation = runtime["mount_translation_body_m"]
            rpy = runtime["mount_rpy_deg"]
            if translation is None or rpy is None:
                lines.append(f"{label}_camera_mount_base_to_optical_6dof=unavailable")
                continue
            lines.extend(
                (
                    f"{label}_camera_mount_base_to_optical_translation_m="
                    + ",".join(f"{float(value):.9g}" for value in translation),
                    f"{label}_camera_mount_base_to_optical_rpy_deg="
                    + ",".join(f"{float(value):.9g}" for value in rpy),
                    f"{label}_camera_mount_source=gazebo_camera_pose_diagnostics",
                )
            )
        if self.mode in ("full", "debug"):
            lines.extend(
                [
                    "left_overlay_mp4=left_overlay.mp4",
                    "right_overlay_mp4=right_overlay.mp4",
                    f"video_fps={self.video_fps:g}",
                ]
            )
        if self.mode == "debug":
            lines.extend([f"rosbag=rosbag", f"rosbag_storage={self.bag_storage}"])
        (self.session_dir / "manifest.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _camera_xlsx_manifest_rows(self):
        rows = []
        for label, runtime in self.camera_runtime.items():
            translation = runtime["mount_translation_body_m"]
            rpy = runtime["mount_rpy_deg"]
            if translation is None or rpy is None:
                rows.append((f"{label}_camera_mount_base_to_optical_6dof", "unavailable"))
                continue
            rows.extend(
                (
                    (
                        f"{label}_camera_mount_base_to_optical_translation_m",
                        ",".join(f"{float(value):.9g}" for value in translation),
                    ),
                    (
                        f"{label}_camera_mount_base_to_optical_rpy_deg",
                        ",".join(f"{float(value):.9g}" for value in rpy),
                    ),
                    (f"{label}_camera_mount_source", "gazebo_camera_pose_diagnostics"),
                )
            )
        return tuple(rows)

    def _resolve_bag_storage(self, requested):
        if requested != "auto":
            return requested
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "rosbag2_storage_mcap"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode == 0:
            return "mcap"
        self.get_logger().warning("rosbag2_storage_mcap not found; debug rosbag will use sqlite3.")
        return "sqlite3"

    def _create_subscriptions(self):
        for topic, field in NUMERIC_TOPICS.items():
            self.create_subscription(
                Float32,
                topic,
                lambda msg, field=field, topic=topic: self._float_cb(msg, field, topic),
                10,
            )
        self.create_subscription(
            Float32MultiArray,
            "/stereo/distance_comparison",
            self._comparison_cb,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/stereo/raw_gps_baseline_position",
            lambda msg: self._gps_baseline_position_cb("raw_gps_baseline", msg),
            10,
        )
        self.create_subscription(
            PointStamped,
            "/stereo/global_position_baseline_position",
            lambda msg: self._gps_baseline_position_cb("global_position_baseline", msg),
            10,
        )
        self.create_subscription(
            PointStamped,
            "/stereo/uam_raw_gps_reference_position",
            lambda msg: self._gps_baseline_position_cb("uam_raw_gps_reference", msg),
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/ground_truth_position",
            self._ground_truth_position_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/stereo/estimated_position",
            self._estimated_position_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/stereo/corrected_position",
            self._corrected_position_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/stereo/optical_position",
            self._optical_position_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/stereo/ideal_position",
            self._ideal_position_cb,
            10,
        )
        self.create_subscription(
            String,
            "/stereo/ray_metrics",
            self._ray_metrics_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/stereo/bbox_adjustment",
            self._bbox_adjustment_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/stereo/detections",
            self._detections_cb,
            10,
        )
        self.create_subscription(
            String,
            "/stereo/profile_status",
            self._profile_status_cb,
            10,
        )
        if SensorGps is None or VehicleGlobalPosition is None or VehicleOdometry is None:
            self.get_logger().warning("PX4 message types unavailable; camera PX4 state recording is disabled.")
            return
        px4_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        camera_pose_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        px4_sources = (
            ("left", "/camera_left/fmu/out"),
            ("right", "/camera_right/fmu/out"),
            ("uam", "/fmu/out"),
        )
        for label, topic_prefix in px4_sources:
            self.create_subscription(
                SensorGps,
                f"{topic_prefix}/vehicle_gps_position",
                lambda msg, label=label: self._camera_raw_gps_cb(label, msg),
                px4_qos,
            )
            self.create_subscription(
                VehicleGlobalPosition,
                f"{topic_prefix}/vehicle_global_position",
                lambda msg, label=label: self._camera_global_position_cb(label, msg),
                px4_qos,
            )
            self.create_subscription(
                VehicleOdometry,
                f"{topic_prefix}/vehicle_odometry",
                lambda msg, label=label: self._camera_odometry_cb(label, msg),
                px4_qos,
            )
            if label == "uam":
                continue
            self.create_subscription(
                String,
                f"/simulation/camera_pose_diagnostics/{label}",
                lambda msg, label=label: self._camera_pose_diagnostic_cb(label, msg),
                camera_pose_qos,
            )

    def _create_video_subscriptions(self):
        global cv2
        if cv2 is None:
            try:
                import cv2 as cv2_module

                cv2 = cv2_module
            except Exception as exc:
                self.get_logger().error(f"OpenCV is not available; MP4 recording disabled: {exc}")
                return
        if cv2 is None:
            self.get_logger().error("OpenCV is not available; MP4 recording disabled.")
            return
        qos = QoSProfile(
            depth=3,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            ImageMsg,
            "/left_camera/detection_image",
            lambda msg: self._image_cb("left_overlay", msg),
            qos,
        )
        self.create_subscription(
            ImageMsg,
            "/right_camera/detection_image",
            lambda msg: self._image_cb("right_overlay", msg),
            qos,
        )

    def _start_debug_bag(self):
        output = self.session_dir / "rosbag"
        cmd = ["ros2", "bag", "record", "-s", self.bag_storage, "-o", str(output)] + DEBUG_BAG_TOPICS
        self.bag_process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

    def _float_cb(self, msg, field, topic):
        self.latest[field] = float(msg.data)
        self.last_stamp[topic] = self.get_clock().now().nanoseconds * 1e-9

    def _comparison_cb(self, msg):
        for field, value in zip(COMPARISON_FIELDS, msg.data):
            self.latest[field] = float(value)
        self.last_stamp["/stereo/distance_comparison"] = self.get_clock().now().nanoseconds * 1e-9

    def _ground_truth_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        self.latest["drone_forward_m"] = float(values[0])
        self.latest["drone_right_m"] = float(values[1])
        self.latest["drone_down_m"] = float(values[2])
        self.latest["ground_truth_m"] = float(values[3])
        self._update_gps_baseline_error("raw_gps_baseline")
        self._update_gps_baseline_error("global_position_baseline")
        self.last_stamp["/ground_truth_position"] = self.get_clock().now().nanoseconds * 1e-9

    def _estimated_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        self.latest["raw_forward_m"] = float(values[0])
        self.latest["raw_right_m"] = float(values[1])
        self.latest["raw_down_m"] = float(values[2])
        self.latest["raw_range_m"] = float(values[3])
        self.latest["estimated_forward_m"] = float(values[0])
        self.latest["estimated_right_m"] = float(values[1])
        self.latest["estimated_down_m"] = float(values[2])
        self.latest["estimated_range_m"] = float(values[3])
        self.last_stamp["/stereo/estimated_position"] = self.get_clock().now().nanoseconds * 1e-9

    def _corrected_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        for field, value in zip(CORRECTED_POSITION_FIELDS, values):
            self.latest[field] = float(value)
        self.last_stamp["/stereo/corrected_position"] = self.get_clock().now().nanoseconds * 1e-9

    def _optical_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        for field, value in zip(OPTICAL_POSITION_FIELDS, values):
            self.latest[field] = float(value)
        self.last_stamp["/stereo/optical_position"] = self.get_clock().now().nanoseconds * 1e-9

    def _ideal_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        for field, value in zip(IDEAL_POSITION_FIELDS, values):
            self.latest[field] = float(value)
        self.last_stamp["/stereo/ideal_position"] = self.get_clock().now().nanoseconds * 1e-9

    def _gps_baseline_position_cb(self, source, msg):
        self.latest[f"{source}_forward_m"] = float(msg.point.x)
        self.latest[f"{source}_right_m"] = float(msg.point.y)
        self.latest[f"{source}_down_m"] = float(msg.point.z)
        self._update_gps_baseline_error(source)
        self._update_gps_baseline_comparison()
        self.last_stamp[f"/stereo/{source}_position"] = _stamp_to_float(msg.header.stamp)

    def _camera_raw_gps_cb(self, label, msg):
        prefix = f"{label}_raw_gps"
        timestamp_us = int(getattr(msg, "timestamp_sample", 0) or getattr(msg, "timestamp", 0))
        self.latest[f"{prefix}_timestamp_us"] = timestamp_us
        self.latest[f"{prefix}_lat_deg"] = float(msg.latitude_deg)
        self.latest[f"{prefix}_lon_deg"] = float(msg.longitude_deg)
        self.latest[f"{prefix}_alt_msl_m"] = float(msg.altitude_msl_m)
        self.latest[f"{prefix}_eph_m"] = float(msg.eph)
        self.latest[f"{prefix}_epv_m"] = float(msg.epv)
        if bool(getattr(msg, "vel_ned_valid", False)):
            self.latest[f"{prefix}_vel_n_m_s"] = float(msg.vel_n_m_s)
            self.latest[f"{prefix}_vel_e_m_s"] = float(msg.vel_e_m_s)
            self.latest[f"{prefix}_vel_d_m_s"] = float(msg.vel_d_m_s)
        else:
            for axis in ("n", "e", "d"):
                self.latest[f"{prefix}_vel_{axis}_m_s"] = math.nan
        topic_prefix = "/fmu/out" if label == "uam" else f"/camera_{label}/fmu/out"
        self.last_stamp[f"{topic_prefix}/vehicle_gps_position"] = timestamp_us * 1e-6

    def _camera_global_position_cb(self, label, msg):
        prefix = f"{label}_global_position"
        timestamp_us = int(getattr(msg, "timestamp_sample", 0) or getattr(msg, "timestamp", 0))
        self.latest[f"{prefix}_timestamp_us"] = timestamp_us
        self.latest[f"{prefix}_lat_deg"] = float(msg.lat)
        self.latest[f"{prefix}_lon_deg"] = float(msg.lon)
        self.latest[f"{prefix}_alt_msl_m"] = float(msg.alt)
        self.latest[f"{prefix}_eph_m"] = float(msg.eph)
        self.latest[f"{prefix}_epv_m"] = float(msg.epv)
        topic_prefix = "/fmu/out" if label == "uam" else f"/camera_{label}/fmu/out"
        self.last_stamp[f"{topic_prefix}/vehicle_global_position"] = timestamp_us * 1e-6

    def _camera_odometry_cb(self, label, msg):
        if int(msg.pose_frame) != int(msg.POSE_FRAME_NED):
            return
        try:
            position_ned = np.array(_finite_vector(msg.position, 3), dtype=float)
            rotation_ned_from_body = _rotation_from_wxyz(msg.q)
        except (TypeError, ValueError):
            return
        timestamp_us = int(getattr(msg, "timestamp_sample", 0) or getattr(msg, "timestamp", 0))
        self.latest[f"{label}_px4_odometry_timestamp_us"] = timestamp_us
        for axis, value in zip(("n", "e", "d"), position_ned):
            self.latest[f"{label}_px4_position_{axis}_m"] = float(value)
        rotation_world_enu_from_body = NED_TO_ENU_ROTATION @ rotation_ned_from_body
        for axis, value in zip(
            ("roll", "pitch", "yaw"), _rpy_deg_from_rotation(rotation_world_enu_from_body)
        ):
            self.latest[f"{label}_px4_attitude_world_enu_{axis}_deg"] = value
        if int(msg.velocity_frame) == int(msg.VELOCITY_FRAME_NED):
            try:
                velocity_ned = _finite_vector(msg.velocity, 3)
            except (TypeError, ValueError):
                velocity_ned = (math.nan, math.nan, math.nan)
        else:
            velocity_ned = (math.nan, math.nan, math.nan)
        for axis, value in zip(("n", "e", "d"), velocity_ned):
            self.latest[f"{label}_px4_velocity_{axis}_m_s"] = value
        self.latest[f"{label}_px4_odometry_reset_counter"] = int(msg.reset_counter)
        topic_prefix = "/fmu/out" if label == "uam" else f"/camera_{label}/fmu/out"
        self.last_stamp[f"{topic_prefix}/vehicle_odometry"] = timestamp_us * 1e-6
        runtime = self.camera_runtime.get(label)
        if runtime is None:
            return
        runtime["px4_position_ned_m"] = position_ned
        runtime["px4_rotation_ned_from_body"] = rotation_ned_from_body
        self._update_px4_estimated_optical_extrinsic(label)

    def _camera_pose_diagnostic_cb(self, label, msg):
        try:
            payload = json.loads(msg.data)
            if payload.get("schema") != "camera_pose_comparison/v1":
                raise ValueError("unexpected camera diagnostic schema")
            base_position = np.array(
                _finite_vector(payload["observed_base_link_position_world_enu_m"], 3), dtype=float
            )
            optical_position = np.array(
                _finite_vector(payload["observed_optical_position_world_enu_m"], 3), dtype=float
            )
            base_rotation = _rotation_from_xyzw(
                payload["observed_base_link_orientation_world_from_entity_xyzw"]
            )
            optical_rotation = _rotation_from_xyzw(
                payload["observed_optical_orientation_world_from_entity_xyzw"]
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        runtime = self.camera_runtime[label]
        runtime["gazebo_base_position_world_enu_m"] = base_position
        runtime["gazebo_base_rotation_world_from_body"] = base_rotation
        runtime["mount_translation_body_m"] = base_rotation.T @ (optical_position - base_position)
        runtime["mount_rotation_body_to_optical"] = base_rotation.T @ optical_rotation
        runtime["mount_rpy_deg"] = _rpy_deg_from_rotation(runtime["mount_rotation_body_to_optical"])
        self._update_px4_estimated_optical_extrinsic(label)

    def _update_px4_estimated_optical_extrinsic(self, label):
        runtime = self.camera_runtime[label]
        position_ned = runtime["px4_position_ned_m"]
        rotation_ned_from_body = runtime["px4_rotation_ned_from_body"]
        base_position = runtime["gazebo_base_position_world_enu_m"]
        mount_translation = runtime["mount_translation_body_m"]
        mount_rotation = runtime["mount_rotation_body_to_optical"]
        if any(value is None for value in (position_ned, rotation_ned_from_body, base_position, mount_translation, mount_rotation)):
            return
        if runtime["bootstrap_position_ned_m"] is None:
            runtime["bootstrap_position_ned_m"] = np.array(position_ned, dtype=float)
            runtime["bootstrap_position_world_enu_m"] = np.array(base_position, dtype=float)
        base_position_estimated = runtime["bootstrap_position_world_enu_m"] + NED_TO_ENU_ROTATION @ (
            np.array(position_ned, dtype=float) - runtime["bootstrap_position_ned_m"]
        )
        base_rotation_estimated = NED_TO_ENU_ROTATION @ np.array(rotation_ned_from_body, dtype=float)
        optical_position_estimated = base_position_estimated + base_rotation_estimated @ np.array(
            mount_translation, dtype=float
        )
        optical_rotation_estimated = base_rotation_estimated @ np.array(mount_rotation, dtype=float)
        for axis, value in zip(("x", "y", "z"), optical_position_estimated):
            self.latest[f"{label}_px4_estimated_optical_world_enu_{axis}_m"] = float(value)
        for axis, value in zip(
            ("roll", "pitch", "yaw"), _rpy_deg_from_rotation(optical_rotation_estimated)
        ):
            self.latest[f"{label}_px4_estimated_optical_world_enu_{axis}_deg"] = value

    def _update_gps_baseline_error(self, source):
        values = (
            self.latest.get(f"{source}_forward_m"),
            self.latest.get(f"{source}_right_m"),
            self.latest.get(f"{source}_down_m"),
            self.latest.get("drone_forward_m"),
            self.latest.get("drone_right_m"),
            self.latest.get("drone_down_m"),
        )
        if not all(value is not None and math.isfinite(float(value)) for value in values):
            self.latest[f"{source}_position_error_m"] = math.nan
            return
        self.latest[f"{source}_position_error_m"] = math.sqrt(
            sum((float(values[index]) - float(values[index + 3])) ** 2 for index in range(3))
        )

    def _update_gps_baseline_comparison(self):
        """Store raw-GPS minus EKF-global baseline in the public FRD contract."""

        raw = tuple(
            self.latest.get(f"raw_gps_baseline_{axis}_m")
            for axis in ("forward", "right", "down")
        )
        global_position = tuple(
            self.latest.get(f"global_position_baseline_{axis}_m")
            for axis in ("forward", "right", "down")
        )
        if not all(
            value is not None and math.isfinite(float(value))
            for value in (*raw, *global_position)
        ):
            for axis in ("forward", "right", "down"):
                self.latest[f"raw_minus_global_baseline_{axis}_m"] = math.nan
            self.latest["raw_minus_global_baseline_delta_m"] = math.nan
            return
        difference = tuple(float(raw[index]) - float(global_position[index]) for index in range(3))
        for axis, value in zip(("forward", "right", "down"), difference):
            self.latest[f"raw_minus_global_baseline_{axis}_m"] = value
        self.latest["raw_minus_global_baseline_delta_m"] = math.sqrt(
            sum(value * value for value in difference)
        )

    def _bbox_adjustment_cb(self, msg):
        values = list(msg.data[: len(BBOX_ADJUSTMENT_FIELDS)])
        while len(values) < len(BBOX_ADJUSTMENT_FIELDS):
            values.append(math.nan)
        for field, value in zip(BBOX_ADJUSTMENT_FIELDS, values):
            self.latest[field] = float(value)
        self.last_stamp["/stereo/bbox_adjustment"] = self.get_clock().now().nanoseconds * 1e-9

    def _detections_cb(self, msg):
        values = list(msg.data[: len(DETECTION_MESSAGE_FIELDS)])
        while len(values) < len(DETECTION_MESSAGE_FIELDS):
            values.append(math.nan)
        for field, value in zip(DETECTION_MESSAGE_FIELDS, values):
            self.latest[field] = float(value)
        left_width = self.latest.get("left_x2", math.nan) - self.latest.get("left_x1", math.nan)
        left_height = self.latest.get("left_y2", math.nan) - self.latest.get("left_y1", math.nan)
        right_width = self.latest.get("right_x2", math.nan) - self.latest.get("right_x1", math.nan)
        right_height = self.latest.get("right_y2", math.nan) - self.latest.get("right_y1", math.nan)
        self.latest["left_width_px"] = left_width
        self.latest["left_height_px"] = left_height
        self.latest["right_width_px"] = right_width
        self.latest["right_height_px"] = right_height
        left_cx = self.latest.get("left_cx", math.nan)
        right_cx = self.latest.get("right_cx", math.nan)
        if math.isfinite(left_cx) and math.isfinite(right_cx):
            self.latest["raw_disparity_px"] = left_cx - right_cx
        else:
            self.latest["raw_disparity_px"] = self.latest.get("disparity_px", math.nan)
        self.latest["yolo_measurement_raw_range_m"] = self.latest.get("raw_range_m", math.nan)
        self.last_stamp["/stereo/detections"] = self.get_clock().now().nanoseconds * 1e-9

    def _profile_status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {}
        self.latest["profile_id"] = str(payload.get("profile_id", ""))
        self.latest["profile_mode"] = str(payload.get("profile_mode", ""))
        self.latest["profile_fallback_level"] = payload.get("fallback_level", math.nan)
        self.latest["profile_fallback_reason"] = str(payload.get("fallback_reason", ""))
        self.last_stamp["/stereo/profile_status"] = self.get_clock().now().nanoseconds * 1e-9

    def _ray_metrics_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        for field in IDEAL_OBSERVATION_FIELDS:
            if field in payload:
                if field == "ideal_pixel_source":
                    self.latest[field] = str(payload[field])
                else:
                    self.latest[field] = payload[field]
        for ray_type in RAY_TYPES:
            ray = payload.get(ray_type)
            if not isinstance(ray, dict):
                continue
            prefix = f"{ray_type}_ray"
            self.latest[f"{prefix}_computation_status"] = str(ray.get("computation_status", ""))
            for key in (
                "forward_m",
                "right_m",
                "down_m",
                "range_m",
                "gap_m",
                "intersection_angle_deg",
                "left_depth_m",
                "right_depth_m",
            ):
                self.latest[f"{prefix}_{key}"] = ray.get(key, math.nan)
            for source_key, field_middle in (
                ("left_point_frd_m", "left_point"),
                ("right_point_frd_m", "right_point"),
                ("midpoint_frd_m", "midpoint"),
            ):
                values = ray.get(source_key)
                if not isinstance(values, (list, tuple)) or len(values) < 3:
                    continue
                for axis, value in zip(("forward", "right", "down"), values[:3]):
                    self.latest[f"{prefix}_{field_middle}_{axis}_m"] = value
        self.last_stamp["/stereo/ray_metrics"] = self.get_clock().now().nanoseconds * 1e-9

    def _write_csv_row(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        row = {
            "time_sec": f"{time.monotonic() - self.start_monotonic:.6f}",
            "ros_time_sec": f"{now:.6f}",
            "axis_contract": AXIS_CONTRACT,
        }
        for field in TELEMETRY_FIELDS[3:]:
            if field in TEXT_TELEMETRY_FIELDS:
                row[field] = self.latest.get(field, "")
            else:
                row[field] = _finite_or_empty(self.latest.get(field))
        truth_forward = self.latest.get("drone_forward_m")
        if truth_forward is not None and math.isfinite(float(truth_forward)):
            row["truth_range_bin_100m"] = _finite_or_empty(
                math.floor(float(truth_forward) / 100.0) * 100.0
            )
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        if not hasattr(self, "row_count"):
            self.row_count = 0
            self.valid_row_counts = {field: 0 for field in PROGRESS_VALIDITY_FIELDS}
            self.last_elapsed_sec = 0.0
        self.row_count += 1
        self.last_elapsed_sec = float(row["time_sec"])
        for field in PROGRESS_VALIDITY_FIELDS:
            if row.get(field, ""):
                self.valid_row_counts[field] += 1

    def _write_progress(self, state="writing"):
        """Atomically publish recorder facts for the read-only Record tab."""
        payload = {
            "state": state,
            "mode": self.mode,
            "row_count": self.row_count,
            "elapsed_sec": round(self.last_elapsed_sec, 3),
            "valid_row_counts": self.valid_row_counts,
            "xlsx_status": getattr(self, "xlsx_status", "unavailable"),
            "xlsx_error": getattr(self, "xlsx_error", ""),
            "compact_status": getattr(self, "compact_status", "unavailable"),
            "compact_error": getattr(self, "compact_error", ""),
        }
        progress_path = self.session_dir / "progress.json"
        temporary_path = self.session_dir / "progress.json.tmp"
        temporary_path.write_text(
            json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(progress_path)

    def _image_cb(self, name, msg):
        image = self._msg_to_bgr(msg)
        if image is None:
            return
        writer = self.video_writers.get(name)
        if writer is None:
            path = self.session_dir / f"{name}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            height, width = image.shape[:2]
            writer = cv2.VideoWriter(str(path), fourcc, self.video_fps, (width, height))
            if not writer.isOpened():
                self.get_logger().error(f"Failed to open MP4 writer: {path}")
                return
            self.video_writers[name] = writer
        writer.write(image)

    def _msg_to_bgr(self, msg):
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            self.get_logger().warning(f"Unsupported image encoding for MP4: {msg.encoding}")
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            rows = raw.reshape((msg.height, msg.step))
            pixels = rows[:, : msg.width * channels].reshape((msg.height, msg.width, channels))
        except ValueError:
            self.get_logger().warning("Image buffer shape did not match metadata; frame skipped.")
            return None
        if encoding == "bgr8":
            return pixels.copy()
        if encoding == "rgb8":
            return pixels[:, :, ::-1].copy()
        if encoding == "bgra8":
            return pixels[:, :, :3].copy()
        if encoding == "rgba8":
            return pixels[:, :, 2::-1].copy()
        return np.repeat(pixels, 3, axis=2).copy()

    def close(self):
        global _CLOSING
        _CLOSING = True
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if self.bag_process is not None:
            try:
                os.killpg(self.bag_process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.bag_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.bag_process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                self.bag_process.wait(timeout=5.0)
            self.bag_process = None
        for writer in self.video_writers.values():
            writer.release()
        self.video_writers.clear()
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self._write_manifest()
            manifest_rows = self._camera_xlsx_manifest_rows()
            try:
                export_session_xlsx(
                    self.session_dir / "telemetry.csv",
                    self.session_dir / "session.xlsx",
                    mode=self.mode,
                    manifest_rows=manifest_rows,
                )
            except RuntimeError as exc:
                self.xlsx_status = "failed"
                self.xlsx_error = str(exc)
                self.get_logger().error(f"CSV saved; {exc}")
            else:
                self.xlsx_status = "saved"
                self.xlsx_error = ""
            try:
                export_compact_session_files(
                    self.session_dir / "telemetry.csv",
                    self.session_dir,
                    mode=self.mode,
                    csv_hz=self.csv_hz,
                    video_fps=self.video_fps,
                    manifest_rows=manifest_rows,
                )
            except (OSError, RuntimeError, ValueError) as exc:
                self.compact_status = "failed"
                self.compact_error = str(exc)
                self.get_logger().error(f"CSV saved; compact export failed: {exc}")
            else:
                self.compact_status = "saved"
                self.compact_error = ""
            self._write_progress(state="finalized")


def parse_args():
    parser = argparse.ArgumentParser(description="Record operator-view CSV, MP4, and debug rosbag presets.")
    parser.add_argument("--mode", choices=("lite", "full", "debug"), required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--csv-hz", type=float, default=30.0)
    parser.add_argument("--video-fps", type=float, default=20.0)
    parser.add_argument("--bag-storage", default=os.environ.get("OPERATOR_BAG_STORAGE", "auto"))
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    recorder = OperatorRecorder(args)

    def _stop(signum, frame):
        if _CLOSING:
            return
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        rclpy.spin(recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
