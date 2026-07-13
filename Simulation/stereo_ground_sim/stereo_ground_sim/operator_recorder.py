import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32, Float32MultiArray, Float64, Float64MultiArray, String

cv2 = None
_CLOSING = False
AXIS_CONTRACT = "stereodistance_public_frd:forward,right,down,range"


NUMERIC_TOPICS = {
    "/ground_truth": "actual_distance_m",
    "/target_distance": "target_distance_m",
    "/stereo/distance_error": "distance_error_m",
    "/stereo/disparity": "disparity_px",
    "/stereo/yolo_inference_ms": "yolo_inference_ms",
    "/stereo/yolo_update_hz": "yolo_update_hz",
    "/stereo/attitude_model/actual/disparity": "analytic_disparity_px",
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

PROFILE_STATUS_FIELDS = [
    "profile_id",
    "profile_mode",
    "profile_fallback_level",
    "profile_fallback_reason",
]

RESEARCH_FIELDS = [
    "analytic_forward_m",
    "analytic_right_m",
    "analytic_down_m",
    "analytic_range_m",
    "left_roll_deg",
    "left_pitch_deg",
    "left_yaw_deg",
    "right_roll_deg",
    "right_pitch_deg",
    "right_yaw_deg",
    "common_roll_deg",
    "common_pitch_deg",
    "common_yaw_deg",
    "relative_roll_deg",
    "relative_pitch_deg",
    "relative_yaw_deg",
    "analytic_vertical_disparity_px",
    "disparity_difference_px",
    "ray_gap_m",
    "intersection_angle_rad",
    "intersection_angle_deg",
    "triangulation_status",
    "oracle_source_role",
    "oracle_matched_entity_name",
    "oracle_fallback_used",
    "px4_connected",
    "px4_armed",
    "px4_navigation_mode",
    "px4_mission_phase",
    "px4_mission_phase_source",
    "px4_failsafe",
    "px4_local_north_m",
    "px4_local_east_m",
    "px4_local_down_m",
    "px4_velocity_north_mps",
    "px4_velocity_east_mps",
    "px4_velocity_down_mps",
    "px4_boot_timestamp_us",
]

RECEIVE_TIME_FIELDS = [
    "ground_truth_receive_ros_time_sec",
    "observed_receive_ros_time_sec",
    "analytic_receive_ros_time_sec",
    "corrected_receive_ros_time_sec",
    "attitude_status_receive_ros_time_sec",
    "observation_status_receive_ros_time_sec",
    "profile_status_receive_ros_time_sec",
    "px4_status_receive_ros_time_sec",
    "left_image_header_time_sec",
    "right_image_header_time_sec",
    "left_image_receive_ros_time_sec",
    "right_image_receive_ros_time_sec",
]

STRING_FIELDS = {
    "axis_contract",
    "profile_id",
    "profile_mode",
    "profile_fallback_reason",
    "triangulation_status",
    "oracle_source_role",
    "oracle_matched_entity_name",
    "px4_navigation_mode",
    "px4_mission_phase",
    "px4_mission_phase_source",
}

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
    "/stereo/bbox_adjustment",
    "/stereo/profile_status",
    "/stereo/attitude_model/status",
    "/stereo/observation_status",
    "/stereo/attitude_model/raw/estimated_position",
    "/stereo/attitude_model/actual/disparity",
    "/stereo/attitude_model/actual/pixels",
    "/stereo/attitude_model/raw/ray_quality",
    "/simulation/px4/status",
    "/stereo/distance_marker",
    "/target_distance",
    "/stereo/yolo_inference_ms",
    "/stereo/yolo_update_hz",
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


def _csv_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return value
    return _finite_or_empty(value)


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
        self.csv_file = None
        self.csv_writer = None
        self.video_writers = {}
        self.bag_process = None
        self._manifest = self._initial_manifest()
        self.bag_storage = (
            self._resolve_bag_storage(args.bag_storage) if self.mode == "debug" else args.bag_storage
        )
        self._open_csv()
        self._write_manifest()
        self._create_subscriptions()
        self.create_timer(1.0 / self.csv_hz, self._write_csv_row)
        if self.mode in ("full", "debug"):
            self._create_video_subscriptions()
        if self.mode == "debug":
            self._start_debug_bag()
        self.get_logger().info(f"Recording {self.mode} session to {self.session_dir}")

    def _open_csv(self):
        path = self.session_dir / "telemetry.csv"
        fields = (
            ["time_sec", "ros_time_sec", "axis_contract"]
            + list(NUMERIC_TOPICS.values())
            + COMPARISON_FIELDS
            + GROUND_TRUTH_POSITION_FIELDS
            + ESTIMATED_POSITION_FIELDS
            + CORRECTED_POSITION_FIELDS
            + BBOX_ADJUSTMENT_FIELDS
            + DETECTION_FIELDS
            + PROFILE_STATUS_FIELDS
            + RESEARCH_FIELDS
            + RECEIVE_TIME_FIELDS
        )
        self.csv_fields = list(dict.fromkeys(fields))
        self.csv_file = path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.csv_fields)
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def _write_manifest(self):
        lines = [
            f"mode={self.mode}",
            f"csv=telemetry.csv",
            f"csv_hz={self.csv_hz:g}",
            f"axis_contract={AXIS_CONTRACT}",
            "axis_public_fields=drone_forward_m/drone_right_m/drone_down_m, raw_forward_m/raw_right_m/raw_down_m, estimated_forward_m/estimated_right_m/estimated_down_m, corrected_forward_m/corrected_right_m/corrected_down_m",
            "axis_aliases=public CSV uses forward/right/down; legacy x/y/z is intentionally not emitted",
            "estimated_position_topic=/stereo/estimated_position publishes raw forward/right/down/range",
            "corrected_position_topic=/stereo/corrected_position publishes bbox-adjusted forward/right/down/range",
            "bbox_adjustment_topic=/stereo/bbox_adjustment publishes corrected centers, center deltas, profile hash, fallback, sample count, raw disparity/range",
            "profile_status_topic=/stereo/profile_status publishes profile_id, mode, fallback_level, fallback_reason",
            "output_frame=baseline_center_frd",
            "position_unit=m",
            "angle_unit=deg",
            "disparity_unit=px",
            "timestamp_policy=latest_value_snapshot_at_recorder_timer",
            "synchronized_measurements=false",
            "formal_cross_source_error_valid=false",
            "ground_truth_policy=gazebo_target_reference_link",
            "px4_role=auxiliary_not_ground_truth",
            "manifest_json=manifest.json",
        ]
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
        self._write_manifest_json()

    def _initial_manifest(self):
        git_branch, git_commit, git_dirty = self._git_metadata()
        return {
            "schema_version": 1,
            "run_id": self.session_dir.name,
            "mode": self.mode,
            "recording_state": "in_progress",
            "start_time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "end_time_utc": None,
            "application": "Simulation",
            "output_frame": "baseline_center_frd",
            "position_unit": "m",
            "angle_unit": "deg",
            "disparity_unit": "px",
            "timestamp_policy": "latest_value_snapshot_at_recorder_timer",
            "synchronized_measurements": False,
            "formal_cross_source_error_valid": False,
            "ground_truth_policy": "gazebo_target_reference_link",
            "px4_role": "auxiliary_not_ground_truth",
            "topic_sources": {
                "images": ["/left_camera/detection_image", "/right_camera/detection_image"],
                "positions": [
                    "/ground_truth_position",
                    "/stereo/estimated_position",
                    "/stereo/attitude_model/raw/estimated_position",
                    "/stereo/corrected_position",
                ],
                "geometry": [
                    "/stereo/disparity",
                    "/stereo/attitude_model/actual/disparity",
                    "/stereo/attitude_model/actual/pixels",
                    "/stereo/attitude_model/raw/ray_quality",
                ],
                "status": [
                    "/stereo/attitude_model/status",
                    "/stereo/observation_status",
                    "/stereo/profile_status",
                    "/simulation/px4/status",
                ],
            },
            "output_files": self._expected_output_files(),
            "git_branch": git_branch,
            "git_commit": git_commit,
            "git_dirty": git_dirty,
        }

    def _expected_output_files(self):
        files = ["telemetry.csv", "manifest.txt", "manifest.json"]
        if self.mode in ("full", "debug"):
            files.extend(["left_overlay.mp4", "right_overlay.mp4"])
        if self.mode == "debug":
            files.append("rosbag")
        return files

    def _git_metadata(self):
        root = Path(__file__).resolve()
        for parent in (root, *root.parents):
            marker = parent / ".git"
            if marker.exists():
                try:
                    branch = subprocess.run(
                        ["git", "-C", str(parent), "branch", "--show-current"],
                        capture_output=True,
                        text=True,
                        check=False,
                    ).stdout.strip()
                    commit = subprocess.run(
                        ["git", "-C", str(parent), "rev-parse", "HEAD"],
                        capture_output=True,
                        text=True,
                        check=False,
                    ).stdout.strip()
                    dirty = subprocess.run(
                        ["git", "-C", str(parent), "status", "--short"],
                        capture_output=True,
                        text=True,
                        check=False,
                    ).stdout.strip()
                    return branch or "unavailable", commit or "unavailable", bool(dirty)
                except OSError:
                    break
        return "unavailable", "unavailable", "unavailable"

    def _write_manifest_json(self):
        path = self.session_dir / "manifest.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(self._manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(temporary, path)

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
            message_type = Float64 if topic == "/stereo/attitude_model/actual/disparity" else Float32
            self.create_subscription(
                message_type,
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
            Float64MultiArray,
            "/stereo/attitude_model/raw/estimated_position",
            self._analytic_position_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/stereo/attitude_model/actual/pixels",
            self._pixels_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/stereo/attitude_model/raw/ray_quality",
            self._ray_quality_cb,
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
            "/stereo/attitude_model/status",
            self._attitude_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/stereo/observation_status",
            self._observation_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/stereo/profile_status",
            self._profile_status_cb,
            10,
        )
        self.create_subscription(
            String,
            "/simulation/px4/status",
            self._px4_status_cb,
            10,
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
            lambda msg: self._image_cb("left", msg, "left_overlay"),
            qos,
        )
        self.create_subscription(
            ImageMsg,
            "/right_camera/detection_image",
            lambda msg: self._image_cb("right", msg, "right_overlay"),
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

    def _analytic_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        for field, value in zip(
            ("analytic_forward_m", "analytic_right_m", "analytic_down_m", "analytic_range_m"),
            values,
        ):
            self.latest[field] = float(value)
        self.last_stamp["/stereo/attitude_model/raw/estimated_position"] = self.get_clock().now().nanoseconds * 1e-9

    def _pixels_cb(self, msg):
        values = list(msg.data)
        self.latest["analytic_vertical_disparity_px"] = values[5] if len(values) > 5 else math.nan
        self.latest["pixels"] = values
        self.last_stamp["/stereo/attitude_model/actual/pixels"] = self.get_clock().now().nanoseconds * 1e-9

    def _ray_quality_cb(self, msg):
        values = list(msg.data)
        self.latest["ray_quality"] = values
        self.latest["ray_gap_m"] = values[0] if len(values) > 0 else math.nan
        self.latest["intersection_angle_rad"] = values[1] if len(values) > 1 else math.nan
        self.last_stamp["/stereo/attitude_model/raw/ray_quality"] = self.get_clock().now().nanoseconds * 1e-9

    def _corrected_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        for field, value in zip(CORRECTED_POSITION_FIELDS, values):
            self.latest[field] = float(value)
        self.last_stamp["/stereo/corrected_position"] = self.get_clock().now().nanoseconds * 1e-9

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
        payload = self._json_payload(msg.data)
        self.latest["profile_status"] = payload
        self.latest["profile_id"] = str(payload.get("profile_id", ""))
        self.latest["profile_mode"] = str(payload.get("profile_mode", ""))
        self.latest["profile_fallback_level"] = payload.get("fallback_level", math.nan)
        self.latest["profile_fallback_reason"] = str(payload.get("fallback_reason", ""))
        self.last_stamp["/stereo/profile_status"] = self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _json_payload(value):
        try:
            payload = json.loads(value)
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = {}
        return payload if isinstance(payload, dict) else {}

    def _attitude_status_cb(self, msg):
        payload = self._json_payload(msg.data)
        self.latest["attitude_status"] = payload
        for field in (
            "left_roll_error_deg",
            "left_pitch_error_deg",
            "left_yaw_error_deg",
            "right_roll_error_deg",
            "right_pitch_error_deg",
            "right_yaw_error_deg",
            "common_roll_deg",
            "common_pitch_deg",
            "common_yaw_deg",
            "relative_roll_deg",
            "relative_pitch_deg",
            "relative_yaw_deg",
            "raw_triangulation_status",
            "ray_gap_m",
            "intersection_angle_rad",
            "oracle_source_role",
            "oracle_matched_entity_name",
            "oracle_fallback_used",
        ):
            if field in payload:
                self.latest[field] = payload[field]
        self.latest["triangulation_status"] = payload.get("raw_triangulation_status", "")
        self.last_stamp["/stereo/attitude_model/status"] = self.get_clock().now().nanoseconds * 1e-9

    def _observation_status_cb(self, msg):
        payload = self._json_payload(msg.data)
        self.latest["observation_status"] = payload
        for field in (
            "oracle_source_role",
            "oracle_matched_entity_name",
            "oracle_fallback_used",
        ):
            if field in payload and field not in self.latest:
                self.latest[field] = payload[field]
        self.last_stamp["/stereo/observation_status"] = self.get_clock().now().nanoseconds * 1e-9

    def _px4_status_cb(self, msg):
        payload = self._json_payload(msg.data)
        self.latest["px4_status"] = payload
        self.last_stamp["/simulation/px4/status"] = self.get_clock().now().nanoseconds * 1e-9

    def _write_csv_row(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        row = {
            "time_sec": f"{time.monotonic() - self.start_monotonic:.6f}",
            "ros_time_sec": f"{now:.6f}",
            "axis_contract": AXIS_CONTRACT,
        }
        for field in (
            list(NUMERIC_TOPICS.values())
            + COMPARISON_FIELDS
            + GROUND_TRUTH_POSITION_FIELDS
            + ESTIMATED_POSITION_FIELDS
            + CORRECTED_POSITION_FIELDS
            + BBOX_ADJUSTMENT_FIELDS
            + DETECTION_FIELDS
            + PROFILE_STATUS_FIELDS
        ):
            row[field] = _csv_value(self.latest.get(field, ""))

        row.update(
            {
                field: _csv_value(self.latest.get(field, ""))
                for field in (
                    "analytic_forward_m",
                    "analytic_right_m",
                    "analytic_down_m",
                    "analytic_range_m",
                    "left_roll_deg",
                    "left_pitch_deg",
                    "left_yaw_deg",
                    "right_roll_deg",
                    "right_pitch_deg",
                    "right_yaw_deg",
                    "common_roll_deg",
                    "common_pitch_deg",
                    "common_yaw_deg",
                    "relative_roll_deg",
                    "relative_pitch_deg",
                    "relative_yaw_deg",
                    "analytic_vertical_disparity_px",
                    "ray_gap_m",
                    "intersection_angle_rad",
                    "triangulation_status",
                )
            }
        )
        observed_disparity = self.latest.get("disparity_px")
        analytic_disparity = self.latest.get("analytic_disparity_px")
        observed_number = _finite_or_empty(observed_disparity)
        analytic_number = _finite_or_empty(analytic_disparity)
        if observed_number and analytic_number:
            row["disparity_difference_px"] = _finite_or_empty(
                float(observed_number) - float(analytic_number)
            )
        else:
            row["disparity_difference_px"] = ""
        angle_rad = self.latest.get("intersection_angle_rad")
        angle_number = _finite_or_empty(angle_rad)
        row["intersection_angle_deg"] = (
            _finite_or_empty(math.degrees(float(angle_number))) if angle_number else ""
        )

        attitude = self.latest.get("attitude_status") or {}
        observation = self.latest.get("observation_status") or {}
        row["oracle_source_role"] = _csv_value(
            attitude.get("oracle_source_role")
            or observation.get("oracle_source_role")
            or self.latest.get("oracle_source_role", "")
        )
        row["oracle_matched_entity_name"] = _csv_value(
            attitude.get("oracle_matched_entity_name")
            or observation.get("oracle_matched_entity_name")
            or self.latest.get("oracle_matched_entity_name", "")
        )
        row["oracle_fallback_used"] = _csv_value(
            attitude.get(
                "oracle_fallback_used",
                observation.get("oracle_fallback_used", self.latest.get("oracle_fallback_used", "")),
            )
        )

        px4 = self.latest.get("px4_status") or {}
        local_ned = px4.get("local_ned_m") if isinstance(px4.get("local_ned_m"), dict) else {}
        velocity_ned = px4.get("velocity_ned_mps") if isinstance(px4.get("velocity_ned_mps"), dict) else {}
        row.update(
            {
                "px4_connected": _csv_value(px4.get("px4_connected", "")),
                "px4_armed": _csv_value(px4.get("armed", "")),
                "px4_navigation_mode": _csv_value(px4.get("navigation_mode", "")),
                "px4_mission_phase": _csv_value(px4.get("mission_phase", "")),
                "px4_mission_phase_source": _csv_value(px4.get("mission_phase_source", "")),
                "px4_failsafe": _csv_value(px4.get("failsafe", "")),
                "px4_local_north_m": _csv_value(local_ned.get("north", "")),
                "px4_local_east_m": _csv_value(local_ned.get("east", "")),
                "px4_local_down_m": _csv_value(local_ned.get("down", "")),
                "px4_velocity_north_mps": _csv_value(velocity_ned.get("north", "")),
                "px4_velocity_east_mps": _csv_value(velocity_ned.get("east", "")),
                "px4_velocity_down_mps": _csv_value(velocity_ned.get("down", "")),
                "px4_boot_timestamp_us": _csv_value(px4.get("sample_timestamp_us", "")),
            }
        )
        receive_topics = {
            "ground_truth_receive_ros_time_sec": "/ground_truth_position",
            "observed_receive_ros_time_sec": "/stereo/estimated_position",
            "analytic_receive_ros_time_sec": "/stereo/attitude_model/raw/estimated_position",
            "corrected_receive_ros_time_sec": "/stereo/corrected_position",
            "attitude_status_receive_ros_time_sec": "/stereo/attitude_model/status",
            "observation_status_receive_ros_time_sec": "/stereo/observation_status",
            "profile_status_receive_ros_time_sec": "/stereo/profile_status",
            "px4_status_receive_ros_time_sec": "/simulation/px4/status",
            "left_image_receive_ros_time_sec": "/left_camera/detection_image",
            "right_image_receive_ros_time_sec": "/right_camera/detection_image",
        }
        for field, topic in receive_topics.items():
            row[field] = _csv_value(self.last_stamp.get(topic, ""))
        row["left_image_header_time_sec"] = _csv_value(self.latest.get("left_image_header_time_sec", ""))
        row["right_image_header_time_sec"] = _csv_value(self.latest.get("right_image_header_time_sec", ""))
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def _image_cb(self, camera_name, msg, output_name=None):
        topic = f"/{camera_name}_camera/detection_image"
        self.last_stamp[topic] = self.get_clock().now().nanoseconds * 1e-9
        if getattr(msg, "header", None) is not None:
            self.latest[f"{camera_name}_image_header_time_sec"] = _stamp_to_float(msg.header.stamp)
        image = self._msg_to_bgr(msg)
        if image is None:
            return
        output_name = output_name or camera_name
        writer = self.video_writers.get(output_name)
        if writer is None:
            path = self.session_dir / f"{output_name}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            height, width = image.shape[:2]
            writer = cv2.VideoWriter(str(path), fourcc, self.video_fps, (width, height))
            if not writer.isOpened():
                self.get_logger().error(f"Failed to open MP4 writer: {path}")
                return
            self.video_writers[output_name] = writer
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
        self._manifest["recording_state"] = "complete"
        self._manifest["end_time_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self._manifest["output_files"] = [
            name
            for name in self._expected_output_files()
            if (self.session_dir / name).exists()
        ]
        self._write_manifest_json()


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
