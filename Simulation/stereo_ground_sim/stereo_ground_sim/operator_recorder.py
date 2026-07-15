import argparse
import copy
import csv
import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32, Float32MultiArray, Float64, Float64MultiArray, String

try:
    from rcl_interfaces.srv import GetParameters
except Exception:  # pragma: no cover - ROS is optional for schema-only tests.
    GetParameters = None

from stereo_ground_sim.simulation_ui_contract import (
    STALE_TIMEOUT_S,
    evaluate_recording_readiness,
    evaluate_recording_outcome,
    latest_unsynchronized_disparity_difference_px,
    source_state_from_monotonic_ns,
)

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
    "latest_unsynchronized_disparity_difference_px",
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

RECEIVE_TIME_NS_FIELDS = [
    field.replace("_sec", "_ns") for field in RECEIVE_TIME_FIELDS
]

RECEIVE_MONOTONIC_NS_FIELDS = [
    "ground_truth_receive_monotonic_ns",
    "observed_receive_monotonic_ns",
    "analytic_receive_monotonic_ns",
    "corrected_receive_monotonic_ns",
    "attitude_status_receive_monotonic_ns",
    "observation_status_receive_monotonic_ns",
    "profile_status_receive_monotonic_ns",
    "px4_status_receive_monotonic_ns",
    "left_image_receive_monotonic_ns",
    "right_image_receive_monotonic_ns",
]

SOURCE_STATE_FIELDS = [
    "ground_truth_state",
    "observed_state",
    "analytic_state",
    "corrected_state",
    "attitude_status_state",
    "observation_status_state",
    "profile_status_state",
    "px4_status_state",
    "left_image_state",
    "right_image_state",
]

SOURCE_RECEIVE_AGE_FIELDS = [
    field.replace("_state", "_receive_age_sec") for field in SOURCE_STATE_FIELDS
]

FLIGHT_CONTRACT_FIELDS = [
    "flight_contract_verified",
    "flight_contract_source",
    "flight_authority",
    "fly_target_in_gazebo",
    "mission_profile",
    "actual_distance_source",
    "flight_contract_error",
]

PARAMETER_SNAPSHOT_NAMES = (
    "scenario_name",
    "target_class",
    "target_size_m",
    "baseline_m",
    "camera_height_m",
    "horizontal_fov_rad",
    "camera_image_width_px",
    "camera_image_height_px",
    "observation_mode",
    "left_roll_error_deg",
    "left_pitch_error_deg",
    "left_yaw_error_deg",
    "right_roll_error_deg",
    "right_pitch_error_deg",
    "right_yaw_error_deg",
    "target_model_name",
    "target_reference_link_name",
    "oracle_base_link_name",
    "flight_authority",
    "fly_target_in_gazebo",
    "mission_profile",
    "actual_distance_source",
    "yolo_target_hz",
    "yolo_imgsz",
)

COUNTER_FIELDS = (
    "csv_row_count",
    "ground_truth_message_count",
    "ground_truth_valid_sample_count",
    "observed_message_count",
    "observed_valid_sample_count",
    "analytic_message_count",
    "analytic_valid_sample_count",
    "corrected_message_count",
    "corrected_valid_sample_count",
    "attitude_status_message_count",
    "observation_status_message_count",
    "profile_status_message_count",
    "px4_status_message_count",
    "left_image_received_count",
    "right_image_received_count",
    "left_video_frame_count",
    "right_video_frame_count",
)

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


def _stamp_to_ns(stamp):
    """Convert a ROS stamp to int64 nanoseconds without a float round-trip."""
    if stamp is None:
        return None
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None


def _timestamp_sec_from_ns(value):
    if value in (None, ""):
        return ""
    try:
        seconds, nanoseconds = divmod(int(value), 1_000_000_000)
    except (TypeError, ValueError):
        return ""
    return f"{seconds}.{nanoseconds:09d}"


def _age_sec(now_ns, receive_ns):
    if now_ns in (None, "") or receive_ns in (None, ""):
        return None
    try:
        age = (int(now_ns) - int(receive_ns)) / 1_000_000_000.0
    except (TypeError, ValueError):
        return None
    return age if math.isfinite(age) and age >= 0.0 else None


def _finite_vector(values, length=4):
    try:
        values = list(values)[:length]
    except (TypeError, ValueError):
        return False
    if len(values) != length:
        return False
    for value in values:
        if isinstance(value, (bool, str)):
            return False
        try:
            if not math.isfinite(float(value)):
                return False
        except (TypeError, ValueError, OverflowError):
            return False
    return True


def _parameter_value(value):
    if hasattr(value, "value"):
        return value.value
    parameter_type = getattr(value, "type", None)
    if parameter_type == 1:
        return bool(getattr(value, "bool_value", False))
    if parameter_type == 2:
        return int(getattr(value, "integer_value", 0))
    if parameter_type == 3:
        return float(getattr(value, "double_value", 0.0))
    if parameter_type == 4:
        return str(getattr(value, "string_value", ""))
    return None


def _is_json_mapping(value):
    return isinstance(value, dict) and bool(value)


def _ensure_state_lock(owner):
    lock = getattr(owner, "_lock", None)
    if lock is None:
        lock = threading.RLock()
        setattr(owner, "_lock", lock)
    return lock


def _wait_process_reaped(process, timeout_s):
    """Wait boundedly and reap a child when it exits."""
    if process is None:
        return True
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while True:
        return_code = process.poll()
        if return_code is not None:
            try:
                process.wait(timeout=0)
            except (AttributeError, OSError, subprocess.TimeoutExpired, TimeoutError):
                return False
            return process.poll() is not None
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        try:
            process.wait(timeout=min(0.05, remaining))
        except AttributeError:
            return False
        except (subprocess.TimeoutExpired, TimeoutError):
            time.sleep(min(0.02, remaining))
        except OSError:
            return False


def evaluate_artifact_integrity(
    session_dir,
    mode,
    *,
    csv_row_count,
    left_video_frame_count=0,
    right_video_frame_count=0,
    bag_return_code=None,
    bag_reaped=False,
):
    """Check file/artifact completeness independently from research validity."""
    root = Path(session_dir)
    errors = []

    def require_file(name, *, nonempty=False):
        path = root / name
        if not path.is_file():
            errors.append(f"missing artifact: {name}")
        elif nonempty and path.stat().st_size <= 0:
            errors.append(f"empty artifact: {name}")

    require_file("telemetry.csv")
    require_file("manifest.txt")
    require_file("manifest.json")
    try:
        rows = int(csv_row_count)
    except (TypeError, ValueError):
        rows = 0
    if rows < 1:
        errors.append("telemetry.csv has zero rows")

    normalized_mode = str(mode).strip().lower()
    if normalized_mode in {"full", "debug"}:
        require_file("left_overlay.mp4", nonempty=True)
        require_file("right_overlay.mp4", nonempty=True)
        try:
            left_frames = int(left_video_frame_count or 0)
        except (TypeError, ValueError):
            left_frames = 0
        try:
            right_frames = int(right_video_frame_count or 0)
        except (TypeError, ValueError):
            right_frames = 0
        if left_frames < 1:
            errors.append("left_overlay.mp4 has zero recorded frames")
        if right_frames < 1:
            errors.append("right_overlay.mp4 has zero recorded frames")
    if normalized_mode == "debug":
        bag_dir = root / "rosbag"
        if not bag_dir.is_dir():
            errors.append("missing artifact: rosbag")
        metadata = bag_dir / "metadata.yaml"
        if not metadata.is_file():
            errors.append("missing artifact: rosbag/metadata.yaml")
        if bag_return_code != 0:
            errors.append(f"rosbag return code is not zero: {bag_return_code}")
        if not bag_reaped:
            errors.append("rosbag process was not reaped")
    return not errors, tuple(dict.fromkeys(errors))


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
        self._lock = threading.RLock()
        self.latest = {}
        self.last_stamp = {}
        self.last_monotonic_stamp = {}
        self.counts = {field: 0 for field in COUNTER_FIELDS}
        self._video_writer_errors = {}
        self.csv_file = None
        self.csv_writer = None
        self.video_writers = {}
        self.bag_process = None
        self._bag_stderr_file = None
        self._bag_started = False
        self._bag_return_code = None
        self._bag_reaped = False
        self._bag_error = None
        self._parameter_client = None
        self._parameter_future = None
        self._parameter_timer = None
        self._parameter_deadline = time.monotonic() + 2.0
        self._parameter_snapshot_status = "pending"
        self._parameter_snapshot_error = ""
        self._parameter_snapshot_values = {}
        self._eligibility_events = []
        self._finalize_lock = threading.Lock()
        self._finalized = False
        self._finalize_result = False
        self._readiness_at_start = self._load_readiness_at_start()
        self._manifest = self._initial_manifest()
        self.bag_storage = (
            self._resolve_bag_storage(args.bag_storage) if self.mode == "debug" else args.bag_storage
        )
        self._open_csv()
        self._write_manifest()
        self._init_parameter_snapshot()
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
            + RECEIVE_TIME_NS_FIELDS
            + RECEIVE_MONOTONIC_NS_FIELDS
            + SOURCE_STATE_FIELDS
            + SOURCE_RECEIVE_AGE_FIELDS
            + FLIGHT_CONTRACT_FIELDS
        )
        self.csv_fields = list(dict.fromkeys(fields))
        self.csv_file = path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.csv_fields)
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def _write_manifest(self):
        lines = [
            f"mode={self.mode}",
            f"recording_state={self._manifest.get('recording_state', 'in_progress')}",
            f"artifact_integrity={str(bool(self._manifest.get('artifact_integrity', False))).lower()}",
            f"formal_experiment_eligible={str(bool(self._manifest.get('formal_experiment_eligible', False))).lower()}",
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
            "snapshot_policy=latest values repeated at recorder timer; consult source receive ns/state/age",
            "timestamp_source=ROS clock receive time for receive_ros_time_ns; image header stamp for image_header_time_ns",
            "timestamp_unit=nanoseconds for *_receive_ros_time_ns and *_image_header_time_ns; seconds are fixed 9-decimal compatibility fields",
            "synchronized_measurements=false",
            "formal_cross_source_error_valid=false",
            "latest_unsynchronized_disparity_difference_policy=latest source values; not same-sample; not valid for formal error analysis",
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
            lines.extend(
                [
                    f"rosbag=rosbag",
                    f"rosbag_storage={self.bag_storage}",
                    "rosbag_stderr=rosbag.stderr.log",
                    f"rosbag_return_code={self._manifest.get('rosbag_return_code')}",
                    f"rosbag_reaped={str(bool(self._manifest.get('rosbag_reaped', False))).lower()}",
                ]
            )
        if self._manifest.get("completion_errors"):
            lines.append("completion_errors=" + " | ".join(self._manifest["completion_errors"]))
        (self.session_dir / "manifest.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
        self._write_manifest_json()

    def _initial_manifest(self):
        git_branch, git_commit, git_dirty = self._git_metadata()
        return {
            "schema_version": 2,
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
            "artifact_integrity": False,
            "formal_experiment_eligible": False,
            "eligibility_reasons": ["readiness not available at recorder start"],
            "completion_errors": [],
            "readiness_at_start": copy.deepcopy(self._readiness_at_start),
            "readiness_at_end": None,
            "eligibility_events": [],
            "counts": {field: 0 for field in COUNTER_FIELDS},
            "source_states": {},
            "source_receive_ros_time_ns": {},
            "source_receive_monotonic_ns": {},
            "source_receive_age_sec": {},
            "flight_contract": None,
            "rosbag_return_code": None,
            "rosbag_reaped": False,
            "rosbag_stderr_file": "rosbag.stderr.log" if self.mode == "debug" else None,
            "parameter_snapshot_file": "parameter_snapshot.json",
            "parameter_snapshot_status": "pending",
            "parameter_snapshot_error": "",
            "timestamp_source": {
                "receive_ros_time_ns": "ROS node clock at callback receipt",
                "receive_monotonic_ns": "monotonic clock at callback receipt; freshness authority",
                "image_header_time_ns": "sensor_msgs/Image.header.stamp",
                "time_sec": "elapsed monotonic compatibility field",
            },
            "timestamp_unit": {
                "receive_ros_time_ns": "ns",
                "receive_monotonic_ns": "ns",
                "image_header_time_ns": "ns",
                "*_time_sec": "s",
            },
            "snapshot_policy": "latest values repeated at recorder timer; consult source receive ns/state/age",
            "latest_unsynchronized_disparity_difference_policy": "latest source values; not same-sample; not valid for formal error analysis",
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
        files = [
            "telemetry.csv",
            "manifest.txt",
            "manifest.json",
            "parameter_snapshot.json",
            "readiness_at_start.json",
        ]
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

    def _load_readiness_at_start(self):
        path = self.session_dir / "readiness_at_start.json"
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError):
            return {
                "start_allowed": False,
                "formal_eligible": False,
                "reasons": ["readiness snapshot was not provided by the UI"],
            }
        if isinstance(payload, dict):
            return payload
        return {
            "start_allowed": False,
            "formal_eligible": False,
            "reasons": ["readiness snapshot was invalid"],
        }

    def _write_parameter_snapshot(self):
        payload = {
            "schema_version": 1,
            "status": self._parameter_snapshot_status,
            "error": self._parameter_snapshot_error,
            "parameter_names": list(PARAMETER_SNAPSHOT_NAMES),
            "parameters": copy.deepcopy(self._parameter_snapshot_values),
        }
        path = self.session_dir / "parameter_snapshot.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(temporary, path)
        self._manifest["parameter_snapshot_status"] = self._parameter_snapshot_status
        self._manifest["parameter_snapshot_error"] = self._parameter_snapshot_error

    def _stop_parameter_timer(self):
        timer = self._parameter_timer
        if timer is None:
            return
        try:
            if hasattr(timer, "cancel"):
                timer.cancel()
            else:
                timer.stop()
        except (AttributeError, RuntimeError):
            pass

    def _init_parameter_snapshot(self):
        self._write_parameter_snapshot()
        if GetParameters is None:
            self._parameter_snapshot_status = "unavailable"
            self._parameter_snapshot_error = "rcl_interfaces GetParameters service type unavailable"
            self._write_parameter_snapshot()
            return
        try:
            self._parameter_client = self.create_client(
                GetParameters,
                "/stereo_distance_node/get_parameters",
            )
            if self._parameter_client.service_is_ready():
                self._request_parameter_snapshot()
            self._parameter_timer = self.create_timer(0.2, self._poll_parameter_snapshot)
        except Exception as exc:  # pragma: no cover - depends on ROS graph setup.
            self._parameter_snapshot_status = "unavailable"
            self._parameter_snapshot_error = f"parameter service setup failed: {exc}"
            self._write_parameter_snapshot()

    def _request_parameter_snapshot(self):
        if self._parameter_future is not None or self._parameter_client is None:
            return
        try:
            request = GetParameters.Request()
            request.names = list(PARAMETER_SNAPSHOT_NAMES)
            self._parameter_future = self._parameter_client.call_async(request)
        except Exception as exc:
            self._parameter_snapshot_status = "unavailable"
            self._parameter_snapshot_error = f"parameter query failed: {exc}"
            self._write_parameter_snapshot()

    def _poll_parameter_snapshot(self):
        if self._parameter_snapshot_status != "pending":
            self._stop_parameter_timer()
            return
        if self._parameter_future is None and self._parameter_client is not None:
            try:
                if self._parameter_client.service_is_ready():
                    self._request_parameter_snapshot()
            except Exception:
                pass
        if self._parameter_future is not None and self._parameter_future.done():
            try:
                response = self._parameter_future.result()
                values = list(getattr(response, "values", []))
                if len(values) != len(PARAMETER_SNAPSHOT_NAMES):
                    raise RuntimeError(
                        f"parameter response length {len(values)} != {len(PARAMETER_SNAPSHOT_NAMES)}"
                    )
                decoded = {
                    name: _parameter_value(value)
                    for name, value in zip(PARAMETER_SNAPSHOT_NAMES, values)
                }
                missing = [name for name, value in decoded.items() if value is None]
                if missing:
                    raise RuntimeError("parameter values unavailable: " + ", ".join(missing))
                self._parameter_snapshot_values = decoded
                self._parameter_snapshot_status = "available"
                self._parameter_snapshot_error = ""
            except Exception as exc:
                self._parameter_snapshot_status = "unavailable"
                self._parameter_snapshot_error = str(exc)
            self._write_parameter_snapshot()
            self._stop_parameter_timer()
            return
        if time.monotonic() >= self._parameter_deadline:
            self._parameter_snapshot_status = "unavailable"
            self._parameter_snapshot_error = "parameter service unavailable or query timed out"
            self._write_parameter_snapshot()
            self._stop_parameter_timer()

    def _complete_parameter_snapshot(self):
        if self._parameter_snapshot_status == "pending":
            self._parameter_snapshot_status = "unavailable"
            self._parameter_snapshot_error = "parameter snapshot did not complete before recorder finalize"
            self._write_parameter_snapshot()

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
        stderr_path = self.session_dir / "rosbag.stderr.log"
        try:
            self._bag_stderr_file = stderr_path.open("w", encoding="utf-8")
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=self._bag_stderr_file,
                preexec_fn=os.setsid,
            )
            self._bag_started = True
            immediate = self.bag_process.poll()
            if immediate is not None:
                self._bag_return_code = immediate
                self._bag_process_wait()
                if immediate != 0:
                    self._bag_error = f"rosbag exited during startup with code {immediate}"
        except (OSError, subprocess.SubprocessError) as exc:
            self._bag_error = f"rosbag start failed: {exc}"
            self._bag_return_code = None
            self._bag_reaped = True
            if self._bag_stderr_file is not None:
                self._bag_stderr_file.close()
                self._bag_stderr_file = None

    def _bag_process_wait(self):
        process = self.bag_process
        if process is None:
            self._bag_reaped = True
            return self._bag_return_code
        self._bag_reaped = _wait_process_reaped(process, 0.2)
        self._bag_return_code = process.poll()
        return self._bag_return_code

    def _stop_debug_bag(self):
        process = self.bag_process
        if process is None:
            if self._bag_stderr_file is not None:
                self._bag_stderr_file.close()
                self._bag_stderr_file = None
            return
        if process.poll() is None:
            for stage, sig, timeout in (
                ("SIGINT", signal.SIGINT, 3.0),
                ("SIGTERM", signal.SIGTERM, 2.0),
                ("SIGKILL", signal.SIGKILL, 1.0),
            ):
                if process.poll() is not None:
                    break
                try:
                    os.killpg(process.pid, sig)
                except (ProcessLookupError, PermissionError):
                    try:
                        process.send_signal(sig)
                    except (ProcessLookupError, PermissionError):
                        pass
                if _wait_process_reaped(process, timeout):
                    break
        self._bag_process_wait()
        if not self._bag_reaped:
            self._bag_error = self._bag_error or "rosbag process was not reaped after SIGKILL"
        if self._bag_return_code not in (0, None):
            self._bag_error = self._bag_error or f"rosbag return code {self._bag_return_code}"
        if self._bag_stderr_file is not None:
            self._bag_stderr_file.close()
            self._bag_stderr_file = None

    def _state_lock(self):
        return _ensure_state_lock(self)

    def _now_ros_time_ns(self):
        return int(self.get_clock().now().nanoseconds)

    def _touch_topic(self, topic, *, value=None, count_field=None, valid=False):
        with _ensure_state_lock(self):
            if value is not None:
                self.latest[topic] = value
            self.last_stamp[topic] = self._now_ros_time_ns()
            if not hasattr(self, "last_monotonic_stamp"):
                self.last_monotonic_stamp = {}
            self.last_monotonic_stamp[topic] = time.monotonic_ns()
            if count_field:
                if not hasattr(self, "counts"):
                    self.counts = {field: 0 for field in COUNTER_FIELDS}
                self.counts[count_field] = self.counts.get(count_field, 0) + 1
                if valid:
                    valid_field = count_field.replace("_message_count", "_valid_sample_count")
                    if valid_field in COUNTER_FIELDS:
                        self.counts[valid_field] = self.counts.get(valid_field, 0) + 1

    def _float_cb(self, msg, field, topic):
        value = float(msg.data)
        with _ensure_state_lock(self):
            self.latest[field] = value
            self._touch_topic(topic, count_field=None)

    def _comparison_cb(self, msg):
        with self._state_lock():
            for field, value in zip(COMPARISON_FIELDS, msg.data):
                self.latest[field] = float(value)
            self._touch_topic("/stereo/distance_comparison")

    def _ground_truth_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        values = [float(value) for value in values]
        with self._state_lock():
            self.latest["ground_truth"] = list(values)
            self.latest["drone_forward_m"] = values[0]
            self.latest["drone_right_m"] = values[1]
            self.latest["drone_down_m"] = values[2]
            self.latest["ground_truth_m"] = values[3]
            self._touch_topic(
                "/ground_truth_position",
                count_field="ground_truth_message_count",
                valid=_finite_vector(values),
            )

    def _estimated_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        values = [float(value) for value in values]
        with self._state_lock():
            self.latest["observed_raw"] = list(values)
            self.latest["raw_forward_m"] = values[0]
            self.latest["raw_right_m"] = values[1]
            self.latest["raw_down_m"] = values[2]
            self.latest["raw_range_m"] = values[3]
            self.latest["estimated_forward_m"] = values[0]
            self.latest["estimated_right_m"] = values[1]
            self.latest["estimated_down_m"] = values[2]
            self.latest["estimated_range_m"] = values[3]
            self._touch_topic(
                "/stereo/estimated_position",
                count_field="observed_message_count",
                valid=_finite_vector(values),
            )

    def _analytic_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        values = [float(value) for value in values]
        with self._state_lock():
            self.latest["analytic_raw"] = list(values)
            for field, value in zip(
                ("analytic_forward_m", "analytic_right_m", "analytic_down_m", "analytic_range_m"),
                values,
            ):
                self.latest[field] = value
            self._touch_topic(
                "/stereo/attitude_model/raw/estimated_position",
                count_field="analytic_message_count",
                valid=_finite_vector(values),
            )

    def _pixels_cb(self, msg):
        values = [float(value) for value in list(msg.data)]
        with self._state_lock():
            self.latest["analytic_vertical_disparity_px"] = values[5] if len(values) > 5 else math.nan
            self.latest["pixels"] = list(values)
            self._touch_topic("/stereo/attitude_model/actual/pixels")

    def _ray_quality_cb(self, msg):
        values = [float(value) for value in list(msg.data)]
        with self._state_lock():
            self.latest["ray_quality"] = list(values)
            self.latest["ray_gap_m"] = values[0] if len(values) > 0 else math.nan
            self.latest["intersection_angle_rad"] = values[1] if len(values) > 1 else math.nan
            self._touch_topic("/stereo/attitude_model/raw/ray_quality")

    def _corrected_position_cb(self, msg):
        values = list(msg.data[:4])
        while len(values) < 4:
            values.append(math.nan)
        values = [float(value) for value in values]
        with self._state_lock():
            self.latest["corrected"] = list(values)
            for field, value in zip(CORRECTED_POSITION_FIELDS, values):
                self.latest[field] = value
            self._touch_topic(
                "/stereo/corrected_position",
                count_field="corrected_message_count",
                valid=_finite_vector(values),
            )

    def _bbox_adjustment_cb(self, msg):
        values = list(msg.data[: len(BBOX_ADJUSTMENT_FIELDS)])
        while len(values) < len(BBOX_ADJUSTMENT_FIELDS):
            values.append(math.nan)
        with self._state_lock():
            for field, value in zip(BBOX_ADJUSTMENT_FIELDS, values):
                self.latest[field] = float(value)
            self._touch_topic("/stereo/bbox_adjustment")

    def _detections_cb(self, msg):
        values = list(msg.data[: len(DETECTION_MESSAGE_FIELDS)])
        while len(values) < len(DETECTION_MESSAGE_FIELDS):
            values.append(math.nan)
        with self._state_lock():
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
            self._touch_topic("/stereo/detections")

    def _profile_status_cb(self, msg):
        payload = self._json_payload(msg.data)
        with self._state_lock():
            self.latest["profile_status"] = payload
            self.latest["profile_id"] = str(payload.get("profile_id", ""))
            self.latest["profile_mode"] = str(payload.get("profile_mode", ""))
            self.latest["profile_fallback_level"] = payload.get("fallback_level", math.nan)
            self.latest["profile_fallback_reason"] = str(payload.get("fallback_reason", ""))
            self._touch_topic(
                "/stereo/profile_status",
                count_field="profile_status_message_count",
            )

    @staticmethod
    def _json_payload(value):
        try:
            payload = json.loads(value)
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = {}
        return payload if isinstance(payload, dict) else {}

    def _attitude_status_cb(self, msg):
        payload = self._json_payload(msg.data)
        with self._state_lock():
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
            self._touch_topic(
                "/stereo/attitude_model/status",
                count_field="attitude_status_message_count",
            )

    def _observation_status_cb(self, msg):
        payload = self._json_payload(msg.data)
        with self._state_lock():
            self.latest["observation_status"] = payload
            for field in (
                "oracle_source_role",
                "oracle_matched_entity_name",
                "oracle_fallback_used",
            ):
                if field in payload and field not in self.latest:
                    self.latest[field] = payload[field]
            self._touch_topic(
                "/stereo/observation_status",
                count_field="observation_status_message_count",
            )

    def _px4_status_cb(self, msg):
        payload = self._json_payload(msg.data)
        with self._state_lock():
            self.latest["px4_status"] = payload
            self.latest["flight_contract"] = copy.deepcopy(payload.get("flight_contract"))
            self._touch_topic(
                "/simulation/px4/status",
                count_field="px4_status_message_count",
            )

    def _source_state_snapshot(
        self,
        latest,
        stamps,
        now_ns,
        monotonic_stamps=None,
        now_monotonic_ns=None,
    ):
        values = {
            "ground_truth": latest.get("ground_truth"),
            "observed_raw": latest.get("observed_raw"),
            "analytic_raw": latest.get("analytic_raw"),
            "corrected": latest.get("corrected"),
            "attitude_status": latest.get("attitude_status"),
            "observation_status": latest.get("observation_status"),
            "profile_status": latest.get("profile_status"),
            "px4_status": latest.get("px4_status"),
            "left_image": latest.get("left_image"),
            "right_image": latest.get("right_image"),
        }
        topics = {
            "ground_truth": "/ground_truth_position",
            "observed_raw": "/stereo/estimated_position",
            "analytic_raw": "/stereo/attitude_model/raw/estimated_position",
            "corrected": "/stereo/corrected_position",
            "attitude_status": "/stereo/attitude_model/status",
            "observation_status": "/stereo/observation_status",
            "profile_status": "/stereo/profile_status",
            "px4_status": "/simulation/px4/status",
            "left_image": "/left_camera/detection_image",
            "right_image": "/right_camera/detection_image",
        }
        states = {}
        ages = {}
        monotonic_stamps = (
            dict(getattr(self, "last_monotonic_stamp", {}))
            if monotonic_stamps is None
            else dict(monotonic_stamps)
        )
        now_monotonic_ns = time.monotonic_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
        for key, value in values.items():
            receive_ns = stamps.get(topics[key])
            receive_monotonic_ns = monotonic_stamps.get(topics[key])
            age = _age_sec(now_monotonic_ns, receive_monotonic_ns)
            ages[key] = age
            if receive_monotonic_ns in (None, ""):
                states[key] = "waiting"
            else:
                states[key] = source_state_from_monotonic_ns(
                    receive_monotonic_ns,
                    value,
                    now_monotonic_ns,
                    STALE_TIMEOUT_S,
                )
                # A malformed/rewound monotonic receipt cannot be treated as
                # current merely because the ROS clock moved forward.
                if states[key] == "valid" and age is None:
                    states[key] = "stale"
        return states, ages

    def _write_csv_row(self):
        now_ns = self._now_ros_time_ns()
        now_monotonic_ns = time.monotonic_ns()
        with self._state_lock():
            latest = copy.deepcopy(self.latest)
            stamps = dict(self.last_stamp)
            monotonic_stamps = dict(getattr(self, "last_monotonic_stamp", {}))
            counts = dict(getattr(self, "counts", {}))
        states, ages = self._source_state_snapshot(
            latest,
            stamps,
            now_ns,
            monotonic_stamps,
            now_monotonic_ns,
        )
        elapsed = time.monotonic() - self.start_monotonic
        row = {
            "time_sec": f"{elapsed:.9f}",
            "ros_time_sec": _timestamp_sec_from_ns(now_ns),
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
            + RESEARCH_FIELDS
        ):
            row[field] = _csv_value(latest.get(field, ""))

        row.update(
            {
                field: _csv_value(latest.get(field, ""))
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
        observed_disparity = latest.get("disparity_px")
        analytic_disparity = latest.get("analytic_disparity_px")
        # The two topics have no common sample timestamp. Keep the formal
        # field empty and retain only a clearly-labelled receive-time hint.
        row["disparity_difference_px"] = ""
        row["latest_unsynchronized_disparity_difference_px"] = _csv_value(
            latest_unsynchronized_disparity_difference_px(observed_disparity, analytic_disparity)
        )
        angle_rad = _finite_or_empty(latest.get("intersection_angle_rad"))
        row["intersection_angle_deg"] = (
            _finite_or_empty(math.degrees(float(angle_rad))) if angle_rad else ""
        )

        attitude = latest.get("attitude_status") or {}
        observation = latest.get("observation_status") or {}
        row["oracle_source_role"] = _csv_value(
            attitude.get("oracle_source_role")
            or observation.get("oracle_source_role")
            or latest.get("oracle_source_role", "")
        )
        row["oracle_matched_entity_name"] = _csv_value(
            attitude.get("oracle_matched_entity_name")
            or observation.get("oracle_matched_entity_name")
            or latest.get("oracle_matched_entity_name", "")
        )
        row["oracle_fallback_used"] = _csv_value(
            attitude.get(
                "oracle_fallback_used",
                observation.get("oracle_fallback_used", latest.get("oracle_fallback_used", "")),
            )
        )

        px4 = latest.get("px4_status") or {}
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
        contract = px4.get("flight_contract") if isinstance(px4.get("flight_contract"), dict) else {}
        row.update(
            {
                "flight_contract_verified": _csv_value(contract.get("verified", "")),
                "flight_contract_source": _csv_value(contract.get("source", "")),
                "flight_authority": _csv_value(contract.get("flight_authority", "")),
                "fly_target_in_gazebo": _csv_value(contract.get("fly_target_in_gazebo", "")),
                "mission_profile": _csv_value(contract.get("mission_profile", "")),
                "actual_distance_source": _csv_value(contract.get("actual_distance_source", "")),
                "flight_contract_error": _csv_value(contract.get("error", "")),
            }
        )

        receive_topics = {
            "ground_truth": "/ground_truth_position",
            "observed_raw": "/stereo/estimated_position",
            "analytic_raw": "/stereo/attitude_model/raw/estimated_position",
            "corrected": "/stereo/corrected_position",
            "attitude_status": "/stereo/attitude_model/status",
            "observation_status": "/stereo/observation_status",
            "profile_status": "/stereo/profile_status",
            "px4_status": "/simulation/px4/status",
            "left_image": "/left_camera/detection_image",
            "right_image": "/right_camera/detection_image",
        }
        for source, topic in receive_topics.items():
            prefix = {
                "ground_truth": "ground_truth",
                "observed_raw": "observed",
                "analytic_raw": "analytic",
                "corrected": "corrected",
                "attitude_status": "attitude_status",
                "observation_status": "observation_status",
                "profile_status": "profile_status",
                "px4_status": "px4_status",
                "left_image": "left_image",
                "right_image": "right_image",
            }[source]
            receive_ns = stamps.get(topic)
            row[f"{prefix}_receive_ros_time_sec"] = _timestamp_sec_from_ns(receive_ns)
            row[f"{prefix}_receive_ros_time_ns"] = "" if receive_ns in (None, "") else str(int(receive_ns))
            receive_monotonic_ns = monotonic_stamps.get(topic)
            row[f"{prefix}_receive_monotonic_ns"] = (
                "" if receive_monotonic_ns in (None, "") else str(int(receive_monotonic_ns))
            )
            row[f"{prefix}_state"] = states.get(source, "waiting").upper()
            row[f"{prefix}_receive_age_sec"] = _csv_value(ages.get(source))
        for side in ("left", "right"):
            header_ns = latest.get(f"{side}_image_header_time_ns")
            row[f"{side}_image_header_time_sec"] = _timestamp_sec_from_ns(header_ns)
            row[f"{side}_image_header_time_ns"] = "" if header_ns in (None, "") else str(int(header_ns))

        row = {
            key: value
            for key, value in row.items()
            if key in getattr(self.csv_writer, "fieldnames", row)
        }
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        with self._state_lock():
            if not hasattr(self, "counts"):
                self.counts = {field: 0 for field in COUNTER_FIELDS}
            self.counts["csv_row_count"] = self.counts.get("csv_row_count", 0) + 1

    def _image_cb(self, camera_name, msg, output_name=None):
        topic = f"/{camera_name}_camera/detection_image"
        header = getattr(msg, "header", None)
        header_ns = _stamp_to_ns(getattr(header, "stamp", None))
        with _ensure_state_lock(self):
            self.latest[f"{camera_name}_image"] = {"received": True}
            if header_ns is not None:
                self.latest[f"{camera_name}_image_header_time_ns"] = header_ns
                # Keep the legacy in-memory float for callers; CSV serialization
                # below uses the exact ns value and fixed nine-decimal text.
                self.latest[f"{camera_name}_image_header_time_sec"] = float(
                    _timestamp_sec_from_ns(header_ns)
                )
            if hasattr(self, "_touch_topic"):
                self._touch_topic(topic, count_field=f"{camera_name}_image_received_count")
            else:  # compatibility with the lightweight callback fixture
                self.last_stamp[topic] = self.get_clock().now().nanoseconds
                if not hasattr(self, "last_monotonic_stamp"):
                    self.last_monotonic_stamp = {}
                self.last_monotonic_stamp[topic] = time.monotonic_ns()
        image = self._msg_to_bgr(msg)
        if image is None:
            return
        output_name = output_name or camera_name
        with _ensure_state_lock(self):
            if not hasattr(self, "_video_writer_errors"):
                self._video_writer_errors = {}
            writer = self.video_writers.get(output_name)
            if writer is None:
                path = self.session_dir / f"{output_name}.mp4"
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                height, width = image.shape[:2]
                try:
                    writer = cv2.VideoWriter(str(path), fourcc, self.video_fps, (width, height))
                except Exception as exc:
                    self._video_writer_errors[output_name] = f"writer creation failed: {exc}"
                    self.get_logger().error(f"Failed to open MP4 writer: {path}: {exc}")
                    return
                if not writer.isOpened():
                    self._video_writer_errors[output_name] = f"writer did not open: {path}"
                    self.get_logger().error(f"Failed to open MP4 writer: {path}")
                    return
                self.video_writers[output_name] = writer
            try:
                writer.write(image)
            except Exception as exc:
                self._video_writer_errors[output_name] = f"writer write failed: {exc}"
                return
            if not hasattr(self, "counts"):
                self.counts = {field: 0 for field in COUNTER_FIELDS}
            frame_field = f"{camera_name}_video_frame_count"
            self.counts[frame_field] = self.counts.get(frame_field, 0) + 1

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

    def _load_eligibility_events(self):
        path = self.session_dir / "eligibility_events.jsonl"
        if not path.is_file():
            return []
        events = []
        try:
            for line in path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    event = {"message": line.strip()}
                events.append(event if isinstance(event, dict) else {"message": str(event)})
        except OSError as exc:
            events.append({"message": f"eligibility event log read failed: {exc}"})
        return events

    def _final_readiness(self):
        now_ns = self._now_ros_time_ns()
        now_monotonic_ns = time.monotonic_ns()
        with self._state_lock():
            latest = copy.deepcopy(self.latest)
            stamps = dict(self.last_stamp)
            monotonic_stamps = dict(getattr(self, "last_monotonic_stamp", {}))
        states, ages = self._source_state_snapshot(
            latest,
            stamps,
            now_ns,
            monotonic_stamps,
            now_monotonic_ns,
        )
        observation = latest.get("observation_status") or {}
        readiness = evaluate_recording_readiness(
            mode=self.mode,
            observation_mode=observation.get(
                "observation_mode",
                self._parameter_snapshot_values.get("observation_mode", "both"),
            ),
            source_states=states,
            px4_status=latest.get("px4_status") or {},
            observation_status=observation,
            attitude_status=latest.get("attitude_status") or {},
            profile_status=latest.get("profile_status") or {},
            parameter_snapshot_status=self._parameter_snapshot_status,
        )
        events = self._load_eligibility_events()
        reasons = list(readiness.reasons)
        if not bool(self._readiness_at_start.get("formal_eligible")):
            reasons.append("formal readiness was not asserted at recording start")
        if events:
            reasons.append("recording eligibility violation occurred")
        readiness = type(readiness)(
            readiness.start_allowed,
            readiness.formal_eligible
            and bool(self._readiness_at_start.get("formal_eligible"))
            and not events,
            tuple(dict.fromkeys(reasons)),
        )
        return readiness, states, ages, latest, events

    def _finalize_manifest(self):
        readiness, states, ages, latest, events = self._final_readiness()
        with self._state_lock():
            counts = dict(getattr(self, "counts", {}))
            writer_errors = dict(self._video_writer_errors)
        self._manifest["counts"] = {field: int(counts.get(field, 0)) for field in COUNTER_FIELDS}
        self._manifest["source_states"] = states
        self._manifest["source_receive_age_sec"] = ages
        with self._state_lock():
            receive_ros_stamps = dict(self.last_stamp)
            receive_monotonic_stamps = dict(getattr(self, "last_monotonic_stamp", {}))
        source_topics = {
            "ground_truth": "/ground_truth_position",
            "observed_raw": "/stereo/estimated_position",
            "analytic_raw": "/stereo/attitude_model/raw/estimated_position",
            "corrected": "/stereo/corrected_position",
            "attitude_status": "/stereo/attitude_model/status",
            "observation_status": "/stereo/observation_status",
            "profile_status": "/stereo/profile_status",
            "px4_status": "/simulation/px4/status",
            "left_image": "/left_camera/detection_image",
            "right_image": "/right_camera/detection_image",
        }
        self._manifest["source_receive_ros_time_ns"] = {
            key: receive_ros_stamps.get(topic) for key, topic in source_topics.items()
        }
        self._manifest["source_receive_monotonic_ns"] = {
            key: receive_monotonic_stamps.get(topic) for key, topic in source_topics.items()
        }
        self._manifest["readiness_at_end"] = readiness.as_dict()
        self._manifest["eligibility_events"] = events
        self._manifest["eligibility_reasons"] = list(readiness.reasons)
        self._manifest["flight_contract"] = copy.deepcopy((latest.get("px4_status") or {}).get("flight_contract"))
        self._manifest["rosbag_return_code"] = self._bag_return_code
        self._manifest["rosbag_reaped"] = bool(self._bag_reaped)
        if writer_errors:
            self._manifest["completion_errors"].extend(
                f"{name}: {error}" for name, error in writer_errors.items()
            )
        if self._bag_error:
            self._manifest["completion_errors"].append(self._bag_error)
        self._manifest["completion_errors"] = list(dict.fromkeys(self._manifest["completion_errors"]))
        artifact_ok, artifact_errors = evaluate_artifact_integrity(
            self.session_dir,
            self.mode,
            csv_row_count=counts.get("csv_row_count", 0),
            left_video_frame_count=counts.get("left_video_frame_count", 0),
            right_video_frame_count=counts.get("right_video_frame_count", 0),
            bag_return_code=self._bag_return_code,
            bag_reaped=self._bag_reaped,
        )
        self._manifest["completion_errors"].extend(artifact_errors)
        self._manifest["completion_errors"] = list(dict.fromkeys(self._manifest["completion_errors"]))
        outcome = evaluate_recording_outcome(artifact_ok, readiness.formal_eligible, events)
        self._manifest.update(outcome)
        self._manifest["end_time_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self._manifest["output_files"] = [
            name
            for name in self._expected_output_files()
            if (self.session_dir / name).exists()
        ]
        self._write_manifest()
        return bool(artifact_ok)

    def finalize(self) -> bool:
        """Close all artifacts once and return artifact integrity."""
        global _CLOSING
        with self._finalize_lock:
            if self._finalized:
                return bool(self._finalize_result)
            self._finalized = True
            _CLOSING = True
        try:
            self._stop_parameter_timer()
            self._complete_parameter_snapshot()
            if self.mode == "debug":
                self._stop_debug_bag()
            with self._state_lock():
                writers = list(self.video_writers.items())
                self.video_writers.clear()
            for name, writer in writers:
                try:
                    writer.release()
                except Exception as exc:
                    self._video_writer_errors[name] = f"writer release failed: {exc}"
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
                self.csv_file = None
            self._finalize_result = self._finalize_manifest()
            return bool(self._finalize_result)
        except Exception as exc:
            self._manifest["recording_state"] = "error"
            self._manifest["artifact_integrity"] = False
            self._manifest["formal_experiment_eligible"] = False
            self._manifest.setdefault("completion_errors", []).append(f"finalize failed: {exc}")
            try:
                self._write_manifest()
            except Exception:
                pass
            self._finalize_result = False
            return False

    def close(self):
        return self.finalize()


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
        success = recorder.finalize()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not success:
            raise SystemExit(2)


if __name__ == "__main__":
    main()
