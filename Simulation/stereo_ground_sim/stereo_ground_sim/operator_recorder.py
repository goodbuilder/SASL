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
from std_msgs.msg import Float32, Float32MultiArray, String

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
        )
        self.csv_file = path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fields)
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
            if field in ("profile_id", "profile_mode", "profile_fallback_reason"):
                row[field] = self.latest.get(field, "")
            else:
                row[field] = _finite_or_empty(self.latest.get(field))
        self.csv_writer.writerow(row)
        self.csv_file.flush()

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
