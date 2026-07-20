from __future__ import annotations

import json
import math
import os
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image as ImageMsg
from std_msgs.msg import Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_FRAME,
    TRIANGULATION_METHOD,
)
from stereo_ground_sim.ray_measurement import (
    camera_model_from_camera_info,
    empty_triangulation_status,
    gazebo_world_xyz_to_baseline_center_frd,
    pixel_grid_contract_reason,
    project_nominal_stereo_point,
    triangulation_status,
    triangulate_nominal_stereo_pixels,
)


class StereoDistanceNode(Node):
    def __init__(self):
        super().__init__("stereo_distance_node")
        defaults = {
            "scenario_name": "unspecified",
            "target_class": "unspecified",
            "target_size_m": float("nan"),
            "scenario_range_min_m": float("nan"),
            "scenario_range_max_m": float("nan"),
            "zoom_index": float("nan"),
            "baseline_m": 25.0,
            "camera_height_m": 5.0,
            "horizontal_fov_rad": 0.154997710698,
            "reference_distance_m": 45.0,
            "approach_start_distance_m": 2000.0,
            "approach_end_distance_m": 300.0,
            "approach_duration_s": 120.0,
            "mission_profile": "manual",
            "takeoff_altitude_m": 10.0,
            "takeoff_duration_s": 10.0,
            "first_hover_duration_s": 5.0,
            "final_hover_duration_s": 5.0,
            "land_duration_s": 10.0,
            "target_model_name": "x500_15m",
            "target_center_z_m": 5.0,
            "target_origin_z_offset_m": 0.0,
            "fly_target_in_gazebo": False,
            "actual_distance_source": "gazebo",
            "actual_distance_read_hz": 15.0,
            "yolo_model_path": "",
            "yolo_class_name": "drone",
            "yolo_confidence": 0.25,
            "yolo_device": "cuda:0",
            "yolo_target_hz": 40.0,
            "yolo_imgsz": 1280,
            "enable_synthetic_detections": False,
            "synthetic_detection_width_px": 1280,
            "synthetic_detection_height_px": 720,
            "synthetic_detection_bbox_size_px": 32.0,
            "enable_external_bbox_detections": False,
            "external_bbox_detections_topic": "/stereo/external_detections",
            "external_bbox_detection_width_px": 1280,
            "external_bbox_detection_height_px": 720,
            "plot_publish_hz": 30.0,
        }
        dynamic_numeric = {"actual_distance_read_hz", "yolo_target_hz", "plot_publish_hz"}
        for name, default in defaults.items():
            if name in dynamic_numeric:
                self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
            else:
                self.declare_parameter(name, default)

        self.scenario_name = str(self.get_parameter("scenario_name").value)
        self.target_class = str(self.get_parameter("target_class").value)
        self.target_size_m = self._float_param("target_size_m")
        self.scenario_range_min_m = self._float_param("scenario_range_min_m")
        self.scenario_range_max_m = self._float_param("scenario_range_max_m")
        self.zoom_index = self._float_param("zoom_index")
        self.baseline_m = self._float_param("baseline_m")
        self.camera_height_m = self._float_param("camera_height_m")
        self.horizontal_fov_rad = self._float_param("horizontal_fov_rad")
        self.reference_distance_m = self._float_param("reference_distance_m")
        self.approach_start_distance_m = self._float_param("approach_start_distance_m")
        self.approach_end_distance_m = self._float_param("approach_end_distance_m")
        self.approach_duration_s = self._float_param("approach_duration_s")
        self.mission_profile = str(self.get_parameter("mission_profile").value)
        self.takeoff_altitude_m = self._float_param("takeoff_altitude_m")
        self.takeoff_duration_s = self._float_param("takeoff_duration_s")
        self.first_hover_duration_s = self._float_param("first_hover_duration_s")
        self.final_hover_duration_s = self._float_param("final_hover_duration_s")
        self.land_duration_s = self._float_param("land_duration_s")
        self.target_model_name = str(self.get_parameter("target_model_name").value)
        self.target_center_z_m = self._float_param("target_center_z_m")
        self.target_origin_z_offset_m = self._float_param("target_origin_z_offset_m")
        self.fly_target_in_gazebo = self._bool_param("fly_target_in_gazebo")
        self.actual_distance_source = str(self.get_parameter("actual_distance_source").value)
        self.actual_distance_read_hz = max(0.1, self._float_param("actual_distance_read_hz"))
        self.yolo_model_path = str(self.get_parameter("yolo_model_path").value)
        self.yolo_class_name = str(self.get_parameter("yolo_class_name").value)
        self.yolo_confidence = self._float_param("yolo_confidence")
        self.yolo_device = str(self.get_parameter("yolo_device").value)
        self.yolo_target_hz = self._float_param("yolo_target_hz")
        self.yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)
        self.enable_synthetic_detections = self._bool_param("enable_synthetic_detections")
        self.synthetic_detection_width_px = max(1, int(self.get_parameter("synthetic_detection_width_px").value))
        self.synthetic_detection_height_px = max(1, int(self.get_parameter("synthetic_detection_height_px").value))
        self.synthetic_detection_bbox_size_px = max(1.0, self._float_param("synthetic_detection_bbox_size_px"))
        self.enable_external_bbox_detections = self._bool_param("enable_external_bbox_detections")
        self.external_bbox_detections_topic = str(self.get_parameter("external_bbox_detections_topic").value)
        self.external_bbox_detection_width_px = max(1, int(self.get_parameter("external_bbox_detection_width_px").value))
        self.external_bbox_detection_height_px = max(1, int(self.get_parameter("external_bbox_detection_height_px").value))
        self.plot_publish_hz = max(1.0, self._float_param("plot_publish_hz"))

        self.left_camera_model = None
        self.right_camera_model = None
        self.left_camera_info_error = ""
        self.right_camera_info_error = ""
        self.yolo_model = None
        self.yolo_model_device = None
        self.yolo_model_loaded_path = None
        self.yolo_load_error = ""
        self.resolved_yolo_device = None
        self.yolo_device_warning_logged = False
        self.yolo_device_info_logged = False
        self.yolo_runtime_disabled = False
        self.yolo_runtime_disable_reason = ""
        self.yolo_runtime_warning_logged = False
        self.yolo_class_warning_logged = False
        self.yolo_cpu_fallback = False
        self.yolo_warmed_up = False
        self.yolo_inference_in_progress = False
        self.last_yolo_update = 0.0
        self.last_yolo_completion = None
        self.left_detection = None
        self.right_detection = None
        self.left_msg = None
        self.right_msg = None
        self.start_time = time.monotonic()
        self.last_gazebo_velocity_update = 0.0
        self.last_gazebo_pose_update = 0.0
        self.gazebo_pose_stream_process = None
        self.gazebo_pose_stream_thread = None
        self.gazebo_pose_stream_started = False
        self.actual_distance_sample_seq = 0
        self.last_published_actual_distance_seq = -1
        initial_down_m = self._truth_down_from_world_up(self.target_center_z_m)
        initial_range_m = self._actual_range_m(0.0, initial_down_m, self.approach_start_distance_m)
        self.cached_actual_distance = initial_range_m
        self.cached_ground_truth_position = {
            "right_m": 0.0,
            "down_m": initial_down_m,
            "forward_m": self.approach_start_distance_m,
            "ground_truth_m": initial_range_m,
        }
        self.metrics = self._empty_metrics(initial_range_m, "init", "waiting for detections")
        self.shutdown_event = threading.Event()
        self.lock = threading.Lock()
        self.yolo_condition = threading.Condition(self.lock)

        self.distance_pub = self.create_publisher(Float32, "/target_distance", 10)
        self.actual_distance_pub = self.create_publisher(Float32, "/ground_truth", 10)
        self.ground_truth_position_pub = self.create_publisher(Float32MultiArray, "/ground_truth_position", 10)
        self.error_pub = self.create_publisher(Float32, "/stereo/distance_error", 10)
        self.comparison_pub = self.create_publisher(Float32MultiArray, "/stereo/distance_comparison", 10)
        self.estimated_position_pub = self.create_publisher(Float32MultiArray, "/stereo/estimated_position", 10)
        self.triangulation_status_pub = self.create_publisher(String, "/stereo/triangulation_status", 10)
        self.marker_pub = self.create_publisher(Marker, "/stereo/distance_marker", 10)
        self.yolo_inference_ms_pub = self.create_publisher(Float32, "/stereo/yolo_inference_ms", 10)
        self.yolo_update_hz_pub = self.create_publisher(Float32, "/stereo/yolo_update_hz", 10)
        self.detections_pub = self.create_publisher(Float32MultiArray, "/stereo/detections", 10)

        camera_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.camera_info_subscriptions = [
            self.create_subscription(CameraInfo, "/left_camera/camera_info", self._left_camera_info_cb, camera_qos),
            self.create_subscription(CameraInfo, "/right_camera/camera_info", self._right_camera_info_cb, camera_qos),
        ]
        if self.enable_external_bbox_detections:
            self.create_subscription(Float32MultiArray, self.external_bbox_detections_topic, self._external_detections_cb, 10)

        self.image_subscriptions = []
        if self._should_subscribe_camera_images():
            self.resolved_yolo_device = self._resolve_yolo_device()
            self._load_yolo_model()
            self.image_subscriptions.extend([
                self.create_subscription(ImageMsg, "/left_camera/image", self._left_cb, camera_qos),
                self.create_subscription(ImageMsg, "/right_camera/image", self._right_cb, camera_qos),
            ])

        self.create_timer(0.2, self._publish_marker)
        if self.enable_synthetic_detections:
            synthetic_hz = self.yolo_target_hz if self.yolo_target_hz > 0.0 else self.plot_publish_hz
            self.create_timer(1.0 / max(1.0, float(synthetic_hz)), self._publish_synthetic_detection_sample)
        self.add_on_set_parameters_callback(self._on_param_update)
        self.plot_thread = threading.Thread(target=self._plot_publish_loop, daemon=True)
        self.plot_thread.start()
        self.yolo_thread = None
        if self._should_subscribe_camera_images():
            self.yolo_thread = threading.Thread(target=self._yolo_worker_loop, daemon=True)
            self.yolo_thread.start()

        self.get_logger().info(
            "Ray-only stereo node started "
            f"(method={TRIANGULATION_METHOD}, frame={BASELINE_CENTER_FRD_FRAME}, baseline={self.baseline_m:.3f} m)"
        )

    def _float_param(self, name):
            return float(self.get_parameter(name).value)

    def _bool_param(self, name):
            return self._coerce_bool(self.get_parameter(name).value)

    def _coerce_bool(self, value):
            if isinstance(value, str):
                return value.strip().lower() in ("1", "true", "yes", "on")
            return bool(value)


    def _on_param_update(self, params):
        for param in params:
            if param.name == "baseline_m":
                self.baseline_m = float(param.value)
            elif param.name == "camera_height_m":
                self.camera_height_m = float(param.value)
            elif param.name == "horizontal_fov_rad":
                self.horizontal_fov_rad = float(param.value)
            elif param.name == "reference_distance_m":
                self.reference_distance_m = float(param.value)
            elif param.name == "approach_start_distance_m":
                self.approach_start_distance_m = float(param.value)
            elif param.name == "approach_end_distance_m":
                self.approach_end_distance_m = float(param.value)
            elif param.name == "approach_duration_s":
                self.approach_duration_s = float(param.value)
            elif param.name == "mission_profile":
                self.mission_profile = str(param.value)
            elif param.name == "plot_publish_hz":
                self.plot_publish_hz = max(1.0, float(param.value))
            elif param.name == "actual_distance_read_hz":
                self.actual_distance_read_hz = max(0.1, float(param.value))
        return SetParametersResult(successful=True)

    def _camera_info_model(self, msg):
        return camera_model_from_camera_info(
            k=msg.k,
            width_px=msg.width,
            height_px=msg.height,
            distortion=msg.d,
        )

    def _left_camera_info_cb(self, msg):
        try:
            self.left_camera_model = self._camera_info_model(msg)
            self.left_camera_info_error = ""
        except ValueError as exc:
            self.left_camera_model = None
            self.left_camera_info_error = str(exc)
            self.get_logger().warning(f"Rejected left CameraInfo: {exc}")

    def _right_camera_info_cb(self, msg):
        try:
            self.right_camera_model = self._camera_info_model(msg)
            self.right_camera_info_error = ""
        except ValueError as exc:
            self.right_camera_model = None
            self.right_camera_info_error = str(exc)
            self.get_logger().warning(f"Rejected right CameraInfo: {exc}")

    def _empty_status(self, reason):
        return empty_triangulation_status(reason)

    def _status_from_attempt(self, attempt):
        return triangulation_status(attempt)

    def _camera_input_contract_reason(self, left_pixel_grid, right_pixel_grid):
        camera_info_errors = (
            self.left_camera_info_error,
            self.right_camera_info_error,
        )
        if "unsupported_camera_distortion" in camera_info_errors:
            return "unsupported_camera_distortion"
        for reason in camera_info_errors:
            if reason:
                return reason
        if self.left_camera_model is None or self.right_camera_model is None:
            return "missing_camera_info"
        return pixel_grid_contract_reason(
            left_input_grid=left_pixel_grid,
            right_input_grid=right_pixel_grid,
            left_camera=self.left_camera_model,
            right_camera=self.right_camera_model,
        )

    def _empty_metrics(self, actual_range_m, phase, status, triangulation_reason="incomplete_detection_pair"):
        return {
            "range_m": None, "estimated_right_m": None, "estimated_down_m": None,
            "estimated_forward_m": None, "estimated_range_m": None,
            "actual_range_m": float(actual_range_m), "range_error_m": None,
            "mission_phase": phase, "status": status,
            "triangulation_status": self._empty_status(triangulation_reason),
        }

    def _actual_distance(self):
        return float(self.cached_ground_truth_position["ground_truth_m"])

    def _publish_actual_distance(self):
        state = self._mission_state()
        actual_range_m = self._read_actual_distance(state["distance_m"])
        self.cached_actual_distance = actual_range_m
        self.reference_distance_m = actual_range_m
        with self.lock:
            self.metrics["actual_range_m"] = actual_range_m
            self.metrics["mission_phase"] = state["phase"]
        self.actual_distance_pub.publish(Float32(data=float(actual_range_m)))
        self._publish_ground_truth_position()
        self._publish_plot_samples()
        if self.fly_target_in_gazebo:
            self._command_gazebo_target_pose(state)

    def _publish_ground_truth_position(self):
        position = self.cached_ground_truth_position
        self.ground_truth_position_pub.publish(Float32MultiArray(data=[
            float(position["forward_m"]), float(position["right_m"]),
            float(position["down_m"]), float(position["ground_truth_m"]),
        ]))

    def _plot_publish_loop(self):
        next_tick = time.monotonic()
        while not self.shutdown_event.is_set():
            self._publish_actual_distance()
            next_tick += 1.0 / max(1.0, float(self.plot_publish_hz))
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                self.shutdown_event.wait(sleep_s)
            else:
                next_tick = time.monotonic()

    def _publish_plot_samples(self):
        with self.lock:
            metrics = dict(self.metrics)
        truth_range_m = float(metrics.get("actual_range_m", self.cached_ground_truth_position["ground_truth_m"]))
        estimated_range_m = metrics.get("estimated_range_m")
        error_m = metrics.get("range_error_m")
        estimate_value = math.nan if estimated_range_m is None else float(estimated_range_m)
        error_value = math.nan if error_m is None else float(error_m)
        self.distance_pub.publish(Float32(data=estimate_value))
        self.error_pub.publish(Float32(data=error_value))
        self._publish_estimated_position(metrics)
        self.comparison_pub.publish(Float32MultiArray(data=[truth_range_m, estimate_value, error_value]))

    def _publish_estimated_position(self, metrics):
        values = [
            metrics.get("estimated_forward_m"), metrics.get("estimated_right_m"),
            metrics.get("estimated_down_m"), metrics.get("estimated_range_m"),
        ]
        self.estimated_position_pub.publish(Float32MultiArray(
            data=[math.nan if value is None else float(value) for value in values]
        ))
        status = metrics.get("triangulation_status") or self._empty_status("missing_status")
        self.triangulation_status_pub.publish(String(
            data=json.dumps(status, ensure_ascii=False, sort_keys=True, allow_nan=False)
        ))

    def _read_actual_distance(self, fallback_forward_m):
        if self.actual_distance_source == "gazebo":
            self._ensure_gazebo_pose_stream()
            return float(self.cached_ground_truth_position["ground_truth_m"])
        down_m = self._truth_down_from_world_up(self.target_center_z_m)
        range_m = self._actual_range_m(0.0, down_m, float(fallback_forward_m))
        self.cached_ground_truth_position = {
            "right_m": 0.0, "down_m": down_m, "forward_m": float(fallback_forward_m),
            "ground_truth_m": range_m,
        }
        return range_m

    def _consume_gazebo_pose_stream(self, process):
        if process.stdout is None:
            return
        for line in process.stdout:
            if self.shutdown_event.is_set():
                return
            now = time.monotonic()
            if now - self.last_gazebo_pose_update < 1.0 / max(0.1, float(self.actual_distance_read_hz)):
                continue
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            ground_truth = self._ground_truth_from_gazebo_pose_payload(payload)
            if ground_truth is None:
                continue
            self.cached_actual_distance = float(ground_truth["ground_truth_m"])
            self.cached_ground_truth_position = ground_truth
            self.last_gazebo_pose_update = now
            self.actual_distance_sample_seq += 1

    def _distance_from_gazebo_pose_payload(self, payload):
        ground_truth = self._ground_truth_from_gazebo_pose_payload(payload)
        return None if ground_truth is None else ground_truth["ground_truth_m"]

    def _should_subscribe_camera_images(self):
            if self.yolo_target_hz <= 0.0:
                return False
            if not self.yolo_model_path.strip():
                return False
            if self.yolo_runtime_disabled:
                return False
            return True

    def _disable_yolo_runtime(self, reason):
            self.yolo_runtime_disabled = True
            self.yolo_runtime_disable_reason = str(reason)
            self.yolo_load_error = str(reason)
            if not self.yolo_runtime_warning_logged:
                self.get_logger().warning(str(reason))
                self.yolo_runtime_warning_logged = True

    def _mission_state(self):
            elapsed = time.monotonic() - self.start_time
            start = self.approach_start_distance_m
            end = self.approach_end_distance_m
            initial_z = self.target_center_z_m
            hover_z = self.takeoff_altitude_m

            if self.mission_profile == "manual":
                return {
                    "phase": "manual_qgc",
                    "distance_m": start,
                    "altitude_m": initial_z,
                    "velocity_x_mps": 0.0,
                    "velocity_z_mps": 0.0,
                }

            if self.mission_profile != "takeoff_forward_land":
                if self.approach_duration_s <= 0.0:
                    return {
                        "phase": "complete",
                        "distance_m": end,
                        "altitude_m": initial_z,
                        "velocity_x_mps": 0.0,
                        "velocity_z_mps": 0.0,
                    }
                ratio = min(max(elapsed / self.approach_duration_s, 0.0), 1.0)
                distance = start + (end - start) * ratio
                velocity_x = 0.0 if ratio >= 1.0 else (end - start) / self.approach_duration_s
                return {
                    "phase": "approach" if ratio < 1.0 else "complete",
                    "distance_m": distance,
                    "altitude_m": initial_z,
                    "velocity_x_mps": velocity_x,
                    "velocity_z_mps": 0.0,
                }

            t0 = max(self.takeoff_duration_s, 0.0)
            h1 = max(self.first_hover_duration_s, 0.0)
            fwd = max(self.approach_duration_s, 0.0)
            h2 = max(self.final_hover_duration_s, 0.0)
            land = max(self.land_duration_s, 0.0)

            if elapsed < t0 and t0 > 0.0:
                ratio = elapsed / t0
                return {
                    "phase": "takeoff",
                    "distance_m": start,
                    "altitude_m": initial_z + (hover_z - initial_z) * ratio,
                    "velocity_x_mps": 0.0,
                    "velocity_z_mps": (hover_z - initial_z) / t0,
                }
            elapsed -= t0

            if elapsed < h1:
                return {
                    "phase": "hover_before_approach",
                    "distance_m": start,
                    "altitude_m": hover_z,
                    "velocity_x_mps": 0.0,
                    "velocity_z_mps": 0.0,
                }
            elapsed -= h1

            if self.approach_duration_s <= 0.0:
                distance = end
                velocity_x = 0.0
            else:
                ratio = min(max(elapsed / fwd, 0.0), 1.0)
                distance = start + (end - start) * ratio
                velocity_x = 0.0 if ratio >= 1.0 else (end - start) / fwd
            if elapsed < fwd:
                return {
                    "phase": "approach",
                    "distance_m": distance,
                    "altitude_m": hover_z,
                    "velocity_x_mps": velocity_x,
                    "velocity_z_mps": 0.0,
                }
            elapsed -= fwd

            if elapsed < h2:
                return {
                    "phase": "hover_after_approach",
                    "distance_m": end,
                    "altitude_m": hover_z,
                    "velocity_x_mps": 0.0,
                    "velocity_z_mps": 0.0,
                }
            elapsed -= h2

            if elapsed < land and land > 0.0:
                ratio = elapsed / land
                return {
                    "phase": "landing",
                    "distance_m": end,
                    "altitude_m": hover_z + (initial_z - hover_z) * ratio,
                    "velocity_x_mps": 0.0,
                    "velocity_z_mps": (initial_z - hover_z) / land,
                }

            return {
                "phase": "complete",
                "distance_m": end,
                "altitude_m": initial_z,
                "velocity_x_mps": 0.0,
                "velocity_z_mps": 0.0,
            }

    def _ensure_gazebo_pose_stream(self):
            if self.gazebo_pose_stream_started:
                return
            self.gazebo_pose_stream_started = True
            self.gazebo_pose_stream_thread = threading.Thread(
                target=self._gazebo_pose_stream_loop,
                daemon=True,
            )
            self.gazebo_pose_stream_thread.start()

    def _gazebo_pose_stream_loop(self):
            command = [
                "gz",
                "topic",
                "-e",
                "-t",
                "/world/stereo_ground_drone/pose/info",
                "--json-output",
            ]
            while not self.shutdown_event.is_set():
                try:
                    self.gazebo_pose_stream_process = subprocess.Popen(
                        command,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL,
                        text=True,
                        bufsize=1,
                    )
                except OSError:
                    self.shutdown_event.wait(0.5)
                    continue

                process = self.gazebo_pose_stream_process
                try:
                    self._consume_gazebo_pose_stream(process)
                finally:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=1.0)
                        except subprocess.TimeoutExpired:
                            process.kill()
                    self.gazebo_pose_stream_process = None

                if not self.shutdown_event.is_set():
                    self.shutdown_event.wait(0.5)

    def _actual_range_m(self, right_m, down_m, forward_m):
            return math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)

    def _truth_down_from_world_up(self, model_origin_up_m):
            reference_up_m = float(model_origin_up_m) + float(self.target_origin_z_offset_m)
            return float(self.camera_height_m) - reference_up_m

    def _ground_truth_from_gazebo_pose_payload(self, payload):
            pose = self._find_pose_by_name(payload, self.target_model_name)
            if pose is None:
                return None
            position = pose.get("position") or pose.get("pos") or {}
            try:
                forward_m, right_m, down_m = gazebo_world_xyz_to_baseline_center_frd(
                    world_x_m=position["x"],
                    world_y_m=position["y"],
                    world_z_m=position["z"],
                    camera_height_m=self.camera_height_m,
                    target_origin_z_offset_m=self.target_origin_z_offset_m,
                )
                range_distance = self._actual_range_m(right_m, down_m, forward_m)
                return {
                    "right_m": right_m,
                    "down_m": down_m,
                    "forward_m": forward_m,
                    "ground_truth_m": range_distance,
                }
            except (KeyError, TypeError, ValueError):
                return None

    def shutdown(self):
            self.shutdown_event.set()
            process = self.gazebo_pose_stream_process
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()

    def _find_pose_by_name(self, value, name):
            if isinstance(value, dict):
                if value.get("name") == name and ("position" in value or "pos" in value):
                    return value
                for child in value.values():
                    found = self._find_pose_by_name(child, name)
                    if found is not None:
                        return found
            elif isinstance(value, list):
                for child in value:
                    found = self._find_pose_by_name(child, name)
                    if found is not None:
                        return found
            return None

    def _command_gazebo_target_pose(self, state):
            now = time.monotonic()
            if now - self.last_gazebo_velocity_update < 0.5:
                return
            self.last_gazebo_velocity_update = now
            pose_msg = (
                f'name: "{self.target_model_name}", '
                f"position: {{x: 0.0, y: {state['distance_m']:.6f}, z: {state['altitude_m']:.6f}}}, "
                "orientation: {x: 0.0, y: 0.0, z: -0.707106781, w: 0.707106781}"
            )
            try:
                subprocess.run(
                    [
                        "gz",
                        "service",
                        "-s",
                        "/world/stereo_ground_drone/set_pose",
                        "--reqtype",
                        "gz.msgs.Pose",
                        "--reptype",
                        "gz.msgs.Boolean",
                        "--timeout",
                        "1000",
                        "--req",
                        pose_msg,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5.0,
                    check=False,
                )
            except (OSError, subprocess.SubprocessError):
                self.get_logger().warning("Failed to command Gazebo target pose; will retry.")

    def _left_cb(self, msg):
            self._store_image("left", msg)

    def _right_cb(self, msg):
            self._store_image("right", msg)

    def _store_image(self, side, msg):
            with self.yolo_condition:
                if side == "left":
                    self.left_msg = msg
                else:
                    self.right_msg = msg
                self.yolo_condition.notify()

    def _yolo_worker_loop(self):
            self._warm_yolo_model()
            next_tick = time.monotonic()
            while not self.shutdown_event.is_set():
                with self.yolo_condition:
                    while (
                        (self.left_msg is None or self.right_msg is None or self.yolo_target_hz <= 0.0)
                        and not self.shutdown_event.is_set()
                    ):
                        self.yolo_condition.wait(timeout=0.1)
                    if self.shutdown_event.is_set():
                        return
                    target_hz = max(0.1, float(self.yolo_target_hz))
                    wait_s = next_tick - time.monotonic()
                    if wait_s > 0.0:
                        self.yolo_condition.wait(timeout=wait_s)
                        continue
                    left_msg = self.left_msg
                    right_msg = self.right_msg
                    actual = self._actual_distance()
                    phase = self._mission_state()["phase"]
                    self.yolo_inference_in_progress = True
                    self.last_yolo_update = time.monotonic()
                    next_tick = self.last_yolo_update + 1.0 / target_hz
                self._update_metrics_from_messages(left_msg, right_msg, actual, phase)


    def _update_metrics_from_messages(self, left_msg, right_msg, actual, phase, device=None):
        started = time.monotonic()
        try:
            left_image = self._msg_to_rgb(left_msg)
            right_image = self._msg_to_rgb(right_msg)
            if left_image is None or right_image is None:
                return
            if device is None:
                device = self.resolved_yolo_device or self._resolve_yolo_device()
            left, right = self._detect_drone_pair(left_image, right_image, device=device)
            metrics = self._calculate_metrics(
                left,
                right,
                actual,
                phase,
                left_pixel_grid=(left_image.shape[1], left_image.shape[0]),
                right_pixel_grid=(right_image.shape[1], right_image.shape[0]),
            )
            with self.lock:
                self.left_detection = left
                self.right_detection = right
                self.metrics = metrics
            self.detections_pub.publish(self._detections_to_msg(left, right))
            self._publish_plot_samples()
        finally:
            finished = time.monotonic()
            self.yolo_inference_ms_pub.publish(Float32(data=float((finished - started) * 1000.0)))
            if self.last_yolo_completion is not None:
                period_s = finished - self.last_yolo_completion
                if period_s > 0.0:
                    self.yolo_update_hz_pub.publish(Float32(data=float(1.0 / period_s)))
            self.last_yolo_completion = finished
            with self.yolo_condition:
                self.yolo_inference_in_progress = False

    def _detections_to_msg(self, left, right):
            msg = Float32MultiArray()
            msg.data = self._detection_to_values(left) + self._detection_to_values(right)
            return msg

    def _detection_from_values(self, values):
            if len(values) < 8:
                return None
            try:
                valid = float(values[0]) > 0.5
                x1 = float(values[1])
                y1 = float(values[2])
                x2 = float(values[3])
                y2 = float(values[4])
                confidence = float(values[5])
                cx = float(values[6])
                cy = float(values[7])
            except (TypeError, ValueError):
                return None
            if not valid:
                return None
            if not all(math.isfinite(value) for value in (x1, y1, x2, y2, confidence, cx, cy)):
                return None
            return {
                "bbox": (x1, y1, x2, y2),
                "confidence": confidence,
                "cx": cx,
                "cy": cy,
            }

    def _detection_to_values(self, detection):
            if detection is None or "bbox" not in detection:
                return [0.0, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan]
            x1, y1, x2, y2 = detection["bbox"]
            return [
                1.0,
                float(x1),
                float(y1),
                float(x2),
                float(y2),
                float(detection.get("confidence", math.nan)),
                float(detection.get("cx", math.nan)),
                float(detection.get("cy", math.nan)),
            ]


    def _external_detections_cb(self, msg):
        values = list(getattr(msg, "data", []))
        left = self._detection_from_values(values[:8])
        right = self._detection_from_values(values[8:16])
        state = self._mission_state()
        actual_range_m = self._read_actual_distance(state["distance_m"])
        width = int(self.external_bbox_detection_width_px)
        height = int(self.external_bbox_detection_height_px)
        external_grid = (width, height)
        metrics = self._calculate_metrics(
            left,
            right,
            actual_range_m,
            state["phase"],
            left_pixel_grid=external_grid,
            right_pixel_grid=external_grid,
        )
        reason = metrics["triangulation_status"]["reason"]
        metrics["status"] = (
            "External bbox ray triangulation"
            if reason == "ok"
            else f"External bbox rejected: {reason}"
        )
        with self.lock:
            self.left_detection = left
            self.right_detection = right
            self.metrics = metrics
        self.detections_pub.publish(self._detections_to_msg(left, right))
        self._publish_plot_samples()

    def _msg_to_rgb(self, msg):
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
                self.get_logger().warning(f"Unsupported image encoding: {msg.encoding}")
                return None

            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rows = raw.reshape((msg.height, msg.step))
            pixels = rows[:, : msg.width * channels].reshape((msg.height, msg.width, channels))

            if encoding == "rgb8":
                return pixels.copy()
            if encoding == "bgr8":
                return pixels[:, :, ::-1].copy()
            if encoding == "rgba8":
                return pixels[:, :, :3].copy()
            if encoding == "bgra8":
                return pixels[:, :, 2::-1].copy()

            return np.repeat(pixels, 3, axis=2).copy()

    def _reset_yolo_model(self):
            self.yolo_model = None
            self.yolo_model_device = None
            self.yolo_model_loaded_path = None
            self.yolo_load_error = ""
            self.yolo_runtime_disabled = False
            self.yolo_runtime_disable_reason = ""
            self.yolo_runtime_warning_logged = False
            self.yolo_class_warning_logged = False

    def _load_yolo_model(self):
            if self.yolo_runtime_disabled:
                self.yolo_load_error = self.yolo_runtime_disable_reason
                return None
            model_path = self.yolo_model_path.strip()
            if not model_path:
                self.yolo_load_error = "set yolo_model_path"
                return None
            expanded = os.path.expanduser(model_path)
            allow_named_model = "/" not in expanded and (os.altsep is None or os.altsep not in expanded)
            if not allow_named_model and not os.path.isabs(expanded):
                expanded = os.path.abspath(expanded)
            if not allow_named_model and not os.path.exists(expanded):
                self._disable_yolo_runtime(f"YOLO pt not found: {expanded}")
                return None
            model_source = model_path if allow_named_model else expanded
            if self.yolo_model is not None and self.yolo_model_loaded_path == model_source:
                return self.yolo_model
            try:
                from ultralytics import YOLO
            except Exception as exc:
                self._disable_yolo_runtime(f"ultralytics import failed: {exc}")
                return None
            try:
                self.yolo_model = YOLO(model_source)
                self.yolo_model_loaded_path = model_source
                self.yolo_load_error = ""
                self.get_logger().info(f"Loaded YOLO model: {model_source}")
            except Exception as exc:
                self._reset_yolo_model()
                self._disable_yolo_runtime(f"YOLO load failed: {exc}")
            return self.yolo_model

    def _warm_yolo_model(self):
            if self.yolo_warmed_up:
                return
            model = self._load_yolo_model()
            if model is None:
                return
            try:
                device = self._move_yolo_model(model, self.resolved_yolo_device or self._resolve_yolo_device())
                warmup_image = np.zeros((self.yolo_imgsz, self.yolo_imgsz, 3), dtype=np.uint8)
                predict_kwargs = {
                    "source": warmup_image,
                    "conf": float(self.yolo_confidence),
                    "device": device,
                    "imgsz": int(self.yolo_imgsz),
                    "verbose": False,
                }
                if device.startswith("cuda"):
                    predict_kwargs["half"] = True
                started = time.monotonic()
                model.predict(**predict_kwargs)
                elapsed_ms = (time.monotonic() - started) * 1000.0
                self.yolo_warmed_up = True
                self.get_logger().info(f"YOLO warmup completed in {elapsed_ms:.1f} ms.")
            except Exception as exc:
                self.yolo_load_error = f"YOLO warmup failed: {exc}"
                self.get_logger().warning(self.yolo_load_error)

    def _target_class_ids(self, model):
            class_name = self.yolo_class_name.strip().lower()
            if class_name in ("", "any", "all", "*"):
                return None
            names = getattr(model, "names", {}) or {}
            target_ids = []
            for class_id, name in names.items():
                if str(name).strip().lower() == class_name:
                    target_ids.append(int(class_id))
            if not target_ids:
                if not self.yolo_class_warning_logged:
                    self.get_logger().warning(
                        f"class not found in pt: {self.yolo_class_name}; using highest-confidence detections from the model."
                    )
                    self.yolo_class_warning_logged = True
                return None
            return target_ids

    def _detect_drone(self, image, device=None):
            model = self._load_yolo_model()
            if model is None:
                return None
            target_ids = self._target_class_ids(model)
            if target_ids == []:
                return None
            if device is None:
                device = self._resolve_yolo_device()
            device = self._move_yolo_model(model, device)
            predict_kwargs = {
                "source": image,
                "conf": float(self.yolo_confidence),
                "device": device,
                "imgsz": int(self.yolo_imgsz),
                "verbose": False,
            }
            if device.startswith("cuda"):
                predict_kwargs["half"] = True
            try:
                results = model.predict(**predict_kwargs)
            except Exception as exc:
                self.yolo_load_error = f"YOLO predict failed: {exc}"
                return None
            if not results:
                return None
            boxes = getattr(results[0], "boxes", None)
            if boxes is None or len(boxes) == 0:
                return None

            best = None
            for box in boxes:
                try:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
                except Exception:
                    continue
                if target_ids is not None and class_id not in target_ids:
                    continue
                x1, y1, x2, y2 = xyxy.tolist()
                if best is None or confidence > best["confidence"]:
                    best = {
                        "cx": (x1 + x2) * 0.5,
                        "cy": (y1 + y2) * 0.5,
                        "bbox": (x1, y1, x2, y2),
                        "confidence": confidence,
                        "class_id": class_id,
                    }
            return best

    def _detect_drone_pair(self, left_image, right_image, device=None):
            model = self._load_yolo_model()
            if model is None:
                return None, None
            target_ids = self._target_class_ids(model)
            if target_ids == []:
                return None, None
            if device is None:
                device = self._resolve_yolo_device()
            device = self._move_yolo_model(model, device)
            predict_kwargs = {
                "source": [left_image, right_image],
                "conf": float(self.yolo_confidence),
                "device": device,
                "imgsz": int(self.yolo_imgsz),
                "verbose": False,
            }
            if device.startswith("cuda"):
                predict_kwargs["half"] = True
            try:
                results = model.predict(**predict_kwargs)
            except Exception as exc:
                self.yolo_load_error = f"YOLO predict failed: {exc}"
                return None, None
            detections = [self._best_detection_from_result(result, target_ids) for result in results[:2]]
            while len(detections) < 2:
                detections.append(None)
            return detections[0], detections[1]

    def _best_detection_from_result(self, result, target_ids):
            boxes = getattr(result, "boxes", None)
            if boxes is None or len(boxes) == 0:
                return None

            best = None
            for box in boxes:
                try:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
                except Exception:
                    continue
                if target_ids is not None and class_id not in target_ids:
                    continue
                x1, y1, x2, y2 = xyxy.tolist()
                if best is None or confidence > best["confidence"]:
                    best = {
                        "cx": (x1 + x2) * 0.5,
                        "cy": (y1 + y2) * 0.5,
                        "bbox": (x1, y1, x2, y2),
                        "confidence": confidence,
                        "class_id": class_id,
                    }
            return best

    def _resolve_yolo_device(self):
            device = self.yolo_device.strip().lower()
            if not device:
                return "cpu"
            if device == "auto":
                try:
                    import torch

                    if torch.cuda.is_available():
                        self.yolo_cpu_fallback = False
                        return self._log_cuda_device("cuda:0", torch)
                    if not self.yolo_device_warning_logged:
                        self.get_logger().warning(
                            "YOLO device is auto, but CUDA is not available; using CPU. "
                            + self._cuda_status_message(torch)
                        )
                        self.yolo_device_warning_logged = True
                    self.yolo_cpu_fallback = True
                except Exception as exc:
                    if not self.yolo_device_warning_logged:
                        self.get_logger().warning(
                            f"YOLO device is auto, but torch CUDA check failed: {exc}; using CPU."
                        )
                        self.yolo_device_warning_logged = True
                    self.yolo_cpu_fallback = True
                    return "cpu"
            if device.startswith("cuda"):
                try:
                    import torch

                    if torch.cuda.is_available():
                        self.yolo_cpu_fallback = False
                        return self._log_cuda_device(device, torch)
                except Exception as exc:
                    if not self.yolo_device_warning_logged:
                        self.get_logger().warning(
                            f"Requested YOLO device '{device}', but torch CUDA check failed: {exc}; "
                            "falling back to CPU."
                        )
                        self.yolo_device_warning_logged = True
                    self.yolo_cpu_fallback = True
                    return "cpu"
                if not self.yolo_device_warning_logged:
                    self.get_logger().warning(
                        f"Requested YOLO device '{device}', but CUDA is not available; falling back to CPU. "
                        + self._cuda_status_message(torch)
                    )
                    self.yolo_device_warning_logged = True
                self.yolo_cpu_fallback = True
                return "cpu"
            self.yolo_cpu_fallback = device == "cpu"
            return device

    def _log_cuda_device(self, device, torch_module):
            if not self.yolo_device_info_logged:
                try:
                    torch_module.backends.cudnn.benchmark = True
                except Exception:
                    pass
                try:
                    name = torch_module.cuda.get_device_name(0)
                except Exception:
                    name = "unknown"
                self.get_logger().info(
                    f"Using YOLO CUDA device '{device}' ({name}); FP16 inference enabled."
                )
                self.yolo_device_info_logged = True
            return device

    def _move_yolo_model(self, model, device):
            if self.yolo_model_device == device:
                return device
            if not hasattr(model, "to"):
                self.yolo_model_device = device
                return device
            try:
                model.to(device)
                self.yolo_model_device = device
                return device
            except Exception as exc:
                self.yolo_load_error = f"YOLO model move to {device} failed: {exc}"
                if device.startswith("cuda") and not self.yolo_device_warning_logged:
                    self.get_logger().warning(
                        f"Could not move YOLO model to '{device}'; falling back to CPU: {exc}"
                    )
                    self.yolo_device_warning_logged = True
                try:
                    model.to("cpu")
                    self.yolo_model_device = "cpu"
                    return "cpu"
                except Exception:
                    pass
            return self.yolo_model_device or "cpu"

    def _cuda_status_message(self, torch_module):
            details = []
            try:
                details.append(f"torch={torch_module.__version__}")
            except Exception:
                pass
            try:
                details.append(f"torch_cuda={torch_module.version.cuda}")
            except Exception:
                pass
            try:
                details.append(f"device_count={torch_module.cuda.device_count()}")
            except Exception:
                pass
            if not os.path.exists("/dev/nvidiactl"):
                details.append("/dev/nvidiactl missing")
            return " ".join(details)


    def _position_from_centers(
        self,
        left_cx,
        left_cy,
        right_cx,
        right_cy,
        *,
        left_pixel_grid,
        right_pixel_grid,
    ):
        contract_reason = self._camera_input_contract_reason(left_pixel_grid, right_pixel_grid)
        if contract_reason is not None:
            return {
                "right_m": None, "down_m": None, "forward_m": None, "range_m": None,
                "triangulation_status": self._empty_status(contract_reason),
            }
        attempt = triangulate_nominal_stereo_pixels(
            left_pixel=(float(left_cx), float(left_cy)),
            right_pixel=(float(right_cx), float(right_cy)),
            left_intrinsics=self.left_camera_model.intrinsics,
            right_intrinsics=self.right_camera_model.intrinsics,
            baseline_m=self.baseline_m,
        )
        status = self._status_from_attempt(attempt)
        if attempt.result is None:
            return {
                "right_m": None, "down_m": None, "forward_m": None, "range_m": None,
                "triangulation_status": status,
            }
        forward_m, right_m, down_m = attempt.result.point_center_frd_m
        return {
            "right_m": float(right_m), "down_m": float(down_m),
            "forward_m": float(forward_m), "range_m": float(attempt.result.range_m),
            "triangulation_status": status,
        }

    def _synthetic_detection_pair_from_rdf(self, *, right_m, down_m, forward_m, width, height):
        synthetic_grid = (int(width), int(height))
        if self._camera_input_contract_reason(synthetic_grid, synthetic_grid) is not None:
            return None, None
        point_center_rdf_m = (float(right_m), float(down_m), float(forward_m))
        left_pixel, right_pixel = project_nominal_stereo_point(
            point_center_rdf_m,
            left_intrinsics=self.left_camera_model.intrinsics,
            right_intrinsics=self.right_camera_model.intrinsics,
            baseline_m=self.baseline_m,
        )
        if left_pixel is None or right_pixel is None:
            return None, None
        bbox_size = float(self.synthetic_detection_bbox_size_px)
        def detection(pixel):
            cx, cy = pixel
            half = bbox_size * 0.5
            return {
                "cx": float(cx), "cy": float(cy), "confidence": 1.0,
                "bbox": (float(cx-half), float(cy-half), float(cx+half), float(cy+half)),
            }
        return detection(left_pixel), detection(right_pixel)

    def _publish_synthetic_detection_sample(self):
        if not self.enable_synthetic_detections:
            return
        state = self._mission_state()
        actual_range_m = self._read_actual_distance(state["distance_m"])
        position = dict(self.cached_ground_truth_position)
        width = int(self.synthetic_detection_width_px)
        height = int(self.synthetic_detection_height_px)
        left, right = self._synthetic_detection_pair_from_rdf(
            right_m=position["right_m"], down_m=position["down_m"],
            forward_m=position["forward_m"], width=width, height=height,
        )
        synthetic_grid = (width, height)
        metrics = self._calculate_metrics(
            left,
            right,
            actual_range_m,
            state["phase"],
            left_pixel_grid=synthetic_grid,
            right_pixel_grid=synthetic_grid,
        )
        reason = metrics["triangulation_status"]["reason"]
        metrics["status"] = (
            "Synthetic full-pose ray projection"
            if reason == "ok"
            else f"Synthetic projection rejected: {reason}"
        )
        with self.lock:
            self.left_detection = left
            self.right_detection = right
            self.metrics = metrics
        self.detections_pub.publish(self._detections_to_msg(left, right))
        self._publish_plot_samples()

    def _calculate_metrics(
        self,
        left,
        right,
        actual_range_m,
        phase,
        *,
        left_pixel_grid,
        right_pixel_grid,
    ):
        contract_reason = self._camera_input_contract_reason(left_pixel_grid, right_pixel_grid)
        if contract_reason is not None:
            return self._empty_metrics(
                actual_range_m,
                phase,
                contract_reason,
                triangulation_reason=contract_reason,
            )
        if left is None or right is None:
            return self._empty_metrics(
                actual_range_m, phase,
                self.yolo_load_error or "Target not detected in both images",
            )
        raw = self._position_from_centers(
            left["cx"], left["cy"], right["cx"], right["cy"],
            left_pixel_grid=left_pixel_grid,
            right_pixel_grid=right_pixel_grid,
        )
        estimated_range_m = raw["range_m"]
        error_m = None if estimated_range_m is None else float(estimated_range_m) - float(actual_range_m)
        return {
            "range_m": estimated_range_m,
            "estimated_right_m": raw["right_m"],
            "estimated_down_m": raw["down_m"],
            "estimated_forward_m": raw["forward_m"],
            "estimated_range_m": estimated_range_m,
            "actual_range_m": float(actual_range_m),
            "range_error_m": error_m,
            "mission_phase": phase,
            "status": f"YOLO tracking {self.yolo_class_name}; raw ray triangulation",
            "triangulation_status": raw["triangulation_status"],
        }

    def _publish_marker(self):
        with self.lock:
            metrics = dict(self.metrics)
        estimated_range_m = metrics.get("estimated_range_m")
        actual_range_m = metrics.get("actual_range_m", self.cached_ground_truth_position["ground_truth_m"])
        error_m = metrics.get("range_error_m")
        forward_m = metrics.get("estimated_forward_m")
        right_m = metrics.get("estimated_right_m")
        down_m = metrics.get("estimated_down_m")
        status = metrics.get("triangulation_status") or self._empty_status("missing_status")
        estimate_text = "Range: --" if estimated_range_m is None else f"Range: {estimated_range_m:.2f} m"
        error_text = "Error: --" if error_m is None else f"Error: {error_m:+.2f} m"
        position_text = (
            "Estimated FRD: --" if forward_m is None or right_m is None or down_m is None
            else f"Estimated FRD: {forward_m:.2f}, {right_m:+.2f}, {down_m:+.2f} m"
        )
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stereo"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 12.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 7.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.35
        marker.color.a = 1.0
        marker.text = "\n".join([
            estimate_text, f"Actual range: {actual_range_m:.2f} m", error_text,
            position_text, f"Ray: {status['reason']}",
            f"Baseline: {self.baseline_m:.2f} m",
            f"Phase: {metrics.get('mission_phase', 'unknown')}",
            f"Updated: {time.strftime('%H:%M:%S')}",
        ])
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = StereoDistanceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
