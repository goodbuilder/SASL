import json
import math
import os
import subprocess
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker

from stereo_ground_sim.reprojection_profile import (
    OriginBBoxReprojectionProfile,
    StereoCameraModel,
)
from stereo_ground_sim.runtime_guard_policy import (
    BoundedGainForwardCellGuardPolicy,
    EstimatedToTruthAffineProfile,
    EstimatedToTruthDisparityProfile,
    EstimatedToTruthSim3Profile,
)
from stereo_ground_sim.runtime_deployment_selector import (
    DisparityResidualRuntimeProfile,
    MultiBurstObservationFusion,
    RepeatedObservationFusion,
    RuntimeImageObservation,
    RuntimeMeasurementSample,
    RuntimeStereoCameraContract,
    load_observation_fusion_adapter_from_manifest,
)
from stereo_ground_sim.disparity_residual_feature_summary import feature_summary_from_arrays, feature_summary_from_runtime_rows
from stereo_ground_sim.disparity_residual_feature_selector import DisparityResidualFeatureSelector
from stereo_ground_sim.gazebo_oracle import (
    OracleStatus,
    ground_truth_from_gazebo_pose_payload as compute_gazebo_ground_truth,
    make_gazebo_world_to_baseline_transform,
    x500_oracle_source_policy,
)
from stereo_ground_sim.stereo_triangulation_adapter import (
    general_ray_shadow_not_computed,
    general_ray_shadow_summary,
    triangulate_nominal_stereo_centers,
)


ORACLE_STATIC_PARAMETERS = {
    "target_model_name",
    "target_reference_link_name",
    "oracle_base_link_name",
    "baseline_origin_world_enu_m",
    "rotation_baseline_world_row_major",
}
DEFAULT_BASELINE_ORIGIN_WORLD_ENU_M = (0.0, 0.0, 5.0)
DEFAULT_ROTATION_BASELINE_WORLD_ROW_MAJOR = (
    0.0,
    1.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -1.0,
)
ORACLE_STATIC_PARAMETER_REJECTION_REASON = (
    "oracle source and baseline frame parameters require node restart"
)


class StereoDistanceNode(Node):
    def __init__(self):
        super().__init__("stereo_distance_node")
        self.declare_parameter("scenario_name", "unspecified")
        self.declare_parameter("target_class", "unspecified")
        self.declare_parameter("target_size_m", float("nan"))
        self.declare_parameter("scenario_range_min_m", float("nan"))
        self.declare_parameter("scenario_range_max_m", float("nan"))
        self.declare_parameter("zoom_index", float("nan"))
        self.declare_parameter("baseline_m", 25.0)
        self.declare_parameter("camera_height_m", 5.0)
        self.declare_parameter("horizontal_fov_rad", 0.154997710698)
        self.declare_parameter("reference_distance_m", 45.0)
        self.declare_parameter("approach_start_distance_m", 2000.0)
        self.declare_parameter("approach_end_distance_m", 300.0)
        self.declare_parameter("approach_duration_s", 120.0)
        self.declare_parameter("mission_profile", "manual")
        self.declare_parameter("takeoff_altitude_m", 10.0)
        self.declare_parameter("takeoff_duration_s", 10.0)
        self.declare_parameter("first_hover_duration_s", 5.0)
        self.declare_parameter("final_hover_duration_s", 5.0)
        self.declare_parameter("land_duration_s", 10.0)
        self.declare_parameter("target_model_name", "x500_15m")
        self.declare_parameter("target_reference_link_name", "target_reference_link")
        self.declare_parameter("oracle_base_link_name", "base_link")
        self.declare_parameter("baseline_origin_world_enu_m", list(DEFAULT_BASELINE_ORIGIN_WORLD_ENU_M))
        self.declare_parameter(
            "rotation_baseline_world_row_major",
            list(DEFAULT_ROTATION_BASELINE_WORLD_ROW_MAJOR),
        )
        self.declare_parameter("target_center_z_m", 5.0)
        self.declare_parameter("fly_target_in_gazebo", False)
        self.declare_parameter("actual_distance_source", "gazebo")
        self.declare_parameter(
            "actual_distance_read_hz",
            15.0,
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("yolo_model_path", "")
        self.declare_parameter("yolo_class_name", "drone")
        self.declare_parameter("yolo_confidence", 0.25)
        self.declare_parameter("yolo_device", "cuda:0")
        self.declare_parameter(
            "yolo_target_hz",
            40.0,
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("yolo_imgsz", 1280)
        self.declare_parameter("enable_synthetic_detections", False)
        self.declare_parameter("synthetic_detection_width_px", 1280)
        self.declare_parameter("synthetic_detection_height_px", 720)
        self.declare_parameter("synthetic_detection_bbox_size_px", 32.0)
        self.declare_parameter("enable_external_bbox_detections", False)
        self.declare_parameter("external_bbox_detections_topic", "/stereo/external_detections")
        self.declare_parameter("external_bbox_detection_width_px", 1280)
        self.declare_parameter("external_bbox_detection_height_px", 720)
        self.declare_parameter(
            "plot_publish_hz",
            30.0,
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("enable_bbox_center_adjustment", False)
        self.declare_parameter("bbox_center_adjustment_path", "")
        self.declare_parameter("enable_observation_fusion", False)
        self.declare_parameter("observation_fusion_manifest_path", "")
        self.declare_parameter("observation_fusion_tier", "")
        self.declare_parameter("enable_disparity_residual_profile", False)
        self.declare_parameter("disparity_residual_profile_path", "")
        self.declare_parameter("enable_disparity_residual_feature_selector", False)
        self.declare_parameter("disparity_residual_feature_selector_model_path", "")
        self.declare_parameter("disparity_residual_feature_selector_catalog_path", "")
        self.declare_parameter("disparity_residual_feature_selector_window_size", 1)
        self.declare_parameter("disparity_residual_feature_selector_min_samples", 1)

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
        self.mission_profile = self.get_parameter("mission_profile").value
        self.takeoff_altitude_m = self._float_param("takeoff_altitude_m")
        self.takeoff_duration_s = self._float_param("takeoff_duration_s")
        self.first_hover_duration_s = self._float_param("first_hover_duration_s")
        self.final_hover_duration_s = self._float_param("final_hover_duration_s")
        self.land_duration_s = self._float_param("land_duration_s")
        self.target_model_name = self.get_parameter("target_model_name").value
        self.target_reference_link_name = self.get_parameter("target_reference_link_name").value
        self.oracle_base_link_name = self.get_parameter("oracle_base_link_name").value
        self.baseline_origin_world_enu_m = tuple(self.get_parameter("baseline_origin_world_enu_m").value)
        self.rotation_baseline_world_row_major = tuple(
            self.get_parameter("rotation_baseline_world_row_major").value
        )
        (
            self.gazebo_oracle_policy,
            self.gazebo_world_to_baseline_transform,
        ) = self._build_gazebo_oracle_configuration(
            target_model_name=self.target_model_name,
            target_reference_link_name=self.target_reference_link_name,
            oracle_base_link_name=self.oracle_base_link_name,
            baseline_origin_world_enu_m=self.baseline_origin_world_enu_m,
            rotation_baseline_world_row_major=self.rotation_baseline_world_row_major,
        )
        self.target_center_z_m = self._float_param("target_center_z_m")
        self.fly_target_in_gazebo = self._bool_param("fly_target_in_gazebo")
        self.actual_distance_source = self.get_parameter("actual_distance_source").value
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
        self.synthetic_detection_bbox_size_px = max(
            1.0,
            self._float_param("synthetic_detection_bbox_size_px"),
        )
        self.enable_external_bbox_detections = self._bool_param("enable_external_bbox_detections")
        self.external_bbox_detections_topic = str(self.get_parameter("external_bbox_detections_topic").value)
        self.external_bbox_detection_width_px = max(1, int(self.get_parameter("external_bbox_detection_width_px").value))
        self.external_bbox_detection_height_px = max(
            1,
            int(self.get_parameter("external_bbox_detection_height_px").value),
        )
        self.plot_publish_hz = max(1.0, self._float_param("plot_publish_hz"))
        self.enable_bbox_center_adjustment = self._bool_param("enable_bbox_center_adjustment")
        self.bbox_center_adjustment_path = str(self.get_parameter("bbox_center_adjustment_path").value)
        self.bbox_center_adjustment = self._load_bbox_center_adjustment()
        self.enable_observation_fusion = self._bool_param("enable_observation_fusion")
        self.observation_fusion_manifest_path = str(self.get_parameter("observation_fusion_manifest_path").value)
        self.observation_fusion_tier = str(self.get_parameter("observation_fusion_tier").value)
        self.observation_fusion_adapter = self._load_observation_fusion_adapter()
        self.observation_fusion_buffer = deque(
            maxlen=self._observation_fusion_required_observations(self.observation_fusion_adapter)
        )
        self.enable_disparity_residual_profile = self._bool_param("enable_disparity_residual_profile")
        self.disparity_residual_profile_path = str(self.get_parameter("disparity_residual_profile_path").value)
        self.disparity_residual_profile = self._load_disparity_residual_profile()
        self.enable_disparity_residual_feature_selector = self._bool_param("enable_disparity_residual_feature_selector")
        self.disparity_residual_feature_selector_model_path = str(
            self.get_parameter("disparity_residual_feature_selector_model_path").value
        )
        self.disparity_residual_feature_selector_catalog_path = str(
            self.get_parameter("disparity_residual_feature_selector_catalog_path").value
        )
        self.disparity_residual_feature_selector_window_size = max(
            1,
            int(self.get_parameter("disparity_residual_feature_selector_window_size").value),
        )
        self.disparity_residual_feature_selector_min_samples = max(
            1,
            int(self.get_parameter("disparity_residual_feature_selector_min_samples").value),
        )
        self.disparity_residual_feature_selector_window = deque(
            maxlen=self.disparity_residual_feature_selector_window_size
        )
        self.disparity_residual_feature_selector = self._load_disparity_residual_feature_selector()
        self.disparity_residual_feature_selected_label = ""
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
        self.start_time = time.monotonic()
        self.last_gazebo_velocity_update = 0.0
        self.last_gazebo_pose_sample_attempt = 0.0
        self.last_gazebo_pose_update = 0.0
        self.gazebo_pose_read_in_progress = False
        self.gazebo_pose_stream_process = None
        self.gazebo_pose_stream_thread = None
        self.gazebo_pose_stream_started = False
        self.actual_distance_sample_seq = 0
        self.last_published_actual_distance_seq = 0
        self.cached_actual_distance = self.approach_start_distance_m
        self.cached_ground_truth_position = {
            "right_m": 0.0,
            "down_m": self.camera_height_m - self.target_center_z_m,
            "forward_m": self.approach_start_distance_m,
            "ground_truth_m": self._actual_range_m(
                0.0,
                self.camera_height_m - self.target_center_z_m,
                self.approach_start_distance_m,
            ),
        }
        self.has_valid_gazebo_ground_truth = False
        self.cached_ground_truth_metadata = self._initial_gazebo_oracle_metadata()
        self.shutdown_event = threading.Event()

        self.left_msg = None
        self.right_msg = None
        self.metrics = {
            "distance_m": None,
            "disparity_px": None,
            "estimated_right_m": None,
            "estimated_down_m": None,
            "estimated_forward_m": None,
            "estimated_range_m": None,
            "corrected_right_m": None,
            "corrected_down_m": None,
            "corrected_forward_m": None,
            "corrected_range_m": None,
            "corrected_disparity_px": None,
            "bbox_adjustment": None,
            "profile_status": {
                "profile_id": "",
                "profile_mode": "",
                "fallback_level": 3,
                "fallback_reason": "init",
            },
            "observation_fusion_status": "disabled",
            "general_ray_shadow": general_ray_shadow_not_computed(
                status="not_computed_init",
            ),
            "actual_distance_m": self.approach_start_distance_m,
            "distance_error_m": None,
            "mission_phase": "init",
            "status": "waiting for images",
        }
        self.lock = threading.Lock()
        self.yolo_condition = threading.Condition(self.lock)
        if self._should_subscribe_camera_images():
            self.resolved_yolo_device = self._resolve_yolo_device()
            self._load_yolo_model()

        self.distance_pub = self.create_publisher(Float32, "/target_distance", 10)
        self.actual_distance_pub = self.create_publisher(Float32, "/ground_truth", 10)
        self.ground_truth_position_pub = self.create_publisher(
            Float32MultiArray, "/ground_truth_position", 10
        )
        self.disparity_pub = self.create_publisher(Float32, "/stereo/disparity", 10)
        self.error_pub = self.create_publisher(Float32, "/stereo/distance_error", 10)
        self.comparison_pub = self.create_publisher(Float32MultiArray, "/stereo/distance_comparison", 10)
        self.estimated_position_pub = self.create_publisher(
            Float32MultiArray, "/stereo/estimated_position", 10
        )
        self.corrected_position_pub = self.create_publisher(
            Float32MultiArray, "/stereo/corrected_position", 10
        )
        self.bbox_adjustment_pub = self.create_publisher(
            Float32MultiArray, "/stereo/bbox_adjustment", 10
        )
        self.profile_status_pub = self.create_publisher(String, "/stereo/profile_status", 10)
        self.marker_pub = self.create_publisher(Marker, "/stereo/distance_marker", 10)
        self.yolo_inference_ms_pub = self.create_publisher(
            Float32, "/stereo/yolo_inference_ms", 10
        )
        self.yolo_update_hz_pub = self.create_publisher(
            Float32, "/stereo/yolo_update_hz", 10
        )
        self.detections_pub = self.create_publisher(
            Float32MultiArray, "/stereo/detections", 10
        )
        if self.enable_external_bbox_detections:
            self.create_subscription(
                Float32MultiArray,
                self.external_bbox_detections_topic,
                self._external_detections_cb,
                10,
            )
            self.get_logger().info(
                "External bbox detections enabled on "
                f"{self.external_bbox_detections_topic}; message contract is "
                "Float32MultiArray [left_valid,x1,y1,x2,y2,conf,cx,cy,"
                "right_valid,x1,y1,x2,y2,conf,cx,cy]."
            )
        camera_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.image_subscriptions = []
        if self._should_subscribe_camera_images():
            self.image_subscriptions.extend(
                [
                    self.create_subscription(
                        ImageMsg, "/left_camera/image", self._left_cb, camera_qos
                    ),
                    self.create_subscription(
                        ImageMsg, "/right_camera/image", self._right_cb, camera_qos
                    ),
                ]
            )
        else:
            disabled_reason = self.yolo_runtime_disable_reason or self.yolo_load_error
            if self.enable_synthetic_detections:
                self.get_logger().info(
                    "YOLO image subscriptions disabled; synthetic detection mode is enabled "
                    "for headless/operator pipeline validation."
                )
            else:
                self.get_logger().warning(
                    "YOLO image subscriptions disabled because yolo_target_hz is 0 "
                    "or no usable yolo_model_path/runtime is configured."
                    + (f" ({disabled_reason})" if disabled_reason else "")
                )
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
            "Stereo distance node started "
            f"(baseline={self.baseline_m:.2f} m, fov={self.horizontal_fov_rad:.3f} rad, "
            f"target={self.approach_start_distance_m:.1f}->{self.approach_end_distance_m:.1f} m)"
        )

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    def _bool_param(self, name):
        return self._coerce_bool(self.get_parameter(name).value)

    def _coerce_bool(self, value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def _finite_float_sequence(values, *, field_name, expected_length):
        if isinstance(values, (str, bytes)) or isinstance(values, bool):
            raise ValueError(f"{field_name} must be a numeric sequence")
        try:
            items = tuple(values)
        except TypeError as exc:
            raise ValueError(f"{field_name} must be a numeric sequence") from exc
        if len(items) != expected_length:
            raise ValueError(f"{field_name} must contain exactly {expected_length} values")
        parsed = []
        for value in items:
            if isinstance(value, bool):
                raise ValueError(f"{field_name} must not contain bool values")
            try:
                number = float(value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{field_name} must contain finite numeric values") from exc
            if not math.isfinite(number):
                raise ValueError(f"{field_name} must contain only finite values")
            parsed.append(number)
        return tuple(parsed)

    @staticmethod
    def _non_empty_string(value, *, field_name):
        if not isinstance(value, str) or not value:
            raise ValueError(f"{field_name} must be a non-empty string")
        return value

    @staticmethod
    def _build_gazebo_oracle_configuration(
        *,
        target_model_name,
        target_reference_link_name,
        oracle_base_link_name,
        baseline_origin_world_enu_m,
        rotation_baseline_world_row_major,
    ):
        model_name = StereoDistanceNode._non_empty_string(
            target_model_name,
            field_name="target_model_name",
        )
        target_reference_name = StereoDistanceNode._non_empty_string(
            target_reference_link_name,
            field_name="target_reference_link_name",
        )
        base_link_name = StereoDistanceNode._non_empty_string(
            oracle_base_link_name,
            field_name="oracle_base_link_name",
        )
        origin = StereoDistanceNode._finite_float_sequence(
            baseline_origin_world_enu_m,
            field_name="baseline_origin_world_enu_m",
            expected_length=3,
        )
        rotation_values = StereoDistanceNode._finite_float_sequence(
            rotation_baseline_world_row_major,
            field_name="rotation_baseline_world_row_major",
            expected_length=9,
        )
        rotation = (
            rotation_values[0:3],
            rotation_values[3:6],
            rotation_values[6:9],
        )
        policy = x500_oracle_source_policy(
            model_name=model_name,
            target_reference_link_name=target_reference_name,
            base_link_name=base_link_name,
        )
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=origin,
            rotation_baseline_world=rotation,
        )
        return policy, transform

    @staticmethod
    def _initial_gazebo_oracle_metadata():
        return {
            "oracle_valid": False,
            "oracle_has_last_valid": False,
            "oracle_status": "not_computed_init",
            "oracle_source_role": None,
            "oracle_matched_entity_name": None,
            "oracle_match_kind": None,
            "oracle_source_priority": None,
            "oracle_fallback_used": False,
            "oracle_fallback_reason": "",
            "oracle_source_frame": "gazebo_world_enu",
            "oracle_output_frame": "baseline_center_frd",
            "oracle_last_sample_monotonic_s": None,
            "oracle_last_valid_monotonic_s": None,
        }

    @staticmethod
    def _enum_value_or_none(value):
        if value is None:
            return None
        return getattr(value, "value", str(value))

    def _gazebo_oracle_metadata_from_result(self, result, *, now):
        last_valid = None
        if hasattr(self, "cached_ground_truth_metadata"):
            last_valid = self.cached_ground_truth_metadata.get("oracle_last_valid_monotonic_s")
        return {
            "oracle_valid": bool(result.valid),
            "oracle_has_last_valid": bool(self.has_valid_gazebo_ground_truth or result.valid),
            "oracle_status": self._enum_value_or_none(result.status),
            "oracle_source_role": self._enum_value_or_none(result.source_role),
            "oracle_matched_entity_name": result.matched_entity_name,
            "oracle_match_kind": self._enum_value_or_none(result.match_kind),
            "oracle_source_priority": result.source_priority,
            "oracle_fallback_used": bool(result.fallback_used),
            "oracle_fallback_reason": str(result.fallback_reason),
            "oracle_source_frame": "gazebo_world_enu",
            "oracle_output_frame": "baseline_center_frd",
            "oracle_last_sample_monotonic_s": float(now),
            "oracle_last_valid_monotonic_s": float(now) if result.valid else last_valid,
        }

    def _gazebo_oracle_metadata_from_status(self, status, *, now):
        metadata = dict(getattr(self, "cached_ground_truth_metadata", self._initial_gazebo_oracle_metadata()))
        metadata.update(
            {
                "oracle_valid": False,
                "oracle_has_last_valid": bool(getattr(self, "has_valid_gazebo_ground_truth", False)),
                "oracle_status": self._enum_value_or_none(status),
                "oracle_last_sample_monotonic_s": float(now),
            }
        )
        return metadata

    def _ground_truth_position_from_oracle_result(self, result):
        if not result.valid or result.point_baseline_center_frd_m is None:
            return None
        forward_m, right_m, down_m = result.point_baseline_center_frd_m.xyz_m
        range_m = self._actual_range_m(right_m, down_m, forward_m)
        return {
            "right_m": float(right_m),
            "down_m": float(down_m),
            "forward_m": float(forward_m),
            "ground_truth_m": float(range_m),
        }

    def _comparison_actual_and_error_for_publish(self, *, metrics, ground_truth_metadata):
        if self.actual_distance_source == "gazebo" and ground_truth_metadata.get("oracle_valid") is not True:
            return math.nan, math.nan
        actual = metrics.get("actual_distance_m")
        if actual is None:
            actual = self._actual_distance()
        error = metrics.get("distance_error_m")
        actual_value = math.nan if actual is None else float(actual)
        error_value = math.nan if error is None else float(error)
        return actual_value, error_value

    def _on_param_update(self, params):
        for param in params:
            if param.name in ORACLE_STATIC_PARAMETERS:
                return SetParametersResult(
                    successful=False,
                    reason=ORACLE_STATIC_PARAMETER_REJECTION_REASON,
                )
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
            elif param.name == "takeoff_altitude_m":
                self.takeoff_altitude_m = float(param.value)
            elif param.name == "takeoff_duration_s":
                self.takeoff_duration_s = float(param.value)
            elif param.name == "first_hover_duration_s":
                self.first_hover_duration_s = float(param.value)
            elif param.name == "final_hover_duration_s":
                self.final_hover_duration_s = float(param.value)
            elif param.name == "land_duration_s":
                self.land_duration_s = float(param.value)
            elif param.name == "target_center_z_m":
                self.target_center_z_m = float(param.value)
            elif param.name == "fly_target_in_gazebo":
                self.fly_target_in_gazebo = self._coerce_bool(param.value)
            elif param.name == "actual_distance_source":
                self.actual_distance_source = str(param.value)
            elif param.name == "actual_distance_read_hz":
                self.actual_distance_read_hz = max(0.1, float(param.value))
            elif param.name == "yolo_model_path":
                self.yolo_model_path = str(param.value)
                self._reset_yolo_model()
            elif param.name == "yolo_class_name":
                self.yolo_class_name = str(param.value)
                self.yolo_class_warning_logged = False
            elif param.name == "yolo_confidence":
                self.yolo_confidence = float(param.value)
            elif param.name == "yolo_device":
                self.yolo_device = str(param.value)
                self.yolo_device_warning_logged = False
                self.yolo_cpu_fallback = False
                self.resolved_yolo_device = None
                self._reset_yolo_model()
            elif param.name == "yolo_target_hz":
                self.yolo_target_hz = max(0.0, float(param.value))
            elif param.name == "yolo_imgsz":
                self.yolo_imgsz = max(64, int(param.value))
            elif param.name == "enable_synthetic_detections":
                self.enable_synthetic_detections = self._coerce_bool(param.value)
            elif param.name == "synthetic_detection_width_px":
                self.synthetic_detection_width_px = max(1, int(param.value))
            elif param.name == "synthetic_detection_height_px":
                self.synthetic_detection_height_px = max(1, int(param.value))
            elif param.name == "synthetic_detection_bbox_size_px":
                self.synthetic_detection_bbox_size_px = max(1.0, float(param.value))
            elif param.name == "enable_external_bbox_detections":
                self.enable_external_bbox_detections = self._coerce_bool(param.value)
            elif param.name == "external_bbox_detections_topic":
                self.external_bbox_detections_topic = str(param.value)
            elif param.name == "external_bbox_detection_width_px":
                self.external_bbox_detection_width_px = max(1, int(param.value))
            elif param.name == "external_bbox_detection_height_px":
                self.external_bbox_detection_height_px = max(1, int(param.value))
            elif param.name == "plot_publish_hz":
                self.plot_publish_hz = max(1.0, float(param.value))
            elif param.name == "enable_bbox_center_adjustment":
                self.enable_bbox_center_adjustment = self._coerce_bool(param.value)
                self.bbox_center_adjustment = self._load_bbox_center_adjustment()
            elif param.name == "bbox_center_adjustment_path":
                self.bbox_center_adjustment_path = str(param.value)
                self.bbox_center_adjustment = self._load_bbox_center_adjustment()
            elif param.name == "enable_observation_fusion":
                self.enable_observation_fusion = self._coerce_bool(param.value)
                self.observation_fusion_adapter = self._load_observation_fusion_adapter()
                self._reset_observation_fusion_buffer()
            elif param.name == "observation_fusion_manifest_path":
                self.observation_fusion_manifest_path = str(param.value)
                self.observation_fusion_adapter = self._load_observation_fusion_adapter()
                self._reset_observation_fusion_buffer()
            elif param.name == "observation_fusion_tier":
                self.observation_fusion_tier = str(param.value)
                self.observation_fusion_adapter = self._load_observation_fusion_adapter()
                self._reset_observation_fusion_buffer()
            elif param.name == "enable_disparity_residual_profile":
                self.enable_disparity_residual_profile = self._coerce_bool(param.value)
                self.disparity_residual_profile = self._load_disparity_residual_profile()
            elif param.name == "disparity_residual_profile_path":
                self.disparity_residual_profile_path = str(param.value)
                self.disparity_residual_profile = self._load_disparity_residual_profile()
            elif param.name == "enable_disparity_residual_feature_selector":
                self.enable_disparity_residual_feature_selector = self._coerce_bool(param.value)
                self.disparity_residual_feature_selector = self._load_disparity_residual_feature_selector()
            elif param.name == "disparity_residual_feature_selector_model_path":
                self.disparity_residual_feature_selector_model_path = str(param.value)
                self.disparity_residual_feature_selector = self._load_disparity_residual_feature_selector()
            elif param.name == "disparity_residual_feature_selector_catalog_path":
                self.disparity_residual_feature_selector_catalog_path = str(param.value)
                self.disparity_residual_feature_selector = self._load_disparity_residual_feature_selector()
            elif param.name == "disparity_residual_feature_selector_window_size":
                self.disparity_residual_feature_selector_window_size = max(1, int(param.value))
                self._reset_disparity_residual_feature_selector_window()
            elif param.name == "disparity_residual_feature_selector_min_samples":
                self.disparity_residual_feature_selector_min_samples = max(1, int(param.value))
        return SetParametersResult(successful=True)

    def _observation_fusion_required_observations(self, adapter):
        if isinstance(adapter, RepeatedObservationFusion):
            return max(1, int(adapter.min_observations))
        if isinstance(adapter, MultiBurstObservationFusion):
            return max(1, int(adapter.min_bursts) * int(adapter.min_observations_per_burst))
        return 1

    def _reset_observation_fusion_buffer(self):
        self.observation_fusion_buffer = deque(
            maxlen=self._observation_fusion_required_observations(self.observation_fusion_adapter)
        )

    def _load_observation_fusion_adapter(self):
        if not self.enable_observation_fusion:
            return None
        path = str(self.observation_fusion_manifest_path or "").strip()
        if not path:
            self.get_logger().warning("observation fusion enabled but no manifest path was provided.")
            return None
        if not os.path.exists(path):
            self.get_logger().warning(f"observation fusion manifest not found: {path}")
            return None
        camera = RuntimeStereoCameraContract(
            width_px=1280.0,
            height_px=720.0,
            horizontal_fov_rad=float(self.horizontal_fov_rad),
            baseline_m=float(self.baseline_m),
        )
        tier = str(self.observation_fusion_tier or "").strip() or None
        try:
            adapter, tier_payload = load_observation_fusion_adapter_from_manifest(
                path,
                tier=tier,
                camera=camera,
            )
        except Exception as exc:
            self.get_logger().warning(f"failed to load observation fusion manifest {path}: {exc}")
            return None
        self.get_logger().info(
            "Loaded observation fusion manifest: "
            f"{path} (tier={tier_payload.get('tier', tier or 'default')}, "
            f"adapter={type(adapter).__name__})"
        )
        return adapter

    def _load_disparity_residual_profile(self):
        if not getattr(self, "enable_disparity_residual_profile", False):
            return None
        path = str(getattr(self, "disparity_residual_profile_path", "") or "").strip()
        if not path:
            self.get_logger().warning("disparity residual profile enabled but no profile path was provided.")
            return None
        if not os.path.exists(path):
            self.get_logger().warning(f"disparity residual profile not found: {path}")
            return None
        try:
            profile = DisparityResidualRuntimeProfile.from_json_file(path)
        except Exception as exc:
            self.get_logger().warning(f"failed to load disparity residual profile {path}: {exc}")
            return None
        self.get_logger().info(
            "Loaded disparity residual profile: "
            f"{path} (profile_id={profile.profile_id}, method={profile.method})"
        )
        return profile

    def _load_disparity_residual_feature_selector(self):
        if not getattr(self, "enable_disparity_residual_feature_selector", False):
            return None
        model_path = str(getattr(self, "disparity_residual_feature_selector_model_path", "") or "").strip()
        catalog_path = str(getattr(self, "disparity_residual_feature_selector_catalog_path", "") or "").strip()
        if not model_path or not catalog_path:
            self.get_logger().warning("disparity residual feature selector enabled but model/catalog path is missing.")
            return None
        if not os.path.exists(model_path):
            self.get_logger().warning(f"disparity residual feature selector model not found: {model_path}")
            return None
        if not os.path.exists(catalog_path):
            self.get_logger().warning(f"disparity residual feature selector catalog not found: {catalog_path}")
            return None
        try:
            selector = DisparityResidualFeatureSelector(
                selector_model_json=model_path,
                catalog_json=catalog_path,
            )
        except Exception as exc:
            self.get_logger().warning(f"failed to load disparity residual feature selector: {exc}")
            return None
        self.get_logger().info(f"Loaded disparity residual feature selector: {model_path}")
        return selector

    def _reset_disparity_residual_feature_selector_window(self):
        self.disparity_residual_feature_selector_window = deque(
            maxlen=max(1, int(getattr(self, "disparity_residual_feature_selector_window_size", 1)))
        )

    def _load_bbox_center_adjustment(self):
        if not self.enable_bbox_center_adjustment:
            return None
        path = str(self.bbox_center_adjustment_path or "").strip()
        if not path:
            self.get_logger().warning("bbox center adjustment enabled but no matrix path was provided.")
            return None
        if not os.path.exists(path):
            self.get_logger().warning(f"bbox center adjustment matrix not found: {path}")
            return None
        if path.lower().endswith(".json"):
            return self._load_bbox_output_profile(path)
        try:
            matrix = np.loadtxt(path, delimiter=",", dtype=float)
        except Exception as exc:
            self.get_logger().warning(f"failed to load bbox center adjustment matrix {path}: {exc}")
            return None
        if matrix.shape != (6, 4):
            self.get_logger().warning(
                f"bbox center adjustment matrix must be 6x4, got {matrix.shape}: {path}"
            )
            return None
        self.get_logger().info(f"Loaded bbox center adjustment matrix: {path}")
        return matrix

    def _load_bbox_output_profile(self, path):
        try:
            with open(path, "r", encoding="utf-8") as handle:
                profile = json.load(handle)
        except Exception as exc:
            self.get_logger().warning(f"failed to load bbox output profile {path}: {exc}")
            return None
        mode = profile.get("mode")
        if mode == OriginBBoxReprojectionProfile.mode:
            try:
                loaded = OriginBBoxReprojectionProfile.from_json_dict(profile)
            except Exception as exc:
                self.get_logger().warning(f"failed to load origin bbox reprojection profile {path}: {exc}")
                return None
            self.get_logger().info(
                f"Loaded origin bbox reprojection profile: {path} "
                f"({len(loaded.cells)} cells, policy={loaded.lookup_policy})"
            )
            return loaded
        if mode == BoundedGainForwardCellGuardPolicy.mode:
            try:
                loaded = BoundedGainForwardCellGuardPolicy.from_json_dict(profile)
            except Exception as exc:
                self.get_logger().warning(f"failed to load bounded gain guard profile {path}: {exc}")
                return None
            self.get_logger().info(
                f"Loaded bounded gain guard profile: {path} "
                f"({len(loaded.down_cells)} Down cells, family={loaded.profile_family})"
            )
            return loaded
        if mode == EstimatedToTruthSim3Profile.mode:
            try:
                loaded = EstimatedToTruthSim3Profile.from_json_dict(profile)
            except Exception as exc:
                self.get_logger().warning(f"failed to load estimated-to-truth Sim(3) profile {path}: {exc}")
                return None
            self.get_logger().info(
                f"Loaded estimated-to-truth Sim(3) profile: {path} "
                f"(profile_id={loaded.profile_id})"
            )
            return loaded
        if mode == EstimatedToTruthDisparityProfile.mode:
            try:
                loaded = EstimatedToTruthDisparityProfile.from_json_dict(profile)
            except Exception as exc:
                self.get_logger().warning(f"failed to load estimated-to-truth disparity profile {path}: {exc}")
                return None
            self.get_logger().info(
                f"Loaded estimated-to-truth disparity profile: {path} "
                f"(profile_id={loaded.profile_id})"
            )
            return loaded
        if mode == EstimatedToTruthAffineProfile.mode:
            try:
                loaded = EstimatedToTruthAffineProfile.from_json_dict(profile)
            except Exception as exc:
                self.get_logger().warning(f"failed to load estimated-to-truth affine RDF profile {path}: {exc}")
                return None
            self.get_logger().info(
                f"Loaded estimated-to-truth affine RDF profile: {path} "
                f"(profile_id={loaded.profile_id})"
            )
            return loaded
        if mode == "origin_bbox_anchor":
            anchor = profile.get("anchor")
            if not isinstance(anchor, dict):
                self.get_logger().warning(f"bbox anchor profile has no anchor object: {path}")
                return None
            required = ("left_x_fraction", "right_x_fraction", "left_y_fraction", "right_y_fraction")
            if any(name not in anchor for name in required):
                self.get_logger().warning(f"bbox anchor profile anchor is incomplete: {path}")
                return None
            loaded = dict(profile)
            loaded["anchor"] = {name: float(anchor[name]) for name in required}
            self.get_logger().info(f"Loaded origin bbox anchor profile: {path}")
            return loaded
        if mode != "origin_bbox_output_ridge":
            self.get_logger().warning(
                f"unsupported bbox output profile mode {profile.get('mode')!r}: {path}"
            )
            return None
        feature_names = profile.get("feature_names")
        coefficients = np.array(profile.get("coefficients", []), dtype=float)
        if not isinstance(feature_names, list) or not feature_names:
            self.get_logger().warning(f"bbox output profile has no feature_names: {path}")
            return None
        if coefficients.shape != (3, len(feature_names)):
            self.get_logger().warning(
                "bbox output profile coefficients must be 3 x feature_count, "
                f"got {coefficients.shape}: {path}"
            )
            return None
        feature_mean = np.array(profile.get("feature_mean", [0.0] * len(feature_names)), dtype=float)
        feature_scale = np.array(profile.get("feature_scale", [1.0] * len(feature_names)), dtype=float)
        if feature_mean.shape != (len(feature_names),) or feature_scale.shape != (len(feature_names),):
            self.get_logger().warning(f"bbox output profile mean/scale length mismatch: {path}")
            return None
        feature_scale = np.where(np.abs(feature_scale) < 1.0e-12, 1.0, feature_scale)
        blend = np.array(profile.get("blend", [1.0, 1.0, 1.0]), dtype=float)
        if blend.shape != (3,):
            self.get_logger().warning(f"bbox output profile blend must have 3 values: {path}")
            return None
        loaded = dict(profile)
        loaded["coefficients"] = coefficients
        loaded["feature_mean"] = feature_mean
        loaded["feature_scale"] = feature_scale
        loaded["blend"] = blend
        self.get_logger().info(
            f"Loaded origin bbox output profile: {path} ({len(feature_names)} features)"
        )
        return loaded

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

    def _actual_distance(self):
        return self.cached_actual_distance

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

    def _publish_actual_distance(self):
        state = self._mission_state()
        actual = self._read_actual_distance(state["distance_m"])
        if self.actual_distance_source == "gazebo":
            actual_to_publish = None
            position_to_publish = None
            with self.lock:
                self.reference_distance_m = actual
                self.metrics["actual_distance_m"] = actual
                self.metrics["mission_phase"] = state["phase"]
                metadata = dict(getattr(self, "cached_ground_truth_metadata", {}))
                if (
                    metadata.get("oracle_valid") is True
                    and self.actual_distance_sample_seq != self.last_published_actual_distance_seq
                ):
                    self.last_published_actual_distance_seq = self.actual_distance_sample_seq
                    position_to_publish = dict(self.cached_ground_truth_position)
                    actual_to_publish = float(position_to_publish["forward_m"])
            if position_to_publish is not None:
                self.actual_distance_pub.publish(Float32(data=actual_to_publish))
                self._publish_ground_truth_position(position_to_publish)
        else:
            self.cached_actual_distance = actual
            self.reference_distance_m = actual
            self.actual_distance_pub.publish(Float32(data=float(actual)))
            self.cached_ground_truth_position = {
                "right_m": 0.0,
                "down_m": self.camera_height_m - self.target_center_z_m,
                "forward_m": float(actual),
                "ground_truth_m": self._actual_range_m(
                    0.0,
                    self.camera_height_m - self.target_center_z_m,
                    float(actual),
                ),
            }
            with self.lock:
                self.metrics["actual_distance_m"] = actual
                self.metrics["mission_phase"] = state["phase"]
            self._publish_ground_truth_position(dict(self.cached_ground_truth_position))
        self._publish_plot_samples()
        if self.fly_target_in_gazebo:
            self._command_gazebo_target_pose(state)

    def _publish_ground_truth_position(self, position=None):
        position = dict(position if position is not None else self.cached_ground_truth_position)
        self.ground_truth_position_pub.publish(
            Float32MultiArray(
                data=[
                    float(position["forward_m"]),
                    float(position["right_m"]),
                    float(position["down_m"]),
                    float(position["ground_truth_m"]),
                ]
            )
        )

    def _plot_publish_loop(self):
        next_tick = time.monotonic()
        while not self.shutdown_event.is_set():
            self._publish_actual_distance()
            period_s = 1.0 / max(1.0, float(self.plot_publish_hz))
            next_tick += period_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                self.shutdown_event.wait(sleep_s)
            else:
                next_tick = time.monotonic()

    def _publish_plot_samples(self):
        with self.lock:
            metrics = dict(self.metrics)
            ground_truth_metadata = dict(
                getattr(self, "cached_ground_truth_metadata", self._initial_gazebo_oracle_metadata())
            )

        distance = metrics.get("distance_m")
        disparity = metrics.get("disparity_px")
        estimated_right = metrics.get("estimated_right_m")
        estimated_down = metrics.get("estimated_down_m")
        estimated_forward = metrics.get("estimated_forward_m")
        estimated_range = metrics.get("estimated_range_m")
        corrected_right = metrics.get("corrected_right_m")
        corrected_down = metrics.get("corrected_down_m")
        corrected_forward = metrics.get("corrected_forward_m")
        corrected_range = metrics.get("corrected_range_m")
        bbox_adjustment = metrics.get("bbox_adjustment")

        distance_value = math.nan if distance is None else float(distance)
        disparity_value = math.nan if disparity is None else float(disparity)
        actual_value, error_value = self._comparison_actual_and_error_for_publish(
            metrics=metrics,
            ground_truth_metadata=ground_truth_metadata,
        )
        estimated_position_values = [
            math.nan if estimated_forward is None else float(estimated_forward),
            math.nan if estimated_right is None else float(estimated_right),
            math.nan if estimated_down is None else float(estimated_down),
            math.nan if estimated_range is None else float(estimated_range),
        ]
        corrected_position_values = [
            math.nan if corrected_forward is None else float(corrected_forward),
            math.nan if corrected_right is None else float(corrected_right),
            math.nan if corrected_down is None else float(corrected_down),
            math.nan if corrected_range is None else float(corrected_range),
        ]

        self.distance_pub.publish(Float32(data=distance_value))
        self.error_pub.publish(Float32(data=error_value))
        self.disparity_pub.publish(Float32(data=disparity_value))
        self.estimated_position_pub.publish(Float32MultiArray(data=estimated_position_values))
        self.corrected_position_pub.publish(Float32MultiArray(data=corrected_position_values))
        if bbox_adjustment is not None:
            self.bbox_adjustment_pub.publish(self._bbox_adjustment_to_msg(bbox_adjustment))
        self.profile_status_pub.publish(self._profile_status_to_msg(metrics))
        self.comparison_pub.publish(
            Float32MultiArray(
                data=[
                    actual_value,
                    distance_value,
                    error_value,
                    disparity_value,
                ]
            )
        )

    def _read_actual_distance(self, fallback):
        if self.actual_distance_source != "gazebo":
            self.cached_actual_distance = fallback
            return fallback
        self._ensure_gazebo_pose_stream()
        return self.cached_actual_distance

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

    def _consume_gazebo_pose_stream(self, process):
        if process.stdout is None:
            return
        for line in process.stdout:
            if self.shutdown_event.is_set():
                return
            now = time.monotonic()
            read_period_s = 1.0 / max(0.1, float(self.actual_distance_read_hz))
            with self.lock:
                if now - self.last_gazebo_pose_sample_attempt < read_period_s:
                    continue
                self.last_gazebo_pose_sample_attempt = now
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                with self.lock:
                    self.cached_ground_truth_metadata = self._gazebo_oracle_metadata_from_status(
                        OracleStatus.MALFORMED_POSE_PAYLOAD,
                        now=now,
                    )
                continue
            oracle_result = self._gazebo_oracle_result_from_payload(payload)
            if not oracle_result.valid:
                with self.lock:
                    self.cached_ground_truth_metadata = self._gazebo_oracle_metadata_from_result(
                        oracle_result,
                        now=now,
                    )
                continue
            ground_truth = self._ground_truth_position_from_oracle_result(oracle_result)
            if ground_truth is None:
                with self.lock:
                    self.cached_ground_truth_metadata = self._gazebo_oracle_metadata_from_status(
                        OracleStatus.UNSUPPORTED_POSE_FORMAT,
                        now=now,
                    )
                continue
            with self.lock:
                self.cached_actual_distance = ground_truth["forward_m"]
                self.cached_ground_truth_position = dict(ground_truth)
                self.last_gazebo_pose_update = now
                self.actual_distance_sample_seq += 1
                self.has_valid_gazebo_ground_truth = True
                self.cached_ground_truth_metadata = self._gazebo_oracle_metadata_from_result(
                    oracle_result,
                    now=now,
                )
                self.last_published_actual_distance_seq = self.actual_distance_sample_seq
                publish_position = dict(self.cached_ground_truth_position)
                publish_forward = float(publish_position["forward_m"])
            self.actual_distance_pub.publish(Float32(data=publish_forward))
            self._publish_ground_truth_position(publish_position)

    def _gazebo_oracle_result_from_payload(self, payload):
        return compute_gazebo_ground_truth(
            payload,
            self.gazebo_oracle_policy,
            self.gazebo_world_to_baseline_transform,
        )

    def _distance_from_gazebo_pose_payload(self, payload):
        ground_truth = self._ground_truth_from_gazebo_pose_payload(payload)
        if ground_truth is None:
            return None
        return ground_truth["forward_m"]

    def _actual_range_m(self, right_m, down_m, forward_m):
        return math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)

    def _ground_truth_from_gazebo_pose_payload(self, payload):
        result = self._gazebo_oracle_result_from_payload(payload)
        return self._ground_truth_position_from_oracle_result(result)

    def shutdown(self):
        self.shutdown_event.set()
        process = self.gazebo_pose_stream_process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()

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
            metrics = self._calculate_metrics(left_image, left, right, actual, phase)
            with self.lock:
                ground_truth_metadata = dict(
                    getattr(self, "cached_ground_truth_metadata", self._initial_gazebo_oracle_metadata())
                )
                comparison_actual, comparison_error = self._comparison_actual_and_error_for_publish(
                    metrics=metrics,
                    ground_truth_metadata=ground_truth_metadata,
                )
                metrics = dict(metrics)
                if self.actual_distance_source == "gazebo" and ground_truth_metadata.get("oracle_valid") is not True:
                    metrics["actual_distance_m"] = comparison_actual
                    metrics["distance_error_m"] = comparison_error
                self.left_detection = left
                self.right_detection = right
                self.metrics = metrics
            self.detections_pub.publish(self._detections_to_msg(left, right))

            disparity = metrics.get("disparity_px")
            distance = metrics.get("distance_m")
            error = metrics.get("distance_error_m")
            if distance is not None:
                self.distance_pub.publish(Float32(data=float(distance)))
            if error is not None:
                self.error_pub.publish(Float32(data=comparison_error))
            if disparity is not None:
                self.disparity_pub.publish(Float32(data=float(disparity)))
            self._publish_estimated_position(metrics)
            if distance is not None and error is not None and disparity is not None:
                self.comparison_pub.publish(
                    Float32MultiArray(
                        data=[
                            comparison_actual,
                            float(distance),
                            comparison_error,
                            float(disparity),
                        ]
                    )
                )
        finally:
            finished = time.monotonic()
            elapsed_ms = (finished - started) * 1000.0
            self.yolo_inference_ms_pub.publish(Float32(data=float(elapsed_ms)))
            if self.last_yolo_completion is not None:
                period_s = finished - self.last_yolo_completion
                if period_s > 0.0:
                    self.yolo_update_hz_pub.publish(Float32(data=float(1.0 / period_s)))
            self.last_yolo_completion = finished
            with self.yolo_condition:
                self.yolo_inference_in_progress = False

    def _publish_estimated_position(self, metrics):
        self.estimated_position_pub.publish(
            Float32MultiArray(
                data=[
                    math.nan
                    if metrics.get("estimated_forward_m") is None
                    else float(metrics["estimated_forward_m"]),
                    math.nan
                    if metrics.get("estimated_right_m") is None
                    else float(metrics["estimated_right_m"]),
                    math.nan
                    if metrics.get("estimated_down_m") is None
                    else float(metrics["estimated_down_m"]),
                    math.nan
                    if metrics.get("estimated_range_m") is None
                    else float(metrics["estimated_range_m"]),
                ]
            )
        )
        self.corrected_position_pub.publish(
            Float32MultiArray(
                data=[
                    math.nan
                    if metrics.get("corrected_forward_m") is None
                    else float(metrics["corrected_forward_m"]),
                    math.nan
                    if metrics.get("corrected_right_m") is None
                    else float(metrics["corrected_right_m"]),
                    math.nan
                    if metrics.get("corrected_down_m") is None
                    else float(metrics["corrected_down_m"]),
                    math.nan
                    if metrics.get("corrected_range_m") is None
                    else float(metrics["corrected_range_m"]),
                ]
            )
        )
        bbox_adjustment = metrics.get("bbox_adjustment")
        if bbox_adjustment is not None:
            self.bbox_adjustment_pub.publish(self._bbox_adjustment_to_msg(bbox_adjustment))
        self.profile_status_pub.publish(self._profile_status_to_msg(metrics))

    def _bbox_adjustment_to_msg(self, adjustment):
        return Float32MultiArray(
            data=[
                float(adjustment["left_cx"]),
                float(adjustment["left_cy"]),
                float(adjustment["right_cx"]),
                float(adjustment["right_cy"]),
                float(adjustment["left_du"]),
                float(adjustment["left_dv"]),
                float(adjustment["right_du"]),
                float(adjustment["right_dv"]),
                float(adjustment.get("profile_id_hash", 0.0)),
                float(adjustment.get("fallback_level", math.nan)),
                float(adjustment.get("sample_count", math.nan)),
                float(adjustment.get("raw_disparity_px", math.nan)),
                float(adjustment.get("raw_range_m", math.nan)),
            ]
        )

    def _profile_status_to_msg(self, metrics):
        status = dict(metrics.get("profile_status") or self._profile_status())
        adapter = getattr(self, "observation_fusion_adapter", None)
        residual_profile = getattr(self, "disparity_residual_profile", None)
        status["observation_fusion_status"] = str(metrics.get("observation_fusion_status", "disabled"))
        status["observation_fusion_adapter"] = "" if adapter is None else type(adapter).__name__
        status["observation_fusion_buffer_count"] = len(getattr(self, "observation_fusion_buffer", []))
        status["observation_fusion_required_count"] = self._observation_fusion_required_observations(adapter)
        status["disparity_residual_status"] = str(metrics.get("disparity_residual_status", "disabled"))
        status["disparity_residual_profile_id"] = "" if residual_profile is None else residual_profile.profile_id
        status["disparity_residual_feature_summary_status"] = str(
            metrics.get("disparity_residual_feature_summary_status", "disabled")
        )
        status["disparity_residual_feature_selector_summary_status"] = str(
            metrics.get("disparity_residual_feature_selector_summary_status", "disabled")
        )
        status["disparity_residual_feature_selector_window_count"] = int(
            metrics.get("disparity_residual_feature_selector_window_count", 0)
        )
        status["disparity_residual_feature_selector_status"] = str(
            metrics.get("disparity_residual_feature_selector_status", "disabled")
        )
        status["disparity_residual_feature_selected_label"] = str(
            metrics.get("disparity_residual_feature_selected_label", "")
        )
        return String(data=json.dumps(status, ensure_ascii=False, sort_keys=True))

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
        actual = self._read_actual_distance(state["distance_m"])
        width = int(getattr(self, "external_bbox_detection_width_px", 1280))
        height = int(getattr(self, "external_bbox_detection_height_px", 720))
        synthetic_image = np.zeros((height, width, 3), dtype=np.uint8)
        metrics = self._calculate_metrics(synthetic_image, left, right, float(actual), state["phase"])
        metrics["status"] = "External bbox tracking"
        with self.lock:
            self.left_detection = left
            self.right_detection = right
            self.metrics = metrics
        self.detections_pub.publish(self._detections_to_msg(left, right))
        self._publish_estimated_position(metrics)

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

    def _position_from_centers(self, left_cx, left_cy, right_cx, right_cy, width, height):
        disparity = abs(float(left_cx) - float(right_cx))
        fx = (float(width) * 0.5) / math.tan(self.horizontal_fov_rad * 0.5)
        fy = fx
        image_cx = float(width) * 0.5
        image_cy = float(height) * 0.5
        target_cx = (float(left_cx) + float(right_cx)) * 0.5
        target_cy = (float(left_cy) + float(right_cy)) * 0.5
        if disparity < 0.5:
            return {
                "right_m": None,
                "down_m": None,
                "forward_m": None,
                "range_m": None,
                "distance_m": None,
                "disparity_px": disparity,
            }
        forward_m = fx * self.baseline_m / disparity
        right_m = (target_cx - image_cx) * forward_m / fx
        down_m = (target_cy - image_cy) * forward_m / fy
        range_m = math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)
        return {
            "right_m": right_m,
            "down_m": down_m,
            "forward_m": forward_m,
            "range_m": range_m,
            "distance_m": forward_m,
            "disparity_px": disparity,
        }

    def _synthetic_detection_pair_from_rdf(self, *, right_m, down_m, forward_m, width, height):
        if not math.isfinite(float(forward_m)) or float(forward_m) <= 0.0:
            return None, None
        width = float(width)
        height = float(height)
        fx = (width * 0.5) / math.tan(self.horizontal_fov_rad * 0.5)
        fy = fx
        image_cx = width * 0.5
        image_cy = height * 0.5
        forward = float(forward_m)
        disparity = fx * float(self.baseline_m) / forward
        if not math.isfinite(disparity) or disparity < 0.5:
            return None, None
        center_u = image_cx + float(right_m) * fx / forward
        center_v = image_cy + float(down_m) * fy / forward
        left_cx = center_u + disparity * 0.5
        right_cx = center_u - disparity * 0.5
        left_cy = center_v
        right_cy = center_v
        if not all(math.isfinite(value) for value in (left_cx, right_cx, left_cy, right_cy)):
            return None, None
        bbox_size = float(self.synthetic_detection_bbox_size_px)

        def detection(cx, cy):
            half = bbox_size * 0.5
            return {
                "cx": float(cx),
                "cy": float(cy),
                "confidence": 1.0,
                "bbox": (
                    float(cx - half),
                    float(cy - half),
                    float(cx + half),
                    float(cy + half),
                ),
            }

        return detection(left_cx, left_cy), detection(right_cx, right_cy)

    def _publish_synthetic_detection_sample(self):
        if not self.enable_synthetic_detections:
            return
        state = self._mission_state()
        actual = self._read_actual_distance(state["distance_m"])
        self.cached_actual_distance = actual
        position = dict(self.cached_ground_truth_position)
        if not math.isfinite(float(position.get("forward_m", math.nan))):
            position["forward_m"] = float(actual)
        width = int(self.synthetic_detection_width_px)
        height = int(self.synthetic_detection_height_px)
        left, right = self._synthetic_detection_pair_from_rdf(
            right_m=float(position.get("right_m", 0.0)),
            down_m=float(position.get("down_m", self.camera_height_m - self.target_center_z_m)),
            forward_m=float(position.get("forward_m", actual)),
            width=width,
            height=height,
        )
        synthetic_image = np.zeros((height, width, 3), dtype=np.uint8)
        metrics = self._calculate_metrics(synthetic_image, left, right, float(actual), state["phase"])
        metrics["status"] = "Synthetic bbox tracking for headless/operator pipeline validation"
        with self.lock:
            self.left_detection = left
            self.right_detection = right
            self.metrics = metrics
        self.detections_pub.publish(self._detections_to_msg(left, right))
        self._publish_estimated_position(metrics)

    def _raw_from_runtime_sample(self, sample):
        raw_range = (
            float(sample.raw_range_m)
            if sample.raw_range_m is not None and math.isfinite(float(sample.raw_range_m))
            else math.sqrt(
                sample.raw_right_m * sample.raw_right_m
                + sample.raw_down_m * sample.raw_down_m
                + sample.raw_forward_m * sample.raw_forward_m
            )
        )
        raw = {
            "right_m": float(sample.raw_right_m),
            "down_m": float(sample.raw_down_m),
            "forward_m": float(sample.raw_forward_m),
            "range_m": raw_range,
            "distance_m": float(sample.raw_forward_m),
            "disparity_px": float(sample.measured_disparity_px),
        }
        if sample.left_u_px is not None:
            raw["left_u_px"] = float(sample.left_u_px)
            raw["right_u_px"] = float(sample.left_u_px) - float(sample.measured_disparity_px)
        if sample.left_v_px is not None:
            raw["left_v_px"] = float(sample.left_v_px)
            raw["right_v_px"] = float(sample.left_v_px)
        return raw

    def _apply_observation_fusion(self, left, right, width, height):
        adapter = getattr(self, "observation_fusion_adapter", None)
        if adapter is None:
            return None, "disabled"
        disparity = abs(float(left["cx"]) - float(right["cx"]))
        if disparity < 0.5:
            return None, "invalid_disparity"
        observation = RuntimeImageObservation.from_mapping(
            {
                "left_u_px": float(left["cx"]),
                "left_v_px": float(left["cy"]),
                "measured_disparity_px": disparity,
            }
        )
        if observation is None:
            return None, "invalid_observation"
        self.observation_fusion_buffer.append(observation)
        adapter.camera = RuntimeStereoCameraContract(
            width_px=float(width),
            height_px=float(height),
            horizontal_fov_rad=float(self.horizontal_fov_rad),
            baseline_m=float(self.baseline_m),
        )
        observations = list(self.observation_fusion_buffer)
        if isinstance(adapter, RepeatedObservationFusion):
            fused = adapter.fuse(observations)
        elif isinstance(adapter, MultiBurstObservationFusion):
            per_burst = int(adapter.min_observations_per_burst)
            required = int(adapter.min_bursts) * per_burst
            recent = observations[-required:]
            bursts = [recent[index : index + per_burst] for index in range(0, len(recent), per_burst)]
            fused = adapter.fuse(bursts)
        else:
            return None, "unsupported_adapter"
        if fused is None:
            return None, f"warming_up:{len(observations)}/{self._observation_fusion_required_observations(adapter)}"
        return self._raw_from_runtime_sample(fused), f"fused:{type(adapter).__name__}:{len(observations)}"

    def _adjust_bbox_centers(self, left, right):
        matrix = self.bbox_center_adjustment
        if matrix is None or not isinstance(matrix, np.ndarray):
            return None
        left_vec = np.array([float(left["cx"]), float(left["cy"]), 1.0])
        right_vec = np.array([float(right["cx"]), float(right["cy"]), 1.0])
        left_adjusted = matrix[0:3, 0:3] @ left_vec + matrix[0:3, 3]
        right_adjusted = matrix[3:6, 0:3] @ right_vec + matrix[3:6, 3]
        return {
            "left_cx": float(left_adjusted[0]),
            "left_cy": float(left_adjusted[1]),
            "right_cx": float(right_adjusted[0]),
            "right_cy": float(right_adjusted[1]),
            "left_du": float(left_adjusted[0] - left_vec[0]),
            "left_dv": float(left_adjusted[1] - left_vec[1]),
            "right_du": float(right_adjusted[0] - right_vec[0]),
            "right_dv": float(right_adjusted[1] - right_vec[1]),
            "profile_id_hash": 0.0,
            "fallback_level": 0.0,
            "sample_count": 0.0,
            "raw_disparity_px": math.nan,
            "raw_range_m": math.nan,
        }

    def _camera_model(self, width, height):
        return StereoCameraModel(
            width_px=float(width),
            height_px=float(height),
            horizontal_fov_rad=float(self.horizontal_fov_rad),
            baseline_m=float(self.baseline_m),
        )

    def _profile_status(self, profile_id="", profile_mode="", fallback_level=3, fallback_reason="no_profile"):
        return {
            "profile_id": str(profile_id or ""),
            "profile_mode": str(profile_mode or ""),
            "fallback_level": int(fallback_level),
            "fallback_reason": str(fallback_reason or ""),
        }

    def _apply_bbox_reprojection_profile(self, left, right, raw, width, height):
        profile = self.bbox_center_adjustment
        if not isinstance(profile, OriginBBoxReprojectionProfile):
            return None, None, self._profile_status()
        if raw["disparity_px"] is None or raw["range_m"] is None:
            result = profile.apply(
                left.get("cx", math.nan) if left else math.nan,
                left.get("cy", math.nan) if left else math.nan,
                right.get("cx", math.nan) if right else math.nan,
                right.get("cy", math.nan) if right else math.nan,
                raw_disparity_px=raw.get("disparity_px"),
                raw_range_m=raw.get("range_m"),
            )
            return None, None, self._profile_status(
                result.profile_id,
                profile.mode,
                result.fallback_level,
                result.fallback_reason,
            )
        result = profile.apply(
            float(left["cx"]),
            float(left["cy"]),
            float(right["cx"]),
            float(right["cy"]),
            raw_disparity_px=raw["disparity_px"],
            raw_range_m=raw["range_m"],
        )
        adjustment = {
            "left_cx": result.corrected_left_u,
            "left_cy": result.corrected_left_v,
            "right_cx": result.corrected_right_u,
            "right_cy": result.corrected_right_v,
            "left_du": result.du_left,
            "left_dv": result.dv_left,
            "right_du": result.du_right,
            "right_dv": result.dv_right,
            "profile_id_hash": float(profile.profile_id_hash),
            "fallback_level": float(result.fallback_level),
            "sample_count": float(result.sample_count),
            "raw_disparity_px": float(raw["disparity_px"]),
            "raw_range_m": float(raw["range_m"]),
        }
        corrected = self._position_from_centers(
            result.corrected_left_u,
            result.corrected_left_v,
            result.corrected_right_u,
            result.corrected_right_v,
            width,
            height,
        )
        return corrected, adjustment, self._profile_status(
            result.profile_id,
            profile.mode,
            result.fallback_level,
            result.fallback_reason,
        )

    def _bbox_feature_base(self, left, right, raw, width, height):
        left_x1, left_y1, left_x2, left_y2 = left["bbox"]
        right_x1, right_y1, right_x2, right_y2 = right["bbox"]
        left_width = max(0.0, float(left_x2) - float(left_x1))
        left_height = max(0.0, float(left_y2) - float(left_y1))
        right_width = max(0.0, float(right_x2) - float(right_x1))
        right_height = max(0.0, float(right_y2) - float(right_y1))
        left_cx = float(left["cx"])
        left_cy = float(left["cy"])
        right_cx = float(right["cx"])
        right_cy = float(right["cy"])
        disparity = left_cx - right_cx
        abs_disparity = abs(disparity)
        width = float(width)
        height = float(height)
        center_x_norm = (((left_cx + right_cx) * 0.5) / width) - 0.5
        center_y_norm = (((left_cy + right_cy) * 0.5) / height) - 0.5
        base = {
            "bias": 1.0,
            "estimated_forward_m": raw["forward_m"],
            "estimated_right_m": raw["right_m"],
            "estimated_down_m": raw["down_m"],
            "estimated_range_m": raw["range_m"],
            "left_x1_norm": float(left_x1) / width,
            "left_y1_norm": float(left_y1) / height,
            "left_x2_norm": float(left_x2) / width,
            "left_y2_norm": float(left_y2) / height,
            "right_x1_norm": float(right_x1) / width,
            "right_y1_norm": float(right_y1) / height,
            "right_x2_norm": float(right_x2) / width,
            "right_y2_norm": float(right_y2) / height,
            "left_cx_norm": left_cx / width,
            "left_cy_norm": left_cy / height,
            "right_cx_norm": right_cx / width,
            "right_cy_norm": right_cy / height,
            "center_x_norm": center_x_norm,
            "center_y_norm": center_y_norm,
            "signed_disparity_norm": disparity / width,
            "disparity_norm": abs_disparity / width,
            "width_over_disparity": width / max(abs_disparity, 1.0e-6),
            "left_width_norm": left_width / width,
            "left_height_norm": left_height / height,
            "right_width_norm": right_width / width,
            "right_height_norm": right_height / height,
            "avg_width_norm": ((left_width + right_width) * 0.5) / width,
            "avg_height_norm": ((left_height + right_height) * 0.5) / height,
            "width_ratio": left_width / max(right_width, 1.0e-6),
            "height_ratio": left_height / max(right_height, 1.0e-6),
            "left_confidence": float(left.get("confidence", 0.0)),
            "right_confidence": float(right.get("confidence", 0.0)),
            "confidence_min": min(float(left.get("confidence", 0.0)), float(right.get("confidence", 0.0))),
        }
        return base

    def _bbox_feature_value(self, name, base):
        if name in base:
            return float(base[name])
        if name.endswith("^2"):
            return self._bbox_feature_value(name[:-2], base) ** 2
        if "*" in name:
            value = 1.0
            for part in name.split("*"):
                value *= self._bbox_feature_value(part, base)
            return float(value)
        raise KeyError(name)

    def _apply_bbox_output_profile(self, left, right, raw, width, height):
        profile = self.bbox_center_adjustment
        if not isinstance(profile, dict):
            return None
        if profile.get("mode") == "origin_bbox_anchor":
            anchor = profile["anchor"]
            left_x1, left_y1, left_x2, left_y2 = left["bbox"]
            right_x1, right_y1, right_x2, right_y2 = right["bbox"]
            left_cx = float(left_x1) + anchor["left_x_fraction"] * (float(left_x2) - float(left_x1))
            right_cx = float(right_x1) + anchor["right_x_fraction"] * (float(right_x2) - float(right_x1))
            left_cy = float(left_y1) + anchor["left_y_fraction"] * (float(left_y2) - float(left_y1))
            right_cy = float(right_y1) + anchor["right_y_fraction"] * (float(right_y2) - float(right_y1))
            return self._position_from_centers(left_cx, left_cy, right_cx, right_cy, width, height)
        if profile.get("mode") != "origin_bbox_output_ridge":
            return None
        if raw["right_m"] is None or raw["down_m"] is None or raw["forward_m"] is None:
            return None
        try:
            base = self._bbox_feature_base(left, right, raw, width, height)
            values = np.array(
                [self._bbox_feature_value(name, base) for name in profile["feature_names"]],
                dtype=float,
            )
        except Exception as exc:
            self.get_logger().warning(f"failed to compute bbox output profile features: {exc}")
            return None
        normalized = (values - profile["feature_mean"]) / profile["feature_scale"]
        predicted = profile["coefficients"] @ normalized
        estimated = np.array([raw["forward_m"], raw["right_m"], raw["down_m"]], dtype=float)
        corrected_frd = estimated + profile["blend"] * (predicted - estimated)
        if not np.all(np.isfinite(corrected_frd)):
            return None
        corrected_forward_m = float(corrected_frd[0])
        corrected_right_m = float(corrected_frd[1])
        corrected_down_m = float(corrected_frd[2])
        return {
            "right_m": corrected_right_m,
            "down_m": corrected_down_m,
            "forward_m": corrected_forward_m,
            "range_m": math.sqrt(
                corrected_right_m * corrected_right_m
                + corrected_down_m * corrected_down_m
                + corrected_forward_m * corrected_forward_m
            ),
            "disparity_px": raw["disparity_px"],
        }

    def _apply_bounded_gain_forward_cell_guard_profile(self, left, right, raw, width, height):
        profile = self.bbox_center_adjustment
        if not isinstance(profile, BoundedGainForwardCellGuardPolicy):
            return None, self._profile_status()
        result = profile.apply(
            left,
            right,
            raw,
            width_px=float(width),
            height_px=float(height),
            horizontal_fov_rad=float(self.horizontal_fov_rad),
            baseline_m=float(self.baseline_m),
        )
        return result.corrected, result.status

    def _apply_estimated_to_truth_sim3_profile(self, raw, left=None, right=None):
        profile = self.bbox_center_adjustment
        if not isinstance(profile, EstimatedToTruthSim3Profile):
            return None, self._profile_status()
        profile_raw = dict(raw)
        if left is not None:
            profile_raw.setdefault("left_u_px", left.get("cx"))
            profile_raw.setdefault("left_v_px", left.get("cy"))
        if right is not None:
            profile_raw.setdefault("right_u_px", right.get("cx"))
            profile_raw.setdefault("right_v_px", right.get("cy"))
        result = profile.apply(profile_raw)
        return result.corrected, result.status

    def _apply_estimated_to_truth_disparity_profile(self, raw):
        profile = self.bbox_center_adjustment
        if not isinstance(profile, EstimatedToTruthDisparityProfile):
            return None, self._profile_status()
        result = profile.apply(raw)
        return result.corrected, result.status

    def _apply_estimated_to_truth_affine_profile(self, raw):
        profile = self.bbox_center_adjustment
        if not isinstance(profile, EstimatedToTruthAffineProfile):
            return None, self._profile_status()
        result = profile.apply(raw)
        return result.corrected, result.status

    def _apply_disparity_residual_profile(self, corrected, raw, profile=None):
        if profile is None:
            profile = getattr(self, "disparity_residual_profile", None)
        if not isinstance(profile, DisparityResidualRuntimeProfile):
            return corrected, "disabled"
        if corrected is None:
            return corrected, "skipped:no_corrected_base"
        disparity = raw.get("disparity_px")
        if disparity is None or not math.isfinite(float(disparity)) or float(disparity) <= 0.0:
            return corrected, "skipped:invalid_disparity"
        sample = RuntimeMeasurementSample(
            raw_right_m=float(corrected["right_m"]),
            raw_down_m=float(corrected["down_m"]),
            raw_forward_m=float(corrected["forward_m"]),
            measured_disparity_px=float(disparity),
            raw_range_m=float(corrected.get("range_m", math.nan)),
            left_u_px=raw.get("left_u_px"),
            left_v_px=raw.get("left_v_px"),
        )
        residual = profile.apply(sample)
        if residual is None:
            return corrected, "skipped:profile_noop"
        return self._raw_from_runtime_sample(residual), f"applied:{profile.profile_id or profile.method}"

    def _select_disparity_residual_profile_from_features(self, feature_summary):
        selector = getattr(self, "disparity_residual_feature_selector", None)
        if not isinstance(selector, DisparityResidualFeatureSelector):
            return None, "disabled", ""
        selection = selector.select(feature_summary)
        label = str(selection.get("selected_label", ""))
        profile_path = str(selection.get("selected_profile", ""))
        if not selection.get("selected_profile_exists", False):
            return None, f"skipped:profile_missing:{label}", label
        try:
            profile = DisparityResidualRuntimeProfile.from_json_file(profile_path)
        except Exception as exc:
            return None, f"skipped:{type(exc).__name__}:{label}", label
        return profile, f"selected:{label}", label

    def _append_disparity_residual_feature_selector_window(self, *, raw, corrected):
        if raw is None:
            return "skipped:no_raw"
        required = ("left_u_px", "left_v_px", "disparity_px", "right_m", "down_m", "forward_m")
        if any(raw.get(field) is None for field in required):
            return "skipped:missing_runtime_fields"
        observation = corrected if corrected is not None else raw
        try:
            row = {
                "left_u_px": float(raw["left_u_px"]),
                "left_v_px": float(raw["left_v_px"]),
                "measured_disparity_px": float(raw["disparity_px"]),
                "fused_disparity_px": float(observation.get("disparity_px", raw["disparity_px"])),
                "raw_right_m": float(raw["right_m"]),
                "raw_down_m": float(raw["down_m"]),
                "raw_forward_m": float(raw["forward_m"]),
                "observation_right_m": float(observation["right_m"]),
                "observation_down_m": float(observation["down_m"]),
                "observation_forward_m": float(observation["forward_m"]),
            }
        except Exception as exc:
            return f"skipped:{type(exc).__name__}"
        window = getattr(self, "disparity_residual_feature_selector_window", None)
        if not isinstance(window, deque):
            self._reset_disparity_residual_feature_selector_window()
            window = self.disparity_residual_feature_selector_window
        window.append(row)
        return f"ready:{len(window)}"

    def _disparity_residual_feature_selector_summary(self, *, raw, corrected):
        append_status = self._append_disparity_residual_feature_selector_window(raw=raw, corrected=corrected)
        if not append_status.startswith("ready:"):
            return {}, append_status
        window = getattr(self, "disparity_residual_feature_selector_window", deque())
        min_samples = max(1, int(getattr(self, "disparity_residual_feature_selector_min_samples", 1)))
        if len(window) < min_samples:
            return {}, f"accumulating:{len(window)}/{min_samples}"
        try:
            feature_row = feature_summary_from_runtime_rows(
                list(window),
                feature_label="stereodistance_node_runtime_window",
                observation_rdf_source="corrected_runtime_window",
            )
        except Exception as exc:
            return {}, f"skipped:{type(exc).__name__}"
        return feature_row, f"ready:runtime_window:{len(window)}"

    def _disparity_residual_feature_summary(self, *, raw, corrected):
        if raw is None:
            return {}, "skipped:no_raw"
        required = ("left_u_px", "left_v_px", "disparity_px", "right_m", "down_m", "forward_m")
        if any(raw.get(field) is None for field in required):
            return {}, "skipped:missing_runtime_fields"
        observation = corrected if corrected is not None else raw
        observation_source = "corrected_current_sample" if corrected is not None else "raw_current_sample_fallback"
        try:
            feature_row = feature_summary_from_arrays(
                left_u_px=[float(raw["left_u_px"])],
                left_v_px=[float(raw["left_v_px"])],
                measured_disparity_px=[float(raw["disparity_px"])],
                raw_rdf_m=[
                    [
                        float(raw["right_m"]),
                        float(raw["down_m"]),
                        float(raw["forward_m"]),
                    ]
                ],
                fused_disparity_px=[float(observation.get("disparity_px", raw["disparity_px"]))],
                observation_rdf_m=[
                    [
                        float(observation["right_m"]),
                        float(observation["down_m"]),
                        float(observation["forward_m"]),
                    ]
                ],
                feature_label="stereodistance_node_current_sample",
                observation_rdf_source=observation_source,
            )
        except Exception as exc:
            return {}, f"skipped:{type(exc).__name__}"
        return feature_row, "ready:current_sample"

    def _calculate_metrics(self, left_image, left, right, actual, phase):
        if left is None or right is None:
            return {
                "distance_m": None,
                "disparity_px": None,
                "estimated_right_m": None,
                "estimated_down_m": None,
                "estimated_forward_m": None,
                "estimated_range_m": None,
                "corrected_right_m": None,
                "corrected_down_m": None,
                "corrected_forward_m": None,
                "corrected_range_m": None,
                "corrected_disparity_px": None,
                "bbox_adjustment": None,
                "profile_status": self._profile_status(
                    getattr(self.bbox_center_adjustment, "profile_id", ""),
                    getattr(self.bbox_center_adjustment, "mode", ""),
                    3,
                    "incomplete_bbox_pair",
                ),
                "observation_fusion_status": "incomplete_bbox_pair",
                "general_ray_shadow": general_ray_shadow_not_computed(
                    status="not_computed_incomplete_bbox_pair",
                ),
                "actual_distance_m": actual,
                "distance_error_m": None,
                "mission_phase": phase,
                "status": self.yolo_load_error or "YOLO target not detected in both images",
            }

        width = left_image.shape[1]
        height = left_image.shape[0]
        legacy_raw = self._position_from_centers(left["cx"], left["cy"], right["cx"], right["cy"], width, height)
        general_ray_shadow_result = triangulate_nominal_stereo_centers(
            left_cx_px=left["cx"],
            left_cy_px=left["cy"],
            right_cx_px=right["cx"],
            right_cy_px=right["cy"],
            width_px=int(width),
            height_px=int(height),
            horizontal_fov_rad=self.horizontal_fov_rad,
            baseline_m=self.baseline_m,
        )
        general_ray_shadow = general_ray_shadow_summary(
            general_ray_shadow_result,
            legacy_forward_m=legacy_raw.get("forward_m"),
            legacy_right_m=legacy_raw.get("right_m"),
            legacy_down_m=legacy_raw.get("down_m"),
            legacy_range_m=legacy_raw.get("range_m"),
        )
        raw = legacy_raw
        observation_fusion_status = "disabled"
        fused_raw, observation_fusion_status = self._apply_observation_fusion(left, right, width, height)
        if fused_raw is not None:
            raw = fused_raw
        raw.setdefault("left_u_px", float(left["cx"]))
        raw.setdefault("left_v_px", float(left["cy"]))
        raw.setdefault("right_u_px", float(right["cx"]))
        raw.setdefault("right_v_px", float(right["cy"]))
        corrected = None
        bbox_adjustment = None
        profile_status = self._profile_status()
        if isinstance(self.bbox_center_adjustment, OriginBBoxReprojectionProfile):
            corrected, bbox_adjustment, profile_status = self._apply_bbox_reprojection_profile(
                left,
                right,
                raw,
                width,
                height,
            )
        elif isinstance(self.bbox_center_adjustment, BoundedGainForwardCellGuardPolicy):
            corrected, profile_status = self._apply_bounded_gain_forward_cell_guard_profile(
                left,
                right,
                raw,
                width,
                height,
            )
        elif isinstance(self.bbox_center_adjustment, EstimatedToTruthSim3Profile):
            corrected, profile_status = self._apply_estimated_to_truth_sim3_profile(raw, left, right)
        elif isinstance(self.bbox_center_adjustment, EstimatedToTruthDisparityProfile):
            corrected, profile_status = self._apply_estimated_to_truth_disparity_profile(raw)
        elif isinstance(self.bbox_center_adjustment, EstimatedToTruthAffineProfile):
            corrected, profile_status = self._apply_estimated_to_truth_affine_profile(raw)
        elif isinstance(self.bbox_center_adjustment, dict):
            corrected = self._apply_bbox_output_profile(left, right, raw, width, height)
            profile_status = self._profile_status(
                self.bbox_center_adjustment.get("profile_id", ""),
                self.bbox_center_adjustment.get("mode", "legacy_json"),
                0 if corrected is not None else 3,
                "legacy_output_profile" if corrected is not None else "legacy_output_profile_noop",
            )
        else:
            bbox_adjustment = self._adjust_bbox_centers(left, right)
        if bbox_adjustment is not None:
            corrected = self._position_from_centers(
                bbox_adjustment["left_cx"],
                bbox_adjustment["left_cy"],
                bbox_adjustment["right_cx"],
                bbox_adjustment["right_cy"],
                width,
                height,
            )
            if not isinstance(self.bbox_center_adjustment, OriginBBoxReprojectionProfile):
                profile_status = self._profile_status("global_affine_csv", "global_affine_csv", 0, "csv_affine")
        pre_residual_feature_summary, pre_residual_feature_summary_status = self._disparity_residual_feature_summary(
            raw=raw,
            corrected=corrected,
        )
        selector_feature_summary = pre_residual_feature_summary
        selector_feature_summary_status = pre_residual_feature_summary_status
        selected_residual_profile = None
        feature_selector_status = "disabled"
        feature_selected_label = ""
        if not isinstance(getattr(self, "disparity_residual_profile", None), DisparityResidualRuntimeProfile):
            if isinstance(getattr(self, "disparity_residual_feature_selector", None), DisparityResidualFeatureSelector):
                selector_feature_summary, selector_feature_summary_status = (
                    self._disparity_residual_feature_selector_summary(raw=raw, corrected=corrected)
                )
            if selector_feature_summary:
                selected_residual_profile, feature_selector_status, feature_selected_label = (
                    self._select_disparity_residual_profile_from_features(selector_feature_summary)
                )
            elif selector_feature_summary_status != pre_residual_feature_summary_status:
                feature_selector_status = selector_feature_summary_status
        corrected, disparity_residual_status = self._apply_disparity_residual_profile(
            corrected,
            raw,
            profile=selected_residual_profile,
        )
        feature_summary, feature_summary_status = self._disparity_residual_feature_summary(raw=raw, corrected=corrected)
        if feature_summary_status != "ready:current_sample":
            feature_summary_status = pre_residual_feature_summary_status
            feature_summary = pre_residual_feature_summary
        error = None if raw["distance_m"] is None else raw["distance_m"] - actual
        return {
            "distance_m": raw["distance_m"],
            "disparity_px": raw["disparity_px"],
            "estimated_right_m": raw["right_m"],
            "estimated_down_m": raw["down_m"],
            "estimated_forward_m": raw["forward_m"],
            "estimated_range_m": raw["range_m"],
            "corrected_right_m": None if corrected is None else corrected["right_m"],
            "corrected_down_m": None if corrected is None else corrected["down_m"],
            "corrected_forward_m": None if corrected is None else corrected["forward_m"],
            "corrected_range_m": None if corrected is None else corrected["range_m"],
            "corrected_disparity_px": None if corrected is None else corrected["disparity_px"],
            "bbox_adjustment": bbox_adjustment,
            "profile_status": profile_status,
            "observation_fusion_status": observation_fusion_status,
            "general_ray_shadow": general_ray_shadow,
            "disparity_residual_status": disparity_residual_status,
            "disparity_residual_feature_summary_status": feature_summary_status,
            "disparity_residual_feature_summary": feature_summary,
            "disparity_residual_feature_selector_summary_status": selector_feature_summary_status,
            "disparity_residual_feature_selector_window_count": len(
                getattr(self, "disparity_residual_feature_selector_window", [])
            ),
            "disparity_residual_feature_selector_status": feature_selector_status,
            "disparity_residual_feature_selected_label": feature_selected_label,
            "actual_distance_m": actual,
            "distance_error_m": error,
            "mission_phase": phase,
            "status": f"YOLO tracking {self.yolo_class_name}; observation_fusion={observation_fusion_status}",
        }

    def _publish_marker(self):
        with self.lock:
            metrics = dict(self.metrics)

        distance = metrics.get("distance_m")
        disparity = metrics.get("disparity_px")
        actual = metrics.get("actual_distance_m", self._actual_distance())
        error = metrics.get("distance_error_m")
        estimated_right = metrics.get("estimated_right_m")
        estimated_down = metrics.get("estimated_down_m")
        estimated_forward = metrics.get("estimated_forward_m")
        phase = metrics.get("mission_phase", "unknown")
        if distance is None:
            distance_text = "Distance: --"
        else:
            distance_text = f"Distance: {distance:.2f} m"

        if disparity is None:
            disparity_text = "Disparity: --"
        else:
            disparity_text = f"Disparity: {disparity:.1f} px"
        if error is None:
            error_text = "Error: --"
        else:
            error_text = f"Error: {error:+.2f} m"
        if estimated_right is None or estimated_down is None or estimated_forward is None:
            position_text = "Estimated FRD: --"
        else:
            position_text = (
                f"Estimated FRD: {estimated_forward:.2f}, "
                f"{estimated_right:+.2f}, {estimated_down:+.2f} m"
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
        marker.text = "\n".join(
            [
                distance_text,
                f"Actual: {actual:.2f} m",
                error_text,
                position_text,
                disparity_text,
                f"Baseline: {self.baseline_m:.2f} m",
                f"Phase: {phase}",
                f"Status: {metrics.get('status', 'unknown')}",
                f"Updated: {time.strftime('%H:%M:%S')}",
            ]
        )
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
