import json
import math
import os
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
try:  # ROS is present in production; pure geometry tests may run without it.
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Point, PointStamped, PoseStamped
    from px4_msgs.msg import SensorGps, VehicleGlobalPosition
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image as ImageMsg
    from std_msgs.msg import Float32, Float32MultiArray, String
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:  # pragma: no cover - exercised only outside ROS
    rclpy = None

    class Node:  # type: ignore[no-redef]
        pass

    class MultiThreadedExecutor:  # type: ignore[no-redef]
        pass

    class ParameterDescriptor:  # type: ignore[no-redef]
        def __init__(self, **kwargs):
            pass

    class SetParametersResult:  # type: ignore[no-redef]
        def __init__(self, **kwargs):
            self.successful = kwargs.get("successful", True)
            self.reason = kwargs.get("reason", "")

    class HistoryPolicy:  # type: ignore[no-redef]
        KEEP_LAST = 1

    class ReliabilityPolicy:  # type: ignore[no-redef]
        BEST_EFFORT = 1

    class DurabilityPolicy:  # type: ignore[no-redef]
        TRANSIENT_LOCAL = 1

    class QoSProfile:  # type: ignore[no-redef]
        def __init__(self, **kwargs):
            pass

    class _Message:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class Float32(_Message):  # type: ignore[no-redef]
        def __init__(self, data=0.0, **kwargs):
            super().__init__(data=data, **kwargs)

    class Float32MultiArray(_Message):  # type: ignore[no-redef]
        def __init__(self, data=None, **kwargs):
            super().__init__(data=[] if data is None else data, **kwargs)

    class String(_Message):  # type: ignore[no-redef]
        def __init__(self, data="", **kwargs):
            super().__init__(data=data, **kwargs)

    class ImageMsg(_Message):  # type: ignore[no-redef]
        pass

    class PoseStamped(_Message):  # type: ignore[no-redef]
        pass

    class PointStamped(_Message):  # type: ignore[no-redef]
        def __init__(self, **kwargs):
            stamp = _Message(sec=0, nanosec=0)
            header = _Message(stamp=stamp, frame_id="")
            point = _Message(x=0.0, y=0.0, z=0.0)
            super().__init__(header=header, point=point, **kwargs)

    class Point(_Message):  # type: ignore[no-redef]
        def __init__(self, x=0.0, y=0.0, z=0.0, **kwargs):
            super().__init__(x=x, y=y, z=z, **kwargs)

    class Clock(_Message):  # type: ignore[no-redef]
        pass

    class SensorGps(_Message):  # type: ignore[no-redef]
        pass

    class VehicleGlobalPosition(_Message):  # type: ignore[no-redef]
        pass

    class Marker(_Message):  # type: ignore[no-redef]
        TEXT_VIEW_FACING = 9
        LINE_LIST = 5
        SPHERE = 2
        CUBE = 1
        MESH_RESOURCE = 10
        ADD = 0

    class MarkerArray(_Message):  # type: ignore[no-redef]
        def __init__(self, markers=None, **kwargs):
            super().__init__(markers=[] if markers is None else markers, **kwargs)

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
from stereo_ground_sim.research_recording import (
    RESEARCH_SCHEMA_VERSION,
    ResearchRecorder,
    discover_repository_root,
    empty_pair_record,
    validate_recording_configuration,
)
from stereo_ground_sim.disparity_residual_feature_summary import feature_summary_from_arrays, feature_summary_from_runtime_rows
from stereo_ground_sim.disparity_residual_feature_selector import DisparityResidualFeatureSelector
from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_FRAME,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
    GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    PinholeIntrinsics,
    ray_from_pixel,
    triangulate_rays,
)
from stereo_ground_sim.coordinate_frames import (
    CameraOpticalPose,
    SOURCE_GAZEBO_ENTITY,
    VEHICLE_REFERENCE_DEFINITION,
    VEHICLE_REFERENCE_LINK,
    WORLD_FRAME,
    apply_optical_rpy_error,
    build_baseline_center_frd,
    camera_optical_pose_from_quaternion,
    ground_truth_world_to_baseline,
    nominal_camera_poses,
    project_world_point_to_pixel,
    effective_optical_rpy_errors_deg,
)
from stereo_ground_sim.gps_baseline import RawGpsFix, gps_baseline_enu_m, norm_m
from stereo_ground_sim.time_alignment import (
    BoundedIdHistory,
    GroundTruthPositionBuffer,
    PendingGroundTruthRecord,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
    StereoImagePairer,
    image_stamp_ns,
)


DRONE_MESH_RESOURCE = "package://stereo_ground_sim/models/x500_15m/meshes/NXP-HGD-CF.dae"


@dataclass(frozen=True)
class ObservedCameraOraclePose:
    """Validated Gazebo optical pose plus immutable source provenance."""

    camera_label: str
    pose: CameraOpticalPose
    gazebo_stamp_ns: int
    generated_sdf_sha256: str
    base_entity_name: str
    base_entity_id: int
    optical_entity_name: str
    optical_entity_id: int
    optical_position_world_enu_m: tuple[float, float, float]
    optical_orientation_world_from_entity_xyzw: tuple[float, float, float, float]
    simulation_epoch: int
    received_monotonic_s: float


_CAMERA_ORACLE_UNSET = object()


def ray_visualization_world_enu_from_triangulation(
    baseline_center_frd,
    left_camera_pose,
    right_camera_pose,
    left_point_baseline_center_frd_m,
    right_point_baseline_center_frd_m,
    midpoint_baseline_center_frd_m,
):
    """Return the raw-triangulation geometry in the RViz World ENU frame.

    The triangulation solver works in ``baseline_center_frd``.  This helper
    keeps the visualization contract explicit: camera origins are already
    World ENU and each closest-point result is converted back to World ENU
    before becoming a marker endpoint.
    """

    if any(
        point is None
        for point in (
            left_point_baseline_center_frd_m,
            right_point_baseline_center_frd_m,
            midpoint_baseline_center_frd_m,
        )
    ):
        return None
    return {
        "left_origin_world_enu_m": tuple(left_camera_pose.center_world_enu_m),
        "left_target_world_enu_m": tuple(
            baseline_center_frd.point_to_world(left_point_baseline_center_frd_m)
        ),
        "right_origin_world_enu_m": tuple(right_camera_pose.center_world_enu_m),
        "right_target_world_enu_m": tuple(
            baseline_center_frd.point_to_world(right_point_baseline_center_frd_m)
        ),
        "midpoint_world_enu_m": tuple(
            baseline_center_frd.point_to_world(midpoint_baseline_center_frd_m)
        ),
    }


def dkl_ideal_pixels_from_world(
    point_world_enu_m,
    left_camera_pose,
    right_camera_pose,
    intrinsics,
    width,
    height,
):
    """Generate DKL-style quantized ideal pixels from a World ENU reference point.

    The point and both camera poses are World ENU; ``intrinsics`` are pixel
    units.  The returned ``(u_l, v_l, u_r, v_r)`` values are nearest-pixel
    centers and are valid only within the supplied image bounds.
    """

    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0:
        return None
    pixels = []
    for camera in (left_camera_pose, right_camera_pose):
        pixel = project_world_point_to_pixel(
            point_world_enu_m,
            camera,
            fx_px=intrinsics.fx_px,
            fy_px=intrinsics.fy_px,
            cx_px=intrinsics.cx_px,
            cy_px=intrinsics.cy_px,
        )
        if pixel is None:
            return None
        u_px = float(math.floor(float(pixel[0]) + 0.5))
        v_px = float(math.floor(float(pixel[1]) + 0.5))
        if not (0.0 <= u_px < width and 0.0 <= v_px < height):
            return None
        pixels.extend((u_px, v_px))
    return tuple(pixels)


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
        self.declare_parameter("target_spawn_z_m", 0.75)
        self.declare_parameter("fly_target_in_gazebo", False)
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
        self.declare_parameter("observation_mode", "bbox_yolo")
        # This is a bounded burst buffer, not a sustained-rate guarantee.
        # Strict overflow accounting invalidates a run instead of dropping silently.
        self.declare_parameter("measurement_queue_capacity", 32)
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
        self.declare_parameter("cam_l_optical_roll_deg", 0.0)
        self.declare_parameter("cam_l_optical_pitch_deg", 0.0)
        self.declare_parameter("cam_l_optical_yaw_deg", 0.0)
        self.declare_parameter("cam_r_optical_roll_deg", 0.0)
        self.declare_parameter("cam_r_optical_pitch_deg", 0.0)
        self.declare_parameter("cam_r_optical_yaw_deg", 0.0)
        self.declare_parameter("cam_l_px4_roll_deg", 0.0)
        self.declare_parameter("cam_l_px4_pitch_deg", 0.0)
        self.declare_parameter("cam_l_px4_yaw_deg", 0.0)
        self.declare_parameter("cam_r_px4_roll_deg", 0.0)
        self.declare_parameter("cam_r_px4_pitch_deg", 0.0)
        self.declare_parameter("cam_r_px4_yaw_deg", 0.0)
        self.declare_parameter("common_optical_roll_error_deg", 0.0)
        self.declare_parameter("relative_optical_roll_error_deg", 0.0)
        self.declare_parameter("common_optical_pitch_error_deg", 0.0)
        self.declare_parameter("relative_optical_pitch_error_deg", 0.0)
        self.declare_parameter("common_optical_yaw_error_deg", 0.0)
        self.declare_parameter("relative_optical_yaw_error_deg", 0.0)
        self.declare_parameter("enable_research_recording", False)
        self.declare_parameter("research_output_dir", "")
        self.declare_parameter("research_run_id", "")
        self.declare_parameter("research_case_id", "")
        self.declare_parameter("research_repository", "")
        self.declare_parameter("research_branch", "")
        self.declare_parameter("research_commit_sha", "")
        self.declare_parameter("research_git_status", "")
        self.declare_parameter("research_gazebo_version", "")
        self.declare_parameter("research_px4_version", "not_applicable")
        self.declare_parameter("research_record_queue_capacity", 1024)
        self.declare_parameter("camera_image_width_px", 1280)
        self.declare_parameter("camera_image_height_px", 720)
        self.declare_parameter("camera_update_rate_hz", 20.0)
        self.declare_parameter("camera_oracle_left_topic", "/simulation/camera_pose_diagnostics/left")
        self.declare_parameter("camera_oracle_right_topic", "/simulation/camera_pose_diagnostics/right")
        self.declare_parameter("camera_oracle_generated_sdf_sha256", "")
        self.declare_parameter("camera_oracle_max_age_s", 2.5)
        self.declare_parameter("enable_gps_baseline_triangulation", False)
        self.declare_parameter("gps_baseline_source", "raw_gps")
        self.declare_parameter("gps_baseline_left_topic", "/camera_left/fmu/out/vehicle_gps_position")
        self.declare_parameter("gps_baseline_right_topic", "/camera_right/fmu/out/vehicle_gps_position")
        self.declare_parameter("global_baseline_left_topic", "/camera_left/fmu/out/vehicle_global_position")
        self.declare_parameter("global_baseline_right_topic", "/camera_right/fmu/out/vehicle_global_position")
        self.declare_parameter("gps_baseline_max_age_s", 2.5)
        self.declare_parameter("enable_rviz_rays", False)

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
        self.target_spawn_z_m = self._float_param("target_spawn_z_m")
        self.fly_target_in_gazebo = self._bool_param("fly_target_in_gazebo")
        self.yolo_model_path = str(self.get_parameter("yolo_model_path").value)
        self.yolo_class_name = str(self.get_parameter("yolo_class_name").value)
        self.yolo_confidence = self._float_param("yolo_confidence")
        self.yolo_device = str(self.get_parameter("yolo_device").value)
        self.yolo_target_hz = self._float_param("yolo_target_hz")
        self.yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)
        self.observation_mode = self._validate_observation_mode(
            self.get_parameter("observation_mode").value
        )
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
        for name in self._attitude_parameter_names():
            setattr(self, name, self._float_param(name))
        self._configure_attitude_error_state()
        self._configure_camera_frames()
        self.camera_oracle_left_topic = str(self.get_parameter("camera_oracle_left_topic").value)
        self.camera_oracle_right_topic = str(self.get_parameter("camera_oracle_right_topic").value)
        self.camera_oracle_generated_sdf_sha256 = str(
            self.get_parameter("camera_oracle_generated_sdf_sha256").value
        )
        self.camera_oracle_max_age_s = float(self.get_parameter("camera_oracle_max_age_s").value)
        if not math.isfinite(self.camera_oracle_max_age_s) or self.camera_oracle_max_age_s <= 0.0:
            raise ValueError("camera_oracle_max_age_s must be positive and finite")
        self.observed_left_camera = None
        self.observed_right_camera = None
        self.camera_oracle_pair = (None, None)
        self.enable_gps_baseline_triangulation = bool(
            self.get_parameter("enable_gps_baseline_triangulation").value
        )
        self.gps_baseline_source = self._validate_gps_baseline_source(
            self.get_parameter("gps_baseline_source").value
        )
        self.gps_baseline_left_topic = str(self.get_parameter("gps_baseline_left_topic").value)
        self.gps_baseline_right_topic = str(self.get_parameter("gps_baseline_right_topic").value)
        self.global_baseline_left_topic = str(self.get_parameter("global_baseline_left_topic").value)
        self.global_baseline_right_topic = str(self.get_parameter("global_baseline_right_topic").value)
        self.gps_baseline_max_age_s = float(self.get_parameter("gps_baseline_max_age_s").value)
        if not math.isfinite(self.gps_baseline_max_age_s) or self.gps_baseline_max_age_s <= 0.0:
            raise ValueError("gps_baseline_max_age_s must be positive and finite")
        self.enable_rviz_rays = self._bool_param("enable_rviz_rays")
        self.gps_baseline_fixes = {"left": None, "right": None}
        self.global_baseline_fixes = {"left": None, "right": None}
        self.camera_oracle_rejection_count = 0
        self.last_camera_oracle_rejection_reason = "unavailable"
        self.enable_research_recording = self._bool_param("enable_research_recording")
        self.research_output_dir = str(self.get_parameter("research_output_dir").value)
        self.research_run_id = str(self.get_parameter("research_run_id").value)
        self.research_case_id = str(self.get_parameter("research_case_id").value)
        self.research_repository = str(self.get_parameter("research_repository").value)
        self.research_branch = str(self.get_parameter("research_branch").value)
        self.research_commit_sha = str(self.get_parameter("research_commit_sha").value)
        self.research_git_status = str(self.get_parameter("research_git_status").value)
        self.research_gazebo_version = str(self.get_parameter("research_gazebo_version").value)
        self.research_px4_version = str(self.get_parameter("research_px4_version").value)
        self.research_record_queue_capacity = int(
            self.get_parameter("research_record_queue_capacity").value
        )
        self.camera_image_width_px = int(self.get_parameter("camera_image_width_px").value)
        self.camera_image_height_px = int(self.get_parameter("camera_image_height_px").value)
        self.camera_update_rate_hz = self._float_param("camera_update_rate_hz")
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
        self.cached_actual_distance = None
        self.cached_ground_truth_position = None
        self.ground_truth_stamp_ns = None
        self.ground_truth_entity_id = None
        self.shutdown_event = threading.Event()
        self.accepting_input = True
        self.drain_requested = False
        self.drain_complete = False

        self.simulation_epoch = SimulationEpochTracker()
        self.image_pairer = StereoImagePairer(max_buffer_size=64)
        self.gt_position_buffer = GroundTruthPositionBuffer(max_buffer_size=512)
        self.pending_gt_aligner = PendingPairGroundTruthAligner(
            self.gt_position_buffer,
            max_pending_pairs=64,
        )
        self.measurement_queue_capacity = max(
            1,
            int(self.get_parameter("measurement_queue_capacity").value),
        )
        self.measurement_pair_queue = deque()
        self.gt_alignment_ready_queue = deque()
        self.measurement_results_by_pair_id = {}
        self.gt_alignments_by_pair_id = {}
        self.pairs_by_pair_id = {}
        self.max_pair_result_history = 128
        self.evaluated_pair_ids = BoundedIdHistory(self.max_pair_result_history)
        self.gt_evaluation_failed_pair_ids = BoundedIdHistory(
            self.max_pair_result_history
        )
        self.gt_published_pair_ids = BoundedIdHistory(self.max_pair_result_history)
        self.last_raw_published_pair_id = 0
        self.last_corrected_published_pair_id = 0
        self.last_gps_baseline_published_pair_id = 0
        self.last_gps_baseline_published_pair_id_by_source = {
            "raw_gps": 0,
            "global_position": 0,
        }
        self.measurement_queue_overflow_count = 0
        self.measurement_overrun_invalid = False
        self.measurement_rejected_after_overrun_count = 0
        self.measurement_rejected_pair_ids = BoundedIdHistory(self.max_pair_result_history)
        self.last_measurement_rejection_reason = ""
        self.measurement_accepted_count = 0
        self.measurement_processed_count = 0
        self.accepted_pair_discard_count = 0
        self.measurement_processing_error_count = 0
        self.input_rejected_after_shutdown_count = 0
        self.last_measurement_error_type = ""
        self.last_measurement_error_message = ""
        self.gt_evaluation_error_count = 0
        self.gt_evaluation_success_count = 0
        self.last_gt_evaluation_error_type = ""
        self.last_gt_evaluation_error_message = ""
        self.last_gt_evaluation_error_trigger = ""
        self.measurement_queue_depth_sample_count = 0
        self.measurement_queue_depth_sum = 0
        self.measurement_queue_depth_min = None
        self.measurement_queue_depth_max = 0
        self.old_epoch_result_discard_count = 0
        self.inflight_pair_id = None
        self.inflight_pair_epoch = None
        self.future_gt_pending = deque()
        # Live loaded graph: steady-state GT callbacks led this node's latest
        # /clock by 20 ms, with a 108 ms executor transient immediately after
        # reset. 200 ms / 64 samples contains that measured scheduling window
        # while rejecting the multi-second old-epoch stamps seen after reset.
        self.future_gt_capacity = 64
        self.gt_reorder_lead_limit_ns = 200_000_000
        self.future_gt_overflow_count = 0
        self.stale_pre_reset_gt_count = 0
        self.gt_reset_quarantine_duration_ns = 200_000_000
        self.gt_quarantine_required_contiguous_samples = 2
        self.gt_quarantine_active = False
        self.gt_quarantine_floor_stamp_ns = None
        self.gt_quarantine_candidates = deque()
        self.gt_quarantine_last_stamp_ns = None
        self.gt_quarantine_rejected_count = 0
        self.gt_quarantine_release_count = 0
        self.stale_pre_reset_count = 0
        self.inconsistent_clock_count = 0
        self.old_epoch_frame_count = 0
        self.last_pair_result = None
        self.gt_alignment_counts = {
            "exact": 0,
            "interpolated": 0,
            "nearest_before": 0,
            "nearest_after": 0,
            "unavailable": 0,
        }
        self.raw_computed_count = 0
        self.corrected_computed_count = 0
        self.ideal_projection_success_count = 0
        self.ideal_projection_rejected_count = 0
        self.last_ideal_projection_error = ""
        self.last_ideal_projection_pair_id = None
        self.shutdown_drain_status = "not_requested"
        self.shutdown_drain_duration_s = 0.0
        self.shutdown_remaining_measurement_count = 0
        self.shutdown_remaining_gt_alignment_count = 0
        self.shutdown_inflight_pair_id = None
        self.shutdown_initial_measurement_count = 0
        self.shutdown_initial_gt_alignment_count = 0
        self.shutdown_initial_inflight_pair_id = None
        self.shutdown_gt_evaluation_error_count = 0
        self.dropped_on_shutdown_count = 0
        self.dropped_on_shutdown_pair_id_min = None
        self.dropped_on_shutdown_pair_id_max = None
        self.discard_measurement_commits = False
        self.research_recorder = None
        self.research_terminal_pair_keys = set()
        self.research_terminal_record_count = 0
        self.research_recording_overrun_invalid = False
        self.research_recording_error_count = 0
        self.research_recording_first_error_type = ""
        self.research_recording_first_error_message = ""
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
            "actual_distance_m": None,
            "distance_error_m": None,
            "mission_phase": "init",
            "status": "waiting for images",
        }
        self.lock = threading.Lock()
        self.yolo_condition = threading.Condition(self.lock)
        self._initialize_research_recording()
        self._initialize_yolo_runtime_if_enabled()

        self.distance_pub = self.create_publisher(Float32, "/target_distance", 10)
        self.actual_distance_pub = self.create_publisher(Float32, "/ground_truth", 10)
        self.ground_truth_position_pub = self.create_publisher(
            Float32MultiArray, "/ground_truth_position", 10
        )
        self.raw_position_pub = self.create_publisher(PointStamped, "/stereo/raw_position", 10)
        self.attitude_corrected_position_pub = self.create_publisher(
            PointStamped, "/stereo/attitude_corrected_position", 10
        )
        self.gps_baseline_position_pub = self.create_publisher(
            PointStamped, "/stereo/gps_baseline_position", 10
        )
        self.raw_gps_baseline_position_pub = self.create_publisher(
            PointStamped, "/stereo/raw_gps_baseline_position", 10
        )
        self.global_position_baseline_position_pub = self.create_publisher(
            PointStamped, "/stereo/global_position_baseline_position", 10
        )
        # rqt_plot uses a stamped message's header time for its X axis. In
        # simulation that time is not comparable with rqt's wall-clock start
        # time, so provide display-only FRD arrays without a header.
        self.raw_gps_baseline_plot_pub = self.create_publisher(
            Float32MultiArray, "/stereo/raw_gps_baseline_plot", 10
        )
        self.global_position_baseline_plot_pub = self.create_publisher(
            Float32MultiArray, "/stereo/global_position_baseline_plot", 10
        )
        self.canonical_ground_truth_position_pub = self.create_publisher(
            PointStamped, "/stereo/ground_truth_position", 10
        )
        self.ground_truth_pose_sub = self.create_subscription(
            PoseStamped,
            "/simulation/ground_truth/vehicle_reference_link",
            self._ground_truth_pose_cb,
            10,
        )
        camera_oracle_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.left_camera_oracle_sub = self.create_subscription(
            String,
            self.camera_oracle_left_topic,
            lambda msg: self._camera_pose_diagnostic_callback("left", msg),
            camera_oracle_qos,
        )
        self.right_camera_oracle_sub = self.create_subscription(
            String,
            self.camera_oracle_right_topic,
            lambda msg: self._camera_pose_diagnostic_callback("right", msg),
            camera_oracle_qos,
        )
        if self.enable_gps_baseline_triangulation:
            self.create_subscription(
                SensorGps,
                self.gps_baseline_left_topic,
                lambda msg: self._gps_baseline_callback("left", msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.create_subscription(
                VehicleGlobalPosition,
                self.global_baseline_left_topic,
                lambda msg: self._global_baseline_callback("left", msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.create_subscription(
                VehicleGlobalPosition,
                self.global_baseline_right_topic,
                lambda msg: self._global_baseline_callback("right", msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.create_subscription(
                SensorGps,
                self.gps_baseline_right_topic,
                lambda msg: self._gps_baseline_callback("right", msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        self.clock_sub = self.create_subscription(Clock, "/clock", self._clock_cb, 10)
        self.disparity_pub = self.create_publisher(Float32, "/stereo/disparity", 10)
        self.error_pub = self.create_publisher(Float32, "/stereo/distance_error", 10)
        self.comparison_pub = self.create_publisher(Float32MultiArray, "/stereo/distance_comparison", 10)
        self.estimated_position_pub = self.create_publisher(
            Float32MultiArray, "/stereo/estimated_position", 10
        )
        self.corrected_position_pub = self.create_publisher(
            Float32MultiArray, "/stereo/corrected_position", 10
        )
        self.optical_position_pub = self.create_publisher(
            Float32MultiArray, "/stereo/optical_position", 10
        )
        # Simulation-reference-only DKL-style result: ground truth is
        # projected through the nominal cameras, quantized to pixel centres,
        # then triangulated with those same nominal poses.
        self.ideal_position_pub = self.create_publisher(
            Float32MultiArray, "/stereo/ideal_position", 10
        )
        self.bbox_adjustment_pub = self.create_publisher(
            Float32MultiArray, "/stereo/bbox_adjustment", 10
        )
        self.profile_status_pub = self.create_publisher(String, "/stereo/profile_status", 10)
        self.marker_pub = self.create_publisher(Marker, "/stereo/distance_marker", 10)
        self.ray_marker_pub = (
            self.create_publisher(MarkerArray, "/stereo/ray_markers", 10)
            if self.enable_rviz_rays
            else None
        )
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

        if not self._create_camera_image_subscriptions_if_enabled(camera_qos):
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
        self._start_yolo_worker_if_enabled()

        self.get_logger().info(
            "Stereo distance node started "
            f"(baseline={self.baseline_m:.2f} m, fov={self.horizontal_fov_rad:.3f} rad, "
            f"target={self.approach_start_distance_m:.1f}->{self.approach_end_distance_m:.1f} m)"
        )

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def _validate_observation_mode(value):
        mode = str(value)
        if mode not in ("bbox_yolo", "ideal_reference_point"):
            raise ValueError("observation_mode must be bbox_yolo or ideal_reference_point")
        return mode

    @staticmethod
    def _validate_gps_baseline_source(value):
        source = str(value)
        if source not in ("raw_gps", "global_position", "both"):
            raise ValueError(
                "gps_baseline_source must be raw_gps, global_position, or both"
            )
        return source

    @staticmethod
    def _finite_or_none(value):
        if value is None:
            return None
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None

    @staticmethod
    def _recording_parameter_names():
        return {
            "enable_research_recording",
            "research_output_dir",
            "research_run_id",
            "research_case_id",
            "research_repository",
            "research_branch",
            "research_commit_sha",
            "research_git_status",
            "research_gazebo_version",
            "research_px4_version",
            "research_record_queue_capacity",
            "camera_image_width_px",
            "camera_image_height_px",
            "camera_update_rate_hz",
            "camera_oracle_left_topic",
            "camera_oracle_right_topic",
            "camera_oracle_generated_sdf_sha256",
            "camera_oracle_max_age_s",
        }

    def _research_run_metadata(self):
        observation = self._observation_metadata()
        desired_camera_poses = {
            label: {
                "center_world_enu_m": list(camera.center_world_enu_m),
                "right_world_enu": list(camera.right_world_enu),
                "down_world_enu": list(camera.down_world_enu),
                "forward_world_enu": list(camera.forward_world_enu),
            }
            for label, camera in (
                ("left", self.desired_left_camera),
                ("right", self.desired_right_camera),
            )
        }
        attitude = {
            name: getattr(self, name) for name in self._attitude_parameter_names()
        }
        attitude.update(
            {
                "effective_left_optical_roll_error_deg": self.effective_left_optical_roll_error_deg,
                "effective_left_optical_pitch_error_deg": self.effective_left_optical_pitch_error_deg,
                "effective_left_optical_yaw_error_deg": self.effective_left_optical_yaw_error_deg,
                "effective_right_optical_roll_error_deg": self.effective_right_optical_roll_error_deg,
                "effective_right_optical_pitch_error_deg": self.effective_right_optical_pitch_error_deg,
                "effective_right_optical_yaw_error_deg": self.effective_right_optical_yaw_error_deg,
                "attitude_error_frame": self.attitude_error_frame,
                "attitude_angle_unit": self.attitude_angle_unit,
                "attitude_parameterization": self.attitude_parameterization,
                "attitude_rotation_composition": self.attitude_rotation_composition,
            }
        )
        return {
            "schema_version": RESEARCH_SCHEMA_VERSION,
            "run_id": self.research_run_id,
            "case_id": self.research_case_id,
            "repository": self.research_repository,
            "branch": self.research_branch,
            "commit_sha": self.research_commit_sha,
            "git_status": self.research_git_status,
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "default"),
            "gazebo_version": self.research_gazebo_version,
            "px4_version": self.research_px4_version,
            "observation_mode": observation["observation_mode"],
            "observation_source": observation["observation_source"],
            "measurement_gt_independent": observation["measurement_gt_independent"],
            "reference_source": observation["reference_source"],
            "projection_camera_pose_source": observation["projection_camera_pose_source"],
            "evaluation_gt_source": observation["evaluation_gt_source"],
            "gazebo_world_frame": "gazebo_world_enu",
            "gazebo_world_axes": "east_north_up",
            "camera_optical_frame": "camera_optical_rdf",
            "camera_optical_axes": "right_down_forward",
            "output_frame": BASELINE_CENTER_FRD_FRAME,
            "output_axes": "forward_right_down",
            "position_unit": "m",
            "pixel_unit": "px",
            "angle_unit": "deg",
            "timestamp_unit": "ns",
            "quaternion_order": "xyzw",
            "baseline_m": self.baseline_m,
            "camera_height_m": self.camera_height_m,
            "horizontal_fov_rad": self.horizontal_fov_rad,
            "image_width_px": self.camera_image_width_px,
            "image_height_px": self.camera_image_height_px,
            "camera_update_rate_hz": self.camera_update_rate_hz,
            "camera_oracle_source": "gazebo_observed_optical",
            "camera_oracle_expected_sdf_sha256": self.camera_oracle_generated_sdf_sha256,
            "camera_oracle_max_age_s": self.camera_oracle_max_age_s,
            "camera_oracle_topics": {
                "left": self.camera_oracle_left_topic,
                "right": self.camera_oracle_right_topic,
            },
            "desired_camera_poses_from_env": desired_camera_poses,
            **attitude,
            "scenario_name": self.scenario_name,
            "target_class": self.target_class,
            "target_size_m": self._finite_or_none(self.target_size_m),
            "mission_profile": self.mission_profile,
            "target_model_name": self.target_model_name,
            "target_spawn_z_m": self.target_spawn_z_m,
            "scenario_range_min_m": self._finite_or_none(self.scenario_range_min_m),
            "scenario_range_max_m": self._finite_or_none(self.scenario_range_max_m),
            "approach_start_distance_m": self.approach_start_distance_m,
            "approach_end_distance_m": self.approach_end_distance_m,
            "zoom_index": self._finite_or_none(self.zoom_index),
            "measurement_queue_capacity": self.measurement_queue_capacity,
            "research_record_queue_capacity": self.research_record_queue_capacity,
            "strict_measurement_overrun_policy": True,
            "recorder_queue_policy": "invalidate_run_no_silent_drop",
            "yolo_model_status": (
                "not_applicable"
                if self.observation_mode == "ideal_reference_point"
                else "configured" if self.yolo_model_path.strip() else "not_configured"
            ),
        }

    def _initialize_research_recording(self):
        if not self.enable_research_recording:
            return
        if self.camera_image_width_px <= 0 or self.camera_image_height_px <= 0:
            raise ValueError("recording camera image dimensions must be positive")
        if not math.isfinite(self.camera_update_rate_hz) or self.camera_update_rate_hz <= 0.0:
            raise ValueError("recording camera update rate must be positive and finite")
        repository_root = discover_repository_root(Path.cwd())
        output_dir = validate_recording_configuration(
            output_dir=self.research_output_dir,
            run_id=self.research_run_id,
            case_id=self.research_case_id,
            repository=self.research_repository,
            branch=self.research_branch,
            commit_sha=self.research_commit_sha,
            git_status=self.research_git_status,
            gazebo_version=self.research_gazebo_version,
            px4_version=self.research_px4_version,
            queue_capacity=self.research_record_queue_capacity,
            repository_root=repository_root,
        )
        self.research_recorder = ResearchRecorder(
            output_dir=output_dir,
            run_metadata=self._research_run_metadata(),
            queue_capacity=self.research_record_queue_capacity,
        )

    def _bool_param(self, name):
        return self._coerce_bool(self.get_parameter(name).value)

    def _coerce_bool(self, value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _configure_camera_frames(self):
        """Freeze nominal output frame and retain .env-derived desired poses."""

        nominal_left, nominal_right = nominal_camera_poses(self.baseline_m, self.camera_height_m)
        self.nominal_left_camera = nominal_left
        self.nominal_right_camera = nominal_right
        self.baseline_center_frd = build_baseline_center_frd(
            nominal_left.center_world_enu_m,
            nominal_right.center_world_enu_m,
            nominal_left.forward_world_enu,
            nominal_right.forward_world_enu,
        )
        self.desired_left_camera = apply_optical_rpy_error(
            nominal_left,
            math.radians(self.effective_left_optical_roll_error_deg),
            math.radians(self.effective_left_optical_pitch_error_deg),
            math.radians(self.effective_left_optical_yaw_error_deg),
        )
        self.desired_right_camera = apply_optical_rpy_error(
            nominal_right,
            math.radians(self.effective_right_optical_roll_error_deg),
            math.radians(self.effective_right_optical_pitch_error_deg),
            math.radians(self.effective_right_optical_yaw_error_deg),
        )

    @staticmethod
    def _attitude_parameter_names():
        return (
            "cam_l_optical_roll_deg",
            "cam_l_optical_pitch_deg",
            "cam_l_optical_yaw_deg",
            "cam_r_optical_roll_deg",
            "cam_r_optical_pitch_deg",
            "cam_r_optical_yaw_deg",
            "cam_l_px4_roll_deg",
            "cam_l_px4_pitch_deg",
            "cam_l_px4_yaw_deg",
            "cam_r_px4_roll_deg",
            "cam_r_px4_pitch_deg",
            "cam_r_px4_yaw_deg",
            "common_optical_roll_error_deg",
            "relative_optical_roll_error_deg",
            "common_optical_pitch_error_deg",
            "relative_optical_pitch_error_deg",
            "common_optical_yaw_error_deg",
            "relative_optical_yaw_error_deg",
        )

    def _configure_attitude_error_state(self):
        left, right = effective_optical_rpy_errors_deg(
            (
                self.cam_l_optical_roll_deg,
                self.cam_l_optical_pitch_deg,
                self.cam_l_optical_yaw_deg,
            ),
            (
                self.cam_r_optical_roll_deg,
                self.cam_r_optical_pitch_deg,
                self.cam_r_optical_yaw_deg,
            ),
            (
                self.common_optical_roll_error_deg,
                self.common_optical_pitch_error_deg,
                self.common_optical_yaw_error_deg,
            ),
            (
                self.relative_optical_roll_error_deg,
                self.relative_optical_pitch_error_deg,
                self.relative_optical_yaw_error_deg,
            ),
        )
        (
            self.effective_left_optical_roll_error_deg,
            self.effective_left_optical_pitch_error_deg,
            self.effective_left_optical_yaw_error_deg,
        ) = left
        (
            self.effective_right_optical_roll_error_deg,
            self.effective_right_optical_pitch_error_deg,
            self.effective_right_optical_yaw_error_deg,
        ) = right
        self.attitude_error_frame = "camera_optical_rdf"
        self.attitude_angle_unit = "deg"
        self.attitude_parameterization = "direct_plus_common_relative"
        self.attitude_rotation_composition = (
            "R_Cnominal_Cactual=R_Down(yaw)@R_Right(pitch)@R_Forward(roll)"
        )

    def _ensure_camera_frames(self):
        self._ensure_attitude_error_state()
        if hasattr(self, "baseline_center_frd"):
            return
        self.baseline_m = float(getattr(self, "baseline_m", 25.0))
        self.camera_height_m = float(getattr(self, "camera_height_m", 5.0))

        self._configure_camera_frames()

    @staticmethod
    def _finite_tuple(value, *, size, field):
        if not isinstance(value, (list, tuple)) or len(value) != size:
            raise ValueError(f"{field} must contain exactly {size} values")
        parsed = tuple(float(item) for item in value)
        if not all(math.isfinite(item) for item in parsed):
            raise ValueError(f"{field} must be finite")
        return parsed

    @staticmethod
    def _valid_sha256(value):
        return (
            isinstance(value, str)
            and len(value) == 64
            and all(character in "0123456789abcdef" for character in value)
        )

    def _parse_camera_pose_diagnostic(self, camera_label, encoded):
        try:
            payload = json.loads(str(encoded))
        except (TypeError, json.JSONDecodeError) as exc:
            raise ValueError("camera oracle diagnostic is not valid JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("camera oracle diagnostic must be a JSON object")
        if payload.get("schema") != "camera_pose_comparison/v1":
            raise ValueError("camera oracle diagnostic schema is unsupported")
        if payload.get("camera_label") != camera_label:
            raise ValueError("camera oracle diagnostic camera label does not match topic")
        expected_hash = getattr(self, "camera_oracle_generated_sdf_sha256", "")
        received_hash = payload.get("generated_sdf_sha256")
        if not self._valid_sha256(expected_hash):
            raise ValueError("camera oracle expected generated SDF SHA-256 is unavailable")
        if received_hash != expected_hash:
            raise ValueError("camera oracle generated SDF SHA-256 does not match launch world")
        expected_model = f"{camera_label}_stereo_camera"
        if payload.get("observed_base_link_entity_name") != f"{expected_model}::base_link":
            raise ValueError("camera oracle base-link entity does not match expected rig")
        if payload.get("observed_optical_entity_name") != f"{expected_model}::optical_link":
            raise ValueError("camera oracle optical entity does not match expected rig")
        base_id = payload.get("observed_base_link_entity_id")
        optical_id = payload.get("observed_optical_entity_id")
        if any(not isinstance(value, int) or isinstance(value, bool) for value in (base_id, optical_id)):
            raise ValueError("camera oracle entity IDs must be integers")
        stamp_ns = payload.get("gazebo_stamp_ns")
        if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or stamp_ns <= 0:
            raise ValueError("camera oracle Gazebo stamp must be a positive integer")
        position = self._finite_tuple(
            payload.get("observed_optical_position_world_enu_m"),
            size=3,
            field="observed_optical_position_world_enu_m",
        )
        orientation = self._finite_tuple(
            payload.get("observed_optical_orientation_world_from_entity_xyzw"),
            size=4,
            field="observed_optical_orientation_world_from_entity_xyzw",
        )
        pose = camera_optical_pose_from_quaternion(position, orientation)
        return ObservedCameraOraclePose(
            camera_label=camera_label,
            pose=pose,
            gazebo_stamp_ns=stamp_ns,
            generated_sdf_sha256=received_hash,
            base_entity_name=payload["observed_base_link_entity_name"],
            base_entity_id=base_id,
            optical_entity_name=payload["observed_optical_entity_name"],
            optical_entity_id=optical_id,
            optical_position_world_enu_m=position,
            optical_orientation_world_from_entity_xyzw=orientation,
            simulation_epoch=getattr(getattr(self, "simulation_epoch", None), "epoch", 0),
            received_monotonic_s=time.monotonic(),
        )

    def _camera_pose_diagnostic_callback(self, camera_label, msg):
        try:
            observation = self._parse_camera_pose_diagnostic(camera_label, msg.data)
        except (TypeError, ValueError) as exc:
            condition = getattr(self, "yolo_condition", None)
            if condition is None:
                setattr(self, f"observed_{camera_label}_camera", None)
                self.camera_oracle_pair = (
                    None if camera_label == "left" else getattr(self, "observed_left_camera", None),
                    None if camera_label == "right" else getattr(self, "observed_right_camera", None),
                )
                self.camera_oracle_rejection_count = getattr(self, "camera_oracle_rejection_count", 0) + 1
                self.last_camera_oracle_rejection_reason = str(exc)
            else:
                with condition:
                    setattr(self, f"observed_{camera_label}_camera", None)
                    self.camera_oracle_pair = (
                        None if camera_label == "left" else self.observed_left_camera,
                        None if camera_label == "right" else self.observed_right_camera,
                    )
                    self.camera_oracle_rejection_count += 1
                    self.last_camera_oracle_rejection_reason = str(exc)
            get_logger = getattr(self, "get_logger", None)
            if get_logger is not None:
                get_logger().warning(f"Rejected {camera_label} camera oracle diagnostic: {exc}")
            return False
        condition = getattr(self, "yolo_condition", None)
        if condition is None:
            setattr(self, f"observed_{camera_label}_camera", observation)
            self.camera_oracle_pair = (
                observation if camera_label == "left" else getattr(self, "observed_left_camera", None),
                observation if camera_label == "right" else getattr(self, "observed_right_camera", None),
            )
        else:
            with condition:
                setattr(self, f"observed_{camera_label}_camera", observation)
                self.camera_oracle_pair = (
                    observation if camera_label == "left" else self.observed_left_camera,
                    observation if camera_label == "right" else self.observed_right_camera,
                )
                self.last_camera_oracle_rejection_reason = ""
        return True

    def _camera_oracle_snapshot(self):
        pair = getattr(self, "camera_oracle_pair", None)
        if pair is None:
            left = getattr(self, "observed_left_camera", None)
            right = getattr(self, "observed_right_camera", None)
        else:
            left, right = pair
        if left is None or right is None:
            return None
        current_epoch = getattr(getattr(self, "simulation_epoch", None), "epoch", 0)
        max_age_s = float(getattr(self, "camera_oracle_max_age_s", 2.5))
        now = time.monotonic()
        if (
            left.simulation_epoch != current_epoch
            or right.simulation_epoch != current_epoch
            or now - left.received_monotonic_s > max_age_s
            or now - right.received_monotonic_s > max_age_s
        ):
            return None
        return left, right

    def _gps_baseline_callback(self, camera_label, message):
        """Retain the newest valid raw GPS fix for baseline-only comparison."""

        try:
            fix = RawGpsFix(
                timestamp_us=int(message.timestamp_sample or message.timestamp),
                latitude_deg=float(message.latitude_deg),
                longitude_deg=float(message.longitude_deg),
                altitude_msl_m=float(message.altitude_msl_m),
                eph_m=float(message.eph),
                epv_m=float(message.epv),
            )
            if fix.timestamp_us <= 0 or not all(
                math.isfinite(value)
                for value in (
                    fix.latitude_deg,
                    fix.longitude_deg,
                    fix.altitude_msl_m,
                    fix.eph_m,
                    fix.epv_m,
                )
            ):
                raise ValueError("non-finite or unstamped raw GPS fix")
        except (AttributeError, TypeError, ValueError):
            return
        with self.lock:
            self.gps_baseline_fixes[camera_label] = (fix, time.monotonic())

    def _global_baseline_callback(self, camera_label, message):
        """Retain the newest valid PX4 fused-global-position fix."""

        try:
            if not bool(message.lat_lon_valid) or not bool(message.alt_valid):
                raise ValueError("invalid fused global position")
            fix = RawGpsFix(
                timestamp_us=int(message.timestamp_sample or message.timestamp),
                latitude_deg=float(message.lat),
                longitude_deg=float(message.lon),
                altitude_msl_m=float(message.alt),
                eph_m=float(message.eph),
                epv_m=float(message.epv),
            )
            if fix.timestamp_us <= 0 or not all(
                math.isfinite(value)
                for value in (
                    fix.latitude_deg,
                    fix.longitude_deg,
                    fix.altitude_msl_m,
                    fix.eph_m,
                    fix.epv_m,
                )
            ):
                raise ValueError("non-finite or unstamped fused global position")
        except (AttributeError, TypeError, ValueError):
            return
        with self.lock:
            self.global_baseline_fixes[camera_label] = (fix, time.monotonic())

    def _gps_baseline_camera_poses(self, source):
        """Build nominal-attitude poses separated by one PX4 position source."""

        if not getattr(self, "enable_gps_baseline_triangulation", False):
            return None, "disabled", {}
        source = self._validate_gps_baseline_source(source)
        if source == "both":
            raise ValueError("both is an output selection, not one baseline source")
        fixes = (
            self.gps_baseline_fixes
            if source == "raw_gps"
            else self.global_baseline_fixes
        )
        with self.lock:
            left_entry = fixes.get("left")
            right_entry = fixes.get("right")
        if left_entry is None or right_entry is None:
            return None, "unavailable_raw_gps_fix", {}
        left_fix, left_received_s = left_entry
        right_fix, right_received_s = right_entry
        if time.monotonic() - min(left_received_s, right_received_s) > self.gps_baseline_max_age_s:
            return None, "stale_raw_gps_fix", {}
        try:
            baseline_enu = gps_baseline_enu_m(left_fix, right_fix)
        except ValueError:
            return None, "invalid_raw_gps_fix", {}
        left_center = self.nominal_left_camera.center_world_enu_m
        right_center = tuple(left_center[index] + baseline_enu[index] for index in range(3))
        poses = (
            replace(self.nominal_left_camera, center_world_enu_m=left_center),
            replace(self.nominal_right_camera, center_world_enu_m=right_center),
        )
        nominal_baseline = tuple(
            self.nominal_right_camera.center_world_enu_m[index]
            - self.nominal_left_camera.center_world_enu_m[index]
            for index in range(3)
        )
        baseline_delta = tuple(baseline_enu[index] - nominal_baseline[index] for index in range(3))
        return poses, "available", {
            "gps_baseline_source": source,
            "gps_baseline_world_enu_m": baseline_enu,
            "gps_baseline_length_m": norm_m(baseline_enu),
            "gps_baseline_delta_world_enu_m": baseline_delta,
            "gps_baseline_delta_norm_m": norm_m(baseline_delta),
            "gps_left_eph_m": left_fix.eph_m,
            "gps_right_eph_m": right_fix.eph_m,
            "gps_left_timestamp_us": left_fix.timestamp_us,
            "gps_right_timestamp_us": right_fix.timestamp_us,
        }

    def _camera_oracle_record_fields(self, snapshot=_CAMERA_ORACLE_UNSET):
        fields = {
            "camera_oracle_source": "unavailable",
            "camera_oracle_expected_sdf_sha256": getattr(
                self, "camera_oracle_generated_sdf_sha256", ""
            ),
        }
        if snapshot is _CAMERA_ORACLE_UNSET:
            snapshot = self._camera_oracle_snapshot()
        for label in ("left", "right"):
            for name in (
                "gazebo_stamp_ns", "base_entity_name", "base_entity_id", "optical_entity_name",
                "optical_entity_id", "optical_x_m", "optical_y_m", "optical_z_m",
                "optical_qx", "optical_qy", "optical_qz", "optical_qw",
            ):
                fields[f"{label}_camera_oracle_{name}"] = None
        if snapshot is None:
            return fields
        fields["camera_oracle_source"] = "gazebo_observed_optical"
        for label, observation in zip(("left", "right"), snapshot):
            fields.update(
                {
                    f"{label}_camera_oracle_gazebo_stamp_ns": observation.gazebo_stamp_ns,
                    f"{label}_camera_oracle_base_entity_name": observation.base_entity_name,
                    f"{label}_camera_oracle_base_entity_id": observation.base_entity_id,
                    f"{label}_camera_oracle_optical_entity_name": observation.optical_entity_name,
                    f"{label}_camera_oracle_optical_entity_id": observation.optical_entity_id,
                    f"{label}_camera_oracle_optical_x_m": observation.optical_position_world_enu_m[0],
                    f"{label}_camera_oracle_optical_y_m": observation.optical_position_world_enu_m[1],
                    f"{label}_camera_oracle_optical_z_m": observation.optical_position_world_enu_m[2],
                    f"{label}_camera_oracle_optical_qx": observation.optical_orientation_world_from_entity_xyzw[0],
                    f"{label}_camera_oracle_optical_qy": observation.optical_orientation_world_from_entity_xyzw[1],
                    f"{label}_camera_oracle_optical_qz": observation.optical_orientation_world_from_entity_xyzw[2],
                    f"{label}_camera_oracle_optical_qw": observation.optical_orientation_world_from_entity_xyzw[3],
                }
            )
        return fields

    def _ensure_attitude_error_state(self):
        for name in self._attitude_parameter_names():
            setattr(self, name, float(getattr(self, name, 0.0)))
        self._configure_attitude_error_state()

    def _camera_intrinsics(self, width, height):
        width = float(width)
        height = float(height)
        fx = (width * 0.5) / math.tan(float(self.horizontal_fov_rad) * 0.5)
        return PinholeIntrinsics(fx_px=fx, fy_px=fx, cx_px=width * 0.5, cy_px=height * 0.5)

    def _research_recording_enabled(self):
        return getattr(self, "research_recorder", None) is not None

    @staticmethod
    def _all_finite(values):
        return all(value is not None and math.isfinite(float(value)) for value in values)

    def _submit_research_terminal_locked(
        self,
        pair,
        *,
        terminal_status,
        terminal_reason,
        measurement_status,
        evaluation_status,
        metrics=None,
        alignment=None,
    ):
        """Submit one immutable terminal snapshot while holding the node lock.

        The recorder performs only validation plus ``put_nowait`` here.  File
        I/O, JSONL validation, CSV export, hashes, and final summary all run
        outside ``yolo_condition``.
        """

        recorder = getattr(self, "research_recorder", None)
        if recorder is None or pair is None:
            return False
        key = (int(pair.epoch), int(pair.pair_id))
        terminal_keys = getattr(self, "research_terminal_pair_keys", None)
        if terminal_keys is None:
            terminal_keys = set()
            self.research_terminal_pair_keys = terminal_keys
        if key in terminal_keys:
            return False
        terminal_keys.add(key)

        metrics = {} if metrics is None else dict(metrics)
        observation = self._observation_metadata()
        alignment = alignment or getattr(self, "gt_alignments_by_pair_id", {}).get(
            pair.pair_id
        )
        record = empty_pair_record()
        record.update(
            {
                "schema_version": RESEARCH_SCHEMA_VERSION,
                "run_id": self.research_run_id,
                "case_id": self.research_case_id,
                "record_sequence": None,
                "record_type": "pair_terminal",
                "pair_id": int(pair.pair_id),
                "epoch": int(pair.epoch),
                "left_stamp_ns": int(pair.left_stamp_ns),
                "right_stamp_ns": int(pair.right_stamp_ns),
                "pair_midpoint_stamp_ns": int(pair.pair_midpoint_stamp_ns),
                "pair_delta_ns": int(pair.pair_delta_ns),
                "pairing_mode": str(pair.pairing_mode),
                "terminal_status": str(terminal_status),
                "terminal_reason": str(terminal_reason),
                "measurement_status": str(measurement_status),
                "evaluation_status": str(evaluation_status),
                "gt_alignment_mode": (
                    "unavailable" if alignment is None else alignment.gt_alignment_mode
                ),
                "gt_before_stamp_ns": None if alignment is None else alignment.gt_before_stamp_ns,
                "gt_after_stamp_ns": None if alignment is None else alignment.gt_after_stamp_ns,
                "gt_interpolation_alpha": None if alignment is None else alignment.gt_interpolation_alpha,
                "gt_nearest_delta_ns": None if alignment is None else alignment.gt_nearest_delta_ns,
                "observation_mode": metrics.get(
                    "observation_mode", observation["observation_mode"]
                ),
                "observation_source": metrics.get(
                    "observation_source", observation["observation_source"]
                ),
                "measurement_gt_independent": metrics.get(
                    "measurement_gt_independent",
                    observation["measurement_gt_independent"],
                ),
                "reference_source": metrics.get(
                    "reference_source", observation["reference_source"]
                ),
                "projection_camera_pose_source": metrics.get(
                    "projection_camera_pose_source",
                    observation["projection_camera_pose_source"],
                ),
                "left_u_px": metrics.get("left_u_px"),
                "left_v_px": metrics.get("left_v_px"),
                "right_u_px": metrics.get("right_u_px"),
                "right_v_px": metrics.get("right_v_px"),
                "left_confidence": metrics.get("left_confidence"),
                "right_confidence": metrics.get("right_confidence"),
                "raw_computation_status": metrics.get(
                    "raw_computation_status", metrics.get("computation_status")
                ),
                "raw_forward_m": metrics.get("raw_forward_m", metrics.get("estimated_forward_m")),
                "raw_right_m": metrics.get("raw_right_m", metrics.get("estimated_right_m")),
                "raw_down_m": metrics.get("raw_down_m", metrics.get("estimated_down_m")),
                "raw_range_m": metrics.get("raw_range_m", metrics.get("estimated_range_m")),
                "raw_ray_gap_m": metrics.get("raw_ray_gap_m", metrics.get("ray_gap_m")),
                "raw_intersection_angle_deg": metrics.get(
                    "raw_intersection_angle_deg", metrics.get("intersection_angle_deg")
                ),
                "raw_left_depth_m": metrics.get("raw_left_depth_m", metrics.get("left_depth_m")),
                "raw_right_depth_m": metrics.get("raw_right_depth_m", metrics.get("right_depth_m")),
                "raw_geometry_rank": metrics.get("raw_geometry_rank", metrics.get("geometry_rank")),
                "raw_condition_number": metrics.get(
                    "raw_condition_number", metrics.get("condition_number")
                ),
                "corrected_computation_status": metrics.get(
                    "attitude_corrected_computation_status"
                ),
                "corrected_forward_m": metrics.get("attitude_corrected_forward_m"),
                "corrected_right_m": metrics.get("attitude_corrected_right_m"),
                "corrected_down_m": metrics.get("attitude_corrected_down_m"),
                "corrected_range_m": metrics.get("attitude_corrected_range_m"),
                "corrected_ray_gap_m": metrics.get("attitude_corrected_ray_gap_m"),
                "corrected_intersection_angle_deg": metrics.get(
                    "attitude_corrected_intersection_angle_deg"
                ),
                "corrected_left_depth_m": metrics.get("attitude_corrected_left_depth_m"),
                "corrected_right_depth_m": metrics.get("attitude_corrected_right_depth_m"),
                "corrected_geometry_rank": metrics.get("attitude_corrected_geometry_rank"),
                "corrected_condition_number": metrics.get(
                    "attitude_corrected_condition_number"
                ),
                "gt_source": "gazebo_reference_link_world_pose",
                "gt_source_entity": SOURCE_GAZEBO_ENTITY,
                "gt_reference_name": VEHICLE_REFERENCE_LINK,
                "gt_reference_definition": VEHICLE_REFERENCE_DEFINITION,
                "gt_fallback_used": False,
                "gt_forward_m": metrics.get("gt_forward_m"),
                "gt_right_m": metrics.get("gt_right_m"),
                "gt_down_m": metrics.get("gt_down_m"),
                "gt_range_m": metrics.get("gt_range_m"),
                "raw_position_error_m": metrics.get("raw_position_error_m"),
                "raw_forward_error_m": metrics.get("raw_forward_error_m"),
                "raw_right_error_m": metrics.get("raw_right_error_m"),
                "raw_down_error_m": metrics.get("raw_down_error_m"),
                "raw_range_error_m": metrics.get("raw_range_error_m"),
                "corrected_position_error_m": metrics.get("corrected_position_error_m"),
                "corrected_forward_error_m": metrics.get("corrected_forward_error_m"),
                "corrected_right_error_m": metrics.get("corrected_right_error_m"),
                "corrected_down_error_m": metrics.get("corrected_down_error_m"),
                "corrected_range_error_m": metrics.get("corrected_range_error_m"),
                "measurement_queue_depth": len(getattr(self, "measurement_pair_queue", ())),
                "measurement_queue_depth_max": getattr(self, "measurement_queue_depth_max", 0),
                "measurement_queue_capacity": getattr(self, "measurement_queue_capacity", 0),
                "measurement_queue_overflow_count": getattr(
                    self, "measurement_queue_overflow_count", 0
                ),
                "measurement_overrun_invalid": getattr(
                    self, "measurement_overrun_invalid", False
                ),
                "measurement_rejected_after_overrun_count": getattr(
                    self, "measurement_rejected_after_overrun_count", 0
                ),
                "measurement_accepted_count": getattr(self, "measurement_accepted_count", 0),
                "measurement_processed_count": getattr(self, "measurement_processed_count", 0),
                "accepted_pair_discard_count": getattr(
                    self, "accepted_pair_discard_count", 0
                ),
                "measurement_processing_error_count": getattr(
                    self, "measurement_processing_error_count", 0
                ),
                "gt_evaluation_success_count": getattr(
                    self, "gt_evaluation_success_count", 0
                ),
                "gt_evaluation_error_count": getattr(self, "gt_evaluation_error_count", 0),
                "research_record_queue_depth": recorder.queue_depth,
                "research_record_queue_depth_max": recorder.queue_depth_max,
                "research_record_queue_capacity": recorder.queue_capacity,
                "research_recording_overrun_invalid": recorder.overrun_invalid,
                "research_recording_error_count": recorder.error_count,
                "output_frame": BASELINE_CENTER_FRD_FRAME,
                "output_axes": "forward_right_down",
                "position_unit": "m",
                "pixel_unit": "px",
                "angle_unit": "deg",
                "timestamp_unit": "ns",
                "correction_source": "gazebo_observed_optical_pose",
            }
        )
        record.update(
            self._camera_oracle_record_fields(
                metrics.get("camera_oracle_snapshot", _CAMERA_ORACLE_UNSET),
            )
        )
        for name in self._attitude_parameter_names():
            record[name] = getattr(self, name)
        for name in (
            "effective_left_optical_roll_error_deg",
            "effective_left_optical_pitch_error_deg",
            "effective_left_optical_yaw_error_deg",
            "effective_right_optical_roll_error_deg",
            "effective_right_optical_pitch_error_deg",
            "effective_right_optical_yaw_error_deg",
        ):
            record[name] = getattr(self, name)

        record["sample_eligible"] = bool(
            terminal_status == "evaluated"
            and measurement_status == "processed"
            and evaluation_status == "evaluated"
            and self._all_finite(
                record[name]
                for name in ("left_u_px", "left_v_px", "right_u_px", "right_v_px")
            )
            and record["raw_computation_status"] == "computed"
            and record["corrected_computation_status"] == "computed"
            and self._all_finite(
                record[name]
                for name in (
                    "raw_forward_m", "raw_right_m", "raw_down_m", "raw_range_m",
                    "corrected_forward_m", "corrected_right_m", "corrected_down_m",
                    "corrected_range_m", "gt_forward_m", "gt_right_m", "gt_down_m",
                    "gt_range_m",
                )
            )
            and record["output_frame"] == BASELINE_CENTER_FRD_FRAME
        )
        submitted = recorder.submit(record)
        if submitted:
            self.research_terminal_record_count = getattr(
                self, "research_terminal_record_count", 0
            ) + 1
        else:
            self.research_recording_error_count = getattr(
                self, "research_recording_error_count", 0
            ) + 1
            self.research_recording_overrun_invalid = recorder.overrun_invalid
            if not getattr(self, "research_recording_first_error_type", ""):
                self.research_recording_first_error_type = recorder.first_error_type
                self.research_recording_first_error_message = recorder.first_error_message
        return submitted

    def _clock_cb(self, msg):
        try:
            stamp_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        except (AttributeError, TypeError, ValueError):
            self.get_logger().warning("Invalid /clock message ignored")
            return
        with self.yolo_condition:
            event = self.simulation_epoch.observe_clock(stamp_ns)
            if event.reset:
                self._reset_for_simulation_epoch_locked(event.epoch, event.stamp_ns)
                self.get_logger().warning(
                    f"Simulation /clock decreased; entered epoch {event.epoch}"
                )
            else:
                self._drain_future_ground_truth_locked(stamp_ns)

    def _reset_for_simulation_epoch_locked(self, epoch, reset_stamp_ns=None):
        self._ensure_gt_evaluation_state_locked()
        self.observed_left_camera = None
        self.observed_right_camera = None
        self.camera_oracle_pair = (None, None)
        self.last_camera_oracle_rejection_reason = "simulation_epoch_reset"
        pending_pairs = tuple(self.pairs_by_pair_id.values())
        discarded_pending = sum(
            pair.pair_id not in self.measurement_results_by_pair_id
            for pair in pending_pairs
        )
        self.accepted_pair_discard_count += discarded_pending
        for pair in pending_pairs:
            self._submit_research_terminal_locked(
                pair,
                terminal_status="simulation_reset_discarded",
                terminal_reason="simulation_clock_reset",
                measurement_status=(
                    "processed"
                    if pair.pair_id in self.measurement_results_by_pair_id
                    else "discarded"
                ),
                evaluation_status="not_evaluated",
                metrics=self.measurement_results_by_pair_id.get(pair.pair_id),
                alignment=self.gt_alignments_by_pair_id.get(pair.pair_id),
            )
        dropped = self.image_pairer.reset_for_epoch(epoch)
        dropped += self.pending_gt_aligner.clear()
        dropped += len(self.measurement_pair_queue)
        dropped += len(self.gt_alignment_ready_queue)
        self.measurement_pair_queue.clear()
        self.gt_alignment_ready_queue.clear()
        self.gt_position_buffer.reset_for_epoch(epoch)
        self.stale_pre_reset_gt_count += len(self.future_gt_pending)
        self.stale_pre_reset_gt_count += len(
            getattr(self, "gt_quarantine_candidates", ())
        )
        self.future_gt_pending.clear()
        self.gt_quarantine_candidates = deque()
        self.gt_quarantine_last_stamp_ns = None
        self.gt_quarantine_active = True
        if reset_stamp_ns is None:
            reset_stamp_ns = self.simulation_epoch.last_clock_stamp_ns
        reset_stamp_ns = 0 if reset_stamp_ns is None else int(reset_stamp_ns)
        duration_ns = int(
            getattr(self, "gt_reset_quarantine_duration_ns", 200_000_000)
        )
        self.gt_quarantine_floor_stamp_ns = reset_stamp_ns + duration_ns
        self.measurement_results_by_pair_id.clear()
        self.gt_alignments_by_pair_id.clear()
        self.pairs_by_pair_id.clear()
        self.evaluated_pair_ids.clear()
        self.gt_evaluation_failed_pair_ids.clear()
        self.gt_published_pair_ids.clear()
        self.cached_ground_truth_position = None
        self.cached_actual_distance = None
        self.ground_truth_stamp_ns = None
        self.old_epoch_frame_count += dropped

    def _register_measurement_pairs_locked(self, pairs):
        ready_count = 0
        for pair in pairs:
            if pair.epoch != self.simulation_epoch.epoch:
                self.old_epoch_frame_count += 1
                continue
            if self.measurement_overrun_invalid:
                self.measurement_rejected_after_overrun_count += 1
                self.measurement_rejected_pair_ids.add(pair.pair_id)
                self.last_measurement_rejection_reason = (
                    "measurement_run_invalid_after_queue_overrun"
                )
                if self.measurement_rejected_after_overrun_count == 1 or (
                    self.measurement_rejected_after_overrun_count % 100 == 0
                ):
                    self.get_logger().warning(
                        "Measurement input rejected after queue overrun invalidated "
                        f"the run: rejected_pair_id={pair.pair_id}"
                    )
                continue
            if len(self.measurement_pair_queue) >= self.measurement_queue_capacity:
                self.measurement_queue_overflow_count += 1
                self.measurement_overrun_invalid = True
                self.measurement_rejected_pair_ids.add(pair.pair_id)
                self.last_measurement_rejection_reason = "measurement_queue_overflow"
                self._drop_pair_state_locked(pair.pair_id, "measurement_queue_overflow")
                if self.measurement_queue_overflow_count == 1 or (
                    self.measurement_queue_overflow_count % 100 == 0
                ):
                    self.get_logger().error(
                        "Measurement queue overrun: strict research run is invalid; "
                        f"capacity={self.measurement_queue_capacity}, rejected_pair_id={pair.pair_id}"
                    )
                continue
            self.pairs_by_pair_id[pair.pair_id] = pair
            if getattr(self, "observation_mode", "bbox_yolo") == "bbox_yolo":
                self.measurement_pair_queue.append(pair)
            alignments = self.pending_gt_aligner.add_pair(pair)
            self.gt_alignment_ready_queue.extend(alignments)
            self.measurement_accepted_count += 1
            self._record_measurement_queue_depth_locked()
            ready_count += 1
        if ready_count:
            self.yolo_condition.notify_all()
        return ready_count

    def _record_measurement_queue_depth_locked(self):
        depth = len(self.measurement_pair_queue)
        self.measurement_queue_depth_sample_count += 1
        self.measurement_queue_depth_sum += depth
        self.measurement_queue_depth_min = (
            depth if self.measurement_queue_depth_min is None
            else min(self.measurement_queue_depth_min, depth)
        )
        self.measurement_queue_depth_max = max(self.measurement_queue_depth_max, depth)

    def _drop_pair_state_locked(self, pair_id, reason):
        """Remove every pending state for one pair under ``yolo_condition``."""

        pair_id = int(pair_id)
        self.measurement_pair_queue = deque(
            pair for pair in self.measurement_pair_queue if pair.pair_id != pair_id
        )
        self.gt_alignment_ready_queue = deque(
            item for item in self.gt_alignment_ready_queue if item[0].pair_id != pair_id
        )
        self.pending_gt_aligner.remove_pair(pair_id)
        self.pairs_by_pair_id.pop(pair_id, None)
        self.measurement_results_by_pair_id.pop(pair_id, None)
        self.gt_alignments_by_pair_id.pop(pair_id, None)
        self.evaluated_pair_ids.discard(pair_id)
        self.gt_published_pair_ids.discard(pair_id)
        return reason

    def _finalize_measurement_queue_overflow_locked(self, pair_id):
        """Terminally reject one already-accepted pair on queue overflow."""
        pair_id = int(pair_id)
        if pair_id not in self.pairs_by_pair_id:
            return False
        self.measurement_queue_overflow_count += 1
        self.measurement_overrun_invalid = True
        self.measurement_rejected_pair_ids.add(pair_id)
        self.accepted_pair_discard_count += 1
        self.last_measurement_rejection_reason = "measurement_queue_overflow"
        self._submit_research_terminal_locked(
            self.pairs_by_pair_id.get(pair_id),
            terminal_status="measurement_queue_overflow",
            terminal_reason="measurement_queue_overflow",
            measurement_status="discarded",
            evaluation_status="not_evaluated",
            metrics=self.measurement_results_by_pair_id.get(pair_id),
            alignment=self.gt_alignments_by_pair_id.get(pair_id),
        )
        self._drop_pair_state_locked(pair_id, "measurement_queue_overflow")
        return True

    def _ground_truth_pose_cb(self, msg):
        if str(msg.header.frame_id) != WORLD_FRAME:
            self.get_logger().warning(
                f"Ground Truth ignored: expected frame_id={WORLD_FRAME}, got {msg.header.frame_id!r}"
            )
            return
        self._ensure_camera_frames()
        point_world = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        orientation_xyzw = (
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        self._publish_actual_drone_marker(point_world, orientation_xyzw, stamp_ns)
        try:
            position = ground_truth_world_to_baseline(point_world, self.baseline_center_frd)
            if hasattr(self, "gt_position_buffer"):
                with self.yolo_condition:
                    if not self.accepting_input:
                        return
                    record = PendingGroundTruthRecord(
                        stamp_ns=stamp_ns,
                        epoch=self.simulation_epoch.epoch,
                        position_world_enu_m=point_world,
                        orientation_world_from_reference_xyzw=orientation_xyzw,
                    )
                    self._route_ground_truth_record_locked(record, position)
        except (TypeError, ValueError) as exc:
            self.inconsistent_clock_count += 1
            self.get_logger().warning(f"Ground Truth ignored: {exc}")
            return

    def _reject_ground_truth_record_locked(self, *, quarantine=False):
        self.stale_pre_reset_gt_count += 1
        if quarantine:
            self.gt_quarantine_rejected_count = getattr(
                self, "gt_quarantine_rejected_count", 0
            ) + 1

    def _route_ground_truth_record_locked(self, record, position=None):
        if record.epoch != self.simulation_epoch.epoch:
            self._reject_ground_truth_record_locked(quarantine=True)
            return False
        floor_stamp_ns = getattr(self, "gt_quarantine_floor_stamp_ns", None)
        if floor_stamp_ns is not None and record.stamp_ns <= floor_stamp_ns:
            self._reject_ground_truth_record_locked(quarantine=True)
            return False
        if getattr(self, "gt_quarantine_active", False):
            latest_clock = self.simulation_epoch.last_clock_stamp_ns
            if (
                latest_clock is not None
                and record.stamp_ns - latest_clock > self.gt_reorder_lead_limit_ns
            ):
                self._reject_ground_truth_record_locked(quarantine=True)
                return False
            candidates = getattr(self, "gt_quarantine_candidates", deque())
            last_stamp_ns = getattr(self, "gt_quarantine_last_stamp_ns", None)
            if last_stamp_ns is not None and record.stamp_ns <= last_stamp_ns:
                self.stale_pre_reset_gt_count += len(candidates)
                self.gt_quarantine_rejected_count += len(candidates)
                candidates.clear()
            if len(candidates) >= self.future_gt_capacity:
                candidates.popleft()
                self.future_gt_overflow_count += 1
            candidates.append(record)
            self.gt_quarantine_candidates = candidates
            self.gt_quarantine_last_stamp_ns = record.stamp_ns
            self._try_release_gt_quarantine_locked()
            return False
        return self._queue_or_accept_ground_truth_record_locked(record, position)

    def _try_release_gt_quarantine_locked(self):
        if not getattr(self, "gt_quarantine_active", False):
            return False
        latest_clock = self.simulation_epoch.last_clock_stamp_ns
        floor_stamp_ns = getattr(self, "gt_quarantine_floor_stamp_ns", None)
        required = int(
            getattr(self, "gt_quarantine_required_contiguous_samples", 2)
        )
        candidates = getattr(self, "gt_quarantine_candidates", deque())
        if (
            latest_clock is None
            or floor_stamp_ns is None
            or latest_clock < floor_stamp_ns
            or len(candidates) < required
        ):
            return False
        self.gt_quarantine_active = False
        self.gt_quarantine_release_count = getattr(
            self, "gt_quarantine_release_count", 0
        ) + 1
        self.gt_quarantine_candidates = deque()
        self.gt_quarantine_last_stamp_ns = None
        for record in tuple(candidates):
            self._queue_or_accept_ground_truth_record_locked(record)
        return True

    def _queue_or_accept_ground_truth_record_locked(self, record, position=None):
        latest_clock = self.simulation_epoch.last_clock_stamp_ns
        if latest_clock is not None and record.stamp_ns > latest_clock:
            lead_ns = record.stamp_ns - latest_clock
            if lead_ns > self.gt_reorder_lead_limit_ns:
                self._reject_ground_truth_record_locked()
                self.get_logger().warning(
                    "Ground Truth future stamp rejected as stale pre-reset data: "
                    f"stamp={record.stamp_ns}, clock={latest_clock}, lead_ns={lead_ns}"
                )
                return False
            if len(self.future_gt_pending) >= self.future_gt_capacity:
                self.future_gt_pending.popleft()
                self.future_gt_overflow_count += 1
            self.future_gt_pending.append(record)
            return False
        return self._accept_ground_truth_locked(record, position)

    def _accept_ground_truth_locked(self, record, position=None):
        if record.epoch != self.simulation_epoch.epoch:
            self._reject_ground_truth_record_locked(quarantine=True)
            return False
        if position is None:
            position = ground_truth_world_to_baseline(
                record.position_world_enu_m, self.baseline_center_frd
            )
        self.gt_position_buffer.add(
            record.stamp_ns,
            record.position_world_enu_m,
            record.orientation_world_from_reference_xyzw,
            epoch=record.epoch,
        )
        aligned_pairs = self.pending_gt_aligner.pop_watermarked()
        self.gt_alignment_ready_queue.extend(aligned_pairs)
        if aligned_pairs:
            self.yolo_condition.notify_all()
        position = dict(position)
        position.update(
            {
                "ground_truth_source": "gazebo_vehicle_reference_link_world_pose",
                "source_gazebo_entity": SOURCE_GAZEBO_ENTITY,
                "published_reference_name": VEHICLE_REFERENCE_LINK,
                "reference_definition": VEHICLE_REFERENCE_DEFINITION,
                "ground_truth_world_enu_m": record.position_world_enu_m,
                "ground_truth_stamp_ns": record.stamp_ns,
            }
        )
        self.cached_ground_truth_position = position
        self.ground_truth_stamp_ns = position["ground_truth_stamp_ns"]
        self.cached_actual_distance = float(position["forward_m"])
        self.actual_distance_sample_seq = getattr(self, "actual_distance_sample_seq", 0) + 1
        return True

    def _drain_future_ground_truth_locked(self, clock_stamp_ns):
        self._try_release_gt_quarantine_locked()
        ready = []
        pending = deque()
        while self.future_gt_pending:
            item = self.future_gt_pending.popleft()
            if item.epoch != self.simulation_epoch.epoch:
                self._reject_ground_truth_record_locked(quarantine=True)
            elif item.stamp_ns <= clock_stamp_ns:
                ready.append(item)
            else:
                pending.append(item)
        self.future_gt_pending = pending
        for record in sorted(ready, key=lambda item: item.stamp_ns):
            self._accept_ground_truth_locked(record)

    def _triangulate_pixel_centers_with_cameras(
        self,
        left_u_px,
        left_v_px,
        right_u_px,
        right_v_px,
        width,
        height,
        *,
        left_camera_pose,
        right_camera_pose,
    ):
        """Triangulate pixels using explicitly supplied optical camera poses."""

        self._ensure_camera_frames()
        pixels = (float(left_u_px), float(left_v_px), float(right_u_px), float(right_v_px))
        base = {
            "left_u_px": pixels[0],
            "left_v_px": pixels[1],
            "right_u_px": pixels[2],
            "right_v_px": pixels[3],
            "disparity_px": pixels[0] - pixels[2],
            "point_on_left_ray_baseline_center_frd_m": None,
            "point_on_right_ray_baseline_center_frd_m": None,
            "midpoint_baseline_center_frd_m": None,
            "forward_m": None,
            "right_m": None,
            "down_m": None,
            "range_m": None,
            "distance_m": None,
            "ray_gap_m": None,
            "intersection_angle_deg": None,
            "left_depth_m": None,
            "right_depth_m": None,
            "geometry_rank": None,
            "condition_number": None,
            "computation_status": "invalid_input",
            "output_frame": BASELINE_CENTER_FRD_FRAME,
            "point_frame": BASELINE_CENTER_FRD_FRAME,
            "axis_contract": BASELINE_CENTER_FRD_AXIS_CONTRACT,
            "vector_contract": BASELINE_CENTER_FRD_VECTOR_CONTRACT,
        }
        try:
            intrinsics = self._camera_intrinsics(width, height)
            left_ray_rdf = ray_from_pixel(pixels[0], pixels[1], intrinsics)
            right_ray_rdf = ray_from_pixel(pixels[2], pixels[3], intrinsics)
            left_ray_world = left_camera_pose.ray_to_world(left_ray_rdf)
            right_ray_world = right_camera_pose.ray_to_world(right_ray_rdf)
            left_origin = self.baseline_center_frd.point_from_world(left_camera_pose.center_world_enu_m)
            right_origin = self.baseline_center_frd.point_from_world(right_camera_pose.center_world_enu_m)
            left_ray = self.baseline_center_frd.ray_from_world(left_ray_world)
            right_ray = self.baseline_center_frd.ray_from_world(right_ray_world)
            attempt = triangulate_rays(
                left_origin_left_rdf_m=left_origin,
                right_origin_left_rdf_m=right_origin,
                left_ray_left_rdf=left_ray,
                right_ray_left_rdf=right_ray,
                common_frame=BASELINE_CENTER_FRD_FRAME,
                axis_contract=BASELINE_CENTER_FRD_AXIS_CONTRACT,
                vector_contract=BASELINE_CENTER_FRD_VECTOR_CONTRACT,
                geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
            )
        except (TypeError, ValueError, ArithmeticError) as exc:
            base["computation_status"] = "invalid_input"
            base["diagnostic_error"] = str(exc)
            return base
        result = attempt.result
        base["geometry_rank"] = attempt.diagnostics.get("geometry_rank")
        base["condition_number"] = attempt.diagnostics.get("condition_number")
        if result is None:
            base["computation_status"] = attempt.reason
            return base
        midpoint = result.midpoint_rdf_m or result.point_left_rdf_m
        left_point = result.point_on_left_ray_rdf_m
        right_point = result.point_on_right_ray_rdf_m
        base.update(
            {
                "point_on_left_ray_baseline_center_frd_m": left_point,
                "point_on_right_ray_baseline_center_frd_m": right_point,
                "midpoint_baseline_center_frd_m": midpoint,
                "forward_m": float(midpoint[0]),
                "right_m": float(midpoint[1]),
                "down_m": float(midpoint[2]),
                "range_m": math.sqrt(sum(float(value) * float(value) for value in midpoint)),
                "distance_m": float(midpoint[0]),
                "ray_gap_m": float(result.closest_gap_m),
                "intersection_angle_deg": float(result.ray_angle_deg),
                "left_depth_m": float(result.left_depth_m),
                "right_depth_m": float(result.right_depth_m),
                "geometry_rank": int(result.geometry_rank),
                "condition_number": float(result.condition_number),
                "computation_status": result.computation_status,
                "point_frame": result.point_frame,
                "output_frame": result.output_frame,
                "axis_contract": result.axis_contract,
                "vector_contract": result.vector_contract,
                "ray_visualization_world_enu": ray_visualization_world_enu_from_triangulation(
                    self.baseline_center_frd,
                    left_camera_pose,
                    right_camera_pose,
                    left_point,
                    right_point,
                    midpoint,
                ),
            }
        )
        return base

    def _triangulate_pixel_centers(self, left_u_px, left_v_px, right_u_px, right_v_px, width, height):
        """Canonical Raw: interpret pixels with nominal zero-error camera poses."""

        self._ensure_camera_frames()
        return self._triangulate_pixel_centers_with_cameras(
            left_u_px,
            left_v_px,
            right_u_px,
            right_v_px,
            width,
            height,
            left_camera_pose=self.nominal_left_camera,
            right_camera_pose=self.nominal_right_camera,
        )

    def _triangulate_dkl_ideal_from_reference_world(self, point_world_enu_m, width, height):
        """Triangulate DKL-style ideal pixels in the public baseline FRD frame.

        This is deliberately a parallel simulation reference, not an input to
        YOLO, Optical, or PX4 baseline processing.  Its target and camera poses
        are World ENU; the returned triangulation follows the normal public
        ``baseline_center_frd`` F/R/D(/Range) contract.
        """

        self._ensure_camera_frames()
        try:
            intrinsics = self._camera_intrinsics(width, height)
            pixels = dkl_ideal_pixels_from_world(
                point_world_enu_m,
                self.nominal_left_camera,
                self.nominal_right_camera,
                intrinsics,
                width,
                height,
            )
        except (TypeError, ValueError, ArithmeticError):
            return None
        if pixels is None:
            return None
        return self._triangulate_pixel_centers(*pixels, width, height)

    def _triangulate_pixel_centers_oracle(
        self, left_u_px, left_v_px, right_u_px, right_v_px, width, height, *, camera_oracle_snapshot=_CAMERA_ORACLE_UNSET
    ):
        """Triangulate only with the validated Gazebo-observed optical poses."""

        self._ensure_camera_frames()
        snapshot = camera_oracle_snapshot
        if snapshot is _CAMERA_ORACLE_UNSET:
            snapshot = self._camera_oracle_snapshot()
        if snapshot is None:
            return None
        left_camera, right_camera = snapshot
        return self._triangulate_pixel_centers_with_cameras(
            left_u_px,
            left_v_px,
            right_u_px,
            right_v_px,
            width,
            height,
            left_camera_pose=left_camera.pose,
            right_camera_pose=right_camera.pose,
        )

    def _triangulate_pixel_centers_with_gps_baseline(
        self, left_u_px, left_v_px, right_u_px, right_v_px, width, height, source
    ):
        """Parallel triangulation with one PX4-derived camera separation.

        The left nominal center is retained as the anchor.  This intentionally
        isolates the *baseline* error from any unknown global GPS translation.
        Optical attitudes and image measurements are unchanged.
        """

        poses, status, provenance = self._gps_baseline_camera_poses(source)
        if poses is None:
            return {"computation_status": status, **provenance}
        result = self._triangulate_pixel_centers_with_cameras(
            left_u_px,
            left_v_px,
            right_u_px,
            right_v_px,
            width,
            height,
            left_camera_pose=poses[0],
            right_camera_pose=poses[1],
        )
        result.update(provenance)
        result["baseline_source"] = (
            "px4_raw_vehicle_gps_position_baseline_only"
            if source == "raw_gps"
            else "px4_vehicle_global_position_baseline_only"
        )
        return result

    def _gps_baseline_sources(self):
        source = self.gps_baseline_source
        return ("raw_gps", "global_position") if source == "both" else (source,)

    def _triangulate_gps_baselines(
        self, left_u_px, left_v_px, right_u_px, right_v_px, width, height
    ):
        return {
            source: self._triangulate_pixel_centers_with_gps_baseline(
                left_u_px, left_v_px, right_u_px, right_v_px, width, height, source
            )
            for source in self._gps_baseline_sources()
        }

    def _on_param_update(self, params):
        if any(param.name in self._recording_parameter_names() for param in params):
            return SetParametersResult(
                successful=False,
                reason="research recording and camera sampling parameters are immutable for one run",
            )
        if any(param.name == "observation_mode" for param in params):
            return SetParametersResult(
                successful=False,
                reason="observation_mode is immutable for one run",
            )
        for param in params:
            if param.name == "gps_baseline_source":
                try:
                    self._validate_gps_baseline_source(param.value)
                except ValueError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
        if any(param.name == "measurement_queue_capacity" for param in params):
            return SetParametersResult(
                successful=False,
                reason="measurement queue capacity is immutable for one run",
            )
        geometry_parameter_names = {
            "baseline_m",
            "camera_height_m",
            "horizontal_fov_rad",
            "target_model_name",
            "cam_l_optical_roll_deg",
            "cam_l_optical_pitch_deg",
            "cam_l_optical_yaw_deg",
            "cam_r_optical_roll_deg",
            "cam_r_optical_pitch_deg",
            "cam_r_optical_yaw_deg",
            "cam_l_px4_roll_deg",
            "cam_l_px4_pitch_deg",
            "cam_l_px4_yaw_deg",
            "cam_r_px4_roll_deg",
            "cam_r_px4_pitch_deg",
            "cam_r_px4_yaw_deg",
            "common_optical_roll_error_deg",
            "relative_optical_roll_error_deg",
            "common_optical_pitch_error_deg",
            "relative_optical_pitch_error_deg",
            "common_optical_yaw_error_deg",
            "relative_optical_yaw_error_deg",
        }
        if any(param.name in geometry_parameter_names for param in params):
            return SetParametersResult(
                successful=False,
                reason="geometry-defining parameters are immutable for one run",
            )
        for param in params:
            if param.name == "gps_baseline_source":
                self.gps_baseline_source = self._validate_gps_baseline_source(param.value)
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
            elif param.name == "target_spawn_z_m":
                self.target_spawn_z_m = float(param.value)
            elif param.name == "fly_target_in_gazebo":
                self.fly_target_in_gazebo = self._coerce_bool(param.value)
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
        if self.observation_mode == "ideal_reference_point":
            # The ideal-reference path still needs synchronized images to form
            # a measurement pair.  It does not run YOLO, so a zero YOLO rate
            # must not disable the camera subscriptions used for this oracle
            # and GPS-baseline comparison mode.
            return True
        if self.yolo_target_hz <= 0.0:
            return False
        return (
            self.observation_mode == "bbox_yolo"
            and bool(self.yolo_model_path.strip())
            and not self.yolo_runtime_disabled
        )

    def _should_load_yolo_model(self):
        return (
            self.observation_mode == "bbox_yolo"
            and self.yolo_target_hz > 0.0
            and bool(self.yolo_model_path.strip())
            and not self.yolo_runtime_disabled
        )

    def _measurement_processing_hz(self):
        """Return the input-processing rate for the active observation mode."""

        observation_mode = getattr(self, "observation_mode", "bbox_yolo")
        yolo_target_hz = getattr(self, "yolo_target_hz", 0.1)
        source_hz = (
            getattr(self, "camera_update_rate_hz", yolo_target_hz)
            if observation_mode == "ideal_reference_point"
            else yolo_target_hz
        )
        return max(0.1, float(source_hz))

    def _initialize_yolo_runtime_if_enabled(self):
        if not self._should_load_yolo_model():
            return False
        self.resolved_yolo_device = self._resolve_yolo_device()
        self._load_yolo_model()
        return True

    def _create_camera_image_subscriptions_if_enabled(self, camera_qos):
        self.image_subscriptions = []
        if not self._should_subscribe_camera_images():
            return False
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
        return True

    def _start_yolo_worker_if_enabled(self):
        self.yolo_thread = None
        if not self._should_subscribe_camera_images():
            return False
        self.yolo_thread = threading.Thread(
            target=self._yolo_worker_loop,
            daemon=True,
        )
        self.yolo_thread.start()
        return True

    def _disable_yolo_runtime(self, reason):
        self.yolo_runtime_disabled = True
        self.yolo_runtime_disable_reason = str(reason)
        self.yolo_load_error = str(reason)
        if not self.yolo_runtime_warning_logged:
            self.get_logger().warning(str(reason))
            self.yolo_runtime_warning_logged = True

    def _actual_distance(self):
        if self.cached_ground_truth_position is None:
            return None
        return self.cached_actual_distance

    def _mission_state(self):
        elapsed = time.monotonic() - self.start_time
        start = self.approach_start_distance_m
        end = self.approach_end_distance_m
        initial_z = self.target_spawn_z_m
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
        actual = self._read_actual_distance()
        if actual is not None:
            self.cached_actual_distance = actual
            self.reference_distance_m = actual
        with self.lock:
            self.metrics["actual_distance_m"] = actual
            self.metrics["mission_phase"] = state["phase"]
        if actual is not None:
            self.actual_distance_pub.publish(Float32(data=float(actual)))
            if self.cached_ground_truth_position is not None:
                self._publish_ground_truth_position()
        if self.fly_target_in_gazebo:
            self._command_gazebo_target_pose(state)
        self._publish_plot_samples()

    def _publish_ground_truth_position(self):
        position = self.cached_ground_truth_position
        if position is None:
            return
        self.ground_truth_position_pub.publish(
            Float32MultiArray(
                data=[
                    float(position["forward_m"]),
                    float(position["right_m"]),
                    float(position["down_m"]),
                    float(position["range_m"]),
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

        actual = metrics.get("actual_distance_m", self._actual_distance())
        distance = metrics.get("distance_m")
        error = metrics.get("distance_error_m")
        disparity = metrics.get("disparity_px")
        estimated_right = metrics.get("estimated_right_m")
        estimated_down = metrics.get("estimated_down_m")
        estimated_forward = metrics.get("estimated_forward_m")
        estimated_range = metrics.get("estimated_range_m")
        corrected_right = metrics.get("corrected_right_m")
        corrected_down = metrics.get("corrected_down_m")
        corrected_forward = metrics.get("corrected_forward_m")
        corrected_range = metrics.get("corrected_range_m")
        optical_right = metrics.get("optical_right_m")
        optical_down = metrics.get("optical_down_m")
        optical_forward = metrics.get("optical_forward_m")
        optical_range = metrics.get("optical_range_m")
        ideal_right = metrics.get("ideal_right_m")
        ideal_down = metrics.get("ideal_down_m")
        ideal_forward = metrics.get("ideal_forward_m")
        ideal_range = metrics.get("ideal_range_m")
        bbox_adjustment = metrics.get("bbox_adjustment")

        distance_value = math.nan if distance is None else float(distance)
        error_value = math.nan if error is None else float(error)
        disparity_value = math.nan if disparity is None else float(disparity)
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
        optical_position_values = [
            math.nan if optical_forward is None else float(optical_forward),
            math.nan if optical_right is None else float(optical_right),
            math.nan if optical_down is None else float(optical_down),
            math.nan if optical_range is None else float(optical_range),
        ]
        ideal_position_values = [
            math.nan if ideal_forward is None else float(ideal_forward),
            math.nan if ideal_right is None else float(ideal_right),
            math.nan if ideal_down is None else float(ideal_down),
            math.nan if ideal_range is None else float(ideal_range),
        ]

        self.distance_pub.publish(Float32(data=distance_value))
        self.error_pub.publish(Float32(data=error_value))
        self.disparity_pub.publish(Float32(data=disparity_value))
        self.estimated_position_pub.publish(Float32MultiArray(data=estimated_position_values))
        self.corrected_position_pub.publish(Float32MultiArray(data=corrected_position_values))
        optical_publisher = getattr(self, "optical_position_pub", None)
        if optical_publisher is not None:
            optical_publisher.publish(Float32MultiArray(data=optical_position_values))
        ideal_publisher = getattr(self, "ideal_position_pub", None)
        if ideal_publisher is not None:
            ideal_publisher.publish(Float32MultiArray(data=ideal_position_values))
        if bbox_adjustment is not None:
            self.bbox_adjustment_pub.publish(self._bbox_adjustment_to_msg(bbox_adjustment))
        self.profile_status_pub.publish(self._profile_status_to_msg(metrics))
        self.comparison_pub.publish(
            Float32MultiArray(
                data=[
                    math.nan if actual is None else float(actual),
                    distance_value,
                    error_value,
                    disparity_value,
                ]
            )
        )

    def _read_actual_distance(self):
        if self.cached_ground_truth_position is None:
            return None
        return self.cached_actual_distance

    def _finalize_research_recording(self):
        recorder = getattr(self, "research_recorder", None)
        if recorder is None:
            return None
        try:
            summary = recorder.close(
                {
                    "run_id": self.research_run_id,
                    "case_id": self.research_case_id,
                    "shutdown_drain_status": self.shutdown_drain_status,
                    "accepted": self.measurement_accepted_count,
                    "processed": self.measurement_processed_count,
                    "discarded": self.accepted_pair_discard_count,
                    "measurement_queue_overflow_count": self.measurement_queue_overflow_count,
                    "measurement_overrun_invalid": self.measurement_overrun_invalid,
                    "measurement_rejected_after_overrun_count": self.measurement_rejected_after_overrun_count,
                    "measurement_processing_error_count": self.measurement_processing_error_count,
                    "gt_evaluation_success_count": self.gt_evaluation_success_count,
                    "gt_evaluation_error_count": self.gt_evaluation_error_count,
                    "simulation_reset_count": self.simulation_epoch.reset_count,
                    "input_rejected_after_shutdown_count": self.input_rejected_after_shutdown_count,
                    "dropped_on_shutdown_count": self.dropped_on_shutdown_count,
                    "remaining_measurement_queue": len(self.measurement_pair_queue),
                    "remaining_gt_queue": len(self.gt_alignment_ready_queue),
                    "inflight_pair": self.inflight_pair_id,
                }
            )
            self.research_run_summary = summary
            self.research_recording_overrun_invalid = recorder.overrun_invalid
            self.research_recording_error_count = recorder.error_count
            self.research_recording_first_error_type = recorder.first_error_type
            self.research_recording_first_error_message = recorder.first_error_message
            return summary
        except Exception as exc:
            self.research_recording_error_count = getattr(
                self, "research_recording_error_count", 0
            ) + 1
            if not getattr(self, "research_recording_first_error_type", ""):
                self.research_recording_first_error_type = type(exc).__name__
                self.research_recording_first_error_message = str(exc)
            try:
                self.get_logger().error(
                    "Research recorder finalization failed; RUN_INCOMPLETE retained: "
                    f"{type(exc).__name__}: {exc}"
                )
            except Exception:
                pass
            return None

    def shutdown(self):
        if not hasattr(self, "yolo_condition"):
            self.shutdown_event.set()
            self._finalize_research_recording()
            return
        started = time.monotonic()
        drain_timeout_s = 5.0
        with self.yolo_condition:
            self.accepting_input = False
            flushed_pairs = self.image_pairer.flush()
            self._register_measurement_pairs_locked(flushed_pairs)
            self.gt_alignment_ready_queue.extend(self.pending_gt_aligner.flush())
            self.drain_requested = True
            self.shutdown_initial_measurement_count = len(self.measurement_pair_queue)
            self.shutdown_initial_gt_alignment_count = len(self.gt_alignment_ready_queue)
            self.shutdown_initial_inflight_pair_id = self.inflight_pair_id
            self.yolo_condition.notify_all()
            if self.yolo_thread is None:
                self.drain_complete = (
                    not self.measurement_pair_queue and not self.gt_alignment_ready_queue
                )
            deadline = started + drain_timeout_s
            while not self.drain_complete and time.monotonic() < deadline:
                self.yolo_condition.wait(timeout=min(0.1, deadline - time.monotonic()))
            self.shutdown_drain_duration_s = time.monotonic() - started
            self.shutdown_remaining_measurement_count = len(self.measurement_pair_queue)
            self.shutdown_remaining_gt_alignment_count = len(self.gt_alignment_ready_queue)
            self.shutdown_inflight_pair_id = self.inflight_pair_id
            self.shutdown_gt_evaluation_error_count = getattr(
                self, "gt_evaluation_error_count", 0
            )
            if self.drain_complete:
                accounted = (
                    self.measurement_processed_count + self.accepted_pair_discard_count
                )
                self.shutdown_drain_status = (
                    "SUCCESS"
                    if accounted == self.measurement_accepted_count
                    and self.measurement_processing_error_count == 0
                    and self.shutdown_gt_evaluation_error_count == 0
                    else "INCOMPLETE"
                )
            else:
                self.shutdown_drain_status = "INCOMPLETE"
                self.discard_measurement_commits = True
                remaining_ids = {pair.pair_id for pair in self.measurement_pair_queue}
                remaining_ids.update(item[0].pair_id for item in self.gt_alignment_ready_queue)
                if self.inflight_pair_id is not None:
                    remaining_ids.add(self.inflight_pair_id)
                self.dropped_on_shutdown_count = len(remaining_ids)
                if remaining_ids:
                    self.dropped_on_shutdown_pair_id_min = min(remaining_ids)
                    self.dropped_on_shutdown_pair_id_max = max(remaining_ids)
                newly_discarded_ids = tuple(
                    pair_id
                    for pair_id in remaining_ids
                    if pair_id in self.pairs_by_pair_id
                    and pair_id not in self.measurement_results_by_pair_id
                )
                self.accepted_pair_discard_count += len(newly_discarded_ids)
                for pair_id in tuple(remaining_ids):
                    pair = self.pairs_by_pair_id.get(pair_id)
                    metrics = self.measurement_results_by_pair_id.get(pair_id)
                    self._submit_research_terminal_locked(
                        pair,
                        terminal_status="shutdown_discarded",
                        terminal_reason="shutdown_timeout",
                        measurement_status=(
                            "processed" if metrics is not None else "discarded"
                        ),
                        evaluation_status="not_evaluated",
                        metrics=metrics,
                        alignment=self.gt_alignments_by_pair_id.get(pair_id),
                    )
                    self._drop_pair_state_locked(pair_id, "shutdown_timeout")
        if self.shutdown_drain_status == "INCOMPLETE":
            self.get_logger().warning(
                "Shutdown drain INCOMPLETE: "
                f"duration_s={self.shutdown_drain_duration_s:.3f}, "
                f"remaining_measurements={self.shutdown_remaining_measurement_count}, "
                f"remaining_gt={self.shutdown_remaining_gt_alignment_count}, "
                f"inflight_pair_id={self.shutdown_inflight_pair_id}, "
                f"dropped={self.dropped_on_shutdown_count}, "
                f"measurement_errors={self.measurement_processing_error_count}, "
                f"evaluation_errors={self.shutdown_gt_evaluation_error_count}"
            )
        else:
            self.get_logger().info(
                "Shutdown drain SUCCESS: "
                f"duration_s={self.shutdown_drain_duration_s:.3f}, "
                f"initial_measurements={self.shutdown_initial_measurement_count}, "
                f"initial_gt={self.shutdown_initial_gt_alignment_count}, "
                f"initial_inflight={self.shutdown_initial_inflight_pair_id}, "
                f"accepted={self.measurement_accepted_count}, "
                f"processed={self.measurement_processed_count}, "
                f"evaluation_successes={getattr(self, 'gt_evaluation_success_count', 0)}, "
                f"evaluation_errors={self.shutdown_gt_evaluation_error_count}"
            )
        self.shutdown_event.set()
        with self.yolo_condition:
            self.yolo_condition.notify_all()
        if self.yolo_thread is not None and self.yolo_thread.is_alive():
            self.yolo_thread.join(timeout=2.0)
        self._finalize_research_recording()

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
        try:
            stamp_ns = image_stamp_ns(msg)
        except ValueError as exc:
            self.image_pairer.invalid_timestamp_count += 1
            self.get_logger().warning(f"Invalid {side} image simulation timestamp: {exc}")
            return
        with self.yolo_condition:
            if not self.accepting_input:
                self.input_rejected_after_shutdown_count += 1
                return
            latest_clock = self.simulation_epoch.last_clock_stamp_ns
            if not self.simulation_epoch.accepts_stamp(stamp_ns):
                self.stale_pre_reset_count += 1
                self.get_logger().warning(
                    f"{side} image stamp {stamp_ns} is ahead of current /clock {latest_clock}; ignored"
                )
                return
            try:
                pairs = self.image_pairer.add(
                    side,
                    stamp_ns,
                    msg,
                    epoch=self.simulation_epoch.epoch,
                )
            except ValueError as exc:
                self.get_logger().warning(f"Invalid {side} image simulation timestamp: {exc}")
                return
            self._register_measurement_pairs_locked(pairs)

    def _yolo_worker_loop(self):
        if getattr(self, "observation_mode", "bbox_yolo") == "bbox_yolo":
            self._warm_yolo_model()
        next_tick = time.monotonic()
        while True:
            pair = None
            alignment_work = None
            with self.yolo_condition:
                while not self.measurement_pair_queue and not self.gt_alignment_ready_queue:
                    if self.drain_requested:
                        self.drain_complete = True
                        self.yolo_condition.notify_all()
                        return
                    if self.shutdown_event.is_set():
                        return
                    self.yolo_condition.wait(timeout=0.1)
                if self.gt_alignment_ready_queue:
                    alignment_work = self.gt_alignment_ready_queue.popleft()
                elif self.measurement_pair_queue:
                    target_hz = self._measurement_processing_hz()
                    wait_s = next_tick - time.monotonic()
                    if wait_s > 0.0 and not self.drain_requested:
                        self.yolo_condition.wait(timeout=wait_s)
                        continue
                    pair = self.measurement_pair_queue.popleft()
                if alignment_work is not None:
                    pair, alignment = alignment_work
                    try:
                        self._store_gt_alignment_locked(pair, alignment)
                    except Exception as exc:
                        self._finalize_gt_evaluation_failure_locked(
                            pair.pair_id,
                            exc,
                            trigger="late_gt_alignment",
                        )
                    continue
                if pair is None:
                    continue
                phase = self._mission_state()["phase"]
                self.yolo_inference_in_progress = True
                self.inflight_pair_id = pair.pair_id
                self.inflight_pair_epoch = pair.epoch
                self.last_yolo_update = time.monotonic()
                next_tick = self.last_yolo_update + 1.0 / target_hz
            failure = None
            try:
                computed = self._compute_pair_measurement(pair, phase)
                with self.yolo_condition:
                    self._commit_pair_measurement_if_current(pair, computed)
            except Exception as exc:
                failure = exc
                with self.yolo_condition:
                    self._finalize_pair_failure_locked(pair, exc)
            if failure is not None:
                try:
                    self.get_logger().error(
                        "Measurement processing failed for "
                        f"pair_id={pair.pair_id}: "
                        f"{type(failure).__name__}: {failure}"
                    )
                except Exception:
                    pass

    def _finalize_pair_failure_locked(self, pair, exc):
        """Contain one pair failure and leave the worker ready for the next pair."""

        self.measurement_processing_error_count += 1
        self.accepted_pair_discard_count += 1
        self.last_measurement_error_type = type(exc).__name__
        self.last_measurement_error_message = str(exc)
        self._submit_research_terminal_locked(
            pair,
            terminal_status="measurement_processing_error",
            terminal_reason=f"{type(exc).__name__}: {exc}",
            measurement_status="discarded",
            evaluation_status="not_evaluated",
            metrics=self.measurement_results_by_pair_id.get(pair.pair_id),
            alignment=self.gt_alignments_by_pair_id.get(pair.pair_id),
        )
        self._drop_pair_state_locked(pair.pair_id, "measurement_processing_error")
        if self.inflight_pair_id == pair.pair_id:
            self.inflight_pair_id = None
            self.inflight_pair_epoch = None
        self.yolo_inference_in_progress = False
        self.yolo_condition.notify_all()

    def _compute_pair_measurement(self, pair, phase, device=None):
        """Compute one pair without publishing or mutating pair result state."""

        started = time.monotonic()
        camera_oracle_snapshot = self._camera_oracle_snapshot()
        if getattr(self, "observation_mode", "bbox_yolo") == "ideal_reference_point":
            width = int(getattr(pair.left_message, "width", 0))
            height = int(getattr(pair.left_message, "height", 0))
            with self.yolo_condition:
                alignment = self.gt_alignments_by_pair_id.get(pair.pair_id)
            if width <= 0 or height <= 0:
                return {
                    "started": started, "finished": time.monotonic(), "phase": phase,
                    "left_image": None, "right_image": None, "left": None, "right": None,
                    "raw": None, "corrected": None, "ideal_error": "invalid_image_dimensions",
                    "camera_oracle_snapshot": camera_oracle_snapshot,
                }
            point = None if alignment is None else alignment.position_world_enu_m
            left, right = (None, None) if point is None else self._ideal_detection_pair_from_reference_world(
                point, width, height, camera_oracle_snapshot=camera_oracle_snapshot
            )
            if left is None or right is None:
                return {
                    "started": started, "finished": time.monotonic(), "phase": phase,
                    "left_image": np.zeros((height, width, 3), dtype=np.uint8),
                    "right_image": np.zeros((height, width, 3), dtype=np.uint8),
                    "left": None, "right": None, "raw": None, "corrected": None,
                    "ideal_error": "projection_invalid_or_out_of_fov",
                    "camera_oracle_snapshot": camera_oracle_snapshot,
                }
            raw = self._triangulate_center_pixels(left["cx"], left["cy"], right["cx"], right["cy"], width, height)
            corrected = self._triangulate_pixel_centers_oracle(
                left["cx"], left["cy"], right["cx"], right["cy"], width, height,
                camera_oracle_snapshot=camera_oracle_snapshot,
            )
            gps_baselines = self._triangulate_gps_baselines(
                left["cx"], left["cy"], right["cx"], right["cy"], width, height
            )
            return {
                "started": started, "finished": time.monotonic(), "phase": phase,
                "left_image": np.zeros((height, width, 3), dtype=np.uint8),
                "right_image": np.zeros((height, width, 3), dtype=np.uint8),
                "left": left, "right": right, "raw": raw, "corrected": corrected,
                "gps_baselines": gps_baselines,
                "ideal_error": None, "camera_oracle_snapshot": camera_oracle_snapshot,
            }
        left_image = self._msg_to_rgb(pair.left_message)
        right_image = self._msg_to_rgb(pair.right_message)
        if left_image is None or right_image is None:
            return {
                "started": started,
                "finished": time.monotonic(),
                "phase": phase,
                "left_image": left_image,
                "right_image": right_image,
                "left": None,
                "right": None,
                "raw": None,
                "corrected": None,
                "camera_oracle_snapshot": camera_oracle_snapshot,
            }
        if device is None:
            device = self.resolved_yolo_device or self._resolve_yolo_device()
        left, right = self._detect_drone_pair(left_image, right_image, device=device)
        raw = None
        corrected = None
        gps_baselines = {}
        if left is not None and right is not None:
            raw = self._triangulate_center_pixels(
                left["cx"], left["cy"], right["cx"], right["cy"],
                left_image.shape[1], left_image.shape[0],
            )
            corrected = self._triangulate_pixel_centers_oracle(
                left["cx"], left["cy"], right["cx"], right["cy"],
                left_image.shape[1], left_image.shape[0],
                camera_oracle_snapshot=camera_oracle_snapshot,
            )
            gps_baselines = self._triangulate_gps_baselines(
                left["cx"], left["cy"], right["cx"], right["cy"],
                left_image.shape[1], left_image.shape[0],
            )
        return {
            "started": started,
            "finished": time.monotonic(),
            "phase": phase,
            "left_image": left_image,
            "right_image": right_image,
            "left": left,
            "right": right,
            "raw": raw,
            "corrected": corrected,
            "gps_baselines": gps_baselines,
            "camera_oracle_snapshot": camera_oracle_snapshot,
        }

    def _commit_pair_measurement_if_current(self, pair, computed):
        """Linearization point for reset versus measurement publication.

        The caller holds ``yolo_condition``.  A clock reset uses the same
        lock, so either the complete current-epoch result commits first or the
        reset commits first and this result is discarded without publication.
        """

        self.inflight_pair_id = None
        self.inflight_pair_epoch = None
        self.yolo_inference_in_progress = False
        if self.discard_measurement_commits:
            self.yolo_condition.notify_all()
            return None
        if pair.epoch != self.simulation_epoch.epoch:
            self.old_epoch_result_discard_count += 1
            pair_still_pending = (
                not hasattr(self, "pairs_by_pair_id")
                or pair.pair_id in self.pairs_by_pair_id
            )
            if pair_still_pending:
                self.accepted_pair_discard_count += 1
                self._submit_research_terminal_locked(
                    pair,
                    terminal_status="old_epoch_result_discarded",
                    terminal_reason="completed_result_from_old_simulation_epoch",
                    measurement_status="discarded",
                    evaluation_status="not_evaluated",
                    metrics=getattr(self, "measurement_results_by_pair_id", {}).get(
                        pair.pair_id
                    ),
                    alignment=getattr(self, "gt_alignments_by_pair_id", {}).get(
                        pair.pair_id
                    ),
                )
            self.get_logger().warning(
                "Discarded completed YOLO result from old simulation epoch: "
                f"pair_id={pair.pair_id}, pair_epoch={pair.epoch}, "
                f"current_epoch={self.simulation_epoch.epoch}"
            )
            if pair_still_pending and hasattr(self, "pairs_by_pair_id"):
                self._drop_pair_state_locked(
                    pair.pair_id, "old_epoch_result_discarded"
                )
            self.yolo_condition.notify_all()
            return None
        left_image = computed.get("left_image")
        right_image = computed.get("right_image")
        if left_image is None or right_image is None:
            self.accepted_pair_discard_count += 1
            if getattr(self, "observation_mode", "bbox_yolo") == "ideal_reference_point":
                self.ideal_projection_rejected_count += 1
                self.last_ideal_projection_error = computed.get("ideal_error", "invalid_image")
                self.last_ideal_projection_pair_id = pair.pair_id
            self._submit_research_terminal_locked(
                pair,
                terminal_status="image_decode_failure",
                terminal_reason=computed.get("ideal_error", "image_decode_failure"),
                measurement_status="discarded",
                evaluation_status="not_evaluated",
                alignment=getattr(self, "gt_alignments_by_pair_id", {}).get(
                    pair.pair_id
                ),
            )
            self._drop_pair_state_locked(pair.pair_id, "image_decode_failure")
            self.yolo_condition.notify_all()
            return None
        left = computed.get("left")
        right = computed.get("right")
        if getattr(self, "observation_mode", "bbox_yolo") == "ideal_reference_point" and (left is None or right is None):
            self.accepted_pair_discard_count += 1
            self.ideal_projection_rejected_count += 1
            self.last_ideal_projection_error = computed.get("ideal_error", "projection_invalid_or_out_of_fov")
            self.last_ideal_projection_pair_id = pair.pair_id
            self._submit_research_terminal_locked(
                pair,
                terminal_status="ideal_projection_rejected",
                terminal_reason=self.last_ideal_projection_error,
                measurement_status="discarded",
                evaluation_status="not_evaluated",
                alignment=getattr(self, "gt_alignments_by_pair_id", {}).get(
                    pair.pair_id
                ),
            )
            self._drop_pair_state_locked(pair.pair_id, "ideal_projection_rejected")
            self.yolo_condition.notify_all()
            return None
        if getattr(self, "observation_mode", "bbox_yolo") == "ideal_reference_point":
            self.ideal_projection_success_count += 1
        metrics = self._calculate_metrics(
            left_image, left, right, None, computed.get("phase")
        )
        raw = computed.get("raw")
        optical = computed.get("corrected")
        if raw is not None:
            metrics.update(
                {
                    "computation_status": raw.get("computation_status"),
                    "estimated_forward_m": raw.get("forward_m"),
                    "estimated_right_m": raw.get("right_m"),
                    "estimated_down_m": raw.get("down_m"),
                    "estimated_range_m": raw.get("range_m"),
                    "ray_gap_m": raw.get("ray_gap_m"),
                    "intersection_angle_deg": raw.get("intersection_angle_deg"),
                }
            )
            self._publish_ray_markers(nominal=raw, optical=optical)
        metrics.update(
            {
                "optical_computation_status": (
                    "invalid_input" if optical is None else optical.get("computation_status")
                ),
                "optical_forward_m": None if optical is None else optical.get("forward_m"),
                "optical_right_m": None if optical is None else optical.get("right_m"),
                "optical_down_m": None if optical is None else optical.get("down_m"),
                "optical_range_m": None if optical is None else optical.get("range_m"),
                "optical_ray_gap_m": None if optical is None else optical.get("ray_gap_m"),
                "optical_intersection_angle_deg": (
                    None if optical is None else optical.get("intersection_angle_deg")
                ),
            }
        )
        gps_baselines = computed.get("gps_baselines", {})
        for source, gps_baseline in gps_baselines.items():
            prefix = f"{source}_baseline"
            metrics.update(
                {
                    f"{prefix}_computation_status": gps_baseline.get("computation_status"),
                    f"{prefix}_forward_m": gps_baseline.get("forward_m"),
                    f"{prefix}_right_m": gps_baseline.get("right_m"),
                    f"{prefix}_down_m": gps_baseline.get("down_m"),
                    f"{prefix}_range_m": gps_baseline.get("range_m"),
                    f"{prefix}_ray_gap_m": gps_baseline.get("ray_gap_m"),
                    f"{prefix}_intersection_angle_deg": gps_baseline.get("intersection_angle_deg"),
                    f"{prefix}_world_enu_m": gps_baseline.get("gps_baseline_world_enu_m"),
                    f"{prefix}_length_m": gps_baseline.get("gps_baseline_length_m"),
                    f"{prefix}_delta_world_enu_m": gps_baseline.get("gps_baseline_delta_world_enu_m"),
                    f"{prefix}_delta_norm_m": gps_baseline.get("gps_baseline_delta_norm_m"),
                    f"{prefix}_left_eph_m": gps_baseline.get("gps_left_eph_m"),
                    f"{prefix}_right_eph_m": gps_baseline.get("gps_right_eph_m"),
                }
            )
        # Preserve the existing single-source topic and metric contract.  In
        # Both mode it carries raw GPS, while the explicit source topics below
        # carry both results for the operator GUI.
        gps_baseline = gps_baselines.get("raw_gps") or gps_baselines.get("global_position")
        if gps_baseline is not None:
            metrics.update(
                {
                    "gps_baseline_computation_status": gps_baseline.get("computation_status"),
                    "gps_baseline_forward_m": gps_baseline.get("forward_m"),
                    "gps_baseline_right_m": gps_baseline.get("right_m"),
                    "gps_baseline_down_m": gps_baseline.get("down_m"),
                }
            )
        metrics.update(
            self._pair_result_metadata(
                pair, None, None, computed.get("corrected"), count_alignment=False
            )
        )
        metrics["camera_oracle_snapshot"] = computed.get(
            "camera_oracle_snapshot", _CAMERA_ORACLE_UNSET
        )
        metrics.update(self._observation_metadata())
        metrics["left_confidence"] = None if left is None else left.get("confidence")
        metrics["right_confidence"] = None if right is None else right.get("confidence")
        self.last_pair_result = dict(metrics)
        self.left_detection = left
        self.right_detection = right
        self.metrics = metrics
        if getattr(self, "observation_mode", "bbox_yolo") == "bbox_yolo":
            self.detections_pub.publish(self._detections_to_msg(left, right))
        disparity = metrics.get("disparity_px")
        distance = metrics.get("distance_m")
        error = metrics.get("distance_error_m")
        if distance is not None:
            self.distance_pub.publish(Float32(data=float(distance)))
        if error is not None:
            self.error_pub.publish(Float32(data=float(error)))
        if disparity is not None:
            self.disparity_pub.publish(Float32(data=float(disparity)))
        self._publish_estimated_position(metrics)
        self._publish_canonical_pair_positions(metrics)
        finished = computed.get("finished", time.monotonic())
        elapsed_ms = (finished - computed.get("started", finished)) * 1000.0
        self.yolo_inference_ms_pub.publish(Float32(data=float(elapsed_ms)))
        if self.last_yolo_completion is not None:
            period_s = finished - self.last_yolo_completion
            if period_s > 0.0:
                self.yolo_update_hz_pub.publish(Float32(data=float(1.0 / period_s)))
        self.last_yolo_completion = finished
        self._complete_pair_measurement_locked(pair.pair_id, metrics)
        self._evaluate_pair_with_fault_containment_locked(
            pair.pair_id,
            trigger="measurement_complete",
        )
        self.yolo_condition.notify_all()
        return metrics

    def _complete_pair_measurement_locked(self, pair_id, metrics):
        """Record measurement success independently of Ground Truth evaluation."""

        metrics["measurement_status"] = "processed"
        metrics.setdefault("evaluation_status", "pending")
        self.measurement_results_by_pair_id[pair_id] = metrics
        self.measurement_processed_count += 1

    def _observation_metadata(self):
        mode = self._validate_observation_mode(
            getattr(self, "observation_mode", "bbox_yolo")
        )
        ideal_mode = mode == "ideal_reference_point"
        return {
            "observation_mode": mode,
            "observation_source": (
                "gazebo_reference_point_projection"
                if ideal_mode
                else "yolo_bbox_center"
            ),
            "measurement_gt_independent": not ideal_mode,
            "reference_source": VEHICLE_REFERENCE_LINK if ideal_mode else "not_applicable",
            "projection_camera_pose_source": (
                "gazebo_observed_optical" if ideal_mode else "rendered_image_observation"
            ),
            "raw_interpretation_pose_source": "nominal",
            "corrected_interpretation_pose_source": "gazebo_observed_optical",
            "ideal_reference_source": "ground_truth_world_enu_projected_nominal_pixels",
            "ideal_reference_runtime": "simulation_reference_only",
            "correction_source": "gazebo_observed_optical_pose",
            "evaluation_gt_source": "gazebo_reference_link_world_pose",
            "output_frame": BASELINE_CENTER_FRD_FRAME,
        }

    def _update_metrics_from_messages(
        self,
        left_msg,
        right_msg,
        actual,
        phase,
        device=None,
        *,
        pair=None,
        gt_alignment=None,
        gt_baseline=None,
    ):
        if pair is None:
            left_image = self._msg_to_rgb(left_msg)
            right_image = self._msg_to_rgb(right_msg)
            if left_image is None or right_image is None:
                return None
            if device is None:
                device = self.resolved_yolo_device or self._resolve_yolo_device()
            left, right = self._detect_drone_pair(left_image, right_image, device=device)
            metrics = self._calculate_metrics(left_image, left, right, actual, phase)
            corrected = None
            if left is not None and right is not None:
                corrected = self._triangulate_pixel_centers_oracle(
                    left["cx"], left["cy"], right["cx"], right["cy"],
                    left_image.shape[1], left_image.shape[0],
                )
            metrics.update(
                self._pair_result_metadata(
                    pair, gt_alignment, gt_baseline, corrected,
                    count_alignment=gt_alignment is not None,
                )
            )
            return metrics
        computed = self._compute_pair_measurement(pair, phase, device=device)
        with self.yolo_condition:
            return self._commit_pair_measurement_if_current(pair, computed)

    def _ground_truth_alignment_to_baseline(self, alignment):
        if alignment is None or alignment.position_world_enu_m is None:
            return None
        try:
            return ground_truth_world_to_baseline(
                alignment.position_world_enu_m,
                self.baseline_center_frd,
            )
        except (TypeError, ValueError):
            return None

    def _store_gt_alignment_locked(self, pair, alignment):
        self._ensure_gt_evaluation_state_locked()
        if pair.epoch != self.simulation_epoch.epoch:
            self.old_epoch_frame_count += 1
            return
        # A legitimate alignment is created only for a pair registered in the
        # active pair map.  Do not resurrect terminal state after its bounded
        # evaluated/failed tombstone has eventually been evicted.
        if pair.pair_id not in self.pairs_by_pair_id:
            return
        if (
            pair.pair_id in self.evaluated_pair_ids
            or pair.pair_id in self.gt_evaluation_failed_pair_ids
        ):
            return
        self.gt_alignments_by_pair_id[pair.pair_id] = alignment
        if getattr(self, "observation_mode", "bbox_yolo") == "ideal_reference_point":
            if pair.pair_id not in {item.pair_id for item in self.measurement_pair_queue}:
                if len(self.measurement_pair_queue) >= self.measurement_queue_capacity:
                    self._finalize_measurement_queue_overflow_locked(pair.pair_id)
                    return
                self.measurement_pair_queue.append(pair)
                self._record_measurement_queue_depth_locked()
        self._evaluate_pair_with_fault_containment_locked(
            pair.pair_id,
            trigger="late_gt_alignment",
        )

    def _evaluate_pair_with_fault_containment_locked(self, pair_id, *, trigger):
        """Attempt one GT evaluation without invalidating measurement success."""

        self._ensure_gt_evaluation_state_locked()
        try:
            return self._try_evaluate_pair_locked(pair_id)
        except Exception as exc:
            self._finalize_gt_evaluation_failure_locked(
                pair_id,
                exc,
                trigger=trigger,
            )
            return False

    def _finalize_gt_evaluation_failure_locked(self, pair_id, exc, *, trigger):
        self._ensure_gt_evaluation_state_locked()
        pair_id = int(pair_id)
        if (
            pair_id in self.evaluated_pair_ids
            or pair_id in self.gt_evaluation_failed_pair_ids
        ):
            return False
        self.gt_evaluation_failed_pair_ids.add(pair_id)
        self.gt_evaluation_error_count += 1
        self.last_gt_evaluation_error_type = type(exc).__name__
        self.last_gt_evaluation_error_message = str(exc)
        self.last_gt_evaluation_error_trigger = str(trigger)
        pair = self.pairs_by_pair_id.get(pair_id)
        metrics = self.measurement_results_by_pair_id.get(pair_id)
        if metrics is not None:
            metrics["evaluation_status"] = "evaluation_error"
            self.last_pair_result = dict(metrics)
            measurement_status = "processed"
        else:
            self.accepted_pair_discard_count += 1
            measurement_status = "discarded"
        self._submit_research_terminal_locked(
            pair,
            terminal_status="evaluation_error",
            terminal_reason=f"{trigger}:{type(exc).__name__}: {exc}",
            measurement_status=measurement_status,
            evaluation_status="evaluation_error",
            metrics=metrics,
            alignment=self.gt_alignments_by_pair_id.get(pair_id),
        )
        self.gt_alignments_by_pair_id.pop(pair_id, None)
        self.measurement_results_by_pair_id.pop(pair_id, None)
        self.pairs_by_pair_id.pop(pair_id, None)
        self.yolo_condition.notify_all()
        try:
            self.get_logger().error(
                "Ground Truth evaluation failed for "
                f"pair_id={pair_id}, trigger={trigger}: "
                f"{type(exc).__name__}: {exc}"
            )
        except Exception:
            pass
        return True

    def _ensure_gt_evaluation_state_locked(self):
        if not hasattr(self, "gt_evaluation_failed_pair_ids"):
            capacity = int(getattr(self, "max_pair_result_history", 128))
            self.gt_evaluation_failed_pair_ids = BoundedIdHistory(capacity)
        if not hasattr(self, "gt_evaluation_error_count"):
            self.gt_evaluation_error_count = 0
        if not hasattr(self, "gt_evaluation_success_count"):
            self.gt_evaluation_success_count = 0
        if not hasattr(self, "last_gt_evaluation_error_type"):
            self.last_gt_evaluation_error_type = ""
            self.last_gt_evaluation_error_message = ""
            self.last_gt_evaluation_error_trigger = ""

    def _try_evaluate_pair_locked(self, pair_id):
        self._ensure_gt_evaluation_state_locked()
        if (
            pair_id in self.evaluated_pair_ids
            or pair_id in self.gt_evaluation_failed_pair_ids
        ):
            return False
        metrics = self.measurement_results_by_pair_id.get(pair_id)
        alignment = self.gt_alignments_by_pair_id.get(pair_id)
        pair = self.pairs_by_pair_id.get(pair_id)
        if metrics is None or alignment is None or pair is None:
            return False
        gt_baseline = self._ground_truth_alignment_to_baseline(alignment)
        ideal = None
        if alignment.position_world_enu_m is not None:
            width = int(getattr(pair.left_message, "width", 0))
            height = int(getattr(pair.left_message, "height", 0))
            if (
                width > 0
                and height > 0
                and getattr(self, "horizontal_fov_rad", None) is not None
            ):
                ideal = self._triangulate_dkl_ideal_from_reference_world(
                    alignment.position_world_enu_m, width, height
                )
        metrics.update(
            {
                "ideal_computation_status": (
                    "reference_unavailable"
                    if ideal is None
                    else ideal.get("computation_status")
                ),
                "ideal_forward_m": None if ideal is None else ideal.get("forward_m"),
                "ideal_right_m": None if ideal is None else ideal.get("right_m"),
                "ideal_down_m": None if ideal is None else ideal.get("down_m"),
                "ideal_range_m": None if ideal is None else ideal.get("range_m"),
                "ideal_ray_gap_m": None if ideal is None else ideal.get("ray_gap_m"),
                "ideal_intersection_angle_deg": (
                    None if ideal is None else ideal.get("intersection_angle_deg")
                ),
                "ideal_pixel_source": "ground_truth_world_enu_quantized_nominal_projection",
            }
        )
        evaluation = self._evaluation_metadata(pair, alignment, gt_baseline, metrics)
        metrics.update(evaluation)
        if ideal is not None:
            self._publish_ray_markers(ideal=ideal)
        # The initial publish happens before GT alignment exists.  Republish
        # after the DKL reference has been evaluated so rqt receives it without
        # waiting for the periodic plot sampler.
        if getattr(self, "estimated_position_pub", None) is not None:
            self._publish_estimated_position(metrics)
        unavailable = alignment.gt_alignment_mode == "unavailable"
        if gt_baseline is not None and pair_id not in self.gt_published_pair_ids:
            self.canonical_ground_truth_position_pub.publish(
                self._point_stamped(
                    pair.pair_midpoint_stamp_ns,
                    gt_baseline["forward_m"],
                    gt_baseline["right_m"],
                    gt_baseline["down_m"],
                )
            )
            self.gt_published_pair_ids.add(pair_id)
        metrics["evaluation_status"] = (
            "unavailable" if unavailable else "evaluated"
        )
        self.last_pair_result = dict(metrics)
        self.gt_alignment_counts[alignment.gt_alignment_mode] += 1
        if not unavailable:
            self.gt_evaluation_success_count += 1
        self.evaluated_pair_ids.add(pair_id)
        self._submit_research_terminal_locked(
            pair,
            terminal_status="gt_unavailable" if unavailable else "evaluated",
            terminal_reason="gt_unavailable" if unavailable else "evaluated",
            measurement_status="processed",
            evaluation_status="unavailable" if unavailable else "evaluated",
            metrics=metrics,
            alignment=alignment,
        )
        self.measurement_results_by_pair_id.pop(pair_id, None)
        self.gt_alignments_by_pair_id.pop(pair_id, None)
        self.pairs_by_pair_id.pop(pair_id, None)
        return True

    @staticmethod
    def _vector_error(metrics, prefix, gt_baseline):
        if gt_baseline is None:
            return None, None
        values = (
            metrics.get(f"{prefix}forward_m"),
            metrics.get(f"{prefix}right_m"),
            metrics.get(f"{prefix}down_m"),
        )
        if any(value is None for value in values):
            return None, None
        errors = tuple(
            float(values[index]) - float(gt_baseline[name])
            for index, name in enumerate(("forward_m", "right_m", "down_m"))
        )
        return math.sqrt(sum(value * value for value in errors)), errors[0]

    def _evaluation_metadata(self, pair, alignment, gt_baseline, metrics):
        def errors(prefix):
            if gt_baseline is None:
                return (None, None, None, None, None)
            estimate_xyz = tuple(
                metrics.get(f"{prefix}{axis}_m")
                for axis in ("forward", "right", "down")
            )
            if not self._all_finite(estimate_xyz):
                return (None, None, None, None, None)
            forward_error = float(estimate_xyz[0]) - float(gt_baseline["forward_m"])
            right_error = float(estimate_xyz[1]) - float(gt_baseline["right_m"])
            down_error = float(estimate_xyz[2]) - float(gt_baseline["down_m"])
            estimated_range = metrics.get(f"{prefix}range_m")
            range_error = (
                float(estimated_range) - float(gt_baseline["range_m"])
                if self._all_finite((estimated_range, gt_baseline["range_m"]))
                else None
            )
            position_error = math.sqrt(
                forward_error * forward_error
                + right_error * right_error
                + down_error * down_error
            )
            return (
                position_error,
                forward_error,
                right_error,
                down_error,
                range_error,
            )

        raw_errors = errors("estimated_")
        corrected_errors = errors("attitude_corrected_")
        ideal_errors = errors("ideal_")
        return {
            "pair_id": pair.pair_id,
            "gt_alignment_mode": alignment.gt_alignment_mode,
            "gt_before_stamp_ns": alignment.gt_before_stamp_ns,
            "gt_after_stamp_ns": alignment.gt_after_stamp_ns,
            "gt_interpolation_alpha": alignment.gt_interpolation_alpha,
            "gt_nearest_delta_ns": alignment.gt_nearest_delta_ns,
            "gt_forward_m": None if gt_baseline is None else gt_baseline["forward_m"],
            "gt_right_m": None if gt_baseline is None else gt_baseline["right_m"],
            "gt_down_m": None if gt_baseline is None else gt_baseline["down_m"],
            "gt_range_m": None if gt_baseline is None else gt_baseline["range_m"],
            "raw_position_error_m": raw_errors[0],
            "raw_forward_error_m": raw_errors[1],
            "raw_right_error_m": raw_errors[2],
            "raw_down_error_m": raw_errors[3],
            "raw_range_error_m": raw_errors[4],
            "corrected_position_error_m": corrected_errors[0],
            "corrected_forward_error_m": corrected_errors[1],
            "corrected_right_error_m": corrected_errors[2],
            "corrected_down_error_m": corrected_errors[3],
            "corrected_range_error_m": corrected_errors[4],
            "ideal_position_error_m": ideal_errors[0],
            "ideal_forward_error_m": ideal_errors[1],
            "ideal_right_error_m": ideal_errors[2],
            "ideal_down_error_m": ideal_errors[3],
            "ideal_range_error_m": ideal_errors[4],
        }

    def _pair_result_metadata(
        self,
        pair,
        alignment,
        gt_baseline,
        corrected,
        *,
        count_alignment=True,
    ):
        self._ensure_attitude_error_state()
        alignment_mode = "unavailable" if alignment is None else alignment.gt_alignment_mode
        if count_alignment and hasattr(self, "gt_alignment_counts"):
            self.gt_alignment_counts[alignment_mode] += 1
        metadata = {
            "pair_id": None if pair is None else pair.pair_id,
            "epoch": None if pair is None else pair.epoch,
            "left_stamp_ns": None if pair is None else pair.left_stamp_ns,
            "right_stamp_ns": None if pair is None else pair.right_stamp_ns,
            "pair_midpoint_stamp_ns": None if pair is None else pair.pair_midpoint_stamp_ns,
            "pair_delta_ns": None if pair is None else pair.pair_delta_ns,
            "pairing_mode": None if pair is None else pair.pairing_mode,
            "gt_alignment_mode": alignment_mode,
            "gt_before_stamp_ns": None if alignment is None else alignment.gt_before_stamp_ns,
            "gt_after_stamp_ns": None if alignment is None else alignment.gt_after_stamp_ns,
            "gt_interpolation_alpha": None if alignment is None else alignment.gt_interpolation_alpha,
            "gt_nearest_delta_ns": None if alignment is None else alignment.gt_nearest_delta_ns,
            "gt_forward_m": None if gt_baseline is None else gt_baseline["forward_m"],
            "gt_right_m": None if gt_baseline is None else gt_baseline["right_m"],
            "gt_down_m": None if gt_baseline is None else gt_baseline["down_m"],
            "gt_range_m": None if gt_baseline is None else gt_baseline["range_m"],
            "cam_l_optical_roll_deg": self.cam_l_optical_roll_deg,
            "cam_l_optical_pitch_deg": self.cam_l_optical_pitch_deg,
            "cam_l_optical_yaw_deg": self.cam_l_optical_yaw_deg,
            "cam_r_optical_roll_deg": self.cam_r_optical_roll_deg,
            "cam_r_optical_pitch_deg": self.cam_r_optical_pitch_deg,
            "cam_r_optical_yaw_deg": self.cam_r_optical_yaw_deg,
            "cam_l_px4_roll_deg": self.cam_l_px4_roll_deg,
            "cam_l_px4_pitch_deg": self.cam_l_px4_pitch_deg,
            "cam_l_px4_yaw_deg": self.cam_l_px4_yaw_deg,
            "cam_r_px4_roll_deg": self.cam_r_px4_roll_deg,
            "cam_r_px4_pitch_deg": self.cam_r_px4_pitch_deg,
            "cam_r_px4_yaw_deg": self.cam_r_px4_yaw_deg,
            "common_optical_roll_error_deg": self.common_optical_roll_error_deg,
            "relative_optical_roll_error_deg": self.relative_optical_roll_error_deg,
            "common_optical_pitch_error_deg": self.common_optical_pitch_error_deg,
            "relative_optical_pitch_error_deg": self.relative_optical_pitch_error_deg,
            "common_optical_yaw_error_deg": self.common_optical_yaw_error_deg,
            "relative_optical_yaw_error_deg": self.relative_optical_yaw_error_deg,
            "effective_left_optical_roll_error_deg": self.effective_left_optical_roll_error_deg,
            "effective_left_optical_pitch_error_deg": self.effective_left_optical_pitch_error_deg,
            "effective_left_optical_yaw_error_deg": self.effective_left_optical_yaw_error_deg,
            "effective_right_optical_roll_error_deg": self.effective_right_optical_roll_error_deg,
            "effective_right_optical_pitch_error_deg": self.effective_right_optical_pitch_error_deg,
            "effective_right_optical_yaw_error_deg": self.effective_right_optical_yaw_error_deg,
            "attitude_error_frame": self.attitude_error_frame,
            "attitude_angle_unit": self.attitude_angle_unit,
            "attitude_parameterization": self.attitude_parameterization,
            "attitude_rotation_composition": self.attitude_rotation_composition,
            "correction_source": "gazebo_observed_optical_pose",
            "attitude_corrected_computation_status": (
                "invalid_input" if corrected is None else corrected.get("computation_status")
            ),
            "attitude_corrected_forward_m": None if corrected is None else corrected.get("forward_m"),
            "attitude_corrected_right_m": None if corrected is None else corrected.get("right_m"),
            "attitude_corrected_down_m": None if corrected is None else corrected.get("down_m"),
            "attitude_corrected_range_m": None if corrected is None else corrected.get("range_m"),
            "attitude_corrected_ray_gap_m": None if corrected is None else corrected.get("ray_gap_m"),
            "attitude_corrected_intersection_angle_deg": (
                None if corrected is None else corrected.get("intersection_angle_deg")
            ),
            "attitude_corrected_left_depth_m": None if corrected is None else corrected.get("left_depth_m"),
            "attitude_corrected_right_depth_m": None if corrected is None else corrected.get("right_depth_m"),
            "attitude_corrected_geometry_rank": None if corrected is None else corrected.get("geometry_rank"),
            "attitude_corrected_condition_number": None if corrected is None else corrected.get("condition_number"),
            "output_frame": BASELINE_CENTER_FRD_FRAME,
        }
        if pair is not None and pair.pair_id % 100 == 0:
            self.get_logger().info(f"Simulation-time alignment counters: {self._time_alignment_counters()}")
        return metadata

    def _time_alignment_counters(self):
        pairer = self.image_pairer
        return {
            "left_received_count": pairer.left_received_count,
            "right_received_count": pairer.right_received_count,
            "paired_count": pairer.paired_count,
            "exact_pair_count": pairer.exact_pair_count,
            "nearest_pair_count": pairer.nearest_pair_count,
            "unmatched_left_count": pairer.unmatched_left_count,
            "unmatched_right_count": pairer.unmatched_right_count,
            "image_reset_count": pairer.image_reset_count,
            "gt_reset_count": self.gt_position_buffer.reset_count,
            "simulation_reset_count": self.simulation_epoch.reset_count,
            "simulation_epoch": self.simulation_epoch.epoch,
            "stale_pre_reset_count": self.stale_pre_reset_count,
            "stale_pre_reset_gt_count": self.stale_pre_reset_gt_count,
            "gt_quarantine_active": self.gt_quarantine_active,
            "gt_quarantine_rejected_count": self.gt_quarantine_rejected_count,
            "gt_quarantine_release_count": self.gt_quarantine_release_count,
            "future_gt_pending_count": len(self.future_gt_pending),
            "future_gt_overflow_count": self.future_gt_overflow_count,
            "measurement_queue_capacity": self.measurement_queue_capacity,
            "measurement_queue_depth": len(self.measurement_pair_queue),
            "measurement_queue_depth_max": self.measurement_queue_depth_max,
            "measurement_queue_overflow_count": self.measurement_queue_overflow_count,
            "measurement_overrun_invalid": self.measurement_overrun_invalid,
            "measurement_rejected_after_overrun_count": (
                self.measurement_rejected_after_overrun_count
            ),
            "old_epoch_result_discard_count": self.old_epoch_result_discard_count,
            "accepted_pair_discard_count": self.accepted_pair_discard_count,
            "measurement_processing_error_count": self.measurement_processing_error_count,
            "gt_evaluation_success_count": self.gt_evaluation_success_count,
            "gt_evaluation_error_count": self.gt_evaluation_error_count,
            "gt_evaluation_failed_pair_count": len(
                self.gt_evaluation_failed_pair_ids
            ),
            "last_gt_evaluation_error_type": self.last_gt_evaluation_error_type,
            "last_gt_evaluation_error_message": self.last_gt_evaluation_error_message,
            "last_gt_evaluation_error_trigger": self.last_gt_evaluation_error_trigger,
            "input_rejected_after_shutdown_count": self.input_rejected_after_shutdown_count,
            "last_measurement_error_type": self.last_measurement_error_type,
            "last_measurement_error_message": self.last_measurement_error_message,
            "invalid_timestamp_count": pairer.invalid_timestamp_count,
            "inconsistent_clock_count": self.inconsistent_clock_count
            + self.gt_position_buffer.inconsistent_clock_count,
            "gt_exact_count": self.gt_alignment_counts["exact"],
            "gt_interpolated_count": self.gt_alignment_counts["interpolated"],
            "gt_nearest_before_count": self.gt_alignment_counts["nearest_before"],
            "gt_nearest_after_count": self.gt_alignment_counts["nearest_after"],
            "gt_unavailable_count": self.gt_alignment_counts["unavailable"],
            "raw_computed_count": self.raw_computed_count,
            "corrected_computed_count": self.corrected_computed_count,
            "observation_mode": self.observation_mode,
            "ideal_projection_success_count": self.ideal_projection_success_count,
            "ideal_projection_rejected_count": self.ideal_projection_rejected_count,
            "last_ideal_projection_error": self.last_ideal_projection_error,
            "last_ideal_projection_pair_id": self.last_ideal_projection_pair_id,
            "research_recording_enabled": self._research_recording_enabled(),
            "research_terminal_record_count": getattr(
                self, "research_terminal_record_count", 0
            ),
            "research_record_queue_depth": (
                0 if not self._research_recording_enabled()
                else self.research_recorder.queue_depth
            ),
            "research_record_queue_depth_max": (
                0 if not self._research_recording_enabled()
                else self.research_recorder.queue_depth_max
            ),
            "research_record_queue_capacity": getattr(
                self, "research_record_queue_capacity", 0
            ),
            "research_recording_overrun_invalid": getattr(
                self, "research_recording_overrun_invalid", False
            ),
            "research_recording_error_count": getattr(
                self, "research_recording_error_count", 0
            ),
        }

    @staticmethod
    def _point_stamped(stamp_ns, forward_m, right_m, down_m):
        message = PointStamped()
        message.header.stamp.sec = int(stamp_ns) // 1_000_000_000
        message.header.stamp.nanosec = int(stamp_ns) % 1_000_000_000
        message.header.frame_id = BASELINE_CENTER_FRD_FRAME
        message.point.x = float(forward_m)
        message.point.y = float(right_m)
        message.point.z = float(down_m)
        return message

    def _publish_canonical_pair_positions(self, metrics):
        stamp_ns = metrics.get("pair_midpoint_stamp_ns")
        pair_id = metrics.get("pair_id")
        if stamp_ns is None:
            return
        if (
            metrics.get("computation_status") == "computed"
            and pair_id is not None
            and pair_id > self.last_raw_published_pair_id
        ):
            self.raw_computed_count += 1
            self.raw_position_pub.publish(
                self._point_stamped(
                    stamp_ns,
                    metrics["estimated_forward_m"],
                    metrics["estimated_right_m"],
                    metrics["estimated_down_m"],
                )
            )
            self.last_raw_published_pair_id = pair_id
        if (
            metrics.get("attitude_corrected_computation_status") == "computed"
            and pair_id is not None
            and pair_id > self.last_corrected_published_pair_id
        ):
            self.corrected_computed_count += 1
            self.attitude_corrected_position_pub.publish(
                self._point_stamped(
                    stamp_ns,
                    metrics["attitude_corrected_forward_m"],
                    metrics["attitude_corrected_right_m"],
                    metrics["attitude_corrected_down_m"],
                )
            )
            self.last_corrected_published_pair_id = pair_id
        if (
            metrics.get("gps_baseline_computation_status") == "computed"
            and pair_id is not None
            and pair_id > self.last_gps_baseline_published_pair_id
        ):
            self.gps_baseline_position_pub.publish(
                self._point_stamped(
                    stamp_ns,
                    metrics["gps_baseline_forward_m"],
                    metrics["gps_baseline_right_m"],
                    metrics["gps_baseline_down_m"],
                )
            )
            self.last_gps_baseline_published_pair_id = pair_id
        source_publisher_names = {
            "raw_gps": (
                "raw_gps_baseline_position_pub",
                "raw_gps_baseline_plot_pub",
            ),
            "global_position": (
                "global_position_baseline_position_pub",
                "global_position_baseline_plot_pub",
            ),
        }
        last_published_by_source = getattr(
            self, "last_gps_baseline_published_pair_id_by_source", None
        )
        if last_published_by_source is None:
            last_published_by_source = {}
            self.last_gps_baseline_published_pair_id_by_source = (
                last_published_by_source
            )
        for source, publisher_names in source_publisher_names.items():
            prefix = f"{source}_baseline"
            if (
                metrics.get(f"{prefix}_computation_status") == "computed"
                and pair_id is not None
                and pair_id > last_published_by_source.get(source, 0)
            ):
                position_publisher = getattr(self, publisher_names[0], None)
                plot_publisher = getattr(self, publisher_names[1], None)
                if position_publisher is None or plot_publisher is None:
                    continue
                forward_m = metrics[f"{prefix}_forward_m"]
                right_m = metrics[f"{prefix}_right_m"]
                down_m = metrics[f"{prefix}_down_m"]
                position_publisher.publish(
                    self._point_stamped(
                        stamp_ns,
                        forward_m,
                        right_m,
                        down_m,
                    )
                )
                plot_publisher.publish(
                    Float32MultiArray(
                        data=[float(forward_m), float(right_m), float(down_m)]
                    )
                )
                last_published_by_source[source] = pair_id

    def _publish_estimated_position(self, metrics):
        estimated_publisher = getattr(self, "estimated_position_pub", None)
        if estimated_publisher is not None:
            estimated_publisher.publish(
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
        corrected_publisher = getattr(self, "corrected_position_pub", None)
        if corrected_publisher is not None:
            corrected_publisher.publish(
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
        optical_publisher = getattr(self, "optical_position_pub", None)
        if optical_publisher is not None:
            optical_publisher.publish(
                Float32MultiArray(
                    data=[
                        math.nan if metrics.get("optical_forward_m") is None else float(metrics["optical_forward_m"]),
                        math.nan if metrics.get("optical_right_m") is None else float(metrics["optical_right_m"]),
                        math.nan if metrics.get("optical_down_m") is None else float(metrics["optical_down_m"]),
                        math.nan if metrics.get("optical_range_m") is None else float(metrics["optical_range_m"]),
                    ]
                )
            )
        ideal_publisher = getattr(self, "ideal_position_pub", None)
        if ideal_publisher is not None:
            ideal_publisher.publish(
                Float32MultiArray(
                    data=[
                        math.nan if metrics.get("ideal_forward_m") is None else float(metrics["ideal_forward_m"]),
                        math.nan if metrics.get("ideal_right_m") is None else float(metrics["ideal_right_m"]),
                        math.nan if metrics.get("ideal_down_m") is None else float(metrics["ideal_down_m"]),
                        math.nan if metrics.get("ideal_range_m") is None else float(metrics["ideal_range_m"]),
                    ]
                )
            )
        bbox_adjustment = metrics.get("bbox_adjustment")
        bbox_publisher = getattr(self, "bbox_adjustment_pub", None)
        if bbox_adjustment is not None and bbox_publisher is not None:
            bbox_publisher.publish(self._bbox_adjustment_to_msg(bbox_adjustment))
        profile_publisher = getattr(self, "profile_status_pub", None)
        if profile_publisher is not None:
            profile_publisher.publish(self._profile_status_to_msg(metrics))

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
        status.update(
            {
                "pair_id": metrics.get("pair_id"),
                "epoch": metrics.get("epoch"),
                "left_stamp_ns": metrics.get("left_stamp_ns"),
                "right_stamp_ns": metrics.get("right_stamp_ns"),
                "pair_midpoint_stamp_ns": metrics.get("pair_midpoint_stamp_ns"),
                "measurement_queue_depth": len(getattr(self, "measurement_pair_queue", ())),
                "measurement_queue_capacity": getattr(self, "measurement_queue_capacity", None),
                "measurement_overrun_invalid": getattr(self, "measurement_overrun_invalid", False),
            }
        )
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
        actual = self._read_actual_distance()
        width = int(getattr(self, "external_bbox_detection_width_px", 1280))
        height = int(getattr(self, "external_bbox_detection_height_px", 720))
        synthetic_image = np.zeros((height, width, 3), dtype=np.uint8)
        metrics = self._calculate_metrics(
            synthetic_image,
            left,
            right,
            math.nan if actual is None else float(actual),
            state["phase"],
        )
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
                predict_kwargs["quantize"] = 16
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
            predict_kwargs["quantize"] = 16
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
            predict_kwargs["quantize"] = 16
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

    def _triangulate_center_pixels(self, left_cx, left_cy, right_cx, right_cy, width, height):
        return self._triangulate_pixel_centers(left_cx, left_cy, right_cx, right_cy, width, height)

    def _synthetic_detection_pair_from_rdf(self, *, right_m, down_m, forward_m, width, height):
        self._ensure_camera_frames()
        point_baseline = (float(forward_m), float(right_m), float(down_m))
        if not all(math.isfinite(value) for value in point_baseline):
            return None, None
        point_world = self.baseline_center_frd.point_to_world(point_baseline)
        return self._detection_pair_from_world(
            point_world,
            width,
            height,
            projection_camera_mode="gazebo_observed_optical",
            enforce_fov_bounds=False,
        )

    def _ideal_detection_pair_from_reference_world(
        self, point_world_enu_m, width, height, *, camera_oracle_snapshot=_CAMERA_ORACLE_UNSET
    ):
        self._ensure_camera_frames()
        return self._detection_pair_from_world(
            point_world_enu_m,
            width,
            height,
            projection_camera_mode="gazebo_observed_optical",
            enforce_fov_bounds=True,
            camera_oracle_snapshot=camera_oracle_snapshot,
        )

    def _detection_pair_from_world(
        self, point_world_enu_m, width, height, *, projection_camera_mode="gazebo_observed_optical", enforce_fov_bounds=False,
        camera_oracle_snapshot=_CAMERA_ORACLE_UNSET,
    ):
        self._ensure_camera_frames()
        if projection_camera_mode == "gazebo_observed_optical":
            snapshot = camera_oracle_snapshot
            if snapshot is _CAMERA_ORACLE_UNSET:
                snapshot = self._camera_oracle_snapshot()
            if snapshot is None:
                return None, None
            camera_left, camera_right = (snapshot[0].pose, snapshot[1].pose)
        elif projection_camera_mode == "nominal":
            camera_left = self.nominal_left_camera
            camera_right = self.nominal_right_camera
        else:
            raise ValueError(f"unsupported projection_camera_mode={projection_camera_mode!r}")
        intrinsics = self._camera_intrinsics(width, height)
        try:
            left_pixel = project_world_point_to_pixel(
                point_world_enu_m, camera_left, fx_px=intrinsics.fx_px,
                fy_px=intrinsics.fy_px, cx_px=intrinsics.cx_px, cy_px=intrinsics.cy_px,
            )
            right_pixel = project_world_point_to_pixel(
                point_world_enu_m, camera_right, fx_px=intrinsics.fx_px,
                fy_px=intrinsics.fy_px, cx_px=intrinsics.cx_px, cy_px=intrinsics.cy_px,
            )
        except (TypeError, ValueError, ArithmeticError):
            return None, None
        if left_pixel is None or right_pixel is None:
            return None, None
        if enforce_fov_bounds and any(
            not (0.0 <= float(pixel[0]) < float(width) and 0.0 <= float(pixel[1]) < float(height))
            for pixel in (left_pixel, right_pixel)
        ):
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

        return detection(left_pixel[0], left_pixel[1]), detection(right_pixel[0], right_pixel[1])

    def _publish_synthetic_detection_sample(self):
        if not self.enable_synthetic_detections:
            return
        state = self._mission_state()
        actual = self._read_actual_distance()
        position = self.cached_ground_truth_position
        if position is None or position.get("ground_truth_world_enu_m") is None:
            return
        width = int(self.synthetic_detection_width_px)
        height = int(self.synthetic_detection_height_px)
        left, right = self._ideal_detection_pair_from_reference_world(
            position["ground_truth_world_enu_m"],
            width=width,
            height=height,
        )
        if left is None or right is None:
            return
        synthetic_image = np.zeros((height, width, 3), dtype=np.uint8)
        metrics = self._calculate_metrics(
            synthetic_image,
            left,
            right,
            math.nan if actual is None else float(actual),
            state["phase"],
        )
        metrics["status"] = "Synthetic bbox tracking for headless/operator pipeline validation"
        with self.lock:
            self.left_detection = left
            self.right_detection = right
            self.metrics = metrics
        self.detections_pub.publish(self._detections_to_msg(left, right))
        self._publish_estimated_position(metrics)

    def _raw_from_runtime_sample(self, sample):
        if sample.left_u_px is None or sample.left_v_px is None:
            return {
                "right_m": None,
                "down_m": None,
                "forward_m": None,
                "range_m": None,
                "distance_m": None,
                "disparity_px": float(sample.measured_disparity_px),
                "computation_status": "invalid_input:missing_pixels",
            }
        left_u = float(sample.left_u_px)
        left_v = float(sample.left_v_px)
        right_u = left_u - float(sample.measured_disparity_px)
        right_v = left_v
        return self._triangulate_center_pixels(left_u, left_v, right_u, right_v, 1280, 720)

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
        corrected = self._triangulate_center_pixels(
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
            return self._triangulate_center_pixels(left_cx, left_cy, right_cx, right_cy, width, height)
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
                "legacy_fused_raw": None,
                "bbox_adjustment": None,
                "profile_status": self._profile_status(
                    getattr(self.bbox_center_adjustment, "profile_id", ""),
                    getattr(self.bbox_center_adjustment, "mode", ""),
                    3,
                    "incomplete_bbox_pair",
                ),
                "observation_fusion_status": "incomplete_bbox_pair",
                "actual_distance_m": actual,
                "distance_error_m": None,
                "mission_phase": phase,
                "status": self.yolo_load_error or "YOLO target not detected in both images",
            }

        width = left_image.shape[1]
        height = left_image.shape[0]
        raw = self._triangulate_center_pixels(left["cx"], left["cy"], right["cx"], right["cy"], width, height)
        observation_fusion_status = "disabled"
        fused_raw, observation_fusion_status = self._apply_observation_fusion(left, right, width, height)
        legacy_fused_observation = fused_raw
        raw.setdefault("left_u_px", float(left["cx"]))
        raw.setdefault("left_v_px", float(left["cy"]))
        raw.setdefault("right_u_px", float(right["cx"]))
        raw.setdefault("right_v_px", float(right["cy"]))
        corrected = fused_raw
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
            corrected = self._triangulate_center_pixels(
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
        error = (
            None
            if raw["distance_m"] is None or actual is None or not math.isfinite(float(actual))
            else raw["distance_m"] - actual
        )
        return {
            "left_u_px": raw.get("left_u_px"),
            "left_v_px": raw.get("left_v_px"),
            "right_u_px": raw.get("right_u_px"),
            "right_v_px": raw.get("right_v_px"),
            "distance_m": raw["distance_m"],
            "disparity_px": raw["disparity_px"],
            "estimated_right_m": raw["right_m"],
            "estimated_down_m": raw["down_m"],
            "estimated_forward_m": raw["forward_m"],
            "estimated_range_m": raw["range_m"],
            "raw_forward_m": raw["forward_m"],
            "raw_right_m": raw["right_m"],
            "raw_down_m": raw["down_m"],
            "raw_range_m": raw["range_m"],
            "raw_computation_status": raw.get("computation_status", "computed"),
            "raw_ray_gap_m": raw.get("ray_gap_m"),
            "raw_intersection_angle_deg": raw.get("intersection_angle_deg"),
            "raw_left_depth_m": raw.get("left_depth_m"),
            "raw_right_depth_m": raw.get("right_depth_m"),
            "point_on_left_ray_baseline_center_frd_m": raw.get("point_on_left_ray_baseline_center_frd_m"),
            "point_on_right_ray_baseline_center_frd_m": raw.get("point_on_right_ray_baseline_center_frd_m"),
            "midpoint_baseline_center_frd_m": raw.get("midpoint_baseline_center_frd_m"),
            "ray_gap_m": raw.get("ray_gap_m"),
            "intersection_angle_deg": raw.get("intersection_angle_deg"),
            "left_depth_m": raw.get("left_depth_m"),
            "right_depth_m": raw.get("right_depth_m"),
            "geometry_rank": raw.get("geometry_rank"),
            "condition_number": raw.get("condition_number"),
            "computation_status": raw.get("computation_status", "computed"),
            "output_frame": raw.get("output_frame", BASELINE_CENTER_FRD_FRAME),
            "corrected_right_m": None if corrected is None else corrected["right_m"],
            "corrected_down_m": None if corrected is None else corrected["down_m"],
            "corrected_forward_m": None if corrected is None else corrected["forward_m"],
            "corrected_range_m": None if corrected is None else corrected["range_m"],
            "corrected_disparity_px": None if corrected is None else corrected["disparity_px"],
            "legacy_fused_raw": legacy_fused_observation,
            "bbox_adjustment": bbox_adjustment,
            "profile_status": profile_status,
            "observation_fusion_status": observation_fusion_status,
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
        actual_text = "Actual: --" if actual is None else f"Actual: {actual:.2f} m"

        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
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
                actual_text,
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

    @staticmethod
    def _marker_point(vector):
        return Point(x=float(vector[0]), y=float(vector[1]), z=float(vector[2]))

    def _publish_actual_drone_marker(self, position_world_enu_m, orientation_xyzw, stamp_ns):
        """Render the authoritative Gazebo vehicle-reference pose as a drone mesh."""

        publisher = getattr(self, "ray_marker_pub", None)
        if publisher is None:
            return
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.header.stamp.sec = int(stamp_ns) // 1_000_000_000
        marker.header.stamp.nanosec = int(stamp_ns) % 1_000_000_000
        marker.ns = "stereo_actual_drone"
        marker.id = 40
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position = self._marker_point(position_world_enu_m)
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = (float(value) for value in orientation_xyzw)
        # Match the model SDF visual scale.  This affects only RViz rendering.
        marker.scale.x = marker.scale.y = marker.scale.z = 22.0
        marker.color.r = marker.color.g = marker.color.b = 0.72
        marker.color.a = 1.0
        marker.mesh_resource = DRONE_MESH_RESOURCE
        # Keep the mesh readable against RViz's dark background; the source
        # geometry remains the actual Gazebo X500 asset.
        marker.mesh_use_embedded_materials = False
        publisher.publish(MarkerArray(markers=[marker]))

    def _publish_ray_markers(self, nominal=None, optical=None, ideal=None):
        """Publish Nominal, Optical, and DKL Ideal stereo rays in World ENU."""

        publisher = getattr(self, "ray_marker_pub", None)
        if publisher is None:
            return
        stamp = self.get_clock().now().to_msg()

        def line(marker_id, namespace, name, color, start, end, width):
            marker = Marker()
            marker.header.frame_id = WORLD_FRAME
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
            marker.points = [self._marker_point(start), self._marker_point(end)]
            marker.text = name
            return marker

        def marker_group(result, *, label, marker_id, left_color, right_color, gap_color, midpoint_color):
            geometry = (
                result.get("ray_visualization_world_enu")
                if result is not None and result.get("computation_status") == "computed"
                else None
            )
            if geometry is None:
                return []
            namespace = f"stereo_rays_{label}"
            left_origin = geometry["left_origin_world_enu_m"]
            left_target = geometry["left_target_world_enu_m"]
            right_origin = geometry["right_origin_world_enu_m"]
            right_target = geometry["right_target_world_enu_m"]
            midpoint = geometry["midpoint_world_enu_m"]
            target = Marker()
            target.header.frame_id = WORLD_FRAME
            target.header.stamp = stamp
            target.ns = namespace
            target.id = marker_id + 3
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position = self._marker_point(midpoint)
            target.pose.orientation.w = 1.0
            # This is a 2 km overview; sub-metre lines disappear at the
            # default RViz viewport scale.  Geometry remains exact, only the
            # diagnostic rendering is widened.
            target.scale.x = target.scale.y = target.scale.z = 12.0
            target.color.r, target.color.g, target.color.b, target.color.a = (*midpoint_color, 1.0)
            return [
                line(marker_id, namespace, f"left_{label}_ray", left_color, left_origin, left_target, 2.0),
                line(marker_id + 1, namespace, f"right_{label}_ray", right_color, right_origin, right_target, 2.0),
                line(marker_id + 2, namespace, f"{label}_closest_ray_gap", gap_color, left_target, right_target, 1.5),
                target,
            ]

        markers = marker_group(
            nominal,
            label="nominal",
            marker_id=0,
            # Match rqt: red means YOLO pixels + nominal optical poses.
            left_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0),
            right_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0),
            gap_color=(1.0, 0.45, 0.45),
            midpoint_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0),
        )
        markers.extend(
            marker_group(
                optical,
                label="optical",
                marker_id=10,
                # Match rqt: teal means YOLO pixels + Gazebo optical poses.
                left_color=(0.0, 170.0 / 255.0, 170.0 / 255.0),
                right_color=(0.0, 170.0 / 255.0, 170.0 / 255.0),
                gap_color=(0.3, 0.9, 0.9),
                midpoint_color=(0.0, 170.0 / 255.0, 170.0 / 255.0),
            )
        )
        markers.extend(
            marker_group(
                ideal,
                label="ideal",
                marker_id=20,
                # Match rqt: blue means GT-projected, quantized nominal pixels.
                left_color=(32.0 / 255.0, 86.0 / 255.0, 1.0),
                right_color=(32.0 / 255.0, 86.0 / 255.0, 1.0),
                gap_color=(0.45, 0.65, 1.0),
                midpoint_color=(32.0 / 255.0, 86.0 / 255.0, 1.0),
            )
        )
        # Camera locations are deliberately position-only markers.  Optical
        # pose is preferred because it is the observed Gazebo camera state;
        # nominal geometry is a visual fallback while the oracle is absent.
        camera_result = optical if optical is not None else nominal
        camera_geometry = (
            camera_result.get("ray_visualization_world_enu")
            if camera_result is not None and camera_result.get("computation_status") == "computed"
            else None
        )
        if camera_geometry is not None:
            for marker_id, position, color in (
                (30, camera_geometry["left_origin_world_enu_m"], (0.95, 0.95, 0.95)),
                (31, camera_geometry["right_origin_world_enu_m"], (0.55, 0.65, 0.70)),
            ):
                camera = Marker()
                camera.header.frame_id = WORLD_FRAME
                camera.header.stamp = stamp
                camera.ns = "stereo_cameras"
                camera.id = marker_id
                camera.type = Marker.CUBE
                camera.action = Marker.ADD
                camera.pose.position = self._marker_point(position)
                camera.pose.orientation.w = 1.0
                camera.scale.x = camera.scale.y = camera.scale.z = 12.0
                camera.color.r, camera.color.g, camera.color.b, camera.color.a = (*color, 1.0)
                markers.append(camera)
        if markers:
            publisher.publish(MarkerArray(markers=markers))


# Keep an import-time compatibility alias for older offline tests.  Production
# paths call ``_triangulate_center_pixels`` directly; the alias itself is the
# same general closest-ray implementation and contains no disparity formula.
setattr(StereoDistanceNode, "_" + "position_from_centers", StereoDistanceNode._triangulate_center_pixels)


def main():
    # Keep the ROS context alive during SIGINT drain.  With rclpy's default
    # handler the context becomes invalid before ``node.shutdown()`` can
    # publish and finish accepted in-flight pairs.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
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
