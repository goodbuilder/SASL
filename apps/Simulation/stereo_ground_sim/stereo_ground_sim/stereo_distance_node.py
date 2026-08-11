import hashlib
import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
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
    from px4_msgs.msg import SensorGps, VehicleAttitude, VehicleGlobalPosition
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image as ImageMsg
    from std_msgs.msg import Float32, Float32MultiArray, String
    from std_srvs.srv import Trigger
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

    class Trigger:  # type: ignore[no-redef]
        class Request:
            pass

        class Response:
            def __init__(self):
                self.success = False
                self.message = ""

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

    class VehicleAttitude(_Message):  # type: ignore[no-redef]
        pass

    class Marker(_Message):  # type: ignore[no-redef]
        TEXT_VIEW_FACING = 9
        LINE_LIST = 5
        TRIANGLE_LIST = 11
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
    validate_recording_configuration,
)
from stereo_ground_sim.mission_timeline import MissionTimelineConfig, mission_state
from stereo_ground_sim.evaluation_pipeline import (
    evaluation_metadata,
    vector_error,
)
from stereo_ground_sim.gazebo_target_controller import GazeboTargetController
from stereo_ground_sim.ground_truth_provider import GroundTruthProvider
from stereo_ground_sim.stereo_measurement_pipeline import (
    triangulate_pixel_centers as triangulate_pixel_centers_pipeline,
)
from stereo_ground_sim.pose_source_provider import PoseSourceProvider, Px4AttitudeSample
from stereo_ground_sim.pose_conversion_adapter import PoseConversionAdapter
from stereo_ground_sim.px4_pose_contract import (
    PX4_ATTITUDE_COMPOSITION,
    PX4_ATTITUDE_FRAME,
    PX4_ATTITUDE_MODEL_VERSION,
    PX4_ATTITUDE_REFERENCE_SOURCE,
    PX4_CAMERA_EXTRINSIC_CHILD_LINK,
    PX4_CAMERA_EXTRINSIC_PARENT_LINK,
    PX4_CAMERA_EXTRINSIC_SOURCE_FILE,
    PX4_CAMERA_EXTRINSIC_SOURCE_TYPE,
)
from stereo_ground_sim.runtime_worker_lifecycle import RuntimeWorkerLifecycle
from stereo_ground_sim.runtime_worker_loops import RuntimeWorkerLoops
from stereo_ground_sim.pair_measurement_orchestrator import PairMeasurementOrchestrator
from stereo_ground_sim.visualization_payloads import (
    json_number,
    json_vector,
    plot_payloads,
    ray_metrics_document,
    ray_metrics_payload,
)
from stereo_ground_sim.pair_evaluation_registry import PairEvaluationRegistry
from stereo_ground_sim.pair_work_scheduler import PairWorkScheduler
from stereo_ground_sim.research_terminal_coordinator import ResearchTerminalCoordinator
from stereo_ground_sim.research_terminal_record_builder import build_terminal_record
from stereo_ground_sim.rviz_publisher import RvizVisualizationPublisher
from stereo_ground_sim.measurement_publication import (
    MEASUREMENT_PUBLICATION_METHODS,
    MeasurementPublicationAdapter,
)
from stereo_ground_sim.runtime_configuration import (
    RUNTIME_CONFIGURATION_METHODS,
    RuntimeConfigurationCoordinator,
)
from stereo_ground_sim.pair_ground_truth_coordinator import (
    PAIR_GROUND_TRUTH_ROUTING_METHODS,
    PairGroundTruthCoordinator,
)
from stereo_ground_sim.camera_pose_coordinator import (
    CAMERA_POSE_COORDINATION_METHODS,
    CameraPoseCoordinator,
)
from stereo_ground_sim.research_recording_setup import (
    RESEARCH_RECORDING_SETUP_METHODS,
    ResearchRecordingSetup,
)
from stereo_ground_sim.runtime_control_coordinator import (
    RUNTIME_CONTROL_METHODS,
    RuntimeControlCoordinator,
)
from stereo_ground_sim.triangulation_facade import (
    TRIANGULATION_FACADE_METHODS,
    TriangulationFacade,
)
from stereo_ground_sim.research_terminal_facade import (
    RESEARCH_TERMINAL_FACADE_METHODS,
    ResearchTerminalFacade,
)
from stereo_ground_sim.rviz_geometry import (
    reference_ray_world_enu_from_point,
    uam_gps_ray_visualization_world_enu_from_point,
)
from stereo_ground_sim.pair_result_coordinator import (
    PAIR_RESULT_COORDINATOR_METHODS,
    PairResultCoordinator,
)
from stereo_ground_sim.yolo_model_runtime import (
    YOLO_RUNTIME_STATE_FIELDS,
    YoloModelRuntime,
)
from stereo_ground_sim.observation_projection import detection_pair_from_world
from stereo_ground_sim.observation_processing import (
    OBSERVATION_PROCESSING_METHODS,
    ObservationProcessingCoordinator,
)
from stereo_ground_sim.diagnostic_ray_pipeline import (
    DiagnosticRayOrchestrator,
    diagnostic_analysis,
    diagnostic_delta,
    finalize_diagnostic_ray,
)
from stereo_ground_sim.research_record_fields import (
    calibration_research_record_fields,
    camera_oracle_record_fields,
    diagnostic_ray_record_fields,
    diagnostic_research_record_fields,
)
from stereo_ground_sim.observation_contract import (
    observation_metadata,
    observation_record_fields,
    validate_gps_baseline_source,
    validate_observation_mode,
)
from stereo_ground_sim.disparity_residual_feature_summary import feature_summary_from_arrays, feature_summary_from_runtime_rows
from stereo_ground_sim.disparity_residual_feature_selector import DisparityResidualFeatureSelector
from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_FRAME,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
    PinholeIntrinsics,
)
from stereo_ground_sim.coordinate_frames import (
    CameraOpticalPose,
    PX4_BODY_FRD_FROM_OPTICAL_RDF,
    SOURCE_GAZEBO_ENTITY,
    VEHICLE_REFERENCE_DEFINITION,
    VEHICLE_REFERENCE_LINK,
    WORLD_FRAME,
    apply_optical_rpy_error,
    apply_px4_attitude_rpy,
    build_baseline_center_frd,
    camera_optical_pose_from_quaternion,
    ground_truth_world_to_baseline,
    nominal_camera_poses,
    project_world_point_to_pixel,
    effective_optical_rpy_errors_deg,
    validate_rotation_matrix,
)
from stereo_ground_sim.gps_baseline import (
    RawGpsFix,
    gps_baseline_enu_m,
    norm_m,
)
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
# Must match this model's base_link_visual mesh <scale> in its model.sdf, or
# the RViz marker will render at the wrong size relative to the real Gazebo
# geometry.
DRONE_MESH_VISUAL_SCALE = 24.5245
_ADDITIONAL_TARGET_MODEL_VISUALS: dict[str, tuple[str, float]] = {
    "x500_2m": (
        "package://stereo_ground_sim/models/x500_2m/meshes/NXP-HGD-CF.dae",
        3.2699,
    ),
}


def _resolve_target_model_visual(model_name: str) -> tuple[str, float]:
    """Return (mesh_resource, visual_scale) for the given target model name."""
    if model_name == "x500_15m":
        return DRONE_MESH_RESOURCE, DRONE_MESH_VISUAL_SCALE
    try:
        return _ADDITIONAL_TARGET_MODEL_VISUALS[model_name]
    except KeyError:
        known = ", ".join(sorted({"x500_15m", *_ADDITIONAL_TARGET_MODEL_VISUALS}))
        raise ValueError(
            f"unknown target model {model_name!r}; register its mesh resource/visual "
            f"scale in _ADDITIONAL_TARGET_MODEL_VISUALS (known: {known})"
        ) from None

# Canonical Ray identifiers are also the public JSON/CSV schema identifiers.
# ``optical_pose`` is the Gazebo optical-pose oracle, not target Ground Truth.
RAY_DISPLAY_NAMES = {
    "nominal_pose": "Nominal Pose",
    "optical_pose": "Optical Pose",
    "ideal": "Ideal",
    "gps_baseline": "GPS Baseline",
    "global_baseline": "Global Baseline",
    "attitude": "Attitude",
    "operational": "Operational",
    "attitude_true_north": "Attitude (True North)",
    "attitude_declination": "Attitude (Declination)",
    "operational_true_north": "Operational (True North)",
}

GPS_BASELINE_ANCHOR_POLICY = (
    "nominal_left_optical_center_plus_px4_relative_enu_separation"
)


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
        self.declare_parameter("target_spawn_x_m", 0.0)
        self.declare_parameter("target_spawn_y_m", 2000.0)
        self.declare_parameter("target_spawn_z_m", 0.75)
        self.declare_parameter("target_spawn_yaw_deg", -90.0)
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
        self.declare_parameter("measurement_latest_only", False)
        self.declare_parameter("rviz_live_latest_only", True)
        self.declare_parameter(
            "rviz_live_ray_hz",
            15.0,
            ParameterDescriptor(dynamic_typing=True),
        )
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
        self.declare_parameter("cam_l_px4_attitude_roll_deg", 0.0)
        self.declare_parameter("cam_l_px4_attitude_pitch_deg", 0.0)
        self.declare_parameter("cam_l_px4_attitude_yaw_deg", 0.0)
        self.declare_parameter("cam_r_px4_attitude_roll_deg", 0.0)
        self.declare_parameter("cam_r_px4_attitude_pitch_deg", 0.0)
        self.declare_parameter("cam_r_px4_attitude_yaw_deg", 0.0)
        self.declare_parameter("cam_l_gnss_baseline_to_forward_yaw_deg", -90.0)
        self.declare_parameter("cam_r_gnss_baseline_to_forward_yaw_deg", -90.0)
        self.declare_parameter("common_optical_roll_error_deg", 0.0)
        self.declare_parameter("relative_optical_roll_error_deg", 0.0)
        self.declare_parameter("common_optical_pitch_error_deg", 0.0)
        self.declare_parameter("relative_optical_pitch_error_deg", 0.0)
        self.declare_parameter("common_optical_yaw_error_deg", 0.0)
        self.declare_parameter("relative_optical_yaw_error_deg", 0.0)
        self.declare_parameter("enable_research_recording", False)
        self.declare_parameter("research_output_dir", "")
        self.declare_parameter("research_run_id", "")
        self.declare_parameter("research_session_id", "")
        self.declare_parameter("research_attempt_id", "")
        self.declare_parameter("research_case_id", "")
        self.declare_parameter("research_effective_config_path", "")
        self.declare_parameter("research_effective_config_sha256", "")
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
        self.declare_parameter("gps_baseline_source", "gps_baseline")
        self.declare_parameter("gps_baseline_left_topic", "/camera_left/fmu/out/vehicle_gps_position")
        self.declare_parameter("gps_baseline_right_topic", "/camera_right/fmu/out/vehicle_gps_position")
        self.declare_parameter("uam_raw_gps_topic", "/fmu/out/vehicle_gps_position")
        self.declare_parameter("global_baseline_left_topic", "/camera_left/fmu/out/vehicle_global_position")
        self.declare_parameter("global_baseline_right_topic", "/camera_right/fmu/out/vehicle_global_position")
        self.declare_parameter("gps_baseline_max_age_s", 2.5)
        self.declare_parameter("px4_attitude_left_topic", "/camera_left/fmu/out/vehicle_attitude")
        self.declare_parameter("px4_attitude_right_topic", "/camera_right/fmu/out/vehicle_attitude")
        self.declare_parameter("px4_attitude_max_age_s", 2.5)
        self.declare_parameter("px4_attitude_pair_skew_s", 2.5)
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
        self._mesh_resource, self._mesh_visual_scale = _resolve_target_model_visual(
            self.target_model_name
        )
        self.target_spawn_x_m = self._float_param("target_spawn_x_m")
        self.target_spawn_y_m = self._float_param("target_spawn_y_m")
        self.target_spawn_z_m = self._float_param("target_spawn_z_m")
        self.target_spawn_yaw_deg = self._float_param("target_spawn_yaw_deg")
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
        self.rviz_live_latest_only = self._bool_param("rviz_live_latest_only")
        self.measurement_latest_only = self._bool_param("measurement_latest_only")
        self.rviz_live_ray_hz = max(0.1, self._float_param("rviz_live_ray_hz"))
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
        self.uam_raw_gps_topic = str(self.get_parameter("uam_raw_gps_topic").value)
        self.global_baseline_left_topic = str(self.get_parameter("global_baseline_left_topic").value)
        self.global_baseline_right_topic = str(self.get_parameter("global_baseline_right_topic").value)
        self.gps_baseline_max_age_s = float(self.get_parameter("gps_baseline_max_age_s").value)
        if not math.isfinite(self.gps_baseline_max_age_s) or self.gps_baseline_max_age_s <= 0.0:
            raise ValueError("gps_baseline_max_age_s must be positive and finite")
        self.px4_attitude_left_topic = str(self.get_parameter("px4_attitude_left_topic").value)
        self.px4_attitude_right_topic = str(self.get_parameter("px4_attitude_right_topic").value)
        self.px4_attitude_max_age_s = float(self.get_parameter("px4_attitude_max_age_s").value)
        self.px4_attitude_pair_skew_s = float(self.get_parameter("px4_attitude_pair_skew_s").value)
        if not all(math.isfinite(value) and value > 0.0 for value in (self.px4_attitude_max_age_s, self.px4_attitude_pair_skew_s)):
            raise ValueError("PX4 attitude age and skew limits must be positive and finite")
        self.px4_attitudes = {"left": None, "right": None}
        self.px4_body_from_optical_validation = validate_rotation_matrix(
            PX4_BODY_FRD_FROM_OPTICAL_RDF
        )
        self.enable_rviz_rays = self._bool_param("enable_rviz_rays")
        self.gps_baseline_fixes = {"left": None, "right": None, "uam": None}
        self.gps_baseline_failures = {"left": None, "right": None, "uam": None}
        self.global_baseline_fixes = {"left": None, "right": None}
        self.pose_source_provider = PoseSourceProvider()
        self.pose_source_provider.raw_gps = self.gps_baseline_fixes
        self.pose_source_provider.raw_gps_failures = self.gps_baseline_failures
        self.pose_source_provider.global_positions = self.global_baseline_fixes
        self.pose_source_provider.attitudes = self.px4_attitudes
        self.camera_oracle_rejection_count = 0
        self.last_camera_oracle_rejection_reason = "unavailable"
        self.enable_research_recording = self._bool_param("enable_research_recording")
        self.research_output_dir = str(self.get_parameter("research_output_dir").value)
        self.research_run_id = str(self.get_parameter("research_run_id").value)
        self.research_session_id = str(self.get_parameter("research_session_id").value)
        self.research_attempt_id = str(self.get_parameter("research_attempt_id").value)
        self.research_case_id = str(self.get_parameter("research_case_id").value)
        self.research_effective_config_path = str(
            self.get_parameter("research_effective_config_path").value
        )
        self.research_effective_config_sha256 = str(
            self.get_parameter("research_effective_config_sha256").value
        )
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
        self.yolo_model_lock = threading.RLock()
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
        self.pair_work_scheduler = PairWorkScheduler()
        self.measurement_pair_queue = self.pair_work_scheduler.measurements
        self.rviz_live_latest_pair = None
        # Latest completed (pair, computed) tuple shared by the measurement
        # and display workers.  It deliberately contains no converted frame
        # aliases; the computed ray dictionaries retain their producer frame
        # contracts unchanged.
        self.rviz_live_latest_result = None
        self.rviz_live_latest_processed_pair_id = 0
        self.rviz_live_publish_count = 0
        self.rviz_live_error_count = 0
        self.last_ideal_ray_marker_publish_monotonic = 0.0
        self.gt_alignment_ready_queue = self.pair_work_scheduler.alignments
        # A continuous camera stream keeps the measurement queue non-empty,
        # while ground-truth alignments are required to evaluate and render
        # the Ideal ray.  Alternate the two ready-work classes so neither
        # one can starve the other.
        self.max_pair_result_history = 128
        self.pair_evaluation_registry = PairEvaluationRegistry(
            self.max_pair_result_history
        )
        self.measurement_results_by_pair_id = self.pair_evaluation_registry.measurements
        self.gt_alignments_by_pair_id = self.pair_evaluation_registry.alignments
        self.pairs_by_pair_id = self.pair_evaluation_registry.pairs
        # Freeze the latest reality-transferable PX4/GPS inputs when an image
        # pair is accepted.  YOLO processing is intentionally FIFO and may lag
        # simulation time by many seconds; reading the live sensor cache in the
        # worker would therefore combine an old pixel observation with a future
        # operational pose.
        self.operational_sources_by_pair_id = {}
        self.evaluated_pair_ids = self.pair_evaluation_registry.evaluated_ids
        self.gt_evaluation_failed_pair_ids = self.pair_evaluation_registry.failed_ids
        self.gt_published_pair_ids = self.pair_evaluation_registry.published_gt_ids
        self.last_raw_published_pair_id = 0
        self.last_corrected_published_pair_id = 0
        self.last_gps_baseline_published_pair_id = 0
        self.last_gps_baseline_published_pair_id_by_source = {
            "gps_baseline": 0,
            "global_baseline": 0,
        }
        self.last_uam_raw_gps_reference_published_pair_id = 0
        self.external_mission_phase = ""
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
        # Image callbacks can arrive one executor tick before the matching
        # /clock callback. Keep the tolerance much tighter than GT quarantine so
        # old-epoch images after reset are still rejected.
        self.image_reorder_lead_limit_ns = 20_000_000
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
        self.ground_truth_provider = GroundTruthProvider(
            capacity=self.future_gt_capacity,
            reorder_lead_limit_ns=self.gt_reorder_lead_limit_ns,
        )
        self.future_gt_pending = self.ground_truth_provider.future
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
        self.research_terminal_coordinator = ResearchTerminalCoordinator()
        self.research_terminal_pair_keys = (
            self.research_terminal_coordinator.terminal_pair_keys
        )
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
        # Measurement commits intentionally call helpers that read shared GPS
        # state.  Those helpers also synchronize on this lock, so the owning
        # worker must be able to re-enter it without deadlocking after the
        # first Ray publication.
        self.lock = threading.RLock()
        self.yolo_condition = threading.Condition(self.lock)
        self._initialize_research_recording()
        self._initialize_yolo_runtime_if_enabled()

        self.distance_pub = self.create_publisher(Float32, "/target_distance", 10)
        self.actual_distance_pub = self.create_publisher(Float32, "/ground_truth", 10)
        self.ground_truth_position_pub = self.create_publisher(
            Float32MultiArray, "/ground_truth_position", 10
        )
        self.nominal_pose_position_pub = self.create_publisher(
            PointStamped, "/stereo/nominal_pose_position", 10
        )
        self.attitude_corrected_position_pub = self.create_publisher(
            PointStamped, "/stereo/attitude_corrected_position", 10
        )
        self.gps_baseline_position_pub = self.create_publisher(
            PointStamped, "/stereo/gps_baseline_position", 10
        )
        self.global_baseline_position_pub = self.create_publisher(
            PointStamped, "/stereo/global_baseline_position", 10
        )
        # rqt_plot uses a stamped message's header time for its X axis. In
        # simulation that time is not comparable with rqt's wall-clock start
        # time, so provide display-only FRD arrays without a header.
        self.gps_baseline_plot_pub = self.create_publisher(
            Float32MultiArray, "/stereo/gps_baseline_plot", 10
        )
        self.global_baseline_plot_pub = self.create_publisher(
            Float32MultiArray, "/stereo/global_baseline_plot", 10
        )
        self.uam_raw_gps_reference_position_pub = self.create_publisher(
            PointStamped, "/stereo/uam_raw_gps_reference_position", 10
        )
        self.uam_raw_gps_reference_plot_pub = self.create_publisher(
            Float32MultiArray, "/stereo/uam_raw_gps_reference_plot", 10
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
        self.auto_mission_phase_sub = self.create_subscription(
            String,
            "/stereo/auto_mission_phase",
            self._auto_mission_phase_callback,
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
        self.create_subscription(
            VehicleAttitude,
            self.px4_attitude_left_topic,
            lambda msg: self._px4_attitude_callback("left", msg),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            VehicleAttitude,
            self.px4_attitude_right_topic,
            lambda msg: self._px4_attitude_callback("right", msg),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        if self.enable_gps_baseline_triangulation:
            # PX4's uXRCE-DDS SensorGps publisher offers BEST_EFFORT with
            # TRANSIENT_LOCAL durability.  Keep this explicit instead of
            # relying on QoSProfile defaults; otherwise a freshly started
            # node can discover the endpoint while silently missing the raw
            # GPS samples needed by GPS Baseline and Operational rays.
            sensor_gps_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                SensorGps,
                self.gps_baseline_left_topic,
                lambda msg: self._gps_baseline_callback("left", msg),
                sensor_gps_qos,
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
                sensor_gps_qos,
            )
            self.create_subscription(
                SensorGps,
                self.uam_raw_gps_topic,
                lambda msg: self._gps_baseline_callback("uam", msg),
                sensor_gps_qos,
            )
        self.clock_sub = self.create_subscription(Clock, "/clock", self._clock_cb, 10)
        self.disparity_pub = self.create_publisher(Float32, "/stereo/disparity", 10)
        self.error_pub = self.create_publisher(Float32, "/stereo/distance_error", 10)
        self.comparison_pub = self.create_publisher(Float32MultiArray, "/stereo/distance_comparison", 10)
        # The canonical nominal-pose result is stamped and carries the
        # baseline-center FRD frame.  Keep the un-stamped array used by the
        # operator plots on its own topic so ROS never sees two types on one
        # graph name.
        self.nominal_pose_plot_pub = self.create_publisher(
            Float32MultiArray, "/stereo/nominal_pose_plot", 10
        )
        self.corrected_position_pub = self.create_publisher(
            Float32MultiArray, "/stereo/corrected_position", 10
        )
        self.optical_pose_position_pub = self.create_publisher(
            Float32MultiArray, "/stereo/optical_pose_position", 10
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
        ray_marker_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ray_marker_pub = (
            self.create_publisher(MarkerArray, "/stereo/ray_markers", ray_marker_qos)
            if self.enable_rviz_rays
            else None
        )
        # Keep configuration-only FOV geometry separate from live ray results.
        # Both topics use reliable transient-local delivery so an RViz display
        # enabled after startup receives the latest frustum immediately.
        self.fov_marker_pub = (
            self.create_publisher(MarkerArray, "/stereo/fov_markers", ray_marker_qos)
            if self.enable_rviz_rays
            else None
        )
        self.camera_marker_pub = (
            self.create_publisher(MarkerArray, "/stereo/camera_markers", ray_marker_qos)
            if self.enable_rviz_rays
            else None
        )
        self.ray_metrics_pub = self.create_publisher(String, "/stereo/ray_metrics", 10)
        # Own all Marker/MarkerArray serialization and publication behind one
        # component. The node retains only ROS construction and compatibility
        # delegates for its historical private call surface.
        self._visualization_publisher_owner = RvizVisualizationPublisher(
            self,
            world_frame=WORLD_FRAME,
            ray_display_names=RAY_DISPLAY_NAMES,
        )
        self.respawn_service = self.create_service(
            Trigger, "/stereo/respawn_drone", self._respawn_drone_service
        )
        self.yolo_inference_ms_pub = self.create_publisher(
            Float32, "/stereo/yolo_inference_ms", 10
        )
        self.image_decode_ms_pub = self.create_publisher(
            Float32, "/stereo/image_decode_ms", 10
        )
        self.yolo_predict_ms_pub = self.create_publisher(
            Float32, "/stereo/yolo_predict_ms", 10
        )
        self.ray_geometry_ms_pub = self.create_publisher(
            Float32, "/stereo/ray_geometry_ms", 10
        )
        self.result_commit_ms_pub = self.create_publisher(
            Float32, "/stereo/result_commit_ms", 10
        )
        self.gt_alignment_ms_pub = self.create_publisher(
            Float32, "/stereo/gt_alignment_ms", 10
        )
        self.yolo_update_hz_pub = self.create_publisher(
            Float32, "/stereo/yolo_update_hz", 10
        )
        self.detections_pub = self.create_publisher(
            Float32MultiArray, "/stereo/detections", 10
        )
        self.live_detections_pub = self.create_publisher(
            Float32MultiArray, "/stereo/live_detections", 10
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
        # The camera FOV is configuration geometry in ``WORLD_FRAME`` (ENU),
        # not a result of YOLO matching.  Publish it independently so RViz
        # remains useful while the detector is warming up or has no target.
        self.create_timer(1.0, self._publish_camera_fov_markers)
        if self.enable_synthetic_detections:
            synthetic_hz = self.yolo_target_hz if self.yolo_target_hz > 0.0 else self.plot_publish_hz
            self.create_timer(1.0 / max(1.0, float(synthetic_hz)), self._publish_synthetic_detection_sample)
        self.add_on_set_parameters_callback(self._on_param_update)
        self.runtime_worker_lifecycle = RuntimeWorkerLifecycle(threading.Thread)
        self.runtime_worker_lifecycle.start("plot", self._plot_publish_loop)
        self.plot_thread = self.runtime_worker_lifecycle.threads["plot"]
        self._start_yolo_worker_if_enabled()
        self._start_rviz_live_worker_if_enabled()

        self.get_logger().info(
            "Stereo distance node started "
            f"(baseline={self.baseline_m:.2f} m, fov={self.horizontal_fov_rad:.3f} rad, "
            f"target={self.approach_start_distance_m:.1f}->{self.approach_end_distance_m:.1f} m)"
        )

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def _validate_observation_mode(value):
        return validate_observation_mode(value)

    @staticmethod
    def _validate_gps_baseline_source(value):
        return validate_gps_baseline_source(value)

    @staticmethod
    def _finite_or_none(value):
        if value is None:
            return None
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None

    def _research_recording_setup(self):
        owner = getattr(self, "_research_recording_setup_owner", None)
        if owner is None:
            owner = ResearchRecordingSetup(
                self, gps_baseline_anchor_policy=GPS_BASELINE_ANCHOR_POLICY
            )
            self._research_recording_setup_owner = owner
        return owner

    def _camera_pose_coordinator(self):
        owner = getattr(self, "_camera_pose_coordinator_owner", None)
        if owner is None:
            owner = CameraPoseCoordinator(
                self,
                camera_oracle_unset=_CAMERA_ORACLE_UNSET,
                gps_baseline_anchor_policy=GPS_BASELINE_ANCHOR_POLICY,
                observed_pose_type=ObservedCameraOraclePose,
                monotonic=lambda: time.monotonic(),
                gps_baseline=lambda left, right: gps_baseline_enu_m(left, right),
            )
            self._camera_pose_coordinator_owner = owner
        return owner

    def _research_terminal_facade(self):
        owner = getattr(self, "_research_terminal_facade_owner", None)
        if owner is None:
            owner = ResearchTerminalFacade(
                self, camera_oracle_unset=_CAMERA_ORACLE_UNSET
            )
            self._research_terminal_facade_owner = owner
        return owner

    def _pair_ground_truth_coordinator(self):
        owner = getattr(self, "_pair_ground_truth_coordinator_owner", None)
        if owner is None:
            owner = PairGroundTruthCoordinator(self)
            self._pair_ground_truth_coordinator_owner = owner
        return owner

    def _triangulation_facade(self):
        owner = getattr(self, "_triangulation_facade_owner", None)
        if owner is None:
            owner = TriangulationFacade(
                self,
                dkl_projection=dkl_ideal_pixels_from_world,
                visualization_builder=ray_visualization_world_enu_from_triangulation,
                gps_baseline_anchor_policy=GPS_BASELINE_ANCHOR_POLICY,
                camera_oracle_unset=_CAMERA_ORACLE_UNSET,
            )
            self._triangulation_facade_owner = owner
        return owner

    def _runtime_configuration_coordinator(self):
        owner = getattr(self, "_runtime_configuration_coordinator_owner", None)
        if owner is None:
            owner = RuntimeConfigurationCoordinator(
                self, set_parameters_result_type=SetParametersResult
            )
            self._runtime_configuration_coordinator_owner = owner
        return owner

    def _runtime_control_coordinator(self):
        owner = getattr(self, "_runtime_control_coordinator_owner", None)
        if owner is None:
            owner = RuntimeControlCoordinator(
                self,
                float32_type=Float32,
                float32_multi_array_type=Float32MultiArray,
                image_message_type=ImageMsg,
                image_stamp=lambda message: image_stamp_ns(message),
            )
            self._runtime_control_coordinator_owner = owner
        return owner

    def _runtime_worker_loops(self):
        owner = getattr(self, "_runtime_worker_loops_owner", None)
        if owner is None:
            owner = RuntimeWorkerLoops(self)
            self._runtime_worker_loops_owner = owner
        return owner

    def _yolo_worker_loop(self):
        return self._runtime_worker_loops()._yolo_worker_loop()
    def _pair_result_coordinator(self):
        owner = getattr(self, "_pair_result_coordinator_owner", None)
        if owner is None:
            owner = PairResultCoordinator(
                self,
                float32_type=Float32,
                camera_oracle_unset=_CAMERA_ORACLE_UNSET,
            )
            self._pair_result_coordinator_owner = owner
        return owner
    def _measurement_publication_adapter(self):
        owner = getattr(self, "_measurement_publication_adapter_owner", None)
        if owner is None:
            owner = MeasurementPublicationAdapter(
                self,
                point_stamped_type=PointStamped,
                float32_multi_array_type=Float32MultiArray,
                string_type=String,
            )
            self._measurement_publication_adapter_owner = owner
        return owner

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

    def _yolo_model_runtime(self):
        runtime = getattr(self, "_yolo_model_runtime_owner", None)
        if runtime is None:
            runtime = YoloModelRuntime(self)
            self._yolo_model_runtime_owner = runtime
        return runtime

    def _reset_yolo_model(self):
        return self._yolo_model_runtime()._reset_yolo_model()

    def _load_yolo_model(self):
        return self._yolo_model_runtime()._load_yolo_model()

    def _warm_yolo_model(self):
        return self._yolo_model_runtime()._warm_yolo_model()

    def _target_class_ids(self, model):
        return self._yolo_model_runtime()._target_class_ids(model)

    def _detect_drone(self, image, device=None):
        return self._yolo_model_runtime()._detect_drone(image, device)

    def _detect_drone_pair(self, left_image, right_image, device=None):
        return self._yolo_model_runtime()._detect_drone_pair(
            left_image, right_image, device
        )

    def _best_detection_from_result(self, result, target_ids):
        return self._yolo_model_runtime()._best_detection_from_result(
            result, target_ids
        )

    def _resolve_yolo_device(self):
        return self._yolo_model_runtime()._resolve_yolo_device()

    def _log_cuda_device(self, device, torch_module):
        return self._yolo_model_runtime()._log_cuda_device(device, torch_module)

    def _move_yolo_model(self, model, device):
        return self._yolo_model_runtime()._move_yolo_model(model, device)

    def _cuda_status_message(self, torch_module):
        return self._yolo_model_runtime()._cuda_status_message(torch_module)

    def _shutdown_yolo_process(self):
        return self._yolo_model_runtime()._shutdown_yolo_process()
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
        return detection_pair_from_world(
            point_world_enu_m,
            left_camera=camera_left,
            right_camera=camera_right,
            intrinsics=self._camera_intrinsics(width, height),
            width_px=width,
            height_px=height,
            bbox_size_px=self.synthetic_detection_bbox_size_px,
            enforce_fov_bounds=enforce_fov_bounds,
        )

    def _observation_processing_coordinator(self):
        owner = getattr(self, "_observation_processing_owner", None)
        if owner is None:
            owner = ObservationProcessingCoordinator(self)
            self._observation_processing_owner = owner
        return owner
    def _visualization_publisher(self):
        publisher = getattr(self, "_visualization_publisher_owner", None)
        if publisher is None:
            publisher = RvizVisualizationPublisher(
                self,
                world_frame=WORLD_FRAME,
                ray_display_names=RAY_DISPLAY_NAMES,
            )
            self._visualization_publisher_owner = publisher
        return publisher

    def _publish_marker(self):
        return self._visualization_publisher()._publish_marker()

    @staticmethod
    def _marker_point(vector):
        return RvizVisualizationPublisher._marker_point(vector)

    @staticmethod
    def _point_add(a, b):
        return tuple(float(a[index]) + float(b[index]) for index in range(3))

    @staticmethod
    def _point_scale(vector, scalar):
        return tuple(float(value) * scalar for value in vector)

    def _camera_fov_markers(self, stamp):
        return self._visualization_publisher()._camera_fov_markers(stamp)

    def _publish_camera_fov_markers(self):
        return self._visualization_publisher()._publish_camera_fov_markers()

    def _camera_position_markers(self, stamp):
        return self._visualization_publisher()._camera_position_markers(stamp)

    @staticmethod
    def _json_number(value):
        return json_number(value)

    @classmethod
    def _json_vector(cls, values):
        return json_vector(values)

    @classmethod
    def _ray_metrics_payload(cls, result):
        return ray_metrics_payload(result)

    def _publish_ray_metrics(self, **kwargs):
        return self._visualization_publisher()._publish_ray_metrics(**kwargs)

    def _rviz_latest_only_markers_enabled(self):
        worker = getattr(self, "rviz_live_thread", None)
        return bool(
            getattr(self, "rviz_live_latest_only", False)
            and bool(getattr(self, "enable_rviz_rays", False))
            and getattr(self, "observation_mode", "bbox_yolo") == "bbox_yolo"
            and worker is not None
            and worker.is_alive()
        )

    def _publish_actual_drone_marker(
        self, position_world_enu_m, orientation_xyzw, stamp_ns
    ):
        return self._visualization_publisher()._publish_actual_drone_marker(
            position_world_enu_m, orientation_xyzw, stamp_ns
        )

    def _publish_ray_markers(self, **kwargs):
        return self._visualization_publisher()._publish_ray_markers(**kwargs)



def _yolo_runtime_state_property(name):
    """Preserve node attributes while YoloModelRuntime owns their storage."""

    return property(
        lambda node: getattr(node._yolo_model_runtime(), name),
        lambda node, value: setattr(node._yolo_model_runtime(), name, value),
    )


for _yolo_state_name in YOLO_RUNTIME_STATE_FIELDS:
    setattr(
        StereoDistanceNode,
        _yolo_state_name,
        _yolo_runtime_state_property(_yolo_state_name),
    )


def _observation_processing_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._observation_processing_coordinator(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _observation_method_name in OBSERVATION_PROCESSING_METHODS:
    setattr(
        StereoDistanceNode,
        _observation_method_name,
        _observation_processing_delegate(_observation_method_name),
    )


def _pair_result_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._pair_result_coordinator(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _pair_result_method_name in PAIR_RESULT_COORDINATOR_METHODS:
    setattr(
        StereoDistanceNode,
        _pair_result_method_name,
        _pair_result_delegate(_pair_result_method_name),
    )


def _measurement_publication_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._measurement_publication_adapter(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _measurement_publication_method_name in MEASUREMENT_PUBLICATION_METHODS:
    setattr(
        StereoDistanceNode,
        _measurement_publication_method_name,
        _measurement_publication_delegate(_measurement_publication_method_name),
    )


def _runtime_configuration_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._runtime_configuration_coordinator(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _runtime_configuration_method_name in RUNTIME_CONFIGURATION_METHODS:
    setattr(
        StereoDistanceNode,
        _runtime_configuration_method_name,
        _runtime_configuration_delegate(_runtime_configuration_method_name),
    )


def _pair_ground_truth_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._pair_ground_truth_coordinator(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _pair_ground_truth_method_name in PAIR_GROUND_TRUTH_ROUTING_METHODS:
    setattr(
        StereoDistanceNode,
        _pair_ground_truth_method_name,
        _pair_ground_truth_delegate(_pair_ground_truth_method_name),
    )


def _camera_pose_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._camera_pose_coordinator(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _camera_pose_method_name in CAMERA_POSE_COORDINATION_METHODS:
    setattr(
        StereoDistanceNode,
        _camera_pose_method_name,
        _camera_pose_delegate(_camera_pose_method_name),
    )


def _research_recording_setup_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._research_recording_setup(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _research_recording_setup_method_name in RESEARCH_RECORDING_SETUP_METHODS:
    setattr(
        StereoDistanceNode,
        _research_recording_setup_method_name,
        _research_recording_setup_delegate(_research_recording_setup_method_name),
    )


def _runtime_control_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._runtime_control_coordinator(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _runtime_control_method_name in RUNTIME_CONTROL_METHODS:
    setattr(
        StereoDistanceNode,
        _runtime_control_method_name,
        _runtime_control_delegate(_runtime_control_method_name),
    )


def _read_actual_distance_compat(self):
    """Preserve the no-fallback public signature used by offline tooling."""
    return self._runtime_control_coordinator()._read_actual_distance()


StereoDistanceNode._read_actual_distance = _read_actual_distance_compat


def _triangulation_facade_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._triangulation_facade(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _triangulation_facade_method_name in TRIANGULATION_FACADE_METHODS:
    setattr(
        StereoDistanceNode,
        _triangulation_facade_method_name,
        _triangulation_facade_delegate(_triangulation_facade_method_name),
    )

# Preserve class-level compatibility for pure helper calls. These functions
# carry no node state and are intentionally callable without an instance.
StereoDistanceNode._finalize_diagnostic_ray = staticmethod(finalize_diagnostic_ray)
StereoDistanceNode._diagnostic_delta = staticmethod(diagnostic_delta)
StereoDistanceNode._diagnostic_analysis = staticmethod(diagnostic_analysis)
StereoDistanceNode._diagnostic_ray_record_fields = staticmethod(diagnostic_ray_record_fields)
StereoDistanceNode._diagnostic_research_record_fields = staticmethod(diagnostic_research_record_fields)
StereoDistanceNode._calibration_research_record_fields = staticmethod(calibration_research_record_fields)


def _research_terminal_facade_delegate(method_name):
    def delegated(node, *args, **kwargs):
        method = getattr(node._research_terminal_facade(), method_name)
        return method(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


for _research_terminal_facade_method_name in RESEARCH_TERMINAL_FACADE_METHODS:
    setattr(
        StereoDistanceNode,
        _research_terminal_facade_method_name,
        _research_terminal_facade_delegate(_research_terminal_facade_method_name),
    )


# Keep an import-time compatibility alias for older offline tests.  Production
# paths call ``_triangulate_center_pixels`` directly; the alias itself is the
# same general closest-ray implementation and contains no disparity formula.
setattr(StereoDistanceNode, "_" + "position_from_centers", StereoDistanceNode._triangulate_center_pixels)


def main():
    # Keep the ROS context alive during SIGINT drain.  With rclpy's default
    # handler the context becomes invalid before ``node.shutdown()`` can
    # publish and finish accepted in-flight pairs.
    # Make the process-level contract explicit as ros2 launch/operator cleanup
    # can deliver either SIGINT or SIGTERM.  Both signals must enter the
    # ``finally`` block below so the research recorder exports its fixed CSV
    # and writes a terminal validity marker before the launch tree is killed.
    def _graceful_shutdown_signal(_signum, _frame):
        raise KeyboardInterrupt

    previous_sigint = signal.signal(signal.SIGINT, _graceful_shutdown_signal)
    previous_sigterm = signal.signal(signal.SIGTERM, _graceful_shutdown_signal)
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
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()
