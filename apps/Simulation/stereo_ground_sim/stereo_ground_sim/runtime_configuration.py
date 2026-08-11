"""Runtime parameter validation and deployment-profile loading."""

import math
import json
import os
from collections import deque
from pathlib import Path

from stereo_ground_sim.disparity_residual_feature_selector import DisparityResidualFeatureSelector
from stereo_ground_sim.reprojection_profile import OriginBBoxReprojectionProfile
from stereo_ground_sim.runtime_deployment_selector import (
    DisparityResidualRuntimeProfile,
    MultiBurstObservationFusion,
    RepeatedObservationFusion,
    RuntimeStereoCameraContract,
    load_observation_fusion_adapter_from_manifest,
)
from stereo_ground_sim.runtime_guard_policy import (
    BoundedGainForwardCellGuardPolicy,
    EstimatedToTruthAffineProfile,
    EstimatedToTruthDisparityProfile,
    EstimatedToTruthSim3Profile,
)

RUNTIME_CONFIGURATION_METHODS = [
    "_on_param_update",
    "_observation_fusion_required_observations",
    "_reset_observation_fusion_buffer",
    "_load_observation_fusion_adapter",
    "_load_disparity_residual_profile",
    "_load_disparity_residual_feature_selector",
    "_reset_disparity_residual_feature_selector_window",
    "_load_bbox_center_adjustment",
    "_load_bbox_output_profile"
]


class RuntimeConfigurationCoordinator:
    def __init__(self, node, *, set_parameters_result_type):
        self.node = node
        self.set_parameters_result_type = set_parameters_result_type

    def _on_param_update(self, params):
        if any(param.name in self.node._recording_parameter_names() for param in params):
            return self.set_parameters_result_type(
                successful=False,
                reason="research recording and camera sampling parameters are immutable for one run",
            )
        if any(param.name == "observation_mode" for param in params):
            return self.set_parameters_result_type(
                successful=False,
                reason="observation_mode is immutable for one run",
            )
        for param in params:
            if param.name == "gps_baseline_source":
                try:
                    self.node._validate_gps_baseline_source(param.value)
                except ValueError as exc:
                    return self.set_parameters_result_type(successful=False, reason=str(exc))
        if any(param.name == "measurement_queue_capacity" for param in params):
            return self.set_parameters_result_type(
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
            "cam_l_px4_attitude_roll_deg",
            "cam_l_px4_attitude_pitch_deg",
            "cam_l_px4_attitude_yaw_deg",
            "cam_r_px4_attitude_roll_deg",
            "cam_r_px4_attitude_pitch_deg",
            "cam_r_px4_attitude_yaw_deg",
            "cam_l_gnss_baseline_to_forward_yaw_deg",
            "cam_r_gnss_baseline_to_forward_yaw_deg",
            "common_optical_roll_error_deg",
            "relative_optical_roll_error_deg",
            "common_optical_pitch_error_deg",
            "relative_optical_pitch_error_deg",
            "common_optical_yaw_error_deg",
            "relative_optical_yaw_error_deg",
        }
        if any(param.name in geometry_parameter_names for param in params):
            return self.set_parameters_result_type(
                successful=False,
                reason="geometry-defining parameters are immutable for one run",
            )
        for param in params:
            if param.name == "gps_baseline_source":
                self.node.gps_baseline_source = self.node._validate_gps_baseline_source(param.value)
            elif param.name == "reference_distance_m":
                self.node.reference_distance_m = float(param.value)
            elif param.name == "approach_start_distance_m":
                self.node.approach_start_distance_m = float(param.value)
            elif param.name == "approach_end_distance_m":
                self.node.approach_end_distance_m = float(param.value)
            elif param.name == "approach_duration_s":
                self.node.approach_duration_s = float(param.value)
            elif param.name == "mission_profile":
                self.node.mission_profile = str(param.value)
            elif param.name == "takeoff_altitude_m":
                self.node.takeoff_altitude_m = float(param.value)
            elif param.name == "takeoff_duration_s":
                self.node.takeoff_duration_s = float(param.value)
            elif param.name == "first_hover_duration_s":
                self.node.first_hover_duration_s = float(param.value)
            elif param.name == "final_hover_duration_s":
                self.node.final_hover_duration_s = float(param.value)
            elif param.name == "land_duration_s":
                self.node.land_duration_s = float(param.value)
            elif param.name == "target_spawn_z_m":
                self.node.target_spawn_z_m = float(param.value)
            elif param.name == "fly_target_in_gazebo":
                self.node.fly_target_in_gazebo = self.node._coerce_bool(param.value)
            elif param.name == "yolo_model_path":
                self.node.yolo_model_path = str(param.value)
                self.node._reset_yolo_model()
            elif param.name == "yolo_class_name":
                self.node.yolo_class_name = str(param.value)
                self.node.yolo_class_warning_logged = False
            elif param.name == "yolo_confidence":
                self.node.yolo_confidence = float(param.value)
            elif param.name == "yolo_device":
                self.node.yolo_device = str(param.value)
                self.node.yolo_device_warning_logged = False
                self.node.yolo_cpu_fallback = False
                self.node.resolved_yolo_device = None
                self.node._reset_yolo_model()
            elif param.name == "yolo_target_hz":
                self.node.yolo_target_hz = max(0.0, float(param.value))
            elif param.name == "yolo_imgsz":
                self.node.yolo_imgsz = max(64, int(param.value))
            elif param.name == "rviz_live_latest_only":
                self.node.rviz_live_latest_only = self.node._coerce_bool(param.value)
                self.node.yolo_condition.notify_all()
            elif param.name == "rviz_live_ray_hz":
                self.node.rviz_live_ray_hz = max(0.1, float(param.value))
                self.node.yolo_condition.notify_all()
            elif param.name == "enable_synthetic_detections":
                self.node.enable_synthetic_detections = self.node._coerce_bool(param.value)
            elif param.name == "synthetic_detection_width_px":
                self.node.synthetic_detection_width_px = max(1, int(param.value))
            elif param.name == "synthetic_detection_height_px":
                self.node.synthetic_detection_height_px = max(1, int(param.value))
            elif param.name == "synthetic_detection_bbox_size_px":
                self.node.synthetic_detection_bbox_size_px = max(1.0, float(param.value))
            elif param.name == "enable_external_bbox_detections":
                self.node.enable_external_bbox_detections = self.node._coerce_bool(param.value)
            elif param.name == "external_bbox_detections_topic":
                self.node.external_bbox_detections_topic = str(param.value)
            elif param.name == "external_bbox_detection_width_px":
                self.node.external_bbox_detection_width_px = max(1, int(param.value))
            elif param.name == "external_bbox_detection_height_px":
                self.node.external_bbox_detection_height_px = max(1, int(param.value))
            elif param.name == "plot_publish_hz":
                self.node.plot_publish_hz = max(1.0, float(param.value))
            elif param.name == "enable_bbox_center_adjustment":
                self.node.enable_bbox_center_adjustment = self.node._coerce_bool(param.value)
                self.node.bbox_center_adjustment = self.node._load_bbox_center_adjustment()
            elif param.name == "bbox_center_adjustment_path":
                self.node.bbox_center_adjustment_path = str(param.value)
                self.node.bbox_center_adjustment = self.node._load_bbox_center_adjustment()
            elif param.name == "enable_observation_fusion":
                self.node.enable_observation_fusion = self.node._coerce_bool(param.value)
                self.node.observation_fusion_adapter = self.node._load_observation_fusion_adapter()
                self.node._reset_observation_fusion_buffer()
            elif param.name == "observation_fusion_manifest_path":
                self.node.observation_fusion_manifest_path = str(param.value)
                self.node.observation_fusion_adapter = self.node._load_observation_fusion_adapter()
                self.node._reset_observation_fusion_buffer()
            elif param.name == "observation_fusion_tier":
                self.node.observation_fusion_tier = str(param.value)
                self.node.observation_fusion_adapter = self.node._load_observation_fusion_adapter()
                self.node._reset_observation_fusion_buffer()
            elif param.name == "enable_disparity_residual_profile":
                self.node.enable_disparity_residual_profile = self.node._coerce_bool(param.value)
                self.node.disparity_residual_profile = self.node._load_disparity_residual_profile()
            elif param.name == "disparity_residual_profile_path":
                self.node.disparity_residual_profile_path = str(param.value)
                self.node.disparity_residual_profile = self.node._load_disparity_residual_profile()
            elif param.name == "enable_disparity_residual_feature_selector":
                self.node.enable_disparity_residual_feature_selector = self.node._coerce_bool(param.value)
                self.node.disparity_residual_feature_selector = self.node._load_disparity_residual_feature_selector()
            elif param.name == "disparity_residual_feature_selector_model_path":
                self.node.disparity_residual_feature_selector_model_path = str(param.value)
                self.node.disparity_residual_feature_selector = self.node._load_disparity_residual_feature_selector()
            elif param.name == "disparity_residual_feature_selector_catalog_path":
                self.node.disparity_residual_feature_selector_catalog_path = str(param.value)
                self.node.disparity_residual_feature_selector = self.node._load_disparity_residual_feature_selector()
            elif param.name == "disparity_residual_feature_selector_window_size":
                self.node.disparity_residual_feature_selector_window_size = max(1, int(param.value))
                self.node._reset_disparity_residual_feature_selector_window()
            elif param.name == "disparity_residual_feature_selector_min_samples":
                self.node.disparity_residual_feature_selector_min_samples = max(1, int(param.value))
        return self.set_parameters_result_type(successful=True)

    def _observation_fusion_required_observations(self, adapter):
        if isinstance(adapter, RepeatedObservationFusion):
            return max(1, int(adapter.min_observations))
        if isinstance(adapter, MultiBurstObservationFusion):
            return max(1, int(adapter.min_bursts) * int(adapter.min_observations_per_burst))
        return 1

    def _reset_observation_fusion_buffer(self):
        self.node.observation_fusion_buffer = deque(
            maxlen=self.node._observation_fusion_required_observations(self.node.observation_fusion_adapter)
        )

    def _load_observation_fusion_adapter(self):
        if not self.node.enable_observation_fusion:
            return None
        path = str(self.node.observation_fusion_manifest_path or "").strip()
        if not path:
            self.node.get_logger().warning("observation fusion enabled but no manifest path was provided.")
            return None
        if not os.path.exists(path):
            self.node.get_logger().warning(f"observation fusion manifest not found: {path}")
            return None
        camera = RuntimeStereoCameraContract(
            width_px=1280.0,
            height_px=720.0,
            horizontal_fov_rad=float(self.node.horizontal_fov_rad),
            baseline_m=float(self.node.baseline_m),
        )
        tier = str(self.node.observation_fusion_tier or "").strip() or None
        try:
            adapter, tier_payload = load_observation_fusion_adapter_from_manifest(
                path,
                tier=tier,
                camera=camera,
            )
        except Exception as exc:
            self.node.get_logger().warning(f"failed to load observation fusion manifest {path}: {exc}")
            return None
        self.node.get_logger().info(
            "Loaded observation fusion manifest: "
            f"{path} (tier={tier_payload.get('tier', tier or 'default')}, "
            f"adapter={type(adapter).__name__})"
        )
        return adapter

    def _load_disparity_residual_profile(self):
        if not getattr(self.node, "enable_disparity_residual_profile", False):
            return None
        path = str(getattr(self.node, "disparity_residual_profile_path", "") or "").strip()
        if not path:
            self.node.get_logger().warning("disparity residual profile enabled but no profile path was provided.")
            return None
        if not os.path.exists(path):
            self.node.get_logger().warning(f"disparity residual profile not found: {path}")
            return None
        try:
            profile = DisparityResidualRuntimeProfile.from_json_file(path)
        except Exception as exc:
            self.node.get_logger().warning(f"failed to load disparity residual profile {path}: {exc}")
            return None
        self.node.get_logger().info(
            "Loaded disparity residual profile: "
            f"{path} (profile_id={profile.profile_id}, method={profile.method})"
        )
        return profile

    def _load_disparity_residual_feature_selector(self):
        if not getattr(self.node, "enable_disparity_residual_feature_selector", False):
            return None
        model_path = str(getattr(self.node, "disparity_residual_feature_selector_model_path", "") or "").strip()
        catalog_path = str(getattr(self.node, "disparity_residual_feature_selector_catalog_path", "") or "").strip()
        if not model_path or not catalog_path:
            self.node.get_logger().warning("disparity residual feature selector enabled but model/catalog path is missing.")
            return None
        if not os.path.exists(model_path):
            self.node.get_logger().warning(f"disparity residual feature selector model not found: {model_path}")
            return None
        if not os.path.exists(catalog_path):
            self.node.get_logger().warning(f"disparity residual feature selector catalog not found: {catalog_path}")
            return None
        try:
            selector = DisparityResidualFeatureSelector(
                selector_model_json=model_path,
                catalog_json=catalog_path,
            )
        except Exception as exc:
            self.node.get_logger().warning(f"failed to load disparity residual feature selector: {exc}")
            return None
        self.node.get_logger().info(f"Loaded disparity residual feature selector: {model_path}")
        return selector

    def _reset_disparity_residual_feature_selector_window(self):
        self.node.disparity_residual_feature_selector_window = deque(
            maxlen=max(1, int(getattr(self.node, "disparity_residual_feature_selector_window_size", 1)))
        )

    def _load_bbox_center_adjustment(self):
        if not self.node.enable_bbox_center_adjustment:
            return None
        path = str(self.node.bbox_center_adjustment_path or "").strip()
        if not path:
            self.node.get_logger().warning("bbox center adjustment enabled but no matrix path was provided.")
            return None
        if not os.path.exists(path):
            self.node.get_logger().warning(f"bbox center adjustment matrix not found: {path}")
            return None
        if path.lower().endswith(".json"):
            return self.node._load_bbox_output_profile(path)
        try:
            matrix = np.loadtxt(path, delimiter=",", dtype=float)
        except Exception as exc:
            self.node.get_logger().warning(f"failed to load bbox center adjustment matrix {path}: {exc}")
            return None
        if matrix.shape != (6, 4):
            self.node.get_logger().warning(
                f"bbox center adjustment matrix must be 6x4, got {matrix.shape}: {path}"
            )
            return None
        self.node.get_logger().info(f"Loaded bbox center adjustment matrix: {path}")
        return matrix

    def _load_bbox_output_profile(self, path):
        try:
            with open(path, "r", encoding="utf-8") as handle:
                profile = json.load(handle)
        except Exception as exc:
            self.node.get_logger().warning(f"failed to load bbox output profile {path}: {exc}")
            return None
        mode = profile.get("mode")
        if mode == OriginBBoxReprojectionProfile.mode:
            try:
                loaded = OriginBBoxReprojectionProfile.from_json_dict(profile)
            except Exception as exc:
                self.node.get_logger().warning(f"failed to load origin bbox reprojection profile {path}: {exc}")
                return None
            self.node.get_logger().info(
                f"Loaded origin bbox reprojection profile: {path} "
                f"({len(loaded.cells)} cells, policy={loaded.lookup_policy})"
            )
            return loaded
        if mode == BoundedGainForwardCellGuardPolicy.mode:
            try:
                loaded = BoundedGainForwardCellGuardPolicy.from_json_dict(profile)
            except Exception as exc:
                self.node.get_logger().warning(f"failed to load bounded gain guard profile {path}: {exc}")
                return None
            self.node.get_logger().info(
                f"Loaded bounded gain guard profile: {path} "
                f"({len(loaded.down_cells)} Down cells, family={loaded.profile_family})"
            )
            return loaded
        if mode == EstimatedToTruthSim3Profile.mode:
            try:
                loaded = EstimatedToTruthSim3Profile.from_json_dict(profile)
            except Exception as exc:
                self.node.get_logger().warning(f"failed to load estimated-to-truth Sim(3) profile {path}: {exc}")
                return None
            self.node.get_logger().info(
                f"Loaded estimated-to-truth Sim(3) profile: {path} "
                f"(profile_id={loaded.profile_id})"
            )
            return loaded
        if mode == EstimatedToTruthDisparityProfile.mode:
            try:
                loaded = EstimatedToTruthDisparityProfile.from_json_dict(profile)
            except Exception as exc:
                self.node.get_logger().warning(f"failed to load estimated-to-truth disparity profile {path}: {exc}")
                return None
            self.node.get_logger().info(
                f"Loaded estimated-to-truth disparity profile: {path} "
                f"(profile_id={loaded.profile_id})"
            )
            return loaded
        if mode == EstimatedToTruthAffineProfile.mode:
            try:
                loaded = EstimatedToTruthAffineProfile.from_json_dict(profile)
            except Exception as exc:
                self.node.get_logger().warning(f"failed to load estimated-to-truth affine RDF profile {path}: {exc}")
                return None
            self.node.get_logger().info(
                f"Loaded estimated-to-truth affine RDF profile: {path} "
                f"(profile_id={loaded.profile_id})"
            )
            return loaded
        if mode == "origin_bbox_anchor":
            anchor = profile.get("anchor")
            if not isinstance(anchor, dict):
                self.node.get_logger().warning(f"bbox anchor profile has no anchor object: {path}")
                return None
            required = ("left_x_fraction", "right_x_fraction", "left_y_fraction", "right_y_fraction")
            if any(name not in anchor for name in required):
                self.node.get_logger().warning(f"bbox anchor profile anchor is incomplete: {path}")
                return None
            loaded = dict(profile)
            loaded["anchor"] = {name: float(anchor[name]) for name in required}
            self.node.get_logger().info(f"Loaded origin bbox anchor profile: {path}")
            return loaded
        if mode != "origin_bbox_output_ridge":
            self.node.get_logger().warning(
                f"unsupported bbox output profile mode {profile.get('mode')!r}: {path}"
            )
            return None
        feature_names = profile.get("feature_names")
        coefficients = np.array(profile.get("coefficients", []), dtype=float)
        if not isinstance(feature_names, list) or not feature_names:
            self.node.get_logger().warning(f"bbox output profile has no feature_names: {path}")
            return None
        if coefficients.shape != (3, len(feature_names)):
            self.node.get_logger().warning(
                "bbox output profile coefficients must be 3 x feature_count, "
                f"got {coefficients.shape}: {path}"
            )
            return None
        feature_mean = np.array(profile.get("feature_mean", [0.0] * len(feature_names)), dtype=float)
        feature_scale = np.array(profile.get("feature_scale", [1.0] * len(feature_names)), dtype=float)
        if feature_mean.shape != (len(feature_names),) or feature_scale.shape != (len(feature_names),):
            self.node.get_logger().warning(f"bbox output profile mean/scale length mismatch: {path}")
            return None
        feature_scale = np.where(np.abs(feature_scale) < 1.0e-12, 1.0, feature_scale)
        blend = np.array(profile.get("blend", [1.0, 1.0, 1.0]), dtype=float)
        if blend.shape != (3,):
            self.node.get_logger().warning(f"bbox output profile blend must have 3 values: {path}")
            return None
        loaded = dict(profile)
        loaded["coefficients"] = coefficients
        loaded["feature_mean"] = feature_mean
        loaded["feature_scale"] = feature_scale
        loaded["blend"] = blend
        self.node.get_logger().info(
            f"Loaded origin bbox output profile: {path} ({len(feature_names)} features)"
        )
        return loaded
