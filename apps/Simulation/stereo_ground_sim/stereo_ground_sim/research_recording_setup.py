"""Research recording configuration and immutable run metadata."""

import hashlib
import json
import math
import os
from pathlib import Path

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME
from stereo_ground_sim.coordinate_frames import (
    PX4_BODY_FRD_FROM_OPTICAL_RDF,
    VEHICLE_REFERENCE_LINK,
)
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
from stereo_ground_sim.research_recording import (
    RESEARCH_SCHEMA_VERSION,
    ResearchRecorder,
    discover_repository_root,
    validate_recording_configuration,
)

RESEARCH_RECORDING_SETUP_METHODS = [
    "_recording_parameter_names",
    "_research_run_metadata",
    "_initialize_research_recording",
    "_bool_param",
    "_coerce_bool"
]


class ResearchRecordingSetup:
    def __init__(self, node, *, gps_baseline_anchor_policy):
        self.node = node
        self.gps_baseline_anchor_policy = gps_baseline_anchor_policy

    @staticmethod
    def _recording_parameter_names():
        return {
            "enable_research_recording",
            "research_output_dir",
            "research_run_id",
            "research_session_id",
            "research_attempt_id",
            "research_case_id",
            "research_effective_config_path",
            "research_effective_config_sha256",
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
            "px4_attitude_left_topic",
            "px4_attitude_right_topic",
            "px4_attitude_max_age_s",
            "px4_attitude_pair_skew_s",
        }

    def _research_run_metadata(self):
        observation = self.node._observation_metadata()
        intrinsics = self.node._camera_intrinsics(
            self.node.camera_image_width_px, self.node.camera_image_height_px
        )
        left_origin_frd = self.node.baseline_center_frd.point_from_world(
            self.node.nominal_left_camera.center_world_enu_m
        )
        right_origin_frd = self.node.baseline_center_frd.point_from_world(
            self.node.nominal_right_camera.center_world_enu_m
        )
        intrinsic_contract = {
            "image_width_px": int(self.node.camera_image_width_px),
            "image_height_px": int(self.node.camera_image_height_px),
            "horizontal_fov_rad": float(self.node.horizontal_fov_rad),
            "fx_px": float(intrinsics.fx_px),
            "fy_px": float(intrinsics.fy_px),
            "cx_px": float(intrinsics.cx_px),
            "cy_px": float(intrinsics.cy_px),
            "skew_px": 0.0,
            "distortion_model": "none",
            "distortion_coefficients": [],
        }
        intrinsic_profile_sha256 = hashlib.sha256(
            json.dumps(
                intrinsic_contract,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        baseline_vector_frd = [
            right_origin_frd[index] - left_origin_frd[index] for index in range(3)
        ]
        baseline_length_m = math.sqrt(sum(value * value for value in baseline_vector_frd))
        rig_contract = {
            "left_nominal_origin_baseline_center_frd_m": list(left_origin_frd),
            "right_nominal_origin_baseline_center_frd_m": list(right_origin_frd),
            "baseline_vector_baseline_center_frd_m": baseline_vector_frd,
            "baseline_length_m": baseline_length_m,
        }
        rig_profile_sha256 = hashlib.sha256(
            json.dumps(
                rig_contract,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        desired_camera_poses = {
            label: {
                "center_world_enu_m": list(camera.center_world_enu_m),
                "right_world_enu": list(camera.right_world_enu),
                "down_world_enu": list(camera.down_world_enu),
                "forward_world_enu": list(camera.forward_world_enu),
            }
            for label, camera in (
                ("left", self.node.desired_left_camera),
                ("right", self.node.desired_right_camera),
            )
        }
        attitude = {
            name: getattr(self.node, name) for name in self.node._attitude_parameter_names()
        }
        attitude.update(
            {
                "effective_left_optical_roll_error_deg": self.node.effective_left_optical_roll_error_deg,
                "effective_left_optical_pitch_error_deg": self.node.effective_left_optical_pitch_error_deg,
                "effective_left_optical_yaw_error_deg": self.node.effective_left_optical_yaw_error_deg,
                "effective_right_optical_roll_error_deg": self.node.effective_right_optical_roll_error_deg,
                "effective_right_optical_pitch_error_deg": self.node.effective_right_optical_pitch_error_deg,
                "effective_right_optical_yaw_error_deg": self.node.effective_right_optical_yaw_error_deg,
                "attitude_error_frame": self.node.attitude_error_frame,
                "attitude_angle_unit": self.node.attitude_angle_unit,
                "attitude_parameterization": self.node.attitude_parameterization,
                "attitude_rotation_composition": self.node.attitude_rotation_composition,
            }
        )
        return {
            "schema_version": RESEARCH_SCHEMA_VERSION,
            "recorder_version": RESEARCH_SCHEMA_VERSION,
            "session_id": self.node.research_session_id,
            "run_id": self.node.research_run_id,
            "attempt_id": self.node.research_attempt_id,
            "case_id": self.node.research_case_id,
            "effective_config_path": self.node.research_effective_config_path,
            "effective_config_sha256": self.node.research_effective_config_sha256,
            "repository": self.node.research_repository,
            "branch": self.node.research_branch,
            "commit_sha": self.node.research_commit_sha,
            "git_status": self.node.research_git_status,
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "default"),
            "gazebo_version": self.node.research_gazebo_version,
            "px4_version": self.node.research_px4_version,
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
            "baseline_m": self.node.baseline_m,
            "camera_height_m": self.node.camera_height_m,
            "horizontal_fov_rad": self.node.horizontal_fov_rad,
            "image_width_px": self.node.camera_image_width_px,
            "image_height_px": self.node.camera_image_height_px,
            "camera_update_rate_hz": self.node.camera_update_rate_hz,
            "intrinsic_profile_id": "simulation_pinhole_from_horizontal_fov",
            "intrinsic_profile_sha256": intrinsic_profile_sha256,
            "fx_px": intrinsics.fx_px,
            "fy_px": intrinsics.fy_px,
            "cx_px": intrinsics.cx_px,
            "cy_px": intrinsics.cy_px,
            "skew_px": 0.0,
            "distortion_model": "none",
            "distortion_coefficients": [],
            "pixel_origin": "top_left",
            "u_axis_direction": "right",
            "v_axis_direction": "down",
            "distorted_or_undistorted": "undistorted_pinhole",
            "crop_policy": "none",
            "resize_policy": "none",
            "letterbox_policy": "none",
            "rig_profile_id": "simulation_symmetric_baseline",
            "rig_profile_sha256": rig_profile_sha256,
            **rig_contract,
            "camera_oracle_source": "gazebo_observed_optical",
            "camera_oracle_expected_sdf_sha256": self.node.camera_oracle_generated_sdf_sha256,
            "camera_oracle_max_age_s": self.node.camera_oracle_max_age_s,
            "camera_oracle_topics": {
                "left": self.node.camera_oracle_left_topic,
                "right": self.node.camera_oracle_right_topic,
            },
            "px4_attitude_topics": {
                "left": self.node.px4_attitude_left_topic,
                "right": self.node.px4_attitude_right_topic,
            },
            "px4_attitude_max_age_s": self.node.px4_attitude_max_age_s,
            "px4_attitude_pair_skew_s": self.node.px4_attitude_pair_skew_s,
            "gps_baseline_max_age_s": self.node.gps_baseline_max_age_s,
            "operational_pose_timestamp_source": (
                "/clock values observed when PX4 attitude/GPS samples were received"
            ),
            "operational_pose_alignment_method": (
                "latest_valid_px4_sensor_snapshot_at_image_pair_acceptance"
            ),
            "px4_source_timestamp_fields": [
                "left_attitude_timestamp_us",
                "right_attitude_timestamp_us",
                "left_gps_timestamp_us",
                "right_gps_timestamp_us",
            ],
            "operational_simulation_timestamp_fields": [
                "left_operational_attitude_timestamp_ns",
                "right_operational_attitude_timestamp_ns",
                "left_operational_position_timestamp_ns",
                "right_operational_position_timestamp_ns",
            ],
            "px4_quaternion_order": "wxyz",
            "px4_attitude_semantics": "body_frd_to_earth_ned",
            "px4_attitude_model_version": PX4_ATTITUDE_MODEL_VERSION,
            "px4_attitude_reference_source": PX4_ATTITUDE_REFERENCE_SOURCE,
            "px4_attitude_frame": PX4_ATTITUDE_FRAME,
            "px4_attitude_composition": PX4_ATTITUDE_COMPOSITION,
            "legacy_px4_absolute_attitude_used": True,
            "left_R_body_from_optical": [
                list(row) for row in PX4_BODY_FRD_FROM_OPTICAL_RDF
            ],
            "right_R_body_from_optical": [
                list(row) for row in PX4_BODY_FRD_FROM_OPTICAL_RDF
            ],
            "extrinsic_matrix_storage_order": "row_major",
            "extrinsic_source_type": PX4_CAMERA_EXTRINSIC_SOURCE_TYPE,
            "extrinsic_source_file": PX4_CAMERA_EXTRINSIC_SOURCE_FILE,
            "extrinsic_source_parent_link": PX4_CAMERA_EXTRINSIC_PARENT_LINK,
            "extrinsic_source_child_link": PX4_CAMERA_EXTRINSIC_CHILD_LINK,
            "extrinsic_validation_orthogonality_error": self.node.px4_body_from_optical_validation[
                "orthogonality_error"
            ],
            "extrinsic_validation_determinant_error": self.node.px4_body_from_optical_validation[
                "determinant_error"
            ],
            "extrinsic_validation_gazebo_angle_error_deg": 0.0,
            "diagnostic_ray_provenance": {
                "attitude": {
                    "pixel_source": "observed",
                    "attitude_source": "px4_vehicle_attitude",
                    "camera_center_source": "gazebo_observed_optical_centers",
                },
                "gps_baseline": {
                    "pixel_source": "observed",
                    "attitude_source": "gazebo_observed_optical_pose",
                    "baseline_source": "px4_raw_sensor_gps",
                },
                "operational": {
                    "pixel_source": "observed",
                    "attitude_source": "px4_vehicle_attitude",
                    "baseline_source": "px4_raw_sensor_gps",
                },
            },
            "baseline_anchor_policy": self.gps_baseline_anchor_policy,
            "desired_camera_poses_from_env": desired_camera_poses,
            **attitude,
            "scenario_name": self.node.scenario_name,
            "target_class": self.node.target_class,
            "target_size_m": self.node._finite_or_none(self.node.target_size_m),
            "mission_profile": self.node.mission_profile,
            "trajectory_id": self.node.mission_profile,
            "target_model_name": getattr(self.node, "target_model_name", "unspecified"),
            "ground_truth_source": "gazebo_vehicle_reference_link_world_pose",
            "ground_truth_model_name": self.node.target_model_name,
            "ground_truth_link_name": VEHICLE_REFERENCE_LINK,
            "ground_truth_fallback_allowed": False,
            "target_spawn_z_m": self.node.target_spawn_z_m,
            "scenario_range_min_m": self.node._finite_or_none(self.node.scenario_range_min_m),
            "scenario_range_max_m": self.node._finite_or_none(self.node.scenario_range_max_m),
            "approach_start_distance_m": self.node.approach_start_distance_m,
            "approach_end_distance_m": self.node.approach_end_distance_m,
            "zoom_index": self.node._finite_or_none(self.node.zoom_index),
            "measurement_queue_capacity": self.node.measurement_queue_capacity,
            "research_record_queue_capacity": self.node.research_record_queue_capacity,
            "strict_measurement_overrun_policy": True,
            "recorder_queue_policy": "invalidate_run_no_silent_drop",
            "yolo_model_status": (
                "not_applicable"
                if self.node.observation_mode == "ideal_reference_point"
                else "configured" if self.node.yolo_model_path.strip() else "not_configured"
            ),
        }

    def _initialize_research_recording(self):
        if not self.node.enable_research_recording:
            return
        if self.node.camera_image_width_px <= 0 or self.node.camera_image_height_px <= 0:
            raise ValueError("recording camera image dimensions must be positive")
        if not math.isfinite(self.node.camera_update_rate_hz) or self.node.camera_update_rate_hz <= 0.0:
            raise ValueError("recording camera update rate must be positive and finite")
        repository_root = discover_repository_root(Path.cwd())
        output_dir = validate_recording_configuration(
            output_dir=self.node.research_output_dir,
            run_id=self.node.research_run_id,
            case_id=self.node.research_case_id,
            repository=self.node.research_repository,
            branch=self.node.research_branch,
            commit_sha=self.node.research_commit_sha,
            git_status=self.node.research_git_status,
            gazebo_version=self.node.research_gazebo_version,
            px4_version=self.node.research_px4_version,
            queue_capacity=self.node.research_record_queue_capacity,
            repository_root=repository_root,
        )
        self.node.research_recorder = ResearchRecorder(
            output_dir=output_dir,
            run_metadata=self.node._research_run_metadata(),
            queue_capacity=self.node.research_record_queue_capacity,
        )
        self.node.research_terminal_coordinator.attach(self.node.research_recorder)

    def _bool_param(self, name):
        return self.node._coerce_bool(self.node.get_parameter(name).value)

    def _coerce_bool(self, value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)
