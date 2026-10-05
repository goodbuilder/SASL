import hashlib
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from stereo_ground_sim.coordinate_frames import (
    apply_optical_rpy_error,
    apply_px4_attitude_rpy,
    effective_optical_rpy_errors_deg,
    nominal_camera_poses,
    relative_sdf_rpy,
    sdf_link_rpy_from_optical_error,
    sdf_pose_rpy_from_camera_pose,
)
from stereo_ground_sim.experiment_context import camera_pose_quaternion_xyzw


def _float_config(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _float_config_default(context, name, default):
    return float(LaunchConfiguration(name, default=str(default)).perform(context))


def _int_config(context, name):
    return int(LaunchConfiguration(name).perform(context))


def _bool_config(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ("1", "true", "yes", "on")


def _bool_config_default(context, name, default):
    return LaunchConfiguration(name, default="true" if default else "false").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def _camera_axis_values(context, camera, sensor, field, unit, default):
    return tuple(
        _float_config_default(
            context, f"cam_{camera}_{sensor}_{field}_{axis}_{unit}", default
        )
        for axis in ("x", "y", "z")
    )


def _set_camera_sensor_noise(world_xml, camera_configs):
    root = ET.fromstring(world_xml)
    for model_name, config in camera_configs.items():
        models = [
            model
            for model in root.findall("./world/model")
            if model.attrib.get("name") == model_name
        ]
        if len(models) != 1:
            raise ValueError(f"expected one camera model named {model_name!r}")
        base = models[0].find("./link[@name='base_link']")
        if base is None:
            raise ValueError(f"camera model {model_name!r} has no base_link")
        for sensor, axis_path in (
            ("mag", "./sensor[@name='magnetometer_sensor']/magnetometer"),
            ("gyro", "./sensor[@name='imu_sensor']/imu/angular_velocity"),
        ):
            parent = base.find(axis_path)
            if parent is None:
                raise ValueError(f"camera model {model_name!r} has no {sensor} sensor")
            for index, axis in enumerate(("x", "y", "z")):
                noise = parent.find(f"{axis}/noise")
                if noise is None:
                    raise ValueError(
                        f"camera model {model_name!r} {sensor} {axis} has no noise element"
                    )
                for field in ("stddev", "bias_mean", "bias_stddev"):
                    element = noise.find(field)
                    if element is None:
                        element = ET.SubElement(noise, field)
                    element.text = f"{config[sensor][field][index]:.12g}"
    return ET.tostring(root, encoding="unicode")


def _set_target_include_static(world_xml, *, target_model_name, static_enabled):
    """Point the world's sole research-target include at target_model_name and
    set its Gazebo static flag.

    The world file has exactly one world-level ``<include>`` (the research
    target; the stereo cameras are inline ``<model>`` blocks, not includes),
    so that include is selected structurally and its ``<uri>`` is rewritten
    to match ``target_model_name`` rather than requiring it to already match.
    """
    root = ET.fromstring(world_xml)
    expected_uri = f"model://{target_model_name}"
    matches = root.findall("./world/include")
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one world-level research target include, found {len(matches)}"
        )
    uri_fields = matches[0].findall("uri")
    if len(uri_fields) != 1:
        raise ValueError("target include must contain exactly one uri field")
    uri_fields[0].text = expected_uri
    static_fields = matches[0].findall("static")
    if len(static_fields) != 1:
        raise ValueError(
            f"target include {expected_uri} must contain exactly one static field"
        )
    current = (static_fields[0].text or "").strip().lower()
    if current not in {"true", "false"}:
        raise ValueError(f"target include {expected_uri} has invalid static value: {current}")
    requested = "true" if static_enabled else "false"
    static_fields[0].text = requested
    generated = ET.tostring(root, encoding="unicode")
    verified = ET.fromstring(generated)
    verified_matches = [
        include
        for include in verified.findall("./world/include")
        if (include.findtext("uri") or "").strip() == expected_uri
    ]
    if len(verified_matches) != 1 or (verified_matches[0].findtext("static") or "").strip().lower() != requested:
        raise ValueError("generated target static contract verification failed")
    return generated


def _param_yaml_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    return json.dumps(str(value), ensure_ascii=False)


def _write_distance_node_params(context, *, camera_oracle_generated_sdf_sha256=""):
    float_params = {
        "baseline_m",
        "camera_height_m",
        "horizontal_fov_rad",
        "approach_start_distance_m",
        "approach_end_distance_m",
        "approach_duration_s",
        "takeoff_altitude_m",
        "takeoff_duration_s",
        "first_hover_duration_s",
        "final_hover_duration_s",
        "land_duration_s",
        "target_spawn_x_m",
        "target_spawn_y_m",
        "target_spawn_z_m",
        "target_spawn_yaw_deg",
        "yolo_confidence",
        "yolo_target_hz",
        "rviz_live_ray_hz",
        "synthetic_detection_bbox_size_px",
        "plot_publish_hz",
        "target_size_m",
        "scenario_range_min_m",
        "scenario_range_max_m",
        "zoom_index",
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
        "camera_update_rate_hz",
        "gps_baseline_max_age_s",
        "px4_attitude_max_age_s",
        "px4_attitude_pair_skew_s",
    }
    int_params = {
        "yolo_imgsz",
        "synthetic_detection_width_px",
        "synthetic_detection_height_px",
        "external_bbox_detection_width_px",
        "external_bbox_detection_height_px",
        "disparity_residual_feature_selector_window_size",
        "disparity_residual_feature_selector_min_samples",
        "measurement_queue_capacity",
        "research_record_queue_capacity",
        "camera_image_width_px",
        "camera_image_height_px",
    }
    bool_params = {
        "fly_target_in_gazebo",
        "enable_synthetic_detections",
        "enable_external_bbox_detections",
        "enable_bbox_center_adjustment",
        "enable_observation_fusion",
        "enable_disparity_residual_profile",
        "enable_disparity_residual_feature_selector",
        "enable_research_recording",
        "enable_gps_baseline_triangulation",
        "enable_rviz_rays",
        "rviz_live_latest_only",
        "measurement_latest_only",
        "yolo_allow_cpu_fallback",
    }
    names = [
        "scenario_name",
        "target_class",
        "target_size_m",
        "scenario_range_min_m",
        "scenario_range_max_m",
        "zoom_index",
        "baseline_m",
        "camera_height_m",
        "horizontal_fov_rad",
        "approach_start_distance_m",
        "approach_end_distance_m",
        "approach_duration_s",
        "mission_profile",
        "takeoff_altitude_m",
        "takeoff_duration_s",
        "first_hover_duration_s",
        "final_hover_duration_s",
        "land_duration_s",
        "target_spawn_x_m",
        "target_spawn_y_m",
        "target_spawn_z_m",
        "target_spawn_yaw_deg",
        "fly_target_in_gazebo",
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
        "camera_image_width_px",
        "camera_image_height_px",
        "camera_update_rate_hz",
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
        "yolo_model_path",
        "yolo_class_name",
        "yolo_confidence",
        "yolo_device",
        "yolo_allow_cpu_fallback",
        "yolo_target_hz",
        "measurement_queue_capacity",
        "measurement_latest_only",
        "rviz_live_latest_only",
        "rviz_live_ray_hz",
        "yolo_imgsz",
        "observation_mode",
        "enable_synthetic_detections",
        "synthetic_detection_width_px",
        "synthetic_detection_height_px",
        "synthetic_detection_bbox_size_px",
        "enable_external_bbox_detections",
        "external_bbox_detections_topic",
        "external_bbox_detection_width_px",
        "external_bbox_detection_height_px",
        "plot_publish_hz",
        "enable_bbox_center_adjustment",
        "bbox_center_adjustment_path",
        "enable_observation_fusion",
        "observation_fusion_manifest_path",
        "observation_fusion_tier",
        "enable_disparity_residual_profile",
        "disparity_residual_profile_path",
        "enable_disparity_residual_feature_selector",
        "disparity_residual_feature_selector_model_path",
        "disparity_residual_feature_selector_catalog_path",
        "disparity_residual_feature_selector_window_size",
        "disparity_residual_feature_selector_min_samples",
        "enable_gps_baseline_triangulation",
        "gps_baseline_source",
        "gps_baseline_left_topic",
        "gps_baseline_right_topic",
        "global_baseline_left_topic",
        "global_baseline_right_topic",
        "gps_baseline_max_age_s",
        "px4_attitude_left_topic",
        "px4_attitude_right_topic",
        "px4_attitude_max_age_s",
        "px4_attitude_pair_skew_s",
        "enable_rviz_rays",
    ]

    params = {}
    for name in names:
        raw = LaunchConfiguration(name).perform(context)
        if name in float_params:
            params[name] = float(raw)
        elif name in int_params:
            params[name] = int(raw)
        elif name in bool_params:
            params[name] = raw.strip().lower() in ("1", "true", "yes", "on")
        else:
            params[name] = raw
    params["camera_oracle_generated_sdf_sha256"] = str(camera_oracle_generated_sdf_sha256)

    descriptor, params_file = tempfile.mkstemp(
        prefix="stereo_distance_node_params_",
        suffix=".yaml",
        text=True,
    )
    with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
        handle.write("/stereo_distance_node:\n")
        handle.write("  ros__parameters:\n")
        for name, value in params.items():
            handle.write(f"    {name}: {_param_yaml_value(value)}\n")
    return params_file


def _write_overlay_node_params(context):
    descriptor, params_file = tempfile.mkstemp(
        prefix="detection_overlay_node_params_",
        suffix=".yaml",
        text=True,
    )
    overlay_publish_hz = float(LaunchConfiguration("overlay_publish_hz").perform(context))
    with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
        handle.write("/detection_overlay_node:\n")
        handle.write("  ros__parameters:\n")
        handle.write(f"    overlay_publish_hz: {overlay_publish_hz}\n")
    return params_file


def _generate_world(context):
    share_dir = get_package_share_directory("stereo_ground_sim")
    source_world = os.path.join(share_dir, "worlds", "stereo_ground_drone.sdf")
    with open(source_world, "r", encoding="utf-8") as handle:
        world = handle.read()

    baseline_m = _float_config(context, "baseline_m")
    camera_height_m = _float_config(context, "camera_height_m")
    horizontal_fov_rad = _float_config(context, "horizontal_fov_rad")
    camera_image_width_px = _int_config(context, "camera_image_width_px")
    camera_image_height_px = _int_config(context, "camera_image_height_px")
    camera_update_rate_hz = _float_config(context, "camera_update_rate_hz")
    camera_clip_near_m = _float_config(context, "camera_clip_near_m")
    camera_clip_far_m = _float_config(context, "camera_clip_far_m")
    camera_sensor_noise = {}
    for camera, model_name in (
        ("l", "left_stereo_camera"),
        ("r", "right_stereo_camera"),
    ):
        camera_sensor_noise[model_name] = {
            "mag": {
                "stddev": _camera_axis_values(context, camera, "mag", "noise", "gauss", 0.0),
                "bias_mean": _camera_axis_values(context, camera, "mag", "bias_mean", "gauss", 0.0),
                "bias_stddev": _camera_axis_values(context, camera, "mag", "bias_stddev", "gauss", 0.0),
            },
            "gyro": {
                "stddev": _camera_axis_values(context, camera, "gyro", "noise", "rad_s", 0.0008726646),
                "bias_mean": _camera_axis_values(context, camera, "gyro", "bias_mean", "rad_s", 0.0),
                "bias_stddev": _camera_axis_values(context, camera, "gyro", "bias_stddev", "rad_s", 0.0),
            },
        }
    all_sensor_values = [
        value
        for config in camera_sensor_noise.values()
        for sensor in config.values()
        for values in sensor.values()
        for value in values
    ]
    if not all(math.isfinite(value) for value in all_sensor_values):
        raise ValueError("camera magnetometer and gyroscope noise/bias values must be finite")
    sensor_stddevs = [
        value
        for config in camera_sensor_noise.values()
        for sensor in config.values()
        for field in ("stddev", "bias_stddev")
        for value in sensor[field]
    ]
    if any(value < 0.0 for value in sensor_stddevs):
        raise ValueError("camera magnetometer and gyroscope standard deviations must be non-negative")
    approach_start_distance_m = _float_config(context, "approach_start_distance_m")
    target_spawn_x_m = _float_config(context, "target_spawn_x_m")
    target_spawn_y_m = _float_config(context, "target_spawn_y_m")
    target_spawn_z_m = _float_config(context, "target_spawn_z_m")
    target_spawn_yaw_deg = _float_config(context, "target_spawn_yaw_deg")
    target_model_name = LaunchConfiguration("target_model_name", default="x500_15m").perform(context)
    target_model_sdf = os.path.join(share_dir, "models", target_model_name, "model.sdf")
    if not os.path.isfile(target_model_sdf):
        raise ValueError(
            f"target_model_name {target_model_name!r} has no model.sdf at {target_model_sdf}; "
            "add the model under stereo_ground_sim/models/ before selecting it"
        )
    target_static_for_research = _bool_config_default(
        context, "target_static_for_research", False
    )
    left_effective_rpy_deg, right_effective_rpy_deg = effective_optical_rpy_errors_deg(
        (
            _float_config(context, "cam_l_optical_roll_deg"),
            _float_config(context, "cam_l_optical_pitch_deg"),
            _float_config(context, "cam_l_optical_yaw_deg"),
        ),
        (
            _float_config(context, "cam_r_optical_roll_deg"),
            _float_config(context, "cam_r_optical_pitch_deg"),
            _float_config(context, "cam_r_optical_yaw_deg"),
        ),
        (
            _float_config(context, "common_optical_roll_error_deg"),
            _float_config(context, "common_optical_pitch_error_deg"),
            _float_config(context, "common_optical_yaw_error_deg"),
        ),
        (
            _float_config(context, "relative_optical_roll_error_deg"),
            _float_config(context, "relative_optical_pitch_error_deg"),
            _float_config(context, "relative_optical_yaw_error_deg"),
        ),
    )
    nominal_link_yaw = math.pi / 2.0
    left_optical_world_rpy = sdf_link_rpy_from_optical_error(
        nominal_link_yaw,
        *(math.radians(value) for value in left_effective_rpy_deg),
    )
    right_optical_world_rpy = sdf_link_rpy_from_optical_error(
        nominal_link_yaw,
        *(math.radians(value) for value in right_effective_rpy_deg),
    )
    # PX4 body attitude is configured independently from the camera optical
    # pose.  The PX4 RPY values are applied to the canonical rig heading, while
    # optical_link is positioned relative to that body pose so the optical
    # sensor keeps the separately configured CAM_*_OPTICAL_* world attitude.
    nominal_left_camera, nominal_right_camera = nominal_camera_poses(baseline_m, camera_height_m)
    left_true_optical_pose = apply_optical_rpy_error(
        nominal_left_camera,
        *(math.radians(value) for value in left_effective_rpy_deg),
    )
    right_true_optical_pose = apply_optical_rpy_error(
        nominal_right_camera,
        *(math.radians(value) for value in right_effective_rpy_deg),
    )
    left_px4_reported_pose = apply_px4_attitude_rpy(
        nominal_left_camera,
        *(
            math.radians(_float_config(context, name))
            for name in (
                "cam_l_px4_attitude_roll_deg",
                "cam_l_px4_attitude_pitch_deg",
                "cam_l_px4_attitude_yaw_deg",
            )
        ),
    )
    right_px4_reported_pose = apply_px4_attitude_rpy(
        nominal_right_camera,
        *(
            math.radians(_float_config(context, name))
            for name in (
                "cam_r_px4_attitude_roll_deg",
                "cam_r_px4_attitude_pitch_deg",
                "cam_r_px4_attitude_yaw_deg",
            )
        ),
    )
    left_px4_world_rpy = sdf_pose_rpy_from_camera_pose(left_px4_reported_pose)
    right_px4_world_rpy = sdf_pose_rpy_from_camera_pose(right_px4_reported_pose)
    left_optical_relative_rpy = relative_sdf_rpy(
        left_px4_world_rpy,
        left_optical_world_rpy,
    )
    right_optical_relative_rpy = relative_sdf_rpy(
        right_px4_world_rpy,
        right_optical_world_rpy,
    )
    left_pose = (
        f"{-baseline_m / 2.0:.6f} 0 {camera_height_m:.6f} "
        f"{left_px4_world_rpy[0]:.9f} {left_px4_world_rpy[1]:.9f} {left_px4_world_rpy[2]:.9f}"
    )
    right_pose = (
        f"{baseline_m / 2.0:.6f} 0 {camera_height_m:.6f} "
        f"{right_px4_world_rpy[0]:.9f} {right_px4_world_rpy[1]:.9f} {right_px4_world_rpy[2]:.9f}"
    )
    left_optical_pose = "0 0 0 " + " ".join(f"{value:.9f}" for value in left_optical_relative_rpy)
    right_optical_pose = "0 0 0 " + " ".join(f"{value:.9f}" for value in right_optical_relative_rpy)

    world = world.replace("<pose>-12.5 0 5.0 0 0 1.570796327</pose>", f"<pose>{left_pose}</pose>", 1)
    world = world.replace("<pose>12.5 0 5.0 0 0 1.570796327</pose>", f"<pose>{right_pose}</pose>", 1)
    world = world.replace(
        '<pose relative_to="base_link">0 0 0 0 0 0</pose>',
        f'<pose relative_to="base_link">{left_optical_pose}</pose>',
        1,
    )
    world = world.replace(
        '<pose relative_to="base_link">0 0 0 0 0 0</pose>',
        f'<pose relative_to="base_link">{right_optical_pose}</pose>',
        1,
    )
    for fov_value in ("1.0471975512", "0.154997710698"):
        world = world.replace(
            f"<horizontal_fov>{fov_value}</horizontal_fov>",
            f"<horizontal_fov>{horizontal_fov_rad:.12f}</horizontal_fov>",
        )
    world = world.replace("<update_rate>20</update_rate>", f"<update_rate>{camera_update_rate_hz:.6f}</update_rate>")
    world = world.replace("<width>1280</width>", f"<width>{camera_image_width_px}</width>")
    world = world.replace("<height>720</height>", f"<height>{camera_image_height_px}</height>")
    world = world.replace("<near>0.1</near>", f"<near>{camera_clip_near_m:.6f}</near>")
    world = world.replace("<far>2500</far>", f"<far>{camera_clip_far_m:.6f}</far>")
    world = _set_camera_sensor_noise(world, camera_sensor_noise)
    for default_target_pose in (
        "<pose>2000 0 0.75 0 0 3.141592654</pose>",
        "<pose>2000 0 0.25 0 0 3.141592654</pose>",
        "<pose>2000 0 5 0 0 0</pose>",
        "<pose>0 2000 0.75 0 0 -1.570796327</pose>",
    ):
        if default_target_pose in world:
            world = world.replace(
                default_target_pose,
                (
                    f"<pose>{target_spawn_x_m:.6f} {target_spawn_y_m:.6f} "
                    f"{target_spawn_z_m:.6f} 0 0 {math.radians(target_spawn_yaw_deg):.9f}</pose>"
                ),
                1,
            )
            break

    # Point the magnetometer plugin at a local build whose geomagnetic table
    # is replaced with PX4's own WMM table (see
    # stereo_ground_sim/vendor/gz_magnetometer_wmm_patch/), so the simulated
    # magnetometer signal agrees with PX4's independently-computed
    # EKF2_MAG_DECL for this world's lat/lon. Falls back to gz-sim's stock,
    # installed plugin if the patched build isn't available.
    magnetometer_plugin_path = os.environ.get(
        "STEREO_GROUND_SIM_MAGNETOMETER_PLUGIN_PATH")
    if magnetometer_plugin_path and os.path.isfile(magnetometer_plugin_path):
        stock_filename = 'filename="gz-sim-magnetometer-system"'
        if world.count(stock_filename) != 1:
            raise ValueError(
                "generated world must contain exactly one stock magnetometer plugin filename"
            )
        world = world.replace(
            stock_filename,
            f'filename="{magnetometer_plugin_path}"',
            1,
        )

    world = world.replace("        <!-- GROUND_GRID_VISUALS -->", "", 1)
    world = _set_target_include_static(
        world,
        target_model_name=target_model_name,
        static_enabled=target_static_for_research,
    )

    descriptor, generated = tempfile.mkstemp(
        prefix="stereo_ground_drone_attitude_error_",
        suffix=".sdf",
        text=True,
    )
    with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
        handle.write(world)
    return generated


def _camera_pose_diagnostic_parameters(context, generated_world):
    """Freeze desired optical poses beside the exact generated SDF hash."""

    baseline_m = _float_config(context, "baseline_m")
    camera_height_m = _float_config(context, "camera_height_m")
    left_effective_rpy_deg, right_effective_rpy_deg = effective_optical_rpy_errors_deg(
        (
            _float_config(context, "cam_l_optical_roll_deg"),
            _float_config(context, "cam_l_optical_pitch_deg"),
            _float_config(context, "cam_l_optical_yaw_deg"),
        ),
        (
            _float_config(context, "cam_r_optical_roll_deg"),
            _float_config(context, "cam_r_optical_pitch_deg"),
            _float_config(context, "cam_r_optical_yaw_deg"),
        ),
        (
            _float_config(context, "common_optical_roll_error_deg"),
            _float_config(context, "common_optical_pitch_error_deg"),
            _float_config(context, "common_optical_yaw_error_deg"),
        ),
        (
            _float_config(context, "relative_optical_roll_error_deg"),
            _float_config(context, "relative_optical_pitch_error_deg"),
            _float_config(context, "relative_optical_yaw_error_deg"),
        ),
    )
    nominal_link_yaw = math.pi / 2.0
    optical_rpys = (
        sdf_link_rpy_from_optical_error(
            nominal_link_yaw, *(math.radians(value) for value in left_effective_rpy_deg)
        ),
        sdf_link_rpy_from_optical_error(
            nominal_link_yaw, *(math.radians(value) for value in right_effective_rpy_deg)
        ),
    )
    with open(generated_world, "rb") as handle:
        generated_sdf_sha256 = hashlib.sha256(handle.read()).hexdigest()
    return (
        {
            "camera_label": "left",
            "parent_model_name": "left_stereo_camera",
            "output_topic": "/simulation/camera_pose_diagnostics/left",
            "desired_optical_position_world_enu_m": [-baseline_m / 2.0, 0.0, camera_height_m],
            "desired_optical_rpy_world_rad": list(optical_rpys[0]),
            "generated_sdf_sha256": generated_sdf_sha256,
        },
        {
            "camera_label": "right",
            "parent_model_name": "right_stereo_camera",
            "output_topic": "/simulation/camera_pose_diagnostics/right",
            "desired_optical_position_world_enu_m": [baseline_m / 2.0, 0.0, camera_height_m],
            "desired_optical_rpy_world_rad": list(optical_rpys[1]),
            "generated_sdf_sha256": generated_sdf_sha256,
        },
    )


def _experiment_context_parameters(context, generated_sdf_sha256):
    """Resolve final Simulation values for the immutable context publisher."""

    baseline_m = _float_config(context, "baseline_m")
    camera_height_m = _float_config(context, "camera_height_m")
    nominal_left, nominal_right = nominal_camera_poses(baseline_m, camera_height_m)
    direct_left = (
        _float_config(context, "cam_l_optical_roll_deg"),
        _float_config(context, "cam_l_optical_pitch_deg"),
        _float_config(context, "cam_l_optical_yaw_deg"),
    )
    direct_right = (
        _float_config(context, "cam_r_optical_roll_deg"),
        _float_config(context, "cam_r_optical_pitch_deg"),
        _float_config(context, "cam_r_optical_yaw_deg"),
    )
    common = (
        _float_config(context, "common_optical_roll_error_deg"),
        _float_config(context, "common_optical_pitch_error_deg"),
        _float_config(context, "common_optical_yaw_error_deg"),
    )
    relative = (
        _float_config(context, "relative_optical_roll_error_deg"),
        _float_config(context, "relative_optical_pitch_error_deg"),
        _float_config(context, "relative_optical_yaw_error_deg"),
    )
    zoom_raw = LaunchConfiguration("zoom_index").perform(context).strip()
    try:
        zoom_index = float(zoom_raw)
        zoom_index_valid = math.isfinite(zoom_index)
    except ValueError:
        zoom_index = 0.0
        zoom_index_valid = False
    if not zoom_index_valid:
        zoom_index = 0.0
    return {
        "context_topic": LaunchConfiguration("experiment_context_topic").perform(context),
        "source_application": "Simulation",
        "run_id": LaunchConfiguration("research_run_id").perform(context),
        "case_id": LaunchConfiguration("research_case_id").perform(context),
        "scenario_name": LaunchConfiguration("scenario_name").perform(context),
        "simulation_epoch": 0,
        "simulation_epoch_valid": False,
        "simulation_epoch_source": "not_published_by_simulation_launch",
        "repository": LaunchConfiguration("research_repository").perform(context),
        "branch": LaunchConfiguration("research_branch").perform(context),
        "commit_sha": LaunchConfiguration("research_commit_sha").perform(context),
        "git_status": LaunchConfiguration("research_git_status").perform(context),
        "generated_sdf_sha256": str(generated_sdf_sha256),
        "nominal_baseline_m": baseline_m,
        "zoom_index": zoom_index,
        "zoom_index_valid": zoom_index_valid,
        "zoom_index_source": "launch_argument" if zoom_index_valid else "unresolved_launch_default",
        "camera_image_width_px": _int_config(context, "camera_image_width_px"),
        "camera_image_height_px": _int_config(context, "camera_image_height_px"),
        "camera_update_rate_hz": _float_config(context, "camera_update_rate_hz"),
        "camera_horizontal_fov_rad": _float_config(context, "horizontal_fov_rad"),
        "observation_mode": LaunchConfiguration("observation_mode").perform(context),
        "nominal_left_position_world_enu_m": list(nominal_left.center_world_enu_m),
        "nominal_left_orientation_xyzw": list(camera_pose_quaternion_xyzw(nominal_left)),
        "nominal_right_position_world_enu_m": list(nominal_right.center_world_enu_m),
        "nominal_right_orientation_xyzw": list(camera_pose_quaternion_xyzw(nominal_right)),
        "left_direct_rpy_deg": list(direct_left),
        "right_direct_rpy_deg": list(direct_right),
        "common_rpy_deg": list(common),
        "relative_rpy_deg": list(relative),
    }


def _launch_setup(context, *args, **kwargs):
    world = _generate_world(context)
    left_camera_pose_diagnostic, right_camera_pose_diagnostic = _camera_pose_diagnostic_parameters(
        context, world
    )
    share_dir = get_package_share_directory("stereo_ground_sim")
    models_dir = os.path.join(share_dir, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        item for item in [models_dir, existing_resource_path] if item
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("stereo_ground_sim"), "rviz", "stereo_ground_sim.rviz"]
    )

    yolo_python = LaunchConfiguration("yolo_python")
    distance_params_file = _write_distance_node_params(
        context,
        camera_oracle_generated_sdf_sha256=left_camera_pose_diagnostic["generated_sdf_sha256"],
    )
    overlay_params_file = _write_overlay_node_params(context)
    gui_config = os.path.join(os.getcwd(), "config", "gazebo_operator.gui.config")
    gz_args = f"-r --gui-config {gui_config} {world}" if _bool_config(context, "gz_gui") else f"-r -s {world}"
    gz_version = LaunchConfiguration("gz_version")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                    )
                ]
            ),
            launch_arguments={"gz_args": gz_args, "gz_version": gz_version}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/world/stereo_ground_drone/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "--ros-args",
                "-r",
                "/world/stereo_ground_drone/clock:=/clock",
                "-r",
                "__node:=operator_camera_bridge",
            ],
            output="screen",
        ),
        Node(
            package="stereo_ground_sim",
            executable="simulation_experiment_context",
            name="simulation_experiment_context",
            parameters=[_experiment_context_parameters(
                context,
                left_camera_pose_diagnostic["generated_sdf_sha256"],
            )],
            output="screen",
        ),
        Node(
            package="stereo_ground_sim",
            executable="gazebo_ground_truth_adapter",
            parameters=[
                {
                    "parent_model_name": LaunchConfiguration("target_model_name"),
                    "source_link_name": "",
                }
            ],
            output="screen",
            respawn=True,
            respawn_delay=1.0,
        ),
        Node(
            package="stereo_ground_sim",
            executable="gazebo_camera_pose_diagnostics",
            name="gazebo_camera_pose_diagnostics_left",
            parameters=[left_camera_pose_diagnostic],
            output="screen",
        ),
        Node(
            package="stereo_ground_sim",
            executable="gazebo_camera_pose_diagnostics",
            name="gazebo_camera_pose_diagnostics_right",
            parameters=[right_camera_pose_diagnostic],
            output="screen",
        ),
        Node(
            package="stereo_ground_sim",
            executable="camera_px4_pose_diagnostics",
            name="camera_px4_pose_diagnostics_left",
            parameters=[
                {
                    "camera_label": "left",
                    "px4_odometry_topic": "/camera_left/fmu/out/vehicle_odometry",
                    "gazebo_camera_diagnostic_topic": "/simulation/camera_pose_diagnostics/left",
                    "output_topic": "/simulation/camera_px4_pose_diagnostics/left",
                }
            ],
            output="screen",
        ),
        Node(
            package="stereo_ground_sim",
            executable="camera_px4_pose_diagnostics",
            name="camera_px4_pose_diagnostics_right",
            parameters=[
                {
                    "camera_label": "right",
                    "px4_odometry_topic": "/camera_right/fmu/out/vehicle_odometry",
                    "gazebo_camera_diagnostic_topic": "/simulation/camera_pose_diagnostics/right",
                    "output_topic": "/simulation/camera_px4_pose_diagnostics/right",
                }
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                yolo_python,
                "-m",
                "stereo_ground_sim.stereo_distance_node",
                "--ros-args",
                "--params-file",
                distance_params_file,
            ],
            name="stereo_distance_node",
            output="screen",
        ),
        Node(
            package="stereo_ground_sim_overlay",
            executable="detection_overlay_node",
            parameters=[overlay_params_file],
            condition=IfCondition(LaunchConfiguration("enable_detection_overlay")),
            output="screen",
        ),
        # Gazebo poses and RViz markers use World ENU directly. Gazebo does
        # not publish that visualization frame into /tf, so register its
        # identity relation to the ROS world root when RViz is enabled.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gazebo_world_enu_tf",
            arguments=["0", "0", "0", "0", "0", "0", "1", "world", "gazebo_world_enu"],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
            output="screen",
        ),
        Node(
            package="stereo_ground_sim",
            executable="calibration_csv_recorder",
            arguments=[
                "--output-csv",
                LaunchConfiguration("calibration_csv_path"),
                "--truth-topic",
                LaunchConfiguration("calibration_truth_topic"),
                "--local-truth-topic",
                LaunchConfiguration("calibration_local_truth_topic"),
                "--global-truth-topic",
                LaunchConfiguration("calibration_global_truth_topic"),
                "--gps-truth-topic",
                LaunchConfiguration("calibration_gps_truth_topic"),
                "--truth-source",
                LaunchConfiguration("calibration_truth_source"),
                "--estimated-topic",
                LaunchConfiguration("calibration_nominal_pose_topic"),
                "--detections-topic",
                LaunchConfiguration("calibration_detections_topic"),
                "--disparity-topic",
                LaunchConfiguration("calibration_disparity_topic"),
                "--sim-truth-topic",
                LaunchConfiguration("calibration_sim_truth_topic"),
                "--sim-truth-role",
                LaunchConfiguration("calibration_sim_truth_role"),
                "--hz",
                LaunchConfiguration("calibration_record_hz"),
            ],
            condition=IfCondition(LaunchConfiguration("record_calibration_csv")),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_name", default_value="unspecified"),
            DeclareLaunchArgument(
                "experiment_context_topic",
                default_value="/simulation/experiment_context",
            ),
            DeclareLaunchArgument("target_class", default_value="unspecified"),
            DeclareLaunchArgument("target_size_m", default_value="nan"),
            DeclareLaunchArgument("scenario_range_min_m", default_value="nan"),
            DeclareLaunchArgument("scenario_range_max_m", default_value="nan"),
            DeclareLaunchArgument("zoom_index", default_value="nan"),
            DeclareLaunchArgument("baseline_m", default_value="25.0"),
            DeclareLaunchArgument("camera_height_m", default_value="5.0"),
            DeclareLaunchArgument("horizontal_fov_rad", default_value="0.154997710698"),
            DeclareLaunchArgument("camera_image_width_px", default_value="1280"),
            DeclareLaunchArgument("camera_image_height_px", default_value="720"),
            DeclareLaunchArgument("camera_update_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("camera_clip_near_m", default_value="0.1"),
            DeclareLaunchArgument("camera_clip_far_m", default_value="2500.0"),
            DeclareLaunchArgument("cam_l_mag_noise_x_gauss", default_value="0.0"),
            DeclareLaunchArgument("cam_l_mag_noise_y_gauss", default_value="0.0"),
            DeclareLaunchArgument("cam_l_mag_noise_z_gauss", default_value="0.0"),
            DeclareLaunchArgument("cam_r_mag_noise_x_gauss", default_value="0.0"),
            DeclareLaunchArgument("cam_r_mag_noise_y_gauss", default_value="0.0"),
            DeclareLaunchArgument("cam_r_mag_noise_z_gauss", default_value="0.0"),
            *(
                DeclareLaunchArgument(name, default_value=default)
                for camera in ("l", "r")
                for axis in ("x", "y", "z")
                for name, default in (
                    (f"cam_{camera}_mag_bias_mean_{axis}_gauss", "0.0"),
                    (f"cam_{camera}_mag_bias_stddev_{axis}_gauss", "0.0"),
                    (f"cam_{camera}_gyro_noise_{axis}_rad_s", "0.0008726646"),
                    (f"cam_{camera}_gyro_bias_mean_{axis}_rad_s", "0.0"),
                    (f"cam_{camera}_gyro_bias_stddev_{axis}_rad_s", "0.0"),
                )
            ),
            DeclareLaunchArgument("approach_start_distance_m", default_value="2000.0"),
            DeclareLaunchArgument("target_spawn_x_m", default_value="0.0"),
            DeclareLaunchArgument("target_spawn_y_m", default_value="2000.0"),
            DeclareLaunchArgument("target_spawn_z_m", default_value="0.75"),
            DeclareLaunchArgument("target_spawn_yaw_deg", default_value="-90.0"),
            DeclareLaunchArgument("approach_end_distance_m", default_value="500.0"),
            DeclareLaunchArgument("approach_duration_s", default_value="120.0"),
            DeclareLaunchArgument("mission_profile", default_value="manual"),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="10.0"),
            DeclareLaunchArgument("takeoff_duration_s", default_value="10.0"),
            DeclareLaunchArgument("first_hover_duration_s", default_value="5.0"),
            DeclareLaunchArgument("final_hover_duration_s", default_value="5.0"),
            DeclareLaunchArgument("land_duration_s", default_value="10.0"),
            DeclareLaunchArgument("fly_target_in_gazebo", default_value="false"),
            DeclareLaunchArgument("target_model_name", default_value="x500_15m"),
            DeclareLaunchArgument("target_static_for_research", default_value="false"),
            DeclareLaunchArgument("yolo_model_path", default_value=""),
            DeclareLaunchArgument("enable_yolo", default_value="true"),
            DeclareLaunchArgument("enable_detection_overlay", default_value="true"),
            DeclareLaunchArgument("yolo_class_name", default_value="drone"),
            DeclareLaunchArgument("yolo_confidence", default_value="0.25"),
            DeclareLaunchArgument("yolo_device", default_value="cuda:0"),
            DeclareLaunchArgument("yolo_allow_cpu_fallback", default_value="true"),
            DeclareLaunchArgument("yolo_target_hz", default_value="40.0"),
            DeclareLaunchArgument("yolo_imgsz", default_value="1280"),
            DeclareLaunchArgument("observation_mode", default_value="bbox_yolo"),
            DeclareLaunchArgument("measurement_queue_capacity", default_value="4096"),
            DeclareLaunchArgument("measurement_latest_only", default_value="false"),
            # Keep every ray family (nominal, optical, ideal, and GPS
            # baseline) on one newest-pair RViz cadence.  This uses a separate
            # display-only YOLO stream; callers can still disable it for a
            # measurement-only run with ``rviz_live_latest_only:=false``.
            DeclareLaunchArgument("rviz_live_latest_only", default_value="true"),
            DeclareLaunchArgument("rviz_live_ray_hz", default_value="15.0"),
            DeclareLaunchArgument("enable_synthetic_detections", default_value="false"),
            DeclareLaunchArgument("enable_gps_baseline_triangulation", default_value="false"),
            DeclareLaunchArgument("gps_baseline_source", default_value="gps_baseline"),
            DeclareLaunchArgument("gps_baseline_left_topic", default_value="/camera_left/fmu/out/vehicle_gps_position"),
            DeclareLaunchArgument("gps_baseline_right_topic", default_value="/camera_right/fmu/out/vehicle_gps_position"),
            DeclareLaunchArgument("global_baseline_left_topic", default_value="/camera_left/fmu/out/vehicle_global_position"),
            DeclareLaunchArgument("global_baseline_right_topic", default_value="/camera_right/fmu/out/vehicle_global_position"),
            DeclareLaunchArgument("gps_baseline_max_age_s", default_value="2.5"),
            DeclareLaunchArgument("px4_attitude_left_topic", default_value="/camera_left/fmu/out/vehicle_attitude"),
            DeclareLaunchArgument("px4_attitude_right_topic", default_value="/camera_right/fmu/out/vehicle_attitude"),
            # Seconds: 2.5 matches the existing camera/GPS diagnostic freshness
            # envelope; pair skew uses the same simulation-data contract.
            DeclareLaunchArgument("px4_attitude_max_age_s", default_value="2.5"),
            DeclareLaunchArgument("px4_attitude_pair_skew_s", default_value="2.5"),
            DeclareLaunchArgument("enable_rviz_rays", default_value="true"),
            DeclareLaunchArgument("synthetic_detection_width_px", default_value="1280"),
            DeclareLaunchArgument("synthetic_detection_height_px", default_value="720"),
            DeclareLaunchArgument("synthetic_detection_bbox_size_px", default_value="32.0"),
            DeclareLaunchArgument("enable_external_bbox_detections", default_value="false"),
            DeclareLaunchArgument("external_bbox_detections_topic", default_value="/stereo/external_detections"),
            DeclareLaunchArgument("external_bbox_detection_width_px", default_value="1280"),
            DeclareLaunchArgument("external_bbox_detection_height_px", default_value="720"),
            DeclareLaunchArgument("overlay_publish_hz", default_value="5.0"),
            DeclareLaunchArgument("yolo_python", default_value="python3"),
            DeclareLaunchArgument("plot_publish_hz", default_value="30.0"),
            DeclareLaunchArgument("enable_bbox_center_adjustment", default_value="false"),
            DeclareLaunchArgument("bbox_center_adjustment_path", default_value=""),
            DeclareLaunchArgument("enable_observation_fusion", default_value="false"),
            DeclareLaunchArgument("observation_fusion_manifest_path", default_value=""),
            DeclareLaunchArgument("observation_fusion_tier", default_value=""),
            DeclareLaunchArgument("enable_disparity_residual_profile", default_value="false"),
            DeclareLaunchArgument("disparity_residual_profile_path", default_value=""),
            DeclareLaunchArgument("enable_disparity_residual_feature_selector", default_value="false"),
            DeclareLaunchArgument("disparity_residual_feature_selector_model_path", default_value=""),
            DeclareLaunchArgument("disparity_residual_feature_selector_catalog_path", default_value=""),
            DeclareLaunchArgument("disparity_residual_feature_selector_window_size", default_value="1"),
            DeclareLaunchArgument("disparity_residual_feature_selector_min_samples", default_value="1"),
            DeclareLaunchArgument("gz_gui", default_value="false"),
            DeclareLaunchArgument(
                "gz_version",
                default_value="8",
                description="Gazebo Sim major version (8 = Harmonic, 6 = Fortress)",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("record_calibration_csv", default_value="false"),
            DeclareLaunchArgument(
                "calibration_csv_path",
                default_value="results/calibration_flight/telemetry.csv",
            ),
            DeclareLaunchArgument("calibration_truth_topic", default_value="/fmu/out/vehicle_local_position"),
            DeclareLaunchArgument("calibration_local_truth_topic", default_value="/fmu/out/vehicle_local_position"),
            DeclareLaunchArgument("calibration_global_truth_topic", default_value="/fmu/out/vehicle_global_position"),
            DeclareLaunchArgument("calibration_gps_truth_topic", default_value="/fmu/out/vehicle_gps_position"),
            DeclareLaunchArgument("calibration_truth_source", default_value="local_ned"),
            DeclareLaunchArgument("calibration_nominal_pose_topic", default_value="/stereo/nominal_pose_plot"),
            DeclareLaunchArgument("calibration_detections_topic", default_value="/stereo/detections"),
            DeclareLaunchArgument("calibration_disparity_topic", default_value="/stereo/disparity"),
            DeclareLaunchArgument("calibration_sim_truth_topic", default_value=""),
            DeclareLaunchArgument("calibration_sim_truth_role", default_value="oracle"),
            DeclareLaunchArgument("calibration_record_hz", default_value="10.0"),
            DeclareLaunchArgument("cam_l_optical_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_l_optical_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_l_optical_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_optical_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_optical_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_optical_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_l_px4_attitude_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_l_px4_attitude_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_l_px4_attitude_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_px4_attitude_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_px4_attitude_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_px4_attitude_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_l_gnss_baseline_to_forward_yaw_deg", default_value="-90.0"),
            DeclareLaunchArgument("cam_r_gnss_baseline_to_forward_yaw_deg", default_value="-90.0"),
            DeclareLaunchArgument("common_optical_roll_error_deg", default_value="0.0"),
            DeclareLaunchArgument("relative_optical_roll_error_deg", default_value="0.0"),
            DeclareLaunchArgument("common_optical_pitch_error_deg", default_value="0.0"),
            DeclareLaunchArgument("relative_optical_pitch_error_deg", default_value="0.0"),
            DeclareLaunchArgument("common_optical_yaw_error_deg", default_value="0.0"),
            DeclareLaunchArgument("relative_optical_yaw_error_deg", default_value="0.0"),
            DeclareLaunchArgument("enable_research_recording", default_value="false"),
            DeclareLaunchArgument("research_output_dir", default_value=""),
            DeclareLaunchArgument("research_run_id", default_value=""),
            DeclareLaunchArgument("research_session_id", default_value=""),
            DeclareLaunchArgument("research_attempt_id", default_value=""),
            DeclareLaunchArgument("research_case_id", default_value=""),
            DeclareLaunchArgument("research_effective_config_path", default_value=""),
            DeclareLaunchArgument("research_effective_config_sha256", default_value=""),
            DeclareLaunchArgument("research_repository", default_value=""),
            DeclareLaunchArgument("research_branch", default_value=""),
            DeclareLaunchArgument("research_commit_sha", default_value=""),
            DeclareLaunchArgument("research_git_status", default_value=""),
            DeclareLaunchArgument("research_gazebo_version", default_value=""),
            DeclareLaunchArgument("research_px4_version", default_value="not_applicable"),
            DeclareLaunchArgument("research_record_queue_capacity", default_value="1024"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
