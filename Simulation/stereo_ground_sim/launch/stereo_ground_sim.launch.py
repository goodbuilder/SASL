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
    effective_optical_rpy_errors_deg,
    relative_sdf_rpy,
    sdf_link_rpy_from_optical_error,
    sdf_link_rpy_from_px4_frd_error,
)


def _float_config(context, name):
    return float(LaunchConfiguration(name).perform(context))


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


def _set_target_include_static(world_xml, *, target_model_name, static_enabled):
    """Set only the configured research target include's Gazebo static flag."""
    root = ET.fromstring(world_xml)
    expected_uri = f"model://{target_model_name}"
    matches = [
        include
        for include in root.findall("./world/include")
        if (include.findtext("uri") or "").strip() == expected_uri
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one target include for {expected_uri}, found {len(matches)}"
        )
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
        "target_spawn_z_m",
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
        "target_spawn_z_m",
        "fly_target_in_gazebo",
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
        "camera_image_width_px",
        "camera_image_height_px",
        "camera_update_rate_hz",
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
        "yolo_model_path",
        "yolo_class_name",
        "yolo_confidence",
        "yolo_device",
        "yolo_target_hz",
        "measurement_queue_capacity",
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
    approach_start_distance_m = _float_config(context, "approach_start_distance_m")
    target_spawn_x_m = _float_config(context, "target_spawn_x_m")
    target_spawn_y_m = _float_config(context, "target_spawn_y_m")
    target_spawn_z_m = _float_config(context, "target_spawn_z_m")
    target_spawn_yaw_deg = _float_config(context, "target_spawn_yaw_deg")
    target_model_name = LaunchConfiguration("target_model_name", default="x500_15m").perform(context)
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
    left_px4_world_rpy = sdf_link_rpy_from_px4_frd_error(
        nominal_link_yaw,
        *(
            math.radians(_float_config(context, name))
            for name in (
                "cam_l_px4_roll_deg",
                "cam_l_px4_pitch_deg",
                "cam_l_px4_yaw_deg",
            )
        ),
    )
    right_px4_world_rpy = sdf_link_rpy_from_px4_frd_error(
        nominal_link_yaw,
        *(
            math.radians(_float_config(context, name))
            for name in (
                "cam_r_px4_roll_deg",
                "cam_r_px4_pitch_deg",
                "cam_r_px4_yaw_deg",
            )
        ),
    )
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
                LaunchConfiguration("calibration_estimated_topic"),
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
            DeclareLaunchArgument("yolo_class_name", default_value="drone"),
            DeclareLaunchArgument("yolo_confidence", default_value="0.25"),
            DeclareLaunchArgument("yolo_device", default_value="cuda:0"),
            DeclareLaunchArgument("yolo_target_hz", default_value="40.0"),
            DeclareLaunchArgument("yolo_imgsz", default_value="1280"),
            DeclareLaunchArgument("observation_mode", default_value="bbox_yolo"),
            DeclareLaunchArgument("measurement_queue_capacity", default_value="4096"),
            # Keep every ray family (nominal, optical, ideal, and GPS
            # baseline) on one newest-pair RViz cadence.  This uses a separate
            # display-only YOLO stream; callers can still disable it for a
            # measurement-only run with ``rviz_live_latest_only:=false``.
            DeclareLaunchArgument("rviz_live_latest_only", default_value="true"),
            DeclareLaunchArgument("rviz_live_ray_hz", default_value="5.0"),
            DeclareLaunchArgument("enable_synthetic_detections", default_value="false"),
            DeclareLaunchArgument("enable_gps_baseline_triangulation", default_value="false"),
            DeclareLaunchArgument("gps_baseline_source", default_value="raw_gps"),
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
            DeclareLaunchArgument("calibration_estimated_topic", default_value="/stereo/estimated_position"),
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
            DeclareLaunchArgument("cam_l_px4_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_l_px4_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_l_px4_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_px4_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_px4_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("cam_r_px4_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("common_optical_roll_error_deg", default_value="0.0"),
            DeclareLaunchArgument("relative_optical_roll_error_deg", default_value="0.0"),
            DeclareLaunchArgument("common_optical_pitch_error_deg", default_value="0.0"),
            DeclareLaunchArgument("relative_optical_pitch_error_deg", default_value="0.0"),
            DeclareLaunchArgument("common_optical_yaw_error_deg", default_value="0.0"),
            DeclareLaunchArgument("relative_optical_yaw_error_deg", default_value="0.0"),
            DeclareLaunchArgument("enable_research_recording", default_value="false"),
            DeclareLaunchArgument("research_output_dir", default_value=""),
            DeclareLaunchArgument("research_run_id", default_value=""),
            DeclareLaunchArgument("research_case_id", default_value=""),
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
