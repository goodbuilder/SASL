import json
import math
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _float_config(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _int_config(context, name):
    return int(LaunchConfiguration(name).perform(context))


def _bool_config(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ("1", "true", "yes", "on")


def _param_yaml_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        numeric_values = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
                raise ValueError("Only finite numeric YAML sequence values are supported")
            numeric_values.append(float(item))
        return json.dumps(numeric_values)
    if isinstance(value, (int, float)):
        return str(value)
    return json.dumps(str(value), ensure_ascii=False)


def _write_distance_node_params(context, params_file=None):
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
        "target_center_z_m",
        "actual_distance_read_hz",
        "yolo_confidence",
        "yolo_target_hz",
        "synthetic_detection_bbox_size_px",
        "plot_publish_hz",
        "target_size_m",
        "scenario_range_min_m",
        "scenario_range_max_m",
        "zoom_index",
    }
    int_params = {
        "yolo_imgsz",
        "synthetic_detection_width_px",
        "synthetic_detection_height_px",
        "external_bbox_detection_width_px",
        "external_bbox_detection_height_px",
        "disparity_residual_feature_selector_window_size",
        "disparity_residual_feature_selector_min_samples",
    }
    bool_params = {
        "fly_target_in_gazebo",
        "enable_synthetic_detections",
        "enable_external_bbox_detections",
        "enable_bbox_center_adjustment",
        "enable_observation_fusion",
        "enable_disparity_residual_profile",
        "enable_disparity_residual_feature_selector",
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
        "target_center_z_m",
        "fly_target_in_gazebo",
        "actual_distance_read_hz",
        "actual_distance_source",
        "target_model_name",
        "target_reference_link_name",
        "oracle_base_link_name",
        "yolo_model_path",
        "yolo_class_name",
        "yolo_confidence",
        "yolo_device",
        "yolo_target_hz",
        "yolo_imgsz",
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

    params["baseline_origin_world_enu_m"] = [0.0, 0.0, float(params["camera_height_m"])]
    params["rotation_baseline_world_row_major"] = [
        0.0,
        1.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -1.0,
    ]

    if params_file is None:
        params_file = os.path.join(tempfile.gettempdir(), "stereo_distance_node_params.yaml")
    with open(params_file, "w", encoding="utf-8") as handle:
        handle.write("/stereo_distance_node:\n")
        handle.write("  ros__parameters:\n")
        for name, value in params.items():
            handle.write(f"    {name}: {_param_yaml_value(value)}\n")
    return params_file


def _write_overlay_node_params(context):
    params_file = os.path.join(tempfile.gettempdir(), "detection_overlay_node_params.yaml")
    overlay_publish_hz = float(LaunchConfiguration("overlay_publish_hz").perform(context))
    with open(params_file, "w", encoding="utf-8") as handle:
        handle.write("/detection_overlay_node:\n")
        handle.write("  ros__parameters:\n")
        handle.write(f"    overlay_publish_hz: {overlay_publish_hz}\n")
    return params_file


def _ground_grid_visuals(size_m=4000.0, spacing_m=100.0, thickness_m=0.7):
    line_count = int(size_m / spacing_m)
    start = -line_count // 2
    end = line_count // 2
    visuals = []
    for index, grid_index in enumerate(range(start, end + 1)):
        offset = grid_index * spacing_m
        is_axis = abs(offset) < 1e-6
        color = "0.32 0.36 0.38 1" if is_axis else "0.50 0.54 0.56 1"
        width = thickness_m * (2.0 if is_axis else 1.0)
        z = 0.018 if is_axis else 0.016
        visuals.append(
            f"""        <visual name="grid_x_{index}">
          <pose>{offset:.3f} 0 {z:.3f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{width:.3f} {size_m:.3f} 0.010</size>
            </box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
        <visual name="grid_y_{index}">
          <pose>0 {offset:.3f} {z:.3f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{size_m:.3f} {width:.3f} 0.010</size>
            </box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>"""
        )
    visuals.append(
        """        <visual name="camera_to_drone_line">
          <pose>0 0 0.026 0 0 0</pose>
          <geometry>
            <box>
              <size>6.000 4000.000 0.018</size>
            </box>
          </geometry>
          <material>
            <ambient>0.08 0.10 0.11 1</ambient>
            <diffuse>0.08 0.10 0.11 1</diffuse>
          </material>
        </visual>"""
    )
    return "\n".join(visuals)


def _generate_world(context, generated_world_path=None):
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
    left_roll = math.radians(_float_config(context, "left_roll_error_deg"))
    left_pitch = math.radians(_float_config(context, "left_pitch_error_deg"))
    left_yaw = math.radians(_float_config(context, "left_yaw_error_deg"))
    right_roll = math.radians(_float_config(context, "right_roll_error_deg"))
    right_pitch = math.radians(_float_config(context, "right_pitch_error_deg"))
    right_yaw = math.radians(_float_config(context, "right_yaw_error_deg"))

    camera_yaw = math.pi / 2.0
    left_pose = (
        f"{-baseline_m / 2.0:.6f} 0 {camera_height_m:.6f} "
        f"{left_roll:.9f} {left_pitch:.9f} {camera_yaw + left_yaw:.9f}"
    )
    right_pose = (
        f"{baseline_m / 2.0:.6f} 0 {camera_height_m:.6f} "
        f"{right_roll:.9f} {right_pitch:.9f} {camera_yaw + right_yaw:.9f}"
    )

    world = world.replace("<pose>-12.5 0 5.0 0 0 1.570796327</pose>", f"<pose>{left_pose}</pose>", 1)
    world = world.replace("<pose>12.5 0 5.0 0 0 1.570796327</pose>", f"<pose>{right_pose}</pose>", 1)
    for fov_value in ("1.0471975512", "0.154997710698"):
        world = world.replace(
            f"<horizontal_fov>{fov_value}</horizontal_fov>",
            f"<horizontal_fov>{horizontal_fov_rad:.12f}</horizontal_fov>",
        )
    camera_rate_token = "<update_rate>20</update_rate>"
    camera_rate_token_count = world.count(camera_rate_token)
    if camera_rate_token_count != 2:
        raise RuntimeError(
            "Expected exactly two source camera update_rate tokens "
            f"{camera_rate_token!r}; found {camera_rate_token_count}"
        )
    world = world.replace(camera_rate_token, f"<update_rate>{camera_update_rate_hz:.6f}</update_rate>")
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

    world = world.replace("        <!-- GROUND_GRID_VISUALS -->", _ground_grid_visuals(), 1)

    generated = generated_world_path or os.path.join(tempfile.gettempdir(), "stereo_ground_drone_attitude_error.sdf")
    with open(generated, "w", encoding="utf-8") as handle:
        handle.write(world)
    return generated


def _launch_setup(context, *args, **kwargs):
    world = _generate_world(context)
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
    distance_params_file = _write_distance_node_params(context)
    overlay_params_file = _write_overlay_node_params(context)
    gui_config = os.path.join(os.getcwd(), "config", "gazebo_operator.gui.config")
    gz_args = f"-r --gui-config {gui_config} {world}" if _bool_config(context, "gz_gui") else f"-r -s {world}"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                    )
                ]
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "--ros-args",
                "-r",
                "__node:=operator_camera_bridge",
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
            DeclareLaunchArgument("approach_end_distance_m", default_value="200.0"),
            DeclareLaunchArgument("approach_duration_s", default_value="120.0"),
            DeclareLaunchArgument("mission_profile", default_value="manual"),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="10.0"),
            DeclareLaunchArgument("takeoff_duration_s", default_value="10.0"),
            DeclareLaunchArgument("first_hover_duration_s", default_value="5.0"),
            DeclareLaunchArgument("final_hover_duration_s", default_value="5.0"),
            DeclareLaunchArgument("land_duration_s", default_value="10.0"),
            DeclareLaunchArgument("target_center_z_m", default_value="5.0"),
            DeclareLaunchArgument("fly_target_in_gazebo", default_value="false"),
            DeclareLaunchArgument("actual_distance_source", default_value="gazebo"),
            DeclareLaunchArgument("actual_distance_read_hz", default_value="15.0"),
            DeclareLaunchArgument("target_model_name", default_value="x500_15m"),
            DeclareLaunchArgument("target_reference_link_name", default_value="target_reference_link"),
            DeclareLaunchArgument("oracle_base_link_name", default_value="base_link"),
            DeclareLaunchArgument("yolo_model_path", default_value=""),
            DeclareLaunchArgument("yolo_class_name", default_value="drone"),
            DeclareLaunchArgument("yolo_confidence", default_value="0.25"),
            DeclareLaunchArgument("yolo_device", default_value="cuda:0"),
            DeclareLaunchArgument("yolo_target_hz", default_value="40.0"),
            DeclareLaunchArgument("yolo_imgsz", default_value="1280"),
            DeclareLaunchArgument("enable_synthetic_detections", default_value="false"),
            DeclareLaunchArgument("synthetic_detection_width_px", default_value="1280"),
            DeclareLaunchArgument("synthetic_detection_height_px", default_value="720"),
            DeclareLaunchArgument("synthetic_detection_bbox_size_px", default_value="32.0"),
            DeclareLaunchArgument("enable_external_bbox_detections", default_value="false"),
            DeclareLaunchArgument("external_bbox_detections_topic", default_value="/stereo/external_detections"),
            DeclareLaunchArgument("external_bbox_detection_width_px", default_value="1280"),
            DeclareLaunchArgument("external_bbox_detection_height_px", default_value="720"),
            DeclareLaunchArgument("overlay_publish_hz", default_value="20.0"),
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
            DeclareLaunchArgument("left_roll_error_deg", default_value="0.0"),
            DeclareLaunchArgument("left_pitch_error_deg", default_value="0.0"),
            DeclareLaunchArgument("left_yaw_error_deg", default_value="0.0"),
            DeclareLaunchArgument("right_roll_error_deg", default_value="0.0"),
            DeclareLaunchArgument("right_pitch_error_deg", default_value="0.0"),
            DeclareLaunchArgument("right_yaw_error_deg", default_value="0.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
