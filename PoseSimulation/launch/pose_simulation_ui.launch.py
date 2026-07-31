"""Launch the PoseSimulation core and its read-only rqt/RViz UI."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("pose_simulation"))
    namespace = LaunchConfiguration("namespace")
    ui_config = LaunchConfiguration("ui_config")
    visualization_config = LaunchConfiguration("visualization_config")
    rviz_config = LaunchConfiguration("rviz_config")
    start_core = LaunchConfiguration("start_pose_simulation")
    start_visualization = LaunchConfiguration("start_visualization")
    start_rqt = LaunchConfiguration("start_rqt")
    start_rviz = LaunchConfiguration("start_rviz")

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "pose_simulation.launch.py")),
        launch_arguments={"namespace": namespace}.items(),
        condition=IfCondition(start_core),
    )
    visualization_node = Node(
        package="pose_simulation",
        executable="pose_visualization",
        name="pose_visualization",
        namespace=namespace,
        parameters=[visualization_config],
        output="screen",
        condition=IfCondition(start_visualization),
    )
    rqt_process = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rqt_gui",
            "rqt_gui",
            "--clear-config",
            "--force-discover",
            "-s",
            "pose_simulation_rqt.pose_simulation_plugin.PoseSimulationPlugin",
        ],
        output="screen",
        condition=IfCondition(start_rqt),
    )
    rviz_process = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config],
        output="screen",
        condition=IfCondition(start_rviz),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_pose_simulation",
                default_value="true",
                description="Start the PoseSimulation core node.",
            ),
            DeclareLaunchArgument(
                "start_visualization",
                default_value="true",
                description="Start the read-only MarkerArray/DiagnosticArray adapter.",
            ),
            DeclareLaunchArgument(
                "start_rqt",
                default_value="true",
                description="Start the PoseSimulation rqt plugin.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the PoseSimulation configuration.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="pose_simulation",
                description="Common ROS namespace for core, visualization, and UI topics.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(package_share / "config" / "pose_simulation.rviz"),
                description="RViz configuration path.",
            ),
            DeclareLaunchArgument(
                "ui_config",
                default_value=str(package_share / "config" / "pose_simulation_ui.yaml"),
                description="Read-only rqt UI configuration path.",
            ),
            DeclareLaunchArgument(
                "visualization_config",
                default_value=str(package_share / "config" / "pose_simulation_visualization.yaml"),
                description="ROS parameters for the visualization adapter.",
            ),
            SetEnvironmentVariable("POSE_SIMULATION_UI_CONFIG", ui_config),
            SetEnvironmentVariable("POSE_SIMULATION_NAMESPACE", namespace),
            core_launch,
            visualization_node,
            rqt_process,
            rviz_process,
        ]
    )
