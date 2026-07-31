"""Launch the read-only PoseSimulation MarkerArray adapter and RViz2.

The PoseSimulation measurement core and the external Simulation are not
started by default.  This launcher is safe to use against an already-running
PoseSimulation evaluation source.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("pose_simulation"))
    namespace = LaunchConfiguration("namespace")
    visualization_config = LaunchConfiguration("visualization_config")
    rviz_config = LaunchConfiguration("rviz_config")
    start_visualization = LaunchConfiguration("start_visualization")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="pose_simulation",
                description="PoseSimulation namespace; the external Simulation is not owned here.",
            ),
            DeclareLaunchArgument(
                "start_visualization",
                default_value="true",
                description="Start the read-only typed MarkerArray adapter.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz2 with the PoseSimulation display configuration.",
            ),
            DeclareLaunchArgument(
                "visualization_config",
                default_value=str(package_share / "config" / "pose_simulation_visualization.yaml"),
                description="Visualization adapter parameter file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(package_share / "config" / "pose_simulation.rviz"),
                description="RViz2 configuration file.",
            ),
            Node(
                package="pose_simulation",
                executable="pose_visualization",
                name="pose_visualization",
                namespace=namespace,
                parameters=[visualization_config],
                output="screen",
                condition=IfCondition(start_visualization),
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config],
                output="screen",
                condition=IfCondition(start_rviz),
            ),
        ]
    )
