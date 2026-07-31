from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("pose_simulation"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="simulation/pose_simulation",
                description="ROS namespace for the diagnostic-only core node.",
            ),
            Node(
                package="pose_simulation",
                executable="pose_simulation",
                name="pose_simulation",
                namespace=LaunchConfiguration("namespace"),
                parameters=[str(package_share / "config" / "pose_simulation.yaml")],
                output="screen",
            )
        ]
    )
