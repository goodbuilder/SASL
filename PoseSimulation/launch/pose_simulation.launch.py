from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("pose_simulation"))
    return LaunchDescription(
        [
            Node(
                package="pose_simulation",
                executable="pose_simulation",
                name="pose_simulation",
                namespace="simulation/pose_simulation",
                parameters=[str(package_share / "config" / "pose_simulation.yaml")],
                output="screen",
            )
        ]
    )
