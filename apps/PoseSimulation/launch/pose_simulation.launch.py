from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="/pose_simulation",
                description="ROS namespace for the read-only M3 node.",
            ),
            Node(
                package="pose_simulation",
                executable="pose_simulation",
                name="pose_simulation",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
            )
        ]
    )
