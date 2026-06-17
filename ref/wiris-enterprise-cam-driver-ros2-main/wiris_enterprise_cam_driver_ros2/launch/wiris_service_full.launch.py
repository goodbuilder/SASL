
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('wiris_enterprise_cam_driver_ros2')
    params_file = os.path.join(pkg_share, 'config', 'full_params.yaml')
    return LaunchDescription([
        Node(
            package='wiris_enterprise_cam_driver_ros2',
            executable='wiris_service_node',
            name='wiris_service',
            output='screen',
            parameters=[params_file],
        ),
    ])
