from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('wiris_enterprise_cam_driver_ros2')
    params = os.path.join(pkg, 'config', 'full_params.yaml')

    def _print_at_launch_start(context, *args, **kwargs):
        print(f"[LAUNCH] using params: {params}", flush=True)
        return []

    svc_node = Node(
        package='wiris_enterprise_cam_driver_ros2',
        executable='wiris_service_node',
        name='wiris_service',
        output='screen',
        respawn=True,          
        respawn_delay=25.0,    
        parameters=[params],
    )
    delayed_start = TimerAction(
        period=25.0,           
        actions=[svc_node]
    )
    return LaunchDescription([OpaqueFunction(function=_print_at_launch_start),delayed_start])
