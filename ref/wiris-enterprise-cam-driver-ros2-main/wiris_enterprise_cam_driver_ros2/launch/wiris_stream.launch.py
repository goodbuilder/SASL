from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
     pkg = get_package_share_directory('wiris_enterprise_cam_driver_ros2')
     params = os.path.join(pkg, 'config', 'params.yaml')

     svc_node_eo = Node(
          package='wiris_enterprise_cam_driver_ros2',
          executable='wiris_stream_eo_node',
          name='wiris_stream_eo',
          output='screen',
          respawn=True,          
          respawn_delay=30.0,              
          parameters=[params]
     )
     svc_node_ir = Node(
          package='wiris_enterprise_cam_driver_ros2',
          executable='wiris_stream_ir_node',
          name='wiris_stream_ir',
          output='screen',
          respawn=True,          
          respawn_delay=30.0,
          parameters=[params]
     )
     delayed_start = TimerAction(
          period=30.0,           
          actions=[svc_node_eo, svc_node_ir]
     )
     return LaunchDescription([delayed_start])
