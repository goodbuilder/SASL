cmake_minimum_required(VERSION 3.8)
project(wiris_enterprise_cam_driver_ros2_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ZoomCmd.srv"
  "srv/ZoomStatus.srv"
  "srv/ServerReboot.srv"
)

ament_export_dependencies(rosidl_default_runtime)
