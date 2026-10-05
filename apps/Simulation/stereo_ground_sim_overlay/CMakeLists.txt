cmake_minimum_required(VERSION 3.8)
project(stereo_ground_sim_overlay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(detection_overlay_node src/detection_overlay_node.cpp)
ament_target_dependencies(detection_overlay_node rclcpp sensor_msgs std_msgs)

install(TARGETS detection_overlay_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
