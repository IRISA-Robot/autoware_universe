cmake_minimum_required(VERSION 3.14)
project(autoware_tracking_speed_guard)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tracking_speed_guard_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::tracking_speed_guard::TrackingSpeedGuardNode"
  EXECUTABLE tracking_speed_guard_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
