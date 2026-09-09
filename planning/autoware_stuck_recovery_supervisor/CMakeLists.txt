cmake_minimum_required(VERSION 3.14)
project(autoware_stuck_recovery_supervisor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/corridor_checker.cpp
  src/stuck_recovery_supervisor_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::stuck_recovery_supervisor::StuckRecoverySupervisorNode"
  EXECUTABLE stuck_recovery_supervisor_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
