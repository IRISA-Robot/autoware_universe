cmake_minimum_required(VERSION 3.14)
project(autoware_stuck_recovery_supervisor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/corridor_checker.cpp
  src/mission_direction.cpp
  src/stuck_recovery_supervisor_node.cpp
  src/turn_in_place_gate.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::stuck_recovery_supervisor::StuckRecoverySupervisorNode"
  EXECUTABLE stuck_recovery_supervisor_node
)

# [TURN-IN-PLACE 2026-11-05] Pure-logic tests: no node is spun up.
if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_mission_direction.cpp
    test/test_turn_in_place_gate.cpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
