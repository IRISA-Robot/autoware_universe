cmake_minimum_required(VERSION 3.14)
project(autoware_turn_in_place_supervisor)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Pure logic (no ROS), shared by the nodes and the tests.
ament_auto_add_library(${PROJECT_NAME}_logic SHARED
  src/geometry.cpp
  src/decision.cpp
  src/executor_logic.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/turn_in_place_supervisor_node.cpp
  src/turn_in_place_executor_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_logic)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::turn_in_place_supervisor::TurnInPlaceSupervisorNode"
  EXECUTABLE turn_in_place_supervisor_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::turn_in_place_supervisor::TurnInPlaceExecutorNode"
  EXECUTABLE turn_in_place_executor_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_geometry.cpp
    test/test_decision.cpp
    test/test_executor_logic.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_logic)
  target_include_directories(test_${PROJECT_NAME} PRIVATE include)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
