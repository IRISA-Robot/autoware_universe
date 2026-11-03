cmake_minimum_required(VERSION 3.14)
project(autoware_road_crossing_fsm)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/road_crossing_fsm_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::road_crossing_fsm::RoadCrossingFsmNode"
  EXECUTABLE road_crossing_fsm_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_direction
    test/test_direction_utils.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_direction ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
