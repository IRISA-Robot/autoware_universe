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

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
