cmake_minimum_required(VERSION 3.14)
project(autoware_lanelet_tag_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lanelet_tag_publisher_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lanelet_tag_publisher::LaneletTagPublisherNode"
  EXECUTABLE lanelet_tag_publisher_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
