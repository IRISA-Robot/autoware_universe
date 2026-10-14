cmake_minimum_required(VERSION 3.14)
project(autoware_active_route_window)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/window_calculator.cpp
  src/active_route_window_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::active_route_window::ActiveRouteWindowNode"
  EXECUTABLE active_route_window_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_window_calculator.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
