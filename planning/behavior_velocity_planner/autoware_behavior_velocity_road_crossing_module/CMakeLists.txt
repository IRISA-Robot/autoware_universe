cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_road_crossing_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/manager.cpp
  src/scene.cpp
  src/experimental/manager.cpp
  src/experimental/scene.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_stop_zone
    test/test_stop_zone.cpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config)
