cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_reverse_lane_follow_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scene.cpp
  src/utils.cpp
  src/manager.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_utils
    test/test_reverse_lane_follow_utils.cpp
  )

  target_include_directories(test_${PROJECT_NAME}_utils PRIVATE src)

  target_link_libraries(test_${PROJECT_NAME}_utils
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config)
