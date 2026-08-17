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

# NOTE: no config/ param yaml is installed yet -- all parameters are declared with in-code
# defaults (see src/manager.cpp) so the package builds and unit-tests standalone. Once
# tuning-engineer adds config/reverse_lane_follow.param.yaml, add it here:
#   ament_auto_package(INSTALL_TO_SHARE config)
ament_auto_package()
