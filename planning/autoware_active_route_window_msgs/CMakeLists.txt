cmake_minimum_required(VERSION 3.14)
project(autoware_active_route_window_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActiveRouteWindow.msg"
  DEPENDENCIES
    std_msgs
    unique_identifier_msgs
)

ament_package()
