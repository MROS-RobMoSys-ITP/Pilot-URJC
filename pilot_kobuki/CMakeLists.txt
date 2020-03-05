cmake_minimum_required(VERSION 3.5)

project(pilot_kobuki)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

add_executable(laser_filter_node src/laser_filter_node.cpp)
ament_target_dependencies(laser_filter_node rclcpp rcutils sensor_msgs)
target_include_directories(laser_filter_node
  PRIVATE
  ${sensor_msgs_INCLUDE_DIR}
)

install(TARGETS laser_filter_node DESTINATION lib/csuro_intro_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  laser_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  config
  map
  DESTINATION share/${PROJECT_NAME})

ament_package()
