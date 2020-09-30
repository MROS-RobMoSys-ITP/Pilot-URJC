cmake_minimum_required(VERSION 3.5)
project(laser_resender)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    std_msgs
    sensor_msgs
)

add_executable(laser_resender_node src/laser_resender_node.cpp)
ament_target_dependencies(laser_resender_node ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  laser_resender_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
