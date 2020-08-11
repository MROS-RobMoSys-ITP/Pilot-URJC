cmake_minimum_required(VERSION 3.5)
project(pilot_urjc_bringup)

set(CMAKE_BUILD_TYPE DEBUG)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
