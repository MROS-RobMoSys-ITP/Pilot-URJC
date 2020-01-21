cmake_minimum_required(VERSION 3.5)
project(metacontroller_pilot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(bica REQUIRED)
find_package(bica_graph REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(bica_behavior_tree REQUIRED)
find_package(rclcpp_action REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(dependencies
  rclcpp
  rclcpp_lifecycle
  behaviortree_cpp_v3
  bica
  bica_graph
  ament_index_cpp
  nav2_msgs
  bica_behavior_tree
  rclcpp_action
)


ament_package()
