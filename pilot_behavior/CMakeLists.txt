cmake_minimum_required(VERSION 3.5)
project(pilot_behavior)

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
find_package(bica_graph REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

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
  bica_graph
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#add_executable(bt_tiago src/bt_tiago.cpp)
#ament_target_dependencies(bt_tiago ${dependencies})

add_executable(bt_bica src/bt_bica.cpp)
ament_target_dependencies(bt_bica ${dependencies})

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  #bt_tiago
  bt_bica
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
