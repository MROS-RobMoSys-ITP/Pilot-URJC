// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "pilot_lifecycle_manager/lifecycle_manager.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  std::map<std::string, std::shared_ptr<pilot_urjc::LifecycleServiceClient>> manager_nodes;
  manager_nodes["laser_resender"] = std::make_shared<pilot_urjc::LifecycleServiceClient>(
    "laser_resender_node", "laser_resender");
  manager_nodes["pointcloud_to_laser"] = std::make_shared<pilot_urjc::LifecycleServiceClient>(
    "pointcloud_to_laserscan_managed", "pointcloud_to_laser");
  
  /*manager_nodes["problem_expert"] = std::make_shared<pilot_urjc::LifecycleServiceClient>(
    "problem_expert_lc_mngr", "problem_expert");
  manager_nodes["planner"] = std::make_shared<pilot_urjc::LifecycleServiceClient>(
    "planner_lc_mngr", "planner");
  manager_nodes["executor"] = std::make_shared<pilot_urjc::LifecycleServiceClient>(
    "executor_lc_mngr", "executor");*/

  rclcpp::executors::SingleThreadedExecutor exe;
  for (auto & manager_node : manager_nodes) {
    manager_node.second->init();
    exe.add_node(manager_node.second);
  }

  std::shared_future<void> script = std::async(
    std::launch::async,
    std::bind(pilot_urjc::startup_script, manager_nodes));
  exe.spin_until_future_complete(script);

  rclcpp::shutdown();

  return 0;
}
