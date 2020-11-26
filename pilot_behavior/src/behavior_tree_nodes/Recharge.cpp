// Copyright 2020 Intelligent Robotics Lab
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

#include <string>
#include <memory>

#include "pilot_behavior/behavior_tree_nodes/Recharge.hpp"

namespace pilot_behavior
{

Recharge::Recharge(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::AsyncActionNode(action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus Recharge::tick()
{
  while(rclcpp::ok()) {
    RCLCPP_INFO(node_->get_logger(), "Recharging battery for 10 seconds");
    rclcpp::Rate(0.1).sleep();
    break;
  }
  RCLCPP_INFO(node_->get_logger(), "Battery fully recharged");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pilot_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<pilot_behavior::Recharge>(
        name, config);
    };

  factory.registerBuilder<pilot_behavior::Recharge>(
    "Recharge", builder);
}
