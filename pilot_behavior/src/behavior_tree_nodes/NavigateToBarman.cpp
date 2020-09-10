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

#include "pilot_behavior/behavior_tree_nodes/NavigateToBarman.hpp"

namespace pilot_behavior
{

NavigateToBarman::NavigateToBarman(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void
NavigateToBarman::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;

  goal.pose.position.x = -0.5;
  goal.pose.position.y = -0.5;
  goal.header.frame_id = "map";

  goal_.pose = goal;
}

BT::NodeStatus
NavigateToBarman::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pilot_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<pilot_behavior::NavigateToBarman>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<pilot_behavior::NavigateToBarman>(
    "NavigateToBarman", builder);
}
