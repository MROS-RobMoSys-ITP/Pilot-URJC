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

#include "pilot_behavior/behavior_tree_nodes/NavigateToWp.hpp"

namespace pilot_behavior
{

NavigateToWp::NavigateToWp(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<NavigateToPoseQos>(xml_tag_name, action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

void NavigateToWp::on_tick()
{
  auto wp_map = 
    config().blackboard->get<std::unordered_map<std::string, geometry_msgs::msg::Pose>>("wp_map");
  
  auto res = getInput<std::string>("goal").value();
  wp_ = wp_map[res];
  RCLCPP_INFO(node_->get_logger(), "Navigating to... [%s -- %f %f]",
    res.c_str(), wp_.position.x, wp_.position.y);
  goal_.pose.pose = wp_;
  goal_.qos_expected.objective_type = "f_navigate"; // should be mros_goal->qos_expected.objective_type = "f_navigate";
  diagnostic_msgs::msg::KeyValue energy_qos;
  energy_qos.key = "energy";
  energy_qos.value = "0.3";
  diagnostic_msgs::msg::KeyValue safety_qos;
  safety_qos.key = "safety";
  safety_qos.value = "0.5";
  goal_.qos_expected.qos.push_back(energy_qos);
  goal_.qos_expected.qos.push_back(safety_qos);
}

void NavigateToWp::on_wait_for_result()
{  
  // check selected_mode, f_energy_saving_mode means the robot needs battery.
  if (feedback_->qos_status.selected_mode == "f_energy_saving_mode" && 
      getInput<std::string>("goal").value() != "recharge_station") {
    RCLCPP_ERROR(node_->get_logger(), "Not enough energy");
    halt();
    result_.code = rclcpp_action::ResultCode::ABORTED;
    goal_result_available_ = true;
  }
}

BT::NodeStatus NavigateToWp::on_success()
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
      return std::make_unique<pilot_behavior::NavigateToWp>(
        name, "navigate_to_pose_qos", config);
    };

  factory.registerBuilder<pilot_behavior::NavigateToWp>(
    "NavigateToWp", builder);
}
