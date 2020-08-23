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

#ifndef PILOT_BEHAVIOR__BEHAVIOR_TREE_NODES__GETORDER_HPP_
#define PILOT_BEHAVIOR__BEHAVIOR_TREE_NODES__GETORDER_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros2_knowledge_graph/GraphNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace pilot_behavior
{

class GetOrder : public BT::ActionNodeBase
{
public:
  explicit GetOrder(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {}
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
};

}  // namespace pilot_behavior

#endif  // PILOT_BEHAVIOR__BEHAVIOR_TREE_NODES__GETORDER_HPP_
