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

#include "pilot_behavior/behavior_tree_nodes/InteractWithBarman.hpp"

namespace pilot_behavior
{

InteractWithBarman::InteractWithBarman(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(xml_tag_name + node_->get_name());

  graph_->start();
}

BT::NodeStatus
InteractWithBarman::tick()
{
  // TODO(fmrico): Decir la comanda al barman
  std::string text_to_say = "The table_1 wants: ";

  RCLCPP_INFO(
    node_->get_logger(), "Nodes table_1 [%d]", graph_->get_node_names_by_id(
      "table_1").size());
  RCLCPP_INFO(
    node_->get_logger(), "Edges in table_1 [%d]", graph_->get_edges_from_node(
      "table_1").size());

  auto edges = graph_->get_edges_from_node_by_data("table_1", "wants", "symbolic");

  for (auto & edge : edges) {
    text_to_say += edge.target + " ,";
  }

  RCLCPP_INFO(node_->get_logger(), "%s", text_to_say.c_str());

  config().blackboard->set("text", text_to_say);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pilot_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pilot_behavior::InteractWithBarman>("InteractWithBarman");
}
