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

#include "pilot_behavior/behavior_tree_nodes/GetOrder.hpp"

namespace pilot_behavior
{

GetOrder::GetOrder(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  //graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(xml_tag_name + node_->get_name());
  //graph_->start();
  graph_ = config().blackboard->get<std::shared_ptr<ros2_knowledge_graph::GraphNode>>("pilot_graph");
}

BT::NodeStatus
GetOrder::tick()
{
  graph_->add_node(ros2_knowledge_graph::Node{"sonny", "robot"});
  graph_->add_node(ros2_knowledge_graph::Node{"table_1", "table"});
  graph_->add_node(ros2_knowledge_graph::Node{"bar", "table"});
  graph_->add_node(ros2_knowledge_graph::Node{"water", "object"});
  graph_->add_node(ros2_knowledge_graph::Node{"toastie", "object"});
  graph_->add_node(ros2_knowledge_graph::Node{"mango_juice", "object"});

  graph_->add_edge(ros2_knowledge_graph::Edge{"wants", "symbolic", "table_1", "water"});
  graph_->add_edge(ros2_knowledge_graph::Edge{"wants", "symbolic", "table_1", "toastie"});
  graph_->add_edge(ros2_knowledge_graph::Edge{"wants", "symbolic", "table_1", "mango_juice"});

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pilot_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pilot_behavior::GetOrder>("GetOrder");
}
