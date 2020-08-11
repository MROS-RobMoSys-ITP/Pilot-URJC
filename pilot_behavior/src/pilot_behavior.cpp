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

/* Author: Lorena Bajo Rebollo lorena.bajo@urjc.es */

/* Mantainer: Lorena Bajo Rebollo lorena.bajo@urjc.es */


#include <math.h>
#include <iostream>
#include <memory>
#include <string>
#include <map>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/blackboard.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("pilot_behavior");
  std::string xml_file = pkgpath + "/behavior_trees/bt.xml";

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  factory.registerFromPlugin(loader.getOSName("pilot_behavior_deliver_order_bt_node"));
  factory.registerFromPlugin(loader.getOSName("pilot_behavior_interact_with_barman_bt_node"));
  factory.registerFromPlugin(loader.getOSName("pilot_behavior_get_order_bt_node"));
  factory.registerFromPlugin(loader.getOSName("pilot_behavior_navigate_to_barman_bt_node"));
  factory.registerFromPlugin(loader.getOSName("pilot_behavior_navigate_to_client_bt_node"));
  factory.registerFromPlugin(loader.getOSName("pilot_behavior_check_order_bt_node"));

  auto blackboard = BT::Blackboard::create();
  auto node = rclcpp::Node::make_shared("pilot_node");
  blackboard->set("node", node);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(1);
  bool finished = false;
  while (rclcpp::ok() && !finished) {
    finished = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Pilot execution finished");

  rclcpp::shutdown();

  return 0;
}
