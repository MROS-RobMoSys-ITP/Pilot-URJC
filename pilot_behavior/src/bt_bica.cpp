/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2020, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Lorena Bajo Rebollo lorena.bajo@urjc.es */

/* Mantainer: Lorena Bajo Rebollo lorena.bajo@urjc.es */


#include <math.h>
#include <iostream> 
#include <memory>
#include <string>
#include <map>

#include "bica/Component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "bica_behavior_tree/action/activation_action.hpp"
#include "bica_behavior_tree/control/activation_sequence.hpp"
#include "bica_behavior_tree/BicaTree.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


using namespace BT;
using namespace std::placeholders;

class GetOrder: public bica_behavior_tree::ActivationActionNode
{
public:
  explicit GetOrder(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  BT::NodeStatus tick() override
  {
    std::cerr << "GetOrder " << std::endl;
    return BT::NodeStatus::SUCCESS;

  }

private:
  int counter_;
};



class NavigateToBarman: public bica_behavior_tree::ActivationActionNode, rclcpp::Node
{
public:
  explicit NavigateToBarman(const std::string & name)
  : Node(name), bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
    /*pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&NavigateToBarman::current_pos_callback, this, _1));*/
  }
//
//  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
//  {
//    current_pos_ = msg->pose.pose;
//  }
//
//   double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
//  {
//    return sqrt((pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
//             (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
//  }
//
  
  // void nav(){
  //   //rclcpp::spin_some(this->get_node_base_interface());

  //   std::cerr << "NavigateToBarman " << std::endl;

  //   navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(), "NavigateToPose");

  //   bool is_action_server_ready = false;
  //   do {
  //     is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));

  //   } while (!is_action_server_ready);

  //   auto wp_to_navigate = "";  // ¡¡CAMBIAR ESTO!!
  //   //RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

  //   goal_pos_ = waypoints_[wp_to_navigate];
  //   navigation_goal_.pose = goal_pos_;

  //   dist_to_move = getDistance(goal_pos_.pose, current_pos_);

  //   auto send_goal_options =
  //     rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  //   send_goal_options.result_callback = [this](auto) {feedback_ = 100.0;};

  //   future_navigation_goal_handle_ =
  //     navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

  //   navigation_goal_handle_ = future_navigation_goal_handle_.get();
  //   if (!navigation_goal_handle_) {
  //     RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  //   }
  // }

  BT::NodeStatus tick() override
  {
    // rclcpp::spin_some(this->get_node_base_interface());

    // auto status = navigation_goal_handle_->get_status();

    // // Check if the goal is still executing
    // if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
    //   status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    // {
    //   RCLCPP_DEBUG(get_logger(), "Executing move action");
    // } else {
    //   RCLCPP_WARN(get_logger(), "Error Executing");
    // }

    // double dist_to_goal = getDistance(goal_pos_.pose, current_pos_);

    // feedback_ = 100.0 * (1.0 - (dist_to_goal / dist_to_move));

    // if (feedback_ >= 100.0) {
    //   return BT::NodeStatus::SUCCESS;

    // }else {
    //   return BT::NodeStatus::RUNNING;
    // }
    std::cerr << "NavigateToBarman " << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  
  // using NavigationGoalHandle =
  //   rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  int counter_;
  
  // std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
  // nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  // NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // double dist_to_move;
  // geometry_msgs::msg::Pose current_pos_;
  // geometry_msgs::msg::PoseStamped goal_pos_;
  // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  // std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  // std::string arguments_;

  // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  // //std::shared_ptr<ExecutionAction::Feedback> feedback_;
  // float feedback_;
  
};





class InteractWithBarman : public bica_behavior_tree::ActivationActionNode
{
public:
  explicit InteractWithBarman(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  BT::NodeStatus tick() override
  {
    std::cerr << "InteractWithBarman " << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  int counter_;
};

class CheckOrder: public bica_behavior_tree::ActivationActionNode
{
public:
  explicit CheckOrder(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  BT::NodeStatus tick() override
  {
    std::cerr << "CheckOrder " << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  int counter_;
};


class NavigateToClient: public bica_behavior_tree::ActivationActionNode
{
public:
  explicit NavigateToClient(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  BT::NodeStatus tick() override
  {
    std::cerr << "NavigateToClient " << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  int counter_;
};


class DeliverOrder: public bica_behavior_tree::ActivationActionNode
{
public:
  explicit DeliverOrder(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  BT::NodeStatus tick() override
  {
    std::cerr << "DeliverOrder " << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  int counter_;
};

class Comp : public bica::Component
{
public:
  Comp()
  : bica::Component("behavior_tree", 1), finished_(false)
  {

    factory.registerNodeType<GetOrder>("GetOrder");
    factory.registerNodeType<NavigateToBarman>("NavigateToBarman");
    factory.registerNodeType<InteractWithBarman>("InteractWithBarman");
    factory.registerNodeType<CheckOrder>("CheckOrder");
    factory.registerNodeType<NavigateToClient>("NavigateToClient");
    factory.registerNodeType<DeliverOrder>("DeliverOrder");

    factory.registerFromPlugin(BT::SharedLibrary().getOSName("activation_sequence_bt_node"));
    factory.registerFromPlugin(BT::SharedLibrary().getOSName("activation_action_bt_node"));
  }

  void on_activate()
  {
    std::string pkgpath = ament_index_cpp::get_package_share_directory("pilot_behavior");
    std::string xml_file = pkgpath + "/behavior_trees/bt.xml";

    tree = factory.createTreeFromFile(xml_file);

    tree.configureActivations<GetOrder>("GetOrder", this, {"GetOrder"});
    tree.configureActivations<NavigateToBarman>("NavigateToBarman", this, {"NavigateToBarman"});
    tree.configureActivations<InteractWithBarman>("InteractWithBarman", this, {"InteractWithBarman"});
    tree.configureActivations<CheckOrder>("CheckOrder", this, {"CheckOrder"});
    tree.configureActivations<NavigateToClient>("NavigateToClient", this, {"NavigateToClient"});
    tree.configureActivations<DeliverOrder>("DeliverOrder", this, {"DeliverOrder"});

  }

  void step()
  {
    if (!finished_) {
      finished_ = tree.root_node->executeTick() == BT::NodeStatus::SUCCESS;
    }
  }

private:
 
  bool finished_;
  BT::BehaviorTreeFactory factory;
  bica_behavior_tree::BicaTree tree;
  
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto component = std::make_shared<Comp>();
    
  component->execute();
    
  rclcpp::shutdown();

  return 0;
}