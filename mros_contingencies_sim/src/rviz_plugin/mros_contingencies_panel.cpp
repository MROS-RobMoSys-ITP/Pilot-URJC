// Copyright (c) 2020 Intelligent Robotics Lab - Rey Juan Carlos University
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

#include "mros_contingencies_sim/rviz_plugin/mros_contingencies_panel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>

#include "rviz_common/display_context.hpp"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

MROSContingenciesPanel::MROSContingenciesPanel(QWidget * parent)
: Panel(parent), managed_node_("laser_resender")
{
  // Create the control button and its tooltip

  laser_contingency_button_ = new QPushButton;

  // Create the state machine used to present the proper control button states in the UI

  const char * laser_contingency_msg = "Configure and activate all nav2 lifecycle nodes";

  initial_ = new QState();
  initial_->setObjectName("initial");
  initial_->assignProperty(laser_contingency_button_, "text", "Deactive Laser Driver");
  initial_->assignProperty(laser_contingency_button_, "toolTip", laser_contingency_msg);
  initial_->assignProperty(laser_contingency_button_, "enabled", true);

  // State entered when navigate_to_pose action is not active
  idle_ = new QState();
  idle_->setObjectName("idle");
  idle_->assignProperty(laser_contingency_button_, "text", "Deactive Laser Driver");
  idle_->assignProperty(laser_contingency_button_, "enabled", false);

  QObject::connect(initial_, SIGNAL(exited()), this, SLOT(onStartup()));
  // Start/Reset button click transitions
  initial_->addTransition(laser_contingency_button_, SIGNAL(clicked()), idle_);

  state_machine_.addState(initial_);
  state_machine_.addState(idle_);
  state_machine_.setInitialState(initial_);

  state_machine_.start();

  // Lay out the items in the panel
  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(laser_contingency_button_);

  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);

  client_node_ = std::make_shared<rclcpp::Node>("_");

  std::string change_state_service_name = managed_node_ + "/change_state";

  client_laser_ = client_node_->create_client<lifecycle_msgs::srv::ChangeState>(
    change_state_service_name);
}

MROSContingenciesPanel::~MROSContingenciesPanel()
{
}

void
MROSContingenciesPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void
MROSContingenciesPanel::onStartup()
{
  std::cout << "onStartup" << std::endl;
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  auto transition = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  request->transition.id = transition;
  std::chrono::seconds time_out = std::chrono::seconds(5);
  if (!client_laser_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "Service %s is not available.",
      client_laser_->get_service_name());
    return;
  }
  // We send the request with the transition we want to invoke.
  auto future_result = client_laser_->async_send_request(request);
  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(), "Server time out while getting current state for node %s",
      managed_node_.c_str());
    return;
  }
  // We have an answer, let's print our success.
  if (future_result.get()->success) {
    RCLCPP_INFO(
      client_node_->get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
    return;
  } else {
    RCLCPP_WARN(
      client_node_->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    return;
  }
}

void
MROSContingenciesPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void
MROSContingenciesPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::MROSContingenciesPanel, rviz_common::Panel)
