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

#ifndef MROS_CONTINGENCIES_SIM__RVIZ_PLUGIN__MROS_CONTINGENCIES_PANEL_HPP_
#define MROS_CONTINGENCIES_SIM__RVIZ_PLUGIN__MROS_CONTINGENCIES_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>

#include <memory>
#include <string>
#include <vector>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

class QPushButton;

namespace nav2_rviz_plugins
{

/// Panel to interface to the nav2 stack
class MROSContingenciesPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MROSContingenciesPanel(QWidget * parent = 0);
  virtual ~MROSContingenciesPanel();

  void onInitialize() override;

  /// Load and save configuration data
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onStartup();

private:
  // The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;
  std::string managed_node_;
  // A timer used to check on the completion status of the action
  QBasicTimer timer_;

  // The client used to control the laser driver
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_laser_;

  QPushButton * laser_contingency_button_{nullptr};
  QStateMachine state_machine_;

  QState * initial_{nullptr};
  QState * idle_{nullptr};

};

}  // namespace nav2_rviz_plugins

#endif  //  MROS_CONTINGENCIES_SIM__RVIZ_PLUGIN__MROS_CONTINGENCIES_PANEL_HPP_
