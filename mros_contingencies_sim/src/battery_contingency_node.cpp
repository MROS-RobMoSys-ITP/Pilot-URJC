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

#include <mros_contingencies_sim/battery_contingency_node.hpp>

#include <string>
#include <vector>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace mros_contingencies_sim
{

BatteryContingency::BatteryContingency(const std::string & name)
: Node(name)
{
  amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", rclcpp::QoS(10), std::bind(
      &BatteryContingency::amclCallback,
      this,
      _1));
  battery_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/robot/battery",
    rclcpp::QoS(10));
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/metacontroller/diagnostics",
    rclcpp::QoS(10));
  battery_level_ = 100.0;
}

float BatteryContingency::calculateDistance(
  float current_x, float current_y, float old_x, float old_y)
{
  double diff_x = current_x - old_x;
  double diff_y = current_y - old_y;
  return sqrt(diff_x * diff_x + diff_y * diff_y);
}

void BatteryContingency::setOldposition(geometry_msgs::msg::Pose current_pose)
{
  last_pose_ = current_pose;
}

void BatteryContingency::publish_diagnostic(std::string value)
{
  diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status;
  std::vector<diagnostic_msgs::msg::KeyValue> key_values;
  diagnostic_msgs::msg::DiagnosticStatus status_msg;
  diagnostic_msg.header.stamp = now();

  status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "energy";
  key_value.value = value;
  key_values.push_back(key_value);
  status_msg.values = key_values;
  status_msg.message = "QA status";
  diag_status.push_back(status_msg);
  diagnostic_msg.status = diag_status;
  diagnostics_pub_->publish(diagnostic_msg);
}

void BatteryContingency::amclCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  float current_x, current_y, distance = 0.0;
  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;
  if (last_pose_.position.x != 0.0 && last_pose_.position.x != 0.0) {
    distance = calculateDistance(
      current_x, current_y, last_pose_.position.x, last_pose_.position.y);
    setOldposition(msg->pose.pose);
  } else {
    setOldposition(msg->pose.pose);
  }

  battery_level_ = battery_level_ - distance * BATTERY_CONSUMPTION;
  if (battery_level_ < 0.0) {
    battery_level_ = 0.0;
  }
  publish_diagnostic(std::to_string(battery_level_));
}

}  // namespace mros_contingencies_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<mros_contingencies_sim::BatteryContingency>("battery_contingency_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
