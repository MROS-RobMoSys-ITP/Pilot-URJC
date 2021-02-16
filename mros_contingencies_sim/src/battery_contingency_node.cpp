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
using namespace std::placeholders;

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
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10), std::bind(
      &BatteryContingency::odomCallback,
      this,
      _1));
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    rclcpp::QoS(10));
  
  std::chrono::milliseconds period(2000); 
  publish_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(
    &BatteryContingency::timerCallback, this));
  battery_charged_= create_service<std_srvs::srv::Empty>("battery_contingency/battery_charged", std::bind(
    &BatteryContingency::batteryCharged, this, _1, _2, _3));
  battery_level_ = 0.7;
  current_vel_= 0.0;
  distance_= 0.0;
  battery_failed_ = false;
  RCLCPP_INFO(this->get_logger(), "BatteryContingency class initialization completed!!");

}
void BatteryContingency::batteryCharged(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  battery_level_ = 1.0;
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

void BatteryContingency::publish_diagnostic(std::string key, 
                                            std::string value,
                                            std::string message)
{
  diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status;
  std::vector<diagnostic_msgs::msg::KeyValue> key_values;
  diagnostic_msgs::msg::DiagnosticStatus status_msg;
  diagnostic_msg.header.stamp = now();

  status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  key_values.push_back(key_value);
  status_msg.values = key_values;
  status_msg.message = message;
  diag_status.push_back(status_msg);
  diagnostic_msg.status = diag_status;
  diagnostics_pub_->publish(diagnostic_msg);
}


void BatteryContingency::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  if (odom_msg->twist.twist.linear.x > current_vel_)
  {
    current_vel_ = odom_msg->twist.twist.linear.x;
  }
  
}


void BatteryContingency::timerCallback()
{
  float energy_comspumtion = current_vel_ * ENERGY_CONSUMPTION_FACTOR;
  publish_diagnostic(std::string("energy"), std::to_string(energy_comspumtion), std::string("QA status"));
  current_vel_ = 0;
  if(battery_level_ < 0.5 && !battery_failed_)
  {
    publish_diagnostic(std::string("battery"), std::string("FALSE"), std::string("Component status"));
    battery_failed_ = true;
  }
  else if(battery_level_ > 0.8 && battery_failed_ )
  {
    publish_diagnostic(std::string("battery"), std::string("RECOVERED"), std::string("Component status"));
    battery_failed_ = false;
  }

}

void BatteryContingency::amclCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  float current_x, current_y;
  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;
  if (last_pose_.position.x != 0.0 && last_pose_.position.x != 0.0) {
    distance_ = calculateDistance(
      current_x, current_y, last_pose_.position.x, last_pose_.position.y);
    setOldposition(msg->pose.pose);
  } else {
    setOldposition(msg->pose.pose);
  }
  battery_level_ = battery_level_ + distance_ * BATTERY_CONSUMPTION;
  RCLCPP_WARN(get_logger(), "battery level %f",battery_level_);
  if (battery_level_ < 0.0) {
    battery_level_ = 0.0;
  }

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
