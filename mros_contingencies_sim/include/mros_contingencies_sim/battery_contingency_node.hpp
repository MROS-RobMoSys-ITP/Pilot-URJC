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

#ifndef MROS_CONTINGENCIES_SIM__BATTERY_CONTINGENCY_NODE_HPP_
#define MROS_CONTINGENCIES_SIM__BATTERY_CONTINGENCY_NODE_HPP_

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>

namespace mros_contingencies_sim
{

class BatteryContingency : public rclcpp::Node
{
public:
  explicit BatteryContingency(const std::string & name);

private:
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void timerCallback();
  float calculateDistance(
    float current_x, float current_y, float old_x, float old_y);
  void setOldposition(geometry_msgs::msg::Pose current_pose);
  void publish_diagnostic(std::string key, std::string value);
  void batteryCharged(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr battery_charged_;
  geometry_msgs::msg::Pose last_pose_;
  float current_vel_;
  float distance_;
  float battery_level_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  const float BATTERY_CONSUMPTION = -0.02;
  const float ENERGY_CONSUMPTION_FACTOR = 1.17;
};

}  // namespace mros_contingencies_sim

#endif  // MROS_CONTINGENCIES_SIM__BATTERY_CONTINGENCY_NODE_HPP_
