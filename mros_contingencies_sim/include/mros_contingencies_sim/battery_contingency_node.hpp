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

#ifndef MROS_CONTINGENCY_SIM__BATTERY_CONTINGENCY_HPP_
#define MROS_CONTINGENCY_SIM__BATTERY_CONTINGENCY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

namespace mros_contingencies_sim
{

class BatteryContingency: public rclcpp::Node
{
public:

  BatteryContingency(std::string name);

private:

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  float calculateDistance(
    float current_x, float current_y, float old_x, float old_y);
  void on_error(std::string msg);
  void setOldposition(geometry_msgs::msg::Pose current_pose);
  
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_pub_;
  geometry_msgs::msg::Pose last_pose_;
  float battery_consumption_, battery_level_;
};

}  // namespace mros_contingencies_sim

#endif  // MROS_CONTINGENCY_SIM__BATTERY_CONTINGENCY_HPP_
