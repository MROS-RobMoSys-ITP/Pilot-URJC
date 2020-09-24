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

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace mros_contingencies_sim
{

BatteryContingency::BatteryContingency(std::string name): Node(name)
{
  amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose",
    rclcpp::QoS(10),
    std::bind(&BatteryContingency::amclCallback,
    this,
    _1));
  battery_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/robot/battery",
    rclcpp::QoS(10));

  battery_level_ = 100.0;
  battery_consumption_ = 2.0;
}

float BatteryContingency::calculateDistance(
  float current_x, float current_y, float old_x, float old_y)
{
  return sqrt((pow(current_x - old_x, 2) + pow(current_y - old_y, 2)));
}

void BatteryContingency::setOldposition(geometry_msgs::msg::Pose current_pose)
{
  last_pose_ = current_pose;
}
  
void BatteryContingency::amclCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  float current_x, current_y, distance;
  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;
  if (last_pose_.position.x != 0.0 && last_pose_.position.x != 0.0)
  {
    distance = calculateDistance(current_x, current_y, last_pose_.position.x, last_pose_.position.y);
    setOldposition(msg->pose.pose);
  }
  else
    setOldposition(msg->pose.pose);
  
  battery_level_ = battery_level_ - distance * battery_consumption_;
  std_msgs::msg::Float64 battery_msg;
  battery_msg.data = battery_level_;
  battery_pub_->publish(battery_msg);
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = 
    std::make_shared<mros_contingencies_sim::BatteryContingency>("battery_contingency_node");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}