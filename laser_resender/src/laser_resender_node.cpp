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

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Execute:
//  ros2 lifecycle list /laser_resender
//  ros2 lifecycle get /laser_resender
//  ros2 lifecycle set /laser_resender configure

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using namespace std::placeholders;

class LaserResender : public rclcpp_lifecycle::LifecycleNode
{
public:
  LaserResender()
  : rclcpp_lifecycle::LifecycleNode("laser_resender")
  {
    declare_parameter("node_name");
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/mros_scan", rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(), std::bind(&LaserResender::scan_cb, this, _1));
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(
      get_logger(), "[%s] Configuring from [%s] state...",
      get_name(),
      state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(
      get_logger(), "[%s] Activating from [%s] state...",
      get_name(),
      state.label().c_str());
    pub_->on_activate();
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    if (all_zero_error_) {
      return CallbackReturnT::ERROR;
    } else {
      RCLCPP_INFO(
        get_logger(), "[%s] Deactivating from [%s] state...",
        get_name(),
        state.label().c_str());
      return CallbackReturnT::SUCCESS;
    }
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(
      get_logger(), "[%s] Cleanning Up from [%s] state...",
      get_name(),
      state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(
      get_logger(), "[%s] Shutting Down from [%s] state...",
      get_name(),
      state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_ERROR(
      get_logger(), "[%s] Error processing from [%s] state...",
      get_name(),
      state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  void scan_cb(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
  {
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      all_zero_error_ = true;
      for (auto range : laser_scan->ranges) {
        if (range != 0.0) {
          all_zero_error_ = false;
          break;
        }
      }

      if (!all_zero_error_) {
        pub_->publish(*laser_scan);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "[%s] ALL-ZEROS. It has to go to error processing state", get_name());
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }
    }

    /* if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    } */
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  bool all_zero_error_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LaserResender>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
