// Copyright 2025 ros2_control Development Team
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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_17__MECANUMWHEELBOT_CONTROLLER_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_17__MECANUMWHEELBOT_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <random>
#include <thread>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "mecanum_drive_controller/mecanum_drive_controller.hpp"
#include "std_msgs/msg/float32.hpp"
# include <random>

namespace ros2_control_demo_example_17
{
class MecanumWheelBotController : public mecanum_drive_controller::MecanumDriveController
{ 
public:
  MecanumWheelBotController();

  // Add this method to read parameters during configuration
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
protected:
  // add is_active_flag
  bool is_active_flag_ = false;
  
  // custom parameters
  double simulated_delay_min_ = 0.0;
  double simulated_delay_max_ = 0.0;

  // subscribe to the custom parameters
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr simulated_delay_min_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr simulated_delay_max_sub_;

  // callback function for the custom parameters
  void simulated_delay_min_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void simulated_delay_max_callback(const std_msgs::msg::Float32::SharedPtr msg);

  // add trigger_exception_flag
  bool trigger_exception_flag_ = false;

  std::uniform_real_distribution<double> dist_;
  std::uniform_real_distribution<double> latency_dist;

private:
  std::random_device rd_;
  std::mt19937 gen_;
};
}  // namespace ros2_control_demo_example_17

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_17__MECANUMWHEELBOT_CONTROLLER_HPP_
