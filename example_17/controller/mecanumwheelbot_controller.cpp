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

#include "ros2_control_demo_example_17/mecanumwheelbot_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <thread>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace ros2_control_demo_example_17
{ 
MecanumWheelBotController::MecanumWheelBotController() : mecanum_drive_controller::MecanumDriveController() {}

controller_interface::CallbackReturn MecanumWheelBotController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // First, call the parent class configuration
  auto result = mecanum_drive_controller::MecanumDriveController::on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  // Read the async parameter
  try {
    // Get the parameter value
    is_async_ = this->get_node()->get_parameter("is_async").as_bool();
    
    RCLCPP_INFO(this->get_node()->get_logger(), "MecanumWheelbotController operating in %s mode", is_async_ ? "async" : "sync");
      
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Exception thrown during parameter retrieval: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumWheelBotController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // If the controller is async, simulate the delay
  if (is_async_) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> latency_dist(MIN_LATENCY_MS, MAX_LATENCY_MS);
    auto latency = std::chrono::milliseconds(latency_dist(gen));
    
    if (latency.count() >= 25) {
      RCLCPP_INFO(this->get_node()->get_logger(), "Controller update latency is high: %ldms", latency.count());
    }
    std::this_thread::sleep_for(latency);
  }

  // Call the parent implementation
  auto ret = mecanum_drive_controller::MecanumDriveController::update_and_write_commands(time, period);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  
  return controller_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_17

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_17::MecanumWheelBotController, 
  controller_interface::ChainableControllerInterface)
