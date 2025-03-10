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
MecanumWheelBotController::MecanumWheelBotController() 
: mecanum_drive_controller::MecanumDriveController(),
  gen_(rd_()),
  dist_(0.0, 1.0)  // Range from 0.0 to 1.0
{}

controller_interface::CallbackReturn MecanumWheelBotController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // First, call the parent class configuration
  auto result = mecanum_drive_controller::MecanumDriveController::on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  // Declare parameters here
  constexpr double DEFAULT_DELAY = 0.0;
  this->get_node()->declare_parameter<double>("simulated_delay_min", DEFAULT_DELAY);
  this->get_node()->declare_parameter<double>("simulated_delay_max", DEFAULT_DELAY);
  this->get_node()->declare_parameter<bool>("trigger_exception", false);

  // Read the custom parameters
  try {
    // Get the custom parameters
    simulated_delay_min_ = this->get_node()->get_parameter("simulated_delay_min").as_double();
    simulated_delay_max_ = this->get_node()->get_parameter("simulated_delay_max").as_double();
    RCLCPP_INFO(this->get_node()->get_logger(), "Simulated delay min: %f, max: %f", simulated_delay_min_, simulated_delay_max_);
      
    if (simulated_delay_min_ < 0 || simulated_delay_max_ < 0) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Delay values cannot be negative");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (simulated_delay_min_ > simulated_delay_max_) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Minimum delay cannot be greater than maximum delay");
        return controller_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Exception thrown during parameter retrieval: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  trigger_exception_flag_ = this->get_node()->get_parameter("trigger_exception").as_bool();

  // add callback function for the custom parameters
  simulated_delay_min_sub_ = this->get_node()->create_subscription<std_msgs::msg::Float32>(
    "simulated_delay_min", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
      this->simulated_delay_min_callback(msg);
    });
  simulated_delay_max_sub_ = this->get_node()->create_subscription<std_msgs::msg::Float32>(
    "simulated_delay_max", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
      this->simulated_delay_max_callback(msg);
    });

  RCLCPP_INFO(this->get_node()->get_logger(), "MecanumwWheelBotController configured successfully.");
  return controller_interface::CallbackReturn::SUCCESS;
}

// add callback function for the custom parameters
void MecanumWheelBotController::simulated_delay_min_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  simulated_delay_min_ = static_cast<double>(msg->data);
}

void MecanumWheelBotController::simulated_delay_max_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  simulated_delay_max_ = static_cast<double>(msg->data);
}

controller_interface::return_type MecanumWheelBotController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Simulate the delay if the custom parameters are set
  if (simulated_delay_min_ > 0.0 || simulated_delay_max_ > 0.0) {
    std::uniform_int_distribution<> latency_dist(simulated_delay_min_, simulated_delay_max_);
    auto latency = std::chrono::milliseconds(latency_dist(gen_));
    
    if (latency.count() == simulated_delay_max_) {
      RCLCPP_INFO(this->get_node()->get_logger(), "Added high delay: %ldms. Delay is at max", latency.count());
    }
    std::this_thread::sleep_for(latency);
  }

  // Call the parent implementation
  auto ret = mecanum_drive_controller::MecanumDriveController::update_and_write_commands(time, period);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  
  // Trigger exception if the flag is set randomly
  if (trigger_exception_flag_) {
    // Generate a random number between 0.0 and 1.0
    double random_value_for_exception = dist_(gen_);
    
    // Set a threshold for triggering the exception (e.g., 0.01 = 1% chance)
    const double EXCEPTION_THRESHOLD = 0.01;
    
    if (random_value_for_exception < EXCEPTION_THRESHOLD) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Triggering exception (random value: %f)", random_value_for_exception  );
        throw std::runtime_error("Exception triggered randomly");
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_17

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_17::MecanumWheelBotController, 
  controller_interface::ChainableControllerInterface)
