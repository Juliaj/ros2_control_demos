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

#include "ros2_control_demo_example_17/mecanumwheelbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_17
{
hardware_interface::CallbackReturn MecanumWheelBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", 
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Verify command interface type
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' has %s command interface. Expected %s", 
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(), 
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      get_logger(),
      "Registered command interfaces: %s/velocity",
      joint.name.c_str()
    );
  }

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
  // code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumWheelBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    RCLCPP_INFO(get_logger(), "Resetting state for joint: %s", name.c_str());
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    RCLCPP_INFO(get_logger(), "Resetting command for joint: %s", name.c_str());
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumWheelBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumWheelBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumWheelBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  std::stringstream ss;
  ss << "Reading states:";
  ss << std::fixed << std::setprecision(2);
  // iterate through all the joints 
  for (const auto & joint : info_.joints) 
  // for (const auto & [name, descr] : joint_state_interfaces_)
  {
    // RCLCPP_INFO(get_logger(), "))) Reading state velocity for joint: %s", joint.name.c_str());
    // if it is velocity, then read velocity
    if (joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
      auto velo = get_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY);
      set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, velo);
      ss << std::endl
         << "\t velocity " << velo << " for '" << joint.name << "'!";
    }
    else {
      // RCLCPP_INFO(get_logger(), "))) Reading state position for joint: %s", joint.name.c_str());
      auto pos = get_state(joint.name + "/" + hardware_interface::HW_IF_POSITION); 
      set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, pos);
      ss << std::endl
         << "\t position " << pos << " for '" << joint.name << "'!";
    }
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_17 ::MecanumWheelBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    set_state(name, get_command(name));

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << "command " << get_command(name) << " for '" << name << "'!";
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_17

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_17::MecanumWheelBotSystemHardware, hardware_interface::SystemInterface)
