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
//
// Author: Julia Jia

#include <algorithm>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"

#include "motion_controller/motion_controller.hpp"

namespace motion_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

CallbackReturn MotionController::on_init()
{
  try
  {
    // Declare parameters
    get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
    get_node()->declare_parameter<std::string>("model_path", "");
    get_node()->declare_parameter<int>("model_input_size", 0);
    get_node()->declare_parameter<int>("model_output_size", 0);
    get_node()->declare_parameter<std::string>(
      "interfaces_broadcaster_topic", "state_interfaces_broadcaster/values");
    get_node()->declare_parameter<std::string>(
      "interfaces_broadcaster_names_topic", "state_interfaces_broadcaster/names");
    get_node()->declare_parameter<std::string>("velocity_command_topic", "~/cmd_vel");
    get_node()->declare_parameter<std::vector<double>>(
      "joint_position_limits_min", std::vector<double>());
    get_node()->declare_parameter<std::vector<double>>(
      "joint_position_limits_max", std::vector<double>());
    get_node()->declare_parameter<double>("action_scale", 0.25);
    get_node()->declare_parameter<std::vector<double>>(
      "default_joint_positions", std::vector<double>());
    get_node()->declare_parameter<std::string>("imu_sensor_name", "imu_2");
    get_node()->declare_parameter<std::vector<int>>(
      "head_joint_indices", std::vector<int>{5, 6, 7, 8});
    get_node()->declare_parameter<std::string>("feet_contact_topic", "~/feet_contacts");
    get_node()->declare_parameter<bool>("start_paused", false);
    get_node()->declare_parameter<bool>("imu_upside_down", false);
    get_node()->declare_parameter<double>("phase_frequency_factor_offset", 0.0);
    get_node()->declare_parameter<double>("phase_period", 100.0);
    get_node()->declare_parameter<double>("cutoff_frequency", 0.0);
    get_node()->declare_parameter<double>("max_motor_velocity", 5.24);
    get_node()->declare_parameter<double>("reference_motion_blend_factor", 0.2);
    get_node()->declare_parameter<double>("training_control_period", 0.02);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  model_loaded_ = false;
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration MotionController::command_interface_configuration() const
{
  InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

InterfaceConfiguration MotionController::state_interface_configuration() const
{
  // This controller doesn't need state interfaces - it subscribes to topics
  return InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

CallbackReturn MotionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Read parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  // Get model path and expand $(find-pkg-share ...) substitution
  std::string raw_model_path = get_node()->get_parameter("model_path").as_string();
  std::regex pkg_share_regex(R"(\$\(find-pkg-share\s+([^\)]+)\))");
  std::smatch match;
  if (std::regex_search(raw_model_path, match, pkg_share_regex))
  {
    std::string package_name = match[1].str();
    std::string package_share = ament_index_cpp::get_package_share_directory(package_name);
    model_path_ = std::regex_replace(raw_model_path, pkg_share_regex, package_share);
  }
  else
  {
    model_path_ = raw_model_path;
  }
  interfaces_broadcaster_topic_ =
    get_node()->get_parameter("interfaces_broadcaster_topic").as_string();
  interfaces_broadcaster_names_topic_ =
    get_node()->get_parameter("interfaces_broadcaster_names_topic").as_string();
  velocity_command_topic_ = get_node()->get_parameter("velocity_command_topic").as_string();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return CallbackReturn::ERROR;
  }

  if (model_path_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model path not specified");
    return CallbackReturn::ERROR;
  }

  // Read model input/output sizes for validation
  model_input_size_ = static_cast<int>(get_node()->get_parameter("model_input_size").as_int());
  model_output_size_ = static_cast<int>(get_node()->get_parameter("model_output_size").as_int());
  if (model_input_size_ > 0)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Model input size (from config): %d", model_input_size_);
  }
  if (model_output_size_ > 0)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Model output size (from config): %d", model_output_size_);
  }

  // Build command interface names: joint_name/position for each joint
  command_interface_names_.clear();
  for (const auto & joint_name : joint_names_)
  {
    command_interface_names_.push_back(joint_name + "/position");
  }

  // TODO(juliaj): Check whteher this is valid.
  // Initialize previous action to zeros
  previous_action_.resize(joint_names_.size(), 0.0);

  update_count_ = 0;

  // Read IMU sensor name parameter
  std::string imu_sensor_name = get_node()->get_parameter("imu_sensor_name").as_string();

  // Read Open Duck Mini specific parameters
  bool imu_upside_down = get_node()->get_parameter("imu_upside_down").as_bool();
  phase_frequency_factor_offset_ =
    get_node()->get_parameter("phase_frequency_factor_offset").as_double();
  phase_period_ = get_node()->get_parameter("phase_period").as_double();

  // Initialize observation formatter
  observation_formatter_ = std::make_unique<ObservationFormatter>(joint_names_, imu_sensor_name);
  observation_formatter_->set_phase_period(phase_period_);
  observation_formatter_->set_imu_upside_down(imu_upside_down);
  
  RCLCPP_INFO(
    get_node()->get_logger(),
    "IMU configuration: sensor_name='%s', imu_upside_down=%s (z-axis will be %s)",
    imu_sensor_name.c_str(), imu_upside_down ? "true" : "false",
    imu_upside_down ? "inverted" : "not inverted");

  // Read action_scale parameter
  double action_scale = get_node()->get_parameter("action_scale").as_double();

  // Initialize action processor with configurable scale
  action_processor_ = std::make_unique<ActionProcessor>(joint_names_, action_scale, true);

  // Configure head joint indices for head command processing
  std::vector<long int> head_joint_indices_long =
    get_node()->get_parameter("head_joint_indices").as_integer_array();
  std::vector<size_t> head_joint_indices;
  for (long int idx : head_joint_indices_long)
  {
    head_joint_indices.push_back(static_cast<size_t>(idx));
  }
  action_processor_->set_head_joint_indices(head_joint_indices);

  // Read default_joint_positions parameter (optional)
  std::vector<double> param_default_positions =
    get_node()->get_parameter("default_joint_positions").as_double_array();

  // Initialize default joint positions
  default_joint_positions_.resize(joint_names_.size(), 0.0);
  if (param_default_positions.size() == joint_names_.size())
  {
    // Use positions from config
    default_joint_positions_ = param_default_positions;
    observation_formatter_->set_default_joint_positions(default_joint_positions_);
    default_joint_positions_initialized_ = true;
    
    RCLCPP_INFO(get_node()->get_logger(),
      "\n[INIT] Default joint positions loaded from CONFIG (14 joints):");
    for (size_t i = 0; i < std::min(default_joint_positions_.size(), size_t(14)); ++i)
    {
      RCLCPP_INFO(get_node()->get_logger(), "  Joint %zu (%s): %.4f rad", 
        i, i < joint_names_.size() ? joint_names_[i].c_str() : "unknown", default_joint_positions_[i]);
    }
  }
  else
  {
    // Will be set from sensor data on first update
    default_joint_positions_initialized_ = false;
    RCLCPP_WARN(get_node()->get_logger(),
      "default_joint_positions parameter size (%zu) != joint count (%zu). Will read from sensors on first update.",
      param_default_positions.size(), joint_names_.size());
  }

  // Initialize joint position limits
  // Try to read from parameters first, otherwise use defaults from ros2_control config
  std::vector<double> param_limits_min =
    get_node()->get_parameter("joint_position_limits_min").as_double_array();
  std::vector<double> param_limits_max =
    get_node()->get_parameter("joint_position_limits_max").as_double_array();

  if (
    param_limits_min.size() == joint_names_.size() &&
    param_limits_max.size() == joint_names_.size())
  {
    joint_position_limits_min_ = param_limits_min;
    joint_position_limits_max_ = param_limits_max;
    RCLCPP_INFO(get_node()->get_logger(), "Using joint limits from parameters");
  }
  else
  {
    // Use defaults from ros2_control config
    joint_position_limits_min_.resize(
      joint_names_.size(), -std::numeric_limits<double>::infinity());
    joint_position_limits_max_.resize(joint_names_.size(), std::numeric_limits<double>::infinity());
    initialize_joint_limits();
    RCLCPP_INFO(get_node()->get_logger(), "Using default joint limits from ros2_control config");
  }

  // Read motor speed limit parameter
  max_motor_velocity_ = get_node()->get_parameter("max_motor_velocity").as_double();

  // Read reference motion blend factor parameter
  reference_motion_blend_factor_ = get_node()->get_parameter("reference_motion_blend_factor").as_double();
  reference_motion_blend_factor_ = std::clamp(reference_motion_blend_factor_, 0.0, 1.0);

  // Read training control period parameter
  training_control_period_ = get_node()->get_parameter("training_control_period").as_double();
  if (training_control_period_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid training_control_period (%.6f). Must be positive. Using default 0.02s.",
      training_control_period_);
    training_control_period_ = 0.02;
  }

  // Initialize motor targets and previous motor targets to default_joint_positions
  // Reference: validate_onnx_simulation.py lines 124-125: motor_targets = default_actuator, prev_motor_targets = default_actuator
  motor_targets_.resize(joint_names_.size(), 0.0);
  prev_motor_targets_.resize(joint_names_.size(), 0.0);
  if (default_joint_positions_initialized_)
  {
    // Initialize from default_joint_positions (same as default_actuator in reference)
    motor_targets_ = default_joint_positions_;
    prev_motor_targets_ = default_joint_positions_;
    prev_motor_targets_initialized_ = true;
    // Set initial motor targets in observation formatter
    observation_formatter_->set_motor_targets(motor_targets_);
  }
  else
  {
    // Will be initialized from default_joint_positions on first update
    prev_motor_targets_initialized_ = false;
  }
  command_received_ = false;
  smoothed_reference_action_.resize(joint_names_.size(), 0.0);

  // Load model
  if (!load_model(model_path_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load model from: %s", model_path_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Create subscribers
  interface_data_subscriber_ = get_node()->create_subscription<control_msgs::msg::Float64Values>(
    interfaces_broadcaster_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const control_msgs::msg::Float64Values::SharedPtr msg)
    { rt_interface_data_.set(*msg); });

  interfaces_names_subscriber_ =
    get_node()->create_subscription<control_msgs::msg::Keys>(
      interfaces_broadcaster_names_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      [this](const control_msgs::msg::Keys::SharedPtr msg)
      { rt_interface_names_.set(msg->keys); });

  velocity_command_subscriber_ =
    get_node()->create_subscription<example_18_motion_controller_msgs::msg::VelocityCommandWithHead>(
      velocity_command_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const example_18_motion_controller_msgs::msg::VelocityCommandWithHead::SharedPtr msg)
      { rt_velocity_command_.set(*msg); });

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!model_loaded_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model not loaded");
    return CallbackReturn::ERROR;
  }

  // Validate command interfaces
  if (command_interfaces_.size() != command_interface_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), command_interfaces_.size());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

return_type MotionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Get latest interface data from thread-safe buffer
  auto interface_data_op = rt_interface_data_.try_get();
  if (!interface_data_op.has_value())
  {
    return return_type::OK;
  }
  control_msgs::msg::Float64Values interface_data = interface_data_op.value();

  // Get latest velocity command from thread-safe buffer
  auto velocity_cmd_op = rt_velocity_command_.try_get();
  example_18_motion_controller_msgs::msg::VelocityCommandWithHead velocity_cmd;
  if (velocity_cmd_op.has_value())
  {
    velocity_cmd = velocity_cmd_op.value();
    command_received_ = true;
  }

  // Format velocity commands for Open Duck Mini (7D: 3 base + 4 head)
  std::vector<double> velocity_commands_7d = {
    velocity_cmd.base_velocity.linear.x,   // lin_vel_x
    velocity_cmd.base_velocity.linear.y,    // lin_vel_y
    velocity_cmd.base_velocity.angular.z,   // ang_vel_z
    velocity_cmd.head_commands.size() > 0 ? velocity_cmd.head_commands[0] : 0.0,  // head_pos_1 (neck_pitch)
    velocity_cmd.head_commands.size() > 1 ? velocity_cmd.head_commands[1] : 0.0,  // head_pos_2 (head_pitch)
    velocity_cmd.head_commands.size() > 2 ? velocity_cmd.head_commands[2] : 0.0,  // head_pos_3 (head_yaw)
    velocity_cmd.head_commands.size() > 3 ? velocity_cmd.head_commands[3] : 0.0   // head_pos_4 (head_roll)
  };
  observation_formatter_->set_velocity_commands(velocity_commands_7d);

  // Refresh interface name mapping when broadcaster publishes it
  if (auto names_op = rt_interface_names_.try_get(); names_op.has_value())
  {
    observation_formatter_->set_interface_names(names_op.value());
  }

  // Initialize default joint positions from current sensor data (first update only)
  if (!default_joint_positions_initialized_)
  {
    default_joint_positions_ = observation_formatter_->extract_joint_positions(interface_data);
    observation_formatter_->set_default_joint_positions(default_joint_positions_);
    default_joint_positions_initialized_ = true;

    // Log default positions for debugging
    RCLCPP_INFO(get_node()->get_logger(),
      "\n[INIT] Default joint positions set from SENSORS (14 joints):");
    for (size_t i = 0; i < std::min(default_joint_positions_.size(), size_t(14)); ++i)
    {
      RCLCPP_INFO(get_node()->get_logger(), "  Joint %zu (%s): %.4f rad", 
        i, i < joint_names_.size() ? joint_names_[i].c_str() : "unknown", default_joint_positions_[i]);
    }

    // Initialize motor_targets and prev_motor_targets to default_joint_positions
    // Reference: validate_onnx_simulation.py lines 124-125
    motor_targets_ = default_joint_positions_;
    prev_motor_targets_ = default_joint_positions_;
    prev_motor_targets_initialized_ = true;
    observation_formatter_->set_motor_targets(motor_targets_);
  }

  // Don't process policy until at least one velocity command has been received
  // But still write motor_targets to maintain default pose
  if (!command_received_)
  {
    if (!prev_motor_targets_initialized_ && default_joint_positions_initialized_)
    {
      motor_targets_ = default_joint_positions_;
      prev_motor_targets_ = default_joint_positions_;
      prev_motor_targets_initialized_ = true;
      observation_formatter_->set_motor_targets(motor_targets_);
    }
    
    // Write motor_targets to hardware to maintain default pose even without command
    if (prev_motor_targets_initialized_ && command_interfaces_.size() == motor_targets_.size())
    {
      for (size_t i = 0; i < command_interfaces_.size() && i < motor_targets_.size(); ++i)
      {
        (void)command_interfaces_[i].set_value(motor_targets_[i]);
      }
    }
    return return_type::OK;
  }

  // Update imitation phase (gait phase encoding)
  // Reference: validate_onnx_simulation.py lines 137-144
  const double actual_control_period = period.seconds();
  if (std::abs(actual_control_period - training_control_period_) > 0.001)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Controller update period (%.4f s) differs from training period (%.4f s). "
      "Consider setting update_rate to %.1f Hz.",
      actual_control_period, training_control_period_, 1.0 / training_control_period_);
  }
  double phase_freq_factor = 1.0 + phase_frequency_factor_offset_;
  observation_formatter_->update_imitation_phase(phase_freq_factor);

  // Get feet contacts from real sensors if available, otherwise estimate from gait phase
  double left_contact = 0.0;
  double right_contact = 0.0;
  bool sensors_available = observation_formatter_->extract_feet_contacts(
    interface_data, left_contact, right_contact);

  if (!sensors_available)
  {
    // Fall back to phase-based estimation
    auto phase = observation_formatter_->get_imitation_phase();
    double sin_phase = phase[1];
    left_contact = (sin_phase > 0.0) ? 1.0 : 0.0;
    right_contact = (sin_phase < 0.0) ? 1.0 : 0.0;
  }

  observation_formatter_->set_feet_contacts(left_contact, right_contact);

  // Set motor_targets BEFORE building observation (reference: validate_onnx_simulation.py line 156)
  observation_formatter_->set_motor_targets(motor_targets_);

  // Format inputs for model using ObservationFormatter
  std::vector<float> model_inputs;
  try
  {
    model_inputs = observation_formatter_->format(interface_data, velocity_cmd.base_velocity, previous_action_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to format observation: %s", e.what());
    return return_type::ERROR;
  }

  size_t expected_dim = observation_formatter_->get_observation_dim();
  if (model_inputs.size() != expected_dim)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Observation size mismatch: got %zu, expected %zu", model_inputs.size(), expected_dim);
    return return_type::ERROR;
  }

  // Log full observation breakdown matching compare_observations.py format
  // Reference: compare_observations.py lines 36-47
  // Log first 5 steps and every 25th step for detailed comparison
  if (update_count_ % 25 == 0 || update_count_ < 5)
  {
    // Get current joint positions for comparison
    std::vector<double> current_joint_pos = observation_formatter_->extract_joint_positions(interface_data);
    
    auto phase = observation_formatter_->get_imitation_phase();
    double imitation_i = observation_formatter_->get_imitation_i();
    double phase_period = observation_formatter_->get_phase_period();
    
    // Detailed observation logging matching Python format exactly
    RCLCPP_INFO(
      get_node()->get_logger(),
      "\n[ROS2 OBS] Full observation (101 dims) - Step %zu:\n"
      "  gyro=[%.4f,%.4f,%.4f] (indices 0-2)\n"
      "  accel=[%.4f,%.4f,%.4f] (x+1.3=%.4f) (indices 3-5)\n"
      "  command=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] (indices 6-12)\n"
      "  joint_pos[ALL 14]=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] (ABSOLUTE)\n"
      "  joint_pos_rel[ALL 14]=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] (indices 13-26)\n"
      "  joint_vel[ALL 14]=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] (indices 27-40)\n"
      "  last_action[ALL 14]=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] (indices 41-54)\n"
      "  last_last_action[ALL 14]=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] (indices 55-68)\n"
      "  last_last_last_action[ALL 14]=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] (indices 69-82)\n"
      "  motor_targets[ALL 14]=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] (indices 83-96)\n"
      "  contacts=[%.4f,%.4f] (indices 97-98)\n"
      "  phase=[%.4f,%.4f] (indices 99-100)\n"
      "  imitation_i=%.1f, phase_period=%.1f",
      update_count_ + 1,
      // Gyro (0-2)
      model_inputs[0], model_inputs[1], model_inputs[2],
      // Accel (3-5)
      model_inputs[3], model_inputs[4], model_inputs[5], model_inputs[3],
      // Commands (6-12)
      model_inputs[6], model_inputs[7], model_inputs[8], model_inputs[9], model_inputs[10], model_inputs[11], model_inputs[12],
      // Absolute joint positions (all 14)
      current_joint_pos.size() > 0 ? current_joint_pos[0] : 0.0, current_joint_pos.size() > 1 ? current_joint_pos[1] : 0.0,
      current_joint_pos.size() > 2 ? current_joint_pos[2] : 0.0, current_joint_pos.size() > 3 ? current_joint_pos[3] : 0.0,
      current_joint_pos.size() > 4 ? current_joint_pos[4] : 0.0, current_joint_pos.size() > 5 ? current_joint_pos[5] : 0.0,
      current_joint_pos.size() > 6 ? current_joint_pos[6] : 0.0, current_joint_pos.size() > 7 ? current_joint_pos[7] : 0.0,
      current_joint_pos.size() > 8 ? current_joint_pos[8] : 0.0, current_joint_pos.size() > 9 ? current_joint_pos[9] : 0.0,
      current_joint_pos.size() > 10 ? current_joint_pos[10] : 0.0, current_joint_pos.size() > 11 ? current_joint_pos[11] : 0.0,
      current_joint_pos.size() > 12 ? current_joint_pos[12] : 0.0, current_joint_pos.size() > 13 ? current_joint_pos[13] : 0.0,
      // Joint positions relative (13-26)
      model_inputs[13], model_inputs[14], model_inputs[15], model_inputs[16], model_inputs[17], model_inputs[18],
      model_inputs[19], model_inputs[20], model_inputs[21], model_inputs[22], model_inputs[23], model_inputs[24],
      model_inputs[25], model_inputs[26],
      // Joint velocities (27-40)
      model_inputs[27], model_inputs[28], model_inputs[29], model_inputs[30], model_inputs[31], model_inputs[32],
      model_inputs[33], model_inputs[34], model_inputs[35], model_inputs[36], model_inputs[37], model_inputs[38],
      model_inputs[39], model_inputs[40],
      // Last action (41-54)
      model_inputs[41], model_inputs[42], model_inputs[43], model_inputs[44], model_inputs[45], model_inputs[46],
      model_inputs[47], model_inputs[48], model_inputs[49], model_inputs[50], model_inputs[51], model_inputs[52],
      model_inputs[53], model_inputs[54],
      // Last last action (55-68)
      model_inputs[55], model_inputs[56], model_inputs[57], model_inputs[58], model_inputs[59], model_inputs[60],
      model_inputs[61], model_inputs[62], model_inputs[63], model_inputs[64], model_inputs[65], model_inputs[66],
      model_inputs[67], model_inputs[68],
      // Last last last action (69-82)
      model_inputs[69], model_inputs[70], model_inputs[71], model_inputs[72], model_inputs[73], model_inputs[74],
      model_inputs[75], model_inputs[76], model_inputs[77], model_inputs[78], model_inputs[79], model_inputs[80],
      model_inputs[81], model_inputs[82],
      // Motor targets (83-96)
      model_inputs[83], model_inputs[84], model_inputs[85], model_inputs[86], model_inputs[87], model_inputs[88],
      model_inputs[89], model_inputs[90], model_inputs[91], model_inputs[92], model_inputs[93], model_inputs[94],
      model_inputs[95], model_inputs[96],
      // Contacts (97-98)
      model_inputs[97], model_inputs[98],
      // Phase (99-100)
      model_inputs[99], model_inputs[100],
      imitation_i, phase_period);
    
    // Summary statistics for quick comparison
    double max_joint_pos_rel = 0.0;
    double max_joint_vel = 0.0;
    for (size_t i = 13; i <= 26 && i < model_inputs.size(); ++i)
    {
      max_joint_pos_rel = std::max(max_joint_pos_rel, std::abs(static_cast<double>(model_inputs[i])));
    }
    for (size_t i = 27; i <= 40 && i < model_inputs.size(); ++i)
    {
      max_joint_vel = std::max(max_joint_vel, std::abs(static_cast<double>(model_inputs[i])));
    }
    
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[OBS SUMMARY] Step %zu: max_joint_pos_rel=%.4f, max_joint_vel=%.4f, contacts=[%.1f,%.1f], phase=[%.4f,%.4f]",
      update_count_ + 1, max_joint_pos_rel, max_joint_vel, model_inputs[97], model_inputs[98], model_inputs[99], model_inputs[100]);
  }

  // Run model inference (returns relative joint positions)
  std::vector<double> model_outputs = run_model_inference(model_inputs);

  // Process model outputs: apply scaling and default offset to get absolute positions
  std::vector<double> joint_commands = action_processor_->process(model_outputs, default_joint_positions_);

  // Blend RL actions with reference motion for stability
  if (reference_motion_blend_factor_ > 0.0 && update_count_ > 0)
  {
    const double smoothing_alpha = 0.1;
    for (size_t i = 0; i < joint_commands.size() && i < smoothed_reference_action_.size(); ++i)
    {
      smoothed_reference_action_[i] = 
        smoothing_alpha * joint_commands[i] + (1.0 - smoothing_alpha) * smoothed_reference_action_[i];
      joint_commands[i] = 
        (1.0 - reference_motion_blend_factor_) * joint_commands[i] +
        reference_motion_blend_factor_ * smoothed_reference_action_[i];
    }
  }
  else if (update_count_ == 0)
  {
    smoothed_reference_action_ = joint_commands;
  }

  // Store current action as previous for next iteration
  previous_action_ = model_outputs;

  // Store original commands before clamping to detect clipping
  std::vector<double> original_joint_commands = joint_commands;
  std::vector<bool> clipped_by_velocity(joint_commands.size(), false);

  // Apply motor speed limits: prevent exceeding physical motor velocity capability
  // Reference: validate_onnx_simulation.py lines 221-226
  // Note: Python code does NOT apply joint position limits, only motor velocity limits
  const double max_change = prev_motor_targets_initialized_ ? max_motor_velocity_ * actual_control_period : 0.0;
  if (prev_motor_targets_initialized_)
  {
    for (size_t i = 0; i < joint_commands.size(); ++i)
    {
      const double original = joint_commands[i];
      // Clamp joint command to prevent exceeding motor velocity limit
      joint_commands[i] = std::clamp(
        joint_commands[i],
        prev_motor_targets_[i] - max_change,
        prev_motor_targets_[i] + max_change
      );
      if (joint_commands[i] != original)
      {
        clipped_by_velocity[i] = true;
      }
    }
  }
  
  // Log clipping events (throttled to avoid spam)
  static int clip_log_counter = 0;
  if (++clip_log_counter % 25 == 0)  // Log every 25 updates
  {
    bool any_velocity_clip = false;
    for (size_t i = 0; i < clipped_by_velocity.size(); ++i)
    {
      if (clipped_by_velocity[i])
      {
        any_velocity_clip = true;
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "[CLIP] Joint '%s' clipped by VELOCITY limit: requested=%.4f, clamped=%.4f, prev=%.4f, max_change=%.4f",
          i < joint_names_.size() ? joint_names_[i].c_str() : "unknown",
          original_joint_commands[i], joint_commands[i], 
          prev_motor_targets_initialized_ ? prev_motor_targets_[i] : 0.0,
          max_change);
      }
    }
    if (!any_velocity_clip && update_count_ % 125 == 0)  // Less frequent "all OK" message
    {
      RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "[CLIP] No joint commands clipped by velocity limit (step %zu)", update_count_ + 1);
    }
  }
  
  // Update motor_targets and prev_motor_targets after clamping
  // Reference: validate_onnx_simulation.py line 226: prev_motor_targets = motor_targets.copy()
  motor_targets_ = joint_commands;
  prev_motor_targets_ = joint_commands;

  // Log step-by-step information every 25 updates (matching compare_observations.py line 83)
  // Reference: compare_observations.py lines 84-88
  if (update_count_ % 25 == 0)
  {
    // Extract contact values from model_inputs (indices 97-98 based on ObservationFormatter)
    double contact_left_debug = model_inputs.size() > 97 ? model_inputs[97] : -1.0;
    double contact_right_debug = model_inputs.size() > 98 ? model_inputs[98] : -1.0;
    
    RCLCPP_INFO(
      get_node()->get_logger(),
      "\n[STEP %zu] Policy update #%zu\n"
      "  Command: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n"
      "  Contacts: L=%.1f R=%.1f\n"
      "  Action (all 14): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n"
      "  Motor targets (all 14): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n"
      "  Max velocity change allowed: %.4f rad (over %.4f s)",
      update_count_ + 1, update_count_ + 1,
      velocity_commands_7d[0], velocity_commands_7d[1], velocity_commands_7d[2],
      velocity_commands_7d[3], velocity_commands_7d[4], velocity_commands_7d[5], velocity_commands_7d[6],
      contact_left_debug, contact_right_debug,
      // All 14 actions (model_outputs)
      model_outputs.size() > 0 ? model_outputs[0] : 0.0, model_outputs.size() > 1 ? model_outputs[1] : 0.0,
      model_outputs.size() > 2 ? model_outputs[2] : 0.0, model_outputs.size() > 3 ? model_outputs[3] : 0.0,
      model_outputs.size() > 4 ? model_outputs[4] : 0.0, model_outputs.size() > 5 ? model_outputs[5] : 0.0,
      model_outputs.size() > 6 ? model_outputs[6] : 0.0, model_outputs.size() > 7 ? model_outputs[7] : 0.0,
      model_outputs.size() > 8 ? model_outputs[8] : 0.0, model_outputs.size() > 9 ? model_outputs[9] : 0.0,
      model_outputs.size() > 10 ? model_outputs[10] : 0.0, model_outputs.size() > 11 ? model_outputs[11] : 0.0,
      model_outputs.size() > 12 ? model_outputs[12] : 0.0, model_outputs.size() > 13 ? model_outputs[13] : 0.0,
      // All 14 motor targets (joint_commands after processing)
      motor_targets_.size() > 0 ? motor_targets_[0] : 0.0, motor_targets_.size() > 1 ? motor_targets_[1] : 0.0,
      motor_targets_.size() > 2 ? motor_targets_[2] : 0.0, motor_targets_.size() > 3 ? motor_targets_[3] : 0.0,
      motor_targets_.size() > 4 ? motor_targets_[4] : 0.0, motor_targets_.size() > 5 ? motor_targets_[5] : 0.0,
      motor_targets_.size() > 6 ? motor_targets_[6] : 0.0, motor_targets_.size() > 7 ? motor_targets_[7] : 0.0,
      motor_targets_.size() > 8 ? motor_targets_[8] : 0.0, motor_targets_.size() > 9 ? motor_targets_[9] : 0.0,
      motor_targets_.size() > 10 ? motor_targets_[10] : 0.0, motor_targets_.size() > 11 ? motor_targets_[11] : 0.0,
      motor_targets_.size() > 12 ? motor_targets_[12] : 0.0, motor_targets_.size() > 13 ? motor_targets_[13] : 0.0,
      max_motor_velocity_ * actual_control_period, actual_control_period);
  }

  // Write joint commands to hardware interfaces
  // Reference: validate_onnx_simulation.py line 228/273: self.data.ctrl = self.motor_targets.copy()
  // Note: Python reference does not apply joint position limits, only motor velocity limits
  size_t write_success_count = 0;
  for (size_t i = 0; i < command_interfaces_.size() && i < joint_commands.size(); ++i)
  {
    const bool write_success = command_interfaces_[i].set_value(joint_commands[i]);
    if (!write_success)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set command for joint '%s' to %.6f",
        joint_names_[i].c_str(), joint_commands[i]);
    }
    else
    {
      write_success_count++;
    }
  }

  // Log command write status every 100 updates
  if (update_count_ % 100 == 0)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[CMD WRITE] Successfully wrote %zu/%zu joint commands",
      write_success_count, command_interfaces_.size());
  }

  update_count_++;

  return return_type::OK;
}

void MotionController::initialize_joint_limits()
{
  // POTENTIAL DEAD CODE: This function is not used since we removed joint position limit clamping
  // to match Python reference behavior (validate_onnx_simulation.py does not apply joint limits).
  // The Python script only applies motor velocity limits, not joint position limits.
  // This function can be removed if joint limits are not needed for safety/hardware protection.
  //
  // Initialize joint limits based on ros2_control config values
  // These match the limits defined in open_duck_mini.ros2_control.xacro
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const std::string & joint_name = joint_names_[i];

    // Set limits based on joint name (matching ros2_control config)
    // Source: open_duck_mini.ros2_control.xacro
    // Note: Joint names don't have "_joint" suffix in ros2_control config
    if (joint_name == "left_hip_yaw" || joint_name == "right_hip_yaw")
    {
      joint_position_limits_min_[i] = -0.52359877559829881566;  // -30 degrees
      joint_position_limits_max_[i] = 0.52359877559829881566;   // +30 degrees
    }
    else if (joint_name == "left_hip_roll" || joint_name == "right_hip_roll")
    {
      joint_position_limits_min_[i] = -0.43633231299858238339;  // -25 degrees
      joint_position_limits_max_[i] = 0.43633231299858238339;   // +25 degrees
    }
    else if (joint_name == "left_hip_pitch")
    {
      joint_position_limits_min_[i] = -1.2217304763960306069;  // -70 degrees
      joint_position_limits_max_[i] = 0.52359877559829881566;  // +30 degrees
    }
    else if (joint_name == "right_hip_pitch")
    {
      joint_position_limits_min_[i] = -0.52359877559829881566;  // -30 degrees
      joint_position_limits_max_[i] = 1.2217304763960306069;    // +70 degrees
    }
    else if (joint_name == "left_knee" || joint_name == "right_knee")
    {
      joint_position_limits_min_[i] = -1.570796326794896558;  // -90 degrees
      joint_position_limits_max_[i] = 1.570796326794896558;   // +90 degrees
    }
    else if (joint_name == "left_ankle" || joint_name == "right_ankle")
    {
      joint_position_limits_min_[i] = -1.570796326794896558;  // -90 degrees
      joint_position_limits_max_[i] = 1.570796326794896558;   // +90 degrees
    }
    else if (joint_name == "neck_pitch")
    {
      joint_position_limits_min_[i] = -0.3490658503988437;  // ~-20 degrees
      joint_position_limits_max_[i] = 1.1344640137963364;   // ~65 degrees
    }
    else if (joint_name == "head_pitch")
    {
      joint_position_limits_min_[i] = -0.785398163397448279;  // -45 degrees
      joint_position_limits_max_[i] = 0.785398163397448279;   // +45 degrees
    }
    else if (joint_name == "head_yaw")
    {
      joint_position_limits_min_[i] = -2.792526803190927;  // ~-160 degrees
      joint_position_limits_max_[i] = 2.792526803190927;   // ~+160 degrees
    }
    else if (joint_name == "head_roll")
    {
      joint_position_limits_min_[i] = -0.523598775598218;  // ~-30 degrees
      joint_position_limits_max_[i] = 0.5235987755983796;   // ~+30 degrees
    }
    // For unknown joints, use no limits (already set to +/- infinity)
  }
}


bool MotionController::load_model(const std::string & model_path)
{
#ifdef ONNXRUNTIME_FOUND
  try
  {
    // Initialize ONNX Runtime environment
    onnx_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "LocomotionController");
    onnx_memory_info_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    // Create session options
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);

    // Create session from model file
    onnx_session_ = std::make_unique<Ort::Session>(*onnx_env_, model_path.c_str(), session_options);

    // Extract model metadata
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_inputs = onnx_session_->GetInputCount();
    size_t num_outputs = onnx_session_->GetOutputCount();

    // Extract input names and shape
    input_names_.clear();
    for (size_t i = 0; i < num_inputs; ++i)
    {
      auto input_name = onnx_session_->GetInputNameAllocated(i, allocator);
      input_names_.push_back(std::string(input_name.get()));
      if (i == 0)
      {
        auto input_type_info = onnx_session_->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        input_shape_ = input_tensor_info.GetShape();
      }
    }

    // Extract output names and shape
    output_names_.clear();
    for (size_t i = 0; i < num_outputs; ++i)
    {
      auto output_name = onnx_session_->GetOutputNameAllocated(i, allocator);
      output_names_.push_back(std::string(output_name.get()));
      if (i == 0)
      {
        auto output_type_info = onnx_session_->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        output_shape_ = output_tensor_info.GetShape();
      }
    }

    validate_model_structure(num_inputs, num_outputs);

    // Create pointer arrays from stored strings
    input_name_ptrs_.clear();
    for (const auto & name : input_names_)
    {
      input_name_ptrs_.push_back(name.c_str());
    }
    output_name_ptrs_.clear();
    for (const auto & name : output_names_)
    {
      output_name_ptrs_.push_back(name.c_str());
    }

    RCLCPP_INFO(get_node()->get_logger(), "Model loaded successfully from: %s", model_path.c_str());
    model_loaded_ = true;
    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load ONNX model: %s", e.what());
    return false;
  }
#else
  (void)model_path;
  RCLCPP_ERROR(
    get_node()->get_logger(), "ONNX Runtime not found. Install onnxruntime-dev package.");
  return false;
#endif
}

std::vector<double> MotionController::run_model_inference(const std::vector<float> & inputs)
{
#ifdef ONNXRUNTIME_FOUND
  if (!model_loaded_ || !onnx_session_)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000, "Model not loaded");
    return std::vector<double>(joint_names_.size(), 0.0);
  }

  try
  {
    // Prepare input tensor
    size_t input_size = inputs.size();

    // Use model's expected input shape if available, otherwise use [input_size]
    std::vector<int64_t> input_shape;
    if (!input_shape_.empty())
    {
      input_shape = input_shape_;
      // Replace -1 (dynamic dimension) with actual size
      for (auto & dim : input_shape)
      {
        if (dim == -1)
        {
          dim = static_cast<int64_t>(input_size);
        }
      }
    }
    else
    {
      // Fallback: single dimension
      input_shape = {static_cast<int64_t>(input_size)};
    }

    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Creating input tensor with shape from %zu inputs", input_size);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      *onnx_memory_info_, const_cast<float *>(inputs.data()), input_size, input_shape.data(),
      input_shape.size());

    // Run inference
    auto output_tensors = onnx_session_->Run(
      Ort::RunOptions{nullptr}, input_name_ptrs_.data(), &input_tensor, 1, output_name_ptrs_.data(),
      1);

    // Extract output tensor
    float * float_array = output_tensors.front().GetTensorMutableData<float>();
    size_t output_size = output_tensors.front().GetTensorTypeAndShapeInfo().GetElementCount();

    // Convert to vector<double>
    std::vector<double> outputs;
    outputs.reserve(output_size);
    for (size_t i = 0; i < output_size; ++i)
    {
      outputs.push_back(static_cast<double>(float_array[i]));
    }

    return outputs;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000, "Model inference failed: %s",
      e.what());
    return std::vector<double>(joint_names_.size(), 0.0);
  }
#else
  (void)inputs;
  RCLCPP_WARN_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 5000,
    "ONNX Runtime not available, returning zeros");
  return std::vector<double>(joint_names_.size(), 0.0);
#endif
}

#ifdef ONNXRUNTIME_FOUND
std::string MotionController::format_shape_string(const std::vector<int64_t> & shape)
{
  std::string shape_str = "[";
  for (size_t i = 0; i < shape.size(); ++i)
  {
    if (i > 0) shape_str += ", ";
    if (shape[i] == -1)
    {
      shape_str += "dynamic";
    }
    else
    {
      shape_str += std::to_string(shape[i]);
    }
  }
  shape_str += "]";
  return shape_str;
}

const char * MotionController::get_onnx_type_name(ONNXTensorElementDataType type)
{
  switch (type)
  {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
      return "float32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
      return "float64";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
      return "int32";
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
      return "int64";
    default:
      return "unknown";
  }
}


void MotionController::validate_model_structure(size_t num_inputs, size_t num_outputs)
{
  // Validate input/output counts
  if (num_inputs != 1)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model expects %zu inputs, expected 1", num_inputs);
    throw std::runtime_error("Invalid number of model inputs");
  }

  if (num_outputs != 1)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model expects %zu outputs, expected 1", num_outputs);
    throw std::runtime_error("Invalid number of model outputs");
  }

  // Validate model input dimensions against expected observation dimension
  RCLCPP_INFO(get_node()->get_logger(), "=== Model Validation ===");
  size_t expected_input_size = observation_formatter_->get_observation_dim();
  size_t model_input_size = 1;
  bool has_dynamic_dim = false;

  for (auto dim : input_shape_)
  {
    if (dim == -1)
    {
      has_dynamic_dim = true;
    }
    else if (dim > 0)
    {
      model_input_size *= dim;
    }
  }

  std::string shape_str = format_shape_string(input_shape_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Expected observation dimension (from config): %zu (17 + 6*%zu joints)", expected_input_size,
    joint_names_.size());
  RCLCPP_INFO(get_node()->get_logger(), "Model input shape: %s", shape_str.c_str());

  if (!has_dynamic_dim)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Model input total elements: %zu", model_input_size);
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Model has dynamic dimensions - will use actual observation size at runtime");
  }

  if (!has_dynamic_dim && model_input_size != expected_input_size)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "MISMATCH: Model input size (%zu) does not match expected observation dimension (%zu). "
      "Please check model or observation formatter configuration.",
      model_input_size, expected_input_size);
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected from Open Duck Mini: 3 (gyro) + 3 (accelero) + 7 (commands) "
      "+ %zu (joint_pos) + %zu (joint_vel) + %zu (last_action) + %zu (last_last_action) "
      "+ %zu (last_last_last_action) + %zu (motor_targets) + 2 (feet_contacts) + 2 (phase) = %zu",
      joint_names_.size(), joint_names_.size(), joint_names_.size(), joint_names_.size(),
      joint_names_.size(), joint_names_.size(), expected_input_size);
  }

  // Validate against config values if provided
  if (model_input_size_ > 0)
  {
    if (static_cast<size_t>(model_input_size_) != expected_input_size)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Config model_input_size (%d) does not match calculated observation dimension (%zu). "
        "Using calculated value.",
        model_input_size_, expected_input_size);
    }
    else if (!has_dynamic_dim && static_cast<size_t>(model_input_size_) != model_input_size)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Config model_input_size (%d) does not match ONNX model input size (%zu). "
        "ONNX model shape takes precedence.",
        model_input_size_, model_input_size);
    }
    else
    {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Config model_input_size (%d) matches calculated observation dimension (%zu)",
        model_input_size_, expected_input_size);
    }
  }

  // Check model output size - allow dynamic dimensions (-1) which will be validated at runtime
  bool has_dynamic_output = false;
  if (!output_shape_.empty())
  {
    for (auto dim : output_shape_)
    {
      if (dim == -1)
      {
        has_dynamic_output = true;
        break;
      }
    }
  }

  if (!has_dynamic_output && !output_shape_.empty())
  {
    // For shapes like [1, 14] or [14], check the last dimension (joint dimension)
    // For 1D shapes, last dimension is the only dimension
    int64_t joint_dim = output_shape_.back();
    
    // Calculate total elements to handle cases where batch dimension might be present
    int64_t total_elements = 1;
    for (auto dim : output_shape_)
    {
      if (dim > 0)
      {
        total_elements *= dim;
      }
    }
    
    // Check if last dimension matches joint count, or if total elements match
    // (for shapes like [1, 14], both should be 14)
    if (joint_dim != static_cast<int64_t>(joint_names_.size()) &&
        total_elements != static_cast<int64_t>(joint_names_.size()))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Model output size (last dim: %ld, total elements: %ld) does not match number of joints (%zu). "
        "This may cause runtime errors if model outputs don't match joint count.",
        joint_dim, total_elements, joint_names_.size());
    }
    
    // Validate against config value if provided
    if (model_output_size_ > 0)
    {
      if (static_cast<size_t>(model_output_size_) != joint_names_.size())
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Config model_output_size (%d) does not match number of joints (%zu). "
          "Using actual joint count.",
          model_output_size_, joint_names_.size());
      }
      else if (joint_dim != static_cast<int64_t>(model_output_size_) &&
               total_elements != static_cast<int64_t>(model_output_size_))
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Config model_output_size (%d) does not match ONNX model output size "
          "(last dim: %ld, total elements: %ld). ONNX model shape takes precedence.",
          model_output_size_, joint_dim, total_elements);
      }
      else
      {
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Config model_output_size (%d) matches number of joints (%zu) and ONNX model output",
          model_output_size_, joint_names_.size());
      }
    }
  }
  else if (has_dynamic_output)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Model has dynamic output dimensions - will validate actual output size at runtime");
    
    // Validate against config value if provided (for dynamic outputs)
    if (model_output_size_ > 0)
    {
      if (static_cast<size_t>(model_output_size_) != joint_names_.size())
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Config model_output_size (%d) does not match number of joints (%zu). "
          "Will validate at runtime.",
          model_output_size_, joint_names_.size());
      }
      else
      {
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Config model_output_size (%d) matches number of joints (%zu) - will validate at runtime",
          model_output_size_, joint_names_.size());
      }
    }
  }
}
#endif

}  // namespace motion_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_controller::MotionController, controller_interface::ControllerInterface)
