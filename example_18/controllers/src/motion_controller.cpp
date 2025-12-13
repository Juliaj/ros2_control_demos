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
    get_node()->declare_parameter<std::string>(
      "interfaces_broadcaster_topic", "interfaces_state_broadcaster/values");
    get_node()->declare_parameter<std::string>(
      "interfaces_broadcaster_names_topic", "interfaces_state_broadcaster/names");
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

  // Build command interface names: joint_name/position for each joint
  command_interface_names_.clear();
  for (const auto & joint_name : joint_names_)
  {
    command_interface_names_.push_back(joint_name + "/position");
  }

  // TODO(juliaj): Check whteher this is valid.
  // Initialize previous action to zeros
  previous_action_.resize(joint_names_.size(), 0.0);

  // Initialize action quality monitoring
  update_count_ = 0;
  total_clamps_ = 0;
  joint_clamp_counts_.resize(joint_names_.size(), 0);
  action_max_values_.resize(joint_names_.size(), -std::numeric_limits<double>::infinity());
  action_min_values_.resize(joint_names_.size(), std::numeric_limits<double>::infinity());
  previous_joint_commands_.resize(joint_names_.size(), 0.0);
  total_action_change_ = 0.0;
  extreme_action_count_ = 0;

  // Read IMU sensor name parameter
  std::string imu_sensor_name = get_node()->get_parameter("imu_sensor_name").as_string();
  RCLCPP_INFO(get_node()->get_logger(), "IMU sensor name: %s", imu_sensor_name.c_str());

  // Read Open Duck Mini specific parameters
  bool imu_upside_down = get_node()->get_parameter("imu_upside_down").as_bool();
  phase_frequency_factor_offset_ =
    get_node()->get_parameter("phase_frequency_factor_offset").as_double();
  double phase_period = get_node()->get_parameter("phase_period").as_double();
  RCLCPP_INFO(get_node()->get_logger(), "IMU upside down: %s", imu_upside_down ? "true" : "false");
  RCLCPP_INFO(
    get_node()->get_logger(), "Phase frequency factor offset: %.4f",
    phase_frequency_factor_offset_);
  RCLCPP_INFO(get_node()->get_logger(), "Phase period: %.1f steps", phase_period);

  // Initialize observation formatter
  observation_formatter_ = std::make_unique<ObservationFormatter>(joint_names_, imu_sensor_name);
  observation_formatter_->set_phase_period(phase_period);

  // Read action_scale parameter
  double action_scale = get_node()->get_parameter("action_scale").as_double();
  RCLCPP_INFO(get_node()->get_logger(), "Action scale: %.6f", action_scale);

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
  RCLCPP_INFO(
    get_node()->get_logger(), "Head joint indices: [%zu, %zu, %zu, %zu]",
    head_joint_indices.size() > 0 ? head_joint_indices[0] : 0,
    head_joint_indices.size() > 1 ? head_joint_indices[1] : 0,
    head_joint_indices.size() > 2 ? head_joint_indices[2] : 0,
    head_joint_indices.size() > 3 ? head_joint_indices[3] : 0);

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
    RCLCPP_INFO(
      get_node()->get_logger(), "Using default joint positions from parameters (%zu joints)",
      default_joint_positions_.size());

    // Log default positions for verification
    std::stringstream pos_info;
    pos_info << "Default joint positions: ";
    for (size_t i = 0; i < default_joint_positions_.size() && i < joint_names_.size(); ++i)
    {
      if (i > 0) pos_info << ", ";
      pos_info << joint_names_[i] << "=" << std::fixed << std::setprecision(4)
               << default_joint_positions_[i];
    }
    RCLCPP_INFO(get_node()->get_logger(), "%s", pos_info.str().c_str());

    // Validate default positions are within reasonable bounds
    bool positions_valid = true;
    for (size_t i = 0; i < default_joint_positions_.size() && i < joint_names_.size(); ++i)
    {
      if (std::abs(default_joint_positions_[i]) > 3.14)
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Default position for joint '%s' (%.4f) exceeds reasonable bounds (±π)",
          joint_names_[i].c_str(), default_joint_positions_[i]);
        positions_valid = false;
      }
    }
    if (positions_valid)
    {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Default joint positions validated: all within reasonable bounds");
    }
  }
  else
  {
    // Will be set from sensor data on first update
    default_joint_positions_initialized_ = false;
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Default joint positions not provided (got %zu, expected %zu). "
      "Will initialize from sensor data on first update - this may cause instability!",
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

  // Log limits for debugging
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Joint '%s' limits: [%.6f, %.6f]", joint_names_[i].c_str(),
      joint_position_limits_min_[i], joint_position_limits_max_[i]);
  }

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

  // Log the topic name being subscribed to
  // Note: ~/cmd_vel expands to /<node_namespace>/cmd_vel (e.g., /motion_controller/cmd_vel)
  RCLCPP_INFO(
    get_node()->get_logger(), "Subscribed to velocity command topic: %s (node namespace: %s)",
    velocity_command_topic_.c_str(), get_node()->get_namespace());

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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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
  bool velocity_cmd_received = false;
  if (velocity_cmd_op.has_value())
  {
    velocity_cmd = velocity_cmd_op.value();
    velocity_cmd_received = true;
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

  // Log observation inputs for debugging (throttled)
  RCLCPP_DEBUG_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 3000,
    "Observation inputs: vel_cmd=[%.3f, %.3f, %.3f], head=[%.3f, %.3f, %.3f, %.3f]",
    velocity_commands_7d[0], velocity_commands_7d[1], velocity_commands_7d[2],
    velocity_commands_7d[3], velocity_commands_7d[4], velocity_commands_7d[5], velocity_commands_7d[6]);

  // Log velocity commands for debugging (throttled)
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 2000,
    "Velocity command: lin_x=%.4f, lin_y=%.4f, ang_z=%.4f, head=[%.4f, %.4f, %.4f, %.4f] (received=%s)",
    velocity_cmd.base_velocity.linear.x, velocity_cmd.base_velocity.linear.y,
    velocity_cmd.base_velocity.angular.z,
    velocity_commands_7d[3], velocity_commands_7d[4], velocity_commands_7d[5], velocity_commands_7d[6],
    velocity_cmd_received ? "yes" : "no");
// Check if velocity command is effectively zero (no movement command)
const double vel_tolerance = 1e-6;
bool is_zero_command =
  !velocity_cmd_received || (std::abs(velocity_cmd.base_velocity.linear.x) < vel_tolerance &&
                             std::abs(velocity_cmd.base_velocity.linear.y) < vel_tolerance &&
                             std::abs(velocity_cmd.base_velocity.angular.z) < vel_tolerance);


    // Warn if no velocity command received (robot may not move as expected)
  if (!velocity_cmd_received)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "No velocity command received - using default joint positions to maintain stable pose. "
      "Send commands to /motion_controller/cmd_vel topic to control robot movement.");
  }

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

    // Log initialized positions for verification
    std::stringstream pos_info;
    pos_info << "Initialized default joint positions from sensor data: ";
    for (size_t i = 0; i < default_joint_positions_.size() && i < joint_names_.size(); ++i)
    {
      if (i > 0) pos_info << ", ";
      pos_info << joint_names_[i] << "=" << std::fixed << std::setprecision(4)
               << default_joint_positions_[i];
    }
    RCLCPP_WARN(get_node()->get_logger(), "%s", pos_info.str().c_str());
    RCLCPP_WARN(
      get_node()->get_logger(),
      "WARNING: Using sensor-initialized default positions may cause instability. "
      "Consider setting default_joint_positions in config for stable operation.");
  }

  // Update imitation phase (gait phase encoding)
  // Source: v2_rl_walk_mujoco.py lines 257-270
  double phase_freq_factor = 1.0 + phase_frequency_factor_offset_;
  observation_formatter_->update_imitation_phase(phase_freq_factor);

  // Get feet contacts from MuJoCo simulation
  // In MuJoCo training (Playground, mujoco_infer_base.py get_feet_contacts), contacts are queried from
  // physics engine: check_contact("foot_assembly", "floor") returns binary 1.0/0.0
  // MuJoCo automatically tracks these from collision detection.
  // 
  // NOTE: Contact sensors are NOT currently implemented in mujoco_ros2_simulation.
  // The package only supports IMU, FTS (force/torque), cameras, and lidar sensors.
  // To add contact sensors, mujoco_ros2_simulation would need to be extended to:
  //   1. Add contact sensor type in register_sensors() (similar to "fts" and "imu")
  //   2. Query mj_data->ncon and mj_data->contact[] in read() method
  //   3. Expose contact state interfaces through export_state_interfaces()
  //   4. Add sensor definition in ros2_control xacro with state interfaces
  // 
  // For now, estimate contacts from gait phase as temporary workaround.
  // Since model was trained with binary contacts, we use binary values here:
  // Alternating left/right based on gait phase sin component
  auto phase = observation_formatter_->get_imitation_phase();  // [cos, sin]
  double sin_phase = phase[1];  // sin component ranges -1 to 1
  
  // Binary contact switching based on phase
  // When sin_phase > 0: left foot stance (1.0), right foot swing (0.0)
  // When sin_phase < 0: right foot stance (1.0), left foot swing (0.0)
  double left_contact = (sin_phase > 0.0) ? 1.0 : 0.0;
  double right_contact = (sin_phase < 0.0) ? 1.0 : 0.0;
  observation_formatter_->set_feet_contacts(left_contact, right_contact);
  
  // Log contact states and gait phase for debugging
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 2000,
    "Gait phase: cos=%.3f, sin=%.3f (theta=%.1f°) | Foot contacts: L=%.0f, R=%.0f | Vel cmd: x=%.3f, y=%.3f, z=%.3f",
    phase[0], sin_phase, std::atan2(sin_phase, phase[0]) * 180.0 / 3.14159,
    left_contact, right_contact,
    velocity_cmd.base_velocity.linear.x, velocity_cmd.base_velocity.linear.y, 
    velocity_cmd.base_velocity.angular.z);

  std::vector<double> model_outputs;
  std::vector<double> joint_commands;


  if (is_zero_command)  
  {
    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Zero velocity command detected - using default joint positions to maintain stable pose");

    // Use default positions as joint commands (model outputs are zero for relative positions)
    model_outputs = std::vector<double>(joint_names_.size(), 0.0);
    joint_commands = default_joint_positions_;

    // Store motor targets for observation
    observation_formatter_->set_motor_targets(joint_commands);

    // Store zero action as previous for next iteration
    previous_action_ = model_outputs;
  }
  else
  {
    // Format inputs for model using ObservationFormatter
    std::vector<float> model_inputs;
    try
    {
      // Pass base_velocity to format() for API compatibility (not actually used, set_velocity_commands() is used instead)
      model_inputs = observation_formatter_->format(interface_data, velocity_cmd.base_velocity, previous_action_);

      // Debug: Log first 13 elements (gyro + accelero + commands) sent to model
      if (model_inputs.size() >= 13)
      {
        RCLCPP_DEBUG_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Observation prefix: gyro=[%.4f,%.4f,%.4f], accel=[%.4f,%.4f,%.4f], "
          "commands=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]",
          model_inputs[0], model_inputs[1], model_inputs[2], model_inputs[3], model_inputs[4],
          model_inputs[5], model_inputs[6], model_inputs[7], model_inputs[8], model_inputs[9],
          model_inputs[10], model_inputs[11], model_inputs[12]);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to format observation: %s", e.what());
      return return_type::ERROR;
    }

    // Debug: Log observation dimensions
    size_t expected_dim = observation_formatter_->get_observation_dim();
    if (model_inputs.size() != expected_dim)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Observation size mismatch: got %zu, expected %zu", model_inputs.size(), expected_dim);
      return return_type::ERROR;
    }

    // Run model inference (returns relative joint positions, scaled by 0.25)
    model_outputs = run_model_inference(model_inputs);

    // Debug: Log raw model outputs to diagnose action quality issues
    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Raw model outputs (before scaling): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, "
      "%.4f, %.4f, %.4f]",
      model_outputs.size() > 0 ? model_outputs[0] : 0.0,
      model_outputs.size() > 1 ? model_outputs[1] : 0.0,
      model_outputs.size() > 2 ? model_outputs[2] : 0.0,
      model_outputs.size() > 3 ? model_outputs[3] : 0.0,
      model_outputs.size() > 4 ? model_outputs[4] : 0.0,
      model_outputs.size() > 5 ? model_outputs[5] : 0.0,
      model_outputs.size() > 6 ? model_outputs[6] : 0.0,
      model_outputs.size() > 7 ? model_outputs[7] : 0.0,
      model_outputs.size() > 8 ? model_outputs[8] : 0.0,
      model_outputs.size() > 9 ? model_outputs[9] : 0.0,
      model_outputs.size() > 10 ? model_outputs[10] : 0.0,
      model_outputs.size() > 11 ? model_outputs[11] : 0.0);

    // Process model outputs: apply scaling and default offset to get absolute positions
    joint_commands = action_processor_->process(model_outputs, default_joint_positions_);

    // Store motor targets for observation (before applying head commands)
    observation_formatter_->set_motor_targets(joint_commands);

    // Store current action as previous for next iteration (use raw model outputs, not processed)
    previous_action_ = model_outputs;
  }

  // Debug: Log all joint commands before clamping (compact format)
  std::stringstream debug_before;
  debug_before << "Joint commands (before clamping): ";
  for (size_t i = 0; i < joint_commands.size() && i < joint_names_.size(); ++i)
  {
    if (i > 0) debug_before << ", ";
    debug_before << joint_names_[i] << "=" << std::fixed << std::setprecision(4)
                 << joint_commands[i];
  }
  RCLCPP_DEBUG_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 2000, "%s", debug_before.str().c_str());

  // Info: Log action history [last-3, last-2, last-1, current] for debugging (throttled)
  auto last_action = observation_formatter_->get_last_action();
  auto last_last_action = observation_formatter_->get_last_last_action();
  auto last_last_last_action = observation_formatter_->get_last_last_last_action();
  
  std::stringstream info_summary;
  info_summary << "Action history [last-3, last-2, last-1, current]:\n";
  
  // Print each joint's 4-step history on one line
  for (size_t i = 0; i < joint_commands.size() && i < joint_names_.size(); ++i)
  {
    info_summary << "  " << joint_names_[i] << ": [";
    
    // Last-3
    if (!last_last_last_action.empty() && i < last_last_last_action.size())
      info_summary << std::fixed << std::setprecision(4) << last_last_last_action[i];
    else
      info_summary << "----";
    info_summary << ", ";
    
    // Last-2
    if (!last_last_action.empty() && i < last_last_action.size())
      info_summary << std::fixed << std::setprecision(4) << last_last_action[i];
    else
      info_summary << "----";
    info_summary << ", ";
    
    // Last-1
    if (!last_action.empty() && i < last_action.size())
      info_summary << std::fixed << std::setprecision(4) << last_action[i];
    else
      info_summary << "----";
    info_summary << ", ";
    
    // Current
    info_summary << std::fixed << std::setprecision(4) << joint_commands[i];
    info_summary << "]\n";
  }
  
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 5000, "%s", info_summary.str().c_str());

  // Track action quality metrics
  size_t num_clamped = 0;
  double total_change = 0.0;

  // Write joint commands to hardware interfaces (with clamping to limits)
  // TODO(juliaj): Consider querying limits from command interface at runtime instead of using
  // hardcoded values. This would ensure limits match what MuJoCo/hardware actually enforces.
  for (size_t i = 0; i < command_interfaces_.size() && i < joint_commands.size(); ++i)
  {
    // Track action statistics
    action_max_values_[i] = std::max(action_max_values_[i], joint_commands[i]);
    action_min_values_[i] = std::min(action_min_values_[i], joint_commands[i]);

    // Track action smoothness (rate of change)
    if (update_count_ > 0)
    {
      double change = std::abs(joint_commands[i] - previous_joint_commands_[i]);
      total_change += change;

      // Flag large action changes (>0.3 rad in one step) - potential cause of instability
      if (change > 0.3)
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 500,
          "LARGE ACTION CHANGE: Joint '%s' changed by %.4f rad (from %.4f to %.4f) - "
          "may cause instability!",
          joint_names_[i].c_str(), change, previous_joint_commands_[i], joint_commands[i]);
      }
    }

    // Check if hip/ankle joints exceed 50% of limit range (critical for stability)
    const std::string & joint_name = joint_names_[i];
    bool is_hip_or_ankle = (joint_name.find("hip") != std::string::npos) ||
                           (joint_name.find("ankle") != std::string::npos);
    if (is_hip_or_ankle)
    {
      double limit_range = joint_position_limits_max_[i] - joint_position_limits_min_[i];
      if (limit_range > 1e-6)  // Avoid division by zero
      {
        double position_in_range =
          (joint_commands[i] - joint_position_limits_min_[i]) / limit_range;
        // Flag if command is beyond 50% of range (either >75% or <25%)
        if (position_in_range > 0.75 || position_in_range < 0.25)
        {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 500,
            "HIP/ANKLE NEAR LIMIT: Joint '%s' at %.4f (%.1f%% of range [%.4f, %.4f]) - "
            "may cause instability!",
            joint_names_[i].c_str(), joint_commands[i], position_in_range * 100.0,
            joint_position_limits_min_[i], joint_position_limits_max_[i]);
        }
      }
    }

    // Clamp command to joint limits
    double clamped_command =
      std::clamp(joint_commands[i], joint_position_limits_min_[i], joint_position_limits_max_[i]);

    if (clamped_command != joint_commands[i])
    {
      num_clamped++;
      joint_clamp_counts_[i]++;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Joint '%s' command clamped from %.6f to %.6f (limits: [%.6f, %.6f])",
        joint_names_[i].c_str(), joint_commands[i], clamped_command, joint_position_limits_min_[i],
        joint_position_limits_max_[i]);
    }

    // Detect extreme actions (beyond reasonable bounds)
    if (std::abs(joint_commands[i]) > 3.0)
    {
      extreme_action_count_++;
    }

    // Additional safety check: if command is still out of reasonable bounds, log error
    if (std::abs(clamped_command) > 3.14)  // Warn if command exceeds ±π
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Joint '%s' command %.6f exceeds reasonable bounds (limits: [%.6f, %.6f])",
        joint_names_[i].c_str(), clamped_command, joint_position_limits_min_[i],
        joint_position_limits_max_[i]);
    }

    const bool write_success = command_interfaces_[i].set_value(clamped_command);
    if (!write_success)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set command for joint '%s' to %.6f (limits: [%.6f, %.6f])",
        joint_names_[i].c_str(), clamped_command, joint_position_limits_min_[i],
        joint_position_limits_max_[i]);
    }
  }

  // Debug: Log all joint commands after clamping (compact format)
  std::stringstream debug_after;
  debug_after << "Joint commands (after clamping): ";
  for (size_t i = 0; i < command_interfaces_.size() && i < joint_commands.size(); ++i)
  {
    if (i > 0) debug_after << ", ";
    double clamped_cmd =
      std::clamp(joint_commands[i], joint_position_limits_min_[i], joint_position_limits_max_[i]);
    debug_after << joint_names_[i] << "=" << std::fixed << std::setprecision(4) << clamped_cmd;
    if (clamped_cmd != joint_commands[i])
    {
      debug_after << "(clamped)";
    }
  }
  RCLCPP_DEBUG_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 2000, "%s", debug_after.str().c_str());

  // Update action quality metrics
  update_count_++;
  total_clamps_ += num_clamped;
  total_action_change_ += total_change;
  previous_joint_commands_ = joint_commands;

  // Report action quality statistics periodically
  report_action_quality(joint_commands, num_clamped);

  return return_type::OK;
}

void MotionController::initialize_joint_limits()
{
  // Initialize joint limits based on ros2_control config values
  // These match the limits defined in open_duck_mini.ros2_control.xacro
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const std::string & joint_name = joint_names_[i];

    // Set limits based on joint name (matching ros2_control config)
    // Source: open_duck_mini.ros2_control.xacro
    if (joint_name == "left_hip_yaw_joint" || joint_name == "right_hip_yaw_joint")
    {
      joint_position_limits_min_[i] = -0.52359877559829881566;  // -30 degrees
      joint_position_limits_max_[i] = 0.52359877559829881566;   // +30 degrees
    }
    else if (joint_name == "left_hip_roll_joint" || joint_name == "right_hip_roll_joint")
    {
      joint_position_limits_min_[i] = -0.43633231299858238339;  // -25 degrees
      joint_position_limits_max_[i] = 0.43633231299858238339;   // +25 degrees
    }
    else if (joint_name == "left_hip_pitch_joint")
    {
      joint_position_limits_min_[i] = -1.2217304763960306069;  // -70 degrees
      joint_position_limits_max_[i] = 0.52359877559829881566;  // +30 degrees
    }
    else if (joint_name == "right_hip_pitch_joint")
    {
      joint_position_limits_min_[i] = -0.52359877559829881566;  // -30 degrees
      joint_position_limits_max_[i] = 1.2217304763960306069;    // +70 degrees
    }
    else if (joint_name == "left_knee_joint" || joint_name == "right_knee_joint")
    {
      joint_position_limits_min_[i] = -1.570796326794896558;  // -90 degrees
      joint_position_limits_max_[i] = 1.570796326794896558;   // +90 degrees
    }
    else if (joint_name == "left_ankle_joint" || joint_name == "right_ankle_joint")
    {
      joint_position_limits_min_[i] = -1.570796326794896558;  // -90 degrees
      joint_position_limits_max_[i] = 1.570796326794896558;   // +90 degrees
    }
    else if (joint_name == "neck_pitch_joint")
    {
      joint_position_limits_min_[i] = -1.047197551196597746;  // -60 degrees
      joint_position_limits_max_[i] = 1.047197551196597746;   // +60 degrees
    }
    else if (joint_name == "head_pitch_joint")
    {
      joint_position_limits_min_[i] = -0.785398163397448279;  // -45 degrees
      joint_position_limits_max_[i] = 0.785398163397448279;   // +45 degrees
    }
    else if (joint_name == "head_yaw_joint")
    {
      joint_position_limits_min_[i] = -1.570796326794896558;  // -90 degrees
      joint_position_limits_max_[i] = 1.570796326794896558;   // +90 degrees
    }
    else if (joint_name == "head_roll_joint")
    {
      joint_position_limits_min_[i] = -0.785398163397448279;  // -45 degrees
      joint_position_limits_max_[i] = 0.785398163397448279;   // +45 degrees
    }
    // For unknown joints, use no limits (already set to +/- infinity)
  }
}

void MotionController::report_action_quality(
  const std::vector<double> & /* joint_commands */, size_t /* num_clamped */)
{
  // Report statistics every 100 updates (~4 seconds at 25Hz)
  if (update_count_ % 100 == 0 && update_count_ > 0)
  {
    double clamp_rate =
      (100.0 * static_cast<double>(total_clamps_)) /
      (static_cast<double>(update_count_) * static_cast<double>(joint_names_.size()));
    double avg_action_change = total_action_change_ / static_cast<double>(update_count_ - 1);
    double extreme_rate =
      (100.0 * static_cast<double>(extreme_action_count_)) /
      (static_cast<double>(update_count_) * static_cast<double>(joint_names_.size()));

    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "=== Action Quality Report (after %zu updates) ===", update_count_);
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "Clamping: %.2f%% of commands clamped (total: %zu)", clamp_rate, total_clamps_);
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "Action smoothness: avg change per step = %.6f rad", avg_action_change);
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "Extreme actions (>3.0 rad): %.2f%%", extreme_rate);

    // Report most frequently clamped joints
    std::vector<std::pair<size_t, size_t>> clamp_ranking;
    for (size_t i = 0; i < joint_clamp_counts_.size(); ++i)
    {
      if (joint_clamp_counts_[i] > 0)
      {
        clamp_ranking.push_back({i, joint_clamp_counts_[i]});
      }
    }
    std::sort(
      clamp_ranking.begin(), clamp_ranking.end(),
      [](const auto & a, const auto & b) { return a.second > b.second; });

    if (!clamp_ranking.empty())
    {
      std::stringstream clamp_info;
      clamp_info << "Most clamped joints: ";
      for (size_t i = 0; i < std::min(clamp_ranking.size(), size_t(5)); ++i)
      {
        if (i > 0) clamp_info << ", ";
        clamp_info << joint_names_[clamp_ranking[i].first] << "(" << clamp_ranking[i].second
                   << "x)";
      }
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 10000, "%s", clamp_info.str().c_str());
    }

    // Report action ranges
    std::stringstream range_info;
    range_info << "Action ranges: ";
    for (size_t i = 0; i < std::min(joint_names_.size(), size_t(6)); ++i)
    {
      if (i > 0) range_info << ", ";
      range_info << joint_names_[i] << "[" << std::fixed << std::setprecision(3)
                 << action_min_values_[i] << "," << action_max_values_[i] << "]";
    }
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000, "%s", range_info.str().c_str());

    // Check for problematic patterns
    if (clamp_rate > 20.0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 10000,
        "WARNING: High clamping rate (%.2f%%) - actions frequently exceed limits!", clamp_rate);
    }
    if (avg_action_change > 0.5)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 10000,
        "WARNING: High action change rate (%.6f rad/step) - actions may be unstable!",
        avg_action_change);
    }
    if (extreme_rate > 5.0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 10000,
        "WARNING: High extreme action rate (%.2f%%) - model may be producing invalid outputs!",
        extreme_rate);
    }
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

    // Extract and log model metadata
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_inputs = onnx_session_->GetInputCount();
    size_t num_outputs = onnx_session_->GetOutputCount();

    log_input_metadata(*onnx_session_, allocator);
    log_output_metadata(*onnx_session_, allocator);
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

void MotionController::log_input_metadata(
  Ort::Session & session, Ort::AllocatorWithDefaultOptions & allocator)
{
  size_t num_inputs = session.GetInputCount();
  RCLCPP_INFO(get_node()->get_logger(), "=== ONNX Model Input Metadata ===");
  RCLCPP_INFO(get_node()->get_logger(), "Number of inputs: %zu", num_inputs);

  input_names_.clear();
  for (size_t i = 0; i < num_inputs; ++i)
  {
    auto input_name = session.GetInputNameAllocated(i, allocator);
    std::string input_name_str(input_name.get());
    input_names_.push_back(input_name_str);

    auto input_type_info = session.GetInputTypeInfo(i);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    ONNXTensorElementDataType input_type = input_tensor_info.GetElementType();
    std::vector<int64_t> input_shape = input_tensor_info.GetShape();

    std::string shape_str = format_shape_string(input_shape);
    const char * type_name = get_onnx_type_name(input_type);

    RCLCPP_INFO(
      get_node()->get_logger(), "  Input[%zu]: name='%s', shape=%s, type=%s", i,
      input_name_str.c_str(), shape_str.c_str(), type_name);

    // Store first input shape for validation
    if (i == 0)
    {
      input_shape_ = input_shape;
    }
  }
}

void MotionController::log_output_metadata(
  Ort::Session & session, Ort::AllocatorWithDefaultOptions & allocator)
{
  size_t num_outputs = session.GetOutputCount();
  RCLCPP_INFO(get_node()->get_logger(), "=== ONNX Model Output Metadata ===");
  RCLCPP_INFO(get_node()->get_logger(), "Number of outputs: %zu", num_outputs);

  output_names_.clear();
  for (size_t i = 0; i < num_outputs; ++i)
  {
    auto output_name = session.GetOutputNameAllocated(i, allocator);
    std::string output_name_str(output_name.get());
    output_names_.push_back(output_name_str);

    auto output_type_info = session.GetOutputTypeInfo(i);
    auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
    ONNXTensorElementDataType output_type = output_tensor_info.GetElementType();
    std::vector<int64_t> output_shape = output_tensor_info.GetShape();

    std::string shape_str = format_shape_string(output_shape);
    const char * type_name = get_onnx_type_name(output_type);

    RCLCPP_INFO(
      get_node()->get_logger(), "  Output[%zu]: name='%s', shape=%s, type=%s", i,
      output_name_str.c_str(), shape_str.c_str(), type_name);

    // Store first output shape for validation
    if (i == 0)
    {
      output_shape_ = output_shape;
    }
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

  if (!has_dynamic_output && !output_shape_.empty() &&
      output_shape_[0] != static_cast<int64_t>(joint_names_.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Model output size (%ld) does not match number of joints (%zu). "
      "This may cause runtime errors if model outputs don't match joint count.",
      output_shape_[0], joint_names_.size());
  }
  else if (has_dynamic_output)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Model has dynamic output dimensions - will validate actual output size at runtime");
  }
}
#endif

}  // namespace motion_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_controller::MotionController, controller_interface::ControllerInterface)
