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
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "motion_controller/motion_controller.hpp"

#ifdef HAVE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

namespace motion_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

// Constants for throttled logging (steps)
namespace
{
constexpr int LOG_INTERVAL_CONTROL = 50;      // ~1 second at 50Hz
constexpr int LOG_INTERVAL_OBSERVATION = 50;  // ~1 second at 50Hz
constexpr int LOG_INTERVAL_VELOCITY = 125;    // ~5 seconds at 25Hz
constexpr int LOG_INTERVAL_CONTACT = 50;      // ~1 second at 50Hz
constexpr int LOG_INTERVAL_CLIP = 25;         // Every 25 updates
constexpr int LOG_INTERVAL_CMD_WRITE = 250;   // Less frequent
}  // anonymous namespace

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
    get_node()->declare_parameter<std::string>(
      "velocity_command_topic", "~/cmd_velocity_with_head");
    get_node()->declare_parameter<double>("action_scale", 0.3);
    get_node()->declare_parameter<std::vector<double>>(
      "default_joint_positions", std::vector<double>());
    get_node()->declare_parameter<std::string>("imu_sensor_name", "imu_2");
    get_node()->declare_parameter<std::string>("feet_contact_topic", "~/feet_contacts");
    get_node()->declare_parameter<bool>("use_contact_sensors", false);
    get_node()->declare_parameter<bool>("log_contact_sensors", true);
    get_node()->declare_parameter<std::string>("left_contact_sensor_name", "left_foot_contact");
    get_node()->declare_parameter<std::string>("right_contact_sensor_name", "right_foot_contact");
    get_node()->declare_parameter<bool>("start_paused", false);
    get_node()->declare_parameter<bool>("imu_upside_down", false);
    get_node()->declare_parameter<double>("phase_frequency_factor_offset", 0.0);
    get_node()->declare_parameter<double>("phase_period", 50.0);
    get_node()->declare_parameter<double>("cutoff_frequency", 0.0);
    get_node()->declare_parameter<double>("max_motor_velocity", 8.0);
    get_node()->declare_parameter<double>("reference_motion_blend_factor", 0.2);
    get_node()->declare_parameter<double>("training_control_period", 0.02);
    get_node()->declare_parameter<double>("gyro_deadband", 0.15);
    get_node()->declare_parameter<int>("stabilization_delay", 50);
    get_node()->declare_parameter<int>("blend_in_steps", 200);
    get_node()->declare_parameter<bool>("enable_debug_publishing", false);
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
  use_contact_sensors_ = get_node()->get_parameter("use_contact_sensors").as_bool();
  log_contact_sensors_ = get_node()->get_parameter("log_contact_sensors").as_bool();
  left_contact_sensor_name_ = get_node()->get_parameter("left_contact_sensor_name").as_string();
  right_contact_sensor_name_ = get_node()->get_parameter("right_contact_sensor_name").as_string();

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
    RCLCPP_INFO(get_node()->get_logger(), "Model input size (from config): %d", model_input_size_);
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
  observation_formatter_->set_gyro_deadband(gyro_deadband_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "IMU configuration: sensor_name='%s', imu_upside_down=%s (z-axis will be %s)",
    imu_sensor_name.c_str(), imu_upside_down ? "true" : "false",
    imu_upside_down ? "inverted" : "not inverted");

  // Read action_scale parameter
  double action_scale = get_node()->get_parameter("action_scale").as_double();

  // Initialize action processor with configurable scale
  action_processor_ = std::make_unique<ActionProcessor>(joint_names_, action_scale, true);

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
      get_node()->get_logger(), "Default joint positions loaded from config (%zu joints)",
      default_joint_positions_.size());
  }
  else
  {
    // Will be set from sensor data on first update
    default_joint_positions_initialized_ = false;
    RCLCPP_WARN(
      get_node()->get_logger(),
      "default_joint_positions parameter size (%zu) != joint count (%zu). Will read from sensors "
      "on first update.",
      param_default_positions.size(), joint_names_.size());
  }

  // Read motor speed limit parameter
  max_motor_velocity_ = get_node()->get_parameter("max_motor_velocity").as_double();

  // Read reference motion blend factor parameter
  reference_motion_blend_factor_ =
    get_node()->get_parameter("reference_motion_blend_factor").as_double();
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

  // Read gyro deadband parameter
  gyro_deadband_ = get_node()->get_parameter("gyro_deadband").as_double();

  // Read stabilization and blend-in parameters
  stabilization_delay_ =
    static_cast<int>(get_node()->get_parameter("stabilization_delay").as_int());
  blend_in_steps_ = static_cast<int>(get_node()->get_parameter("blend_in_steps").as_int());
  stabilization_steps_ = 0;
  onnx_active_steps_ = 0;

  // Initialize motor targets and previous motor targets to default_joint_positions
  // Reference: mujoco_infer.py lines 124-125: motor_targets = default_actuator, prev_motor_targets
  // = default_actuator
  motor_targets_.resize(joint_names_.size(), 0.0);
  prev_motor_targets_.resize(joint_names_.size(), 0.0);
  if (default_joint_positions_initialized_)
  {
    initialize_motor_targets_from_defaults();
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
    {
      // Don't overwrite injected data until explicitly allowed
      if (!ignore_interface_data_updates_.load())
      {
        rt_interface_data_.set(*msg);
      }
    });

  interfaces_names_subscriber_ = get_node()->create_subscription<control_msgs::msg::Keys>(
    interfaces_broadcaster_names_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    [this](const control_msgs::msg::Keys::SharedPtr msg) { rt_interface_names_.set(msg->keys); });

  velocity_command_subscriber_ =
    get_node()
      ->create_subscription<example_18_motion_controller_msgs::msg::VelocityCommandWithHead>(
        velocity_command_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const example_18_motion_controller_msgs::msg::VelocityCommandWithHead::SharedPtr msg)
        { rt_velocity_command_.set(*msg); });

  // Topic for receiving test observations
  test_observation_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/test_observation", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      test_observation_.clear();
      test_observation_.reserve(msg->data.size());
      for (const auto & val : msg->data)
      {
        test_observation_.push_back(static_cast<float>(val));
      }
    });

  // Publisher for test action output
  test_action_output_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/test_action_output", rclcpp::QoS(10));

  // Publisher for test hardware commands (final processed actions)
  test_hardware_commands_publisher_ =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/test_hardware_commands", rclcpp::QoS(10));

  // Service for testing: run ONNX inference from test observation
  test_inference_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/test_inference", [this](
                          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    { this->handle_test_inference(request, response); });

  // Service for resetting controller state
  reset_state_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/reset_state", [this](
                       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    { this->handle_reset_state(request, response); });

  // Topic for receiving YAML file path
  inject_yaml_path_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
    "~/inject_yaml_path", rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr msg) { inject_yaml_file_path_ = msg->data; });

  // Service for injecting state from YAML file
  inject_state_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/inject_state", [this](
                        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    { this->handle_inject_state(request, response); });

  // Service for resuming updates after injection
  resume_updates_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/resume_updates", [this](
                          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    { this->handle_resume_updates(request, response); });

  // Get debug publishing flag
  enable_debug_publishing_ = get_node()->get_parameter("enable_debug_publishing").as_bool();

  // Initialize update halt flag (for state injection debugging)
  update_halted_after_injection_ = false;
  ignore_interface_data_updates_ = false;

  // Create debug publishers if enabled
  if (enable_debug_publishing_)
  {
    debug_formatted_observation_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/debug/formatted_observation", rclcpp::QoS(10));
    debug_raw_action_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/debug/raw_action", rclcpp::QoS(10));
    debug_processed_action_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/debug/processed_action", rclcpp::QoS(10));
    debug_blended_action_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/debug/blended_action", rclcpp::QoS(10));
    debug_rate_limited_action_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/debug/rate_limited_action", rclcpp::QoS(10));
    debug_prev_motor_targets_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/debug/prev_motor_targets", rclcpp::QoS(10));
    debug_update_complete_publisher_ = get_node()->create_publisher<std_msgs::msg::UInt64>(
      "~/debug/update_complete", rclcpp::QoS(10));
    RCLCPP_INFO(get_node()->get_logger(), "Debug publishing enabled");
  }

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

return_type MotionController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Check if updates are halted after injection
  // Allow one update cycle if interface data updates are ignored (for observation capture)
  // This allows formatting and publishing debug topics while keeping injected data protected
  static bool one_update_allowed = false;
  static bool last_ignore_flag = false;

  // Reset flag when ignore_interface_data_updates_ changes from false to true (new injection)
  bool current_ignore_flag = ignore_interface_data_updates_.load();
  if (current_ignore_flag && !last_ignore_flag)
  {
    one_update_allowed = false;  // Reset to allow one update for new injection
  }
  last_ignore_flag = current_ignore_flag;

  if (update_halted_after_injection_)
  {
    if (ignore_interface_data_updates_.load() && !one_update_allowed)
    {
      // Allow this one update cycle to format and publish debug topics
      one_update_allowed = true;
      // Continue to normal update processing below
    }
    else
    {
      // Still write current motor_targets to hardware to maintain position
      if (prev_motor_targets_initialized_ && command_interfaces_.size() == motor_targets_.size())
      {
        write_commands_to_hardware(motor_targets_);
      }
      return return_type::OK;
    }
  }
  else
  {
    // Reset flag when updates are not halted
    one_update_allowed = false;
  }

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

    // Log velocity command (throttled to avoid spam)
    if (update_count_ % LOG_INTERVAL_VELOCITY == 0)
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "[VEL_CMD] lin_x=%.3f, lin_y=%.3f, ang_z=%.3f, head=[%.3f, %.3f, %.3f, %.3f]",
        velocity_cmd.base_velocity.linear.x, velocity_cmd.base_velocity.linear.y,
        velocity_cmd.base_velocity.angular.z,
        velocity_cmd.head_commands.size() > 0 ? velocity_cmd.head_commands[0] : 0.0,
        velocity_cmd.head_commands.size() > 1 ? velocity_cmd.head_commands[1] : 0.0,
        velocity_cmd.head_commands.size() > 2 ? velocity_cmd.head_commands[2] : 0.0,
        velocity_cmd.head_commands.size() > 3 ? velocity_cmd.head_commands[3] : 0.0);
    }
  }
  else if (update_count_ % (LOG_INTERVAL_CMD_WRITE) == 0)  // Log missing command less frequently
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "No velocity command received on topic '%s'. Robot will hold default pose.",
      velocity_command_topic_.c_str());
  }

  // Format velocity commands for Open Duck Mini (7D: 3 base + 4 head)
  std::vector<double> velocity_commands_7d = {
    velocity_cmd.base_velocity.linear.x,   // lin_vel_x
    velocity_cmd.base_velocity.linear.y,   // lin_vel_y
    velocity_cmd.base_velocity.angular.z,  // ang_vel_z
    velocity_cmd.head_commands.size() > 0 ? velocity_cmd.head_commands[0]
                                          : 0.0,  // head_pos_1 (neck_pitch)
    velocity_cmd.head_commands.size() > 1 ? velocity_cmd.head_commands[1]
                                          : 0.0,  // head_pos_2 (head_pitch)
    velocity_cmd.head_commands.size() > 2 ? velocity_cmd.head_commands[2]
                                          : 0.0,  // head_pos_3 (head_yaw)
    velocity_cmd.head_commands.size() > 3 ? velocity_cmd.head_commands[3]
                                          : 0.0  // head_pos_4 (head_roll)
  };
  observation_formatter_->set_velocity_commands(velocity_commands_7d);

  // Refresh interface name mapping when broadcaster publishes it
  if (auto names_op = rt_interface_names_.try_get(); names_op.has_value())
  {
    observation_formatter_->set_interface_names(names_op.value());
    interface_names_cache_ = names_op.value();
  }

  auto get_state_value = [&](const std::string & full_name) -> std::optional<double>
  {
    if (interface_names_cache_.empty())
    {
      return std::nullopt;
    }
    for (size_t i = 0; i < interface_names_cache_.size(); ++i)
    {
      if (interface_names_cache_[i] == full_name)
      {
        if (i < interface_data.values.size())
        {
          return interface_data.values[i];
        }
        return std::nullopt;
      }
    }
    return std::nullopt;
  };

  const std::string left_contact_raw_name = left_contact_sensor_name_ + "/contact_raw";
  const std::string right_contact_raw_name = right_contact_sensor_name_ + "/contact_raw";
  const std::string left_contact_name = left_contact_sensor_name_ + "/contact";
  const std::string right_contact_name = right_contact_sensor_name_ + "/contact";

  const auto left_contact_raw = get_state_value(left_contact_raw_name);
  const auto right_contact_raw = get_state_value(right_contact_raw_name);
  const auto left_contact_sensor = get_state_value(left_contact_name);
  const auto right_contact_sensor = get_state_value(right_contact_name);

  // Initialize default joint positions from current sensor data (first update only)
  if (!default_joint_positions_initialized_)
  {
    default_joint_positions_ = observation_formatter_->extract_joint_positions(interface_data);
    observation_formatter_->set_default_joint_positions(default_joint_positions_);
    default_joint_positions_initialized_ = true;

    // Reset stabilization counters when default positions are initialized
    stabilization_steps_ = 0;
    onnx_active_steps_ = 0;

    // Initialize motor_targets and prev_motor_targets to default_joint_positions
    // Reference: mujoco_infer.py lines 124-125
    initialize_motor_targets_from_defaults();

    RCLCPP_INFO(
      get_node()->get_logger(), "Default joint positions initialized from sensors (%zu joints)",
      default_joint_positions_.size());
  }

  // Don't process policy until at least one velocity command has been received
  // But still write motor_targets to maintain default pose
  if (!command_received_)
  {
    // Ensure motor_targets are initialized
    if (!prev_motor_targets_initialized_ && default_joint_positions_initialized_)
    {
      initialize_motor_targets_from_defaults();
    }

    // Write motor_targets to hardware to maintain default pose even without command
    if (prev_motor_targets_initialized_ && command_interfaces_.size() == motor_targets_.size())
    {
      write_commands_to_hardware(motor_targets_);
    }
    return return_type::OK;
  }

  // Update imitation phase (gait phase encoding)
  // Reference: mujoco_infer.py lines 137-144
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

  // Contacts:
  // - Default behavior: phase-based contacts (matches training behavior in current controller
  // setup).
  // - Optional: use real contact sensors (use_contact_sensors=true).
  // We also log contact_raw/contact when available so they can be compared against MuJoCo expected
  // logs.
  auto phase = observation_formatter_->get_imitation_phase();
  double sin_phase = phase[1];
  double left_contact = (sin_phase > 0.0) ? 1.0 : 0.0;
  double right_contact = (sin_phase < 0.0) ? 1.0 : 0.0;

  if (use_contact_sensors_ && left_contact_sensor.has_value() && right_contact_sensor.has_value())
  {
    observation_formatter_->set_feet_contacts(
      left_contact_sensor.value(), right_contact_sensor.value());
  }
  else
  {
    observation_formatter_->set_feet_contacts(left_contact, right_contact);
  }

  if (log_contact_sensors_ && update_count_ % LOG_INTERVAL_CONTACT == 0)
  {
    auto fmt_opt = [](const std::optional<double> & v) -> std::string
    {
      if (!v.has_value())
      {
        return "n/a";
      }
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(0) << v.value();
      return ss.str();
    };

    RCLCPP_INFO(
      get_node()->get_logger(),
      "[CONTACT] step=%zu phase=[L=%d R=%d] sensor_raw=[L=%s R=%s] sensor=[L=%s R=%s] "
      "use_contact_sensors=%s",
      update_count_ + 1, static_cast<int>(left_contact), static_cast<int>(right_contact),
      fmt_opt(left_contact_raw).c_str(), fmt_opt(right_contact_raw).c_str(),
      fmt_opt(left_contact_sensor).c_str(), fmt_opt(right_contact_sensor).c_str(),
      use_contact_sensors_ ? "true" : "false");
  }

  // Set motor_targets BEFORE building observation (reference: mujoco_infer.py line 156)
  observation_formatter_->set_motor_targets(motor_targets_);

  // Format inputs for model using ObservationFormatter
  std::vector<float> model_inputs;
  try
  {
    // DEBUG ONLY: Log accelerometer values when injecting state (for comparing ROS2 vs MuJoCo)
    if (ignore_interface_data_updates_.load() && interface_data.values.size() >= 10)
    {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[DEBUG] Using accelerometer for observation: [%.6f, %.6f, %.6f] (skip_inversion=%s)",
        interface_data.values[7], interface_data.values[8], interface_data.values[9],
        "true");  // skip_imu_inversion is set when ignore_interface_data_updates_ is true
    }

    model_inputs =
      observation_formatter_->format(interface_data, velocity_cmd.base_velocity, previous_action_);

    // DEBUG ONLY: Log formatted observation accelerometer values when injecting state
    if (ignore_interface_data_updates_.load() && model_inputs.size() >= 6)
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "[DEBUG] Formatted observation accelerometer: [%.6f, %.6f, %.6f]",
        model_inputs[3], model_inputs[4], model_inputs[5]);
    }

    // DEBUG ONLY: Publish formatted observation for debugging inference quality
    publish_debug_formatted_observation(model_inputs);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000, "Failed to format observation: %s",
      e.what());
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

  // Increment stabilization counter (before logging check)
  stabilization_steps_++;

  // Log raw observation vector (throttled, every LOG_INTERVAL_OBSERVATION steps to match control
  // logging) Only log after stabilization is complete (when observations are used for control)
  if (
    update_count_ % LOG_INTERVAL_OBSERVATION == 0 &&
    stabilization_steps_ >= static_cast<size_t>(stabilization_delay_))
  {
    std::string obs_str = "[RAW_OBS] step=" + std::to_string(update_count_ + 1) + ", obs=[";
    for (size_t i = 0; i < model_inputs.size(); ++i)
    {
      // Format with 3 decimal places for consistency
      char buf[32];
      snprintf(buf, sizeof(buf), "%.3f", model_inputs[i]);
      obs_str += (i == 0 ? "" : ",") + std::string(buf);
    }
    obs_str += "]";
    RCLCPP_INFO(get_node()->get_logger(), "%s", obs_str.c_str());
  }

  // Log observation summary (throttled)
  if (update_count_ % LOG_INTERVAL_VELOCITY == 0)
  {
    double max_joint_pos_rel = 0.0;
    double max_joint_vel = 0.0;
    for (size_t i = 13; i <= 26 && i < model_inputs.size(); ++i)
    {
      max_joint_pos_rel =
        std::max(max_joint_pos_rel, std::abs(static_cast<double>(model_inputs[i])));
    }
    for (size_t i = 27; i <= 40 && i < model_inputs.size(); ++i)
    {
      max_joint_vel = std::max(max_joint_vel, std::abs(static_cast<double>(model_inputs[i])));
    }

    // Reuse phase variable from above (already computed for contact estimation)
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "[OBS] Step %zu: max_joint_pos_rel=%.4f, max_joint_vel=%.4f, contacts=[%.1f,%.1f], "
      "phase=[%.4f,%.4f]",
      update_count_ + 1, max_joint_pos_rel, max_joint_vel, model_inputs[97], model_inputs[98],
      phase[0], phase[1]);
  }

  // During stabilization, hold default positions
  std::vector<double> joint_commands;
  std::vector<double> model_outputs;  // Declare outside if/else for logging
  if (stabilization_steps_ < static_cast<size_t>(stabilization_delay_))
  {
    // Hold default positions during stabilization
    joint_commands = default_joint_positions_;
    // model_outputs remains empty during stabilization

    // Log stabilization progress (throttled)
    if (stabilization_steps_ % LOG_INTERVAL_CONTROL == 0)
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "Stabilization: %zu/%d steps, holding default positions",
        stabilization_steps_, stabilization_delay_);
    }
  }
  else
  {
    // After stabilization, use ONNX model
    if (stabilization_steps_ == static_cast<size_t>(stabilization_delay_))
    {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Stabilization complete! Starting ONNX control (blend-in: %d steps)", blend_in_steps_);
      onnx_active_steps_ = 0;
    }

    // Run model inference (returns relative joint positions)
    model_outputs = run_model_inference(model_inputs);

    // DEBUG ONLY: Publish raw action output for debugging inference quality
    publish_debug_raw_action(model_outputs);

    // Process model outputs: apply scaling and default offset to get absolute positions
    joint_commands = action_processor_->process(model_outputs, default_joint_positions_);

    // Note: Head commands are included in observation (commands[3:7]) for model context,
    // but head joint outputs are controlled by the ONNX model, not overridden by commands.
    // Source: Open_Duck_Playground/playground/open_duck_mini_v2/mujoco_infer.py lines 229-230
    // (head command override is commented out: # head_targets = self.commands[3:], #
    // self.motor_targets[5:9] = head_targets)

    // DEBUG ONLY: Publish processed action for debugging inference quality
    publish_debug_processed_action(joint_commands);

    // Increment ONNX active steps for blend-in
    onnx_active_steps_++;

    // Gradually blend in ONNX actions to prevent sudden movements
    double blend_factor =
      std::min(1.0, static_cast<double>(onnx_active_steps_) / static_cast<double>(blend_in_steps_));
    apply_blend_in(joint_commands, blend_factor);

    // Store current action as previous for next iteration
    previous_action_ = model_outputs;

    // Blend RL actions with reference motion for stability (only after blend-in completes)
    if (onnx_active_steps_ >= static_cast<size_t>(blend_in_steps_))
    {
      apply_reference_motion_blending(joint_commands);
    }
    else if (onnx_active_steps_ == 1)
    {
      smoothed_reference_action_ = joint_commands;
    }

    // DEBUG ONLY: Publish blended action for debugging inference quality
    publish_debug_blended_action(joint_commands);
  }

  // DEBUG ONLY: Publish prev_motor_targets before rate limiting for debugging
  publish_debug_prev_motor_targets(prev_motor_targets_);

  // Store original commands before clamping to detect clipping
  std::vector<double> original_joint_commands = joint_commands;

  // Calculate max_change for logging (before applying rate limiting)
  const double max_change =
    prev_motor_targets_initialized_ ? max_motor_velocity_ * actual_control_period : 0.0;

  // Apply motor speed limits: prevent exceeding physical motor velocity capability
  // Reference: mujoco_infer.py lines 221-226
  // Note: Python code does NOT apply joint position limits, only motor velocity limits
  std::vector<bool> clipped_by_velocity =
    apply_rate_limiting(joint_commands, actual_control_period);

  // DEBUG ONLY: Publish rate-limited action for debugging inference quality
  publish_debug_rate_limited_action(joint_commands);

  // Log clipping events (throttled to avoid spam)
  static int clip_log_counter = 0;
  if (++clip_log_counter % LOG_INTERVAL_CLIP == 0)
  {
    bool any_velocity_clip = false;
    for (size_t i = 0; i < clipped_by_velocity.size(); ++i)
    {
      if (clipped_by_velocity[i])
      {
        any_velocity_clip = true;
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "[CLIP] Joint '%s' clipped by VELOCITY limit: requested=%.4f, clamped=%.4f, prev=%.4f, "
          "max_change=%.4f",
          i < joint_names_.size() ? joint_names_[i].c_str() : "unknown", original_joint_commands[i],
          joint_commands[i], prev_motor_targets_initialized_ ? prev_motor_targets_[i] : 0.0,
          max_change);
      }
    }
    if (!any_velocity_clip && update_count_ % LOG_INTERVAL_VELOCITY == 0)  // Less frequent "all OK"
                                                                           // message
    {
      RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "[CLIP] No joint commands clipped by velocity limit (step %zu)", update_count_ + 1);
    }
  }

  // Update motor_targets and prev_motor_targets after clamping
  // Reference: mujoco_infer.py line 226: prev_motor_targets = motor_targets.copy()
  motor_targets_ = joint_commands;

  // Calculate motor_diff (max absolute difference between current and previous motor_targets)
  double motor_diff = 0.0;
  if (prev_motor_targets_initialized_ && prev_motor_targets_.size() == motor_targets_.size())
  {
    for (size_t i = 0; i < motor_targets_.size(); ++i)
    {
      motor_diff = std::max(motor_diff, std::abs(motor_targets_[i] - prev_motor_targets_[i]));
    }
  }

  // Log control status similar to manual_onnx.log format
  // Format: Control: step=900, motor_diff=0.2747 rad, vel_cmd=[0.09, 0.0, 0.0],
  // motor_targets[0]=-0.0837
  if (update_count_ % LOG_INTERVAL_CONTROL == 0)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Control: step=%zu, motor_diff=%.4f rad, vel_cmd=[%.3f, %.3f, %.3f], motor_targets[0]=%.4f",
      update_count_ + 1, motor_diff, velocity_cmd.base_velocity.linear.x,
      velocity_cmd.base_velocity.linear.y, velocity_cmd.base_velocity.angular.z,
      motor_targets_.size() > 0 ? motor_targets_[0] : 0.0);
  }

  prev_motor_targets_ = joint_commands;

  // Write joint commands to hardware interfaces
  // Reference: mujoco_infer.py line 228/273: self.data.ctrl = self.motor_targets.copy()
  // Note: Python reference does not apply joint position limits, only motor velocity limits
  size_t write_success_count = write_commands_to_hardware(joint_commands);

  // Log command write status (throttled, debug level)
  if (update_count_ % LOG_INTERVAL_CMD_WRITE == 0)
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(), "[CMD WRITE] Successfully wrote %zu/%zu joint commands",
      write_success_count, command_interfaces_.size());
  }

  update_count_++;

  // DEBUG ONLY: Publish update completion marker (after all debug topics are published)
  publish_debug_update_complete();

  return return_type::OK;
}

void MotionController::initialize_motor_targets_from_defaults()
{
  if (!default_joint_positions_initialized_)
  {
    return;
  }

  motor_targets_ = default_joint_positions_;
  prev_motor_targets_ = default_joint_positions_;
  prev_motor_targets_initialized_ = true;
  observation_formatter_->set_motor_targets(motor_targets_);
}

size_t MotionController::write_commands_to_hardware(const std::vector<double> & joint_commands)
{
  size_t write_success_count = 0;
  for (size_t i = 0; i < command_interfaces_.size() && i < joint_commands.size(); ++i)
  {
    const bool write_success = command_interfaces_[i].set_value(joint_commands[i]);
    if (!write_success)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set command for joint '%s' to %.6f", joint_names_[i].c_str(), joint_commands[i]);
    }
    else
    {
      write_success_count++;
    }
  }
  return write_success_count;
}

void MotionController::apply_blend_in(std::vector<double> & joint_commands, double blend_factor)
{
  blend_factor = std::clamp(blend_factor, 0.0, 1.0);
  for (size_t i = 0; i < joint_commands.size() && i < default_joint_positions_.size(); ++i)
  {
    joint_commands[i] =
      (1.0 - blend_factor) * default_joint_positions_[i] + blend_factor * joint_commands[i];
  }
}

void MotionController::apply_reference_motion_blending(std::vector<double> & joint_commands)
{
  if (reference_motion_blend_factor_ <= 0.0)
  {
    return;
  }

  const double smoothing_alpha = 0.1;
  for (size_t i = 0; i < joint_commands.size() && i < smoothed_reference_action_.size(); ++i)
  {
    smoothed_reference_action_[i] =
      smoothing_alpha * joint_commands[i] + (1.0 - smoothing_alpha) * smoothed_reference_action_[i];
    joint_commands[i] = (1.0 - reference_motion_blend_factor_) * joint_commands[i] +
                        reference_motion_blend_factor_ * smoothed_reference_action_[i];
  }
}

std::vector<bool> MotionController::apply_rate_limiting(
  std::vector<double> & joint_commands, double control_period)
{
  std::vector<bool> clipped_by_velocity(joint_commands.size(), false);

  if (!prev_motor_targets_initialized_)
  {
    return clipped_by_velocity;
  }

  const double max_change = max_motor_velocity_ * control_period;
  for (size_t i = 0; i < joint_commands.size(); ++i)
  {
    const double original = joint_commands[i];
    joint_commands[i] = std::clamp(
      joint_commands[i], prev_motor_targets_[i] - max_change, prev_motor_targets_[i] + max_change);
    if (joint_commands[i] != original)
    {
      clipped_by_velocity[i] = true;
    }
  }

  return clipped_by_velocity;
}

// Debug-only helper functions for publishing intermediate values
void MotionController::publish_debug_formatted_observation(const std::vector<float> & model_inputs)
{
  if (!enable_debug_publishing_ || !debug_formatted_observation_publisher_)
  {
    return;
  }

  auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  msg->data.reserve(model_inputs.size());
  for (const auto & val : model_inputs)
  {
    msg->data.push_back(static_cast<double>(val));
  }
  debug_formatted_observation_publisher_->publish(*msg);
}

void MotionController::publish_debug_raw_action(const std::vector<double> & model_outputs)
{
  if (!enable_debug_publishing_ || !debug_raw_action_publisher_)
  {
    return;
  }

  auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  msg->data = model_outputs;
  debug_raw_action_publisher_->publish(*msg);
}

void MotionController::publish_debug_processed_action(const std::vector<double> & joint_commands)
{
  if (!enable_debug_publishing_ || !debug_processed_action_publisher_)
  {
    return;
  }

  auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  msg->data = joint_commands;
  debug_processed_action_publisher_->publish(*msg);
}

void MotionController::publish_debug_blended_action(const std::vector<double> & joint_commands)
{
  if (!enable_debug_publishing_ || !debug_blended_action_publisher_)
  {
    return;
  }

  auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  msg->data = joint_commands;
  debug_blended_action_publisher_->publish(*msg);
}

void MotionController::publish_debug_prev_motor_targets(const std::vector<double> & prev_targets)
{
  if (!enable_debug_publishing_ || !debug_prev_motor_targets_publisher_)
  {
    return;
  }

  auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  msg->data = prev_targets;
  debug_prev_motor_targets_publisher_->publish(*msg);
}

void MotionController::publish_debug_rate_limited_action(const std::vector<double> & joint_commands)
{
  if (!enable_debug_publishing_ || !debug_rate_limited_action_publisher_)
  {
    return;
  }

  auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  msg->data = joint_commands;
  debug_rate_limited_action_publisher_->publish(*msg);
}

void MotionController::publish_debug_update_complete()
{
  if (!enable_debug_publishing_ || !debug_update_complete_publisher_)
  {
    return;
  }

  auto msg = std::make_shared<std_msgs::msg::UInt64>();
  msg->data = update_count_;
  debug_update_complete_publisher_->publish(*msg);
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
    if (
      joint_dim != static_cast<int64_t>(joint_names_.size()) &&
      total_elements != static_cast<int64_t>(joint_names_.size()))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Model output size (last dim: %ld, total elements: %ld) does not match number of joints "
        "(%zu). "
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
      else if (
        joint_dim != static_cast<int64_t>(model_output_size_) &&
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

void MotionController::handle_test_inference(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused

  if (test_observation_.empty())
  {
    response->success = false;
    response->message = "No test observation available. Publish to ~/test_observation topic first.";
    return;
  }

  if (!model_loaded_)
  {
    response->success = false;
    response->message = "Model not loaded.";
    return;
  }

  if (!default_joint_positions_initialized_)
  {
    response->success = false;
    response->message = "Default joint positions not initialized. Controller must be active.";
    return;
  }

  try
  {
    // Run ONNX inference to get raw model output
    test_action_output_ = run_model_inference(test_observation_);

    // Process action through full pipeline to get final hardware commands
    // This matches the processing in update(): action_processor + blending + rate limiting
    std::vector<double> joint_commands =
      action_processor_->process(test_action_output_, default_joint_positions_);

    // Apply blend-in factor (assume fully blended for testing, matching normal flow after blend-in
    // completes)
    double blend_factor = 1.0;  // Full blend for testing (matches normal flow when
                                // onnx_active_steps_ >= blend_in_steps_)
    apply_blend_in(joint_commands, blend_factor);

    // Store current action as previous for next iteration (matches update() behavior)
    previous_action_ = test_action_output_;

    // Blend RL actions with reference motion for stability (matches update() behavior)
    // Only apply if reference motion blending is enabled (assumes fully blended state for testing)
    apply_reference_motion_blending(joint_commands);

    // Apply rate limiting (matches update() behavior)
    // Use training_control_period_ since we don't have actual_control_period in test context
    // Note: This assumes actual_control_period matches training_control_period_ (should be true in
    // normal operation)
    apply_rate_limiting(joint_commands, training_control_period_);

    test_hardware_commands_ = joint_commands;

    // Update prev_motor_targets_ for next test call (matches update() behavior at line 734)
    // This ensures rate limiting uses the correct previous state for subsequent test observations
    prev_motor_targets_ = joint_commands;
    if (!prev_motor_targets_initialized_)
    {
      prev_motor_targets_initialized_ = true;
    }

    // Publish raw action output
    auto action_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    action_msg->data = test_action_output_;
    test_action_output_publisher_->publish(*action_msg);

    // Publish final hardware commands
    auto hw_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    hw_msg->data = test_hardware_commands_;
    test_hardware_commands_publisher_->publish(*hw_msg);

    response->success = true;
    response->message = "Inference successful. Action output and hardware commands published.";
  }
  catch (const std::exception & e)
  {
    response->success = false;
    response->message = std::string("Inference failed: ") + e.what();
  }
}

void MotionController::handle_reset_state(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused

  RCLCPP_INFO(get_node()->get_logger(), "[RESET] Received reset state service request");

  try
  {
    if (!default_joint_positions_initialized_)
    {
      response->success = false;
      response->message = "Default joint positions not initialized. Cannot reset state.";
      return;
    }

    // Reset controller internal state to defaults
    motor_targets_ = default_joint_positions_;
    prev_motor_targets_ = default_joint_positions_;
    prev_motor_targets_initialized_ = true;

    // Reset action history
    previous_action_.assign(joint_names_.size(), 0.0);
    smoothed_reference_action_ = default_joint_positions_;

    // Reset counters
    stabilization_steps_ = 0;
    onnx_active_steps_ = 0;
    update_count_ = 0;

    // Reset observation formatter state
    observation_formatter_->set_motor_targets(motor_targets_);

    // Reset action history in observation formatter
    std::vector<double> zero_action(joint_names_.size(), 0.0);
    observation_formatter_->update_action_history(zero_action);
    observation_formatter_->update_action_history(zero_action);
    observation_formatter_->update_action_history(zero_action);

    // Reset imitation phase
    observation_formatter_->set_imitation_i(0.0);

    // Re-enable IMU inversion (will use normal sensor data after reset)
    observation_formatter_->set_skip_imu_inversion(false);

    // Re-enable interface data updates (will use normal sensor data after reset)
    ignore_interface_data_updates_ = false;

    // Resume updates (in case they were halted after injection)
    update_halted_after_injection_ = false;

    response->success = true;
    response->message = "Controller state reset to defaults. " +
                        std::to_string(joint_names_.size()) + " joints reset.";

    RCLCPP_INFO(
      get_node()->get_logger(),
      "[RESET] Controller state reset: motor_targets, prev_motor_targets, and action history reset "
      "to default positions");
  }
  catch (const std::exception & e)
  {
    response->success = false;
    response->message = std::string("Reset failed: ") + e.what();
  }
}

void MotionController::handle_inject_state(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused

  RCLCPP_INFO(get_node()->get_logger(), "[INJECT] Received injection state service request");

  if (!inject_yaml_file_path_.has_value() || inject_yaml_file_path_->empty())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "[INJECT] YAML file path not received. Publish to ~/inject_yaml_path topic first.");
    response->success = false;
    response->message = "YAML file path not received. Publish to ~/inject_yaml_path topic first.";
    return;
  }

  RCLCPP_INFO(
    get_node()->get_logger(), "[INJECT] Injecting state from YAML: %s",
    inject_yaml_file_path_->c_str());

  // Prevent interface data subscriber from overwriting injected data BEFORE injecting
  // This must be set before parse_and_inject_yaml to avoid race condition
  ignore_interface_data_updates_ = true;

  bool success = parse_and_inject_yaml(inject_yaml_file_path_.value());
  response->success = success;
  if (success)
  {
    // Halt updates after successful injection to prevent data from being overwritten
    update_halted_after_injection_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[INJECT] State injected successfully. Updates halted to preserve injected state.");
    response->message =
      "State injected successfully from YAML file: " + inject_yaml_file_path_.value();
  }
  else
  {
    // Reset flag if injection failed
    ignore_interface_data_updates_ = false;
    RCLCPP_ERROR(get_node()->get_logger(), "[INJECT] Failed to inject state from YAML file.");
    response->message = "Failed to inject state from YAML file. Check logs for details.";
  }
}

void MotionController::handle_resume_updates(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused

  RCLCPP_INFO(get_node()->get_logger(), "[RESUME] Received resume updates service request");

  update_halted_after_injection_ = false;

  // Re-enable interface data updates (will use normal sensor data after resume)
  ignore_interface_data_updates_ = false;

  // Re-enable IMU inversion (will use normal sensor data after resume)
  observation_formatter_->set_skip_imu_inversion(false);

  response->success = true;
  response->message =
    "Updates resumed. Controller will process velocity commands and update state.";

  RCLCPP_INFO(get_node()->get_logger(), "[RESUME] Updates resumed successfully");
}

bool MotionController::parse_and_inject_yaml(const std::string & yaml_file_path)
{
#ifdef HAVE_YAML_CPP
  try
  {
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    // Parse interface data
    if (!config["interface_data"] || !config["interface_data"]["values"])
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing interface_data.values in YAML");
      return false;
    }

    control_msgs::msg::Float64Values interface_data;
    for (const auto & val : config["interface_data"]["values"])
    {
      interface_data.values.push_back(val.as<double>());
    }

    // Parse controller state
    if (!config["controller_state"])
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing controller_state in YAML");
      return false;
    }

    auto state = config["controller_state"];
    size_t N = joint_names_.size();

    // Parse action history
    std::vector<double> last_action, last_last_action, last_last_last_action;
    if (state["last_action"])
    {
      for (const auto & val : state["last_action"]) last_action.push_back(val.as<double>());
    }
    if (state["last_last_action"])
    {
      for (const auto & val : state["last_last_action"])
        last_last_action.push_back(val.as<double>());
    }
    if (state["last_last_last_action"])
    {
      for (const auto & val : state["last_last_last_action"])
        last_last_last_action.push_back(val.as<double>());
    }

    // Parse other state
    std::vector<double> motor_targets;
    if (state["motor_targets"])
    {
      for (const auto & val : state["motor_targets"]) motor_targets.push_back(val.as<double>());
    }

    double imitation_i = state["imitation_i"] ? state["imitation_i"].as<double>() : 0.0;

    std::vector<double> prev_motor_targets;
    if (state["prev_motor_targets"])
    {
      for (const auto & val : state["prev_motor_targets"])
        prev_motor_targets.push_back(val.as<double>());
    }

    size_t onnx_active_steps =
      state["onnx_active_steps"] ? state["onnx_active_steps"].as<size_t>() : 0;
    size_t stabilization_steps =
      state["stabilization_steps"] ? state["stabilization_steps"].as<size_t>() : 0;

    // Validate sizes
    if (
      last_action.size() != N || last_last_action.size() != N ||
      last_last_last_action.size() != N || motor_targets.size() != N ||
      prev_motor_targets.size() != N)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "State array size mismatch. Expected %zu joints, got different sizes.", N);
      return false;
    }

    // Inject interface data into thread-safe box
    rt_interface_data_.set(interface_data);

    // Debug: Log injected accelerometer values to verify they're being used
    if (interface_data.values.size() >= 10)
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "[INJECT] Injected accelerometer: [%.6f, %.6f, %.6f]",
        interface_data.values[7], interface_data.values[8], interface_data.values[9]);
    }

    // Set controller state
    motor_targets_ = motor_targets;
    prev_motor_targets_ = prev_motor_targets;
    prev_motor_targets_initialized_ = true;
    previous_action_ = last_action;
    onnx_active_steps_ = onnx_active_steps;
    stabilization_steps_ = stabilization_steps;
    smoothed_reference_action_ = default_joint_positions_;

    // Set observation formatter state
    observation_formatter_->set_motor_targets(motor_targets_);

    // Set action history (order matters: set oldest first)
    observation_formatter_->update_action_history(last_last_last_action);
    observation_formatter_->update_action_history(last_last_action);
    observation_formatter_->update_action_history(last_action);

    // Set imitation phase
    observation_formatter_->set_imitation_i(imitation_i);

    // Skip IMU inversion for injected data (data from Python already in correct format)
    observation_formatter_->set_skip_imu_inversion(true);

    RCLCPP_INFO(get_node()->get_logger(), "State injected from YAML: %s", yaml_file_path.c_str());
    return true;
  }
  catch (const YAML::Exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "YAML parsing error: %s", e.what());
    return false;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error injecting state: %s", e.what());
    return false;
  }
#else
  RCLCPP_ERROR(get_node()->get_logger(), "yaml-cpp not available. Cannot parse YAML file.");
  return false;
#endif
}

}  // namespace motion_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_controller::MotionController, controller_interface::ControllerInterface)
