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

#ifndef MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "control_msgs/msg/float64_values.hpp"
#include "control_msgs/msg/keys.hpp"
#include "controller_interface/controller_interface.hpp"
#include "example_18_motion_controller_msgs/msg/velocity_command_with_head.hpp"
#include "motion_controller/action_processor.hpp"
#include "motion_controller/observation_formatter.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"

#if defined(ONNXRUNTIME_FOUND) && __has_include("onnxruntime_cxx_api.h")
#include "onnxruntime_cxx_api.h"
#else
#undef ONNXRUNTIME_FOUND
#endif

namespace motion_controller
{

class MotionController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Load ML model (ONNX or other format)
  bool load_model(const std::string & model_path);

  // Run model inference - ONNX runtime expects float inputs
  std::vector<double> run_model_inference(const std::vector<float> & inputs);

  // Helper functions to reduce code duplication
  // Initialize motor targets and prev_motor_targets from default positions
  void initialize_motor_targets_from_defaults();

  // Write joint commands to hardware interfaces
  size_t write_commands_to_hardware(const std::vector<double> & joint_commands);

  // Apply blend-in factor to gradually transition from default positions to ONNX actions
  void apply_blend_in(std::vector<double> & joint_commands, double blend_factor);

  // Apply reference motion blending for stability
  void apply_reference_motion_blending(std::vector<double> & joint_commands);

  // Apply motor velocity rate limiting
  std::vector<bool> apply_rate_limiting(
    std::vector<double> & joint_commands, double control_period);

  // Debug-only: Publish intermediate values for debugging
  void publish_debug_formatted_observation(const std::vector<float> & model_inputs);
  void publish_debug_raw_action(const std::vector<double> & model_outputs);
  void publish_debug_processed_action(const std::vector<double> & joint_commands);
  void publish_debug_blended_action(const std::vector<double> & joint_commands);
  void publish_debug_prev_motor_targets(const std::vector<double> & prev_targets);
  void publish_debug_rate_limited_action(const std::vector<double> & joint_commands);
  void publish_debug_update_complete();

#ifdef ONNXRUNTIME_FOUND
  // Utility functions for ONNX model validation
  static std::string format_shape_string(const std::vector<int64_t> & shape);
  static const char * get_onnx_type_name(ONNXTensorElementDataType type);
  void validate_model_structure(size_t num_inputs, size_t num_outputs);
#endif

  // Parameters
  std::vector<std::string> joint_names_;
  std::string model_path_;
  int model_input_size_;   // Expected model input dimension (0 = not specified)
  int model_output_size_;  // Expected model output dimension (0 = not specified)
  std::string interfaces_broadcaster_topic_;
  std::string interfaces_broadcaster_names_topic_;
  std::string velocity_command_topic_;
  bool use_contact_sensors_{false};
  bool log_contact_sensors_{true};
  std::string left_contact_sensor_name_{"left_foot_contact"};
  std::string right_contact_sensor_name_{"right_foot_contact"};
  std::vector<std::string> interface_names_cache_;

  // Model runtime (ONNX Runtime)
#ifdef ONNXRUNTIME_FOUND
  // Session corresponding to the ONNX model
  std::unique_ptr<Ort::Session> onnx_session_;
  // ONNX Runtime environment managing logging and global state
  std::unique_ptr<Ort::Env> onnx_env_;
  // Memory allocation information for tensor creation (CPU, arena allocator)
  std::unique_ptr<Ort::MemoryInfo> onnx_memory_info_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char *> input_name_ptrs_;
  std::vector<const char *> output_name_ptrs_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
#endif

  // Subscribers
  rclcpp::Subscription<control_msgs::msg::Float64Values>::SharedPtr interface_data_subscriber_;
  rclcpp::Subscription<control_msgs::msg::Keys>::SharedPtr interfaces_names_subscriber_;
  rclcpp::Subscription<example_18_motion_controller_msgs::msg::VelocityCommandWithHead>::SharedPtr
    velocity_command_subscriber_;

  // Debug/test infrastructure for diagnosing ONNX model inference quality issues:
  // - Test observation injection and inference verification
  // - State injection/reset for comparing ROS2 vs MuJoCo observation formatting
  // - Isolated inference testing without full control loop
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr test_observation_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_action_output_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_hardware_commands_publisher_;

  // Service: run ONNX inference from test observation (isolated inference test)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_inference_service_;
  void handle_test_inference(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Service: reset controller state (for debugging inference quality)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_state_service_;
  void handle_reset_state(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Service: inject state from YAML file (for comparing ROS2 vs MuJoCo observations)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr inject_state_service_;
  void handle_inject_state(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Service: resume updates after state injection (for debugging inference quality)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_updates_service_;
  void handle_resume_updates(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Topic: receive YAML file path for state injection
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inject_yaml_path_subscriber_;

  // Store YAML file path for state injection
  std::optional<std::string> inject_yaml_file_path_;

  // Helper: parse YAML and inject state (for debugging inference quality)
  bool parse_and_inject_yaml(const std::string & yaml_file_path);

  // Store test observation and action output
  std::vector<float> test_observation_;
  std::vector<double> test_action_output_;
  std::vector<double> test_hardware_commands_;

  // Debug publishers for intermediate values in update() loop
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    debug_formatted_observation_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_raw_action_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_processed_action_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_blended_action_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    debug_rate_limited_action_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    debug_prev_motor_targets_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr debug_update_complete_publisher_;

  // Flag to enable/disable debug publishing
  bool enable_debug_publishing_;

  // Flag to halt updates after state injection (for debugging/comparison)
  std::atomic<bool> update_halted_after_injection_;

  // Flag to prevent interface data subscriber from overwriting injected data
  std::atomic<bool> ignore_interface_data_updates_;

  // Thread-safe message buffers
  realtime_tools::RealtimeThreadSafeBox<control_msgs::msg::Float64Values> rt_interface_data_;
  realtime_tools::RealtimeThreadSafeBox<std::vector<std::string>> rt_interface_names_;
  realtime_tools::RealtimeThreadSafeBox<
    example_18_motion_controller_msgs::msg::VelocityCommandWithHead>
    rt_velocity_command_;

  // Internal state
  std::vector<double> previous_action_;
  bool model_loaded_;

  // Command interface names
  std::vector<std::string> command_interface_names_;

  // Observation formatter for ML model input
  std::unique_ptr<ObservationFormatter> observation_formatter_;

  // Action processor for ML model output (scaling and default offset)
  std::unique_ptr<ActionProcessor> action_processor_;

  // Default joint positions (used for action offset, initialized from current positions)
  std::vector<double> default_joint_positions_;
  bool default_joint_positions_initialized_;

  // Motor speed limits: prevent exceeding physical motor velocity capability
  double max_motor_velocity_;  // rad/s (default: 5.24 from reference implementation)
  std::vector<double>
    motor_targets_;  // Current motor targets (used in observation, matches reference)
  std::vector<double> prev_motor_targets_;  // Previous joint commands for rate limiting
  bool prev_motor_targets_initialized_;     // Track if prev_motor_targets_ has been initialized
  bool command_received_;  // Track if any velocity command has been received (to avoid moving
                           // before command)

  // 1st attempt to tackle model stability
  double reference_motion_blend_factor_;  // Weight for blending RL actions with reference motion
                                          // (0.0-1.0)
  std::vector<double>
    smoothed_reference_action_;  // Smoothed previous actions used as reference baseline

  // Open Duck Mini specific parameters
  double phase_frequency_factor_offset_;
  double phase_period_;             // Phase period (nb_steps_in_period) for logging
  double training_control_period_;  // Training control period (default: 0.02s for 50Hz)
  double gyro_deadband_;            // Gyro deadband threshold (rad/s)
  int stabilization_delay_;         // Steps to wait before applying ONNX actions
  int blend_in_steps_;              // Steps to gradually blend in ONNX actions
  size_t stabilization_steps_;      // Current stabilization step counter
  size_t onnx_active_steps_;        // Steps since ONNX started

  // Update counter
  size_t update_count_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
