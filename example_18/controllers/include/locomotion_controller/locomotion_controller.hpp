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

#ifndef LOCOMOTION_CONTROLLER__LOCOMOTION_CONTROLLER_HPP_
#define LOCOMOTION_CONTROLLER__LOCOMOTION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/interfaces_names.hpp"
#include "control_msgs/msg/interfaces_values.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "locomotion_controller/action_processor.hpp"
#include "locomotion_controller/observation_formatter.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#if defined(ONNXRUNTIME_FOUND) && __has_include("onnxruntime_cxx_api.h")
#include "onnxruntime_cxx_api.h"
#else
#undef ONNXRUNTIME_FOUND
#endif

namespace locomotion_controller
{

class LocomotionController : public controller_interface::ControllerInterface
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

  // Run model inference
  std::vector<double> run_model_inference(const std::vector<float> & inputs);

  // Initialize joint position limits from ros2_control config
  void initialize_joint_limits();

#ifdef ONNXRUNTIME_FOUND
  // Utility functions for ONNX model debugging
  static std::string format_shape_string(const std::vector<int64_t> & shape);
  static const char * get_onnx_type_name(ONNXTensorElementDataType type);
  void log_input_metadata(Ort::Session & session, Ort::AllocatorWithDefaultOptions & allocator);
  void log_output_metadata(Ort::Session & session, Ort::AllocatorWithDefaultOptions & allocator);
  void validate_model_structure(size_t num_inputs, size_t num_outputs);
#endif

  // Parameters
  std::vector<std::string> joint_names_;
  std::string model_path_;
  std::string interfaces_broadcaster_topic_;
  std::string interfaces_broadcaster_names_topic_;
  std::string velocity_command_topic_;

  // Model runtime (ONNX Runtime)
#ifdef ONNXRUNTIME_FOUND
  std::unique_ptr<Ort::Session> onnx_session_;
  std::unique_ptr<Ort::Env> onnx_env_;
  std::unique_ptr<Ort::MemoryInfo> onnx_memory_info_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char *> input_name_ptrs_;
  std::vector<const char *> output_name_ptrs_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
#endif

  // Subscribers
  rclcpp::Subscription<control_msgs::msg::InterfacesValues>::SharedPtr sensor_data_subscriber_;
  rclcpp::Subscription<control_msgs::msg::InterfacesNames>::SharedPtr interfaces_names_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber_;

  // Thread-safe message buffers
  realtime_tools::RealtimeThreadSafeBox<control_msgs::msg::InterfacesValues> rt_sensor_data_;
  realtime_tools::RealtimeThreadSafeBox<std::vector<std::string>> rt_interface_names_;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Twist> rt_velocity_command_;

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

  // Joint position limits (min/max) for clamping commands
  std::vector<double> joint_position_limits_min_;
  std::vector<double> joint_position_limits_max_;
};

}  // namespace locomotion_controller

#endif  // LOCOMOTION_CONTROLLER__LOCOMOTION_CONTROLLER_HPP_
