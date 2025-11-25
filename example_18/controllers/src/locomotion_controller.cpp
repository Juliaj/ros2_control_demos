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

#include "locomotion_controller/locomotion_controller.hpp"

#include <algorithm>
#include <limits>

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace locomotion_controller
{

controller_interface::CallbackReturn LocomotionController::on_init()
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
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  model_loaded_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LocomotionController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration LocomotionController::state_interface_configuration()
  const
{
  // This controller doesn't need state interfaces - it subscribes to topics
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn LocomotionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Read parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  model_path_ = get_node()->get_parameter("model_path").as_string();
  interfaces_broadcaster_topic_ =
    get_node()->get_parameter("interfaces_broadcaster_topic").as_string();
  interfaces_broadcaster_names_topic_ =
    get_node()->get_parameter("interfaces_broadcaster_names_topic").as_string();
  velocity_command_topic_ = get_node()->get_parameter("velocity_command_topic").as_string();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (model_path_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model path not specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Build command interface names: joint_name/position for each joint
  command_interface_names_.clear();
  for (const auto & joint_name : joint_names_)
  {
    command_interface_names_.push_back(joint_name + "/position");
  }

  // Initialize previous action to zeros
  previous_action_.resize(joint_names_.size(), 0.0);

  // Initialize observation formatter
  observation_formatter_ = std::make_unique<ObservationFormatter>(joint_names_);

  // Initialize action processor (scale=0.25, use_default_offset=true from env_cfg.py)
  action_processor_ = std::make_unique<ActionProcessor>(joint_names_, 0.25, true);

  // Initialize default joint positions (will be set from sensor data on first update)
  default_joint_positions_.resize(joint_names_.size(), 0.0);
  default_joint_positions_initialized_ = false;

  // Load model
  if (!load_model(model_path_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load model from: %s", model_path_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Create subscribers
  sensor_data_subscriber_ = get_node()->create_subscription<control_msgs::msg::InterfacesValues>(
    interfaces_broadcaster_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const control_msgs::msg::InterfacesValues::SharedPtr msg)
    { rt_sensor_data_.set(*msg); });

  interfaces_names_subscriber_ =
    get_node()->create_subscription<control_msgs::msg::InterfacesNames>(
      interfaces_broadcaster_names_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      [this](const control_msgs::msg::InterfacesNames::SharedPtr msg)
      { rt_interface_names_.set(msg->names); });

  velocity_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    velocity_command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { rt_velocity_command_.set(*msg); });

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LocomotionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!model_loaded_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model not loaded");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Validate command interfaces
  if (command_interfaces_.size() != command_interface_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LocomotionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LocomotionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get latest sensor data from thread-safe buffer
  auto sensor_data_op = rt_sensor_data_.try_get();
  if (!sensor_data_op.has_value())
  {
    return controller_interface::return_type::OK;
  }
  control_msgs::msg::InterfacesValues sensor_data = sensor_data_op.value();

  // Get latest velocity command from thread-safe buffer
  auto velocity_cmd_op = rt_velocity_command_.try_get();
  geometry_msgs::msg::Twist velocity_cmd;
  if (velocity_cmd_op.has_value())
  {
    velocity_cmd = velocity_cmd_op.value();
  }

  // Refresh interface name mapping when broadcaster publishes it
  if (auto names_op = rt_interface_names_.try_get(); names_op.has_value())
  {
    observation_formatter_->set_interface_names(names_op.value());
  }

  // Initialize default joint positions from current sensor data (first update only)
  if (!default_joint_positions_initialized_)
  {
    default_joint_positions_ = observation_formatter_->extract_joint_positions(sensor_data);
    observation_formatter_->set_default_joint_positions(default_joint_positions_);
    default_joint_positions_initialized_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(), "Initialized default joint positions from current sensor data");
  }

  // Format inputs for model using ObservationFormatter
  std::vector<float> model_inputs =
    observation_formatter_->format(sensor_data, velocity_cmd, previous_action_);

  // Run model inference (returns relative joint positions, scaled by 0.25)
  std::vector<double> model_outputs = run_model_inference(model_inputs);

  // Process model outputs: apply scaling and default offset to get absolute positions
  std::vector<double> joint_commands =
    action_processor_->process(model_outputs, default_joint_positions_);

  // Store current action as previous for next iteration (use raw model outputs, not processed)
  previous_action_ = model_outputs;

  // Write joint commands to hardware interfaces
  for (size_t i = 0; i < command_interfaces_.size() && i < joint_commands.size(); ++i)
  {
    const bool write_success = command_interfaces_[i].set_value(joint_commands[i]);
    (void)write_success;
  }

  return controller_interface::return_type::OK;
}

bool LocomotionController::load_model(const std::string & model_path)
{
#ifdef ONNXRUNTIME_FOUND
  try
  {
    // Initialize ONNX Runtime environment
    onnx_env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LocomotionController");
    onnx_memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    // Create session options
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);

    // Create session from model file
    onnx_session_ = std::make_unique<Ort::Session>(onnx_env_, model_path.c_str(), session_options);

    // Get input/output names and shapes
    Ort::AllocatorWithDefaultOptions allocator;

    // Get input info
    size_t num_input_nodes = onnx_session_->GetInputCount();
    if (num_input_nodes != 1)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Model expects %zu inputs, expected 1", num_input_nodes);
      return false;
    }

    auto input_name = onnx_session_->GetInputNameAllocated(0, allocator);
    input_names_.push_back(input_name.get());
    auto input_type_info = onnx_session_->GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    input_shape_ = input_tensor_info.GetShape();

    // Get output info
    size_t num_output_nodes = onnx_session_->GetOutputCount();
    if (num_output_nodes != 1)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Model expects %zu outputs, expected 1", num_output_nodes);
      return false;
    }

    auto output_name = onnx_session_->GetOutputNameAllocated(0, allocator);
    output_names_.push_back(output_name.get());
    auto output_type_info = onnx_session_->GetOutputTypeInfo(0);
    auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
    output_shape_ = output_tensor_info.GetShape();

    // Validate model structure
    size_t expected_input_size = observation_formatter_->get_observation_dim();
    size_t model_input_size = 1;
    for (auto dim : input_shape_)
    {
      if (dim > 0)
      {
        model_input_size *= dim;
      }
    }

    if (model_input_size != expected_input_size)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Model input size (%zu) does not match expected observation dimension (%zu)",
        model_input_size, expected_input_size);
    }

    if (output_shape_.empty() || output_shape_[0] != static_cast<int64_t>(joint_names_.size()))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Model output size may not match number of joints (%zu)",
        joint_names_.size());
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

std::vector<double> LocomotionController::run_model_inference(const std::vector<float> & inputs)
{
#ifdef ONNXRUNTIME_FOUND
  if (!model_loaded_ || !onnx_session_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model not loaded");
    return std::vector<double>(joint_names_.size(), 0.0);
  }

  try
  {
    // Prepare input tensor
    size_t input_size = inputs.size();
    std::vector<int64_t> input_shape = {1, static_cast<int64_t>(input_size)};

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      onnx_memory_info_, const_cast<float *>(inputs.data()), input_size, input_shape.data(),
      input_shape.size());

    // Run inference
    auto output_tensors = onnx_session_->Run(
      Ort::RunOptions{nullptr}, input_names_.data(), &input_tensor, 1, output_names_.data(), 1);

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
    RCLCPP_ERROR(get_node()->get_logger(), "Model inference failed: %s", e.what());
    return std::vector<double>(joint_names_.size(), 0.0);
  }
#else
  (void)inputs;
  RCLCPP_WARN(get_node()->get_logger(), "ONNX Runtime not available, returning zeros");
  return std::vector<double>(joint_names_.size(), 0.0);
#endif
}

}  // namespace locomotion_controller

PLUGINLIB_EXPORT_CLASS(
  locomotion_controller::LocomotionController, controller_interface::ControllerInterface)
