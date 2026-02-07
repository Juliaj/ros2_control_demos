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

#include "motion_controller/action_processor.hpp"

#include <algorithm>
#include <stdexcept>

namespace motion_controller
{

ActionProcessor::ActionProcessor(
  const std::vector<std::string> & joint_names, double action_scale, bool use_default_offset)
: joint_names_(joint_names),
  num_joints_(joint_names.size()),
  action_scale_(action_scale),
  use_default_offset_(use_default_offset)
{
  if (joint_names.empty())
  {
    throw std::invalid_argument("Joint names cannot be empty");
  }

  // Default head joint indices (unused: kept for potential future use)
  // Source: Open_Duck_Playground/playground/open_duck_mini_v2/mujoco_infer.py lines 147-150
  // (commands[3:7] for neck_pitch, head_pitch, head_yaw, head_roll)
  // NOTE: Currently unused - head commands are in observation but not applied to override model
  // outputs (matching mujoco_infer.py where head override is commented out at lines 229-230)
  if (num_joints_ >= 9)
  {
    head_joint_indices_ = {5, 6, 7, 8};
  }
}

std::vector<double> ActionProcessor::process(
  const std::vector<double> & model_outputs, const std::vector<double> & default_joint_positions)
{
  if (model_outputs.size() != num_joints_)
  {
    throw std::invalid_argument(
      "Model outputs size (" + std::to_string(model_outputs.size()) +
      ") does not match number of joints (" + std::to_string(num_joints_) + ")");
  }

  if (use_default_offset_ && default_joint_positions.size() != num_joints_)
  {
    throw std::invalid_argument(
      "Default joint positions size (" + std::to_string(default_joint_positions.size()) +
      ") does not match number of joints (" + std::to_string(num_joints_) + ")");
  }

  std::vector<double> processed_actions;
  processed_actions.reserve(num_joints_);

  for (size_t i = 0; i < num_joints_; ++i)
  {
    // Apply scaling: model outputs are scaled by action_scale (default: 0.25)
    // Source: Open_Duck_Playground/playground/open_duck_mini_v2/mujoco_infer.py line 31
    // (action_scale = 0.25)
    double scaled_action = model_outputs[i] * action_scale_;

    // Add default offset if enabled (model outputs are relative to default positions)
    if (use_default_offset_)
    {
      scaled_action += default_joint_positions[i];
    }

    processed_actions.push_back(scaled_action);
  }

  return processed_actions;
}

void ActionProcessor::set_head_joint_indices(const std::vector<size_t> & head_joint_indices)
{
  // Validate indices
  for (const auto & idx : head_joint_indices)
  {
    if (idx >= num_joints_)
    {
      throw std::invalid_argument(
        "Head joint index " + std::to_string(idx) +
        " is out of range (num_joints=" + std::to_string(num_joints_) + ")");
    }
  }
  head_joint_indices_ = head_joint_indices;
}

}  // namespace motion_controller
