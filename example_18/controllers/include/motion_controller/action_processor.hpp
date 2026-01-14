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

#ifndef MOTION_CONTROLLER__ACTION_PROCESSOR_HPP_
#define MOTION_CONTROLLER__ACTION_PROCESSOR_HPP_

#include <string>
#include <vector>

namespace motion_controller
{

class ActionProcessor
{
public:
  ActionProcessor(
    const std::vector<std::string> & joint_names, double action_scale = 0.25,
    bool use_default_offset = true);

  // Process model outputs: apply scaling and default offset
  // Model outputs are relative joint positions (scaled by action_scale)
  // Returns absolute joint positions ready for hardware command interfaces
  // Reference: Open Duck Mini v2_rl_walk_mujoco.py lines 290, 310-311
  std::vector<double> process(
    const std::vector<double> & model_outputs, const std::vector<double> & default_joint_positions);

  // Process model outputs with head commands
  // Head commands (4D) are added to head joint targets
  // Reference: Open Duck Mini v2_rl_walk_mujoco.py line 310-311
  std::vector<double> process_with_head_commands(
    const std::vector<double> & model_outputs, const std::vector<double> & default_joint_positions,
    const std::vector<double> & head_commands);

  // Set head joint indices (which joints are controlled by head commands)
  // Default: indices 5-9 (4 joints) for Open Duck Mini
  void set_head_joint_indices(const std::vector<size_t> & head_joint_indices);

  double get_action_scale() const { return action_scale_; }
  bool get_use_default_offset() const { return use_default_offset_; }

private:
  std::vector<std::string> joint_names_;
  size_t num_joints_;
  double action_scale_;      // Scale factor for model outputs (default: 0.25 from env_cfg.py)
  bool use_default_offset_;  // Whether to add default joint positions (default: true)
  std::vector<size_t>
    head_joint_indices_;  // Indices of head joints (default: 5-9 for 4 head joints)
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__ACTION_PROCESSOR_HPP_
