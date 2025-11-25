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

#include <gtest/gtest.h>

#include <vector>

#include "locomotion_controller/action_processor.hpp"

class TestActionProcessor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    joint_names_ = {
      "leg_left_hip_roll_joint",    "leg_left_hip_yaw_joint",      "leg_left_hip_pitch_joint",
      "leg_left_knee_pitch_joint",  "leg_left_ankle_pitch_joint",  "leg_left_ankle_roll_joint",
      "leg_right_hip_roll_joint",   "leg_right_hip_yaw_joint",     "leg_right_hip_pitch_joint",
      "leg_right_knee_pitch_joint", "leg_right_ankle_pitch_joint", "leg_right_ankle_roll_joint"};
    num_joints_ = joint_names_.size();
  }

  std::vector<std::string> joint_names_;
  size_t num_joints_;
};

TEST_F(TestActionProcessor, ScaleAndOffset)
{
  // Create processor with scale=0.25 and use_default_offset=true (from env_cfg.py)
  locomotion_controller::ActionProcessor processor(joint_names_, 0.25, true);

  // Model outputs (relative, scaled by 0.25 in training)
  std::vector<double> model_outputs(num_joints_, 0.4);  // Example: 0.4 relative position

  // Default joint positions
  std::vector<double> default_positions(num_joints_, 0.5);

  // Process actions
  std::vector<double> processed = processor.process(model_outputs, default_positions);

  // Verify: processed = model_output * scale + default_offset
  // processed = 0.4 * 0.25 + 0.5 = 0.1 + 0.5 = 0.6
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_DOUBLE_EQ(processed[i], 0.6);
  }
}

TEST_F(TestActionProcessor, ScaleOnly)
{
  // Create processor with scale=0.25 and use_default_offset=false
  locomotion_controller::ActionProcessor processor(joint_names_, 0.25, false);

  std::vector<double> model_outputs(num_joints_, 0.4);
  std::vector<double> default_positions(num_joints_, 0.5);

  std::vector<double> processed = processor.process(model_outputs, default_positions);

  // Verify: processed = model_output * scale (no offset)
  // processed = 0.4 * 0.25 = 0.1
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_DOUBLE_EQ(processed[i], 0.1);
  }
}

TEST_F(TestActionProcessor, ZeroScale)
{
  locomotion_controller::ActionProcessor processor(joint_names_, 1.0, true);

  std::vector<double> model_outputs(num_joints_, 0.2);
  std::vector<double> default_positions(num_joints_, 0.3);

  std::vector<double> processed = processor.process(model_outputs, default_positions);

  // Verify: processed = 0.2 * 1.0 + 0.3 = 0.5
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_DOUBLE_EQ(processed[i], 0.5);
  }
}

TEST_F(TestActionProcessor, NegativeValues)
{
  locomotion_controller::ActionProcessor processor(joint_names_, 0.25, true);

  std::vector<double> model_outputs(num_joints_, -0.2);  // Negative relative position
  std::vector<double> default_positions(num_joints_, 0.5);

  std::vector<double> processed = processor.process(model_outputs, default_positions);

  // Verify: processed = -0.2 * 0.25 + 0.5 = -0.05 + 0.5 = 0.45
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_DOUBLE_EQ(processed[i], 0.45);
  }
}

TEST_F(TestActionProcessor, SizeMismatch)
{
  locomotion_controller::ActionProcessor processor(joint_names_, 0.25, true);

  std::vector<double> wrong_size_outputs(num_joints_ + 1, 0.1);
  std::vector<double> default_positions(num_joints_, 0.5);

  // Should throw exception
  EXPECT_THROW(processor.process(wrong_size_outputs, default_positions), std::invalid_argument);
}

TEST_F(TestActionProcessor, GetParameters)
{
  locomotion_controller::ActionProcessor processor(joint_names_, 0.25, true);

  EXPECT_DOUBLE_EQ(processor.get_action_scale(), 0.25);
  EXPECT_TRUE(processor.get_use_default_offset());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
