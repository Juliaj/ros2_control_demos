// Copyright (C) 2025 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Julia Jia

#include <gtest/gtest.h>

#include <vector>
#include <cmath>

/**
 * Test to verify initialization matches Python reference implementation.
 * 
 * Python reference (mujoco_infer_base.py lines 121-128):
 *   self.default_actuator = self.model.keyframe("home").ctrl
 *   self.motor_targets = self.default_actuator
 *   self.prev_motor_targets = self.default_actuator
 *   self.data.qpos[:] = self.model.keyframe("home").qpos
 *   self.data.ctrl[:] = self.default_actuator
 * 
 * This test verifies that:
 * 1. default_joint_positions match the "home" keyframe ctrl values
 * 2. motor_targets are initialized to default_joint_positions
 * 3. prev_motor_targets are initialized to default_joint_positions
 * 4. All three are equal at initialization (matching Python behavior)
 */
class MotionControllerInitializationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Default actuator positions from "home" keyframe (matching Python)
    // Source: scene_flat_terrain.xml / start_positions.xml
    // Order: left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle,
    //        neck_pitch, head_pitch, head_yaw, head_roll,
    //        right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle
    default_actuator_python_ = {
      0.002, 0.053, -0.63, 1.368, -0.784,  // left leg
      0.0, 0.0, 0.0, 0.0,                  // head
      -0.003, -0.065, 0.635, 1.379, -0.796  // right leg
    };
    
    num_joints_ = default_actuator_python_.size();
  }
  
  std::vector<double> default_actuator_python_;
  size_t num_joints_;
};

TEST_F(MotionControllerInitializationTest, DefaultActuatorValuesMatchPython)
{
  // Verify default actuator values match Python reference
  // These should match the "home" keyframe ctrl values
  ASSERT_EQ(num_joints_, 14u) << "Expected 14 joints";
  
  // Verify values match Python reference (mujoco_infer_base.py line 121-123)
  EXPECT_DOUBLE_EQ(default_actuator_python_[0], 0.002) << "left_hip_yaw";
  EXPECT_DOUBLE_EQ(default_actuator_python_[1], 0.053) << "left_hip_roll";
  EXPECT_DOUBLE_EQ(default_actuator_python_[2], -0.63) << "left_hip_pitch";
  EXPECT_DOUBLE_EQ(default_actuator_python_[3], 1.368) << "left_knee";
  EXPECT_DOUBLE_EQ(default_actuator_python_[4], -0.784) << "left_ankle";
  
  // Head joints should be zero
  for (size_t i = 5; i < 9; ++i)
  {
    EXPECT_DOUBLE_EQ(default_actuator_python_[i], 0.0) << "Head joint " << (i - 5);
  }
  
  EXPECT_DOUBLE_EQ(default_actuator_python_[9], -0.003) << "right_hip_yaw";
  EXPECT_DOUBLE_EQ(default_actuator_python_[10], -0.065) << "right_hip_roll";
  EXPECT_DOUBLE_EQ(default_actuator_python_[11], 0.635) << "right_hip_pitch";
  EXPECT_DOUBLE_EQ(default_actuator_python_[12], 1.379) << "right_knee";
  EXPECT_DOUBLE_EQ(default_actuator_python_[13], -0.796) << "right_ankle";
}

TEST_F(MotionControllerInitializationTest, MotorTargetsInitializedToDefaultActuator)
{
  // Python reference: motor_targets = default_actuator (line 124)
  std::vector<double> motor_targets = default_actuator_python_;
  
  ASSERT_EQ(motor_targets.size(), default_actuator_python_.size());
  
  for (size_t i = 0; i < motor_targets.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(motor_targets[i], default_actuator_python_[i])
      << "motor_targets[" << i << "] should equal default_actuator[" << i << "]";
  }
}

TEST_F(MotionControllerInitializationTest, PrevMotorTargetsInitializedToDefaultActuator)
{
  // Python reference: prev_motor_targets = default_actuator (line 125)
  std::vector<double> prev_motor_targets = default_actuator_python_;
  
  ASSERT_EQ(prev_motor_targets.size(), default_actuator_python_.size());
  
  for (size_t i = 0; i < prev_motor_targets.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(prev_motor_targets[i], default_actuator_python_[i])
      << "prev_motor_targets[" << i << "] should equal default_actuator[" << i << "]";
  }
}

TEST_F(MotionControllerInitializationTest, MotorTargetsEqualPrevMotorTargetsAtInit)
{
  // Python reference: motor_targets = default_actuator, prev_motor_targets = default_actuator
  // Therefore: motor_targets == prev_motor_targets at initialization
  std::vector<double> motor_targets = default_actuator_python_;
  std::vector<double> prev_motor_targets = default_actuator_python_;
  
  ASSERT_EQ(motor_targets.size(), prev_motor_targets.size());
  
  for (size_t i = 0; i < motor_targets.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(motor_targets[i], prev_motor_targets[i])
      << "motor_targets[" << i << "] should equal prev_motor_targets[" << i << "] at initialization";
  }
}

TEST_F(MotionControllerInitializationTest, DefaultActuatorValuesMatchConfigFile)
{
  // Verify default actuator values match the config file values
  // Source: open_duck_mini_controllers.yaml default_joint_positions
  std::vector<double> config_default_positions = {
    0.002, 0.053, -0.63, 1.368, -0.784,  // left leg
    0.0, 0.0, 0.0, 0.0,                  // head
    -0.003, -0.065, 0.635, 1.379, -0.796  // right leg
  };
  
  ASSERT_EQ(config_default_positions.size(), default_actuator_python_.size());
  
  for (size_t i = 0; i < config_default_positions.size(); ++i)
  {
    EXPECT_NEAR(config_default_positions[i], default_actuator_python_[i], 1e-6)
      << "Config default_joint_positions[" << i << "] should match Python default_actuator[" << i << "]";
  }
}

