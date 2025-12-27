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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

// Test fixture for MuJoCo-based controller testing
// 
// This test verifies MuJoCo simulation works correctly with the robot model.
// For full controller integration testing, this would need to be extended with:
// 1. MuJoCo hardware interface (mujoco_ros2_control) integration
// 2. State interfaces broadcaster setup to provide sensor data
// 3. Controller manager lifecycle (configure, activate)
// 4. Controller update loop with proper timing
// 5. Velocity command publishing
//
// Reference: validate_onnx_simulation.py shows the expected behavior
class MotionControllerMuJoCoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize MuJoCo model
    // Model path should point to the MuJoCo XML file
    // Can be overridden via MUJOCO_MODEL_PATH environment variable
    const char * env_path = getenv("MUJOCO_MODEL_PATH");
    std::string model_path;
    if (env_path)
    {
      model_path = env_path;
    }
    else
    {
      model_path = 
        std::string(getenv("HOME")) + "/dev/Open_Duck_Playground/playground/open_duck_mini_v2/xmls/scene_flat_terrain.xml";
    }
    
    char error[1000] = "Could not load binary model";
    model_ = mj_loadXML(model_path.c_str(), 0, error, 1000);
    
    if (!model_)
    {
      GTEST_SKIP() << "Could not load MuJoCo model from: " << model_path 
                   << ". Error: " << error;
    }
    
    data_ = mj_makeData(model_);
    
    // Set simulation parameters
    sim_dt_ = 0.002;  // 2ms timestep
    decimation_ = 10;  // Control at 50Hz (10 * 2ms = 20ms)
    model_->opt.timestep = sim_dt_;
    
    // Initialize robot to "home" keyframe pose (matching reference implementation)
    // Reference: validate_onnx_simulation.py line 127-128:
    //   self.data.qpos[:] = self.model.keyframe("home").qpos
    //   self.data.ctrl[:] = self.default_actuator
    int home_keyframe_id = mj_name2id(model_, mjOBJ_KEY, "home");
    int keyframe_id = (home_keyframe_id >= 0) ? home_keyframe_id : 0;  // Use "home" or first keyframe
    
    // Store default actuator positions for stability test
    default_actuator_.resize(model_->nu, 0.0);
    
    if (model_->nkey > 0 && keyframe_id < model_->nkey)
    {
      // Copy qpos from keyframe
      mju_copy(data_->qpos, model_->key_qpos + keyframe_id * model_->nq, model_->nq);
      
      // Copy ctrl from keyframe (default actuator positions)
      if (model_->nkey > 0 && model_->key_ctrl)
      {
        mju_copy(data_->ctrl, model_->key_ctrl + keyframe_id * model_->nu, model_->nu);
        // Store default actuator positions for continuous control
        mju_copy(default_actuator_.data(), model_->key_ctrl + keyframe_id * model_->nu, model_->nu);
      }
      
      // Forward kinematics to update dependent quantities
      mj_forward(model_, data_);
    }
    
    // Initialize simulation (step once to settle)
    mj_step(model_, data_);
    
    // Get number of actuators (joints)
    num_joints_ = model_->nu;
  }
  
  void TearDown() override
  {
    if (data_)
    {
      mj_deleteData(data_);
    }
    if (model_)
    {
      mj_deleteModel(model_);
    }
  }
  
  // Simulate one control step with control inputs
  void simulateStep()
  {
    // Apply control inputs (default actuator positions) to maintain stability
    // Reference: validate_onnx_simulation.py applies motor_targets = default_actuator + action * scale
    // For stability test, we use default_actuator (action = 0)
    if (!default_actuator_.empty() && default_actuator_.size() == static_cast<size_t>(model_->nu))
    {
      mju_copy(data_->ctrl, default_actuator_.data(), model_->nu);
    }
    
    // Run MuJoCo simulation for decimation steps
    for (int i = 0; i < decimation_; ++i)
    {
      mj_step(model_, data_);
    }
  }
  
  // Get body height (z position of floating base)
  double getBodyHeight()
  {
    // Find floating base body (first body with type 0)
    int floating_base_id = -1;
    for (int i = 0; i < model_->nbody; ++i)
    {
      if (model_->body_jntnum[i] > 0)
      {
        int jnt_id = model_->body_jntadr[i];
        if (model_->jnt_type[jnt_id] == mjJNT_FREE)
        {
          floating_base_id = i;
          break;
        }
      }
    }
    
    if (floating_base_id >= 0)
    {
      return data_->xpos[floating_base_id * 3 + 2];  // z coordinate
    }
    return 0.0;
  }
  
  // Get forward position (x position of floating base)
  double getForwardPosition()
  {
    int floating_base_id = -1;
    for (int i = 0; i < model_->nbody; ++i)
    {
      if (model_->body_jntnum[i] > 0)
      {
        int jnt_id = model_->body_jntadr[i];
        if (model_->jnt_type[jnt_id] == mjJNT_FREE)
        {
          floating_base_id = i;
          break;
        }
      }
    }
    
    if (floating_base_id >= 0)
    {
      return data_->xpos[floating_base_id * 3];  // x coordinate
    }
    return 0.0;
  }
  
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
  double sim_dt_;
  int decimation_;
  int num_joints_;
  std::vector<double> default_actuator_;  // Default actuator positions from home keyframe
};

// Basic test: Controller can be configured
TEST_F(MotionControllerMuJoCoTest, ControllerConfiguration)
{
  // This is a placeholder test - full integration requires hardware interface setup
  // which is complex with MuJoCo. For now, we verify the model loads correctly.
  ASSERT_NE(model_, nullptr);
  ASSERT_NE(data_, nullptr);
  EXPECT_GT(num_joints_, 0);
}

// Test: MuJoCo simulation model loading and basic physics
// Reference: validate_onnx_simulation.py - tests robot stability WITH active control
// This test verifies the MuJoCo model loads and simulates correctly
// Note: Without active control, robot will fall under gravity (expected behavior)
// Full controller integration test would require hardware interface setup and active control
TEST_F(MotionControllerMuJoCoTest, SimulationStability)
{
  // Skip if model not loaded
  if (!model_ || !data_)
  {
    GTEST_SKIP() << "MuJoCo model not loaded";
  }
  
  // Initial state - verify model loaded correctly
  double start_z = getBodyHeight();
  double start_x = getForwardPosition();
  
  // Verify robot is initialized at reasonable height
  // Reference: validate_onnx_simulation.py expects robot to start above ground
  EXPECT_GT(start_z, 0.0) << "Robot should start above ground (got " << start_z << "m)";
  
  // Simulate with control inputs (default actuator positions) to test stability
  // Reference: validate_onnx_simulation.py uses ONNX model for control, we use default pose
  // This tests that the robot can maintain stability when held at default pose
  const int num_steps = 500;  // 1 second (500 * 2ms * 10 decimation)
  const double fall_threshold = 0.15;  // meters (below this is considered fallen)
  const int fall_duration_steps = 50;  // Consecutive steps below threshold to consider fallen
  int fall_steps_below_threshold = 0;
  bool fall_detected = false;
  int fall_step = -1;
  
  // Run simulation with control inputs
  for (int step = 0; step < num_steps; ++step)
  {
    simulateStep();
    
    double current_z = getBodyHeight();
    
    // Verify simulation is progressing (no NaN or invalid values)
    EXPECT_FALSE(std::isnan(current_z)) << "Body height should not be NaN at step " << step;
    EXPECT_FALSE(std::isinf(current_z)) << "Body height should not be infinite at step " << step;
    
    // Track fall detection (same logic as reference)
    if (current_z < fall_threshold)
    {
      fall_steps_below_threshold++;
      if (fall_steps_below_threshold >= fall_duration_steps && !fall_detected)
      {
        fall_detected = true;
        fall_step = step;
        break;
      }
    }
    else
    {
      fall_steps_below_threshold = 0;
    }
  }
  
  // Verify robot maintains stability
  double end_z = getBodyHeight();
  double end_x = getForwardPosition();
  double forward_distance = end_x - start_x;
  
  // Log simulation results
  std::cout << "Simulation results:" << std::endl;
  std::cout << "  Duration: " << (num_steps * sim_dt_ * decimation_) << " seconds" << std::endl;
  std::cout << "  Start position: x=" << start_x << "m, z=" << start_z << "m" << std::endl;
  std::cout << "  End position: x=" << end_x << "m, z=" << end_z << "m" << std::endl;
  std::cout << "  Forward distance: " << forward_distance << "m" << std::endl;
  std::cout << "  Stability: " << (fall_detected ? "UNSTABLE" : "STABLE") << std::endl;
  if (fall_detected)
  {
    std::cout << "  Fall detected at step: " << fall_step << std::endl;
  }
  
  // Test passes if robot maintains stability (doesn't fall)
  EXPECT_FALSE(fall_detected) 
    << "Robot should maintain stability with default actuator positions "
    << "(fell at step " << fall_step << ", height: " << end_z << "m)";
  
  // Verify final height is reasonable
  EXPECT_GT(end_z, fall_threshold) 
    << "Robot should remain above fall threshold (final height: " << end_z << "m, threshold: " << fall_threshold << "m)";
}

// Test: Model structure validation
// Verifies the MuJoCo model has expected structure for Open Duck Mini
TEST_F(MotionControllerMuJoCoTest, ModelStructure)
{
  if (!model_ || !data_)
  {
    GTEST_SKIP() << "MuJoCo model not loaded";
  }
  
  // Verify expected number of actuators (14 joints: 10 leg + 4 head)
  EXPECT_EQ(num_joints_, 14) << "Expected 14 actuators (10 leg + 4 head joints)";
  
  // Verify model has bodies
  EXPECT_GT(model_->nbody, 0) << "Model should have at least one body";
  
  // Verify model has sensors (IMU, etc.)
  EXPECT_GT(model_->nsensor, 0) << "Model should have sensors";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

