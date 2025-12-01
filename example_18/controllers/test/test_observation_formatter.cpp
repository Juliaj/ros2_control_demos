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

#include "control_msgs/msg/interfaces_values.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "locomotion_controller/observation_formatter.hpp"

class TestObservationFormatter : public ::testing::Test
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
    formatter_ = std::make_unique<locomotion_controller::ObservationFormatter>(joint_names_);

    // Build interface names list following broadcaster configuration (position/velocity per joint)
    interface_names_.clear();
    for (const auto & joint_name : joint_names_)
    {
      interface_names_.push_back(joint_name + "/position");
      interface_names_.push_back(joint_name + "/velocity");
    }
    formatter_->set_interface_names(interface_names_);
  }

  std::vector<std::string> joint_names_;
  size_t num_joints_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<locomotion_controller::ObservationFormatter> formatter_;
};

TEST_F(TestObservationFormatter, ObservationDimension)
{
  // Expected: 10 + 3*N where N = 12 joints
  // 4 (velocity_commands) + 3 (base_ang_vel) + 3 (projected_gravity) + 12 (joint_pos) + 12
  // (joint_vel) + 12 (actions)
  size_t expected_dim = 10 + 3 * num_joints_;
  EXPECT_EQ(formatter_->get_observation_dim(), expected_dim);
}

TEST_F(TestObservationFormatter, ObservationVectorFormat)
{
  // Create test interface data
  control_msgs::msg::InterfacesValues interface_data;
  geometry_msgs::msg::Twist velocity_cmd;
  velocity_cmd.linear.x = 0.5;
  velocity_cmd.linear.y = 0.0;
  velocity_cmd.angular.z = 0.0;

  std::vector<double> previous_action(num_joints_, 0.1);

  // Set default joint positions
  std::vector<double> default_positions(num_joints_, 0.5);
  formatter_->set_default_joint_positions(default_positions);

  // Format observation
  std::vector<float> observation =
    formatter_->format(interface_data, velocity_cmd, previous_action);

  // Verify dimension
  EXPECT_EQ(observation.size(), formatter_->get_observation_dim());

  // Verify order: velocity_commands (4D)
  EXPECT_FLOAT_EQ(observation[0], 0.5f);  // lin_vel_x
  EXPECT_FLOAT_EQ(observation[1], 0.0f);  // lin_vel_y
  EXPECT_FLOAT_EQ(observation[2], 0.0f);  // ang_vel_z
  EXPECT_FLOAT_EQ(observation[3], 0.0f);  // heading

  // Verify order: base_angular_velocity (3D) - should be zeros from empty interface_data
  EXPECT_FLOAT_EQ(observation[4], 0.0f);
  EXPECT_FLOAT_EQ(observation[5], 0.0f);
  EXPECT_FLOAT_EQ(observation[6], 0.0f);

  // Verify order: projected_gravity (3D) - should have fallback values
  // When IMU orientation not found, defaults to [0, 0, -1]
  EXPECT_FLOAT_EQ(observation[7], 0.0f);
  EXPECT_FLOAT_EQ(observation[8], 0.0f);
  EXPECT_FLOAT_EQ(observation[9], -1.0f);

  // Verify order: joint_positions (N joints, relative to default)
  // Since interface_data is empty, positions are 0.0, relative to default 0.5 = -0.5
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_FLOAT_EQ(observation[10 + i], -0.5f);
  }

  // Verify order: joint_velocities (N joints)
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_FLOAT_EQ(observation[10 + num_joints_ + i], 0.0f);
  }

  // Verify order: previous_action (N joints)
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_FLOAT_EQ(observation[10 + 2 * num_joints_ + i], 0.1f);
  }
}

TEST_F(TestObservationFormatter, RelativeJointPositions)
{
  // Create interface data with joint positions
  control_msgs::msg::InterfacesValues interface_data;
  geometry_msgs::msg::Twist velocity_cmd;

  // Interface data format: 10 IMU values + 2*num_joints_ (position + velocity per joint)
  // Indices: 0-3 (IMU orientation), 4-6 (IMU ang vel), 7-9 (IMU lin acc - unused),
  //          10-10+num_joints_-1 (joint positions), 10+num_joints_-10+2*num_joints_-1 (joint
  //          velocities)
  const size_t expected_size = 10 + 2 * num_joints_;
  interface_data.values.resize(expected_size, 0.0);

  // Set IMU orientation (required for extraction to work properly)
  interface_data.values[0] = 0.0;  // orientation x
  interface_data.values[1] = 0.0;  // orientation y
  interface_data.values[2] = 0.0;  // orientation z
  interface_data.values[3] = 1.0;  // orientation w

  // Set joint positions at correct indices (starting at 10)
  for (size_t i = 0; i < num_joints_; ++i)
  {
    interface_data.values[10 + i] = 1.0;  // Absolute position
  }

  // Set default positions to 0.5
  std::vector<double> default_positions(num_joints_, 0.5);
  formatter_->set_default_joint_positions(default_positions);

  std::vector<double> previous_action(num_joints_, 0.0);

  // Format observation
  std::vector<float> observation =
    formatter_->format(interface_data, velocity_cmd, previous_action);

  // Verify joint positions are relative (1.0 - 0.5 = 0.5)
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_FLOAT_EQ(observation[10 + i], 0.5f);
  }
}

TEST_F(TestObservationFormatter, VelocityCommandFormat)
{
  control_msgs::msg::InterfacesValues interface_data;
  geometry_msgs::msg::Twist velocity_cmd;
  velocity_cmd.linear.x = 0.3;
  velocity_cmd.linear.y = 0.2;
  velocity_cmd.angular.z = 0.1;

  std::vector<double> previous_action(num_joints_, 0.0);
  std::vector<float> observation =
    formatter_->format(interface_data, velocity_cmd, previous_action);

  // Verify velocity commands are first 4 elements
  EXPECT_FLOAT_EQ(observation[0], 0.3f);  // lin_vel_x
  EXPECT_FLOAT_EQ(observation[1], 0.2f);  // lin_vel_y
  EXPECT_FLOAT_EQ(observation[2], 0.1f);  // ang_vel_z
  EXPECT_FLOAT_EQ(observation[3], 0.0f);  // heading (not in Twist)
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
