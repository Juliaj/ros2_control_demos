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

#include "locomotion_controller/observation_formatter.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace locomotion_controller
{

ObservationFormatter::ObservationFormatter(const std::vector<std::string> & joint_names)
: joint_names_(joint_names),
  num_joints_(joint_names.size()),
  default_joint_positions_(num_joints_, 0.0),
  default_joint_positions_set_(false),
  imu_sensor_name_("imu_1")
{
  // Observation dimension: 4 (velocity_commands) + 3 (base_ang_vel) + 3 (projected_gravity)
  // + N (joint_pos) + N (joint_vel) + N (actions) = 10 + 3*N
  observation_dim_ = 10 + 3 * num_joints_;
}

std::vector<float> ObservationFormatter::format(
  const control_msgs::msg::InterfacesValues & sensor_data,
  const geometry_msgs::msg::Twist & velocity_cmd, const std::vector<double> & previous_action)
{
  // Extract sensor data from message
  std::vector<double> base_angular_velocity;
  std::vector<double> projected_gravity;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  extract_sensor_data(
    sensor_data, base_angular_velocity, projected_gravity, joint_positions, joint_velocities);

  // Build observation vector in the correct order
  std::vector<float> observation;
  observation.reserve(observation_dim_);

  // 1. Velocity commands (4D: lin_vel_x, lin_vel_y, ang_vel_z, heading)
  std::vector<float> velocity_commands = format_velocity_commands(velocity_cmd);
  observation.insert(observation.end(), velocity_commands.begin(), velocity_commands.end());

  // 2. Base angular velocity (3D vector)
  for (const auto & val : base_angular_velocity)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 3. Projected gravity vector (3D vector)
  for (const auto & val : projected_gravity)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 4. Joint positions (N joints, relative to default positions)
  if (default_joint_positions_set_ && joint_positions.size() == default_joint_positions_.size())
  {
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
      observation.push_back(static_cast<float>(joint_positions[i] - default_joint_positions_[i]));
    }
  }
  else
  {
    // Fallback to absolute if defaults not set
    for (const auto & val : joint_positions)
    {
      observation.push_back(static_cast<float>(val));
    }
  }

  // 5. Joint velocities (N joints, relative - currently using absolute)
  // Note: Relative velocities would require tracking previous velocities
  for (const auto & val : joint_velocities)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 6. Previous action (N joints)
  for (const auto & val : previous_action)
  {
    observation.push_back(static_cast<float>(val));
  }

  return observation;
}

std::vector<double> ObservationFormatter::extract_joint_positions(
  const control_msgs::msg::InterfacesValues & sensor_data)
{
  std::vector<double> base_angular_velocity;
  std::vector<double> projected_gravity;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  extract_sensor_data(
    sensor_data, base_angular_velocity, projected_gravity, joint_positions, joint_velocities);

  return joint_positions;
}

void ObservationFormatter::set_default_joint_positions(
  const std::vector<double> & default_positions)
{
  if (default_positions.size() == num_joints_)
  {
    default_joint_positions_ = default_positions;
    default_joint_positions_set_ = true;
  }
}

void ObservationFormatter::set_interface_names(const std::vector<std::string> & interface_names)
{
  interface_names_ = interface_names;
}

std::vector<float> ObservationFormatter::format_velocity_commands(
  const geometry_msgs::msg::Twist & velocity_cmd)
{
  // Format: [lin_vel_x, lin_vel_y, ang_vel_z, heading]
  // Note: heading is not available in Twist, set to 0.0
  // This matches the observation format from env_cfg.py
  return {
    static_cast<float>(velocity_cmd.linear.x), static_cast<float>(velocity_cmd.linear.y),
    static_cast<float>(velocity_cmd.angular.z),
    0.0f  // heading (not available in Twist message)
  };
}

void ObservationFormatter::extract_sensor_data(
  const control_msgs::msg::InterfacesValues & msg, std::vector<double> & base_angular_velocity,
  std::vector<double> & projected_gravity, std::vector<double> & joint_positions,
  std::vector<double> & joint_velocities)
{
  base_angular_velocity.clear();
  projected_gravity.clear();
  joint_positions.clear();
  joint_velocities.clear();

  // Initialize with zeros
  base_angular_velocity.resize(3, 0.0);
  projected_gravity.resize(3, 0.0);
  joint_positions.resize(num_joints_, 0.0);
  joint_velocities.resize(num_joints_, 0.0);

  // Extract IMU orientation quaternion components
  double imu_orientation_x = 0.0, imu_orientation_y = 0.0, imu_orientation_z = 0.0,
         imu_orientation_w = 1.0;
  bool imu_orientation_found = false;

  // Iterate through interface values to extract data, matching names published by broadcaster
  const size_t count = std::min(interface_names_.size(), msg.values.size());
  for (size_t idx = 0; idx < count; ++idx)
  {
    const std::string & name = interface_names_[idx];
    const double value = msg.values[idx];

    // Extract base angular velocity from IMU (try multiple naming conventions)
    std::string imu_ang_vel_x = imu_sensor_name_ + "/angular_velocity/x";
    std::string imu_ang_vel_y = imu_sensor_name_ + "/angular_velocity/y";
    std::string imu_ang_vel_z = imu_sensor_name_ + "/angular_velocity/z";

    if (
      name == imu_ang_vel_x || name == "imu/angular_velocity/x" ||
      name == "imu_1/angular_velocity/x")
    {
      base_angular_velocity[0] = value;
    }
    else if (
      name == imu_ang_vel_y || name == "imu/angular_velocity/y" ||
      name == "imu_1/angular_velocity/y")
    {
      base_angular_velocity[1] = value;
    }
    else if (
      name == imu_ang_vel_z || name == "imu/angular_velocity/z" ||
      name == "imu_1/angular_velocity/z")
    {
      base_angular_velocity[2] = value;
    }
    // Extract IMU orientation quaternion
    else if (name == imu_sensor_name_ + "/orientation/x" || name == "imu_1/orientation/x")
    {
      imu_orientation_x = value;
      imu_orientation_found = true;
    }
    else if (name == imu_sensor_name_ + "/orientation/y" || name == "imu_1/orientation/y")
    {
      imu_orientation_y = value;
      imu_orientation_found = true;
    }
    else if (name == imu_sensor_name_ + "/orientation/z" || name == "imu_1/orientation/z")
    {
      imu_orientation_z = value;
      imu_orientation_found = true;
    }
    else if (name == imu_sensor_name_ + "/orientation/w" || name == "imu_1/orientation/w")
    {
      imu_orientation_w = value;
      imu_orientation_found = true;
    }
    // Extract joint positions and velocities
    else
    {
      for (size_t i = 0; i < joint_names_.size(); ++i)
      {
        if (name == joint_names_[i] + "/position")
        {
          joint_positions[i] = value;
        }
        else if (name == joint_names_[i] + "/velocity")
        {
          joint_velocities[i] = value;
        }
      }
    }
  }

  // Compute projected gravity vector from IMU orientation
  if (imu_orientation_found)
  {
    compute_projected_gravity(
      imu_orientation_x, imu_orientation_y, imu_orientation_z, imu_orientation_w,
      projected_gravity);
  }
  else
  {
    // Fallback: assume upright orientation (gravity = [0, 0, -9.81] in world frame)
    // Projected gravity in robot frame when upright: [0, 0, -1] (normalized)
    projected_gravity[0] = 0.0;
    projected_gravity[1] = 0.0;
    projected_gravity[2] = -1.0;
  }
}

void ObservationFormatter::compute_projected_gravity(
  double qx, double qy, double qz, double qw, std::vector<double> & projected_gravity)
{
  // Gravity vector in world frame (assuming z-up): [0, 0, -9.81]
  // Projected gravity = R^T * [0, 0, -1] where R is rotation matrix from quaternion
  // This gives gravity direction in robot body frame

  // Normalize quaternion
  double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm > 1e-6)
  {
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
  }

  // Rotate gravity vector [0, 0, -1] by inverse quaternion (conjugate)
  // Inverse quaternion: [qx, qy, qz, qw] -> [-qx, -qy, -qz, qw]
  // For a vector v, rotated vector = q * v * q^-1
  // Simplified: R^T * [0, 0, -1] where R is rotation matrix

  // Compute rotation matrix elements (row-major)
  double r02 = 2 * (qx * qz + qy * qw);
  double r12 = 2 * (qy * qz - qx * qw);
  double r22 = 1 - 2 * (qx * qx + qy * qy);

  // Project gravity [0, 0, -1] using transpose (inverse for rotation matrix)
  projected_gravity[0] = r02;  // R^T[0,2] = R[2,0]
  projected_gravity[1] = r12;  // R^T[1,2] = R[2,1]
  projected_gravity[2] = r22;  // R^T[2,2] = R[2,2]
}

}  // namespace locomotion_controller
