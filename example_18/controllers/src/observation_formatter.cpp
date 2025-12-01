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

#include "rclcpp/logging.hpp"

namespace locomotion_controller
{

ObservationFormatter::ObservationFormatter(
  const std::vector<std::string> & joint_names, const std::string & imu_sensor_name)
: joint_names_(joint_names),
  num_joints_(joint_names.size()),
  default_joint_positions_(num_joints_, 0.0),
  default_joint_positions_set_(false),
  default_joint_velocities_(num_joints_, 0.0),
  default_joint_velocities_set_(false),
  imu_sensor_name_(imu_sensor_name),
  previous_joint_positions_(num_joints_, 0.0),
  previous_joint_velocities_(num_joints_, 0.0),
  previous_states_initialized_(false)
{
  // Observation dimension: 4 (velocity_commands) + 3 (base_ang_vel) + 3 (projected_gravity)
  // + N (joint_pos) + N (joint_vel) + N (actions) = 10 + 3*N
  observation_dim_ = 10 + 3 * num_joints_;
}

std::vector<float> ObservationFormatter::format(
  const control_msgs::msg::InterfacesValues & interface_data,
  const geometry_msgs::msg::Twist & velocity_cmd, const std::vector<double> & previous_action)
{
  // Extract interface data from message
  std::vector<double> base_angular_velocity;
  std::vector<double> projected_gravity;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  extract_interface_data(
    interface_data, base_angular_velocity, projected_gravity, joint_positions, joint_velocities);

  // Build observation vector in the correct order
  std::vector<float> observation;
  observation.reserve(observation_dim_);

  // 1. Velocity commands (4D: lin_vel_x, lin_vel_y, ang_vel_z, heading)
  std::vector<float> velocity_commands = format_velocity_commands(velocity_cmd);
  if (velocity_commands.size() != 4)
  {
    throw std::runtime_error(
      "Velocity commands size mismatch: expected 4, got " +
      std::to_string(velocity_commands.size()));
  }
  observation.insert(observation.end(), velocity_commands.begin(), velocity_commands.end());

  // 2. Base angular velocity (3D vector)
  if (base_angular_velocity.size() != 3)
  {
    throw std::runtime_error(
      "Base angular velocity size mismatch: expected 3, got " +
      std::to_string(base_angular_velocity.size()));
  }
  for (const auto & val : base_angular_velocity)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 3. Projected gravity vector (3D vector)
  if (projected_gravity.size() != 3)
  {
    throw std::runtime_error(
      "Projected gravity size mismatch: expected 3, got " +
      std::to_string(projected_gravity.size()));
  }
  for (const auto & val : projected_gravity)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 4. Joint positions (relative to default positions)
  format_joint_positions_relative(joint_positions, observation);

  // 5. Joint velocities (absolute velocities)
  format_joint_velocities_relative(joint_velocities, observation);

  // 6. Previous action (N joints)
  if (previous_action.size() != num_joints_)
  {
    throw std::runtime_error(
      "Previous action size mismatch: expected " + std::to_string(num_joints_) + ", got " +
      std::to_string(previous_action.size()));
  }
  for (const auto & val : previous_action)
  {
    observation.push_back(static_cast<float>(val));
  }

  // Validate final observation size
  if (observation.size() != observation_dim_)
  {
    throw std::runtime_error(
      "Observation size mismatch: expected " + std::to_string(observation_dim_) + ", got " +
      std::to_string(observation.size()) +
      ". Components: velocity_commands=" + std::to_string(velocity_commands.size()) +
      ", base_ang_vel=" + std::to_string(base_angular_velocity.size()) +
      ", projected_gravity=" + std::to_string(projected_gravity.size()) +
      ", joint_pos=" + std::to_string(joint_positions.size()) +
      ", joint_vel=" + std::to_string(joint_velocities.size()) +
      ", previous_action=" + std::to_string(previous_action.size()));
  }

  return observation;
}

std::vector<double> ObservationFormatter::extract_joint_positions(
  const control_msgs::msg::InterfacesValues & interface_data)
{
  std::vector<double> base_angular_velocity;
  std::vector<double> projected_gravity;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  extract_interface_data(
    interface_data, base_angular_velocity, projected_gravity, joint_positions, joint_velocities);

  return joint_positions;
}

std::vector<double> ObservationFormatter::extract_joint_velocities(
  const control_msgs::msg::InterfacesValues & interface_data)
{
  std::vector<double> base_angular_velocity;
  std::vector<double> projected_gravity;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  extract_interface_data(
    interface_data, base_angular_velocity, projected_gravity, joint_positions, joint_velocities);

  return joint_velocities;
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

void ObservationFormatter::set_default_joint_velocities(
  const std::vector<double> & default_velocities)
{
  if (default_velocities.size() == num_joints_)
  {
    default_joint_velocities_ = default_velocities;
    default_joint_velocities_set_ = true;
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
    0.0f  // heading (not available in Twist message): 0.0 = no heading change (maintain current
          // direction)
  };
}

void ObservationFormatter::extract_interface_data(
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

  // Get logger (respects RCUTILS_LOGGING_SEVERITY environment variable)
  auto logger = rclcpp::get_logger("observation_formatter");

  // Expected interface order (aligned with observation vector):
  // 0-3: IMU orientation (x, y, z, w)
  // 4-6: IMU angular velocity (x, y, z)
  // 7-9: IMU linear acceleration (x, y, z) - not used
  // 10-21: Joint positions (num_joints_)
  // 22-33: Joint velocities (num_joints_)
  const size_t expected_size = 10 + 2 * num_joints_;  // 10 IMU + 2*num_joints_ (pos+vel)
  const size_t count = std::min(interface_names_.size(), msg.values.size());

  // Validate interface count matches expected size
  if (count < expected_size)
  {
    static size_t warn_size_counter = 0;
    if (++warn_size_counter % 125 == 0)  // Warn every 125 calls (~5 seconds at 25Hz)
    {
      RCLCPP_WARN(
        logger,
        "Interface count mismatch: expected at least %zu, got %zu. "
        "Some data may be missing.",
        expected_size, count);
    }
  }

  // Extract IMU orientation quaternion components (indices 0-3)
  double imu_orientation_x = 0.0, imu_orientation_y = 0.0, imu_orientation_z = 0.0,
         imu_orientation_w = 1.0;
  bool imu_orientation_found = false;
  if (count >= 4)
  {
    imu_orientation_x = msg.values[0];
    imu_orientation_y = msg.values[1];
    imu_orientation_z = msg.values[2];
    imu_orientation_w = msg.values[3];
    imu_orientation_found = true;
  }

  // Extract base angular velocity from IMU (indices 4-6)
  bool imu_ang_vel_found[3] = {false, false, false};
  if (count >= 7)
  {
    base_angular_velocity[0] = msg.values[4];
    base_angular_velocity[1] = msg.values[5];
    base_angular_velocity[2] = msg.values[6];
    imu_ang_vel_found[0] = true;
    imu_ang_vel_found[1] = true;
    imu_ang_vel_found[2] = true;
  }

  // Extract joint positions (indices 10 to 10+num_joints_-1)
  const size_t joint_pos_start = 10;
  const size_t joint_vel_start = 10 + num_joints_;
  if (count >= joint_vel_start + num_joints_)
  {
    for (size_t i = 0; i < num_joints_; ++i)
    {
      joint_positions[i] = msg.values[joint_pos_start + i];
      joint_velocities[i] = msg.values[joint_vel_start + i];
    }
  }
  else if (count >= joint_pos_start + num_joints_)
  {
    // Only positions available
    for (size_t i = 0; i < num_joints_; ++i)
    {
      joint_positions[i] = msg.values[joint_pos_start + i];
    }
  }

  // Debug: Log IMU data extraction status (throttled using static counter)
  static size_t debug_counter = 0;
  if (++debug_counter % 50 == 0)  // Log every 50 calls (~2 seconds at 25Hz)
  {
    RCLCPP_INFO(
      logger,
      "IMU data extraction: orientation_found=%s, ang_vel_found=[%s, %s, %s], "
      "ang_vel=[%.4f, %.4f, %.4f], orientation=[%.4f, %.4f, %.4f, %.4f]",
      imu_orientation_found ? "yes" : "no", imu_ang_vel_found[0] ? "yes" : "no",
      imu_ang_vel_found[1] ? "yes" : "no", imu_ang_vel_found[2] ? "yes" : "no",
      base_angular_velocity[0], base_angular_velocity[1], base_angular_velocity[2],
      imu_orientation_x, imu_orientation_y, imu_orientation_z, imu_orientation_w);
  }

  if (!imu_orientation_found)
  {
    static size_t warn_counter = 0;
    if (++warn_counter % 125 == 0)  // Warn every 125 calls (~5 seconds at 25Hz)
    {
      RCLCPP_WARN(
        logger,
        "IMU orientation not found! Looking for '%s/orientation.{x,y,z,w}'. "
        "Total interfaces checked: %zu",
        imu_sensor_name_.c_str(), count);
    }
  }
  if (!imu_ang_vel_found[0] || !imu_ang_vel_found[1] || !imu_ang_vel_found[2])
  {
    static size_t warn_vel_counter = 0;
    if (++warn_vel_counter % 125 == 0)  // Warn every 125 calls (~5 seconds at 25Hz)
    {
      RCLCPP_WARN(
        logger,
        "IMU angular velocity incomplete! Found: [%s, %s, %s]. "
        "Looking for '%s/angular_velocity.{x,y,z}'. Total interfaces checked: %zu",
        imu_ang_vel_found[0] ? "yes" : "no", imu_ang_vel_found[1] ? "yes" : "no",
        imu_ang_vel_found[2] ? "yes" : "no", imu_sensor_name_.c_str(), count);
    }
  }

  // Compute projected gravity vector from IMU orientation
  // Basis: Berkeley-Humanoid-Lite uses quat_rotate_inverse(base_quat, gravity_vector)
  // to transform gravity from world frame [0, 0, -1] to robot body frame
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
  // Basis: Equivalent to quat_rotate_inverse(q, [0, 0, -1]) from Berkeley-Humanoid-Lite
  // Rotates gravity vector from world frame to robot body frame using inverse quaternion
  // Implementation uses rotation matrix: R^T * [0, 0, -1] where R is from quaternion

  // Normalize quaternion
  double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm > 1e-6)
  {
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
  }

  // Compute rotation matrix elements (third column = z-axis in body frame)
  double r02 = 2 * (qx * qz + qy * qw);
  double r12 = 2 * (qy * qz - qx * qw);
  double r22 = 1 - 2 * (qx * qx + qy * qy);

  // Project gravity [0, 0, -1] using transpose (inverse for rotation matrix)
  projected_gravity[0] = r02;  // R^T[0,2] = R[2,0]
  projected_gravity[1] = r12;  // R^T[1,2] = R[2,1]
  projected_gravity[2] = r22;  // R^T[2,2] = R[2,2]
}

void ObservationFormatter::format_joint_positions_relative(
  const std::vector<double> & joint_positions, std::vector<float> & observation)
{
  // Validate input sizes
  if (joint_positions.size() != num_joints_)
  {
    throw std::runtime_error(
      "Joint positions size mismatch in format_joint_positions_relative: positions=" +
      std::to_string(joint_positions.size()) + ", expected=" + std::to_string(num_joints_));
  }

  // Joint positions (N joints, relative to default positions)
  // Compute: current_position - default_position
  // Reference:
  // https://github.com/isaac-sim/IsaacLab/blob/18c7c58d7a6758b6119401945b881e21c8ec0392/source/isaaclab/isaaclab/envs/mdp/observations.py#L209
  for (size_t i = 0; i < num_joints_; ++i)
  {
    double relative_position = joint_positions[i] - default_joint_positions_[i];
    observation.push_back(static_cast<float>(relative_position));
  }
}

void ObservationFormatter::format_joint_velocities_relative(
  const std::vector<double> & joint_velocities, std::vector<float> & observation)
{
  // Validate input size
  if (joint_velocities.size() != num_joints_)
  {
    throw std::runtime_error(
      "Joint velocities size mismatch in format_joint_velocities_relative: velocities=" +
      std::to_string(joint_velocities.size()) + ", expected=" + std::to_string(num_joints_));
  }

  // Joint velocities (N joints, absolute velocities)
  // Reference:
  // https://github.com/isaac-sim/IsaacLab/blob/18c7c58d7a6758b6119401945b881e21c8ec0392/source/isaaclab/isaaclab/envs/mdp/observations.py#L254
  for (size_t i = 0; i < num_joints_; ++i)
  {
    observation.push_back(static_cast<float>(joint_velocities[i]));
  }
}

}  // namespace locomotion_controller
