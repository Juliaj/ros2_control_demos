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

#include "motion_controller/observation_formatter.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "rclcpp/logging.hpp"

namespace motion_controller
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
  previous_states_initialized_(false),
  last_action_(num_joints_, 0.0),
  last_last_action_(num_joints_, 0.0),
  last_last_last_action_(num_joints_, 0.0),
  motor_targets_(num_joints_, 0.0),
  left_foot_contact_(0.0),
  right_foot_contact_(0.0),
  imitation_i_(0.0),
  phase_period_(100.0),
  imitation_phase_(2, 0.0),
  velocity_commands_(7, 0.0)
{
  // Observation dimension: 3 (gyro) + 3 (accelero) + 7 (commands)
  // + N (joint_pos) + N (joint_vel) + N (last_action) + N (last_last_action)
  // + N (last_last_last_action) + N (motor_targets) + 2 (feet_contacts) + 2 (phase)
  // = 17 + 6*N
  observation_dim_ = 17 + 6 * num_joints_;
  imitation_phase_[0] = 1.0;  // cos(0) = 1
  imitation_phase_[1] = 0.0;  // sin(0) = 0
}

std::vector<float> ObservationFormatter::format(
  const control_msgs::msg::InterfacesValues & interface_data,
  const geometry_msgs::msg::Twist & velocity_cmd, const std::vector<double> & previous_action)
{
  // Note: velocity_cmd parameter kept for API compatibility but not used directly
  // Velocity commands should be set via set_velocity_commands() before calling format()
  (void)velocity_cmd;  // Suppress unused parameter warning

  // Update action history
  update_action_history(previous_action);

  // Extract interface data from message
  std::vector<double> gyro;
  std::vector<double> accelero;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  extract_interface_data(interface_data, gyro, accelero, joint_positions, joint_velocities);

  // Build observation vector in the correct order
  std::vector<float> observation;
  observation.reserve(observation_dim_);

  // 1. IMU gyroscope (3D)
  if (gyro.size() != 3)
  {
    throw std::runtime_error("Gyro size mismatch: expected 3, got " + std::to_string(gyro.size()));
  }
  for (const auto & val : gyro)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 2. IMU accelerometer (3D)
  // Apply bias to match MuJoCo training data (Playground, mujoco_infer.py line 74: accelerometer[0] += 1.3)
  if (accelero.size() != 3)
  {
    throw std::runtime_error(
      "Accelerometer size mismatch: expected 3, got " + std::to_string(accelero.size()));
  }
  observation.push_back(static_cast<float>(accelero[0] + 1.3));  // Apply x-axis bias
  observation.push_back(static_cast<float>(accelero[1]));
  observation.push_back(static_cast<float>(accelero[2]));

  // 3. Commands (7D: 3 base + 4 head)
  if (velocity_commands_.size() != 7)
  {
    throw std::runtime_error(
      "Velocity commands size mismatch: expected 7, got " +
      std::to_string(velocity_commands_.size()));
  }
  for (const auto & val : velocity_commands_)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 4. Joint positions (relative to default positions)
  format_joint_positions_relative(joint_positions, observation);

  // 5. Joint velocities (scaled by 0.05)
  format_joint_velocities_scaled(joint_velocities, observation);

  // 6. Last action (N joints)
  for (const auto & val : last_action_)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 7. Last-last action (N joints)
  for (const auto & val : last_last_action_)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 8. Last-last-last action (N joints)
  for (const auto & val : last_last_last_action_)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 9. Motor targets (N joints)
  for (const auto & val : motor_targets_)
  {
    observation.push_back(static_cast<float>(val));
  }

  // 10. Feet contacts (2D)
  observation.push_back(static_cast<float>(left_foot_contact_));
  observation.push_back(static_cast<float>(right_foot_contact_));

  // 11. Imitation phase (2D: cos, sin)
  observation.push_back(static_cast<float>(imitation_phase_[0]));
  observation.push_back(static_cast<float>(imitation_phase_[1]));

  // Validate final observation size
  if (observation.size() != observation_dim_)
  {
    throw std::runtime_error(
      "Observation size mismatch: expected " + std::to_string(observation_dim_) + ", got " +
      std::to_string(observation.size()));
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

void ObservationFormatter::extract_interface_data(
  const control_msgs::msg::InterfacesValues & msg, std::vector<double> & gyro,
  std::vector<double> & accelero, std::vector<double> & joint_positions,
  std::vector<double> & joint_velocities)
{
  gyro.clear();
  accelero.clear();
  joint_positions.clear();
  joint_velocities.clear();

  // Initialize with zeros
  gyro.resize(3, 0.0);
  accelero.resize(3, 0.0);
  joint_positions.resize(num_joints_, 0.0);
  joint_velocities.resize(num_joints_, 0.0);

  // Get logger (respects RCUTILS_LOGGING_SEVERITY environment variable)
  auto logger = rclcpp::get_logger("observation_formatter");

  // Expected interface order (aligned with Open Duck Mini observation):
  // 0-3: IMU orientation (x, y, z, w) - not used in observation
  // 4-6: IMU angular velocity (x, y, z) - gyro
  // 7-9: IMU linear acceleration (x, y, z) - accelero
  // 10 to 10+num_joints_-1: Joint positions
  // 10+num_joints_ to 10+2*num_joints_-1: Joint velocities
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

  // Extract IMU gyroscope (angular velocity) from indices 4-6
  bool imu_gyro_found[3] = {false, false, false};
  if (count >= 7)
  {
    gyro[0] = msg.values[4];
    gyro[1] = msg.values[5];
    gyro[2] = msg.values[6];
    imu_gyro_found[0] = true;
    imu_gyro_found[1] = true;
    imu_gyro_found[2] = true;
  }

  // Extract IMU accelerometer (linear acceleration) from indices 7-9
  bool imu_accel_found[3] = {false, false, false};
  if (count >= 10)
  {
    accelero[0] = msg.values[7];
    accelero[1] = msg.values[8];
    accelero[2] = msg.values[9];
    imu_accel_found[0] = true;
    imu_accel_found[1] = true;
    imu_accel_found[2] = true;
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
      "IMU data extraction: gyro_found=[%s, %s, %s], accel_found=[%s, %s, %s], "
      "gyro=[%.4f, %.4f, %.4f], accel=[%.4f, %.4f, %.4f]",
      imu_gyro_found[0] ? "yes" : "no", imu_gyro_found[1] ? "yes" : "no",
      imu_gyro_found[2] ? "yes" : "no", imu_accel_found[0] ? "yes" : "no",
      imu_accel_found[1] ? "yes" : "no", imu_accel_found[2] ? "yes" : "no", gyro[0], gyro[1],
      gyro[2], accelero[0], accelero[1], accelero[2]);
  }

  if (!imu_gyro_found[0] || !imu_gyro_found[1] || !imu_gyro_found[2])
  {
    static size_t warn_gyro_counter = 0;
    if (++warn_gyro_counter % 125 == 0)  // Warn every 125 calls (~5 seconds at 25Hz)
    {
      RCLCPP_WARN(
        logger,
        "IMU gyroscope incomplete! Found: [%s, %s, %s]. "
        "Looking for '%s/angular_velocity.{x,y,z}'. Total interfaces checked: %zu",
        imu_gyro_found[0] ? "yes" : "no", imu_gyro_found[1] ? "yes" : "no",
        imu_gyro_found[2] ? "yes" : "no", imu_sensor_name_.c_str(), count);
    }
  }

  if (!imu_accel_found[0] || !imu_accel_found[1] || !imu_accel_found[2])
  {
    static size_t warn_accel_counter = 0;
    if (++warn_accel_counter % 125 == 0)  // Warn every 125 calls (~5 seconds at 25Hz)
    {
      RCLCPP_WARN(
        logger,
        "IMU accelerometer incomplete! Found: [%s, %s, %s]. "
        "Looking for '%s/linear_acceleration.{x,y,z}'. Total interfaces checked: %zu",
        imu_accel_found[0] ? "yes" : "no", imu_accel_found[1] ? "yes" : "no",
        imu_accel_found[2] ? "yes" : "no", imu_sensor_name_.c_str(), count);
    }
  }
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

void ObservationFormatter::format_joint_velocities_scaled(
  const std::vector<double> & joint_velocities, std::vector<float> & observation)
{
  // Validate input size
  if (joint_velocities.size() != num_joints_)
  {
    throw std::runtime_error(
      "Joint velocities size mismatch in format_joint_velocities_scaled: velocities=" +
      std::to_string(joint_velocities.size()) + ", expected=" + std::to_string(num_joints_));
  }

  // Joint velocities (N joints, scaled by 0.05)
  // Reference: Open Duck Mini v2_rl_walk_mujoco.py line 162: dof_vel * 0.05
  const double velocity_scale = 0.05;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    observation.push_back(static_cast<float>(joint_velocities[i] * velocity_scale));
  }
}

void ObservationFormatter::update_action_history(const std::vector<double> & action)
{
  // Shift action history: last_last_last <- last_last <- last <- action
  last_last_last_action_ = last_last_action_;
  last_last_action_ = last_action_;
  if (action.size() == num_joints_)
  {
    last_action_ = action;
  }
  else if (!action.empty())
  {
    auto logger = rclcpp::get_logger("observation_formatter");
    RCLCPP_WARN(
      logger, "Action size mismatch: expected %zu, got %zu. Keeping previous action.", num_joints_,
      action.size());
  }
}

void ObservationFormatter::set_motor_targets(const std::vector<double> & motor_targets)
{
  if (motor_targets.size() == num_joints_)
  {
    motor_targets_ = motor_targets;
  }
}

void ObservationFormatter::set_feet_contacts(double left_contact, double right_contact)
{
  left_foot_contact_ = left_contact;
  right_foot_contact_ = right_contact;
}

void ObservationFormatter::update_imitation_phase(double phase_frequency_factor)
{
  // Increment phase counter (line 257-260 in v2_rl_walk_mujoco.py)
  imitation_i_ += 1.0 * phase_frequency_factor;
  imitation_i_ = std::fmod(imitation_i_, phase_period_);

  // Compute cos/sin encoding (line 261-270 in v2_rl_walk_mujoco.py)
  const double PI = 3.14159265358979323846;
  double theta = (imitation_i_ / phase_period_) * 2.0 * PI;
  imitation_phase_[0] = std::cos(theta);
  imitation_phase_[1] = std::sin(theta);
}

void ObservationFormatter::set_phase_period(double nb_steps_in_period)
{
  phase_period_ = nb_steps_in_period;
}

void ObservationFormatter::set_velocity_commands(const std::vector<double> & commands)
{
  if (commands.size() == 7)
  {
    velocity_commands_ = commands;
  }
  else if (!commands.empty())
  {
    auto logger = rclcpp::get_logger("observation_formatter");
    RCLCPP_WARN(
      logger, "Velocity commands size mismatch: expected 7, got %zu. Keeping previous commands.",
      commands.size());
  }
}

}  // namespace motion_controller
