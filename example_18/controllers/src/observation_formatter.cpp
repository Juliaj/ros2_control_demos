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

#include "rclcpp/clock.hpp"
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
  imu_upside_down_(false),
  left_foot_contact_sensor_name_("left_foot_contact"),
  right_foot_contact_sensor_name_("right_foot_contact"),
  contact_force_threshold_(5.0),  // 5N threshold for contact detection from FTS
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
  const control_msgs::msg::Float64Values & interface_data,
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
  // If imu_upside_down is true, invert z-axis (MuJoCo reports negative z when standing)
  if (accelero.size() != 3)
  {
    throw std::runtime_error(
      "Accelerometer size mismatch: expected 3, got " + std::to_string(accelero.size()));
  }
  
  // Warn if accelerometer data appears to be uninitialized (all zeros except x-bias)
  // This happens when interface data is incomplete at startup (count < 10)
  static bool warned_about_zero_accel = false;
  if (!warned_about_zero_accel && accelero[0] == 0.0 && accelero[1] == 0.0 && accelero[2] == 0.0)
  {
    auto logger = rclcpp::get_logger("observation_formatter");
    RCLCPP_WARN(
      logger,
      "IMU accelerometer data is zero (interface data incomplete). "
      "This is normal at startup but should resolve once state_interfaces_broadcaster publishes complete data. "
      "Observation will show [1.3, 0.0, 0.0] until IMU data is available.");
    warned_about_zero_accel = true;
  }
  
  observation.push_back(static_cast<float>(accelero[0] + 1.3));  // Apply x-axis bias
  observation.push_back(static_cast<float>(accelero[1]));
  // Invert z-acceleration if IMU is upside down (MuJoCo reports -9.8 when standing, should be +9.8)
  float accel_z_processed = static_cast<float>(imu_upside_down_ ? -accelero[2] : accelero[2]);
  
  // Debug logging for z-acceleration inversion (throttled to avoid spam)
  static rclcpp::Clock clock;
  static bool logged_inversion_status = false;
  if (!logged_inversion_status || (imu_upside_down_ && accelero[2] < -5.0))  // Log when z is very negative
  {
    auto logger = rclcpp::get_logger("observation_formatter");
    RCLCPP_INFO_THROTTLE(
      logger, clock, 2000,  // Log at most once every 2 seconds
      "IMU z-acceleration: raw=%.4f, imu_upside_down=%s, processed=%.4f",
      accelero[2], imu_upside_down_ ? "true" : "false", accel_z_processed);
    if (imu_upside_down_ && accelero[2] < -5.0)
    {
      logged_inversion_status = true;  // Only log once after we see real data
    }
  }
  
  observation.push_back(accel_z_processed);

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
  double left_contact = left_foot_contact_;
  double right_contact = right_foot_contact_;
  
  observation.push_back(static_cast<float>(left_contact));
  observation.push_back(static_cast<float>(right_contact));

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
  const control_msgs::msg::Float64Values & interface_data)
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
  const control_msgs::msg::Float64Values & interface_data)
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
  const control_msgs::msg::Float64Values & msg, std::vector<double> & gyro,
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
  else
  {
    // Interface data incomplete - accelerometer will remain at initialized zeros [0, 0, 0]
    // This causes observation to show [1.3, 0.0, 0.0] (x-bias applied to zero)
    // This is normal at startup until state_interfaces_broadcaster publishes complete data
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

  // Debug: Log IMU data extraction status (throttled)
  static rclcpp::Clock clock;
  RCLCPP_DEBUG_THROTTLE(
    logger, clock, 5000,  // Log at most once every 5 seconds
    "IMU data extraction: gyro_found=[%s, %s, %s], accel_found=[%s, %s, %s], "
    "gyro=[%.4f, %.4f, %.4f], accel=[%.4f, %.4f, %.4f]",
    imu_gyro_found[0] ? "yes" : "no", imu_gyro_found[1] ? "yes" : "no",
    imu_gyro_found[2] ? "yes" : "no", imu_accel_found[0] ? "yes" : "no",
    imu_accel_found[1] ? "yes" : "no", imu_accel_found[2] ? "yes" : "no", gyro[0], gyro[1],
    gyro[2], accelero[0], accelero[1], accelero[2]);

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
        "Looking for '%s/linear_acceleration.{x,y,z}' at indices 7-9. "
        "Total interfaces available: %zu (expected at least 10). "
        "Accelerometer values will be zero [0, 0, 0], resulting in observation [1.3, 0.0, 0.0] "
        "until state_interfaces_broadcaster publishes complete data.",
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

bool ObservationFormatter::extract_feet_contacts(
  const control_msgs::msg::Float64Values & interface_data, double & left_contact,
  double & right_contact)
{
  left_contact = 0.0;
  right_contact = 0.0;

  if (interface_names_.empty() || interface_data.values.empty())
  {
    return false;
  }

  auto logger = rclcpp::get_logger("observation_formatter");
  const size_t count = std::min(interface_names_.size(), interface_data.values.size());

  // Try to find contact sensors first (binary contact interfaces)
  // Pattern: "{sensor_name}/contact" or "{sensor_name}/value"
  bool left_contact_found = false;
  bool right_contact_found = false;

  for (size_t i = 0; i < count; ++i)
  {
    const std::string & interface_name = interface_names_[i];
    const double value = interface_data.values[i];

    // Check for left foot contact sensor
    if (!left_contact_found &&
        (interface_name.find(left_foot_contact_sensor_name_) != std::string::npos ||
         interface_name.find("left_foot") != std::string::npos) &&
        (interface_name.find("/contact") != std::string::npos ||
         interface_name.find("/value") != std::string::npos))
    {
      left_contact = (value > 0.5) ? 1.0 : 0.0;  // Binary threshold
      left_contact_found = true;
    }

    // Check for right foot contact sensor
    if (!right_contact_found &&
        (interface_name.find(right_foot_contact_sensor_name_) != std::string::npos ||
         interface_name.find("right_foot") != std::string::npos) &&
        (interface_name.find("/contact") != std::string::npos ||
         interface_name.find("/value") != std::string::npos))
    {
      right_contact = (value > 0.5) ? 1.0 : 0.0;  // Binary threshold
      right_contact_found = true;
    }
  }

  // If both contact sensors found, return success
  if (left_contact_found && right_contact_found)
  {
    static size_t info_counter = 0;
    if (++info_counter % 250 == 0)  // Log every 250 calls (~10 seconds at 25Hz)
    {
      RCLCPP_INFO(
        logger, "Using contact sensors: left=%.0f, right=%.0f", left_contact, right_contact);
    }
    return true;
  }

  // Try to find force/torque sensors (FTS) as fallback
  // Pattern: "{sensor_name}/force.x", "{sensor_name}/force.y", "{sensor_name}/force.z"
  // Compute force magnitude: sqrt(fx^2 + fy^2 + fz^2)
  bool left_force_x_found = false, left_force_y_found = false, left_force_z_found = false;
  bool right_force_x_found = false, right_force_y_found = false, right_force_z_found = false;
  double left_force_x = 0.0, left_force_y = 0.0, left_force_z = 0.0;
  double right_force_x = 0.0, right_force_y = 0.0, right_force_z = 0.0;

  for (size_t i = 0; i < count; ++i)
  {
    const std::string & interface_name = interface_names_[i];
    const double value = interface_data.values[i];

    // Check for left foot FTS
    if (interface_name.find("left_foot") != std::string::npos ||
        interface_name.find("left_fts") != std::string::npos)
    {
      if (interface_name.find("/force.x") != std::string::npos)
      {
        left_force_x = value;
        left_force_x_found = true;
      }
      else if (interface_name.find("/force.y") != std::string::npos)
      {
        left_force_y = value;
        left_force_y_found = true;
      }
      else if (interface_name.find("/force.z") != std::string::npos)
      {
        left_force_z = value;
        left_force_z_found = true;
      }
    }

    // Check for right foot FTS
    if (interface_name.find("right_foot") != std::string::npos ||
        interface_name.find("right_fts") != std::string::npos)
    {
      if (interface_name.find("/force.x") != std::string::npos)
      {
        right_force_x = value;
        right_force_x_found = true;
      }
      else if (interface_name.find("/force.y") != std::string::npos)
      {
        right_force_y = value;
        right_force_y_found = true;
      }
      else if (interface_name.find("/force.z") != std::string::npos)
      {
        right_force_z = value;
        right_force_z_found = true;
      }
    }
  }

  // Compute contact from force magnitude if all FTS components found
  // Use z-component only if x/y not available (vertical force is most important for contact)
  if (left_force_z_found)
  {
    double left_force_magnitude;
    if (left_force_x_found && left_force_y_found)
    {
      // Full 3D force vector
      left_force_magnitude =
        std::sqrt(left_force_x * left_force_x + left_force_y * left_force_y +
                  left_force_z * left_force_z);
    }
    else
    {
      // Use z-component only (vertical force)
      left_force_magnitude = std::abs(left_force_z);
    }
    left_contact = (left_force_magnitude > contact_force_threshold_) ? 1.0 : 0.0;
    left_contact_found = true;
  }

  if (right_force_z_found)
  {
    double right_force_magnitude;
    if (right_force_x_found && right_force_y_found)
    {
      // Full 3D force vector
      right_force_magnitude =
        std::sqrt(right_force_x * right_force_x + right_force_y * right_force_y +
                  right_force_z * right_force_z);
    }
    else
    {
      // Use z-component only (vertical force)
      right_force_magnitude = std::abs(right_force_z);
    }
    right_contact = (right_force_magnitude > contact_force_threshold_) ? 1.0 : 0.0;
    right_contact_found = true;
  }

  // Return true if at least one sensor type was found
  if (left_contact_found && right_contact_found)
  {
    static size_t info_counter = 0;
    if (++info_counter % 250 == 0)  // Log every 250 calls (~10 seconds at 25Hz)
    {
      RCLCPP_INFO(
        logger, "Using FTS sensors for contact: left=%.0f, right=%.0f", left_contact,
        right_contact);
    }
    return true;
  }

  // No contact sensors found
  return false;
}

}  // namespace motion_controller
