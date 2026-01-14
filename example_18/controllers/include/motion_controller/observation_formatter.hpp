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

#ifndef MOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_
#define MOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_

#include <string>
#include <vector>

#include "control_msgs/msg/float64_values.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace motion_controller
{

class ObservationFormatter
{
public:
  explicit ObservationFormatter(
    const std::vector<std::string> & joint_names, const std::string & imu_sensor_name = "imu_2");

  // Extract sensor data and format into model input vector
  // Observation order (based on Open Duck Mini v2_rl_walk_mujoco.py):
  // 1. IMU gyroscope (3D: angular velocity)
  // 2. IMU accelerometer (3D: linear acceleration)
  // 3. Commands (7D: lin_vel_x, lin_vel_y, ang_vel_z, head_pos_1, head_pos_2, head_pos_3,
  // head_pos_4)
  // 4. Joint positions (N joints, relative to default positions)
  // 5. Joint velocities (N joints, scaled by 0.05)
  // 6. Last action (N joints)
  // 7. Last-last action (N joints)
  // 8. Last-last-last action (N joints)
  // 9. Motor targets (N joints)
  // 10. Feet contacts (2D: left, right)
  // 11. Imitation phase (2D: cos, sin)
  std::vector<float> format(
    const control_msgs::msg::Float64Values & interface_data,
    const geometry_msgs::msg::Twist & velocity_cmd, const std::vector<double> & previous_action);

  size_t get_observation_dim() const { return observation_dim_; }

  // Extract joint positions from interface data (for initializing default positions)
  std::vector<double> extract_joint_positions(
    const control_msgs::msg::Float64Values & interface_data);

  // Extract joint velocities from interface data (for initializing default velocities)
  std::vector<double> extract_joint_velocities(
    const control_msgs::msg::Float64Values & interface_data);

  // Extract feet contact data from interface data (uses real sensors if available)
  // Returns [left_contact, right_contact] as binary values (1.0 = contact, 0.0 = no contact)
  // Checks for:
  //   1. Contact sensors (e.g., "left_foot_contact/contact", "right_foot_contact/contact")
  //   2. Force/torque sensors (FTS) - uses force magnitude threshold
  //   3. Returns false if no sensors found (caller should use phase-based estimation)
  bool extract_feet_contacts(
    const control_msgs::msg::Float64Values & interface_data, double & left_contact,
    double & right_contact);

  // Set default joint positions for relative position calculation
  void set_default_joint_positions(const std::vector<double> & default_positions);

  // Set default joint velocities (placeholder for future use)
  void set_default_joint_velocities(const std::vector<double> & default_velocities);

  // Define mapping between interface indices and names published by state_interfaces_broadcaster
  void set_interface_names(const std::vector<std::string> & interface_names);

  // Update action history (3 previous actions)
  void update_action_history(const std::vector<double> & action);

  // Set motor targets
  void set_motor_targets(const std::vector<double> & motor_targets);

  // Set feet contacts
  void set_feet_contacts(double left_contact, double right_contact);

  // Update imitation phase (increments phase counter and computes cos/sin)
  void update_imitation_phase(double phase_frequency_factor);

  // Set imitation phase counter directly (for state injection)
  void set_imitation_i(double imitation_i);

  // Get imitation phase values [cos, sin]
  std::vector<double> get_imitation_phase() const { return imitation_phase_; }

  // Get imitation phase counter (for logging/debugging)
  // Imitation: gait phase counter (0 to phase_period_) tracking position in walking cycle.
  // Used to encode rhythmic leg coordination via [cos(θ), sin(θ)] where θ = (imitation_i / phase_period_) * 2π.
  // The model was trained with imitation learning to follow reference gait patterns.
  double get_imitation_i() const { return imitation_i_; }

  // Get phase period (for logging/debugging)
  double get_phase_period() const { return phase_period_; }

  // Get action history for debugging
  std::vector<double> get_last_action() const { return last_action_; }
  std::vector<double> get_last_last_action() const { return last_last_action_; }
  std::vector<double> get_last_last_last_action() const { return last_last_last_action_; }

  // Set imitation phase period length (number of steps in one gait cycle)
  void set_phase_period(double nb_steps_in_period);

  // Set velocity commands (7D: 3 for base + 4 for head)
  void set_velocity_commands(const std::vector<double> & commands);

  // Set IMU upside down flag (inverts z-acceleration if true)
  // MuJoCo accelerometer reports negative z when standing (ground pushes down in sensor frame)
  // Set to true to invert z-axis to match expected +9.8 m/s² for standing robot
  void set_imu_upside_down(bool upside_down) { imu_upside_down_ = upside_down; }

  // Skip IMU z-axis inversion for injected data (data from Python already in correct format)
  // When true, z-acceleration is not inverted even if imu_upside_down_ is true
  void set_skip_imu_inversion(bool skip) { skip_imu_inversion_ = skip; }

  // Set gyro deadband threshold (filters out small drift/noise)
  void set_gyro_deadband(double deadband) { gyro_deadband_ = deadband; }

private:
  // Extract interface data (IMU and joint states) from float64_values message
  void extract_interface_data(
    const control_msgs::msg::Float64Values & msg, std::vector<double> & gyro,
    std::vector<double> & accelero, std::vector<double> & joint_positions,
    std::vector<double> & joint_velocities);

  // Format joint positions relative to default positions
  // Computes: current_position - default_position
  void format_joint_positions_relative(
    const std::vector<double> & joint_positions, std::vector<float> & observation);

  // Format joint velocities with 0.05 scaling
  void format_joint_velocities_scaled(
    const std::vector<double> & joint_velocities, std::vector<float> & observation);

  std::vector<std::string> joint_names_;
  size_t num_joints_;
  size_t
    observation_dim_;  // 17 + 6*N (gyro: 3, accelero: 3, commands: 7, joint_pos: N,
                       // joint_vel: N, last_action: N, last_last_action: N,
                       // last_last_last_action: N, motor_targets: N, feet_contacts: 2, phase: 2)

  // Default joint positions for relative position calculation
  std::vector<double> default_joint_positions_;
  bool default_joint_positions_set_;

  // Default joint velocities (placeholder for future use)
  std::vector<double> default_joint_velocities_;
  bool default_joint_velocities_set_;

  // IMU sensor name (configurable via parameter, defaults to "imu_2")
  std::string imu_sensor_name_;

  // IMU upside down flag (inverts z-acceleration if true)
  bool imu_upside_down_;

  // Skip IMU inversion flag (for injected data from Python that's already in correct format)
  bool skip_imu_inversion_;

  // Gyro deadband threshold (rad/s) - values below this are set to zero
  double gyro_deadband_;

  // Contact sensor names (configurable, defaults to common patterns)
  std::string left_foot_contact_sensor_name_;
  std::string right_foot_contact_sensor_name_;
  double contact_force_threshold_;  // Force threshold for FTS-based contact detection (N)

  // Names associated with incoming Float64Values message
  std::vector<std::string> interface_names_;

  // Track previous joint positions and velocities for relative calculations
  std::vector<double> previous_joint_positions_;
  std::vector<double> previous_joint_velocities_;
  bool previous_states_initialized_;

  // Action history (3 previous actions)
  std::vector<double> last_action_;
  std::vector<double> last_last_action_;
  std::vector<double> last_last_last_action_;

  // Motor targets
  std::vector<double> motor_targets_;

  // Feet contacts
  double left_foot_contact_;
  double right_foot_contact_;

  // Imitation phase state
  double imitation_i_;                   // phase counter
  double phase_period_;                  // nb_steps_in_period
  std::vector<double> imitation_phase_;  // [cos, sin]

  // Velocity commands (7D)
  std::vector<double> velocity_commands_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_
