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

#ifndef LOCOMOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_
#define LOCOMOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_

#include <string>
#include <vector>

#include "control_msgs/msg/interfaces_values.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace locomotion_controller
{

class ObservationFormatter
{
public:
  explicit ObservationFormatter(
    const std::vector<std::string> & joint_names, const std::string & imu_sensor_name = "imu_2");

  // Extract sensor data and format into model input vector
  // Observation order (based on
  // berkeley_humanoid_lite/tasks/locomotion/velocity/config/biped/env_cfg.py):
  // 1. Velocity commands (4D: lin_vel_x, lin_vel_y, ang_vel_z, heading)
  // 2. Base angular velocity (3D vector from IMU)
  // 3. Projected gravity vector (3D vector from IMU orientation)
  // 4. Joint positions (N joints, relative to default positions)
  // 5. Joint velocities (N joints, relative)
  // 6. Previous action (N joints)
  std::vector<float> format(
    const control_msgs::msg::InterfacesValues & interface_data,
    const geometry_msgs::msg::Twist & velocity_cmd, const std::vector<double> & previous_action);

  size_t get_observation_dim() const { return observation_dim_; }

  // Extract joint positions from interface data (for initializing default positions)
  std::vector<double> extract_joint_positions(
    const control_msgs::msg::InterfacesValues & interface_data);

  // Set default joint positions for relative position calculation
  void set_default_joint_positions(const std::vector<double> & default_positions);

  // Define mapping between interface indices and names published by interfaces_state_broadcaster
  void set_interface_names(const std::vector<std::string> & interface_names);

private:
  // Extract interface data (IMU and joint states) from interfaces_values message
  void extract_interface_data(
    const control_msgs::msg::InterfacesValues & msg, std::vector<double> & base_angular_velocity,
    std::vector<double> & projected_gravity, std::vector<double> & joint_positions,
    std::vector<double> & joint_velocities);

  // Format velocity commands from Twist message
  // Returns [lin_vel_x, lin_vel_y, ang_vel_z, heading]
  std::vector<float> format_velocity_commands(const geometry_msgs::msg::Twist & velocity_cmd);

  // Compute projected gravity vector from IMU orientation quaternion
  void compute_projected_gravity(
    double qx, double qy, double qz, double qw, std::vector<double> & projected_gravity);

  std::vector<std::string> joint_names_;
  size_t num_joints_;
  size_t observation_dim_;  // 10 + 3*N (velocity_commands: 4, base_ang_vel: 3, projected_gravity:
                            // 3, joint_pos: N, joint_vel: N, actions: N)

  // Default joint positions for relative position calculation
  std::vector<double> default_joint_positions_;
  bool default_joint_positions_set_;

  // IMU sensor name (configurable via parameter, defaults to "imu_2")
  std::string imu_sensor_name_;

  // Names associated with incoming InterfacesValues message
  std::vector<std::string> interface_names_;
};

}  // namespace locomotion_controller

#endif  // LOCOMOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_
