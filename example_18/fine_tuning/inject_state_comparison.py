#!/usr/bin/env python3
# Copyright (C) 2025 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Julia Jia

"""
Script to inject state from Python code path into C++ motion controller and compare outputs.

Workflow:
1. Reset controller state
2. Publish interface data and state data to injection topics
3. Call injection service
4. Send velocity command
5. Capture debug topics from one update cycle
6. Compare with Python path output
"""

import argparse
import sys
import time
import yaml
import numpy as np
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, String, UInt64
    from std_srvs.srv import Trigger
    from control_msgs.msg import Float64Values
    from example_18_motion_controller_msgs.msg import VelocityCommandWithHead
except ImportError:
    print("Error: ROS2 packages required.")
    sys.exit(1)


class StateInjector(Node):
    """Inject state and compare C++ vs Python outputs."""

    def __init__(self):
        super().__init__('state_injector')
        
        # Publishers
        self.yaml_path_pub = self.create_publisher(
            String, '/motion_controller/inject_yaml_path', 10)
        
        # Service clients
        self.reset_client = self.create_client(Trigger, '/motion_controller/reset_state')
        self.inject_client = self.create_client(Trigger, '/motion_controller/inject_state')
        self.resume_client = self.create_client(Trigger, '/motion_controller/resume_updates')
        
        # Velocity command publisher
        # Note: Topic name should match controller config (default: ~/cmd_head_velocity)
        # The ~ expands to /motion_controller/ if node is named motion_controller
        self.vel_cmd_pub = self.create_publisher(
            VelocityCommandWithHead, '/motion_controller/cmd_head_velocity', 10)
        
        # Subscribers for debug topics
        self.debug_formatted_observation = None
        self.debug_raw_action = None
        self.debug_processed_action = None
        self.debug_blended_action = None
        self.debug_rate_limited_action = None
        self.debug_update_complete = None
        self.update_complete_count = None
        
        self.sub_observation = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/formatted_observation',
            lambda msg: setattr(self, 'debug_formatted_observation', np.array(msg.data)), 10)
        self.sub_raw = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/raw_action',
            lambda msg: setattr(self, 'debug_raw_action', np.array(msg.data)), 10)
        self.sub_processed = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/processed_action',
            lambda msg: setattr(self, 'debug_processed_action', np.array(msg.data)), 10)
        self.sub_blended = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/blended_action',
            lambda msg: setattr(self, 'debug_blended_action', np.array(msg.data)), 10)
        self.sub_rate_limited = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/rate_limited_action',
            lambda msg: setattr(self, 'debug_rate_limited_action', np.array(msg.data)), 10)
        self.sub_update_complete = self.create_subscription(
            UInt64, '/motion_controller/debug/update_complete',
            lambda msg: setattr(self, 'update_complete_count', msg.data), 10)
        
        self.get_logger().info("State injector initialized")

    def reset_controller(self):
        """Reset controller state."""
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Reset service not available")
            return False
        
        request = Trigger.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info("Controller reset successful")
                return True
            else:
                self.get_logger().error(f"Reset failed: {response.message}")
                return False
        return False

    def create_yaml_file(self, yaml_path, interface_data, last_action, last_last_action, 
                         last_last_last_action, motor_targets, imitation_i, prev_motor_targets,
                         onnx_active_steps, stabilization_steps):
        """Create YAML file with injection data."""
        data = {
            'interface_data': {
                'values': interface_data.tolist() if isinstance(interface_data, np.ndarray) else interface_data
            },
            'controller_state': {
                'last_action': last_action.tolist() if isinstance(last_action, np.ndarray) else last_action,
                'last_last_action': last_last_action.tolist() if isinstance(last_last_action, np.ndarray) else last_last_action,
                'last_last_last_action': last_last_last_action.tolist() if isinstance(last_last_last_action, np.ndarray) else last_last_last_action,
                'motor_targets': motor_targets.tolist() if isinstance(motor_targets, np.ndarray) else motor_targets,
                'imitation_i': float(imitation_i),
                'prev_motor_targets': prev_motor_targets.tolist() if isinstance(prev_motor_targets, np.ndarray) else prev_motor_targets,
                'onnx_active_steps': int(onnx_active_steps),
                'stabilization_steps': int(stabilization_steps)
            }
        }
        
        with open(yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        self.get_logger().info(f"Created YAML file: {yaml_path}")

    def inject_state_from_yaml(self, yaml_file_path):
        """Inject state from YAML file."""
        # Publish YAML file path
        path_msg = String()
        path_msg.data = yaml_file_path
        self.yaml_path_pub.publish(path_msg)
        time.sleep(0.1)  # Wait for message to be received
        
        # Call injection service
        if not self.inject_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Injection service not available")
            return False
        
        request = Trigger.Request()
        future = self.inject_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info("State injection successful")
                return True
            else:
                self.get_logger().error(f"Injection failed: {response.message}")
                return False
        return False

    def send_velocity_command(self, lin_x=0.15, lin_y=0.0, ang_z=0.0, head_commands=None):
        """Send velocity command."""
        msg = VelocityCommandWithHead()
        msg.base_velocity.linear.x = float(lin_x)
        msg.base_velocity.linear.y = float(lin_y)
        msg.base_velocity.angular.z = float(ang_z)
        msg.head_commands = head_commands if head_commands else [0.0, 0.0, 0.0, 0.0]
        
        # Publish multiple times to ensure it's received
        for _ in range(10):
            self.vel_cmd_pub.publish(msg)
            time.sleep(0.01)
        
        # Wait a bit longer to ensure it's in the buffer
        time.sleep(0.1)
        
        self.get_logger().info(f"Sent velocity command: lin_x={lin_x:.3f}")
    
    def resume_updates(self):
        """Resume controller updates after injection."""
        if not self.resume_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Resume service not available")
            return False
        
        request = Trigger.Request()
        future = self.resume_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info("Updates resumed successfully")
                return True
            else:
                self.get_logger().error(f"Resume failed: {response.message}")
                return False
        return False

    def capture_debug_topics(self, timeout=2.0, wait_for_update_complete=True):
        """Capture debug topics from one update cycle.
        
        If wait_for_update_complete is True, waits for update_complete marker
        to ensure we capture from the correct update cycle.
        """
        start_time = time.time()
        initial_update_count = self.update_complete_count
        
        # Reset captured data
        self.debug_formatted_observation = None
        self.debug_raw_action = None
        self.debug_processed_action = None
        self.debug_blended_action = None
        self.debug_rate_limited_action = None
        
        if wait_for_update_complete:
            # Wait for update completion marker (new update cycle)
            while time.time() - start_time < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                if (self.update_complete_count is not None and 
                    self.update_complete_count != initial_update_count):
                    # New update cycle detected, now capture all topics
                    break
            
            if self.update_complete_count is None or self.update_complete_count == initial_update_count:
                self.get_logger().warn("Update completion marker not received")
                return False
        
        # Now capture all debug topics
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.debug_formatted_observation is not None and
                self.debug_raw_action is not None and
                self.debug_processed_action is not None and
                self.debug_blended_action is not None and
                self.debug_rate_limited_action is not None):
                self.get_logger().info(f"Captured debug topics from update cycle {self.update_complete_count}")
                return True
        
        return False

    def load_injection_yaml(self, yaml_file_path):
        """Load injection YAML file."""
        try:
            with open(yaml_file_path, 'r') as f:
                data = yaml.safe_load(f)
            return data
        except Exception as e:
            self.get_logger().error(f"Error loading injection YAML: {e}")
            return None

    def compute_python_observation(self, injection_yaml_path, velocity_cmd, num_joints=14):
        """Compute Python observation from injection YAML data.
        
        Matches observation_formatter.cpp format_observation logic.
        Note: For injected data, the C++ code skips IMU z-axis inversion,
        so we also don't apply inversion here.
        
        Args:
            injection_yaml_path: Path to YAML file with injection data
            velocity_cmd: Velocity command (Twist message)
            num_joints: Number of joints (default: 14)
        """
        data = self.load_injection_yaml(injection_yaml_path)
        if data is None:
            return None
        
        interface_data = data['interface_data']
        controller_state = data['controller_state']
        
        # Extract interface data components
        # Format: [0-3: IMU quaternion, 4-6: gyro, 7-9: accelero, 10-23: joint_pos, 24-37: joint_vel, 38-39: contacts, 40-42: velocimeter]
        interface_values = np.array(interface_data['values'])
        
        self.get_logger().info(f"Interface data size: {len(interface_values)}")
        self.get_logger().info(f"First 10 interface values: {interface_values[:10].tolist()}")
        
        # Check if interface data has quaternion prefix (should be 43 values for 14 joints)
        # If it's shorter, it might not have quaternion or velocimeter
        if len(interface_values) >= 10 + 2 * num_joints:
            # Full format with quaternion
            gyro = interface_values[4:7]  # indices 4-6
            accelero = interface_values[7:10]  # indices 7-9
            joint_positions = interface_values[10:10+num_joints]  # indices 10-23
            joint_velocities = interface_values[10+num_joints:10+2*num_joints]  # indices 24-37
            # Contacts are at indices 38-39
            left_contact = interface_values[38] if len(interface_values) > 38 else 0.0
            right_contact = interface_values[39] if len(interface_values) > 39 else 0.0
            
            self.get_logger().info(f"Extracted (with quaternion): gyro={gyro.tolist()}, accelero={accelero.tolist()}")
        else:
            # Possibly missing quaternion prefix - try direct indexing
            # Assume format: [gyro(3), accelero(3), joint_pos(14), joint_vel(14), contacts(2), ...]
            if len(interface_values) >= 3:
                gyro = interface_values[0:3]
            else:
                gyro = np.zeros(3)
            if len(interface_values) >= 6:
                accelero = interface_values[3:6]
            else:
                accelero = np.zeros(3)
            if len(interface_values) >= 6 + num_joints:
                joint_positions = interface_values[6:6+num_joints]
            else:
                joint_positions = np.zeros(num_joints)
            if len(interface_values) >= 6 + 2*num_joints:
                joint_velocities = interface_values[6+num_joints:6+2*num_joints]
            else:
                joint_velocities = np.zeros(num_joints)
            # Contacts
            if len(interface_values) >= 6 + 2*num_joints + 2:
                left_contact = interface_values[6 + 2*num_joints]
                right_contact = interface_values[6 + 2*num_joints + 1]
            else:
                left_contact = 0.0
                right_contact = 0.0
        
        obs = []
        
        # 1. Gyro (3D) - apply deadband (threshold 0.01)
        for val in gyro:
            obs.append(0.0 if abs(val) < 0.01 else float(val))
        
        # 2. Accelerometer (3D) with x-bias +1.3
        # Note: The injected data contains raw MuJoCo sensor values from Python (no z-axis inversion applied).
        # The C++ code now skips IMU inversion for injected data (set_skip_imu_inversion(true)),
        # so we also don't apply inversion here to match the C++ behavior.
        obs.append(float(accelero[0] + 1.3))
        obs.append(float(accelero[1]))
        # Don't invert z-acceleration for injected data (C++ code skips inversion for injected data)
        obs.append(float(accelero[2]))
        
        # 3. Velocity commands (7D: lin_x, lin_y, ang_z, head_1-4)
        obs.append(float(velocity_cmd.linear.x))
        obs.append(float(velocity_cmd.linear.y))
        obs.append(float(velocity_cmd.angular.z))
        obs.extend([0.0, 0.0, 0.0, 0.0])  # Head commands
        
        # 4. Joint positions relative to default (N)
        # Default positions should come from the controller config or be initialized from first sensor reading
        # For now, use motor_targets as approximation (but this might not be correct)
        # The C++ code initializes default_joint_positions from first sensor reading or config
        motor_targets = np.array(controller_state.get('motor_targets', [0.0]*num_joints))
        prev_motor_targets = np.array(controller_state.get('prev_motor_targets', [0.0]*num_joints))
        
        # Try to infer default positions: if motor_targets == prev_motor_targets, they might be defaults
        # Otherwise, use joint_positions as defaults (first reading assumption)
        # Actually, the safest is to use joint_positions themselves as the "default" for relative calculation
        # But C++ uses actual default_joint_positions from config or first reading
        # For comparison, we should use the same default that C++ uses
        # Since we don't have access to C++ default, use joint_positions as baseline (relative = 0)
        # Or better: use motor_targets if they're close to joint_positions
        default_joint_positions = joint_positions.copy()  # This makes rel_positions = 0, which might be wrong
        # Alternative: use motor_targets
        # default_joint_positions = motor_targets.copy()
        rel_positions = joint_positions - default_joint_positions
        obs.extend(rel_positions.tolist())
        
        # 5. Joint velocities scaled by 0.05 (N)
        dof_vel_scale = 0.05
        scaled_velocities = joint_velocities * dof_vel_scale
        obs.extend(scaled_velocities.tolist())
        
        # 6-8. Action history (3*N)
        last_action = np.array(controller_state.get('last_action', [0.0]*num_joints))
        last_last_action = np.array(controller_state.get('last_last_action', [0.0]*num_joints))
        last_last_last_action = np.array(controller_state.get('last_last_last_action', [0.0]*num_joints))
        obs.extend(last_action.tolist())
        obs.extend(last_last_action.tolist())
        obs.extend(last_last_last_action.tolist())
        
        # 9. Motor targets (N)
        obs.extend(motor_targets.tolist())
        
        # 10. Feet contacts (2D)
        obs.append(float(left_contact))
        obs.append(float(right_contact))
        
        # 11. Imitation phase (2D) - cos/sin encoding
        imitation_i = controller_state.get('imitation_i', 0.0)
        phase_period = 50.0
        phase_theta = (imitation_i / phase_period) * 2.0 * np.pi
        obs.append(float(np.cos(phase_theta)))
        obs.append(float(np.sin(phase_theta)))
        
        return np.array(obs, dtype=np.float32)

    def load_python_outputs(self, yaml_file_path):
        """Load Python outputs from YAML file.
        
        Expected format:
        python_outputs:
          raw_action: [...]
          processed_action: [...]
          blended_action: [...]
          rate_limited_action: [...]
        
        Or flat format:
        raw_action: [...]
        processed_action: [...]
        blended_action: [...]
        rate_limited_action: [...]
        """
        try:
            with open(yaml_file_path, 'r') as f:
                data = yaml.safe_load(f)
            
            # Handle nested format
            if 'python_outputs' in data:
                outputs = data['python_outputs']
            else:
                outputs = data
            
            raw_action = np.array(outputs.get('raw_action', []), dtype=np.float64)
            processed_action = np.array(outputs.get('processed_action', []), dtype=np.float64)
            blended_action = np.array(outputs.get('blended_action', []), dtype=np.float64)
            rate_limited_action = np.array(outputs.get('rate_limited_action', []), dtype=np.float64)
            
            self.get_logger().info(f"Loaded Python outputs from {yaml_file_path}")
            return raw_action, processed_action, blended_action, rate_limited_action
            
        except FileNotFoundError:
            self.get_logger().error(f"Python outputs file not found: {yaml_file_path}")
            return None, None, None, None
        except Exception as e:
            self.get_logger().error(f"Error loading Python outputs: {e}")
            return None, None, None, None

    def compare_observation(self, python_observation):
        """Compare C++ and Python observations."""
        if self.debug_formatted_observation is None:
            print("\nFormatted Observation: C++ data not captured")
            return False
        if python_observation is None:
            print("\nFormatted Observation: Python data not provided")
            return False
        
        cpp_obs = self.debug_formatted_observation
        py_obs = python_observation
        
        if len(cpp_obs) != len(py_obs):
            print(f"\nFormatted Observation: Length mismatch - C++: {len(cpp_obs)}, Python: {len(py_obs)}")
            return False
        
        abs_diff = np.abs(cpp_obs - py_obs)
        max_diff = np.max(abs_diff)
        mean_diff = np.mean(abs_diff)
        
        # Tolerance check
        abs_tol = 1e-5
        rel_tol = 1e-4
        rel_diff = abs_diff / (np.abs(py_obs) + 1e-8)
        passed = np.all((abs_diff < abs_tol) | (rel_diff < rel_tol))
        
        print("\n" + "="*60)
        print("Observation Comparison")
        print("="*60)
        print(f"\nFormatted Observation: {'✓ PASS' if passed else '✗ FAIL'}")
        print(f"  Max absolute diff: {max_diff:.6e}")
        print(f"  Mean absolute diff: {mean_diff:.6e}")
        print(f"  Dimension: {len(cpp_obs)}")
        print(f"  C++ (first 10): {cpp_obs[:10].tolist()}")
        print(f"  Python (first 10): {py_obs[:10].tolist()}")
        
        if not passed:
            worst_idx = np.argmax(abs_diff)
            print(f"  Worst diff at index {worst_idx}: C++={cpp_obs[worst_idx]:.6f}, "
                  f"Python={py_obs[worst_idx]:.6f}, diff={abs_diff[worst_idx]:.6e}")
            
            # Show breakdown by observation components
            print(f"\n  Observation component breakdown:")
            print(f"    Gyro (0-2): max_diff={np.max(abs_diff[0:3]):.6e}")
            print(f"    Accelero (3-5): max_diff={np.max(abs_diff[3:6]):.6e}")
            print(f"    Commands (6-12): max_diff={np.max(abs_diff[6:13]):.6e}")
            if len(cpp_obs) > 13:
                joint_start = 13
                joint_end = 13 + 14  # Assuming 14 joints
                if len(cpp_obs) > joint_end:
                    print(f"    Joint pos rel (13-26): max_diff={np.max(abs_diff[joint_start:joint_end]):.6e}")
                if len(cpp_obs) > joint_end + 14:
                    print(f"    Joint vel (27-40): max_diff={np.max(abs_diff[joint_end:joint_end+14]):.6e}")
        
        return passed

    def compare_outputs(self, python_raw_action, python_processed_action, 
                       python_blended_action, python_rate_limited_action):
        """Compare C++ and Python outputs."""
        print("\n" + "="*60)
        print("Action Comparison Results")
        print("="*60)
        
        stages = [
            ("Raw Action", self.debug_raw_action, python_raw_action),
            ("Processed Action", self.debug_processed_action, python_processed_action),
            ("Blended Action", self.debug_blended_action, python_blended_action),
            ("Rate-Limited Action", self.debug_rate_limited_action, python_rate_limited_action),
        ]
        
        all_match = True
        for stage_name, cpp_data, python_data in stages:
            if cpp_data is None:
                print(f"\n{stage_name}: C++ data not captured")
                all_match = False
                continue
            if python_data is None:
                print(f"\n{stage_name}: Python data not provided")
                all_match = False
                continue
            
            if len(cpp_data) != len(python_data):
                print(f"\n{stage_name}: Length mismatch - C++: {len(cpp_data)}, Python: {len(python_data)}")
                all_match = False
                continue
            
            abs_diff = np.abs(cpp_data - python_data)
            max_diff = np.max(abs_diff)
            mean_diff = np.mean(abs_diff)
            
            # Tolerance check
            abs_tol = 1e-5
            rel_tol = 1e-4
            rel_diff = abs_diff / (np.abs(python_data) + 1e-8)
            passed = np.all((abs_diff < abs_tol) | (rel_diff < rel_tol))
            
            print(f"\n{stage_name}: {'✓ PASS' if passed else '✗ FAIL'}")
            print(f"  Max absolute diff: {max_diff:.6e}")
            print(f"  Mean absolute diff: {mean_diff:.6e}")
            print(f"  C++ (first 5): {cpp_data[:5].tolist()}")
            print(f"  Python (first 5): {python_data[:5].tolist()}")
            
            if not passed:
                all_match = False
                worst_idx = np.argmax(abs_diff)
                print(f"  Worst diff at index {worst_idx}: C++={cpp_data[worst_idx]:.6f}, "
                      f"Python={python_data[worst_idx]:.6f}, diff={abs_diff[worst_idx]:.6e}")
        
        print("\n" + "="*60)
        print(f"Overall: {'✓ ALL PASS' if all_match else '✗ SOME FAILED'}")
        print("="*60)
        
        return all_match


def main():
    parser = argparse.ArgumentParser(description='Inject state and compare C++ vs Python')
    parser.add_argument('--yaml-file', type=str, required=True,
                       help='Path to YAML file with injection data')
    parser.add_argument('--num-joints', type=int, default=14,
                       help='Number of joints (default: 14)')
    parser.add_argument('--create-yaml', action='store_true',
                       help='Create YAML file from provided data (requires --interface-data and --state-data)')
    parser.add_argument('--interface-data', type=str,
                       help='Interface data (comma-separated or file path)')
    parser.add_argument('--state-data', type=str,
                       help='State data (comma-separated or file path)')
    parser.add_argument('--python-outputs', type=str,
                       help='Path to Python output data file for comparison')
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        injector = StateInjector()
        
        # Create YAML if requested
        if args.create_yaml:
            if not args.interface_data or not args.state_data:
                print("Error: --interface-data and --state-data required when --create-yaml is used")
                return 1
            # TODO: Parse interface_data and state_data, then create YAML
            print("YAML creation not yet implemented - provide YAML file directly")
            return 1
        
        print("Reset controller...")
        if not injector.reset_controller():
            return 1
        
        # Send velocity command FIRST, before injection
        # This ensures it's in the buffer when the update cycle runs
        print("Send velocity command (before injection)...")
        injector.send_velocity_command(lin_x=0.15)
        
        # Wait to ensure velocity command is in buffer
        time.sleep(0.1)
        for _ in range(5):
            rclpy.spin_once(injector, timeout_sec=0.01)
        
        print(f"Inject state from YAML: {args.yaml_file}...")
        if not injector.inject_state_from_yaml(args.yaml_file):
            return 1
        
        # Create velocity command for Python computation
        from geometry_msgs.msg import Twist
        velocity_cmd = Twist()
        velocity_cmd.linear.x = 0.15
        velocity_cmd.linear.y = 0.0
        velocity_cmd.angular.z = 0.0
        
        # Wait for injection to complete and ensure velocity command is still in buffer
        time.sleep(0.1)
        for _ in range(5):
            rclpy.spin_once(injector, timeout_sec=0.01)
        
        # Re-send velocity command to ensure it's in buffer after injection
        print("Re-send velocity command (after injection)...")
        injector.send_velocity_command(lin_x=0.15)
        time.sleep(0.1)
        
        # Capture debug topics BEFORE resuming updates
        # The controller allows one update cycle when ignore_interface_data_updates_ is true,
        # which will format and publish debug topics while keeping injected data protected
        print("Capture debug topics (before resuming updates)...")
        
        # Reset captured data to ensure we get fresh data from the injected update cycle
        injector.debug_formatted_observation = None
        injector.debug_raw_action = None
        injector.debug_processed_action = None
        injector.debug_blended_action = None
        injector.debug_rate_limited_action = None
        
        # Wait a bit for the one allowed update cycle to run after injection
        time.sleep(0.1)
        
        # Spin to receive the debug topics from the injected update cycle
        start_time = time.time()
        while time.time() - start_time < 1.0:  # Wait up to 1 second
            rclpy.spin_once(injector, timeout_sec=0.05)
            # Check if we've captured all topics
            if (injector.debug_formatted_observation is not None and
                injector.debug_raw_action is not None and
                injector.debug_processed_action is not None and
                injector.debug_blended_action is not None and
                injector.debug_rate_limited_action is not None):
                print(f"Captured all debug topics from injected update cycle")
                # Verify we got the correct observation (check accelerometer z-value)
                if injector.debug_formatted_observation is not None and len(injector.debug_formatted_observation) >= 6:
                    accel_z = injector.debug_formatted_observation[5]
                    print(f"Captured observation accel_z = {accel_z:.6f} (expected ~11.074 for injected data)")
                break
        
        if injector.debug_formatted_observation is not None:
            print("Captured all debug topics")
            
            # Compute Python observation
            print("Computing Python observation...")
            python_observation = injector.compute_python_observation(
                args.yaml_file, velocity_cmd, args.num_joints)
            
            # Compare observations first
            if python_observation is not None:
                obs_passed = injector.compare_observation(python_observation)
            else:
                print("Failed to compute Python observation")
                obs_passed = False
            
            # Note: No need to resume updates for debugging - we only need one cycle for comparison
            # If you want the controller to continue running, call resume_updates() manually
            
            if args.python_outputs:
                print(f"Loading Python outputs from {args.python_outputs}...")
                python_raw, python_processed, python_blended, python_rate_limited = \
                    injector.load_python_outputs(args.python_outputs)
                
                if python_raw is not None:
                    print("Comparing C++ and Python outputs...")
                    injector.compare_outputs(
                        python_raw, python_processed, python_blended, python_rate_limited)
                else:
                    print("Failed to load Python outputs")
                    return 1
            else:
                print("No Python outputs file provided - skipping action comparison")
                print("Use --python-outputs to provide a YAML file with Python outputs")
        else:
            print("Failed to capture all debug topics")
            return 1
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

