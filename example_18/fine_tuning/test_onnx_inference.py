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
Test script to compare ONNX inference between Python and C++ implementations.

Loads 5 observations from Python logs and compares action outputs.
"""

import argparse
import random
import re
import sys
import numpy as np
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import Trigger
except ImportError:
    print("Error: ROS2 packages required.")
    sys.exit(1)

# Add Open Duck Playground to path for OnnxInfer
possible_playground_paths = [
    Path(__file__).parent.parent.parent.parent / "Open_Duck_Playground",
    Path.home() / "dev" / "Open_Duck_Playground",
    Path("/home/juliajia/dev/Open_Duck_Playground"),
]

playground_path = None
for path in possible_playground_paths:
    if path.exists():
        playground_path = path
        sys.path.insert(0, str(path))
        try:
            from playground.common.onnx_infer import OnnxInfer
            break
        except ImportError:
            pass

if playground_path is None:
    print("Error: Open Duck Playground not found or OnnxInfer not available.")
    sys.exit(1)


def parse_observation_from_log(log_line):
    """Extract observation array from log line: [RAW_OBS] step=N, obs=[...]"""
    match = re.search(r'obs=\[(.*?)\]', log_line)
    if not match:
        return None
    obs_str = match.group(1)
    try:
        obs = [float(x.strip()) for x in obs_str.split(',')]
        return np.array(obs, dtype=np.float32)
    except ValueError:
        return None


def load_observations_from_log(log_file, num_obs=5):
    """Load observations from log file, randomly selecting with variety in linear velocity.
    
    Groups observations by linear velocity ranges and randomly picks from each group
    to ensure variety. Linear velocity is at index 6 in the observation vector.
    Ensures at least 3 observations with lin_x = 0.15 are selected.
    """
    
    # Parse all observations with their linear velocities
    all_observations = []
    target_vel_obs = []  # Observations with lin_x = 0.15 (with tolerance)
    with open(log_file, 'r') as f:
        for line in f:
            if '[RAW_OBS]' in line:
                obs = parse_observation_from_log(line)
                if obs is not None and len(obs) > 6:
                    lin_vel = obs[6]  # Linear velocity (lin_x) is at index 6
                    all_observations.append((lin_vel, obs))
                    # Check if lin_x is close to 0.15 (within 0.01 tolerance)
                    if abs(lin_vel - 0.15) < 0.01:
                        target_vel_obs.append(obs)
    
    if len(all_observations) == 0:
        return []
    
    selected = []
    
    # First, ensure at least 3 observations with lin_x = 0.15
    if len(target_vel_obs) >= 3:
        # Randomly select 3 from target velocity observations
        selected.extend(random.sample(target_vel_obs, 3))
    elif len(target_vel_obs) > 0:
        # Use all available target velocity observations
        selected.extend(target_vel_obs)
    
    # Group remaining observations by velocity ranges
    # Define velocity bins: [0.0], [0.01-0.05], [0.06-0.10], [0.11-0.15], [>0.15]
    velocity_groups = {
        'zero': [],      # 0.0
        'low': [],       # 0.01-0.05
        'medium': [],    # 0.06-0.10
        'high': [],      # 0.11-0.15 (but not exactly 0.15)
        'very_high': []  # >0.15
    }
    
    # Filter out already selected observations
    selected_set = set(id(obs) for obs in selected)
    remaining_obs = [(lin_vel, obs) for lin_vel, obs in all_observations 
                     if id(obs) not in selected_set]
    
    for lin_vel, obs in remaining_obs:
        if abs(lin_vel) < 0.005:  # Essentially zero
            velocity_groups['zero'].append(obs)
        elif 0.01 <= abs(lin_vel) <= 0.05:
            velocity_groups['low'].append(obs)
        elif 0.06 <= abs(lin_vel) <= 0.10:
            velocity_groups['medium'].append(obs)
        elif 0.11 <= abs(lin_vel) <= 0.15 and abs(lin_vel - 0.15) >= 0.01:
            velocity_groups['high'].append(obs)
        else:  # >0.15
            velocity_groups['very_high'].append(obs)
    
    # Select remaining observations from different groups
    non_empty_groups = [(name, group) for name, group in velocity_groups.items() if len(group) > 0]
    
    if len(non_empty_groups) == 0:
        # Fallback: fill from any remaining observations
        if len(selected) < num_obs:
            remaining = [obs for _, obs in remaining_obs]
            needed = num_obs - len(selected)
            if len(remaining) > 0:
                selected.extend(random.sample(remaining, min(needed, len(remaining))))
        random.shuffle(selected)
        return selected[:num_obs]
    
    # Distribute remaining selections across groups
    remaining_needed = num_obs - len(selected)
    if remaining_needed > 0:
        num_groups = len(non_empty_groups)
        obs_per_group = max(1, remaining_needed // num_groups)
        remainder = remaining_needed % num_groups
        
        # Shuffle groups for randomness
        random.shuffle(non_empty_groups)
        
        for i, (group_name, group) in enumerate(non_empty_groups):
            # Add one extra observation to first 'remainder' groups
            count = obs_per_group + (1 if i < remainder else 0)
            if len(selected) + count > num_obs:
                count = num_obs - len(selected)
            
            if count > 0:
                # Randomly select from this group
                selected.extend(random.sample(group, min(count, len(group))))
            
            if len(selected) >= num_obs:
                break
    
    # If we still need more, fill from any remaining observations
    if len(selected) < num_obs:
        remaining = [obs for _, obs in remaining_obs if obs not in selected]
        needed = num_obs - len(selected)
        if len(remaining) > 0:
            selected.extend(random.sample(remaining, min(needed, len(remaining))))
    
    # Shuffle final selection for randomness
    random.shuffle(selected)
    return selected[:num_obs]


class InferenceTester(Node):
    """Test ONNX inference comparison."""

    def __init__(self, onnx_model_path):
        super().__init__('inference_tester')
        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(onnx_model_path, awd=True)
        
        # ROS2 clients
        self.test_obs_pub = self.create_publisher(
            Float64MultiArray, '/motion_controller/test_observation', 10)
        self.test_action_sub = self.create_subscription(
            Float64MultiArray, '/motion_controller/test_action_output',
            self.action_callback, 10)
        self.test_hw_cmd_sub = self.create_subscription(
            Float64MultiArray, '/motion_controller/test_hardware_commands',
            self.hardware_commands_callback, 10)
        self.inference_client = self.create_client(Trigger, '/motion_controller/test_inference')
        
        self.received_action = None
        self.received_hardware_commands = None
        self.waiting_for_action = False
        self.waiting_for_hardware_commands = False
        
        # Processing parameters (matching motion_controller config)
        # Note: motion_controller uses action_scale=0.25, but manual_control_ros2.py uses 0.3
        # Using motion_controller values for consistency
        self.action_scale = 0.25
        self.max_motor_velocity = 5.24  # rad/s (matching motion_controller config)
        self.control_period = 0.02  # 50Hz
        self.blend_in_steps = 200
        self.onnx_active_steps = 200  # Assume fully blended for testing
        
        # Default joint positions (from open_duck_mini_controllers.yaml)
        # Order: left leg (5), head (4), right leg (5)
        self.default_joint_positions = np.array([
            0.002, 0.053, -0.63, 1.368, -0.784,  # left leg
            0.0, 0.0, 0.0, 0.0,  # head
            -0.003, -0.065, 0.635, 1.379, -0.796  # right leg
        ], dtype=np.float64)
        
        # Previous motor targets (initialize to default positions)
        self.prev_motor_targets = self.default_joint_positions.copy()
        self.prev_motor_targets_initialized = True

    def action_callback(self, msg):
        """Callback for receiving C++ action output."""
        if self.waiting_for_action:
            self.received_action = np.array(msg.data, dtype=np.float64)
            self.waiting_for_action = False

    def hardware_commands_callback(self, msg):
        """Callback for receiving C++ hardware commands."""
        if self.waiting_for_hardware_commands:
            self.received_hardware_commands = np.array(msg.data, dtype=np.float64)
            self.waiting_for_hardware_commands = False

    def run_cpp_inference(self, observation):
        """Run inference via C++ motion_controller."""
        # Publish observation
        msg = Float64MultiArray()
        msg.data = observation.tolist()
        self.test_obs_pub.publish(msg)
        
        # Wait for publisher to register
        import time
        time.sleep(0.1)
        
        # Call service
        self.waiting_for_action = True
        self.waiting_for_hardware_commands = True
        self.received_action = None
        self.received_hardware_commands = None
        
        request = Trigger.Request()
        future = self.inference_client.call_async(request)
        
        # Wait for service response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            return None, None, "Service call timeout"
        
        response = future.result()
        if not response.success:
            return None, None, response.message
        
        # Wait for both action output and hardware commands (with timeout)
        timeout = 2.0
        start_time = time.time()
        while ((self.waiting_for_action or self.waiting_for_hardware_commands) and 
               (time.time() - start_time) < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.waiting_for_action:
            return None, None, "Timeout waiting for action output"
        if self.waiting_for_hardware_commands:
            return None, None, "Timeout waiting for hardware commands"
        
        return self.received_action, self.received_hardware_commands, None

    def run_python_inference(self, observation):
        """Run inference via Python OnnxInfer."""
        return self.policy.infer(observation)
    
    def process_python_action_to_hardware(self, python_action):
        """Process Python ONNX action through full pipeline to get hardware commands.
        
        Matches the processing in motion_controller.cpp update() method:
        1. Process through action_processor (scale + default offset)
        2. Apply blend-in factor
        3. Apply rate limiting (motor velocity limits)
        """
        # Step 1: Convert action to motor targets (action_scale * action + default_positions)
        # This matches action_processor_->process() in motion_controller.cpp
        desired_motor_targets = self.default_joint_positions + python_action * self.action_scale
        
        # Step 2: Apply blend-in factor (assume fully blended for testing)
        blend_factor = min(1.0, self.onnx_active_steps / self.blend_in_steps)
        blended_targets = (
            (1.0 - blend_factor) * self.default_joint_positions +
            blend_factor * desired_motor_targets
        )
        
        # Step 3: Apply rate limiting (motor velocity limits)
        # This matches the rate limiting in motion_controller.cpp lines 655-671
        if self.prev_motor_targets_initialized:
            max_change = self.max_motor_velocity * self.control_period
            hardware_commands = np.clip(
                blended_targets,
                self.prev_motor_targets - max_change,
                self.prev_motor_targets + max_change
            )
            # Update previous motor targets for next iteration
            self.prev_motor_targets = hardware_commands.copy()
        else:
            # No rate limiting if no previous targets
            hardware_commands = blended_targets
            self.prev_motor_targets = hardware_commands.copy()
            self.prev_motor_targets_initialized = True
        
        return hardware_commands

    def compare_actions(self, cpp_action, python_action, obs_idx, observation=None, 
                       cpp_hardware_commands=None, python_hardware_commands=None):
        """Compare actions and report differences."""
        if cpp_action is None:
            print(f"  Observation {obs_idx}: C++ inference failed")
            return False
        
        if cpp_action.shape != python_action.shape:
            print(f"  Observation {obs_idx}: Shape mismatch - C++: {cpp_action.shape}, Python: {python_action.shape}")
            return False
        
        abs_diff = np.abs(cpp_action - python_action)
        rel_diff = abs_diff / (np.abs(python_action) + 1e-8)
        
        max_abs_diff = np.max(abs_diff)
        mean_abs_diff = np.mean(abs_diff)
        max_rel_diff = np.max(rel_diff)
        
        # Tolerance: absolute < 1e-5 OR relative < 1e-4
        abs_tol = 1e-5
        rel_tol = 1e-4
        passed = np.all((abs_diff < abs_tol) | (rel_diff < rel_tol))
        
        print(f"  Observation {obs_idx}:")
        
        # Output observation (first few and last few values for brevity)
        if observation is not None:
            obs_preview = f"[{','.join([f'{v:.3f}' for v in observation[:5]])}...{','.join([f'{v:.3f}' for v in observation[-5:]])}]"
            print(f"    Observation (first 5, last 5): {obs_preview}")
            if len(observation) > 8:
                # Velocity commands are at indices 6-8: lin_vel_x, lin_vel_y, ang_vel_z
                print(f"    Velocity commands: lin_x={observation[6]:.3f}, lin_y={observation[7]:.3f}, ang_z={observation[8]:.3f}")
        
        # Output action values (first few and last few for brevity)
        action_preview_cpp = f"[{','.join([f'{v:.6f}' for v in cpp_action[:5]])}...{','.join([f'{v:.6f}' for v in cpp_action[-5:]])}]"
        action_preview_py = f"[{','.join([f'{v:.6f}' for v in python_action[:5]])}...{','.join([f'{v:.6f}' for v in python_action[-5:]])}]"
        print(f"    C++ action (first 5, last 5): {action_preview_cpp}")
        print(f"    Python action (first 5, last 5): {action_preview_py}")
        
        # Output full action arrays for detailed inspection
        print(f"    C++ action (full): [{','.join([f'{v:.6f}' for v in cpp_action])}]")
        print(f"    Python action (full): [{','.join([f'{v:.6f}' for v in python_action])}]")
        
        print(f"    Max absolute diff: {max_abs_diff:.6e}")
        print(f"    Mean absolute diff: {mean_abs_diff:.6e}")
        print(f"    Max relative diff: {max_rel_diff:.6e}")
        print(f"    Status: {'PASS' if passed else 'FAIL'}")
        
        if not passed:
            # Find indices with largest differences
            worst_indices = np.argsort(abs_diff)[-5:][::-1]
            print(f"    Worst differences at indices: {worst_indices.tolist()}")
            for idx in worst_indices:
                print(f"      [{idx}]: C++={cpp_action[idx]:.6f}, Python={python_action[idx]:.6f}, diff={abs_diff[idx]:.6e}")
        
        # Compare hardware commands if provided
        hw_passed = True
        if cpp_hardware_commands is not None and python_hardware_commands is not None:
            print(f"\n    Hardware Commands Comparison:")
            if cpp_hardware_commands.shape != python_hardware_commands.shape:
                print(f"      Shape mismatch - C++: {cpp_hardware_commands.shape}, Python: {python_hardware_commands.shape}")
                hw_passed = False
            else:
                hw_abs_diff = np.abs(cpp_hardware_commands - python_hardware_commands)
                hw_rel_diff = hw_abs_diff / (np.abs(python_hardware_commands) + 1e-8)
                
                hw_max_abs_diff = np.max(hw_abs_diff)
                hw_mean_abs_diff = np.mean(hw_abs_diff)
                hw_max_rel_diff = np.max(hw_rel_diff)
                
                # Tolerance for hardware commands (slightly relaxed due to processing differences)
                hw_abs_tol = 1e-4
                hw_rel_tol = 1e-3
                hw_passed = np.all((hw_abs_diff < hw_abs_tol) | (hw_rel_diff < hw_rel_tol))
                
                hw_preview_cpp = f"[{','.join([f'{v:.6f}' for v in cpp_hardware_commands[:5]])}...{','.join([f'{v:.6f}' for v in cpp_hardware_commands[-5:]])}]"
                hw_preview_py = f"[{','.join([f'{v:.6f}' for v in python_hardware_commands[:5]])}...{','.join([f'{v:.6f}' for v in python_hardware_commands[-5:]])}]"
                print(f"      C++ hardware commands (first 5, last 5): {hw_preview_cpp}")
                print(f"      Python hardware commands (first 5, last 5): {hw_preview_py}")
                print(f"      Max absolute diff: {hw_max_abs_diff:.6e}")
                print(f"      Mean absolute diff: {hw_mean_abs_diff:.6e}")
                print(f"      Max relative diff: {hw_max_rel_diff:.6e}")
                print(f"      Status: {'PASS' if hw_passed else 'FAIL'}")
                
                if not hw_passed:
                    worst_indices = np.argsort(hw_abs_diff)[-5:][::-1]
                    print(f"      Worst differences at indices: {worst_indices.tolist()}")
                    for idx in worst_indices:
                        print(f"        [{idx}]: C++={cpp_hardware_commands[idx]:.6f}, Python={python_hardware_commands[idx]:.6f}, diff={hw_abs_diff[idx]:.6e}")
        
        return passed and hw_passed


def main():
    parser = argparse.ArgumentParser(description='Test ONNX inference comparison')
    parser.add_argument('--log-file', type=str, required=True,
                        help='Path to Python log file with [RAW_OBS] entries')
    parser.add_argument('--onnx-model', type=str, required=True,
                        help='Path to ONNX model file')
    parser.add_argument('--num-obs', type=int, default=5,
                        help='Number of observations to test (default: 5)')
    
    args = parser.parse_args()
    
    # Load observations from log
    print(f"Loading {args.num_obs} observations from {args.log_file}...")
    observations = load_observations_from_log(args.log_file, args.num_obs)
    
    if len(observations) < args.num_obs:
        print(f"Warning: Only found {len(observations)} observations, requested {args.num_obs}")
    
    if len(observations) == 0:
        print("Error: No observations found in log file")
        sys.exit(1)
    
    # Initialize ROS2
    rclpy.init()
    tester = InferenceTester(args.onnx_model)
    
    # Wait for service to be available
    print("Waiting for motion_controller service...")
    if not tester.inference_client.wait_for_service(timeout_sec=10.0):
        print("Error: motion_controller service not available")
        sys.exit(1)
    
    print(f"\nTesting {len(observations)} observations...")
    print("=" * 60)
    
    passed_count = 0
    for i, obs in enumerate(observations):
        print(f"\nObservation {i+1}/{len(observations)} (dim={len(obs)}):")
        
        # Python inference
        python_action = tester.run_python_inference(obs)
        
        # Process Python action to hardware commands
        python_hardware_commands = tester.process_python_action_to_hardware(python_action)
        
        # C++ inference
        cpp_action, cpp_hardware_commands, error = tester.run_cpp_inference(obs)
        if error:
            print(f"  C++ inference error: {error}")
            continue
        
        # Compare (pass observation and hardware commands for output)
        if tester.compare_actions(cpp_action, python_action, i+1, observation=obs,
                                  cpp_hardware_commands=cpp_hardware_commands,
                                  python_hardware_commands=python_hardware_commands):
            passed_count += 1
    
    print("\n" + "=" * 60)
    print(f"Summary: {passed_count}/{len(observations)} passed")
    
    if passed_count == len(observations):
        print("✓ All tests passed - ONNX inference logic is correct")
        sys.exit(0)
    else:
        print("✗ Some tests failed - investigate inference differences")
        sys.exit(1)


if __name__ == '__main__':
    main()

