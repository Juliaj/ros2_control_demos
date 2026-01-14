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
Test script to compare C++ and Python processing at each stage via debug topics.

Subscribes to debug topics published by motion_controller and compares with
Python equivalent processing at each stage:
1. Formatted observation
2. Raw action (ONNX inference)
3. Processed action
4. Blended action
5. Rate-limited action
"""

import argparse
import re
import sys
import time
import numpy as np
from pathlib import Path
from collections import deque
from typing import Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import Trigger
    from example_18_motion_controller_msgs.msg import VelocityCommandWithHead
except ImportError:
    print("Error: ROS2 packages required.")
    sys.exit(1)

# Try to import MuJoCo reset service
try:
    from mujoco_ros2_control.srv import ResetPhysicsState
    MUJOCO_RESET_AVAILABLE = True
except ImportError:
    MUJOCO_RESET_AVAILABLE = False
    print("Warning: MuJoCo reset service not available. Physics reset will be skipped.")

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


class DebugTopicTester(Node):
    """Test comparison using debug topics from motion controller."""

    def __init__(self, onnx_model_path, num_comparisons=5, test_observations=None):
        super().__init__('debug_topic_tester')
        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(onnx_model_path, awd=True)
        self.num_comparisons = num_comparisons
        self.test_observations = test_observations  # List of observations to test
        self.current_observation_idx = 0
        
        # Processing parameters (matching motion_controller config)
        self.action_scale = 0.25
        self.max_motor_velocity = 5.24  # rad/s
        self.control_period = 0.02  # 50Hz
        self.blend_in_steps = 200
        self.onnx_active_steps = 200  # Assume fully blended
        
        # Default joint positions
        self.default_joint_positions = np.array([
            0.002, 0.053, -0.63, 1.368, -0.784,  # left leg
            0.0, 0.0, 0.0, 0.0,  # head
            -0.003, -0.065, 0.635, 1.379, -0.796  # right leg
        ], dtype=np.float64)
        
        # Previous motor targets (initialize to default positions)
        self.prev_motor_targets = self.default_joint_positions.copy()
        self.prev_motor_targets_initialized = True
        
        # Subscribe to all debug topics
        self.sub_formatted_obs = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/formatted_observation',
            self.formatted_obs_callback, 10)
        self.sub_raw_action = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/raw_action',
            self.raw_action_callback, 10)
        self.sub_processed_action = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/processed_action',
            self.processed_action_callback, 10)
        self.sub_blended_action = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/blended_action',
            self.blended_action_callback, 10)
        self.sub_prev_motor_targets = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/prev_motor_targets',
            self.prev_motor_targets_callback, 10)
        self.sub_rate_limited_action = self.create_subscription(
            Float64MultiArray, '/motion_controller/debug/rate_limited_action',
            self.rate_limited_action_callback, 10)
        
        # Store received values (using deque for synchronization)
        self.received_formatted_obs = None
        self.received_raw_action = None
        self.received_processed_action = None
        self.received_blended_action = None
        self.received_prev_motor_targets = None
        self.received_rate_limited_action = None
        
        # Track if we have a complete set for comparison
        self.comparison_count = 0
        self.comparison_results = []
        
        # Reset service clients
        self.reset_client = self.create_client(Trigger, '/motion_controller/reset_state')
        
        # MuJoCo physics reset client (if available)
        if MUJOCO_RESET_AVAILABLE:
            self.mujoco_reset_client = self.create_client(
                ResetPhysicsState, '/mujoco_node/reset_physics_state')
        else:
            self.mujoco_reset_client = None
        
        # Velocity command publisher
        self.velocity_cmd_pub = self.create_publisher(
            VelocityCommandWithHead, '/motion_controller/cmd_vel', 10)
        
        # Flag to control when to capture next cycle
        self.waiting_for_cycle = False
        
        self.get_logger().info("Debug topic tester initialized. Waiting for debug topics...")

    def formatted_obs_callback(self, msg):
        """Callback for formatted observation."""
        self.received_formatted_obs = np.array(msg.data, dtype=np.float32)
        self.try_compare()

    def raw_action_callback(self, msg):
        """Callback for raw action."""
        self.received_raw_action = np.array(msg.data, dtype=np.float64)
        self.try_compare()

    def processed_action_callback(self, msg):
        """Callback for processed action."""
        self.received_processed_action = np.array(msg.data, dtype=np.float64)
        self.try_compare()

    def blended_action_callback(self, msg):
        """Callback for blended action."""
        self.received_blended_action = np.array(msg.data, dtype=np.float64)
        self.try_compare()

    def prev_motor_targets_callback(self, msg):
        """Callback for previous motor targets."""
        self.received_prev_motor_targets = np.array(msg.data, dtype=np.float64)
        self.try_compare()

    def rate_limited_action_callback(self, msg):
        """Callback for rate-limited action."""
        self.received_rate_limited_action = np.array(msg.data, dtype=np.float64)
        self.try_compare()

    def try_compare(self):
        """Try to perform comparison if we have all values."""
        # Check if we have all required values
        if (self.received_formatted_obs is not None and
            self.received_raw_action is not None and
            self.received_processed_action is not None and
            self.received_blended_action is not None and
            self.received_prev_motor_targets is not None and
            self.received_rate_limited_action is not None):
            
            # Only compare if we're waiting for a cycle and haven't exceeded the limit
            if (self.waiting_for_cycle and 
                self.comparison_count < self.num_comparisons):
                self.perform_comparison()
                self.waiting_for_cycle = False
                # Clear values for next comparison
                self.received_formatted_obs = None
                self.received_raw_action = None
                self.received_processed_action = None
                self.received_blended_action = None
                self.received_prev_motor_targets = None
                self.received_rate_limited_action = None
                
                # If using observations, prepare next one
                if self.test_observations and self.current_observation_idx < len(self.test_observations):
                    self.prepare_next_observation()

    def perform_comparison(self):
        """Perform comparison at each stage."""
        self.comparison_count += 1
        print(f"\n{'='*60}")
        print(f"Comparison {self.comparison_count}/{self.num_comparisons}")
        print(f"{'='*60}")
        
        # Stage 1: Formatted observation (no comparison, this is input)
        formatted_obs = self.received_formatted_obs
        print(f"\nStage 1: Formatted Observation (input)")
        print(f"  Dimension: {len(formatted_obs)}")
        print(f"  First 5, last 5: {formatted_obs[:5].tolist()}...{formatted_obs[-5:].tolist()}")
        
        # Stage 2: Raw action (ONNX inference)
        cpp_raw_action = self.received_raw_action
        python_raw_action = self.policy.infer(formatted_obs)
        raw_action_match = self.compare_arrays(
            cpp_raw_action, python_raw_action, "Raw Action (ONNX inference)")
        
        # Stage 3: Processed action
        cpp_processed_action = self.received_processed_action
        python_processed_action = self.default_joint_positions + python_raw_action * self.action_scale
        processed_action_match = self.compare_arrays(
            cpp_processed_action, python_processed_action, "Processed Action")
        
        # Stage 4: Blended action
        cpp_blended_action = self.received_blended_action
        blend_factor = min(1.0, self.onnx_active_steps / self.blend_in_steps)
        python_blended_action = (
            (1.0 - blend_factor) * self.default_joint_positions +
            blend_factor * python_processed_action
        )
        # Apply reference motion blending if enabled (assume disabled for now)
        blended_action_match = self.compare_arrays(
            cpp_blended_action, python_blended_action, "Blended Action")
        
        # Stage 5: Rate-limited action
        cpp_rate_limited = self.received_rate_limited_action
        # Update prev_motor_targets from received value
        if self.received_prev_motor_targets is not None:
            self.prev_motor_targets = self.received_prev_motor_targets.copy()
            self.prev_motor_targets_initialized = True
        
        # Apply rate limiting
        if self.prev_motor_targets_initialized:
            max_change = self.max_motor_velocity * self.control_period
            python_rate_limited = np.clip(
                python_blended_action,
                self.prev_motor_targets - max_change,
                self.prev_motor_targets + max_change
            )
            # Update for next iteration
            self.prev_motor_targets = python_rate_limited.copy()
        else:
            python_rate_limited = python_blended_action
        
        rate_limited_match = self.compare_arrays(
            cpp_rate_limited, python_rate_limited, "Rate-Limited Action (FINAL)")
        
        # Store results
        result = {
            'stage': self.comparison_count,
            'raw_action': raw_action_match,
            'processed_action': processed_action_match,
            'blended_action': blended_action_match,
            'rate_limited': rate_limited_match,
        }
        self.comparison_results.append(result)
        
        # Print summary
        print(f"\n{'='*60}")
        print("Stage Comparison Summary:")
        print(f"  Raw Action: {'✓ PASS' if raw_action_match else '✗ FAIL'}")
        print(f"  Processed Action: {'✓ PASS' if processed_action_match else '✗ FAIL'}")
        print(f"  Blended Action: {'✓ PASS' if blended_action_match else '✗ FAIL'}")
        print(f"  Rate-Limited Action: {'✓ PASS' if rate_limited_match else '✗ FAIL'}")
        print(f"{'='*60}")

    def compare_arrays(self, cpp_array, python_array, stage_name, 
                      abs_tol=1e-5, rel_tol=1e-4):
        """Compare two arrays and report differences."""
        if cpp_array.shape != python_array.shape:
            print(f"\n{stage_name}:")
            print(f"  ✗ Shape mismatch - C++: {cpp_array.shape}, Python: {python_array.shape}")
            return False
        
        abs_diff = np.abs(cpp_array - python_array)
        max_abs_diff = np.max(abs_diff)
        mean_abs_diff = np.mean(abs_diff)
        
        # Relative difference (avoid division by zero)
        with np.errstate(divide='ignore', invalid='ignore'):
            rel_diff = np.abs((cpp_array - python_array) / (python_array + 1e-10))
        max_rel_diff = np.max(rel_diff)
        
        # Check if within tolerance
        abs_ok = max_abs_diff < abs_tol
        rel_ok = max_rel_diff < rel_tol
        match = abs_ok or rel_ok
        
        print(f"\n{stage_name}:")
        print(f"  C++ (first 5, last 5): {cpp_array[:5].tolist()}...{cpp_array[-5:].tolist()}")
        print(f"  Python (first 5, last 5): {python_array[:5].tolist()}...{python_array[-5:].tolist()}")
        print(f"  Max absolute diff: {max_abs_diff:.6e}")
        print(f"  Mean absolute diff: {mean_abs_diff:.6e}")
        print(f"  Max relative diff: {max_rel_diff:.6e}")
        print(f"  Status: {'✓ PASS' if match else '✗ FAIL'}")
        
        if not match:
            # Find worst differences
            worst_indices = np.argsort(abs_diff)[-5:][::-1]
            print(f"  Worst differences at indices: {worst_indices.tolist()}")
            for idx in worst_indices[:3]:
                print(f"    [{idx}]: C++={cpp_array[idx]:.6f}, Python={python_array[idx]:.6f}, "
                      f"diff={abs_diff[idx]:.6e}")
        
        return match

    def reset_state(self, reset_physics=False, qpos=None, qvel=None, ctrl=None):
        """Reset controller state and optionally MuJoCo physics state."""
        # Reset controller state
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Controller reset service not available")
            return False
        
        request = Trigger.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info("Controller state reset successful")
                # Reset Python state too
                self.prev_motor_targets = self.default_joint_positions.copy()
                self.prev_motor_targets_initialized = True
            else:
                self.get_logger().warn(f"Controller reset failed: {response.message}")
                return False
        else:
            return False
        
        # Reset MuJoCo physics state if requested
        if reset_physics and self.mujoco_reset_client:
            if not self.mujoco_reset_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("MuJoCo reset service not available")
                return False
            
            reset_req = ResetPhysicsState.Request()
            if qpos is not None:
                reset_req.qpos.data = qpos.tolist() if isinstance(qpos, np.ndarray) else qpos
            if qvel is not None:
                reset_req.qvel.data = qvel.tolist() if isinstance(qvel, np.ndarray) else qvel
            if ctrl is not None:
                reset_req.ctrl.data = ctrl.tolist() if isinstance(ctrl, np.ndarray) else ctrl
            
            # If no values provided, use defaults (zeros)
            if qpos is None:
                # Need to get model dimensions - for now, skip if not provided
                self.get_logger().warn("MuJoCo reset requires qpos/qvel/ctrl arrays")
                return True
            
            future = self.mujoco_reset_client.call_async(reset_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info("MuJoCo physics state reset successful")
                else:
                    self.get_logger().warn(f"MuJoCo reset failed: {response.message}")
                    return False
        
        return True

    def send_velocity_command(self, lin_x=0.0, lin_y=0.0, ang_z=0.0, head_commands=None):
        """Send velocity command to controller."""
        msg = VelocityCommandWithHead()
        msg.base_velocity.linear.x = float(lin_x)
        msg.base_velocity.linear.y = float(lin_y)
        msg.base_velocity.angular.z = float(ang_z)
        msg.head_commands = head_commands if head_commands else [0.0, 0.0, 0.0, 0.0]
        
        # Publish multiple times to ensure it's received
        for _ in range(5):
            self.velocity_cmd_pub.publish(msg)
            time.sleep(0.01)
        
        self.get_logger().info(
            f"Sent velocity command: lin_x={lin_x:.3f}, lin_y={lin_y:.3f}, ang_z={ang_z:.3f}")

    def extract_velocity_from_observation(self, observation):
        """Extract velocity command from observation vector.
        
        Observation format (101 dim):
        - Indices 0-2: Gyro (3D)
        - Indices 3-5: Accelerometer (3D)
        - Indices 6-12: Commands (7D: 3 base + 4 head)
          - Index 6: lin_x
          - Index 7: lin_y
          - Index 8: ang_z
          - Indices 9-12: head commands
        """
        if len(observation) < 13:
            return 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0]
        
        lin_x = float(observation[6])
        lin_y = float(observation[7])
        ang_z = float(observation[8])
        head_commands = [float(observation[i]) for i in range(9, 13)] if len(observation) >= 13 else [0.0, 0.0, 0.0, 0.0]
        
        return lin_x, lin_y, ang_z, head_commands

    def prepare_next_observation(self):
        """Prepare for next observation test."""
        if not self.test_observations or self.current_observation_idx >= len(self.test_observations):
            return
        
        obs = self.test_observations[self.current_observation_idx]
        obs_num = self.current_observation_idx + 1
        self.current_observation_idx += 1
        
        # Reset state
        self.get_logger().info(f"Preparing observation {obs_num}/{len(self.test_observations)}")
        self.reset_state(reset_physics=False)  # Don't reset physics for now (needs qpos/qvel)
        
        # Extract and send velocity command
        lin_x, lin_y, ang_z, head_commands = self.extract_velocity_from_observation(obs)
        self.send_velocity_command(lin_x, lin_y, ang_z, head_commands)
        
        # Wait a bit for command to be processed
        time.sleep(0.1)
        
        # Mark that we're waiting for next cycle
        self.waiting_for_cycle = True
        
        # Clear any stale received values
        self.received_formatted_obs = None
        self.received_raw_action = None
        self.received_processed_action = None
        self.received_blended_action = None
        self.received_prev_motor_targets = None
        self.received_rate_limited_action = None

    def print_summary(self):
        """Print final summary."""
        print(f"\n{'='*60}")
        print("FINAL SUMMARY")
        print(f"{'='*60}")
        
        if not self.comparison_results:
            print("No comparisons performed.")
            return
        
        total = len(self.comparison_results)
        raw_passed = sum(1 for r in self.comparison_results if r['raw_action'])
        processed_passed = sum(1 for r in self.comparison_results if r['processed_action'])
        blended_passed = sum(1 for r in self.comparison_results if r['blended_action'])
        rate_limited_passed = sum(1 for r in self.comparison_results if r['rate_limited'])
        
        print(f"\nTotal comparisons: {total}")
        print(f"Raw Action: {raw_passed}/{total} passed")
        print(f"Processed Action: {processed_passed}/{total} passed")
        print(f"Blended Action: {blended_passed}/{total} passed")
        print(f"Rate-Limited Action: {rate_limited_passed}/{total} passed")
        
        # Identify problematic stages
        print(f"\nStage Analysis:")
        if raw_passed < total:
            print(f"  ⚠ Raw Action stage has {total - raw_passed} failures (ONNX inference issue)")
        if processed_passed < total:
            print(f"  ⚠ Processed Action stage has {total - processed_passed} failures (action processor issue)")
        if blended_passed < total:
            print(f"  ⚠ Blended Action stage has {total - blended_passed} failures (blending logic issue)")
        if rate_limited_passed < total:
            print(f"  ⚠ Rate-Limited Action stage has {total - rate_limited_passed} failures (rate limiting issue)")
        
        if all([raw_passed == total, processed_passed == total, 
                blended_passed == total, rate_limited_passed == total]):
            print(f"\n✓ All stages passed - C++ and Python implementations match!")
        else:
            print(f"\n✗ Some stages have differences - investigate above")


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
    """Load observations from log file."""
    observations = []
    with open(log_file, 'r') as f:
        for line in f:
            if '[RAW_OBS]' in line:
                obs = parse_observation_from_log(line)
                if obs is not None:
                    observations.append(obs)
                    if len(observations) >= num_obs:
                        break
    return observations


def main():
    parser = argparse.ArgumentParser(
        description='Test C++ vs Python processing using debug topics')
    parser.add_argument(
        '--onnx-model', type=str, required=True,
        help='Path to ONNX model file')
    parser.add_argument(
        '--num-comparisons', type=int, default=5,
        help='Number of comparisons to perform (default: 5)')
    parser.add_argument(
        '--reset-before', action='store_true',
        help='Reset controller state before starting')
    parser.add_argument(
        '--log-file', type=str, default=None,
        help='Path to log file with observations to test (optional)')
    parser.add_argument(
        '--velocity', type=float, nargs=3, default=None, metavar=('LIN_X', 'LIN_Y', 'ANG_Z'),
        help='Velocity command to send (lin_x lin_y ang_z). If not provided and no log file, waits passively.')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        # Load observations if log file provided
        test_observations = None
        if args.log_file:
            print(f"Loading observations from {args.log_file}...")
            test_observations = load_observations_from_log(args.log_file, args.num_comparisons)
            print(f"Loaded {len(test_observations)} observations")
            if len(test_observations) == 0:
                print("Error: No observations found in log file")
                return
        
        tester = DebugTopicTester(args.onnx_model, args.num_comparisons, test_observations)
        
        if args.reset_before:
            print("Resetting controller state...")
            tester.reset_state()
            time.sleep(1.0)  # Wait for reset to complete
        
        # If using observations, start the workflow
        if test_observations:
            print(f"\nTesting {len(test_observations)} observations from log file...")
            print("Workflow: Reset -> Send velocity -> Capture cycle -> Compare -> Repeat\n")
            tester.prepare_next_observation()
        elif args.velocity:
            # Send specified velocity command
            print(f"\nSending velocity command: lin_x={args.velocity[0]}, lin_y={args.velocity[1]}, ang_z={args.velocity[2]}")
            tester.send_velocity_command(args.velocity[0], args.velocity[1], args.velocity[2])
            tester.waiting_for_cycle = True
            time.sleep(0.1)
        else:
            # Passive mode - wait for whatever comes
            print(f"\nWaiting for {args.num_comparisons} complete update cycles...")
            print("Make sure the controller is running and debug publishing is enabled.")
            print("The controller should be actively processing updates.\n")
            tester.waiting_for_cycle = True
        
        # Spin until we have enough comparisons
        timeout = 120.0  # 120 second timeout
        start_time = time.time()
        while tester.comparison_count < args.num_comparisons:
            rclpy.spin_once(tester, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                print(f"\nTimeout: Only got {tester.comparison_count}/{args.num_comparisons} comparisons")
                break
        
        tester.print_summary()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

