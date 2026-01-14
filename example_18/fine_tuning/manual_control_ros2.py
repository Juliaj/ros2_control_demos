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
Manual control script for ROS2/MuJoCo to collect training data.

This script:
1. Runs in ROS2 environment (uses mujoco_ros2_control)
2. Uses ONNX model to generate actions based on observations
3. Allows manual keyboard/joystick control of velocity commands
4. Publishes joint commands to control the robot
5. Collects observations and actions in ROS2 format

Usage:
    # require data_collection.launch.py to be running
    ros2 launch ros2_control_demo_example_18 data_collection.launch.py
    python3 manual_control_ros2.py --onnx-model path/to/model.onnx --output data.h5
"""

import argparse
import sys
from pathlib import Path
import time
import numpy as np
import h5py
import threading
import termios
import tty
import os
import fcntl

try:
    import rclpy
    from rclpy.node import Node
    from control_msgs.msg import Float64Values
    from example_18_motion_controller_msgs.msg import VelocityCommandWithHead
    from std_msgs.msg import Float64MultiArray
except ImportError:
    print("Error: ROS2 packages required.")
    print("Make sure ROS2 is sourced: source /opt/ros/jazzy/setup.bash")
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


class ManualControlROS2(Node):
    """Manual control data collector in ROS2 environment."""

    def __init__(self, onnx_model_path, num_joints=14):
        """Initialize manual control collector.

        Parameters
        ----------
        onnx_model_path : str
            Path to ONNX model file
        num_joints : int
            Number of joints (default: 14 for open_duck_mini_v2)
        """
        super().__init__('manual_control_ros2')
        self.num_joints = num_joints

        # Load ONNX model
        self.policy = OnnxInfer(onnx_model_path, awd=True)
        self.get_logger().info(f"Loaded ONNX model: {onnx_model_path}")

        # Control parameters
        # Original value from MuJoCo training: 0.25
        # Slightly increased to allow larger leg movements for proper stepping
        self.action_scale = 0.3
        self.dof_vel_scale = 0.05
        # Original value from MuJoCo training: 5.24 rad/s
        # Moderately increased to allow faster joint movements for quicker gait execution
        self.max_motor_velocity = 8.0  # rad/s
        self.control_period = 0.02  # 50Hz
        # Gyro deadband to filter out small drift/noise that causes unwanted turning
        self.gyro_deadband = 0.15  # rad/s - values below this are set to zero
        
        # Home keyframe joint positions from scene.xml
        # Format: [left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle,
        #          neck_pitch, head_pitch, head_yaw, head_roll,
        #          right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle]
        self.home_joint_positions = np.array([
            0.002,    # left_hip_yaw
            0.053,    # left_hip_roll
            -0.63,    # left_hip_pitch
            1.368,    # left_knee
            -0.784, # left_ankle
            0.0,      # neck_pitch
            0.0,      # head_pitch
            0.0,      # head_yaw
            0.0,      # head_roll
            -0.003,   # right_hip_yaw
            -0.065,   # right_hip_roll
            0.635,    # right_hip_pitch
            1.379,    # right_knee
            -0.796,   # right_ankle
        ])

        # State tracking
        self.default_joint_positions = None
        self.default_joint_positions_set = False
        self.last_action = np.zeros(num_joints)
        self.last_last_action = np.zeros(num_joints)
        self.last_last_last_action = np.zeros(num_joints)
        self.motor_targets = np.zeros(num_joints)
        self.prev_motor_targets = None
        self.prev_motor_targets_initialized = False
        self.use_real_contacts = False
        self.left_contact_sensor = 0.0
        self.right_contact_sensor = 0.0

        # Stabilization and blend-in
        self.stabilization_steps = 0
        self.stabilization_delay = 50  # Wait 50 steps (~1 second at 50Hz) before applying ONNX actions
        self.onnx_blend_in_steps = 200  # Gradually blend in ONNX actions over 200 steps (~4 seconds)
        self.onnx_active_steps = 0  # Track steps since ONNX started
        self.use_onnx = False  # Start with ONNX disabled by default (user can enable with 'O')
        
        # Imitation phase tracking (for gait cycle)
        self.imitation_i = 0.0  # Phase counter
        self.phase_period = 50.0  # Steps per gait cycle (from reference motion)
        self.phase_frequency_factor = 1.0  # Can be adjusted for speed

        # Velocity command (manually controlled)
        self.current_velocity_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity_step = 0.01  # Step size for velocity adjustments

        # Data collection
        self.observations = []
        self.actions = []
        self.velocity_commands = []
        self.base_linear_velocities = []  # Base linear velocity from velocimeter sensor
        self.timestamps = []
        self.collecting = False
        
        # Additional data for state injection
        self.interface_data_list = []  # Raw Float64Values messages
        self.controller_state_list = []  # Controller state at each step
        
        # Control pause/resume
        self.control_paused = False

        # Subscribers
        self.interface_sub = self.create_subscription(
            Float64Values,
            '/state_interfaces_broadcaster/values',
            self.interface_callback,
            10
        )

        # Publishers
        self.velocity_cmd_pub = self.create_publisher(
            VelocityCommandWithHead,
            '/motion_controller/cmd_vel',
            10
        )
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )
        
        # Wait for subscribers to connect (controller may take time to start)
        max_wait_time = 3.0  # Wait up to 3 seconds
        wait_interval = 0.1
        waited = 0.0
        subscriber_count = 0
        while waited < max_wait_time:
            subscriber_count = self.joint_cmd_pub.get_subscription_count()
            if subscriber_count > 0:
                break
            time.sleep(wait_interval)
            waited += wait_interval
        
        if subscriber_count == 0:
            self.get_logger().warn(
                "WARNING: No subscribers to /forward_command_controller/commands after {:.1f}s. "
                "Make sure forward_command_controller is running in the launch file!".format(waited)
            )
        else:
            self.get_logger().info(f"forward_command_controller connected ({subscriber_count} subscriber(s)) after {waited:.1f}s")

        # Keyboard input setup
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info("Manual control ROS2 collector initialized")
        self.get_logger().info("  Controls:")
        self.get_logger().info("    Arrow keys or WASD: Adjust velocity")
        self.get_logger().info("      Up/W: Forward, Down/S: Backward")
        self.get_logger().info("      Left/A: Left, Right/D: Right")
        self.get_logger().info("    Q/E: Turn left/right")
        self.get_logger().info("    Space: Toggle data collection")
        self.get_logger().info("    P: Pause/resume control (allows MuJoCo reset)")
        self.get_logger().info("    R: Reset robot to home/standing position")
        self.get_logger().info("    O: Toggle ONNX model (currently OFF - press O to enable)")
        self.get_logger().info("    Z: Zero velocity")
        self.get_logger().info("    Ctrl+C: Exit and save")

    def keyboard_listener(self):
        """Listen for keyboard input in background thread."""
        # Set stdin to non-blocking
        flags = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        
        while rclpy.ok():
            try:
                # Read first byte
                key = sys.stdin.read(1)
                if not key:
                    time.sleep(0.01)
                    continue
                
                # Check if it's escape sequence (arrow keys start with ESC)
                if key == '\x1b':
                    # Try to read the next two bytes immediately
                    # In cbreak mode, arrow keys send ESC [ A/B/C/D all at once
                    try:
                        key2 = sys.stdin.read(1)
                        if key2 == '[':
                            key3 = sys.stdin.read(1)
                            if key3:
                                # Full arrow key sequence: ESC [ A/B/C/D
                                self.handle_key(key + key2 + key3)
                            else:
                                # Incomplete sequence, wait a bit and try again
                                time.sleep(0.01)
                                key3 = sys.stdin.read(1)
                                if key3:
                                    self.handle_key(key + key2 + key3)
                        else:
                            # ESC followed by something else, ignore
                            pass
                    except (IOError, OSError):
                        # No more bytes available, just ESC was pressed
                        pass
                else:
                    # Regular single character key
                    self.handle_key(key)
            except (IOError, OSError):
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().warn(f"Keyboard input error: {e}")
                time.sleep(0.1)

    def handle_key(self, key):
        """Handle keyboard input."""
        # Debug: log raw key input for arrow keys
        if len(key) == 3 and key[0] == '\x1b' and key[1] == '[':
            self.get_logger().debug(f"Arrow key detected: {repr(key)} -> {key[2]}")
        
        # Handle arrow keys (ESC [ A/B/C/D)
        if len(key) == 3 and key[0] == '\x1b' and key[1] == '[':
            if key[2] == 'A':  # Up arrow
                self.current_velocity_command[0] = min(0.15, self.current_velocity_command[0] + self.velocity_step)
                self.get_logger().info(f"Up arrow pressed, lin_x={self.current_velocity_command[0]:.3f}")
                self.publish_velocity_command()
            elif key[2] == 'B':  # Down arrow
                self.current_velocity_command[0] = max(-0.15, self.current_velocity_command[0] - self.velocity_step)
                self.get_logger().info(f"Down arrow pressed, lin_x={self.current_velocity_command[0]:.3f}")
                self.publish_velocity_command()
            elif key[2] == 'C':  # Right arrow
                self.current_velocity_command[1] = min(0.2, self.current_velocity_command[1] + self.velocity_step)
                self.get_logger().info(f"Right arrow pressed, lin_y={self.current_velocity_command[1]:.3f}")
                self.publish_velocity_command()
            elif key[2] == 'D':  # Left arrow
                self.current_velocity_command[1] = max(-0.2, self.current_velocity_command[1] - self.velocity_step)
                self.get_logger().info(f"Left arrow pressed, lin_y={self.current_velocity_command[1]:.3f}")
                self.publish_velocity_command()
        # Handle single character keys
        elif len(key) == 1:
            if key == 'q' or key == 'Q':
                self.current_velocity_command[2] = min(1.0, self.current_velocity_command[2] + 0.05)
                self.publish_velocity_command()
            elif key == 'e' or key == 'E':
                self.current_velocity_command[2] = max(-1.0, self.current_velocity_command[2] - 0.05)
                self.publish_velocity_command()
            elif key == ' ':
                self.collecting = not self.collecting
                self.get_logger().info(f"Data collection: {'ON' if self.collecting else 'OFF'}")
            elif key == 'z' or key == 'Z':
                self.current_velocity_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.publish_velocity_command()
                self.get_logger().info("Velocity commands reset to zero")
            elif key == 'o' or key == 'O':
                self.use_onnx = not self.use_onnx
                if self.use_onnx:
                    self.get_logger().info("ONNX model ENABLED")
                else:
                    self.get_logger().info("ONNX model DISABLED - using default positions only")
            elif key == 'p' or key == 'P':
                self.control_paused = not self.control_paused
                if self.control_paused:
                    self.get_logger().info("Control PAUSED - You can now reset in MuJoCo (press R)")
                else:
                    self.get_logger().info("Control RESUMED")
            elif key == 'r' or key == 'R':
                # Reset robot to home position
                self.reset_to_home()
            elif key == 'w' or key == 'W':
                # Forward
                self.current_velocity_command[0] = min(0.15, self.current_velocity_command[0] + self.velocity_step)
                self.publish_velocity_command()
            elif key == 's' or key == 'S':
                # Backward
                self.current_velocity_command[0] = max(-0.15, self.current_velocity_command[0] - self.velocity_step)
                self.publish_velocity_command()
            elif key == 'a' or key == 'A':
                # Left
                self.current_velocity_command[1] = max(-0.2, self.current_velocity_command[1] - self.velocity_step)
                self.publish_velocity_command()
            elif key == 'd' or key == 'D':
                # Right
                self.current_velocity_command[1] = min(0.2, self.current_velocity_command[1] + self.velocity_step)
                self.publish_velocity_command()

    def publish_velocity_command(self):
        """Publish current velocity command."""
        msg = VelocityCommandWithHead()
        msg.base_velocity.linear.x = float(self.current_velocity_command[0])
        msg.base_velocity.linear.y = float(self.current_velocity_command[1])
        msg.base_velocity.angular.z = float(self.current_velocity_command[2])
        msg.head_commands = [0.0, 0.0, 0.0, 0.0]
        self.velocity_cmd_pub.publish(msg)
        self.get_logger().info(
            f"Velocity: lin_x={self.current_velocity_command[0]:.3f}, "
            f"lin_y={self.current_velocity_command[1]:.3f}, "
            f"ang_z={self.current_velocity_command[2]:.3f}"
        )

    def reset_to_home(self):
        """Reset robot to home keyframe position."""
        self.get_logger().info("Resetting robot to home position...")
        
        # Publish home joint positions multiple times to ensure it's received
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.home_joint_positions.tolist()
        
        for i in range(10):
            self.joint_cmd_pub.publish(cmd_msg)
            time.sleep(0.01)  # Small delay between publishes
        
        # Reset default positions and state
        self.default_joint_positions_set = False
        self.motor_targets = self.home_joint_positions.copy()
        self.prev_motor_targets = self.home_joint_positions.copy()
        self.prev_motor_targets_initialized = True
        self.stabilization_steps = 0
        self.onnx_active_steps = 0
        self.imitation_i = 0.0  # Reset phase
        self.current_velocity_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_action = np.zeros(self.num_joints)
        self.last_last_action = np.zeros(self.num_joints)
        self.last_last_last_action = np.zeros(self.num_joints)
        
        self.get_logger().info("Robot reset to home position. Default positions will be re-initialized on next callback.")

    def interface_callback(self, msg):
        """Process interface data and generate control."""
        # Extract interface data
        # Expected: 10 IMU + 2*num_joints (positions + velocities) + 2 contacts + 3 velocimeter
        expected_size_with_velocimeter = 10 + 2 * self.num_joints + 2 + 3  # 43 minimum for 14 joints
        expected_size_min = 10 + 2 * self.num_joints  # Minimum without velocimeter
        if len(msg.values) < expected_size_min:
            return

        # Extract IMU data
        gyro = np.array([msg.values[4], msg.values[5], msg.values[6]])
        # Apply deadband to filter out small drift/noise
        gyro = np.where(np.abs(gyro) < self.gyro_deadband, 0.0, gyro)
        accelero = np.array([msg.values[7], msg.values[8], msg.values[9]])

        # Extract joint positions and velocities
        joint_pos_start = 10
        joint_vel_start = 10 + self.num_joints
        joint_positions = np.array(msg.values[joint_pos_start:joint_pos_start + self.num_joints])
        joint_velocities = np.array(msg.values[joint_vel_start:joint_vel_start + self.num_joints])
        
        # Extract contact sensors (indices 10 + 2*num_joints)
        # Note: Model was trained with phase-based contacts, so we use phase-based by default for stability
        # Real contact sensors are extracted but not used unless explicitly enabled
        contact_start = 10 + 2 * self.num_joints
        if len(msg.values) >= contact_start + 2:
            self.left_contact_sensor = float(msg.values[contact_start])
            self.right_contact_sensor = float(msg.values[contact_start + 1])
        else:
            self.left_contact_sensor = 0.0
            self.right_contact_sensor = 0.0
        
        # Always use phase-based contacts (model was trained with this)
        # Set self.use_real_contact_sensors = True in __init__ to enable real sensors (experimental)
        if hasattr(self, 'use_real_contact_sensors') and self.use_real_contact_sensors:
            # Validate sensor values are in reasonable range
            if 0.0 <= self.left_contact_sensor <= 1.0 and 0.0 <= self.right_contact_sensor <= 1.0:
                self.use_real_contacts = True
            else:
                self.use_real_contacts = False
                if self.stabilization_steps == 1:  # Warn once
                    self.get_logger().warn(
                        f"Contact sensor values out of range: [{self.left_contact_sensor}, {self.right_contact_sensor}]. "
                        "Using phase-based estimation."
                    )
        else:
            self.use_real_contacts = False
        
        # Extract base linear velocity from velocimeter sensor (indices 40-42)
        # If velocimeter data is available, use it; otherwise use zeros
        if len(msg.values) >= expected_size_with_velocimeter:
            velocimeter_start = 40
            base_linear_velocity = np.array([
                msg.values[velocimeter_start],
                msg.values[velocimeter_start + 1],
                msg.values[velocimeter_start + 2]
            ])
        else:
            # Fallback: use zeros if velocimeter not available
            base_linear_velocity = np.array([0.0, 0.0, 0.0])
            if len(self.observations) == 0:  # Only warn once
                self.get_logger().warn(
                    f"Velocimeter data not available (got {len(msg.values)} values, expected {expected_size_with_velocimeter}). "
                    "Using zeros for base linear velocity."
                )
        
        # If control is paused, don't send any commands
        if self.control_paused:
            return

        # Initialize default joint positions on first callback
        if not self.default_joint_positions_set:
            self.default_joint_positions = joint_positions.copy()
            self.default_joint_positions_set = True
            self.motor_targets = joint_positions.copy()
            self.prev_motor_targets = joint_positions.copy()
            self.prev_motor_targets_initialized = True
            self.stabilization_steps = 0
            self.get_logger().info("Default joint positions initialized")
            self.get_logger().info(f"Stabilization: holding default positions for {self.stabilization_delay} steps (~{self.stabilization_delay * self.control_period:.1f}s)")

        # Increment stabilization counter
        self.stabilization_steps += 1

        # During stabilization, hold default positions
        model_action = None
        observation = None
        
        if self.stabilization_steps < self.stabilization_delay:
            # Hold default positions
            self.motor_targets = self.default_joint_positions.copy()
            if not self.prev_motor_targets_initialized:
                self.prev_motor_targets = self.default_joint_positions.copy()
                self.prev_motor_targets_initialized = True
            
            # Log stabilization progress
            if self.stabilization_steps % 50 == 0:
                self.get_logger().info(
                    f"Stabilization: {self.stabilization_steps}/{self.stabilization_delay} steps, "
                    f"holding default positions, vel_cmd = {self.current_velocity_command[:3]}"
                )
        else:
            # Log transition to ONNX mode
            if self.stabilization_steps == self.stabilization_delay:
                self.get_logger().info(
                    f"Stabilization complete! Starting ONNX control (use_onnx={self.use_onnx})"
                )
            # After stabilization
            if self.use_onnx:
                # Use ONNX model
                # Update imitation phase (gait cycle)
                self.imitation_i += 1.0 * self.phase_frequency_factor
                self.imitation_i = self.imitation_i % self.phase_period
                
                # Format observation
                observation = self.format_observation(gyro, accelero, joint_positions, joint_velocities, 
                                                       self.left_contact_sensor, self.right_contact_sensor)

                # Log raw observation vector (throttled, every 50 steps to match control logging)
                if self.stabilization_steps % 50 == 0:
                    obs_str = f"[RAW_OBS] step={self.stabilization_steps}, obs=["
                    obs_str += ",".join([f"{val:.3f}" for val in observation])
                    obs_str += "]"
                    self.get_logger().info(obs_str)

                # Get action from ONNX model
                model_action = self.policy.infer(observation)

                # Increment ONNX active steps for blend-in
                self.onnx_active_steps += 1

                # Debug: log action statistics occasionally
                if self.stabilization_steps % 50 == 0:
                    blend_factor = min(1.0, self.onnx_active_steps / self.onnx_blend_in_steps)
                    contact_info = f"contacts=[{self.left_contact_sensor:.3f}, {self.right_contact_sensor:.3f}]" if self.use_real_contacts else "contacts=phase_est"
                    self.get_logger().info(
                        f"ONNX active: step={self.stabilization_steps}, active_steps={self.onnx_active_steps}, "
                        f"blend_factor={blend_factor:.3f}, "
                        f"action_range=[{np.min(model_action):.3f}, {np.max(model_action):.3f}], "
                        f"vel_cmd={self.current_velocity_command[:3]}, "
                        f"gyro=[{gyro[0]:.3f}, {gyro[1]:.3f}, {gyro[2]:.3f}], "
                        f"base_vel=[{base_linear_velocity[0]:.3f}, {base_linear_velocity[1]:.3f}, {base_linear_velocity[2]:.3f}], "
                        f"{contact_info}"
                    )

                # Convert action to motor targets
                desired_motor_targets = self.default_joint_positions + model_action * self.action_scale

                # Gradually blend in ONNX actions to prevent sudden movements
                blend_factor = min(1.0, self.onnx_active_steps / self.onnx_blend_in_steps)
                
                # Blend between default positions and ONNX-generated targets
                blended_targets = (
                    (1.0 - blend_factor) * self.default_joint_positions +
                    blend_factor * desired_motor_targets
                )

                # Apply rate limiting
                if self.prev_motor_targets_initialized:
                    max_change = self.max_motor_velocity * self.control_period
                    self.motor_targets = np.clip(
                        blended_targets,
                        self.prev_motor_targets - max_change,
                        self.prev_motor_targets + max_change
                    )
                    self.prev_motor_targets = self.motor_targets.copy()
                else:
                    self.motor_targets = blended_targets.copy()
                    self.prev_motor_targets = blended_targets.copy()
                    self.prev_motor_targets_initialized = True
            else:
                # ONNX disabled - just hold default positions
                self.motor_targets = self.default_joint_positions.copy()
                if not self.prev_motor_targets_initialized:
                    self.prev_motor_targets = self.default_joint_positions.copy()
                    self.prev_motor_targets_initialized = True
                model_action = None
                observation = None

        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.motor_targets.tolist()
        self.joint_cmd_pub.publish(cmd_msg)
        
        # Debug logging (throttled)
        if self.stabilization_steps % 50 == 0:  # Log every 50 samples (~1 second at 50Hz)
            motor_diff = np.max(np.abs(self.motor_targets - self.default_joint_positions))
            onnx_status = "ON" if (self.use_onnx and self.stabilization_steps >= self.stabilization_delay) else "OFF"
            self.get_logger().info(
                f"Control: step={self.stabilization_steps}, ONNX={onnx_status}, "
                f"motor_diff={motor_diff:.4f} rad, vel_cmd={self.current_velocity_command[:3]}, "
                f"motor_targets[0]={self.motor_targets[0]:.4f}"
            )

        # Collect data if enabled (only after stabilization starts)
        if self.collecting and self.stabilization_steps >= self.stabilization_delay and model_action is not None:
            self.observations.append(observation)
            self.actions.append(model_action)
            self.velocity_commands.append(self.current_velocity_command.copy())
            self.base_linear_velocities.append(base_linear_velocity.copy())
            self.timestamps.append(time.time())
            
            # Collect interface data (raw Float64Values from msg)
            self.interface_data_list.append(list(msg.values))
            
            # Collect controller state (capture before updating action history)
            controller_state = {
                'last_action': self.last_action.copy(),
                'last_last_action': self.last_last_action.copy(),
                'last_last_last_action': self.last_last_last_action.copy(),
                'motor_targets': self.motor_targets.copy(),
                'imitation_i': float(self.imitation_i),
                'prev_motor_targets': self.prev_motor_targets.copy() if self.prev_motor_targets is not None else self.default_joint_positions.copy(),
                'onnx_active_steps': int(self.onnx_active_steps),
                'stabilization_steps': int(self.stabilization_steps)
            }
            self.controller_state_list.append(controller_state)
            
            # Update action history (after capturing state)
            self.last_last_last_action = self.last_last_action.copy()
            self.last_last_action = self.last_action.copy()
            self.last_action = model_action.copy()
        elif self.stabilization_steps >= self.stabilization_delay and model_action is not None:
            # Update action history even if not collecting (for observation formatting)
            self.last_last_last_action = self.last_last_action.copy()
            self.last_last_action = self.last_action.copy()
            self.last_action = model_action.copy()

    def format_observation(self, gyro, accelero, joint_positions, joint_velocities, 
                           left_contact_sensor, right_contact_sensor):
        """Format observation exactly as observation_formatter.cpp does."""
        obs = []

        # 1. Gyro (3D)
        obs.extend(gyro.tolist())

        # 2. Accelerometer (3D) with x-bias +1.3
        obs.append(accelero[0] + 1.3)
        obs.append(accelero[1])
        obs.append(accelero[2])

        # 3. Commands (7D)
        obs.extend(self.current_velocity_command)

        # 4. Joint positions relative to default (N)
        rel_positions = joint_positions - self.default_joint_positions
        obs.extend(rel_positions.tolist())

        # 5. Joint velocities scaled by 0.05 (N)
        obs.extend((joint_velocities * self.dof_vel_scale).tolist())

        # 6-8. Action history (3*N)
        obs.extend(self.last_action.tolist())
        obs.extend(self.last_last_action.tolist())
        obs.extend(self.last_last_last_action.tolist())

        # 9. Motor targets (N)
        obs.extend(self.motor_targets.tolist())

        # Calculate phase for imitation phase encoding (always needed)
        phase_theta = (self.imitation_i / self.phase_period) * 2.0 * np.pi
        sin_phase = np.sin(phase_theta)
        cos_phase = np.cos(phase_theta)

        # 10. Feet contacts (2D) - use real sensors if available, otherwise estimate from phase
        if self.use_real_contacts:
            # Use real contact sensor values (thresholded to 0/1)
            left_contact = 1.0 if left_contact_sensor > 0.5 else 0.0
            right_contact = 1.0 if right_contact_sensor > 0.5 else 0.0
        else:
            # Fallback to phase-based estimation
            left_contact = 1.0 if sin_phase > 0.0 else 0.0
            right_contact = 1.0 if sin_phase < 0.0 else 0.0
        obs.extend([left_contact, right_contact])

        # 11. Imitation phase (2D) - cos/sin encoding
        obs.extend([cos_phase, sin_phase])

        return np.array(obs, dtype=np.float32)

    def save_data(self, output_path):
        """Save collected data to HDF5 file."""
        if len(self.observations) == 0:
            self.get_logger().warn("No data collected")
            return

        self.get_logger().info(f"Saving {len(self.observations)} samples to {output_path}")

        with h5py.File(output_path, 'w') as f:
            f.create_dataset('observations', data=np.array(self.observations))
            f.create_dataset('actions', data=np.array(self.actions))
            f.create_dataset('velocity_commands', data=np.array(self.velocity_commands))
            f.create_dataset('base_linear_velocities', data=np.array(self.base_linear_velocities))
            f.create_dataset('timestamps', data=np.array(self.timestamps))
            
            # Save interface data
            if len(self.interface_data_list) > 0:
                f.create_dataset('interface_data', data=np.array(self.interface_data_list))
            
            # Save controller state (as structured data)
            if len(self.controller_state_list) > 0:
                state_grp = f.create_group('controller_state')
                state_grp.create_dataset('last_action', data=np.array([s['last_action'] for s in self.controller_state_list]))
                state_grp.create_dataset('last_last_action', data=np.array([s['last_last_action'] for s in self.controller_state_list]))
                state_grp.create_dataset('last_last_last_action', data=np.array([s['last_last_last_action'] for s in self.controller_state_list]))
                state_grp.create_dataset('motor_targets', data=np.array([s['motor_targets'] for s in self.controller_state_list]))
                state_grp.create_dataset('imitation_i', data=np.array([s['imitation_i'] for s in self.controller_state_list]))
                state_grp.create_dataset('prev_motor_targets', data=np.array([s['prev_motor_targets'] for s in self.controller_state_list]))
                state_grp.create_dataset('onnx_active_steps', data=np.array([s['onnx_active_steps'] for s in self.controller_state_list]))
                state_grp.create_dataset('stabilization_steps', data=np.array([s['stabilization_steps'] for s in self.controller_state_list]))
            
            f.attrs['num_joints'] = self.num_joints
            f.attrs['num_samples'] = len(self.observations)
            f.attrs['observation_dim'] = len(self.observations[0])
            f.attrs['action_dim'] = len(self.actions[0])

        self.get_logger().info("Data saved successfully")

    def cleanup(self):
        """Restore terminal settings."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main():
    parser = argparse.ArgumentParser(description='Manual control data collection in ROS2')
    parser.add_argument(
        '--onnx-model',
        type=str,
        required=True,
        help='Path to ONNX model file'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='ros2_manual_data.h5',
        help='Output HDF5 file path'
    )
    parser.add_argument(
        '--num-joints',
        type=int,
        default=14,
        help='Number of joints (default: 14)'
    )

    args = parser.parse_args()

    rclpy.init()
    node = ManualControlROS2(args.onnx_model, args.num_joints)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nInterrupted by user")
    finally:
        node.cleanup()
        if len(node.observations) > 0:
            node.save_data(args.output)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

