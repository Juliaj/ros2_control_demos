#!/usr/bin/env python3
#
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
Test motion sequences for Open Duck Mini robot.

Based on manual_control_ros2.py and motion_controller behavior:
- Controller has internal stabilization delay: 50 steps (~1.0s at 50Hz)
- Controller has blend-in period: 200 steps (~4.0s at 50Hz)
- Total controller warm-up time: ~5.0 seconds before full ONNX control
- Default forward velocity: 0.15 m/s (max forward, matching training)
- Command format: [lin_vel_x, lin_vel_y, ang_vel, neck_pitch, head_pitch, head_yaw, head_roll]
- Head commands default to zero (neutral position)
- Velocity ramp-up: Gradually increases velocity from 0.01 m/s to target (default 3.0s)
  This prevents oscillation when applying full velocity immediately, matching manual control behavior.

This script accounts for controller warm-up periods and includes velocity ramp-up
to ensure smooth motion transitions.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_18_motion_controller_msgs.msg import VelocityCommandWithHead
import time


class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_forward')
        self.publisher_ = self.create_publisher(
            VelocityCommandWithHead, '/motion_controller/cmd_head_velocity', 10
        )
        self.get_logger().info('Drive forward node started')
        
        # Controller timing parameters (matching motion_controller and manual_control_ros2.py)
        self.controller_update_rate = 50.0  # Hz
        self.stabilization_delay_steps = 50  # Steps before ONNX starts
        self.blend_in_steps = 200  # Steps to blend in ONNX actions
        self.stabilization_delay_time = self.stabilization_delay_steps / self.controller_update_rate  # ~1.0s
        self.blend_in_time = self.blend_in_steps / self.controller_update_rate  # ~4.0s
        self.controller_warmup_time = self.stabilization_delay_time + self.blend_in_time  # ~5.0s

    def drive_forward(self, duration=2.0, lin_vel_x=0.15, lin_vel_y=0.0, ang_vel_z=0.0, 
                     stabilize_time=None, wait_for_warmup=True, ramp_up_duration=3.0, 
                     ramp_up_increment=0.01):
        # Wait for publisher to discover subscribers
        self.get_logger().info('Waiting for subscribers...')
        while self.publisher_.get_subscription_count() == 0:
            time.sleep(0.1)
        self.get_logger().info(f'Connected to {self.publisher_.get_subscription_count()} subscriber(s)')
        
        # Give connection time to fully establish
        time.sleep(0.5)

        publish_rate = self.controller_update_rate  # Hz (match controller update rate)
        sleep_time = 1.0 / publish_rate

        # Determine stabilization time (default: controller warm-up time if wait_for_warmup is True)
        if stabilize_time is None:
            if wait_for_warmup:
                stabilize_time = self.controller_warmup_time
                self.get_logger().info(
                    f'Using default stabilization time: {stabilize_time:.1f}s '
                    f'(controller warm-up: {self.stabilization_delay_time:.1f}s stabilization + '
                    f'{self.blend_in_time:.1f}s blend-in)'
                )
            else:
                stabilize_time = 0.0

        # Stabilization phase: send zero velocity to let robot stabilize and controller warm up
        if stabilize_time > 0:
            self.get_logger().info(
                f'Stabilizing for {stabilize_time:.1f} seconds (zero velocity) - '
                f'controller will complete warm-up during this time'
            )
            zero_msg = VelocityCommandWithHead()
            zero_msg.base_velocity.linear.x = 0.0
            zero_msg.base_velocity.linear.y = 0.0
            zero_msg.base_velocity.angular.z = 0.0
            zero_msg.head_commands = [0.0, 0.0, 0.0, 0.0]
            
            start_time = time.time()
            while rclpy.ok() and (time.time() - start_time) < stabilize_time:
                self.publisher_.publish(zero_msg)
                time.sleep(sleep_time)
            
            if wait_for_warmup and stabilize_time >= self.controller_warmup_time:
                self.get_logger().info(
                    f'Controller warm-up complete - ONNX control should now be fully active '
                    f'(blend-in finished after {self.controller_warmup_time:.1f}s)'
                )

        # Motion phase (skip if duration is 0)
        # Command format matches validate_onnx_simulation.py ForwardWalkCommand:
        # [lin_vel_x, lin_vel_y, ang_vel, neck_pitch, head_pitch, head_yaw, head_roll]
        if duration > 0:
            # Ramp-up phase: gradually increase velocity to target (matching manual_control_ros2.py behavior)
            # This prevents oscillation when applying full velocity immediately
            if ramp_up_duration > 0 and abs(lin_vel_x) > 0.001:
                self.get_logger().info(
                    f'Ramping up velocity from {ramp_up_increment:.3f} to {lin_vel_x:.3f} m/s '
                    f'over {ramp_up_duration:.1f} seconds'
                )
                
                # Calculate number of steps and velocity increment per step
                ramp_steps = int(ramp_up_duration * publish_rate)
                start_vel = ramp_up_increment if lin_vel_x > 0 else -ramp_up_increment
                target_vel = lin_vel_x
                vel_range = abs(target_vel - start_vel)
                
                if ramp_steps > 0:
                    vel_increment_per_step = vel_range / ramp_steps
                    current_vel = start_vel
                    
                    ramp_msg = VelocityCommandWithHead()
                    ramp_msg.base_velocity.linear.y = float(lin_vel_y)
                    ramp_msg.base_velocity.angular.z = float(ang_vel_z)
                    ramp_msg.head_commands = [0.0, 0.0, 0.0, 0.0]
                    
                    for step in range(ramp_steps):
                        if lin_vel_x > 0:
                            current_vel = min(start_vel + step * vel_increment_per_step, target_vel)
                        else:
                            current_vel = max(start_vel - step * vel_increment_per_step, target_vel)
                        
                        ramp_msg.base_velocity.linear.x = float(current_vel)
                        self.publisher_.publish(ramp_msg)
                        
                        if step % 25 == 0:  # Log every 0.5 seconds
                            self.get_logger().info(
                                f'Ramp-up: vel_x={current_vel:.3f} m/s (target: {target_vel:.3f})'
                            )
                        
                        time.sleep(sleep_time)
                    
                    self.get_logger().info(
                        f'Ramp-up complete: reached target velocity {target_vel:.3f} m/s'
                    )
            
            # Constant velocity phase: maintain target velocity
            msg = VelocityCommandWithHead()
            msg.base_velocity.linear.x = float(lin_vel_x)
            msg.base_velocity.linear.y = float(lin_vel_y)
            msg.base_velocity.angular.z = float(ang_vel_z)
            msg.head_commands = [0.0, 0.0, 0.0, 0.0]  # Head neutral (matches reference default)

            self.get_logger().info(
                f'Moving at constant velocity for {duration} seconds: '
                f'lin_vel_x={lin_vel_x}, lin_vel_y={lin_vel_y}, ang_vel_z={ang_vel_z}'
            )

            start_time = time.time()
            msg_count = 0
            while rclpy.ok() and (time.time() - start_time) < duration:
                self.publisher_.publish(msg)
                msg_count += 1
                if msg_count % 50 == 0:  # Log every 50 messages (~1 second at 50 Hz)
                    self.get_logger().info(
                        f'Published {msg_count} messages (elapsed: {time.time() - start_time:.1f}s)'
                    )
                time.sleep(sleep_time)

            self.get_logger().info('Motion command completed')
        
        # Stopping phase: send zero velocity to stop robot
        self.get_logger().info('Sending stop command (zero velocity)...')
        stop_msg = VelocityCommandWithHead()
        stop_msg.base_velocity.linear.x = 0.0
        stop_msg.base_velocity.linear.y = 0.0
        stop_msg.base_velocity.angular.z = 0.0
        stop_msg.head_commands = [0.0, 0.0, 0.0, 0.0]
        
        # Publish stop command for 2 seconds to ensure controller receives it
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 2.0:
            self.publisher_.publish(stop_msg)
            time.sleep(sleep_time)
        
        self.get_logger().info('Test completed')


def main(args=None):
    rclpy.init(args=args)
    node = DriveForward()
    
    node.get_logger().info('=== Starting Multi-Motion Test (matching manual_control_ros2.py behavior) ===')
    node.get_logger().info(
        f'Controller timing: {node.stabilization_delay_time:.1f}s stabilization + '
        f'{node.blend_in_time:.1f}s blend-in = {node.controller_warmup_time:.1f}s total warm-up'
    )
    
    # Test 1: Initial stabilization and controller warm-up
    # Reference: manual_control_ros2.py - controller needs time to stabilize and blend-in
    node.get_logger().info(
        f'Test 1: Initial stabilization ({node.controller_warmup_time:.1f}s) - '
        'controller warm-up (stabilization + blend-in)'
    )
    node.drive_forward(duration=0.0, stabilize_time=node.controller_warmup_time, wait_for_warmup=True)
    
    # Test 2: Forward walk - matching manual_control_ros2.py default velocity
    # Reference: manual_control_ros2.py uses 0.15 m/s as default forward velocity
    # Note: Controller is now fully warmed up, so motion should start immediately
    node.get_logger().info('Test 2: Forward walk (20s at 0.15 m/s - matching manual_control default)')
    node.drive_forward(duration=20.0, lin_vel_x=0.15, stabilize_time=0.0, wait_for_warmup=False)
    
    # # Test 3: Backward walk - reverse of forward
    # node.get_logger().info('Test 3: Backward walk (4s at -0.15 m/s)')
    # node.drive_forward(duration=4.0, lin_vel_x=-0.15, stabilize_time=0.0)
    
    # # Test 4: Turn left - moderate angular velocity
    # node.get_logger().info('Test 4: Turn left (4s at 0.5 rad/s)')
    # node.drive_forward(duration=4.0, ang_vel_z=0.5, stabilize_time=0.0)
    
    # # Test 5: Turn right - reverse of turn left
    # node.get_logger().info('Test 5: Turn right (4s at -0.5 rad/s)')
    # node.drive_forward(duration=4.0, ang_vel_z=-0.5, stabilize_time=0.0)
    
    # # Test 6: Forward walk at reference velocity (longer duration for validation)
    # # Reference: validate_onnx_simulation.py uses 120s default duration
    # node.get_logger().info('Test 6: Extended forward walk (10s at 0.15 m/s - validation test)')
    # node.drive_forward(duration=10.0, lin_vel_x=0.15, stabilize_time=0.0)
    
    # # Final: Stop and stabilize
    # node.get_logger().info('Final: Stopping and stabilizing (3s)')
    # node.drive_forward(duration=0.0, stabilize_time=3.0)
    
    node.get_logger().info('=== Multi-Motion Test Complete ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()

