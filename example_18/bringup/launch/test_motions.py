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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_18_motion_controller_msgs.msg import VelocityCommandWithHead
import time


class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_forward')
        self.publisher_ = self.create_publisher(
            VelocityCommandWithHead, '/motion_controller/cmd_vel', 10
        )
        self.get_logger().info('Drive forward node started')

    def drive_forward(self, duration=2.0, lin_vel_x=0.4, lin_vel_y=0.0, ang_vel_z=0.0, stabilize_time=2.0):
        # Wait for publisher to discover subscribers
        self.get_logger().info('Waiting for subscribers...')
        while self.publisher_.get_subscription_count() == 0:
            time.sleep(0.1)
        self.get_logger().info(f'Connected to {self.publisher_.get_subscription_count()} subscriber(s)')

        publish_rate = 10  # Hz
        sleep_time = 1.0 / publish_rate

        # Stabilization phase: send zero velocity to let robot stabilize
        if stabilize_time > 0:
            self.get_logger().info(f'Stabilizing for {stabilize_time} seconds (zero velocity)...')
            zero_msg = VelocityCommandWithHead()
            zero_msg.base_velocity.linear.x = 0.0
            zero_msg.base_velocity.linear.y = 0.0
            zero_msg.base_velocity.angular.z = 0.0
            zero_msg.head_commands = [0.0, 0.0, 0.0, 0.0]
            
            start_time = time.time()
            while rclpy.ok() and (time.time() - start_time) < stabilize_time:
                self.publisher_.publish(zero_msg)
                time.sleep(sleep_time)

        # Motion phase (skip if duration is 0)
        if duration > 0:
            msg = VelocityCommandWithHead()
            msg.base_velocity.linear.x = float(lin_vel_x)
            msg.base_velocity.linear.y = float(lin_vel_y)
            msg.base_velocity.angular.z = float(ang_vel_z)
            msg.head_commands = [0.0, 0.0, 0.0, 0.0]  # Keep head neutral

            self.get_logger().info(
                f'Moving for {duration} seconds: '
                f'lin_vel_x={lin_vel_x}, lin_vel_y={lin_vel_y}, ang_vel_z={ang_vel_z}'
            )

            start_time = time.time()
            while rclpy.ok() and (time.time() - start_time) < duration:
                self.publisher_.publish(msg)
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
    
    node.get_logger().info('=== Starting Multi-Motion Test (using real robot velocities) ===')
    
    # Test 1: Stabilization (standing still)
    node.get_logger().info('Test 1: Stabilization (4s)')
    node.drive_forward(duration=0.0, stabilize_time=4.0)
    
    # Test 2: Forward walk - EXACT velocity from real robot (fc_test.py line 47)
    node.get_logger().info('Test 2: Forward walk (4s at 0.15 m/s - trained velocity)')
    node.drive_forward(duration=4.0, lin_vel_x=0.15, stabilize_time=0.0)
    
    # Test 3: Backward walk - EXACT velocity from real robot (fc_test.py line 74)
    node.get_logger().info('Test 3: Backward walk (4s at -0.15 m/s)')
    node.drive_forward(duration=4.0, lin_vel_x=-0.15, stabilize_time=0.0)
    
    # Test 4: Turn left - EXACT velocity from real robot (fc_test.py line 56)
    node.get_logger().info('Test 4: Turn left (4s at 1.0 rad/s - trained velocity)')
    node.drive_forward(duration=4.0, ang_vel_z=1.0, stabilize_time=0.0)
    
    # Test 5: Turn right - EXACT velocity from real robot (fc_test.py line 65)
    node.get_logger().info('Test 5: Turn right (4s at -1.0 rad/s)')
    node.drive_forward(duration=4.0, ang_vel_z=-1.0, stabilize_time=0.0)
    
    # Final: Stop and stabilize
    node.get_logger().info('Final: Stopping and stabilizing (3s)')
    node.drive_forward(duration=0.0, stabilize_time=3.0)
    
    node.get_logger().info('=== Multi-Motion Test Complete ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()

