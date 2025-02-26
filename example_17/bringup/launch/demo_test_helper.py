# Copyright 2025 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from enum import Enum

import random

class MovementState(Enum):
    SIDE_TO_SIDE = 1
    SPINNING = 2

class MecanumDriveControllersTest(Node):
    def __init__(self):
        super().__init__("mecanum_drive_controllers_demo_helper_node")
        self.publisher_ = self.create_publisher(TwistStamped, "/mecanum_drive_controller/reference", 10)

        # Add these class variables to track position and direction
        self.current_y_position = 0.0
        self.y_direction = 1.0  # 1.0 for positive, -1.0 for negative
        self.y_min_bound = -1.0
        self.y_max_bound = 1.0

        # Movement pattern variables
        self.current_movement_state = MovementState.SIDE_TO_SIDE
        self.state_start_time = 0.0
        self.side_to_side_duration = 10.0  # seconds
        self.spinning_duration = 5.0  # seconds

        self.state_start_time = time.time()

    def publish_cmd_vel(self, delay=0.1):

        twist_msg = TwistStamped()

        while rclpy.ok():            
            # do a series of side to side and spinning movements
            twist_msg = self.create_movements()

            # set the timestamp to the current time
            twist_msg.header.stamp = self.get_clock().now().to_msg()

            # log the twist message
            self.get_logger().info(f"Publishing twist message to cmd_vel: {twist_msg}")
            self.publisher_.publish(twist_msg)
            
            time.sleep(delay)

    def create_movements(self):
        """
        Creates a movement message that alternates between side-by-side
        and spinning motions based on time intervals.
        """
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        
        # Check if it's time to transition to the next state
        if self.current_movement_state == MovementState.SIDE_TO_SIDE and elapsed_time > self.side_to_side_duration:
            self.current_movement_state = MovementState.SPINNING
            self.state_start_time = current_time
            self.get_logger().info("Transitioning to SPINNING")
        elif self.current_movement_state == MovementState.SPINNING and elapsed_time > self.spinning_duration:
            self.current_movement_state = MovementState.SIDE_TO_SIDE
            self.state_start_time = current_time
            self.get_logger().info("Transitioning to SIDE_TO_SIDE")
        
        # Generate message based on current state
        if self.current_movement_state == MovementState.SIDE_TO_SIDE:
            # Use our side-to-side logic
            side_speed = 0.2
            delta_time = 0.1
            
            # Update position tracking
            self.current_y_position += self.y_direction * side_speed * delta_time
            
            # Check bounds and change direction if needed
            if self.current_y_position >= self.y_max_bound:
                self.current_y_position = self.y_max_bound
                self.y_direction = -1.0
            elif self.current_y_position <= self.y_min_bound:
                self.current_y_position = self.y_min_bound
                self.y_direction = 1.0
            
            return self.create_msg_forward_and_backward_and_rotate_and_side(
                forward=0.0,
                rotate=0.0,
                side=side_speed * self.y_direction
            )
        else:  # SPINNING state
            # Alternate spinning direction halfway through the spinning duration
            half_duration = self.spinning_duration / 2
            spin_direction = 1.0 if (elapsed_time <= half_duration) else -1.0
            return self.create_msg_spin(rotation_speed=0.5 * spin_direction)
    
    
    def create_msg_side_to_side(self, side_speed=0.2, delta_time=0.1):
        """
        Create a message for side-to-side movement.
        This function updates the current position based on the velocity and time,
        and checks if we've reached the bounds and needs to change direction.   

        Args:
            side_speed: Side speed (-1.0 to 1.0)
            delta_time: Time interval for the movement

        Returns:
            TwistStamped message
        """

        # Update current position based on velocity and time
        self.current_y_position += self.y_direction * side_speed * delta_time
        
        # Check if we've reached the bounds and need to change direction
        if self.current_y_position >= self.y_max_bound:
            self.current_y_position = self.y_max_bound
            self.y_direction = -1.0
        elif self.current_y_position <= self.y_min_bound:
            self.current_y_position = self.y_min_bound
            self.y_direction = 1.0
            
        # Create the message with the appropriate velocity
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = side_speed * self.y_direction
        twist_msg.twist.linear.z = 0.0
        
        self.get_logger().info(f"Position Y: {self.current_y_position:.2f}, Direction: {'right' if self.y_direction > 0 else 'left'}")
        
        return twist_msg
    
    
    def create_msg_forward_and_backward_and_rotate_and_side(self, forward=0.0, rotate=0.5, side=0.0):
        """
        Create a message for robot movement.
        When forward=0.0 and side=0.0, the robot will spin in place at the specified rotate speed.
        
        Args:
            forward: Forward/backward speed (-1.0 to 1.0)
            rotate: Rotation speed in rad/s (positive = counterclockwise)
            side: Side-to-side speed (-1.0 to 1.0)
            
        Returns:
            TwistStamped message
        """
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = forward
        twist_msg.twist.linear.y = side
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.z = rotate
        return twist_msg
        
    def create_msg_spin(self, rotation_speed=1.0):
        """
        Create a message for pure spinning motion.
        
        Args:
            rotation_speed: Rotation speed in rad/s (positive = counterclockwise)
            
        Returns:
            TwistStamped message
        """
        return self.create_msg_forward_and_backward_and_rotate_and_side(
            forward=0.0, 
            rotate=rotation_speed, 
            side=0.0
        )

if __name__ == "__main__":
    rclpy.init()
    test_node = MecanumDriveControllersTest()
    test_node.publish_cmd_vel(delay=0.1)
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()
