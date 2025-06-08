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


class DiffbotChainedControllersTest(Node):
    def __init__(self):
        super().__init__("diffbot_chained_controllers_demo_helper_node")
        # Enable feedforward control via service call
        self.client_ = self.create_client(SetBool, "/wheel_pids/set_feedforward_control")
        self.publisher_ = self.create_publisher(TwistStamped, "/cmd_vel", 10)

    def set_feedforward_control(self):
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for feedforward control service to be available...")

        request = SetBool.Request()
        request.data = True
        future = self.client_.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("Enabled feedforward control for both wheels.")

    def publish_cmd_vel(self, delay=0.1):

        twist_msg_straight = TwistStamped()
        twist_msg_straight.twist.linear.x = 0.5
        twist_msg_straight.twist.linear.y = 0.0
        twist_msg_straight.twist.linear.z = 0.0
        twist_msg_straight.twist.angular.x = 0.0
        twist_msg_straight.twist.angular.y = 0.0
        twist_msg_straight.twist.angular.z = 0.0

        twist_msg_turn = TwistStamped()
        twist_msg_turn.twist.linear.x = 0.5
        twist_msg_turn.twist.linear.y = 0.0
        twist_msg_turn.twist.linear.z = 0.0
        twist_msg_turn.twist.angular.x = 0.0
        twist_msg_turn.twist.angular.y = 0.0
        twist_msg_turn.twist.angular.z = 1.0

        twist_msg_turn_backward = TwistStamped()
        twist_msg_turn_backward.twist.linear.x = -0.5
        twist_msg_turn_backward.twist.linear.y = 0.0
        twist_msg_turn_backward.twist.linear.z = 0.0
        twist_msg_turn_backward.twist.angular.x = 0.0
        twist_msg_turn_backward.twist.angular.y = 0.0
        twist_msg_turn_backward.twist.angular.z = 0.0

        count = 0
        while rclpy.ok():
            if count < 180:
                self.get_logger().info(f"{count}, driving straight: {twist_msg_straight}")
                self.publisher_.publish(twist_msg_straight)

            elif count < 200:
                self.get_logger().info(f"{count}, turn: {twist_msg_turn}")
                self.publisher_.publish(twist_msg_turn)

            elif count < 300:
                self.get_logger().info(f"{count}, go backward: {twist_msg_turn_backward}")
                self.publisher_.publish(twist_msg_turn_backward)
            # else:
            #     self.get_logger().info(f"Publishing twist message to cmd_vel: {twist_msg_turn}")
            #     self.publisher_.publish(twist_msg_turn)
            count += 1
            time.sleep(delay)


if __name__ == "__main__":
    rclpy.init()
    test_node = DiffbotChainedControllersTest()
    # test_node.set_feedforward_control()
    test_node.publish_cmd_vel(delay=0.1)
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()
