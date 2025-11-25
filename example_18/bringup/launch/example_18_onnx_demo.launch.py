#!/usr/bin/env python3

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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ros2_control_demo_example_18"), "model", "policy_biped_25hz_a.onnx"]
        ),
        description="Path to ONNX model file",
    )

    interfaces_state_topic_arg = DeclareLaunchArgument(
        "interfaces_state_topic",
        default_value="/interfaces_state_broadcaster/values",
        description="Topic for interfaces state broadcaster values",
    )

    joint_commands_topic_arg = DeclareLaunchArgument(
        "joint_commands_topic",
        default_value="/forward_position_controller/commands",
        description="Topic to publish joint commands",
    )

    return LaunchDescription(
        [
            model_path_arg,
            interfaces_state_topic_arg,
            joint_commands_topic_arg,
        ]
    )
