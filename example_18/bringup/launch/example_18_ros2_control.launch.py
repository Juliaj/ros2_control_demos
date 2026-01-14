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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro with mock hardware option
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_18"),
                    "description",
                    "urdf",
                    "open_duck_mini.urdf.xacro",
                ]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )

    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    controller_parameters = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare("ros2_control_demo_example_18"),
                "config",
                "open_duck_mini_controllers.yaml",
            ]
        ),
    )

    # robot_state_publisher: Publishes TF transforms based on joint states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Control node using controller_manager (not mujoco_ros2_control)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            controller_parameters,
        ],
        arguments=["--log-level", "debug"],
    )

    # Spawn joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=["joint_state_broadcaster"],
        output="both",
    )

    # Spawn state_interfaces_broadcaster
    spawn_state_interfaces_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_state_interfaces_broadcaster",
        arguments=["state_interfaces_broadcaster"],
        output="both",
    )

    # Spawn motion_controller
    spawn_motion_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_motion_controller",
        arguments=["motion_controller"],
        output="both",
    )

    # RViz2 node (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_description"),
                    "rrbot/rviz/rrbot.rviz",
                ]
            ),
        ],
        condition=IfCondition(gui),
    )

    return LaunchDescription(
        [
            *declared_arguments,
            robot_state_publisher_node,
            control_node,
            spawn_joint_state_broadcaster,
            spawn_state_interfaces_broadcaster,
            spawn_motion_controller,
            # rviz_node,
        ]
    )

