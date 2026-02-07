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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_name = LaunchConfiguration("controller_name")

    # NOTE: URDF is loaded ONLY for robot_state_publisher to compute TF transforms.
    # The actual robot model for simulation is loaded by MuJoCo from scene.xml via
    # the mujoco_model parameter in ros2_control xacro (open_duck_mini.ros2_control.xacro).
    # This URDF should be minimal - containing only kinematic structure (links/joints)
    # needed for TF tree computation, not the full robot description.
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
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    controller_parameters = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare("ros2_control_demo_example_18"),
                "config",
                "open_duck_mini_controllers.yaml",
            ]
        ),
    )

    # robot_state_publisher: Publishes TF transforms based on joint states from joint_state_broadcaster.
    # Requires minimal URDF with kinematic structure (links/joints) to compute TF tree.
    # The actual simulation model comes from MuJoCo XML, not from this URDF.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            controller_parameters,
        ],
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
        ],
        output="both",
    )

    spawn_state_interfaces_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_state_interfaces_broadcaster",
        arguments=[
            "state_interfaces_broadcaster",
        ],
        output="both",
    )

    # Spawn motion_controller: controls robot joints using ONNX model inference
    spawn_motion_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_motion_controller",
        arguments=[
            "motion_controller",
        ],
        output="both",
        condition=IfCondition(
            PythonExpression(["'", controller_name, "' == 'motion_controller'"])
        ),
    )

    # Spawn forward_command_controller: direct joint position command interface.
    # Useful for data collection / scripted trajectories in simulation.
    # Tip: you can use Open_Duck_Playground's headless MuJoCo driver to generate action trajectories
    # (policy inference -> action -> motor_targets) and publish those joint targets into ROS2:
    # https://github.com/Juliaj/Open_Duck_Playground/blob/adapt_to_rtx5090/tests/validate_onnx_simulation.py
    spawn_forward_command_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_forward_command_controller",
        arguments=[
            "forward_command_controller",
        ],
        output="both",
        condition=IfCondition(
            PythonExpression(["'", controller_name, "' == 'forward_command_controller'"])
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_name",
                default_value="motion_controller",
                description=(
                    "Which controller to spawn: "
                    "'motion_controller' (ONNX policy) or 'forward_command_controller' (direct joint position commands)."
                ),
            ),
            robot_state_publisher_node,
            control_node,
            spawn_joint_state_broadcaster,
            spawn_state_interfaces_broadcaster,
            spawn_motion_controller,
            spawn_forward_command_controller,
        ]
    )
