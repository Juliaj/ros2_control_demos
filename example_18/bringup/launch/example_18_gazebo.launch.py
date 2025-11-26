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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="ros2_control_demo_example_18",
            description="Package containing the humanoid description files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="berkeley_humanoid_lite_biped.urdf.xacro",
            description="Top level humanoid xacro file.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Optional joint prefix for multi-robot setups.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="berkeley_humanoid_lite_biped",
            description="Name used for the spawned entity in Gazebo.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r -v 3 empty.sdf",
            description="Arguments passed to gz_sim.launch.py (world, verbosity, etc.).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Pass use_sim_time to ROS 2 nodes launched here.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_name = LaunchConfiguration("robot_name")
    gz_args = LaunchConfiguration("gz_args")
    use_sim_time = LaunchConfiguration("use_sim_time")

    description_share = FindPackageShare(description_package)

    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([description_share]),
            ":",
            EnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                default_value="",
            ),
        ],
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([description_share]),
            ":",
            EnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                default_value="",
            ),
        ],
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "description",
                    "urdf",
                    description_file,
                ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
        ],
    )

    bridge_clock_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster_2", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    interfaces_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["interfaces_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    locomotion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["locomotion_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nodes = [
        set_ign_resource_path,
        set_gz_resource_path,
        gz_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_clock_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        interfaces_state_broadcaster_spawner,
        locomotion_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
