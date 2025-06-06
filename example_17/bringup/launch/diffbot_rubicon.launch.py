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


import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import LogInfo

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    example_17_share_dir = get_package_share_directory("ros2_control_demo_example_17")

    sim_plugin_arg = DeclareLaunchArgument(
        "sim_plugin",
        default_value="gz_ros2_control",
        description="whether to use ros2_control plugin or gz_sim differential drive plugin",
    )

    # Launch gazebo with rubicon world
    gz_args = os.path.join(example_17_share_dir, "worlds", "rubicon.sdf")
    gz_args += " -v 4 -r"
    gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # Gazebo Bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Spawn diffbot in gazebo
    x, y, z = -10.0, 0.0, 5.96
    qx, qy, qz = 0.0, 0.0, 0.0
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "diffbot",
            "-allow_renaming",
            "true",
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-roll",
            str(qx),
            "-pitch",
            str(qy),
            "-yaw",
            str(qz),
            "-world",
            "rubicon",
            "-pause",
            "false",
        ],
    )

    def robot_state_publisher(context):
        # choose diffbot model based on sim_plugin_arg
        sim_plugin = LaunchConfiguration("sim_plugin").perform(context)
        model_file = "model.sdf" if sim_plugin == "gz_ros2_control" else "model_gz.sdf"
        diffbot_path = os.path.join(example_17_share_dir, "models", "costar_husky", model_file)

        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                diffbot_path,
            ]
        )
        robot_description = {"robot_description": robot_description_content}
        node_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )
        return [node_robot_state_publisher]

    def ros2_controllers(context):
        sim_plugin = LaunchConfiguration("sim_plugin").perform(context)
        if sim_plugin != "gz_ros2_control":
            LogInfo(msg="Skipping ros2_control nodes because sim_plugin is not gz_ros2_control")
            return []

        robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("ros2_control_demo_example_17"),
                "config",
                "diffbot_controllers.yaml",
            ]
        )
        robot_base_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diffbot_base_controller",
                "--param-file",
                robot_controllers,
                "--controller-ros-args",
                "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
            ],
        )
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        )

        return [robot_base_controller_spawner, joint_state_broadcaster_spawner]

    ld = LaunchDescription(
        [
            sim_plugin_arg,
            gz_world,
            # Add delay to allow Gazebo to fully load
            TimerAction(period=10.0, actions=[]),
            ros_gz_bridge,
            gz_spawn_entity,
        ]
    )
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    ld.add_action(OpaqueFunction(function=ros2_controllers))
    return ld
