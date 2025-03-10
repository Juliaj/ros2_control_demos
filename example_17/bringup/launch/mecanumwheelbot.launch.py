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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
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
            "remap_odometry_tf",
            default_value="true",
            description="Remap odometry TF from the steering controller to the TF tree.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_async_hw",
            default_value="true",
            description="Set to true to enable async mode for ros2_control",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "thread_priority",
    #         default_value="1",
    #         description="Set thread priority for ros2_control",
    #     )
    # )

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "is_async_controller",
    #         default_value="true",
    #         description="Set to true to enable async mode for ros2_control",
    #     )
    # )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")
    is_async_hw = LaunchConfiguration("is_async_hw")
    # is_async_controller = LaunchConfiguration("is_async_controller")
    # thread_priority = LaunchConfiguration("thread_priority")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_17"),
                    "urdf",
                    "mecanumwheelbot.urdf.xacro",
                ]
            ),
            " ",
            "is_async:=",
            is_async_hw,
            # " ",
            # "thread_priority:=",
            # thread_priority,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_17"),
            "config",
            "mecanumwheelbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_description"),
            "mecanumwheelbot/rviz",
            "mecanumwheelbot.rviz",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
            # {"is_async": is_async_controller}
        ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file,],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # the steering controller libraries by default publish odometry on a separate topic than /tf
    robot_controller_spawner_remapped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /mecanum_drive_controller/tf_odometry:=/tf",
        ],
        condition=IfCondition(remap_odometry_tf),
    )


    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner_remapped = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner_remapped,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner_remapped,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # launch diagnostic aggregator
    diagnostic_aggregator_config = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_example_17"), "config", "analyzers_config.yaml"]
    )

    diagnostic_aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        parameters=[diagnostic_aggregator_config],
    )

    # launch rqt_robot_monitor if it is installed 
    rqt_robot_monitor_node = Node(
        package="rqt_robot_monitor",
        executable="rqt_robot_monitor", 
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner_remapped,
        delay_joint_state_broadcaster_after_robot_controller_spawner_remapped,
        delay_rviz_after_joint_state_broadcaster_spawner,
        # diagnostic_aggregator_node,
        # rqt_robot_monitor_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
