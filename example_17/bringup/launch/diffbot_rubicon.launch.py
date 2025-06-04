import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    example_17_share_dir = get_package_share_directory('ros2_control_demo_example_17')
    ros_gz_sim_share_dir = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(example_17_share_dir, 'worlds', 'rubicon.sdf')
    world_path += " -v 4 -r"
    diffbot_path = os.path.join(example_17_share_dir, 'models', 'costar_husky', 'model.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    def robot_state_publisher(context):
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                diffbot_path,
            ]
        )
        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ros2_control_demo_example_17'),
            'config',
            'diffbot_chained_controllers.yaml',
        ]
    )

    x, y, z = -10.0, 0.0, 5.96
    qx, qy, qz = 0.0, 0.0, 0.0
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-name','diffbot', 
            '-allow_renaming', 'true',
            '-x', str(x), '-y', str(y), '-z', str(z),
            '-roll', str(qx), '-pitch', str(qy), '-yaw', str(qz),
            '-world', 'rubicon',
            '-pause', 'false'
            ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diffbot_base_controller',
            '--param-file',
            robot_controllers,
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
            ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    example_17_share_dir = get_package_share_directory('ros2_control_demo_example_17')    

    gz_args = os.path.join(example_17_share_dir, 'worlds', 'rubicon.sdf')
    gz_args += " -v 4 -r"

    ld = LaunchDescription([
        # Launch gazebo with rubicon world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': gz_args}.items(),
        ),

        # Add delay to allow Gazebo to fully load
        TimerAction(
            period=10.0,
            actions=[]
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        bridge,
        gz_spawn_entity,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
