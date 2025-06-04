import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_prefix



def generate_launch_description():

    pkg_share_path = os.pathsep + os.path.join(get_package_prefix('ros2_control_demo_example_17'), 'share')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += pkg_share_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] =  pkg_share_path

    example_17_share_dir = get_package_share_directory('ros2_control_demo_example_17')
    ros_gz_sim_share_dir = get_package_share_directory('ros_gz_sim')
    print(ros_gz_sim_share_dir)

    gz_args = os.path.join(example_17_share_dir, 'worlds', 'rubicon.sdf')
    gz_args += " -v 4 -r"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return LaunchDescription([gz_sim])