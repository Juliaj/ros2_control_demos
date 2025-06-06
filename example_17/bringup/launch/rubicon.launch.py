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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_prefix


def generate_launch_description():

    pkg_share_path = os.pathsep + os.path.join(
        get_package_prefix("ros2_control_demo_example_17"), "share"
    )
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += pkg_share_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = pkg_share_path

    example_17_share_dir = get_package_share_directory("ros2_control_demo_example_17")
    ros_gz_sim_share_dir = get_package_share_directory("ros_gz_sim")
    print(ros_gz_sim_share_dir)

    gz_args = os.path.join(example_17_share_dir, "worlds", "rubicon.sdf")
    gz_args += " -v 4 -r"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    return LaunchDescription([gz_sim])
