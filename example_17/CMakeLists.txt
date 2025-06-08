cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_17)

find_package(ament_cmake REQUIRED)
find_package(gazebo QUIET)

# INSTALL
install(DIRECTORY
  bringup/launch
  bringup/config
  worlds
  models
  DESTINATION share/ros2_control_demo_example_17
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_17_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_17_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_17_launch test/test_diffbot_launch.py)
endif()

# setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")

ament_package()
