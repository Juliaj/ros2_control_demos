cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_17 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  mecanum_drive_controller
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ros2_control_demo_example_17
  SHARED
  hardware/mecanumwheelbot_system.cpp
  controller/mecanumwheelbot_controller.cpp
)
target_compile_features(ros2_control_demo_example_17 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_17 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_17>
)
ament_target_dependencies(
  ros2_control_demo_example_17 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_17.xml)
# Export controller plugin
pluginlib_export_plugin_description_file(controller_interface ros2_control_demo_example_17.xml)

# INSTALL
install(
  DIRECTORY hardware/include/ controller/include/
  DESTINATION include/ros2_control_demo_example_17
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_17
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_17
)
install(TARGETS 
  ros2_control_demo_example_17
  EXPORT export_ros2_control_demo_example_17
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_17_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_17_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_17_launch test/test_mecanumwheelbot_launch.py)
  ament_add_pytest_test(run_example_17_launch_remapped test/test_mecanumwheelbot_launch_remapped.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_17 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()

include_directories(
  include
  ${mecanum_drive_controller_INCLUDE_DIRS}
)
