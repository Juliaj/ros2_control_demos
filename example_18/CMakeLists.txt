cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_18 LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Disable automatic RPATH from link paths to avoid cycles with mujoco_ros2_control
# ament_add_gtest will set BUILD_RPATH automatically from dependencies
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set(CMAKE_BUILD_RPATH_USE_LINK_PATH FALSE)

# find dependencies
# ROS2 package dependencies (matching package.xml)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  control_msgs
  controller_interface
  geometry_msgs
  example_18_motion_controller_msgs
  hardware_interface
  mujoco_ros2_control
  mujoco_ros2_control_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  transmission_interface
  control_toolbox
)

find_package(ament_cmake REQUIRED)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Non-ROS2 dependencies
find_package(Eigen3 REQUIRED)

# YAML parsing for state injection
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
  add_definitions(-DHAVE_YAML_CPP)
  message(STATUS "yaml-cpp found: State injection from YAML available")
else()
  message(WARNING "yaml-cpp not found. State injection from YAML will not be available.")
endif()

# Optional ONNX Runtime
find_library(ONNXRUNTIME_LIB onnxruntime
  PATHS /usr/local/lib /usr/lib
  NO_DEFAULT_PATH
)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS /usr/local/include /usr/include
  NO_DEFAULT_PATH
)

if(ONNXRUNTIME_LIB AND ONNXRUNTIME_INCLUDE_DIR)
  set(onnxruntime_FOUND TRUE)
  add_definitions(-DONNXRUNTIME_FOUND)
  message(STATUS "ONNX Runtime found: ${ONNXRUNTIME_LIB}")
  message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
else()
  set(onnxruntime_FOUND FALSE)
  message(WARNING "ONNX Runtime not found. Install onnxruntime-dev for full functionality.")
endif()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
# Add library for observation formatter (used by controller and tests)
add_library(observation_formatter STATIC
  controller/src/observation_formatter.cpp
)
target_compile_features(observation_formatter PUBLIC cxx_std_17)
# Observation formatter links into shared controller, so enable -fPIC
set_target_properties(observation_formatter PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(observation_formatter PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/motion_controller>
)
target_link_libraries(observation_formatter
  ${control_msgs_TARGETS}
)
ament_target_dependencies(observation_formatter
  control_msgs
  geometry_msgs
  rclcpp
)

# Add library for action processor (used by controller and tests)
add_library(action_processor STATIC
  controller/src/action_processor.cpp
)
target_compile_features(action_processor PUBLIC cxx_std_17)
# Action libraries are linked into shared motion_controller, so force -fPIC
set_target_properties(action_processor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(action_processor PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/motion_controller>
)

# Add library combining both hardware and controller (matching example_7 pattern)
add_library(
  ros2_control_demo_example_18
  SHARED
  hardware/duck_mini_mujoco_system_interface.cpp
  controller/src/motion_controller.cpp
)
target_compile_features(ros2_control_demo_example_18 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_18 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ros2_control_demo_example_18
  rclcpp
  rclcpp_lifecycle
  control_msgs
  geometry_msgs
  example_18_motion_controller_msgs
  std_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  control_toolbox
  transmission_interface
  mujoco_ros2_control_msgs
  mujoco_ros2_control
)
target_link_libraries(ros2_control_demo_example_18
  observation_formatter
  action_processor
  ${control_msgs_TARGETS}
  mujoco_ros2_control::mujoco_ros2_control
  hardware_interface::hardware_interface
  controller_interface::controller_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  Eigen3::Eigen
)
if(onnxruntime_FOUND)
  target_include_directories(ros2_control_demo_example_18 PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(ros2_control_demo_example_18 ${ONNXRUNTIME_LIB})
endif()
if(yaml-cpp_FOUND)
  target_link_libraries(ros2_control_demo_example_18 yaml-cpp)
endif()
# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_18.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ros2_control_demo_example_18.xml)

# INSTALL
install(
  DIRECTORY controller/include/
  DESTINATION include/motion_controller
)
install(
  DIRECTORY hardware/include/
  DESTINATION include
)
install(
  DIRECTORY bringup/
  DESTINATION share/ros2_control_demo_example_18
)
install(
  DIRECTORY description/
  DESTINATION share/ros2_control_demo_example_18/description
)
install(
  DIRECTORY onnx_model/
  DESTINATION share/ros2_control_demo_example_18/onnx_model
)
install(TARGETS
    observation_formatter
    action_processor
    ros2_control_demo_example_18
  EXPORT export_ros2_control_demo_example_18
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests for ObservationFormatter
  ament_add_gtest(test_observation_formatter
    controller/test/test_observation_formatter.cpp
  )
  target_include_directories(test_observation_formatter PRIVATE
    ${PROJECT_SOURCE_DIR}/controller/include
  )
  target_link_libraries(test_observation_formatter
    observation_formatter
    rclcpp::rclcpp
    ${control_msgs_TARGETS}
  )
  ament_target_dependencies(test_observation_formatter
    control_msgs
    geometry_msgs
  )

  # Unit tests for ActionProcessor
  ament_add_gtest(test_action_processor
    controller/test/test_action_processor.cpp
  )
  target_include_directories(test_action_processor PRIVATE
    ${PROJECT_SOURCE_DIR}/controller/include
  )
  target_link_libraries(test_action_processor
    action_processor
  )

  # Unit tests for MotionController initialization
  ament_add_gtest(test_motion_controller_initialization
    controller/test/test_motion_controller_initialization.cpp
  )
  target_include_directories(test_motion_controller_initialization PRIVATE
    ${PROJECT_SOURCE_DIR}/controller/include
  )

  # Unit tests for ONNX shape handling
  ament_add_gtest(test_onnx_shape_handling
    controller/test/test_onnx_shape_handling.cpp
  )

  # Unit tests for MotionController with MuJoCo (headless simulation)
  # MuJoCo dependency resolution: Use same MuJoCo instance as mujoco_ros2_control
  # 1. Try find_package(mujoco) - if installed from source with CMake
  # 2. Check MUJOCO_INSTALL_DIR environment variable - if installed to custom location
  # Note: We don't download MuJoCo here to avoid RPATH conflicts with mujoco_ros2_control
  find_package(mujoco QUIET)
  if(mujoco_FOUND)
    message(STATUS "MuJoCo build from source has been found")
    set(MUJOCO_LIB mujoco::mujoco)
    set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
  elseif(DEFINED ENV{MUJOCO_INSTALL_DIR})
    message(STATUS "MuJoCo build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_INSTALL_DIR} instead.")
    set(MUJOCO_DIR $ENV{MUJOCO_INSTALL_DIR})
    find_library(MUJOCO_LIB mujoco HINTS ${MUJOCO_DIR}/lib)
    if(NOT MUJOCO_LIB)
      set(mujoco_FOUND FALSE)
    else()
      set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
      set(mujoco_FOUND TRUE)
    endif()
  else()
    message(STATUS "MuJoCo not found via find_package or MUJOCO_INSTALL_DIR. Tests require MuJoCo "
      "to be installed or mujoco_ros2_control to provide it.")
    set(mujoco_FOUND FALSE)
  endif()

  if(mujoco_FOUND)
    ament_add_gtest(test_motion_controller_mujoco
      controller/test/test_motion_controller_mujoco.cpp
    )
    target_include_directories(test_motion_controller_mujoco PRIVATE
      ${PROJECT_SOURCE_DIR}/controller/include
    )

    # Use imported target if available, otherwise use manual variables
    if(TARGET mujoco::mujoco)
      target_link_libraries(test_motion_controller_mujoco
        mujoco::mujoco
      )
    else()
      target_include_directories(test_motion_controller_mujoco SYSTEM PRIVATE
        ${MUJOCO_INCLUDE_DIR}
      )
      target_link_libraries(test_motion_controller_mujoco
        ${MUJOCO_LIB}
      )
    endif()

    # Unit test for contact detection in DuckMiniMujocoSystemInterface
    ament_add_gtest(test_contact_detection
      hardware/test/test_contact_detection.cpp
    )
    target_include_directories(test_contact_detection PRIVATE
      ${PROJECT_SOURCE_DIR}/hardware/include
    )
    target_link_libraries(test_contact_detection
      ros2_control_demo_example_18
    )
    ament_target_dependencies(test_contact_detection
      rclcpp
      rclcpp_lifecycle
      hardware_interface
      mujoco_ros2_control
      example_18_motion_controller_msgs
    )
    if(TARGET mujoco::mujoco)
      target_link_libraries(test_contact_detection
        mujoco::mujoco
      )
    else()
      target_include_directories(test_contact_detection SYSTEM PRIVATE
        ${MUJOCO_INCLUDE_DIR}
      )
      target_link_libraries(test_contact_detection
        ${MUJOCO_LIB}
      )
    endif()

    add_custom_target(build_example_18_tests ALL
      DEPENDS
        test_observation_formatter
        test_action_processor
        test_motion_controller_initialization
        test_onnx_shape_handling
        test_motion_controller_mujoco
        test_contact_detection
    )
  else()
    message(STATUS "MuJoCo C++ library not found. Skipping MuJoCo-dependent tests.")
    add_custom_target(build_example_18_tests ALL
      DEPENDS
        test_observation_formatter
        test_action_processor
        test_motion_controller_initialization
        test_onnx_shape_handling
    )
  endif()
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_18 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
