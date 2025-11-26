cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_18 LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  realtime_tools
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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
  controllers/src/observation_formatter.cpp
)
target_compile_features(observation_formatter PUBLIC cxx_std_17)
# Observation formatter links into shared controller, so enable -fPIC
set_target_properties(observation_formatter PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(observation_formatter PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
  $<INSTALL_INTERFACE:include/locomotion_controller>
)
ament_target_dependencies(observation_formatter
  control_msgs
  geometry_msgs
)

# Add library for action processor (used by controller and tests)
add_library(action_processor STATIC
  controllers/src/action_processor.cpp
)
target_compile_features(action_processor PUBLIC cxx_std_17)
# Action libraries are linked into shared locomotion_controller, so force -fPIC
set_target_properties(action_processor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(action_processor PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
  $<INSTALL_INTERFACE:include/locomotion_controller>
)

# Add library for locomotion controller
add_library(locomotion_controller SHARED
  controllers/src/locomotion_controller.cpp
)
target_compile_features(locomotion_controller PUBLIC cxx_std_17)
target_include_directories(locomotion_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
  $<INSTALL_INTERFACE:include/locomotion_controller>
)
target_link_libraries(locomotion_controller
  observation_formatter
  action_processor
)
if(onnxruntime_FOUND)
  target_include_directories(locomotion_controller PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(locomotion_controller ${ONNXRUNTIME_LIB})
endif()
ament_target_dependencies(locomotion_controller
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  control_msgs
  geometry_msgs
)
pluginlib_export_plugin_description_file(controller_interface locomotion_controller.xml)

# INSTALL
install(
  DIRECTORY controllers/include/
  DESTINATION include/locomotion_controller
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
    locomotion_controller
  EXPORT export_locomotion_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Export Gazebo resource paths so meshes are discoverable at runtime.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gazebo_model_path.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests for ObservationFormatter
  ament_add_gtest(test_observation_formatter
    controllers/test/test_observation_formatter.cpp
  )
  target_include_directories(test_observation_formatter PRIVATE
    ${PROJECT_SOURCE_DIR}/controllers/include
  )
  target_link_libraries(test_observation_formatter
    observation_formatter
  )
  ament_target_dependencies(test_observation_formatter
    control_msgs
    geometry_msgs
  )

  # Unit tests for ActionProcessor
  ament_add_gtest(test_action_processor
    controllers/test/test_action_processor.cpp
  )
  target_include_directories(test_action_processor PRIVATE
    ${PROJECT_SOURCE_DIR}/controllers/include
  )
  target_link_libraries(test_action_processor
    action_processor
  )
  add_custom_target(build_example_18_tests ALL
    DEPENDS
      test_observation_formatter
      test_action_processor
  )
endif()

## EXPORTS
ament_export_targets(export_locomotion_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
