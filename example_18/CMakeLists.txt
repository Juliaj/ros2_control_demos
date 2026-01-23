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
find_package(example_18_motion_controller_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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
  controllers/src/observation_formatter.cpp
)
target_compile_features(observation_formatter PUBLIC cxx_std_17)
# Observation formatter links into shared controller, so enable -fPIC
set_target_properties(observation_formatter PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(observation_formatter PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
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
  controllers/src/action_processor.cpp
)
target_compile_features(action_processor PUBLIC cxx_std_17)
# Action libraries are linked into shared motion_controller, so force -fPIC
set_target_properties(action_processor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(action_processor PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
  $<INSTALL_INTERFACE:include/motion_controller>
)

# Add library for motion controller
add_library(motion_controller SHARED
  controllers/src/motion_controller.cpp
)
target_compile_features(motion_controller PUBLIC cxx_std_17)
target_include_directories(motion_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
  $<INSTALL_INTERFACE:include/motion_controller>
)
target_link_libraries(motion_controller
  observation_formatter
  action_processor
  ${control_msgs_TARGETS}
)
if(onnxruntime_FOUND)
  target_include_directories(motion_controller PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(motion_controller ${ONNXRUNTIME_LIB})
endif()
if(yaml-cpp_FOUND)
  target_link_libraries(motion_controller yaml-cpp)
endif()
ament_target_dependencies(motion_controller
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  control_msgs
  geometry_msgs
  example_18_motion_controller_msgs
  std_msgs
  std_srvs
)
pluginlib_export_plugin_description_file(controller_interface motion_controller.xml)

# INSTALL
install(
  DIRECTORY controllers/include/
  DESTINATION include/motion_controller
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
    motion_controller
  EXPORT export_motion_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

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
    rclcpp::rclcpp
    ${control_msgs_TARGETS}
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

  # Unit tests for MotionController initialization
  ament_add_gtest(test_motion_controller_initialization
    controllers/test/test_motion_controller_initialization.cpp
  )
  target_include_directories(test_motion_controller_initialization PRIVATE
    ${PROJECT_SOURCE_DIR}/controllers/include
  )

  # Unit tests for ONNX shape handling
  ament_add_gtest(test_onnx_shape_handling
    controllers/test/test_onnx_shape_handling.cpp
  )

  # Unit tests for MotionController with MuJoCo (headless simulation)
  # MuJoCo dependency resolution (following mujoco_ros2_control pattern):
  # 1. Try find_package(mujoco) - if installed from source with CMake
  # 2. Check MUJOCO_INSTALL_DIR environment variable - if installed to custom location
  # 3. Auto-download via FetchContent - as last resort (downloads binary release)
  find_package(mujoco QUIET)
  if(mujoco_FOUND)
    message(STATUS "MuJoCo build from source has been found")
    set(MUJOCO_LIB mujoco::mujoco)
    set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
  elseif(DEFINED ENV{MUJOCO_INSTALL_DIR})
    message(STATUS "MuJoCo build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_INSTALL_DIR} instead.")
    set(MUJOCO_DIR $ENV{MUJOCO_INSTALL_DIR})
    find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
    if(NOT MUJOCO_LIB)
      set(mujoco_FOUND FALSE)
    else()
      set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
      set(mujoco_FOUND TRUE)
    endif()
  else()
    include(FetchContent)

    # Detect architecture
    set(MUJOCO_VERSION "3.4.0")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
      set(CPU_ARCH "x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      set(CPU_ARCH "aarch64")
    endif()

    if(CPU_ARCH)
      message(STATUS "No MuJoCo installation found. Downloading MuJoCo ${MUJOCO_VERSION} for ${CPU_ARCH}.")

      set(MUJOCO_DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mujoco)
      set(MUJOCO_EXTRACT_DIR ${MUJOCO_DOWNLOAD_DIR}/mujoco-${MUJOCO_VERSION})
      set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
      fetchcontent_declare(
        mujoco_download
        URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz
        SOURCE_DIR ${MUJOCO_EXTRACT_DIR}
        DOWNLOAD_EXTRACT_TIMESTAMP True
      )
      fetchcontent_makeavailable(mujoco_download)

      set(MUJOCO_DIR ${MUJOCO_EXTRACT_DIR})
      message(STATUS "MuJoCo downloaded to: ${MUJOCO_DIR}")

      find_library(MUJOCO_LIB mujoco HINTS ${MUJOCO_DIR}/lib NO_DEFAULT_PATH)
      if(MUJOCO_LIB)
        set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
        set(mujoco_FOUND TRUE)
      endif()
    endif()
  endif()

  if(mujoco_FOUND)
    ament_add_gtest(test_motion_controller_mujoco
      controllers/test/test_motion_controller_mujoco.cpp
    )
    target_include_directories(test_motion_controller_mujoco PRIVATE
      ${PROJECT_SOURCE_DIR}/controllers/include
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

    add_custom_target(build_example_18_tests ALL
      DEPENDS
        test_observation_formatter
        test_action_processor
        test_motion_controller_initialization
        test_onnx_shape_handling
        test_motion_controller_mujoco
    )
  else()
    message(STATUS "MuJoCo C++ library not found. Skipping test_motion_controller_mujoco.")
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
ament_export_targets(export_motion_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
