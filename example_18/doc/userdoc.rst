ros2_control_demo_example_18
================================

This demo runs the Berkeley Humanoid Lite ONNX policy
`(HybridRobotics/Berkeley-Humanoid-Lite) <https://github.com/HybridRobotics/Berkeley-Humanoid-Lite>`_
in Gazebo and streams observations via ``interfaces_state_broadcaster``.

Setup
-----

Prerequisites
~~~~~~~~~~~~~

.. code-block:: bash

   # ONNX Runtime
   wget https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-1.23.2.tgz
   tar -xzf onnxruntime-linux-x64-1.23.2.tgz
   sudo cp -r onnxruntime-linux-x64-1.23.2/include/* /usr/local/include/
   sudo cp -r onnxruntime-linux-x64-1.23.2/lib/* /usr/local/lib/
   sudo ldconfig

   # Gazebo (gz-sim Harmonic or newer recommended)
   sudo apt-get update
   sudo apt-get install ros-${ROS_DISTRO}-ros-gz

- Place the trained policy at ``onnx_model/policy_biped_25hz_a.onnx``.
- Source your ROS 2 workspace before running any commands below.

Troubleshooting ONNX Runtime installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you encounter the following error:

.. code-block:: bash
   # find where the library is extracted
   find ~ -name "onnxruntime-linux-x64-*" -type d 2>/dev/null
   
   # copy to the right location
   sudo cp -r /pathto_extracted_library/onnxruntime-linux-x64-1.23.2/include/* /usr/local/include/
   sudo cp -r /pathto_extracted_library/onnxruntime-linux-x64-1.23.2/lib/* /usr/local/lib/
   sudo ldconfig

   # Check library
   ls -l /usr/local/lib/libonnxruntime.so*

   # Check headers
   ls -l /usr/local/include/onnxruntime/core/session/onnxruntime_cxx_api.h

   # If not in /usr/local, check where you extracted
   ls -l ~/onnxruntime-linux-x64-*/lib/
   ls -l ~/onnxruntime-linux-x64-*/include/

   ls -l /usr/local/lib/libonnxruntime.so*
   ls -l /usr/local/include/onnxruntime/

   # check header files
   ls -l /usr/local/include/onnxruntime_cxx_api.h


Run the demo
------------

1. **Optional URDF check**

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 view_robot.launch.py \
        gui:=true use_gazebo:=false

2. **Launch Gazebo and spawn the robot**

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 example_18_gazebo.launch.py \
        gz_args:="-r -v 3 empty.sdf" bridge_clock:=true

   Set ``bridge_clock:=false`` if you do not need ``/clock``.

3. **Bring up controllers**

   .. code-block:: bash

      ros2 control load_controller joint_state_broadcaster --set-state active
      ros2 control load_controller imu_sensor_broadcaster_2 --set-state active 
      ros2 control load_controller interfaces_state_broadcaster --set-state active
      ros2 control load_controller locomotion_controller \
        --param-file $(ros2 pkg prefix ros2_control_demo_example_18)/share/ros2_control_demo_example_18/bringup/config/biped_robot_controllers.yaml
      ros2 control list_controllers

   Ensure every controller reports ``active``.

4. **Start the ONNX pipeline**

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 example_18_onnx_demo.launch.py

   This node subscribes to ``/interfaces_state_broadcaster/values`` and publishes
   actions for the forward position controller.

5. **Command and observe**

   .. code-block:: bash

      # Send a 0.5 m/s forward command
      ros2 topic pub --once /locomotion_controller/cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.5}, angular: {z: 0.0}}"

      # Inspect state/broadcaster streams
      ros2 topic echo /interfaces_state_broadcaster/values --once
      ros2 topic echo /joint_states

Expected behaviour: the robot spawns balanced, follows velocity commands, and logs
warnings if the observation vector dimension deviates from ``10 + 3*N`` (``N=12``).

Testing
-------

Unit tests
~~~~~~~~~~

.. code-block:: bash

   cd ~/ros2_control_ws
   colcon build --packages-select ros2_control_demo_example_18 --cmake-args -DBUILD_TESTING=ON
   source install/setup.bash
   ros2 run ros2_control_demo_example_18 test_observation_formatter
   ros2 run ros2_control_demo_example_18 test_action_processor

- Observation formatter tests cover ordering, relative joint math, and dimension checks.
- Action processor tests confirm scale (0.25), offsets, and negative-value handling.

End-to-end validation
~~~~~~~~~~~~~~~~~~~~~

After the launch sequence above:

- Monitor ``/interfaces_state_broadcaster/values`` and ``/imu_sensor_broadcaster_1/imu`` to confirm sensor flow.
- Check ``ros2 topic echo /joint_states`` to ensure the forward position controller reflects ONNX outputs.
- Try alternative ``cmd_vel`` inputs (turning, reverse) to ensure stability.
- Review the controller logs at ``--log-level debug`` if the observation vector size mismatches the ONNX model.

Architecture
------------

.. code-block:: text

   [Gazebo Simulator]
       ↓ (joint states)
   [Hardware Interface]
       ↓ (state interfaces)
   [Interfaces State Broadcaster]
       ↓ (ROS2 topic: /interfaces_state_broadcaster/values)
   [ONNX Humanoid Controller]
       ↓ (ONNX inference)
       ↓ (ROS2 topic: /forward_position_controller/commands)
   [Forward Position Controller]
       ↓ (command interfaces)
   [Hardware Interface]
       ↓ (joint commands)
   [Gazebo Simulator]
