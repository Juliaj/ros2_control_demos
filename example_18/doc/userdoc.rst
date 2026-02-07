ros2_control_demo_example_18
================================

This demo runs a policy driven
`Open Duck Mini v2 <https://github.com/apirrone/Open_Duck_Mini/tree/v2/mini_bdx/robots/open_duck_mini_v2>`_
in MuJoCo and streams observations via ``state_interfaces_broadcaster``.

Setup
-----

Prerequisites
~~~~~~~~~~~~~

Dependencies
^^^^^^^^^^^^

This demo requires `mujoco_ros2_control <https://github.com/ros-controls/mujoco_ros2_control>`_ and uses a custom hardware interface (``DuckMiniMujocoSystemInterface``) that extends the base MuJoCo interface to add foot contact detection.

Follow the installation instructions from the mujoco_ros2_control repository to set up the MuJoCo simulation environment.

ONNX Runtime
^^^^^^^^^^^^

.. code-block:: bash

   # ONNX Runtime (tested on RTX 3080Ti and RTX 5090 with CUDA 12.8 and driver 570.144)
   wget https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-1.23.2.tgz
   tar -xzf onnxruntime-linux-x64-1.23.2.tgz
   sudo cp -r onnxruntime-linux-x64-1.23.2/include/* /usr/local/include/
   sudo cp -r onnxruntime-linux-x64-1.23.2/lib/* /usr/local/lib/
   sudo ldconfig

- Place your trained ONNX model in the ``onnx_model/`` directory, or update the ``model_path`` parameter in ``open_duck_mini_controllers.yaml`` to point to your model file. The default configuration uses ``onnx_model/2025_12_26_165635_300482560.onnx``.

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

   # If not in /usr/local, check where you extracted
   ls -l ~/onnxruntime-linux-x64-*/lib/
   ls -l ~/onnxruntime-linux-x64-*/include/

   ls -l /usr/local/lib/libonnxruntime.so*
   ls -l /usr/local/include/onnxruntime/

   # check header files
   ls -l /usr/local/include/onnxruntime_cxx_api.h

Compile the demo
-----------------

1. Clone and build mujoco_ros2_control:

   .. code-block:: bash

      # Clone the repository (if not already cloned)
      cd ~/ros2_ws/src
      git clone https://github.com/ros-controls/mujoco_ros2_control.git

      # Build mujoco_ros2_control
      cd ~/ros2_ws
      colcon build --symlink-install --packages-select mujoco_ros2_control

      # Source the mujoco_ros2_control workspace
      source install/setup.bash

2. Build example_18:

   .. code-block:: bash

      # From your ROS 2 workspace root
      colcon build --symlink-install --packages-select ros2_control_demo_example_18

      # Source the workspace in this order
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash

Run the demo
------------

The demo supports two controller modes. Each mode has its own launch command and corresponding test script.

1. Motion Controller (ONNX policy)

   Launch MuJoCo simulation with motion controller:

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py controller_name:=motion_controller

      # Check that controllers are running:
      ros2 control list_controllers

      # Send velocity commands using the test script:
      python3 $(ros2 pkg prefix ros2_control_demo_example_18)/share/ros2_control_demo_example_18/launch/test_motions.py

   The script publishes velocity commands at 50 Hz and includes warm-up time for controller stabilization and blend-in. You should see the duck moving its feet.

2. Forward Command Controller (direct joint position commands)

   Launch MuJoCo simulation with forward command controller:

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py controller_name:=forward_command_controller

      # Check that controllers are running:
      ros2 control list_controllers

   Send commands using the ONNX inference script. First, clone the Open_Duck_Playground repository:

   .. code-block:: bash

      git clone -b adapt_to_rtx5090 https://github.com/Juliaj/Open_Duck_Playground.git ~/dev/Open_Duck_Playground

   Then run the ONNX inference script from another terminal:

   .. code-block:: bash

      cd ~/dev/Open_Duck_Playground
      uv run python3 ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/bringup/launch/onnx_infer_ros2.py \
        --onnx-model $(ros2 pkg prefix ros2_control_demo_example_18)/share/ros2_control_demo_example_18/onnx_model/2025_12_26_165635_300482560.onnx

   The script uses ONNX model inference to generate joint actions. Press 'O' to enable ONNX control, and use arrow keys or WASD to adjust velocity commands.
