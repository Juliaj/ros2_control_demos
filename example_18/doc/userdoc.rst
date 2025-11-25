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
   sudo apt-get install libonnxruntime-dev

   # Gazebo (gz-sim Harmonic or newer recommended)
   sudo apt-get update
   sudo apt-get install ros-${ROS_DISTRO}-ros-gz

- Place the trained policy at ``onnx_model/policy_biped_25hz_a.onnx``.
- Source your ROS 2 workspace before running any commands below.

Run the demo
------------

1. **Optional URDF check**

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 view_robot.launch.py \
        gui:=true use_gazebo:=false

You should be able to see the robot in Rviz2 by running the above command.
<img src="biped_robot.png" alt="Rviz2" width="500">

2. **Launch Gazebo and spawn the robot**

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 example_18_gazebo.launch.py \
        gz_args:="-r -v 3 empty.sdf" bridge_clock:=true

   Set ``bridge_clock:=false`` if you do not need ``/clock``.


To be continued...
