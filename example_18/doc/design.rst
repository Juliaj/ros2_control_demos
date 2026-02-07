Design
======

Overview
--------

This demo controls an Open Duck Mini robot to walk using velocity commands. The system uses an ONNX machine learning model to generate joint position commands for locomotion.

The control pipeline follows three main stages:

1. **Observation Formatter**: Formats sensor data (IMU, joint states, velocity commands) into an observation vector matching the ONNX model's expected input format.

2. **ONNX Model Inference**: Runs the trained policy model to generate raw action outputs.

3. **Action Formatter**: Processes the model outputs by scaling, clamping to joint limits, applying rate limiting, and blending with reference motions before sending commands to hardware.

The system runs at 50 Hz (0.02s period) with a simulation timestep of 0.002s, resulting in an implicit decimation of 10 simulation steps per control update, matching the reference implementation.

Architecture
------------

The system consists of two main components:

1. **state_interfaces_broadcaster**: Aggregates hardware state interfaces into a control_msgs/Float64Values topic, including IMU data and joint states.

2. **MotionController**: A ros2_control controller that subscribes to sensor data and velocity commands, formats observations, runs ONNX inference, and writes joint position commands to hardware.

Data Flow
---------

.. code-block:: text

   [MuJoCo Simulator]
       ↓ (joint states, IMU data, contact sensors)
   [Hardware Interface]
       ↓ (state interfaces)
   [State Interfaces Broadcaster]
       ↓ (ROS2 topic: /state_interfaces_broadcaster/values)
   [Motion Controller]
       ↑ (ROS2 topic: /motion_controller/cmd_vel - velocity commands)
       ↓ (ONNX inference)
       ↓ (command interfaces - joint position commands)
   [Hardware Interface]
       ↓ (joint commands)
   [MuJoCo Simulator]

Challenges
----------

During development, several challenges were encountered when porting the Python reference implementation to ROS2:

Setup 1: Python inference in Mujoco directly
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The robot walks forward successfully.

.. raw:: html

   <video width="640" height="480" controls>
     <source src="imgs/python_mujoco.mp4" type="video/mp4">
     Your browser does not support the video tag.
   </video>

`Video: python_mujoco.mp4 <imgs/python_mujoco.mp4>`_

First, clone the Open_Duck_Playground repository with the required branch:

.. code-block:: bash

   git clone -b adapt_to_rtx5090 https://github.com/Juliaj/Open_Duck_Playground.git ~/dev/Open_Duck_Playground

Terminal 1:

.. code-block:: bash

   cd ~/dev/Open_Duck_Playground
   uv run python tests/validate_onnx_simulation.py --onnx checkpoints/2025_12_26_165635_300482560.onnx --viewer --fall-duration-steps 150000

Setup 2: forward_command_controller in mujoco_ros2_control simulation with Python inference code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The duck walks forward but in circles.

.. raw:: html

   <video width="640" height="480" controls>
     <source src="imgs/forward_command_controller_mujoco.mp4" type="video/mp4">
     Your browser does not support the video tag.
   </video>

`Video: forward_command_controller_mujoco.mp4 <imgs/forward_command_controller_mujoco.mp4>`_

Commands:

Terminal 1:

.. code-block:: bash

   ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py controller_name:=forward_command_controller

Terminal 2:

.. code-block:: bash

   uv run python3 ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/bringup/launch/onnx_infer_ros2.py \
     --onnx-model ~/update_me/example_18/onnx_model/2025_12_26_165635_300482560.onnx

Setup 3: Motion controller in mujoco_ros2_control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The robot doesn't walk forward, rather moves its feet in place.

.. raw:: html

   <video width="640" height="480" controls>
     <source src="imgs/motion_controller_mujoco.mp4" type="video/mp4">
     Your browser does not support the video tag.
   </video>

`Video: motion_controller_mujoco.mp4 <imgs/motion_controller_mujoco.mp4>`_

Terminal 1:

.. code-block:: bash

   ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py

Terminal 2:

.. code-block:: bash

   python3 $(ros2 pkg prefix ros2_control_demo_example_18)/share/ros2_control_demo_example_18/launch/test_motions.py

Debug Support
-------------

The controller includes comprehensive debug support to aid in troubleshooting:

- Debug topic publishers for observations, actions, and intermediate processing steps
- Action quality metrics reporting (clamping rates, action smoothness, extreme actions)
- Throttled logging for observation formatting, action processing, and rate limiting
- ONNX model metadata validation and dimension mismatch detection
- Detailed logging of motor targets, reference motion blending, and command writing

These debug features can be enabled via ROS2 logging levels and are documented in the user documentation.

MotionController Details
------------------------

Update Rate: 50 Hz

The controller runs at 50 Hz (0.02s period) to match the reference implementation's decimation of 10 simulation steps per control update.

Control Decimation
~~~~~~~~~~~~~~~~~~

The reference implementation uses explicit decimation: 10 simulation steps (0.002s each) per control update, resulting in a 50 Hz control rate. mujoco_ros2_control achieves the same decimation implicitly:

- Controller update rate: 50 Hz (0.02s period)
- Simulation timestep: 0.002s (from scene.xml)
- Implicit decimation: 0.02s / 0.002s = 10 steps per control update

This matches the reference implementation exactly. Actions are applied at each controller update and held constant for approximately 10 simulation steps.

Motor velocity limits are calculated using the controller period:
``max_change = max_motor_velocity * dt = 5.24 * 0.02 = 0.1048 rad per control period``

Inputs
~~~~~~

- Sensor data from state_interfaces_broadcaster/values topic:
  - Base angular velocity (from IMU)
  - Projected gravity vector (from IMU orientation)
  - Joint positions and velocities

- Velocity commands from cmd_vel topic (geometry_msgs/Twist)

- Previous action (stored internally)

Processing
~~~~~~~~~~

- Format observation vector for ONNX model
- Run inference using ONNX model
- Process model outputs (scale, clamp to limits)
- Write joint position commands to hardware

Observation Vector Format
-------------------------

The observation vector is concatenated in this order:

1. Velocity commands (4D: lin_vel_x, lin_vel_y, ang_vel_z, heading)
2. Base angular velocity (3D vector from IMU)
3. Projected gravity vector (3D vector from IMU orientation)
4. Joint positions (N joints, relative to default positions)
5. Joint velocities (N joints, absolute)
6. Previous action (N joints)

Total dimension: 10 + 3*N (where N = number of leg joints)


Hardware Layer
--------------

- Exposes state interfaces for IMU sensor and joint states
- Accepts command interfaces for joint positions

DuckMiniMujocoSystemInterface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``DuckMiniMujocoSystemInterface`` extends ``mujoco_ros2_control::MujocoSystemInterface`` to add foot contact detection for gait control. It reads collisions from MuJoCo's ``mjData->contact[]`` array and exports two state interfaces per sensor: ``contact_raw`` (unfiltered) and ``contact`` (debounced in GAIT mode). Contact sensors are configured in the ros2_control xacro with ``mujoco_type="contact"``, ``body1_name``, ``body2_name``, and optional ``contact_consumer`` ("collision" or "gait") and debounce parameters.

MuJoCo Model Selection
-----------------------

The simulation uses ``open_duck_mini_v2.xml`` which includes motor actuators with BAM-identified parameters (damping, armature, frictionloss). This matches the actuator dynamics used during ONNX model training, ensuring sim-to-real transfer accuracy.

The model is loaded via ``scene.xml``, which includes ``open_duck_mini_v2.xml`` (line 6). The BAM-identified parameters for STS3215 servos are defined in the ``sts3215`` default class:

- ``damping="0.56"``
- ``armature="0.027"``
- ``frictionloss="0.068"``
- ``kp="13.37"`` (position gain)
- ``forcerange="-3.23 3.23"`` (N⋅m)

These parameters are embedded directly in ``open_duck_mini_v2.xml`` (line 47) and match the actuator dynamics used during ONNX model training. The model uses MuJoCo's built-in position actuators, ensuring the same control behavior as the reference implementation.

Controller Manager
------------------

- Runs control loop at 50 Hz
- Manages lifecycle of state_interfaces_broadcaster and MotionController

User Command Interface
-----------------------

Publish geometry_msgs/Twist messages to the velocity command topic (default: ~/cmd_vel).

Example: To command forward walking at 0.5 m/s, publish Twist with linear.x = 0.5, linear.y = 0.0, angular.z = 0.0. The controller formats this as [0.5, 0.0, 0.0, 0.0] (lin_vel_x, lin_vel_y, ang_vel_z, heading) and includes it in the observation vector.
