Design
======

Scenario
--------

Control a Open Duck Mini robot to walk forward using geometry_msgs/Twist messages. An ONNX ML model generates joint position commands to achieve the desired locomotion.

Architecture
------------

The system consists of two main components:

1. state_interfaces_broadcaster: Aggregates hardware state interfaces into a control_msgs/Float64Values topic, including IMU data and joint states.

2. MotionController: A ros2_control controller that subscribes to sensor data and velocity commands, formats observations, runs ONNX inference, and writes joint position commands to hardware.


Challenges:

Setup 1: Python inference in Mujoco directly: The robot walks forward.

.. raw:: html

   <video width="640" height="480" controls>
     <source src="imgs/python_mujoco.mp4" type="video/mp4">
     Your browser does not support the video tag.
   </video>

`Video: python_mujoco.mp4 <imgs/python_mujoco.mp4>`_

Terminal 1:

.. code-block:: bash

   uv run python tests/validate_onnx_simulation.py --onnx checkpoints/2025_12_26_165635_300482560.onnx --viewer --fall-duration-steps 5000

Setup 2: forward_command_controller in mujoco_ros2_control simulation with Python inference code, duck walks forward somehow but in circles.

.. raw:: html

   <video width="640" height="480" controls>
     <source src="imgs/forward_command_controller_mujoco.mp4" type="video/mp4">
     Your browser does not support the video tag.
   </video>

`Video: forward_command_controller_mujoco.mp4 <imgs/forward_command_controller_mujoco.mp4>`_

Commands:

Terminal 1:

.. code-block:: bash

   ros2 launch ros2_control_demo_example_18 data_collection.launch.py

Terminal 2:

.. code-block:: bash

   uv run python3 ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/fine_tuning/manual_control_ros2.py \
     --output mujoco_manual_data.h5 \
     --onnx-model ~/dev/Open_Duck_Playground/checkpoints/2025_12_26_165635_300482560.onnx

Setup 3: Motion controller in mujoco_ros2_control, action is driven by MotionController: The robot doesn't walk forward, rather moves its feet in place.

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



Data Flow
---------

.. code-block:: text

   Hardware State Interfaces
       ↓
   state_interfaces_broadcaster → control_msgs/Float64Values topic
       ↓
   LocomotionController
       ↓ (ONNX inference)
   Hardware Command Interfaces

LocomotionController
--------------------

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

Inputs:

- Sensor data from state_interfaces_broadcaster/values topic:
  - Base angular velocity (from IMU)
  - Projected gravity vector (from IMU orientation)
  - Joint positions and velocities

- Velocity commands from cmd_vel topic (geometry_msgs/Twist)

- Previous action (stored internally)

Processing:

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

Gazebo Integration
------------------

The gz_ros2_control/GazeboSimSystem plugin handles Gazebo communication:

- Reads joint positions/velocities from simulation
- Reads IMU data from Gazebo IMU sensors
- Writes joint position commands to Gazebo actuators

Hardware Layer
--------------

- Exposes state interfaces for IMU sensor and joint states
- Accepts command interfaces for joint positions

MuJoCo Model Selection
-----------------------

The simulation uses robot_motors.xml which includes motor actuators with BAM-identified parameters (damping, armature, frictionloss). This matches the actuator dynamics used during ONNX model training, ensuring sim-to-real transfer accuracy.

Checking sim2real.md and the ONNX model setup to confirm the actuator configuration.

Use `robot_motors.xml` (via `scene.xml`)

Observations: 
1. Training configuration: `sim2real.md` states BAM-identified parameters (damping, armature, frictionloss) are set in the MJCF. `robot_motors.xml` includes these:
   - `damping="0.0"`, `armature="0.027"`, `frictionloss="0.083"` (line 69)
2. Scene files:
   - `scene.xml` includes `robot_motors.xml` (line 6)
   - `scene_position.xml` includes `robot.xml` (line 5)
3. ONNX testing: `experiments/v2/onnx_AWD_mujoco.py` uses `scene.xml` (line 70), which loads `robot_motors.xml`.
4. Actuator type: `robot_motors.xml` uses `<motor>` actuators, matching the BAM-identified motor model used during training.

Design decision: Use `robot_motors.xml` (via `scene.xml`) to match the training setup and BAM parameters. This ensures the ONNX model sees the same actuator dynamics it was trained on.

Controller Manager
------------------

- Runs control loop at 50 Hz
- Manages lifecycle of state_interfaces_broadcaster and LocomotionController

User Command Interface
----------------------

Publish geometry_msgs/Twist messages to the velocity command topic (default: ~/cmd_vel).

Example: To command forward walking at 0.5 m/s, publish Twist with linear.x = 0.5, linear.y = 0.0, angular.z = 0.0. The controller formats this as [0.5, 0.0, 0.0, 0.0] (lin_vel_x, lin_vel_y, ang_vel_z, heading) and includes it in the observation vector.


https://docs.google.com/document/d/1RVRAUgEUzKis0tg4bWb-FpZigZ4RG5k669Oa3MTUaKE/edit?usp=drive_link