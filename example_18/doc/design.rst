Design
======

Scenario
--------

User issues a command to the Humanoid Lite biped robot to walk forward at
0.5 m/s using ``geometry_msgs/Twist`` messages.

Implementation Design
---------------------

Architecture
~~~~~~~~~~~~

The implementation relies on two main components:

1. ``interfaces_state_broadcaster`` aggregates every hardware state interface
   into the ``control_msgs/InterfacesValues`` topic so downstream consumers
   receive a consistent snapshot, including IMU data (orientation, angular
   velocity, linear acceleration) and joint states.
2. LocomotionController (a ros2_control controller plugin) is the core component:
   it subscribes to the broadcaster topic, formats observations, performs ONNX
   inference, and writes the resulting joint position targets back to the hardware
   command interfaces.

Data Flow
~~~~~~~~~

.. code-block:: text

   Hardware State Interfaces
       ↓ (read)
   interfaces_state_broadcaster → control_msgs/InterfacesValues topic
       ↓ (subscribe)
   LocomotionController in controller_manager
       ↓ (ML inference)
   Hardware Command Interfaces (joint position commands)

LocomotionController
~~~~~~~~~~~~~~~~~~~~

Update Rate: 25 Hz (configured in controller YAML).

Inputs:

- Sensor data: subscribe to ``interfaces_state_broadcaster/values`` topic

  - All interface states including:
    - Base angular velocity (from IMU)
    - Projected gravity vector (from IMU orientation)
    - Joint positions and velocities

- User command: subscribe to the velocity command topic
  (e.g., ``geometry_msgs/Twist``)
- Previous action: stored as internal state (member variable)

Processing:

- Format inputs for the ML model (ONNX)
- Run model inference using ``policy_biped_25hz_a.onnx``

Model Input Vector Format (order preserved, based on
`env_cfg.py <https://github.com/HybridRobotics/Berkeley-Humanoid-Lite/blob/main/source/berkeley_humanoid_lite/berkeley_humanoid_lite/tasks/locomotion/velocity/config/biped/env_cfg.py>`_):

The observation vector is concatenated in the following order:

1. Velocity commands (4D: ``lin_vel_x``, ``lin_vel_y``, ``ang_vel_z``, heading)
2. Base angular velocity (3D vector from IMU)
3. Projected gravity vector (3D vector from IMU orientation)
4. Joint positions (N joints, relative to default positions)
5. Joint velocities (N joints, relative)
6. Previous action (N joints)

Total dimension: ``10 + 3*N`` (where N = number of leg joints).


Gazebo Integration
~~~~~~~~~~~~~~~~~~

- The ``gz_ros2_control/GazeboSimSystem`` plugin (configured when
  ``use_gazebo:=true``) handles all Gazebo communication:

  - Reads joint positions/velocities from Gazebo simulation
  - Reads IMU data (orientation, angular velocity, linear acceleration) from
    Gazebo IMU sensors
  - Writes joint position commands to Gazebo joint actuators

- Hardware interface exposes IMU state interfaces (orientation,
  ``angular_velocity``, ``linear_acceleration``) as defined in the ros2_control
  xacro, which are aggregated by interfaces_state_broadcaster
- Store raw model outputs (not processed) as previous action for the next
  iteration

Hardware Layer
~~~~~~~~~~~~~~

- Lightweight implementation: only manages state and command interfaces
- Exposes state interfaces for:

  - IMU sensor (orientation, angular velocity, linear acceleration)
  - Joint positions and velocities

- Accepts command interfaces for joint positions

Controller Manager
~~~~~~~~~~~~~~~~~~

- Runs the control loop at 25 Hz
- Manages the lifecycle of all controllers (interfaces_state_broadcaster and LocomotionController)

User Command Interface
~~~~~~~~~~~~~~~~~~~~~~

Topic-based: publish ``geometry_msgs/Twist`` messages to the velocity command
topic (default: ``~/cmd_vel``).

Example: To command forward walking at 0.5 m/s:

- Publish ``Twist`` with ``linear.x = 0.5``, ``linear.y = 0.0``,
  ``angular.z = 0.0``
- Controller subscribes and formats as velocity command
  ``[0.5, 0.0, 0.0, 0.0]`` (``lin_vel_x``, ``lin_vel_y``, ``ang_vel_z``,
  heading)
- This becomes the first four elements of the observation vector fed to the ML
  model
- The model generates joint position commands to achieve the desired velocity
