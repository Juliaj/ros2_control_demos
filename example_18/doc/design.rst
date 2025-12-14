Design
======

Scenario
--------

User issues a velocity command to the Humanoid Lite biped robot using geometry_msgs/Twist messages. The robot uses an ONNX ML model to generate joint position commands that achieve the desired locomotion.

Architecture
------------

The system consists of two main components:

1. interfaces_state_broadcaster: Aggregates hardware state interfaces into a control_msgs/InterfacesValues topic, including IMU data and joint states.

2. LocomotionController: A ros2_control controller that subscribes to sensor data and velocity commands, formats observations, runs ONNX inference, and writes joint position commands to hardware.

Data Flow
---------

.. code-block:: text

   Hardware State Interfaces
       ↓
   interfaces_state_broadcaster → control_msgs/InterfacesValues topic
       ↓
   LocomotionController
       ↓ (ONNX inference)
   Hardware Command Interfaces

LocomotionController
--------------------

Update Rate: 25 Hz

Inputs:

- Sensor data from interfaces_state_broadcaster/values topic:
  - Base angular velocity (from IMU)
  - Projected gravity vector (from IMU orientation)
  - Joint positions and velocities

- Velocity commands from cmd_vel topic (geometry_msgs/Twist)

- Previous action (stored internally)

Processing:

- Format observation vector for ONNX model
- Run inference using policy_biped_25hz_a.onnx
- Process model outputs (scale, clamp to limits)
- Write joint position commands to hardware

Observation Vector Format
-------------------------

The observation vector is concatenated in this order:

1. Velocity commands (3D: lin_vel_x, lin_vel_y, ang_vel_z)
2. Base angular velocity (3D vector from IMU)
3. Projected gravity vector (3D vector from IMU orientation)
4. Joint positions (N joints, relative to default positions)
5. Joint velocities (N joints, absolute)
6. Previous action (N joints)

Total dimension: 9 + 3*N (where N = number of leg joints)

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

Controller Manager
------------------

- Runs control loop at 25 Hz
- Manages lifecycle of interfaces_state_broadcaster and LocomotionController

User Command Interface
----------------------

Publish geometry_msgs/Twist messages to the velocity command topic (default: ~/cmd_vel).

Example: To command forward walking at 0.5 m/s, publish Twist with linear.x = 0.5, linear.y = 0.0, angular.z = 0.0. The controller formats this as [0.5, 0.0, 0.0] (lin_vel_x, lin_vel_y, ang_vel_z) and includes it in the observation vector.
