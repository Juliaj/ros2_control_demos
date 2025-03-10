:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_17/doc/userdoc.rst

.. _ros2_control_demos_example_17_userdoc:

************************************************************************
MecanumWheelBot running with asynchronous controller and hardware
************************************************************************

In this example, we illustrate key concepts of the ros2_control framework, particularly the controller manager, asynchronous controllers, and asynchronous hardware interfaces. *MecanumWheelBot* is a simple mobile base using bicycle model with 4 wheels. 

An asynchronous controller is a controller that for some reason cannot (or we don't want to) perform the operations needed in an update() call.


For instance if ros control is running at 100Hz, the sum of the execution time of all controllers update() calls must be below 10ms. If a controller requires 15ms it cannot be executed synchronously without affecting the whole ros control update rate.

This example shows how to use the bicycle steering controller, which is a sub-design of the steering controller library.

Even though the robot has 4 wheels with front steering, the vehicle dynamics of this robot is similar to a bicycle. There is a virtual front wheel joint that is used to control the steering angle of the front wheels and the front wheels on the robot mimic the steering angle of the virtual front wheel joint. Similarly the rear wheels are controlled by a virtual rear wheel joint.

This example shows how to use the bicycle steering controller to control a carlike robot with 4 wheels but only 2 joints that can be controlled, one for steering and one for driving.

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.

The *MecanumWheelBot* URDF files can be found in ``ros2_control_demo_description/MecanumWheelBot/urdf`` folder.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

.. code-block:: shell

   # set delay 
   ros2 topic pub -1 /simulated_delay_max std_msgs/msg/Float32 "data: 50.0"

   # set delay 
   ros2 topic pub -1 /simulated_delay_min std_msgs/msg/Float32 "data: 50.0"

   # set exception
   ros2 topic pub -1 /trigger_exception std_msgs/msg/Bool "data: true"

Mecanum Robot Demonstration: Controller Manager and Asynchronous Components
==========================================================================

This demonstration showcases a robot with mecanum wheels to illustrate key concepts of the ros2_control framework, particularly the controller manager, asynchronous controllers, and asynchronous hardware interfaces.


Scenario 1: Synchronous Controller with Synchronous Hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This baseline scenario uses the default synchronous setup.

**Configuration:**

.. code-block:: yaml

   mecanum_controller:
     ros__parameters:
       is_async: false
       
   mecanum_robot_hardware:
     ros__parameters:
       is_async: false
       update_rate: 100.0  # Same as controller manager

**Behavior:**

* The controller manager thread runs at 100Hz
* In each cycle, the read-update-write operations happen sequentially
* If any operation takes too long, the entire control loop is delayed
* Suitable for simple robots with predictable computation times

**Implementation Tip:**

Monitor loop timing with:

.. code-block:: bash

   ros2 topic echo /controller_manager/statistics

Scenario 2: Asynchronous Controller with Synchronous Hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This scenario demonstrates how to offload complex controller calculations.

**Configuration:**

.. code-block:: yaml

   mecanum_controller:
     ros__parameters:
       is_async: true
       update_rate: 50.0  # Lower than the controller manager
       
   mecanum_robot_hardware:
     ros__parameters:
       is_async: false

**Behavior:**

* The hardware interface is read/written in the main controller manager thread
* The mecanum controller runs calculations in a separate thread at 50Hz
* The main control loop doesn't block waiting for controller calculations
* Useful when the controller performs complex operations like dynamic obstacle avoidance

**Implementation:**

Add processing delay to simulate complex controller calculations:

.. code-block:: cpp

   // In MecanumDriveController::update
   if (is_async_) {
     // Simulate complex computation in async controller
     std::this_thread::sleep_for(std::chrono::milliseconds(15));
   }

Scenario 3: Synchronous Controller with Asynchronous Hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This scenario demonstrates handling hardware with variable communication times.

**Configuration:**

.. code-block:: yaml

   mecanum_controller:
     ros__parameters:
       is_async: false
       
   mecanum_robot_hardware:
     ros__parameters:
       is_async: true
       update_rate: 200.0  # Higher than controller manager

**Behavior:**

* Hardware communication runs in its own thread at 200Hz
* The controller manager reads the latest data available but doesn't block waiting for hardware
* The mecanum controller executes in the main thread
* Useful for hardware with variable communication latencies (CAN bus, network-connected devices)

**Implementation:**

Create an asynchronous hardware thread:

.. code-block:: cpp

   // In MecanumRobotHardware::on_activate
   if (is_async_) {
     running_ = true;
     async_thread_ = std::thread([this]() {
       rclcpp::Rate rate(update_rate_);
       while (running_) {
         auto start = std::chrono::steady_clock::now();
         
         // Simulate variable communication time
         std::this_thread::sleep_for(
           std::chrono::milliseconds(random_between(1, 8)));
         
         {
           std::lock_guard<std::mutex> guard(io_mutex_);
           // Update hw_positions_ and hw_velocities_ based on hw_commands_
         }
         
         rate.sleep();
       }
     });
   }

Scenario 4: Fully Asynchronous System
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This scenario demonstrates a system with both asynchronous hardware and controllers.

**Configuration:**

.. code-block:: yaml

   mecanum_controller:
     ros__parameters:
       is_async: true
       update_rate: 50.0
       
   mecanum_robot_hardware:
     ros__parameters:
       is_async: true
       update_rate: 200.0
       thread_priority: 0  # Highest priority

**Behavior:**

* Hardware interface runs in highest priority thread (0) at 200Hz
* Mecanum controller runs in medium priority thread at 50Hz
* Controller manager thread coordinates but doesn't block on either component
* Allows for optimal resource utilization in complex systems

**Implementation Challenges:**

* Proper mutex handling to avoid deadlocks
* Clear priority hierarchy to prevent priority inversion
* Careful resource management to avoid thread contention

Best Practices and Recommendations
---------------------------------

**When to Use Synchronous Components**

* Simple systems with predictable computation times
* When deterministic timing is critical
* For controllers requiring tight coupling with hardware timing
* During initial development for easier debugging

**When to Use Asynchronous Controllers**

* For controllers with computationally intensive algorithms
* When the controller runs at a lower frequency than the hardware
* For non-critical controllers that can tolerate some timing variation
* To prevent complex controllers from blocking the main control loop

**When to Use Asynchronous Hardware**

* For hardware with variable communication latencies
* When hardware operates at a different frequency than controllers
* For devices requiring their own update rhythms (cameras, LIDAR)
* When hardware operations might occasionally block or timeout

**Priority Considerations**

* Hardware interfaces: Highest priority (0-10)
* Critical controllers: Medium-high priority (20-30)
* Controller manager: Medium priority (50)
* Non-critical controllers: Lower priority (60-80)

**Debugging Asynchronous Systems**

* Use controller manager statistics to monitor timing
* Add detailed logging with timing information
* Consider temporary synchronous operation during initial debugging
* Monitor thread CPU usage to identify bottlenecks

This demonstration provides a practical example of how the controller manager orchestrates both synchronous and asynchronous components in a mecanum wheel robot, illustrating the flexibility and power of the ros2_control framework.



Files used for this demos
--------------------------

* Launch file: `MecanumWheelBot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/bringup/launch/MecanumWheelBot.launch.py>`__
* Controllers yaml: `MecanumWheelBot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/bringup/config/MecanumWheelBot_controllers.yaml>`__
* URDF file: `MecanumWheelBot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/description/urdf/MecanumWheelBot.urdf.xacro>`__

  * Description: `MecanumWheelBot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/MecanumWheelBot/urdf/MecanumWheelBot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `MecanumWheelBot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/description/ros2_control/MecanumWheelBot.ros2_control.xacro>`__

* RViz configuration: `MecanumWheelBot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/MecanumWheelBot/rviz/MecanumWheelBot.rviz>`__

* Hardware interface plugin: `MecanumWheelBot_system.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/hardware/MecanumWheelBot_system.cpp>`__


Controllers from this demo
--------------------------

* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): :ref:`doc <joint_state_broadcaster_userdoc>`
* ``Mecanum Drive Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/mecanum_drive_controller>`__): :ref:`doc <mecanum_drive_controller_userdoc>`
