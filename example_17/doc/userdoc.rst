:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_17/doc/userdoc.rst

.. _ros2_control_demos_example_17_userdoc:

************************************************************************
MecanumWheelBot running with asynchronous controller and hardware
************************************************************************

In this example, we illustrate key concepts of the ros2_control framework, particularly the controller manager, asynchronous controllers, and asynchronous hardware interfaces. *MecanumWheelBot* is a simple mobile base using 4 mecanum wheels. 

The *MecanumWheelBot* URDF files can be found in ``ros2_control_demo_description/MecanumWheelBot/urdf`` folder.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

Scenario 1: Asynchronous Controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**Configuration:**

.. code-block:: yaml

   mecanum_controller:
     ros__parameters:
      is_async: true

      simulated_delay_min: 45.0
      simulated_delay_max: 48.0
      trigger_exception: false
       

The controller is configured to run asynchronously with confiurable delay which you can set by publishing to the following topics:

.. code-block:: shell

   # set delay 
   ros2 topic pub -1 /simulated_delay_max std_msgs/msg/Float32 "data: 50.0"

   # set delay 
   ros2 topic pub -1 /simulated_delay_min std_msgs/msg/Float32 "data: 50.0"

   # set exception
   ros2 topic pub -1 /trigger_exception std_msgs/msg/Bool "data: true"

In addition, you can trigger an exception by publishing to the ``/trigger_exception`` topic.

Scenario 2: Asynchronous Controller with Asynchronous Hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**Configuration:**

.. code-block:: yaml

  mecanum_controller:
     ros__parameters:
      is_async: true

      simulated_delay_min: 45.0
      simulated_delay_max: 48.0
      trigger_exception: false
  
  mecanum_robot_hardware:
    ros__parameters:
      is_async: true



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
