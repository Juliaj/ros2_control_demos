:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_17/doc/userdoc.rst

.. _ros2_control_demos_example_17_userdoc:

************
MecanumWheelBot
************

*MecanumWheelBot* is a simple mobile base using bicycle model with 4 wheels.

This example shows how to use the bicycle steering controller, which is a sub-design of the steering controller library.

Even though the robot has 4 wheels with front steering, the vehicle dynamics of this robot is similar to a bicycle. There is a virtual front wheel joint that is used to control the steering angle of the front wheels and the front wheels on the robot mimic the steering angle of the virtual front wheel joint. Similarly the rear wheels are controlled by a virtual rear wheel joint.

This example shows how to use the bicycle steering controller to control a carlike robot with 4 wheels but only 2 joints that can be controlled, one for steering and one for driving.

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.

The *MecanumWheelBot* URDF files can be found in ``ros2_control_demo_description/MecanumWheelBot/urdf`` folder.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *MecanumWheelBot* description is working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_17 view_robot.launch.py

   .. warning::
    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node needs some time to start.

   .. image:: MecanumWheelBot.png
    :width: 400
    :alt: Carlike Mobile Robot

2. To start *MecanumWheelBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_17 mecanumwheelbot.launch.py remap_odometry_tf:=true

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In the starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This excessive printing is only added for demonstration. In general, printing to the terminal should be avoided as much as possible in a hardware interface implementation.

   If you can see an orange box with four wheels in *RViz* everything has started properly.

   .. note::

    For robots being fixed to the world frame, like the RRbot examples of this repository, the ``robot_state_publisher`` subscribes to the ``/joint_states`` topic and creates the TF tree. For mobile robots, we need a node publishing the TF tree including the pose of the robot in the world coordinate systems. The most simple one is the odometry calculated by the ``bicycle_steering_controller``.

   By default, the controller publishes the odometry of the robot to the ``~/tf_odometry`` topic. The ``remap_odometry_tf`` argument is used to remap the odometry TF to the ``/tf`` topic. If you set this argument to ``false`` (or not set it at all) the TF tree will not be updated with the odometry data.

3. Now, let's introspect the control system before moving *MecanumWheelBot*. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   You should get

   .. code-block:: shell



   The ``[claimed]`` marker on command interfaces means that a controller has access to command *MecanumWheelBot*.

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   You should get

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    bicycle_steering_controller[bicycle_steering_controller/BicycleSteeringController] active

5. If everything is fine, now you can send a command to *bicycle_steering_controller* using ROS 2 CLI:

   .. code-block:: shell

    ros2 topic pub --rate 30 /mecanum_drive_controller/reference geometry_msgs/msg/TwistStamped "
      header:
        stamp:
          sec: $(date +%s)
          nanosec: 0
      twist:
        linear:
          x: 0.7
          y: 0.0
          z: 0.0
        angular:
          x: 0.0
          y: 0.0
          z: 1.0"

   You should now see an orange box circling in *RViz*.
   Also, you should see changing states in the terminal where launch file is started.

   .. code-block:: shell



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
