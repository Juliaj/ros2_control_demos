:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_17/doc/userdoc.rst

.. _ros2_control_demos_example_17_userdoc:

********************************************************
DiffBot with Gazebo Simulation World.
********************************************************

This example shows how to control a custom differential drive robot (diffbot) using ``gz_ros2_control::GazeboSimROS2ControlPlugin`` and Gazebo built-in ``gz::sim::systems::DiffDrive`` plugin in Rubicon world.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To start *DiffBot* example in Rubicon world with ``gz_ros2_control::GazeboSimROS2ControlPlugin`` plugin, open a terminal, source your ROS2-workspace and execute its launch file with

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_17 diffbot_rubicon.launch.py


  The launch file loads and starts the robot hardware, and opens *Gazebo GUI*. You may need to zoom out to see the robot in *Gazebo GUI*. 
  

2. To drive around the robot, open another terminal and execute

  .. code-block:: shell

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

  Follow the instructions to move the robot using keyboard. Rubicon world is a difficult terrain so you may need to drive slowly and press `k` time to time to stop the robot. 


3. To start *DiffBot* example with Gazebo ``gz::sim::systems::DiffDrive`` plugin, stop the commands from previous steps. Then run following command

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_17 diffbot_rubicon.launch.py sim_plugin:=gz_sim


  The launch file loads and starts the robot hardware, and opens *Gazebo GUI*. You may need to zoom out to see the robot in *Gazebo GUI*. 
  
4. To drive around the robot, from Gazebo GUI, search and add ``Teleop`` plugin. Under ``Teleop`` plugin, click on ``KEYBOARD``, then follow the instructions to move the robot. 


Files used for this demo
--------------------------

* Launch file: `diffbot_rubicon.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/bringup/launch/diffbot_rubicon.launch.py>`__
* Controllers yaml: `diffbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/bringup/config/diffbot_controllers.yaml>`__
* Robot description: `model.sdf <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/description/models/costar_husky/model.sdf>`__ with `gz_ros2_control` plugin
* Robot description: `model_gz.sdf <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/description/models/costar_husky/model_gz.sdf>`__ with Gazebo `diff_drive_controller` plugin
* Gazebo Simulation World: `rubicon.sdf <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/worlds/rubicon.sdf>`__

Controllers from this demo
--------------------------

* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): :ref:`doc <joint_state_broadcaster_userdoc>`
* ``Diff Drive Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/diff_drive_controller>`__): :ref:`doc <diff_drive_controller_userdoc>`
