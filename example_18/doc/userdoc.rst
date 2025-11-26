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
   wget https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-1.23.2.tgz
   tar -xzf onnxruntime-linux-x64-1.23.2.tgz
   sudo cp -r onnxruntime-linux-x64-1.23.2/include/* /usr/local/include/
   sudo cp -r onnxruntime-linux-x64-1.23.2/lib/* /usr/local/lib/
   sudo ldconfig

   # Gazebo (gz-sim Harmonic or newer recommended)
   sudo apt-get update
   sudo apt-get install ros-${ROS_DISTRO}-ros-gz

- Place the trained policy at ``onnx_model/policy_biped_25hz_a.onnx``.
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

   # Check headers
   ls -l /usr/local/include/onnxruntime/core/session/onnxruntime_cxx_api.h

   # If not in /usr/local, check where you extracted
   ls -l ~/onnxruntime-linux-x64-*/lib/
   ls -l ~/onnxruntime-linux-x64-*/include/

   ls -l /usr/local/lib/libonnxruntime.so*
   ls -l /usr/local/include/onnxruntime/

   # check header files
   ls -l /usr/local/include/onnxruntime_cxx_api.h


Run the demo
------------

1. Optional URDF check

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 view_robot.launch.py \
        gui:=true use_gazebo:=false

2. Launch Gazebo and spawn the robot

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 example_18_gazebo.launch.py \
        gz_args:="-r -v 3 empty.sdf" bridge_clock:=true

   Set ``bridge_clock:=false`` if you do not need ``/clock``.

3. Bring up controllers

   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 example_18_gazebo.launch.py

      # Check controllers
      ros2 control list_controllers

4. Command and observe

   .. code-block:: bash

      # Send a 0.5 m/s forward command
      ros2 topic pub --once /locomotion_controller/cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.5}, angular: {z: 0.0}}"

      # Inspect state/broadcaster streams
      ros2 topic echo /interfaces_state_broadcaster/values --once
      ros2 topic echo /joint_states

Expected behaviour: the robot spawns balanced, follows velocity commands, and logs
warnings if the observation vector dimension deviates from ``10 + 3*N`` (``N=12``).

Testing
-------

Unit tests
~~~~~~~~~~

.. code-block:: bash

   cd ~/ros2_control_ws
   colcon build --packages-select ros2_control_demo_example_18 --cmake-args -DBUILD_TESTING=ON
   source install/setup.bash
   ros2 run ros2_control_demo_example_18 test_observation_formatter
   ros2 run ros2_control_demo_example_18 test_action_processor

- Observation formatter tests cover ordering, relative joint math, and dimension checks.
- Action processor tests confirm scale (0.25), offsets, and negative-value handling.

End-to-end validation
~~~~~~~~~~~~~~~~~~~~~

After the launch sequence above:

- Monitor ``/interfaces_state_broadcaster/values`` and ``/imu_sensor_broadcaster_1/imu`` to confirm sensor flow.
- Check ``ros2 topic echo /joint_states`` to ensure the forward position controller reflects ONNX outputs.
- Try alternative ``cmd_vel`` inputs (turning, reverse) to ensure stability.
- Review the controller logs at ``--log-level debug`` if the observation vector size mismatches the ONNX model.

Architecture
------------

.. code-block:: text

   [Gazebo Simulator]
       ↓ (joint states)
   [Hardware Interface]
       ↓ (state interfaces)
   [Interfaces State Broadcaster]
       ↓ (ROS2 topic: /interfaces_state_broadcaster/values)
   [ONNX Humanoid Controller]
       ↓ (ONNX inference)
       ↓ (ROS2 topic: /forward_position_controller/commands)
   [Forward Position Controller]
       ↓ (command interfaces)
   [Hardware Interface]
       ↓ (joint commands)
   [Gazebo Simulator]



## How We Determined the Observation Vector Dimension

The dimensions are determined by the source code and also [IsaacLab MDP documentation](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.mdp.html).


1. `generated_commands` (line 51-54):
   - Output: 4D when `heading_command=True`
   - From `UniformVelocityCommandCfg`: `[lin_vel_x, lin_vel_y, ang_vel_z, heading]`
   - Without heading: 3D `[lin_vel_x, lin_vel_y, ang_vel_z]`

```25:39:Berkeley-Humanoid-Lite/source/berkeley_humanoid_lite/berkeley_humanoid_lite/tasks/locomotion/velocity/config/biped/env_cfg.py
base_velocity = mdp.UniformVelocityCommandCfg(
    resampling_time_range=(10.0, 10.0),
    debug_vis=True,
    asset_name="robot",
    heading_command=True,
    heading_control_stiffness=0.5,
    rel_standing_envs=0.02,
    rel_heading_envs=1.0,
    ranges=mdp.UniformVelocityCommandCfg.Ranges(
        lin_vel_x=(-0.5, 0.5),
        lin_vel_y=(-0.25, 0.25),
        ang_vel_z=(-1.0, 1.0),
        heading=(-math.pi, math.pi),
    ),
)
```

2. `base_ang_vel` (line 55-58):
   - Output: 3D
   - [mdp.base_ang_vel](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.mdp.html#isaaclab.envs.mdp.base_ang_vel) returns torch.Tensor with shape (3,)
   - Returns base angular velocity in world frame: `[ang_vel_x, ang_vel_y, ang_vel_z]` (roll, pitch, yaw rates)

3. `projected_gravity` (line 59-62):
   - Output: 3D
   - [mdp.projected_gravity](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.mdp.html#isaaclab.envs.mdp.projected_gravity) returns torch.Tensor with shape (3,)
   - Returns gravity vector projected into body frame: `[g_x, g_y, g_z]`

4. `joint_pos_rel` (line 63-67):
   - output: N dimensions (one per joint in `HUMANOID_LITE_LEG_JOINTS`) 
   - [mdp.joint_pos_rel](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.mdp.html#isaaclab.envs.mdp.joint_pos_rel) returns torch.Tensor with shape (N,)

5. `joint_vel_rel` (line 68-72):
   - output: N dimensions (one per joint in `HUMANOID_LITE_LEG_JOINTS`)
   - [mdp.joint_vel_rel](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.mdp.html#isaaclab.envs.mdp.joint_vel_rel) returns torch.Tensor with shape (N,)

6. `last_action` (line 73):
   - output: N dimensions (one per joint in `HUMANOID_LITE_LEG_JOINTS`)
   - [mdp.last_action](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.mdp.html#isaaclab.envs.mdp.last_action) returns torch.Tensor with shape (N,)

Your ROS2 implementation matches these dimensions: 4 + 3 + 3 + N + N + N = 10 + 3N.

Debugging Dimension Mismatch Errors
-------------------------------------

If you encounter "Got invalid dimensions for input: obs" error:

1. Check ONNX model metadata: The controller prints detailed model information on startup:
   - Look for ``=== ONNX Model Input Metadata ===`` section
   - Shows input name, shape, and data type
   - Example: ``Input[0]: name='obs', shape=[1, 46], type=float32``
   - Also prints output metadata: ``=== ONNX Model Output Metadata ===``

2. Compare with config: The controller compares model expectations with ``env_cfg.py``:
   - Expected: ``10 + 3*N`` where N = number of joints
   - Shows breakdown: ``4 (velocity_commands) + 3 (base_ang_vel) + 3 (projected_gravity) + N (joint_pos) + N (joint_vel) + N (previous_action)``

3. Verify observation size: The controller validates observation size. Check logs for:
   - ``Observation size mismatch: got X, expected Y``
   - Component breakdown showing which part failed

4. Common issues:
   - Model input name: Check if model expects ``'obs'`` or different name (printed in metadata)
   - Previous action size: Ensure ``previous_action_`` is initialized with ``joint_names_.size()`` elements
   - Joint count mismatch: Verify number of joints matches model training (12 for biped)
   - Model shape: ONNX models may expect ``[1, 46]`` (with batch) or ``[46]`` (without batch)
   - Dynamic dimensions: Models with ``-1`` in input shape are handled automatically

5. Debug steps:
   - Check startup logs for complete model metadata (input/output names, shapes, types)
   - Enable debug logging: ``--log-level debug`` for runtime observation validation
   - Verify all sensor data is being received (IMU, joints)
   - Ensure default joint positions are initialized before first inference
   - Compare model's expected input shape with ``env_cfg.py`` observation configuration

For example, if there are 12 leg joints, the observation vector dimension will be 10 + 3*12 = 46.
