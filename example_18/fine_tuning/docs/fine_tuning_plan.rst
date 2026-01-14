Fine-Tuning Plan for ROS2 Deployment
====================================

Overview
--------

This document outlines the plan for fine-tuning MuJoCo-trained models for deployment in ROS2 control environment (example_18). Models trained in MuJoCo simulation fail in ROS2 due to observation distribution shift, but can be adapted through fine-tuning on ROS2-compatible data.

Problem Statement
-----------------

Models trained in MuJoCo simulation do not work correctly when deployed in ROS2 control due to:

1. **Distribution Shift**: ROS2 observations have different initialization, timing, and sensor formatting compared to MuJoCo training data
2. **Initialization Mismatch**: Different initial states (actions, joint positions, sensor readings)
3. **Temporal Misalignment**: Observations sampled at different times relative to control updates

Current Status
--------------

- **Robot**: Same robot (open_duck_mini_v2) used in both MuJoCo and ROS2
- **Task**: Same task (bipedal walking)
- **Model Validation**: MuJoCo-trained ONNX model validated in MuJoCo simulation
- **ROS2 Test**: Model tested in ROS2 - shuffles in place instead of walking forward
- **Assessment**: Model has learned relevant locomotion skills but needs adaptation to ROS2 observation distribution

Fine-Tuning Approach
--------------------

**Strategy**: Fine-tune MuJoCo-trained model on ROS2-compatible data to adapt to ROS2 observation distribution while preserving learned locomotion skills.

**Why This Works**:
- Same robot and task → policy structure is relevant
- Model has learned locomotion skills (evidenced by shuffling behavior, not falling)
- Fine-tuning on ROS2 data adapts to distribution shift
- More efficient than training from scratch

Data Collection
---------------

**Method**: Collect (observation, action) pairs from ROS2 using reference motion instead of ONNX model.

**Approach**: Option 2 - Separate Controller
- Disable ``motion_controller`` (ONNX-based)
- Enable ``forward_command_controller`` (direct joint position commands)
- Python script controls robot using reference motion

**Steps**:

1. **Start MuJoCo Simulation**
   .. code-block:: bash

      cd ~/ros2_ws
      ${MUJOCO_DIR}/bin/simulate src/ros-controls/ros2_control_demos/example_18/description/mujoco/scene.xml

2. **Launch ROS2 Control (Data Collection Mode)**
   .. code-block:: bash

      ros2 launch ros2_control_demo_example_18 fine_tuning/data_collection.launch.py

   This launches:
   - ``state_interfaces_broadcaster`` (for observations)
   - ``forward_command_controller`` (for joint commands)
   - **NOT** ``motion_controller`` (disabled for data collection)

3. **Publish Velocity Commands (Optional)**
   .. code-block:: bash

      # Simple publisher
      python3 fine_tuning/publish_velocity_commands.py --lin-vel-x 0.15 --duration 300

      # Or use existing test_motions.py
      python3 $(ros2 pkg prefix ros2_control_demo_example_18)/share/ros2_control_demo_example_18/launch/test_motions.py

4. **Run Data Collection Script**
   .. code-block:: bash

      # Collect data for 5 minutes (300 seconds)
      python3 fine_tuning/collect_ros2_data.py --output ros2_data.h5 --duration 300 --publish-commands

   The script:
   - Subscribes to ``/state_interfaces_broadcaster/values`` (observations)
   - Subscribes to ``/motion_controller/cmd_vel`` (velocity commands)
   - Generates actions using reference motion (polynomial_coefficients.pkl)
   - Publishes joint commands to ``/forward_command_controller/commands``
   - Saves observations and actions to HDF5 file

5. **Data Format**
   - Observations: Same format as training (IMU, joints, velocity commands, action history)
   - Actions: Reference motion joint positions (14D)
   - Metadata: Velocity commands, timestamps, base linear velocities (from velocimeter sensor)

**Files**: ``fine_tuning/collect_ros2_data.py``, ``fine_tuning/data_collection.launch.py``

Base Linear Velocity (Velocimeter Sensor)
------------------------------------------

**Problem**: Without actual base linear velocity measurements, the velocity tracking reward becomes constant because it compares the velocity command to itself (using command as proxy), providing no learning signal.

**Solution**: Added velocimeter sensor support to expose base linear velocity from MuJoCo's ``local_linvel`` sensor as ROS2 state interfaces.

**Implementation**:

1. **MuJoCo Hardware Interface** (``mujoco_ros2_control``):
   - Added ``VelocimeterSensorData`` struct to store 3D linear velocity
   - Registered velocimeter sensors in ``register_sensors()``
   - Exported state interfaces: ``velocimeter/linear_velocity.{x,y,z}``
   - Reads velocimeter data from MuJoCo sensor data in ``read()`` method

2. **ROS2 Control Configuration**:
   - Added velocimeter sensor to ``open_duck_mini.ros2_control.xacro``
   - Added velocimeter interfaces to ``state_interfaces_broadcaster`` config (indices 40-42)

3. **Data Collection** (``collect_ros2_data.py``):
   - Extracts base linear velocity from state interfaces (indices 40-42)
   - Saves to HDF5 file as ``base_linear_velocities`` dataset
   - Falls back to zeros if velocimeter data not available (backward compatible)

4. **Replay Environment** (``ros2_replay.py``):
   - Loads ``base_linear_velocities`` from HDF5 file
   - Uses actual measured velocity for reward computation instead of command proxy
   - Enables proper velocity tracking reward: compares desired vs. actual velocity

**Why This Matters**:

The velocity tracking reward measures how well the robot achieves the desired locomotion velocity:

- **Without velocimeter**: Reward compares command to command → constant reward → no learning signal
- **With velocimeter**: Reward compares command to actual velocity → varies with tracking quality → enables learning

Even though we send **position commands** to joints, the **velocity tracking reward** measures the high-level locomotion goal. The neural network learns to coordinate joint positions to achieve the desired base velocity.

**Files Modified**:
- ``mujoco_ros2_control/include/mujoco_ros2_control/data.hpp``
- ``mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system_interface.hpp``
- ``mujoco_ros2_control/src/mujoco_system_interface.cpp``
- ``example_18/description/ros2_control/open_duck_mini.ros2_control.xacro``
- ``example_18/bringup/config/open_duck_mini_controllers_data_collection.yaml``
- ``example_18/fine_tuning/collect_ros2_data.py``
- ``playground/open_duck_mini_v2/ros2_replay.py``

Training Setup
--------------

**Environment**: Create ROS2 replay environment that wraps collected data.

**Steps**:

1. **Create Replay Environment**
   - New file: ``playground/open_duck_mini_v2/ros2_replay.py``
   - Implements same interface as ``Joystick``/``Standing`` (``reset``, ``step``, ``observation_size``, ``action_size``)
   - Loads collected ROS2 data
   - Replays data instead of running MuJoCo simulation
   - Computes rewards (same as training: velocity tracking, alive, etc.)

2. **Add to Runner**
   - Add ``"ros2_replay"`` to ``available_envs`` in ``runner.py``
   - Use existing training pipeline (PPO) and ONNX export

3. **Training Options**
   - **Option A**: Behavioral cloning (supervised learning on collected data)
   - **Option B**: RL fine-tuning (use dataset as initial policy, then PPO with replay env)
   - **Option C**: Dataset-based RL (train PPO using replay environment)

**Training Command**:
.. code-block:: bash

   uv run playground/open_duck_mini_v2/runner.py    \
    --env ros2_replay \
    --data_path ros2_data.h5 \
    --restore_checkpoint_path /home/juliajia/dev/Open_Duck_Playground/checkpoints/2025_12_26_165635_300482560 \
    --num_timesteps 5000000 \
    --use_jax_to_onnx

Deployment
----------

After fine-tuning:

1. Export trained model to ONNX (automatic during training)
2. Test in ROS2/example_18
3. Verify walking behavior (should walk forward, not shuffle)

Expected Results
----------------

- Model adapts to ROS2 observation distribution
- Preserves locomotion skills learned in MuJoCo
- Works correctly in ROS2 control environment
- Reduced training time compared to training from scratch

Files
-----

- **Data Collection Script**: ``fine_tuning/collect_ros2_data.py``
- **Data Collection Launch**: ``fine_tuning/data_collection.launch.py``
- **Velocity Command Publisher**: ``fine_tuning/publish_velocity_commands.py``
- **Replay Environment**: ``playground/open_duck_mini_v2/ros2_replay.py`` (to be created)
- **Reference Motion**: ``playground/open_duck_mini_v2/data/polynomial_coefficients.pkl``
- **Training Runner**: ``playground/open_duck_mini_v2/runner.py``

Related Documentation
--------------------

- ``mystash/mujoco_to_ros2_inference_differences.md``: Detailed analysis of differences
- ``doc/design.rst``: System architecture
- ``doc/userdoc.rst``: User documentation

