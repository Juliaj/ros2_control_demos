# Steps to Enable Base Linear Velocity for Fine-Tuning

## Overview

This guide walks through the steps to enable base linear velocity collection and use it for proper reward computation during fine-tuning.

## Prerequisites

- ROS2 workspace built with `mujoco_ros2_control` package
- Open Duck Playground repository available
- MuJoCo simulation environment set up

## Step-by-Step Instructions

### Step 1: Rebuild ROS2 Workspace

The velocimeter sensor support was added to `mujoco_ros2_control`. You need to rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control
source install/setup.bash
```

**Verify**: Check that the build succeeded and no errors related to `VelocimeterSensorData` or `velocimeter_sensor_data_`.

### Step 2: Verify Velocimeter Sensor in ROS2 Control

Check that the velocimeter sensor is properly configured:

```bash
# Check the xacro file includes velocimeter sensor
grep -A 5 "velocimeter" ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/description/ros2_control/open_duck_mini.ros2_control.xacro

# Check the broadcaster config includes velocimeter interfaces
grep -A 3 "velocimeter" ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/bringup/config/open_duck_mini_controllers_data_collection.yaml
```

**Expected**: You should see:
- Velocimeter sensor definition in xacro with 3 state interfaces (x, y, z)
- Velocimeter interfaces in broadcaster config (indices 40-42)

### Step 3: Launch Data Collection

Start the ROS2 control system in data collection mode:

```bash
cd ~/ros2_ws
ros2 launch ros2_control_demo_example_18 data_collection.launch.py
```

**Note**: MuJoCo simulation is started automatically. If you want a separate GUI window, you can optionally start it first (see fine_tuning_plan.rst for details).

### Step 4: Verify Velocimeter Data is Available

In a separate terminal, check that velocimeter data is being published:

```bash
# Check state interfaces broadcaster topic
ros2 topic echo /state_interfaces_broadcaster/values --once | head -50
```

**Expected**: The topic should have at least 43 values (10 IMU + 28 joints + 2 contacts + 3 velocimeter).

**Verify velocimeter values** (indices 40-42):
```bash
ros2 topic echo /state_interfaces_broadcaster/values --once | \
  python3 -c "import sys; data = eval(sys.stdin.read().split('data:')[1].split('}')[0]); print(f'Velocimeter: x={data[40]:.4f}, y={data[41]:.4f}, z={data[42]:.4f}')"
```

**Expected**: Non-zero values when robot is moving, near-zero when stationary.

### Step 5: Publish Velocity Commands (Optional)

If you want to control the robot during data collection:

```bash
cd ~/ros2_ws
python3 src/ros-controls/ros2_control_demos/example_18/fine_tuning/publish_velocity_commands.py \
  --lin-vel-x 0.15 \
  --duration 300
```

Or use the data collection script's auto-publish feature (see Step 6).

### Step 6: Collect Data with Velocimeter Support

Run the data collection script. It will automatically extract velocimeter data:

```bash
cd ~/ros2_ws
python3 src/ros-controls/ros2_control_demos/example_18/fine_tuning/collect_ros2_data.py \
  --output ros2_data.h5 \
  --duration 300 \
  --publish-commands \
  --auto-publish-velocity \
  --velocity-lin-x 0.15
```

**What it does**:
- Subscribes to `/state_interfaces_broadcaster/values`
- Extracts velocimeter data (indices 40-42)
- Saves to HDF5 file as `base_linear_velocities` dataset
- Publishes joint commands using reference motion
- Auto-publishes velocity commands if enabled

**Verify data collection**:
- Check console output for "Velocimeter data not available" warning (should NOT appear)
- Script should log successful data collection

### Step 7: Verify Collected Data

Check that the HDF5 file contains base linear velocities:

```bash
python3 << EOF
import h5py
import numpy as np

with h5py.File('ros2_data.h5', 'r') as f:
    print("Datasets:", list(f.keys()))
    if 'base_linear_velocities' in f:
        velocities = f['base_linear_velocities'][:]
        print(f"\nBase linear velocities shape: {velocities.shape}")
        print(f"Sample (first 5):\n{velocities[:5]}")
        print(f"Statistics:")
        print(f"  Mean: {np.mean(velocities, axis=0)}")
        print(f"  Std:  {np.std(velocities, axis=0)}")
        print(f"  Min:  {np.min(velocities, axis=0)}")
        print(f"  Max:  {np.max(velocities, axis=0)}")
    else:
        print("\nWARNING: base_linear_velocities not found in file!")
        print("This means velocimeter data was not collected.")
EOF
```

**Expected**:
- `base_linear_velocities` dataset exists
- Shape: `(num_samples, 3)` for (x, y, z)
- Values are finite and reasonable (e.g., x: -0.5 to 0.5 m/s, z: near 0)

### Step 8: Train with Replay Environment

Use the replay environment which now uses actual base linear velocity:

```bash
cd ~/dev/Open_Duck_Playground
uv run playground/open_duck_mini_v2/runner.py \
  --env ros2_replay \
  --data_path ~/ros2_ws/ros2_data.h5 \
  --restore_checkpoint_path /home/juliajia/dev/Open_Duck_Playground/checkpoints/2025_12_26_165635_300482560 \
  --num_timesteps 5000000 \
  --use_jax_to_onnx
```

**What to watch for**:
- Console should show: "Loaded X base linear velocity measurements"
- Reward should NOT be constant (should vary with tracking quality)
- Training should show learning progress (reward improving over time)

### Step 9: Verify Reward Computation

During training, check that rewards are varying (not constant):

```bash
# Watch training output for reward values
# Before (without velocimeter): reward: 228.998, reward_std: 0.0 (constant)
# After (with velocimeter): reward: varies, reward_std: > 0 (learning signal)
```

**Expected behavior**:
- `tracking_lin_vel` reward component varies (not always 1.0)
- Overall reward changes as model learns
- `reward_std` > 0 indicates learning signal

### Step 10: Test Deployed Model

After training, test the fine-tuned model in ROS2:

```bash
cd ~/ros2_ws
ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py
```

**Expected**: Robot should walk forward properly (not shuffle in place).

## Troubleshooting

### Issue: "Velocimeter data not available" warning

**Cause**: Velocimeter sensor not properly configured or data not in expected indices.

**Solution**:
1. Verify Step 2 (velocimeter in xacro and broadcaster config)
2. Check that state_interfaces_broadcaster is publishing 43+ values
3. Verify MuJoCo model has `local_linvel` sensor defined

### Issue: base_linear_velocities all zeros

**Cause**: Robot not moving, or velocimeter sensor not reading correctly.

**Solution**:
1. Ensure robot is actually moving (publish velocity commands)
2. Check velocimeter sensor values in Step 4
3. Verify MuJoCo sensor is attached to correct site (should be "imu" site)

### Issue: Constant reward during training

**Cause**: Replay environment not using velocimeter data, or old data file without velocimeter.

**Solution**:
1. Verify Step 7 (data file has base_linear_velocities)
2. Check replay environment logs for "Loaded X base linear velocity measurements"
3. Re-collect data if using old file without velocimeter data

### Issue: Build errors for velocimeter

**Cause**: Missing includes or incorrect code changes.

**Solution**:
1. Verify all files were modified correctly (see fine_tuning_plan.rst)
2. Check for typos in `VelocimeterSensorData` or `velocimeter_sensor_data_`
3. Clean build: `colcon build --cmake-clean-cache --packages-select mujoco_ros2_control`

## Summary Checklist

- [ ] Step 1: Rebuilt mujoco_ros2_control package
- [ ] Step 2: Verified velocimeter sensor configuration
- [ ] Step 3: Launched data collection successfully
- [ ] Step 4: Verified velocimeter data in ROS2 topic
- [ ] Step 5: Published velocity commands (optional)
- [ ] Step 6: Collected data with velocimeter support
- [ ] Step 7: Verified base_linear_velocities in HDF5 file
- [ ] Step 8: Started training with replay environment
- [ ] Step 9: Verified rewards are varying (not constant)
- [ ] Step 10: Tested deployed model in ROS2

## Next Steps

After completing these steps:
1. Monitor training progress (rewards should improve)
2. Export ONNX model when training completes
3. Deploy and test in ROS2 environment
4. Compare performance: before (constant reward) vs. after (learning signal)

