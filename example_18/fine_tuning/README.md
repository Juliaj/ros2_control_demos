# Fine-Tuning Data Collection

This directory contains scripts and launch files for collecting ROS2 data for fine-tuning MuJoCo-trained models.

## Overview

Models trained in MuJoCo simulation fail in ROS2 due to observation distribution shift. Fine-tuning on ROS2-compatible data adapts the model to ROS2 observations while preserving learned locomotion skills.

## Files

- `data_collection.launch.py`: Launch file that uses `forward_command_controller` instead of `motion_controller`
- `manual_control_ros2.py` : Manually drives robot via keys (e.g. up/down arrows etc.)

## Dependencies

Install required Python packages:

```bash
sudo apt install python3-h5py python3-numpy
```

Or via pip (if not using system packages):
```bash
pip install h5py numpy
```

## Quick Start

### 1. Launch ROS2 Control (Data Collection Mode)

**Note**: The launch file automatically starts MuJoCo simulation via `ros2_control_node`. You don't need to start MuJoCo separately unless you want a separate GUI window.

```bash
ros2 launch ros2_control_demo_example_18 data_collection.launch.py
```

This launches:
- `ros2_control_node` (MuJoCo simulation)
- `state_interfaces_broadcaster` (for observations)
- `forward_command_controller` (for joint commands)
- **NOT** `motion_controller` (disabled for data collection)

### 2. Collect Data

There are two options for data collection:

#### Option A: Manual Control with ONNX Model (`manual_control_ros2.py`)

Manually drive the robot using keyboard controls while the ONNX model generates actions:

```bash
# From source directory
cd ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18
python3 fine_tuning/manual_control_ros2.py \
    --onnx-model /path/to/model.onnx \
    --output ros2_manual_data.h5 \
    --num-joints 14

# Or use installed script
python3 $(ros2 pkg prefix ros2_control_demo_example_18)/share/ros2_control_demo_example_18/fine_tuning/manual_control_ros2.py \
    --onnx-model /path/to/model.onnx \
    --output ros2_manual_data.h5 \
    --num-joints 14
```

**Controls:**
- Arrow keys or WASD: Adjust velocity (Up/W: forward, Down/S: backward, Left/A: left, Right/D: right)
- Q/E: Turn left/right
- Space: Toggle data collection
- P: Pause/resume control
- R: Reset robot to home position
- O: Toggle ONNX model on/off
- Z: Zero velocity
- Ctrl+C: Exit and save

**The script will:**
- Subscribe to `/state_interfaces_broadcaster/values` (observations)
- Use ONNX model to generate actions based on observations
- Publish velocity commands to `/motion_controller/cmd_vel`
- Publish joint commands to `/forward_command_controller/commands`
- Save observations and actions to HDF5 file

#### Option B: Reference Motion (`collect_ros2_data.py`)

Use pre-recorded reference motion to drive the robot:

```bash
# From source directory
cd ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18
python3 fine_tuning/collect_ros2_data.py --output ros2_data.h5 --duration 300 --publish-commands

# Or use installed script
python3 $(ros2 pkg prefix ros2_control_demo_example_18)/share/ros2_control_demo_example_18/fine_tuning/collect_ros2_data.py --output ros2_data.h5 --duration 300 --publish-commands
```

**The script will:**
- Subscribe to `/state_interfaces_broadcaster/values` (observations)
- Subscribe to `/motion_controller/cmd_vel` (velocity commands)
- Generate actions using reference motion (polynomial_coefficients.pkl)
- Publish joint commands to `/forward_command_controller/commands`
- Save observations and actions to HDF5 file

## Data Format

The collected HDF5 file contains:
- `observations`: Observation vectors (same format as motion_controller)
  - **Important**: Should be 101 elements (includes all 14 joints: 10 legs + 4 head)
  - Format: [gyro(3), accel(3), cmd(7), joint_pos(14), joint_vel(14), last_act(14), last_last_act(14), last_last_last_act(14), motor_targets(14), contact(2), phase(2)]
  - If you see 77 elements instead of 101, the data only has 10 joints (legs only) - this won't match the checkpoint
- `actions`: Reference motion actions (14D joint positions)
- `velocity_commands`: Velocity commands used during collection
- `timestamps`: Collection timestamps

**Note**: The checkpoint expects 101-element observations with all 14 joints. Ensure `state_interfaces_broadcaster` in your launch config includes all 14 joints (see `bringup/config/open_duck_mini_controllers_data_collection.yaml`).

## Fine-Tuning with Replay Environment

After data collection, fine-tune your MuJoCo-trained model using the collected ROS2 data.

### 1. Prepare Data File

Copy or move your collected HDF5 file to the Open_Duck_Playground directory:

```bash
# If data was collected in example_18 directory
cp ~/ros2_ws/ros2_data.h5 \
   ~/dev/Open_Duck_Playground/ros2_data.h5

# Or use the existing data file if already there
```

### 2. Find the Base Checkpoint

**Important**: Fine-tuning requires a JAX/Brax checkpoint (not an ONNX model). ONNX models are for inference only and cannot be used for training.

If you have an ONNX model, find its source checkpoint:

**Option A: Check ONNX metadata file**
```bash
# If the ONNX model has a .metadata.json file, check it for checkpoint_path
cat /path/to/model.onnx.metadata.json | grep checkpoint_path

# Example output:
# "checkpoint_path": "/home/juliajia/dev/Open_Duck_Playground/checkpoints/2025_12_26_165635_300482560"
```

**Option B: Find checkpoint in checkpoints directory**
```bash
# Checkpoints are typically in Open_Duck_Playground/checkpoints/
ls ~/dev/Open_Duck_Playground/checkpoints/

# Look for directories matching the ONNX filename (without .onnx extension)
```

**For the example ONNX model** (`open_duck_mini_v2.onnx`):
```bash
# The checkpoint path from metadata is:
# /home/juliajia/dev/Open_Duck_Playground/checkpoints/2025_12_26_165635_300482560
```

### 3. Train with Replay Environment

Navigate to the Open_Duck_Playground directory and run training with the `ros2_replay` environment:

```bash
cd ~/dev/Open_Duck_Playground

# Fine-tune from a pretrained checkpoint
uv run playground/open_duck_mini_v2/runner.py \
    --env ros2_replay \
    --data_path ros2_data.h5 \
    --restore_checkpoint_path /home/juliajia/dev/Open_Duck_Playground/checkpoints/2025_12_26_165635_300482560 \
    --num_timesteps 5000000 \
    --use_jax_to_onnx

# Arguments:
# --env ros2_replay: Use the ROS2 replay environment
# --data_path: Path to collected HDF5 data file
# --restore_checkpoint_path: Path to pretrained JAX/Brax checkpoint directory (NOT ONNX file)
# --num_timesteps: Number of training timesteps (see recommendations below)
# --use_jax_to_onnx: Use JAX-to-ONNX export (recommended)

# Recommended --num_timesteps values:
# - Minimum: 1,000,000 (1M) - for quick adaptation with small datasets
# - Recommended: 5,000,000 (5M) - good balance for most cases
# - Extended: 10,000,000 (10M) - for larger datasets or more thorough fine-tuning
# - Maximum: 20,000,000 (20M) - rarely needed, risk of overfitting
#
# Note: Full training uses ~150M timesteps. Fine-tuning typically needs
# only 1-10% of that since the model already has learned skills.
```

The replay environment will:
- Load observations and actions from the collected ROS2 data
- Replay them during training to adapt the model to ROS2 observations
- Compute rewards based on the observations
- Export to ONNX automatically during training

### 4. Verify Replay Environment (Optional)

Before training, you can verify that the replay environment correctly preserves types for JAX compatibility:

```bash
cd ~/dev/Open_Duck_Playground

# Run unit tests for type preservation
uv run python -m unittest tests.test_ros2_replay_types -v

# This verifies:
# - episode_done and truncation maintain float32 dtype
# - done field remains boolean
# - Types remain consistent across multiple steps
```

This test ensures the environment is compatible with JAX's scan operations used during training.

### 5. Test Fine-Tuned Model in ROS2

After training, test the exported ONNX model in ROS2:

```bash
# The ONNX model will be saved in the checkpoints directory
# Use it with the motion_controller in ROS2
ros2 launch ros2_control_demo_example_18 bringup.launch.py
```

### Notes

- The replay environment (`playground/open_duck_mini_v2/ros2_replay.py`) has been created and integrated into the training pipeline
- Fine-tuning adapts the model to ROS2 observation distributions while preserving learned locomotion skills
- Training time is typically much shorter than training from scratch
- See `doc/fine_tuning_plan.rst` for additional details

