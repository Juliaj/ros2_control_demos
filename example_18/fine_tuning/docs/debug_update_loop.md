# new rounds of debugging

Problem: with Python inference code, Duck walks forward. With C++ motion controller, Duck doesn't walk forward, rather moves its feet in place. 

Intent/objective, send the same obs from Python code path to c++ motion controller update loop, extract the command before controller sends to hardware and compare it with the command from Python path.

## Data capture and code changes

### Data to capture from Python code path

Sensor data (from hardware/MuJoCo):
- IMU gyroscope (3D)
- IMU accelerometer (3D)
- Joint positions (N joints, absolute)
- Joint velocities (N joints, raw)
- Feet contact sensors (if available)

Controller internal state:
- Last action (N joints)
- Last-last action (N joints)
- Last-last-last action (N joints)
- Motor targets (N joints)
- Imitation phase counter (imitation_i)

State for hardware command generation:
- default_joint_positions (for action processor scaling + offset, also needed for observation formatting)
- prev_motor_targets (for rate limiting)
- onnx_active_steps and blend_in_steps (for blend factor calculation)
- max_motor_velocity and training_control_period (for rate limiting calculation)

Velocity command (7D: lin_x, lin_y, ang_z, head_1-4) — needed to format observation. Use lin_x=0.15, rest zeros.

### Code changes needed

1. Create service to inject sensor data and controller state:
   - Accept sensor data as Float64Values message (matching interface data format)
   - Accept controller state (action history, motor targets, phase counter, etc.)
   - Inject sensor data into rt_interface_data_ thread-safe box
   - Set controller internal state (motor_targets_, previous_action_, etc.)
   - Set observation formatter internal state (action history via update_action_history, phase via setter, motor targets)
   - Set smoothed_reference_action_ (read from parameter or initialize to default_joint_positions)
   - Return success when all state is set

2. Update reset_state service:
   - Also reset observation formatter state (action history, phase counter)

3. Service should return confirmation when injection is complete, before triggering update cycle.

4. Create Python script to:
   - Reset controller state to match Python code path (call reset_state service)
   - Call injection service with all captured data (sensor data, controller state)
   - Wait for service response confirming injection complete
   - Send velocity command (lin_x=0.15, rest zeros) to trigger update
   - Subscribe to debug topics and capture one update cycle output:
     * /motion_controller/debug/raw_action
     * /motion_controller/debug/processed_action
     * /motion_controller/debug/blended_action
     * /motion_controller/debug/rate_limited_action
   - Note: Controller runs at 50Hz (20ms period). May need to slow update rate or use synchronization to capture correct cycle.

## Testing procedure

1. Run Python data collection to generate and extract all required data with lin_x=0.15:
```
ros2 launch ros2_control_demo_example_18 data_collection.launch.py
```

```
model="/home/juliajia//dev/Open_Duck_Playground/checkpoints/2025_12_26_165635_300482560.onnx"
uv run python3 ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/fine_tuning/manual_control_ros2.py --output mujoco_manual_data.h5 --onnx-model $model
```

2. Generate the injection data YAML file and Python outputs YAML:
```
python3 ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/fine_tuning/create_injection_yaml.py \
  --h5-file mujoco_manual_data.h5 \
  --output injection_state.yaml \
  --lin-x 0.15 \
  --python-outputs
```
This creates two YAML files with datetime prefix:
- Injection state YAML (e.g., `20250130_143025_injection_state.yaml`)
- Python outputs YAML (e.g., `20250130_143025_python_outputs.yaml`)

3. Lower the controller manager and motion controller to run at 10Hz:
Update `open_duck_mini_controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 25  # Change from 50 to 25

```

4. Launch Duck in MuJoCo:
```
ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py
```

5. Run the injection and comparison script:
```
python3 ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/fine_tuning/inject_state_comparison.py \
  --yaml-file /home/juliajia/dev/Open_Duck_Playground/20251230_213814_injection_state.yaml \
  --python-outputs python_outputs.yaml
```
The script will:
- Reset controller state
- Publish YAML file path to `~/inject_yaml_path` topic
- Call injection service to inject state
- Send velocity command (lin_x=0.15)
- Capture debug topics from one update cycle
- Compare with Python path outputs from the provided YAML file

Note: The `--python-outputs` argument expects a YAML file with the following format:
```yaml
python_outputs:
  raw_action: [...]
  processed_action: [...]
  blended_action: [...]
  rate_limited_action: [...]
```
This file should contain the Python path outputs for the same step used in the injection YAML. 
