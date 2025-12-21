# Open Duck Mini

## Sources

- Meshes (assets/): Downloaded from https://github.com/apirrone/Open_Duck_Mini/tree/v2/mini_bdx/robots/open_duck_mini_v2
- MuJoCo Model (mujoco/): Adapted from https://github.com/apirrone/Open_Duck_Playground/tree/main/playground/open_duck_mini_v2/xmls
  - `open_duck_mini_v2.xml` - Main robot model with IMU sensor, tuned motor parameters from BAM identification
  - `sensors.xml` - IMU and foot sensor definitions
  - `joints_properties.xml` - STS3215 motor parameters (damping, kp, armature, forcerange)
  - `scene.xml` - Custom scene wrapper for ros2_control (100 Hz timestep)

## Structure

```
description/
├── assets/          - STL mesh files
├── mujoco/          - MuJoCo XML model files
├── urdf/            - URDF/xacro files for RViz visualization
└── ros2_control/    - ros2_control hardware interface configuration
```

## Modifications

### MuJoCo Model Integration

The Playground model (`open_duck_mini_v2.xml`) includes:
- IMU sensor site at position (-0.08, 0, 0.05) on base body
- Sensor definitions: gyro, accelerometer, velocimeter, frame sensors
- Tuned motor parameters based on BAM (Behavioral Actuation Model) identification for Feetech STS3215 servos
- Optimized collision geometry with only foot contact surfaces enabled

### Mesh URI Updates

All mesh URIs in `urdf/robot.urdf.xacro` have been modified from `package:///filename.stl` to use a parameterized package name: `package://$(arg package_name)/description/assets/filename.stl`.

Why? The original URDF uses `package:///` with an empty package name, which RViz2 cannot resolve. RViz2 requires a valid package name in the URI to locate mesh resources. The meshes are installed in the `description/assets/` directory of this package, so the URIs are updated to point to the correct location using the proper `package://` format. The package name is now parameterized via the `package_name` xacro argument (defaults to `ros2_control_demo_example_18`), allowing the URDF to be reused with different package names.

### ros2_control Sensor Configuration

The `open_duck_mini.ros2_control.xacro` file defines an IMU sensor named `imu` (matching MuJoCo sensor site name) with 10 state interfaces:
- Orientation (quaternion w,x,y,z)
- Angular velocity (x,y,z)
- Linear acceleration (x,y,z)

Configuration method: 
- The sensor name `imu` matches the MuJoCo site name defined in `open_duck_mini_v2.xml`. 
- The 10 state interfaces map to MuJoCo sensor outputs: `framequat` (orientation), `gyro` (angular velocity), and `accelerometer` (linear acceleration). 
- Joint limits are extracted directly from the MuJoCo XML joint `range` attributes to ensure consistency between simulation and control interfaces.

### Minimal URDF for TF Publishing

The `mujoco/robot.urdf` file provides a minimal URDF containing only the kinematic structure (links and joints) needed for `robot_state_publisher` to publish TF transforms. This URDF includes all 14 joints (10 leg + 4 head) with proper parent-child relationships. The actual robot model for simulation is loaded from the MuJoCo XML file via the `mujoco_model` parameter in the ros2_control configuration, while the URDF is used solely for TF transform computation.

The method used is to extract joint names and parent-child relationships from `open_duck_mini_v2.xml`, joint limits from `open_duck_mini.ros2_control.xacro`, and approximate transform positions from MuJoCo body positions. Links include minimal placeholder inertials for URDF validity. Joint limits include required `effort` (3.23 N⋅m, matching PID u_clamp_max) and `velocity` (10.0 rad/s) attributes for URDF parser compliance.

### Command to Drive Mini Duck (WIP)

Use the ROS2 topic /motion_controller/cmd_vel with the VelocityCommandWithHead message.

Note, these commands will cause Duck to fall immediately. Cause is being investigated.

# Launch demo

ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py

# Forward walk (0.15 m/s - trained velocity)
ros2 topic pub /motion_controller/cmd_vel example_18_motion_controller_msgs/msg/VelocityCommandWithHead \
  "{base_velocity: {linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, head_commands: [0.0, 0.0, 0.0, 0.0]}" \
  --rate 10

# Turn left (1.0 rad/s)
ros2 topic pub /motion_controller/cmd_vel example_18_motion_controller_msgs/msg/VelocityCommandWithHead \
  "{base_velocity: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}, head_commands: [0.0, 0.0, 0.0, 0.0]}" \
  --rate 10

# Stop
ros2 topic pub /motion_controller/cmd_vel example_18_motion_controller_msgs/msg/VelocityCommandWithHead \
  "{base_velocity: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, head_commands: [0.0, 0.0, 0.0, 0.0]}" \
  --rate 10

Using Python Script:
You can also use the existing test script:

ros2 run ros2_control_demo_example_18 test_motions