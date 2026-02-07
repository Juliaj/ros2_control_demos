Open Duck ros2_control + ONNX Development Pain Points

Slide 1: Integration & Environment Challenges

Pain Point 1: Model-to-ROS2 Integration Friction

Challenge:
Artifacts scattered across multiple repos (Mujoco scene.xml, mesh files, model artifacts, URDF)
URDF not provided from original training repo
Requires manual Python script with many tweaks to generate URDF from Mujoco model

Impact:
Difficult to reproduce complete setup
Hard to maintain and share across team
Manual conversion process error-prone

Solutions Implemented:
Consolidated all artifacts in example_18 directory structure
Created Python script to generate URDF from Mujoco XML with configurable tweaks
Documented artifact dependencies and generation process

Pain Point 2: Simulation Environment Mismatch

Challenge:
Behavior differences between pure Mujoco simulation vs Mujoco ROS control
Models trained in Mujoco fail in ROS2 due to observation distribution shift
Different initialization, timing, and sensor formatting

Impact:
Model works in pure Mujoco but shuffles in place in ROS2
Requires fine-tuning on ROS2-compatible data
Need validation tools to compare environments

Solutions Implemented:
State interface broadcaster aggregates hardware state into Float64Values topic
Plot Juggler integration for real-time observation comparison
Fine-tuning workflow with ROS2 replay environment
Timestep alignment (0.002s) and control rate matching (50Hz)

Pain Point 3: Sensor Data Collection Boundaries

Challenge:
Additional data (contact info, velocimeter) available via Mujoco APIs but not out-of-box
Unclear where to draw the line for standard sensor availability
Manual work needed for each new sensor type

Impact:
Fine-tuning requires base linear velocity for proper reward signal
Without velocimeter, velocity tracking reward provides no learning signal
Inconsistent sensor support across projects

Solutions Implemented:
Added velocimeter sensor support to mujoco_ros2_control
Exported velocimeter/linear_velocity.{x,y,z} as ROS2 state interfaces
Integrated into state_interfaces_broadcaster config
Data collection scripts extract and save velocimeter data to HDF5
Replay environment uses measured velocity for reward computation

Slide 2: Development & Debugging Challenges

Pain Point 4: Observation Injection & Synchronization

Challenge:
Injecting observations into controller loop is difficult — hard to synchronize
Injected data gets overwritten before action is applied
Controller update cycle timing makes state injection unreliable

Impact:
Debugging requires complex workarounds
Cannot easily test with known observation sequences
State injection for validation is unreliable

Solutions Implemented:
Created injection service to inject sensor data and controller state
Reset state service to initialize controller to known state
MuJoCo physics reset service for full environment reset
Python scripts for systematic observation-based testing
YAML-based state injection with Python output comparison

Pain Point 5: Closed-Loop Training Workflow Gaps

Challenge:
Training data collection in closed-loop systems — unclear workflow
Environment reset semantics unclear for closed-loop control
Fine-tuning requires adapting to ROS2 observation distribution

Impact:
Custom data collection scripts needed for each project
Reset behavior inconsistent between training and deployment
Distribution shift requires fine-tuning instead of direct deployment

Solutions Implemented:
Data collection launch file with forward_command_controller mode
collect_ros2_data.py script for systematic data collection
ROS2 replay environment (ros2_replay.py) for fine-tuning
HDF5 data format with observations, actions, and metadata
Integration with existing training pipeline (PPO, ONNX export)
Reference motion-based data collection for stable demonstrations

Pain Point 6: Debug Observability Limitations

Challenge:
Need to publish all debug information via topics for validation
Current debug topics lack synchronization/timestamps for correlation
Difficult to trace issues through control pipeline

Impact:
Manual correlation needed between processing stages
Hard to identify where differences occur in pipeline
No systematic way to compare C++ vs Python implementations

Solutions Implemented:
6 debug topic publishers covering all processing stages:
formatted_observation (after observation formatting)
raw_action (after ONNX inference)
processed_action (after action processor)
blended_action (after blend-in + reference motion)
prev_motor_targets (before rate limiting)
rate_limited_action (after rate limiting)
Enable/disable via parameter (enable_debug_publishing)
Python test script (test_debug_topics.py) for automated comparison
Stage-by-stage comparison with detailed statistics
Observation-based testing loop for systematic validation
