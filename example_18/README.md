# ros2_control_demo_example_18

This example demonstrates how an ONNX policy can drive a robot to follow a command. This This also demonstrates observation obtained from state_interfaces_broadcasters. Design documentation: [doc/design.rst](doc/design.rst)

Find the full documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_18/doc/userdoc.html).



scratch pad 12/27/2025

uv run python3 ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/fine_tuning/manual_control_ros2.py --output mujoco_manual_data.h5 --onnx-model /home/juliajia/ros2_ws/src/ros-controls/ros2_control_demos/example_18/onnx_model/open_duck_mini_v2.onnx

**Note**: This script now collects base linear velocity from the velocimeter sensor (indices 40-42 in state_interfaces_broadcaster). The data is saved as `base_linear_velocities` dataset in the HDF5 file, which is used by the replay environment for proper reward computation.

cp /home/juliajia/dev/Open_Duck_Playground/checkpoints/2025_12_27_220956_6881280.onnx src/ros-controls/ros2_control_demos/example_18/onnx_model/2025_12_27_220956_6881280.onnx

ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py