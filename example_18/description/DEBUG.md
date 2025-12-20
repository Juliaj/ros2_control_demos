# DEBUG LOGS


## 12/13/2025

```
$ source run_onnx_ctrl.sh
```

[rviz2-6] terminate called after throwing an instance of 'Ogre::ItemIdentityException'
[rviz2-6]   what():  ItemIdentityException: Can not find codec for 'mesh' format.
[rviz2-6] Supported formats are: bmp gif hdr jpeg jpg pgm pic png ppm psd tga. in getCodec at /home/runner/conda-bld/bld/rattler-build_ros-jazzy-rviz-ogre-vendor_1759134848/work/build/ogre_vendor-prefix/src/ogre_vendor/OgreMain/src/OgreCodec.cpp (line 81)

```
# removed rviz2 node, in pixi shell
$ source run_onnx_ctrl.sh
```
[ros2_control_node-2] [INFO] [1765646303.242816085] [controller_manager]: Received robot description from topic.
[ERROR] [ros2_control_node-2]: process has died [pid 229698, exit code -11, cmd '/home/juliaj/ros2_ws/install/controller_manager/lib/controller_manager/ros2_control_node --log-level debug --ros-args --params-file /tmp/launch_params_8u9kr_ou --params-file /home/juliaj/ros2_ws/install/ros2_control_demo_example_18/share/ros2_control_demo_example_18/config/open_duck_mini_controllers.yaml'].

```
# removed rviz2 node, no pixi shell, plain ros2_ws workspace
$ source run_onnx_ctrl.sh
```


[ros2_control_node-2] [INFO] [1765646415.244651675] [motion_controller]: === ONNX Model Input Metadata ===
[ros2_control_node-2] [INFO] [1765646415.244684865] [motion_controller]: Number of inputs: 1
[ros2_control_node-2] [INFO] [1765646415.244796437] [motion_controller]:   Input[0]: name='obs', shape=[1, 101], type=float32
[ros2_control_node-2] [INFO] [1765646415.244840784] [motion_controller]: === ONNX Model Output Metadata ===
[ros2_control_node-2] [INFO] [1765646415.244844031] [motion_controller]: Number of outputs: 1
[ros2_control_node-2] [INFO] [1765646415.244855266] [motion_controller]:   Output[0]: name='continuous_actions', shape=[1, 14], type=float32
[ros2_control_node-2] [INFO] [1765646415.244862106] [motion_controller]: === Model Validation ===
[ros2_control_node-2] [INFO] [1765646415.244870104] [motion_controller]: Expected observation dimension (from config): 101 (17 + 6*14 joints)
[ros2_control_node-2] [INFO] [1765646415.244873497] [motion_controller]: Model input shape: [1, 101]
[ros2_control_node-2] [INFO] [1765646415.244876188] [motion_controller]: Model input total elements: 101
[ros2_control_node-2] [WARN] [1765646415.244885096] [motion_controller]: Model output size may not match number of joints (14)
[ros2_control_node-2] [INFO] [1765646415.244929021] [motion_controller]: Model loaded successfully from: /home/juliaj/ros2_ws/install/ros2_control_demo_example_18/share/ros2_control_demo_example_18/onnx_model/BEST_WALK_ONNX_2.onnx
[ros2_control_node-2] [rcutils|error_handling.c:65] an error string (message, file name, or formatted message) will be truncated
[ros2_control_node-2] 
[ros2_control_node-2] >>> [rcutils|error_handling.c:108] rcutils_set_error_state()
[ros2_control_node-2] This error state is being overwritten:
[ros2_control_node-2] 
[ros2_control_node-2]   'Type support not from this implementation. Got:
[ros2_control_node-2]     Handle's typesupport identifier (rosidl_typesupport_cpp) is not supported by this library, at /home/runner/conda-bld/bld/rattler-build_ros-jazzy-rosidl-typesupport-cpp_1759136271/work/ros-jazzy-rosidl-typesupport-cpp/src/work/src/type_support_dispatch.hpp:114
[ros2_control_node-2]     Failed to find symbol 'rosidl_typesupport_fastrtps_cpp__get_message_type_support_handle__control_msgs__msg__Float64Values' in library, at /home/runner/conda-bld/bld/rattler-build_ros-jazzy-rosidl-typesupport-cpp_1759136271/work/ros-jazzy-rosidl-typesupport-cpp/src/work/src/type_support_dispatch.hpp:96
[ros2_control_node-2] while fetching it, at /home/runner/conda-bld/bld/rattler-build_ros-jazzy-rmw-fastrtps-cpp_1759137049/work/ros-jazzy-rmw-fastrtps-cpp/src/work/src/su, at /home/runner/conda-bld/bld/rattler-build_ros-jazzy-rcl_1759137495/work/ros-jazzy-rcl/src/work/src/rcl/subscription.c:112'
[ros2_control_node-2] 
[ros2_control_node-2] with this new error message:
[ros2_control_node-2] 
[ros2_control_node-2]   'invalid allocator, at /home/runner/conda-bld/bld/rattler-build_ros-jazzy-rcl_1759137495/work/ros-jazzy-rcl/src/work/src/rcl/subscription.c:261'
[ros2_control_node-2] 
[ros2_control_node-2] rcutils_reset_error() should be called after error handling to avoid this.
[ros2_control_node-2] <<<
[ros2_control_node-2] invalid allocator, at /home/runner/conda-bld/bld/rattler-build_ros-jazzy-rcl_1759137495/work/ros-jazzy-rcl/src/work/src/rcl/subscription.c:261
[ros2_control_node-2] [ERROR] [1765646415.247918443] [motion_controller]: Caught exception in callback for transition 10
[ros2_control_node-2] [ERROR] [1765646415.247940000] [motion_controller]: Original error: could not create subscription: invalid allocator, at /home/runner/conda-bld/bld/rattler-build_ros-jazzy-rcl_1759137495/work/ros-jazzy-rcl/src/work/src/rcl/subscription.c:261
[ros2_control_node-2] [WARN] [1765646415.247960295] [motion_controller]: Callback returned ERROR during the transition: configure
[ros2_control_node-2] [ERROR] [1765646415.247976028] [controller_manager]: After configuring, controller 'motion_controller' is in state 'unconfigured' , expected inactive.
[spawner-5] [ERROR] [1765646415.248745203] [spawn_motion_controller]: Failed to configure controller
[spawner-4] [INFO] [1765646415.279963171] [spawn_interfaces_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[ERROR] [spawner-5]: process has died [pid 235185, exit code 1, cmd '/home/juliaj/ros2_ws/install/controller_manager/lib/controller_manager/spawner motion_controller --ros-args -r __node:=spawn_motion_controller'].
[ros2_control_node-2] [INFO] [1765646415.534447924] [controller_manager]: Loading controller : 'interfaces_state_broadcaster' of type 'interfaces_state_broadcaster/InterfacesStateBroadcaster'
[ros2_control_node-2] [INFO] [1765646415.534584837] [controller_manager]: Loading controller 'interfaces_state_broadcaster'
[ros2_control_node-2] [INFO] [1765646415.540359408] [controller_manager]: Controller 'interfaces_state_broadcaster' node arguments: --ros-args --params-file /tmp/launch_params_fv57cm4r --params-file /home/juliaj/ros2_ws/install/ros2_control_demo_example_18/share/ros2_control_demo_example_18/config/open_duck_mini_controllers.yaml 
[spawner-4] [INFO] [1765646415.572155041] [spawn_interfaces_state_broadcaster]: Loaded interfaces_state_broadcaster
[ros2_control_node-2] [INFO] [1765646415.574549841] [controller_manager]: Configuring controller: 'interfaces_state_broadcaster'
[ros2_control_node-2] [rcutils|error_handling.c:65] an error string (message, file name, or formatted message) will be truncated
[ros2_control_node-2] [ERROR] [1765646415.575561216] [interfaces_state_broadcaster]: Caught exception in callback for transition 10

```
# plain ros2_ws workspace

```


Initial round of loading controllers failed

workaround, manually loaded all the controllers
/home/juliaj/ros2_ws/install/controller_manager/lib/controller_manager/spawner joint_state_broadcaster --controller-manager /controller_manager


[gazebo-1] 2025-12-13 14:51:15.121550216 [W:onnxruntime:LocomotionController, device_discovery.cc:164 DiscoverDevicesForPlatform] GPU device discovery failed: device_discovery.cc:89 ReadFileContents Failed to open file: "/sys/class/drm/card0/device/vendor"


[gazebo-1] [ERROR] [1765666275.126176962] [locomotion_controller]: MISMATCH: Model input size (45) does not match expected observation dimension (46). Please check model or observation formatter configuration.
[gazebo-1] [ERROR] [1765666275.126203234] [locomotion_controller]: Expected from env_cfg.py: 4 (velocity_commands) + 3 (base_ang_vel) + 3 (projected_gravity) + 12 (joint_pos) + 12 (joint_vel) + 12 (previous_action) = 46
[gazebo-1] [WARN] [1765666275.126213403] [locomotion_controller]: Model output size may not match number of joints (12)
[gazebo-1] [INFO] [1765666275.126266824] [locomotion_controller]: Model loaded successfully from: /home/juliaj/ros2_ws/install/ros2_control_demo_example_18/share/ros2_control_demo_example_18/onnx_model/policy_biped_25hz_a.onnx


## Issue to follow up
[ros2_control_node-2] 2025-12-13 09:20:15.228522277 [W:onnxruntime:LocomotionController, device_discovery.cc:164 DiscoverDevicesForPlatform] GPU device discovery failed: device_discovery.cc:89 ReadFileContents Failed to open file: "/sys/class/drm/card0/device/vendor"