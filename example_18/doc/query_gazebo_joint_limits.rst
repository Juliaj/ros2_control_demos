Querying Gazebo Joint Limits
=============================

During test, we noticed that the controller uses correct limits ([-0.174533, 1.5708] for hip_roll), but Gazebo may reject commands >0.15. The steps below is to investigate whether this is indeed the case.

gz_ros2_control has no connection to this
------------------------------------------

:code:`gz_ros2_control` does not enforce joint limits. It passes commands directly to Gazebo without validation.

Findings:

1. No limit checking in :code:`write()` method (``747:843:gz_ros2_control/gz_ros2_control/src/gz_system.cpp``):
   - Position commands are converted to velocity and sent via :code:`JointVelocityCmd`
   - Velocity/effort commands are sent directly
   - No clamping or validation against limits

2. Joint axis is read but limits aren't used (``300:301:gz_ros2_control/gz_ros2_control/src/gz_system.cpp``):
   - Code reads :code:`JointAxis` from Gazebo's ECM
   - Position limits from the axis are not accessed or enforced

3. Limit enforcement happens in Gazebo:
   - :code:`gz_ros2_control` relies on Gazebo's internal enforcement
   - The issue (URDF :code:`velocity="15"` interpreted as position limit 0.15) is in Gazebo's SDF/URDF parsing, not in :code:`gz_ros2_control`

Hypothesis: The mismatch may be coming from Gazebo misinterpreting the URDF velocity limit as a position limit. :code:`gz_ros2_control` is a pass-through and doesn't add its own limit checks.

Method 1: Check Error Messages
------------------------------------------------

Check the logs for messages when Gazebo rejects a command, it logs the enforced limit. For example:

.. code-block:: bash

   # Monitor ROS2 logs for limit rejection messages
   ros2 topic echo /rosout | grep -i "clamped\|limit\|out of limits"

   # Look for messages like:
   # "Joint 'leg_left_hip_roll_joint' command clamped from -0.196391 to -0.174533 (limits: [-0.174533, 1.570800])"

Example output analysis:
- :code:`command clamped from X to Y (limits: [min, max])` - Shows the actual limits being enforced
- :code:`Enforcing command limits is enabled...` - Indicates ros2_control's JointLimiter is active
- :code:`Creating JointSaturationLimiter for joint...` - ros2_control is enforcing limits, not Gazebo directly
- If limits show :code:`[-0.174533, 1.570800]` - Correct URDF limits are being used
- If limits show :code:`[-0.15, 0.15]` - May indicate velocity="15" was misinterpreted as position limit

Important: When you see "JointLimiter" or "JointSaturationLimiter" messages, the limits are being enforced by ros2_control, not Gazebo. This means:
- ros2_control reads limits from URDF/ros2_control config
- ros2_control clamps commands before sending to Gazebo
- Gazebo may have different limits, but commands never reach Gazebo if they exceed ros2_control limits

To check Gazebo's limits: Look for messages that don't mention "JointLimiter" - those would be from Gazebo's internal enforcement.
