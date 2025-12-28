# Walking Forward Debugging Guide

## Problem Summary

The duck robot receives forward velocity commands (e.g., 0.15 m/s) but may not walk forward as expected. Common symptoms include:
- Robot shuffling in place (X position oscillates instead of increasing)
- Both feet always in contact (no foot lift-off during swing phase)
- Robot falling or unstable behavior

## Debugging Tools

### 1. Base Position Telemetry

**File**: `mujoco_ros2_control/src/mujoco_system_interface.cpp`  
**Location**: PhysicsLoop after `mj_copyData`

Logs floating base position every 1 second:
```
Floating base position: x=%.4f, y=%.4f, z=%.4f | time=%.2f
```

**What to look for**:
- X-coordinate should **increase over time** if robot is walking forward
- If X stays constant (~0.0), robot is moving in place
- Z should stay around 0.13-0.15 m (standing height)

### 2. Full Action Vector Logging

**File**: `example_18/controllers/src/motion_controller.cpp`  
**Location**: Every 25 updates

Logs:
- **Velocity command**: [lin_vel_x, lin_vel_y, ang_vel_z, head_commands...]
- **Contact sensors**: Left and Right foot contact (should alternate during gait)
- **All 14 policy actions**: Raw model outputs before scaling
- **All 14 motor targets**: Final joint position commands after processing
- **Max velocity change allowed**: Shows motor speed limiting effect

**What to look for**:
- **Contacts should alternate**: When left=1.0, right should be ~0.0 (and vice versa)
  - If both feet are always 1.0, robot is shuffling (not lifting feet)
  - This would prevent forward motion (no push-off phase)
- **Actions should vary**: All 14 joint commands should show variation
  - Left leg actions [0-4] vs right leg actions [9-13] should be out of phase
- **Motor targets should follow actions**: Scaled and offset versions of actions

### 3. Command Write Status

**File**: `example_18/controllers/src/motion_controller.cpp`  
**Location**: Every 100 updates

Logs number of successfully written joint commands:
```
[CMD WRITE] Successfully wrote X/14 joint commands
```

**What to look for**:
- Should always be 14/14 (all joints commanded)
- If less than 14, some command interfaces are failing

## Testing Procedure

1. **Rebuild packages**:
```bash
cd /home/juliajia/ros2_ws
colcon build --packages-select mujoco_ros2_control ros2_control_demo_example_18
```

2. **Run simulation with logging**:
```bash
ros2 launch ros2_control_demo_example_18 example_18_mujoco.launch.py 2>&1 | tee walk_debug.log
```

3. **In another terminal, send walk command**:
```bash
ros2 run ros2_control_demo_example_18 test_motions
```

4. **Analyze logs**:
```bash
# Check if base is moving forward
grep "Floating base position" walk_debug.log | head -20

# Check contact alternation pattern
grep "Contacts:" walk_debug.log | head -40

# Check action magnitudes
grep "Action (all 14)" walk_debug.log | head -10

# Check command write success
grep "CMD WRITE" walk_debug.log

# Check contact sensor messages
grep "Contact sensor" walk_debug.log | grep "in_contact"
```

## Expected Behavior vs Issues

### ✅ Expected (Robot Walking Forward)
- Base X position increases steadily (e.g., 0.00 → 0.05 → 0.10 → ...)
- Contacts alternate: L=1.0/R=0.0 → L=0.0/R=1.0 (phase pattern)
- Actions show coordinated leg movement (left/right out of phase)
- Z position stable around 0.13-0.15 m

### ❌ Issue: Robot Moving in Place (Shuffling)

**Symptoms**:
- Base X position stays constant or oscillates (e.g., 0.0152 ↔ 0.0165)
- Y position may drift sideways
- Contacts show both feet always in contact: L=1.0, R=1.0
- Actions show leg motion but no lift-off phase
- Phase values cycle correctly but physical motion doesn't follow

**Root Cause**: Both contact sensors report 1.0 simultaneously, meaning:
1. The robot cannot lift a foot during swing phase
2. No push-off phase occurs (both feet always on ground)
3. Robot can only shift weight, not take steps
4. This causes shuffling motion instead of forward walking

**Possible Causes**:

1. **Feet Not Lifting High Enough**
   - Policy actions don't generate sufficient lift
   - Action scaling too conservative (action_scale = 0.25)
   - Motor velocity limits preventing rapid position changes
   - Joint position limits constraining swing motion

2. **Contact Sensor Sensitivity**
   - Contact detection threshold too low
   - Sensor geometry (foot too large, detecting contact at edges)
   - MuJoCo contact detection parameters (margin, friction)

3. **Policy-Observation Mismatch**
   - Training used phase-based contact estimation (alternating)
   - Actual sensors report both feet always in contact
   - Policy expects alternating contacts but receives constant [1.0, 1.0]

4. **Action Processing Issue**
   - Default joint positions offset incorrect
   - Action scaling insufficient for swing phase
   - Motor targets not reaching positions needed for foot lift

### ❌ Issue: Robot Falling

**Symptoms**:
- Z position decreasing rapidly
- Base tilting (would need to add pitch/roll logging to confirm)

**Likely cause**: IMU or observation mismatch

## Solutions

### Solution 1: Verify Contact Sensor Behavior

**Action**: Check if contact sensors are correctly detecting lift-off
```bash
# In log, search for contact sensor messages
grep "Contact sensor" walk_debug.log | grep "in_contact"
```

**Expected**: Should see alternating "in_contact=YES" and "in_contact=NO" for each foot

**If both always YES**: Contact detection is the problem

### Solution 2: Compare with Python Reference

**Action**: Run Python validation to see expected contact pattern
```bash
cd /home/juliajia/dev/Open_Duck_Playground
python tests/validate_onnx_simulation.py
```

**Compare**:
- Contact values during walking
- Foot lift heights during swing phase
- Joint positions during swing vs stance

### Solution 3: Increase Action Scale or Swing Amplitude

**Action**: If actions are too small, increase action_scale or modify policy
- Current: `action_scale = 0.25`
- Try: `action_scale = 0.3` or `0.35` (if policy allows)
- Or: Check if policy outputs need different scaling for swing phase

### Solution 4: Use Phase-Based Contact Estimation (Temporary)

**Action**: If sensors are unreliable, fall back to phase-based contacts
- Modify `motion_controller.cpp` to force phase-based contacts
- This matches training data if policy was trained with phase-based contacts

### Solution 5: Check MuJoCo Contact Parameters

**Action**: Verify MuJoCo contact detection settings
- Check `solimp` and `solref` parameters in MJCF
- Verify contact margin settings
- Ensure foot geometry allows proper lift-off

## Diagnostic Checklist

Based on log analysis, follow these steps:

1. **If base X is NOT increasing**: Focus on contact alternation and action magnitudes
2. **If contacts are NOT alternating**: Compare with Python reference contact values
3. **If actions look wrong**: Check ObservationFormatter input vs Python reference
4. **If commands not writing**: Check command_interfaces_ registration in on_configure()

## Key Metrics Summary

| Metric | Expected | Actual (Shuffling) | Status |
|--------|----------|-------------------|--------|
| X position | Increasing steadily | Oscillating 0.015-0.016 | ❌ |
| Contact alternation | L=1/R=0 ↔ L=0/R=1 | L=1/R=1 (constant) | ❌ |
| Phase cycling | Alternating pattern | Correct pattern | ✅ |
| Actions | Varying appropriately | Varying | ✅ |
| Motor targets | Following actions | Following | ✅ |
| Command write | 14/14 success | 14/14 success | ✅ |

**Conclusion**: The robot has correct timing (phase) and action generation, but cannot lift feet due to contact sensors always reporting contact. This prevents proper walking gait and causes shuffling motion.

## Files to Investigate

1. `mujoco_ros2_control/src/mujoco_system_interface.cpp` - Contact sensor implementation
2. `example_18/controllers/src/observation_formatter.cpp` - Contact extraction logic
3. `example_18/controllers/src/motion_controller.cpp` - Contact usage in observation
4. `Open_Duck_Playground/tests/validate_onnx_simulation.py` - Reference implementation
5. `Open_Duck_Playground/playground/open_duck_mini_v2/mujoco_infer_base.py` - Python contact detection (lines 259-283)

## Reference Files

- Python validation: `/home/juliajia/dev/Open_Duck_Playground/tests/validate_onnx_simulation.py`
- Python contact detection: `/home/juliajia/dev/Open_Duck_Playground/playground/open_duck_mini_v2/mujoco_infer_base.py` (lines 259-283)

