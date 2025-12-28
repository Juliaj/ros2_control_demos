# MuJoCo and Open Duck Mini Setup Comparison

This document compares the MuJoCo simulation setup between:
- **Reference**: `Open_Duck_Playground/tests/validate_onnx_simulation.py` (working)
- **Example 18**: `ros2_control_demos/example_18/description/` (duck falls immediately)

## High-Level Differences

### 1. Simulation Timestep (CRITICAL)

**Reference Implementation:**
- MuJoCo timestep: `0.002` seconds (2ms) - set programmatically
- Source: `mujoco_infer_base.py` line 15: `self.sim_dt = 0.002`
- Source: `mujoco_infer_base.py` line 17: `self.model.opt.timestep = self.sim_dt`
- Decimation: `10` steps per control update
- Effective control rate: `1/(0.002 * 10) = 50 Hz`

**Example 18:**
- MuJoCo timestep: `0.01` seconds (10ms) - set in `scene.xml`
- Source: `description/mujoco/scene.xml` line 9: `<option timestep="0.01"/>`
- **5x larger timestep** than reference
- Controller update rate: `50 Hz` (configured in `open_duck_mini_controllers.yaml`)
- **Mismatch**: Controller expects 50Hz but simulation runs at 100Hz (1/0.01)

**Impact**: Larger timestep causes:
- Less stable physics integration
- Coarser control updates
- Potential instability in contact dynamics
- Different effective control frequency

### 2. Initial Robot Pose

**Reference Implementation:**
- Uses "home" keyframe from `scene_flat_terrain.xml`
- Initial base height: `0.15m` (from keyframe qpos)
- Default actuator positions from keyframe `ctrl` values
- Source: `mujoco_infer_base.py` line 127-128:
  ```python
  self.data.qpos[:] = self.model.keyframe("home").qpos
  self.data.ctrl[:] = self.default_actuator
  ```

**Example 18:**
- Uses "home" keyframe from `open_duck_mini_v2.xml` (if present)
- Initial base height: `0.22m` (from `open_duck_mini_v2.xml` line 58: `pos="0 0 0.22"`)
- **Different initial height** (0.22m vs 0.15m)
- May not have explicit keyframe initialization in mujoco_ros2_control

**Impact**: Different starting pose can affect:
- Initial stability
- Contact forces at startup
- Transition to walking motion

### 3. Ground Friction

**Reference Implementation:**
- Ground friction: `0.6` (from `scene_flat_terrain.xml` line 36)
- Friction dimensions: `condim="3"` (3D friction cone)

**Example 18:**
- Ground friction: `1.5` (from `scene.xml` line 14: `friction="1.5 0.01 0.0006"`)
- **2.5x higher sliding friction**
- Different friction model (sliding/torsional/rolling components)

**Impact**: Higher friction can cause:
- Different contact dynamics
- Potential sticking/slipping issues
- Different force transmission during walking

### 4. Control Update Timing

**Reference Implementation:**
- Control updates every `decimation=10` simulation steps
- Control period: `0.002 * 10 = 0.02s` (50 Hz)
- Action applied and held for 10 steps

**Example 18:**
- Controller update rate: `50 Hz` (from `open_duck_mini_controllers.yaml`)
- But simulation timestep is `0.01s` (100 Hz)
- **Mismatch**: Controller updates at 50Hz but simulation runs at 100Hz
- Action may be applied for only 1 step instead of 10

**Impact**: Control timing mismatch causes:
- Actions not held long enough
- Inconsistent control application
- Potential instability from rapid action changes

### 5. Scene Configuration

**Reference Implementation:**
- Scene file: `scene_flat_terrain.xml`
- Minimal scene (just ground plane)
- No explicit timestep in scene (set programmatically)
- Ground plane: `size="0 0 0.01"` (thin)

**Example 18:**
- Scene file: `scene.xml` (wraps `open_duck_mini_v2.xml`)
- Includes visualization settings
- Explicit timestep: `0.01s` in scene file
- Ground plane: `size="0 0 .125"` (thicker)

### 6. Motor/Actuator Control

**Reference Implementation:**
- Direct control via `data.ctrl` array
- Actions applied as: `motor_targets = default_actuator + action * action_scale`
- Action scale: `0.25`
- Motor velocity limits: `5.24 rad/s`

**Example 18:**
- Control via ros2_control hardware interface
- Position commands sent to joints
- May have additional filtering/delay in hardware interface
- Same action scale and limits (from config)

**Impact**: Hardware interface may introduce:
- Additional latency
- Different control application timing
- Potential smoothing/filtering effects

## Summary of Critical Issues

1. **Timestep Mismatch (HIGH PRIORITY)**
   - Reference: 2ms timestep
   - Example 18: 10ms timestep (5x larger)
   - **Fix**: Change `scene.xml` timestep to `0.002` or override programmatically

2. **Control Rate Mismatch (HIGH PRIORITY)**
   - Controller expects 50Hz updates
   - Simulation runs at 100Hz (1/0.01s)
   - **Fix**: Either match timestep to 2ms OR adjust controller rate to match simulation

3. **Initial Pose Difference (MEDIUM PRIORITY)**
   - Different starting heights (0.15m vs 0.22m)
   - May affect initial stability
   - **Fix**: Ensure keyframe initialization matches reference

4. **Friction Difference (MEDIUM PRIORITY)**
   - Reference: 0.6 sliding friction
   - Example 18: 1.5 sliding friction
   - **Fix**: Match friction values to reference

## Recommended Fixes

### Priority 1: Fix Timestep
```xml
<!-- description/mujoco/scene.xml -->
<option timestep="0.002"/>  <!-- Change from 0.01 to 0.002 -->
```

### Priority 2: Verify Control Rate
- Ensure controller update rate matches: `1/(0.002 * 10) = 50 Hz`
- Verify mujoco_ros2_control applies actions correctly with decimation

### Priority 3: Match Initial Pose
- Ensure "home" keyframe is used for initialization
- Verify initial base height matches reference (0.15m)

### Priority 4: Match Friction
```xml
<!-- description/mujoco/scene.xml -->
<default>
    <geom friction="0.6 0.01 0.0006"/>  <!-- Match reference -->
</default>
```

## Additional Notes

- The reference uses direct MuJoCo API calls, while example_18 uses mujoco_ros2_control hardware interface
- Hardware interface may introduce additional delays or filtering
- Verify that action application timing matches the reference (actions held for decimation steps)

