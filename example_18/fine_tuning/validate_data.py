#!/usr/bin/env python3
# Copyright (C) 2025 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Julia Jia

"""
Validate collected ROS2 data quality.

Checks:
1. Data format and dimensions
2. Data completeness (no NaN, inf, missing values)
3. Data statistics (ranges, distributions)
4. Velocity command coverage
5. Action reasonableness
6. Temporal consistency
7. Observation format correctness

Usage:
    python3 validate_data.py --input data.h5
"""

import argparse
import h5py
import numpy as np
import sys


def validate_data(input_path):
    """Validate collected data quality."""
    print(f"Validating data file: {input_path}\n")
    
    try:
        with h5py.File(input_path, 'r') as f:
            # Check required datasets
            required_datasets = ['observations', 'actions', 'velocity_commands', 'timestamps']
            missing = [ds for ds in required_datasets if ds not in f]
            if missing:
                print(f"ERROR: Missing required datasets: {missing}")
                return False
            
            observations = f['observations'][:]
            actions = f['actions'][:]
            velocity_commands = f['velocity_commands'][:]
            timestamps = f['timestamps'][:]
            
            num_samples = len(observations)
            print(f"Data Summary:")
            print(f"  Number of samples: {num_samples}")
            print(f"  Observation dimension: {observations.shape[1] if len(observations.shape) > 1 else len(observations)}")
            print(f"  Action dimension: {actions.shape[1] if len(actions.shape) > 1 else len(actions)}")
            print(f"  Velocity command dimension: {velocity_commands.shape[1] if len(velocity_commands.shape) > 1 else len(velocity_commands)}")
            print()
            
            # Check dimensions match
            if len(observations) != len(actions) or len(observations) != len(velocity_commands):
                print(f"ERROR: Dimension mismatch:")
                print(f"  observations: {len(observations)}")
                print(f"  actions: {len(actions)}")
                print(f"  velocity_commands: {len(velocity_commands)}")
                return False
            
            # Check for NaN and Inf
            print("Checking for invalid values...")
            obs_nan = np.isnan(observations).any()
            obs_inf = np.isinf(observations).any()
            act_nan = np.isnan(actions).any()
            act_inf = np.isinf(actions).any()
            
            if obs_nan or obs_inf:
                print(f"  WARNING: Invalid values in observations:")
                if obs_nan:
                    nan_count = np.isnan(observations).sum()
                    print(f"    NaN count: {nan_count}")
                if obs_inf:
                    inf_count = np.isinf(observations).sum()
                    print(f"    Inf count: {inf_count}")
            else:
                print("  ✓ No NaN or Inf in observations")
            
            if act_nan or act_inf:
                print(f"  WARNING: Invalid values in actions:")
                if act_nan:
                    nan_count = np.isnan(actions).sum()
                    print(f"    NaN count: {nan_count}")
                if act_inf:
                    inf_count = np.isinf(actions).sum()
                    print(f"    Inf count: {inf_count}")
            else:
                print("  ✓ No NaN or Inf in actions")
            print()
            
            # Check observation format (expected: 17 + 6*N for N joints)
            # Format: 3 (gyro) + 3 (accel) + 7 (commands) + N (joint_pos) + N (joint_vel) + 3*N (action_history) + N (motor_targets) + 2 (contacts) + 2 (phase)
            obs_dim = observations.shape[1] if len(observations.shape) > 1 else len(observations[0])
            act_dim = actions.shape[1] if len(actions.shape) > 1 else len(actions[0])
            
            # Expected: 17 + 6*N where N = num_joints
            # For 14 joints: 17 + 6*14 = 17 + 84 = 101
            expected_obs_dim = 17 + 6 * act_dim
            if obs_dim != expected_obs_dim:
                print(f"WARNING: Observation dimension mismatch:")
                print(f"  Expected: {expected_obs_dim} (for {act_dim} joints)")
                print(f"  Actual: {obs_dim}")
                print(f"  Difference: {obs_dim - expected_obs_dim}")
            else:
                print(f"✓ Observation dimension correct: {obs_dim} (for {act_dim} joints)")
            print()
            
            # Check data statistics
            print("Data Statistics:")
            print(f"  Observations:")
            print(f"    Range: [{np.min(observations):.4f}, {np.max(observations):.4f}]")
            print(f"    Mean: {np.mean(observations):.4f}, Std: {np.std(observations):.4f}")
            print(f"  Actions:")
            print(f"    Range: [{np.min(actions):.4f}, {np.max(actions):.4f}]")
            print(f"    Mean: {np.mean(actions):.4f}, Std: {np.std(actions):.4f}")
            print()
            
            # Check velocity command coverage
            print("Velocity Command Coverage:")
            vel_lin_x = velocity_commands[:, 0]
            vel_lin_y = velocity_commands[:, 1]
            vel_ang_z = velocity_commands[:, 2]
            
            print(f"  Linear X: range=[{np.min(vel_lin_x):.3f}, {np.max(vel_lin_x):.3f}], "
                  f"mean={np.mean(vel_lin_x):.3f}, std={np.std(vel_lin_x):.3f}")
            print(f"  Linear Y: range=[{np.min(vel_lin_y):.3f}, {np.max(vel_lin_y):.3f}], "
                  f"mean={np.mean(vel_lin_y):.3f}, std={np.std(vel_lin_y):.3f}")
            print(f"  Angular Z: range=[{np.min(vel_ang_z):.3f}, {np.max(vel_ang_z):.3f}], "
                  f"mean={np.mean(vel_ang_z):.3f}, std={np.std(vel_ang_z):.3f}")
            
            # Check for diverse velocity commands
            unique_vel_x = len(np.unique(np.round(vel_lin_x, decimals=2)))
            unique_vel_y = len(np.unique(np.round(vel_lin_y, decimals=2)))
            unique_vel_z = len(np.unique(np.round(vel_ang_z, decimals=2)))
            
            if unique_vel_x < 3:
                print(f"  WARNING: Low diversity in linear X velocity ({unique_vel_x} unique values)")
            if unique_vel_y < 3:
                print(f"  WARNING: Low diversity in linear Y velocity ({unique_vel_y} unique values)")
            if unique_vel_z < 3:
                print(f"  WARNING: Low diversity in angular Z velocity ({unique_vel_z} unique values)")
            print()
            
            # Check action reasonableness
            print("Action Quality:")
            # Actions should be in reasonable range (typically -1 to 1 after scaling)
            # With action_scale=0.25, actions of ±4 correspond to ±1 rad joint movement
            extreme_actions = np.abs(actions) > 4.0
            extreme_count = extreme_actions.sum()
            extreme_pct = 100.0 * extreme_count / actions.size
            
            if extreme_pct > 5.0:
                print(f"  WARNING: {extreme_pct:.1f}% of actions exceed ±4.0 (may be unstable)")
            else:
                print(f"  ✓ {extreme_pct:.1f}% extreme actions (threshold: 5%)")
            
            # Check action smoothness (rate of change)
            if len(actions) > 1:
                action_diffs = np.diff(actions, axis=0)
                max_action_change = np.max(np.abs(action_diffs))
                mean_action_change = np.mean(np.abs(action_diffs))
                
                print(f"  Max action change per step: {max_action_change:.4f} rad")
                print(f"  Mean action change per step: {mean_action_change:.4f} rad")
                
                if max_action_change > 1.0:
                    print(f"  WARNING: Large action changes detected (may cause instability)")
                if mean_action_change > 0.5:
                    print(f"  WARNING: High mean action change rate (may be unstable)")
            print()
            
            # Check temporal consistency
            print("Temporal Consistency:")
            if len(timestamps) > 1:
                time_diffs = np.diff(timestamps)
                expected_period = 0.02  # 50Hz = 0.02s
                mean_period = np.mean(time_diffs)
                std_period = np.std(time_diffs)
                
                print(f"  Mean period: {mean_period:.4f}s (expected: {expected_period:.4f}s)")
                print(f"  Period std: {std_period:.4f}s")
                
                if abs(mean_period - expected_period) > 0.01:
                    print(f"  WARNING: Period differs significantly from expected 50Hz")
                if std_period > 0.01:
                    print(f"  WARNING: High period variance (may indicate dropped samples)")
            print()
            
            # Check observation components
            print("Observation Component Analysis:")
            # Expected structure: 3 (gyro) + 3 (accel) + 7 (commands) + N (joint_pos) + N (joint_vel) + 3*N (actions) + N (motor_targets) + 2 (contacts) + 2 (phase)
            idx = 0
            gyro = observations[:, idx:idx+3]
            idx += 3
            accel = observations[:, idx:idx+3]
            idx += 3
            cmds = observations[:, idx:idx+7]
            idx += 7
            joint_pos = observations[:, idx:idx+act_dim]
            idx += act_dim
            joint_vel = observations[:, idx:idx+act_dim]
            idx += act_dim
            last_action = observations[:, idx:idx+act_dim]
            idx += act_dim
            last_last_action = observations[:, idx:idx+act_dim]
            idx += act_dim
            last_last_last_action = observations[:, idx:idx+act_dim]
            idx += act_dim
            motor_targets = observations[:, idx:idx+act_dim]
            idx += act_dim
            contacts = observations[:, idx:idx+2]
            idx += 2
            phase = observations[:, idx:idx+2]
            
            print(f"  Gyro range: [{np.min(gyro):.3f}, {np.max(gyro):.3f}]")
            print(f"  Accelerometer range: [{np.min(accel):.3f}, {np.max(accel):.3f}]")
            print(f"  Commands range: [{np.min(cmds):.3f}, {np.max(cmds):.3f}]")
            print(f"  Joint positions (relative) range: [{np.min(joint_pos):.3f}, {np.max(joint_pos):.3f}]")
            print(f"  Joint velocities (scaled) range: [{np.min(joint_vel):.3f}, {np.max(joint_vel):.3f}]")
            print(f"  Phase range: [{np.min(phase):.3f}, {np.max(phase):.3f}]")
            print(f"    Phase magnitude: [{np.min(np.linalg.norm(phase, axis=1)):.3f}, {np.max(np.linalg.norm(phase, axis=1)):.3f}] (should be ~1.0)")
            
            # Check phase is actually changing (not stuck)
            phase_changes = np.diff(phase, axis=0)
            phase_change_magnitude = np.linalg.norm(phase_changes, axis=1)
            if np.max(phase_change_magnitude) < 0.01:
                print(f"  WARNING: Phase not changing (stuck at constant value)")
            else:
                print(f"  ✓ Phase is changing (max change: {np.max(phase_change_magnitude):.4f})")
            print()
            
            # Overall assessment
            print("Overall Assessment:")
            issues = []
            if obs_nan or obs_inf:
                issues.append("Invalid values in observations")
            if act_nan or act_inf:
                issues.append("Invalid values in actions")
            if obs_dim != expected_obs_dim:
                issues.append("Observation dimension mismatch")
            if extreme_pct > 5.0:
                issues.append("High percentage of extreme actions")
            if unique_vel_x < 3:
                issues.append("Low velocity command diversity")
            
            if issues:
                print(f"  ⚠ Found {len(issues)} issue(s):")
                for issue in issues:
                    print(f"    - {issue}")
            else:
                print("  ✓ Data quality looks good!")
            
            return len(issues) == 0
            
    except Exception as e:
        print(f"ERROR: Failed to validate data: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(description='Validate collected ROS2 data quality')
    parser.add_argument(
        '--input',
        type=str,
        required=True,
        help='Input HDF5 data file to validate'
    )
    
    args = parser.parse_args()
    
    success = validate_data(args.input)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

