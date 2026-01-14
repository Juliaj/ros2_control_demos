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
Create YAML file for state injection from collected HDF5 data.

Reads HDF5 file created by manual_control_ros2.py and extracts data
for a specific step (e.g., where lin_x=0.15) to create injection YAML.
"""

import argparse
import h5py
import numpy as np
import yaml
from datetime import datetime
from pathlib import Path


def find_step_with_velocity(h5_file, target_lin_x=0.15, tolerance=0.01):
    """Find step index where lin_x is closest to target value."""
    velocity_commands = h5_file['velocity_commands'][:]
    lin_x_values = velocity_commands[:, 0]
    
    # Find step with lin_x closest to target
    differences = np.abs(lin_x_values - target_lin_x)
    best_idx = np.argmin(differences)
    best_diff = differences[best_idx]
    
    if best_diff > tolerance:
        print(f"Warning: Best match has lin_x={lin_x_values[best_idx]:.3f}, "
              f"diff={best_diff:.3f} (target={target_lin_x:.3f}, tolerance={tolerance:.3f})")
    
    return best_idx, lin_x_values[best_idx]


def compute_python_outputs(h5_file, step_idx, num_joints, action_scale=0.25, blend_in_steps=50):
    """Compute Python outputs (all 4 stages) from HDF5 data."""
    # Extract raw action (ONNX inference output)
    if 'actions' not in h5_file:
        print("Error: actions not found in HDF5 file.")
        return None
    
    raw_action = h5_file['actions'][step_idx, :]
    
    # Extract controller state
    state_grp = h5_file['controller_state']
    motor_targets = state_grp['motor_targets'][step_idx, :]  # This is rate_limited_action
    prev_motor_targets = state_grp['prev_motor_targets'][step_idx, :]
    onnx_active_steps = int(state_grp['onnx_active_steps'][step_idx])
    
    # Get default joint positions (use motor_targets from step 0 or prev_motor_targets)
    # In practice, default_joint_positions should be initialized from robot description
    # For now, we'll estimate from the data
    if step_idx > 0:
        # Use motor_targets from previous step as approximation
        default_joint_positions = prev_motor_targets.copy()
    else:
        # Use current motor_targets as approximation (not ideal but works)
        default_joint_positions = motor_targets.copy()
    
    # 1. Raw action (already have)
    raw_action_array = np.array(raw_action, dtype=np.float64)
    
    # 2. Processed action: default + raw_action * action_scale
    action_scale_array = np.ones(num_joints, dtype=np.float64) * action_scale
    processed_action = default_joint_positions + raw_action_array * action_scale_array
    
    # 3. Blended action
    blend_factor = min(1.0, onnx_active_steps / blend_in_steps)
    blended_action = (
        (1.0 - blend_factor) * default_joint_positions +
        blend_factor * processed_action
    )
    
    # 4. Rate-limited action (already have as motor_targets)
    rate_limited_action = np.array(motor_targets, dtype=np.float64)
    
    return {
        'raw_action': raw_action_array.tolist(),
        'processed_action': processed_action.tolist(),
        'blended_action': blended_action.tolist(),
        'rate_limited_action': rate_limited_action.tolist()
    }


def create_yaml_from_h5(h5_path, output_yaml_path, target_lin_x=0.15, step_idx=None, 
                        create_python_outputs=False, python_outputs_path=None):
    """Create YAML file from HDF5 data."""
    with h5py.File(h5_path, 'r') as f:
        num_samples = f.attrs['num_samples']
        num_joints = f.attrs['num_joints']
        
        print(f"Loaded HDF5 file: {num_samples} samples, {num_joints} joints")
        
        # Find step with target velocity or use provided index
        if step_idx is None:
            step_idx, actual_lin_x = find_step_with_velocity(f, target_lin_x)
            print(f"Selected step {step_idx} with lin_x={actual_lin_x:.3f}")
        else:
            if step_idx >= num_samples:
                print(f"Error: step_idx {step_idx} >= num_samples {num_samples}")
                return False
            actual_lin_x = f['velocity_commands'][step_idx, 0]
            print(f"Using step {step_idx} with lin_x={actual_lin_x:.3f}")
        
        # Extract interface data
        if 'interface_data' not in f:
            print("Error: interface_data not found in HDF5 file.")
            print("Make sure manual_control_ros2.py was run with data collection enabled.")
            return False
        
        interface_data = f['interface_data'][step_idx, :].tolist()
        
        # Extract controller state
        if 'controller_state' not in f:
            print("Error: controller_state not found in HDF5 file.")
            print("Make sure manual_control_ros2.py was run with data collection enabled.")
            return False
        
        state_grp = f['controller_state']
        controller_state = {
            'last_action': state_grp['last_action'][step_idx, :].tolist(),
            'last_last_action': state_grp['last_last_action'][step_idx, :].tolist(),
            'last_last_last_action': state_grp['last_last_last_action'][step_idx, :].tolist(),
            'motor_targets': state_grp['motor_targets'][step_idx, :].tolist(),
            'imitation_i': float(state_grp['imitation_i'][step_idx]),
            'prev_motor_targets': state_grp['prev_motor_targets'][step_idx, :].tolist(),
            'onnx_active_steps': int(state_grp['onnx_active_steps'][step_idx]),
            'stabilization_steps': int(state_grp['stabilization_steps'][step_idx])
        }
        
        # Create YAML structure
        yaml_data = {
            'interface_data': {
                'values': interface_data
            },
            'controller_state': controller_state
        }
        
        # Write YAML file
        with open(output_yaml_path, 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)
        
        print(f"Created YAML file: {output_yaml_path}")
        print(f"  Step: {step_idx}/{num_samples}")
        print(f"  lin_x: {actual_lin_x:.3f}")
        print(f"  Interface data size: {len(interface_data)}")
        print(f"  Motor targets: {controller_state['motor_targets'][:3]}...")
        print(f"  Imitation phase: {controller_state['imitation_i']:.3f}")
        print(f"  ONNX active steps: {controller_state['onnx_active_steps']}")
        
        # Create Python outputs YAML if requested
        if create_python_outputs:
            python_outputs = compute_python_outputs(f, step_idx, num_joints)
            if python_outputs is None:
                print("Warning: Failed to compute Python outputs")
            else:
                if python_outputs_path is None:
                    # Generate path from injection YAML path
                    python_outputs_path = str(output_yaml_path).replace('injection_state.yaml', 'python_outputs.yaml')
                
                python_outputs_data = {'python_outputs': python_outputs}
                with open(python_outputs_path, 'w') as yaml_file:
                    yaml.dump(python_outputs_data, yaml_file, default_flow_style=False, sort_keys=False)
                
                print(f"Created Python outputs YAML file: {python_outputs_path}")
        
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Create YAML file for state injection from HDF5 data')
    parser.add_argument(
        '--h5-file', type=str, required=True,
        help='Path to HDF5 file created by manual_control_ros2.py')
    parser.add_argument(
        '--output', type=str, required=True,
        help='Output YAML file path')
    parser.add_argument(
        '--lin-x', type=float, default=0.15,
        help='Target lin_x value to find (default: 0.15)')
    parser.add_argument(
        '--step', type=int, default=None,
        help='Use specific step index instead of searching for lin_x')
    parser.add_argument(
        '--python-outputs', action='store_true',
        help='Also create Python outputs YAML file')
    parser.add_argument(
        '--python-outputs-path', type=str, default=None,
        help='Path for Python outputs YAML file (default: auto-generated from output path)')
    
    args = parser.parse_args()
    
    h5_path = Path(args.h5_file)
    if not h5_path.exists():
        print(f"Error: HDF5 file not found: {h5_path}")
        return 1
    
    # Add datetime prefix to output filename
    output_path = Path(args.output)
    datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = output_path.parent / f"{datetime_str}_{output_path.name}"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    success = create_yaml_from_h5(h5_path, output_path, args.lin_x, args.step,
                                  create_python_outputs=args.python_outputs,
                                  python_outputs_path=args.python_outputs_path)
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())

