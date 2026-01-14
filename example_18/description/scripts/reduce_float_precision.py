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

#!/usr/bin/env python3
"""
Script to reduce floating-point precision in URDF files.
Rounds floating-point numbers to 8 decimal places for better performance.
"""

import re
import sys
from pathlib import Path


def round_float(match):
    """Round a floating-point number to 8 decimal places."""
    num_str = match.group(0)
    try:
        # Handle scientific notation
        if 'e' in num_str.lower():
            num = float(num_str)
            # Round to 8 significant digits for scientific notation
            if abs(num) < 1e-8:
                return f"{num:.8e}".rstrip('0').rstrip('.')
            else:
                return f"{num:.8f}".rstrip('0').rstrip('.')
        else:
            num = float(num_str)
            # Round to 8 decimal places
            rounded = round(num, 8)
            # Format without trailing zeros
            if rounded == int(rounded):
                return str(int(rounded))
            else:
                return f"{rounded:.8f}".rstrip('0').rstrip('.')
    except ValueError:
        return num_str


def reduce_precision(content, precision=8):
    """
    Reduce floating-point precision in URDF content.
    
    Args:
        content: String content of URDF file
        precision: Number of decimal places (default: 8)
    
    Returns:
        Modified content with reduced precision
    """
    # Pattern to match floating-point numbers (including scientific notation)
    # Matches: -0.019000000000000002998, 1e-9, 3.4694469519536141888e-18, etc.
    # Updated pattern to handle numbers with many decimal places
    float_pattern = r'-?\d+\.\d+(?:[eE][+-]?\d+)?|-?\d+[eE][+-]?\d+'
    
    def round_float_prec(match):
        num_str = match.group(0)
        try:
            num = float(num_str)
            # Handle very small numbers (scientific notation)
            if abs(num) < 1e-6 and num != 0:
                # Use scientific notation for very small numbers
                formatted = f"{num:.{precision}e}"
                # Remove trailing zeros after decimal point in mantissa
                parts = formatted.split('e')
                if len(parts) == 2:
                    mantissa = parts[0].rstrip('0').rstrip('.')
                    if mantissa == '' or mantissa == '-':
                        mantissa = '0'
                    formatted = f"{mantissa}e{parts[1]}"
                return formatted
            else:
                # Regular decimal notation
                rounded = round(num, precision)
                if rounded == int(rounded):
                    return str(int(rounded))
                else:
                    formatted = f"{rounded:.{precision}f}"
                    # Remove trailing zeros
                    return formatted.rstrip('0').rstrip('.')
        except (ValueError, OverflowError):
            return num_str
    
    # Replace all floating-point numbers
    # Use word boundary to avoid matching parts of other strings
    modified_content = re.sub(float_pattern, round_float_prec, content)
    
    return modified_content


def main():
    if len(sys.argv) < 2:
        print("Usage: reduce_float_precision.py <urdf_file> [precision]")
        print("  urdf_file: Path to URDF file to process")
        print("  precision: Number of decimal places (default: 8)")
        sys.exit(1)
    
    urdf_file = Path(sys.argv[1])
    precision = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    
    if not urdf_file.exists():
        print(f"Error: File not found: {urdf_file}")
        sys.exit(1)
    
    print(f"Reading {urdf_file}...")
    with open(urdf_file, 'r') as f:
        content = f.read()
    
    print(f"Reducing floating-point precision to {precision} decimal places...")
    modified_content = reduce_precision(content, precision)
    
    # Create backup
    backup_file = urdf_file.with_suffix(urdf_file.suffix + '.bak')
    print(f"Creating backup: {backup_file}")
    with open(backup_file, 'w') as f:
        f.write(content)
    
    # Write modified content
    print(f"Writing modified content to {urdf_file}...")
    with open(urdf_file, 'w') as f:
        f.write(modified_content)
    
    print("Done!")


if __name__ == '__main__':
    main()

