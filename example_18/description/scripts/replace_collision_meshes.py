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
Script to replace collision meshes with simplified box primitives in URDF.

This significantly improves Gazebo Sim performance.
"""

import re
import sys
from pathlib import Path


def replace_collision_mesh_with_box(match):
    """Replace a collision mesh block with a simplified box."""
    full_match = match.group(0)

    # Extract origin if present (handle both single-line and multi-line)
    origin_match = re.search(r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\s*/>', full_match)
    if origin_match:
        xyz = origin_match.group(1)
        rpy = origin_match.group(2)
    else:
        xyz = "0 0 0"
        rpy = "0 0 0"

    # Use a default small box size - can be adjusted per link if needed
    # For now, use a reasonable default that works for most robot parts
    box_size = "0.1 0.1 0.1"

    # Replace the entire collision block with simplified box
    replacement = f"""        <collision>
            <origin xyz="{xyz}" rpy="{rpy}" />
            <geometry>
                <box>
                    <size>{box_size}</size>
                </box>
            </geometry>
        </collision>"""

    return replacement


def replace_collision_meshes_in_urdf(urdf_content):
    """Replace all collision mesh geometries with simplified boxes."""
    # Direct regex replacement: find mesh tags within collision blocks and replace
    # Pattern: <mesh filename="..."/> within <collision>...</collision>

    # First, replace mesh tags that are within collision blocks
    # Match: <mesh filename="..."/> and replace with box
    def replace_mesh_in_collision(match):
        # Extract indentation from the mesh line
        mesh_line = match.group(1)
        indent = len(mesh_line) - len(mesh_line.lstrip())

        # Replace mesh with box
        return f"""{' ' * indent}<box>
{' ' * (indent + 4)}<size>0.1 0.1 0.1</size>
{' ' * indent}</box>"""

    # Pattern to match mesh tags (with any whitespace/indentation)
    # This will match mesh tags anywhere, but we'll only process those in collision blocks

    # Process line by line, tracking state
    lines = urdf_content.split("\n")
    result_lines = []
    in_collision = False
    in_geometry = False

    for line in lines:
        stripped = line.strip()

        # Track collision block state
        if "<collision>" in stripped:
            in_collision = True
            result_lines.append(line)
        elif "</collision>" in stripped:
            in_collision = False
            in_geometry = False
            result_lines.append(line)
        elif in_collision and "<geometry>" in stripped:
            in_geometry = True
            result_lines.append(line)
        elif in_collision and "</geometry>" in stripped:
            in_geometry = False
            result_lines.append(line)
        elif in_collision and in_geometry and "<mesh" in stripped:
            # Replace mesh with box, preserving indentation
            indent = len(line) - len(line.lstrip())
            result_lines.append(" " * indent + "<box>")
            result_lines.append(" " * (indent + 4) + "<size>0.1 0.1 0.1</size>")
            result_lines.append(" " * indent + "</box>")
        else:
            result_lines.append(line)

    return "\n".join(result_lines)


def main():
    if len(sys.argv) < 2:
        print("Usage: replace_collision_meshes.py <urdf_file>")
        print("  urdf_file: Path to URDF file to process")
        sys.exit(1)

    urdf_file = Path(sys.argv[1])

    if not urdf_file.exists():
        print(f"Error: File not found: {urdf_file}")
        sys.exit(1)

    print(f"Reading {urdf_file}...")
    with open(urdf_file) as f:
        urdf_content = f.read()

    # Count collision meshes before
    mesh_count = len(re.findall(r"<collision>.*?<mesh", urdf_content, re.DOTALL))
    print(f"Found {mesh_count} collision meshes to replace")

    print("Replacing collision meshes with simplified boxes...")
    modified_content = replace_collision_meshes_in_urdf(urdf_content)

    # Verify replacement
    remaining_meshes = len(re.findall(r"<collision>.*?<mesh", modified_content, re.DOTALL))
    box_count = len(re.findall(r"<collision>.*?<box", modified_content, re.DOTALL))

    print(f"Replaced {mesh_count - remaining_meshes} collision meshes")
    print(f"Found {box_count} box collision geometries")

    # Create backup
    backup_file = urdf_file.with_suffix(urdf_file.suffix + ".collision_backup")
    print(f"Creating backup: {backup_file}")
    with open(backup_file, "w") as f:
        f.write(urdf_content)

    # Write modified content
    print(f"Writing modified URDF to {urdf_file}...")
    with open(urdf_file, "w") as f:
        f.write(modified_content)

    print("Done!")
    print("\nNote: All collision meshes have been replaced with 0.1x0.1x0.1m boxes.")
    print("You may need to adjust box sizes for specific links if needed.")


if __name__ == "__main__":
    main()
