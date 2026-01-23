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
Convert MuJoCo XML file to URDF with visual mesh geometry.

This script extracts the kinematic structure and visual mesh geometry from
a MuJoCo XML file and generates a URDF file suitable for RViz visualization.
"""

import argparse
import xml.etree.ElementTree as ET
import math
import os
from pathlib import Path


def quat_to_rpy(w, x, y, z):
    """Convert quaternion (w, x, y, z) to roll, pitch, yaw (RPY) in radians."""
    # Normalize quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0:
        return (0.0, 0.0, 0.0)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    # Convert to RPY
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return (roll, pitch, yaw)


def parse_vec3(attr_str, default=(0.0, 0.0, 0.0)):
    """Parse a space-separated 3D vector string."""
    if not attr_str:
        return default
    parts = attr_str.strip().split()
    if len(parts) >= 3:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    return default


def parse_quat(attr_str, default=(1.0, 0.0, 0.0, 0.0)):
    """Parse a space-separated quaternion string (w, x, y, z)."""
    if not attr_str:
        return default
    parts = attr_str.strip().split()
    if len(parts) >= 4:
        return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
    return default


def get_meshdir_from_xml(root):
    """Extract meshdir from compiler tag."""
    compiler = root.find(".//compiler")
    if compiler is not None:
        meshdir = compiler.get("meshdir", "assets")
        return meshdir
    return "assets"


def build_mesh_name_to_file_mapping(root):
    """Build a mapping from mesh name to filename from asset definitions."""
    mesh_map = {}
    asset = root.find("asset")
    if asset is not None:
        for mesh_elem in asset.findall("mesh"):
            file_attr = mesh_elem.get("file")
            if file_attr:
                # Extract filename (may include path, but we only need the basename)
                filename = os.path.basename(file_attr)
                # Get mesh name - if name attribute exists, use it; otherwise derive from filename
                mesh_name = mesh_elem.get("name")
                if not mesh_name:
                    # MuJoCo uses filename without extension as mesh name
                    mesh_name = os.path.splitext(filename)[0]
                mesh_map[mesh_name] = filename
    return mesh_map


def find_body_by_name(root, body_name):
    """Find a body element by name recursively."""
    for body in root.iter("body"):
        if body.get("name") == body_name:
            return body
    return None


def extract_visual_geoms(body_elem):
    """
    Extract visual geometry elements from a body.

    Assumptions:
    1. Visual geoms are identified by class="visual" or type="mesh" with contype="0"
    2. Geom positions (pos) are relative to the body frame (not absolute)
    3. Geom orientations (quat) are relative to the body frame
    4. In URDF, these positions/orientations are used directly as visual mesh origins
       relative to the link frame (which is at the joint location)
    5. This is correct because the body frame in MuJoCo IS the link frame in URDF
    """
    visuals = []
    for geom in body_elem.findall("geom"):
        geom_class = geom.get("class", "")
        geom_type = geom.get("type", "")
        if "visual" in geom_class or (geom_type == "mesh" and geom.get("contype") == "0"):
            mesh_name = geom.get("mesh")
            if mesh_name:
                # Geom position is relative to body frame
                # This will be used as visual origin relative to link frame in URDF
                pos = parse_vec3(geom.get("pos"))
                quat = parse_quat(geom.get("quat"))
                visuals.append(
                    {
                        "mesh": mesh_name,
                        "pos": pos,
                        "quat": quat,
                    }
                )
    return visuals


def extract_joint_info(body_elem):
    """
    Extract joint information from a body element.

    Assumptions:
    1. Each body has at most one joint (MuJoCo convention)
    2. Joint is a direct child element of the body
    3. Joint type "hinge" maps to URDF "revolute"
    4. Joint axis defaults to Z-axis (0 0 1) if not specified
    5. Joint range is space-separated "min max" string
    """
    joint = body_elem.find("joint")
    if joint is None:
        return None

    joint_name = joint.get("name")
    if not joint_name:
        # Joint without name is invalid - return None
        return None

    joint_type = joint.get("type", "hinge")
    joint_range = joint.get("range", "")
    # Default axis is Z-axis (0 0 1) - common for revolute joints
    axis = parse_vec3(joint.get("axis", "0 0 1"))

    # Parse range if available
    # Format: "min max" as space-separated floats
    range_min, range_max = None, None
    if joint_range:
        parts = joint_range.split()
        if len(parts) >= 2:
            try:
                range_min = float(parts[0])
                range_max = float(parts[1])
            except ValueError:
                # Invalid range format - leave as None
                pass

    return {
        "name": joint_name,
        "type": "revolute" if joint_type == "hinge" else "fixed",
        "range": (range_min, range_max),
        "axis": axis,
    }


def extract_inertial(body_elem):
    """Extract inertial properties from a body."""
    inertial = body_elem.find("inertial")
    if inertial is None:
        return None

    mass = float(inertial.get("mass", "0.01"))
    pos = parse_vec3(inertial.get("pos"))
    inertia_str = inertial.get("fullinertia", "")
    # Parse fullinertia (ixx, iyy, izz, ixy, ixz, iyz)
    ixx, iyy, izz, ixy, ixz, iyz = 0.001, 0.001, 0.001, 0.0, 0.0, 0.0
    if inertia_str:
        parts = inertia_str.split()
        if len(parts) >= 6:
            ixx, iyy, izz, ixy, ixz, iyz = (float(p) for p in parts[:6])

    return {
        "mass": mass,
        "pos": pos,
        "inertia": (ixx, iyy, izz, ixy, ixz, iyz),
    }


def get_link_name_from_joint(joint_name):
    """
    Map joint name to URDF link name.

    Assumptions:
    1. Joint names follow a consistent naming pattern
    2. Link names are derived by appending "_link" to joint names
    3. Some joints have explicit mappings (e.g., left_hip_yaw -> left_hip_yaw_link)
    4. Unknown joints use fallback pattern: {joint_name}_link
    """
    if not joint_name:
        return None

    # Explicit joint name to link name mapping
    # This ensures correct link names for known joints, especially for left/right pairs
    joint_to_link = {
        "left_hip_yaw": "left_hip_yaw_link",
        "left_hip_roll": "left_hip_roll_link",
        "left_hip_pitch": "left_hip_pitch_link",
        "left_knee": "left_knee_link",
        "left_ankle": "left_ankle_link",
        "right_hip_yaw": "right_hip_yaw_link",
        "right_hip_roll": "right_hip_roll_link",
        "right_hip_pitch": "right_hip_pitch_link",
        "right_knee": "right_knee_link",
        "right_ankle": "right_ankle_link",
        "neck_pitch": "neck_pitch_link",
        "head_pitch": "head_pitch_link",
        "head_yaw": "head_yaw_link",
        "head_roll": "head_roll_link",
    }

    # Use explicit mapping if available, otherwise use pattern-based fallback
    return joint_to_link.get(joint_name, f"{joint_name}_link")


def build_body_hierarchy(root, parent_body=None, parent_link_name="base"):
    """
    Recursively build body hierarchy and extract link/joint information.

    Assumptions:
    1. MuJoCo body structure: Each body with a joint represents a URDF link
    2. Body position (pos) in MuJoCo is relative to parent body frame
    3. Body position directly maps to URDF joint origin (transform from parent to child link)
    4. Bodies are processed in XML order, maintaining parent-child relationships
    5. Joint names follow a pattern that can be mapped to link names via get_link_name_from_joint()
    """
    bodies = []
    if parent_body is None:
        # Start from worldbody
        worldbody = root.find("worldbody")
        if worldbody is None:
            return bodies

        # Find trunk_assembly - this is the root body for the robot
        # Assumption: trunk_assembly exists and is a direct child of worldbody
        trunk_body = find_body_by_name(worldbody, "trunk_assembly")
        if trunk_body is not None:
            parent_body = trunk_body
            parent_link_name = "trunk_assembly"
        else:
            return bodies

    body_name = parent_body.get("name", "")
    if not body_name:
        return bodies

    # Extract joint info to determine link name
    # Assumption: Bodies with joints represent movable links; bodies without joints are fixed
    joint_info = extract_joint_info(parent_body)
    if joint_info and joint_info["name"]:
        # Use joint name to determine link name (e.g., "left_hip_yaw" -> "left_hip_yaw_link")
        link_name = get_link_name_from_joint(joint_info["name"])
    else:
        # Fallback: use body name for bodies without joints (e.g., trunk_assembly)
        if body_name == "trunk_assembly":
            link_name = "trunk_assembly"
        else:
            # For bodies without explicit joints, derive link name from body name
            # Note: This handles cases like "hip_roll_assembly_2" -> "hip_roll_link"
            # but should not be used if joint_info exists (which maps to correct link name)
            link_name = (
                body_name.replace("_assembly", "_link")
                .replace("_2", "")
                .replace("_3", "")
                .replace("_4", "")
            )

    # Extract body properties
    # Assumption: Body position (pos) in MuJoCo is the transform from parent body frame
    # to this body's frame. This directly maps to URDF joint origin.
    body_pos = parse_vec3(parent_body.get("pos"))
    body_quat = parse_quat(parent_body.get("quat"))
    visual_geoms = extract_visual_geoms(parent_body)
    inertial_info = extract_inertial(parent_body)

    body_data = {
        "name": body_name,
        "link_name": link_name,
        "parent": parent_link_name,
        "pos": body_pos,  # Used as joint origin in URDF
        "quat": body_quat,  # Used as joint origin rotation in URDF
        "visuals": visual_geoms,
        "joint": joint_info,
        "inertial": inertial_info,
        "children": [],
    }

    # Process child bodies recursively
    # Assumption: All direct child <body> elements in XML are kinematic children
    # This processes bodies in XML order, which should match the kinematic structure
    # IMPORTANT: Pass link_name as parent so children know their parent link
    # This ensures correct parent-child relationships in the URDF joint definitions
    for child_body in parent_body.findall("body"):
        # Recursively process child bodies, passing the current link_name as the parent
        # This ensures the child's "parent" field is set to the correct parent link name
        child_data = build_body_hierarchy(root, child_body, link_name)
        if child_data:
            # Extend to handle multiple children (e.g., left and right legs)
            body_data["children"].extend(child_data)

    bodies.append(body_data)
    return bodies


def generate_urdf_link(link_data, meshdir, package_name, mesh_map):
    """Generate URDF link XML from link data."""
    link_name = link_data["link_name"]
    visuals = link_data["visuals"]
    inertial = link_data["inertial"]

    link_xml = f'  <link name="{link_name}">\n'

    # Add visual elements
    if visuals:
        for i, visual in enumerate(visuals):
            mesh_name = visual["mesh"]
            pos = visual["pos"]
            quat = visual["quat"]
            rpy = quat_to_rpy(*quat)

            # Look up mesh filename from mapping, fallback to mesh_name.stl
            mesh_file = mesh_map.get(mesh_name, f"{mesh_name}.stl")

            # Always use package:// URI - ROS2 will resolve it dynamically using ament_index
            # This works regardless of install mode (symlink or copy) and is the standard ROS2 approach
            mesh_uri = f"package://{package_name}/description/assets/{mesh_file}"

            link_xml += "    <visual>\n"
            link_xml += "      <geometry>\n"
            link_xml += f'        <mesh filename="{mesh_uri}"/>\n'
            link_xml += "      </geometry>\n"
            link_xml += f'      <origin xyz="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}" '
            link_xml += f'rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>\n'
            link_xml += "    </visual>\n"
    else:
        # Fallback to simple box if no visuals
        link_xml += "    <visual>\n"
        link_xml += "      <geometry>\n"
        link_xml += '        <box size="0.05 0.05 0.05"/>\n'
        link_xml += "      </geometry>\n"
        link_xml += "    </visual>\n"

    # Add inertial
    if inertial:
        mass = inertial["mass"]
        pos = inertial["pos"]
        ixx, iyy, izz, ixy, ixz, iyz = inertial["inertia"]
        link_xml += "    <inertial>\n"
        link_xml += f'      <mass value="{mass:.6f}"/>\n'
        link_xml += f'      <origin xyz="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}"/>\n'
        link_xml += f'      <inertia ixx="{ixx:.6f}" ixy="{ixy:.6f}" ixz="{ixz:.6f}" '
        link_xml += f'iyy="{iyy:.6f}" iyz="{iyz:.6f}" izz="{izz:.6f}"/>\n'
        link_xml += "    </inertial>\n"
    else:
        # Minimal inertial
        link_xml += "    <inertial>\n"
        link_xml += '      <mass value="0.01"/>\n'
        link_xml += '      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" '
        link_xml += 'iyy="0.0001" iyz="0.0" izz="0.0001"/>\n'
        link_xml += "    </inertial>\n"

    link_xml += "  </link>\n"
    return link_xml


def generate_urdf_joint(joint_data, link_data):
    """
    Generate URDF joint XML from joint data.

    Assumptions:
    1. Joint origin uses the body's position and orientation from MuJoCo
    2. Body position (pos) in MuJoCo represents the transform from parent to child frame
    3. This directly maps to URDF joint origin (transform from parent link to child link)
    4. Joint axis is extracted from MuJoCo joint definition
    5. Joint limits come from MuJoCo joint range attribute
    """
    joint = joint_data["joint"]
    if joint is None:
        return ""

    joint_name = joint["name"]
    joint_type = joint["type"]
    parent = link_data["parent"]
    child = link_data["link_name"]

    # Use body position and orientation as joint origin
    # Assumption: In MuJoCo, body pos/quat define the frame transform from parent to child
    # This is exactly what URDF joint origin needs (transform from parent link to child link)
    pos = link_data["pos"]
    quat = link_data["quat"]
    rpy = quat_to_rpy(*quat)
    axis = joint["axis"]
    range_min, range_max = joint["range"]

    joint_xml = f'  <joint name="{joint_name}" type="{joint_type}">\n'
    joint_xml += f'    <parent link="{parent}"/>\n'
    joint_xml += f'    <child link="{child}"/>\n'
    # Joint origin: transform from parent link frame to child link frame
    # This uses the body's position and orientation from MuJoCo
    joint_xml += f'    <origin xyz="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}" '
    joint_xml += f'rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>\n'
    joint_xml += f'    <axis xyz="{axis[0]:.6f} {axis[1]:.6f} {axis[2]:.6f}"/>\n'

    if joint_type == "revolute" and range_min is not None and range_max is not None:
        joint_xml += f'    <limit lower="{range_min:.17f}" upper="{range_max:.17f}" '
        joint_xml += 'effort="3.23" velocity="10.0"/>\n'

    joint_xml += "  </joint>\n"
    return joint_xml


def generate_urdf(hierarchy, meshdir, package_name, mesh_map):
    """
    Generate complete URDF XML from hierarchy.

    Assumptions:
    1. Hierarchy is a flat list of top-level bodies (starting from trunk_assembly)
    2. Each body contains its children in the "children" field
    3. Bodies are processed recursively to maintain parent-child relationships
    4. The trunk_assembly is the root body and gets a fixed joint to base link
    """
    urdf = '<?xml version="1.0"?>\n'
    urdf += "<!-- URDF generated from MuJoCo XML with visual mesh geometry -->\n"
    urdf += '<robot name="open-duck-mini">\n'
    urdf += '  <link name="base"/>\n\n'

    # Generate links and joints recursively
    # This processes the tree structure: each body generates its link, then its joint (if any),
    # then recursively processes all its children
    def process_body(body_data):
        # Generate link first (child link must exist before joint references it)
        urdf_part = generate_urdf_link(body_data, meshdir, package_name, mesh_map)
        urdf_part += "\n"

        # Generate joint (if not root body)
        # Assumption: Bodies without joints are fixed (like trunk_assembly)
        if body_data["joint"] is not None:
            urdf_part += generate_urdf_joint(body_data, body_data)
            urdf_part += "\n"

        # Process children recursively
        # This ensures all kinematic chains are processed (e.g., left leg, right leg, head)
        for child in body_data["children"]:
            urdf_part += process_body(child)

        return urdf_part

    # Find trunk_assembly (root body)
    # Assumption: trunk_assembly exists in hierarchy and is the root of the kinematic tree
    trunk_data = None
    for body in hierarchy:
        if body["link_name"] == "trunk_assembly":
            trunk_data = body
            break

    if trunk_data:
        # Add fixed joint from base to trunk
        # Assumption: Base link is at origin, trunk is offset by 0.22m in Z (from MuJoCo worldbody)
        urdf += '  <joint name="base_to_trunk" type="fixed">\n'
        urdf += '    <parent link="base"/>\n'
        urdf += '    <child link="trunk_assembly"/>\n'
        urdf += '    <origin xyz="0 0 0.22" rpy="0 0 0"/>\n'
        urdf += "  </joint>\n\n"

        # Process all bodies starting from trunk_assembly
        # This will recursively process all children (left leg, right leg, head, etc.)
        for body in hierarchy:
            urdf += process_body(body)
    else:
        # Fallback: process all bodies even if trunk_assembly not found
        # This should not happen with correct MuJoCo XML structure
        for body in hierarchy:
            urdf += process_body(body)

    urdf += "</robot>\n"
    return urdf


def main():
    parser = argparse.ArgumentParser(
        description="Convert MuJoCo XML to URDF with visual mesh geometry"
    )
    parser.add_argument(
        "input_xml",
        type=str,
        help="Input MuJoCo XML file path",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output URDF file path (default: input filename with .urdf extension)",
    )
    parser.add_argument(
        "-p",
        "--package",
        type=str,
        default="ros2_control_demo_example_18",
        help="ROS package name for mesh URIs (default: ros2_control_demo_example_18)",
    )

    args = parser.parse_args()

    # Parse input XML
    tree = ET.parse(args.input_xml)
    root = tree.getroot()

    # Extract meshdir
    meshdir = get_meshdir_from_xml(root)

    # Build mesh name to filename mapping
    mesh_map = build_mesh_name_to_file_mapping(root)

    # Build body hierarchy
    # This recursively processes all bodies starting from trunk_assembly
    # Assumption: All kinematic bodies are children of trunk_assembly
    hierarchy = build_body_hierarchy(root)

    if not hierarchy:
        print("Error: No bodies found in hierarchy. Check if trunk_assembly exists in XML.")
        return

    # Count total bodies (including children) for verification
    def count_bodies(bodies):
        count = len(bodies)
        for body in bodies:
            count += count_bodies(body.get("children", []))
        return count

    total_bodies = count_bodies(hierarchy)
    print(f"Found {len(hierarchy)} top-level bodies, {total_bodies} total bodies in hierarchy")

    # Generate URDF with package:// URIs
    # ROS2 will resolve package:// URIs dynamically using ament_index at runtime
    urdf_content = generate_urdf(hierarchy, meshdir, args.package, mesh_map)

    # Write output
    if args.output:
        output_path = args.output
    else:
        input_path = Path(args.input_xml)
        output_path = input_path.with_suffix(".urdf")

    with open(output_path, "w") as f:
        f.write(urdf_content)

    print(f"Generated URDF: {output_path}")


if __name__ == "__main__":
    main()

# DO NOT MODIFY or DELETE
# Usage
# cd ~/ros2_ws/src/ros-controls/ros2_control_demos/example_18/description
# python3 scripts/mujoco_xml_to_urdf.py mujoco/open_duck_mini_v2.xml -o mujoco/robot.urdf -p ros2_control_demo_example_18
