# Generate URDF from Mujoco Model file


## Basic URDF concepts

Links (<link>) represent physical parts with mass, inertia, and visual/collision geometry (e.g., wheels, base, leg segments).
Joints (<joint>) are conceptual relationships that define how links connect: parent/child, axis of rotation/translation, limits, and the transform between link frames.

A link frame is a coordinate system (origin + x/y/z axes) attached to each link. It defines:
- Origin: a fixed point on the link (often the link’s center of mass or a joint attachment point)
- Orientation: three orthogonal axes (x, y, z) that define the link’s orientation in space

In URDF:each <link> has its own frame. Visual/collision geometry and inertial properties are defined relative to that link’s frame. The joint’s <origin> specifies the transform from the parent link’s frame to the child link’s frame. The rpy in a joint’s <origin> defines the orientation of the child link’s frame relative to the parent’s frame. This sets the child frame’s orientation, so all geometry defined in the child link’s frame (visual meshes, inertial properties) is positioned and oriented accordingly.

rpy maps to rotations around x, y, z axes:
- r (roll) = rotation around the x-axis
- p (pitch) = rotation around the y-axis
- y (yaw) = rotation around the z-axis

The order of applying rpy matters. Rotations are not commutative, so applying the same angles in different orders yields different final orientations.
In URDF, rpy is always applied in fixed-axis order: roll (x) → pitch (y) → yaw (z). Changing the order would change the result.

Question: The <origin> tag in a joint defines the transform from the parent link's frame to the child link's frame. In the left/right hip pitch alignment issue, how does the rpy (roll-pitch-yaw) in the joint's <origin> affect the orientation of the child link's coordinate frame relative to the parent, and why would different rpy values between left and right joints cause the legs to point in different directions even if the xyz translation is symmetric?
those are good questions. My guess is that the rpy are used for calcuting the points within the child frame. by the way, when you say link frames, what does it exactly mean ? 

Answer: Different joint-level rpy values create different child link frame orientations. Even if mesh-level rpy values are the same, the final transformation differs because:The mesh-level rpy is applied in the child link's frame'. That frame's orientation depends on the joint's rpy. Same mesh-level rotation in different frames yields different final orientations

Question: In the left_hip_pitch_link, the visual meshes have their own <origin> tags (e.g., xyz="0.016060 0.065000 0.109150" rpy="-1.570796 0.000000 0.000000"). Are these mesh origins defined relative to the link’s frame, or relative to the parent link’s frame? How does the joint’s rpy interact with these mesh-level rpy values to determine the final visual appearance?

Answer: Mesh origins are defined relative to the parent link's frame (not the child link's frame).
Transformations are applied sequentially:
- First: joint's rpy (defines the child link's frame orientation relative to parent)
- Then: mesh-level rpy (applied within that child link's frame)



