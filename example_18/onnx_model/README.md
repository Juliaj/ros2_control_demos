# Berkeley-Humanoid-Lite Models

This model is downloaded from https://github.com/HybridRobotics/Berkeley-Humanoid-Lite

## Overview

This ONNX model implements a reinforcement learning policy for bipedal locomotion on the Berkeley Humanoid Lite robot. The policy enables the robot to walk forward and follow user-commanded velocities, such as "walk forward at 0.5 m/s" or "turn left while moving."

## Model Background

The model is based on research from https://lite.berkeley-humanoid.org/static/paper/demonstrating-berkeley-humanoid-lite.pdf

The locomotion task is formulated as a Partially Observed Markov Decision Process (POMDP) and trained using Proximal Policy Optimization (PPO). The policy achieves direct sim-to-real transfer from Isaac Gym training to the physical robot without additional state estimation methods.

Policy inputs:
- Base angular velocity
- Projected gravity vector
- Joint positions and velocities
- User-commanded linear velocity
- Previous time-step action

Policy outputs:
- Desired joint positions for actuators

The multi-layer perceptron (MLP) policy runs at 25 Hz during deployment. Experiments show the robot operates at only 30% of actuator torque limits, indicating significant capacity for larger humanoid designs.
