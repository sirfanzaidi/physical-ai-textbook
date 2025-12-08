# Chapter 8: Humanoid Robot Modeling

## Learning Objectives

After completing this chapter, you will be able to:

- **Design** URDF models representing complete humanoid robots with accurate kinematics
- **Validate** kinematic chains using forward and inverse kinematics solvers
- **Model** actuators, friction, and damping for physics-accurate simulation
- **Debug** model issues using visualization tools and dynamic verification

## Overview

A precise robot model is the foundation of successful humanoid control. This chapter covers creating, validating, and testing detailed CAD-derived models that accurately represent robot structure, mass distribution, and actuator capabilities.

## URDF (Unified Robot Description Format)

URDF is the standard XML format for describing robot structure in ROS 2.

### Basic Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_v1">
  <!-- Base link -->
  <link name="pelvis">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Left leg chain -->
  <joint name="hip_pitch_l" type="revolute">
    <parent link="pelvis"/>
    <child link="upper_leg_l"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <link name="upper_leg_l">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
</robot>
```

### Key Components

| Element | Purpose | Example |
|---------|---------|---------|
| **link** | Rigid body with mass and geometry | `<link name="thigh_l">` |
| **joint** | Connection between links with DOF | `<joint name="knee_l" type="revolute">` |
| **inertial** | Mass properties for dynamics | `<mass value="5.0"/>` |
| **collision** | Geometry for collision detection | `<collision><cylinder radius="0.05" length="0.4"/></collision>` |
| **visual** | Mesh for visualization | `<visual><mesh filename="package://model/thigh.stl"/></visual>` |

## Mass Properties from CAD

Accurate mass and inertia tensors are critical for realistic dynamics.

### Extracting from CAD
1. Export CAD model (STEP/IGES) from design software
2. Use URDF generators: `mesh_to_urdf`, `solidworks_urdf_exporter`
3. Automatically computed inertia matrices from mesh geometry
4. Validate against known physical parameters

### Inertia Tensor Computation
For a box of mass M, width W, height H, depth D:
```
Ixx = M/12 * (H² + D²)
Iyy = M/12 * (W² + D²)
Izz = M/12 * (W² + H²)
```

## Kinematic Chains

### Forward Kinematics
Compute end-effector position given joint angles using transformation matrices.

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def forward_kinematics_leg(theta_hip, theta_knee):
    """Compute foot position from hip and knee angles"""
    # Thigh length: 0.4m, Calf length: 0.4m
    L1, L2 = 0.4, 0.4

    # Hip to knee
    hip_frame = np.array([[1, 0, 0],
                          [0, np.cos(theta_hip), -np.sin(theta_hip)],
                          [0, np.sin(theta_hip), np.cos(theta_hip)]])
    knee_pos = hip_frame @ np.array([0, 0, -L1])

    # Knee to foot
    knee_frame = hip_frame @ np.array([[1, 0, 0],
                                       [0, np.cos(theta_knee), -np.sin(theta_knee)],
                                       [0, np.sin(theta_knee), np.cos(theta_knee)]])
    foot_pos = knee_pos + knee_frame @ np.array([0, 0, -L2])

    return foot_pos
```

### Inverse Kinematics
Find joint angles given desired end-effector position (more complex).

Analytical IK for 2-DOF leg:
```python
def inverse_kinematics_leg(foot_x, foot_z):
    """Compute hip and knee angles from desired foot position"""
    L1, L2 = 0.4, 0.4

    # Distance to foot
    d = np.sqrt(foot_x**2 + foot_z**2)

    # Law of cosines for knee angle
    cos_knee = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta_knee = np.arccos(np.clip(cos_knee, -1, 1))

    # Hip angle
    alpha = np.arctan2(foot_z, foot_x)
    beta = np.arctan2(L2 * np.sin(theta_knee), L1 + L2 * np.cos(theta_knee))
    theta_hip = alpha - beta

    return theta_hip, theta_knee
```

## Model Validation

### Kinematic Verification
```bash
# Check for self-collisions
urdf_check humanoid.urdf

# Visualize in RViz
ros2 run rviz2 rviz2
# Load humanoid.urdf as fixed frame
```

### Dynamic Properties Validation
1. Center of mass location (should be above feet when standing)
2. Joint torque limits (match physical actuator specs)
3. Velocity limits (match motor speed)
4. Mass distribution (should feel natural when picked up)

## Advanced Modeling

### Soft Actuators
Model series elastic actuators (SEA) with virtual springs:
```xml
<joint name="ankle_pitch_l" type="revolute">
  <parent link="calf_l"/>
  <child link="foot_l"/>
  <axis xyz="0 1 0"/>
  <!-- Virtual spring for compliance -->
  <dynamics damping="10.0" friction="0.1"/>
</joint>
```

### Friction Models
Realistic friction affects walking stability:
```xml
<contact>
  <collision name="foot_l">
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>
          <mu2>0.8</mu2>
          <slip1>0.001</slip1>
          <slip2>0.001</slip2>
        </ode>
      </friction>
    </surface>
  </collision>
</contact>
```

## Exercises

**Exercise 8.1**: Create URDF for a 3-DOF arm and compute forward kinematics
**Exercise 8.2**: Validate humanoid model in Gazebo and check for self-collisions
**Exercise 8.3**: Implement inverse kinematics solver for bimanual reaching

## References

1. ROS 2 URDF Documentation: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
2. URDF Standard: http://wiki.ros.org/urdf
3. Kinematics & Robotics (Craig): Chapters 2-3
4. Gazebo Model Format: https://gazebosim.org/docs/latest/sdf_models/
5. MoveIt Motion Planning: https://moveit.ros.org/
