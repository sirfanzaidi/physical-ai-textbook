# Chapter 7: Gazebo & Ignition Simulation

## Learning Objectives

After completing this chapter, you will be able to:

- **Understand** Gazebo/Ignition architecture, rendering pipeline, and physics simulation
- **Configure** physics engines (ODE, Bullet, DART) for accurate robot dynamics
- **Build** custom simulation environments with multiple robots and sensors
- **Debug** simulation-reality gaps using sensor simulation and noise modeling

## Overview

Gazebo and its next-generation replacement Ignition (now Gazebo v11+) are the industry-standard open-source 3D robotics simulators. They provide high-fidelity physics, sensor simulation, and integration with ROS 2, making them essential for developing and testing humanoid robots before hardware deployment.

## Why Simulation Matters

Simulation accelerates robot development by orders of magnitude:

### Cost and Safety
- **Hardware preservation**: Test dangerous algorithms safely in simulation first
- **Parallel development**: Multiple teams simulate different components simultaneously
- **Iteration speed**: Run thousands of simulations in time it takes to run one real experiment
- **Rare event testing**: Safely explore edge cases and failure modes

### Physics-Based Development
- **Physics validation**: Verify algorithms work with accurate dynamics before hardware
- **Parameter tuning**: Optimize joint stiffness, damping, friction in simulation
- **Sensor modeling**: Add realistic noise, latency, and dropout to test robustness
- **Actuator simulation**: Model motor dynamics, torque limits, mechanical backlash

## Gazebo Architecture

Gazebo is built on three core components:

### 1. Physics Engine
Gazebo supports multiple physics backends:

| Engine | Accuracy | Speed | Use Case |
|--------|----------|-------|----------|
| **ODE** | Medium | Fast | General robotics, development |
| **Bullet** | High | Medium | Complex collisions, soft bodies |
| **DART** | Very High | Slower | Research, humanoid dynamics |
| **Physics Plugins** | Varies | Varies | Custom physics (water, deformable) |

**Configuration Example**:
```xml
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <gravity>0 0 -9.81</gravity>
</physics>
```

### 2. Rendering Engine
- **GPU-accelerated visualization** using OpenGL
- **Headless mode** for CI/CD pipelines (no display needed)
- **Plugins for custom visuals** (point clouds, semantic segmentation)
- **Camera and sensor visualization** for debugging

### 3. Communication Layer
- **gazebo_ros_pkgs**: Bridges Gazebo and ROS 2 topics/services
- **World states**: Subscribe to /gazebo/link_states for ground truth
- **Services**: Spawn/delete models, reset simulation, pause/unpause

## Practical: Building a Humanoid Simulation

### Step 1: Create a World File
```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="humanoid_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Spawn humanoid robot -->
    <model name="humanoid">
      <pose>0 0 1.0 0 0 0</pose>
      <include>
        <uri>model://my_humanoid_robot</uri>
      </include>
    </model>
  </world>
</sdf>
```

### Step 2: Launch in ROS 2
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=humanoid_world.sdf
```

### Step 3: Publish Joint Commands
```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
publisher = node.create_publisher(JointTrajectory, '/humanoid/trajectory_controller/joint_trajectory', 10)

# Command all joints to stand up
trajectory = JointTrajectory()
trajectory.joint_names = ['hip_pitch_l', 'hip_pitch_r', 'knee_l', 'knee_r', ...]
point = JointTrajectoryPoint(
    positions=[0.0] * len(trajectory.joint_names),
    time_from_start=Duration(seconds=1.0)
)
trajectory.points.append(point)
publisher.publish(trajectory)
```

## Advanced Topics

### Contact Simulation
- **Friction models**: Coulomb, viscous, combined
- **Contact dynamics**: Impulse-based, constraint-based
- **Surface properties**: Restitution, stiffness per material

### Sensor Simulation
- **Camera**: RGB, depth, RGB-D with intrinsic calibration
- **IMU**: Accelerometer, gyro with realistic noise and drift
- **LiDAR**: Ray casting with configurable range and angular resolution
- **Contact sensors**: Force-torque measurement on joints/end-effectors

### Ignition (Gazebo v11+)
Next-generation simulator with improvements:
- **Modular architecture**: Load only needed plugins
- **Better performance**: Improved physics solver stability
- **Synthetic data generation**: For training vision models
- **Multi-physics support**: Fluid dynamics, deformable bodies

## Common Pitfalls

1. **Unrealistic physics**: Too low gravity, frictionless surfaces
2. **Simulation timestep too large**: Causes instability and divergence
3. **Missing sensor noise**: Algorithm works in sim but fails on hardware
4. **Ignoring real-world latencies**: Network delays, sensor processing
5. **Physics engine mismatch**: Different behavior between Gazebo and real robot

## Exercises

**Exercise 7.1**: Build a simple 2-DOF humanoid leg and make it walk in simulation
**Exercise 7.2**: Add a force-torque sensor to the foot and verify ground reaction forces
**Exercise 7.3**: Compare walking with different physics engines (ODE vs Bullet)

## References

1. Gazebo Official: https://gazebosim.org/
2. Ignition Robotics: https://ignitionrobotics.org/
3. Gazebo Tutorials: https://gazebosim.org/docs
4. ROS 2 + Gazebo Integration: https://docs.ros.org/en/humble/Tutorials/Advanced/Gazebo/Gazebo.html
5. Humanoid Locomotion in Simulation (Kajita et al.)
