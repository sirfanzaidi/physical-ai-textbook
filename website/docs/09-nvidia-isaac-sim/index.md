# Chapter 9: NVIDIA Isaac Sim

## Learning Objectives

After completing this chapter, you will be able to:

- **Understand** NVIDIA's GPU-accelerated simulation platform and PhysX physics engine
- **Build** large-scale multi-robot simulation environments with realistic rendering
- **Generate** synthetic training data (images, point clouds, poses) for AI models
- **Execute** sim-to-real transfer workflows using domain randomization

## Overview

NVIDIA Isaac Sim is a next-generation simulation platform built on Omniverse, delivering GPU-accelerated physics, photorealistic rendering, and synthetic data generation at scale. It's designed for training and validating humanoid robots with unprecedented realism and speed.

## Isaac Sim vs Gazebo

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Physics Engine** | ODE/Bullet/DART | PhysX (GPU) |
| **Rendering** | OpenGL | RTX Ray Tracing |
| **Simulation Speed** | 1-10x realtime | 10-100x realtime (GPU) |
| **Synthetic Data** | Plugins | Native (synthetic sensors) |
| **Multi-Robot** | Moderate | Massive scale |
| **Cost** | Free | Free (cloud), paid (enterprise) |
| **Learning Curve** | Easy | Moderate |

## Architecture

### USD (Universal Scene Description)
Isaac Sim uses USD/USDĀ as its native format:
```python
from pxr import Usd, UsdGeom, UsdPhysics

# Create a stage (world)
stage = Usd.Stage.CreateNew("my_scene.usd")

# Add a cube
xform = UsdGeom.Xform.Define(stage, "/World/Cube")
cube = UsdGeom.Cube.Define(stage, "/World/Cube/Geometry")
cube.GetSizeAttr().Set(0.2)

# Add physics
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

stage.Save()
```

### PhysX Physics
GPU-accelerated physics solver enabling:
- **Massive scale**: Thousands of objects simultaneously
- **High precision**: Accurate rigid body dynamics
- **Real-time**: Runs at 100+ Hz on GPU
- **Deterministic**: Same simulation always produces identical results

## Humanoid Robot in Isaac Sim

### Step 1: Import Robot URDF
```python
import omni.kit.app
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World

# Create Isaac world
world = World(stage_units_in_meters=1.0)

# Import humanoid from URDF
add_reference_to_stage(
    usd_path="/isaac_sim/robots/my_humanoid.usd",
    prim_path="/World/humanoid",
    position=[0, 0, 1]
)

world.reset()
```

### Step 2: Control Joints
```python
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.robots import Robot

# Get robot instance
robot = Robot(prim_path="/World/humanoid", name="humanoid")

# Command joint positions
positions = [0.0] * 23  # 23 DOF humanoid
robot.set_joint_positions(positions)

world.step(render=True)  # Advance simulation
```

### Step 3: Synthetic Data Generation
```python
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Setup camera and synthetic data
camera = world.scene.add(
    Camera(
        prim_path="/World/camera",
        position=[2, 2, 2],
        orientation=[0, 0, 0.707, 0.707]
    )
)

# Generate RGB-D data
rgb = camera.get_rgb()  # Shape: (H, W, 4)
depth = camera.get_depth()  # Shape: (H, W, 1)
instance_segmentation = camera.get_instance_segmentation()
```

## Domain Randomization for Sim-to-Real

Key randomization parameters:
```python
import random

def randomize_environment():
    """Apply domain randomization for robust training"""

    # Randomize physics
    random_mass_scale = random.uniform(0.8, 1.2)
    random_friction = random.uniform(0.3, 1.0)

    # Randomize visuals
    random_lighting = random.uniform(0.3, 1.0)
    random_texture = random.choice(['metal', 'plastic', 'rubber'])

    # Randomize robot properties
    random_joint_friction = random.uniform(0.01, 0.1)
    random_joint_damping = random.uniform(0.1, 1.0)

    return {
        'mass_scale': random_mass_scale,
        'friction': random_friction,
        'lighting': random_lighting,
        'texture': random_texture,
        'joint_friction': random_joint_friction,
        'joint_damping': random_joint_damping
    }

# Apply randomization to all objects
for i in range(1000):
    params = randomize_environment()
    # ... simulate with these params ...
```

## Training Data Generation

### Pose Estimation Dataset
```python
from omni.isaac.core.utils.numpy import quat_to_euler_angles

# Collect training data
training_data = []

for episode in range(10000):
    # Reset simulation
    world.reset()

    # Random initial pose
    random_positions = [random.uniform(-1, 1) for _ in range(23)]
    robot.set_joint_positions(random_positions)

    # Capture data
    rgb = camera.get_rgb()
    poses = robot.get_joint_positions()

    training_data.append({
        'image': rgb,
        'joint_positions': poses,
        'episode': episode
    })

# Save for training with VLA model
import numpy as np
np.save('training_data.npy', training_data)
```

### Object Detection Dataset
```python
# Generate diverse object scenarios
for i in range(5000):
    # Add random objects
    for _ in range(random.randint(3, 8)):
        shape = random.choice(['cube', 'sphere', 'cylinder'])
        position = [random.uniform(-2, 2) for _ in range(3)]
        # Add to scene...

    # Capture with bounding boxes
    rgb = camera.get_rgb()
    bboxes = camera.get_bounding_boxes()  # Automatic annotation

    # Save training pair
    save_training_pair(rgb, bboxes, label=f'object_detection_{i}')
```

## Performance Optimization

### Headless Mode (for training)
```bash
# Run Isaac Sim without visualization
./isaac-sim.sh --headless --python physics_training.py
```

### Batch Simulation
Run multiple independent environments in parallel:
```python
from omni.isaac.core.envs import VectorEnv

# Create 256 parallel environments on GPU
env = VectorEnv(num_envs=256, device='cuda:0')

# Step all environments simultaneously
observations = env.reset()
for _ in range(1000):
    actions = policy.predict(observations)  # Batched
    observations = env.step(actions)  # All 256 envs step at once
```

## Integration with ML Pipelines

### Connect to TensorFlow/PyTorch
```python
import torch
from torch.utils.data import DataLoader

# Convert Isaac synthetic data to PyTorch dataset
dataset = IsaacSyntheticDataset('training_data.npy')
dataloader = DataLoader(dataset, batch_size=64, shuffle=True)

# Train vision model
for epoch in range(10):
    for images, poses in dataloader:
        predictions = model(images.cuda())
        loss = criterion(predictions, poses.cuda())
        loss.backward()
        optimizer.step()
```

## Exercises

**Exercise 9.1**: Create a simple humanoid scene and generate 1000 random poses with synthetic images
**Exercise 9.2**: Implement domain randomization for friction and mass
**Exercise 9.3**: Generate object detection dataset with 500 randomized scenes

## References

1. NVIDIA Isaac Sim: https://developer.nvidia.com/isaac-sim
2. Omniverse Platform: https://www.nvidia.com/en-us/omniverse/
3. Isaac SDK Documentation: https://docs.omniverse.nvidia.com/isaacsim/
4. Domain Randomization (Tobin et al., 2017): https://arxiv.org/abs/1703.06907
5. Sim-to-Real Transfer Learning: https://arxiv.org/abs/1604.04671
