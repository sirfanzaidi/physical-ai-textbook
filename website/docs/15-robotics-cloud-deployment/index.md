# Chapter 15: Robotics Cloud Deployment

## Learning Objectives

After completing this chapter, you will be able to:

- **Design** cloud architectures for distributed robot systems
- **Implement** edge-cloud pipelines for real-time processing
- **Deploy** robot control software using containerization and orchestration
- **Scale** robot fleets across multiple physical locations

## Overview

Modern robotics systems span edge (on-robot) and cloud (data center) infrastructure. This chapter covers deploying robot control stacks in the cloud, coordinating fleets, and streaming telemetry at scale.

## Edge vs Cloud Processing

### Latency Considerations

| Task | Location | Latency | Why |
|------|----------|---------|-----|
| **Motor control loop** | Edge/Robot | 1-10ms | Real-time, safety-critical |
| **Balance & walking** | Edge/Robot | 10-100ms | Local feedback control |
| **Perception** | Edge | 100-500ms | Can process locally, privacy |
| **Planning** | Cloud | 100-1000ms | Heavy computation OK |
| **Monitoring** | Cloud | 1-10s | Aggregation across fleet |
| **Learning** | Cloud | Hours | Training models offline |

### Edge-Cloud Division

```
Robot Hardware
    ↓
[Local Controller] ← 1-100ms
    ↓ (sensor data, joint states)
[Edge AI] ← 100-500ms
    ↓ (plans, detections)
[Cloud Backend] ← 500-5000ms
    ↓ (fleet coordination, analytics)
```

## Container Deployment

### Dockerizing Robot Stack

```dockerfile
# Dockerfile for robotics system
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-control-toolbox \
    ros-humble-moveit2 \
    ros-humble-navigation2

# Install Python packages
RUN pip install \
    torch \
    torchvision \
    numpy \
    scipy \
    pydantic

# Copy robot software
COPY ./src /workspace/src
COPY ./config /workspace/config

# Build ROS packages
WORKDIR /workspace
RUN colcon build

# Entry point
ENTRYPOINT ["ros2", "launch", "robot_system", "robot.launch.py"]
```

### Kubernetes Orchestration

Deploy multiple robot instances across clusters:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: robot-controller
spec:
  replicas: 3  # Run 3 instances
  selector:
    matchLabels:
      app: robot-controller
  template:
    metadata:
      labels:
        app: robot-controller
    spec:
      containers:
      - name: robot-system
        image: robotics/robot-controller:v1.0
        resources:
          requests:
            memory: "2Gi"
            cpu: "2"
          limits:
            memory: "4Gi"
            cpu: "4"
        env:
        - name: ROBOT_ID
          valueFrom:
            fieldRef:
              fieldPath: metadata.name
        - name: CLOUD_ENDPOINT
          value: "api.roboticscloud.com"
```

## Real-Time Streaming Architecture

### Telemetry Pipeline

```python
import asyncio
import json
from datetime import datetime
from motor.motor_asyncio import AsyncIOMotorClient

class RobotTelemetryCollector:
    def __init__(self, robot_id: str, cloud_endpoint: str):
        self.robot_id = robot_id
        self.cloud_endpoint = cloud_endpoint
        self.buffer = []
        self.buffer_size = 100

    async def collect_telemetry(self):
        """Continuously collect sensor data"""
        while True:
            # Sample robot state at 100Hz
            state = {
                'robot_id': self.robot_id,
                'timestamp': datetime.utcnow().isoformat(),
                'joint_angles': self.robot.get_joint_angles(),
                'joint_velocities': self.robot.get_joint_velocities(),
                'base_position': self.robot.get_base_position(),
                'base_velocity': self.robot.get_base_velocity(),
                'battery_level': self.robot.get_battery_level(),
                'cpu_usage': self.get_cpu_usage(),
                'network_latency_ms': self.measure_latency()
            }

            self.buffer.append(state)

            # Send to cloud when buffer full
            if len(self.buffer) >= self.buffer_size:
                await self.send_batch()

            await asyncio.sleep(0.01)  # 100Hz sampling

    async def send_batch(self):
        """Send batch of telemetry to cloud"""
        try:
            payload = {
                'robot_id': self.robot_id,
                'data': self.buffer
            }

            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f'{self.cloud_endpoint}/telemetry',
                    json=payload
                ) as resp:
                    if resp.status == 200:
                        self.buffer = []  # Clear after successful send
                    else:
                        print(f"Failed to send telemetry: {resp.status}")
        except Exception as e:
            print(f"Telemetry send error: {e}")
```

## Fleet Management

### Coordinating Multiple Robots

```python
class FleetCoordinator:
    def __init__(self, num_robots: int):
        self.robots = {f'robot_{i}': RobotState() for i in range(num_robots)}
        self.task_queue = []

    async def assign_tasks(self, tasks: List[Task]):
        """Assign tasks to robots optimally"""
        # Sort robots by current load
        available_robots = sorted(
            self.robots.values(),
            key=lambda r: r.current_load
        )

        for task in tasks:
            # Find best robot for this task
            best_robot = self.find_best_robot(available_robots, task)

            if best_robot:
                await best_robot.accept_task(task)

    def find_best_robot(self, robots: List[RobotState], task: Task) -> RobotState:
        """Select robot minimizing completion time"""
        best = None
        min_time = float('inf')

        for robot in robots:
            # Estimate task completion time
            travel_time = self.estimate_travel_time(robot.location, task.location)
            execution_time = task.estimated_duration
            total_time = travel_time + execution_time

            if total_time < min_time:
                min_time = total_time
                best = robot

        return best if min_time < float('inf') else None

    async def monitor_fleet(self):
        """Monitor all robots continuously"""
        while True:
            for robot_id, robot in self.robots.items():
                # Check health
                if robot.battery_level < 0.1:
                    await self.send_to_charger(robot_id)

                # Check location
                if robot.is_outside_geofence:
                    await self.emergency_stop(robot_id)

                # Aggregate metrics
                self.fleet_metrics.update(robot.metrics)

            await asyncio.sleep(1.0)  # Check every second
```

## Machine Learning Pipelines

### Training Data Collection

```python
class DataCollectionPipeline:
    def __init__(self):
        self.s3_client = boto3.client('s3')
        self.mongo_client = AsyncIOMotorClient('mongodb+srv://...')

    async def collect_training_data(self, robot_id: str):
        """Collect diverse training data from fleet"""
        # Subscribe to robot telemetry
        async with self.subscribe_telemetry(robot_id) as telemetry_stream:
            async for data in telemetry_stream:
                # Capture image from robot camera
                image = data['camera_frame']
                joints = data['joint_angles']
                ee_pose = data['end_effector_pose']

                # Store training sample
                sample = {
                    'image': self.encode_image(image),
                    'joints': joints,
                    'ee_pose': ee_pose,
                    'timestamp': data['timestamp'],
                    'robot_id': robot_id
                }

                # Save to MongoDB for indexing
                await self.mongo_client.training_data.insert_one(sample)

                # Save raw image to S3
                key = f"training_data/{robot_id}/{data['timestamp']}.jpg"
                self.s3_client.put_object(
                    Bucket='robotics-training-data',
                    Key=key,
                    Body=image
                )

    async def prepare_training_batch(self, model_type: str, num_samples: int):
        """Prepare data for model training"""
        # Query MongoDB for samples
        query = {'model_type': model_type}
        samples = await self.mongo_client.training_data.find(query).limit(num_samples).to_list(None)

        # Create PyTorch dataset
        dataset = RoboticsDataset(samples)
        dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

        return dataloader
```

### Distributed Training

```python
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel

def train_distributed(model, train_loader, num_epochs):
    """Train model across multiple GPUs/nodes"""
    # Initialize distributed training
    dist.init_process_group("nccl")

    # Wrap model for distributed training
    model = model.to(dist.get_rank())
    ddp_model = DistributedDataParallel(model)

    optimizer = torch.optim.Adam(ddp_model.parameters())

    for epoch in range(num_epochs):
        for batch_idx, (images, targets) in enumerate(train_loader):
            # Forward pass
            logits = ddp_model(images.cuda())
            loss = criterion(logits, targets.cuda())

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            if batch_idx % 100 == 0:
                print(f"Epoch {epoch}, Batch {batch_idx}, Loss {loss.item()}")

        # Synchronize across all processes
        dist.barrier()

        # Save checkpoint on rank 0 only
        if dist.get_rank() == 0:
            torch.save(ddp_model.state_dict(), f"model_epoch_{epoch}.pt")
```

## Scalability Patterns

### Horizontal Scaling

```python
# Pattern 1: Load balancing across cloud nodes
class LoadBalancedAPI:
    def __init__(self, num_replicas: int):
        self.replicas = [f"api-server-{i}" for i in range(num_replicas)]
        self.current = 0

    async def handle_request(self, request):
        # Round-robin load balancing
        server = self.replicas[self.current % len(self.replicas)]
        self.current += 1

        async with aiohttp.ClientSession() as session:
            async with session.post(f"http://{server}/process", json=request) as resp:
                return await resp.json()

# Pattern 2: Sharding robot fleet by geography
class GeographicSharding:
    def __init__(self):
        self.regions = {
            'us-west': Shard(region='us-west-2'),
            'us-east': Shard(region='us-east-1'),
            'eu': Shard(region='eu-central-1'),
        }

    def get_shard(self, robot_location):
        """Route robot to nearest shard"""
        region = self.get_region(robot_location)
        return self.regions[region]

    def get_region(self, location):
        # Determine region from lat/long
        lat, lng = location
        if lat > 40: return 'us-east' if lng < -75 else 'eu'
        return 'us-west'
```

## Security Considerations

### Authentication & Authorization

```python
from fastapi_jwt_auth import AuthJWT
from pydantic import BaseModel

class Settings(BaseModel):
    jwt_secret: str = "robotics-secret-key"

class Robot(BaseModel):
    robot_id: str
    api_key: str

async def authenticate_robot(api_key: str):
    """Verify robot's credentials"""
    robot = await db.robots.find_one({'api_key': api_key})
    if not robot:
        raise HTTPException(status_code=401, detail="Invalid API key")
    return robot

@app.post("/command")
async def execute_command(
    command: Command,
    robot: Robot = Depends(authenticate_robot),
    authorize: AuthJWT = Depends()
):
    """Execute command with authentication"""
    # Verify robot has permission for this command
    permissions = await get_robot_permissions(robot.robot_id)

    if command.type not in permissions:
        raise HTTPException(status_code=403, detail="Permission denied")

    return await execute_safe_command(robot, command)
```

## Exercises

**Exercise 15.1**: Containerize a robot control system with Docker
**Exercise 15.2**: Deploy fleet coordinator using Kubernetes
**Exercise 15.3**: Implement telemetry streaming pipeline to cloud

## References

1. Kubernetes Documentation: https://kubernetes.io/docs/
2. Docker for Robotics: https://docs.docker.com/
3. ROS 2 on Cloud (AWS RoboMaker): https://aws.amazon.com/robomaker/
4. Distributed ML (PyTorch): https://pytorch.org/docs/stable/distributed.html
5. Fleet Management Architecture: https://arxiv.org/abs/2209.01657
