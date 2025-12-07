# Chapter 3: ROS2 Fundamentals

## Learning Objectives

After completing this chapter, you will be able to:

- **Understand** the ROS2 architecture and its advantages over ROS1
- **Create** ROS2 nodes, topics, services, and actions for robot control
- **Implement** publish-subscribe communication patterns for sensor data
- **Apply** ROS2 tools for debugging, visualization, and system monitoring

## Overview

Robot Operating System 2 (ROS2) is the industry-standard middleware for building modular, distributed robot software. This chapter introduces ROS2's core concepts, communication patterns, and ecosystem tools, providing the foundation for developing scalable physical AI systems.

## What is ROS2?

ROS2 is **not** an operating system despite its name. It is a **middleware framework** that provides:

- **Communication infrastructure**: Publish-subscribe messaging, services, actions
- **Hardware abstraction**: Uniform interfaces for sensors, actuators, and algorithms
- **Development tools**: Build systems, debugging, visualization, simulation
- **Community packages**: Thousands of open-source libraries for perception, planning, control

**Key Insight**: ROS2 enables **modularity** and **interoperability**. You can swap perception algorithms, control strategies, or hardware platforms without rewriting the entire system.

## ROS1 vs ROS2: Why the Upgrade?

ROS1 (introduced 2007) served robotics research well but had limitations for production deployments. ROS2 (released 2017) addresses these issues:

| Feature | ROS1 | ROS2 |
|---------|------|------|
| **Communication** | Custom TCPROS/UDPROS | DDS (Data Distribution Service) standard |
| **Real-time support** | Limited, best-effort | Native real-time capabilities |
| **Security** | None | DDS encryption, authentication, access control |
| **Multi-robot** | Complex, requires custom setup | Built-in domain isolation |
| **Platforms** | Linux only | Linux, Windows, macOS, RTOS |
| **Lifecycle management** | Manual | Managed node lifecycle (configuring, active, inactive) |
| **Python support** | Python 2 | Python 3 |
| **QoS policies** | None | Reliability, durability, lifespan, history depth |

**Migration Note**: ROS1 reaches end-of-life in 2025 (ROS Noetic). New projects should use ROS2. ROS1-ROS2 bridge exists for gradual migration.

## Core Concepts

### Nodes

A **node** is a process that performs a specific computation. Each node is a modular, single-purpose component.

**Examples**:
- `camera_driver`: Publishes images from a camera
- `object_detector`: Subscribes to images, publishes bounding boxes
- `motion_planner`: Subscribes to goals and map, publishes path
- `motor_controller`: Subscribes to velocity commands, controls motors

**Design Philosophy**: Small, focused nodes are easier to test, debug, and reuse than monolithic programs.

### Topics (Publish-Subscribe)

**Topics** enable asynchronous, many-to-many communication. Publishers send messages without knowing who (if anyone) is listening. Subscribers receive messages without knowing who sent them.

**Characteristics**:
- **Decoupled**: Publishers and subscribers don't need to know about each other
- **Unidirectional**: Data flows one way (publisher → topic → subscriber)
- **Best for**: Continuous data streams (sensor readings, robot state)

**Example**: Camera publishing images
```
[Camera Driver] --publishes→ /camera/image_raw (topic) --subscribes← [Object Detector]
                                                       --subscribes← [Video Recorder]
                                                       --subscribes← [UI Display]
```

Multiple subscribers can receive the same data simultaneously without coordination.

### Services (Request-Reply)

**Services** provide synchronous, one-to-one communication. A client sends a request and waits for a response.

**Characteristics**:
- **Synchronous**: Client blocks until response received (or timeout)
- **Bidirectional**: Request and response messages
- **Best for**: Infrequent operations with immediate results (queries, triggers)

**Example**: Map server providing map data
```
Client: "Give me the map of region (x, y, width, height)"
Server: "Here is the map: [occupancy grid data]"
```

### Actions (Goal-Oriented Tasks)

**Actions** combine request-reply (like services) with **feedback** and **cancellation**. Suitable for long-running tasks.

**Characteristics**:
- **Asynchronous**: Client sends goal, receives feedback updates, waits for final result
- **Cancellable**: Client can cancel goal mid-execution
- **Feedback**: Server sends progress updates during execution
- **Best for**: Long-duration tasks (navigation, grasping, trajectory execution)

**Example**: Navigate to goal
```
Client → Goal: "Drive to coordinates (5.0, 3.0)"
Server → Feedback: "Distance remaining: 8.2m" (every second)
Server → Feedback: "Distance remaining: 3.1m"
Server → Result: "Goal reached. Final position: (5.01, 2.98)"
```

If obstacle blocks path, client can cancel and send new goal.

### Comparison Table

| Mechanism | Direction | Synchronous? | Use Case |
|-----------|-----------|--------------|----------|
| **Topic** | One-way, many-to-many | No | Sensor streams, robot state |
| **Service** | Two-way, one-to-one | Yes | Configuration, queries |
| **Action** | Two-way, one-to-one, with feedback | No | Long tasks with progress |

## ROS2 Communication Middleware: DDS

ROS2 uses **Data Distribution Service (DDS)** as its communication layer. DDS is an OMG standard for real-time, distributed systems.

**Advantages**:
- **Quality of Service (QoS)**: Configure reliability, latency, bandwidth per topic
- **Discovery**: Automatic node discovery without central master (unlike ROS1's roscore)
- **Scalability**: Designed for large-scale systems (hundreds of nodes)
- **Security**: Built-in encryption and authentication

**DDS Vendors** (ROS2 supports multiple):
- **Fast-DDS** (eProsima): Default, high performance
- **Cyclone DDS** (Eclipse): Lightweight, embeddable
- **RTI Connext DDS**: Commercial, advanced features

Vendors are interoperable - nodes using different DDS implementations can communicate.

## Quality of Service (QoS) Policies

QoS policies allow fine-grained control over message delivery guarantees.

### Key QoS Parameters

**1. Reliability**:
- **Reliable**: Guarantee message delivery (retransmit if lost). Use for critical data (commands, goals).
- **Best effort**: No delivery guarantee. Use for high-frequency, loss-tolerant data (sensor streams).

**2. Durability**:
- **Transient local**: Store last N messages for new subscribers (late joiners get recent data).
- **Volatile**: No message storage (new subscribers only get future messages).

**3. History**:
- **Keep last N**: Store only last N messages in queue.
- **Keep all**: Store all messages (until queue full).

**4. Deadline**:
- Maximum time between messages. Alert if publisher doesn't meet rate.

**5. Lifespan**:
- Messages expire after duration (e.g., 5-second-old sensor data is stale).

**Example QoS Profiles**:
```python
# High-frequency sensor (LIDAR, cameras)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Tolerate loss
    durability=DurabilityPolicy.VOLATILE,       # No history for late joiners
    history=HistoryPolicy.KEEP_LAST,
    depth=10  # Keep last 10 messages
)

# Robot commands (velocity, joint angles)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Must receive
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Map updates (low frequency, critical)
map_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Late joiners get latest map
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

**Pitfall**: Publishers and subscribers with incompatible QoS won't connect. Use `ros2 topic info -v` to check QoS compatibility.

## ROS2 Workspace and Build System

### Workspace Structure

A ROS2 workspace organizes packages:

```
ros2_ws/
├── src/              # Source code
│   ├── package_1/
│   │   ├── package.xml          # Package metadata
│   │   ├── CMakeLists.txt       # Build configuration (C++)
│   │   ├── setup.py             # Build configuration (Python)
│   │   ├── launch/              # Launch files
│   │   ├── config/              # YAML configurations
│   │   └── src/ or scripts/     # Source files
│   ├── package_2/
│   └── ...
├── build/            # Build artifacts (auto-generated)
├── install/          # Installed packages (auto-generated)
└── log/              # Build logs (auto-generated)
```

### Build Tool: colcon

`colcon` is the ROS2 build tool (replaces ROS1's `catkin`).

**Key Commands**:
```bash
# Build all packages in workspace
colcon build

# Build specific package
colcon build --packages-select my_robot_control

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build and run tests
colcon test
colcon test-result --verbose  # Show test results
```

### Package Dependencies

Packages declare dependencies in `package.xml`:

```xml
<package format="3">
  <name>my_robot_control</name>
  <version>1.0.0</version>
  <description>Control algorithms for my robot</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclcpp</depend>  <!-- ROS2 C++ library -->
  <depend>std_msgs</depend>  <!-- Standard message types -->
  <depend>geometry_msgs</depend>  <!-- Geometry messages (pose, twist, etc.) -->

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
</package>
```

`colcon` resolves dependencies and builds packages in correct order.

## Launch Files

**Launch files** start multiple nodes with configuration in a single command.

**Benefits**:
- Start complex systems (10+ nodes) with one command
- Set parameters, remap topics, configure namespaces
- Conditional logic (launch nodes only if condition met)

**Example Launch File** (Python):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera driver node
        Node(
            package='camera_driver',
            executable='camera_node',
            name='front_camera',
            parameters=[{'frame_rate': 30, 'resolution': '1920x1080'}],
            remappings=[('/camera/image_raw', '/front_camera/image')]
        ),

        # Object detection node
        Node(
            package='object_detection',
            executable='yolo_detector',
            name='detector',
            parameters=[{'model': 'yolov8n.pt', 'confidence': 0.5}]
        ),

        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', '/path/to/config.rviz']
        )
    ])
```

**Launch**:
```bash
ros2 launch my_package system.launch.py
```

This starts all three nodes with specified configurations.

## ROS2 Command-Line Tools

ROS2 provides CLI tools for introspection and debugging.

### Node Introspection

```bash
# List running nodes
ros2 node list

# Get info about a node
ros2 node info /camera_driver

# Output:
#   Subscribers:
#     /camera/config: std_msgs/msg/String
#   Publishers:
#     /camera/image_raw: sensor_msgs/msg/Image
#     /camera/camera_info: sensor_msgs/msg/CameraInfo
#   Services:
#     /camera/set_parameters
```

### Topic Introspection

```bash
# List active topics
ros2 topic list

# Get topic info (publishers, subscribers, QoS)
ros2 topic info /camera/image_raw -v

# Echo messages (print to terminal)
ros2 topic echo /camera/image_raw

# Publish message manually (for testing)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"

# Measure topic frequency
ros2 topic hz /camera/image_raw
# Output: average rate: 30.012 Hz

# Plot topic data
ros2 topic echo /joint_states | grep position  # Simple text-based
rqt_plot /joint_states/position[0]  # GUI plot
```

### Service and Action Tools

```bash
# List services
ros2 service list

# Call service
ros2 service call /get_map nav_msgs/srv/GetMap

# List actions
ros2 action list

# Send action goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 2.0, y: 1.0}}}"
```

### Parameter Management

Nodes can have runtime-configurable parameters:

```bash
# List node parameters
ros2 param list /camera_driver

# Get parameter value
ros2 param get /camera_driver frame_rate

# Set parameter value
ros2 param set /camera_driver frame_rate 60

# Dump all parameters to file
ros2 param dump /camera_driver > camera_params.yaml

# Load parameters from file
ros2 param load /camera_driver camera_params.yaml
```

## Visualization with RViz2

**RViz2** is the ROS2 visualization tool for robot state, sensor data, and planning results.

**Common Displays**:
- **RobotModel**: Visualize URDF robot description
- **TF**: Show coordinate frame transformations
- **LaserScan**: Display LIDAR data
- **Image**: Show camera streams
- **PointCloud2**: Render 3D point clouds
- **Map**: Display occupancy grids
- **Path**: Show planned trajectories

**Example Use Case**: Debugging a navigation stack
1. Load robot URDF to see robot model
2. Add LaserScan display to verify sensor data
3. Add Map display to see costmap
4. Add Path display to visualize planned route
5. Add markers to show detected obstacles

If robot veers off path, RViz reveals whether it's a perception issue (LIDAR not seeing obstacle), planning issue (path collides with obstacle), or control issue (robot not following path).

## TF2: Coordinate Frame Transformations

Physical AI systems have many coordinate frames: robot base, sensors, wheels, arms, world map, etc. **TF2** manages transformations between these frames.

**Key Concepts**:

**Transform Tree**:
```
world
├── map
│   └── odom
│       └── base_link
│           ├── camera_link
│           ├── lidar_link
│           └── left_wheel_link
```

Each arrow represents a transformation (position + orientation).

**Use Cases**:
- **Sensor fusion**: Convert LIDAR points (in `lidar_link` frame) to robot frame (`base_link`)
- **Manipulation**: Transform grasp pose (in `camera_link` frame) to arm frame (`arm_base_link`)
- **Localization**: Track robot position in map frame (`map`)

**Example**:
```python
# Python: Transform point from camera frame to base frame
from tf2_ros import Buffer, TransformListener

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer)

# Wait for transform to become available
transform = tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())

# Transform point
point_camera = PointStamped()
point_camera.header.frame_id = 'camera_link'
point_camera.point.x = 1.0  # 1 meter in front of camera

point_base = tf_buffer.transform(point_camera, 'base_link')
print(f"Point in base frame: {point_base.point}")
```

**TF CLI Tools**:
```bash
# View transform tree
ros2 run tf2_tools view_frames

# Print transform between frames
ros2 run tf2_ros tf2_echo base_link camera_link

# Monitor TF latency
ros2 topic hz /tf
```

## ROS2 in Physical AI: Example Architecture

**Scenario**: Autonomous mobile manipulator in warehouse

**Node Graph**:
```
Perception Layer:
  /camera_driver → /camera/image_raw → /object_detector → /detected_objects
  /lidar_driver → /scan → /obstacle_detector → /obstacles

Localization & Mapping:
  /scan + /odom → /slam_toolbox → /map
  /map + /scan → /amcl → /pose_estimate

Planning Layer:
  /detected_objects + /pose_estimate → /task_planner → /grasp_goal
  /grasp_goal + /map → /motion_planner → /arm_trajectory

Control Layer:
  /arm_trajectory → /joint_controller → /joint_commands (to hardware)
  /cmd_vel → /mobile_base_controller → /wheel_velocities (to hardware)
```

Each node is independently testable. Swapping perception algorithms (e.g., YOLOv8 → Mask R-CNN) requires only replacing `/object_detector` node.

## Exercises

### Exercise 1: Topic vs Service vs Action

For each scenario, choose the appropriate communication mechanism (topic, service, or action) and justify:

1. **Streaming camera images** at 30 Hz
2. **Requesting the robot's current battery level** (one-time query)
3. **Executing a pick-and-place task** that takes 10 seconds, with progress updates
4. **Publishing robot joint states** at 100 Hz for visualization
5. **Retrieving a pre-built map** from a map server
6. **Navigating to a goal pose** with ability to cancel mid-route

### Exercise 2: QoS Configuration

Design QoS profiles for these topics:

1. **Topic**: `/imu/data` (IMU sensor data at 200 Hz, critical for balance control)
   - Reliability: ?
   - Durability: ?
   - History depth: ?
   - Justification: ?

2. **Topic**: `/diagnostics` (system health messages, 1 Hz, new nodes should see latest)
   - Reliability: ?
   - Durability: ?
   - History depth: ?
   - Justification: ?

3. **Topic**: `/camera/compressed` (compressed images, 15 Hz, occasional drops okay, large messages)
   - Reliability: ?
   - Durability: ?
   - History depth: ?
   - Justification: ?

### Exercise 3: ROS2 Debugging Challenge

You launch a ROS2 system and notice that a subscriber is not receiving messages from a publisher, even though both nodes are running.

**Debugging Steps** (use ROS2 CLI tools):
1. Verify both nodes are running: `ros2 node list`
2. Check if topic exists: `ros2 topic list`
3. Check topic publishers and subscribers: `ros2 topic info /topic_name`
4. Inspect QoS compatibility: `ros2 topic info -v /topic_name`
5. Echo messages to confirm publishing: `ros2 topic echo /topic_name`

**Scenario**: Publisher uses `RELIABLE` reliability, subscriber uses `BEST_EFFORT`.

**Question**: Will messages be received? Why or why not? How do you fix it?

**Hint**: QoS policies must be compatible. `BEST_EFFORT` subscriber cannot connect to `RELIABLE` publisher in some DDS implementations.

## Key Takeaways

- ROS2 provides modular, distributed architecture for physical AI systems using nodes, topics, services, and actions
- DDS middleware enables real-time, secure, multi-platform communication with configurable QoS policies
- Launch files orchestrate complex multi-node systems with a single command
- CLI tools (`ros2 node`, `topic`, `service`, `param`) enable rapid debugging and system introspection
- TF2 manages coordinate transformations between robot components and world frames
- RViz2 visualizes robot state, sensor data, and planning outputs for debugging and monitoring

## References

1. **ROS2 Documentation**. Open Robotics. https://docs.ros.org/en/humble/

2. Martinez, A., & Fernández, E. (2021). **A Concise Introduction to Robot Programming with ROS2**. Independently published.

3. Pyo, Y., et al. (2017). **ROS Robot Programming** (ROS1, but concepts apply). ROBOTIS.

4. **DDS Specification v1.4**. Object Management Group. https://www.omg.org/spec/DDS/

5. **ROS2 Design**. Open Robotics. http://design.ros2.org/

6. Koubâa, A., et al. (2017). **Robot Operating System (ROS): The Complete Reference** (Volume 2). Springer.

7. **Nav2 Documentation** (Navigation2 stack for ROS2). https://navigation.ros.org/

8. **MoveIt 2 Documentation** (Motion planning framework). https://moveit.picknik.ai/

---

**Previous Chapter**: [← Chapter 2: Humanoid Robotics](../02-humanoid-robotics)
**Next Chapter**: [Chapter 4: Digital Twin →](../04-digital-twin)
