# Chapter 6: Capstone Project - Building an Autonomous Service Robot

## Learning Objectives

After completing this chapter, you will be able to:

- **Integrate** all concepts from previous chapters into a complete physical AI system
- **Design** and implement a multi-component robot software architecture
- **Deploy** a functional autonomous service robot with perception, planning, and control
- **Evaluate** system performance and iterate on design based on real-world testing

## Overview

This capstone project synthesizes knowledge from Chapters 1-5 to build an **autonomous service robot** that navigates environments, interacts with objects, and follows natural language commands. You'll create a digital twin, implement ROS2-based control, integrate vision-language models, and deploy to real or simulated hardware.

## Project Overview: CoffeeBot

**Mission**: Build a mobile manipulator that autonomously prepares and delivers coffee in an office environment.

**Key Capabilities**:
1. **Navigation**: Move through office, avoid obstacles, localize on map
2. **Manipulation**: Grasp coffee cups, operate coffee machine, pour liquid
3. **Language interaction**: Accept spoken orders ("bring me a latte"), provide status updates
4. **Safety**: Detect people, slow down in crowded areas, handle errors gracefully

**Integration of Course Concepts**:
- **Chapter 1**: Embodied AI principles, perception-action loop
- **Chapter 2**: Whole-body motion planning, humanoid manipulation
- **Chapter 3**: ROS2 nodes for modularity, launch files for orchestration
- **Chapter 4**: Digital twin for testing, sim-to-real transfer
- **Chapter 5**: Vision-language model for task understanding, object grounding

## System Architecture

### High-Level Design

```
┌─────────────────────────────────────────────────────────────┐
│                        User Interface                        │
│              (Voice commands, status display)                │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────────┐
│                     Task Planner (VLA)                       │
│        (Parse "make latte" → subtasks: grind, brew,         │
│                     steam milk, pour)                         │
└───────┬─────────────────┬─────────────────┬────────────────┘
        │                 │                 │
┌───────▼───────┐ ┌──────▼───────┐ ┌────────▼──────────┐
│  Navigation   │ │ Manipulation │ │   Monitoring      │
│   (Nav2)      │ │   (MoveIt2)  │ │  (Diagnostics)    │
└───────┬───────┘ └──────┬───────┘ └────────┬──────────┘
        │                 │                   │
┌───────▼─────────────────▼───────────────────▼──────────┐
│              Perception Layer (Sensors)                  │
│   LIDAR, Cameras, Depth sensors, IMU, Joint encoders    │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│           Hardware Abstraction Layer (ROS2)              │
│         (Mobile base, robot arm, gripper control)        │
└──────────────────────────────────────────────────────────┘
```

### ROS2 Node Graph

**Perception Nodes**:
- `/camera_driver`: Publishes RGB images (`/camera/image_raw`)
- `/lidar_driver`: Publishes laser scans (`/scan`)
- `/object_detector`: Detects cups, coffee machine (`/detected_objects`)
- `/people_detector`: Detects humans for safety (`/people_poses`)

**Localization & Mapping**:
- `/slam_toolbox`: SLAM for map building (`/map`)
- `/amcl`: Monte Carlo localization (`/pose_estimate`)

**Planning Nodes**:
- `/task_planner`: VLA model interprets commands (`/task_plan`)
- `/nav2`: Navigation stack (`/navigate_to_pose` action)
- `/moveit_planning`: Arm trajectory planning (`/plan_arm_motion` service)

**Control Nodes**:
- `/mobile_base_controller`: Executes velocity commands (`/cmd_vel`)
- `/arm_controller`: Executes joint trajectories (`/joint_trajectory`)
- `/gripper_controller`: Opens/closes gripper (`/gripper_command`)

**Monitoring**:
- `/diagnostics`: System health (`/diagnostics` topic)
- `/battery_monitor`: Battery level alerts (`/battery_state`)

## Phase 1: Digital Twin Development

### 1.1 Build the Robot Model (URDF)

**Robot Specification**: Mobile manipulator with differential drive base and 6-DOF arm

**URDF Components**:
```xml
<robot name="coffeebot">
  <!-- Mobile base -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.25" length="0.1"/></geometry>
    </visual>
    <inertial>
      <mass value="30.0"/>
      <inertia ixx="0.5" iyy="0.5" izz="1.0"/>
    </inertial>
  </link>

  <!-- Wheels (left, right, caster) -->
  <link name="left_wheel">...</link>
  <link name="right_wheel">...</link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Robot arm (6 joints: shoulder pan/tilt, elbow, wrist roll/pitch/yaw) -->
  <link name="shoulder_link">...</link>
  <joint name="shoulder_pan" type="revolute">
    <limit lower="-3.14" upper="3.14" effort="50" velocity="1.0"/>
  </joint>

  <!-- Gripper (parallel jaw) -->
  <link name="gripper_left_finger">...</link>
  <link name="gripper_right_finger">...</link>

  <!-- Sensors -->
  <link name="camera_link">...</link>
  <link name="lidar_link">...</link>
</robot>
```

**Action Items**:
1. Model robot in SolidWorks or Fusion 360
2. Export to URDF using `sw_urdf_exporter` plugin
3. Validate kinematics with `ros2 run joint_state_publisher_gui joint_state_publisher_gui`

### 1.2 Create Simulation Environment

**Tool**: NVIDIA Isaac Sim or Gazebo

**Environment Features**:
- Office layout (desks, chairs, walls)
- Coffee station (coffee machine, cups, counter)
- Dynamic obstacles (people walking)
- Lighting variations (sunny, cloudy, nighttime)

**Procedural Randomization** (for robust policy training):
```python
# Randomize coffee machine position (±10cm)
machine_pos = [2.0 + random.uniform(-0.1, 0.1),
               1.0 + random.uniform(-0.1, 0.1),
               0.9]

# Randomize cup colors/textures
cup_texture = random.choice(["ceramic_white", "ceramic_blue", "matte_black"])

# Randomize lighting (intensity ±30%)
light_intensity = base_intensity * random.uniform(0.7, 1.3)
```

### 1.3 Sensor Simulation

**Camera** (RGB, 1920x1080, 30Hz):
```xml
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.0472</horizontal_fov>  <!-- 60 degrees -->
    <image><width>1920</width><height>1080</height></image>
    <clip><near>0.1</near><far>10.0</far></clip>
    <noise><mean>0.0</mean><stddev>0.01</stddev></noise>
  </camera>
</sensor>
```

**LIDAR** (360°, 0.25° resolution, 10Hz):
```xml
<sensor name="lidar" type="gpu_lidar">
  <update_rate>10</update_rate>
  <ray>
    <scan><horizontal><samples>1440</samples><resolution>1</resolution></horizontal></scan>
    <range><min>0.1</min><max>10.0</max></range>
    <noise><mean>0.0</mean><stddev>0.02</stddev></noise>
  </ray>
</sensor>
```

## Phase 2: Perception Pipeline

### 2.1 Object Detection

**Approach**: Use pretrained YOLO model fine-tuned on coffee-related objects

**Objects to Detect**:
- Coffee cups (empty, full)
- Coffee machine
- Milk carton
- Coffee beans bag

**Implementation**:
```python
import cv2
from ultralytics import YOLO

class CoffeeObjectDetector:
    def __init__(self):
        self.model = YOLO('yolov8n.pt')
        # Fine-tune on custom dataset of 500 labeled images

    def detect(self, image):
        results = self.model(image, conf=0.5)
        detections = []
        for result in results:
            for box in result.boxes:
                detections.append({
                    'class': result.names[int(box.cls)],
                    'confidence': float(box.conf),
                    'bbox': box.xyxy.tolist(),
                    'center_3d': self.estimate_3d_position(box, depth_image)
                })
        return detections
```

**ROS2 Integration**:
```python
class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_pub = self.create_publisher(
            DetectionArray, '/detected_objects', 10)
        self.detector = CoffeeObjectDetector()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        detections = self.detector.detect(cv_image)
        self.detection_pub.publish(self.detections_to_msg(detections))
```

### 2.2 People Detection (Safety)

**Approach**: Use MoveNet or MediaPipe for human pose estimation

**Safety Logic**:
- Slow down to 0.2 m/s when person within 2m
- Stop if person within 0.5m
- Resume normal speed when clear

```python
def check_safety(people_poses, robot_pose):
    for person in people_poses:
        distance = compute_distance(person.position, robot_pose.position)
        if distance < 0.5:
            return VelocityCommand(linear=0.0, angular=0.0)  # STOP
        elif distance < 2.0:
            return VelocityCommand(linear=0.2, angular=0.3)  # SLOW
    return VelocityCommand(linear=0.5, angular=0.5)  # NORMAL
```

## Phase 3: Navigation Stack

### 3.1 Mapping with SLAM Toolbox

**Launch SLAM**:
```bash
ros2 launch slam_toolbox online_async_launch.py
```

**Drive robot** (teleoperation or autonomous exploration) to build map

**Save map**:
```bash
ros2 run nav2_map_server map_saver_cli -f office_map
```

### 3.2 Localization with AMCL

**Configuration** (`amcl_params.yaml`):
```yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 2000
    laser_model_type: "likelihood_field"
    odom_model_type: "diff"  # Differential drive
    recovery_alpha_slow: 0.001
    recovery_alpha_fast: 0.1
```

**Launch**:
```bash
ros2 launch nav2_bringup localization_launch.py map:=office_map.yaml
```

### 3.3 Path Planning with Nav2

**Send navigation goal** (coffee station):
```python
from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()
navigator.waitUntilNav2Active()

coffee_station_pose = PoseStamped()
coffee_station_pose.header.frame_id = 'map'
coffee_station_pose.pose.position.x = 2.5
coffee_station_pose.pose.position.y = 1.0
coffee_station_pose.pose.orientation.w = 1.0

navigator.goToPose(coffee_station_pose)
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    print(f"Distance remaining: {feedback.distance_remaining:.2f}m")

result = navigator.getResult()
if result == TaskResult.SUCCEEDED:
    print("Reached coffee station!")
```

## Phase 4: Manipulation

### 4.1 Arm Control with MoveIt2

**Setup MoveIt2** for arm:
```bash
ros2 launch moveit_setup_assistant setup_assistant.py
# Configure arm kinematics, collision checking, controllers
```

**Plan and execute grasp**:
```python
import moveit_commander

arm_group = moveit_commander.MoveGroupCommander("arm")
arm_group.set_planner_id("RRTConnect")

# Approach pose (above cup)
approach_pose = Pose()
approach_pose.position = cup_position + Vector3(0, 0, 0.1)  # 10cm above
approach_pose.orientation = Quaternion(0, 0.707, 0, 0.707)  # Gripper down

arm_group.set_pose_target(approach_pose)
plan = arm_group.plan()
arm_group.execute(plan[1])  # Move to approach pose

# Open gripper
gripper_pub.publish(GripperCommand(position=0.08))  # Open

# Lower to grasp
grasp_pose = cup_position
arm_group.set_pose_target(grasp_pose)
arm_group.go()

# Close gripper
gripper_pub.publish(GripperCommand(position=0.0, max_effort=20.0))  # Grasp

# Lift cup
lift_pose = grasp_pose + Vector3(0, 0, 0.15)
arm_group.set_pose_target(lift_pose)
arm_group.go()
```

### 4.2 Grasp Pose Estimation

**Approach**: 6-DOF grasp pose detection using depth images

**Algorithm**:
1. Segment cup from point cloud
2. Fit cylinder to cup shape
3. Compute grasp axis (perpendicular to cylinder axis)
4. Generate top-down grasp (avoid spilling liquid)

```python
def estimate_grasp_pose(cup_point_cloud):
    # Fit cylinder
    cylinder = fit_cylinder_ransac(cup_point_cloud)

    # Grasp from top (for filled cups)
    grasp_pose = Pose()
    grasp_pose.position = cylinder.center + Vector3(0, 0, cylinder.height/2)
    grasp_pose.orientation = Quaternion(0, 0.707, 0, 0.707)  # Gripper vertical

    return grasp_pose
```

## Phase 5: Task Planning with VLA

### 5.1 Language-to-Task Decomposition

**User Command**: "Make me a cappuccino"

**VLA Model Output** (task decomposition):
```json
{
  "task": "make_cappuccino",
  "subtasks": [
    {"action": "navigate_to", "target": "coffee_station"},
    {"action": "grasp", "object": "coffee_cup"},
    {"action": "operate", "target": "coffee_machine", "button": "espresso"},
    {"action": "wait", "duration": 30},
    {"action": "grasp", "object": "milk_carton"},
    {"action": "pour", "target": "coffee_cup", "amount": "50ml"},
    {"action": "operate", "target": "steam_wand"},
    {"action": "navigate_to", "target": "user_desk"},
    {"action": "place", "object": "coffee_cup", "location": "desk"}
  ]
}
```

### 5.2 VLA Integration

**Pretrained Model**: RT-2 or fine-tuned on coffee task dataset

**Inference**:
```python
class VLATaskPlanner:
    def __init__(self):
        self.model = RT2Model.from_pretrained("google/rt-2-base")
        self.task_executor = TaskExecutor()

    def execute_command(self, voice_command, camera_image):
        # Encode instruction and current scene
        task_plan = self.model.predict_task_sequence(
            instruction=voice_command,
            image=camera_image
        )

        # Execute each subtask
        for subtask in task_plan['subtasks']:
            success = self.task_executor.execute(subtask)
            if not success:
                self.handle_failure(subtask)
                break

        return success
```

### 5.3 Error Recovery

**Common Failures**:
- Cup not detected → Re-scan environment, adjust camera angle
- Grasp failed → Retry with different grasp pose, use two-handed grasp
- Navigation blocked → Replan path, request human assistance

**Logging**:
```python
if not success:
    self.logger.error(f"Subtask {subtask['action']} failed")
    diagnostics_msg = DiagnosticStatus()
    diagnostics_msg.level = DiagnosticStatus.ERROR
    diagnostics_msg.name = "task_planner"
    diagnostics_msg.message = f"Failed to {subtask['action']}"
    self.diagnostics_pub.publish(diagnostics_msg)
```

## Phase 6: Testing and Evaluation

### 6.1 Unit Testing

**Test each component independently**:

```python
import unittest

class TestObjectDetection(unittest.TestCase):
    def setUp(self):
        self.detector = CoffeeObjectDetector()
        self.test_image = cv2.imread('test_data/cup_scene.jpg')

    def test_cup_detection(self):
        detections = self.detector.detect(self.test_image)
        cups = [d for d in detections if d['class'] == 'coffee_cup']
        self.assertGreater(len(cups), 0, "No cups detected")
        self.assertGreater(cups[0]['confidence'], 0.8, "Low confidence")

    def test_false_positives(self):
        empty_image = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = self.detector.detect(empty_image)
        self.assertEqual(len(detections), 0, "False positive detection")
```

### 6.2 Integration Testing

**End-to-End Scenarios**:
1. **Coffee delivery**: User → "Make latte" → Robot delivers → Success/Failure
2. **Obstacle avoidance**: Place obstacles in path → Robot reroutes → Success
3. **Dynamic environment**: People walking → Robot slows down → Safety maintained

**Metrics**:
- **Task success rate**: % of successful coffee deliveries
- **Navigation time**: Time from start to goal (target: <120 seconds)
- **Grasp success rate**: % of successful grasps (target: >90%)
- **Safety incidents**: Collisions with people/objects (target: 0)

### 6.3 Sim-to-Real Validation

**Procedure**:
1. Train policy in Isaac Sim (10,000 episodes)
2. Validate in sim (test set: 1,000 episodes, target: >85% success)
3. Deploy to real robot (start with 10 test runs)
4. Compare sim vs real performance
5. Fine-tune with real-world data (100 demonstrations)

**Reality Gap Analysis**:
- Measure position error: || sim_cup_pos - real_cup_pos ||
- Measure grasp success rate drop (sim: 95% → real: 80% → needs tuning)

## Phase 7: Deployment and Monitoring

### 7.1 Launch System

**Master Launch File** (`coffeebot.launch.py`):
```python
def generate_launch_description():
    return LaunchDescription([
        # Hardware drivers
        Node(package='lidar_driver', executable='lidar_node'),
        Node(package='camera_driver', executable='camera_node'),

        # Perception
        Node(package='object_detection', executable='yolo_detector'),
        Node(package='people_detection', executable='pose_estimator'),

        # Localization & mapping
        IncludeLaunchDescription('nav2_bringup', 'localization_launch.py'),

        # Planning
        Node(package='task_planner', executable='vla_planner'),

        # Control
        Node(package='mobile_base', executable='diff_drive_controller'),
        Node(package='arm_control', executable='trajectory_controller'),

        # Monitoring
        Node(package='diagnostics', executable='diagnostic_aggregator'),

        # Visualization
        Node(package='rviz2', executable='rviz2', arguments=['-d', 'coffeebot.rviz'])
    ])
```

**Start**:
```bash
ros2 launch coffeebot coffeebot.launch.py
```

### 7.2 Real-Time Monitoring

**RViz2 Dashboard**:
- Robot model (joint states, current pose)
- LIDAR scan (obstacles)
- Camera feed (with bounding boxes)
- Planned path (Nav2 trajectory)
- Diagnostics (system health)

**Terminal Monitoring**:
```bash
# Monitor task planner output
ros2 topic echo /task_plan

# Check battery level
ros2 topic echo /battery_state

# View diagnostics
ros2 topic echo /diagnostics
```

### 7.3 Performance Logging

**Log critical data for post-analysis**:
```python
import rosbag2_py

writer = rosbag2_py.SequentialWriter()
writer.open(storage_options, converter_options)

# Record all topics for 1-hour operation
topics_to_record = ['/camera/image_raw', '/scan', '/detected_objects',
                    '/joint_states', '/odom', '/task_plan']

# Playback for debugging
ros2 bag play coffeebot_run_2024-12-07
```

## Deliverables and Evaluation

### Required Deliverables

1. **Code Repository** (GitHub):
   - ROS2 packages (perception, planning, control)
   - Launch files
   - URDF/SDF robot description
   - Documentation (README, setup instructions)

2. **Digital Twin**:
   - Isaac Sim or Gazebo world file
   - Trained policies (model weights)

3. **Demo Video** (5 minutes):
   - Show robot accepting voice command
   - Navigate to coffee station
   - Manipulate coffee machine and cup
   - Deliver to user
   - Highlight safety features (people avoidance)

4. **Technical Report** (10-15 pages):
   - Architecture diagram
   - Design decisions (sensors, algorithms, ROS2 graph)
   - Sim-to-real transfer approach
   - Performance metrics (success rate, latency)
   - Failure modes and recovery strategies
   - Lessons learned

### Evaluation Rubric

| Category | Weight | Criteria |
|----------|--------|----------|
| **Functionality** | 40% | Task success rate (coffee delivery), navigation accuracy, manipulation success |
| **Integration** | 20% | ROS2 architecture, modularity, sensor fusion, VLA integration |
| **Safety** | 15% | People detection, collision avoidance, graceful degradation |
| **Sim-to-Real** | 10% | Digital twin fidelity, domain randomization, transfer success |
| **Documentation** | 10% | Code quality, README, technical report clarity |
| **Innovation** | 5% | Novel solutions, creative error recovery, advanced features |

## Extension Ideas

**Advanced Features** (optional enhancements):

1. **Multi-Robot Coordination**: Deploy fleet of CoffeeBots, coordinate task allocation
2. **Adaptive Learning**: Continual learning from human feedback ("too much milk")
3. **Personalization**: Remember user preferences (saved in database)
4. **Mobile Base Upgrade**: Replace differential drive with omnidirectional wheels for tighter spaces
5. **Advanced Manipulation**: Latte art drawing using visual servoing

## Key Takeaways

- Capstone project integrates embodied AI, ROS2, digital twins, and VLA models into a cohesive system
- Modularity (ROS2 nodes) enables independent development and testing of subsystems
- Digital twin accelerates development and enables safe policy training before deployment
- Sim-to-real transfer requires domain randomization, sensor calibration, and real-world fine-tuning
- Comprehensive testing (unit, integration, end-to-end) ensures robustness and safety
- Real-world deployment demands monitoring, logging, and error recovery mechanisms

## References

1. **Nav2 Documentation**. https://navigation.ros.org/

2. **MoveIt 2 Documentation**. https://moveit.picknik.ai/

3. **SLAM Toolbox**. https://github.com/SteveMacenski/slam_toolbox

4. **NVIDIA Isaac Sim**. https://docs.omniverse.nvidia.com/isaacsim/

5. **Gazebo**. https://gazebosim.org/

6. Wise, M., et al. (2016). **Fetch and Freight: Standard Platforms for Service Robot Applications**. Workshop on Autonomous Mobile Service Robots.

7. **RT-2 (Robotics Transformer 2)**. https://robotics-transformer2.github.io/

8. **ROS2 Design Patterns**. http://design.ros2.org/

---

**Previous Chapter**: [← Chapter 5: Vision-Language-Action Systems](../05-vision-language-action)
**Congratulations!** You've completed the Physical AI textbook.
