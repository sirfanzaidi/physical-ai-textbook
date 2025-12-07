# Chapter 2: Humanoid Robotics

## Learning Objectives

After completing this chapter, you will be able to:

- **Understand** the design principles and challenges of humanoid robot hardware
- **Explain** bipedal locomotion control and balance strategies
- **Implement** whole-body motion planning algorithms
- **Analyze** human-robot interaction considerations for humanoid systems

## Overview

Humanoid robots are designed to mimic human form and movement, enabling them to navigate human-built environments and interact naturally with people. This chapter explores the unique challenges of humanoid robotics, from mechanical design to control algorithms, and examines state-of-the-art systems transforming industries.

## Why Humanoid Form?

The humanoid form offers several advantages for physical AI systems operating in human environments:

### Environmental Compatibility

Human-built infrastructure is optimized for bipedal locomotion:
- **Stairs and ladders**: Designed for human leg length and reach
- **Doorways and corridors**: Sized for human dimensions
- **Tools and interfaces**: Handles, buttons, keyboards designed for human hands
- **Furniture**: Tables, chairs, shelves at human-ergonomic heights

A humanoid robot can navigate these spaces without requiring environmental modifications, unlike wheeled or tracked robots that may struggle with stairs or narrow passages.

### Natural Human Interaction

Humanoid form facilitates intuitive communication:
- **Gestures**: Pointing, waving, handshaking for non-verbal communication
- **Gaze direction**: Eye contact and head orientation signal attention and intent
- **Proxemics**: Understanding personal space and social distance
- **Anthropomorphic expectations**: Humans intuitively understand humanoid capabilities and limitations

### Tool Use and Manipulation

Human hands and arms enable versatile object interaction:
- **Grasping diversity**: From precision pinch grips to power grasps
- **Bimanual coordination**: Using two hands for complex tasks (assembly, carrying)
- **Tool adaptation**: Can use tools designed for humans without modification
- **Reach and dexterity**: Access high shelves, operate machinery, perform delicate tasks

## Humanoid Hardware Design

Designing humanoid hardware involves complex tradeoffs between human-like appearance, functionality, cost, and robustness.

### Degrees of Freedom (DOF)

Humanoid robots typically have 20-60+ degrees of freedom distributed across the body:

**Typical DOF Allocation**:
- **Legs** (each): 6-7 DOF
  - Hip: 3 DOF (roll, pitch, yaw)
  - Knee: 1 DOF (pitch)
  - Ankle: 2 DOF (pitch, roll)
- **Arms** (each): 7 DOF
  - Shoulder: 3 DOF
  - Elbow: 1-2 DOF
  - Wrist: 2-3 DOF
- **Torso**: 1-3 DOF (waist rotation, bending)
- **Neck/Head**: 2-3 DOF (pan, tilt, roll)
- **Hands** (each): 1-20+ DOF (simple grippers to fully articulated fingers)

**Design Principle**: More DOF increases dexterity but adds weight, cost, complexity, and control difficulty. The optimal DOF count depends on task requirements.

### Actuation Technologies

Different actuator types suit different humanoid applications:

| Actuator Type | Advantages | Disadvantages | Use Cases |
|--------------|------------|---------------|-----------|
| **Electric Motors (DC/BLDC)** | Precise control, efficient, quiet | Lower power density | Research platforms, service robots |
| **Hydraulic** | High power density, strong | Noisy, heavy, maintenance | Heavy-duty tasks, construction |
| **Pneumatic** | Compliant, safe | Low precision, requires compressor | Soft robotics, safe interaction |
| **Series Elastic Actuators (SEA)** | Force control, shock absorption | Added complexity, size | Dynamic walking, manipulation |

**Atlas (Boston Dynamics)**: Uses hydraulic actuators for explosive power in dynamic movements like backflips and parkour.

**ASIMO (Honda)**: Electric motors for quiet, precise indoor operation in service environments.

### Sensors for Humanoid Perception

Humanoid robots require rich sensory feedback for balance, navigation, and manipulation:

**Proprioceptive Sensors** (internal state):
- **Joint encoders**: Measure joint angles for kinematic state estimation
- **Torque sensors**: Detect forces at joints for compliance control
- **Inertial Measurement Unit (IMU)**: Accelerometers and gyroscopes for orientation and acceleration
- **Force-torque sensors**: Measure contact forces at feet, hands, wrists

**Exteroceptive Sensors** (external environment):
- **Cameras** (RGB, stereo, depth): Visual perception for navigation and object recognition
- **LIDAR**: 3D scanning for obstacle avoidance and mapping
- **Tactile sensors**: Pressure-sensitive skin for contact detection
- **Microphones**: Audio input for speech recognition and sound localization

**Sensor Fusion Challenge**: Integrating data from 50+ sensors at high frequency (100-1000Hz) while managing noise, latency, and sensor failures.

### Power and Energy Management

Humanoid robots face severe energy constraints:

**Energy Storage**:
- **Lithium-ion batteries**: 100-300 Wh/kg, typical runtime 1-3 hours
- **Weight tradeoff**: Larger battery extends runtime but reduces mobility
- **Thermal management**: High-power actuators generate heat requiring cooling

**Power Budget Example** (typical humanoid):
- **Idle standing**: 50-100W (sensor processing, balance control)
- **Walking**: 200-500W (actuators, computation)
- **Running/jumping**: 1000-2000W+ (peak actuation power)

**Efficiency Strategies**:
- Regenerative braking in joints to recover energy
- Passive dynamics: leveraging natural pendulum motion of limbs
- Intermittent high-power modes: sprint briefly, rest frequently

## Bipedal Locomotion Control

Walking on two legs is a complex control problem requiring continuous balance and gait coordination.

### The Balance Problem

Unlike quadrupeds or wheeled robots, bipedal robots have a small support polygon (area defined by foot contacts). The robot's **center of mass (CoM)** must remain above this polygon to avoid tipping.

**Key Concepts**:

1. **Zero Moment Point (ZMP)**: The point on the ground where the sum of moments from gravity and inertia equals zero. For stable walking, ZMP must stay within the support polygon.

2. **Center of Pressure (CoP)**: The point where the ground reaction force acts. CoP coincides with ZMP in ideal conditions.

3. **Capture Point**: The point on the ground where the robot can step to come to a complete stop. Used for predictive balance control.

### Gait Phases

A walking gait consists of alternating **stance** and **swing** phases:

**Single Support Phase**:
- One foot on the ground (stance foot)
- Other leg swinging forward (swing leg)
- Robot is statically unstable, requires dynamic balance

**Double Support Phase**:
- Both feet on the ground
- Transfer weight from trailing foot to leading foot
- Brief period (10-20% of gait cycle)

**Gait Cycle**:
```
Right Stance → Double Support → Left Stance → Double Support → Right Stance...
```

### Walking Control Strategies

**1. ZMP-Based Control** (ASIMO, HRP-series robots):
- Pre-plan footstep locations and CoM trajectory
- Enforce ZMP within support polygon at all times
- Stable but conservative, limited to flat terrain

**2. Capture Point Control**:
- Compute where to step to maintain balance under current momentum
- Enables faster, more dynamic walking
- Used in robots like Atlas for rough terrain

**3. Model Predictive Control (MPC)**:
- Optimize future trajectory over finite horizon (0.5-2 seconds)
- Consider upcoming terrain, obstacles, task goals
- Replan at high frequency (10-100Hz) for reactive adaptation

**4. Learning-Based Control** (modern approach):
- Train neural network policies in simulation
- Learn robust gaits that generalize to varied terrain
- Examples: Google's Robotics Transformer (RT), DeepMind's MuJoCo-trained policies

### Balance Recovery

When disturbed (pushed, stepping on uneven ground), humanoids must recover balance:

**Strategies**:
1. **Ankle strategy**: Rotate about ankle joint to shift CoM (small disturbances)
2. **Hip strategy**: Bend at hips to move CoM quickly (medium disturbances)
3. **Step strategy**: Take a step to widen support polygon (large disturbances)
4. **Reaching strategy**: Extend arms to grab support or shift momentum

**Example**: When Atlas is pushed, it rapidly evaluates whether ankle/hip adjustments suffice. If not, it computes a recovery step location using capture point prediction, executes the step within ~300ms.

## Whole-Body Motion Planning

Humanoid robots must coordinate all joints to achieve tasks while maintaining balance and avoiding obstacles.

### Inverse Kinematics (IK)

Given a desired **end-effector pose** (e.g., hand position and orientation), find joint angles that achieve it.

**Challenges**:
- **Redundancy**: Humanoid arms have 7 DOF but only need 6 to specify a pose (3 position + 3 orientation). Infinite solutions exist.
- **Joint limits**: Physical constraints on how far joints can rotate
- **Singularities**: Configurations where small end-effector motions require infinite joint velocities

**Solutions**:
- **Damped least squares**: Numerical IK solver robust to singularities
- **Optimization-based IK**: Minimize cost function (e.g., prefer elbow down, minimize joint travel)
- **Learning-based IK**: Train neural networks to predict joint angles from end-effector goals

### Whole-Body Control Frameworks

Coordinate all DOF to satisfy multiple objectives simultaneously:

**Hierarchical Quadratic Programming (HQP)**:
- Prioritize tasks: balance > collision avoidance > hand reaching > joint limits
- Solve QP optimization at each control cycle (1-10ms)
- Used in robots like TALOS, TORO

**Operational Space Control**:
- Define task spaces (e.g., hand position, CoM position, foot orientation)
- Compute joint torques to achieve desired accelerations in task space
- Decouple tasks: moving hand doesn't affect balance

**Example Task Hierarchy** (robot reaching for object while walking):
1. **Highest priority**: Maintain balance (CoM above support polygon)
2. **Second priority**: Avoid collision with environment
3. **Third priority**: Move hand to grasp position
4. **Fourth priority**: Minimize joint velocities (smooth motion)

If lower-priority tasks conflict with higher ones, they are relaxed.

## State-of-the-Art Humanoid Robots

### Atlas (Boston Dynamics)

**Specifications**:
- Height: 1.5m, Weight: 89kg
- Actuators: Hydraulic, 28 DOF
- Sensors: LIDAR, stereo cameras, IMU, joint encoders

**Capabilities**:
- **Dynamic locomotion**: Running, jumping, backflips, parkour
- **Rough terrain**: Navigate rocks, stairs, uneven ground
- **Manipulation**: Pick up boxes, open doors, use tools

**Control Approach**:
- Model predictive control for footstep planning
- Whole-body optimization for coordinated movement
- Extensive sim-to-real transfer with domain randomization

**Use Case**: Search and rescue, hazardous environment inspection

### Optimus (Tesla)

**Specifications**:
- Height: 1.73m, Weight: 73kg
- Actuators: Custom electric motors, 40+ DOF
- Sensors: Cameras (no LIDAR), neural network perception

**Capabilities**:
- **Object manipulation**: Sort items, operate tools
- **Walking**: Flat terrain navigation
- **Learning**: End-to-end neural network control

**Design Philosophy**:
- Leverage Tesla's self-driving AI and manufacturing expertise
- Mass production at automotive scale ($20-30k target cost)
- Vision-only perception (similar to Tesla Full Self-Driving)

**Use Case**: Factory automation, household assistance

### Digit (Agility Robotics)

**Specifications**:
- Height: 1.75m, Weight: 65kg
- Actuators: Electric, 20 DOF
- Sensors: LIDAR, depth cameras, IMU

**Capabilities**:
- **Box handling**: Pick, carry, stack packages
- **Stairs and curbs**: Navigate typical delivery environments
- **Bipedal walking**: Stable gait on flat and sloped terrain

**Design Focus**:
- Practical logistics tasks (warehouse, delivery)
- Robust, simple design for deployment
- Streamlined humanoid (torso-less for compactness)

**Use Case**: Package delivery, warehouse automation (partnered with Amazon)

### Comparison Table

| Robot | Creator | Height (m) | Weight (kg) | Actuators | Primary Application |
|-------|---------|-----------|-------------|-----------|---------------------|
| **Atlas** | Boston Dynamics | 1.5 | 89 | Hydraulic | Search & rescue, research |
| **Optimus** | Tesla | 1.73 | 73 | Electric | Manufacturing, home assistance |
| **Digit** | Agility Robotics | 1.75 | 65 | Electric | Logistics, delivery |
| **ASIMO** | Honda (retired) | 1.3 | 48 | Electric | Service, research |

## Human-Robot Interaction for Humanoids

Humanoid robots designed to work alongside humans must consider safety, communication, and social norms.

### Safety Considerations

**Physical Safety**:
- **Collision detection**: Torque sensors detect unexpected contact, trigger emergency stop
- **Compliant control**: Yield when touched, don't resist human force
- **Speed limits**: Reduce velocity when humans are nearby (detected via cameras)
- **Emergency stop**: Accessible kill switch, wireless e-stop

**Standards**:
- **ISO 10218**: Industrial robot safety (collaborative operation modes)
- **ISO/TS 15066**: Collaborative robots (force/pressure limits for human contact)

### Communication Modalities

**Verbal Communication**:
- **Speech recognition**: Understand natural language commands
- **Text-to-speech**: Provide status updates, ask clarifying questions
- **Challenges**: Noisy environments, accents, ambiguous phrasing

**Non-Verbal Communication**:
- **Gestures**: Point to objects, wave to acknowledge
- **Gaze**: Look at person speaking, indicate attention
- **Facial expressions**: LEDs or screens display simple emotions (alert, processing, error)
- **Body language**: Lean forward (engagement), lean back (yielding space)

### Trust and Acceptance

Factors influencing human acceptance of humanoid robots:

1. **Predictability**: Clear, consistent behavior reduces anxiety
2. **Transparency**: Explain actions ("I'm moving to the shelf to retrieve the box")
3. **Appearance**: Uncanny valley effect - overly realistic faces can be unsettling; stylized designs often preferred
4. **Competence**: Reliable task execution builds trust over time
5. **Social norms**: Respect personal space, wait for turn in conversation

**Example**: Softbank's Pepper robot uses a friendly, cartoon-like face and gentle voice to avoid uncanny valley while engaging customers in retail settings.

## Exercises

### Exercise 1: DOF Analysis

Consider three tasks for a humanoid robot:

1. **Task A**: Walk forward on flat ground
2. **Task B**: Climb a ladder
3. **Task C**: Assemble a car dashboard (requires bimanual dexterity)

**Questions**:
1. What is the minimum number of DOF required in the legs for Task A? Why?
2. How do DOF requirements change for Task B (ladder climbing) compared to Task A?
3. For Task C, how many DOF would you allocate to each hand, and what gripper design would you choose?
4. If you had to reduce total robot DOF by 20% due to cost constraints, which joints would you simplify or remove for each task?

### Exercise 2: Balance Recovery Simulation

You are designing a balance controller for a humanoid robot. The robot is standing on one leg when pushed with a force.

**Scenario**: Robot mass = 80kg, CoM height = 1.0m, foot size = 0.25m x 0.12m, push force = 100N applied horizontally at hip.

**Questions**:
1. Estimate the disturbance torque about the ankle. Will ankle strategy suffice?
2. If not, where should the robot step to recover? (Use capture point heuristic: step ~0.5 * sqrt(CoM_height / g) * velocity ahead)
3. How quickly must the robot execute the step to avoid falling? (Estimate fall time using inverted pendulum model)
4. Design a simple state machine for balance recovery: [Idle] -> [Detect Push] -> [Choose Strategy] -> [Execute Recovery] -> [Idle]

### Exercise 3: Humanoid vs Non-Humanoid Comparison

Compare a humanoid robot (e.g., Digit) to a wheeled mobile manipulator (e.g., Boston Dynamics Spot) for a warehouse environment.

**Evaluation Criteria**:
1. **Stairs and uneven terrain**: Which robot handles better? Why?
2. **Energy efficiency**: Which robot uses less energy per meter traveled?
3. **Payload capacity**: Which can carry heavier loads?
4. **Workspace reach**: Which has better vertical reach (high shelves)?
5. **Complexity and cost**: Which is simpler and cheaper?
6. **Human interaction**: Which is more intuitive for workers to collaborate with?

**Conclusion**: For which warehouse tasks would you choose humanoid vs wheeled? Justify your decision.

## Key Takeaways

- Humanoid form enables robots to navigate human-built environments and use human tools without modification
- Bipedal locomotion requires continuous balance control using ZMP, capture point, or learning-based methods
- Whole-body control coordinates all joints to satisfy multiple objectives (balance, reach, avoid obstacles) simultaneously
- State-of-the-art humanoids like Atlas, Optimus, and Digit demonstrate practical capabilities in dynamic movement, manipulation, and logistics
- Human-robot interaction requires safety mechanisms, natural communication, and attention to social norms to build trust

## References

1. Kajita, S., et al. (2014). **Introduction to Humanoid Robotics**. Springer. https://doi.org/10.1007/978-3-642-54536-8

2. Siciliano, B., & Khatib, O. (Eds.). (2016). **Springer Handbook of Robotics** (2nd ed.). Springer. (Chapters on Humanoid Robots and Legged Robots)

3. Hirai, K., et al. (1998). **The development of Honda humanoid robot ASIMO**. IEEE/RSJ International Conference on Intelligent Robots and Systems. https://doi.org/10.1109/IROS.1998.724781

4. Kuindersma, S., et al. (2016). **Optimization-based locomotion planning, estimation, and control design for the Atlas humanoid robot**. Autonomous Robots, 40(3). https://doi.org/10.1007/s10514-015-9479-3

5. Englsberger, J., et al. (2015). **Three-dimensional bipedal walking control based on divergent component of motion**. IEEE Transactions on Robotics. https://doi.org/10.1109/TRO.2015.2405592

6. Metta, G., et al. (2010). **The iCub humanoid robot: An open-systems platform for research in cognitive development**. Neural Networks, 23(8-9). https://doi.org/10.1016/j.neunet.2010.08.010

7. Boston Dynamics. (2023). **Atlas: The most dynamic humanoid robot**. https://www.bostondynamics.com/atlas

8. Tesla. (2024). **Optimus Gen 2**. https://www.tesla.com/optimus

---

**Previous Chapter**: [← Chapter 1: Introduction to Physical AI](../01-introduction)
**Next Chapter**: [Chapter 3: ROS2 Fundamentals →](../03-ros2-fundamentals)
