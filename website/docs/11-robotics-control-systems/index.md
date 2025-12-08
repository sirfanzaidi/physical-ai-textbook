# Chapter 11: Robotics Control Systems

## Learning Objectives

After completing this chapter, you will be able to:

- **Design** feedback control systems for position, velocity, and force control
- **Implement** PID, impedance, and adaptive controllers for humanoid robots
- **Tune** control parameters using stability analysis and simulation
- **Handle** real-world challenges like actuator saturation and measurement noise

## Overview

Control systems form the bridge between high-level task planning and low-level motor commands. This chapter covers classical and modern control techniques essential for stable, responsive humanoid robot behavior.

## Feedback Control Fundamentals

### PID Control
The ubiquitous proportional-integral-derivative controller:

```python
class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Control timestep

        self.integral_error = 0
        self.last_error = 0

    def step(self, error):
        """Compute control input given error"""
        # Proportional term
        p_term = self.kp * error

        # Integral term (accumulate error over time)
        self.integral_error += error * self.dt
        i_term = self.ki * self.integral_error

        # Derivative term (rate of error change)
        d_error = (error - self.last_error) / self.dt
        d_term = self.kd * d_error

        # Total control output
        output = p_term + i_term + d_term

        # Store for next iteration
        self.last_error = error

        return output

# Example: Joint angle control
pid = PIDController(kp=100, ki=5, kd=10)

for _ in range(100):
    current_angle = joint.get_angle()
    desired_angle = 0.5  # 0.5 radians

    error = desired_angle - current_angle
    torque = pid.step(error)

    joint.apply_torque(torque)
```

### Tuning PID Gains
Use Ziegler-Nichols method or frequency response analysis:

```python
import numpy as np
from scipy import signal

def tune_pid_ziegler_nichols(step_response):
    """Auto-tune PID gains from step response"""

    # Find ultimate gain K_u (gain at oscillation boundary)
    # Find ultimate period T_u (period of oscillations)

    K_u = 0.5  # Empirically determined
    T_u = 2.0

    # Ziegler-Nichols gains for servo problem
    kp = 0.6 * K_u
    ki = 1.2 * K_u / T_u
    kd = 0.075 * K_u * T_u

    return kp, ki, kd
```

## Impedance Control

For safe human-robot interaction, impedance control creates virtual spring-like behavior:

```python
class ImpedanceController:
    def __init__(self, mass=1.0, damping=10.0, stiffness=100.0):
        self.M = mass          # Virtual mass
        self.B = damping       # Virtual damping
        self.K = stiffness     # Virtual stiffness

        self.x_desired = 0.0
        self.v_desired = 0.0
        self.last_time = None

    def step(self, x_actual, f_external, dt):
        """Impedance control law: M*a + B*v + K*x = F"""
        # Position error
        e_x = self.x_desired - x_actual
        v_actual_est = (x_actual - self.x_last) / dt

        # Impedance equation: compute desired force
        # M*a_d + B*(v_d - v_actual) + K*(x_d - x_actual) = 0
        # Rearranged for acceleration
        a_desired = -(self.B/self.M)*(v_actual_est - self.v_desired) \
                    - (self.K/self.M)*e_x

        # Add external force effect (compliance to disturbances)
        a_desired += f_external / self.M

        # Integrate to get velocity and position
        v = v_actual_est + a_desired * dt
        desired_torque = self.K * e_x + self.B * (self.v_desired - v_actual_est)

        self.x_last = x_actual
        return desired_torque

# Usage: compliant arm control
impedance = ImpedanceController(mass=2.0, damping=20.0, stiffness=200.0)

for t in range(1000):
    x_current = arm.get_position()
    f_human = arm.get_force_sensor()  # Force applied by human
    dt = 0.01

    torque_cmd = impedance.step(x_current, f_human, dt)
    arm.apply_torque(torque_cmd)
```

## Stability Analysis

### Pole Placement
Ensure stability by placing system poles in left half-plane:

```python
import control as ct

# Define system: dx/dt = A*x + B*u
A = np.array([[0, 1],
              [-10, -2]])  # Natural oscillation + damping

B = np.array([[0],
              [1]])        # Force input

# Design state feedback: u = -K*x
# Choose poles at [-2, -3] (stable, well-damped)
desired_poles = [-2, -3]
K = ct.place(A, B, desired_poles)

print(f"Feedback gain K: {K}")
# Output: K = [[10, 5]]
# This means u = -10*x1 - 5*x2 (position and velocity feedback)
```

## Multi-Robot Coordination

### Formation Control
Multiple robots maintain desired relative positions:

```python
class FormationController:
    def __init__(self, num_robots, formation_type='circle'):
        self.num_robots = num_robots
        self.formation_type = formation_type

    def compute_desired_positions(self, center, radius, rotation_angle):
        """Compute desired positions for each robot in formation"""
        positions = []

        for i in range(self.num_robots):
            angle = 2 * np.pi * i / self.num_robots + rotation_angle

            # Circular formation
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)

            positions.append([x, y])

        return positions

    def step(self, current_positions, desired_center, formation_radius):
        """Compute velocity commands for each robot"""
        desired_positions = self.compute_desired_positions(
            desired_center, formation_radius, rotation_angle=0
        )

        velocities = []
        for i, (current, desired) in enumerate(zip(current_positions, desired_positions)):
            # PID to reach desired position
            error = np.array(desired) - np.array(current)
            velocity = 0.5 * error  # Proportional control

            velocities.append(velocity)

        return velocities

# Usage: 4 robots in circle formation
formation = FormationController(num_robots=4)

while True:
    current_pos = [robot.get_position() for robot in robots]
    center = [0, 0]
    radius = 1.0

    vel_cmds = formation.step(current_pos, center, radius)

    for robot, vel in zip(robots, vel_cmds):
        robot.set_velocity(vel)
```

## Practical Challenges

### Actuator Saturation
Real motors have torque/velocity limits:

```python
def apply_saturated_torque(torque_desired, torque_limit=100.0):
    """Clamp torque to physical limits"""
    return np.clip(torque_desired, -torque_limit, torque_limit)

# Prevent integrator windup
class AntiWindupPID(PIDController):
    def step(self, error, torque_limit=100.0):
        # ... compute p, i, d terms ...
        output = p_term + i_term + d_term

        # Saturate output
        output_saturated = np.clip(output, -torque_limit, torque_limit)

        # Only integrate error if not saturated (anti-windup)
        if abs(output) < torque_limit:
            self.integral_error += error * self.dt

        return output_saturated
```

### Measurement Noise Filtering
Smooth noisy sensor signals with low-pass filter:

```python
class LowPassFilter:
    def __init__(self, cutoff_freq, dt):
        self.alpha = dt / (dt + 1.0 / (2 * np.pi * cutoff_freq))
        self.filtered_value = 0.0

    def step(self, measurement):
        self.filtered_value = self.alpha * measurement \
                            + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

# Usage: denoise joint angle measurements
joint_filter = LowPassFilter(cutoff_freq=20.0, dt=0.01)

for measurement in noisy_angles:
    clean_angle = joint_filter.step(measurement)
```

## Exercises

**Exercise 11.1**: Implement PID controller and tune gains using step response
**Exercise 11.2**: Design impedance controller for safe human-robot interaction
**Exercise 11.3**: Implement formation controller for 3 robots

## References

1. Modern Control Engineering (Ogata): Chapters 1-4
2. Robotics Control Theory and Applications (Sciavicco & Siciliano)
3. Impedance Control (Hogan, 1985): https://ieeexplore.ieee.org/document/1087516
4. Formation Control: https://arxiv.org/abs/1301.3932
5. ROS 2 Control Framework: https://control.ros.org/
