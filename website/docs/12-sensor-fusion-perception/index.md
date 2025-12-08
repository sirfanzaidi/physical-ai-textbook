# Chapter 12: Sensor Fusion & Perception

## Learning Objectives

After completing this chapter, you will be able to:

- **Fuse** multiple sensor modalities (camera, LiDAR, IMU, depth) using probabilistic methods
- **Implement** visual odometry and SLAM for robot localization
- **Detect** and track objects in real-time using deep learning
- **Handle** sensor noise and latency in real-world systems

## Overview

Humanoid robots perceive their environment through diverse sensors: cameras (RGB, depth, thermal), LiDAR, IMU, force sensors. Sensor fusion combines these modalities for robust perception despite noise and occlusions.

## Multi-Sensor Modalities

### Sensor Types and Characteristics

| Sensor | Modality | Advantages | Limitations | Use Cases |
|--------|----------|------------|-------------|-----------|
| **RGB Camera** | Vision | Rich color info, semantics | 2D projection, no depth | Object recognition, face detection |
| **Depth (RGB-D)** | 3D | Precise distance, 3D structure | Limited range, occlusions | Grasping, 3D mapping |
| **LiDAR** | 3D | Long range, 3D point clouds | Expensive, sparse data | Outdoor navigation, mapping |
| **IMU** | Inertial | Accel, gyro, magnetometer | Drift over time | Balance, fall detection |
| **Force/Torque** | Tactile | Contact forces | Limited spatial resolution | Manipulation, grasp stability |

## Probabilistic Sensor Fusion

### Kalman Filter
Optimal linear estimate combining measurements with dynamics model:

```python
import numpy as np

class KalmanFilter:
    def __init__(self, A, H, Q, R, P, x0):
        """
        A: State transition matrix
        H: Measurement matrix
        Q: Process noise covariance
        R: Measurement noise covariance
        P: Initial state covariance
        x0: Initial state
        """
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P
        self.x = x0

    def predict(self):
        """Predict next state"""
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        """Update with measurement"""
        # Innovation (measurement residual)
        y = z - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ self.H) @ self.P

        return self.x

# Example: Fuse IMU and wheel odometry for position
# State: [x, y, theta, v_x, v_y]
A = np.eye(5)  # Assume constant velocity
A[0, 3] = 0.1  # dt = 0.1
A[1, 4] = 0.1

H = np.array([[1, 0, 0, 0, 0],  # Wheel odometry measures x
              [0, 1, 0, 0, 0]])  # Wheel odometry measures y

kf = KalmanFilter(A=A, H=H, Q=np.eye(5)*0.01, R=np.eye(2)*0.1,
                  P=np.eye(5), x0=np.zeros(5))

# Update loop
for t in range(1000):
    kf.predict()
    measurement = get_wheel_odometry()  # [x, y]
    state = kf.update(measurement)
```

### Extended Kalman Filter (EKF)
For nonlinear systems:

```python
class ExtendedKalmanFilter:
    def __init__(self, f, h, F_jacobian, H_jacobian, Q, R, P, x0):
        """
        f: Nonlinear state transition function
        h: Nonlinear measurement function
        F_jacobian: Jacobian of f
        H_jacobian: Jacobian of h
        """
        self.f = f
        self.h = h
        self.F_jacobian = F_jacobian
        self.H_jacobian = H_jacobian
        self.Q = Q
        self.R = R
        self.P = P
        self.x = x0

    def predict(self):
        self.x = self.f(self.x)
        F = self.F_jacobian(self.x)
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        # Measurement Jacobian
        H = self.H_jacobian(self.x)

        # Innovation
        y = z - self.h(self.x)

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P

        return self.x
```

## Visual Odometry & SLAM

### Optical Flow
Track feature motion between frames:

```python
import cv2

def compute_optical_flow(frame1, frame2):
    """Compute optical flow using Lucas-Kanade"""
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Find corners
    corners = cv2.goodFeaturesToTrack(gray1, maxCorners=200, qualityLevel=0.01, minDistance=30)

    # Track corners
    flow, st, err = cv2.calcOpticalFlowLK(gray1, gray2, corners, (15, 15), 3)

    # Estimate motion from flow
    if flow is not None:
        avg_motion = np.mean(flow[st==1], axis=0)
        return avg_motion
    return None

# Visual odometry loop
prev_frame = None
position = [0, 0]

for frame in video_stream:
    if prev_frame is not None:
        motion = compute_optical_flow(prev_frame, frame)
        if motion is not None:
            position[0] += motion[0]
            position[1] += motion[1]

    prev_frame = frame
```

### SLAM (Simultaneous Localization and Mapping)
Build map while localizing robot:

```python
class SimpleSLAM:
    def __init__(self):
        self.map = {}  # Feature ID -> 3D position
        self.pose = np.array([0, 0, 0])  # [x, y, theta]
        self.feature_id = 0

    def add_observation(self, measured_features):
        """
        measured_features: List of [range, bearing, height] to observed features
        """
        for r, bearing, h in measured_features:
            # Convert polar to Cartesian in robot frame
            fx = r * np.cos(bearing)
            fy = r * np.sin(bearing)

            # Transform to world frame
            cos_theta = np.cos(self.pose[2])
            sin_theta = np.sin(self.pose[2])

            world_x = self.pose[0] + cos_theta * fx - sin_theta * fy
            world_y = self.pose[1] + sin_theta * fx + cos_theta * fy

            # Update or create map entry
            feature_key = self.get_closest_feature(world_x, world_y)

            if feature_key is None:
                self.map[self.feature_id] = [world_x, world_y, h]
                self.feature_id += 1
            else:
                # Update existing feature (moving average)
                self.map[feature_key][0] = 0.9 * self.map[feature_key][0] + 0.1 * world_x
                self.map[feature_key][1] = 0.9 * self.map[feature_key][1] + 0.1 * world_y

    def update_pose(self, linear_velocity, angular_velocity, dt):
        """Dead reckoning from motion"""
        self.pose[0] += linear_velocity * dt * np.cos(self.pose[2])
        self.pose[1] += linear_velocity * dt * np.sin(self.pose[2])
        self.pose[2] += angular_velocity * dt
```

## Deep Learning for Perception

### Object Detection
Real-time detection using YOLO or similar:

```python
import torch
from torchvision.models import detection

class ObjectDetector:
    def __init__(self, model='faster_rcnn'):
        if model == 'faster_rcnn':
            self.model = detection.fasterrcnn_resnet50_fpn(pretrained=True)
        self.model.eval()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)

    def detect(self, image):
        """Detect objects in image"""
        # Preprocess
        x = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
        x = x.unsqueeze(0).to(self.device)

        with torch.no_grad():
            predictions = self.model(x)

        # Parse results
        boxes = predictions[0]['boxes'].cpu().numpy()
        scores = predictions[0]['scores'].cpu().numpy()
        labels = predictions[0]['labels'].cpu().numpy()

        # Filter low confidence
        threshold = 0.5
        mask = scores > threshold

        return {
            'boxes': boxes[mask],
            'scores': scores[mask],
            'labels': labels[mask]
        }

# Usage
detector = ObjectDetector()

while True:
    frame = camera.get_frame()
    detections = detector.detect(frame)

    for box, score, label in zip(detections['boxes'], detections['scores'], detections['labels']):
        x1, y1, x2, y2 = box
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f'{label}:{score:.2f}', (int(x1), int(y1)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
```

## Handling Real-World Challenges

### Sensor Latency Compensation
Predict state at current time from delayed measurement:

```python
def compensate_latency(state_at_measurement_time, latency, control_history):
    """Forward predict state from past measurement"""
    state = state_at_measurement_time

    # Replay control commands that occurred during latency
    for control_input in control_history[-latency:]:
        state = dynamics_model(state, control_input, dt=0.01)

    return state
```

### Outlier Rejection
Reject measurements that don't match model predictions:

```python
def mahalanobis_distance(measurement, predicted, covariance):
    """Outlier detection using Mahalanobis distance"""
    innovation = measurement - predicted
    try:
        cov_inv = np.linalg.inv(covariance)
        distance = innovation @ cov_inv @ innovation.T
        return distance
    except:
        return float('inf')

# Threshold for outliers (chi-squared with 2 DOF)
outlier_threshold = 6.0

if mahalanobis_distance(z, h_x, S) < outlier_threshold:
    state = kf.update(z)
```

## Exercises

**Exercise 12.1**: Implement Kalman filter for robot localization using encoder odometry and compass
**Exercise 12.2**: Build simple visual odometry system from stereo camera
**Exercise 12.3**: Fuse IMU, camera, and LiDAR for robust pose estimation

## References

1. Probabilistic Robotics (Thrun, Burgard, Fox): Chapters 2-4
2. Multiple View Geometry in Computer Vision (Hartley & Zisserman)
3. ORB-SLAM: https://arxiv.org/abs/1502.00956
4. YOLO Object Detection: https://arxiv.org/abs/2104.01738
5. ROS 2 Perception Stack: https://docs.ros.org/en/humble/Concepts/About-Middleware.html
