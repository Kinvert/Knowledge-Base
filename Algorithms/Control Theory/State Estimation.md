---
title: State Estimation
tags: [algorithms, control-theory, robotics, estimation, filtering, sensor-fusion]
aliases: [State Observer, State Estimator, Estimation Theory]
---

# 🧮 State Estimation

## 🧭 Overview

**State estimation** is the process of inferring the internal state of a dynamic system from indirect, noisy, or incomplete measurements. In engineering, robotics, and control theory, state estimation is essential for feedback control, navigation, sensor fusion, and autonomous systems. It enables systems to "know" their position, velocity, orientation, or other internal variables that cannot be measured directly.

---

## 🛠️ Key Features

- **Combines Multiple Sensors**: Fuses data from various sources (e.g., IMU, GPS, cameras) to improve accuracy and robustness.
- **Handles Noise and Uncertainty**: Uses probabilistic models to account for sensor noise and process disturbances.
- **Real-Time Operation**: Designed for online, recursive updates as new data arrives.
- **Foundation for Control**: Provides reliable state information for feedback controllers (e.g., PID, MPC).
- **Supports Nonlinear and Linear Systems**: Can be applied to both types using different algorithms.

---

## 📦 Common Use Cases

- **Robotics**: Localization, mapping (SLAM), and motion tracking.
- **Autonomous Vehicles**: Estimating position, velocity, and orientation for navigation and control.
- **Aerospace**: Attitude and orbit determination for aircraft and spacecraft.
- **Industrial Automation**: Monitoring and controlling machinery and processes.
- **Sensor Fusion**: Combining IMU, GPS, LiDAR, and other sensor data for robust estimation.

---

## 🧩 Common Algorithms

- **Kalman Filter**: Optimal for linear systems with Gaussian noise.
- **Extended Kalman Filter (EKF)**: Handles nonlinear systems by linearizing around the current estimate.
- **Unscented Kalman Filter (UKF)**: Uses a deterministic sampling approach for nonlinear systems.
- **Particle Filter**: Handles highly nonlinear, non-Gaussian systems using a set of weighted samples.
- **Observers (e.g., Luenberger Observer)**: Used in control theory for deterministic systems.

---

## 🆚 State Estimation vs. Filtering

| Aspect                | State Estimation         | Filtering                  |
|-----------------------|-------------------------|----------------------------|
| **Goal**              | Estimate full system state | Remove noise from signals |
| **Techniques**        | Kalman, EKF, UKF, Particle | Moving average, low-pass  |
| **Application**       | Control, navigation, robotics | Signal processing        |
| **Output**            | Multi-dimensional state vector | Smoothed signal          |

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Improves System Performance**: Enables accurate feedback and control.
- **Robust to Noise**: Handles sensor and process noise effectively.
- **Enables Autonomy**: Critical for robotics and autonomous vehicles.
- **Flexible**: Applicable to a wide range of systems and sensors.

### ❌ Disadvantages
- **Model Dependency**: Requires accurate system and noise models.
- **Computational Complexity**: Advanced algorithms (e.g., particle filters) can be resource-intensive.
- **Tuning Required**: Performance depends on correct parameter tuning.

---

## 🔗 Related Topics

- [[Kalman Filter]]
- [[Extended Kalman Filter]]
- [[Unscented Kalman Filter]]
- [[Particle Filter]]
- [[Sensor Fusion]]
- [[SLAM]]
- [[Control Theory]]
- [[IMU]]
- [[Robotics]]

---

## 📚 Further Reading

- [Wikipedia: State Estimation](https://en.wikipedia.org/wiki/State_estimation_(control_theory))
- [Probabilistic Robotics (Book)](https://www.probabilistic-robotics.org/)
- [Kalman Filter Tutorial (Welch & Bishop)](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)
- [Sensor Fusion and State Estimation (YouTube)](https://www.youtube.com/watch?v=2pzxEmh0gYw)
- [State Estimation in Robotics (ROS)](https://wiki.ros.org/robot_pose_ekf)

---

## 🧠 Summary

State estimation is a foundational concept in control, robotics, and autonomous systems. By combining noisy sensor data and mathematical models, it enables reliable inference of system states that are critical for feedback, navigation, and decision-making.
