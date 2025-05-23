---
title: Extended Kalman Filter (EKF)
tags: [algorithms, estimation, control-theory, sensor-fusion, robotics, filtering]
aliases: [EKF, Nonlinear Kalman Filter, Extended KF]
---

# 📉 Extended Kalman Filter (EKF)

## 🧭 Overview

The **Extended Kalman Filter (EKF)** is an extension of the standard Kalman Filter designed to handle **nonlinear** dynamic systems. While the classic Kalman Filter is optimal for linear systems with Gaussian noise, the EKF adapts the algorithm to nonlinear models by linearizing them around the current estimate using a first-order Taylor expansion (Jacobian).

EKF is widely used in robotics, navigation, sensor fusion, and SLAM, where system dynamics or measurement models are nonlinear.

---

## 🛠️ Key Features

- **Nonlinear System Support**: Handles nonlinear process and measurement models.
- **Linearization**: Uses Jacobian matrices to approximate nonlinear functions.
- **Recursive Estimation**: Updates state estimates as new measurements arrive.
- **Sensor Fusion**: Integrates data from multiple sensors (e.g., IMU, GPS, cameras).
- **Real-Time Operation**: Suitable for embedded and real-time systems.

---

## 📦 Common Use Cases

- **Robotics**: Localization, mapping (SLAM), and motion tracking with nonlinear kinematics.
- **Autonomous Vehicles**: Fusing GPS, IMU, and wheel odometry for accurate state estimation.
- **Aerospace**: Attitude and orbit determination for aircraft and spacecraft.
- **Sensor Fusion**: Combining nonlinear sensor data for robust estimation.

---

## 🧩 EKF vs. Standard Kalman Filter

| Aspect                | Kalman Filter (KF)         | Extended Kalman Filter (EKF)      |
|-----------------------|----------------------------|------------------------------------|
| **System Type**       | Linear                     | Nonlinear (linearized locally)     |
| **Model Functions**   | Matrix multiplication      | Arbitrary nonlinear functions      |
| **Linearization**     | Not needed                 | Required (Jacobian computation)    |
| **Optimality**        | Optimal for linear systems | Approximate for nonlinear systems  |
| **Complexity**        | Lower                      | Higher (due to Jacobians)          |
| **Use Cases**         | Linear estimation problems | Robotics, SLAM, sensor fusion      |

**Key Difference:**  
- The standard Kalman Filter assumes both the process and measurement models are linear, so it uses matrix operations directly.
- The EKF allows for nonlinear models by linearizing them at each step using the Jacobian, making it suitable for real-world systems where perfect linearity is rare.

---

## 🧩 How EKF Works

1. **Prediction Step**:  
   - Predicts the next state using the nonlinear process model.
   - Linearizes the process model around the current estimate (Jacobian).

2. **Update Step**:  
   - Incorporates new measurements using the nonlinear measurement model.
   - Linearizes the measurement model (Jacobian) to update the estimate.

3. **Covariance Update**:  
   - Propagates uncertainty through the linearized models.

---

## 🆚 EKF vs. Other Nonlinear Filters

| Filter Type             | System Type      | Linearization | Complexity   | Use Cases                  |
|-------------------------|------------------|---------------|--------------|----------------------------|
| **EKF**                 | Nonlinear        | Jacobian      | Moderate     | Robotics, SLAM, sensor fusion |
| **Unscented Kalman Filter (UKF)** | Nonlinear | None (uses sigma points) | Higher | Nonlinear estimation       |
| **Particle Filter**     | Nonlinear        | None          | High         | Highly nonlinear, non-Gaussian |

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Handles Nonlinear Systems**: Extends Kalman filtering to a broader range of problems.
- **Widely Used**: Standard in robotics and navigation.
- **Real-Time Capable**: Efficient enough for embedded systems.

### ❌ Disadvantages
- **Approximate**: Only locally optimal; can diverge if linearization is poor.
- **Requires Jacobians**: Analytical or numerical derivatives can be complex to compute.
- **Sensitive to Model Errors**: Inaccurate models or poor initial estimates can degrade performance.

---

## 🔗 Related Topics

- [[Kalman Filter]]
- [[Unscented Kalman Filter]]
- [[Particle Filter]]
- [[State Estimation]]
- [[Sensor Fusion]]
- [[SLAM]]
- [[IMU]]
- [[Robotics]]

---

## 📚 Further Reading

- [Wikipedia: Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
- [Probabilistic Robotics (Book)](https://www.probabilistic-robotics.org/)
- [Kalman Filter Tutorial (Welch & Bishop)](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)
- [EKF in Robotics (ROS)](https://wiki.ros.org/robot_pose_ekf)
- [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)

---

## 🧠 Summary

The Extended Kalman Filter generalizes the Kalman Filter to nonlinear systems by linearizing models at each step. While not always optimal, it is a practical and widely used solution for real-world estimation problems in robotics, navigation, and sensor fusion.
