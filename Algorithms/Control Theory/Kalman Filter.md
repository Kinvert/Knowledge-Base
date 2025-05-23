---
title: Kalman Filter
tags: [algorithms, estimation, control-theory, sensor-fusion, robotics, filtering]
aliases: [Kalman Filtering, Linear Quadratic Estimation, KF]
---

# 📈 Kalman Filter

## 🧭 Overview

The **Kalman Filter** is an efficient recursive algorithm used for estimating the state of a dynamic system from a series of noisy measurements. It is widely used in control systems, robotics, navigation, sensor fusion, and signal processing. The Kalman Filter provides optimal estimates (in the least-squares sense) for linear systems with Gaussian noise.

---

## 🛠️ Key Features

- **Recursive Estimation**: Updates estimates as new measurements arrive, without needing to store all past data.
- **Predict-Update Cycle**: Alternates between predicting the next state and updating the estimate with new measurements.
- **Optimal for Linear Gaussian Systems**: Minimizes mean squared error for linear systems with Gaussian noise.
- **Computationally Efficient**: Suitable for real-time applications.
- **Extensible**: Forms the basis for more advanced filters (e.g., Extended Kalman Filter, Unscented Kalman Filter).

---

## 📦 Common Use Cases

- **Sensor Fusion**: Combining data from IMUs, GPS, cameras, and other sensors.
- **Robotics**: Localization, mapping (SLAM), and motion tracking.
- **Navigation**: Aircraft, spacecraft, and autonomous vehicles.
- **Signal Processing**: Noise reduction and smoothing.
- **Finance**: Time series prediction and filtering.

---

## 🧩 How It Works

1. **Prediction Step**:  
   - Predict the next state and its uncertainty based on the system model.

2. **Update Step**:  
   - Incorporate new measurements to correct the prediction, reducing uncertainty.

3. **Mathematical Model**:  
   - Assumes a linear system with process noise and measurement noise, both Gaussian.

---

## 🆚 Kalman Filter vs. Other Filters

| Filter Type             | System Type      | Noise Assumption | Complexity   | Use Cases                  |
|-------------------------|------------------|------------------|--------------|----------------------------|
| **Kalman Filter**       | Linear           | Gaussian         | Low          | Robotics, navigation       |
| **Extended Kalman Filter (EKF)** | Nonlinear | Gaussian         | Moderate     | SLAM, sensor fusion        |
| **Unscented Kalman Filter (UKF)** | Nonlinear | Gaussian         | Higher       | Nonlinear estimation       |
| **Particle Filter**     | Nonlinear        | Any              | High         | Complex, non-Gaussian      |
| **Moving Average**      | Any              | Any              | Very Low     | Signal smoothing           |

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Optimal for Linear Gaussian Systems**
- **Real-Time Capable**
- **Widely Used and Well-Studied**
- **Foundation for Advanced Filters**

### ❌ Disadvantages
- **Assumes Linearity and Gaussian Noise**
- **Requires Accurate System and Noise Models**
- **Can Diverge if Models Are Poor**

---

## 🔗 Related Topics

- [[Sensor Fusion]]
- [[SLAM]]
- [[PID Control]]
- [[Extended Kalman Filter]]
- [[Unscented Kalman Filter]]
- [[State Estimation]]
- [[IMU]]
- [[Robotics]]

---

## 📚 Further Reading

- [Wikipedia: Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter)
- [Kalman Filter Tutorial (Greg Welch & Gary Bishop)](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)
- [Probabilistic Robotics (Book)](https://www.probabilistic-robotics.org/)
- [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
- [Sensor Fusion and Kalman Filtering (YouTube)](https://www.youtube.com/watch?v=2pzxEmh0gYw)

---

## 🧠 Summary

The Kalman Filter is a cornerstone algorithm for state estimation in dynamic systems, especially where measurements are noisy or incomplete. Its efficiency and optimality for linear Gaussian systems make it a go-to solution in robotics, navigation, and sensor fusion.
