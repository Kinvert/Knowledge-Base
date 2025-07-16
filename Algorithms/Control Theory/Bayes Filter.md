# Bayes Filter

The **Bayes Filter** is a probabilistic algorithm used for estimating the state of a dynamic system in the presence of uncertainty. It is widely used in **robotics**, **sensor fusion**, and **SLAM** to continuously update a robot’s belief about its position or other hidden variables based on control inputs and sensor observations.

Bayes Filters provide the foundation for many common state estimation techniques such as the Kalman Filter, Particle Filter, and Histogram Filter.

---

## 🧠 Overview

Bayes Filters are recursive estimators. At each timestep, they:

1. **Predict** the new state using a motion model and previous belief.  
2. **Update** the belief based on sensor measurements and an observation model.

The algorithm applies **Bayes’ Rule** to compute the posterior belief distribution over the system’s state.

---

## 🔩 Core Concepts

- **Belief (`bel(x)`)**: Probability distribution over the possible states.  
- **Prediction Step**: Uses control input and motion model to update belief.  
- **Correction Step**: Incorporates sensor data to refine belief using Bayes’ Rule.  
- **Markov Assumption**: Future state depends only on the current state and action.  
- **Recursive Estimation**: Continuously refines belief with each timestep.

---

## 🧪 Use Cases

- Mobile robot localization  
- SLAM systems (e.g., [[ORB-SLAM]], [[GMapping]])  
- Sensor fusion (e.g., GPS + IMU)  
- Object tracking in vision systems  
- Estimating dynamic environments or human motion

---

## 📊 Comparison Table

| Filter Type         | Based On      | Assumptions                   | Use Case                         | Notes                                 |
|---------------------|---------------|-------------------------------|----------------------------------|----------------------------------------|
| Bayes Filter         | Bayes’ Rule   | Probabilistic                 | General state estimation         | Foundation for many filters            |
| [[Kalman Filter]]    | Linear-Gaussian | Gaussian noise, linear models | Continuous state estimation      | Efficient for linear systems           |
| [[Extended Kalman Filter]] | Nonlinear models | Gaussian noise, differentiable models | Nonlinear robotics models        | Linearizes around current estimate     |
| [[Particle Filter]]  | Monte Carlo   | Non-parametric, nonlinear     | Localization, tracking           | Works with arbitrary distributions     |
| [[Histogram Filter]] | Discrete Bayes | Grid-based state space        | Low-dimensional discrete systems | Used in simpler mobile robot tasks     |

---

## ✅ Pros

- General probabilistic framework  
- Handles noisy and partial observations  
- Flexible: can be adapted to many problem domains  
- Foundation for more specific algorithms (e.g., EKF, PF)

---

## ❌ Cons

- Computationally expensive in raw form  
- Requires accurate motion and observation models  
- Hard to scale to high-dimensional continuous states directly

---

## 🔗 Related Concepts

- [[Kalman Filter]]  
- [[Extended Kalman Filter]]  
- [[Unscented Kalman Filter]]  
- [[Particle Filter]]  
- [[SLAM]]  
- [[Sensor Fusion]]  
- [[Markov Decision Process]]  
- [[Gaussian Distribution]]

---

## 📚 Further Reading

- [Probabilistic Robotics (Thrun, Burgard, Fox)]  
- [Bayes Filter Tutorial (Stanford CS)](https://web.stanford.edu/class/cs223b/lectures/Stanford_CS223B_Lecture05.pdf)  
- [Bayes Filters Explained (Python code)](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)

---
