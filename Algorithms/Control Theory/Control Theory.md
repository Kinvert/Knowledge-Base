# 🎛️ Control Theory

**Control theory** is a field of engineering and mathematics focused on modeling, analyzing, and designing systems that regulate themselves or are regulated through feedback. It is foundational in robotics, aerospace, automotive systems, process engineering, and more.

---

## 🧠 Summary

- **Goal**: Design controllers that make a system behave in a desired way
- **Core idea**: Use feedback to automatically adjust system behavior
- **Applications**: Robotics, automation, vehicle dynamics, industrial processes, electronics

---

## 🛠️ Main Components

| Concept            | Description                                                                                     |
|--------------------|-------------------------------------------------------------------------------------------------|
| **Plant**           | The system being controlled (e.g. robot arm, vehicle, motor)                                    |
| **Controller**      | The algorithm or device that applies inputs to achieve desired output                           |
| **Sensor**          | Measures system outputs for feedback                                                            |
| **Actuator**        | Device that applies force, motion, or other effects to the plant                               |
| **Feedback Loop**   | Mechanism that adjusts inputs based on measured outputs                                         |

---

## 🎯 Common Types of Control

| Type                | Description                                                           | Example Use |
|---------------------|-----------------------------------------------------------------------|-------------|
| Open-loop            | No feedback, fixed input                                             | Washing machine timer |
| Closed-loop (feedback)| Adjusts input based on output                                       | Cruise control |
| PID control          | Combines proportional, integral, derivative actions                  | Motor speed control |
| State-space control   | Models system as state equations, often for multivariable systems   | Aerospace systems |
| Adaptive control      | Adjusts controller parameters in real-time                          | Robotics in changing environments |
| Optimal control       | Finds control inputs that minimize a cost function                  | Trajectory planning |

---

## ✅ Strengths

- Enables precise, automated regulation of dynamic systems
- Reduces reliance on manual adjustment
- Can handle disturbances and model inaccuracies (in feedback systems)

---

## ❌ Weaknesses

- Requires accurate system modeling for best performance
- Can become complex for nonlinear or high-order systems
- Poorly tuned controllers can cause instability

---

## ⚙️ Typical Tools and Methods

- [[PID Controller]]
- [[Kalman Filter]] (for estimation, not control directly, but often combined)
- LQR (Linear Quadratic Regulator)
- MPC (Model Predictive Control)
- Bode plots, Nyquist plots, root locus (analysis tools)

---

## 🌐 External References

- [ControlTheoryPro basics](https://controltheorypro.com/control-theory/)
- [MIT OpenCourseWare - Feedback Control](https://ocw.mit.edu/courses/mechanical-engineering/2-14-analysis-and-design-of-feedback-control-systems-fall-2004/)

---

## 🔗 Related Notes

- [[Algorithms]]
- [[PID Controller]]
- [[Kalman Filter]]
- [[State-Space Model]]
- [[Optimal Control]]
- [[Robotics]]

---
