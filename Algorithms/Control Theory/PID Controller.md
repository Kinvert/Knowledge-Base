# PID Controller

A **PID Controller** is a fundamental feedback control algorithm used across robotics, automation, and embedded systems. It attempts to minimize the error between a desired setpoint and a measured process variable by adjusting control inputs. The acronym **PID** stands for **Proportional, Integral, and Derivative**, the three terms used to compute the output.

PID control is simple yet effective and forms the basis of many modern control systems in robotics (motor speed, joint angle, temperature), drones, automotive systems, and more.

---

## ⚙️ How It Works

A PID controller calculates the control signal as:

`u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt`

Where:  
- `e(t)` = error at time `t` = `setpoint - measured_value`  
- `Kp` = proportional gain  
- `Ki` = integral gain  
- `Kd` = derivative gain  

Each term influences system behavior differently.

---

## 🧠 Core Concepts

- **Proportional (P)**: Correction proportional to the error. Increases responsiveness.
- **Integral (I)**: Accounts for accumulated past errors. Eliminates steady-state offset.
- **Derivative (D)**: Predicts future error based on rate of change. Damps oscillations.
- **Tuning**: Adjusting Kp, Ki, Kd to balance responsiveness, stability, and accuracy.
- **Ziegler–Nichols**: A common empirical method to tune PID parameters.

---

## 🧪 Use Cases in Robotics

- Controlling motor RPM or joint angles  
- Balancing two-wheeled robots  
- Inverse Kinematics control loops for manipulators  
- Trajectory following in drones and mobile robots  
- Temperature Control in 3D printers  
- Gimbal stabilization systems

---

## 📊 Comparison with Related Controllers

| Controller Type | Integrates Past Errors | Predicts Future | Simple to Tune | Robust to Noise | Common Use Case             |
|-----------------|------------------------|------------------|----------------|------------------|------------------------------|
| P               | ❌                     | ❌               | ✅              | ✅                | Basic motor control          |
| PI              | ✅                     | ❌               | ✅              | ✅                | Temperature control          |
| PD              | ❌                     | ✅               | ✅              | ⚠️ (Sensitive)   | Drone orientation control    |
| PID             | ✅                     | ✅               | ⚠️ (Complex)    | ⚠️ (Depends on D) | Mobile robot path following  |
| LQR             | ✅ (via model)         | ✅               | ❌              | ✅                | Optimal control in robotics  |
| MPC             | ✅                     | ✅ (future horizon) | ❌              | ✅                | Advanced predictive control  |

---

## ✅ Pros

- Simple, well-understood, and widely supported  
- Real-time capable on low-power embedded hardware  
- General-purpose — works in many domains  
- Easily implementable in microcontrollers and PLCs

---

## ❌ Cons

- Requires tuning, often trial-and-error  
- Not optimal or adaptive in dynamic environments  
- Poor handling of large delays or non-linear systems  
- Derivative term is sensitive to noise

---

## 🔧 Tuning Strategies

- **Ziegler–Nichols** method  
- **Manual tuning** using trial/error  
- **Auto-tuners** in simulation or on real hardware  
- **Software-in-the-loop (SIL)** validation via [[Simulation Environments]]

---

## 🔗 Related Concepts

- [[Control Theory]]  
- [[Model Predictive Control]]  
- [[State Estimation]]  
- [[Kalman Filter]]  
- [[Embedded System]]  
- [[ROS2 Parameters]]

---

## 📚 Further Reading

- [PID Theory Explained – Control Guru](https://controlguru.com)  
- [Ziegler-Nichols Method Overview – Wikipedia](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)  
- [PID Control – MathWorks](https://www.mathworks.com/help/control/ug/pid-controller.html)

---
