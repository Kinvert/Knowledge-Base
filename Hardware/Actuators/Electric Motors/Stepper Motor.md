# Stepper Motor

Stepper motors are electromechanical actuators that move in discrete steps rather than continuous rotation. Their ability to hold position without feedback makes them ideal for open-loop control in applications requiring precise positioning, repeatability, and simplicity — such as CNC machines, 3D printers, and pick-and-place systems.

Unlike continuous-rotation motors, stepper motors increment in fixed angular steps, typically 1.8° per step (200 steps/rev), though microstepping can improve resolution and smoothness.

---

## ⚙️ Overview

Stepper motors divide a full rotation into a fixed number of steps, making them inherently suitable for digital control. Each input pulse advances the shaft by a known angle, allowing control systems to move to exact positions without requiring encoders in many cases.

They are especially well-suited for tasks where speed is moderate, torque is predictable, and the motion path is known in advance.

---

## 🧠 Core Concepts

- **Step Angle**: The angular distance per pulse, often 1.8° or 0.9°.
- **Microstepping**: Intermediate current levels create finer positioning steps.
- **Open-loop Control**: Operates without position feedback; low complexity.
- **Holding Torque**: Ability to resist external force when powered but not moving.
- **Resonance**: Mechanical vibration at certain frequencies, may require damping.

---

## 📊 Comparison Table

| Feature              | [[Stepper Motor]] | [[Servo Motor]]          | [[BLDC]]              |
|----------------------|-------------------|---------------------------|------------------------|
| Control              | Open-loop          | Closed-loop               | Sensorless or closed   |
| Feedback             | Optional           | Required                  | Optional or required   |
| Position Accuracy    | High (in ideal case) | Very High              | High                   |
| Efficiency           | Low                | High                      | High                   |
| Torque at Speed      | Drops quickly      | Consistent                | Consistent             |
| Holding Torque       | High               | Depends on tuning         | Low (unless geared)    |
| Use Case             | CNC, 3D printing   | Robotic joints, gimbals   | Drones, drivetrains    |

---

## ✅ Strengths

- Precise and repeatable movement  
- Simple, low-cost control electronics  
- High holding torque at standstill  
- No need for feedback in many applications  
- Wide availability of drivers and documentation

---

## ❌ Weaknesses

- Loses torque quickly at high speeds  
- Can skip steps under load without feedback  
- Inefficient at standstill (constant power draw)  
- Prone to vibration and resonance  
- Limited torque per size compared to alternatives

---

## 🧩 Compatible Components

- [[Microstepping Driver]] (e.g. TMC2209, A4988)  
- [[PID Controller]] (for closed-loop variants)  
- [[Encoder]] (for hybrid servo-stepper systems)  
- [[Motion Planning Algorithms]]  
- [[Power Supply]] (with sufficient current for peak draw)

---

## 🔗 Related Concepts

- [[Electric Motors]]  
- [[Servo Motor]]  
- [[BLDC]]  
- [[Encoder]]  
- [[Microstepping]]  
- [[Motor Control Algorithms]]  
