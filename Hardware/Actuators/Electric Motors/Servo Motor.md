# Servo Motor

Servo Motor refers to any motor system that operates under closed-loop feedback to control position, velocity, or torque. While the term is sometimes used ambiguously, in engineering contexts it typically describes a motor (often a [[BLDC]] or [[PMSM]]) paired with sensors (like an [[Encoder]]) and a controller capable of maintaining a desired setpoint via continuous feedback.

Servo systems are foundational to robotics, CNC, and automation, offering high precision, dynamic response, and robust control.

---

## ⚙️ Overview

A servo motor is not a distinct type of motor, but rather a **system** that includes:

- A motor (brushed or brushless)
- A position or velocity sensor (e.g., encoder)
- A controller that adjusts current or voltage to maintain a control target

The controller compares the desired value with the actual value and adjusts motor input accordingly — a classic closed-loop control system, often implemented with a [[PID Controller]].

---

## 🧠 Core Concepts

- **Closed-loop Control**: Continuously compares actual vs. desired value.
- **Position vs. Torque Control**: Servo systems can regulate angle, speed, or force.
- **Backlash and Compliance**: Important in mechanical linkages attached to servos.
- **Bandwidth**: Determines how quickly the system can respond to changes.
- **Gearing**: Often used to trade speed for torque and resolution.

---

## 📊 Comparison Table

| Feature              | [[Stepper Motor]] | [[Servo Motor]]         |
|----------------------|-------------------|--------------------------|
| Control Type         | Open-loop (mostly) | Closed-loop              |
| Accuracy             | Medium             | High                     |
| Feedback             | Optional           | Required                 |
| Torque at Speed      | Poor               | Excellent                |
| Holding Torque       | Strong             | Depends on tuning        |
| Efficiency           | Low                | High                     |
| Typical Use          | 3D printers, CNC   | Arms, gimbals, vehicles  |

---

## ✅ Strengths

- High precision and dynamic performance  
- Smooth motion, ideal for continuous rotation  
- Closed-loop feedback allows for adaptive correction  
- Scalable: from micro-sized to industrial systems

---

## ❌ Weaknesses

- Requires tuning (gain selection, stability margins)  
- More expensive than open-loop alternatives  
- Control complexity increases integration time  
- Can oscillate or overshoot if poorly tuned

---

## 🧩 Compatible Components

- [[Encoder]]
- [[FOC]] (Field Oriented Control) (when using [[PMSM]]/[[BLDC]])
- [[Motor Control Algorithms]]
- [[PID Controller]]
- [[Microcontroller]]
- [[Gearbox]] (for torque amplification and resolution)

---

## 🔗 Related Concepts

- [[Electric Motors]]
- [[BLDC]]
- [[PMSM]]
- [[Stepper Motor]]
- [[FOC]]
- [[Encoder]]
- [[PID Controller]]
- [[Actuators]]
