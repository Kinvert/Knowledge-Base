# FOC

FOC (Field Oriented Control) is an advanced motor control technique used to regulate the torque and speed of AC motors such as [[PMSM]] and [[BLDC]]. By aligning the stator current vector with the rotor magnetic field, FOC enables efficient and smooth operation with high dynamic performance.

Unlike simpler methods (like trapezoidal commutation), FOC uses mathematical transforms and feedback loops to achieve decoupled control of torque and flux — a fundamental feature for high-performance robotics and industrial systems.

---

## ⚙️ Overview

FOC transforms motor control from a three-phase system to a rotating reference frame, allowing for independent control of torque-producing and magnetizing components of current. This results in more precise control over torque, higher efficiency, and smoother rotation, even at low speeds.

Implementation typically requires rotor position feedback (via encoder, resolver, or back EMF estimation) and a microcontroller capable of fast real-time computation.

---

## 🧠 Core Concepts

- **Clarke Transform**: Converts 3-phase currents to 2-axis stationary frame (α/β).
- **Park Transform**: Converts stationary frame to rotor-aligned rotating frame (d/q).
- **Decoupled Torque Control**: Enables independent regulation of flux (`id`) and torque (`iq`).
- **Inverse Transforms**: Convert back to 3-phase voltages to apply to the motor.
- **PI Controllers**: Maintain desired `id` and `iq` values.
- **Space Vector Modulation (SVM)**: Often used to drive the inverter with minimal harmonic distortion.

---

## 📊 Comparison Table

| Control Method         | Motor Types     | Feedback Required | Torque Smoothness | Efficiency | Complexity |
|------------------------|------------------|--------------------|--------------------|------------|------------|
| Trapezoidal Commutation| BLDC              | Optional           | Low                | Moderate   | Low        |
| Sine Wave Commutation  | BLDC, PMSM        | Usually            | Medium-High        | High       | Moderate   |
| Field Oriented Control | PMSM, BLDC        | Required           | Very High          | Very High  | High       |

---

## ✅ Strengths

- High torque output even at low speeds  
- Excellent smoothness and low acoustic noise  
- Maximizes efficiency in most operating conditions  
- Ideal for precision robotics and servo systems  

---

## ❌ Weaknesses

- Requires fast control loops and significant computational power  
- Needs accurate rotor position sensing or estimation  
- Complex tuning and parameterization  
- Can become unstable if feedback or current sensing is faulty  

---

## 🧩 Compatible Components

- [[PMSM]]  
- [[BLDC]]  
- [[Encoder]]  
- [[Resolver]]  
- [[ESC]] (must support FOC)  
- [[VESC]]  
- [[Current Sensor]]  
- [[Motor Control Algorithms]]  

---

## 🔗 Related Concepts

- [[Electric Motors]]  
- [[ESC]]  
- [[Sine Wave Controller]]  
- [[Sensorless Control]]  
- [[PID Controller]]  
- [[Space Vector Modulation]]  
- [[Torque Control]]

## Links

- https://www.youtube.com/watch?v=e0sQnVmE7DU
