# PMSM

**PMSM (Permanent Magnet Synchronous Motor)** is a highly efficient and precise type of AC motor that uses permanent magnets on the rotor and requires sophisticated control techniques such as [[FOC]] (Field Oriented Control). Unlike [[BLDC]], which uses trapezoidal commutation, PMSMs are driven with sinusoidal current for smoother torque output and better control at low speeds.

PMSMs are commonly used in applications demanding high performance, efficiency, and torque density.

---

## ⚙️ Overview

PMSMs feature a rotor with embedded or surface-mounted permanent magnets and a stator similar to other AC motors. They operate synchronously with the stator’s rotating magnetic field, requiring precise rotor position tracking. Control typically involves continuous sinusoidal waveforms and closed-loop feedback, usually via encoders or resolvers.

---

## 🧠 Core Concepts

- **Synchronous Operation**: Rotor speed matches the stator field frequency.
- **Sinusoidal Commutation**: Produces smooth torque and low acoustic noise.
- **[[FOC]] (Field Oriented Control)**: Enables decoupled control of torque and flux.
- **Saliency**: Surface-mounted vs. interior magnets affect torque ripple and control.
- **Back EMF**: Sinusoidal, used for sensorless control methods.

---

## 📊 Comparison Table

| Feature              | [[BLDC]]         | [[PMSM]]              |
|----------------------|------------------|------------------------|
| Commutation          | Trapezoidal      | Sinusoidal             |
| Control              | ESC              | FOC                    |
| Feedback             | Optional         | Required (usually)     |
| Torque Smoothness    | Moderate         | Very High              |
| Efficiency           | High             | Very High              |
| Noise                | Noticeable       | Very Low               |
| Complexity           | Moderate         | High                   |
| Typical Applications | Consumer/General | Industrial/Precision   |

---

## ✅ Strengths

- Extremely high efficiency and torque density  
- Very smooth rotation with minimal torque ripple  
- Excellent low-speed performance with proper control  
- Ideal for precision control in servo applications

---

## ❌ Weaknesses

- Requires complex motor control techniques (usually [[FOC]])
- Typically needs rotor position feedback (encoder or resolver)
- Higher cost and development effort than [[BLDC]] or [[Brushed DC Motor]]

---

## 🧩 Compatible Components

- [[FOC]] (Field Oriented Control)
- [[Encoder]]
- [[Resolver]]
- [[Inverter]]
- [[Motor Control Algorithms]]
- [[Sensorless Control]] (in some cases)
- [[VESC]] (can support PMSM)

---

## 🔗 Related Concepts

- [[BLDC]]
- [[Electric Motors]]
- [[FOC]]
- [[Torque Control]]
- [[Motor Control Algorithms]]
- [[Encoder]]


## Links

- https://www.youtube.com/watch?v=cmRGhoF_ZEM
