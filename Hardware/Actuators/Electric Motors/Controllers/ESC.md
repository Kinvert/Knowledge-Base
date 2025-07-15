# ESC

ESC (Electronic Speed Controller) is a device that electronically regulates the speed, direction, and braking of an electric motor. It acts as the interface between the power source and the motor, interpreting control signals and switching power transistors to drive the motor accordingly.

ESCs are essential for controlling [[BLDC]] and [[Brushed DC Motor]] types in robotics, drones, and automation. Their design varies based on the motor type, control method (e.g., trapezoidal or sinusoidal), and feedback integration.

---

## ⚙️ Overview

An ESC modulates power to a motor by switching transistors (typically MOSFETs) rapidly in patterns that generate the desired motor phase currents. In simple cases, this means PWM control. In more advanced ESCs, it includes position estimation, current regulation, and motor commutation.

ESCs for brushless motors must handle three-phase output and either estimate or measure rotor position for proper timing.

---

## 🧠 Core Concepts

- **PWM Input**: Most ESCs accept PWM signals (e.g., 1000–2000 µs pulses) to set speed.
- **3-Phase Commutation**: For brushless motors, ESCs cycle current through 3 phases to create rotation.
- **Sensorless vs. Sensored**: Some ESCs rely on back EMF; others use Hall sensors or encoders.
- **Regenerative Braking**: Some ESCs support energy recovery during deceleration.
- **Timing and Advance**: Adjusting phase advance for better efficiency or speed at different loads.

---

## 📊 Comparison Table

| ESC Type              | Motor Type        | Feedback     | Commutation Style  | Example Use Cases         |
|------------------------|-------------------|----------------|----------------------|----------------------------|
| Brushed ESC            | Brushed DC        | None           | Single-switch PWM    | Toy cars, basic robotics   |
| Trapezoidal BLDC ESC   | BLDC              | Optional       | 6-step               | Drones, RC vehicles        |
| Sine Wave ESC          | BLDC / PMSM       | Required       | Sinusoidal           | Gimbals, robotics joints   |
| FOC ESC                | PMSM / BLDC       | Required       | Vector (FOC)         | EVs, precision control     |

---

## ✅ Strengths

- Enables efficient electronic motor control  
- Widely available and standardized interfaces  
- Low-cost options for basic applications  
- High-performance variants for advanced control  

---

## ❌ Weaknesses

- Sensorless startup can be unreliable under load  
- Tuning required for advanced features (e.g. regenerative braking, timing advance)  
- Cheap ESCs may produce audible noise or torque ripple  
- Limited configuration options in off-the-shelf firmware  

---

## 🧩 Compatible Components

- [[BLDC]]  
- [[PMSM]]  
- [[FOC]]  
- [[Hall Effect Sensors]]  
- [[Encoder]]  
- [[Motor Control Algorithms]]  
- [[VESC]] (open source ESC platform)  
- [[PWM]] input from microcontrollers or flight controllers  

---

## 🔗 Related Concepts

- [[Electric Motors]]  
- [[FOC]]  
- [[Sine Wave Controller]]  
- [[Back EMF]]  
- [[Sensorless Control]]  
- [[Motor Driver ICs]]  
