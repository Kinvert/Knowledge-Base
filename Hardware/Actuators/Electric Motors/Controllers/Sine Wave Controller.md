# Sine Wave Controller

A Sine Wave Controller is a type of motor driver that supplies sinusoidal voltage waveforms to each phase of a motor — typically [[BLDC]] or [[PMSM]] — to achieve smooth, quiet, and efficient rotation. It contrasts with simpler trapezoidal or square-wave controllers, which produce more abrupt changes in current and torque.

Sine wave controllers are often implemented with advanced algorithms like [[FOC]] (Field Oriented Control), which mathematically resolve torque-producing and magnetizing current components for maximum efficiency.

---

## ⚙️ Overview

In a sine wave controller, the motor phases are energized with continuously varying sinusoidal current profiles, synchronized with rotor position. This leads to minimal torque ripple and reduced acoustic noise compared to trapezoidal commutation.

Precise rotor position — typically from an [[Encoder]] or Hall sensors — is necessary to generate the correct waveform timing.

---

## 🧠 Core Concepts

- **Sinusoidal Commutation**: Delivers current in a smooth waveform aligned with rotor magnetic field.
- **Torque Ripple Reduction**: Key reason for using sine wave control — ideal for smooth motion.
- **[[FOC]] Integration**: Sine wave controllers often use FOC for dynamic torque control.
- **Sensor vs. Sensorless**: High-end controllers use encoders; some low-cost options estimate rotor position via back EMF.

---

## 📊 Comparison Table

| Controller Type      | Waveform     | Torque Ripple | Noise     | Feedback Required | Complexity  |
|----------------------|--------------|----------------|-----------|-------------------|-------------|
| Trapezoidal ESC      | Trapezoidal  | High           | High      | No (can be sensorless) | Low        |
| Sine Wave Controller | Sinusoidal   | Low            | Very Low  | Yes (usually)     | Moderate    |
| [[FOC]] Controller   | Sinusoidal (Vector-based) | Very Low  | Very Low  | Yes               | High        |

---

## ✅ Strengths

- Smooth motion with minimal vibration  
- Lower audible and electrical noise  
- Better performance at low speeds  
- Ideal for robotic joints and high-precision movement

---

## ❌ Weaknesses

- Requires rotor position feedback or accurate estimation  
- More expensive and complex than trapezoidal controllers  
- Poor performance if misconfigured or if timing is off  
- Requires tuning for optimal performance

---

## 🧩 Compatible Components

- [[PMSM]]  
- [[BLDC]]  
- [[Encoder]]  
- [[FOC]]  
- [[Motor Control Algorithms]]  
- [[Inverter]]  
- [[VESC]] (can operate in sine wave mode)

---

## 🔗 Related Concepts

- [[Electric Motors]]  
- [[FOC]]  
- [[ESC]]  
- [[PWM]]  
- [[Back EMF]]  
- [[Motor Control Algorithms]]  
