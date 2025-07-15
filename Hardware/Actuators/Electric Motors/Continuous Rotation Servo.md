# Continuous Rotation Servo

A Continuous Rotation Servo is a type of servo motor that has been modified (or manufactured) to rotate continuously in either direction, rather than being limited to a specific angular range like a standard positional [[Servo Motor]]. Instead of controlling position, the input signal determines rotation direction and speed.

These motors are commonly used in low-cost robotics, drive systems for small wheeled robots, and remote-controlled vehicles where simplicity and ease of control are prioritized over precision.

---

## ⚙️ Overview

Continuous rotation servos retain the same form factor and 3-wire interface (power, ground, signal) as standard hobby servos. However, their internal feedback mechanism has been removed or altered so the control signal adjusts rotational speed and direction instead of absolute position.

They are controlled by sending a PWM signal:
- Neutral signal (typically ~1.5 ms pulse) = no motion
- Longer pulse (>1.5 ms) = clockwise rotation
- Shorter pulse (<1.5 ms) = counter-clockwise rotation

---

## 🧠 Core Concepts

- **PWM Control**: Pulse width determines speed and direction, not angle.
- **Modified Feedback**: Internal potentiometer and stop are removed or bypassed.
- **Ease of Use**: Easily controlled from microcontrollers or [[RC Receiver]]s.
- **Low-Cost Mobility**: Ideal for quick robot prototyping and hobbyist bots.
- **Limited Precision**: No absolute positioning—acts more like a basic DC motor with simple control.

---

## 📊 Comparison Table

| Feature                   | Continuous Rotation Servo | Standard Servo         | [[Stepper Motor]]       | [[BLDC]]              |
|---------------------------|---------------------------|------------------------|--------------------------|------------------------|
| Rotation Type             | Unlimited                 | Limited (~180° or 270°)| Unlimited                | Unlimited              |
| Control Type              | PWM (speed/direction)     | PWM (position)         | Step/direction pulses    | PWM + commutation      |
| Position Feedback         | None                      | Internal potentiometer | External (optional)      | Encoder (optional)     |
| Precision                 | Low                       | Medium                 | High                     | High                   |
| Typical Use               | Mobile bots, toys         | Arms, gimbals          | CNC, 3D printers         | Drones, robotics        |

---

## ✅ Strengths

- Simple interface with standard 3-wire PWM  
- No external driver required  
- Inexpensive and compact  
- Ideal for beginner robotics and mobile platforms  

---

## ❌ Weaknesses

- No feedback or positional awareness  
- Inconsistent speed control across units  
- Limited torque and performance for demanding applications  
- No braking or fine motion control  

---

## 🧩 Compatible Components

- [[PWM]] signal generators (e.g. Arduino `servo.writeMicroseconds`)  
- [[Microcontrollers]]  
- [[RC Receivers]]  
- [[Robot Chassis Kits]] (for educational platforms)  

---

## 🔗 Related Concepts

- [[Servo Motor]]  
- [[PWM]]  
- [[DC Motor]]  
- [[Motor Controllers]]  
- [[Robot Mobility Systems]]  
