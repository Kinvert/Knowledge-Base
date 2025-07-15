# H-Bridge

An H-Bridge is a fundamental electronic circuit used to control the direction of current through a load, typically a DC motor. It enables bidirectional control—allowing motors to spin forward or reverse—by selectively switching high and low voltage paths using transistors or MOSFETs arranged in an “H” configuration.

H-Bridges are commonly used in robotics, motor drivers, and power electronics, especially with [[Brushed DC Motor]]s and [[Continuous Rotation Servo]]s modified for external motor control.

---

## ⚙️ Overview

A basic H-Bridge consists of four switching elements (usually transistors or MOSFETs) configured in a manner that forms the shape of an "H":
- The motor is placed in the center (the crossbar of the "H")
- High-side and low-side switches on each leg allow polarity reversal

Control is achieved by turning on a complementary pair of switches:
- High-left + Low-right → forward
- High-right + Low-left → reverse
- Both lows or both highs off → brake
- All off → coast

Advanced H-Bridge ICs may include protections like shoot-through prevention, current sensing, and thermal shutdown.

---

## 🧠 Core Concepts

- **Bidirectional Motor Control**: Enables motors to spin in both directions  
- **Transistor Switching**: Uses MOSFETs, BJTs, or IGBTs for efficient power control  
- **PWM Speed Control**: Combines with [[PWM]] to regulate speed  
- **Braking and Coasting Modes**: Some implementations support dynamic braking  
- **Shoot-Through Prevention**: Ensures opposite-side switches don’t short the power supply  

---

## 📊 Comparison Table

| Feature                | H-Bridge             | Half-Bridge            | [[ESC]]                | [[Motor Driver ICs]]    |
|------------------------|----------------------|-------------------------|------------------------|-------------------------|
| Direction Control      | Yes (forward/reverse)| No (1 direction)        | Yes (with commutation)| Depends on IC           |
| Speed Control          | Yes (via PWM)        | Yes (1 direction)       | Yes                   | Yes                     |
| Complexity             | Low to moderate      | Very low                | High                  | Low to moderate         |
| Motor Compatibility    | DC, Servo, Stepper   | DC only                 | [[BLDC]], brushed      | Varies by chip          |
| External Components    | Needed               | Minimal                 | Fewer (all-in-one)     | Minimal                 |

---

## ✅ Strengths

- Enables full directional control with speed modulation  
- Inexpensive and widely available  
- Easily integrated with [[Microcontrollers]] and [[PWM]]  
- Scalable from small hobby motors to industrial drives  

---

## ❌ Weaknesses

- Must avoid shoot-through conditions in discrete designs  
- Requires care in switch timing and heat management  
- Limited by switching element ratings (voltage, current)  
- Brushed motors wear out faster when driven aggressively  

---

## 🧩 Compatible Components

- [[Brushed DC Motor]]  
- [[Continuous Rotation Servo]] (if modified to be externally controlled)  
- [[Microcontrollers]] (e.g., Arduino, STM32)  
- [[Motor Controllers]]  
- [[PWM]] sources  

---

## 🔗 Related Concepts

- [[PWM]]  
- [[Motor Driver ICs]]  
- [[ESC]]  
- [[MOSFET]]  
- [[Brushed DC Motor]]  
- [[Motor Controllers]]  
