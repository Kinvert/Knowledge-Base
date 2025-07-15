# Brushed DC Motor

Brushed DC motors are simple and widely-used electric motors that operate using mechanical commutation via carbon brushes and a commutator. They are known for their ease of use, low cost, and compatibility with direct voltage sources, making them a common choice in hobby robotics, small actuators, and early automation systems.

Though largely supplanted by [[BLDC]] and [[PMSM]] motors in advanced applications, brushed motors remain relevant in systems where simplicity, cost, or legacy hardware are priorities.

---

## ⚙️ Overview

A brushed DC motor consists of:

- A rotor (armature) with windings
- A stator with permanent magnets or electromagnets
- Brushes that contact a rotating commutator to switch current direction in the rotor

This internal switching maintains torque in the same direction and allows continuous rotation. Speed is typically controlled by adjusting the supply voltage or using PWM.

---

## 🧠 Core Concepts

- **Mechanical Commutation**: Brushes and commutator physically reverse current to maintain torque direction.
- **PWM Speed Control**: Varying duty cycle to adjust average voltage and speed.
- **Brushed vs. Brushless**: Brushed motors are simpler but wear out faster due to mechanical contact.
- **Voltage-Torque Relationship**: Output torque is proportional to current; speed is proportional to voltage.

---

## 📊 Comparison Table

| Feature               | Brushed DC Motor | [[BLDC]]             | [[PMSM]]              | [[Stepper Motor]]     |
|------------------------|------------------|------------------------|------------------------|------------------------|
| Commutation            | Mechanical        | Electronic             | Electronic (FOC)       | Open-loop (step pulses)|
| Feedback Required      | No                | Optional/Required      | Required               | No                     |
| Efficiency             | Moderate          | High                   | Very High              | Low-Moderate           |
| Lifespan               | Limited (brush wear) | Long                 | Long                   | Moderate               |
| Complexity             | Very Low          | Moderate               | High                   | Low                    |
| Control                | Voltage or PWM    | ESC                    | [[FOC]]                | Stepper Driver         |

---

## ✅ Strengths

- Incredibly simple to operate and control  
- Low cost and readily available  
- Can run directly off DC power  
- Easy to prototype with  

---

## ❌ Weaknesses

- Brushes wear out → maintenance and lifespan concerns  
- Lower efficiency than modern alternatives  
- Generates electrical noise and EMI due to arcing  
- Noisy and less smooth torque delivery  
- Poor performance at high speeds under load  

---

## 🧩 Compatible Components

- [[Motor Driver ICs]] (e.g. L298N, DRV8833)  
- [[PWM]] controller  
- [[Encoder]] (if closed-loop feedback is needed)  
- [[Current Sensor]] (for protection or torque control)  
- [[Battery]] or DC power supply  

---

## 🔗 Related Concepts

- [[Electric Motors]]  
- [[BLDC]]  
- [[ESC]]  
- [[FOC]]  
- [[PWM]]  
- [[Motor Driver ICs]]  
