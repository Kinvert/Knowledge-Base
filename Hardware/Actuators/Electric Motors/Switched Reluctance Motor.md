# Switched Reluctance Motor

A Switched Reluctance Motor (SRM) is a type of electric motor that operates by switching current through stator windings to align the rotor with the lowest magnetic reluctance path. It features a simple and robust construction with no magnets or windings on the rotor, making it ideal for high-speed, high-temperature, and harsh-environment applications.

SRMs are gaining renewed interest in EVs, industrial drives, and aerospace due to their fault tolerance and independence from rare-earth materials.

---

## ⚙️ Overview

The stator of an SRM contains concentrated windings, while the rotor is a simple laminated steel structure with salient poles but no windings or magnets. Torque is produced by energizing stator phases in a sequence that pulls the rotor toward the position of least magnetic reluctance.

Precise timing of phase switching is critical and is handled by a dedicated controller.

---

## 🧠 Core Concepts

- **Magnetic Reluctance**: Rotor moves to minimize the magnetic resistance between stator and rotor.
- **Salient Pole Rotor**: Rotor consists of protruding poles without coils or magnets.
- **Unipolar Drive**: Current flows in one direction per phase; simpler power electronics.
- **Discrete Phase Commutation**: Phases are energized in sequence to produce continuous torque.
- **Independent from Rare Earths**: No need for permanent magnets.

---

## 📊 Comparison Table

| Feature                | Switched Reluctance Motor | [[PMSM]]               | [[BLDC]]               | [[Synchronous Reluctance Motor]] |
|------------------------|---------------------------|------------------------|------------------------|----------------------------------|
| Rotor Type             | Laminated steel, no coils | Permanent magnets      | Permanent magnets      | Salient pole, anisotropic        |
| Commutation            | Electronic (discrete)     | FOC                    | FOC or trapezoidal     | FOC                              |
| Efficiency             | Moderate to high          | Very high              | High                   | High                             |
| Torque Ripple          | High                      | Low                    | Moderate               | Moderate                         |
| Noise & Vibration      | High                      | Low                    | Moderate               | Moderate                         |
| Rare Earth Use         | None                      | Required               | Required               | None                             |
| Control Complexity     | High                      | High                   | Moderate               | High                             |

---

## ✅ Strengths

- Rugged, simple rotor—ideal for extreme environments  
- No permanent magnets or rotor windings  
- High-speed capability with fault-tolerant operation  
- Efficient at certain speed/load ranges  
- Scales well for cost-sensitive applications  

---

## ❌ Weaknesses

- Requires specialized, high-speed switching controller  
- High torque ripple and acoustic noise  
- Control and modeling are more complex than traditional motors  
- Not ideal for precision motion or smooth operation without compensation  

---

## 🧩 Compatible Components

- [[SRM Controllers]] (specialized motor controller)  
- [[Current Sensors]] (for phase current monitoring)  
- [[Encoder]] or rotor position sensor  
- [[DC Bus Capacitor]] and fast-switching semiconductors  
- [[Motor Control Algorithms]] (typically tailored for SRMs)  

---

## 🔗 Related Concepts

- [[Electric Motors]]  
- [[Synchronous Reluctance Motor]]  
- [[FOC]]  
- [[Motor Controllers]]  
- [[Rotor Position Sensing]]  
- [[BLDC]]  
- [[PMSM]]  
