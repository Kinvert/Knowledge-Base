# VFD (Variable Frequency Drive)

A Variable Frequency Drive (VFD) is an electronic device that controls the speed and torque of an AC motor by varying the frequency and voltage of its power supply. VFDs are essential in modern motor control systems for improving efficiency, reducing wear, and enabling dynamic operation in a wide range of industrial and robotic applications.

They are commonly used with motors like [[Induction Motor]], [[Synchronous Motor]], and even advanced types like [[PMSM]] when high-performance control is needed.

---

## ⚙️ Overview

A VFD converts fixed-frequency AC from the mains into DC using a rectifier stage, then uses an inverter (typically using [[PWM]]-based switching) to synthesize variable-frequency AC for motor control.

This allows precise control over:
- Speed (by adjusting frequency)
- Torque (via voltage/frequency ratio)
- Direction (by changing phase sequence)
- Soft start/stop (for reduced mechanical stress)

---

## 🧠 Core Concepts

- **Rectifier**: Converts AC to DC  
- **DC Link/Bus**: Filters and stabilizes intermediate DC voltage  
- **Inverter**: Reconstructs AC output with controlled frequency and voltage  
- **PWM (Pulse Width Modulation)**: Used in inverter stage for fine control  
- **Vector Control / [[FOC]]**: For dynamic torque/speed control in advanced applications  
- **Braking**: Regenerative or resistive braking to decelerate motors efficiently  

---

## 📊 Comparison Table

| Feature                   | VFD                     | [[ESC]]                  | [[Motor Controller]]     | [[Soft Starter]]            |
|---------------------------|-------------------------|--------------------------|---------------------------|-----------------------------|
| Motor Compatibility       | AC motors (Induction, Synchronous) | [[BLDC]], [[Brushed DC Motor]] | DC, stepper, servo        | Induction Motors            |
| Frequency Control         | Yes                     | No (voltage/current only)| Sometimes                 | No                          |
| Direction Control         | Yes                     | Yes                      | Yes                       | No                          |
| Soft Start/Stop           | Yes                     | Yes                      | Yes                       | Yes                         |
| Advanced Control (FOC)    | Supported               | Supported (BLDC/PMSM)    | Sometimes                 | No                          |
| Cost                      | Moderate                | Low to Moderate          | Varies                    | Low                         |

---

## ✅ Strengths

- Smooth and precise speed control for AC motors  
- Energy savings in variable load applications  
- Reduced mechanical stress on equipment  
- Enables complex drive profiles and programmable logic  
- Built-in protections: overcurrent, overvoltage, undervoltage, etc.  

---

## ❌ Weaknesses

- Can introduce harmonics and EMI without proper filtering  
- May require shielding and grounding considerations  
- Complex to configure for dynamic torque applications  
- Not suitable for all motor types without tuning  

---

## 🧩 Compatible Components

- [[Induction Motor]]  
- [[Synchronous Motor]]  
- [[PMSM]] (with sensorless or [[FOC]])  
- [[Current Sensor]]  
- [[Braking Resistors]]  
- [[Line Reactors]] or EMI filters for harmonic mitigation  

---

## 🔗 Related Concepts

- [[FOC]] (Field Oriented Control)  
- [[ESC]]  
- [[Motor Controllers]]  
- [[Induction Motor]]  
- [[PMSM]]  
- [[Power Electronics]]  
- [[PWM]]  
- [[Braking Techniques]]  
