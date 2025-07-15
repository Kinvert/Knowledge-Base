# BLDC

[[BLDC]] (Brushless DC Motor) is a type of electric motor that uses electronic commutation instead of brushes to control current flow through its windings. Known for high efficiency, reliability, and power-to-weight ratio, BLDC motors are widely used in robotics, drones, electric vehicles, and industrial automation.

Their low maintenance requirements and precise controllability make them ideal for applications where performance and durability are critical.

---

## ⚙️ Overview

BLDC motors consist of a rotating permanent magnet rotor and a stationary stator with coils. Electronic speed controllers (ESCs) use position information (from Hall sensors or back EMF) to switch current through the windings at the right time, replacing the mechanical brushes in traditional DC motors.

They can be built in **inrunner** or **outrunner** configurations, impacting torque and speed characteristics.

---

## 🧠 Core Concepts

- **Electronic Commutation**: Achieved using an ESC, which switches phases based on rotor position.
- **Hall Sensors**: Embedded in the stator to detect rotor position.
- **Sensorless Control**: Uses back EMF instead of Hall sensors for position detection.
- **Kv Rating**: RPM per volt; used to characterize speed performance.
- **Torque Constant (Kt)**: Torque output per amp of current.
- **Field Weakening**: Advanced method to increase top-end RPM at reduced torque.

---

## 📊 Comparison to Other Motors

| Motor Type         | Control Method   | Feedback | Efficiency | Typical Use Cases                     |
|--------------------|------------------|----------|------------|---------------------------------------|
| [[Brushed DC Motor]] | Mechanical       | Optional | Moderate   | Toys, hobby bots                      |
| [[BLDC]]             | Electronic (ESC) | Yes/No   | High       | Drones, EVs, mobile robots            |
| [[PMSM]]             | Electronic (FOC) | Required | Very High  | High-end EVs, gimbals, precision apps |
| [[Stepper Motor]]    | Open-loop        | No       | Low-Mod    | CNC, 3D printers                      |
| [[Servo Motor]]      | Closed-loop      | Yes      | High       | Arms, legs, precise robotic joints    |

---

## 🧪 Use Cases

- Drones: Lightweight and efficient propulsion
- Electric Skateboards: With VESC-based control
- Mobile Robots: BLDC hub motors
- 3D Printers: Occasionally used for extruders or gantry movement
- RC Cars and Planes: Core motor type for hobbyists
- Prosthetics: Silent, smooth motion

---

## ✅ Strengths

- No brushes → Low maintenance and long lifespan
- High torque-to-weight ratio
- Excellent efficiency, even at small sizes
- Capable of very high speeds
- Smooth rotation (especially with [[FOC]])

---

## ❌ Weaknesses

- Requires an ESC or driver circuit
- Sensorless startup can be unreliable under load
- EMI concerns at high switching frequencies
- Can be more expensive than simple brushed motors

---

## 🧩 Compatible Items

- [[ESC]] (Electronic Speed Controller)
- [[Hall Effect Sensors]]
- [[FOC]] (Field Oriented Control)
- [[VESC]] (Open source BLDC controller)
- [[Quadrature Encoder]]
- [[Motor Control Algorithms]]
- [[LiPo Battery]] (for high discharge rates)

---

## 🔗 Related Concepts

- [[Electric Motors]]
- [[PMSM]]
- [[FOC]] (Field Oriented Control)
- [[ESC]]
- [[VESC]]
- [[PWM]] (Pulse Width Modulation)
- [[Back EMF]]
- [[Sensorless Control]]
- [[Motor Control Algorithms]]
- [[Magnetic Fields and Motors]]

---

## 🛠️ Developer Tools

- [[VESC Tool]] for motor tuning and telemetry
- `Arduino` with SimpleFOC library
- `STM32` and `TI C2000` series for high-performance control
- `ROS` motor drivers (e.g., `ros_control`)
- `BLHeli` / `SimonK` firmware for drones
- `SimpleFOC` for brushless motor experimentation

---

## 🌐 External Resources

- [VESC Project](https://vesc-project.com/)
- [SimpleFOC Documentation](https://docs.simplefoc.com/)
- [TI BLDC Motor Control Design Guide](https://www.ti.com/motor-drivers/)
- [ODrive Robotics](https://odriverobotics.com/)
- [Pololu BLDC Overview](https://www.pololu.com)

---

## 📚 Further Reading

- "Electric Motors and Drives" by Hughes & Drury
- [SimpleFOC Blog](https://www.simplefoc.com)
- [[Motor Control Algorithms]]
- [[PMSM]]
- [[FOC]]
