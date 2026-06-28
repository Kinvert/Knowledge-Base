# CoppeliaSim

`CoppeliaSim` (formerly V-REP) is a robotics simulation framework with scene-based robot construction, multi-physics support, and scripting-oriented control hooks. It is useful for UGV work when you need custom mechanics plus tight control over simulation internals.

---

## Core positioning

The official feature list explicitly calls out multiple physics engines, scripting layers, remote control, and integrated planning and sensor interfaces. This makes it good for robot-level experimentation where you want to model actuators/sensors explicitly and keep the environment inspectable.[^coppelia-features]

---

## Why it fits UGV work

- Good for custom wheeled chassis and actuator-level validation.
- Strong for combining ground vehicle motion with additional robotic payloads (manipulators, docks, sensing rigs).
- Useful when you need a unified environment for control, planning, and sensor pipelines.

---

## Comparison chart (control-first UGV simulators)

| Axis | CoppeliaSim | Gazebo Garden | Webots | CARLA | AirSim | LGSVL/SVL |
|---|---|---|---|---|---|---|
| Script/control integration | Very high | High | High | Medium | Medium | Medium |
| Physics stack diversity | High (multiple engines) | Plugin/modular | Moderate | Moderate | Engine-tied | Engine/plugin |
| Sensor flexibility | High | High | High | AD-focused | Broad | High |
| UGV mission scene breadth | User-defined | user-defined | user-defined | AD scene-first | scenario-first | AD scene-first |
| Best first use | custom robotic systems | ROS robots | rapid teaching/prototyping | urban driving stack testing | mixed aero+ground | AD stack validation |

---

## Pros and cons

### Strengths
- Highly customizable model graph and scripting architecture.
- Strong scene introspection and per-sensor/per-actuator control.
- Suitable for combining UGV and manipulation in one environment.

### Weaknesses
- Not as domain-specific as CARLA/LGSVL for urban-road certification.
- Model quality depends heavily on your scene-building discipline.
- High-fidelity visual realism can lag behind engines like CARLA or AirSim in some setups.

---

## Practical notes for UGV pipelines

- Start with a minimal physical model (frame, wheels, suspension placeholders, IMU, odom).
- If ROS2 is in your deployment loop, ensure bridge nodes are in place before reward design.
- Use scripted scenarios for deterministic RL resets.

---

## Related notes

- [[Gazebo]]
- [[Gazebo Garden]]
- [[Webots]]
- [[CARLA]]
- [[AirSim]]
- [[ROS2]]

---

## External links

- https://manual.coppeliarobotics.com/en/coppeliaSimFeatures.htm
- https://www.coppeliarobotics.com/coppeliaSimSpecifications.pdf
- https://www.coppeliarobotics.com/coppeliaSimArchitecture.pdf
- https://www.coppeliarobotics.com/

[^coppelia-features]: CoppeliaSim features documentation lists remote control, multiple physics engines, and embedded script-based composition across actuators/sensors.

