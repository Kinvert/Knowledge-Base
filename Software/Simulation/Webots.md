# Webots

`Webots` is an open-source 3D robotics simulator used for robotics education and prototyping, including mobile-robot and UGV behavior research. It is a practical midpoint between lightweight teaching simulators and research-grade integration loops.

---

## Core positioning

Webots supports rapid robot creation, scripting, and model-based world design, and its official site positions it as an open-source solution used in robotics education and industry projects.[^webots-home]

---

## What makes it relevant for UGV

- Good for quickly turning a differential-drive concept into runnable closed-loop tests.
- Useful when your focus is controller correctness, sensor fusion, and algorithm iteration, not ultra-large traffic scenario generation.
- Easy to run on desktop pipelines before deciding on high-fidelity cloud-grade simulators.

---

## Comparison chart (UAV? no, UGV-focused view)

| Capability | Webots | Gazebo Garden | CARLA | AirSim | LGSVL/SVL | CoppeliaSim |
|---|---|---|---|---|---|---|
| Accessibility / onboarding | High | Medium | Medium | Medium | Medium | Medium |
| Differential-drive support | good (vehicle controllers) | explicit (DiffDrive) | scenario-first | scenario-first | scenario-first | good (scripted components) |
| Scripting flexibility | high | medium | medium | medium | medium | high |
| ROS2 compatibility | common | strong | strong | possible | strong | common |
| Best UGV use | rapid algorithm prototyping | custom mobile robot stacks | road-scene training | multi-domain DRL | AD stack bridging | custom mechanics + robots |

---

## Practical pros/cons

### Pros
- Quick experiment velocity for new models and control laws.
- Strong educational documentation and broad language bindings.
- Lower barrier than many AD-only simulators.

### Cons
- Scenario realism is usually less rich than AV-focused simulators.
- For large-scale autonomous-driving behavior studies, you’ll eventually need heavier tooling.
- Not always the best default choice for high-fidelity terrain effects out of the box.

---

## Workflow sketch

1. Build differential/wheeled base model and sensor nodes.
2. Drive using `cmd_vel`-style logic or direct wheel speed commands.
3. Add odometry + noise + timing constraints.
4. Validate controller behavior against known trajectories.

---

## Related notes

- [[Gazebo]]
- [[Gazebo Garden]]
- [[CARLA]]
- [[CoppeliaSim]]
- [[ROS2]]

---

## External links

- https://cyberbotics.com/?lang=en
- https://cyberbotics.com/doc/reference/differentialwheels?version=cyberbotics%3AR2019a
- https://www.mathworks.com/products/connections/product_detail/webots.html
- https://github.com/cyberbotics/webots

[^webots-home]: Cyberbotics homepage positions Webots as open-source 3D robot simulation software for industry and education.

