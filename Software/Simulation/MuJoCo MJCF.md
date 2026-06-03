# ⚙️ MuJoCo MJCF

**MJCF** (MuJoCo XML) is the native model format for [[MuJoCo]]. It describes bodies, joints, geometry, actuators, sensors, constraints, and simulation defaults for robot dynamics and control tasks.

---

## 📚 Overview

MJCF is one of the most important file formats for robotics RL because it sits close to the simulator's dynamics model. A good MJCF model is not just a visual robot. It has sane inertias, stable contacts, correct joint axes, useful actuator definitions, and collision geometry that behaves well during learning.

---

## 🧠 Core Concepts

- **`worldbody`**: Root scene tree containing the robot and environment.
- **`body`**: Rigid body frame, usually corresponding to a robot link.
- **`joint`**: Degree of freedom such as hinge, slide, ball, or free joint.
- **`geom`**: Visual or collision geometry.
- **`actuator`**: Motor, position servo, velocity servo, or muscle model.
- **`sensor`**: Joint state, IMU, force, touch, camera, or custom measurement.
- **`default`**: Reusable defaults for geoms, joints, and actuators.
- **`asset`**: Meshes, textures, and materials.

---

## 📊 Comparison Chart

| Format | Main Ecosystem | Strength | Weakness | Robotics RL Fit |
|---|---|---|---|---|
| **MJCF** | [[MuJoCo]], [[MJX]] | Excellent dynamics control | XML learning curve | Very high |
| [[URDF]] | [[ROS2]], [[MoveIt]] | Standard robot descriptions | Weak actuator/contact semantics | High as source format |
| [[XACRO]] | ROS robot generation | Parameterized URDF | Adds macro complexity | Medium-high |
| [[OpenUSD]] | [[Isaac Sim]], [[Isaac Lab]] | Rich scenes and rendering | Heavy ecosystem | Very high |
| SDF | Gazebo/Ignition | Simulation scenes | Less common in ML stacks | Medium |
| Raw Python model | Custom sims | Flexible | Hard to share and inspect | Low-medium |

---

## ✅ Pros

- Direct control over MuJoCo dynamics.
- Good support for actuators and sensors.
- Compact and readable for small robots.
- Strong fit for [[Gymnasium]] and continuous control benchmarks.
- Works with [[MJX]] for JAX-oriented workflows.

---

## ❌ Cons

- Requires careful tuning of mass, inertia, damping, friction, and contacts.
- Visual meshes often need separate simplified collision geometry.
- Conversion from [[URDF]] can lose actuator or contact intent.
- XML files can become hard to manage for large robots.
- Less integrated with ROS tooling than URDF.

---

## 🧰 Modeling Checklist

1. Verify units are meters, kilograms, seconds, and radians.
2. Check joint axes and parent-child transforms.
3. Use simple collision geometry before detailed meshes.
4. Set realistic mass and inertia for every moving body.
5. Add actuator force limits and control ranges.
6. Tune friction and damping before reward weights.
7. Run passive simulation before training.

---

## 🔧 Compatible Items

- [[Gymnasium]]
- [[PufferLib]]
- [[Stable-Baselines3]]
- [[SAC]]
- [[PPO]]
- [[MJX]]

---

## 🔗 Related Notes

- [[MuJoCo]]
- [[URDF to MJCF and USD Pipeline]]
- [[Robot Dynamics and Spatial Algebra]]
- [[Actuator Modeling for Sim2Real]]
- [[System Identification for Sim2Real]]
- [[Gymnasium Environment Authoring for Robotics]]

---

## 🌐 External Resources

- MuJoCo Modeling Docs: https://mujoco.readthedocs.io/en/stable/modeling.html
- MJCF XML Reference: https://mujoco.readthedocs.io/en/stable/XMLreference.html

---

## 📝 Summary

MJCF is the practical robot modeling layer for MuJoCo-based RL. It is worth learning because many policy failures come from bad model structure, actuator settings, or contacts rather than the RL algorithm itself.
