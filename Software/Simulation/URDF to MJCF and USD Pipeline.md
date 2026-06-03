# 🔄 URDF to MJCF and USD Pipeline

**URDF to MJCF and USD pipelines** convert robot descriptions into simulator-ready assets for [[MuJoCo]], [[MJX]], [[Isaac Sim]], and [[Isaac Lab]]. The hard part is preserving frames, joints, inertias, actuators, and collision behavior across ecosystems.

---

## 📚 Overview

Robot learning usually starts with a robot description file. [[URDF]] and [[XACRO]] are common in [[ROS2]] and [[MoveIt]]. [[MuJoCo]] uses MJCF. [[Isaac Sim]] and [[Isaac Lab]] use [[OpenUSD]]. Conversion is useful, but every conversion should be treated as a model audit, not a blind file-format operation.

---

## 🧠 Core Concepts

- **Kinematic Tree**: Link and joint hierarchy.
- **Frames**: Coordinate transforms for links, joints, sensors, and tools.
- **Inertials**: Mass, center of mass, and inertia tensor.
- **Visual Geometry**: Meshes used for rendering.
- **Collision Geometry**: Simplified geometry used for contacts.
- **Actuator Metadata**: Control mode, limits, gear ratios, gains, and torque bounds.
- **Material and Contact Parameters**: Friction, restitution, compliance, and solver-related settings.

---

## 📊 Comparison Chart

| Format | Best Use | Strength | Weakness | Common Destination |
|---|---|---|---|---|
| **URDF** | ROS robot descriptions | Standard and widely supported | Limited contact/actuator detail | ROS2, MoveIt |
| **XACRO** | Generated URDFs | Reusable macros and params | Harder to inspect final model | URDF |
| **MJCF** | MuJoCo dynamics | Strong actuator/contact modeling | Less native to ROS | MuJoCo, MJX |
| **OpenUSD** | Isaac scenes | Rich scene graph and rendering | Complex ecosystem | Isaac Sim, Isaac Lab |
| **SDF** | Gazebo scenes | Good simulator structure | Less common in RL stacks | Gazebo/Ignition |
| **Mesh-only assets** | Visual models | Easy to view | No robot semantics | Needs robot description |

---

## ✅ Pros

- Enables reuse of robot models across ROS, MuJoCo, and Isaac.
- Makes simulator comparison easier.
- Helps build Sim2Sim checks before [[Sim2Real]].
- Lets the same robot description support planning, training, and deployment.
- Encourages explicit audits of joints, limits, and inertials.

---

## ❌ Cons

- Conversions often drop actuator semantics.
- Mesh scale and coordinate frames frequently break.
- Inertias may be missing, fake, or unstable.
- Collision geometry may be too detailed for RL.
- Converted models still need simulator-specific tuning.

---

## 🧰 Conversion Checklist

1. Inspect the generated model visually.
2. Compare link and joint names to the source.
3. Verify joint axes, limits, and parent-child transforms.
4. Replace visual meshes with simple collision geometry where possible.
5. Check every link mass and inertia.
6. Add actuator limits and gains in the target simulator.
7. Run passive simulation before adding a policy.

---

## 🔗 Related Notes

- [[URDF]]
- [[XACRO]]
- [[MuJoCo MJCF]]
- [[OpenUSD]]
- [[Isaac Lab Task Authoring]]
- [[MoveIt]]
- [[Robot Dynamics and Spatial Algebra]]

---

## 🌐 External Resources

- MuJoCo XML Reference: https://mujoco.readthedocs.io/en/stable/XMLreference.html
- Isaac Sim Asset Import: https://docs.isaacsim.omniverse.nvidia.com/
- ROS URDF XML: https://wiki.ros.org/urdf/XML

---

## 📝 Summary

Asset conversion is a robotics RL bottleneck because small model errors can look like algorithm failures. Treat URDF, MJCF, and USD conversion as a physics and control audit, not just a file conversion step.
