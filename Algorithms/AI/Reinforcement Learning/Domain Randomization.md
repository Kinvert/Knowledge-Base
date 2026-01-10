# Domain Randomization

**Domain Randomization** (DR) is a technique for [[Sim2Real]] transfer where simulation parameters are varied randomly during training to produce policies robust enough to handle real-world conditions. Instead of trying to perfectly model reality, DR exposes the agent to such a wide variety of simulated conditions that the real world becomes just another sample from the training distribution.

---

## Overview

The core insight: if you train a policy to handle thousands of variations in friction, mass, lighting, textures, and sensor noise, it will generalize to the real world without needing accurate simulation. DR trades simulation fidelity for training diversity.

Originally popularized by OpenAI's work on dexterous manipulation (Dactyl), DR has become standard practice in robotics RL, especially for locomotion and manipulation tasks.

---

## Core Concepts

- **Visual Randomization**: Vary textures, lighting, colors, camera positions, backgrounds
- **Dynamics Randomization**: Vary mass, friction, damping, motor strength, joint limits
- **Sensor Randomization**: Add noise, latency, dropout to observations
- **Task Randomization**: Vary goal positions, object shapes, initial conditions
- **Automatic Domain Randomization (ADR)**: Algorithms that expand randomization ranges based on training performance

---

## Comparison: Randomization Types

| Type | What's Randomized | Use Case | Example |
|------|-------------------|----------|---------|
| **Visual DR** | Textures, lighting, colors | Vision-based policies | Random table textures for grasping |
| **Dynamics DR** | Physics params (mass, friction) | Locomotion, manipulation | Random floor friction for walking |
| **Sensor DR** | Noise, latency, dropout | Robust perception | Gaussian noise on joint encoders |
| **Task DR** | Goals, objects, layouts | Generalization | Random target positions |
| **ADR** | All above, auto-expanding | Maximum robustness | OpenAI Dactyl |

---

## Common Parameters to Randomize

| Category | Parameters |
|----------|------------|
| **Dynamics** | Mass, inertia, friction coefficients, damping, motor strength |
| **Visual** | Texture, color, lighting intensity/direction, camera FOV/position |
| **Sensor** | Observation noise, latency, action delay, sensor dropout |
| **Environment** | Gravity, terrain roughness, object positions/shapes |
| **Robot** | Link lengths, joint limits, PD gains |

---

## Implementation Approaches

| Approach | Description | Pros | Cons |
|----------|-------------|------|------|
| **Uniform Random** | Sample params uniformly from fixed ranges | Simple, predictable | May include unrealistic configs |
| **Gaussian** | Sample around nominal with variance | Centers on realistic | Tails may be insufficient |
| **Curriculum** | Start narrow, widen over training | Easier learning | Requires tuning schedule |
| **ADR** | Auto-expand ranges when agent succeeds | Finds necessary variety | Complex implementation |
| **Stratified** | Ensure coverage of param space | Even distribution | Harder to implement |

---

## Strengths

- Enables zero-shot sim2real transfer
- No need for accurate system identification
- Produces robust, generalizable policies
- Works with any simulator
- Parallelizes well on GPU (e.g., [[Isaac Gym]])

---

## Weaknesses

- Over-randomization makes learning harder
- May produce overly conservative policies
- Doesn't help if observation space differs fundamentally
- Can require significant compute to explore variation space
- Hard to know optimal randomization ranges a priori

---

## Tools and Frameworks

| Tool | DR Support |
|------|------------|
| [[Isaac Gym]] | Built-in domain randomization APIs |
| [[Isaac Lab]] | Configurable randomization via managers |
| [[MuJoCo]] | Manual XML modification or MJX |
| [[PyBullet]] | Programmatic param changes |
| [[ProcTHOR]] | Procedural scene generation (extreme visual DR) |

---

## Related Notes

- [[Sim2Real]] (DR is the primary technique)
- [[Isaac Gym]] (Native DR support)
- [[Isaac Lab]] (Configurable randomization)
- [[RSL-RL]] (Used with DR for locomotion)
- [[Legged Gym]] (Locomotion training with DR)
- [[PufferLib]] (RL training infrastructure)
- [[Reinforcement Learning]]

---

## External Resources

- [Domain Randomization for Transferring DNNs (2017)](https://arxiv.org/abs/1703.06907) - Original paper
- [OpenAI Dactyl: Learning Dexterity](https://openai.com/research/learning-dexterity) - ADR in action
- [Sim-to-Real Survey](https://arxiv.org/abs/2009.13303) - Comprehensive overview
- [Isaac Gym DR Tutorial](https://developer.nvidia.com/isaac-gym)

---
