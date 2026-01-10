# Sim2Real

**Sim2Real** (simulation-to-reality transfer) is the process of deploying policies trained in simulation to physical robots. The core challenge is bridging the **reality gap**—the mismatch between simulated and real-world physics, visuals, and sensor behavior. Successful sim2real enables safe, cheap, scalable robot training without wearing down hardware or risking damage.

---

## Overview

Training robots directly in the real world is slow, expensive, and dangerous. Simulation offers parallelized training at millions of FPS with no hardware wear. But naive transfer fails because simulators can't perfectly model friction, motor dynamics, contact forces, sensor noise, and visual appearance.

Key approaches to close the reality gap:
- [[Domain Randomization]] (vary sim parameters during training)
- System identification (tune sim to match real)
- Progressive transfer (sim → sim → real)
- Sim2sim validation (test on different simulator first)

---

## Core Concepts

- **Reality Gap**: Differences between simulation and real-world physics, visuals, and dynamics that cause trained policies to fail on hardware
- **Zero-Shot Transfer**: Deploying directly from sim to real without fine-tuning
- **Few-Shot Transfer**: Brief real-world adaptation after sim training
- **Domain Adaptation**: Techniques to align sim and real distributions (adversarial, style transfer)
- **System Identification**: Measuring real robot parameters to configure simulation accurately
- **Sim2Sim**: Testing on a second simulator before real deployment

---

## Techniques

| Technique | Approach | Pros | Cons |
|-----------|----------|------|------|
| **Domain Randomization** | Vary physics, visuals, dynamics randomly | Robust policies, no real data needed | Over-conservative, training harder |
| **System Identification** | Measure real params, tune sim | Accurate for known robot | Doesn't generalize to new environments |
| **Progressive Training** | Start simple sim → harder sim → real | Curriculum reduces gap | Complex pipeline |
| **Adversarial Adaptation** | Train discriminator to align domains | Learns transferable features | Training instability |
| **Teacher-Student Distillation** | Privileged sim teacher → deployable student | Clean separation of concerns | Requires careful design |

---

## Success Stories

| Project | Robot | Task | Approach |
|---------|-------|------|----------|
| **OpenAI Dactyl** | Shadow Hand | Rubik's cube manipulation | Automatic DR, thousands of params |
| **ANYmal (ETH)** | Quadruped | Rough terrain locomotion | [[RSL-RL]], domain randomization |
| **Boston Dynamics Spot** (NVIDIA) | Quadruped | Velocity tracking | [[Isaac Lab]], zero-shot |
| **Walk-These-Ways** | Unitree Go1 | Multi-gait locomotion | [[Isaac Gym]], multiplicity of behavior |
| **Dactyl 2.0** | Shadow Hand | Pen spinning | Vision-based, DR + real fine-tuning |

---

## Common Failure Modes

- **Dynamics mismatch**: Friction, mass, motor behavior differ
- **Sensor gap**: Simulated cameras/IMUs don't match real noise
- **Latency differences**: Real control loops have delays
- **State estimation errors**: Sim assumes perfect state, real doesn't
- **Contact modeling**: Rigid body sim can't capture deformation, slipping

---

## Best Practices

1. **Validate with sim2sim** before touching hardware
2. **Domain randomize conservatively** at first, increase variance gradually
3. **Match control frequency** between sim and real
4. **Add realistic latency** to simulated observations and actions
5. **Use real sensor noise profiles** if available
6. **Test edge cases** (slippery surfaces, unexpected contacts)
7. **Log everything** on first real-world runs for debugging

---

## Comparison: Transfer Strategies

| Strategy | When to Use | Reality Gap Tolerance |
|----------|-------------|----------------------|
| **Zero-shot + DR** | Locomotion, robust manipulation | High |
| **Few-shot fine-tuning** | Dexterous manipulation | Medium |
| **System ID + sim** | Known robot, controlled environment | Low |
| **Full real-world training** | Final polish, behavior cloning | None (no sim) |

---

## Related Notes

- [[Domain Randomization]] (Core technique for sim2real)
- [[RSL-RL]] (ETH's library for sim2real locomotion)
- [[Isaac Lab]] (NVIDIA's sim2real framework)
- [[Isaac Gym]] (GPU-accelerated training platform)
- [[Legged Gym]] (Locomotion training on Isaac Gym)
- [[PufferLib]] (High-performance RL library)
- [[Reinforcement Learning]]
- [[PPO]] (Common algorithm for sim2real)

---

## External Resources

- [Sim-to-Real Transfer Survey (2020)](https://arxiv.org/abs/2009.13303) - Comprehensive overview
- [Domain Randomization for Transferring DNNs](https://arxiv.org/abs/1703.06907) - Original DR paper
- [Learning Dexterous In-Hand Manipulation (OpenAI)](https://openai.com/research/learning-dexterity)
- [ETH Robotic Systems Lab](https://rsl.ethz.ch/research/researchtopics/rl-robotics.html)

---
