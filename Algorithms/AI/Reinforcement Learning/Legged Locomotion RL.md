# 🦿 Legged Locomotion RL

**Legged Locomotion RL** uses reinforcement learning to train walking, running, recovery, and terrain traversal policies for quadrupeds, bipeds, and humanoids. It is one of the strongest real-world success areas for [[Sim2Real]] robotics.

---

## 📚 Overview

Legged locomotion is a good fit for RL because contact-rich movement is hard to hand-design, but simulation can provide millions of steps of practice. Modern systems often train in [[Isaac Gym]] or [[Isaac Lab]] with [[PPO]], [[RSL-RL]], [[Domain Randomization]], and command-conditioned policies.

---

## 🧠 Core Concepts

- **Command-Conditioned Policy**: Policy receives target velocity, yaw rate, or gait command.
- **Terrain Curriculum**: Gradually increases terrain difficulty.
- **Privileged Teacher**: Training policy with extra simulator state.
- **Student Policy**: Deployable policy using only real sensor observations.
- **Gait Reward**: Encourages speed tracking, foot clearance, smoothness, and stability.
- **Recovery Behavior**: Ability to recover from pushes, slips, or bad footholds.

---

## 📊 Comparison Chart

| Framework | Simulator | Best For | Strength | Weakness |
|---|---|---|---|---|
| [[Legged Gym]] | [[Isaac Gym]] | Quadrupeds | Proven Sim2Real | Legacy simulator |
| [[Isaac Lab]] | [[Isaac Sim]] | Modern robot learning | Active ecosystem | Heavy setup |
| [[RSL-RL]] | Isaac stack | PPO locomotion | Fast and focused | Fewer algorithms |
| [[RL-GAMES]] | Isaac stack | GPU RL | High throughput | Less educational |
| [[MuJoCo]] | MuJoCo | Dynamics research | Clean physics | Less GPU-scale |
| [[Brax]] | JAX physics | Accelerator experiments | Very fast | Different asset workflow |

---

## ✅ Pros

- Strong record of real robot transfer.
- Parallel simulation makes training practical.
- Policies can learn recovery behaviors not easily hand-coded.
- Command-conditioned policies are reusable.
- Good entry point for serious Sim2Real work.

---

## ❌ Cons

- Reward tuning can become fragile.
- Bad actuator models cause real-world failure.
- Terrain and contact modeling matter a lot.
- Direct torque policies are risky without safety layers.
- Evaluation requires careful hardware procedures.

---

## 🧰 Practical Recipe

1. Start with a known robot config in [[Legged Gym]] or [[Isaac Lab]].
2. Train velocity tracking with [[PPO]].
3. Add terrain curriculum after flat-ground walking works.
4. Add [[Domain Randomization]] for mass, friction, motor strength, and latency.
5. Distill privileged teacher policies into deployable student policies.
6. Deploy with action limits, watchdogs, and fallback standing behavior.

---

## 🔗 Related Notes

- [[Sim2Real]]
- [[Domain Randomization]]
- [[Actuator Modeling for Sim2Real]]
- [[RSL-RL]]
- [[Isaac Lab Task Authoring]]
- [[Unitree]]
- [[Whole-Body Control]]

---

## 🌐 External Resources

- Legged Gym: https://github.com/leggedrobotics/legged_gym
- RSL-RL: https://github.com/leggedrobotics/rsl_rl
- Isaac Lab: https://isaac-sim.github.io/IsaacLab/main/

---

## 📝 Summary

Legged locomotion is a practical RL robotics target because simulation is scalable and real-world success is well established. The core skills are reward design, actuator modeling, domain randomization, and safe deployment.
