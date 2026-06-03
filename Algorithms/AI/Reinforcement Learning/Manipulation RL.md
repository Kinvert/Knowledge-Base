# 🦾 Manipulation RL

**Manipulation RL** trains policies for grasping, pushing, insertion, tool use, dexterous hands, and contact-rich arm control. It overlaps heavily with [[Imitation Learning]], [[Offline RL for Robotics]], [[Diffusion Policy]], and robot dataset work.

---

## 📚 Overview

Manipulation is harder than many locomotion tasks because object contacts, perception, resets, and sparse rewards dominate the problem. Modern manipulation learning often mixes demonstrations, simulation, behavior cloning, offline RL, and visual policies rather than relying only on online RL.

---

## 🧠 Core Concepts

- **Contact-Rich Control**: Tasks where small force and friction differences matter.
- **Sparse Rewards**: Success signal may appear only when the task is completed.
- **Reset Distribution**: Initial object and robot states strongly affect learning.
- **Object Randomization**: Varying object shape, mass, friction, and pose.
- **Visual Servoing**: Using camera observations to guide motion.
- **Teleoperation Data**: Human demonstrations used for imitation or offline learning.

---

## 📊 Comparison Chart

| Approach | Best For | Strength | Weakness | Common Tools |
|---|---|---|---|---|
| Online RL | Sim manipulation | Can discover behavior | Sparse rewards hard | [[SAC]], [[PPO]] |
| Behavior cloning | Real demos | Simple and stable | Distribution shift | [[LeRobot]], [[robomimic]] |
| [[Diffusion Policy]] | Visuomotor actions | Multimodal actions | Needs good demos | Real robot datasets |
| [[ACT Action Chunking Transformer]] | Teleop imitation | Smooth action chunks | Dataset dependent | [[LeRobot]] |
| Offline RL | Logged data | Reuses data | Extrapolation error | [[Offline RL for Robotics]] |
| Motion planning + RL | Structured tasks | Uses planners and policies | Integration work | [[MoveIt]], [[OMPL]] |

---

## ✅ Pros

- Directly relevant to robot arms and practical automation.
- Demonstrations can reduce exploration difficulty.
- Sim benchmarks exist for reproducible experiments.
- Visual policies can avoid hand-engineered state.
- Combines well with classical planning and controllers.

---

## ❌ Cons

- Real-world resets are slow and expensive.
- Contact physics is difficult to simulate accurately.
- Vision introduces dataset and latency problems.
- Sparse rewards make pure RL inefficient.
- Policies can overfit to camera placement and object set.

---

## 🧰 Practical Recipe

1. Start with state-based reaching or pushing.
2. Add object pose and gripper state before adding images.
3. Use demonstrations for grasping or insertion tasks.
4. Train behavior cloning before offline RL.
5. Add domain randomization for object pose, mass, friction, lighting, and camera.
6. Deploy with workspace limits and force limits.

---

## 🔗 Related Notes

- [[Imitation Learning]]
- [[Diffusion Policy]]
- [[ACT Action Chunking Transformer]]
- [[LeRobot]]
- [[robomimic]]
- [[Operational Space Control]]
- [[Robot Data Collection and Teleoperation]]

---

## 🌐 External Resources

- ManiSkill: https://www.maniskill.ai/
- robosuite: https://robosuite.ai/
- robomimic: https://robomimic.github.io/

---

## 📝 Summary

Manipulation RL is usually a hybrid workflow: simulation, demonstrations, imitation learning, offline methods, and safety-bounded deployment. Pure online RL is often not the first practical choice for real robot arms.
