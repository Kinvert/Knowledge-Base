# 🌲 RABIT* (Regionally Accelerated Batch Informed Trees)

**RABIT\*** (Regionally Accelerated Batch Informed Trees) is an advanced, sampling-based motion planning algorithm that extends [[BIT*]] by introducing *regional acceleration*. The core idea is to dynamically adjust the density of samples in promising regions of the space, accelerating convergence to an optimal solution.

---

## 🧠 Summary

- Builds upon the BIT* framework.
- Dynamically focuses sampling in regions likely to yield better solutions.
- Retains asymptotic optimality while reducing the number of unnecessary samples.
- Leverages heuristic-informed search with adaptive regional refinement.

---

## ⚙️ Key Features

- **Regionally accelerated sampling:** Focuses sampling in specific regions based on current best solutions and heuristic cost-to-go estimates.
- **Batch-based incremental planning:** Like BIT*, but smarter in how batches are allocated spatially.
- **Asymptotically optimal:** Guaranteed convergence to the optimal solution as samples → ∞.
- **Efficient collision checking:** By focusing samples, fewer checks are required on average.

---

## 🚀 Applications

- High-DOF robot motion planning (e.g. humanoid arms, legged robots).
- Autonomous vehicles in complex environments.
- UAV navigation through cluttered airspace.
- Real-time or resource-constrained planners needing faster convergence.

---

## 🏆 Strengths

- Faster convergence than BIT* in many scenarios.
- Reduced computational effort through regional focus.
- Suitable for high-dimensional and complex environments.

---

## ⚠️ Weaknesses

- More complex to implement than BIT* or FMT*.
- Still requires good heuristics for maximum benefit.
- May not outperform simpler methods in low-dimensional or easy spaces.

---

## 📊 Comparison with Similar Techniques

| Planner   | Type                        | Focus Strategy         | Optimality          | Notes                                      |
|-----------|-----------------------------|------------------------|--------------------|--------------------------------------------|
| RABIT*    | Sampling-based (optimal)     | Regional acceleration   | ✅ Asymptotically optimal | Regionally focused sampling |
| BIT*      | Sampling-based (optimal)     | Global informed batches | ✅ Asymptotically optimal | Uniform batch sampling within heuristic bounds |
| FMT*      | Sampling-based (optimal)     | Global batch            | ✅ Asymptotically optimal | Single large batch, no regional focus |
| RRT*      | Sampling-based (optimal)     | Incremental tree growth | ✅ Asymptotically optimal | No batch, purely incremental |
| PRM*      | Sampling-based (optimal)     | Uniform roadmap         | ✅ Asymptotically optimal | No incremental refinement |

---

## 🔗 Related Notes

- [[BIT*]]
- [[FMT*]]
- [[RRT*]]
- [[PRM*]]
- [[Path Planning]]
- [[Sampling-based Planning]]
- [[RGG]]

---

## 🌐 External References

- [RABIT* Original Paper (Gammell et al.)](https://arxiv.org/abs/2109.09263)
- [OMPL RABIT* Discussion](https://ompl.kavrakilab.org)

---
