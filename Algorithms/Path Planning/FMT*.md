# 🤖 FMT* (Fast Marching Tree*)

**FMT\*** is a sampling-based motion planning algorithm that combines elements of graph search and sampling to efficiently compute asymptotically optimal paths. It constructs a tree like RRT*, but in a batch fashion inspired by the Fast Marching Method for solving boundary value problems.

---

## 🧠 Summary

- Developed by J. J. Alonso-Mora, L. Jaillet, and E. Frazzoli (2013).
- Asymptotically optimal: path cost approaches optimal as samples increase.
- Uses lazy collision checking to minimize expensive calls.
- Explores space similarly to wavefront expansion (Fast Marching).

---

## ⚙️ Key Features

- Batch planner: samples are generated up front.
- Uses dynamic programming on a disk-connected random geometric graph.
- Prioritizes cost-to-come during expansion.
- Efficient in high-dimensional spaces compared to PRM*, RRT*.
- Suitable for static environments.

---

## 🚀 Applications

- Autonomous vehicle motion planning.
- Robotic manipulators.
- Drone path planning.
- Planning in high-dimensional configuration spaces.

---

## 🏆 Strengths

- Efficiently finds high-quality paths with fewer samples.
- Fewer collision checks compared to PRM* or RRT*.
- Asymptotically optimal with guaranteed convergence to the best path.

---

## ⚠️ Weaknesses

- Batch approach: not suitable for anytime or online planning.
- Not ideal for highly dynamic environments (since graph is built in advance).
- Path smoothness can require post-processing.

---

## 📊 Comparison with Similar Techniques

| Planner      | Type                        | Smoothness       | Handles Obstacles | Global Planning | ROS Integration | Notable Traits                    |
|--------------|-----------------------------|-----------------|------------------|----------------|----------------|-----------------------------------|
| FMT*         | Sampling-based (optimal)     | ⚠️ Needs post-processing | ✅                | ✅ (asymptotically optimal) | ⚠️ No direct integration | Efficient batch sampling, low collision checks |
| RRT*         | Sampling-based (optimal)     | ⚠️ Better than RRT | ✅                | ✅ (asymptotically optimal) | ✅              | Incremental, anytime capable |
| PRM*         | Sampling-based (optimal)     | ❌               | ✅                | ✅              | ✅              | Probabilistic roadmap approach |
| RRT          | Sampling-based               | ❌               | ✅                | ✅              | ✅              | Fast but non-optimal paths |
| A*           | Graph search                 | ❌               | ✅                | ✅              | ✅              | Grid-based, exact cost if grid fine |
| Dijkstra     | Graph search                 | ❌               | ✅                | ✅              | ✅              | Exhaustive, guarantees shortest on graph |
| CHOMP        | Optimization-based           | ✅ Excellent     | ✅                | ❌ (local refinement) | ✅              | Gradient-based smoothing |
| STOMP        | Stochastic optimization      | ✅ Good          | ✅                | ❌ (local refinement) | ✅              | Robust to local minima |

---

## 🔗 Related Notes

- [[RRT*]]
- [[PRM]]
- [[CHOMP]]
- [[Path Planning]]
- [[Sampling-based Planning]]
- [[OMPL]]

---

## 🌐 External References

- [Original FMT* Paper](https://arxiv.org/abs/1401.5843)
- [OMPL FMT* Documentation](https://ompl.kavrakilab.org/FMTstar.html)

---
