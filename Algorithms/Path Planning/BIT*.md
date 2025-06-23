# 🤖 BIT* (Batch Informed Trees)

**BIT\*** (Batch Informed Trees) is a sampling-based, asymptotically optimal motion planning algorithm that combines elements of informed graph search (like A*) with sampling-based planners (like RRT* and FMT*). It incrementally grows a tree of paths using batches of samples and focuses its search on the most promising areas of the space.

---

## 🧠 Summary

- Introduced by Jonathan D. Gammell, Siddhartha S. Srinivasa, and Timothy D. Barfoot (2015).
- Asymptotically optimal: path cost approaches optimal as the number of samples increases.
- Uses batches of samples + heuristic search to efficiently explore.
- Prioritizes exploration in regions most likely to improve the solution (informed sampling).

---

## ⚙️ Key Features

- Incremental, anytime algorithm: improves solution as time allows.
- Batches of samples progressively focus on regions near the best path.
- Integrates heuristic cost-to-go like A* with sampling-based exploration.
- Requires fewer collision checks than RRT* in many cases.

---

## 🚀 Applications

- Autonomous driving motion planning.
- Robotic arms with high-DOF (Degrees of Freedom).
- UAV and drone navigation.
- Any high-dimensional or complex environment planning.

---

## 🏆 Strengths

- Asymptotically optimal with faster convergence than RRT*.
- Anytime: returns initial solution quickly, improves with more time.
- Efficient use of sampling + search heuristics reduces computation.
- Suitable for high-dimensional planning spaces.

---

## ⚠️ Weaknesses

- Requires a good heuristic for best performance.
- More complex implementation than basic planners like RRT or PRM.
- May still need post-processing for smoothness.

---

## 📊 Comparison with Similar Techniques

| Planner      | Type                        | Smoothness       | Handles Obstacles | Global Planning | ROS Integration | Notable Traits                                 |
|--------------|-----------------------------|-----------------|------------------|----------------|----------------|-----------------------------------------------|
| BIT*         | Sampling-based (optimal)     | ⚠️ Needs smoothing | ✅                | ✅ (asymptotically optimal) | ⚠️ Limited direct support | Informed sampling, batch + tree search |
| FMT*         | Sampling-based (optimal)     | ⚠️ Needs smoothing | ✅                | ✅              | ⚠️ Limited direct support | Batch sampling, efficient collision checking |
| RRT*         | Sampling-based (optimal)     | ⚠️ Better than RRT | ✅                | ✅              | ✅              | Incremental, anytime |
| PRM*         | Sampling-based (optimal)     | ❌               | ✅                | ✅              | ✅              | Roadmap-based |
| CHOMP        | Optimization-based           | ✅ Excellent     | ✅                | ❌ (local only) | ✅              | Gradient-based smoothing |
| A*           | Graph search                 | ❌               | ✅                | ✅              | ✅              | Grid-based |
| RRT          | Sampling-based               | ❌               | ✅                | ✅              | ✅              | Fast, non-optimal |

---

## 🔗 Related Notes

- [[FMT*]]
- [[RRT*]]
- [[PRM]]
- [[Path Planning]]
- [[Sampling-based Planning]]
- [[OMPL]]

---

## 🌐 External References

- [BIT* Original Paper](https://arxiv.org/abs/1405.5848)
- [OMPL BIT* Documentation](https://ompl.kavrakilab.org/BITstar.html)

---
