# 🌳 Informed RRT*

**Informed RRT\*** is an enhancement of [[RRT*]] that focuses the sampling process within a subset of the configuration space that could potentially contain better solutions, thereby improving convergence toward the optimal path.

---

## 🧠 Summary

- Reduces unnecessary sampling by restricting exploration to an ellipsoidal subset once a solution is found.
- The ellipsoid is defined by the current best solution's cost and the start and goal positions.
- Retains asymptotic optimality while accelerating convergence.
- Particularly useful in high-dimensional spaces where uniform sampling is inefficient.

---

## ⚙️ Key Features

- **Informed sampling:** Only samples states that could improve the current best solution.
- **Asymptotically optimal:** Like [[RRT*]], guarantees convergence to the optimal path as samples → ∞.
- **Ellipsoidal sampling region:** Dynamically updates as better paths are found.

---

## 🚀 Applications

- Autonomous vehicles navigating complex environments.
- High-DOF robotic arms with tight motion constraints.
- UAVs and drones operating in cluttered 3D spaces.

---

## 🏆 Strengths

- Faster convergence to an optimal solution than standard RRT*.
- Focuses computation where it's most likely to improve the solution.
- Reduces time wasted on irrelevant parts of the space.

---

## ⚠️ Weaknesses

- Requires a feasible solution before informed sampling begins.
- Performance gain depends on the quality of the heuristic and the environment.

---

## 📊 Comparison with Related Planners

| Planner          | Type                     | Optimality               | Sampling Strategy        | Notes                                     |
|------------------|--------------------------|--------------------------|--------------------------|-------------------------------------------|
| Informed RRT*     | Sampling-based (optimal) | ✅ Asymptotically optimal | Focused (ellipsoidal)     | Faster convergence than RRT* |
| RRT*             | Sampling-based (optimal) | ✅ Asymptotically optimal | Uniform                  | Slower convergence in large spaces |
| RRT-Connect      | Sampling-based (feasible) | ❌ Not asymptotically optimal | Bidirectional + connect | Fast feasible paths |
| BIT*             | Sampling-based (optimal) | ✅ Asymptotically optimal | Batch + heuristic        | Efficient with informed sampling and batches |
| FMT*             | Sampling-based (optimal) | ✅ Asymptotically optimal | Batch                    | Similar performance to BIT* without incremental refinement |

---

## 🔗 Related Notes

- [[RRT*]]
- [[BIT*]]
- [[FMT*]]
- [[Path Planning]]
- [[Sampling-based Planning]]

---

## 🌐 External References

- [Original Informed RRT* Paper](https://arxiv.org/abs/1404.2334)
- [OMPL Informed RRT*](https://ompl.kavrakilab.org)

---
