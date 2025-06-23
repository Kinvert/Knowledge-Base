# 🌳 RRT* (Rapidly-exploring Random Tree Star)

**RRT\*** is an enhancement of the original **RRT (Rapidly-exploring Random Tree)** that not only explores high-dimensional spaces efficiently but also guarantees asymptotic optimality. Over time, as more samples are added, RRT* refines the tree structure to produce shorter, more optimal paths.

---

## 🧠 Summary

- Proposed by Karaman and Frazzoli in 2011.
- Extends RRT by rewiring the tree to improve path quality as sampling proceeds.
- Provides guarantees of finding the optimal path as the number of samples approaches infinity.

---

## ⚙️ How It Works (High-Level)

1. Sample a random point in the space.
2. Find the nearest node in the tree.
3. Steer toward the sample within a set step size.
4. Identify nearby nodes within a radius.
5. Choose the node that provides the least-cost path to the sample point.
6. Add the new point to the tree.
7. Rewire nearby nodes if connecting through the new point improves their path cost.

---

## 🚀 Applications

- Autonomous driving (optimal path planning)
- Robotic arm motion planning with optimality constraints
- Complex manipulation tasks requiring smooth, short paths
- Any scenario where both feasibility and near-optimal solutions are required

---

## 🏆 Strengths

- Produces asymptotically optimal paths.
- Retains benefits of RRT in high-dimensional spaces.
- Can handle complex, non-convex spaces.

---

## ⚠️ Weaknesses

- Computationally more intensive than RRT.
- Slower convergence in very high-dimensional or heavily constrained spaces.
- Requires tuning (e.g., radius, step size) for good performance.

---

## 🔎 RRT* vs. RRT — Key Differences

| Aspect                  | RRT                          | RRT*                              |
|-------------------------|-----------------------------|-----------------------------------|
| Path Quality             | Feasible but often sub-optimal | Asymptotically optimal (improves over time) |
| Tree Structure           | Grows without revisiting connections | Rewires tree to reduce path costs |
| Computational Cost       | Lower                        | Higher (more neighbor checks, rewiring) |
| Use Case Preference      | Fast initial solution, feasibility | Optimal or near-optimal paths needed |
| Convergence              | No guarantee of optimal path | Guarantees optimal path as samples → ∞ |

---

## 🔗 Related Notes

- [[Path Planning]]
- [[RRT]]
- [[PRM]]
- [[A*]]
- [[Potential Field Method]]

---

## 🌐 External References

- [Original RRT* Paper](http://motion.cs.illinois.edu/papers/karaman2011sampling.pdf)
- [Wikipedia - RRT*](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)

---
