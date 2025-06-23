# 🌳 RRT (Rapidly-exploring Random Tree)

**RRT (Rapidly-exploring Random Tree)** is a sampling-based path planning algorithm designed for efficient exploration of high-dimensional or complex continuous spaces. It's commonly used in robotics, autonomous vehicles, and manipulation tasks where the configuration space may be large or cluttered.

---

## 🧠 Summary

- Introduced by Steven M. LaValle in 1998.
- Builds a space-filling tree by incrementally sampling random points and extending the tree toward them.
- Suitable for high-dimensional spaces where grid- or graph-based methods become impractical.

---

## ⚙️ How It Works (High-Level)

1. Initialize the tree at the start configuration.
2. Sample a random point in the space.
3. Find the nearest node in the tree to the sample.
4. Extend the tree toward the sample (by a fixed step or until obstacle).
5. Repeat until the goal is reached or a stopping condition is met.

---

## 🚀 Applications

- Motion planning for robotic arms and mobile robots.
- Autonomous vehicle navigation in complex environments.
- Manipulation and grasp planning.
- Planning in dynamic or partially known environments (with variants like RRT* or RRT-Connect).

---

## 🏆 Strengths

- Works well in high-dimensional or complex spaces.
- Easy to implement and extend.
- Can be adapted for kinodynamic constraints.

---

## ⚠️ Weaknesses

- Paths are often sub-optimal (unless using variants like RRT*).
- May require post-processing to smooth or shorten paths.
- Performance can degrade in spaces with narrow passages.

---

## 🔗 Related Notes

- [[Path Planning]]
- [[PRM]]
- [[A*]]
- [[Potential Field Method]]
- [[RRT*]]

---

## 🌐 External References

- [LaValle’s original paper](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)
- [Wikipedia - RRT](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)

---

## 📊 Comparison with Similar Planners

| Planner         | Handles High-Dim Spaces | Path Optimality | Typical Use                                    |
|-----------------|------------------------|----------------|-----------------------------------------------|
| RRT             | ✅                       | ❌ (sub-optimal) | High-dim motion planning                      |
| RRT*            | ✅                       | ✅ (asymptotically optimal) | High-dim planning with optimality requirement |
| PRM             | ✅                       | ✅ (for static env) | Multi-query static environments               |
| A*              | ❌                       | ✅              | Discrete or grid-based spaces                  |

---
