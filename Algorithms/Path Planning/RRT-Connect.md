# 🌳 RRT-Connect

**RRT-Connect** is an extension of the classic [[RRT]] (Rapidly-exploring Random Tree) algorithm designed to improve performance in difficult path planning problems by using *bidirectional tree growth* and an aggressive connection strategy.

---

## 🧠 Summary

- RRT-Connect grows two trees simultaneously: one from the start and one from the goal.
- When one tree extends toward a random sample, it attempts to connect to the other tree as far as possible along that direction.
- Known for fast performance in high-dimensional spaces with narrow passages.
- Not asymptotically optimal (unlike [[RRT*]]), but highly practical for many applications.

---

## ⚙️ Key Features

- **Bidirectional search:** Two trees grow toward each other to reduce search time.
- **Connect extension:** Instead of small incremental steps, RRT-Connect tries to extend as far as possible along a straight line until it hits an obstacle.
- **Fast runtime:** Often finds a feasible solution quickly, especially in cluttered environments.

---

## 🚀 Applications

- Robotic arm planning in confined workspaces.
- Mobile robot navigation in cluttered indoor environments.
- Real-time applications where feasible paths matter more than optimal ones.

---

## 🏆 Strengths

- Very fast at finding feasible paths.
- Bidirectional growth is effective for narrow passages.
- Simple to implement with good practical performance.

---

## ⚠️ Weaknesses

- Paths tend to be suboptimal and require post-processing (e.g., smoothing).
- Not asymptotically optimal — it doesn't guarantee convergence to the best path with increasing samples.

---

## 📊 Comparison with Similar Planners

| Planner       | Type                    | Optimality               | Strategy               | Notes                                 |
|---------------|-------------------------|--------------------------|------------------------|---------------------------------------|
| RRT-Connect    | Sampling-based (feasible) | ❌ Not asymptotically optimal | Bidirectional + connect | Fast feasible paths, suboptimal |
| RRT            | Sampling-based (feasible) | ❌ Not asymptotically optimal | Single tree, incremental | Slower in complex spaces |
| RRT*           | Sampling-based (optimal) | ✅ Asymptotically optimal | Single tree, incremental | Slower initially, refines over time |
| PRM            | Sampling-based (feasible) | ❌ (PRM*) optimal variant exists | Roadmap (precomputed) | Good for multi-query problems |
| FMT*           | Sampling-based (optimal) | ✅ Asymptotically optimal | Batch + graph           | Efficient in high-DOF problems |

---

## 🔗 Related Notes

- [[RRT]]
- [[RRT*]]
- [[Path Planning]]
- [[Sampling-based Planning]]

---

## 🌐 External References

- [Original RRT-Connect Paper](http://www.kavrakilab.org/publications/latombe_2000_rrtconnect.pdf)
- [OMPL RRT-Connect](https://ompl.kavrakilab.org/rrtconnect.html)

---
