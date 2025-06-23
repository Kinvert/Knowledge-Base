# 🌳 ABIT* (Anytime Batch Informed Trees)

**ABIT\*** is a sampling-based motion planning algorithm that blends features of [[BIT*]] (Batch Informed Trees) and [[Informed RRT*]], providing *anytime* performance — meaning it can return a feasible solution quickly and then progressively refine it toward optimality as time allows.

---

## 🧠 Summary

- Designed for scenarios where a quick feasible path is needed, but refinement toward optimality is desired if more computation time is available.
- Combines batch processing (like BIT*) with anytime characteristics (returns early feasible paths).
- Uses informed sampling techniques to focus search on promising regions.
- Asymptotically optimal.

---

## ⚙️ Key Features

- **Anytime planning:** Delivers a path early, improves as more time is given.
- **Batch + incremental:** Samples in batches but outputs paths incrementally as solutions are refined.
- **Informed sampling:** Focuses computation on regions that could yield better solutions.
- **Graph-based search:** Efficient exploration using heuristics on a sparse graph.

---

## 🚀 Applications

- Robotics systems where both fast initial paths and long-term refinement matter (e.g., autonomous vehicles, manipulators).
- Real-time systems that must start moving quickly but want to improve motion over time.
- Dynamic environments where conditions can change as planning continues.

---

## 🏆 Strengths

- Produces quick initial solutions and improves them over time.
- Efficient in high-dimensional configuration spaces.
- Combines strengths of BIT* and Informed RRT*.

---

## ⚠️ Weaknesses

- More complex implementation than simpler planners (e.g., [[RRT-Connect]]).
- Performance benefits most apparent in large or complex planning spaces.

---

## 📊 Comparison with Related Planners

| Planner       | Type                     | Optimality               | Sampling Strategy         | Notes                                   |
|---------------|--------------------------|--------------------------|--------------------------|-----------------------------------------|
| ABIT*         | Sampling-based (optimal) | ✅ Asymptotically optimal | Batch + anytime + informed | Early feasible path + refinement |
| BIT*          | Sampling-based (optimal) | ✅ Asymptotically optimal | Batch + informed           | Efficient, no anytime behavior |
| Informed RRT* | Sampling-based (optimal) | ✅ Asymptotically optimal | Informed incremental       | No batch, slower initial path |
| RRT-Connect   | Sampling-based (feasible) | ❌ Not asymptotically optimal | Bidirectional + connect    | Very fast initial path, no refinement |
| FMT*          | Sampling-based (optimal) | ✅ Asymptotically optimal | Batch                      | Efficient in high-DOF problems |

---

## 🔗 Related Notes

- [[BIT*]]
- [[Informed RRT*]]
- [[RRT*]]
- [[FMT*]]
- [[Path Planning]]
- [[Sampling-based Planning]]

---

## 🌐 External References

- [ABIT* Original Paper](https://arxiv.org/abs/1907.00530)
- [OMPL ABIT*](https://ompl.kavrakilab.org)

---
