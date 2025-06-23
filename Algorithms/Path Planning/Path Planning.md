# 🛤️ Path Planning

**Path Planning** refers to the process of determining a feasible route or sequence of movements for an agent (such as a robot or vehicle) to navigate from a start point to a goal while avoiding obstacles. It is a cornerstone of robotics, autonomous driving, and AI navigation systems.

---

## 🧠 Summary

- **Goal:** Find an optimal or feasible path in an environment.
- May consider static or dynamic obstacles, kinematics, and cost metrics.
- Used in robotics, autonomous vehicles, games, drones, and logistics.

---

## ⚙️ Categories of Path Planning Algorithms

| Category                | Example Algorithms                            | Notes                                        |
|-------------------------|-----------------------------------------------|----------------------------------------------|
| **Graph-based Search**   | [[A*]], [[Dijkstra]], [[Breadth First Search]] | Discretize environment as graph or grid      |
| **Sampling-based**       | [[RRT]], [[PRM]]                              | Good for high-dimensional spaces             |
| **Optimization-based**   | Gradient descent planners, CHOMP, TrajOpt     | Solve as optimization problem                |
| **Potential Field**      | [[Potential Field Method]]                    | Treat obstacles as repulsive forces          |
| **Heuristic/Meta-heuristic** | Genetic algorithms, Ant Colony Optimization | Explore large, complex spaces heuristically  |

---

## 📊 Path Planning Algorithm Comparison

| Algorithm / Type            | Suitable For                                   | Strengths                                       | Weaknesses                                      |
|-----------------------------|------------------------------------------------|------------------------------------------------|-------------------------------------------------|
| A* (Graph-based)             | Grid or graph-based maps                       | Guarantees optimal path (if heuristic admissible), efficient | Can struggle in high-dimensional or continuous spaces |
| Dijkstra (Graph-based)       | Known map with uniform cost                    | Finds shortest path, simple                     | Slower than A* when heuristic available         |
| Breadth First Search (Graph) | Small discrete spaces                          | Complete, simple                               | Inefficient in large or weighted graphs         |
| RRT (Sampling-based)         | High-dimensional, complex environments         | Handles complex, continuous spaces well         | Paths can be sub-optimal, post-processing needed |
| PRM (Sampling-based)         | Multi-query problems in static environments    | Efficient for repeated planning                 | Slow for dynamic obstacles                      |
| Potential Field (Reactive)   | Real-time obstacle avoidance                   | Fast, low computational load                    | Can get stuck in local minima                   |
| CHOMP / TrajOpt (Optimization)| Smooth, kinematic-aware paths                  | Generates smooth, feasible trajectories         | Requires good initial guess, slower for large spaces |
| Genetic / Ant Colony (Meta)  | Complex, non-convex spaces                     | Good for global exploration                     | Slow convergence, computationally intensive     |

---

## 🚀 Applications

- Robot navigation in structured and unstructured environments
- Self-driving car route planning
- UAV flight path generation
- Logistics and warehouse automation
- Game AI movement and NPC behavior

---

## 🏆 Strengths

- Enables autonomous operation in complex environments.
- Can balance optimality, safety, and efficiency.
- Wide variety of algorithms for different problem types.

---

## ⚠️ Challenges

- Computationally expensive in high-dimensional spaces.
- Dynamic environments require real-time replanning.
- Trade-offs between path optimality and computation time.

---

## 🔗 Related Notes

- [[A*]]
- [[Dijkstra]]
- [[Potential Field Method]]
- [[RRT]]
- [[SLAM]]
- [[Graph Algorithms]]
- [[Obstacle Avoidance]]

---

## 🌐 External References

- [Wikipedia - Motion Planning](https://en.wikipedia.org/wiki/Motion_planning)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)

---
