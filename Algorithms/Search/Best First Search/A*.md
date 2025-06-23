# ⭐ A* Algorithm

**A\*** (pronounced "A star") is one of the most popular and widely used graph search algorithms for pathfinding and graph traversal. It efficiently finds the shortest path from a start node to a goal node using a heuristic to guide the search.

---

## 🧠 Summary

- Combines the benefits of [[Dijkstra's Algorithm]] (optimal, exhaustive search) and greedy best-first search (guided by heuristic).
- Uses a cost function `f(n) = g(n) + h(n)` where:
  - `g(n)` = cost from the start node to current node `n`
  - `h(n)` = heuristic estimate of the cost from `n` to goal
- Finds the shortest path if `h(n)` is admissible (never overestimates the true cost).

---

## ⚙️ Key Features

- **Optimal (with admissible heuristic):** Guarantees shortest path.
- **Complete:** Will always find a solution if one exists.
- **Flexible:** Can use various heuristics depending on the problem domain (e.g., Euclidean distance, Manhattan distance).

---

## 🚀 Applications

- Robot motion planning
- Game AI (pathfinding on maps)
- Network routing
- Puzzle solving (e.g., sliding tile puzzles)

---

## 🏆 Strengths

- Finds optimal solutions efficiently in many scenarios.
- Can be adapted to different domains by changing the heuristic.
- Works on graphs, grids, and continuous spaces (with discretization).

---

## ⚠️ Weaknesses

- Performance heavily depends on the heuristic.
- Can be slow and memory-intensive on very large graphs or poor heuristics.
- Not suited for dynamic environments without re-planning.

---

## 📊 Comparison with Related Algorithms

| Algorithm            | Optimality               | Completeness | Heuristic Used | Notes |
|----------------------|-------------------------|--------------|----------------|-------|
| A*                   | ✅ Optimal (if h admissible) | ✅ Yes       | ✅ Yes          | Balances cost-so-far and heuristic |
| Dijkstra's Algorithm  | ✅ Optimal                | ✅ Yes       | ❌ No           | Equivalent to A* with h=0 |
| Greedy Best-First     | ❌ Not guaranteed optimal | ✅ Yes       | ✅ Yes          | Faster but less reliable |
| RRT*                 | ✅ Asymptotically optimal | ✅ Yes       | ✅ (implicit via sampling) | Used in continuous space motion planning |
| FMT*                  | ✅ Asymptotically optimal | ✅ Yes       | ✅ (implicit via batches) | Efficient in high-DOF spaces |

---

## 🔗 Related Notes

- [[Dijkstra's Algorithm]]
- [[Greedy Search]]
- [[Path Planning]]
- [[RRT*]]
- [[FMT*]]

---

## 🌐 External References

- [A* on Red Blob Games](https://www.redblobgames.com/pathfinding/a-star/introduction.html) — an excellent interactive explanation
- [A* Wikipedia](https://en.wikipedia.org/wiki/A*_search_algorithm)

---
