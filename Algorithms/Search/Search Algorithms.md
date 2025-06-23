# 🔍 Search Algorithms

**Search algorithms** are fundamental techniques in computer science and artificial intelligence for finding solutions, paths, or specific data within a defined problem space. They can be applied to graphs, trees, arrays, and more, depending on the domain.

---

## 🧠 Summary

- **Purpose**: Explore or traverse data structures to locate specific elements, paths, or solutions
- **Domains**: AI (pathfinding, planning), databases, robotics, web search, game playing
- **Types**: Informed (uses heuristics) or uninformed (blind search)

---

## 🛠️ Common Search Algorithms

| Algorithm         | Type          | Key Features                           | Use Cases                       |
|-------------------|---------------|----------------------------------------|----------------------------------|
| Breadth-First Search (BFS) | Uninformed     | Explores level by level                | Shortest path in unweighted graph |
| Depth-First Search (DFS)   | Uninformed     | Explores as deep as possible before backtracking | Pathfinding, topological sorting |
| Dijkstra’s Algorithm       | Informed       | Finds shortest path with positive edge weights | Routing, mapping                 |
| A* Search                   | Informed       | Combines cost-so-far and heuristic     | Game AI, robotics, path planning |
| Greedy Best-First Search     | Informed       | Expands node closest to goal (heuristic only) | Quick but not optimal pathfinding |
| Iterative Deepening DFS     | Uninformed     | Combines DFS space efficiency with BFS completeness | Memory-limited search            |
| Bidirectional Search        | Uninformed     | Searches forward from start and backward from goal | Speeds up search in large graphs |
| Uniform Cost Search         | Uninformed     | Like BFS but considers path cost       | Pathfinding with varying costs    |

---

## ✅ Strengths

- Wide applicability across domains (AI, robotics, networking)
- Some methods (e.g. A*) can guarantee optimal solutions
- Can handle both discrete and continuous spaces

---

## ❌ Weaknesses

- Performance depends on the problem space and algorithm choice
- Memory-intensive (especially BFS and A* on large graphs)
- Heuristic design for informed search can be challenging

---

## 🌐 External References

- [MIT OCW: Introduction to Search](https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-034-artificial-intelligence-fall-2010/lecture-videos/lecture-6-search/)
- [Red Blob Games Pathfinding Guide](https://www.redblobgames.com/pathfinding/a-star/introduction.html)

---

## 🔗 Related Notes

- [[Algorithms]]
- [[A*]]
- [[Dijkstra’s Algorithm]]
- [[Graph Theory]]
- [[Pathfinding]]
- [[Heuristic Functions]]
- [[Tree Traversal]]

---
