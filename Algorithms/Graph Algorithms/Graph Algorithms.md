# 🕸️ Graph Algorithms

**Graph algorithms** are designed to solve problems involving graphs — data structures made of nodes (vertices) connected by edges. They are critical in computer science, robotics, networking, logistics, and many other domains.

This note serves as an index for various graph-related algorithm categories and key examples.

---

## 🧠 What is a Graph?

A **graph** consists of:

- **Vertices (nodes)**: The entities or points.
- **Edges (links)**: The connections between vertices (can be directed/undirected, weighted/unweighted).

Graphs can represent anything from social networks and maps to circuits and dependency structures.

---

## 🛠️ Categories of Graph Algorithms

| Category                   | Description                                           | Example Algorithms                       |
|----------------------------|-------------------------------------------------------|------------------------------------------|
| [[Pathfinding Algorithms]]   | Find paths between nodes                             | [[Dijkstra’s Algorithm]], [[A* Search]], BFS, DFS |
| [[Minimum Spanning Tree]]    | Find subset of edges forming a tree of minimal total weight | [[Kruskal's Algorithm]], [[Prim's Algorithm]] |
| [[Connectivity Algorithms]]  | Identify connected components, cycles, bridges       | Tarjan's SCC, Kosaraju’s SCC, Union-Find |
| [[Graph Traversal]]          | Explore nodes systematically                         | BFS, DFS                                 |
| [[Topological Sorting]]      | Linear ordering of vertices in DAGs                  | Kahn’s Algorithm, DFS-based sort         |
| [[Network Flow Algorithms]]  | Compute max flow, min cut in networks                | Ford-Fulkerson, Edmonds-Karp             |

---

## ✅ Strengths

- Solve a wide variety of structural and optimization problems
- Foundation for higher-level algorithms in AI, networking, and operations research

---

## ❌ Weaknesses

- Large or dense graphs can require significant memory and time
- Some algorithms (e.g. max flow) can be complex to implement efficiently

---

## 🔗 Related Notes

- [[Algorithms]]
- [[Data Structures]]
- [[Pathfinding Algorithms]]
- [[Minimum Spanning Tree]]

---

## 🌐 External References

- [GeeksforGeeks Graph Algorithms](https://www.geeksforgeeks.org/graph-data-structure-and-algorithms/)
- [MIT OCW - Advanced Graph Algorithms](https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-851-advanced-data-structures-spring-2012/lecture-videos/)

---
