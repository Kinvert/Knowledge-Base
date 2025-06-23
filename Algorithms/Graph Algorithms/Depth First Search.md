# 🌿 Depth-First Search (DFS)

**Depth-First Search (DFS)** is a fundamental graph traversal algorithm that explores as far as possible along a branch before backtracking. It is widely used in scenarios involving pathfinding, connectivity, cycle detection, and topological sorting.

---

## 🧠 Summary

- Explores a node's descendants before visiting its siblings.
- Typically implemented using **recursion** (implicit stack) or an explicit **stack**.
- Useful for problems requiring complete exploration of all paths or structures in a graph.

```
function DFS(graph, start, visited):
    if visited does not contain start:
        add start to visited
        process(start)

        for each neighbor in graph[start]:
            DFS(graph, neighbor, visited)
```

---

## ⚙️ Key Properties

| Property                   | Value                               |
|----------------------------|-------------------------------------|
| Time complexity             | O(V + E) (V = vertices, E = edges) |
| Space complexity            | O(V) (due to stack/visited tracking) |
| Suitable for                | Cycle detection, topological sort, connected components |
| Data structure used         | Stack (explicit or via recursion)  |

---

## 🏁 Applications

- Detecting cycles in a graph
- Topological sorting (DAGs)
- Connected components detection
- Solving puzzles and mazes
- Pathfinding in deep or sparse graphs
- Articulation points, bridges

---

## 🚀 Strengths

- Lower memory use than [[Breadth First Search]] (BFS) on wide graphs.
- Can be easily adapted for many advanced graph algorithms.
- Naturally suited for recursive solutions.

---

## ⚠️ Weaknesses

- Does **not** guarantee the shortest path in unweighted graphs.
- Can get stuck exploring deep paths unnecessarily (when shallow solutions exist).

---

## 🔄 Comparison to Breadth-First Search (BFS)

| Aspect            | [[DFS]]                   | [[Breadth First Search]]         |
|-------------------|---------------------------|----------------------------------|
| Order of traversal | Deepest nodes first        | Level-by-level                  |
| Memory use         | Lower in wide graphs       | Higher in wide graphs           |
| Finds shortest path | ❌ No                      | ✅ Yes (unweighted graphs)       |
| Data structure     | Stack (or recursion)       | Queue                           |

---

## 🔗 Related Notes

- [[Graph Algorithms]]
- [[Topological Sort]]
- [[Connected Components]]
- [[Minimum Spanning Tree]]
- [[Tarjan's Algorithm]]

---

## 🌐 External References

- [Wikipedia - Depth-First Search](https://en.wikipedia.org/wiki/Depth-first_search)
- [CP-Algorithms - DFS](https://cp-algorithms.com/graph/depth-first-search.html)

---
