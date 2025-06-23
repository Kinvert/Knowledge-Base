# 🔍 Breadth-First Search (BFS)

**Breadth-First Search (BFS)** is a fundamental graph traversal algorithm that explores vertices level by level, visiting all neighbors of a vertex before moving on to their neighbors. It is commonly used in shortest path finding (on unweighted graphs), connectivity checks, and many other graph-related problems.

---

## 🧠 Summary

- Visits nodes in layers: first the root, then its neighbors, then their neighbors, etc.
- Utilizes a **queue** data structure (FIFO) to track the next node to visit.
- Guarantees the shortest path (fewest edges) in unweighted graphs.

```
function BFS(graph, start):
    create empty set visited
    create queue
    enqueue start
    add start to visited

    while queue is not empty:
        node = dequeue from queue
        process(node)

        for each neighbor in graph[node]:
            if neighbor not in visited:
                add neighbor to visited
                enqueue neighbor
```

---

## ⚙️ Key Properties

| Property                   | Value                              |
|----------------------------|------------------------------------|
| Time complexity             | O(V + E) (V = vertices, E = edges) |
| Space complexity            | O(V) for storing visited and queue |
| Suitable for                | Unweighted graphs, shortest paths, connected components |
| Data structure used         | Queue                             |

---

## 🏁 Applications

- Finding the shortest path in unweighted graphs
- Connected component analysis
- Peer-to-peer networks (e.g., finding nearest node)
- Web crawlers
- Level-order traversal of trees
- Network broadcasting / flooding

---

## 🚀 Strengths

- Simple and easy to implement.
- Always finds shortest path in unweighted graphs.
- Works on both trees and general graphs (including disconnected).

---

## ⚠️ Weaknesses

- Can consume large amounts of memory in wide graphs.
- Not suitable for weighted graphs (see [[Dijkstra's Algorithm]]).

---

## 🔄 Comparison to Depth-First Search (DFS)

| Aspect            | BFS                    | [[DFS]]                  |
|-------------------|------------------------|--------------------------|
| Order of traversal | Level-by-level          | Depth-first (explores deeper first) |
| Memory use         | Higher in wide graphs   | Lower in wide graphs     |
| Finds shortest path | ✅ Yes (unweighted)     | ❌ No                    |
| Data structure     | Queue                   | Stack (or recursion)     |

---

## 🔗 Related Notes

- [[Graph Algorithms]]
- [[Dijkstra's Algorithm]]
- [[Depth First Search]]
- [[Connected Components]]
- [[Minimum Spanning Tree]]

---

## 🌐 External References

- [Wikipedia - Breadth-First Search](https://en.wikipedia.org/wiki/Breadth-first_search)
- [CP-Algorithms - BFS](https://cp-algorithms.com/graph/breadth-first-search.html)

---
