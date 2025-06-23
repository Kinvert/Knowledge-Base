# 🌳 Lowest Common Ancestor (LCA) Algorithm

The **Lowest Common Ancestor (LCA)** of two nodes in a tree is the deepest (i.e., farthest from the root) node that is an ancestor of both nodes. LCA algorithms are widely used in computational biology, file systems, network routing, and many graph-related problems.

---

## 🧠 Summary

- **Input:** A rooted tree (often represented as a graph) and two nodes `u` and `v`.
- **Output:** The lowest node in the tree that has both `u` and `v` as descendants (where a node can be a descendant of itself).

---

## ⚙️ Common Algorithms

| Algorithm                   | Time Complexity | Preprocessing | Space Complexity | Notes                                        |
|-----------------------------|----------------|---------------|-----------------|----------------------------------------------|
| Naive upward traversal       | O(h) per query  | None          | O(1)             | Simple but slow for deep trees (h = height) |
| Binary lifting               | O(log n)        | O(n log n)    | O(n log n)       | Efficient for static trees                  |
| Euler tour + RMQ (segment tree/sparse table) | O(1) query / O(n) query | O(n) / O(n log n) | O(n) / O(n log n) | Very fast query time with preprocessing     |
| Tarjan's offline LCA (union-find) | O(n + q α(n)) | O(n + q)      | O(n + q)         | Good for offline multiple queries           |

---

## 🚀 Applications

- Network routing
- Version control ancestry (e.g., Git merge base)
- File system hierarchy queries
- Phylogenetic trees (computational biology)
- [[Graph Algorithms]]

---

## 🏆 Strengths

- Fundamental building block for many hierarchical queries.
- Well-studied; multiple algorithms exist for different scenarios (online, offline, dynamic).

---

## ⚠️ Challenges

- Dynamic trees require more complex solutions (e.g., link-cut trees).
- Preprocessing may be costly for large trees with few queries.

---

## 🔗 Related Notes

- [[Graph Algorithms]]
- [[Tree Data Structures]]
- [[Binary Lifting]]
- [[Segment Trees]]
- [[Union-Find]]

---

## 🌐 External References

- [Wikipedia - Lowest Common Ancestor](https://en.wikipedia.org/wiki/Lowest_common_ancestor)
- [CP-Algorithms - LCA](https://cp-algorithms.com/graph/lca.html)

---
