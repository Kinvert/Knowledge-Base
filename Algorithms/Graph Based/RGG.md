# 🌐 RGG (Random Geometric Graph)

**Random Geometric Graph (RGG)** is a type of probabilistic graph used extensively in sampling-based motion planning. In an RGG, nodes are placed randomly in a metric space, and edges are added between nodes that are within a certain distance (or satisfy a connection criterion). Many modern path planners like PRM, FMT*, and BIT* rely on RGGs as the underlying structure for connecting samples.

---

## 🧠 Summary

- Nodes: Randomly distributed samples in a configuration space.
- Edges: Created between nodes that are within a fixed connection radius.
- Often used in planning algorithms for creating a roadmap or connection graph.
- Theoretical foundations ensure probabilistic completeness and (in some cases) asymptotic optimality when the connection radius is chosen appropriately.

---

## ⚙️ Key Features

- Simple to implement and generalize to high-dimensional spaces.
- Captures the connectivity of free space with high probability as sample count increases.
- Basis for planners like PRM, PRM*, FMT*, BIT*, and other graph-based methods.

---

## 🚀 Applications

- Underpins roadmap generation in PRM and PRM*.
- Used in batch planners like FMT* to define connectivity.
- Supports informed sampling and batch processing in BIT*.
- General modeling of connectivity in random networks.

---

## 🏆 Strengths

- Strong theoretical guarantees (probabilistic completeness, asymptotic optimality when used properly).
- Scales reasonably with dimension (relative to grid-based methods).
- Simple construction.

---

## ⚠️ Weaknesses

- Requires careful tuning of connection radius for good performance.
- No path smoothing — requires separate refinement step in motion planning.
- Large number of nodes/edges may result in high memory consumption for dense graphs.

---

## 📊 Comparison with Related Concepts

| Structure  | Type        | Deterministic | Supports Optimality | Use Case                   |
|------------|-------------|---------------|--------------------|----------------------------|
| RGG        | Probabilistic | ❌             | ✅ (if parameters set properly) | Roadmap/connectivity graph for sampling planners |
| Grid Graph | Deterministic | ✅             | ✅ (on grid)         | A*, Dijkstra on discrete space |
| Visibility Graph | Deterministic | ✅             | ✅                    | Exact shortest paths in polygonal environments |
| K-Nearest Neighbor Graph | Probabilistic | ❌             | ✅                    | Alternative to RGG for roadmap connections |

---

## 🔗 Related Notes

- [[PRM]]
- [[PRM*]]
- [[FMT*]]
- [[BIT*]]
- [[Path Planning]]
- [[Sampling-based Planning]]

---

## 🌐 External References

- [RGG in Motion Planning (LaValle)](http://planning.cs.uiuc.edu/node233.html)
- [Random Geometric Graphs (Wikipedia)](https://en.wikipedia.org/wiki/Random_geometric_graph)

---
