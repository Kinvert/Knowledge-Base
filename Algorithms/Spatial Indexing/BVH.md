---
title: BVH
aliases:
  - Bounding Volume Hierarchy
  - BVH Tree
tags:
  - spatial-indexing
  - geometry
  - acceleration-structure
  - simulation
  - computer-graphics
---

# BVH (Bounding Volume Hierarchy) 🌲

**BVH** is a tree of nested spatial bounds used to prune large parts of geometric queries.

In practice, BVHs are used where you have lots of geometry queries (ray casting, broad-phase collision, proximity), and testing every primitive would be too expensive.

---

## Overview

- **Node:** stores a bounding volume (usually `AABB`) that encloses child primitives.
- **Internal node:** groups two or more child nodes whose bounds are tightly merged.
- **Leaf:** stores actual geometry references (triangles, boxes, spheres, points, volumes).
- **Goal:** reduce query cost by skipping whole subtrees when the query and node bound do not intersect.

For long-horizon perception training or synthetic sensor work, this is a core optimization because the same scene is queried repeatedly (many rays, lidar beams, occlusion probes) per step.

---

## How BVHs are built

Two common construction styles:

- **Top-down (recursive split)**
  - Pick a split axis (often largest box extent).
  - Partition primitives into two child sets.
  - Recurse until leaf threshold is met.
  - Better quality usually, but rebuild/update can be heavier.

- **Bottom-up (agglomerative)**
  - Merge closest pairs or groups into parent nodes.
  - Easier to reason about in some offline pipelines.
  - Can be slower to build, often used less in online systems.

Typical split policies:

- **Median split** (balanced tree): stable build cost, easy to implement.
- **SAH (Surface Area Heuristic):** better traversal cost, higher build cost.
- **Axis alternation** (fixed axis cycling): simple but less robust for skewed geometry.

Leaf size is a key tuning knob:

- Too small: deeper tree, more traversal overhead.
- Too large: less pruning, more triangle-level tests.

---

## Query behavior

BVH traversal is branch-and-bound:

1. Test the query against node bound.
2. If miss, skip the whole subtree.
3. If hit and node is leaf, test child primitives.
4. If hit and internal, recurse into children (near-child-first ordering often helps).

### Typical queries

- **Ray intersection:** return first/closest hit, all hits, occlusion yes/no.
- **AABB overlap / frustum intersection:** prune many triangles quickly.
- **Closest point / nearest neighbor:** approximate nearest with bounds and backtracking.
- **Temporal broad-phase:** compare moving bounds or updates between frames.

---

## Variants you will encounter

- **AABB-BVH:** common for dynamic scenes and general collision/raycast.
- **OBB/BV variants:** tighter fit in some cases, harder overlap tests.
- **Binary BVH (bvh2):** 2-child nodes, popular in real-time engines.
- **Wide BVH (4/8/16 nodes):** trades memory for SIMD-friendly traversal.
- **LBVH (Linear BVH):** built from Morton/Z-order keys, good for rebuild-heavy workloads.
- **Refit vs rebuild:** refit updates bounds for moving geometry; rebuild recreates topology.

Rule of thumb:
- Use **refit** for small local deformations where topology is mostly valid.
- Rebuild when topology quality degrades (increased overlap/inefficient traversal).

---

## Why it matters for vision/audio sensory simulation

- **Vision:** per-pixel or per-sample raycasting for depth, segmentation, and synthetic lidar.
- **Lidar:** each beam query can early-exit on hit; BVH cuts per-beam candidate checks from O(N) to near-logarithmic behavior.
- **Audio acoustics proxies:** obstacle visibility/occlusion approximations are usually ray-style queries over room surfaces; BVH makes this affordable at high sample counts.
- **Proprioception contacts:** fast contact candidate generation before narrow-phase checks.

For synthetic robotics stacks, this is the reason BVH is a default primitive in high-throughput render/physics pipelines.

---

## GPU implications

- Traversal is pointer-chasing heavy; memory layout matters.
- SoA (structure of arrays) and compact node packing improve cache behavior.
- Wide BVHs pair better with SIMT/SIMD traversal than deep binary recursion.
- On NVIDIA GPUs, BVH-style queries are often mixed with custom kernels and graph-captured rollouts for fixed pipelines.

In practice, the biggest gains come from:
- stable topology across frames,
- coherent rays (same origin/order),
- and keeping BVH build/update on-device when possible.

---

## Complexity (practical, not theoretical)

- **Build:** from O(N log N) to O(N log² N) depending on split heuristic and sorting strategy.
- **Query:** average near O(log N), worst-case O(N) for pathological layouts.
- **Update:** refit is cheap per-frame; rebuild cost can dominate for complex deforming meshes.
- **Memory:** typically a few extra floats/ints per node plus primitive index storage.

The worst-case pathologies matter more than averages:
- long thin triangles,
- high overlap between sibling bounds,
- dynamic scenes with poor update quality.

---

## Code sketch (conceptual)

```text
build(node):
  if primitives <= leaf_threshold:
     return leaf(node, primitives)
  split_axis = choose_axis(primitives)
  left_set, right_set = split(primitives, split_axis)
  node.left = build(left_set)
  node.right = build(right_set)
  node.bounds = union(node.left.bounds, node.right.bounds)
  return node

intersect(node, ray):
  if not overlap(node.bounds, ray):
     return miss
  if node is leaf:
     test primitives in node
  else:
     hit_left = intersect(node.left, ray)
     hit_right = intersect(node.right, ray)
     return nearest(hit_left, hit_right)
```

---

## Comparison chart

| Structure | Build time | Query latency | Dynamic updates | Typical use | Notes |
|---|---:|---:|---|---|---|
| BVH (AABB) | Medium | Very good (pruned traversal) | Medium (refit/rebuild) | Ray tracing, collision, visibility | Best general pick for sparse triangle geometry |
| Uniform grid | Low | Good if density uniform | High | Particle, dense occupancy, simple scenes | Degenerates with uneven density |
| Octree | Medium | Good | Medium | Voxelized maps, sparse 3D data | Great for occupancy, less ideal for arbitrary triangles |
| KD-tree | Medium-high | Good in low dims | Poor | NN queries, static point clouds | Insert/delete expensive compared to BVH |
| R-tree | Medium | Good | High | Spatial DB, bounding boxes/2D GIS | Not as common for triangles in graphics |
| BSP tree | Medium-high | Very good for split-plane visibility | Low | Engine culling/visibility in games | Plane choices can be unstable for dynamic data |

For triangle-heavy simulation, BVH and BVH-like traversal are usually the first fallback before custom acceleration is worth building.

---

## Strengths

- Great general-purpose acceleration for arbitrary geometric primitives.
- Excellent pruning for repeated ray/overlap workloads.
- Works well with mixed query mixes (visibility + collision + distance).
- Can be GPU-accelerated and serialized/reused across frames.

## Weaknesses

- Sensitive to build strategy and scene distribution.
- Can collapse in highly dynamic or highly overlapping scenes.
- Memory overhead and indirection compared to flat arrays/grids.
- Worst-case pathological cases still exist.

---

## Practical checklist

- Start with AABB-BVH with SAH-like split heuristics for geometry-heavy workloads.
- Keep leaf size small enough to prune, large enough to amortize primitive tests.
- Profile rebuild vs refit cadence for your motion profile.
- For sensor workloads, test coherent vs incoherent ray batches separately.
- On GPU, prioritize compact node layout and coherent launch order.

---

## 🧪 Hello-world / starter tutorials

- **Core BVH references**
  - https://www.pbr-book.org/4ed/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies
  - https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing/ray-tracing-bounding-volume-hierarchies
- **Implementation-focused learning**
  - https://www.pbr-book.org/4ed/Primitives_and_Intersection_Acceleration/A_Bounding_Volume_Hierarchy_Accelerator
  - https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing/rendering-pipeline-radiosity
- **GPU and sensory pipelines**
  - https://nvidia.github.io/warp/stable/user_guide/mesh.html
  - https://nvidia.github.io/warp/stable/user_guide/geometry.html
  - https://github.com/NVIDIA/warp/tree/main/warp/examples/core/example_raycast.py
  - https://github.com/NVIDIA/warp/tree/main/warp/examples/core/example_marching_cubes.py
- **Alternative production references**
  - https://www.embree.org/overview.html
  - https://github.com/mmp/pbrt-v4/blob/master/src/accelerators/bvh.h

---

## Use and ecosystems

- Physics engines: collision broad-phase acceleration.
- Rendering engines: primary ray and visibility acceleration.
- Simulation frameworks: sensor models that cast many rays.
- Robotics perception: lidar/vision/occlusion simulation at scale.
- ML stacks: high-throughput synthetic data generation when geometry is static or mostly rigid.

---

## Related notes

- [[Spatial Indexing Algorithms]]
- [[Z-order]]
- [[Octree]]
- [[KDTree]]
- [[NVIDIA Warp]]
- [[MuJoCo Warp]]
- [[APIC Graph]]

## External resources

- Bounding volume hierarchy references in classic rendering literature (PBRT and practical BVH guides).
- NVIDIA Warp spatial primitives docs (BVH + mesh/raycast workflows).
- NVIDIA Warp examples include BVH-adjacent acceleration workflows (`example_raycast.py`, etc.).
