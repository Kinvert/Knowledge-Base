# Z-order curve (Morton order / Morton code) 🧭

Z-order (a.k.a. Morton order or Morton code) is a space-filling curve that maps multidimensional coordinates to one dimension by bit-interleaving the coordinate components. It preserves *locality* (nearby points in N-D space map to nearby 1-D keys) in a simple, fast way and is widely used in databases, graphics, caches, spatial indexing, and robotics (KD-trees, octrees, occupancy grids, point clouds, etc.). This note focuses on the concepts, variants (2D/3D), practical encodings (Morton), comparisons (Hilbert and others), uses, strengths/weaknesses, and implementation tips for engineers.

---

## 🔎 Overview

- **Name(s):** Z-order, Morton order, Morton code, Morton-encoded array  
- **Core idea:** Interleave bits of coordinates (e.g., `x`, `y`, `z`) into a single integer so multi-dimensional space can be traversed, indexed, and compared with one numeric key.  
- **Dimensionality:** 1D mapping of 2D, 3D (or higher) coordinates. Common: 2D (quadtrees) and 3D (octrees).  
- **Locality:** Good but not optimal — nearby points usually map to nearby keys, with occasional discontinuities at coarse boundaries.  
- **Complexity:** Very fast encode/decode via bit masks and shift operations (constant time per bit-word). Easily parallelizable and GPU/CPU friendly.

---

## ✅ Summary

- Morton order converts N-D coordinates to 1-D by *bit interleaving*.  
- It’s extremely cheap computationally (bit ops) and amenable to hardware acceleration.  
- It’s widely used where fast mapping between spatial positions and linear memory/index is needed (tile storage, GPU texture layout, octrees, spatial DBs).  
- Compared to Hilbert curve it’s simpler and faster but provides slightly worse locality at some scales.

---

## 🧠 Core concepts

- **Bit interleaving (bit dilation):** For 2D, take bits of `x` and `y` and weave them: `x0 y0 x1 y1 x2 y2 ...`. For 3D, interleave `x y z` bits: `x0 y0 z0 x1 y1 z1 ...`.  
- **Morton code:** The resulting integer from interleaving is the Morton code (Z-index).  
- **Morton order traversal:** Sorting by Morton code gives the Z-order traversal of the grid/cells.  
- **Hierarchy mapping:** Z-order maps naturally to quadtrees (2D) and octrees (3D): node ID = prefix of Morton code.  
- **Locality property:** Preserves locality within blocks (power-of-two aligned regions) well; crossing block boundaries produces discontinuities.  
- **Endian/bit-order note:** Implementation-dependent: define whether `x` is MSB or LSB in interleaving (common convention: `x` as most significant interleaved bit for each bit position, but consistent definition is critical).

---

## 🛠 How it works (concept + pseudo code as an indented bullet list)

- **High level:** for each bit position `i` (0..Nbits-1) take `xi`, `yi`, `zi` and append them in order to the result word.
- **2D encode (conceptual):**
  - take bit `i` from `x` -> append
  - take bit `i` from `y` -> append
- **3D encode (conceptual):**
  - take bit `i` from `x` -> append
  - take bit `i` from `y` -> append
  - take bit `i` from `z` -> append
- **Morton encode (3-bit example for cube corners):**
  - Coordinates each are 1 bit (0 or 1): `x y z`. If `x` is most significant interleaved bit and `z` least:
  - `x y z` -> bit pattern `xyz` -> integer 0..7. This yields a sweep of all 8 cube corners using a 3-bit address.
- **Pseudo code (very compact):**
  -  `MortonEncode3D(x,y,z, nbits):`
     -  For `i` in `0..nbits-1`:
        -  append bit `(x >> i) & 1`
        -  append bit `(y >> i) & 1`
        -  append bit `(z >> i) & 1`
     -  return assembled bits (LSB-first or MSB-first depending on convention)
  -  `MortonDecode3D(code):`
     -  For `i` in `0..nbits-1`:
        -  `xi = (code >> (3*i + shift_x)) & 1` and similarly for `y`,`z`
        -  assemble `x |= xi << i` etc.
- **Bit tricks for speed:** "dilate" bits: expand a 10-bit integer into 30 bits separated by two zeros using mask/shift patterns (aka `part1by2`), then combine dilated `x`, `y`, `z`.

---

## 🔢 Concrete 3D corner mapping (Morton 3-bit example)

If we treat the single-bit coordinates `x y z` (x = MSB of the triplet, z = LSB):

| Morton code (binary) | Morton code (dec) | (x, y, z) |
|---:|---:|---|
| `000` | 0 | (0,0,0) |
| `001` | 1 | (0,0,1) |
| `010` | 2 | (0,1,0) |
| `011` | 3 | (0,1,1) |
| `100` | 4 | (1,0,0) |
| `101` | 5 | (1,0,1) |
| `110` | 6 | (1,1,0) |
| `111` | 7 | (1,1,1) |

This is the "sweep" of cube corners by Morton index with `x` as most significant interleaved bit and `z` least — a fast way to address every corner using a 3-bit address.

---

## 🧾 Comparison Chart — Z-order (Morton) vs Hilbert vs Others

| Feature | Z-order (Morton) | Hilbert curve | Row/Column-major | Gray-code variants |
|---|---|---|---|---|
| Ease of implementation | Very simple — bit interleave | More complex recursion/bit ops | Trivial | Moderate |
| Computational cost | Very low (bit masks/shifts) | Moderate (bit twiddling or table) | Lowest | Low |
| Locality preservation | Good in blocks; poor across some boundaries | Better global locality | Poor for spatial locality | Varies |
| Hierarchy / quadtree/octree tie-in | Natural (prefix = node) | Possible but trickier | Not hierarchical | Possible |
| Sorting / range queries | Fast, natural; ranges correspond to subtree intervals | Ranges are more complex | Not suited | Complex |
| GPU/parallel friendly | Excellent | Less straightforward | Yes | Depends |
| Use in databases / indexing | Common (spatial indices) | Used (when locality critical) | Used for simple layouts | Special use cases |
| Bitwise friendly | Yes (bit interleave) | Requires mapping algorithms | No | Yes (for Gray ordering) |

Notes:
- Hilbert gives superior locality (less long jumps) — useful for nearest-neighbor tasks where locality is paramount — but is more expensive and less directly hierarchical (prefixes don't map to simple quadtree nodes without additional mapping).
- Morton is the go-to when performance and simplicity matter, or when you want tight integration with quad/octrees.

---

## ⚙️ Variants & related curves

- **Morton 2D:** interleave `x` and `y` -> used with quadtrees, image tiling, mipmapping.  
- **Morton 3D:** interleave `x, y, z` -> used with octrees, voxel grids, point cloud indexing.  
- **Hilbert curve:** better compactness/locality—used in some DBs, clustering, NN caching.  
- **Peano / Lebesgue / Gray-code variants:** alternative space-filling curves with various locality/ordering tradeoffs.  
- **Row-major / column-major:** simple linearizations for arrays, poor spatial locality for 2D/3D neighborhoods.  
- **Z-ordering + blocking (tile layout):** combine Morton ordering with tile sizes to reduce cache misses for block processing.

---

## 🧩 Use Cases (lots)

- **Spatial indexing / databases:** fast range queries on geospatial or multi-dimensional data, e.g., RDBMS spatial indexing, geohashes alternatives.  
- **Quadtrees / Octrees:** node IDs and traversal order; prefix of Morton code = ancestor node.  
- **Image/texture memory layout:** texture atlases, cache-friendly traversal for rendering, tiled storage for GPUs.  
- **Voxel engines & point clouds:** fast indexing, neighbor searches, streaming LOD (level of detail).  
- **Cache-efficient algorithms:** reorder iteration to improve cache locality for stencil ops, convolution, or streaming.  
- **GPU parallelism & compute shaders:** map threads/tiles to Morton keys for better locality and coalesced memory access.  
- **Collision detection & broad-phase:** partitioning world into Morton buckets for quick candidate lookups.  
- **Compression & serialization:** compress monotonic ranges or use run-length encoding on Morton-sorted data.  
- **Robotics:** occupancy grid mapping, fast map lookups, octree based SLAM backends and storage.  
- **Distributed storage / sharding:** split space by Morton intervals to partition data across servers.

---

## 🔧 Strengths

- Extremely fast encode/decode via bit operations.  
- Very simple to implement and test.  
- Natural mapping to quadtrees and octrees — prefix semantics make hierarchical operations straightforward.  
- Good locality for many practical problems and power-of-two aligned regions.  
- Hardware/GPU friendly; easily vectorized.

---

## ↘️ Weaknesses

- Locality is not optimal — Hilbert often better for nearest-neighbor locality.  
- Adjacent 1-D keys can correspond to points that are not geometrically adjacent if they cross Z boundaries.  
- Performance assumes coordinates are integer (or quantized) and power-of-two grid — otherwise you must quantize/scale.  
- For arbitrary floating coordinates you must decide quantization/scale carefully to avoid collisions or loss of precision.

---

## 📌 Practical implementation tips

- **Quantize coordinates:** map continuous coordinates to integer grid (e.g., fixed point) before interleaving. Choose grid resolution according to application precision needs.  
- **Endian/bit-order:** pick a convention (LSB-first or MSB-first) and stick to it across systems. Document it.  
- **Dilation masks:** use known bit-dilation patterns (`part1by1`, `part1by2`) to expand bits quickly via sequence of shifts and masks instead of bit-looping.  
- **Hardware intrinsics:** some CPUs/GPUs have bit-shuffle instructions; use them if available for speed.  
- **Range queries:** decompose spatial range into a set of Morton intervals — bounding boxes can map to several disjoint Morton ranges; use recursive subdivision to generate minimal intervals.  
- **Cache blocking:** combine Morton order with block sizes to further improve cache behavior for block operations.  
- **Sorting large data:** stable integer sort on Morton codes is fast (radix sort).  
- **Memory layout:** store keys alongside data or use key as array index for dense grids (e.g., `data[morton_key]`).

---

## 📈 Performance & implementation notes

- **Encode complexity:** O(1) per word (bit ops); loop over machine word size when simpler.  
- **Decode complexity:** symmetrical to encode — extract interleaved bits.  
- **Sorting:** integer radix sort on Morton keys is cache-efficient and often faster than multi-dimensional sorts.  
- **Prefix operations:** finding all points in a subtree (quadtree node) = range of Morton codes sharing a prefix; use bit masks to isolate ranges.

---

## 🧪 Examples (short inline snippets / reasoning — no multi-line code blocks)

- **Single-bit cube corners:** `code = (x << 2) | (y << 1) | z` for 3-bit ordering (x MSB, z LSB).  
- **2D 1-bit example:** `code = (x << 1) | y` produces 2-bit Morton index with `x` as higher interleaved bit.  
- **Dilation idea (concept):** `dilated = part1by2(x)` produces bits separated by two zeros; final code `dilated_x | (part1by2(y) << 1) | (part1by2(z) << 2)`.

---

## ⚖️ When to choose Morton vs Hilbert (rule of thumb)

- Choose **Morton** when: speed, simplicity, hierarchical prefix semantics, and hardware friendliness are top priorities.  
- Choose **Hilbert** when: you need the best possible locality for nearest neighbor tasks and can accept added complexity.  
- In many systems, Morton is "good enough" and wins for throughput-critical paths.

---

## 📚 Related Concepts / Notes (linkable for Obsidian)

- [[Quadtrees]] (2D hierarchical spatial partitioning)  
- [[Octrees]] (3D hierarchical partitioning)  
- [[Hilbert Curve]] (space-filling curve with stronger locality)  
- [[Spatial Indexing]] (R-trees, KD-trees, etc.)  
- [[Radix Sort]] (integer sort often used with Morton keys)  
- [[Point Cloud]] (voxelization and indexing)  
- [[SLAM]] (map structures using octrees/occupancy grids)  
- [[Texture Tiling]] (GPU texture layout strategies)  
- [[Geohash]] (alternative spatial hashing)  
- [[Fixed Point Arithmetic]] (quantization for Morton mapping)

---

## 🔗 Compatible Items

- Octree libraries (use Morton keys as node ids)  
- Spatial DBs (Morton indexing as secondary index)  
- GPUs/compute shaders (thread mapping via Morton)  
- Cache-aware algorithms and blocked matrix operations

---

## 🧰 Developer Tools & Libraries (implementation pointers)

- Look for `morton`, `zorder`, `libmorton` libraries (C/C++/Rust/Python) — they usually provide `encode/decode` and `range` helpers.  
- Example intrinsics: CPU `BMI`/`BMI2` bit instructions may accelerate masks/shifts.  
- Use `radix sort` libraries for sorting large Morton keyed arrays.

---

## 📖 External Resources & Further Reading

- Classic papers and blog posts on Morton order and Hilbert curve (search for "Morton order", "Z-order curve", "Hilbert curve locality").  
- `libmorton` repos and SIMD-accelerated implementations for concrete fast examples.  
- DBMS documentation for spatial indexes that mention Morton/Hilbert alternatives.

---

## 🔍 Key features / Highlights

- Fast bitwise mapping from N-D to 1-D.  
- Natural hierarchical ID for quad/octor trees.  
- Good locality, excellent speed, trivial parallelization.

---

## ⚙️ Variants & Extensions

- **Signed/offset coordinates:** map coordinates with offsets; encode `(x - minX)` etc.  
- **Non-power-of-two grids:** quantize to nearest power-of-two or handle remainder buckets.  
- **Higher dimensions:** same bit interleaving idea extends beyond 3D (e.g., 4D spatio-temporal indexing).  
- **Combined keys:** interleave spatial with time or other attributes to build composite Morton keys.

---

## 🧾 Final tips

- Always document your bit-order convention.  
- For floating data, choose quantization carefully and keep track of resolution vs collisions.  
- When performance critical, benchmark Morton vs Hilbert for your dataset: sometimes the locality difference matters, sometimes raw throughput dominates.

