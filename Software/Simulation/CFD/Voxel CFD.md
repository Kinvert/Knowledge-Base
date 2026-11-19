# Voxel CFD

`Voxel CFD` is a workflow where flow is solved on a structured Cartesian grid, and geometry is represented by removing solid voxels from a fluid domain rather than building a conformal mesh.  
The core idea is exactly your loop: *geometry generation → boolean subtraction from an air box → voxel or signed-distance field conversion → fast CFD solve → objective extraction → model update*.

---

## Why this matches AI design loops

For external incompressible screening, this approach can collapse end-to-end iteration latency:

- No body-fitted meshing step for every candidate.
- Geometry changes are just diffs in a regular lattice field, which is cheaper to update than topology-remeshing an unstructured mesh.
- Solver kernels become embarrassingly regular (local stencils, predictable memory patterns, GPU-friendly index math).

This is why many fast-loop papers frame it as a geometry-preprocessing bottleneck replacement strategy: they focus on extracting robust geometry data on Cartesian grids cheaply before solving the PDEs (for example, via fast signed-distance or voxel generators).

[[XLB]] and [[FluidX3D]] are representative structured-grid LBM branches that align well with this loop style.

---

## What “subtracting a body from air volume” means in practice

In this style, you define:

- A bounding box or computational volume `V_air`.
- A closed solid object/assembly mesh `S`.
- A scalar field `phi(x)` that is negative in solid, positive in air, and zero on the interface.
- Then the active fluid mask is `phi > 0`.

Common field choices:

- **Binary occupancy**: `1` for fluid, `0` for solid.
- **Signed distance field (SDF)**: distance to the nearest interface with sign (negative inside solid).
- **Volume fractions**: fraction of each cell occupied by fluid/solid.

Binary occupancy is fastest; SDF and volume fraction are usually more accurate near curved walls because they capture sub-cell boundary placement.

---

## White-paper/peer-reviewed context

Foundational references for this class of methods:

- **Cartesian grid + immersed/embedded geometry**
  - A Cartesian-grid projection method paper describes representing bodies as embedded interfaces with volume/area fractions on a regular grid and projection/correction for incompressible flow. [[SIAM J. Sci. Comput. (1997)](https://epubs.siam.org/doi/10.1137/S1064827594273730)]
- **AIAA/Cartesian grid method for complex moving or fixed objects**
  - The paper shows solid points are handled on a fixed Cartesian grid and boundary data is enforced on nearby interface points with second-order behavior in reported studies. [[AIAA Journal, 2005](https://doi.org/10.2514/1.8553)]
- **Conservative cut-cell methods**
  - Recent work compares multiple conservative immersed-boundary/cut-cell variants and notes small cut-cell stability/time-step pressure. [[Nidiye et al., IJHFF, 2025](https://doi.org/10.1016/j.ijheatfluidflow.2025.109775)]
- **Second-order conservative finite-volume cut-cell operators**
  - Conservative Cartesian cut-cell operators are shown for complex boundaries with first-order/accuracy restoration details. [[A Conservative Cartesian Cut Cell Method (2022)](https://arxiv.org/abs/2211.10698)]
- **Cut-cell treatment with local reconstruction**
  - A smaller 2D cut-cell immersed method for incompressible flow uses Shortley-Weller and local finite-volume conservation near obstacles. [[A 2D incompressible cut-cell method (2019)](https://arxiv.org/abs/1901.08303)]
- **Voxel-to-grid distance fields**
  - Geometry-to-SDF pipelines using GPU-accelerated scan conversion were developed for fluid-dynamics mesh generation and show practical preprocessing gains for complex triangulated surfaces. [[Fast Distance Fields for Fluid Dynamics Mesh Generation on Graphics Hardware (2019)](https://arxiv.org/abs/1903.00353)]
- **Surface voxelization + LBM readiness**
  - A parallel voxelization pipeline describes mapping tessellated triangles to structured grids for LBM-style CFD stages. [[Fast and Rigorously Parallel Surface Voxelization Technique (2025)](https://www.global-sci.com/index.php/cicp/article/view/7565)]

For direct-use workflows in common open-source ecosystems:
- IBM-style discretizations in OpenFOAM are explicitly implemented on a background Cartesian mesh and avoid full body-fitted meshing in that implementation style. [[Implementation of a discrete IBM in OpenFOAM, arXiv](https://arxiv.org/abs/1609.04364)]
- A more recent OpenFOAM variant adds level-set/volume-penalization coupling with mesh refinement near geometry. [[Implementation of a level-set based VPM method, arXiv](https://arxiv.org/abs/2309.08882)]

---

## Geometry-to-voxel algorithm stack (important for your loop)

### 1) Geometry ingestion

Import STEP/Parasolid/mesh-derived geometry, then:

- Repair and normalize topology.
- Scale/axis-align into solver coordinate conventions.
- Convert to triangle surface representation if needed (`.stl` is still common for voxel kernels).

### 2) Voxel / SDF generation

- Rasterize triangle surface to a coarse or high-res uniform lattice.
- Run inside/outside and boundary-distance tagging.
- Optionally compute narrow-band SDF only near boundaries to reduce preprocessing cost.

The signed-distance route is especially useful for:

- sub-cell boundary positioning,
- implicit geometry operations,
- and coupling to cut-cell/immersed formulations.

### 3) Boundary strategy selection

Different ways to apply geometry to the flow equations:

- **Staircase / binary mask + no-slip at cell faces**  
  Fastest, least accurate for curved boundaries at coarse resolution.
- **Cut-cell method**  
  Trims boundary-crossed cells and uses modified geometric stencils for conservation and sharper boundaries.
- **Immersed boundary forcing**  
  Keeps grid fixed and enforces no-slip through forcing terms.
- **Volume-of-solid / level-set coupling**  
  Uses continuous occupancy/SDF to blend boundary conditions in a force or penalty formulation.

### 4) Solver + objective pass

For screening, run fixed short horizon solves with:

- stabilized CFL,
- limited timestep budgets,
- strict residual cutoffs,
- metric extraction only.

Then promote top candidates to denser/high-fidelity meshed or hybrid runs.

---

## Comparison chart (speed/accuracy/robustness triage)

| Method | Boundary fidelity | Preprocessing cost | Memory/computation | Typical wall-clock for iteration loops | Best use in screening |
|---|---|---|---|---|---|
| Binary voxel + bounce-back | Low (stair-step walls) | Very low | Low | Fastest | Very rough ranking, early elimination |
| Binary+fraction-aware wall reconstruction | Medium | Low | Low | Fast | External drag/lift trends, coarse ranking |
| Cut-cell (conservative) | Medium-high (geometry-aware) | Medium | Medium | Medium | Better geometric fidelity at moderate resolution |
| Immersed boundary forcing | Medium-high | Medium | Medium | Medium-fast | moving/complex geometry where remeshing is bad |
| VoS / VOF-style solid fraction coupling | Medium-high | Medium | Medium-high | Medium | when near-wall gradients are important, still voxel-friendly |
| SDF + cut-cell hybrid | High | Medium-high | Medium-high | Slower | Better ranking than pure binary at same resolution |
| Body-fitted mesh (finite-volume/FEM) | Highest | High | High | Slowest | validation / final ranking only |

The key practical pattern:
- for AI search: start with binary or fraction-aware methods,
- then refine top designs with cut-cell or body-fitted methods,
- reserve finest physics for last-stage confirmation.

---

## Why not always use voxel CFD (and still why you should for now)

### Advantages for your use case

- High iteration speed from cheap geometry updates.
- Easy caching of intermediate voxel fields (hash by rule seed + bounding box + resolution).
- Natural fit for GPU kernels and batched candidate evaluation.

### Failure modes to expect

- Curvature and wall shear sensitivity can be under-resolved.
- Thin features may vanish after rasterization at low resolution.
- Staircase artifacts can bias force coefficients if near-wall layers are under-resolved.
- Moving geometry changes can create aliasing in occupancy unless re-rasterized with margin logic.

Mitigation:

- maintain adaptive or AMR-like grids where needed,
- compare candidate winners across multiple resolutions,
- feed high-gradient metrics into a secondary high-fidelity pass.

---

## Data handling details for fast pipelines

For large candidate batches, byte-level treatment matters:

- Use bit-packed masks for occupancy (`0/1`) in hot paths.
- Store sparse fields (SDF/velocity) in chunked blocks so only near-body regions expand.
- Use metadata like bounding boxes, hashes, and coarse hashes of geometry signatures to deduplicate repeated candidates.
- Prefer brick/voxel-page layouts (or sparse trees) to speed neighborhood access and neighbor cache misses.

OpenVDB and NanoVDB exist specifically for sparse voxel hierarchies and fast access patterns in volumetric workflows; this is often the difference between “toy-scale” and “production-scale” voxel preprocessing. [[OpenVDB FAQ](https://www.openvdb.org/documentation/doxygen/faq.html), [NanoVDB](https://research.nvidia.com/labs/prl/nanovdb/nanovdb2021.pdf)]

NASA LAVA’s cartesian input model also uses immersed-boundary geometry blocks and octree AMR-style box refinement for efficient focused resolution. [[NASA LAVA cartesian docs](https://www.nas.nasa.gov/LAVA/cart_docs/input_file_cartesian/)]

---

## Suggested implementation pattern (your proposed loop)

1. Generate CAD from parameter rules.
2. Reuse stable body hash and bounding extents.
3. Rasterize to `N0` resolution binary field in parallel.
4. Optionally compute narrow-band SDF within `k` cells of surfaces.
5. Run fast screen-solve on GPU (short transient/incremental objective).
6. Keep top-N candidates and rerun at:
   - higher voxel resolution,
   - cut-cell or level-set boundary treatment,
   - then final high-fidelity body-fitted meshing.

Use this to gate expensive meshed CFD and preserve wall-clock budget.

---

## Related notes

- [[Voxel Grid]]
- [[Lattice Boltzmann Method]]
- [[OpenFOAM]]
- [[XLB]]
- [[FluidX3D]]
- [[AMGX]]

---

## External references

- https://arxiv.org/abs/2211.10698
- https://arxiv.org/abs/1901.08303
- https://arxiv.org/abs/1503.04544
- https://arxiv.org/abs/1903.00353
- https://www.global-sci.com/index.php/cicp/article/view/7565
- https://www.openvdb.org/documentation/doxygen/faq.html
- https://research.nvidia.com/labs/prl/nanovdb/nanovdb2021.pdf
- https://www.nas.nasa.gov/LAVA/cart_docs/input_file_cartesian/
- https://epubs.siam.org/doi/10.1137/S1064827594273730
- https://doi.org/10.2514/1.8553
- https://doi.org/10.1016/j.ijheatfluidflow.2025.109775
- https://arxiv.org/abs/1609.04364
- https://arxiv.org/abs/2309.08882
- https://graphics.stanford.edu/courses/cs348a-21-winter/Papers/Marching_Cubes.pdf
