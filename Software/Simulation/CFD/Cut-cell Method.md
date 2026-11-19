# Cut-cell Method

The **cut-cell method** keeps a background structured mesh but modifies cells intersected by a solid surface using partial volume and face-area fractions. It is a compromise between body-fitted fidelity and fixed-grid speed.

---

## 🧭 Core idea

Each background cell can be:
- fully fluid,
- fully solid,
- or partially filled by geometry.

For partial cells, geometry-aware coefficients replace plain Cartesian control-volume assumptions:
- reduced cell volumes,
- clipped face areas/face fluxes,
- corrected source terms at the interface.

That gives better wall location and normal force behavior than plain forcing-only approaches, while keeping a regular mesh backbone.

--- 

## 🔧 What must be handled

Implementation steps usually include:
- geometry-to-grid intersection, often via triangle-polygon and ray/scanline logic,
- robust area/volume fraction computation,
- stable treatment of very small fluid slivers (tiny-cell stabilization or merging),
- conservative flux correction for partially open faces,
- special handling for high curvature junctions and touching objects.

If tiny-cell instabilities are unmanaged, timestep limits can become stricter or the local solver becomes noisy.

--- 

## 🧠 Why this fits AI-first screening loops

- Better boundary fidelity than forcing-only immersed boundary at equal base cell counts.
- More geometry-aware pressure drag and wall-distance behavior.
- More expensive than ghost-cell/IBM setups, but cheaper than repeated full remeshing.
- Useful as a **mid-fidelity gate** before final verification mesh runs.

--- 

## 📚 Representative algorithms and variants

- Cartesian cut-cell / embedded boundary finite-volume approach (originally developed to preserve conservation on structured grids),
- Adaptive mesh overlays around complicated interfaces,
- Conservative small-cell mitigation (cell-merging or sub-timestep splitting),
- Level-set-driven geometry clipping feeding cut-fractions.

--- 

## 📊 Comparison chart

| Method | Geometry representation | Conservativeness | Small-feature behavior | Accuracy near boundary | Stability risk | Typical use |
|---|---|---:|---:|---:|---:|---|
| Cut-cell | Explicit partial cells (area/volume fractions) | High | Medium (needs mitigation) | High | Medium-high | Mid-to-high fidelity pre-screen |
| Ghost Cell | Virtual value reconstruction | Medium | Medium | Medium | Low | Ultra-fast candidate scanning |
| Immersed Boundary (forcing) | Distributed body force | Medium | Medium | Medium | Low-medium | High-motion / deforming-body problems |
| Immersed Boundary (continuous forcing) | Smoothed forcing kernel | Medium-high | Medium-high | Medium | Medium | Smooth interface motions |
| Body-fitted mesh | Conformal mesh faces | High | High | High | Medium | Final validation |
| Level-set CFD with re-distancing | Signed distance interface | Medium-high | High (with remesh) | Medium-high | Medium | Multi-physics interface problems |

--- 

## ✅ When to choose cut-cell over alternatives

- Choose cut-cell when geometry details are starting to dominate error budget and remeshing all candidates is too expensive.
- Choose ghost-cell or forcing IBM when throughput matters more than strict boundary gradients.
- Choose body-fitted or high-order overset when final report-quality drag/lift and separation topology are required.

--- 

## 📖 Source links

- Development of Cartesian cut-cell methods (SIAM style overview and core method): https://doi.org/10.1016/S0378-4754(02)00107-6
- Publication metadata and abstract summary: https://www.research.ed.ac.uk/en/publications/developments-in-cartesian-cut-cell-methods/
- Review and small-cell behavior context (with conservative formulation discussion): https://ris.utwente.nl/ws/portalfiles/portal/299026392/annurev_fluid_120720_022129.pdf
- Open paper with cut-cell context and comparative notes: https://arxiv.org/abs/1806.01721
- Open source/benchmark context: https://ibamr.github.io/

--- 

## 🔗 Related notes

- [[Ghost Cell Method]]
- [[Immersed Boundary Method]]
- [[Voxel CFD]]
- [[Lattice Boltzmann Method]]
- [[OpenFOAM]]
- [[CFD Solvers]]
