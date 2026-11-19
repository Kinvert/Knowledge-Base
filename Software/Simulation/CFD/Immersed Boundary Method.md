# Immersed Boundary Method

The **immersed boundary method (IBM)** solves flow on a fixed grid while representing a solid body via forcing terms, marker points, or interface constraints. It avoids full remeshing for moving and changing geometry while maintaining compatibility with standard solvers.

---

## 🧭 Core idea

An Eulerian fluid grid is coupled to a boundary representation (often Lagrangian markers). Instead of deforming the mesh:
- fluid equations are solved on a simple background grid,
- interface conditions are imposed through coupling operators,
- body effects are transferred through regularized deltas, forcing kernels, or ghost-state constraints.

IBM is common in fluid-structure interaction, moving objects, and AI-driven shape loops where geometry changes frequently.

--- 

## 🧩 Major algorithm families

- **Direct forcing IBM**: enforce boundary velocity directly in momentum equation forcing term each step.
- **Continuous forcing IBM**: spread interface force smoothly to neighboring grid points (kernel-regularized).
- **Feedback forcing IBM**: iterative correction loop to satisfy interface constraints more tightly.
- **Ghost-cell immersed boundary (hybrid)**: uses ghost-state reconstruction around boundary.
- **Direct-forcing FSI variants**: often combine structural coupling and contact logic in the forcing operator.

Different families trade near-wall sharpness for robustness on very fast motion and topological changes.

---

## 🛠️ Practical implementation pattern

1. Represent body with a surface mesh or marker set.
2. Compute closest-point projection / signed distance / intersection data.
3. Build coupling terms between Lagrangian boundary points and Eulerian grid cells.
4. Apply forcing or ghost-state enforcement.
5. Solve momentum + pressure coupling step (SIMPLE/PISO/PIMPLE-style flow loop in many engines).
6. Update geometry and repeat.

For high cadence AI loops, the key speed factor is how expensive steps 2–4 are per candidate shape.

--- 

## ⚖️ Strengths and limitations

### Pros
- High geometric flexibility, especially for moving boundaries.
- Very fast candidate throughput versus full remeshing.
- Naturally compatible with Cartesian/grid-based solvers and level sets.

### Cons
- Geometry-pressure stress transfer can be noisier than body-fitted meshes.
- Accuracy for wall loads depends heavily on delta-spread kernel and forcing strategy.
- Additional tuning is often needed for high-Re separation prediction.

--- 

## 🔬 Use in AI geometry-to-CFD loops

IBM is often best used as an early and mid-stage screen:
- **Early stage**: evaluate many candidate shapes quickly.
- **Mid stage**: compare cut-cell and improved IBM variants for shape ranking.
- **Late stage**: validate finalists with body-fitted or high-order solver setups.

--- 

## 📊 Comparison chart

| Approach | Boundary movement | Surface fidelity | Wall-load fidelity | Mesh cost | Throughput | Best role |
|---|---|---:|---:|---:|---:|---|
| Immersed Boundary (direct forcing) | Very high | Medium | Medium | Very low | Very high | Fast changing topologies |
| Immersed Boundary (continuous forcing) | High | Medium-high | Medium-high | Low | High | Smooth motion with reduced noise |
| Ghost Cell Method | Medium | Medium | Medium | Very low | Very high | Quick screening with simple BCs |
| Cut-cell Method | Medium | High | High | Medium | Medium | Mid-fidelity geometry discrimination |
| Body-fitted mesh | Low | Very high | Very high | High | Low | Final accuracy and certification |
| Overset (chimera) | High | High | High | High | Medium | Complex moving parts, high setup cost |

--- 

## 📚 Source links

- Historical IBM foundation (Peskin): https://doi.org/10.1016/0021-9991(79)90111-0
- Annual review context with classifications and open questions: https://ris.utwente.nl/ws/portalfiles/portal/299026392/annurev_fluid_120720_022129.pdf
- IBM survey and implementation pointers: https://www.mdpi.com/2311-5521/10/5/134
- Open-source framework index for applied IBM research codes: https://ibamr.github.io/
- NASA LAVA Cartesian boundary/IB context: https://www.nas.nasa.gov/LAVA/cart_docs/input_file_cartesian/

--- 

## 🔗 Related notes

- [[Ghost Cell Method]]
- [[Cut-cell Method]]
- [[Voxel CFD]]
- [[SIMPLE]]
- [[PISO]]
- [[CFD Solvers]]
