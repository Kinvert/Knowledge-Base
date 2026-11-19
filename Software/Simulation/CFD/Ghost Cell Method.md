# Ghost Cell Method

The **ghost cell method** is a boundary treatment for structured-grid CFD where virtual cells are added across an interface and assigned values from boundary-aware interpolation/extrapolation. This lets interior differential operators run unchanged while still enforcing wall/edge/interface conditions.

---

## 🧭 Core idea

Ghost cells are extra cells that live just outside the physical fluid domain (or just inside cut regions) and are populated so boundary constraints are satisfied.

Common steps:
- identify boundary normals and face intersections,
- find nearest fluid neighbors,
- reconstruct ghost state from governing BC model,
- feed ghost values back into standard stencils.

This is especially useful in Cartesian-grid or block-structured solvers where remeshing is expensive and frequent geometry updates are needed.

--- 

## 🧱 Typical reconstruction examples

For velocity/temperature-like Dirichlet walls:
- mirrored extrapolation is often used (`u_g = 2u_b - u_f`) or equivalent geometry-aware interpolation;
 
For Neumann-like flux constraints:
- ghost and fluid-side values are reconstructed so the target normal derivative is met.

For pressure correction steps:
- ghost pressure is set to match normal gradient consistency with pressure Poisson correction terms.

These operations are local and usually repeated every nonlinear or time-step update.

---

## 🧰 Implementation workflow

1. Generate/update Cartesian or structured mesh.
2. Flag boundary-adjacent cells as ghost-candidate.
3. Compute intersection geometry and local normals for each candidate.
4. Reconstruct ghost values for all required fields (`u`, `v`, `w`, `p`, `ρ`, scalars).
5. Run existing PDE kernels with modified neighbor values on boundary stencils.

For moving boundaries, the full solver matrix and timestepper usually stay the same; only ghost updates and boundary flags change.

---

## ⚖️ Accuracy and tradeoffs

### Strengths
- Fast updates for geometry-heavy loops.
- No full mesh regeneration for most interface updates.
- Very easy to integrate with existing finite-volume/finite-difference operators.

### Weaknesses
- Geometric stair-stepping can dominate local wall stress and separation behavior.
- Curved/small features are sensitive to stencil choice.
- Accuracy is highly dependent on reconstruction order and interface normal estimation.

In practical AI geometry screening, ghost-cell methods are usually a good "fast first pass" rather than final validation.

--- 

## 🗂️ Comparison chart (for this AI-driven CFD workflow)

| Method | Geometry fidelity | Local conservation | Boundary smoothness | Mesh/pre-processing | Geometry change cost | Typical throughput |
|---|---:|---:|---:|---:|---:|---:|
| Ghost Cell | Medium | Medium | Medium | Low | Very low | Very high |
| Immersed Boundary (direct forcing) | Medium | Medium | Medium | Low | Very low | Very high |
| Immersed Boundary (continuous forcing) | Medium-high | Medium | Medium-high | Low | Very low | High |
| Cut-cell | High | High (if conservative) | High | Medium | Medium | Medium |
| Body-fitted mesh | Very high | High | High | High | High | Low |
| Cartesian with embedded wall function + ghost cells | Medium | Medium | Medium | Low | Low | Very high |

Use recommendation:
- Use **Ghost Cell** for initial ranking across many candidates.
- Promote top candidates to **Cut-cell** or body-fitted methods when wall-layer fidelity becomes dominant.

---

## 📚 Source links

- NASA LAVA Cartesian input guide (ghost-cell / immersed-boundary settings): https://www.nas.nasa.gov/LAVA/cart_docs/input_file_cartesian/
- NASA LAVA Cartesian solver introduction: https://www.nas.nasa.gov/LAVA/cart_docs/introduction_cartesian/
- Annual review of immersed-boundary methods (includes ghost-cell/cut-cell context and limitations): https://ris.utwente.nl/ws/portalfiles/portal/299026392/annurev_fluid_120720_022129.pdf
- Original immersed boundary foundational paper (Peskin): https://doi.org/10.1016/0021-9991(79)90111-0
- Review framing of IBM families in modern CFD/mesoscopy contexts: https://www.mdpi.com/2311-5521/10/5/134

---

## 🔗 Related notes

- [[Cut-cell Method]]
- [[Immersed Boundary Method]]
- [[Voxel CFD]]
- [[Signed Distance Field (SDF) Geometry Pipeline]]
- [[Lattice Boltzmann Method]]
